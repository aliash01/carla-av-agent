"""Sampling-based local planner for manoeuvre execution.

Given a reference line (the global route ahead), generates a small fan of
laterally-offset candidate paths - a raised-cosine blend from the current
offset to each target - rejects candidates whose swept rectangle would touch
a vehicle, and scores the survivors: smallest deviation wins (overtaking as
last resort), with a bend penalty and a hysteresis discount so the incumbent
is not dethroned by measurement noise.

Pure 2D geometry, no CARLA imports - unit-testable offline. The consumer
executes the winning blend itself; plan() returns only
(target_offset_m, blend_length_m), or None if every candidate collides.
"""

import math
import bisect

MARGIN = 0.5          # clearance around the ego rectangle, metres. Its ONLY job is to
                      # absorb the gap between the path the planner draws and the one
                      # the van drives - the footprint itself is exact (box_gap), so
                      # this is not shape slop. Derived, not chosen: route 5 measured
                      # with min_clearance() in true-rectangle space, the sweep assumed
                      # 0.2m and the van overlapped by 0.088m, so execution costs
                      # 0.288m against plan; 0.288 + 0.2 leaves 0.2m of REAL clearance.
                      # (An earlier 0.6 came from the same measurement taken in circle
                      # space and paid twice for the 0.41m-per-vehicle bulge.)
                      # Belongs to this van AND this controller - re-measure if the
                      # vehicle, the turning radius or the lateral gains change.
EXEC_LAG = 1.8        # the van reaches a commanded offset after ~1.8x the blend
                      # distance, measured on route 5: a 6.68m blend to 3.5m was not
                      # achieved until ~12m of travel. The blend cannot be drawn any
                      # tighter (6.68m is already full lock for that shift), so the
                      # sweep must instead check the path actually DRIVEN - a blend
                      # stretched by this factor - or it certifies a pass that is
                      # still turning as it goes by. Re-measure with track_err if the
                      # lateral gains or the vehicle change.
LATERAL_RES = 0.05    # declared: resolve the swept path to 5cm laterally. The old
                      # fixed 1m step was an invented distance that silently ate the
                      # margin - a blend moves sideways up to shift*pi/(2L) per metre
                      # (0.82 m/m for a 3.5m shift), so consecutive samples could sit
                      # either side of the closest approach and miss it by more than
                      # MARGIN. Step is now derived from each candidate's own slope.
STEP_CAP = 1.0        # metres; also bounded by the ego half-length below so successive
                      # footprints always overlap and nothing can pass between them
TAIL = 8.0            # keep checking this far beyond the blend, metres
T_BLEND = 1.5         # blend length grows with speed: L = speed * T_BLEND, seconds.
                      # KNOWN UNPRINCIPLED, and a principled replacement was tried and
                      # REVERTED. For a lane-width shift this implies ~7.7 m/s^2 sideways -
                      # above the 4.46 the van ever achieves - so the blend demands
                      # cornering it cannot deliver. Deriving the length from a declared
                      # lateral comfort limit instead (1.6 m/s^2, the p90 of this agent's
                      # own bends on route 2, matching COMFORT_DECEL) roughly doubles the
                      # length above 2 m/s, and made things worse: route 5 clearance
                      # +1.06m -> -0.081m, peak lag 1.70 -> 2.25m, route 7 248s -> 308s.
                      # A longer blend reaches full offset LATER in absolute distance, so
                      # the van is less far over when it draws level - the same failure as
                      # the quintic shape. Lag is the control law's convergence rate, not
                      # the path's difficulty, so no path-side change helps: with the
                      # comfort limit set to 1.6 the van still measured p95 2.13, because
                      # it does not follow the blend it is given. Revisit once the
                      # controller has feedforward and actually tracks.
S_CAP = 25.0          # maximum blend length, metres
W_OFFSET = 1.0        # cost per metre of lateral deviation ("last resort")
W_BEND = 2.0          # cost per (metre of offset change / metre travelled)
HYSTERESIS = 0.5      # score discount for last tick's winner (anti-chatter)


class RefLine:
    """The route ahead as an arc-length-parameterised polyline."""

    def __init__(self, pts):
        self.pts = pts                      # [(x, y)], ~2m apart
        self.cum = [0.0]                    # cumulative arc length per point
        for i in range(1, len(pts)):
            d = math.dist(pts[i - 1], pts[i])
            self.cum.append(self.cum[-1] + d)

    @property
    def length(self):
        return self.cum[-1]

    def at(self, s):
        """(x, y, tangent_angle) at arc length s, clamped to the line."""
        s = max(0.0, min(s, self.cum[-1]))
        i = min(bisect.bisect_right(self.cum, s) - 1, len(self.pts) - 2)
        x1, y1 = self.pts[i]
        x2, y2 = self.pts[i + 1]
        seg = self.cum[i + 1] - self.cum[i]
        t = 0.0 if seg == 0 else (s - self.cum[i]) / seg
        return (x1 + (x2 - x1) * t, y1 + (y2 - y1) * t,
                math.atan2(y2 - y1, x2 - x1))


def shape(u):
    """Raised-cosine profile, the ONE definition of a lateral transition: 0 at
    u=0, 1 at u=1, zero slope at both ends.

    NULL RESULT, do not retry without new evidence: replacing this with a quintic
    smoothstep (zero CURVATURE at both ends as well, so no step demand on the
    steering at u=0) changed nothing. Peak lag 2.03m -> 2.07m, and clearance fell
    0.834m -> 0.480m because the quintic's longer blend reaches full offset later.
    The reason it could not help: at peak error the controller commands only 0.21
    of full lock, so the steering was never the binding constraint - the lag is
    the control law's convergence rate (K = 0.15 corrects over ~5.87m, comparable
    to the whole blend length), and no choice of curve fixes that."""
    u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
    return 0.5 * (1.0 - math.cos(math.pi * u))


def min_blend_length(shift, r_turn):
    """Shortest blend the vehicle can physically follow: this profile's peak
    curvature is shift*pi^2/(2L^2), which must not exceed 1/r_turn ->
    L = pi*sqrt(shift*r_turn/2). An invented 4m floor here passed a 3.5m shift
    the van could not steer (needed a 0.93m radius against a measured 2.58m)
    and drove it into the blocker."""
    return math.pi * math.sqrt(abs(shift) * r_turn / 2)


def blend(from_off, to_off, length, s):
    """Lateral blend from from_off to to_off over length metres of travel;
    flat at to_off beyond. Shape comes from shape() - see there for why."""
    if s >= length:
        return to_off
    return from_off + (to_off - from_off) * shape(s / length)


def plan(ref, cur_offset, speed, ego_box, vehicles, targets, prev_target, r_turn):
    """Choose the best feasible (target_offset, blend_length), or None.

    ref: RefLine ahead of the vehicle (ego assumed near its start).
    cur_offset: current lateral offset from the route line, metres.
    ego_box: (half_length, half_width). vehicles: oriented boxes as dicts
    {x, y, yaw, half_length, half_width}. targets: candidate offsets, metres
    (caller has already gated these for lane legality). prev_target: last
    tick's winner, for hysteresis - None on the first tick. r_turn: the
    vehicle's measured full-lock turning radius - each candidate's blend is
    floored at the shortest length the vehicle can physically steer."""
    best = None
    for target in targets:
        # TWO independent lower bounds, conflated in the old invented S_FLOOR
        # and only one of them replaced when it was removed:
        #   steerability - can the van physically follow this shift (r_turn)
        #   sweep extent - is the path long enough to be a meaningful collision
        #     check at all? A plan shorter than the vehicle is not one, and at
        #     standstill speed * T_BLEND is 0, so a zero shift collapsed length
        #     to 0 and the cost term below divided by it (route 5 crashed at
        #     375s). The van's own length is the honest floor here: you cannot
        #     plan over less road than you occupy.
        floor = max(min_blend_length(target - cur_offset, r_turn), 2 * ego_box[0])
        length = max(floor, min(speed * T_BLEND, S_CAP))
        length = min(length, max(ref.length, 0.5))
        if length < floor:
            continue                       # route ahead too short to steer this shift
        # command the tightest steerable blend, but sweep the SLOWER path the van
        # will really follow: reaching the offset late is what puts it alongside
        # mid-turn. This is what makes it wait (or reverse) for room to be settled
        # BEFORE it arrives, rather than settling as it passes.
        if _collides(ref, cur_offset, target, length * EXEC_LAG, ego_box, vehicles):
            continue
        cost = W_OFFSET * abs(target) + W_BEND * abs(target - cur_offset) / length
        if prev_target is not None and abs(target - prev_target) < 1e-6:
            cost -= HYSTERESIS
        if best is None or cost < best[0]:
            best = (cost, target, length)
    return None if best is None else (best[1], best[2])


def _extent_on(axis, u, v, half_length, half_width):
    """How far a box reaches along `axis`, given its own axes u (length) and
    v (width). The support function of an oriented rectangle."""
    return (abs(axis[0] * u[0] + axis[1] * u[1]) * half_length
            + abs(axis[0] * v[0] + axis[1] * v[1]) * half_width)


def box_gap(ax, ay, ayaw, a_half, bx, by, byaw, b_half, margin=0.0):
    """Separation between two oriented RECTANGLES, metres; negative means overlap.

    Separating-axis test on the four face normals - exact for convex boxes, so
    the answer is the real geometry rather than an approximation of it. This
    replaced a three-circle cover, which bulged 0.41m past each vehicle's side:
    measured on route 5, the sweep demanded 3.58m of lateral room to pass an
    ambulance whose true walls need 2.17m, in a 3.50m lane. It could therefore
    never certify a legal pass, and only ever returned 'feasible' once the
    obstacle fell outside the sweep horizon - the van reversed until the
    ambulance left its field of view rather than until the pass was safe.

    a_half / b_half are (half_length, half_width). margin inflates box A only.
    """
    ca, sa = math.cos(ayaw), math.sin(ayaw)
    cb, sb = math.cos(byaw), math.sin(byaw)
    ua, va = (ca, sa), (-sa, ca)
    ub, vb = (cb, sb), (-sb, cb)
    dx, dy = bx - ax, by - ay
    best = None
    for axis in (ua, va, ub, vb):
        centre_d = abs(dx * axis[0] + dy * axis[1])
        reach = (_extent_on(axis, ua, va, a_half[0], a_half[1]) + margin
                 + _extent_on(axis, ub, vb, b_half[0], b_half[1]))
        gap = centre_d - reach
        if best is None or gap > best:
            best = gap            # widest separating axis; >0 on any axis = clear
    return best


def min_clearance(x, y, yaw, ego_box, vehicles):
    """Smallest gap between the ego rectangle and any vehicle, metres; negative
    means overlap. MARGIN is deliberately excluded - this measures what actually
    happened, so it can be compared against what the sweep assumed. Same
    geometry as the sweep, so the two numbers are directly comparable."""
    if not vehicles:
        return None
    best = None
    for v in vehicles:
        gap = box_gap(x, y, yaw, ego_box,
                      v["x"], v["y"], v["yaw"], (v["half_length"], v["half_width"]))
        if best is None or gap < best:
            best = gap
    return best


def _collides(ref, cur_offset, target, length, ego_box, vehicles):
    """Sweep the ego rectangle along the candidate path; True on any overlap.

    The horizon must outlast the obstacle, not just the blend: if it stops short,
    'no collision found' can mean 'never looked', which is how the old sweep
    passed candidates that drove into a parked ambulance. So extend it to cover
    the furthest vehicle we can actually reach, plus both vehicles' lengths, so
    the van is fully past whatever it is passing before checking stops."""
    if not vehicles:
        return False
    need = 0.0
    for v in vehicles:
        along = math.dist((v["x"], v["y"]), ref.pts[0])
        need = max(need, along + v["half_length"] * 2 + ego_box[0] * 2)
    horizon = min(ref.length, max(length + TAIL, need))
    # step from geometry, not a fixed distance: the blend's steepest lateral rate is
    # d(offset)/ds = shift*pi/(2L), so a step of LATERAL_RES/that keeps the offset
    # change between samples inside the declared resolution
    slope = abs(target - cur_offset) * math.pi / (2 * max(length, 0.5))
    step = min(STEP_CAP, ego_box[0], LATERAL_RES / slope) if slope > 1e-9         else min(STEP_CAP, ego_box[0])
    s = 0.0
    while s <= horizon:
        x, y, yaw = ref.at(s)
        off = blend(cur_offset, target, length, s)
        # same normal convention as the controller: +offset = (-sin, cos) side
        px = x - math.sin(yaw) * off
        py = y + math.cos(yaw) * off
        for v in vehicles:
            if box_gap(px, py, yaw, ego_box,
                       v["x"], v["y"], v["yaw"],
                       (v["half_length"], v["half_width"]), MARGIN) < 0.0:
                return True
        s += step
    return False
