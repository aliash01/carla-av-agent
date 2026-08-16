"""Sampling-based local planner for manoeuvre execution.

Given a reference line (the global route ahead), generates a small fan of
laterally-offset candidate paths - a raised-cosine blend from the current
offset to each target - rejects candidates whose swept footprint would touch
a vehicle, and scores the survivors: smallest deviation wins (overtaking as
last resort), with a bend penalty and a hysteresis discount so the incumbent
is not dethroned by measurement noise.

Pure 2D geometry, no CARLA imports - unit-testable offline. The consumer
executes the winning blend itself; plan() returns only
(target_offset_m, blend_length_m), or None if every candidate collides.
"""

import math
import bisect

MARGIN = 0.2          # extra clearance around the ego footprint, metres
SAMPLE_STEP = 1.0     # collision-check sampling interval along the path, metres
TAIL = 8.0            # keep checking this far beyond the blend, metres
T_BLEND = 1.5         # blend length grows with speed: L = speed * T_BLEND, seconds
S_FLOOR = 4.0         # minimum blend length, metres
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


def blend(from_off, to_off, length, s):
    """Raised-cosine lateral blend: smooth S from from_off to to_off over
    length metres of travel; flat at to_off beyond."""
    if s >= length:
        return to_off
    return from_off + (to_off - from_off) * 0.5 * (1 - math.cos(math.pi * s / length))


def plan(ref, cur_offset, speed, ego_box, vehicles, targets, prev_target):
    """Choose the best feasible (target_offset, blend_length), or None.

    ref: RefLine ahead of the vehicle (ego assumed near its start).
    cur_offset: current lateral offset from the route line, metres.
    ego_box: (half_length, half_width). vehicles: oriented boxes as dicts
    {x, y, yaw, half_length, half_width}. targets: candidate offsets, metres
    (caller has already gated these for lane legality). prev_target: last
    tick's winner, for hysteresis - None on the first tick."""
    length = max(S_FLOOR, min(speed * T_BLEND, S_CAP, ref.length))
    best = None
    for target in targets:
        if _collides(ref, cur_offset, target, length, ego_box, vehicles):
            continue
        cost = W_OFFSET * abs(target) + W_BEND * abs(target - cur_offset) / length
        if prev_target is not None and abs(target - prev_target) < 1e-6:
            cost -= HYSTERESIS
        if best is None or cost < best[0]:
            best = (cost, target, length)
    return None if best is None else (best[1], best[2])


N_CIRCLES = 3         # box-cover resolution: 2 leaves ~1.5m of phantom width
                      # (measured: it vetoed a legal pass), 3 leaves ~0.9m


def _circles(x, y, yaw, half_length, half_width, margin=0.0):
    """Cover an oriented box with N_CIRCLES circles spaced along its length,
    radius reaching the corner of each covered slice. Conservative - bulges
    past the true footprint - but less so as N grows."""
    n = N_CIRCLES
    r = math.hypot(half_length / n, half_width) + margin
    cx, cy = math.cos(yaw), math.sin(yaw)
    step = 2 * half_length / n
    start = -half_length + step / 2
    return tuple((x + cx * (start + i * step), y + cy * (start + i * step))
                 for i in range(n)), r


def _collides(ref, cur_offset, target, length, ego_box, vehicles):
    """Sweep the ego footprint along the candidate path; True on any overlap."""
    if not vehicles:
        return False
    obstacles = [_circles(v["x"], v["y"], v["yaw"],
                          v["half_length"], v["half_width"])
                 for v in vehicles]
    horizon = min(ref.length, length + TAIL)
    s = 0.0
    while s <= horizon:
        x, y, yaw = ref.at(s)
        off = blend(cur_offset, target, length, s)
        # same normal convention as the controller: +offset = (-sin, cos) side
        px = x - math.sin(yaw) * off
        py = y + math.cos(yaw) * off
        ego_centres, r_ego = _circles(px, py, yaw, ego_box[0], ego_box[1], MARGIN)
        for obs_centres, r_obs in obstacles:
            reach = r_ego + r_obs
            for ex, ey in ego_centres:
                for ox, oy in obs_centres:
                    if math.hypot(ex - ox, ey - oy) < reach:
                        return True
        s += SAMPLE_STEP
    return False
