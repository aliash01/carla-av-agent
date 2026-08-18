from enum import Enum, auto

class Manoeuvre(Enum):
    FOLLOWING = auto()
    PREPARING = auto()
    CHANGING = auto()
    RETURNING = auto()
    REVERSING = auto()

import carla, math, os, csv
from agents.navigation.local_planner import RoadOption
from agent.planner import (RefLine, plan as plan_manoeuvre_path, min_blend_length,
                           min_clearance, shape, EXEC_LAG, TAIL)

STEERING_LOOKAHEAD = 2  # waypoints ahead for steering aim
CURVE_LOOKAHEAD = 4     # route bend measured over target_index .. +4 (~8m)
K = 0.15  # cross-track gain
STEER_GAIN = math.pi / 6
MAX_DECEL = 6.0
COMFORT_DECEL = 1.6 # declared preference: firmness of a normal, unhurried stop.
                    # Not invented from nothing - back-derived from four versions of
                    # validated behaviour: the old 20m light gate at benchmark top
                    # speed (7.94 m/s) implies engagement at 7.94^2/(2*20) ~= 1.6.
                    # Sweepable; the physics scales it to any speed.
FOLLOW_GAP = 2.5    # declared preference: bumper-to-bumper metres kept clear at
                    # standstill ("see their tyres" convention); vehicle sizes are
                    # already subtracted, so one convention covers car and truck
HEADWAY_S = 2.0     # declared preference: moving follow gap in TIME (two-second
                    # rule, Highway Code) - gap = HEADWAY_S * speed scales from
                    # car park to motorway where fixed metres cannot
SPEED_KP = 0.15

STOPPED_MS = 0.1            # one definition of "stopped": the stall counter and the
                            # follow latch must agree, or the van can be stopped for
                            # one and moving for the other
STALL_TICK_LIMIT = 200      # 10s at 0.05s ticks: blocker considered parked
PREPARE_TICKS = 20          # 1s of confirmation before committing
LANE_TTC_MIN = 3.0          # seconds; reject if someone arrives sooner

TICK_S = 0.05                   # benchmark fixed delta; blends advance by speed * TICK_S
T_REJOIN_S = 1.8                # rejoin blend length grows with speed: L = speed * T_REJOIN_S
REJOIN_CAP_M = 15.0             # maximum rejoin blend length
R_TURN = 2.58                   # measured full-lock turning radius (calibration run,
                                # 2026-08-16: crawl speed, steer 1.0, circle fit,
                                # spread 0.41m). CARLA's 70-degree lock is generous
                                # against a real Sprinter's ~7m - but it is this
                                # van's truth, measured through the control channel

REVERSE_THROTTLE = 0.6

class PilotAgent:
    def __init__(self, vehicle, destination, grp, trace_path=None):
        self.vehicle = vehicle
        self.route = grp.trace_route(vehicle.get_location(), destination.location) # agent's own route copy (benchmark judges against its own)
        self.target_index = 0
        # light map: (light actor, stop-line location) pairs, built once - lights don't move.
        # Real-AV analogue: HD map supplies light positions/geometry; only the STATE
        # query below gets replaced by camera detection in the perception stage.
        world = vehicle.get_world()
        self.lights = [(l, wp.transform.location)
                       for l in world.get_actors().filter('traffic.traffic_light*')
                       for wp in l.get_stop_waypoints()]
        self.half_length = vehicle.bounding_box.extent.x   # centre-to-bumper, for gap maths
        self.half_width = vehicle.bounding_box.extent.y    # centre-to-side, for the planner footprint
        self._last_obs = None   # previous tick's obstacle distance, for "gap opening?" checks
        self._stopping_for_light = False    # latched once a stop is committed; cleared on green
        self._closing_on_obstacle = False   # latched while regulating the follow gap; cleared when restored
        self.map = world.get_map()
        self.state = Manoeuvre.FOLLOWING
        self.lane_offset = 0.0          # metres to shift our reference line sideways
        self._blocked_ticks = 0
        self._prepare_ticks = 0
        self._overtake_side = None
        self._blocker_box = None        # the blocker's real oriented box
        # planner execution state: the chosen blend and our progress through it
        self._plan_target = None        # winning lateral offset, metres
        self._plan_length = 1.0         # blend length of the winning candidate
        self._plan_phase = 0.0          # 0..1 progress through the blend (by distance)
        self._blend_from = 0.0          # offset the current blend started from
        self._return_from = 0.0         # offset at the start of the rejoin blend
        self._return_length = 1.0
        self._return_phase = 0.0
        self._noprogress_ticks = 0      # commanded to proceed but not moving
        self._made_room = False         # reversed for a blocker: hold the gap
        self._gate_reason = ''          # why the unstick gate last refused
        self._lane_reason = ''          # which lane check last refused
        self._lane_nums = (None,) * 4   # (ahead gap, closing, behind gap, closing)
        self._n_vehicles = 0            # trace only, logged every tick
        self._rear_gap = None
        self._lane_state = ''
        self._clearance = None          # measured gap to the nearest vehicle,
                                        # trace only (see min_clearance)
        self._cross = 0.0             # last tick's achieved offset from the route line,
                                        # for the trace: (cross - lane_offset) is the
                                        # execution error MARGIN has to absorb

        # opt-in control trace: one CSV row per tick, for before/after equivalence diffs
        self._trace = None
        self._trace_tick = 0
        if trace_path is not None:
            os.makedirs(os.path.dirname(trace_path), exist_ok=True)
            self._trace = open(trace_path, 'w', newline='')
            self._trace_writer = csv.writer(self._trace)
            self._trace_writer.writerow(
                ['tick', 'throttle', 'steer', 'brake', 'reverse', 'state', 'lane_offset',
                 'cross', 'track_err', 'speed', 'clearance', 'gate',
                 'ahead_gap', 'ahead_closing', 'behind_gap', 'behind_closing',
                 'n_vehicles', 'rear_gap', 'lane_right'])

    def run_step(self):
        """Thin wrapper: compute the control, optionally trace it, return it unchanged."""
        control = self._run_step()
        if self._trace is not None:
            # repr() keeps full float precision: the diff must catch tiny divergences
            self._trace_writer.writerow(
                [self._trace_tick, repr(control.throttle), repr(control.steer),
                 repr(control.brake), int(control.reverse),
                 self.state.name, repr(self.lane_offset),
                 repr(self._cross), repr(self._cross - self.lane_offset),
                 repr(self.vehicle.get_velocity().length()),
                 '' if self._clearance is None else repr(self._clearance),
                 self._gate_reason,
                 *('' if v is None else repr(v) for v in self._lane_nums),
                 self._n_vehicles,
                 '' if self._rear_gap is None else f'{self._rear_gap:.2f}',
                 self._lane_state])
            self._trace.flush()
        self._trace_tick += 1
        return control

    def _run_step(self):
        """One tick: sense -> assess hazards -> manoeuvre -> track route -> control.
        Hazard assessment runs BEFORE the manoeuvre update, so it reads last
        tick's state; the manoeuvre machine may amend the hazard afterwards."""
        obs = self._sense()
        hazard = self._assess_hazards(obs)

        if self._trace is not None:
            # instrumentation only: how close we ACTUALLY came, in the same
            # geometry the planner's sweep uses, so planned and achieved
            # clearance are directly comparable
            tf = self.vehicle.get_transform()
            self._clearance = min_clearance(
                tf.location.x, tf.location.y, math.radians(tf.rotation.yaw),
                (self.half_length, self.half_width), obs["vehicles"])
            self._n_vehicles = len(obs["vehicles"])
            self._rear_gap = obs["rear"]
            lane = obs["lane_right"]
            if lane is None:
                self._lane_state = 'none'
            else:
                a, b = lane["ahead"], lane["behind"]
                self._lane_state = (
                    ('beside' if lane["beside"] else 'clear')
                    + (f" a{a[0]:.1f}/{a[1]:+.1f}" if a else " a-")
                    + (f" b{b[0]:.1f}/{b[1]:+.1f}" if b else " b-"))

        # reversing has its own control path (rear target, inverted kinematics).
        # Exit is feasibility-driven: back up only until the planner says a
        # swing-out exists (asked each tick), not to a stored clearance. Once
        # the gap exceeds the planner's own sweep horizon, more backing cannot
        # change its answer - that is the cap. Abort if the car behind is at
        # our standstill gap.
        if self.state is Manoeuvre.REVERSING:
            loc = obs["loc"]
            wp_here = self.map.get_waypoint(loc)
            lane_w = wp_here.lane_width if wp_here else 3.5
            feasible = self._swing_out_feasible(obs, lane_w)
            beyond_horizon = (obs["obstacle"] is None
                              or obs["obstacle"] >= min_blend_length(lane_w, R_TURN) + TAIL)
            if (feasible or beyond_horizon
                    or (obs["rear"] is not None and obs["rear"] < FOLLOW_GAP)):
                self.state = Manoeuvre.PREPARING
                self._prepare_ticks = 0
                self._made_room = True    # keep this gap: see _update_manoeuvre
                return carla.VehicleControl(brake=1.0)        # settle before swinging out
            return self._reverse_step(loc)

        self._update_manoeuvre(obs, hazard)

        if self._advance_route(obs["loc"]):
            return carla.VehicleControl(brake=1.0)            # route finished: stop and stay

        steer, heading_error = self._lateral_control(obs)
        throttle, brake = self._longitudinal_control(obs, hazard, heading_error)
        return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)

    def _sense(self):
        """Build this tick's perception snapshot - the ONLY caller of _perceive_*.
        Everything below reads the snapshot, never the sensors: one perception
        pass per tick, and every decision sees the same world."""
        loc = self.vehicle.get_location()
        obstacle = self._perceive_obstacle()
        return {"loc": loc,
                "speed": self.vehicle.get_velocity().length(),
                "light": self._perceive_traffic_light(),
                "obstacle": obstacle[0] if obstacle is not None else None,
                "obstacle_closing": obstacle[1] if obstacle is not None else None,
                "lane_left": self._perceive_lane('left', loc),
                "lane_right": self._perceive_lane('right', loc),
                "rear": self._perceive_rear(loc),
                "vehicles": self._perceive_vehicles(loc),
                "speed_limit": self._get_current_speed_limit()}

    def _assess_hazards(self, obs):
        """This-tick longitudinal judgement: how hard to brake, whether to
        suppress throttle, and whether we are pinned behind an obstacle (held).
        Never touches steering. Reads self.state (last tick's - see _run_step).
        Keeps one memory of its own, _last_obs, for the pulling-away check."""
        hazard = {"hold": False,   # suppress throttle only while a hazard actually demands slowing
                  "brake": 0.0,
                  "held": False}   # pinned behind an obstacle this tick (manoeuvre machine counts these)
        speed = obs["speed"]

        # red light: engage when the decel needed to stop AT the line reaches a
        # comfortable braking level - speed-correct at any speed, unlike the old
        # fixed 20m gate (which encoded ~1.6 m/s2 at town speed and nothing else).
        # Once stopped, hold until the light stops reporting red: a_req is 0 at
        # standstill, so physics alone would release the hold and run the light.
        red_dist = obs["light"]
        if red_dist is None:
            self._stopping_for_light = False      # green or behind us: commitment ends
        else:
            if speed < 0.5 or (red_dist < 2.0 and speed < 2.0):
                hazard["hold"] = True     # stopped at (or crawling onto) the line
                hazard["brake"] = 1.0     # finish the stop and hold until green
            else:
                a_req = speed * speed / (2 * max(red_dist, 0.3))   # decel to stop AT the line
                if a_req >= COMFORT_DECEL:
                    self._stopping_for_light = True    # latch: the stop is committed
                if self._stopping_for_light:
                    # brake keeps tracking a_req even when it dips back under the
                    # engagement level - the dip means the stop is working, not
                    # that it should be cancelled (unlatched, this chattered:
                    # brake episodes doubled)
                    hazard["hold"] = True
                    hazard["brake"] = min(1.0, a_req / MAX_DECEL)

        # vehicle ahead on our route: brake on CLOSING SPEED, not our speed -
        # a_req is the decel that stops the gap shrinking just before the
        # desired gap runs out. A same-speed leader inside the gap reads as
        # closing 0 -> no brake (v^2/2d here treated it as a wall and slammed:
        # 32 rear-end collisions from TM traffic). Desired gap is a time
        # headway with a standstill floor. Latched (bare thresholds chatter,
        # v2.6); released only on evidence about the LEADER - pulling away or
        # gone - never on gap quantities our own braking moves.
        obs_dist = obs["obstacle"]
        if obs_dist is None:
            self._closing_on_obstacle = False
        else:
            closing = obs["obstacle_closing"]
            desired_gap = max(FOLLOW_GAP, HEADWAY_S * speed)
            gap_dist = obs_dist - desired_gap
            opening = self._last_obs is not None and obs_dist > self._last_obs + 0.05   # they're pulling away
            held = (obs_dist - FOLLOW_GAP) <= 0.5 or (speed < 0.5 and obs_dist < FOLLOW_GAP + 2 and not opening)
            hazard["held"] = held
            if held:
                self._closing_on_obstacle = False             # pinned, not regulating
                if self.state is not Manoeuvre.CHANGING:      # don't brake for the car we're passing
                    hazard["hold"] = True
                    hazard["brake"] = 1.0
            else:
                a_req = 0.0
                if closing > 0:
                    a_req = closing * closing / (2 * max(gap_dist, 0.3))
                if a_req >= COMFORT_DECEL:
                    self._closing_on_obstacle = True          # latch: regulate the approach
                if opening:
                    self._closing_on_obstacle = False         # leader pulling away: release
                if speed < STOPPED_MS:
                    # at rest there is no approach left to regulate. Holding the
                    # latch here deadlocks: closing is 0 so brake is 0, but hold
                    # still pins the throttle off, and a van that cannot move can
                    # never observe the leader pulling away - the only other
                    # release. Measured: 4717 ticks (236s) frozen on route 5,
                    # throttle 0, brake 0, beyond the pinned window. Releasing
                    # lets it close up and become properly pinned, which is also
                    # what makes _blocked_ticks run and the unstick gate reachable.
                    self._closing_on_obstacle = False
                if self._closing_on_obstacle and self.state is not Manoeuvre.CHANGING:
                    hazard["hold"] = True
                    hazard["brake"] = max(hazard["brake"], min(1.0, a_req / MAX_DECEL))
        self._last_obs = obs_dist
        return hazard

    def _update_manoeuvre(self, obs, hazard):
        """Manoeuvre state machine: a local deviation from the global route.
        Owns all cross-tick manoeuvre memory (stall counter, prepare counter,
        blocker position) and sets lane_offset. May amend the hazard - PREPARING
        holds the brake on so we don't creep back onto the blocker's bumper."""
        # stall counter: hazard reports this tick's fact, the machine owns the history
        if obs["obstacle"] is None:
            self._blocked_ticks = 0
        elif hazard["held"] or self._made_room:
            # _made_room counts too: holding a gap we reversed for IS being blocked, and
            # without this the counter resets every tick (we are no longer close enough
            # to be "held"), the gate never reopens, and the van holds its room forever -
            # 362s frozen, the same shape of deadlock as the follow latch and the wedge.
            if obs["speed"] < STOPPED_MS:
                self._blocked_ticks += 1                      # accruing stall time
        else:
            self._blocked_ticks = 0

        # wedged mid-manoeuvre: told to proceed, and not proceeding. The blends
        # advance by DISTANCE travelled, so a van that cannot move never finishes
        # the manoeuvre and sits there at full throttle - measured: 149.7s in
        # RETURNING, throttle 1.0, grinding (3008 contacts). Nothing holding us
        # plus no movement is the signature; a legitimate stop (light, leader)
        # sets hazard["hold"] and resets the count. Reuses the blocker patience
        # budget rather than inventing a second one.
        if (self.state in (Manoeuvre.CHANGING, Manoeuvre.RETURNING)
                and not hazard["hold"] and obs["speed"] < STOPPED_MS):
            self._noprogress_ticks += 1
        else:
            self._noprogress_ticks = 0
        if self._noprogress_ticks >= STALL_TICK_LIMIT:
            self._noprogress_ticks = 0
            self.lane_offset = 0.0          # abandon the deviation; the cross-track
            self._plan_target = None        # term recovers the lane from wherever we are
            self._blocker_box = None
            self._blocked_ticks = 0         # full patience period before retrying
            self.state = Manoeuvre.FOLLOWING
            return

        # hold the gap we reversed for. The follow rule would otherwise close back up to
        # FOLLOW_GAP (it releases at standstill so it can pin properly), which throws
        # away the room the reverse just bought: observed reversing, creeping forward,
        # reversing again while waiting for a gap in the next lane. Released when the
        # blocker is gone or the manoeuvre is over.
        if self._made_room:
            if obs["obstacle"] is None or self.state in (Manoeuvre.CHANGING,
                                                         Manoeuvre.RETURNING):
                self._made_room = False
            elif self.state is Manoeuvre.FOLLOWING:
                hazard["hold"] = True
                hazard["brake"] = 1.0

        loc = obs["loc"]
        wp_here = self.map.get_waypoint(loc)
        lane_w = wp_here.lane_width if wp_here else 3.5

        if self.state is Manoeuvre.FOLLOWING:
            self.lane_offset = 0.0
            side = self._unstick_gate(obs) if obs["obstacle"] is not None else None
            if side is not None:
                self._overtake_side = side
                self._prepare_ticks = 0
                # ask the planner whether a swing-out exists from here, rather
                # than predicting with a stored clearance (the late REVERSE_CLEAR_M)
                if self._swing_out_feasible(obs, lane_w):
                    self.state = Manoeuvre.PREPARING
                elif obs["rear"] is None or obs["rear"] > FOLLOW_GAP:
                    self.state = Manoeuvre.REVERSING          # infeasible: back up to make room
                else:
                    # boxed in: no feasible swing-out, and someone on our bumper so we
                    # cannot make room either. Wait. Going to PREPARING here was a
                    # livelock - route 7 spent 16 cycles of 11s each rediscovering the
                    # same infeasibility, until the car behind happened to move.
                    self._gate_reason = 'boxed_in'

        elif self.state is Manoeuvre.PREPARING:
            self.lane_offset = 0.0
            if not self._lane_is_safe(self._overtake_side, obs):
                # record it: this caller, not _unstick_gate, is where route 7's eight
                # refusals actually happened, and the first run could not show it
                self._gate_reason = f'abort:{self._overtake_side}:{self._lane_reason}'
                self.state = Manoeuvre.FOLLOWING              # abort before moving
            else:
                self._prepare_ticks += 1
                hazard["hold"] = True                         # stay put: don't creep back onto the bumper
                hazard["brake"] = 1.0
                if self._prepare_ticks >= PREPARE_TICKS:
                    # keep the blocker's REAL box, not a point estimated from the gap
                    # plus a guessed half-length. Perception already returns true
                    # extents for every vehicle; the old estimate assumed 2.4m and
                    # cut back in early whenever the car was longer than that.
                    self._blocker_box = self._nearest_blocker(obs)
                    self._plan_target = None                  # fresh plan for a fresh manoeuvre
                    self._plan_phase = 0.0
                    self._blend_from = 0.0
                    self.state = Manoeuvre.CHANGING

        elif self.state is Manoeuvre.CHANGING:
            # the planner picks the path; we execute its blend by distance travelled.
            # No feasible candidate this tick: keep executing the last plan - the
            # hazard layer still brakes independently if the world closes in.
            result = self._plan_swing_out(obs, lane_w)
            if result is not None:
                target, length = result
                if self._plan_target is None or abs(target - self._plan_target) > 1e-6:
                    self._blend_from = self.lane_offset       # re-anchor: offset stays continuous
                    self._plan_phase = 0.0
                self._plan_target, self._plan_length = target, length
            if self._plan_target is None:
                self._blocked_ticks = 0                       # full stall period before retrying, not a hot loop
                self.state = Manoeuvre.FOLLOWING              # never had a plan: stand down
            else:
                ds = obs["speed"] * TICK_S
                self._plan_phase = min(1.0, self._plan_phase + ds / max(self._plan_length, 0.5))
                self.lane_offset = (self._blend_from + (self._plan_target - self._blend_from)
                                    * shape(self._plan_phase))
                # past it when its own rear extent is behind our rear bumper, measured
                # from its real box projected onto our heading - no guessed margin
                # RUNTIME PLAN CHECK. The sweep certified the LAG-STRETCHED path,
                # not the commanded one, so that stretched path is the promise the
                # clearance rests on. Falling behind it means the certified clearance
                # no longer bounds us, and driving deeper into the pass is unvalidated.
                # Response is to stop accelerating, which is genuinely corrective here:
                # the lateral controller acts per metre travelled, so less speed buys
                # more sideways movement per metre and the van catches its plan up.
                # Needs no tolerance of its own - EXEC_LAG defines it - and cannot
                # deadlock, because the hold lifts once the van is stopped.
                if obs["speed"] > STOPPED_MS and self._behind_plan():
                    hazard["hold"] = True

                past = False
                if self._blocker_box is not None:
                    b = self._blocker_box
                    fx = self.vehicle.get_transform().get_forward_vector()
                    along = fx.x * (b["x"] - loc.x) + fx.y * (b["y"] - loc.y)
                    d_yaw = b["yaw"] - math.atan2(fx.y, fx.x)
                    reach = (abs(math.cos(d_yaw)) * b["half_length"]
                             + abs(math.sin(d_yaw)) * b["half_width"])
                    past = along < -(self.half_length + reach)
                # and only rejoin once the planner agrees the way back is clear. The
                # swing-out was swept; the rejoin never was, which is where the van
                # cut into the blocker (measured overlap -1.12m, worst AFTER the
                # manoeuvre had "finished").
                if past and self._return_is_clear(obs, lane_w):
                    # rejoin blend: speed-driven length, unlike the old per-tick taper,
                    # so the S covers the same road distance at any speed
                    self._return_from = self.lane_offset
                    self._return_phase = 0.0
                    self._return_length = max(min_blend_length(self.lane_offset, R_TURN),
                                              min(obs["speed"] * T_REJOIN_S, REJOIN_CAP_M))
                    self.state = Manoeuvre.RETURNING

        elif self.state is Manoeuvre.RETURNING:
            ds = obs["speed"] * TICK_S
            self._return_phase = min(1.0, self._return_phase + ds / max(self._return_length, 0.5))
            self.lane_offset = self._return_from * (1.0 - shape(self._return_phase))
            if self._return_phase >= 1.0 or abs(self.lane_offset) < 0.05:
                self.lane_offset = 0.0
                self._blocked_ticks = 0
                self._blocker_box = None
                self._plan_target = None
                self.state = Manoeuvre.FOLLOWING

    def _behind_plan(self):
        """Is the van further behind its commanded offset than the plan allowed for?

        The collision sweep passes a blend stretched by EXEC_LAG, so the promise
        that clearance rests on is the stretched path - being behind THAT is the
        condition that invalidates it, not merely being behind the command (which
        is expected and already accounted for). Uses last tick's achieved offset,
        since _lateral_control runs after this."""
        if self.state is not Manoeuvre.CHANGING or self._plan_target is None:
            return False
        promised = (self._blend_from + (self._plan_target - self._blend_from)
                   * shape(self._plan_phase / EXEC_LAG))
        toward = 1.0 if self._plan_target >= self._blend_from else -1.0
        return (promised - self._cross) * toward > 0.0

    def _nearest_blocker(self, obs):
        """The real oriented box of the vehicle we are stuck behind, from perception
        rather than estimated from a gap. Nearest vehicle ahead of us on our own
        route - route membership is the same filter used everywhere else, and it
        keeps a car on a crossing road from being mistaken for our blocker."""
        loc = obs["loc"]
        f = self.vehicle.get_transform().get_forward_vector()
        upcoming = [p[0].transform.location
                    for p in self.route[self.target_index:self.target_index + 15]]
        best, best_along = None, None
        for v in obs["vehicles"]:
            along = f.x * (v["x"] - loc.x) + f.y * (v["y"] - loc.y)
            if along <= 0.0:
                continue                            # behind us
            if not any(math.hypot(v["x"] - p.x, v["y"] - p.y) < 3.0 for p in upcoming):
                continue                            # not on our route
            if best_along is None or along < best_along:
                best, best_along = v, along
        return best

    def _return_is_clear(self, obs, lane_w):
        """Whether the planner would choose the lane centre from here - i.e. a blend
        back to offset 0 survives its own collision sweep. Reuses plan() rather than
        adding a second check: the offset cost already prefers 0 whenever it fits, so
        the planner picking anything else means the way back is not clear yet."""
        result = self._plan_swing_out(obs, lane_w)
        return result is not None and abs(result[0]) < 1e-6

    def _swing_out_feasible(self, obs, lane_w):
        """Ask the planner whether a collision-free swing-out exists from the
        current position: a surviving candidate with a non-zero offset (offset
        0 is the blocked lane we are stuck in)."""
        result = self._plan_swing_out(obs, lane_w)
        return result is not None and abs(result[0]) > 1e-6

    def _plan_swing_out(self, obs, lane_w):
        """Build planner inputs from the snapshot: route ahead as the reference
        line, candidate targets gated by lane legality, all nearby vehicles as
        obstacles. Returns (target_offset, blend_length) or None."""
        # anchor the reference line at the VAN, not at the next waypoint: after
        # reversing, the waypoint sits up at the blocker, so a waypoint-anchored
        # sweep starts inside the collision and every candidate dies. Project
        # our position onto the route line (undo the commanded offset), then
        # append only route points genuinely ahead of us.
        loc = obs["loc"]
        f = self.vehicle.get_transform().get_forward_vector()
        A = self.route[self.target_index][0].transform.get_forward_vector()
        pts = [(loc.x + A.y * self.lane_offset, loc.y - A.x * self.lane_offset)]
        for p in self.route[self.target_index:self.target_index + 20]:
            l = p[0].transform.location
            if f.x * (l.x - loc.x) + f.y * (l.y - loc.y) > 1.0:
                pts.append((l.x, l.y))
        if len(pts) < 2:
            return None
        # candidates are lane centres, not arbitrary metres: stay on line, or
        # move to a legal neighbouring lane. No straddle options - the offset
        # penalty would make the planner prefer them whenever they fit, and
        # habitual lane-straddling is exactly what the benchmark penalises.
        targets = [0.0]
        if obs["lane_left"] is not None:
            targets.append(-lane_w)
        if obs["lane_right"] is not None:
            targets.append(lane_w)
        return plan_manoeuvre_path(RefLine(pts), self.lane_offset, obs["speed"],
                                   (self.half_length, self.half_width),
                                   obs["vehicles"], targets, self._plan_target, R_TURN)

    def _advance_route(self, loc):
        """Ratchet target_index past reached waypoints; True when the route is done."""
        while self._target_reached(loc):
            self.target_index += 1
            if self.done():
                return True
        return False

    def _offset_ahead(self, d):
        """The commanded lateral offset d metres further along the path, not here.

        The aim point lies ahead of us, so shifting it by the offset we should have
        NOW under-shifts it by however much the blend grows over that distance: the
        van steers for a fraction of the shift, finds itself pointing correctly, and
        levels out. Measured: commanded 0.55m while the blend was already 2.9m over
        at the aim point, so the van drifted right and then went straight into the
        blocker. Same failure as the original unshifted aim point, one layer along.

        Blends advance by distance travelled, so 'd metres ahead' is just d/length
        of extra phase. Returns lane_offset unchanged when not manoeuvring, so
        ordinary lane-following is bit-identical to before."""
        if self.state is Manoeuvre.CHANGING and self._plan_target is not None:
            phase = min(1.0, self._plan_phase + d / max(self._plan_length, 0.5))
            return (self._blend_from + (self._plan_target - self._blend_from)
                    * shape(phase))
        if self.state is Manoeuvre.RETURNING:
            phase = min(1.0, self._return_phase + d / max(self._return_length, 0.5))
            return self._return_from * (1.0 - shape(phase))
        return self.lane_offset

    def _lateral_control(self, obs):
        """Steering: aim at a waypoint ahead, shifted sideways by the offset the
        blend calls for AT THAT POINT (not the offset here - see _offset_ahead),
        so the heading term pulls the same way as the cross-track term instead of
        levelling out early, plus a cross-track correction against the route line
        shifted by our present offset. Returns (steer, heading_error) - the error
        also feeds longitudinal control, which slows for how sharply we're turning."""
        loc = obs["loc"]
        f = self.vehicle.get_transform().get_forward_vector()
        heading = math.atan2(f.y, f.x)

        aim_index = min(self.target_index + STEERING_LOOKAHEAD, len(self.route) - 1)
        wp_aim = self.route[aim_index][0].transform
        A_aim = wp_aim.get_forward_vector()
        aim_offset = self._offset_ahead(loc.distance(wp_aim.location))
        aim_loc = carla.Location(
            x=wp_aim.location.x - A_aim.y * aim_offset,
            y=wp_aim.location.y + A_aim.x * aim_offset,
            z=wp_aim.location.z)
        desired = math.atan2(aim_loc.y - loc.y, aim_loc.x - loc.x)
        # angles wrap at 180 degrees: without this, a small left correction can read as a huge right turn
        error = (desired - heading + math.pi) % (2 * math.pi) - math.pi

        # cross-track: signed distance from the route line (+/- = side),
        # shifted sideways by lane_offset during a lane-change manoeuvre
        wp_tf = self.route[self.target_index][0].transform
        A = wp_tf.get_forward_vector()
        Bx = loc.x - wp_tf.location.x
        By = loc.y - wp_tf.location.y
        cross = A.x * By - A.y * Bx
        self._cross = cross     # recorded for the trace, not used by the controller

        # NOT a candidate for a distance-derived gain: scaling K by the room left in
        # the blend (k = 2*R_TURN/L_remaining^2) was tried and is much worse. As the
        # blend closes, L -> 0 and the gain grows without bound, so any small error
        # saturates: 245 of 288 manoeuvre ticks at full lock, van dragged 4.9m off
        # the route line, route 5 timed out at 12% with 211 contacts. The term stops
        # being a correction and becomes bang-bang. The swept K stands.
        steer = max(-1.0, min(1.0, error / (STEER_GAIN) - K * (cross - self.lane_offset)))
        return steer, error

    def _longitudinal_control(self, obs, hazard, heading_error):
        """Throttle and brake: P-control toward the speed limit, target reduced by
        how sharply we're turning; launch ramp from standstill; the hazard's hold
        overrides throttle (steering stays live throughout)."""
        # how sharply we're turning: live error, or the road's bend over the next ~8m
        worst = max(abs(heading_error), self._route_curvature())

        # target speed: the limit, reduced for how sharply we're turning
        speed = obs["speed"]
        speed_kmh = 3.6 * speed
        target_kmh = obs["speed_limit"] * max(0.35, 1.0 - worst)
        throttle = max(0.0, min(1.0, SPEED_KP * (target_kmh - speed_kmh)))

        # pulling away: hold extra throttle until rolling (~2 m/s), not just at standstill
        if speed < 2.0:
            throttle = max(throttle, 0.5 if speed < 0.5 else 0.35)

        if hazard["hold"]:
            throttle = 0.0

        return throttle, hazard["brake"]
    
    def _lane_is_safe(self, side, obs):
        """True if the lane on `side` is usable and has room to enter. The
        required gaps are the v2.7 following rule applied from both seats:
        ahead we become the follower (our headway at OUR speed), behind we
        make THEM a follower (their headway at THEIR speed, reconstructed
        from closing + ours) - entering must not force anyone into
        tailgating. Standstill floor either way; TTC rejects fast arrivals."""
        info = obs["lane_left"] if side == 'left' else obs["lane_right"]
        self._lane_nums = (None, None, None, None)
        if info is None:
            self._lane_reason = 'absent'      # missing, undrivable or oncoming
            return False
        # keep the (gap, closing) pairs the decision rests on: "the gate refused" is
        # not evidence the gate was RIGHT - that needs the numbers it compared
        a, b = info["ahead"], info["behind"]
        self._lane_nums = (a[0] if a else None, a[1] if a else None,
                           b[0] if b else None, b[1] if b else None)
        if info["beside"]:
            self._lane_reason = 'beside'
            return False
        speed = obs["speed"]
        for key, hit in (("ahead", info["ahead"]), ("behind", info["behind"])):
            if hit is None:
                continue
            gap, closing = hit
            if key == "ahead":
                min_gap = max(FOLLOW_GAP, HEADWAY_S * speed)
            else:
                their_speed = max(0.0, closing + speed)   # behind: closing = theirs - ours
                min_gap = max(FOLLOW_GAP, HEADWAY_S * their_speed)
            if gap < min_gap:
                self._lane_reason = f'{key}_gap'
                return False
            if closing > 0.1 and gap / closing < LANE_TTC_MIN:   # arriving too soon
                self._lane_reason = f'{key}_ttc'
                return False
        self._lane_reason = ''
        return True

    def _unstick_gate(self, obs):
        """Which side to move out to, or None. Records WHY it refused: a van that
        simply waits is indistinguishable in the trace from one that never looked,
        so without this a clean run cannot tell us the gates were exercised at all
        (route 6 taught us that lesson the expensive way)."""
        if self._blocked_ticks < STALL_TICK_LIMIT:
            self._gate_reason = 'waiting'
            return None
        red = obs["light"]
        if red is not None and red <= 25.0:
            self._gate_reason = 'light'
            return None
        upcoming = self.route[self.target_index:self.target_index + 10]
        if any(opt != RoadOption.LANEFOLLOW for _, opt in upcoming):
            self._gate_reason = 'junction'
            return None
        if self._queue_beyond(obs["loc"], obs["obstacle"]):
            self._gate_reason = 'queue'
            return None
        refused = []
        for side in ('left', 'right'):
            if self._lane_is_safe(side, obs):
                self._gate_reason = ''
                return side
            refused.append(f'{side}:{self._lane_reason}')
        self._gate_reason = '|'.join(refused)
        return None

    def _queue_beyond(self, loc, obs_dist):
        """Another stationary vehicle on our route beyond the immediate blocker."""
        upcoming = [p[0].transform.location
                    for p in self.route[self.target_index:self.target_index + 25]]
        for actor in self.vehicle.get_world().get_actors().filter('vehicle.*'):
            if actor.id == self.vehicle.id:
                continue
            other_loc = actor.get_location()
            d = other_loc.distance(loc)
            if d <= obs_dist + self.half_length + actor.bounding_box.extent.x + 2.0 or d > 45.0:
                continue
            if not any(other_loc.distance(p) < 2.0 for p in upcoming):
                continue
            if actor.get_velocity().length() < 0.5:
                return True
        return False

    def _route_curvature(self):
        """How much the road bends ahead (radians): direction of the next
        route segment vs the one after it. Van-independent."""
        i = self.target_index
        j = min(i + CURVE_LOOKAHEAD // 2, len(self.route) - 1)
        k = min(i + CURVE_LOOKAHEAD, len(self.route) - 1)
        if i == j or j == k:
            return 0.0  # route too short ahead to measure
        p1 = self.route[i][0].transform.location
        p2 = self.route[j][0].transform.location
        p3 = self.route[k][0].transform.location
        dir1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
        dir2 = math.atan2(p3.y - p2.y, p3.x - p2.x)
        # same wrap as heading error: bend is the short-way angle difference
        return abs((dir2 - dir1 + math.pi) % (2 * math.pi) - math.pi)

    def _perceive_traffic_light(self):
        """Ground-truth light perception (CARLA state query + map geometry).
        Returns distance to the stop line if a red light governing OUR ROUTE
        lies within 25m ahead, else None. Contract stays fixed when the state
        query is replaced by camera-based detection."""
        loc = self.vehicle.get_location()
        # our upcoming path: next ~15 route points (~30m)
        upcoming = [p[0].transform.location for p in self.route[max(0, self.target_index - 2):self.target_index + 15]]
        best = None
        for light, stop_loc in self.lights:
            d = stop_loc.distance(loc)
            if d > 25.0:
                continue
            if light.get_state() != carla.TrafficLightState.Red:
                continue
            # route membership: the stop line must sit on OUR upcoming path,
            # not a crossing road's (kills phantom mid-junction stops)
            if not any(stop_loc.distance(p) < 3.0 for p in upcoming):
                continue
            if best is None or d < best:
                best = d
        return best

    def _perceive_obstacle(self):
        """Ground-truth obstacle perception (CARLA actor query). Returns
        (bumper-to-bumper distance, closing speed in m/s) for the nearest
        vehicle on our upcoming route within ~30m, else None. closing > 0
        means the gap is shrinking - same convention as _perceive_lane.
        Radar's native measurement pair; contract stays fixed when replaced."""
        loc = self.vehicle.get_location()
        upcoming = [p[0].transform.location
                    for p in self.route[self.target_index:self.target_index + 15]]
        f = self.vehicle.get_transform().get_forward_vector()
        my_v = self.vehicle.get_velocity()
        my_along = my_v.x * f.x + my_v.y * f.y
        best = None
        for actor in self.vehicle.get_world().get_actors().filter('vehicle.*'):
            if actor.id == self.vehicle.id:
                continue                      # we are not our own obstacle
            other_loc = actor.get_location()
            d = other_loc.distance(loc)
            if d > 30.0:
                continue
            # route membership: it blocks OUR path, not a neighbouring lane's
            if not any(other_loc.distance(p) < 2.0 for p in upcoming):
                continue
            # centre-to-centre -> bumper-to-bumper: subtract both half-lengths
            d = max(0.0, d - self.half_length - actor.bounding_box.extent.x)
            if best is None or d < best[0]:
                ov = actor.get_velocity()
                their_along = ov.x * f.x + ov.y * f.y   # their speed along OUR heading
                best = (d, my_along - their_along)
        return best

    def _perceive_vehicles(self, loc):
        """Ground-truth surround perception (CARLA actor query): every vehicle
        within 40m as an oriented box, for the planner's collision check.
        Returns [{x, y, yaw, half_length, half_width}]. Contract stays fixed
        when replaced by lidar/camera detection in the perception stage."""
        out = []
        for actor in self.vehicle.get_world().get_actors().filter('vehicle.*'):
            if actor.id == self.vehicle.id:
                continue
            other_loc = actor.get_location()
            if other_loc.distance(loc) > 40.0:
                continue
            tf = actor.get_transform()
            out.append({"x": other_loc.x, "y": other_loc.y,
                        "yaw": math.radians(tf.rotation.yaw),
                        "half_length": actor.bounding_box.extent.x,
                        "half_width": actor.bounding_box.extent.y})
        return out

    def _get_current_speed_limit(self):
        return self.vehicle.get_speed_limit() # for now use CARLA system for grabbing speed limit, in future may be changed to suit more realistic scenario

    def _perceive_lane(self, side, loc):
        """Ground-truth blind-spot perception (CARLA actor query) for the lane
        on `side` ('left'/'right'). Returns None if that lane is unusable
        (absent, not drivable, or oncoming), else a dict:
            {"beside": True/False,
             "ahead":  (gap_m, closing_ms) or None,
             "behind": (gap_m, closing_ms) or None}
        beside=True is an absolute veto: a vehicle overlaps our length in that
        lane. closing > 0 means the gap is shrinking. Contract stays fixed when
        replaced by radar/lidar in the perception stage."""
        wp = self.map.get_waypoint(loc)
        candidate = wp.get_left_lane() if side == 'left' else wp.get_right_lane()
        if candidate is None or candidate.lane_type != carla.LaneType.Driving:
            return None
        if (candidate.lane_id > 0) != (wp.lane_id > 0):      # sign flip = oncoming
            return None

        # the lane as a stretch: ~20m ahead and ~20m behind our position
        chain = [candidate.transform.location]
        w = candidate
        for _ in range(10):
            nxt = w.next(2.0)
            if not nxt:
                break
            w = nxt[0]
            chain.append(w.transform.location)
        w = candidate
        for _ in range(10):
            prv = w.previous(2.0)
            if not prv:
                break
            w = prv[0]
            chain.append(w.transform.location)

        f = self.vehicle.get_transform().get_forward_vector()
        my_v = self.vehicle.get_velocity()
        my_along = my_v.x * f.x + my_v.y * f.y          # our speed along our heading
        beside = False
        ahead = behind = None

        for actor in self.vehicle.get_world().get_actors().filter('vehicle.*'):
            if actor.id == self.vehicle.id:
                continue
            other_loc = actor.get_location()
            if other_loc.distance(loc) > 40.0:
                continue
            if not any(other_loc.distance(p) < 2.0 for p in chain):
                continue                               # not in that lane

            other_half = actor.bounding_box.extent.x
            along = f.x * (other_loc.x - loc.x) + f.y * (other_loc.y - loc.y)

            # overlapping our length: neither ahead nor behind - beside us
            if abs(along) < self.half_length + other_half:
                beside = True
                continue

            gap = abs(along) - self.half_length - other_half   # bumper-to-bumper along the lane
            ov = actor.get_velocity()
            their_along = ov.x * f.x + ov.y * f.y
            closing = (my_along - their_along) if along > 0 else (their_along - my_along)
            if along > 0:
                if ahead is None or gap < ahead[0]:
                    ahead = (gap, closing)
            else:
                if behind is None or gap < behind[0]:
                    behind = (gap, closing)

        return {"beside": beside, "ahead": ahead, "behind": behind}
    
    def _target_reached(self, loc):
        target_range = 3.0
        # target exists AND (within target range OR closer to next target than current)
        if self.target_index >= len(self.route):
            return False
        d_curr = self.route[self.target_index][0].transform.location.distance(loc)
        if self.target_index + 1 < len(self.route):
            d_next = self.route[self.target_index + 1][0].transform.location.distance(loc)
        else:
            d_next = float('inf')  # no next waypoint: the OR can never fire
        return d_curr < target_range or d_next < d_curr

    def _perceive_rear(self, loc):
        """Ground-truth rear perception in OUR lane. Returns gap in metres to the
        nearest vehicle behind us, or None if clear. See Assumptions."""
        wp = self.map.get_waypoint(loc)
        if wp is None:
            return None
        chain = []
        w = wp
        for _ in range(8):                      # ~16m back
            prv = w.previous(2.0)
            if not prv:
                break
            w = prv[0]
            chain.append(w.transform.location)
        if not chain:
            return None
        f = self.vehicle.get_transform().get_forward_vector()
        best = None
        for actor in self.vehicle.get_world().get_actors().filter('vehicle.*'):
            if actor.id == self.vehicle.id:
                continue
            other_loc = actor.get_location()
            if other_loc.distance(loc) > 30.0:
                continue
            if not any(other_loc.distance(p) < 2.0 for p in chain):
                continue
            along = f.x * (other_loc.x - loc.x) + f.y * (other_loc.y - loc.y)
            if along >= 0:
                continue                        # not behind us
            gap = abs(along) - self.half_length - actor.bounding_box.extent.x
            if best is None or gap < best:
                best = max(0.0, gap)
        return best

    def _reverse_step(self, loc):
        """Path-following in reverse: aim at a waypoint behind us along our lane,
        measure error against our reversed heading, and negate the steer (turning
        the wheels one way swings the tail the other)."""
        wp = self.map.get_waypoint(loc)
        target = None
        if wp is not None:
            w = wp
            for _ in range(3):                  # ~6m back
                prv = w.previous(2.0)
                if not prv:
                    break
                w = prv[0]
            target = w.transform.location
        if target is None:
            return carla.VehicleControl(throttle=REVERSE_THROTTLE, reverse=True)

        f = self.vehicle.get_transform().get_forward_vector()
        rev_heading = math.atan2(-f.y, -f.x)                 # we travel backwards
        desired = math.atan2(target.y - loc.y, target.x - loc.x)
        error = (desired - rev_heading + math.pi) % (2 * math.pi) - math.pi
        steer = max(-1.0, min(1.0, -(error / STEER_GAIN)))   # negated: reverse kinematics
        return carla.VehicleControl(throttle=REVERSE_THROTTLE, steer=steer, reverse=True)

    def done(self):
        return self.target_index >= len(self.route)