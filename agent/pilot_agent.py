from enum import Enum, auto

class Manoeuvre(Enum):
    FOLLOWING = auto()
    PREPARING = auto()
    CHANGING = auto()
    RETURNING = auto()
    REVERSING = auto()

import carla, math
from agents.navigation.local_planner import RoadOption

STEERING_LOOKAHEAD = 2  # waypoints ahead for steering aim
CURVE_LOOKAHEAD = 4     # route bend measured over target_index .. +4 (~8m)
MINTHROTTLE = 0.1
MAXTHROTTLE = 0.25
K = 0.15  # cross-track gain
STEER_GAIN = math.pi / 6
COAST_DECEL = 0.8 
MAX_DECEL = 6.0
FOLLOW_GAP = 2.5  # metres kept clear behind a stopped obstacle
SPEED_KP = 0.15

STALL_TICK_LIMIT = 200      # 10s at 0.05s ticks: blocker considered parked
PREPARE_TICKS = 20          # 1s of confirmation before committing
LANE_CLEAR_AHEAD = 8.0      # metres needed ahead in the target lane
LANE_CLEAR_BEHIND = 10.0    # metres needed behind
LANE_TTC_MIN = 3.0          # seconds; reject if someone arrives sooner

REVERSE_MAX_M = 8.0             # never back up further than this
REVERSE_THROTTLE = 0.6
REVERSE_CLEAR_M = 6.0           # clearance from the blocker that makes a swing-out feasible
REAR_CLEAR_M = 10.0              # required clear space behind us before reversing

class PilotAgent:
    def __init__(self, vehicle, destination, grp):
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
        self._last_obs = None   # previous tick's obstacle distance, for "gap opening?" checks
        self.map = world.get_map()
        self.state = Manoeuvre.FOLLOWING
        self.lane_offset = 0.0          # metres to shift our reference line sideways
        self._blocked_ticks = 0
        self._prepare_ticks = 0
        self._overtake_side = None
        self._blocker_ahead_index = None
        self._blocker_loc = None
        self._reverse_start = None      # where reversing began, to cap distance backed
        

    def run_step(self):
        loc = self.vehicle.get_location()
        speed = self.vehicle.get_velocity().length()

        # -- longitudinal decisions (never touch steering) --
        hazard_hold = False   # suppress throttle only while a hazard actually demands slowing
        brake = 0.0

        # red light: physics-brake to the stop line, hold until green
        red_dist = self._perceive_traffic_light()
        if red_dist is not None and red_dist <= 20:
            hazard_hold = True
            if speed < 0.5 or (red_dist < 2.0 and speed < 2.0):
                brake = 1.0        # stopped, or nearly there: finish the stop and hold until green
            else:
                a_req = speed * speed / (2 * max(red_dist, 0.3))   # decel to stop AT the line
                if a_req >= COAST_DECEL:
                    brake = min(1.0, a_req / MAX_DECEL)

        # vehicle ahead on our route: physics-brake to a gap SHORT of it
        obs_dist = self._perceive_obstacle()
        if obs_dist is not None:
            gap_dist = obs_dist - FOLLOW_GAP
            opening = self._last_obs is not None and obs_dist > self._last_obs + 0.05   # they're pulling away
            held = gap_dist <= 0.5 or (speed < 0.5 and obs_dist < FOLLOW_GAP + 2 and not opening)
            if held:
                if self.state is not Manoeuvre.CHANGING:      # don't brake for the car we're passing
                    hazard_hold = True
                    brake = 1.0
                if speed < 0.1:
                    self._blocked_ticks += 1                  # accruing stall time
            else:
                a_req = speed * speed / (2 * max(gap_dist, 0.3))
                if a_req >= COAST_DECEL and self.state is not Manoeuvre.CHANGING:
                    hazard_hold = True
                    brake = max(brake, min(1.0, a_req / MAX_DECEL))
                self._blocked_ticks = 0
        else:
            self._blocked_ticks = 0
        self._last_obs = obs_dist

        # -- reversing has its own control path (rear target, inverted kinematics) --
        if self.state is Manoeuvre.REVERSING:
            backed = self._reverse_start.distance(loc) if self._reverse_start is not None else 0.0
            rear = self._perceive_rear(loc)
            enough_room = obs_dist is None or obs_dist >= REVERSE_CLEAR_M
            if enough_room or backed >= REVERSE_MAX_M or (rear is not None and rear < 3.0):
                self.state = Manoeuvre.PREPARING
                self._prepare_ticks = 0
                return carla.VehicleControl(brake=1.0)        # settle before swinging out
            return self._reverse_step(loc)

        # -- manoeuvre state machine: a local deviation from the global route --
        wp_here = self.map.get_waypoint(loc)
        lane_w = wp_here.lane_width if wp_here else 3.5

        if self.state is Manoeuvre.FOLLOWING:
            self.lane_offset = 0.0
            side = self._unstick_gate(loc, obs_dist) if obs_dist is not None else None
            if side is not None:
                self._overtake_side = side
                self._prepare_ticks = 0
                if obs_dist < REVERSE_CLEAR_M:
                    rear = self._perceive_rear(loc)
                    if rear is None or rear >= REAR_CLEAR_M:
                        self._reverse_start = loc
                        self.state = Manoeuvre.REVERSING      # too close to swing out: back up first
                    else:
                        self.state = Manoeuvre.PREPARING      # nowhere to reverse: try anyway
                else:
                    self.state = Manoeuvre.PREPARING

        elif self.state is Manoeuvre.PREPARING:
            self.lane_offset = 0.0
            if not self._lane_is_safe(self._overtake_side, loc):
                self.state = Manoeuvre.FOLLOWING              # abort before moving
            else:
                self._prepare_ticks += 1
                hazard_hold = True                            # stay put: don't creep back onto the bumper
                brake = 1.0
                if self._prepare_ticks >= PREPARE_TICKS:
                    # remember where the blocker is, so we know when we're past it
                    f0 = self.vehicle.get_transform().get_forward_vector()
                    ahead_m = (obs_dist or 0.0) + self.half_length + 2.4
                    self._blocker_loc = carla.Location(
                        x=loc.x + f0.x * ahead_m, y=loc.y + f0.y * ahead_m, z=loc.z)
                    self.state = Manoeuvre.CHANGING

        elif self.state is Manoeuvre.CHANGING:
            self.lane_offset = -lane_w if self._overtake_side == 'left' else lane_w
            past = False
            if self._blocker_loc is not None:
                fx = self.vehicle.get_transform().get_forward_vector()
                along = fx.x * (self._blocker_loc.x - loc.x) + fx.y * (self._blocker_loc.y - loc.y)
                past = along < -(self.half_length + 3.0)      # blocker's centre well behind our nose
            if past:
                self.state = Manoeuvre.RETURNING

        elif self.state is Manoeuvre.RETURNING:
            self.lane_offset *= 0.9                           # taper back onto the route line
            if abs(self.lane_offset) < 0.2:
                self.lane_offset = 0.0
                self._blocked_ticks = 0
                self._blocker_loc = None
                self.state = Manoeuvre.FOLLOWING

        while (self._target_reached(loc)):
            self.target_index += 1
            if self.done():
                return carla.VehicleControl(brake=1.0)

        f = self.vehicle.get_transform().get_forward_vector()
        heading = math.atan2(f.y, f.x)

        # steering: aim at a waypoint ahead, shifted sideways during a manoeuvre so the
        # heading term pulls the same way as the cross-track term instead of fighting it
        aim_index = min(self.target_index + STEERING_LOOKAHEAD, len(self.route) - 1)
        wp_aim = self.route[aim_index][0].transform
        A_aim = wp_aim.get_forward_vector()
        aim_loc = carla.Location(
            x=wp_aim.location.x - A_aim.y * self.lane_offset,
            y=wp_aim.location.y + A_aim.x * self.lane_offset,
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

        steer = max(-1.0, min(1.0, error / (STEER_GAIN) - K * (cross - self.lane_offset)))

        # how sharply we're turning: live error, or the road's bend over the next ~8m
        worst = max(abs(error), self._route_curvature())

        # target speed: the limit, reduced for how sharply we're turning
        speed_kmh = 3.6 * speed
        target_kmh = self._get_current_speed_limit() * max(0.35, 1.0 - worst)
        throttle = max(0.0, min(1.0, SPEED_KP * (target_kmh - speed_kmh)))

        # pulling away: hold extra throttle until rolling (~2 m/s), not just at standstill
        if speed < 2.0:
            throttle = max(throttle, 0.5 if speed < 0.5 else 0.35)

        # hazard overrides throttle (steering stays live throughout)
        if hazard_hold:
            throttle = 0.0

        return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)
    
    def _lane_is_safe(self, side, loc):
        """True if the lane on `side` is usable and has room to enter."""
        info = self._perceive_lane(side, loc)
        if info is None or info["beside"]:
            return False
        for key, min_gap in (("ahead", LANE_CLEAR_AHEAD), ("behind", LANE_CLEAR_BEHIND)):
            hit = info[key]
            if hit is None:
                continue
            gap, closing = hit
            if gap < min_gap:
                return False
            if closing > 0.1 and gap / closing < LANE_TTC_MIN:   # arriving too soon
                return False
        return True

    def _unstick_gate(self, loc, obs_dist):
        if self._blocked_ticks < STALL_TICK_LIMIT:
            return None
        red = self._perceive_traffic_light()
        if red is not None and red <= 25.0:
            return None
        upcoming = self.route[self.target_index:self.target_index + 10]
        if any(opt != RoadOption.LANEFOLLOW for _, opt in upcoming):
            return None
        if self._queue_beyond(loc, obs_dist):
            return None
        for side in ('left', 'right'):
            if self._lane_is_safe(side, loc):
                return side
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
        """Ground-truth obstacle perception (CARLA actor query). Returns the
        BUMPER-TO-BUMPER distance to the nearest vehicle on our upcoming route
        within ~30m, else None. Contract stays fixed when replaced by
        radar/lidar detection in the perception stage."""
        loc = self.vehicle.get_location()
        upcoming = [p[0].transform.location
                    for p in self.route[self.target_index:self.target_index + 15]]
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
            if best is None or d < best:
                best = d
        return best

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
    
    def _heading_error(self, loc, index, heading):
        """Signed angle between vehicle heading and direction to route[index]."""
        target = self.route[index][0].transform.location
        # 2D steering; z ignored
        desired = math.atan2(target.y - loc.y, target.x - loc.x)
        # angles wrap at 180°: without this, a small left correction can read as a huge right turn
        return (desired - heading + math.pi) % (2 * math.pi) - math.pi

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