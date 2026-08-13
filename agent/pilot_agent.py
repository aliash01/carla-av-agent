import carla, math

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
                print(f"HELD at red_dist {red_dist:.2f}  (bumper ≈ {red_dist - self.half_length:.2f} from line)")
            else:
                a_req = speed * speed / (2 * max(red_dist, 0.3))   # decel to stop AT the line
                if a_req >= COAST_DECEL:
                    brake = min(1.0, a_req / MAX_DECEL)

        # vehicle ahead on our route: physics-brake to a gap SHORT of it
        obs_dist = self._perceive_obstacle()
        if obs_dist is not None:
            gap_dist = obs_dist - FOLLOW_GAP
            opening = self._last_obs is not None and obs_dist > self._last_obs + 0.05   # they're pulling away
            if gap_dist <= 0.5 or (speed < 0.5 and obs_dist < FOLLOW_GAP + 2 and not opening):
                hazard_hold = True
                brake = 1.0                                        # inside the gap, or held while blocked
            else:
                a_req = speed * speed / (2 * max(gap_dist, 0.3))
                if a_req >= COAST_DECEL:                           # only hold when we actually need to slow
                    hazard_hold = True
                    brake = max(brake, min(1.0, a_req / MAX_DECEL))
        self._last_obs = obs_dist

        while (self._target_reached(loc)):
            self.target_index += 1
            if self.done():
                return carla.VehicleControl(brake=1.0)

        f = self.vehicle.get_transform().get_forward_vector()
        heading = math.atan2(f.y, f.x)

        # steering: aim at a waypoint ahead to anticipate turns
        aim_index = min(self.target_index + STEERING_LOOKAHEAD, len(self.route) - 1)
        error = self._heading_error(loc, aim_index, heading)

        # cross-track: signed distance from the route line (+/- = side)
        wp_tf = self.route[self.target_index][0].transform
        A = wp_tf.get_forward_vector()
        Bx = loc.x - wp_tf.location.x
        By = loc.y - wp_tf.location.y
        cross = A.x * By - A.y * Bx

        steer = max(-1.0, min(1.0, error / (STEER_GAIN) - K * cross))

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

    def done(self):
        return self.target_index >= len(self.route)