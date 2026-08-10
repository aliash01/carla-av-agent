import carla, math

STEERING_LOOKAHEAD = 2  # waypoints ahead for steering aim
CURVE_LOOKAHEAD = 4     # route bend measured over target_index .. +4 (~8m)
MINTHROTTLE = 0.1
MAXTHROTTLE = 0.25
K = 0.15  # cross-track gain
STEER_GAIN = math.pi / 6

class V1Agent:
    def __init__(self, vehicle, destination, grp):
        self.vehicle = vehicle
        self.route = grp.trace_route(vehicle.get_location(), destination.location) # agent's own route copy (benchmark judges against its own)
        self.target_index = 0

    def run_step(self):
        loc = self.vehicle.get_location()
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

        # slow before turns: throttle obeys the worst of current error
        # and how much the road itself bends over the next ~8m
        worst = max(abs(error), self._route_curvature())
        throttle = max(MINTHROTTLE, MAXTHROTTLE - worst**2)

        return carla.VehicleControl(throttle=throttle, steer=steer)

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