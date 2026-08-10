import carla, math

LOOKAHEAD = 2 # number of waypoints to look ahead to anticipate movement
MINTHROTTLE = 0.1
MAXTHROTTLE = 0.25
K = 0.15

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

        # calculate steer needed to waypoint using vector between current location and waypoint
        aim_index = min(self.target_index + LOOKAHEAD, len(self.route) - 1)

        current_target_location = self.route[aim_index][0].transform.location
        # 2D steering; z ignored
        dx = current_target_location.x - loc.x
        dy = current_target_location.y - loc.y
        desired = math.atan2(dy, dx)
        f = self.vehicle.get_transform().get_forward_vector()
        heading = math.atan2(f.y, f.x)
        # angles wrap at 180°: without this, a small left correction can read as a huge right turn
        error = (desired - heading + math.pi) % (2 * math.pi) - math.pi

        # cross-track: signed distance from the route line (+/- = side)
        wp_tf = self.route[self.target_index][0].transform
        A = wp_tf.get_forward_vector()
        Bx = loc.x - wp_tf.location.x
        By = loc.y - wp_tf.location.y
        cross = A.x * By - A.y * Bx
         
        steer = max(-1.0, min(1.0, error / (math.pi / 2) - K * cross))

        throttle = max(MINTHROTTLE, MAXTHROTTLE - error**2)

        return carla.VehicleControl(throttle=throttle, steer=steer)

    def _target_reached(self, loc):
        target_range = 3.0
        # target exists AND (within target range OR closer to next target than current)
        if self.target_index >= len(self.route):
            return False
        d_curr = self.route[self.target_index][0].transform.location.distance(loc)
        if self.target_index + 1 < len(self.route):
            d_next = self.route[self.target_index + 1][0].transform.location.distance(loc)
        else:
            d_next = float('inf')   # no next waypoint: the OR can never fire
        return d_curr < target_range or d_next < d_curr
    
    def done(self):
        return self.target_index >= len(self.route)