import carla, math

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
        current_target_location = self.route[self.target_index][0].transform.location
        # 2D steering; z ignored
        dx = current_target_location.x - loc.x
        dy = current_target_location.y - loc.y
        desired = math.atan2(dy, dx)
        f = self.vehicle.get_transform().get_forward_vector()
        heading = math.atan2(f.y, f.x)
        # angles wrap at 180°: without this, a small left correction can read as a huge right turn
        error = (desired - heading + math.pi) % (2 * math.pi) - math.pi 
        steer = max(-1.0, min(1.0, error / (math.pi / 2)))

        return carla.VehicleControl(throttle=0.2, steer=steer)

    def _target_reached(self, loc):
            target_range = 3.0
            return (self.target_index < len(self.route) and # target exists
                self.route[self.target_index][0].transform.location.distance(loc) < target_range) # within target range
    
    def done(self):
        return self.target_index >= len(self.route)