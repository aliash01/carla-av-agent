import carla

class MetricsRecorder:
    def __init__(self, world, vehicle, route):
        self.collisions = 0
        self.lane_invasions = 0
        self.solid_invasions = 0
        self.max_speed = 0
        self.speed_sum = 0
        self.speed_ticks = 0
        self.route = route
        self.route_index = 0

        bp_lib = world.get_blueprint_library()
        collision_detector_bp = bp_lib.find('sensor.other.collision')
        lane_invasion_detector_bp = bp_lib.find('sensor.other.lane_invasion')

        self.collision_detector = world.spawn_actor(collision_detector_bp, carla.Transform(), attach_to=vehicle)
        self.lane_invasion_detector = world.spawn_actor(lane_invasion_detector_bp, carla.Transform(), attach_to=vehicle)

        self.collision_detector.listen(self._on_collision)
        self.lane_invasion_detector.listen(self._on_lane_invasion)

    def _on_collision(self, event):
        self.collisions += 1

    def _on_lane_invasion(self, event):
        self.lane_invasions += 1
        if any('Solid' in str(m.type) for m in event.crossed_lane_markings):
            self.solid_invasions += 1

    def on_tick(self, vehicle):
        current_speed = 3.6 * vehicle.get_velocity().length() # 3.6 to convert from m/s to km/h

        if current_speed > self.max_speed:
            self.max_speed = current_speed

        # keep count of total speed and tick to calculate average
        self.speed_sum += current_speed 
        self.speed_ticks += 1

        loc = vehicle.get_location()
        for i in range(self.route_index + 1, min(self.route_index + 10, len(self.route))):
            if self.route[i][0].transform.location.distance(loc) < 5.0:
                self.route_index = i

    def finalize(self) -> dict:
        return {"collisions": self.collisions,
                "lane_invasions": self.lane_invasions,
                "solid_invasions": self.solid_invasions,
                "max_speed_kmh": round(self.max_speed, 1),
                "avg_speed_kmh": round(self.speed_sum / self.speed_ticks, 1) if self.speed_ticks else 0.0,
                "completion_pct": round(100 * self.route_index / (len(self.route) - 1), 1)}

    def stop(self):
        for sensor in (self.collision_detector, self.lane_invasion_detector):
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()