import carla

class MetricsRecorder:
    def __init__(self, world, vehicle):
        self.collisions = 0
        self.lane_invasions = 0
        self.solid_invasions = 0

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


    def stop(self):
        for sensor in (self.collision_detector, self.lane_invasion_detector):
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()