import carla

class MetricsRecorder:
    def __init__(self, world, vehicle, route):
        self.collisions = 0 # collisions with objects (vehicles, trees, poles, etc...)
        self.lane_invasions = 0 # all lane-marking crossings (incl. legal changes).
        self.solid_invasions = 0 # going over solid lines (illegal)
        self.max_speed = 0 # max speed for the run
        self.speed_sum = 0 # for average calculation
        self.speed_ticks = 0 # for average calculation
        self.route = route # judge's route copy, for completion %
        self.route_index = 0 # furthest route waypoint reached (ratchet)
        self.red_light_violations = 0 # going on a red light (illegal)
        self._lights = [(l, wp.transform.location)
                        for l in world.get_actors().filter('traffic.traffic_light*')
                        for wp in l.get_stop_waypoints()] # light stop-line map, built once (lights don't move)
        self._pending_red = None   # (light, stop_loc) we're currently approaching on red

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
        loc = vehicle.get_location()
        self._track_speed(vehicle)
        self._track_completion(loc)
        self._track_red_lights(vehicle, loc)

    def _track_speed(self, vehicle):
        current_speed = 3.6 * vehicle.get_velocity().length() # 3.6 to convert from m/s to km/h
        if current_speed > self.max_speed:
            self.max_speed = current_speed
        # keep count of total speed and ticks to calculate average at the end
        self.speed_sum += current_speed
        self.speed_ticks += 1

    def _track_completion(self, loc):
        # completion ratchet: advance to the furthest of the next few waypoints reached.
        # Windowed scan - the route can pass near itself; progress must be earned in order.
        for i in range(self.route_index + 1, min(self.route_index + 10, len(self.route))):
            if self.route[i][0].transform.location.distance(loc) < 5.0:
                self.route_index = i

    def _track_red_lights(self, vehicle, loc):
        # arm when a red stop line ON OUR ROUTE is near ahead, convict if it
        # ends up behind us while still red, absolve on green.
        # (Route membership, not dot product, decides arming - a forward-vector
        # test is meaningless while turning through a junction.)
        f = vehicle.get_transform().get_forward_vector()
        if self._pending_red is not None:
            light, stop_loc = self._pending_red
            behind = f.x * (stop_loc.x - loc.x) + f.y * (stop_loc.y - loc.y) < 0
            if light.get_state() != carla.TrafficLightState.Red:
                self._pending_red = None                      # turned green: approach absolved
            elif behind and stop_loc.distance(loc) > 1.5:     # 1.5m grace: bumper-over isn't crossing
                self.red_light_violations += 1
                self._pending_red = None
        else:
            window = self.route[max(0, self.route_index - 2):self.route_index + 10]
            for light, stop_loc in self._lights:
                if (light.get_state() == carla.TrafficLightState.Red
                        and stop_loc.distance(loc) < 12.0
                        and any(stop_loc.distance(p[0].transform.location) < 3.0 for p in window)):
                    self._pending_red = (light, stop_loc)
                    break

    def finalize(self) -> dict:
        return {"collisions": self.collisions,
                "lane_invasions": self.lane_invasions,
                "solid_invasions": self.solid_invasions,
                "red_light_violations": self.red_light_violations,
                "max_speed_kmh": round(self.max_speed, 1),
                "avg_speed_kmh": round(self.speed_sum / self.speed_ticks, 1) if self.speed_ticks else 0.0,
                "completion_pct": round(100 * self.route_index / (len(self.route) - 1), 1)}

    def stop(self):
        for sensor in (self.collision_detector, self.lane_invasion_detector):
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()