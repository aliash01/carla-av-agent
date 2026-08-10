import sys
sys.path.insert(0, r'C:\CarlaUE5\PythonAPI\carla')
import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from collections import Counter

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

town_map = world.get_map()
spawnpoint = town_map.get_spawn_points()[0] 

grp = GlobalRoutePlanner(town_map, sampling_resolution=2.0)

for i, dest in enumerate(town_map.get_spawn_points()):
    distance_from_spawn = spawnpoint.location.distance(dest.location)
    if distance_from_spawn > 20.0:
        route = grp.trace_route(spawnpoint.location, dest.location)
        print(i, distance_from_spawn, len(route) * 2, Counter(opt.name for _, opt in route))