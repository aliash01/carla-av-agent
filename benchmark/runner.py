import sys, os, csv
sys.path.insert(0, r'C:\CarlaUE5\PythonAPI\carla')

from typing import Callable
import carla
from benchmark.metrics import MetricsRecorder
from benchmark.routes import ROUTES
from agent.v1_agent import V1Agent
from agents.navigation.global_route_planner import GlobalRoutePlanner

FIXED_DELTA_S = 0.05


def run_route(client: carla.Client,
              world: carla.World,
              agent_factory: Callable,
              route_def: dict,
              grp: GlobalRoutePlanner,
              timeout_s: float = 400.0) -> dict:
    """
    Run a single benchmark route in synchronous mode.

    agent_factory: callable (vehicle, destination) -> agent with run_step()/done().
    route_def: entry from ROUTES, e.g. {"id": 0, "start": 0, "end": 9}.

    Restores world settings and destroys spawned actors on exit,
    including on error. Returns a result dict.
    """
    spawnpoints = world.get_map().get_spawn_points() # get all spawnpoints from current map
    trafficManager = client.get_trafficmanager()

    start = spawnpoints[route_def["start"]] # get start point of selected route
    end = spawnpoints[route_def["end"]] # end of route (target spawnpoint)

    # sweep leftovers
    for a in world.get_actors().filter('vehicle.*'):
        a.destroy()
    for a in world.get_actors().filter('sensor.*'):
        a.destroy()

    vehicle = None 
    agent = None
    ticks = 0
    timed_out = False
    error = None
    recorder = None

    try:
        bp_lib = world.get_blueprint_library() 
        vehicle_bp = bp_lib.filter('sprinter')[0] # fixed vehicle for comparable results across agents
        vehicle = world.spawn_actor(vehicle_bp, start) # spawns vehicle actor at start point
        # judge's own copy of the route, for completion % (the agent traces its own)
        route = grp.trace_route(start.location, end.location)
        recorder = MetricsRecorder(world, vehicle, route)
        # set synchronous mode on for benchmark
        settings = world.get_settings() 
        settings.synchronous_mode = True 
        settings.fixed_delta_seconds = FIXED_DELTA_S
        world.apply_settings(settings)
        trafficManager.set_synchronous_mode(True)

        agent = agent_factory(vehicle, end) 

        while not agent.done():
            world.tick()
            recorder.on_tick(vehicle)
            vehicle.apply_control(agent.run_step())
            ticks += 1
            if ticks * FIXED_DELTA_S > timeout_s:
                timed_out = True
                break

    except Exception as e:
        error = str(e) # agent failure is a benchmark result, not a batch-stopper

    finally:
        if recorder is not None: 
            recorder.stop()
        if vehicle is not None and vehicle.is_alive:
            vehicle.destroy()

        # return to async restoring settings
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        trafficManager.set_synchronous_mode(False)

    metrics = recorder.finalize() if recorder is not None else {}

    return {"route_id": route_def["id"],
            "done": agent.done() if agent is not None and error is None else False,
            "timeout": timed_out,
            "sim_time_s": ticks * FIXED_DELTA_S,
            "error": error,
            **metrics}

def run_batch(client, world, agent_factory, out_name: str) -> list:
    """Run all ROUTES with the given agent, write results/<out_name>.csv."""
    results = []
    grp = GlobalRoutePlanner(world.get_map(), 2.0)
    for route_def in ROUTES:
        result = run_route(client, world, agent_factory, route_def, grp)
        print(result)
        results.append(result)

    os.makedirs('results', exist_ok=True)
    out_path = os.path.join('results', f'{out_name}.csv')
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)
    print(f'wrote {out_path}')
    return results

if __name__ == '__main__':
    from agents.navigation.behavior_agent import BehaviorAgent

    def v1_factory(vehicle, destination):
        grp = GlobalRoutePlanner(world.get_map(), 2.0)
        return V1Agent(vehicle, destination, grp)

    def behavior_agent_factory(vehicle, destination):
        agent = BehaviorAgent(vehicle, behavior='normal')
        agent.set_destination(destination.location)
        return agent

    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    world = client.get_world()

    grp = GlobalRoutePlanner(world.get_map(), 2.0)
    print(run_batch(client, world, v1_factory, 'v2_lights'))
    #print(run_route(client, world, v1_factory, ROUTES[2], grp))