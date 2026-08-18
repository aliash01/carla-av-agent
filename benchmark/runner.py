import sys, os, csv, random
sys.path.insert(0, r'C:\CarlaUE5\PythonAPI\carla')

from typing import Callable
import carla
from benchmark.metrics import MetricsRecorder
from benchmark.routes import ROUTES
from agent.pilot_agent import PilotAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner

FIXED_DELTA_S = 0.05


def _neighbour_lane(wp):
    """First drivable SAME-DIRECTION neighbour of wp, or None. Map-derived rather than
    a hand-picked spawn point, so an adjacent-lane scenario works on any route with a
    neighbour. Same direction only: the agent never overtakes into oncoming traffic, so
    an oncoming neighbour is not a lane it could ever choose."""
    for cand in (wp.get_right_lane(), wp.get_left_lane()):
        if (cand is not None and cand.lane_type == carla.LaneType.Driving
                and cand.lane_id * wp.lane_id > 0):
            return cand
    return None


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
    obstacle = None
    traffic_vehicles = []

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

        # scenario: parked vehicle on the route (optional per route definition)
        if "parked_obstacle" in route_def:
            obs_tf = route[min(route_def["parked_obstacle"], len(route) - 10)][0].transform
            obs_tf.location.z += 0.5                     # drop onto the road, don't clip it
            obstacle_bp = bp_lib.filter('vehicle.*')[1]
            obstacle = world.try_spawn_actor(obstacle_bp, obs_tf)

        # scenario: TM-driven traffic (optional per route definition)
        if "traffic" in route_def:
            cfg = route_def["traffic"]
            trafficManager.set_random_device_seed(cfg["seed"])   # pin the TM's dice
            traffic_bps = bp_lib.filter('vehicle.*')
            for i, sp in enumerate(spawnpoints):
                if len(traffic_vehicles) >= cfg["vehicles"]:
                    break
                if i == route_def["start"]:
                    continue                                     # ego's spot stays free
                tv = world.try_spawn_actor(traffic_bps[i % len(traffic_bps)], sp)
                if tv is not None:
                    tv.set_autopilot(True, trafficManager.get_port())
                    traffic_vehicles.append(tv)
            world.tick()   # let spawns + TM registration settle before the run starts

        # scenario: an open-ended stream into the neighbouring lane, released from our
        # own start at random intervals, so the ego has to find its own gap rather than
        # wait out a fixed queue. Seeded, so the arrival pattern is reproducible.
        adj_cfg, adj_tf, adj_bps, adj_rng = None, None, None, None
        adj_next, adj_done, adj_spawned = 0, False, 0
        adj_live = []                   # active stream cars, oldest first
        if "adjacent_traffic" in route_def:
            adj_cfg = route_def["adjacent_traffic"]
            trafficManager.set_random_device_seed(adj_cfg["seed"])
            adj_rng = random.Random(adj_cfg["seed"])
            neighbour = _neighbour_lane(route[0][0])
            if neighbour is None:
                raise RuntimeError("adjacent_traffic: no same-direction neighbour lane "
                                   "at the route start")
            adj_tf = neighbour.transform
            adj_tf.location.z += 0.5                      # drop on, don't clip
            adj_bps = bp_lib.filter('vehicle.*')

        # lights keep cycling between runs, so each run would otherwise start at an
        # arbitrary phase - measured 34s of variance on route 2 from this alone.
        # Reset to phase zero so runs are reproducible and agents comparable.
        world.reset_all_traffic_lights()

        agent = agent_factory(vehicle, end)

        while not agent.done():
            world.tick()
            # stop the stream once the ego is fully past the blocker: it found its gap,
            # and the rest of the route should run normally. Judged geometrically from
            # the two footprints - the benchmark takes any agent through a factory, so
            # it must not read the agent's own state to decide this.
            if adj_cfg is not None and not adj_done and obstacle is not None:
                ol, el = obstacle.get_location(), vehicle.get_location()
                fo = obstacle.get_transform().get_forward_vector()
                if (fo.x * (el.x - ol.x) + fo.y * (el.y - ol.y)
                        > vehicle.bounding_box.extent.x + obstacle.bounding_box.extent.x):
                    adj_done = True
                    # the test window is over: give them their lane changes back. Locked,
                    # a car whose lane ends has nowhere to go - one sat pinned alongside
                    # and scraped us for a second where the right lane runs out.
                    for tv in adj_live:
                        if tv.is_alive:
                            trafficManager.auto_lane_change(tv, True)
            if (adj_cfg is not None and not adj_done and ticks >= adj_next):
                # rolling window: cull the longest-serving car to make room, but only
                # once it is outside our perception range. Culling one still in range
                # would delete a leader mid-decision and release the follow latch -
                # inventing agent behaviour rather than testing it.
                if len(adj_live) >= adj_cfg["active"]:
                    el = vehicle.get_location()
                    for i, tv in enumerate(adj_live):
                        if not tv.is_alive or tv.get_location().distance(el) > 40.0:
                            if tv.is_alive:
                                tv.set_autopilot(False)
                                tv.destroy()
                            adj_live.pop(i)
                            break
                if len(adj_live) < adj_cfg["active"]:
                    # spawn RELATIVE TO THE EGO, a set distance back in the neighbouring
                    # lane. A fixed point at the route start was rate-limited to a
                    # trickle - it stays blocked while a car accelerates from rest, so
                    # most attempts failed and only 2 cars ever appeared. A point that
                    # moves with us is always clear, and the cars always arrive from
                    # behind at speed, which is the case worth testing.
                    ewp = world.get_map().get_waypoint(vehicle.get_location())
                    back = ewp.previous(adj_cfg["behind_m"]) if ewp is not None else []
                    lane = _neighbour_lane(back[0]) if back else None
                    if lane is None:
                        adj_next = ticks + 20        # no neighbour lane back there
                        tv = None
                    else:
                        adj_tf = lane.transform
                        adj_tf.location.z += 0.5
                        tv = world.try_spawn_actor(
                            adj_bps[adj_spawned % len(adj_bps)], adj_tf)
                    if tv is None:
                        adj_next = max(adj_next, ticks + 10)   # busy: retry in 0.5s
                    else:
                        tv.set_autopilot(True, trafficManager.get_port())
                        # keep them in the lane we put them in: by default TM
                        # lane-changes them, and they drifted into OUR lane and queued
                        # behind the ego, which blocks reversing and stops the
                        # manoeuvre being tested at all
                        trafficManager.auto_lane_change(tv, False)
                        adj_live.append(tv)
                        traffic_vehicles.append(tv)
                        adj_spawned += 1
                        # start frequent and thin out, so early cars deny the gap and
                        # later ones eventually grant one - the ego has to spot it
                        lo, hi = adj_cfg["interval_s"]
                        grow = adj_cfg["growth"] ** (adj_spawned - 1)
                        adj_next = ticks + max(
                            1, round(adj_rng.uniform(lo, hi) * grow / FIXED_DELTA_S))
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
        if obstacle is not None and obstacle.is_alive:
            obstacle.destroy()
        for tv in traffic_vehicles:
            if tv.is_alive:
                tv.set_autopilot(False)
                tv.destroy()

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
            # a scenario that spawned fewer cars than intended is not the scenario we
            # designed - surface the count rather than reading a clean run
            "traffic_spawned": adj_spawned if "adjacent_traffic" in route_def
                               else len(traffic_vehicles),
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

    def pilot_factory(vehicle, destination):
        grp = GlobalRoutePlanner(world.get_map(), 2.0)
        return PilotAgent(vehicle, destination, grp)

    def traced_pilot_factory(name):
        """Same agent, writing a per-tick control+tracking trace. Used to measure
        execution error (commanded vs achieved lane offset), not for scoring."""
        def factory(vehicle, destination):
            grp = GlobalRoutePlanner(world.get_map(), 2.0)
            return PilotAgent(vehicle, destination, grp,
                              trace_path=f'results/traces/{name}.csv')
        return factory

    def behavior_agent_factory(vehicle, destination):
        agent = BehaviorAgent(vehicle, behavior='normal')
        agent.set_destination(destination.location)
        return agent

    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    world = client.get_world()

    grp = GlobalRoutePlanner(world.get_map(), 2.0)
    #print(run_batch(client, world, pilot_factory, 'v2_lights'))
    print(run_route(client, world, traced_pilot_factory('route5_demagic'),
                    ROUTES[5], grp))