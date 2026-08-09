# carla-av-agent
A staged autonomous driving agent and evaluation benchmark for CARLA 0.10 (UE5).

## Status
Working: benchmark + baseline + v1 waypoint-following agent. Next: v1.1 (look-ahead steering)

## Setup
1. CARLA 0.10 (UE5) built from source; start the server:
   `UnrealEditor.exe CarlaUnreal.uproject -game -windowed -ResX=1280 -ResY=720`
2. `python -m venv .venv` + activate, then `pip install -r requirements.txt`
   (carla wheel comes from your CARLA build: `Build/PythonAPI/dist/`)
3. Run the benchmark from the repo root: `python -m benchmark.runner`

## Benchmark
5 fixed routes in Town 10 (138–802 m), sync mode @ 0.05s.
Metrics: completion, collisions, lane invasions (solid = illegal), speed.

## Results
Baseline: BehaviorAgent ('normal'), empty town, single run per route.

| route | sim_time_s | collisions | solid_inv | lane_inv | avg km/h | completion |
|-------|-----------|------------|-----------|----------|----------|------------|
| 0     | 22.3      | 0          | 0         | 3        | 23.3     | 100%       |
| 1     | 59.0      | 0          | 0         | 2        | 22.0     | 100%       |
| 2     | 143.9     | 0          | 0         | 8        | 19.2     | 100%       |
| 3     | 111.0     | 0          | 0         | 8        | 11.7     | 100%       |
| 4     | 146.4     | 0          | 0         | 2        | 15.9     | 100%       |

### v1 — waypoint-following P-controller (own agent)

P-only steering toward the nearest route waypoint, fixed 0.2 throttle.
Known limits: cuts corners entering turns, overshoots exiting (P-controller
lag); ignores traffic lights by design (M3 scope).

| route | sim_time_s | collisions | solid_inv | lane_inv | avg km/h | completion |
|-------|-----------|------------|-----------|----------|----------|------------|
| 0     | 65.1      | 0          | 6         | 16       | 8.0      | 100%       |
| 1     | 121.8     | 1          | 14        | 29       | 10.9     | 100%       |
| 2     | 189.0     | 0          | 23        | 58       | 14.9     | 100%       |
| 3     | 115.1     | 0          | 11        | 34       | 11.5     | 100%       |
| 4     | 300.1*    | 3583*      | 12        | 24       | 3.9      | 48%        |

*route 4: timed out stuck against an obstacle; collision count = sustained contact.


## Notes / future metrics
- BehaviorAgent overshoots stop lines with long vehicles
- Straddles lanes when changing before junctions
- red-light violation count (v1 crosses reds; not yet measured)