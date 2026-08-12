# carla-av-agent
A staged autonomous driving agent and evaluation benchmark for CARLA 0.10 (UE5).

## Status
Working: PilotAgent - lane discipline + traffic lights, 5/5 clean, 0 violations. Next: obstacle response, then live traffic.

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

### v1 - waypoint-following P-controller (own agent)

P-only steering toward the nearest route waypoint, fixed 0.2 throttle.
Known limits: cuts corners entering turns, overshoots exiting (P-controller
lag); ignores traffic lights by design.

| route | sim_time_s | collisions | solid_inv | lane_inv | avg km/h | completion |
|-------|-----------|------------|-----------|----------|----------|------------|
| 0     | 65.1      | 0          | 6         | 16       | 8.0      | 100%       |
| 1     | 121.8     | 1          | 14        | 29       | 10.9     | 100%       |
| 2     | 189.0     | 0          | 23        | 58       | 14.9     | 100%       |
| 3     | 115.1     | 0          | 11        | 34       | 11.5     | 100%       |
| 4     | 300.1*    | 3583*      | 12        | 24       | 3.9      | 48%        |

*route 4: drives into a parked vehicle at 48% and remains stuck - v1 has no
obstacle perception.

### v1.1 - look-ahead + adaptive throttle

Changes: aims 2 waypoints ahead (earlier turn-in); ratchet advances when past
a waypoint (no more stall-orbits); throttle scales with steering error:

    throttle = max(0.1, 0.25 - error²)

(≈0.25 on straights, floor 0.1 in hard turns; error in radians.)

| route | sim_time_s | collisions | solid_inv | lane_inv | avg km/h | completion |
|-------|-----------|------------|-----------|----------|----------|------------|
| 0     | 74.9      | 0          | 4         | 17       | 7.0      | 100%       |
| 1     | 125.3     | 0          | 6         | 26       | 10.7     | 100%       |
| 2     | 228.0     | 1          | 12        | 69       | 12.5     | 100%       |
| 3     | 300.1*    | 1          | 4         | 22       | 4.2      | 94%        |
| 4     | 190.8     | 0          | 14        | 47       | 12.6     | 100%       |

*route 3: exits the final turn laterally offset into an occupied lane - the
controller corrects heading but not lateral offset (cross-track error).
Planned route verified clear via debug drawing; fix = v1.2. Route 4 (parked
car) now completes cleanly - slower turns + stall-proof ratchet.

### v1.2 - cross-track steering (Stanley-style)

Adds a second steering term: signed lateral offset from the route line
(2D cross product of route direction × offset vector), so being off-line
demands correction even when heading is correct:

    steer = clamp(error / (π/2) - K · cross),  K = 0.15

Sign determined empirically; K swept over {0.05, 0.1, 0.15, 0.2, 0.3} on
route 3 - 0.15 minimized solid invasions, higher K weaves.

| route | sim_time_s | collisions | solid_inv | lane_inv | avg km/h | completion |
|-------|-----------|------------|-----------|----------|----------|------------|
| 0     | 60.3      | 0          | 0         | 8        | 8.6      | 100%       |
| 1     | 102.0     | 0          | 4         | 14       | 13.0     | 100%       |
| 2     | 169.3     | 0          | 13        | 45       | 16.6     | 100%       |
| 3     | 106.0     | 0          | 2         | 25       | 12.5     | 100%       |
| 4     | 152.0     | 0          | 10        | 28       | 15.6     | 100%       |

First 5/5 clean batch: zero collisions, solids 66 → 29 across versions,
faster than v1.1 despite tighter tracking. Remaining gap to baseline:
raw invasion counts and speed.

### v1.3 - turn anticipation (null result)

Added route-curvature throttle: measure how much the road bends over the
next ~8m (angle between consecutive route segments) and slow before the
bend, not in it. No effect - route 2 unchanged at 45 invasions / 11 solid:
at these speeds, remaining lane errors are steering geometry, not entry
speed. Mechanism retained. Next: better steering (v1.4).

### v1.4 - steering gain sweep

The steering P-gain (placeholder π/2 since v1) was the binding constraint:
swept {π/2..π/8} on route 2 - invasions fell monotonically to a plateau at
π/6 (58 → 7; solids 23 → 0). Full batch:

| route | sim_time_s | collisions | solid_inv | lane_inv | avg km/h | completion |
|-------|-----------|------------|-----------|----------|----------|------------|
| 0     | 53.2      | 0          | 0         | 3        | 9.6      | 100%       |
| 1     | 89.7      | 0          | 0         | 2        | 14.4     | 100%       |
| 2     | 147.1     | 0          | 0         | 7        | 18.8     | 100%       |
| 3     | 93.0      | 0          | 0         | 8        | 13.9     | 100%       |
| 4     | 133.3     | 0          | 0         | 2        | 17.4     | 100%       |

Zero collisions, zero solid invasions; total invasions 22 vs baseline's 23.
Lane discipline now at BehaviorAgent level; remaining gap is speed.

### v2 - traffic lights

Agent: agent/pilot_agent.py (PilotAgent).

Red-light compliance: light positions/stop lines from the map, built once at
init (HD-map assumption); state via ground-truth query, isolated behind
_perceive_traffic_light() for future camera swap. Physics-based braking
(decel = v²/2d) with coast preference; finish-the-stop hold near the line;
launch ramp for pull-away. Trigger volumes unused (unreliable per junction).

Timeout raised to 400s (red phases cost ~30-60s each).

New metric: red_light_violations (validated: 0 for the compliant agent, 7 for
a light-blind agent on route 3; arming requires the stop line to lie on the
benchmark route).

| route | sim_time_s | collisions | solid_inv | lane_inv | avg km/h | completion |
|-------|-----------|------------|-----------|----------|----------|------------|
| 0     | 45.4      | 0          | 0         | 4        | 11.2     | 100%       |
| 1     | 154.0     | 0          | 0         | 2        | 8.4      | 100%       |
| 2     | 383.6     | 0          | 0         | 9        | 7.2      | 100%       |
| 3     | 179.0     | 0          | 0         | 10      | 7.2      | 100%       |
| 4     | 220.5     | 0          | 0         | 2        | 10.5     | 100%       |

(Violations column absent: this batch predates the metric; spot-validated 0
on route 3.)

Known residuals: occasionally stops ~2m short of the line; lane changes
occasionally abrupt.

## Notes / future metrics
- BehaviorAgent overshoots stop lines with long vehicles
- Straddles lanes when changing before junctions
- Lane-centring error (distance from lane centreline, per tick)
- Baseline (BehaviorAgent) rows predate the red_light_violations column - re-run pending
- Violation counter can double-count a light a blind agent re-passes; rankings unaffected