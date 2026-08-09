# carla-av-agent
A staged autonomous driving agent and evaluation benchmark for CARLA 0.10 (UE5).

## Status
Working: benchmark harness + baseline metrics. Next: PID waypoint-following agent (v1).

## Setup

## Benchmark
5 fixed routes in Town 10 (138–802 m), sync mode @ 0.05s.
Metrics: completion, collisions, lane invasions (solid = illegal), speed.

## Results


## Notes / future metrics
- BehaviorAgent overshoots stop lines with long vehicles
- Straddles lanes when changing before junctions