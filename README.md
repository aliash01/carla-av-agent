# carla-av-agent
A staged autonomous driving agent and evaluation benchmark for CARLA 0.10 (UE5).

## Status
Working: benchmark + baseline + PilotAgent v2.2 - lane discipline, traffic lights, obstacle response, live traffic, speed-limit control. Next: surround perception (adjacent/rear awareness), then overtaking.

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

### v2.1 - obstacle response

Perception: _perceive_obstacle() - ground-truth actor query, route-membership
filter (a vehicle counts only if it sits on our upcoming waypoints), returns
bumper-to-bumper distance via bounding-box half-lengths. Same frozen-contract
pattern as light perception; radar/lidar swap later.

Response: physics braking (decel = v²/2d) to a FOLLOW_GAP (5m) short of the
obstacle; held stop while blocked. No overtaking by design - future work.

Scenario mechanism: routes may declare "parked_obstacle": <waypoint index>;
the runner spawns a stationary vehicle there. Added as route 5 (route 2's
geometry + parked car at waypoint 50) so routes 0-4 stay comparable.

Route 5 result: timeout at 400s (correct - no overtaking), 0 collisions,
completion 11.5% = pinned exactly at the obstacle.

Validation note: an earlier "pass" was a false positive - a red light was
doing the stopping, masking a centre-vs-bumper distance bug that a green-phase
rerun exposed. Light phase is a hidden variable in any scenario near a
junction.

### v2.2 - live traffic and speed control

Traffic scenarios: route definitions may declare
`"traffic": {"vehicles": N, "seed": S}`; the runner seeds the Traffic Manager,
spawns N autopiloted vehicles on fixed spawn points (the ego's excluded) and
ticks once to settle registration before the run starts. Added as route 6
(route 2's geometry + 20 vehicles) so routes 0-5 stay comparable.

Motivation for speed control: with a ~15 km/h average against traffic driving
~21+, NPCs repeatedly misjudged the ego - lane-changing into it and rear-ending
it. A vehicle far below traffic speed is itself a hazard, so speed was raised to
track the posted limit rather than a hand-tuned throttle ceiling. (Fault was not
proven per-incident: a later seed change removed one shunt entirely.)

Speed control: proportional control toward the posted speed limit, the target
reduced by how sharply we are turning:

    target = speed_limit x max(0.35, 1 - worst)
    throttle = clamp(SPEED_KP x (target - speed))

Following: FOLLOW_GAP reduced to 2.5m bumper-to-bumper; the hazard hold now
applies only when a hazard actually demands braking, so a car ahead that needs
no slowing no longer suppresses throttle.

| route | traffic      | sim_time_s | collisions | solid_inv | lane_inv | avg km/h | violations | completion |
|-------|--------------|-----------|------------|-----------|----------|----------|------------|------------|
| 2     | none         | 188.0     | 0          | 0         | 8        | 14.7     | 0          | 100%       |
| 6     | 20 vehicles  | 187.6     | 0          | 0         | 8        | 14.7     | 0          | 100%       |

Traffic costs the agent nothing measurable: identical times and lane numbers
with 20 vehicles present. Max speed rose 21.2 -> 28.7 km/h and route 2's time
fell 383s -> 188s versus v2.1's fixed-ceiling throttle, with lane discipline
unchanged.

Findings: an NPC rear-ending the ego proved seed-specific - a different seed ran
clean - so traffic scenarios must report their seed to be interpretable. The
stop position at lights comes from CARLA's stop waypoint, typically ~1-2m
upstream of the painted line; left as-is rather than offset by hand, since
camera-based light detection will re-derive it. The collision counter inflates
during sustained contact (one shunt read as 140 events); counting episodes
rather than contacts is future work.

### v2.3 - surround perception (blind-spot awareness)

Capability only: the agent can now inspect the lane beside it. No behaviour
change - nothing calls this yet; the consumer is the overtaking chapter.

Perception tier: ground truth - see Assumptions.

_perceive_lane(side, loc) returns None if the lane on that side is unusable
(absent, not Driving, or oncoming - detected by lane_id sign flip, since
lane_type alone would permit steering into oncoming traffic), otherwise:

    {"beside": bool,                    # a vehicle overlaps our length: veto
     "ahead":  (gap_m, closing_ms),     # nearest ahead in that lane, or None
     "behind": (gap_m, closing_ms)}     # nearest behind in that lane, or None

Gaps are bumper-to-bumper measured along our heading; closing > 0 means the gap
is shrinking, for both ahead and behind, so consumers can apply one threshold.
Lane membership uses a chain of that lane's waypoints ~20m fore and aft
(next()/previous()), the same membership pattern as route and light perception.

Ground-truth actor query for now, behind the usual frozen contract - the
real-world analogue is radar (production blind-spot monitors use it; closing
speed is measured natively).

Validated by trace inspection on route 6 rather than by benchmark numbers,
since no behaviour changed: a vehicle overtaking in the right lane read as
behind/closing, then beside (veto), then ahead with an opening gap, while a
second vehicle further up read as ahead throughout. Oncoming lanes correctly
returned None.

Rules drafted for the overtaking chapter: beside = wait and adjust speed, not
abandon; the ahead margin must allow stopping if they brake hard; the behind
margin must allow reaching speed before they arrive (time-to-collision, ~2-3s);
and a lane change is a re-evaluated state with an abort path, not a committed
action.

## Assumptions

The agent currently assumes solved perception and localisation, and says so
explicitly rather than implying otherwise:

- **Road geometry** comes from the map (OpenDRIVE via CARLA): lanes, waypoints,
  junctions, stop-line positions, speed limits. Real-world analogue: an HD map
  plus localisation (Waymo-style). Map-light stacks (Tesla-style) derive this
  from cameras instead - a genuine open architecture argument, not a settled one.
- **Other vehicles** (positions, extents, velocities) come from ground-truth
  actor queries. Real-world analogue: radar for closing speed and blind spots,
  camera/lidar for detection and classification.
- **Traffic light state** comes from a ground-truth query. Real-world analogue:
  camera detection of the light head, with geometry still supplied by the map.

Every one of these sits behind a `_perceive_*` method with a fixed contract
(distance-or-None, or a small dict), so the perception stage swaps the method
body without touching any decision logic. That staging is deliberate: it keeps
attribution clean while the driving logic is being built - a failure is either
the logic or the sensing, never ambiguously both.

What is *not* assumed away: vehicle dynamics (the benchmark vehicle is a
long-wheelbase Sprinter, deliberately unforgiving), control (every steering and
throttle command is computed here), and judgement (the benchmark scores from its
own ground truth, independently of what the agent believed).

## Notes / future metrics
- BehaviorAgent overshoots stop lines with long vehicles
- Straddles lanes when changing before junctions
- Lane-centring error (distance from lane centreline, per tick)
- Baseline (BehaviorAgent) rows predate the red_light_violations column - re-run pending
- Violation counter can double-count a light a blind agent re-passes; rankings unaffected