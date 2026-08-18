# carla-av-agent
A staged autonomous driving agent and evaluation benchmark for CARLA 0.10 (UE5).

## Status
Working: benchmark + baseline + PilotAgent v2.9 - lane discipline, traffic lights, closing-speed following, live traffic, speed-limit control, unsticking past blockers with a planner-certified manoeuvre. Next: camera perception, or feedforward steering.

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

### v2.4 - unsticking (reverse and go around)

Route 5's parked blocker now completes: the agent waits, reverses to make room,
swings into the adjacent lane, passes, and rejoins the route.

Manoeuvre state machine (FOLLOWING -> REVERSING -> PREPARING -> CHANGING ->
RETURNING), re-evaluated every tick with abort paths, rather than a committed
action. Gate order before any manoeuvre: stalled >10s behind a stationary
vehicle -> no red light governing -> upcoming route is LANEFOLLOW (no
manoeuvring in or near junctions) -> nothing else queued beyond the blocker ->
an adjacent lane that exists, is same-direction, and is clear with adequate
ahead/behind gaps and time-to-collision.

The manoeuvre is a *local deviation*, not a replan: the global route stays the
reference and a lane_offset shifts the reference line sideways, so aborting is
free and the benchmark's completion metric keeps working. Rejoining tapers the
offset back to zero.

Reversing has its own control path: it aims at a waypoint behind along the lane,
measures error against the reversed heading, and negates the steer command
(turning the wheels one way swings the tail the other). Rear perception in our
own lane was added for it.

| route | scenario        | sim_time_s | collisions | solid_inv | lane_inv | avg km/h | completion |
|-------|-----------------|-----------|------------|-----------|----------|----------|------------|
| 5     | parked blocker  | 193.6     | 0          | 0         | 15       | 14.4     | 100%       |

Debugging note worth recording: the manoeuvre appeared to be a geometry problem
(clearance, reverse distance) and resisted every geometric fix. It was a
*controller* problem - the heading term aimed at the unshifted route waypoint and
overpowered the cross-track offset, so the van steered almost straight into the
blocker. Shifting the aim point sideways by the same offset made both terms agree
and the manoeuvre worked immediately. Deciding correctly and executing correctly
are separate failures.

Limits: same-direction lanes only (overtaking into oncoming traffic needs
legality and oncoming-gap logic - future work); triggers on stationary blockers,
not slow-rolling ones; reversing follows the lane but is not path-planned.

### Benchmark reproducibility and control traces

No agent behaviour change - measurement infrastructure ahead of a controller
refactor.

Light-phase reset: lights cycle freely between runs, so each run started at
whatever phase the world clock happened to be at - measured 34s of sim-time
variance on empty route 2 from this alone (one extra red). The runner now
calls reset_all_traffic_lights() before the agent is built. Repeat runs of
routes 2, 5 and 6 became metric-identical, all landing on exactly 187.65s:
red lights act as resynchronisation points, absorbing time lost mid-route
and releasing at a fixed phase.

Control traces: PilotAgent takes an optional trace_path and writes one CSV
row per tick (throttle, steer, brake, reverse, state, lane_offset). Purpose:
before/after equivalence diffs for refactors, replacing ad-hoc prints.

Finding: CARLA physics is not bit-deterministic. Same-code trace pairs
diverge from tick 2 (~1e-7 in steer), amplified to full-scale single-tick
differences at decision thresholds, and to visibly divergent steering with
traffic present (route 6). Behaviour is deterministic regardless: identical
tick counts, manoeuvre state sequences and metrics across all repeat runs.
Refactor equivalence is therefore judged on state sequences, tick counts and
metrics, with float diffs bounded by the measured same-code noise band
(~0.15 on empty routes) - not byte equality.

### Refactor: sense - assess - manoeuvre - control

No behaviour change. run_step, grown to ~150 lines across four versions, split
along the pipeline: _sense builds a per-tick perception snapshot and is the
only caller of _perceive_* (one perception pass per tick; every decision reads
the same world - the property a camera swap needs); _assess_hazards turns the
snapshot into a longitudinal judgement {hold, brake, held}; _update_manoeuvre
owns the state machine and all cross-tick manoeuvre memory, and may amend the
hazard (PREPARING holds the brake); _lateral_control and _longitudinal_control
are the unchanged maths. Made explicit in signatures rather than left implicit:
hazard assessment reads last tick's manoeuvre state, and the machine writes
back into the hazard - the two are coupled in both directions.

Verified against pre-refactor traces on routes 2, 5 and 6: identical tick
counts, identical state sequences, float diffs within the measured same-code
noise band, identical metrics.

### v2.5 - sampling-based local planner (manoeuvre execution)

The revisit of the deleted local-planner branch, with the two pieces it died
without: candidates are legality-gated through _perceive_lane at generation,
and the planner only runs once the v2.4 gate has already committed to a
manoeuvre - patience stays the state machine's job. The planner (agent/
planner.py, pure 2D geometry, no CARLA imports, testable offline) generates
raised-cosine offset blends along the route line, sweeps an N-circle footprint
cover along each, discards colliders, and scores survivors: offset penalty
(smallest deviation wins), bend penalty, hysteresis discount against
tick-to-tick chatter. Candidates are lane centres, not arbitrary metres -
straddle options were tried and removed, since the offset penalty makes the
planner prefer them whenever they fit, and habitual lane-straddling is exactly
what the benchmark penalises. Output stays a lane_offset; the controller is
untouched. New perception contract: _perceive_vehicles(), nearby vehicles as
oriented boxes. The rejoin is now a speed-scaled S (distance-based) rather
than the old per-tick 0.9 taper - a time-based decay that covered more road
at higher speed by accident.

Route 5: completes identically (187.65s, 0 collisions, 0 solids), lane
invasions unchanged (15-17 across runs, both versions), max steering during
the manoeuvre 0.61 vs 1.00 - v2.4 saturated the wheel the tick it committed;
the planned blend never does. Routes 2 and 6: trace-equivalent to baseline
(planner provably inactive off-manoeuvre).

Found by measurement, not foresight: a 2-circle footprint cover carries ~1.5m
of phantom width against 1.6m of true clearance and vetoed a legal pass -
raised to 3 circles (~0.9m). The 6m reverse clearance that v2.4 "passed" with
was never actually checked; a collision-checked swing-out needs 8m
(REVERSE_CLEAR_M raised, validated offline). If no candidate survives, the
agent stands down and waits a full stall period - the frozen-robot failure is
accepted and recorded rather than papered over with margin hacks.

Limits: the rejoin blend is not collision-checked (the queue check and
past-blocker check protect it today); obstacles are swept where they are -
no motion prediction, correct for parked blockers only; blend execution is
open-loop (no feedback if the controller lags the profile); candidates come
from immediate neighbour lanes only. Planner constants (blend durations,
scoring weights, margins) are declared first guesses - the next unit converts
scale-assuming values (fixed metres and durations) to generalisable
references: lane widths, footprints, time headways and v²/2a physics.

### v2.6 - de-magic I: physics-derived light response

First of a series converting scale assumptions (fixed metres, guessed rates)
into generalisable references: map geometry, vehicle calibration, physics.

Coast-down calibration: the Sprinter's engine-off deceleration was measured
(throttle to top speed, release, record per-tick decel, samples gated on
near-zero steering): 0.10-0.26 m/s2 across 13-23 km/h, rising with speed.
COAST_DECEL = 0.8 was therefore wrong by 4-8x - the agent's "coast
preference" was largely fictional; the brake was doing almost all the work.
True coast-to-stop from 25 km/h would need ~120m, so coast-first is bounded
by a comfort-braking tier.

Light response: the 20m engagement gate is gone. The van now engages when
the deceleration needed to stop at the line (v^2/2d) reaches COMFORT_DECEL -
correct at any speed, where the fixed gate assumed town speeds. COMFORT_DECEL
= 1.6 m/s2 is a declared, sweepable preference, back-derived from the old
gate's own behaviour at benchmark top speed (7.94^2/(2*20) ~= 1.6), so
benchmark behaviour is preserved by construction. Stopped-at-line holding is
its own explicit condition (a_req = 0 at standstill would otherwise release
the hold and run the light), cleared when the light stops reporting red.

Finding - thresholds chatter without memory: the bare physics threshold
doubled brake episodes (route 2: 23 -> 55); braking lowers a_req below the
threshold, releasing the brake, which raises a_req again. Same failure the
planner's hysteresis prevents laterally. Fixed by latching the committed
stop until the light clears: 6 episodes - one committed stop per red -
against 23 under the old gate, which had its own mild flicker. Route 6's
episode count is unchanged (164): its braking is obstacle-following, the
unconverted block, which is the next conversion.

Metrics unchanged throughout: 0 violations, 0 collisions, 100%, sim times
within a tick of 187.65s. Residual: _perceive_traffic_light's 25m range
covers comfort engagement only up to ~32 km/h - a sensor-model conversion,
pending.

### v2.7 - de-magic II: closing-speed following

The obstacle block converted: FOLLOW_GAP now applies only at standstill (a
declared "see their tyres" convention, 2.5m bumper-to-bumper); the moving
follow gap is a time headway (HEADWAY_S = 2.0s, the Highway Code two-second
rule) - time scales from car park to motorway where fixed metres cannot.
Engagement and latching follow v2.6's pattern.

Two failures on the way, both caught by the benchmark before commit:

1. Absolute-speed physics in traffic: carrying v^2/2d over from the light
   stop treats every leader as a wall. A same-speed TM vehicle inside the
   desired gap read as an emergency -> full brake in flowing traffic -> 32
   rear-end collisions from following NPCs (route 6). Fixed by braking on
   CLOSING SPEED: _perceive_obstacle now returns (gap, closing) - radar's
   native pair, same convention as _perceive_lane - and a_req =
   closing^2/2d stops the gap shrinking rather than the van. A stationary
   blocker gives closing = our speed, so route 5 and light stops are
   unchanged by construction.

2. Latch release via the gap re-oscillated (route 5: 58 episodes): the
   desired gap moves with our own speed, so braking "restored" it, released
   the latch, sped up, re-latched. Release now keys only on evidence about
   the leader - pulling away or gone - never on quantities our own braking
   moves. The v2.6 light latch had this property by accident; now it is a
   stated rule.

Brake episodes: route 6 133 -> 9, route 5 33 -> 7, route 2 unchanged.
Metrics clean throughout (0 collisions, 0 solids, 0 violations, 100%,
sim times unchanged at 187.65s). Average speed did not rise - route timing
is light-dominated; the gain is comfort and legibility, not pace.

Remaining scale assumptions, next in line: lane-entry gates (8m/10m),
reverse clearances, blend lengths, route-membership tolerances, perception
ranges (25/30/40m).

### v2.8 - de-magic III: lane-entry gates from both seats

LANE_CLEAR_AHEAD (8m) and LANE_CLEAR_BEHIND (10m) - invented in v2.3 -
replaced by the v2.7 following rule applied symmetrically: entering a lane
makes us a follower of the car ahead (our headway at OUR speed, standstill
floor) and makes the car behind a follower of us (their headway at THEIR
speed, reconstructed as closing + ours from the existing lane contract).
Entering must not force anyone into tailgating. The TTC >= 3s check was
already time-based and stays.

The new gates are more permissive at low speed (floor 2.5m vs 8/10m) and
stricter against fast approachers (a 36 km/h closer needs 20m + TTC, where
10m used to pass). Nothing on the current routes exercises the gates
(route 5's adjacent lane is empty), so validation is ten synthetic
gate tests (floors, both headway directions, TTC, beside-veto) plus
benchmark invisibility: brake profiles identical to v2.7, metrics clean.

Incidental finding: first equivalence comparison across a server restart -
metrics reproduce within the same noise band as same-session repeats
(route 2: 187.60s/9 inv vs 187.65s/8), so the light-phase reset makes
reproducibility a property of the benchmark, not of a server session.

### v2.9 - de-magic IV: the planner checks the path actually driven

Last scale assumptions gone (S_FLOOR 4m, REVERSE_CLEAR_M 8m, REJOIN_FLOOR_M 5m,
REVERSE_MAX_M 8m, REAR_CLEAR_M 10m): blend lengths derive from a measured full-lock
turning radius (R_TURN = 2.58m), and reversing continues until the planner says a
swing-out exists. Route 5 fell to 52% and a timeout - the constants had masked that
nothing checked whether the van could follow the path the planner certified.

The footprint was the headline. Three circles per vehicle, each reaching the corner of
the third it covered, bulged 0.41m past every flank: passing route 5's blocker (a 6.36m
ambulance, not the guessed 2.4m half-length) needed 3.58m of lateral room in a 3.50m
lane. A legal pass could never be certified, so "feasible" only became true once the
ambulance left the sweep horizon - the van reversed until it stopped *seeing* it. Exact
oriented rectangles (separating-axis test) replaced them: exact, and 1.4-1.7x faster with
no square roots. True walls need 2.17m; clearance is now +1.06m where it used to overlap.

Four more checks that assumed instead of measuring:
- "am I past it?" used that guessed half-length - now the real box on our heading
- the rejoin was never swept, only the swing-out - now gated through plan()
- 1m sampling could straddle the closest approach (a blend moves 0.82m sideways per metre,
  more than MARGIN) - the step now derives from each candidate's rate to a declared 5cm
- the horizon stopped at blend + 8m, so "clear" could mean "never looked"

Two deadlocks, each hidden by the other:
- the v2.7 follow latch released only on evidence about the leader, so a van stopped
  beyond FOLLOW_GAP+0.5 behind a *stationary* one was pinned forever: closing 0 so brake
  0, hold suppressing throttle, and a van that cannot move never sees a leader pull away
  (236s frozen). Releasing at standstill lets it close up and pin - which is also what
  lets the unstick gate be reached at all
- blends advance by distance, so a van wedged mid-manoeuvre never finishes one (149.7s at
  full throttle). A no-progress escape abandons the deviation after the same 10s budget

Execution was the other half: the aim point was shifted by the offset owed *here*, but
sits ~4m ahead where the blend has grown, so the van steered for a fraction of the shift,
levelled out on heading and drifted straight at the blocker. Shifting it by the offset
owed AT the aim point cut peak lag 2.47 -> 1.70m and collisions 54 -> 7, lane-following
bit-identical. EXEC_LAG = 1.8 then sweeps the slower path actually driven, so the van
waits for room to be *settled* rather than settling as it passes, and a runtime check
holds throttle whenever it falls behind that certified path.

| route | scenario        | sim_time_s | collisions | solid_inv | lane_inv | avg km/h | min clear m | completion |
|-------|-----------------|-----------|------------|-----------|----------|----------|-------------|------------|
| 2     | empty           | 187.60    | 0          | 0         | 8        | 14.7     | n/a         | 100%       |
| 5     | parked blocker  | 187.60    | 0          | 0         | 17       | 15.0     | +1.06       | 100%       |
| 6     | 20 TM vehicles  | 187.65    | 0          | 0         | 8        | 14.7     | -0.56 *     | 100%       |

Route 5 beats v2.4 (193.6s, 14.4 km/h) and is provably clear, not merely uncollided;
15 -> 17 invasions is the cost of swinging out earlier. Route 2 saw no vehicle in 3752
ticks, so its match is structural. (*) Route 6's -0.56m is conservative box overlap on
close passes with no contact - min_clearance is a safe proxy, not a collision predictor -
and it never left FOLLOWING, so it tests the latch fix, not the planner.

Null results, recorded in code: scaling the cross-track gain by the blend's remaining room
grows unbounded as it closes (245/288 ticks at full lock, 4.9m off route, 12%); a quintic
blend with zero end curvature changed nothing (lag 2.03 -> 2.07m) and cost clearance
(0.834 -> 0.480m). Both failed for one reason, measurable throughout - at peak error the
controller commands 0.21 of full lock, so steering authority was never the constraint. The
lag is the control law's convergence distance (K = 0.15 corrects over ~5.87m, about a
whole blend); feedforward from path curvature is the fix, and the next chapter.

Limits: EXEC_LAG is from one route and fails unsafe if real lag is worse (the runtime check
makes it observable, not safe); the sweep checks a future path against a present-tense
world; geometry is 2D, so an overpass reads as an obstacle; fine sampling is a resolution,
not a provable bound; no route combines a blocker with traffic, so the lane-entry gates
have never had to refuse.

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
- Lane-centring error: implemented - the control trace carries `cross`, `track_err` and
  `clearance` per tick, summarised by `scripts/track_error.py`
- Baseline (BehaviorAgent) rows predate the red_light_violations column - re-run pending
- No scenario combines a stationary blocker with traffic, so the lane-entry gates have
  never had to refuse a manoeuvre - candidate route 7
- Violation counter can double-count a light a blind agent re-passes; rankings unaffected