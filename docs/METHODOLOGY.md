# Methodology

## Study framing

The study asks:

> How reliably, safely, and comfortably does OpenPilot provide intervention-free
> lane and longitudinal assistance in controlled CARLA urban scenarios as
> environmental condition and hazard type vary?

OpenPilot is evaluated as an assistance system. Pedestrian, signal, stationary
obstruction, and close-cut-in cases are exploratory operating-boundary probes.
The study must not claim autonomous-driving certification, ISO conformance, or
product noncompliance.

## Experimental unit and factors

The independent unit is one complete seeded run. Telemetry frames are repeated
measurements inside that unit and must never be counted as independent samples.

Main factors:

- controller: OpenPilot or privileged CARLA Traffic Manager reference;
- scenario family: seven configured urban cases;
- environmental condition: clear noon, clear night, or rainy daylight with a
  0.8 tire-friction factor;
- paired seed: ten seeds, `41000` through `41009`.

This produces 7 x 3 x 10 x 2 = 420 controller trials (210 matched pairs). The
same seed is paired across both controllers. Pair-member order alternates by
repetition, while `--randomize-order --order-seed 20260714` randomizes complete
scenario-condition-seed blocks and keeps each pair adjacent.

Rain versus clear estimates a bundled rendering-plus-traction effect; it is not
an isolated visibility estimand. Night versus clear changes rendering/lighting
while retaining the reference friction factor.

## Preregistered execution rules

1. Conduct a three-seed pilot for setup, trigger realization, and runtime only.
2. Freeze the OpenPilot commit, model files, CARLA version, scenario YAML,
   runtime YAML, camera calibration, vehicle blueprint, and analysis code.
3. Delete/exclude pilot results before confirmatory collection.
4. Start from a freshly reloaded synchronous world for every run. For each
   OpenPilot run, start and stop a fresh evaluator-owned manager/modeld process
   group; reject a pre-existing manager from the selected checkout.
5. Seed Python scenario selection, Traffic Manager, and camera noise.
6. Require the scenario-declared continuous active-control warmup before
   evaluation starts. Lane-following cases use one second after a speed-capped
   bootstrap; hazard cases use five seconds.
7. After the continuous warmup, run exactly two neutral-control ticks for either
   controller. For OpenPilot, use the final settling camera pair as the causal
   target and require an exact non-conflated
   `camera -> modelV2 -> longitudinalPlan -> controlsState -> selected carControl`
   lineage. Keep applying neutral commands and pause the world after tick two
   until the verified command is applied; only then advance the first evaluated
   tick. Reject a continuous origin that is in a junction, more than 10 degrees
   from lane heading, or more than 0.75 m from lane center.
8. Provide only declared simulated camera/CAN/IMU/GNSS inputs to OpenPilot.
9. Use evaluator ground truth only for scenario triggers and offline metrics.
10. Count engagement timeout and post-warmup disengagement as controller failures.
   Treat camera timeout, bridge crash, or deadline failure as invalid
   infrastructure artifacts and retain their attempted seeds in accounting.
11. Treat CARLA crash, version mismatch, failed required actor spawn, or invalid
    scripted-actor realization as an invalid trial; report it and rerun the same
    seed.

Repeat 5% of completed seeds exactly as a determinism audit. Keep audit repeats
separate from inference.

## Timing and determinism

CARLA uses:

```text
synchronous_mode = true
fixed_delta_seconds = 0.05
substepping = true
max_substep_delta_time = 0.01
max_substeps = 10
```

Thus `0.05 <= 0.01 * 10`, satisfying CARLA's physics-substep constraint. The
world is reloaded after synchronous settings are applied. Traffic Manager is
synchronous, seeded after reload, and hybrid physics is disabled. Only the
bridge calls `world.tick()`. Continuous IMU measurements are matched to the
exact returned world frame.

Every real run also starts a server-side CARLA recorder. At teardown, the
recorder is stopped before sensor subscriptions and its completed collision log
is reconciled against the client world-frame map. The stopped recorder is the
authority for collision occurrence; any stop, parse, or frame-mapping failure
invalidates the trial. Lane-boundary crossings are computed synchronously from
the ego bounding box, driving-lane width, and present markings. Asynchronous
collision and lane-invasion callbacks are queued diagnostics; they are not a
terminal-event completion barrier and do not define the lane metric.

These requirements follow CARLA's
[determinism documentation](https://carla.readthedocs.io/en/0.9.16/adv_synchrony_timestep/).

## Outcomes

### Primary outcome

`intervention_free_success` is true when all applicable primary criteria pass:

- route completion at least 99%;
- zero deduplicated collision episodes;
- zero lane-invasion events, except the explicitly configured intersection or
  obstruction tolerance;
- off-road duration no more than 0.1 s;
- zero red-light violations in the included signal scenario;
- controller-active fraction at least 99% after evaluation begins;
- zero post-warmup controller disengagement events.

The schema reserves a stop-sign metric for a future map-validated stop-sign
scenario, but the included suite does not instrument one and stop-sign counts do
not enter the primary composite.

Thresholds are stored in each scenario file and copied into each summary. A
threshold change creates a new scenario version; it must not rewrite completed
trials.

`quality_pass` additionally requires the scenario's TTC, lateral-error, jerk,
and control-latency limits. Comfort is not part of the primary safety composite,
so necessary emergency braking cannot redefine an otherwise intervention-free
outcome.

### Safety

Collision episodes are deduplicated by other-actor ID within 0.5 s. Recorder
entries guarantee occurrence accounting; callback event count and peak impulse
remain available when the asynchronous collision callback was captured. Any
pre-termination collision fails the run and sets minimum TTC to zero to avoid
survivor bias.

For a forward actor, the implemented TTC approximation is:

```text
gap = max(0, center_distance - 4.0 m)
closing_speed = max(0, ego_forward_speed - actor_forward_speed)
TTC = gap / closing_speed, when closing_speed > 0.05 m/s
```

The 4 m footprint approximation should be replaced with projected CARLA bounding
boxes before publication-grade vehicle-specific TTC. TTC below 1.5 s creates a
near-miss episode. The 1.5 s conflict convention is consistent with the FHWA
[SSAM report](https://highways.dot.gov/sites/fhwa.dot.gov/files/FHWA-HRT-08-051.pdf),
but is not a collision-certification threshold.

Hard braking is an episode with longitudinal acceleration below -3 m/s2.
Off-road time is integrated from frame intervals where no driving-lane waypoint
contains the ego. Outside junctions, a projected-lane fallback allows the lane
half-width plus 0.5 m; junctions retain the last unambiguous lateral offset and
are not labeled off-road by this approximation.

### Lane keeping

Signed lateral error is the projection from the lane waypoint center to the ego
location onto the waypoint right vector:

```text
e_y = (ego_position - lane_center) dot lane_right
```

Report mean signed error, MAE, RMSE, and maximum absolute error. Junction
centerline values are held at the last unambiguous lane value. Synchronous
marking-crossing detection is authoritative only outside junctions when a
driving-lane waypoint exists; the asynchronous lane-invasion callback is
diagnostic only. Off-road classification is evaluated separately. A future
route corridor and junction-specific boundary model should replace these
approximations for junction-focused studies.

### Speed and mission

Report route completion, elapsed simulation time, traveled distance, mean speed,
speed MAE/RMSE, and overspeed duration. Overspeed is currently speed above the
CARLA limit plus 0.5 m/s. A speeding event is a transition into that state.

### Comfort

Project CARLA acceleration onto the ego forward and right vectors. Before
calculating longitudinal jerk, apply the declared causal 0.5 s moving-average
filter:

```text
j[k] = (a_filtered[k] - a_filtered[k-1]) / dt
```

Report RMS/peak longitudinal and lateral acceleration, RMS jerk, and 95th
percentile absolute jerk. The scenario's engineering bounds are not regulatory
claims.

### Timing

The latency proxy is wall-clock time from handoff of the most recent
frame-matched camera pair to the next newly published `carControl` message.
The one-frame handoff applies backpressure until RGB-to-YUV conversion of the
previous pair has finished, and `events.jsonl` maps each CARLA frame to its
OpenPilot camera `frameId`. Report p50, p95, maximum, evaluated camera drops,
startup discards, and camera timeouts. The proxy includes model and messaging
delay but is not a hardware end-to-end latency measurement. The default quality
bound is p95 <= 100 ms.

This latency proxy is distinct from the post-reset release proof. The release
gate losslessly captures non-conflated `modelV2`, `longitudinalPlan`,
`controlsState`, `carControl`, and `lateralManeuverPlan` events and keys them
by `Event.logMonoTime`. It accepts only the exact `carControl` selected by the
simulator when its model frame IDs equal the final settling camera ID, its
longitudinal plan references that model, the same controlsd cycle's
`controlsState` references both messages, all messages are valid/active, and no
maneuver/debug bypass is enabled. The event timestamps must be strictly ordered,
and a gap in the captured alternating `controlsState`/`carControl` stream fails
closed. `summary.json` records the accepted lineage.

## Statistical analysis

The included analysis performs descriptive and paired run-level statistics:

- success rate with Wilson 95% confidence interval for each cell;
- paired OpenPilot-minus-reference success risk difference;
- paired nonparametric bootstrap 95% interval with 5,000 resamples;
- exact McNemar test for discordant paired binary outcomes;
- paired mean lateral-RMSE difference with bootstrap interval.
- matched condition-minus-clear success effects with Holm-adjusted McNemar
  p-values.

The included analysis reports within-controller night-minus-clear and
rain-minus-clear contrasts. Planned severity contrasts require adding frozen
mild/medium/hard scenario files; do not infer severity effects from the single
included medium case.

For a full thesis analysis, add:

- paired median differences and Wilcoxon/permutation tests for continuous data;
- mixed-effects logistic regression
  `success ~ controller * environmental_condition + severity + (1|family) + (1|seed)`;
- negative-binomial models for naturalistic infraction counts with distance as
  an offset;
- Holm correction across preregistered primary contrasts;
- effect sizes and confidence intervals alongside p-values.

Ten seeds per cell give wide cell-level binary intervals. Pool only according to
the preregistered model; never inflate sample size with frames.

## Failure taxonomy

Assign three independent labels after reviewing synchronized telemetry, events,
and a retained CARLA recorder replay when visual adjudication is required.
Recorder collision auditing is always active; replay retention requires
`carla.recording.carla_recorder: true` and separate archiving of the server-side
file.

Observed impact:

```text
COLLISION_VRU       COLLISION_VEHICLE  COLLISION_STATIC
TTC_CRITICAL        LANE_DEPARTURE     RULE_RED
RULE_STOP           RULE_SPEED         ROUTE_DEVIATION
STUCK               DISENGAGEMENT      COMFORT
DEADLINE
```

Probable failure mode:

```text
PERCEPTION_LANE       PERCEPTION_LEAD       PERCEPTION_VRU
PERCEPTION_SIGNAL     PREDICTION            DECISION
PATH_PLANNING         LONGITUDINAL_CONTROL  LATERAL_CONTROL
ACTUATOR_SATURATION   INTEGRATION_TIMING    UNSUPPORTED_ODD
TEST_ARTIFACT         UNKNOWN
```

Severity:

- S0: pass;
- S1: tracking or comfort degradation only;
- S2: intervention, rule violation, road departure, or critical conflict;
- S3: collision.

Label the first causal failure as primary and link downstream consequences.
Two reviewers should independently classify all S2/S3 runs plus a 10% pass
sample, report Cohen's kappa, and adjudicate disagreements.

## Construct-validity limitations

- OpenPilot simulates Honda Civic 2022 radarless CAN while CARLA uses a Lincoln
  vehicle physics actor.
- Steering mapping uses an explicit but synthetic steering ratio.
- OpenPilot's sim CAN, GNSS, IMU, and driver monitoring are simplified.
- The exact post-reset causal lineage proves direct participation of the final
  settling camera pair, not OpenPilot state purity. At the pinned commit,
  modeld retains roughly 4.85 seconds of temporal feature history, while model
  action smoothing and planner/control state can survive the two-tick reset.
  The per-trial manager restart removes cross-trial carryover but does not erase
  the same trial's pre-evaluation warmup history.
- Background Traffic Manager behavior is not a calibrated human-driver model.
- CARLA camera rendering and weather are not real sensor-domain validation.
- Traffic-light stop-line crossing currently uses the ego center and CARLA
  trigger geometry.
- TTC uses a fixed footprint approximation.
- The seven cases are scenario samples, not a proof of urban safety coverage.

These limitations define the claim: results characterize this pinned
software-in-the-loop system, not a real vehicle and not general urban autonomy.
