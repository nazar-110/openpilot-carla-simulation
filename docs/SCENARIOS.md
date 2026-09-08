# Scenario catalog

All scenario parameters live in versioned YAML. The main suite uses one
predefined medium-severity case per family, three environmental conditions, and
ten paired seeds. Add severity variants as separate YAML files so results retain
a stable, human-readable scenario ID.

## Assistance-envelope cases

### Marked-lane urban curves

Target speed is 40 km/h over a 300 m route with at least 20 degrees of heading
change. The main outputs are centerline error, lane invasions, lateral
acceleration, jerk, and intervention-free completion.

The selected start must provide at least 160 m before the first junction. A
5 m/s capped bootstrap and one-second active warmup keep the continuous
evaluation origin inside that corridor; the origin guard rejects junction,
heading, or lateral misalignment instead of projecting the ego onto another
lane.

### Hard-braking lead vehicle

Ego and lead initially target 40 km/h. After the common warmup/reset boundary,
the lead receives a maximum-brake command when the desired TTC boundary is
reached, or at the declared fallback time if it is not. It stops, dwells for five
seconds after reaching standstill, and resumes. CARLA-measured gap, closing
speed, TTC, achieved peak deceleration, recorder-authoritative collision
occurrence, and callback impulse availability are logged.
Non-collision trials require at least 3.0 m/s² observed peak lead deceleration.

## Boundary/unsupported cases

These cases intentionally probe functions OpenPilot does not claim. Report them
descriptively and never as evidence of regulatory or product noncompliance.

### Scripted pedestrian crossing

An adult walker crosses at 1.4 m/s from the roadside. The trigger targets a 2.5 s
TTC and has an explicit fallback time. No occluder is modeled. A collision is a
failure; PET is a planned extension.

Non-collision trials require at least 2.0 m of observed walker travel.

### Adjacent-lane cut-in

A vehicle traveling about 10 km/h slower begins a forced lane change from the
adjacent same-direction lane when the prospective TTC reaches the declared
boundary, or at its fallback time. CARLA Traffic Manager performs the lateral
motion; measured realization must be checked before accepting the trial.

Non-collision trials require the actor to reach no more than 1.75 m lateral
separation from the ego path.

### Red-light intersection

The selected governing signal group is commanded red on each synchronous tick
for the declared hold interval. The evaluator detects the ego center crossing
the traffic-light stop line while red. A production study
should refine this to the front-bumper crossing of a map-validated stop line.

### Parked obstruction

A hand-braked four-wheel vehicle is offset 1.1 m from lane center, partially
blocking the travel lane. The simulated radar CAN remains non-privileged, so the
case primarily probes the vision/planning path.

### Degraded-visibility lane tracking

Repeats urban lane tracking under clear-night or rainy-daylight conditions.
Weather changes rendering; rainy tire friction is separately scaled to 0.8 on
all vehicles because CARLA precipitation does not change road friction
automatically.

## Environmental conditions

| ID | Rendering | Tire-friction factor | Purpose |
|---|---|---:|---|
| `clear_noon` | `ClearNoon` | 1.0 | reference |
| `clear_night` | custom negative sun angle, streetlights | 1.0 | lighting contrast |
| `rainy_daylight` | `HardRainNoon` | 0.8 | bundled rain/wet-traction stress |

Night and rain are not combined in the confirmatory matrix. Night versus clear
is a lighting/rendering contrast. Rain versus clear intentionally bundles rain
rendering and reduced vehicle friction and must be interpreted as an
environmental-condition effect, not isolated visibility.

## Adding a scenario

1. Copy the nearest file in `scenarios/`.
2. Assign a unique lowercase `id` and one of the supported `kind` values.
3. Declare duration, route length, trigger parameters, and every threshold
   explicitly. The pinned bridge currently requires the 40 km/h target used by
   OpenPilot's simulator cruise helper.
4. Add the path to an experiment YAML.
5. Run `opencarla-eval validate` and the tests.
6. Pilot with three seeds and inspect actor realization before freezing the case.

Do not change a scenario YAML after confirmatory collection begins. Create a new
ID/version instead.
