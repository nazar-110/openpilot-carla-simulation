# Empirical longitudinal actuation

The CARLA Lincoln MKZ 2020 now uses a measured, speed-dependent inverse pedal
model with bounded acceleration-domain PI correction. OpenPilot supplies the
acceleration request; the adapter does **not** impose a target speed, substitute
a route follower, or remove commanded braking during evaluation.

The stateful controller advances once per 20 Hz CARLA frame. Its chosen request,
measured acceleration, feedforward pedal, feedback correction, saturation and
out-of-calibration speed are exposed in dashboard snapshots and demo metadata.
The causal startup gate still requires fresh OpenPilot controls before release.

## Measurements and independent check

`scripts/calibrate_vehicle.py` drives the actual CARLA drivetrain to each initial
speed, then measures pedal steps at 50 ms intervals. It does not set chassis
velocity: doing so would leave wheel dynamics inconsistent. Standstill/clutch
transients are excluded from fitting. The checked-in profile covers speed bins
2, 5, 8, 12 and 16 m/s, using 712 samples on dry Town10HD_Opt. Outside this speed
range, the nearest measured curve is used and the condition is flagged.

The independent `scripts/validate_actuation.py` compares stock and calibrated
control using the same nine acceleration steps: requests -1, 0 and +1 m/s² from
initial speeds 5, 8 and 12 m/s. Each step lasts 3 seconds; the first second is
excluded. Both controllers contribute 360 samples, including stopped states.

| Metric | Stock linear mapping | Calibrated |
|---|---:|---:|
| Acceleration RMSE (m/s²) | 3.651 | 0.779 |
| Acceleration MAE (m/s²) | 1.976 | 0.471 |

This is a **78.7% RMSE reduction in this actuator test**, not a claim of improved
urban-driving safety. Gear shifts and drivetrain hysteresis remain unmodeled.
Fitting residuals are reported separately from independent validation.

Raw sweeps, profile parameters, physical vehicle properties, measured wheel-angle
checks and validation samples are committed under `reports/calibration` and
`config/calibration`. The profile's SHA-256 is included in run metadata.

## Reproduce

With a separate CARLA 0.9.16 server running and no evaluator active:

```powershell
python scripts/calibrate_vehicle.py --host 127.0.0.1
python scripts/validate_actuation.py --host 127.0.0.1
```

These scripts temporarily own synchronous world ticks and restore the original
world settings. Do not run them alongside another simulator client that ticks
the world. They create and destroy their own calibration actors.

The steering-angle mapping remains the existing geometric mapping; wheel-angle
measurements are diagnostic, not a fitted lateral controller. Wet-road friction,
other vehicles and other CARLA releases need separate calibration. This is a
simulation-only adapter, never a controller for a physical vehicle.

To compare the previous adapter, set `carla.actuation.calibration_profile: null`
in a copy of `config/carla.yaml` and select that runtime config for the trial.
