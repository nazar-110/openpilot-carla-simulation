# Reproducibility checklist

Complete and archive this checklist before confirmatory data collection.

## Frozen software

- [ ] OpenPilot commit is `4df40d2c1946a57242230186edd073c4073060a6`.
- [ ] OpenPilot Git LFS model files are present and `scons -u` succeeds.
- [ ] CARLA client and server both report exactly `0.9.16`.
- [ ] This repository commit is recorded.
- [ ] Python reports at least 3.12.3 and below 3.13 inside Ubuntu 24.04/WSL2.
- [ ] `python -m pytest` passes.
- [ ] `python -m ruff check .` passes.

## Frozen configuration

- [ ] `config/carla.yaml` is archived with a SHA-256 hash.
- [ ] Every scenario YAML is archived with a SHA-256 hash.
- [ ] Camera mount, resolution, FOV, gamma, postprocessing, and sensor tick are fixed.
- [ ] Ego blueprint, steering sign, ratio, and friction factors are fixed.
- [ ] Every ego target is 40 km/h, matching the pinned simulator's fixed initial vCruise.
- [ ] Experiment seeds and order seed are fixed.
- [ ] Metric thresholds and failure taxonomy are preregistered.

## Simulator checks

- [ ] CARLA runs at Epic quality with rendering enabled (`-RenderOffScreen` is acceptable).
- [ ] `no_rendering_mode` is false; camera images are not empty.
- [ ] Exactly one client owns `world.tick()`.
- [ ] World and Traffic Manager are synchronous at 0.05 s.
- [ ] Physics substeps are enabled at <=0.01 s.
- [ ] Traffic Manager hybrid physics is disabled.
- [ ] World reload and all seeds occur before each trial.
- [ ] No manually launched OpenPilot manager is running; the evaluator starts and stops a fresh manager/modeld process group for each OpenPilot trial.
- [ ] Camera road/wide frame IDs match each evaluated tick.
- [ ] Camera handoff events map CARLA frames to OpenPilot camera frame IDs with no duplicates.
- [ ] IMU frame IDs match world ticks.
- [ ] The stopped server-recorder frame count matches the client map and its collision audit is valid.
- [ ] Authoritative lane-invasion events come from synchronous vehicle geometry; asynchronous sensor events remain diagnostic.
- [ ] Client/server version mismatch causes a hard failure.

## Pilot acceptance

- [ ] Three pilot seeds run for every scenario family.
- [ ] Required actor spawn succeeds for every pilot seed.
- [ ] Trigger timing matches the declared TTC condition or its explicit fallback.
- [ ] Scripted speed is within 0.5 km/h before trigger.
- [ ] Scripted lateral path error is within 0.15 m where applicable.
- [ ] Scripted heading error is within 2 degrees where applicable.
- [ ] OpenPilot remains active for the scenario-declared warmup (one second for
      lane-following cases; five seconds for the hazard cases).
- [ ] Lane-following starts have at least 160 m junction clearance and pass the
      non-junction, <=10 degree heading, and <=0.75 m lateral origin checks.
- [ ] Both controllers receive exactly two neutral post-reset ticks before evaluation.
- [ ] The first evaluated OpenPilot tick follows a verified final-settling-camera -> `modelV2` -> `longitudinalPlan` -> `controlsState` -> exact selected `carControl` lineage.
- [ ] Both runs in the two-trial lifecycle smoke check have distinct `openpilot_manager.log` files and no VisionIPC/model timeout on run two.
- [ ] Lane/sign/light geometry is visually inspected for each selected corridor.
- [ ] Pilot results are excluded from confirmatory analysis.

## Collection

- [ ] Paired blocks are randomized with the declared order seed.
- [ ] Invalid simulator trials are retained, reported, and rerun with the same seed.
- [ ] System failures are not rerun away.
- [ ] Existing summaries are reused only after the provenance gate reports a match.
- [ ] Stale completed summaries are archived only through explicit `--overwrite`.
- [ ] Raw metadata, telemetry, events, summaries, and OpenPilot manager logs are backed up read-only.
- [ ] When recorder replay retention is enabled, unique server-side `.rec` files are archived separately; otherwise the fixed integrity recorder is expected to be overwritten.
- [ ] A 5% exact-repeat determinism audit is collected separately.

## Analysis and reporting

- [ ] Only complete runs are treated as independent units.
- [ ] Collision trials remain in TTC analysis with TTC = 0.
- [ ] Every artifact with `valid_for_research: false` (smoke or invalid trial) is excluded from performance claims and separately accounted.
- [ ] Traffic Manager is labeled a privileged-state reference.
- [ ] Unsupported/boundary scenarios are reported descriptively.
- [ ] Confidence intervals and effect sizes accompany p-values.
- [ ] Multiple primary contrasts use the preregistered correction.
- [ ] All S2/S3 failures and a 10% pass sample receive dual review.
- [ ] Construct-validity limitations appear with every main conclusion.
