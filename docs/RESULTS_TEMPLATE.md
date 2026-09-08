# Results chapter template

## Reproducibility statement

Report the exact CARLA, OpenPilot, bridge, model, scenario, runtime-config, and
analysis commits/hashes. Use `collection_manifest.json` for run order and
toolkit context, and archive the separately captured OpenPilot LFS, hardware,
GPU-driver, WSL, and CARLA-launch manifest. State excluded pilot runs,
invalid-trial count, and determinism-audit result. Confirm that OpenPilot
manager/modeld was restarted for every trial and retain the per-run
`openpilot_manager.log` files.

## Dataset accounting

| Category | Planned | Completed | Invalid simulator trials | System failures | Included |
|---|---:|---:|---:|---:|---:|
| Assistance envelope | | | | | |
| Boundary/unsupported | | | | | |
| Total | 420 | | | | |

List every invalid seed, reason, and same-seed rerun. Never merge simulator
invalidity with controller failure.

## Primary outcome

Present intervention-free success by scenario, environmental condition, and controller with
95% confidence intervals. Include the components: completion, collision, lane,
off-road, rule compliance, and disengagement.

Do not lead with a single pooled success percentage. The mix of unsupported
boundary cases makes unqualified pooling misleading.

## Safety and conflict results

Report collision type/rate, impulse availability, minimum TTC including
collision-as-zero, near-miss rate, hard-braking episodes, synchronous geometric
lane invasions, and off-road exposure. Report the stopped server-recorder audit
status and distinguish recorder-authoritative collision occurrence from
callback-supplied impulse. Include synchronized telemetry/event case studies and
retained CARLA recorder replays of every S3 and representative S2 failure when
server-side replay retention was enabled.

## Tracking, comfort, and timing

Report lateral RMSE/maximum, speed RMSE, acceleration, filtered jerk, the
camera-to-new-`carControl` latency proxy, evaluated dropped frames, intentional
startup discards, CARLA-to-OpenPilot frame mapping, and timeouts. Separately
report the post-reset final-camera -> model -> longitudinal plan -> controls
state -> exact selected-control lineage result; do not present the latency proxy
as causal proof.

## Paired contrasts

Report paired effect sizes and confidence intervals for:

- OpenPilot night minus clear;
- OpenPilot rain minus clear;
- OpenPilot minus privileged reference, with the fairness caveat;
- any preregistered severity contrasts added before collection.

## Failure taxonomy and review reliability

Tabulate observed impact, probable failure mode, and severity. Report dual-review
coverage, Cohen's kappa, disagreements, and adjudication method.

## Limitations

At minimum discuss the simulated Honda/Lincoln mismatch, synthetic steering
mapping, simplified CAN/sensors, CARLA rendering domain gap, Traffic Manager's
privileged state, scenario coverage, stop-line/TTC approximations, and the
assistance-system framing.

## Claim template

Use wording such as:

> In the pinned OpenPilot v0.11.1/CARLA 0.9.16 software-in-the-loop
> configuration, intervention-free assistance success changed by [effect] under
> [condition], with [confidence interval]. Boundary tests involving [unsupported
> function] are descriptive and do not represent OpenPilot product-compliance
> requirements.

Avoid wording that generalizes to real vehicles, autonomous urban driving, or
regulatory certification.
