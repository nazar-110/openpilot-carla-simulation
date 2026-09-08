"""Small deterministic backend used to verify the experiment pipeline in CI.

This is deliberately not a vehicle-dynamics or OpenPilot substitute. Every
artifact it emits is marked ``valid_for_research: false``.
"""

from __future__ import annotations

import hashlib
import math
import random
from dataclasses import dataclass
from typing import Any

from .attempts import archive_existing_attempt
from .models import EventRecord, RunSpec, TelemetrySample
from .provenance import require_reusable_summary
from .recorder import RunRecorder


@dataclass
class _SyntheticState:
  x: float = 0.0
  y: float = 0.0
  speed: float = 0.0
  acceleration: float = 0.0
  lead_x: float | None = None
  lead_speed: float = 0.0
  trigger_fired: bool = False
  collision_recorded: bool = False
  near_miss_recorded: bool = False
  red_violation_recorded: bool = False
  lane_invasion_active: bool = False


def _environment_noise(run: RunSpec) -> float:
  weather = run.condition.weather.lower()
  penalty = 0.02
  if "night" in weather:
    penalty += 0.07
  if "rain" in weather:
    penalty += 0.10
  return penalty + run.condition.camera_noise_std / 255.0


def _desired_acceleration(
  run: RunSpec,
  state: _SyntheticState,
  sim_time: float,
) -> tuple[float, float | None, float | None]:
  """Return desired acceleration, forward gap, and TTC for the smoke model."""

  target = run.scenario.target_speed_mps
  desired = max(-3.5, min(2.0, (target - state.speed) * 0.8))
  gap: float | None = None
  ttc: float | None = None
  kind = run.scenario.kind
  params = run.scenario.parameters

  if kind == "lead_vehicle_braking":
    if state.lead_x is None:
      state.lead_x = float(params.get("lead_initial_gap_m", 30.0))
      state.lead_speed = float(params.get("lead_speed_mps", target))
    gap = state.lead_x - state.x - 4.5
    closing = max(0.0, state.speed - state.lead_speed)
    ttc = gap / closing if closing > 0.05 and gap >= 0 else None
    trigger_min = float(params.get("trigger_min_time_s", 2.0))
    fallback = float(params.get("fallback_trigger_s", 7.0))
    if (
      sim_time >= trigger_min
      and (
        ttc is not None and ttc <= float(params.get("trigger_ttc_s", 2.0)) or sim_time >= fallback
      )
      and not state.trigger_fired
    ):
      state.trigger_fired = True
    if state.trigger_fired:
      state.lead_speed = max(0.0, state.lead_speed - 6.0 * 0.05)
    desired_gap = 4.0 + max(state.speed, 0.0) * 1.6
    if gap < desired_gap:
      desired = min(desired, -min(6.0, (desired_gap - gap) * 0.7))

  elif kind in {"pedestrian_crossing", "parked_vehicle_obstruction"}:
    obstacle_x = float(
      params.get(
        "crossing_distance_m" if kind == "pedestrian_crossing" else "obstacle_distance_m",
        40.0,
      )
    )
    gap = obstacle_x - state.x
    closing = max(state.speed, 0.0)
    ttc = gap / closing if closing > 0.05 and gap >= 0 else None
    trigger_min = float(params.get("trigger_min_time_s", 2.0))
    fallback = float(params.get("fallback_trigger_s", 8.0))
    if sim_time >= trigger_min and (
      ttc is not None and ttc <= float(params.get("trigger_ttc_s", 2.5)) or sim_time >= fallback
    ):
      state.trigger_fired = True
    if state.trigger_fired and gap < max(8.0, state.speed * 2.0):
      desired = min(desired, -4.0)

  elif kind == "vehicle_cut_in":
    if state.lead_x is None:
      state.lead_x = state.x + float(params.get("intruder_gap_m", 24.0))
      state.lead_speed = float(params.get("intruder_speed_mps", target - 2.0))
    prospective_gap = state.lead_x - state.x - 4.5
    prospective_closing = max(0.0, state.speed - state.lead_speed)
    prospective_ttc = (
      prospective_gap / prospective_closing
      if prospective_closing > 0.05 and prospective_gap >= 0
      else None
    )
    trigger_min = float(params.get("trigger_min_time_s", 2.0))
    fallback = float(params.get("fallback_trigger_s", 10.0))
    if sim_time >= trigger_min and (
      prospective_ttc is not None
      and prospective_ttc <= float(params.get("trigger_ttc_s", 2.0))
      or sim_time >= fallback
    ):
      state.trigger_fired = True
    if state.trigger_fired:
      gap = state.lead_x - state.x - 4.5
      closing = max(0.0, state.speed - state.lead_speed)
      ttc = gap / closing if closing > 0.05 and gap >= 0 else None
      if gap < 4.0 + state.speed * 1.5:
        desired = min(desired, -4.0)

  elif kind == "signalized_intersection":
    stop_line = float(params.get("signal_sight_distance_m", 45.0))
    red_until = float(params.get("red_hold_s", 20.0))
    state.trigger_fired = True
    gap = stop_line - state.x
    ttc = gap / state.speed if state.speed > 0.05 and gap >= 0 else None
    if sim_time < red_until and gap < max(7.0, state.speed * 2.0):
      desired = min(desired, -3.5)

  return desired, gap, ttc


def run_synthetic(run: RunSpec, overwrite: bool = False) -> dict[str, Any]:
  """Execute one fast deterministic pipeline smoke run."""

  summary_path = run.output_dir / "summary.json"
  if summary_path.exists() and not overwrite:
    import json

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    require_reusable_summary(summary, run, backend="synthetic_smoke")
    return summary
  if overwrite:
    archive_existing_attempt(run.output_dir)

  rng = random.Random(run.seed)
  metadata = run.metadata(backend="synthetic_smoke", valid_for_research=False)
  metadata["artifact_class"] = "synthetic_smoke"
  metadata["invalid_reason"] = "synthetic_smoke"
  if run.scenario.source_path is not None and run.scenario.source_path.is_file():
    metadata["scenario_sha256"] = hashlib.sha256(run.scenario.source_path.read_bytes()).hexdigest()
  metadata["warning"] = (
    "Synthetic smoke data verifies plumbing only; it is not an OpenPilot or CARLA result."
  )
  state = _SyntheticState()
  recorder = RunRecorder(run.output_dir, metadata, run.scenario.thresholds)
  dt = 0.05
  # Exercise the declared warmup parameter without recording it as evaluation
  # exposure. The smoke backend starts the evaluated window at the declared
  # cruise speed, just like the real adapter's post-warmup reset.
  warmup_s = 1.0 + float(run.scenario.parameters.get("warmup_active_s", 5.0))
  for _ in range(int(math.ceil(warmup_s / dt))):
    desired = max(-3.5, min(2.0, (run.scenario.target_speed_mps - state.speed) * 0.8))
    state.acceleration += max(-4.0 * dt, min(4.0 * dt, desired - state.acceleration))
    state.speed = max(0.0, state.speed + state.acceleration * dt)
  state = _SyntheticState(speed=run.scenario.target_speed_mps)
  frame_count = int(math.ceil(run.scenario.duration_s / dt)) + 1
  noise = _environment_noise(run)
  termination = "timeout"

  try:
    recorder.record_event(EventRecord("evaluation_started", 0.0, 0))
    for frame in range(frame_count):
      sim_time = frame * dt
      controller_active = True

      desired_accel, gap, ttc = _desired_acceleration(run, state, sim_time)
      previous_acceleration = state.acceleration
      max_accel_change = 4.0 * dt
      state.acceleration += max(
        -max_accel_change, min(max_accel_change, desired_accel - state.acceleration)
      )
      state.speed = max(0.0, state.speed + state.acceleration * dt)
      state.x += state.speed * dt
      if state.lead_x is not None:
        state.lead_x += state.lead_speed * dt

      phase = 0.16 * sim_time + (run.seed % 31) / 10.0
      lane_offset = noise * math.sin(phase) + rng.gauss(0.0, noise * 0.08)
      state.y = lane_offset
      lateral_acceleration = -noise * 0.16**2 * math.sin(phase)
      offroad = abs(lane_offset) > 1.75

      lane_crossed = abs(lane_offset) > 0.9
      if lane_crossed and not state.lane_invasion_active:
        recorder.record_event(
          EventRecord(
            "lane_invasion",
            sim_time,
            frame,
            {"markings": ["synthetic_boundary"]},
          )
        )
      state.lane_invasion_active = lane_crossed

      if state.trigger_fired and not any(
        event.event_type == "scenario_trigger" for event in recorder.events
      ):
        recorder.record_event(
          EventRecord("scenario_trigger", sim_time, frame, {"kind": run.scenario.kind})
        )

      if ttc is not None and ttc < 1.5 and not state.near_miss_recorded:
        recorder.record_event(EventRecord("near_miss", sim_time, frame, {"ttc_s": ttc}))
        state.near_miss_recorded = True

      if gap is not None and gap <= 0.0 and not state.collision_recorded:
        recorder.record_event(
          EventRecord(
            "collision",
            sim_time,
            frame,
            {"other_actor_id": "synthetic_hazard", "impulse_ns": 1.0},
          )
        )
        state.collision_recorded = True

      if (
        run.scenario.kind == "signalized_intersection"
        and sim_time < float(run.scenario.parameters.get("red_hold_s", 20.0))
        and state.x > float(run.scenario.parameters.get("signal_sight_distance_m", 45.0))
        and not state.red_violation_recorded
      ):
        recorder.record_event(EventRecord("red_light_violation", sim_time, frame))
        state.red_violation_recorded = True

      route_completion = min(1.0, state.x / run.scenario.route_length_m)
      control_latency = max(0.0, rng.gauss(45.0 + noise * 60.0, 5.0))
      steering_angle = -lane_offset * 4.0
      sample = TelemetrySample(
        frame=frame,
        sim_time_s=sim_time,
        x_m=state.x,
        y_m=state.y,
        z_m=0.0,
        yaw_deg=0.0,
        speed_mps=state.speed,
        target_speed_mps=run.scenario.target_speed_mps,
        acceleration_mps2=state.acceleration,
        lateral_acceleration_mps2=lateral_acceleration,
        steering_angle_deg=steering_angle,
        throttle=max(0.0, min(1.0, state.acceleration / 2.0)),
        brake=max(0.0, min(1.0, -state.acceleration / 6.0)),
        lane_offset_m=lane_offset,
        route_completion=route_completion,
        controller_active=controller_active,
        offroad=offroad,
        nearest_actor_distance_m=gap,
        closing_speed_mps=(state.speed - state.lead_speed) if state.lead_x is not None else None,
        ttc_s=ttc,
        speed_limit_mps=run.scenario.target_speed_mps,
        control_latency_ms=control_latency,
      )
      recorder.record_sample(sample)

      if route_completion >= 1.0:
        termination = "route_completed"
        break
      if state.collision_recorded and run.scenario.parameters.get("terminate_on_collision", True):
        termination = "collision"
        break

    return recorder.finalize(
      termination,
      {
        "generator": "deterministic_kinematic_smoke",
        "final_acceleration_mps2": state.acceleration,
        "last_acceleration_delta_mps2": state.acceleration - previous_acceleration,
      },
    )
  except BaseException as exc:
    recorder.abort("synthetic_backend_error", exc)
    raise
