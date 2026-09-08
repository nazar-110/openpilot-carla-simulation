from __future__ import annotations

from pathlib import Path

import pytest

from opencarla_eval.metrics import compute_metrics
from opencarla_eval.models import EventRecord, MetricThresholds, TelemetrySample
from opencarla_eval.recorder import RunRecorder


def _sample(index: int, *, active: bool = True, route: float | None = None) -> TelemetrySample:
  return TelemetrySample(
    frame=index,
    sim_time_s=index * 0.1,
    x_m=float(index),
    y_m=0.1,
    z_m=0.0,
    yaw_deg=0.0,
    speed_mps=10.0,
    target_speed_mps=10.0,
    acceleration_mps2=0.0,
    lateral_acceleration_mps2=0.1,
    steering_angle_deg=0.0,
    throttle=0.1,
    brake=0.0,
    lane_offset_m=0.1,
    route_completion=route if route is not None else index / 20,
    controller_active=active,
    ttc_s=2.0,
    speed_limit_mps=10.0,
    control_latency_ms=40.0,
  )


def test_nominal_run_passes_primary_and_quality_criteria() -> None:
  samples = [_sample(index, active=index >= 2) for index in range(21)]
  metrics = compute_metrics(samples, [], MetricThresholds())

  assert metrics["success"] is True
  assert metrics["quality_pass"] is True
  assert metrics["mission"]["route_completion"] == 1.0
  # Startup samples before the first activation are outside the evaluation fraction.
  assert metrics["system"]["controller_active_fraction"] == 1.0


def test_collision_callbacks_are_deduplicated_and_ttc_is_zero() -> None:
  samples = [_sample(index) for index in range(21)]
  events = [
    EventRecord("collision", 1.0, 10, {"other_actor_id": 7, "impulse_ns": 10.0}),
    EventRecord("collision", 1.1, 11, {"other_actor_id": 7, "impulse_ns": 12.0}),
    EventRecord("collision", 1.2, 12, {"other_actor_id": 9, "impulse_ns": 8.0}),
  ]
  metrics = compute_metrics(samples, events)

  assert metrics["safety"]["collision_count"] == 2
  assert metrics["safety"]["collision_event_count"] == 3
  assert metrics["safety"]["min_ttc_s"] == 0.0
  assert metrics["success"] is False


def test_comfort_failure_does_not_redefine_intervention_free_success() -> None:
  samples = [_sample(index) for index in range(21)]
  for index in range(5, 21, 2):
    samples[index] = TelemetrySample(**{**samples[index].to_dict(), "acceleration_mps2": 20.0})
  metrics = compute_metrics(samples, [], MetricThresholds(max_rms_jerk_mps3=0.1))

  assert metrics["success"] is True
  assert metrics["quality_pass"] is False
  assert metrics["criteria"]["comfort_jerk"]["pass"] is False


def test_single_disengagement_fails_even_when_active_fraction_is_high() -> None:
  samples = [_sample(index) for index in range(201)]
  events = [EventRecord("controller_disengagement", 20.0, 200)]
  metrics = compute_metrics(samples, events)

  assert metrics["system"]["controller_active_fraction"] == 1.0
  assert metrics["system"]["controller_disengagement_count"] == 1
  assert metrics["success"] is False


def test_duplicate_timestamp_is_rejected() -> None:
  samples = [_sample(0), TelemetrySample(**{**_sample(1).to_dict(), "sim_time_s": 0.0})]

  with pytest.raises(ValueError, match="timestamps"):
    compute_metrics(samples, [])


def test_infrastructure_termination_overrides_passing_metrics(tmp_path: Path) -> None:
  metadata = {
    "run_id": "test",
    "experiment": "test",
    "controller": "openpilot",
    "backend": "carla",
    "valid_for_research": True,
    "repetition": 1,
    "seed": 1,
    "scenario": {"id": "lane", "kind": "lane_following"},
    "condition": {"id": "clear"},
  }
  recorder = RunRecorder(tmp_path, metadata, MetricThresholds())
  for index in range(21):
    recorder.record_sample(_sample(index))

  summary = recorder.finalize("camera_timeout")

  assert summary["metrics"]["success"] is False
  assert summary["metrics"]["criteria"]["system_integrity"]["pass"] is False
  assert summary["valid_for_research"] is False


def test_failed_sample_serialization_does_not_corrupt_recorder(tmp_path: Path) -> None:
  metadata = {
    "run_id": "test",
    "experiment": "test",
    "controller": "openpilot",
    "backend": "carla",
    "valid_for_research": True,
    "repetition": 1,
    "seed": 1,
    "scenario": {"id": "lane", "kind": "lane_following"},
    "condition": {"id": "clear"},
  }
  recorder = RunRecorder(tmp_path, metadata, MetricThresholds())
  bad = TelemetrySample(**{**_sample(0).to_dict(), "speed_mps": float("nan")})

  with pytest.raises(ValueError):
    recorder.record_sample(bad)
  summary = recorder.finalize("camera_timeout")

  assert recorder.samples == []
  assert summary["metrics"]["success"] is False
