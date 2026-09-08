"""Deterministic safety, comfort, compliance, and performance metrics."""

from __future__ import annotations

import math
from collections import defaultdict
from collections.abc import Iterable, Sequence
from statistics import fmean
from typing import Any

from .models import EventRecord, MetricThresholds, TelemetrySample


def _rms(values: Sequence[float]) -> float:
  return math.sqrt(fmean(value * value for value in values)) if values else 0.0


def _percentile(values: Sequence[float], percentile: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  position = (len(ordered) - 1) * percentile / 100.0
  lower = math.floor(position)
  upper = math.ceil(position)
  if lower == upper:
    return float(ordered[lower])
  fraction = position - lower
  return float(ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction)


def _event_count(events: Iterable[EventRecord], event_type: str) -> int:
  return sum(event.event_type == event_type for event in events)


def _deduplicated_collisions(events: Sequence[EventRecord], window_s: float = 0.5) -> int:
  """Count impact episodes, not every consecutive-frame collision record."""

  last_by_actor: dict[str, float] = defaultdict(lambda: -math.inf)
  count = 0
  for event in sorted(events, key=lambda item: item.sim_time_s):
    if event.event_type != "collision":
      continue
    actor = str(event.details.get("other_actor_id", "unknown"))
    if event.sim_time_s - last_by_actor[actor] > window_s:
      count += 1
    last_by_actor[actor] = event.sim_time_s
  return count


def _episode_count(flags: Sequence[bool]) -> int:
  count = 0
  previous = False
  for flag in flags:
    if flag and not previous:
      count += 1
    previous = flag
  return count


def _moving_average(values: Sequence[float], window: int) -> list[float]:
  if window <= 1:
    return list(values)
  output: list[float] = []
  running = 0.0
  for index, value in enumerate(values):
    running += value
    if index >= window:
      running -= values[index - window]
    divisor = min(index + 1, window)
    output.append(running / divisor)
  return output


def _criteria(
  metrics: dict[str, Any], thresholds: MetricThresholds
) -> tuple[dict[str, dict[str, Any]], bool, bool]:
  minimum_ttc = metrics["safety"]["min_ttc_s"]
  latency = metrics["system"]["control_latency_p95_ms"]
  checks: dict[str, dict[str, Any]] = {
    "collisions": {
      "value": metrics["safety"]["collision_count"],
      "operator": "<=",
      "threshold": thresholds.max_collisions,
      "pass": metrics["safety"]["collision_count"] <= thresholds.max_collisions,
    },
    "lane_invasions": {
      "value": metrics["safety"]["lane_invasion_count"],
      "operator": "<=",
      "threshold": thresholds.max_lane_invasions,
      "pass": metrics["safety"]["lane_invasion_count"] <= thresholds.max_lane_invasions,
    },
    "lateral_error": {
      "value": metrics["lane_keeping"]["max_abs_lateral_error_m"],
      "operator": "<=",
      "threshold": thresholds.max_abs_lateral_error_m,
      "pass": metrics["lane_keeping"]["max_abs_lateral_error_m"]
      <= thresholds.max_abs_lateral_error_m,
    },
    "offroad_duration": {
      "value": metrics["safety"]["offroad_duration_s"],
      "operator": "<=",
      "threshold": thresholds.max_offroad_duration_s,
      "pass": metrics["safety"]["offroad_duration_s"] <= thresholds.max_offroad_duration_s,
    },
    "route_completion": {
      "value": metrics["mission"]["route_completion"],
      "operator": ">=",
      "threshold": thresholds.min_route_completion,
      "pass": metrics["mission"]["route_completion"] >= thresholds.min_route_completion,
    },
    "minimum_ttc": {
      "value": minimum_ttc,
      "operator": ">=",
      "threshold": thresholds.min_ttc_s,
      "pass": minimum_ttc is None or minimum_ttc >= thresholds.min_ttc_s,
    },
    "comfort_jerk": {
      "value": metrics["comfort"]["rms_jerk_mps3"],
      "operator": "<=",
      "threshold": thresholds.max_rms_jerk_mps3,
      "pass": metrics["comfort"]["rms_jerk_mps3"] <= thresholds.max_rms_jerk_mps3,
    },
    "control_latency": {
      "value": latency,
      "operator": "<=",
      "threshold": thresholds.max_p95_control_latency_ms,
      "pass": latency is None or latency <= thresholds.max_p95_control_latency_ms,
    },
    "red_light_compliance": {
      "value": metrics["compliance"]["red_light_violations"],
      "operator": "<=",
      "threshold": thresholds.max_red_light_violations,
      "pass": metrics["compliance"]["red_light_violations"] <= thresholds.max_red_light_violations,
    },
    "stop_sign_compliance": {
      "value": metrics["compliance"]["stop_sign_violations"],
      "operator": "<=",
      "threshold": thresholds.max_stop_sign_violations,
      "pass": metrics["compliance"]["stop_sign_violations"] <= thresholds.max_stop_sign_violations,
    },
    "controller_active": {
      "value": metrics["system"]["controller_active_fraction"],
      "operator": ">=",
      "threshold": thresholds.min_controller_active_fraction,
      "pass": metrics["system"]["controller_active_fraction"]
      >= thresholds.min_controller_active_fraction,
    },
    "controller_disengagements": {
      "value": metrics["system"]["controller_disengagement_count"],
      "operator": "<=",
      "threshold": thresholds.max_controller_disengagements,
      "pass": metrics["system"]["controller_disengagement_count"]
      <= thresholds.max_controller_disengagements,
    },
  }
  primary_names = {
    "collisions",
    "lane_invasions",
    "offroad_duration",
    "route_completion",
    "red_light_compliance",
    "controller_active",
    "controller_disengagements",
  }
  intervention_free_success = all(checks[name]["pass"] for name in primary_names)
  quality_pass = all(check["pass"] for check in checks.values())
  return checks, intervention_free_success, quality_pass


def compute_metrics(
  samples: Sequence[TelemetrySample],
  events: Sequence[EventRecord],
  thresholds: MetricThresholds | None = None,
) -> dict[str, Any]:
  """Compute a JSON-safe metric tree from ordered frame telemetry."""

  if not samples:
    raise ValueError("At least one telemetry sample is required")
  thresholds = thresholds or MetricThresholds()
  ordered = sorted(samples, key=lambda sample: (sample.sim_time_s, sample.frame))

  intervals: list[tuple[TelemetrySample, TelemetrySample, float]] = []
  distance_m = 0.0
  for previous, current in zip(ordered, ordered[1:], strict=False):
    dt = current.sim_time_s - previous.sim_time_s
    if dt <= 0:
      raise ValueError("Telemetry timestamps must be strictly increasing")
    if current.frame <= previous.frame:
      raise ValueError("Telemetry frame numbers must be strictly increasing")
    intervals.append((previous, current, dt))
    distance_m += math.hypot(current.x_m - previous.x_m, current.y_m - previous.y_m)
  deltas = [interval[2] for interval in intervals]
  nominal_dt = _percentile(deltas, 50) or 0.0
  duration_s = max(0.0, ordered[-1].sim_time_s - ordered[0].sim_time_s)

  speed_errors = [sample.speed_mps - sample.target_speed_mps for sample in ordered]
  lateral_errors = [sample.lane_offset_m for sample in ordered]
  longitudinal_acceleration = [sample.acceleration_mps2 for sample in ordered]
  lateral_acceleration = [sample.lateral_acceleration_mps2 for sample in ordered]
  # A fixed 0.5 s moving-average filter makes the jerk calculation less
  # sensitive to simulator quantization while retaining emergency transients.
  window = max(1, round(0.5 / nominal_dt)) if nominal_dt else 1
  filtered_acceleration = _moving_average(longitudinal_acceleration, window)
  jerk: list[float] = []
  for index, (_, _, dt) in enumerate(intervals):
    jerk.append((filtered_acceleration[index + 1] - filtered_acceleration[index]) / dt)

  ttc_values = [
    sample.ttc_s for sample in ordered if sample.ttc_s is not None and sample.ttc_s >= 0
  ]
  latency_values = [
    sample.control_latency_ms
    for sample in ordered
    if sample.control_latency_ms is not None and sample.control_latency_ms >= 0
  ]
  offroad_duration = sum(dt for sample, _, dt in intervals if sample.offroad)
  overspeed_duration = 0.0
  for sample, _, dt in intervals:
    limit = sample.speed_limit_mps or sample.target_speed_mps
    if sample.speed_mps > limit + 0.5:
      overspeed_duration += dt

  collision_events = [event for event in events if event.event_type == "collision"]
  collision_impulses = [float(event.details.get("impulse_ns", 0.0)) for event in collision_events]
  collision_count = _deduplicated_collisions(events)
  active_start = next(
    (index for index, sample in enumerate(ordered) if sample.controller_active), None
  )
  evaluation_samples = ordered[active_start:] if active_start is not None else ordered
  metrics: dict[str, Any] = {
    "mission": {
      "duration_s": duration_s,
      "distance_m": distance_m,
      "route_completion": max(sample.route_completion for sample in ordered),
      "mean_speed_mps": fmean(sample.speed_mps for sample in ordered),
    },
    "safety": {
      "collision_count": collision_count,
      "collision_event_count": len(collision_events),
      "max_collision_impulse_ns": max(collision_impulses, default=0.0),
      "lane_invasion_count": _event_count(events, "lane_invasion"),
      "offroad_duration_s": offroad_duration,
      "offroad_episode_count": _episode_count([sample.offroad for sample in ordered]),
      "min_ttc_s": 0.0 if collision_count else min(ttc_values, default=None),
      "near_miss_count": _event_count(events, "near_miss"),
      "hard_braking_episodes": _episode_count(
        [sample.acceleration_mps2 < -3.0 for sample in ordered]
      ),
    },
    "lane_keeping": {
      "mean_lateral_error_m": fmean(lateral_errors),
      "mae_lateral_error_m": fmean(abs(value) for value in lateral_errors),
      "rmse_lateral_error_m": _rms(lateral_errors),
      "max_abs_lateral_error_m": max(abs(value) for value in lateral_errors),
    },
    "speed_tracking": {
      "mean_error_mps": fmean(speed_errors),
      "mae_mps": fmean(abs(value) for value in speed_errors),
      "rmse_mps": _rms(speed_errors),
      "overspeed_duration_s": overspeed_duration,
    },
    "comfort": {
      "rms_longitudinal_acceleration_mps2": _rms(longitudinal_acceleration),
      "max_abs_longitudinal_acceleration_mps2": max(
        abs(value) for value in longitudinal_acceleration
      ),
      "rms_lateral_acceleration_mps2": _rms(lateral_acceleration),
      "max_abs_lateral_acceleration_mps2": max(abs(value) for value in lateral_acceleration),
      "rms_jerk_mps3": _rms(jerk),
      "p95_abs_jerk_mps3": _percentile([abs(value) for value in jerk], 95) or 0.0,
    },
    "compliance": {
      "red_light_violations": _event_count(events, "red_light_violation"),
      "stop_sign_violations": _event_count(events, "stop_sign_violation"),
      "speeding_events": _episode_count(
        [
          sample.speed_mps > (sample.speed_limit_mps or sample.target_speed_mps) + 0.5
          for sample in ordered
        ]
      ),
    },
    "system": {
      "sample_count": len(ordered),
      "evaluation_sample_count": len(evaluation_samples),
      "nominal_dt_s": nominal_dt,
      "controller_active_fraction": fmean(
        1.0 if sample.controller_active else 0.0 for sample in evaluation_samples
      ),
      "controller_disengagement_count": _event_count(events, "controller_disengagement"),
      "control_latency_p50_ms": _percentile(latency_values, 50),
      "control_latency_p95_ms": _percentile(latency_values, 95),
      "control_latency_max_ms": max(latency_values, default=None),
    },
  }
  criteria, success, quality_pass = _criteria(metrics, thresholds)
  metrics["criteria"] = criteria
  metrics["intervention_free_success"] = success
  metrics["quality_pass"] = quality_pass
  metrics["success"] = success
  return metrics
