"""Typed domain models shared by the runner, recorder, and analysis code."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

SUPPORTED_SCENARIO_KINDS = frozenset(
  {
    "lane_following",
    "lead_vehicle_braking",
    "pedestrian_crossing",
    "vehicle_cut_in",
    "signalized_intersection",
    "parked_vehicle_obstruction",
    "adverse_weather_lane_following",
  }
)

SUPPORTED_CONTROLLERS = frozenset({"openpilot", "traffic_manager", "synthetic"})


@dataclass(frozen=True)
class MetricThresholds:
  """Pass/fail thresholds applied to one completed run."""

  max_collisions: int = 0
  max_lane_invasions: int = 0
  max_abs_lateral_error_m: float = 1.00
  max_offroad_duration_s: float = 0.10
  min_route_completion: float = 0.99
  min_ttc_s: float = 1.00
  max_rms_jerk_mps3: float = 5.00
  max_p95_control_latency_ms: float = 100.0
  max_red_light_violations: int = 0
  max_stop_sign_violations: int = 0
  max_controller_disengagements: int = 0
  min_controller_active_fraction: float = 0.99

  def __post_init__(self) -> None:
    integer_fields = (
      "max_collisions",
      "max_lane_invasions",
      "max_red_light_violations",
      "max_stop_sign_violations",
      "max_controller_disengagements",
    )
    for name in integer_fields:
      value = getattr(self, name)
      if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    numeric_fields = (
      "max_abs_lateral_error_m",
      "max_offroad_duration_s",
      "min_ttc_s",
      "max_rms_jerk_mps3",
      "max_p95_control_latency_ms",
    )
    for name in numeric_fields:
      value = getattr(self, name)
      if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0
      ):
        raise ValueError(f"{name} must be a non-negative number")
    for name in ("min_route_completion", "min_controller_active_fraction"):
      value = getattr(self, name)
      if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or not 0 <= value <= 1
      ):
        raise ValueError(f"{name} must be in [0, 1]")

  @classmethod
  def from_mapping(cls, values: dict[str, Any] | None) -> MetricThresholds:
    values = values or {}
    unknown = set(values) - set(cls.__dataclass_fields__)
    if unknown:
      names = ", ".join(sorted(unknown))
      raise ValueError(f"Unknown metric threshold(s): {names}")
    return cls(**values)

  def to_dict(self) -> dict[str, Any]:
    return asdict(self)


@dataclass(frozen=True)
class ScenarioSpec:
  """One parameterized urban-driving scenario."""

  id: str
  name: str
  kind: str
  description: str
  map_name: str
  duration_s: float
  route_length_m: float
  target_speed_mps: float
  parameters: dict[str, Any] = field(default_factory=dict)
  thresholds: MetricThresholds = field(default_factory=MetricThresholds)
  source_path: Path | None = None

  def to_dict(self) -> dict[str, Any]:
    data = asdict(self)
    data["source_path"] = str(self.source_path) if self.source_path else None
    return data


@dataclass(frozen=True)
class ConditionSpec:
  """Environment and traffic condition crossed with each scenario."""

  id: str
  weather: str = "ClearNoon"
  traffic_vehicles: int = 0
  traffic_walkers: int = 0
  friction: float = 1.0
  camera_noise_std: float = 0.0

  def to_dict(self) -> dict[str, Any]:
    return asdict(self)


@dataclass(frozen=True)
class ExperimentSpec:
  """Factorial experiment definition."""

  name: str
  description: str
  scenario_paths: tuple[Path, ...]
  controllers: tuple[str, ...]
  conditions: tuple[ConditionSpec, ...]
  repetitions: int
  base_seed: int
  output_dir: Path
  source_path: Path

  @property
  def run_count(self) -> int:
    return (
      len(self.scenario_paths) * len(self.controllers) * len(self.conditions) * self.repetitions
    )


@dataclass(frozen=True)
class RunSpec:
  """Fully expanded configuration for one deterministic run."""

  experiment_name: str
  scenario: ScenarioSpec
  condition: ConditionSpec
  controller: str
  repetition: int
  seed: int
  run_id: str
  output_dir: Path
  experiment_source_path: Path | None = None
  execution_index: int | None = None
  order_seed: int | None = None

  def metadata(self, backend: str, valid_for_research: bool) -> dict[str, Any]:
    metadata = {
      "schema_version": 1,
      "experiment": self.experiment_name,
      "run_id": self.run_id,
      "controller": self.controller,
      "backend": backend,
      "valid_for_research": valid_for_research,
      "repetition": self.repetition,
      "seed": self.seed,
      "execution_index": self.execution_index,
      "order_seed": self.order_seed,
      "scenario": self.scenario.to_dict(),
      "condition": self.condition.to_dict(),
    }
    metadata["condition_sha256"] = hashlib.sha256(
      json.dumps(self.condition.to_dict(), sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    if self.scenario.source_path is not None and self.scenario.source_path.is_file():
      metadata["scenario_sha256"] = hashlib.sha256(
        self.scenario.source_path.read_bytes()
      ).hexdigest()
    if self.experiment_source_path is not None and self.experiment_source_path.is_file():
      metadata["experiment_sha256"] = hashlib.sha256(
        self.experiment_source_path.read_bytes()
      ).hexdigest()
    return metadata


@dataclass(frozen=True)
class TelemetrySample:
  """One frame-aligned ego-vehicle telemetry record."""

  frame: int
  sim_time_s: float
  x_m: float
  y_m: float
  z_m: float
  yaw_deg: float
  speed_mps: float
  target_speed_mps: float
  acceleration_mps2: float
  lateral_acceleration_mps2: float
  steering_angle_deg: float
  throttle: float
  brake: float
  lane_offset_m: float
  route_completion: float
  controller_active: bool
  offroad: bool = False
  nearest_actor_distance_m: float | None = None
  closing_speed_mps: float | None = None
  ttc_s: float | None = None
  nearest_actor_id: int | str | None = None
  nearest_actor_type: str | None = None
  nearest_actor_speed_mps: float | None = None
  nearest_actor_x_m: float | None = None
  nearest_actor_y_m: float | None = None
  traffic_light_state: str | None = None
  speed_limit_mps: float | None = None
  control_latency_ms: float | None = None
  commanded_acceleration_mps2: float | None = None
  commanded_steering_angle_deg: float | None = None
  controller_state: str | None = None
  controller_alert: str | None = None

  def to_dict(self) -> dict[str, Any]:
    return asdict(self)


@dataclass(frozen=True)
class EventRecord:
  """A discrete safety or scenario event."""

  event_type: str
  sim_time_s: float
  frame: int | None = None
  details: dict[str, Any] = field(default_factory=dict)

  def to_dict(self) -> dict[str, Any]:
    return asdict(self)
