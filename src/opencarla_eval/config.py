"""YAML configuration loading, validation, and experiment expansion."""

from __future__ import annotations

import math
import re
from collections.abc import Iterable
from pathlib import Path
from typing import Any

import yaml

from .errors import ConfigurationError
from .models import (
  SUPPORTED_CONTROLLERS,
  SUPPORTED_SCENARIO_KINDS,
  ConditionSpec,
  ExperimentSpec,
  MetricThresholds,
  RunSpec,
  ScenarioSpec,
)

SCHEMA_VERSION = 1
_SAFE_ID = re.compile(r"^[a-z0-9][a-z0-9_-]*$")

_COMMON_PARAMETER_KEYS = {
  "warmup_active_s",
  "terminate_on_collision",
  "startup_target_speed_mps",
  "startup_max_throttle",
  "stuck_timeout_s",
}
_SCENARIO_PARAMETER_KEYS: dict[str, set[str]] = {
  "lane_following": {"require_heading_change_deg", "minimum_junction_clearance_m"},
  "adverse_weather_lane_following": {
    "require_heading_change_deg",
    "minimum_junction_clearance_m",
  },
  "lead_vehicle_braking": {
    "lead_initial_gap_m",
    "lead_speed_mps",
    "lead_brake_command",
    "trigger_ttc_s",
    "trigger_min_time_s",
    "fallback_trigger_s",
    "lead_stop_dwell_s",
    "min_observed_lead_deceleration_mps2",
  },
  "pedestrian_crossing": {
    "crossing_distance_m",
    "pedestrian_speed_mps",
    "trigger_ttc_s",
    "trigger_min_time_s",
    "fallback_trigger_s",
    "start_side",
    "min_pedestrian_travel_m",
  },
  "vehicle_cut_in": {
    "intruder_gap_m",
    "intruder_speed_mps",
    "trigger_ttc_s",
    "trigger_min_time_s",
    "fallback_trigger_s",
    "preferred_side",
    "max_cut_in_lateral_separation_m",
  },
  "signalized_intersection": {"signal_sight_distance_m", "red_hold_s"},
  "parked_vehicle_obstruction": {"obstacle_distance_m", "obstacle_lateral_offset_m"},
}


def _read_yaml(path: Path) -> dict[str, Any]:
  if not path.is_file():
    raise ConfigurationError(f"Configuration file does not exist: {path}")
  try:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
  except yaml.YAMLError as exc:
    raise ConfigurationError(f"Invalid YAML in {path}: {exc}") from exc
  if not isinstance(raw, dict):
    raise ConfigurationError(f"Expected a YAML mapping at the root of {path}")
  version = raw.get("schema_version")
  if version != SCHEMA_VERSION:
    raise ConfigurationError(
      f"Unsupported schema_version {version!r} in {path}; expected {SCHEMA_VERSION}"
    )
  return raw


def _reject_unknown(data: dict[str, Any], allowed: set[str], label: str) -> None:
  unknown = set(data) - allowed
  if unknown:
    raise ConfigurationError(f"Unknown {label} key(s): {', '.join(sorted(unknown))}")


def _mapping(value: Any, label: str) -> dict[str, Any]:
  if not isinstance(value, dict):
    raise ConfigurationError(f"{label} must be a mapping")
  return value


def _nonempty_string(value: Any, label: str) -> str:
  if not isinstance(value, str) or not value.strip():
    raise ConfigurationError(f"{label} must be a non-empty string")
  return value.strip()


def _identifier(value: Any, label: str) -> str:
  result = _nonempty_string(value, label)
  if not _SAFE_ID.fullmatch(result):
    raise ConfigurationError(
      f"{label} must contain only lowercase letters, digits, '_' or '-': {result!r}"
    )
  return result


def _positive_number(value: Any, label: str) -> float:
  if (
    isinstance(value, bool)
    or not isinstance(value, (int, float))
    or not math.isfinite(value)
    or value <= 0
  ):
    raise ConfigurationError(f"{label} must be a positive number")
  return float(value)


def _nonnegative_number(value: Any, label: str) -> float:
  if (
    isinstance(value, bool)
    or not isinstance(value, (int, float))
    or not math.isfinite(value)
    or value < 0
  ):
    raise ConfigurationError(f"{label} must be a non-negative number")
  return float(value)


def _nonnegative_int(value: Any, label: str) -> int:
  if isinstance(value, bool) or not isinstance(value, int) or value < 0:
    raise ConfigurationError(f"{label} must be a non-negative integer")
  return value


def _validate_parameters(kind: str, parameters: dict[str, Any]) -> dict[str, Any]:
  allowed = _COMMON_PARAMETER_KEYS | _SCENARIO_PARAMETER_KEYS[kind]
  _reject_unknown(parameters, allowed, f"scenario.parameters for {kind}")
  result = dict(parameters)

  positive = {
    "warmup_active_s",
    "lead_initial_gap_m",
    "lead_speed_mps",
    "trigger_ttc_s",
    "fallback_trigger_s",
    "crossing_distance_m",
    "pedestrian_speed_mps",
    "intruder_gap_m",
    "intruder_speed_mps",
    "signal_sight_distance_m",
    "red_hold_s",
    "obstacle_distance_m",
    "min_observed_lead_deceleration_mps2",
    "min_pedestrian_travel_m",
    "max_cut_in_lateral_separation_m",
    "startup_target_speed_mps",
    "startup_max_throttle",
    "stuck_timeout_s",
  }
  nonnegative = {
    "require_heading_change_deg",
    "minimum_junction_clearance_m",
    "trigger_min_time_s",
    "lead_stop_dwell_s",
  }
  for key in positive & result.keys():
    result[key] = _positive_number(result[key], f"scenario.parameters.{key}")
  for key in nonnegative & result.keys():
    result[key] = _nonnegative_number(result[key], f"scenario.parameters.{key}")
  if "obstacle_lateral_offset_m" in result:
    value = result["obstacle_lateral_offset_m"]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
      raise ConfigurationError("scenario.parameters.obstacle_lateral_offset_m must be finite")
    result["obstacle_lateral_offset_m"] = float(value)
  if "terminate_on_collision" in result and not isinstance(result["terminate_on_collision"], bool):
    raise ConfigurationError("scenario.parameters.terminate_on_collision must be boolean")
  if "startup_max_throttle" in result and result["startup_max_throttle"] > 1.0:
    raise ConfigurationError("scenario.parameters.startup_max_throttle must be <= 1")
  for key in ("start_side", "preferred_side"):
    if key in result and result[key] not in {"left", "right"}:
      raise ConfigurationError(f"scenario.parameters.{key} must be 'left' or 'right'")
  if "lead_brake_command" in result and result["lead_brake_command"] != "maximum":
    raise ConfigurationError("scenario.parameters.lead_brake_command must be 'maximum'")
  return result


def load_scenario(path: str | Path) -> ScenarioSpec:
  source = Path(path).expanduser().resolve()
  raw = _read_yaml(source)
  _reject_unknown(raw, {"schema_version", "scenario"}, f"root of {source}")
  data = _mapping(raw.get("scenario"), f"scenario in {source}")
  _reject_unknown(
    data,
    {
      "id",
      "name",
      "kind",
      "description",
      "map",
      "duration_s",
      "route_length_m",
      "target_speed_mps",
      "parameters",
      "thresholds",
    },
    f"scenario in {source}",
  )

  scenario_id = _identifier(data.get("id"), "scenario.id")
  kind = _identifier(data.get("kind"), "scenario.kind")
  if kind not in SUPPORTED_SCENARIO_KINDS:
    supported = ", ".join(sorted(SUPPORTED_SCENARIO_KINDS))
    raise ConfigurationError(f"Unsupported scenario.kind {kind!r}; choose one of: {supported}")

  parameters = data.get("parameters", {})
  if not isinstance(parameters, dict):
    raise ConfigurationError("scenario.parameters must be a mapping")
  parameters = _validate_parameters(kind, parameters)

  threshold_values = data.get("thresholds", {})
  if not isinstance(threshold_values, dict):
    raise ConfigurationError("scenario.thresholds must be a mapping")
  try:
    thresholds = MetricThresholds.from_mapping(threshold_values)
  except (TypeError, ValueError) as exc:
    raise ConfigurationError(f"Invalid thresholds in {source}: {exc}") from exc

  target_speed_mps = _positive_number(data.get("target_speed_mps"), "scenario.target_speed_mps")
  if not math.isclose(target_speed_mps, 40.0 / 3.6, abs_tol=0.02):
    raise ConfigurationError(
      "The pinned OpenPilot bridge initializes vCruise to 40 km/h; "
      "scenario.target_speed_mps must be 11.11 until explicit set-speed control is added"
    )

  return ScenarioSpec(
    id=scenario_id,
    name=_nonempty_string(data.get("name"), "scenario.name"),
    kind=kind,
    description=_nonempty_string(data.get("description"), "scenario.description"),
    map_name=_nonempty_string(data.get("map", "Town10HD_Opt"), "scenario.map"),
    duration_s=_positive_number(data.get("duration_s"), "scenario.duration_s"),
    route_length_m=_positive_number(data.get("route_length_m"), "scenario.route_length_m"),
    target_speed_mps=target_speed_mps,
    parameters=parameters,
    thresholds=thresholds,
    source_path=source,
  )


def _load_condition(raw: Any, index: int) -> ConditionSpec:
  data = _mapping(raw, f"experiment.conditions[{index}]")
  known = {
    "id",
    "weather",
    "traffic_vehicles",
    "traffic_walkers",
    "friction",
    "camera_noise_std",
  }
  _reject_unknown(data, known, f"experiment.conditions[{index}]")
  condition_id = _identifier(data.get("id"), f"experiment.conditions[{index}].id")
  friction = data.get("friction", 1.0)
  if (
    isinstance(friction, bool)
    or not isinstance(friction, (int, float))
    or not math.isfinite(friction)
    or friction <= 0
  ):
    raise ConfigurationError(f"condition {condition_id}: friction must be positive")
  camera_noise = data.get("camera_noise_std", 0.0)
  if (
    isinstance(camera_noise, bool)
    or not isinstance(camera_noise, (int, float))
    or camera_noise < 0
    or not math.isfinite(camera_noise)
  ):
    raise ConfigurationError(f"condition {condition_id}: camera_noise_std must be non-negative")
  traffic_walkers = _nonnegative_int(data.get("traffic_walkers", 0), "condition.traffic_walkers")
  if traffic_walkers:
    raise ConfigurationError(
      f"condition {condition_id}: background walkers are not implemented; use traffic_walkers: 0"
    )
  return ConditionSpec(
    id=condition_id,
    weather=_nonempty_string(data.get("weather", "ClearNoon"), "condition.weather"),
    traffic_vehicles=_nonnegative_int(
      data.get("traffic_vehicles", 0), "condition.traffic_vehicles"
    ),
    traffic_walkers=traffic_walkers,
    friction=float(friction),
    camera_noise_std=float(camera_noise),
  )


def load_experiment(path: str | Path) -> ExperimentSpec:
  source = Path(path).expanduser().resolve()
  raw = _read_yaml(source)
  _reject_unknown(raw, {"schema_version", "experiment"}, f"root of {source}")
  data = _mapping(raw.get("experiment"), f"experiment in {source}")
  _reject_unknown(
    data,
    {
      "name",
      "description",
      "scenarios",
      "controllers",
      "conditions",
      "repetitions",
      "base_seed",
      "output_dir",
    },
    f"experiment in {source}",
  )
  experiment_id = _identifier(data.get("name"), "experiment.name")

  raw_scenarios = data.get("scenarios")
  if not isinstance(raw_scenarios, list) or not raw_scenarios:
    raise ConfigurationError("experiment.scenarios must be a non-empty list")
  scenario_paths = tuple((source.parent / str(item)).resolve() for item in raw_scenarios)
  loaded_ids: set[str] = set()
  for scenario_path in scenario_paths:
    scenario = load_scenario(scenario_path)
    if scenario.id in loaded_ids:
      raise ConfigurationError(f"Duplicate scenario id {scenario.id!r} in experiment")
    loaded_ids.add(scenario.id)

  controllers_raw = data.get("controllers")
  if not isinstance(controllers_raw, list) or not controllers_raw:
    raise ConfigurationError("experiment.controllers must be a non-empty list")
  controllers = tuple(_identifier(item, "experiment.controllers item") for item in controllers_raw)
  unsupported = set(controllers) - SUPPORTED_CONTROLLERS
  if unsupported:
    raise ConfigurationError(f"Unsupported controller(s): {', '.join(sorted(unsupported))}")
  if len(set(controllers)) != len(controllers):
    raise ConfigurationError("experiment.controllers must not contain duplicates")

  conditions_raw = data.get("conditions")
  if not isinstance(conditions_raw, list) or not conditions_raw:
    raise ConfigurationError("experiment.conditions must be a non-empty list")
  conditions = tuple(_load_condition(item, index) for index, item in enumerate(conditions_raw))
  condition_ids = [item.id for item in conditions]
  if len(set(condition_ids)) != len(condition_ids):
    raise ConfigurationError("experiment.conditions must have unique ids")

  repetitions = _nonnegative_int(data.get("repetitions"), "experiment.repetitions")
  if repetitions < 1:
    raise ConfigurationError("experiment.repetitions must be at least 1")
  base_seed = _nonnegative_int(data.get("base_seed", 1000), "experiment.base_seed")

  output_value = _nonempty_string(data.get("output_dir"), "experiment.output_dir")
  output_dir = (source.parent / output_value).resolve()
  return ExperimentSpec(
    name=experiment_id,
    description=_nonempty_string(data.get("description"), "experiment.description"),
    scenario_paths=scenario_paths,
    controllers=controllers,
    conditions=conditions,
    repetitions=repetitions,
    base_seed=base_seed,
    output_dir=output_dir,
    source_path=source,
  )


def expand_runs(experiment: ExperimentSpec) -> Iterable[RunSpec]:
  """Expand the factorial matrix in stable order, preserving paired seeds."""

  scenarios = [load_scenario(path) for path in experiment.scenario_paths]
  for scenario in scenarios:
    for condition in experiment.conditions:
      for repetition in range(experiment.repetitions):
        seed = experiment.base_seed + repetition
        controllers = (
          experiment.controllers if repetition % 2 == 0 else tuple(reversed(experiment.controllers))
        )
        for controller in controllers:
          run_id = f"{scenario.id}__{condition.id}__{controller}__r{repetition + 1:02d}__s{seed}"
          yield RunSpec(
            experiment_name=experiment.name,
            scenario=scenario,
            condition=condition,
            controller=controller,
            repetition=repetition + 1,
            seed=seed,
            run_id=run_id,
            output_dir=experiment.output_dir / run_id,
            experiment_source_path=experiment.source_path,
          )
