"""Runtime-only CARLA, camera, actuation, and recording configuration."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .errors import ConfigurationError


@dataclass(frozen=True)
class CameraConfig:
  width: int
  height: int
  road_fov_deg: float
  wide_fov_deg: float
  sensor_tick_s: float
  gamma: float
  mount: dict[str, float]


@dataclass(frozen=True)
class ActuationConfig:
  steer_sign: float
  steering_ratio: float
  fallback_max_wheel_angle_deg: float
  calibration_profile: Path | None = None


@dataclass(frozen=True)
class RecordingConfig:
  carla_recorder: bool


@dataclass(frozen=True)
class SpectatorConfig:
  enabled: bool
  distance_m: float
  height_m: float
  pitch_deg: float
  smoothing_time_s: float


@dataclass(frozen=True)
class DashboardConfig:
  enabled: bool
  host: str
  port: int
  frame_stride: int
  downsample: int
  jpeg_quality: int


@dataclass(frozen=True)
class CarlaRuntimeConfig:
  required_version: str
  host: str
  port: int
  timeout_s: float
  traffic_manager_port: int
  fixed_delta_seconds: float
  substepping: bool
  max_substep_delta_time: float
  max_substeps: int
  quality_level: str
  ego_blueprint: str
  role_name: str
  camera: CameraConfig
  actuation: ActuationConfig
  recording: RecordingConfig
  spectator: SpectatorConfig
  dashboard: DashboardConfig
  source_path: Path


def _section(data: dict[str, Any], name: str) -> dict[str, Any]:
  value = data.get(name)
  if not isinstance(value, dict):
    raise ConfigurationError(f"config section {name!r} must be a mapping")
  return value


def _optional_section(data: dict[str, Any], name: str) -> dict[str, Any]:
  value = data.get(name, {})
  if not isinstance(value, dict):
    raise ConfigurationError(f"config section {name!r} must be a mapping")
  return value


def _reject_unknown(data: dict[str, Any], allowed: set[str], label: str) -> None:
  unknown = set(data) - allowed
  if unknown:
    raise ConfigurationError(f"Unknown {label} key(s): {', '.join(sorted(unknown))}")


def _number(value: Any, label: str, *, positive: bool = False) -> float:
  if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
    raise ConfigurationError(f"{label} must be a finite number")
  result = float(value)
  if positive and result <= 0:
    raise ConfigurationError(f"{label} must be positive")
  return result


def _integer(value: Any, label: str, *, minimum: int = 1) -> int:
  if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
    raise ConfigurationError(f"{label} must be an integer >= {minimum}")
  return value


def _boolean(value: Any, label: str) -> bool:
  if not isinstance(value, bool):
    raise ConfigurationError(f"{label} must be boolean")
  return value


def _environment_port(name: str, fallback: int) -> int:
  raw = os.getenv(name)
  if raw is None:
    return fallback
  try:
    value = int(raw)
  except ValueError as exc:
    raise ConfigurationError(f"{name} must be an integer TCP port") from exc
  if not 1 <= value <= 65535:
    raise ConfigurationError(f"{name} must be in [1, 65535]")
  return value


def _environment_boolean(name: str, fallback: bool) -> bool:
  raw = os.getenv(name)
  if raw is None:
    return fallback
  normalized = raw.strip().lower()
  if normalized in {"1", "true", "yes", "on"}:
    return True
  if normalized in {"0", "false", "no", "off"}:
    return False
  raise ConfigurationError(f"{name} must be a boolean (0/1, false/true, no/yes, off/on)")


def load_runtime_config(path: str | Path) -> CarlaRuntimeConfig:
  source = Path(path).expanduser().resolve()
  if not source.is_file():
    raise ConfigurationError(f"Runtime configuration does not exist: {source}")
  try:
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
  except yaml.YAMLError as exc:
    raise ConfigurationError(f"Invalid runtime YAML in {source}: {exc}") from exc
  if not isinstance(raw, dict) or raw.get("schema_version") != 1:
    raise ConfigurationError(f"{source} must use schema_version: 1")
  _reject_unknown(raw, {"schema_version", "carla"}, f"root of {source}")
  data = _section(raw, "carla")
  _reject_unknown(
    data,
    {
      "required_version",
      "host",
      "port",
      "timeout_s",
      "traffic_manager_port",
      "fixed_delta_seconds",
      "substepping",
      "max_substep_delta_time",
      "max_substeps",
      "quality_level",
      "ego_blueprint",
      "role_name",
      "camera",
      "actuation",
      "recording",
      "spectator",
      "dashboard",
    },
    "carla",
  )
  camera = _section(data, "camera")
  _reject_unknown(
    camera,
    {"width", "height", "road_fov_deg", "wide_fov_deg", "sensor_tick_s", "gamma", "mount"},
    "carla.camera",
  )
  mount = _section(camera, "mount")
  _reject_unknown(mount, {"x", "y", "z", "pitch", "yaw", "roll"}, "carla.camera.mount")
  actuation = _section(data, "actuation")
  _reject_unknown(
    actuation,
    {"steer_sign", "steering_ratio", "fallback_max_wheel_angle_deg", "calibration_profile"},
    "carla.actuation",
  )
  recording = _section(data, "recording")
  _reject_unknown(recording, {"carla_recorder"}, "carla.recording")
  spectator = _optional_section(data, "spectator")
  _reject_unknown(
    spectator,
    {"enabled", "distance_m", "height_m", "pitch_deg", "smoothing_time_s"},
    "carla.spectator",
  )
  dashboard = _optional_section(data, "dashboard")
  _reject_unknown(
    dashboard,
    {"enabled", "host", "port", "frame_stride", "downsample", "jpeg_quality"},
    "carla.dashboard",
  )

  required_version = data.get("required_version", "0.9.16")
  host = data.get("host", "127.0.0.1")
  quality_level = data.get("quality_level", "Epic")
  ego_blueprint = data.get("ego_blueprint", "vehicle.lincoln.mkz_2020")
  role_name = data.get("role_name", "ego_vehicle")
  for value, label in (
    (required_version, "carla.required_version"),
    (host, "carla.host"),
    (quality_level, "carla.quality_level"),
    (ego_blueprint, "carla.ego_blueprint"),
    (role_name, "carla.role_name"),
  ):
    if not isinstance(value, str) or not value.strip():
      raise ConfigurationError(f"{label} must be a non-empty string")

  port = _integer(data.get("port", 2000), "carla.port")
  traffic_manager_port = _integer(
    data.get("traffic_manager_port", 8000), "carla.traffic_manager_port"
  )
  if port > 65535 or traffic_manager_port > 65535:
    raise ConfigurationError("CARLA ports must be in [1, 65535]")

  dashboard_host = dashboard.get("host", "127.0.0.1")
  if not isinstance(dashboard_host, str):
    raise ConfigurationError("carla.dashboard.host must be a non-empty string")

  result = CarlaRuntimeConfig(
    required_version=required_version.strip(),
    host=os.getenv("CARLA_HOST", host).strip(),
    port=_environment_port("CARLA_PORT", port),
    timeout_s=_number(data.get("timeout_s", 20), "carla.timeout_s", positive=True),
    traffic_manager_port=traffic_manager_port,
    fixed_delta_seconds=_number(
      data.get("fixed_delta_seconds", 0.05), "carla.fixed_delta_seconds", positive=True
    ),
    substepping=_boolean(data.get("substepping", True), "carla.substepping"),
    max_substep_delta_time=_number(
      data.get("max_substep_delta_time", 0.01),
      "carla.max_substep_delta_time",
      positive=True,
    ),
    max_substeps=_integer(data.get("max_substeps", 10), "carla.max_substeps"),
    quality_level=quality_level.strip(),
    ego_blueprint=ego_blueprint.strip(),
    role_name=role_name.strip(),
    camera=CameraConfig(
      width=_integer(camera.get("width", 1928), "carla.camera.width"),
      height=_integer(camera.get("height", 1208), "carla.camera.height"),
      road_fov_deg=_number(camera.get("road_fov_deg", 40), "carla.camera.road_fov_deg"),
      wide_fov_deg=_number(camera.get("wide_fov_deg", 120), "carla.camera.wide_fov_deg"),
      sensor_tick_s=_number(
        camera.get("sensor_tick_s", 0.05), "carla.camera.sensor_tick_s", positive=True
      ),
      gamma=_number(camera.get("gamma", 2.2), "carla.camera.gamma", positive=True),
      mount={
        key: _number(mount.get(key, 0.0), f"carla.camera.mount.{key}")
        for key in ("x", "y", "z", "pitch", "yaw", "roll")
      },
    ),
    actuation=ActuationConfig(
      calibration_profile=(source.parent / str(actuation["calibration_profile"])).resolve()
      if actuation.get("calibration_profile")
      else None,
      steer_sign=_number(actuation.get("steer_sign", -1.0), "carla.actuation.steer_sign"),
      steering_ratio=_number(
        actuation.get("steering_ratio", 15.0),
        "carla.actuation.steering_ratio",
        positive=True,
      ),
      fallback_max_wheel_angle_deg=_number(
        actuation.get("fallback_max_wheel_angle_deg", 70.0),
        "carla.actuation.fallback_max_wheel_angle_deg",
        positive=True,
      ),
    ),
    recording=RecordingConfig(
      carla_recorder=_boolean(
        recording.get("carla_recorder", False), "carla.recording.carla_recorder"
      ),
    ),
    spectator=SpectatorConfig(
      enabled=_environment_boolean(
        "OPENCARLA_SPECTATOR", _boolean(spectator.get("enabled", True), "carla.spectator.enabled")
      ),
      distance_m=_number(
        spectator.get("distance_m", 7.0), "carla.spectator.distance_m", positive=True
      ),
      height_m=_number(spectator.get("height_m", 3.0), "carla.spectator.height_m", positive=True),
      pitch_deg=_number(spectator.get("pitch_deg", -15.0), "carla.spectator.pitch_deg"),
      smoothing_time_s=_number(
        spectator.get("smoothing_time_s", 0.12), "carla.spectator.smoothing_time_s"
      ),
    ),
    dashboard=DashboardConfig(
      enabled=_environment_boolean(
        "OPENCARLA_DASHBOARD",
        _boolean(dashboard.get("enabled", False), "carla.dashboard.enabled"),
      ),
      host=dashboard_host.strip(),
      port=_environment_port(
        "OPENCARLA_DASHBOARD_PORT",
        _integer(dashboard.get("port", 8765), "carla.dashboard.port"),
      ),
      frame_stride=_integer(dashboard.get("frame_stride", 4), "carla.dashboard.frame_stride"),
      downsample=_integer(dashboard.get("downsample", 3), "carla.dashboard.downsample"),
      jpeg_quality=_integer(dashboard.get("jpeg_quality", 82), "carla.dashboard.jpeg_quality"),
    ),
    source_path=source,
  )
  if not result.host:
    raise ConfigurationError("CARLA_HOST/carla.host must be non-empty")
  if result.quality_level != "Epic":
    raise ConfigurationError("The preregistered evaluation requires carla.quality_level: Epic")
  if not math.isclose(result.fixed_delta_seconds, 0.05, abs_tol=1e-12):
    raise ConfigurationError("OpenPilot v0.11.1 adapter requires fixed_delta_seconds: 0.05")
  if result.substepping and result.fixed_delta_seconds > (
    result.max_substep_delta_time * result.max_substeps + 1e-12
  ):
    raise ConfigurationError("fixed_delta_seconds must be <= max_substep_delta_time * max_substeps")
  if result.camera.width != 1928 or result.camera.height != 1208:
    raise ConfigurationError("OpenPilot v0.11.1 sim camerad requires camera resolution 1928x1208")
  if not math.isclose(result.camera.sensor_tick_s, 0.05, abs_tol=1e-12):
    raise ConfigurationError("OpenPilot frame matching requires camera.sensor_tick_s: 0.05")
  if not 1.0 <= result.camera.road_fov_deg < 180.0:
    raise ConfigurationError("carla.camera.road_fov_deg must be in [1, 180)")
  if not 1.0 <= result.camera.wide_fov_deg < 180.0:
    raise ConfigurationError("carla.camera.wide_fov_deg must be in [1, 180)")
  if result.actuation.steer_sign not in {-1.0, 1.0}:
    raise ConfigurationError("carla.actuation.steer_sign must be -1 or 1")
  if not -90.0 < result.spectator.pitch_deg < 90.0:
    raise ConfigurationError("carla.spectator.pitch_deg must be strictly between -90 and 90")
  if result.spectator.smoothing_time_s < 0:
    raise ConfigurationError("carla.spectator.smoothing_time_s must be non-negative")
  if not result.dashboard.host:
    raise ConfigurationError("carla.dashboard.host must be non-empty")
  if result.dashboard.port > 65535:
    raise ConfigurationError("carla.dashboard.port must be in [1, 65535]")
  if not 1 <= result.dashboard.jpeg_quality <= 95:
    raise ConfigurationError("carla.dashboard.jpeg_quality must be in [1, 95]")
  return result
