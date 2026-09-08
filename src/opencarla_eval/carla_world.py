"""CARLA world adapter for OpenPilot's current simulator bridge interface."""

from __future__ import annotations

import hashlib
import json
import math
import multiprocessing
import os
import platform
import re
import subprocess
import threading
import time
import weakref
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from .actuation import ActuatorCalibration, CalibratedActuatorController
from .carla_scenarios import CarlaScenarioRuntime, build_reference_route, select_spawn
from .demo_capture import DemoCapture
from .errors import DependencyUnavailableError, SimulatorConnectionError
from .live_dashboard import LiveDashboard, build_openpilot_dashboard_state
from .models import EventRecord, RunSpec, TelemetrySample
from .recorder import RunRecorder
from .runtime_config import CarlaRuntimeConfig

_POST_RESET_SETTLING_TICKS = 2
_POST_RESET_REQUIRED_SANE_IMU_TICKS = 2
_POST_RESET_READY_TIMEOUT_S = 30.0
_POST_RESET_ACCEL_NORM_MIN_MPS2 = 5.0
_POST_RESET_ACCEL_NORM_MAX_MPS2 = 100.0
_POST_RESET_GYRO_NORM_MAX_RAD_S = 10.0
_POST_RESET_VERTICAL_SPEED_MAX_MPS = 0.5
_ROUTE_RESERVE_M = 10.0
_EVALUATION_ORIGIN_MAX_HEADING_ERROR_DEG = 10.0
_EVALUATION_ORIGIN_MAX_LATERAL_OFFSET_M = 0.75

_RECORDER_FRAME_RE = re.compile(r"^Frame\s+(\d+)\s+at\s+")
_RECORDER_COLLISION_RE = re.compile(
  r"^\s*Collision id\s+\d+\s+between\s+(\d+)(\s+\(hero\))?"
  r"\s+with\s+(\d+)(\s+\(hero\))?\s*$"
)
_RECORDER_FRAMES_RE = re.compile(r"^Frames:\s*(\d+)\s*$")


def _weak_sensor_callback(callback: Callable[[Any], None]) -> Callable[[Any], None]:
  """Dispatch a CARLA sensor callback without retaining its world adapter."""

  callback_ref = weakref.WeakMethod(callback)

  def dispatch(value: Any) -> None:
    resolved = callback_ref()
    if resolved is not None:
      resolved(value)

  return dispatch


@dataclass(frozen=True, slots=True)
class _RecorderCollision:
  recorder_frame: int
  actor_1_id: int
  actor_2_id: int
  actor_1_is_hero: bool
  actor_2_is_hero: bool


def _parse_recorder_collisions(text: str) -> tuple[list[_RecorderCollision], int | None]:
  """Parse the stable human-readable CARLA 0.9.16 recorder query format."""

  current_frame: int | None = None
  final_frame: int | None = None
  collisions: list[_RecorderCollision] = []
  for line in text.splitlines():
    if match := _RECORDER_FRAME_RE.match(line):
      current_frame = int(match.group(1))
      continue
    if match := _RECORDER_FRAMES_RE.match(line):
      final_frame = int(match.group(1))
      continue
    if "Collision id" not in line:
      continue
    match = _RECORDER_COLLISION_RE.match(line)
    if match is None or current_frame is None:
      raise ValueError(f"Unrecognized CARLA recorder collision line: {line!r}")
    collisions.append(
      _RecorderCollision(
        recorder_frame=current_frame,
        actor_1_id=int(match.group(1)),
        actor_2_id=int(match.group(3)),
        actor_1_is_hero=match.group(2) is not None,
        actor_2_is_hero=match.group(4) is not None,
      )
    )
  return collisions, final_frame


def _enum_identity(value: Any) -> int | str:
  raw = getattr(value, "raw", value)
  try:
    return int(raw)
  except (TypeError, ValueError):
    return str(raw)


@dataclass(slots=True)
class _OpenPilotFreshnessBarrier:
  """Losslessly correlate the pinned v0.11.1 model/plan/control event graph."""

  target_frame_id: int | None = None
  maneuver_modes_disabled: bool = False
  models: dict[int, dict[str, Any]] = field(default_factory=dict)
  plans: dict[int, dict[str, Any]] = field(default_factory=dict)
  controls_states: dict[int, dict[str, Any]] = field(default_factory=dict)
  car_controls: dict[int, dict[str, Any]] = field(default_factory=dict)
  lateral_maneuver_plans: dict[int, bool] = field(default_factory=dict)
  timestamps: dict[int, str] = field(default_factory=dict)
  integrity_errors: list[str] = field(default_factory=list)
  last_evidence: dict[str, Any] = field(default_factory=dict)

  def clear(self) -> None:
    self.target_frame_id = None
    self.maneuver_modes_disabled = False
    self.models.clear()
    self.plans.clear()
    self.controls_states.clear()
    self.car_controls.clear()
    self.lateral_maneuver_plans.clear()
    self.timestamps.clear()
    self.integrity_errors.clear()
    self.last_evidence.clear()

  def arm(self, target_frame_id: int, *, maneuver_modes_disabled: bool) -> None:
    self.clear()
    self.target_frame_id = target_frame_id
    self.maneuver_modes_disabled = maneuver_modes_disabled

  def _register(self, service: str, mono_time: int) -> bool:
    if mono_time <= 0:
      self.integrity_errors.append(f"{service} has a non-positive Event.logMonoTime")
      return False
    previous = self.timestamps.get(mono_time)
    if previous is not None:
      self.integrity_errors.append(
        f"duplicate Event.logMonoTime {mono_time} for {previous} and {service}"
      )
      return False
    self.timestamps[mono_time] = service
    return True

  def ingest(self, service: str, mono_time: int, valid: bool, payload: Any) -> None:
    if self.target_frame_id is None or not self._register(service, mono_time):
      return
    if service == "modelV2":
      self.models[mono_time] = {
        "valid": valid,
        "frame_id": int(getattr(payload, "frameId", -1)),
        "frame_id_extra": int(getattr(payload, "frameIdExtra", -1)),
      }
    elif service == "longitudinalPlan":
      self.plans[mono_time] = {
        "valid": valid,
        "model_mono_time": int(getattr(payload, "modelMonoTime", 0)),
      }
    elif service == "controlsState":
      self.controls_states[mono_time] = {
        "valid": valid,
        "model_mono_time": int(getattr(payload, "lateralPlanMonoTime", 0)),
        "plan_mono_time": int(getattr(payload, "longitudinalPlanMonoTime", 0)),
        "desired_curvature": float(getattr(payload, "desiredCurvature", math.nan)),
        "long_control_state": _enum_identity(getattr(payload, "longControlState", "missing")),
      }
    elif service == "carControl":
      actuators = getattr(payload, "actuators", SimpleNamespace())
      self.car_controls[mono_time] = {
        "valid": valid,
        "enabled": bool(getattr(payload, "enabled", False)),
        "lat_active": bool(getattr(payload, "latActive", False)),
        "long_active": bool(getattr(payload, "longActive", False)),
        "curvature": float(getattr(actuators, "curvature", math.nan)),
        "long_control_state": _enum_identity(getattr(actuators, "longControlState", "missing")),
      }
    elif service == "lateralManeuverPlan":
      self.lateral_maneuver_plans[mono_time] = valid
    else:
      self.integrity_errors.append(f"unexpected causal service {service!r}")

  def evaluate(self, selected_car_control_mono_time: int) -> dict[str, Any]:
    evidence: dict[str, Any] = {
      "valid": False,
      "selected_car_control_mono_time": selected_car_control_mono_time,
      "target_camera_frame_id": self.target_frame_id,
    }

    def reject(reason: str) -> dict[str, Any]:
      evidence["reason"] = reason
      self.last_evidence = evidence
      return evidence

    if self.target_frame_id is None:
      return reject("barrier_not_armed")
    if not self.maneuver_modes_disabled:
      return reject("maneuver_or_joystick_mode_enabled")
    if self.integrity_errors:
      evidence["integrity_errors"] = list(self.integrity_errors)
      return reject("lossless_capture_integrity_error")
    control = self.car_controls.get(selected_car_control_mono_time)
    if control is None:
      return reject("selected_car_control_not_captured")

    controlsd_events = sorted(
      [(mono_time, "controlsState") for mono_time in self.controls_states]
      + [(mono_time, "carControl") for mono_time in self.car_controls]
    )
    selected_index = controlsd_events.index((selected_car_control_mono_time, "carControl"))
    captured_prefix = controlsd_events[: selected_index + 1]
    # Arming can occur between the two publications in one controlsd cycle, so
    # one leading carControl is the only permissible partial-cycle prefix.
    if captured_prefix and captured_prefix[0][1] == "carControl":
      captured_prefix = captured_prefix[1:]
    if not captured_prefix:
      return reject("controlsState_carControl_alternation_gap")
    for index, (_, service) in enumerate(captured_prefix):
      expected = "controlsState" if index % 2 == 0 else "carControl"
      if service != expected:
        return reject("controlsState_carControl_alternation_gap")
    if captured_prefix[-1] != (selected_car_control_mono_time, "carControl"):
      return reject("controlsState_carControl_alternation_gap")
    state_mono_time = captured_prefix[-2][0]
    state = self.controls_states[state_mono_time]
    evidence["controls_state_mono_time"] = state_mono_time

    if not state["valid"] or not control["valid"]:
      return reject("invalid_controls_message")
    if not (control["enabled"] and control["lat_active"] and control["long_active"]):
      return reject("selected_car_control_not_fully_active")
    if (
      not math.isclose(
        state["desired_curvature"],
        control["curvature"],
        rel_tol=1e-6,
        abs_tol=1e-7,
      )
      or state["long_control_state"] != control["long_control_state"]
    ):
      return reject("controlsState_carControl_payload_mismatch")
    if any(
      valid and mono_time <= selected_car_control_mono_time
      for mono_time, valid in self.lateral_maneuver_plans.items()
    ):
      return reject("lateral_maneuver_plan_was_valid")

    model_mono_time = int(state["model_mono_time"])
    plan_mono_time = int(state["plan_mono_time"])
    evidence["model_mono_time"] = model_mono_time
    evidence["longitudinal_plan_mono_time"] = plan_mono_time
    model = self.models.get(model_mono_time)
    plan = self.plans.get(plan_mono_time)
    if model is None or plan is None:
      return reject("referenced_model_or_plan_not_captured")
    if not model["valid"] or not plan["valid"]:
      return reject("referenced_model_or_plan_invalid")
    if not (model_mono_time < plan_mono_time < state_mono_time < selected_car_control_mono_time):
      return reject("causal_event_timestamps_not_strictly_ordered")
    if int(plan["model_mono_time"]) != model_mono_time:
      return reject("longitudinal_plan_model_mismatch")
    if (
      int(model["frame_id"]) != self.target_frame_id
      or int(model["frame_id_extra"]) != self.target_frame_id
    ):
      evidence["model_frame_id"] = int(model["frame_id"])
      evidence["model_frame_id_extra"] = int(model["frame_id_extra"])
      return reject("model_not_from_final_settling_camera_pair")

    evidence.update(
      {
        "valid": True,
        "reason": "causal_lineage_verified",
        "model_frame_id": int(model["frame_id"]),
        "model_frame_id_extra": int(model["frame_id_extra"]),
      }
    )
    self.last_evidence = evidence
    return evidence

  def evidence(self) -> dict[str, Any]:
    if self.last_evidence:
      return dict(self.last_evidence)
    return {
      "valid": False,
      "reason": "no_car_control_evaluated",
      "target_camera_frame_id": self.target_frame_id,
      "integrity_errors": list(self.integrity_errors),
    }


try:  # The real bridge uses openpilot's World; the baseline can run without it.
  from openpilot.tools.sim.lib.common import World as _OpenPilotWorld
except ImportError:  # pragma: no cover - exercised only in a CARLA-only environment

  class _OpenPilotWorld:
    def __init__(self, dual_camera: bool) -> None:
      self.dual_camera = dual_camera
      self.image_lock = multiprocessing.Semaphore(value=0)
      self.road_image = np.zeros((1208, 1928, 3), dtype=np.uint8)
      self.wide_road_image = np.zeros((1208, 1928, 3), dtype=np.uint8)
      self.exit_event = multiprocessing.Event()


class _CameraFrameHandoff:
  """Single-slot producer/consumer gate compatible with OpenPilot's camera thread.

  The pinned ``SimulatedSensors`` consumer only calls ``acquire()`` before it
  converts the shared RGB arrays.  Releasing the producer slot at the start of
  the *next* acquire guarantees that conversion of the previous arrays has
  finished before CARLA is allowed to overwrite them.
  """

  def __init__(self) -> None:
    self._ready = threading.Semaphore(0)
    self._slot = threading.Semaphore(1)
    self._state_lock = threading.Lock()
    self._consumer_holds_frame = False
    self._closed = False

  def wait_for_publish_slot(self, timeout: float) -> bool:
    if not self._slot.acquire(timeout=timeout):
      return False
    with self._state_lock:
      if self._closed:
        self._slot.release()
        return False
    return True

  def release(self) -> None:
    """Mark the producer's fully copied frame ready for the consumer."""

    self._ready.release()

  def acquire(self, blocking: bool = True, timeout: float | None = None) -> bool:
    """OpenPilot-compatible consumer acquire with implicit prior-frame ack."""

    with self._state_lock:
      if self._consumer_holds_frame:
        self._consumer_holds_frame = False
        self._slot.release()
    if timeout is None:
      acquired = self._ready.acquire(blocking)
    else:
      acquired = self._ready.acquire(blocking, timeout)
    if acquired:
      with self._state_lock:
        self._consumer_holds_frame = True
    return acquired

  def close(self) -> None:
    """Unblock either side during bridge teardown."""

    with self._state_lock:
      if self._closed:
        return
      self._closed = True
      self._slot.release()
    self._ready.release()


def _scenario_sha256(run: RunSpec) -> str | None:
  path = run.scenario.source_path
  if path is None or not path.is_file():
    return None
  return hashlib.sha256(path.read_bytes()).hexdigest()


def _file_sha256(path: Path) -> str | None:
  return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def _git_state(path: Path) -> tuple[str | None, bool | None]:
  try:
    commit = subprocess.run(
      ["git", "rev-parse", "HEAD"],
      cwd=path,
      check=True,
      capture_output=True,
      text=True,
      timeout=30,
    ).stdout.strip()
    dirty = bool(
      subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=path,
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
      ).stdout.strip()
    )
    return commit, dirty
  except (OSError, subprocess.SubprocessError):
    return None, None


def _speed(vector: Any) -> float:
  return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)


def _body_components(vector: Any, transform: Any) -> tuple[float, float, float]:
  """Resolve a world-space vector in a vehicle transform's local basis."""

  forward = transform.get_forward_vector()
  right = transform.get_right_vector()
  up = transform.get_up_vector()
  return (
    vector.x * forward.x + vector.y * forward.y + vector.z * forward.z,
    vector.x * right.x + vector.y * right.y + vector.z * right.z,
    vector.x * up.x + vector.y * up.y + vector.z * up.z,
  )


def _world_vector(carla: Any, components: tuple[float, float, float], transform: Any) -> Any:
  """Rotate local forward/right/up components into a world-space vector."""

  forward = transform.get_forward_vector()
  right = transform.get_right_vector()
  up = transform.get_up_vector()
  longitudinal, lateral, vertical = components
  return carla.Vector3D(
    x=longitudinal * forward.x + lateral * right.x + vertical * up.x,
    y=longitudinal * forward.y + lateral * right.y + vertical * up.y,
    z=longitudinal * forward.z + lateral * right.z + vertical * up.z,
  )


def _carla_accel_to_openpilot_sensor(x: float, y: float, z: float) -> tuple[float, float, float]:
  """Map CARLA forward/right/up acceleration to OpenPilot's raw IMU axes."""

  # locationd maps raw hardware vectors as [-z, -y, -x] into its
  # forward/right/down device frame. Acceleration is a polar vector.
  return z, -y, -x


def _carla_gyro_to_openpilot_sensor(x: float, y: float, z: float) -> tuple[float, float, float]:
  """Map CARLA roll/pitch/yaw rates to OpenPilot's raw IMU axes."""

  # CARLA/Unreal's handedness makes angular velocity an axial-vector case:
  # positive CARLA yaw is positive device-frame yaw about the down axis.
  return -z, -y, -x


def _chase_spectator_pose(
  ego_transform: Any, distance_m: float, height_m: float, pitch_deg: float
) -> tuple[float, float, float, float, float, float]:
  """Return a third-person pose behind the ego in CARLA world coordinates."""

  yaw_deg = float(ego_transform.rotation.yaw)
  yaw_rad = math.radians(yaw_deg)
  return (
    float(ego_transform.location.x) - distance_m * math.cos(yaw_rad),
    float(ego_transform.location.y) - distance_m * math.sin(yaw_rad),
    float(ego_transform.location.z) + height_m,
    pitch_deg,
    yaw_deg,
    0.0,
  )


def _smooth_spectator_pose(
  previous: tuple[float, float, float, float, float, float] | None,
  target: tuple[float, float, float, float, float, float],
  *,
  delta_s: float,
  smoothing_time_s: float,
  snap_distance_m: float,
) -> tuple[float, float, float, float, float, float]:
  """Exponentially smooth a chase pose while taking the shortest yaw arc."""

  if previous is None:
    return target
  displacement = math.sqrt(sum((target[index] - previous[index]) ** 2 for index in range(3)))
  if smoothing_time_s <= 0 or displacement >= snap_distance_m:
    return target
  alpha = 1.0 - math.exp(-max(0.0, delta_s) / smoothing_time_s)
  result = [previous[index] + alpha * (target[index] - previous[index]) for index in range(6)]
  yaw_delta = (target[4] - previous[4] + 180.0) % 360.0 - 180.0
  result[4] = previous[4] + alpha * yaw_delta
  return tuple(result)  # type: ignore[return-value]


def _evaluation_origin_alignment(ego_transform: Any, waypoint: Any) -> dict[str, Any]:
  """Measure whether a continuous ego pose is safe to use as a route origin."""

  location = ego_transform.location
  center = waypoint.transform.location
  right = waypoint.transform.get_right_vector()
  lateral_offset_m = (
    (location.x - center.x) * right.x
    + (location.y - center.y) * right.y
    + (location.z - center.z) * right.z
  )
  ego_heading_deg = float(ego_transform.rotation.yaw)
  lane_heading_deg = float(waypoint.transform.rotation.yaw)
  heading_error_deg = abs((ego_heading_deg - lane_heading_deg + 180.0) % 360.0 - 180.0)
  reasons = []
  if bool(waypoint.is_junction):
    reasons.append("junction")
  if heading_error_deg > _EVALUATION_ORIGIN_MAX_HEADING_ERROR_DEG:
    reasons.append("heading_error")
  if abs(lateral_offset_m) > _EVALUATION_ORIGIN_MAX_LATERAL_OFFSET_M:
    reasons.append("lateral_offset")
  return {
    "valid": not reasons,
    "reasons": reasons,
    "is_junction": bool(waypoint.is_junction),
    "ego_heading_deg": ego_heading_deg,
    "lane_heading_deg": lane_heading_deg,
    "heading_error_deg": heading_error_deg,
    "lateral_offset_m": lateral_offset_m,
    "maximum_heading_error_deg": _EVALUATION_ORIGIN_MAX_HEADING_ERROR_DEG,
    "maximum_lateral_offset_m": _EVALUATION_ORIGIN_MAX_LATERAL_OFFSET_M,
  }


class CarlaEvaluationWorld(_OpenPilotWorld):
  """Single tick owner, sensor synchronizer, evaluator, and actor lifecycle owner."""

  def __init__(
    self,
    client: Any,
    run: RunSpec,
    config: CarlaRuntimeConfig,
    status_queue: Any | None = None,
    dual_camera: bool = True,
    high_quality: bool = True,
    enable_cameras: bool = True,
    controller_name: str = "openpilot",
    valid_for_research: bool = True,
    openpilot_commit: str | None = None,
    openpilot_dirty: bool | None = None,
    applied_control_mono_time_supplier: Callable[[], int | None] | None = None,
    openpilot_maneuver_modes_disabled: bool = False,
    requested_acceleration_supplier: Callable[[], float] | None = None,
  ) -> None:
    super().__init__(dual_camera)
    # OpenPilot's stock World exposes a counting semaphore around shared RGB
    # arrays.  A single-slot handshake prevents semaphore credits and buffer
    # overwrites when RGB-to-YUV conversion takes longer than one CARLA tick.
    self.image_lock = _CameraFrameHandoff()
    try:
      import carla
    except ImportError as exc:  # pragma: no cover - guarded by command-level doctor
      raise DependencyUnavailableError(
        "CARLA Python API is missing; install the 0.9.16 wheel in this environment"
      ) from exc

    self.carla = carla
    self.client = client
    self.run = run
    self.config = config
    self._requested_acceleration_supplier = requested_acceleration_supplier
    self._actuator_controller = None
    self._actuation_diagnostics = None
    if config.actuation.calibration_profile is not None and controller_name == "openpilot":
      profile = ActuatorCalibration.load(config.actuation.calibration_profile)
      profile.validate_vehicle(config.ego_blueprint, client.get_server_version())
      self._actuator_controller = CalibratedActuatorController(profile)
    self.status_queue = status_queue
    self.dual_camera = dual_camera
    self.high_quality = high_quality
    self.enable_cameras = enable_cameras
    self.controller_name = controller_name
    self.valid_for_research = valid_for_research
    self._closed = False
    self._termination_reason = "bridge_closed"
    self._world_frame = -1
    self._last_camera_frame = -1
    self._camera_condition = threading.Condition()
    self._road_frames: dict[int, Any] = {}
    self._wide_frames: dict[int, Any] = {}
    self._camera_publish_sequence = 0
    self._last_handoff_openpilot_frame_id: int | None = None
    self._latest_imu: Any | None = None
    self._latest_imu_frame = -1
    self._sensor_condition = threading.Condition()
    self._imu_frames: dict[int, Any] = {}
    self._pending_measurement_events: list[EventRecord] = []
    self._sensors: list[Any] = []
    self._sensors_stopped = False
    self._camera_sensor_ids: list[int] = []
    self._camera_drop_count = 0
    self._startup_camera_discard_count = 0
    self._camera_timeout_count = 0
    self._control_latency_ms: float | None = None
    self._commanded_acceleration_mps2: float | None = None
    self._commanded_steering_angle_deg: float | None = None
    self._controller_state = "traffic_manager" if controller_name == "traffic_manager" else None
    self._controller_alert: str | None = None
    self._openpilot_state_subscriber: Any | None = None
    self._openpilot_messaging: Any | None = None
    self._openpilot_causal_sockets: dict[str, Any] = {}
    self._openpilot_maneuver_modes_disabled = openpilot_maneuver_modes_disabled
    self._last_camera_wall_ns: int | None = None
    self._latency_measurement_pending = False
    self._controller_active = False
    self._openpilot_localization_ready = controller_name != "openpilot"
    self._active_duration_s = 0.0
    self._post_reset_settling = False
    self._post_reset_frame: int | None = None
    self._post_reset_ticks = 0
    self._post_reset_elapsed_s = 0.0
    self._post_reset_sane_imu_ticks = 0
    self._post_reset_last_accel_norm_mps2: float | None = None
    self._post_reset_last_gyro_norm_rad_s: float | None = None
    self._post_reset_speed_mps: float | None = None
    self._post_reset_started_wall_s: float | None = None
    self._post_reset_openpilot_camera_id: int | None = None
    self._post_reset_model_seen = False
    self._post_reset_control_ready = False
    self._post_reset_fresh_control_applied = False
    self._post_reset_localization_ready_at_release: bool | None = None
    self._post_reset_freshness = _OpenPilotFreshnessBarrier()
    self._applied_control_mono_time_supplier = applied_control_mono_time_supplier
    self._post_reset_applied_control_mono_time: int | None = None
    self._evaluation_started = False
    self._evaluation_start_elapsed_s: float | None = None
    self._evaluation_start_frame: int | None = None
    self._evaluation_start_recorder_frame: int | None = None
    self._post_reset_start_recorder_frame: int | None = None
    self._evaluation_time_s = 0.0
    self._route_distance_m = 0.0
    self._route_index = 0
    self._last_lane_offset_m = 0.0
    self._stuck_duration_s = 0.0
    self._evaluation_origin_alignment: dict[str, Any] | None = None
    self._collision_during_run = False
    self._scenario_actor_collision = False
    self._disengagement_recorded = False
    self._near_miss_active = False
    self._geometric_lane_crossing_active = False
    self._startup_elapsed_s = 0.0
    self._last_snapshot_elapsed_s: float | None = None
    # The bridge advances twenty controller-neutral physics ticks before its
    # control loop starts. Capture that grounded pose on the first control
    # callback and reuse it for the post-warmup reset instead of teleporting
    # back to the map's deliberately elevated actor-spawn transform.
    self._settled_spawn_transform: Any | None = None
    self._vc = carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0)
    self._rng = np.random.default_rng(run.seed + 101)
    self._carla_recorder_filename: str | None = None
    self._carla_recorder_active = False
    self._recorder_frame_index = 0
    self._recorder_world_frames: dict[int, int] = {}
    self._recorder_audit: dict[str, Any] | None = None
    self._dashboard: LiveDashboard | None = None
    self._dashboard_last_state_ns = 0
    self._dashboard_state_sequence = 0
    self._dashboard_speed_mps: float | None = None
    self._dashboard_failure_recorded = False
    self._demo = None
    self._chase_frames = {}
    if os.getenv("OPENCARLA_RECORD_DEMO") == "1":
      self._demo = DemoCapture(run.output_dir / "demo_capture")
    self._spectator: Any | None = None
    self._spectator_pose: tuple[float, float, float, float, float, float] | None = None
    self._spectator_failure_recorded = False

    if controller_name == "openpilot":
      try:
        import cereal.messaging as messaging

        self._openpilot_messaging = messaging
        self._openpilot_state_subscriber = messaging.SubMaster(
          [
            "selfdriveState",
            "carControl",
            "livePose",
            "modelV2",
            "longitudinalPlan",
            "controlsState",
            "carState",
            "liveCalibration",
          ]
        )
        self._openpilot_causal_sockets = {
          service: messaging.sub_sock(service, conflate=False)
          for service in (
            "modelV2",
            "longitudinalPlan",
            "controlsState",
            "carControl",
            "lateralManeuverPlan",
          )
        }
      except ImportError:
        # The OpenPilot bridge command verifies this dependency before spawning.
        self._openpilot_state_subscriber = None

    self.client_version = client.get_client_version()
    self.server_version = client.get_server_version()
    if self.client_version != self.server_version or self.client_version != config.required_version:
      raise SimulatorConnectionError(
        "CARLA client/server mismatch: "
        f"client={self.client_version!r}, server={self.server_version!r}, "
        f"required={config.required_version!r}"
      )
    current_world = client.get_world()
    current_map_name = current_world.get_map().name.rsplit("/", maxsplit=1)[-1]
    if current_map_name == run.scenario.map_name:
      # CARLA's packaged server starts in Town10HD_Opt, which is also the map
      # used by this suite. Reusing it avoids an unnecessary Epic-quality map
      # reload and the associated transient VRAM spike.
      self._notify("start", "reusing_world")
      world = current_world
    else:
      self._notify("start", "loading_world")
      world = client.load_world(run.scenario.map_name, reset_settings=True)
    settings = world.get_settings()
    if settings.no_rendering_mode:
      raise SimulatorConnectionError(
        "CARLA no_rendering_mode is enabled; RGB evaluation requires rendered camera frames"
      )
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = config.fixed_delta_seconds
    settings.substepping = config.substepping
    settings.max_substep_delta_time = config.max_substep_delta_time
    settings.max_substeps = config.max_substeps
    world.apply_settings(settings)
    # The map was freshly loaded above and no evaluation actors exist yet. A
    # second reload here temporarily duplicates Epic-quality render resources
    # and can exhaust GPUs with 6 GB of VRAM. Applying synchrony before actor
    # creation retains deterministic evaluation setup without loading the map
    # twice.
    self.world = world
    self.original_settings = self.world.get_settings()
    self.traffic_manager = client.get_trafficmanager(config.traffic_manager_port)
    self.traffic_manager.set_synchronous_mode(True)
    self.traffic_manager.set_random_device_seed(run.seed)
    if hasattr(self.traffic_manager, "set_hybrid_physics_mode"):
      self.traffic_manager.set_hybrid_physics_mode(False)
    self._apply_weather(run.condition.weather)

    toolkit_root = Path(__file__).resolve().parents[2]
    toolkit_commit, toolkit_dirty = _git_state(toolkit_root)
    research_valid = valid_for_research and toolkit_commit is not None and toolkit_dirty is False
    metadata = run.metadata(backend="carla", valid_for_research=research_valid)
    if not research_valid:
      metadata["invalid_reason"] = (
        "untested_or_dirty_openpilot" if not valid_for_research else "dirty_or_unversioned_toolkit"
      )
    metadata["software"] = {
      "carla_client": self.client_version,
      "carla_server": self.server_version,
      "openpilot_target": "v0.11.1",
      "openpilot_commit": openpilot_commit,
      "openpilot_worktree_dirty": openpilot_dirty,
      "toolkit_commit": toolkit_commit,
      "toolkit_worktree_dirty": toolkit_dirty,
      "python": platform.python_version(),
      "platform": platform.platform(),
      "bridge": "opencarla-eval current-World port",
    }
    metadata["runtime_config"] = {
      "source_path": str(config.source_path),
      "effective_host": config.host,
      "effective_port": config.port,
      "timeout_s": config.timeout_s,
      "fixed_delta_seconds": config.fixed_delta_seconds,
      "traffic_manager_port": config.traffic_manager_port,
      "substepping": config.substepping,
      "max_substep_delta_time": config.max_substep_delta_time,
      "max_substeps": config.max_substeps,
      "quality_level": config.quality_level,
      "ego_blueprint": config.ego_blueprint,
      "role_name": config.role_name,
      "camera": {
        "width": config.camera.width,
        "height": config.camera.height,
        "road_fov_deg": config.camera.road_fov_deg,
        "wide_fov_deg": config.camera.wide_fov_deg,
        "sensor_tick_s": config.camera.sensor_tick_s,
        "gamma": config.camera.gamma,
        "mount": config.camera.mount,
      },
      "actuation": {
        "steer_sign": config.actuation.steer_sign,
        "steering_ratio": config.actuation.steering_ratio,
        "fallback_max_wheel_angle_deg": config.actuation.fallback_max_wheel_angle_deg,
        "calibration_profile_sha256": _file_sha256(config.actuation.calibration_profile)
        if config.actuation.calibration_profile
        else None,
        "controller": "empirical_inverse_with_bounded_PI"
        if self._actuator_controller
        else "stock_linear",
      },
      "recording": {
        "carla_recorder": config.recording.carla_recorder,
        "server_integrity_recorder": True,
      },
      "spectator": {
        "enabled": config.spectator.enabled,
        "distance_m": config.spectator.distance_m,
        "height_m": config.spectator.height_m,
        "pitch_deg": config.spectator.pitch_deg,
        "smoothing_time_s": config.spectator.smoothing_time_s,
      },
      "dashboard": {
        "enabled": config.dashboard.enabled,
        "host": config.dashboard.host,
        "port": config.dashboard.port,
        "frame_stride": config.dashboard.frame_stride,
        "downsample": config.dashboard.downsample,
        "jpeg_quality": config.dashboard.jpeg_quality,
      },
    }
    metadata["scenario_sha256"] = _scenario_sha256(run)
    metadata["runtime_config_sha256"] = _file_sha256(config.source_path)
    metadata["runtime_effective_sha256"] = hashlib.sha256(
      json.dumps(metadata["runtime_config"], sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    self.recorder = RunRecorder(run.output_dir, metadata, run.scenario.thresholds)

    selection = select_spawn(self.world, run)
    self.reference_route, self._route_cumulative_m = build_reference_route(
      selection.waypoint, run.scenario.route_length_m + _ROUTE_RESERVE_M
    )
    self.spawn_point = selection.transform
    self._route_origin_progress_m = 0.0
    blueprint_library = self.world.get_blueprint_library()
    try:
      ego_blueprint = blueprint_library.find(config.ego_blueprint)
    except IndexError as exc:
      raise SimulatorConnectionError(
        f"Ego blueprint {config.ego_blueprint!r} is not installed in this CARLA build"
      ) from exc
    if ego_blueprint.has_attribute("role_name"):
      ego_blueprint.set_attribute("role_name", config.role_name)
    self.vehicle = self.world.try_spawn_actor(ego_blueprint, self.spawn_point)
    if self.vehicle is None:
      raise SimulatorConnectionError("Failed to spawn the ego vehicle")
    if config.spectator.enabled:
      try:
        self._spectator = self.world.get_spectator()
      except RuntimeError as exc:
        self._disable_spectator(f"get_spectator failed: {exc}")
    self._apply_tire_friction(run.condition.friction)
    physics = self.vehicle.get_physics_control()
    steer_angles = [float(wheel.max_steer_angle) for wheel in physics.wheels]
    self.max_wheel_angle_deg = (
      max(steer_angles) if steer_angles else config.actuation.fallback_max_wheel_angle_deg
    )
    self._spawn_measurement_sensors()
    self.scenario = CarlaScenarioRuntime(
      client,
      self.world,
      self.vehicle,
      selection.waypoint,
      selection.traffic_light,
      self.traffic_manager,
      config.traffic_manager_port,
      run,
      self.recorder,
      self.reference_route,
    )
    safe_run_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", run.run_id)
    self._carla_recorder_filename = (
      f"opencarla_eval_{safe_run_id}.rec"
      if config.recording.carla_recorder
      else "opencarla_eval_integrity.rec"
    )
    client.start_recorder(self._carla_recorder_filename, True)
    self._carla_recorder_active = True
    if controller_name == "openpilot" and config.dashboard.enabled:
      try:
        self._dashboard = LiveDashboard(
          config.dashboard.host,
          config.dashboard.port,
          frame_stride=config.dashboard.frame_stride,
          downsample=config.dashboard.downsample,
          jpeg_quality=config.dashboard.jpeg_quality,
        )
        self._dashboard.start()
        self._dashboard.publish_state(
          {
            "runtime": {
              "phase": "starting",
              "run_id": run.run_id,
              "target_speed_mps": run.scenario.target_speed_mps,
            }
          }
        )
        self.recorder.record_event(
          EventRecord(
            "dashboard_started",
            0.0,
            details={"host": config.dashboard.host, "port": self._dashboard.port},
          )
        )
      except Exception as exc:
        self._disable_dashboard(f"startup: {type(exc).__name__}: {exc}")
    self._notify("start", "ready")

  def _notify(self, event: str, message: str) -> None:
    if self.status_queue is None:
      return
    with suppress(OSError, ValueError):
      self.status_queue.put({"event": event, "message": message, "run_id": self.run.run_id})

  def _apply_weather(self, name: str) -> None:
    carla = self.carla
    if name == "ClearNight":
      self.world.set_weather(carla.WeatherParameters.ClearNoon)
      weather = self.world.get_weather()
      weather.sun_altitude_angle = -20.0
      weather.sun_azimuth_angle = 15.0
      self.world.set_weather(weather)
      try:
        manager = self.world.get_lightmanager()
        lights = manager.get_all_lights(carla.LightGroup.Street)
        manager.turn_on(lights)
      except (AttributeError, RuntimeError):
        pass
      return
    if not hasattr(carla.WeatherParameters, name):
      raise SimulatorConnectionError(f"Unknown CARLA weather preset {name!r}")
    self.world.set_weather(getattr(carla.WeatherParameters, name))

  def _apply_tire_friction(self, factor: float) -> None:
    if math.isclose(factor, 1.0):
      return
    physics = self.vehicle.get_physics_control()
    wheels = list(physics.wheels)
    for wheel in wheels:
      wheel.tire_friction *= factor
    physics.wheels = wheels
    self.vehicle.apply_physics_control(physics)

  def _disable_spectator(self, reason: str) -> None:
    self._spectator = None
    if self._spectator_failure_recorded:
      return
    self._spectator_failure_recorded = True
    self.recorder.record_event(
      EventRecord(
        "spectator_disabled", self._evaluation_time_s, self._world_frame, {"reason": reason}
      )
    )

  def _disable_dashboard(self, reason: str) -> None:
    dashboard = self._dashboard
    self._dashboard = None
    if dashboard is not None:
      with suppress(Exception):
        dashboard.close()
    if self._dashboard_failure_recorded:
      return
    self._dashboard_failure_recorded = True
    with suppress(Exception):
      self.recorder.record_event(
        EventRecord(
          "dashboard_disabled",
          self._evaluation_time_s,
          self._world_frame,
          {"reason": reason},
        )
      )

  def _update_spectator(self) -> None:
    """Move CARLA's server-owned spectator without touching simulation timing."""

    if self._spectator is None:
      return
    config = self.config.spectator
    target = _chase_spectator_pose(
      self.vehicle.get_transform(), config.distance_m, config.height_m, config.pitch_deg
    )
    pose = _smooth_spectator_pose(
      self._spectator_pose,
      target,
      delta_s=self.config.fixed_delta_seconds,
      smoothing_time_s=config.smoothing_time_s,
      snap_distance_m=max(15.0, 2.0 * config.distance_m),
    )
    transform = self.carla.Transform(
      self.carla.Location(x=pose[0], y=pose[1], z=pose[2]),
      self.carla.Rotation(pitch=pose[3], yaw=pose[4], roll=pose[5]),
    )
    try:
      self._spectator.set_transform(transform)
      self._spectator_pose = pose
    except RuntimeError as exc:
      self._disable_spectator(f"set_transform failed: {exc}")

  def _sensor_transform(self) -> Any:
    mount = self.config.camera.mount
    return self.carla.Transform(
      self.carla.Location(x=mount["x"], y=mount["y"], z=mount["z"]),
      self.carla.Rotation(pitch=mount["pitch"], yaw=mount["yaw"], roll=mount["roll"]),
    )

  def _spawn_measurement_sensors(self) -> None:
    carla = self.carla
    library = self.world.get_blueprint_library()
    transform = self._sensor_transform()

    collision_bp = library.find("sensor.other.collision")
    collision = self.world.spawn_actor(
      collision_bp,
      carla.Transform(),
      attach_to=self.vehicle,
      attachment_type=carla.AttachmentType.Rigid,
    )
    collision.listen(_weak_sensor_callback(self._on_collision))
    self._sensors.append(collision)

    lane_bp = library.find("sensor.other.lane_invasion")
    lane = self.world.spawn_actor(
      lane_bp,
      carla.Transform(),
      attach_to=self.vehicle,
      attachment_type=carla.AttachmentType.Rigid,
    )
    lane.listen(_weak_sensor_callback(self._on_lane_invasion))
    self._sensors.append(lane)

    imu_bp = library.find("sensor.other.imu")
    # In synchronous mode CARLA's 0.05 sensor scheduler can occasionally skip
    # a frame when its floating-point accumulator lands just below the 0.05 s
    # physics step. Zero means emit on every world tick; fixed_delta_seconds
    # still determines the effective 20 Hz sampling period.
    imu_bp.set_attribute("sensor_tick", "0.0")
    imu = self.world.spawn_actor(
      imu_bp,
      transform,
      attach_to=self.vehicle,
      attachment_type=carla.AttachmentType.Rigid,
    )
    imu.listen(_weak_sensor_callback(self._on_imu))
    self._sensors.append(imu)

    if self.enable_cameras:
      if self._demo is not None:
        blueprint = self.world.get_blueprint_library().find("sensor.camera.rgb")
        for key, value in {
          "image_size_x": "960",
          "image_size_y": "540",
          "fov": "90",
          "sensor_tick": "0.0",
        }.items():
          blueprint.set_attribute(key, value)
        chase = self.world.spawn_actor(
          blueprint,
          self.carla.Transform(self.carla.Location(x=-7, z=3), self.carla.Rotation(pitch=-15)),
          attach_to=self.vehicle,
        )
        chase.listen(_weak_sensor_callback(self._on_chase_camera))
        self._sensors.append(chase)
      self.road_camera = self._spawn_camera(self.config.camera.road_fov_deg, self._on_road_camera)
      if self.dual_camera:
        self.wide_road_camera = self._spawn_camera(
          self.config.camera.wide_fov_deg, self._on_wide_camera
        )
      else:
        self.wide_road_camera = None
    else:
      self.road_camera = None
      self.wide_road_camera = None

  def _spawn_camera(self, fov: float, callback: Any) -> Any:
    carla = self.carla
    blueprint = self.world.get_blueprint_library().find("sensor.camera.rgb")
    blueprint.set_attribute("image_size_x", str(self.config.camera.width))
    blueprint.set_attribute("image_size_y", str(self.config.camera.height))
    blueprint.set_attribute("fov", str(fov))
    # Runtime validation requires camera.sensor_tick_s to match the fixed world
    # delta, so emitting every world tick has the same effective sample rate and
    # avoids CARLA's floating-point sensor scheduler skipping a frame.
    blueprint.set_attribute("sensor_tick", "0.0")
    blueprint.set_attribute("gamma", str(self.config.camera.gamma))
    blueprint.set_attribute("enable_postprocess_effects", "True" if self.high_quality else "False")
    camera = self.world.spawn_actor(
      blueprint,
      self._sensor_transform(),
      attach_to=self.vehicle,
      attachment_type=carla.AttachmentType.Rigid,
    )
    camera.listen(_weak_sensor_callback(callback))
    self._sensors.append(camera)
    self._camera_sensor_ids.append(camera.id)
    return camera

  def _on_chase_camera(self, image: Any) -> None:
    with self._camera_condition:
      self._chase_frames[int(image.frame)] = image
      for old in sorted(self._chase_frames)[:-16]:
        del self._chase_frames[old]

  def _on_collision(self, event: Any) -> None:
    impulse = event.normal_impulse
    magnitude = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
    record = EventRecord(
      "collision",
      0.0,
      int(event.frame),
      {
        "other_actor_id": event.other_actor.id,
        "other_actor_type": event.other_actor.type_id,
        "impulse_ns": magnitude,
      },
    )
    with self._sensor_condition:
      self._pending_measurement_events.append(record)
      self._sensor_condition.notify_all()

  def _on_lane_invasion(self, event: Any) -> None:
    record = EventRecord(
      "lane_invasion_sensor",
      0.0,
      int(event.frame),
      {
        "markings": [str(marking.type) for marking in event.crossed_lane_markings],
        "source": "carla_async_sensor",
      },
    )
    with self._sensor_condition:
      self._pending_measurement_events.append(record)
      self._sensor_condition.notify_all()

  def _on_imu(self, measurement: Any) -> None:
    frame = int(measurement.frame)
    with self._sensor_condition:
      self._imu_frames[frame] = measurement
      if len(self._imu_frames) > 24:
        for stale_frame in sorted(self._imu_frames)[:-24]:
          self._imu_frames.pop(stale_frame, None)
      self._sensor_condition.notify_all()

  def _wait_for_imu(self, frame: int, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    with self._sensor_condition:
      while frame not in self._imu_frames:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
          break
        self._sensor_condition.wait(timeout=min(remaining, 0.25))
      measurement = self._imu_frames.pop(frame, None)
      for stale_frame in [item for item in self._imu_frames if item < frame]:
        self._imu_frames.pop(stale_frame, None)
    if measurement is None:
      self.recorder.record_event(
        EventRecord("imu_timeout", self._evaluation_time_s, frame, {"expected_frame": frame})
      )
      self._termination_reason = "imu_timeout"
      self.exit_event.set()
      return False
    self._latest_imu = measurement
    self._latest_imu_frame = frame
    return True

  def _update_post_reset_imu_health(self, frame: int) -> bool:
    """Audit raw CARLA inertial data while the evaluation start is disarmed."""

    if not self._post_reset_settling or self._latest_imu is None:
      return True
    acceleration = self._latest_imu.accelerometer
    gyroscope = self._latest_imu.gyroscope
    accel_norm = math.sqrt(acceleration.x**2 + acceleration.y**2 + acceleration.z**2)
    gyro_norm = math.sqrt(gyroscope.x**2 + gyroscope.y**2 + gyroscope.z**2)
    vertical_speed = abs(self.vehicle.get_velocity().z)
    self._post_reset_last_accel_norm_mps2 = accel_norm
    self._post_reset_last_gyro_norm_rad_s = gyro_norm

    if (
      not math.isfinite(accel_norm)
      or accel_norm >= _POST_RESET_ACCEL_NORM_MAX_MPS2
      or not math.isfinite(gyro_norm)
      or gyro_norm >= _POST_RESET_GYRO_NORM_MAX_RAD_S
    ):
      self.recorder.record_event(
        EventRecord(
          "post_reset_imu_discontinuity",
          0.0,
          frame,
          {
            "accelerometer_mps2": [acceleration.x, acceleration.y, acceleration.z],
            "acceleration_norm_mps2": accel_norm,
            "gyroscope_rad_s": [gyroscope.x, gyroscope.y, gyroscope.z],
            "gyroscope_norm_rad_s": gyro_norm,
            "vertical_speed_mps": vertical_speed,
          },
        )
      )
      self._termination_reason = "post_reset_imu_discontinuity"
      self.exit_event.set()
      return False

    sane = (
      accel_norm >= _POST_RESET_ACCEL_NORM_MIN_MPS2
      and vertical_speed <= _POST_RESET_VERTICAL_SPEED_MAX_MPS
    )
    self._post_reset_sane_imu_ticks = self._post_reset_sane_imu_ticks + 1 if sane else 0
    return True

  def _drain_measurement_events(self, up_to_frame: int) -> None:
    """Commit queued sparse sensor events only after their CARLA frame is known."""

    with self._sensor_condition:
      ready = [
        event
        for event in self._pending_measurement_events
        if event.frame is not None and event.frame <= up_to_frame
      ]
      self._pending_measurement_events = [
        event
        for event in self._pending_measurement_events
        if event.frame is None or event.frame > up_to_frame
      ]
    ready.sort(key=lambda event: int(event.frame or -1))
    for event in ready:
      if self._evaluation_start_frame is None or event.frame is None:
        if (
          event.event_type == "collision"
          and event.frame is not None
          and self._post_reset_settling
          and self._post_reset_frame is not None
          and event.frame >= self._post_reset_frame
        ):
          self.recorder.record_event(
            EventRecord("post_reset_collision", 0.0, event.frame, event.details)
          )
          self._termination_reason = "post_reset_collision"
          self.exit_event.set()
        continue
      if event.frame < self._evaluation_start_frame:
        continue
      sim_time = (event.frame - self._evaluation_start_frame) * self.config.fixed_delta_seconds
      synchronized = EventRecord(event.event_type, sim_time, event.frame, event.details)
      self.recorder.record_event(synchronized)
      if event.event_type != "collision":
        continue
      self._collision_during_run = True
      scenario_actor = getattr(self.scenario, "_scenario_actor", None)
      if scenario_actor is not None and event.details.get("other_actor_id") == scenario_actor.id:
        self._scenario_actor_collision = True
      if self.run.scenario.parameters.get(
        "terminate_on_collision", True
      ) and self._termination_reason in {"bridge_closed", "route_completed", "timeout"}:
        self._termination_reason = "collision"
        self.exit_event.set()

  def _store_camera_frame(self, target: dict[int, Any], image: Any) -> None:
    with self._camera_condition:
      target[int(image.frame)] = image
      if len(target) > 24:
        for frame in sorted(target)[:-24]:
          target.pop(frame, None)
          if self._evaluation_started:
            self._camera_drop_count += 1
          else:
            self._startup_camera_discard_count += 1
      self._camera_condition.notify_all()

  def _on_road_camera(self, image: Any) -> None:
    self._store_camera_frame(self._road_frames, image)

  def _on_wide_camera(self, image: Any) -> None:
    self._store_camera_frame(self._wide_frames, image)

  def _image_to_rgb(self, image: Any) -> np.ndarray:
    array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
      (self.config.camera.height, self.config.camera.width, 4)
    )
    rgb = np.ascontiguousarray(array[:, :, [2, 1, 0]])
    std = self.run.condition.camera_noise_std
    if std > 0:
      noise = self._rng.normal(0.0, std, size=rgb.shape)
      rgb = np.clip(rgb.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return rgb

  @staticmethod
  def _copy_openpilot_causal_payload(service: str, payload: Any) -> Any:
    """Detach the small causal record from Cap'n Proto message storage."""

    if service == "modelV2":
      return SimpleNamespace(
        frameId=int(getattr(payload, "frameId", -1)),
        frameIdExtra=int(getattr(payload, "frameIdExtra", -1)),
      )
    if service == "longitudinalPlan":
      return SimpleNamespace(modelMonoTime=int(getattr(payload, "modelMonoTime", 0)))
    if service == "controlsState":
      return SimpleNamespace(
        lateralPlanMonoTime=int(getattr(payload, "lateralPlanMonoTime", 0)),
        longitudinalPlanMonoTime=int(getattr(payload, "longitudinalPlanMonoTime", 0)),
        desiredCurvature=float(getattr(payload, "desiredCurvature", math.nan)),
        longControlState=_enum_identity(getattr(payload, "longControlState", "missing")),
      )
    if service == "carControl":
      actuators = getattr(payload, "actuators", SimpleNamespace())
      return SimpleNamespace(
        enabled=bool(getattr(payload, "enabled", False)),
        latActive=bool(getattr(payload, "latActive", False)),
        longActive=bool(getattr(payload, "longActive", False)),
        actuators=SimpleNamespace(
          curvature=float(getattr(actuators, "curvature", math.nan)),
          longControlState=_enum_identity(getattr(actuators, "longControlState", "missing")),
        ),
      )
    return SimpleNamespace()

  def _drain_openpilot_causal_events(self) -> None:
    if self._openpilot_messaging is None or not self._openpilot_causal_sockets:
      return
    captured: list[tuple[int, str, bool, Any]] = []
    for service, socket in self._openpilot_causal_sockets.items():
      exhausted = False
      for _ in range(4096):
        message = self._openpilot_messaging.recv_one_or_none(socket)
        if message is None:
          exhausted = True
          break
        captured.append(
          (
            int(message.logMonoTime),
            service,
            bool(message.valid),
            self._copy_openpilot_causal_payload(service, getattr(message, service)),
          )
        )
      if not exhausted:
        self._post_reset_freshness.integrity_errors.append(
          f"lossless {service} socket exceeded its per-cycle drain bound"
        )
    for mono_time, service, valid, payload in sorted(captured, key=lambda item: item[0]):
      self._post_reset_freshness.ingest(service, mono_time, valid, payload)

  def _capture_settled_spawn_transform(self) -> None:
    """Remember the suspension-settled pose produced by initial physics ticks."""

    if self._settled_spawn_transform is None:
      self._settled_spawn_transform = self.vehicle.get_transform()

  def _post_reset_physics_ready(self) -> bool:
    return (
      self._post_reset_ticks >= _POST_RESET_SETTLING_TICKS
      and self._post_reset_sane_imu_ticks >= _POST_RESET_REQUIRED_SANE_IMU_TICKS
    )

  def _post_reset_controller_ready(self) -> bool:
    if self.controller_name != "openpilot":
      return True
    return (
      self._controller_active
      and self._controller_state == "enabled"
      and self._openpilot_localization_ready
    )

  def _post_reset_waiting_for_causal_control(self) -> bool:
    """Latch CARLA once the final settling camera pair has been released."""

    return (
      self._post_reset_settling
      and self.controller_name == "openpilot"
      and self._post_reset_openpilot_camera_id is not None
      and not self._post_reset_fresh_control_applied
    )

  def apply_controls(self, steer_angle: float, throttle_out: float, brake_out: float) -> None:
    if (
      getattr(self, "_settled_spawn_transform", None) is None
      and not self._post_reset_settling
      and not self._evaluation_started
      and hasattr(self.vehicle, "get_transform")
    ):
      self._capture_settled_spawn_transform()
    if self._post_reset_settling:
      release_control = False
      if (
        self.controller_name == "openpilot"
        and self._post_reset_physics_ready()
        and self._post_reset_controller_ready()
        and self._applied_control_mono_time_supplier is not None
      ):
        self._drain_openpilot_causal_events()
        try:
          applied_mono_time = self._applied_control_mono_time_supplier()
          self._post_reset_applied_control_mono_time = (
            int(applied_mono_time) if applied_mono_time is not None else None
          )
        except (AttributeError, KeyError, TypeError, ValueError):
          self._post_reset_applied_control_mono_time = None
        if self._post_reset_applied_control_mono_time is not None:
          evidence = self._post_reset_freshness.evaluate(self._post_reset_applied_control_mono_time)
          release_control = bool(evidence["valid"])
      self._post_reset_control_ready = release_control
      self._post_reset_fresh_control_applied = release_control
      if not release_control:
        steer_angle = 0.0
        throttle_out = 0.0
        brake_out = 0.0
    elif (
      self.controller_name == "openpilot"
      and not self._controller_active
      and not self._evaluation_started
    ):
      startup_target_speed_mps = float(
        self.run.scenario.parameters.get(
          "startup_target_speed_mps", self.run.scenario.target_speed_mps
        )
      )
      startup_max_throttle = float(self.run.scenario.parameters.get("startup_max_throttle", 0.65))
      speed_error = startup_target_speed_mps - _speed(self.vehicle.get_velocity())
      throttle_out = float(np.clip(speed_error * 0.25, 0.0, startup_max_throttle))
      brake_out = float(np.clip(-speed_error * 0.2, 0.0, 0.5))
    requested = float(throttle_out) * 1.6 if throttle_out > 0 else -float(brake_out) * 4.0
    if (
      getattr(self, "_actuator_controller", None) is not None
      and self._controller_active
      and (not self._post_reset_settling or self._post_reset_control_ready)
    ):
      if self._requested_acceleration_supplier is None:
        raise RuntimeError("Calibrated actuation requires the selected raw carControl acceleration")
      requested = self._requested_acceleration_supplier()
      acceleration = _body_components(
        self.vehicle.get_acceleration(), self.vehicle.get_transform()
      )[0]
      command = self._actuator_controller.update(
        frame=self._world_frame,
        dt_s=self.config.fixed_delta_seconds,
        requested_acceleration_mps2=requested,
        speed_mps=_speed(self.vehicle.get_velocity()),
        measured_acceleration_mps2=acceleration,
      )
      throttle_out, brake_out = command.throttle, command.brake
      requested = command.requested_acceleration_mps2
      from dataclasses import asdict

      self._actuation_diagnostics = asdict(command)
    denominator = self.max_wheel_angle_deg * self.config.actuation.steering_ratio
    normalized = self.config.actuation.steer_sign * steer_angle / max(denominator, 1e-6)
    self._vc.throttle = float(np.clip(throttle_out, 0.0, 1.0))
    self._vc.brake = float(np.clip(brake_out, 0.0, 1.0))
    self._vc.steer = float(np.clip(normalized, -1.0, 1.0))
    self._commanded_steering_angle_deg = float(steer_angle)
    self._commanded_acceleration_mps2 = requested
    self.vehicle.apply_control(self._vc)

  def _steering_feedback_deg(self) -> float:
    denominator = self.config.actuation.steer_sign
    return (
      self.vehicle.get_control().steer
      * self.max_wheel_angle_deg
      * self.config.actuation.steering_ratio
      / denominator
    )

  def read_state(self) -> None:
    """Current OpenPilot bridge hook; CARLA state is read directly on demand."""

  def read_sensors(self, simulator_state: Any) -> None:
    transform = self.vehicle.get_transform()
    velocity = self.vehicle.get_velocity()
    self._dashboard_speed_mps = _speed(velocity)
    # SimulatedSensors applies [-y, x, z] before publishing NED velocity. This
    # inverse transform keeps CARLA +x aligned with latitude/north and +y with
    # longitude/east, matching GPSState.from_xy and bearing.
    simulator_state.velocity = SimpleNamespace(x=velocity.y, y=-velocity.x, z=-velocity.z)
    simulator_state.bearing = transform.rotation.yaw
    simulator_state.gps.from_xy([transform.location.x, transform.location.y])
    simulator_state.valid = True
    simulator_state.steering_angle = self._steering_feedback_deg()
    measurement = self._latest_imu
    if measurement is not None:
      accelerometer = _carla_accel_to_openpilot_sensor(
        measurement.accelerometer.x,
        measurement.accelerometer.y,
        measurement.accelerometer.z,
      )
      gyroscope = _carla_gyro_to_openpilot_sensor(
        measurement.gyroscope.x,
        measurement.gyroscope.y,
        measurement.gyroscope.z,
      )
      simulator_state.imu.accelerometer = SimpleNamespace(
        x=accelerometer[0], y=accelerometer[1], z=accelerometer[2]
      )
      simulator_state.imu.gyroscope = SimpleNamespace(
        x=gyroscope[0], y=gyroscope[1], z=gyroscope[2]
      )
    else:
      acceleration = self.vehicle.get_acceleration()
      angular = self.vehicle.get_angular_velocity()
      body_acceleration = _body_components(acceleration, transform)
      body_angular = _body_components(
        SimpleNamespace(
          x=math.radians(angular.x),
          y=math.radians(angular.y),
          z=math.radians(angular.z),
        ),
        transform,
      )
      accelerometer = _carla_accel_to_openpilot_sensor(*body_acceleration)
      gyroscope = _carla_gyro_to_openpilot_sensor(*body_angular)
      simulator_state.imu.accelerometer = SimpleNamespace(
        x=accelerometer[0], y=accelerometer[1], z=accelerometer[2]
      )
      simulator_state.imu.gyroscope = SimpleNamespace(
        x=gyroscope[0], y=gyroscope[1], z=gyroscope[2]
      )
    simulator_state.imu.bearing = transform.rotation.yaw
    self._controller_active = bool(simulator_state.is_engaged)
    if self._openpilot_state_subscriber is not None:
      self._openpilot_state_subscriber.update(0)
      self._drain_openpilot_causal_events()
      state = self._openpilot_state_subscriber["selfdriveState"]
      self._controller_state = str(getattr(state, "state", "unknown"))
      pose = self._openpilot_state_subscriber["livePose"]
      self._openpilot_localization_ready = (
        bool(self._openpilot_state_subscriber.valid["livePose"])
        and bool(getattr(pose, "inputsOK", False))
        and bool(getattr(pose, "sensorsOK", False))
      )
      alert_parts = [
        str(value)
        for value in (getattr(state, "alertText1", ""), getattr(state, "alertText2", ""))
        if value
      ]
      self._controller_alert = " | ".join(alert_parts) or None
      if self._post_reset_settling:
        target = self._post_reset_freshness.target_frame_id
        self._post_reset_model_seen = target is not None and any(
          model["valid"] and model["frame_id"] == target and model["frame_id_extra"] == target
          for model in self._post_reset_freshness.models.values()
        )
      if (
        self._openpilot_state_subscriber.updated["carControl"]
        and self._last_camera_wall_ns is not None
        and self._latency_measurement_pending
      ):
        self._control_latency_ms = (time.monotonic_ns() - self._last_camera_wall_ns) / 1e6
        self._latency_measurement_pending = False
      self._publish_dashboard_state()

  def _publish_dashboard_state(self, *, force: bool = False) -> None:
    if self._dashboard is None:
      return
    try:
      self._publish_dashboard_state_unchecked(force=force)
    except Exception as exc:
      self._disable_dashboard(f"state publisher: {type(exc).__name__}: {exc}")

  def _publish_dashboard_state_unchecked(self, *, force: bool = False) -> None:
    if self._dashboard is None or self._openpilot_state_subscriber is None:
      return
    now_ns = time.monotonic_ns()
    if not force and now_ns - self._dashboard_last_state_ns < 100_000_000:
      return
    self._dashboard_last_state_ns = now_ns
    subscriber = self._openpilot_state_subscriber
    services = (
      "selfdriveState",
      "carControl",
      "modelV2",
      "longitudinalPlan",
      "controlsState",
      "carState",
      "liveCalibration",
    )
    messages = {service: subscriber[service] for service in services}
    valid = {service: bool(subscriber.valid[service]) for service in services}
    service_health = {
      service: {
        "valid": valid[service],
        "alive": bool(getattr(subscriber, "alive", {}).get(service, False)),
        "frequency_ok": bool(getattr(subscriber, "freq_ok", {}).get(service, False)),
        "updated": bool(subscriber.updated[service]),
      }
      for service in services
    }
    phase = (
      "evaluating"
      if self._evaluation_started
      else "causal gate"
      if self._post_reset_settling
      else "warmup"
      if self._controller_active
      else "initializing"
    )
    self._dashboard_state_sequence += 1
    runtime = {
      "phase": phase,
      "run_id": self.run.run_id,
      "sim_time_s": self._evaluation_time_s,
      "speed_mps": self._dashboard_speed_mps,
      "actuation": self._actuation_diagnostics,
      "target_speed_mps": self.run.scenario.target_speed_mps,
      "control_latency_ms": self._control_latency_ms,
      "carla_frame": self._last_camera_frame,
      "openpilot_frame": self._last_handoff_openpilot_frame_id,
      "causal_target_frame": self._post_reset_freshness.target_frame_id,
      "causal_lineage_valid": bool(self._post_reset_freshness.last_evidence.get("valid")),
      "state_sequence": self._dashboard_state_sequence,
      "wall_time_epoch_s": time.time(),
      "service_health": service_health,
      "camera_projection": {
        "width_px": self.config.camera.width,
        "height_px": self.config.camera.height,
        "horizontal_fov_deg": self.config.camera.road_fov_deg,
        "path_half_width_m": 0.9,
      },
    }
    state = build_openpilot_dashboard_state(messages, valid, runtime)
    self._dashboard.publish_state(state)
    if self._demo is not None:
      self._demo.state(state)

  def set_controller_active(self, active: bool) -> None:
    """Set active state for a non-OpenPilot reference controller."""

    self._controller_active = active

  def read_cameras(self) -> None:
    if (
      not self.enable_cameras
      or self._world_frame < 0
      or self._world_frame == self._last_camera_frame
    ):
      return
    expected = self._world_frame
    deadline = time.monotonic() + 5.0
    with self._camera_condition:
      while expected not in self._road_frames or (
        self.dual_camera and expected not in self._wide_frames
      ):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
          self._camera_timeout_count += 1
          self.recorder.record_event(
            EventRecord(
              "camera_timeout",
              self._evaluation_time_s,
              expected,
              {"expected_frame": expected},
            )
          )
          self._termination_reason = "camera_timeout"
          self.exit_event.set()
          return
        self._camera_condition.wait(timeout=min(remaining, 0.25))
      road = self._road_frames.pop(expected)
      wide = self._wide_frames.pop(expected) if self.dual_camera else None
      for frame in [item for item in self._road_frames if item < expected]:
        self._road_frames.pop(frame, None)
        if self._evaluation_started:
          self._camera_drop_count += 1
        else:
          self._startup_camera_discard_count += 1
      for frame in [item for item in self._wide_frames if item < expected]:
        self._wide_frames.pop(frame, None)
        if self._evaluation_started:
          self._camera_drop_count += 1
        else:
          self._startup_camera_discard_count += 1
    if not self.image_lock.wait_for_publish_slot(timeout=5.0):
      self._camera_timeout_count += 1
      self.recorder.record_event(
        EventRecord(
          "camera_backpressure_timeout",
          self._evaluation_time_s,
          expected,
          {"expected_frame": expected},
        )
      )
      self._termination_reason = "camera_backpressure_timeout"
      self.exit_event.set()
      return
    road_rgb = self._image_to_rgb(road)
    self.road_image[:] = road_rgb
    wide_rgb = None
    if wide is not None:
      wide_rgb = self._image_to_rgb(wide)
      self.wide_road_image[:] = wide_rgb
    openpilot_frame_id = self._camera_publish_sequence
    self._camera_publish_sequence += 1
    self._last_handoff_openpilot_frame_id = openpilot_frame_id
    self._last_camera_frame = expected
    self._last_camera_wall_ns = time.monotonic_ns()
    self._latency_measurement_pending = True
    if (
      self._post_reset_settling
      and self._post_reset_physics_ready()
      and self._post_reset_controller_ready()
      and self._post_reset_openpilot_camera_id is None
    ):
      self._post_reset_openpilot_camera_id = openpilot_frame_id
      self._post_reset_freshness.arm(
        openpilot_frame_id,
        maneuver_modes_disabled=self._openpilot_maneuver_modes_disabled,
      )
    self.recorder.record_event(
      EventRecord(
        "camera_frame_handoff",
        self._evaluation_time_s if self._evaluation_started else 0.0,
        expected,
        {
          "carla_frame": expected,
          "openpilot_camera_frame_id": openpilot_frame_id,
          "dual_camera_pair": self.dual_camera,
        },
      )
    )
    self.image_lock.release()
    if getattr(self, "_demo", None) is not None:
      with self._camera_condition:
        chase = self._chase_frames.pop(expected, None)
      chase_rgb = (
        np.ascontiguousarray(
          np.frombuffer(chase.raw_data, dtype=np.uint8).reshape(chase.height, chase.width, 4)[
            :, :, [2, 1, 0]
          ]
        )
        if chase is not None
        else None
      )
      self._demo.camera(openpilot_frame_id, expected, self._evaluation_time_s, road_rgb, chase_rgb)
    if self._dashboard is not None:
      try:
        self._dashboard.publish_frame(
          "road", road_rgb, carla_frame=expected, openpilot_frame=openpilot_frame_id
        )
        if wide_rgb is not None:
          self._dashboard.publish_frame(
            "wide", wide_rgb, carla_frame=expected, openpilot_frame=openpilot_frame_id
          )
      except Exception as exc:
        self._disable_dashboard(f"camera publisher: {type(exc).__name__}: {exc}")

  def _lane_state(self) -> tuple[float, bool, Any | None]:
    location = self.vehicle.get_location()
    waypoint = self.world.get_map().get_waypoint(
      location,
      project_to_road=False,
      lane_type=self.carla.LaneType.Driving,
    )
    if waypoint is None:
      projected = self.world.get_map().get_waypoint(
        location,
        project_to_road=True,
        lane_type=self.carla.LaneType.Driving,
      )
      if projected is None:
        return self._last_lane_offset_m, True, None
      right = projected.transform.get_right_vector()
      delta = location - projected.transform.location
      offset = delta.x * right.x + delta.y * right.y + delta.z * right.z
      self._last_lane_offset_m = offset
      return offset, abs(offset) > projected.lane_width / 2.0 + 0.5, projected
    if waypoint.is_junction:
      return self._last_lane_offset_m, False, waypoint
    right = waypoint.transform.get_right_vector()
    delta = location - waypoint.transform.location
    offset = delta.x * right.x + delta.y * right.y + delta.z * right.z
    self._last_lane_offset_m = offset
    return offset, False, waypoint

  @staticmethod
  def _lane_marking_name(marking: Any) -> str:
    return str(getattr(marking, "type", "None"))

  @classmethod
  def _lane_marking_is_present(cls, marking: Any) -> bool:
    return cls._lane_marking_name(marking).rsplit(".", maxsplit=1)[-1].casefold() != "none"

  def _record_geometric_lane_invasion(self, frame: int, waypoint: Any | None) -> None:
    """Record lane-boundary crossings synchronously from the physics snapshot."""

    if waypoint is None:
      return
    if waypoint.is_junction:
      self._geometric_lane_crossing_active = False
      return

    center = waypoint.transform.location
    right = waypoint.transform.get_right_vector()
    vertices = self.vehicle.bounding_box.get_world_vertices(self.vehicle.get_transform())
    offsets = [
      (vertex.x - center.x) * right.x
      + (vertex.y - center.y) * right.y
      + (vertex.z - center.z) * right.z
      for vertex in vertices
    ]
    half_lane_width = float(waypoint.lane_width) / 2.0
    crossings: list[tuple[str, Any]] = []
    if (
      offsets
      and min(offsets) <= -half_lane_width
      and self._lane_marking_is_present(waypoint.left_lane_marking)
    ):
      crossings.append(("left", waypoint.left_lane_marking))
    if (
      offsets
      and max(offsets) >= half_lane_width
      and self._lane_marking_is_present(waypoint.right_lane_marking)
    ):
      crossings.append(("right", waypoint.right_lane_marking))

    crossing = bool(crossings)
    if crossing and not self._geometric_lane_crossing_active:
      self.recorder.record_event(
        EventRecord(
          "lane_invasion",
          self._evaluation_time_s,
          frame,
          {
            "source": "synchronous_vehicle_geometry",
            "sides": [side for side, _ in crossings],
            "markings": [self._lane_marking_name(marking) for _, marking in crossings],
            "lane_width_m": float(waypoint.lane_width),
            "minimum_vertex_offset_m": min(offsets),
            "maximum_vertex_offset_m": max(offsets),
          },
        )
      )
    self._geometric_lane_crossing_active = crossing

  def _record_sample(self, frame: int) -> None:
    transform = self.vehicle.get_transform()
    velocity = self.vehicle.get_velocity()
    acceleration = self.vehicle.get_acceleration()
    forward = transform.get_forward_vector()
    right = transform.get_right_vector()
    longitudinal_acceleration = (
      acceleration.x * forward.x + acceleration.y * forward.y + acceleration.z * forward.z
    )
    lateral_acceleration = (
      acceleration.x * right.x + acceleration.y * right.y + acceleration.z * right.z
    )
    lane_offset, offroad, lane_waypoint = self._lane_state()
    self._record_geometric_lane_invasion(frame, lane_waypoint)
    hazard = self.scenario.hazard_metrics()
    gap = hazard["distance_m"] if hazard else None
    closing = hazard["closing_speed_mps"] if hazard else None
    ttc = hazard["ttc_s"] if hazard else None
    actor_id = hazard["actor_id"] if hazard else None
    if ttc is not None and ttc < 1.5 and not self._near_miss_active:
      self.recorder.record_event(
        EventRecord(
          "near_miss", self._evaluation_time_s, frame, {"ttc_s": ttc, "actor_id": actor_id}
        )
      )
      self._near_miss_active = True
    elif ttc is None or ttc >= 1.5:
      self._near_miss_active = False

    location = transform.location
    candidate_indices = range(
      max(0, self._route_index - 2), min(len(self.reference_route), self._route_index + 11)
    )
    nearest_index = min(
      candidate_indices,
      key=lambda index: _speed(location - self.reference_route[index].transform.location),
    )
    self._route_index = max(self._route_index, nearest_index)
    evaluation_progress = max(
      0.0, self._route_cumulative_m[nearest_index] - self._route_origin_progress_m
    )
    self._route_distance_m = max(self._route_distance_m, evaluation_progress)
    route_completion = min(1.0, self._route_distance_m / self.run.scenario.route_length_m)
    speed_mps = _speed(velocity)
    stuck_timeout_s = self.run.scenario.parameters.get("stuck_timeout_s")
    if stuck_timeout_s is not None and speed_mps < 0.2:
      self._stuck_duration_s += self.config.fixed_delta_seconds
    else:
      self._stuck_duration_s = 0.0
    vehicle_stuck = (
      stuck_timeout_s is not None
      and self.run.scenario.target_speed_mps > 2.0
      and self._stuck_duration_s + 1e-9 >= float(stuck_timeout_s)
    )
    control = self.vehicle.get_control()
    sample = TelemetrySample(
      frame=frame,
      sim_time_s=self._evaluation_time_s,
      x_m=location.x,
      y_m=location.y,
      z_m=location.z,
      yaw_deg=transform.rotation.yaw,
      speed_mps=speed_mps,
      target_speed_mps=self.run.scenario.target_speed_mps,
      acceleration_mps2=longitudinal_acceleration,
      lateral_acceleration_mps2=lateral_acceleration,
      steering_angle_deg=(
        control.steer
        * self.max_wheel_angle_deg
        * self.config.actuation.steering_ratio
        / self.config.actuation.steer_sign
      ),
      throttle=control.throttle,
      brake=control.brake,
      lane_offset_m=lane_offset,
      route_completion=route_completion,
      controller_active=self._controller_active,
      offroad=offroad,
      nearest_actor_distance_m=gap,
      closing_speed_mps=closing,
      ttc_s=ttc,
      nearest_actor_id=actor_id,
      nearest_actor_type=hazard["actor_type"] if hazard else None,
      nearest_actor_speed_mps=hazard["actor_speed_mps"] if hazard else None,
      nearest_actor_x_m=hazard["actor_x_m"] if hazard else None,
      nearest_actor_y_m=hazard["actor_y_m"] if hazard else None,
      traffic_light_state=self.scenario.traffic_light_state(),
      speed_limit_mps=self.vehicle.get_speed_limit() / 3.6,
      control_latency_ms=self._control_latency_ms,
      commanded_acceleration_mps2=self._commanded_acceleration_mps2,
      commanded_steering_angle_deg=self._commanded_steering_angle_deg,
      controller_state=self._controller_state,
      controller_alert=self._controller_alert,
    )
    self.recorder.record_sample(sample)
    self.scenario.check_signal_compliance(self._evaluation_time_s, frame)

    if self._collision_during_run and self.run.scenario.parameters.get(
      "terminate_on_collision", True
    ):
      self._termination_reason = "collision"
      self.exit_event.set()
    elif route_completion >= 1.0:
      self._termination_reason = "route_completed"
      self.exit_event.set()
    elif vehicle_stuck:
      self.recorder.record_event(
        EventRecord(
          "vehicle_stuck",
          self._evaluation_time_s,
          frame,
          {
            "duration_s": self._stuck_duration_s,
            "speed_mps": speed_mps,
            "commanded_acceleration_mps2": self._commanded_acceleration_mps2,
            "throttle": float(control.throttle),
            "brake": float(control.brake),
          },
        )
      )
      self._termination_reason = "vehicle_stuck"
      self.exit_event.set()
    elif self._evaluation_time_s >= self.run.scenario.duration_s:
      self._termination_reason = "timeout"
      self.exit_event.set()

  def _set_evaluation_origin(self) -> None:
    """Anchor hazards and a fresh route at the continuous evaluation pose."""

    transform = self.vehicle.get_transform()
    location = transform.location
    waypoint = self.world.get_map().get_waypoint(
      location,
      project_to_road=False,
      lane_type=self.carla.LaneType.Driving,
    )
    if waypoint is None:
      details = {
        "valid": False,
        "reasons": ["not_on_driving_lane"],
        "ego_heading_deg": float(transform.rotation.yaw),
      }
      self._evaluation_origin_alignment = details
      self.recorder.record_event(
        EventRecord("evaluation_start_rejected", 0.0, self._world_frame, details)
      )
      raise SimulatorConnectionError("Ego left the driving lane before evaluation")
    alignment = _evaluation_origin_alignment(transform, waypoint)
    alignment["speed_mps"] = _speed(self.vehicle.get_velocity())
    self._evaluation_origin_alignment = alignment
    if not alignment["valid"]:
      self.recorder.record_event(
        EventRecord("evaluation_start_rejected", 0.0, self._world_frame, alignment)
      )
      raise SimulatorConnectionError(
        "Unsafe continuous evaluation origin: " + ", ".join(alignment["reasons"])
      )
    self.reference_route, self._route_cumulative_m = build_reference_route(
      waypoint, self.run.scenario.route_length_m + _ROUTE_RESERVE_M
    )
    self._route_index = 0
    self._route_origin_progress_m = 0.0
    self._route_distance_m = 0.0
    self.scenario.start_waypoint = waypoint
    self.scenario.reference_route = self.reference_route

  def _begin_continuous_evaluation_transition(self) -> None:
    """Disarm actuation without teleporting or changing vehicle kinematics."""

    if self.controller_name == "traffic_manager":
      self.vehicle.set_autopilot(False, self.config.traffic_manager_port)
    self._vc = self.carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0)
    self.vehicle.apply_control(self._vc)
    self._post_reset_speed_mps = _speed(self.vehicle.get_velocity())
    self._route_distance_m = 0.0
    self._route_index = 0
    self._route_origin_progress_m = 0.0
    self._last_lane_offset_m = 0.0
    self._geometric_lane_crossing_active = False

  def _enable_reference_controller(self) -> None:
    """Re-arm Traffic Manager after the controller-neutral settling window."""

    self.vehicle.set_autopilot(True, self.config.traffic_manager_port)
    self.traffic_manager.auto_lane_change(self.vehicle, False)
    target_kph = self.run.scenario.target_speed_mps * 3.6
    if hasattr(self.traffic_manager, "set_desired_speed"):
      self.traffic_manager.set_desired_speed(self.vehicle, target_kph)
    else:  # pragma: no cover - CARLA 0.9.16 provides set_desired_speed
      limit = max(self.vehicle.get_speed_limit(), 1.0)
      self.traffic_manager.vehicle_percentage_speed_difference(
        self.vehicle, 100.0 * (1.0 - target_kph / limit)
      )
    route_tail = self.reference_route[self._route_index + 1 :]
    if hasattr(self.traffic_manager, "set_path") and route_tail:
      self.traffic_manager.set_path(
        self.vehicle, [waypoint.transform.location for waypoint in route_tail]
      )

  def tick(self) -> None:
    if self.exit_event.is_set():
      return
    warmup = float(self.run.scenario.parameters.get("warmup_active_s", 5.0))
    begin_post_reset_settling = (
      not self._evaluation_started
      and not self._post_reset_settling
      and self._controller_active
      and self._post_reset_controller_ready()
      and self._active_duration_s + self.config.fixed_delta_seconds >= warmup
    )
    if begin_post_reset_settling:
      self._begin_continuous_evaluation_transition()
      self._post_reset_settling = True
      self._post_reset_frame = None
      self._post_reset_ticks = 0
      self._post_reset_elapsed_s = 0.0
      self._post_reset_sane_imu_ticks = 0
      self._post_reset_last_accel_norm_mps2 = None
      self._post_reset_last_gyro_norm_rad_s = None
      self._post_reset_started_wall_s = time.monotonic()
      self._post_reset_openpilot_camera_id = None
      self._post_reset_model_seen = False
      self._post_reset_control_ready = False
      self._post_reset_fresh_control_applied = False
      self._post_reset_localization_ready_at_release = None
      self._post_reset_applied_control_mono_time = None
      self._post_reset_freshness.clear()

    if self._post_reset_settling and not self._controller_active:
      self.recorder.record_event(
        EventRecord("engagement_timeout", 0.0, self._world_frame, {"phase": "post_reset"})
      )
      self._termination_reason = "engagement_timeout"
      self.exit_event.set()
      return

    if self._post_reset_settling and not self._post_reset_fresh_control_applied:
      wait_started = self._post_reset_started_wall_s or time.monotonic()
      if time.monotonic() - wait_started >= _POST_RESET_READY_TIMEOUT_S:
        self.recorder.record_event(
          EventRecord(
            "post_reset_settling_timeout",
            0.0,
            self._world_frame,
            {
              "physics_ready": self._post_reset_physics_ready(),
              "controller_ready": self._post_reset_controller_ready(),
              "sane_imu_ticks": self._post_reset_sane_imu_ticks,
            },
          )
        )
        self._termination_reason = "post_reset_settling_timeout"
        self.exit_event.set()
        return

    settling_ticks_complete = (
      self._post_reset_physics_ready() and self._post_reset_controller_ready()
    )
    if self._post_reset_waiting_for_causal_control():
      # Do not advance CARLA while the already-delivered frame is processed;
      # this preserves the same healthy settling trajectory for both arms. The
      # latch deliberately survives a transient controller/localization status
      # flicker so the causal target can never become stale.
      return

    start_evaluation = (
      self._post_reset_settling
      and settling_ticks_complete
      and (self.controller_name != "openpilot" or self._post_reset_fresh_control_applied)
    )
    if start_evaluation:
      # The preceding unrecorded tick supplied a post-reset camera/state.  The
      # first evaluated physics step therefore uses fresh controller output.
      if self.controller_name == "openpilot":
        self._post_reset_localization_ready_at_release = self._openpilot_localization_ready
      try:
        self._set_evaluation_origin()
      except SimulatorConnectionError:
        self._termination_reason = "evaluation_start_rejected"
        self.exit_event.set()
        return
      if self.controller_name == "traffic_manager":
        self._enable_reference_controller()
      self.scenario.arm()
      self._post_reset_settling = False
      self._evaluation_started = True
      self._evaluation_time_s = 0.0
      self._camera_drop_count = 0
      self._camera_timeout_count = 0
    if self._evaluation_started:
      self.scenario.update(self._evaluation_time_s)
    frame = self.world.tick()
    snapshot = self.world.get_snapshot()
    self._world_frame = int(frame)
    self._update_spectator()
    if self._carla_recorder_active:
      self._recorder_frame_index += 1
      self._recorder_world_frames[self._recorder_frame_index] = int(frame)
    elapsed = float(snapshot.timestamp.elapsed_seconds)
    if self._last_snapshot_elapsed_s is None:
      delta = self.config.fixed_delta_seconds
    else:
      delta = max(0.0, elapsed - self._last_snapshot_elapsed_s)
    self._last_snapshot_elapsed_s = elapsed
    self._startup_elapsed_s += delta
    if begin_post_reset_settling:
      self._post_reset_frame = int(frame)
      self._post_reset_start_recorder_frame = self._recorder_frame_index
    if self._post_reset_settling:
      self._post_reset_ticks += 1
      self._post_reset_elapsed_s += delta

    if self._controller_active:
      self._active_duration_s += delta
    elif not self._evaluation_started:
      self._active_duration_s = 0.0

    if start_evaluation:
      self._evaluation_start_elapsed_s = elapsed
      self._evaluation_start_frame = int(frame)
      self._evaluation_start_recorder_frame = self._recorder_frame_index
      self.recorder.record_event(EventRecord("evaluation_started", 0.0, frame))

    if not self._wait_for_imu(int(frame)):
      return
    if not self._update_post_reset_imu_health(int(frame)):
      return
    self._drain_measurement_events(int(frame))

    initial_settling_duration = 20 * self.config.fixed_delta_seconds
    if (
      self._settled_spawn_transform is None
      and not self._post_reset_settling
      and not self._evaluation_started
      and self._startup_elapsed_s + 1e-9 >= initial_settling_duration
    ):
      self._capture_settled_spawn_transform()

    if not self._evaluation_started:
      if not self._post_reset_settling and self._startup_elapsed_s >= 20.0:
        self.recorder.record_event(EventRecord("engagement_timeout", 0.0, frame))
        self._termination_reason = "engagement_timeout"
        self.exit_event.set()
      return

    start_elapsed = (
      self._evaluation_start_elapsed_s if self._evaluation_start_elapsed_s is not None else elapsed
    )
    self._evaluation_time_s = max(0.0, elapsed - start_elapsed)
    if not self._controller_active and not self._disengagement_recorded:
      self._disengagement_recorded = True
      self.recorder.record_event(
        EventRecord("controller_disengagement", self._evaluation_time_s, frame)
      )
      self._termination_reason = "controller_disengagement"
      self.exit_event.set()
    self._record_sample(frame)

  def reset(self) -> None:
    if self.controller_name == "traffic_manager":
      self.vehicle.set_autopilot(False, self.config.traffic_manager_port)
    if self._settled_spawn_transform is None:
      raise SimulatorConnectionError(
        "Cannot reset before CARLA's initial suspension-settling pose was captured"
      )
    current_transform = self.vehicle.get_transform()
    linear_components = _body_components(self.vehicle.get_velocity(), current_transform)
    angular_components = _body_components(self.vehicle.get_angular_velocity(), current_transform)
    reset_transform = self._settled_spawn_transform
    self.vehicle.set_transform(reset_transform)
    self._vc = self.carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0)
    self.vehicle.apply_control(self._vc)
    self.vehicle.set_target_velocity(_world_vector(self.carla, linear_components, reset_transform))
    self.vehicle.set_target_angular_velocity(
      _world_vector(self.carla, angular_components, reset_transform)
    )
    self._post_reset_speed_mps = math.sqrt(sum(component**2 for component in linear_components))
    self._route_distance_m = 0.0
    self._route_index = 0
    self._route_origin_progress_m = 0.0
    self._last_lane_offset_m = 0.0
    self._geometric_lane_crossing_active = False

  def _fail_recorder_audit(self, error: BaseException | str) -> dict[str, Any]:
    message = str(error)
    audit = {
      "valid": False,
      "error": message,
      "server_filename": self._carla_recorder_filename,
      "mapped_frames": self._recorder_frame_index,
    }
    self.recorder.record_event(
      EventRecord(
        "recorder_collision_audit_failed",
        self._evaluation_time_s,
        self._world_frame if self._world_frame >= 0 else None,
        audit,
      )
    )
    self._termination_reason = "recorder_collision_audit_failed"
    self.exit_event.set()
    self._recorder_audit = audit
    return audit

  def _recorder_actor_type(self, actor_id: int) -> str:
    if actor_id == 0xFFFFFFFF:
      return "static_or_environment"
    try:
      actor = self.world.get_actor(actor_id)
      if actor is not None:
        return str(actor.type_id)
    except (AttributeError, RuntimeError):
      pass
    return "unknown"

  def _audit_recorder_collisions(self) -> dict[str, Any]:
    """Use the stopped server recorder as terminal-frame collision authority."""

    if self._carla_recorder_filename is None:
      return self._fail_recorder_audit("CARLA integrity recorder was never started")
    try:
      query = self.client.show_recorder_file_info(self._carla_recorder_filename, False)
      collisions, final_frame = _parse_recorder_collisions(query)
    except (RuntimeError, TypeError, ValueError) as exc:
      return self._fail_recorder_audit(f"{type(exc).__name__}: {exc}")

    if final_frame is None:
      return self._fail_recorder_audit("CARLA recorder query omitted its final Frames count")
    if final_frame != self._recorder_frame_index:
      return self._fail_recorder_audit(
        "CARLA recorder/world frame mapping is incomplete: "
        f"recorder={final_frame}, mapped={self._recorder_frame_index}"
      )

    ego_id = int(self.vehicle.id)
    ego_collisions = [
      collision
      for collision in collisions
      if collision.actor_1_id == ego_id
      or collision.actor_2_id == ego_id
      or collision.actor_1_is_hero
      or collision.actor_2_is_hero
    ]
    existing_evaluation: set[tuple[int, int]] = set()
    existing_post_reset: set[tuple[int, int]] = set()
    for event in self.recorder.events:
      if event.frame is None:
        continue
      try:
        other_id = int(event.details.get("other_actor_id", -1))
      except (TypeError, ValueError):
        continue
      key = (int(event.frame), other_id)
      if event.event_type == "collision":
        existing_evaluation.add(key)
      elif event.event_type == "post_reset_collision":
        existing_post_reset.add(key)

    evaluation_entries = 0
    post_reset_entries = 0
    added_events = 0
    for collision in ego_collisions:
      world_frame = self._recorder_world_frames.get(collision.recorder_frame)
      if world_frame is None:
        return self._fail_recorder_audit(
          f"Recorder collision frame {collision.recorder_frame} has no CARLA frame mapping"
        )
      ego_is_actor_1 = collision.actor_1_id == ego_id or collision.actor_1_is_hero
      other_id = collision.actor_2_id if ego_is_actor_1 else collision.actor_1_id
      details = {
        "other_actor_id": other_id,
        "other_actor_type": self._recorder_actor_type(other_id),
        "impulse_ns": 0.0,
        "impulse_available": False,
        "source": "carla_server_recorder",
        "recorder_frame": collision.recorder_frame,
      }
      key = (world_frame, other_id)
      if (
        self._evaluation_start_recorder_frame is not None
        and collision.recorder_frame >= self._evaluation_start_recorder_frame
      ):
        evaluation_entries += 1
        if key not in existing_evaluation:
          sim_time = max(
            0.0,
            (world_frame - int(self._evaluation_start_frame or world_frame))
            * self.config.fixed_delta_seconds,
          )
          self.recorder.record_event(EventRecord("collision", sim_time, world_frame, details))
          existing_evaluation.add(key)
          added_events += 1
        self._collision_during_run = True
        scenario_actor = getattr(self.scenario, "_scenario_actor", None)
        if scenario_actor is not None and other_id == scenario_actor.id:
          self._scenario_actor_collision = True
      elif (
        self._post_reset_start_recorder_frame is not None
        and collision.recorder_frame >= self._post_reset_start_recorder_frame
      ):
        post_reset_entries += 1
        if key not in existing_post_reset:
          self.recorder.record_event(EventRecord("post_reset_collision", 0.0, world_frame, details))
          existing_post_reset.add(key)
          added_events += 1

    if post_reset_entries:
      self._termination_reason = "post_reset_collision"
      self.exit_event.set()
    elif (
      evaluation_entries
      and self.run.scenario.parameters.get("terminate_on_collision", True)
      and self._termination_reason in {"bridge_closed", "route_completed", "timeout"}
    ):
      self._termination_reason = "collision"
      self.exit_event.set()

    audit = {
      "valid": True,
      "server_filename": self._carla_recorder_filename,
      "server_file_retained": self.config.recording.carla_recorder,
      "recorder_frames": final_frame,
      "mapped_world_frames": len(self._recorder_world_frames),
      "raw_collision_entries": len(collisions),
      "ego_collision_entries": len(ego_collisions),
      "evaluation_collision_entries": evaluation_entries,
      "post_reset_collision_entries": post_reset_entries,
      "events_added_from_recorder": added_events,
    }
    self._recorder_audit = audit
    self.recorder.record_event(
      EventRecord("recorder_collision_audit", self._evaluation_time_s, details=audit)
    )
    return audit

  def close(self, reason: str = "closed") -> None:
    if getattr(self, "_demo", None) is not None:
      self._demo.close()
      self._demo = None
    if self._closed:
      return
    self._closed = True
    self.exit_event.set()
    self.image_lock.close()
    finalize_error: BaseException | None = None
    try:
      recorder_stop_error: BaseException | None = None
      if self._carla_recorder_active:
        try:
          self.client.stop_recorder()
        except RuntimeError as exc:
          recorder_stop_error = exc
        finally:
          self._carla_recorder_active = False
      # Sparse sensor callbacks are supplemental. The stopped server recorder
      # and synchronous lane geometry provide the authoritative safety events.
      for sensor in reversed(self._sensors):
        with suppress(AttributeError, RuntimeError):
          sensor.stop()
      self._sensors_stopped = True
      self._drain_measurement_events(self._world_frame)
      if recorder_stop_error is not None:
        self._fail_recorder_audit(
          f"stop_recorder failed: {type(recorder_stop_error).__name__}: {recorder_stop_error}"
        )
      else:
        self._audit_recorder_collisions()
      realization: dict[str, Any] | None = None
      if self.scenario.armed:
        try:
          realization = self.scenario.realization_status(self._scenario_actor_collision)
        except (AttributeError, RuntimeError) as exc:
          realization = {
            "valid": False,
            "reasons": ["realization_measurement_failed"],
            "measured": {"error": f"{type(exc).__name__}: {exc}"},
          }
        self.recorder.record_event(
          EventRecord("scenario_realization", self._evaluation_time_s, details=realization)
        )
        if not realization["valid"] and self._termination_reason in {
          "route_completed",
          "timeout",
        }:
          self._termination_reason = "scenario_realization_invalid"
      runtime_summary = {
        "carla_client_version": self.client_version,
        "carla_server_version": self.server_version,
        "world_last_frame": self._world_frame,
        "evaluation_start_frame": self._evaluation_start_frame,
        "camera_dropped_frames": self._camera_drop_count,
        "startup_camera_discarded_frames": self._startup_camera_discard_count,
        "camera_timeouts": self._camera_timeout_count,
        "camera_handoff_count": self._camera_publish_sequence,
        "last_carla_camera_frame": self._last_camera_frame,
        "last_openpilot_camera_frame_id": self._last_handoff_openpilot_frame_id,
        "camera_frame_mapping": (
          "events.jsonl camera_frame_handoff: CARLA frame -> OpenPilot camera frameId"
        ),
        "last_synchronized_imu_frame": self._latest_imu_frame,
        "recorder_collision_audit": self._recorder_audit,
        "post_reset_settling_ticks": self._post_reset_ticks,
        "post_reset_required_ticks": _POST_RESET_SETTLING_TICKS,
        "post_reset_required_sane_imu_ticks": _POST_RESET_REQUIRED_SANE_IMU_TICKS,
        "post_reset_sane_imu_ticks": self._post_reset_sane_imu_ticks,
        "post_reset_last_acceleration_norm_mps2": self._post_reset_last_accel_norm_mps2,
        "post_reset_last_gyroscope_norm_rad_s": self._post_reset_last_gyro_norm_rad_s,
        "post_reset_speed_mps": self._post_reset_speed_mps,
        "post_reset_localization_ready": self._post_reset_localization_ready_at_release
        if self.controller_name == "openpilot"
        else None,
        "post_reset_fresh_control_applied": self._post_reset_fresh_control_applied
        if self.controller_name == "openpilot"
        else None,
        "post_reset_control_lineage": self._post_reset_freshness.evidence()
        if self.controller_name == "openpilot"
        else None,
        "post_reset_applied_car_control_mono_time": self._post_reset_applied_control_mono_time
        if self.controller_name == "openpilot"
        else None,
        "route_origin_progress_m": self._route_origin_progress_m,
        "evaluation_origin_alignment": self._evaluation_origin_alignment,
        "maximum_wheel_angle_deg": self.max_wheel_angle_deg,
        "steering_ratio": self.config.actuation.steering_ratio,
        "latency_definition": "camera-release to next updated carControl message",
        "spectator_follow_configured": self.config.spectator.enabled,
        "spectator_follow_enabled": self._spectator is not None,
        "dashboard": {
          "enabled": self._dashboard is not None,
          "host": self.config.dashboard.host if self._dashboard is not None else None,
          "port": self._dashboard.port if self._dashboard is not None else None,
        },
        "scenario_realization": realization,
        "close_reason": reason,
      }
      self.recorder.finalize(self._termination_reason, runtime_summary)
    except BaseException as exc:
      finalize_error = exc
    finally:
      if self._dashboard is not None:
        with suppress(Exception):
          self._publish_dashboard_state(force=True)
          self._dashboard.close()
        self._dashboard = None
      # CARLA 0.9.16's local Traffic Manager owns a worker thread that polls
      # registered vehicles. Stop and join it before destroying those actors;
      # otherwise the worker can call GetTransform on an already-destroyed ego
      # or background actor and abort the entire Python process.
      with suppress(AttributeError, RuntimeError):
        self.vehicle.set_autopilot(False, self.config.traffic_manager_port)
      with suppress(AttributeError, RuntimeError):
        self.traffic_manager.shut_down()
      for socket in self._openpilot_causal_sockets.values():
        with suppress(AttributeError, RuntimeError):
          socket.close()
      self._openpilot_causal_sockets.clear()
      for sensor in reversed(self._sensors):
        with suppress(AttributeError, RuntimeError):
          sensor.destroy()
      self._sensors.clear()
      with suppress(AttributeError, RuntimeError):
        self.scenario.close()
      with suppress(AttributeError, RuntimeError):
        self.vehicle.destroy()
      with suppress(AttributeError, RuntimeError):
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
      self._notify("close", self._termination_reason)
    if finalize_error is not None:
      raise finalize_error


def load_summary(path: Path) -> dict[str, Any]:
  return json.loads(path.read_text(encoding="utf-8"))
