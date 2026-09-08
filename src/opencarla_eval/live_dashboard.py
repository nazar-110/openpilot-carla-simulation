"""Local read-only dashboard for OpenPilot perception, planning, and control."""

from __future__ import annotations

import io
import json
import math
import threading
import time
from collections.abc import Mapping, Sequence
from contextlib import suppress
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np


def _finite(value: Any) -> float | None:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return None
  return result if math.isfinite(result) else None


def _enum(value: Any) -> str:
  # pycapnp's ``raw`` property is the numeric ordinal; its string form carries
  # the schema enum name that an operator can actually understand.
  text = str(value).strip()
  if text and not text.startswith("<"):
    return text.rsplit(".", maxsplit=1)[-1]
  return str(getattr(value, "raw", "unknown"))


def _series(values: Any, limit: int = 64) -> list[float]:
  result: list[float] = []
  try:
    iterator = iter(values)
  except TypeError:
    return result
  for value in iterator:
    finite = _finite(value)
    if finite is not None:
      result.append(finite)
    if len(result) >= limit:
      break
  return result


def _xyzt(value: Any) -> dict[str, list[float]]:
  return {
    "x": _series(getattr(value, "x", [])),
    "y": _series(getattr(value, "y", [])),
    "z": _series(getattr(value, "z", [])),
    "t": _series(getattr(value, "t", [])),
  }


def _paths(values: Any, probabilities: Any = ()) -> list[dict[str, Any]]:
  result: list[dict[str, Any]] = []
  probability_values = _series(probabilities, limit=16)
  try:
    iterator = iter(values)
  except TypeError:
    return result
  for index, value in enumerate(iterator):
    result.append(
      {
        **_xyzt(value),
        "probability": probability_values[index] if index < len(probability_values) else None,
      }
    )
  return result


def _camera_projection(runtime: Mapping[str, Any]) -> dict[str, float]:
  """Return the raw road-camera pinhole geometry used by the live overlay."""

  supplied = runtime.get("camera_projection", {})
  if not isinstance(supplied, Mapping):
    supplied = {}

  width = _finite(supplied.get("width_px")) or 1928.0
  height = _finite(supplied.get("height_px")) or 1208.0
  horizontal_fov = _finite(supplied.get("horizontal_fov_deg")) or 40.0
  half_width = _finite(supplied.get("path_half_width_m")) or 0.9
  if width <= 0.0:
    width = 1928.0
  if height <= 0.0:
    height = 1208.0
  if not 1.0 <= horizontal_fov < 180.0:
    horizontal_fov = 40.0
  if half_width <= 0.0:
    half_width = 0.9

  return {
    "width_px": width,
    "height_px": height,
    "horizontal_fov_deg": horizontal_fov,
    "focal_length_px": width / (2.0 * math.tan(math.radians(horizontal_fov) / 2.0)),
    "path_half_width_m": half_width,
  }


def _project_path_ribbon(
  trajectory: Mapping[str, Sequence[float]],
  *,
  rpy_calib: Sequence[float],
  camera_height_m: float,
  camera_projection: Mapping[str, float],
) -> list[list[float]]:
  """Project OpenPilot's car-space path ribbon into raw road-camera pixels.

  OpenPilot calibration coordinates use x forward, y right, and z down.  This
  mirrors its on-road UI: rotate calibration into device space, add camera
  height to model z, convert device to view coordinates, and apply the pinhole
  intrinsics.  The returned points form one closed-polygon winding.
  """

  xs = list(trajectory.get("x", ()))
  ys = list(trajectory.get("y", ()))
  zs = list(trajectory.get("z", ()))
  count = min(len(xs), len(ys))
  if count < 2:
    return []

  roll, pitch, yaw = (list(rpy_calib) + [0.0, 0.0, 0.0])[:3]
  cr, sr = math.cos(roll), math.sin(roll)
  cp, sp = math.cos(pitch), math.sin(pitch)
  cy, sy = math.cos(yaw), math.sin(yaw)

  width = float(camera_projection["width_px"])
  height = float(camera_projection["height_px"])
  focal = float(camera_projection["focal_length_px"])
  half_width = float(camera_projection["path_half_width_m"])
  center_x, center_y = width / 2.0, height / 2.0
  clip_margin = 500.0

  finite_xs = [float(value) for value in xs[:count] if math.isfinite(float(value))]
  if not finite_xs:
    return []
  max_distance = min(100.0, max(10.0, finite_xs[-1]))

  def project(x: float, y: float, z: float) -> tuple[float, float] | None:
    # Rz(yaw) @ Ry(pitch) @ Rx(roll), matching rot_from_euler in OpenPilot.
    rx_x = x
    rx_y = cr * y - sr * z
    rx_z = sr * y + cr * z
    ry_x = cp * rx_x + sp * rx_z
    ry_y = rx_y
    ry_z = -sp * rx_x + cp * rx_z
    device_x = cy * ry_x - sy * ry_y
    device_y = sy * ry_x + cy * ry_y
    device_z = ry_z
    if device_x <= 0.5:
      return None
    return (
      center_x + focal * device_y / device_x,
      center_y + focal * device_z / device_x,
    )

  left: list[tuple[float, float]] = []
  right: list[tuple[float, float]] = []
  minimum_screen_y = math.inf
  for index in range(count):
    try:
      x = float(xs[index])
      y = float(ys[index])
      z = float(zs[index]) if index < len(zs) else 0.0
    except (TypeError, ValueError):
      continue
    if not all(math.isfinite(value) for value in (x, y, z)) or not 0.0 <= x <= max_distance:
      continue
    left_point = project(x, y - half_width, z + camera_height_m)
    right_point = project(x, y + half_width, z + camera_height_m)
    if left_point is None or right_point is None:
      continue
    if not all(
      -clip_margin <= point[0] <= width + clip_margin
      and -clip_margin <= point[1] <= height + clip_margin
      for point in (left_point, right_point)
    ):
      continue
    # The OpenPilot renderer drops points that would fold the polygon back down
    # the image on crests or noisy z estimates.
    if left_point[1] > minimum_screen_y:
      continue
    minimum_screen_y = left_point[1]
    left.append(left_point)
    right.append(right_point)

  if len(left) < 2:
    return []
  return [[float(x), float(y)] for x, y in (*left, *reversed(right))]


def _lead_snapshot(lead: Any) -> dict[str, Any]:
  def first(name: str) -> float | None:
    values = _series(getattr(lead, name, []), limit=1)
    return values[0] if values else None

  return {
    "probability": _finite(getattr(lead, "prob", None)),
    "probability_time_s": _finite(getattr(lead, "probTime", None)),
    "x_m": first("x"),
    "y_m": first("y"),
    "speed_mps": first("v"),
    "acceleration_mps2": first("a"),
  }


def build_openpilot_dashboard_state(
  messages: Mapping[str, Any],
  valid: Mapping[str, bool],
  runtime: Mapping[str, Any],
) -> dict[str, Any]:
  """Detach the latest OpenPilot messages into JSON-safe diagnostic values."""

  model = messages.get("modelV2", SimpleNamespace())
  plan = messages.get("longitudinalPlan", SimpleNamespace())
  controls = messages.get("controlsState", SimpleNamespace())
  car_control = messages.get("carControl", SimpleNamespace())
  selfdrive = messages.get("selfdriveState", SimpleNamespace())
  car_state = messages.get("carState", SimpleNamespace())
  calibration = messages.get("liveCalibration", SimpleNamespace())
  actuators = getattr(car_control, "actuators", SimpleNamespace())
  action = getattr(model, "action", SimpleNamespace())
  meta = getattr(model, "meta", SimpleNamespace())
  trajectory = _xyzt(getattr(model, "position", SimpleNamespace()))
  calibration_rpy = _series(getattr(calibration, "rpyCalib", []), limit=3)
  calibration_height = (_series(getattr(calibration, "height", []), limit=1) or [1.22])[0]
  calibration_status = _enum(getattr(calibration, "calStatus", "unknown"))
  projection = _camera_projection(runtime)
  calibration_applied = (
    bool(valid.get("liveCalibration", False))
    and calibration_status.lower() == "calibrated"
    and len(calibration_rpy) == 3
  )
  path_polygon = _project_path_ribbon(
    trajectory,
    rpy_calib=calibration_rpy if calibration_applied else (0.0, 0.0, 0.0),
    camera_height_m=calibration_height,
    camera_projection=projection,
  )

  try:
    leads = [_lead_snapshot(value) for value in list(getattr(model, "leadsV3", []))[:3]]
  except TypeError:
    leads = []
  planned_speeds = _series(getattr(plan, "speeds", []))
  planned_accelerations = _series(getattr(plan, "accels", []))
  planned_jerks = _series(getattr(plan, "jerks", []))
  planning_points = max(len(planned_speeds), len(planned_accelerations), len(planned_jerks))

  return {
    "runtime": dict(runtime),
    "services": {name: bool(valid.get(name, False)) for name in sorted(messages)},
    "model": {
      "valid": bool(valid.get("modelV2", False)),
      "frame_id": int(getattr(model, "frameId", -1)),
      "frame_id_extra": int(getattr(model, "frameIdExtra", -1)),
      "frame_age": int(getattr(model, "frameAge", 0)),
      "frame_drop_percent": _finite(getattr(model, "frameDropPerc", None)),
      "execution_time_ms": (
        1000.0 * float(getattr(model, "modelExecutionTime", 0.0))
        if _finite(getattr(model, "modelExecutionTime", None)) is not None
        else None
      ),
      "trajectory": trajectory,
      "trajectory_overlay": {
        "polygon_px": path_polygon,
        "calibration_applied": calibration_applied,
      },
      "camera_projection": projection,
      "lane_lines": _paths(getattr(model, "laneLines", []), getattr(model, "laneLineProbs", [])),
      "road_edges": _paths(getattr(model, "roadEdges", [])),
      "leads": leads,
      "action": {
        "desired_curvature": _finite(getattr(action, "desiredCurvature", None)),
        "desired_acceleration_mps2": _finite(getattr(action, "desiredAcceleration", None)),
        "should_stop": bool(getattr(action, "shouldStop", False)),
      },
      "lane_change_state": _enum(getattr(meta, "laneChangeState", "unknown")),
      "lane_change_direction": _enum(getattr(meta, "laneChangeDirection", "none")),
      "hard_brake_predicted": bool(getattr(meta, "hardBrakePredicted", False)),
    },
    "planning": {
      "valid": bool(valid.get("longitudinalPlan", False)),
      "model_mono_time": int(getattr(plan, "modelMonoTime", 0)),
      "source": _enum(getattr(plan, "longitudinalPlanSource", "unknown")),
      "has_lead": bool(getattr(plan, "hasLead", False)),
      "fcw": bool(getattr(plan, "fcw", False)),
      "should_stop": bool(getattr(plan, "shouldStop", False)),
      "allow_throttle": bool(getattr(plan, "allowThrottle", False)),
      "allow_brake": bool(getattr(plan, "allowBrake", False)),
      "target_acceleration_mps2": _finite(getattr(plan, "aTarget", None)),
      "times_s": [10.0 * (index / 32.0) ** 2 for index in range(planning_points)],
      "speeds_mps": planned_speeds,
      "accelerations_mps2": planned_accelerations,
      "jerks_mps3": planned_jerks,
    },
    "control": {
      "controls_valid": bool(valid.get("controlsState", False)),
      "car_control_valid": bool(valid.get("carControl", False)),
      "desired_curvature": _finite(getattr(controls, "desiredCurvature", None)),
      "measured_curvature": _finite(getattr(controls, "curvature", None)),
      "long_control_state": _enum(getattr(controls, "longControlState", "unknown")),
      "force_deceleration": bool(getattr(controls, "forceDecel", False)),
      "enabled": bool(getattr(car_control, "enabled", False)),
      "lateral_active": bool(getattr(car_control, "latActive", False)),
      "longitudinal_active": bool(getattr(car_control, "longActive", False)),
      "command_acceleration_mps2": _finite(getattr(actuators, "accel", None)),
      "command_steering_angle_deg": _finite(getattr(actuators, "steeringAngleDeg", None)),
      "command_curvature": _finite(getattr(actuators, "curvature", None)),
      "command_torque": _finite(getattr(actuators, "torque", None)),
      "command_long_state": _enum(getattr(actuators, "longControlState", "unknown")),
    },
    "selfdrive": {
      "valid": bool(valid.get("selfdriveState", False)),
      "state": _enum(getattr(selfdrive, "state", "unknown")),
      "enabled": bool(getattr(selfdrive, "enabled", False)),
      "active": bool(getattr(selfdrive, "active", False)),
      "engageable": bool(getattr(selfdrive, "engageable", False)),
      "alert_text_1": str(getattr(selfdrive, "alertText1", "")),
      "alert_text_2": str(getattr(selfdrive, "alertText2", "")),
      "alert_type": str(getattr(selfdrive, "alertType", "")),
      "experimental_mode": bool(getattr(selfdrive, "experimentalMode", False)),
    },
    "vehicle": {
      "speed_mps": _finite(getattr(car_state, "vEgo", runtime.get("speed_mps"))),
      "acceleration_mps2": _finite(getattr(car_state, "aEgo", None)),
      "steering_angle_deg": _finite(getattr(car_state, "steeringAngleDeg", None)),
    },
    "calibration": {
      "valid": bool(valid.get("liveCalibration", False)),
      "status": calibration_status,
      "rpy_calib": calibration_rpy,
      "height_m": calibration_height,
    },
  }


def _json_safe(value: Any) -> Any:
  if isinstance(value, Mapping):
    return {str(key): _json_safe(item) for key, item in value.items()}
  if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
    return [_json_safe(item) for item in value]
  # bool is a subclass of int, so preserve it before the numeric branches.
  if isinstance(value, (bool, np.bool_)):
    return bool(value)
  if isinstance(value, (float, np.floating)):
    return float(value) if math.isfinite(float(value)) else None
  if isinstance(value, (int, np.integer)):
    return int(value)
  if isinstance(value, str) or value is None:
    return value
  return str(value)


class _DashboardHTTPServer(ThreadingHTTPServer):
  daemon_threads = True
  allow_reuse_address = True


class LiveDashboard:
  """Latest-value HTTP publisher with a non-blocking asynchronous JPEG worker."""

  def __init__(
    self,
    host: str,
    port: int,
    *,
    frame_stride: int,
    downsample: int,
    jpeg_quality: int,
  ) -> None:
    self.host = host
    self.requested_port = port
    self.frame_stride = frame_stride
    self.downsample = downsample
    self.jpeg_quality = jpeg_quality
    self._lock = threading.Lock()
    self._frame_condition = threading.Condition()
    self._state: dict[str, Any] = {"runtime": {"phase": "starting"}}
    self._state_json = json.dumps(self._state, separators=(",", ":")).encode()
    self._frames: dict[str, tuple[bytes, dict[str, int]]] = {}
    self._pending_frames: dict[str, tuple[np.ndarray, dict[str, int]]] = {}
    self._stop = threading.Event()
    self._server: _DashboardHTTPServer | None = None
    self._server_thread: threading.Thread | None = None
    self._encoder_thread: threading.Thread | None = None
    self._server_thread_started = False
    self._encoder_thread_started = False
    self._encoder_error: str | None = None
    self._encoder_last_success_epoch_s: float | None = None
    self._image_module: Any | None = None
    self._html = Path(__file__).with_name("dashboard.html").read_bytes()

  @property
  def port(self) -> int:
    return int(self._server.server_address[1]) if self._server is not None else self.requested_port

  def start(self) -> None:
    if self._server is not None:
      return
    from PIL import Image

    self._image_module = Image
    dashboard = self

    class Handler(BaseHTTPRequestHandler):
      def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", maxsplit=1)[0]
        if path in {"/", "/index.html"}:
          self._send(HTTPStatus.OK, "text/html; charset=utf-8", dashboard._html)
          return
        if path == "/health":
          with dashboard._lock:
            health = {
              "ok": dashboard._encoder_error is None,
              "encoder_error": dashboard._encoder_error,
              "encoder_last_success_epoch_s": dashboard._encoder_last_success_epoch_s,
            }
          self._send(
            HTTPStatus.OK,
            "application/json",
            json.dumps(health, separators=(",", ":")).encode(),
          )
          return
        if path == "/api/state":
          with dashboard._lock:
            body = dashboard._state_json
          self._send(HTTPStatus.OK, "application/json", body)
          return
        if path.startswith("/camera/") and path.endswith(".jpg"):
          stream = path.removeprefix("/camera/").removesuffix(".jpg")
          with dashboard._lock:
            frame = dashboard._frames.get(stream)
          if frame is None:
            self._send(HTTPStatus.NO_CONTENT, "image/jpeg", b"")
            return
          body, metadata = frame
          headers = {
            "X-Carla-Frame": str(metadata["carla_frame"]),
            "X-Openpilot-Frame": str(metadata["openpilot_frame"]),
          }
          self._send(HTTPStatus.OK, "image/jpeg", body, headers)
          return
        self._send(HTTPStatus.NOT_FOUND, "text/plain; charset=utf-8", b"not found")

      def _send(
        self,
        status: HTTPStatus,
        content_type: str,
        body: bytes,
        extra_headers: Mapping[str, str] | None = None,
      ) -> None:
        self.send_response(int(status))
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("X-Content-Type-Options", "nosniff")
        for name, value in (extra_headers or {}).items():
          self.send_header(name, value)
        self.end_headers()
        if body:
          with suppress(BrokenPipeError, ConnectionResetError):
            self.wfile.write(body)

      def log_message(self, _format: str, *_args: Any) -> None:
        return

    self._server = _DashboardHTTPServer((self.host, self.requested_port), Handler)
    self._server_thread = threading.Thread(
      target=self._server.serve_forever, name="opencarla-dashboard-http", daemon=True
    )
    self._encoder_thread = threading.Thread(
      target=self._encode_frames, name="opencarla-dashboard-jpeg", daemon=True
    )
    self._server_thread.start()
    self._server_thread_started = True
    self._encoder_thread.start()
    self._encoder_thread_started = True

  def publish_state(self, state: Mapping[str, Any]) -> None:
    safe = _json_safe(state)
    with self._lock:
      encoder_error = self._encoder_error
      encoder_last_success = self._encoder_last_success_epoch_s
    runtime = safe.setdefault("runtime", {})
    runtime["dashboard_encoder_ok"] = encoder_error is None
    runtime["dashboard_encoder_last_success_epoch_s"] = encoder_last_success
    if encoder_error is not None:
      runtime["dashboard_warning"] = encoder_error
    body = json.dumps(safe, separators=(",", ":")).encode()
    with self._lock:
      self._state = safe
      self._state_json = body

  def publish_frame(
    self,
    stream: str,
    rgb: np.ndarray,
    *,
    carla_frame: int,
    openpilot_frame: int,
  ) -> None:
    if openpilot_frame % self.frame_stride:
      return
    # Keep this producer path O(1). The CARLA adapter hands us a fresh immutable
    # frame array; the worker owns the downsampled view until encoding finishes.
    display = rgb[:: self.downsample, :: self.downsample, :3]
    metadata = {"carla_frame": int(carla_frame), "openpilot_frame": int(openpilot_frame)}
    with self._frame_condition:
      self._pending_frames[stream] = (display, metadata)
      self._frame_condition.notify()

  def _encode_frames(self) -> None:
    while not self._stop.is_set():
      with self._frame_condition:
        self._frame_condition.wait_for(
          lambda: bool(self._pending_frames) or self._stop.is_set(), timeout=0.25
        )
        pending = self._pending_frames
        self._pending_frames = {}
      for stream, (rgb, metadata) in pending.items():
        try:
          rgb = np.ascontiguousarray(rgb)
          buffer = io.BytesIO()
          self._image_module.fromarray(rgb).save(
            buffer, format="JPEG", quality=self.jpeg_quality, optimize=False
          )
          with self._lock:
            self._frames[stream] = (buffer.getvalue(), metadata)
            self._encoder_error = None
            self._encoder_last_success_epoch_s = time.time()
        except Exception as exc:  # Dashboard work must never stop the evaluator.
          with self._lock:
            self._encoder_error = f"JPEG encoder: {type(exc).__name__}: {exc}"
            state = dict(self._state)
            runtime = dict(state.get("runtime", {}))
            runtime["dashboard_encoder_ok"] = False
            runtime["dashboard_warning"] = self._encoder_error
            state["runtime"] = runtime
            self._state = state
            self._state_json = json.dumps(state, separators=(",", ":")).encode()

  def close(self) -> None:
    if self._stop.is_set():
      return
    self._stop.set()
    with self._frame_condition:
      self._frame_condition.notify_all()
    if self._server is not None:
      if self._server_thread_started:
        self._server.shutdown()
      self._server.server_close()
    if self._server_thread_started and self._server_thread is not None:
      self._server_thread.join(timeout=2.0)
    if self._encoder_thread_started and self._encoder_thread is not None:
      self._encoder_thread.join(timeout=2.0)
