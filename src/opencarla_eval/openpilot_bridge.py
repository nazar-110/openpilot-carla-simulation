"""Port of CARLA to OpenPilot v0.11.1's current simulator ``World`` API."""

from __future__ import annotations

import argparse
import json
import multiprocessing
import os
import signal
import subprocess
import sys
import time
from collections.abc import Iterator, Sequence
from contextlib import contextmanager, suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO

import numpy as np

from .attempts import archive_existing_attempt
from .carla_world import CarlaEvaluationWorld, load_summary
from .config import expand_runs, load_experiment
from .errors import DependencyUnavailableError, EvaluationError, SimulatorConnectionError
from .models import RunSpec
from .provenance import require_reusable_summary
from .run_failure import mark_existing_run_invalid, write_invalid_run
from .runtime_config import CarlaRuntimeConfig, load_runtime_config

TESTED_OPENPILOT_COMMIT = "4df40d2c1946a57242230186edd073c4073060a6"  # v0.11.1


def _camera_timestamp_ns() -> int:
  """Use the same monotonic clock domain as OpenPilot's Python sensor shim."""

  return time.monotonic_ns()


def _pad_nv12_for_openpilot(
  yuv: bytes,
  width: int,
  height: int,
  stride: int,
  y_height: int,
  uv_height: int,
  yuv_size: int,
) -> bytes:
  """Repack tightly stored NV12 into OpenPilot's aligned camerad layout."""

  tight_size = width * height * 3 // 2
  if len(yuv) == yuv_size:
    return yuv
  if len(yuv) != tight_size:
    raise EvaluationError(
      f"Unexpected simulator NV12 size {len(yuv)}; expected {tight_size} or {yuv_size}"
    )
  if stride < width or y_height < height or uv_height < height // 2:
    raise EvaluationError("OpenPilot NV12 layout cannot contain the simulator image")
  uv_offset = stride * y_height
  if yuv_size < uv_offset + stride * uv_height:
    raise EvaluationError("OpenPilot NV12 allocation is smaller than its declared planes")

  source = np.frombuffer(yuv, dtype=np.uint8)
  padded = np.zeros(yuv_size, dtype=np.uint8)
  padded[:uv_offset].reshape(y_height, stride)[:height, :width] = source[: width * height].reshape(
    height, width
  )
  padded[uv_offset : uv_offset + stride * uv_height].reshape(uv_height, stride)[
    : height // 2, :width
  ] = source[width * height :].reshape(height // 2, width)
  return padded.tobytes()


def _install_openpilot_camerad_buffer_compatibility() -> None:
  """Make simulated VisionIPC buffers match the compiled model's NV12 layout."""

  from cereal import messaging
  from msgq.visionipc import VisionIpcServer, VisionStreamType
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  from openpilot.tools.sim.lib.camerad import Camerad
  from openpilot.tools.sim.lib.common import H, W

  if getattr(Camerad, "_opencarla_padded_nv12_compatible", False):
    return

  def __init__(self: Any, dual_camera: bool) -> None:
    self.pm = messaging.PubMaster(["roadCameraState", "wideRoadCameraState"])
    self.frame_road_id = 0
    self.frame_wide_id = 0
    self.vipc_server = VisionIpcServer("camerad")

    stride, y_height, uv_height, yuv_size = get_nv12_info(W, H)
    uv_offset = stride * y_height
    self._opencarla_nv12_layout = (W, H, stride, y_height, uv_height, yuv_size)
    self.vipc_server.create_buffers_with_sizes(
      VisionStreamType.VISION_STREAM_ROAD,
      5,
      W,
      H,
      yuv_size,
      stride,
      uv_offset,
    )
    if dual_camera:
      self.vipc_server.create_buffers_with_sizes(
        VisionStreamType.VISION_STREAM_WIDE_ROAD,
        5,
        W,
        H,
        yuv_size,
        stride,
        uv_offset,
      )
    self.vipc_server.start_listener()

  Camerad.__init__ = __init__
  Camerad._opencarla_padded_nv12_compatible = True


def _install_openpilot_camerad_timestamp_compatibility() -> None:
  """Give simulator VisionIPC frames valid OpenPilot monotonic timestamps.

  The pinned simulator derives timestamps from a frame counter that starts at
  zero. Current locationd compares camera odometry with CLOCK_BOOTTIME and
  rejects those frames as older than its rewind window. Keep the upstream
  camera payload and frame IDs unchanged, but timestamp each frame in the same
  clock domain as cereal messages.
  """

  from cereal import messaging
  from openpilot.tools.sim.lib.camerad import Camerad

  if getattr(Camerad._send_yuv, "_opencarla_boottime_compatible", False):
    return

  def _send_yuv(self: Any, yuv: bytes, frame_id: int, pub_type: str, yuv_type: Any) -> None:
    layout = getattr(self, "_opencarla_nv12_layout", None)
    if layout is not None:
      yuv = _pad_nv12_for_openpilot(yuv, *layout)
    if getattr(self, "_opencarla_timestamp_frame_id", None) != frame_id:
      self._opencarla_timestamp_frame_id = frame_id
      self._opencarla_timestamp_eof = _camera_timestamp_ns()
    eof = self._opencarla_timestamp_eof
    self.vipc_server.send(yuv_type, yuv, frame_id, eof, eof)

    dat = messaging.new_message(pub_type, valid=True)
    setattr(
      dat,
      pub_type,
      {
        "frameId": frame_id,
        "timestampSof": eof,
        "timestampEof": eof,
        "transform": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
      },
    )
    self.pm.send(pub_type, dat)

  _send_yuv._opencarla_boottime_compatible = True  # type: ignore[attr-defined]
  Camerad._send_yuv = _send_yuv


def _install_openpilot_sensor_rate_compatibility() -> None:
  """Publish simulator inertial and GPS samples at their declared service rates.

  OpenPilot v0.11.1's simulator helper emits five copies of every IMU message
  and ten copies of every GPS message on each 100 Hz bridge iteration.  CARLA
  supplies one new IMU measurement per 20 Hz physics step, so those bursts can
  turn a single spawn/reset transient into enough consecutive invalid samples
  for locationd to reject the sensor.  Preserve the upstream payloads while
  publishing IMU at 100 Hz and rate-limiting GPS to 10 Hz.
  """

  from cereal import log, messaging
  from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors

  if getattr(SimulatedSensors, "_opencarla_sensor_rate_compatible", False):
    return

  def send_imu_message(self: Any, simulator_state: Any) -> None:
    accelerometer = messaging.new_message("accelerometer", valid=True)
    accelerometer.accelerometer.timestamp = accelerometer.logMonoTime
    accelerometer.accelerometer.init("acceleration")
    accelerometer.accelerometer.acceleration.v = [
      simulator_state.imu.accelerometer.x,
      simulator_state.imu.accelerometer.y,
      simulator_state.imu.accelerometer.z,
    ]
    self.pm.send("accelerometer", accelerometer)

    gyroscope = messaging.new_message("gyroscope", valid=True)
    gyroscope.gyroscope.timestamp = gyroscope.logMonoTime
    gyroscope.gyroscope.init("gyroUncalibrated")
    gyroscope.gyroscope.gyroUncalibrated.v = [
      simulator_state.imu.gyroscope.x,
      simulator_state.imu.gyroscope.y,
      simulator_state.imu.gyroscope.z,
    ]
    self.pm.send("gyroscope", gyroscope)

  def send_gps_message(self: Any, simulator_state: Any) -> None:
    if not simulator_state.valid:
      return
    now_ns = time.monotonic_ns()
    previous_ns = getattr(self, "_opencarla_last_gps_ns", None)
    if previous_ns is not None and now_ns - previous_ns < 100_000_000:
      return
    self._opencarla_last_gps_ns = now_ns

    # Transform CARLA velocity into the NED convention used by OpenPilot.
    velocity_ned = [
      -simulator_state.velocity.y,
      simulator_state.velocity.x,
      simulator_state.velocity.z,
    ]
    gps = messaging.new_message("gpsLocationExternal", valid=True)
    gps.gpsLocationExternal = {
      "unixTimestampMillis": int(time.time() * 1000),  # noqa: TID251
      "flags": 1,
      "horizontalAccuracy": 1.0,
      "verticalAccuracy": 1.0,
      "speedAccuracy": 0.1,
      "bearingAccuracyDeg": 0.1,
      "vNED": velocity_ned,
      "bearingDeg": simulator_state.imu.bearing,
      "latitude": simulator_state.gps.latitude,
      "longitude": simulator_state.gps.longitude,
      "altitude": simulator_state.gps.altitude,
      "speed": simulator_state.speed,
      "source": log.GpsLocationData.SensorSource.ublox,
    }
    self.pm.send("gpsLocationExternal", gps)

  SimulatedSensors.send_imu_message = send_imu_message
  SimulatedSensors.send_gps_message = send_gps_message
  SimulatedSensors._opencarla_sensor_rate_compatible = True


@dataclass(frozen=True)
class OpenPilotEnvironment:
  root: Path
  commit: str
  dirty: bool | None
  valid_for_research: bool


@dataclass
class _ManagedOpenPilot:
  process: subprocess.Popen[bytes]
  log_file: BinaryIO
  log_path: Path
  exit_code_before_stop: int | None = None


def _running_openpilot_managers(root: Path) -> list[int]:
  """Find manager.py processes whose working directory is this checkout."""

  if os.name != "posix" or not Path("/proc").is_dir():
    return []
  expected_cwd = (root / "system" / "manager").resolve()
  matches: list[int] = []
  for process_dir in Path("/proc").glob("[0-9]*"):
    try:
      pid = int(process_dir.name)
      if pid == os.getpid() or (process_dir / "cwd").resolve() != expected_cwd:
        continue
      command = (process_dir / "cmdline").read_bytes().replace(b"\0", b" ")
      if b"manager.py" in command:
        matches.append(pid)
    except (FileNotFoundError, OSError, PermissionError, ValueError):
      continue
  return sorted(matches)


def _stop_openpilot_manager(manager: _ManagedOpenPilot) -> None:
  process = manager.process
  try:
    manager.exit_code_before_stop = process.poll()
    if manager.exit_code_before_stop is None:
      try:
        os.killpg(process.pid, signal.SIGTERM)
      except (AttributeError, OSError):
        process.terminate()
      try:
        process.wait(timeout=30)
      except subprocess.TimeoutExpired:
        try:
          os.killpg(process.pid, signal.SIGKILL)
        except (AttributeError, OSError):
          process.kill()
        process.wait(timeout=10)
  finally:
    manager.log_file.close()


def _wait_for_openpilot_manager_ready(manager: _ManagedOpenPilot, timeout_s: float = 60.0) -> None:
  """Wait until manager initialization/parameter clearing has completed."""

  import cereal.messaging as messaging

  socket = messaging.sub_sock("managerState", conflate=True, timeout=1000)
  deadline = time.monotonic() + timeout_s
  try:
    while time.monotonic() < deadline:
      exit_code = manager.process.poll()
      if exit_code is not None:
        raise SimulatorConnectionError(
          f"OpenPilot manager exited during startup with code {exit_code}; "
          f"inspect {manager.log_path}"
        )
      message = messaging.recv_one(socket)
      if message is not None and bool(message.valid):
        return
  finally:
    with suppress(AttributeError, RuntimeError):
      socket.close()
  raise SimulatorConnectionError(
    f"OpenPilot manager did not publish managerState within {timeout_s:.0f} s; "
    f"inspect {manager.log_path}"
  )


@contextmanager
def _managed_openpilot_process(
  environment: OpenPilotEnvironment, output_dir: Path
) -> Iterator[_ManagedOpenPilot]:
  existing = _running_openpilot_managers(environment.root)
  if existing:
    raise DependencyUnavailableError(
      "A pre-existing OpenPilot manager is running for this checkout "
      f"(PID(s) {existing}). Stop it: this evaluator owns and restarts manager/modeld "
      "for every OpenPilot trial to prevent cross-run state and VisionIPC reuse."
    )
  output_dir.mkdir(parents=True, exist_ok=True)
  log_path = output_dir / "openpilot_manager.log"
  log_file = log_path.open("wb")
  manager_env = os.environ.copy()
  openpilot_venv = environment.root / ".venv"
  venv_bin = openpilot_venv / "bin"
  manager_env["VIRTUAL_ENV"] = str(openpilot_venv)
  manager_env["PATH"] = os.pathsep.join([str(venv_bin), manager_env.get("PATH", "")]).rstrip(
    os.pathsep
  )
  blocked = {name for name in manager_env.get("BLOCK", "").split(",") if name}
  blocked.update({"soundd", "ui"})
  manager_env["BLOCK"] = ",".join(sorted(blocked))
  try:
    process = subprocess.Popen(
      ["bash", str(environment.root / "tools" / "sim" / "launch_openpilot.sh")],
      cwd=environment.root,
      env=manager_env,
      stdout=log_file,
      stderr=subprocess.STDOUT,
      start_new_session=True,
    )
  except BaseException:
    log_file.close()
    raise
  manager = _ManagedOpenPilot(process, log_file, log_path)
  try:
    _wait_for_openpilot_manager_ready(manager)
    yield manager
  finally:
    with suppress(OSError, subprocess.SubprocessError):
      _stop_openpilot_manager(manager)


def _openpilot_root() -> Path | None:
  explicit = os.getenv("OPENPILOT_ROOT")
  candidates = [Path(explicit)] if explicit else []
  candidates.extend([Path.cwd(), Path.cwd().parent])
  for candidate in candidates:
    root = candidate.expanduser().resolve()
    if (root / "tools" / "sim" / "launch_openpilot.sh").is_file() and (
      root / "system" / "manager" / "manager.py"
    ).is_file():
      return root
  return None


def _git_commit(root: Path) -> str | None:
  try:
    result = subprocess.run(
      ["git", "rev-parse", "HEAD"],
      cwd=root,
      check=True,
      capture_output=True,
      text=True,
      timeout=30,
    )
  except (OSError, subprocess.SubprocessError):
    return None
  return result.stdout.strip()


def _git_dirty(root: Path) -> bool | None:
  try:
    result = subprocess.run(
      ["git", "status", "--porcelain"],
      cwd=root,
      check=True,
      capture_output=True,
      text=True,
      timeout=30,
    )
  except (OSError, subprocess.SubprocessError):
    return None
  return bool(result.stdout.strip())


def verify_openpilot_environment() -> OpenPilotEnvironment:
  root = _openpilot_root()
  if root is None:
    raise DependencyUnavailableError(
      "Cannot locate the OpenPilot checkout; run from its root or set OPENPILOT_ROOT"
    )
  if str(root) not in sys.path:
    sys.path.insert(0, str(root))
  try:
    from openpilot.tools.sim.bridge.common import SimulatorBridge  # noqa: F401
  except ImportError as exc:
    raise DependencyUnavailableError(
      "OpenPilot simulator modules are unavailable. Run this command inside the "
      "OpenPilot v0.11.1 virtual environment with this project installed editable."
    ) from exc
  commit = _git_commit(root)
  dirty = _git_dirty(root)
  override = os.getenv("OPENCARLA_ALLOW_UNTESTED_OPENPILOT") == "1"
  if (commit != TESTED_OPENPILOT_COMMIT or dirty is not False) and not override:
    raise DependencyUnavailableError(
      f"This bridge is tested against OpenPilot v0.11.1 ({TESTED_OPENPILOT_COMMIT}), "
      f"but {root} is at {commit or 'an unknown commit'} with "
      f"worktree_dirty={dirty!r}. Check out the exact clean commit or explicitly set "
      "OPENCARLA_ALLOW_UNTESTED_OPENPILOT=1 (outputs will be non-research-valid)."
    )
  if commit is None:
    raise DependencyUnavailableError("Cannot determine the OpenPilot Git commit")
  return OpenPilotEnvironment(
    root,
    commit,
    dirty,
    commit == TESTED_OPENPILOT_COMMIT and dirty is False and not override,
  )


def _restore_async(client: Any, config: CarlaRuntimeConfig) -> None:
  try:
    manager = client.get_trafficmanager(config.traffic_manager_port)
    manager.set_synchronous_mode(False)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
  except Exception:
    pass


def _make_bridge(
  run: RunSpec,
  config: CarlaRuntimeConfig,
  environment: OpenPilotEnvironment,
  status_queue: Any,
) -> Any:
  from openpilot.tools.sim.bridge.common import SimulatorBridge

  _install_openpilot_camerad_buffer_compatibility()
  _install_openpilot_camerad_timestamp_compatibility()
  _install_openpilot_sensor_rate_compatibility()

  class OpenPilotCarlaBridge(SimulatorBridge):
    def __init__(self) -> None:
      super().__init__(dual_camera=True, high_quality=True)
      if not (
        abs(config.fixed_delta_seconds - 0.05) < 1e-12
        and abs(config.camera.sensor_tick_s - 0.05) < 1e-12
      ):
        raise EvaluationError(
          "The pinned OpenPilot camera adapter requires world and camera periods of 0.05 s"
        )
      self.TICKS_PER_FRAME = 5
      self.test_run = True
      causal_bypass_params = (
        "LateralManeuverMode",
        "LongitudinalManeuverMode",
        "JoystickDebugMode",
      )
      enabled_bypasses = [name for name in causal_bypass_params if self.params.get_bool(name)]
      if enabled_bypasses:
        raise EvaluationError(
          "OpenPilot causal-control certification requires these modes to be disabled: "
          + ", ".join(enabled_bypasses)
        )
      self.causal_bypass_modes_disabled = True

    def spawn_world(self, queue: Any) -> CarlaEvaluationWorld:
      try:
        import carla
      except ImportError as exc:
        raise DependencyUnavailableError("Install carla==0.9.16 in the OpenPilot venv") from exc
      client = carla.Client(config.host, config.port)
      client.set_timeout(config.timeout_s)
      try:
        return CarlaEvaluationWorld(
          client,
          run,
          config,
          status_queue=status_queue,
          dual_camera=True,
          high_quality=True,
          enable_cameras=True,
          controller_name="openpilot",
          valid_for_research=environment.valid_for_research,
          openpilot_commit=environment.commit,
          openpilot_dirty=environment.dirty,
          applied_control_mono_time_supplier=lambda: int(
            self.simulated_car.sm.logMonoTime["carControl"]
          ),
          requested_acceleration_supplier=lambda: float(
            self.simulated_car.sm["carControl"].actuators.accel
          ),
          openpilot_maneuver_modes_disabled=self.causal_bypass_modes_disabled,
        )
      except BaseException:
        _restore_async(client, config)
        raise

  return OpenPilotCarlaBridge()


def execute_run(
  run: RunSpec,
  runtime_config: str | Path,
  overwrite: bool = False,
) -> dict[str, Any]:
  summary_path = run.output_dir / "summary.json"
  environment = verify_openpilot_environment()
  config = load_runtime_config(runtime_config)
  if summary_path.exists() and not overwrite:
    summary = load_summary(summary_path)
    require_reusable_summary(
      summary,
      run,
      backend="carla",
      runtime_config=config,
      openpilot_commit=environment.commit,
      openpilot_dirty=environment.dirty,
    )
    return summary
  if overwrite:
    archive_existing_attempt(run.output_dir)
  command_queue: Any = multiprocessing.Queue()
  status_queue: Any = multiprocessing.Queue()
  deadline = time.monotonic() + max(240.0, run.scenario.duration_s * 4.0 + 120.0)
  messages: list[Any] = []
  timed_out = False
  manager_exit_code: int | None = None
  with _managed_openpilot_process(environment, run.output_dir) as manager:
    bridge = _make_bridge(run, config, environment, status_queue)
    process = bridge.run(command_queue)
    try:
      while process.is_alive():
        process.join(timeout=0.25)
        while not status_queue.empty():
          messages.append(status_queue.get())
        manager_exit_code = manager.process.poll()
        if manager_exit_code is not None:
          process.terminate()
          process.join(timeout=10)
          if process.is_alive():
            process.kill()
            process.join(timeout=5)
          break
        if time.monotonic() > deadline:
          timed_out = True
          process.terminate()
          process.join(timeout=10)
          if process.is_alive():
            process.kill()
            process.join(timeout=5)
          break
      while not status_queue.empty():
        messages.append(status_queue.get())
      if manager_exit_code is None:
        manager_exit_code = manager.process.poll()
    except BaseException:
      if process.is_alive():
        process.terminate()
        process.join(timeout=10)
        if process.is_alive():
          process.kill()
          process.join(timeout=5)
      raise

  if manager_exit_code is None:
    manager_exit_code = manager.exit_code_before_stop
  if manager_exit_code is not None:
    error = SimulatorConnectionError(
      f"Per-trial OpenPilot manager exited unexpectedly with code {manager_exit_code}; "
      f"inspect {run.output_dir / 'openpilot_manager.log'}"
    )
    if summary_path.exists():
      mark_existing_run_invalid(run, "openpilot_manager_exited", error)
    else:
      write_invalid_run(run, "carla", "openpilot_manager_exited", error)
    raise error

  if timed_out:
    if summary_path.exists():
      mark_existing_run_invalid(run, "bridge_wall_clock_timeout")
      return load_summary(summary_path)
    return write_invalid_run(run, "carla", "bridge_wall_clock_timeout")
  if summary_path.exists():
    if process.exitcode not in (0, None):
      error = SimulatorConnectionError(
        f"OpenPilot bridge finalized artifacts but exited with code {process.exitcode}"
      )
      mark_existing_run_invalid(run, "bridge_process_failed_after_finalize", error)
      raise error
    return load_summary(summary_path)
  error = SimulatorConnectionError(
    f"OpenPilot bridge exited with code {process.exitcode}; status messages: {messages[-5:]}"
  )
  write_invalid_run(run, "carla", "bridge_process_failed", error)
  raise error


def _select_run(experiment_path: Path, run_id: str) -> RunSpec:
  experiment = load_experiment(experiment_path)
  matches = [run for run in expand_runs(experiment) if run.run_id == run_id]
  if not matches:
    raise EvaluationError(f"Run id {run_id!r} is not present in {experiment_path}")
  run = matches[0]
  if run.controller != "openpilot":
    raise EvaluationError(f"Run {run_id!r} uses controller {run.controller!r}, not openpilot")
  return run


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--experiment", type=Path, required=True)
  parser.add_argument("--run-id", required=True)
  parser.add_argument(
    "--runtime-config",
    type=Path,
    default=Path(__file__).resolve().parents[2] / "config" / "carla.yaml",
  )
  parser.add_argument("--overwrite", action="store_true")
  return parser


def main(argv: Sequence[str] | None = None) -> int:
  args = build_parser().parse_args(argv)
  try:
    run = _select_run(args.experiment, args.run_id)
    summary = execute_run(run, args.runtime_config, args.overwrite)
    print(json.dumps({"run_id": run.run_id, "success": summary["metrics"]["success"]}, indent=2))
    return 0
  except (EvaluationError, OSError, ValueError) as exc:
    print(f"error: {exc}", file=sys.stderr)
    return 2


if __name__ == "__main__":  # pragma: no cover
  raise SystemExit(main())
