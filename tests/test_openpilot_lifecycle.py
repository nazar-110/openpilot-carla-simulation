from pathlib import Path
from types import SimpleNamespace

import numpy as np

from opencarla_eval import openpilot_bridge
from opencarla_eval.openpilot_bridge import (
  OpenPilotEnvironment,
  _install_openpilot_camerad_buffer_compatibility,
  _install_openpilot_camerad_timestamp_compatibility,
  _managed_openpilot_process,
  _pad_nv12_for_openpilot,
)


def test_two_consecutive_trials_get_distinct_manager_processes(
  tmp_path: Path, monkeypatch: object
) -> None:
  starts = []
  signals = []

  class FakeProcess:
    def __init__(self, pid: int) -> None:
      self.pid = pid
      self.returncode = None

    def poll(self) -> int | None:
      return self.returncode

    def wait(self, timeout: float) -> int:
      del timeout
      self.returncode = -15
      return self.returncode

    def terminate(self) -> None:
      self.returncode = -15

    def kill(self) -> None:
      self.returncode = -9

  def fake_popen(*args: object, **kwargs: object) -> FakeProcess:
    starts.append((args, kwargs))
    return FakeProcess(1000 + len(starts))

  monkeypatch.setattr(
    "opencarla_eval.openpilot_bridge._running_openpilot_managers", lambda _root: []
  )
  monkeypatch.setattr(
    "opencarla_eval.openpilot_bridge._wait_for_openpilot_manager_ready",
    lambda _manager: None,
  )
  monkeypatch.setattr("opencarla_eval.openpilot_bridge.subprocess.Popen", fake_popen)
  monkeypatch.setattr(
    "opencarla_eval.openpilot_bridge.os.killpg",
    lambda pid, signal_number: signals.append((pid, signal_number)),
    raising=False,
  )
  root = tmp_path / "openpilot"
  (root / "tools" / "sim").mkdir(parents=True)
  environment = OpenPilotEnvironment(root, "commit", False, True)

  for index in range(2):
    with _managed_openpilot_process(environment, tmp_path / f"run-{index}") as manager:
      assert manager.process.poll() is None

  assert len(starts) == 2
  assert [pid for pid, _signal in signals] == [1001, 1002]
  assert all(call[1]["start_new_session"] is True for call in starts)
  assert all({"soundd", "ui"} <= set(call[1]["env"]["BLOCK"].split(",")) for call in starts)
  assert (tmp_path / "run-0" / "openpilot_manager.log").is_file()
  assert (tmp_path / "run-1" / "openpilot_manager.log").is_file()


def test_simulator_camera_frames_use_boottime_clock(monkeypatch: object) -> None:
  sent_vision = []
  sent_messages = []

  class FakeCamerad:
    def _send_yuv(self, *_args: object) -> None:
      raise AssertionError("upstream frame-counter timestamp should be replaced")

  class FakeMessage:
    pass

  fake_messaging = SimpleNamespace(new_message=lambda *_args, **_kwargs: FakeMessage())
  monkeypatch.setitem(
    __import__("sys").modules, "cereal", SimpleNamespace(messaging=fake_messaging)
  )
  monkeypatch.setitem(
    __import__("sys").modules,
    "openpilot.tools.sim.lib.camerad",
    SimpleNamespace(Camerad=FakeCamerad),
  )
  timestamps = iter((123456789, 987654321))
  monkeypatch.setattr(
    "opencarla_eval.openpilot_bridge._camera_timestamp_ns", lambda: next(timestamps)
  )

  _install_openpilot_camerad_timestamp_compatibility()
  instance = FakeCamerad()
  instance.vipc_server = SimpleNamespace(
    send=lambda *args: sent_vision.append(args),
  )
  instance.pm = SimpleNamespace(send=lambda *args: sent_messages.append(args))
  instance._send_yuv(b"nv12", 7, "roadCameraState", "road")
  instance._send_yuv(b"wide-nv12", 7, "wideRoadCameraState", "wide")
  instance._send_yuv(b"next-nv12", 8, "roadCameraState", "road")

  assert sent_vision == [
    ("road", b"nv12", 7, 123456789, 123456789),
    ("wide", b"wide-nv12", 7, 123456789, 123456789),
    ("road", b"next-nv12", 8, 987654321, 987654321),
  ]
  assert sent_messages[0][0] == "roadCameraState"
  assert sent_messages[0][1].roadCameraState["frameId"] == 7
  assert sent_messages[0][1].roadCameraState["timestampEof"] == 123456789


def test_tight_nv12_is_repacked_into_openpilot_aligned_planes() -> None:
  width = height = 4
  stride, y_height, uv_height, yuv_size = 8, 6, 3, 72
  tight = np.arange(width * height * 3 // 2, dtype=np.uint8)

  padded = np.frombuffer(
    _pad_nv12_for_openpilot(tight.tobytes(), width, height, stride, y_height, uv_height, yuv_size),
    dtype=np.uint8,
  )
  y_plane = padded[: stride * y_height].reshape(y_height, stride)
  uv_plane = padded[stride * y_height :].reshape(uv_height, stride)

  assert np.array_equal(y_plane[:height, :width], tight[: width * height].reshape(4, 4))
  assert not y_plane[:height, width:].any()
  assert not y_plane[height:].any()
  assert np.array_equal(uv_plane[: height // 2, :width], tight[width * height :].reshape(2, 4))
  assert not uv_plane[: height // 2, width:].any()
  assert not uv_plane[height // 2 :].any()


def test_camerad_allocates_aligned_road_and_wide_buffers(monkeypatch: object) -> None:
  buffer_calls = []

  class FakeCamerad:
    pass

  class FakeVisionIpcServer:
    def __init__(self, name: str) -> None:
      self.name = name
      self.started = False

    def create_buffers_with_sizes(self, *args: object) -> None:
      buffer_calls.append(args)

    def start_listener(self) -> None:
      self.started = True

  fake_messaging = SimpleNamespace(PubMaster=lambda services: SimpleNamespace(services=services))
  fake_stream_types = SimpleNamespace(
    VISION_STREAM_ROAD="road",
    VISION_STREAM_WIDE_ROAD="wide",
  )
  monkeypatch.setitem(
    __import__("sys").modules, "cereal", SimpleNamespace(messaging=fake_messaging)
  )
  monkeypatch.setitem(
    __import__("sys").modules,
    "msgq.visionipc",
    SimpleNamespace(VisionIpcServer=FakeVisionIpcServer, VisionStreamType=fake_stream_types),
  )
  monkeypatch.setitem(
    __import__("sys").modules,
    "openpilot.system.camerad.cameras.nv12_info",
    SimpleNamespace(get_nv12_info=lambda _width, _height: (2048, 1216, 1130, 4_804_608)),
  )
  monkeypatch.setitem(
    __import__("sys").modules,
    "openpilot.tools.sim.lib.camerad",
    SimpleNamespace(Camerad=FakeCamerad),
  )
  monkeypatch.setitem(
    __import__("sys").modules,
    "openpilot.tools.sim.lib.common",
    SimpleNamespace(W=1928, H=1208),
  )

  _install_openpilot_camerad_buffer_compatibility()
  instance = FakeCamerad(dual_camera=True)

  assert instance.vipc_server.started
  assert instance._opencarla_nv12_layout == (1928, 1208, 2048, 1216, 1130, 4_804_608)
  assert buffer_calls == [
    ("road", 5, 1928, 1208, 4_804_608, 2048, 2_490_368),
    ("wide", 5, 1928, 1208, 4_804_608, 2048, 2_490_368),
  ]


def _fake_sensor_environment(
  monkeypatch: object,
) -> tuple[type, list[tuple[str, object]], list[int]]:
  sent_messages: list[tuple[str, object]] = []
  now_ns = [1_000_000_000]

  class FakeSensorPayload:
    def __init__(self) -> None:
      self.timestamp = 0
      self.acceleration = SimpleNamespace(v=[])
      self.gyroUncalibrated = SimpleNamespace(v=[])

    def init(self, _kind: str) -> None:
      pass

  class FakeMessage:
    def __init__(self, service: str, valid: bool, log_mono_time: int) -> None:
      self.valid = valid
      self.logMonoTime = log_mono_time
      if service == "accelerometer":
        self.accelerometer = FakeSensorPayload()
      elif service == "gyroscope":
        self.gyroscope = FakeSensorPayload()
      elif service == "gpsLocationExternal":
        self.gpsLocationExternal = {}

  def new_message(service: str, **kwargs: object) -> FakeMessage:
    return FakeMessage(
      service,
      bool(kwargs.get("valid", False)),
      int(kwargs.get("logMonoTime", now_ns[0])),
    )

  class FakeSimulatedSensors:
    def send_imu_message(self, _simulator_state: object) -> None:
      raise AssertionError("upstream five-message IMU burst should be replaced")

    def send_gps_message(self, _simulator_state: object) -> None:
      raise AssertionError("upstream ten-message GPS burst should be replaced")

  fake_messaging = SimpleNamespace(new_message=new_message)
  fake_log = SimpleNamespace(
    GpsLocationData=SimpleNamespace(SensorSource=SimpleNamespace(ublox="ublox"))
  )
  monkeypatch.setitem(
    __import__("sys").modules,
    "cereal",
    SimpleNamespace(log=fake_log, messaging=fake_messaging),
  )
  monkeypatch.setitem(__import__("sys").modules, "cereal.messaging", fake_messaging)
  monkeypatch.setitem(
    __import__("sys").modules,
    "openpilot.tools.sim.lib.simulated_sensors",
    SimpleNamespace(SimulatedSensors=FakeSimulatedSensors),
  )
  monkeypatch.setattr(openpilot_bridge.time, "monotonic_ns", lambda: now_ns[0])
  monkeypatch.setattr(openpilot_bridge.time, "time", lambda: 1_700_000_000.0)
  monkeypatch.setattr(openpilot_bridge.time, "time_ns", lambda: 1_700_000_000_000_000_000)

  return FakeSimulatedSensors, sent_messages, now_ns


def test_simulator_imu_publishes_one_monotonic_sample_per_update(monkeypatch: object) -> None:
  sensor_class, sent_messages, _now_ns = _fake_sensor_environment(monkeypatch)
  installer = openpilot_bridge._install_openpilot_sensor_rate_compatibility
  installer()
  patched_imu = sensor_class.send_imu_message
  patched_gps = sensor_class.send_gps_message
  installer()
  assert sensor_class._opencarla_sensor_rate_compatible is True
  assert sensor_class.send_imu_message is patched_imu
  assert sensor_class.send_gps_message is patched_gps

  state = SimpleNamespace(
    imu=SimpleNamespace(
      accelerometer=SimpleNamespace(x=1.25, y=-2.5, z=9.75),
      gyroscope=SimpleNamespace(x=0.1, y=-0.2, z=0.3),
    )
  )
  instance = sensor_class()
  instance.pm = SimpleNamespace(
    send=lambda service, message: sent_messages.append((service, message))
  )
  instance.send_imu_message(state)

  assert [service for service, _message in sent_messages] == ["accelerometer", "gyroscope"]
  acceleration = sent_messages[0][1]
  gyroscope = sent_messages[1][1]
  assert acceleration.valid and gyroscope.valid
  assert acceleration.accelerometer.timestamp == acceleration.logMonoTime == 1_000_000_000
  assert gyroscope.gyroscope.timestamp == gyroscope.logMonoTime == 1_000_000_000
  assert acceleration.accelerometer.acceleration.v == [1.25, -2.5, 9.75]
  assert gyroscope.gyroscope.gyroUncalibrated.v == [0.1, -0.2, 0.3]


def test_simulator_gps_publishes_one_fix_at_ten_hz(monkeypatch: object) -> None:
  sensor_class, sent_messages, now_ns = _fake_sensor_environment(monkeypatch)
  installer = openpilot_bridge._install_openpilot_sensor_rate_compatibility
  installer()
  state = SimpleNamespace(
    valid=False,
    velocity=SimpleNamespace(x=3.0, y=-4.0, z=0.5),
    imu=SimpleNamespace(bearing=87.0),
    gps=SimpleNamespace(latitude=32.75, longitude=-117.2, altitude=12.0),
    speed=5.0,
  )
  instance = sensor_class()
  instance.pm = SimpleNamespace(
    send=lambda service, message: sent_messages.append((service, message))
  )

  instance.send_gps_message(state)
  state.valid = True
  instance.send_gps_message(state)
  now_ns[0] += 99_999_999
  instance.send_gps_message(state)
  now_ns[0] += 1
  instance.send_gps_message(state)
  now_ns[0] += 1_000_000_000
  instance.send_gps_message(state)

  assert [service for service, _message in sent_messages] == [
    "gpsLocationExternal",
    "gpsLocationExternal",
    "gpsLocationExternal",
  ]
  fixes = [message.gpsLocationExternal for _service, message in sent_messages]
  assert fixes[0] == {
    "unixTimestampMillis": 1_700_000_000_000,
    "flags": 1,
    "horizontalAccuracy": 1.0,
    "verticalAccuracy": 1.0,
    "speedAccuracy": 0.1,
    "bearingAccuracyDeg": 0.1,
    "vNED": [4.0, 3.0, 0.5],
    "bearingDeg": 87.0,
    "latitude": 32.75,
    "longitude": -117.2,
    "altitude": 12.0,
    "speed": 5.0,
    "source": "ublox",
  }
  assert fixes[1:] == [fixes[0], fixes[0]]
  assert [message.logMonoTime for _service, message in sent_messages] == [
    1_000_000_000,
    1_100_000_000,
    2_100_000_000,
  ]
  assert instance._opencarla_last_gps_ns == 2_100_000_000
