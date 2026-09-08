import json
import time
import urllib.error
import urllib.request
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from opencarla_eval.live_dashboard import (
  LiveDashboard,
  _camera_projection,
  _enum,
  _project_path_ribbon,
  build_openpilot_dashboard_state,
)


def test_capnp_enum_uses_human_readable_name_not_raw_ordinal() -> None:
  class CapnpEnum:
    raw = 2

    def __str__(self) -> str:
      return "selfdriveState.enabled"

  assert _enum(CapnpEnum()) == "enabled"


def _path(x: list[float], y: list[float]) -> SimpleNamespace:
  return SimpleNamespace(x=x, y=y, z=[0.0] * len(x), t=list(range(len(x))))


def test_openpilot_path_ribbon_projects_into_road_camera_pixels() -> None:
  projection = _camera_projection({})
  polygon = _project_path_ribbon(
    {"x": [3.0, 10.0, 100.0], "y": [0.0, 0.0, 0.0], "z": [0.0, 0.0, 0.0]},
    rpy_calib=(0.0, 0.0, 0.0),
    camera_height_m=1.22,
    camera_projection=projection,
  )

  assert len(polygon) == 6
  # Left and right edge of the 1.8 m OpenPilot ribbon at 10 m.
  assert polygon[1] == pytest.approx([725.628859088117, 927.1253243472192])
  assert polygon[4] == pytest.approx([1202.371140911883, 927.1253243472192])
  # The far path converges on the optical center/horizon.
  assert sum(point[0] for point in (polygon[2], polygon[3])) / 2.0 == pytest.approx(964.0)
  assert polygon[2][1] == pytest.approx(636.3125324347219)


def test_path_projection_uses_right_positive_coordinates_and_rejects_polygon_folds() -> None:
  projection = _camera_projection({})
  centered = _project_path_ribbon(
    {"x": [5.0, 10.0, 20.0], "y": [0.0, 0.0, 0.0], "z": [0.0, 0.0, 0.0]},
    rpy_calib=(0.0, 0.0, 0.0),
    camera_height_m=1.22,
    camera_projection=projection,
  )
  shifted_right = _project_path_ribbon(
    {"x": [5.0, 10.0, 20.0], "y": [1.0, 1.0, 1.0], "z": [0.0, 0.0, 0.0]},
    rpy_calib=(0.0, 0.0, 0.0),
    camera_height_m=1.22,
    camera_projection=projection,
  )
  folded = _project_path_ribbon(
    {"x": [5.0, 10.0, 20.0], "y": [0.0, 0.0, 0.0], "z": [0.0, 1.5, 0.0]},
    rpy_calib=(0.0, 0.0, 0.0),
    camera_height_m=1.22,
    camera_projection=projection,
  )

  centered_midpoint = sum(point[0] for point in (centered[1], centered[4])) / 2.0
  shifted_midpoint = sum(point[0] for point in (shifted_right[1], shifted_right[4])) / 2.0
  assert shifted_midpoint > centered_midpoint
  assert len(folded) == 4


def test_dashboard_html_layers_green_ribbon_without_removing_other_views() -> None:
  html = Path(__file__).parents[1].joinpath("src/opencarla_eval/dashboard.html").read_text()

  assert 'id="roadOverlay"' in html
  assert "rgba(13,248,122,0.40)" in html
  assert 'id="wide" class="wide"' in html
  assert 'id="planCanvas"' in html


def test_openpilot_messages_are_detached_into_dashboard_state() -> None:
  model = SimpleNamespace(
    frameId=42,
    frameIdExtra=42,
    frameAge=1,
    frameDropPerc=0.0,
    modelExecutionTime=0.021,
    position=_path([0.0, 10.0, 20.0], [0.0, 0.5, 1.0]),
    laneLines=[_path([0.0, 20.0], [-1.8, -1.5]), _path([0.0, 20.0], [1.8, 2.0])],
    laneLineProbs=[0.9, 0.8],
    roadEdges=[],
    leadsV3=[SimpleNamespace(prob=0.75, probTime=0.0, x=[18.0], y=[0.2], v=[8.0], a=[-0.5])],
    action=SimpleNamespace(desiredCurvature=0.002, desiredAcceleration=-0.4, shouldStop=False),
    meta=SimpleNamespace(
      laneChangeState="off", laneChangeDirection="none", hardBrakePredicted=False
    ),
  )
  plan = SimpleNamespace(
    modelMonoTime=123,
    longitudinalPlanSource="lead0",
    hasLead=True,
    fcw=False,
    shouldStop=False,
    allowThrottle=False,
    allowBrake=True,
    aTarget=-0.5,
    speeds=[10.0, 9.5],
    accels=[-0.5, -0.4],
    jerks=[0.1, 0.2],
  )
  actuators = SimpleNamespace(
    accel=-0.45, steeringAngleDeg=2.0, curvature=0.002, torque=0.1, longControlState="pid"
  )
  messages = {
    "modelV2": model,
    "longitudinalPlan": plan,
    "controlsState": SimpleNamespace(
      desiredCurvature=0.002, curvature=0.0018, longControlState="pid", forceDecel=False
    ),
    "carControl": SimpleNamespace(
      enabled=True, latActive=True, longActive=True, actuators=actuators
    ),
    "selfdriveState": SimpleNamespace(
      state="enabled",
      enabled=True,
      active=True,
      engageable=True,
      alertText1="",
      alertText2="",
      alertType="",
      experimentalMode=False,
    ),
    "carState": SimpleNamespace(vEgo=10.0, aEgo=-0.2, steeringAngleDeg=1.5),
    "liveCalibration": SimpleNamespace(
      calStatus="calibrated", rpyCalib=[0.0, 0.01, 0.0], height=[1.22]
    ),
  }

  state = build_openpilot_dashboard_state(
    messages, {service: True for service in messages}, {"phase": "evaluating"}
  )

  assert state["model"]["frame_id"] == 42
  assert state["model"]["trajectory"]["y"] == [0.0, 0.5, 1.0]
  assert len(state["model"]["trajectory_overlay"]["polygon_px"]) == 4
  assert state["model"]["trajectory_overlay"]["calibration_applied"] is True
  assert state["model"]["camera_projection"]["width_px"] == 1928.0
  assert state["model"]["leads"][0]["x_m"] == 18.0
  assert state["planning"]["source"] == "lead0"
  assert state["planning"]["times_s"] == [0.0, 10.0 * (1.0 / 32.0) ** 2]
  assert state["control"]["command_acceleration_mps2"] == -0.45
  assert state["selfdrive"]["active"] is True
  assert state["vehicle"]["speed_mps"] == 10.0
  assert state["calibration"]["status"] == "calibrated"


def test_dashboard_serves_state_and_asynchronously_encoded_frame() -> None:
  dashboard = LiveDashboard("127.0.0.1", 0, frame_stride=1, downsample=1, jpeg_quality=80)
  dashboard.start()
  try:
    dashboard.publish_state(
      {"runtime": {"phase": "evaluating"}, "active": True, "bad": float("nan")}
    )
    dashboard.publish_frame(
      "road",
      np.full((24, 32, 3), (20, 120, 220), dtype=np.uint8),
      carla_frame=7,
      openpilot_frame=9,
    )
    base = f"http://127.0.0.1:{dashboard.port}"
    with urllib.request.urlopen(f"{base}/health", timeout=2) as response:
      assert response.status == 200
      health = json.loads(response.read())
      assert health["ok"] is True
      assert health["encoder_error"] is None
    with urllib.request.urlopen(f"{base}/api/state", timeout=2) as response:
      state = json.loads(response.read())
      assert state["runtime"]["phase"] == "evaluating"
      assert state["runtime"]["dashboard_encoder_ok"] is True
      assert state["active"] is True
      assert state["bad"] is None

    deadline = time.monotonic() + 2.0
    while True:
      try:
        with urllib.request.urlopen(f"{base}/camera/road.jpg", timeout=2) as response:
          body = response.read()
          if response.status == 200:
            assert response.headers["X-Carla-Frame"] == "7"
            assert response.headers["X-Openpilot-Frame"] == "9"
            assert body.startswith(b"\xff\xd8")
            break
      except urllib.error.HTTPError as exc:
        if exc.code != 204:
          raise
      if time.monotonic() >= deadline:
        raise AssertionError("dashboard did not encode the latest road frame")
      time.sleep(0.01)
  finally:
    dashboard.close()
    dashboard.close()
