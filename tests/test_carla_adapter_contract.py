import threading
from types import SimpleNamespace

import pytest

from opencarla_eval.carla_world import (
  CarlaEvaluationWorld,
  _CameraFrameHandoff,
  _carla_accel_to_openpilot_sensor,
  _carla_gyro_to_openpilot_sensor,
  _chase_spectator_pose,
  _evaluation_origin_alignment,
  _OpenPilotFreshnessBarrier,
  _parse_recorder_collisions,
  _smooth_spectator_pose,
)


def test_steering_feedback_inverts_configured_sign() -> None:
  world = object.__new__(CarlaEvaluationWorld)
  world.vehicle = SimpleNamespace(get_control=lambda: SimpleNamespace(steer=-0.2))
  world.max_wheel_angle_deg = 70.0
  world.config = SimpleNamespace(actuation=SimpleNamespace(steering_ratio=15.0, steer_sign=-1.0))

  assert world._steering_feedback_deg() == pytest.approx(210.0)


def test_carla_imu_axes_map_to_openpilot_forward_right_down() -> None:
  raw_accel = _carla_accel_to_openpilot_sensor(1.0, 2.0, 3.0)
  raw_gyro = _carla_gyro_to_openpilot_sensor(1.0, 2.0, 3.0)

  # locationd's fixed raw-sensor-to-device mapping is [-z, -y, -x].
  locationd_accel = (-raw_accel[2], -raw_accel[1], -raw_accel[0])
  locationd_gyro = (-raw_gyro[2], -raw_gyro[1], -raw_gyro[0])
  assert locationd_accel == pytest.approx((1.0, 2.0, -3.0))
  assert locationd_gyro == pytest.approx((1.0, 2.0, 3.0))


def test_chase_spectator_pose_rotates_behind_ego() -> None:
  transform = SimpleNamespace(
    location=SimpleNamespace(x=10.0, y=20.0, z=1.0),
    rotation=SimpleNamespace(yaw=90.0),
  )

  pose = _chase_spectator_pose(transform, 7.0, 3.0, -15.0)

  assert pose == pytest.approx((10.0, 13.0, 4.0, -15.0, 90.0, 0.0))


def test_evaluation_origin_rejects_junction_heading_and_lateral_misalignment() -> None:
  waypoint = SimpleNamespace(
    is_junction=True,
    transform=SimpleNamespace(
      location=SimpleNamespace(x=0.0, y=0.0, z=0.0),
      rotation=SimpleNamespace(yaw=-179.0),
      get_right_vector=lambda: SimpleNamespace(x=0.0, y=1.0, z=0.0),
    ),
  )
  ego = SimpleNamespace(
    location=SimpleNamespace(x=0.0, y=1.0, z=0.0),
    rotation=SimpleNamespace(yaw=-140.0),
  )

  alignment = _evaluation_origin_alignment(ego, waypoint)

  assert not alignment["valid"]
  assert alignment["reasons"] == ["junction", "heading_error", "lateral_offset"]
  assert alignment["heading_error_deg"] == pytest.approx(39.0)
  assert alignment["lateral_offset_m"] == pytest.approx(1.0)


def test_openpilot_bootstrap_obeys_explicit_speed_and_throttle_cap() -> None:
  applied = []
  velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
  world = object.__new__(CarlaEvaluationWorld)
  world._settled_spawn_transform = object()
  world._post_reset_settling = False
  world._evaluation_started = False
  world._controller_active = False
  world.controller_name = "openpilot"
  world.run = SimpleNamespace(
    scenario=SimpleNamespace(
      target_speed_mps=11.11,
      parameters={"startup_target_speed_mps": 5.0, "startup_max_throttle": 0.35},
    )
  )
  world.max_wheel_angle_deg = 70.0
  world.config = SimpleNamespace(actuation=SimpleNamespace(steering_ratio=15.0, steer_sign=-1.0))
  world._vc = SimpleNamespace(throttle=0.0, brake=0.0, steer=0.0)
  world.vehicle = SimpleNamespace(
    get_velocity=lambda: velocity,
    apply_control=lambda control: applied.append(control),
  )

  world.apply_controls(steer_angle=0.0, throttle_out=1.0, brake_out=0.0)

  assert world._vc.throttle == pytest.approx(0.35)
  assert world._vc.brake == 0.0
  assert applied


def test_spectator_smoothing_uses_shortest_yaw_arc_and_snaps_large_jump() -> None:
  previous = (0.0, 0.0, 0.0, -15.0, 179.0, 0.0)
  target = (1.0, 0.0, 0.0, -15.0, -179.0, 0.0)

  smoothed = _smooth_spectator_pose(
    previous, target, delta_s=0.05, smoothing_time_s=0.1, snap_distance_m=15.0
  )
  snapped = _smooth_spectator_pose(
    previous,
    (20.0, 0.0, 0.0, -15.0, 0.0, 0.0),
    delta_s=0.05,
    smoothing_time_s=0.1,
    snap_distance_m=15.0,
  )

  assert 179.0 < smoothed[4] < 181.0
  assert snapped == (20.0, 0.0, 0.0, -15.0, 0.0, 0.0)


def test_spectator_failure_is_nonfatal_and_recorded_once() -> None:
  events = []

  class FailingSpectator:
    def set_transform(self, _transform: object) -> None:
      raise RuntimeError("viewport closed")

  world = object.__new__(CarlaEvaluationWorld)
  world._spectator = FailingSpectator()
  world._spectator_pose = None
  world._spectator_failure_recorded = False
  world._evaluation_time_s = 0.0
  world._world_frame = 12
  world.config = SimpleNamespace(
    fixed_delta_seconds=0.05,
    spectator=SimpleNamespace(distance_m=7.0, height_m=3.0, pitch_deg=-15.0, smoothing_time_s=0.12),
  )
  world.vehicle = SimpleNamespace(
    get_transform=lambda: SimpleNamespace(
      location=SimpleNamespace(x=1.0, y=2.0, z=0.5),
      rotation=SimpleNamespace(yaw=0.0),
    )
  )
  world.carla = SimpleNamespace(
    Location=lambda **values: SimpleNamespace(**values),
    Rotation=lambda **values: SimpleNamespace(**values),
    Transform=lambda location, rotation: SimpleNamespace(location=location, rotation=rotation),
  )
  world.recorder = SimpleNamespace(record_event=lambda event: events.append(event))

  world._update_spectator()
  world._update_spectator()

  assert world._spectator is None
  assert [event.event_type for event in events] == ["spectator_disabled"]


def test_dashboard_publisher_failure_is_nonfatal_and_recorded_once() -> None:
  class FailingDashboard:
    def close(self) -> None:
      return

  world = object.__new__(CarlaEvaluationWorld)
  events = []
  world._dashboard = FailingDashboard()
  world._dashboard_failure_recorded = False
  world._evaluation_time_s = 1.25
  world._world_frame = 17
  world.recorder = SimpleNamespace(record_event=events.append)

  def fail(*, force: bool = False) -> None:
    del force
    raise ValueError("diagnostic serialization failed")

  world._publish_dashboard_state_unchecked = fail

  world._publish_dashboard_state()
  world._publish_dashboard_state()

  assert world._dashboard is None
  assert [event.event_type for event in events] == ["dashboard_disabled"]


def test_post_reset_control_is_released_only_after_fresh_control_is_ready() -> None:
  applied = []
  world = object.__new__(CarlaEvaluationWorld)
  world._post_reset_settling = True
  world._post_reset_ticks = 1
  world._post_reset_sane_imu_ticks = 2
  world._post_reset_control_ready = False
  world._post_reset_fresh_control_applied = False
  world._post_reset_freshness = SimpleNamespace(
    evaluate=lambda selected: {"valid": selected == 400}
  )
  world._openpilot_messaging = None
  world._openpilot_causal_sockets = {}
  applied_mono_time = [399]
  world._applied_control_mono_time_supplier = lambda: applied_mono_time[0]
  world._post_reset_applied_control_mono_time = None
  world.controller_name = "openpilot"
  world._controller_active = True
  world._controller_state = "enabled"
  world._openpilot_localization_ready = True
  world._evaluation_started = False
  world.max_wheel_angle_deg = 70.0
  world.config = SimpleNamespace(actuation=SimpleNamespace(steering_ratio=15.0, steer_sign=-1.0))
  world._vc = SimpleNamespace(throttle=0.0, brake=0.0, steer=0.0)
  world.vehicle = SimpleNamespace(apply_control=lambda control: applied.append(control))

  world.apply_controls(steer_angle=30.0, throttle_out=0.4, brake_out=0.0)
  assert world._vc.throttle == 0.0
  assert world._vc.steer == 0.0
  assert not world._post_reset_fresh_control_applied

  world._post_reset_ticks = 2
  world.apply_controls(steer_angle=30.0, throttle_out=0.4, brake_out=0.0)
  assert world._vc.throttle == 0.0
  assert not world._post_reset_fresh_control_applied

  applied_mono_time[0] = 400
  world.apply_controls(steer_angle=30.0, throttle_out=0.4, brake_out=0.0)
  assert world._vc.throttle == pytest.approx(0.4)
  assert world._vc.steer != 0.0
  assert world._post_reset_fresh_control_applied

  applied_mono_time[0] = 399
  world.apply_controls(steer_angle=30.0, throttle_out=0.4, brake_out=0.0)
  assert world._vc.throttle == 0.0
  assert not world._post_reset_control_ready
  assert not world._post_reset_fresh_control_applied


def test_causal_camera_target_latches_carla_pause_across_readiness_flicker() -> None:
  world = object.__new__(CarlaEvaluationWorld)
  world._post_reset_settling = True
  world.controller_name = "openpilot"
  world._post_reset_openpilot_camera_id = 145
  world._post_reset_fresh_control_applied = False
  world._controller_active = True
  world._controller_state = "enabled"
  world._openpilot_localization_ready = False

  assert not world._post_reset_controller_ready()
  assert world._post_reset_waiting_for_causal_control()

  world._post_reset_fresh_control_applied = True
  assert not world._post_reset_waiting_for_causal_control()


def test_reset_rebases_grounded_pose_without_changing_body_kinematics() -> None:
  def transform(
    location: object, forward: tuple[float, float, float], right: tuple[float, float, float]
  ) -> SimpleNamespace:
    def vector(values: tuple[float, float, float]) -> SimpleNamespace:
      return SimpleNamespace(x=values[0], y=values[1], z=values[2])

    return SimpleNamespace(
      location=location,
      get_forward_vector=lambda: vector(forward),
      get_right_vector=lambda: vector(right),
      get_up_vector=lambda: vector((0.0, 0.0, 1.0)),
    )

  current = transform(SimpleNamespace(z=0.7), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))
  grounded = transform(SimpleNamespace(z=-0.1), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
  elevated_spawn = transform(SimpleNamespace(z=0.8), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
  calls: dict[str, object] = {}
  vehicle = SimpleNamespace(
    get_transform=lambda: current,
    get_velocity=lambda: SimpleNamespace(x=-2.0, y=8.0, z=0.1),
    get_angular_velocity=lambda: SimpleNamespace(x=-4.0, y=3.0, z=5.0),
    set_transform=lambda value: calls.__setitem__("transform", value),
    apply_control=lambda value: calls.__setitem__("control", value),
    set_target_velocity=lambda value: calls.__setitem__("velocity", value),
    set_target_angular_velocity=lambda value: calls.__setitem__("angular_velocity", value),
  )
  world = object.__new__(CarlaEvaluationWorld)
  world.controller_name = "openpilot"
  world.vehicle = vehicle
  world.spawn_point = elevated_spawn
  world._settled_spawn_transform = grounded
  world.carla = SimpleNamespace(
    VehicleControl=lambda **kwargs: SimpleNamespace(**kwargs),
    Vector3D=lambda **kwargs: SimpleNamespace(**kwargs),
  )

  world.reset()

  assert calls["transform"] is grounded
  assert calls["transform"] is not elevated_spawn
  assert vars(calls["velocity"]) == pytest.approx({"x": 8.0, "y": 2.0, "z": 0.1})
  assert vars(calls["angular_velocity"]) == pytest.approx({"x": 3.0, "y": 4.0, "z": 5.0})
  assert world._post_reset_speed_mps == pytest.approx((8.0**2 + 2.0**2 + 0.1**2) ** 0.5)


def test_evaluation_transition_preserves_live_pose_and_velocity() -> None:
  calls = []
  vehicle = SimpleNamespace(
    get_velocity=lambda: SimpleNamespace(x=3.0, y=4.0, z=0.0),
    get_transform=lambda: (_ for _ in ()).throw(AssertionError("pose must not be read")),
    set_transform=lambda _value: (_ for _ in ()).throw(AssertionError("must not teleport")),
    set_target_velocity=lambda _value: (_ for _ in ()).throw(
      AssertionError("must not replace velocity")
    ),
    apply_control=lambda control: calls.append(control),
  )
  world = object.__new__(CarlaEvaluationWorld)
  world.controller_name = "openpilot"
  world.vehicle = vehicle
  world.carla = SimpleNamespace(VehicleControl=lambda **kwargs: SimpleNamespace(**kwargs))
  world._route_distance_m = 12.0
  world._route_index = 3
  world._route_origin_progress_m = 12.0
  world._last_lane_offset_m = 0.25
  world._geometric_lane_crossing_active = True

  world._begin_continuous_evaluation_transition()

  assert world._post_reset_speed_mps == pytest.approx(5.0)
  assert len(calls) == 1
  assert calls[0].throttle == calls[0].brake == calls[0].steer == 0.0


def test_reset_requires_a_captured_grounded_pose() -> None:
  world = object.__new__(CarlaEvaluationWorld)
  world.controller_name = "openpilot"
  world._settled_spawn_transform = None

  with pytest.raises(RuntimeError, match="suspension-settling pose"):
    world.reset()


def test_post_reset_extreme_imu_is_reported_without_mutating_sample() -> None:
  events = []
  measurement = SimpleNamespace(
    accelerometer=SimpleNamespace(x=101.0, y=0.0, z=0.0),
    gyroscope=SimpleNamespace(x=0.0, y=0.0, z=0.0),
  )
  world = object.__new__(CarlaEvaluationWorld)
  world._post_reset_settling = True
  world._latest_imu = measurement
  world._post_reset_last_accel_norm_mps2 = None
  world._post_reset_last_gyro_norm_rad_s = None
  world._post_reset_sane_imu_ticks = 0
  world.vehicle = SimpleNamespace(get_velocity=lambda: SimpleNamespace(z=0.0))
  world.recorder = SimpleNamespace(record_event=lambda event: events.append(event))
  world.exit_event = threading.Event()

  assert not world._update_post_reset_imu_health(42)
  assert measurement.accelerometer.x == 101.0
  assert events[0].event_type == "post_reset_imu_discontinuity"
  assert events[0].details["acceleration_norm_mps2"] == pytest.approx(101.0)
  assert world.exit_event.is_set()


def test_post_reset_freshness_requires_complete_model_to_control_lineage() -> None:
  barrier = _OpenPilotFreshnessBarrier()
  barrier.arm(12, maneuver_modes_disabled=True)
  barrier.ingest("modelV2", 100, True, SimpleNamespace(frameId=12, frameIdExtra=12))
  barrier.ingest("longitudinalPlan", 200, True, SimpleNamespace(modelMonoTime=100))
  barrier.ingest(
    "controlsState",
    300,
    True,
    SimpleNamespace(
      lateralPlanMonoTime=100,
      longitudinalPlanMonoTime=200,
      desiredCurvature=0.01,
      longControlState=1,
    ),
  )
  barrier.ingest(
    "carControl",
    301,
    True,
    SimpleNamespace(
      enabled=True,
      latActive=True,
      longActive=True,
      actuators=SimpleNamespace(curvature=0.01, longControlState=1),
    ),
  )

  assert barrier.evaluate(301)["valid"]
  assert barrier.evaluate(299)["reason"] == "selected_car_control_not_captured"


def test_post_reset_freshness_rejects_a_plan_from_another_model() -> None:
  barrier = _OpenPilotFreshnessBarrier()
  barrier.arm(12, maneuver_modes_disabled=True)
  barrier.ingest("modelV2", 100, True, SimpleNamespace(frameId=12, frameIdExtra=12))
  barrier.ingest("longitudinalPlan", 200, True, SimpleNamespace(modelMonoTime=99))
  barrier.ingest(
    "controlsState",
    300,
    True,
    SimpleNamespace(
      lateralPlanMonoTime=100,
      longitudinalPlanMonoTime=200,
      desiredCurvature=0.01,
      longControlState=1,
    ),
  )
  barrier.ingest(
    "carControl",
    301,
    True,
    SimpleNamespace(
      enabled=True,
      latActive=True,
      longActive=True,
      actuators=SimpleNamespace(curvature=0.01, longControlState=1),
    ),
  )

  assert barrier.evaluate(301)["reason"] == "longitudinal_plan_model_mismatch"


def test_post_reset_freshness_rejects_a_lossless_capture_gap() -> None:
  barrier = _OpenPilotFreshnessBarrier()
  barrier.arm(12, maneuver_modes_disabled=True)
  state = SimpleNamespace(
    lateralPlanMonoTime=100,
    longitudinalPlanMonoTime=200,
    desiredCurvature=0.01,
    longControlState=1,
  )
  barrier.ingest("controlsState", 250, True, state)
  barrier.ingest("controlsState", 300, True, state)
  barrier.ingest(
    "carControl",
    301,
    True,
    SimpleNamespace(
      enabled=True,
      latActive=True,
      longActive=True,
      actuators=SimpleNamespace(curvature=0.01, longControlState=1),
    ),
  )

  assert barrier.evaluate(301)["reason"] == "controlsState_carControl_alternation_gap"


def test_post_reset_freshness_rejects_noncausal_timestamp_order() -> None:
  barrier = _OpenPilotFreshnessBarrier()
  barrier.arm(12, maneuver_modes_disabled=True)
  barrier.ingest("modelV2", 500, True, SimpleNamespace(frameId=12, frameIdExtra=12))
  barrier.ingest("longitudinalPlan", 200, True, SimpleNamespace(modelMonoTime=500))
  barrier.ingest(
    "controlsState",
    300,
    True,
    SimpleNamespace(
      lateralPlanMonoTime=500,
      longitudinalPlanMonoTime=200,
      desiredCurvature=0.01,
      longControlState=1,
    ),
  )
  barrier.ingest(
    "carControl",
    301,
    True,
    SimpleNamespace(
      enabled=True,
      latActive=True,
      longActive=True,
      actuators=SimpleNamespace(curvature=0.01, longControlState=1),
    ),
  )

  assert barrier.evaluate(301)["reason"] == "causal_event_timestamps_not_strictly_ordered"


def test_camera_handoff_backpressures_until_previous_conversion_finishes() -> None:
  handoff = _CameraFrameHandoff()

  assert handoff.wait_for_publish_slot(timeout=0.01)
  handoff.release()
  assert handoff.acquire(timeout=0.01)
  assert not handoff.wait_for_publish_slot(timeout=0.01)

  acquired: list[bool] = []
  consumer = threading.Thread(target=lambda: acquired.append(handoff.acquire(timeout=0.5)))
  consumer.start()
  assert handoff.wait_for_publish_slot(timeout=0.5)
  handoff.release()
  consumer.join(timeout=0.5)

  assert acquired == [True]
  handoff.close()


def test_sparse_sensor_event_is_committed_with_its_exact_frame_time() -> None:
  class Recorder:
    def __init__(self) -> None:
      self.events = []

    def record_event(self, event: object) -> None:
      self.events.append(event)

  world = object.__new__(CarlaEvaluationWorld)
  world._sensor_condition = threading.Condition()
  world._pending_measurement_events = []
  world._evaluation_start_frame = 100
  world._collision_during_run = False
  world._scenario_actor_collision = False
  world._termination_reason = "bridge_closed"
  world.config = SimpleNamespace(fixed_delta_seconds=0.05)
  world.recorder = Recorder()
  world.scenario = SimpleNamespace(_scenario_actor=SimpleNamespace(id=42))
  world.run = SimpleNamespace(scenario=SimpleNamespace(parameters={"terminate_on_collision": True}))
  world.exit_event = threading.Event()
  event = SimpleNamespace(
    frame=103,
    normal_impulse=SimpleNamespace(x=3.0, y=4.0, z=0.0),
    other_actor=SimpleNamespace(id=42, type_id="vehicle.test"),
  )

  world._on_collision(event)
  assert world.recorder.events == []
  world._drain_measurement_events(102)
  assert world.recorder.events == []
  world._drain_measurement_events(103)

  assert len(world.recorder.events) == 1
  recorded = world.recorder.events[0]
  assert recorded.frame == 103
  assert recorded.sim_time_s == pytest.approx(0.15)
  assert recorded.details["impulse_ns"] == pytest.approx(5.0)
  assert world._scenario_actor_collision
  assert world._termination_reason == "collision"


def test_post_reset_collision_invalidates_before_evaluation() -> None:
  recorded = []
  world = object.__new__(CarlaEvaluationWorld)
  world._sensor_condition = threading.Condition()
  world._pending_measurement_events = []
  world._evaluation_start_frame = None
  world._post_reset_settling = True
  world._post_reset_frame = 50
  world._termination_reason = "bridge_closed"
  world.recorder = SimpleNamespace(record_event=lambda event: recorded.append(event))
  world.exit_event = threading.Event()
  event = SimpleNamespace(
    frame=50,
    normal_impulse=SimpleNamespace(x=1.0, y=0.0, z=0.0),
    other_actor=SimpleNamespace(id=8, type_id="static.test"),
  )

  world._on_collision(event)
  world._drain_measurement_events(50)

  assert [item.event_type for item in recorded] == ["post_reset_collision"]
  assert world._termination_reason == "post_reset_collision"
  assert world.exit_event.is_set()


def test_imu_handoff_selects_the_requested_world_frame() -> None:
  world = object.__new__(CarlaEvaluationWorld)
  world._sensor_condition = threading.Condition()
  world._imu_frames = {}
  world._latest_imu = None
  world._latest_imu_frame = -1
  world._evaluation_time_s = 0.0
  world._termination_reason = "bridge_closed"
  world.exit_event = threading.Event()
  world.recorder = SimpleNamespace(record_event=lambda event: None)
  older = SimpleNamespace(frame=7)
  requested = SimpleNamespace(frame=8)

  world._on_imu(older)
  world._on_imu(requested)

  assert world._wait_for_imu(8, timeout_s=0.01)
  assert world._latest_imu is requested
  assert world._latest_imu_frame == 8
  assert 7 not in world._imu_frames


def test_carla_recorder_collision_parser_preserves_frame_and_hero_identity() -> None:
  text = """Version: 1
Frame 12 at 0.6 seconds
 Collision id 3 between 42 (hero)  with 8
Frame 13 at 0.65 seconds
 Collision id 4 between 42 (hero)  with 4294967295

Frames: 13
Duration: 0.65 seconds
"""

  collisions, final_frame = _parse_recorder_collisions(text)

  assert final_frame == 13
  assert [(item.recorder_frame, item.actor_2_id) for item in collisions] == [
    (12, 8),
    (13, 0xFFFFFFFF),
  ]
  assert all(item.actor_1_is_hero for item in collisions)


def test_server_recorder_restores_a_terminal_collision_missing_from_callback() -> None:
  recorded = []

  class Recorder:
    events = recorded

    @staticmethod
    def record_event(event: object) -> None:
      recorded.append(event)

  query = """Version: 1
Frame 2 at 0.1 seconds
 Collision id 1 between 42 (hero)  with 8

Frames: 2
Duration: 0.1 seconds
"""
  world = object.__new__(CarlaEvaluationWorld)
  world._carla_recorder_filename = "integrity.rec"
  world._recorder_frame_index = 2
  world._recorder_world_frames = {1: 100, 2: 101}
  world._evaluation_start_recorder_frame = 1
  world._evaluation_start_frame = 100
  world._post_reset_start_recorder_frame = 1
  world._evaluation_time_s = 0.05
  world._world_frame = 101
  world._termination_reason = "timeout"
  world._collision_during_run = False
  world._scenario_actor_collision = False
  world._recorder_audit = None
  world.client = SimpleNamespace(show_recorder_file_info=lambda _name, _all: query)
  world.vehicle = SimpleNamespace(id=42)
  world.world = SimpleNamespace(
    get_actor=lambda actor_id: SimpleNamespace(type_id="vehicle.test") if actor_id == 8 else None
  )
  world.scenario = SimpleNamespace(_scenario_actor=SimpleNamespace(id=8))
  world.run = SimpleNamespace(scenario=SimpleNamespace(parameters={"terminate_on_collision": True}))
  world.config = SimpleNamespace(
    fixed_delta_seconds=0.05, recording=SimpleNamespace(carla_recorder=False)
  )
  world.recorder = Recorder()
  world.exit_event = threading.Event()

  audit = world._audit_recorder_collisions()

  collisions = [event for event in recorded if event.event_type == "collision"]
  assert audit["valid"]
  assert len(collisions) == 1
  assert collisions[0].frame == 101
  assert collisions[0].details["source"] == "carla_server_recorder"
  assert world._collision_during_run
  assert world._scenario_actor_collision
  assert world._termination_reason == "collision"


def test_synchronous_lane_geometry_records_only_crossing_onsets() -> None:
  recorded = []
  vertex_offsets = [[0.0, 2.0], [0.0, 2.0], [0.0, 0.5], [0.0, 2.0]]

  class BoundingBox:
    @staticmethod
    def get_world_vertices(_transform: object) -> list[object]:
      offsets = vertex_offsets.pop(0)
      return [SimpleNamespace(x=0.0, y=value, z=0.0) for value in offsets]

  world = object.__new__(CarlaEvaluationWorld)
  world.vehicle = SimpleNamespace(bounding_box=BoundingBox(), get_transform=lambda: object())
  world.recorder = SimpleNamespace(record_event=lambda event: recorded.append(event))
  world._evaluation_time_s = 1.0
  world._geometric_lane_crossing_active = False
  waypoint = SimpleNamespace(
    is_junction=False,
    lane_width=3.0,
    transform=SimpleNamespace(
      location=SimpleNamespace(x=0.0, y=0.0, z=0.0),
      get_right_vector=lambda: SimpleNamespace(x=0.0, y=1.0, z=0.0),
    ),
    left_lane_marking=SimpleNamespace(type="LaneMarkingType.None"),
    right_lane_marking=SimpleNamespace(type="LaneMarkingType.Solid"),
  )

  for frame in range(4):
    world._record_geometric_lane_invasion(frame, waypoint)

  assert [event.frame for event in recorded] == [0, 3]
  assert all(event.details["source"] == "synchronous_vehicle_geometry" for event in recorded)
