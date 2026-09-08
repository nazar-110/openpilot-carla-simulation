import math

import pytest

from opencarla_eval.actuation import (
  ActuatorCalibration,
  CalibratedActuatorController,
  CalibrationSample,
  SpeedCurve,
  fit_calibration,
)


def synthetic_profile() -> ActuatorCalibration:
  # Synthetic known plant response for verification; never a production profile.
  samples = [
    CalibrationSample(speed, pedal, (2.0 * pedal if pedal > 0 else 20.0 * pedal) - speed * 0.05)
    for speed in (0.0, 10.0)
    for pedal in (-1.0, -0.2, 0.0, 0.2, 1.0)
    for _ in range(3)
  ]
  return fit_calibration(
    samples,
    speed_bin_centers_mps=(0.0, 10.0),
    vehicle_blueprint="test.synthetic",
    carla_version="test",
  )


def test_inverse_fit_interpolates_speed_and_avoids_generic_brake_gain() -> None:
  profile = synthetic_profile()
  assert profile.pedal_for_acceleration(5.0, -3.5) == pytest.approx(-0.1625)
  assert profile.pedal_for_acceleration(5.0, 0.0) == pytest.approx(0.125)
  assert profile.pedal_for_acceleration(5.0, 1.0) == pytest.approx(0.625)
  assert profile.fit["training_rmse_mps2"] == pytest.approx(0.0)
  for speed in (0.0, 2.0, 5.0, 10.0):
    for acceleration in (-10.0, -1.0, 0.0, 1.0):
      pedal = profile.pedal_for_acceleration(speed, acceleration)
      assert profile.acceleration_for_pedal(speed, pedal) == pytest.approx(acceleration)


def test_fit_uses_actual_speed_and_is_robust_to_outliers_and_nonmonotonic_noise() -> None:
  samples = [
    CalibrationSample(5.0, pedal, acceleration, nominal_speed_mps=0.0)
    for pedal, acceleration in [
      (-0.5, -8.0),
      (0.0, -0.2),
      (0.2, 1.0),
      (0.4, 0.8),
      (1.0, 2.0),
      (1.0, 2.0),
      (1.0, 200.0),
    ]
  ]
  profile = fit_calibration(
    samples, speed_bin_centers_mps=(5.0,), vehicle_blueprint="test", carla_version="test"
  )
  assert profile.speed_curves[0].accelerations_mps2 == pytest.approx((-8.0, -0.2, 0.9, 0.9, 2.0))
  assert profile.pedal_for_acceleration(5.0, 0.9) == pytest.approx(0.2)


def test_stationary_braking_plateau_does_not_brake_for_zero_acceleration() -> None:
  profile = ActuatorCalibration(
    "test", "test", (SpeedCurve(0.0, (-1.0, -0.5, 0.0, 0.1, 1.0), (0.0, 0.0, 0.0, 0.0, 2.0), 5),)
  )
  assert profile.pedal_for_acceleration(0.0, 0.0) == 0.0
  assert profile.pedal_for_acceleration(0.0, -1.0) == -1.0
  assert profile.pedal_for_acceleration(0.0, 1.0) == pytest.approx(0.55)


def test_profile_round_trip_and_vehicle_guard(tmp_path) -> None:
  import json

  profile = synthetic_profile()
  path = tmp_path / "profile.json"
  path.write_text(json.dumps(profile.to_dict()), encoding="utf-8")
  loaded = ActuatorCalibration.load(path)
  assert loaded == profile
  loaded.validate_vehicle("test.synthetic", "test")
  with pytest.raises(ValueError, match="profile is for"):
    loaded.validate_vehicle("different.vehicle", "test")
  with pytest.raises(ValueError, match="schema"):
    ActuatorCalibration.from_dict({**profile.to_dict(), "schema_version": 99})


def test_controller_advances_once_per_physics_frame_and_preserves_raw_request() -> None:
  controller = CalibratedActuatorController(synthetic_profile(), measurement_filter_s=0.0)
  first = controller.update(
    frame=10,
    dt_s=0.05,
    requested_acceleration_mps2=-3.5,
    speed_mps=5.0,
    measured_acceleration_mps2=-3.0,
  )
  for _ in range(5):
    repeated = controller.update(
      frame=10,
      dt_s=0.05,
      requested_acceleration_mps2=1.6,
      speed_mps=5.0,
      measured_acceleration_mps2=-10.0,
    )
    assert repeated.feedback_correction_mps2 == first.feedback_correction_mps2
    assert repeated.measured_acceleration_mps2 == first.measured_acceleration_mps2
    assert repeated.requested_acceleration_mps2 == 1.6
  assert first.requested_acceleration_mps2 == -3.5
  assert first.throttle == 0
  assert 0 < first.brake < 0.2
  next_frame = controller.update(
    frame=11,
    dt_s=0.05,
    requested_acceleration_mps2=1.6,
    speed_mps=5.0,
    measured_acceleration_mps2=1.6,
  )
  assert next_frame.requested_acceleration_mps2 == 1.6
  assert next_frame.brake == 0


def test_feedback_is_bounded_and_saturation_does_not_wind_up() -> None:
  controller = CalibratedActuatorController(synthetic_profile(), measurement_filter_s=0.0)
  for frame in range(100):
    command = controller.update(
      frame=frame,
      dt_s=0.05,
      requested_acceleration_mps2=8.0,
      speed_mps=5.0,
      measured_acceleration_mps2=0.0,
    )
    assert command.saturated
    assert command.throttle == 1.0
    assert abs(command.feedback_correction_mps2) <= 0.75
  assert controller._integral == 0.0
  neutral = controller.update(
    frame=100,
    dt_s=0.05,
    requested_acceleration_mps2=0.0,
    speed_mps=5.0,
    measured_acceleration_mps2=0.0,
  )
  assert neutral.feedback_correction_mps2 == 0.0
  assert neutral.throttle == pytest.approx(0.125)


def test_feedback_reduces_constant_model_bias_without_throttle_brake_overlap() -> None:
  profile = synthetic_profile()
  controller = CalibratedActuatorController(profile, measurement_filter_s=0.0)
  measured = 0.0
  for frame in range(300):
    command = controller.update(
      frame=frame,
      dt_s=0.05,
      requested_acceleration_mps2=0.5,
      speed_mps=5.0,
      measured_acceleration_mps2=measured,
    )
    measured = profile.acceleration_for_pedal(5.0, command.throttle - command.brake) - 0.3
    assert command.throttle * command.brake == 0.0
  assert measured == pytest.approx(0.5, abs=0.02)


def test_reset_frame_and_invalid_inputs() -> None:
  controller = CalibratedActuatorController(synthetic_profile())
  values = dict(
    dt_s=0.05, requested_acceleration_mps2=0.5, speed_mps=5.0, measured_acceleration_mps2=0.0
  )
  first = controller.update(frame=100, **values)
  restarted = controller.update(frame=0, **values)
  assert restarted.feedback_correction_mps2 == first.feedback_correction_mps2
  with pytest.raises(ValueError, match="finite"):
    controller.update(frame=1, **{**values, "requested_acceleration_mps2": math.nan})
  with pytest.raises(ValueError, match="positive physics"):
    controller.update(frame=1, **{**values, "dt_s": 0.0})
  with pytest.raises(ValueError, match="at least three"):
    fit_calibration(
      [], speed_bin_centers_mps=(0.0,), vehicle_blueprint="test", carla_version="test"
    )
