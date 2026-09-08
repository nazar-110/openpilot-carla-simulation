"""Empirical CARLA acceleration-to-pedal conversion.

Profiles contain measured vehicle response, not assumed throttle/brake gains.
The controller preserves OpenPilot's acceleration request and adds a bounded
tracking correction. Its state advances at most once per CARLA physics frame.
"""

from __future__ import annotations

import json
import math
from bisect import bisect_right
from dataclasses import asdict, dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from statistics import median
from typing import Any


def _finite(value: float, name: str) -> float:
  value = float(value)
  if not math.isfinite(value):
    raise ValueError(f"{name} must be finite")
  return value


def _clip(value: float, low: float, high: float) -> float:
  return min(high, max(low, value))


def _interpolate(x: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
  if x <= xs[0]:
    return ys[0]
  if x >= xs[-1]:
    return ys[-1]
  index = bisect_right(xs, x) - 1
  fraction = (x - xs[index]) / (xs[index + 1] - xs[index])
  return ys[index] + fraction * (ys[index + 1] - ys[index])


def _isotonic(values: list[float], weights: list[int]) -> tuple[float, ...]:
  """Weighted pool-adjacent-violators fit; pedal response cannot decrease."""

  blocks: list[list[float]] = []
  for index, (value, weight) in enumerate(zip(values, weights, strict=True)):
    blocks.append([float(index), float(index + 1), value * weight, float(weight)])
    while len(blocks) > 1 and blocks[-2][2] / blocks[-2][3] > blocks[-1][2] / blocks[-1][3]:
      right = blocks.pop()
      left = blocks.pop()
      blocks.append([left[0], right[1], left[2] + right[2], left[3] + right[3]])
  fitted = [0.0] * len(values)
  for start, stop, total, weight in blocks:
    fitted[int(start) : int(stop)] = [total / weight] * int(stop - start)
  return tuple(fitted)


@dataclass(frozen=True)
class CalibrationSample:
  speed_mps: float
  signed_pedal: float
  acceleration_mps2: float
  nominal_speed_mps: float | None = None

  def __post_init__(self) -> None:
    for name in ("speed_mps", "signed_pedal", "acceleration_mps2"):
      _finite(getattr(self, name), name)
    if self.speed_mps < 0 or not -1.0 <= self.signed_pedal <= 1.0:
      raise ValueError("Samples require nonnegative speed and signed pedal in [-1, 1]")


@dataclass(frozen=True)
class SpeedCurve:
  speed_mps: float
  signed_pedals: tuple[float, ...]
  accelerations_mps2: tuple[float, ...]
  sample_count: int

  def __post_init__(self) -> None:
    _finite(self.speed_mps, "curve speed")
    if self.speed_mps < 0 or len(self.signed_pedals) < 3:
      raise ValueError("Each speed curve requires a nonnegative speed and at least three pedals")
    if len(self.signed_pedals) != len(self.accelerations_mps2):
      raise ValueError("Pedal and acceleration arrays must have equal lengths")
    if self.sample_count < len(self.signed_pedals):
      raise ValueError("Curve sample count is smaller than its pedal count")
    for value in self.signed_pedals + self.accelerations_mps2:
      _finite(value, "curve value")
    if any(b <= a for a, b in zip(self.signed_pedals, self.signed_pedals[1:], strict=False)):
      raise ValueError("Pedal knots must be strictly increasing")
    if not -1.0 <= self.signed_pedals[0] < 0 < self.signed_pedals[-1] <= 1.0:
      raise ValueError("Each curve must cover both braking and throttle within [-1, 1]")
    if any(
      b < a for a, b in zip(self.accelerations_mps2, self.accelerations_mps2[1:], strict=False)
    ):
      raise ValueError("Acceleration response must be monotonic")
    if self.accelerations_mps2[-1] <= self.accelerations_mps2[0]:
      raise ValueError("Calibration curve has no measurable acceleration range")


@dataclass(frozen=True)
class ActuatorCalibration:
  vehicle_blueprint: str
  carla_version: str
  speed_curves: tuple[SpeedCurve, ...]
  metadata: dict[str, Any] = field(default_factory=dict)
  fit: dict[str, Any] = field(default_factory=dict)
  schema_version: int = 1

  def __post_init__(self) -> None:
    if self.schema_version != 1:
      raise ValueError(f"Unsupported actuator calibration schema: {self.schema_version}")
    if not self.vehicle_blueprint or not self.carla_version or not self.speed_curves:
      raise ValueError("Calibration requires vehicle, CARLA version, and measured speed curves")
    speeds = [curve.speed_mps for curve in self.speed_curves]
    if any(b <= a for a, b in zip(speeds, speeds[1:], strict=False)):
      raise ValueError("Calibration speeds must be strictly increasing")

  def _response(self, speed_mps: float) -> tuple[tuple[float, ...], tuple[float, ...]]:
    speed = max(0.0, _finite(speed_mps, "speed"))
    speeds = tuple(curve.speed_mps for curve in self.speed_curves)
    if speed <= speeds[0]:
      curve = self.speed_curves[0]
      return curve.signed_pedals, curve.accelerations_mps2
    if speed >= speeds[-1]:
      curve = self.speed_curves[-1]
      return curve.signed_pedals, curve.accelerations_mps2
    index = bisect_right(speeds, speed) - 1
    low, high = self.speed_curves[index : index + 2]
    fraction = (speed - low.speed_mps) / (high.speed_mps - low.speed_mps)
    # Restrict to pedal support measured in both neighboring speed bins.
    minimum = max(low.signed_pedals[0], high.signed_pedals[0])
    maximum = min(low.signed_pedals[-1], high.signed_pedals[-1])
    pedals = tuple(
      sorted({p for p in low.signed_pedals + high.signed_pedals if minimum <= p <= maximum})
    )
    accelerations = tuple(
      (1.0 - fraction) * _interpolate(p, low.signed_pedals, low.accelerations_mps2)
      + fraction * _interpolate(p, high.signed_pedals, high.accelerations_mps2)
      for p in pedals
    )
    return pedals, accelerations

  def acceleration_for_pedal(self, speed_mps: float, signed_pedal: float) -> float:
    pedals, accelerations = self._response(speed_mps)
    return _interpolate(_finite(signed_pedal, "pedal"), pedals, accelerations)

  def acceleration_limits(self, speed_mps: float) -> tuple[float, float]:
    _, accelerations = self._response(speed_mps)
    return accelerations[0], accelerations[-1]

  def pedal_for_acceleration(self, speed_mps: float, acceleration_mps2: float) -> float:
    pedals, accelerations = self._response(speed_mps)
    requested = _finite(acceleration_mps2, "requested acceleration")
    # A standstill/deadband plateau must not turn a zero request into full brake.
    matches = [p for p, a in zip(pedals, accelerations, strict=True) if abs(a - requested) < 1e-9]
    if matches:
      if min(matches) <= 0.0 <= max(matches):
        return 0.0
      return min(matches, key=abs)
    if requested < accelerations[0]:
      return pedals[0]
    if requested > accelerations[-1]:
      return pedals[-1]
    index = bisect_right(accelerations, requested) - 1
    fraction = (requested - accelerations[index]) / (
      accelerations[index + 1] - accelerations[index]
    )
    return pedals[index] + fraction * (pedals[index + 1] - pedals[index])

  def to_dict(self) -> dict[str, Any]:
    return asdict(self)

  @classmethod
  def from_dict(cls, data: dict[str, Any]) -> ActuatorCalibration:
    curves = tuple(
      SpeedCurve(
        speed_mps=float(curve["speed_mps"]),
        signed_pedals=tuple(float(value) for value in curve["signed_pedals"]),
        accelerations_mps2=tuple(float(value) for value in curve["accelerations_mps2"]),
        sample_count=int(curve["sample_count"]),
      )
      for curve in data["speed_curves"]
    )
    return cls(
      vehicle_blueprint=str(data["vehicle_blueprint"]),
      carla_version=str(data["carla_version"]),
      speed_curves=curves,
      metadata=dict(data.get("metadata", {})),
      fit=dict(data.get("fit", {})),
      schema_version=int(data.get("schema_version", 0)),
    )

  @classmethod
  def load(cls, path: str | Path) -> ActuatorCalibration:
    return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

  def validate_vehicle(self, vehicle_blueprint: str, carla_version: str) -> None:
    if vehicle_blueprint != self.vehicle_blueprint or carla_version != self.carla_version:
      raise ValueError(
        f"Actuator profile is for {self.vehicle_blueprint} / CARLA {self.carla_version}, "
        f"not {vehicle_blueprint} / CARLA {carla_version}"
      )


def fit_calibration(
  samples: list[CalibrationSample],
  *,
  speed_bin_centers_mps: tuple[float, ...] | list[float],
  vehicle_blueprint: str,
  carla_version: str,
  metadata: dict[str, Any] | None = None,
) -> ActuatorCalibration:
  """Fit measured response by actual speed, using medians and monotonic regression.

  Each caller-selected speed bin must contain measurements at three or more
  pedals, including both throttle and braking. The fit never invents missing
  speed bins or extrapolates beyond measured pedal support.
  """

  centers = tuple(_finite(speed, "speed bin") for speed in speed_bin_centers_mps)
  if (
    not centers or centers[0] < 0 or any(b <= a for a, b in zip(centers, centers[1:], strict=False))
  ):
    raise ValueError("Speed bin centers must be nonnegative and strictly increasing")
  grouped: dict[float, dict[float, list[float]]] = {center: {} for center in centers}
  for sample in samples:
    center = min(centers, key=lambda value: abs(sample.speed_mps - value))
    pedal = round(sample.signed_pedal, 6)
    grouped[center].setdefault(pedal, []).append(sample.acceleration_mps2)
  curves = []
  for center in centers:
    values = grouped[center]
    pedals = tuple(sorted(values))
    accelerations = [median(values[pedal]) for pedal in pedals]
    counts = [len(values[pedal]) for pedal in pedals]
    curves.append(SpeedCurve(center, pedals, _isotonic(accelerations, counts), sum(counts)))
  profile = ActuatorCalibration(
    vehicle_blueprint,
    carla_version,
    tuple(curves),
    {"created_utc": datetime.now(UTC).isoformat(), **(metadata or {})},
  )
  errors = [
    sample.acceleration_mps2 - profile.acceleration_for_pedal(sample.speed_mps, sample.signed_pedal)
    for sample in samples
  ]
  fit = {
    "method": "actual-speed bins, median pedal response, weighted isotonic regression",
    "sample_count": len(samples),
    "training_rmse_mps2": math.sqrt(sum(error * error for error in errors) / len(errors)),
    "speed_bin_centers_mps": list(centers),
    "validation": "training residual only; independent closed-loop verification is required",
  }
  return ActuatorCalibration(
    profile.vehicle_blueprint, profile.carla_version, profile.speed_curves, profile.metadata, fit
  )


@dataclass(frozen=True)
class ActuatorCommand:
  frame: int
  throttle: float
  brake: float
  requested_acceleration_mps2: float
  corrected_acceleration_mps2: float
  measured_acceleration_mps2: float
  feedforward_pedal: float
  feedback_correction_mps2: float
  saturated: bool
  speed_outside_calibration: bool


class CalibratedActuatorController:
  """Inverse empirical response with bounded acceleration-domain PI feedback.

  Tuning values are controller settings, not empirical calibration results.
  There is deliberately no speed target, throttle bootstrap, or jerk clamp.
  """

  def __init__(
    self,
    calibration: ActuatorCalibration,
    *,
    proportional_gain: float = 0.15,
    integral_gain: float = 0.25,
    maximum_correction_mps2: float = 0.75,
    measurement_filter_s: float = 0.15,
  ) -> None:
    self.calibration = calibration
    for name, value in {
      "proportional_gain": proportional_gain,
      "integral_gain": integral_gain,
      "maximum_correction_mps2": maximum_correction_mps2,
      "measurement_filter_s": measurement_filter_s,
    }.items():
      if _finite(value, name) < 0:
        raise ValueError(f"{name} must be nonnegative")
      setattr(self, name, float(value))
    self.reset()

  def reset(self) -> None:
    self._integral = 0.0
    self._filtered_acceleration: float | None = None
    self._last_command: ActuatorCommand | None = None

  def update(
    self,
    *,
    frame: int,
    dt_s: float,
    requested_acceleration_mps2: float,
    speed_mps: float,
    measured_acceleration_mps2: float,
  ) -> ActuatorCommand:
    requested = _finite(requested_acceleration_mps2, "requested acceleration")
    measured = _finite(measured_acceleration_mps2, "measured acceleration")
    speed = max(0.0, _finite(speed_mps, "speed"))
    dt = _finite(dt_s, "physics step")
    if dt <= 0 or not isinstance(frame, int) or frame < 0:
      raise ValueError("Controller requires a positive physics step and nonnegative integer frame")
    if self._last_command is not None:
      if frame == self._last_command.frame:
        # The bridge polls commands at 100 Hz while physics advances at 20 Hz.
        # Reuse feedback state, but never discard a newer selected carControl.
        previous = self._last_command
        low, high = self.calibration.acceleration_limits(speed)
        corrected = requested + previous.feedback_correction_mps2
        pedal = self.calibration.pedal_for_acceleration(speed, corrected)
        self._last_command = ActuatorCommand(
          frame=frame,
          throttle=max(0.0, pedal),
          brake=max(0.0, -pedal),
          requested_acceleration_mps2=requested,
          corrected_acceleration_mps2=corrected,
          measured_acceleration_mps2=previous.measured_acceleration_mps2,
          feedforward_pedal=self.calibration.pedal_for_acceleration(speed, requested),
          feedback_correction_mps2=previous.feedback_correction_mps2,
          saturated=corrected < low or corrected > high,
          speed_outside_calibration=previous.speed_outside_calibration,
        )
        return self._last_command
      if frame < self._last_command.frame:
        self.reset()
    if self._filtered_acceleration is None:
      self._filtered_acceleration = measured
    else:
      alpha = 1.0 if self.measurement_filter_s == 0 else dt / (self.measurement_filter_s + dt)
      self._filtered_acceleration += alpha * (measured - self._filtered_acceleration)
    error = requested - self._filtered_acceleration
    low, high = self.calibration.acceleration_limits(speed)
    candidate_integral = _clip(
      self._integral + self.integral_gain * error * dt,
      -self.maximum_correction_mps2,
      self.maximum_correction_mps2,
    )
    proposed = requested + self.proportional_gain * error + candidate_integral
    if not ((proposed >= high and error > 0) or (proposed <= low and error < 0)):
      self._integral = candidate_integral
    if speed < 0.1 and requested <= 0.0:
      self._integral = 0.0
    correction = _clip(
      self.proportional_gain * error + self._integral,
      -self.maximum_correction_mps2,
      self.maximum_correction_mps2,
    )
    corrected = requested + correction
    feedforward = self.calibration.pedal_for_acceleration(speed, requested)
    pedal = self.calibration.pedal_for_acceleration(speed, corrected)
    self._last_command = ActuatorCommand(
      frame=frame,
      throttle=max(0.0, pedal),
      brake=max(0.0, -pedal),
      requested_acceleration_mps2=requested,
      corrected_acceleration_mps2=corrected,
      measured_acceleration_mps2=measured,
      feedforward_pedal=feedforward,
      feedback_correction_mps2=correction,
      saturated=corrected < low or corrected > high,
      speed_outside_calibration=(
        speed < self.calibration.speed_curves[0].speed_mps
        or speed > self.calibration.speed_curves[-1].speed_mps
      ),
    )
    return self._last_command
