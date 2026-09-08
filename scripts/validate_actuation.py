"""Independent real-CARLA acceleration steps; compare stock and calibrated pedals."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import carla

from opencarla_eval.actuation import ActuatorCalibration, CalibratedActuatorController


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--host", default="127.0.0.1")
  parser.add_argument(
    "--profile", type=Path, default=Path("config/calibration/lincoln_mkz_2020.json")
  )
  parser.add_argument("--output", type=Path, default=Path("reports/calibration/validation.json"))
  args = parser.parse_args()
  c = carla.Client(args.host, 2000)
  c.set_timeout(20)
  w = c.get_world()
  original = w.get_settings()
  v = None
  rows = []
  profile = ActuatorCalibration.load(args.profile)
  profile.validate_vehicle("vehicle.lincoln.mkz_2020", c.get_server_version())
  try:
    settings = w.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    settings.no_rendering_mode = True
    w.apply_settings(settings)
    pose = w.get_map().get_spawn_points()[0]
    for mode in ["stock", "calibrated"]:
      for initial in [5.0, 8.0, 12.0]:
        for requested in [-1.0, 0.0, 1.0]:
          v = w.spawn_actor(w.get_blueprint_library().find(profile.vehicle_blueprint), pose)
          for _ in range(40):
            w.tick()
          for _ in range(500):
            if v.get_velocity().length() >= initial:
              break
            v.apply_control(carla.VehicleControl(throttle=1.0))
            w.tick()
          controller = CalibratedActuatorController(profile)
          for index in range(60):
            f = v.get_transform().get_forward_vector()
            a = v.get_acceleration()
            measured = a.x * f.x + a.y * f.y + a.z * f.z
            speed = v.get_velocity().length()
            if mode == "calibrated":
              command = controller.update(
                frame=w.get_snapshot().frame,
                dt_s=0.05,
                requested_acceleration_mps2=requested,
                speed_mps=speed,
                measured_acceleration_mps2=measured,
              )
              throttle, brake = command.throttle, command.brake
            else:
              throttle, brake = (
                min(1.0, max(0.0, requested / 1.6)),
                min(1.0, max(0.0, -requested / 4.0)),
              )
            v.apply_control(carla.VehicleControl(throttle=throttle, brake=brake))
            frame = w.tick()
            a = v.get_acceleration()
            if index >= 20:
              rows.append(
                dict(
                  mode=mode,
                  initial_speed_mps=initial,
                  request_mps2=requested,
                  measured_mps2=a.x * f.x + a.y * f.y + a.z * f.z,
                  speed_mps=speed,
                  frame=frame,
                )
              )
          v.destroy()
          v = None
    metrics = {}
    for mode in ["stock", "calibrated"]:
      errors = [r["measured_mps2"] - r["request_mps2"] for r in rows if r["mode"] == mode]
      metrics[mode] = dict(
        samples=len(errors),
        acceleration_rmse_mps2=math.sqrt(sum(e * e for e in errors) / len(errors)),
        acceleration_mae_mps2=sum(abs(e) for e in errors) / len(errors),
      )
    result = dict(
      protocol="3 second independent acceleration steps; first second excluded; identical sample counts",
      scope="dry-road longitudinal actuator check, not an OpenPilot driving-quality score",
      metrics=metrics,
      samples=rows,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))
  finally:
    if v is not None:
      v.destroy()
    w.apply_settings(original)


if __name__ == "__main__":
  main()
