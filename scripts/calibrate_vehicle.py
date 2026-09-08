"""Measure the real CARLA actor response; never supplies evaluation controls."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import carla

from opencarla_eval.actuation import CalibrationSample, fit_calibration


def speed(vehicle):
  v = vehicle.get_velocity()
  return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--host", default="127.0.0.1")
  parser.add_argument("--port", type=int, default=2000)
  parser.add_argument(
    "--output", type=Path, default=Path("config/calibration/lincoln_mkz_2020.json")
  )
  parser.add_argument("--raw", type=Path, default=Path("reports/calibration/measurements.jsonl"))
  args = parser.parse_args()
  client = carla.Client(args.host, args.port)
  client.set_timeout(30)
  print("CARLA", client.get_server_version(), flush=True)
  world = client.get_world()
  if any(
    a.attributes.get("role_name") == "ego_vehicle" for a in world.get_actors().filter("vehicle.*")
  ):
    raise RuntimeError("An evaluation ego already exists; finish that trial first.")
  original = world.get_settings()
  vehicle = None
  try:
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    settings.substepping = True
    settings.max_substep_delta_time = 0.01
    settings.max_substeps = 10
    settings.no_rendering_mode = True
    world.apply_settings(settings)
    map_ = world.get_map()
    choices = []
    for pose in map_.get_spawn_points():
      wp = map_.get_waypoint(pose.location)
      ahead = wp.next(35)
      if wp.is_junction or not ahead:
        continue
      turn = abs((ahead[0].transform.rotation.yaw - pose.rotation.yaw + 180) % 360 - 180)
      choices.append((turn, pose))
    pose = min(choices, key=lambda pair: pair[0])[1]
    vehicle = world.spawn_actor(
      world.get_blueprint_library().find("vehicle.lincoln.mkz_2020"), pose
    )
    for _ in range(20):
      world.tick()
    physics = vehicle.get_physics_control()
    rows = []
    pedals = [-0.4, -0.2, -0.1, -0.05, -0.02, 0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0]
    for initial in [0.0, 2.0, 5.0, 8.0, 12.0, 16.0, 20.0, 24.0]:
      for pedal in pedals:
        vehicle.destroy()
        vehicle = world.spawn_actor(
          world.get_blueprint_library().find("vehicle.lincoln.mkz_2020"), pose
        )
        for _ in range(20):
          world.tick()
        # Reach speed through actual drivetrain dynamics. Setting chassis velocity
        # leaves wheel RPM inconsistent and produces fictitious tire-slip braking.
        for _ in range(500):
          if speed(vehicle) >= initial - 0.1:
            break
          vehicle.apply_control(carla.VehicleControl(throttle=1.0))
          world.tick()
        else:
          raise RuntimeError(f"Could not reach calibration speed {initial}")
        vehicle.apply_control(carla.VehicleControl(throttle=max(0, pedal), brake=max(0, -pedal)))
        for index in range(16):
          before = speed(vehicle)
          frame = world.tick()
          after = speed(vehicle)
          a = vehicle.get_acceleration()
          f = vehicle.get_transform().get_forward_vector()
          if index >= 3 and 1.5 <= (before + after) / 2 <= 17.5:
            rows.append(
              {
                "frame": frame,
                "speed_mps": (before + after) / 2,
                "signed_pedal": pedal,
                "acceleration_mps2": (after - before) / 0.05,
                "carla_acceleration_mps2": a.x * f.x + a.y * f.y + a.z * f.z,
                "nominal_speed_mps": initial,
              }
            )
      print(f"Measured nominal speed {initial:g} m/s; {len(rows)} samples", flush=True)
    steering_checks = []
    for initial in [2.0, 8.0, 12.0]:
      vehicle.destroy()
      vehicle = world.spawn_actor(
        world.get_blueprint_library().find("vehicle.lincoln.mkz_2020"), pose
      )
      for _ in range(20):
        world.tick()
      for _ in range(500):
        if speed(vehicle) >= initial - 0.1:
          break
        vehicle.apply_control(carla.VehicleControl(throttle=1.0))
        world.tick()
      vehicle.apply_control(carla.VehicleControl(throttle=0.25, steer=0.1))
      for _ in range(5):
        world.tick()
      steering_checks.append(
        {
          "speed_mps": speed(vehicle),
          "normalized_steer": 0.1,
          "front_left_angle_deg": vehicle.get_wheel_steer_angle(
            carla.VehicleWheelLocation.FL_Wheel
          ),
          "front_right_angle_deg": vehicle.get_wheel_steer_angle(
            carla.VehicleWheelLocation.FR_Wheel
          ),
        }
      )
    args.raw.parent.mkdir(parents=True, exist_ok=True)
    args.raw.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
    metadata = {
      "source": "real CARLA synchronous pedal sweeps",
      "map": map_.name,
      "fixed_delta_seconds": 0.05,
      "friction_scale": 1.0,
      "measurement_sha256": hashlib.sha256(args.raw.read_bytes()).hexdigest(),
      "steering_checks": steering_checks,
      "physics": {
        "mass_kg": physics.mass,
        "steering_curve": [[p.x, p.y] for p in physics.steering_curve],
        "wheel_max_steer_angle_deg": [w.max_steer_angle for w in physics.wheels],
        "wheel_tire_friction": [w.tire_friction for w in physics.wheels],
      },
    }
    samples = [
      CalibrationSample(**{k: r[k] for k in ("speed_mps", "signed_pedal", "acceleration_mps2")})
      for r in rows
    ]
    profile = fit_calibration(
      samples,
      speed_bin_centers_mps=[2, 5, 8, 12, 16],
      vehicle_blueprint=vehicle.type_id,
      carla_version=client.get_server_version(),
      metadata=metadata,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(profile.to_dict(), indent=2) + "\n", encoding="utf-8")
    print(f"Saved {args.output}", flush=True)
  finally:
    if vehicle is not None:
      vehicle.destroy()
    world.apply_settings(original)


if __name__ == "__main__":
  main()
