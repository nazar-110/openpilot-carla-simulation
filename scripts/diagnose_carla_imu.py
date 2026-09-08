"""Check native CARLA synchronous IMU delivery without the evaluation harness."""

from __future__ import annotations

import argparse
import math
import queue
from contextlib import suppress
from typing import Any

import carla


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--host", default="127.0.0.1")
  parser.add_argument("--port", type=int, default=2000)
  parser.add_argument("--ticks", type=int, default=10)
  parser.add_argument("--sensor-tick", type=float, default=0.0)
  args = parser.parse_args()

  client = carla.Client(args.host, args.port)
  client.set_timeout(30.0)
  world = client.get_world()
  print(f"map={world.get_map().name}", flush=True)
  original = world.get_settings()
  vehicle: Any | None = None
  imu: Any | None = None
  samples: queue.Queue[Any] = queue.Queue()
  try:
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    vehicle_bp = world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
    vehicle = world.try_spawn_actor(vehicle_bp, world.get_map().get_spawn_points()[0])
    if vehicle is None:
      raise RuntimeError("Could not spawn diagnostic vehicle")
    imu_bp = world.get_blueprint_library().find("sensor.other.imu")
    imu_bp.set_attribute("sensor_tick", str(args.sensor_tick))
    imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)
    imu.listen(samples.put)

    matched = 0
    for index in range(args.ticks):
      expected = int(world.tick())
      observed: list[int] = []
      deadline_samples = 0
      while deadline_samples < 8:
        try:
          sample = samples.get(timeout=1.0)
        except queue.Empty:
          deadline_samples += 1
          continue
        frame = int(sample.frame)
        observed.append(frame)
        if frame >= expected:
          break
      ok = expected in observed
      matched += int(ok)
      accel = sample.accelerometer
      gyro = sample.gyroscope
      accel_norm = math.sqrt(accel.x**2 + accel.y**2 + accel.z**2)
      gyro_norm = math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2)
      print(
        f"tick={index + 1} expected={expected} observed={observed} match={ok} "
        f"accel=({accel.x:.3f},{accel.y:.3f},{accel.z:.3f}) "
        f"accel_norm={accel_norm:.3f} gyro_norm={gyro_norm:.3f}",
        flush=True,
      )
    print(f"matched={matched}/{args.ticks}", flush=True)
    return 0 if matched == args.ticks else 1
  finally:
    if imu is not None:
      with suppress(RuntimeError):
        imu.stop()
      with suppress(RuntimeError):
        imu.destroy()
    if vehicle is not None:
      with suppress(RuntimeError):
        vehicle.destroy()
    world.apply_settings(original)


if __name__ == "__main__":
  raise SystemExit(main())
