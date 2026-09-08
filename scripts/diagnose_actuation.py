"""Print short real-physics traces to diagnose calibration timing."""

import carla

c = carla.Client("127.0.0.1", 2000)
c.set_timeout(10)
w = c.get_world()
original = w.get_settings()
v = None
try:
  s = w.get_settings()
  s.synchronous_mode = True
  s.fixed_delta_seconds = 0.05
  s.no_rendering_mode = True
  w.apply_settings(s)
  v = w.spawn_actor(
    w.get_blueprint_library().find("vehicle.lincoln.mkz_2020"), w.get_map().get_spawn_points()[0]
  )
  for _ in range(40):
    w.tick()
  for pedal in [0.2, 0.5, 1.0, 0.0, -0.02, -0.1]:
    v.apply_control(carla.VehicleControl(throttle=max(0.0, pedal), brake=max(0.0, -pedal)))
    for i in range(20):
      frame = w.tick()
      snap = w.get_snapshot()
      if i % 4 == 0:
        print(
          pedal,
          frame,
          snap.frame,
          snap.timestamp.delta_seconds,
          v.get_velocity(),
          v.get_control(),
          flush=True,
        )
finally:
  if v:
    v.destroy()
  w.apply_settings(original)
