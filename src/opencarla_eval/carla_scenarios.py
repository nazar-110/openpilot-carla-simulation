"""Deterministic CARLA spawn selection and scripted urban scenario actors."""

from __future__ import annotations

import math
import random
from contextlib import suppress
from dataclasses import dataclass
from typing import Any

from .errors import SimulatorConnectionError
from .models import EventRecord, RunSpec
from .recorder import RunRecorder

_ROUTE_RESERVE_M = 10.0


def _angle_difference(a: float, b: float) -> float:
  return abs((a - b + 180.0) % 360.0 - 180.0)


def _distance(a: Any, b: Any) -> float:
  return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _longitudinal_distance(ego: Any, target: Any) -> float:
  transform = ego.get_transform()
  forward = transform.get_forward_vector()
  delta = target.get_location() - transform.location
  return delta.x * forward.x + delta.y * forward.y + delta.z * forward.z


def _same_direction(first: Any, second: Any) -> bool:
  return _angle_difference(first.transform.rotation.yaw, second.transform.rotation.yaw) < 45.0


def _choose_continuation(waypoint: Any, distance_m: float) -> Any | None:
  candidates = waypoint.next(distance_m)
  if not candidates:
    return None
  return min(
    candidates,
    key=lambda candidate: _angle_difference(
      waypoint.transform.rotation.yaw, candidate.transform.rotation.yaw
    ),
  )


def _route_heading_change(waypoint: Any, length_m: float, step_m: float = 5.0) -> float | None:
  start_yaw = waypoint.transform.rotation.yaw
  current = waypoint
  maximum = 0.0
  for _ in range(max(1, round(length_m / step_m))):
    current = _choose_continuation(current, step_m)
    if current is None:
      return None
    maximum = max(maximum, _angle_difference(start_yaw, current.transform.rotation.yaw))
  return maximum


def build_reference_route(
  start_waypoint: Any, length_m: float, step_m: float = 5.0
) -> tuple[list[Any], list[float]]:
  """Build the deterministic straightest-continuation route used for evaluation."""

  waypoints = [start_waypoint]
  cumulative = [0.0]
  current = start_waypoint
  progress = 0.0
  while progress < length_m:
    step = min(step_m, length_m - progress)
    next_waypoint = _choose_continuation(current, step)
    if next_waypoint is None:
      raise SimulatorConnectionError(
        f"Map route ends at {progress:.1f} m before requested length {length_m:.1f} m"
      )
    progress += step
    waypoints.append(next_waypoint)
    cumulative.append(progress)
    current = next_waypoint
  return waypoints, cumulative


def _adjacent_driving_lane(waypoint: Any, preferred_side: str = "left") -> tuple[Any, str] | None:
  ordered = (
    ((waypoint.get_left_lane(), "left"), (waypoint.get_right_lane(), "right"))
    if preferred_side == "left"
    else ((waypoint.get_right_lane(), "right"), (waypoint.get_left_lane(), "left"))
  )
  for candidate, side in ordered:
    if candidate is None:
      continue
    if str(candidate.lane_type).endswith("Driving") and _same_direction(waypoint, candidate):
      return candidate, side
  return None


@dataclass(frozen=True)
class SpawnSelection:
  transform: Any
  waypoint: Any
  traffic_light: Any | None = None


def select_spawn(world: Any, run: RunSpec) -> SpawnSelection:
  """Select a topology-compatible spawn using only the run's paired seed."""

  world_map = world.get_map()
  rng = random.Random(run.seed)
  spawn_points = list(world_map.get_spawn_points())
  rng.shuffle(spawn_points)
  kind = run.scenario.kind

  if kind == "signalized_intersection":
    lights = list(world.get_actors().filter("traffic.traffic_light*"))
    rng.shuffle(lights)
    sight_distance = float(run.scenario.parameters.get("signal_sight_distance_m", 45.0))
    for light in lights:
      for stop_waypoint in light.get_stop_waypoints():
        previous = stop_waypoint.previous(sight_distance)
        if not previous:
          continue
        waypoint = min(
          previous,
          key=lambda candidate: _angle_difference(
            stop_waypoint.transform.rotation.yaw, candidate.transform.rotation.yaw
          ),
        )
        if waypoint.is_junction:
          continue
        try:
          build_reference_route(waypoint, run.scenario.route_length_m + _ROUTE_RESERVE_M)
        except SimulatorConnectionError:
          continue
        transform = waypoint.transform
        transform.location.z += 0.35
        return SpawnSelection(transform, waypoint, light)
    raise SimulatorConnectionError(
      f"No suitable traffic-light approach found in {world_map.name} for seed {run.seed}"
    )

  minimum_change = float(run.scenario.parameters.get("require_heading_change_deg", 0.0))
  preferred_side = str(run.scenario.parameters.get("preferred_side", "left"))
  minimum_junction_clearance_m = float(
    run.scenario.parameters.get("minimum_junction_clearance_m", 0.0)
  )
  required_route_length = run.scenario.route_length_m + _ROUTE_RESERVE_M
  required_span = min(required_route_length, 180.0)
  for transform in spawn_points:
    waypoint = world_map.get_waypoint(transform.location, project_to_road=True)
    if waypoint is None or waypoint.is_junction:
      continue
    if not str(waypoint.lane_type).endswith("Driving"):
      continue
    heading_change = _route_heading_change(waypoint, required_span)
    if heading_change is None or heading_change < minimum_change:
      continue
    if kind == "vehicle_cut_in" and _adjacent_driving_lane(waypoint, preferred_side) is None:
      continue
    try:
      route, cumulative_m = build_reference_route(waypoint, required_route_length)
    except SimulatorConnectionError:
      continue
    first_junction_m = next(
      (
        distance
        for candidate, distance in zip(route, cumulative_m, strict=True)
        if candidate.is_junction
      ),
      math.inf,
    )
    if first_junction_m < minimum_junction_clearance_m:
      continue
    transform.location.z += 0.35
    return SpawnSelection(transform, waypoint)
  raise SimulatorConnectionError(
    f"No topology-compatible spawn found for {run.scenario.id} in {world_map.name}; "
    "try another seed or inspect the map topology"
  )


class CarlaScenarioRuntime:
  """Own scripted hazards, background traffic, and scenario realization events."""

  def __init__(
    self,
    client: Any,
    world: Any,
    ego: Any,
    start_waypoint: Any,
    selected_light: Any | None,
    traffic_manager: Any,
    traffic_manager_port: int,
    run: RunSpec,
    recorder: RunRecorder,
    reference_route: list[Any],
  ) -> None:
    self.client = client
    self.world = world
    self.ego = ego
    self.start_waypoint = start_waypoint
    self.selected_light = selected_light
    self.traffic_manager = traffic_manager
    self.traffic_manager_port = traffic_manager_port
    self.run = run
    self.recorder = recorder
    self.reference_route = reference_route
    self.rng = random.Random(run.seed + 17)
    self.scripted_actors: list[Any] = []
    self.background_actors: list[Any] = []
    self.triggered = False
    self.trigger_time_s: float | None = None
    self._lead_resumed = False
    self._lead_stop_time_s: float | None = None
    self._lead_previous_speed_mps: float | None = None
    self._lead_previous_time_s: float | None = None
    self._lead_peak_deceleration_mps2 = 0.0
    self._signal_previous_side: float | None = None
    self._signal_violation = False
    self._light_was_controlled = False
    self._signal_group: list[Any] = []
    self._scenario_actor: Any | None = None
    self._scenario_actor_initial_location: Any | None = None
    self._cut_in_min_lateral_separation_m = math.inf
    self._background_requested = run.condition.traffic_vehicles
    self._crossing_direction: Any | None = None
    self.armed = False

  def arm(self) -> None:
    """Create all moving actors at the common post-warmup evaluation boundary."""

    if self.armed:
      return
    self._setup()
    self.armed = True
    self.recorder.record_event(
      EventRecord(
        "scenario_armed",
        0.0,
        details={
          "scripted_actor_count": len(self.scripted_actors),
          "background_actor_count": len(self.background_actors),
        },
      )
    )

  def _waypoint_ahead(self, distance_m: float, start: Any | None = None) -> Any:
    current = start or self.start_waypoint
    remaining = distance_m
    while remaining > 0:
      step = min(5.0, remaining)
      next_waypoint = _choose_continuation(current, step)
      if next_waypoint is None:
        raise SimulatorConnectionError(
          f"Route ends before {distance_m:.1f} m while setting up {self.run.scenario.id}"
        )
      current = next_waypoint
      remaining -= step
    return current

  def _vehicle_blueprint(self, role_name: str) -> Any:
    candidates = [
      item
      for item in self.world.get_blueprint_library().filter("vehicle.*")
      if int(item.get_attribute("number_of_wheels")) == 4
      and not any(word in item.id for word in ("ambulance", "firetruck", "carlacola"))
    ]
    if not candidates:
      raise SimulatorConnectionError("CARLA blueprint library has no four-wheel vehicle")
    candidates.sort(key=lambda item: item.id)
    blueprint = candidates[self.rng.randrange(len(candidates))]
    if blueprint.has_attribute("role_name"):
      blueprint.set_attribute("role_name", role_name)
    if blueprint.has_attribute("color"):
      colors = list(blueprint.get_attribute("color").recommended_values)
      if colors:
        blueprint.set_attribute("color", colors[self.rng.randrange(len(colors))])
    return blueprint

  def _spawn_vehicle(self, waypoint: Any, role_name: str, lateral_offset_m: float = 0.0) -> Any:
    transform = waypoint.transform
    if lateral_offset_m:
      right = transform.get_right_vector()
      transform.location.x += right.x * lateral_offset_m
      transform.location.y += right.y * lateral_offset_m
      transform.location.z += right.z * lateral_offset_m
    transform.location.z += 0.35
    actor = self.world.try_spawn_actor(self._vehicle_blueprint(role_name), transform)
    if actor is None:
      raise SimulatorConnectionError(f"Failed to spawn required actor {role_name}")
    self._apply_tire_friction(actor)
    self.scripted_actors.append(actor)
    return actor

  def _apply_tire_friction(self, actor: Any) -> None:
    factor = self.run.condition.friction
    if math.isclose(factor, 1.0):
      return
    physics = actor.get_physics_control()
    wheels = list(physics.wheels)
    for wheel in wheels:
      wheel.tire_friction *= factor
    physics.wheels = wheels
    actor.apply_physics_control(physics)

  def _set_desired_speed(
    self, actor: Any, speed_mps: float, *, initialize_velocity: bool = True
  ) -> None:
    import carla

    if initialize_velocity:
      forward = actor.get_transform().get_forward_vector()
      actor.set_target_velocity(
        carla.Vector3D(
          x=forward.x * speed_mps,
          y=forward.y * speed_mps,
          z=forward.z * speed_mps,
        )
      )
    actor.set_autopilot(True, self.traffic_manager_port)
    self.traffic_manager.auto_lane_change(actor, False)
    if hasattr(self.traffic_manager, "set_desired_speed"):
      self.traffic_manager.set_desired_speed(actor, speed_mps * 3.6)
    else:  # pragma: no cover - compatibility with older CARLA clients
      limit = max(actor.get_speed_limit(), 1.0)
      difference = 100.0 * (1.0 - speed_mps * 3.6 / limit)
      self.traffic_manager.vehicle_percentage_speed_difference(actor, difference)

  def _setup(self) -> None:
    import carla

    kind = self.run.scenario.kind
    params = self.run.scenario.parameters
    if kind == "lead_vehicle_braking":
      waypoint = self._waypoint_ahead(float(params.get("lead_initial_gap_m", 30.0)))
      actor = self._spawn_vehicle(waypoint, "scenario_lead")
      self._set_desired_speed(actor, float(params.get("lead_speed_mps", 11.11)))
      self._scenario_actor = actor

    elif kind == "pedestrian_crossing":
      crossing = self._waypoint_ahead(float(params.get("crossing_distance_m", 35.0)))
      transform = crossing.transform
      right = transform.get_right_vector()
      side_sign = 1.0 if str(params.get("start_side", "right")) == "right" else -1.0
      offset = crossing.lane_width / 2.0 + 2.0
      transform.location.x += right.x * offset * side_sign
      transform.location.y += right.y * offset * side_sign
      transform.location.z += 0.3
      walkers = sorted(
        self.world.get_blueprint_library().filter("walker.pedestrian.*"), key=lambda item: item.id
      )
      if not walkers:
        raise SimulatorConnectionError("CARLA blueprint library has no pedestrian")
      blueprint = walkers[self.rng.randrange(len(walkers))]
      actor = self.world.try_spawn_actor(blueprint, transform)
      if actor is None:
        raise SimulatorConnectionError("Failed to spawn required pedestrian actor")
      self.scripted_actors.append(actor)
      self._scenario_actor = actor
      self._crossing_direction = carla.Vector3D(
        x=-right.x * side_sign, y=-right.y * side_sign, z=0.0
      )

    elif kind == "vehicle_cut_in":
      ahead = self._waypoint_ahead(float(params.get("intruder_gap_m", 24.0)))
      adjacent = _adjacent_driving_lane(ahead, str(params.get("preferred_side", "left")))
      if adjacent is None:
        raise SimulatorConnectionError("Selected cut-in route lost its adjacent driving lane")
      adjacent_waypoint, side = adjacent
      actor = self._spawn_vehicle(adjacent_waypoint, "scenario_intruder")
      self._cut_in_side = side
      self._set_desired_speed(actor, float(params.get("intruder_speed_mps", 8.33)))
      self._scenario_actor = actor

    elif kind == "signalized_intersection":
      if self.selected_light is None:
        raise SimulatorConnectionError("Signalized scenario did not receive a traffic light")
      self._light_was_controlled = True
      self._signal_group = list(self.selected_light.get_group_traffic_lights())
      for light in self._signal_group:
        light.set_state(carla.TrafficLightState.Red)

    elif kind == "parked_vehicle_obstruction":
      waypoint = self._waypoint_ahead(float(params.get("obstacle_distance_m", 45.0)))
      actor = self._spawn_vehicle(
        waypoint,
        "scenario_parked",
        lateral_offset_m=float(params.get("obstacle_lateral_offset_m", 1.1)),
      )
      actor.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
      self._scenario_actor = actor

    if self._scenario_actor is not None:
      self._scenario_actor_initial_location = self._scenario_actor.get_location()
    self._spawn_background_traffic(self.run.condition.traffic_vehicles)

  def _spawn_background_traffic(self, requested: int) -> None:
    if requested <= 0:
      return
    import carla

    spawn_points = list(self.world.get_map().get_spawn_points())
    self.rng.shuffle(spawn_points)
    commands: list[Any] = []
    ego_location = self.ego.get_location()
    for transform in spawn_points:
      if len(commands) >= requested:
        break
      if _distance(transform.location, ego_location) < 70.0:
        continue
      if any(
        _distance(transform.location, actor.get_location()) < 15.0 for actor in self.scripted_actors
      ):
        continue
      blueprint = self._vehicle_blueprint("background")
      command = carla.command.SpawnActor(blueprint, transform).then(
        carla.command.SetAutopilot(carla.command.FutureActor, True, self.traffic_manager_port)
      )
      commands.append(command)
    responses = self.client.apply_batch_sync(commands, False) if commands else []
    for response in responses:
      if not response.error:
        actor = self.world.get_actor(response.actor_id)
        if actor is not None:
          self._apply_tire_friction(actor)
          self.background_actors.append(actor)
    if len(self.background_actors) < requested:
      self.recorder.record_event(
        EventRecord(
          "background_spawn_shortfall",
          0.0,
          details={"requested": requested, "spawned": len(self.background_actors)},
        )
      )

  def _fire_trigger(self, evaluation_time_s: float, details: dict[str, Any]) -> None:
    if self.triggered:
      return
    self.triggered = True
    self.trigger_time_s = evaluation_time_s
    self.recorder.record_event(EventRecord("scenario_trigger", evaluation_time_s, details=details))

  def update(self, evaluation_time_s: float) -> None:
    import carla

    actor = self._scenario_actor
    params = self.run.scenario.parameters
    kind = self.run.scenario.kind

    if kind == "lead_vehicle_braking" and actor is not None:
      gap = max(0.0, _longitudinal_distance(self.ego, actor) - 4.5)
      ego_speed = self.ego.get_velocity()
      lead_speed = actor.get_velocity()
      ego_scalar = math.hypot(ego_speed.x, ego_speed.y)
      lead_scalar = math.hypot(lead_speed.x, lead_speed.y)
      closing = max(0.0, ego_scalar - lead_scalar)
      ttc = gap / closing if closing > 0.05 else math.inf
      trigger_min = float(params.get("trigger_min_time_s", 2.0))
      fallback = float(params.get("fallback_trigger_s", 7.0))
      if (
        not self.triggered
        and evaluation_time_s >= trigger_min
        and (ttc <= float(params.get("trigger_ttc_s", 2.0)) or evaluation_time_s >= fallback)
      ):
        trigger_reason = "ttc" if ttc <= float(params.get("trigger_ttc_s", 2.0)) else "fallback"
        self._fire_trigger(
          evaluation_time_s,
          {
            "gap_m": gap,
            "ttc_s": None if math.isinf(ttc) else ttc,
            "trigger_reason": trigger_reason,
            "brake_command": params.get("lead_brake_command", "maximum"),
          },
        )
        actor.set_autopilot(False, self.traffic_manager_port)
      if self.triggered and not self._lead_resumed:
        if self._lead_previous_speed_mps is not None and self._lead_previous_time_s is not None:
          dt = evaluation_time_s - self._lead_previous_time_s
          if dt > 0:
            observed = (self._lead_previous_speed_mps - lead_scalar) / dt
            self._lead_peak_deceleration_mps2 = max(self._lead_peak_deceleration_mps2, observed)
        self._lead_previous_speed_mps = lead_scalar
        self._lead_previous_time_s = evaluation_time_s
        actor.apply_control(carla.VehicleControl(brake=1.0))
        dwell = float(params.get("lead_stop_dwell_s", 5.0))
        if actor.get_velocity().length() < 0.2 and self._lead_stop_time_s is None:
          self._lead_stop_time_s = evaluation_time_s
        if (
          self._lead_stop_time_s is not None and evaluation_time_s - self._lead_stop_time_s >= dwell
        ):
          self._lead_resumed = True
          self._set_desired_speed(
            actor,
            float(params.get("lead_speed_mps", 11.11)),
            initialize_velocity=False,
          )

    elif kind == "pedestrian_crossing" and actor is not None:
      longitudinal = _longitudinal_distance(self.ego, actor)
      ego_velocity = self.ego.get_velocity().length()
      ttc = longitudinal / ego_velocity if ego_velocity > 0.1 else math.inf
      trigger_min = float(params.get("trigger_min_time_s", 2.0))
      fallback = float(params.get("fallback_trigger_s", 8.0))
      if (
        not self.triggered
        and evaluation_time_s >= trigger_min
        and (ttc <= float(params.get("trigger_ttc_s", 2.5)) or evaluation_time_s >= fallback)
      ):
        self._fire_trigger(
          evaluation_time_s,
          {
            "ttc_s": None if math.isinf(ttc) else ttc,
            "trigger_reason": "ttc"
            if ttc <= float(params.get("trigger_ttc_s", 2.5))
            else "fallback",
          },
        )
      if self.triggered:
        actor.apply_control(
          carla.WalkerControl(
            direction=self._crossing_direction,
            speed=float(params.get("pedestrian_speed_mps", 1.4)),
          )
        )

    elif kind == "vehicle_cut_in" and actor is not None:
      gap = _longitudinal_distance(self.ego, actor)
      ego_transform = self.ego.get_transform()
      forward = ego_transform.get_forward_vector()
      delta = actor.get_location() - ego_transform.location
      lateral = abs(delta.x * -forward.y + delta.y * forward.x)
      if self.triggered:
        self._cut_in_min_lateral_separation_m = min(self._cut_in_min_lateral_separation_m, lateral)
      ego_speed = self.ego.get_velocity().length()
      intruder_speed = actor.get_velocity().length()
      closing = max(0.0, ego_speed - intruder_speed)
      ttc = max(0.0, gap - 4.5) / closing if closing > 0.05 else math.inf
      trigger_min = float(params.get("trigger_min_time_s", 2.0))
      fallback = float(params.get("fallback_trigger_s", 10.0))
      if (
        not self.triggered
        and evaluation_time_s >= trigger_min
        and (ttc <= float(params.get("trigger_ttc_s", 2.0)) or evaluation_time_s >= fallback)
      ):
        self._fire_trigger(
          evaluation_time_s,
          {
            "gap_m": gap,
            "ttc_s": None if math.isinf(ttc) else ttc,
            "side": self._cut_in_side,
            "trigger_reason": "ttc"
            if ttc <= float(params.get("trigger_ttc_s", 2.0))
            else "fallback",
          },
        )
        # CARLA Traffic Manager uses True for right and False for left.
        move_right = self._cut_in_side == "left"
        self.traffic_manager.force_lane_change(actor, move_right)

    elif kind == "signalized_intersection" and self.selected_light is not None:
      if not self.triggered:
        self._fire_trigger(
          evaluation_time_s,
          {"traffic_light_id": self.selected_light.id, "state": "Red"},
        )
      if evaluation_time_s < float(params.get("red_hold_s", 20.0)):
        for light in self._signal_group:
          light.set_state(carla.TrafficLightState.Red)
      else:
        self.selected_light.set_state(carla.TrafficLightState.Green)

  def hazard_metrics(self) -> dict[str, Any] | None:
    ego_transform = self.ego.get_transform()
    forward = ego_transform.get_forward_vector()
    ego_velocity = self.ego.get_velocity()
    ego_forward_speed = (
      ego_velocity.x * forward.x + ego_velocity.y * forward.y + ego_velocity.z * forward.z
    )
    best: dict[str, Any] | None = None
    for actor in self.scripted_actors + self.background_actors:
      if not actor.is_alive:
        continue
      delta = actor.get_location() - ego_transform.location
      longitudinal = delta.x * forward.x + delta.y * forward.y + delta.z * forward.z
      if longitudinal < -2.0:
        continue
      lateral = abs(delta.x * -forward.y + delta.y * forward.x)
      if lateral > 6.0:
        continue
      gap = max(0.0, math.hypot(delta.x, delta.y) - 4.0)
      velocity = actor.get_velocity()
      actor_forward_speed = velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z
      closing = max(0.0, ego_forward_speed - actor_forward_speed)
      ttc = gap / closing if closing > 0.05 else None
      actor_location = actor.get_location()
      candidate = {
        "distance_m": gap,
        "closing_speed_mps": closing,
        "ttc_s": ttc,
        "actor_id": actor.id,
        "actor_type": actor.type_id,
        "actor_speed_mps": velocity.length(),
        "actor_x_m": actor_location.x,
        "actor_y_m": actor_location.y,
      }
      if best is None or candidate["distance_m"] < best["distance_m"]:
        best = candidate
    return best

  def traffic_light_state(self) -> str | None:
    return str(self.selected_light.get_state()) if self.selected_light is not None else None

  def realization_status(self, scenario_actor_collision: bool = False) -> dict[str, Any]:
    """Return deterministic acceptance checks for the scripted trial realization."""

    kind = self.run.scenario.kind
    params = self.run.scenario.parameters
    reasons: list[str] = []
    measured: dict[str, Any] = {
      "triggered": self.triggered,
      "background_requested": self._background_requested,
      "background_spawned": len(self.background_actors),
    }
    if len(self.background_actors) != self._background_requested:
      reasons.append("background_spawn_shortfall")

    triggered_kinds = {
      "lead_vehicle_braking",
      "pedestrian_crossing",
      "vehicle_cut_in",
      "signalized_intersection",
    }
    if kind in triggered_kinds and not self.triggered:
      reasons.append("scenario_trigger_missing")

    if kind == "lead_vehicle_braking":
      minimum = float(params.get("min_observed_lead_deceleration_mps2", 3.0))
      measured["peak_lead_deceleration_mps2"] = self._lead_peak_deceleration_mps2
      measured["minimum_required_deceleration_mps2"] = minimum
      if not scenario_actor_collision and self._lead_peak_deceleration_mps2 < minimum:
        reasons.append("lead_deceleration_below_minimum")
    elif kind == "pedestrian_crossing" and self._scenario_actor is not None:
      distance = (
        _distance(self._scenario_actor.get_location(), self._scenario_actor_initial_location)
        if self._scenario_actor_initial_location is not None
        else 0.0
      )
      minimum = float(params.get("min_pedestrian_travel_m", 2.0))
      measured["pedestrian_travel_m"] = distance
      measured["minimum_required_travel_m"] = minimum
      if not scenario_actor_collision and distance < minimum:
        reasons.append("pedestrian_travel_below_minimum")
    elif kind == "vehicle_cut_in":
      maximum = float(params.get("max_cut_in_lateral_separation_m", 1.75))
      measured["minimum_lateral_separation_m"] = (
        None
        if math.isinf(self._cut_in_min_lateral_separation_m)
        else self._cut_in_min_lateral_separation_m
      )
      measured["maximum_accepted_lateral_separation_m"] = maximum
      if not scenario_actor_collision and self._cut_in_min_lateral_separation_m > maximum:
        reasons.append("cut_in_merge_not_realized")

    return {"valid": not reasons, "reasons": reasons, "measured": measured}

  def check_signal_compliance(self, evaluation_time_s: float, frame: int) -> None:
    if self.selected_light is None or self._signal_violation:
      return
    stop_waypoints = list(self.selected_light.get_stop_waypoints())
    if not stop_waypoints:
      return
    waypoint = min(
      stop_waypoints,
      key=lambda item: _distance(item.transform.location, self.start_waypoint.transform.location),
    )
    trigger_location = waypoint.transform.location
    forward = waypoint.transform.get_forward_vector()
    delta = self.ego.get_location() - trigger_location
    side = delta.x * forward.x + delta.y * forward.y + delta.z * forward.z
    if (
      self._signal_previous_side is not None
      and self._signal_previous_side < 0 <= side
      and str(self.selected_light.get_state()).endswith("Red")
    ):
      self._signal_violation = True
      self.recorder.record_event(
        EventRecord(
          "red_light_violation",
          evaluation_time_s,
          frame,
          {"traffic_light_id": self.selected_light.id},
        )
      )
    self._signal_previous_side = side

  def close(self) -> None:
    if self.selected_light is not None and self._light_was_controlled:
      with suppress(RuntimeError):
        self.selected_light.reset_group()
    actors = [actor for actor in self.scripted_actors + self.background_actors if actor is not None]
    # Destroy through each proxy so CARLA marks it dead locally. Batch-destroying
    # by ID leaves live Python proxies behind; their later destructors can try to
    # destroy the same server actor again and terminate the native client.
    for actor in reversed(actors):
      with suppress(RuntimeError):
        actor.destroy()
    self.scripted_actors.clear()
    self.background_actors.clear()
