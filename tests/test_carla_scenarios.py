from types import SimpleNamespace

import pytest

from opencarla_eval.carla_scenarios import select_spawn
from opencarla_eval.errors import SimulatorConnectionError


class _Waypoint:
  def __init__(self, index: int, *, junction: bool = False) -> None:
    self.is_junction = junction
    self.lane_type = "LaneType.Driving"
    self.transform = SimpleNamespace(
      location=SimpleNamespace(x=float(index * 5), y=0.0, z=0.0),
      rotation=SimpleNamespace(yaw=0.0),
    )
    self._next = None

  def next(self, _distance_m: float) -> list["_Waypoint"]:
    return [] if self._next is None else [self._next]


def _world_with_first_junction_at(distance_m: float) -> SimpleNamespace:
  waypoints = [_Waypoint(i, junction=i * 5 == distance_m) for i in range(9)]
  for current, following in zip(waypoints, waypoints[1:], strict=False):
    current._next = following
  spawn = SimpleNamespace(location=SimpleNamespace(x=0.0, y=0.0, z=0.0))
  world_map = SimpleNamespace(
    name="TestTown",
    get_spawn_points=lambda: [spawn],
    get_waypoint=lambda _location, project_to_road=True: waypoints[0],
  )
  return SimpleNamespace(get_map=lambda: world_map)


def _run(clearance_m: float) -> SimpleNamespace:
  return SimpleNamespace(
    seed=41000,
    scenario=SimpleNamespace(
      id="lane_following",
      kind="lane_following",
      route_length_m=30.0,
      parameters={
        "require_heading_change_deg": 0.0,
        "minimum_junction_clearance_m": clearance_m,
      },
    ),
  )


def test_spawn_selection_rejects_a_junction_inside_startup_clearance() -> None:
  world = _world_with_first_junction_at(10.0)

  with pytest.raises(SimulatorConnectionError, match="No topology-compatible spawn"):
    select_spawn(world, _run(15.0))


def test_spawn_selection_accepts_a_junction_beyond_startup_clearance() -> None:
  world = _world_with_first_junction_at(10.0)

  selection = select_spawn(world, _run(5.0))

  assert selection.waypoint.transform.location.x == 0.0
