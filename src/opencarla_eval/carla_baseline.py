"""Privileged-state CARLA Traffic Manager reference controller."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from contextlib import suppress
from pathlib import Path
from typing import Any

from .attempts import archive_existing_attempt
from .carla_world import CarlaEvaluationWorld, load_summary
from .config import expand_runs, load_experiment
from .errors import DependencyUnavailableError, EvaluationError, SimulatorConnectionError
from .models import RunSpec
from .provenance import require_reusable_summary
from .run_failure import mark_existing_run_invalid, write_invalid_run
from .runtime_config import load_runtime_config


def _restore_async(client: Any, traffic_manager_port: int) -> None:
  try:
    traffic_manager = client.get_trafficmanager(traffic_manager_port)
    traffic_manager.set_synchronous_mode(False)
    traffic_manager.shut_down()
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
  except Exception:
    pass


def execute_run(
  run: RunSpec,
  runtime_config: str | Path,
  overwrite: bool = False,
) -> dict[str, Any]:
  summary_path = run.output_dir / "summary.json"
  config = load_runtime_config(runtime_config)
  if summary_path.exists() and not overwrite:
    summary = load_summary(summary_path)
    require_reusable_summary(summary, run, backend="carla", runtime_config=config)
    return summary
  try:
    import carla
  except ImportError as exc:
    raise DependencyUnavailableError("Install carla==0.9.16 to run the reference baseline") from exc
  if overwrite:
    archive_existing_attempt(run.output_dir)
  client = carla.Client(config.host, config.port)
  client.set_timeout(config.timeout_s)
  evaluation_world: CarlaEvaluationWorld | None = None
  try:
    evaluation_world = CarlaEvaluationWorld(
      client,
      run,
      config,
      dual_camera=False,
      high_quality=True,
      enable_cameras=False,
      controller_name="traffic_manager",
    )
    vehicle = evaluation_world.vehicle
    # Match the pinned SimulatorBridge's 20 empty world ticks before either
    # controller begins its active warmup.
    for _ in range(20):
      evaluation_world.tick()
    vehicle.set_autopilot(True, config.traffic_manager_port)
    manager = evaluation_world.traffic_manager
    manager.auto_lane_change(vehicle, False)
    if hasattr(manager, "set_desired_speed"):
      manager.set_desired_speed(vehicle, run.scenario.target_speed_mps * 3.6)
    else:  # pragma: no cover - old CARLA fallback
      limit = max(vehicle.get_speed_limit(), 1.0)
      difference = 100.0 * (1.0 - run.scenario.target_speed_mps * 3.6 / limit)
      manager.vehicle_percentage_speed_difference(vehicle, difference)
    if hasattr(manager, "set_path"):
      manager.set_path(
        vehicle,
        [waypoint.transform.location for waypoint in evaluation_world.reference_route[1:]],
      )
    evaluation_world.set_controller_active(True)
    while not evaluation_world.exit_event.is_set():
      evaluation_world.tick()
      evaluation_world.set_controller_active(True)
    evaluation_world.close("baseline_complete")
    return load_summary(summary_path)
  except BaseException as exc:
    if evaluation_world is not None:
      evaluation_world._termination_reason = "baseline_runtime_error"
      with suppress(Exception):
        evaluation_world.close("exception")
      mark_existing_run_invalid(run, "baseline_runtime_error", exc)
    else:
      _restore_async(client, config.traffic_manager_port)
      write_invalid_run(run, "carla", "baseline_setup_failed", exc)
    if isinstance(exc, (KeyboardInterrupt, SystemExit)):
      raise
    if isinstance(exc, (EvaluationError, OSError, RuntimeError)):
      raise
    raise SimulatorConnectionError(str(exc)) from exc


def _select_run(experiment_path: Path, run_id: str) -> RunSpec:
  experiment = load_experiment(experiment_path)
  matches = [run for run in expand_runs(experiment) if run.run_id == run_id]
  if not matches:
    raise EvaluationError(f"Run id {run_id!r} is not present in {experiment_path}")
  run = matches[0]
  if run.controller != "traffic_manager":
    raise EvaluationError(f"Run {run_id!r} uses controller {run.controller!r}, not traffic_manager")
  return run


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--experiment", type=Path, required=True)
  parser.add_argument("--run-id", required=True)
  parser.add_argument(
    "--runtime-config",
    type=Path,
    default=Path(__file__).resolve().parents[2] / "config" / "carla.yaml",
  )
  parser.add_argument("--overwrite", action="store_true")
  return parser


def main(argv: Sequence[str] | None = None) -> int:
  args = build_parser().parse_args(argv)
  try:
    run = _select_run(args.experiment, args.run_id)
    summary = execute_run(run, args.runtime_config, args.overwrite)
    print(json.dumps({"run_id": run.run_id, "success": summary["metrics"]["success"]}, indent=2))
    return 0
  except (EvaluationError, OSError, ValueError) as exc:
    print(f"error: {exc}", file=sys.stderr)
    return 2


if __name__ == "__main__":  # pragma: no cover
  raise SystemExit(main())
