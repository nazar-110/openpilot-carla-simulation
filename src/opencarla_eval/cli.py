"""Command-line interface for validation, execution, and analysis."""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
import platform
import random
import subprocess
import sys
from collections.abc import Sequence
from dataclasses import replace
from importlib import import_module
from pathlib import Path
from typing import Any

from . import __version__
from .analysis import analyze_results
from .config import expand_runs, load_experiment
from .errors import EvaluationError
from .recorder import atomic_write_json
from .runtime_config import load_runtime_config
from .synthetic import run_synthetic

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EXPERIMENT = PROJECT_ROOT / "experiments" / "smoke.yaml"
DEFAULT_RUNTIME = PROJECT_ROOT / "config" / "carla.yaml"


def _add_experiment_argument(parser: argparse.ArgumentParser) -> None:
  parser.add_argument(
    "--experiment",
    type=Path,
    default=DEFAULT_EXPERIMENT,
    help=f"Experiment YAML (default: {DEFAULT_EXPERIMENT})",
  )


def _cmd_validate(args: argparse.Namespace) -> int:
  experiment = load_experiment(args.experiment)
  runtime = load_runtime_config(args.runtime_config)
  payload = {
    "experiment": experiment.name,
    "description": experiment.description,
    "scenario_count": len(experiment.scenario_paths),
    "controllers": experiment.controllers,
    "conditions": [condition.id for condition in experiment.conditions],
    "repetitions": experiment.repetitions,
    "run_count": experiment.run_count,
    "output_dir": str(experiment.output_dir),
    "carla_version": runtime.required_version,
    "fixed_delta_seconds": runtime.fixed_delta_seconds,
  }
  print(json.dumps(payload, indent=2))
  return 0


def _cmd_matrix(args: argparse.Namespace) -> int:
  experiment = load_experiment(args.experiment)
  runs = list(expand_runs(experiment))
  if args.controller:
    runs = [run for run in runs if run.controller == args.controller]
  if args.limit is not None:
    runs = runs[: args.limit]
  if args.json:
    print(
      json.dumps(
        [
          {
            "run_id": run.run_id,
            "scenario": run.scenario.id,
            "condition": run.condition.id,
            "controller": run.controller,
            "seed": run.seed,
            "status": "complete" if (run.output_dir / "summary.json").exists() else "pending",
            "output_dir": str(run.output_dir),
          }
          for run in runs
        ],
        indent=2,
      )
    )
  else:
    for run in runs:
      marker = "done" if (run.output_dir / "summary.json").exists() else "pending"
      print(f"{marker:7}  {run.run_id}")
    print(f"\n{len(runs)} run(s) shown; full matrix has {experiment.run_count} run(s).")
  return 0


def _run_real(run: Any, runtime_path: Path, overwrite: bool) -> dict[str, Any]:
  if run.controller == "openpilot":
    from .openpilot_bridge import execute_run

    return execute_run(run, runtime_path, overwrite=overwrite)
  if run.controller == "traffic_manager":
    from .carla_baseline import execute_run

    return execute_run(run, runtime_path, overwrite=overwrite)
  if run.controller == "synthetic":
    return run_synthetic(run, overwrite=overwrite)
  raise EvaluationError(f"No CARLA runner for controller {run.controller!r}")


def _cmd_run(args: argparse.Namespace) -> int:
  experiment = load_experiment(args.experiment)
  runs = list(expand_runs(experiment))
  if args.controller:
    runs = [run for run in runs if run.controller == args.controller]
  if args.randomize_order:
    # Keep paired controllers adjacent while randomizing scenario-condition-seed blocks.
    blocks: dict[tuple[str, str, int, int], list[Any]] = {}
    for run in runs:
      key = (run.scenario.id, run.condition.id, run.repetition, run.seed)
      blocks.setdefault(key, []).append(run)
    keys = list(blocks)
    random.Random(args.order_seed).shuffle(keys)
    runs = [run for key in keys for run in blocks[key]]
  if args.limit is not None:
    runs = runs[: args.limit]
  if not runs:
    raise EvaluationError("No runs matched the supplied filters")
  order_seed = args.order_seed if args.randomize_order else None
  runs = [
    replace(run, execution_index=index, order_seed=order_seed)
    for index, run in enumerate(runs, start=1)
  ]
  toolkit_commit = _git_commit(PROJECT_ROOT)
  toolkit_dirty = _git_dirty(PROJECT_ROOT)
  first_metadata = runs[0].metadata(backend=args.backend, valid_for_research=False)
  atomic_write_json(
    experiment.output_dir / "collection_manifest.json",
    {
      "schema_version": 1,
      "experiment": experiment.name,
      "experiment_sha256": first_metadata.get("experiment_sha256"),
      "runtime_config": str(args.runtime_config.expanduser().resolve()),
      "backend": args.backend,
      "controller_filter": args.controller,
      "randomized_blocks": args.randomize_order,
      "order_seed": order_seed,
      "toolkit_commit": toolkit_commit,
      "toolkit_worktree_dirty": toolkit_dirty,
      "python": platform.python_version(),
      "platform": platform.platform(),
      "selected_run_ids": [run.run_id for run in runs],
    },
  )

  completed = skipped = failed = 0
  for index, run in enumerate(runs, start=1):
    summary_path = run.output_dir / "summary.json"
    reuse_candidate = summary_path.exists() and not args.overwrite
    action = "check" if reuse_candidate else "run  "
    print(f"[{index}/{len(runs)}] {action} {run.run_id}", flush=True)
    try:
      if args.backend == "synthetic":
        if run.controller != "synthetic":
          print(
            "  warning: controller label is preserved, but the backend is synthetic and "
            "the result will not be research-valid",
            file=sys.stderr,
          )
        run_synthetic(run, overwrite=args.overwrite)
      else:
        _run_real(run, args.runtime_config, args.overwrite)
      if reuse_candidate:
        skipped += 1
        print("  provenance matched; keeping completed artifact")
      else:
        completed += 1
    except (EvaluationError, RuntimeError, OSError) as exc:
      failed += 1
      print(f"  failed: {exc}", file=sys.stderr)
      if not args.keep_going:
        raise
  print(f"Completed {completed}; skipped {skipped}; failed {failed}.")
  return 1 if failed else 0


def _cmd_analyze(args: argparse.Namespace) -> int:
  manifest = analyze_results(args.results, args.output)
  print(json.dumps(manifest, indent=2))
  return 0


def _git_commit(path: Path) -> str | None:
  try:
    result = subprocess.run(
      ["git", "rev-parse", "HEAD"],
      cwd=path,
      check=True,
      capture_output=True,
      text=True,
      timeout=30,
    )
  except (OSError, subprocess.SubprocessError):
    return None
  return result.stdout.strip()


def _git_dirty(path: Path) -> bool | None:
  try:
    result = subprocess.run(
      ["git", "status", "--porcelain"],
      cwd=path,
      check=True,
      capture_output=True,
      text=True,
      timeout=30,
    )
  except (OSError, subprocess.SubprocessError):
    return None
  return bool(result.stdout.strip())


def _cmd_doctor(args: argparse.Namespace) -> int:
  runtime = load_runtime_config(args.runtime_config)
  checks: dict[str, Any] = {
    "toolkit_version": __version__,
    "python": platform.python_version(),
    "python_supported": (3, 12, 3) <= sys.version_info[:3] < (3, 13, 0),
    "platform": platform.platform(),
    "wsl": "microsoft" in platform.release().lower() or "WSL_DISTRO_NAME" in os.environ,
    "runtime_config": str(runtime.source_path),
    "carla_target": runtime.required_version,
    "carla_host": runtime.host,
    "carla_port": runtime.port,
    "toolkit_root": str(PROJECT_ROOT),
    "toolkit_commit": _git_commit(PROJECT_ROOT),
    "toolkit_worktree_dirty": _git_dirty(PROJECT_ROOT),
  }
  carla_module: Any | None = None
  try:
    import carla

    carla_module = carla
    try:
      checks["carla_python_api"] = importlib.metadata.version("carla")
    except importlib.metadata.PackageNotFoundError:
      checks["carla_python_api"] = "imported (package version unavailable)"
    checks["carla_import"] = True
  except Exception as exc:  # diagnostic command deliberately reports all failures
    checks["carla_import"] = False
    checks["carla_import_error"] = f"{type(exc).__name__}: {exc}"

  if args.connect and carla_module is not None:
    try:
      client = carla_module.Client(runtime.host, runtime.port)
      client.set_timeout(runtime.timeout_s)
      checks["carla_client_version"] = client.get_client_version()
      checks["carla_server_version"] = client.get_server_version()
      checks["carla_version_match"] = (
        client.get_client_version() == client.get_server_version() == runtime.required_version
      )
    except Exception as exc:  # retain the successful import diagnostic
      checks["carla_connect"] = False
      checks["carla_connect_error"] = f"{type(exc).__name__}: {exc}"

  if args.openpilot_root:
    root = args.openpilot_root.expanduser().resolve()
    checks["openpilot_root"] = str(root)
    checks["openpilot_exists"] = (root / "tools" / "sim" / "launch_openpilot.sh").is_file()
    checks["openpilot_commit"] = _git_commit(root)
    checks["openpilot_worktree_dirty"] = _git_dirty(root)
    checks["openpilot_v0111_expected"] = "4df40d2c1946a57242230186edd073c4073060a6"
    checks["openpilot_commit_match"] = (
      checks["openpilot_commit"] == checks["openpilot_v0111_expected"]
    )
    try:
      if str(root) not in sys.path:
        sys.path.insert(0, str(root))
      import_module("openpilot.tools.sim.bridge.common")
      checks["openpilot_sim_import"] = True
      from .openpilot_bridge import _running_openpilot_managers

      checks["openpilot_manager_pids"] = _running_openpilot_managers(root)
      checks["openpilot_manager_not_running"] = not checks["openpilot_manager_pids"]
    except Exception as exc:
      checks["openpilot_sim_import"] = False
      checks["openpilot_sim_import_error"] = f"{type(exc).__name__}: {exc}"
      checks["openpilot_manager_not_running"] = False
  print(json.dumps(checks, indent=2))
  required = [
    checks["python_supported"],
    checks["carla_import"],
    isinstance(checks["toolkit_commit"], str),
    checks["toolkit_worktree_dirty"] is False,
  ]
  if args.connect:
    required.append(bool(checks.get("carla_version_match", False)))
  if args.openpilot_root:
    required.extend(
      [
        checks["openpilot_exists"],
        checks["openpilot_commit_match"],
        checks["openpilot_worktree_dirty"] is False,
        checks["openpilot_sim_import"],
        checks["openpilot_manager_not_running"],
      ]
    )
  return 0 if all(required) else 1


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
    prog="opencarla-eval",
    description="Evaluate OpenPilot in deterministic CARLA urban scenarios.",
  )
  parser.add_argument("--version", action="version", version=__version__)
  subparsers = parser.add_subparsers(dest="command", required=True)

  validate = subparsers.add_parser("validate", help="Validate experiment and runtime YAML")
  _add_experiment_argument(validate)
  validate.add_argument("--runtime-config", type=Path, default=DEFAULT_RUNTIME)
  validate.set_defaults(handler=_cmd_validate)

  matrix = subparsers.add_parser("matrix", help="List the expanded experiment matrix")
  _add_experiment_argument(matrix)
  matrix.add_argument("--controller")
  matrix.add_argument("--limit", type=int)
  matrix.add_argument("--json", action="store_true")
  matrix.set_defaults(handler=_cmd_matrix)

  run = subparsers.add_parser("run", help="Run a synthetic smoke or real CARLA matrix")
  _add_experiment_argument(run)
  run.add_argument("--backend", choices=("synthetic", "carla"), default="synthetic")
  run.add_argument("--runtime-config", type=Path, default=DEFAULT_RUNTIME)
  run.add_argument("--controller")
  run.add_argument("--limit", type=int)
  run.add_argument("--overwrite", action="store_true")
  run.add_argument("--keep-going", action="store_true")
  run.add_argument(
    "--randomize-order",
    action="store_true",
    help="Deterministically randomize paired scenario-condition-seed blocks",
  )
  run.add_argument("--order-seed", type=int, default=20260714)
  run.set_defaults(handler=_cmd_run)

  analyze = subparsers.add_parser("analyze", help="Aggregate summary.json artifacts")
  analyze.add_argument("results", type=Path)
  analyze.add_argument("--output", type=Path, required=True)
  analyze.set_defaults(handler=_cmd_analyze)

  doctor = subparsers.add_parser("doctor", help="Check the local simulator environment")
  doctor.add_argument("--runtime-config", type=Path, default=DEFAULT_RUNTIME)
  doctor.add_argument("--openpilot-root", type=Path)
  doctor.add_argument("--connect", action="store_true", help="Connect to the CARLA server")
  doctor.set_defaults(handler=_cmd_doctor)
  return parser


def main(argv: Sequence[str] | None = None) -> int:
  parser = build_parser()
  args = parser.parse_args(argv)
  try:
    return int(args.handler(args))
  except KeyboardInterrupt:
    print("Interrupted.", file=sys.stderr)
    return 130
  except (EvaluationError, FileNotFoundError, ValueError) as exc:
    print(f"error: {exc}", file=sys.stderr)
    return 2


if __name__ == "__main__":  # pragma: no cover
  raise SystemExit(main())
