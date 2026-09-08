"""Provenance gates for safely reusing completed run artifacts."""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any

from .errors import StaleResultError
from .models import RunSpec
from .recorder import EXPECTED_EVALUATION_TERMINATIONS
from .runtime_config import CarlaRuntimeConfig

PROJECT_ROOT = Path(__file__).resolve().parents[2]
_EXPLORATORY_INVALID_REASONS = {
  "untested_or_dirty_openpilot",
  "dirty_or_unversioned_toolkit",
}


def _file_sha256(path: Path) -> str | None:
  return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def _git_state(path: Path) -> tuple[str | None, bool | None]:
  try:
    commit = subprocess.run(
      ["git", "rev-parse", "HEAD"],
      cwd=path,
      check=True,
      capture_output=True,
      text=True,
      timeout=30,
    ).stdout.strip()
    dirty = bool(
      subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=path,
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
      ).stdout.strip()
    )
    return commit, dirty
  except (OSError, subprocess.SubprocessError):
    return None, None


def _runtime_mapping(config: CarlaRuntimeConfig) -> dict[str, Any]:
  """Mirror the effective runtime configuration persisted by the CARLA adapter."""

  return {
    "source_path": str(config.source_path),
    "effective_host": config.host,
    "effective_port": config.port,
    "timeout_s": config.timeout_s,
    "fixed_delta_seconds": config.fixed_delta_seconds,
    "traffic_manager_port": config.traffic_manager_port,
    "substepping": config.substepping,
    "max_substep_delta_time": config.max_substep_delta_time,
    "max_substeps": config.max_substeps,
    "quality_level": config.quality_level,
    "ego_blueprint": config.ego_blueprint,
    "role_name": config.role_name,
    "camera": {
      "width": config.camera.width,
      "height": config.camera.height,
      "road_fov_deg": config.camera.road_fov_deg,
      "wide_fov_deg": config.camera.wide_fov_deg,
      "sensor_tick_s": config.camera.sensor_tick_s,
      "gamma": config.camera.gamma,
      "mount": config.camera.mount,
    },
    "actuation": {
      "steer_sign": config.actuation.steer_sign,
      "steering_ratio": config.actuation.steering_ratio,
      "fallback_max_wheel_angle_deg": config.actuation.fallback_max_wheel_angle_deg,
    },
    "recording": {
      "carla_recorder": config.recording.carla_recorder,
      "server_integrity_recorder": True,
    },
    "spectator": {
      "enabled": config.spectator.enabled,
      "distance_m": config.spectator.distance_m,
      "height_m": config.spectator.height_m,
      "pitch_deg": config.spectator.pitch_deg,
      "smoothing_time_s": config.spectator.smoothing_time_s,
    },
    "dashboard": {
      "enabled": config.dashboard.enabled,
      "host": config.dashboard.host,
      "port": config.dashboard.port,
      "frame_stride": config.dashboard.frame_stride,
      "downsample": config.dashboard.downsample,
      "jpeg_quality": config.dashboard.jpeg_quality,
    },
  }


def _runtime_effective_sha256(config: CarlaRuntimeConfig) -> str:
  return hashlib.sha256(
    json.dumps(_runtime_mapping(config), sort_keys=True, separators=(",", ":")).encode()
  ).hexdigest()


def reusable_summary_mismatches(
  summary: Any,
  run: RunSpec,
  *,
  backend: str,
  runtime_config: CarlaRuntimeConfig | None = None,
  openpilot_commit: str | None = None,
  openpilot_dirty: bool | None = None,
) -> list[str]:
  """Return fields that prevent a completed artifact from being safely reused."""

  if not isinstance(summary, dict):
    return ["summary_root"]

  metadata = run.metadata(backend=backend, valid_for_research=False)
  expected: dict[str, Any] = {
    "schema_version": 1,
    "run_id": run.run_id,
    "experiment": run.experiment_name,
    "scenario_id": run.scenario.id,
    "condition_id": run.condition.id,
    "controller": run.controller,
    "backend": backend,
    "repetition": run.repetition,
    "seed": run.seed,
    "scenario_sha256": metadata.get("scenario_sha256"),
    "condition_sha256": metadata.get("condition_sha256"),
    "experiment_sha256": metadata.get("experiment_sha256"),
  }
  if runtime_config is not None:
    expected["runtime_config_sha256"] = _file_sha256(runtime_config.source_path)
    expected["runtime_effective_sha256"] = _runtime_effective_sha256(runtime_config)

  mismatches = [name for name, value in expected.items() if summary.get(name) != value]
  metrics = summary.get("metrics")
  if not isinstance(metrics, dict) or any(
    not isinstance(metrics.get(name), bool)
    for name in ("success", "intervention_free_success", "quality_pass")
  ):
    mismatches.append("artifact_incomplete_metrics")
  for filename in ("metadata.json", "telemetry.jsonl", "events.jsonl"):
    if not (run.output_dir / filename).is_file():
      mismatches.append(f"artifact_missing.{filename}")
  if backend == "carla":
    runtime = summary.get("runtime")
    rerun_requested = isinstance(runtime, dict) and runtime.get("rerun_same_seed") is True
    invalid_trial = (
      summary.get("invalid_trial") is True
      or summary.get("artifact_class") == "invalid_trial"
      or rerun_requested
    )
    invalid_termination = summary.get("termination_reason") not in EXPECTED_EVALUATION_TERMINATIONS
    unexplained_nonresearch = (
      summary.get("valid_for_research") is not True
      and summary.get("invalid_reason") not in _EXPLORATORY_INVALID_REASONS
    )
    if invalid_trial or invalid_termination or unexplained_nonresearch:
      mismatches.append("artifact_requires_same_seed_rerun")
    else:
      recorder_audit = (
        runtime.get("recorder_collision_audit") if isinstance(runtime, dict) else None
      )
      if not isinstance(recorder_audit, dict) or recorder_audit.get("valid") is not True:
        mismatches.append("runtime.recorder_collision_audit")
      if run.controller == "openpilot" and not (run.output_dir / "openpilot_manager.log").is_file():
        mismatches.append("artifact_missing.openpilot_manager.log")
  if runtime_config is not None:
    software = summary.get("software")
    if not isinstance(software, dict):
      mismatches.append("software")
    else:
      toolkit_commit, toolkit_dirty = _git_state(PROJECT_ROOT)
      software_expected = {
        "toolkit_commit": toolkit_commit,
        "toolkit_worktree_dirty": toolkit_dirty,
      }
      if run.controller == "openpilot":
        software_expected.update(
          {
            "openpilot_commit": openpilot_commit,
            "openpilot_worktree_dirty": openpilot_dirty,
          }
        )
      mismatches.extend(
        f"software.{name}"
        for name, value in software_expected.items()
        if name not in software or software.get(name) != value
      )
  return sorted(set(mismatches))


def require_reusable_summary(
  summary: Any,
  run: RunSpec,
  *,
  backend: str,
  runtime_config: CarlaRuntimeConfig | None = None,
  openpilot_commit: str | None = None,
  openpilot_dirty: bool | None = None,
) -> None:
  """Reject stale output and require an explicit, archival overwrite."""

  mismatches = reusable_summary_mismatches(
    summary,
    run,
    backend=backend,
    runtime_config=runtime_config,
    openpilot_commit=openpilot_commit,
    openpilot_dirty=openpilot_dirty,
  )
  if mismatches:
    fields = ", ".join(mismatches)
    raise StaleResultError(
      f"Existing result for {run.run_id!r} cannot be reused ({fields}). "
      "Rerun with --overwrite to archive it before collecting a replacement."
    )
