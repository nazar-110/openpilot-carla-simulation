"""Consistent invalid-trial artifacts for simulator/setup failures."""

from __future__ import annotations

import hashlib
from typing import Any

from .models import RunSpec
from .recorder import RunRecorder, atomic_write_json


def write_invalid_run(
  run: RunSpec,
  backend: str,
  reason: str,
  error: BaseException | None = None,
) -> dict[str, Any]:
  metadata = run.metadata(backend=backend, valid_for_research=False)
  metadata["invalid_trial"] = True
  metadata["artifact_class"] = "invalid_trial"
  metadata["invalid_reason"] = reason
  if run.scenario.source_path is not None and run.scenario.source_path.is_file():
    metadata["scenario_sha256"] = hashlib.sha256(run.scenario.source_path.read_bytes()).hexdigest()
  recorder = RunRecorder(run.output_dir, metadata, run.scenario.thresholds)
  details: dict[str, Any] = {"rerun_same_seed": True}
  if error is not None:
    details.update({"error_type": type(error).__name__, "error": str(error)})
  return recorder.finalize(reason, details)


def mark_existing_run_invalid(
  run: RunSpec,
  reason: str,
  error: BaseException | None = None,
) -> dict[str, Any]:
  """Mark an already finalized run invalid without destroying its raw telemetry."""

  import json

  summary_path = run.output_dir / "summary.json"
  if not summary_path.is_file():
    return write_invalid_run(run, "carla", reason, error)
  summary = json.loads(summary_path.read_text(encoding="utf-8"))
  summary["valid_for_research"] = False
  summary["invalid_trial"] = True
  summary["artifact_class"] = "invalid_trial"
  summary["invalid_reason"] = reason
  summary["termination_reason"] = reason
  summary.setdefault("runtime", {})["rerun_same_seed"] = True
  if error is not None:
    summary["runtime"].update({"error_type": type(error).__name__, "error": str(error)})
  if isinstance(summary.get("metrics"), dict):
    summary["metrics"]["success"] = False
    summary["metrics"]["intervention_free_success"] = False
    summary["metrics"]["quality_pass"] = False
  atomic_write_json(summary_path, summary)

  metadata_path = run.output_dir / "metadata.json"
  if metadata_path.is_file():
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["valid_for_research"] = False
    metadata["invalid_trial"] = True
    atomic_write_json(metadata_path, metadata)
  return summary
