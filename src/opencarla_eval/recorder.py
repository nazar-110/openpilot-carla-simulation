"""Thread-safe, crash-tolerant recording of raw run data and summaries."""

from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Any

from .metrics import compute_metrics
from .models import EventRecord, MetricThresholds, TelemetrySample

EXPECTED_EVALUATION_TERMINATIONS = {
  "route_completed",
  "timeout",
  "collision",
  "engagement_timeout",
  "controller_disengagement",
}


def atomic_write_json(path: Path, value: Any) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_suffix(path.suffix + ".tmp")
  temporary.write_text(
    json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8"
  )
  os.replace(temporary, path)


class RunRecorder:
  """Write JSONL during a run and an atomic summary when it finishes."""

  def __init__(
    self,
    output_dir: str | Path,
    metadata: dict[str, Any],
    thresholds: MetricThresholds,
  ) -> None:
    self.output_dir = Path(output_dir).resolve()
    self.output_dir.mkdir(parents=True, exist_ok=True)
    self.metadata = metadata
    self.thresholds = thresholds
    self.samples: list[TelemetrySample] = []
    self.events: list[EventRecord] = []
    self._lock = threading.RLock()
    self._closed = False
    self._telemetry_file = (self.output_dir / "telemetry.jsonl").open(
      "w", encoding="utf-8", buffering=1
    )
    self._events_file = (self.output_dir / "events.jsonl").open("w", encoding="utf-8", buffering=1)
    atomic_write_json(self.output_dir / "metadata.json", metadata)

  def record_sample(self, sample: TelemetrySample) -> None:
    with self._lock:
      if self._closed:
        return
      line = json.dumps(sample.to_dict(), allow_nan=False) + "\n"
      self._telemetry_file.write(line)
      self.samples.append(sample)

  def record_event(self, event: EventRecord) -> None:
    with self._lock:
      if self._closed:
        return
      line = json.dumps(event.to_dict(), allow_nan=False) + "\n"
      self._events_file.write(line)
      self.events.append(event)

  def finalize(
    self,
    termination_reason: str,
    extra_summary: dict[str, Any] | None = None,
  ) -> dict[str, Any]:
    with self._lock:
      if self._closed:
        summary_path = self.output_dir / "summary.json"
        return json.loads(summary_path.read_text(encoding="utf-8"))
      self._telemetry_file.flush()
      self._events_file.flush()

      metric_error = False
      if self.samples:
        try:
          metrics = compute_metrics(self.samples, self.events, self.thresholds)
        except (ArithmeticError, TypeError, ValueError) as exc:
          metric_error = True
          metrics = {
            "success": False,
            "intervention_free_success": False,
            "quality_pass": False,
            "error": f"metric computation failed: {type(exc).__name__}: {exc}",
          }
      else:
        metrics = {
          "success": False,
          "intervention_free_success": False,
          "quality_pass": False,
          "error": "run ended before any telemetry samples were recorded",
        }
      missing_required_telemetry = not self.samples and termination_reason != "engagement_timeout"
      integrity_failure_reason = (
        "metric_computation_failed"
        if metric_error
        else "missing_required_telemetry"
        if missing_required_telemetry
        else termination_reason
        if termination_reason not in EXPECTED_EVALUATION_TERMINATIONS
        else None
      )
      system_failure = integrity_failure_reason is not None
      if system_failure:
        metrics["success"] = False
        metrics["intervention_free_success"] = False
        metrics["quality_pass"] = False
        criteria = metrics.setdefault("criteria", {})
        criteria["system_integrity"] = {
          "value": integrity_failure_reason,
          "operator": "in",
          "threshold": sorted(EXPECTED_EVALUATION_TERMINATIONS),
          "pass": False,
        }
      summary: dict[str, Any] = {
        "schema_version": 1,
        "run_id": self.metadata.get("run_id"),
        "experiment": self.metadata.get("experiment"),
        "controller": self.metadata.get("controller"),
        "backend": self.metadata.get("backend"),
        "valid_for_research": bool(self.metadata.get("valid_for_research", False))
        and not system_failure,
        "scenario_id": self.metadata.get("scenario", {}).get("id"),
        "scenario_kind": self.metadata.get("scenario", {}).get("kind"),
        "condition_id": self.metadata.get("condition", {}).get("id"),
        "repetition": self.metadata.get("repetition"),
        "seed": self.metadata.get("seed"),
        "execution_index": self.metadata.get("execution_index"),
        "order_seed": self.metadata.get("order_seed"),
        "termination_reason": termination_reason,
        "artifact_class": self.metadata.get("artifact_class", "evaluation_trial"),
        "invalid_reason": (
          integrity_failure_reason
          if system_failure
          else self.metadata.get("invalid_reason")
          if not self.metadata.get("valid_for_research", False)
          else None
        ),
        "thresholds": self.thresholds.to_dict(),
        "scenario_sha256": self.metadata.get("scenario_sha256"),
        "condition_sha256": self.metadata.get("condition_sha256"),
        "experiment_sha256": self.metadata.get("experiment_sha256"),
        "runtime_config_sha256": self.metadata.get("runtime_config_sha256"),
        "runtime_effective_sha256": self.metadata.get("runtime_effective_sha256"),
        "software": self.metadata.get("software", {}),
        "metrics": metrics,
      }
      if extra_summary:
        summary["runtime"] = extra_summary
      atomic_write_json(self.output_dir / "summary.json", summary)
      self._telemetry_file.close()
      self._events_file.close()
      self._closed = True
      return summary

  def abort(self, reason: str, error: BaseException | None = None) -> dict[str, Any]:
    details: dict[str, Any] = {}
    if error is not None:
      details = {"error_type": type(error).__name__, "error": str(error)}
    return self.finalize(reason, details)

  def __enter__(self) -> RunRecorder:
    return self

  def __exit__(self, exc_type: Any, exc: BaseException | None, traceback: Any) -> bool:
    if not self._closed:
      self.abort("exception" if exc else "context_closed", exc)
    return False
