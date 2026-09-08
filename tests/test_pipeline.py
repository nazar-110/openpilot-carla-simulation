from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import pytest

from opencarla_eval import provenance
from opencarla_eval.analysis import (
  analyze_results,
  environmental_comparisons,
  load_run_rows,
  paired_comparisons,
)
from opencarla_eval.config import expand_runs, load_experiment
from opencarla_eval.errors import StaleResultError
from opencarla_eval.provenance import reusable_summary_mismatches
from opencarla_eval.runtime_config import load_runtime_config
from opencarla_eval.synthetic import run_synthetic

ROOT = Path(__file__).resolve().parents[1]


def test_synthetic_run_emits_raw_and_summary_artifacts(tmp_path: Path) -> None:
  experiment = load_experiment(ROOT / "experiments" / "smoke.yaml")
  original = next(iter(expand_runs(experiment)))
  run = replace(original, output_dir=tmp_path / original.run_id)

  summary = run_synthetic(run)

  assert summary["backend"] == "synthetic_smoke"
  assert summary["valid_for_research"] is False
  assert (run.output_dir / "metadata.json").is_file()
  assert (run.output_dir / "telemetry.jsonl").is_file()
  assert (run.output_dir / "events.jsonl").is_file()
  assert (run.output_dir / "summary.json").is_file()
  first_sample = json.loads(
    (run.output_dir / "telemetry.jsonl").read_text(encoding="utf-8").splitlines()[0]
  )
  assert first_sample["frame"] == 0


def test_analysis_writes_research_validity_warning(tmp_path: Path) -> None:
  experiment = load_experiment(ROOT / "experiments" / "smoke.yaml")
  runs = list(expand_runs(experiment))[:2]
  results = tmp_path / "results"
  for run in runs:
    run_synthetic(replace(run, output_dir=results / run.run_id))

  output = tmp_path / "analysis"
  manifest = analyze_results(results, output)

  assert manifest["run_count"] == 2
  assert manifest["research_valid_run_count"] == 0
  assert "No result in this report is marked" in (output / "report.md").read_text(encoding="utf-8")
  assert (output / "runs.csv").is_file()
  assert (output / "aggregate.csv").is_file()


def test_paired_comparison_matches_seed_and_repetition() -> None:
  common = {
    "experiment": "urban_suite",
    "scenario_id": "lane_following",
    "condition_id": "clear",
    "backend": "carla",
    "valid_for_research": True,
    "seed": 12,
    "repetition": 1,
    "lateral_rmse_m": 0.2,
  }
  rows = [
    {**common, "controller": "openpilot", "success": True},
    {**common, "controller": "traffic_manager", "success": False, "lateral_rmse_m": 0.1},
  ]

  comparison = paired_comparisons(rows)[0]

  assert comparison["paired_n"] == 1
  assert comparison["success_risk_difference_a_minus_b"] == 1.0
  assert comparison["mcnemar_a_only_success"] == 1


def test_duplicate_pair_member_is_rejected() -> None:
  row = {
    "experiment": "urban_suite",
    "scenario_id": "lane_following",
    "condition_id": "clear",
    "backend": "carla",
    "valid_for_research": True,
    "seed": 12,
    "repetition": 1,
    "lateral_rmse_m": 0.2,
    "controller": "openpilot",
    "success": True,
  }
  rows = [row, dict(row), {**row, "controller": "traffic_manager"}]

  with pytest.raises(ValueError, match="Duplicate pair member"):
    paired_comparisons(rows)


def test_malformed_summary_is_rejected(tmp_path: Path) -> None:
  (tmp_path / "summary.json").write_text("{}\n", encoding="utf-8")

  with pytest.raises(ValueError, match="schema_version"):
    load_run_rows(tmp_path)


def test_overwrite_archives_prior_attempt(tmp_path: Path) -> None:
  experiment = load_experiment(ROOT / "experiments" / "smoke.yaml")
  original = next(iter(expand_runs(experiment)))
  run = replace(original, output_dir=tmp_path / original.run_id)
  run_synthetic(run)

  run_synthetic(run, overwrite=True)

  archived = run.output_dir / "attempts" / "attempt_001" / "summary.archived.json"
  assert archived.is_file()
  assert len(list(run.output_dir.rglob("summary.json"))) == 1
  manifest = analyze_results(run.output_dir, tmp_path / "attempt_analysis")
  assert manifest["archived_attempt_count"] == 1
  assert manifest["archived_invalid_attempt_count"] == 1


def test_stale_completed_result_requires_archival_overwrite(tmp_path: Path) -> None:
  experiment = load_experiment(ROOT / "experiments" / "smoke.yaml")
  original = next(iter(expand_runs(experiment)))
  run = replace(original, output_dir=tmp_path / original.run_id)
  summary = run_synthetic(run)
  summary["scenario_sha256"] = "0" * 64
  (run.output_dir / "summary.json").write_text(json.dumps(summary), encoding="utf-8")

  with pytest.raises(StaleResultError, match="scenario_sha256"):
    run_synthetic(run)

  assert not (run.output_dir / "attempts").exists()
  replacement = run_synthetic(run, overwrite=True)
  assert replacement["scenario_sha256"] != "0" * 64
  assert (run.output_dir / "attempts" / "attempt_001" / "summary.archived.json").is_file()


def test_reuse_rejects_incomplete_raw_artifact_set(tmp_path: Path) -> None:
  experiment = load_experiment(ROOT / "experiments" / "smoke.yaml")
  original = next(iter(expand_runs(experiment)))
  run = replace(original, output_dir=tmp_path / original.run_id)
  run_synthetic(run)
  (run.output_dir / "telemetry.jsonl").unlink()

  with pytest.raises(StaleResultError, match=r"artifact_missing\.telemetry\.jsonl"):
    run_synthetic(run)


def test_real_reuse_gate_requires_runtime_and_software_provenance(tmp_path: Path) -> None:
  experiment = load_experiment(ROOT / "experiments" / "smoke.yaml")
  original = next(iter(expand_runs(experiment)))
  run = replace(original, output_dir=tmp_path / original.run_id)
  summary = run_synthetic(run)
  summary["backend"] = "carla"

  mismatches = reusable_summary_mismatches(
    summary,
    run,
    backend="carla",
    runtime_config=load_runtime_config(ROOT / "config" / "carla.yaml"),
  )

  assert "runtime_config_sha256" in mismatches
  assert "runtime_effective_sha256" in mismatches
  assert any(item.startswith("software.") for item in mismatches)


def test_valid_openpilot_reuse_requires_integrity_audit_and_manager_log(
  tmp_path: Path,
) -> None:
  experiment = load_experiment(ROOT / "experiments" / "smoke.yaml")
  original = next(iter(expand_runs(experiment)))
  run = replace(
    original,
    controller="openpilot",
    output_dir=tmp_path / original.run_id,
  )
  summary = run_synthetic(run)
  config = load_runtime_config(ROOT / "config" / "carla.yaml")
  toolkit_commit, toolkit_dirty = provenance._git_state(provenance.PROJECT_ROOT)
  openpilot_commit = "4df40d2c1946a57242230186edd073c4073060a6"
  summary.update(
    {
      "backend": "carla",
      "valid_for_research": True,
      "artifact_class": "evaluation_trial",
      "invalid_reason": None,
      "termination_reason": "route_completed",
      "runtime_config_sha256": provenance._file_sha256(config.source_path),
      "runtime_effective_sha256": provenance._runtime_effective_sha256(config),
      "software": {
        "toolkit_commit": toolkit_commit,
        "toolkit_worktree_dirty": toolkit_dirty,
        "openpilot_commit": openpilot_commit,
        "openpilot_worktree_dirty": False,
      },
      "runtime": {},
    }
  )

  mismatches = reusable_summary_mismatches(
    summary,
    run,
    backend="carla",
    runtime_config=config,
    openpilot_commit=openpilot_commit,
    openpilot_dirty=False,
  )

  assert mismatches == [
    "artifact_missing.openpilot_manager.log",
    "runtime.recorder_collision_audit",
  ]
  summary["runtime"] = {"recorder_collision_audit": {"valid": True}}
  assert reusable_summary_mismatches(
    summary,
    run,
    backend="carla",
    runtime_config=config,
    openpilot_commit=openpilot_commit,
    openpilot_dirty=False,
  ) == ["artifact_missing.openpilot_manager.log"]
  (run.output_dir / "openpilot_manager.log").touch()
  assert (
    reusable_summary_mismatches(
      summary,
      run,
      backend="carla",
      runtime_config=config,
      openpilot_commit=openpilot_commit,
      openpilot_dirty=False,
    )
    == []
  )


def test_full_provenance_invalid_real_artifact_requires_same_seed_rerun(
  tmp_path: Path,
) -> None:
  experiment = load_experiment(ROOT / "experiments" / "smoke.yaml")
  original = next(iter(expand_runs(experiment)))
  run = replace(original, output_dir=tmp_path / original.run_id)
  summary = run_synthetic(run)
  config = load_runtime_config(ROOT / "config" / "carla.yaml")
  toolkit_commit, toolkit_dirty = provenance._git_state(provenance.PROJECT_ROOT)
  summary.update(
    {
      "backend": "carla",
      "artifact_class": "invalid_trial",
      "invalid_trial": True,
      "invalid_reason": "camera_timeout",
      "termination_reason": "camera_timeout",
      "runtime_config_sha256": provenance._file_sha256(config.source_path),
      "runtime_effective_sha256": provenance._runtime_effective_sha256(config),
      "software": {
        "toolkit_commit": toolkit_commit,
        "toolkit_worktree_dirty": toolkit_dirty,
      },
      "runtime": {"rerun_same_seed": True},
    }
  )

  mismatches = reusable_summary_mismatches(
    summary,
    run,
    backend="carla",
    runtime_config=config,
  )

  assert mismatches == ["artifact_requires_same_seed_rerun"]


def test_environmental_comparisons_separate_condition_hash_pairs() -> None:
  common = {
    "experiment": "urban_suite",
    "scenario_id": "lane_following",
    "controller": "openpilot",
    "backend": "carla",
    "valid_for_research": True,
    "success": True,
  }
  rows: list[dict[str, object]] = []
  for seed, reference_hash, comparison_hash in (
    (1, "reference-v1", "rain-v1"),
    (2, "reference-v2", "rain-v2"),
  ):
    rows.extend(
      [
        {
          **common,
          "condition_id": "clear_noon",
          "condition_sha256": reference_hash,
          "seed": seed,
          "repetition": seed,
        },
        {
          **common,
          "condition_id": "rainy_daylight",
          "condition_sha256": comparison_hash,
          "seed": seed,
          "repetition": seed,
          "success": False,
        },
      ]
    )

  comparisons = environmental_comparisons(rows)

  assert len(comparisons) == 2
  assert {row["paired_n"] for row in comparisons} == {1}
  assert {
    (row["reference_condition_sha256"], row["comparison_condition_sha256"]) for row in comparisons
  } == {("reference-v1", "rain-v1"), ("reference-v2", "rain-v2")}


def test_synthetic_lead_trigger_uses_declared_fallback(tmp_path: Path) -> None:
  experiment = load_experiment(ROOT / "experiments" / "smoke.yaml")
  lead = next(run for run in expand_runs(experiment) if run.scenario.kind == "lead_vehicle_braking")
  run = replace(lead, output_dir=tmp_path / lead.run_id)

  run_synthetic(run)
  events = [json.loads(line) for line in (run.output_dir / "events.jsonl").read_text().splitlines()]
  trigger = next(event for event in events if event["event_type"] == "scenario_trigger")

  assert trigger["sim_time_s"] == pytest.approx(7.0, abs=0.05)
