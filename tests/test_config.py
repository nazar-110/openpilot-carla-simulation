from __future__ import annotations

from pathlib import Path

import pytest

from opencarla_eval.config import expand_runs, load_experiment, load_scenario
from opencarla_eval.errors import ConfigurationError

ROOT = Path(__file__).resolve().parents[1]


def test_urban_suite_expands_to_paired_420_run_matrix() -> None:
  experiment = load_experiment(ROOT / "experiments" / "urban_suite.yaml")
  runs = list(expand_runs(experiment))

  assert experiment.run_count == 420
  assert len(runs) == 420
  first_pair = runs[:2]
  assert [run.controller for run in first_pair] == ["openpilot", "traffic_manager"]
  assert first_pair[0].seed == first_pair[1].seed == 41000
  assert first_pair[0].scenario.id == first_pair[1].scenario.id == "lane_following"
  assert first_pair[0].condition.id == first_pair[1].condition.id == "clear_noon"
  assert [run.controller for run in runs[2:4]] == ["traffic_manager", "openpilot"]
  assert len({run.run_id for run in runs}) == len(runs)


def test_all_scenarios_load_with_explicit_thresholds() -> None:
  experiment = load_experiment(ROOT / "experiments" / "urban_suite.yaml")
  scenarios = [load_scenario(path) for path in experiment.scenario_paths]

  assert len(scenarios) == 7
  assert all(scenario.duration_s > 0 for scenario in scenarios)
  assert all(scenario.thresholds.max_collisions == 0 for scenario in scenarios)


def test_invalid_schema_has_actionable_error(tmp_path: Path) -> None:
  path = tmp_path / "bad.yaml"
  path.write_text("schema_version: 99\nscenario: {}\n", encoding="utf-8")

  with pytest.raises(ConfigurationError, match="schema_version"):
    load_scenario(path)


def test_invalid_threshold_type_is_rejected(tmp_path: Path) -> None:
  source = (ROOT / "scenarios" / "lane_following.yaml").read_text(encoding="utf-8")
  path = tmp_path / "bad_threshold.yaml"
  path.write_text(source.replace("max_collisions: 0", 'max_collisions: "none"'), encoding="utf-8")

  with pytest.raises(ConfigurationError, match="max_collisions"):
    load_scenario(path)


def test_unknown_scenario_parameter_is_rejected(tmp_path: Path) -> None:
  source = (ROOT / "scenarios" / "lane_following.yaml").read_text(encoding="utf-8")
  path = tmp_path / "typo.yaml"
  path.write_text(
    source.replace("warmup_active_s: 1", "warmup_active_s: 1\n    warmup_active: 5"),
    encoding="utf-8",
  )

  with pytest.raises(ConfigurationError, match="warmup_active"):
    load_scenario(path)


def test_lane_following_startup_guard_parameters_are_validated() -> None:
  scenario = load_scenario(ROOT / "scenarios" / "lane_following.yaml")

  assert scenario.parameters["minimum_junction_clearance_m"] == 160.0
  assert scenario.parameters["warmup_active_s"] == 1.0
  assert scenario.parameters["startup_target_speed_mps"] == 5.0
  assert scenario.parameters["startup_max_throttle"] == 0.35
  assert scenario.parameters["stuck_timeout_s"] == 5.0


def test_nonfinite_threshold_is_rejected(tmp_path: Path) -> None:
  source = (ROOT / "scenarios" / "lane_following.yaml").read_text(encoding="utf-8")
  path = tmp_path / "nan.yaml"
  path.write_text(
    source.replace("max_abs_lateral_error_m: 1.0", "max_abs_lateral_error_m: .nan"),
    encoding="utf-8",
  )

  with pytest.raises(ConfigurationError, match="max_abs_lateral_error_m"):
    load_scenario(path)
