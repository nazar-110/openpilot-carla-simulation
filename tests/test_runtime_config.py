from pathlib import Path

import pytest
import yaml

from opencarla_eval.errors import ConfigurationError
from opencarla_eval.runtime_config import load_runtime_config

ROOT = Path(__file__).resolve().parents[1]


def test_default_runtime_is_openpilot_camera_compatible() -> None:
  config = load_runtime_config(ROOT / "config" / "carla.yaml")

  assert config.required_version == "0.9.16"
  assert config.camera.width == 1928
  assert config.camera.height == 1208
  assert config.fixed_delta_seconds == 0.05
  assert config.max_substep_delta_time * config.max_substeps >= config.fixed_delta_seconds
  assert config.spectator.enabled
  assert config.spectator.distance_m == 7.0
  assert not config.dashboard.enabled
  assert config.dashboard.host == "127.0.0.1"


def test_inconsistent_substeps_are_rejected(tmp_path: Path) -> None:
  source = (ROOT / "config" / "carla.yaml").read_text(encoding="utf-8")
  path = tmp_path / "bad_runtime.yaml"
  path.write_text(source.replace("max_substeps: 10", "max_substeps: 2"), encoding="utf-8")

  with pytest.raises(ConfigurationError, match="fixed_delta_seconds"):
    load_runtime_config(path)


def test_string_boolean_is_not_coerced(tmp_path: Path) -> None:
  source = (ROOT / "config" / "carla.yaml").read_text(encoding="utf-8")
  path = tmp_path / "bad_boolean.yaml"
  path.write_text(source.replace("substepping: true", 'substepping: "false"'), encoding="utf-8")

  with pytest.raises(ConfigurationError, match="substepping"):
    load_runtime_config(path)


def test_camera_period_must_match_openpilot(tmp_path: Path) -> None:
  source = (ROOT / "config" / "carla.yaml").read_text(encoding="utf-8")
  path = tmp_path / "bad_period.yaml"
  path.write_text(source.replace("sensor_tick_s: 0.05", "sensor_tick_s: 0.10"), encoding="utf-8")

  with pytest.raises(ConfigurationError, match="sensor_tick_s"):
    load_runtime_config(path)


def test_invalid_spectator_pitch_is_rejected(tmp_path: Path) -> None:
  source = (ROOT / "config" / "carla.yaml").read_text(encoding="utf-8")
  path = tmp_path / "bad_spectator.yaml"
  path.write_text(source.replace("pitch_deg: -15.0", "pitch_deg: -90.0"), encoding="utf-8")

  with pytest.raises(ConfigurationError, match="spectator.pitch_deg"):
    load_runtime_config(path)


def test_dashboard_environment_override(monkeypatch: pytest.MonkeyPatch) -> None:
  monkeypatch.setenv("OPENCARLA_DASHBOARD", "true")
  monkeypatch.setenv("OPENCARLA_DASHBOARD_PORT", "8788")

  config = load_runtime_config(ROOT / "config" / "carla.yaml")

  assert config.dashboard.enabled
  assert config.dashboard.port == 8788


def test_visualization_sections_are_optional_for_schema_one(tmp_path: Path) -> None:
  data = yaml.safe_load((ROOT / "config" / "carla.yaml").read_text(encoding="utf-8"))
  data["carla"].pop("spectator")
  data["carla"].pop("dashboard")
  path = tmp_path / "legacy_runtime.yaml"
  path.write_text(yaml.safe_dump(data), encoding="utf-8")

  config = load_runtime_config(path)

  assert config.spectator.enabled
  assert config.spectator.distance_m == 7.0
  assert not config.dashboard.enabled
  assert config.dashboard.frame_stride == 4
