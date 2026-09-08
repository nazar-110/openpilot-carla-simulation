"""Run-level aggregation, paired comparisons, plots, and Markdown reporting."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import random
from collections import defaultdict
from collections.abc import Sequence
from pathlib import Path
from statistics import fmean
from typing import Any


def _nested(data: dict[str, Any], *keys: str, default: Any = None) -> Any:
  value: Any = data
  for key in keys:
    if not isinstance(value, dict) or key not in value:
      return default
    value = value[key]
  return value


def discover_summaries(root: str | Path) -> list[Path]:
  path = Path(root).expanduser().resolve()
  if path.is_file() and path.name == "summary.json":
    return [path]
  if not path.exists():
    raise FileNotFoundError(f"Results path does not exist: {path}")
  return sorted(path.rglob("summary.json"))


def load_attempt_rows(root: str | Path) -> list[dict[str, Any]]:
  """Read archived attempts for accounting without treating them as outcome units."""

  path = Path(root).expanduser().resolve()
  if path.is_file():
    return []
  rows: list[dict[str, Any]] = []
  for summary_path in sorted(path.rglob("summary.archived.json")):
    try:
      summary = json.loads(summary_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError) as exc:
      raise ValueError(f"Cannot read archived summary {summary_path}: {exc}") from exc
    if not isinstance(summary, dict) or summary.get("schema_version") != 1:
      raise ValueError(f"{summary_path}: archived summary must use schema_version 1")
    rows.append(
      {
        "run_id": summary.get("run_id"),
        "experiment": summary.get("experiment"),
        "attempt": summary_path.parent.name,
        "research_valid": summary.get("valid_for_research"),
        "artifact_class": summary.get("artifact_class", "evaluation_trial"),
        "termination_reason": summary.get("termination_reason"),
        "invalid_reason": summary.get("invalid_reason")
        or (
          summary.get("termination_reason") if summary.get("valid_for_research") is False else None
        ),
        "summary_path": str(summary_path),
      }
    )
  return rows


def _nonempty_string(value: Any, label: str, path: Path) -> str:
  if not isinstance(value, str) or not value.strip():
    raise ValueError(f"{path}: {label} must be a non-empty string")
  return value


def _finite_number(value: Any, label: str, path: Path) -> float:
  if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
    raise ValueError(f"{path}: {label} must be a finite number")
  return float(value)


def _validate_summary(summary: Any, path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
  if not isinstance(summary, dict):
    raise ValueError(f"{path}: summary root must be an object")
  if summary.get("schema_version") != 1:
    raise ValueError(f"{path}: schema_version must equal 1")
  for name in (
    "run_id",
    "experiment",
    "scenario_id",
    "scenario_kind",
    "condition_id",
    "controller",
    "backend",
    "termination_reason",
  ):
    _nonempty_string(summary.get(name), name, path)
  if type(summary.get("valid_for_research")) is not bool:
    raise ValueError(f"{path}: valid_for_research must be boolean")
  for name in ("repetition", "seed"):
    value = summary.get(name)
    if isinstance(value, bool) or not isinstance(value, int):
      raise ValueError(f"{path}: {name} must be an integer")
  metrics = summary.get("metrics")
  if not isinstance(metrics, dict):
    raise ValueError(f"{path}: metrics must be an object")
  for name in ("success", "quality_pass"):
    if type(metrics.get(name)) is not bool:
      raise ValueError(f"{path}: metrics.{name} must be boolean")
  software = summary.get("software", {})
  if not isinstance(software, dict):
    raise ValueError(f"{path}: software must be an object")
  if summary["valid_for_research"]:
    for name in (
      "scenario_sha256",
      "condition_sha256",
      "experiment_sha256",
      "runtime_config_sha256",
      "runtime_effective_sha256",
    ):
      value = summary.get(name)
      if not isinstance(value, str) or len(value) != 64:
        raise ValueError(f"{path}: {name} must be a SHA-256 string for research-valid runs")
    if (
      not isinstance(software.get("toolkit_commit"), str)
      or software.get("toolkit_worktree_dirty") is not False
    ):
      raise ValueError(f"{path}: research-valid runs require a clean, versioned toolkit")
    if summary["controller"] == "openpilot" and (
      not isinstance(software.get("openpilot_commit"), str)
      or software.get("openpilot_worktree_dirty") is not False
    ):
      raise ValueError(f"{path}: research-valid OpenPilot runs require a clean commit")
    required_metrics = (
      ("mission", "route_completion"),
      ("mission", "duration_s"),
      ("safety", "collision_count"),
      ("safety", "lane_invasion_count"),
      ("lane_keeping", "rmse_lateral_error_m"),
      ("speed_tracking", "rmse_mps"),
      ("comfort", "rms_jerk_mps3"),
      ("system", "controller_active_fraction"),
    )
    if summary["termination_reason"] != "engagement_timeout":
      for keys in required_metrics:
        _finite_number(_nested(metrics, *keys), f"metrics.{'.'.join(keys)}", path)
  return summary, metrics


def load_run_rows(root: str | Path) -> list[dict[str, Any]]:
  rows: list[dict[str, Any]] = []
  identities: dict[tuple[str, str], Path] = {}
  for path in discover_summaries(root):
    try:
      summary = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError) as exc:
      raise ValueError(f"Cannot read summary {path}: {exc}") from exc
    summary, metrics = _validate_summary(summary, path)
    identity = (summary["experiment"], summary["run_id"])
    if identity in identities:
      raise ValueError(
        f"Duplicate summary identity {identity!r}: {identities[identity]} and {path}"
      )
    identities[identity] = path
    software = summary.get("software", {})
    software_sha256 = hashlib.sha256(
      json.dumps(software, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    common_software = {
      key: software.get(key)
      for key in (
        "carla_client",
        "carla_server",
        "toolkit_commit",
        "toolkit_worktree_dirty",
        "python",
        "platform",
        "bridge",
      )
    }
    common_software_sha256 = hashlib.sha256(
      json.dumps(common_software, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    rows.append(
      {
        "run_id": summary["run_id"],
        "experiment": summary["experiment"],
        "scenario_id": summary.get("scenario_id"),
        "scenario_kind": summary.get("scenario_kind"),
        "condition_id": summary.get("condition_id"),
        "controller": summary.get("controller"),
        "backend": summary.get("backend"),
        "valid_for_research": summary["valid_for_research"],
        "repetition": summary.get("repetition"),
        "seed": summary.get("seed"),
        "termination_reason": summary.get("termination_reason"),
        "success": metrics["success"],
        "quality_pass": metrics["quality_pass"],
        "invalid_reason": summary.get("invalid_reason")
        or (summary.get("termination_reason") if not summary["valid_for_research"] else None),
        "scenario_sha256": summary.get("scenario_sha256"),
        "condition_sha256": summary.get("condition_sha256"),
        "experiment_sha256": summary.get("experiment_sha256"),
        "runtime_config_sha256": summary.get("runtime_config_sha256"),
        "runtime_effective_sha256": summary.get("runtime_effective_sha256"),
        "software_sha256": software_sha256,
        "common_software_sha256": common_software_sha256,
        "route_completion": _nested(metrics, "mission", "route_completion"),
        "duration_s": _nested(metrics, "mission", "duration_s"),
        "distance_m": _nested(metrics, "mission", "distance_m"),
        "collision_count": _nested(metrics, "safety", "collision_count"),
        "lane_invasion_count": _nested(metrics, "safety", "lane_invasion_count"),
        "offroad_duration_s": _nested(metrics, "safety", "offroad_duration_s"),
        "min_ttc_s": _nested(metrics, "safety", "min_ttc_s"),
        "near_miss_count": _nested(metrics, "safety", "near_miss_count"),
        "lateral_rmse_m": _nested(metrics, "lane_keeping", "rmse_lateral_error_m"),
        "lateral_max_m": _nested(metrics, "lane_keeping", "max_abs_lateral_error_m"),
        "speed_rmse_mps": _nested(metrics, "speed_tracking", "rmse_mps"),
        "rms_jerk_mps3": _nested(metrics, "comfort", "rms_jerk_mps3"),
        "red_light_violations": _nested(metrics, "compliance", "red_light_violations"),
        "controller_active_fraction": _nested(metrics, "system", "controller_active_fraction"),
        "latency_p95_ms": _nested(metrics, "system", "control_latency_p95_ms"),
        "summary_path": str(path),
      }
    )
  return rows


def _wilson(successes: int, total: int, z: float = 1.959963984540054) -> tuple[float, float]:
  if total == 0:
    return 0.0, 0.0
  proportion = successes / total
  denominator = 1.0 + z * z / total
  center = (proportion + z * z / (2.0 * total)) / denominator
  half = (
    z
    * math.sqrt(proportion * (1.0 - proportion) / total + z * z / (4.0 * total * total))
    / denominator
  )
  return max(0.0, center - half), min(1.0, center + half)


def _mean_present(rows: Sequence[dict[str, Any]], key: str) -> float | None:
  values = [float(row[key]) for row in rows if row.get(key) is not None]
  return fmean(values) if values else None


def aggregate_rows(rows: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
  grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
  for row in rows:
    key = (
      row["experiment"],
      row["scenario_id"],
      row["condition_id"],
      row["controller"],
      row["backend"],
      row["valid_for_research"],
      row.get("scenario_sha256"),
      row.get("condition_sha256"),
      row.get("experiment_sha256"),
      row.get("runtime_config_sha256"),
      row.get("runtime_effective_sha256"),
      row.get("software_sha256"),
    )
    grouped[key].append(row)

  output: list[dict[str, Any]] = []
  for key, group in sorted(grouped.items(), key=lambda item: tuple(str(x) for x in item[0])):
    successes = sum(bool(row["success"]) for row in group)
    ci_low, ci_high = _wilson(successes, len(group))
    collision_values = [
      row["collision_count"] for row in group if row.get("collision_count") is not None
    ]
    output.append(
      {
        "experiment": key[0],
        "scenario_id": key[1],
        "condition_id": key[2],
        "controller": key[3],
        "backend": key[4],
        "valid_for_research": key[5],
        "scenario_sha256": key[6],
        "condition_sha256": key[7],
        "experiment_sha256": key[8],
        "runtime_config_sha256": key[9],
        "runtime_effective_sha256": key[10],
        "software_sha256": key[11],
        "n": len(group),
        "successes": successes,
        "success_rate": successes / len(group),
        "success_ci95_low": ci_low,
        "success_ci95_high": ci_high,
        "collision_rate": (
          sum(float(value) > 0 for value in collision_values) / len(collision_values)
          if collision_values
          else None
        ),
        "mean_collision_count": _mean_present(group, "collision_count"),
        "mean_route_completion": _mean_present(group, "route_completion"),
        "mean_lane_invasions": _mean_present(group, "lane_invasion_count"),
        "mean_min_ttc_s": _mean_present(group, "min_ttc_s"),
        "mean_lateral_rmse_m": _mean_present(group, "lateral_rmse_m"),
        "mean_speed_rmse_mps": _mean_present(group, "speed_rmse_mps"),
        "mean_rms_jerk_mps3": _mean_present(group, "rms_jerk_mps3"),
        "mean_latency_p95_ms": _mean_present(group, "latency_p95_ms"),
      }
    )
  return output


def _bootstrap_mean_ci(values: Sequence[float], seed: int = 20260714) -> tuple[float, float]:
  if not values:
    return 0.0, 0.0
  if len(values) == 1:
    return values[0], values[0]
  rng = random.Random(seed)
  estimates = [fmean(values[rng.randrange(len(values))] for _ in values) for _ in range(5000)]
  estimates.sort()
  return estimates[124], estimates[4874]


def _mcnemar_exact(b: int, c: int) -> float:
  discordant = b + c
  if discordant == 0:
    return 1.0
  tail = sum(math.comb(discordant, index) for index in range(min(b, c) + 1))
  return min(1.0, 2.0 * tail / (2**discordant))


def paired_comparisons(rows: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
  rows = [row for row in rows if row.get("valid_for_research")]
  controllers = sorted({str(row["controller"]) for row in rows})
  if len(controllers) != 2:
    return []
  first, second = controllers
  cells: dict[tuple[str, ...], list[dict[str, Any]]] = defaultdict(list)
  for row in rows:
    cells[
      (
        str(row["experiment"]),
        str(row["scenario_id"]),
        str(row["condition_id"]),
        str(row["backend"]),
        str(row.get("scenario_sha256")),
        str(row.get("condition_sha256")),
        str(row.get("experiment_sha256")),
        str(row.get("runtime_config_sha256")),
        str(row.get("runtime_effective_sha256")),
        str(row.get("common_software_sha256")),
      )
    ].append(row)

  output: list[dict[str, Any]] = []
  for (
    experiment,
    scenario,
    condition,
    backend,
    scenario_sha256,
    condition_sha256,
    experiment_sha256,
    runtime_config_sha256,
    runtime_effective_sha256,
    common_software_sha256,
  ), cell_rows in sorted(cells.items()):
    by_key: dict[tuple[Any, Any], dict[str, dict[str, Any]]] = defaultdict(dict)
    for row in cell_rows:
      pair = by_key[(row["seed"], row["repetition"])]
      controller = str(row["controller"])
      if controller in pair:
        raise ValueError(
          "Duplicate pair member for "
          f"{experiment}/{scenario}/{condition}/{row['seed']}/{controller}"
        )
      pair[controller] = row
    pairs = [pair for pair in by_key.values() if first in pair and second in pair]
    if not pairs:
      continue
    success_differences = [
      float(pair[first]["success"]) - float(pair[second]["success"]) for pair in pairs
    ]
    ci_low, ci_high = _bootstrap_mean_ci(success_differences)
    b = sum(pair[first]["success"] and not pair[second]["success"] for pair in pairs)
    c = sum(not pair[first]["success"] and pair[second]["success"] for pair in pairs)
    lateral_differences = [
      float(pair[first]["lateral_rmse_m"]) - float(pair[second]["lateral_rmse_m"])
      for pair in pairs
      if pair[first].get("lateral_rmse_m") is not None
      and pair[second].get("lateral_rmse_m") is not None
    ]
    lat_low, lat_high = _bootstrap_mean_ci(lateral_differences)
    output.append(
      {
        "experiment": experiment,
        "scenario_id": scenario,
        "condition_id": condition,
        "backend": backend,
        "scenario_sha256": scenario_sha256,
        "condition_sha256": condition_sha256,
        "experiment_sha256": experiment_sha256,
        "runtime_config_sha256": runtime_config_sha256,
        "runtime_effective_sha256": runtime_effective_sha256,
        "common_software_sha256": common_software_sha256,
        "controller_a": first,
        "controller_b": second,
        "paired_n": len(pairs),
        "success_risk_difference_a_minus_b": fmean(success_differences),
        "success_difference_ci95_low": ci_low,
        "success_difference_ci95_high": ci_high,
        "mcnemar_a_only_success": b,
        "mcnemar_b_only_success": c,
        "mcnemar_exact_p": _mcnemar_exact(b, c),
        "lateral_rmse_mean_difference_m": fmean(lateral_differences)
        if lateral_differences
        else None,
        "lateral_rmse_difference_ci95_low": lat_low if lateral_differences else None,
        "lateral_rmse_difference_ci95_high": lat_high if lateral_differences else None,
      }
    )
  _add_holm_adjustment(output, "mcnemar_exact_p")
  return output


def _add_holm_adjustment(rows: list[dict[str, Any]], key: str) -> None:
  """Add family-wise Holm-adjusted p-values in place."""

  ordered = sorted(enumerate(rows), key=lambda item: float(item[1][key]))
  running = 0.0
  total = len(ordered)
  for rank, (index, row) in enumerate(ordered):
    running = max(running, min(1.0, float(row[key]) * (total - rank)))
    rows[index][f"{key}_holm"] = running


def environmental_comparisons(
  rows: Sequence[dict[str, Any]], reference: str = "clear_noon"
) -> list[dict[str, Any]]:
  """Matched within-controller condition contrasts against a declared reference."""

  valid = [row for row in rows if row.get("valid_for_research")]
  cells: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
  for row in valid:
    cells[
      (
        row["experiment"],
        row["scenario_id"],
        row["controller"],
        row["backend"],
        row.get("scenario_sha256"),
        row.get("experiment_sha256"),
        row.get("runtime_config_sha256"),
        row.get("runtime_effective_sha256"),
        row.get("common_software_sha256"),
      )
    ].append(row)

  output: list[dict[str, Any]] = []
  for cell, cell_rows in sorted(cells.items(), key=lambda item: tuple(str(x) for x in item[0])):
    conditions = sorted({str(row["condition_id"]) for row in cell_rows} - {reference})
    for condition in conditions:
      by_key: dict[tuple[Any, Any], dict[str, dict[str, Any]]] = defaultdict(dict)
      for row in cell_rows:
        condition_id = str(row["condition_id"])
        if condition_id not in {reference, condition}:
          continue
        pair = by_key[(row["seed"], row["repetition"])]
        if condition_id in pair:
          raise ValueError(
            f"Duplicate environmental pair member for {cell}/{row['seed']}/{condition_id}"
          )
        pair[condition_id] = row
      complete_pairs = [pair for pair in by_key.values() if reference in pair and condition in pair]
      pairs_by_hash: dict[tuple[str, str], list[dict[str, dict[str, Any]]]] = defaultdict(list)
      for pair in complete_pairs:
        hash_pair = (
          str(pair[reference].get("condition_sha256")),
          str(pair[condition].get("condition_sha256")),
        )
        pairs_by_hash[hash_pair].append(pair)
      for (reference_sha256, comparison_sha256), pairs in sorted(pairs_by_hash.items()):
        differences = [
          float(pair[condition]["success"]) - float(pair[reference]["success"]) for pair in pairs
        ]
        ci_low, ci_high = _bootstrap_mean_ci(differences)
        b = sum(pair[condition]["success"] and not pair[reference]["success"] for pair in pairs)
        c = sum(not pair[condition]["success"] and pair[reference]["success"] for pair in pairs)
        output.append(
          {
            "experiment": cell[0],
            "scenario_id": cell[1],
            "controller": cell[2],
            "backend": cell[3],
            "scenario_sha256": cell[4],
            "experiment_sha256": cell[5],
            "runtime_config_sha256": cell[6],
            "runtime_effective_sha256": cell[7],
            "common_software_sha256": cell[8],
            "reference_condition_sha256": reference_sha256,
            "comparison_condition_sha256": comparison_sha256,
            "reference_condition": reference,
            "comparison_condition": condition,
            "paired_n": len(pairs),
            "success_risk_difference_comparison_minus_reference": fmean(differences),
            "success_difference_ci95_low": ci_low,
            "success_difference_ci95_high": ci_high,
            "mcnemar_exact_p": _mcnemar_exact(b, c),
          }
        )
  _add_holm_adjustment(output, "mcnemar_exact_p")
  return output


def _write_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  if not rows:
    path.write_text("", encoding="utf-8")
    return
  with path.open("w", encoding="utf-8", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)


def _format(value: Any, digits: int = 3) -> str:
  if value is None:
    return "—"
  if isinstance(value, bool):
    return "yes" if value else "no"
  if isinstance(value, float):
    return f"{value:.{digits}f}"
  return str(value)


def _write_report(
  path: Path,
  rows: Sequence[dict[str, Any]],
  aggregates: Sequence[dict[str, Any]],
  comparisons: Sequence[dict[str, Any]],
  environment_comparisons: Sequence[dict[str, Any]],
  attempt_rows: Sequence[dict[str, Any]],
) -> None:
  valid_count = sum(row["valid_for_research"] for row in rows)
  invalid_count = len(rows) - valid_count
  invalid_reasons: dict[str, int] = defaultdict(int)
  for row in rows:
    if not row["valid_for_research"]:
      invalid_reasons[str(row.get("invalid_reason") or "unspecified")] += 1
  lines = [
    "# OpenPilot–CARLA evaluation report",
    "",
    f"Analyzed **{len(rows)}** completed artifacts; **{valid_count}** are marked as "
    f"research-valid and **{invalid_count}** are smoke or invalid-trial artifacts.",
    "",
  ]
  if valid_count == 0:
    lines.extend(
      [
        "> [!WARNING]",
        "> No result in this report is marked `valid_for_research`. Do not use these "
        "numbers as OpenPilot performance claims.",
        "",
      ]
    )
  lines.extend(
    [
      "## Run-level aggregate",
      "",
      "| Experiment | Scenario | Condition | Controller | Backend | Research-valid | n | Success (95% CI) | Collision rate | Route completion | Lateral RMSE (m) |",
      "|---|---|---|---:|---|---:|---:|---:|---:|---:|---:|",
    ]
  )
  for row in aggregates:
    success = (
      f"{row['success_rate']:.1%} [{row['success_ci95_low']:.1%}, {row['success_ci95_high']:.1%}]"
    )
    lines.append(
      "| "
      + " | ".join(
        [
          str(row["experiment"]),
          str(row["scenario_id"]),
          str(row["condition_id"]),
          str(row["controller"]),
          str(row["backend"]),
          "yes" if row["valid_for_research"] else "no",
          str(row["n"]),
          success,
          f"{row['collision_rate']:.1%}" if row["collision_rate"] is not None else "—",
          _format(row["mean_route_completion"]),
          _format(row["mean_lateral_rmse_m"]),
        ]
      )
      + " |"
    )
  if invalid_reasons:
    lines.extend(["", "## Non-research-valid artifact accounting", ""])
    for reason, count in sorted(invalid_reasons.items()):
      lines.append(f"- `{reason}`: {count}")
  if attempt_rows:
    lines.extend(
      [
        "",
        "## Archived attempt history",
        "",
        f"Found **{len(attempt_rows)}** archived prior attempts. They are excluded from outcome denominators and listed in `attempt_history.csv`.",
        "",
      ]
    )
  if comparisons:
    lines.extend(
      [
        "",
        "## Paired controller comparisons",
        "",
        "Risk difference is controller A minus controller B using matched seed/repetition runs.",
        "",
        "| Experiment | Scenario | Condition | Backend | A | B | Paired n | Success risk difference (95% bootstrap CI) | McNemar p |",
        "|---|---|---|---|---:|---:|---:|---:|---:|",
      ]
    )
    for row in comparisons:
      difference = (
        f"{row['success_risk_difference_a_minus_b']:.3f} "
        f"[{row['success_difference_ci95_low']:.3f}, "
        f"{row['success_difference_ci95_high']:.3f}]"
      )
      lines.append(
        "| "
        + " | ".join(
          [
            str(row["experiment"]),
            str(row["scenario_id"]),
            str(row["condition_id"]),
            str(row["backend"]),
            str(row["controller_a"]),
            str(row["controller_b"]),
            str(row["paired_n"]),
            difference,
            _format(row["mcnemar_exact_p"]),
          ]
        )
        + " |"
      )
  if environment_comparisons:
    lines.extend(
      [
        "",
        "## Matched environmental-condition comparisons",
        "",
        "Effects are comparison condition minus `clear_noon`; p-values use a Holm correction.",
        "",
        "| Scenario | Controller | Condition | Paired n | Success risk difference | McNemar p (Holm) |",
        "|---|---|---|---:|---:|---:|",
      ]
    )
    for row in environment_comparisons:
      lines.append(
        "| "
        + " | ".join(
          [
            str(row["scenario_id"]),
            str(row["controller"]),
            str(row["comparison_condition"]),
            str(row["paired_n"]),
            _format(row["success_risk_difference_comparison_minus_reference"]),
            _format(row["mcnemar_exact_p_holm"]),
          ]
        )
        + " |"
      )
  lines.extend(
    [
      "",
      "## Interpretation guardrails",
      "",
      "- The independent unit is a complete run, not an individual telemetry frame.",
      "- CARLA Traffic Manager is a privileged-state reference, not a sensor-equivalent competitor.",
      "- Pedestrian, traffic-light, stationary-obstacle, and close cut-in cases are boundary tests outside OpenPilot's declared operating claims.",
      "- Review `runs.csv`, telemetry, events, and CARLA recordings before assigning a failure cause.",
      "",
    ]
  )
  path.write_text("\n".join(lines), encoding="utf-8")


def _write_success_plot(path: Path, aggregates: Sequence[dict[str, Any]]) -> bool:
  try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
  except ImportError:
    return False
  valid = [row for row in aggregates if row.get("valid_for_research")]
  plot_rows = valid or list(aggregates)
  collapsed: dict[tuple[str, str], tuple[int, int]] = {}
  for row in plot_rows:
    cell = " | ".join(
      (
        str(row["experiment"]),
        str(row["backend"]),
        str(row["scenario_id"]),
        str(row["condition_id"]),
      )
    )
    key = (cell, str(row["controller"]))
    previous = collapsed.get(key, (0, 0))
    collapsed[key] = (previous[0] + int(row["successes"]), previous[1] + int(row["n"]))
  cells = sorted({key[0] for key in collapsed})
  controllers = sorted({key[1] for key in collapsed})
  if not cells or not controllers:
    return False
  width = 0.8 / len(controllers)
  figure, axis = plt.subplots(figsize=(max(10, len(cells) * 1.35), 5.5))
  for controller_index, controller in enumerate(controllers):
    values: list[float] = []
    positions: list[float] = []
    for cell_index, cell in enumerate(cells):
      counts = collapsed.get((cell, controller))
      values.append(counts[0] / counts[1] if counts and counts[1] else math.nan)
      positions.append(cell_index - 0.4 + width / 2 + controller_index * width)
    axis.bar(positions, values, width=width, label=controller)
  axis.set_ylim(0, 1.05)
  axis.set_ylabel("Intervention-free success rate")
  axis.set_xticks(range(len(cells)), cells, rotation=35, ha="right")
  axis.grid(axis="y", alpha=0.25)
  axis.legend()
  figure.tight_layout()
  path.parent.mkdir(parents=True, exist_ok=True)
  figure.savefig(path, dpi=160)
  plt.close(figure)
  return True


def analyze_results(results_root: str | Path, output_dir: str | Path) -> dict[str, Any]:
  rows = load_run_rows(results_root)
  if not rows:
    raise ValueError(f"No summary.json files found below {Path(results_root).resolve()}")
  aggregates = aggregate_rows(rows)
  comparisons = paired_comparisons(rows)
  environment_comparisons = environmental_comparisons(rows)
  attempt_rows = load_attempt_rows(results_root)
  output = Path(output_dir).expanduser().resolve()
  output.mkdir(parents=True, exist_ok=True)
  _write_csv(output / "runs.csv", rows)
  _write_csv(output / "aggregate.csv", aggregates)
  _write_csv(output / "paired_comparisons.csv", comparisons)
  _write_csv(output / "environmental_comparisons.csv", environment_comparisons)
  _write_csv(output / "attempt_history.csv", attempt_rows)
  _write_report(
    output / "report.md",
    rows,
    aggregates,
    comparisons,
    environment_comparisons,
    attempt_rows,
  )
  plotted = _write_success_plot(output / "success_rates.png", aggregates)
  manifest = {
    "run_count": len(rows),
    "aggregate_cell_count": len(aggregates),
    "paired_comparison_count": len(comparisons),
    "environmental_comparison_count": len(environment_comparisons),
    "research_valid_run_count": sum(row["valid_for_research"] for row in rows),
    "non_research_valid_artifact_count": sum(not row["valid_for_research"] for row in rows),
    "archived_attempt_count": len(attempt_rows),
    "archived_invalid_attempt_count": sum(
      row.get("research_valid") is False for row in attempt_rows
    ),
    "plot_created": plotted,
    "output_dir": str(output),
  }
  (output / "analysis_manifest.json").write_text(
    json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
  )
  return manifest
