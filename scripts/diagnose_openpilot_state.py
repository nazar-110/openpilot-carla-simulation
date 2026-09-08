#!/usr/bin/env python3
"""Print changing OpenPilot engagement diagnostics during a simulator run."""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

from cereal import messaging


def _enum_name(value: Any) -> str:
  return str(value).rsplit(".", maxsplit=1)[-1]


def _snapshot(sm: Any) -> dict[str, Any]:
  selfdrive = sm["selfdriveState"]
  calibration = sm["liveCalibration"]
  pose = sm["livePose"]
  manager = sm["managerState"]
  return {
    "selfdrive": {
      "engageable": bool(selfdrive.engageable),
      "enabled": bool(selfdrive.enabled),
      "active": bool(selfdrive.active),
      "state": _enum_name(selfdrive.state),
      "alert_type": str(selfdrive.alertType),
      "alert_text_1": str(selfdrive.alertText1),
      "alert_text_2": str(selfdrive.alertText2),
    },
    "onroad_events": sorted(_enum_name(event.name) for event in sm["onroadEvents"]),
    "calibration": {
      "status": _enum_name(calibration.calStatus),
      "percent": int(calibration.calPerc),
      "valid_blocks": int(calibration.validBlocks),
    },
    "pose": {
      "valid": bool(sm.valid["livePose"]),
      "inputs_ok": bool(pose.inputsOK),
      "posenet_ok": bool(pose.posenetOK),
    },
    "not_running": sorted(
      process.name
      for process in manager.processes
      if process.shouldBeRunning and not process.running
    ),
    "service_checks": {
      service: {
        "alive": bool(sm.alive[service]),
        "valid": bool(sm.valid[service]),
        "freq_ok": bool(sm.freq_ok[service]),
      }
      for service in sm.services
    },
  }


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--duration", type=float, default=30.0)
  args = parser.parse_args()
  services = [
    "selfdriveState",
    "onroadEvents",
    "liveCalibration",
    "livePose",
    "managerState",
  ]
  sm = messaging.SubMaster(services)
  deadline = time.monotonic() + args.duration
  previous: dict[str, Any] | None = None
  while time.monotonic() < deadline:
    sm.update(500)
    if not sm.updated["selfdriveState"]:
      continue
    current = _snapshot(sm)
    if current != previous:
      print(json.dumps(current, sort_keys=True), flush=True)
      previous = current


if __name__ == "__main__":
  main()
