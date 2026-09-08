#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXPERIMENT="${1:-$PROJECT_ROOT/experiments/urban_suite.yaml}"
RUNTIME_CONFIG="${2:-$PROJECT_ROOT/config/carla.yaml}"

: "${OPENPILOT_ROOT:?Set OPENPILOT_ROOT to the pinned OpenPilot v0.11.1 checkout}"

opencarla-eval doctor \
  --runtime-config "$RUNTIME_CONFIG" \
  --openpilot-root "$OPENPILOT_ROOT" \
  --connect

# The evaluator starts and stops OpenPilot manager/modeld inside each trial.
# Two consecutive trials exercise VisionIPC teardown and clean reconnection.
opencarla-eval run \
  --experiment "$EXPERIMENT" \
  --runtime-config "$RUNTIME_CONFIG" \
  --backend carla \
  --controller openpilot \
  --limit 2 \
  --overwrite
