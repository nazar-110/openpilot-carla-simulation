#!/usr/bin/env bash
set -euo pipefail

OPENPILOT_DIR="${1:-$HOME/openpilot}"
MODE="${2:-apply}"
TESTED_COMMIT="4df40d2c1946a57242230186edd073c4073060a6"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PATCH_FILE="$PROJECT_ROOT/patches/openpilot-v0.11.1-wsl-discrete-gpu.patch"
DEVICE_CONFIG="$OPENPILOT_DIR/selfdrive/modeld/models/tg_input_devices.json"

if [[ "$MODE" != "apply" && "$MODE" != "--check" ]]; then
  echo "usage: $0 [openpilot-directory] [apply|--check]" >&2
  exit 2
fi
if [[ ! -d "$OPENPILOT_DIR/.git" ]]; then
  echo "OpenPilot checkout not found at $OPENPILOT_DIR" >&2
  exit 2
fi
if [[ "$(git -C "$OPENPILOT_DIR" rev-parse HEAD)" != "$TESTED_COMMIT" ]]; then
  echo "OpenPilot must be at tested commit $TESTED_COMMIT" >&2
  exit 2
fi

if git -C "$OPENPILOT_DIR" apply --reverse --check "$PATCH_FILE" 2>/dev/null; then
  patch_applied=true
else
  patch_applied=false
fi

if [[ "$MODE" == "--check" ]]; then
  if [[ "$patch_applied" != true ]]; then
    echo "The required discrete-GPU OpenPilot compatibility patch is not applied." >&2
    echo "Run: bash '$0' '$OPENPILOT_DIR'" >&2
    exit 1
  fi
  if [[ ! -f "$DEVICE_CONFIG" ]] \
    || ! grep -q '"WARP_DEV": "CUDA"' "$DEVICE_CONFIG" \
    || ! grep -q '"QUEUE_DEV": "CUDA"' "$DEVICE_CONFIG" \
    || ! grep -q '"DEV": "CUDA"' "$DEVICE_CONFIG"; then
    echo "OpenPilot's CUDA device configuration is missing or incomplete." >&2
    echo "Run: bash '$0' '$OPENPILOT_DIR'" >&2
    exit 1
  fi
  echo "OpenPilot WSL discrete-GPU compatibility is configured."
  exit 0
fi

if [[ "$patch_applied" != true ]]; then
  if ! git -C "$OPENPILOT_DIR" apply --check "$PATCH_FILE"; then
    echo "Cannot apply the compatibility patch cleanly. Preserve your changes and restore modeld.py first." >&2
    exit 1
  fi
  git -C "$OPENPILOT_DIR" apply "$PATCH_FILE"
  echo "Applied the OpenPilot discrete-GPU compatibility patch."
else
  echo "OpenPilot discrete-GPU compatibility patch is already applied."
fi

mkdir -p "$(dirname "$DEVICE_CONFIG")"
cat > "$DEVICE_CONFIG" <<'JSON'
{"selfdrive.modeld.modeld": {"default": {"WARP_DEV": "CUDA", "QUEUE_DEV": "CUDA"}}, "selfdrive.modeld.dmonitoringmodeld": {"default": {"DEV": "CUDA"}}}
JSON

echo "Configured modeld and dmonitoringmodeld for the WSL CUDA device."
echo "Note: the patched OpenPilot checkout is intentionally dirty; exploratory artifacts are not research-valid."
