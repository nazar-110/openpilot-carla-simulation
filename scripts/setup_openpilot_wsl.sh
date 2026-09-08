#!/usr/bin/env bash
set -euo pipefail

OPENPILOT_DIR="${1:-$HOME/openpilot}"
TESTED_COMMIT="4df40d2c1946a57242230186edd073c4073060a6"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if ! grep -qi microsoft /proc/version; then
  echo "warning: this helper was designed for Ubuntu 24.04 under WSL2" >&2
fi

if ! command -v git >/dev/null || ! command -v git-lfs >/dev/null; then
  sudo apt update
  sudo apt install -y git git-lfs
fi
git lfs install

if [[ -e "$OPENPILOT_DIR" && ! -d "$OPENPILOT_DIR/.git" ]]; then
  echo "existing path is not an OpenPilot Git checkout: $OPENPILOT_DIR" >&2
  exit 2
fi
if [[ ! -d "$OPENPILOT_DIR/.git" ]]; then
  git clone https://github.com/commaai/openpilot.git "$OPENPILOT_DIR"
fi

cd "$OPENPILOT_DIR"
CURRENT_COMMIT="$(git rev-parse HEAD)"
if [[ "$CURRENT_COMMIT" != "$TESTED_COMMIT" ]]; then
  if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "refusing to switch commits while the existing OpenPilot checkout has tracked changes" >&2
    exit 2
  fi
  git checkout --detach "$TESTED_COMMIT"
fi
git submodule update --init --recursive
git lfs pull

tools/op.sh setup
source .venv/bin/activate
scons -u -j"$(nproc)"

bash "$PROJECT_ROOT/scripts/apply_openpilot_wsl_compat.sh" "$OPENPILOT_DIR"

UV_BIN="$(command -v uv || true)"
if [[ -z "$UV_BIN" && -x "$HOME/.local/bin/uv" ]]; then
  UV_BIN="$HOME/.local/bin/uv"
fi
if [[ -z "$UV_BIN" ]]; then
  echo "uv was not found after tools/op.sh setup" >&2
  exit 2
fi
"$UV_BIN" pip install --python "$OPENPILOT_DIR/.venv/bin/python" carla==0.9.16
"$UV_BIN" pip install --python "$OPENPILOT_DIR/.venv/bin/python" -e "$PROJECT_ROOT[analysis]"

echo
echo "OpenPilot and opencarla-eval are installed."
echo "Next: source '$OPENPILOT_DIR/.venv/bin/activate'"
echo "From Windows PowerShell, run: .\\scripts\\run_visible_openpilot.ps1"
