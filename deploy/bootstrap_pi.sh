#!/usr/bin/env bash
set -euo pipefail

# Repo root is parent of deploy/
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${REPO_ROOT}/.venv"
REQ_FILE="${REPO_ROOT}/requirements.txt"

echo "Repo root: ${REPO_ROOT}"

# Ensure python3 exists
if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 not found. Install Python 3 first."
  exit 1
fi

# Install required system packages (safe to run multiple times)
if command -v apt-get >/dev/null 2>&1; then
  echo "Installing system dependencies (python3-venv, python3-pip)..."
  sudo apt-get update
  sudo apt-get install -y python3-venv python3-pip
else
  echo "WARNING: apt-get not found; skipping system package install."
fi

# Create venv if missing
if [[ ! -d "${VENV_DIR}" ]]; then
  echo "Creating virtual environment at ${VENV_DIR}..."
  python3 -m venv "${VENV_DIR}"
else
  echo "Virtual environment already exists at ${VENV_DIR}."
fi

# Activate venv
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

echo "Upgrading pip..."
python -m pip install --upgrade pip

if [[ ! -f "${REQ_FILE}" ]]; then
  echo "ERROR: requirements.txt not found at ${REQ_FILE}"
  exit 1
fi

echo "Installing Python dependencies from requirements.txt..."
pip install -r "${REQ_FILE}"

# Basic sanity check: imports only (does not run hardware logic)
echo "Running import sanity check..."
python - <<'PY'
import importlib
import sys
mods = [
    "pi_runtime",
    "pi_runtime.Driver_Class",
    "pi_runtime.Encoder_Class",
    "pi_runtime.Event_Class",
    "pi_runtime.Tower_Class",
]
failed = []
for m in mods:
    try:
        importlib.import_module(m)
    except Exception as e:
        failed.append((m, str(e)))
if failed:
    print("Import check FAILED:")
    for m, e in failed:
        print(f"  - {m}: {e}")
    sys.exit(1)
print("Import check OK.")
PY

echo
echo "Bootstrap complete."
echo "Next:"
echo "  1) (Optional) Run manually: source .venv/bin/activate && python -m pi_runtime.main"
echo "  2) Install service: sudo ./deploy/install_service.sh"
