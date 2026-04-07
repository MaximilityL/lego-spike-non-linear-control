#!/usr/bin/env bash
# BootstrapEnv.sh
#
# One shot setup helper for the desktop side of the LegoBalance project.
# Creates a virtual environment in .venv, installs the package in editable
# mode with the dev extras, and runs the unit tests.
#
# Safe to re run. Idempotent.
#
# Usage:
#     bash scripts/BootstrapEnv.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${REPO_ROOT}/.venv"

echo "[BootstrapEnv] Repo root: ${REPO_ROOT}"

if [[ ! -d "${VENV_DIR}" ]]; then
    echo "[BootstrapEnv] Creating virtual environment at ${VENV_DIR}"
    python3 -m venv "${VENV_DIR}"
else
    echo "[BootstrapEnv] Virtual environment already exists, reusing"
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

echo "[BootstrapEnv] Upgrading pip"
python -m pip install --upgrade pip

echo "[BootstrapEnv] Installing project in editable mode with dev extras"
pip install -e "${REPO_ROOT}[dev]"

echo "[BootstrapEnv] Running unit tests"
pytest "${REPO_ROOT}/tests"

cat <<'EOF'

[BootstrapEnv] Done.

Activate the virtual environment in new shells with:
    source .venv/bin/activate

Common next steps:
    python examples/ClosedLoopSimulation.py
    python scripts/RunDiagnostics.py
    python scripts/DetectHub.py

EOF
