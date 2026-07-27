#!/bin/bash
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_ACTIVATE="${PROJECT_DIR}/venv/bin/activate"

if [ ! -f "${VENV_ACTIVATE}" ]; then
    echo "❌ Error: Virtual environment not found. Please run './build_drivers.sh' first."
    exit 1
fi

source "${VENV_ACTIVATE}"
export DYLD_LIBRARY_PATH="${PROJECT_DIR}/_libs/install/lib:$DYLD_LIBRARY_PATH"

exec python -u "${PROJECT_DIR}/kinect_osc_driver.py" "$@"
