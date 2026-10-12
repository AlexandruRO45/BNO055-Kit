#!/bin/sh
# BNO055 kit systemd wrapper. Started by bno055-imu.service at boot;
# stopped via SIGINT (KillSignal) on unit stop.
#
# Thin exec-wrapper so the unit stays declarative: config path and log
# dir come from environment with defaults, and `exec` replaces the shell
# with the daemon so systemd signals land directly on the Python process
set -e

APP_ROOT="${BNO055_KIT_ROOT:-/opt/bno055}"
CONFIG="${BNO055_KIT_CONFIG:-/etc/bno055/bno055_imu.yaml}"
PYTHON="${APP_ROOT}/.venv/bin/python"

if [ ! -x "${PYTHON}" ]; then
    echo "bno055-imu: venv python not found at ${PYTHON}" >&2
    exit 78
fi

exec "${PYTHON}" -u -m bno055_kit.cli --config "${CONFIG}" run \
    --log-dir "${BNO055_KIT_LOG_DIR:-/var/log/bno055}"
