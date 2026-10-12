#!/usr/bin/env bash
# install_kit.sh — install the BNO055 kit on the Kit (Jetson Orin, JetPack 6).
#
# Installs to /opt/bno055 with its OWN venv, seeds the shipped calibration into
# /var/lib/bno055, installs the systemd unit, and ENABLES it at boot.
# Like the deb postinst, it deliberately does NOT start the service
# during install — the boot-time start is the tested path.
#
# The daemon runs as a dedicated service user/group. There is no built-in
# default: pass --user/--group, or the installer auto-detects the sudo caller
# (SUDO_USER); when neither is available it prompts you to type them. The
# unit is templated with the chosen user/group at install time.
#
# Usage:
#   install_kit.sh [--confirm] [--src DIR] [--no-enable]
#                  [--user USER] [--group GROUP]
#
#   --confirm     actually apply changes (default is DRY-RUN)
#   --src DIR     kit source tree (default: this script's ../..)
#   --no-enable   install files but do not systemctl enable
#   --user USER   service user (no default; taken from sudo/SUDO_USER when
#                 omitted, prompted interactively when not auto-detectable)
#   --group GROUP service group (default: same as the resolved --user)
#
# Exit codes: 0 success, 1 stage failure, 2 bad arguments.
set -euo pipefail

SRC_DIR=""
CONFIRM=0
ENABLE=1
SERVICE_USER=""
SERVICE_GROUP=""
DRY_PREFIX="[DRY-RUN] "

usage() { sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        --confirm)  CONFIRM=1; DRY_PREFIX=""; shift ;;
        --src)      SRC_DIR="${2:?--src needs a directory}"; shift 2 ;;
        --user)     SERVICE_USER="${2:?--user needs a name}"; shift 2 ;;
        --group)    SERVICE_GROUP="${2:?--group needs a name}"; shift 2 ;;
        --no-enable) ENABLE=0; shift ;;
        -h|--help)  usage; exit 0 ;;
        *)          echo "[FAIL] unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

[ -n "$SRC_DIR" ] || SRC_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
# No default user/group: an explicit --user wins; else auto-detect the sudo
# caller (SUDO_USER); else prompt in real time. Group defaults to the user.
if [ -z "$SERVICE_USER" ]; then
    if [ -n "${SUDO_USER:-}" ]; then
        SERVICE_USER="$SUDO_USER"
        echo "[auto] service user/group taken from sudo: $SERVICE_USER"
    else
        printf 'service user (required, no default): '
        read -r SERVICE_USER
    fi
fi
[ -n "$SERVICE_USER" ] \
    || { echo "[FAIL] no service user: pass --user or run under sudo" >&2; exit 2; }
[ -n "$SERVICE_GROUP" ] || SERVICE_GROUP="$SERVICE_USER"

APP_ROOT="/opt/bno055"
SERVICE="bno055-imu.service"
UNIT_SRC="${SRC_DIR}/scripts/deploy/systemd/${SERVICE}"
WRAPPER_SRC="${SRC_DIR}/scripts/deploy/systemd/bno055_imu_wrapper.sh"
CAL_SRC="${SRC_DIR}/calibs/active.json"
FALLBACK_SRC="${SRC_DIR}/calibs/factory_fallback.json"
FALLBACK_DST="/var/lib/bno055/factory_fallback.json"
CONFIG_SRC="${SRC_DIR}/configs/bno055_imu.yaml"

# Apply runs tee the whole session to a UTC-stamped log (ops-script
# convention); dry-runs stay silent so they make no filesystem changes.i
LOG_FILE=""

if [ "$CONFIRM" -eq 1 ]; then
    LOG_FILE="/tmp/install_kit_$(date -u +%Y%m%dT%H%M%SZ).log"
    exec > >(tee -a "$LOG_FILE") 2>&1
else
    LOG_FILE="(none — dry-run)"
fi

stage() { echo ""; echo "=== [STAGE] $* ==="; }
pass()  { echo "[KIT OK] $*"; }
fail()  { echo "[KIT FAIL: $1] $2" >&2; exit 1; }

run() {
    if [ "$CONFIRM" -eq 1 ]; then
        "$@"
    else
        echo "${DRY_PREFIX}$*"
    fi
}

echo "BNO055 kit install — $(date -u +%Y-%m-%dT%H:%M:%SZ) UTC"
echo "source : ${SRC_DIR}"
echo "mode   : $([ "$CONFIRM" -eq 1 ] && echo APPLY || echo DRY-RUN)"
echo "log    : ${LOG_FILE}"

# ------------------------------------------------------------------ preflight
stage "preflight"
[ "$(id -u)" -eq 0 ] || fail preflight "must run as root (sudo bash install_kit.sh)"
[ -d "$SRC_DIR/src/bno055_kit" ] || fail preflight "kit source tree not found at $SRC_DIR"
[ -f "$UNIT_SRC" ]     || fail preflight "missing unit: $UNIT_SRC"
[ -f "$WRAPPER_SRC" ]   || fail preflight "missing wrapper: $WRAPPER_SRC"
[ -f "$CAL_SRC" ]       || fail preflight "missing calibration: $CAL_SRC"
[ -f "$FALLBACK_SRC" ]  || fail preflight "missing factory fallback calibration: $FALLBACK_SRC"
[ -f "$CONFIG_SRC" ]    || fail preflight "missing config: $CONFIG_SRC"
PYTHON_BIN="$(command -v python3)" || fail preflight "python3 not found"
PYVER="$("$PYTHON_BIN" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
case "$PYVER" in
    3.10|3.11|3.12) pass "python $PYVER" ;;
    *) fail preflight "python $PYVER outside supported range 3.10-3.12" ;;
esac
id -u "$SERVICE_USER" >/dev/null 2>&1 \
    || fail preflight "user '$SERVICE_USER' not found — pass --user/--group or create it first"
command -v rsync >/dev/null 2>&1 \
    || fail preflight "rsync not found — install it with: sudo apt-get install rsync"
pass "preflight"

# --------------------------------------------------------------------- install
stage "install tree -> ${APP_ROOT}"
run install -d -o "$SERVICE_USER" -g "$SERVICE_GROUP" "$APP_ROOT"
# --delete keeps the tree clean but MUST NOT touch the venv or the editable
# install's egg-info — wiping .venv would force a full network reinstall.
run rsync -a --delete --exclude __pycache__ --exclude '*.py[co]' \
    --exclude .venv --exclude '*.egg-info' \
    --chown "$SERVICE_USER:$SERVICE_GROUP" "${SRC_DIR}/src" "${SRC_DIR}/scripts" \
    "${SRC_DIR}/configs" "${SRC_DIR}/pyproject.toml" "${APP_ROOT}/"
pass "tree installed"

# ----------------------------------------------------------------------- venv
stage "venv ${APP_ROOT}/.venv"
if [ -x "${APP_ROOT}/.venv/bin/python" ]; then
    pass "venv already present — refreshing deps"
else
    run sudo -u "$SERVICE_USER" "$PYTHON_BIN" -m venv "${APP_ROOT}/.venv"
fi
run sudo -u "$SERVICE_USER" "${APP_ROOT}/.venv/bin/pip" install -e "${APP_ROOT}"
# blinka's Jetson backend needs the Jetson.GPIO udev rules so the
# non-root service user can touch the GPIO sysfs/chip nodes it probes.
# Best-effort: I2C itself only needs the i2c group (unit
# SupplementaryGroups), so a missing rule file must not break install.
GPIO_RULE="${APP_ROOT}/.venv/lib/python3.10/site-packages/Jetson/GPIO/99-gpio.rules"
if [ -f "$GPIO_RULE" ]; then
    run groupadd -f -r gpio
    run install -m 0644 "$GPIO_RULE" /etc/udev/rules.d/99-gpio.rules
    run udevadm control --reload-rules
    run udevadm trigger
    pass "Jetson.GPIO udev rules installed"
else
    pass "Jetson.GPIO udev rules not present (skipped)"
fi
pass "venv"

# ------------------------------------------------------------- state + config
stage "state + config"
run install -d -o "$SERVICE_USER" -g "$SERVICE_GROUP" /var/lib/bno055 /var/log/bno055 /run/bno055
# The factory fallback is the permanent known-good calibration shipped
# with the Kit. It is (re)installed on EVERY install, owned root:root mode
# 0444 — read-only for the service user the daemon and the calibration
# flow run as, so nothing but a root re-install can ever change or delete
# it. It is what `start_kit.py fallback` restores active.json from when no
# on-Kit session is good enough.
run install -o root -g root -m 0444 "$FALLBACK_SRC" "$FALLBACK_DST"
pass "factory fallback calibration installed (frozen): $FALLBACK_DST"
# The ACTIVE calibration is the mutable working copy the daemon loads and
# the on-Kit calibration flow overwrites. Seeded ONLY if absent — never
# overwrite a newer on-Kit/bench result.
if [ -f /var/lib/bno055/active.json ]; then
    pass "keeping existing /var/lib/bno055/active.json (not overwriting)"
else
    run install -o "$SERVICE_USER" -g "$SERVICE_GROUP" -m 0644 "$CAL_SRC" /var/lib/bno055/active.json
    pass "active calibration seeded from $CAL_SRC"
fi
# Config: seed only when absent — an operator's tuned config survives a
# re-install (same keep-existing rule as the calibration above).
run install -d -m 0755 /etc/bno055
if [ -f /etc/bno055/bno055_imu.yaml ]; then
    pass "keeping existing /etc/bno055/bno055_imu.yaml (not overwriting)"
else
    run install -m 0644 "$CONFIG_SRC" /etc/bno055/bno055_imu.yaml
    pass "config seeded from $CONFIG_SRC"
fi
pass "state + config"

# ---------------------------------------------------------------------- unit
stage "systemd unit"
run install -m 0755 "$WRAPPER_SRC" /opt/bno055/scripts/deploy/systemd/bno055_imu_wrapper.sh
# Render the unit with the chosen service user/group (the shipped unit carries
# @SERVICE_USER@/@SERVICE_GROUP@ placeholders; they are substituted here).
UNIT_RENDERED="$(mktemp)"
sed -e "s/^User=.*/User=${SERVICE_USER}/" \
    -e "s/^Group=.*/Group=${SERVICE_GROUP}/" "$UNIT_SRC" > "$UNIT_RENDERED"
run install -m 0644 "$UNIT_RENDERED" "/lib/systemd/system/${SERVICE}"
if [ "$CONFIRM" -eq 1 ]; then rm -f "$UNIT_RENDERED"; fi
run systemctl daemon-reload
if [ "$ENABLE" -eq 1 ]; then
    run systemctl enable "$SERVICE"
    pass "unit installed and enabled (NOT started — boots with the Kit)"
else
    pass "unit installed (--no-enable: not enabled)"
fi

echo ""
echo "=== [INSTALL ${DRY_PREFIX:+DRY-RUN }OK] ${APP_ROOT} ==="
echo "next: sudo systemctl start ${SERVICE}   # or reboot to test the boot path"
echo "verify: systemctl status ${SERVICE} ; journalctl -u ${SERVICE} -n 50"
exit 0
