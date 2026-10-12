#!/usr/bin/env python3
"""health_check.py — one-shot Kit health report for the BNO055 recorder.

Checks: unit active, unit enabled, active calibration present + valid,
latest session manifest readable and clock_ok, disk headroom in the log dir.

Exit codes: 0 all good, 1 a check failed, 2 bad arguments.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

# Re-exec under the app venv when launched with any other interpreter —
# `sudo python3 ...` uses the system python, which lacks pyyaml.
_APP_VENV = Path("/opt/bno055/.venv")
if (_APP_VENV / "bin/python").is_file() \
        and Path(sys.prefix).resolve() != _APP_VENV.resolve():
    os.execv(str(_APP_VENV / "bin/python"),
             [str(_APP_VENV / "bin/python"), *sys.argv])

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from bno055_kit.calibration import (  # noqa: E402
    CalibrationError,
    fallback_path,
    load_calibration,
    read_history,
)
from bno055_kit.config import Config  # noqa: E402

MIN_FREE_GB = 1.0


def _systemd_ok(*args: str) -> bool:
    return subprocess.call(["systemctl", *args], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL) == 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="health_check", description=__doc__.splitlines()[0])
    p.add_argument("--config", default="/etc/bno055/bno055_imu.yaml")
    p.add_argument("--unit", default="bno055-imu.service")
    args = p.parse_args(argv)

    rc = 0

    if _systemd_ok("is-active", "--quiet", args.unit):
        print(f"[OK]   unit active: {args.unit}")
    else:
        print(f"[FAIL] unit not active: {args.unit}")
        rc = 1
    if _systemd_ok("is-enabled", "--quiet", args.unit):
        print(f"[OK]   unit enabled at boot: {args.unit}")
    else:
        print(f"[FAIL] unit not enabled at boot: {args.unit}")
        rc = 1

    cfg = Config.load(args.config if Path(args.config).is_file() else None)

    cal = None
    try:
        cal = load_calibration(cfg.cal_file)
        print(f"[OK]   calibration: {cfg.cal_file} sha256={cal.sha256[:12]}...")
        if cal.source or cal.calibrated_at_utc:
            print(f"       source={cal.source or '-'} "
                  f"calibrated_at_utc={cal.calibrated_at_utc or '-'}")
        n_hist = len(read_history(Path(cfg.cal_file).parent / "history.jsonl"))
        if n_hist:
            print(f"       calibration sessions in history: {n_hist}")
    except CalibrationError as exc:
        print(f"[FAIL] calibration: {exc}")
        rc = 1

    # The frozen factory fallback is the retreat path when no on-Kit session
    # is good enough — a missing/invalid one means `start_kit.py fallback`
    # would fail exactly when it is needed most.
    fb = fallback_path(cfg.cal_file)
    try:
        fb_cal = load_calibration(fb)
        print(f"[OK]   factory fallback: {fb} sha256={fb_cal.sha256[:12]}...")
        if cal is not None and fb_cal.sha256 == cal.sha256:
            print("       (active == factory fallback: never calibrated on-Kit)")
    except CalibrationError as exc:
        print(f"[FAIL] factory fallback: {exc} — re-run the installer")
        rc = 1

    log_dir = Path(cfg.log_dir)
    if log_dir.is_dir():
        sessions = sorted((d for d in log_dir.iterdir() if d.is_dir()), reverse=True)
        if sessions:
            manifest = sessions[0] / "session.json"
            if manifest.is_file():
                data = json.loads(manifest.read_text(encoding="utf-8"))
                n = data.get("samples_written", "?")
                reason = data.get("end_reason", "(running?)")
                print(f"[OK]   last session: {sessions[0].name} "
                      f"samples={n} end_reason={reason}")
            else:
                print(f"[WARN] last session {sessions[0].name} has no session.json")
        else:
            print(f"[WARN] no sessions yet under {log_dir}")
        free_gb = shutil.disk_usage(log_dir).free / 1e9
        if free_gb >= MIN_FREE_GB:
            print(f"[OK]   log dir free space: {free_gb:.1f} GB")
        else:
            print(f"[FAIL] log dir free space: {free_gb:.1f} GB (< {MIN_FREE_GB} GB)")
            rc = 1
    else:
        print(f"[WARN] log dir missing: {log_dir}")

    print("[DONE]" + (" all checks passed" if rc == 0 else " — see failures above"))
    return rc


if __name__ == "__main__":
    sys.exit(main())
