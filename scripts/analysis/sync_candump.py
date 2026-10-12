#!/usr/bin/env python3
"""sync_candump.py — align a BNO055 session with a candump -L capture.

Usage:
    sync_candump.py <session_dir> <candump.log> [--out sync.json]
                    [--method auto|correlation|anchor]
                    [--fs 20] [--max-lag 5] [--json]

Writes a sync manifest (default: <session_dir>/sync.json) describing the
constant clock offset between the IMU session timeline and the candump
wall-clock timeline, plus the residual jitter — the companion anchor to the
flight recorder's sync.json for offline correlation.

Exit codes: 0 ok, 1 analysis failed, 2 bad arguments.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

# Re-exec under the app venv when launched with any other interpreter —
# `sudo python3 ...` on the Kit uses the system python, which lacks numpy.
# On an analysis host the venv path does not exist and this is a no-op.
_APP_VENV = Path("/opt/bno055/.venv")
if (_APP_VENV / "bin/python").is_file() \
        and Path(sys.prefix).resolve() != _APP_VENV.resolve():
    os.execv(str(_APP_VENV / "bin/python"),
             [str(_APP_VENV / "bin/python"), *sys.argv])

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from bno055_kit.analysis import align  # noqa: E402
from bno055_kit.analysis.sync import load_can_series, load_imu_session  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="sync_candump", description=__doc__.splitlines()[0])
    p.add_argument("session_dir", help="BNO055 session dir (session.json + imu.jsonl)")
    p.add_argument("candump_log", help="candump -L capture file")
    p.add_argument("--out", default=None, help="output manifest path "
                                              "(default <session_dir>/sync.json)")
    p.add_argument("--method", choices=("auto", "correlation", "anchor"), default="auto")
    p.add_argument("--fs", type=float, default=20.0, help="correlation grid Hz")
    p.add_argument("--max-lag", type=float, default=5.0, help="lag search window +/- s")
    p.add_argument("--json", action="store_true", help="print result as JSON only")
    args = p.parse_args(argv)

    session_dir = Path(args.session_dir)
    candump_log = Path(args.candump_log)
    if not session_dir.is_dir():
        print(f"[FAIL] not a directory: {session_dir}", file=sys.stderr)
        return 2
    if not candump_log.is_file():
        print(f"[FAIL] not a file: {candump_log}", file=sys.stderr)
        return 2

    try:
        imu = load_imu_session(session_dir)
        can = load_can_series(candump_log)
        result = align(imu, can, method=args.method, fs=args.fs, max_lag_s=args.max_lag)
    except ValueError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1

    manifest = {
        "schema_version": 1,
        "imu_session": str(session_dir),
        "imu_session_id": imu.session_id,
        "candump_log": str(candump_log),
        "can_frames": can.n_frames,
        **result.to_dict(),
    }
    out = Path(args.out) if args.out else session_dir / "sync.json"
    out.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    if args.json:
        print(json.dumps(manifest, indent=2))
    else:
        print(f"[OK] {out}")
        print(f"     method     : {result.method} ({result.confidence} confidence)")
        print(f"     offset     : {result.offset_s * 1e3:+.3f} ms")
        print(f"     jitter std : {result.jitter_std_s * 1e3:.3f} ms")
        print(f"     max |res|  : {result.max_abs_residual_s * 1e3:.3f} ms")
        print(f"     n          : {result.n}")
        for note in result.notes:
            print(f"     note       : {note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
