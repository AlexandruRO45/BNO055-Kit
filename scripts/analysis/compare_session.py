#!/usr/bin/env python3
"""compare_session.py — sync + side-by-side compare a BNO session vs a CAN capture.

The capture is any candump -L log carrying the IMU frames; it need not come
from a VISAN airframe — point it at any host's capture and it works.

Usage:
    compare_session.py <bno_session_dir> <capture_dir>
                       [--out-dir DIR] [--euler-convention none|ned_to_nwu]
                       [--accel-convention raw|reconstruct_gravity]
                       [--gyro-convention none|negate|rot_x180]
                       [--method auto|correlation|anchor] [--fs 20] [--max-lag 5]
                       [--window LO:HI]  # seconds, relative to overlap start

Runs the clock sync (sync_candump), then compares every BNO signal against
the matching DBC-decoded CAN signal at each aligned timestamp. Writes the
per-session analysis bundle to <capture_dir>/analysis/ by default:

    sync.json            clock offset + jitter (same schema as sync_candump)
    compare.jsonl        one row per sample: t + {signal: {bno, can, delta}}
    compare_summary.json per-signal stats + the conventions/offsets used

Exit codes: 0 ok, 1 analysis failed, 2 bad arguments.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

# Re-exec under the app venv when launched with any other interpreter.
_APP_VENV = Path("/opt/bno055/.venv")
if (_APP_VENV / "bin/python").is_file() \
        and Path(sys.prefix).resolve() != _APP_VENV.resolve():
    os.execv(str(_APP_VENV / "bin/python"),
             [str(_APP_VENV / "bin/python"), *sys.argv])

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from bno055_kit.analysis.compare import (  # noqa: E402
    compare as compare_signals,
)
from bno055_kit.analysis.compare import (  # noqa: E402
    load_can_signals,
    load_imu_signals,
)
from bno055_kit.analysis.sync import (  # noqa: E402
    align,
    load_can_series,
    load_imu_session,
)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="compare_session",
                                description=__doc__.splitlines()[0])
    p.add_argument("bno_dir", help="BNO055 session dir (session.json + imu.jsonl)")
    p.add_argument("capture_dir", help="CAN capture session dir (holds can.candump)")
    p.add_argument("--out-dir", default=None,
                   help="bundle dir (default <capture_dir>/analysis)")
    p.add_argument("--candump", default=None,
                   help="candump log (default <capture_dir>/can.candump)")
    p.add_argument("--method", choices=("auto", "correlation", "anchor"), default="auto")
    p.add_argument("--fs", type=float, default=20.0)
    p.add_argument("--max-lag", type=float, default=5.0)
    p.add_argument("--euler-convention", choices=("none", "ned_to_nwu"), default="none")
    p.add_argument("--accel-convention", choices=("raw", "reconstruct_gravity"),
                   default="raw")
    p.add_argument("--gyro-convention", choices=("none", "negate", "rot_x180"),
                   default="none")
    p.add_argument("--window", default=None,
                   help="compare only this time window, LO:HI seconds from overlap start")
    args = p.parse_args(argv)

    bno_dir = Path(args.bno_dir)
    capture_dir = Path(args.capture_dir)
    if not bno_dir.is_dir():
        print(f"[FAIL] not a directory: {bno_dir}", file=sys.stderr)
        return 2
    candump = Path(args.candump) if args.candump else capture_dir / "can.candump"
    if not candump.is_file():
        print(f"[FAIL] not a file: {candump}", file=sys.stderr)
        return 2

    out_dir = Path(args.out_dir) if args.out_dir else capture_dir / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    window = None
    if args.window:
        try:
            lo, hi = (float(x) for x in args.window.split(":"))
        except ValueError:
            print("[FAIL] --window must be LO:HI seconds", file=sys.stderr)
            return 2
        window = (lo, hi)

    # 1. sync ---------------------------------------------------------------
    try:
        imu_sync = load_imu_session(bno_dir)
        can_sync = load_can_series(candump)
        sync_res = align(imu_sync, can_sync, method=args.method,
                         fs=args.fs, max_lag_s=args.max_lag)
    except ValueError as exc:
        print(f"[FAIL] sync: {exc}", file=sys.stderr)
        return 1

    sync_manifest = {
        "schema_version": 1,
        "imu_session": str(bno_dir),
        "imu_session_id": imu_sync.session_id,
        "candump_log": str(candump),
        "can_frames": can_sync.n_frames,
        **sync_res.to_dict(),
    }
    (out_dir / "sync.json").write_text(
        json.dumps(sync_manifest, indent=2) + "\n", encoding="utf-8")

    # 2. compare ------------------------------------------------------------
    try:
        imu = load_imu_signals(bno_dir)
        can = load_can_signals(candump)
        result, rows = compare_signals(
            imu, can, sync_res.offset_s,
            method=sync_res.method, confidence=sync_res.confidence,
            euler_convention=args.euler_convention,
            accel_convention=args.accel_convention,
            gyro_convention=args.gyro_convention,
            window_s=window)
    except ValueError as exc:
        print(f"[FAIL] compare: {exc}", file=sys.stderr)
        return 1

    with open(out_dir / "compare.jsonl", "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")

    summary = result.to_summary_dict()
    summary["imu_session"] = str(bno_dir)
    summary["capture_session"] = str(capture_dir)
    summary["candump_log"] = str(candump)
    summary["can_frames"] = can.n_frames
    summary["imu_n_bad_lines"] = imu.n_bad_lines
    (out_dir / "compare_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    # 3. report -------------------------------------------------------------
    print(f"[OK] {out_dir}")
    print(f"     sync      : {sync_res.method} ({sync_res.confidence}), "
          f"offset {sync_res.offset_s * 1e3:+.3f} ms")
    print(f"     conventions: {result.conventions}")
    print(f"     signals   : {len(result.summaries)} compared, {len(rows):,} rows")
    for s in result.summaries:
        print(f"       {s.bno_key:<9} vs {s.can_signal:<14} "
              f"rmse={s.rmse:8.4f} max|d|={s.max_abs_delta:8.4f} "
              f"mean={s.mean_delta:+8.4f} {s.unit}  n={s.n:,}")
    for note in result.notes:
        print(f"     note      : {note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
