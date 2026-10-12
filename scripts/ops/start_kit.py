#!/usr/bin/env python3
"""start_kit.py — manual start/stop/status helper for the BNO055 kit service.

For bench use; at boot the unit starts on its own (WantedBy=multi-user.target).

Usage:
    start_kit.py status|start|stop|restart|logs [--unit bno055-imu.service]
    start_kit.py calibrate [--unit ...] [extra cli calibrate args...]
    start_kit.py best [--unit ...] [--time 30]
    start_kit.py fallback [--unit ...]
    start_kit.py bench [--unit ...] [extra cli bench args...]

``calibrate`` is the on-Kit operator toggle: it stops the recorder, runs the
interactive calibration (sensor already glued to the drone), and ALWAYS
restarts the unit afterwards — including on Ctrl-C or failure — so the
drone never sits unrecorded. It only *saves* the session (cal_<ts>.json +
history.jsonl); it never touches the active calibration.
``best`` is the only promoter: it ranks the scored sessions in
history.jsonl to pick a candidate, then benches that session AND the factory
fallback live, back-to-back, under identical conditions, and overwrites the
active calibration with the session ONLY if its live score beats the
factory's — otherwise the factory cal stays active.
``fallback`` restores the factory calibration (the known-good calib shipped
with the Kit, never deleted) as the active calibration and restarts the
unit — the escape hatch when no on-Kit session is good enough.
``bench`` scores one calibration (default: the active one) at rest and
appends it to history.jsonl, without changing the active calibration.

Every bench re-converges first: figure-8 the board until mag=3, then hold
it PERFECTLY still for the window (mag decays to 0 while still, so only
accel=3 and a live, non-frozen heading are required during the window).

Exit codes: 0 ok, 1 command failed, 2 bad arguments.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

# Re-exec under the app venv when launched with any other interpreter:
# `sudo python3 ...` resets PATH to the *system* python, which lacks the
# kit's deps (pyyaml) — the venv is the only runtime that can import
# bno055_kit.config. sys.prefix (not the resolved executable, which is a
# symlink to the same base interpreter) identifies the running venv.
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
    pick_best,
    rank_and_compare,
    read_history,
    save_calibration,
)

ACTIONS = {
    "status": ["systemctl", "status", "{unit}", "--no-pager", "-n", "30"],
    "start": ["systemctl", "start", "{unit}"],
    "stop": ["systemctl", "stop", "{unit}"],
    "restart": ["systemctl", "restart", "{unit}"],
    "logs": ["journalctl", "-u", "{unit}", "--no-pager", "-n", "100", "--follow"],
}

DEFAULT_UNIT = "bno055-imu.service"
DEFAULT_CONFIG = "/etc/bno055/bno055_imu.yaml"

# Actions that stop/start the recorder unit: without root, polkit refuses
# the systemctl call mid-run ("Access denied"), which would leave `best`
# half-done (history ranked, factory never benched). Fail fast instead.
_PRIVILEGED_ACTIONS = {"calibrate", "best", "fallback", "bench", "start",
                       "stop", "restart"}


def _systemd(*args: str) -> int:
    cmd = ["systemctl", *args]
    print("$ " + " ".join(cmd), flush=True)
    return subprocess.call(cmd)


def _kit_cli(argv: list[str]) -> int:
    """Run the kit CLI in the app venv when present, else this interpreter."""
    venv_py = Path("/opt/bno055/.venv/bin/python")
    exe = str(venv_py) if venv_py.is_file() else sys.executable
    cmd = [exe, "-m", "bno055_kit.cli", *argv]
    print("$ " + " ".join(cmd), flush=True)
    return subprocess.call(cmd)


def cmd_calibrate(unit: str, config: str, extra: list[str]) -> int:
    """Stop the recorder, calibrate interactively, ALWAYS restart recording."""
    _systemd("stop", unit)
    rc = 1
    try:
        rc = _kit_cli(["--config", config, "calibrate", "--force", "--unit",
                       unit, *extra])
    except KeyboardInterrupt:
        print("\n[ABORT] calibration interrupted — nothing saved.")
        rc = 130
    finally:
        # Recording must resume even on abort/failure: an unrecorded drone
        # is the worse failure mode, and the boot path re-applies whatever
        # active.json holds.
        restart_rc = _systemd("start", unit)
        if rc == 0 and restart_rc != 0:
            print(f"[FAIL] calibration saved but {unit} did not start "
                  f"(systemctl start rc={restart_rc})")
            rc = 1
    if rc == 0:
        print(f"[OK]   calibration saved — {unit} restarted with the "
              "current active calibration (unchanged).")
        print("       Promote the best session with: sudo python3 "
              "scripts/ops/start_kit.py best")
    return rc


def _print_ranking(ranked: list[dict]) -> None:
    print(f"{'#':>2} {'score':>8} {'bias m/s2':>10} {'drift deg/min':>14} "
          f"{'noise deg':>10}  file")
    for i, r in enumerate(ranked, 1):
        print(f"{i:>2} {r['score']:>8} {r['bias_linacc_m_s2']:>10} "
              f"{r['drift_deg_per_min']:>14} {r['head_noise_deg']:>10}  "
              f"{r['cal_file']}")


def cmd_best(unit: str, config: str, bench_time: float) -> int:
    """Rank on-Kit sessions, then decide the active calibration by benching
    the top session and the factory fallback LIVE, back-to-back, under
    identical conditions — and promote the session only if it beats factory.

    Both candidates are re-benched now (not compared against a stale stored
    score) so the two scores are apples-to-apples: same settle, same thermal
    state, same minute. A historical score is only used to pick *which*
    session to challenge the factory with.
    """
    from bno055_kit.cli import BenchNotConverged, _run_bench  # shared bench
    from bno055_kit.config import Config

    cfg = Config.load(config if Path(config).is_file() else None)
    hist_path = Path(cfg.cal_file).parent / "history.jsonl"
    records = read_history(hist_path)
    ranked = sorted((r for r in records if r.get("score") is not None
                     and Path(r["cal_file"]).is_file()),
                    key=lambda r: r["score"])
    best = pick_best(records)

    if best is None:
        print(f"[!] no rankable on-Kit session: history at {hist_path} is "
              "empty or has no record with a score and a readable cal file.")
        print("    Run: sudo python3 scripts/ops/start_kit.py calibrate "
              "(answer the bench prompt to score a session),")
        print("    or keep the factory cal: sudo python3 "
              "scripts/ops/start_kit.py fallback")
        return 1

    print(f"On-Kit sessions ranked by stored score (candidate selection), "
          f"{bench_time:.0f}s bench window each:")
    _print_ranking(ranked)

    fallback = fallback_path(cfg.cal_file)
    if not fallback.is_file():
        print(f"[FAIL] factory fallback calibration missing: {fallback}")
        print("       re-run the installer: sudo bash "
              "scripts/deploy/install_kit.sh --confirm")
        return 1

    src = best["cal_file"]
    try:
        winner_cal = load_calibration(src)
        load_calibration(fallback)  # validate the baseline before touching it
    except CalibrationError as exc:
        print(f"[FAIL] {exc}")
        print("       re-run the installer: sudo bash "
              "scripts/deploy/install_kit.sh --confirm")
        return 1

    # Bench both live, back-to-back, sensor free. Any failure aborts the
    # promotion: never overwrite active.json on an untrustworthy comparison.
    if _systemd("stop", unit) != 0:
        print(f"[FAIL] could not stop {unit} to bench — nothing promoted")
        return 1
    winner_score: float | None = None
    factory_score: float | None = None
    compare_ok = True
    try:
        print(f"[*] benching on-Kit winner {src} ({bench_time:.0f}s) ...")
        winner_rec = _run_bench(src, cfg.bus, cfg.address, bench_time,
                                record_source="best-session")
        print(json.dumps(winner_rec, indent=2))
        winner_score = winner_rec["score"]

        print(f"[*] benching factory fallback ({bench_time:.0f}s) ...")
        factory_rec = _run_bench(fallback, cfg.bus, cfg.address, bench_time,
                                 record_source="best-factory")
        print(json.dumps(factory_rec, indent=2))
        factory_score = factory_rec["score"]
    except CalibrationError as exc:
        print(f"[FAIL] calibration unreadable during bench: {exc}")
        compare_ok = False
    except BenchNotConverged as exc:
        print(f"[FAIL] bench invalid: {exc}")
        print("       the comparison needs a converged window — figure-8 the "
              "board until mag=3, then hold it PERFECTLY still.")
        compare_ok = False
    finally:
        restart_rc = _systemd("start", unit)
        if restart_rc != 0:
            print(f"[!] {unit} did not restart (systemctl start "
                  f"rc={restart_rc}) — start it manually.")

    if not compare_ok or winner_score is None or factory_score is None:
        print("[FAIL] could not produce a fair live comparison — "
              "active.json left untouched.")
        return 1

    _, promote = rank_and_compare(
        [{"cal_file": src, "score": winner_score}], factory_score)
    verdict = "BETTER" if promote else "NOT better"
    print(f"\n[i]     live winner {winner_score} vs live factory "
          f"{factory_score} -> {verdict}")

    if not promote:
        print("[i]     active.json left untouched — the factory "
              "calibration is still the best. Promote manually with: "
              "sudo python3 scripts/ops/start_kit.py fallback")
        return 0

    sha = save_calibration(winner_cal, cfg.cal_file)
    _systemd("stop", unit)
    _systemd("start", unit)
    print(f"\n[OK]   active calibration <- {src} (live score {winner_score}) "
          f"sha256={sha[:12]}...")
    print(f"[OK]   {unit} restarted with the promoted calibration.")
    return 0


def cmd_fallback(unit: str, config: str) -> int:
    """Restore the factory calibration as the active one and restart."""
    from bno055_kit.config import Config

    cfg = Config.load(config if Path(config).is_file() else None)
    fallback = fallback_path(cfg.cal_file)
    if not fallback.is_file():
        print(f"[FAIL] factory fallback calibration missing: {fallback}")
        print("       re-run the installer: sudo bash "
              "scripts/deploy/install_kit.sh --confirm")
        return 1
    try:
        cal = load_calibration(fallback)
    except CalibrationError as exc:
        print(f"[FAIL] factory fallback unreadable: {exc}")
        return 1
    sha = save_calibration(cal, cfg.cal_file)
    print(f"[OK]   active calibration <- factory fallback {fallback} "
          f"sha256={sha[:12]}...")
    _systemd("stop", unit)
    restart_rc = _systemd("start", unit)
    if restart_rc != 0:
        print(f"[FAIL] {unit} did NOT start (systemctl start rc={restart_rc}) "
              "— inspect: systemctl status " + unit)
        return 1
    print(f"[OK]   {unit} restarted with the factory calibration.")
    return 0


def cmd_bench(unit: str, config: str, extra: list[str]) -> int:
    """Stop the recorder, bench a calibration (default: active), restart.

    The unit is stopped here, so the kit CLI needs no --force/--unit — the
    sensor is already free. ``extra`` carries the bench flags (--cal-file,
    --time, --no-history ...).
    """
    _systemd("stop", unit)
    rc = 1
    try:
        rc = _kit_cli(["--config", config, "bench", *extra])
    finally:
        restart_rc = _systemd("start", unit)
        if restart_rc != 0:
            print(f"[!] {unit} did not restart (systemctl start "
                  f"rc={restart_rc}) — start it manually.")
    return rc


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="start_kit", description=__doc__.splitlines()[0])
    p.add_argument("action",
                   choices=sorted([*ACTIONS, "calibrate", "best", "fallback",
                                   "bench"]))
    p.add_argument("--unit", default=DEFAULT_UNIT)
    p.add_argument("--config", default=DEFAULT_CONFIG)
    p.add_argument("--time", type=float, default=30.0, dest="bench_time",
                   help="bench seconds per candidate for `best` "
                        "(default %(default)s)")
    args, extra = p.parse_known_args(argv)

    if args.action in _PRIVILEGED_ACTIONS and os.geteuid() != 0:
        print(f"[FAIL] '{args.action}' controls {args.unit} and needs root: "
              f"run\n       sudo python3 {' '.join(sys.argv[:1])} "
              f"{' '.join(sys.argv[1:])}")
        return 2

    if args.action == "calibrate":
        return cmd_calibrate(args.unit, args.config, extra)
    if args.action == "best":
        if extra:
            p.error(f"unrecognized arguments: {' '.join(extra)}")
        return cmd_best(args.unit, args.config, args.bench_time)
    if args.action == "fallback":
        if extra:
            p.error(f"unrecognized arguments: {' '.join(extra)}")
        return cmd_fallback(args.unit, args.config)
    if args.action == "bench":
        return cmd_bench(args.unit, args.config, extra)

    if extra:
        p.error(f"unrecognized arguments: {' '.join(extra)}")
    cmd = [a.format(unit=args.unit) for a in ACTIONS[args.action]]
    print("$ " + " ".join(cmd), flush=True)
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
