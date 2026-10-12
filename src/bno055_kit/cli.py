"""Operator CLI for the BNO055 kit (scriptable; the daemon runs under systemd).

    bno055-kit scan      — probe the I2C bus for the sensor
    bno055-kit run       — run the recorder in the foreground (systemd ExecStart)
    bno055-kit calibrate — interactive on-Kit calibration (stop the unit first;
                           prefer scripts/ops/start_kit.py calibrate, which
                           stops/starts recording around it)
    bno055-kit bench     — quantify the active calibration at rest (bias/drift/noise)
    bno055-kit status    — show active calibration + last session summary
"""
from __future__ import annotations

import argparse
import json
import logging
import subprocess
import sys
import time
from collections.abc import Callable
from pathlib import Path

from .bench import bench_metrics, make_record, sample_bench_window
from .calibration import (
    CalibrationError,
    append_history,
    fallback_path,
    load_calibration,
    save_calibration,
)
from .config import Config
from .recorder import MANIFEST_NAME
from .sensor import Bno055, probe_i2c

log = logging.getLogger("bno055_kit.cli")


def _setup_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stdout,
    )


# --------------------------------------------------------------------- scan
def cmd_scan(args: argparse.Namespace) -> int:
    bus = args.bus if args.bus is not None else Config.load(args.config).bus
    addr = probe_i2c(bus)
    if addr is None:
        print(f"[FAIL] no BNO055 (CHIP_ID 0xA0) on i2c-{bus} at 0x28/0x29")
        print(f"        check wiring and run: sudo i2cdetect -y -r {bus}")
        return 1
    print(f"[OK]   BNO055 answers at 0x{addr:02x} on i2c-{bus} (CHIP_ID=0xa0)")
    return 0


# ---------------------------------------------------------------------- run
def cmd_run(args: argparse.Namespace) -> int:
    from .daemon import run_daemon

    cfg = Config.load(args.config)
    if args.bus is not None:
        cfg.bus = args.bus
    if args.address is not None:
        cfg.address = args.address
    if args.rate is not None:
        cfg.rate_hz = args.rate
    if args.log_dir is not None:
        cfg.log_dir = args.log_dir
    if args.cal_file is not None:
        cfg.cal_file = args.cal_file
    return run_daemon(cfg)


# ------------------------------------------------------------------- bench
class BenchNotConverged(RuntimeError):
    """Raised when a bench window was sampled before the fusion converged."""


# A live (even if still) BNO055 heading steps through its 1/16 deg
# quantisation, so a real 30 s window has many distinct values; a frozen
# sensor returns one value (the fake-~0 pathology). Anything at/below this
# many distinct samples is treated as frozen and rejected.
_MIN_HEAD_DISTINCT = 5


def _run_bench(cal_file: str | Path, bus: int, address: int, duration_s: float,
               *, record_source: str = "bench",
               converge_timeout_s: float = 120.0,
               prompt: Callable[[str], str] = input) -> dict:
    """Bench one calibration file at rest; return the history record.

    The record's ``cal_file`` is the *path that was benched* — never the
    calibration's internal ``source`` label — so ``best`` can re-read and
    promote exactly the file that was scored.

    The fusion is re-converged first: ``apply_calibration`` round-trips
    through CONFIG_MODE, which resets the live calibration, and the
    magnetometer only re-converges while the field *changes* — so the
    operator figures-8 until mag=3, then holds perfectly still while the
    window is sampled. The window is rejected (not scored) unless accel=3
    and the heading is *live* (not frozen): a frozen heading — the sensor
    returning one value for the whole window — scores a fake ~0 and would
    poison the ``best`` comparison. ``mag`` is not required during the still
    window because it decays to 0 the moment the board stops moving.
    """
    from .calibrate import CalibrationTimeout, _wait_level

    cal_file = Path(cal_file)
    cal = load_calibration(cal_file)
    sensor = Bno055(bus=bus, address=address)
    sensor.connect()
    sensor.apply_calibration(cal)

    try:
        prompt("\n>>> BENCH: figure-8 the board until mag=3, then hold it "
               "PERFECTLY still. [Enter] when ready ...")
    except (EOFError, KeyboardInterrupt) as exc:
        raise BenchNotConverged("bench aborted before sampling") from exc
    try:
        _wait_level(sensor, 3, "mag", out=print, sleep=time.sleep,
                    monotonic=time.monotonic, timeout_s=converge_timeout_s)
        print("    now hold PERFECTLY still ...")
        _wait_level(sensor, 0, "sys", out=print, sleep=time.sleep,
                    monotonic=time.monotonic, timeout_s=converge_timeout_s)
    except CalibrationTimeout as exc:
        raise BenchNotConverged(
            f"could not converge before benching ({exc}) — figure-8 the "
            f"board until mag=3 and retry.") from exc

    print(f"[*] bench {duration_s:.0f}s — keep the board perfectly still ...")
    heads, accs, levels = sample_bench_window(sensor, duration_s)
    rec = make_record(str(cal_file), cal.sha256,
                      bench_metrics(heads, accs, duration_s, levels=levels))
    rec["source"] = record_source
    # Trustworthy window = accel converged (bias is measured from it, and it
    # holds at 3 while still) AND the heading actually moved (not frozen).
    # mag is deliberately NOT required: it decays to 0 the instant the board
    # stops moving, so demanding mag=3 over a still window is impossible.
    settled = rec.get("cal_levels_at_bench")
    accel_ok = settled is not None and settled[2] >= 3
    heading_live = rec["head_distinct"] >= _MIN_HEAD_DISTINCT
    if not (accel_ok and heading_live):
        raise BenchNotConverged(
            f"bench window invalid: accel level "
            f"{settled[2] if settled is not None else '?'}/3, "
            f"{rec['head_distinct']} distinct heading values "
            f"(need accel=3 and a live, non-frozen heading) — figure-8 until "
            f"mag=3, then hold the board perfectly still and re-run.")
    return rec


def cmd_bench(args: argparse.Namespace) -> int:
    try:
        cfg = Config.load(args.config)
        bus = args.bus if args.bus is not None else cfg.bus
        address = args.address if args.address is not None else cfg.address
        if args.time < 2.0:
            print("[FAIL] --time too short for a meaningful bench (use >= 2 s)")
            return 2
        rec = _run_bench(args.cal_file, bus, address, args.time)
    except CalibrationError as exc:
        print(f"[FAIL] {exc}")
        return 1
    except BenchNotConverged as exc:
        print(f"[FAIL] {exc}")
        return 1

    print(json.dumps(rec, indent=2))
    print(f"[score {rec['score']} — lower is better]")
    if not args.no_history:
        hist = Path(args.cal_file).parent / "history.jsonl"
        append_history(rec, hist)
        print(f"[OK]   history: {hist}")
    return 0


# ---------------------------------------------------------------- calibrate
def _unit_active(unit: str) -> bool:
    try:
        return subprocess.call(["systemctl", "is-active", "--quiet", unit],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL) == 0
    except FileNotFoundError:
        return False  # no systemd on this host


def cmd_calibrate(args: argparse.Namespace) -> int:
    from .calibrate import CalibrationAborted, CalibrationTimeout, run_calibration

    cfg = Config.load(args.config)
    bus = args.bus if args.bus is not None else cfg.bus
    address = args.address if args.address is not None else cfg.address

    if Path(cfg.cal_file).name == fallback_path(cfg.cal_file).name:
        print(f"[FAIL] refusing to overwrite the frozen factory fallback "
              f"({cfg.cal_file}) — point --cal-file at the active "
              f"calibration (e.g. /var/lib/bno055/active.json).")
        return 2

    if not args.force and _unit_active(args.unit):
        print(f"[FAIL] {args.unit} is recording — the sensor must be free.")
        print("       Use: sudo python3 scripts/ops/start_kit.py calibrate"
              " (stops/starts the unit around it),")
        print("       or stop it manually / pass --force to calibrate anyway.")
        return 1

    sensor = Bno055(bus=bus, address=address)
    sensor.connect()
    try:
        cal = run_calibration(sensor)
    except CalibrationAborted as exc:
        print(f"\n[ABORT] {exc} — nothing saved.")
        return 130
    except CalibrationTimeout as exc:
        print(f"\n[FAIL] calibration incomplete: {exc} — nothing saved.")
        return 1

    from dataclasses import replace

    now_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    cal = replace(cal, calibrated_at_utc=now_utc)

    # The session is only *saved* here — active.json is never touched.
    # Promotion is `best`'s job exclusively: it ranks every scored session
    # against the factory fallback and overwrites active.json only when the
    # winner actually beats the known-good calibration.
    cal_dir = Path(cfg.cal_file).parent
    ts = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    cal_path = cal_dir / f"cal_{ts}.json"
    sha = save_calibration(cal, cal_path)
    print(f"[OK]   saved {cal_path} sha256={sha[:12]}...")

    try:
        answer = input("    [Enter]=bench 30s (board still) / s=skip >> ")
    except (EOFError, KeyboardInterrupt):
        answer = "s"
    if answer.strip().lower() != "s":
        try:
            rec = _run_bench(cal_path, bus, address, 30.0,
                             record_source="calibrate")
        except BenchNotConverged as exc:
            # The session is still valid and saved — only its *score* is
            # untrustworthy right now, so it stays unscored in history and
            # `best` can bench it later, once the fusion has settled.
            print(f"[!]     post-calibration bench invalid: {exc}")
            print("        session kept UNSCORED — `best` will bench it "
                  "later, once the fusion has settled.")
            append_history(
                {"cal_file": str(cal_path), "cal_sha256": sha,
                 "when_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                           time.gmtime()),
                 "source": "calibrate", "score": None},
                cal_dir / "history.jsonl")
        else:
            append_history(rec, cal_dir / "history.jsonl")
            print(json.dumps(rec, indent=2))
            print(f"[score {rec['score']} — lower is better]")
    else:
        append_history(
            {"cal_file": str(cal_path), "cal_sha256": sha,
             "when_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
             "source": "calibrate", "score": None},
            cal_dir / "history.jsonl")
    print("[i]     active.json untouched — promote with: "
          "python3 scripts/ops/start_kit.py best")
    return 0


# -------------------------------------------------------------------- status
def cmd_status(args: argparse.Namespace) -> int:
    cfg = Config.load(args.config)
    rc = 0
    try:
        cal = load_calibration(cfg.cal_file)
        print(f"[OK]   active calibration: {cfg.cal_file}")
        print(f"       sha256={cal.sha256[:16]}...  source={cal.source}")
        print(f"       accel_offset={list(cal.accel_offset)} "
              f"gyro_offset={list(cal.gyro_offset)} mag_offset={list(cal.mag_offset)} "
              f"radii={cal.accel_radius}/{cal.mag_radius}")
    except CalibrationError as exc:
        print(f"[FAIL] calibration: {exc}")
        rc = 1

    log_dir = Path(cfg.log_dir)
    sessions = sorted((d for d in log_dir.iterdir() if d.is_dir()), reverse=True) \
        if log_dir.is_dir() else []
    if sessions:
        manifest = sessions[0] / MANIFEST_NAME
        if manifest.is_file():
            data = json.loads(manifest.read_text(encoding="utf-8"))
            print(f"[OK]   last session: {sessions[0].name}")
            for k in ("start_wall_ns", "start_mono_ns", "samples_written",
                      "end_reason", "clock_ok", "cal_status_at_start"):
                if k in data:
                    print(f"       {k} = {data[k]}")
        else:
            print(f"[WARN] last session {sessions[0].name} has no {MANIFEST_NAME}")
    else:
        print(f"[--]   no sessions under {log_dir}")
    return rc


# ---------------------------------------------------------------------- main
def _parse_args(p: argparse.ArgumentParser,
                argv: list[str] | None) -> argparse.Namespace:
    """Parse argv, accepting global flags before *or* after the subcommand.

    argparse only knows --bus/--address/--config/--log-level on the root
    parser, so `bno055-kit run --config X` would die with
    "unrecognized arguments". Operate (and the systemd wrapper) naturally
    put the flags after the verb, so retry with globals hoisted in front
    of the subcommand rather than depending on one invocation order.
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    first_err: SystemExit | None = None
    try:
        return p.parse_args(argv)
    except SystemExit as err:  # noqa: PERF203 - retry path below
        first_err = err

    globals_: list[str] = []
    rest: list[str] = []
    i = 0
    while i < len(argv):
        tok = argv[i]
        if tok in ("--bus", "--address", "--config", "--log-level"):
            globals_.extend((tok, argv[i + 1]))
            i += 2
        elif tok.startswith(("--bus=", "--address=", "--config=", "--log-level=")):
            globals_.append(tok)
            i += 1
        else:
            rest.append(tok)
            i += 1
    if not globals_:
        raise first_err  # not a global-flag problem: report the original error
    return p.parse_args(globals_ + rest)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="bno055-kit",
                                description="BNO055 kit recorder (VISAN companion)")
    p.add_argument("--bus", type=int, default=None, help="I2C bus (default from config)")
    p.add_argument("--address", type=lambda x: int(x, 0), default=None,
                   help="I2C address, e.g. 0x29 (default from config)")
    p.add_argument("--config", default=None, help="YAML config file")
    p.add_argument("--log-level", default="INFO")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan", help="probe the I2C bus for the sensor")

    pr = sub.add_parser("run", help="run the recorder (foreground; systemd ExecStart)")
    pr.add_argument("--rate", type=float, default=None, help="sample rate Hz")
    pr.add_argument("--log-dir", default=None, help="session log root dir")
    pr.add_argument("--cal-file", default=None, help="active calibration file")

    pb = sub.add_parser("bench", help="quantify the active calibration at rest")
    pb.add_argument("--time", type=float, default=30.0, help="seconds at rest")
    pb.add_argument("--cal-file", default=None, help="calibration file to bench")
    pb.add_argument("--no-history", action="store_true",
                    help="do not append the record to history.jsonl")

    pc = sub.add_parser("calibrate",
                        help="interactive on-Kit calibration (stop the unit first)")
    pc.add_argument("--cal-file", default=None,
                    help="active calibration file (session files are saved "
                         "next to it; the active file itself is never "
                         "written — default from config)")
    pc.add_argument("--unit", default="bno055-imu.service",
                    help="unit that must NOT be running (default %(default)s)")
    pc.add_argument("--force", action="store_true",
                    help="calibrate even if the recorder unit is active")

    sub.add_parser("status", help="show active calibration + last session")
    return p


def main(argv: list[str] | None = None) -> int:
    p = _build_parser()
    args = _parse_args(p, argv)
    _setup_logging(args.log_level)

    if args.cmd == "scan":
        return cmd_scan(args)
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "bench":
        if args.cal_file is None:
            args.cal_file = Config.load(args.config).cal_file
        return cmd_bench(args)
    if args.cmd == "calibrate":
        if args.cal_file is None:
            args.cal_file = Config.load(args.config).cal_file
        return cmd_calibrate(args)
    if args.cmd == "status":
        return cmd_status(args)
    raise AssertionError(f"unhandled command {args.cmd!r}")


if __name__ == "__main__":
    sys.exit(main())
