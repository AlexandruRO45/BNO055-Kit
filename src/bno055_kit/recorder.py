"""Session recording: JSONL sample log + session manifest with clock anchors.

Layout per session (mirrors the flight-recorder session-dir pattern::

    <log_dir>/<session_id>/
        imu.jsonl      — one JSON object per sample, flushed per line
        session.json   — manifest: clock anchor pair, calibration identity

The manifest carries BOTH ``start_wall_ns`` (CLOCK_REALTIME) and
``start_mono_ns`` (CLOCK_MONOTONIC), sampled as a tight pair around session
start — the same anchor-pair reasoning as a candump recorder's
``sync.json``: the pair lets offline analysis place the monotonic timeline of
this session onto the wall-clock timeline of a candump capture, and detect
any NTP step that happened during the session (per-sample ``clock_ok``).
"""
from __future__ import annotations

import json
import logging
import os
import platform
import time
from collections.abc import Iterable, Mapping
from pathlib import Path

from . import SCHEMA_VERSION
from .clock import now_mono_ns, stamp_pair
from .sensor import Sample

log = logging.getLogger("bno055_kit.recorder")

MANIFEST_NAME = "session.json"
SAMPLES_NAME = "imu.jsonl"


def boot_id() -> str | None:
    """Machine boot id (stable correlation key across all Kit logs)."""
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
    except OSError:
        return None


def session_id_from(wall_ns: int) -> str:
    return f"bno_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime(wall_ns / 1e9))}"


class SessionWriter:
    """Owns one session directory; writes manifest at start, samples as they come."""

    def __init__(self, session_dir: str | Path):
        self.session_dir = Path(session_dir)
        self._samples_path = self.session_dir / SAMPLES_NAME
        self._samples_file = None
        self.manifest: dict | None = None

    def start(self, manifest: Mapping) -> None:
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.manifest = dict(manifest)
        self._write_manifest()
        self._samples_file = open(self._samples_path, "w", encoding="utf-8")
        log.info("session started: %s", self.session_dir)

    def _write_manifest(self) -> None:
        assert self.manifest is not None
        manifest_path = self.session_dir / MANIFEST_NAME
        tmp = manifest_path.with_suffix(".json.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.manifest, f, indent=2)
            f.write("\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, manifest_path)

    def write_sample(self, wall_ns: int, mono_ns: int, sample: Sample, clock_ok: bool) -> None:
        if self._samples_file is None:
            raise RuntimeError("session not started")
        row = {
            "t_wall_ns": wall_ns,
            "t_mono_ns": mono_ns,
            "qw": sample.quat[0],
            "qx": sample.quat[1],
            "qy": sample.quat[2],
            "qz": sample.quat[3],
            "head": sample.euler[0],
            "roll": sample.euler[1],
            "pitch": sample.euler[2],
            "lin_acc": list(sample.lin_acc),
            "gyro": list(sample.gyro),
            "cal": list(sample.cal),
            "c": round(sample.confidence, 4),
            "clock_ok": clock_ok,
        }
        self._samples_file.write(json.dumps(row) + "\n")
        self._samples_file.flush()

    def finish(self, extra: Mapping | None = None) -> None:
        if self.manifest is not None:
            end_wall, end_mono = stamp_pair()
            self.manifest["end_wall_ns"] = end_wall
            self.manifest["end_mono_ns"] = end_mono
            if extra:
                self.manifest.update(extra)
            self._write_manifest()
        if self._samples_file is not None:
            self._samples_file.close()
            self._samples_file = None
        log.info("session closed: %s", self.session_dir)


def make_manifest(
    *,
    session_id: str,
    wall_ns: int,
    mono_ns: int,
    spawn_jitter_s: float,
    bus: int,
    address: int,
    rate_hz: float,
    cal_file: str,
    cal_sha256: str | None,
    cal_status: Iterable[int],
) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "session_id": session_id,
        "boot_id": boot_id(),
        "hostname": platform.node(),
        "pid": os.getpid(),
        "start_wall_ns": wall_ns,
        "start_mono_ns": mono_ns,
        "spawn_jitter_s": round(spawn_jitter_s, 9),
        "i2c_bus": bus,
        "i2c_address": address,
        "rate_hz": rate_hz,
        "cal_file": cal_file,
        "cal_sha256": cal_sha256,
        "cal_status_at_start": list(cal_status),
    }


def open_session(
    log_dir: str | Path,
    *,
    bus: int,
    address: int,
    rate_hz: float,
    cal_file: str,
    cal_sha256: str | None,
    cal_status: Iterable[int],
) -> SessionWriter:
    """Create and start a new session directory under ``log_dir``.

    The anchor pair (``wall_ns``, ``mono_ns``) is sampled tightly around the
    session-dir creation; ``spawn_jitter_s`` bounds how stale the anchor could
    be (same reasoning as the candump recorder's spawn-jitter anchor).
    """
    t_before_mono = now_mono_ns()
    wall_ns, mono_ns = stamp_pair()
    jitter = (now_mono_ns() - t_before_mono) / 1e9
    sid = session_id_from(wall_ns)
    session_path = Path(log_dir) / sid
    # Session ids have 1 s resolution; two sessions in the same second must
    # not silently truncate the first (open(..., "w")).
    if session_path.exists():
        sid = f"{sid}_{mono_ns % 1_000_000_000:09d}"
        session_path = Path(log_dir) / sid
    writer = SessionWriter(session_path)
    writer.start(
        make_manifest(
            session_id=sid,
            wall_ns=wall_ns,
            mono_ns=mono_ns,
            spawn_jitter_s=jitter,
            bus=bus,
            address=address,
            rate_hz=rate_hz,
            cal_file=cal_file,
            cal_sha256=cal_sha256,
            cal_status=cal_status,
        )
    )
    return writer
