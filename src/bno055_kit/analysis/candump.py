"""Parse ``candump -L`` logs into wall-clock-stamped CAN frames (DBC-driven decode).

``candump -L <iface>`` (what the VISAN flight recorder spawns) writes one
frame per line::

    (2026-10-01 11:10:15.123456)  can0  384#1122334455667788

The timestamp is CLOCK_REALTIME (local time, microsecond resolution) — there
is no monotonic stamp in a candump log, which is exactly why the IMU session
manifest carries a wall/mono anchor pair: the anchor maps the IMU's
monotonic timeline onto this wall-clock timeline.

Supported timestamp forms (tolerant parser — unknown lines are skipped):
  * ``(YYYY-MM-DD HH:MM:SS.ffffff)``  — candump -L (default)
  * ``(1234567890.123456)``           — candump -t a (epoch seconds)
  * ``1234567890.123456``             — candump -t a without parens

Frame bodies: classic ``<id>#<hexdata>``, RTR ``<id>#R<n>``, and CAN-FD
``<id>##<flags><hexdata>``.
"""
from __future__ import annotations

import re
from collections.abc import Iterator
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

# (timestamp)  iface  id#data   — iface optional in some captures
LINE_RE = re.compile(
    r"^\s*(?:\((?P<ts>[^)]+)\)|(?P<ts_bare>\d+\.\d+))\s+(?P<iface>\S+)\s+(?P<frame>\S+)\s*$"
)
TS_DATE_RE = re.compile(r"^(?P<y>\d{4})-(?P<mo>\d{2})-(?P<d>\d{2}) "
                        r"(?P<h>\d{2}):(?P<mi>\d{2}):(?P<s>\d{2})\.(?P<frac>\d+)$")
TS_EPOCH_RE = re.compile(r"^\d+\.\d+$")
FRAME_RE = re.compile(r"^(?P<id>[0-9A-Fa-f]{3,8})#(?P<body>.*)$")


@dataclass(frozen=True)
class CandumpFrame:
    t_wall_s: float  # CLOCK_REALTIME seconds (float) — candump has no monotonic stamp
    iface: str
    arbitration_id: int
    data: bytes
    is_rtr: bool = False
    is_fd: bool = False


def _parse_ts(text: str) -> float | None:
    m = TS_EPOCH_RE.match(text)
    if m:
        return float(text)
    m = TS_DATE_RE.match(text)
    if not m:
        return None
    # candump -L prints CLOCK_REALTIME in *local* time (strftime %F %T).
    # A naive datetime's .timestamp() interprets it as local time, which is
    # exactly the contract; assuming UTC would shift every stamp by the
    # zone offset on any Kit not running UTC.
    g = m.groupdict()
    dt = datetime(
        int(g["y"]), int(g["mo"]), int(g["d"]),
        int(g["h"]), int(g["mi"]), int(g["s"]),
        tzinfo=None,
    )
    frac = g["frac"].ljust(9, "0")[:9]  # normalise to nanoseconds
    return dt.timestamp() + int(frac) / 1e9


def parse_line(line: str) -> CandumpFrame | None:
    m = LINE_RE.match(line)
    if not m:
        return None
    ts_text = m.group("ts") or m.group("ts_bare")
    if ts_text is None:
        return None
    t_wall_s = _parse_ts(ts_text.strip())
    if t_wall_s is None:
        return None

    fm = FRAME_RE.match(m.group("frame"))
    if not fm:
        return None
    body = fm.group("body")
    arbitration_id = int(fm.group("id"), 16)

    is_rtr = False
    is_fd = body.startswith("#")  # classic '#' separator consumed; '##' => FD
    if body.startswith("R"):
        is_rtr = True
        data = b""
    elif is_fd:
        payload = body[1:]  # drop FD flags nibble
        data = bytes.fromhex(payload) if payload else b""
    elif body:
        try:
            data = bytes.fromhex(body)
        except ValueError:
            return None
    else:
        data = b""

    return CandumpFrame(
        t_wall_s=t_wall_s,
        iface=m.group("iface"),
        arbitration_id=arbitration_id,
        data=data,
        is_rtr=is_rtr,
        is_fd=is_fd,
    )


def parse_candump(path: str | Path) -> Iterator[CandumpFrame]:
    """Yield frames in file order. Blank/unparseable lines are skipped."""
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if not line.strip():
                continue
            frame = parse_line(line)
            if frame is not None:
                yield frame


# The shipped DBC is the single source of truth for scaling, byte order and
# signedness (v5 marks Yaw UNSIGNED 0..359.99 — the old hand-rolled v3 bit
# decode read it signed). Loaded lazily and cached; the .dbc ships with the
# package (analysis/dbc/).
DBC_PATH = Path(__file__).resolve().parent / "dbc" / "DBC_HirrusUAS_VISAN_v5.dbc"

_DB = None
_MSG_BY_ID: dict[int, object] = {}


def get_database():
    """Load (once) and return the shipped HirrusUAS DBC database.

    Also caches the IMU message map (frame id -> message) used by
    :func:`decode_imu_frame`.
    """
    global _DB
    if _DB is None:
        import cantools

        _DB = cantools.database.load_file(DBC_PATH)
        for m in _DB.messages:
            if m.name.startswith("CAN_ID_IMU_"):
                _MSG_BY_ID[m.frame_id] = m
    return _DB


def decode_imu_frame(frame: CandumpFrame) -> dict | None:
    """Decode a HirrusUAS IMU frame (IDs 0x384-0x387) via the shipped DBC.

    Returns ``{"kind", "message", "t_wall_s", "signals": {name: physical}}``
    where kind is the DBC message suffix (attitude/accel/aspeed/press/...)
    and signals carry DBC physical units (deg, deg/s, m/s^2 ...). Unit
    conversion to the BNO sample units is the caller's explicit choice,
    recorded in the analysis manifest — never hidden here. Non-IMU ids and
    undecodable (e.g. truncated) frames return None.
    """
    get_database()  # ensure loaded/cached
    msg = _MSG_BY_ID.get(frame.arbitration_id)
    if msg is None:
        return None
    try:
        signals = msg.decode(frame.data, decode_choices=False, scaling=True)
    except Exception:
        return None  # truncated/undecodable payload
    return {
        "kind": msg.name.removeprefix("CAN_ID_IMU_").lower(),
        "message": msg.name,
        "t_wall_s": frame.t_wall_s,
        "signals": dict(signals),
    }
