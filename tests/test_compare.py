"""Tests for side-by-side BNO/CAN signal comparison (synthetic, no hardware)."""
from __future__ import annotations

import json

import numpy as np

from bno055_kit.analysis.candump import get_database
from bno055_kit.analysis.compare import (
    compare,
    load_can_signals,
    load_imu_signals,
)

_RAD2DEG = 180.0 / np.pi


def _encode(kind: str, signals: dict) -> str:
    """Encode a full IMU frame via the shipped DBC and return upper hex.

    Any signal the message requires but the caller did not supply (status/
    flag bits) defaults to 0 so tests only spell out the signals they assert.
    """
    db = get_database()
    msg = next(m for m in db.messages if m.name == f"CAN_ID_IMU_{kind.upper()}")
    full = {s.name: 0 for s in msg.signals}
    full.update(signals)
    return msg.encode(full, scaling=True).hex().upper()


def test_load_imu_signals_exposes_components(imu_session):
    series = load_imu_signals(imu_session)
    assert set(series.series) >= {"head", "roll", "pitch",
                                  "acc_x", "acc_y", "acc_z",
                                  "gyro_x", "gyro_y", "gyro_z"}
    # gyro is published rad/s on disk but exposed in deg/s for comparison.
    assert series.series["gyro_x"].unit == "deg/s"


def test_compare_recovers_zero_delta_when_frames_match(imu_session, tmp_path):
    """The capture replays the session's own signals, offset by a clock skew.

    The synthetic session carries head/roll/pitch = 0 and sinusoidal
    lin_acc/gyro; the capture replays exactly those on the CAN side, so every
    aligned delta is ~0 once the injected offset is removed.
    """
    manifest = json.loads((imu_session / "session.json").read_text(encoding="utf-8"))
    start_wall = manifest["start_wall_ns"] / 1e9
    offset = 0.250  # can_wall = imu_wall + offset

    capture = tmp_path / "can.candump"
    with open(capture, "w", encoding="utf-8") as f:
        for i in range(1000):
            t = start_wall + i * 0.01 + offset
            phase = 2 * np.pi * (i / 1000) * 3
            ax = 0.1 * np.sin(phase)
            gx = 0.5 * np.sin(phase) * _RAD2DEG
            gy = 0.2 * np.cos(phase * 0.7) * _RAD2DEG
            f.write(f"({t:.6f})  can0  384#"
                    + _encode("att", {"Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}) + "\n")
            f.write(f"({t:.6f})  can0  386#"
                    + _encode("acc", {"Acceleration_X": ax, "Acceleration_Y": 0.0,
                                     "Acceleration_Z": 9.81, "Side_Slip_Angle": 0.0}) + "\n")
            f.write(f"({t:.6f})  can0  387#"
                    + _encode("aspeed", {"AngSpeedX": gx, "AngSpeedY": gy,
                                        "AngSpeedZ": 0.0, "CorrFactor": 0.0}) + "\n")

    imu = load_imu_signals(imu_session)
    can = load_can_signals(capture)
    _result, rows = compare(imu, can, offset)

    # Rows are grouped by timestamp: each line carries every signal's
    # bno/can/delta at that instant. Collect per-signal delta series.
    by: dict[str, list] = {}
    for r in rows:
        for sig, v in r["signals"].items():
            by.setdefault(sig, []).append(v["delta"])

    for sig in ("head~Yaw", "roll~Roll", "pitch~Pitch",
                "gyro_x~AngSpeedX", "gyro_y~AngSpeedY", "gyro_z~AngSpeedZ",
                "acc_x~Acceleration_X", "acc_z~Acceleration_Z"):
        assert by[sig], f"missing {sig}"
        rmse = float(np.sqrt(np.mean(np.asarray(by[sig]) ** 2)))
        assert rmse < 0.05, f"{sig} rmse {rmse}"


def test_rows_grouped_by_timestamp(imu_session, tmp_path):
    """One line per sample, carrying all signals at that instant."""
    manifest = json.loads((imu_session / "session.json").read_text(encoding="utf-8"))
    start_wall = manifest["start_wall_ns"] / 1e9
    capture = tmp_path / "can.candump"
    with open(capture, "w", encoding="utf-8") as f:
        for i in range(1000):
            t = start_wall + i * 0.01
            f.write(f"({t:.6f})  can0  384#"
                    + _encode("att", {"Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}) + "\n")
    imu = load_imu_signals(imu_session)
    can = load_can_signals(capture)
    _result, rows = compare(imu, can, 0.0)
    # Each row is one timestamp with all three attitude signals together.
    assert rows, "no rows"
    assert set(rows[0]["signals"]) >= {"head~Yaw", "roll~Roll", "pitch~Pitch"}
    # Timestamps strictly ascending (iterate the timeline, not the signals).
    ts = [r["t"] for r in rows]
    assert ts == sorted(ts)


def test_compare_window_limits_samples(imu_session, tmp_path):
    manifest = json.loads((imu_session / "session.json").read_text(encoding="utf-8"))
    start_wall = manifest["start_wall_ns"] / 1e9
    capture = tmp_path / "can.candump"
    with open(capture, "w", encoding="utf-8") as f:
        for i in range(1000):
            t = start_wall + i * 0.01
            f.write(f"({t:.6f})  can0  384#"
                    + _encode("att", {"Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}) + "\n")
    imu = load_imu_signals(imu_session)
    can = load_can_signals(capture)
    _result, rows = compare(imu, can, 0.0, window_s=(0.0, 1.0))
    # 1 s window at 100 Hz => ~100 grouped samples, not the full 1000.
    assert 50 < len(rows) < 400
