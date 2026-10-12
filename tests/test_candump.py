"""Tests for the candump -L parser and IMU frame decode."""
from __future__ import annotations

from datetime import datetime

from bno055_kit.analysis.candump import decode_imu_frame, parse_line
from conftest import accel_payload, can_line, gyro_payload


def test_parse_classic_line():
    frame = parse_line("(2026-10-01 11:10:15.123456)  can0  384#1122334455667788")
    assert frame is not None
    assert frame.iface == "can0"
    assert frame.arbitration_id == 0x384
    assert frame.data == bytes.fromhex("1122334455667788")
    # candump -L stamps are local time: expected value computed the same way.
    expected = datetime(2026, 10, 1, 11, 10, 15).timestamp() + 0.123456
    assert abs(frame.t_wall_s - expected) < 1e-6


def test_parse_epoch_ts_form():
    frame = parse_line("(1790853015.5)  can0  387#0102000000000000")
    assert frame is not None
    assert abs(frame.t_wall_s - 1790853015.5) < 1e-9


def test_parse_bare_epoch_ts_form():
    frame = parse_line("1790853015.5 can0 387#0102000000000000")
    assert frame is not None
    assert frame.arbitration_id == 0x387


def test_parse_rtr_and_junk():
    frame = parse_line("(2026-10-01 11:10:15.000000)  can0  123#R8")
    assert frame is not None and frame.is_rtr and frame.data == b""
    assert parse_line("garbage line") is None
    assert parse_line("") is None


def test_decode_gyro_scaling():
    frame = parse_line("(2026-10-01 11:10:15.000000)  can0  387#" +
                      gyro_payload(1.0).hex().upper())  # 1.0 rad/s = 57.2958 deg/s
    dec = decode_imu_frame(frame)
    # kind is the DBC message suffix (CAN_ID_IMU_ASPEED -> "aspeed").
    assert dec is not None and dec["kind"] == "aspeed"
    assert dec["message"] == "CAN_ID_IMU_ASPEED"
    # DBC physical units are preserved as-is: deg/s in, deg/s out — no
    # hidden rad/s conversion (the caller converts explicitly if needed).
    assert abs(dec["signals"]["AngSpeedX"] - 57.2958) < 0.01


def test_decode_accel_scaling():
    payload = accel_payload(9.81)
    frame = parse_line(f"(2026-10-01 11:10:15.000000)  can0  386#{payload.hex().upper()}")
    dec = decode_imu_frame(frame)
    assert dec is not None and dec["kind"] == "acc"
    assert abs(dec["signals"]["Acceleration_X"] - 9.81) < 0.01


def test_decode_yaw_is_unsigned_per_v5_dbc():
    """Regression: v5 marks Yaw UNSIGNED (0..359.99); the old hand-rolled v3
    bit decode read it signed and produced negative yaw."""
    # Yaw (start bit 47, big-endian) occupies bytes 5-6: raw 60000 must
    # decode to 600.00 deg unsigned (a signed read would give a negative).
    payload = bytes(5) + (60000).to_bytes(2, "big") + bytes(1)
    frame = parse_line(f"(2026-10-01 11:10:15.000000)  can0  384#{payload.hex().upper()}")
    dec = decode_imu_frame(frame)
    assert dec is not None and dec["kind"] == "att"
    assert abs(dec["signals"]["Yaw"] - 600.0) < 0.01


def test_decode_ignores_non_imu_ids():
    # 0x7AA is not a CAN_ID_IMU_* message in the DBC -> not decoded.
    frame = parse_line("(2026-10-01 11:10:15.000000)  can0  7AA#0000000000000000")
    assert decode_imu_frame(frame) is None


def test_can_line_round_trip():
    line = can_line(1790853015.123456, "can0", 0x386, b"\x01\x02")
    frame = parse_line(line)
    assert frame is not None
    assert abs(frame.t_wall_s - 1790853015.123456) < 1e-6
    assert frame.data == b"\x01\x02"
