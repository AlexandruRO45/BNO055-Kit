"""BNO055 sensor access: connect, apply calibration, read samples.

Hardware imports (``board``/``busio``/``adafruit_bno055`` via blinka) are
lazy so the CLI and tests import cleanly on a development host without the
sensor attached.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from .calibration import Calibration

log = logging.getLogger("bno055_kit.sensor")

# Settle time around the CONFIG_MODE write window (matches the bench tooling).
_MODE_SETTLE_S = 0.05


@dataclass(frozen=True)
class Sample:
    """One NDOF sample. Units: quaternion (unit), euler (deg), accel (m/s^2),
    gyro (rad/s), cal = (sys, gyro, accel, mag) each in 0..3."""

    quat: tuple[float, float, float, float]  # w, x, y, z
    euler: tuple[float, float, float]  # heading, roll, pitch (deg)
    lin_acc: tuple[float, float, float]
    gyro: tuple[float, float, float]
    cal: tuple[int, int, int, int]

    @property
    def confidence(self) -> float:
        """Mean calibration status normalised to [0, 1] (VISAN ``c``)."""
        return sum(self.cal) / 12.0


class Bno055:
    """Thin wrapper over ``adafruit_bno055.BNO055_I2C`` with bounded reopen.

    Note on ``bus``: the sensor itself is opened on blinka's default header
    I2C pair (``board.SCL``/``board.SDA`` — on the Orin Nano these map to
    i2c-7, header pins 3/5, the same wiring the bench tooling used). The
    ``bus`` number is authoritative for :func:`probe_i2c` and is recorded in
    the session manifest; if blinka's mapping ever differs from the
    configured bus the connect log line makes it visible.
    """

    def __init__(self, bus: int = 7, address: int = 0x29):
        self.bus = bus
        self.address = address
        self._sensor = None
        self._i2c = None  # blinka bus object, created once and reused across reconnects

    def connect(self) -> None:
        import adafruit_bno055
        import board
        import busio

        if self._i2c is None:
            self._i2c = busio.I2C(board.SCL, board.SDA)
        self._sensor = adafruit_bno055.BNO055_I2C(self._i2c, address=self.address)
        log.info("connected BNO055 at 0x%02x (blinka SCL/SDA, configured bus %d)",
                 self.address, self.bus)

    def close(self) -> None:
        self._sensor = None  # keep the bus object open — reused on reconnect

    @property
    def connected(self) -> bool:
        return self._sensor is not None

    def _require(self):
        if self._sensor is None:
            raise RuntimeError("sensor not connected — call connect() first")
        return self._sensor

    def apply_calibration(self, cal: Calibration) -> None:
        """Write offsets/radii via CONFIG_MODE, then return to NDOF_MODE."""
        from adafruit_bno055 import CONFIG_MODE, NDOF_MODE

        s = self._require()
        s.mode = CONFIG_MODE
        time.sleep(_MODE_SETTLE_S)
        s.offsets_accelerometer = tuple(cal.accel_offset)
        s.offsets_magnetometer = tuple(cal.mag_offset)
        s.offsets_gyroscope = tuple(cal.gyro_offset)
        s.radius_accelerometer = cal.accel_radius
        s.radius_magnetometer = cal.mag_radius
        time.sleep(_MODE_SETTLE_S)
        s.mode = NDOF_MODE
        time.sleep(_MODE_SETTLE_S)
        log.info("calibration applied: status=%s", s.calibration_status)

    def read(self) -> Sample:
        s = self._require()

        def vec(v, n):
            # The driver returns a tuple of Nones when a read fails — an
            # all-None tuple is truthy, so filter per element, not per tuple.
            if v is None:
                return tuple([0.0] * n)
            return tuple(float(x) if x is not None else 0.0 for x in v)

        return Sample(
            quat=vec(s.quaternion, 4),
            euler=vec(s.euler, 3),
            lin_acc=vec(s.linear_acceleration, 3),
            gyro=vec(s.gyro, 3),
            cal=tuple(s.calibration_status),
        )

    def calibration_status(self) -> tuple[int, int, int, int]:
        """Raw (sys, gyro, accel, mag) calibration status, each 0..3."""
        return tuple(self._require().calibration_status)

    def raw_acceleration(self) -> tuple[float, float, float]:
        """Raw acceleration INCLUDING gravity (m/s^2) — face validation."""
        v = self._require().acceleration
        if v is None:
            return (0.0, 0.0, 0.0)
        return tuple(float(x) if x is not None else 0.0 for x in v)

    def capture_calibration(self, source: str | None = None,
                           calibrated_at_utc: str | None = None) -> Calibration:
        """Read the sensor's current offsets/radii back into a Calibration.

        Used by the on-kit calibration flow after the convergence phases;
        the result is validated by :meth:`Calibration.from_dict` before it
        is ever written to disk.
        """
        s = self._require()
        data = {
            "accel_offset": [int(round(x)) for x in s.offsets_accelerometer],
            "mag_offset": [int(round(x)) for x in s.offsets_magnetometer],
            "gyro_offset": [int(round(x)) for x in s.offsets_gyroscope],
            "accel_radius": int(round(s.radius_accelerometer)),
            "mag_radius": int(round(s.radius_magnetometer)),
            "calibrated_at_utc": calibrated_at_utc,
        }
        return Calibration.from_dict(data, source=source)

    def reset(self) -> None:
        self._require()._reset()
        log.info("sensor soft-reset issued (calibration lost)")


def probe_i2c(bus: int, addresses: tuple[int, ...] = (0x28, 0x29)) -> int | None:
    """Return the first address answering with CHIP_ID 0xA0, else None."""
    from smbus2 import SMBus

    with SMBus(bus) as smbus:
        for addr in addresses:
            try:
                if smbus.read_byte_data(addr, 0x00) == 0xA0:
                    return addr
            except OSError:
                continue
    return None
