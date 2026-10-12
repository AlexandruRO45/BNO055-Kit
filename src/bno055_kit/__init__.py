"""bno055_kit — boot-time BNO055 I2C IMU recorder.

Design doctrine inherited from VISAN:
  * better no data than wrong data — a session that cannot load a valid
    calibration refuses to record instead of recording an uncalibrated stream;
  * every sample carries BOTH a CLOCK_MONOTONIC and a CLOCK_REALTIME
    nanosecond stamp, so offline analysis can detect NTP steps (monotonic
    cannot jump; realtime can) — same reasoning as the monotonic anchor in
    the VISAN candump recorder;
  * math/alignment uses monotonic ns; wall clock is archival only.
"""

__version__ = "0.1.0"

SCHEMA_VERSION = 1

__all__ = ["__version__", "SCHEMA_VERSION"]
