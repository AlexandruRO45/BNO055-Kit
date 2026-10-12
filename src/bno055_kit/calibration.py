"""Calibration file handling: load, validate, hash.

The on-disk schema is byte-compatible with the bench tooling in the parent
directory (``imu.py`` / ``imu_cli.py``): the five BNO055 raw-unit keys below.
``calibrated_at_utc`` / ``source`` / ``sha256`` are additive metadata and are
optional.
"""
from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path

REQUIRED_KEYS = ("accel_offset", "mag_offset", "gyro_offset", "accel_radius", "mag_radius")

# The permanent known-good calibration shipped with the Kit. Unlike the
# active calibration it is frozen (root-owned, mode 0444 on the Kit) and is
# never written by the calibration flow — it is what the fallback mechanism
# restores the active calibration from.
FALLBACK_NAME = "factory_fallback.json"


def fallback_path(cal_file: str | Path) -> Path:
    """Location of the factory fallback next to an active calibration file."""
    return Path(cal_file).parent / FALLBACK_NAME

INT16_RANGE = range(-32768, 32768)  # BNO055 offset registers are signed 16-bit
# The adafruit driver packs radii as signed '<h' — values above 32767 pass
# the wire format but raise struct.error on write, so validate the driver's
# real range, not the register's.
RADIUS_MAX = 32767


class CalibrationError(ValueError):
    """Raised when a calibration file is missing, malformed, or out of range."""


def file_sha256(path: str | Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


@dataclass(frozen=True)
class Calibration:
    accel_offset: tuple[int, int, int]
    mag_offset: tuple[int, int, int]
    gyro_offset: tuple[int, int, int]
    accel_radius: int
    mag_radius: int
    source: str | None = None
    calibrated_at_utc: str | None = None
    sha256: str | None = None

    @classmethod
    def from_dict(
        cls,
        data: dict,
        source: str | None = None,
        sha256: str | None = None,
    ) -> Calibration:
        missing = [k for k in REQUIRED_KEYS if k not in data]
        if missing:
            raise CalibrationError(f"calibration missing keys: {', '.join(missing)}")

        def vec3(key: str) -> tuple[int, int, int]:
            v = data[key]
            if not isinstance(v, list) or len(v) != 3 or not all(isinstance(x, int) for x in v):
                raise CalibrationError(f"calibration {key} must be a list of 3 ints")
            if not all(x in INT16_RANGE for x in v):
                raise CalibrationError(f"calibration {key} outside int16 range")
            return (v[0], v[1], v[2])

        def u16(key: str) -> int:
            v = data[key]
            if not isinstance(v, int) or not 0 <= v <= RADIUS_MAX:
                raise CalibrationError(f"calibration {key} must be an int in [0, {RADIUS_MAX}]")
            return v

        return cls(
            accel_offset=vec3("accel_offset"),
            mag_offset=vec3("mag_offset"),
            gyro_offset=vec3("gyro_offset"),
            accel_radius=u16("accel_radius"),
            mag_radius=u16("mag_radius"),
            source=data.get("source", source),
            calibrated_at_utc=data.get("calibrated_at_utc"),
            sha256=sha256,
        )

    def to_dict(self) -> dict:
        out = {
            "accel_offset": list(self.accel_offset),
            "mag_offset": list(self.mag_offset),
            "gyro_offset": list(self.gyro_offset),
            "accel_radius": self.accel_radius,
            "mag_radius": self.mag_radius,
        }
        if self.source is not None:
            out["source"] = self.source
        if self.calibrated_at_utc is not None:
            out["calibrated_at_utc"] = self.calibrated_at_utc
        if self.sha256 is not None:
            out["sha256"] = self.sha256
        return out


def load_calibration(path: str | Path) -> Calibration:
    path = Path(path)
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError as exc:
        raise CalibrationError(f"calibration file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise CalibrationError(f"calibration file is not valid JSON: {path}") from exc
    if not isinstance(data, dict):
        raise CalibrationError(f"calibration file is not a JSON object: {path}")
    return Calibration.from_dict(data, source=str(path), sha256=file_sha256(path))


def save_calibration(cal: Calibration, path: str | Path) -> str:
    """Validate then atomically write a calibration file; return its sha256.

    Written via a temp file + rename so a crash mid-write can never leave a
    truncated active.json for the daemon to choke on at boot.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # A file cannot meaningfully contain its own hash — drop any stale one
    # (e.g. a loaded calibration promoted by ``best``) before writing.
    payload_dict = cal.to_dict()
    payload_dict.pop("sha256", None)
    payload = json.dumps(payload_dict, indent=2) + "\n"
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(payload, encoding="utf-8")
    tmp.replace(path)
    return file_sha256(path)


def append_history(record: dict, path: str | Path) -> None:
    """Append one bench record to the JSONL history (legacy-compatible)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")


def read_history(path: str | Path) -> list[dict]:
    """Read the JSONL history; tolerate a missing file and blank lines."""
    path = Path(path)
    if not path.is_file():
        return []
    records = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            records.append(json.loads(line))
    return records


def pick_best(records: list[dict]) -> dict | None:
    """Lowest-score record whose cal_file still exists (legacy ``best``)."""
    usable = [r for r in records
              if r.get("cal_file") and r.get("score") is not None
              and os.path.exists(r["cal_file"])]
    if not usable:
        return None
    return min(usable, key=lambda r: r["score"])


def rank_and_compare(records: list[dict],
                     factory_score: float | None) -> tuple[dict | None, bool]:
    """Rank on-Kit sessions and compare the winner with the factory fallback.

    Returns ``(winner, promote)``: the lowest-score record whose cal_file
    still exists, and whether it should be promoted over the factory
    calibration. Promotion requires a winner that *strictly* beats the
    factory score — a tie keeps the frozen known-good cal. A ``None``
    factory score (e.g. the fallback failed to bench) is treated as "no
    baseline": the winner is promoted rather than losing to a missing
    reference.
    """
    winner = pick_best(records)
    if winner is None:
        return None, False
    if factory_score is None:
        return winner, True
    return winner, winner["score"] < factory_score
