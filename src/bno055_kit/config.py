"""Daemon configuration: YAML file + sane defaults.

Defaults mirror the bench tooling that produced the shipped calibration:
I2C bus 7 (Orin Nano header pins 3/5), address 0x29 (alt 0x28), 100 Hz
NDOF sampling.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import yaml

log = logging.getLogger("bno055_kit.config")

DEFAULTS: dict[str, Any] = {
    "bus": 7,
    "address": 0x29,
    "rate_hz": 100.0,
    "log_dir": "/var/log/bno055",
    "cal_file": "/var/lib/bno055/active.json",
    "slew_threshold_s": 0.05,
    "log_level": "INFO",
    # Reconnect policy for transient I2C failures (mirrors the bounded
    # reopen/backoff used by the flight-stack CAN transport).
    "max_reconnects": 10,
    "reconnect_backoff_s": 1.0,
    "reconnect_backoff_max_s": 30.0,
}


@dataclass
class Config:
    bus: int = DEFAULTS["bus"]
    address: int = DEFAULTS["address"]
    rate_hz: float = DEFAULTS["rate_hz"]
    log_dir: str = DEFAULTS["log_dir"]
    cal_file: str = DEFAULTS["cal_file"]
    slew_threshold_s: float = DEFAULTS["slew_threshold_s"]
    log_level: str = DEFAULTS["log_level"]
    max_reconnects: int = DEFAULTS["max_reconnects"]
    reconnect_backoff_s: float = DEFAULTS["reconnect_backoff_s"]
    reconnect_backoff_max_s: float = DEFAULTS["reconnect_backoff_max_s"]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Config:
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            log.warning("ignoring unknown config keys: %s", ", ".join(unknown))
        return cls(**{k: v for k, v in data.items() if k in known})

    @classmethod
    def load(cls, path: str | Path | None) -> Config:
        if path is None:
            return cls()
        if not Path(path).is_file():
            # The unit always passes --config; a missing file (unit installed
            # without the installer) must not traceback-loop the service.
            log.warning("config file %s not found — using defaults", path)
            return cls()
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError(f"config {path} must be a YAML mapping")
        return cls.from_dict(data)
