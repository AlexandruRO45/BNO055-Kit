"""Clock helpers: paired monotonic/realtime stamps and NTP-step (slew) detection.

The flight-stack sub-source envelope contract stamps samples with integer
nanoseconds: ``sensor_timestamp_ns`` / ``published_at_mono_ns`` from
CLOCK_MONOTONIC (canonical for fusion math, invariant to NTP steps) and
``sensor_timestamp_wall_ns`` from CLOCK_REALTIME (archival only). This module
provides the same primitives for the BNO055 recorder.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

CLOCK_MONOTONIC = time.CLOCK_MONOTONIC
CLOCK_REALTIME = time.CLOCK_REALTIME

# A realtime-vs-monotonic delta change larger than this (seconds) between two
# consecutive samples means the wall clock was stepped/adjusted mid-session.
DEFAULT_SLEW_THRESHOLD_S = 0.050


def now_mono_ns() -> int:
    """CLOCK_MONOTONIC nanoseconds (never steps; canonical for math)."""
    return time.clock_gettime_ns(CLOCK_MONOTONIC)


def now_wall_ns() -> int:
    """CLOCK_REALTIME nanoseconds (archival; may step on NTP sync)."""
    return time.clock_gettime_ns(CLOCK_REALTIME)


def stamp_pair() -> tuple[int, int]:
    """(wall_ns, mono_ns) sampled as close together as possible."""
    mono = now_mono_ns()
    wall = now_wall_ns()
    return wall, mono


@dataclass
class SlewDetector:
    """Detect CLOCK_REALTIME steps by watching the wall-minus-mono delta.

    ``observe`` returns True while the wall clock stays consistent with the
    monotonic clock; once a delta change larger than ``threshold_s`` is seen,
    ``slewed`` latches True for the rest of the session (the exact step
    instant is recorded in ``slew_wall_ns``).
    """

    threshold_s: float = DEFAULT_SLEW_THRESHOLD_S
    _last_delta_ns: int | None = None
    slewing: bool = False
    slew_wall_ns: int | None = None
    slew_delta_ns: int | None = None

    def observe(self, wall_ns: int, mono_ns: int) -> bool:
        delta_ns = wall_ns - mono_ns
        if self._last_delta_ns is not None:
            drift_s = abs(delta_ns - self._last_delta_ns) / 1e9
            if drift_s > self.threshold_s and not self.slewing:
                self.slewing = True
                self.slew_wall_ns = wall_ns
                self.slew_delta_ns = delta_ns - self._last_delta_ns
        self._last_delta_ns = delta_ns
        return not self.slewing

    @property
    def clock_ok(self) -> bool:
        return not self.slewing
