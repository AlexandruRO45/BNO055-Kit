"""Bench metrics shared by ``bench`` and post-calibration verification.

Ported from the legacy bench tooling (``imu.py do_bench``): at rest, a good
calibration shows near-zero linear-accel bias, near-zero unwrapped heading
drift, and low heading noise. ``score = bias + |drift| + noise`` (lower is
better) — the same ranking key the legacy ``best`` command used.
"""
from __future__ import annotations

import time
from collections.abc import Callable, Sequence

import numpy as np

# Fraction of the window dropped as warm-up, for BOTH heading and accel.
# Right after apply_calibration() flips the part into NDOF the fusion has
# not re-converged: the heading can sit frozen at one value until the mag
# filter initialises, which would score drift=0/noise=0 — a fake perfect
# score. The legacy bench discarded this settle for accel only; a frozen
# heading is far more damaging, so the same discard applies to heading.
_WARMUP_MIN_S = 0.5
_WARMUP_FRACTION = 0.25


def bench_metrics(heads: Sequence[float], accs: Sequence[Sequence[float]],
                  duration_s: float, *,
                  levels: Sequence[Sequence[int]] | None = None) -> dict:
    """Score a bench window: bias (m/s^2), drift (deg/min), noise (deg).

    ``levels`` (optional per-sample (sys, gyro, accel, mag)) is recorded so a
    window benched before full convergence is visible after the fact.
    """
    heads_arr = np.asarray(heads, dtype=float)
    # Drop the settle window from heading too, not just accel.
    warm = max(int(round(_WARMUP_MIN_S / max(duration_s, 1e-9) * len(heads_arr))),
               int(_WARMUP_FRACTION * len(heads_arr)))
    warm = min(warm, max(len(heads_arr) - 2, 0))
    settled = np.degrees(np.unwrap(np.radians(heads_arr[warm:])))
    drift_per_min = float((settled[-1] - settled[0]) * 60.0 / duration_s)
    # Guard short windows so the slice can never be empty (mean of [] -> NaN).
    accs_arr = np.asarray(accs[warm:], dtype=float)
    bias = float(np.linalg.norm(accs_arr, axis=1).mean())
    head_noise = float(np.degrees(np.std(np.radians(settled))))
    # A *frozen* heading (the sensor returning one value for the whole
    # window — the real pathology behind a fake ~0 score) shows up as almost
    # no distinct samples. A live-but-still board still steps through the
    # 1/16 deg heading quantisation, so it has many distinct values.
    head_distinct = int(len(np.unique(np.round(settled, 4))))
    out = {
        "bias_linacc_m_s2": round(bias, 4),
        "drift_deg_per_min": round(drift_per_min, 3),
        "head_noise_deg": round(head_noise, 4),
        "head_distinct": head_distinct,
        "score": round(bias + abs(drift_per_min) + head_noise, 4),
    }
    if levels is not None and len(levels) > warm:
        lv = np.asarray(list(levels[warm:]), dtype=float)
        out["cal_levels_at_bench"] = [int(round(x)) for x in lv.mean(axis=0)]
    return out


def sample_bench_window(sensor, duration_s: float, *,
                        sleep: Callable[[float], None] = time.sleep,
                        monotonic: Callable[[], float] = time.monotonic,
                        dt: float = 0.01) -> tuple[list[float], list[tuple], list[tuple]]:
    """Sample heading + linear accel + cal levels at ~100 Hz for ``duration_s``."""
    t0 = monotonic()
    heads, accs, levels = [], [], []
    while monotonic() - t0 < duration_s:
        s = sensor.read()
        heads.append(s.euler[0])
        accs.append(s.lin_acc)
        levels.append(tuple(s.cal))
        sleep(dt)
    return heads, accs, levels


def make_record(cal_file: str, cal_sha256: str | None, metrics: dict, *,
                when_utc: str | None = None) -> dict:
    """History record: legacy keys plus the kit's sha256/ISO-8601 extras."""
    if when_utc is None:
        when_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    return {
        "cal_file": str(cal_file),
        "cal_sha256": cal_sha256,
        "when_utc": when_utc,
        **metrics,
    }
