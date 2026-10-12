"""Align a BNO055 session with a ``candump -L`` capture (clock offset + jitter).

Both logs are wall-clock stamped, but neither stamp is the physical sample
instant: the IMU stamp is taken after an I2C transaction (sub-millisecond to
few-ms, depending on bus load), and a candump stamp is taken when candump's
userspace process is scheduled after the kernel queued the frame. The two
pipelines therefore sit at a constant relative offset plus jitter.

Two methods, in order of trust:

``correlation``
    Decode the CAN IMU frames (902 accel / 903 gyro), resample both series
    onto a common uniform grid, and cross-correlate to recover the offset.
    This measures the true pipeline offset and needs no shared clock.

``anchor``
    Fallback when the capture carries no IMU frames: take the residual from
    each sample's real wall stamp to the nearest candump frame. Reports the
    wall-clock agreement plus jitter; assumes both processes shared one
    CLOCK_REALTIME (same machine). An NTP step inside the session shows up
    honestly in the residual spread (and the ``clock_ok`` note).
"""
from __future__ import annotations

import json
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .candump import CandumpFrame, decode_imu_frame, parse_candump

IMU_GYRO_IDS = (0x387,)
IMU_ACCEL_IDS = (0x386,)

# A channel whose detrended magnitude series has no real correlation
# structure (peak below this) carries no timing information — typically
# |accel| pinned near 1 g and quantized by the CAN payload.
_MIN_PEAK_CORR = 0.3


@dataclass
class ImuSeries:
    """IMU session in wall-clock seconds + monotonic seconds.

    ``t_wall_s`` is the anchor projection of the monotonic timeline (valid
    while the wall clock is stable); ``t_wall_raw_s`` is the wall stamp the
    daemon actually wrote per sample — the only trustworthy wall timeline
    after an NTP step (``clock_ok`` False).
    """

    t_wall_s: np.ndarray
    t_wall_raw_s: np.ndarray
    t_mono_s: np.ndarray
    gyro_mag: np.ndarray
    accel_mag: np.ndarray
    clock_ok: bool = True
    anchor_uncertainty_s: float = 0.0
    session_id: str | None = None
    n_bad_lines: int = 0


@dataclass
class CanSeries:
    """All frame times (anchor residuals) + per-signal series for correlation."""

    t_wall_s: np.ndarray
    t_gyro_s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    gyro_mag: np.ndarray = field(default_factory=lambda: np.zeros(0))
    t_accel_s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    accel_mag: np.ndarray = field(default_factory=lambda: np.zeros(0))
    n_frames: int = 0


@dataclass
class SyncResult:
    offset_s: float
    jitter_std_s: float
    max_abs_residual_s: float
    n: int
    method: str
    confidence: str
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "offset_s": round(self.offset_s, 6),
            "jitter_std_s": round(self.jitter_std_s, 6),
            "max_abs_residual_s": round(self.max_abs_residual_s, 6),
            "n": self.n,
            "method": self.method,
            "confidence": self.confidence,
            "notes": self.notes,
        }


# --------------------------------------------------------------------- load
def load_imu_session(session_dir: str | Path) -> ImuSeries:
    """Read ``session.json`` + ``imu.jsonl`` and project monotonic → wall clock."""
    session_dir = Path(session_dir)
    manifest = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
    start_wall = int(manifest["start_wall_ns"])
    start_mono = int(manifest["start_mono_ns"])

    t_mono, t_wall, gyro_mag, accel_mag = [], [], [], []
    clock_ok = True
    n_bad = 0
    with open(session_dir / "imu.jsonl", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                # A truncated/NUL-padded tail (e.g. a copy from a Windows
                # volume) must not kill analysis of the whole session.
                n_bad += 1
                continue
            mono = row["t_mono_ns"] / 1e9
            t_mono.append(mono)
            t_wall.append(row["t_wall_ns"] / 1e9)
            gyro_mag.append(float(np.linalg.norm(row["gyro"])))
            accel_mag.append(float(np.linalg.norm(row["lin_acc"])))
            clock_ok = clock_ok and bool(row.get("clock_ok", True))

    if not t_mono:
        raise ValueError(f"no samples in {session_dir / 'imu.jsonl'}")

    mono_arr = np.asarray(t_mono)
    # Anchor projection: where the monotonic timeline *should* be on wall clock.
    projected = start_wall / 1e9 + (mono_arr - start_mono / 1e9)
    return ImuSeries(
        t_wall_s=projected,
        t_wall_raw_s=np.asarray(t_wall),
        t_mono_s=mono_arr - mono_arr[0],
        gyro_mag=np.asarray(gyro_mag),
        accel_mag=np.asarray(accel_mag),
        clock_ok=clock_ok,
        anchor_uncertainty_s=float(manifest.get("spawn_jitter_s", 0.0)),
        session_id=manifest.get("session_id"),
        n_bad_lines=n_bad,
    )


def load_can_series(
    path: str | Path,
    *,
    gyro_ids: Sequence[int] = IMU_GYRO_IDS,
    accel_ids: Sequence[int] = IMU_ACCEL_IDS,
) -> CanSeries:
    """All frame times (for anchor residuals) + decoded accel/gyro magnitudes."""
    times: list[float] = []
    gt: list[float] = []
    gm: list[float] = []
    at: list[float] = []
    am: list[float] = []
    n = 0
    frame: CandumpFrame
    for frame in parse_candump(path):
        n += 1
        times.append(frame.t_wall_s)
        dec = decode_imu_frame(frame)
        if dec is None:
            continue
        s = dec["signals"]
        if dec["kind"] == "aspeed" and frame.arbitration_id in gyro_ids:
            # deg/s here; correlation is scale-invariant (corrcoef), so no
            # rad/s conversion is needed for the timing estimate.
            gt.append(dec["t_wall_s"])
            gm.append(float(np.linalg.norm(
                [s["AngSpeedX"], s["AngSpeedY"], s["AngSpeedZ"]])))
        elif dec["kind"] == "acc" and frame.arbitration_id in accel_ids:
            at.append(dec["t_wall_s"])
            am.append(float(np.linalg.norm(
                [s["Acceleration_X"], s["Acceleration_Y"], s["Acceleration_Z"]])))

    return CanSeries(
        t_wall_s=np.asarray(times),
        t_gyro_s=np.asarray(gt) if gt else np.zeros(0),
        gyro_mag=np.asarray(gm) if gm else np.zeros(0),
        t_accel_s=np.asarray(at) if at else np.zeros(0),
        accel_mag=np.asarray(am) if am else np.zeros(0),
        n_frames=n,
    )


# -------------------------------------------------------------------- align
def _resample(t: np.ndarray, v: np.ndarray, grid: np.ndarray) -> np.ndarray:
    return np.interp(grid, t, v, left=np.nan, right=np.nan)


def _detrend(v: np.ndarray) -> np.ndarray:
    """Remove linear trend + mean: correlate the *motion*, not the bias."""
    t = np.arange(v.size, dtype=float)
    good = np.isfinite(v)
    if good.sum() < 4:
        return np.zeros_like(v)
    coef = np.polyfit(t[good], v[good], 1)
    out = v - np.polyval(coef, t)
    return np.nan_to_num(out, nan=0.0)


def _xcorr_offset(
    a: np.ndarray, b: np.ndarray, fs: float, max_lag_s: float
) -> tuple[float, float]:
    """Lag (seconds) maximising correlation of a(t) against b(t - lag).

    Returns (lag, peak_corr). If a is the same physical signal stamped on
    timeline A (a(t) = f(t - e_a)) and b on timeline B (b(t) = f(t - e_b)),
    the peak sits at lag = e_a - e_b, i.e. the negative of the B-minus-A
    offset convention used by SyncResult.offset_s. A degenerate window
    returns a -inf peak so callers can reject it as "no correlation
    structure" (0.0 would be indistinguishable from a real zero offset).
    """
    max_lag = int(max_lag_s * fs)
    if max_lag < 1 or a.size < 8:
        return 0.0, float("-inf")
    best_lag, best_val = 0, -np.inf
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            x, y = a[lag:], b[: a.size - lag]
        else:
            x, y = a[: a.size + lag], b[-lag:]
        m = np.isfinite(x) & np.isfinite(y)
        if m.sum() < 8:
            continue
        c = float(np.corrcoef(x[m], y[m])[0, 1])
        if np.isfinite(c) and c > best_val:
            best_val, best_lag = c, lag
    # Parabolic refinement around the integer peak.
    def corr(lag: int) -> float:
        if lag >= 0:
            x, y = a[lag:], b[: a.size - lag]
        else:
            x, y = a[: a.size + lag], b[-lag:]
        m = np.isfinite(x) & np.isfinite(y)
        if m.sum() < 8:
            return -np.inf
        c = np.corrcoef(x[m], y[m])[0, 1]
        return float(c) if np.isfinite(c) else -np.inf

    y0, y1, y2 = corr(best_lag - 1), corr(best_lag), corr(best_lag + 1)
    denom = y0 - 2 * y1 + y2
    frac = 0.5 * (y0 - y2) / denom if abs(denom) > 1e-12 else 0.0
    frac = float(np.clip(frac, -1.0, 1.0))
    return (best_lag + frac) / fs, y1


def align_anchor(imu: ImuSeries, can: CanSeries) -> SyncResult:
    """Wall-clock agreement via the session's real wall stamps (no shared signal).

    Uses the per-sample wall stamps the daemon actually wrote (not the anchor
    projection), so an NTP step mid-session shows up honestly in the residual
    spread instead of being smoothed away by the projection.
    """
    notes: list[str] = []
    if can.t_wall_s.size == 0:
        raise ValueError("candump capture has no frames")
    if not imu.clock_ok:
        notes.append("IMU session reported a realtime clock step; "
                     "residual spread reflects the step, not jitter")
    if imu.anchor_uncertainty_s > 0:
        notes.append(
            f"anchor uncertainty ±{imu.anchor_uncertainty_s * 1e6:.0f} us "
            "(session spawn jitter)"
        )
    # Sign convention shared with the correlation method:
    # offset = can_wall - imu_wall (add to IMU time to reach CAN time).
    imu_t = imu.t_wall_raw_s
    nearest = np.searchsorted(can.t_wall_s, imu_t)
    nearest = np.clip(nearest, 0, can.t_wall_s.size - 1)
    lo = np.clip(nearest - 1, 0, can.t_wall_s.size - 1)
    a = can.t_wall_s[lo]
    b = can.t_wall_s[nearest]
    nearest_can = np.where(np.abs(a - imu_t) <= np.abs(b - imu_t), a, b)
    resid = nearest_can - imu_t
    return SyncResult(
        offset_s=float(np.median(resid)),
        jitter_std_s=float(np.std(resid)),
        max_abs_residual_s=float(np.max(np.abs(resid))),
        n=int(resid.size),
        method="anchor",
        confidence="low",
        notes=notes + ["anchor method assumes one shared CLOCK_REALTIME"],
    )


def align_correlation(
    imu: ImuSeries, can: CanSeries, *, fs: float = 20.0, max_lag_s: float = 5.0
) -> SyncResult:
    """Cross-correlate accel/gyro magnitude series to recover the true offset."""
    notes: list[str] = []
    offsets: list[float] = []
    peaks: list[float] = []
    for imu_t, imu_series, can_t, can_series, label in (
        (imu.t_wall_s, imu.gyro_mag, can.t_gyro_s, can.gyro_mag, "gyro"),
        (imu.t_wall_s, imu.accel_mag, can.t_accel_s, can.accel_mag, "accel"),
    ):
        if can_series.size < 8 or imu_series.size < 8:
            continue
        t0 = max(imu_t[0], can_t[0])
        t1 = min(imu_t[-1], can_t[-1])
        if t1 - t0 < 2.0 / fs:
            continue
        grid = np.arange(t0, t1, 1.0 / fs)
        a = _detrend(_resample(imu_t, imu_series, grid))
        b = _detrend(_resample(can_t, can_series, grid))
        lag, peak = _xcorr_offset(a, b, fs, max_lag_s)
        if not np.isfinite(peak) or peak < _MIN_PEAK_CORR:
            # Degenerate channel (e.g. |accel| dominated by gravity and
            # quantized to one CAN LSB is flat after detrending: any "peak"
            # is float noise) — it carries no timing information and must
            # not dilute the channels that do.
            notes.append(f"{label}: skipped (no correlation structure"
                         f", peak {peak:.3f})")
            continue
        offset = -lag  # convert to the shared can_wall - imu_wall convention
        offsets.append(offset)
        peaks.append(peak)
        notes.append(f"{label}: offset {offset * 1e3:+.1f} ms, corr {peak:.3f}")

    if not offsets:
        raise ValueError(
            "no overlapping CAN IMU frames to correlate — "
            "capture must contain IDs 0x386/0x387 (902/903)"
        )
    offset = float(np.mean(offsets))
    peak = float(np.mean(peaks))
    confidence = "high" if peak > 0.6 else "medium" if peak > 0.3 else "low"
    if not imu.clock_ok:
        notes.append("IMU session reported a realtime clock step")
    notes.append(f"grid {fs:g} Hz, lag window ±{max_lag_s:g} s")
    notes.append("max_abs_residual_s = grid resolution (correlation bound, "
                 "not a measured residual)")
    return SyncResult(
        offset_s=offset,
        jitter_std_s=float(np.std(offsets)) if len(offsets) > 1 else 0.0,
        max_abs_residual_s=1.0 / fs,
        n=int(min(imu.t_wall_s.size, can.t_wall_s.size)),
        method="correlation",
        confidence=confidence,
        notes=notes,
    )


def align(
    imu: ImuSeries,
    can: CanSeries,
    *,
    method: str = "auto",
    fs: float = 20.0,
    max_lag_s: float = 5.0,
) -> SyncResult:
    if method == "correlation":
        return align_correlation(imu, can, fs=fs, max_lag_s=max_lag_s)
    if method == "anchor":
        return align_anchor(imu, can)
    try:
        return align_correlation(imu, can, fs=fs, max_lag_s=max_lag_s)
    except ValueError:
        return align_anchor(imu, can)
