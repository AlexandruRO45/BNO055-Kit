"""Interactive on-Kit calibration procedure (port of the legacy bench flow).

Same 3-step operator procedure as the bench tooling, with English prompts,
run on the Kit with the sensor already glued to the drone:

    PAS 1/3 (GYRO)  : board flat and still until gyro status reaches 3
    PAS 2/3 (ACCEL) : six validated faces against the gravity vector
    PAS 3/3 (MAG)   : figure-8 in free air until mag status reaches 3

Every prompt shows a *live preview line* while waiting for [Enter]: the
pose/calibration status is sampled continuously, so the operator only
presses Enter when it says READY — a wrong position is caught before the
2.5 s hold is wasted, not after three failed attempts.

The face validators are pure functions so the whole procedure is testable
without hardware; the orchestration takes an injectable sensor, preview
and print so tests can script the operator.
"""
from __future__ import annotations

import math
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from .calibration import Calibration
from .sensor import Bno055

G = 9.81  # m/s^2

# Face gates (identical tolerances to the legacy bench tooling).
FACE_MAG_TOL = 2.0    # |accel| must be within G +/- this
FACE_STD_MAX = 0.4    # per-axis std ceiling: "too much movement"
FACE_ATTEMPTS = 3

# (name, how-to-hint, dominant axis index, expected sign or None)
FACES: tuple[tuple[str, str, int], ...] = (
    ("Z1", "flat on the table, chip facing UP", 2),
    ("Z2", "flipped 180 deg, chip facing DOWN (rest it on a book/box)", 2),
    ("X1", "VERTICAL on one edge", 0),
    ("X2", "same edge, rotated 180 deg from X1", 0),
    ("Y1", "VERTICAL on the other edge (90 deg from X1)", 1),
    ("Y2", "same edge, rotated 180 deg from Y1", 1),
)


@dataclass(frozen=True)
class FaceResult:
    ok: bool
    reason: str
    mean: tuple[float, float, float]
    std_max: float
    sign: int | None = None


def dominant_axis(v: Sequence[float]) -> int:
    a = [abs(x) for x in v]
    return a.index(max(a))  # 0=X, 1=Y, 2=Z


def validate_face(mean: Sequence[float], std_max: float, axis: int,
                  ref_sign: int | None = None) -> FaceResult:
    """Gate one sampled face against the gravity vector (pure function)."""
    mag = math.sqrt(sum(c * c for c in mean))
    dom = dominant_axis(mean)
    sign = int(math.copysign(1, mean[dom]))
    if not (G - FACE_MAG_TOL < mag < G + FACE_MAG_TOL):
        ok, reason = False, f"magnitude {mag:.2f} m/s^2 is not ~1g"
    elif dom != axis:
        ok, reason = False, f"dominant axis is {'XYZ'[dom]}, expected {'XYZ'[axis]}"
    elif std_max > FACE_STD_MAX:
        ok, reason = False, f"too much movement (jitter {std_max:.3f})"
    elif ref_sign is not None and sign != -ref_sign:
        ok, reason = False, (f"sign must be OPPOSITE to the first "
                             f"{'XYZ'[axis]} face")
    else:
        ok, reason = True, f"validated ({'XYZ'[dom]}{'+' if sign > 0 else '-'})"
    return FaceResult(ok=ok, reason=reason, mean=tuple(mean),
                      std_max=std_max, sign=sign)


def sample_face(sensor, hold_s: float = 2.5, dt: float = 0.1, *,
                sleep: Callable[[float], None] = time.sleep) -> tuple[tuple, float]:
    """Average accel over ``hold_s`` at ~1/dt Hz; return (mean, max std)."""
    xs, ys, zs = [], [], []
    for _ in range(int(hold_s / dt)):
        x, y, z = sensor.raw_acceleration()
        xs.append(x)
        ys.append(y)
        zs.append(z)
        sleep(dt)

    def stats(v):
        m = sum(v) / len(v)
        var = sum((x - m) ** 2 for x in v) / len(v)
        return m, math.sqrt(var)

    mx, sx = stats(xs)
    my, sy = stats(ys)
    mz, sz = stats(zs)
    return (mx, my, mz), max(sx, sy, sz)


class CalibrationAborted(Exception):
    """Raised when the operator cancels (EOF/Ctrl-C) mid-procedure."""


class CalibrationTimeout(Exception):
    """Raised when a convergence phase never reaches level 3."""


def _levels_line(sensor, need: tuple[int, ...]) -> str:
    """Live preview for a level-gated step: READY once the needed level hits 3."""
    st = sensor.read().cal
    line = f"sys={st[0]} gyro={st[1]} accel={st[2]} mag={st[3]}"
    return ("READY: " if all(st[i] >= 3 for i in need) else "wait:  ") + line


def _face_line(sensor, axis: int, ref_sign: int | None, *,
               window_s: float, dt: float,
               sleep: Callable[[float], None]) -> str:
    """Live preview for a face: short sample window -> the same gates as the
    real validation, so READY here means the attempt will pass."""
    mean, std_max = sample_face(sensor, hold_s=window_s, dt=dt, sleep=sleep)
    res = validate_face(mean, std_max, axis, ref_sign=ref_sign)
    if res.ok:
        sign = "+" if res.sign > 0 else "-"
        return f"READY: pose correct ({'XYZ'[axis]}{sign}) — [Enter]"
    return f"adjust: {res.reason}"


def _stdin_preview(*, out: Callable[..., None],
                   color: bool) -> Callable[[str, Callable[[], str], float], None]:
    """Default preview loop: print the header, then poll a live status line
    every ``poll_s`` until [Enter] arrives on stdin (EOF -> abort)."""
    import select

    def _preview(header: str, status_fn: Callable[[], str], poll_s: float) -> None:
        out(header)
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], poll_s)
            if ready:
                ch = sys.stdin.read(1)
                if ch == "":
                    raise EOFError  # stdin closed -> same as Ctrl-C
                if ch in ("\n", "\r"):
                    out("")
                    return
            else:
                line = status_fn()
                if color:
                    line = (f"\033[32m{line}\033[0m" if line.startswith("READY")
                            else f"\033[33m{line}\033[0m")
                out(f"\r    {line:<72}", end="", flush=True)

    return _preview


def _skip_advice(reasons: list[str]) -> str:
    """Why did all attempts fail? Only blame the calibration when the pose
    itself was right every time — a wrong position is not a sensor problem."""
    kinds = set()
    for r in reasons:
        if "jitter" in r:
            kinds.add("motion")
        elif "dominant axis" in r or "OPPOSITE" in r:
            kinds.add("pose")
        else:
            kinds.add("magnitude")
    if kinds == {"pose"}:
        return ("the pose never matched — reposition the board and run "
                "again; the sensor itself is fine")
    if "motion" in kinds:
        return ("the board kept moving — hold it perfectly still and run "
                "again")
    return "gravity magnitude off in a correct pose — more calibration recommended"


def _wait_level(sensor, idx: int, label: str, *,
                out: Callable[..., None], sleep: Callable[[float], None],
                monotonic: Callable[[], float], timeout_s: float,
                poll_s: float = 0.3) -> None:
    out(f"    waiting for {label} = 3 ...")
    t0 = monotonic()
    while True:
        st = sensor.read().cal
        if st[idx] >= 3:
            break
        if monotonic() - t0 > timeout_s:
            raise CalibrationTimeout(
                f"{label} never reached 3 (last status={tuple(st)})")
        out(f"\r    sys={st[0]} gyro={st[1]} accel={st[2]} mag={st[3]}   ",
            end="", flush=True)
        sleep(poll_s)
    out(f"\r    {label}: OK (3)                              ")


def run_calibration(sensor: Bno055, *,
                    preview: Callable[[str, Callable[[], str], float], None]
                    | None = None,
                    out: Callable[..., None] = print,
                    sleep: Callable[[float], None] = time.sleep,
                    monotonic: Callable[[], float] = time.monotonic,
                    hold_s: float = 2.5, dt: float = 0.1,
                    timeout_s: float = 120.0,
                    min_faces: int = 4,
                    preview_poll_s: float = 0.4,
                    preview_window_s: float = 0.4,
                    color: bool | None = None) -> Calibration:
    """Run the full interactive procedure; return the captured Calibration.

    Every prompt runs ``preview(header, status_fn, poll_s)``: a live status
    line (pose gates / calibration levels) refreshes until the operator
    presses [Enter] — the default implementation reads the sensor while it
    waits. Raises CalibrationAborted on EOF/Ctrl-C and CalibrationTimeout
    when a convergence phase stalls; returns normally even if some faces
    were skipped, as long as ``min_faces`` validated (legacy behaviour).
    """
    if preview is None:
        if color is None:
            color = sys.stdout.isatty()
        preview = _stdin_preview(out=out, color=color)

    out("=== BNO055 CALIBRATION (on-Kit) ===")
    try:
        preview("\n>>> STEP 1/3 (GYRO): board flat on the ground, perfectly "
                "STILL.\n    [Enter] when the live line says READY ...",
                lambda: _levels_line(sensor, (1,)), preview_poll_s)
    except (EOFError, KeyboardInterrupt) as exc:
        raise CalibrationAborted("aborted at step 1/3") from exc
    _wait_level(sensor, 1, "gyro", out=out, sleep=sleep, monotonic=monotonic,
                timeout_s=timeout_s)

    out("\nSTEP 2/3 (ACCEL): six faces, each validated against the gravity "
        "vector.")
    axis_sign: dict[int, int] = {}
    ok_faces = 0
    for name, how, axis in FACES:
        reasons: list[str] = []
        for attempt in range(1, FACE_ATTEMPTS + 1):
            ref_sign = axis_sign.get(axis)  # bind per-face for the preview
            try:
                preview(f"\n>>> FACE {name}: {how}\n"
                        f"    Hold perfectly still for {hold_s:.1f}s. "
                        f"[Enter] when READY, then DO NOT move ...",
                        lambda a=axis, rs=ref_sign: _face_line(
                            sensor, a, rs, window_s=preview_window_s,
                            dt=dt, sleep=sleep),
                        preview_poll_s)
            except (EOFError, KeyboardInterrupt) as exc:
                raise CalibrationAborted(f"aborted at face {name}") from exc
            mean, std_max = sample_face(sensor, hold_s=hold_s, dt=dt,
                                        sleep=sleep)
            res = validate_face(mean, std_max, axis,
                                ref_sign=axis_sign.get(axis))
            out(f"    read: mag={math.sqrt(sum(c * c for c in res.mean)):5.2f}"
                f" m/s^2  axis={'XYZ'[dominant_axis(res.mean)]}"
                f"  sign={res.sign:+d}  jitter={res.std_max:.3f}")
            if res.ok:
                out(f"    [OK] face {name} {res.reason}")
                ok_faces += 1
                axis_sign.setdefault(axis, res.sign)
                break
            reasons.append(res.reason)
            out(f"    [X] {res.reason}  attempt {attempt}/{FACE_ATTEMPTS}.")
        else:
            out(f"    [!] face {name} SKIPPED after {FACE_ATTEMPTS} attempts"
                f" — {_skip_advice(reasons)}.")

    out(f"\n    faces validated: {ok_faces}/6")
    if ok_faces < min_faces:
        raise CalibrationTimeout(
            f"only {ok_faces}/6 faces validated (need >= {min_faces})")

    _wait_level(sensor, 2, "accel", out=out, sleep=sleep, monotonic=monotonic,
                timeout_s=timeout_s)
    try:
        preview("\n>>> STEP 3/3 (MAG): hold it in the air, away from metal and"
                " motors, slow figure-8.\n    [Enter] when the live line says "
                "READY ...",
                lambda: _levels_line(sensor, (3,)), preview_poll_s)
    except (EOFError, KeyboardInterrupt) as exc:
        raise CalibrationAborted("aborted at step 3/3") from exc
    _wait_level(sensor, 3, "mag", out=out, sleep=sleep, monotonic=monotonic,
                timeout_s=timeout_s)
    _wait_level(sensor, 0, "sys", out=out, sleep=sleep, monotonic=monotonic,
                timeout_s=timeout_s)

    try:
        preview("\n>>> All levels at 3! Keep the board still, [Enter] to "
                "capture ...",
                lambda: _levels_line(sensor, (0, 1, 2, 3)), preview_poll_s)
    except (EOFError, KeyboardInterrupt) as exc:
        raise CalibrationAborted("aborted at capture") from exc
    cal = sensor.capture_calibration(source="on-kit calibrate")
    out(f"[OK] captured: accel={list(cal.accel_offset)} "
        f"gyro={list(cal.gyro_offset)} mag={list(cal.mag_offset)} "
        f"radii={cal.accel_radius}/{cal.mag_radius}")
    return cal
