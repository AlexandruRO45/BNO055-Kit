"""Hardware-free tests for the on-Kit calibration procedure and bench metrics."""
from __future__ import annotations

import math

import pytest

from bno055_kit.bench import bench_metrics, make_record
from bno055_kit.calibrate import (
    FACES,
    CalibrationAborted,
    CalibrationTimeout,
    dominant_axis,
    run_calibration,
    sample_face,
    validate_face,
)

G = 9.81


def face_mean(axis: int, sign: float = 1.0) -> tuple[float, float, float]:
    v = [0.0, 0.0, 0.0]
    v[axis] = sign * G
    return (v[0], v[1], v[2])


# ------------------------------------------------------------- validate_face
def test_dominant_axis():
    assert dominant_axis((0.1, -9.8, 0.2)) == 1
    assert dominant_axis((9.8, 0.0, 0.0)) == 0


def test_validate_face_ok():
    res = validate_face(face_mean(2, +1), std_max=0.1, axis=2)
    assert res.ok and res.sign == 1


def test_validate_face_magnitude_gate():
    res = validate_face((0, 0, 5.0), std_max=0.1, axis=2)
    assert not res.ok and "1g" in res.reason


def test_validate_face_axis_gate():
    res = validate_face(face_mean(0), std_max=0.1, axis=2)
    assert not res.ok and "dominant axis" in res.reason


def test_validate_face_jitter_gate():
    res = validate_face(face_mean(2), std_max=0.9, axis=2)
    assert not res.ok and "jitter" in res.reason


def test_validate_face_opposite_sign_gate():
    res = validate_face(face_mean(2, +1), std_max=0.1, axis=2, ref_sign=+1)
    assert not res.ok and "OPPOSITE" in res.reason
    res = validate_face(face_mean(2, -1), std_max=0.1, axis=2, ref_sign=+1)
    assert res.ok and res.sign == -1


# --------------------------------------------------------------- sample_face
class AccelSensor:
    """Replays a scripted list of accel vectors, then repeats the last."""

    def __init__(self, means):
        self._means = list(means)
        self._i = 0

    def raw_acceleration(self):
        v = self._means[min(self._i, len(self._means) - 1)]
        self._i += 1
        return v


def test_sample_face_stats():
    sensor = AccelSensor([(0.0, 0.0, G)] * 25)
    mean, std_max = sample_face(sensor, hold_s=0.5, dt=0.05,
                                sleep=lambda s: None)
    assert std_max < 1e-12
    assert all(abs(m) < 1e-12 for m in mean[:2])
    assert math.isclose(mean[2], G, abs_tol=1e-12)


# ------------------------------------------------------------------- bench
def test_bench_metrics_perfect():
    m = bench_metrics([0.0] * 200, [(0.0, 0.0, 0.0)] * 200, duration_s=30.0)
    assert m["score"] == 0.0


def test_bench_metrics_drift():
    heads = [i * 0.01 for i in range(300)]  # ~3 deg over 30 s -> ~6 deg/min
    m = bench_metrics(heads, [(0.1, 0.0, 0.0)] * 300, duration_s=30.0)
    assert m["drift_deg_per_min"] > 1.0
    assert m["score"] > 1.0


def test_bench_metrics_ignores_frozen_heading_warmup():
    """Regression: right after apply_calibration the heading can sit frozen
    for the first part of the window. The warm-up must be discarded for the
    heading too — otherwise drift/noise collapse to ~0 and a bogus perfect
    score poisons the `best` comparison (observed on-Kit: factory scored
    0.1756 with drift=0.0 / noise=0.002)."""
    heads = [42.0] * 100 + [42.0 + i * 0.1 for i in range(1, 101)]
    accs = [(0.0, 0.0, 0.0)] * 200
    m = bench_metrics(heads, accs, duration_s=30.0)
    assert m["drift_deg_per_min"] > 1.0  # real drift, not the frozen 0.0
    assert m["head_noise_deg"] > 0.01


def test_bench_metrics_records_levels_after_warmup():
    heads = [0.0] * 200
    accs = [(0.0, 0.0, 0.0)] * 200
    levels = [(0, 0, 0, 0)] * 50 + [(3, 3, 3, 3)] * 150
    m = bench_metrics(heads, accs, duration_s=30.0, levels=levels)
    assert m["cal_levels_at_bench"] == [3, 3, 3, 3]  # warm-up excluded


def test_bench_metrics_flags_frozen_heading():
    """A frozen heading (one value all window — the fake-~0 pathology) has
    almost no distinct samples; a live-but-still board steps through the
    1/16 deg quantisation and has many. This is the real gate signal, since
    mag decays to 0 whenever the board is still."""
    frozen = bench_metrics([42.0] * 200, [(0.0, 0.0, 0.0)] * 200,
                           duration_s=30.0)
    assert frozen["head_distinct"] <= 1
    live = bench_metrics([42.0 + (i % 16) * 0.0625 for i in range(200)],
                         [(0.0, 0.0, 0.0)] * 200, duration_s=30.0)
    assert live["head_distinct"] > 5


def test_make_record_keys():
    rec = make_record("cal.json", "abc", {"score": 1.0},
                      when_utc="2026-10-02T00:00:00Z")
    assert rec["cal_file"] == "cal.json"
    assert rec["cal_sha256"] == "abc"
    assert rec["when_utc"] == "2026-10-02T00:00:00Z"
    assert rec["score"] == 1.0


def test_bench_record_cal_file_is_the_path_not_the_source_label(tmp_path):
    """Regression: the history record must carry the *path* that was benched.

    A cal file's internal ``source`` label (e.g. "on-kit calibrate") is not a
    path, so storing it made ``pick_best`` drop every on-Kit session with
    "no record has a score and a readable cal file".
    """
    from bno055_kit.calibration import pick_best

    cal_path = tmp_path / "cal_20261005_095233.json"
    cal_path.write_text("{}")
    rec = make_record(str(cal_path), "fa41", {"score": 99.3858})
    assert rec["cal_file"] == str(cal_path)
    assert rec["cal_file"] != "on-kit calibrate"
    # ... and with the file present, ranking finds it.
    assert pick_best([rec]) is rec
    # A label stored instead of a path is (correctly) dropped.
    assert pick_best([make_record("on-kit calibrate", "fa41",
                                  {"score": 99.3858})]) is None


# --------------------------------------------------------- run_calibration
class FakeSample:
    def __init__(self, lin_acc, cal):
        self.lin_acc = lin_acc
        self.cal = cal


class ScriptedSensor:
    """Full fake for run_calibration: scripted face samples, levels driven
    by the prompts (each prompt converges the phase it announces)."""

    SAMPLES_PER_FACE = 5  # hold_s=0.5 / dt=0.1 in the tests

    def __init__(self, face_means=None):
        if face_means is None:
            face_means = [face_mean(2, +1), face_mean(2, -1), face_mean(0, +1),
                          face_mean(0, -1), face_mean(1, +1), face_mean(1, -1)]
        self._means = [m for m in face_means for _ in range(self.SAMPLES_PER_FACE)]
        self.levels = [0, 0, 0, 0]  # sys, gyro, accel, mag
        self.captured = None
        self._i = 0

    def read(self):
        return FakeSample((0.0, 0.0, G), tuple(self.levels))

    def raw_acceleration(self):
        v = self._means[min(self._i, len(self._means) - 1)]
        self._i += 1
        return v

    def capture_calibration(self, source=None, calibrated_at_utc=None):
        from bno055_kit.calibration import Calibration

        self.captured = Calibration(accel_offset=(-1, -2, -3),
                                    mag_offset=(4, 5, 6),
                                    gyro_offset=(7, 8, 9),
                                    accel_radius=1000, mag_radius=700,
                                    source=source,
                                    calibrated_at_utc=calibrated_at_utc)
        return self.captured


def phase_preview(sensor):
    """preview() replacement: each prompt converges the phase it announces.

    Levels must converge *at or before* the prompt that precedes their
    wait loop: gyro at step 1, accel during the faces, mag+sys during the
    figure-8 (the sys wait follows the mag wait, before the capture
    prompt). The live status line is rendered once per prompt so the
    preview path is exercised without consuming face samples.
    """
    def _preview(header, status_fn, _poll_s):
        if "STEP 1/3" in header:
            sensor.levels[1] = 3      # gyro
            status_fn()               # levels line consumes no samples
        elif "FACE" in header:
            sensor.levels[2] = 3      # accel (converges during the faces)
            # deliberately NOT rendered: the face preview consumes accel
            # samples; it is covered directly by test_face_line_* below.
        elif "STEP 3/3" in header:
            sensor.levels[3] = 3      # mag
            sensor.levels[0] = 3      # sys follows during the figure-8
            status_fn()

    return _preview


def _run(sensor, preview, **kw):
    kwargs = dict(out=lambda *a, **k: None, sleep=lambda s: None,
                  monotonic=lambda: 0.0, hold_s=0.5, dt=0.1,
                  timeout_s=1e9)
    kwargs.update(kw)
    return run_calibration(sensor, preview=preview, **kwargs)


def test_run_calibration_full_success():
    sensor = ScriptedSensor()
    cal = _run(sensor, phase_preview(sensor))
    assert cal is sensor.captured
    assert cal.accel_offset == (-1, -2, -3)
    assert cal.source == "on-kit calibrate"


def test_run_calibration_abort_on_eof():
    sensor = ScriptedSensor()

    def boom(*_args):
        raise EOFError

    with pytest.raises(CalibrationAborted):
        _run(sensor, boom)


def test_run_calibration_timeout_gyro():
    sensor = ScriptedSensor()  # levels stay 0 -> gyro never converges
    with pytest.raises(CalibrationTimeout, match="gyro"):
        _run(sensor, lambda *a: None, timeout_s=-1.0)


def test_run_calibration_too_few_faces():
    # every face garbage -> 0/6 validated -> incomplete
    sensor = ScriptedSensor(face_means=[(0.0, 0.0, 1.0)] * 6)
    with pytest.raises(CalibrationTimeout, match="faces validated"):
        _run(sensor, phase_preview(sensor))


def test_run_calibration_skips_bad_face():
    # Z1 garbage on all 3 attempts (15 samples) is skipped; the other five
    # faces validate -> 5/6 >= min_faces(4) -> success (legacy behaviour).
    bad = (0.0, 0.0, 1.0)
    good = [face_mean(2, -1), face_mean(0, +1), face_mean(0, -1),
            face_mean(1, +1), face_mean(1, -1)]
    sensor = ScriptedSensor()
    sensor._means = [bad] * (3 * sensor.SAMPLES_PER_FACE) + \
        [m for m in good for _ in range(sensor.SAMPLES_PER_FACE)]
    cal = _run(sensor, phase_preview(sensor))
    assert cal is sensor.captured


def test_faces_table_matches_legacy():
    assert tuple(f[0] for f in FACES) == ("Z1", "Z2", "X1", "X2", "Y1", "Y2")
    assert tuple(f[2] for f in FACES) == (2, 2, 0, 0, 1, 1)


# ------------------------------------------------------------ live preview
class LevelsSensor:
    def __init__(self, levels):
        self.cal = tuple(levels)

    def read(self):
        return FakeSample((0.0, 0.0, G), self.cal)

    def raw_acceleration(self):
        return (0.0, 0.0, G)


def test_levels_line_ready_only_when_needed_level_at_3():
    from bno055_kit.calibrate import _levels_line

    assert _levels_line(LevelsSensor([0, 3, 3, 3]), (1,)).startswith("READY")
    assert _levels_line(LevelsSensor([0, 2, 3, 3]), (1,)).startswith("wait")
    assert _levels_line(LevelsSensor([2, 3, 3, 3]), (0, 1, 2, 3)).startswith("wait")
    assert "gyro=3" in _levels_line(LevelsSensor([0, 3, 3, 3]), (1,))


def test_face_line_reports_pose_before_the_hold():
    """The preview must apply the *same* gates as validation, so a wrong
    position is visible before Enter instead of after a failed attempt."""
    from bno055_kit.calibrate import _face_line

    no_sleep = lambda _s: None  # noqa: E731 - preview sampling is instant here

    ok = _face_line(AccelSensor([face_mean(2, +1)] * 8), 2, None,
                    window_s=0.4, dt=0.1, sleep=no_sleep)
    assert ok.startswith("READY") and "Z+" in ok

    wrong = _face_line(AccelSensor([face_mean(1, -1)] * 8), 0, None,
                       window_s=0.4, dt=0.1, sleep=no_sleep)
    assert wrong.startswith("adjust") and "expected X" in wrong

    same_sign = _face_line(AccelSensor([face_mean(2, +1)] * 8), 2, +1,
                           window_s=0.4, dt=0.1, sleep=no_sleep)
    assert same_sign.startswith("adjust") and "OPPOSITE" in same_sign


def test_stdin_preview_polls_status_then_returns_on_enter(monkeypatch):
    """Default preview: polls the live line while waiting, returns on Enter."""
    import os

    from bno055_kit.calibrate import _stdin_preview

    r, w = os.pipe()
    os.write(w, b"x\n")   # stray keypress ignored, Enter accepted
    os.close(w)
    monkeypatch.setattr("sys.stdin", os.fdopen(r, "r"))
    preview = _stdin_preview(out=lambda *a, **k: None, color=False)
    preview("header", lambda: "READY: pose correct", 0.01)  # returns, no raise


def test_stdin_preview_raises_eof_on_closed_stdin(monkeypatch):
    import os

    from bno055_kit.calibrate import _stdin_preview

    r, w = os.pipe()
    os.close(w)           # writer closed -> stdin reads as EOF immediately
    monkeypatch.setattr("sys.stdin", os.fdopen(r, "r"))
    preview = _stdin_preview(out=lambda *a, **k: None, color=False)
    with pytest.raises(EOFError):
        preview("header", lambda: "wait:  sys=0", 0.01)


# ------------------------------------------------------------- skip advice
def test_skip_advice_blames_pose_not_the_sensor():
    from bno055_kit.calibrate import _skip_advice

    advice = _skip_advice(["dominant axis is Y, expected X"] * 3)
    assert "reposition" in advice and "sensor itself is fine" in advice


def test_skip_advice_blames_motion():
    from bno055_kit.calibrate import _skip_advice

    advice = _skip_advice(["too much movement (jitter 0.900)",
                           "dominant axis is Y, expected X"])
    assert "still" in advice


def test_skip_advice_blames_calibration_only_when_pose_was_right():
    from bno055_kit.calibrate import _skip_advice

    advice = _skip_advice(["magnitude 4.10 m/s^2 is not ~1g"] * 3)
    assert "more calibration recommended" in advice
