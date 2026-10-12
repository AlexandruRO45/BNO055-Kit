"""Tests for calibration loading/validation/persistence."""
from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

from bno055_kit.calibration import (
    Calibration,
    CalibrationError,
    append_history,
    file_sha256,
    load_calibration,
    pick_best,
    read_history,
    save_calibration,
)


def test_round_trip(cal_dict):
    cal = Calibration.from_dict(cal_dict, source="unit-test")
    assert cal.accel_offset == (-17, -21, -27)
    assert cal.mag_radius == 764
    assert cal.to_dict()["accel_offset"] == [-17, -21, -27]


def test_missing_key_rejected(cal_dict):
    del cal_dict["gyro_offset"]
    with pytest.raises(CalibrationError, match="gyro_offset"):
        Calibration.from_dict(cal_dict)


def test_out_of_range_offset_rejected(cal_dict):
    cal_dict["accel_offset"] = [0, 0, 40000]
    with pytest.raises(CalibrationError, match="int16"):
        Calibration.from_dict(cal_dict)


def test_negative_radius_rejected(cal_dict):
    cal_dict["mag_radius"] = -1
    with pytest.raises(CalibrationError, match="mag_radius"):
        Calibration.from_dict(cal_dict)


def test_load_from_file(tmp_path, cal_dict):
    path = tmp_path / "cal.json"
    path.write_text(json.dumps(cal_dict), encoding="utf-8")
    cal = load_calibration(path)
    assert cal.sha256 == file_sha256(path)
    assert cal.source == str(path)


def test_load_missing_file(tmp_path):
    with pytest.raises(CalibrationError, match="not found"):
        load_calibration(tmp_path / "nope.json")


def test_load_bad_json(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(CalibrationError, match="not valid JSON"):
        load_calibration(path)


def test_shipped_calibrations_validate():
    """The calibs shipped in the kit bundle must load cleanly: the mutable
    active seed AND the permanent factory fallback (which the fallback
    mechanism restores into active.json — it must always validate)."""
    kit_calibs = Path(__file__).resolve().parents[1] / "calibs"
    for name in ("active.json", "factory_fallback.json"):
        cal = load_calibration(kit_calibs / name)
        assert cal.accel_radius > 0 and cal.mag_radius > 0


def test_history_not_tracked_in_repo():
    """History is a per-instance artifact generated next to the active cal
    (on the Kit / on the bench) — the repo *ships* none.

    Checks git-tracking, not filesystem existence: a developer may keep a
    gitignored local reference copy under calibs/ (the legacy bench writes
    one there), which must not fail the suite.
    """
    repo = Path(__file__).resolve().parents[1]
    tracked = subprocess.run(["git", "-C", str(repo), "ls-files",
                              "calibs/history.jsonl"],
                             capture_output=True, text=True)
    if tracked.returncode != 0:
        pytest.skip("not a git checkout — nothing to be tracked")
    assert tracked.stdout.strip() == ""


# ------------------------------------------------------------------ save
def test_save_round_trip(tmp_path, cal_dict):
    cal_dict["calibrated_at_utc"] = "2026-10-02T00:00:00Z"
    cal = Calibration.from_dict(cal_dict, source="unit-test")
    path = tmp_path / "sub" / "cal_x.json"
    sha = save_calibration(cal, path)
    assert sha == file_sha256(path)
    back = load_calibration(path)
    assert back.accel_offset == cal.accel_offset
    assert back.mag_offset == cal.mag_offset
    assert back.gyro_offset == cal.gyro_offset
    assert back.accel_radius == cal.accel_radius
    assert back.mag_radius == cal.mag_radius
    assert back.calibrated_at_utc == "2026-10-02T00:00:00Z"


def test_save_never_embeds_own_sha256(tmp_path, cal_dict):
    cal = Calibration.from_dict(cal_dict, source="s", sha256="deadbeef")
    path = tmp_path / "cal.json"
    save_calibration(cal, path)
    assert "sha256" not in json.loads(path.read_text(encoding="utf-8"))


def test_save_rejects_invalid_before_write(tmp_path):
    """A Calibration can only exist validated (from_dict gates it), so the
    write path is safe by construction — prove the gate itself holds."""
    with pytest.raises(CalibrationError):
        Calibration.from_dict({"accel_offset": [1, 2, 3]})


def test_save_leaves_no_tmp_file(tmp_path, cal_dict):
    cal = Calibration.from_dict(cal_dict)
    path = tmp_path / "cal.json"
    save_calibration(cal, path)
    assert not (tmp_path / "cal.json.tmp").exists()


# ---------------------------------------------------------------- history
def test_history_append_read(tmp_path):
    hist = tmp_path / "history.jsonl"
    assert read_history(hist) == []  # missing file tolerated
    append_history({"cal_file": "a.json", "score": 1.0}, hist)
    append_history({"cal_file": "b.json", "score": 2.0}, hist)
    recs = read_history(hist)
    assert [r["score"] for r in recs] == [1.0, 2.0]


def test_pick_best(tmp_path):
    a, b = tmp_path / "a.json", tmp_path / "b.json"
    a.write_text("{}")
    b.write_text("{}")
    recs = [
        {"cal_file": str(a), "score": 5.0},
        {"cal_file": str(b), "score": 2.0},
        {"cal_file": str(tmp_path / "gone.json"), "score": 0.1},  # dropped
        {"cal_file": str(a), "score": None},                        # skipped
    ]
    best = pick_best(recs)
    assert best["cal_file"] == str(b)


def test_pick_best_empty():
    assert pick_best([]) is None
    assert pick_best([{"cal_file": "nope.json", "score": 1.0}]) is None


# ------------------------------------------------------- rank_and_compare
def test_rank_and_compare_winner_beats_factory(tmp_path):
    """Only a winner strictly better than factory is promoted."""
    from bno055_kit.calibration import rank_and_compare

    a, b = tmp_path / "a.json", tmp_path / "b.json"
    a.write_text("{}")
    b.write_text("{}")
    recs = [{"cal_file": str(a), "score": 41.0}, {"cal_file": str(b), "score": 12.5}]
    winner, promote = rank_and_compare(recs, factory_score=20.0)
    assert winner["cal_file"] == str(b)
    assert promote is True


def test_rank_and_compare_factory_still_best(tmp_path):
    from bno055_kit.calibration import rank_and_compare

    a = tmp_path / "a.json"
    a.write_text("{}")
    recs = [{"cal_file": str(a), "score": 99.3858}]
    winner, promote = rank_and_compare(recs, factory_score=20.0)
    assert winner["cal_file"] == str(a)
    assert promote is False  # worse than factory -> active.json untouched


def test_rank_and_compare_tie_keeps_factory(tmp_path):
    from bno055_kit.calibration import rank_and_compare

    a = tmp_path / "a.json"
    a.write_text("{}")
    recs = [{"cal_file": str(a), "score": 20.0}]
    _, promote = rank_and_compare(recs, factory_score=20.0)
    assert promote is False


def test_rank_and_compare_no_baseline_promotes(tmp_path):
    """No usable factory score (failed bench) -> winner wins by default."""
    from bno055_kit.calibration import rank_and_compare

    a = tmp_path / "a.json"
    a.write_text("{}")
    recs = [{"cal_file": str(a), "score": 99.0}]
    winner, promote = rank_and_compare(recs, factory_score=None)
    assert winner["cal_file"] == str(a)
    assert promote is True


def test_rank_and_compare_no_session():
    from bno055_kit.calibration import rank_and_compare

    assert rank_and_compare([], factory_score=20.0) == (None, False)


# ---------------------------------------------------------------- fallback
def test_fallback_path_sits_next_to_active():
    from bno055_kit.calibration import FALLBACK_NAME, fallback_path

    assert fallback_path("/var/lib/bno055/active.json") == \
        Path("/var/lib/bno055") / FALLBACK_NAME
    assert FALLBACK_NAME == "factory_fallback.json"


def test_fallback_copy_restores_active(tmp_path, cal_dict):
    """The fallback mechanism: frozen fallback -> overwrite active.json."""
    from bno055_kit.calibration import fallback_path

    fb = fallback_path(tmp_path / "active.json")
    cal_dict["source"] = "factory"
    sha_fb = save_calibration(Calibration.from_dict(cal_dict), fb)
    # active drifts away (bad on-Kit sessions) ...
    drifted = dict(cal_dict, accel_offset=[1, 2, 3], source="on-kit")
    save_calibration(Calibration.from_dict(drifted), tmp_path / "active.json")
    assert load_calibration(tmp_path / "active.json").accel_offset == (1, 2, 3)
    # ... then fallback restores the frozen values into active.
    save_calibration(load_calibration(fb), tmp_path / "active.json")
    active = load_calibration(tmp_path / "active.json")
    assert list(active.accel_offset) == cal_dict["accel_offset"]
    assert active.sha256 == sha_fb
