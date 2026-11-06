"""The D batch's two monitor candidates read from the records (tools/pose_check_stands.py, plan
docs/plans/2026-11-06-unconfirmed-validation.md section 6 step 4): the correction's change over a sliding window of path,
and the offline re-match of the mission stands' scan sets (a snapshot approximation, plan 1.5)."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "sim/isaac"))
sys.path.insert(0, str(ROOT / "tests/unit/localization"))

import pose_check_stands as S  # noqa: E402
import test_global_match as TG  # noqa: E402

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import start_reloc as R  # noqa: E402
from forklift_core.localization.slam_pose import compose, invert  # noqa: E402

CFG = G.MatchConfig(yaw_step_rad=math.radians(2.0))
RELOC = R.RelocConfig(e_min=0.8, r_max=0.9, r_struct=0.8)
ANGLES = (0.0, 2 * math.pi / 360)


def _samples(n=300, step=0.1, c_per_m=0.01, jump_at=None, jump=0.0):
    """A straight drive; the correction moves c_per_m per metre sideways, and steps by ``jump`` at sample ``jump_at``."""
    out = []
    for k in range(n):
        d = k * step
        side = c_per_m * d + (jump if jump_at is not None and k >= jump_at else 0.0)
        out.append({"t": 0.1 * k, "o": [d, 0.0, 0.0], "applied": [d, side, 0.0], "truth": [d, 0.0, 0.0]})
    return out


def test_the_window_change_follows_the_correction_over_that_path():
    rows = S.window_changes(_samples(), window_m=5.0)
    assert max(r["m"] for r in rows) == pytest.approx(0.05, abs=0.002)  # 1 % over 5 m
    assert S.window_changes(_samples(n=20), window_m=5.0) == []  # never that much path: nothing to judge


def test_a_jump_shows_in_the_window_whatever_the_path_before_it():
    rows = S.window_changes(_samples(jump_at=200, jump=6.0), window_m=5.0)
    assert max(r["m"] for r in rows) > 6.0


def test_a_standing_correction_step_counts_in_the_window():
    samples = _samples(n=100) + [{"t": 10 + 0.1 * k, "o": [9.9, 0.0, 0.0], "applied": [9.9, 0.3 + 0.099, 0.0],
                                  "truth": [9.9, 0.0, 0.0]} for k in range(50)]
    assert max(r["m"] for r in S.window_changes(samples, window_m=5.0)) > 0.3


def _record(o, C=(0.0, 0.0, 0.0), truth=None):
    sample = {"t": 5.0, "o": list(o), "applied": list(compose(C, o)), "truth": list(truth or o)}
    return {"mode": "shadow", "start": {"o_scan": list(o)}, "hyps": [], "samples": [sample],
            "stand_sets": [{"t0": 4.0, "t1": 6.0, "delta": [0.15, 0.01], "ref": "int",
                            "sets": [{"stamps": [4.6, 4.7, 4.8, 4.9, 5.0], "o": list(o)}]}]}


def _rematch_ctx(grid):
    import pose_runtime as PR

    return PR.JobContext(grid, CFG, RELOC, PR.SHADOW_BOUNDS)


def _scans(grid, base, tmp_path, name="s.npz"):
    ranges = np.asarray(TG._scan(grid, R.laser_from_base(base), beams=360)[0], float)
    path = tmp_path / name
    np.savez_compressed(path, angles=np.asarray(ANGLES), ranges_m=np.asarray([0.2, 12.0, 8.0]),
                        s0_0=np.stack([ranges] * 5).astype(np.float32))
    return path


def test_a_stand_the_correction_agrees_with_would_renew_the_reference(tmp_path):
    grid = TG._grid(*TG.ROOM)
    o = (2.42, 1.7, 0.6)
    out = S.rematch_stands(_record(o), np.load(_scans(grid, o, tmp_path)), _rematch_ctx(grid), b=o, o0=o,
                           e_slam=(0.15, 0.05))
    [row] = out
    assert row["outcome"] == "renew" and row["truth_class"] == "success" and row["sets_used"] == 1


def test_a_stand_the_correction_disagrees_with_is_evidence_of_a_revocation(tmp_path):
    grid = TG._grid(*TG.ROOM)
    o = (2.42, 1.7, 0.6)
    out = S.rematch_stands(_record(o, C=(0.5, 0.0, 0.0)), np.load(_scans(grid, o, tmp_path)), _rematch_ctx(grid), b=o,
                           o0=o, e_slam=(0.15, 0.05))
    assert out[0]["outcome"] == "revoke" and out[0]["agree_m"] == pytest.approx(0.5, abs=0.06)


def test_a_stand_with_no_scan_set_is_not_estimable_not_no_evidence(tmp_path):
    grid = TG._grid(*TG.ROOM)
    o = (2.42, 1.7, 0.6)
    rec = _record(o)
    rec["stand_sets"][0]["sets"] = []
    out = S.rematch_stands(rec, np.load(_scans(grid, o, tmp_path)), _rematch_ctx(grid), b=o, o0=o, e_slam=(0.15, 0.05))
    assert out[0]["outcome"] == "not_estimable"


def test_the_summary_counts_outcomes_and_the_path_between_renewals():
    rows = [{"t": 10.0, "outcome": "renew"}, {"t": 20.0, "outcome": "no_evidence"}, {"t": 30.0, "outcome": "renew"}]
    out = S.summary({"a": rows})
    assert out["outcomes"] == {"renew": 2, "no_evidence": 1} and out["stands"] == 3
    assert json.dumps(out)
