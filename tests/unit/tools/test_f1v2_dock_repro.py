"""F1 v2 stage 0: dock() on a recorded match's own inputs is compared with the recorded round 1, and any difference in a
start or a detail is caught."""

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f1v2_dock_repro as R  # noqa: E402

from forklift_core.localization.scan_docking import dock  # noqa: E402


def _scene():
    """An L-shaped wall pair seen from a little in front of the reference station."""
    xs = np.linspace(0.5, 4.0, 120)
    wall = np.column_stack((xs, np.full_like(xs, 1.5)))
    back = np.column_stack((np.full(80, 4.0), np.linspace(-1.5, 1.5, 80)))
    side = np.column_stack((xs, np.full_like(xs, -1.5)))
    reference = np.vstack((wall, back, side))
    live = reference - np.array([0.3, 0.02])
    return reference, live


def _write(tmp_path: Path, tamper=None):
    reference, live = _scene()
    kw = dict(rear_from_laser=(0.4, 0.0, 0.0), estimate_rear=(-0.3, 0.0, 0.0), delivery_rear=(0.0, 0.0, 0.0))
    got = dock(reference, live, **kw)
    rnd = {"accepted": got.accepted, "reason": got.reason,
           "starts": [[a, r, list(q)] for a, r, q in got.starts], "start_details": [dict(d) for d in got.start_details]}
    if tamper:
        tamper(rnd)
    np.savez(tmp_path / "m.npz", reference=reference, live=live, rear_from_laser=np.array(kw["rear_from_laser"]),
             estimate=np.array(kw["estimate_rear"]), seed=np.array(kw["delivery_rear"]))
    (tmp_path / "r.json").write_text(json.dumps({"rounds": [rnd], "docking_acceptance": "information"}))
    return tmp_path / "m.npz", tmp_path / "r.json"


def test_an_untouched_record_is_bit_identical(tmp_path):
    m, r = _write(tmp_path)
    assert R.compare(m, r)["bit_identical"] is True


def test_a_changed_information_or_start_is_caught(tmp_path):
    def bump(rnd):
        rnd["start_details"][0]["min_information"] += 1e-12

    m, r = _write(tmp_path, bump)
    out = R.compare(m, r)
    assert out["bit_identical"] is False and out["details_equal"] is False

    def flip(rnd):
        rnd["starts"][1][1] = "tampered"

    (tmp_path / "b").mkdir()
    m, r = _write(tmp_path / "b", flip)
    assert R.compare(m, r)["starts_equal"] is False
