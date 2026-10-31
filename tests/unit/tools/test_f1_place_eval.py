"""F1's offline evaluator (tools/f1_place_eval.py, docs/plans/2026-10-31-operator-placed-start.md 2-4): the click
model, one session per valid start under the registered policy (at most three placements, each on its own scan set,
the first preview confirmed), the R_AMB sweep from one run, and the registered choice rule."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from tools import f1_place_eval as F

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "localization"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_global_match as TG  # noqa: E402
from test_global_match_eval import _capture  # noqa: E402

TRUTH = (2.0, 1.0, 0.5)


def test_the_click_model_is_reproducible_bounded_and_uniform_over_the_disc():
    clicks = [F.click(7, i, k, TRUTH) for i in range(400) for k in range(3)]
    assert F.click(7, 3, 1, TRUTH) == F.click(7, 3, 1, TRUTH) != F.click(8, 3, 1, TRUTH)
    r = np.array([math.hypot(c[0] - TRUTH[0], c[1] - TRUTH[1]) for c in clicks])
    dyaw = np.array([abs(math.atan2(math.sin(c[2] - TRUTH[2]), math.cos(c[2] - TRUTH[2]))) for c in clicks])
    assert r.max() <= F.CLICK_M + 1e-12 and dyaw.max() <= F.CLICK_RAD + 1e-12
    assert np.median(r) == pytest.approx(F.CLICK_M / math.sqrt(2), abs=0.05)  # area-uniform: half within r/sqrt 2


def _rec(reason, amb=None, base=TRUTH):
    return {"reason": reason, "amb_ratio": amb, "base": list(base) if base is not None else None}


def test_a_placement_is_rescored_for_any_r_amb_from_its_ambiguity_ratio():
    assert F.placement_decision(_rec("confirmed", 0.85), 0.90) == (True, "confirmed")
    assert F.placement_decision(_rec("confirmed", 0.85), 0.85) == (False, "ambiguous")
    assert F.placement_decision(_rec("edge", 0.95), 0.90) == (False, "ambiguous")  # ambiguity before the edge
    assert F.placement_decision(_rec("edge", 0.5), 0.90) == (False, "edge")
    assert F.placement_decision(_rec("low_fit", None), 0.80) == (False, "low_fit")


def test_a_session_ends_at_its_first_confirmation_and_classifies_it():
    far = (TRUTH[0] + 0.5, TRUTH[1], TRUTH[2])
    s = F.session([_rec("low_fit"), _rec("confirmed", 0.5), _rec("confirmed", 0.5, far)], 0.9, TRUTH)
    assert s["outcome"] == "success" and s["placements"] == 2
    s = F.session([_rec("confirmed", 0.5, far)], 0.9, TRUTH)
    assert s["outcome"] == "wrong"
    s = F.session([_rec("confirmed", 0.5, (TRUTH[0] + 0.02, TRUTH[1], TRUTH[2] + 0.08))], 0.9, TRUTH)
    assert s["outcome"] == "imprecise"  # neither success nor wrong (Codex F1 v1 review P2 4)
    s = F.session([_rec("ambiguous", 1.0), _rec("unsteady"), _rec("edge", 0.2)], 0.9, TRUTH)
    assert s["outcome"] == "failed" and s["reasons"] == ["ambiguous", "unsteady"]  # unsteady ends it


def test_the_choice_is_the_clean_value_with_the_most_rescues_and_the_stricter_on_a_tie():
    table = [{"r_amb": 0.80, "wrong": 0, "imprecise": 0, "rescued": 5},
             {"r_amb": 0.85, "wrong": 0, "imprecise": 0, "rescued": 7},
             {"r_amb": 0.90, "wrong": 0, "imprecise": 0, "rescued": 7},
             {"r_amb": 0.95, "wrong": 1, "imprecise": 0, "rescued": 9}]
    assert F.choose(table)["r_amb"] == 0.85
    assert F.choose([{"r_amb": 0.9, "wrong": 0, "imprecise": 1, "rescued": 9}]) is None


def _bundle(tmp_path, grid):
    b = tmp_path / "bundle"
    b.mkdir()
    np.savez(b / "grid.npz", data=grid.data, origin=np.array(grid.origin), resolution_m=grid.resolution_m)
    (b / "manifest.json").write_text(json.dumps({"map_id": "m" * 64, "map_from_world": [0, 0, 0]}))
    return b


def test_a_run_simulates_one_session_per_valid_start(tmp_path):
    grid = TG._grid(*TG.ROOM)
    caps = [_capture(tmp_path / f"c{k}.npz", [(2.42, 1.7, 0.6), (4.8, 1.2, 0.0)], 1, grid) for k in range(3)]
    with np.load(caps[0]) as z:  # the second start failed its input check in the first set: out of the sessions
        d = dict(z)
    d["valid"] = np.array([True, False])
    np.savez(caps[0], **d)
    with np.load(caps[1]) as z:  # the first start moved before its second placement: that one is unsteady
        d = dict(z)
    d["valid"] = np.array([False, True])
    np.savez(caps[1], **d)
    rows = tmp_path / "rows.jsonl"
    assert F.main(["run", "--bundle", str(_bundle(tmp_path, grid)), "--capture", str(caps[0]), "--capture",
                   str(caps[1]), "--capture", str(caps[2]), "--seed", "3", "--out", str(rows),
                   "--config", json.dumps({"yaw_step_rad": math.radians(2.0)})]) == 0
    out = [json.loads(line) for line in rows.read_text().splitlines()]
    assert [r["valid"] for r in out] == [True, False] and out[1]["placements"] == []
    p = out[0]["placements"]
    assert len(p) == 3 and p[1]["reason"] == "unsteady" and all("wall_s" in x for x in p if x["reason"] != "unsteady")
    summary = tmp_path / "sweep.json"
    assert F.main(["sweep", "--rows", str(rows), "--out", str(summary)]) == 0
    s = json.loads(summary.read_text())
    assert s["valid"] == 1 and len(s["table"]) == len(F.R_AMB_GRID) and "time" in s


def test_an_unsteady_placement_ends_the_session_as_live_does():
    # Codex F1 implementation review P2 3: the live phase ends the session on unsteady
    s = F.session([_rec("edge", 0.2), _rec("unsteady"), _rec("confirmed", 0.5)], 0.9, TRUTH)
    assert s["outcome"] == "failed" and s["placements"] == 2 and s["reasons"] == ["edge", "unsteady"]


def test_a_click_outside_the_map_is_one_failed_placement_offline(tmp_path):
    grid = TG._grid(*TG.ROOM)
    caps = [_capture(tmp_path / f"c{k}.npz", [(0.3, 2.0, math.pi / 2)], 1, grid) for k in range(3)]
    rows = tmp_path / "rows.jsonl"
    F.main(["run", "--bundle", str(_bundle(tmp_path, grid)), "--capture", str(caps[0]), "--capture", str(caps[1]),
            "--capture", str(caps[2]), "--seed", "3", "--out", str(rows),
            "--config", json.dumps({"yaw_step_rad": math.radians(2.0)})])
    [row] = [json.loads(line) for line in rows.read_text().splitlines()]
    outside = [p for p in row["placements"] if not (0.0 <= p["click"][0] < 6.0 and 0.0 <= p["click"][1] < 4.0)]
    assert all(p["reason"] == "invalid" for p in outside)
