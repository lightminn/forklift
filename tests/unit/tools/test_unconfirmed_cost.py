"""The unconfirmed position plan's offline cost tool (docs/plans/2026-11-02-unconfirmed-position.md 1.3): one row per
believed start -- the start check of plan 1.2, the hypothesis set's completeness against the capture's truth, the rear
corridor -- and a summary per world."""

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "localization"))

import test_global_match as TG  # noqa: E402
from test_global_match_eval import _capture  # noqa: E402

from tools import unconfirmed_cost as U  # noqa: E402

CFG = json.dumps({"yaw_step_rad": math.radians(2.0)})


def _bundle(tmp_path, grid):
    b = tmp_path / "bundle"
    b.mkdir()
    np.savez(b / "grid.npz", data=grid.data, origin=np.array(grid.origin), resolution_m=grid.resolution_m)
    (b / "manifest.json").write_text(json.dumps({"map_id": "m" * 64, "map_from_world": [0, 0, 0]}))
    return b


def test_a_start_in_an_asymmetric_room_is_confirmed_and_its_truth_is_in_the_set(tmp_path):
    grid = TG._grid(*TG.ROOM)
    truth = (2.42, 1.7, 0.6)
    cap = _capture(tmp_path / "c.npz", [truth, (4.8, 1.2, 0.0)], 1, grid)
    believed = tmp_path / "b.jsonl"
    believed.write_text(json.dumps({"id": "p0", "base": list(truth)}) + "\n" +
                        json.dumps({"id": "nope", "base": [1, 1, 0]}) + "\n")
    rows = tmp_path / "rows.jsonl"
    assert U.main(["run", "--bundle", str(_bundle(tmp_path, grid)), "--capture", str(cap), "--believed",
                   str(believed), "--config", CFG, "--out", str(rows)]) == 0
    out = [json.loads(line) for line in rows.read_text().splitlines()]
    assert out[1]["status"] == "not_captured"
    r = out[0]
    assert r["status"] == "ok" and r["confirmed"] and r["truth_in_set"] and r["believed_is_truth"]
    assert r["n_hyps"] >= 25 and "shift" in r["sources"] and r["impossible"] > 0
    summary = tmp_path / "s.json"
    assert U.main(["summary", "--rows", str(rows), "--out", str(summary)]) == 0
    s = json.loads(summary.read_text())["rows.jsonl"]
    assert s["checked"] == 1 and s["unconfirmed"] == 0 and s["skipped"]["not_captured"] == 1


def test_the_rear_corridor_sees_occupied_cells_only():
    grid = TG._grid(*TG.ROOM)  # walls one cell thick around a 6 m x 4 m room
    # base 1.2 m from the left wall, facing +x: the rear end is 0.34 + 0.17 = 0.51 m behind base_link
    assert U.rear_clear(grid, (1.2 + 0.1, 2.0, 0.0), rear_m=0.5)  # 0.79 m to the wall cell's face, corridor 0.5 m
    assert not U.rear_clear(grid, (1.0, 2.0, 0.0), rear_m=0.62)  # the corridor reaches the wall


# --- Codex piece 1 review ---
def _one(tmp_path, row, monkeypatch=None, search=None):
    grid = TG._grid(*TG.ROOM)
    truth = (2.42, 1.7, 0.6)
    cap = _capture(tmp_path / "c.npz", [truth], 1, grid)
    believed = tmp_path / "b.jsonl"
    believed.write_text(json.dumps({"id": "p0", "base": list(truth), **row}) + "\n")
    if search is not None:
        monkeypatch.setattr(U, "pool_and_leaves", lambda *a, **k: search)
    rows = tmp_path / "rows.jsonl"
    assert U.main(["run", "--bundle", str(_bundle(tmp_path, grid)), "--capture", str(cap), "--believed",
                   str(believed), "--config", CFG, "--out", str(rows)]) == 0
    return json.loads(rows.read_text().splitlines()[0])


def test_the_generations_own_pool_is_used_when_the_row_carries_it(tmp_path):
    r = _one(tmp_path, {"pool": [[4.8, 1.2, 0.0], [2.42, 1.7, 0.6]]})
    assert r["pool_source"] == "generation" and r["n_pool"] == 2
    assert "pool" in r["sources"] and r["n_proxy_pool"] >= 1


def test_a_hypothesis_search_that_failed_leaves_the_start_unconfirmed(tmp_path, monkeypatch):
    r = _one(tmp_path, {}, monkeypatch, search=("overflow", [], []))
    assert not r["confirmed"] and "hypotheses:overflow" in r["reasons"]
    s = U.summarize([r])
    assert s["unconfirmed"] == 1 and s["reason_kinds"] == {"hypotheses": 1}
    assert s["reasons"] == {"hypotheses:overflow": 1}
