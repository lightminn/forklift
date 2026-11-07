"""Record-only diagnostics of the unconfirmed position check (the lead's decision after the second B, item 3)."""

import random
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

import pose_diag as D  # noqa: E402

from forklift_core.control.drive_permission import Check  # noqa: E402

KAPPAS = (-0.05, 0.0, 0.05)


def _old(kappas, length, zones, zone_contact, obstacle_check, calls):
    """pose_rear_blocked's loop as it was before the record (run_transport, 2026-11-07)."""
    for kappa in kappas:
        if zones and zone_contact(kappa)[0] < length:
            return True
        if obstacle_check(kappa).verified_m < length:
            return True
    return False


def _fakes(table, calls):
    def zone(k):
        calls.append(("zone", k))
        return table[k][0]

    def obstacle(k):
        calls.append(("obstacle", k))
        return table[k][1]
    return zone, obstacle


def test_a_zone_short_on_the_middle_arc_blocks_there_and_the_rest_is_unchecked():
    length = 0.5683
    table = {-0.05: ((float("inf"), None), Check(1.0, None, 0.0, True)),
             0.0: ((0.2, "P-1@h3"), Check(1.0, None, 0.0, True)),
             0.05: ((float("inf"), None), Check(1.0, None, 0.0, True))}
    calls = []
    blocked, arcs = D.walk_rear(KAPPAS, length, ["z"], *_fakes(table, calls))
    assert blocked and calls == [("zone", -0.05), ("obstacle", -0.05), ("zone", 0.0)]
    assert arcs[0]["zone"] == {"contact_m": "inf", "zone_id": None} and arcs[0]["obstacle"]["verified_m"] == 1.0
    assert arcs[1]["zone"] == {"contact_m": 0.2, "zone_id": "P-1@h3"} and arcs[1]["obstacle"] == D.UNCHECKED
    assert arcs[2] == {"kappa": 0.05, "zone": D.UNCHECKED, "obstacle": D.UNCHECKED}
    assert D.blocking_branch(arcs, length) == ("zone", 0.0)


def test_an_obstacle_short_records_its_kind_and_cells_and_no_zones_skips_the_zone_check():
    length = 0.5683
    table = {k: ((0.0, "never"), Check(0.3 if k == -0.05 else 1.0, "unknown" if k == -0.05 else None, 0.0, False,
                                      blocked_cells=((3, 4), (5, 6)) if k == -0.05 else ())) for k in KAPPAS}
    calls = []
    blocked, arcs = D.walk_rear(KAPPAS, length, [], *_fakes(table, calls))
    assert blocked and calls == [("obstacle", -0.05)]
    assert arcs[0]["zone"] == "no_zones" and arcs[0]["obstacle"] == {"verified_m": 0.3, "blocked": "unknown",
                                                                       "cells": [[3, 4], [5, 6]]}
    assert D.blocking_branch(arcs, length) == ("obstacle", -0.05)


def test_the_walk_decides_what_the_old_loop_decided_in_the_same_order():
    rng = random.Random(20261107)
    for _ in range(2000):
        length = rng.choice((0.5, 0.5683))
        zones = rng.choice(([], ["z"]))
        table = {k: ((rng.choice((0.0, 0.3, 0.5683, 2.0, float("inf"))), "Z@h1"),
                     Check(rng.choice((0.0, 0.4, 0.5683, 3.0)), None, 0.0, True)) for k in KAPPAS}
        old_calls, new_calls = [], []
        old = _old(KAPPAS, length, zones, *_fakes(table, old_calls), old_calls)
        new, arcs = D.walk_rear(KAPPAS, length, zones, *_fakes(table, new_calls))
        assert new == old and new_calls == old_calls
        assert (D.blocking_branch(arcs, length)[0] is not None) == new


def test_the_layers_go_to_plain_arrays():
    class Occ:
        occupied, origin_x_m, origin_y_m, resolution_m = np.ones((2, 3), bool), 1.0, 2.0, 0.05
    out = D.layers_arrays({"occupancy": Occ(), "live": np.zeros((2, 3), bool), "use_slam": True,
                           "snapshot_stamp_s": 4.5, "correction_version": 7}, prefix="pre_")
    assert out["pre_occupied"].all() and out["pre_origin"].tolist() == [1.0, 2.0]
    assert not out["pre_live"].any() and out["pre_correction_version"] == 7 and "pre_memory_observed" not in out
    assert out["pre_resolution_m"] == pytest.approx(0.05)


class _Occ:
    def __init__(self, cells):
        self.occupied, self.origin_x_m, self.origin_y_m, self.resolution_m = cells, 1.0, 2.0, 0.05


def test_the_pre_move_state_and_the_failed_preparations_inputs_are_written(tmp_path):
    import json
    import pickle

    layers = {"occupancy": _Occ(np.eye(3, dtype=bool)), "live": np.eye(3, dtype=bool), "use_slam": True,
              "snapshot_stamp_s": 2.6, "correction_version": 4}
    D.save_premove(tmp_path / "pre.npz", {"t": 2.6, "C": [0.1, 0.0, 0.0], "O": [1.0, 2.0, 0.3], "layers": layers})
    pre = np.load(tmp_path / "pre.npz")
    assert json.loads(str(pre["meta"]))["O"] == [1.0, 2.0, 0.3] and pre["occupied"].sum() == 3
    payload = {"world": "w", "waypoints": [(1.0, 2.0, 0.0)] * 16}
    caps = [{"t": 40.0, "mission_id": "m1", "payload": pickle.dumps(payload), "inputs": None, "layers": layers,
             "layers_match": True},
            {"t": 90.0, "mission_id": "m2", "payload": pickle.dumps({}), "inputs": None, "layers": layers}]
    results = [{"mission_id": "m1", "reason": "prepare:all_candidates_failed:invalid_start"},
               {"mission_id": "m2", "reason": None, "outcome": "done"}]
    entries = [D.save_capture(tmp_path, k, c) for k, c in enumerate(caps, 1)]
    assert (tmp_path / "prepare_input_2.pkl").exists()  # written at the submit, whatever comes later
    assert D.prune_captures(tmp_path, entries, None) == entries  # results unread: nothing removed
    kept = D.prune_captures(tmp_path, entries, results)
    assert [e["file"] for e in kept] == ["prepare_input_1"] and kept[0]["layers_match"] is True
    assert not (tmp_path / "prepare_input_2.pkl").exists() and not (tmp_path / "prepare_input_2.npz").exists()
    with open(tmp_path / "prepare_input_1.pkl", "rb") as fh:
        got = pickle.load(fh)
    assert pickle.loads(got["payload"]) == payload and got["meta"]["mission_id"] == "m1"
    assert np.load(tmp_path / "prepare_input_1.npz")["live"].sum() == 3
    # a failure whose result has rotated out of the book stays on disk
    assert [e["file"] for e in D.prune_captures(tmp_path, entries, [])] == ["prepare_input_1"]


def test_a_run_without_a_grid_plan_writes_its_record_without_arrays(tmp_path):
    import json

    D.save_premove(tmp_path / "pre.npz", {"t": 1.0, "layers": {}})
    pre = np.load(tmp_path / "pre.npz")
    assert json.loads(str(pre["meta"]))["grid"] == "no_grid_plan" and list(pre.files) == ["meta"]
