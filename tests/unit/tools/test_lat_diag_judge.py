"""The layer budget diagnosis' perturbation batch (docs/plans/2026-11-13-layer-budget-diagnosis.md 3): the registered
50-run list, Pn's exposure, the triplet endpoints (mission failure only where B0 completed, operating exceedance only
where all three completed), the one-sided sign test and the results sentences fixed before the results. Synthetic."""

import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
D = ROOT / "docs/validation/2026-11-11-latency"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, D / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


J = _load("lat_diag_judge")
L = _load("lat_lists")


def test_the_list_is_the_registered_50_runs(tmp_path):
    rows = L.diag_rows("/s", "/o")
    assert len(rows) == 50
    by = {r["name"]: r["env"] for r in rows}
    seeds = {sc: {int(e["SEED"]) for n, e in by.items() if e["SCENE"] == sc and not n.endswith("_can")}
             for sc in ("random_1", "aisle_rack")}
    assert seeds == {"random_1": set(range(3, 11)), "aisle_rack": set(range(11, 19))}  # disjoint, new
    assert by["lbd_Pn_random_1_s3"]["EXTRA_ARGS"] == "--deterministic --lidar-cast loop --seed 1003"
    assert by["lbd_Pn_random_1_s3"]["LAYER_BUDGET"] == "off" and by["lbd_Pn_random_1_s3"]["SEED"] == "3"
    assert by["lbd_B0_aisle_rack_s11"]["LAYER_BUDGET"] == "off"
    assert by["lbd_B0_aisle_rack_s11"]["EXTRA_ARGS"] == "--deterministic --lidar-cast loop"
    assert by["lbd_Bb10_aisle_rack_s11"]["LAYER_BUDGET"] == "config/layer_latency_budget.yaml"
    assert sorted(n for n in by if n.endswith("_can")) == ["lbd_B0_aisle_rack_s2_can", "lbd_B0_random_1_s0_can"]
    sys_path = str(ROOT / "tools")
    import sys

    sys.path.insert(0, sys_path)
    import isaac_bundle as B

    text = "".join(json.dumps(r) + "\n" for r in rows)
    assert len(B.load_runs(tmp_path / "x.jsonl", text=text)) == 50


def test_the_sign_test_values():
    assert J.sign_test(4, 0) == pytest.approx(0.0625)
    assert J.sign_test(5, 1) == pytest.approx(7 / 64)
    assert J.sign_test(0, 0) is None
    assert J.sign_test(0, 3) == 1.0


def test_pn_exposure_is_decided_by_its_arguments():
    ok = J.pn_exposure({"arguments": {"seed": 1003, "slam_noise_seed": 3}}, 3, "h0", "h1")
    assert ok["exposed"] and ok["control_equal_to_b0"] is False
    same = J.pn_exposure({"arguments": {"seed": 1003, "slam_noise_seed": 3}}, 3, "h0", "h0")
    assert same["exposed"] and same["control_equal_to_b0"] is True  # kept in the test, recorded
    assert not J.pn_exposure({"arguments": {"seed": 3, "slam_noise_seed": 3}}, 3, "h0", "h1")["exposed"]
    assert not J.pn_exposure(None, 3, None, None)["exposed"]


def mission(done=True, sim=100.0, deliv=1.0, rounds=1):
    return {"done": done, "mission_sim_s": sim if done else None, "delivery_error_cm": deliv if done else None,
            "docking_rounds": rounds if done else None}


def test_endpoints_count_only_where_defined():
    b0, failed = {"mission": mission()}, {"mission": mission(done=False)}
    assert J.worsened(failed, b0) == {"ga": True, "na": None}  # a failure is never an operating normal
    assert J.worsened({"mission": mission(deliv=2.5)}, b0) == {"ga": False, "na": True}
    assert J.worsened({"mission": mission(deliv=1.9)}, b0) == {"ga": False, "na": False}
    assert J.worsened({"mission": mission()}, failed) == {"ga": None, "na": None}  # B0 failed: neither


def row(arm, done=True, deliv=1.0, exposed=True):
    r = {"arm": arm, "invalid": [], "mission": mission(done=done, deliv=deliv)}
    if arm == "Pn":
        r["exposure"] = {"exposed": exposed}
    return r


def scene_rows(scene, spec):
    """spec: seed -> (B0 (done, deliv), Pn (done, deliv), Bb10 (done, deliv))"""
    out = {}
    for s, (b0, pn, bb) in spec.items():
        out[f"B0:{scene}:{s}"] = row("B0", *b0)
        out[f"Pn:{scene}:{s}"] = row("Pn", *pn)
        out[f"Bb10:{scene}:{s}"] = row("Bb10", *bb)
    return out


def test_the_verdicts_and_their_fixed_sentences():
    ok, fail, far = (True, 1.0), (False, 1.0), (True, 3.0)
    rows = {**scene_rows("random_1", {3: (ok, ok, fail), 4: (ok, ok, fail), 5: (ok, ok, fail), 6: (ok, ok, far)}),
            **scene_rows("aisle_rack", {11: (ok, ok, fail), 12: (fail, ok, ok), 13: (ok, far, far)})}
    c = J.conclusions(rows, {"random_1": [3, 4, 5, 6], "aisle_rack": [11, 12, 13]})
    ga = c["pooled"]["ga"]
    assert (ga["b"], ga["c"], ga["n"]) == (4, 0, 6) and ga["verdict"] == "confirmed"  # seed 12: B0 failed, out
    na = c["pooled"]["na"]  # only seeds 6 and 13 have all three completed
    assert (na["n"], na["b"], na["c"], na["both"]) == (2, 1, 0, 1) and na["verdict"] == "not_confirmed"
    assert "confirmed for this endpoint" in c["sentences"]["mission failure"]
    assert "not evidence that the budget costs nothing" in c["sentences"]["operating exceedance"]
    assert c["per_scene"]["random_1"]["ga"]["b"] == 3 and c["per_scene"]["aisle_rack"]["ga"]["b"] == 1


def test_unexposed_pn_leaves_the_seed_out_and_two_leave_the_scene_unjudged():
    ok, fail = (True, 1.0), (False, 1.0)
    rows = scene_rows("random_1", {3: (ok, ok, fail), 4: (ok, ok, fail), 5: (ok, ok, fail)})
    rows["Pn:random_1:3"]["exposure"]["exposed"] = False
    c = J.conclusions(rows, {"random_1": [3, 4, 5]})
    assert c["per_scene"]["random_1"]["judged"] and c["per_scene"]["random_1"]["triplets"] == 2
    rows["Pn:random_1:4"]["exposure"]["exposed"] = False
    c = J.conclusions(rows, {"random_1": [3, 4, 5]})
    assert not c["per_scene"]["random_1"]["judged"] and c["pooled"]["ga"]["n"] == 0
