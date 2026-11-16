"""The short cusp arc comparison tools (docs/validation/2026-11-16-aisle-short-cusp-arc/)."""

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
D = ROOT / "docs/validation/2026-11-16-aisle-short-cusp-arc"


def load(name):
    spec = importlib.util.spec_from_file_location(name, D / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


C = load("compare")
R = load("make_runs")


def path(*legs):
    """legs: (direction, curvature, n steps of 0.25 m along x) -> a recorded path dict."""
    poses, dirs, curv, x = [[0.0, 0.0, 0.0]], [legs[0][0]], [legs[0][1]], 0.0
    for d, k, n in legs:
        for _ in range(n):
            x += 0.25 * d
            poses.append([x, 0.0, 0.0])
            dirs.append(d)
            curv.append(k)
    return {"success": True, "poses": poses, "directions": dirs, "curvatures_inv_m": curv, "length_m": 1.0}


def test_short_curved_cusp_legs_are_counted_by_the_registered_rule():
    assert C.short_curved_cusps(path((1, 0.0, 4), (-1, 0.29, 1), (1, 0.0, 4))) == 1  # 0.25 m curved reverse
    assert C.short_curved_cusps(path((1, 0.0, 4), (-1, 0.0, 1), (1, 0.0, 4))) == 0  # straight
    assert C.short_curved_cusps(path((1, 0.0, 4), (-1, 0.29, 3), (1, 0.0, 4))) == 0  # 0.75 m
    assert C.short_curved_cusps(path((1, 0.29, 2), (-1, 0.29, 2))) == 1  # 0.5 m curved, then the goal (no cusp)
    assert C.short_curved_cusps({"success": False}) == 0


def test_run_measures_and_the_verdict():
    res = {"success": True, "planning_wall_s": 10.0, "paths": {"transport": path((1, 0.0, 4), (-1, 0.29, 1), (1, 0.0, 2))},
           "cusp_replans": [{"planning_wall_s": 5.0, "path": path((1, 0.29, 1), (-1, 0.0, 2))}], "scene_replans": []}
    m = C.run_measures(res)
    assert m["planning_wall_sum_s"] == 15.0 and m["planning_wall_max_s"] == 10.0
    assert m["short_curved_cusps_transport"] == 2 and m["short_curved_cusps_all"] == 2
    a = {s: {"fail_replan": 1, "fail_final": 0, "success": True, "judgements": 8, "planning_wall_sum_s": 10.0,
             "planning_wall_max_s": 10.0, "short_curved_cusps_all": 1, "short_curved_cusps_transport": 1} for s in range(3)}
    b = {s: {**a[s], "fail_replan": 0, "planning_wall_max_s": 29.0} for s in range(3)}
    v = C.verdict(a, b)
    assert v["adopt"] and v["sums"]["fail_replan"] == {"A": 3, "B": 0}
    b2 = {s: {**b[s], "planning_wall_max_s": 31.0} for s in range(3)}
    assert not C.verdict(a, b2)["adopt"] and not C.verdict(a, b2)["rules"]["wall_max_within"]
    with pytest.raises(ValueError):
        C.verdict(a, {0: b[0]})


def test_the_run_lists():
    smoke = R.rows("acs_x", "smoke")
    assert len(smoke) == 1 and "--short-cusp-arc 0.6,1.0" in smoke[0]["env"]["EXTRA_ARGS"]
    comp = R.rows("acs_x", "compare")
    assert len(comp) == 20 and sum("--short-cusp-arc" in r["env"]["EXTRA_ARGS"] for r in comp) == 10
    assert all("--deterministic" in r["env"]["EXTRA_ARGS"] and r["env"]["SCENE"] == "aisle_rack" for r in comp)
    assert {r["env"]["SEED"] for r in comp} == {str(s) for s in range(10)}


def test_arm_reads_cusp_heading_runs(monkeypatch, tmp_path):
    """arm() takes cusp_heading.Run objects (.name, .result), one analyse per run (a bug first met on the batch)."""
    class FakeCH:
        class Run:
            def __init__(self, name, result):
                self.name, self.result = name, result

        def runs_from_raw(self, roots, name_filter):
            return [self.Run("acsA_aisle_rack_s3", {"success": True, "planning_wall_s": 4.0, "paths": {}})]

        def analyse(self, runs, period, tdep):
            assert len(runs) == 1 and period == "postfix"
            return {"counts": {"failures_replan": 1, "failures_final": 0, "judgements": 7}}

    out = C.arm(tmp_path, FakeCH())
    assert out[3]["fail_replan"] == 1 and out[3]["success"] is True and out[3]["planning_wall_max_s"] == 4.0
