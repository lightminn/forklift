"""The latency test's run lists (docs/plans/2026-11-11-latency-tolerance.md 3, 8): the registered arms, counts and
settings, rows the bundle accepts, reruns under new names."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("lat_lists", ROOT / "docs/validation/2026-11-11-latency/lat_lists.py")
L = importlib.util.module_from_spec(spec)
spec.loader.exec_module(L)
sys.path.insert(0, str(ROOT / "tools"))
import isaac_bundle as B  # noqa: E402


def test_a_has_the_registered_80_runs():
    rows = L.a_rows("/s", "/o")
    arms = [(r["env"]["LAT_ARM"], r["env"]["SCENE"], r["env"]["SEED"]) for r in rows]
    assert len(rows) == 80
    aisle = [a for a in arms if a[1] == "aisle_rack"]
    assert sorted(aisle) == sorted((arm, "aisle_rack", str(s)) for s in (0, 1, 2) for arm in ("S0", "S12", "U12", "C6"))
    for scene in ("line_2", "random_1"):
        main = [r for r in rows if r["env"]["SCENE"] == scene]
        assert len(main) == 34  # 10 arms x 3 seeds + S0, L0, B0 rep, C6 rep
    for r in rows:
        e = r["env"]
        assert e["VIDEO"] == "off" and e["RETURN_START"] == "boxed" and e["NEAR_END_STAND"] == "on"
        assert e["EXTRA_ARGS"].startswith("--deterministic --lidar-cast loop")
        assert "FORKLIFT_GRID_CLEAR_KERNEL" not in e and "VIEWS" not in e
    j = next(r for r in rows if r["name"] == "lat_J_line_2_s1")
    assert j["env"]["EXTRA_ARGS"].endswith("--latency-slam jitter:12 --latency-layer jitter:12")


def test_the_rows_are_bundle_rows(tmp_path):
    for rows in (L.a_rows("/s", "/o"), L.smoke_rows("/s", "/o"), L.b_jobs("/s", "/o")):
        text = "".join(json.dumps(r) + "\n" for r in rows)
        assert len(B.load_runs(tmp_path / "x.jsonl", text=text)) == len(rows)


def test_b_is_three_rounds_of_five_delays_the_second_reversed():
    jobs = L.b_jobs("/s", "/o")
    ds = [int(j["env"]["LAT_DELAY_MS"]) for j in jobs]
    assert ds == [0, 25, 50, 100, 500, 500, 100, 50, 25, 0, 0, 25, 50, 100, 500]
    assert all(("LATENCY_PLAN_MS" in j["env"]) == (d > 0) for j, d in zip(jobs, ds))
    assert jobs[0]["env"]["SET"] == "far" and jobs[0]["env"]["LIDAR_CAST"] == "fast"


def test_reruns_take_new_names(tmp_path):
    assert L.main(["rerun", "/s", str(tmp_path), "lat_U6_line_2_s2", "lat_b03_d50_r1"]) == 0
    (row,) = [json.loads(x) for x in (tmp_path / "RUNS_rerun.jsonl").read_text().splitlines()]
    assert row["name"] == "lat_r_lat_U6_line_2_s2" and row["env"]["LAT_ARM"] == "U6"
    assert (tmp_path / "lat_r_lat_b03_d50_r1.jsonl").exists()
    with pytest.raises(SystemExit):
        L.main(["rerun", "/s", str(tmp_path), "lat_X_line_2_s0"])


def test_a_b_out_too_long_for_the_slam_socket_is_refused():
    with pytest.raises(SystemExit):
        L.b_entry("/s", "/home/projects/forklift/test_runs/latency/smoke2_d32554f6", "lat_sm_B50", 50)
    assert L.b_entry("/s", "/home/projects/forklift/test_runs/lat/sm", "lat_sm_B50", 50)
    L.b_jobs("/s", "/home/projects/forklift/test_runs/lat/b_99d08772")  # the main B root fits
