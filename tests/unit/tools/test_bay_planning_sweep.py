"""Resume and failure paths of tools/bay_planning_sweep.py (the results are not frozen)."""

import json

import numpy as np
from pathlib import Path

from tools import bay_planning_sweep as sweep

ROOT = Path(__file__).resolve().parents[3]
BASE = [
    "--seeds",
    "0",
    "--settings",
    str(ROOT / "config/isaac_transport.yaml"),
    "--forklift-urdf",
    str(ROOT / "sim/models/dls08_provisional/forklift.urdf"),
    "--retry-expansions",
    "0",
]


def _record(out: Path) -> dict:
    return json.loads((out / "seed_0.json").read_text())


def test_timeout_keeps_the_scene_and_a_rerun_with_the_same_inputs_is_skipped(tmp_path):
    out = tmp_path / "run"
    assert sweep.main([*BASE, "--timeout-s", "0.01", "--output", str(out)]) == 0
    first = _record(out)
    assert first["outcome"] in ("timeout", "crash")
    stamp = (out / "seed_0.json").stat().st_mtime_ns
    assert sweep.main([*BASE, "--timeout-s", "0.01", "--output", str(out)]) == 0
    assert (out / "seed_0.json").stat().st_mtime_ns == stamp


def test_changing_the_budget_reruns_the_seed(tmp_path):
    out = tmp_path / "run"
    sweep.main([*BASE, "--max-expansions", "10", "--output", str(out)])
    small = _record(out)
    sweep.main([*BASE, "--max-expansions", "30000", "--output", str(out)])
    large = _record(out)
    assert small["input_sha256"] != large["input_sha256"]
    assert large["outcome"] == "success"
    assert small["status"] != large["status"]


def test_a_crash_is_not_read_back_as_an_older_success(tmp_path):
    out = tmp_path / "run"
    out.mkdir()
    (out / "seed_0.json").write_text(
        json.dumps({"seed": 0, "input_sha256": "OLD", "outcome": "success"})
    )
    (out / "seed_0.trace.jsonl").write_text('{"stage": "OLD"}\n')
    broken = tmp_path / "pallet.yaml"
    broken.write_text("not: [a pallet geometry")
    sweep.main([*BASE, "--pallet-geometry", str(broken), "--output", str(out)])
    record = _record(out)
    assert record["outcome"] == "crash" and record["input_sha256"] != "OLD"
    assert not (out / "seed_0.trace.jsonl").exists()


def test_the_factory_mode_plans_the_runners_mission_on_the_v10_scene(tmp_path):
    # Priority-5 plan D7 CPU baseline: observe -> approach -> transport -> home on the
    # factory hall with props under 1.15 m removed, as the runner assembles it (seed 1:
    # 48 removed, 42 kept -- the l1p run's scene).
    out = tmp_path / "factory"
    assert sweep.main([
        "--layout", "factory", "--seeds", "1", "--output", str(out),
        "--settings", str(ROOT / "config/isaac_transport_measured.yaml"),
        "--forklift-urdf", str(ROOT / "sim/models/dls08_measured/forklift.urdf"),
    ]) == 0
    record = json.loads((out / "seed_1.json").read_text())
    assert (record["props_removed"], record["props_kept"]) == (48, 42)
    assert record["outcome"] == "success" and record["observation"] == {"candidate_index": 0, "extended": False}
    stages = [t["stage"] for t in record["trace"]]
    assert stages[0] == "observe_candidate" and stages[-1] == "return_home"
    assert all("gear_changes" in t for t in record["trace"] if t["stage"] == "observe_candidate")
    inputs = json.loads((out / "inputs.json").read_text())
    assert (inputs["alignment_straight_m"], inputs["delivery_straight_m"], inputs["withdrawal_m"]) == (2.1, 1.5, 0.55)
    assert inputs["search_budget_s"] == 20.0 and inputs["return_home"]


def test_each_search_ladder_gets_its_own_budget(monkeypatch):
    from forklift_core.planning import pallet_mission

    seen = []

    def fake(start, goal, obstacles, footprint, bounds, config, **kw):
        seen.append(kw.get("deadline"))
        return pallet_mission.PlanResult(True, "success", np.zeros((1, 3)), np.zeros(1, np.int8), np.zeros(1), 0.0, 1)

    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", fake)
    budget = pallet_mission.SearchBudget(20.0)
    import time

    before = time.monotonic()
    pallet_mission._search(None, None, [], None, None, pallet_mission.make_transport_planner_config(), deadline=budget)
    pallet_mission._search(None, None, [], None, None, pallet_mission.make_transport_planner_config(), deadline=budget)
    assert len(seen) == 2 and all(isinstance(d, float) for d in seen)
    assert before + 20.0 <= seen[0] <= seen[1] <= time.monotonic() + 20.0
    assert len(budget.spent_s) == 2


def test_the_runner_reads_the_shared_observation_candidates():
    from forklift_core.planning.observation_viewpoints import DEFAULT_OBSERVATION_WAYPOINTS

    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert "args.observation_waypoints = [list(w) for w in DEFAULT_OBSERVATION_WAYPOINTS]" in source
    assert DEFAULT_OBSERVATION_WAYPOINTS[0] == (-0.10, 0.90, 0.0) and len(DEFAULT_OBSERVATION_WAYPOINTS) == 8


def test_a_ladder_that_ends_after_its_budget_fails(monkeypatch):
    # Codex D7 3rd P2: a goal connection found past the last 256-expansion check came
    # back as success after the budget.
    from forklift_core.planning import pallet_mission

    clock = iter([100.0, 121.0])
    monkeypatch.setattr(pallet_mission.time, "monotonic", lambda: next(clock))

    def slow(start, goal, obstacles, footprint, bounds, config, **kw):
        return pallet_mission.PlanResult(True, "success", np.zeros((2, 3)), np.ones(2, np.int8), np.zeros(2), 1.0, 1)

    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", slow)
    budget = pallet_mission.SearchBudget(20.0)
    result = pallet_mission._search(None, None, [], None, None, pallet_mission.make_transport_planner_config(),
                                    deadline=budget)
    assert result.status == "timeout" and not result.success and len(result.poses) == 0
    assert budget.spent_s == [21.0]


def test_the_factory_mode_refuses_a_provisional_scene(tmp_path):
    import pytest

    with pytest.raises(SystemExit):
        sweep.main(["--layout", "factory", "--seeds", "1", "--output", str(tmp_path / "x"), "--scene-from",
                    "provisional", "--settings", str(ROOT / "config/isaac_transport_measured.yaml"),
                    "--forklift-urdf", str(ROOT / "sim/models/dls08_measured/forklift.urdf")])
