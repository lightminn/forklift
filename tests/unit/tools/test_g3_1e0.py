"""Real-time plan 5.28 1e-0: the list (delay then thread, the registered switches) and the prediction rule (k chosen by
the mean paired wait, the net saving against 0.03707, the direct W difference beside it)."""

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
D = ROOT / "docs" / "validation" / "2026-11-10-g3-1e0"


def load(name):
    spec = importlib.util.spec_from_file_location(name, D / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


lists = load("g3_1e0_lists")
an = load("g3_1e0_analyze")


def test_the_list_holds_delay_then_thread_with_the_registered_switches():
    e = lists.entries("/snap", "/out")
    assert [x["env"]["LAYER_WORKER"] for x in e] == ["delay", "thread"]
    for x in e:
        env = x["env"]
        assert (env["FORKLIFT_GRID_CLEAR_KERNEL"], env["FORKLIFT_SHADOW_KERNEL"], env["LIDAR_CAST"],
                env["LAYER_WORKER_K"], env["SET"], env["ARM"], env["FACTOR"], env["INSTR"]) == \
            ("1", "1", "fast", "3", "far", "K", "1.0", "off")
        assert x["argv"][1].endswith("r1c_session.sh")


def test_the_rerun_list_adds_one_off_control_after_the_registered_two():
    """docs/validation/2026-11-10-g3-1e0-run.md: the re-measurement adds a session without a worker (report only)."""
    e = lists.entries("/snap", "/out", control=True)
    assert [x["name"] for x in e] == ["g3e0_01_delay", "g3e0_02_thread", "g3e0_03_off"]
    off = e[2]["env"]
    assert "LAYER_WORKER" not in off and off["ARM_NAME"] == "off"  # r1c_session adds no --layer-worker
    assert {k: v for k, v in off.items() if k != "ARM_NAME"} == \
        {k: v for k, v in e[0]["env"].items() if k not in ("LAYER_WORKER", "ARM_NAME")}
    assert lists.entries("/snap", "/out")[:2] == e[:2]


def m(main_layer, other, W, waits, barrier_wait=0.0, scans=10.0):
    return {"main_layer": main_layer, "main_other": other, "W": W, "scans_per_sim_s": scans,
            "barrier": {"wait": barrier_wait},
            "paired_wait_ms": {k: (None if v is None else {"mean": v}) for k, v in waits.items()}}


def test_k_is_the_smallest_with_a_small_wait_and_the_net_decides():
    delay = m(0.090, 0.400, 1.05, {2: None, 3: None, 4: None})
    thread = m(0.030, 0.405, 1.00, {2: 2.0, 3: 0.4, 4: 0.0}, barrier_wait=0.002)
    r = an.predict(delay, thread)
    assert r["chosen_k"] == 3 and r["chosen_by_rule"]
    # moved 0.060 - 0.4 ms x 10 /s - 0.002 - 0.005 = 0.049
    assert abs(r["net_chosen"] - 0.049) < 1e-12 and r["decision"] == "go to 1e-1"
    assert abs(r["direct"] - 0.05) < 1e-12 and abs(r["residual_direct_minus_net"] - 0.001) < 1e-12


def test_no_k_with_a_small_wait_takes_4_and_a_short_net_goes_to_the_lead():
    delay = m(0.060, 0.400, 1.05, {2: None, 3: None, 4: None})
    thread = m(0.030, 0.390, 1.03, {2: 5.0, 3: 3.0, 4: 1.0})
    r = an.predict(delay, thread)
    assert r["chosen_k"] == 4 and not r["chosen_by_rule"]
    assert abs(r["net_chosen"] - 0.020) < 1e-12  # main_other fell: no increase is subtracted
    assert r["decision"] == "process design to the lead"


def test_the_list_must_be_delay_then_thread(tmp_path, monkeypatch, capsys):
    f = tmp_path / "l.jsonl"
    f.write_text("\n".join(json.dumps(x) for x in reversed(lists.entries("/s", "/o"))))
    monkeypatch.setattr(sys, "argv", ["g3_1e0_analyze.py", str(tmp_path), "--list", str(f)])
    assert an.main() == 2 and "list_not_registered" in capsys.readouterr().out


def test_an_off_control_may_follow_and_nothing_else(tmp_path, monkeypatch, capsys):
    e = lists.entries("/s", "/o", control=True)
    for bad in (e[1:] + e[:1], e + [e[2]], [e[0], e[2], e[1]]):
        f = tmp_path / "l.jsonl"
        f.write_text("\n".join(json.dumps(x) for x in bad))
        monkeypatch.setattr(sys, "argv", ["g3_1e0_analyze.py", str(tmp_path), "--list", str(f)])
        assert an.main() == 2 and "list_not_registered" in capsys.readouterr().out
    assert an.registered_arms([x["env"].get("LAYER_WORKER") or "off" for x in e])


def test_the_off_control_never_changes_the_judgement():
    """A failed or missing control is reported beside the result; the result is the registered two's alone."""
    result = {"chosen_k": 3, "decision": "go to 1e-1"}
    for control in (None, {"invalid": ["no result.json"]}, {"mission_done": False}):
        out = an.attach_control({"result": dict(result)}, control)
        assert out["result"] == result and out["control"] == control


def test_a_control_that_cannot_be_read_is_reported_not_raised(tmp_path, monkeypatch):
    e = lists.entries("/s", "/o", control=True)
    assert an.read_control(tmp_path, e[:2]) is None

    def boom(*a, **k):
        raise RuntimeError("no run")

    monkeypatch.setattr(an, "OffSession", boom)
    out = an.read_control(tmp_path, e)
    assert out == {"invalid": ["control unreadable: RuntimeError: no run"]}


def test_the_evidence_stale_ticks_are_read_per_phase():
    res = {"obstacle_layer": {"slowed_reasons": {"transport:evidence_stale": 768, "transport:limit": 3,
                                                 "return_home:evidence_stale": 670, "observe:ok": 5}}}
    assert an.stale_ticks(res) == {"transport": 768, "return_home": 670, "total": 1438}
    assert an.stale_ticks({}) == {"total": 0}


def test_the_other_work_takes_row_i_steps_and_row_i_plus_1_controls():
    """Codex 1e-0 round 3 P2: W_i holds row i's step and row i+1's control; the other work follows it, and an install
    is taken out when it ran in such a control tick."""
    import numpy as np

    C = an.C
    D = np.full((4, len(C)), np.nan)
    for i in range(4):
        D[i, C["tick"]] = 100 + i
        D[i, C["world_start"]], D[i, C["world_end"]] = 0.0, 0.002 * (i + 1)
        D[i, C["ctrl_start"]], D[i, C["ctrl_end"]] = 0.0, 0.010 * (i + 1)
    ok = np.array([False, True, True, False])
    installs = [{"install_tick": 102, "reason": "scheduled", "wait_s": 0.001, "install_s": 0.002},
                {"install_tick": 101, "reason": "scheduled", "wait_s": 0.5, "install_s": 0.5},  # row 1's own control
                {"install_tick": 103, "reason": "add_scans", "wait_s": 0.5, "install_s": 0.5}]  # in the scan step
    raw, taken, n = an.other_work(D, ok, installs)
    assert abs(raw - ((0.004 + 0.006) + (0.030 + 0.040))) < 1e-12
    assert (abs(taken - 0.003) < 1e-12, n) == (True, 1)
