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
