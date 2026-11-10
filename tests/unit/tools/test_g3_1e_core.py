"""1e stage 1 (docs/plans/2026-11-10-g3-1e-process-worker.md 3): the list (D T T D, T pinned), the judgement (k only
at or below the executed k, common to both pairs; net and direct W; thread first), the invalidity kinds, and the
report-only readers (the worker's scheduler counters, k = 4's corrected wait, the thread sampler)."""

import importlib.util
import json
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
D = ROOT / "docs" / "validation" / "2026-11-10-g3-1e-core"


def load(name):
    spec = importlib.util.spec_from_file_location(name, D / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


lists = load("g3_1e_core_lists")
an = load("g3_1e_core_analyze")
smp = load("g3_1e_thread_sampler")


def test_the_list_is_d_t_t_d_with_t_pinned():
    e = lists.entries("/snap", "/out")
    assert [(x["env"]["ARM_NAME"], x["env"]["PAIR"]) for x in e] == [("D", "1"), ("T", "1"), ("T", "2"), ("D", "2")]
    for x in e:
        env = x["env"]
        assert (env["FORKLIFT_GRID_CLEAR_KERNEL"], env["FORKLIFT_SHADOW_KERNEL"], env["LIDAR_CAST"], env["SET"],
                env["ARM"], env["FACTOR"], env["INSTR"], env["LAYER_WORKER_K"]) == \
            ("1", "1", "fast", "far", "K", "1.0", "off", "3")
        assert env["LAYER_WORKER"] == ("delay" if env["ARM_NAME"] == "D" else "thread")
        assert (env.get("LAYER_WORKER_CPUS") == "auto") is (env["ARM_NAME"] == "T")
    assert {x["env"]["LAYER_WORKER_K"] for x in lists.entries("/s", "/o", k=4)} == {"4"}
    assert lists.entries("/s", "/o", k=4)[0]["name"] == "g3ec4_01_D"
    with pytest.raises(ValueError):
        lists.entries("/s", "/o", k=2)


def test_the_wrapper_adds_the_cpus_argument_only_with_a_worker():
    text = (ROOT / "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh").read_text()
    assert '[ -n "${LAYER_WORKER:-}" ] && [ -n "${LAYER_WORKER_CPUS:-}" ] && LW="$LW --layer-worker-cpus' in text


def m(main_layer, other, W, waits, barrier_wait=0.0, scans=10.0):
    return {"main_layer": main_layer, "main_other": other, "W": W, "scans_per_sim_s": scans,
            "barrier": {"wait": barrier_wait},
            "paired_wait_ms": {k: (None if v is None else {"mean": v}) for k, v in waits.items()}}


def test_k_is_read_only_at_or_below_the_executed_k_and_common_to_both_pairs():
    t1 = m(0.03, 0.40, 1.0, {2: 0.4, 3: 0.1, 4: 0.0})
    t2 = m(0.03, 0.40, 1.0, {2: 0.9, 3: 0.2, 4: 0.0})
    assert an.choose_k([t1, t2], 3) == 3  # k = 2 passes in one pair only
    assert an.choose_k([t1, t2], 2) is None
    t3 = m(0.03, 0.40, 1.0, {2: 3.0, 3: 0.7, 4: 0.0})  # k = 4's 0 is structural in a k = 3 run: never read
    assert an.choose_k([t1, t3], 3) is None


def test_thread_first_needs_net_and_direct_in_both_pairs():
    d = m(0.095, 0.66, 0.90, {2: None, 3: None})
    t = m(0.025, 0.66, 0.84, {2: 1.0, 3: 0.2})
    r = an.judge({1: {"D": d, "T": t}, 2: {"D": d, "T": t}}, 3)
    # net = 0.070 - 0.0002 x 10 = 0.068; direct 0.06 >= 0.034
    assert r["k"] == 3 and not r["k_is_prediction"] and r["decision"] == "(a) thread plus a core"
    assert abs(r["pairs"][1]["net"] - 0.068) < 1e-12
    t_weak = m(0.025, 0.66, 0.89, {2: 1.0, 3: 0.2})  # direct 0.01 < net / 2
    r2 = an.judge({1: {"D": d, "T": t}, 2: {"D": d, "T": t_weak}}, 3)
    assert r2["decision"] == "stage 2 to the lead" and not r2["pairs"][2]["passes"]
    t_slow = m(0.025, 0.66, 0.84, {2: 3.0, 3: 0.8})
    r3 = an.judge({1: {"D": d, "T": t_slow}, 2: {"D": d, "T": t}}, 3)
    assert r3["decision"] == "supplementary k = 4 pair due"
    assert an.judge({1: {"D": d, "T": t_slow}, 2: {"D": d, "T": t}}, 4)["decision"] == "stage 2 to the lead"


class S:
    def __init__(self, invalid, failed=None):
        self.invalid, self.worker_failed = invalid, failed


def test_the_invalidity_kinds_keep_worker_and_mission_failures_out_of_the_rerun():
    k = an.kinds(S(["other users' CPU over 1 core in more than 5% of samples"]), {"mission_done": True})
    assert k["environment"] and not k["worker"] and not k["mission"]
    k = an.kinds(S([], failed="timeout after 0.5 s (scheduled)"), {"mission_done": False, "mission_outcomes": [1]})
    assert not k["environment"] and k["worker"] and k["mission"]
    k = an.kinds(S([]), {"mission_done": True, "safety": {"forbidden_contacts": 1}})
    assert k["mission"] and not k["environment"]


def test_the_scheduler_report_and_k4_correction():
    inst = [{"run_ns": 5_000_000, "run_delay_ns": 3_000_000, "cpu": 13, "compute_s": 0.010},
            {"run_ns": None, "compute_s": 0.010}]
    rep = an.scheduler_report(inst)
    assert rep["jobs"] == 1 and abs(rep["cpu_over_wall"] - 0.5) < 1e-12 and abs(rep["delay_over_wall"] - 0.3) < 1e-12
    rec = {"scan_tick": 10, "install_tick": 13, "reason": "scheduled", "submit_t": 0.0, "done_t": 0.020,
           "wait_s": 0.009, "install_s": 0.001, "tick_t": {"3": 0.011, "4": 0.025}}
    # uncorrected S(4) = 25 ms > 20 ms -> 0; corrected S(4) = 25 - 9 - 1 = 15 ms -> 5 ms
    q = an.k4_corrected_ms([rec])
    assert abs(q["mean"] - 5.0) < 1e-9


def test_the_thread_report_sums_consecutive_samples_inside_the_window():
    s = [{"wall": 0.0, "threads": {"1/1": ["kit", "kit", 100, 0, 0, 0]}},
         {"wall": 10.0, "threads": {"1/1": ["kit", "kit", 600, 0, 2_000_000_000, 0], "1/2": ["kit", "lw", 0, 0, 0, 1]}},
         {"wall": 20.0, "threads": {"1/1": ["kit", "kit", 1100, 0, 2_000_000_000, 0],
                                    "1/2": ["kit", "lw", 300, 0, 500_000_000, 1]}},
         {"wall": 99.0, "threads": {"1/1": ["kit", "kit", 9999, 0, 0, 0]}}]
    r = an.threads_report(s, (0.0, 20.0), 100)
    assert r["samples"] == 3 and r["missed_threads"] == "unknown"
    assert r["by_name"]["kit/kit"] == {"cpu_s": 10.0, "delay_s": 2.0, "threads": 1}
    assert r["by_name"]["kit/lw"] == {"cpu_s": 3.0, "delay_s": 0.5, "threads": 1}


def test_the_thread_sampler_reads_only_this_users_threads_in_this_cgroup(tmp_path):
    def proc(pid, tids, cg, comm="python"):
        p = tmp_path / str(pid)
        (p / "task").mkdir(parents=True)
        (p / "cgroup").write_text(cg)
        stat = f"{pid} ({comm}) S " + " ".join(["0"] * 10) + " 7 3 " + " ".join(["0"] * 23) + " 5 0\n"
        (p / "stat").write_text(stat)
        for t in tids:
            (p / "task" / str(t)).mkdir()
            (p / "task" / str(t) / "stat").write_text(stat.replace(f"({comm})", "(th one)"))
            (p / "task" / str(t) / "schedstat").write_text("1000 200 3\n")
    proc(10, [10, 11], "0::/job_1\n")
    proc(20, [20], "0::/other\n")
    out = smp.sample(str(tmp_path), uid=os.getuid(), cgroup="0::/job_1\n")
    assert sorted(out) == ["10/10", "10/11"]
    assert out["10/11"] == ["python", "th one", 10, 1000, 200, 5]


def test_the_list_must_be_registered(tmp_path, monkeypatch, capsys):
    f = tmp_path / "l.jsonl"
    f.write_text("\n".join(json.dumps(x) for x in reversed(lists.entries("/s", "/o"))))
    monkeypatch.setattr(sys, "argv", ["g3_1e_core_analyze.py", str(tmp_path), "--list", str(f)])
    assert an.main() == 2 and "list_not_registered" in capsys.readouterr().out
