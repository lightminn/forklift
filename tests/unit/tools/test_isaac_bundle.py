"""The Isaac bundle runner (docs/plans/2026-10-24-ws1-isaac-bundle.md): several runs in one Slurm job, by slots."""

import json
import os
import signal
import sys
import threading
import time

import pytest

from tools import isaac_bundle as ib

PY = sys.executable


def py(code, *args):
    return [PY, "-c", code, *args]


def stamp_run(name, tmp_path, sleep_s=0.3, code=0, **extra):
    """A run that writes its start/end times and bundle env to <tmp>/<name>.json, sleeps, and exits with code."""
    body = (
        "import json,os,sys,time; t0=time.time(); time.sleep(float(sys.argv[2]));"
        "keys=['SESSION_ID','BRIDGE_CPU','PORT','BUNDLE_SLOT','BUNDLE_JOB','BUNDLE_RUN','MARK'];"
        "json.dump({'t0':t0,'t1':time.time(),'env':{k:os.environ.get(k) for k in keys}},open(sys.argv[1],'w'));"
        "sys.exit(int(sys.argv[3]))"
    )
    return ib.Run(name=name, argv=py(body, str(tmp_path / f"{name}.json"), str(sleep_s), str(code)), **extra)


def bundle(tmp_path, runs, **k):
    k.setdefault("slots", 2)
    k.setdefault("stagger_s", 0.0)
    k.setdefault("grace_s", 1.0)
    k.setdefault("poll_s", 0.02)
    k.setdefault("job_id", "77")
    k.setdefault("cpus", [4, 5, 16, 17])
    k.setdefault("cleanup", lambda session_id: [])
    k.setdefault("leftover_grace_s", 1.0)
    k.setdefault("oom_counter", None)
    return ib.run_bundle(runs, tmp_path / "out", **k)


def stamps(tmp_path, names):
    return {n: json.loads((tmp_path / f"{n}.json").read_text()) for n in names}


def results(tmp_path):
    return {r["name"]: r for r in map(json.loads, (tmp_path / "out" / "results.jsonl").read_text().splitlines())}


# --- the run list ---------------------------------------------------------------------------------------------------


def test_a_run_list_is_read_from_json_lines(tmp_path):
    path = tmp_path / "runs.jsonl"
    path.write_text(
        json.dumps({"name": "a", "argv": ["true"], "env": {"SEED": "1"}, "cwd": str(tmp_path)})
        + "\n\n"
        + json.dumps({"name": "b.2", "argv": ["false"]})
        + "\n"
    )
    runs = ib.load_runs(path)
    assert [r.name for r in runs] == ["a", "b.2"]
    assert runs[0].env == {"SEED": "1"} and runs[0].cwd == str(tmp_path)
    assert runs[1].env == {} and runs[1].cwd is None


@pytest.mark.parametrize(
    "lines, word",
    [
        ([{"name": "a", "argv": ["true"]}, {"name": "a", "argv": ["true"]}], "twice"),
        ([{"name": "a/b", "argv": ["true"]}], "name"),
        ([{"name": "a", "argv": []}], "argv"),
        ([{"name": "a", "argv": "true"}], "argv"),
        ([{"name": "a", "argv": ["true"], "env": {"X": 1}}], "env"),
        ([{"name": "a", "argv": ["true"], "extra": 1}], "unknown"),
        ([{"name": "a", "argv": ["true"], "env": {"SESSION_ID": "x"}}], "sets"),
        ([{"name": "a", "argv": ["true"], "env": {"BUNDLE_TOKEN": "x"}}], "sets"),
        ([{"name": "a", "argv": ["true"], "env": {"PORT": "9001"}},
          {"name": "b", "argv": ["true"], "env": {"PORT": "9001"}}], "PORT"),
    ],
)
def test_a_bad_run_list_is_refused(tmp_path, lines, word):
    path = tmp_path / "runs.jsonl"
    path.write_text("".join(json.dumps(x) + "\n" for x in lines))
    with pytest.raises(ValueError, match=word):
        ib.load_runs(path)


# --- slots, stagger, isolation ----------------------------------------------------------------------------------------


def test_no_more_runs_than_slots_run_at_once(tmp_path):
    names = [f"r{i}" for i in range(5)]
    summary = bundle(tmp_path, [stamp_run(n, tmp_path, 0.3) for n in names], slots=2)
    s = stamps(tmp_path, names)
    edges = sorted([(v["t0"], 1) for v in s.values()] + [(v["t1"], -1) for v in s.values()])
    level = peak = 0
    for _, d in edges:
        level += d
        peak = max(peak, level)
    assert peak == 2
    assert summary["runs"] == 5 and summary["failed"] == 0 and summary["exit_code"] == 0


def test_starts_are_staggered(tmp_path):
    names = ["a", "b", "c"]
    bundle(tmp_path, [stamp_run(n, tmp_path, 1.0) for n in names], slots=3, stagger_s=0.3)
    starts = sorted(results(tmp_path)[n]["start"] for n in names)
    assert all(b - a >= 0.29 for a, b in zip(starts, starts[1:], strict=False))


def test_a_failed_run_does_not_stop_the_others(tmp_path):
    runs = [stamp_run("ok1", tmp_path, 0.2), stamp_run("bad", tmp_path, 0.05, code=3), stamp_run("ok2", tmp_path, 0.2)]
    summary = bundle(tmp_path, runs, slots=1)
    r = results(tmp_path)
    assert (r["ok1"]["rc"], r["bad"]["rc"], r["ok2"]["rc"]) == (0, 3, 0)
    assert summary["failed"] == 1 and summary["exit_code"] == 1
    assert (tmp_path / "out" / "runs" / "bad.1.log").exists()


def test_each_run_gets_its_own_session_bridge_cpu_and_port(tmp_path):
    names = ["a", "b", "c"]
    runs = [stamp_run(n, tmp_path, 0.5) for n in names]
    runs[2] = stamp_run("c", tmp_path, 0.5, env={"MARK": "x", "PORT": "9999"})
    bundle(tmp_path, runs, slots=3, cpus=[3, 6, 15, 18])
    env = {n: v["env"] for n, v in stamps(tmp_path, names).items()}
    assert [env[n]["SESSION_ID"] for n in names] == ["77s0", "77s1", "77s2"]
    assert [env[n]["BUNDLE_SLOT"] for n in names] == ["0", "1", "2"]
    assert [env[n]["BRIDGE_CPU"] for n in names] == ["3", "6", "15"]
    assert env["a"]["PORT"] != env["b"]["PORT"]
    assert env["c"]["PORT"] == "9999" and env["c"]["MARK"] == "x"  # the list's env wins
    assert env["a"]["BUNDLE_JOB"] == "77" and env["b"]["BUNDLE_RUN"] == "b"


def test_a_free_slot_takes_the_lowest_slot_number(tmp_path):
    runs = [stamp_run("long", tmp_path, 0.8), stamp_run("short", tmp_path, 0.1), stamp_run("next", tmp_path, 0.1)]
    bundle(tmp_path, runs, slots=2)
    r = results(tmp_path)
    assert (r["long"]["slot"], r["short"]["slot"], r["next"]["slot"]) == (0, 1, 1)


# --- ending runs --------------------------------------------------------------------------------------------------------

TERM_CHILD = (
    "import signal,sys,time,pathlib; d=pathlib.Path(sys.argv[1]);"
    "signal.signal(signal.SIGTERM, lambda *a: (d.joinpath('child_term').write_text('1'), sys.exit(0)));"
    "signal.signal(signal.SIGUSR1, lambda *a: (d.joinpath('child_usr1').write_text('1'), sys.exit(0)));"
    "time.sleep(30)"
)


def test_a_timeout_terms_only_the_top_process(tmp_path):
    # the top process (bash) forwards its own way (USR1 here); the child must not get the TERM directly
    script = (
        f'{PY} -c "$CHILD" "$D" & c=$!\n'
        'trap \'touch "$D/leader_term"; kill -USR1 $c; wait $c; exit 5\' TERM\n'
        "wait $c\n"
    )
    run = ib.Run(name="t", argv=["bash", "-c", script], env={"CHILD": TERM_CHILD, "D": str(tmp_path)})
    bundle(tmp_path, [run], run_timeout_s=0.5)
    r = results(tmp_path)["t"]
    assert r["timeout"] is True and r["rc"] == 5
    assert (tmp_path / "leader_term").exists() and (tmp_path / "child_usr1").exists()
    assert not (tmp_path / "child_term").exists()


def test_a_run_that_ignores_term_is_killed_after_the_grace(tmp_path):
    code = "import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)"
    t0 = time.monotonic()
    bundle(tmp_path, [ib.Run(name="stubborn", argv=py(code))], run_timeout_s=0.3, grace_s=0.4)
    r = results(tmp_path)["stubborn"]
    assert r["timeout"] is True and r["signal"] == signal.SIGKILL
    assert time.monotonic() - t0 < 5


def test_processes_left_by_a_finished_run_are_ended(tmp_path):
    pidfile = tmp_path / "orphan.pid"
    script = f'sleep 30 & echo $! > "{pidfile}"; exit 0'
    bundle(tmp_path, [ib.Run(name="o", argv=["bash", "-c", script])], grace_s=0.5)
    pid = int(pidfile.read_text())
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and ib.alive(pid):
        time.sleep(0.05)
    assert not ib.alive(pid)
    assert results(tmp_path)["o"]["leftover"] == 1


def test_a_stop_terms_running_runs_and_starts_no_more(tmp_path):
    stop = threading.Event()
    threading.Timer(0.4, stop.set).start()
    runs = [stamp_run("a", tmp_path, 5.0), stamp_run("b", tmp_path, 5.0), stamp_run("c", tmp_path, 0.1)]
    t0 = time.monotonic()
    summary = bundle(tmp_path, runs, slots=2, stop=stop)
    r = results(tmp_path)
    assert r["a"]["signal"] == signal.SIGTERM and r["b"]["signal"] == signal.SIGTERM
    assert r["c"]["status"] == "not_started"
    assert summary["not_started"] == 1 and summary["exit_code"] == 1
    assert time.monotonic() - t0 < 4


def test_leftover_bridge_containers_are_removed_per_session(tmp_path):
    seen = []
    bundle(tmp_path, [stamp_run("a", tmp_path, 0.05), stamp_run("b", tmp_path, 0.05)], slots=2,
           cleanup=lambda sid: seen.append(sid) or (["slam_bridge_77s1_e1"] if sid == "77s1" else []))
    assert sorted(seen) == ["77s0", "77s1"]
    r = results(tmp_path)
    assert r["a"]["containers_removed"] == [] and r["b"]["containers_removed"] == ["slam_bridge_77s1_e1"]


# --- the GPU ------------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "env, gpus",
    [
        ({"CUDA_VISIBLE_DEVICES": "0"}, ["0"]),
        ({"CUDA_VISIBLE_DEVICES": "0,1"}, ["0", "1"]),
        ({"SLURM_JOB_GPUS": "1"}, ["1"]),
        ({}, []),
    ],
)
def test_visible_gpus(env, gpus):
    assert ib.visible_gpus(env) == gpus


def test_two_visible_gpus_are_refused(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0,1")
    path = tmp_path / "runs.jsonl"
    path.write_text(json.dumps({"name": "a", "argv": ["true"]}) + "\n")
    assert ib.main(["--list", str(path), "--out", str(tmp_path / "out"), "--slots", "1", "--no-vram"]) == 2
    assert "GPU" in capsys.readouterr().err


def test_the_vram_budget_holds_the_next_start(tmp_path):
    # the first run reads 9,000 MiB: 9,000 + 3,950 > 3 x 4,076, so the second waits for the first to end
    def probe(leaders):
        return ([(leaders[0], 9000, 0)] if leaders else []), {0: 16000}

    runs = [stamp_run("big", tmp_path, 0.6), stamp_run("next", tmp_path, 0.1)]
    summary = bundle(tmp_path, runs, slots=3, shards=3, vram_probe=probe, vram_period_s=0.05)
    s = stamps(tmp_path, ["big", "next"])
    assert s["next"]["t0"] >= s["big"]["t1"] - 0.05
    assert summary["vram_waits"] >= 1
    assert results(tmp_path)["big"]["vram_max_mib"] == 9000 and results(tmp_path)["big"]["gpus"] == [0]


def test_vram_within_the_budget_does_not_hold_starts(tmp_path):
    def probe(leaders):
        return [(pid, 3900, 0) for pid in leaders], {0: 16000}

    names = ["a", "b", "c"]
    summary = bundle(tmp_path, [stamp_run(n, tmp_path, 0.5) for n in names], slots=3, shards=3, vram_probe=probe,
                     vram_period_s=0.05)
    s = stamps(tmp_path, names)
    assert max(v["t0"] for v in s.values()) < min(v["t1"] for v in s.values())  # all three overlapped
    assert summary["vram_waits"] == 0
    assert summary["vram_job_max_mib"] <= 3 * 3900


def test_a_run_alone_starts_even_over_the_budget(tmp_path):
    summary = bundle(tmp_path, [stamp_run("a", tmp_path, 0.1)], slots=1, shards=1, session_vram_mib=5000,
                     vram_probe=lambda leaders: ([], {0: 16000}), vram_period_s=0.05)
    assert summary["failed"] == 0


def test_a_summary_is_written(tmp_path):
    bundle(tmp_path, [stamp_run("a", tmp_path, 0.05)], slots=1)
    s = json.loads((tmp_path / "out" / "summary.json").read_text())
    assert s["runs"] == 1 and s["exit_code"] == 0 and s["slots"] == 1 and "wall_s" in s


def test_an_existing_output_is_refused(tmp_path):
    (tmp_path / "out").mkdir()
    (tmp_path / "out" / "results.jsonl").write_text("")
    with pytest.raises(FileExistsError):
        bundle(tmp_path, [stamp_run("a", tmp_path, 0.05)], slots=1)


def test_main_runs_a_list(tmp_path):
    path = tmp_path / "runs.jsonl"
    path.write_text(json.dumps({"name": "a", "argv": py("pass")}) + "\n")
    env = {k: v for k, v in os.environ.items() if k not in ("CUDA_VISIBLE_DEVICES", "SLURM_JOB_GPUS")}
    old = os.environ.copy()
    os.environ.clear()
    os.environ.update(env)
    try:
        rc = ib.main(["--list", str(path), "--out", str(tmp_path / "out"), "--slots", "1", "--stagger-s", "0",
                      "--no-vram", "--no-cleanup", "--node-ram-min-mib", "0"])
    finally:
        os.environ.clear()
        os.environ.update(old)
    assert rc == 0


def test_more_slots_than_shards_are_refused(tmp_path):
    with pytest.raises(ValueError, match="shard"):
        bundle(tmp_path, [stamp_run("a", tmp_path, 0.05)], slots=4, shards=3)


def test_a_child_in_its_own_process_group_is_killed_by_its_token(tmp_path):
    # GNU timeout puts its child in a process group of its own: the run's group KILL would miss it
    pidfile = tmp_path / "child.pid"
    child = ("import os,signal,sys,time; open(sys.argv[1],'w').write(str(os.getpid()));"
             "signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)")
    run = ib.Run(name="t", argv=["bash", "-c", f'timeout 60 {PY} -c "$C" "{pidfile}"; echo "timeout rc $?"'], env={"C": child})
    t0 = time.monotonic()
    bundle(tmp_path, [run], run_timeout_s=0.5, grace_s=0.5)
    pid = int(pidfile.read_text())
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and ib.alive(pid):
        time.sleep(0.05)
    assert not ib.alive(pid)
    assert time.monotonic() - t0 < 15


@pytest.mark.parametrize(
    "current, unattributed, free, verdict",
    [
        ([3900, 3900], 0, 8000, "ok"),  # 3 x 3,950 = 11,850 <= 12,228
        ([3900, 3900, 3900], 0, 4000, "over"),  # a fourth session over three shards
        ([3900, 4500], 0, 8000, "over"),  # one run above its share eats the margin
        ([3900, 3900], 600, 8000, "over"),  # job memory no run owns counts
        ([1000, 3900], 0, 6000, "gpu_full"),  # the warming run will still take 2,950 of the 6,000 free
        ([1000, 3900], 0, 7000, "ok"),
        ([], 0, None, "ok"),
    ],
)
def test_admits(current, unattributed, free, verdict):
    assert ib.admits(current, unattributed, budget_mib=3 * 4076, session_mib=3950, free_mib=free) == verdict


def test_an_unreadable_gpu_holds_starts_then_goes_on_by_the_slots(tmp_path):
    names = ["a", "b"]
    summary = bundle(tmp_path, [stamp_run(n, tmp_path, 0.6) for n in names], slots=2, shards=2,
                     vram_probe=lambda leaders: None, vram_period_s=0.05, vram_unreadable_s=0.3)
    s = stamps(tmp_path, names)
    assert s["a"]["t0"] - summary["start"] >= 0.25  # held, then on by the slots
    assert summary["unreadable_starts"] == 2 and summary["failed"] == 0


# --- infrastructure failures and retries -------------------------------------------------------------------------------

FLAKY = (
    "import pathlib,sys; d=pathlib.Path(sys.argv[1]); d.mkdir(parents=True);"
    "(d/'run.log').write_text('BFCArena::AllocateRawInternal bfc_arena failed\\n' if sys.argv[2]=='1' else 'ok\\n');"
    "sys.exit(1 if sys.argv[2]=='1' else 0)"
)


def test_a_gpu_allocation_failure_is_retried_into_the_same_folder(tmp_path):
    out = tmp_path / "sbr_s0"
    run = ib.Run(name="s0", argv=py(FLAKY, str(out), "{attempt}"), scan=[str(out / "run.log")],
                 retry_move=[str(out)])
    summary = bundle(tmp_path, [run])
    lines = [json.loads(x) for x in (tmp_path / "out" / "results.jsonl").read_text().splitlines()]
    assert [(x["attempt"], x["status"], x["retried"]) for x in lines] == [(1, "infra_failed", True), (2, "ended", False)]
    assert "bfc_arena" in lines[0]["infra"][0]
    assert (out / "run.log").read_text() == "ok\n"  # the final attempt keeps the folder name
    assert "bfc_arena" in (tmp_path / "sbr_s0.attempt1" / "run.log").read_text()
    assert summary["exit_code"] == 0 and summary["failed"] == 0 and summary["infra_failed_attempts"] == 1


def test_a_run_that_cannot_be_redone_is_recorded_as_an_infrastructure_failure(tmp_path):
    out = tmp_path / "x"
    run = ib.Run(name="x", argv=py(FLAKY, str(out), "1"), scan=[str(out / "run.log")])  # no retry_move, no {attempt}
    summary = bundle(tmp_path, [run])
    r = results(tmp_path)["x"]
    assert r["status"] == "infra_failed" and r["retried"] is False
    assert summary["failed"] == 1 and summary["exit_code"] == 1


def test_a_run_failing_on_its_own_is_not_retried(tmp_path):
    run = stamp_run("own", tmp_path, 0.05, code=4, retry_move=[str(tmp_path / "nothing")])
    summary = bundle(tmp_path, [run])
    r = results(tmp_path)["own"]
    assert r["status"] == "ended" and r["rc"] == 4 and r["retried"] is False and summary["infra_failed_attempts"] == 0


def test_an_oom_kill_during_a_failed_run_counts_as_infrastructure(tmp_path):
    counts = iter([0, 1, 1, 1])
    run = stamp_run("o", tmp_path, 0.05, code=137)
    bundle(tmp_path, [run], oom_counter=lambda: next(counts), retries=0)
    r = results(tmp_path)["o"]
    assert r["status"] == "infra_failed" and r["oom_kills_during"] == 1 and "oom_kill" in r["infra"][0]


def test_an_oom_kill_beside_a_run_that_exited_0_does_not_fail_it(tmp_path):
    counts = iter([0, 1])
    bundle(tmp_path, [stamp_run("fine", tmp_path, 0.05)], oom_counter=lambda: next(counts))
    r = results(tmp_path)["fine"]
    assert r["status"] == "ended" and r["oom_kills_during"] == 1


# --- real-time back-off -----------------------------------------------------------------------------------------------

PACED = (
    "import json,sys,time; f=open(sys.argv[1],'a'); t=0.0; end=time.monotonic()+float(sys.argv[3])\n"
    "while time.monotonic()<end:\n"
    "    t+=0.02*float(sys.argv[2]); f.write(json.dumps({'tick_t':t,'mono':time.monotonic(),'pacing_ok':False})+'\\n');"
    " f.flush(); time.sleep(0.02)"
)


def paced_run(name, tmp_path, ratio, duration_s):
    log = tmp_path / f"{name}_live_log.jsonl"
    return ib.Run(name=name, argv=py(PACED, str(log), str(ratio), str(duration_s)), pace_log=str(log))


def test_slow_live_runs_step_the_bundle_back(tmp_path):
    runs = [paced_run("a", tmp_path, 0.25, 1.5), paced_run("b", tmp_path, 0.25, 1.5), stamp_run("c", tmp_path, 0.05)]
    summary = bundle(tmp_path, runs, slots=2, pace_floor=0.345, pace_window_s=0.4, pace_warmup_s=0.3,
                     pace_period_s=0.1)
    assert summary["backoffs"] and summary["final_cap"] == 1
    r = results(tmp_path)
    assert r["c"]["start"] >= min(r["a"]["end"], r["b"]["end"]) - 0.05  # c waited for the cap
    assert r["a"]["pace_min"] < 0.345
    assert (tmp_path / "out" / "pace.jsonl").exists()


def test_live_runs_at_pace_do_not_step_back(tmp_path):
    runs = [paced_run("a", tmp_path, 0.4, 1.0), paced_run("b", tmp_path, 0.4, 1.0)]
    summary = bundle(tmp_path, runs, slots=2, pace_floor=0.345, pace_window_s=0.4, pace_warmup_s=0.3,
                     pace_period_s=0.1)
    assert summary["backoffs"] == [] and summary["final_cap"] == 2


def test_the_run_list_takes_scan_retry_move_and_pace_log(tmp_path):
    path = tmp_path / "runs.jsonl"
    path.write_text(json.dumps({"name": "a", "argv": ["true"], "scan": ["/x/*.log"], "retry_move": ["/x"],
                                "pace_log": "/x/live_log.jsonl"}) + "\n")
    (run,) = ib.load_runs(path)
    assert run.scan == ["/x/*.log"] and run.retry_move == ["/x"] and run.pace_log == "/x/live_log.jsonl"
    assert run.retryable


# --- Codex round 2 counterexamples -------------------------------------------------------------------------------------


def test_a_summary_line_naming_out_of_device_is_not_a_failure(tmp_path):
    # riib_scene prints "OUT_OF_DEVICE lines: 0" at every normal end
    run = ib.Run(name="n", argv=["bash", "-c", "echo 'runner exit 0'; echo 'OUT_OF_DEVICE lines: 0'"],
                 retry_move=[str(tmp_path / "nothing")])
    summary = bundle(tmp_path, [run])
    r = results(tmp_path)["n"]
    assert r["status"] == "ended" and r["retried"] is False and summary["infra_failed_attempts"] == 0


def test_each_attempt_is_judged_on_its_own_log(tmp_path):
    out = tmp_path / "f"
    code = ("import sys; print('bfc_arena.cc:360 void* onnxruntime::BFCArena::AllocateRaw failed') "
            "if sys.argv[1]=='1' else print('fine'); sys.exit(1 if sys.argv[1]=='1' else 0)")
    run = ib.Run(name="f", argv=py(code, "{attempt}"), retry_move=[str(out)])
    summary = bundle(tmp_path, [run], retries=2)
    lines = [json.loads(x) for x in (tmp_path / "out" / "results.jsonl").read_text().splitlines()]
    assert [(x["attempt"], x["status"]) for x in lines] == [(1, "infra_failed"), (2, "ended")]
    assert summary["exit_code"] == 0
    assert (tmp_path / "out" / "runs" / "f.1.log").exists() and (tmp_path / "out" / "runs" / "f.2.log").exists()


def test_a_run_holds_its_slot_until_what_it_left_has_ended(tmp_path):
    # the top process ends at once, its child ignores TERM for a while: the next run waits for the child
    marker = tmp_path / "child_end"
    child = f"import time,pathlib; time.sleep(0.8); pathlib.Path('{marker}').write_text(str(time.time()))"
    first = ib.Run(name="first", argv=["bash", "-c", f'(trap "" TERM; exec {PY} -c "$C") & exit 0'], env={"C": child})
    second = stamp_run("second", tmp_path, 0.05)
    bundle(tmp_path, [first, second], slots=1, leftover_grace_s=5.0)
    assert stamps(tmp_path, ["second"])["second"]["t0"] >= float(marker.read_text()) - 0.05
    assert results(tmp_path)["first"]["leftover"] == 1


def test_a_retry_waits_for_the_old_attempts_processes(tmp_path):
    out = tmp_path / "o"
    marker = tmp_path / "old_child_end"
    code = (
        "import os,sys,time,signal,subprocess,pathlib; a=sys.argv[1]; d=pathlib.Path(sys.argv[2]);"
        "d.mkdir(parents=True, exist_ok=True);"
        f"c=\"import time,pathlib; time.sleep(0.8); pathlib.Path('{marker}').write_text(str(time.time()))\";"
        "subprocess.Popen(['bash','-c','trap \\'\\' TERM; exec \"$0\" -c \"$1\"',sys.executable,c]) if a=='1' else None;"
        "(d/'mark').write_text(str(time.time()));"
        "print('CUDA out of memory') if a=='1' else None; sys.exit(1 if a=='1' else 0)"
    )
    run = ib.Run(name="o", argv=py(code, "{attempt}", str(out)), retry_move=[str(out)])
    bundle(tmp_path, [run], leftover_grace_s=5.0)
    assert float((out / "mark").read_text()) >= float(marker.read_text()) - 0.05  # attempt 2 started after
    assert (tmp_path / "o.attempt1" / "mark").exists()


def test_a_cleanup_that_fails_is_recorded_and_the_others_go_on(tmp_path):
    def cleanup(sid):
        raise TimeoutError("docker rm took too long")

    summary = bundle(tmp_path, [stamp_run("a", tmp_path, 0.05), stamp_run("b", tmp_path, 0.3)], cleanup=cleanup)
    r = results(tmp_path)
    assert "docker rm" in r["a"]["cleanup_error"] and r["b"]["status"] == "ended"
    assert summary["runs"] == 2 and summary["failed"] == 0


def test_a_move_that_cannot_happen_stops_the_retry_and_is_recorded(tmp_path):
    out = tmp_path / "m"
    out.mkdir()
    (tmp_path / "m.attempt1").mkdir()  # the destination is taken
    code = "import sys; print('CUDA out of memory'); sys.exit(1)"
    summary = bundle(tmp_path, [ib.Run(name="m", argv=py(code), retry_move=[str(out)])])
    r = results(tmp_path)["m"]
    assert r["retried"] is False and "attempt1" in r["move_error"] and summary["failed"] == 1


def test_an_oom_is_blamed_only_on_a_killed_run(tmp_path):
    counts = iter([0, 0, 1, 1])  # a's start, b's start, a's end, b's end
    runs = [stamp_run("other_failure", tmp_path, 0.2, code=7), stamp_run("killed", tmp_path, 0.3, code=137)]
    bundle(tmp_path, runs, slots=2, oom_counter=lambda: next(counts), retries=0)
    r = results(tmp_path)
    assert r["killed"]["status"] == "infra_failed" and "oom_kill" in r["killed"]["infra"][0]
    assert r["other_failure"]["status"] == "ended" and r["other_failure"]["oom_kills_during"] == 1


def test_a_run_whose_attempt_is_only_in_its_argv_is_retried(tmp_path):
    code = "import sys; print('CUDA_ERROR_OUT_OF_MEMORY') if sys.argv[1]=='1' else None; sys.exit(int(sys.argv[1]=='1'))"
    summary = bundle(tmp_path, [ib.Run(name="p", argv=py(code, "{attempt}"))])
    lines = [json.loads(x) for x in (tmp_path / "out" / "results.jsonl").read_text().splitlines()]
    assert [x["status"] for x in lines] == ["infra_failed", "ended"] and summary["exit_code"] == 0


def test_stopped_ticks_step_the_bundle_back(tmp_path):
    # ticks for 0.4 s at pace, then silence while the run lives on
    log = tmp_path / "stall_live_log.jsonl"
    code = PACED.replace("time.monotonic()+float(sys.argv[3])", "time.monotonic()+0.4") + "\ntime.sleep(1.5)"
    runs = [ib.Run(name="stall", argv=py(code, str(log), "0.4", "0"), pace_log=str(log)),
            ib.Run(name="ok", argv=py(PACED, str(tmp_path / "ok_log.jsonl"), "0.4", "2.0"),
                   pace_log=str(tmp_path / "ok_log.jsonl")),
            stamp_run("later", tmp_path, 0.05)]
    summary = bundle(tmp_path, runs, slots=2, pace_floor=0.1, pace_window_s=0.5, pace_warmup_s=0.3,
                     pace_period_s=0.1, pace_stall_s=0.4)
    assert summary["backoffs"] and "no tick" in summary["backoffs"][0]["reason"]


def test_the_first_run_waits_for_free_gpu_memory(tmp_path):
    frees = iter([1000, 1000, 1000] + [16000] * 1000)
    summary = bundle(tmp_path, [stamp_run("a", tmp_path, 0.05)], slots=1, shards=1,
                     vram_probe=lambda leaders: ([], {0: next(frees)}), vram_period_s=0.05)
    assert summary["gpu_full_waits"] == 1 and summary["failed"] == 0


# --- Codex round 3 counterexamples -------------------------------------------------------------------------------------


def test_a_child_forked_after_the_first_scan_still_holds_the_slot(tmp_path):
    # the top process leaves a child that soon forks a grandchild and exits: no scan may settle the run early
    marker = tmp_path / "grandchild_end"
    grand = f"import time,pathlib; time.sleep(0.6); pathlib.Path('{marker}').write_text(str(time.time()))"
    child = f"import subprocess,sys,time; time.sleep(0.05); subprocess.Popen([sys.executable,'-c',{grand!r}])"
    first = ib.Run(name="first", argv=["bash", "-c", f'(trap "" TERM; exec {PY} -c "$C") & exit 0'], env={"C": child})
    second = stamp_run("second", tmp_path, 0.05)
    bundle(tmp_path, [first, second], slots=1, leftover_grace_s=5.0, settle_quiet_s=0.3)
    assert stamps(tmp_path, ["second"])["second"]["t0"] >= float(marker.read_text()) - 0.05


def test_a_cleanup_failure_stops_the_retry_and_fails_the_bundle(tmp_path):
    def cleanup(sid):
        raise TimeoutError("docker rm took too long")

    code = "import sys; print('CUDA out of memory'); sys.exit(1)"
    summary = bundle(tmp_path, [ib.Run(name="c", argv=py(code), retry_move=[str(tmp_path / "c")])], cleanup=cleanup)
    r = results(tmp_path)["c"]
    assert r["retried"] is False and "docker rm" in r["cleanup_error"]
    assert summary["errors"] and summary["exit_code"] == 1


def test_the_exception_path_removes_the_bridges(tmp_path):
    seen = []

    def probe(leaders):
        if leaders:
            raise RuntimeError("probe broke")
        return [], {0: 16000}

    runs = [stamp_run("a", tmp_path, 3.0)]
    summary = bundle(tmp_path, runs, slots=1, shards=1, vram_probe=probe, vram_period_s=0.05, grace_s=0.5,
                     cleanup=lambda sid: seen.append(sid) or [])
    assert seen == ["77s0"] and results(tmp_path)["a"]["status"] == "aborted" and summary["exit_code"] == 1


def test_a_live_run_that_never_ticks_steps_the_bundle_back(tmp_path):
    runs = [ib.Run(name=n, argv=py("import time; time.sleep(1.5)"), pace_log=str(tmp_path / f"{n}.jsonl"))
            for n in ("a", "b")] + [stamp_run("c", tmp_path, 0.05)]
    summary = bundle(tmp_path, runs, slots=2, pace_floor=0.1, pace_window_s=0.5, pace_warmup_s=0.2,
                     pace_period_s=0.1, pace_stall_s=0.3)
    assert summary["backoffs"] and "no tick" in summary["backoffs"][0]["reason"]


def test_an_unidentified_gpu_holds_the_first_start(tmp_path):
    summary = bundle(tmp_path, [stamp_run("a", tmp_path, 0.05)], slots=1, shards=1,
                     vram_probe=lambda leaders: ([], {0: 1000, 1: 1000}), vram_period_s=0.05, vram_unreadable_s=0.3)
    s = stamps(tmp_path, ["a"])
    assert s["a"]["t0"] - summary["start"] >= 0.25 and summary["unreadable_starts"] == 1


def test_the_slurm_gpu_index_picks_the_device(tmp_path):
    frees = iter([{0: 16000, 1: 1000}] * 3 + [{0: 16000, 1: 16000}] * 1000)
    summary = bundle(tmp_path, [stamp_run("a", tmp_path, 0.05)], slots=1, shards=1, gpu_index=1,
                     vram_probe=lambda leaders: ([], next(frees)), vram_period_s=0.05)
    assert summary["gpu_full_waits"] == 1 and summary["failed"] == 0


@pytest.mark.parametrize(
    "env, index",
    [
        ({"CUDA_VISIBLE_DEVICES": "0", "SLURM_JOB_GPUS": "1"}, 1),  # ConstrainDevices renumbers CUDA's view to 0
        ({"CUDA_VISIBLE_DEVICES": "0"}, None),
        ({"SLURM_JOB_GPUS": "0,1"}, None),
        ({}, None),
    ],
)
def test_the_job_gpu_comes_from_slurm_not_from_cuda(env, index):
    assert ib.job_gpu_index(env) == index


# --- sizes and the node's memory ---------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "kind, slots, args",
    [
        ("live", 4, {"slots": 4, "gres": "shard:4", "cpus": 8, "mem_gib": 24}),  # 4 x 5.5 + 1.5 + 0.5 = 24
        ("riib", 3, {"slots": 3, "gres": "shard:4", "cpus": 6, "mem_gib": 20}),  # 3 x 4,500 MiB > 3 shards
        ("riib", None, {"slots": 3, "gres": "shard:4", "cpus": 6, "mem_gib": 20}),
        ("riib", 1, {"slots": 1, "gres": "shard:2", "cpus": 2, "mem_gib": 8}),  # one riib run is over a shard
        ("bd", 2, {"slots": 2, "gres": "shard:2", "cpus": 4, "mem_gib": 12}),
    ],
)
def test_sizes_grow_with_the_bundle(kind, slots, args):
    assert ib.sbatch_sizes(kind, slots) == args


def test_more_sessions_than_a_gpu_holds_are_refused():
    with pytest.raises(ValueError, match="riib"):
        ib.sbatch_sizes("riib", 4)


def test_the_sizes_command_prints_sbatch_options(capsys):
    assert ib.main(["sizes", "live"]) == 0
    assert capsys.readouterr().out.strip() == "--gres=shard:4 --cpus-per-task=8 --mem=24G"


def test_a_start_waits_for_the_node_s_memory(tmp_path):
    avail = iter([3000, 3000, 3000] + [20000] * 1000)
    summary = bundle(tmp_path, [stamp_run("a", tmp_path, 0.05)], slots=1, node_ram_min_mib=6344,
                     node_ram=lambda: next(avail))
    assert summary["node_ram_waits"] == 1 and summary["failed"] == 0


def test_bundles_sharing_a_claim_dir_run_each_run_once(tmp_path):
    names = [f"r{i}" for i in range(6)]
    claim = tmp_path / "claims"
    outs = {}

    def one(tag):
        runs = [stamp_run(n, tmp_path, 0.2) for n in names]
        outs[tag] = ib.run_bundle(runs, tmp_path / f"out_{tag}", slots=2, stagger_s=0.0, grace_s=1.0, poll_s=0.02,
                                  job_id=tag, cpus=[0], cleanup=lambda sid: [], leftover_grace_s=1.0,
                                  oom_counter=None, claim_dir=claim)

    threads = [threading.Thread(target=one, args=(t,)) for t in ("j1", "j2")]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    ran = [json.loads(x)["name"] for t in ("j1", "j2")
           for x in (tmp_path / f"out_{t}" / "results.jsonl").read_text().splitlines()]
    assert sorted(ran) == names  # every run once, across the two jobs
    assert outs["j1"]["runs"] + outs["j2"]["runs"] == 6
    assert outs["j1"]["exit_code"] == 0 and outs["j2"]["exit_code"] == 0
    assert {(claim / n / "by").read_text().strip() for n in names} <= {"j1", "j2"}


# --- Codex round 6 counterexamples -------------------------------------------------------------------------------------


def test_the_node_ledger_does_not_admit_the_same_free_memory_twice(tmp_path):
    a = ib.NodeLedger(tmp_path / "ledger")
    b = ib.NodeLedger(tmp_path / "ledger")  # another job on the node
    assert a.admit("j1-0", 6344, lambda: 7000) is True
    assert b.admit("j2-0", 6344, lambda: 7000) is False  # 7,000 - 6,344 reserved < 6,344
    assert b.admit("j2-0", 6344, lambda: 14000) is True


def test_a_reservation_expires(tmp_path):
    led = ib.NodeLedger(tmp_path / "ledger", hold_s=0.1)
    assert led.admit("x", 6000, lambda: 7000)
    time.sleep(0.15)
    assert led.admit("y", 6000, lambda: 7000)


def test_a_claimed_run_that_cannot_start_is_recorded(tmp_path):
    claim = tmp_path / "claims"
    bad = ib.Run(name="a", argv=[str(tmp_path / "no_such_program")])
    summary = bundle(tmp_path, [bad], claim_dir=claim)
    r = results(tmp_path)["a"]
    assert r["status"] == "start_failed" and summary["not_started"] == 1 and summary["exit_code"] == 1


def test_a_stop_does_not_count_runs_another_job_claimed(tmp_path):
    claim = tmp_path / "claims"
    (claim / "b").mkdir(parents=True)
    (claim / "b" / "by").write_text("other\n")
    stop = threading.Event()
    threading.Timer(0.2, stop.set).start()
    summary = bundle(tmp_path, [stamp_run("a", tmp_path, 2.0), stamp_run("b", tmp_path, 0.05),
                                stamp_run("c", tmp_path, 0.05)], slots=1, claim_dir=claim, stop=stop)
    r = results(tmp_path)
    assert "b" not in r and r["c"]["status"] == "not_started"
    assert summary["claimed_elsewhere"] == 1 and summary["not_started"] == 1


def test_runs_claimed_elsewhere_are_skipped_before_any_wait(tmp_path):
    claim = tmp_path / "claims"
    for n in ("a", "b"):
        (claim / n).mkdir(parents=True)
        (claim / n / "by").write_text("other\n")
    t0 = time.monotonic()
    summary = bundle(tmp_path, [stamp_run("a", tmp_path, 0.05), stamp_run("b", tmp_path, 0.05)], slots=1,
                     claim_dir=claim, node_ram_min_mib=6344, node_ram=lambda: 100)  # the node is full
    assert summary["claimed_elsewhere"] == 2 and summary["runs"] == 0 and summary["exit_code"] == 0
    assert time.monotonic() - t0 < 2


def test_a_reservation_is_given_back_when_nothing_starts(tmp_path):
    led = tmp_path / "ledger"
    claim = tmp_path / "claims"
    (claim / "a").mkdir(parents=True)
    (claim / "a" / "by").write_text("77\n")  # our own claim from before: not skipped, but mkdir loses
    bundle(tmp_path, [ib.Run(name="a", argv=[str(tmp_path / "no_such_program")])], claim_dir=claim,
           node_ram_min_mib=6344, node_ram=lambda: 7000, node_ledger_dir=led)
    assert list(led.glob("r-*")) == []
    assert ib.NodeLedger(led).admit("next", 6344, lambda: 7000)


def test_a_failed_start_gives_its_reservation_back(tmp_path):
    led = tmp_path / "ledger"
    bundle(tmp_path, [ib.Run(name="a", argv=[str(tmp_path / "no_such_program")])], node_ram_min_mib=6344,
           node_ram=lambda: 7000, node_ledger_dir=led)
    assert results(tmp_path)["a"]["status"] == "start_failed" and list(led.glob("r-*")) == []


# --- a core for the SLAM bridges --------------------------------------------------------------------------------------

WS1_SIBLINGS = {0: (0, 12), 12: (0, 12), 1: (1, 13), 13: (1, 13), 4: (4, 16), 16: (4, 16), 5: (5, 17), 17: (5, 17),
                6: (6, 18), 18: (6, 18)}


def test_a_whole_core_is_set_apart_for_the_bridges():
    bridge, rest = ib.split_bridge_cpus([0, 1, 4, 5, 6, 12, 13, 16, 17, 18], 2, lambda c: WS1_SIBLINGS[c])
    assert bridge == [6, 18] and rest == [0, 1, 4, 5, 12, 13, 16, 17]


def test_no_core_is_set_apart_when_nothing_would_be_left():
    assert ib.split_bridge_cpus([0, 12], 2, lambda c: WS1_SIBLINGS[c]) == ([], [0, 12])
    assert ib.split_bridge_cpus([0, 1, 12, 13], 0, lambda c: WS1_SIBLINGS[c]) == ([], [0, 1, 12, 13])


def test_sessions_run_on_the_cpus_the_bridges_do_not_have(tmp_path):
    cpus = sorted(os.sched_getaffinity(0))
    if len(cpus) < 3:
        pytest.skip("needs 3 CPUs")
    code = "import json,os,sys; json.dump(sorted(os.sched_getaffinity(0)), open(sys.argv[1],'w'))"
    run = ib.Run(name="a", argv=py(code, str(tmp_path / "aff.json")), env={"MARK": "x"})
    summary = bundle(tmp_path, [run], slots=1, cpus=cpus, bridge_cpus=1, siblings=lambda c: (c,))
    aff = json.loads((tmp_path / "aff.json").read_text())
    assert summary["bridge_cpus"] == [cpus[-1]] and aff == cpus[:-1]
    env_bridge = results(tmp_path)["a"]["bridge_cpu"]
    assert env_bridge == str(cpus[-1])


@pytest.mark.parametrize("n_cpus, slots, n", [(10, 4, 2), (8, 4, 0), (8, 3, 2), (4, 1, 2), (2, 1, 0)])
def test_auto_sets_a_core_apart_only_when_the_job_has_one_to_spare(n_cpus, slots, n):
    assert ib.auto_bridge_cpus(n_cpus, slots) == n


def test_a_run_that_ends_at_once_gives_its_reservation_back(tmp_path):
    # wrapped in taskset the start succeeds even when the program is missing; the run's end returns the memory
    led = tmp_path / "ledger"
    bundle(tmp_path, [stamp_run("a", tmp_path, 0.05, code=127)], node_ram_min_mib=6904, node_ram=lambda: 7000,
           node_ledger_dir=led)
    assert results(tmp_path)["a"]["rc"] == 127 and list(led.glob("r-*")) == []


def test_a_reservation_that_cannot_be_given_back_leaves_the_run_recorded(tmp_path, monkeypatch):
    seen = []

    def broken(self, key):
        raise OSError("ledger gone")

    monkeypatch.setattr(ib.NodeLedger, "release", broken)
    summary = bundle(tmp_path, [stamp_run("a", tmp_path, 0.05)], node_ram_min_mib=100, node_ram=lambda: 7000,
                     node_ledger_dir=tmp_path / "ledger", cleanup=lambda sid: seen.append(sid) or [])
    assert results(tmp_path)["a"]["status"] == "ended" and seen == ["77s0"]
    assert summary["errors"] and summary["exit_code"] == 1


def test_the_runner_takes_its_slots_and_session_memory_from_the_kind(tmp_path, monkeypatch):
    seen = {}

    def fake(runs, out, **k):
        seen.update(k)
        return {"exit_code": 0}

    monkeypatch.setattr(ib, "run_bundle", fake)
    monkeypatch.setenv("SLURM_SHARDS_ON_NODE", "4")
    monkeypatch.delenv("CUDA_VISIBLE_DEVICES", raising=False)
    monkeypatch.delenv("SLURM_JOB_GPUS", raising=False)
    path = tmp_path / "runs.jsonl"
    path.write_text(json.dumps({"name": "a", "argv": ["true"]}) + "\n")
    assert ib.main(["--list", str(path), "--out", str(tmp_path / "o"), "--no-vram", "--kind", "riib"]) == 0
    assert seen["slots"] == 3 and seen["shards"] == 4 and seen["session_vram_mib"] == 4500


@pytest.mark.parametrize("kind, shards, slots", [("riib", 4, 3), ("riib", 2, 1), ("riib", 1, 0), ("live", 4, 4),
                                                ("bd", 4, 4)])
def test_the_kind_s_slots_follow_the_shards(kind, shards, slots):
    assert ib.kind_slots(kind, shards) == slots


@pytest.mark.parametrize("extra", [["--kind", "riib"], ["--kind", "riib", "--slots", "1"], ["--slots", "1",
                                                                                        "--session-vram-mib", "4500"]])
def test_sessions_that_do_not_fit_the_shards_are_refused(tmp_path, monkeypatch, capsys, extra):
    monkeypatch.setenv("SLURM_SHARDS_ON_NODE", "1")
    monkeypatch.delenv("CUDA_VISIBLE_DEVICES", raising=False)
    monkeypatch.delenv("SLURM_JOB_GPUS", raising=False)
    path = tmp_path / "runs.jsonl"
    path.write_text(json.dumps({"name": "a", "argv": ["true"]}) + "\n")
    assert ib.main(["--list", str(path), "--out", str(tmp_path / "o"), "--no-vram", *extra]) == 2
    assert "refusing" in capsys.readouterr().err


# --- the check before a submission ------------------------------------------------------------------------------------

RUNNER_SRC = 'ap.add_argument("--seed")\nap.add_argument("--scene-file")\n'
LAUNCHER = ('mkdir -p "$1"; echo touched > "$1/defaults.txt"\n'
            'if [ "$DRY_RUN" = 1 ]; then echo RUNNER_ARGS; printf "%s\\n" --seed "$SEED" $EXTRA; exit 0; fi; exit 9\n')


def check_case(tmp_path, extra, out_exists=False):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "launch.sh").write_text(LAUNCHER)
    out = tmp_path / "out_a"
    if out_exists:
        out.mkdir()
    run = ib.Run(name="a", argv=["bash", str(tmp_path / "launch.sh"), str(out)], env={"SEED": "1", "EXTRA": extra},
                 retry_move=[str(out)])
    return ib.check_runs([run], tmp_path / "runner.py")


def test_a_list_whose_flags_the_runner_defines_passes(tmp_path):
    assert check_case(tmp_path, "--scene-file x") == []


def test_a_flag_the_snapshot_lacks_is_caught(tmp_path):
    (problem,) = check_case(tmp_path, "--queue-resume-not-stopped")
    assert "--queue-resume-not-stopped" in problem


def test_an_existing_output_is_caught_and_left_alone(tmp_path):
    assert any("exists" in p for p in check_case(tmp_path, "", out_exists=True))
    assert not (tmp_path / "out_a" / "defaults.txt").exists()  # the launcher was not run on it


def test_the_dry_run_writes_to_a_scratch_output(tmp_path):
    assert check_case(tmp_path, "") == []
    assert not (tmp_path / "out_a").exists()


def test_a_launcher_without_a_dry_run_mode_is_not_run(tmp_path):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "live.sbatch").write_text('touch "$0.ran"\n')
    run = ib.Run(name="a", argv=["bash", str(tmp_path / "live.sbatch"), str(tmp_path / "o")],
                 retry_move=[str(tmp_path / "o")])
    (problem,) = ib.check_runs([run], tmp_path / "runner.py")
    assert "DRY_RUN mode" in problem and not (tmp_path / "live.sbatch.ran").exists()


def test_a_marker_inside_another_word_does_not_count(tmp_path):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "l.sh").write_text('# DRY_RUN RUNNER_ARGS\necho NO_RUNNER_ARGS\n')
    run = ib.Run(name="a", argv=["bash", str(tmp_path / "l.sh"), str(tmp_path / "o")], retry_move=[str(tmp_path / "o")])
    (problem,) = ib.check_runs([run], tmp_path / "runner.py")
    assert "no RUNNER_ARGS line" in problem


def test_a_dry_run_that_hangs_is_killed(tmp_path):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    # the child sits in a process group of its own (as GNU timeout makes one): the token still finds it
    (tmp_path / "h.sh").write_text('# DRY_RUN RUNNER_ARGS\nsetsid sleep 30 & echo $! > "$0.pid"; wait\n')
    t0 = time.monotonic()
    run = ib.Run(name="a", argv=["bash", str(tmp_path / "h.sh"), str(tmp_path / "o")], retry_move=[str(tmp_path / "o")])
    (problem,) = ib.check_runs([run], tmp_path / "runner.py", timeout_s=0.3)
    assert "killed" in problem and time.monotonic() - t0 < 5
    pid = int((tmp_path / "h.sh.pid").read_text())
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and ib.alive(pid):
        time.sleep(0.05)
    assert not ib.alive(pid)


def test_a_launcher_without_dry_run_output_is_caught(tmp_path):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    run = ib.Run(name="a", argv=["bash", "-c", "exit 0"], retry_move=[str(tmp_path / "o")])
    (problem,) = ib.check_runs([run], tmp_path / "runner.py")
    assert "DRY_RUN mode" in problem


def test_the_check_command_exits_1_on_a_problem(tmp_path, capsys):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "launch.sh").write_text(LAUNCHER)
    path = tmp_path / "runs.jsonl"
    path.write_text(json.dumps({"name": "a", "argv": ["bash", str(tmp_path / "launch.sh"), str(tmp_path / "o")],
                                "env": {"SEED": "1", "EXTRA": "--nope"}, "retry_move": [str(tmp_path / "o")]}) + "\n")
    assert ib.main(["check", "--list", str(path), "--runner", str(tmp_path / "runner.py")]) == 1
    assert "--nope" in capsys.readouterr().out



def test_a_relative_output_is_still_sent_to_scratch(tmp_path):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "launch.sh").write_text(LAUNCHER)
    run = ib.Run(name="a", argv=["bash", "launch.sh", "out_a"], cwd=str(tmp_path), env={"SEED": "1", "EXTRA": ""},
                 retry_move=[str(tmp_path / "out_a")])
    assert ib.check_runs([run], tmp_path / "runner.py") == []
    assert not (tmp_path / "out_a").exists()


def test_a_run_without_its_output_in_argv_is_not_run(tmp_path):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "launch.sh").write_text(LAUNCHER)
    run = ib.Run(name="a", argv=["bash", str(tmp_path / "launch.sh"), str(tmp_path / "elsewhere")],
                 env={"SEED": "1", "EXTRA": ""}, retry_move=[str(tmp_path / "out_a")])
    (problem,) = ib.check_runs([run], tmp_path / "runner.py")
    assert "not each a whole argument" in problem and not (tmp_path / "elsewhere").exists()


def test_a_run_without_an_output_is_not_run(tmp_path):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "launch.sh").write_text(LAUNCHER)
    run = ib.Run(name="a", argv=["bash", str(tmp_path / "launch.sh"), str(tmp_path / "x")], env={"SEED": "1"})
    (problem,) = ib.check_runs([run], tmp_path / "runner.py")
    assert "no retry_move" in problem and not (tmp_path / "x").exists()



def test_every_output_must_be_sent_to_scratch(tmp_path):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "launch.sh").write_text(LAUNCHER)
    a, b = str(tmp_path / "a"), str(tmp_path / "b")
    run = ib.Run(name="x", argv=["bash", str(tmp_path / "launch.sh"), a, f"--aux={b}"], env={"SEED": "1"},
                 retry_move=[a, b])
    (problem,) = ib.check_runs([run], tmp_path / "runner.py")
    assert "not each a whole argument" in problem and not (tmp_path / "a").exists()


def test_an_output_named_in_the_env_is_refused(tmp_path):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "launch.sh").write_text(LAUNCHER)
    a = str(tmp_path / "a")
    run = ib.Run(name="x", argv=["bash", str(tmp_path / "launch.sh"), a], env={"SEED": "1", "OUT2": a},
                 retry_move=[a])
    (problem,) = ib.check_runs([run], tmp_path / "runner.py")
    assert "elsewhere" in problem


def test_a_child_forked_at_the_end_is_ended(tmp_path):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    # the launcher ends at once, leaving a child that would fork a grandchild in its own session a moment later
    script = ('# DRY_RUN RUNNER_ARGS\necho RUNNER_ARGS\n'
              '(echo $BASHPID > "$0.pid"; sleep 0.3; setsid sleep 30 & echo $! > "$0.grand"; wait) &\nexit 0\n')
    (tmp_path / "f.sh").write_text(script)
    run = ib.Run(name="x", argv=["bash", str(tmp_path / "f.sh"), str(tmp_path / "o")], retry_move=[str(tmp_path / "o")])
    assert ib.check_runs([run], tmp_path / "runner.py") == []
    pids = [int((tmp_path / f"f.sh.{x}").read_text()) for x in ("pid", "grand") if (tmp_path / f"f.sh.{x}").exists()]
    assert pids and not any(ib.alive(p) for p in pids)  # the child is gone, and so is a grandchild it managed



def test_an_output_in_the_inherited_environment_is_refused(tmp_path, monkeypatch):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "launch.sh").write_text(LAUNCHER)
    a = str(tmp_path / "a")
    monkeypatch.setenv("OUT2", a)
    run = ib.Run(name="x", argv=["bash", str(tmp_path / "launch.sh"), a], env={"SEED": "1"}, retry_move=[a])
    (problem,) = ib.check_runs([run], tmp_path / "runner.py")
    assert "elsewhere" in problem


def test_processes_that_survive_stop_the_check(tmp_path, monkeypatch):
    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "launch.sh").write_text(LAUNCHER)
    monkeypatch.setattr(ib, "_end_token", lambda token: False)
    runs = [ib.Run(name=n, argv=["bash", str(tmp_path / "launch.sh"), str(tmp_path / n)], env={"SEED": "1"},
                   retry_move=[str(tmp_path / n)]) for n in ("a", "b")]
    problems = ib.check_runs(runs, tmp_path / "runner.py")
    assert len(problems) == 1 and "survive" in problems[0]  # b was not run
