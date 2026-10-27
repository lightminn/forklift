"""A session's mission results in the bundle's verdict: a run that ended rc 0 after its missions failed is not a
success (lvram pilot, 2026-10-27: random_m sessions asked P-1, which the scene blocks, failed all three missions in
two seconds and ended rc 0 in under a minute)."""

import json
import sys

from tools import isaac_bundle as ib

PY = sys.executable


def report(*outcomes, refused=False):
    """A console probe report (tools/live_console_probe.py --report): one missions_run entry per outcome."""
    entries = [{"pallet_id": "P-1", "destination_id": f"D-{i}", "submit": {"t": "ack", "mission_id": f"m{i}"},
                "result": {"outcome": o, "reason": "prepare:all_candidates_failed:invalid_goal" if o != "done" else None}}
               for i, o in enumerate(outcomes, 1)]
    if refused:
        entries.insert(0, {"pallet_id": "P-9", "destination_id": "D-1", "submit": {"t": "nack", "reason": "unknown_pallet"}})
    return {"missions_run": entries, "session_end": {}}


def writer(path, data):
    """A run that writes ``data`` as JSON to ``path`` and exits 0."""
    code = "import json,sys,os; os.makedirs(os.path.dirname(sys.argv[1]),exist_ok=True); json.dump(json.loads(sys.argv[2]),open(sys.argv[1],'w'))"
    return [PY, "-c", code, str(path), json.dumps(data)]


def bundle(tmp_path, runs):
    return ib.run_bundle(runs, tmp_path / "out", slots=2, stagger_s=0.0, grace_s=1.0, poll_s=0.02, job_id="77",
                         cpus=[4, 5, 16, 17], cleanup=lambda session_id: [], leftover_grace_s=1.0, oom_counter=None)


def results(tmp_path):
    return {r["name"]: r for r in map(json.loads, (tmp_path / "out" / "results.jsonl").read_text().splitlines())}


def test_a_session_whose_missions_all_failed_is_an_early_mission_failure(tmp_path):
    out = tmp_path / "run_a"
    run = ib.Run(name="a", argv=writer(out / "probe.json", report("failed", "failed", "failed")), retry_move=[str(out)])
    summary = bundle(tmp_path, [run])
    row = results(tmp_path)["a"]
    assert row["rc"] == 0 and row["status"] == "early_mission_failure"
    assert row["missions"]["submitted"] == 3 and row["missions"]["done"] == 0
    assert row["missions"]["first"]["outcome"] == "failed" and "invalid_goal" in row["missions"]["first"]["reason"]
    assert row["missions"]["report"] == str(out / "probe.json")
    assert summary["failed"] == 1 and summary["early_mission_failures"] == 1 and summary["exit_code"] == 1
    assert not row["retried"]  # the list's problem: the same session would fail again


def test_a_session_with_a_done_mission_has_ended(tmp_path):
    # a later mission's failure is the experiment's result, not the run's: the row keeps ended and counts it
    out = tmp_path / "run_b"
    run = ib.Run(name="b", argv=writer(out / "probe.json", report("done", "failed")), retry_move=[str(out)])
    summary = bundle(tmp_path, [run])
    row = results(tmp_path)["b"]
    assert row["status"] == "ended" and row["missions"]["submitted"] == 2 and row["missions"]["done"] == 1
    assert row["missions"]["outcomes"] == {"done": 1, "failed": 1}
    assert summary["failed"] == 0 and summary["missions_submitted"] == 2 and summary["missions_done"] == 1


def test_a_refused_submission_counts_as_a_mission_that_was_not_done(tmp_path):
    out = tmp_path / "run_c"
    run = ib.Run(name="c", argv=writer(out / "probe.json", report(refused=True)), retry_move=[str(out)])
    bundle(tmp_path, [run])
    row = results(tmp_path)["c"]
    assert row["status"] == "early_mission_failure"
    assert row["missions"]["first"] == {"outcome": "refused", "reason": "unknown_pallet"}


def test_a_named_report_that_is_missing_fails_the_run(tmp_path):
    run = ib.Run(name="d", argv=[PY, "-c", "pass"], missions=str(tmp_path / "nowhere" / "probe.json"))
    bundle(tmp_path, [run])
    row = results(tmp_path)["d"]
    assert row["rc"] == 0 and row["status"] == "no_mission_report"


def test_a_named_report_is_read_where_the_list_says(tmp_path):
    path = tmp_path / "elsewhere" / "report_{attempt}.json"
    run = ib.Run(name="e", argv=writer(str(path).replace("{attempt}", "1"), report("failed")), missions=str(path))
    bundle(tmp_path, [run])
    assert results(tmp_path)["e"]["status"] == "early_mission_failure"


def test_a_run_without_a_report_is_judged_by_its_rc_as_before(tmp_path):
    # batch runs (riib scenes, gates) write no probe report: nothing changes for them
    run = ib.Run(name="f", argv=[PY, "-c", "pass"], retry_move=[str(tmp_path / "run_f")])
    summary = bundle(tmp_path, [run])
    row = results(tmp_path)["f"]
    assert row["status"] == "ended" and row["missions"] is None and summary["exit_code"] == 0


def test_a_session_that_submitted_nothing_has_ended(tmp_path):
    # a teleop-only session (an empty cycle) asks no mission: no mission verdict
    out = tmp_path / "run_g"
    run = ib.Run(name="g", argv=writer(out / "probe.json", {"missions_run": []}), retry_move=[str(out)])
    bundle(tmp_path, [run])
    row = results(tmp_path)["g"]
    assert row["status"] == "ended" and row["missions"]["submitted"] == 0


def test_an_unreadable_report_fails_the_run(tmp_path):
    out = tmp_path / "run_h"
    code = f"import os; os.makedirs({str(out)!r}); open({str(out / 'probe.json')!r},'w').write('{{not json')"
    run = ib.Run(name="h", argv=[PY, "-c", code], retry_move=[str(out)])
    bundle(tmp_path, [run])
    assert results(tmp_path)["h"]["status"] == "no_mission_report"


def tour(end, progress_m=27.1, reason="pallet:P-1"):
    events = [{"event": "departed", "progress_m": 0.0},
              {"event": "stalled", "progress_m": progress_m, "permit": {"limit_mps": 0.0, "reason": reason}}]
    return {"tour": {"path": "tour.npz", "samples": 542, "end": end, "progress_m": progress_m, "events": events},
            "session_end": {}}


def test_a_tour_that_stopped_short_is_incomplete_with_its_reason(tmp_path):
    # d7oor 11832 (2026-10-27): both tours stalled at 27.1 m on pallet:P-1's permit, rc 0
    out = tmp_path / "run_t"
    run = ib.Run(name="t", argv=writer(out / "probe.json", tour("stalled")), retry_move=[str(out)])
    summary = bundle(tmp_path, [run])
    row = results(tmp_path)["t"]
    assert row["status"] == "tour_incomplete"
    assert row["missions"]["tour"] == {"end": "stalled", "progress_m": 27.1, "permit_reason": "pallet:P-1"}
    assert summary["failed"] == 1 and summary["tours_incomplete"] == 1 and summary["exit_code"] == 1


def test_a_tour_that_arrived_has_ended(tmp_path):
    out = tmp_path / "run_u"
    run = ib.Run(name="u", argv=writer(out / "probe.json", tour("arrived", 30.8, None)), retry_move=[str(out)])
    bundle(tmp_path, [run])
    row = results(tmp_path)["u"]
    assert row["status"] == "ended" and row["missions"]["tour"]["end"] == "arrived"


def test_the_summary_counts_each_status_in_one_line(tmp_path, capsys):
    runs = []
    for name, data in (("a", report("done")), ("b", report("done")), ("c", report("failed")), ("d", tour("stalled"))):
        out = tmp_path / f"run_{name}"
        runs.append(ib.Run(name=name, argv=writer(out / "probe.json", data), retry_move=[str(out)]))
    runs.append(ib.Run(name="e", argv=[PY, "-c", "raise SystemExit(3)"]))
    summary = bundle(tmp_path, runs)
    assert summary["statuses"] == {"ended": 2, "early_mission_failure": 1, "tour_incomplete": 1, "ended_rc_nonzero": 1}
    assert summary["status_line"] == "ended 2, early_mission_failure 1, ended_rc_nonzero 1, tour_incomplete 1"


def test_the_next_session_s_missions_after_a_reset_count_too(tmp_path):
    # Codex: the probe keeps a reset's next session under after_reset.missions_run
    out = tmp_path / "run_r"
    data = {"missions_run": [], "after_reset": report("failed", "failed", "failed")}
    run = ib.Run(name="r", argv=writer(out / "probe.json", data), retry_move=[str(out)])
    bundle(tmp_path, [run])
    row = results(tmp_path)["r"]
    assert row["status"] == "early_mission_failure" and row["missions"]["submitted"] == 3


def test_a_report_of_another_shape_stays_with_its_run(tmp_path):
    # Codex: {"missions_run": 1} raised inside settle and stopped the bundle; now that run alone is no_mission_report
    runs = []
    for name, data in (("m", {"missions_run": 1}), ("n", {"missions_run": [{"submit": [], "result": 3}]}),
                       ("o", {"tour": {"end": "arrived", "events": 7}}), ("p", report("done")),
                       ("q", {"missions_run": {}})):  # Codex: an empty value of another type is not "none asked"
        out = tmp_path / f"run_{name}"
        runs.append(ib.Run(name=name, argv=writer(out / "probe.json", data), retry_move=[str(out)]))
    summary = bundle(tmp_path, runs)
    rows = results(tmp_path)
    assert {n: rows[n]["status"] for n in "mnpq"} == {"m": "no_mission_report", "n": "no_mission_report",
                                                       "p": "ended", "q": "no_mission_report"}
    assert rows["o"]["status"] == "ended" and rows["o"]["missions"]["tour"]["end"] == "arrived"
    assert summary["not_started"] == 0 and summary["no_mission_reports"] == 3


def test_a_report_older_than_the_run_is_not_its_own(tmp_path):
    # Codex: a done report left in the folder by an earlier run would have counted for a run that wrote none
    out = tmp_path / "run_s"
    out.mkdir()
    (out / "probe.json").write_text(json.dumps(report("done")))
    old = __import__("time").time() - 3600
    __import__("os").utime(out / "probe.json", (old, old))
    run = ib.Run(name="s", argv=[PY, "-c", "pass"], retry_move=[str(out)])
    bundle(tmp_path, [run])
    row = results(tmp_path)["s"]
    assert row["status"] == "no_mission_report" and "stale" in row["missions"]["error"]


def test_the_list_takes_a_missions_path(tmp_path):
    path = tmp_path / "runs.jsonl"
    path.write_text(json.dumps({"name": "a", "argv": ["true"], "missions": "out/{attempt}/probe.json"}) + "\n")
    (run,) = ib.load_runs(path)
    assert run.missions == "out/{attempt}/probe.json" and run.at(2).missions == "out/2/probe.json"
    path.write_text(json.dumps({"name": "a", "argv": ["true"], "missions": 3}) + "\n")
    try:
        ib.load_runs(path)
    except ValueError as exc:
        assert "missions" in str(exc)
    else:
        raise AssertionError("a non-string missions path was accepted")


def test_collect_counts_an_early_mission_failure_as_bad(tmp_path, capsys):
    d = tmp_path / "task_0" / "bundle"
    d.mkdir(parents=True)
    row = {"name": "r0", "status": "early_mission_failure", "rc": 0, "missions": {"submitted": 3, "done": 0}}
    (d / "results.jsonl").write_text(json.dumps(row) + "\n")
    (d / "summary.json").write_text(json.dumps({"exit_code": 1}))
    rows, _ = ib.collect(tmp_path, 1)
    assert rows[0]["status"] == "early_mission_failure" and rows[0]["missions"] == {"submitted": 3, "done": 0}
    assert ib.main(["collect", str(tmp_path), "--expect", "2"]) == 1
    assert "statuses: early_mission_failure 1, missing 1" in capsys.readouterr().out
