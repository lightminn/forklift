"""tools/teleop_realtime_report.py on a synthetic session: per-mode ratio and pacing, a hold whose output waits for
pacing, the lift change, timers per call, and the overlap rule."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import teleop_realtime_report as tr  # noqa: E402

DT = 1 / 120


def make_session(root: Path, name: str, *, other_cores: float = 0.0) -> Path:
    """AUTO for 2 s at ratio 1.0, then TELEOP for 2 s at ratio 0.9 with pacing not ok for its first 0.5 s and a
    driving hold from 0.2 s that outputs once pacing is ok, then a lift hold."""
    sd = root / name / "s"
    run = sd / "live" / "epoch_1" / "run"
    run.mkdir(parents=True)
    recs, mono, t = [], 100.0, 0.0
    for _ in range(240):
        recs.append({"tick_t": t, "mode": "AUTO", "pacing_ok": True, "mono": mono, "speed_mps": 0.3,
                     "armed": False, "deadline": None})
        t += DT
        mono += DT
    tele0 = mono
    for k in range(216):
        ok = mono - tele0 >= 0.5
        drive = ok and 0.2 <= mono - tele0 < 1.2
        recs.append({"tick_t": t, "mode": "TELEOP", "pacing_ok": ok, "mono": mono, "speed_mps": 0.1 if drive else 0.0,
                     "armed": True, "deadline": None})
        if k % 108 == 107:
            recs.append({"status": True, "tick_t": t, "mono": mono,
                         "timers": {"teleop.permit": [0.06, 60, 0.01, 0.0]}})
        t += DT
        mono += DT / 0.9
    recs.append({"status": True, "tick_t": recs[200]["tick_t"], "mono": recs[200]["mono"] + 1e-6,
                 "timers": {"core.permission": [0.05, 100, 0.002, 0.0]}})
    with open(run / "live_log.jsonl", "w") as fh:
        for r in sorted(recs, key=lambda r: r["mono"]):
            fh.write(json.dumps(r) + "\n")
    probe = {"states": [{"mono": tele0 + 0.3, "teleop_forbidden": "pacing"}],
             "confirm": {"holds": [
                 {"label": "forward", "mode": "direct", "lift_dir": 0, "start_mono": tele0 + 0.2,
                  "end_mono": tele0 + 1.2, "start_state": {"lift_m": 0.0}, "end_state": {"lift_m": 0.0}},
                 {"label": "lift_up", "mode": "direct", "lift_dir": 1, "start_mono": tele0 + 1.3,
                  "end_mono": tele0 + 1.9, "start_state": {"lift_m": 0.10}, "end_state": {"lift_m": 0.13}}]}}
    (sd / "probe.json").write_text(json.dumps(probe))
    with open(root / f"{name}_ps.jsonl", "w") as fh:
        fh.write(json.dumps({"sampler": "x"}) + "\n")
        for _ in range(10):
            fh.write(json.dumps({"other_cores": other_cores}) + "\n")
    return sd


def test_modes_holds_and_timers(tmp_path):
    s = tr.session(make_session(tmp_path, "lat_x"))
    assert s["modes"]["AUTO"]["ratio"] == pytest.approx(1.0, abs=1e-6)
    assert s["modes"]["TELEOP"]["ratio"] == pytest.approx(0.9, abs=1e-6)
    assert s["modes"]["TELEOP"]["longest_not_ok_s"] == pytest.approx(0.5, abs=0.02)
    assert s["modes"]["TELEOP"]["ok_wall"] == pytest.approx(1.5 / 2.0, abs=0.02)
    fwd = next(h for h in s["holds"] if h["label"] == "forward")
    assert fwd["ok_at_start"] is False and fwd["first_output_ms"] == pytest.approx(fwd["first_ok_ms"])
    assert fwd["first_output_ms"] == pytest.approx(300, abs=10) and fwd["reasons_before_output"] == ["pacing"]
    lift = next(h for h in s["holds"] if h["label"] == "lift_up")
    assert lift["lift_delta_m"] == pytest.approx(0.03) and lift["output_share"] == 0.0
    t = s["timers"]["TELEOP"]["teleop.permit"]
    assert t["mean_ms"] == pytest.approx(1.0) and t["max_ms"] == pytest.approx(10.0)
    assert s["overlap"] == 0.0


def test_the_summary_leaves_out_overlapped_sessions(tmp_path):
    a = tr.session(make_session(tmp_path, "lat_a"))
    b = tr.session(make_session(tmp_path, "lat_b", other_cores=2.0))
    out = tr.summarize([a, b])
    assert out["kept"] == ["lat_a"] and out["left_out"] == {"lat_b": 1.0}
    assert out["holds"]["forward"]["started_not_ok"] == 1 and out["holds"]["forward"]["output_at_first_ok"] == 1
    assert out["modes"]["TELEOP"]["ratio"]["median"] == pytest.approx(0.9, abs=1e-6)


def test_the_cli_writes_json(tmp_path):
    sd = make_session(tmp_path, "lat_c")
    out = tmp_path / "o.json"
    assert tr.main([str(sd), "--json", str(out)]) == 0
    assert json.loads(out.read_text())["summary"]["kept"] == ["lat_c"]
    with pytest.raises(SystemExit):
        tr.main([str(sd), "--verdict", "--factor", "0.9"])  # the registered slots are required


# --- the smoothness confirmation's verdict (docs/plans/2026-11-12-teleop-smooth-confirmation.md) ----------------------

def add_records(sd: Path, *, rows, obstacles, damaged=None, contacts=(), unpermitted=0, success=True):
    """result.json (scene obstacles, safety records) and the spool's slam.control (truth rear pose in columns 4-6)."""
    from forklift_core.live.spool import RecordSpool

    run = sd / "live" / "epoch_1" / "run"
    (run / "result.json").write_text(json.dumps({
        "success": success, "scene_damaged": damaged, "forbidden_pocket_contacts": list(contacts),
        "obstacle_layer": {"unpermitted_entries": unpermitted, "unpermitted_dir_entries": 0},
        "geometry": {"unloaded_footprint": {"front_m": 1.29, "rear_m": 0.17, "half_width_m": 0.36},
                     "loaded_footprint": {"front_m": 1.53, "rear_m": 0.17, "half_width_m": 0.40}},
        "scene_layout": {"obstacles": [dict(x_m=x, y_m=y, length_m=0.4, width_m=0.4, yaw_rad=0.0, height_m=1.0)
                                       for x, y in obstacles]}}))
    items: list = []
    spool = RecordSpool(run / "spool", interval_s=1e9)
    spool.register("slam.control", items, keep=0)
    items.extend(rows)
    spool.close()


def teleop_rows(sd: Path, pose=(0.0, 0.0, 0.0)):
    """One truth row per TELEOP tick of make_session's log, the truck standing at pose."""
    recs = [json.loads(x) for x in (sd / "live/epoch_1/run/live_log.jsonl").read_text().splitlines()]
    return [[r["tick_t"], 0, 0, 0, *pose] for r in recs if r.get("mode") == "TELEOP"]


def test_a_hold_reports_its_longest_pacing_gap(tmp_path):
    s = tr.session(make_session(tmp_path, "lat_g"))
    fwd = next(h for h in s["holds"] if h["label"] == "forward")
    assert fwd["longest_not_ok_ms"] == pytest.approx(300, abs=12)  # from 0.2 s to the first ok at 0.5 s
    assert fwd["longest_quiet_after_output_ms"] == 0.0 and fwd["pacing_drops"] == 0  # output never stopped once begun
    assert s["modes"]["TELEOP"]["pacing_drops"] == 0  # not ok from the start, then ok: no ok -> not-ok


@pytest.mark.parametrize("drop", ["forbidden_pocket_contacts", "obstacle_layer", "unpermitted_dir_entries",
                                  "scene_layout", "geometry"])
def test_a_missing_safety_field_is_unevaluated_not_zero(tmp_path, drop):
    """Codex registration round 3 P2-2: scene_damaged alone may be absent; every other safety field is required."""
    sd = make_session(tmp_path, "ts_m")
    add_records(sd, rows=teleop_rows(sd), obstacles=[(3.0, 0.0)])
    path = sd / "live/epoch_1/run/result.json"
    res = json.loads(path.read_text())
    if drop == "unpermitted_dir_entries":
        del res["obstacle_layer"][drop]
    else:
        del res[drop]
    path.write_text(json.dumps(res))
    safe = tr.session(sd)["safety"]
    assert safe["result"] is False and safe["unevaluated"]
    res.pop("scene_damaged", None)


def test_the_safety_check_finds_truth_overlap_in_teleop_only(tmp_path):
    sd = make_session(tmp_path, "lat_s")
    rows = teleop_rows(sd)
    rows.insert(0, [0.5, 0, 0, 0, 5.0, 5.0, 0.0])  # an AUTO tick inside an obstacle: not a TELEOP finding
    add_records(sd, rows=rows, obstacles=[(5.0, 5.0), (3.0, 0.0)])
    safe = tr.session(sd)["safety"]
    assert safe["teleop_rows"] == len(rows) - 1 and safe["teleop_overlap_ticks"] == 0
    assert safe["teleop_min_clearance_m"] == pytest.approx(3.0 - 0.2 - 1.29, abs=1e-6)
    sd2 = make_session(tmp_path, "lat_t")
    add_records(sd2, rows=teleop_rows(sd2, pose=(2.0, 0.0, 0.0)), obstacles=[(3.0, 0.0)])
    assert tr.session(sd2)["safety"]["teleop_overlap_ticks"] > 0
    sd3 = make_session(tmp_path, "lat_u")  # scene_damaged absent (the runner writes it only on damage): evaluated
    add_records(sd3, rows=teleop_rows(sd3), obstacles=[(3.0, 0.0)])
    path = sd3 / "live/epoch_1/run/result.json"
    res = json.loads(path.read_text())
    del res["scene_damaged"]
    path.write_text(json.dumps(res))
    assert tr.session(sd3)["safety"]["result"] is True


def copy_ok(s: dict, name: str | None = None) -> dict:
    """The session with pacing ok throughout, every registered hold kind present and covered, and pacing windows."""
    holds = [dict(h, covered=True, output_share=1.0 if not h["lift_dir"] else 0.0, ok_share=1.0, ok_at_start=True,
                  longest_not_ok_ms=0.0, longest_quiet_after_output_ms=0.0, pacing_drops=0,
                  first_output_ms=8.0 if not h["lift_dir"] else None) for h in s["holds"]]
    drive = next(h for h in holds if not h["lift_dir"])
    lift = next(h for h in holds if h["lift_dir"])
    have = {h["label"] for h in holds}
    holds += [dict(drive, label=k) for k in tr.DRIVE_HOLDS if k not in have]
    holds += [dict(lift, label=k) for k in tr.LIFT_HOLDS if k not in have]
    modes = dict(s["modes"], TELEOP=dict(s["modes"]["TELEOP"], pacing_drops=0, entry_not_ok_s=0.0,
                                         stretches_never_ok=0))
    return dict(s, session=name or s["session"], holds=holds, modes=modes,
                windows={"threshold": 0.81, "teleop_trace_rows": 216, "teleop_ticks": 216, "teleop_steps": 100,
                         "teleop_min": 0.88, "teleop_p1": 0.89, "teleop_below": 0})


def judged(tmp_path, name="ts_a"):
    sd = make_session(tmp_path, name)
    add_records(sd, rows=teleop_rows(sd), obstacles=[(3.0, 0.0)])
    return tr.session(sd)


def test_the_verdict_passes_and_fails_by_the_registered_rules(tmp_path):
    s = judged(tmp_path)
    # make_session's forward hold waits 0.3 s for pacing; holds and windows are missing: unevaluated, never a pass
    v = tr.verdict([s], factor=0.9)
    assert v["pass"] is False and v["failures"] == []
    problems = v["unevaluated"][0]["problems"]
    assert "missing hold reverse" in problems and "missing hold lift_down" in problems
    assert any("pacing windows" in p for p in problems)
    ok = copy_ok(s)
    v2 = tr.verdict([ok], factor=0.9)
    assert v2["pass"] is True and v2["failures"] == [] and v2["unevaluated"] == []
    # each rule fails it on its own
    for change, rule in ((lambda x: x["modes"]["TELEOP"].update(pacing_drops=1), "pacing_drops"),
                         (lambda x: x["modes"]["TELEOP"].update(entry_not_ok_s=0.15), "entry_not_ok"),
                         (lambda x: x["holds"][0].update(output_share=0.94), "hold_output"),
                         (lambda x: x["holds"][0].update(longest_quiet_after_output_ms=101.0), "output_stutter"),
                         (lambda x: x["holds"][1].update(longest_not_ok_ms=101.0), "pacing_gap")):
        bad = copy_ok(s)
        change(bad)
        assert [c["rule"] for c in tr.verdict([bad], factor=0.9)["failures"]] == [rule]
    bad = dict(ok, safety=dict(ok["safety"], scene_damaged={"reason": "pallet_moved"}))
    assert {c["rule"] for c in tr.verdict([bad], factor=0.9)["failures"]} == {"safety"}
    never = copy_ok(s)
    never["modes"]["TELEOP"]["stretches_never_ok"] = 1  # entered not ok and left before any ok tick (round 3 P3)
    assert [c["rule"] for c in tr.verdict([never], factor=0.9)["failures"]] == ["entry_not_ok"]
    # missing records: unevaluated
    for missing in (dict(ok, safety=dict(ok["safety"], teleop_rows=0)), dict(ok, overlap=None),
                    dict(ok, log_error="lost lines"), dict(ok, windows=None),
                    dict(ok, windows=dict(ok["windows"], teleop_min=None, teleop_steps=0)),
                    dict(ok, windows=dict(ok["windows"], teleop_trace_rows=100)),
                    dict(ok, holds=[dict(h, covered=False) if h["label"] == "left" else h for h in ok["holds"]])):
        v3 = tr.verdict([missing], factor=0.9)
        assert v3["pass"] is False and v3["unevaluated"], missing.get("overlap")


def test_three_registered_slots_must_each_hold_one_judged_session(tmp_path):
    base = copy_ok(judged(tmp_path))
    reg = ["ts1_k09_r1", "ts4_k09_r2", "ts5_k09_r3"]
    a, b, c = (copy_ok(base, n) for n in reg)
    assert tr.verdict([a, b, c], factor=0.9, slots=reg)["pass"] is True
    # two of three over the overlap rule: never a pass on one session (Codex registration P2-3)
    over = [a, dict(b, overlap=0.2), dict(c, overlap=0.2)]
    v = tr.verdict(over, factor=0.9, slots=reg)
    assert v["pass"] is False and sorted(v["left_out"]) == ["ts4_k09_r2", "ts5_k09_r3"]
    # a rerun stands in for its original only; its own rerun is not allowed
    re_b = copy_ok(base, "ts4_k09_r2_re")
    re_c = dict(copy_ok(base, "ts5_k09_r3_re"), overlap=0.3)
    v2 = tr.verdict(over + [re_b, re_c], factor=0.9, slots=reg)
    assert v2["judged"] == {"ts1_k09_r1": "ts1_k09_r1", "ts4_k09_r2": "ts4_k09_r2_re"} and v2["pass"] is False
    assert tr.verdict(over + [re_b, copy_ok(base, "ts5_k09_r3_re")], factor=0.9, slots=reg)["pass"] is True
    # a session a same-account job overlapped counts as over the overlap rule: its rerun stands in
    v5 = tr.verdict([a, b, c, re_b], factor=0.9, slots=reg, overlapped=("ts4_k09_r2",))
    assert v5["judged"]["ts4_k09_r2"] == "ts4_k09_r2_re" and "ts4_k09_r2" in v5["left_out"] and v5["pass"] is True
    # a valid original is judged even when a rerun exists; a missing slot fails the count
    assert tr.verdict([a, b, re_b, c], factor=0.9, slots=reg)["judged"]["ts4_k09_r2"] == "ts4_k09_r2"
    assert tr.verdict([a, b], factor=0.9, slots=reg)["pass"] is False
    # fewer (or repeated) registered names than the arm's three: never a pass on one session (round 3 P2-1)
    assert tr.verdict([a], factor=0.9, slots=["ts1_k09_r1"])["pass"] is False
    assert tr.verdict([a, b, c], factor=0.9, slots=reg + ["ts1_k09_r1"])["pass"] is False
    # another arm's session, a rerun without its original, a stray name: never judged into this arm
    other = copy_ok(base, "ts2_l04_r1")
    v3 = tr.verdict([a, b, c, other], factor=0.9, slots=reg)
    assert v3["pass"] is False and v3["judged"] == {n: n for n in reg}
    v4 = tr.verdict([a, b, copy_ok(base, "ts5_k09_r3_re")], factor=0.9, slots=reg)
    assert v4["pass"] is False and "ts5_k09_r3" not in v4["judged"]


def test_a_hold_is_judged_over_its_own_edges():
    """Codex registration P2-1: output that stops after the last inner tick, or starts late after a tick before the
    hold, counts from the hold's start and to its end."""
    def tick(mono, speed, ok=True):
        return {"tick_t": mono, "mode": "TELEOP", "pacing_ok": ok, "mono": mono, "speed_mps": speed}

    late_stop = [tick(-0.05, 0.1)] + [tick(0.01 + 0.1 * k, 0.1) for k in range(9)] + [tick(0.89, 0.0), tick(1.01, 0.0)]
    probe = {"confirm": {"holds": [{"label": "forward", "mode": "direct", "lift_dir": 0, "start_mono": 0.0,
                                    "end_mono": 1.0}]}}
    h = tr._holds(probe, late_stop)[0]
    assert h["covered"] and h["output_share"] == pytest.approx(0.89)
    assert h["longest_quiet_after_output_ms"] == pytest.approx(110.0)
    late_start = [tick(-0.05, 0.0, ok=False), tick(0.11, 0.1)] + [tick(0.2 + 0.1 * k, 0.1) for k in range(9)]
    h2 = tr._holds(probe, late_start)[0]
    assert h2["ok_at_start"] is False and h2["first_output_ms"] == pytest.approx(110.0)
    assert h2["output_share"] == pytest.approx(0.89) and h2["longest_not_ok_ms"] == pytest.approx(110.0)
    # no tick at or before the start, or none at or after the end: not covered
    assert tr._holds(probe, late_stop[1:])[0]["covered"] is False
    assert tr._holds(probe, late_stop[:-1])[0]["covered"] is False


def test_entering_teleop_not_ok_is_measured(tmp_path):
    s = tr.session(make_session(tmp_path, "ts_e"))
    assert s["modes"]["TELEOP"]["entry_not_ok_s"] == pytest.approx(0.5, abs=0.02)  # make_session: not ok for 0.5 s
    assert s["modes"]["TELEOP"]["pacing_drops"] == 0


def test_a_turn_that_starts_ok_must_answer_within_100_ms(tmp_path):
    s = copy_ok(judged(tmp_path, "ts_r"))
    left = next(h for h in s["holds"] if h["label"] == "left")
    left.update(first_output_ms=140.0)
    assert [c["rule"] for c in tr.verdict([s], factor=0.9)["failures"]] == ["turn_first_output"]
    left["ok_at_start"] = False  # started not ok: reported, never judged by this rule (entry and gap rules judge it)
    assert tr.verdict([s], factor=0.9)["failures"] == []


def test_the_pacing_windows_are_read_from_the_tick_trace(tmp_path):
    """The 1 s window the pacer computes (steps over step-end wall times), in TELEOP: its minimum and the steps whose
    window fell below 0.9 f."""
    sys.path.insert(0, str(REPO / "sim" / "isaac"))
    from tick_trace import COLUMNS

    sd = make_session(tmp_path, "ts_w")
    recs = [json.loads(x) for x in (sd / "live/epoch_1/run/live_log.jsonl").read_text().splitlines()]
    ticks = [r for r in recs if "mode" in r]
    data = np.full((len(ticks), len(COLUMNS)), np.nan)
    c = {n: i for i, n in enumerate(COLUMNS)}
    for i, r in enumerate(ticks):
        data[i, c["tick"]] = round(r["tick_t"] * 120)
        data[i, c["S"]] = r["mono"]
        data[i, c["step_end"]] = r["mono"] + 0.001
    np.savez(sd / "live/epoch_1/run/tick_trace.npz", data=data)
    w = tr.session(sd, factor=1.1)["windows"]
    assert w["teleop_trace_rows"] == w["teleop_ticks"] == 216
    assert w["teleop_min"] == pytest.approx(0.9, abs=0.01)  # TELEOP runs at 0.9 of real time
    assert w["teleop_below"] > 0 and w["threshold"] == pytest.approx(0.99)
    assert tr.session(sd, factor=0.9)["windows"]["teleop_below"] == 0


def test_the_budget_on_verdict_needs_the_budget_records_and_fails_an_invariant_violation(tmp_path):
    """2nd registration (Codex P2): with --budget-on the applied budget, its invariant count, watchdog record and
    layer_latency.json are required; a positive invariant count fails a session either way."""
    base = copy_ok(judged(tmp_path, "ts_b"))
    assert base["safety"]["layer_budget"] is None  # make_session's result.json carries no budget
    v = tr.verdict([base], factor=0.4, budget_on=True)
    assert v["pass"] is False and any("not applied" in p for u in v["unevaluated"] for p in u.get("problems", []))
    assert tr.verdict([base], factor=0.4)["pass"] is True  # the 1st registration's sessions: no budget asked
    lb = {"budget_s": 0.1, "source": "config/layer_latency_budget.yaml", "invariant_violations": 0,
          "watchdog": {"events": 2, "longest_s": 0.3}, "latency_record": True}
    ok = dict(base, safety=dict(base["safety"], layer_budget=lb))
    assert tr.verdict([ok], factor=0.4, budget_on=True)["pass"] is True
    for missing in ({**lb, "latency_record": False}, {**lb, "watchdog": None}, {**lb, "invariant_violations": None},
                    {**lb, "budget_s": 0.0}):
        bad = dict(base, safety=dict(base["safety"], layer_budget=missing))
        assert tr.verdict([bad], factor=0.4, budget_on=True)["unevaluated"], missing
    broke = dict(base, safety=dict(base["safety"], layer_budget={**lb, "invariant_violations": 1}))
    for on in (True, False):
        assert [c["rule"] for c in tr.verdict([broke], factor=0.4, budget_on=on)["failures"]] == ["safety"]


def test_the_safety_record_reads_the_layer_budget(tmp_path):
    sd = make_session(tmp_path, "ts_lb")
    add_records(sd, rows=teleop_rows(sd), obstacles=[(3.0, 0.0)])
    run = sd / "live/epoch_1/run"
    res = json.loads((run / "result.json").read_text())
    res["layer_budget"] = {"budget_s": 0.1, "invariant_violations": 0, "overdue_end_s": 1.0, "source": "x",
                           "watchdog": {"events": 1, "ticks": 3, "longest_s": 0.2, "total_s": 0.2,
                                        "longest_event_s": 0.2}}
    (run / "result.json").write_text(json.dumps(res))
    (run / "layer_latency.json").write_text("{}")
    lb = tr.session(sd)["safety"]["layer_budget"]
    assert lb["budget_s"] == 0.1 and lb["invariant_violations"] == 0 and lb["latency_record"] is True
    assert lb["watchdog"]["events"] == 1


def test_a_run_with_no_record_is_a_stub_its_rerun_can_stand_in_for(tmp_path):
    """2nd registration (Codex P2-3): an original that an infrastructure failure left without a live log or probe is
    read as a stub; listed --overlapped it is left out and its rerun judged; otherwise it is unevaluated, never a
    crash."""
    reg = ["t01_l04_r1", "t10_l04_r2", "t11_l04_r3"]
    dead = tmp_path / "t10_l04_r2" / "s"
    (dead / "live" / "epoch_1" / "run").mkdir(parents=True)
    stub = tr.session(dead)
    assert stub["missing"] and stub["session"] == "t10_l04_r2"
    base = copy_ok(judged(tmp_path, "ts_x"))
    a, c, re = (copy_ok(base, n) for n in ("t01_l04_r1", "t11_l04_r3", "t10_l04_r2_re"))
    v = tr.verdict([a, stub, c, re], factor=0.4, slots=reg, overlapped=("t10_l04_r2",))
    assert v["pass"] is True and v["judged"]["t10_l04_r2"] == "t10_l04_r2_re"
    v2 = tr.verdict([a, stub, c], factor=0.4, slots=reg)
    assert v2["pass"] is False and any("no record" in p for u in v2["unevaluated"] for p in u.get("problems", []))
    out = tmp_path / "o.json"  # the command line reads it too, without a crash
    assert tr.main([str(dead), "--json", str(out)]) == 0
