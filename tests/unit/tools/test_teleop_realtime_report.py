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
                  "end_mono": tele0 + 2.2, "start_state": {"lift_m": 0.10}, "end_state": {"lift_m": 0.13}}]}}
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


def test_the_verdict_passes_and_fails_by_the_registered_rules(tmp_path):
    sd = make_session(tmp_path, "ts_a")
    add_records(sd, rows=teleop_rows(sd), obstacles=[(3.0, 0.0)])
    s = tr.session(sd)
    # make_session's forward hold waits 0.3 s for pacing: the output share, the gap and nothing else fail
    v = tr.verdict([s], factor=0.9)
    assert v["pass"] is False
    assert {c["rule"] for c in v["failures"]} == {"hold_output", "pacing_gap"}
    # a pacing failure in TELEOP and a stutter after output fail it too
    drop = copy_ok(s)
    drop["modes"]["TELEOP"]["pacing_drops"] = 1
    drop["holds"][0]["longest_quiet_after_output_ms"] = 150.0
    assert {c["rule"] for c in tr.verdict([drop], factor=0.9)["failures"]} == {"pacing_drops", "output_stutter"}
    assert {u.get("missing_hold") for u in v["unevaluated"]} == {"loaded_forward", "reverse", "left", "right"}
    # the same session with pacing ok from the start passes
    ok = copy_ok(s)
    v2 = tr.verdict([ok], factor=0.9)
    assert v2["pass"] is True and v2["failures"] == [] and v2["unevaluated"] == []
    # a safety record fails it whatever the holds did
    bad = dict(ok, safety=dict(ok["safety"], scene_damaged={"reason": "pallet_moved"}))
    assert {c["rule"] for c in tr.verdict([bad], factor=0.9)["failures"]} == {"safety"}
    # no truth rows: unevaluated, never a pass
    blind = dict(ok, safety=dict(ok["safety"], teleop_rows=0))
    v3 = tr.verdict([blind], factor=0.9)
    assert v3["pass"] is False and v3["unevaluated"]


def copy_ok(s: dict) -> dict:
    """The session with pacing ok throughout and every registered drive hold kind present."""
    holds = [dict(h, output_share=1.0 if not h["lift_dir"] else 0.0, ok_share=1.0, ok_at_start=True,
                  longest_not_ok_ms=0.0, longest_quiet_after_output_ms=0.0, pacing_drops=0,
                  first_output_ms=8.0 if not h["lift_dir"] else None) for h in s["holds"]]
    have = {h["label"] for h in holds}
    holds += [dict(holds[0], label=k) for k in tr.DRIVE_HOLDS if k not in have]
    modes = dict(s["modes"], TELEOP=dict(s["modes"]["TELEOP"], pacing_drops=0))
    return dict(s, holds=holds, modes=modes)


def test_a_turn_that_starts_ok_must_answer_within_100_ms(tmp_path):
    sd = make_session(tmp_path, "ts_r")
    add_records(sd, rows=teleop_rows(sd), obstacles=[(3.0, 0.0)])
    s = copy_ok(tr.session(sd))
    s["holds"].append({"label": "left", "lift_dir": 0, "mode_kind": "direct", "wall_s": 1.0, "ok_at_start": True,
                       "ok_share": 1.0, "output_share": 0.96, "first_ok_ms": 0.0, "first_output_ms": 140.0,
                       "longest_not_ok_ms": 0.0, "reasons_before_output": [], "lift_delta_m": None})
    assert [c["rule"] for c in tr.verdict([s], factor=0.9)["failures"]] == ["turn_first_output"]
    s["holds"][-1]["ok_at_start"] = False  # started not ok: reported, never judged by this rule
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
    assert w["teleop_min"] == pytest.approx(0.9, abs=0.01)  # TELEOP runs at 0.9 of real time
    assert w["teleop_below"] > 0 and w["threshold"] == pytest.approx(0.99)
    assert tr.session(sd, factor=0.9)["windows"]["teleop_below"] == 0
