"""The R1 confirmation analysis (docs/validation/2026-11-08-r1-confirmation/r1c_analyze.py, plan 5.17): the verdict's
rules on synthetic session rows, a synthetic run folder read back (STOP intervals, the stop-complete step, holds,
expiry), and the real arbiter's expiry path the analysis relies on (a disarm event with reason expired)."""

import importlib.util
import itertools
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "r1c_analyze", ROOT / "docs" / "validation" / "2026-11-08-r1-confirmation" / "r1c_analyze.py")
ra = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ra)

from forklift_core.safety.arbiter import Arbiter, ArbiterConfig, Feedback  # noqa: E402
from tick_trace import COLUMNS  # noqa: E402  (sim/isaac on the path through the module above)


# --- the verdict on synthetic rows --------------------------------------------------------------------------------------
def stop(label):
    # the command-zero bracket is one status line wide: one control tick, 1/(120 f) s at f = 0.4
    return {"label": label, "moving_at_send": True, "send_to_processed_s": 0.01, "processed_to_zero_s": [0.0, 0.02],
            "processed_to_still_s": 0.6}


def holds_row(blocked=0.1, heading=1.0, lift=1.0, dist=1.0, stop_t=True):
    hs = [{"label": "loaded_forward", "ticks": 100, "pacing_blocked_tick_share": blocked, "moving_output_share": 0.9,
           "distance_m": 0.3, "heading_change_rad": 0.0, "lateral_max_m": 0.01}]
    for label, d in ra.DISTANCE.items():
        h = {"label": label, "ticks": 100, "pacing_blocked_tick_share": blocked, "moving_output_share": 0.9,
             "distance_m": d * dist, "heading_change_rad": 0.0, "lateral_max_m": 0.01}
        if label == "forward" and stop_t:
            h["stop"] = stop("T")
        hs.append(h)
    for label, hc in ra.HEADING.items():
        hs.append({"label": label, "ticks": 100, "pacing_blocked_tick_share": blocked, "moving_output_share": 0.9,
                   "heading_change_rad": ra.HEADING_SIGN[label] * hc * heading, "distance_m": 0.3, "lateral_max_m": 0.02})
    for label, lc in ra.LIFT.items():
        hs.append({"label": label, "ticks": 100, "pacing_blocked_tick_share": blocked, "distance_m": 0.0,
                   "heading_change_rad": 0.0, "lift_change_m": ra.LIFT_SIGN[label] * lc * lift})
    return hs


def row(name, set_, arm, factor, *, ok=0.8, blocked=0.1, outcome="done", reason=None, **kw):
    r = {"name": name, "set": set_, "arm": arm, "factor": factor, "invalid": []}
    if set_ in ("far", "off"):
        r["T1"] = holds_row(blocked=blocked, **{k: v for k, v in kw.items() if k in ("heading", "lift", "dist", "stop_t")})
        r["T3"] = [stop("A1"), stop("A2")]
        r["T2"] = {"boundary": {"moving_before": True, "to_expired_s": 0.5, "to_zero_s": 0.5}, "normal": {"expired": 0}}
        r["T5"] = {"mission": {"outcome": outcome, "reason": reason}, "auto_ok_share_after_first": ok, "auto_ratio": 0.4,
                   "safety": {"forbidden_contacts_run": 0, "forbidden_contacts_missions": 0, "unpermitted_entries": 0,
                              "unpermitted_dir_entries": 0}}
    elif set_ == "mark":
        r["T4"] = {"pixel": {"p99": 0.3}, "freeze_disarmed_stale_video": True}
    elif set_ == "wall":
        r["wall"] = [{"leg": "forward_to_wall", "held": True, "moved": True, "permit_reason": "occupied", "base_x_m": 4.0},
                     {"leg": "reverse_swing", "held": False, "moved": True},
                     {"leg": "forward_again", "held": True, "moved": True, "permit_reason": "occupied", "base_x_m": 4.0}]
    return r


def far_set(factor=0.4, k_kw=None, l_kw=None):
    expected, rows = {}, []
    n = 0
    for arm in ("L", "K", "K", "L", "L", "K"):
        n += 1
        name = f"r{n:02d}"
        expected[name] = ("far", arm, str(factor))
        rows.append(row(name, "far", arm, factor, **((k_kw if arm == "K" else l_kw) or {})))
    expected["m1"] = ("mark", "K", str(factor))
    rows.append(row("m1", "mark", "K", factor))
    if factor == 0.4:
        for arm in ("L", "K"):
            expected[f"w{arm}"] = ("wall", arm, "0.4")
            rows.append(row(f"w{arm}", "wall", arm, 0.4))
    return expected, rows


def test_a_candidate_as_good_with_a_better_ok_is_a_change():
    expected, rows = far_set(k_kw={"ok": 0.9}, l_kw={"ok": 0.8})
    v = ra.verdict(rows, expected, "far", 0.4, "K")
    assert v["verdict"] == "change" and "ok" in v["gains"]


def test_no_gain_when_nothing_improves():
    expected, rows = far_set()
    assert ra.verdict(rows, expected, "far", 0.4, "K")["verdict"] == "no gain"


def test_a_missing_or_invalid_registered_session_waits_for_its_supplement():
    expected, rows = far_set()
    rows[1]["invalid"] = ["tick trace missing, errored or dropped"]
    assert ra.verdict(rows, expected, "far", 0.4, "K")["verdict"] == "pending (supplement)"
    expected, rows = far_set()
    assert ra.verdict(rows[1:], expected, "far", 0.4, "K")["verdict"] == "pending (supplement)"


def test_a_row_whose_conditions_differ_from_the_list_is_refused():
    expected, rows = far_set()
    rows[1]["factor"] = 0.3
    assert ra.verdict(rows, expected, "far", 0.4, "K")["verdict"].startswith("invalid")


def test_a_candidate_without_the_loaded_teleop_or_moving_stops_or_a_moving_expiry_is_held():
    expected, rows = far_set(k_kw={"ok": 0.9})
    for r in rows:
        if r["arm"] == "K" and r["set"] == "far":
            r["T1"] = [h for h in r["T1"] if h["label"] != "loaded_forward"]
    assert ra.verdict(rows, expected, "far", 0.4, "K")["verdict"] == "hold"
    expected, rows = far_set(k_kw={"ok": 0.9})
    for r in rows:
        if r["arm"] == "K" and r["set"] == "far":
            for s in r["T3"]:
                s["moving_at_send"] = False
    assert ra.verdict(rows, expected, "far", 0.4, "K")["verdict"] == "hold"
    expected, rows = far_set(k_kw={"ok": 0.9})
    for r in rows:
        if r["arm"] == "K" and r["set"] == "far":
            r["T2"]["boundary"]["moving_before"] = False
    assert ra.verdict(rows, expected, "far", 0.4, "K")["verdict"] == "hold"


@pytest.mark.parametrize("kw", [{"heading": -1.0}, {"lift": -1.0}])
def test_a_turn_or_a_lift_the_other_way_fails_the_function_rule(kw):
    expected, rows = far_set(k_kw={"ok": 0.9, **kw})
    v = ra.verdict(rows, expected, "far", 0.4, "K")
    assert v["verdict"].startswith("no change") and any(w.startswith("R-c") for w in v["why"])


@pytest.mark.parametrize("label, key, value", [("loaded_forward", "distance_m", 0.0),
                                               ("loaded_forward", "moving_output_share", 0.0),
                                               ("left", "distance_m", 0.0), ("right", "distance_m", 0.0),
                                               ("forward", "heading_change_rad", 0.5),
                                               ("reverse", "heading_change_rad", 0.5),
                                               ("video_forward", "heading_change_rad", 0.5)])
def test_every_block_is_compared_on_every_measure(label, key, value):
    expected, rows = far_set(k_kw={"ok": 0.9})
    for r in rows:
        if r["arm"] == "K" and r["set"] == "far":
            for h in r["T1"]:
                if h["label"] == label:
                    h[key] = value
    v = ra.verdict(rows, expected, "far", 0.4, "K")
    assert v["verdict"] != "change" and any(w.startswith("R-c") and label in w for w in v["why"])


def test_a_slower_stop_beyond_the_bracket_fails_and_inside_it_holds():
    expected, rows = far_set(k_kw={"ok": 0.9})
    for r in rows:
        if r["arm"] == "K" and r["set"] == "far":
            for s in r["T3"]:
                s["processed_to_zero_s"] = [0.2, 0.25]
    v = ra.verdict(rows, expected, "far", 0.4, "K")
    assert any(w == "R-b fail" for w in v["why"])
    expected, rows = far_set(k_kw={"ok": 0.9})
    for r in rows:
        if r["arm"] == "K" and r["set"] == "far":
            for s in r["T3"]:
                s["processed_to_zero_s"] = [0.0, 0.05]  # wider than a tick: the bracket cannot tell
    v = ra.verdict(rows, expected, "far", 0.4, "K")
    assert v["verdict"] == "hold" and any("resolution" in w for w in v["why"])


def test_a_wall_stop_not_held_by_an_occupied_permission_fails():
    expected, rows = far_set(k_kw={"ok": 0.9})
    for r in rows:
        if r["name"] == "wK":
            r["wall"][0]["permit_reason"] = "pacing"
    v = ra.verdict(rows, expected, "far", 0.4, "K")
    assert any(w.startswith("R-g") for w in v["why"])


def test_a_new_failure_reason_on_the_candidate_holds():
    expected, rows = far_set(k_kw={"ok": 0.9})
    rows[1]["T5"]["mission"] = {"outcome": "failed", "reason": "cusp_position_stall"}
    v = ra.verdict(rows, expected, "far", 0.4, "K")
    assert v["verdict"] == "hold"


def test_a_baseline_that_does_not_complete_is_undecidable():
    expected, rows = far_set(l_kw={"outcome": "failed", "reason": "x"})
    assert ra.verdict(rows, expected, "far", 0.4, "K")["verdict"] == "undecidable"


# --- a synthetic run folder read back -----------------------------------------------------------------------------------
C = {n: i for i, n in enumerate(COLUMNS)}


def write_run(tmp_path, *, profile=("6", ""), records=True, rearm_step=None):
    out = tmp_path / "r01"
    run = out / "live" / "epoch_1" / "run"
    (run / "spool" / "slam_log.joint").mkdir(parents=True)
    env = [f"SET=far", "ARM=K", "FACTOR=0.4", f"FORKLIFT_LIVE_CATCHUP_STEPS={profile[0]}"]
    if profile[1]:
        env.append(f"FORKLIFT_LIVE_CAMERA_PHASE={profile[1]}")
    (out / "args.txt").write_text("\n".join(env) + "\n")
    (out / "probe_rc").write_text("0\n")
    # 600 physics steps at factor 0.4: paced starts every 20.8 ms, world.step 2 ms
    n = 600
    data = np.full((n, len(COLUMNS)), np.nan)
    h = (1 / 120) / 0.4
    S = 1000.0 + h * np.arange(n)
    data[:, C["S"]], data[:, C["q"]], data[:, C["d"]] = S, S - 0.001, 0.0
    data[:, C["world_start"]], data[:, C["world_end"]] = S + 0.0005, S + 0.0025
    data[:, C["step_end"]], data[:, C["after_end"]] = S + 0.003, S + 0.0035
    data[:, C["tick"]] = np.arange(n)
    np.savez(run / "tick_trace.npz", data=data, columns=np.array(COLUMNS), flags=np.array(["x"]))
    # wheels turn until step 300, then stop at once; the base moves along x and turns slowly
    stamps = (np.arange(n) + 1) / 120
    rates = np.where(np.arange(n) < 300, 2.0, 0.0)[:, None] * np.ones((1, 4))
    x = 0.15 * np.minimum(stamps, 300 / 120)
    yaw = 0.05 * np.minimum(stamps, 300 / 120)
    pose = np.stack([x, np.zeros(n), np.zeros(n), np.cos(yaw / 2), np.zeros(n), np.zeros(n), np.sin(yaw / 2)], axis=1)
    np.savez(run / "spool" / "slam_log.joint" / "000000.npz", joint_stamps_s=stamps, wheel_rates_rad_s=rates,
             base_pose_world=pose)
    (run / "spool" / "manifest.json").write_text(json.dumps({"complete": True, "slam_log.joint": {"items": n}}))
    # the STOP processed between steps 298 and 299; the status lines every step, zero speed from step 300
    proc = float(S[298] + 0.01)
    lines = [{"tick_t": float(i / 120), "mono": float(S[i] + 0.004), "mode": "AUTO", "pacing_ok": True,
              "speed_mps": 0.15 if i < 300 else 0.0, "step": i} for i in range(n)]
    lines.append({"mono": proc, "where": "step", "conn": "c", "client": "op", "t": "stop", "request_id": "r9",
                  "reply": ["ack", None]})
    lines.append({"t": float(S[400]), "event": "disarm", "reason": "expired", "arbiter": True, "tick_t": 400 / 120})
    if rearm_step is not None:
        lines.append({"t": float(S[rearm_step]), "event": "mode", "reason": "rearm", "arbiter": True,
                      "tick_t": rearm_step / 120})
    lines.sort(key=lambda e: e.get("mono", e.get("t")))
    (run / "live_log.jsonl").write_text("".join(json.dumps(e) + "\n" for e in lines))
    (run / "result.json").write_text(json.dumps({"tick_trace": {"rows": n, "dropped": 0}}))
    if records:
        (run / "missions" / "m1").mkdir(parents=True)
        (run / "missions" / "m1" / "records.json").write_text(json.dumps({"state": {"forbidden_pocket_contacts": []}}))
    confirm = {"submit": {"mission_id": "m1"},
               "stops": [{"label": "A1", "sent_mono": proc - 0.005, "request_id": "r9", "moving_at_send": True}],
               "holds": [{"label": "forward", "mode": "direct", "start_mono": float(S[100]), "end_mono": float(S[200]),
                          "stand_mono": float(S[200]), "start_state": {"lift_m": 0.0}, "stand_state": {"lift_m": 0.0}}],
               "expiry": {"silence_start_mono": float(S[390]), "silence_s": 1.25, "moving_before": True},
               "mission_result": {"outcome": "done"}, "steps": []}
    (out / "probe.json").write_text(json.dumps({"confirm": confirm}))
    return out


def test_a_run_folder_reads_back_its_stop_intervals_hold_and_expiry(tmp_path):
    s = ra.Session(write_run(tmp_path))
    assert s.invalid == []
    r = s.analyse()
    st = r["T3"][0]
    assert st["send_to_processed_s"] == pytest.approx(0.005)
    lo, hi = st["processed_to_zero_s"]
    assert 0 <= lo <= hi and hi < 0.05
    assert st["processed_to_still_s"] >= 0  # never a still window that ended before the STOP was processed
    hold = r["T1"][0]
    assert hold["distance_m"] == pytest.approx(0.15 * 100 / 120, rel=0.05)
    assert hold["heading_change_rad"] == pytest.approx(0.05 * 100 / 120, rel=0.05)  # yaw from the quaternion
    assert hold["pacing_blocked_tick_share"] == 0.0 and hold["moving_output_share"] == 1.0
    exp = r["T2"]["boundary"]
    assert exp["to_expired_s"] == pytest.approx(float((400 - 390) * (1 / 120) / 0.4), rel=1e-6)


def test_a_wrong_profile_or_a_missing_mission_record_makes_the_session_invalid(tmp_path):
    assert any("profile" in w for w in ra.Session(write_run(tmp_path / "a", profile=("0", ""))).invalid)
    assert any("records.json" in w for w in ra.Session(write_run(tmp_path / "b", records=False)).invalid)


# --- the real arbiter's expiry path -------------------------------------------------------------------------------------
def test_the_arbiter_logs_an_expired_disarm_when_no_command_arrives():
    """The analysis reads the boundary expiry from the arbiter's disarm event (last_teleop_reject stays as it was)."""
    counter = itertools.count(1)
    epoch = 3
    safe = {"load": {"loaded": False}, "fork_in_pocket": {"value": False}, "near": None,
            "frame": {"hold": "none", "slam": True}, "mission": {"phase": "observe"}}
    a = Arbiter(epoch, ArbiterConfig(), lambda: f"tok{next(counter)}")
    a.connect("op", 0.0)
    lease_id = a.handle("op", {"t": "lease_request", "request_id": "L", "session_epoch": epoch}, 0.0)["lease_id"]
    t = 0.0
    for _ in range(40):
        t += 1 / 120
        a.observe(t, Feedback(t, 0.0))
        a.step(t, safe, (0.0, 0.0))
    a.last_safety = safe
    base = {"session_epoch": epoch, "lease_id": lease_id}
    assert a.handle("op", {**base, "t": "mode_request", "request_id": "M", "control_gen": a.control_gen,
                           "to": "TELEOP"}, t)["t"] == "ack"
    arm_gen = a.handle("op", {**base, "t": "arm", "request_id": "A", "control_gen": a.control_gen}, t)["arm_gen"]
    tick = a.tick(t)
    a.handle("op", {**base, "t": "teleop", "control_gen": a.control_gen, "arm_gen": arm_gen, "seq": 1,
                    "echo": {"state_seq": tick["state_seq"], "token": tick["token"]}, "mode": "direct",
                    "deadman": True, "speed_ratio": 0.3, "curvature_ratio": 0.0, "lift_dir": 0}, t)
    a.events.clear()
    a.observe(t + 0.31, Feedback(t + 0.31, 0.1))
    disarms = [e for e in a.events if e.get("event") == "disarm"]
    assert disarms and disarms[0]["reason"] == "expired" and a.last_teleop_reject is None


def freeze_session(samples, end_mono=None, factor=0.4, events=()):
    """A mark session for the freeze check: ticks at 120 Hz, wall S at the factor, the args' freeze 30:60, and the
    runner's arm / disarm events (sim times) the judgement reads (R1 results)."""
    ticks = np.arange(0, 120 * 70)
    data = np.full((len(ticks), len(ra.C)), np.nan)
    data[:, ra.C["tick"]] = ticks
    data[:, ra.C["S"]] = 1000.0 + ticks * ra.DT / factor
    s = SimpleNamespace(trace=data, args={"EXTRA_ARGS": r"--live-test-marker\ --live-test-camera-freeze\ 30:60"},
                        report={"freeze": {"samples": samples}, **({"end_requested_mono": end_mono} if end_mono else {})},
                        arbiter_events=[{"arbiter": True, "event": ev, "reason": why, "tick_t": t} for ev, why, t in events])
    s.row_at_mono = lambda m: ra.Session.row_at_mono(s, m)
    s.start_sim = lambda i: ra.Session.start_sim(s, i)
    return s


def at(sim, factor=0.4):
    return 1000.0 + sim / factor


def test_the_freeze_refusal_needs_an_arm_before_it_and_the_runners_disarm_inside_it():
    inside = {"mono": at(35.0), "armed": False, "reject": "stale_video", "last_frame_sim": 29.95}
    assert ra.freeze_window({"EXTRA_ARGS": r"--live-test-camera-freeze\ 30:60\ --x"}) == (30.0, 60.0)
    good = freeze_session([inside], end_mono=at(36.0), events=[("arm", None, 20.7), ("disarm", "stale_video", 30.15)])
    out = ra.freeze_check(good)
    assert out["armed_at_freeze_start"] and out["freeze_disarmed_stale_video"] and out["freeze_disarm_sim_s"] == 30.15
    assert out["ended_after_measures"] is True
    # r1c_18: the samples missed it (the last reject overwritten) but the runner's event is there
    missed = freeze_session([{"mono": at(31.0), "armed": False, "reject": "not_armed", "last_frame_sim": 29.95}],
                            events=[("arm", None, 20.7), ("disarm", "stale_video", 30.15)])
    assert ra.freeze_check(missed)["freeze_disarmed_stale_video"]
    # disarmed before the freeze and never armed again: nothing proved, whatever the samples say
    early = freeze_session([inside], events=[("arm", None, 20.7), ("disarm", "stale_video", 23.96),
                                             ("disarm", "stale_video", 30.2)])
    out = ra.freeze_check(early)
    assert out["armed_at_freeze_start"] is False and not out["freeze_disarmed_stale_video"]
    # armed again before the freeze: proved again
    rearmed = freeze_session([inside], events=[("arm", None, 20.7), ("disarm", "expired", 23.96), ("arm", None, 24.5),
                                               ("disarm", "stale_video", 30.2)])
    assert ra.freeze_check(rearmed)["freeze_disarmed_stale_video"]
    # another reason inside the freeze is not the freeze's refusal
    other = freeze_session([inside], events=[("arm", None, 20.7), ("disarm", "expired", 31.0)])
    assert not ra.freeze_check(other)["freeze_disarmed_stale_video"]
    assert ra.freeze_check(freeze_session([inside], end_mono=at(34.0),
                                          events=[("arm", None, 20.7)]))["ended_after_measures"] is False


def test_the_freeze_window_is_read_from_a_real_args_file(tmp_path):
    """Round 2 P2: read_args dropped EXTRA_ARGS, so a real session never had a freeze window -- from the file on."""
    (tmp_path / "args.txt").write_text(
        "ARM=K\nFACTOR=0.4\nFORKLIFT_LIVE_CATCHUP_STEPS=6\nSET=mark\n"
        "EXTRA_ARGS=--live-test-marker\\ --live-test-camera-freeze\\ 30:60\\ --max-sim-seconds\\ 1500\n"
        "PROBE=--scenario marker --frames-dir /x/frames --frames 200 --freeze-from 30 --end-session \n"
        "0123abcd  /x/r1c_session.sh\n")
    args = ra.read_args(tmp_path)
    assert args["SET"] == "mark" and ra.freeze_window(args) == (30.0, 60.0)
    s = freeze_session([], events=[("arm", None, 20.7), ("disarm", "stale_video", 30.2)])
    s.args = args
    assert ra.freeze_check(s)["freeze_disarmed_stale_video"]


def test_the_stop_search_closes_at_the_rearm(tmp_path):
    """R1 results: a still window after the rearm belongs to later driving, not to the STOP (r1c_02's STOP-T)."""
    early = ra.Session(write_run(tmp_path / "a", rearm_step=305)).analyse()["T3"][0]  # before 0.2 sim s of stillness
    assert early["still_before_rearm"] is False and "processed_to_still_s" not in early
    assert early["rearm_after_processed_s"] > 0
    late = ra.Session(write_run(tmp_path / "b", rearm_step=450)).analyse()["T3"][0]
    assert late["still_before_rearm"] is True and late["processed_to_still_s"] >= 0
