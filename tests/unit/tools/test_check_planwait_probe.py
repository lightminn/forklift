"""Real-time plan V8-5: the planwait judge passes a run where pause, STOP and cancel each land in a plan wait and
fails the runs Codex built against its earlier versions (real-time D8 rounds 10-11): requests after the plans,
stale or repeated samples, a handling later than the first send, a control connection lost before the cancel."""

import copy

from tools import check_planwait_probe as chk

T0 = 1000.0  # the host's monotonic clock at sim 0; one command line every 0.1 s, a status line every 0.5 s
PAUSE, RESUME1, STOP, RESUME2, CANCEL = 20.0, 25.0, 35.0, 40.0, 50.0


def mono(t):
    return T0 + t


def run():
    log = []
    for k in range(0, 503):
        t = round(k * 0.1, 1)
        mode = ("PAUSED" if PAUSE <= t < RESUME1 or STOP + 3 <= t < RESUME2
                else "STOPPED" if STOP <= t < STOP + 3 else "AUTO")
        log.append({"tick_t": t, "mode": mode, "speed_mps": 0.0, "mono": mono(t)})
        if k % 5 == 0:
            log.append({"status": True, "tick_t": t, "mono": mono(t), "feedback_ok": True,
                        "feedback": [mono(t) - 0.05, 0.01, 0.0]})
    for rid, kind, t in (("r5", "mission_pause", PAUSE), ("r6", "mission_resume", RESUME1), ("r7", "stop", STOP),
                         ("r11", "mission_resume", RESUME2), ("r12", "mission_cancel", CANCEL)):
        log.append({"mono": mono(t), "where": "cycle", "t": kind, "request_id": rid, "reply": None})
    log.sort(key=lambda d: d["mono"])
    plans = [{"site": "observe", "job": 1, "requested_s": 10.0, "finished_s": PAUSE, "dropped": "paused"},
             {"site": "observe", "job": 2, "requested_s": 30.0, "finished_s": STOP, "dropped": "paused"},
             {"site": "observe", "job": 3, "requested_s": 45.0, "finished_s": CANCEL, "dropped": "paused"}]
    events = [{"mono": mono(0), "event": "reached", "label": s} for s in chk.STEPS]
    for rid, kind, t in (("r5", "mission_pause", PAUSE), ("r6", "mission_resume", RESUME1), ("r7", "stop", STOP),
                         ("r11", "mission_resume", RESUME2), ("r12", "mission_cancel", CANCEL)):
        events.append({"mono": mono(t) + 0.02, "event": "reply", "kind": kind, "request_id": rid, "t": "ack"})
    events.append({"mono": mono(CANCEL) + 0.2, "event": "control_closed", "code": 1001, "current": True})
    probe = {"events": events, "pause_reply": {"t": "ack", "request_id": "r5"},
             "stop_reply": {"t": "ack", "request_id": "r7"}, "cancel_reply": {"t": "ack", "request_id": "r12"}}
    result = {"success": False, "failure_reason": "mission_cancelled", "async_plans": plans}
    return probe, result, log


def judge(probe, result, log):
    return chk.judge(probe, result, log)[0]


def test_a_run_whose_requests_land_in_plan_waits_passes():
    assert judge(*run()) == []


def test_requests_made_after_the_plans_fail():
    probe, result, log = run()
    for p in result["async_plans"]:
        p["finished_s"] -= 2.0  # each plan ended before the request was handled
    fails = judge(probe, result, log)
    assert any("0 plans computing" in f for f in fails)


def test_a_taken_plan_fails():
    probe, result, log = run()
    result["async_plans"][1]["dropped"] = None
    assert any("was taken" in f for f in judge(probe, result, log))


def test_stale_or_repeated_samples_do_not_count():
    probe, result, log = run()
    first = None
    for d in log:
        if d.get("status") and PAUSE <= d["tick_t"] <= RESUME1:
            first = first or list(d["feedback"])
            d["feedback"] = list(first)  # the same sample repeated, older each status line
    assert any("fresh samples" in f for f in judge(probe, result, log))
    probe, result, log = run()
    for d in log:
        if d.get("status") and STOP <= d["tick_t"] <= RESUME2:
            d["feedback_ok"] = False
    assert any("stopped: 0 fresh samples" in f for f in judge(probe, result, log))


def test_a_gap_in_the_samples_fails():
    probe, result, log = run()
    log = [d for d in log if not (d.get("status") and 21.0 <= d["tick_t"] <= 23.0)]
    assert any("gap in the samples" in f for f in judge(probe, result, log))


def test_the_handling_is_the_servers_record_of_the_acked_request():
    probe, result, log = run()
    for d in log:
        if d.get("request_id") == "r5":
            d["mono"] = mono(PAUSE + 2.0)  # handled after the plan had ended
    log.sort(key=lambda d: d["mono"])
    assert any(f.startswith("pause: 0 plans computing") for f in judge(probe, result, log))
    probe, result, log = run()
    probe["pause_reply"] = {"t": "ack", "request_id": "r99"}  # an ack the server never recorded
    assert any("no server record for the acked r99" in f for f in judge(probe, result, log))


def test_a_connection_lost_before_the_cancel_fails():
    probe, result, log = run()
    probe["events"].append({"mono": mono(30.0), "event": "control_closed", "code": 1006, "current": True})
    assert any("probe problems" in f for f in judge(probe, result, log))


def test_a_cancel_that_did_not_end_the_run_or_kept_driving_fails():
    probe, result, log = run()
    result["failure_reason"] = None
    fails = judge(probe, result, log)
    assert any("want mission_cancelled" in f for f in fails) and any("probe problems" in f for f in fails)
    probe, result, log = run()
    for d in reversed(log):
        if "speed_mps" in d:
            d["speed_mps"] = 0.2
            break
    assert any("commanded speeds after the cancel" in f for f in judge(probe, result, log))


def test_a_plan_asked_while_paused_fails():
    probe, result, log = run()
    result["async_plans"].insert(1, {"site": "observe", "job": 9, "requested_s": 22.0, "finished_s": 22.5,
                                     "dropped": "paused"})
    assert any("asked while paused" in f for f in judge(probe, result, copy.deepcopy(log)))


def test_a_plan_asked_in_the_tick_the_pause_or_cancel_took_effect_fails_and_the_resume_tick_may_ask():
    """Codex real-time D8 round 12 P1: commands apply at the tick's start; the boundary ticks count."""
    probe, result, log = run()
    result["async_plans"].insert(1, {"site": "observe", "job": 8, "requested_s": PAUSE, "finished_s": PAUSE + 0.1,
                                     "dropped": "paused"})
    assert any("asked while paused" in f for f in judge(probe, result, log))
    probe, result, log = run()
    result["async_plans"].append({"site": "observe", "job": 10, "requested_s": CANCEL})
    assert any("asked after the cancel" in f for f in judge(probe, result, log))
    probe, result, log = run()
    result["async_plans"][1]["requested_s"] = RESUME1  # asked in the resume's own tick: allowed
    assert judge(probe, result, log) == []
