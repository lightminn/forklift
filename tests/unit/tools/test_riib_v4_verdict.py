"""rv4 verdict pieces (docs/plans/2026-11-10-live-riib-rv4-registration.md 1.1, 1.2, 3, 4, 5): H3 on the direct apply
record, the start-wait retry list from the probe's attempt records, attempt-level rejects in place of row-level ones,
the plant gate and the #346-family report column."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import riib_v2_verdict as V  # noqa: E402

CLOCK = {"boot_id": "b", "time_ns": "time:[1]"}


# --- H3 -------------------------------------------------------------------------------------------------------------
def h3_sess(*, sent=10.0, applied=10.5, reply=11.0, receipt=10.4, accepted=True, step_mono=11.2, probe_clock=CLOCK,
            run_clock=CLOCK, extra_commands=()):
    entries = [{"clock": run_clock}] if run_clock is not None else []
    if receipt is not None:
        entries.append({"mono": receipt, "t": "mission_cancel", "request_id": "r4"})
    if applied is not None:
        entries.append({"mono": applied, "tick_t": 52.1, "mission_commands": [
            {"t": 52.1, "command": "cancel", "request_id": "r4", "accepted": accepted}, *extra_commands]})
    events = []
    if reply is not None:
        events.append({"event": "reply", "kind": "mission_cancel", "request_id": "r4", "t": "ack",
                       "sent_mono": sent, "reply_mono": reply, "mono": reply})
    probe = {"events": events}
    if probe_clock is not None:
        probe["clock"] = probe_clock
    return {"entries": entries, "probe": probe, "steps": {1: (step_mono, 52.2, True)}}


def test_h3_direct_passes_when_the_record_precedes_the_reply_though_the_substitute_is_late():
    h = V.h3_direct(h3_sess(applied=600249.508872, reply=600249.509140, sent=600249.485, step_mono=600249.509168,
                            receipt=600249.507))
    assert h["ordered"] is True
    assert h["substitute_ordered"] is False  # rv3 r4's shape: the step stamp after the reply, reported only


@pytest.mark.parametrize("kw", [{"applied": None}, {"reply": None}, {"sent": None}])
def test_h3_direct_missing_piece_fails(kw):
    s = h3_sess(**kw) if "sent" not in kw else h3_sess()
    if "sent" in kw:
        s["probe"]["events"][0].pop("sent_mono")
    assert V.h3_direct(s)["ordered"] is False


def test_h3_direct_each_inequality_and_equality():
    assert V.h3_direct(h3_sess(sent=10.6, applied=10.5))["ordered"] is False
    assert V.h3_direct(h3_sess(applied=11.1, reply=11.0))["ordered"] is False
    assert V.h3_direct(h3_sess(sent=10.5, applied=10.5, reply=10.5))["ordered"] is True


def test_h3_direct_refused_cancel_only_counts_as_missing():
    assert V.h3_direct(h3_sess(accepted=False))["ordered"] is False


def test_h3_direct_takes_the_first_of_two_records():
    s = h3_sess(applied=10.5)
    s["entries"].append({"mono": 12.0, "mission_commands": [{"command": "cancel", "request_id": "r4",
                                                              "accepted": True}]})
    assert V.h3_direct(s)["applied_mono"] == 10.5


def test_h3_direct_receipt_race_is_reported_not_judged():
    h = V.h3_direct(h3_sess(sent=10.0, receipt=9.9))
    assert h["ordered"] is True and h["receipt_ordered"] is False


@pytest.mark.parametrize("probe_clock,run_clock", [
    ({"boot_id": "b", "time_ns": "time:[2]"}, CLOCK), (CLOCK, None), (None, CLOCK),
    ({"boot_id": None, "time_ns": None}, {"boot_id": None, "time_ns": None})])
def test_h3_direct_clock_mismatch_or_missing_fails(probe_clock, run_clock):
    h = V.h3_direct(h3_sess(probe_clock=probe_clock, run_clock=run_clock))
    assert h["ordered"] is False and h["clock_ok"] is False


# --- attempts -------------------------------------------------------------------------------------------------------
def att(rid, seq, t, reason=None, epoch=9, pallet="P-1", dest="D-1", item=None, sent=True, mono=1.0):
    meta = {"kind": "mission_submit", "request_id": rid, "session_epoch": epoch, "attempt_seq": seq,
            "pallet_id": pallet, "destination_id": dest, "item_id": item, "path": "cycle"}
    out = [{"event": "sent", "mono": mono, **meta}] if sent else []
    if t == "timeout":
        out.append({"event": "reply_timeout", "mono": mono + 0.5, **meta})
    elif t is not None:
        out.append({"event": "reply", "mono": mono + 0.1, "t": t, "reason": reason, "sent_mono": mono,
                    "reply_mono": mono + 0.1, **meta})
    return out


def probe(*attempts, extra=()):
    return {"events": [e for a in attempts for e in a] + list(extra)}


def test_first_start_wait_reject_then_same_target_ack_is_a_retry():
    for reason in ("not_stopped", "no_feedback:rear_axle_speed"):
        a = V.submit_attempts(probe(att("r2", 1, "reject", reason), att("r3", 2, "ack", mono=2.0)))
        assert a["complete"] and [r["reject_request_id"] for r in a["retries"]] == ["r2"]
        assert a["rejects"] == []


def test_consecutive_rejects_all_fail():
    for second in ("not_stopped", "no_feedback:rear_axle_speed"):
        a = V.submit_attempts(probe(att("r2", 1, "reject", "not_stopped"), att("r3", 2, "reject", second, mono=2),
                                    att("r4", 3, "ack", mono=3)))
        assert a["retries"] == [] and [r["request_id"] for r in a["rejects"]] == ["r2", "r3"]


def test_a_later_submits_start_wait_reject_fails():
    a = V.submit_attempts(probe(att("r2", 1, "ack"), att("r5", 2, "reject", "not_stopped", mono=2),
                                att("r6", 3, "ack", mono=3)))
    assert a["retries"] == [] and [r["request_id"] for r in a["rejects"]] == ["r5"]


@pytest.mark.parametrize("nxt", [dict(pallet="P-2"), dict(dest="D-2"), dict(item="I-2"), dict(t="timeout")])
def test_next_attempt_not_the_same_target_ack_is_no_retry(nxt):
    t = nxt.pop("t", "ack")
    first = att("r2", 1, "reject", "not_stopped", item="I-1" if "item" in nxt else None)
    a = V.submit_attempts(probe(first, att("r3", 2, t, mono=2.0, **nxt)))
    assert a["retries"] == [] and [r["request_id"] for r in a["rejects"]] == ["r2"]


def test_a_resent_request_id_is_one_attempt():
    # request_resent: one sent event, possibly several reply events would not happen; a duplicate sent is a problem
    a = V.submit_attempts(probe(att("q4", 1, "ack")))
    assert a["complete"] and len(a["attempts"]) == 1


def test_epochs_and_the_session_cap():
    a = V.submit_attempts(probe(att("r2", 1, "reject", "not_stopped", epoch=9), att("r3", 2, "ack", epoch=9, mono=2),
                                att("r8", 1, "reject", "not_stopped", epoch=10, mono=5),
                                att("r9", 2, "ack", epoch=10, mono=6)))
    assert [r["reject_request_id"] for r in a["retries"]] == ["r2"]  # one per session in all
    assert [r["request_id"] for r in a["rejects"]] == ["r8"]


@pytest.mark.parametrize("events", [
    probe(att("r2", 1, "ack"), att("r4", 3, "ack", mono=3)),  # seq skipped, only acks left
    probe(att("r2", 1, "reject", "not_stopped", sent=False), att("r3", 2, "ack", mono=2)),  # a reply without its send
    probe(att("r2", 1, "ack"), att("r3", 1, "ack", mono=2)),  # duplicate seq
    {"events": [{"event": "reply", "kind": "mission_submit", "request_id": "r2", "t": "ack"}]},  # no attempt fields
])
def test_an_incomplete_sequence_is_flagged_and_exempts_nothing(events):
    a = V.submit_attempts(events)
    assert a["complete"] is False and a["retries"] == []


def test_non_submit_requests_between_submits_keep_the_sequence():
    other = [{"event": "reply", "kind": "lease_request", "request_id": "r3", "t": "ack"},
             {"event": "reply", "kind": "session_end", "request_id": "r9", "t": "reject", "reason": "stale_control"}]
    a = V.submit_attempts(probe(att("r2", 1, "ack"), att("r4", 2, "ack", mono=2), extra=other))
    assert a["complete"] and a["rejects"] == []  # a session_end refusal is outside the list


# --- failures -------------------------------------------------------------------------------------------------------
def _row(mid, pallet, outcome, reason=None, start="confirmed"):
    return {"mission_id": mid, "pallet_id": pallet, "outcome": outcome, "reason": reason, "start": start}


def fsess(name, rows, events, records=None, has_result=True, cmds=()):
    return {"name": name, "rows": rows, "records": records or {}, "probe": {"events": events},
            "result": {"arguments": {}} if has_result else None,
            "entries": [{"mission_commands": list(cmds)}] if cmds else []}


def test_a_final_reject_is_counted_once_as_an_attempt():
    rows = [_row("m1", "P-1", "done"), {**_row(None, "P-2", None, start="rejected"), "reject_reason": "busy_x"}]
    ev = probe(att("r2", 1, "ack", mono=1), att("r5", 2, "reject", "busy_x", pallet="P-2", mono=2))["events"]
    f = V.rv4_failures(fsess("ab_line_m_s0_B", rows[:1] + rows[1:], ev))
    rej = [x for x in f if x["outcome"] == "rejected"]
    assert len(rej) == 1 and rej[0]["attempt_request_id"] == "r5"


def test_life2_cargo_held_only_the_two_planned_after_the_cancel():
    cancel = {"event": "reply", "kind": "mission_cancel", "request_id": "r4", "t": "ack", "reply_mono": 5.0, "mono": 5.0}
    ev = probe(att("r3", 1, "ack", pallet="P-4", mono=1), extra=[cancel])["events"]
    ev += probe(att("r5", 2, "reject", "cargo_held", mono=6), att("r6", 3, "reject", "cargo_held", mono=7))["events"]
    rows = [_row("m1", "P-4", "cancelled", "operator")]
    assert [x for x in V.rv4_failures(fsess("life2_random_m_held", rows, ev)) if x["outcome"] == "rejected"] == []
    ev3 = ev + probe(att("r7", 4, "reject", "cargo_held", mono=8))["events"]
    assert [x["attempt_request_id"] for x in V.rv4_failures(fsess("life2_random_m_held", rows, ev3))
            if x["outcome"] == "rejected"] == ["r7"]
    early = probe(att("r3", 1, "reject", "cargo_held", pallet="P-4", mono=1), extra=[cancel])["events"]
    assert [x["attempt_request_id"] for x in V.rv4_failures(fsess("life2_random_m_held", rows, early))
            if x["outcome"] == "rejected"] == ["r3"]


def test_an_incomplete_sequence_adds_one_start_unknown():
    ev = probe(att("r2", 1, "ack"), att("r4", 3, "ack", mono=3))["events"]
    f = V.rv4_failures(fsess("ab_line_m_s0_B", [_row("m1", "P-1", "done")], ev))
    assert [x["outcome"] for x in f] == ["start_unknown"] and f[0]["why"] == "attempt_sequence_incomplete"


def test_no_events_is_start_unknown_unless_a_recorded_pre_submit_end():
    s = fsess("ab_line_m_s0_B", [_row(None, None, None, start="not_submitted")], [])
    assert any(x["outcome"] == "start_unknown" for x in V.rv4_failures(s))
    l3 = fsess("life3_random_m_reloc", [_row(None, None, None, start="not_submitted")], [])
    assert V.rv4_failures(l3, life3_ended_per_record=True) == []
    l3_sub = fsess("life3_random_m_reloc", [_row(None, None, None, start="not_submitted")], [],
                   cmds=[{"command": "submit", "request_id": "r2", "accepted": True}])
    assert [x["outcome"] for x in V.rv4_failures(l3_sub, life3_ended_per_record=True)] == ["start_unknown"]


def test_session_end_refusal_is_out_of_scope():
    ev = probe(att("r2", 1, "ack"))["events"] + [
        {"event": "reply", "kind": "session_end", "request_id": "r9", "t": "reject", "reason": "stale_control"}]
    assert V.rv4_failures(fsess("ab_line_m_s0_B", [_row("m1", "P-1", "done")], ev)) == []


# --- plant ----------------------------------------------------------------------------------------------------------
def plant_result(scene_plant="p", **over):
    r = {"settings_synthetic": {"physics_hz": 120}, "physics_scene": {"dt_s": 1 / 120}, "plant_tyre_contact": [1],
         "plant_backend": {"masses_kg": [1]}, "forklift_urdf_sha256": "U", "pallet_urdf_sha256": "PU",
         "scene_built_from_urdf_sha256": "U", "scene_sha256": {"scene.usda": "S"},
         "pallet_mass": {"kg": 4.93, "engine_usd_kg": {"x": scene_plant}}, "plant": {"materials": [scene_plant]}}
    r.update(over)
    return r


MANIFEST = {"forklift_urdf_sha256": "U", "scene_sha256": {"scene.usda": "S"}}


def test_plant_gate_passes_when_global_keys_match_and_scene_keys_match_per_scene():
    sess = {"a": {"result": plant_result("x")}, "b": {"result": plant_result("x")}, "c": {"result": plant_result("y")}}
    g = V.plant_check(sess, {"a": "line_m", "b": "line_m", "c": "random_m"}, MANIFEST)
    assert g["ok"] is True


@pytest.mark.parametrize("bad", [
    {"c": plant_result("y", physics_scene={"dt_s": 1 / 60})},  # one session differs
    {"c": {k: v for k, v in plant_result("y").items() if k != "plant_backend"}},  # a key missing
    {"b": plant_result("z")},  # a scene key differs within its scene
])
def test_plant_gate_refuses(bad):
    sess = {"a": {"result": plant_result("x")}, "b": {"result": plant_result("x")}, "c": {"result": plant_result("y")}}
    sess.update({k: {"result": v} for k, v in bad.items()})
    assert V.plant_check(sess, {"a": "line_m", "b": "line_m", "c": "random_m"}, MANIFEST)["ok"] is False


def test_plant_gate_refuses_all_equal_but_not_the_manifest():
    sess = {"a": {"result": plant_result("x", forklift_urdf_sha256="V", scene_built_from_urdf_sha256="V")}}
    assert V.plant_check(sess, {"a": "line_m"}, MANIFEST)["ok"] is False


def test_plant_gate_skips_only_a_recorded_pre_submit_end_without_result():
    sess = {"a": {"result": plant_result("x")}, "life3_random_m_reloc": {"result": None}}
    scenes = {"a": "line_m", "life3_random_m_reloc": "random_m"}
    assert V.plant_check(sess, scenes, MANIFEST, skip={"life3_random_m_reloc"})["ok"] is True
    assert V.plant_check(sess, scenes, MANIFEST)["ok"] is False


# --- #346 family ----------------------------------------------------------------------------------------------------
def test_family_346_predicate():
    assert V.family_346_of({"flip_yaw_peak_rad": -0.189, "back_within_0_05_s": 10.45}) == "family"
    assert V.family_346_of({"flip_yaw_peak_rad": -0.229, "back_within_0_05_s": None}) == "family"
    assert V.family_346_of({"flip_yaw_peak_rad": -0.058, "back_within_0_05_s": 0.12}) == "not_family"
    assert V.family_346_of({"flip_yaw_peak_rad": -0.12, "back_within_0_05_s": 1.5}) == "not_family"
    assert V.family_346_of({"error": "x"}) == "unclassifiable"
    assert V.family_346_of({"flip_yaw_peak_rad": None}) == "unclassifiable"
