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
    entries = [{"event": "start", "session_epoch": 5, "mono": 1.0}]
    if receipt is not None:
        entries.append({"mono": receipt, "t": "mission_cancel", "request_id": "r4"})
    if applied is not None:
        entries.append({"mono": applied, "tick_t": 52.1, "mission_commands": [
            {"t": 52.1, "command": "cancel", "request_id": "r4", "accepted": accepted, "mission_id": "m1"},
            *extra_commands]})
    events = []
    if reply is not None:
        events.append({"event": "reply", "kind": "mission_cancel", "request_id": "r4", "t": "ack",
                       "sent_mono": sent, "reply_mono": reply, "mono": reply})
    probe = {"events": events, "session": {"session_epoch": 5},
             "missions_run": [{"cancel_reply": {"t": "ack", "request_id": "r4", "mission_id": "m1"}}]}
    if probe_clock is not None:
        probe["clock"] = probe_clock
    return {"entries": entries, "probe": probe, "steps": {1: (step_mono, 52.2, True)}, "runtime_clock": run_clock}


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
                                                              "accepted": True, "mission_id": "m1"}]})
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
    rows = [_row("m1", "P-4", "cancelled", "operator")]

    def rej(ev):
        s_ = fsess("life2_random_m_held", rows, ev)
        s_["probe"] = life2_probe_extra(ev)
        return [x["attempt_request_id"] for x in V.rv4_failures(s_) if x["outcome"] == "rejected"]
    ev = life2_events()
    assert rej(ev) == []
    ev3 = ev + probe(att("r7", 4, "reject", "cargo_held", pallet="P-4", mono=8))["events"]
    assert rej(ev3) == ["r7"]  # a third cargo_held
    no_plan = fsess("life2_random_m_held", rows, ev)  # no missions_run: nothing is waived
    assert [x["attempt_request_id"] for x in V.rv4_failures(no_plan) if x["outcome"] == "rejected"] == ["r5", "r6"]


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


# --- Codex implementation review round 1 ------------------------------------------------------------------------------
def test_attempt_fields_missing_or_reply_contradicting_its_send_is_incomplete():
    bare = probe(att("r2", 1, "reject", "not_stopped"), att("r3", 2, "ack", mono=2))
    for e in bare["events"]:
        e.pop("pallet_id")
    assert V.submit_attempts(bare)["complete"] is False
    noepoch = probe(att("r2", 1, "reject", "not_stopped"), att("r3", 2, "ack", mono=2))
    for e in noepoch["events"]:
        e["session_epoch"] = None
    assert V.submit_attempts(noepoch)["complete"] is False
    contra = probe(att("r2", 1, "reject", "not_stopped"), att("r3", 2, "ack", mono=2))
    contra["events"][1]["pallet_id"] = "P-9"  # the reply names another target than its send
    a = V.submit_attempts(contra)
    assert a["complete"] is False and a["retries"] == []


def test_an_incomplete_sequence_keeps_the_rejects_it_saw():
    ev = probe(att("r2", 1, "reject", "busy_x", sent=False), att("r3", 2, "ack", mono=2))["events"]
    f = V.rv4_failures(fsess("ab_line_m_s0_B", [_row("m1", "P-1", "done")], ev))
    assert {x["outcome"] for x in f} == {"start_unknown", "rejected"}
    assert [x.get("attempt_request_id") for x in f if x["outcome"] == "rejected"] == ["r2"]


def test_rows_with_submit_evidence_and_no_submit_events_keep_the_row_refusal():
    rows = [{**_row(None, "P-1", None, start="rejected"), "reject_reason": "busy_x"}, _row("m2", "P-2", "done"),
            _row("m3", "P-3", "done")]
    f = V.rv4_failures(fsess("ab_line_m_s0_B", rows, [{"event": "reached", "label": "welcome"}]))
    assert {x["outcome"] for x in f} == {"start_unknown", "rejected"}


def h3_full(**kw):
    return h3_sess(**kw)


def test_h3_direct_requires_the_same_epoch_and_mission():
    assert V.h3_direct(h3_full())["ordered"] is True
    other_epoch = h3_full()
    other_epoch["entries"][0]["session_epoch"] = 6
    assert V.h3_direct(other_epoch)["ordered"] is False
    other_mission = h3_full()
    other_mission["probe"]["missions_run"][0]["cancel_reply"]["mission_id"] = "m2"
    assert V.h3_direct(other_mission)["ordered"] is False


def test_h3_direct_two_accepted_records_for_different_missions_is_ambiguous():
    s = h3_full()
    s["entries"].append({"mono": 10.6, "mission_commands": [{"command": "cancel", "request_id": "r4", "accepted": True,
                                                              "mission_id": "m2"}]})
    assert V.h3_direct(s)["ordered"] is False


def test_clock_values_must_be_non_empty():
    empty = {"boot_id": "", "time_ns": ""}
    assert V.h3_direct(h3_full(probe_clock=empty, run_clock=empty))["clock_ok"] is False


def life2_events(targets=(("P-4", "D-1"), ("P-5", "D-2")), sent_before_cancel=False):
    cancel = {"event": "reply", "kind": "mission_cancel", "request_id": "r4", "t": "ack", "reply_mono": 5.0, "mono": 5.0}
    ev = probe(att("r3", 1, "ack", pallet="P-4", mono=1), extra=[cancel])["events"]
    for i, (pal, dest) in enumerate(targets):
        mono = 4.0 if sent_before_cancel and i == 0 else 6.0 + i
        ev += probe(att(f"r{5 + i}", 2 + i, "reject", "cargo_held", pallet=pal, dest=dest, mono=mono))["events"]
    return ev


def life2_probe_extra(ev):
    return {"events": ev, "missions_run": [
        {"pallet_id": "P-4", "destination_id": "D-1", "cancel_reply": {"t": "ack", "request_id": "r4", "mission_id": "m1"}},
        {"pallet_id": "P-4", "destination_id": "D-1"}, {"pallet_id": "P-5", "destination_id": "D-2"}]}


def test_life2_cargo_held_waived_only_for_the_planned_targets_sent_after_the_cancel():
    rows = [_row("m1", "P-4", "cancelled", "operator")]

    def rej(ev):
        s = fsess("life2_random_m_held", rows, ev)
        s["probe"] = life2_probe_extra(ev)
        return [x["attempt_request_id"] for x in V.rv4_failures(s) if x["outcome"] == "rejected"]
    assert rej(life2_events()) == []
    assert rej(life2_events(targets=(("P-4", "D-1"), ("P-3", "D-3")))) == ["r6"]  # not the planned second target
    assert rej(life2_events(sent_before_cancel=True)) == ["r5"]  # sent before the cancel's ack


def test_plant_gate_refuses_a_missing_pallet_mass_kg():
    sess = {"a": {"result": plant_result("x", pallet_mass={"engine_usd_kg": {}})},
            "b": {"result": plant_result("x", pallet_mass={"engine_usd_kg": {}})}}
    assert V.plant_check(sess, {"a": "line_m", "b": "line_m"}, MANIFEST)["ok"] is False


def test_a_plant_failure_sends_the_reading_to_the_lead_even_when_a_criterion_holds():
    combined = {"criteria": {"lifecycle_2": False}, "held_by": ["lifecycle_2"], "to_lead_by": [], "outcome": "hold"}
    out = V.rv4_outcome(combined, plant_ok=False)
    assert out["outcome"] == "to_lead" and "plant" in out["to_lead_by"] and out["judged"] is False
    assert V.rv4_outcome(combined, plant_ok=True)["outcome"] == "hold"


def fam_sess(dest="D-2", records=None):
    rec = records if records is not None else {"m2": {"tracking_failure": {"tracking": {"failure": "cross_track"}}}}
    return {"run": "/nonexistent", "records": rec,
            "rows": [{"mission_id": "m2", "pallet_id": "P-5", "destination_id": dest}]}


FAIL = {"mission": "m2", "reason": "Tracking failed in transport: pos=1.6"}


def test_family_346_extractor_errors_are_unclassifiable(monkeypatch):
    def boom(run, mission):
        raise IndexError("list index out of range")
    monkeypatch.setattr(V, "_nf_measure", lambda: boom)
    assert V.family_346(fam_sess(), FAIL, "random_m")["class"] == "unclassifiable"


def test_family_346_needs_the_destination_on_record(monkeypatch):
    monkeypatch.setattr(V, "_nf_measure", lambda: (lambda run, m: {"flip_yaw_peak_rad": -0.2,
                                                                   "back_within_0_05_s": None}))
    assert V.family_346(fam_sess(), FAIL, "random_m")["class"] == "family"
    assert V.family_346(fam_sess(dest=None), FAIL, "random_m")["class"] == "unclassifiable"
    assert V.family_346(fam_sess(), FAIL, "line_m")["class"] == "unclassifiable"
    assert V.family_346(fam_sess(records={"m2": {}}), FAIL, "random_m") is None  # not a cross_track: not in the column


# --- Codex implementation review round 2 ------------------------------------------------------------------------------
def test_a_row_refusal_without_its_reject_attempt_is_kept_and_marks_the_record_incomplete():
    rows = [_row("m1", "P-1", "done"), _row("m2", "P-2", "done"),
            {**_row(None, "P-3", None, start="rejected"), "reject_reason": "busy_x", "destination_id": "D-3"}]
    for r, d in zip(rows[:2], ("D-1", "D-2")):
        r["destination_id"] = d
    ev = probe(att("r2", 1, "ack", pallet="P-1", dest="D-1"), att("r3", 2, "ack", pallet="P-2", dest="D-2", mono=2))
    f = V.rv4_failures(fsess("ab_line_m_s0_B", rows, ev["events"]))
    assert {x["outcome"] for x in f} == {"start_unknown", "rejected"}


def test_a_matched_row_refusal_is_counted_once_and_an_unmatched_one_kept():
    rows = [{**_row(None, "P-1", None, start="rejected"), "reject_reason": "busy_x", "destination_id": "D-1"},
            {**_row(None, "P-2", None, start="rejected"), "reject_reason": "busy_x", "destination_id": "D-2"}]
    ev = probe(att("r2", 1, "reject", "busy_x", pallet="P-1", dest="D-1"))
    f = V.rv4_failures(fsess("ab_line_m_s0_B", rows, ev["events"]))
    rej = [x for x in f if x["outcome"] == "rejected"]
    assert len(rej) == 2 and any(x.get("attempt_request_id") == "r2" for x in rej)
    assert any(x["outcome"] == "start_unknown" for x in f)


def test_conflicting_replies_for_one_request_are_incomplete_and_keep_the_reject():
    ev = probe(att("r2", 1, "reject", "not_stopped"))["events"]
    ev.append({**ev[-1], "t": "ack", "reason": None, "mono": 1.3})
    a = V.submit_attempts({"events": ev})
    assert a["complete"] is False
    f = V.rv4_failures(fsess("ab_line_m_s0_B", [_row("m1", "P-1", "done")], ev))
    assert any(x["outcome"] == "rejected" and x["attempt_request_id"] == "r2" for x in f)


def test_request_id_and_the_queue_item_are_required():
    no_rid = probe(att("r2", 1, "reject", "not_stopped"), att("r3", 2, "ack", mono=2))
    for e in no_rid["events"][:2]:
        e["request_id"] = None
    assert V.submit_attempts(no_rid)["complete"] is False
    queue = probe(att("q4", 1, "reject", "not_stopped"), att("q5", 2, "ack", mono=2))
    for e in queue["events"]:
        e["path"] = "queue"  # a queue attempt without its item
    a = V.submit_attempts(queue)
    assert a["complete"] is False and a["retries"] == []


def test_life3_plant_and_safety_exemption_only_before_any_submit(tmp_path):
    sess = {"a": {"result": plant_result("x"), "name": "a"},
            "life3_random_m_reloc": {"result": None, "name": "life3_random_m_reloc", "rows": [],
                                     "probe": {"events": []}, "records": {"m1": {}}, "entries": []}}
    assert V.rv4_plant_skip(sess, life3_ended_per_record=True) == set()  # a mission folder: submitted
    sess["life3_random_m_reloc"]["records"] = {}
    assert V.rv4_plant_skip(sess, life3_ended_per_record=True) == {"life3_random_m_reloc"}
    assert V.rv4_plant_skip(sess, life3_ended_per_record=False) == set()


# --- Codex implementation review round 3 ------------------------------------------------------------------------------
def test_an_exempted_or_superseded_reject_does_not_stand_for_another_submissions_refusal():
    rows = [{**_row("m1", "P-1", "done"), "destination_id": "D-1"},
            {**_row(None, "P-1", None, start="rejected"), "reject_reason": "cargo_held", "destination_id": "D-1"}]
    ev = probe(att("r2", 1, "reject", "not_stopped"), att("r3", 2, "ack", mono=2))  # the later refusal's events lost
    f = V.rv4_failures(fsess("ab_line_m_s0_B", rows, ev["events"]))
    assert any(x["outcome"] == "rejected" and x.get("reject_reason", x.get("reason")) == "cargo_held" for x in f)
    assert any(x["outcome"] == "start_unknown" for x in f)


def test_a_duplicate_send_keeps_the_seen_reject_and_orphans_count_once():
    ev = probe(att("r2", 1, "reject", "busy_x"))["events"]
    ev += [{**ev[0], "mono": 1.2}, {**ev[1], "t": "ack", "reason": None, "mono": 1.4}]
    f = V.rv4_failures(fsess("ab_line_m_s0_B", [_row("m1", "P-1", "done")], ev))
    assert [x.get("attempt_request_id") for x in f if x["outcome"] == "rejected"] == ["r2"]
    orphan = probe(att("r2", 1, "reject", "busy_x", sent=False))["events"] * 2
    f2 = V.rv4_failures(fsess("ab_line_m_s0_B", [_row("m1", "P-1", "done")], orphan))
    assert [x.get("attempt_request_id") for x in f2 if x["outcome"] == "rejected"] == ["r2"]


def test_retries_are_listed_only_for_a_complete_record():
    rows = [{**_row("m1", "P-1", "done"), "destination_id": "D-1"}, {**_row("m2", "P-2", "done"),
                                                                       "destination_id": "D-2"}]
    ev = probe(att("r2", 1, "reject", "not_stopped"), att("r3", 2, "ack", mono=2))  # m2's ack missing
    s = fsess("ab_line_m_s0_B", rows, ev["events"])
    fails = V.rv4_failures(s)
    assert V.rv4_retries(s, fails) == []
    assert any(x["outcome"] == "rejected" and x["attempt_request_id"] == "r2" for x in fails)


# --- Codex implementation review round 4 ------------------------------------------------------------------------------
def test_attempt_seq_against_the_send_order_is_incomplete():
    ev = probe(att("r3", 2, "ack", mono=1.0), att("r2", 1, "reject", "not_stopped", mono=2.0))
    a = V.submit_attempts(ev)
    assert a["complete"] is False and a["retries"] == []


def test_a_row_level_start_unknown_does_not_drop_the_retry_list():
    rows = [{**_row("m1", "P-1", "done"), "destination_id": "D-1"},
            {**_row(None, "P-2", None, start="unknown"), "reject_reason": "busy", "destination_id": "D-2"}]
    ev = probe(att("r2", 1, "reject", "not_stopped"), att("r3", 2, "ack", mono=2),
               att("r5", 3, "reject", "busy", pallet="P-2", dest="D-2", mono=3))
    s = fsess("ab_line_m_s0_B", rows, ev["events"])
    fails = V.rv4_failures(s)
    assert not any(x.get("why") for x in fails)  # the record is complete
    assert [r["reject_request_id"] for r in V.rv4_retries(s, fails)] == ["r2"]


def test_an_attempt_and_its_orphan_copy_count_once():
    ev = probe(att("r2", 1, "reject", "busy_x"))["events"]
    copy = {k: v for k, v in ev[1].items() if k != "attempt_seq"}
    ev.append(copy)
    f = V.rv4_failures(fsess("ab_line_m_s0_B", [_row("m1", "P-1", "done")], ev))
    assert [x.get("attempt_request_id") for x in f if x["outcome"] == "rejected"] == ["r2"]
