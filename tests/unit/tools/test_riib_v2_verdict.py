"""v2 main batch verdict pieces (docs/plans/2026-11-08-live-riib-defaults-v2.md 3.3): the longest ok=false run's end
rules, stalls with the C2 replan steps apart and the previous row's split, the four pooled comparisons, trace breaks."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import riib_v2_verdict as V  # noqa: E402


def test_the_longest_false_run_ends_at_the_next_true_step_a_gap_or_the_window_end():
    steps = {0: (0.0, 0, True), 1: (1.0, 0, False), 2: (2.0, 0, False), 3: (3.0, 0, True),  # 1 -> 3: 2 s
             4: (4.0, 0, False), 5: (5.0, 0, False), 7: (20.0, 0, False),  # gap after 5: ends at 5 (1 s), 7 alone
             8: (21.0, 0, False), 9: (24.0, 0, False)}  # 7..9 false to the window's end: 24 - 20 = 4 s
    assert V.longest_false(steps, 0.0, 100.0) == pytest.approx(4.0)
    assert V.longest_false(steps, 0.0, 6.0) == pytest.approx(2.0)


def trace(S, flags=None):
    names = ["live_camera", "video", "render_arg", "scan", "rendered", "capture", "pocket", "far"]
    S = np.asarray(S, float)
    return {"S": S, "live_step": np.arange(len(S), dtype=float),
            "flags": np.asarray(flags if flags is not None else [math.nan] * len(S), float), "flag_names": names}


def test_stalls_split_by_the_previous_row_and_c2_steps_apart():
    rendered, capture = 1 << 4, (1 << 5) | (1 << 4)
    tr = trace([0.0, 0.03, 0.40, 0.43, 0.80, 0.83, 1.30], [math.nan, rendered, math.nan, capture, math.nan, 0, math.nan])
    out = V.stall_rows(tr, 0.0, 10.0, exclude={6})
    assert (out["total"], out["rendered"], out["capture"], out["other"], out["c2_excluded"]) == (2, 1, 1, 0, 1)
    assert V.stall_rows(tr, 0.5, 10.0, exclude=set())["total"] == 2  # S[k] in the interval: steps 4 and 6


def test_the_four_pooled_rules():
    a = {"sim": 300.0, "wall": 1000.0, "valid_wall": 1000.0, "false_wall": 50.0, "longest": 10.0, "stalls": 3}
    ok = {"sim": 292.0, "wall": 1000.0, "valid_wall": 1000.0, "false_wall": 60.0, "longest": 14.0, "stalls": 5}
    out = V.arm_rules(a, ok)
    assert out["pass"] and out["F"]["pass"] and out["R"]["pass"] and out["longest"]["pass"] and out["stalls"]["pass"]
    assert not V.arm_rules(a, {**ok, "sim": 290.0})["F"]["pass"]  # 0.290 < 0.97 x 0.300
    assert not V.arm_rules(a, {**ok, "false_wall": 61.0})["R"]["pass"]  # 0.061 > 0.05 + 0.01
    assert not V.arm_rules(a, {**ok, "longest": 14.1})["longest"]["pass"]  # > 1.2 x 10 + 2
    assert not V.arm_rules(a, {**ok, "stalls": 6})["stalls"]["pass"]  # > 3 + 2


def test_a_trace_break_inside_the_interval_is_missing():
    tr = trace([0.0, 0.1, 0.2, 0.3])
    assert not V.trace_missing(tr, 0.0, 1.0)
    tr["live_step"] = np.array([0.0, 1.0, 3.0, 4.0])
    assert V.trace_missing(tr, 0.0, 1.0) and not V.trace_missing(tr, 0.15, 1.0)


def test_pooled_r_uses_the_valid_intervals_not_the_wall_with_gaps():
    # Codex: R x wall_s overcounts when allowed gaps sit inside wall_s
    a = {"sim": 300.0, "wall": 1000.0, "valid_wall": 1000.0, "false_wall": 90.6, "longest": 1.0, "stalls": 0}
    b = {"sim": 300.0, "wall": 1009.0, "valid_wall": 1000.0, "false_wall": 100.45, "longest": 1.0, "stalls": 0}
    assert V.arm_rules(a, b)["R"]["pass"]  # 0.10045 <= 0.0906 + 0.01


def test_the_lifecycle_order_needs_each_step_after_the_previous():
    ev = [{"event": "reached", "label": "x"}, {"event": "reply", "kind": "rearm", "t": "ack"},
          {"event": "reply", "kind": "mission_resume", "t": "ack"}]
    ok, order = V.in_order(ev, [("x", V._label("x")), ("rearm", V._reply("rearm")), ("resume", V._reply("mission_resume"))])
    assert ok and [j for _, j in order] == [0, 1, 2]
    ok2, _ = V.in_order(list(reversed(ev)), [("x", V._label("x")), ("rearm", V._reply("rearm"))])
    assert not ok2  # the rearm before the label does not count


def test_expected_missions_must_have_their_files_and_contact_field(tmp_path):
    import json as _json

    run = tmp_path / "run"
    (run / "missions" / "m1").mkdir(parents=True)
    (run / "spool").mkdir()
    (run / "spool" / "manifest.json").write_text(_json.dumps({"complete": True}))
    for f in ("result.json", "records.json"):
        (run / "missions" / "m1" / f).write_text("{}")
    res = {"forbidden_pocket_contacts": [], "session_neighbour_gap": {}, "session_rest_largest": {},
           "obstacle_layer": {"unpermitted_dir_entries": 0},
           "mission_results": [{"mission_id": "m1"}, {"mission_id": "m2"}]}
    sess = {"run": run, "result": res, "rows": [], "probe": {},
            "records": {"m1": {"forbidden_pocket_contacts": []}}}
    out = V.safety(sess)
    assert "undecidable" in out and out["undecidable"]["missing_missions"] == ["m2"]
    res["mission_results"] = [{"mission_id": "m1"}]
    sess["records"] = {"m1": {}}  # parsed, but without the contact field
    out = V.safety(sess)
    assert "undecidable" in out and out["undecidable"]["records_without_contacts"] == ["m1"]
    sess["records"] = {"m1": {"forbidden_pocket_contacts": []}}
    assert V.safety(sess)["pass"] is True


def _confirm_ok():
    w = {"phase_reached": True, "mission_id": "m1", "start_sim_s": 19.2, "end_sim_s": 24.2, "end_reason": "sim_time",
         "mode_at_end": "STOPPED", "mission_at_end": {"mission_id": "m1", "status": "paused", "phase": "approach",
                                                      "pallet_id": "P-4", "destination_id": "D-1"}}
    rows = [{"pallet_id": "P-4", "destination_id": "D-1", "mission_id": "m1"},
            {"pallet_id": "P-5", "destination_id": "D-2", "mission_id": "m2"}]
    runs = [{"pallet_id": "P-4", "destination_id": "D-1", "stop_wait": w}, {"pallet_id": "P-5", "destination_id": "D-2"}]
    steps = [{"step": i, "tick_t": 19.0 + i * 0.1, "mode": "STOPPED" if 19.1 <= 19.0 + i * 0.1 <= 24.3 else "AUTO",
              "speed_mps": 0.0} for i in range(60)]
    return {"lc": True, "safe": True, "rows": rows, "run": "x", "probe": {"missions_run": runs},
            "args": {"ARM": "B", "SEED": "0"}, "slam_choice": [MAP], "entries": steps,
            "mission_results": {"m1": ("P-4", "D-1"), "m2": ("P-5", "D-2")}, "log_complete": True,
            "truth_track": [(19.2 + k * 0.5, 0.0, 0.0) for k in range(11)], "image_id": IMG}


IMG = "sha256:aaa"


MAP = "localization (registry: random_m_team_spec_quantized_0facac3eea66)"


TG = (("P-4", "D-1"), ("P-5", "D-2"))


def test_a_confirmation_session_needs_lifecycle_4_safety_the_cap_and_a_different_m2_target(monkeypatch):
    # docs/plans/2026-11-09-live-riib-v2-lifecycle4-confirmation.md 4
    monkeypatch.setattr(V, "lifecycle", lambda s, w: {"pass": s["lc"]})
    monkeypatch.setattr(V, "safety", lambda s: {"pass": s["safe"]})
    monkeypatch.setattr(V, "c2_steps", lambda s: (set(), []))
    assert V.confirm_session(_confirm_ok(), TG, MAP, IMG)["pass"] is True
    assert confirm_session_({**_confirm_ok(), "lc": False}, TG)["pass"] is False
    assert confirm_session_({**_confirm_ok(), "safe": False}, TG)["pass"] is False
    same = _confirm_ok()
    same["rows"][1] = {"pallet_id": "P-4", "destination_id": "D-1", "mission_id": "m2"}
    assert confirm_session_(same, TG)["pass"] is False
    short = _confirm_ok()
    short["rows"] = short["rows"][:1]
    assert confirm_session_(short, TG)["pass"] is False
    monkeypatch.setattr(V, "c2_steps", lambda s: ({7}, []))
    monkeypatch.setattr(V, "c2_gaps", lambda s, ks: [{"step": 7, "gap_s": 2.5, "over_cap": True}])
    assert confirm_session_(_confirm_ok(), TG)["pass"] is False


@pytest.mark.parametrize("change", [
    ("mission_at_end", "phase", "insert"),  # the STOP held the mission in another phase
    ("mission_at_end", "pallet_id", "P-5"),
    ("mission_at_end", "mission_id", "m2"),
    (None, "mode_at_end", "AUTO"),
    (None, "end_reason", "timeout"),
    (None, "end_sim_s", 23.0),  # the registered 5 s wait not met
    (None, "mission_id", "m2"),
])
def test_the_stop_must_hold_m1_in_approach_for_the_registered_wait(monkeypatch, change):
    monkeypatch.setattr(V, "lifecycle", lambda s, w: {"pass": True})
    monkeypatch.setattr(V, "safety", lambda s: {"pass": True})
    monkeypatch.setattr(V, "c2_steps", lambda s: (set(), []))
    sess = _confirm_ok()
    w = sess["probe"]["missions_run"][0]["stop_wait"]
    where, key, value = change
    (w[where] if where else w)[key] = value
    out = confirm_session_(sess, TG)
    assert out["pass"] is False and out["stop"]["pass"] is False


def test_the_target_is_checked_on_the_request_the_result_and_the_row(monkeypatch):
    # independent review P2-2: the row fills the probe's request first, the result can name another pallet
    monkeypatch.setattr(V, "lifecycle", lambda s, w: {"pass": True})
    monkeypatch.setattr(V, "safety", lambda s: {"pass": True})
    monkeypatch.setattr(V, "c2_steps", lambda s: (set(), []))
    sess = _confirm_ok()
    sess["mission_results"] = {"m1": ("P-4", "D-1"), "m2": ("P-4", "D-1")}
    assert V.confirm_session(sess, TG, MAP, IMG)["target_ok"] is False
    sess = _confirm_ok()
    sess["mission_results"] = {"m1": ("P-4", "D-1")}  # m2's result missing
    assert V.confirm_session(sess, TG, MAP, IMG)["target_ok"] is False
    sess = _confirm_ok()
    sess["probe"]["missions_run"][1]["pallet_id"] = "P-3"
    assert V.confirm_session(sess, TG, MAP, IMG)["target_ok"] is False


def test_the_stop_hold_reports_motion_during_the_wait(monkeypatch):
    monkeypatch.setattr(V, "lifecycle", lambda s, w: {"pass": True})
    monkeypatch.setattr(V, "safety", lambda s: {"pass": True})
    monkeypatch.setattr(V, "c2_steps", lambda s: (set(), []))
    out = V.confirm_session(_confirm_ok(), TG, MAP, IMG)
    assert out["stop_hold"]["held"] is True and out["pass"] is True
    sess = _confirm_ok()
    sess["entries"][30]["speed_mps"] = 0.05  # a wheel command inside the wait
    assert V.confirm_session(sess, TG, MAP, IMG)["stop_hold"]["held"] is False
    sess = _confirm_ok()
    sess["entries"][30]["mode"] = "AUTO"
    assert V.confirm_session(sess, TG, MAP, IMG)["stop_hold"]["held"] is False
    sess = _confirm_ok()
    sess["truth_track"] = [(19.5, 0.0, 0.0), (22.0, 0.02, 0.0)]  # the truck moved 2 cm while STOPPED
    assert V.confirm_session(sess, TG, MAP, IMG)["stop_hold"]["held"] is False
    s = {"l4c_s0_B": {"pass": True, "arm": "B", "seed": "0", "stop_hold": {"held": False}},
         "l4c_s1_B": {"pass": True, "arm": "B", "seed": "1", "stop_hold": {"held": True}},
         "l4c_s0_A": {"pass": True, "arm": "A", "seed": "0"}, "l4c_s1_A": {"pass": True, "arm": "A", "seed": "1"}}
    out = V.confirm_verdict({"judged": ["l4c_s0_B", "l4c_s1_B"], "controls": ["l4c_s0_A", "l4c_s1_A"]}, s)
    assert out["outcome"] == "to_lead" and out["stop_not_held"] == ["l4c_s0_B"]


def confirm_session_(sess, tg):
    return V.confirm_session(sess, tg, MAP, IMG)


def test_a_session_on_another_map_choice_is_undecidable(monkeypatch):
    monkeypatch.setattr(V, "lifecycle", lambda s, w: {"pass": True})
    monkeypatch.setattr(V, "safety", lambda s: {"pass": True})
    monkeypatch.setattr(V, "c2_steps", lambda s: (set(), []))
    for choices in ([], ["mapping (fallback: no registry row)"], [MAP, "mapping (fallback: x)"]):
        out = V.confirm_session({**_confirm_ok(), "slam_choice": choices}, TG, MAP, IMG)
        assert out["pass"] is False and out["undecidable"] == "slam_choice"


def _r(arm, seed, ok=True):
    return {"pass": ok, "arm": arm, "seed": seed, "stop_hold": {"held": True}}


def test_the_confirmation_passes_on_both_judged_sessions_and_sends_a_failed_control_to_the_lead():
    s = {"l4c_s0_B": _r("B", "0"), "l4c_s1_B": _r("B", "1"), "l4c_s0_A": _r("A", "0"), "l4c_s1_A": _r("A", "1", False)}
    plan = {"judged": ["l4c_s0_B", "l4c_s1_B"], "controls": ["l4c_s0_A", "l4c_s1_A"]}
    out = V.confirm_verdict(plan, s)
    assert out["judged_pass"] is True and out["failed_controls"] == ["l4c_s1_A"]
    assert out["outcome"] == "to_lead"  # an A failure is for the lead to explain before any promotion
    s["l4c_s1_A"]["pass"] = True
    assert V.confirm_verdict(plan, s)["outcome"] == "pass"
    s["l4c_s0_B"]["pass"] = False
    assert V.confirm_verdict(plan, s)["outcome"] == "not_passed"
    assert V.confirm_verdict({"judged": [], "controls": []}, {})["outcome"] == "not_passed"


@pytest.mark.parametrize("plan", [
    {"judged": ["l4c_s0_B"], "controls": []},  # a judged seed and the controls missing
    {"judged": ["l4c_s0_B", "l4c_s0_B"], "controls": ["l4c_s0_A", "l4c_s1_A"]},  # a duplicate
    {"judged": ["l4c_s0_B", "l4c_s1_B"], "controls": ["l4c_s0_A", "l4c_s0_B"]},  # in both groups
    {"judged": ["l4c_s0_B", "l4c_s0_A"], "controls": ["l4c_s1_B", "l4c_s1_A"]},  # arms swapped
])
def test_the_confirmation_needs_the_registered_composition(plan):
    s = {"l4c_s0_B": _r("B", "0"), "l4c_s1_B": _r("B", "1"), "l4c_s0_A": _r("A", "0"), "l4c_s1_A": _r("A", "1")}
    out = V.confirm_verdict(plan, s)
    assert out["outcome"] == "not_passed" and out["composition_ok"] is False


def test_a_session_on_another_bridge_image_is_undecidable(monkeypatch):
    monkeypatch.setattr(V, "lifecycle", lambda s, w: {"pass": True})
    monkeypatch.setattr(V, "safety", lambda s: {"pass": True})
    monkeypatch.setattr(V, "c2_steps", lambda s: (set(), []))
    assert V.confirm_session(_confirm_ok(), TG, MAP, IMG)["pass"] is True
    for got in ("sha256:bbb", None):
        out = V.confirm_session({**_confirm_ok(), "image_id": got}, TG, MAP, IMG)
        assert out["pass"] is False and out["undecidable"] == "image_id"
    for expected in (None, ""):  # independent review round 2 P2-2: an empty expectation never switches the check off
        with pytest.raises(ValueError):
            V.confirm_session(_confirm_ok(), TG, MAP, expected)


@pytest.mark.parametrize("change", ["no_truth", "truth_outside", "one_step", "log_incomplete", "step_gap"])
def test_missing_stop_evidence_is_not_a_hold(monkeypatch, change):
    # independent review round 2 P2-1: missing evidence is told apart from a hold and goes to the lead
    monkeypatch.setattr(V, "lifecycle", lambda s, w: {"pass": True})
    monkeypatch.setattr(V, "safety", lambda s: {"pass": True})
    monkeypatch.setattr(V, "c2_steps", lambda s: (set(), []))
    sess = _confirm_ok()
    if change == "no_truth":
        sess["truth_track"] = None
    elif change == "truth_outside":
        sess["truth_track"] = [(10.0, 0.0, 0.0), (30.0, 0.0, 0.0)]
    elif change == "one_step":
        sess["entries"] = [e for e in sess["entries"] if e["step"] == 30]
    elif change == "log_incomplete":
        sess["log_complete"] = False
    else:
        sess["entries"] = [e for e in sess["entries"] if e["step"] != 30]
    hold = V.confirm_session(sess, TG, MAP, IMG)["stop_hold"]
    assert hold["held"] is None
    s = {"l4c_s0_B": {"pass": True, "arm": "B", "seed": "0", "stop_hold": hold},
         "l4c_s1_B": {"pass": True, "arm": "B", "seed": "1", "stop_hold": {"held": True}},
         "l4c_s0_A": {"pass": True, "arm": "A", "seed": "0"}, "l4c_s1_A": {"pass": True, "arm": "A", "seed": "1"}}
    out = V.confirm_verdict({"judged": ["l4c_s0_B", "l4c_s1_B"], "controls": ["l4c_s0_A", "l4c_s1_A"]}, s)
    assert out["outcome"] == "to_lead" and out["stop_not_held"] == ["l4c_s0_B"]


# --- the new-snapshot re-comparison (docs/plans/2026-11-10-live-riib-v2-rerun-new-snapshot.md 2) ---------------------
def _row(mid, pallet, outcome, reason=None):
    return {"mission_id": mid, "pallet_id": pallet, "outcome": outcome, "reason": reason}


def _sess(name, rows, records=None, occ=False):
    return {"name": name, "rows": rows, "records": records or {}, "result": {"arguments": {"occupied_dumps": occ}}}


def test_the_expected_ends_are_registered_and_narrow():
    shadow = _sess("c2_line_m_shadow", [_row("m1", "P-1", "failed", "Tracking timeout in observe")],
                   {"m1": {"observe_end_stands": [{"kind": "near", "outcome": "shadow"}]}})
    assert V.rv3_failures(shadow) == []
    no_stand = _sess("c2_line_m_shadow", [_row("m1", "P-1", "failed", "Tracking timeout in observe")], {"m1": {}})
    assert [f["mission"] for f in V.rv3_failures(no_stand)] == ["m1"]  # not a shadow firing: a failure (Codex P3)
    other = _sess("c2_line_m_shadow", [_row("m1", "P-1", "failed", "Tracking timeout in transport")],
                  {"m1": {"observe_end_stands": [{"kind": "near", "outcome": "shadow"}]}})
    assert len(V.rv3_failures(other)) == 1  # another reason in the same session is not waived
    life2 = _sess("life2_random_m_held", [_row("m1", "P-4", "cancelled", "operator"), _row(None, "P-4", None),
                                          _row(None, "P-5", None)])
    assert V.rv3_failures(life2) == []  # the cancel and the two refused resubmits
    l4 = _sess("l4_s0_B", [_row("m1", "P-4", "cancelled", "operator"), _row("m2", "P-5", "failed", "x")])
    assert [f["mission"] for f in V.rv3_failures(l4)] == ["m2"]  # m1's cancel is expected, m2's failure is not
    queue = _sess("ab_line_m_s0_A", [_row("m1", "P-1", "done"), _row("m2", "P-2", "done"), _row(None, None, None)])
    assert [f["outcome"] for f in V.rv3_failures(queue)] == ["not_run"]  # a three-mission queue stopped early
    reloc = _sess("life3_random_m_reloc", [_row(None, None, None, "probe_error: launcher_ended")])
    assert V.rv3_failures(reloc, life3_ended_per_record=True) == []
    assert len(V.rv3_failures(reloc, life3_ended_per_record=False)) == 1


def test_failures_are_disposed_b_only_a_only_or_both():
    fails = {"ab_line_m_s0_B": [{"mission": "m2"}], "ab_line_m_s1_A": [{"mission": "m1"}],
             "ab_random_m_s0_A": [{"mission": "m3"}], "ab_random_m_s0_B": [{"mission": "m3"}],
             "boxed_line_m": [{"mission": "m1"}], "l4_s1_A": [{"mission": "m2"}]}
    partners = {"ab_line_m_s0_B": "ab_line_m_s0_A", "ab_line_m_s1_A": "ab_line_m_s1_B",
                "ab_random_m_s0_A": "ab_random_m_s0_B", "ab_random_m_s0_B": "ab_random_m_s0_A",
                "boxed_line_m": None, "l4_s1_A": "l4_s1_B"}
    arms = {"ab_line_m_s0_B": "B", "ab_line_m_s1_A": "A", "ab_random_m_s0_A": "A", "ab_random_m_s0_B": "B",
            "boxed_line_m": "B", "l4_s1_A": "A"}
    out = V.rv3_dispose(fails, partners, arms)
    assert out["b_only"] == ["ab_line_m_s0_B", "boxed_line_m"]
    assert out["a_only"] == ["ab_line_m_s1_A", "l4_s1_A"]
    assert out["both"] == ["ab_random_m_s0_A", "ab_random_m_s0_B"]


def _main_ok():
    return {"pace": {"verdict": "pass"}, "c2_cap": {"pass": True},
            "safety": {"x": {"pass": True}, "life3_random_m_reloc": {"not_applied": "reloc_needs_operator"}},
            "backoff": {"boxed_line_m": [{"firing": True}], "boxed_random_m": [{"firing": False}]},
            "c2_function": {"c2_line_m_on": [{"firing": True}], "c2_line_m_shadow": [{"firing": False}]},
            "lifecycle": {"1": {"pass": True}, "2": {"pass": True}, "3": {"outcome": "ended_per_record", "pass": False}},
            "pairs": [{"sessions": {"A": {"F": 0.2999}, "B": {"F": 0.2999}}}]}


def _conf_ok():
    return {"judged_pass": True, "outcome": "pass"}


def _disp(**kw):
    return {"b_only": [], "a_only": [], "both": [], **kw}


def test_the_combined_outcome_needs_every_criterion_and_keeps_the_leads_cases():
    assert V.rv3_outcome(_main_ok(), _conf_ok(), _disp(), checks_ok=True)["outcome"] == "pass"
    assert V.rv3_outcome(_main_ok(), {**_conf_ok(), "judged_pass": False, "outcome": "not_passed"}, _disp(),
                         checks_ok=True)["outcome"] == "hold"
    assert V.rv3_outcome(_main_ok(), {**_conf_ok(), "outcome": "to_lead"}, _disp(), checks_ok=True)["outcome"] == "to_lead"
    assert V.rv3_outcome(_main_ok(), _conf_ok(), _disp(b_only=["ab_line_m_s0_B"]), checks_ok=True)["outcome"] == "hold"
    assert V.rv3_outcome(_main_ok(), _conf_ok(), _disp(both=["a", "b"]), checks_ok=True)["outcome"] == "to_lead"
    assert V.rv3_outcome(_main_ok(), _conf_ok(), _disp(a_only=["a"]), checks_ok=True)["outcome"] == "to_lead"
    assert V.rv3_outcome(_main_ok(), _conf_ok(), _disp(), checks_ok=False)["outcome"] == "to_lead"
    slow = {**_main_ok(), "pairs": [{"sessions": {"A": {"F": 0.2999}, "B": {"F": 0.2690}}}]}
    out = V.rv3_outcome(slow, _conf_ok(), _disp(), checks_ok=True)
    assert out["outcome"] == "to_lead" and out["pace_guard"] is False  # F < 0.27: the pace is not judged
    for broken in ({"pace": {"verdict": "hold"}}, {"c2_cap": {"pass": False}}, {"safety": {"x": {"pass": False}}},
                   {"backoff": {"boxed_line_m": [{"firing": False}]}}, {"c2_function": {"c2_line_m_on": []}},
                   {"lifecycle": {**_main_ok()["lifecycle"], "2": {"pass": False}}},
                   {"lifecycle": {**_main_ok()["lifecycle"], "3": {"outcome": "fail", "pass": False}}}):
        assert V.rv3_outcome({**_main_ok(), **broken}, _conf_ok(), _disp(), checks_ok=True)["outcome"] == "hold", broken


def test_the_dump_option_must_be_on_in_the_lifecycle_4_sessions_only():
    sessions = {"ab_line_m_s0_A": _sess("ab_line_m_s0_A", [], occ=False), "l4_s0_B": _sess("l4_s0_B", [], occ=True)}
    assert V.rv3_dump_args(sessions, {"l4_s0_B"}) == []
    sessions["ab_line_m_s0_A"]["result"]["arguments"]["occupied_dumps"] = True
    sessions["l4_s0_B"]["result"]["arguments"]["occupied_dumps"] = False
    assert V.rv3_dump_args(sessions, {"l4_s0_B"}) == ["ab_line_m_s0_A", "l4_s0_B"]
