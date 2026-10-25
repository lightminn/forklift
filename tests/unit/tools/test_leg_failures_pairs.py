"""The V4a seed-pair test (docs/plans/2026-10-25-leg-failures.md V4a): the one-sided signed-rank p by enumerating
every sign combination over average ranks, zero differences dropped."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

from leg_failures_pairs import judge, pairs_from_rows, signed_rank_p  # noqa: E402


def test_ties_take_average_ranks_over_every_sign_combination():
    # Codex plan round 4 P2: scipy's method="exact" gives 0.046875 here; the average-rank enumeration 0.0625
    assert signed_rank_p([0.5, 0.5, -0.5, 1, 1, 1]) == pytest.approx(4 / 64)


def test_without_ties_it_is_the_exact_distribution_and_zeros_are_dropped():
    assert signed_rank_p([1, 2, 3, 4, 5, 6]) == pytest.approx(1 / 64)
    assert signed_rank_p([0, 0, 1, 2, 3, 4, 5, 6]) == pytest.approx(1 / 64)
    assert signed_rank_p([-1, 2, 3, 4, 5, 6]) == pytest.approx(2 / 64)
    assert signed_rank_p([]) == 1.0


def test_the_judgement_rules():
    # pair: (main rate or None when no attempt, fix rate or None, main refusals, fix refusals)
    six_better = [(0.0, 1.0, 0, 0)] * 6
    assert judge(six_better)["verdict"] == "pass"
    # fewer than six usable pairs (a pair with no attempt on one side is not used): undecidable
    assert judge([(0.0, 1.0, 0, 0)] * 5 + [(None, 1.0, 0, 0)])["verdict"] == "undecidable"
    # more than two pairs worse
    worse = [(0.0, 1.0, 0, 0)] * 7 + [(1.0, 0.5, 0, 0)] * 3
    assert judge(worse)["verdict"] == "fail" and judge(worse)["worse_pairs"] == 3
    # refusals must not increase
    assert judge([(0.0, 1.0, 0, 1)] * 6)["verdict"] == "fail"
    # p >= 0.05: not adopted (zero differences count as usable pairs but leave the test)
    assert judge([(0.5, 1.0, 0, 0)] * 2 + [(0.5, 0.5, 0, 0)] * 6)["verdict"] == "fail"
    out = judge([(0.0, 1.0, 0, 0)] * 3 + [(1.0, 0.5, 0, 0)] * 2 + [(0.5, 0.5, 0, 0)] * 3)
    assert out["verdict"] == "fail" and out["usable_pairs"] == 8 and out["p"] >= 0.05


def test_a_session_counts_its_counted_reapproaches_by_the_docking_round_in_each_window(tmp_path):
    import json

    from leg_failures_pairs import session_counts

    md = tmp_path / "run" / "live/epoch_1/run/missions/m1"
    md.mkdir(parents=True)
    retries = [{"trigger": "a", "time_s": 90.0, "counted": False},
               {"trigger": "b", "time_s": 100.0, "counted": True, "plan": {"length_m": 2.0}},  # stood: no round
               {"trigger": "obstacle_replan", "time_s": 105.0, "counted": False, "plan": {"length_m": 2.0}},
               {"trigger": "d", "time_s": 110.0, "counted": True, "plan": {"length_m": 2.0}},  # arrived: round 120
               {"trigger": "b", "time_s": 130.0, "counted": True, "plan": {"refused": "dry_run"}}]
    (md / "records.json").write_text(json.dumps({"state": {"docking_retries": retries, "delivery_docking": {
        "rounds": [{"time_s": 95.0}, {"time_s": 120.0}]}}}))
    (md / "result.json").write_text(json.dumps({"result": {"ended_s": 140.0}}))
    assert session_counts(str(tmp_path / "run")) == [1, 3, 1]
    assert session_counts(str(tmp_path / "missing")) == [0, 0, 0]


def test_mission_outcomes_and_arm_names_for_the_reapproach_hold_judgement(tmp_path):
    """Re-approach hold plan V4a guard 2: each arm's done missions, its transport_recovery_after_docking endings and
    its hold exits, read from run folders named by the arms given (off / on)."""
    import json

    from leg_failures_pairs import collect, mission_totals

    def mission(run, m, outcome, reason=None, retries=()):
        md = tmp_path / run / "live/epoch_1/run/missions" / m
        md.mkdir(parents=True)
        (md / "records.json").write_text(json.dumps({"state": {"docking_retries": list(retries)}}))
        (md / "result.json").write_text(json.dumps({"result": {"outcome": outcome, "reason": reason, "ended_s": 9}}))

    mission("p_off_random_m_s0", "m1", "done")
    mission("p_off_random_m_s0", "m2", "failed", "transport_recovery_after_docking")
    mission("p_on_random_m_s0", "m1", "done", retries=[{"trigger": "h", "time_s": 3.0, "counted": False}])
    mission("p_on_random_m_s0", "m2", "done")
    rows = collect(str(tmp_path), "p", "random_m", range(1), arms=("off", "on"))
    tot = mission_totals(rows)
    assert tot["main"] == {"missions": 2, "done": 1, "recovery_after_docking": 1, "hold_exits": 0, "session_failed": 0}
    assert tot["fix"] == {"missions": 2, "done": 2, "recovery_after_docking": 0, "hold_exits": 1, "session_failed": 0}
    assert rows[0]["fix"] == [0, 0, 0]  # an uncounted h is no attempt


def test_a_session_cut_mid_mission_counts_its_last_mission_from_the_run_record(tmp_path):
    """Codex V4 hold review P2: a session that ended with the run failed (slam_link_failed) keeps its unfinished
    mission only in the run's result.json; its re-approaches count, once -- a run record that repeats the last mission
    folder's (a run that ended after its missions) adds nothing."""
    import json

    from leg_failures_pairs import session_counts, session_missions

    run = tmp_path / "s" / "live/epoch_1/run"
    md = run / "missions/m1"
    md.mkdir(parents=True)
    done_retries = [{"trigger": "b", "time_s": 10.0, "counted": True, "plan": {"length_m": 2.0}}]
    (md / "records.json").write_text(json.dumps({"state": {"docking_retries": done_retries, "delivery_docking": {
        "rounds": [{"time_s": 20.0}]}}}))
    (md / "result.json").write_text(json.dumps({"result": {"outcome": "done", "ended_s": 30.0}}))
    cut = [{"trigger": "b", "time_s": 50.0, "counted": True, "plan": {"length_m": 2.0}},
           {"trigger": "d", "time_s": 60.0, "counted": True, "plan": {"length_m": 2.0}}]
    (run / "result.json").write_text(json.dumps({"success": False, "failure_reason": "slam_link_failed: x",
                                                 "docking_retries": cut, "delivery_docking": {"rounds": []}}))
    assert session_counts(str(tmp_path / "s")) == [1, 3, 0]  # m1 1/1, the cut mission 0/2
    assert session_missions(str(tmp_path / "s"))["session_failed"] == 1
    # a finished run whose record repeats its last mission's: no extra mission
    (run / "result.json").write_text(json.dumps({"success": True, "docking_retries": done_retries,
                                                 "delivery_docking": {"rounds": [{"time_s": 20.0}]}}))
    assert session_counts(str(tmp_path / "s")) == [1, 1, 0]
    assert session_missions(str(tmp_path / "s"))["session_failed"] == 0


def test_the_probes_acked_submits_count_a_mission_cut_before_any_retry(tmp_path):
    """Codex V5 review P2-1: a session cut in m2 before D7 has a run record with the same (empty) retries as m1 --
    the mission count comes from the probe's acked submits against the mission folders, not from the retries."""
    import json

    from leg_failures_pairs import session_missions

    session = tmp_path / "s"
    md = session / "live/epoch_1/run/missions/m1"
    md.mkdir(parents=True)
    (md / "records.json").write_text(json.dumps({"state": {"docking_retries": []}}))
    (md / "result.json").write_text(json.dumps({"result": {"outcome": "done", "ended_s": 30.0}}))
    (session / "live/epoch_1/run/result.json").write_text(json.dumps(
        {"success": False, "failure_reason": "slam_link_failed: x", "docking_retries": []}))
    (session / "probe.json").write_text(json.dumps({"missions_run": [
        {"pallet_id": "P-4", "submit": {"t": "ack"}}, {"pallet_id": "P-5", "submit": {"t": "ack"}},
        {"pallet_id": "P-3", "submit": {"t": "reject"}}]}))
    out = session_missions(str(session))
    assert (out["missions"], out["done"], out["session_failed"]) == (2, 1, 1)
    # the same session that ended between missions (as many acked as folders): nothing added
    (session / "probe.json").write_text(json.dumps({"missions_run": [{"pallet_id": "P-4", "submit": {"t": "ack"}}]}))
    out = session_missions(str(session))
    assert (out["missions"], out["session_failed"]) == (1, 0)


def test_an_acked_mission_with_no_run_record_still_counts_as_unfinished(tmp_path):
    """Codex V5 2nd review P2: a session killed in m2 leaves no final run result.json; the acked m2 must still count
    as an unfinished mission, apart from whether the run record exists."""
    import json

    from leg_failures_pairs import session_missions, session_score

    session = tmp_path / "s"
    md = session / "live/epoch_1/run/missions/m1"
    md.mkdir(parents=True)
    (md / "records.json").write_text(json.dumps({"state": {"docking_retries": []}}))
    (md / "result.json").write_text(json.dumps({"result": {"outcome": "done", "ended_s": 30.0}}))
    (session / "probe.json").write_text(json.dumps({"missions_run": [
        {"pallet_id": "P-4", "submit": {"t": "ack"}}, {"pallet_id": "P-5", "submit": {"t": "ack"}}]}))
    out = session_missions(str(session))
    assert (out["missions"], out["done"], out["session_failed"]) == (2, 1, 1)
    assert session_score(str(session)) == 1


def test_each_failed_epoch_keeps_its_own_cut_mission_and_a_surplus_of_folders_is_inconsistent(tmp_path):
    """Codex V5 3rd review P2: two failed epochs each cut a mission -- each keeps its own retries and reason (one plan
    refused, one re-approach that arrived), not the last epoch's twice; more mission folders than acked submits is
    flagged inconsistent, and the V5 judgement will not decide on it."""
    import json

    from leg_failures_pairs import session_counts, session_missions

    session = tmp_path / "s"
    e1 = session / "live/epoch_1/run"
    (e1 / "missions/m1").mkdir(parents=True)
    (e1 / "missions/m1/records.json").write_text(json.dumps({"state": {"docking_retries": []}}))
    (e1 / "missions/m1/result.json").write_text(json.dumps({"result": {"outcome": "done", "ended_s": 30.0}}))
    (e1 / "result.json").write_text(json.dumps({"success": False, "failure_reason": "first",
        "docking_retries": [{"trigger": "b", "time_s": 40.0, "counted": True, "plan": {"refused": "dry_run"}}]}))
    e2 = session / "live/epoch_2/run"
    e2.mkdir(parents=True)
    (e2 / "result.json").write_text(json.dumps({"success": False, "failure_reason": "second",
        "docking_retries": [{"trigger": "b", "time_s": 10.0, "counted": True, "plan": {"length_m": 2.0}}],
        "delivery_docking": {"rounds": [{"time_s": 12.0}]}}))
    acks = [{"submit": {"t": "ack"}}] * 3
    (session / "probe.json").write_text(json.dumps({"missions_run": acks}))
    out = session_missions(str(session))
    assert (out["missions"], out["done"], out["session_failed"], out["inconsistent"]) == (3, 1, 2, False)
    assert out["reasons"] == ["first", "second"]
    assert session_counts(str(session)) == [1, 2, 1]
    # a folder surplus (four done folders over three acked submits) is inconsistent
    for k in (2, 3, 4):
        md = e2 / f"missions/m{k}"
        md.mkdir(parents=True)
        (md / "records.json").write_text(json.dumps({"state": {"docking_retries": []}}))
        (md / "result.json").write_text(json.dumps({"result": {"outcome": "done", "ended_s": 30.0}}))
    assert session_missions(str(session))["inconsistent"] is True


def test_a_mission_missing_a_file_is_bounded_best_on_off_and_worst_on_the_hold(tmp_path):
    """Codex V5 4th-6th review P2: a mission missing records.json, result.json or both is unknown there -- at the best
    bound (off) an unknown outcome is done, unknown records are clean and add no D7 failure or refusal; at the worst
    (the hold arm) an unknown outcome is not done, unknown records are a stall, a D7 failure when not done, one refusal
    and an unconfirmed contact. Without the probe's report, scheduled missions with no folder are unknown."""
    import json

    from leg_failures_pairs import session_missions, session_v5

    session = tmp_path / "s"
    run = session / "live/epoch_1/run"
    (run / "missions/m1").mkdir(parents=True)  # records with a re-approach that stood, no result.json
    (run / "missions/m1/records.json").write_text(json.dumps({"state": {"docking_retries": [
        {"trigger": "b", "time_s": 5.0, "counted": True, "plan": {"length_m": 2.0}}]}}))
    (run / "missions/m2").mkdir(parents=True)  # a done result, no records.json
    (run / "missions/m2/result.json").write_text(json.dumps({"result": {"outcome": "done", "ended_s": 9.0}}))
    (session / "probe.json").write_text(json.dumps({"missions_run": [{"submit": {"t": "ack"}}] * 3}))  # m3: nothing
    out = session_v5(str(session))
    assert out["unknown"] == 3 and out["inconsistent"] is False
    assert out["best"] == {"score": 2, "done": 3, "d7_failed": 0, "refusals": 0, "contacts": 0, "contact_unknown": 0}
    assert out["worst"] == {"score": 0, "done": 1, "d7_failed": 2, "refusals": 2, "contacts": 0, "contact_unknown": 2}
    assert session_missions(str(session))["unknown"] == 3
    # no probe report: the scheduled missions with no folder are unknown too
    (session / "probe.json").unlink()
    out = session_v5(str(session))
    assert out["unknown"] == 3 and out["worst"]["done"] == 1 and out["best"]["done"] == 3
    # a known forbidden contact counts on either bound
    (run / "missions/m2/records.json").write_text(json.dumps({"state": {"docking_retries": [],
                                                                       "forbidden_pocket_contacts": [{"t": 1}]}}))
    assert session_v5(str(session))["best"]["contacts"] == 1


def test_an_empty_mission_folder_is_an_unknown_mission_and_counts_against_the_schedule(tmp_path):
    """Codex V5 7th review P2: a folder with neither file is still a mission -- unknown at both bounds, counted against
    the acked and scheduled missions (three done folders and an empty fourth are inconsistent); after a folder with no
    records a failed run's record is not added as another mission (it cannot be told from that one)."""
    import json

    from leg_failures_pairs import session_v5

    session = tmp_path / "s"
    run = session / "live/epoch_1/run"
    for k in (1, 2, 3):
        md = run / f"missions/m{k}"
        md.mkdir(parents=True)
        (md / "records.json").write_text(json.dumps({"state": {"docking_retries": []}}))
        (md / "result.json").write_text(json.dumps({"result": {"outcome": "done", "ended_s": 9.0}}))
    (run / "missions/m4").mkdir()
    assert session_v5(str(session))["inconsistent"] is True
    (session / "probe.json").write_text(json.dumps({"missions_run": [{"submit": {"t": "ack"}}] * 3}))
    assert session_v5(str(session))["inconsistent"] is True
    # two done folders, an empty third and a failed run record: three missions, the third unknown
    for f in ("records.json", "result.json"):
        (run / "missions/m3" / f).unlink()
    (run / "missions/m4").rmdir()
    (run / "result.json").write_text(json.dumps({"success": False, "failure_reason": "x", "docking_retries": [
        {"trigger": "b", "time_s": 1.0, "counted": True, "plan": {"length_m": 2.0}}]}))
    out = session_v5(str(session))
    assert (out["inconsistent"], out["unknown"]) == (False, 1)
    assert (out["best"]["score"], out["worst"]["score"], out["worst"]["contact_unknown"]) == (3, 2, 1)


def test_a_forgotten_or_unanswered_submit_may_have_started_a_mission(tmp_path):
    """Codex V5 10th review P2: the probe keeps a submit whose reply it never saw as forgotten (or with no reply at
    all); the vehicle may have taken it, so it counts as a mission that may have started -- unknown -- and only an
    explicit reject does not."""
    import json

    from leg_failures_pairs import session_v5

    session = tmp_path / "s"
    for k in (1, 2):
        md = session / f"live/epoch_1/run/missions/m{k}"
        md.mkdir(parents=True)
        (md / "records.json").write_text(json.dumps({"state": {"docking_retries": []}}))
        (md / "result.json").write_text(json.dumps({"result": {"outcome": "done", "ended_s": 9.0}}))
    for third in ({"submit": {"t": "forgotten"}}, {}, {"submit": {"t": "reject", "reason": "busy"}},
                  {"submit": {"t": "reject", "reason": "stale"}}):  # unsure rejects too (Codex V5 11th review P2)
        (session / "probe.json").write_text(json.dumps({"missions_run": [{"submit": {"t": "ack"}}] * 2 + [third]}))
        out = session_v5(str(session))
        assert (out["inconsistent"], out["unknown"], out["worst"]["contact_unknown"]) == (False, 1, 1)
    (session / "probe.json").write_text(json.dumps({"missions_run": [{"submit": {"t": "ack"}}] * 2 + [
        {"submit": {"t": "reject", "reason": "invalid_target"}}]}))
    assert session_v5(str(session))["unknown"] == 0


def test_after_a_folder_with_no_records_the_run_record_is_told_apart_by_time(tmp_path):
    """Codex V5 8th review P2: m3 done at 30 s with no records.json, then a failed run record whose counted retry is at
    40 s -- a separate cut mission (a fourth: over the schedule, inconsistent); a retry within m3 (20 s) is m3's own;
    with no end time to compare it cannot be settled (inconsistent)."""
    import json

    from leg_failures_pairs import session_v5

    session = tmp_path / "s"
    run = session / "live/epoch_1/run"
    for k in (1, 2, 3):
        md = run / f"missions/m{k}"
        md.mkdir(parents=True)
        if k < 3:
            (md / "records.json").write_text(json.dumps({"state": {"docking_retries": []}}))
        (md / "result.json").write_text(json.dumps({"result": {"outcome": "done", "ended_s": 10.0 * k}}))

    def record(t):
        (run / "result.json").write_text(json.dumps({"success": False, "failure_reason": "x", "docking_retries": [
            {"trigger": "b", "time_s": t, "counted": True, "plan": {"length_m": 2.0}}]}))

    # without the probe's report any failed run record leaves the cut count unsettled (Codex V5 9th review P2: a
    # record with no retry but a docking round after m3 is a fourth mission too)
    (run / "result.json").write_text(json.dumps({"success": False, "failure_reason": "x", "docking_retries": [],
                                                 "delivery_docking": {"rounds": [{"time_s": 40.0}]}}))
    assert session_v5(str(session))["inconsistent"] is True
    # with three acked submits the count is settled; the retries' times tell a separate mission from m3's own
    (session / "probe.json").write_text(json.dumps({"missions_run": [{"submit": {"t": "ack"}}] * 3}))
    record(40.0)
    assert session_v5(str(session))["inconsistent"] is True  # a fourth mission over three acked
    record(20.0)
    out = session_v5(str(session))
    assert (out["inconsistent"], out["unknown"]) == (False, 1)
    (run / "missions/m3/result.json").unlink()
    (run / "missions/m3/result.json").write_text(json.dumps({"result": {"outcome": "done"}}))
    assert session_v5(str(session))["inconsistent"] is False  # no end time: not added, the acked count holds


def test_the_v5_score_counts_missions_done_with_no_stall_and_the_sign_test_is_exact(tmp_path):
    """Re-approach hold plan 2a (V5): a session's score is how many of its scheduled missions completed with no
    re-approach stall -- a stalled mission that recovered scores 0, a mission not reached or cut scores 0 (Codex V5
    review P2-2: failing earlier must not look like a win); the pair test is an exact one-sided sign test."""
    import json

    from leg_failures_pairs import session_score, sign_test_p

    session = tmp_path / "s"

    def mission(m, outcome, retries=(), rounds=()):
        md = session / "live/epoch_1/run/missions" / m
        md.mkdir(parents=True)
        (md / "records.json").write_text(json.dumps({"state": {"docking_retries": list(retries), "delivery_docking": {
            "rounds": [{"time_s": t} for t in rounds]}}}))
        (md / "result.json").write_text(json.dumps({"result": {"outcome": outcome, "ended_s": 100.0}}))

    b = {"trigger": "b", "time_s": 10.0, "counted": True, "plan": {"length_m": 2.0}}
    d = {"trigger": "d", "time_s": 20.0, "counted": True, "plan": {"length_m": 2.0}}
    mission("m1", "done")  # no D7 at all: clean
    mission("m2", "done", [b, d], [30.0])  # b stood (no round before d), d arrived: stalled, recovered -> 0
    mission("m3", "done", [b], [15.0])  # one re-approach that arrived: clean
    assert session_score(str(session), scheduled=3) == 2
    assert session_score(str(session), scheduled=4) == 2  # a mission never reached scores 0
    assert sign_test_p([1, 1, 1, 1, 1]) == pytest.approx(1 / 32)
    assert sign_test_p([1, -1, 0, 0]) == pytest.approx(3 / 4)
    assert sign_test_p([0, 0]) == 1.0


def test_the_v5_judgement_takes_off_at_its_best_and_the_hold_at_its_worst():
    """Re-approach hold plan 2a (V5): p < 0.05 on the score differences, the summed score not lower (Codex V5 2nd
    review P2: 20 wins 2->3 and 10 losses 3->0 pass the sign test with the total 160 -> 150), failures (scheduled minus
    done) on <= off + 6, D7 failures and refusals not increased, no contact on the hold arm and none unconfirmed;
    off is read at its best bound and the hold at its worst; an inconsistent session is undecidable."""
    from leg_failures_pairs import v5_judge

    def q(score, done=None, d7=0, ref=0, contacts=0, cu=0):
        return {"score": score, "done": score if done is None else done, "d7_failed": d7, "refusals": ref,
                "contacts": contacts, "contact_unknown": cu}

    def row(off, on, inconsistent=False, off_best=None, on_worst=None):
        return {"seed": 0, "v5": {"main": {"inconsistent": False, "unknown": 0, "best": off_best or off, "worst": off},
                                  "fix": {"inconsistent": inconsistent, "unknown": 0, "best": on,
                                          "worst": on_worst or on}}}

    # every stall recovered (done 3 on both arms), so only the summed score can reject it
    counter = [row(q(2, 3), q(3))] * 20 + [row(q(3), q(0, 3))] * 10 + [row(q(3), q(3))] * 30
    out = v5_judge(counter)
    assert out["p"] < 0.05 and (out["score_main"], out["score_fix"]) == (160, 150) and out["verdict"] == "fail"
    good = [row(q(2), q(3))] * 10 + [row(q(3), q(3))] * 50
    assert v5_judge(good)["verdict"] == "pass"
    # the score rose but done fell by 7 (> 6): the failure guard rejects; by 6 it holds
    worse_done = [row(q(2, 3), q(3))] * 10 + [row(q(3), q(3))] * 43 + [row(q(2, 3), q(2))] * 7
    assert v5_judge(worse_done)["verdict"] == "fail"
    assert v5_judge(worse_done[:-1] + [row(q(3), q(3))])["verdict"] == "pass"
    # Codex V5 3rd review: six recovering stalls turned into failures pass the margin; the D7 guard rejects
    convert = [row(q(2, 3), q(3))] * 10 + [row(q(2, 3), q(2, d7=1))] * 6 + [row(q(3), q(3))] * 44
    assert v5_judge(convert)["verdict"] == "fail"
    assert v5_judge(good[:-1] + [row(q(3), q(3, ref=1))])["verdict"] == "fail"
    assert v5_judge(good[:-1] + [row(q(3), q(3, contacts=1))])["verdict"] == "fail"
    # Codex V5 6th review P2: an unknown mission on the hold arm leaves its contact unconfirmed -- not adopted
    assert v5_judge(good[:-1] + [row(q(3), q(3), on_worst=q(2, 2, d7=0, ref=0, cu=1))])["verdict"] == "fail"
    # the bounds are what is read: off's best and the hold's worst (Codex V5 5th/6th review: five off results lost
    # read at the best bound are no win)
    lost = [row(q(3), q(3))] * 55 + [row(q(2, 2), q(3), off_best=q(3))] * 5
    assert v5_judge(lost)["wins"] == 0 and v5_judge(lost)["verdict"] == "fail"
    assert v5_judge(good[:-1] + [row(q(3), q(3), inconsistent=True)])["verdict"] == "undecidable"


def test_rates_from_counts_stay_exact_so_ties_hold():
    """Codex plan round 5 P2: 3/3 - 2/3 and 0/3 - 1/3 are the same size; in floats they were not (p 0.015625, pass)."""
    rows = [{"seed": k, "main": [2, 3, 0], "fix": [3, 3, 0]} for k in range(6)]
    rows.append({"seed": 6, "main": [1, 3, 0], "fix": [0, 3, 0]})
    out = judge(pairs_from_rows(rows))
    assert out["p"] == pytest.approx(8 / 128) and out["verdict"] == "fail"
    assert pairs_from_rows([{"seed": 0, "main": [0, 0, 1], "fix": [1, 2, 0]}])[0][:2] == (None, 0.5)
