"""M-a3a: mission commands and the lifecycle (docs/plans/2026-10-16-m-a3-mission-commands.md E1, E2, E4)."""

import pytest

from forklift_core.mission.commands import Cancel, Context, MissionCommands, Pause, Resume, Submit


def running(t=0.0):
    m = MissionCommands()
    m.queue(Submit("P1", "D1"))
    m.apply(t)
    m.start()
    return m


def test_submit_issues_a_mission_id_and_one_mission_at_a_time():
    m = MissionCommands()
    m.queue(Submit("P1", "D1", 3))
    (rec,) = m.apply(1.0)
    assert rec == {"t": 1.0, "command": "submit", "accepted": True, "reason": None, "mission_id": "m1",
                   "status": "accepted"}
    assert m.mission == Submit("P1", "D1", 3)
    m.start()
    assert m.status == "running"
    m.queue(Submit("P2", "D2"))
    (rec,) = m.apply(2.0)
    assert (rec["accepted"], rec["reason"], m.mission_id) == (False, "mission_active", "m1")


@pytest.mark.parametrize("context, reason", [(Context(stop_kind="preempt"), "stopping"),
                                             (Context(stop_kind="controlled"), "stopping"),
                                             (Context(clear_for_submit=False), "recovery_required"),
                                             # plan D5a: what the forks are in, from the session's cargo state
                                             (Context(submit_refusal="recovery_required"), "recovery_required"),
                                             (Context(submit_refusal="cargo_held"), "cargo_held")])
def test_submit_is_refused_while_stopping_or_when_a_recovery_is_needed(context, reason):
    m = MissionCommands()
    m.queue(Submit("P1", "D1"))
    assert m.apply(0.0, context)[0]["reason"] == reason and m.status == "idle"


def test_pause_resume_and_the_generation():
    m = running()
    m.queue(Pause())
    assert m.apply(10.0)[0]["accepted"] and m.status == "paused" and m.stopped and m.mission_gen == 1
    m.queue(Resume())
    assert m.apply(12.0, Context(stop_kind="controlled"))[0]["reason"] == "stopping"  # not while the arbiter stops
    m.queue(Resume())
    assert m.apply(13.0)[0]["accepted"] and m.status == "resuming" and m.stopped
    assert m.resumed(14.5) == pytest.approx(4.5) and m.status == "running" and m.mission_gen == 2
    assert not m.stopped


def test_pause_is_refused_outside_a_run_and_while_a_near_tracker_exists():
    m = MissionCommands()
    m.queue(Pause())
    assert m.apply(0.0)[0]["reason"] == "not_running"
    m = running()
    m.queue(Pause())
    assert m.apply(1.0, Context(near_tracker=True))[0]["reason"] == "near_tracking_active" and m.status == "running"
    m.queue(Resume())
    assert m.apply(2.0)[0]["reason"] == "not_paused"


def test_same_tick_commands_apply_by_priority_not_arrival():
    """agy 6: cancel > pause > resume, submit; a command the earlier one made pointless is refused and recorded."""
    m = running()
    m.queue(Pause())
    m.apply(1.0)
    m.queue(Resume())
    m.queue(Cancel())
    records = m.apply(2.0)
    assert [(r["command"], r["accepted"], r["reason"]) for r in records] == [
        ("cancel", True, None), ("resume", False, "not_paused")]
    assert m.status == "cancelling"
    m = running()
    m.queue(Submit("P2", "D2"))
    m.queue(Pause())
    assert [(r["command"], r["reason"]) for r in m.apply(3.0)] == [("pause", None), ("submit", "mission_active")]


def test_a_finished_mission_frees_the_slot_before_the_same_ticks_submit():
    """agy 1: the end is settled first (idle), then the queued submit is judged."""
    m = running()
    m.finish("done", 4.0)
    m.queue(Submit("P2", "D2"))
    (rec,) = m.apply(5.0)
    assert rec["accepted"] and m.mission_id == "m2" and m.status == "accepted"


def test_cancel_confirms_standing_first_and_an_unstarted_mission_ends_at_once():
    m = running()
    m.queue(Cancel("m9"))
    assert m.apply(1.0)[0]["reason"] == "unknown_mission"
    m.queue(Cancel("m1"))
    assert m.apply(2.0)[0]["accepted"] and m.status == "cancelling" and m.stopped
    with pytest.raises(RuntimeError):
        m.resumed(3.0)
    m.cancelled(3.0)
    assert m.status == "cancelled" and m.pauses == [[2.0, 3.0]]
    m.queue(Cancel())
    assert m.apply(4.0)[0]["reason"] == "not_running"  # cancelled is final; the slot is free again
    m = MissionCommands()
    m.queue(Submit("P1", "D1"))
    m.apply(0.0)
    m.queue(Cancel())
    m.apply(0.1)
    assert m.status == "cancelled" and m.pauses == []


def test_without_a_pause_work_elapsed_is_the_plain_difference_bit_for_bit():
    """agy 2: no subtraction at all before the first pause."""
    m = running()
    for since, now in ((0.1, 0.3), (1e-9, 12345.678), (0.0, 0.0)):
        assert m.work_elapsed(since, now) == now - since


def test_work_elapsed_takes_out_every_pause_inside_the_interval():
    m = running()
    m.queue(Pause())
    m.apply(10.0)
    assert m.work_elapsed(8.0, 13.0) == pytest.approx(2.0)  # paused since 10: the open pause counts up to now
    m.queue(Resume())
    m.apply(14.0)
    m.resumed(15.0)
    m.queue(Pause())
    m.apply(20.0)
    m.queue(Resume())
    m.apply(21.0)
    m.resumed(22.0)
    assert m.work_elapsed(8.0, 25.0) == pytest.approx(17.0 - 5.0 - 2.0)
    assert m.work_elapsed(16.0, 25.0) == pytest.approx(9.0 - 2.0)  # the first pause is before the interval


def test_resumed_returns_each_pauses_own_length_for_the_deadlines():
    """agy 3: deadline-type anchors move by each pause's length -- two pauses, two moves that add up."""
    m = running()
    lengths = []
    for start in (10.0, 30.0):
        m.queue(Pause())
        m.apply(start)
        m.queue(Resume())
        m.apply(start + 1.0)
        lengths.append(m.resumed(start + 2.5))
    assert lengths == pytest.approx([2.5, 2.5])


def test_an_unknown_stop_kind_is_refused():
    with pytest.raises(ValueError):
        MissionCommands().apply(0.0, Context(stop_kind="halt"))


def test_pausing_again_while_resuming_keeps_one_open_stop():
    """Codex M-a3a review: pause(10), resume(12), pause(14), resumed(20) is one 10 s stop, not two overlapping."""
    m = running()
    for t, command in ((10.0, Pause()), (12.0, Resume()), (14.0, Pause()), (16.0, Resume())):
        m.queue(command)
        assert m.apply(t)[0]["accepted"]
    assert m.mission_gen == 2  # two stop boundaries
    assert m.resumed(20.0) == pytest.approx(10.0) and m.pauses == [[10.0, 20.0]]
    assert m.work_elapsed(0.0, 25.0) == pytest.approx(15.0)


def test_a_mission_ending_while_paused_closes_its_stop_for_the_next():
    """Codex M-a3a review: pause(10), failed at 30, the next mission's work clock runs."""
    m = running()
    m.queue(Pause())
    m.apply(10.0)
    m.finish("failed", 30.0)
    assert m.status == "failed" and m.pauses == [[10.0, 30.0]] and m.paused_since is None
    m.queue(Submit("P2", "D2"))
    m.apply(30.0)
    m.start()
    assert m.work_elapsed(30.0, 35.0) == pytest.approx(5.0)


@pytest.mark.parametrize("before", [[Pause()], [Pause(), Resume()]])
def test_a_cancel_while_stopped_is_a_generation_of_its_own(before):
    m = running()
    for k, command in enumerate(before):
        m.queue(command)
        m.apply(10.0 + k)
    gen = m.mission_gen
    m.queue(Cancel())
    assert m.apply(20.0)[0]["accepted"] and m.mission_gen == gen + 1 and len(m.pauses) == 1
    m.cancelled(21.0)
    assert m.pauses == [[10.0, 21.0]]



@pytest.mark.parametrize("kind", ["controlled", "preempt"])
def test_an_arbiter_stop_pauses_the_mission_until_a_resume_after_it(kind):
    """M-a3d (plan E7): a stop kind is a pause; resume is refused while it lasts and needed after it."""
    m = running()
    (rec,) = m.apply(10.0, Context(stop_kind=kind))
    assert rec["command"] == f"stop_kind:{kind}" and m.status == "paused" and m.pauses == [[10.0, None]]
    m.apply(11.0, Context(stop_kind=kind))
    assert len(m.pauses) == 1  # one stop while it lasts
    m.queue(Resume())
    assert m.apply(12.0, Context(stop_kind=kind))[0]["reason"] == "stopping"
    assert m.apply(13.0)[0:] == [] and m.status == "paused"  # lifted: still paused until a resume
    m.queue(Resume())
    assert m.apply(14.0)[0]["accepted"] and m.status == "resuming"


def test_hold_stops_the_work_clock_until_a_resume_after_hold_off():
    """M-b B6: TELEOP holds the mission; handing back leaves it paused, and only a resume moves it."""
    m = running()
    assert m.hold_on(10.0) and m.status == "held" and m.stopped and m.pauses == [[10.0, None]]
    gen = m.mission_gen
    for command, reason in ((Pause(), "held"), (Resume(), "held")):
        m.queue(command)
        assert m.apply(11.0)[0]["reason"] == reason
    assert m.hold_off(20.0) and m.status == "paused" and m.mission_gen == gen + 1 and m.pauses == [[10.0, None]]
    m.queue(Resume())
    assert m.apply(21.0)[0]["accepted"] and m.status == "resuming"
    assert m.resumed(22.0) == pytest.approx(12.0) and m.work_elapsed(0.0, 25.0) == pytest.approx(13.0)
    assert [r["command"] for r in m.log if r["command"].startswith("hold")] == ["hold_on", "hold_off"]


def test_a_held_mission_can_be_cancelled_and_a_paused_one_held_keeps_its_stop():
    m = running()
    m.queue(Pause())
    m.apply(5.0)
    assert m.hold_on(6.0) and m.pauses == [[5.0, None]]
    m.queue(Cancel())
    assert m.apply(7.0)[0]["accepted"] and m.status == "cancelling"
    assert not MissionCommands().hold_on(0.0) and not running().hold_off(0.0)



def test_a_new_mission_does_not_carry_the_last_ones_resume_check():
    m = running()
    m.resume_check = {"step": "path_rechecked", "reason": "no_resume:rejoin_failed", "refused": True}
    m.finish("failed", 5.0)
    m.queue(Submit("P2", "D2"))
    assert m.apply(6.0)[0]["accepted"] and m.resume_check is None


def test_waits_stop_the_work_clock_once_even_inside_a_pause():
    """Real-time plan D8c: a wait for a plan is taken out of the work time while it is open and after it closes;
    overlapping a pause it counts once."""
    c = MissionCommands()
    assert c.work_elapsed(0.0, 10.0) == 10.0
    c.begin_wait(2.0)
    assert c.work_elapsed(0.0, 4.0) == pytest.approx(2.0)  # open: stopped since 2.0
    with pytest.raises(ValueError):
        c.begin_wait(3.0)
    c.end_wait(5.0)
    c.end_wait(6.0)  # none open: nothing
    assert c.work_elapsed(0.0, 10.0) == pytest.approx(7.0)
    assert c.work_elapsed(3.0, 10.0) == pytest.approx(5.0)  # the wait counted from where the interval starts
    c.pauses.append([4.0, 6.0])  # overlaps the wait 4..5
    assert c.work_elapsed(0.0, 10.0) == pytest.approx(6.0)  # stopped 2..6 once


# --- the persistent console's mission end (docs/plans/2026-10-18-persistent-live-console.md D1) ---------------------
def test_close_open_closes_the_open_pause_and_wait_without_changing_the_status():
    m = running()
    m.queue(Pause())
    m.apply(2.0)
    m.begin_wait(3.0)
    gen = m.mission_gen
    m.close_open(5.0)
    assert m.status == "paused" and m.mission_gen == gen
    assert m.pauses == [[2.0, 5.0]] and m.waits == [[3.0, 5.0]] and m.paused_since is None
    m.close_open(6.0)  # nothing open: nothing changes
    assert m.pauses == [[2.0, 5.0]] and m.waits == [[3.0, 5.0]]


@pytest.mark.parametrize("outcome", ["done", "failed", "cancelled"])
def test_terminate_ends_an_active_mission_with_its_reason_once(outcome):
    m = running()
    m.queue(Pause())
    m.apply(2.0)
    gen = m.mission_gen
    assert m.terminate(outcome, 4.0, "why") is True
    assert m.status == outcome and m.end_reason == "why" and m.mission_gen == gen + 1
    assert m.pauses == [[2.0, 4.0]] and m.paused_since is None
    assert m.log[-1] == {"t": 4.0, "command": "end", "accepted": True, "reason": "why", "mission_id": "m1",
                         "status": outcome}
    assert m.terminate("failed", 5.0, "again") is False  # already over: nothing changes
    assert m.status == outcome and m.end_reason == "why" and len(m.log) == 3
    m.apply(6.0)
    assert m.status == "idle" and m.mission_id is None


def test_terminate_keeps_the_advice_for_this_end_only():
    """Persistent console plan D13 item 6: the advice goes with the end that made it."""
    m = running()
    assert m.terminate("failed", 4.0, "prepare:all_candidates_failed:invalid_start", advice={"backoff_m": 0.1})
    assert m.end_advice == {"backoff_m": 0.1}
    m2 = running()
    m2.terminate("failed", 4.0, "why")
    assert m2.end_advice is None
    m2.note_end(5.0, "again")
    assert m2.end_advice is None


def test_terminate_refuses_an_unknown_outcome_and_an_idle_slot():
    m = MissionCommands()
    with pytest.raises(ValueError):
        m.terminate("finished", 1.0, None)
    assert m.terminate("failed", 1.0, "x") is False and m.status == "idle"


def test_take_clock_hands_over_the_finished_missions_stops_and_empties_them():
    m = running()
    m.queue(Pause())
    m.apply(2.0)
    m.begin_wait(2.5)
    m.terminate("failed", 4.0, "why")
    assert m.take_clock() == {"pauses": [[2.0, 4.0]], "waits": [[2.5, 4.0]]}
    assert m.pauses == [] and m.waits == []
    assert m.work_elapsed(0.0, 10.0) == 10.0  # the next mission's work clock starts clean


def test_take_clock_refuses_while_a_mission_is_active():
    m = running()
    with pytest.raises(RuntimeError):
        m.take_clock()


def test_a_submit_in_the_same_apply_as_an_end_waits_for_the_next_tick():
    """Codex stage 1 P3: a cancel of an accepted mission and a submit in one apply -- the end is collected first."""
    m = MissionCommands()
    m.queue(Submit("P1", "D1"))
    m.apply(1.0)
    m.queue(Cancel())
    m.queue(Submit("P2", "D2"))
    cancel, submit = m.apply(2.0)
    assert cancel["accepted"] and (submit["accepted"], submit["reason"]) == (False, "mission_active")
    assert m.status == "cancelled" and m.mission_id == "m1"


def test_note_end_logs_an_end_already_reached():
    """Codex stage 1 P3: a cancel confirmed standing is cancelled by the core; its end is still logged once."""
    m = running()
    m.queue(Cancel())
    m.apply(1.0)
    m.cancelled(2.0)
    assert m.terminate("cancelled", 2.0, "operator") is False
    m.note_end(2.0, "operator")
    assert m.log[-1] == {"t": 2.0, "command": "end", "accepted": True, "reason": "operator", "mission_id": "m1",
                         "status": "cancelled"} and m.end_reason == "operator"
    with pytest.raises(RuntimeError):
        running().note_end(3.0, "x")


# --- persistent console plan D2: preparing (a live mission's first leg planned before it runs) ----------------------
def preparing(t=0.0):
    m = MissionCommands()
    m.queue(Submit("P1", "D1"))
    m.apply(t)
    m.begin_prepare()
    return m


def test_a_submit_is_prepared_then_adopted():
    m = preparing()
    assert m.status == "preparing" and not m.stopped
    gen = m.mission_gen
    m.adopt()
    assert m.status == "running" and m.mission_gen == gen + 1
    with pytest.raises(RuntimeError):
        m.adopt()
    with pytest.raises(RuntimeError):
        running().begin_prepare()


@pytest.mark.parametrize("kind, reason", [("preempt", "prepare:stopped"), ("controlled", "prepare:paused")])
def test_a_stop_ends_the_preparation(kind, reason):
    m = preparing()
    (rec,) = m.apply(1.0, Context(stop_kind=kind))
    assert m.status == "failed" and m.end_reason == reason and m.pauses == []
    assert rec["command"] == "end" and rec["reason"] == reason


def test_a_cancel_ends_the_preparation_at_once():
    m = preparing()
    m.queue(Cancel())
    m.apply(1.0)
    assert m.status == "cancelled" and m.pauses == []


def test_teleop_ends_the_preparation():
    m = preparing()
    assert m.hold_on(1.0) is True
    assert m.status == "failed" and m.end_reason == "prepare:teleop"


def test_the_adopted_mission_is_named_by_start_and_adopt():
    m = MissionCommands()
    assert m.adopted_id is None
    m.queue(Submit("P1", "D1"))
    m.apply(0.0)
    assert m.adopted_id is None
    m.start()
    assert m.adopted_id == "m1"
    p = preparing(1.0)
    assert p.adopted_id is None
    p.adopt()
    assert p.adopted_id == p.mission_id


def test_a_submit_is_refused_while_the_scene_is_damaged():
    """Persistent console plan D6: a latched damage refuses automatic work (teleop stays allowed)."""
    m = MissionCommands()
    m.queue(Submit("P1", "D1"))
    assert m.apply(0.0, Context(scene_damaged=True))[0]["reason"] == "scene_damaged" and m.status == "idle"


def recovering(t=10.0):
    m = running()
    m.queue(Cancel())
    m.apply(t)
    m.begin_recovery(t + 1.0)  # the cancel stood: the forks are in a pocket (plan D4)
    return m


def test_a_recovery_reopens_the_work_clock_and_refuses_a_cancel():
    m = recovering()
    assert m.status == "recovering" and m.recovery and m.paused_since is None
    assert m.work_elapsed(11.0, 15.0) == pytest.approx(4.0)  # the cancel's stop closed at the recovery's start
    m.queue(Cancel())
    (rec,) = m.apply(12.0)
    assert (rec["accepted"], rec["reason"], m.status) == (False, "recovering", "recovering")


def test_a_stop_pauses_a_recovery_and_the_resume_returns_to_it():
    m = recovering()
    m.apply(12.0, Context(stop_kind="preempt"))
    assert m.status == "paused" and m.recovery
    m.queue(Resume())
    m.apply(13.0)
    m.resumed(13.5)
    assert m.status == "recovering"
    m.queue(Pause())
    m.apply(14.0)
    assert m.status == "paused"


def test_a_recovery_ends_cancelled_or_recovery_required():
    m = recovering()
    m.recovered(20.0)
    assert m.status == "cancelled" and not m.recovery
    m2 = recovering()
    assert m2.terminate("recovery_required", 20.0, "recovery:refused") and m2.status == "recovery_required"
    m2.queue(Submit("P1", "D1"))
    (rec,) = m2.apply(21.0)  # the slot frees at the next apply, the submit judged then
    assert rec["accepted"] and not m2.recovery


def test_a_submit_before_localization_tracks_is_refused_localization_lost():
    """Persistent console plan D3 (3rd ed.): the tracker not yet tracking (warming_up) refuses a submit; the
    reasons before it keep their order."""
    from forklift_core.mission.commands import Context, Submit  # noqa: PLC0415

    m = MissionCommands()
    m.queue(Submit("P-1", "D-1", 0))
    rec = m.apply(1.0, Context(localization_ok=False))
    assert rec[-1]["reason"] == "localization_lost"
    m.queue(Submit("P-1", "D-1", 0))
    rec = m.apply(2.0, Context(localization_ok=False, clear_for_submit=False))
    assert rec[-1]["reason"] == "recovery_required"  # a hold is said first
    m.queue(Submit("P-1", "D-1", 0))
    assert m.apply(3.0, Context())[-1]["accepted"]


def test_a_loaded_submit_names_the_carried_pallet_and_marks_the_mission_loaded():
    """Persistent console plan D5 6c: cargo_held lets through only the submit of the pallet the runner says can be
    delivered as carried; any other is still cargo_held."""
    m = MissionCommands()
    m.queue(Submit("P-2", "D-1", 0))
    rec = m.apply(1.0, Context(submit_refusal="cargo_held", loaded_pallet="P-1"))
    assert rec[-1]["reason"] == "cargo_held"
    m.queue(Submit("P-1", "D-1", 0))
    rec = m.apply(2.0, Context(submit_refusal="cargo_held", loaded_pallet=None))
    assert rec[-1]["reason"] == "cargo_held"
    m.queue(Submit("P-1", "D-1", 0))
    rec = m.apply(3.0, Context(submit_refusal="cargo_held", loaded_pallet="P-1"))
    assert rec[-1]["accepted"] and m.mission_loaded
    m2 = MissionCommands()
    m2.queue(Submit("P-1", "D-1", 0))
    assert m2.apply(1.0, Context())[-1]["accepted"] and not m2.mission_loaded
    m3 = MissionCommands()
    m3.queue(Submit("P-1", "D-1", 0))
    assert m3.apply(1.0, Context(submit_refusal="recovery_required", loaded_pallet="P-1"))[-1]["reason"] == \
        "recovery_required"


def _book_with_a_put_down_pallet():
    from forklift_core.mission.targets import Pose, TargetsBook

    b = TargetsBook()
    b.add_shape("epal6", 0.6, 0.8)
    b.add_pallet("P-1", "epal6", Pose(0.0, 0.0, 0.0), depth_m=0.6, width_m=0.8)
    b.add_pallet("P-2", "epal6", Pose(0.0, 3.0, 0.0), depth_m=0.6, width_m=0.8)
    for did, x in (("D-1", 9.0), ("D-2", 9.0), ("D-3", 9.0)):
        b.add_destination(did, Pose(x, {"D-1": -4.0, "D-2": -6.0, "D-3": -8.0}[did], 0.0))
    b.carried("P-1")
    b.put_down("P-1", Pose(9.0, -4.0, 0.0), uncertainty_m=0.2)
    b.verify_at("P-1", "D-1")
    return b


def test_a_repick_the_pick_judges_blocked_is_refused_with_what_blocks_it():
    # the az8 line_m queue (2026-10-28): a manual re-pick of a put-down pallet runs the pick's own blocking judgement
    # first; refused "blocked", the blockers in the record's detail (the arbiter's reject carries it)
    b = _book_with_a_put_down_pallet()
    asked = []

    def blockers(pid, did):
        asked.append((pid, did))
        return ["pickup:P-2"]

    m = MissionCommands(targets=b)
    m.queue(Submit("P-1", "D-3", b.revision, request_id="r1"))
    (rec,) = m.apply(0.0, Context(repick_blockers=blockers))
    assert (rec["accepted"], rec["reason"], rec["detail"]) == (False, "blocked", "pickup:P-2")
    assert asked == [("P-1", "D-3")] and m.status == "idle"
    # nothing blocks it: accepted, no detail
    m.queue(Submit("P-1", "D-3", b.revision, request_id="r2"))
    (rec,) = m.apply(1.0, Context(repick_blockers=lambda pid, did: []))
    assert rec["accepted"] and "detail" not in rec


def test_only_a_put_down_pallet_is_judged_and_without_the_judge_nothing_changes():
    b = _book_with_a_put_down_pallet()
    called = []
    m = MissionCommands(targets=b)
    m.queue(Submit("P-2", "D-3", b.revision))  # P-2 waits where it was first placed: the planner judges it, as before
    (rec,) = m.apply(0.0, Context(repick_blockers=lambda pid, did: called.append(pid) or ["pickup:P-1"]))
    assert rec["accepted"] and called == []
    m2 = MissionCommands(targets=_book_with_a_put_down_pallet())
    m2.queue(Submit("P-1", "D-3", m2.targets.revision))
    (rec,) = m2.apply(0.0, Context())  # no judge (a batch run): accepted as before
    assert rec["accepted"]
