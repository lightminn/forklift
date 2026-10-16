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
                                             (Context(clear_for_submit=False), "recovery_required")])
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
