import pytest

from tools import bundle_slots as bs


def test_the_next_run_goes_to_the_slot_that_can_start_it_first():
    # Codex (node-queue plan, round 2): the stagger applied after picking a slot gave held 26 instead of 40
    held, used, span = bs.simulate([10, 10, 1, 1], jobs=2, slots=2)
    assert (held, used, span) == (40.0, 22.0, 10.0)


def test_one_job_holds_its_slots_until_its_last_run():
    held, used, span = bs.simulate([5, 1], jobs=1, slots=2)
    assert (held, used, span) == (10.0, 6.0, 5.0)


def test_the_tail_counts_from_the_list_s_last_start_and_from_a_late_job_s_start():
    # job a: two slots from 0 to 30 min, runs 0-10 and 5-30 (last start 5 min); job b came at 29 min and ran nothing
    a = ("a", 2, 0.0, 1800.0, [{"start": 0.0, "end": 600.0}, {"start": 300.0, "end": 1800.0}])
    b = ("b", 2, 1740.0, 1800.0, [])
    per, tot = bs.idle_table([a, b])
    assert per[0]["held"] == 60.0 and per[0]["used"] == 35.0
    assert per[0]["tail"] == pytest.approx(2 * 25 - (5 + 25))  # from 5 min: 50 held, 30 used
    assert per[1]["tail"] == pytest.approx(2.0)  # from its own start, not from the list's last start
    assert tot["tail"] == pytest.approx(22.0)


def test_jobs_that_ran_nothing_are_refused_alone():
    with pytest.raises(ValueError):
        bs.idle_table([("b", 2, 0.0, 60.0, [])])
