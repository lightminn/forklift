"""docs/design/2026-11-19-real-safety-wiring.md 6, 2 (stage 5c-2): the core's own hold marks go with a relocalization
reset (the real loop calls it with the tracker's reset, after the mission is over)."""

from test_core_field_recovery import make


def test_the_reset_clears_the_hold_marks_and_the_deferred_release():
    core, _, _ = make("idle")
    core.in_hold = True
    core.put_downs_in_hold.add("P1")
    core._last_pose_hold = ("P1", 0)  # noqa: SLF001
    core.end_release_due = 12.0
    core.relocalization_reset(20.0)
    assert core.in_hold is False and not core.put_downs_in_hold
    assert core._last_pose_hold is None and core.end_release_due is None  # noqa: SLF001
    assert core.state["field_recoveries"][-1] == {"event": "relocalization_reset", "time_s": 20.0}


def test_the_reset_leaves_the_relocalization_lock_alone():
    core, _, _ = make("idle")
    gen = core.require_relocalization(5.0, "mcu_reboot")
    core.relocalization_reset(6.0)
    assert core.relocalize_generation() == gen
