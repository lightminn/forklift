"""Sim-to-real 4d-2b-1: the real truck's operator confirmations at lift and lower, their request contract, and no
automatic pocket retrace (docs/design/2026-11-11-p05-truth-audit.md 4-3)."""

from types import SimpleNamespace

import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState


class Failed(RuntimeError):
    pass


class Hooks:
    def __init__(self, lift=0.0, answer=None):
        self.lift, self.answer, self.asked, self.z_reads = lift, answer, [], 0

    def require(self, ok, reason):
        if not ok:
            raise Failed(reason)

    def lift_position(self):
        return self.lift

    def operator_check(self, request):
        self.asked.append(request)
        return self.answer(request) if callable(self.answer) else self.answer

    def pallet_z(self):
        self.z_reads += 1
        return 0.1


def make(phase="lift", *, real=True, lift=0.0, answer=None):
    hooks = Hooks(lift, answer)
    ms = MissionState(phase=phase, phase_started=0.0)
    opts = SimpleNamespace(drive_geometry=None, command_accel_mps2=0.8, real_vehicle=real)
    core = MissionCore(ms, state={"transitions": []}, obstacle=None, slam=None, pocket={"check": None}, paths={},
                       trackers={}, geometry=None,
                       settings={"lift_target_m": 0.10, "lift_rate_mps": 0.05, "steering_command_rate_rad_s": 1.0},
                       pallet_geometry=None, hooks=hooks, near=None, options=opts)
    moved = []
    core.transition = lambda to, t: moved.append(to) or setattr(ms, "phase", to)
    return core, hooks, moved


def events(core):
    return [(e["serial"], e["event"]) for e in core.state.get("operator_checks", [])]


def test_lift_waits_for_the_lift_then_for_the_operator_and_never_reads_truth():
    core, hooks, moved = make(lift=0.05)
    core.lift_and_timers(5.0, 0.01)
    assert moved == [] and hooks.asked == []  # the lift has not reached: the motion stage
    hooks.lift = 0.098
    core.lift_and_timers(6.0, 0.01)
    assert moved == [] and len(hooks.asked) == 1 and events(core) == [(1, "requested")]
    req = hooks.asked[0]
    assert (req["kind"], req["phase"], req["serial"]) == ("lifted", "lift", 1)
    hooks.answer = (1, "confirm")
    core.lift_and_timers(6.1, 0.01)
    assert moved == ["extract"] and hooks.z_reads == 0
    assert events(core)[-1] == (1, "confirm")


def test_a_lift_that_never_reaches_ends_the_mission_after_the_reach_limit():
    core, hooks, _ = make(lift=0.05)
    core.lift_and_timers(5.0, 0.01)
    core.lift_and_timers(14.9, 0.01)
    with pytest.raises(Failed, match="lifted: lift did not reach"):
        core.lift_and_timers(15.1, 0.01)


def test_the_operator_wait_has_no_limit_and_a_reject_ends_the_mission():
    core, hooks, moved = make(lift=0.1)
    for t in (5.0, 100.0, 1000.0):
        core.lift_and_timers(t, 0.01)
    assert moved == [] and len(hooks.asked) == 3  # polled, never answered
    hooks.answer = (1, "reject")
    with pytest.raises(Failed, match=r"Pallet was not lifted \(operator\)"):
        core.lift_and_timers(1001.0, 0.01)


@pytest.mark.parametrize("answer", [None, True, "confirm", (2, "confirm"), (1, "yes"), (1, None), ("1", "confirm")])
def test_only_the_current_serial_and_a_confirm_or_reject_count(answer):
    core, hooks, moved = make(lift=0.1, answer=answer)
    core.lift_and_timers(5.0, 0.01)
    core.lift_and_timers(5.1, 0.01)
    assert moved == [] and core._operator["answer"] is None  # noqa: SLF001


def test_a_broken_precondition_drops_the_request_and_its_answer():
    core, hooks, moved = make(lift=0.1)
    core.lift_and_timers(5.0, 0.01)  # requested (serial 1)
    hooks.lift = 0.08  # sagged 20 mm while waiting
    core.lift_and_timers(5.1, 0.01)
    assert events(core)[-1] == (1, "dropped:precondition")
    hooks.answer = (1, "confirm")  # a late confirm of the dropped request
    hooks.lift = 0.1
    core.lift_and_timers(5.2, 0.01)
    assert moved == [] and events(core)[-1] == (2, "requested")  # a new request; the old answer does not count
    hooks.answer = (2, "confirm")
    core.lift_and_timers(5.3, 0.01)
    assert moved == ["extract"]


def test_a_lower_reject_ends_the_mission_at_once_and_asking_again_drops_a_pending_request():
    core, hooks, moved = make("lower", lift=0.0)
    core.lift_and_timers(5.0, 0.01)
    assert core.operator_request_again(5.05) is True  # e.g. the console lost the request: a fresh serial
    hooks.answer = (1, "confirm")  # the dropped serial's answer does not count
    core.lift_and_timers(5.1, 0.01)
    assert moved == [] and [e for _, e in events(core)] == ["requested", "dropped:again", "requested"]
    hooks.answer = (2, "reject")
    with pytest.raises(Failed, match=r"Pallet did not settle onto destination floor \(operator\)"):
        core.lift_and_timers(5.2, 0.01)
    fresh, _, _ = make("lower")
    assert fresh.operator_request_again(1.0) is False  # nothing open


def test_a_new_phase_or_generation_is_a_new_request():
    core, hooks, _ = make(lift=0.1)
    core.lift_and_timers(5.0, 0.01)
    core.commands.mission_gen += 1  # a pause and resume
    core.lift_and_timers(5.1, 0.01)
    assert [s for s, e in events(core) if e == "requested"] == [1, 2]
    assert core.operator_request_again(5.2) is True and core._operator["request"]["serial"] == 3  # noqa: SLF001


def test_the_simulation_judges_lift_and_lower_on_truth_as_before():
    core, hooks, moved = make(real=False, lift=0.0)
    core.lift_and_timers(5.0, 0.01)
    assert moved == ["extract"] and hooks.z_reads == 1 and hooks.asked == []
    core, hooks, moved = make("lower", real=False, lift=0.0)
    hooks.pallet_z = lambda: 0.0
    core.lift_and_timers(5.0, 0.01)
    assert moved == ["withdraw"] and hooks.asked == []


def test_the_real_truck_never_assumes_the_retrace_premise():
    core, _, _ = make()
    core.cargo.on_insert_start((0.0, 0.0, 0.0))
    assert core.cargo.trace_valid is False
    sim, _, _ = make(real=False)
    sim.cargo.on_insert_start((0.0, 0.0, 0.0))
    assert sim.cargo.trace_valid is True


def test_a_real_recover_lower_fails_before_any_truth_height():
    core, hooks, _ = make("recover_lower", lift=0.0)
    failed = []
    core.recovery_failed = lambda t, why: failed.append(why)
    core.commands.status = "recovering"
    core.cargo.trace_valid = False
    from forklift_core.mission.core import RECOVERY_LOWER_MIN_S

    core.recovery_timers(RECOVERY_LOWER_MIN_S + 0.1)
    assert failed == ["trace_invalid"] and hooks.z_reads == 0
