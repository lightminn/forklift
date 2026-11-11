"""Sim-to-real 4d-2b-2 in the mission core: the settle confirmation, the field recovery's opening and its end, and
the teleop refusals (docs/design/2026-11-11-p05-truth-audit.md 4-3/4-4)."""

from types import SimpleNamespace

import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.mission.targets import Pose
from forklift_core.safety.teleop_policy import teleop_policy


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


def standing(core, yes=True):
    from forklift_core.safety.truth_access import SafetySpeed

    core.hooks.safety_speed = lambda: SafetySpeed(0.0, 0.0 if yes else 0.05, (1,) if yes else (1, -1))


def test_settle_waits_for_the_operator_to_confirm_the_forks_out():
    core, hooks, moved = make("settle")
    standing(core)
    core.cargo.on_insert_start((0.0, 0.0, 0.0))
    assert core.lift_and_timers(2.0, 0.01) is False and moved == []
    assert hooks.asked[-1]["kind"] == "withdrawn"
    hooks.answer = (hooks.asked[-1]["serial"], "confirm")
    core.lift_and_timers(2.1, 0.01)
    assert moved == ["complete"] and core.cargo.pocket_entered is False


def test_a_settle_reject_ends_the_mission():
    core, hooks, _ = make("settle", answer=lambda r: (r["serial"], "reject"))
    standing(core)
    with pytest.raises(Failed, match=r"Forks are not out \(operator\)"):
        core.lift_and_timers(2.0, 0.01)


def test_the_simulation_settles_on_its_timer():
    core, hooks, moved = make("settle", real=False)
    core.lift_and_timers(2.0, 0.01)
    assert moved == ["complete"] and hooks.asked == []


def field_core(relation="carrying", entered=True):
    core, hooks, _ = make("transport")
    core.commands.mission = SimpleNamespace(pallet_id="P1", destination_id="D1")
    core.commands.mission_id = "m1"
    if entered:
        core.cargo.on_insert_start((0.0, 0.0, 0.0))
    core.cargo.relation = relation
    core.targets.add_destination("D1", Pose(5.0, 0.0, 0.0))
    core.targets.add_pallet("P1", "epal", Pose(1.0, 0.0, 0.0), depth_m=1.2, width_m=0.8)
    return core


def test_a_mission_ending_with_the_forks_in_opens_one_field_recovery_and_no_core_assisted_one():
    core = field_core()
    core._open_field_recovery(10.0)  # noqa: SLF001  (what end_reset does)
    core.capture_recovery_context()
    assert core.field_recovery["id"] == 1 and core.field_recovery["pallet_id"] == "P1"
    assert core.cargo.recovery_context is None  # decision 1: B, no D14 rows
    core._open_field_recovery(11.0)  # noqa: SLF001
    assert core.field_recovery["id"] == 1  # still the same one
    empty = field_core(relation="free", entered=False)
    empty._open_field_recovery(10.0)  # noqa: SLF001
    assert empty.field_recovery is None


@pytest.mark.parametrize("kw,status,why", [
    (dict(recovery_id=2, disarmed=True, outstanding=False), "failed", "stale_recovery"),
    (dict(recovery_id=1, disarmed=True, outstanding=False), "running", "mission_active"),
    (dict(recovery_id=1, disarmed=True, outstanding=False), "preparing", "mission_active"),
    (dict(recovery_id=1, disarmed=False, outstanding=False), "failed", "not_disarmed"),
    (dict(recovery_id=1, disarmed=True, outstanding=True), "failed", "commands_outstanding"),
])
def test_recovery_done_is_refused_unless_every_condition_holds(kw, status, why):
    core = field_core()
    core._open_field_recovery(10.0)  # noqa: SLF001
    core.commands.status = status
    assert core.field_recovery_done(20.0, **kw) == (False, why)
    assert core.field_recovery is not None and core.cargo.relation == "carrying"  # nothing changed


def test_recovery_done_blocks_first_then_frees_the_forks_and_never_resumes():
    core = field_core()
    core._open_field_recovery(10.0)  # noqa: SLF001
    core.commands.status = "failed"
    abandoned = []
    core.jobs = object()
    core.abandon_jobs = lambda t: abandoned.append(t)  # the ended mission's work, without a resume's undo
    assert core.field_recovery_done(20.0, 1, disarmed=True, outstanding=False) == (True, None)
    assert abandoned == [20.0] and core.field_recovery is None
    assert core.targets.pallets["P1"].state == "unlocated"
    assert (core.cargo.relation, core.cargo.load, core.cargo.pocket_entered) == ("free", "none", False)
    assert core.cargo.submit_refusal() == "relocalize_required" and core.commands.status == "failed"
    assert core.field_recovery_done(21.0, 1, disarmed=True, outstanding=False) == (False, "no_open_recovery")
    gen = core.relocalize_generation()
    assert gen == 1 and core.relocalized(29.0, gen - 1) is False  # an older relocalization's late answer
    assert core.cargo.submit_refusal() == "relocalize_required"
    assert core.relocalized(30.0, gen) is True and core.cargo.submit_refusal() is None
    assert [e["event"] for e in core.state["field_recoveries"]] == ["opened", "done", "relocalized"]


@pytest.mark.parametrize("lift_m", [0.0, 0.3])
def test_teleop_moves_nothing_while_a_field_recovery_is_open(lift_m):
    s = {"load": {"loaded": False}, "fork_in_pocket": {"value": True, "relation": "constrained"},
         "recovery_required": True}
    p = teleop_policy(s, feedback_ok=True, standing=True, lift_m=lift_m)
    assert not p.drive_allowed and not p.lift_allowed and p.reason == "recovery_required"


def test_the_real_truck_refuses_loaded_teleop():
    s = {"load": {"loaded": True}, "fork_in_pocket": {"value": True, "relation": "carrying"},
         "teleop_refused": "real_loaded"}
    p = teleop_policy(s, feedback_ok=True, standing=True, lift_m=0.1)
    assert not p.drive_allowed and not p.lift_allowed and p.reason == "real_loaded"
    s.pop("teleop_refused")
    assert teleop_policy(s, feedback_ok=True, standing=True, lift_m=0.1).drive_allowed  # the simulation as before


def test_the_settle_request_holds_only_while_standing_with_the_forks_down():
    core, hooks, moved = make("settle")
    standing(core)
    core.lift_and_timers(2.0, 0.01)
    standing(core, False)  # rolling while the operator looks
    core.lift_and_timers(2.1, 0.01)
    assert core.state["operator_checks"][-1]["event"] == "dropped:precondition" and moved == []


def test_forks_not_confirmed_out_count_as_in_a_pocket_for_teleop():
    core, _, _ = make("settle")
    core.cargo.on_insert_start((0.0, 0.0, 0.0))
    core.cargo.relation = "free"  # the withdraw straight ran out (the tracker's)
    fields = core.cargo.safety_fields()
    assert fields["fork_in_pocket"]["value"] is True
    s = {"load": {"loaded": False}, "fork_in_pocket": fields["fork_in_pocket"]}
    p = teleop_policy(s, feedback_ok=True, standing=True, lift_m=0.0)
    assert not p.drive_allowed and not p.lift_allowed
    core.cargo.confirm_out()
    assert core.cargo.safety_fields()["fork_in_pocket"]["value"] is False


def test_a_new_insertion_forgets_the_last_confirmation_and_an_invalidation_overrides_it():
    core, _, _ = make("insert")
    c = core.cargo
    c.on_insert_start((0.0, 0.0, 0.0))
    c.relation = "free"
    c.confirm_out()
    assert c.cancel_decision((0.0, 0.0, 0.0), in_withdraw=False) == "cancelled"
    c.on_insert_start((1.0, 0.0, 0.0))  # the next mission's insertion
    assert c.cancel_decision((0.0, 0.0, 0.0), in_withdraw=False) == "recovery_required"
    c.confirm_out()
    c.invalidate("dropped")
    assert c.cancel_decision((0.0, 0.0, 0.0), in_withdraw=False) == "recovery_required"
