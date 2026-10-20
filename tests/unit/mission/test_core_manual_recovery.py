"""Persistent console plan D14 (5th ed.): after a live mission ended recovery_required, the core keeps the recovery
context, reports the recovery stage for the teleop policy, takes the manual ground verdict, judges the forks free
by the automatic recovery's criteria, re-expresses the cargo's map poses at every applied correction change and
leaves the lift alone while no mission is adopted."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from test_core_pause import Layer  # noqa: E402
from test_core_recovery import inserted_core, stand  # noqa: E402


def ended(*, depth=0.30, load=None):
    """A live insert cancelled at ``depth``, its retrace refused (evidence valid): recovery_required, ended."""
    core, rear = inserted_core(layer=Layer(blocked="occupied", verified_m=0.05), depth=depth)
    stand(core, 6.0, rear)
    core.lift_and_timers(7.2, 1 / 120)
    assert core.commands.status == "recovery_required" and core.cargo.relation == "constrained"
    if load is not None:
        core.cargo.load = load
    core.capture_recovery_context()
    core.commands.adopted_id = None
    core.cargo.last_rear = tuple(float(v) for v in rear)
    return core, rear


def test_the_context_survives_the_mission_and_reports_the_stage():
    core, rear = ended()
    ctx = core.cargo.recovery_context
    assert ctx["pallet_id"] == core.commands.mission.pallet_id and ctx["stage"] == "in_pocket" and ctx["grounded"]
    rec = core.safety_state(8.0)["recovery"]
    assert rec == {"stage": "in_pocket", "grounded": True, "evidence": True, "aligned": True,
                   "pallet_id": ctx["pallet_id"]}


def test_off_the_pocket_axis_is_not_aligned():
    core, rear = ended()
    core.cargo.last_rear = (rear[0], rear[1] + 0.05, rear[2])  # 5 cm across the axis (band 0.02 m here)
    assert core.safety_state(8.0)["recovery"]["aligned"] is False
    core.cargo.last_rear = (rear[0], rear[1], rear[2] + 0.03)  # 0.03 rad off the axis
    assert core.safety_state(8.0)["recovery"]["aligned"] is False


def test_backed_out_by_hand_the_forks_are_free_and_the_manual_clear_begins():
    core, rear = ended()
    out = (rear[0] - 0.60, rear[1], rear[2])  # depth 0.30 -> -0.30, past -(0.10 + band)
    core.hooks.lift_position, core.hooks.pallet_z = (lambda: 0.0), (lambda: 0.0)
    core.manual_recovery_tick(9.0, out, standing=True)
    assert core.cargo.relation == "free" and core.cargo.recovery_context["stage"] == "manual_clear"
    assert core.safety_state(9.0).get("recovery_required") is None
    assert core.state["manual_recovery"][-1]["event"] == "free"


@pytest.mark.parametrize("why", ["moving", "lifted", "evidence", "not_out"])
def test_each_free_criterion_holds_it_back(why):
    core, rear = ended()
    out = (rear[0] - (0.30 if why == "not_out" else 0.60), rear[1], rear[2])
    core.hooks.lift_position = lambda: 0.02 if why == "lifted" else 0.0
    core.hooks.pallet_z = lambda: 0.0
    if why == "evidence":
        core.cargo.trace_valid = False
    core.manual_recovery_tick(9.0, out, standing=why != "moving")
    assert core.cargo.relation == "constrained"


def test_a_pallet_that_rode_the_forks_waits_for_the_manual_ground_verdict():
    core, rear = ended(load="confirmed")
    assert core.cargo.recovery_context["grounded"] is False
    core.cargo.recovery_context["pose"] = (0.75, 0.0, 0.0)  # the pickup estimate the context kept
    grounded = []
    core.hooks.trace_ground = lambda: grounded.append(True)
    core.hooks.lift_position, core.hooks.pallet_z = (lambda: 0.005), (lambda: 0.0)
    core.targets.carried(core.commands.mission.pallet_id)
    core.manual_recovery_tick(9.0, rear, standing=True)
    core.manual_recovery_tick(9.5, rear, standing=True)
    assert core.cargo.recovery_context["grounded"] is False  # held for 1 s
    core.manual_recovery_tick(10.1, rear, standing=True)
    assert core.cargo.recovery_context["grounded"] is True and grounded == [True]
    assert core.targets.pallets[core.commands.mission.pallet_id].state != "carried"  # back in the targets


def test_the_restored_pallet_comes_from_the_context_once_the_mission_is_gone():
    from forklift_core.planning.pallet_mission import PalletSite

    core, rear = ended(load="confirmed")
    core.cargo.recovery_context["pose"] = (0.75, 0.0, 0.0)
    pid = core.cargo.recovery_context["pallet_id"]
    core.targets.carried(pid)
    core.commands.mission = None  # the next apply() clears it (Codex D14 design round 1 P2-7)
    core.restore_pallet_target()
    assert core.targets.pallets[pid].state == "available" and core.targets.pallets[pid].pose.x_m == pytest.approx(0.75)


def test_every_applied_correction_change_moves_the_cargo_with_it():
    """Codex D14 design round 4 P2: a reply's 0.04 m correction must not push a standing truck off the axis."""
    core, rear = ended()
    anchor = core.cargo.anchor
    change = (0.0, 0.04, 0.01)
    core.obstacle["applied"] = (0.0, 0.0, 0.0)
    core.obstacle["layer"].refresh = lambda *a, **k: None
    core.trackers = {}
    moved = tuple(float(v) for v in np.asarray(_compose(change, tuple(rear))))
    core._reproject(9.0, change, moved, False)
    assert core.cargo.anchor == pytest.approx(_compose(change, anchor))
    core.cargo.last_rear = moved
    assert core.safety_state(9.0)["recovery"]["aligned"] is True


def test_no_context_no_re_expression():
    core, rear = inserted_core()
    anchor = core.cargo.anchor
    core.obstacle["applied"] = (0.0, 0.0, 0.0)
    core.obstacle["layer"].refresh = lambda *a, **k: None
    core.trackers = {}
    core._reproject(9.0, (0.0, 0.04, 0.0), rear, False)
    assert core.cargo.anchor == anchor


def test_a_live_core_with_no_adopted_mission_leaves_the_lift_alone():
    core, rear = ended()
    core.ms.lift_command = 0.05
    core.ms.phase = "transport"  # the ended mission's phase would ramp it to the lift target
    core.lift_and_timers(9.0, 1 / 120)
    assert core.ms.lift_command == pytest.approx(0.05)


def _compose(a, b):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (x + c * b[0] - s * b[1], y + s * b[0] + c * b[1], t + b[2])


def test_the_lower_test_flag_refuses_the_recovery_before_its_lowering():
    """Plan D14 U20 c (lifted): the test flag's "lower" mode leaves the pallet on the forks for the manual lowering."""
    core, rear = inserted_core(lift=0.10, pallet_z=0.05)
    core.hooks.test_recovery_blocked = "lower"
    stand(core, 6.0, rear)
    assert core.commands.status == "recovery_required" and core.commands.end_reason == "recovery:blocked:test_lower"
    assert core.ms.lift_command == pytest.approx(0.10) or core.ms.lift_command >= 0.0


def test_a_live_first_pause_in_the_lift_holds_within_the_tolerance_not_at_the_measurement():
    """Codex D14 impl round 5 P1: a STOP handled at the tick's start reached paused_tick first, which set the target
    to the measured lift (a loaded 33 mm sag lower) before the gate clipped it. Live, with the runner's tolerance,
    the first pause clips too; a batch run keeps the measured latch (the replay gate's)."""
    from test_core_pause import make as make_pause
    from forklift_core.mission.commands import Pause

    core, obs, _ = make_pause("lift", lift=0.0666)
    core.ms.lift_command = 0.100
    core.baseline = object()
    core.lift_hold_tolerance = lambda: 0.0434  # the runner's: loaded here
    core.commands.queue(Pause())
    core.commands.apply(5.0)
    core.paused_tick(5.0, np.array([0.0, 0.0, 0.0]), 0.0, True)
    assert core.ms.lift_command == pytest.approx(0.100)
    batch, _, _ = make_pause("lift", lift=0.0666)
    batch.ms.lift_command = 0.100
    batch.commands.queue(Pause())
    batch.commands.apply(5.0)
    batch.paused_tick(5.0, np.array([0.0, 0.0, 0.0]), 0.0, True)
    assert batch.ms.lift_command == pytest.approx(0.0666)
