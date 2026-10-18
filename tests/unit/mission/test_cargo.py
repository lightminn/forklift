"""The fork relation and the load as session state (persistent console plan D5a, 4th ed.): driven by the mission's
transitions and the lift/lower verdicts, kept across a mission's end, invalidated where a pallet check fails."""

import pytest

from forklift_core.mission.cargo import CargoState

NORMAL = [("observe", "approach"), ("approach", "insert"), ("insert", "lift"), ("lift", "extract"),
          ("extract", "transport"), ("transport", "lower"), ("lower", "withdraw"), ("withdraw", "settle"),
          ("settle", "return_home"), ("return_home", "home_settle"), ("home_settle", "complete")]


def run(until=None):
    cargo = CargoState()
    seen = []
    for a, b in NORMAL:
        cargo.on_transition(a, b, "P-1")
        seen.append((b, cargo.relation, cargo.load))
        if b == until:
            break
    return cargo, seen


def test_a_session_starts_free_and_empty():
    cargo = CargoState()
    assert (cargo.relation, cargo.load, cargo.pallet_id) == ("free", "none", None)
    assert cargo.submit_refusal() is None and cargo.release_allowed() and not cargo.loaded_outline()


def test_a_normal_mission_walks_the_table():
    _, seen = run()
    assert seen == [
        ("approach", "free", "none"), ("insert", "constrained", "none"), ("lift", "constrained", "acquiring"),
        ("extract", "carrying", "confirmed"), ("transport", "carrying", "confirmed"),
        ("lower", "constrained", "releasing"), ("withdraw", "constrained", "none"), ("settle", "free", "none"),
        ("return_home", "free", "none"), ("home_settle", "free", "none"), ("complete", "free", "none")]


@pytest.mark.parametrize("until, refusal, release, loaded", [
    ("approach", None, True, False),
    ("insert", "recovery_required", False, False),
    ("lift", "recovery_required", False, True),
    ("transport", "cargo_held", True, True),
    ("lower", "recovery_required", False, True),
    ("withdraw", "recovery_required", False, False),
    ("settle", None, True, False),
])
def test_what_a_mission_ended_in_each_phase_leaves(until, refusal, release, loaded):
    cargo, _ = run(until)
    assert cargo.submit_refusal() == refusal
    assert cargo.release_allowed() is release
    assert cargo.loaded_outline() is loaded
    assert cargo.pallet_id == (None if until == "approach" else "P-1")  # set once the forks went to a pallet


def test_a_pallet_failure_makes_it_unknown_wherever_it_is_seen():
    cargo, _ = run("transport")
    cargo.invalidate("Pallet dropped during transport")
    assert (cargo.relation, cargo.load) == ("unknown", "unknown")
    assert cargo.submit_refusal() == "recovery_required" and not cargo.release_allowed() and cargo.loaded_outline()
    fresh = CargoState()
    fresh.invalidate("Forbidden fork/pallet contact in observe: [...]")  # a free truck driven into a pallet by hand
    assert fresh.relation == "unknown"


def test_the_safety_fields_say_what_they_rest_on():
    cargo, _ = run("transport")
    fields = cargo.safety_fields()
    assert fields["load"] == {"loaded": True, "basis": "phase+cargo", "pallet_id": "P-1", "state": "confirmed"}
    assert fields["fork_in_pocket"] == {"value": True, "basis": "phase+cargo", "relation": "carrying"}
    free, _ = run("settle")
    assert free.safety_fields()["fork_in_pocket"]["value"] is False
    assert free.safety_fields()["load"]["loaded"] is False


def test_a_lift_whose_verdict_fails_stays_acquiring():
    cargo, _ = run("lift")  # the verdict raises before lift -> extract
    assert (cargo.relation, cargo.load) == ("constrained", "acquiring")


# --- plan D4 (6b, 6th ed.): the insert trace, the depth, the recovery path -------------------------------------------
import math  # noqa: E402

import numpy as np  # noqa: E402


def inserted(depth_m, *, heading=0.0, y0=0.0):
    """A cargo that entered straight along x from the approach end at (0, y0) to ``depth_m`` past the face."""
    cargo = CargoState()
    cargo.on_transition("approach", "insert", "P-1")
    cargo.on_insert_start((0.0, y0, heading))
    travel = depth_m + 0.10
    for s in np.arange(0.0, travel + 1e-9, 0.01):
        cargo.observe("insert", (s * math.cos(heading), y0 + s * math.sin(heading), heading))
    return cargo, (travel * math.cos(heading), y0 + travel * math.sin(heading), heading)


def test_the_depth_is_the_net_displacement_along_the_insert_axis():
    cargo, end = inserted(0.30)
    assert cargo.depth(end) == pytest.approx(0.30)
    # standing still with noise: a sum of moves grows, the net displacement does not (Codex 6b round 1 P1-1)
    for k in range(200):
        cargo.observe("insert", (end[0] + 0.002 * (-1) ** k, end[1], 0.0))
    assert cargo.depth(end) == pytest.approx(0.30)


def test_the_trace_is_recorded_every_two_centimetres_with_its_depth():
    cargo, _ = inserted(0.30)
    depths = [d for _, d in cargo.trace]
    assert depths[0] == pytest.approx(-0.10) and depths[-1] == pytest.approx(0.30, abs=0.02)
    assert all(b - a == pytest.approx(0.02, abs=0.011) for a, b in zip(depths, depths[1:]))


def test_a_pickup_recovery_retraces_the_trace_to_its_margin_behind_the_face():
    cargo, end = inserted(0.30)
    poses = cargo.recovery_path(end, step_m=0.02)
    assert poses[0][0] == pytest.approx(end[0], abs=0.011)  # starts where the forks are now
    u = cargo.band(end)
    assert poses[-1][0] == pytest.approx(-u, abs=1e-6)  # depth -(0.10 + u): u behind the trace's start
    assert all(p[1] == pytest.approx(0.0) for p in poses)


def test_the_destination_placement_is_fixed_once_and_cut_at_the_depth_now():
    """Codex 6b round 3 P2-1: placed at -> lower once; a withdraw or a pause never re-fits it to the pose now."""
    cargo, end = inserted(0.33)
    cargo.on_transition("insert", "lift", "P-1")
    cargo.on_insert_end(end)
    cargo.on_transition("lift", "extract", "P-1")
    cargo.on_transition("extract", "transport", "P-1")
    dest = (10.0, 5.0, math.pi / 2)  # the pallet now at the destination, the truck facing +y
    cargo.on_transition("transport", "lower", "P-1")
    cargo.on_lower_start(dest)
    cargo.on_transition("lower", "withdraw", "P-1")
    back = (10.0, 5.0 - 0.40, math.pi / 2)  # withdrawn 0.40 m: depth 0.33 - 0.40 = -0.07, still a recovery
    assert cargo.depth(back) == pytest.approx(-0.07)
    poses = cargo.recovery_path(back, step_m=0.02)
    assert poses[0][1] == pytest.approx(back[1], abs=0.011) and poses[0][0] == pytest.approx(10.0)
    assert poses[-1][1] == pytest.approx(5.0 - 0.33 - 0.10 - cargo.band(back), abs=1e-6)


def test_a_cancel_already_past_the_margin_needs_no_recovery_and_at_the_end_it_does_not_move():
    cargo, end = inserted(0.33)
    out = (end[0] - 0.33 - 0.10 - 0.20, 0.0, 0.0)
    assert cargo.cancel_decision(out, in_withdraw=True) == "cancelled"
    assert cargo.relation == "free"
    cargo2, end2 = inserted(0.33)
    assert cargo2.cancel_decision(end2, in_withdraw=False) == "recover"
    at_end = (end2[0] - 0.33 - 0.10 - cargo2.band(end2) - 0.001, 0.0, 0.0)
    assert cargo2.recovery_path(at_end, step_m=0.02) == []  # at or past the end: done without moving


def test_an_invalid_trace_or_an_unknown_relation_is_recovery_required():
    cargo, end = inserted(0.33)
    cargo.trace_valid = False
    assert cargo.cancel_decision(end, in_withdraw=False) == "recovery_required"
    unknown = CargoState()
    unknown.invalidate("Pallet dropped during transport")
    assert unknown.cancel_decision((0.0, 0.0, 0.0), in_withdraw=False) == "recovery_required"
    free = CargoState()
    assert free.cancel_decision((0.0, 0.0, 0.0), in_withdraw=False) == "cancelled"


def test_a_cancel_before_the_first_record_backs_out_along_the_anchor_heading():
    cargo = CargoState()
    cargo.on_transition("approach", "insert", "P-1")
    cargo.on_insert_start((0.0, 0.0, 0.1))
    cargo.observe("insert", (0.005, 0.0005, 0.1))
    poses = cargo.recovery_path((0.005, 0.0005, 0.1), step_m=0.02)
    assert len(poses) >= 2 and all(p[2] == pytest.approx(0.1) for p in poses)
