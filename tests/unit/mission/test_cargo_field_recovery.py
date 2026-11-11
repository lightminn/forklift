"""Sim-to-real 4d-2b-2: the real truck's cancel boundary (lead decision 2, audit 4-3/4-4) -- empty forks by the
mission's state (no pocket entered, or the operator confirmed them out) back out; anything else is a field recovery."""

import pytest

from forklift_core.mission.cargo import CargoState

REAR = (0.0, 0.0, 0.0)


def cargo(real):
    c = CargoState()
    c.assume_trace_valid = not real
    c.field_recovery = real
    return c


# (phase reached, pocket entered, operator confirmed out) -> the real truck's decision; the simulation's for reference
TABLE = [
    ("observe", False, False, "cancelled", "cancelled"),
    ("approach", False, False, "cancelled", "cancelled"),
    ("insert", True, False, "recovery_required", "recover"),
    ("lift", True, False, "recovery_required", "recover"),
    ("extract", True, False, "recovery_required", "cancelled"),
    ("transport", True, False, "recovery_required", "cancelled"),
    ("lower", True, False, "recovery_required", "recover"),
    ("withdraw", True, False, "recovery_required", "recover"),
    ("settle", True, False, "recovery_required", "cancelled"),  # out by the tracker only: not confirmed
    ("settle", True, True, "cancelled", "cancelled"),  # the operator confirmed the forks out
    ("return_home", True, True, "cancelled", "cancelled"),
]
RELATION = {"observe": "free", "approach": "free", "insert": "constrained", "lift": "constrained",
            "extract": "carrying", "transport": "carrying", "lower": "constrained", "withdraw": "constrained",
            "settle": "free", "return_home": "free"}


@pytest.mark.parametrize("phase,entered,out,real_decision,sim_decision", TABLE)
def test_the_cancel_boundary_by_phase(phase, entered, out, real_decision, sim_decision):
    for real, want in ((True, real_decision), (False, sim_decision)):
        c = cargo(real)
        if entered:
            c.on_insert_start(REAR)
        c.relation = RELATION[phase]
        if out:
            c.confirm_out()
        assert c.cancel_decision(REAR, in_withdraw=False) == want, (phase, real)


def test_the_real_truck_refuses_a_submit_until_the_forks_are_confirmed_out():
    c = cargo(True)
    c.on_insert_start(REAR)
    c.relation = "carrying"
    assert c.submit_refusal() == "recovery_required"  # not cargo_held: no loaded resubmit on the real truck
    c.relation = "free"
    assert c.submit_refusal() == "recovery_required"
    c.confirm_out()
    assert c.submit_refusal() is None
    s = cargo(False)
    s.on_insert_start(REAR)
    s.relation = "carrying"
    assert s.submit_refusal() == "cargo_held"  # the simulation as before


def test_an_unknown_relation_is_a_field_recovery_even_before_a_pocket():
    c = cargo(True)
    c.invalidate("dropped")
    assert c.cancel_decision(REAR, in_withdraw=False) == "recovery_required"
