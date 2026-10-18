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
