"""K1's runner side (docs/plans/2026-11-13-k1-near-loose-arrival.md 2.1-2.3): the argument and its checks, the near
contract, the smoke's detour size and the record-only safety watch. No Isaac."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402

import near_end_detour as nd  # noqa: E402
from near_loose_watch import NearLooseWatch  # noqa: E402


def test_the_argument_is_absent_by_default_and_needs_c2_on():
    args = _runner_args([])
    assert args.near_observe_arrival is None and args.inject_near_end_detour_size is None
    import run_transport
    assert run_transport.near_observe_arrival_of(args) is None
    for stand in ("off", "shadow"):
        with pytest.raises(SystemExit):
            _runner_args(["--near-observe-arrival", "0.05,0.05", "--near-end-stand", stand])
    args = _runner_args(["--near-observe-arrival", "0.05,0.05", "--near-end-stand", "on"])
    assert run_transport.near_observe_arrival_of(args) == (0.05, 0.05)


@pytest.mark.parametrize("bad", ["0.05", "0.05,0.05,0.05", "a,b", "nan,0.05", "0.05,inf", "0,0.05", "0.11,0.05",
                                 "0.05,0.2", "-0.01,0.05"])
def test_bad_values_are_refused(bad):
    with pytest.raises(SystemExit):
        _runner_args(["--near-observe-arrival", bad, "--near-end-stand", "on"])


def test_a_record_from_before_the_option_reads_absent():
    import run_transport
    from types import SimpleNamespace
    assert run_transport.near_observe_arrival_of(SimpleNamespace()) is None


def test_the_detour_size_needs_the_detour_and_is_checked():
    with pytest.raises(SystemExit):
        _runner_args(["--inject-near-end-detour-size", "0.04,0.07", "--near-end-stand", "on"])
    with pytest.raises(SystemExit):
        _runner_args(["--inject-near-end-detour", "--inject-near-end-detour-size", "0.3,0.07", "--near-end-stand", "on"])
    args = _runner_args(["--inject-near-end-detour", "--inject-near-end-detour-size", "0.04,0.07", "--near-end-stand", "on"])
    assert args.inject_near_end_detour_size == "0.04,0.07"


def test_the_detour_takes_its_size():
    d = nd.NearEndDetour(0.04, 0.07)
    assert (d.d_m, d.psi_rad) == (0.04, 0.07)
    assert (nd.NearEndDetour().d_m, nd.NearEndDetour().psi_rad) == (nd.D_M, nd.PSI_RAD)


def test_the_near_contract_and_the_getattr_reads():
    import near_tracking
    assert {"near_observe_arrival", "inject_near_end_detour_size"} <= set(near_tracking.RUN_SPECIFIC_ARGUMENTS)
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert "args.near_observe_arrival" not in source and "args.inject_near_end_detour_size" not in source


def test_without_the_option_the_step_watch_is_only_an_is_none_test():
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert "if near_loose_watch_ is not None and near_loose_watch_.active:" in source
    assert source.count("NearLooseWatch()") == 1
    assert "if near_observe_arrival_of(args) is not None:" in source


# ---- the watch ---------------------------------------------------------------------------------------------------


def test_the_watch_opens_at_an_intervention_and_closes_after_the_capture():
    w, state = NearLooseWatch(), {"near_capture": {"status": "pending"}}
    w.tick(1.0, state, (2.0, 0.0))
    assert not w.active and "near_loose_watch" not in state
    state["near_loose_refusals"] = [{}]
    w.tick(2.0, state, (2.0, 0.0))
    assert w.active and state["near_loose_watch"][0]["start_pallet_xy"] == [2.0, 0.0]
    w.step((2.003, 0.0), [], 2.01)
    w.step((2.0, 0.0), [], 2.02)
    w.tick(3.0, state, (2.0, 0.0))
    assert w.active  # still pending
    state["near_capture"]["status"] = "done"
    w.tick(4.0, state, (2.0, 0.0))
    (win,) = state["near_loose_watch"]
    assert not w.active and win["end_s"] == 4.0 and win["near_capture_status"] == "done"
    assert win["steps"] == 2 and abs(win["max_move_m"] - 0.003) < 1e-12


def test_a_move_out_and_back_counts_from_the_interventions_start():
    """Codex review: measured from the position at the intervention, not from the initial pallet pose."""
    w, state = NearLooseWatch(), {"near_capture": {"status": "pending"}, "near_loose_refusals": [{}]}
    w.tick(2.0, state, (2.01, 0.0))
    w.step((2.0, 0.0), [], 2.1)
    w.step((2.02, 0.0), [], 2.2)
    assert abs(state["near_loose_watch"][0]["max_move_m"] - 0.01) < 1e-12


def test_a_contact_that_comes_and_goes_inside_a_capture_is_kept():
    w, state = NearLooseWatch(), {"near_capture": {"status": "pending"}, "near_loose_refusals": [{}]}
    w.tick(2.0, state, (2.0, 0.0))
    w.step((2.0, 0.0), [], 2.1)
    w.step((2.0, 0.0), [{"part": "left_fork"}], 2.2)
    w.step((2.0, 0.0), [], 2.3)
    win = state["near_loose_watch"][0]
    assert win["contact_steps"] == 1 and win["first_contact"] == {"time_s": 2.2, "contacts": [{"part": "left_fork"}]}
    assert win["max_move_m"] == 0.0


def test_a_second_intervention_inside_an_open_window_is_part_of_it():
    w, state = NearLooseWatch(), {"near_capture": {"status": "pending"}, "near_loose_refusals": [{}]}
    w.tick(2.0, state, (2.0, 0.0))
    state["near_loose_refusals"].append({})
    w.tick(3.0, state, (2.0, 0.0))
    assert len(state["near_loose_watch"]) == 1
    state["near_capture"]["status"] = "failed"
    w.tick(4.0, state, (2.0, 0.0))
    w.tick(5.0, state, (2.0, 0.0))
    assert len(state["near_loose_watch"]) == 1 and not w.active
