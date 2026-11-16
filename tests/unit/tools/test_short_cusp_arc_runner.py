"""--short-cusp-arc in the runner (docs/plans/2026-11-16-aisle-short-cusp-arc.md 1): the argument, its record, the
near-tracking contract and the wiring (source checks; the runner needs Isaac)."""

import argparse
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))

import near_tracking  # noqa: E402
import run_transport  # noqa: E402

from forklift_core.planning.pallet_mission import make_transport_planner_config  # noqa: E402

RUNNER = (ROOT / "sim/isaac/run_transport.py").read_text()


def test_the_argument_takes_two_positive_metres():
    assert run_transport._short_cusp_arc("0.6,1.0") == (0.6, 1.0)
    for bad in ("0.6", "0.6,1.0,2", "0,1", "0.6,-1", "nan,1", "a,b", "0.6,inf"):
        with pytest.raises(argparse.ArgumentTypeError):
            run_transport._short_cusp_arc(bad)


def test_absent_it_is_not_recorded_given_it_is():
    off = run_transport.recorded_arguments(argparse.Namespace(short_cusp_arc=None, seed=3))
    assert "short_cusp_arc" not in off and off["seed"] == 3
    on = run_transport.recorded_arguments(argparse.Namespace(short_cusp_arc=(0.6, 1.0), seed=3))
    assert on["short_cusp_arc"] == (0.6, 1.0)


def test_the_near_tracking_contract_names_the_option():
    assert "short_cusp_arc" in near_tracking.RUN_SPECIFIC_ARGUMENTS


def test_the_runner_wiring():
    assert '"--short-cusp-arc"' in RUNNER and "type=_short_cusp_arc" in RUNNER
    assert '{"short_cusp_arc_m": args.short_cusp_arc[0], "short_cusp_arc_penalty_m": args.short_cusp_arc[1]}' in RUNNER
    assert 'state["planner_config"] = planner_config_record(planner_config)' in RUNNER
    assert 'state["travel_planner_config"] = planner_config_record(travel_config)' in RUNNER
    assert "asdict(planner_config)" not in RUNNER and "asdict(travel_config)" not in RUNNER


def test_the_mission_config_records_as_before_when_off():
    from dataclasses import asdict

    from forklift_core.planning.hybrid_astar import planner_config_record

    off = make_transport_planner_config(curvature_limit_inv_m=0.29, clearance_m=0.10, max_expansions=30000)
    rec = planner_config_record(off)
    assert "short_cusp_arc_m" not in rec and set(asdict(off)) - set(rec) == {"short_cusp_arc_m",
                                                                               "short_cusp_arc_penalty_m"}
    on = make_transport_planner_config(curvature_limit_inv_m=0.29, clearance_m=0.10, short_cusp_arc_m=0.6,
                                       short_cusp_arc_penalty_m=1.0)
    assert planner_config_record(on)["short_cusp_arc_penalty_m"] == 1.0
