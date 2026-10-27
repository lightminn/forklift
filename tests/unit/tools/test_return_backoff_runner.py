"""The runner's ``--return-start-backoff`` (docs/plans/2026-10-27-return-start-withdraw-extension.md R1, R2): off by
default and in a record from before it, the cap when on, and the planners' keyword passed only when on -- to the mission
plan, the core's planner hook and the loaded preparation."""

import sys
from argparse import Namespace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402

from forklift_core.planning.pallet_mission import RETURN_START_BACKOFF_CAP_M  # noqa: E402


def test_off_by_default_and_the_cap_when_on():
    import run_transport

    assert _runner_args([]).return_start_backoff == "off"
    on = _runner_args(["--return-start-backoff", "on"])
    assert run_transport.return_start_backoff_m_of(on) == RETURN_START_BACKOFF_CAP_M
    assert run_transport.return_start_backoff_kw(on) == {"return_start_backoff_m": RETURN_START_BACKOFF_CAP_M}
    assert run_transport.return_start_backoff_kw(_runner_args([])) == {}
    assert run_transport.return_start_backoff_m_of(Namespace()) == 0.0  # a record from before the option


def test_the_mission_planners_get_it():
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert source.count("**return_start_backoff_kw(args)") == 2  # the first mission plan, the loaded preparation
    assert "return_start_backoff_m=return_start_backoff_m_of(args)" in source  # the core's option
    # the core's own mission plans (observe, its worker job, the blocked replan and its tight retry) add it themselves,
    # as they add the exit prefix: the runner's hook stays a pass-through
    core = (ROOT / "src/forklift_core/mission/core.py").read_text()
    assert core.count("**self._mission_backoff_kwargs()") == 4


def test_the_first_mission_plan_records_its_attempts():
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    first = source[source.index('        state.setdefault("planning_traces", []).append(planning_trace)\n'):][:600]
    assert 'note_trace_backoffs(state, "mission_plan", planning_trace, plans.success)' in first
