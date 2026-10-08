"""CPU proxy for run-time viewpoints (docs/plans/2026-10-03-runtime-observation-viewpoints.md)."""

import pytest

from tools import observation_candidate_design as design
from tools import observation_viewpoint_eval as tool


def test_the_next_frozen_evaluation_seeds_are_refused():
    evaluator = design.Evaluator(config=None, reserved=tool.RESERVED_SEEDS)
    with pytest.raises(ValueError, match="reserved"):
        evaluator.scenario(6000)
    with pytest.raises(SystemExit):
        tool.main(["--record", "unused.json", "--seeds", "5998:6001"])


def test_seeds_used_by_earlier_evaluations_can_be_diagnosed():
    """1028 was a G5 seed; this tool reserves only 6000-6029."""
    evaluator = design.Evaluator(config=None, reserved=tool.RESERVED_SEEDS)
    assert 1028 not in evaluator.reserved and 110 not in evaluator.reserved
    # The G4 design keeps refusing its own reserved seeds.
    assert 1028 in design.Evaluator(config=None).reserved


def test_only_epal6_records_are_accepted():
    """The visibility boxes are EPAL 6's; another shape would be scored wrongly."""
    with pytest.raises(ValueError, match="EPAL 6"):
        tool.evaluator_for({"arguments": {"pallet_urdf": "sim/models/t11_pallet/pallet.urdf"}})


def test_the_fixed_list_matches_the_runner_default():
    import ast
    from pathlib import Path

    from forklift_core.planning.observation_viewpoints import DEFAULT_OBSERVATION_WAYPOINTS

    script = Path(design.ROOT / "sim/isaac/run_transport.py").read_text()
    assigns = [
        ast.unparse(node.value)
        for node in ast.walk(ast.parse(script))
        if isinstance(node, ast.Assign)
        and ast.unparse(node.targets[0]) == "args.observation_waypoints"
    ]
    assert "[list(w) for w in DEFAULT_OBSERVATION_WAYPOINTS]" in assigns
    assert [tuple(w) for w in DEFAULT_OBSERVATION_WAYPOINTS] == list(tool.FIXED)
