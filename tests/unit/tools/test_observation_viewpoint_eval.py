"""CPU proxy for run-time viewpoints (docs/plans/2026-10-03-runtime-observation-viewpoints.md)."""

import pytest

from tools import observation_candidate_design as design
from tools import observation_viewpoint_eval as tool


def test_the_next_frozen_evaluation_seeds_are_refused():
    evaluator = design.Evaluator(config=None, reserved=tool.RESERVED_SEEDS)
    with pytest.raises(ValueError, match="reserved"):
        evaluator.scenario(5000)
    with pytest.raises(SystemExit):
        tool.main(["--record", "unused.json", "--seeds", "4998:5001"])


def test_seeds_used_by_earlier_evaluations_can_be_diagnosed():
    """1028 was a G5 seed; this tool reserves only 5000-5029."""
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

    script = Path(design.ROOT / "sim/isaac/run_transport.py").read_text()
    (assign,) = [
        node
        for node in ast.walk(ast.parse(script))
        if isinstance(node, ast.Assign)
        and ast.unparse(node.targets[0]) == "args.observation_waypoints"
        and isinstance(node.value, ast.List)
    ]
    assert [tuple(ast.literal_eval(e)) for e in assign.value.elts] == list(tool.FIXED)
