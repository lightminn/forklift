"""Priority-5 plan audit table (AST, no Isaac): with --grid-planning no planning or
acceptance input of the Isaac runner is built from the ground truth."""

import ast
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "sim/isaac/run_transport.py"
TREE = ast.parse(SCRIPT.read_text())
SOURCE = SCRIPT.read_text()
TRUTH = ("obstacles", "scenario.pickup", "truth_rear", "ppos", "pallet_yaw", "pickup_obstacle", "checked_obstacles")


def _calls(prefix):
    for node in ast.walk(TREE):
        if isinstance(node, ast.Call):
            f = node.func
            name = f.id if isinstance(f, ast.Name) else (f.attr if isinstance(f, ast.Attribute) else "")
            if name.startswith(prefix):
                yield node


def test_every_plan_sees_the_grid_world_without_props():
    plans = list(_calls("plan_"))
    assert len(plans) >= 20
    for call in plans:
        first = call.args[0]
        assert isinstance(first, ast.Call) and getattr(first.func, "id", "") == "grid_world", (
            call.lineno, ast.unparse(first))
    grid_world = next(n for n in ast.walk(TREE) if isinstance(n, ast.FunctionDef) and n.name == "grid_world")
    assert "replace(sc, props=()) if grid_planning else sc" in ast.unparse(grid_world)


def test_no_plan_takes_a_ground_truth_argument():
    for call in _calls("plan_"):
        for arg in list(call.args) + [k.value for k in call.keywords]:
            text = ast.unparse(arg)
            for name in TRUTH:
                assert name not in text.replace("pickup_obstacle=", ""), (call.lineno, name, text)


def test_the_planner_pickup_obstacle_is_the_zone_prior_or_the_estimate():
    grid_kwargs = next(n for n in ast.walk(TREE) if isinstance(n, ast.FunctionDef) and n.name == "grid_kwargs")
    text = ast.unparse(grid_kwargs)
    assert "pickup_zone if known is None else Rectangle(" in text
    assert "scenario.pickup" not in text


def test_the_delivery_docking_sweep_uses_the_grid_when_grid_planning():
    i = SOURCE.index("swept_checker = GridFootprintChecker(")
    head = SOURCE[SOURCE.rindex("if grid_planning:", 0, i) : i]
    assert "grid_kwargs(\"docking\")" in SOURCE[i : i + 200] and len(head) < 400


def test_runtime_viewpoints_cannot_join_grid_planning():
    assert "--runtime-viewpoints uses the true pallet rectangle; not with --grid-planning" in SOURCE
