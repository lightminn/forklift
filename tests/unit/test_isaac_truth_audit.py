"""Priority-5 plan audit table (AST, no Isaac): with --grid-planning no planning or
acceptance input of the Isaac runner is built from the ground truth."""

import ast
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "sim/isaac/run_transport.py"
TREE = ast.parse(SCRIPT.read_text())
SOURCE = SCRIPT.read_text()
TRUTH = ("obstacles", "scenario.pickup", "truth_rear", "ppos", "pallet_yaw", "pickup_obstacle", "checked_obstacles")


# The mission logic moved out of the runner (M-a1) plans through the same ports: its calls are audited too.
CORE_TREE = ast.parse((Path(__file__).resolve().parents[2] / "src/forklift_core/mission/core.py").read_text())


def _hook_forwards(tree) -> set:
    """The runner's hooks for the mission core: ``lambda *a, **k: f(*a, **k)`` -- a lambda whose whole body
    is one call passing exactly the lambda's own *args / **kwargs on, nothing added (Codex slice-3/4a P3:
    an argument shape alone, ``plan_transport(*obstacles)``, is not that)."""
    ids = set()
    for lam in ast.walk(tree):
        if not isinstance(lam, ast.Lambda) or not isinstance(lam.body, ast.Call):
            continue
        call, a = lam.body, lam.args
        star = a.vararg.arg if a.vararg else None
        kw = a.kwarg.arg if a.kwarg else None
        positional = [p.arg for p in a.args]
        args_ok = [ast.unparse(x) for x in call.args] == positional + ([f"*{star}"] if star else [])
        kws_ok = [(k.arg, ast.unparse(k.value)) for k in call.keywords] == ([(None, kw)] if kw else [])
        if args_ok and kws_ok and (star or kw or positional):
            ids.add(id(call))
    return ids


def _calls(prefix):
    for tree in (TREE, CORE_TREE):
        forwards = _hook_forwards(tree)
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and id(node) not in forwards:
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
    # The delivery docking is a MissionCore method since M-a1 slice 6b.
    core = (Path(__file__).resolve().parents[2] / "src/forklift_core/mission/core.py").read_text()
    i = core.index("swept_checker = GridFootprintChecker(")
    head = core[core.rindex("if grid_planning:", 0, i) : i]
    assert "grid_kwargs(\"docking\")" in core[i : i + 200] and len(head) < 400


def test_runtime_viewpoints_cannot_join_grid_planning():
    assert "--runtime-viewpoints uses the true pallet rectangle; not with --grid-planning" in SOURCE


def test_grid_planning_plans_on_perception_only():
    # Codex checkpoint 14: without recognition, or with an oracle target, the
    # planner reads the pickup pallet's true pose.
    assert '"--grid-planning needs --use-perception and --planning-target perception"' in SOURCE
