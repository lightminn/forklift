"""docs/design/2026-11-19-real-loop.md 6: the tick's hook table over the whole S10-A run, and the common partial order
of the core calls in the real loop and the Isaac loop (the contract's AST check)."""

import ast
from pathlib import Path

from forklift_core.live import real_assembly as asm
from forklift_core.live import real_autonomy as ra
from forklift_core.mission.hook_contract import HOOKS

from real_loop_rig import Rig

ROOT = Path(__file__).resolve().parents[3]


def test_the_whole_s10_a_run_reads_only_the_tables_hooks():
    r = Rig(frames=True, operator=[(1, "confirm")])
    read = set()
    base = type(r.core.hooks).__getattr__  # FailClosedHooks

    def recording(self, name):
        read.add(name)
        return base(self, name)

    type(r.core.hooks).__getattr__ = recording
    try:
        r.start()
        r.submit()
        for _ in range(4000):
            r.tick()
            if r.permit.state == ra.LATCHED:
                break
    finally:
        type(r.core.hooks).__getattr__ = base
    assert r.core.commands.end_reason == "s10_lifted" or r.loop.session.results[-1]["reason"] == "s10_lifted"
    table = asm.TICK_HOOKS | asm.ADOPTION_HOOKS | asm.STAND_AND_END_HOOKS | asm.ADOPTION_OPTIONAL_ABSENT
    evaluation = {n for n, (cls, _) in HOOKS.items() if cls.value == "eval"}
    assert read - evaluation <= table, sorted(read - evaluation - table)


CALLS = ("slam_replies", "release_gate", "job_hold", "follow", "docking_and_stall", "cusp_and_observe_recovery",
         "command", "arrive", "lift_and_timers", "permission", "assemble_command", "observe_motion", "safety_state")


def first_lines(path: Path, function: str | None = None, after: str | None = None) -> dict:
    """The first line of each call; ``after``: only calls past the first call of that name (the control section)."""
    tree = ast.parse(path.read_text())
    if function is not None:
        tree = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == function)
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr in CALLS and isinstance(n.func.value, ast.Name) and n.func.value.id == "core"]
    start = min((n.lineno for n in calls if n.func.attr == after), default=0) if after else 0
    out = {}
    for node in calls:
        if node.lineno >= start:
            out.setdefault(node.func.attr, node.lineno)
            out[node.func.attr] = min(out[node.func.attr], node.lineno)
    return out


def test_the_real_loop_calls_the_core_in_the_isaac_loops_order():
    real = {}
    src = ROOT / "src/forklift_core/live/real_loop.py"
    for fn in ("_control_and_output", "_assemble_and_arbitrate"):
        for k, v in first_lines(src, fn).items():
            real[k] = (0 if fn == "_control_and_output" else 100000) + v
    isaac = first_lines(ROOT / "sim/isaac/run_transport.py", "drive", after="slam_replies")
    common = [c for c in CALLS if c in real and c in isaac]
    assert len(common) >= 10, common
    assert sorted(common, key=real.get) == sorted(common, key=isaac.get)
