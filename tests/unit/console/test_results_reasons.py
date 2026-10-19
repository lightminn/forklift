"""Persistent console plan D13 item 3: every reason a mission result can carry has an explanation on the screen
(console/web/js/results.js REASONS). The reasons are read from where results are made, by AST (keyword arguments
too -- Codex D13 impl round 2 P3): every ``terminate``, ``recovery_failed`` and ``end_mission`` call. A reason that is
a string, or starts with one, must be explained; one made at run time must be listed below with where its values
come from, so a new kind of reason cannot slip past unexplained."""

import ast
import json
import re
from pathlib import Path

from forklift_core.mission import delivery
from forklift_core.mission.failures import PREFIXES

ROOT = Path(__file__).resolve().parents[3]
SOURCES = ("src/forklift_core/mission/core.py", "src/forklift_core/mission/commands.py",
           "sim/isaac/run_transport.py", "sim/isaac/live_runtime.py")
# the call, which argument is the reason, and what the call puts before it
CALLS = {"terminate": (2, ""), "recovery_failed": (1, "recovery:"), "end_mission": (2, "")}
# reasons made at run time, by (file, expression): where their values come from (each covered below)
DYNAMIC = {
    ("sim/isaac/run_transport.py", "reason_"): "a raised failure classified MISSION or SCENE (failures.PREFIXES)",
    ("sim/isaac/run_transport.py", "core.commands.end_reason"): "a reason one of these calls already made",
    ("sim/isaac/live_runtime.py", "cmds.end_reason"): "a reason one of these calls already made",
    ("sim/isaac/live_runtime.py", "reason"): "end_mission's own argument, or the delivery verdict's reason",
    ("src/forklift_core/mission/core.py", "self.cargo.relation"): "recovery:<relation> -- under the recovery: head",
}


def table_heads() -> list[str]:
    source = (ROOT / "console/web/js/results.js").read_text()
    body = source[source.index("export const REASONS = {"):source.index("};", source.index("export const REASONS"))]
    return [json.loads(m.group(1)) for m in re.finditer(r'^\s*("(?:[^"\\]|\\.)*")\s*:', body, re.M)]


def heads_of(node, prefix: str, file: str, unknown: list) -> set[str]:
    """The fixed heads a reason expression can start with; run-time parts not in DYNAMIC go to ``unknown``."""
    if isinstance(node, ast.Constant):
        return set() if node.value is None else {prefix + str(node.value)}
    if isinstance(node, ast.JoinedStr):
        first = node.values[0] if node.values else None
        if isinstance(first, ast.Constant):
            return {prefix + str(first.value)}
        if prefix:
            return {prefix}  # f"{...}" after a fixed prefix: explained by that prefix
        unknown.append((file, ast.unparse(node)))
        return set()
    if isinstance(node, ast.IfExp):
        return heads_of(node.body, prefix, file, unknown) | heads_of(node.orelse, prefix, file, unknown)
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = heads_of(node.left, prefix, file, unknown)
        if left:
            return left
    key = (file, ast.unparse(node))
    if key in DYNAMIC:
        return {prefix} if prefix else set()
    if isinstance(node, ast.IfExp) or prefix:
        return {prefix}
    unknown.append(key)
    return set()


def result_reasons() -> tuple[set[str], list]:
    found, unknown = set(), []
    for file in SOURCES:
        for node in ast.walk(ast.parse((ROOT / file).read_text())):
            if not isinstance(node, ast.Call):
                continue
            name = node.func.attr if isinstance(node.func, ast.Attribute) else getattr(node.func, "id", None)
            if name not in CALLS:
                continue
            index, prefix = CALLS[name]
            arg = next((k.value for k in node.keywords if k.arg in ("reason", "why")), None)
            if arg is None and len(node.args) > index:
                arg = node.args[index]
            if arg is not None:
                found |= heads_of(arg, prefix, file, unknown)
    return found, unknown


def explained(reason: str, heads: list[str]) -> bool:
    return any(reason.startswith(h) for h in heads)


def test_every_reason_a_result_can_carry_is_explained():
    heads = table_heads()
    found, unknown = result_reasons()
    assert not unknown, f"reasons made at run time with no entry in DYNAMIC: {unknown}"
    wanted = found | set(PREFIXES) | {delivery.MISSED, delivery.NOT_GROUNDED, delivery.FORK_INSIDE}
    missing = sorted(r for r in wanted if not explained(r, heads))
    assert not missing, missing


def test_the_scan_finds_the_calls_in_every_shape():
    found, _ = result_reasons()
    assert {"recovery:clear_out:budget", "recovery:trace_invalid", "prepare:timeout", "prepare:error:",
            "prepare:all_candidates_failed:", "prepare:teleop", "operator", "session_end"} <= found


def test_a_new_reason_by_keyword_or_at_run_time_is_caught(tmp_path, monkeypatch):
    """Codex D13 impl round 2 P3: cmds.terminate("failed", t, reason="...") passed the old regex scan."""
    found, unknown = set(), []
    tree = ast.parse('cmds.terminate("failed", t, reason="d13_unexplained_new_reason")\n'
                     'cmds.terminate("failed", t, made_now())\n')
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "attr", None) == "terminate":
            arg = next((k.value for k in node.keywords if k.arg == "reason"), None) or node.args[2]
            found |= heads_of(arg, "", "x.py", unknown)
    assert "d13_unexplained_new_reason" in found and not explained("d13_unexplained_new_reason", table_heads())
    assert unknown == [("x.py", "made_now()")]


def test_every_classified_head_has_its_own_entry():
    """A classified head explained only by a shorter generic one would lose its meaning (near_field_contract is a
    session failure, near_field_ a mission's)."""
    assert not sorted(set(PREFIXES) - set(table_heads()))
