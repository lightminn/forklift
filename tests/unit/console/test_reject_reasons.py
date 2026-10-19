"""Persistent console plan D3 item 5 (U19): every reason the vehicle can refuse a request with has a Korean text on
the screen (console/web/js/profile.js REJECT). The reasons are read from the source: the arbiter's ``_reply`` reasons,
and the values that flow into them followed to where they are made (the mission commands' returns, the targets
check, the cargo's submit refusal, the message check, the request ledger). A value no rule below explains fails."""

import ast
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def parse(rel):
    return ast.parse((ROOT / rel).read_text())


def strings(node, unknown, where):
    """The reason strings an expression can be; a run-time value goes to ``unknown``."""
    if isinstance(node, ast.Constant):
        return set() if node.value is None else {str(node.value)}
    if isinstance(node, ast.IfExp):
        return strings(node.body, unknown, where) | strings(node.orelse, unknown, where)
    unknown.append((where, ast.unparse(node)))
    return set()


def returns(rel, names, unknown):
    found = set()
    for node in ast.walk(parse(rel)):
        if isinstance(node, ast.FunctionDef) and node.name in names:
            for ret in ast.walk(node):
                if isinstance(ret, ast.Return) and ret.value is not None:
                    found |= strings(ret.value, unknown, (rel, node.name))
    return found


# run-time values, by (file, function or "_reply"), and the sources they come from
FLOWS = {
    ("src/forklift_core/safety/arbiter.py", "problem"): ("src/forklift_core/safety/arbiter.py", {"_generation_problem"}),
    ("src/forklift_core/safety/arbiter.py", "verdict"): "ledger",  # RequestLedger.admit: stale / busy
    ("src/forklift_core/safety/arbiter.py", "rec['reason']"): "commands",
    (("src/forklift_core/mission/commands.py", "_submit"), "context.submit_refusal"): (
        "src/forklift_core/mission/cargo.py", {"submit_refusal"}),
    (("src/forklift_core/mission/commands.py", "_submit"), "refused"): (
        "src/forklift_core/mission/targets.py", {"check_submit"}),
}
COMMANDS = {"_submit", "_pause", "_resume", "_cancel", "_hold_on", "_hold_off"}


def vehicle_reasons():
    unknown, found = [], set()
    arbiter = "src/forklift_core/safety/arbiter.py"
    for node in ast.walk(parse(arbiter)):
        if isinstance(node, ast.Call) and getattr(node.func, "attr", None) == "_reply" and len(node.args) >= 3:
            found |= strings(node.args[2], unknown, arbiter)
    pending, done = list(unknown), set()
    unknown.clear()
    while pending:
        key = pending.pop()
        if key in done:
            continue
        done.add(key)
        flow = FLOWS.get(key)
        if flow is None:
            unknown.append(key)
        elif flow == "ledger":
            found |= {"stale", "busy"}
        elif flow == "commands":
            more = []
            found |= returns("src/forklift_core/mission/commands.py", COMMANDS, more)
            pending.extend(more)
        else:
            rel, names = flow
            more = []
            if names is None:
                tree = parse(rel)
                for ret in ast.walk(tree):
                    if isinstance(ret, ast.Return) and ret.value is not None:
                        found |= strings(ret.value, more, rel)
            else:
                found |= returns(rel, names, more)
            pending.extend(more)
    return found, unknown


def table():
    source = (ROOT / "console/web/js/profile.js").read_text()
    body = source[source.index("export const REJECT = {"):source.index("};", source.index("export const REJECT"))]
    keys = []
    for m in re.finditer(r'^\s*("(?:[^"\\]|\\.)*"|[A-Za-z_]+)\s*:', body, re.M):
        k = m.group(1)
        keys.append(json.loads(k) if k.startswith('"') else k)
    return keys


def explained(reason, keys):
    return reason in keys or any(k.endswith(":") and reason.startswith(k) for k in keys)


def test_every_reason_the_vehicle_can_refuse_with_has_a_text():
    found, unknown = vehicle_reasons()
    assert not unknown, f"run-time refusal reasons with no rule in FLOWS: {unknown}"
    keys = table()
    missing = sorted(r for r in found if not explained(r, keys))
    assert not missing, missing


def test_the_scan_reaches_through_the_flows():
    """Codex D3 round 1 P2-3: not_running (a pause's return) and cargo_held (the cargo's refusal) were missed."""
    found, _ = vehicle_reasons()
    assert {"not_running", "cargo_held", "already_there", "destination_occupied", "localization_lost", "stale",
            "busy", "no_feedback:rear_axle_speed", "stale_targets", "invalid"} <= found
