"""Persistent console plan D3 item 5 (U19): every reason a vehicle can refuse a request with has a Korean text on the
screen (console/web/js/profile.js REJECT). The reasons are read from the source -- the live arbiter's ``_reply`` and
the fake vehicle's, positional or ``reason=`` -- and a run-time value is followed to where it is made: a name only
through the assignment in the same function that FLOWS names (Codex D3 impl round 1 P2: a name matched by spelling
alone let a new ``problem = "..."`` through), into the mission commands' returns, the targets check, the cargo's
refusal and the request ledger. A value no rule explains fails the test."""

import ast
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ARBITER = "src/forklift_core/safety/arbiter.py"
MOCK = "console/mock_vehicle/core.py"
COMMANDS = ("src/forklift_core/mission/commands.py",
            frozenset({"_submit", "_pause", "_resume", "_cancel", "_hold_on", "_hold_off"}))
# a run-time value, by (file, the expression), and where it is made: ("call", the called method's name -- the name
# must be assigned from that call in the same function -- then the sources to read), or ("direct", ...)
FLOWS = {
    (ARBITER, "problem"): ("call", "_generation_problem", [(ARBITER, frozenset({"_generation_problem"}))]),
    (ARBITER, "verdict"): ("call", "admit", "ledger"),
    (ARBITER, "rec['reason']"): ("direct", None, [COMMANDS]),
    ("src/forklift_core/mission/commands.py", "context.submit_refusal"): (
        "direct", None, [("src/forklift_core/mission/cargo.py", frozenset({"submit_refusal"}))]),
    ("src/forklift_core/mission/commands.py", "refused"): (
        "call", "check_submit", [("src/forklift_core/mission/targets.py", frozenset({"check_submit"}))]),
    (MOCK, "verdict"): ("call", "admit", "ledger"),
    (MOCK, "reason"): [("call", "_valid", [(MOCK, frozenset({"_valid"}))]),
                       ("call", "_teleop_reason", [(MOCK, frozenset({"_teleop_reason"}))])],
}


REASON_ARG = {ARBITER: 2, MOCK: 1}


def parse(rel):
    return ast.parse((ROOT / rel).read_text())


def enclosing(tree):
    """Each node's innermost enclosing function."""
    owner = {}
    for fn in ast.walk(tree):
        if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for node in ast.walk(fn):
                owner[id(node)] = fn  # a nested function walked later overrides: innermost wins
    return owner


def called(value):
    func = value.func if isinstance(value, ast.Call) else None
    return getattr(func, "attr", None) or getattr(func, "id", None)


def assigned_from(fn, name, method):
    """Whether every write to ``name`` in ``fn`` is an assignment from a call to ``method`` (a tuple target's first
    element too). Any other write -- annotated, augmented, walrus, a loop, ``with`` or comprehension target, or a
    later element of a tuple -- counts as unexplained (Codex D3 impl round 2 P2: ``problem: str = "..."`` slipped)."""
    sources = []

    def names_in(target):
        return [n for n in ast.walk(target) if isinstance(n, ast.Name) and n.id == name]

    for node in ast.walk(fn):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    if target.id == name:
                        sources.append(node.value)
                elif names_in(target):
                    elts = list(getattr(target, "elts", []))
                    first = elts[0] if elts else None
                    ok = isinstance(first, ast.Name) and first.id == name and len(names_in(target)) == 1
                    sources.append(node.value if ok else None)
        elif isinstance(node, (ast.AnnAssign, ast.AugAssign)) and names_in(node.target):
            sources.append(None)
        elif isinstance(node, ast.NamedExpr) and node.target.id == name:
            sources.append(None)
        elif isinstance(node, (ast.For, ast.AsyncFor, ast.comprehension)) and names_in(node.target):
            sources.append(None)
        elif isinstance(node, ast.withitem) and node.optional_vars is not None and names_in(node.optional_vars):
            sources.append(None)
        elif isinstance(node, (ast.MatchAs, ast.MatchStar)) and node.name == name:
            sources.append(None)  # a pattern capture (Codex D3 impl round 3 P2)
        elif isinstance(node, ast.MatchMapping) and node.rest == name:
            sources.append(None)
    return bool(sources) and all(v is not None and called(v) == method for v in sources)


def strings(node, rel, fn, unknown, pending):
    if isinstance(node, ast.Constant):
        return set() if node.value is None else {str(node.value)}
    if isinstance(node, ast.IfExp):
        return strings(node.body, rel, fn, unknown, pending) | strings(node.orelse, rel, fn, unknown, pending)
    key = (rel, ast.unparse(node))
    flow = FLOWS.get(key)
    if flow is None:
        unknown.append(key)
        return set()
    for how, method, targets in (flow if isinstance(flow, list) else [flow]):
        if how == "direct" or (isinstance(node, ast.Name) and fn is not None and assigned_from(fn, node.id, method)):
            pending.append(targets)
            return set()
    unknown.append(key)  # the name is not made where any rule says
    return set()


def returns(rel, names, unknown, pending):
    found = set()
    for fn in ast.walk(parse(rel)):
        if isinstance(fn, ast.FunctionDef) and fn.name in names:
            for ret in ast.walk(fn):
                if isinstance(ret, ast.Return) and ret.value is not None:
                    found |= strings(ret.value, rel, fn, unknown, pending)
    return found


def replies(rel, tree, unknown, pending):
    """Every reason given to ``_reply``: the live arbiter's third positional argument, ``reason=`` anywhere."""
    found, owner = set(), enclosing(tree)
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and getattr(node.func, "attr", None) == "_reply"):
            continue
        if any(isinstance(a, ast.Starred) for a in node.args) or any(k.arg is None for k in node.keywords):
            unknown.append((rel, ast.unparse(node)))  # unpacked arguments are not read (Codex D3 impl round 3 P2)
            continue
        arg = next((k.value for k in node.keywords if k.arg == "reason"), None)
        index = REASON_ARG[rel]  # the live arbiter: _reply(kind, request_id, reason); the fake: _reply(msg, reason)
        if arg is None and len(node.args) > index:
            arg = node.args[index]
        if arg is not None:
            found |= strings(arg, rel, owner.get(id(node)), unknown, pending)
    return found


def vehicle_reasons():
    unknown, pending, found, seen = [], [], set(), set()
    for rel in (ARBITER, MOCK):
        found |= replies(rel, parse(rel), unknown, pending)
    while pending:
        targets = pending.pop()
        if targets == "ledger":
            found |= {"stale", "busy"}  # RequestLedger.admit's refusals
            continue
        for rel, names in targets:
            if (rel, names) not in seen:
                seen.add((rel, names))
                found |= returns(rel, names, unknown, pending)
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


def test_every_reason_a_vehicle_can_refuse_with_has_a_text():
    found, unknown = vehicle_reasons()
    assert not unknown, f"run-time refusal reasons with no rule in FLOWS: {unknown}"
    missing = sorted(r for r in found if not explained(r, table()))
    assert not missing, missing


def test_the_scan_reaches_through_the_flows_and_the_fake_vehicle():
    """Codex D3 round 1 P2-3 (not_running, cargo_held) and impl round 1 P3 (the fake vehicle's reasons)."""
    found, _ = vehicle_reasons()
    assert {"not_running", "cargo_held", "already_there", "destination_occupied", "localization_lost", "stale",
            "busy", "no_feedback:rear_axle_speed", "stale_targets", "invalid", "fork_in_pocket", "load_present",
            "resume_failed"} <= found


def test_a_keyword_reason_and_a_name_made_elsewhere_are_caught():
    """Codex D3 impl round 1 P2: reason="novel" passed; so did problem = "novel" returned through _reply."""
    tree = ast.parse("class A:\n"
                     "    def f(self, kind, rid):\n"
                     "        return self._reply(kind, rid, reason='novel_reply')\n"
                     "    def g(self, kind, rid):\n"
                     "        problem = 'novel_flow'\n"
                     "        return self._reply(kind, rid, problem)\n")
    unknown, pending = [], []
    found = replies(ARBITER, tree, unknown, pending)
    assert "novel_reply" in found and not explained("novel_reply", table())
    assert unknown == [(ARBITER, "problem")] and not pending


def test_a_positional_fake_reason_and_an_annotated_rewrite_are_caught():
    """Codex D3 impl round 2 P2: _reply(msg, "novel") on the fake vehicle and problem: str = "novel" after the call."""
    fake = ast.parse("class V:\n    def f(self, msg):\n        return self._reply(msg, 'novel_mock_reply')\n")
    unknown, pending = [], []
    assert "novel_mock_reply" in replies(MOCK, fake, unknown, pending)
    live = ast.parse("class A:\n"
                     "    def g(self, kind, rid, msg):\n"
                     "        problem = self._generation_problem(msg)\n"
                     "        problem: str = 'novel_flow'\n"
                     "        return self._reply(kind, rid, problem)\n")
    unknown, pending = [], []
    replies(ARBITER, live, unknown, pending)
    assert unknown == [(ARBITER, "problem")] and not pending


def test_unpacked_arguments_and_a_pattern_capture_are_caught():
    """Codex D3 impl round 3 P2: **{"reason": ...} and *(...) passed; so did a match capture of the name."""
    tree = ast.parse("class A:\n"
                     "    def f(self, kind, rid):\n"
                     "        return self._reply(kind, rid, **{'reason': 'novel_expanded'})\n"
                     "    def g(self, kind, rid):\n"
                     "        return self._reply(*(kind, rid, 'novel_starred'))\n"
                     "    def h(self, kind, rid, msg):\n"
                     "        problem = self._generation_problem(msg)\n"
                     "        match 'novel_match':\n"
                     "            case problem:\n"
                     "                pass\n"
                     "        return self._reply(kind, rid, problem)\n")
    unknown, pending = [], []
    replies(ARBITER, tree, unknown, pending)
    assert len(unknown) == 3 and (ARBITER, "problem") in unknown and not pending
