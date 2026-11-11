"""Sim-to-real 4e-1a (docs/design/2026-11-11-p04e1-control-tick.md 2): the control tick's names the closures defined
outside drive()'s loop read go through ``tick_vars`` -- every assignment syncs it at once, ``set_rear`` too, and no
such closure reads the bare name (so 4e-1b can move the tick out of drive() without leaving them stale)."""

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SRC = (ROOT / "sim" / "isaac" / "run_transport.py").read_text(encoding="utf-8")
TREE = ast.parse(SRC)
DRIVE = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == "drive")
NAMES = {"rear", "forward", "truth_speed", "steering_command", "requested_speed"}
NESTED = (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda, ast.ClassDef)


def _bodies(node):
    """Every statement list under ``node`` outside nested functions, with its statements in order."""
    for field in ("body", "orelse", "finalbody", "handlers"):
        block = getattr(node, field, None)
        if isinstance(block, list) and block and isinstance(block[0], ast.stmt):
            yield block
            for st in block:
                if not isinstance(st, NESTED):
                    yield from _bodies(st)
        elif isinstance(block, list):
            for h in block:
                if isinstance(h, ast.ExceptHandler):
                    yield h.body
                    for st in h.body:
                        if not isinstance(st, NESTED):
                            yield from _bodies(st)
    for case in getattr(node, "cases", []) or []:
        yield case.body


def _stored(st):
    if not isinstance(st, (ast.Assign, ast.AugAssign, ast.AnnAssign)):
        return set()
    targets = st.targets if isinstance(st, ast.Assign) else [st.target]
    return {x.id for t in targets for x in ast.walk(t)
            if isinstance(x, ast.Name) and isinstance(x.ctx, ast.Store) and x.id in NAMES}


def _synced(st):
    if not (isinstance(st, ast.Expr) and isinstance(st.value, ast.Call)):
        return set()
    f = st.value.func
    if not (isinstance(f, ast.Attribute) and f.attr == "sync" and isinstance(f.value, ast.Name)
            and f.value.id == "tick_vars"):
        return set()
    return {k.arg for k in st.value.keywords
            if isinstance(k.value, ast.Name) and k.value.id == k.arg}


def test_every_assignment_of_a_tick_name_is_synced_at_once():
    seen = 0
    for block in _bodies(DRIVE):
        for i, st in enumerate(block):
            names = _stored(st)
            if not names:
                continue
            seen += 1
            nxt = block[i + 1] if i + 1 < len(block) else None
            assert nxt is not None and names <= _synced(nxt), (st.lineno, sorted(names))
    assert seen >= 30  # the loop's and the setup's (steering_command's first value included)


def test_set_rear_syncs_the_box():
    set_rear = next(n for n in ast.walk(DRIVE) if isinstance(n, ast.FunctionDef) and n.name == "set_rear")
    body = set_rear.body
    i = next(k for k, st in enumerate(body) if _stored(st) == {"rear"})
    assert "rear" in _synced(body[i + 1])


def test_no_closure_outside_the_loop_reads_a_bare_tick_name():
    loop = next(n for n in ast.walk(DRIVE) if isinstance(n, ast.For) and isinstance(n.target, ast.Name)
                and n.target.id == "step")
    bad = []
    for n in ast.walk(DRIVE):
        if not isinstance(n, (ast.FunctionDef, ast.Lambda)) or n is DRIVE:
            continue
        if loop.lineno <= n.lineno <= loop.end_lineno:
            continue  # the loop's own closures move with it (4e-1b)
        in_sync = {id(x) for st in ast.walk(n) if isinstance(st, ast.Expr) and _synced(st) for x in ast.walk(st)}
        bad += [(x.lineno, x.id) for x in ast.walk(n)
                if isinstance(x, ast.Name) and isinstance(x.ctx, ast.Load) and x.id in NAMES and id(x) not in in_sync]
    assert not bad, bad


def test_the_box_raises_name_error_until_set():
    # TickVars is plain Python; load the class body alone (the runner imports Isaac lazily, not at module level)
    cls = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == "TickVars")
    ns: dict = {}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), "run_transport.py", "exec"), ns)
    box = ns["TickVars"]()
    with pytest.raises(NameError):
        box.rear  # noqa: B018 (grid_kwargs's first plans rely on it)
    arr = [0.0, 0.0]
    box.sync(steering_command=arr, rear=(1.0, 2.0, 0.0))
    assert box.steering_command is arr and box.rear == (1.0, 2.0, 0.0)
