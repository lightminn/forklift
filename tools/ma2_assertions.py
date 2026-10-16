"""M-a2: freeze the assertions of every test that reads the runner's or the mission core's source
(docs/plans/2026-10-16-m-a2-source-tests-to-behaviour.md, step 1).

A test "reads source" when its body -- or a module-level helper it calls, transitively -- names a source handle:
a module constant built from ``sim/isaac/run_transport.py`` or ``src/forklift_core/mission/core.py`` (``SCRIPT``,
``CORE``, a parsed ``TREE``...), or calls ``read_text``/``ast.parse``/``inspect.getsource`` on one. Every ``assert``
in such a test is listed with its file, test, line and text, under a stable id (``file::test::n``); the property map
then has to account for each id.

    python tools/ma2_assertions.py [--json OUT]
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ("sim/isaac/run_transport.py", "src/forklift_core/mission/core.py", "run_transport.py", "mission/core.py")


SOURCE_CALLS = ("getsource", "getsourcelines")  # inspect on a runner/core object reads its source too


def _reads_source_text(node: ast.AST) -> bool:
    text = ast.unparse(node)
    return any(s in text for s in SOURCES) or any(f"inspect.{c}" in text or f"{c}(" in text for c in SOURCE_CALLS)


def _functions(tree: ast.Module):
    """Module functions and class methods (a class-based test's methods count as tests)."""
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            yield node
        elif isinstance(node, ast.ClassDef):
            for item in node.body:
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    yield item


def _source_handles(tree: ast.Module, imported: set[str] = frozenset()) -> set[str]:
    """Names that lead to runner/core source: module constants built from a source path, helpers and fixtures whose
    own body reads one (Codex P2: a helper reading the source in its body is a starting point too), anything using
    those, to a fixpoint; ``imported`` adds handles of other test modules imported by name."""
    handles: set[str] = set(imported)
    for node in tree.body:
        if isinstance(node, ast.Assign) and _reads_source_text(node.value):
            handles.update(t.id for t in node.targets if isinstance(t, ast.Name))
    for fn in _functions(tree):
        if not fn.name.startswith("test") and _reads_source_text(fn):
            handles.add(fn.name)
    changed = True
    while changed:
        changed = False
        for node in [*tree.body, *_functions(tree)]:
            names = {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}
            if isinstance(node, ast.Assign) and names & handles:
                new = {t.id for t in node.targets if isinstance(t, ast.Name)} - handles
                if new:
                    handles |= new
                    changed = True
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and not node.name.startswith("test") \
                    and node.name not in handles and names & handles:
                handles.add(node.name)
                changed = True
    return handles


def _conditions(fn, handles: set[str], helpers: dict) -> list[dict]:
    """The checks a test makes: every ``assert``, every ``pytest.raises``/``pytest.warns`` (its type, match and the
    block it covers), every call into a helper that holds checks of its own, and the test's normal completion (a test
    whose only requirement is to run through -- Codex P2)."""
    out = []
    for node in ast.walk(fn):
        if isinstance(node, ast.Assert):
            out.append(("assert", node.lineno, ast.unparse(node)))
        elif isinstance(node, (ast.With, ast.AsyncWith)):
            for item in node.items:
                text = ast.unparse(item.context_expr)
                if text.startswith(("pytest.raises", "pytest.warns", "raises(", "warns(")):
                    body = "\n".join(ast.unparse(b) for b in node.body)
                    out.append(("raises", node.lineno, f"with {text}:\n{body}"))
        elif isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in helpers \
                and helpers[node.func.id]:
            out.append(("helper", node.lineno, f"{ast.unparse(node)}  # checks inside {node.func.id}"))
    out.append(("completes", fn.lineno, f"{fn.name} runs to its end without an unexpected exception"))
    return out


def _is_fixture(fn) -> tuple[bool, bool]:
    """(a pytest fixture, autouse)."""
    for dec in fn.decorator_list:
        text = ast.unparse(dec)
        if "fixture" in text:
            return True, "autouse=True" in text
    return False, False


def _fixture_checks(tree: ast.Module) -> dict:
    """Per fixture: (autouse, the checks inside it -- its own asserts/raises and those of the test doubles it defines
    and installs; Codex P2: a selected test's autouse fixture checks the runner's calls too)."""
    out = {}
    for fn in _functions(tree):
        fixture, autouse = _is_fixture(fn)
        if not fixture:
            continue
        checks = [(f"fixture:{fn.name}", node.lineno, ast.unparse(node))
                  for node in ast.walk(fn) if isinstance(node, ast.Assert)]
        out[fn.name] = (autouse, checks)
    return out


def _conftest_fixtures(path: Path) -> dict:
    """Fixtures of the conftest files above ``path``."""
    out = {}
    for parent in reversed(path.parents):
        conf = parent / "conftest.py"
        if conf.is_file() and ROOT in conf.parents:
            out.update(_fixture_checks(ast.parse(conf.read_text())))
    return out


def _test_names(tree: ast.Module):
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name.startswith("test"):
            yield node, node.name
        elif isinstance(node, ast.ClassDef) and node.name.startswith("Test"):
            for item in node.body:
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)) and item.name.startswith("test"):
                    yield item, f"{node.name}::{item.name}"


def inventory(paths: list[Path], cross: dict | None = None) -> list[dict]:
    rows = []
    cross = cross or {}
    for path in paths:
        tree = ast.parse(path.read_text())
        imported = set()
        for node in tree.body:  # helpers imported from other test modules that read source
            if isinstance(node, ast.ImportFrom) and node.module:
                imported |= {a.asname or a.name for a in node.names} & cross.get(node.module.split(".")[-1], set())
        handles = _source_handles(tree, imported)
        fixtures = {**_conftest_fixtures(path), **_fixture_checks(tree)}
        helpers = {fn.name: any(isinstance(n, (ast.Assert, ast.With)) for n in ast.walk(fn))
                   for fn in _functions(tree) if not fn.name.startswith("test")}
        rel = str(path.relative_to(ROOT))
        for fn, name in _test_names(tree):
            params = {a.arg for a in fn.args.args}
            body_names = {n.id for n in ast.walk(fn) if isinstance(n, ast.Name)} | params  # fixtures by parameter
            if not (body_names & handles or _reads_source_text(fn)):
                continue
            conditions = _conditions(fn, handles, helpers)
            for fx, (autouse, checks) in sorted(fixtures.items()):  # autouse and requested fixtures' own checks
                if autouse or fx in params:
                    conditions += checks
            for n, (kind, line, text) in enumerate(conditions, start=1):
                rows.append({"id": f"{rel}::{name}::{n}", "file": rel, "test": name, "kind": kind, "line": line,
                             "text_sha256": hashlib.sha256(text.encode()).hexdigest(), "preview": text[:200],
                             "via": sorted(body_names & handles) or ["direct"]})
    return rows


def candidate_files() -> list[Path]:
    """Test modules and conftest files that name a runner/core source, use inspect on one, or import a helper that
    does from another test module."""
    files = sorted({*(ROOT / "tests").rglob("test_*.py"), *(ROOT / "tests").rglob("conftest.py")})
    reading = {p for p in files if any(s in p.read_text() for s in SOURCES) or "getsource" in p.read_text()}
    return sorted(reading | {p for p in files if any(f"from {q.stem} import" in p.read_text() or f".{q.stem} import"
                                                      in p.read_text() for q in reading)})


def cross_handles(paths: list[Path]) -> dict:
    return {p.stem: _source_handles(ast.parse(p.read_text())) for p in paths}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    files = candidate_files()
    rows = inventory(files, cross_handles(files))
    rev = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    out = {"base_revision": rev, "assertions": rows,
           "digest": hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()}
    text = json.dumps(out, indent=1, ensure_ascii=False)
    if args.json:
        args.json.write_text(text + "\n")
    tests = {(r["file"], r["test"]) for r in rows}
    import collections

    kinds = dict(collections.Counter(r["kind"] for r in rows))
    print(f"{len(rows)} conditions {kinds} in {len(tests)} tests ({len({r['file'] for r in rows})} files), base {rev}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
