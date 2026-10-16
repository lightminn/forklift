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


def _source_handles(tree: ast.Module) -> set[str]:
    """Module-level names bound to (something derived from) a runner/core source path, and module functions that
    use one, to a fixpoint."""
    handles: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.Assign):
            text = ast.unparse(node.value)
            if any(s in text for s in SOURCES):
                handles.update(t.id for t in node.targets if isinstance(t, ast.Name))
    changed = True
    while changed:
        changed = False
        for node in tree.body:
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


def inventory(paths: list[Path]) -> list[dict]:
    rows = []
    for path in paths:
        tree = ast.parse(path.read_text())
        handles = _source_handles(tree)
        rel = str(path.relative_to(ROOT))
        for fn in tree.body:
            if not (isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)) and fn.name.startswith("test")):
                continue
            body_names = {n.id for n in ast.walk(fn) if isinstance(n, ast.Name)}
            text = ast.unparse(fn)
            direct = any(s in text for s in SOURCES)
            if not (body_names & handles or direct):
                continue
            n = 0
            for node in ast.walk(fn):
                if isinstance(node, ast.Assert):
                    n += 1
                    rows.append({"id": f"{rel}::{fn.name}::{n}", "file": rel, "test": fn.name, "line": node.lineno,
                                 "assert": ast.unparse(node)[:400],
                                 "via": sorted(body_names & handles) or ["direct"]})
    return rows


def candidate_files() -> list[Path]:
    out = []
    for path in sorted((ROOT / "tests").rglob("test_*.py")):
        text = path.read_text()
        if any(s in text for s in SOURCES):
            out.append(path)
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    rows = inventory(candidate_files())
    rev = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    out = {"base_revision": rev, "assertions": rows,
           "digest": hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()}
    text = json.dumps(out, indent=1, ensure_ascii=False)
    if args.json:
        args.json.write_text(text + "\n")
    tests = {(r["file"], r["test"]) for r in rows}
    print(f"{len(rows)} assertions in {len(tests)} tests ({len({r['file'] for r in rows})} files), base {rev}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
