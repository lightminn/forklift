"""M-a2: sweep the core-side text the kept (R) source tests read for code no behaviour test guards.

For every frozen test that some R property keeps, the string constants its ``assert`` statements (and the ``for``
loops feeding them) look for in ``src/forklift_core/mission/core.py`` -- a constant of at least 12 characters whose
first line occurs exactly once there -- name a place in the core. Each place gets the mutations that break what a
source test reading it could have meant:

- ``drop_operand``: the innermost ``and``/``or`` operand holding it becomes ``True``/``False`` (a condition removed);
- ``test_true`` / ``test_false``: an ``if``/``elif``/``while`` test holding it (with no operand to drop) is forced;
- ``pass``: the innermost simple statement holding it is replaced by ``pass``.

Each mutation runs the mission behaviour tests on a scratch copy: ``killed`` (a test failed), ``survived`` (all
passed -- a candidate property no behaviour test holds) or ``invalid`` (the mutated core does not import).
Survivors are candidates, judged by hand: a record-only line can rightly survive.

    python tools/ma2_core_sweep.py [--out sweep.json] [--tests tests/unit/mission]
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CORE = "src/forklift_core/mission/core.py"
MAP = ROOT / "tests/ma2_property_map.json"
FROZEN = ROOT / "docs/validation/2026-10-16-m-a2-assertions.json"
sys.path.insert(0, str(ROOT / "tools"))
from ma2_mutate import copy_tree  # noqa: E402

COMPOUND = (ast.If, ast.For, ast.AsyncFor, ast.While, ast.With, ast.AsyncWith, ast.Try, ast.FunctionDef,
            ast.AsyncFunctionDef, ast.ClassDef, ast.Match)


def kept_tests() -> dict:
    prop_map = json.loads(MAP.read_text())
    frozen = json.loads(FROZEN.read_text())
    by_id = {r["id"]: (r["file"], r["test"]) for r in frozen["assertions"]}
    kept: dict = {}
    for prop in prop_map["properties"]:
        if prop["class"] == "R":
            for cid in prop["checks"]:
                if cid in by_id:
                    kept.setdefault(by_id[cid], set()).add(prop["id"])
    return kept


def asserted_constants(path: Path, test: str) -> set[str]:
    tree = ast.parse(path.read_text())
    name = test.split("::")[-1]
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            for sub in ast.walk(node):
                if isinstance(sub, (ast.Assert, ast.For)):
                    for c in ast.walk(sub):
                        if isinstance(c, ast.Constant) and isinstance(c.value, str):
                            out.add(c.value)
    return out


class Source:
    """The core's text with byte offsets (ast columns are UTF-8 byte offsets)."""

    def __init__(self, text: str):
        self.text, self.data = text, text.encode()
        self.line_starts = [0]
        for i, b in enumerate(self.data):
            if b == 0x0A:
                self.line_starts.append(i + 1)
        self.tree = ast.parse(text)

    def span(self, node) -> tuple[int, int]:
        return (self.line_starts[node.lineno - 1] + node.col_offset,
                self.line_starts[node.end_lineno - 1] + node.end_col_offset)

    def replace(self, span, new: str) -> str:
        return (self.data[: span[0]] + new.encode() + self.data[span[1]:]).decode()

    def mutations(self, offset: int) -> list[tuple[str, str, str]]:
        """(kind, what, mutated text) for the place at byte ``offset``."""
        out = []
        operand, stmt, test = None, None, None
        for node in ast.walk(self.tree):
            if isinstance(node, ast.BoolOp):
                for v in node.values:
                    a, b = self.span(v)
                    if a <= offset < b and (operand is None or b - a < operand[1][1] - operand[1][0]):
                        operand = (node, (a, b), v)
            elif isinstance(node, (ast.If, ast.While)):
                a, b = self.span(node.test)
                if a <= offset < b and (test is None or b - a < test[0][1] - test[0][0]):
                    test = ((a, b), node.test)
            if isinstance(node, ast.stmt) and not isinstance(node, COMPOUND):
                a, b = self.span(node)
                if a <= offset < b and (stmt is None or b - a < stmt[0][1] - stmt[0][0]):
                    stmt = ((a, b), node)
        if operand is not None:
            boolop, span_, v = operand
            out.append(("drop_operand", ast.unparse(v)[:120],
                        self.replace(span_, "True" if isinstance(boolop.op, ast.And) else "False")))
        elif test is not None:
            out.append(("test_true", ast.unparse(test[1])[:120], self.replace(test[0], "True")))
            out.append(("test_false", ast.unparse(test[1])[:120], self.replace(test[0], "False")))
        if stmt is not None:
            out.append(("pass", ast.unparse(stmt[1]).split("\n")[0][:120], self.replace(stmt[0], "pass")))
        return out


def places(src: Source, kept: dict) -> dict:
    found: dict[int, dict] = {}
    for (file_, test), _props in sorted(kept.items()):
        for const in asserted_constants(ROOT / file_, test):
            probe = const.split("\n")[0].strip()
            if len(probe) < 12 or src.text.count(probe) != 1:
                continue
            offset = len(src.text[: src.text.index(probe)].encode())
            entry = found.setdefault(offset, {"probe": probe[:120], "line": src.text.count("\n", 0, src.text.index(probe)) + 1,
                                              "tests": []})
            entry["tests"].append(f"{file_}::{test}")
    return found


def run(tree: Path, tests: str) -> str:
    env = dict(os.environ, PYTHONPATH=str(tree / "src"), PYTHONDONTWRITEBYTECODE="1")
    proc = subprocess.run([sys.executable, "-m", "pytest", "-x", "-q", "-p", "no:cacheprovider", tests], cwd=tree,
                          env=env, capture_output=True, text=True, timeout=900)
    if proc.returncode == 0:
        return "survived"
    if any(s in proc.stdout for s in ("SyntaxError", "IndentationError", "ERROR collecting")):
        return "invalid"
    return "killed"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", type=Path, default=Path("ma2_core_sweep.json"))
    parser.add_argument("--tests", default="tests/unit/mission")
    args = parser.parse_args(argv)
    core_text = (ROOT / CORE).read_text()
    src = Source(core_text)
    found = places(src, kept_tests())
    results = []
    with tempfile.TemporaryDirectory(prefix="ma2_sweep_") as tmp:
        tree = Path(tmp)
        copy_tree(tree)
        target = tree / CORE
        assert run(tree, args.tests) == "survived", "the behaviour tests must pass unmutated"
        for offset, entry in sorted(found.items()):
            entry["mutations"] = []
            for kind, what, text in src.mutations(offset):
                target.write_text(text)
                try:
                    outcome = run(tree, args.tests)
                finally:
                    target.write_text(core_text)
                entry["mutations"].append({"kind": kind, "what": what, "outcome": outcome})
                print(f"{entry['line']:5d} {kind:12s} {outcome:9s} {what[:90]}", flush=True)
            results.append(entry)
    args.out.write_text(json.dumps(results, indent=1, ensure_ascii=False) + "\n")
    counts: dict = {}
    for r in results:
        for m in r["mutations"]:
            counts[m["outcome"]] = counts.get(m["outcome"], 0) + 1
    print(counts)
    return 0


if __name__ == "__main__":
    sys.exit(main())
