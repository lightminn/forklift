"""M-a2: sweep the core-side text the kept (R) source tests read for code no behaviour test guards.

For every frozen test that some R property keeps, the string constants its ``assert`` statements (and the ``for``
loops feeding them) look for in ``src/forklift_core/mission/core.py`` -- a constant of at least 12 characters whose
first line occurs at most ten times there (each occurrence) -- name a place in the core. Each place gets the mutations that break what a
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

MAX_OCCURRENCES = 10  # a constant found more often names no place in particular
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


def all_tests() -> dict:
    """Every frozen test (kept or not), read at the frozen list's base revision."""
    frozen = json.loads(FROZEN.read_text())
    return {(r["file"], r["test"]): {"frozen"} for r in frozen["assertions"]}


def test_source(file_: str, base: str | None) -> str:
    if base is None:
        return (ROOT / file_).read_text()
    return subprocess.run(["git", "-C", str(ROOT), "show", f"{base}:{file_}"], capture_output=True, text=True,
                          check=True).stdout


def asserted_constants(source: str, test: str) -> set[str]:
    tree = ast.parse(source)
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

    def __init__(self, text: str, operators: bool = False):
        self.operators = operators  # also boundaries, narrowing and order (Codex M-a2 review round 3)
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
        if self.operators:
            out += self.boundary_mutations(offset) + self.order_mutations(offset)
        return out

    BOUNDARY = {ast.LtE: "<", ast.Lt: "<=", ast.GtE: ">", ast.Gt: ">=", ast.Eq: "!=", ast.NotEq: "=="}

    def boundary_mutations(self, offset: int) -> list[tuple[str, str, str]]:
        """The innermost comparison holding ``offset``: each operator moved across its boundary, and a membership
        in a literal tuple narrowed to each of its members (a condition strengthened)."""
        best = None
        for node in ast.walk(self.tree):
            if isinstance(node, ast.Compare):
                a, b = self.span(node)
                if a <= offset < b and (best is None or b - a < best[0][1] - best[0][0]):
                    best = ((a, b), node)
        if best is None or len(best[1].ops) != 1:
            return []
        span_, cmp_ = best
        op, right = cmp_.ops[0], cmp_.comparators[0]
        left = ast.unparse(cmp_.left)
        out = []
        if type(op) in self.BOUNDARY:
            new = f"{left} {self.BOUNDARY[type(op)]} {ast.unparse(right)}"
            out.append(("boundary", ast.unparse(cmp_)[:120], self.replace(span_, new)))
        if isinstance(op, ast.In) and isinstance(right, (ast.Tuple, ast.List, ast.Set)) and len(right.elts) > 1:
            for elt in right.elts:
                out.append(("narrow", f"{ast.unparse(cmp_)[:100]} -> {ast.unparse(elt)}",
                            self.replace(span_, f"{left} == {ast.unparse(elt)}")))
        return out

    def order_mutations(self, offset: int) -> list[tuple[str, str, str]]:
        """The innermost simple statement holding ``offset`` swapped with each simple neighbour in its block."""
        out = []
        for node in ast.walk(self.tree):
            for name in ("body", "orelse", "finalbody"):
                block = getattr(node, name, None)
                if not isinstance(block, list):
                    continue
                for i, stmt in enumerate(block):
                    if isinstance(stmt, COMPOUND) or not isinstance(stmt, ast.stmt):
                        continue
                    a, b = self.span(stmt)
                    if not a <= offset < b:
                        continue
                    for j in (i - 1, i + 1):
                        if 0 <= j < len(block) and not isinstance(block[j], COMPOUND) \
                                and not independent(stmt, block[j]):
                            (a1, b1), (a2, b2) = sorted([self.span(stmt), self.span(block[j])])
                            d = self.data
                            text = (d[:a1] + d[a2:b2] + d[b1:a2] + d[a1:b1] + d[b2:]).decode()
                            try:
                                ast.parse(text)
                            except SyntaxError:
                                continue
                            out.append(("swap", f"{ast.unparse(stmt).split(chr(10))[0][:60]} <-> "
                                                f"{ast.unparse(block[j]).split(chr(10))[0][:50]}", text))
        return out


def _written(stmt) -> set:
    """Names and subscript/attribute roots a statement assigns."""
    out = set()
    targets = stmt.targets if isinstance(stmt, ast.Assign) else [getattr(stmt, "target", None)]
    for t in targets:
        if t is None:
            continue
        node = t
        while isinstance(node, (ast.Subscript, ast.Attribute)):
            node = node.value
        out |= {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}
    return out


def _read(stmt) -> set:
    return {n.id for n in ast.walk(stmt) if isinstance(n, ast.Name)}


def independent(s1, s2) -> bool:
    """Two statements whose order cannot matter: no call in either (no port, no side effect, no exception
    between them that a partial state could show) and neither assigns a name the other reads."""
    if any(isinstance(n, ast.Call) for st in (s1, s2) for n in ast.walk(st)):
        return False
    return not (_written(s1) & _read(s2) or _written(s2) & _read(s1))


def places(src: Source, kept: dict, base: str | None = None) -> dict:
    found: dict[int, dict] = {}
    sources: dict = {}
    for (file_, test), _props in sorted(kept.items()):
        if file_ not in sources:
            sources[file_] = test_source(file_, base)
        for const in sorted(asserted_constants(sources[file_], test)):  # sorted: the same names every run
            probe = const.split("\n")[0].strip()
            if len(probe) < 12 or not 1 <= src.text.count(probe) <= MAX_OCCURRENCES:
                continue
            at = -1
            while (at := src.text.find(probe, at + 1)) >= 0:  # every occurrence: the test may mean any of them
                offset = len(src.text[:at].encode())
                entry = found.setdefault(offset, {"probe": probe[:120], "line": src.text.count("\n", 0, at) + 1,
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
    parser.add_argument("--operators", action="store_true",
                        help="add comparison boundaries, membership narrowing and adjacent-statement swaps")
    parser.add_argument("--all", action="store_true",
                        help="every frozen test at the frozen base revision, not only the kept (R) ones")
    parser.add_argument("--emit", type=Path, help="a sweep's JSON: write the find/replace and failing tests of its "
                                                  "survivors to --out instead of sweeping")
    args = parser.parse_args(argv)
    if args.emit:
        return emit_batch(args.emit, args.out, args.tests, args.operators)
    core_text = (ROOT / CORE).read_text()
    src = Source(core_text, operators=args.operators)
    if args.all:
        found = places(src, all_tests(), json.loads(FROZEN.read_text())["base_revision"])
    else:
        found = places(src, kept_tests())
    results = []
    with tempfile.TemporaryDirectory(prefix="ma2_sweep_") as tmp:
        tree = Path(tmp)
        copy_tree(tree)
        target = tree / CORE
        assert run(tree, args.tests) == "survived", "the behaviour tests must pass unmutated"
        seen: dict = {}  # a mutated text already run (two places can make the same one)
        for offset, entry in sorted(found.items()):
            entry["mutations"] = []
            for kind, what, text in src.mutations(offset):
                if text in seen:
                    outcome = seen[text]
                else:
                    target.write_text(text)
                    try:
                        outcome = seen[text] = run(tree, args.tests)
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



def unique_block(text: str, mutated: str) -> tuple[str, str]:
    """The smallest run of whole lines around the change that occurs once in ``text`` -- a find/replace pair."""
    a, b = text.split("\n"), mutated.split("\n")
    first = next(i for i in range(min(len(a), len(b))) if a[i] != b[i])
    tail = 0
    while tail < min(len(a), len(b)) - first and a[len(a) - 1 - tail] == b[len(b) - 1 - tail]:
        tail += 1
    lo, hi_a, hi_b = first, len(a) - tail, len(b) - tail
    while True:
        find = "\n".join(a[lo:hi_a]) + "\n"
        if text.count(find) == 1:
            return find, "\n".join(b[lo:hi_b]) + "\n"
        lo, hi_a, hi_b = max(lo - 1, 0), hi_a + 1, hi_b + 1


def failing_tests(tree: Path, tests: str) -> list[str]:
    env = dict(os.environ, PYTHONPATH=str(tree / "src"), PYTHONDONTWRITEBYTECODE="1")
    proc = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", "-rf", tests], cwd=tree,
                          env=env, capture_output=True, text=True, timeout=900)
    return sorted({line.split()[1] for line in proc.stdout.splitlines() if line.startswith("FAILED ")})


def emit_batch(sweep_json: Path, out_json: Path, tests: str, operators: bool = False) -> int:
    """For every mutation a sweep left alive: the find/replace that makes it, and the behaviour tests that now fail
    on it (written as JSON for tools/ma2/batch_09_sweep.py)."""
    core_text = (ROOT / CORE).read_text()
    src = Source(core_text, operators=operators)
    found = places(src, all_tests(), json.loads(FROZEN.read_text())["base_revision"])
    by_line = {}
    for offset, entry in found.items():
        by_line.setdefault((entry["line"], entry["probe"]), offset)
    frozen = json.loads(FROZEN.read_text())
    ids_by_test = {}
    for r in frozen["assertions"]:
        ids_by_test.setdefault(f"{r['file']}::{r['test']}", r["id"].rsplit("::", 1)[0] + "::*")
    out = []
    with tempfile.TemporaryDirectory(prefix="ma2_emit_") as tmp:
        tree = Path(tmp)
        copy_tree(tree)
        target = tree / CORE
        for entry in json.loads(sweep_json.read_text()):
            alive = [m for m in entry["mutations"] if m["outcome"] == "survived"]
            if not alive:
                continue
            offset = by_line.get((entry["line"], entry["probe"]))
            if offset is None:  # a sweep from before the names were sorted: the place on that line
                offset = next(o for (line_, _), o in sorted(by_line.items()) if line_ == entry["line"])
            for kind, what, text in src.mutations(offset):
                if not any(m["kind"] == kind and m["what"] == what for m in alive):
                    continue
                find, replace_ = unique_block(core_text, text)
                target.write_text(text)
                try:
                    guards = failing_tests(tree, tests)
                finally:
                    target.write_text(core_text)
                out.append({"line": entry["line"], "kind": kind, "what": what, "find": find, "replace": replace_,
                            "guards": guards, "checks": sorted({ids_by_test[t] for t in entry["tests"]})})
                print(f"{entry['line']:5d} {kind:12s} {len(guards)} guards  {what[:80]}", flush=True)
    out_json.write_text(json.dumps(out, indent=1, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
