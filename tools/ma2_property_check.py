"""M-a2: check the property map against the frozen checks (docs/plans/2026-10-16-m-a2-source-tests-to-behaviour.md,
section 2.4).

The map (``tests/ma2_property_map.json``) lists properties: an id, a statement, a class (D existing behaviour test,
N new behaviour test, R runner wiring/order kept, S structural check kept, X no longer a property), the frozen check
ids it accounts for, the tests that now guard it (pytest node ids), a reason (R/S/X) and, for D/N, the mutation that
breaks it and the evidence that the guarding test passed without it and failed on a check with it.

    python tools/ma2_property_check.py [--map MAP] [--frozen FROZEN]   # prints the problems, exit 1 if any
"""

from __future__ import annotations

import argparse
import ast
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / "tests/ma2_property_map.json"
FROZEN = ROOT / "docs/validation/2026-10-16-m-a2-assertions.json"
CLASSES = {"D", "N", "R", "S", "X"}


def _test_exists(nodeid: str) -> bool:
    path, _, rest = nodeid.partition("::")
    file_ = ROOT / path
    if not file_.is_file():
        return False
    name = rest.split("[")[0].split("::")[-1]
    tree = ast.parse(file_.read_text())
    return any(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == name for n in ast.walk(tree))


def check(prop_map: dict, frozen: dict, *, complete: bool | None = None) -> list[str]:
    """The problems: unknown or doubly-claimed check ids, properties without class/tests/reason/evidence, and, when the
    map says it is complete (or ``complete`` is given), frozen checks no property accounts for."""
    problems = []
    frozen_ids = {row["id"] for row in frozen["assertions"]}
    if prop_map.get("assertions_digest") != frozen.get("digest"):
        problems.append("the map was built against another frozen list (digest differs)")
    claimed: dict[str, str] = {}
    ids = set()
    for prop in prop_map.get("properties", []):
        pid = prop.get("id")
        if not pid or pid in ids:
            problems.append(f"property id missing or repeated: {pid!r}")
            continue
        ids.add(pid)
        cls = prop.get("class")
        if cls not in CLASSES:
            problems.append(f"{pid}: class {cls!r} is not one of {sorted(CLASSES)}")
        if not prop.get("statement"):
            problems.append(f"{pid}: no statement")
        checks = prop.get("checks") or []
        if not checks:
            problems.append(f"{pid}: accounts for no frozen check")
        for cid in checks:
            if cid not in frozen_ids:
                problems.append(f"{pid}: unknown check {cid}")
            claimed.setdefault(cid, pid)
        if cls in ("R", "S", "X") and not prop.get("reason"):
            problems.append(f"{pid}: class {cls} needs a reason")
        if cls in ("D", "N", "R", "S"):
            tests = prop.get("tests") or []
            if not tests:
                problems.append(f"{pid}: no guarding test")
            for nodeid in tests:
                if not _test_exists(nodeid):
                    problems.append(f"{pid}: guarding test {nodeid} does not exist")
        if cls in ("D", "N"):
            mutation, evidence = prop.get("mutation") or {}, prop.get("evidence") or {}
            if not (mutation.get("file") and mutation.get("find") is not None and "replace" in mutation):
                problems.append(f"{pid}: no mutation (file, find, replace)")
            if evidence.get("baseline") != "passed" or evidence.get("mutated") != "failed_on_check":
                problems.append(f"{pid}: no evidence (baseline passed, mutated failed on a check): {evidence}")
    is_complete = prop_map.get("complete") if complete is None else complete
    if is_complete:
        missing = sorted(frozen_ids - set(claimed))
        if missing:
            problems.append(f"{len(missing)} frozen checks no property accounts for, e.g. {missing[:3]}")
    return problems


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--map", type=Path, default=MAP)
    parser.add_argument("--frozen", type=Path, default=FROZEN)
    parser.add_argument("--complete", action="store_true", help="require every frozen check to be accounted for")
    args = parser.parse_args(argv)
    problems = check(json.loads(args.map.read_text()), json.loads(args.frozen.read_text()),
                     complete=True if args.complete else None)
    for p in problems:
        print(p)
    frozen = json.loads(args.frozen.read_text())
    prop_map = json.loads(args.map.read_text())
    covered = {c for p in prop_map.get("properties", []) for c in p.get("checks", [])}
    print(f"{len(prop_map.get('properties', []))} properties; {len(covered)}/{len(frozen['assertions'])} frozen checks "
          f"accounted for; {len(problems)} problems")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
