"""M-a2: the mutation evidence of the property map (docs/plans/2026-10-16-m-a2-source-tests-to-behaviour.md, 2.3).

For each D/N property (or the ones named): copy the tracked tree to a scratch directory once, run the guarding tests
there unmutated (they must pass), apply the property's mutation -- an exact text replacement in one file, the text
required to occur exactly ``count`` times (default 1) -- run the same tests (they must fail, and on a check: an
AssertionError, a pytest.raises/fail, never a collection, import or syntax error), restore the file, and write the
outcome into the map's ``evidence``.

    python tools/ma2_mutate.py [--map MAP] [--only P001 P002 ...] [--write]
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / "tests/ma2_property_map.json"
CHECK_FAILURE = re.compile(r"^E\s+(AssertionError|assert |Failed: DID NOT RAISE|Failed: DID NOT WARN|"
                           r"pytest\.fail|Failed:)", re.M)
NOT_A_CHECK = re.compile(r"(ImportError|ModuleNotFoundError|SyntaxError|IndentationError|NameError|"
                         r"errors? during collection|ERROR collecting)")


def copy_tree(dest: Path) -> None:
    files = subprocess.run(["git", "-C", str(ROOT), "ls-files", "-z"], capture_output=True, check=True).stdout
    for rel in files.decode().split("\0"):
        if not rel:
            continue
        src, dst = ROOT / rel, dest / rel
        if src.is_file():
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)


def run_tests(tree: Path, nodeids: list[str]) -> tuple[int, str]:
    env = dict(os.environ, PYTHONPATH=str(tree / "src"), PYTHONDONTWRITEBYTECODE="1")
    proc = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", "-x", "--tb=short",
                           *nodeids], cwd=tree, env=env, capture_output=True, text=True, timeout=900)
    return proc.returncode, proc.stdout[-6000:] + proc.stderr[-2000:]


def evidence_for(tree: Path, prop: dict) -> dict:
    mutation, nodeids = prop["mutation"], prop["tests"]
    target = tree / mutation["file"]
    original = target.read_text()
    count = mutation.get("count", 1)
    found = original.count(mutation["find"])
    if found != count:
        return {"baseline": None, "mutated": None,
                "error": f"mutation text occurs {found} times in {mutation['file']}, expected {count}"}
    code, out = run_tests(tree, nodeids)
    baseline = "passed" if code == 0 else "failed"
    if baseline != "passed":
        return {"baseline": baseline, "mutated": None, "baseline_output": out[-1500:]}
    try:
        target.write_text(original.replace(mutation["find"], mutation["replace"]))
        code, out = run_tests(tree, nodeids)
    finally:
        target.write_text(original)
    if code == 0:
        mutated = "passed"
    elif NOT_A_CHECK.search(out) and not CHECK_FAILURE.search(out):
        mutated = "failed_not_on_check"
    elif CHECK_FAILURE.search(out):
        mutated = "failed_on_check"
    else:
        mutated = "failed_other"
    failing = re.findall(r"^FAILED (\S+)", out, re.M)
    return {"baseline": "passed", "mutated": mutated, "failing_tests": failing[:5],
            "check_line": (CHECK_FAILURE.search(out).group(0)[:200] if CHECK_FAILURE.search(out) else None)}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--map", type=Path, default=MAP)
    parser.add_argument("--only", nargs="*")
    parser.add_argument("--write", action="store_true", help="store the evidence in the map")
    args = parser.parse_args(argv)
    prop_map = json.loads(args.map.read_text())
    props = [p for p in prop_map["properties"] if p.get("class") in ("D", "N") and p.get("mutation")
             and (not args.only or p["id"] in args.only)]
    bad = 0
    with tempfile.TemporaryDirectory(prefix="ma2_mutate_") as tmp:
        tree = Path(tmp)
        copy_tree(tree)
        for prop in props:
            ev = evidence_for(tree, prop)
            prop["evidence"] = ev
            ok = ev.get("baseline") == "passed" and ev.get("mutated") == "failed_on_check"
            bad += not ok
            print(f"{prop['id']}: {'ok' if ok else 'NOT OK'} {json.dumps({k: v for k, v in ev.items() if k != 'baseline_output'})[:300]}")
    if args.write:
        args.map.write_text(json.dumps(prop_map, indent=1, ensure_ascii=False) + "\n")
    print(f"{len(props)} properties mutated, {bad} without valid evidence")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
