"""Build tests/ma2_property_map.json from the per-batch property sources in tools/ma2/batch_*.py (M-a2).

Each batch module defines ``PROPERTIES``: a list of dicts (id, statement, class, checks -- frozen check ids or
``"<file>::<test>::*"`` for all of a test's checks -- tests, reason, mutation). Evidence already in the map is kept for
an unchanged property (same mutation and tests); otherwise it is dropped and tools/ma2_mutate.py has to run again.
"""
import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "tests/ma2_property_map.json"
FROZEN = json.loads((ROOT / "docs/validation/2026-10-16-m-a2-assertions.json").read_text())


def expand(checks):
    out = []
    for c in checks:
        if c.endswith("::*"):
            prefix = c[:-1]
            hits = [r["id"] for r in FROZEN["assertions"] if r["id"].startswith(prefix)]
            if not hits:
                raise SystemExit(f"no frozen checks match {c}")
            out += hits
        else:
            out.append(c)
    return out


def main():
    old = {p["id"]: p for p in json.loads(MAP.read_text()).get("properties", [])} if MAP.exists() else {}
    props = []
    for path in sorted(Path(__file__).parent.glob("batch_*.py")):
        spec = importlib.util.spec_from_file_location(path.stem, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for prop in module.PROPERTIES:
            prop = dict(prop, checks=expand(prop["checks"]))
            prev = old.get(prop["id"])
            if prev and prev.get("mutation") == prop.get("mutation") and prev.get("tests") == prop.get("tests") \
                    and prev.get("evidence"):
                prop["evidence"] = prev["evidence"]
            props.append(prop)
    complete = "--complete" in sys.argv
    MAP.write_text(json.dumps({"base_revision": FROZEN["base_revision"], "assertions_digest": FROZEN["digest"],
                               "complete": complete, "properties": props}, indent=1, ensure_ascii=False) + "\n")
    covered = {c for p in props for c in p["checks"]}
    print(f"{len(props)} properties, {len(covered)}/{len(FROZEN['assertions'])} checks")


if __name__ == "__main__":
    main()
