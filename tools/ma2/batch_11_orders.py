"""M-a2 batch 11: orders the Codex implementation review round 4 found among the swaps first judged equivalent, and
the rest of a stricter re-triage (docs/validation/2026-10-16-m-a2-sweep-equivalent.json states the rule): a refusal
before the wait it would set, a callback that needs a name bound before the call, the paths file written with the
record just updated, a release's before and after poses, the phase clock and the stand label set before a refusing
realignment install. ``batch_11_orders.json`` (``--emit --operators``, ws1 job 3166) holds each swap as a
find/replace with the tests that now fail on it."""

import json
from pathlib import Path

CORE = "src/forklift_core/mission/core.py"
PROPERTIES = []
_seen: dict = {}
for row in json.loads((Path(__file__).with_suffix(".json")).read_text()):
    key = f"L{row['line']}-{row['kind']}"
    _seen[key] = _seen.get(key, 0) + 1
    PROPERTIES.append({
        "id": f"B11:{key}" + (f"-{_seen[key]}" if _seen[key] > 1 else ""), "class": "N", "checks": row["checks"],
        "statement": f"order: `{row['what']}` (core.py line {row['line']}) as written",
        "tests": row["guards"][:4],
        "mutation": {"file": CORE, "find": row["find"], "replace": row["replace"], "count": 1},
    })
