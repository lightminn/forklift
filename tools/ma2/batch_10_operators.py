"""M-a2 batch 10: the operator sweep's survivors (tools/ma2_core_sweep.py --all --operators, ws1 job 3141:
comparison boundaries, membership narrowing and swaps of dependent neighbouring statements on top of the earlier
operators). Each non-equivalent survivor got a behaviour test; ``batch_10_operators.json`` (``--emit --operators``,
job 3150) holds each mutation as a find/replace with the tests that now fail on it, plus the return leg's backoff
narrowing the re-sweep left (job 3150's sweep: 817 killed, 217 alive -- 148 distinct swaps judged equivalent in
docs/validation/2026-10-16-m-a2-sweep-equivalent.json, and the unreachable backoff site, X)."""

import json
from pathlib import Path

CORE = "src/forklift_core/mission/core.py"
PROPERTIES = []
_seen: dict = {}
for row in json.loads((Path(__file__).with_suffix(".json")).read_text()):
    key = f"L{row['line']}-{row['kind']}"
    _seen[key] = _seen.get(key, 0) + 1
    PROPERTIES.append({
        "id": f"B10:{key}" + (f"-{_seen[key]}" if _seen[key] > 1 else ""), "class": "N", "checks": row["checks"],
        "statement": f"operator sweep: {row['kind']} on `{row['what']}` (core.py line {row['line']}) fails a behaviour test",
        "tests": row["guards"][:4],
        "mutation": {"file": CORE, "find": row["find"], "replace": row["replace"], "count": 1},
    })
