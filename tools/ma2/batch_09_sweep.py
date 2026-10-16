"""M-a2 batch 9: the core sweep's survivors (tools/ma2_core_sweep.py --all, ws1 job 3132: 542 mutations of the core
places every frozen test read, 40 alive). Each got a behaviour test; ``batch_09_sweep.json`` (written by
``ma2_core_sweep.py --emit``, job 3136) holds each mutation as a find/replace and the tests that fail on it. The one
survivor left after the tests (job 3136's re-sweep: 538 killed, 1 alive) is the unreachable backoff realign site,
already X (B7:s2-site-backoff)."""

import json
from pathlib import Path

CORE = "src/forklift_core/mission/core.py"
PROPERTIES = []
_seen: dict = {}
for row in json.loads((Path(__file__).with_suffix(".json")).read_text()):
    key = f"L{row['line']}-{row['kind']}"
    _seen[key] = _seen.get(key, 0) + 1
    PROPERTIES.append({
        "id": f"B9:{key}" + (f"-{_seen[key]}" if _seen[key] > 1 else ""), "class": "N", "checks": row["checks"],
        "statement": f"core sweep: {row['kind']} on `{row['what']}` (core.py line {row['line']}) fails a behaviour test",
        "tests": row["guards"][:4],
        "mutation": {"file": CORE, "find": row["find"], "replace": row["replace"], "count": 1},
    })
