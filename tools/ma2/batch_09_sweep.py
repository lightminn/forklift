"""M-a2 batch 9: the core sweep's survivors (tools/ma2_core_sweep.py --all, ws1 job 3132: 542 mutations of the core
places every frozen test read, 40 alive). Each got a behaviour test; ``batch_09_sweep.json`` (written by
``ma2_core_sweep.py --emit``, job 3136) holds each mutation as a find/replace and the tests that fail on it. The one
survivor left after the tests (job 3136's re-sweep: 538 killed, 1 alive) is the unreachable backoff realign site,
already X (B7:s2-site-backoff)."""

import json
from pathlib import Path

CORE = "src/forklift_core/mission/core.py"
# Where the sweep's own mutation (a whole statement replaced by ``pass``) breaks a name rather than the property, or
# its first failing test fails by an exception before the test that checks it: the mutation that breaks the
# property itself, and the test that checks it.
OVERRIDES = {
    "L892-test_true": {"tests": ["tests/unit/mission/test_core_blocked_replan.py::"
                                 "test_when_every_remaining_candidate_fails_the_search_ends_with_the_leg_failed"]},
    "L2741-pass": {"find": '"observe", grid_world(scenario), geometry, pickup_obstacle=pickup_,',
                   "replace": '"transport", grid_world(scenario), geometry, pickup_obstacle=pickup_,'},
    "L2742-pass": {"find": "geometry, pickup_obstacle=pickup_, pickup_bounds=pickup_bounds\n        )\n        return GridChecker",
                   "replace": "geometry, pickup_obstacle=None, pickup_bounds=pickup_bounds\n        )\n        return GridChecker"},
    "L3592-pass": {"find": "grid_world(return_scenario), start, return_to_pose, tight_cfg, geometry=geometry,",
                   "replace": "grid_world(return_scenario), start, return_to_pose, planner_config, geometry=geometry,"},
    "L3594-pass": {"find": "grid_world(return_scenario), start, return_to_pose, tight_cfg, geometry=geometry,",
                   "replace": "grid_world(scenario), start, return_to_pose, tight_cfg, geometry=geometry,"},
}
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
    over = OVERRIDES.get(key)
    if over:
        PROPERTIES[-1]["tests"] = over.get("tests", PROPERTIES[-1]["tests"])
        PROPERTIES[-1]["mutation"].update({k: v for k, v in over.items() if k in ("find", "replace")})
