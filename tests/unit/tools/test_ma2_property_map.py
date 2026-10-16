"""The M-a2 property map accounts for the frozen checks, with evidence (docs/plans/2026-10-16-m-a2-source-tests-to-
behaviour.md, 2.4), and the checker itself fails on a broken map (agy 12)."""

import copy
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("ma2_property_check", ROOT / "tools/ma2_property_check.py")
C = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(C)
MAP = json.loads((ROOT / "tests/ma2_property_map.json").read_text())
FROZEN = json.loads((ROOT / "docs/validation/2026-10-16-m-a2-assertions.json").read_text())


def test_the_property_map_has_no_problems():
    assert C.check(MAP, FROZEN) == []


def _toy():
    frozen = {"digest": "d", "assertions": [{"id": "a::t::1"}, {"id": "a::t::2"}]}
    prop = {"id": "P1", "statement": "s", "class": "N", "checks": ["a::t::1", "a::t::2"],
            "tests": ["tests/unit/tools/test_ma2_property_map.py::test_the_property_map_has_no_problems"],
            "mutation": {"file": "f", "find": "x", "replace": "y"},
            "evidence": {"baseline": "passed", "mutated": "failed_on_check"}}
    return {"assertions_digest": "d", "complete": True, "properties": [prop]}, frozen


def test_the_checker_fails_on_a_missing_check_a_missing_evidence_or_an_unknown_test():
    good, frozen = _toy()
    assert C.check(good, frozen) == []
    dropped = copy.deepcopy(good)
    dropped["properties"][0]["checks"] = ["a::t::1"]
    assert any("no property accounts for" in p for p in C.check(dropped, frozen))
    no_evidence = copy.deepcopy(good)
    no_evidence["properties"][0]["evidence"] = {"baseline": "passed", "mutated": "passed"}
    assert any("no evidence" in p for p in C.check(no_evidence, frozen))
    ghost = copy.deepcopy(good)
    ghost["properties"][0]["tests"] = ["tests/unit/tools/test_ma2_property_map.py::test_no_such_test"]
    assert any("does not exist" in p for p in C.check(ghost, frozen))
    stale = copy.deepcopy(good)
    stale["assertions_digest"] = "other"
    assert any("digest differs" in p for p in C.check(stale, frozen))
    unknown = copy.deepcopy(good)
    unknown["properties"][0]["checks"].append("a::t::9")
    assert any("unknown check" in p for p in C.check(unknown, frozen))
    kept = copy.deepcopy(good)
    kept["properties"][0].update({"class": "R", "reason": ""})
    assert any("needs a reason" in p for p in C.check(kept, frozen))
