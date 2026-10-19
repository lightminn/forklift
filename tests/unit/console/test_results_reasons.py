"""Persistent console plan D13 item 3: every reason a mission result can carry has an explanation on the screen
(console/web/js/results.js REASONS). The reasons are read from where results are made, not from a list kept here:
the failures classification (its mission and scene reasons reach a result; the session ones end the session and
are covered too), the delivery verdict, and the preparation and recovery ends in the core."""

import json
import re
from pathlib import Path

from forklift_core.mission import delivery
from forklift_core.mission.failures import PREFIXES

ROOT = Path(__file__).resolve().parents[3]


def table_heads() -> list[str]:
    source = (ROOT / "console/web/js/results.js").read_text()
    body = source[source.index("export const REASONS = {"):source.index("};", source.index("export const REASONS"))]
    return [json.loads(m.group(1)) for m in re.finditer(r'^\s*("(?:[^"\\]|\\.)*")\s*:', body, re.M)]


def core_reasons() -> set[str]:
    core = (ROOT / "src/forklift_core/mission/core.py").read_text()
    found = set()
    for m in re.finditer(r'recovery_failed\(t, f?"([^"{]*)', core):
        found.add("recovery:" + m.group(1))
    for m in re.finditer(r'terminate\("[a-z_]+", t, f?"([^"{]*)', core):
        found.add(m.group(1))
    return found


def explained(reason: str, heads: list[str]) -> bool:
    return any(reason.startswith(h) for h in heads)


def test_the_table_explains_every_reason_a_result_can_carry():
    heads = table_heads()
    wanted = set(PREFIXES) | {delivery.MISSED, delivery.NOT_GROUNDED, delivery.FORK_INSIDE} | core_reasons()
    wanted |= {"operator", "session_end", "prepare:all_candidates_failed:invalid_start"}
    missing = sorted(r for r in wanted if not explained(r, heads))
    assert not missing, missing


def test_the_core_reasons_were_found_where_they_are_made():
    # the scan itself: a change of the call shapes must not quietly empty it
    reasons = core_reasons()
    assert {"recovery:clear_out:budget", "recovery:trace_invalid", "prepare:timeout", "prepare:error:"} <= reasons


def test_every_classified_head_has_its_own_entry():
    """A classified head explained only by a shorter generic one would lose its meaning (near_field_contract is a
    session failure, near_field_ a mission's)."""
    heads = set(table_heads())
    assert not sorted(set(PREFIXES) - heads)
