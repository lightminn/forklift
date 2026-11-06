"""The shadow's same-process audit (sim/isaac/shadow_audit.py, plan 2026-11-06-unconfirmed-validation 1.1 as the lead
decided 2026-11-06): the control side's objects fingerprinted before and after the shadow computes, every change
counted with its attributes."""

import sys
from collections import deque
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

import shadow_audit as SA  # noqa: E402


class Inner:
    def __init__(self):
        self.best = {(1, 2): [0.1, 0.2]}
        self.grid = np.zeros((4, 4))


class Arbiter:
    def __init__(self):
        self.pose, self.mode, self.inner, self.q = None, "PAUSED", Inner(), deque([1, 2])
        self.me = self  # a cycle


def test_a_nested_change_is_found_by_its_attribute_and_an_unchanged_object_is_not():
    a = Arbiter()
    before = SA.fingerprint(a)
    assert SA.fingerprint(a) == before
    a.inner.best[(1, 2)].append(0.3)
    a.inner.grid[1, 1] = 1.0
    after = SA.fingerprint(a)
    assert [k for k in after if after[k] != before[k]] == ["inner", "me"]  # me: the cycle sees the same change


def test_the_audit_counts_each_site_and_keeps_what_changed():
    a = Arbiter()
    audit = SA.ShadowAudit({"arbiter": lambda: a})
    audit.before()
    audit.after("checkpoint", 1.0)
    audit.before()
    a.pose = {"status": "confirmed"}
    audit.after("note_tick", 2.0)
    rec = audit.record()
    assert rec["checks"] == 2 and rec["changed"] == 1
    assert rec["by_site"] == {"checkpoint": {"checks": 1, "changed": 0}, "note_tick": {"checks": 1, "changed": 1}}
    assert rec["first"] == [{"t": 2.0, "site": "note_tick", "attributes": ["arbiter.me", "arbiter.pose"]}]
