"""F2 V4 report (plan 4.4): an arc keeps its V2 verdict between 120 and 240 Hz; no arc with a window at both dt is not
a report."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f2_v4_report as R  # noqa: E402


def _v2(entries):
    return {"v2": [{"label": lab, "kappa_cmd": k, "window": None if p is None else [0, 10], "pass": p}
                   for lab, k, p in entries]}


def test_preserved_broken_and_no_comparison():
    a = _v2([("fwd", 0.36, True), ("fwd", -0.36, False), ("rev", 0.36, None)])
    assert R.compare(a, _v2([("fwd", 0.36, True), ("fwd", -0.36, False), ("rev", 0.36, True)]))["v4_v2_preserved"] is True
    out = R.compare(a, _v2([("fwd", 0.36, False), ("fwd", -0.36, False), ("rev", 0.36, True)]))
    assert out["v4_v2_preserved"] is False and out["comparable"] == 2
    none = R.compare(_v2([("fwd", 0.36, None)]), _v2([("fwd", 0.36, True)]))
    assert none["v4_v2_preserved"] is None and none["comparable"] == 0
