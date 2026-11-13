"""C3 record diagnosis (docs/validation/2026-11-13-baseline-fragility/c3_diag.py): the decomposition is pointed at the
second near capture when it was adopted, and left alone otherwise. Synthetic records, no Isaac."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("c3_diag", ROOT / "docs/validation/2026-11-13-baseline-fragility/c3_diag.py")
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)


def test_the_second_capture_is_used_only_when_adopted():
    result = {"near_capture": {"near_attempt": 2, "near_estimate_m": {"x_m": 1.0, "y_m": 0.0},
                               "second": {"status": "adopted", "capture_attempt": 3, "target": [1.1, 0.02, 0.0]}}}
    used = C.last_capture(result)
    assert used["near_capture"]["near_attempt"] == 3
    assert used["near_capture"]["near_estimate_m"] == {"x_m": 1.1, "y_m": 0.02}
    assert result["near_capture"]["near_attempt"] == 2  # the input is not changed
    refused = {"near_capture": {"near_attempt": 2, "second": {"status": "skipped_short"}}}
    assert C.last_capture(refused) is refused
    assert C.last_capture({"near_capture": {"near_attempt": 2}})["near_capture"]["near_attempt"] == 2


def test_names():
    m = C.NAME_RE.match("t2d_c3_line_1_m_s4")
    assert (m["arm"], m["scene"], m["seed"]) == ("c3", "line_1_m", "4")
