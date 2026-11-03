"""The near-leg cusp arrival comparison's frame and tracker identification (lead scope 2026-11-03): errors in the cusp's
frame, and the near-leg trackers proved from the near-serial replans (installed at one, or replaced by the first)."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import near_leg_cusp_arrival as arr  # noqa: E402


def test_errors_are_signed_components_in_the_cusp_frame():
    ref = [1.0, 2.0, math.pi / 2]  # heading +y
    along, lateral, heading = arr.in_frame([0.9, 2.3, math.pi / 2 + 0.1], ref)
    assert math.isclose(along, 0.3, abs_tol=1e-12) and math.isclose(lateral, 0.1, abs_tol=1e-12)
    assert math.isclose(heading, 0.1, abs_tol=1e-12)


def test_near_trackers_are_proved_by_the_near_serial_replans():
    s = {"tracker_versions": [
        {"serial": 1, "phase": "observe", "installed_s": None, "replaced_s": 10.0},   # far, replaced by a far replan
        {"serial": 2, "phase": "observe", "installed_s": 10.0, "replaced_s": 30.0},   # far replan's; replaced by near leg
        {"serial": 5, "phase": "observe", "installed_s": 30.0, "replaced_s": 40.0},   # the near leg (replaced by a miss)
        {"serial": 6, "phase": "observe", "installed_s": 40.0, "replaced_s": None}],  # the miss replan's
        "observe_replans": [{"time_s": 10.0, "leg_serial": 0}, {"time_s": 40.0, "leg_serial": 1}]}
    assert sorted(arr.near_trackers(s, {1})) == [5, 6]
    assert arr.near_trackers({**s, "observe_replans": [{"time_s": 10.0, "leg_serial": 0}]}, {1}) == {}
