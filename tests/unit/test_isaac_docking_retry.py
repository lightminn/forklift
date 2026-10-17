"""Plan D7c delivery docking retry frame bookkeeping (no Isaac)."""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.planning.pallet_mission import PalletSite

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("docking_retry", ROOT / "sim/isaac/docking_retry.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["docking_retry"] = MODULE
SPEC.loader.exec_module(MODULE)


def test_the_plans_worked_example_moves_the_held_line_start_once():
    # Plan D7c ①: held pose (-1.5, 0), matched line start (-1.5, 0.23); the release moves the
    # estimate by -0.23 m in y, so the line start in the new frame is (-1.5, 0).
    goal = (0.0, 0.23, 0.0)
    docking = {"previous_goal": list(goal), "goal_frame": "held", "to_world": [0.0, 0.0, 0.0]}
    out = MODULE.carry_on_release(docking, (-1.5, 0.0, 0.0), (-1.5, -0.23, 0.0))
    assert out["moved"] and docking["goal_frame"] == "map"
    np.testing.assert_allclose(MODULE.line_start(docking["previous_goal"], 1.5), (-1.5, 0.0, 0.0), atol=1e-12)
    # W = identity, T lateral -0.23 -> W o T^-1 lateral +0.23 (and +0.23 -> -0.23).
    np.testing.assert_allclose(docking["to_world"], (0.0, 0.23, 0.0), atol=1e-12)


def test_a_map_goal_is_not_moved_by_a_later_release():
    # Codex D7c re-review P1-1: the hold re-engaged near L, then a cusp retry releases it.
    docking = {"previous_goal": [0.0, 0.0, 0.0], "goal_frame": "map", "to_world": [0.1, 0.0, 0.0]}
    out = MODULE.carry_on_release(docking, (-1.5, 0.0, 0.0), (-1.5, 0.20, 0.0),
                                  withdraw_poses=np.array([[0.0, 0.0, 0.0]]), destination=(1.0, 0.0, 0.0))
    assert not out["moved"] and docking["previous_goal"] == [0.0, 0.0, 0.0] and docking["to_world"] == [0.1, 0.0, 0.0]
    np.testing.assert_allclose(out["frame_change"], (0.0, 0.20, 0.0), atol=1e-12)
    np.testing.assert_allclose(out["withdraw_poses"], [[0.0, 0.0, 0.0]])
    assert out["destination"] == (1.0, 0.0, 0.0)


def test_everything_held_moves_with_the_goal_including_a_rotation():
    before, after = (2.0, 1.0, 0.3), (2.05, 0.9, 0.32)
    goal, withdraw, dest = (4.0, 1.5, 0.3), np.array([[4.0, 1.5, 0.3], [3.45, 1.33, 0.3]]), (5.0, 1.8, 0.3)
    docking = {"previous_goal": list(goal), "goal_frame": "held"}
    out = MODULE.carry_on_release(docking, before, after, withdraw_poses=withdraw, destination=dest)
    # The truck-relative relation of each held item survives the release.
    for moved, original in [(docking["previous_goal"], goal), (out["destination"], dest),
                            *zip(out["withdraw_poses"], withdraw)]:
        rel_before = MODULE.compose(MODULE.invert(before), tuple(original))
        rel_after = MODULE.compose(MODULE.invert(after), tuple(moved))
        np.testing.assert_allclose(rel_after, rel_before, atol=1e-12)


def test_the_prior_error_moves_the_site_along_its_own_lateral_axis():
    site = PalletSite(3.0, -2.0, math.pi / 2)
    moved = MODULE.destination_with_prior_error(site, 0.2)
    assert (moved.x_m, moved.y_m, moved.yaw_rad) == pytest.approx((2.8, -2.0, math.pi / 2))
