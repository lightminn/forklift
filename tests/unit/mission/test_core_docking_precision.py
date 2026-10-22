"""The core side of the sim-only weak-direction docking acceptance (docs/plans/2026-10-22-open-zone-docking.md D2):
the cumulative bound over rounds measured on the goal positions, and the acceptance reaching both dock calls only when
it is on."""

import math
from pathlib import Path

import pytest

from forklift_core.localization.scan_docking import DockingResult
from forklift_core.mission.core import MissionCore

ROOT = Path(__file__).resolve().parents[3]
DELIVERY = (0.678, -8.946, 0.0)  # line_2's delivery rear goal (Codex v4 example)


def result(goal, reason="ok:precision", accepted=True):
    return DockingResult(accepted, reason, goal, (0, 0, 0), (0, 0, 0), (), (), {"reason": "ok"})


def test_a_goal_within_the_bound_is_kept_and_its_cumulative_move_recorded():
    r = result((DELIVERY[0] + 0.10, DELIVERY[1] - 0.15, 0.0))
    out = MissionCore._precision_cumulative(r, DELIVERY)
    assert out.accepted and out.goal_estimate == r.goal_estimate
    assert out.precision["cumulative_m"] == pytest.approx(math.hypot(0.10, 0.15))


def test_a_turn_in_place_is_no_movement():
    r = result((DELIVERY[0], DELIVERY[1], 0.025))  # yaw only: the path transform's translation would read 0.22 m
    assert MissionCore._precision_cumulative(r, DELIVERY).accepted


def test_rounds_adding_up_past_the_bound_are_refused():
    # 0.19 m then 0.19 m from the latest goal (a D7c re-approach reseeds) -- 0.38 m from the first SLAM goal
    r = result((DELIVERY[0], DELIVERY[1] + 0.38, 0.0))
    out = MissionCore._precision_cumulative(r, DELIVERY)
    assert not out.accepted and out.reason == "degenerate:cumulative" and out.goal_estimate is None
    assert out.precision["cumulative_m"] == pytest.approx(0.38)


def test_a_turn_about_the_world_origin_that_moves_the_goal_is_caught():
    # the goal pose turned 0.03 rad about the world origin: 0.27 m away although the whole-path transform is ~0
    c, s = math.cos(0.03), math.sin(0.03)
    goal = (c * DELIVERY[0] - s * DELIVERY[1], s * DELIVERY[0] + c * DELIVERY[1], 0.03)
    out = MissionCore._precision_cumulative(result(goal), DELIVERY)
    assert not out.accepted and out.precision["cumulative_m"] == pytest.approx(0.2691, abs=1e-3)


def test_other_results_pass_untouched():
    for r in (result((5.0, 5.0, 0.0), reason="ok"), result(None, reason="start_rejected:degenerate", accepted=False)):
        assert MissionCore._precision_cumulative(r, DELIVERY) is r


def test_the_acceptance_reaches_the_worker_and_the_direct_call_only_when_on():
    source = (ROOT / "src/forklift_core/mission/core.py").read_text()
    block = source[source.index("acceptance_ = getattr(options, \"docking_acceptance\", \"information\")"):][:1600]
    assert 'acceptance_kw = {} if acceptance_ == "information" else {"acceptance": acceptance_}' in block
    assert "delivery_rear=seed, **acceptance_kw)}" in block  # the worker's dock job
    assert "delivery_rear=seed,\n                **acceptance_kw," in block  # the synchronous call
    assert "result = self._precision_cumulative(result, delivery)" in block
