"""A boxed return start through the mission core as the live console runs it (docs/plans/2026-11-08-live-riib-defaults-
validation.md 2.2, the lead's condition for promoting --return-start-backoff boxed on riib evidence): the recorded
riib boxed release replan (rc_G_aisle_rack_s1, no_path from every attempt at one node) asked by MissionCore's release
request, planned by the real planning worker, and taken by MissionCore -- the backoff installed as the return's first,
reverse leg in the path and the tracker, and recorded as adopted. Off (`on` only) the same replan ends the run."""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "planning"))

import test_core_async_plan as A  # noqa: E402
import test_return_start_backoff_boxed as B  # noqa: E402

from forklift_core.control.path_tracking import RearAxlePathTracker  # noqa: E402
from forklift_core.planning import pallet_mission as pm  # noqa: E402
from forklift_core.planning.plan_worker import _run  # noqa: E402


def _core(boxed: bool):
    status, scenario, start, home, config, geometry = B.case("rc_G_aisle_rack_s1")
    assert status == "no_path"  # the recorded release replan: a boxed start
    core, ms, state, obstacle, slam, calls, refreshes = A.make("return_home")
    rear = np.array([start.x_m, start.y_m, start.yaw_rad])
    core.hooks.grid_world = lambda sc: sc  # the Riibotics scene: no grid
    core.hooks.grid_kwargs = lambda *a, **k: {}
    core.hooks.slam_rear = lambda t: rear.copy()
    core.geometry = geometry
    core.options = SimpleNamespace(grid_planning=False, return_turnaround=False, planner_config=config,
                                   travel_config=None, scenario=scenario, return_to_pose=home,
                                   return_start_backoff_m=pm.RETURN_START_BACKOFF_CAP_M,
                                   **({"return_start_backoff_boxed": True} if boxed else {}))
    core.paths["return_home"] = A.Plan("success")  # the return the release replaces
    event = {"event": "release"}
    core._release_request(20.0, {"leg": "return_home", "event": event, "after": tuple(rear),
                                 "return_scenario": scenario, "redo": False})
    (name, payload), = core.jobs.submitted
    answer = _run(name, payload)  # the real planning worker
    core.jobs.answers[1] = (True, answer)
    return core, state, payload, answer, rear, event


def test_a_boxed_start_is_installed_as_the_returns_reverse_backoff_leg_through_the_core():
    core, state, payload, answer, rear, event = _core(boxed=True)
    assert payload["kwargs"]["return_start_backoff_boxed"] is True
    assert payload["tight_kwargs"]["return_start_backoff_boxed"] is True
    core.job_hold(21.0, rear, 0.0, False)
    path = core.paths["return_home"]
    assert path is answer["result"] and event["replanned"] and not core.job_pending()
    rec = path.start_backoff
    assert (rec["status"], rec["trigger"], rec["check"]) == ("prefix", "boxed", "first_step")
    # the tracker the core installed follows that plan, its first leg the straight reverse backoff
    tracker = core.trackers["return_home"]
    assert isinstance(tracker, RearAxlePathTracker)
    assert np.array_equal(tracker._poses, np.asarray(path.poses, dtype=float))
    np.testing.assert_allclose(path.poses[0], rear, atol=1e-9)
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(path.poses[:, :2], axis=0).T))]
    inside = s <= rec["length_m"] - 1e-6
    assert inside.sum() >= 2 and (np.asarray(path.directions)[inside] == -1).all()
    back = path.poses[np.searchsorted(s, rec["length_m"] - 1e-6)]
    heading = np.array([np.cos(rear[2]), np.sin(rear[2])])
    assert float(np.dot(back[:2] - rear[:2], heading)) == pytest.approx(-rec["length_m"], abs=0.02)
    # recorded as the adopted backoff of the release replan
    entry = state["return_start_backoffs"][-1]
    assert entry["where"] == "release_replan" and entry["adopted"] is True and entry["trigger"] == "boxed"


def test_on_alone_the_same_boxed_start_ends_the_run():
    core, state, payload, answer, rear, event = _core(boxed=False)
    assert "return_start_backoff_boxed" not in payload["kwargs"]
    assert not answer["result"].success
    with pytest.raises(RuntimeError, match="slam_release_replan_failed"):
        core.job_hold(21.0, rear, 0.0, False)
