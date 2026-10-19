"""Persistent console plan D11 (5th ed.): the return turnaround is planned by the worker with a frozen arc check;
its correction drops have a count of their own, outside the shared drop limit, and after three that return is
planned in the tick as before; every other drop reason still counts against the shared limit."""

from types import SimpleNamespace

import numpy as np
import pytest

from test_core_async_plan import REAR, Plan, make  # noqa: E402


class Checker:
    callback_spec = {"kind": "turnaround_arc"}

    def __call__(self, poses):
        return None


def turnaround_core(phase="return_home"):
    core, ms, state, obstacle, slam, calls, refreshes = make(phase)
    core.options.return_turnaround = True
    core.options.return_to_pose = "HOME"
    obstacle["layer"].turnaround_arc_check = lambda callback_spec=None: Checker()
    obstacle["layer"].check_turnaround_arc = lambda poses, current_pose: None
    slam["pending_release"] = None
    slam["return_scenario"] = SimpleNamespace(destination="DEST")
    sync = []
    core.hooks.plan_return_leg = lambda *a, **k: sync.append(k) or Plan("success")
    return core, state, obstacle, slam, sync


def answer(core, job, *, applied=None, slam=None):
    if applied is not None:
        slam["tracker"].applied = (applied, 0)
    core.jobs.answers[job] = (True, {"result": Plan("success"), "tight": False})


def test_the_return_turnaround_goes_to_the_worker_with_a_frozen_check():
    core, state, obstacle, slam, sync = turnaround_core()
    assert core.live_leg(20.0, REAR, False) is True
    name, payload = core.jobs.submitted[0]
    assert name == "leg" and payload["kind"] == "return" and not sync
    assert payload["kwargs"]["forward_turnaround"] is True and isinstance(payload["kwargs"]["turnaround_check"], Checker)


def test_no_return_plan_is_asked_while_the_release_is_pending():
    """Codex D11 design round 1 P2-3: the outer release wait stays (batch and live)."""
    core, state, obstacle, slam, sync = turnaround_core()
    slam["pending_release"] = "settle"
    core.live_leg(20.0, REAR, False)
    assert not core.jobs.submitted and not sync


def test_three_correction_drops_of_the_return_fall_back_to_the_tick_plan_without_failing():
    core, state, obstacle, slam, sync = turnaround_core()
    for k in range(3):
        core.live_leg(20.0 + k, REAR, False)
        answer(core, k + 1, applied=(0.1 + 1e-6 * (k + 1), 0.0, 0.0), slam=slam)  # the correction moved by 1 um
        core.obstacle["applied"] = slam["tracker"].applied[0]
        core.job_hold(20.5 + k, REAR, 0.0, False)
        assert state["async_plans"][-1]["dropped"] == "correction"
    assert state.get("async_plan_drops", {}).get("live_leg", 0) == 0  # outside the shared limit
    assert core.return_correction_drops["return_home"] == 3
    core.live_leg(30.0, REAR, False)
    assert len(core.jobs.submitted) == 3 and len(sync) == 1  # planned in the tick, as before D11
    assert sync[0]["forward_turnaround"] is True and callable(sync[0]["turnaround_check"])


def test_a_transport_drop_and_three_return_correction_drops_do_not_end_the_mission():
    """Codex D11 design round 4 P2: the shared count reached four before the fallback."""
    core, state, obstacle, slam, sync = turnaround_core("transport")
    core.live_leg(10.0, REAR, True)
    answer(core, 1)
    core.job_hold(10.5, REAR + [0.03, 0.0, 0.0], 0.0, False)  # drift: the shared count
    assert state["async_plan_drops"] == {"live_leg": 1}
    core.ms.phase = "return_home"
    core.trackers["return_home"] = core.trackers.pop("transport")
    obstacle["live_replanned"] = {}
    for k in range(3):
        core.live_leg(20.0 + k, REAR, False)
        answer(core, len(core.jobs.submitted), applied=(0.2 + 1e-6 * k, 0.0, 0.0), slam=slam)
        core.obstacle["applied"] = slam["tracker"].applied[0]
        core.job_hold(20.5 + k, REAR, 0.0, False)
    assert state["async_plan_drops"] == {"live_leg": 1}
    core.live_leg(30.0, REAR, False)
    assert len(sync) == 1


def test_a_failed_fallback_retries_in_the_tick_not_the_worker():
    core, state, obstacle, slam, sync = turnaround_core()
    core.return_correction_drops["return_home"] = 3
    core.hooks.plan_return_leg = lambda *a, **k: sync.append(k) or Plan("no_path")
    core.live_leg(20.0, REAR, False)
    assert obstacle["live_retry_after_s"] == pytest.approx(21.0) and len(sync) == 2  # the tight one too
    core.live_leg(21.0, REAR, False)
    assert not core.jobs.submitted and len(sync) == 4


def test_other_drop_reasons_of_the_return_still_count_against_the_shared_limit():
    core, state, obstacle, slam, sync = turnaround_core()
    core.live_leg(20.0, REAR, False)
    answer(core, 1)
    core.job_hold(20.5, REAR + [0.03, 0.0, 0.0], 0.0, False)
    assert state["async_plans"][-1]["dropped"] == "drift" and state["async_plan_drops"] == {"live_leg": 1}


def test_the_fallback_survives_a_pause_and_a_resume_and_a_new_mission_starts_it_over():
    """Codex D11 impl round 1 P2: the count was keyed by the commands generation, which a pause and a resume raise
    -- the fallback came undone in the same return. P3: it is a mission's, restored by the baseline."""
    from forklift_core.mission.baseline import CORE_FIELDS, MISSION_KEYS

    core, state, obstacle, slam, sync = turnaround_core()
    core.return_correction_drops["return_home"] = 3
    core.commands.mission_gen += 2  # a pause and a resume
    core.live_leg(20.0, REAR, False)
    assert not core.jobs.submitted and len(sync) == 1
    assert "return_correction_drops" in CORE_FIELDS and "return_correction_drops" in MISSION_KEYS["state"]
