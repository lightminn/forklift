"""Real-time plan V8-2 (docs/plans/2026-10-17-realtime-sim.md, D8a): the planning worker.

In process: each job name reaches its planner with the arguments given, the trace comes back filled, and a leg is
asked again with the tight arguments exactly when its first status is a retry status. As a process: a real
transport plan and its objects both ways, a job's failure as text, the control thread's calls never wait (a slow
job, a discard that restarts the worker), a dead worker fails the job it had, and close ends it."""

import time
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.planning import plan_worker as pw
from forklift_core.planning.grid_collision import OccupancyGrid
from forklift_core.planning.pallet_mission import AssetSpec, make_scenario, plan_transport

ASSETS = (AssetSpec("test://crate.usd", 0.450, 0.662, 0.188),)


def wait_for(worker, job_id, timeout_s=60.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        out = worker.poll(job_id)
        if out is not None:
            return out
        time.sleep(0.01)
    raise AssertionError("no answer")


# --- in process: the dispatch ------------------------------------------------------------------------------------
def test_plan_transport_job_passes_its_arguments_and_returns_the_filled_trace(monkeypatch):
    seen = {}

    def fake(*args, **kwargs):
        seen.update(args=args, kwargs=kwargs)
        kwargs["trace"].append("searched")
        return "plan"

    monkeypatch.setattr("forklift_core.planning.pallet_mission.plan_transport", fake)
    out = pw._run("plan_transport", {"args": ("scenario", "config"), "kwargs": {"trace": [], "deadline": 7.0}})
    assert out == {"result": "plan", "trace": ["searched"]}
    assert seen["args"] == ("scenario", "config") and seen["kwargs"]["deadline"] == 7.0


@pytest.mark.parametrize("kind,planner", [("transport", "plan_transport_leg"), ("return", "plan_return_leg")])
@pytest.mark.parametrize("first,retried", [("invalid_start", True), ("no_path", False), ("success", False)])
def test_a_leg_is_asked_again_tight_only_on_a_retry_status(monkeypatch, kind, planner, first, retried):
    calls = []

    def fake(*args, **kwargs):
        calls.append((args, kwargs))
        return SimpleNamespace(status=first if len(calls) == 1 else "tight_ok")

    monkeypatch.setattr(f"forklift_core.planning.pallet_mission.{planner}", fake)
    payload = {"kind": kind, "args": ("sc", "start"), "kwargs": {"deadline": 5.0, "occupancy": "grid"},
               "retry_on": ("invalid_start",), "tight_args": ("sc", "start", "tight"),
               "tight_kwargs": {"deadline": 5.0, "occupancy": "grid"}}
    out = pw._run("leg", payload)
    assert out["tight"] is retried and len(calls) == (2 if retried else 1)
    if retried:
        assert calls[1][0] == ("sc", "start", "tight") and calls[1][1]["deadline"] == calls[0][1]["deadline"]
        assert out["result"].status == "tight_ok"


def test_an_unknown_job_is_refused():
    with pytest.raises(ValueError):
        pw._run("plan_everything", {})


# --- as a process ---------------------------------------------------------------------------------------------------
@pytest.fixture
def worker():
    w = pw.PlanWorker()
    yield w
    w.close()


def test_a_real_transport_plan_and_its_objects_go_both_ways(worker):
    scenario = make_scenario(0, ASSETS)
    trace: list = []
    expected = plan_transport(scenario, trace=trace)
    ok, value = wait_for(worker, worker.submit("plan_transport", {"args": (scenario,), "kwargs": {"trace": []}}))
    assert ok and value["result"].success == expected.success
    np.testing.assert_array_equal(value["result"].transport.poses, expected.transport.poses)
    assert len(value["trace"]) == len(trace)
    grid = OccupancyGrid(-1.0, -2.0, 0.05, np.eye(4, dtype=bool), version=3)
    ok, value = wait_for(worker, worker.submit("echo", {"value": grid}))
    assert ok and value["result"].version == 3 and np.array_equal(value["result"].occupied, grid.occupied)


def test_a_failing_job_comes_back_as_text(worker):
    ok, value = wait_for(worker, worker.submit("echo", {"raise": "no path"}))
    assert not ok and "ValueError: no path" in value
    ok, _ = wait_for(worker, worker.submit("echo", {"value": 1}))  # the worker goes on
    assert ok


def test_the_control_threads_calls_never_wait(worker):
    slow = worker.submit("echo", {"sleep_s": 30.0, "value": "late"})
    behind = worker.submit("echo", {"value": "queued", "pad": b"x" * (4 << 20)})  # a big request behind it
    time.sleep(0.3)
    started = time.monotonic()
    assert worker.poll(slow) is None
    worker.discard(slow)  # the computing job: the I/O thread restarts the worker
    assert time.monotonic() - started < 0.05
    ok, value = wait_for(worker, behind)  # written again to the new process
    assert ok and value["result"] == "queued"
    ok, value = wait_for(worker, worker.submit("echo", {"value": "after"}))
    assert ok and value["result"] == "after" and worker.restarts == 1
    assert worker.poll(slow) is None  # a discarded job never answers


def test_a_big_answer_and_a_big_request_do_not_block_each_other(worker):
    """Codex real-time D8 implementation P1: a 1 MiB answer while a 4 MiB request waits to be written."""
    first = worker.submit("echo", {"sleep_s": 0.5, "value": b"a" * (1 << 20)})
    second = worker.submit("echo", {"value": "second", "pad": b"x" * (4 << 20)})
    ok1, v1 = wait_for(worker, first, 30.0)
    ok2, v2 = wait_for(worker, second, 30.0)
    assert ok1 and len(v1["result"]) == 1 << 20 and ok2 and v2["result"] == "second"


def test_a_dead_worker_fails_the_job_it_had_and_comes_back(worker):
    job = worker.submit("echo", {"sleep_s": 30.0})
    time.sleep(0.3)
    worker.proc.kill()
    ok, value = wait_for(worker, job)
    assert not ok and "exited" in value
    ok, _ = wait_for(worker, worker.submit("echo", {"value": 2}))
    assert ok


def test_a_request_made_right_after_a_discard_is_answered_by_the_new_process(worker):
    """Codex real-time D8 implementation round 6 P1: discard, submit at once -- the new request is not lost."""
    slow = worker.submit("echo", {"sleep_s": 30.0})
    time.sleep(0.3)
    worker.discard(slow)
    fresh = worker.submit("echo", {"value": "fresh"})
    ok, value = wait_for(worker, fresh)
    assert ok and value["result"] == "fresh"


def test_after_a_failed_restart_every_job_fails_at_once(worker, monkeypatch):
    """Codex real-time D8 implementation round 7 P2: no worker any more -- a new job is answered, not kept."""
    slow = worker.submit("echo", {"sleep_s": 30.0})
    time.sleep(0.3)

    def broken():
        raise pw.WorkerFailure("injected")

    monkeypatch.setattr(worker, "_start_process", broken)
    worker.discard(slow)
    time.sleep(0.5)
    ok, value = wait_for(worker, worker.submit("echo", {"value": "fresh"}), 5.0)
    assert not ok and "restart failed" in value


def test_a_worker_that_cannot_even_be_spawned_fails_every_later_job(worker, monkeypatch):
    """Codex real-time D8 implementation round 8 P2: Popen itself failing on a restart."""
    slow = worker.submit("echo", {"sleep_s": 30.0})
    time.sleep(0.3)

    def no_process(*a, **k):
        raise OSError(11, "Resource temporarily unavailable")

    monkeypatch.setattr(pw.subprocess, "Popen", no_process)
    worker.discard(slow)
    time.sleep(0.5)
    ok, value = wait_for(worker, worker.submit("echo", {"value": "fresh"}), 5.0)
    assert not ok and ("could not start" in value or "restart failed" in value)


def test_close_during_a_restart_leaves_no_process():
    w = pw.PlanWorker()
    slow = w.submit("echo", {"sleep_s": 30.0})
    time.sleep(0.3)
    w.discard(slow)
    w.close()
    time.sleep(0.5)
    assert w.proc is None or w.proc.poll() is not None


def test_close_ends_the_process():
    w = pw.PlanWorker()
    proc = w.proc
    w.close()
    assert proc.poll() is not None


# --- observe_ladder: a live mission's first leg (persistent console plan D2, agreed with the main session) ---------
def ladder_payload(waypoints, deadline=None):
    return {"world": SimpleNamespace(start_rear=SimpleNamespace(x_m=1.0, y_m=2.0, yaw_rad=0.5)),
            "waypoints": waypoints, "config": "CFG", "geometry": "GEO", "pickup_bounds": None,
            "grid_kwargs": {"occupancy": "GRID"}, "deadline": deadline}


def fake_leg(monkeypatch, succeed_on=(), sleep_s=0.0):
    calls = []

    def fake(world, waypoint, config, **kwargs):
        calls.append(((waypoint.x_m, waypoint.y_m, waypoint.yaw_rad), kwargs["extended"], kwargs))
        if sleep_s:
            time.sleep(sleep_s)
        ok = (len(calls) - 1) in succeed_on
        return SimpleNamespace(success=ok, status="success" if ok else "no_path", analytic_expansion_interval=4,
                               search_attempts=[("base", 10)])

    monkeypatch.setattr("forklift_core.planning.pallet_mission.plan_observation_leg", fake)
    return calls


def test_the_ladder_runs_every_waypoint_plain_then_every_one_extended_as_the_setup_does(monkeypatch):
    calls = fake_leg(monkeypatch)
    out = pw._run("observe_ladder", ladder_payload([(1, 0, 0), (2, 0, 0), (3, 0, 0)]))
    assert [(c[0][0], c[1]) for c in calls] == [(1, False), (2, False), (3, False), (1, True), (2, True), (3, True)]
    assert calls[0][2]["geometry"] == "GEO" and calls[0][2]["pickup_bounds"] is None
    assert calls[0][2]["occupancy"] == "GRID" and calls[0][2]["deadline"] is None
    # the setup's own order (run_transport, "initial_candidates"): every candidate plain first, then extended
    from pathlib import Path

    source = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()
    assert ("for extended in (False, True)\n            for index, coordinates in enumerate(args.observation_waypoints)"
            in source)
    assert out["result"] is None and out["selected"] is None and not out["timed_out"]
    assert len(out["candidates"]) == 6 and "wall_s" in out


def test_the_ladder_stops_at_the_first_success_with_the_setups_candidate_records(monkeypatch):
    calls = fake_leg(monkeypatch, succeed_on=(4,))  # the second waypoint, extended
    out = pw._run("observe_ladder", ladder_payload([(1, 0, 0), (2, 0, 0.3), (3, 0, 0)]))
    assert len(calls) == 5 and out["result"].success and out["selected"] == 1
    assert out["candidates"][-1] == {"candidate_index": 1, "pose": [2.0, 0.0, 0.3], "success": True,
                                     "status": "success", "analytic_expansion_interval": 4,
                                     "search_attempts": [["base", 10]], "start_rear": [1.0, 2.0, 0.5],
                                     "extended_ladder": True}
    assert [c["status"] for c in out["candidates"][:4]] == ["no_path"] * 4


def test_the_ladder_stops_between_candidates_at_its_deadline(monkeypatch):
    calls = fake_leg(monkeypatch, sleep_s=0.05)
    out = pw._run("observe_ladder", ladder_payload([(1, 0, 0), (2, 0, 0), (3, 0, 0)],
                                                   deadline=time.monotonic() + 0.08))
    assert out["timed_out"] and out["result"] is None and 1 <= len(calls) < 6
    assert len(out["candidates"]) == len(calls)
    assert all(c[2]["deadline"] == calls[0][2]["deadline"] for c in calls)  # the one deadline, to every call


@pytest.mark.parametrize("own, grid, sent", [(5.0, 9.0, 5.0), (None, 9.0, 9.0), (5.0, None, 5.0)])
def test_the_ladders_deadline_is_passed_once_even_when_the_grid_arguments_carry_one(monkeypatch, own, grid, sent):
    """Codex stage 2 review P2: grid_kwargs("observe") returns a deadline while obstacle["deadline"] is set."""
    calls = fake_leg(monkeypatch)
    payload = ladder_payload([(1, 0, 0)], deadline=None)
    payload["deadline"] = own
    payload["grid_kwargs"] = {"occupancy": "GRID", **({} if grid is None else {"deadline": grid})}
    monkeypatch.setattr(pw.time, "monotonic", lambda: 0.0)
    pw._run("observe_ladder", payload)
    assert calls and all(c[2]["deadline"] == sent for c in calls)
