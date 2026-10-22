"""The live far recognition's worker thread and its tick side (docs/plans/2026-10-22-live-far-recognition.md T1 (A),
tasks 3-8): the frames' pure work off the control thread, applied on the tick in order, dropped by tag."""
import importlib.util
import threading
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tools import far_batch_golden as G

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("far_recognition_runtime", ROOT / "sim/isaac/far_recognition_runtime.py")
R = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R)
DT = G.DT


def _wait_idle(live, timeout=30.0):
    done = threading.Event()
    for _ in range(int(timeout / 0.01)):
        if live.worker.idle():
            done.set()
            break
        threading.Event().wait(0.01)
    assert done.is_set()


class _Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class _Gate:
    """A compute that waits for its release (a slow model), then does the real work."""

    def __init__(self, compute):
        self.compute, self.go, self.started = compute, threading.Event(), threading.Event()

    def __call__(self, job):
        self.started.set()
        assert self.go.wait(30)
        return self.compute(job)


def _drive_live(live, xs, gen=1, lag=11, ticks=None, pump=True):
    run = live.rt
    for tick in (range(len(xs)) if ticks is None else ticks):
        base = (float(xs[tick]), 0.0, 0.0)
        run.record_step(tick, tick * DT, base, base, G.PALLET)
        if run.due(tick, observing=True) and tick >= lag:
            scene, ids = G._render(R, (float(xs[tick - lag]), 0.0, 0.0))
            live.drive(gen, tick, tick * DT, scene.rgb, scene.depth_m, ids, scene.intrinsics, scene.base_from_optical)
        if pump:
            live.pump(gen, active=True)


def _stop_kw(xs, stamp_s):
    base = (float(xs[-1]), 0.0, 0.0)
    scene, ids = G._render(R, base)
    scene = replace(scene, stamp_ns=int(round(stamp_s * 1e9)))
    return dict(scene_input=scene, instance_ids=ids, initial_time_s=0.0, control_base=base, odom_base=base,
                truth_base=base, truth_pallet=G.PALLET)


def test_live_frames_applied_on_the_tick_decide_as_the_batch_run_does():
    xs = G._leg()
    batch = G._runtime(R, lag_steps=11)
    G._drive(R, batch, xs, 11)
    obs_b, rec_b = G._stop(R, batch, (float(xs[-1]), 0.0, 0.0), len(xs) * DT + 0.1)
    live = R.LiveFar(G._runtime(R, lag_steps=11))
    try:
        _drive_live(live, xs)
        handle = live.stop(1, **_stop_kw(xs, len(xs) * DT + 0.1))
        assert live.rt.stop_pending and not live.rt.due(24 * 1000, observing=True)  # no frame while it waits
        _wait_idle(live)
        live.pump(1, active=True)
        assert live.stop_state(handle) == "done"
        obs, rec = live.take(handle)
    finally:
        live.close()
    assert obs.status == obs_b.status == "valid" and rec["source"] == rec_b["source"] == "fused"
    assert rec["used"] == rec_b["used"] and rec["observations"] == rec_b["observations"]
    assert not live.rt.stop_pending and all(j["status"] == "applied" for j in live.log)


def test_the_acquisition_pose_is_taken_when_the_frame_is_handed_over():
    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    gate = _Gate(run.compute)
    live = R.LiveFar(run, worker=R.FarWorker(gate))
    try:
        _drive_live(live, xs[:60], pump=False, ticks=range(60))  # frames at ticks 24 and 48 handed over
        for k in range(60, 400):  # the step log (256) moves well past the frames' poses before their work ends
            run.record_step(k, k * DT, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), G.PALLET)
        gate.go.set()
        _wait_idle(live)
        live.pump(1, active=True)
    finally:
        live.close()
    applied = [r for r in run.frame_records if r["kind"] == "drive"]
    assert applied and all("skipped" not in r for r in applied)


def test_a_late_result_of_an_earlier_generation_is_dropped_and_changes_nothing():
    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    gate = _Gate(run.compute)
    live = R.LiveFar(run, worker=R.FarWorker(gate))
    try:
        _drive_live(live, xs, pump=False, ticks=range(60))
        assert gate.started.wait(10)
        live.pump(2, active=True)  # a STOP boundary moved the generation while the work ran
        gate.go.set()
        _wait_idle(live)
        live.pump(2, active=True)
    finally:
        live.close()
    assert run.frame_records == [] and run.far.tracker.selected is None
    assert {j["why"] for j in live.log} == {"generation"} and all(j["status"] == "dropped" for j in live.log)


def test_results_of_an_inactive_session_are_dropped():
    xs = G._leg()
    live = R.LiveFar(G._runtime(R, lag_steps=11))
    try:
        _drive_live(live, xs, pump=False, ticks=range(60))
        _wait_idle(live)
        live.pump(1, active=False)
    finally:
        live.close()
    assert live.rt.frame_records == [] and {j["why"] for j in live.log} == {"inactive"}


def test_the_queue_keeps_four_drive_frames_and_drops_the_oldest_waiting_never_the_stop():
    gate = _Gate(lambda job: None)
    worker = R.FarWorker(gate)
    try:
        jobs = [{"kind": "drive", "seq": i} for i in range(6)]
        worker.submit(jobs[0])
        assert gate.started.wait(10)  # job 0 running
        for j in jobs[1:]:
            worker.submit(j)
        stop = {"kind": "stop", "seq": 99}
        worker.submit(stop)
        dropped = [(j["seq"], why) for j, status, why in worker.poll() if status == "dropped"]
        assert dropped == [(1, "queue_full"), (2, "queue_full")]
        gate.go.set()
        for _ in range(3000):
            if worker.idle():
                break
            threading.Event().wait(0.01)
        done = [j["seq"] for j, status, _ in worker.poll() if status == "done"]
        assert done == [0, 3, 4, 5, 99]  # in submission order, the stop after the frames before it
    finally:
        worker.close()


def test_a_clear_drops_the_waiting_work_and_the_pending_stop():
    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    gate = _Gate(run.compute)
    live = R.LiveFar(run, worker=R.FarWorker(gate))
    try:
        _drive_live(live, xs, pump=False, ticks=range(100))
        handle = live.stop(1, **_stop_kw(xs, 1.0))
        live.clear("stop")
        assert live.stop_state(handle) == "invalid" and handle.why == "stop" and not run.stop_pending
        gate.go.set()
        _wait_idle(live)
        live.pump(1, active=True)
    finally:
        live.close()
    whys = [j["why"] for j in live.log]
    assert "stop" in whys and run.attempts == []  # the running frame may land; the stop never decides


def test_a_stop_still_waiting_after_thirty_wall_seconds_fails_the_mission():
    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    gate = _Gate(run.compute)
    clock = _Clock()
    live = R.LiveFar(run, worker=R.FarWorker(gate), clock=clock)
    try:
        handle = live.stop(1, **_stop_kw(xs, 1.0))
        clock.t += 29.9
        assert live.stop_state(handle) == "waiting"
        clock.t += 0.2
        with pytest.raises(RuntimeError, match="^recognition_timeout$"):
            live.stop_state(handle)
        assert not run.stop_pending
    finally:
        gate.go.set()
        live.close()


def test_a_leg_lost_before_the_stop_invalidates_it():
    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    live = R.LiveFar(run)
    try:
        _drive_live(live, xs, ticks=range(200))
        _wait_idle(live)
        live.pump(1, active=True)
        assert run.far.tracker.selected is not None
        still = (float(xs[199]), 0.0, 0.0)
        scene, ids = G._render(R, still)
        empty = np.full_like(ids, -1)
        gate = _Gate(run.compute)
        live.worker.close()
        live.worker = R.FarWorker(gate)
        for k in range(200, 360):  # the pallet out of view: the leg's target is deleted
            run.record_step(k, k * DT, still, still, G.PALLET)
            if run.due(k, observing=True):
                live.drive(1, k, k * DT, scene.rgb, scene.depth_m, empty, scene.intrinsics, scene.base_from_optical)
        handle = live.stop(1, **_stop_kw([still[0]], 3.1))
        gate.go.set()
        _wait_idle(live)
        live.pump(1, active=True)
    finally:
        live.close()
    assert live.stop_state(handle) == "invalid" and handle.why == "leg" and run.attempts == []
    assert run.leg_endings and not run.stop_pending


def test_a_worker_failure_is_a_mission_failure_of_its_own():
    def broken(job):
        raise ValueError("bad frame")

    xs = G._leg()
    live = R.LiveFar(G._runtime(R, lag_steps=11), worker=R.FarWorker(broken))
    try:
        _drive_live(live, xs, pump=False, ticks=range(30))
        _wait_idle(live)
        with pytest.raises(RuntimeError, match="^recognition_error:ValueError$"):
            live.pump(1, active=True)
    finally:
        live.close()
    assert live.log[-1]["status"] == "error"


def test_a_drive_frame_read_that_raises_is_raised_by_the_tick_not_inside_the_step():
    live = R.LiveFar(G._runtime(R, lag_steps=11))
    try:
        live.read_frame(lambda: (_ for _ in ()).throw(ValueError("two labels")))  # inside the physics step
        with pytest.raises(RuntimeError, match="^recognition_error:ValueError$"):
            live.pump(1, active=True)  # the tick, after the step finished its sensors
        live.pump(1, active=True)  # raised once
    finally:
        live.close()


def test_a_mission_start_gives_fresh_state_its_own_directory_and_statistics(tmp_path):
    xs = G._leg()
    model = G.TruthModel(0.9)
    source = R.LearnedSource(model)
    run = G._runtime(R, lag_steps=11, proposer=source, identity_threshold=0.5)
    live = R.LiveFar(run)
    try:
        live.start_mission("m1", zone_centre_xy=G.PALLET[:2], zone_radius_m=0.6, out_dir=tmp_path / "m1")
        for tick, x in enumerate(xs[:240]):
            base = (float(x), 0.0, 0.0)
            run.record_step(tick, tick * DT, base, base, G.PALLET)
            if run.due(tick, observing=True) and tick >= 11:
                model._ids = G._render(R, (float(xs[tick - 11]), 0.0, 0.0))
                scene, ids = model._ids
                live.drive(1, tick, tick * DT, scene.rgb, scene.depth_m, ids, scene.intrinsics,
                           scene.base_from_optical)
                _wait_idle(live)
                live.pump(1, active=True)
        assert run.far.tracker.selected is not None
        times_m1 = source.times_ms
        n = len(times_m1)
        live.start_mission("m2", zone_centre_xy=(9.0, 1.0), zone_radius_m=0.4, out_dir=tmp_path / "m2")
    finally:
        live.close()
    assert run.far.tracker.selected is None and run.far.selection_history == [] and run.leg == 0
    assert run.frame_records == [] and run.attempts == [] and run.leg_steps == [] and live.log == []
    assert run.out_dir == tmp_path / "m2" and run.zone_initial == {"centre_xy": [9.0, 1.0], "radius_m": 0.4}
    assert source.times_ms == [] and times_m1 is not source.times_ms and len(times_m1) == n > 0


def test_frames_go_to_the_writer_and_its_refusals_are_the_missions_record(tmp_path):
    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    written, refuse = [], {"on": False}

    class _IO:
        def put_line(self, job, nbytes):
            if refuse["on"]:
                return False
            written.append(nbytes)
            job()
            return True

    live = R.LiveFar(run, io=_IO())
    try:
        live.start_mission("m1", zone_centre_xy=G.PALLET[:2], zone_radius_m=0.6, out_dir=tmp_path / "m1" / "recognition")
        _drive_live(live, xs, ticks=range(100))
        _wait_idle(live)
        live.pump(1, active=True)
        refuse["on"] = True
        _drive_live(live, xs, ticks=range(100, 130))
        _wait_idle(live)
        live.pump(1, active=True)
        summary = live.summary()
    finally:
        live.close()
    assert written and sorted((tmp_path / "m1" / "recognition").glob("frame_*.npz"))
    assert summary["live"]["record_errors"] and "refused" in summary["live"]["record_errors"][0]


def test_the_capture_guard_pauses_frames_until_the_session_ends_however_it_ends():
    class _S:
        def __init__(self, steps):
            self.steps, self.result, self.done = steps, None, False

        def start(self):
            return False

        def stepped(self, error=None):
            self.steps -= 1
            if self.steps < 0:
                raise ValueError("decode")
            self.done = self.steps == 0
            self.result = "raw" if self.done else None
            return self.done

        def close(self):
            pass

    run = G._runtime(R)
    s = R.GuardedSession(_S(2), run)
    assert run.paused and s.start() is False and run.paused
    assert s.stepped() is False and run.paused
    assert s.stepped() is True and not run.paused and s.result == "raw"
    s = R.GuardedSession(_S(5), run)
    s.close()
    assert not run.paused
    s = R.GuardedSession(_S(0), run)
    with pytest.raises(ValueError):
        s.stepped()
    assert not run.paused


@pytest.mark.parametrize("status, observing, stopped, ok", [
    ("running", True, False, True), ("running", True, True, False), ("running", False, False, False),
    ("idle", True, False, False), ("cancelling", True, False, False), ("done", True, False, False)])
def test_the_live_gate_takes_frames_only_for_a_running_observe_mission_without_a_stop(status, observing, stopped, ok):
    assert R.live_observing(status, observing, stopped) is ok


def test_live_jobs_are_recorded_with_their_order_and_the_recompute_replays_the_applied_ones(tmp_path):
    import json

    from tools import recompute_far_recognition as RC

    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    gate = _Gate(run.compute)
    live = R.LiveFar(run)
    try:
        live.start_mission("m1", zone_centre_xy=G.PALLET[:2], zone_radius_m=0.6,
                           out_dir=tmp_path / "m1" / "recognition")
        _drive_live(live, xs, ticks=range(100))
        _wait_idle(live)
        live.pump(1, active=True)
        live.worker.close()
        live.worker = R.FarWorker(gate)
        _drive_live(live, xs, ticks=range(100, 160), pump=False)
        assert gate.started.wait(10)
        live.pump(2, active=True)  # a boundary: the running and waiting frames of generation 1 go
        gate.go.set()
        _wait_idle(live)
        live.pump(2, active=True)
        _drive_live(live, xs, gen=2, ticks=range(160, len(xs)))
        _wait_idle(live)
        live.pump(2, active=True)
        handle = live.stop(2, **_stop_kw(xs, len(xs) * DT + 0.1))
        _wait_idle(live)
        live.pump(2, active=True)
        assert live.stop_state(handle) == "done"
        live.take(handle)
        summary = live.summary()
    finally:
        live.close()
    jobs = summary["live"]["jobs"]
    assert [j["order"] for j in jobs if j["status"] == "applied"] == list(range(1, 1 + sum(
        j["status"] == "applied" for j in jobs)))
    assert any(j["status"] == "dropped" and j["why"] == "generation" for j in jobs)
    (tmp_path / "m1" / "result.json").write_text(json.dumps({"recognition": summary}, default=float))
    pairs = RC.recompute(tmp_path / "m1")
    assert len(pairs) == 1 and RC.differences(*pairs[0]) == []


# --- Codex implementation round 1 ------------------------------------------------------------------------------------
def test_a_decision_applied_after_the_wait_limit_is_a_timeout():
    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    gate = _Gate(run.compute)
    clock = _Clock()
    live = R.LiveFar(run, worker=R.FarWorker(gate), clock=clock)
    try:
        handle = live.stop(1, **_stop_kw(xs, 1.0))
        clock.t += 29.99
        assert live.stop_state(handle) == "waiting"
        clock.t += 0.11
        gate.go.set()
        _wait_idle(live)
        live.pump(1, active=True)  # decided at 30.1 s
        with pytest.raises(RuntimeError, match="^recognition_timeout$"):
            live.stop_state(handle)
    finally:
        live.close()


def test_a_decision_applied_in_time_is_not_a_timeout_when_read_a_tick_later():
    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    clock = _Clock()
    live = R.LiveFar(run, clock=clock)
    try:
        handle = live.stop(1, **_stop_kw(xs, 1.0))
        _wait_idle(live)
        clock.t += 29.9
        live.pump(1, active=True)
        clock.t += 0.5  # read on the next tick, past the limit: decided in time
        assert live.stop_state(handle) == "done"
    finally:
        live.close()


def test_a_missions_end_accounts_for_every_job_and_owns_only_its_own_record(tmp_path):
    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    gate = _Gate(run.compute)
    live = R.LiveFar(run, worker=R.FarWorker(gate))
    try:
        live.start_mission("m1", zone_centre_xy=G.PALLET[:2], zone_radius_m=0.6, out_dir=tmp_path / "m1")
        _drive_live(live, xs, pump=False, ticks=range(80))  # frames at 24, 48, 72 handed over
        assert gate.started.wait(10)
        assert live.end_mission("m2") is None  # not the mission this record belongs to
        live.start_mission("m3", zone_centre_xy=G.PALLET[:2], zone_radius_m=0.6, out_dir=tmp_path / "m3")
        _drive_live(live, xs, pump=False, ticks=range(80))  # m1's frame 24 still in the worker
        summary = live.end_mission("m3")
        assert live.end_mission("m3") is None  # ended once
    finally:
        gate.go.set()
        live.close()
    jobs = summary["live"]["jobs"]
    assert len(jobs) == 3 and {j["status"] for j in jobs} <= {"dropped", "in_flight"}
    assert all(j["why"] == "mission_end" for j in jobs)


def test_write_errors_after_the_end_reach_the_missions_record(tmp_path):
    import json

    pending = []

    class _IO:
        def put_line(self, job, nbytes):
            pending.append(job)
            return True

    xs = G._leg()
    run = G._runtime(R, lag_steps=11)
    live = R.LiveFar(run, io=_IO())
    try:
        live.start_mission("m1", zone_centre_xy=G.PALLET[:2], zone_radius_m=0.6,
                           out_dir=tmp_path / "blocked" / "recognition")
        (tmp_path / "blocked").write_text("a file where the directory should be")
        _drive_live(live, xs, ticks=range(30))
        _wait_idle(live)
        live.pump(1, active=True)
        summary = live.end_mission("m1")
        live.write_json(tmp_path / "result.json", {"recognition": summary})
        assert summary["live"]["record_errors"] == []  # nothing written yet
        for job in pending:  # the writer reaches them in order: the frame, then the record
            job()
    finally:
        live.close()
    assert summary["live"]["record_errors"] and "frame_0001.npz" in summary["live"]["record_errors"][0]
    assert json.loads((tmp_path / "result.json").read_text())["recognition"]["live"]["record_errors"]


def test_the_runner_takes_nothing_in_on_a_stopped_step_and_keeps_the_metadata_a_session_contract():
    src = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert 'active=core.commands.status == "running" and not live.stopped_in_tick)' in src
    head = src[src.index("def far_drive_frame"):src.index("def read_() -> None:")]
    assert 'rendering_s_ = float(frame_["rendering_time"]) - initial_time' in head  # outside the read's boundary
    assert "far_live.close()" in src
