"""M-c4: the live runtime with a fake core and fake Isaac hooks (plan C2, C4)."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("websockets")

from forklift_core.mission.commands import MissionCommands  # noqa: E402
from forklift_core.mission.targets import Pose, TargetsBook  # noqa: E402
from sim.isaac.live_runtime import Hooks, LiveRuntime  # noqa: E402

DT = 1 / 120


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += max(0.0, s)


def runtime(tmp_path, **k):
    targets = TargetsBook()
    targets.add_pallet("P-1", "epal6", Pose(5.0, 0.0, 0.0))
    targets.add_destination("D-1", Pose(9.0, 0.0, 0.0))
    core = SimpleNamespace(commands=MissionCommands(targets), targets=targets)
    did = {"zero": 0, "lift": [], "lift_pos": 0.10, "render": None}
    hooks = Hooks(zero_wheels=lambda: did.__setitem__("zero", did["zero"] + 1),
                  hold_lift=lambda p: did["lift"].append(p), lift_position=lambda: did["lift_pos"],
                  camera=lambda: (np.zeros((4, 4, 3), np.uint8), did["render"]), grid=lambda: None, scan=lambda: None)
    clock = Clock()
    rt = LiveRuntime(core, hooks, output=tmp_path, port_file=tmp_path / "port.json", dt=DT, clock=clock,
                     sleep=clock.sleep, session_epoch=9, **k)
    return rt, core, did, clock


def step(rt, clock, sim_t, *, speed=0.0, rendered=False, cost=0.001):
    zero = rt.before_step(sim_t)
    clock.t += cost
    rt.after_step(sim_t + DT, odom_speed=speed, yaw_rate=0.0, rendered=rendered)
    return zero


def test_a_stop_zeroes_the_wheels_at_the_next_step_and_latches_the_lift_once(tmp_path):
    rt, core, did, clock = runtime(tmp_path)
    rt.start()
    try:
        assert step(rt, clock, 0.0) is False
        rt.arbiter.stop(clock(), "stop:test")
        did["lift_pos"] = 0.10
        assert step(rt, clock, DT) is True and rt.stopped_in_tick
        did["lift_pos"] = 0.0972  # the load sags: the latch does not follow it
        step(rt, clock, 2 * DT)
        assert did["zero"] == 2 and did["lift"] == [0.10, 0.10]
    finally:
        rt.close()
    assert json.loads((tmp_path / "port.json").read_text())["state"] == "ended"


def test_a_frame_is_one_per_render_time_and_its_latency_counts_from_the_step_start(tmp_path):
    rt, core, did, clock = runtime(tmp_path)
    rt.start()
    try:
        did["render"] = 0.5
        rt.step_start.append((0.5, clock.t - 0.03))  # the step that drew sim 0.5 started 30 ms ago
        step(rt, clock, 0.5 + DT, rendered=True)
        first = rt.store.get("camera_rgb")
        step(rt, clock, 0.5 + 2 * DT, rendered=True)  # the same render time again: no new frame
        assert rt.store.get("camera_rgb")[0] == first[0] == 1 and rt.frame_id == 1
        assert rt.camera_latency[0] == pytest.approx(0.03 + 0.001, abs=1e-6)
    finally:
        rt.close()


def test_feedback_is_stamped_at_the_step_that_made_it(tmp_path):
    rt, core, did, clock = runtime(tmp_path)
    rt.start()
    try:
        step(rt, clock, 0.0, speed=0.2)
        stamp = rt.feedback.stamp_mono
        clock.t += 0.5  # a synchronous call: no step
        rt.tick_start(0.01, False)
        assert rt.feedback.stamp_mono == stamp and not rt.arbiter.feedback_ok  # stale, not re-stamped
    finally:
        rt.close()


def test_the_pacer_holds_the_declared_factor_through_the_runtime(tmp_path):
    rt, core, did, clock = runtime(tmp_path, factor=0.5)
    rt.start()
    try:
        start = clock.t
        for k in range(120):
            step(rt, clock, k * DT)
        assert clock.t - start == pytest.approx(119 * 2 * DT + 0.001, rel=0.01)
    finally:
        rt.close()
