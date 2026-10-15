"""Replaying a format-2 boundary record (docs/plans/2026-10-16-m-a1-replay.md, steps R1/R4): the same code
run against the record gets the recorded answers, argument changes, watched changes and exceptions, and any
difference in the calls it makes fails at that call."""

from dataclasses import dataclass

import numpy as np
import pytest

from forklift_core.mission.boundary_record import (
    BoundaryRecorder,
    DictWatch,
    RecordingGate,
    RecordingProxy,
    recorded_function,
)
from forklift_core.mission.replay import ReplayGate, ReplayMismatch, ReplayProxy, ReplaySession
from forklift_core.planning import Pose2D


@dataclass
class Reading:
    pose: tuple
    ranges: np.ndarray


class Refused(RuntimeError):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class Vehicle:
    """The live service in the recorded run."""

    def __init__(self):
        self.mode = "tracking"
        self.k = 0

    def read(self, step):
        self.k += 1
        return Reading((float(step), 0.5, 0.1), np.arange(3.0) + step)

    def capture(self, step):
        if step == 2:
            raise Refused("no_frame")
        return {"pose": Pose2D(step, 0.0, 0.0), "ids": (step, step + 1)}


def plan(trace, goal):
    """A planner that appends to the caller's trace list (the request_after case)."""
    trace.append(("searched", goal))
    return [goal, goal * 2]


def drive(gate, vehicle, plan_fn, step_fn, core, *, ticks=4, variant=None):
    """The replayed code: what a runner does around its boundary calls. ``variant`` injects a change."""
    outputs = []
    for step in range(ticks):
        if variant == "stop_early" and step == 3:
            return outputs  # the replayed run ends a tick before the recorded one did
        gate_rec = gate.recorder if hasattr(gate, "recorder") else gate.session
        gate_rec.tick_begin(step, tick_t=step / 120, work_t=step / 120, commands=[], stop_kind="none")
        reading = vehicle.read(step if variant != "request" or step != 1 else 9)
        mode = vehicle.mode
        trace = []
        path = plan_fn(trace, reading.pose[0])
        step_fn(step)
        try:
            shot = vehicle.capture(step)
            seen = shot["ids"]
        except Refused as exc:
            seen = ("refused", exc.reason)
        if variant == "extra" and step == 3:
            vehicle.read(step)
        out = {"x": reading.pose[0], "mode": mode, "trace": trace, "path": path, "seen": seen,
               "odom": core["odom"], "events": list(core["events"]), "sum": float(reading.ranges.sum())}
        if variant == "output" and step == 2:
            out["x"] = -1.0
        outputs.append(out)
        gate_rec.tick_end(outputs=out, branches=["state.events"] if step == 0 else [])
    return outputs


def record(tmp_path):
    rec = BoundaryRecorder(tmp_path, chunk_ticks=2, value_format=2)
    gate = RecordingGate(rec)
    core = {"odom": 0.0, "events": []}

    def step(i):
        core["odom"] = i * 0.1  # the physics step changes the core's dictionary
        if i == 0:
            core["events"].append({"event": "start"})

    watch = DictWatch(rec, {"core": core}, keys={"core": ["odom", "events"]}, append_only={"events"})
    vehicle = RecordingProxy(Vehicle(), "Vehicle", gate)
    outs = drive(gate, vehicle, recorded_function(plan, "Planner", "plan", gate),
                 recorded_function(step, "Vehicle", "step", gate, watch=watch), core)
    rec.close()
    return outs


def replay(tmp_path, variant=None, finish=True):
    session = ReplaySession(tmp_path)
    gate = ReplayGate(session)
    core = {"odom": 0.0, "events": []}

    def step_not_run(i):
        raise AssertionError("a replayed physics step must not run")

    def plan_not_run(trace, goal):
        raise AssertionError("a replayed planner must not run")

    watch = DictWatch(None, {"core": core}, keys={"core": ["odom", "events"]}, append_only={"events"})
    outs = drive(gate, ReplayProxy("Vehicle", gate), recorded_function(plan_not_run, "Planner", "plan", gate),
                 recorded_function(step_not_run, "Vehicle", "step", gate, watch=watch), core, variant=variant)
    if finish:
        session.finish()
    return outs


def test_the_same_code_replays_its_own_record_with_every_answer_and_effect(tmp_path):
    recorded = record(tmp_path)
    replayed = replay(tmp_path)
    assert len(replayed) == 4
    for a, b in zip(recorded, replayed):
        assert a["x"] == b["x"] and a["mode"] == b["mode"] and a["path"] == b["path"]
        assert b["trace"] == [("searched", a["x"])] and isinstance(b["trace"][0], tuple)  # request_after, typed
        assert a["seen"] == b["seen"] and a["odom"] == b["odom"] and a["events"] == b["events"]  # watched
        assert a["sum"] == b["sum"]
    assert replayed[2]["seen"] == ("refused", "no_frame")  # the exception re-raised with its attribute
    assert replayed[1]["seen"] == (1, 2) and isinstance(replayed[1]["seen"], tuple)


@pytest.mark.parametrize("variant, message", [
    ("request", "request differs"),
    ("extra", "was not recorded here"),
    ("output", "outputs differ"),
])
def test_a_different_call_or_output_fails_at_that_point(tmp_path, variant, message):
    record(tmp_path)
    with pytest.raises(ReplayMismatch, match=message):
        replay(tmp_path, variant=variant)


def test_calls_left_unmade_and_ticks_left_unreplayed_fail(tmp_path):
    record(tmp_path)
    with pytest.raises(ReplayMismatch, match="ticks left, from tick 3"):
        replay(tmp_path, variant="stop_early")
    session = ReplaySession(tmp_path)
    session.tick_begin(0, tick_t=0.0)
    with pytest.raises(ReplayMismatch, match="recorded calls not made"):
        session.tick_end(outputs={}, branches=["state.events"])
    session = ReplaySession(tmp_path)
    with pytest.raises(ReplayMismatch, match="next tick is 0"):
        session.tick_begin(1)


def test_a_format_1_record_is_not_replayed(tmp_path):
    rec = BoundaryRecorder(tmp_path)
    rec.close()
    with pytest.raises(ValueError, match="not a format-2"):
        ReplaySession(tmp_path)
