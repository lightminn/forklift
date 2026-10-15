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
    outs = drive(gate, ReplayProxy("Vehicle", gate, spec=Vehicle), recorded_function(plan_not_run, "Planner", "plan", gate),
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


# --- Codex R1 review (P2 x9, P3 x1): each hole closed ---------------------------------------------------

def one_call(tmp_path, fn_record, call, *, value_types=(), port="Svc", method="f", hash_only=False, watch_dicts=None,
             commands=()):
    """Record a single call in tick 0; return a function that replays ``call(gate_fn)`` against it."""
    rec = BoundaryRecorder(tmp_path, value_format=2)
    rec.value_types.update(value_types)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, watch_dicts) if watch_dicts is not None else None
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=list(commands), stop_kind="none")
    try:
        call(recorded_function(fn_record, port, method, gate, hash_only=hash_only, watch=watch))
    except Exception:
        pass
    rec.tick_end(outputs={}, branches=[])
    rec.close()

    def replay_with(call_replay, watch_dicts_replay=None, commands_replay=None):
        session = ReplaySession(tmp_path)
        rgate = ReplayGate(session)
        rwatch = DictWatch(None, watch_dicts_replay) if watch_dicts_replay is not None else None
        session.tick_begin(0, tick_t=0.0, work_t=0.0,
                           commands=list(commands if commands_replay is None else commands_replay), stop_kind="none")
        out = call_replay(recorded_function(lambda *a, **k: None, port, method, rgate, hash_only=hash_only,
                                            watch=rwatch))
        session.tick_end(outputs={}, branches=[])
        session.finish()
        return out
    return replay_with


def test_a_different_alias_graph_in_a_request_is_a_mismatch(tmp_path):
    a, b = [1], [1]
    replay_with = one_call(tmp_path, lambda d: d["c"] is d["a"], lambda f: f({"a": a, "b": b, "c": a}))
    assert replay_with(lambda f: f({"a": a, "b": b, "c": a})) is True
    with pytest.raises(ReplayMismatch, match="request differs"):
        replay_with(lambda f: f({"a": a, "b": b, "c": b}))  # same values, c now aliases b
    with pytest.raises(ReplayMismatch, match="request differs"):
        replay_with(lambda f: f({"a": a, "b": b, "c": [1]}))  # no alias at all


class Action:
    def __init__(self, joint):
        self.joint = joint


def test_registered_value_types_are_compared_by_their_fields(tmp_path):
    replay_with = one_call(tmp_path, lambda act: None, lambda f: f(Action(1)), value_types={"Action"})
    replay_with(lambda f: f(Action(1)))
    with pytest.raises(ReplayMismatch, match="request differs"):
        replay_with(lambda f: f(Action(99)))


def test_an_unregistered_object_in_a_request_is_refused_unless_allowed(tmp_path):
    class Thing:
        pass
    replay_with = one_call(tmp_path, lambda x: 1, lambda f: f(Thing()))
    with pytest.raises(ReplayMismatch, match="opaque request value"):
        replay_with(lambda f: f(Thing()))


def test_an_extra_attribute_read_fails_and_a_none_child_comes_back_none(tmp_path):
    class Layer:
        def __init__(self):
            self.mode = "on"
            self.permission = None

        def limit(self):
            return 0.3

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    layer = RecordingProxy(Layer(), "Obstacles", gate, child_ports={"permission": "Permission"})
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    assert layer.permission is None and layer.limit() == 0.3
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    session = ReplaySession(tmp_path)
    proxy = ReplayProxy("Obstacles", ReplayGate(session), spec=Layer, child_ports={"permission": "Permission"})
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    with pytest.raises(ReplayMismatch, match="Obstacles.mode read where Obstacles.permission was recorded"):
        proxy.mode
    session = ReplaySession(tmp_path)
    proxy = ReplayProxy("Obstacles", ReplayGate(session), spec=Layer, child_ports={"permission": "Permission"})
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    assert proxy.permission is None and proxy.limit() == 0.3
    session.tick_end(outputs={}, branches=[])
    session.finish()


def test_a_mismatch_the_replayed_code_catches_still_fails_the_replay(tmp_path):
    replay_with = one_call(tmp_path, lambda x: x + 1, lambda f: f(1))

    def sloppy(f):
        try:
            f(2)  # wrong request, caught by the code under test
        except Exception:
            pass
        return f(1)

    with pytest.raises(ReplayMismatch):
        replay_with(sloppy)


def test_argument_and_watched_changes_keep_the_callers_inner_objects(tmp_path):
    def fill(box, pair):
        box["items"].append(7)
        pair[1].append("x")
        pair[0][0] = 5.0

    def call(f):
        items, inner, arr = [], [], np.zeros(2)
        box, pair = {"items": items}, (arr, inner)
        f(box, pair)
        return items, inner, arr

    replay_with = one_call(tmp_path, fill, call)
    items, inner, arr = replay_with(call)
    assert items == [7] and inner == ["x"] and arr[0] == 5.0  # the caller's own objects were changed

    core = {"held": [1, 2]}
    held = core["held"]

    def grow(_):
        core["held"].extend([3])

    replay_with = one_call(tmp_path / "w", grow, lambda f: f(0), watch_dicts={"core": core})
    rcore = {"held": [1, 2]}
    rheld = rcore["held"]
    replay_with(lambda f: f(0), watch_dicts_replay={"core": rcore})
    assert rcore["held"] is rheld and rheld == [1, 2, 3] and held == [1, 2, 3]


def test_a_response_that_hands_back_an_argument_object_is_refused(tmp_path):
    replay_with = one_call(tmp_path, lambda lst: lst, lambda f: f([1]))
    with pytest.raises(ReplayMismatch, match="argument's own object"):
        replay_with(lambda f: f([1]))


@pytest.mark.parametrize("exc", [RuntimeError("refused"), KeyError("k"),
                                 FileNotFoundError(2, "No such file", "/x/y")])
def test_exceptions_come_back_with_their_args(tmp_path, exc):
    def boom(_):
        raise exc

    replay_with = one_call(tmp_path, boom, lambda f: f(0))
    with pytest.raises(type(exc)) as got:
        replay_with(lambda f: f(0))
    assert got.value.args == exc.args and str(got.value) == str(exc)
    if isinstance(exc, FileNotFoundError):
        assert got.value.errno == 2 and got.value.filename == "/x/y"


def test_hash_only_requests_keep_every_type_but_the_array_payload(tmp_path):
    replay_with = one_call(tmp_path, lambda frame, ids: None, lambda f: f(np.ones(4), (1, 2)), hash_only=True)
    replay_with(lambda f: f(np.ones(4), (1, 2)))
    with pytest.raises(ReplayMismatch, match="request differs"):
        replay_with(lambda f: f(np.ones(4), [1, 2]))  # a list where a tuple was
    with pytest.raises(ReplayMismatch, match="request differs"):
        replay_with(lambda f: f(np.zeros(4), (1, 2)))  # another frame
    assert not list(tmp_path.glob("arrays_*.npz"))  # the frame itself was not stored


def test_capped_tampered_or_short_records_are_refused(tmp_path):
    rec = BoundaryRecorder(tmp_path / "cap", value_format=2, max_bytes=10)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rec.tick_end(outputs={"x": 1}, branches=[])
    rec.close()
    with pytest.raises(ValueError, match="stopped at tick 0"):
        ReplaySession(tmp_path / "cap")
    replay_with = one_call(tmp_path / "t", lambda a: a.sum(), lambda f: f(np.arange(3.0)))
    path = next((tmp_path / "t").glob("arrays_*.npz"))
    with np.load(path) as z:
        tampered = {k: z[k] + 1 for k in z.files}
    np.savez_compressed(path, **tampered)
    with pytest.raises(ValueError, match="does not match its hash"):
        replay_with(lambda f: f(np.arange(3.0)))


def test_the_tick_heads_commands_are_compared(tmp_path):
    replay_with = one_call(tmp_path, lambda x: x, lambda f: f(1), commands=["submit"])
    replay_with(lambda f: f(1))
    with pytest.raises(ReplayMismatch, match="commands"):
        replay_with(lambda f: f(1), commands_replay=["cancel"])
