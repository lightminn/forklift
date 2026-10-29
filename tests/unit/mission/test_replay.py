"""Replaying a format-2 boundary record (docs/plans/2026-10-16-m-a1-replay.md, steps R1/R4): the same code
run against the record gets the recorded answers, argument changes, watched changes and exceptions, and any
difference in the calls it makes fails at that call."""

import gzip
import json
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
from forklift_core.mission.replay import ReplayGate, ReplayMismatch, ReplayProxy, ReplaySession, read_ticks
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


# --- Codex R1 re-review (P2 x5) ---------------------------------------------------------------------------

def test_an_object_the_call_replaced_is_replaced_in_the_replay_too(tmp_path):
    def swap(box):
        box["items"] = []  # an equal value, another object

    def call(f):
        box = {"items": []}
        old = box["items"]
        f(box)
        return box["items"] is old

    replay_with = one_call(tmp_path, swap, call)
    assert replay_with(call) is False

    core = {"box": {"items": []}}
    inner = core["box"]["items"]

    def swap_inner(_):
        core["box"]["items"] = []  # inside a watched value, an equal value, another object

    replay_with = one_call(tmp_path / "w", swap_inner, lambda f: f(0), watch_dicts={"core": core})
    assert core["box"]["items"] is not inner
    rcore = {"box": {"items": []}}
    rbox, rinner = rcore["box"], rcore["box"]["items"]
    replay_with(lambda f: f(0), watch_dicts_replay={"core": rcore})
    assert rcore["box"] is rbox and rcore["box"]["items"] is not rinner


@pytest.mark.parametrize("change", ["merge_args", "merge_watched", "fresh_twice"])
def test_a_change_outside_the_change_model_is_recorded_as_such_and_refused(tmp_path, change):
    core = {"a": [1], "b": [2]}

    def fn(box):
        if change == "merge_args":
            box["b"] = box["a"]
        elif change == "merge_watched":
            core["b"] = core["a"]
        else:
            core["a"] = core["b"] = [9]

    replay_with = one_call(tmp_path, fn, lambda f: f({"a": [1], "b": [1]}), watch_dicts={"core": core})
    with pytest.raises(ReplayMismatch, match="outside the replay's model"):
        replay_with(lambda f: f({"a": [1], "b": [1]}), watch_dicts_replay={"core": {"a": [1], "b": [2]}})


def test_an_unchanged_read_only_array_beside_a_changed_argument_is_left_alone(tmp_path):
    frozen = np.zeros(2)
    frozen.setflags(write=False)

    def fill(arr, box):
        box.append(1)

    def call(f):
        box = []
        f(frozen, box)
        return box

    replay_with = one_call(tmp_path, fill, call)
    assert replay_with(call) == [1]


def test_a_failing_exception_restoration_the_code_catches_still_fails_the_replay(tmp_path):
    def boom():
        raise Refused("x")

    replay_with = one_call(tmp_path, boom, lambda f: f())
    path = next(tmp_path.glob("boundary_*.jsonl"))
    text = path.read_text()
    path.write_text(text.replace('"args":{"$tuple":["x"]}', '"args":{"$ref":9}'))

    def sloppy(f):
        try:
            f()
        except Exception:
            pass

    with pytest.raises(ReplayMismatch, match="restoring the recorded exception failed"):
        replay_with(sloppy)


def test_a_registered_value_objects_attributes_are_restored(tmp_path):
    def bump(action):
        action.joint = 7

    def call(f):
        act = Action(1)
        f(act)
        return act.joint

    replay_with = one_call(tmp_path, bump, call, value_types={"Action"})
    assert replay_with(call) == 7


def test_a_response_sharing_a_value_objects_inner_list_is_refused(tmp_path):
    replay_with = one_call(tmp_path, lambda act: act.joint, lambda f: f(Action([1])), value_types={"Action"})
    with pytest.raises(ReplayMismatch, match="argument's own object"):
        replay_with(lambda f: f(Action([1])))


def test_a_restoration_error_the_code_catches_still_fails_the_replay(tmp_path):
    rec = BoundaryRecorder(tmp_path, value_format=2)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rec.call("Svc", "f", request={"args": [], "kwargs": {}}, response=None)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    path = next(tmp_path.glob("boundary_*.jsonl"))
    path.write_text(path.read_text().replace('"response":null', '"response":{"$ref":5}'))  # an unresolvable answer
    session = ReplaySession(tmp_path)
    gate = ReplayGate(session)
    f = recorded_function(lambda: None, "Svc", "f", gate)
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    try:
        f()
    except Exception:
        pass  # caught by the code under test
    with pytest.raises(ReplayMismatch, match="restoring the recorded response failed"):
        session.tick_end(outputs={}, branches=[])
        session.finish()


# --- Codex R1 re-review 3 (P2 x7): the change model, checked over one view per call ------------------------

# An edit to an already-logged item of an append-only list is not checked per call (it would re-encode the
# whole log every physics step); the end-of-run final state catches it (test below).
@pytest.mark.parametrize("change", ["old_child_in_new_parent", "pop_then_mutate", "arg_takes_watched",
                                    "watched_takes_other_watched", "append_old_object", "object_key"])
def test_out_of_model_changes_are_refused(tmp_path, change):
    core = {"anchor": [5], "log": [{"n": 1}]}
    other = {"x": [6]}

    @dataclass(eq=False)
    class Key:
        n: int

    def fn(box):
        if change == "old_child_in_new_parent":
            box["p"] = {"child": box["p"]["child"]}
        elif change == "pop_then_mutate":
            box.pop("items").append(2)
        elif change == "arg_takes_watched":
            box["x"] = core["anchor"]
        elif change == "watched_takes_other_watched":
            core["anchor"] = other["x"]
        elif change == "append_edits_prefix":
            core["log"][0]["n"] = 9
            core["log"].append({"n": 2})
        elif change == "append_old_object":
            core["log"].append(core["anchor"])
        else:
            box["items"].append(1)

    def call(f):
        box = {"p": {"child": [1]}, "items": [1]}
        if change == "object_key":
            box[Key(1)] = 1
        f(box)

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, {"core": core, "other": other}, append_only={"log"})
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    call(recorded_function(fn, "Svc", "f", gate, watch=watch))
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    entry = next(iter(__import__("forklift_core.mission.replay", fromlist=["read_ticks"]).read_ticks(tmp_path)))
    assert "unsupported" in entry["calls"][0]["changes"], change


def test_a_popped_object_left_unchanged_is_fine(tmp_path):
    def fn(box):
        box.pop("items")

    def call(f):
        box = {"items": [1], "keep": 2}
        held = box["items"]
        f(box)
        return box, held

    replay_with = one_call(tmp_path, fn, call)
    box, held = replay_with(call)
    assert box == {"keep": 2} and held == [1]


def test_a_tuple_value_change_installs_a_new_tuple(tmp_path):
    core = {"odom_rear": (1.0, 2.0, 0.0), "pose": [(0.0, 0.0)]}

    def step(_):
        core["odom_rear"] = (1.1, 2.0, 0.0)
        core["pose"][0] = (0.5, 0.0)

    replay_with = one_call(tmp_path, step, lambda f: f(0), watch_dicts={"core": core})
    rcore = {"odom_rear": (1.0, 2.0, 0.0), "pose": [(0.0, 0.0)]}
    rpose = rcore["pose"]
    replay_with(lambda f: f(0), watch_dicts_replay={"core": rcore})
    assert rcore["odom_rear"] == (1.1, 2.0, 0.0) and rcore["pose"] is rpose and rpose == [(0.5, 0.0)]


def test_array_bytes_and_read_only_state_are_restored_exactly(tmp_path):
    def flip(arr):
        arr[0] = -0.0
        arr.setflags(write=False)

    def call(f):
        arr = np.zeros(2)
        f(arr)
        return arr

    replay_with = one_call(tmp_path, flip, call)
    arr = replay_with(call)
    assert np.signbit(arr[0]) and not arr.flags.writeable


def test_the_final_state_catches_what_no_call_record_carries(tmp_path):
    rec = BoundaryRecorder(tmp_path, value_format=2)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rec.tick_end(outputs={}, branches=[])
    rec.close(final_state={"log": [{"n": 9}], "a": np.ones(2)})
    session = ReplaySession(tmp_path)
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    session.tick_end(outputs={}, branches=[])
    with pytest.raises(ReplayMismatch, match="final state differs"):
        session.finish(final_state={"log": [{"n": 1}], "a": np.ones(2)})
    session = ReplaySession(tmp_path)
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    session.tick_end(outputs={}, branches=[])
    session.finish(final_state={"log": [{"n": 9}], "a": np.ones(2)})


# --- Codex R1 re-review 4 (P2 x8) --------------------------------------------------------------------------

class Opaque:
    def __init__(self):
        self.n = 1


@pytest.mark.parametrize("change", ["arg_takes_logged_child", "new_log_takes_watched", "opaque_in_view",
                                    "edit_last_logged", "cycle", "float_key", "int_watched_key"])
def test_more_out_of_model_changes_are_refused(tmp_path, change):
    core = {"anchor": [5], "log": [{"child": [1]}], "obj": Opaque() if change == "opaque_in_view" else 0}
    if change == "int_watched_key":
        core[1] = [0]

    def fn(box):
        if change == "arg_takes_logged_child":
            box["x"] = core["log"][0]["child"]
        elif change == "new_log_takes_watched":
            core["newlog"] = core["anchor"]
        elif change == "opaque_in_view":
            core["obj"].n = 9
        elif change == "edit_last_logged":
            core["log"][-1]["child"].append(2)
        elif change == "cycle":
            box["items"].append(box["items"])
        elif change == "float_key":
            box[0.5] = 1
        else:
            core[1].append(1)

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, {"core": core}, append_only={"log", "newlog"})
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    recorded_function(fn, "Svc", "f", gate, watch=watch)({"items": [1]})
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    from forklift_core.mission.replay import read_ticks

    entry = next(iter(read_ticks(tmp_path)))
    assert "unsupported" in (entry["calls"][0].get("changes") or {}), change


def test_a_list_replaced_by_none_is_restored(tmp_path):
    def fn(box):
        box["items"] = None

    def call(f):
        box = {"items": [1]}
        held = box["items"]
        f(box)
        return box, held

    replay_with = one_call(tmp_path, fn, call)
    box, held = replay_with(call)
    assert box == {"items": None} and held == [1]


def test_a_popped_array_made_read_only_is_refused(tmp_path):
    def fn(box):
        box.pop("a").setflags(write=False)

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    recorded_function(fn, "Svc", "f", gate)({"a": np.zeros(2)})
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    from forklift_core.mission.replay import read_ticks

    assert "unsupported" in next(iter(read_ticks(tmp_path)))["calls"][0]["changes"]


def test_a_missing_final_state_file_and_nan_arrays(tmp_path):
    rec = BoundaryRecorder(tmp_path, value_format=2)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rec.tick_end(outputs={}, branches=[])
    rec.close(final_state={"a": np.array([np.nan])})

    def run(final):
        session = ReplaySession(tmp_path)
        session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
        session.tick_end(outputs={}, branches=[])
        session.finish(final_state=final)

    run({"a": np.array([np.nan])})  # the same NaN array passes
    (tmp_path / "final_state.json").unlink()
    with pytest.raises(ReplayMismatch, match="final_state.json is missing"):
        run({"a": np.array([1.0])})


# --- Codex R1 re-review 5 (P2 x4) --------------------------------------------------------------------------

def test_a_cached_bundle_shared_by_two_log_rows_replays(tmp_path):
    """grid_kwargs appends {kind, **bundle} for a cached bundle: two rows share bundle["applied"]."""
    obstacle = {"plans": []}
    bundle = {"applied": [0.1, 0.0, 0.0], "mode": "tracking"}

    def grid(kind):
        obstacle["plans"].append({"kind": kind, **bundle})
        return 1

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, {"obstacle": obstacle}, keys={"obstacle": ["plans"]}, append_only={"plans"})
    f = recorded_function(grid, "Planner", "grid_kwargs", gate, watch=watch)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    f("mission")
    f("mission")
    rec.tick_end(outputs={}, branches=[])
    rec.close(final_state=obstacle)
    from forklift_core.mission.replay import read_ticks

    assert all("unsupported" not in (c.get("changes") or {}) for c in next(iter(read_ticks(tmp_path)))["calls"])
    session = ReplaySession(tmp_path)
    rob = {"plans": []}
    rwatch = DictWatch(None, {"obstacle": rob}, keys={"obstacle": ["plans"]}, append_only={"plans"})
    rf = recorded_function(lambda k: None, "Planner", "grid_kwargs", ReplayGate(session), watch=rwatch)
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    assert rf("mission") == 1 and rf("mission") == 1
    session.tick_end(outputs={}, branches=[])
    session.finish(final_state=rob)  # compared by value: the rows' shared list is not an identity claim
    assert rob["plans"] == obstacle["plans"]


@pytest.mark.parametrize("change", ["set_in_view", "opaque_logged", "new_lists_alias"])
def test_unknown_kinds_opaque_log_items_and_aliased_new_logs_are_refused(tmp_path, change):
    core = {"tags": {1}, "log": []}

    def fn(_):
        if change == "set_in_view":
            core["tags"].add(2)
        elif change == "opaque_logged":
            core["log"].append(Opaque())
        else:
            core["a"] = core["b"] = []

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, {"core": core}, append_only={"log", "a", "b"})
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    recorded_function(fn, "Svc", "f", gate, watch=watch)(0)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    from forklift_core.mission.replay import read_ticks

    assert "unsupported" in (next(iter(read_ticks(tmp_path)))["calls"][0].get("changes") or {}), change


# --- Codex R1 re-review 6 (P2 x3) --------------------------------------------------------------------------

def test_a_new_log_on_an_empty_watched_dict_records_and_replays(tmp_path):
    core = {}

    def fn(_):
        core["log"] = [{"n": 1}]

    replay_with = one_call(tmp_path, fn, lambda f: f(0), watch_dicts={"core": core})
    rcore = {}
    replay_with(lambda f: f(0), watch_dicts_replay={"core": rcore})
    assert rcore == {"log": [{"n": 1}]}


def test_bytes_are_values(tmp_path):
    core = {"payload": b"old"}

    def fn(_):
        core["payload"] = b"new"

    replay_with = one_call(tmp_path, fn, lambda f: f(0), watch_dicts={"core": core})
    rcore = {"payload": b"old"}
    replay_with(lambda f: f(0), watch_dicts_replay={"core": rcore})
    assert rcore["payload"] == b"new"


def test_a_numpy_scalar_without_a_value_encoding_in_the_view_is_refused(tmp_path):
    core = {"z": np.void(b"ab")}

    def fn(_):
        core["z"] = np.void(b"cd")

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, {"core": core})
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    recorded_function(fn, "Svc", "f", gate, watch=watch)(0)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    from forklift_core.mission.replay import read_ticks

    assert "unsupported" in next(iter(read_ticks(tmp_path)))["calls"][0]["changes"]


def test_a_frozen_value_shared_between_an_argument_and_a_watched_key_is_fine(tmp_path):
    """grid_kwargs stores the pose it was given as the pickup estimate: a frozen Pose2D, harmless to share."""
    core = {"pickup_estimate": None}

    def fn(target):
        core["pickup_estimate"] = target

    replay_with = one_call(tmp_path, fn, lambda f: f(Pose2D(1.0, 2.0, 0.1)), watch_dicts={"core": core})
    rcore = {"pickup_estimate": None}
    replay_with(lambda f: f(Pose2D(1.0, 2.0, 0.1)), watch_dicts_replay={"core": rcore})
    assert rcore["pickup_estimate"] == Pose2D(1.0, 2.0, 0.1)


def test_a_frozen_value_replaced_by_another_is_assigned(tmp_path):
    core = {"pickup_estimate": Pose2D(0.0, 0.0, 0.0)}

    def fn(target):
        core["pickup_estimate"] = target

    replay_with = one_call(tmp_path, fn, lambda f: f(Pose2D(1.0, 2.0, 0.1)), watch_dicts={"core": core})
    rcore = {"pickup_estimate": Pose2D(0.0, 0.0, 0.0)}
    replay_with(lambda f: f(Pose2D(1.0, 2.0, 0.1)), watch_dicts_replay={"core": rcore})
    assert rcore["pickup_estimate"] == Pose2D(1.0, 2.0, 0.1)


@dataclass(frozen=True)
class FrozenHolder:
    pose: Pose2D
    items: list


def test_an_unchanged_frozen_field_inside_a_changed_argument_is_left_alone(tmp_path):
    def fn(holder):
        holder.items.append(1)

    def call(f):
        holder = FrozenHolder(Pose2D(1.0, 0.0, 0.0), [])
        pose = holder.pose
        f(holder)
        return holder, pose

    replay_with = one_call(tmp_path, fn, call)
    holder, pose = replay_with(call)
    assert holder.items == [1] and holder.pose is pose


# --- Codex R3/R4 review (P2 x7, P3 x2) ---------------------------------------------------------------------

def test_a_tick_whose_head_times_were_deferred_must_set_them(tmp_path):
    rec = BoundaryRecorder(tmp_path, value_format=2)
    rec.tick_begin(0, tick_t=None, work_t=None, commands=[], stop_kind="none")
    rec.tick_time(tick_t=1.0, work_t=1.0)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    session = ReplaySession(tmp_path)
    session.tick_begin(0, tick_t=None, work_t=None, commands=[], stop_kind="none")
    with pytest.raises(ReplayMismatch, match="head times were never set"):
        session.tick_end(outputs={}, branches=[])
    session = ReplaySession(tmp_path)
    session.tick_begin(0, tick_t=None, work_t=None, commands=[], stop_kind="none")
    with pytest.raises(ReplayMismatch, match="tick_t"):
        session.tick_time(tick_t=2.0, work_t=2.0)


class Tagged(int):
    """An int carrying state of its own (not a plain value)."""


def test_a_scalar_subclass_with_state_is_not_a_plain_value(tmp_path):
    from forklift_core.mission.replay_codec import immutable, plain_value

    tagged = Tagged(3)
    tagged.items = []
    assert not plain_value(tagged) and not immutable(tagged) and plain_value(3) and immutable((1, Pose2D(0, 0, 0)))

    def fn(x):
        x.items.append(1)

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    recorded_function(fn, "Svc", "f", gate)(tagged)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    from forklift_core.mission.replay import read_ticks

    assert "unsupported" in next(iter(read_ticks(tmp_path)))["calls"][0]["changes"]


def test_a_frozen_value_shared_by_an_argument_and_a_watched_key_replays_as_values(tmp_path):
    core = {"pose": None}
    p = Pose2D(1.0, 2.0, 0.0)

    def store(x):
        core["pose"] = x

    def use(a, b):
        return a == b

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, {"core": core})
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    recorded_function(store, "Svc", "store", gate, watch=watch)(p)
    recorded_function(use, "Svc", "use", gate)(p, core["pose"])
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    session = ReplaySession(tmp_path)
    rgate = ReplayGate(session)
    rcore = {"pose": None}
    rwatch = DictWatch(None, {"core": rcore})
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    q = Pose2D(1.0, 2.0, 0.0)
    recorded_function(lambda x: None, "Svc", "store", rgate, watch=rwatch)(q)
    assert recorded_function(lambda a, b: None, "Svc", "use", rgate)(q, rcore["pose"]) is True
    session.tick_end(outputs={}, branches=[])
    session.finish()


def test_a_service_a_call_hands_back_is_recorded_and_replayed_as_its_own_port(tmp_path):
    class Controller:
        def __init__(self):
            self.kps = np.array([1.0, 2.0])

        def get_gains(self):
            return self.kps.copy()

        def set_gains(self, kps):
            self.kps = np.asarray(kps, dtype=float)

    class Robot:
        def __init__(self):
            self.controller = Controller()

        def get_articulation_controller(self):
            return self.controller

    def use(robot, factor):
        c = robot.get_articulation_controller()
        gains = c.get_gains()
        c.set_gains(kps=gains * factor)
        return robot.get_articulation_controller().get_gains()

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    robot = RecordingProxy(Robot(), "Vehicle", gate, returns={"get_articulation_controller": "Controller"})
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    assert np.array_equal(use(robot, 2.0), [2.0, 4.0])
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    assert rec.opaque_values == 0

    def replay(factor):
        session = ReplaySession(tmp_path)
        proxy = ReplayProxy("Vehicle", ReplayGate(session), spec=Robot,
                            returns={"get_articulation_controller": "Controller"})
        session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
        out = use(proxy, factor)
        session.tick_end(outputs={}, branches=[])
        session.finish()
        return out

    assert np.array_equal(replay(2.0), [2.0, 4.0])
    with pytest.raises(ReplayMismatch, match="Controller.set_gains"):
        replay(3.0)
    # Without the handle contract the replay cannot stand in for the controller.
    session = ReplaySession(tmp_path)
    proxy = ReplayProxy("Vehicle", ReplayGate(session), spec=Robot)
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    with pytest.raises((ReplayMismatch, AttributeError)):
        use(proxy, 2.0)


def test_a_memory_child_port_records_its_reads_and_calls(tmp_path):
    class Memory:
        def __init__(self):
            self.revision = 3

        def set_slam(self, index):
            self.revision += index

    class Layer:
        def __init__(self):
            self.memory = Memory()

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    layer = RecordingProxy(Layer(), "Obstacles", gate, child_ports={"memory": "Memory"})
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    assert layer.memory is not None
    layer.memory.set_slam(2)
    assert layer.memory.revision == 5
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    assert rec.opaque_values == 0
    session = ReplaySession(tmp_path)
    proxy = ReplayProxy("Obstacles", ReplayGate(session), spec=Layer, child_ports={"memory": "Memory"},
                        child_specs={"memory": Memory})
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    assert proxy.memory is not None
    proxy.memory.set_slam(2)
    assert proxy.memory.revision == 5
    session.tick_end(outputs={}, branches=[])
    session.finish()


def test_a_callback_descriptor_is_recorded_before_the_call_and_compared(tmp_path):
    from forklift_core.mission.boundary_record import recorded_function as rf

    def redraw(goal, *, dry_runs):
        return [run(goal) for run in dry_runs]

    def dry(speed):
        run = lambda goal: goal * speed  # noqa: E731
        run.callback_spec = {"kind": "rollout", "speed": speed}
        return run

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    assert rf(redraw, "Planner", "redraw", gate)(2.0, dry_runs=(dry(0.2), dry(0.29))) == [0.4, 0.58]
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    calls = next(iter(read_ticks(tmp_path)))["calls"]
    assert [(c["port"], c["method"]) for c in calls] == [("Trace", "callback_spec"), ("Planner", "redraw")]

    def replay(speeds):
        session = ReplaySession(tmp_path)
        rgate = ReplayGate(session)
        session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
        out = rf(lambda *a, **k: None, "Planner", "redraw", rgate)(2.0, dry_runs=tuple(dry(s) for s in speeds))
        session.tick_end(outputs={}, branches=[])
        session.finish()
        return out

    assert replay((0.2, 0.29)) == [0.4, 0.58]
    # The same request by type (two callables), different settings inside them: refused at the descriptor.
    with pytest.raises(ReplayMismatch, match="Trace.callback_spec"):
        replay((0.3, 0.39))


def test_format_1_records_no_callback_descriptor(tmp_path):
    rec = BoundaryRecorder(tmp_path, value_format=1)
    gate = RecordingGate(rec)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    recorded_function(lambda *, check: 1, "Planner", "plan", gate)(check=lambda p: None)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    lines = []
    for f in sorted(tmp_path.glob("boundary_*.jsonl*")):
        with gzip.open(f, "rt") if f.suffix == ".gz" else open(f) as stream:  # closed: the suite runs -W error
            lines += [json.loads(line) for line in stream]
    assert [c["method"] for t in lines for c in t["calls"]] == ["plan"]


def test_a_tick_left_by_an_exception_before_its_times_were_known_replays(tmp_path):
    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    rec.tick_begin(0, tick_t=None, work_t=None, commands=[], stop_kind="none")
    recorded_function(lambda: 1.5, "Clock", "now", gate)()
    rec.tick_end(outputs={"exit": "failed"}, branches=[])  # the time computation raised: tick_time never ran
    rec.close()
    session = ReplaySession(tmp_path)
    session.tick_begin(0, tick_t=None, work_t=None, commands=[], stop_kind="none")
    recorded_function(lambda: None, "Clock", "now", ReplayGate(session))()
    session.tick_end(outputs={"exit": "failed"}, branches=[])
    session.finish()
    # A tick whose recorded head times were set still needs tick_time.
    rec_dir = tmp_path / "set"
    rec = BoundaryRecorder(rec_dir, value_format=2)
    rec.tick_begin(0, tick_t=None, work_t=None, commands=[], stop_kind="none")
    rec.tick_time(tick_t=0.5, work_t=0.5)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    session = ReplaySession(rec_dir)
    session.tick_begin(0, tick_t=None, work_t=None, commands=[], stop_kind="none")
    with pytest.raises(ReplayMismatch, match="never set"):
        session.tick_end(outputs={}, branches=[])


def test_an_append_to_a_deque_argument_is_recorded_and_replayed(tmp_path):
    from collections import deque

    def push(history, x):
        history.append(x)
        return len(history)

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    h = deque([1.0], maxlen=3)
    assert recorded_function(push, "Svc", "push", gate)(h, 2.0) == 2
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    assert "unsupported" not in json.dumps(next(iter(read_ticks(tmp_path)))["calls"][0].get("changes"))
    session = ReplaySession(tmp_path)
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    r = deque([1.0], maxlen=3)
    assert recorded_function(lambda *a: None, "Svc", "push", ReplayGate(session))(r, 2.0) == 2
    assert list(r) == [1.0, 2.0] and r.maxlen == 3
    session.tick_end(outputs={}, branches=[])
    session.finish()


def test_a_sink_effect_is_compared_by_path_and_payload_without_hiding_the_reads_inside_the_sink(tmp_path):
    from forklift_core.mission.boundary_record import sink_effect

    class Pallet:
        def pose(self):
            return (1.0, 2.0)

    def save(gate, pallet, rgb):
        sink_effect(gate, "save_capture", "perception_capture_1", {"rgb": rgb})
        return pallet.pose()  # the sink body's own read stays a recorded call

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    pallet = RecordingProxy(Pallet(), "SimTruth", gate)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    save(gate, pallet, np.zeros((2, 2), dtype=np.uint8))
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    calls = next(iter(read_ticks(tmp_path)))["calls"]
    assert [(c["port"], c["method"]) for c in calls] == [("Sink", "save_capture"), ("SimTruth", "pose")]

    def replay(rgb):
        session = ReplaySession(tmp_path)
        rgate = ReplayGate(session)
        session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
        out = save(rgate, ReplayProxy("SimTruth", rgate, spec=Pallet), rgb)
        session.tick_end(outputs={}, branches=[])
        session.finish()
        return out

    assert replay(np.zeros((2, 2), dtype=np.uint8)) == (1.0, 2.0)
    with pytest.raises(ReplayMismatch, match="Sink.save_capture request differs"):
        replay(np.ones((2, 2), dtype=np.uint8))  # the payload changed
    session = ReplaySession(tmp_path)
    rgate = ReplayGate(session)
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    with pytest.raises(ReplayMismatch, match="SimTruth.pose where Sink.save_capture"):
        ReplayProxy("SimTruth", rgate, spec=Pallet).pose()  # the file write left out


def test_one_descriptor_does_not_let_an_undescribed_positional_callback_through(tmp_path):
    def plan(weight_fn, *, check):
        return weight_fn(1.0) + check(0.0)

    def described(x):
        return x

    described.callback_spec = {"kind": "check"}
    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    recorded_function(plan, "Planner", "plan", gate)(lambda w: w * 1.3, check=described)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    calls = next(iter(read_ticks(tmp_path)))["calls"]
    assert "unspecified" in json.dumps(calls[0]["request"])  # the positional callback is listed, undescribed
    session = ReplaySession(tmp_path)
    rgate = ReplayGate(session)
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    with pytest.raises(ReplayMismatch, match="opaque request value"):
        recorded_function(lambda *a, **k: None, "Planner", "plan", rgate)(lambda w: w * 2.7, check=described)


def _baseline_stage_one(tmp_path, *, fail=False):
    """The baseline's stage-1 bundle: redraw_docking with described closures, then the rollouts trace (also when
    the Planner call raised)."""
    from forklift_core.mission.boundary_record import trace_entry

    def redraw_docking(grid, pose, *, dry_runs, max_yaw_rad):
        results = [run(pose) for run in dry_runs[:1]]  # the second dry run never ran (a refusal)
        if fail:
            raise Refused("no_plan")
        return ["PLAN", grid], {"yaw": max_yaw_rad, "runs": len(results)}

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rollouts = []

    def dry(speed):
        def run(pose):
            rollouts.append(np.array([pose, pose + speed]))
            return rollouts[-1]
        run.callback_spec = {"kind": "bicycle_rollout", "speed": speed, "start": [1.0]}
        return run

    try:
        recorded_function(redraw_docking, "Planner", "redraw_docking", gate)(
            "G", 1.0, dry_runs=(dry(0.2), dry(0.29)), max_yaw_rad=0.12)
    except Refused:
        pass
    finally:
        trace_entry(gate, "redraw_rollouts", list(rollouts))
    rec.tick_end(outputs={}, branches=[])
    rec.close()


def _moved_stage_one(tmp_path, speeds=(0.2, 0.29), yaw=0.12):
    session = ReplaySession(tmp_path)
    gate = ReplayGate(session)
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    traced = recorded_function(
        lambda *a, **k: None, "Planner", "redraw_docking_traced", gate,
        callback_spec=lambda a, k: {"dry_runs": [{"kind": "bicycle_rollout", "speed": s, "start": list(k["start_rear"])}
                                                 for s in k["dry_configs"]]})
    try:
        out = traced("G", 1.0, start_rear=(1.0,), dry_configs=speeds, max_yaw_rad=yaw)
    except Refused as exc:
        out = exc
    session.tick_end(outputs={}, branches=[])
    session.finish()
    return out


def test_the_baseline_stage_one_bundle_answers_the_moved_traced_redraw(tmp_path):
    _baseline_stage_one(tmp_path)
    plans, record, trajectories = _moved_stage_one(tmp_path)
    assert plans == ["PLAN", "G"] and record == {"yaw": 0.12, "runs": 1}
    assert len(trajectories) == 1 and np.array_equal(trajectories[0], [1.0, 1.2])
    with pytest.raises(ReplayMismatch, match="Trace.callback_spec request differs"):
        _moved_stage_one(tmp_path, speeds=(0.3, 0.39))  # the dry runs' settings changed
    with pytest.raises(ReplayMismatch, match="Planner.redraw_docking request differs"):
        _moved_stage_one(tmp_path, yaw=0.05)  # another Planner argument changed


def test_a_baseline_stage_one_exception_is_raised_after_its_rollouts_are_consumed(tmp_path):
    _baseline_stage_one(tmp_path, fail=True)
    out = _moved_stage_one(tmp_path)
    assert isinstance(out, Refused) and out.reason == "no_plan"


def test_an_unreadable_rollouts_trace_fails_also_on_the_exception_path(tmp_path):
    _baseline_stage_one(tmp_path, fail=True)
    chunk = next(tmp_path.glob("boundary_*.jsonl"))
    tick = json.loads(chunk.read_text().splitlines()[0])
    assert tick["calls"][-1]["method"] == "redraw_rollouts"
    tick["calls"][-1]["request"] = {"$opaque": "builtins:object"}
    chunk.write_text(json.dumps(tick) + "\n")
    with pytest.raises(ReplayMismatch, match="not a list of trajectories"):
        _moved_stage_one(tmp_path)


def test_a_rollouts_trace_with_extra_arguments_fails(tmp_path):
    _baseline_stage_one(tmp_path, fail=True)
    chunk = next(tmp_path.glob("boundary_*.jsonl"))
    tick = json.loads(chunk.read_text().splitlines()[0])
    tick["calls"][-1]["request"]["$map"]["args"]["$list"].append("extra")
    chunk.write_text(json.dumps(tick) + "\n")
    with pytest.raises(ReplayMismatch, match="up to two trajectories"):
        _moved_stage_one(tmp_path)


def test_a_set_member_with_an_identity_is_refused_not_misordered():
    from forklift_core.mission.replay_codec import Encoder

    @dataclass(frozen=True, eq=True)
    class Key:
        name: str

        def __hash__(self):
            return hash(self.name)

    assert Encoder().encode({("a", 1), ("b", 2)})["$set"] == [{"$tuple": ["a", 1]}, {"$tuple": ["b", 2]}]
    with pytest.raises(TypeError, match="plain value"):
        Encoder().encode({Key("a")})


def test_an_older_logged_item_replaced_in_place_is_caught_when_the_record_closes(tmp_path):
    log = {"records": [{"i": 0}, {"i": 1}, {"i": 2}]}

    def step(n):
        log["records"].append({"i": n})
        if n == 5:
            log["records"][0] = {"i": "replaced"}  # an old item, not the last one

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, {"log": log}, append_only={"records"})
    for n in range(3, 7):
        rec.tick_begin(n, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
        recorded_function(step, "Vehicle", "step", gate, watch=watch)(n)
        rec.tick_end(outputs={}, branches=[])
    rec.close()
    with pytest.raises(ValueError, match="changed in place"):
        ReplaySession(tmp_path)


def test_a_watched_call_on_a_long_log_costs_no_pass_over_the_log(tmp_path):
    import time

    log = {"records": [np.zeros(3) for _ in range(20000)]}
    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, {"log": log}, append_only={"records"})
    step = recorded_function(lambda: log["records"].append(np.zeros(3)), "Vehicle", "step", gate, watch=watch)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    step()  # the first call walks the existing items once
    start = time.perf_counter()
    for _ in range(300):
        step()
    per_call = (time.perf_counter() - start) / 300
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    assert per_call < 2e-3, per_call  # a pass over 20k items per call took several ms


# --- M-a3d (docs/plans/2026-10-16-m-a3-mission-commands.md E2, E7, E9) ---------------------------------------
def test_a_format_2_head_carries_the_commands_and_stop_kind_set_with_its_times(tmp_path):
    rec = BoundaryRecorder(tmp_path, value_format=2)
    rec.tick_begin(0, tick_t=None, work_t=None, commands=None, stop_kind=None)
    rec.tick_time(tick_t=1.0, work_t=0.5, commands=[{"command": "pause", "accepted": True}], stop_kind="preempt")
    rec.tick_end(outputs={}, branches=[])
    rec.close()

    def replay(commands, kind):
        session = ReplaySession(tmp_path)
        session.tick_begin(0, tick_t=None, work_t=None, commands=None, stop_kind=None)
        session.tick_time(tick_t=1.0, work_t=0.5, commands=commands, stop_kind=kind)
        session.tick_end(outputs={}, branches=[])
        session.finish()

    replay([{"command": "pause", "accepted": True}], "preempt")
    with pytest.raises(ReplayMismatch, match="commands"):
        replay([{"command": "resume", "accepted": True}], "preempt")
    with pytest.raises(ReplayMismatch, match="stop_kind"):
        replay([{"command": "pause", "accepted": True}], "none")


def test_a_physics_step_that_zeroes_the_command_is_restored_in_the_replay(tmp_path):
    """E9: a preempt inside a capture changes the applied command, its slew and the stop detector's command in the
    step itself; the watch records it and the replay, which does not step, restores it."""
    state = {"applied_command": (0.3, [0.1, 0.1]), "command_speed": 0.3}
    slam = {"last_command": 0.3}

    def step(_):
        state["applied_command"] = (0.0, [0.1, 0.1])
        state["command_speed"] = 0.0
        slam["last_command"] = 0.0

    replay_with = one_call(tmp_path, step, lambda f: f(1),
                           watch_dicts={"state": state, "slam": slam})
    state_r = {"applied_command": (0.3, [0.1, 0.1]), "command_speed": 0.3}
    slam_r = {"last_command": 0.3}
    replay_with(lambda f: f(1), watch_dicts_replay={"state": state_r, "slam": slam_r})
    assert state_r == {"applied_command": (0.0, [0.1, 0.1]), "command_speed": 0.0} and slam_r == {"last_command": 0.0}


def _video_record(tmp_path, recorded):
    """One tick whose Sink writes are ``recorded``: (path, payload) pairs."""
    from forklift_core.mission.boundary_record import sink_effect

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    for path, payload in recorded:
        sink_effect(gate, "write_text", path, payload)
    rec.tick_end(outputs={}, branches=[])
    rec.close()


def _replay_writes(tmp_path, writes, video_paths=frozenset()):
    from forklift_core.mission.boundary_record import sink_effect

    session = ReplaySession(tmp_path)
    session.video_sink_paths = frozenset(video_paths)
    rgate = ReplayGate(session)
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    for path, payload in writes:
        sink_effect(rgate, "write_text", path, payload)
    session.tick_end(outputs={}, branches=[])
    session.finish()
    return session


VIDEO = frozenset({"frame_audit.json", "plan_history.json"})


def test_a_video_records_derived_outputs_are_matched_by_path_alone_and_may_be_absent_in_the_replay(tmp_path):
    """2026-10-29 (riib scene records, video on): the video frames are made inside the physics step a replay does not
    run, so what they fill (the frame audit, the overlay's plan history) is empty there -- those outputs are matched by
    their path when the replay writes them and skipped when it does not; every other output is compared as ever."""
    _video_record(tmp_path, [("obstacle_scans.json", "s"), ("plan_history.json", "p"), ("frame_audit.json", "f1"),
                             ("meta.json", "m")])
    s = _replay_writes(tmp_path, [("obstacle_scans.json", "s"), ("frame_audit.json", "f-empty"), ("meta.json", "m")],
                       VIDEO)
    assert s.skipped_video_sinks == ["plan_history.json"]
    # a non-video output still compares its payload, and still must be there
    with pytest.raises(ReplayMismatch, match="request differs"):
        _replay_writes(tmp_path, [("obstacle_scans.json", "s2"), ("frame_audit.json", "f1"), ("meta.json", "m")], VIDEO)
    with pytest.raises(ReplayMismatch):
        _replay_writes(tmp_path, [("obstacle_scans.json", "s"), ("frame_audit.json", "f1")], VIDEO)


def test_without_video_the_derived_outputs_compare_as_ever(tmp_path):
    _video_record(tmp_path, [("plan_history.json", "p"), ("frame_audit.json", "f1")])
    _replay_writes(tmp_path, [("plan_history.json", "p"), ("frame_audit.json", "f1")])
    with pytest.raises(ReplayMismatch):
        _replay_writes(tmp_path, [("frame_audit.json", "f-empty")])


def test_a_trailing_unwritten_video_output_is_skipped_at_the_tick_end(tmp_path):
    _video_record(tmp_path, [("meta.json", "m"), ("plan_history.json", "p")])
    s = _replay_writes(tmp_path, [("meta.json", "m")], VIDEO)
    assert s.skipped_video_sinks == ["plan_history.json"]


def test_a_video_runs_frame_count_is_left_out_of_the_final_state_and_only_that(tmp_path):
    """2026-10-29: the video frames a run counts are taken inside the physics step a replay does not run."""
    def final(frames, other):
        return {"containers": {"state": {"frames": frames, "result": other}}}

    rec = BoundaryRecorder(tmp_path, value_format=2)
    rec.close(final_state=final(120, "a"))

    def finish(state, video):
        session = ReplaySession(tmp_path)
        session.video_sink_paths = frozenset({"frame_audit.json"}) if video else frozenset()
        session.finish(final_state=state)

    finish(final(0, "a"), video=True)
    with pytest.raises(ReplayMismatch, match="final state differs"):
        finish(final(0, "b"), video=True)  # every other key still compared
    with pytest.raises(ReplayMismatch, match="final state differs"):
        finish(final(0, "a"), video=False)  # without video the count is compared as ever
    finish(final(120, "a"), video=False)
