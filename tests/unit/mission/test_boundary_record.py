"""Boundary recorder for the M-a replay evidence (docs/plans/2026-10-15-m-a05-boundary.md section 5)."""

import json

import numpy as np
import pytest

from forklift_core.mission.boundary_record import BoundaryRecorder, read_record


def test_ticks_calls_and_outputs_round_trip(tmp_path):
    rec = BoundaryRecorder(tmp_path, chunk_ticks=2)
    for tick in range(3):
        rec.tick_begin(tick, tick_t=tick / 120, work_t=tick / 120, commands=[], stop_kind="none")
        rec.call("Slam", "mode", request={}, response="tracking")
        rec.call("Planner", "plan_transport", request={"goal": [1.0, 2.0, 0.5]},
                 response={"success": True, "path": np.arange(6, dtype=float).reshape(3, 2)},
                 changes={"installs": {"pickup_estimate": [1.0, 2.0, 0.5]}})
        rec.tick_end(outputs={"speed": 0.1 * tick, "phase": "approach"}, branches=["approach.track"])
    rec.close()
    ticks = list(read_record(tmp_path))
    assert [t["tick"] for t in ticks] == [0, 1, 2]
    call = ticks[1]["calls"][1]
    assert (call["port"], call["method"], call["seq"]) == ("Planner", "plan_transport", 1)
    assert np.array_equal(call["response"]["path"], np.arange(6, dtype=float).reshape(3, 2))
    assert call["changes"] == {"installs": {"pickup_estimate": [1.0, 2.0, 0.5]}}
    assert ticks[2]["outputs"] == {"speed": 0.2, "phase": "approach"}
    assert ticks[0]["branches"] == ["approach.track"]
    # chunks of two ticks were flushed as they filled
    assert sorted(p.name for p in tmp_path.glob("boundary_*.jsonl")) == ["boundary_00000.jsonl", "boundary_00001.jsonl"]


def test_arrays_are_stored_once_by_content_and_values_are_copied(tmp_path):
    rec = BoundaryRecorder(tmp_path, chunk_ticks=10)
    grid = np.zeros((240, 240), dtype=np.uint8)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rec.call("Obstacles", "snapshot", request={}, response={"state": grid})
    grid[0, 0] = 1  # the service changes its array after the call: the record keeps the old value
    rec.call("Obstacles", "snapshot", request={}, response={"state": grid})
    rec.call("Obstacles", "snapshot", request={}, response={"state": grid.copy()})  # same content again
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    (tick,) = list(read_record(tmp_path))
    first, second, third = (c["response"]["state"] for c in tick["calls"])
    assert first[0, 0] == 0 and second[0, 0] == 1 and np.array_equal(second, third)
    lines = (tmp_path / "boundary_00000.jsonl").read_text().splitlines()
    refs = [json.loads(lines[0])["calls"][i]["response"]["state"]["$array"] for i in range(3)]
    assert refs[1] == refs[2] and refs[0] != refs[1]
    assert len(np.load(tmp_path / "arrays_00000.npz").files) == 2


def test_exceptions_are_recorded_and_a_size_cap_stops_recording_not_the_run(tmp_path):
    rec = BoundaryRecorder(tmp_path, chunk_ticks=1, max_bytes=2_000)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rec.call("Planner", "plan_transport", request={}, exception=RuntimeError("stall_replan_failed"))
    rec.tick_end(outputs={}, branches=[])
    for tick in range(1, 5):
        rec.tick_begin(tick, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
        rec.call("Obstacles", "snapshot", request={}, response={"state": np.full(400, tick, dtype=np.float64)})
        rec.tick_end(outputs={}, branches=[])
    rec.close()
    ticks = list(read_record(tmp_path))
    assert ticks[0]["calls"][0]["exception"] == {"type": "RuntimeError", "message": "stall_replan_failed"}
    meta = json.loads((tmp_path / "boundary_meta.json").read_text())
    assert meta["stopped_at_tick"] is not None and meta["stopped_reason"] == "max_bytes"
    assert len(ticks) == meta["stopped_at_tick"]


def test_a_call_outside_a_tick_is_refused(tmp_path):
    rec = BoundaryRecorder(tmp_path)
    with pytest.raises(RuntimeError, match="outside a tick"):
        rec.call("Slam", "mode", request={}, response="tracking")


class _Layer:
    def __init__(self):
        self.version = 3
        self.calls = 0

    def limit(self, speed, path=None):
        self.calls += 1
        return min(speed, 0.2), {"reason": "clear", "grid": np.zeros(4)}

    def nested(self):
        return self.limit(1.0)


def _tick(rec, tick=0):
    rec.tick_begin(tick, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")


def test_a_proxy_records_calls_and_attribute_reads_and_returns_the_real_values(tmp_path):
    from forklift_core.mission.boundary_record import RecordingGate, RecordingProxy

    rec = BoundaryRecorder(tmp_path)
    gate = RecordingGate(rec)
    layer = _Layer()
    proxy = RecordingProxy(layer, "Obstacles", gate)
    _tick(rec)
    speed, info = proxy.limit(0.5, path=[1, 2])
    assert speed == 0.2 and info["reason"] == "clear" and layer.calls == 1
    assert proxy.version == 3
    proxy.version = 4  # a write goes to the real object and is recorded
    assert layer.version == 4
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    (tick,) = list(read_record(tmp_path))
    names = [(c["method"], c["request"]) for c in tick["calls"]]
    assert names[0] == ("limit", {"args": [0.5], "kwargs": {"path": [1, 2]}})
    assert names[1] == ("version", {"get": True})
    assert names[2] == ("version", {"set": 4})


def test_calls_inside_a_recorded_call_and_outside_ticks_are_not_recorded(tmp_path):
    from forklift_core.mission.boundary_record import RecordingGate, RecordingProxy, recorded_function

    rec = BoundaryRecorder(tmp_path)
    gate = RecordingGate(rec)
    layer = _Layer()
    proxy = RecordingProxy(layer, "Obstacles", gate)
    plan = recorded_function(lambda goal: proxy.limit(goal), "Planner", "plan", gate)  # nested service call
    proxy.limit(0.1)  # no tick open: not recorded, still executed
    _tick(rec)
    plan(0.3)
    with gate.suspended():  # e.g. physics stepping: service internals
        proxy.limit(0.2)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    (tick,) = list(read_record(tmp_path))
    assert [(c["port"], c["method"]) for c in tick["calls"]] == [("Planner", "plan")]
    assert layer.calls == 3


def test_hash_only_requests_keep_large_frames_out_of_the_record(tmp_path):
    from forklift_core.mission.boundary_record import RecordingGate, RecordingProxy

    class Tracker:
        def on_frame(self, scene):
            depth = getattr(scene, "depth_m", scene)
            return {"valid": bool(depth.mean() > 0)}

    rec = BoundaryRecorder(tmp_path)
    gate = RecordingGate(rec)
    proxy = RecordingProxy(Tracker(), "NearTracking", gate, hash_only_requests={"on_frame"})
    _tick(rec)
    proxy.on_frame(np.ones((480, 640), dtype=np.uint16))
    import dataclasses as dc

    @dc.dataclass
    class Scene:  # frames inside a dataclass (the near tracker's SceneInput) are hashed too
        depth_m: np.ndarray
        stamp_s: float

    proxy.on_frame(Scene(np.ones((480, 640), dtype=np.float32), 1.5))
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    (tick,) = list(read_record(tmp_path))
    ref = tick["calls"][0]["request"]["args"][0]
    assert set(ref) == {"$sha256", "dtype", "shape"} and ref["shape"] == [480, 640]
    scene = tick["calls"][1]["request"]["args"][0]
    assert scene["$dataclass"] == "Scene" and scene["stamp_s"] == 1.5 and "$sha256" in scene["depth_m"]
    assert not list(tmp_path.glob("arrays_*.npz"))


def test_exceptions_pass_through_and_are_recorded(tmp_path):
    from forklift_core.mission.boundary_record import RecordingGate, recorded_function

    def fail(x):
        raise RuntimeError("Tracking timeout")

    rec = BoundaryRecorder(tmp_path)
    gate = RecordingGate(rec)
    wrapped = recorded_function(fail, "Planner", "plan", gate)
    _tick(rec)
    with pytest.raises(RuntimeError, match="Tracking timeout"):
        wrapped(1)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    (tick,) = list(read_record(tmp_path))
    assert tick["calls"][0]["exception"]["message"] == "Tracking timeout"


def test_a_child_port_proxies_a_sub_service_held_as_an_attribute(tmp_path):
    from forklift_core.mission.boundary_record import RecordingGate, RecordingProxy

    class Permission:
        def allowed_speed(self, v):
            return v / 2

    class Layer:
        def __init__(self):
            self.permission = Permission()

    rec = BoundaryRecorder(tmp_path)
    gate = RecordingGate(rec)
    layer = Layer()
    proxy = RecordingProxy(layer, "Obstacles", gate, child_ports={"permission": "Permission"})
    _tick(rec)
    assert proxy.permission.allowed_speed(0.4) == 0.2
    assert proxy.permission is proxy.permission  # one proxy while the attribute holds the same object
    layer.permission = Permission()  # replaced: a new proxy follows the new object
    assert proxy.permission.allowed_speed(1.0) == 0.5
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    (tick,) = list(read_record(tmp_path))
    assert [(c["port"], c["method"]) for c in tick["calls"]] == [("Permission", "allowed_speed")] * 2
    assert json.loads((tmp_path / "boundary_meta.json").read_text())["opaque_values"] == 0


def test_the_request_is_recorded_as_it_was_before_the_call_and_changes_beside_it(tmp_path):
    from forklift_core.mission.boundary_record import RecordingGate, recorded_function

    def plan(goal, trace):
        trace.append({"rung": 8})  # plan_transport(trace=...) fills its argument
        return "ok"

    rec = BoundaryRecorder(tmp_path)
    gate = RecordingGate(rec)
    wrapped = recorded_function(plan, "Planner", "plan_transport", gate)
    _tick(rec)
    trace = []
    wrapped(1.0, trace=trace)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    (tick,) = list(read_record(tmp_path))
    call = tick["calls"][0]
    assert call["request"]["kwargs"]["trace"] == []
    assert call["changes"]["request_after"]["kwargs"]["trace"] == [{"rung": 8}]


def test_a_watch_records_what_a_service_call_changed_in_core_dictionaries(tmp_path):
    from forklift_core.mission.boundary_record import DictWatch, RecordingGate, recorded_function

    slam = {"stop_now": False, "tracker_speed": 0.2, "records": [{"scan": 1}], "tracker": object()}

    def step(render):
        slam["stop_now"] = True
        slam["records"].append({"scan": 2})

    rec = BoundaryRecorder(tmp_path)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, {"slam": slam}, append_only={"records"}, skip={"tracker"})
    wrapped = recorded_function(step, "Vehicle", "step", gate, watch=watch)
    _tick(rec)
    wrapped(False)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    (tick,) = list(read_record(tmp_path))
    assert tick["calls"][0]["changes"] == {"watched": {"slam": {"stop_now": True, "records": {"$append": [{"scan": 2}]}}}}


def test_value_types_exception_attributes_and_an_idempotent_close(tmp_path):
    from forklift_core.mission.boundary_record import RecordingGate, recorded_function

    class ArticulationAction:
        def __init__(self, v):
            self.joint_velocities = np.array([v, v])
            self.joint_indices = [2, 3]

    class CaptureFailure(RuntimeError):
        def __init__(self, reason, attempts):
            super().__init__(reason)
            self.reason, self.attempts = reason, attempts

    def fail():
        raise CaptureFailure("sensor_frame_not_ready", 7)

    rec = BoundaryRecorder(tmp_path)
    rec.value_types.add("ArticulationAction")
    gate = RecordingGate(rec)
    _tick(rec)
    recorded_function(lambda a: None, "Vehicle", "apply_action", gate)(ArticulationAction(0.5))
    with pytest.raises(CaptureFailure):
        recorded_function(fail, "Capture", "capture", gate)()
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    rec.close()
    (tick,) = list(read_record(tmp_path))
    action = tick["calls"][0]["request"]["args"][0]
    assert action["$object"] == "ArticulationAction" and action["joint_indices"] == [2, 3]
    assert np.array_equal(action["joint_velocities"], [0.5, 0.5])
    assert tick["calls"][1]["exception"]["attributes"] == {"reason": "sensor_frame_not_ready", "attempts": 7}


def test_a_method_watch_covers_what_nested_unrecorded_steps_change(tmp_path):
    from forklift_core.mission.boundary_record import DictWatch, RecordingGate, RecordingProxy, recorded_function

    slam = {"stop_now": False, "scan_id": 4}

    def step(render):
        slam["scan_id"] += 1
        slam["stop_now"] = True

    rec = BoundaryRecorder(tmp_path)
    gate = RecordingGate(rec)
    watch = DictWatch(rec, {"slam": slam}, keys={"slam": ["stop_now", "scan_id"]})
    stepper = recorded_function(step, "Vehicle", "step", gate, watch=watch)

    class Capture:
        def capture(self):
            stepper(True)  # nested: not recorded as its own call
            stepper(True)
            return {"accepted": True}

    proxy = RecordingProxy(Capture(), "Capture", gate, watches={"capture": watch})
    _tick(rec)
    proxy.capture()
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    (tick,) = list(read_record(tmp_path))
    assert [(c["port"], c["method"]) for c in tick["calls"]] == [("Capture", "capture")]
    assert tick["calls"][0]["changes"]["watched"] == {"slam": {"scan_id": 6, "stop_now": True}}


def test_a_recorded_run_is_refused_as_measurement_evidence_at_every_entry_point():
    """M-a0 (Codex rounds 4-5): bounds calibration and s4 drive terms (check_contract), observations and
    the bounds export (check_calibration, check_supplement_run), the P0a tables and the gate bounds."""
    from pathlib import Path

    from forklift_core.mission.boundary_record import measurement_refusal

    assert measurement_refusal({"arguments": {}}) is None
    assert measurement_refusal({}) is None
    assert "never measurement evidence" in measurement_refusal({"arguments": {"record_boundary": True}})
    root = Path(__file__).resolve().parents[3]
    assert "check_not_recorded(run)" in (root / "tools/export_near_field_bounds.py").read_text()
    assert (root / "tools/p0a_measure.py").read_text().count("    refuse_recorded(args.run)\n") == 2
    assert (root / "tools/gate_measure.py").read_text().count("        _refuse_recorded(run, result)\n") == 2
