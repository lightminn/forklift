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
