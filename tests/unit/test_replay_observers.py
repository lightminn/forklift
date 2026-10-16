"""The format-2 output observers record each file write as a Sink entry where it happens (M-a1 replay plan 3.5)."""

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))

import replay_observers as O  # noqa: E402
from forklift_core.mission.boundary_record import BoundaryRecorder, RecordingGate  # noqa: E402
from forklift_core.mission.replay import read_ticks  # noqa: E402


def test_writes_through_the_output_and_numpy_are_sinks_by_relative_path_and_payload(tmp_path):
    out = tmp_path / "run"
    out.mkdir()
    rec = BoundaryRecorder(tmp_path / "boundary", value_format=2)
    gate = RecordingGate(rec)
    output = O.observed_output(out, lambda: gate)
    fake_np = O.NumpyObserver(np, lambda: gate, output)
    assert isinstance(np.zeros(2), fake_np.ndarray) and fake_np.float32 is np.float32
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    (output / "paths.json").write_text('{"a": 1}\n')
    fake_np.savez_compressed(output / "grid_mission_plan_1.npz", occupied=np.ones((2, 2), dtype=bool), res=0.05)
    fake_np.save(output / "slam_control.npy", np.arange(3.0))
    rec.tick_end(outputs={}, branches=[])
    (output / "after.json").write_text("{}")  # outside a tick: written, not recorded
    rec.close()
    calls = next(iter(read_ticks(tmp_path / "boundary")))["calls"]
    assert [(c["port"], c["method"]) for c in calls] == [
        ("Sink", "write_text"), ("Sink", "np_savez_compressed"), ("Sink", "np_save")]
    paths = [json.dumps(c["request"]) for c in calls]
    assert '"paths.json"' in paths[0] and '"grid_mission_plan_1.npz"' in paths[1] and '"slam_control.npy"' in paths[2]
    assert (out / "paths.json").read_text() == '{"a": 1}\n' and (out / "after.json").exists()
    with np.load(out / "grid_mission_plan_1.npz") as z:
        assert z["occupied"].all() and float(z["res"]) == 0.05


def test_a_different_payload_has_a_different_digest(tmp_path):
    from forklift_core.mission.boundary_record import payload_digest

    assert payload_digest({"kwds": {"a": np.ones(2)}}) != payload_digest({"kwds": {"a": np.zeros(2)}})
    assert payload_digest("x\n") == payload_digest("x\n")


def test_a_failed_write_is_recorded_and_raised_again_at_the_same_place_on_replay(tmp_path):
    from forklift_core.mission.replay import ReplayGate, ReplayMismatch, ReplaySession

    out = tmp_path / "run"
    (out / "blocked.json").mkdir(parents=True)  # writing a file over a directory fails
    rec = BoundaryRecorder(tmp_path / "boundary", value_format=2)
    gate = RecordingGate(rec)
    output = O.observed_output(out, lambda: gate)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    try:
        (output / "blocked.json").write_text("{}")
    except OSError:
        pass
    (output / "ok.json").write_text("{}")
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    calls = next(iter(read_ticks(tmp_path / "boundary")))["calls"]
    assert "exception" in calls[0] and "exception" not in calls[1]

    def replay(replay_out):
        session = ReplaySession(tmp_path / "boundary")
        rgate = ReplayGate(session)
        routput = O.observed_output(replay_out, lambda: rgate)
        session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
        raised = None
        try:
            (routput / "blocked.json").write_text("{}")
        except OSError as exc:
            raised = exc
        (routput / "ok.json").write_text("{}")
        session.tick_end(outputs={}, branches=[])
        session.finish()
        return raised

    fresh = tmp_path / "replay"
    fresh.mkdir()
    assert isinstance(replay(fresh), OSError)  # raised from the record although this directory could be written
    assert (fresh / "ok.json").read_text() == "{}" and not (fresh / "blocked.json").exists()
    bad = tmp_path / "replay_bad"
    (bad / "ok.json").mkdir(parents=True)  # the write the record says succeeded fails here
    try:
        replay(bad)
    except ReplayMismatch as exc:
        assert "failed in the replay, not in the record" in str(exc)
    else:
        raise AssertionError("a write failing only in the replay passed")
