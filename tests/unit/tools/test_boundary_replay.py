"""The CPU replay harness's own checks (docs/plans/2026-10-16-m-a1-replay.md, R4): how a replayed run has to
end, and the stand-ins it hands drive() refusing what the record cannot answer."""

import importlib.util
import json
from pathlib import Path

import pytest

from forklift_core.mission.boundary_record import BoundaryRecorder, boundary_ending
from forklift_core.mission.replay import ReplayMismatch, ReplaySession

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("boundary_replay", ROOT / "tools/boundary_replay.py")
H = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(H)


class Vehicle:
    def get_world_pose(self):
        return (0.0, 0.0)


def _record(run_dir: Path, ending) -> Path:
    rec = BoundaryRecorder(run_dir / "boundary", value_format=2)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rec.tick_end(outputs={}, branches=[])
    rec.close(ending=ending)
    return run_dir / "boundary"


def test_a_replay_has_to_end_as_the_recorded_run_did(tmp_path):
    boundary = _record(tmp_path, boundary_ending(None))
    session = ReplaySession(boundary)
    session.closed = True
    H.check_ending(session, ValueError("after the last call"))
    assert "the replay ended" in session.failure
    session = ReplaySession(boundary)
    session.closed = True
    H.check_ending(session, None)
    assert session.failure is None
    session = ReplaySession(boundary)  # returned, but never closed its record
    H.check_ending(session, None)
    assert "never closed" in session.failure


def test_the_same_exception_is_the_same_ending_and_a_record_without_one_is_refused(tmp_path):
    boundary = _record(tmp_path / "a", boundary_ending(RuntimeError("estop_timeout")))
    session = ReplaySession(boundary)
    session.closed = True
    H.check_ending(session, RuntimeError("estop_timeout"))
    assert session.failure is None
    session = ReplaySession(boundary)
    session.closed = True
    H.check_ending(session, RuntimeError("another reason"))
    assert session.failure is not None
    boundary = _record(tmp_path / "b", None)
    session = ReplaySession(boundary)
    session.closed = True
    H.check_ending(session, None)
    assert "does not say how" in session.failure


def _snapshot(run_dir: Path) -> None:
    (run_dir / "boundary/setup_snapshot.json").write_text(json.dumps({
        "format": "forklift_setup_snapshot/1", "opaque": [], "files": [], "service_specs": {},
        "names": {
            "robot": {"kind": "service", "id": "Vehicle", "spec": {"methods": ["get_world_pose"], "properties": []}},
            "detect_pockets": {"kind": "port", "port": "Perception", "method": "detect_pockets"},
        }}))


def test_the_proxy_factory_refuses_another_gate_and_a_service_wired_to_another_port(tmp_path):
    _record(tmp_path, boundary_ending(None))
    _snapshot(tmp_path)
    gate = H.ReplayBoundary(ReplaySession(tmp_path / "boundary"))
    S, _, _ = H.build_inputs(tmp_path, gate)
    with pytest.raises(ReplayMismatch, match="wrapped as SimTruth"):
        S.RecordingProxy(S.robot, "SimTruth", gate)
    assert "wrapped as SimTruth" in gate.session.failure  # latched
    gate = H.ReplayBoundary(ReplaySession(tmp_path / "boundary"))
    S, _, _ = H.build_inputs(tmp_path, gate)
    other = H.ReplayBoundary(ReplaySession(tmp_path / "boundary"))
    with pytest.raises(ReplayMismatch, match="another gate"):
        S.RecordingProxy(S.robot, "Vehicle", other)
    assert gate.session.failure is not None
    gate = H.ReplayBoundary(ReplaySession(tmp_path / "boundary"))
    S, _, _ = H.build_inputs(tmp_path, gate)
    assert object.__getattribute__(S.RecordingProxy(S.robot, "Vehicle", gate), "_port") == "Vehicle"
    assert gate.session.failure is None


def test_a_port_called_outside_a_recorded_tick_fails_even_when_the_code_catches_it(tmp_path):
    _record(tmp_path, boundary_ending(None))
    _snapshot(tmp_path)
    gate = H.ReplayBoundary(ReplaySession(tmp_path / "boundary"))
    S, adapters, _ = H.build_inputs(tmp_path, gate)
    try:
        S.detect_pockets("frame")
    except ReplayMismatch:
        pass
    assert "detect_pockets ran outside a recorded tick" in gate.session.failure
    gate = H.ReplayBoundary(ReplaySession(tmp_path / "boundary"))
    S, adapters, _ = H.build_inputs(tmp_path, gate)
    try:
        adapters.cast_reference_scan()
    except ReplayMismatch:
        pass
    assert "cast_reference_scan" in gate.session.failure
