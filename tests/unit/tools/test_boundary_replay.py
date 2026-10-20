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


def test_a_recorded_system_exit_or_early_failure_is_matched_against_the_recorded_ending(tmp_path):
    boundary = _record(tmp_path, boundary_ending(SystemExit("bootstrap: no SLAM link")))
    session = ReplaySession(boundary)
    session.closed = True
    H.check_ending(session, SystemExit("bootstrap: no SLAM link"))
    assert session.failure is None
    session = ReplaySession(boundary)
    session.closed = True
    H.check_ending(session, None)
    assert "the replay ended" in session.failure


def test_a_sibling_module_the_runner_loads_by_file_in_its_setup_resolves_on_replay():
    import importlib

    from forklift_core.mission.replay_codec import _resolve

    assert _resolve("run_transport_new_obstacles:Schedule").__name__ == "Schedule"
    assert importlib.util.find_spec("run_transport_no_such_module") is None


def test_the_replay_recorder_takes_every_argument_the_runner_and_the_recorder_do():
    """The runner calls tick_time with the mission commands and the stop kind (M-a3d); the replay tool's wrapper
    dropped them and every replay failed at its second tick."""
    import inspect

    wrapper = set(inspect.signature(H.SessionRecorder.tick_time).parameters) - {"self"}
    for other in (BoundaryRecorder.tick_time, ReplaySession.tick_time):
        assert wrapper == set(inspect.signature(other).parameters) - {"self"}


def test_a_snapshot_from_before_a_setup_name_binds_it_as_the_setup_does_with_its_feature_off():
    """PR #53 far recognition: the 29 gate records predate far_runtime / far_intrinsics / frr (UnboundLocalError in
    drive on every replay); today's setup binds all three None when --recognition is off."""
    from types import SimpleNamespace

    values = {"args": SimpleNamespace(output="x")}  # a record from before the flag
    assert sorted(H.bind_added_names(values, {"args": {}})) == ["far_intrinsics", "far_runtime", "frr", "slam_bundle_"]
    assert values["far_runtime"] is None and values["far_intrinsics"] is None and values["frr"] is None
    values = {"args": SimpleNamespace(recognition="off"), "far_runtime": "kept"}
    assert H.bind_added_names(values, {"far_runtime": {}, "far_intrinsics": {}, "frr": {}, "slam_bundle_": {}}) == []
    assert values["far_runtime"] == "kept"
    with pytest.raises(ReplayMismatch, match="no far_runtime although recognition='far'"):
        H.bind_added_names({"args": SimpleNamespace(recognition="far")}, {"args": {}})
    # SLAM localization plan L7: an older mapping record binds no bundle; a localization one must have its own
    values = {"args": SimpleNamespace(output="x")}
    H.bind_added_names(values, {"args": {}})
    assert values["slam_bundle_"] is None
    with pytest.raises(ReplayMismatch, match="no slam_bundle_ although slam_map_bundle='/b'"):
        H.bind_added_names({"args": SimpleNamespace(slam_map_bundle="/b")}, {"args": {}})


def test_a_bottom_deck_check_on_record_is_refused_by_name_and_an_off_or_older_one_is_not():
    """Plan 2026-10-19 task 4: the side legs' records are not replayable yet; earlier records ran without them."""
    from types import SimpleNamespace

    H.refuse_unreplayable({"args": SimpleNamespace(output="x")})  # before the flag
    H.refuse_unreplayable({"args": SimpleNamespace(bottom_deck_check="off")})
    with pytest.raises(ReplayMismatch, match="a bottom_deck_check='on' record is not replayable"):
        H.refuse_unreplayable({"args": SimpleNamespace(bottom_deck_check="on")})
    # gyro-heading plan G2 (A): a gyro-heading record is refused by name too
    H.refuse_unreplayable({"args": SimpleNamespace(odometry_heading="steering")})
    with pytest.raises(ReplayMismatch, match="a odometry_heading='gyro' record is not replayable"):
        H.refuse_unreplayable({"args": SimpleNamespace(odometry_heading="gyro")})


def test_every_added_setup_name_is_one_drive_binds_and_the_setup_always_binds():
    import ast

    tree = ast.parse((ROOT / "sim/isaac/run_transport.py").read_text())
    fns = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
    prologue = {ast.literal_eval(st.test.left) for st in fns["drive"].body
                if isinstance(st, ast.If) and ast.unparse(st.test).endswith("in _bound")}
    unconditional = {}  # the constant run() assigns each name at its own top level (not under an if)
    for st in fns["run"].body:
        if isinstance(st, ast.Assign):
            try:
                value = ast.literal_eval(st.value)
            except ValueError:
                continue
            unconditional.update({t.id: value for t in st.targets if isinstance(t, ast.Name)})
    for name, (_flag, _off, value) in H.SETUP_NAMES_ADDED.items():
        assert name in prologue and name in unconditional, name
        assert unconditional[name] == value and type(unconditional[name]) is type(value), name  # Codex P3
