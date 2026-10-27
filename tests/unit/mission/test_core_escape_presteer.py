"""The escape presteer in the core (docs/plans/2026-10-27-escape-presteer.md D2, D3, D2-6): the probe evaluates the
escape at a set steering on a permission copy, an escape the presteer started keeps its curvature between the measured
steering and 0, and a resume while a scene request runs is owned by that request."""

import copy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

import test_core_pause as cp
from test_core_scene_replan import DT, _real_layer, make


def _open_core(*, band=None):
    core, ms, state, calls, rear, held = make()
    layer, config = _real_layer()
    pose = (2.0, 0.3, 0.05)
    n = len(layer.beam_angles)
    open_ = {s_.name: (np.full(n, 4.0), np.ones(n, bool), np.zeros(n, bool)) for s_ in config["sensors"]}
    for k in range(3):
        layer.add_scans(0.1 * k, open_, odom_rear=pose, loaded=True)
    layer.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=np.array([pose, (2.5, 0.3, 0.05)]),
                  loaded=True)
    core.obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "version": 0}
    if band is not None:
        core.options.start_band_release = "on"
        layer.permission.config = replace(layer.permission.config, start_band_m=band)
    return core, state, held, layer, pose


def test_the_probe_gives_the_escape_at_its_steering_on_a_copy_leaving_records_and_the_anchor_alone():
    core, state, held, layer, pose = _open_core(band=0.03)
    ports = core._scene_ports(0.2, np.array(pose), 0.0, DT)
    held["kappa"] = 0.05
    at_held = ports.escape_plan((), None)
    record = state["scene_escape_candidates"][-1]
    anchor = copy.deepcopy(layer.permission.release_anchor)
    stats = dict(layer.permission.release_stats)
    n_records = len(state["scene_escape_candidates"])
    held["kappa"] = 0.0
    probe = ports.escape_probe((), None, 0.05)
    assert probe["ok"] is (at_held is not None)
    assert probe["chosen"] == "{:+d}x{:g}".format(record["chosen"]["direction"], record["chosen"]["length_m"])
    assert probe["end_clearance_m"] == record["chosen"]["end_clearance_m"]
    assert len(state["scene_escape_candidates"]) == n_records  # nothing recorded
    assert layer.permission.release_anchor == anchor and layer.permission.release_stats == stats
    assert "escape_start_release" not in core.obstacle  # nothing counted


def test_a_presteered_escape_carries_the_measured_steering_range_and_records_it():
    core, state, held, layer, pose = _open_core()
    ports = core._scene_ports(0.2, np.array(pose), 0.0, DT)
    held["kappa"] = 0.04
    chosen = ports.escape_plan((), None, presteered=True)
    assert chosen is not None and chosen[4] == (0.0, 0.04)
    assert state["scene_escape_candidates"][-1]["presteered"] is True
    assert state["scene_escape_candidates"][-1]["kappa_measured"] == 0.04
    held["kappa"] = -0.03
    assert ports.escape_plan((), None, presteered=True)[4] == (-0.03, 0.0)
    assert len(ports.escape_plan((), None)) == 4  # an escape without the presteer is as before


def _running_escape(core, state, held, pose, kappa, offset_y):
    core.scene_replan.records = state.setdefault("scene_replans", [])
    core.scene_replan.request(0.2, "permit_stop", phase="transport", tracker_serial=1, leg=0, rear=pose,
                              truth_rear=None)
    held["kappa"] = kappa
    ports = core._scene_ports(0.2, np.array(pose), 0.0, DT)
    chosen = ports.escape_plan((), None, presteered=True)
    ports.escape_install(chosen)
    core.scene_replan.state = "escape"
    rear = np.array(pose) + np.array([0.0, offset_y, 0.0])  # off the straight: the tracker steers back
    return rear


def test_the_escape_s_command_is_kept_between_the_measured_steering_and_zero():
    for kappa, offset in ((0.05, 0.03), (0.05, -0.03), (-0.05, 0.03), (-0.05, -0.03)):
        core, state, held, layer, pose = _open_core()
        core.last_permission = (0.15, "ok")
        rear = _running_escape(core, state, held, pose, kappa, offset)
        out = core.scene_replan_tick(0.3, rear, -0.05, DT)
        lo, hi = min(kappa, 0.0), max(kappa, 0.0)
        assert out.output == "escape" and lo - 1e-12 <= out.curvature <= hi + 1e-12
        clip = core.scene_escape["clip"]
        assert clip["range"] == [lo, hi]
        assert not lo <= clip["raw_min"] <= hi and clip["clipped"] == 1  # 3 cm off: the tracker asks ~0.36 m^-1


def test_an_escape_without_the_presteer_is_not_clipped():
    core, state, held, layer, pose = _open_core()
    core.last_permission = (0.15, "ok")
    core.scene_replan.records = state.setdefault("scene_replans", [])
    core.scene_replan.request(0.2, "permit_stop", phase="transport", tracker_serial=1, leg=0, rear=pose,
                              truth_rear=None)
    ports = core._scene_ports(0.2, np.array(pose), 0.0, DT)
    ports.escape_install(ports.escape_plan((), None))
    core.scene_replan.state = "escape"
    core.scene_replan_tick(0.3, np.array(pose) + np.array([0.0, 0.03, 0.0]), -0.05, DT)
    assert "clip" not in core.scene_escape


def test_the_presteered_escape_s_end_is_recorded_once():
    core, state, held, layer, pose = _open_core()
    core.last_permission = (0.15, "ok")
    _running_escape(core, state, held, pose, 0.05, 0.0)
    goal = np.array(core.scene_escape["clip"]["goal"])
    core.hooks.truth_velocity = lambda: np.zeros(3)
    for k in range(3):
        core.scene_replan_tick(1.0 + 0.1 * k, goal, 0.0, DT)
    drives = state["escape_presteer_drives"]
    assert len(drives) == 1 and drives[0]["end"]["status"] in ("done", "stopped")
    assert abs(drives[0]["end"]["along_m"]) < 1e-9 and abs(drives[0]["end"]["across_m"]) < 1e-9


def _scene_request(core):
    core.scene_replan.records = core.state.setdefault("scene_replans", [])
    core.scene_replan.request(9.0, "permit_stop", phase="transport", tracker_serial=1, leg=0, rear=(0.0, 0.0, 0.0),
                              truth_rear=None)


def test_a_resume_during_a_scene_request_neither_retries_the_docking_nor_replans():
    core, asked = cp.docking_core()
    _scene_request(core)
    core.paused_tick(11.0, cp.REAR + np.array([0.05, 0.0, 0.0]), 0.0, False)  # moved: R11 would retry
    assert asked == [] and core.commands.status == "running"
    assert core.commands.resume_check["step"] == "running"


def test_a_resume_during_a_scene_request_on_a_blocked_path_does_not_run_the_general_recovery():
    core, _, _ = cp.make(layer=cp.Layer(blocked="occupied"))
    _scene_request(core)
    cp.pause(core, 10.0)
    cp.resume(core, 11.0)
    asked = []
    core.blocked_replan = lambda t, rear, kappa, requested: asked.append(t) or SimpleNamespace(rear=rear,
                                                                                              requested_speed=0.0)
    core.paused_tick(11.0, cp.REAR, 0.0, True)
    assert asked == [] and core.commands.status == "running"


def test_without_a_scene_request_the_moved_docking_straight_still_retries():
    core, asked = cp.docking_core()
    core.paused_tick(11.0, cp.REAR + np.array([0.05, 0.0, 0.0]), 0.0, False)
    assert asked == [(11.0, "resume", "docking_moved_while_paused")]


def test_the_resume_moves_a_running_escape_s_zero_wait_by_the_pause():
    core, obs, _ = cp.make()
    core.scene_escape = {"status": "running", "zero_since": 10.0}
    cp.pause(core, 10.0)
    cp.resume(core, 11.0)
    core.paused_tick(13.0, cp.REAR, 0.0, True)
    assert core.commands.status == "running"
    assert core.scene_escape["zero_since"] == 13.0  # 10.0 + the 3 s pause


def test_a_resume_with_another_phase_s_request_left_is_checked_as_before():
    # Codex plan round 4: only this phase's request is owned; anything else gets the resume's own checks
    core, _, _ = cp.make("return_home", layer=cp.Layer(blocked="occupied"))
    _scene_request(core)  # a transport request
    cp.pause(core, 10.0)
    cp.resume(core, 11.0)
    asked = []
    core.blocked_replan = lambda t, rear, kappa, requested: asked.append(t) or SimpleNamespace(rear=rear,
                                                                                              requested_speed=0.0)
    core.paused_tick(11.0, cp.REAR, 0.0, True)
    assert asked == [11.0]


def test_a_cancel_that_backs_out_ends_the_scene_request_and_its_escape():
    core, obs, _ = cp.make("return_home")
    _scene_request(core)
    core.scene_escape = {"status": "running", "zero_since": None}
    obs["backoff"] = {"phase": "return_home", "scene": True}
    core.end_gate = lambda t, rear: False
    started = []
    core.start_clear_out = lambda t, rear: started.append(t)
    assert core.finish_cancel(12.0, cp.REAR) is False and started == [12.0]
    assert not core.scene_replan.active() and core.scene_escape is None and "backoff" not in obs
    assert core.state["scene_replans"][-1]["reason"] == "cancelled:clear_out"
