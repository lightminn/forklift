"""The scene replan's escape with the start-band release (docs/plans/2026-10-24-start-band-release.md 2.2): the
trigger's rectangles the stop already comes within the sweep pad of are released to the moving-away rule and named in
the escape's record; off, the record has no such key."""

import math

import numpy as np
from test_core_scene_replan import DT, _real_layer, make  # the device's core fixtures

from forklift_core.control import escape as esc
from forklift_core.planning import Rectangle


def _standing(core):
    layer, config = _real_layer()
    pose = (2.0, 0.3, 0.05)
    n = len(layer.beam_angles)
    open_ = {s_.name: (np.full(n, 4.0), np.ones(n, bool), np.zeros(n, bool)) for s_ in config["sensors"]}
    for k in range(3):
        layer.add_scans(0.1 * k, open_, odom_rear=pose, loaded=True)
    layer.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=np.array([pose, (2.5, 0.3, 0.05)]),
                  loaded=True)
    core.obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "version": 0}
    return pose


def _near_rect(core, pose):
    footprint = core.escape_leg("transport")[3][1]
    # a 5 cm square beside the left side's middle, 5 mm off it
    c, s = math.cos(pose[2]), math.sin(pose[2])
    lon, lat = 0.6, footprint.half_width_m + 0.005 + 0.025 / math.cos(pose[2]) + 0.002
    rect = Rectangle(pose[0] + lon * c - lat * s, pose[1] + lon * s + lat * c, 0.05, 0.05, 0.0)
    near, far = esc.start_band_rectangles([rect], footprint, pose)
    assert near == [rect], "the fixture's rectangle must lie within the sweep pad of the start"
    return rect


def test_on_the_released_rectangles_are_named_in_the_escape_record():
    core, ms, state, calls, rear, held = make()
    core.options.start_band_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    ports = core._scene_ports(0.2, np.array(pose), 0.0, DT)
    ports.escape_plan((rect,), None)
    rec = state["scene_escape_candidates"][-1]
    assert rec["released"] == [{"x_m": rect.x_m, "y_m": rect.y_m, "length_m": 0.05, "width_m": 0.05, "yaw_rad": 0.0}]


def test_off_the_record_has_no_released_key():
    core, ms, state, calls, rear, held = make()
    pose = _standing(core)
    rect = _near_rect(core, pose)
    ports = core._scene_ports(0.2, np.array(pose), 0.0, DT)
    ports.escape_plan((rect,), None)
    assert "released" not in state["scene_escape_candidates"][-1]


def _crafted(core, layer, pose, *, version):
    """A processed snapshot of the layer's grid, every cell free and fresh, but for cells 8-19 mm off the truck's
    left side (the start band of a stop beside a wall)."""
    import dataclasses

    from forklift_core.control import start_release as sr
    from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot

    cfg = layer.grid_config
    nx, ny = cfg.shape
    snap = GridSnapshot(np.full((nx, ny), FREE, dtype=np.uint8), np.full((nx, ny), 10.0), 10.0, (0.0, 0.0, 0.0), version, 1,
                        {}, cfg.x_min_m, cfg.y_min_m, cfg.resolution_m)
    footprint = core.escape_leg("transport")[3][1]
    res = cfg.resolution_m
    ii, jj = np.meshgrid(np.arange(nx), np.arange(ny), indexing="ij")
    near = (np.abs(cfg.x_min_m + (ii + 0.5) * res - pose[0]) < 2.5) & (np.abs(cfg.y_min_m + (jj + 0.5) * res - pose[1]) < 1.5)
    cand = np.column_stack((ii[near], jj[near]))
    d = sr.cell_distances(snap, cand, pose, footprint)
    cx = cfg.x_min_m + (cand[:, 0] + 0.5) * res
    beside = (cx > pose[0] + 0.2) & (cx < pose[0] + 1.2)  # alongside the left side, not behind or ahead of the faces
    wall = cand[(d >= 0.008) & (d <= 0.019) & (cfg.y_min_m + (cand[:, 1] + 0.5) * res > pose[1]) & beside]
    assert len(wall), "the fixture needs cells within the start band"
    state = np.array(snap.state)
    state[wall[:, 0], wall[:, 1]] = OCCUPIED
    return dataclasses.replace(snap, state=state)


def _escape_through_the_device(on: bool):
    import dataclasses
    from types import SimpleNamespace

    from test_core_scene_replan import _drive

    core, ms, state, calls, rear, held = make()
    core.options.start_band_release = "on" if on else "off"
    layer, config = _real_layer()
    if on:
        pc = layer.permission.config
        layer.permission.config = dataclasses.replace(pc, start_band_m=pc.envelope_offset_m + pc.step_m / 2)
    pose = (2.0, 0.3, 0.05)
    rear["pose"] = np.array(pose)
    layer.last_scan_s = {s_.name: 10.0 for s_ in layer.sensors}  # fresh at the device's ticks
    layer.permission.note_standing(True, pose, 0)  # latched before the release's correction ...
    snap = _crafted(core, layer, pose, version=1)  # ... which re-projected the grid (version 1)
    layer.snapshot = snap
    layer.permission.snapshot = snap
    core.obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "version": 1}
    core.hooks.plan_transport_leg = lambda *a, **k: SimpleNamespace(success=False, status="invalid_start")
    core.hooks.truth_velocity = lambda: np.zeros(3)
    assert core.scene_replan_raise(10.0, "permit_stop", rear["pose"]) is None
    try:
        _drive(core, rear, held, n=6)
    except RuntimeError:
        pass
    return state


def test_the_scene_replan_s_escape_re_latches_after_the_release_s_correction_and_backs_away():
    # Codex round 3 P2: the device's ticks run the permission watch, so the escape port itself re-latches the anchor;
    # wait_stop -> wait_jobs -> plan -> wait_plan (invalid_start) -> escape_plan, on a real layer
    on = _escape_through_the_device(True)
    rec = on["scene_escape_candidates"][-1]
    assert rec["chosen"] is not None and rec["permit"]["ok"]
    assert on["scene_replans"][-1]["escape"] == "running"
    off = _escape_through_the_device(False)
    assert off["scene_escape_candidates"][-1]["chosen"] is None  # the walk's own rule: held at the start
