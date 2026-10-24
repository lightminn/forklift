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
