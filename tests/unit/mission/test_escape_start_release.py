"""The P6 escape's own start-pad release (docs/plans/2026-10-25-escape-start-release.md E1-E3): the scene replan's
escape sweep passes the trigger's rectangles the stop already stands within the pad of while it moves away from them,
with the permission left as it is -- no start band, no anchor; the escape's other checks and its final permit stay."""

import numpy as np
from test_core_scene_replan import DT, make  # the device's core fixtures
from test_start_band_escape import _escape_through_the_device, _near_rect, _standing

from forklift_core.planning import Rectangle


def _plan(core, extra):
    pose = (2.0, 0.3, 0.05)
    ports = core._scene_ports(0.2, np.array(pose), 0.0, DT)
    return ports.escape_plan(tuple(extra), None)


def test_on_the_escape_backs_away_from_a_rectangle_within_the_pad_and_names_it():
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    chosen = _plan(core, [rect])
    rec = state["scene_escape_candidates"][-1]
    assert chosen is not None and rec["permit"]["ok"]
    assert rec["released"] == [{"x_m": rect.x_m, "y_m": rect.y_m, "length_m": 0.05, "width_m": 0.05, "yaw_rad": 0.0}]
    assert core.obstacle["escape_start_release"] == {"escapes_released": 1}


def test_off_the_same_rectangle_refuses_every_straight_at_its_start():
    core, ms, state, calls, rear, held = make()
    pose = _standing(core)
    rect = _near_rect(core, pose)
    assert _plan(core, [rect]) is None
    rec = state["scene_escape_candidates"][-1]
    assert not any(c["sweep_free"] for c in rec["candidates"])
    assert "released" not in rec and "escape_start_release" not in core.obstacle


def test_on_the_permission_keeps_its_own_start_rule():
    # the release is the sweep's alone: no start band, no anchor latched (unlike --start-band-release on)
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    _plan(core, [rect])
    perm = core.obstacle["layer"].permission
    assert perm.config.start_band_m == 0.0 and perm.release_anchor is None


def test_on_a_rectangle_the_start_overlaps_is_not_released():
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    over = Rectangle(pose[0] + 0.6, pose[1], 0.05, 0.05, 0.0)  # under the truck's middle
    assert _plan(core, [over]) is None
    rec = state["scene_escape_candidates"][-1]
    assert "released" not in rec and not any(c["sweep_free"] for c in rec["candidates"])
    assert "escape_start_release" not in core.obstacle


def test_on_a_released_rectangle_still_counts_for_the_end_region():
    # a second rectangle within the pad that the straight backs along stays within the end threshold of its end
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    long_ = Rectangle(rect.x_m - 1.5 * np.cos(pose[2]), rect.y_m - 1.5 * np.sin(pose[2]), 3.0, 0.05, pose[2])
    _plan(core, [long_])
    rec = state["scene_escape_candidates"][-1]
    assert rec.get("released"), "the fixture's rectangle must be released"
    backs = [c for c in rec["candidates"] if c["direction"] == -1]
    assert all(c["sweep_free"] for c in backs)  # it runs alongside: never nearer
    assert all(not c["qualifies"] and c["end_clearance_m"] < c["threshold_m"] for c in backs)


def test_on_a_refused_final_permit_installs_no_escape():
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    layer = core.obstacle["layer"]
    layer.candidate_permit = lambda *a, **k: {"ok": False, "why": "occupied"}
    assert _plan(core, [rect]) is None
    assert state["scene_escape_candidates"][-1]["chosen"] is not None


def test_on_through_the_device_the_permission_still_holds_a_start_within_its_own_band():
    # the start-band test's stop beside a wall of cells 8-19 mm off: the sweep part alone does not override the
    # permission's own rule, which holds the start (only --start-band-release on releases the walk)
    import test_start_band_escape as sbe

    real_make = sbe.make

    def make_escape_only():
        core, *rest = real_make()
        core.options.escape_start_release = "on"
        return (core, *rest)

    sbe.make = make_escape_only
    try:
        state = _escape_through_the_device(False)
    finally:
        sbe.make = real_make
    rec = state["scene_escape_candidates"][-1]
    assert rec["chosen"] is None


def test_on_a_start_nearer_than_the_dense_sweep_error_is_not_passed():
    # agy: 0 < d0 < min_d0_m -- named as within the pad, but the moving-away rule refuses it (the continuous sweep
    # could touch between the checked poses), so no straight passes the sweep
    import math

    from forklift_core.control import escape as esc
    from forklift_core.control.start_release import footprint_rect_distance, min_d0_m

    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    footprint = core.escape_leg("transport")[3][1]
    rect = _near_rect(core, pose)
    gap = footprint_rect_distance(pose, footprint, rect)
    shift = gap - min_d0_m(footprint) / 2.0  # move it in to half the dense error
    c, s = math.cos(pose[2]), math.sin(pose[2])
    close = Rectangle(rect.x_m + shift * s, rect.y_m - shift * c, 0.05, 0.05, 0.0)
    assert 0.0 < footprint_rect_distance(pose, footprint, close) < min_d0_m(footprint)
    assert esc.start_band_rectangles([close], footprint, pose)[0] == [close]
    assert _plan(core, [close]) is None
    rec = state["scene_escape_candidates"][-1]
    assert rec["released"] and not any(c_["sweep_free"] for c_ in rec["candidates"])


def test_both_switches_on_release_the_sweep_once_and_keep_both_records_apart():
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    core.options.start_band_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    assert _plan(core, [rect]) is not None
    assert len(state["scene_escape_candidates"][-1]["released"]) == 1
    assert core.obstacle["escape_start_release"] == {"escapes_released": 1}


def test_the_start_band_release_alone_keeps_no_escape_start_record():
    core, ms, state, calls, rear, held = make()
    core.options.start_band_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    _plan(core, [rect])
    assert state["scene_escape_candidates"][-1]["released"]
    assert "escape_start_release" not in core.obstacle
