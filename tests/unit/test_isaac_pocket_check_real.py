"""sim/isaac/pocket_check.py's real D435i path (PR 7 7e-2a, docs/plans/2026-11-26-7e-depth-pocket-check.md 6.2, 9.2).

The same synthetic straight as tests/unit/test_isaac_pocket_check.py, with ``real=RealPocketOptions(...)`` and
``synthetic_noise=False``: novelty by device identity, the usable-observation freshness, the three-state visibility,
refutation, the guard. A few frames per case; the long drives (the passable error bound) are
tools/d5_real_bound_sweep.py's."""

import numpy as np
import pytest

from forklift_core.perception.insertion_clearance import Box, ray_blocked

from test_isaac_pocket_check import BLADES, CAM, END, FACE, GEOM, OFFSET, START, base_from_optical, pocket_check, \
    render
from forklift_core.control.drive_permission import StoppingModel

TIP = 1.29  # BLADES' tip, rear-axle frame


def make(real=None, **config):
    real = real if real is not None else pocket_check.RealPocketOptions(
        view_min_px=500, pallet_rel_error_m=0.0, own_error_m=0.002, cam_trans_error_m=0.0, cam_rot_error_rad=0.0)
    return pocket_check.PocketCheck(
        estimate=(0.0, 0.0, 0.0), axis_yaw=0.0, pallet_geometry=GEOM, blades_rear=BLADES,
        body_front_m=0.884, body_rear_m=0.17, body_half_width_m=0.36, insertion_depth_m=0.36, base_z_m=0.0,
        base_from_optical=base_from_optical(), rear_axle_offset_m=OFFSET, camera=CAM,
        stopping=StoppingModel(0.15, 1.5, 0.05), config=pocket_check.PocketConfig(**config),
        synthetic_noise=False, real=real)


def options(**kw):
    base = dict(view_min_px=500, pallet_rel_error_m=0.0, own_error_m=0.002, cam_trans_error_m=0.0,
                cam_rot_error_rad=0.0)
    base.update(kw)
    return pocket_check.RealPocketOptions(**base)


def approach(check, xs, t0=0.0, dt=0.1, extra=(), speed=0.0, n0=0):
    """Frames at rear-axle x positions (on the axis), one per dt; the last record."""
    rec = None
    for k, x in enumerate(xs):
        rear = (x, 0.0, 0.0)
        rec = check.add_frame(t0 + k * dt, render(check, rear, extra), rear, speed_mps=speed,
                              frame_key=(1, (), n0 + k))
    return rec


def arc_pixels(check, rear, speed, radius=2):
    """A pixel mask of the voxels the stop arc at ``rear`` demands (each with a small square)."""
    from forklift_core.control.drive_permission import arc_poses
    from forklift_core.perception.insertion_clearance import overlapping_voxels

    length = check.real_stop.distance_m(speed) + check.config.sample_m
    samples, _ = arc_poses(tuple(rear), 0.0, 1, length, check.config.sample_m)
    vol = check.memory.volume
    cells = set()
    for pose in samples:
        for b in check.truck_boxes(check.to_insertion(*pose), 0.0):
            idx, _ = overlapping_voxels(vol, b, check.region)
            cells.update(map(tuple, idx))
    idx = np.array(sorted(cells))
    centres = vol.lo + (idx + 0.5) * vol.voxel_m
    rot, trans = check.optical_from_insertion(rear)
    p = centres @ rot.T + trans
    mask = np.zeros((CAM.height, CAM.width), dtype=bool)
    ok = p[:, 2] > 0
    u = np.round(CAM.fx * p[ok, 0] / p[ok, 2] + CAM.cx).astype(int)
    w = np.round(CAM.fy * p[ok, 1] / p[ok, 2] + CAM.cy).astype(int)
    for du in range(-radius, radius + 1):
        for dw in range(-radius, radius + 1):
            uu, ww = u + du, w + dw
            good = (uu >= 0) & (uu < CAM.width) & (ww >= 0) & (ww < CAM.height)
            mask[ww[good], uu[good]] = True
    return mask


# --- the Isaac path is untouched ------------------------------------------------------------------------------------
def test_the_isaac_defaults_never_reach_the_real_path(monkeypatch):
    check = pocket_check.PocketCheck(
        estimate=(0.0, 0.0, 0.0), axis_yaw=0.0, pallet_geometry=GEOM, blades_rear=BLADES, body_front_m=0.884,
        body_rear_m=0.17, body_half_width_m=0.36, insertion_depth_m=0.36, base_z_m=0.0,
        base_from_optical=base_from_optical(), rear_axle_offset_m=OFFSET, camera=CAM,
        stopping=StoppingModel(0.15, 1.5, 0.05))
    assert check.real is None and check.synthetic_noise is True

    def boom(*a, **k):
        raise AssertionError("the real path was reached")

    monkeypatch.setattr(check, "_visibility", boom)
    monkeypatch.setattr(check, "_add_real_frame", boom)
    monkeypatch.setattr(check, "forget", boom)
    rec = check.add_frame(0.0, render(check, (START, 0.0, 0.0)), (START, 0.0, 0.0))
    assert rec["new"] and "usable" not in rec and "refuted" not in rec
    check.limit(0.05, (START, 0.0, 0.0), 0.0, 1, 0.3, 0.0)
    s = check.summary()
    assert s["depth"].startswith("synthetic") and "refuted" not in s


# --- novelty and freshness ------------------------------------------------------------------------------------------
def test_a_frame_is_new_by_its_device_identity_not_its_pixels():
    check = make()
    rear = (START, 0.0, 0.0)
    depth = render(check, rear)
    assert check.add_frame(0.0, depth, rear, frame_key=(1, (), 5))["new"]
    assert check.add_frame(0.1, depth, rear, frame_key=(1, (), 6))["new"]  # the same pixels, a new device frame
    assert check.add_frame(0.2, depth, rear, frame_key=(1, (), 6))["new"] is False
    with pytest.raises(ValueError):
        check.add_frame(0.3, depth, rear)


def test_frames_without_usable_depth_do_not_keep_the_check_fresh():
    check = make()
    xs = [START + 0.1 * k for k in range(4)]
    approach(check, xs)
    rear = (xs[-1], 0.0, 0.0)
    assert check.limit(0.35, rear, 0.0, 1, 0.3, 0.0)[1] != "depth_stale"
    nan = np.full((CAM.height, CAM.width), np.nan)
    one = nan.copy()
    one[CAM.height - 1, CAM.width // 2] = 1.0  # a single valid pixel
    for k, frame in enumerate((nan, one, nan)):
        rec = check.add_frame(0.4 + 0.1 * k, frame, rear, frame_key=(1, (), 10 + k))
        assert rec["new"] and rec["usable"] is False
    # new device frames, but none usable: the freshness runs from the last usable one (0.3 s)
    assert check.limit(0.65, rear, 0.0, 1, 0.3, 0.0) == (0.0, "depth_stale")


def test_the_usable_test_counts_the_pixels_whose_rays_meet_the_volume():
    rear = (END, 0.0, 0.0)
    check = make()
    rec = check.add_frame(0.0, np.full((CAM.height, CAM.width), 5.0), rear, frame_key=(1, (), 0))
    # the voxel-centre pixels alone are far fewer than the ray mask (Codex 7e plan round 6 P2-2: 1,430 vs 27,548)
    vol = check.memory.volume
    idx, centres = vol.centres(evaluated=True)
    rot, trans = check.optical_from_insertion(rear)
    p = centres @ rot.T + trans
    ok = (p[:, 2] > 0) & vol.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
    u = np.round(CAM.fx * p[ok, 0] / p[ok, 2] + CAM.cx)
    w = np.round(CAM.fy * p[ok, 1] / p[ok, 2] + CAM.cy)
    inside = (u >= 0) & (u < CAM.width) & (w >= 0) & (w < CAM.height)
    centre_px = len({(int(a), int(b)) for a, b in zip(u[inside], w[inside])})
    assert rec["view_px"] > 3 * centre_px
    assert rec["view_valid_px"] == rec["view_px"] and rec["usable"]
    rec = check.add_frame(0.1, np.full((CAM.height, CAM.width), np.nan), rear, frame_key=(1, (), 1))
    assert rec["view_px"] > 0 and rec["view_valid_px"] == 0 and not rec["usable"]
    # half the mask valid: under view_valid_min 0.5 by a pixel is not usable
    depth = np.full((CAM.height, CAM.width), 5.0)
    depth[:, : CAM.width // 2] = np.nan
    rec = check.add_frame(0.2, depth, rear, frame_key=(1, (), 2))
    assert rec["usable"] == (rec["view_valid_px"] >= 500 and rec["view_valid_px"] >= 0.5 * rec["view_px"])


# --- refutation ------------------------------------------------------------------------------------------------------
def test_a_frame_that_cannot_pass_the_stop_arc_clears_its_recent_certification():
    # Codex 7e plan round 4 P1-2: a normal frame, then 60 ms later one with the stop arc's pixels gone; the query at
    # the send margin 35 ms after must stop although the earlier certification is only 95 ms old
    def run(blank):
        check = make()
        xs = [FACE - 0.9 - TIP + 0.1 * k for k in range(5)]
        approach(check, xs)
        rear = (xs[-1], 0.0, 0.0)
        t = 0.4
        assert check.limit(t + 0.035, rear, 0.0, 1, 0.3, 0.0)[0] > 0
        depth = render(check, rear)
        if blank:
            depth[arc_pixels(check, rear, 0.3)] = np.nan
        rec = check.add_frame(t + 0.06, depth, rear, frame_key=(1, (), 99))
        return check, rec, check.limit(t + 0.095, rear, 0.0, 1, 0.3, 0.0)

    _c, _rec, ok = run(blank=False)
    assert ok[0] > 0
    check, rec, stopped = run(blank=True)
    assert rec["refuted"] > 0
    assert stopped == (0.0, "pocket_unverified")
    # a refuted voxel is not revived by a later frame that cannot see it
    gone = np.isneginf(check.memory.certified_s)
    check.memory.add_frame(0.6, np.full((CAM.height, CAM.width), np.nan), CAM,
                           check.optical_from_insertion((START - 5.0, 0.0, 0.0)), check.surfaces,
                           visible=np.zeros(len(check.memory.volume.centres(evaluated=True)[0]), dtype=bool))
    assert np.isneginf(check.memory.certified_s[gone]).all()


def test_an_object_stuck_to_the_pallet_in_its_error_band_refutes_and_stops_before_contact():
    # Codex 7e plan round 5 P1-1: a thin object against the block's side in the pocket mouth, its returns within the
    # surface tolerance (no obstacle latch); a long certification exists from before it was placed
    check = make(real=options(pallet_rel_error_m=0.02))
    xs = [FACE - 1.0 - TIP + 0.1 * k for k in range(6)]
    approach(check, xs)
    # the left pocket's outer block side (EPAL 6: 100 mm blocks, 227.5 mm openings) -- a 30 mm object on it
    y_side = GEOM.overall_width_m / 2 - 0.100
    thing = Box((FACE + 0.10, y_side - 0.015, 0.05), (0.02, 0.015, 0.03))
    rear = (xs[-1], 0.0, 0.0)
    rec = check.add_frame(0.6, render(check, rear, [thing]), rear, frame_key=(1, (), 50))
    assert rec["refuted"] > 0
    x, t, n = rear[0], 0.7, 51
    while x < END:
        r = (x, 0.0, 0.0)
        check.add_frame(t, render(check, r, [thing]), r, speed_mps=0.1, frame_key=(1, (), n))
        allowed, why = check.limit(t + 0.035, r, 0.0, 1, 0.1, 0.0)
        if allowed <= 0:
            break
        x += min(0.1, allowed) * 0.1
        t, n = t + 0.1, n + 1
    assert x < END
    # the blade's outer edge (y 0.1725 + allowances) stops before the object's front face
    assert x + TIP < FACE + 0.08


# --- the three-state visibility --------------------------------------------------------------------------------------
def test_the_camera_ray_error_unhides_a_voxel_beside_a_box_edge():
    # Codex 7e plan round 7 P1-2: a box's side at 40 mm, its front face 0.60 m out; the nominal ray to the voxel
    # crosses the face at 38 mm and is blocked, the camera moved 10 mm sideways crosses it at 43 mm and is not
    box = Box((0.65, 0.0, 0.0), (0.05, 0.040, 0.5))
    voxel = np.array([[1.20, 0.076, 0.0]])  # crosses x = 0.60 at y = 0.038 from the origin
    assert ray_blocked((0.0, 0.0, 0.0), voxel, [box], 0.0)[0]
    assert not ray_blocked((0.0, 0.010, 0.0), voxel, [box], 0.0)[0]
    # with the 10 mm ray error the box shrinks by it: not surely hidden
    assert not ray_blocked((0.0, 0.0, 0.0), voxel, [box], -0.010)[0]


def test_ray_blocked_rules():
    box = Box((1.0, 0.0, 0.0), (0.1, 0.1, 0.1))
    pts = np.array([[2.0, 0.0, 0.0], [0.5, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.5, 0.0]])
    # behind the box: hidden; in front: not; inside it: hidden by its face; off it: not
    assert ray_blocked((0.0, 0.0, 0.0), pts, [box], 0.0).tolist() == [True, False, True, False]
    # grown, a near miss is blocked; shrunk below zero, nothing is
    assert ray_blocked((0.0, 0.0, 0.0), np.array([[2.0, 0.22, 0.0]]), [box], 0.02)[0]
    assert not ray_blocked((0.0, 0.0, 0.0), pts, [box], -0.2).any()
    # the camera inside a box (its own carriage) is not hidden by it
    assert not ray_blocked((1.0, 0.0, 0.0), pts, [box], 0.0).any()


def test_the_pallet_band_counts_visible_and_the_trucks_own_band_hidden():
    rear = (FACE - 0.5 - TIP, 0.0, 0.0)
    exact, banded = make(), make(real=options(pallet_rel_error_m=0.02, own_error_m=0.02))
    v0 = exact._visibility(rear, 0.0, 0.0, 0.0)
    v1 = banded._visibility(rear, 0.0, 0.0, 0.0)
    idx, centres = exact.memory.volume.centres(evaluated=True)
    behind_pallet = centres[:, 0] > FACE + 0.02
    # behind the pallet's face: the band can only turn hidden into visible
    assert (v1 & ~v0 & behind_pallet).any() and not (v0 & ~v1 & behind_pallet).any()
    # around the own blades (ahead of their tips, at their height): the band only turns visible into hidden
    near_tip = (centres[:, 0] < FACE) & (centres[:, 2] < 0.06)
    assert (v0 & ~v1 & near_tip).any()


def test_a_shadowed_voxel_keeps_its_lifetime_and_a_visible_one_needs_a_recent_certification():
    check = make()
    xs = [FACE - 0.8 - TIP + 0.1 * k for k in range(6)]
    approach(check, xs)
    mem = check.memory
    certified = np.isfinite(mem.certified_s)
    # voxels the newest frame expected and answered need a recent certification; out of view or hidden ones not
    assert (mem.need_recent & certified).any() and (~mem.need_recent & certified).any()
    rear = (xs[-1], 0.0, 0.0)
    assert check.limit(0.55, rear, 0.0, 1, 0.1, 0.0)[0] > 0
    # 0.3 s later with no frame: depth_stale anyway; the recent window is frame_max_age - frame_lag
    assert check.limit(0.85, rear, 0.0, 1, 0.1, 0.0)[1] == "depth_stale"


# --- the guard -------------------------------------------------------------------------------------------------------
def test_a_generation_or_correction_change_forgets_before_anything_else():
    state = {"g": (1, ())}
    check = make(real=options(guard=lambda: state["g"]))
    xs = [FACE - 0.9 - TIP + 0.1 * k for k in range(5)]
    approach(check, xs)
    rear = (xs[-1], 0.0, 0.0)
    assert check.limit(0.435, rear, 0.0, 1, 0.3, 0.0)[0] > 0
    state["g"] = (2, ())  # a new depth generation, no frame yet
    assert check.limit(0.44, rear, 0.0, 1, 0.3, 0.0) == (0.0, "depth_stale")
    assert not np.isfinite(check.memory.certified_s).any()
    assert check.records[-1]["forget"] == "guard_changed"
    # a frame of the old generation taken before the boundary is refused
    rec = check.add_frame(0.45, render(check, rear), rear, frame_key=(1, (), 77))
    assert rec["rejected"] == "frame_generation_stale" and rec["new"] is False
    # an applied correction change forgets too
    approach(check, [rear[0]], t0=0.5, n0=200)
    check.add_frame(0.5, render(check, rear), rear, frame_key=(2, (), 300))
    state["g"] = (2, (0.01, 0.0, 0.0))
    check.limit(0.55, rear, 0.0, 1, 0.3, 0.0)
    assert not np.isfinite(check.memory.certified_s).any()
    assert check.boundary_s == 0.55 and check.summary()["boundary_s"] == 0.55


def test_forgetting_near_the_insertion_end_cannot_be_recertified_in_place():
    # Codex 7e plan: forgotten 0.10 m before the insertion end, the next frames see only what is still in view -- the
    # shadowed band under the face stays uncertified
    state = {"g": (1, ())}
    check = make(real=options(guard=lambda: state["g"]))
    xs = list(np.linspace(START, END - 0.10, 40))
    approach(check, xs, dt=0.1)
    t = 40 * 0.1
    rear = (END - 0.10, 0.0, 0.0)
    state["g"] = (2, ())
    for k in range(3):
        check.add_frame(t + 0.1 * k, render(check, rear), rear, frame_key=(2, (), 1000 + k))
        assert check.limit(t + 0.1 * k + 0.035, rear, 0.0, 1, 0.055, 0.0) == (0.0, "pocket_unverified")
