"""Depth-certified insertion volume (plan D5); synthetic ray-cast depth only.

The camera is the carriage mount (0.27 m, pitched down 0.10 rad) on the
insertion axis; the pallet is EPAL 6 seen across its 0.6 m depth, face at
x = -0.30 in the insertion frame. The counterexamples of the Codex L3c
reviews are kept: a far frame cannot certify the strip against the face, a
margin that meets the pallet is refused, a box that slivers into an
uncertified voxel is not contained.
"""

import math

import numpy as np

from forklift_core.perception.insertion_clearance import Box, ClearanceMemory, InsertionVolume
from forklift_core.perception.pallet_geometry import load_pallet_geometry, pallet_boxes
from forklift_core.perception.pocket_clearance import DepthCamera

CAM = DepthCamera(fx=193.0, fy=193.0, cx=160.0, cy=120.0, width=320, height=240, sigma_a=0.0036, sigma_k=3.0)
FACE = -0.30
GEOM = load_pallet_geometry("config/pallet_geometry_epal6.yaml")
SOLIDS = [Box(b.centre_m, tuple(np.asarray(b.size_m) / 2), 0.0) for b in pallet_boxes(GEOM)]
FORK_Y = 0.1445
LAT = 0.0275 + 0.036


def volume(lat=LAT):
    # Insertion 0.36 m plus the real stop at insertion speed (0.02 m with the
    # envelope); a 0.42 m column puts its deepest inner strip in the centre
    # block corner's shadow from a camera on the axis.
    forks = [Box(((FACE - 0.10 + FACE + 0.38) / 2, sy * FORK_Y, 0.045), ((0.48) / 2, lat, 0.027)) for sy in (1, -1)]
    body = Box(((FACE - 0.425 + FACE) / 2, 0.0, 0.095), (0.425 / 2, 0.396, 0.075))
    return InsertionVolume(forks + [body])


def camera_pose(x_cam, pitch=0.10, z_cam=0.27):
    c, s = math.cos(pitch), math.sin(pitch)
    zx = np.array([c, 0.0, -s])
    xx = np.array([0.0, -1.0, 0.0])
    yx = np.cross(zx, xx)
    rot = np.stack([xx, yx, zx])
    pos = np.array([x_cam, 0.0, z_cam])
    return rot, -rot @ pos, pos


def render(x_cam, extra=()):
    """Depth (optical z) of the floor, the pallet solids and any extra boxes."""
    rot, _, pos = camera_pose(x_cam)
    u, w = np.meshgrid(np.arange(CAM.width), np.arange(CAM.height))
    d_opt = np.stack([(u - CAM.cx) / CAM.fx, (w - CAM.cy) / CAM.fy, np.ones_like(u, dtype=float)], axis=-1)
    d = d_opt @ rot  # insertion-frame direction per unit optical z
    t = np.full(u.shape, np.inf)
    floor = np.where(d[..., 2] < -1e-9, -pos[2] / np.where(d[..., 2] < -1e-9, d[..., 2], -1.0), np.inf)
    t = np.minimum(t, np.where(floor > 0, floor, np.inf))
    for b in list(SOLIDS) + list(extra):
        lo = np.subtract(b.center, b.half) - pos
        hi = np.add(b.center, b.half) - pos
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = lo / d
            t2 = hi / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= tmin) & (tmax > 0)
        t = np.where(hit, np.minimum(t, np.maximum(tmin, 0.0)), t)
    return np.where(np.isfinite(t), t, np.nan)


def frame(mem, x_cam, stamp, extra=()):
    rot, trans, _ = camera_pose(x_cam)
    return mem.add_frame(stamp, render(x_cam, extra), CAM, (rot, trans), SOLIDS)


def blade(x_front, sy=1, yaw=0.0):
    return Box((x_front - 0.21, sy * FORK_Y, 0.04), (0.21, 0.0275 + 0.036, 0.012), yaw)


def body(x_front):
    return Box((x_front - 0.2, 0.0, 0.095), (0.2, 0.36 + 0.036, 0.075))


REGION = (FACE - 0.425, 0.30 + 0.425, -0.60, 0.60)


def test_the_margins_fit_the_pockets():
    assert ClearanceMemory(volume(), 20.0).solid_conflict(SOLIDS) == 0


def test_a_margin_that_meets_the_blocks_is_refused():
    assert ClearanceMemory(volume(lat=0.10), 20.0).solid_conflict(SOLIDS) > 0


def test_the_standoff_frame_certifies_the_pockets_but_not_the_strip_at_the_face():
    mem = ClearanceMemory(volume(), 20.0)
    frame(mem, FACE - 2.59, 0.0)
    assert not mem.obstacle
    assert mem.contained(blade(FACE + 0.38), 0.1, REGION)  # blade at full depth
    # At 2.6 m the 3 sigma margin is 7 cm: the strip against the face is not
    # separable from the face yet.
    assert not mem.contained(body(FACE - 0.03), 0.1, REGION)


def test_beyond_the_certified_depth_a_blade_is_not_contained():
    mem = ClearanceMemory(volume(), 20.0)
    frame(mem, FACE - 2.59, 0.0)
    assert not mem.contained(blade(FACE + 0.40), 0.1, REGION)


def test_closer_frames_certify_the_strip_at_the_face():
    mem = ClearanceMemory(volume(), 20.0)
    for k, x in enumerate((2.59, 1.6, 1.0, 0.7, 0.5)):
        frame(mem, FACE - x, 0.1 * k)
    assert mem.contained(body(FACE - 0.03), 0.5, REGION)


def test_a_bar_in_a_pocket_is_an_obstacle():
    mem = ClearanceMemory(volume(), 20.0)
    bar = Box((FACE + 0.20, FORK_Y, 0.04), (0.01, 0.01, 0.04))
    # Far away the bar sits within the 7 cm surface tolerance of the stringer
    # above it; the closer frames separate it.
    for k, x in enumerate((2.59, 1.6, 1.0)):
        frame(mem, FACE - x, 0.1 * k, extra=[bar])
    assert mem.obstacle
    assert not mem.contained(blade(FACE + 0.10), 0.1, REGION)


def test_a_bar_just_in_front_of_the_face_is_an_obstacle():
    mem = ClearanceMemory(volume(), 20.0)
    bar = Box((FACE - 0.05, 0.0, 0.05), (0.01, 0.01, 0.05))
    for k, x in enumerate((2.59, 1.6, 1.0)):
        frame(mem, FACE - x, 0.1 * k, extra=[bar])
    assert mem.obstacle


def test_certification_expires():
    mem = ClearanceMemory(volume(), 20.0)
    frame(mem, FACE - 2.59, 0.0)
    assert mem.contained(blade(FACE + 0.30), 19.9, REGION)
    assert not mem.contained(blade(FACE + 0.30), 20.1, REGION)


def test_a_turned_blade_that_slivers_out_of_its_column_is_not_contained():
    mem = ClearanceMemory(volume(), 20.0)
    frame(mem, FACE - 2.59, 0.0)
    assert mem.contained(blade(FACE + 0.30), 0.1, REGION)
    assert not mem.contained(blade(FACE + 0.30, yaw=math.radians(3.0)), 0.1, REGION)


def test_outside_the_region_the_box_is_not_asked():
    mem = ClearanceMemory(volume(), 20.0)
    far = body(FACE - 1.0)
    assert mem.contained(far, 0.0, REGION)  # wholly before the region: the grid's business
