"""sim/isaac/pocket_check.py on synthetic ray-cast depth (plan D5 L3c).

The truck drives the docking straight on the insertion axis from the
stand-off (fork tips 2.2 m before the face) to the insertion end (tips 0.36 m
inside); a 10 Hz carriage frame is rendered at every pose. The check must let
the clean run through, stop for a bar in a pocket or against the face, stop
when the depth stalls, and refuse a volume that meets the pallet.
"""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

from forklift_core.control.drive_permission import StoppingModel
from forklift_core.perception.insertion_clearance import Box
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pocket_clearance import DepthCamera

_SPEC = importlib.util.spec_from_file_location(
    "pocket_check", Path(__file__).resolve().parents[2] / "sim" / "isaac" / "pocket_check.py"
)
pocket_check = importlib.util.module_from_spec(_SPEC)
sys.modules["pocket_check"] = pocket_check
_SPEC.loader.exec_module(pocket_check)

GEOM = load_pallet_geometry("config/pallet_geometry_epal6.yaml")
CAM = DepthCamera(fx=193.0, fy=193.0, cx=160.0, cy=120.0, width=320, height=240)
OFFSET = 0.34  # rear axle behind base_link
BLADES = ((0.87, 1.29, 0.1175, 0.1725), (0.87, 1.29, -0.1725, -0.1175))
TILT = 0.10


def base_from_optical():
    c, s = math.cos(TILT), math.sin(TILT)
    z = np.array([c, 0.0, -s])  # optical z (forward, pitched down) in base_link
    x = np.array([0.0, -1.0, 0.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z]), np.array([0.559, 0.0, 0.27])


def make(**config):
    return pocket_check.PocketCheck(
        estimate=(0.0, 0.0, 0.0), axis_yaw=0.0, pallet_geometry=GEOM, blades_rear=BLADES,
        body_front_m=0.884, body_rear_m=0.17, body_half_width_m=0.36, insertion_depth_m=0.36, base_z_m=0.0,
        base_from_optical=base_from_optical(), rear_axle_offset_m=OFFSET, camera=CAM,
        stopping=StoppingModel(0.15, 1.5, 0.05), config=pocket_check.PocketConfig(**config),
    )


def render(check, rear, extra=()):
    rot, trans = check.optical_from_insertion(rear)
    pos = -rot.T @ trans
    u, w = np.meshgrid(np.arange(CAM.width), np.arange(CAM.height))
    d_opt = np.stack([(u - CAM.cx) / CAM.fx, (w - CAM.cy) / CAM.fy, np.ones_like(u, dtype=float)], axis=-1)
    d = d_opt @ rot
    t = np.where(d[..., 2] < -1e-9, -pos[2] / np.where(d[..., 2] < -1e-9, d[..., 2], -1.0), np.inf)
    t = np.where(t > 0, t, np.inf)
    for b in list(check.solids) + list(extra):
        lo = np.subtract(b.center, b.half) - pos
        hi = np.add(b.center, b.half) - pos
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = lo / d, hi / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= tmin) & (tmax > 0)
        t = np.where(hit, np.minimum(t, np.maximum(tmin, 0.0)), t)
    return np.where(np.isfinite(t), t, np.nan)


FACE = -0.30
START = FACE - 2.2 - 1.29  # rear axle x at the stand-off
END = FACE + 0.36 - 1.29  # rear axle x at the insertion end


def drive(check, extra=()):
    """10 Hz frames while driving at the permitted speed: 0.3 m/s on the
    straight, 0.055 m/s for the last 0.46 m (the insertion). Returns
    (stop x, reason) or (None, time to the insertion end)."""
    x, t = START, 0.0
    while x < END - 1e-9:
        rear = (x, 0.0, 0.0)
        check.add_frame(t, render(check, rear, extra), rear)
        cruise = 0.055 if x > END - 0.46 else 0.3
        allowed, why = check.limit(t + 0.05, rear, 0.0, 1, cruise, 0.0)
        v = min(cruise, allowed)
        if v <= 0.0:
            return x, why
        x = min(END, x + v * 0.1)
        t += 0.1
        assert t < 60.0
    return None, t


def test_the_volume_fits_the_pockets():
    assert make().conflict == 0


def test_a_volume_that_meets_the_pallet_is_refused():
    check = make(estimate_m=0.08)
    assert check.conflict > 0 and not check.valid


def test_a_clean_pallet_lets_the_truck_reach_the_insertion_end_within_the_lifetime():
    stop, elapsed = drive(make())
    assert stop is None
    # 2.1 m at 0.3 m/s and 0.46 m at 0.055 m/s without acceleration: about
    # 15 s, inside the 20 s lifetime.
    assert 14.0 < elapsed < 20.0


def test_a_blade_lifted_off_the_certified_band_is_not_contained():
    check = make()
    rear = (END - 0.3, 0.0, 0.0)
    for k, x in enumerate((START, END - 1.0, END - 0.3)):
        check.add_frame(0.1 * k, render(check, (x, 0.0, 0.0)), (x, 0.0, 0.0))
    assert check.limit(0.25, rear, 0.0, 1, 0.055, 0.0)[0] > 0
    assert check.limit(0.25, rear, 0.0, 1, 0.055, 0.03)[0] == 0.0


def test_a_bar_in_a_pocket_stops_the_truck_before_the_blade_reaches_it():
    bar = Box((FACE + 0.20, 0.1445, 0.04), (0.01, 0.01, 0.04))
    stop = drive(make(), extra=[bar])
    assert stop[0] is not None and stop[1] == "pocket_obstacle"
    assert stop[0] + 1.29 < FACE + 0.20 - 0.01  # blade tip short of the bar


def test_a_bar_in_front_of_the_face_stops_the_truck_before_the_body_reaches_it():
    bar = Box((FACE - 0.05, 0.0, 0.05), (0.01, 0.01, 0.05))
    stop = drive(make(), extra=[bar])
    assert stop[0] is not None and stop[1] == "pocket_obstacle"
    assert stop[0] + 0.884 < FACE - 0.06


def test_stalled_depth_stops_the_truck():
    check = make()
    rear = (START, 0.0, 0.0)
    depth = render(check, rear)
    check.add_frame(0.0, depth, rear)
    assert check.limit(0.1, rear, 0.0, 1, 0.3, 0.0)[0] > 0
    rec = check.add_frame(0.1, depth, rear)  # the same raw frame again
    assert rec["new"] is False
    assert check.limit(0.25, rear, 0.0, 1, 0.3, 0.0) == (0.0, "depth_stale")


def test_a_return_off_the_estimated_surface_by_the_estimate_error_counts_as_pallet():
    # L3c v6: the front stringer's underside ~2 cm behind its estimated edge,
    # seen from about 1.1 m: within the noise tolerance it would be an
    # obstacle; within noise + estimate error it is the pallet.
    from forklift_core.perception.pocket_clearance import on_surface, surface_tolerance_m

    check = make()
    rot, trans = check.optical_from_insertion((FACE - 1.1 - 0.899, 0.0, 0.0))
    point_i = np.array([FACE + 0.145 + 0.019, 0.12, 0.1])  # stringer x0 ends at FACE + 0.145
    point_o = (point_i @ rot.T + trans)[None, :]
    solids_o = [(np.asarray(b.center) @ rot.T + trans, np.asarray(b.half), rot) for b in check.solids]
    noise_only = surface_tolerance_m(CAM, point_o)
    assert not on_surface(point_o, solids_o, noise_only)[0]
    assert on_surface(point_o, solids_o, noise_only + check.config.surface_estimate_m)[0]


def test_frames_older_than_their_pose_are_tolerated_with_the_speed():
    # L3c v7: at 0.5-0.6 m/s the pixels were 3.5 cm behind the pose they were
    # placed with. Rendered 0.066 s back, placed at the present pose.
    def run(pass_speed):
        check = make()
        x, t, v = START, 0.0, 0.6
        while x < END - 0.6:
            check.add_frame(t, render(check, (x - v * 0.066, 0.0, 0.0)), (x, 0.0, 0.0),
                            speed_mps=v if pass_speed else 0.0)
            x += v * 0.1
            t += 0.1
        return check.memory.obstacle

    assert not run(pass_speed=True)


def test_with_lagged_frames_a_bar_in_a_pocket_still_stops_the_truck_before_the_blade():
    # With the estimate and lag allowances the bar's top (2 cm under the
    # stringer) may read as pallet; the voxels behind it then never certify,
    # and the blade stops short of it all the same.
    check = make()
    bar = Box((FACE + 0.20, 0.1445, 0.04), (0.01, 0.01, 0.04))
    x, t = START, 0.0
    while x < END - 1e-9:
        v_cmd = 0.3 if x < END - 0.46 else 0.055
        check.add_frame(t, render(check, (x - v_cmd * 0.066, 0.0, 0.0), [bar]), (x, 0.0, 0.0), speed_mps=v_cmd)
        allowed, why = check.limit(t + 0.05, (x, 0.0, 0.0), 0.0, 1, v_cmd, 0.0)
        v = min(v_cmd, allowed)
        if v <= 0.0:
            break
        x = min(END, x + v * 0.1)
        t += 0.1
        assert t < 60.0
    assert x + 1.29 < FACE + 0.19  # stopped with the blade tip short of the bar


def test_a_lagged_frame_does_not_certify_an_object_its_old_rays_passed_beside():
    # Codex checkpoint P1: 5 cm of lag, a 2 cm bar; the depth threshold alone
    # let the bar's voxels through.
    check = make()
    bar = Box((FACE - 0.20, 0.30, 0.05), (0.01, 0.01, 0.05))  # in the body band
    x = FACE - 1.2 - 0.884
    lag = 0.05
    check.add_frame(0.0, render(check, (x - lag, 0.0, 0.0), [bar]), (x, 0.0, 0.0), speed_mps=lag / check.config.frame_lag_s)
    vol = check.memory.volume
    i = int((FACE - 0.20 - vol.lo[0]) / vol.voxel_m)
    j = int((0.30 - vol.lo[1]) / vol.voxel_m)
    k = int((0.05 - vol.lo[2]) / vol.voxel_m)
    assert not np.isfinite(check.memory.certified_s[i - 1:i + 2, j - 1:j + 2, k]).any() or \
        check.memory.certified_s[i - 1:i + 2, j - 1:j + 2, k].max() < 0.0


def test_depth_certified_body_band_cells_are_offered_as_memory_evidence():
    from forklift_core.perception.obstacle_grid import FREE, GridSnapshot

    check = make()
    for k, x in enumerate((START, FACE - 1.5 - 0.884, FACE - 1.0 - 0.884, FACE - 0.7 - 0.884)):
        check.add_frame(0.1 * k, render(check, (x, 0.0, 0.0)), (x, 0.0, 0.0))
    snap = GridSnapshot(np.full((60, 40), FREE, dtype=np.uint8), np.zeros((60, 40)), 0.4, (0.0, 0.0, 0.0), 0, 1,
                        {}, -2.0, -1.0, 0.05)
    cells = check.depth_free_cells(snap, 0.4)
    assert cells
    for (i, j) in cells:
        x = -2.0 + (i + 0.5) * 0.05
        assert FACE - 0.60 <= x <= FACE  # only over the body band


def test_depth_support_carries_the_columns_own_observation_time():
    # Codex checkpoint P1: a live stream is not a re-observation of a column.
    from forklift_core.perception.obstacle_grid import FREE, GridSnapshot

    check = make()
    for k, x in enumerate((START, FACE - 1.5 - 0.884, FACE - 1.0 - 0.884)):
        check.add_frame(0.1 * k, render(check, (x, 0.0, 0.0)), (x, 0.0, 0.0))
    snap = GridSnapshot(np.full((60, 40), FREE, dtype=np.uint8), np.zeros((60, 40)), 0.25, (0.0, 0.0, 0.0), 0, 1,
                        {}, -2.0, -1.0, 0.05)
    fresh = check.depth_free_cells(snap, 0.25)
    assert fresh and all(t <= 0.2 for t in fresh.values())
    # Long after, with no new frame certifying them, the columns are no support.
    check.last_new_s = 5.0
    assert check.depth_free_cells(snap, 5.0) == {}


def test_the_trucks_own_blades_in_view_are_not_an_obstacle():
    # L3c v32 seed 4: with the 0.28 m near plane the blade tops came into view.
    check = make()
    rear = (FACE - 0.1 - 1.29, 0.0, 0.0)
    blades = check.own_boxes(check.to_insertion(*rear), 0.0)[:2]
    check.add_frame(0.0, render(check, rear, extra=blades), rear)
    assert not check.memory.obstacle
