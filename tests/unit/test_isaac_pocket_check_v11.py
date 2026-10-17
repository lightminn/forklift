"""sim/isaac/pocket_check_v11.py on synthetic ray-cast depth (D5 v11 plan).

The truck drives the docking straight on the axis from the stand-off (fork tips
2.2 m before the face) to the insertion end with the carriage 16 mm before the
face; a 10 Hz carriage frame is rendered at every pose, each with the true
observation (the two pocket front centres) and a fixed observation bound. The
pallet frame has its origin at the face between the pockets, x into the pallet.
"""

import importlib.util
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.drive_permission import StoppingModel
from forklift_core.perception.insertion_clearance import Box
from forklift_core.perception.near_field_bounds import NearFieldBounds
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pocket_clearance import DepthCamera

ROOT = Path(__file__).resolve().parents[2]
_SPEC = importlib.util.spec_from_file_location("pocket_check_v11", ROOT / "sim" / "isaac" / "pocket_check_v11.py")
V11 = importlib.util.module_from_spec(_SPEC)
sys.modules["pocket_check_v11"] = V11
_SPEC.loader.exec_module(V11)

GEOM = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
BOUNDS_DATA = json.loads((ROOT / "config/near_field_bounds_measured.json").read_text())
BOUNDS = NearFieldBounds.from_dict(BOUNDS_DATA)
CAM = DepthCamera(fx=193.0, fy=193.0, cx=160.0, cy=120.0, width=320, height=240)
OFFSET = 0.34  # rear axle behind base_link
BLADES = ((0.884, 1.29, 0.1175, 0.1725), (0.884, 1.29, -0.1725, -0.1175))  # rear-axle frame, synthetic 0.29 m spacing
BODY_FRONT = 0.884  # carriage face from the rear axle
TILT = 0.10
INSERTION = 1.29 - BODY_FRONT - 0.016  # fork tips in when the carriage stands 16 mm before the face
BOUND = {"lateral_m": 0.004, "along_m": 0.008, "yaw_rad": 0.002, "wall_m": 0.004, "width_m": 0.004}
HANDOFF_M = 0.5  # front-face observations from this camera-face distance on, roof below (as the tracker)


def measured_bound(base_x):
    """The measured observation bound at the camera's distance to the face."""
    d = -(base_x + base_from_optical()[1][0])
    return BOUNDS.observation("front" if d >= HANDOFF_M else "roof", max(d, 0.3))


def base_from_optical():
    c, s = math.cos(TILT), math.sin(TILT)
    z = np.array([c, 0.0, -s])
    x = np.array([0.0, -1.0, 0.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z]), np.array([0.559, 0.0, 0.27])


def make(**config):
    return V11.PalletPocketCheck(
        pallet_geometry=GEOM, blades_rear=BLADES, body_front_m=BODY_FRONT, body_rear_m=0.17,
        body_half_width_m=0.36, insertion_depth_m=INSERTION, base_z_m=0.0, base_from_optical=base_from_optical(),
        rear_axle_offset_m=OFFSET, camera=CAM, stopping=StoppingModel(0.15, 1.5, 0.05), bounds=BOUNDS,
        config=V11.V11Config(**config), latency_band_s=tuple(BOUNDS_DATA["latency"]["band_s"]),
    )


def pocket_centres():
    ys, widths = GEOM.block_centres_y_m(), GEOM.block_widths_m
    out = []
    for k in range(len(ys) - 1):
        lo, hi = ys[k] + widths[k] / 2, ys[k + 1] - widths[k + 1] / 2
        out.append((lo + hi) / 2)
    return out


def observation(base_p, face_offset_m=0.0):
    """The true observation at a base pose in the pallet frame (face_offset_m: the observed
    face this much nearer the truck than the real one)."""
    x, y, yaw = base_p
    c, s = math.cos(yaw), math.sin(yaw)
    pockets = []
    for yp in sorted(pocket_centres(), reverse=True):  # left (+y) first
        dx, dy = -face_offset_m - x, yp - y
        pockets.append(SimpleNamespace(center_m=(dx * c + dy * s, -dx * s + dy * c, GEOM.opening_centre_height_m)))
    return SimpleNamespace(left=pockets[0], right=pockets[1], insertion_yaw_rad=-yaw)


def render(check, base_p, extra=()):
    """Ray-cast depth in the real pallet frame from the camera at base pose base_p."""
    rot, trans = check.optical_from_pallet(observation(base_p), 0.0)
    pos = -rot.T @ trans
    u, w = np.meshgrid(np.arange(CAM.width), np.arange(CAM.height))
    d = np.stack([(u - CAM.cx) / CAM.fx, (w - CAM.cy) / CAM.fy, np.ones_like(u, dtype=float)], axis=-1) @ rot
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


START = -2.2 - 1.29  # rear axle x at the stand-off
END = INSERTION - 1.29  # rear axle x at the insertion end


def drive(check, extra=(), face_offset_m=0.0, bound=None):
    """10 Hz frames while driving: 0.3 m/s on the straight, 0.055 m/s for the last 0.46 m; each
    frame's bound from the measured table at its distance (or a fixed one). Returns (reached the
    insertion end, the last reason, rear x reached)."""
    t, x, last, latest = 0.0, START, None, None
    dt = 1.0 / 120
    next_frame = 0.0
    while x < END - 1e-9:
        speed = 0.055 if x > END - 0.46 else 0.3
        if t >= next_frame - 1e-12:
            base = (x + OFFSET, 0.0, 0.0)
            # The first frame is the near capture (its own bound); later ones are accepted only
            # where the table bounds them, else they are obstacle checks only.
            b = bound or (dict(BOUNDS.near_capture) if latest is None else measured_bound(base[0]))
            if b is not None and not b.get("unbounded"):
                check.add_frame(t, render(check, base, extra), observation(base, face_offset_m), b, 0.0,
                                motion=(speed, 0.0, 3.0))
                latest, last = b, t
            else:
                check.add_frame(t, render(check, base, extra), None, None, 0.0, truck_pallet=base,
                                carried=(latest, last))
            next_frame += 0.1
        allowed, why = check.limit(t + dt, (x, 0.0, 0.0), latest, last, 0.0, 1, speed, 0.0)
        if allowed < speed:
            return False, why, x
        x += speed * dt
        t += dt
    return True, "ok", x


def test_the_pocket_volume_is_the_two_pocket_interiors_and_the_body_band():
    check = make()
    v = check.volume
    idx, centres = v.centres()
    pockets = centres[centres[:, 0] > 0.0]
    walls = sorted(pocket_centres())
    assert len(pockets) and np.all(np.abs(pockets[:, 1]) >= 0.0725) and np.all(np.abs(pockets[:, 1]) <= 0.300)
    assert pockets[:, 0].max() <= INSERTION + 0.05 + 0.01
    body = centres[centres[:, 0] < -0.1]
    assert body[:, 1].max() >= 0.36 + 0.026 + 0.05 - 0.01
    assert walls[0] < 0 < walls[1]


def test_a_clean_run_reaches_the_insertion_end_with_the_body_cut_at_the_face():
    check = make()
    reached, why, x = drive(check)
    assert reached, (why, x, check.summary()["last_fail"])
    s = check.summary()
    assert s["rules"] == "d5_v11" and s["accepted_frames"] > 30 and not s["obstacle"]


def test_without_the_face_cut_the_standing_insertion_end_is_refused():
    check = make(face_cut_m=0.0)
    reached, why, x = drive(check)
    assert not reached and why == "pocket_unverified" and x > END - 0.03, (why, x)


def test_a_bar_in_a_pocket_stops_the_truck_before_contact():
    y = sorted(pocket_centres())[1]
    bar = Box((0.15, y, 0.05), (0.01, 0.01, 0.02))
    check = make()
    reached, why, x = drive(check, extra=[bar])
    assert not reached and why in ("pocket_obstacle", "pocket_unverified")
    assert x + 1.29 < 0.15 - 0.01, x  # the fork tips stopped short of the bar


def test_an_eighteen_millimetre_plate_on_the_face_stops_the_truck():
    plate = Box((-0.009, 0.0, 0.06), (0.009, 0.39, 0.03))  # 18 mm thick, on the face, low
    check = make()
    reached, why, x = drive(check, extra=[plate])
    assert not reached
    assert x + BODY_FRONT < -0.018, x  # the carriage stood before the plate


def test_an_invalidation_drops_every_certification():
    check = make()
    drive(check)
    assert check.certified.any()
    check.invalidate("mismatch", 100.0)
    assert not check.certified.any() and check.events[-1]["reason"] == "mismatch"


def test_an_age_beyond_the_odometry_table_is_unbounded():
    check = make()
    base = (START + OFFSET, 0.0, 0.0)
    check.add_frame(0.0, render(check, base), observation(base), BOUND, 0.0)
    allowed, why = check.limit(0.05, (START, 0.0, 0.0), BOUND, -1e6, 0.0, 1, 0.3, 0.0)
    assert allowed == 0.0 and why == "pocket_unbounded"


def test_a_frame_without_an_accepted_observation_certifies_nothing():
    check = make()
    base = (START + OFFSET, 0.0, 0.0)
    rec = check.add_frame(0.0, render(check, base), None, None, 0.0, truck_pallet=base)
    assert rec["accepted"] is False and not check.certified.any()


def test_the_erosion_is_per_axis_and_at_least_one_voxel():
    check = make()
    e_x, e_y = check.frame_errors({"lateral_m": 0.0, "along_m": 0.0, "yaw_rad": 0.0}, (0.0, 0.0, 0.0))
    assert e_x == pytest.approx(V11.PALLET_TERM_M) and e_y == pytest.approx(V11.PALLET_TERM_M)
    base = (START + OFFSET, 0.0, 0.0)
    rec = check.add_frame(0.0, render(check, base), observation(base),
                          {"lateral_m": 0.0, "along_m": 0.025, "yaw_rad": 0.0}, 0.0)
    assert rec["k"][0] >= 3 and rec["k"][1] == 1


def test_box_filter_xy_erodes_each_axis_by_its_own_width():
    grid = np.zeros((9, 9, 1), dtype=bool)
    grid[2:7, 1:8, 0] = True
    out = V11.box_filter_xy(grid, 1, 2)
    assert out[3:6, 3:6, 0].all() and out.sum() == 3 * 3


def test_a_blade_box_on_the_scope_floor_does_not_leave_the_grid_by_rounding():
    # At a lift of -1e-5 the blades' centre - half is 0.03 - 5e-18 (replay seed_8 none
    # t = 74.75): height is not asked, x and y carry a 1e-9 tolerance.
    check = make()
    boxes, _ = check.truck_boxes((-1.66, 0.0, 0.0), -1e-5, BOUND, (0.0, 0.0))
    assert all(b.center[2] - b.half[2] < V11.SCOPE_FLOOR_M for b in boxes[1:])
    assert not any(check._box_leaves_grid(b) for b in boxes[1:])
    top_y = check.volume.lo[1] + check.volume.shape[1] * check.volume.voxel_m
    assert check._box_leaves_grid(Box((0.0, top_y + 0.02, 0.05), (0.05, 0.05, 0.02), 0.0))


def test_a_non_finite_bound_or_pose_is_unbounded_not_free():
    check = make()
    base = (START + OFFSET, 0.0, 0.0)
    check.add_frame(0.0, render(check, base), observation(base), BOUND, 0.0)
    nan_bound = dict(BOUND, yaw_rad=float("nan"))
    assert check.limit(0.05, (-0.9, 0.0, 0.0), nan_bound, 0.0, 0.0, 1, 0.05, 0.0) == (0.0, "pocket_unbounded")
    assert check.limit(0.05, (float("nan"), 0.0, 0.0), BOUND, 0.0, 0.0, 1, 0.05, 0.0) == (0.0, "pocket_unbounded")
    rec = check.add_frame(0.1, render(check, base), observation(base), nan_bound, 0.0)
    assert rec["accepted"] is False and "certified" not in rec


def test_the_demand_error_is_taken_at_each_box_corner():
    # rear axle 1 m before the face, lateral 4 mm, yaw 2 mrad, no odometry: the body's rear
    # corners sit at x = -1.17, so e_d,y = 0.004 + 1.17 * 0.002 + the pallet term (V3).
    check = make()
    bound = {"lateral_m": 0.004, "along_m": 0.0, "yaw_rad": 0.002}
    _, errors = check.truck_boxes((-1.0, 0.0, 0.0), 0.0, bound, (0.0, 0.0))
    assert errors[0][1] == pytest.approx(0.004 + 1.17 * 0.002 + V11.PALLET_TERM_M)
    assert errors[0][0] == pytest.approx(0.36 * 0.002 + V11.PALLET_TERM_M)


def test_an_obstacle_latch_drops_the_store_and_certifies_nothing():
    check = make()
    base = (-2.0, 0.0, 0.0)
    bar = Box((0.15, pocket_centres()[0], 0.05), (0.02, 0.02, 0.02))
    rec = check.add_frame(0.0, render(check, base, extra=[bar]), observation(base), BOUND, 0.0)
    assert rec["obstacle"] > 0 and check.obstacle and "certified" not in rec
    assert not check.certified.any() and check.events[-1]["reason"] == "obstacle"


def test_identical_pixels_at_a_new_stamp_are_a_new_frame():
    check = make()
    base = (START + OFFSET, 0.0, 0.0)
    depth = render(check, base)
    check.add_frame(0.0, depth, None, None, 0.0)
    rec = check.add_frame(0.1, depth, observation(base), BOUND, 0.0)
    assert rec["new"] and rec["certified"] > 0
    assert check.add_frame(0.1, depth, observation(base), BOUND, 0.0)["new"] is False


def test_a_frame_placed_by_the_carried_estimate_allows_that_estimates_error():
    # The truck is truly at base, 0.34 m camera-to-face, but the carried estimate puts it 35 mm
    # farther: beyond kσ + the surface share 0.02 + the odometry share alone (a zero bound
    # latches), within it + the bound's along 8 mm (V5 with the carried e_d). Without a carried
    # bound the frame cannot be placed.
    base = (-0.9, 0.0, 0.0)
    off = (base[0] - 0.035, 0.0, 0.0)
    check = make()
    depth = render(check, base)
    bound = {"lateral_m": 0.004, "along_m": 0.008, "yaw_rad": 0.002}
    rec = check.add_frame(0.0, depth, None, None, 0.0, truck_pallet=off, carried=(bound, -0.05))
    assert rec["obstacle"] == 0 and not check.obstacle
    assert check.add_frame(0.1, depth, None, None, 0.0, truck_pallet=off)["placed"] is False
    zero = make()
    rec = zero.add_frame(0.0, depth, None, None, 0.0, truck_pallet=off,
                         carried=({"lateral_m": 0.0, "along_m": 0.0, "yaw_rad": 0.0}, -0.05))
    assert rec["obstacle"] > 0


def test_the_erosion_width_is_the_largest_error_over_v_not_its_halo():
    check = make()
    base = (START + OFFSET, 0.0, 0.0)
    rec = check.add_frame(0.0, render(check, base), observation(base),
                          {"lateral_m": 0.007, "along_m": 0.0, "yaw_rad": 0.02}, 0.0)
    e_x, e_y = rec["e_c"]
    assert e_y == pytest.approx(0.007 + 0.60 * 0.02 + V11.PALLET_TERM_M)
    assert rec["k"] == [math.ceil(e_x / 0.01), 2]


def test_only_a_placed_frame_keeps_the_depth_fresh():
    check = make()
    base = (START + OFFSET, 0.0, 0.0)
    check.add_frame(0.0, render(check, base), observation(base), BOUND, 0.0)
    rec = check.add_frame(0.1, render(check, base), None, None, 0.0)  # no observation, nothing to place it by
    assert rec["new"] and rec["placed"] is False
    allowed, why = check.limit(0.15, (START, 0.0, 0.0), BOUND, 0.0, 0.0, 1, 0.0, 0.0)
    assert why == "depth_stale" and allowed == 0.0


def test_a_frame_placed_by_the_anchor_while_standing_allows_the_residual_motion_bound():
    # V13: the same 35 mm carried offset that latches with a zero bound is allowed once the
    # standing bound d0 = 10 mm is added (d0 + rho * phi0 to both axes).
    base = (-0.9, 0.0, 0.0)
    off = (base[0] - 0.035, 0.0, 0.0)
    zero = {"lateral_m": 0.0, "along_m": 0.0, "yaw_rad": 0.0}
    check = make()
    depth = render(check, base)
    assert check.add_frame(0.0, depth, None, None, 0.0, truck_pallet=off, carried=(zero, -0.05))["obstacle"] > 0
    check = make()
    rec = check.add_frame(0.0, depth, None, None, 0.0, truck_pallet=off, carried=(zero, -0.05), standing=(0.010, 0.0))
    assert rec["obstacle"] == 0 and not check.obstacle
    bad = make().add_frame(0.0, depth, None, None, 0.0, truck_pallet=off, carried=(zero, -0.05),
                           standing=(float("nan"), 0.0))
    assert bad["placed"] is False


def _urdf(tmp_path, extra_base="", carriage_axis="0 0 1", wheel_geometry="", nested=False, wheel_child="",
          carriage_extra="", lift_parent="base_link"):
    """A minimal truck: a body box, two steered front wheels, a carriage with two blades."""
    wheel = wheel_geometry or '<cylinder radius="0.1" length="0.08" />'
    carrier = "carrier_l"
    nest = ('<joint name="nested" type="revolute"><parent link="carrier_l" /><child link="carrier_n" />'
            '<axis xyz="0 0 1" /><limit lower="-0.2" upper="0.2" /></joint><link name="carrier_n" />') if nested else ""
    if nested:
        carrier = "carrier_n"
    text = f"""<robot name="t">
<link name="base_link">
  <collision name="body"><origin xyz="0 0 0.09" /><geometry><box size="0.6 0.5 0.12" /></geometry></collision>
  {extra_base}
</link>
<joint name="steer_l" type="revolute"><parent link="base_link" /><child link="carrier_l" />
  <origin xyz="0.3 0.2 0.1" /><axis xyz="0 0 1" /><limit lower="-0.2" upper="0.2" /></joint>
<link name="carrier_l" />
{nest}
<joint name="spin_l" type="continuous"><parent link="{carrier}" /><child link="wheel_l" /><axis xyz="0 1 0" /></joint>
<link name="wheel_l"><collision name="tire"><origin rpy="1.570796327 0 0" /><geometry>{wheel}</geometry></collision></link>
{wheel_child}
<joint name="lift" type="prismatic"><parent link="{lift_parent}" /><child link="carriage" />
  <axis xyz="{carriage_axis}" /><limit lower="0" upper="0.28" /></joint>
<link name="carriage">
  <collision name="face"><origin xyz="0.5 0 0.1" /><geometry><box size="0.02 0.4 0.12" /></geometry></collision>
  <collision name="left_fork_collision"><origin xyz="0.7 0.15 0.04" /><geometry><box size="0.36 0.05 0.02" /></geometry></collision>
  <collision name="right_fork_collision"><origin xyz="0.7 -0.15 0.04" /><geometry><box size="0.36 0.05 0.02" /></geometry></collision>
  {carriage_extra}
</link>
</robot>"""
    path = tmp_path / "t.urdf"
    path.write_text(text)
    return path


def test_the_truck_split_refuses_what_it_cannot_cover(tmp_path):
    # Rear axle at base_link x = -0.3; body half-width 0.36 (the wheels reach 0.24 + 0.04 = 0.28).
    split = V11.truck_split_from_urdf(_urdf(tmp_path), 0.3, 0.6, 0.36, 0.81, 0.2)
    assert split["carriage_half_width_m"] == 0.2 and split["chassis_front_m"] > 0.3 + 0.4
    # A box rotated 45 degrees centred on the carriage half-width (base x 0.43, y 0.2): its corners
    # wider than 0.2 sit at x 0.43, but its edge stays wider than 0.2 up to x 0.43 + 0.0495 (Codex
    # S0b P2: a corner test would put the front at 0.43 and leave that edge outside both boxes).
    corner = ('<collision name="diamond"><origin xyz="0.43 0.2 0.1" rpy="0 0 0.785398" />'
              '<geometry><box size="0.07 0.07 0.05" /></geometry></collision>')
    split = V11.truck_split_from_urdf(_urdf(tmp_path, extra_base=corner), 0.3, 0.6, 0.36, 0.81, 0.2)
    assert split["chassis_front_m"] >= 0.3 + 0.43 + 0.07 / math.sqrt(2) - 1e-6
    with pytest.raises(ValueError):  # a body box narrower than the steered wheels
        V11.truck_split_from_urdf(_urdf(tmp_path), 0.3, 0.6, 0.25, 0.81, 0.2)
    hub_box = ('<joint name="hub" type="fixed"><parent link="wheel_l" /><child link="hub_l" /></joint>'
               '<link name="hub_l"><collision name="flag"><origin xyz="0.3 0 0" /><geometry><box size="0.05 0.05 0.05" />'
               '</geometry></collision></link>')
    for kwargs in ({"carriage_axis": "0 1 0"},  # a lift that moves sideways
                   {"carriage_axis": "0 0 -1"},  # a lift that moves down
                   {"wheel_geometry": '<box size="0.2 0.08 0.2" />'},  # a spinning box
                   {"wheel_child": hub_box},  # a box fixed under a spinning wheel
                   {"nested": True},  # a revolute under a revolute
                   {"lift_parent": "carrier_l"},  # a lift under a steered link
                   # a wide plate below the band at lift 0 that the lift raises into it, ahead of the face
                   {"carriage_extra": '<collision name="plate"><origin xyz="0.6 0 0.015" />'
                                      '<geometry><box size="0.05 0.6 0.02" /></geometry></collision>'}):
        with pytest.raises(ValueError):
            V11.truck_split_from_urdf(_urdf(tmp_path, **kwargs), 0.3, 0.6, 0.36, 0.81, 0.2)


def test_the_truck_split_front_holds_the_finely_sampled_steering_envelope():
    # 61 samples with each step's arcs circumscribed cover what 2001 plain samples find.
    urdf = ROOT / "sim/models/dls08_measured/forklift.urdf"
    coarse = V11.truck_split_from_urdf(urdf, 0.34, 0.17, 0.36, 0.944, 0.235)
    fine = V11.truck_split_from_urdf(urdf, 0.34, 0.17, 0.36, 0.944, 0.235, steer_samples=2001)
    assert coarse["chassis_front_m"] >= fine["chassis_front_m"] - 1e-9
