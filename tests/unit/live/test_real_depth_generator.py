"""The real D5 pocket check's generator (PR 7 7e-2a, docs/plans/2026-11-26-7e-depth-pocket-check.md 6.1, 6.3, 4.3)."""

import math
import shutil
from pathlib import Path

import numpy as np
import pytest

from forklift_core.control.drive_permission import StoppingModel
from forklift_core.live import real_depth
from forklift_core.live.real_depth import PocketRefused

ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"
FX = 388.81207275390625
INTR = {"serial": "s", "firmware": "f", "stream": "depth", "width": 640, "height": 480, "fx": FX, "fy": FX,
        "ppx": 320.0, "ppy": 240.0, "model": "brown_conrady", "coeffs": [0.0] * 5}


def bfo():
    c, s = math.cos(0.1), math.sin(0.1)
    z = np.array([c, 0.0, -s])
    x = np.array([0.0, -1.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z]), np.array([0.619, 0.0, 0.27])


def pocket(**err):
    e = {"operator_m": 0.01, "slam_m": 0.005, "own_m": 0.002, "cam_trans_m": 0.003, "cam_rot_deg": 0.3}
    e.update(err)
    return {"sigma_a": 0.0058, "surface_estimate_m": 0.02, "lifetime_s": 20.0, "view_min_px": 2000,
            "view_valid_min": 0.5, "error": e}


@pytest.fixture
def geom_dir(tmp_path):
    shutil.copy(ROOT / "config/pallet_geometry_epal6.yaml", tmp_path / "pallet_geometry_epal6.yaml")
    return tmp_path


def build(geom_dir, *, pallet=None, intr=None, pk=None, guard=None):
    return real_depth.make_pocket_check(
        estimate=(2.0, 0.5, 0.1), axis_yaw=0.1, pallet=pallet or {"kind": "epal6", "depth_m": 0.6, "width_m": 0.8},
        urdf=URDF, intrinsics=intr or INTR, base_from_optical=bfo(), stopping=StoppingModel(0.30, 0.5, 0.05),
        pocket=pk or pocket(), distortion_max_px=0.5, guard=guard, geometry_dir=geom_dir)


def test_the_generator_builds_the_isaac_check_with_the_real_options(geom_dir):
    check = build(geom_dir)
    assert check.valid and check.real is not None and check.synthetic_noise is False
    # the error budget: operator + SLAM for the pallet, the camera's angle in radians
    assert check.real.pallet_rel_error_m == pytest.approx(0.015)
    assert check.real.cam_rot_error_rad == pytest.approx(math.radians(0.3))
    assert check.camera.sigma_a == 0.0058 and (check.camera.width, check.camera.height) == (640, 480)
    assert check.config.lifetime_s == 20.0 and check.config.surface_estimate_m == 0.02
    # the chassis model: the tyres stand on the floor, the rear axle 0.34 m behind base_link, the 330 mm insertion
    assert check.base_z_m == pytest.approx(0.0)
    assert check.rear_axle_offset_m == pytest.approx(0.34)
    assert check.insertion_depth_m == pytest.approx(0.33)
    assert check.summary()["depth"] == "real D435i depth"


def test_the_own_boxes_are_every_fork_carriage_collision_box_of_the_urdf(geom_dir):
    check = build(geom_dir)
    boxes = check.real.own_boxes_rear
    assert len(boxes) == 14  # 2 blades, 2 heels, 2 carriage sides, 3 cross bars, 5 grille bars
    # the left blade (dls08_measured forklift.urdf left_fork_collision: 0.36 x 0.055 x 0.025 at z 0.04), rear frame
    blade = [b for b in boxes if b[4] == pytest.approx(0.0275) and b[2] > 0]
    assert len(blade) == 1
    x0, x1, y0, y1, z0, z1 = blade[0]
    assert (x1 - x0, y1 - y0, z1 - z0) == pytest.approx((0.36, 0.055, 0.025))
    assert x1 == pytest.approx(max(b[1] for b in check.blades_rear))  # the same tip as the blades the check drives
    # carried up by the lift
    lifted = check.own_boxes(check.to_insertion(-1.0, 0.0, 0.1), 0.05)
    lowered = check.own_boxes(check.to_insertion(-1.0, 0.0, 0.1), 0.0)
    assert all(a.center[2] - b.center[2] == pytest.approx(0.05) for a, b in zip(lifted, lowered))


def test_a_pallet_without_its_shape_file_or_dimensions_is_refused(geom_dir):
    with pytest.raises(PocketRefused) as e:
        build(geom_dir, pallet={"kind": "s10_pallet", "depth_m": 0.6, "width_m": 0.8})
    assert e.value.reason == "pallet_geometry_unset"
    with pytest.raises(PocketRefused) as e:
        build(geom_dir, pallet={"kind": "epal6", "depth_m": None, "width_m": 0.8})
    assert e.value.reason == "pallet_geometry_unset"


def test_outer_dimensions_more_than_a_millimetre_off_are_a_mismatch(geom_dir):
    build(geom_dir, pallet={"kind": "epal6", "depth_m": 0.6009, "width_m": 0.8})
    with pytest.raises(PocketRefused) as e:
        build(geom_dir, pallet={"kind": "epal6", "depth_m": 0.602, "width_m": 0.8})
    assert e.value.reason == "pallet_geometry_mismatch"


def test_a_budget_term_still_null_refuses(geom_dir):
    for term in ("operator_m", "slam_m", "cam_trans_m", "cam_rot_deg"):
        with pytest.raises(PocketRefused) as e:
            build(geom_dir, pk=pocket(**{term: None}))
        assert e.value.reason == f"pocket_budget_unset:{term}"


def test_the_shipped_depth_config_refuses_until_the_bench_enters_the_budget(geom_dir):
    from forklift_core.sensors.depth_cfg import load_depth_config

    with pytest.raises(PocketRefused) as e:
        build(geom_dir, pk=load_depth_config().pocket)
    assert e.value.reason.startswith("pocket_budget_unset:")


def test_distortion_is_measured_in_pixels_and_refused_over_the_limit(geom_dir):
    assert real_depth.distortion_shift_px(INTR) == 0.0
    small = dict(INTR, coeffs=[1e-4, 0.0, 0.0, 0.0, 0.0])
    big = dict(INTR, coeffs=[0.01, 0.0, 0.0, 0.0, 0.0])
    # forward Brown-Conrady at the corner: r^2 = (320^2 + 240^2) / fx^2, shift = fx * r * k1 r^2
    r = math.hypot(320, 240) / FX
    assert real_depth.distortion_shift_px(big) == pytest.approx(FX * r * 0.01 * r * r, rel=1e-6)
    build(geom_dir, intr=small)
    with pytest.raises(PocketRefused) as e:
        build(geom_dir, intr=big)
    assert e.value.reason == "depth_distortion"
    with pytest.raises(PocketRefused) as e:
        build(geom_dir, intr=dict(INTR, model="kannala_brandt4", coeffs=[0.1, 0, 0, 0, 0]))
    assert e.value.reason == "depth_distortion_model"
    # an unknown model with zero coefficients is a pinhole
    build(geom_dir, intr=dict(INTR, model="distortion.none"))


def test_the_depth_config_pocket_section_is_checked(tmp_path):
    import yaml

    from forklift_core.sensors.depth_cfg import DEPTH_YAML, load_depth_config

    raw = yaml.safe_load(DEPTH_YAML.read_text())
    for bad in ({"pocket": {**raw["pocket"], "extra": 1}},
                {"pocket": {**raw["pocket"], "error": {**raw["pocket"]["error"], "own_m": -0.001}}},
                {"pocket": {**raw["pocket"], "view_valid_min": 1.5}},
                {"pocket": {**raw["pocket"], "sigma_a": 0}}):
        p = tmp_path / "d.yaml"
        p.write_text(yaml.safe_dump({**raw, **bad}))
        with pytest.raises(ValueError):
            load_depth_config(p)


def test_two_checks_on_the_same_input_agree_without_synthetic_noise(geom_dir):
    a, b = build(geom_dir), build(geom_dir)
    rear = (0.3, 0.38, 0.1)  # about 1.5 m before the face on the axis
    rng = np.random.default_rng(3)
    depth = rng.uniform(0.5, 3.0, (480, 640))
    ra = a.add_frame(1.0, depth, rear, frame_key=(1, (), 7))
    rb = b.add_frame(1.0, depth, rear, frame_key=(1, (), 7))
    assert {k: ra[k] for k in ("free", "refuted", "view_px", "view_valid_px")} == \
        {k: rb[k] for k in ("free", "refuted", "view_px", "view_valid_px")}
    assert np.array_equal(a.memory.certified_s, b.memory.certified_s)
