"""URDF box clearances using actual full body poses, without an Isaac SDK."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.pallet_geometry import load_pallet_geometry

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "insertion_geometry", ROOT / "sim/isaac/insertion_geometry.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def geometry():
    return MODULE.InsertionGeometry.from_urdfs(
        ROOT / "sim/models/dls08_provisional/forklift.urdf",
        ROOT / "sim/models/epal6_pallet/pallet.urdf",
    )


def check(geometry, base=(-0.89, 0, 0), q=(1, 0, 0, 0), lift=0, **kwargs):
    return geometry.forbidden_contacts(base, q, lift, (0, 0, 0), (1, 0, 0, 0), **kwargs)


def test_aligned_insertion_clears_all_source_boxes(geometry):
    assert len(geometry.pallet_boxes) == 22
    assert len(geometry.fork_boxes) == 4
    for x in np.linspace(-1.35, -0.89, 20):
        assert not check(geometry, base=(x, 0, 0))


def test_side_scrape_is_detected_without_any_pallet_displacement(geometry):
    assert check(geometry, base=(-0.89, 0.05, 0))


def test_lift_into_top_boards_is_forbidden_before_lift_phase(geometry):
    assert check(geometry, lift=0.055)


def test_pitch_and_roll_affect_clearance(geometry):
    angle = 0.12
    assert check(geometry, q=(np.cos(angle / 2), 0, -np.sin(angle / 2), 0))
    assert not check(geometry, q=(np.cos(angle / 2), np.sin(angle / 2), 0, 0))
    assert check(geometry, q=(np.cos(angle / 2), np.sin(angle / 2), 0, 0), lift=0.04)


def test_relative_yaw_can_scrape_stationary_pallet(geometry):
    for angle, expected in [(0.04, False), (0.10, True)]:
        assert (
            bool(check(geometry, q=(np.cos(angle / 2), 0, 0, np.sin(angle / 2))))
            == expected
        )


def test_exact_touch_is_rejected_at_zero_margin(geometry):
    assert check(geometry, base=(-0.89, 0.045, 0), clearance_m=0)


def test_common_world_rotation_and_translation_preserve_result(geometry):
    q = np.array([0.9, 0.1, 0.2, 0.3])
    q /= np.linalg.norm(q)
    from forklift_core.geometry import rotation_matrix_from_quaternion_xyzw

    rotation = rotation_matrix_from_quaternion_xyzw(q[[1, 2, 3, 0]])
    translation = np.array([2, -3, 1])
    for offset in [0, 0.05]:
        result = geometry.forbidden_contacts(
            rotation @ [-0.89, offset, 0] + translation,
            q,
            0,
            translation,
            q,
        )
        assert result == check(geometry, base=(-0.89, offset, 0))


def test_clearance_margin_rejects_near_scrape(geometry):
    # Inner fork edge is at -73 mm; centre block ends at -72.5 mm.
    assert not check(geometry, base=(-0.89, 0.0445, 0), clearance_m=0)
    assert check(geometry, base=(-0.89, 0.0445, 0), clearance_m=0.002)


@pytest.mark.parametrize("kwargs", [{"clearance_m": -1}, {"lift": np.nan}])
def test_invalid_geometry_state_is_rejected(geometry, kwargs):
    with pytest.raises(ValueError):
        check(geometry, **kwargs)


def test_chassis_reference_is_axle_relative():
    assert MODULE.read_chassis_reference_m(
        ROOT / "sim/models/dls08_provisional/forklift.urdf"
    ) == pytest.approx((1.29, -0.34))


@pytest.mark.parametrize("joint_name", ["left_fork_tip_fixed", "rear_left_spin"])
@pytest.mark.parametrize(
    "defect",
    ["missing_joint", "missing_origin", "missing_xyz", "short_xyz", "nan", "text"],
)
def test_chassis_reference_rejects_missing_or_malformed(tmp_path, joint_name, defect):
    import xml.etree.ElementTree as ET

    tree = ET.parse(ROOT / "sim/models/dls08_provisional/forklift.urdf")
    root = tree.getroot()
    joint = root.find(f"joint[@name='{joint_name}']")
    origin = joint.find("origin")
    if defect == "missing_joint":
        root.remove(joint)
    elif defect == "missing_origin":
        joint.remove(origin)
    elif defect == "missing_xyz":
        origin.attrib.pop("xyz")
    else:
        origin.set(
            "xyz", {"short_xyz": "1 2", "nan": "nan 0 0", "text": "1 2 bad"}[defect]
        )
    path = tmp_path / "truck.urdf"
    tree.write(path)
    with pytest.raises(ValueError):
        MODULE.read_chassis_reference_m(path)


def test_epal_urdf_envelope_matches_yaml_dimensions():
    path = ROOT / "sim/models/epal6_pallet/pallet.urdf"
    assert len(MODULE.pallet_boxes_from_urdf(path)) == 22
    MODULE.assert_pallet_urdf_matches_geometry(path, 0.60, 0.80)


@pytest.mark.parametrize(
    "kwargs", [{"x_m": 0.03}, {"z_m": 0.075}, {"depth_m": 0.60}, {"width_m": 0.80}]
)
def test_pallet_envelope_rejects_dimensions_or_origin(synthetic_pallet_urdf, kwargs):
    with pytest.raises(ValueError, match="envelope"):
        MODULE.assert_pallet_urdf_matches_geometry(
            synthetic_pallet_urdf(**kwargs), 0.66, 0.66
        )


@pytest.mark.parametrize("shift, accepted", [(0.0009, True), (0.0011, False)])
def test_pallet_envelope_tolerance_is_absolute(synthetic_pallet_urdf, shift, accepted):
    path = synthetic_pallet_urdf(x_m=shift)
    if accepted:
        MODULE.assert_pallet_urdf_matches_geometry(path, 0.66, 0.66)
    else:
        with pytest.raises(ValueError, match="envelope"):
            MODULE.assert_pallet_urdf_matches_geometry(path, 0.66, 0.66)


@pytest.mark.parametrize(
    "links", ["", '<link name="a"/><link name="b"/>', '<link name="a"/>']
)
def test_pallet_boxes_reject_missing_multiple_or_empty_links(tmp_path, links):
    path = tmp_path / "invalid.urdf"
    path.write_text(f'<robot name="invalid">{links}</robot>')
    with pytest.raises(ValueError):
        MODULE.pallet_boxes_from_urdf(path)


def test_named_boxes_accept_normal_t11(full_t11_pallet_urdf):
    pallet_geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_t11_06.yaml")
    path = full_t11_pallet_urdf(swap_all=False, swap_boards_only=False)
    assert len(MODULE.pallet_boxes_from_urdf(path)) == 22
    MODULE.assert_pallet_urdf_matches_named_boxes(path, pallet_geometry)


@pytest.mark.parametrize("swap", ["swap_all", "swap_boards_only"])
def test_named_boxes_reject_t11_90_degree_swap(full_t11_pallet_urdf, swap):
    pallet_geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_t11_06.yaml")
    path = full_t11_pallet_urdf(**{swap: True})
    assert len(MODULE.pallet_boxes_from_urdf(path)) == 22
    MODULE.assert_pallet_urdf_matches_geometry(path, 0.66, 0.66)
    with pytest.raises(ValueError, match="bottom_board_0"):
        MODULE.assert_pallet_urdf_matches_named_boxes(path, pallet_geometry)


PROVISIONAL_URDF = ROOT / "sim/models/dls08_provisional/forklift.urdf"
MEASURED_URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"


def edited_urdf(tmp_path, edit):
    import xml.etree.ElementTree as ET

    tree = ET.parse(MEASURED_URDF)
    edit(tree.getroot())
    path = tmp_path / "forklift.urdf"
    tree.write(path)
    return path


@pytest.mark.parametrize(
    ("urdf", "expected"),
    [
        (PROVISIONAL_URDF, (0.64, 0.51, 0.135, 0.45)),
        (MEASURED_URDF, (0.66, 0.53, 0.125, np.radians(15))),
    ],
)
def test_drive_geometry_is_read_from_the_model(urdf, expected):
    geometry = MODULE.read_drive_geometry_m(urdf, 8.0)
    np.testing.assert_allclose(
        (
            geometry.wheelbase_m,
            geometry.track_m,
            geometry.wheel_radius_m,
            geometry.max_steering_rad,
        ),
        expected,
        atol=1e-9,
    )
    assert geometry.max_wheel_rate_rad_s == 8.0


@pytest.mark.parametrize(
    ("urdf", "expected"), [(PROVISIONAL_URDF, 0.406), (MEASURED_URDF, 0.346)]
)
def test_carriage_limit_is_read_from_the_model(urdf, expected):
    assert MODULE.read_carriage_limit_m(urdf) == pytest.approx(expected)


def _insert_parent(root, joint_name, offset):
    """Move a joint under a new intermediate link offset from base_link."""
    import xml.etree.ElementTree as ET

    joint = root.find(f"joint[@name='{joint_name}']")
    ET.SubElement(root, "link", name=f"{joint_name}_mount")
    mount = ET.SubElement(root, "joint", name=f"{joint_name}_mount_fixed", type="fixed")
    ET.SubElement(mount, "parent", link="base_link")
    ET.SubElement(mount, "child", link=f"{joint_name}_mount")
    ET.SubElement(mount, "origin", xyz=offset, rpy="0 0 0")
    joint.find("parent").set("link", f"{joint_name}_mount")


def test_readers_compose_parent_links(tmp_path):
    def raise_steering(root):
        for name in ("left_steer", "right_steer"):
            _insert_parent(root, name, "0.10 0 0")

    geometry = MODULE.read_drive_geometry_m(edited_urdf(tmp_path, raise_steering), 8.0)
    assert geometry.wheelbase_m == pytest.approx(0.76)

    def shift_lift(root):
        origin = root.find("joint[@name='fork_lift']/origin")
        origin.set("xyz", "0.10 0 0")

    shifted = edited_urdf(tmp_path, shift_lift)
    assert MODULE.read_chassis_reference_m(shifted)[0] == pytest.approx(1.39)
    assert MODULE.read_carriage_limit_m(shifted) == pytest.approx(0.346)


@pytest.mark.parametrize(
    "edit",
    [
        lambda root: root.find("joint[@name='left_steer']/origin").set(
            "rpy", "0 0 0.1"
        ),
        lambda root: root.find("joint[@name='left_steer']/axis").set("xyz", "0 1 0"),
        lambda root: root.find("joint[@name='left_steer']/origin").set(
            "xyz", "0.32 0.27 0.125"
        ),
        lambda root: root.find("joint[@name='right_steer']/limit").set("lower", "-0.3"),
        lambda root: root.find(
            "link[@name='rear_left_wheel']/collision/geometry/cylinder"
        ).set("radius", "0.13"),
    ],
    ids=["rotated-chain", "steer-axis", "asymmetric", "uneven-limit", "tyre-radius"],
)
def test_drive_readers_reject_unsupported_models(tmp_path, edit):
    path = edited_urdf(tmp_path, edit)
    with pytest.raises(ValueError):
        MODULE.read_drive_geometry_m(path, 8.0)


def test_carriage_readers_reject_a_rotated_chain(tmp_path):
    path = edited_urdf(
        tmp_path,
        lambda root: root.find("joint[@name='fork_lift']/origin").set("rpy", "0 0 0.1"),
    )
    for reader in (MODULE.read_carriage_limit_m, MODULE.read_chassis_reference_m):
        with pytest.raises(ValueError):
            reader(path)


def test_a_moving_intermediate_parent_is_refused(tmp_path):
    def revolute_mount(root):
        for name in ("left_steer", "right_steer"):
            _insert_parent(root, name, "0.10 0 0")
            mount = root.find(f"joint[@name='{name}_mount_fixed']")
            mount.set("type", "revolute")

    with pytest.raises(ValueError, match="moving parent"):
        MODULE.read_drive_geometry_m(edited_urdf(tmp_path, revolute_mount), 8.0)


def test_insertion_measures_read_each_fork_against_the_pallet():
    measured = MODULE.InsertionGeometry.from_urdfs(
        MEASURED_URDF, ROOT / "sim/models/epal6_pallet/pallet.urdf"
    )
    # Truck base at x b, pallet at the origin facing it: tip x = b + 0.95.
    b = -0.95 + (-0.30 + 0.30)  # tip exactly at the pallet centre
    m = measured.insertion_measures(
        (b, 0, 0), (1, 0, 0, 0), 0.0, (0, 0, 0), (1, 0, 0, 0), 0.60
    )
    for side in ("left", "right"):
        assert m[side]["insertion_m"] == pytest.approx(0.30)
        assert m[side]["beyond_centre_m"] == pytest.approx(0.0)
    # The cross members' front 0.604 is 0.346 behind the tip: 0.046 before the face.
    assert m["carriage_face_gap_m"] == pytest.approx(0.046)
    assert m["carriage_nearest_box"].startswith("carriage_cross")
    assert m["carriage_overlaps"] == []
    yawed = measured.insertion_measures(
        (b, 0, 0),
        (math.cos(0.025), 0, 0, math.sin(0.025)),
        0.0,
        (0, 0, 0),
        (1, 0, 0, 0),
        0.60,
    )
    assert yawed["left"]["insertion_m"] != pytest.approx(yawed["right"]["insertion_m"])


def test_carriage_gap_uses_the_whole_box_and_reports_overlap(tmp_path):
    import subprocess
    import sys

    subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools/build_pallet_model.py"),
            "--geometry",
            str(ROOT / "config/pallet_geometry_t11_06.yaml"),
            "--output",
            str(tmp_path),
        ],
        check=True,
        capture_output=True,
    )
    g = MODULE.InsertionGeometry.from_urdfs(MEASURED_URDF, tmp_path / "pallet.urdf")
    yaw = 0.0195
    q = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
    near = g.insertion_measures(
        (-0.33 + 0.336 - 0.95, 0, 0), q, 0.0, (0, 0, 0), (1, 0, 0, 0), 0.66
    )
    # The yawed cross member's corner, not its centre, sets the clearance.
    assert 0 < near["carriage_face_gap_m"] < 0.010 - 0.004
    assert near["carriage_overlaps"] == []
    into = g.insertion_measures(
        (-0.33 + 0.342 - 0.95, 0, 0), q, 0.0, (0, 0, 0), (1, 0, 0, 0), 0.66
    )
    assert into["carriage_face_gap_m"] < 0
    assert into["carriage_overlaps"]


def lateral(geometry, base=(-0.89, 0, 0), q=(1, 0, 0, 0), lift=0):
    return geometry.lateral_clearances(base, q, lift, (0, 0, 0), (1, 0, 0, 0))


def test_lateral_clearance_is_symmetric_when_aligned_and_shifts_with_offset(geometry):
    centred = lateral(geometry)
    assert centred["left"] == pytest.approx(centred["right"], abs=1e-9)
    assert 0.03 < centred["left"] < 0.08
    # The nearest wall of each blade is the centre block column (EPAL 6: 45 mm
    # each side of it, 127.5 mm to the outer blocks), so a +y shift opens the
    # left blade's gap and closes the right one's.
    shifted = lateral(geometry, base=(-0.89, 0.02, 0))
    assert shifted["left"] == pytest.approx(centred["left"] + 0.02, abs=1e-9)
    assert shifted["right"] == pytest.approx(centred["right"] - 0.02, abs=1e-9)


def test_lateral_clearance_goes_negative_on_a_scrape_and_is_none_outside(geometry):
    assert min(lateral(geometry, base=(-0.89, 0.05, 0)).values()) < 0
    assert lateral(geometry, base=(-3.0, 0, 0)) == {"left": None, "right": None}


def test_lateral_clearance_shrinks_with_relative_yaw(geometry):
    angle = 0.04
    turned = lateral(geometry, q=(np.cos(angle / 2), 0, 0, np.sin(angle / 2)))
    centred = lateral(geometry)
    assert min(turned.values()) < min(centred.values())


def test_lateral_clearance_ignores_the_boards_and_stringers_over_the_blade(geometry):
    # Codex counterexample: lifted 50 mm, pitched 0.01 rad, a hair into the
    # stringer above -- not a side wall; the block columns are still 45 mm away.
    angle = 0.01
    q = (np.cos(angle / 2), 0, np.sin(angle / 2), 0)
    gaps = geometry.lateral_clearances((-0.89, 0, 0.0036), q, 0.05, (0, 0, 0), (1, 0, 0, 0))
    assert gaps["left"] == pytest.approx(0.045, abs=1e-3)
    assert gaps["right"] == pytest.approx(0.045, abs=1e-3)
