"""Instantaneous fork/pallet box clearance; no simulator SDK or contact claims.

Use during insertion, immediately before lift, and during lowered withdrawal.
Supporting contact during lift/transport is intentional and must not use this
guard. This geometric check does not measure PhysX contact forces or prove
continuous swept clearance between updates.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike

from forklift_core.control.path_tracking import AckermannGeometry
from forklift_core.geometry import rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.pallet_geometry import PalletGeometry, pallet_boxes


def _vector(value: ArrayLike) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("position/geometry must be a finite three-vector")
    return result


def _rotation(quaternion_wxyz: ArrayLike) -> np.ndarray:
    q = np.asarray(quaternion_wxyz, dtype=float)
    if q.shape != (4,):
        raise ValueError("quaternion_wxyz must have shape (4,)")
    return rotation_matrix_from_quaternion_xyzw(q[[1, 2, 3, 0]])


def _boxes(link: ET.Element) -> tuple:
    boxes = []
    for collision in link.findall("collision"):
        box = collision.find("geometry/box")
        origin = collision.find("origin")
        if box is None or origin is None:
            raise ValueError("Only explicit URDF collision boxes are supported")
        center = _vector(np.fromstring(origin.get("xyz", "0 0 0"), sep=" "))
        half = _vector(np.fromstring(box.get("size", ""), sep=" ")) / 2
        rpy = _vector(np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
        if np.any(half <= 0):
            raise ValueError("Box dimensions must be positive")
        # Source models use unrotated boxes; reject a changed contract explicitly.
        if np.any(rpy != 0):
            raise ValueError("Source collision boxes must have zero local rpy")
        boxes.append((collision.get("name", "unnamed"), center, half))
    if not boxes:
        raise ValueError("Collision boxes are required")
    return tuple(boxes)


_JOINT_TOLERANCE_M = 0.001
# Moving joints a frame may hang from; read at the lowered / straight pose.
_NEUTRAL_PARENT_JOINTS = frozenset({"fork_lift", "left_steer", "right_steer"})


def _xyz(element: ET.Element | None, attribute: str, what: str) -> np.ndarray:
    if element is None:
        raise ValueError(f"Missing {what}")
    try:
        return _vector([float(value) for value in element.get(attribute, "").split()])
    except ValueError as exc:
        raise ValueError(f"Malformed {attribute} of {what}") from exc


def _base_position_m(truck: ET.Element, joint_name: str) -> np.ndarray:
    """Base-link position of a joint frame, composing every parent joint.

    Joint origins are relative to their parent link. Only translations are
    supported: a rotated origin anywhere in the chain is rejected rather than
    half-composed. Parents above the named joint must be fixed, except the
    lift and steering joints, which are read at their neutral (lowered,
    straight) position.
    """
    joints = {}
    for joint in truck.findall("joint"):
        child = joint.find("child")
        if child is not None:
            joints[child.get("link")] = joint
    joint = truck.find(f"joint[@name='{joint_name}']")
    position = np.zeros(3)
    for _ in range(len(joints) + 1):
        if joint is None:
            raise ValueError(f"Missing chassis joint: {joint_name}")
        what = f"origin of {joint.get('name')}"
        origin = joint.find("origin")
        position += _xyz(origin, "xyz", what)
        if origin.get("rpy") is not None and np.any(_xyz(origin, "rpy", what) != 0):
            raise ValueError(f"Rotated chassis joint origin: {joint.get('name')}")
        parent = joint.find("parent")
        link = None if parent is None else parent.get("link")
        if link == "base_link":
            return position
        joint = joints.get(link)
        if (
            joint is not None
            and joint.get("type") != "fixed"
            and joint.get("name") not in _NEUTRAL_PARENT_JOINTS
        ):
            raise ValueError(
                f"{joint_name} hangs from moving parent {joint.get('name')}"
            )
    raise ValueError(f"Joint chain does not reach base_link: {joint_name}")


def read_chassis_reference_m(forklift_urdf: Path) -> tuple[float, float]:
    """Return axle-to-tip distance and signed base-frame rear axle x in metres.

    Read the named URDF joint origins, composed up to base_link; reject missing
    or malformed coordinates instead of guessing.
    """
    truck = ET.parse(forklift_urdf).getroot()
    tip = float(_base_position_m(truck, "left_fork_tip_fixed")[0])
    rear = float(_base_position_m(truck, "rear_left_spin")[0])
    return tip - rear, rear


def read_carriage_limit_m(forklift_urdf: Path) -> float:
    """Fork tip x minus the front face of the carriage cross members.

    This is the all-box insertion limit of the model (ADR 0003 section 1); it is
    only as measured as the model's fork root and cross-member thickness.
    """
    truck = ET.parse(forklift_urdf).getroot()
    tip = float(_base_position_m(truck, "left_fork_tip_fixed")[0])
    carriage = _base_position_m(truck, "fork_lift")
    link = truck.find("link[@name='fork_carriage']")
    if link is None:
        raise ValueError("Missing fork_carriage link")
    fronts = [
        carriage[0] + center[0] + half[0]
        for name, center, half in _boxes(link)
        if name.startswith("carriage_cross_")
    ]
    if not fronts:
        raise ValueError("Missing carriage_cross collision boxes")
    return tip - max(fronts)


def _cylinder_radius_m(truck: ET.Element, link_name: str) -> float:
    """Radius of the link's collision cylinder, which must roll about base y."""
    collision = None
    for candidate in truck.findall(f"link[@name='{link_name}']/collision"):
        if candidate.find("geometry/cylinder") is not None:
            collision = candidate
    if collision is None:
        raise ValueError(f"Missing tyre collision cylinder: {link_name}")
    radius = float(collision.find("geometry/cylinder").get("radius", "nan"))
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError(f"Invalid tyre radius: {link_name}")
    origin = collision.find("origin")
    rpy = np.zeros(3) if origin is None else _xyz(origin, "rpy", f"{link_name} tyre")
    roll, pitch, yaw = rpy
    # URDF cylinders run along local z; Rz(yaw) Ry(pitch) Rx(roll) applied to z.
    axis = np.array(
        [
            np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll),
            np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll),
            np.cos(pitch) * np.cos(roll),
        ]
    )
    if abs(axis[1]) < 1 - 1e-6:
        raise ValueError(f"The {link_name} tyre does not roll about the y axis")
    return radius


def read_drive_geometry_m(
    forklift_urdf: Path, max_wheel_rate_rad_s: float
) -> AckermannGeometry:
    """Front-steered Ackermann geometry from the model's joints and tyres.

    Wheelbase is steer pivot x minus rear axle x, track the steer pivot
    separation, radius the common tyre collision radius, and the steering limit
    the symmetric limit shared by both steer joints. Anything outside that
    contract (rotated chains, other axes, asymmetry, rear steering) is rejected.
    """
    truck = ET.parse(forklift_urdf).getroot()
    tol = _JOINT_TOLERANCE_M
    for name, axis in [
        ("left_steer", (0, 0, 1)),
        ("right_steer", (0, 0, 1)),
        ("front_left_spin", (0, 1, 0)),
        ("front_right_spin", (0, 1, 0)),
        ("rear_left_spin", (0, 1, 0)),
        ("rear_right_spin", (0, 1, 0)),
    ]:
        joint = truck.find(f"joint[@name='{name}']")
        if joint is None:
            raise ValueError(f"Missing chassis joint: {name}")
        if not np.allclose(_xyz(joint.find("axis"), "xyz", f"axis of {name}"), axis):
            raise ValueError(f"Unexpected joint axis: {name}")
    steer_l = _base_position_m(truck, "left_steer")
    steer_r = _base_position_m(truck, "right_steer")
    rear_l = _base_position_m(truck, "rear_left_spin")
    rear_r = _base_position_m(truck, "rear_right_spin")
    wheels = {
        name: _base_position_m(truck, f"{name}_spin")
        for name in ("front_left", "front_right", "rear_left", "rear_right")
    }
    for left, right in [(steer_l, steer_r), (rear_l, rear_r)]:
        if abs(left[0] - right[0]) > tol or abs(left[1] + right[1]) > tol:
            raise ValueError("Chassis joints are not left/right symmetric")
    track = steer_l[1] - steer_r[1]
    if abs(track - (rear_l[1] - rear_r[1])) > tol:
        raise ValueError("Front and rear tracks differ")
    wheelbase = steer_l[0] - rear_l[0]
    if wheelbase <= 0:
        raise ValueError("Steering axle must be in front of the rear axle")
    radii = {name: _cylinder_radius_m(truck, f"{name}_wheel") for name in wheels}
    radius = radii["rear_left"]
    if any(abs(r - radius) > tol for r in radii.values()) or any(
        abs(position[2] - radius) > tol for position in wheels.values()
    ):
        raise ValueError("Tyres differ or do not stand on the ground plane")
    limits = []
    for name in ("left_steer", "right_steer"):
        limit = truck.find(f"joint[@name='{name}']/limit")
        if limit is None:
            raise ValueError(f"Missing steering limit: {name}")
        lower, upper = (
            float(limit.get("lower", "nan")),
            float(limit.get("upper", "nan")),
        )
        if not (np.isfinite(lower) and np.isfinite(upper)) or abs(lower + upper) > 1e-9:
            raise ValueError(f"Steering limit is not symmetric: {name}")
        limits.append(upper)
    if abs(limits[0] - limits[1]) > 1e-9:
        raise ValueError("Left and right steering limits differ")
    return AckermannGeometry(
        float(wheelbase), float(track), radius, limits[0], max_wheel_rate_rad_s
    )


def pallet_boxes_from_urdf(pallet_urdf: Path) -> tuple:
    """Return all named collision boxes of the single free pallet link."""
    pallet = ET.parse(pallet_urdf).getroot()
    links = pallet.findall("link")
    if len(links) != 1 or pallet.findall("joint"):
        raise ValueError("Expected single-link free pallet")
    return _boxes(links[0])


def assert_pallet_urdf_matches_geometry(
    pallet_urdf: Path,
    geometry_depth_m: float,
    geometry_width_m: float,
    *,
    tolerance_m: float = 0.001,
) -> None:
    """Check depth/width, xy centring and z floor with absolute metre tolerance.

    This checks the overall collision envelope only, not named internal layout.
    """
    boxes = pallet_boxes_from_urdf(pallet_urdf)
    low = np.min([center - half for _, center, half in boxes], axis=0)
    high = np.max([center + half for _, center, half in boxes], axis=0)
    actual = np.array([low[0], high[0], low[1], high[1], low[2]])
    expected = np.array(
        [
            -geometry_depth_m / 2,
            geometry_depth_m / 2,
            -geometry_width_m / 2,
            geometry_width_m / 2,
            0,
        ]
    )
    if not np.allclose(actual, expected, rtol=0, atol=tolerance_m):
        raise ValueError(
            f"Pallet collision envelope mismatch: got {actual.tolist()}, "
            f"expected {expected.tolist()} within {tolerance_m} m"
        )


def assert_pallet_urdf_matches_named_boxes(
    pallet_urdf: Path,
    geometry: PalletGeometry,
    *,
    tolerance_m: float = 0.001,
) -> None:
    """Compare every named collision box's centre and size against the YAML.

    Catches assembly a bounding-box check cannot: for a square pallet like
    T11, a 90-degree-swapped or 180-degree-rotated URDF keeps the same
    envelope, centre and box count but moves most named box centres/sizes.
    See docs/plans/2026-09-17-hybrid-astar-transport.md:214-245 for the
    verified counts (21/22 boxes differ under a 90-degree name-preserving
    swap, 18/22 under 180 degrees).
    """
    expected_boxes = {box.name: box for box in pallet_boxes(geometry)}
    actual_boxes = {}
    for name, actual_centre_m, actual_half_m in pallet_boxes_from_urdf(pallet_urdf):
        if name in actual_boxes:
            raise ValueError(f"Duplicate pallet collision box name: {name}")
        actual_boxes[name] = (actual_centre_m, actual_half_m)
    if expected_boxes.keys() != actual_boxes.keys():
        missing = sorted(expected_boxes.keys() - actual_boxes.keys())
        extra = sorted(actual_boxes.keys() - expected_boxes.keys())
        raise ValueError(
            f"Pallet collision box names mismatch: missing={missing}, extra={extra}"
        )
    mismatches = []
    for name, expected_box in expected_boxes.items():
        actual_centre_m, actual_half_m = actual_boxes[name]
        expected_size_m = expected_box.size_m
        actual_size_m = actual_half_m * 2
        for field, actual, expected in (
            ("centre_m", actual_centre_m, expected_box.centre_m),
            ("size_m", actual_size_m, expected_size_m),
        ):
            if not np.allclose(actual, expected, rtol=0, atol=tolerance_m):
                mismatches.append(
                    f"{name} {field}: got {actual.tolist()}, expected {list(expected)}"
                )
    if mismatches:
        raise ValueError(
            f"Pallet named collision boxes mismatch (tolerance {tolerance_m} m): "
            + "; ".join(mismatches)
        )


class InsertionGeometry:
    """Source URDF boxes for the provisional lift chain and a single-link pallet."""

    @classmethod
    def from_urdfs(cls, forklift_urdf: Path, pallet_urdf: Path) -> "InsertionGeometry":
        """Read actual fork blades/heels and all pallet collision boxes once.

        The supported provisional lift chain has zero joint origin and local +z
        translation. Reject changed structure instead of guessing its geometry.
        """
        truck = ET.parse(forklift_urdf).getroot()
        pallet = ET.parse(pallet_urdf).getroot()
        joint = truck.find("joint[@name='fork_lift']")
        if joint is None or joint.get("type") != "prismatic":
            raise ValueError("Expected provisional fork_lift prismatic joint")
        for element, attribute, expected in [
            ("origin", "xyz", [0, 0, 0]),
            ("origin", "rpy", [0, 0, 0]),
            ("axis", "xyz", [0, 0, 1]),
        ]:
            node = joint.find(element)
            if node is None or not np.array_equal(
                _vector(np.fromstring(node.get(attribute, ""), sep=" ")), expected
            ):
                raise ValueError("Unsupported fork_lift origin/axis")
        if (
            joint.find("parent").get("link") != "base_link"
            or joint.find("child").get("link") != "fork_carriage"
        ):
            raise ValueError("Unsupported fork_lift chain")
        carriage = truck.find("link[@name='fork_carriage']")
        links = pallet.findall("link")
        if carriage is None or len(links) != 1 or pallet.findall("joint"):
            raise ValueError("Expected fork carriage and single-link free pallet")
        result = cls()
        result.fork_boxes = tuple(b for b in _boxes(carriage) if "fork" in b[0])
        if len(result.fork_boxes) != 4:
            raise ValueError("Expected two fork blades and two heels")
        result.pallet_boxes = _boxes(links[0])
        result._pallet_centers = np.array([b[1] for b in result.pallet_boxes])
        result._pallet_halves = np.array([b[2] for b in result.pallet_boxes])
        return result

    def forbidden_contacts(
        self,
        base_position_m: ArrayLike,
        base_quaternion_wxyz: ArrayLike,
        lift_m: float,
        pallet_position_m: ArrayLike,
        pallet_quaternion_wxyz: ArrayLike,
        *,
        clearance_m: float = 0.002,
    ) -> tuple[tuple[str, str], ...]:
        """Return potentially contacting (fork, pallet) source collision names.

        Positions are finite world-frame (3,) metres; rotations are unit (4,)
        wxyz quaternions. Lift is measured local +z joint displacement in metres.
        Exact 15-axis OBB SAT includes touching at zero clearance. Positive
        clearance inflates fork boxes on every local face, conservatively
        rejecting separation under the margin; 2 mm matches the synthetic
        contact offset. Results are instantaneous, independent of pallet motion.
        """
        if not np.isfinite([lift_m, clearance_m]).all() or clearance_m < 0:
            raise ValueError("Lift/margin must be finite and margin nonnegative")
        world_from_pallet = _rotation(pallet_quaternion_wxyz)
        pallet_from_base = world_from_pallet.T @ _rotation(base_quaternion_wxyz)
        translation = world_from_pallet.T @ (
            _vector(base_position_m) - _vector(pallet_position_m)
        )
        # All source boxes are axis aligned in their owning link. Their 15 SAT
        # axes are shared by every pair, so compute projections just once.
        axes = np.concatenate(
            (
                np.eye(3),
                pallet_from_base.T,
                np.cross(np.eye(3)[:, None, :], pallet_from_base.T[None, :, :]).reshape(
                    9, 3
                ),
            )
        )
        projected_pallet = self._pallet_halves @ np.abs(axes).T
        projected_fork_axes = np.abs(axes @ pallet_from_base)
        contacts = []
        for name, center, half in self.fork_boxes:
            fork_center = translation + pallet_from_base @ (center + [0, 0, lift_m])
            separations = np.abs((self._pallet_centers - fork_center) @ axes.T)
            radii = projected_pallet + projected_fork_axes @ (half + clearance_m)
            overlap = np.all(separations <= radii + 1e-12, axis=1)
            contacts.extend(
                (name, self.pallet_boxes[i][0]) for i in np.flatnonzero(overlap)
            )
        return tuple(contacts)
