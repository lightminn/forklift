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

from forklift_core.geometry import rotation_matrix_from_quaternion_xyzw


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


def read_chassis_reference_m(forklift_urdf: Path) -> tuple[float, float]:
    """Return axle-to-tip distance and signed base-frame rear axle x in metres.

    Read the named provisional URDF joint origins; reject missing or malformed
    coordinates instead of guessing. The fork-tip origin is base-relative, so
    subtract the rear axle origin to obtain the axle-relative distance.
    """
    truck = ET.parse(forklift_urdf).getroot()
    coordinates = []
    for name in ("left_fork_tip_fixed", "rear_left_spin"):
        origin = truck.find(f"joint[@name='{name}']/origin")
        if origin is None:
            raise ValueError(f"Missing chassis joint/origin: {name}")
        try:
            xyz = _vector([float(value) for value in origin.get("xyz", "").split()])
        except ValueError as exc:
            raise ValueError(f"Malformed chassis joint origin xyz: {name}") from exc
        coordinates.append(float(xyz[0]))
    tip, rear = coordinates
    return tip - rear, rear


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
