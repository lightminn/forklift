"""Explicit coordinate frames and metric rigid transforms."""

from dataclasses import dataclass
from math import hypot

import numpy as np
from numpy.typing import ArrayLike, NDArray

from forklift_core._validation import _frame_id, _real_array


def rotation_matrix_from_quaternion_xyzw(q: ArrayLike) -> NDArray[np.float64]:
    """Return a float64 (3, 3) rotation from a finite unit quaternion [x,y,z,w].

    Norm error up to 1e-6 is accepted and removed before conversion. A quaternion
    and its negation represent the same rotation; non-unit inputs raise ValueError.
    """
    quaternion = _real_array(q, "quaternion_xyzw")
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("quaternion_xyzw must be a finite four-vector")
    norm = hypot(*quaternion)
    if abs(norm - 1.0) > 1e-6:
        raise ValueError("quaternion_xyzw must have unit norm within 1e-6")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class FramePoints:
    """One row per source sample; all-NaN rows are unknown measurements.

    Invalid or absent measurements do not establish free space. Arrays are
    copied on construction so caller mutation cannot silently alter geometry.
    """

    frame_id: str
    xyz_m: ArrayLike

    def __post_init__(self) -> None:
        _frame_id(self.frame_id)
        xyz = _real_array(self.xyz_m, "xyz_m")
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError("xyz_m must have shape (N, 3)")
        complete = np.isfinite(xyz).all(axis=1) | np.isnan(xyz).all(axis=1)
        if not complete.all():
            raise ValueError("Each point must be entirely finite or entirely NaN")
        xyz.setflags(write=False)
        object.__setattr__(self, "xyz_m", xyz)

    @property
    def valid(self) -> NDArray[np.bool_]:
        return np.isfinite(self.xyz_m).all(axis=1)


@dataclass(frozen=True)
class RigidTransform:
    """Map source to target using p_target = R @ p_source + translation_m.

    Supply calibration explicitly. Optical-axis conversion alone does not
    establish a physical camera mounting transform.
    """

    source_frame: str
    target_frame: str
    rotation: ArrayLike
    translation_m: ArrayLike

    def __post_init__(self) -> None:
        _frame_id(self.source_frame)
        _frame_id(self.target_frame)
        rotation = _real_array(self.rotation, "rotation")
        translation = _real_array(self.translation_m, "translation_m")
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            raise ValueError("rotation must be a finite 3-by-3 matrix")
        with np.errstate(over="ignore", invalid="ignore"):
            gram = rotation.T @ rotation
        if not np.allclose(gram, np.eye(3), atol=1e-6, rtol=0):
            raise ValueError("rotation must be orthonormal")
        if not np.isclose(np.linalg.det(rotation), 1, atol=1e-6, rtol=0):
            raise ValueError("rotation must be proper, with determinant +1")
        if translation.shape != (3,) or not np.isfinite(translation).all():
            raise ValueError("translation_m must be a finite three-vector")
        rotation.setflags(write=False)
        translation.setflags(write=False)
        object.__setattr__(self, "rotation", rotation)
        object.__setattr__(self, "translation_m", translation)

    def apply(self, points: FramePoints) -> FramePoints:
        if points.frame_id != self.source_frame:
            raise ValueError(
                f"Expected frame {self.source_frame!r}, got {points.frame_id!r}"
            )
        return FramePoints(
            self.target_frame, points.xyz_m @ self.rotation.T + self.translation_m
        )
