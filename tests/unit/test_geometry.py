import numpy as np
import pytest

from forklift_core import geometry as g


def test_optical_axes_and_mount_translation_are_applied_in_target_frame():
    points = g.FramePoints("depth_optical", [[0, 0, 2], [1, 0, 2], [0, 1, 2]])
    transform = g.RigidTransform(
        "depth_optical",
        "base_link",
        [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
        [0.2, 0, 0.5],
    )
    result = transform.apply(points)
    assert result.frame_id == "base_link"
    np.testing.assert_allclose(
        result.xyz_m, [[2.2, 0, 0.5], [2.2, -1, 0.5], [2.2, 0, -0.5]]
    )


def test_transform_preserves_missing_measurement_and_sample_order():
    points = g.FramePoints("laser", [[np.nan] * 3, [1, 0, 0]])
    result = g.RigidTransform("laser", "base", np.eye(3), [0.1, 0.2, 0.3]).apply(points)
    np.testing.assert_array_equal(result.valid, [False, True])
    assert np.isnan(result.xyz_m[0]).all()
    np.testing.assert_allclose(result.xyz_m[1], [1.1, 0.2, 0.3])


def test_wrong_source_frame_cannot_be_transformed():
    transform = g.RigidTransform("depth", "base", np.eye(3), [0, 0, 0])
    with pytest.raises(ValueError):
        transform.apply(g.FramePoints("color", [[0, 0, 1]]))


@pytest.mark.parametrize(
    "rotation",
    [
        np.diag([1, 1, -1]),
        np.diag([2, 1, 1]),
        np.zeros((2, 2)),
        np.full((3, 3), np.nan),
        np.diag([1e308, 1, 1]),
    ],
)
def test_reflection_scaling_or_malformed_rotation_is_rejected(rotation):
    with pytest.raises(ValueError):
        g.RigidTransform("a", "b", rotation, [0, 0, 0])


@pytest.mark.parametrize("translation", [[0, 0], [0, 0, np.inf], [0, np.nan, 0]])
def test_invalid_translation_is_rejected(translation):
    with pytest.raises(ValueError):
        g.RigidTransform("a", "b", np.eye(3), translation)


@pytest.mark.parametrize(
    "points", [[1, 2, 3], [[1, 2]], [[1, np.nan, 2]], [[np.inf, np.inf, np.inf]]]
)
def test_malformed_points_cannot_enter_geometry_pipeline(points):
    with pytest.raises(ValueError):
        g.FramePoints("depth", points)


@pytest.mark.parametrize("frame", ["", "  ", None])
def test_unidentified_coordinate_frame_is_rejected(frame):
    with pytest.raises(ValueError):
        g.FramePoints(frame, [[0, 0, 1]])
    with pytest.raises(ValueError):
        g.RigidTransform("a", frame, np.eye(3), [0, 0, 0])


def test_empty_points_remain_empty_after_transform():
    result = g.RigidTransform("a", "b", np.eye(3), [0, 0, 0]).apply(
        g.FramePoints("a", np.empty((0, 3)))
    )
    assert result.xyz_m.shape == (0, 3)
    assert result.valid.size == 0


def test_mutating_constructor_inputs_cannot_change_calibration_or_points():
    rotation, translation, xyz = np.eye(3), np.zeros(3), np.array([[1.0, 2.0, 3.0]])
    transform = g.RigidTransform("a", "b", rotation, translation)
    points = g.FramePoints("a", xyz)
    rotation[:] = 0
    translation[:] = 99
    xyz[:] = 99
    np.testing.assert_array_equal(transform.apply(points).xyz_m, [[1, 2, 3]])


def test_quaternion_xyzw_to_rotation_matches_optical_to_base_convention():
    # Existing synthetic mounting: optical +z -> base +x, +x -> -y, +y -> -z.
    rotation = g.rotation_matrix_from_quaternion_xyzw([-0.5, 0.5, -0.5, 0.5])
    np.testing.assert_allclose(
        rotation, [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-12
    )
    identity = g.rotation_matrix_from_quaternion_xyzw([0, 0, 0, 1])
    np.testing.assert_allclose(identity, np.eye(3), atol=1e-12)


@pytest.mark.parametrize(
    "quaternion",
    [[0, 0, 0, 0.5], [0, 0, 0, 2], [1, 0, 0], [0, 0, np.nan, 1], [0, 0, 0, np.inf]],
)
def test_non_unit_or_malformed_quaternions_are_rejected(quaternion):
    with pytest.raises(ValueError):
        g.rotation_matrix_from_quaternion_xyzw(quaternion)


def test_near_unit_quaternion_is_normalised_before_rigid_transform_validation():
    rotation = g.rotation_matrix_from_quaternion_xyzw([1.0000005, 0, 0, 0])
    transform = g.RigidTransform("a", "b", rotation, [0, 0, 0])
    np.testing.assert_allclose(transform.rotation, np.diag([1, -1, -1]), atol=1e-12)


def test_quaternion_sign_does_not_change_rotation():
    rotation = g.rotation_matrix_from_quaternion_xyzw([-0.5, 0.5, -0.5, 0.5])
    negated = g.rotation_matrix_from_quaternion_xyzw([0.5, -0.5, 0.5, -0.5])
    np.testing.assert_allclose(rotation, negated, atol=1e-12)


@pytest.mark.parametrize("quaternion", [[0, 0, 0, 0], [1e308, 0, 0, 0], [[0, 0, 0, 1]]])
def test_zero_huge_and_matrix_quaternions_are_rejected(quaternion):
    with pytest.raises(ValueError):
        g.rotation_matrix_from_quaternion_xyzw(quaternion)
