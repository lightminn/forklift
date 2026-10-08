"""Frame-chain arithmetic of rig_slam_bridge (plan 2026-10-07 D2-D3), no ROS needed."""

import math

import pytest
from forklift_ros.rig_slam_bridge import Chain

from forklift_core.localization.slam_pose import compose


def _close(a, b, tol=1e-9):
    assert a[0] == pytest.approx(b[0], abs=tol) and a[1] == pytest.approx(b[1], abs=tol)
    assert math.remainder(a[2] - b[2], 2 * math.pi) == pytest.approx(0.0, abs=tol)


def test_the_fused_and_output_frames_start_on_the_wheel_odometry_frame():
    chain = Chain("vision")
    start = (3.0, -1.0, 0.4)
    # Visual odometry has its own origin; only its increments matter.
    top = chain.update({"vo": (0.0, 0.0, 0.0)}, start)
    _close(top, start)
    _close(chain.top_from_below(start), (0.0, 0.0, 0.0))
    _close(chain.output_from_base(start), start)


def test_lidar_increments_override_slipping_wheels():
    chain = Chain("lidar")
    chain.update({"icp": (10.0, 10.0, 0.0)}, (0.0, 0.0, 0.0))
    chain.output_from_base((0.0, 0.0, 0.0))
    # Wheels claim 0.105 m, the ICP frame moved 0.100 m.
    top = chain.update({"icp": (10.100, 10.0, 0.0)}, (0.105, 0.0, 0.0))
    # Weighted by the assumed noise: much nearer the ICP than the wheels.
    assert abs(top[0] - 0.100) < abs(top[0] - 0.105) / 3
    # The TF published keeps the chain consistent: fused <- odom <- base.
    _close(compose(chain.top_from_below((0.105, 0.0, 0.0)), (0.105, 0.0, 0.0)), top)


def test_a_lost_front_end_leaves_the_wheels_and_the_other_front_end():
    chain = Chain("fusion")
    chain.update({"vo": (0.0, 0.0, 0.0), "icp": (0.0, 0.0, 0.0)}, (0.0, 0.0, 0.0))
    top = chain.update({"vo": (0.1, 0.0, 0.0), "icp": None}, (0.1, 0.0, 0.0))
    _close(top, (0.1, 0.0, 0.0))
    assert chain.fused.last_sources == ("vo", "wheel")


def test_the_back_end_correction_applies_on_top():
    chain = Chain("lidar")
    chain.update({"icp": (0.0, 0.0, 0.0)}, (0.0, 0.0, 0.0))
    chain.output_from_base((0.0, 0.0, 0.0))
    chain.update({"icp": (5.0, 0.0, 0.0)}, (5.0, 0.0, 0.0))
    chain.map_from_top = (0.0, 0.3, 0.0)
    _close(chain.output_from_base((5.0, 0.0, 0.0)), (5.0, 0.3, 0.0))


def test_without_wheels_the_estimate_is_the_vision_increments():
    chain = Chain("vision_noodom")
    chain.update({"vo": (7.0, 7.0, 1.0)}, (9.0, 9.0, 1.0))  # odom ignored
    _close(chain.output_from_base((9.0, 9.0, 1.0)), (0.0, 0.0, 0.0))
    chain.update({"vo": compose((7.0, 7.0, 1.0), (1.0, 2.0, 0.3))}, (0.0, 0.0, 0.0))
    _close(chain.output_from_base((0.0, 0.0, 0.0)), (1.0, 2.0, 0.3))
    # Vision lost: nothing moves the estimate.
    chain.update({"vo": None}, (0.0, 0.0, 0.0))
    _close(chain.output_from_base((0.0, 0.0, 0.0)), (1.0, 2.0, 0.3))


def test_camera_tf_quaternions_reproduce_the_rig_mounts():
    from pathlib import Path

    import numpy as np
    import yaml
    from forklift_ros.rig_slam_bridge import _quaternion_xyzw

    from forklift_core.geometry import rotation_matrix_from_quaternion_xyzw
    from forklift_core.sensors.camera_rig import rig_from_config

    root = Path(__file__).resolve().parents[4]
    rig = rig_from_config(
        yaml.safe_load((root / "config/isaac_depth_rig.yaml").read_text())
    )
    for camera in rig.cameras:
        rotation = np.asarray(camera.rotation_base_from_optical)
        np.testing.assert_allclose(
            rotation_matrix_from_quaternion_xyzw(_quaternion_xyzw(rotation)),
            rotation,
            atol=1e-12,
        )
