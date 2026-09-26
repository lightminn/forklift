"""Runner-level braking and freshness gating with an external SDK boundary.

The controller, Ackermann mapping, clearance guard, file writes and stop checking
remain real. Only acquisition/detection outputs and the simulator are substituted.
"""

import importlib.util
import json
import math
import sys
from dataclasses import replace
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.perception import pocket_detector
from forklift_core.perception.pocket_detector import (
    DetectionDiagnostics,
    DetectionResult,
)
from forklift_core.perception.pocket_observation import Pocket, PocketObservation
from forklift_core.perception.scene_dataset import SceneInput
from forklift_core.planning import Bounds
from forklift_core.sensors.rgbd import PinholeIntrinsics

ROOT = Path(__file__).resolve().parents[2]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def runner(monkeypatch):
    for name in (
        "run_transport",
        "run_perception_approach",
        "perception_camera",
        "insertion_geometry",
        "pocket_tracking_handoff",
    ):
        module = load_module(f"pocket_stop_{name}", ROOT / f"sim/isaac/{name}.py")
        monkeypatch.setitem(sys.modules, name, module)
    for name in (
        "isaacsim",
        "isaacsim.core",
        "isaacsim.core.utils",
        "isaacsim.core.utils.types",
    ):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    sys.modules["isaacsim.core.utils.types"].ArticulationAction = SimpleNamespace
    return load_module("pocket_stop_runner", ROOT / "sim/isaac/run_pocket_insertion.py")


class RobotBoundary:
    dof_names = [
        "front_left_spin",
        "front_right_spin",
        "rear_left_spin",
        "rear_right_spin",
        "left_steer",
        "right_steer",
        "fork_lift",
    ]

    def __init__(self):
        self.position = np.array([0.34, 0.0, 0.0])
        self.speed = 0.0
        self.other_linear = np.zeros(3)
        self.angular = np.zeros(3)
        self.target_speed = 0.0
        self.events = []
        self.world = None

    def get_world_pose(self):
        return self.position.copy(), np.array([1.0, 0, 0, 0])

    def get_linear_velocity(self):
        self.events.append(("measurement", self.world.current_time, self.speed))
        return np.array([self.speed, 0.0, 0.0]) + self.other_linear

    def get_angular_velocity(self):
        self.events.append(("angular_measurement", self.world.current_time))
        return self.angular.copy()

    def get_joint_positions(self):
        return np.zeros(7)

    def apply_action(self, action):
        if hasattr(action, "joint_velocities"):
            np.testing.assert_array_equal(action.joint_indices, [0, 1, 2, 3])
            self.target_speed = float(np.mean(action.joint_velocities)) * 0.135
            self.events.append(
                (
                    "zero" if self.target_speed == 0 else "drive",
                    self.world.current_time,
                    self.target_speed,
                )
            )


class WorldBoundary:
    def __init__(self, robot, *, brakes_work=True):
        self.robot = robot
        robot.world = self
        self.current_time = 0.0
        self.brakes_work = brakes_work
        self.on_step = lambda: None

    def step(self, *, render):
        assert render is False
        if self.robot.target_speed:
            self.robot.speed = self.robot.target_speed
        elif self.brakes_work:
            self.robot.speed *= 0.8
            self.robot.other_linear *= 0.8
            self.robot.angular *= 0.8
        self.robot.position[0] += self.robot.speed / 120
        self.current_time += 1 / 120
        self.robot.events.append(("physics", self.current_time))
        self.on_step()


def observation(stamp_ns, *, lost=False, x=0.9):
    obs = PocketObservation(
        stamp_ns,
        "synthetic",
        "base_link",
        "synthetic",
        "valid",
        Pocket((x, 0.145, 0.056), 0.12, 0.072),
        Pocket((x, -0.145, 0.056), 0.12, 0.072),
        0.0,
        0.001,
        0.001,
        None,
    )
    if lost:
        obs = replace(
            obs,
            status="invalid",
            left=None,
            right=None,
            insertion_yaw_rad=None,
            position_sigma_m=None,
            yaw_sigma_rad=None,
            reason="occluded",
        )
    return obs


def install_observations(
    monkeypatch, world, *, failure=None, lose_once=False, restored_x=0.9, initial_x=0.9
):
    def acquire(camera, actual_world, robot):
        assert actual_world is world
        robot.events.append(("capture", world.current_time))
        if failure == "acquisition" and world.current_time >= 0.29:
            raise RuntimeError("acquisition unavailable")
        transform = RigidTransform("base_link", "world", np.eye(3), robot.position)
        scene = SceneInput(
            np.full((2, 2, 3), 73, np.uint8),
            np.ones((2, 2)),
            PinholeIntrinsics(2, 2, 1, 1, 1, 1, "camera_optical_frame"),
            RigidTransform("camera_optical_frame", "base_link", np.eye(3), [0, 0, 0]),
            round(world.current_time * 1e9),
            "synthetic",
            "synthetic",
        )
        return scene, transform

    def detect(scene, prior, params):
        if failure == "detection" and world.current_time >= 0.29:
            raise RuntimeError("detection unavailable")
        lost = lose_once and 0.29 <= world.current_time < 0.39
        obs = observation(
            scene.stamp_ns,
            lost=lost,
            x=restored_x if world.current_time >= 0.39 else initial_x,
        )
        diagnostics = DetectionDiagnostics(
            None, 0, 0, (), {}, None, 0.001, False, 0.0, 0, None, None, None, None
        )
        return DetectionResult(obs, diagnostics)

    monkeypatch.setattr(
        sys.modules["perception_camera"], "acquire_frozen_snapshot", acquire
    )
    monkeypatch.setattr(pocket_detector, "detect_pockets", detect)


def execute(
    runner,
    world,
    robot,
    output,
    *,
    duration=2.0,
    controller=None,
    tracking=None,
    pallet_geometry=None,
):
    pallet = SimpleNamespace(
        get_world_pose=lambda: (np.array([5.0, 0, 0]), np.array([1.0, 0, 0, 0]))
    )
    args = SimpleNamespace(
        output=output,
        video=False,
        max_tracking_s=duration,
        drop_after_s=None,
        forklift_urdf=ROOT / "sim/models/dls08_provisional/forklift.urdf",
        pallet_urdf=ROOT / "sim/models/epal6_pallet/pallet.urdf",
    )
    return runner.follow_pockets(
        world,
        robot,
        pallet,
        None,
        None,
        SimpleNamespace(props=[], bounds=Bounds(-10, 10, -10, 10)),
        {},
        {
            "controller": {"max_frame_interval_ns": 110_000_000, **(controller or {})},
            **(tracking or {}),
        },
        None,
        None,
        args,
        reference_target=reference_target(),
        pallet_geometry=pallet_geometry,
    )


@pytest.mark.parametrize("failure", ["acquisition", "detection"])
@pytest.mark.parametrize("brakes_work", [True, False])
def test_capture_or_detection_failure_measures_physical_stop_before_reraising(
    runner, monkeypatch, tmp_path, failure, brakes_work
):
    robot = RobotBoundary()
    world = WorldBoundary(robot, brakes_work=brakes_work)
    install_observations(monkeypatch, world, failure=failure)
    reason = f"{failure} unavailable" if brakes_work else "Physical stop failed"
    with pytest.raises(RuntimeError, match=reason):
        execute(runner, world, robot, tmp_path)
    assert any(event[0] == "drive" for event in robot.events)
    last_zero = max(i for i, event in enumerate(robot.events) if event[0] == "zero")
    settling = robot.events[last_zero + 1 :]
    assert sum(event[0] == "physics" for event in settling) == 120
    assert settling[-2][0] == "measurement"
    assert settling[-1][0] == "angular_measurement"
    assert settling[-1][1] - robot.events[last_zero][1] == pytest.approx(1.0)
    stop = json.loads((tmp_path / "pocket_braking.json").read_text())
    assert stop["zero_wheel_setpoint_applied"] is True
    assert stop["stopped"] is brakes_work
    assert stop["measured_speed_mps"] == pytest.approx(robot.speed)
    assert stop["angular_speed_rad_s"] == 0.0
    assert (stop["measured_speed_mps"] < 0.012) is brakes_work
    assert (tmp_path / "pocket_observations.json").exists()
    assert not (tmp_path / "pocket_stop.json").exists()


def test_loss_latch_blocks_drive_even_after_fresh_frames_reconfirm_tracking(
    runner, monkeypatch, tmp_path
):
    robot = RobotBoundary()
    world = WorldBoundary(robot)
    install_observations(monkeypatch, world, lose_once=True)
    result = execute(runner, world, robot, tmp_path)
    rows = json.loads((tmp_path / "pocket_observations.json").read_text())
    lost = next(row for row in rows if row["command"]["status"] == "lost")
    assert any(
        row["command"]["speed_mps"] > 0
        for row in rows
        if row["time_s"] > lost["time_s"]
    )
    assert any(event[0] == "drive" for event in robot.events)
    assert all(
        event[0] != "drive" for event in robot.events if event[1] >= lost["time_s"]
    )
    assert result["success"] is False
    assert result["status"] == "stopped_observation_loss"
    assert result["loss_latched_for_this_trial"] is True
    assert result["stopped_speed_mps"] < 0.012


def test_watchdog_latches_zero_through_later_fresh_camera_samples(
    runner, monkeypatch, tmp_path
):
    robot = RobotBoundary()
    world = WorldBoundary(robot)
    install_observations(monkeypatch, world)
    result = execute(
        runner,
        world,
        robot,
        tmp_path,
        duration=1.2,
        controller={"confirmation_frames": 1, "max_observation_age_ns": 25_000_000},
    )
    assert sum(event[0] == "capture" for event in robot.events) > 3
    assert result["loss_latched_for_this_trial"] is True
    assert result["status"] == "stopped_observation_loss"
    assert result["success"] is False
    assert any(event[0] == "drive" for event in robot.events)
    wheel_events = [event for event in robot.events if event[0] in {"drive", "zero"}]
    first_zero = next(event for event in wheel_events if event[0] == "zero")
    assert first_zero[1] == pytest.approx(4 / 120)
    assert all(event[0] == "zero" for event in wheel_events if event[1] > 0.025)


def test_loss_latch_cannot_report_success_from_later_goal_observation(
    runner, monkeypatch, tmp_path
):
    robot = RobotBoundary()
    world = WorldBoundary(robot)
    install_observations(monkeypatch, world, lose_once=True, restored_x=0.59)
    result = execute(runner, world, robot, tmp_path)
    assert result["loss_latched_for_this_trial"] is True
    assert result["success"] is False
    assert result["status"] == "stopped_observation_loss"


@pytest.mark.parametrize("motion", ["lateral", "vertical", "angular"])
@pytest.mark.parametrize("brakes_work", [True, False])
def test_nonforward_motion_blocks_acquisition_and_requires_physical_stop(
    runner, monkeypatch, tmp_path, motion, brakes_work
):
    robot = RobotBoundary()
    if motion == "angular":
        robot.angular[2] = 0.05
    else:
        robot.other_linear[1 if motion == "lateral" else 2] = 0.1
    world = WorldBoundary(robot, brakes_work=brakes_work)
    install_observations(monkeypatch, world)
    if brakes_work:
        result = execute(runner, world, robot, tmp_path)
        assert result["success"] is False
        assert result["tracking_motion_commanded"] is False
        assert result["loss_latched_for_this_trial"] is True
    else:
        with pytest.raises(RuntimeError, match="Physical stop failed"):
            execute(runner, world, robot, tmp_path)
    assert not any(event[0] == "drive" for event in robot.events)
    stop = json.loads((tmp_path / "pocket_braking.json").read_text())
    assert stop["stopped"] is brakes_work
    assert stop["angular_speed_rad_s"] == pytest.approx(np.linalg.norm(robot.angular))
    assert stop["measured_speed_mps"] == pytest.approx(
        np.linalg.norm(robot.other_linear)
    )
    if not brakes_work and motion == "angular":
        assert stop["measured_speed_mps"] == 0.0
        assert stop["angular_speed_rad_s"] == 0.05


def test_excess_angular_motion_aborts_tracking_and_measures_rotation_after_settling(
    runner, monkeypatch, tmp_path
):
    robot = RobotBoundary()
    world = WorldBoundary(robot)
    install_observations(monkeypatch, world)

    def rotate_after_first_drive():
        if world.current_time < 0.01:
            robot.angular[2] = 0.09

    world.on_step = rotate_after_first_drive
    with pytest.raises(RuntimeError, match="Truck angular speed limit"):
        execute(runner, world, robot, tmp_path, controller={"confirmation_frames": 1})
    assert any(event[0] == "drive" for event in robot.events)
    assert not any(event[0] == "drive" for event in robot.events if event[1] > 0)
    stop = json.loads((tmp_path / "pocket_braking.json").read_text())
    assert stop["stopped"] is True
    assert stop["angular_speed_rad_s"] < 0.012


def test_final_settling_rotation_cannot_be_reported_as_a_physical_stop(
    runner, monkeypatch, tmp_path
):
    robot = RobotBoundary()
    world = WorldBoundary(robot)
    install_observations(monkeypatch, world)

    def rotation_during_final_settling():
        if world.current_time > 0.095:
            robot.angular[2] = 0.05

    world.on_step = rotation_during_final_settling
    with pytest.raises(RuntimeError, match="Physical stop failed"):
        execute(
            runner,
            world,
            robot,
            tmp_path,
            duration=0.09,
            controller={"confirmation_frames": 1},
        )
    assert any(event[0] == "drive" for event in robot.events)
    stop = json.loads((tmp_path / "pocket_braking.json").read_text())
    assert stop["measured_speed_mps"] < 0.012
    assert stop["angular_speed_rad_s"] == 0.05
    assert stop["stopped"] is False
    assert not (tmp_path / "pocket_stop.json").exists()


def reference_target():
    return SimpleNamespace(
        success=True,
        front_midpoint_world_m=np.array([1.24, 0.0, 0.056]),
        pallet_site=SimpleNamespace(yaw_rad=0.0),
    )


def test_association_rejects_another_pallet_with_valid_per_frame_geometry(runner):
    transform = RigidTransform("base_link", "world", np.eye(3), [0.34, 0, 0])
    associated = runner.associate_observation(
        observation(0, x=0.96), transform, reference_target()
    )
    assert associated["matched"] is False
    assert associated["position_error_m"] == pytest.approx(0.06)


def test_association_accepts_same_world_pallet_after_robot_translation_and_rotation(
    runner,
):
    target = reference_target()
    initial = RigidTransform("base_link", "world", np.eye(3), [0.34, 0, 0])
    translated = RigidTransform("base_link", "world", np.eye(3), [0.54, 0, 0])
    rotated = RigidTransform(
        "base_link", "world", [[0, -1, 0], [1, 0, 0], [0, 0, 1]], [1.24, -0.9, 0]
    )
    # For a base yaw +90 deg, the world's +x insertion axis lies at base -90 deg.
    rotated_observation = replace(
        observation(2),
        left=Pocket((1.045, 0, 0.056), 0.12, 0.072),
        right=Pocket((0.755, 0, 0.056), 0.12, 0.072),
        insertion_yaw_rad=-math.pi / 2,
    )
    for obs, transform in [
        (observation(0), initial),
        (observation(1, x=0.7), translated),
        (rotated_observation, rotated),
    ]:
        associated = runner.associate_observation(obs, transform, target)
        assert associated["matched"] is True
        assert associated["position_error_m"] == pytest.approx(0, abs=1e-14)
        assert associated["yaw_error_rad"] == pytest.approx(0, abs=1e-14)


def test_association_rejects_yaw_jump_despite_unchanged_world_midpoint(runner):
    transform = RigidTransform("base_link", "world", np.eye(3), [0.34, 0, 0])
    associated = runner.associate_observation(
        replace(observation(0), insertion_yaw_rad=0.04), transform, reference_target()
    )
    assert associated["matched"] is False
    assert associated["position_error_m"] == pytest.approx(0, abs=1e-14)
    assert associated["yaw_error_rad"] == pytest.approx(0.04)


@pytest.mark.parametrize(
    "source,target", [("camera_optical_frame", "world"), ("base_link", "map")]
)
def test_association_rejects_wrong_transform_frames(runner, source, target):
    transform = RigidTransform(source, target, np.eye(3), [0.34, 0, 0])
    with pytest.raises(RuntimeError, match="transform frames"):
        runner.associate_observation(observation(0), transform, reference_target())


def test_association_cannot_match_invalid_observation(runner):
    transform = RigidTransform("base_link", "world", np.eye(3), [0.34, 0, 0])
    associated = runner.associate_observation(
        observation(0, lost=True), transform, reference_target()
    )
    assert associated["matched"] is False


def test_failed_association_blocks_motion_despite_repeated_valid_detections(
    runner, monkeypatch, tmp_path
):
    robot = RobotBoundary()
    world = WorldBoundary(robot)
    install_observations(monkeypatch, world, initial_x=0.96, restored_x=0.96)
    result = execute(runner, world, robot, tmp_path)
    rows = json.loads((tmp_path / "pocket_observations.json").read_text())
    assert len(rows) >= 3
    assert all(row["detection"]["observation"]["status"] == "valid" for row in rows)
    assert all(not row["initial_target_association"]["matched"] for row in rows)
    assert not any(event[0] == "drive" for event in robot.events)
    assert result["success"] is False
    assert result["loss_latched_for_this_trial"] is True
    assert result["tracking_motion_commanded"] is False


@pytest.mark.parametrize("acceleration", [0.24, 0.6])
def test_high_speed_request_accelerates_within_limit_but_loss_commands_zero_immediately(
    runner, monkeypatch, tmp_path, acceleration
):
    robot = RobotBoundary()
    world = WorldBoundary(robot)
    install_observations(monkeypatch, world, lose_once=True)
    result = execute(
        runner,
        world,
        robot,
        tmp_path,
        controller={
            "confirmation_frames": 1,
            "cruise_speed_mps": 0.6,
            "max_speed_mps": 0.75,
            "slowdown_front_x_m": 0.8,
            "approach_gain_inv_s": 3.0,
        },
        tracking={"acceleration_mps2": acceleration},
    )
    rows = json.loads((tmp_path / "pocket_observations.json").read_text())
    assert rows[0]["command"]["speed_mps"] == 0.6
    lost_time = next(
        row["time_s"] for row in rows if row["command"]["status"] == "lost"
    )
    actions = [event for event in robot.events if event[0] in {"drive", "zero"}]
    speeds = np.array([event[2] for event in actions])
    increments = np.diff(np.r_[0.0, speeds])
    assert speeds[0] == pytest.approx(acceleration / 120)
    assert np.all(increments <= acceleration / 120 + 1e-12)
    assert speeds.max() > 5 * acceleration / 120
    loss_index = next(
        index for index, event in enumerate(actions) if event[1] >= lost_time
    )
    assert actions[loss_index][1] == pytest.approx(lost_time)
    assert actions[loss_index][2] == 0.0
    assert actions[loss_index - 1][2] > 5 * acceleration / 120
    assert all(event[2] == 0.0 for event in actions[loss_index:])
    assert result["success"] is False
    assert result["loss_latched_for_this_trial"] is True


@pytest.mark.parametrize("rotating_at_acquisition", [False, True])
def test_configured_turning_limit_allows_tracking_but_acquisition_still_requires_rest(
    runner, monkeypatch, tmp_path, rotating_at_acquisition
):
    robot = RobotBoundary()
    world = WorldBoundary(robot)
    if rotating_at_acquisition:
        robot.angular[2] = 0.18
    else:

        def start_turn_after_first_drive():
            if world.current_time < 0.01:
                robot.angular[2] = 0.18

        world.on_step = start_turn_after_first_drive
    install_observations(monkeypatch, world)
    result = execute(
        runner,
        world,
        robot,
        tmp_path,
        duration=0.35,
        controller={"confirmation_frames": 1},
        tracking={"max_angular_speed_rad_s": 0.3},
    )
    rows = json.loads((tmp_path / "pocket_observations.json").read_text())
    drives = [event for event in robot.events if event[0] == "drive"]
    if rotating_at_acquisition:
        assert not drives
        assert rows[0]["rotation_blocks_acquisition"] is True
        assert result["loss_latched_for_this_trial"] is True
    else:
        assert any(event[1] >= 0.2 for event in drives)
        assert any(
            row["angular_speed_rad_s"] == 0.18
            and row["command"]["status"] == "tracking"
            for row in rows
        )
        assert result["loss_latched_for_this_trial"] is False
    stop = json.loads((tmp_path / "pocket_braking.json").read_text())
    assert stop["stopped"] is True
    assert stop["angular_speed_rad_s"] < 0.012


def test_qualified_roof_survives_front_loss_then_its_own_loss_brakes(
    runner, monkeypatch, tmp_path
):
    from forklift_core.perception import roof_tracking

    robot = RobotBoundary()
    world = WorldBoundary(robot)
    install_observations(monkeypatch, world, lose_once=True)

    def track(scene, geometry, expected_front, expected_yaw):
        diagnostics = DetectionDiagnostics(
            None, 0, 0, (), {}, None, 0.001, False, 0.0, 0, None, None, None, None
        )
        return DetectionResult(
            observation(scene.stamp_ns, lost=world.current_time >= 0.59), diagnostics
        )

    monkeypatch.setattr(roof_tracking, "track_roof", track)
    result = execute(
        runner,
        world,
        robot,
        tmp_path,
        tracking={"roof_tracking": {}},
        pallet_geometry=object(),
    )
    rows = json.loads((tmp_path / "pocket_observations.json").read_text())
    front_lost = next(r for r in rows if 0.29 <= r["time_s"] < 0.39)
    assert front_lost["detection"]["observation"]["status"] == "invalid"
    assert front_lost["observation_mode"] == "roof_model"
    assert front_lost["command"]["status"] == "tracking"
    loss = next(r for r in rows if r["command"]["status"] == "lost")
    assert loss["time_s"] == pytest.approx(0.6)
    assert result["status"] == "stopped_observation_loss"
    assert result["roof_tracking_qualified"] is True
    assert result["stopped_speed_mps"] < 0.012
    assert not any(e[0] == "drive" and e[1] >= loss["time_s"] for e in robot.events)
