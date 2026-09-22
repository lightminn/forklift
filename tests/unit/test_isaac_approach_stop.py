"""Failure braking through the real approach runner at a minimal SDK boundary.

The former finally block sent a zero setpoint but skipped physics and stop
measurement when tracking raised. These tests keep the real tracker and geometry
checks; the fake world injects a deviation after the first moving command.
"""

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from forklift_core.planning import Bounds, Rectangle

ROOT = Path(__file__).resolve().parents[2]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def runner(monkeypatch):
    transport = load_module(
        "approach_stop_transport", ROOT / "sim/isaac/run_transport.py"
    )
    monkeypatch.setitem(sys.modules, "run_transport", transport)
    for name in (
        "isaacsim",
        "isaacsim.core",
        "isaacsim.core.utils",
        "isaacsim.core.utils.types",
    ):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    sys.modules["isaacsim.core.utils.types"].ArticulationAction = SimpleNamespace
    return load_module(
        "approach_stop_runner", ROOT / "sim/isaac/run_perception_approach.py"
    )


class RobotBoundary:
    dof_names = [
        "front_left_spin",
        "front_right_spin",
        "rear_left_spin",
        "rear_right_spin",
        "left_steer",
        "right_steer",
    ]

    def __init__(self):
        self.position = np.array([0.34, 0.0, 0.0])
        self.speed = 0.1
        self.zero_commanded = False
        self.events = []
        self.world = None

    def get_world_pose(self):
        return self.position.copy(), np.array([1.0, 0, 0, 0])

    def get_linear_velocity(self):
        self.events.append(("measurement", self.world.current_time, self.speed))
        return np.array([self.speed, 0.0, 0.0])

    def get_joint_positions(self):
        return np.zeros(6)

    def apply_action(self, action):
        if hasattr(action, "joint_velocities"):
            self.zero_commanded = bool(np.all(action.joint_velocities == 0))
            self.events.append(
                ("zero" if self.zero_commanded else "drive", self.world.current_time)
            )
            np.testing.assert_array_equal(action.joint_indices, [0, 1, 2, 3])


class WorldBoundary:
    def __init__(self, robot, brakes_work):
        self.robot = robot
        robot.world = self
        self.current_time = 0.0
        self.brakes_work = brakes_work

    def step(self, *, render):
        assert render is False
        if self.robot.zero_commanded:
            if self.brakes_work:
                self.robot.speed *= 0.9
        else:
            # An SDK-reported deviation triggers the real tracker or SAT guard.
            self.robot.position[1] = 1.0
        self.current_time += 1 / 120
        self.robot.events.append(("physics", self.current_time))


def execute(runner, world, robot, output, collision):
    path = SimpleNamespace(
        poses=np.array([[0.0, 0, 0], [1.0, 0, 0]]),
        directions=np.array([1, 1]),
        curvatures_inv_m=np.zeros(2),
        length_m=1.0,
    )
    return runner.follow_approach(
        world,
        robot,
        None,
        path,
        [Rectangle(0.5, 1, 0.1, 0.1)] if collision else [],
        Bounds(-3, 3, -3, 3),
        {
            "approach_speed_mps": 0.2,
            "tracker_curvature_inv_m": 0.62,
            "drive_acceleration_mps2": 0.3,
            "steering_command_rate_rad_s": 1.0,
        },
        output,
        False,
    )


@pytest.mark.parametrize("collision", [False, True])
def test_control_failure_brakes_and_measures_stop_before_reraising(
    runner, tmp_path, collision
):
    robot = RobotBoundary()
    world = WorldBoundary(robot, brakes_work=True)
    reason = "Truck overlap" if collision else "Tracking failed"
    with pytest.raises(RuntimeError, match=reason):
        execute(runner, world, robot, tmp_path, collision)

    # A physical response after the zero command, not sending zero alone, must
    # precede the measured stop record and preservation of the original failure.
    assert any(event[0] == "drive" for event in robot.events)
    zero_index = next(i for i, event in enumerate(robot.events) if event[0] == "zero")
    after_zero = robot.events[zero_index + 1 :]
    assert after_zero[0][0] == "physics"
    assert after_zero[-1][0] == "measurement"
    assert after_zero[-1][1] - robot.events[zero_index][1] == pytest.approx(1.0)
    stop = json.loads((tmp_path / "stop.json").read_text())
    assert stop["zero_wheel_setpoint_applied"] is True
    assert stop["stopped"] is True
    assert stop["measured_speed_mps"] == pytest.approx(robot.speed)
    assert stop["measured_speed_mps"] < 0.012
    tracking = json.loads((tmp_path / "tracking.json").read_text())
    assert tracking[0]["signed_speed_mps"] == 0.1
    assert not (tmp_path / "aligned.png").exists()


def test_zero_setpoint_without_physical_stopping_is_recorded_as_failure(
    runner, tmp_path
):
    robot = RobotBoundary()
    world = WorldBoundary(robot, brakes_work=False)
    with pytest.raises(RuntimeError, match="Robot did not stop"):
        execute(runner, world, robot, tmp_path, collision=False)
    stop = json.loads((tmp_path / "stop.json").read_text())
    assert stop["zero_wheel_setpoint_applied"] is True
    assert stop["stopped"] is False
    assert stop["measured_speed_mps"] == 0.1
    assert (tmp_path / "tracking.json").exists()
    assert not (tmp_path / "aligned.png").exists()
