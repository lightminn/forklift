"""P0-4 4a: the Isaac actuation extracted from drive() makes the same robot calls (sim-to-real plan)."""

import ast
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
from isaac_plant import IsaacPlant  # noqa: E402

WHEELS, STEERS, LIFT = [3, 4, 5, 6], [1, 2], [7]


class Robot:
    def __init__(self):
        self.calls = []

    def apply_action(self, action):
        self.calls.append(("apply_action", action))

    def get_articulation_controller(self):
        self.calls.append(("get_articulation_controller", None))
        return "controller"


def action(**kw):
    return SimpleNamespace(**kw)


def same(a, b):
    assert set(vars(a)) == set(vars(b))
    for k in vars(a):
        x, y = getattr(a, k), getattr(b, k)
        if isinstance(x, np.ndarray) or isinstance(y, np.ndarray):
            assert isinstance(x, np.ndarray) and isinstance(y, np.ndarray) and x.dtype == y.dtype
            assert np.array_equal(x, y)
        else:
            assert x == y


def test_each_method_makes_the_call_drive_made():
    r = Robot()
    p = IsaacPlant(r, action, WHEELS, STEERS, LIFT)
    rates, steering = np.array([1.0, 2.0, 3.0, 4.0]), np.array([0.1, 0.2])
    p.zero_wheels()
    p.hold_lift(0.12)
    p.drive_wheels(rates, None)
    p.drive_wheels(rates, np.array([5.0, 6.0, 7.0, 8.0]))
    p.steer(steering)
    assert p.controller() == "controller"
    expected = [
        action(joint_velocities=np.zeros(len(WHEELS)), joint_indices=WHEELS),
        action(joint_positions=np.array([0.12]), joint_indices=LIFT),
        action(joint_velocities=rates, joint_positions=None, joint_indices=WHEELS),
        action(joint_velocities=rates, joint_positions=np.array([5.0, 6.0, 7.0, 8.0]), joint_indices=WHEELS),
        action(joint_positions=steering, joint_indices=STEERS),
    ]
    applied = [a for name, a in r.calls if name == "apply_action"]
    assert len(applied) == len(expected)
    for got, want in zip(applied, expected, strict=True):
        same(got, want)
    assert [name for name, _ in r.calls] == ["apply_action"] * 5 + ["get_articulation_controller"]
    assert applied[2].joint_velocities is rates and applied[4].joint_positions is steering  # passed through as is


def test_rebinding_the_robot_redirects_the_calls():
    a, b = Robot(), Robot()
    p = IsaacPlant(a, action, WHEELS, STEERS, LIFT)
    p.robot = b
    p.zero_wheels()
    assert a.calls == [] and len(b.calls) == 1


def test_drive_makes_no_direct_actuation_or_gain_call():
    """Every actuation in drive() goes through the plant (4a); the steering probe outside drive() is not part of it."""
    tree = ast.parse((ROOT / "sim" / "isaac" / "run_transport.py").read_text())
    drive = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "drive")
    direct = [n.lineno for n in ast.walk(drive)
              if isinstance(n, ast.Attribute) and n.attr in ("apply_action", "get_articulation_controller")
              and isinstance(n.value, ast.Name) and n.value.id == "robot"]
    assert direct == []
    plant_made = [n.lineno for n in ast.walk(drive) if isinstance(n, ast.Call)
                  and isinstance(n.func, ast.Name) and n.func.id == "IsaacPlant"]
    assert len(plant_made) == 1
