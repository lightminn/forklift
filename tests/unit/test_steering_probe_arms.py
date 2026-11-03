"""The steering left-asymmetry plan's probe arms (docs/plans/2026-11-03-steering-left-asymmetry.md section 4.2): the
pure parts of sim/isaac/steering_probe_arms.py and the runner's wiring (probe only, nothing changes without them)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))
import steering_probe_arms as arms  # noqa: E402

SCRIPT = ROOT / "sim/isaac/run_transport.py"


def test_the_runner_lists_the_same_arm_keys_without_importing_the_module():
    source = SCRIPT.read_text()
    namespace = {}
    exec(source[source.index("DIAGNOSTIC_PROBE_SETTINGS = "): source.index("TYRE_LINKS = ")], namespace)  # noqa: S102
    assert namespace["PROBE_ARM_SETTINGS"] == arms.PROBE_ARM_SETTINGS
    # the module is imported only under --steering-probe (or when a probe key is present, to refuse it)
    head = source[: source.index("def main() -> None:")]
    assert "import steering_probe_arms" not in head.split("def run_steering_probe(")[0]


def test_the_runner_refuses_arm_keys_and_the_geometry_urdf_outside_a_probe():
    source = SCRIPT.read_text()
    main = source[source.index("def validate_before_isaac("):]
    guard = main.index("PROBE_ARM_SETTINGS")
    block = main[guard: guard + 600]
    assert 'require(args.steering_probe, "probe arm settings and --probe-geometry-urdf: --steering-probe only")' in block
    assert "arm_problems(settings)" in block
    assert "args.forklift_urdf if args.probe_geometry_urdf is None else args.probe_geometry_urdf" in main


@pytest.mark.parametrize("settings,problem", [
    ({"probe_load_kg": 4.93}, "probe load needs all"),
    ({"probe_load_kg": 4.93, "probe_load_dx_m": 0.93, "probe_load_dz_m": 0.13, "probe_lift_m": 0.3}, "lift travel"),
    ({"probe_shim_wheel": 1}, "probe shim needs both"),
    ({"probe_shim_wheel": 3, "probe_shim_height_m": 0.001}, "1 (front left) or 2 (front right)"),
    ({"probe_shim_wheel": 1, "probe_shim_height_m": 0.02}, "(0, 0.01]"),
    ({"probe_front_spin_free": 2}, "is 1 or absent"),
    ({"probe_fix_base_z_m": 0.5}, "(0, 0.2]"),
    ({"probe_spawn_yaw_offset_rad": 4.0}, "(0, pi]"),
])
def test_arm_problems(settings, problem):
    assert any(problem in p for p in arms.arm_problems(settings))


def test_registered_arm_settings_are_well_formed():
    good = [
        {},
        {"probe_fix_base_z_m": 1e-6}, {"probe_fix_base_z_m": 0.05},
        {"probe_load_kg": 4.93, "probe_load_dx_m": 0.93, "probe_load_dz_m": 0.1304, "probe_lift_m": 0.178},
        {"probe_front_tyre_frictionless": 1},
        {"probe_shim_wheel": 1, "probe_shim_height_m": 0.001}, {"probe_shim_wheel": 2, "probe_shim_height_m": 0.001},
        {"probe_spawn_yaw_offset_rad": math.pi}, {"probe_spawn_yaw_offset_rad": math.pi / 2},
        {"probe_assign_by_y": 1}, {"probe_front_spin_free": 1},
    ]
    for settings in good:
        assert arms.arm_problems(settings) == [], settings


WHEELS = {"front_left_wheel": [0.32, 0.30, 0.0], "front_right_wheel": [0.32, -0.30, 0.0],
          "rear_left_wheel": [-0.34, 0.265, 0.0], "rear_right_wheel": [-0.34, -0.265, 0.0], "base_link": [0, 0, 0]}


def test_physical_assignment_keeps_the_names_of_an_unswapped_model():
    place = arms.physical_assignment(WHEELS)
    assert place == {"steer": ["left_steer", "right_steer"],
                     "spin": ["front_left_spin", "front_right_spin", "rear_left_spin", "rear_right_spin"],
                     "tyre_links": ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]}


def test_physical_assignment_follows_the_side_of_a_name_swapped_model():
    swapped = {n.replace("left", "@").replace("right", "left").replace("@", "right"): p for n, p in WHEELS.items()}
    place = arms.physical_assignment(swapped)
    assert place["steer"] == ["right_steer", "left_steer"]
    assert place["spin"] == ["front_right_spin", "front_left_spin", "rear_right_spin", "rear_left_spin"]
    assert place["tyre_links"][0] == "front_right_wheel"


def test_physical_assignment_refuses_two_wheels_on_one_side():
    bad = dict(WHEELS, front_right_wheel=[0.32, 0.10, 0.0])
    with pytest.raises(ValueError, match="one per side"):
        arms.physical_assignment(bad)


def test_composite_body_is_the_parallel_axis_sum():
    # two point-ish masses 1 m apart on x: the combined COM in between, Iyy = Izz = m1 m2 d^2 / (m1 + m2)
    m, c, inertia = arms.composite_body(2.0, [0, 0, 0], np.zeros((3, 3)), 3.0, [1, 0, 0], np.zeros((3, 3)))
    assert m == 5.0
    np.testing.assert_allclose(c, [0.6, 0, 0])
    np.testing.assert_allclose(np.diag(inertia), [0.0, 1.2, 1.2])
    # own inertias carry over unchanged when the centres coincide
    i1 = np.diag([0.1, 0.2, 0.3])
    m, c, inertia = arms.composite_body(1.0, [0.5, 0, 0], i1, 1.0, [0.5, 0, 0], i1)
    np.testing.assert_allclose(inertia, 2 * i1)


def test_load_in_carriage_moves_into_the_carriage_frame():
    rotation = np.eye(3)
    translation = np.array([0.6, 0.0, 0.33])  # the carriage at the load's lift
    com = arms.load_in_carriage((rotation, translation), [0.93, 0.0, 0.1304], [0.0, 0.0, 0.07])
    np.testing.assert_allclose(com, [0.33, 0.0, 0.1304 + 0.07 - 0.33])


def test_steer_axis_moment_is_the_z_of_r_cross_j():
    # a +y impulse 0.1 m ahead of the axis turns it counter-clockwise (+z)
    assert arms.steer_axis_moment_z([[1.1, 0.0, 0.0]], [[0.0, 2.0, -5.0]], [1.0, 0.0, 0.3]) == pytest.approx(0.2)
    assert arms.steer_axis_moment_z([[1.0, 0.1, 0.0]], [[2.0, 0.0, 0.0]], [1.0, 0.0, 0.0]) == pytest.approx(-0.2)


def test_quat_matrix_turns_x_into_y_at_a_quarter_turn():
    np.testing.assert_allclose(arms.quat_matrix(arms.yaw_quat_wxyz(math.pi / 2)) @ [1, 0, 0], [0, 1, 0], atol=1e-12)


def test_an_unset_geometry_urdf_is_not_recorded():
    # every other run records the arguments it always did (Codex review P3)
    source = SCRIPT.read_text()
    block = source[source.index("def recorded_arguments("): source.index("def return_start_backoff_m_of(")]
    assert 'and (k != "probe_geometry_urdf" or v is not None)' in block
