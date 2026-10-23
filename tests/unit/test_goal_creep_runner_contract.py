"""The runner's half of the goal creep watch (docs/plans/2026-10-24-goal-creep-stall.md G2, "적용 명령의 값"): the
wheel speed each physics step runs on goes to slam["applied_speed"] -- after the wheel command is applied, and as 0 where
a step stops the wheels itself (a preempt, a gyro fault, the live zero) -- and the step's watch carries it, so a
replay restores what a step changed. Source checks: the runner needs Isaac to run."""

import re
from pathlib import Path

SOURCE = (Path(__file__).resolve().parents[2] / "sim/isaac/run_transport.py").read_text()


def test_the_applied_wheel_speed_follows_the_wheel_command():
    apply_ = SOURCE.index("joint_velocities=np.asarray(drive.wheel_rates_rad_s)")
    write_ = SOURCE.index('slam["applied_speed"] = float(drive.speed_mps)')
    step_ = SOURCE.index('stepper["fn"]((pocket_frames and step % 12 == 0) or capturing_)')
    assert apply_ < write_ < step_


def test_every_step_that_stops_the_wheels_itself_writes_zero():
    zeros = [m.start() for m in re.finditer(r"joint_velocities=np\.zeros\(len\(wheels\)\), joint_indices=wheels\)\)",
                                            SOURCE)]
    assert len(zeros) == 3  # the preempt, the gyro fault, the live zero
    for start in zeros:
        assert 'slam["applied_speed"] = 0.0' in SOURCE[start:start + 700]


def test_the_step_watch_carries_it():
    watch = SOURCE[SOURCE.index("step_watch = DictWatch("):SOURCE.index("append_only=", SOURCE.index("step_watch = DictWatch("))]
    assert '"applied_speed"' in watch


def test_the_applied_speed_is_not_in_the_end_state_comparison(tmp_path):
    # Codex implementation round 2 P2: the runner writes it in replays too, and a format 2 record made before it has
    # no such key -- the end state leaves it out (each tick's outputs already carry the wheel command it copies).
    import importlib.util

    from forklift_core.mission.state import MissionState

    spec = importlib.util.spec_from_file_location("run_transport_goal_creep", Path(__file__).resolve().parents[2]
                                                  / "sim/isaac/run_transport.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ms = MissionState(phase="return_home")
    old = module.final_projection({"slam": {"odom_speed": 0.0}}, ms, tmp_path)
    new = module.final_projection({"slam": {"odom_speed": 0.0, "applied_speed": -3.6e-5}}, ms, tmp_path)
    assert old == new
