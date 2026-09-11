"""Run a bounded synthetic Gazebo -> ROS -> bag -> fresh replay experiment."""

import argparse
import json
import math
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path


def fresh_output(root: Path) -> Path:
    """Allow runner infrastructure in the mount, but never reuse experiment data."""
    root.mkdir(parents=True, exist_ok=True)
    output = root / "sensor_smoke"
    output.mkdir(exist_ok=False)
    return output


class Children:
    """Own process groups only; never kill by a global process name."""

    def __init__(self, output: Path, env: dict):
        self.output = output
        self.env = env
        self.processes = []
        self.logs = []

    def start(self, name: str, command: list[str], cwd=None):
        log = (self.output / f"{name}.log").open("w")
        self.logs.append(log)
        process = subprocess.Popen(
            command,
            cwd=cwd,
            env=self.env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.processes.append((name, process, command))
        return process

    def wait(self, process, timeout_s: float, guards=()) -> None:
        deadline = time.monotonic() + timeout_s
        while process.poll() is None:
            for guard in guards:
                if guard.poll() is not None:
                    raise RuntimeError(
                        f"Required child {guard.pid} exited early: {guard.returncode}"
                    )
            if time.monotonic() > deadline:
                raise TimeoutError(f"Child {process.pid} exceeded {timeout_s}s")
            time.sleep(0.05)
        if process.returncode != 0:
            raise RuntimeError(
                f"Child {process.pid} failed with exit {process.returncode}"
            )

    @staticmethod
    def _wait_group(process, timeout_s: float) -> bool:
        """Wait for the owned group, including descendants of an exited leader."""
        deadline = time.monotonic() + timeout_s
        while True:
            process.poll()
            if process.returncode is not None:
                # As container PID 1, this runner may adopt orphan descendants.
                # Reap only this group, after Popen has reaped its own leader.
                while True:
                    try:
                        child, _ = os.waitpid(-process.pid, os.WNOHANG)
                    except ChildProcessError:
                        break
                    if child == 0:
                        break
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)

    def stop(self, process, grace_s=10.0) -> None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        if not self._wait_group(process, grace_s):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            if not self._wait_group(process, 5.0):
                raise RuntimeError(f"Owned group {process.pid} survived SIGKILL")
            raise RuntimeError(f"Child group {process.pid} required SIGKILL")
        if process.returncode not in (0, -signal.SIGINT, 130):
            raise RuntimeError(
                f"Child {process.pid} failed during shutdown: {process.returncode}"
            )

    def close(self) -> list[str]:
        errors = []
        for name, process, _ in reversed(self.processes):
            try:
                self.stop(process)
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        for log in self.logs:
            log.close()
        return errors


def wait_ready(path, process, timeout_s=30.0):
    deadline = time.monotonic() + timeout_s
    while not path.exists():
        if process.poll() is not None:
            raise RuntimeError(f"Validator exited before ready: {process.returncode}")
        if time.monotonic() > deadline:
            raise TimeoutError(f"No readiness marker: {path}")
        time.sleep(0.05)


def wait_clock(path: Path, timeout_s=30.0, guards=()) -> None:
    """Do not publish one-shot static TF before recording has a simulation clock."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if any(process.poll() is not None for process in guards):
            raise RuntimeError("Required child exited before the first clock")
        if path.exists() and json.loads(path.read_text())["counts"]["clock"] > 0:
            return
        time.sleep(0.05)
    raise TimeoutError("No actual /clock received before static TF startup")


def run(args) -> int:
    source = Path(__file__).resolve().parents[2]
    output = fresh_output(args.output.resolve())
    runtime = args.output.resolve() / ".runtime"
    runtime.mkdir(exist_ok=True)
    for name in ("home", "tmp", "pycache", "ros_logs"):
        (runtime / name).mkdir(exist_ok=True)
    env = os.environ.copy()
    env.pop("ROS_LOCALHOST_ONLY", None)
    env.update(
        HOME=str(runtime / "home"),
        TMPDIR=str(runtime / "tmp"),
        PYTHONPYCACHEPREFIX=str(runtime / "pycache"),
        ROS_LOG_DIR=str(runtime / "ros_logs"),
        ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
        ROS_DOMAIN_ID=str(20 + os.getpid() % 180),
        GZ_IP="127.0.0.1",
        GZ_PARTITION=f"forklift_sensor_{os.getpid()}",
        LIBGL_ALWAYS_SOFTWARE="1",
    )
    children = Children(output, env)
    result = {
        "passed": False,
        "source_provenance": "synthetic",
        "validation_kind": "sensor_rendering",
        "duration_required_s": args.duration,
        "environment": {
            key: env[key]
            for key in [
                "ROS_DOMAIN_ID",
                "ROS_AUTOMATIC_DISCOVERY_RANGE",
                "GZ_IP",
                "GZ_PARTITION",
                "LIBGL_ALWAYS_SOFTWARE",
            ]
        },
    }
    start = time.monotonic()
    try:
        generate = children.start(
            "generate",
            [
                sys.executable,
                str(source / "sim/gazebo/build_sensor_world.py"),
                "--output",
                str(output),
            ],
        )
        children.wait(generate, 30)
        build = children.start(
            "colcon_build",
            [
                "colcon",
                "--log-base",
                str(runtime / "colcon_log"),
                "build",
                "--base-paths",
                str(source / "ros2/src"),
                "--build-base",
                str(runtime / "build"),
                "--install-base",
                str(runtime / "install"),
                "--event-handlers",
                "console_direct+",
            ],
            cwd=runtime,
        )
        children.wait(build, 120)
        setup = runtime / "install/setup.bash"
        installed = subprocess.run(
            ["bash", "-c", f"source {shlex.quote(str(setup))} && env -0"],
            env=env,
            capture_output=True,
            timeout=20,
            check=True,
        )
        env.update(
            dict(
                item.split("=", 1)
                for item in installed.stdout.decode().split("\0")
                if "=" in item
            )
        )
        children.env = env
        test = children.start(
            "colcon_test",
            [
                "colcon",
                "--log-base",
                str(runtime / "test_log"),
                "test",
                "--base-paths",
                str(source / "ros2/src"),
                "--build-base",
                str(runtime / "build"),
                "--install-base",
                str(runtime / "install"),
                "--event-handlers",
                "console_direct+",
                "--return-code-on-test-failure",
                "--python-testing",
                "pytest",
                "--pytest-args",
                "-p",
                "no:cacheprovider",
            ],
            cwd=runtime,
        )
        children.wait(test, 120)
        # Installed module import must work outside the source tree.
        check = children.start(
            "installed_import",
            [
                sys.executable,
                "-c",
                "import forklift_ros.observation as m; print(m.__file__); assert '/install/' in m.__file__",
            ],
            cwd=runtime,
        )
        children.wait(check, 20)
        qos = output / "bag_qos.yaml"
        qos.write_text(
            "/tf_static:\n  reliability: reliable\n  durability: transient_local\n  history: keep_last\n  depth: 10\n"
        )
        topics = [
            "/camera/image",
            "/camera/depth_image",
            "/camera/camera_info",
            "/scan",
            "/clock",
            "/tf_static",
        ]
        validator = [
            "ros2",
            "run",
            "forklift_ros",
            "sensor_validator",
            "--duration",
            str(args.duration),
            "--wall-timeout",
            str(args.wall_timeout),
        ]
        live = children.start(
            "live_validator",
            [*validator, "--phase", "live", "--output", str(output / "live")],
            cwd=runtime,
        )
        wait_ready(output / "live/ready.json", live)
        recorder = children.start(
            "bag_record",
            [
                "ros2",
                "bag",
                "record",
                "--storage",
                "sqlite3",
                "--output",
                str(output / "bag"),
                "--use-sim-time",
                "--disable-keyboard-controls",
                "--qos-profile-overrides-path",
                str(qos),
                "--polling-interval",
                "100",
                "--topics",
                *topics,
            ],
            cwd=runtime,
        )
        bridge = children.start(
            "bridge",
            [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                "--ros-args",
                "-p",
                f"config_file:={output / 'bridge.yaml'}",
            ],
            cwd=runtime,
        )
        gazebo = children.start(
            "gazebo",
            [
                "gz",
                "sim",
                "-s",
                "-r",
                "--headless-rendering",
                str(output / "sensor_world.sdf"),
            ],
            cwd=runtime,
        )
        wait_clock(
            output / "live/progress.json", guards=[live, recorder, bridge, gazebo]
        )
        tf = children.start(
            "static_tf",
            [
                "ros2",
                "run",
                "forklift_ros",
                "synthetic_tf",
                "--config",
                str(output / "transforms.yaml"),
            ],
            cwd=runtime,
        )
        children.wait(live, args.wall_timeout, guards=[recorder, bridge, tf, gazebo])
        children.stop(recorder)
        for process in (gazebo, bridge, tf):
            children.stop(process)
        result["original_publishers_stopped_before_replay"] = {
            name: process.returncode
            for name, process in [
                ("gazebo", gazebo),
                ("bridge", bridge),
                ("static_tf", tf),
            ]
        }
        bag = children.start(
            "bag_validator",
            [
                *validator,
                "--phase",
                "bag",
                "--bag",
                str(output / "bag"),
                "--output",
                str(output / "stored_bag"),
            ],
            cwd=runtime,
        )
        children.wait(bag, 120)
        replay = children.start(
            "replay_validator",
            [*validator, "--phase", "replay", "--output", str(output / "replay")],
            cwd=runtime,
        )
        wait_ready(output / "replay/ready.json", replay)
        player = children.start(
            "bag_play",
            [
                "ros2",
                "bag",
                "play",
                str(output / "bag"),
                "--disable-keyboard-controls",
                "--qos-profile-overrides-path",
                str(qos),
                "--delay",
                "2",
            ],
            cwd=runtime,
        )
        children.wait(player, args.wall_timeout, guards=[replay])
        time.sleep(1)
        (output / "replay/playback_done").write_text(
            "Original recorded /clock; no generated playback clock.\n"
        )
        children.wait(replay, 15)
        result["phases"] = {
            name: json.loads((output / directory / "result.json").read_text())
            for name, directory in [
                ("live", "live"),
                ("stored_bag", "stored_bag"),
                ("replay", "replay"),
            ]
        }
        stored = result["phases"]["stored_bag"]["streams"]
        replayed = result["phases"]["replay"]["streams"]
        for name in stored:
            if stored[name] != replayed[name]:
                raise ValueError(
                    f"Fresh replay does not reproduce stored stream counts/time range: {name}"
                )
        result["passed"] = all(phase["passed"] for phase in result["phases"].values())
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        result["traceback"] = traceback.format_exc()
    finally:
        cleanup = children.close()
        if cleanup:
            result["cleanup_errors"] = cleanup
            result["passed"] = False
        evidence = output / "runtime_evidence"
        evidence.mkdir()
        # Preserve regular logs/XML; never expose runtime symlinks to collection.
        for directory in (runtime / "build", runtime / "home", runtime / "ros_logs"):
            if directory.exists():
                for path in directory.rglob("*"):
                    if (
                        path.is_file()
                        and not path.is_symlink()
                        and (
                            path.suffix in {".log", ".xml"} or path.name == "ogre2.log"
                        )
                    ):
                        target = evidence / path.relative_to(runtime)
                        target.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copyfile(path, target)
        result["processes"] = [
            {
                "name": name,
                "pid": process.pid,
                "exit_code": process.poll(),
                "command": command,
            }
            for name, process, command in children.processes
        ]
        result["wall_elapsed_s"] = time.monotonic() - start
        (output / "result.json").write_text(
            json.dumps(result, indent=2, allow_nan=False) + "\n"
        )
    print(
        json.dumps(
            {
                "passed": result["passed"],
                "output": str(output),
                "error": result.get("error"),
            }
        ),
        flush=True,
    )
    return 0 if result["passed"] else 1


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--wall-timeout", type=float, default=300.0)
    args = parser.parse_args()
    if (
        not math.isfinite(args.duration)
        or args.duration < 30
        or not math.isfinite(args.wall_timeout)
        or args.wall_timeout <= 0
    ):
        parser.error("duration must be >=30; wall timeout must be finite and positive")
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
