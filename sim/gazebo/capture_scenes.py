"""Capture a bounded catalogue range with one installed ROS build per batch."""

import argparse
import hashlib
import importlib.util
import json
import math
import os
import re
import shlex
import subprocess
import sys
import time
from copy import deepcopy
from pathlib import Path


def _load_sibling(name: str):
    spec = importlib.util.spec_from_file_location(
        f"capture_{name}", Path(__file__).with_name(f"{name}.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


smoke = _load_sibling("run_sensor_smoke")
world_builder = _load_sibling("build_scene_world")
Children = smoke.Children
wait_ready = smoke.wait_ready
wait_clock = smoke.wait_clock


def parse_scene_range(text: str, catalogue_ids: list[str]) -> list[str]:
    """Expand sNNN-sMMM in ascending order, rejecting missing catalogue IDs."""
    if not isinstance(text, str) or not re.fullmatch(
        r"s\d{3}-s\d{3}", text, flags=re.ASCII
    ):
        raise ValueError("scene range must have format sNNN-sMMM")
    first, last = int(text[1:4]), int(text[6:9])
    if first > last:
        raise ValueError("scene range must be ordered start <= end")
    requested = [f"s{number:03d}" for number in range(first, last + 1)]
    missing = set(requested) - set(catalogue_ids)
    if missing:
        raise ValueError(
            f"scene range contains unknown catalogue IDs: {sorted(missing)}"
        )
    return requested


def build_manifest(
    *,
    catalogue: dict,
    catalogue_sha256: str,
    image_id: str,
    source_sha: str,
    run_id: str,
    requested_scenes: list[str],
    scenes: dict,
) -> dict:
    """Snapshot batch provenance, per-scene results and the number of failures."""
    return deepcopy(
        {
            "catalogue_version": catalogue["catalogue_version"],
            "catalogue_sha256": catalogue_sha256,
            "camera": catalogue["camera"],
            "image_id": image_id,
            "source_snapshot_sha256": source_sha,
            "run_id": run_id,
            "requested_scenes": requested_scenes,
            "scenes": scenes,
            "failed_count": sum(
                result["passed"] is not True for result in scenes.values()
            ),
        }
    )


def manifest_exit_code(manifest: dict) -> int:
    """Success requires every requested scene, no failures and no batch error."""
    passed = (
        not manifest.get("error")
        and manifest["failed_count"] == 0
        and set(manifest["requested_scenes"]) == set(manifest["scenes"])
        and all(result["passed"] is True for result in manifest["scenes"].values())
    )
    return 0 if passed else 1


def write_json(path: Path, value: dict) -> None:
    """Publish each manifest update without exposing partial JSON to readers."""
    temporary = path.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def _remaining(deadline: float) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError("scene exceeded its total wall deadline")
    return remaining


def _failed_result(error: str) -> dict:
    return {
        "passed": False,
        "error": error,
        "files": {},
        "wall_times_s": dict.fromkeys(
            ("build_world", "ready", "first_clock", "captured", "stopped")
        ),
    }


def prepare_runtime(source: Path, runtime: Path, output: Path, env: dict) -> None:
    """Build/test once and source the installed environment for every later child."""
    children = Children(output, env)
    try:
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
        installed = subprocess.run(
            [
                "bash",
                "-c",
                f"source {shlex.quote(str(runtime / 'install/setup.bash'))} && env -0",
            ],
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
        check = children.start(
            "installed_import",
            [
                sys.executable,
                "-c",
                "import forklift_ros.scene_files as m; import forklift_ros.scene_capture as n; "
                "print(m.__file__, n.__file__); "
                "assert '/install/' in m.__file__ and '/install/' in n.__file__",
            ],
            cwd=runtime,
        )
        children.wait(check, 20)
    finally:
        cleanup = children.close()
        if cleanup:
            raise RuntimeError("batch setup cleanup failed: " + "; ".join(cleanup))


def capture_scene(
    args: argparse.Namespace,
    scene_id: str,
    scene_dir: Path,
    runtime: Path,
    env: dict,
    source: Path,
) -> dict:
    """Capture one scene within a shared deadline and always clean its process groups.

    wall_times_s records cumulative monotonic elapsed times from this scene's
    start. Unreached milestones remain null, including failed capture stages.
    """
    start = time.monotonic()
    deadline = start + args.scene_deadline_s
    result = _failed_result("")
    result["error"] = None
    wall_times = result["wall_times_s"]
    scene_dir.mkdir(parents=True, exist_ok=False)
    children = Children(scene_dir, env)
    try:
        world = scene_dir / "world"
        generate = children.start(
            "build_world",
            [
                sys.executable,
                str(source / "sim/gazebo/build_scene_world.py"),
                "--catalogue",
                str(args.catalogue.resolve()),
                "--scene",
                scene_id,
                "--output",
                str(world),
            ],
            cwd=runtime,
        )
        children.wait(generate, _remaining(deadline))
        wall_times["build_world"] = time.monotonic() - start
        capture = children.start(
            "scene_capture",
            [
                "ros2",
                "run",
                "forklift_ros",
                "scene_capture",
                "--output",
                str(scene_dir),
                "--entry",
                str(world / "scene.yaml"),
                "--image-id",
                args.image_id,
                "--source-sha256",
                args.source_sha256,
                "--run-id",
                args.run_id,
                "--warmup-s",
                "2",
                "--deadline-s",
                str(_remaining(deadline)),
            ],
            cwd=runtime,
        )
        wait_ready(scene_dir / "ready.json", capture, timeout_s=_remaining(deadline))
        wall_times["ready"] = time.monotonic() - start
        _remaining(deadline)
        bridge = children.start(
            "bridge",
            [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                "--ros-args",
                "-p",
                f"config_file:={world / 'bridge.yaml'}",
            ],
            cwd=runtime,
        )
        _remaining(deadline)
        gazebo = children.start(
            "gazebo",
            [
                "gz",
                "sim",
                "-s",
                "-r",
                "--headless-rendering",
                str(world / "scene_world.sdf"),
            ],
            cwd=runtime,
        )
        wait_clock(
            scene_dir / "progress.json",
            timeout_s=_remaining(deadline),
            guards=[capture, bridge, gazebo],
        )
        wall_times["first_clock"] = time.monotonic() - start
        _remaining(deadline)
        tf = children.start(
            "static_tf",
            [
                "ros2",
                "run",
                "forklift_ros",
                "synthetic_tf",
                "--config",
                str(world / "transforms.yaml"),
            ],
            cwd=runtime,
        )
        children.wait(capture, _remaining(deadline), guards=[bridge, gazebo, tf])
        _remaining(deadline)
        captured = json.loads((scene_dir / "result.json").read_text())
        result["files"] = captured.get("files", {})
        if captured.get("passed") is not True:
            raise RuntimeError(captured.get("error") or "capture node did not pass")
        wall_times["captured"] = time.monotonic() - start
        result["passed"] = True
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        cleanup = children.close()
        wall_times["stopped"] = time.monotonic() - start
        if cleanup:
            result["passed"] = False
            result["error"] = "; ".join(filter(None, [result["error"], *cleanup]))
    return result


def run(args: argparse.Namespace) -> int:
    """Continue after scene failures, publishing the manifest after each scene."""
    if not math.isfinite(args.scene_deadline_s) or args.scene_deadline_s <= 0:
        raise ValueError("scene deadline must be finite and positive")
    source = Path(__file__).resolve().parents[2]
    catalogue = world_builder.load_catalogue(args.catalogue.resolve())
    requested = parse_scene_range(
        args.scenes, [entry["scene_id"] for entry in catalogue["scenes"]]
    )
    output = args.output.resolve() / "scene_capture"
    output.mkdir(parents=True, exist_ok=False)
    runtime = args.output.resolve() / ".runtime"
    for name in ("home", "tmp", "pycache", "ros_logs"):
        (runtime / name).mkdir(parents=True, exist_ok=True)
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
        GZ_PARTITION=f"forklift_scenes_{os.getpid()}",
        LIBGL_ALWAYS_SOFTWARE="1",
    )
    manifest = build_manifest(
        catalogue=catalogue,
        catalogue_sha256=hashlib.sha256(args.catalogue.read_bytes()).hexdigest(),
        image_id=args.image_id,
        source_sha=args.source_sha256,
        run_id=args.run_id,
        requested_scenes=requested,
        scenes={},
    )
    manifest_path = output / "manifest.json"
    write_json(manifest_path, manifest)
    try:
        prepare_runtime(source, runtime, output, env)
    except Exception as exc:
        manifest["error"] = f"{type(exc).__name__}: {exc}"
        manifest["scenes"] = {
            scene_id: _failed_result(manifest["error"]) for scene_id in requested
        }
        manifest["failed_count"] = len(requested)
        write_json(manifest_path, manifest)
        return 1
    for scene_id in requested:
        try:
            result = capture_scene(
                args, scene_id, output / "scenes" / scene_id, runtime, env, source
            )
        except Exception as exc:
            result = _failed_result(f"{type(exc).__name__}: {exc}")
        manifest["scenes"][scene_id] = result
        manifest["failed_count"] = sum(
            scene["passed"] is not True for scene in manifest["scenes"].values()
        )
        write_json(manifest_path, manifest)
        print(json.dumps({"scene_id": scene_id, **result}, allow_nan=False), flush=True)
    return manifest_exit_code(manifest)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalogue", type=Path, required=True)
    parser.add_argument(
        "--scenes", required=True, help="inclusive catalogue range, e.g. s001-s025"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--source-sha256", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--scene-deadline-s", type=float, default=120.0)
    args = parser.parse_args()
    if not math.isfinite(args.scene_deadline_s) or args.scene_deadline_s <= 0:
        parser.error("scene deadline must be finite and positive")
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
