#!/usr/bin/env python3
"""Run one verified model or Gazebo job inside an allocated Slurm task."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import signal
import subprocess
import sys
import traceback
from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any
from xml.etree import ElementTree

MANIFEST_NAME = ".remote-source-manifest.json"
_SLURM_KEYS = (
    "SLURM_CLUSTER_NAME",
    "SLURM_CPUS_PER_TASK",
    "SLURM_JOB_ACCOUNT",
    "SLURM_JOB_ID",
    "SLURM_JOB_NAME",
    "SLURM_JOB_NODELIST",
    "SLURM_MEM_PER_NODE",
    "SLURM_PARTITION",
)
CommandRunner = Callable[..., subprocess.CompletedProcess[str]]


@dataclass(frozen=True)
class CoreEnvironment:
    python: str
    wheel_path: str
    wheel_sha256: str
    import_path: str
    venv_pip_version: str = ""
    venv_setuptools_version: str = ""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _snapshot_digest(files: Mapping[str, Mapping[str, Any]]) -> str:
    digest = hashlib.sha256()
    for relative, metadata in sorted(files.items()):
        digest.update(f"{metadata['sha256']}  {relative}\n".encode())
    return digest.hexdigest()


def verify_source(source: Path) -> dict[str, Any]:
    source = Path(source)
    if source.is_symlink() or not source.is_dir():
        raise ValueError("source must be a real directory")
    manifest_path = source / MANIFEST_NAME
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ValueError("source manifest is missing or is a symlink")
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema_version") != 1 or not isinstance(
        manifest.get("files"), dict
    ):
        raise ValueError("unsupported source manifest")
    for relative, metadata in manifest["files"].items():
        pure = PurePosixPath(relative)
        if pure.is_absolute() or ".." in pure.parts or pure.as_posix() != relative:
            raise ValueError(f"unsafe manifest path: {relative}")
        path = source / relative
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"manifest file is missing or linked: {relative}")
        data = path.read_bytes()
        if len(data) != metadata.get("size"):
            raise ValueError(f"source size mismatch: {relative}")
        if hashlib.sha256(data).hexdigest() != metadata.get("sha256"):
            raise ValueError(f"source hash mismatch: {relative}")
    digest = _snapshot_digest(manifest["files"])
    if digest != manifest.get("snapshot_sha256"):
        raise ValueError("snapshot hash mismatch")
    return manifest


def validate_scene_selection(
    source: Path,
    catalogue: str | None,
    scene_range: str | None,
    files: Sequence[str] | Mapping[str, Any],
) -> None:
    """Require a normalized, included catalogue and an existing ordered ID range."""
    if not catalogue:
        raise ValueError("scenes requires --catalogue")
    path = PurePosixPath(catalogue)
    if (
        path.is_absolute()
        or path.as_posix() != catalogue
        or any(part.startswith(".") for part in path.parts)
        or "\\" in catalogue
        or catalogue not in files
    ):
        raise ValueError("catalogue must be a normalized relative snapshot file")
    if not scene_range or not re.fullmatch(r"s[0-9]{3}-s[0-9]{3}", scene_range):
        raise ValueError("scene-range must have format sNNN-sMMM")
    first, last = int(scene_range[1:4]), int(scene_range[6:9])
    if first > last:
        raise ValueError("scene-range must be ordered start <= end")
    # Only scenes needs YAML; existing model/gazebo commands stay stdlib-only.
    import yaml

    try:
        entries = yaml.safe_load((source / catalogue).read_text())["scenes"]
        ids = [entry["scene_id"] for entry in entries]
        if not ids or any(not isinstance(value, str) for value in ids):
            raise ValueError("catalogue requires string scene IDs")
        if len(ids) != len(set(ids)):
            raise ValueError("catalogue has duplicate scene IDs")
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read catalogue scene IDs: {error}") from error
    requested = {f"s{number:03d}" for number in range(first, last + 1)}
    if requested - set(ids):
        raise ValueError("scene-range contains unknown catalogue IDs")


def _format_cpuset(cpus: set[int]) -> str:
    if not cpus:
        raise ValueError("Slurm process has no CPU affinity")
    values = sorted(cpus)
    ranges: list[str] = []
    start = previous = values[0]
    for value in values[1:]:
        if value == previous + 1:
            previous = value
            continue
        ranges.append(str(start) if start == previous else f"{start}-{previous}")
        start = previous = value
    ranges.append(str(start) if start == previous else f"{start}-{previous}")
    return ",".join(ranges)


def build_command(
    *,
    mode: str,
    source: Path,
    output: Path,
    python: str | None,
    image: str | None,
    duration: int,
    container_name: str,
    catalogue: str | None = None,
    scene_range: str | None = None,
    source_sha256: str | None = None,
    run_id: str | None = None,
) -> list[str]:
    if duration <= 0:
        raise ValueError("duration must be positive")
    if mode == "model-cpu":
        if not python:
            raise ValueError("model-cpu requires an explicit Python interpreter")
        return [
            python,
            "-m",
            "pytest",
            "tests",
            "-m",
            "not rendering",
            "-q",
            "-p",
            "no:cacheprovider",
            "-W",
            "error",
            "--junitxml",
            str(output / "pytest.xml"),
        ]
    if mode == "model-render":
        if not python:
            raise ValueError("model-render requires an explicit Python interpreter")
        return [
            python,
            "tools/preview_forklift_model.py",
            "--model",
            "sim/models/dls08_provisional/scene.xml",
            "--output",
            str(output / "preview"),
            "--backend",
            "osmesa",
            "--frames",
            "96",
        ]
    if mode not in {"gazebo", "scenes"}:
        raise ValueError(f"unsupported mode: {mode}")
    if not image:
        raise ValueError(f"{mode} requires an explicit image ID")
    cpuset = _format_cpuset(set(os.sched_getaffinity(0)))
    groups = sorted(set(os.getgroups()) - {os.getgid()})
    argv = [
        "docker",
        "run",
        "--rm",
        "--name",
        container_name,
        "--cpuset-cpus",
        cpuset,
        "--memory",
        "4g",
        "--memory-swap",
        "4g",
        "--user",
        f"{os.getuid()}:{os.getgid()}",
    ]
    for group in groups:
        argv.extend(["--group-add", str(group)])
    argv.extend(
        [
            "--network",
            "none",
            "--env",
            "HOME=/output/.runtime/home",
            "--env",
            "XDG_CACHE_HOME=/output/.runtime/xdg-cache",
            "--env",
            "PYTHONPYCACHEPREFIX=/output/.runtime/pycache",
            "--env",
            "GZ_IP=127.0.0.1",
            "--mount",
            f"type=bind,src={source},dst=/workspace,readonly",
            "--mount",
            f"type=bind,src={output},dst=/output",
            "--workdir",
            "/workspace",
            image,
        ]
    )
    if mode == "scenes":
        if not all((catalogue, scene_range, source_sha256, run_id)):
            raise ValueError(
                "scenes requires catalogue, scene-range, source sha and run-id"
            )
        argv.extend(
            [
                "python3",
                "sim/gazebo/capture_scenes.py",
                "--catalogue",
                f"/workspace/{catalogue}",
                "--scenes",
                scene_range,
                "--output",
                "/output",
                "--image-id",
                image,
                "--source-sha256",
                source_sha256,
                "--run-id",
                run_id,
            ]
        )
    else:
        argv.extend(
            [
                "python3",
                "sim/gazebo/run_sensor_smoke.py",
                "--output",
                "/output",
                "--duration",
                str(duration),
            ]
        )
    return argv


def _run_and_record(
    *,
    label: str,
    argv: list[str],
    cwd: Path,
    environment: Mapping[str, str],
    command_runner: CommandRunner,
    output: Path,
    commands: list[dict[str, Any]],
) -> subprocess.CompletedProcess[str]:
    result = command_runner(
        argv,
        cwd=cwd,
        env=dict(environment),
        capture_output=True,
        text=True,
        check=False,
    )
    (output / f"{label}.log").write_text((result.stdout or "") + (result.stderr or ""))
    commands.append({"label": label, "argv": argv, "exit_code": result.returncode})
    return result


def prepare_core_environment(
    *,
    python: str,
    source: Path,
    output: Path,
    runtime: Path,
    environment: Mapping[str, str],
    command_runner: CommandRunner,
    commands: list[dict[str, Any]],
) -> CoreEnvironment:
    """Install a snapshot wheel without writing to the read-only source tree."""
    source = Path(source).resolve()
    output = Path(output).absolute()
    runtime = Path(runtime).absolute()
    build_src = runtime / "core-build"
    build_src.mkdir(parents=True)
    shutil.copyfile(source / "pyproject.toml", build_src / "pyproject.toml")
    shutil.copytree(source / "src", build_src / "src", copy_function=shutil.copyfile)
    # copytree also copies directory modes, including the snapshot's a-w bits.
    for path in [build_src, *build_src.rglob("*")]:
        os.chmod(path, path.stat().st_mode | (0o700 if path.is_dir() else 0o600))

    def run(label: str, argv: list[str]) -> subprocess.CompletedProcess[str]:
        result = _run_and_record(
            label=label,
            argv=argv,
            cwd=runtime,
            environment=environment,
            command_runner=command_runner,
            output=output,
            commands=commands,
        )
        if result.returncode:
            raise RuntimeError(f"{label} failed with exit code {result.returncode}")
        return result

    wheels = output / "wheels"
    wheels.mkdir()
    run(
        "core-wheel",
        [
            python,
            "-m",
            "pip",
            "wheel",
            "--no-deps",
            "--no-build-isolation",
            "--no-index",
            "--wheel-dir",
            str(wheels),
            str(build_src),
        ],
    )
    built = sorted(wheels.glob("*.whl"))
    if len(built) != 1:
        raise RuntimeError(
            f"core-wheel must produce exactly one wheel, got {len(built)}"
        )
    wheel = built[0]
    wheel_sha256 = hashlib.sha256(wheel.read_bytes()).hexdigest()
    venv = runtime / "venv"
    run("core-venv", [python, "-m", "venv", "--system-site-packages", str(venv)])
    venv_python = str(venv / "bin" / "python")
    run(
        "core-install",
        [
            venv_python,
            "-m",
            "pip",
            "install",
            "--no-deps",
            "--no-index",
            "--force-reinstall",
            str(wheel),
        ],
    )
    # Query metadata without importing pip/setuptools, whose distutils hooks
    # conflict in some base environments. Keep one recorded import-check step.
    checked = run(
        "core-import-check",
        [
            venv_python,
            "-c",
            "import forklift_core, json; from importlib.metadata import version; "
            "print(json.dumps({'import_path': forklift_core.__file__, "
            "'venv_pip_version': version('pip'), "
            "'venv_setuptools_version': version('setuptools')}))",
        ],
    )
    metadata = json.loads(checked.stdout)
    import_path = metadata["import_path"]
    if not isinstance(import_path, str) or not Path(
        import_path
    ).resolve().is_relative_to(venv.resolve()):
        raise RuntimeError(f"core import is outside the run venv: {import_path}")
    return CoreEnvironment(
        python=venv_python,
        wheel_path=str(wheel),
        wheel_sha256=wheel_sha256,
        import_path=import_path,
        venv_pip_version=metadata["venv_pip_version"],
        venv_setuptools_version=metadata["venv_setuptools_version"],
    )


class _ContainerSignalCleanup:
    def __init__(self, name: str):
        self.name = name
        self.previous: Any = None

    def __enter__(self) -> _ContainerSignalCleanup:
        self.previous = signal.getsignal(signal.SIGTERM)

        def stop_container(signum: int, _frame: Any) -> None:
            subprocess.run(
                ["docker", "rm", "-f", "--", self.name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            raise SystemExit(128 + signum)

        signal.signal(signal.SIGTERM, stop_container)
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback_value: Any) -> None:
        signal.signal(signal.SIGTERM, self.previous)
        if exc_type is not None:
            subprocess.run(
                ["docker", "rm", "-f", "--", self.name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )


def execute_job(
    *,
    mode: str,
    source: Path,
    output: Path,
    python: str | None,
    image: str | None,
    duration: int,
    catalogue: str | None = None,
    scene_range: str | None = None,
    run_id: str | None = None,
    command_runner: CommandRunner = subprocess.run,
    environment: Mapping[str, str] | None = None,
    core_environment_builder: Callable[..., CoreEnvironment] = prepare_core_environment,
) -> int:
    source = Path(source).resolve()
    output = Path(output).absolute()
    if output == source or source in output.parents or output in source.parents:
        raise ValueError("source and output paths must not collide")
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ if environment is None else environment)
    runtime = output / ".runtime"
    env.update(
        {
            "HOME": str(runtime / "home"),
            "TMPDIR": str(runtime / "tmp"),
            "PYTHONPYCACHEPREFIX": str(runtime / "pycache"),
            "PYTHONDONTWRITEBYTECODE": "1",
            "PYTHONUNBUFFERED": "1",
            "OMP_NUM_THREADS": env.get("SLURM_CPUS_PER_TASK", "2"),
            "OPENBLAS_NUM_THREADS": env.get("SLURM_CPUS_PER_TASK", "2"),
            "MKL_NUM_THREADS": env.get("SLURM_CPUS_PER_TASK", "2"),
        }
    )
    for directory in (runtime / "home", runtime / "tmp", runtime / "pycache"):
        directory.mkdir(parents=True, exist_ok=True)
    env.pop("PYTHONPATH", None)
    commands: list[dict[str, Any]] = []
    job_id = env.get("SLURM_JOB_ID", "unallocated")
    container_name = f"forklift-{job_id}-{os.getpid()}"
    report: dict[str, Any] = {
        "schema_version": 1,
        "mode": mode,
        "host": platform.node(),
        "started_at_utc": _utc_now(),
        "source": {"verified": False},
        "slurm": {key: env.get(key) for key in _SLURM_KEYS},
        "commands": commands,
    }
    code = 1
    try:
        manifest = verify_source(source)
        report["source"] = {
            "verified": True,
            "revision": manifest.get("source_revision"),
            "dirty_status": manifest.get("source_dirty_status"),
            "snapshot_sha256": manifest["snapshot_sha256"],
            "files": manifest["files"],
        }
        if mode == "scenes":
            if not image or not run_id:
                raise ValueError("scenes requires --image and --run-id")
            validate_scene_selection(source, catalogue, scene_range, manifest["files"])
        immutable_image = image
        if mode in {"model-cpu", "model-render"}:
            if not python:
                raise ValueError(f"{mode} requires an explicit Python interpreter")
            core_environment = core_environment_builder(
                python=python,
                source=source,
                output=output,
                runtime=runtime,
                environment=env,
                command_runner=command_runner,
                commands=commands,
            )
            report["core_environment"] = asdict(core_environment)
            python = core_environment.python
        if mode in {"gazebo", "scenes"}:
            inspected = _run_and_record(
                label="docker-image-inspect",
                argv=["docker", "image", "inspect", "--format", "{{.Id}}", str(image)],
                cwd=output,
                environment=env,
                command_runner=command_runner,
                output=output,
                commands=commands,
            )
            if inspected.returncode:
                code = inspected.returncode
                raise RuntimeError("docker image inspection failed")
            immutable_image = inspected.stdout.strip()
            if not immutable_image.startswith("sha256:"):
                raise RuntimeError("docker image inspection returned no immutable ID")
            report["container_image"] = {"requested": image, "id": immutable_image}
        argv = build_command(
            mode=mode,
            source=source,
            output=output,
            python=python,
            image=immutable_image,
            duration=duration,
            container_name=container_name,
            **(
                {
                    "catalogue": catalogue,
                    "scene_range": scene_range,
                    "source_sha256": manifest["snapshot_sha256"],
                    "run_id": run_id,
                }
                if mode == "scenes"
                else {}
            ),
        )
        if mode == "model-render":
            env["MUJOCO_GL"] = "osmesa"
            env["CUDA_VISIBLE_DEVICES"] = ""
        if mode in {"gazebo", "scenes"}:
            with _ContainerSignalCleanup(container_name):
                result = _run_and_record(
                    label=mode,
                    argv=argv,
                    cwd=source,
                    environment=env,
                    command_runner=command_runner,
                    output=output,
                    commands=commands,
                )
        else:
            result = _run_and_record(
                label=mode,
                argv=argv,
                cwd=source,
                environment=env,
                command_runner=command_runner,
                output=output,
                commands=commands,
            )
        code = result.returncode
        if mode == "model-cpu" and code == 0:
            suites = (
                ElementTree.parse(output / "pytest.xml").getroot().findall("testsuite")
            )
            counts = {
                key: sum(int(suite.attrib.get(key, 0)) for suite in suites)
                for key in ("tests", "failures", "errors", "skipped")
            }
            report["pytest"] = counts
            if (
                counts["tests"] < 1
                or counts["failures"]
                or counts["errors"]
                or counts["skipped"]
            ):
                code = 1
                raise RuntimeError(f"pytest result is not clean: {counts}")
    except BaseException as error:
        if isinstance(error, (KeyboardInterrupt, SystemExit)):
            code = 130 if isinstance(error, KeyboardInterrupt) else int(error.code or 1)
        elif code == 0:
            code = 1
        report["error"] = f"{type(error).__name__}: {error}"
        if not isinstance(error, (KeyboardInterrupt, SystemExit)):
            traceback.print_exc()
    finally:
        try:
            shutil.rmtree(runtime)
        except Exception as error:
            report["cleanup_error"] = f"{type(error).__name__}: {error}"
            if code == 0:
                code = 1
        report["exit_code"] = code
        report["ended_at_utc"] = _utc_now()
        (output / "job_result.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
        )
    return code


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-only", action="store_true")
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--mode", choices=("model-cpu", "model-render", "gazebo", "scenes")
    )
    parser.add_argument("--python")
    parser.add_argument("--image")
    parser.add_argument("--duration", type=int, default=30)
    parser.add_argument("--catalogue")
    parser.add_argument("--scene-range")
    parser.add_argument("--run-id")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.verify_only:
        manifest = verify_source(args.source)
        print(json.dumps({"snapshot_sha256": manifest["snapshot_sha256"]}))
        return 0
    if args.output is None or args.mode is None:
        raise ValueError("--output and --mode are required for execution")
    return execute_job(
        mode=args.mode,
        source=args.source,
        output=args.output,
        python=args.python,
        image=args.image,
        duration=args.duration,
        catalogue=args.catalogue,
        scene_range=args.scene_range,
        run_id=args.run_id,
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2) from error
