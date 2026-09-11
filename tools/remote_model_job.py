#!/usr/bin/env python3
"""Run one verified model or Gazebo job inside an allocated Slurm task."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import signal
import subprocess
import sys
import traceback
from collections.abc import Callable, Mapping, Sequence
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
    if mode != "gazebo":
        raise ValueError(f"unsupported mode: {mode}")
    if not image:
        raise ValueError("gazebo requires an explicit image ID")
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
    command_runner: CommandRunner = subprocess.run,
    environment: Mapping[str, str] | None = None,
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
        immutable_image = image
        if mode == "gazebo":
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
        )
        if mode == "model-render":
            env["MUJOCO_GL"] = "osmesa"
            env["CUDA_VISIBLE_DEVICES"] = ""
        if mode == "gazebo":
            with _ContainerSignalCleanup(container_name):
                result = _run_and_record(
                    label="gazebo",
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
    parser.add_argument("--mode", choices=("model-cpu", "model-render", "gazebo"))
    parser.add_argument("--python")
    parser.add_argument("--image")
    parser.add_argument("--duration", type=int, default=30)
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
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2) from error
