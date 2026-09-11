#!/usr/bin/env python3
"""Submit immutable forklift source snapshots to a remote Slurm worker."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shlex
import signal
import stat
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any

MANIFEST_NAME = ".remote-source-manifest.json"

_ROOT_FILES = {
    "pyproject.toml",
    "requirements/model_py311.txt",
    "deploy/slurm/model_check.sbatch",
    "deploy/gazebo/Dockerfile",
}
_SOURCE_RULES = {
    "src": {".py"},
    "examples": {".py"},
    "tools": {".py", ".sh"},
    "tests": {".py"},
    "sim/models": {
        ".dae",
        ".json",
        ".md",
        ".obj",
        ".stl",
        ".urdf",
        ".xml",
        ".yaml",
        ".yml",
    },
    "sim/gazebo": {
        ".json",
        ".md",
        ".py",
        ".rviz",
        ".sdf",
        ".sh",
        ".urdf",
        ".xml",
        ".yaml",
        ".yml",
    },
    "ros2/src": {
        ".cfg",
        ".json",
        ".launch",
        ".md",
        ".msg",
        ".py",
        ".rviz",
        ".sh",
        ".srv",
        ".txt",
        ".xml",
        ".yaml",
        ".yml",
    },
}
_SPECIAL_SOURCE_NAMES = {"CMakeLists.txt", "package.xml"}
_EXCLUDED_DIRS = {
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    "__pycache__",
    "artifacts",
    "build",
    "data",
    "dist",
    "install",
    "log",
    "presentation",
}
_SECRET_PARTS = ("credential", "password", "private_key", "secret", "token")
_SECRET_NAMES = {".env", "id_dsa", "id_ed25519", "id_rsa"}
_SECRET_SUFFIXES = {".key", ".p12", ".pem", ".pfx"}
_RUN_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}\Z")
_HOST_RE = re.compile(r"[A-Za-z0-9_.:@-]+\Z")
_TERMINAL_STATES = {
    "BOOT_FAIL",
    "CANCELLED",
    "COMPLETED",
    "DEADLINE",
    "FAILED",
    "NODE_FAIL",
    "OUT_OF_MEMORY",
    "PREEMPTED",
    "REVOKED",
    "TIMEOUT",
}
_RESULT_RUNTIME_DIRS = {".runtime", "home", "pycache", "tmp"}

CommandRunner = Callable[..., subprocess.CompletedProcess[str]]
_DEFAULT_COMMAND_RUNNER = subprocess.run
_SSH_TIMEOUT_SECONDS = 60.0
_RSYNC_TIMEOUT_SECONDS = 600.0
_SSH_COMMAND = (
    "ssh",
    "-o",
    "BatchMode=yes",
    "-o",
    "ConnectTimeout=10",
    "-o",
    "ServerAliveInterval=10",
    "-o",
    "ServerAliveCountMax=3",
    "-o",
    "ControlMaster=no",
    "-o",
    "ControlPath=none",
)


class RemoteError(RuntimeError):
    """A remote command or validation failed."""


class RemoteRunExistsError(RemoteError):
    """A snapshot, output, or record already exists for a run ID."""


@dataclass(frozen=True)
class SnapshotFile:
    relative_path: str
    sha256: str
    size: int


@dataclass(frozen=True)
class SnapshotPlan:
    source: Path
    files: tuple[SnapshotFile, ...]
    revision: str | None
    dirty_status: str | None

    @property
    def sha256(self) -> str:
        digest = hashlib.sha256()
        for item in self.files:
            digest.update(f"{item.sha256}  {item.relative_path}\n".encode())
        return digest.hexdigest()

    def manifest(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "source_revision": self.revision,
            "source_dirty_status": self.dirty_status,
            "snapshot_sha256": self.sha256,
            "files": {
                item.relative_path: {"sha256": item.sha256, "size": item.size}
                for item in self.files
            },
        }


@dataclass(frozen=True)
class SubmitRequest:
    host: str
    remote_root: str
    source: Path
    mode: str
    python: str | None
    image: str | None
    run_id: str
    duration: int = 30


@dataclass(frozen=True)
class RemoteLayout:
    root: str
    snapshot: str
    output: str
    record: str
    slurm_log: str


@dataclass(frozen=True)
class JobStatus:
    job_id: str
    state: str
    slurm_exit_code: str
    result_exit_code: int | None
    success: bool
    terminal: bool
    slurm_output: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "job_id": self.job_id,
            "state": self.state,
            "slurm_exit_code": self.slurm_exit_code,
            "result_exit_code": self.result_exit_code,
            "success": self.success,
            "terminal": self.terminal,
            "slurm_output": self.slurm_output,
        }


def _is_secret(relative: PurePosixPath) -> bool:
    for part in relative.parts:
        lowered = part.casefold()
        if lowered in _SECRET_NAMES or any(token in lowered for token in _SECRET_PARTS):
            return True
        if Path(part).suffix.casefold() in _SECRET_SUFFIXES:
            return True
    return False


def _is_allowed(relative: PurePosixPath) -> bool:
    text = relative.as_posix()
    if text in _ROOT_FILES:
        return True
    for prefix, suffixes in _SOURCE_RULES.items():
        if text.startswith(prefix + "/"):
            if relative.name in _SPECIAL_SOURCE_NAMES:
                return True
            if prefix == "ros2/src" and "resource" in relative.parts:
                return relative.suffix == ""
            return relative.suffix.casefold() in suffixes
    return False


def _git_metadata(source: Path) -> tuple[str | None, str | None]:
    revision = subprocess.run(
        ["git", "--no-optional-locks", "-C", str(source), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=False,
    )
    if revision.returncode != 0:
        return None, None
    status_result = subprocess.run(
        [
            "git",
            "--no-optional-locks",
            "-C",
            str(source),
            "status",
            "--short",
            "--untracked-files=all",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    status_text = None
    if status_result.returncode == 0:
        status_text = "dirty" if status_result.stdout else "clean"
    return revision.stdout.strip(), status_text


def discover_snapshot(source: Path) -> SnapshotPlan:
    """Discover regular files in the named project allowlist without following links."""
    source = Path(source)
    if source.is_symlink():
        raise ValueError("source root must not be a symlink")
    if not source.is_dir():
        raise ValueError(f"source is not a directory: {source}")
    files: list[SnapshotFile] = []
    for directory, dirnames, filenames in os.walk(source, followlinks=False):
        directory_path = Path(directory)
        dirnames[:] = sorted(
            name
            for name in dirnames
            if name not in _EXCLUDED_DIRS
            and not name.startswith(".")
            and not (directory_path / name).is_symlink()
        )
        for name in sorted(filenames):
            path = directory_path / name
            relative = PurePosixPath(path.relative_to(source).as_posix())
            if any(
                part in _EXCLUDED_DIRS or part.startswith(".")
                for part in relative.parts
            ):
                continue
            if _is_secret(relative) or not _is_allowed(relative):
                continue
            info = path.lstat()
            if not stat.S_ISREG(info.st_mode):
                continue
            data = path.read_bytes()
            files.append(
                SnapshotFile(
                    relative.as_posix(), hashlib.sha256(data).hexdigest(), len(data)
                )
            )
    if not files:
        raise ValueError("source snapshot allowlist selected no files")
    revision, dirty_status = _git_metadata(source)
    return SnapshotPlan(
        source,
        tuple(sorted(files, key=lambda item: item.relative_path)),
        revision,
        dirty_status,
    )


def quote_remote_argv(argv: Sequence[str]) -> str:
    """Encode exact argv for the single remote shell hop used by ssh."""
    return shlex.join([str(value) for value in argv])


def _validate_remote_root(value: str) -> str:
    path = PurePosixPath(value)
    if not path.is_absolute() or ".." in path.parts or path == PurePosixPath("/"):
        raise ValueError("remote root must be an absolute, bounded POSIX path")
    return path.as_posix()


def _validate_request(request: SubmitRequest) -> None:
    if not _HOST_RE.fullmatch(request.host):
        raise ValueError("host contains unsupported characters")
    _validate_remote_root(request.remote_root)
    if not _RUN_ID_RE.fullmatch(request.run_id):
        raise ValueError("run ID must use letters, digits, dot, underscore, or hyphen")
    if request.mode not in {"model-cpu", "model-render", "gazebo"}:
        raise ValueError(f"unsupported mode: {request.mode}")
    if request.duration <= 0:
        raise ValueError("duration must be positive")
    if request.mode == "gazebo":
        if not request.image:
            raise ValueError("gazebo mode requires --image")
    elif not request.python:
        raise ValueError("model modes require --python")


def remote_layout(remote_root: str, run_id: str) -> RemoteLayout:
    root = _validate_remote_root(remote_root).rstrip("/")
    snapshot = f"{root}/snapshots/{run_id}"
    output = f"{root}/artifacts/{run_id}"
    record = f"{root}/jobs/{run_id}.json"
    if (
        snapshot == output
        or snapshot.startswith(output + "/")
        or output.startswith(snapshot + "/")
    ):
        raise ValueError("source snapshot and result paths must not collide")
    return RemoteLayout(root, snapshot, output, record, f"{root}/jobs/{run_id}-%j.log")


def _run(
    command_runner: CommandRunner,
    argv: list[str],
    *,
    input_text: str | None = None,
    timeout: float,
) -> subprocess.CompletedProcess[str]:
    if command_runner is _DEFAULT_COMMAND_RUNNER:
        process = subprocess.Popen(
            argv,
            stdin=subprocess.PIPE if input_text is not None else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        try:
            stdout, stderr = process.communicate(input_text, timeout=timeout)
        except subprocess.TimeoutExpired as error:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.communicate(timeout=1.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    process.communicate(timeout=1.0)
                except subprocess.TimeoutExpired:
                    if process.stdout is not None:
                        process.stdout.close()
                    if process.stderr is not None:
                        process.stderr.close()
                    process.wait(timeout=1.0)
            raise RemoteError(
                f"transport timed out after {timeout:g}s: {argv[0]}"
            ) from error
        return subprocess.CompletedProcess(argv, process.returncode, stdout, stderr)
    return command_runner(
        argv,
        input=input_text,
        capture_output=True,
        text=True,
        check=False,
        timeout=timeout,
    )


def _ssh(
    command_runner: CommandRunner,
    host: str,
    remote_argv: Sequence[str],
    *,
    input_text: str | None = None,
) -> subprocess.CompletedProcess[str]:
    return _run(
        command_runner,
        [*_SSH_COMMAND, "--", host, quote_remote_argv(remote_argv)],
        input_text=input_text,
        timeout=_SSH_TIMEOUT_SECONDS,
    )


def _rsync_command(
    source: str, destination: str, *, extra_args: Sequence[str] = ()
) -> list[str]:
    return [
        "rsync",
        "--archive",
        "--protect-args",
        "--rsh",
        quote_remote_argv(_SSH_COMMAND),
        *extra_args,
        source,
        destination,
    ]


def _stage_snapshot(plan: SnapshotPlan, destination: Path) -> None:
    for item in plan.files:
        source_path = plan.source / item.relative_path
        if source_path.is_symlink():
            raise RemoteError(f"source became a symlink: {item.relative_path}")
        data = source_path.read_bytes()
        if hashlib.sha256(data).hexdigest() != item.sha256:
            raise RemoteError(f"source changed during snapshot: {item.relative_path}")
        target = destination / item.relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    (destination / MANIFEST_NAME).write_text(
        json.dumps(plan.manifest(), ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    )


_RESERVE_SCRIPT = """\
import pathlib, sys
snapshot, output, record = map(pathlib.Path, sys.argv[1:])
if snapshot.exists() or output.exists() or record.exists():
    print('run already exists', file=sys.stderr)
    raise SystemExit(17)
snapshot.parent.mkdir(parents=True, exist_ok=True)
output.parent.mkdir(parents=True, exist_ok=True)
record.parent.mkdir(parents=True, exist_ok=True)
try:
    snapshot.mkdir()
except FileExistsError:
    raise SystemExit(17)
"""

_WRITE_RECORD_SCRIPT = """\
import pathlib, sys
path = pathlib.Path(sys.argv[1])
with path.open('x', encoding='utf-8') as stream:
    stream.write(sys.stdin.read())
"""


def submit(
    request: SubmitRequest,
    *,
    command_runner: CommandRunner = subprocess.run,
) -> dict[str, Any]:
    _validate_request(request)
    plan = discover_snapshot(request.source)
    layout = remote_layout(request.remote_root, request.run_id)
    reserve = _ssh(
        command_runner,
        request.host,
        [
            "python3",
            "-c",
            _RESERVE_SCRIPT,
            layout.snapshot,
            layout.output,
            layout.record,
        ],
    )
    if reserve.returncode == 17:
        raise RemoteRunExistsError(f"remote run already exists: {request.run_id}")
    if reserve.returncode:
        raise RemoteError(reserve.stderr.strip() or "failed to reserve remote run")

    with tempfile.TemporaryDirectory(prefix="forklift-snapshot-") as staging_text:
        staging = Path(staging_text)
        _stage_snapshot(plan, staging)
        transfer = _run(
            command_runner,
            _rsync_command(
                f"{staging}/",
                f"{request.host}:{layout.snapshot}/",
                extra_args=("--chmod=Du=rwx,Dgo=rx,Fu=rw,Fgo=r",),
            ),
            timeout=_RSYNC_TIMEOUT_SECONDS,
        )
        if transfer.returncode:
            raise RemoteError(transfer.stderr.strip() or "snapshot transfer failed")

    verify = _ssh(
        command_runner,
        request.host,
        [
            "python3",
            f"{layout.snapshot}/tools/remote_model_job.py",
            "--verify-only",
            "--source",
            layout.snapshot,
        ],
    )
    if verify.returncode:
        raise RemoteError(
            verify.stderr.strip() or "remote manifest verification failed"
        )
    freeze = _ssh(command_runner, request.host, ["chmod", "-R", "a-w", layout.snapshot])
    if freeze.returncode:
        raise RemoteError(freeze.stderr.strip() or "failed to freeze snapshot")

    sbatch_argv = [
        "sbatch",
        "--parsable",
        f"--output={layout.slurm_log}",
        f"{layout.snapshot}/deploy/slurm/model_check.sbatch",
        request.mode,
        layout.snapshot,
        layout.output,
        request.python or "-",
        request.image or "-",
        str(request.duration),
    ]
    submitted = _ssh(command_runner, request.host, sbatch_argv)
    if submitted.returncode:
        raise RemoteError(submitted.stderr.strip() or "sbatch failed")
    job_id = submitted.stdout.strip().split(";", 1)[0]
    if not job_id.isdigit():
        raise RemoteError(f"unexpected sbatch job ID: {submitted.stdout!r}")
    record = {
        "schema_version": 1,
        "run_id": request.run_id,
        "job_id": job_id,
        "mode": request.mode,
        "snapshot": layout.snapshot,
        "output": layout.output,
        "snapshot_sha256": plan.sha256,
        "submitted_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    written = _ssh(
        command_runner,
        request.host,
        ["python3", "-c", _WRITE_RECORD_SCRIPT, layout.record],
        input_text=json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n",
    )
    if written.returncode:
        raise RemoteError(written.stderr.strip() or "failed to store job record")
    return record


def dry_run_plan(request: SubmitRequest, *, wait: bool = False) -> dict[str, Any]:
    _validate_request(request)
    plan = discover_snapshot(request.source)
    layout = remote_layout(request.remote_root, request.run_id)
    return {
        "dry_run": True,
        "host": request.host,
        "run_id": request.run_id,
        "mode": request.mode,
        "duration": request.duration,
        "python": request.python,
        "image": request.image,
        "wait": wait,
        "source": plan.manifest(),
        "remote": {
            "root": layout.root,
            "snapshot": layout.snapshot,
            "output": layout.output,
            "record": layout.record,
        },
    }


_READ_JSON_SCRIPT = """\
import json, pathlib, sys
path = pathlib.Path(sys.argv[1])
if not path.is_file():
    raise SystemExit(3)
print(json.dumps(json.loads(path.read_text(encoding='utf-8'))))
"""

_HASH_TREE_SCRIPT = """\
import hashlib, json, os, pathlib, sys
root = pathlib.Path(sys.argv[1])
if root.is_symlink() or not root.is_dir():
    raise SystemExit(4)
hashes = {}
for directory, dirnames, filenames in os.walk(root, followlinks=False):
    base = pathlib.Path(directory)
    if base == root:
        dirnames[:] = [name for name in dirnames if name not in {'.runtime', 'home', 'pycache', 'tmp'}]
    if any((base / name).is_symlink() for name in dirnames):
        raise SystemExit(4)
    for name in filenames:
        path = base / name
        if path.is_symlink() or not path.is_file():
            raise SystemExit(4)
        relative = path.relative_to(root).as_posix()
        hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
print(json.dumps(dict(sorted(hashes.items()))))
"""


def _read_remote_json(
    command_runner: CommandRunner, host: str, path: str
) -> dict[str, Any] | None:
    result = _ssh(command_runner, host, ["python3", "-c", _READ_JSON_SCRIPT, path])
    if result.returncode == 3:
        return None
    if result.returncode:
        raise RemoteError(result.stderr.strip() or f"failed to read {path}")
    return json.loads(result.stdout)


def evaluate_status(
    *, job_id: str, slurm_output: str, job_result: dict[str, Any] | None
) -> JobStatus:
    state_match = re.search(r"(?:^|\s)JobState=([^\s]+)", slurm_output)
    exit_match = re.search(r"(?:^|\s)ExitCode=([^\s]+)", slurm_output)
    state = state_match.group(1).split("+", 1)[0] if state_match else "UNKNOWN"
    slurm_exit = exit_match.group(1) if exit_match else "UNKNOWN"
    result_exit = None if job_result is None else job_result.get("exit_code")
    terminal = state in _TERMINAL_STATES
    success = state == "COMPLETED" and slurm_exit == "0:0" and result_exit == 0
    return JobStatus(
        job_id, state, slurm_exit, result_exit, success, terminal, slurm_output
    )


def hash_regular_tree(root: Path) -> dict[str, str]:
    """Hash a result tree and reject links rather than following them."""
    root = Path(root)
    hashes: dict[str, str] = {}
    for directory, dirnames, filenames in os.walk(root, followlinks=False):
        directory_path = Path(directory)
        if directory_path == root:
            dirnames[:] = [
                name for name in dirnames if name not in _RESULT_RUNTIME_DIRS
            ]
        for name in dirnames:
            if (directory_path / name).is_symlink():
                raise RemoteError(f"result tree contains symlink: {name}")
        for name in filenames:
            path = directory_path / name
            if path.is_symlink():
                raise RemoteError(f"result tree contains symlink: {name}")
            if not path.is_file():
                raise RemoteError(f"result tree contains non-regular file: {name}")
            relative = path.relative_to(root).as_posix()
            hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(sorted(hashes.items()))


def get_status(
    *,
    host: str,
    remote_root: str,
    run_id: str,
    command_runner: CommandRunner = subprocess.run,
) -> JobStatus:
    layout = remote_layout(remote_root, run_id)
    record = _read_remote_json(command_runner, host, layout.record)
    if record is None:
        raise RemoteError(f"no remote job record for run: {run_id}")
    job_id = str(record["job_id"])
    control = _ssh(command_runner, host, ["scontrol", "show", "job", job_id, "-o"])
    if control.returncode:
        raise RemoteError(control.stderr.strip() or "scontrol failed")
    result = _read_remote_json(command_runner, host, f"{layout.output}/job_result.json")
    return evaluate_status(
        job_id=job_id, slurm_output=control.stdout, job_result=result
    )


def wait_for_completion(
    *,
    host: str,
    remote_root: str,
    run_id: str,
    command_runner: CommandRunner = subprocess.run,
    poll_interval: float = 2.0,
) -> JobStatus:
    while True:
        status = get_status(
            host=host,
            remote_root=remote_root,
            run_id=run_id,
            command_runner=command_runner,
        )
        if status.terminal:
            return status
        time.sleep(poll_interval)


def collect(
    *,
    host: str,
    remote_root: str,
    run_id: str,
    destination: Path,
    command_runner: CommandRunner = subprocess.run,
) -> dict[str, Any]:
    status = get_status(
        host=host,
        remote_root=remote_root,
        run_id=run_id,
        command_runner=command_runner,
    )
    if not status.success:
        raise RemoteError(
            f"run is not successful: state={status.state}, "
            f"Slurm={status.slurm_exit_code}, result={status.result_exit_code}"
        )
    destination = Path(destination)
    if destination.exists():
        raise FileExistsError(f"collection destination already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    layout = remote_layout(remote_root, run_id)
    remote_hashes_result = _ssh(
        command_runner,
        host,
        ["python3", "-c", _HASH_TREE_SCRIPT, layout.output],
    )
    if remote_hashes_result.returncode:
        raise RemoteError(
            remote_hashes_result.stderr.strip() or "remote result hashing failed"
        )
    remote_hashes = json.loads(remote_hashes_result.stdout)
    transferred = _run(
        command_runner,
        _rsync_command(
            f"{host}:{layout.output}/",
            f"{destination}/",
            extra_args=(
                "--exclude=/.runtime/",
                "--exclude=/home/",
                "--exclude=/pycache/",
                "--exclude=/tmp/",
            ),
        ),
        timeout=_RSYNC_TIMEOUT_SECONDS,
    )
    if transferred.returncode:
        raise RemoteError(transferred.stderr.strip() or "result collection failed")
    result_path = destination / "job_result.json"
    if not result_path.is_file():
        raise RemoteError("collected output has no job_result.json")
    local_hashes = hash_regular_tree(destination)
    if local_hashes != remote_hashes:
        raise RemoteError("collected result hashes do not match the remote output")
    result = json.loads(result_path.read_text())
    if result.get("exit_code") != 0:
        raise RemoteError("collected job_result.json is not successful")
    (destination / "slurm_status.txt").write_text(status.slurm_output)
    (destination / "collection.json").write_text(
        json.dumps(
            {
                "collected_at_utc": datetime.now(timezone.utc).isoformat(),
                "job_status": status.as_dict(),
                "remote_file_sha256": remote_hashes,
            },
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    )
    return result


def _utc_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_remote_%f")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action", required=True)
    submit_parser = subparsers.add_parser("submit")
    submit_parser.add_argument("--host", required=True)
    submit_parser.add_argument("--remote-root", required=True)
    submit_parser.add_argument("--source", required=True, type=Path)
    submit_parser.add_argument(
        "--mode", required=True, choices=("model-cpu", "model-render", "gazebo")
    )
    submit_parser.add_argument("--python")
    submit_parser.add_argument("--image")
    submit_parser.add_argument("--run-id", default=None)
    submit_parser.add_argument("--duration", type=int, default=30)
    submit_parser.add_argument("--dry-run", action="store_true")
    submit_parser.add_argument("--wait", action="store_true")
    submit_parser.add_argument("--output", type=Path)

    for action in ("status", "collect"):
        child = subparsers.add_parser(action)
        child.add_argument("--host", required=True)
        child.add_argument("--remote-root", required=True)
        child.add_argument("--run-id", required=True)
        if action == "collect":
            child.add_argument("--output", required=True, type=Path)
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    command_runner: CommandRunner = subprocess.run,
) -> int:
    args = _build_parser().parse_args(argv)
    if args.action == "submit":
        run_id = args.run_id or _utc_run_id()
        request = SubmitRequest(
            host=args.host,
            remote_root=args.remote_root,
            source=args.source,
            mode=args.mode,
            python=args.python,
            image=args.image,
            run_id=run_id,
            duration=args.duration,
        )
        if args.dry_run:
            print(
                json.dumps(
                    dry_run_plan(request, wait=args.wait), ensure_ascii=False, indent=2
                )
            )
            return 0
        record = submit(request, command_runner=command_runner)
        payload: dict[str, Any] = {"submission": record}
        if args.wait:
            status = wait_for_completion(
                host=args.host,
                remote_root=args.remote_root,
                run_id=run_id,
                command_runner=command_runner,
            )
            payload["status"] = status.as_dict()
            if not status.success:
                print(json.dumps(payload, ensure_ascii=False, indent=2))
                return 1
            destination = args.output or args.source / "artifacts" / run_id
            payload["result"] = collect(
                host=args.host,
                remote_root=args.remote_root,
                run_id=run_id,
                destination=destination,
                command_runner=command_runner,
            )
            payload["collected_to"] = str(destination)
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0
    if args.action == "status":
        status = get_status(
            host=args.host,
            remote_root=args.remote_root,
            run_id=args.run_id,
            command_runner=command_runner,
        )
        print(json.dumps(status.as_dict(), indent=2))
        return 0 if status.success else 1
    result = collect(
        host=args.host,
        remote_root=args.remote_root,
        run_id=args.run_id,
        destination=args.output,
        command_runner=command_runner,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RemoteError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2) from error
