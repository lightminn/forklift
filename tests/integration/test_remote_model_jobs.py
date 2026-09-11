from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest

from tools import remote_model_job, submit_model_check


def _write_source(source: Path) -> None:
    files = {
        "pyproject.toml": "[project]\nname='forklift-core'\n",
        "src/forklift_core/__init__.py": "VALUE = 1\n",
        "examples/sensor_geometry.py": "print('example')\n",
        "tools/remote_model_job.py": "# runner\n",
        "sim/gazebo/run_sensor_smoke.py": "# new untracked source\n",
        "ros2/src/forklift_ros/setup.cfg": "[develop]\nscript_dir=$base/lib/forklift_ros\n",
        "ros2/src/forklift_ros/resource/forklift_ros": "",
        "tests/test_core.py": "def test_ok(): pass\n",
    }
    for relative, contents in files.items():
        path = source / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)


def _write_manifest(source: Path) -> None:
    plan = submit_model_check.discover_snapshot(source)
    (source / submit_model_check.MANIFEST_NAME).write_text(
        json.dumps(plan.manifest(), sort_keys=True) + "\n"
    )


def test_snapshot_includes_named_untracked_source_but_excludes_links_and_secrets(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source"
    _write_source(source)
    (source / "artifacts/run/result.json").parent.mkdir(parents=True)
    (source / "artifacts/run/result.json").write_text("{}")
    (source / "tools/access_token.py").write_text("TOKEN = 'no'\n")
    (source / "tools/server.pem").write_text("private\n")
    (source / ".env").write_text("PASSWORD=no\n")
    (source / "tools/core_link.py").symlink_to(source / "src/forklift_core/__init__.py")

    plan = submit_model_check.discover_snapshot(source)
    paths = {item.relative_path for item in plan.files}

    assert "src/forklift_core/__init__.py" in paths
    assert "examples/sensor_geometry.py" in paths
    assert "sim/gazebo/run_sensor_smoke.py" in paths
    assert "ros2/src/forklift_ros/setup.cfg" in paths
    assert "ros2/src/forklift_ros/resource/forklift_ros" in paths
    assert "tools/access_token.py" not in paths
    assert "tools/server.pem" not in paths
    assert "tools/core_link.py" not in paths
    assert not any(path.startswith("artifacts/") for path in paths)


def test_snapshot_rejects_a_symlink_as_source_root(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    link = tmp_path / "source-link"
    link.symlink_to(source, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        submit_model_check.discover_snapshot(link)


def test_dry_run_prints_plan_without_filesystem_or_external_side_effects(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    source = tmp_path / "source"
    _write_source(source)
    before = sorted(path.relative_to(tmp_path) for path in tmp_path.rglob("*"))

    def forbidden_run(
        *args: object, **kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        raise AssertionError("dry-run invoked an external command")

    code = submit_model_check.main(
        [
            "submit",
            "--host",
            "example-host",
            "--remote-root",
            "/srv/수업 자료/robot $(false)",
            "--source",
            str(source),
            "--mode",
            "model-cpu",
            "--python",
            "/opt/env/bin/python",
            "--run-id",
            "dry-run-01",
            "--dry-run",
        ],
        command_runner=forbidden_run,
    )

    after = sorted(path.relative_to(tmp_path) for path in tmp_path.rglob("*"))
    payload = json.loads(capsys.readouterr().out)
    assert code == 0
    assert before == after
    assert payload["dry_run"] is True
    assert payload["remote"]["snapshot"] == (
        "/srv/수업 자료/robot $(false)/snapshots/dry-run-01"
    )


def test_snapshot_git_metadata_disables_optional_index_writes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "source"
    _write_source(source)
    calls: list[list[str]] = []

    def git_read(argv: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(argv)
        output = "abc123\n" if "rev-parse" in argv else ""
        return subprocess.CompletedProcess(argv, 0, output, "")

    monkeypatch.setattr(submit_model_check.subprocess, "run", git_read)

    submit_model_check.discover_snapshot(source)

    assert len(calls) == 2
    assert all(argv[:2] == ["git", "--no-optional-locks"] for argv in calls)


def test_remote_shell_arguments_round_trip_metacharacters() -> None:
    argv = [
        "python3",
        "-c",
        "print('ok')",
        "/srv/한글 자료/a b;$(touch nope)/[x]",
        "quote'and\"double",
    ]

    command = submit_model_check.quote_remote_argv(argv)

    assert shlex.split(command) == argv


def test_ssh_uses_noninteractive_bounded_connection_options() -> None:
    calls: list[tuple[list[str], dict[str, object]]] = []

    def capture(argv: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append((argv, kwargs))
        return subprocess.CompletedProcess(argv, 0, "", "")

    submit_model_check._ssh(capture, "example-host", ["true"])

    argv, kwargs = calls[0]
    joined = " ".join(argv)
    assert "BatchMode=yes" in joined
    assert "ConnectTimeout=10" in joined
    assert "ServerAliveInterval=10" in joined
    assert "ServerAliveCountMax=3" in joined
    assert "ControlMaster=no" in joined
    assert "ControlPath=none" in joined
    assert kwargs["timeout"] == 60


def test_transport_timeout_kills_orphan_that_keeps_capture_pipe_open(
    tmp_path: Path,
) -> None:
    pid_file = tmp_path / "child.pid"
    child_code = (
        "import os,pathlib,signal,sys,time;"
        "signal.signal(signal.SIGTERM, signal.SIG_IGN);"
        "pathlib.Path(sys.argv[1]).write_text(str(os.getpid()));"
        "time.sleep(60)"
    )
    parent_code = (
        "import pathlib,subprocess,sys,time;"
        "p=subprocess.Popen([sys.executable,'-I','-S','-c',sys.argv[1],sys.argv[2]]);"
        "f=pathlib.Path(sys.argv[2]);"
        "[(time.sleep(.01)) for _ in range(100) if not f.exists()]"
    )
    started = time.monotonic()

    # The fixture uses only stdlib; ignore inherited startup hooks/cache paths
    # so the timeout measures pipe/process cleanup rather than Python startup.
    with pytest.raises(submit_model_check.RemoteError, match="timed out"):
        submit_model_check._run(
            subprocess.run,
            [sys.executable, "-I", "-S", "-c", parent_code, child_code, str(pid_file)],
            timeout=0.1,
        )

    elapsed = time.monotonic() - started
    child_pid = int(pid_file.read_text())
    assert elapsed < 3
    # Orphan reaping is asynchronous; zombies still respond to kill(pid, 0).
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        try:
            os.kill(child_pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.01)
    else:
        pytest.fail(f"orphan process {child_pid} was not reaped within 2s")


def test_rsync_transport_reuses_bounded_ssh_options() -> None:
    argv = submit_model_check._rsync_command("source/", "host:/remote/")

    assert argv[0] == "rsync"
    ssh_command = argv[argv.index("--rsh") + 1]
    assert "BatchMode=yes" in ssh_command
    assert "ConnectTimeout=10" in ssh_command
    assert "ServerAliveInterval=10" in ssh_command
    assert "ServerAliveCountMax=3" in ssh_command
    assert "ControlMaster=no" in ssh_command
    assert "ControlPath=none" in ssh_command


def test_submit_rejects_duplicate_run_before_transfer(tmp_path: Path) -> None:
    source = tmp_path / "source"
    _write_source(source)
    calls: list[list[str]] = []

    def duplicate_run(
        argv: list[str], **kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        calls.append(argv)
        return subprocess.CompletedProcess(argv, 17, "", "run already exists\n")

    with pytest.raises(submit_model_check.RemoteRunExistsError):
        submit_model_check.submit(
            submit_model_check.SubmitRequest(
                host="example-host",
                remote_root="/srv/course/team",
                source=source,
                mode="model-cpu",
                python="/opt/env/bin/python",
                image=None,
                run_id="same-run",
                duration=30,
            ),
            command_runner=duplicate_run,
        )

    assert len(calls) == 1
    assert calls[0][0] == "ssh"
    assert shlex.split(calls[0][-1])[0] == "python3"


def test_failed_child_command_is_recorded_and_propagated(tmp_path: Path) -> None:
    source = tmp_path / "source"
    _write_source(source)
    _write_manifest(source)
    output = tmp_path / "output"

    def fail(argv: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(argv, 9, "", "model failed")

    code = remote_model_job.execute_job(
        core_environment_builder=lambda **kw: remote_model_job.CoreEnvironment(
            python=sys.executable, wheel_path="", wheel_sha256="", import_path=""
        ),
        mode="model-cpu",
        source=source,
        output=output,
        python=sys.executable,
        image=None,
        duration=30,
        command_runner=fail,
        environment={"SLURM_JOB_ID": "42", "SLURM_CPUS_PER_TASK": "2"},
    )

    result = json.loads((output / "job_result.json").read_text())
    assert code == 9
    assert result["exit_code"] == 9
    assert result["commands"][0]["exit_code"] == 9
    assert result["source"]["files"]


def test_manifest_failure_still_writes_a_failed_job_result(tmp_path: Path) -> None:
    source = tmp_path / "source"
    _write_source(source)
    _write_manifest(source)
    (source / "src/forklift_core/__init__.py").write_text("CHANGED = True\n")
    output = tmp_path / "output"

    code = remote_model_job.execute_job(
        core_environment_builder=lambda **kw: remote_model_job.CoreEnvironment(
            python=sys.executable, wheel_path="", wheel_sha256="", import_path=""
        ),
        mode="model-cpu",
        source=source,
        output=output,
        python=sys.executable,
        image=None,
        duration=30,
        environment={"SLURM_JOB_ID": "43"},
    )

    result = json.loads((output / "job_result.json").read_text())
    assert code != 0
    assert result["exit_code"] != 0
    assert "source" in result["error"] and "mismatch" in result["error"]


def test_model_cpu_rejects_a_zero_exit_pytest_run_with_skips(tmp_path: Path) -> None:
    source = tmp_path / "source"
    _write_source(source)
    _write_manifest(source)
    output = tmp_path / "output"

    def skipped(argv: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        pytest_xml = output / "pytest.xml"
        pytest_xml.write_text(
            '<testsuites><testsuite tests="2" failures="0" errors="0" '
            'skipped="1"/></testsuites>\n'
        )
        return subprocess.CompletedProcess(argv, 0, "1 passed, 1 skipped\n", "")

    code = remote_model_job.execute_job(
        core_environment_builder=lambda **kw: remote_model_job.CoreEnvironment(
            python=sys.executable, wheel_path="", wheel_sha256="", import_path=""
        ),
        mode="model-cpu",
        source=source,
        output=output,
        python=sys.executable,
        image=None,
        duration=30,
        command_runner=skipped,
        environment={"SLURM_JOB_ID": "45", "SLURM_CPUS_PER_TASK": "2"},
    )

    result = json.loads((output / "job_result.json").read_text())
    assert code != 0
    assert result["pytest"] == {
        "tests": 2,
        "failures": 0,
        "errors": 0,
        "skipped": 1,
    }


@pytest.mark.parametrize("pytest_xml", [None, "<not-xml"])
def test_model_cpu_rejects_missing_or_malformed_junit(
    tmp_path: Path, pytest_xml: str | None
) -> None:
    source = tmp_path / "source"
    _write_source(source)
    _write_manifest(source)
    output = tmp_path / "output"

    def zero_exit(
        argv: list[str], **kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        if pytest_xml is not None:
            (output / "pytest.xml").write_text(pytest_xml)
        return subprocess.CompletedProcess(argv, 0, "pytest exited zero\n", "")

    code = remote_model_job.execute_job(
        core_environment_builder=lambda **kw: remote_model_job.CoreEnvironment(
            python=sys.executable, wheel_path="", wheel_sha256="", import_path=""
        ),
        mode="model-cpu",
        source=source,
        output=output,
        python=sys.executable,
        image=None,
        duration=30,
        command_runner=zero_exit,
        environment={"SLURM_JOB_ID": "47", "SLURM_CPUS_PER_TASK": "2"},
    )

    result = json.loads((output / "job_result.json").read_text())
    assert code != 0
    assert result["exit_code"] != 0
    assert "error" in result


@pytest.mark.parametrize(
    ("state", "exit_code", "result_code", "expected"),
    [
        ("PENDING", "0:0", None, False),
        ("RUNNING", "0:0", None, False),
        ("FAILED", "1:0", 1, False),
        ("CANCELLED", "0:0", None, False),
        ("COMPLETED", "1:0", 0, False),
        ("COMPLETED", "0:0", None, False),
        ("COMPLETED", "0:0", 0, True),
    ],
)
def test_slurm_success_requires_completed_zero_and_successful_result(
    state: str, exit_code: str, result_code: int | None, expected: bool
) -> None:
    status = submit_model_check.evaluate_status(
        job_id="123",
        slurm_output=f"JobId=123 JobState={state} ExitCode={exit_code}",
        job_result=None if result_code is None else {"exit_code": result_code},
    )

    assert status.success is expected


def test_status_preserves_original_slurm_evidence() -> None:
    raw = "JobId=123 JobState=COMPLETED ExitCode=0:0 RunTime=00:00:02\n"

    status = submit_model_check.evaluate_status(
        job_id="123", slurm_output=raw, job_result={"exit_code": 0}
    )

    assert status.as_dict()["slurm_output"] == raw


def test_artifact_hashing_rejects_symlinks(tmp_path: Path) -> None:
    (tmp_path / "job_result.json").write_text("{}\n")
    (tmp_path / "linked.log").symlink_to(tmp_path / "job_result.json")

    with pytest.raises(submit_model_check.RemoteError, match="symlink"):
        submit_model_check.hash_regular_tree(tmp_path)


def test_artifact_hashing_ignores_only_runner_runtime_directories(
    tmp_path: Path,
) -> None:
    (tmp_path / "job_result.json").write_text("{}\n")
    runtime = tmp_path / "tmp"
    runtime.mkdir()
    (runtime / "pytest-current").symlink_to(tmp_path / "job_result.json")

    hashes = submit_model_check.hash_regular_tree(tmp_path)

    assert set(hashes) == {"job_result.json"}


def test_runner_removes_its_runtime_tree_before_collection(tmp_path: Path) -> None:
    source = tmp_path / "source"
    _write_source(source)
    _write_manifest(source)
    output = tmp_path / "output"

    def pytest_with_temp_link(
        argv: list[str], **kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        environment = kwargs["env"]
        temp = Path(environment["TMPDIR"])
        target = temp / "target"
        target.write_text("temporary\n")
        (temp / "current").symlink_to(target)
        (output / "pytest.xml").write_text(
            '<testsuites><testsuite tests="1" failures="0" errors="0" '
            'skipped="0"/></testsuites>\n'
        )
        return subprocess.CompletedProcess(argv, 0, "1 passed\n", "")

    code = remote_model_job.execute_job(
        core_environment_builder=lambda **kw: remote_model_job.CoreEnvironment(
            python=sys.executable, wheel_path="", wheel_sha256="", import_path=""
        ),
        mode="model-cpu",
        source=source,
        output=output,
        python=sys.executable,
        image=None,
        duration=30,
        command_runner=pytest_with_temp_link,
        environment={"SLURM_JOB_ID": "46", "SLURM_CPUS_PER_TASK": "2"},
    )

    assert code == 0
    result = json.loads((output / "job_result.json").read_text())
    assert "core_environment" in result
    assert not (output / ".runtime").exists()
    assert not any(path.is_symlink() for path in output.rglob("*"))


def test_cleanup_failure_is_recorded_without_suppressing_job_result(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "source"
    _write_source(source)
    _write_manifest(source)
    output = tmp_path / "output"

    def child_success(
        argv: list[str], **kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        (output / "pytest.xml").write_text(
            '<testsuites><testsuite tests="1" failures="0" errors="0" '
            'skipped="0"/></testsuites>\n'
        )
        return subprocess.CompletedProcess(argv, 0, "1 passed\n", "")

    def cleanup_failure(path: Path) -> None:
        raise PermissionError(f"cannot remove {path}")

    monkeypatch.setattr(remote_model_job.shutil, "rmtree", cleanup_failure)

    code = remote_model_job.execute_job(
        core_environment_builder=lambda **kw: remote_model_job.CoreEnvironment(
            python=sys.executable, wheel_path="", wheel_sha256="", import_path=""
        ),
        mode="model-cpu",
        source=source,
        output=output,
        python=sys.executable,
        image=None,
        duration=30,
        command_runner=child_success,
        environment={"SLURM_JOB_ID": "48", "SLURM_CPUS_PER_TASK": "2"},
    )

    result = json.loads((output / "job_result.json").read_text())
    assert code == 1
    assert result["exit_code"] == 1
    assert "PermissionError" in result["cleanup_error"]


def test_collect_preserves_slurm_evidence_and_verifies_remote_hashes(
    tmp_path: Path,
) -> None:
    destination = tmp_path / "collected"
    result_bytes = b'{"exit_code": 0, "source": {"snapshot_sha256": "abc"}}\n'
    digest = __import__("hashlib").sha256(result_bytes).hexdigest()

    def remote(argv: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        if argv[0] == "rsync":
            destination.mkdir()
            (destination / "job_result.json").write_bytes(result_bytes)
            return subprocess.CompletedProcess(argv, 0, "", "")
        command = shlex.split(argv[-1])
        if command[:3] == ["scontrol", "show", "job"]:
            return subprocess.CompletedProcess(
                argv, 0, "JobId=88 JobState=COMPLETED ExitCode=0:0\n", ""
            )
        if "hashlib" in command[2]:
            return subprocess.CompletedProcess(
                argv, 0, json.dumps({"job_result.json": digest}) + "\n", ""
            )
        path = command[-1]
        if path.endswith("/jobs/run-88.json"):
            payload = {"job_id": "88", "snapshot_sha256": "abc"}
        else:
            payload = {"exit_code": 0, "source": {"snapshot_sha256": "abc"}}
        return subprocess.CompletedProcess(argv, 0, json.dumps(payload) + "\n", "")

    result = submit_model_check.collect(
        host="example-host",
        remote_root="/srv/course/team",
        run_id="run-88",
        destination=destination,
        command_runner=remote,
    )

    assert result["exit_code"] == 0
    assert "JobState=COMPLETED" in (destination / "slurm_status.txt").read_text()
    collection = json.loads((destination / "collection.json").read_text())
    assert collection["remote_file_sha256"] == {"job_result.json": digest}


def test_gazebo_docker_argv_is_bounded_to_slurm_resources(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "source with space"
    output = tmp_path / "output"
    source.mkdir()
    output.mkdir()
    monkeypatch.setattr(os, "sched_getaffinity", lambda _pid: {2, 3, 7})
    monkeypatch.setattr(os, "getuid", lambda: 1001)
    monkeypatch.setattr(os, "getgid", lambda: 1002)
    monkeypatch.setattr(os, "getgroups", lambda: [1002, 1200])

    argv = remote_model_job.build_command(
        mode="gazebo",
        source=source,
        output=output,
        python=None,
        image="forklift/gazebo:explicit",
        duration=41,
        container_name="forklift-job-77",
    )

    assert argv[:3] == ["docker", "run", "--rm"]
    assert argv[argv.index("--cpuset-cpus") + 1] == "2-3,7"
    assert argv[argv.index("--memory") + 1] == "4g"
    assert argv[argv.index("--memory-swap") + 1] == "4g"
    assert argv[argv.index("--user") + 1] == "1001:1002"
    assert "1200" in [
        argv[i + 1] for i, value in enumerate(argv) if value == "--group-add"
    ]
    mounts = [argv[i + 1] for i, value in enumerate(argv) if value == "--mount"]
    assert f"type=bind,src={source},dst=/workspace,readonly" in mounts
    assert f"type=bind,src={output},dst=/output" in mounts
    assert argv[-7:] == [
        "forklift/gazebo:explicit",
        "python3",
        "sim/gazebo/run_sensor_smoke.py",
        "--output",
        "/output",
        "--duration",
        "41",
    ]
    assert "--gpus" not in argv


def test_gazebo_resolves_and_records_immutable_image_before_launch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "source"
    _write_source(source)
    _write_manifest(source)
    output = tmp_path / "output"
    monkeypatch.setattr(os, "sched_getaffinity", lambda _pid: {4, 5})
    calls: list[list[str]] = []

    def run(argv: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(argv)
        if argv[:3] == ["docker", "image", "inspect"]:
            return subprocess.CompletedProcess(argv, 0, "sha256:abc123\n", "")
        return subprocess.CompletedProcess(argv, 0, "gazebo ok\n", "")

    code = remote_model_job.execute_job(
        mode="gazebo",
        source=source,
        output=output,
        python=None,
        image="forklift/gazebo:mutable",
        duration=30,
        command_runner=run,
        environment={"SLURM_JOB_ID": "44", "SLURM_CPUS_PER_TASK": "2"},
    )

    result = json.loads((output / "job_result.json").read_text())
    docker_run = calls[1]
    assert code == 0
    assert "sha256:abc123" in docker_run
    assert "forklift/gazebo:mutable" not in docker_run
    assert result["container_image"] == {
        "requested": "forklift/gazebo:mutable",
        "id": "sha256:abc123",
    }


def _tree_listing(root: Path) -> list[tuple[str, int, bytes | None]]:
    return sorted(
        (
            str(p.relative_to(root)),
            p.stat().st_mode,
            p.read_bytes() if p.is_file() else None,
        )
        for p in root.rglob("*")
    )


def _write_buildable_source(source: Path) -> None:
    _write_source(source)
    (source / "pyproject.toml").write_text(
        "[build-system]\nrequires=['setuptools>=61']\n"
        "build-backend='setuptools.build_meta'\n"
        "[project]\nname='forklift-core'\nversion='0.0.1'\n"
        "[tool.setuptools.packages.find]\nwhere=['src']\n"
    )


def test_prepare_core_environment_installs_snapshot_wheel_into_run_venv(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source"
    _write_buildable_source(source)
    for path in [source, *source.rglob("*")]:  # mimic the submitter's chmod -R a-w
        path.chmod(path.stat().st_mode & ~0o222)
    try:
        before = _tree_listing(source)
        output = tmp_path / "output"
        runtime = output / ".runtime"
        runtime.mkdir(parents=True)
        commands: list[dict] = []

        env = remote_model_job.prepare_core_environment(
            python=sys.executable,
            source=source,
            output=output,
            runtime=runtime,
            environment=dict(os.environ),
            command_runner=subprocess.run,
            commands=commands,
        )

        assert Path(env.python).is_file() and str(runtime) in env.python
        assert Path(env.wheel_path).is_file() and env.wheel_path.startswith(str(output))
        assert env.import_path.startswith(
            str(runtime)
        )  # not the snapshot, not site-packages of the base interpreter
        assert not (source / "build").exists()  # read-only snapshot untouched
        assert not list(
            source.rglob("*.egg-info")
        )  # no in-tree build artefacts in the snapshot
        assert (
            _tree_listing(source) == before
        )  # contents and modes unchanged (see helper below)
        probe = subprocess.run(
            [env.python, "-c", "import forklift_core; print(forklift_core.VALUE)"],
            capture_output=True,
            text=True,
            check=True,
            cwd=tmp_path,
        )
        assert probe.stdout.strip() == "1"
        shutil.rmtree(
            runtime
        )  # the run-time copy must be removable despite the read-only source
        assert [c["label"] for c in commands] == [
            "core-wheel",
            "core-venv",
            "core-install",
            "core-import-check",
        ]
    finally:
        for path in [source, *source.rglob("*")]:
            path.chmod(path.stat().st_mode | (0o700 if path.is_dir() else 0o600))
