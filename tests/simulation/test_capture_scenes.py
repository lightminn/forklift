"""Scene runner contracts using fake processes and a controlled monotonic clock."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[2]
CATALOGUE = ROOT / "sim/gazebo/scenes/catalogue_v1.yaml"


def runner():
    path = ROOT / "sim/gazebo/capture_scenes.py"
    assert path.exists(), "capture_scenes implementation is missing"
    spec = importlib.util.spec_from_file_location("tested_capture_runner", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_scene_range_is_inclusive_and_ordered():
    assert runner().parse_scene_range("s009-s011", ["s011", "s009", "s010"]) == [
        "s009",
        "s010",
        "s011",
    ]


@pytest.mark.parametrize(
    "text",
    [
        "s003-s001",
        "s1-s003",
        "s001",
        "s001,s002",
        " s001-s002",
        "s001-s002\n",
        "s001-s1000",
    ],
)
def test_invalid_scene_range_is_rejected(text):
    with pytest.raises(ValueError):
        runner().parse_scene_range(text, ["s001", "s002", "s003"])


def test_scene_range_requires_every_catalogue_id():
    with pytest.raises(ValueError, match="s002"):
        runner().parse_scene_range("s001-s003", ["s001", "s003"])


def test_manifest_preserves_metadata_counts_failures_and_requires_completion():
    api = runner()
    results = {
        "s001": {
            "passed": True,
            "error": None,
            "files": {"rgb.png": "digest"},
            "wall_times_s": {"captured": 3.0},
        },
        "s002": {"passed": False, "error": "timeout", "files": {}, "wall_times_s": {}},
    }
    manifest = api.build_manifest(
        catalogue={"catalogue_version": "v1", "camera": {"width": 640}},
        catalogue_sha256="b" * 64,
        image_id="sha256:image",
        source_sha="a" * 64,
        run_id="run-1",
        requested_scenes=["s001", "s002"],
        scenes=results,
    )
    assert manifest == {
        "catalogue_version": "v1",
        "camera": {"width": 640},
        "catalogue_sha256": "b" * 64,
        "image_id": "sha256:image",
        "source_snapshot_sha256": "a" * 64,
        "run_id": "run-1",
        "requested_scenes": ["s001", "s002"],
        "scenes": results,
        "failed_count": 1,
    }
    assert api.manifest_exit_code(manifest) == 1
    manifest["scenes"]["s002"]["passed"] = True
    manifest["failed_count"] = 0
    assert api.manifest_exit_code(manifest) == 0
    del manifest["scenes"]["s002"]
    assert api.manifest_exit_code(manifest) == 1
    assert results["s002"]["passed"] is False


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class Process:
    def __init__(self, name, command):
        self.name = name
        self.command = command
        self.returncode = None
        self.pid = 123

    def poll(self):
        return self.returncode


class Harness:
    """Replace only process execution and environment sourcing, retaining runner flow."""

    def __init__(self, api, monkeypatch, output, fail=None):
        self.api = api
        self.output = output
        self.fail = fail
        self.events = []
        self.instances = []
        self.timeouts = []
        self.clock = Clock()
        harness = self

        class Children:
            def __init__(self, output, env):
                self.output = output
                self.env = env
                self.processes = []
                harness.instances.append(self)

            def start(self, name, command, cwd=None):
                harness.events.append(
                    (self.output.name, name, command, dict(self.env), cwd)
                )
                process = Process(name, command)
                self.processes.append((name, process, command))
                if name == "scene_capture" and not self.fails("ready"):
                    (self.output / "ready.json").write_text("{}")
                if name == "gazebo" and not self.fails("clock_timeout"):
                    (self.output / "progress.json").write_text('{"counts":{"clock":1}}')
                if name == "bridge" and self.fails("early_bridge"):
                    process.returncode = 7
                return process

            def fails(self, phase):
                return self.output.name == "s001" and harness.fail == phase

            def wait(self, process, timeout_s, guards=()):
                harness.timeouts.append((self.output.name, process.name, timeout_s))
                assert timeout_s > 0
                if self.fails(process.name):
                    process.returncode = 7
                    raise RuntimeError(f"{process.name} exited early: 7")
                if any(p.poll() is not None for p in guards):
                    raise RuntimeError("guard exited early")
                harness.clock.sleep(2 if process.name == "build_world" else 1)
                if process.name == "scene_capture":
                    (self.output / "result.json").write_text(
                        json.dumps(
                            {
                                "passed": not self.fails("result"),
                                "error": "save failed"
                                if self.fails("result")
                                else None,
                                "stamp_ns": 2_400_000_000,
                                "counts": {"clock": 1},
                                "files": {"rgb.png": "rgb-digest"},
                                "wall_times_s": {"captured": 5.0},
                            }
                        )
                    )
                process.returncode = 0

            def close(self):
                harness.events.append(
                    (self.output.name, "closed", [], dict(self.env), None)
                )
                for _, process, _ in self.processes:
                    process.returncode = (
                        0 if process.returncode is None else process.returncode
                    )
                return ["gazebo: shutdown failed"] if self.fails("cleanup") else []

        monkeypatch.setattr(api, "Children", Children)
        monkeypatch.setattr(api, "time", self.clock)
        monkeypatch.setattr(api.smoke, "time", self.clock)

        def source_env(command, **kwargs):
            assert command[:2] == ["bash", "-c"]
            assert "install/setup.bash" in command[2]
            assert kwargs["check"] and kwargs["timeout"] > 0
            return NS(stdout=b"CAPTURE_INSTALLED=yes\0")

        monkeypatch.setattr(api.subprocess, "run", source_env)
        real_ready, real_clock = api.wait_ready, api.wait_clock

        def wait_ready(path, process, timeout_s):
            self.timeouts.append((path.parent.name, "ready", timeout_s))
            real_ready(path, process, timeout_s)
            self.events.append((path.parent.name, "ready", [], {}, None))
            self.clock.sleep(3)

        def wait_clock(path, timeout_s, guards):
            self.timeouts.append((path.parent.name, "clock", timeout_s))
            real_clock(path, timeout_s, guards)
            self.events.append((path.parent.name, "clock", [], {}, None))
            self.clock.sleep(4)

        monkeypatch.setattr(api, "wait_ready", wait_ready)
        monkeypatch.setattr(api, "wait_clock", wait_clock)

    def run(self, deadline=30):
        return self.api.run(
            NS(
                catalogue=CATALOGUE,
                scenes="s001-s002",
                output=self.output,
                image_id="sha256:image",
                source_sha256="a" * 64,
                run_id="run-1",
                scene_deadline_s=deadline,
            )
        )

    def manifest(self):
        return json.loads((self.output / "scene_capture/manifest.json").read_text())


def test_runner_orders_ready_clock_tf_and_builds_installed_package_once(
    tmp_path, monkeypatch
):
    api = runner()
    harness = Harness(api, monkeypatch, tmp_path)
    assert harness.run() == 0
    names = [event[1] for event in harness.events]
    assert (
        names.count("colcon_build")
        == names.count("colcon_test")
        == names.count("installed_import")
        == 1
    )
    for scene_id in ("s001", "s002"):
        scene_events = [event for event in harness.events if event[0] == scene_id]
        assert [event[1] for event in scene_events] == [
            "build_world",
            "scene_capture",
            "ready",
            "bridge",
            "gazebo",
            "clock",
            "static_tf",
            "closed",
        ]
        for _, _, command, env, _ in scene_events:
            if command:
                assert env["CAPTURE_INSTALLED"] == "yes"
        capture = next(e[2] for e in scene_events if e[1] == "scene_capture")
        for flag, value in [
            ("--image-id", "sha256:image"),
            ("--source-sha256", "a" * 64),
            ("--run-id", "run-1"),
        ]:
            assert capture[capture.index(flag) + 1] == value
        waits = {
            phase: limit
            for scene, phase, limit in harness.timeouts
            if scene == scene_id
        }
        assert waits == {
            "build_world": 30.0,
            "ready": 28.0,
            "clock": 25.0,
            "scene_capture": 21.0,
        }
        assert float(capture[capture.index("--deadline-s") + 1]) == 28.0
    test = next(e[2] for e in harness.events if e[1] == "colcon_test")
    assert test[test.index("--python-testing") + 1] == "pytest"
    check = next(e for e in harness.events if e[1] == "installed_import")
    assert "forklift_ros.scene_files" in check[2][-1]
    assert check[4] == tmp_path / ".runtime"
    manifest = harness.manifest()
    assert manifest["failed_count"] == 0
    assert manifest["scenes"]["s001"]["files"] == {"rgb.png": "rgb-digest"}
    assert set(manifest["scenes"]["s001"]["wall_times_s"]) == {
        "build_world",
        "ready",
        "first_clock",
        "captured",
        "stopped",
    }
    assert not list((tmp_path / "scene_capture").glob("*.tmp"))


@pytest.mark.parametrize(
    "failure",
    [
        "ready",
        "clock_timeout",
        "early_bridge",
        "scene_capture",
        "result",
        "cleanup",
        "build_world",
    ],
)
def test_scene_failures_are_recorded_cleaned_and_do_not_stop_remaining_scenes(
    tmp_path, monkeypatch, failure
):
    api = runner()
    harness = Harness(api, monkeypatch, tmp_path, fail=failure)
    # Observe the real manifest before starting scene 2, not only at final return.
    original = api.capture_scene

    def capture_scene(*args, **kwargs):
        if len(harness.instances) >= 2:
            manifest = harness.manifest()
            assert manifest["scenes"]["s001"]["passed"] is False
            assert manifest["failed_count"] == 1
        return original(*args, **kwargs)

    monkeypatch.setattr(api, "capture_scene", capture_scene)
    assert harness.run() == 1
    manifest = harness.manifest()
    assert manifest["failed_count"] == 1
    assert manifest["scenes"]["s001"]["error"]
    assert manifest["scenes"]["s002"]["passed"] is True
    assert sum(e[0] == "s001" and e[1] == "closed" for e in harness.events) == 1
    if failure == "ready":
        assert not any(e[0] == "s001" and e[1] == "gazebo" for e in harness.events)
    if failure == "cleanup":
        assert "shutdown failed" in manifest["scenes"]["s001"]["error"]


def test_scene_deadline_is_not_reset_between_waits(tmp_path, monkeypatch):
    api = runner()
    harness = Harness(api, monkeypatch, tmp_path)
    assert harness.run(deadline=4) == 1
    assert harness.manifest()["failed_count"] == 2
    assert not any(e[1] == "gazebo" for e in harness.events)
    assert sum(e[1] == "closed" for e in harness.events) == 3


def test_existing_capture_output_is_rejected_without_modification(
    tmp_path, monkeypatch
):
    api = runner()
    output = tmp_path / "scene_capture"
    output.mkdir()
    (output / "manifest.json").write_text("previous evidence")
    harness = Harness(api, monkeypatch, tmp_path)
    with pytest.raises(FileExistsError):
        harness.run()
    assert (output / "manifest.json").read_text() == "previous evidence"
    assert harness.events == []
