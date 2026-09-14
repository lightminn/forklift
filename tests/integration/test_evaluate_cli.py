import csv
import hashlib
import importlib.util
import json
import subprocess
from pathlib import Path

import pytest

from forklift_core.perception import pocket_detector
from forklift_core.perception.evaluation import POSITION_TOLERANCE_M, YAW_TOLERANCE_RAD
from forklift_core.perception.pocket_observation import PocketObservation
from forklift_core.perception.scene_dataset import SceneInput

REPO_ROOT = Path(__file__).resolve().parents[2]
REPO_PRIOR = REPO_ROOT / "config" / "pallet_prior_v1.yaml"
REAL_DATASET = REPO_ROOT / "data" / "synthetic_scenes" / "catalogue_v1"


def _load_cli():
    """tools/ is not an installed package, so load the CLI by path."""
    path = REPO_ROOT / "tools" / "evaluate_pocket_detector.py"
    spec = importlib.util.spec_from_file_location("evaluate_pocket_detector", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


cli = _load_cli()


def run(dataset, out, *extra):
    return cli.main(
        [
            "--dataset",
            str(dataset),
            "--prior",
            str(REPO_PRIOR),
            "--output",
            str(out),
            *extra,
        ]
    )


def test_the_cli_writes_metrics_scenes_and_overlays_for_a_small_dataset(
    tmp_path, pallet_scene
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)  # helper in this file
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev") == 0
    metrics = json.loads((out / "metrics.json").read_text())
    assert metrics["scene_count"] == 3
    assert metrics["run"]["split"] == "dev" and metrics["run"]["prior_sha256"]
    assert metrics["run"]["scene_ids"] == ["s001", "s002", "s003"]
    assert metrics["counts"]["positive"]["true_positive"] == 2
    assert metrics["counts"]["negative_no_pallet"]["true_negative"] == 1
    rows = list(csv.DictReader((out / "scenes.csv").read_text().splitlines()))
    assert [r["scene_id"] for r in rows] == ["s001", "s002", "s003"]
    assert {r["outcome"] for r in rows} == {"true_positive", "true_negative"}
    assert float(rows[0]["position_error_m"]) < 0.03
    assert (out / "overlay" / "s001.png").exists()
    assert (
        json.loads((out / "observations" / "s001.json").read_text())["status"]
        == "valid"
    )


def test_the_dev_run_never_hands_an_eval_scene_to_the_recognizer(
    tmp_path, pallet_scene, monkeypatch
):
    # the tiny data set holds s001-s003 in dev and s004 in eval
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    real = cli.detect_pockets
    calls = {"n": 0}

    def counting(*args, **kwargs):
        calls["n"] += 1
        return real(*args, **kwargs)

    monkeypatch.setattr(cli, "detect_pockets", counting)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--no-overlay") == 0
    assert calls["n"] == 3
    rows = list(csv.DictReader((out / "scenes.csv").read_text().splitlines()))
    assert {r["split"] for r in rows} == {"dev"}
    assert not (out / "observations" / "s004.json").exists()


def test_the_eval_split_evaluates_only_its_own_scene(tmp_path, pallet_scene):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "eval", "--no-overlay") == 0
    metrics = json.loads((out / "metrics.json").read_text())
    assert metrics["scene_count"] == 1
    assert metrics["run"]["scene_ids"] == ["s004"]


@pytest.mark.parametrize(
    "selection",
    ["", "s999", "s004"],
    ids=["empty", "unknown id", "id from the eval split"],
)
def test_bad_scene_selections_are_rejected(tmp_path, pallet_scene, selection):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    with pytest.raises(ValueError):
        run(dataset, tmp_path / "run", "--split", "dev", "--scenes", selection)


def test_the_scene_table_has_the_declared_columns(tmp_path, pallet_scene):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--no-overlay") == 0
    lines = (out / "scenes.csv").read_text().splitlines()
    assert lines[0].split(",") == [
        "scene_id",
        "category",
        "split",
        "truth_status",
        "estimate_status",
        "outcome",
        "left_error_m",
        "right_error_m",
        "position_error_m",
        "yaw_error_rad",
        "reason",
        "elapsed_s",
        "plane_residual_p95_m",
        "plane_inlier_count",
        "selected_support_min",
        "selected_lower",
        "selected_upper_left",
        "selected_upper_right",
        "left_front_frac",
        "left_behind_frac",
        "right_front_frac",
        "right_behind_frac",
    ]
    rows = {r["scene_id"]: r for r in csv.DictReader(lines)}
    # the no-pallet scene produces no opening rays: blank, never 0.0
    assert rows["s003"]["left_front_frac"] == ""
    assert rows["s003"]["right_behind_frac"] == ""
    # No pattern survived there either, so its gate terms are blank rather than
    # zero: zero would read as "measured none", which is a different claim.
    assert rows["s003"]["selected_support_min"] == ""
    assert rows["s003"]["selected_lower"] == ""
    # A detected scene reports which term was scarcest.
    detected = next(r for r in rows.values() if r["estimate_status"] == "valid")
    assert int(detected["selected_support_min"]) > 0
    assert int(detected["selected_upper_left"]) > 0
    assert int(detected["selected_upper_right"]) > 0
    assert 0.0 <= float(rows["s001"]["left_front_frac"]) <= 1.0


def test_the_cli_refuses_to_overwrite_an_existing_output_directory(
    tmp_path, pallet_scene
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    out = tmp_path / "run"
    out.mkdir()
    with pytest.raises(FileExistsError):
        run(dataset, out, "--split", "dev")


def test_unknown_parameter_keys_are_rejected(tmp_path, pallet_scene):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    params = tmp_path / "p.yaml"
    params.write_text("cell_m: 0.01\nnot_a_parameter: 1\n")
    with pytest.raises(ValueError):
        run(
            dataset,
            tmp_path / "run",
            "--split",
            "dev",
            "--params",
            str(params),
        )


def test_a_detector_exception_becomes_an_invalid_scene_and_does_not_stop_the_run(
    tmp_path, pallet_scene, monkeypatch
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    calls = {"n": 0}

    def boom(*args, **kwargs):
        calls["n"] += 1
        raise RuntimeError("synthetic failure")

    monkeypatch.setattr(cli, "detect_pockets", boom)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--no-overlay") == 0
    assert calls["n"] == 3
    metrics = json.loads((out / "metrics.json").read_text())
    assert sum(c.get("invalid", 0) for c in metrics["counts"].values()) == 3
    saved = json.loads((out / "observations" / "s001.json").read_text())
    assert saved["reason"] == "exception:RuntimeError"
    assert "synthetic failure" in saved["diagnostics"]["exception_traceback"]


def test_a_failure_inside_the_detector_is_saved_with_its_own_traceback(
    tmp_path, pallet_scene, monkeypatch
):
    # patched inside the recognizer, so the CLI never sees the exception: this
    # exercises Task 1 end to end instead of the CLI's own guard
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)

    def boom(*args, **kwargs):
        raise RuntimeError("synthetic internal failure")

    monkeypatch.setattr(pocket_detector, "_vertical_plane_candidates", boom)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--no-overlay") == 0
    saved = json.loads((out / "observations" / "s001.json").read_text())
    assert saved["reason"] == "exception:RuntimeError"
    trace = saved["diagnostics"]["exception_traceback"]
    assert "synthetic internal failure" in trace
    assert "_vertical_plane_candidates" in trace


def test_missing_optional_video_tool_does_not_fail_the_run(
    tmp_path, pallet_scene, monkeypatch
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    monkeypatch.setattr(cli.shutil, "which", lambda name: None)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--video") == 0
    assert json.loads((out / "run.json").read_text())["video_error"]
    assert not (out / "overlay.mp4").exists()


@pytest.mark.skipif(
    not REAL_DATASET.is_dir(), reason="synthetic scene data set v1 is not present"
)
def test_three_real_dev_scenes_run_end_to_end(tmp_path):
    out = tmp_path / "run"
    assert run(REAL_DATASET, out, "--split", "dev", "--scenes", "s003,s006,s010") == 0
    metrics = json.loads((out / "metrics.json").read_text())
    assert metrics["scene_count"] == 3
    assert metrics["run"]["dataset_manifest_sha256"]
    assert (out / "overlay" / "s003.png").exists()


_WRITER_SPEC = importlib.util.spec_from_file_location(
    "scene_writer", Path(__file__).resolve().parents[1] / "fixtures" / "scene_writer.py"
)
_WRITER = importlib.util.module_from_spec(_WRITER_SPEC)
_WRITER_SPEC.loader.exec_module(_WRITER)
write_v1_scene = _WRITER.write_v1_scene


def build_tiny_dataset(root, pallet_scene):
    """Write three dev scenes and one eval control, with explicit negative truth."""
    for scene_id, split, positive, kwargs in (
        ("s001", "dev", True, {}),
        ("s002", "dev", True, {"centre_xy_m": (2.5, 0.1), "yaw_rad": 0.2}),
        ("s003", "dev", False, {"pallet": False}),
        ("s004", "eval", True, {}),
    ):
        scene, truth = pallet_scene(**kwargs)
        if not positive:
            truth = PocketObservation(
                stamp_ns=scene.stamp_ns,
                clock_domain=scene.clock_domain,
                frame_id="base_link",
                source_provenance="synthetic_ground_truth",
                status="no_pallet",
                left=None,
                right=None,
                insertion_yaw_rad=None,
                position_sigma_m=None,
                yaw_sigma_rad=None,
                reason="no target pallet in scene",
            )
        write_v1_scene(
            root / "scenes" / scene_id,
            scene,
            truth,
            category="positive" if positive else "negative_no_pallet",
            split=split,
        )
    (root / "manifest.json").write_text(
        json.dumps(
            {"catalogue_version": "v1", "scene_ids": ["s001", "s002", "s003", "s004"]}
        )
        + "\n",
        encoding="utf-8",
    )
    return root


def test_run_metadata_records_effective_params_and_detector_inputs_have_no_labels(
    tmp_path, pallet_scene, monkeypatch
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    params = tmp_path / "params.yaml"
    params.write_text("seed: 42\n")
    inputs = []
    real = cli.detect_pockets

    def capture(scene_input, *args, **kwargs):
        inputs.append(scene_input)
        return real(scene_input, *args, **kwargs)

    monkeypatch.setattr(cli, "detect_pockets", capture)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--params", str(params)) == 0
    assert len(inputs) == 3
    assert all(type(scene_input) is SceneInput for scene_input in inputs)
    record = json.loads((out / "run.json").read_text())
    metrics = json.loads((out / "metrics.json").read_text())
    assert record == metrics["run"]
    assert record["params"]["seed"] == 42
    assert record["params"]["cell_m"] == 0.01
    assert record["params_path"] == str(params.resolve())
    assert record["scene_count"] == 3
    assert (
        record["dataset_manifest_sha256"]
        == hashlib.sha256((dataset / "manifest.json").read_bytes()).hexdigest()
    )
    assert record["prior_sha256"] == hashlib.sha256(REPO_PRIOR.read_bytes()).hexdigest()
    assert record["position_tolerance_m"] == POSITION_TOLERANCE_M
    assert record["yaw_tolerance_rad"] == YAW_TOLERANCE_RAD
    assert record["started_at_utc"] <= record["ended_at_utc"]
    assert record["python_version"] and record["numpy_version"]
    assert len(record["git_revision"]) == 40
    assert isinstance(record["git_dirty"], bool)
    negative = json.loads((out / "observations" / "s003.json").read_text())
    assert negative["diagnostics"]["left_front_frac"] is None
    positive = json.loads((out / "observations" / "s001.json").read_text())
    assert 0 <= positive["diagnostics"]["opening_rays"]["left"]["front_fraction"] <= 1


def test_metrics_are_the_summarizer_result_with_only_run_metadata_added(
    tmp_path, pallet_scene, monkeypatch
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    monkeypatch.setattr(cli, "summarize", lambda results: {"scene_count": len(results)})
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--no-overlay") == 0
    metrics = json.loads((out / "metrics.json").read_text())
    assert set(metrics) == {"scene_count", "run"}
    assert metrics["scene_count"] == 3
    assert not (out / "overlay").exists()


def test_missing_required_input_is_a_tool_failure(tmp_path, pallet_scene):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    (dataset / "scenes" / "s001" / "rgb.png").unlink()
    with pytest.raises(FileNotFoundError):
        run(dataset, tmp_path / "run", "--split", "dev", "--no-overlay")


def test_required_output_failure_keeps_a_traceback_and_fails_the_tool(
    tmp_path, pallet_scene, monkeypatch, capsys
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    real = Path.write_text

    def fail_observation(path, *args, **kwargs):
        if path.parent.name == "observations":
            raise OSError("synthetic output failure")
        return real(path, *args, **kwargs)

    monkeypatch.setattr(Path, "write_text", fail_observation)
    with pytest.raises(OSError, match="synthetic output failure"):
        run(dataset, tmp_path / "run", "--split", "dev", "--no-overlay")
    error = json.loads(capsys.readouterr().err)
    assert error["reason"] == "exception:OSError"
    assert "synthetic output failure" in error["diagnostics"]["exception_traceback"]


def test_an_incompatible_prior_and_band_are_rejected_before_recognition(
    tmp_path, pallet_scene, monkeypatch
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    params = tmp_path / "params.yaml"
    params.write_text("band_margin_m: 0.1\n")
    inputs = []
    monkeypatch.setattr(cli, "detect_pockets", lambda *args: inputs.append(args))
    with pytest.raises(ValueError, match="band_margin_m"):
        run(dataset, tmp_path / "run", "--split", "dev", "--params", str(params))
    assert inputs == []


def test_ffmpeg_failure_is_recorded_without_losing_metrics(
    tmp_path, pallet_scene, monkeypatch
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    monkeypatch.setattr(cli.shutil, "which", lambda name: "/synthetic/ffmpeg")
    real = cli.subprocess.run

    def fail_ffmpeg(command, *args, **kwargs):
        if Path(command[0]).name == "ffmpeg":
            raise subprocess.CalledProcessError(
                1, command, stderr="synthetic encoder failure"
            )
        return real(command, *args, **kwargs)

    monkeypatch.setattr(cli.subprocess, "run", fail_ffmpeg)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--video") == 0
    record = json.loads((out / "run.json").read_text())
    assert "synthetic encoder failure" in record["video_error"]
    assert json.loads((out / "metrics.json").read_text())["run"] == record


def test_the_script_returns_nonzero_for_configuration_errors(tmp_path):
    result = subprocess.run(
        [
            cli.sys.executable,
            str(REPO_ROOT / "tools" / "evaluate_pocket_detector.py"),
            "--dataset",
            str(tmp_path / "missing"),
            "--prior",
            str(REPO_PRIOR),
            "--output",
            str(tmp_path / "run"),
            "--split",
            "dev",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    error = json.loads(result.stderr)
    assert "Traceback" in error["diagnostics"]["exception_traceback"]
