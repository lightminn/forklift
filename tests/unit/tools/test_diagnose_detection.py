"""G3 detection-diagnosis tool (plan docs/plans/2026-10-01-g3-detection-diagnosis.md)."""

import dataclasses
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from tools import diagnose_detection as diag
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[3]
GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
PRIOR = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
PARAMS = DetectorParams.derived_for(PRIOR)
URDF = ROOT / "sim/models/epal6_pallet/pallet.urdf"
PARTS = diag.pallet_parts(URDF)
LEVEL = ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])  # base_link at the world origin


def truth(x, y=0.0, yaw=0.0):
    return diag.PalletTruth(x, y, 0.0, yaw, GEOMETRY.overall_depth_m, PARTS)


def scene(x, y=0.0, yaw=0.0, extra=()):
    boxes = [
        *scene_rig.place(scene_rig.pallet(GEOMETRY), x_m=x, y_m=y, yaw_rad=yaw),
        *extra,
    ]
    rendered = scene_rig.render(boxes, quantize=False)
    k = dataclasses.replace(rendered.intrinsics, cx=319.5, cy=239.5)
    return scene_rig.render(boxes, quantize=False, intrinsics=k)


def front_cover(x, gap_m, thickness_m=0.002):
    """A thin box over the whole approach face; its front surface gap_m ahead of it."""
    face = x - GEOMETRY.overall_depth_m / 2
    centre = face - gap_m + thickness_m / 2
    return scene_rig.Box((centre, 0.0, 0.08), (thickness_m, 1.0, 0.16))


def test_detection_ignores_the_truth():
    s = scene(3.0)
    plain = diag.diagnose(s, PRIOR, PARAMS)["detection"]
    for t in (truth(3.0), truth(3.4, 0.3, 0.2)):
        assert diag.diagnose(s, PRIOR, PARAMS, t, LEVEL)["detection"] == plain


@pytest.mark.parametrize(
    "x, y, yaw", [(2.5, 0.0, 0.0), (3.2, 0.2, 0.1), (4.0, -0.3, -0.15)]
)
def test_replicas_match_the_detector(x, y, yaw):
    out = diag.diagnose(scene(x, y, yaw), PRIOR, PARAMS, truth(x, y, yaw), LEVEL)
    assert out["extraction"]["replica_matches"]
    assert out["selection"]["replica_matches"]
    assert out["unmodified"]


def test_a_visible_pallet_agrees_with_the_truth_chain():
    # 2.5 m is detected by the current detector in this rig; 3.0 m is not
    # (lower-deck evidence), which the next test classifies.
    out = diag.diagnose(scene(2.5), PRIOR, PARAMS, truth(2.5), LEVEL)
    assert out["visibility"]["agreement"] == pytest.approx(1.0)
    assert out["family"]["family"] == "OK"
    assert any(p.get("front") for p in out["planes"])


def test_a_visible_but_undetected_pallet_names_its_stage():
    out = diag.diagnose(scene(3.0), PRIOR, PARAMS, truth(3.0), LEVEL)
    assert out["detection"]["observation"]["status"] == "no_pallet"
    assert out["family"]["family"] in {"C", "D"}
    assert any(p.get("front") for p in out["planes"])


def test_a_cover_three_centimetres_ahead_is_occlusion():
    """Codex's counterexample: depth agreement alone counted this as visible."""
    out = diag.diagnose(
        scene(3.0, extra=[front_cover(3.0, 0.03)]), PRIOR, PARAMS, truth(3.0), LEVEL
    )
    assert out["visibility"]["front_face_visible"] < PARAMS.min_plane_points
    assert out["family"] == {"family": "A", "why": "occluded"}


def test_a_cover_within_a_centimetre_cannot_be_told_apart():
    """The method's stated limit: a surface within 1 cm reads as the pallet."""
    out = diag.diagnose(
        scene(3.0, extra=[front_cover(3.0, 0.005)]), PRIOR, PARAMS, truth(3.0), LEVEL
    )
    assert out["visibility"]["front_face_visible"] >= PARAMS.min_plane_points


def test_a_pallet_out_of_view_is_outside_view():
    out = diag.diagnose(scene(3.0, y=8.0), PRIOR, PARAMS, truth(3.0, y=8.0), LEVEL)
    assert out["family"]["family"] == "A"
    assert out["visibility"]["pallet_pixels_in_view"] == 0


def test_lower_points_are_traced_to_parts():
    out = diag.diagnose(scene(2.5), PRIOR, PARAMS, truth(2.5), LEVEL)
    front = next(p for p in out["planes"] if p.get("front"))
    stages = front["lower_provenance"][0]["stages"]
    assert set(stages) == {
        "dropped_by_height",
        "dropped_by_depth_cap",
        "height",
        "dropped_by_depth_ahead",
        "depth_ahead",
        "depth_cap",
        "dropped_by_window",
        "window",
    }
    assert set(stages["window"]) <= {
        "bottom_board",
        "block",
        "stringer",
        "top_board",
        "outside",
        "ambiguous",
        "other_pallet",
    }
    assert sum(stages["window"].values()) == front["lower_provenance"][0]["lower"]


def test_float_difference_ignores_only_last_bits():
    assert (
        diag.max_float_difference({"a": [1.0, "x"]}, {"a": [1.0 + 1e-16, "x"]}) < 1e-15
    )
    assert diag.max_float_difference({"a": "x"}, {"a": "y"}) is None
    assert diag.max_float_difference({"a": 1}, {"a": 1.0}) is None
    assert diag.max_float_difference({"a": [1.0]}, {"a": [1.0, 2.0]}) is None


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_run(tmp_path, s, x, *, tamper_depth=False, mount="legacy", recorded=None, quantize=0):
    """A run directory shaped like run_transport's, around a rig scene."""
    # run_transport scenes are clock_domain/provenance "synthetic".
    s = dataclasses.replace(s, clock_domain="synthetic", source_provenance="synthetic")
    result = detect_pockets(s, PRIOR, PARAMS)
    depth = np.asarray(s.depth_m)
    np.save(
        tmp_path / "perception_capture_1_depth_m.npy",
        depth + (1e-3 if tamper_depth else 0),
    )
    sources = {}
    for name in diag.DETECTION_SOURCES:
        local = ROOT / ("src/" + name if name.startswith("forklift_core") else name)
        sources[name] = sha(local)
    record = {
        "arguments": {
            "perception_camera_axes": "ros",
            "perception_mount": mount,
            "pallet_prior_loaded": dataclasses.asdict(PRIOR),
            "pallet_geometry_loaded": {"overall_depth_m": GEOMETRY.overall_depth_m},
        },
        "pallet_urdf_sha256": sha(URDF),
        "source_sha256": sources,
        # The committed runner, so the mount gate can find it in git history.
        "script_sha256": sha(ROOT / "sim/isaac/run_transport.py"),
        "detector_params": dataclasses.asdict(PARAMS),
        "scenario": {"pickup": {"x_m": 3.0, "y_m": 0.0, "yaw_rad": 0.0}},
        "samples": [
            {
                "phase": "observe",
                "pallet_position_m": [3.0, 0.0, 0.0],
                "pallet_yaw_rad": 0.0,
            }
        ],
        "observation_attempts": [
            {
                "candidate_index": 0,
                "depth_sha256": diag.depth_array_sha256(depth),
                # The committed runner records each capture's mount.
                "base_from_optical": recorded
                or {
                    "translation_m": np.asarray(
                        s.base_from_optical.translation_m
                    ).tolist(),
                    "rotation": np.asarray(s.base_from_optical.rotation).tolist(),
                },
                "depth_quantize_mm": quantize,
                "capture_diagnostics": {
                    "accepted_pose": LEVEL,
                    "intrinsics": {
                        "integer_index": {
                            "matrix": [
                                [s.intrinsics.fx, 0.0, s.intrinsics.cx],
                                [0.0, s.intrinsics.fy, s.intrinsics.cy],
                                [0.0, 0.0, 1.0],
                            ]
                        }
                    },
                },
                "pocket_observation": diag._jsonable(result.observation),
                "detection_diagnostics": diag._jsonable(result.diagnostics),
            }
        ],
    }
    (tmp_path / "result.json").write_text(json.dumps(record))
    return tmp_path


def test_a_run_record_passes_its_gates(tmp_path):
    # The adapter's nominal mount is the rig's default camera.
    out = diag.diagnose_attempt(write_run(tmp_path, scene(2.5), 2.5), 1, URDF)
    gates = {k: v for k, v in out["gates"].items() if k != "sources_mismatched"}
    assert all(gates.values()), out["gates"]
    assert out["family"]["family"] == "OK"


def test_a_tampered_depth_fails_the_hash_gate(tmp_path):
    out = diag.diagnose_attempt(
        write_run(tmp_path, scene(3.0), 3.0, tamper_depth=True), 1, URDF
    )
    assert out["gates"]["depth_hash"] is False
    assert out["family"] == {"family": "undecidable", "why": "gate_failed"}


@pytest.mark.parametrize("x", [1.4, 1.74])
def test_top_boards_seen_from_above_are_not_the_front_face(x):
    """Codex: a +-1 cm band around the front plane counted top-board edges."""
    out = diag.diagnose(scene(x), PRIOR, PARAMS, truth(x), LEVEL)
    assert out["visibility"]["front_face_pixels"] == 0
    assert out["family"] == {"family": "A", "why": "outside_view"}


def test_a_selection_that_differs_from_the_detector_stops(monkeypatch):
    real = diag.detect_pockets

    def shifted(*args, **kwargs):
        result = real(*args, **kwargs)
        diagnostics = dataclasses.replace(
            result.diagnostics, selected_lower=result.diagnostics.selected_lower + 1
        )
        return dataclasses.replace(result, diagnostics=diagnostics)

    monkeypatch.setattr(diag, "detect_pockets", shifted)
    out = diag.diagnose(scene(2.5), PRIOR, PARAMS, truth(2.5), LEVEL)
    assert out["selection"]["replica_matches"] is False
    assert out["family"] == {
        "family": "undecidable",
        "why": "selection_replica_mismatch",
    }


def test_a_nan_against_a_number_is_a_difference():
    """Codex: abs(nan - x) vanished inside max() and passed the gate."""
    assert diag.max_float_difference([float("nan")], [1e-4]) is None
    assert diag.max_float_difference([float("nan")], [float("nan")]) == 0.0
    assert diag.max_float_difference([float("inf")], [1.0]) is None


def observe(samples):
    return {"samples": [dict(phase="observe", **s) for s in samples]}


def test_pallet_pose_checks_height_and_wraps_yaw():
    import math

    near_pi = observe(
        [
            {"pallet_position_m": [3.0, 0.0, 0.0], "pallet_yaw_rad": math.pi - 1e-5},
            {"pallet_position_m": [3.0, 0.0, 0.0], "pallet_yaw_rad": -math.pi + 1e-5},
        ]
    )
    pose, why = diag.observe_pallet_pose(near_pi)
    assert why is None and abs(abs(pose[3]) - math.pi) < 1e-4
    lifted = observe(
        [
            {"pallet_position_m": [3.0, 0.0, 0.0], "pallet_yaw_rad": 0.0},
            {"pallet_position_m": [3.0, 0.0, 0.1], "pallet_yaw_rad": 0.0},
        ]
    )
    pose, why = diag.observe_pallet_pose(lifted)
    assert pose is None and why.startswith("pallet_moved_during_observe")


def test_lower_stages_account_for_every_point():
    out = diag.diagnose(scene(2.5), PRIOR, PARAMS, truth(2.5), LEVEL)
    front = next(p for p in out["planes"] if p.get("front"))
    for entry in front["lower_provenance"]:
        stages = {k: sum(v.values()) for k, v in entry["stages"].items()}
        assert (
            stages["height"] - stages["dropped_by_depth_ahead"] == stages["depth_ahead"]
        )
        assert (
            stages["depth_ahead"] - stages["dropped_by_depth_cap"]
            == stages["depth_cap"]
        )
        assert stages["depth_cap"] - stages["dropped_by_window"] == stages["window"]
        assert stages["window"] == entry["lower"]
    assert out["preprocessing"]["removed_by"]["nonfinite"]["points"] >= 0


def test_an_unverified_truth_chain_makes_the_configuration_undecidable(tmp_path):
    record = diag.diagnose_attempt(write_run(tmp_path, scene(2.5), 2.5), 1, URDF)
    record["visibility"]["agreement"] = 0.5
    chains = diag.chain_check([record])
    assert not next(iter(chains.values()))["ok"]
    assert record["family"] == {
        "family": "undecidable",
        "why": "truth_chain_unverified",
    }


def test_a_replayed_pattern_at_another_position_stops(monkeypatch):
    """Codex: shifting a pattern's bounds kept widths and counts and passed."""
    real = diag.det._opening_candidates

    def shifted(plane, prior, params, workspace, *, plane_index=0):
        patterns, rejections = real(
            plane, prior, params, workspace, plane_index=plane_index
        )
        moved = [
            dataclasses.replace(
                p, gaps=tuple((a + 0.0625, b + 0.0625) for a, b in p.gaps)
            )
            for p in patterns
        ]
        return moved, rejections

    s = scene(2.5)
    reference = diag.detect_pockets(s, PRIOR, PARAMS)
    monkeypatch.setattr(diag, "detect_pockets", lambda *a, **k: reference)
    monkeypatch.setattr(diag.det, "_opening_candidates", shifted)
    out = diag.diagnose(s, PRIOR, PARAMS, truth(2.5), LEVEL)
    assert out["family"] == {
        "family": "undecidable",
        "why": "selection_replica_mismatch",
    }


def test_a_front_partly_out_of_view_is_not_occlusion():
    """Codex: all 34 front pixels in view were seen, yet the reason read occluded."""
    out = diag.diagnose(scene(3.0, y=1.735), PRIOR, PARAMS, truth(3.0, y=1.735), LEVEL)
    vis = out["visibility"]
    assert 0 < vis["front_face_pixels"] < PARAMS.min_plane_points
    assert vis["front_face_nearer"] == 0
    assert out["family"] == {"family": "A", "why": "partial_view"}


def test_a_recorded_mount_that_is_not_the_named_one_fails_the_mount_gate(tmp_path):
    """The run asked for carriage_low but recorded the legacy transform."""
    out = diag.diagnose_attempt(
        write_run(tmp_path, scene(2.5), 2.5, mount="carriage_low"), 1, URDF
    )
    assert out["gates"]["runner_mount"] is False
    assert out["family"] == {"family": "undecidable", "why": "gate_failed"}


def test_the_replayed_scene_uses_the_recorded_mount_and_rounding():
    import numpy as np

    adapter = diag._adapter()
    low = adapter.mount_base_from_optical("carriage_low")
    result = {"arguments": {"perception_mount": "carriage_low"}}
    attempt = {
        "base_from_optical": {"translation_m": list(low.translation_m), "rotation": np.asarray(low.rotation).tolist()},
        "depth_quantize_mm": 1,
        "capture_diagnostics": {"intrinsics": {"integer_index": {"matrix": [[465.7, 0, 319.5], [0, 465.7, 239.5], [0, 0, 1]]}}},
        "pocket_observation": {"stamp_ns": 1},
    }
    assert diag.recorded_mount_matches(result, attempt)
    depth = np.full((480, 640), 1.23449)
    scene = diag.attempt_scene(result, attempt, depth)
    np.testing.assert_allclose(scene.base_from_optical.translation_m, low.translation_m)
    assert float(scene.depth_m[0, 0]) == 1.234


def test_the_replay_uses_the_recorded_matrix_and_a_tiny_offset_fails_the_gate():
    import numpy as np

    adapter = diag._adapter()
    low = adapter.mount_base_from_optical("carriage_low")
    shifted = {"translation_m": [0.559005, 0.0, 0.27], "rotation": np.asarray(low.rotation).tolist()}
    result = {"arguments": {"perception_mount": "carriage_low"}}
    attempt = {
        "base_from_optical": shifted,
        "depth_quantize_mm": 0,
        "capture_diagnostics": {"intrinsics": {"integer_index": {"matrix": [[465.7, 0, 319.5], [0, 465.7, 239.5], [0, 0, 1]]}}},
        "pocket_observation": {"stamp_ns": 1},
    }
    assert not diag.recorded_mount_matches(result, attempt)  # rtol 0, atol 1e-9
    scene = diag.attempt_scene(result, attempt, np.full((480, 640), 2.0))
    assert scene.base_from_optical.translation_m[0] == 0.559005  # the record, not the table


def test_configurations_with_different_mounts_are_checked_separately(tmp_path):
    runs = []
    for name, mount in (("a", "legacy"), ("b", "carriage_low")):
        d = tmp_path / name
        d.mkdir()
        (d / "result.json").write_text(json.dumps({
            "script_sha256": "x", "pallet_urdf_sha256": "y",
            "arguments": {"perception_mount": mount, "depth_quantize_mm": 1 if mount != "legacy" else 0},
        }))
        runs.append(d)
    records = [
        {"run": str(runs[0]), "visibility": {"agreement": 1.0}, "family": {"family": "OK"}},
        {"run": str(runs[1]), "visibility": {"agreement": 0.1}, "family": {"family": "C"}},
    ]
    chains = diag.chain_check(records)
    assert len(chains) == 2
    assert records[1]["family"] == {"family": "undecidable", "why": "truth_chain_unverified"}
    assert records[0]["family"] == {"family": "OK"}
