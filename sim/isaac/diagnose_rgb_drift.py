"""Observe RGB drift, independently of G1 scoring (2026-09-21 report section 9).

Run each condition in a NEW Isaac Sim Python process, with a NEW output directory:
    python sim/isaac/diagnose_rgb_drift.py --condition A \
        --base-scene /path/to/base.usd --output artifacts/drift_A
Repeat the command for B..H. A/B/D leave SimulationApp AA unspecified;
C requests anti_aliasing=0. Actual carb settings are read before/after captures.

A is the reference; every other condition names exactly one change against it
and records both sides of it (CONDITION_NOTES). E, F and G are runtime carb
writes, requested once before the first board and read straight back, so a write
that carb drops, refuses or coerces reads as such instead of being reported as
the request. H is a SimulationApp argument, because Isaac maps the launcher's
`renderer` onto /rtx/rendermode while the RTX plugins load and no later write
re-runs that setup; it is verified through the /rtx/rendermode readback that
every capture already records.

Only r=3 m, nine existing placements, twelve single-step captures per board.
No warmup, readiness retry, geometry edit or camera setter between captures.
Twelve is an observation length, never a convergence or acceptance threshold.

Each capture NPZ retains original RGB(A), depth and segmentation, the Boolean
target mask, corner coordinates and ROI bounds (exclusive right/bottom). JSON
retains label mappings, settings and readback errors. Each board's corners NPZ
is (12, 63, 2); missing detections remain NaN at their original capture index.
curves.csv and SVGs show signed means, translation-removed p95 and affine slopes
versus nominal projection, first capture and previous capture. Nothing is fitted
back into the camera. Neither G1 gates nor a sufficient warmup count are emitted.

CPU helpers import without Isaac. OpenCV is needed for corner extraction; the
runtime uses the existing G1 scene author and camera_calibration measurements,
but never invokes the G1 run/finish/scoring path.
"""

import argparse
import csv
import importlib.util
import json
import os
import sys
import traceback
from collections.abc import Callable
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CAPTURES = 12


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


MEASURE = load_module(
    "rgb_drift_measure", Path(__file__).with_name("camera_calibration.py")
)


CONDITIONS = ("A", "B", "C", "D", "E", "F", "G", "H")

# The stages E-G change are the ones report section 12.6 recorded as missing:
# "the sub-pixel jitter, sample pattern and reconstruction filter settings are
# not in the record". The observed starting values on ws1 are op 6
# (acesApproximation), sharpness 0.5 and 8 TAA samples; tonemap op 1 is the
# renderer's own "none" operator, i.e. no tone-mapping operation at all.
RENDER_SETTING_OVERRIDES = {
    "E": {"/rtx/post/tonemap/op": 1},
    "F": {"/rtx/post/aa/sharpness": 0.0},
    "G": {"/rtx/post/taa/samples": 1},
}

_UNCHANGED_BY_A_RUNTIME_WRITE = (
    "the renderer, the DLSS anti-aliasing mode, rt_subframes 4, forward board "
    "order, the nine placements, r = 3.0 m and the capture history"
)

# What each condition moves and what it leaves where A has it. Kept beside the
# request so the record answers it without re-deriving it from the settings.
CONDITION_NOTES = {
    "A": {
        "changed": "nothing; A is the reference the others are read against",
        "unchanged": (
            "everything: Isaac's default DLSS anti-aliasing, rt_subframes 4, "
            "forward board order, RaytracedLighting, the acesApproximation "
            "tone mapper, 0.5 sharpening and the 8-sample TAA sequence"
        ),
        "expressed_as": "none",
    },
    "B": {
        "changed": "rt_subframes 4 -> 16",
        "unchanged": "every carb render setting, the renderer, board order and placements",
        "expressed_as": "orchestrator step argument",
    },
    "C": {
        "changed": "anti_aliasing 3 (DLSS) -> 0, which Isaac writes to /rtx/post/aa/op",
        "unchanged": (
            "the renderer, the tone mapper, sharpening, the TAA sequence, "
            "rt_subframes 4, board order and placements"
        ),
        "expressed_as": "SimulationApp argument",
    },
    "D": {
        "changed": "board order reversed; position 8 is authored first",
        "unchanged": "every carb render setting, the renderer, rt_subframes 4 and the placements themselves",
        "expressed_as": "position ordering",
    },
    "E": {
        "changed": (
            "/rtx/post/tonemap/op 6 (acesApproximation) -> 1 (none: the renderer "
            "applies no tone-mapping operation)"
        ),
        "unchanged": "sharpening, the TAA sequence, " + _UNCHANGED_BY_A_RUNTIME_WRITE,
        "expressed_as": "runtime carb write",
    },
    "F": {
        "changed": "/rtx/post/aa/sharpness 0.5 -> 0.0",
        "unchanged": "the tone mapper, the TAA sequence, "
        + _UNCHANGED_BY_A_RUNTIME_WRITE,
        "expressed_as": "runtime carb write",
    },
    "G": {
        "changed": "/rtx/post/taa/samples 8 -> 1",
        "unchanged": "the tone mapper, sharpening, " + _UNCHANGED_BY_A_RUNTIME_WRITE,
        "expressed_as": "runtime carb write",
    },
    "H": {
        "changed": "/rtx/rendermode RaytracedLighting -> PathTracing",
        "unchanged": (
            "the path-tracing sample count (/rtx/pathtracing/spp) and the OptiX "
            "denoiser stay at Isaac's launcher defaults, 64 and on; so do the "
            "anti-aliasing mode, the tone mapper, sharpening, the TAA sequence, "
            "rt_subframes 4, board order and the nine placements"
        ),
        "expressed_as": "SimulationApp argument",
    },
}


def condition_config(condition: str) -> dict:
    """Build independent requests; default AA must remain unspecified in A/B/D."""
    if condition not in CONDITIONS:
        raise ValueError("Unknown condition; choose one of " + ", ".join(CONDITIONS))
    app = {
        "headless": True,
        "renderer": "PathTracing" if condition == "H" else "RaytracedLighting",
    }
    if condition == "C":
        app["anti_aliasing"] = 0
    positions = [
        {"position_index": i, "centre_uv": list(uv)}
        for i, uv in enumerate(MEASURE.SCREEN_CENTRES_UV)
    ]
    return {
        "condition": condition,
        "simulation_app": app,
        "render_setting_overrides": deepcopy(
            RENDER_SETTING_OVERRIDES.get(condition, {})
        ),
        "changes_relative_to_a": dict(CONDITION_NOTES[condition]),
        "step": {
            "rt_subframes": 16 if condition == "B" else 4,
            "delta_time": 0.0,
            "pause_timeline": True,
        },
        "distance_m": 3.0,
        "captures_per_board": CAPTURES,
        "positions": positions[::-1] if condition == "D" else positions,
        "initial_history": {
            "stage_app_updates": 20,
            "settling_world_steps": 60,
            "board_warmup_steps": 0,
            "capture_retries": 0,
        },
    }


def displacement_statistics(current: np.ndarray, reference: np.ndarray) -> dict:
    """Summarize current-reference (N,2) pixels, excluding nonfinite pairs.

    Regression rows are residual du,dv; columns are [1,u-u_mean,v-v_mean]
    evaluated at REFERENCE corner locations. Slopes are px/px. The p95 is
    computed BEFORE regression, after removing translation only. Rank-deficient
    fits are unavailable, not silently interpreted as zero rotation/shear.
    """
    current, reference = np.asarray(current, float), np.asarray(reference, float)
    if current.shape != reference.shape or current.ndim != 2 or current.shape[1] != 2:
        raise ValueError("Corner arrays must have matching (N,2) shapes")
    valid = np.isfinite(current).all(axis=1) & np.isfinite(reference).all(axis=1)
    n = int(valid.sum())
    result = {
        "count": n,
        "unavailable_count": len(valid) - n,
        "mean_delta_uv_px": None,
        "centered_euclidean_p95_px": None,
        "centered_abs_p95_uv_px": None,
        "regression_origin_uv_px": None,
        "regression_rank": 0,
        "residual_affine_coefficients": None,
    }
    if not n:
        return result
    delta = current[valid] - reference[valid]
    mean = delta.mean(axis=0)
    residual = delta - mean
    origin = reference[valid].mean(axis=0)
    design = np.column_stack((np.ones(n), reference[valid] - origin))
    coefficients, _, rank, _ = np.linalg.lstsq(design, residual, rcond=None)
    result.update(
        mean_delta_uv_px=mean.tolist(),
        centered_euclidean_p95_px=float(
            np.percentile(np.linalg.norm(residual, axis=1), 95)
        ),
        centered_abs_p95_uv_px=np.percentile(np.abs(residual), 95, axis=0).tolist(),
        regression_origin_uv_px=origin.tolist(),
        regression_rank=int(rank),
        residual_affine_coefficients=coefficients.T.tolist() if rank == 3 else None,
    )
    return result


def mask_geometry(mask: np.ndarray) -> dict:
    """Mirror checkerboard_corners' 4-pixel ROI padding, without changing it."""
    rows, cols = np.nonzero(mask)
    if not len(rows):
        return {
            "bbox_xyxy_exclusive": None,
            "crop_xyxy_exclusive": None,
            "crop_origin_uv": None,
            "pixel_count": 0,
        }
    x0, y0, x1, y1 = (
        int(cols.min()),
        int(rows.min()),
        int(cols.max()) + 1,
        int(rows.max()) + 1,
    )
    crop = [
        max(0, x0 - 4),
        max(0, y0 - 4),
        min(mask.shape[1], x1 + 4),
        min(mask.shape[0], y1 + 4),
    ]
    return {
        "bbox_xyxy_exclusive": [x0, y0, x1, y1],
        "crop_xyxy_exclusive": crop,
        "crop_origin_uv": crop[:2],
        "pixel_count": len(rows),
    }


def depth_difference(
    current: np.ndarray, reference: np.ndarray, selection: np.ndarray | None = None
) -> dict:
    """Compare metre axial depth at identical integer pixels, never RGB corners.

    Both values must be finite and positive; validity changes are counted
    separately. An optional selection is FIXED from the first capture's mask.
    An empty common support has null statistics, never an apparent zero delta.
    """
    current, reference = np.asarray(current), np.asarray(reference)
    if current.ndim != 2 or current.shape != reference.shape:
        raise ValueError("Depth arrays must have matching (H,W) shapes")
    selected = (
        np.ones(current.shape, bool)
        if selection is None
        else np.asarray(selection, bool)
    )
    if selected.shape != current.shape:
        raise ValueError("Depth selection shape mismatch")
    a, b = (
        np.isfinite(current) & (current > 0),
        np.isfinite(reference) & (reference > 0),
    )
    common = selected & a & b
    delta = current[common].astype(float) - reference[common].astype(float)
    return {
        "common_valid_count": len(delta),
        "changed_valid_count": int(np.count_nonzero(delta)),
        "validity_changed_count": int(np.count_nonzero(selected & (a != b))),
        "mean_delta_m": float(delta.mean()) if len(delta) else None,
        "max_abs_delta_m": float(np.abs(delta).max()) if len(delta) else None,
        "p95_abs_delta_m": float(np.percentile(np.abs(delta), 95))
        if len(delta)
        else None,
    }


def native(value):
    """The record's only accepted conversions; anything else stays unsupported."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Unsupported record type: {type(value).__name__}")


def recordable_setting_value(value):
    """Keep a carb value the record cannot encode as text, never drop its key.

    A setting whose type is version-dependent must not cost the whole condition,
    so an unencodable value (including NaN/inf, which allow_nan=False rejects)
    is replaced by its repr and its original type name. Everything encodable is
    returned untouched, and write_record stays strict for every other field.
    """
    try:
        json.dumps(value, allow_nan=False, default=native)
    except (TypeError, ValueError, RecursionError):
        try:
            text = repr(value)
        except Exception as exc:  # A carb proxy may refuse even repr().
            text = f"<repr failed: {type(exc).__name__}>"
        return {"unserializable_type": type(value).__name__, "value_repr": text}
    return value


RENDER_SETTING_KEYS = (
    # The anti-aliasing set that refuted DLSS.
    "/rtx/post/aa/op",
    "/rtx/rendermode",
    "/rtx/post/dlss/execMode",
    "/rtx/post/dlss/enabled",
    "/rtx/post/dlss/rr/enabled",
    "/rtx-transient/dlssg/enabled",
    # The stages conditions E-H move, read for EVERY condition so that each one
    # has A's actual value to be read against, and so that H's claim to leave
    # the path-tracing sample count and denoiser alone is checkable.
    "/rtx/post/tonemap/op",
    "/rtx/post/aa/sharpness",
    "/rtx/post/taa/samples",
    "/rtx/pathtracing/spp",
    "/rtx/pathtracing/optixDenoiser/enabled",
)


def read_render_settings(settings) -> dict:
    """Read carb, recording missing/version-dependent keys and errors explicitly."""
    values, errors = {}, {}
    for key in RENDER_SETTING_KEYS:
        try:
            values[key] = recordable_setting_value(settings.get(key))
        except Exception as exc:
            values[key] = None
            errors[key] = f"{type(exc).__name__}: {exc}"
    return {"values": values, "readback_errors": errors}


def set_render_setting(settings, key: str, value) -> None:
    """Write with the type carb registered for the key, as Isaac's helper does.

    bool is tested before int because bool subclasses int: an /enabled key
    written through set_int is one of the silent non-applications this tool
    exists to expose, rather than a setting that took.
    """
    if isinstance(value, bool):
        settings.set_bool(key, value)
    elif isinstance(value, int):
        settings.set_int(key, value)
    elif isinstance(value, float):
        settings.set_float(key, value)
    elif isinstance(value, str):
        settings.set_string(key, value)
    else:
        raise TypeError(f"Unsupported carb value type: {type(value).__name__}")


def apply_render_settings(settings, overrides: dict) -> dict:
    """Request each override, then read the same key straight back from carb.

    DLSS was refuted on a read-back value, never a requested one, and the same
    rule holds here: `requested` and `observed` stay side by side, and `applied`
    is exact equality against what carb returned. A write carb drops (an
    unregistered key reads back None), refuses (raises) or coerces therefore
    reads as `applied: false` instead of being reported as the request. An
    empty override map writes nothing at all, which is how A-D stay untouched.
    """
    record = {}
    for key, value in overrides.items():
        entry = {"requested": recordable_setting_value(value)}
        try:
            entry["before"] = recordable_setting_value(settings.get(key))
        except Exception as exc:
            entry["before"] = None
            entry["before_error"] = f"{type(exc).__name__}: {exc}"
        try:
            set_render_setting(settings, key, value)
        except Exception as exc:
            entry["set_error"] = f"{type(exc).__name__}: {exc}"
        try:
            observed = settings.get(key)
        except Exception as exc:
            entry["observed"] = None
            entry["readback_error"] = f"{type(exc).__name__}: {exc}"
        else:
            entry["observed"] = recordable_setting_value(observed)
            entry["applied"] = observed == value
        record[key] = entry
    return record


def capture_once(rep, annotators: dict, step: dict) -> dict:
    """Exactly one orchestrator call; own buffers before the next call."""
    rep.orchestrator.step(**step)
    return {
        name: deepcopy(annotator.get_data()) for name, annotator in annotators.items()
    }


def scene_transforms(scene, paths: dict) -> dict:
    """Read the named world matrices fresh, in one place, with no cache."""
    return {name: scene.world_matrix(path) for name, path in paths.items()}


def changed_transforms(before: dict, after: dict) -> dict:
    """Record which matrices the step moved, exactly; never judge the amount.

    Exact equality keeps a tolerance out of a diagnostic: both matrices are in
    the record, so the size of any change is read off them, not decided here.
    """
    return {
        name: not np.array_equal(np.asarray(matrix), np.asarray(after.get(name)))
        for name, matrix in before.items()
    }


def make_capture(scene, paths: dict, timeline, rep, annotators, step, settings, camera):
    """Build one board's capture: read transforms, step once, read them back.

    The tool's premise is that the scene is untouched between captures, so the
    transforms are measured on both sides of the step rather than assumed; a
    change caused within the step is otherwise invisible in the record.
    """

    def capture():
        before_matrices = scene_transforms(scene, paths)
        before = {
            "timeline_time_s": float(timeline.get_current_time()),
            **before_matrices,
        }
        frame = capture_once(rep, annotators, step)
        after = scene_transforms(scene, paths)
        metadata = {
            "before": before,
            "after": after,
            "transforms_changed": changed_transforms(before_matrices, after),
            "time_after_s": float(timeline.get_current_time()),
            "timeline_playing_after": bool(timeline.is_playing()),
            "render_settings": read_render_settings(settings),
            "step_request": step,
            "actual_frame_source": "Camera.get_current_frame; identity with Replicator buffers unverified",
        }
        try:
            sdk_frame = camera.get_current_frame()
            metadata["camera_frame_readback"] = {
                key: deepcopy(sdk_frame.get(key))
                for key in ("rendering_frame", "rendering_time")
            }
        except Exception as exc:
            metadata["camera_frame_readback_error"] = str(exc)
        return frame, metadata

    return capture


def write_record(path: Path, record: dict) -> None:
    path.write_text(
        json.dumps(record, default=native, allow_nan=False, indent=2) + "\n",
        encoding="utf-8",
    )


def array_difference(current: np.ndarray, reference: np.ndarray) -> dict:
    """Keep exact identity and per-pixel differences distinct from shape changes."""
    matching = current.shape == reference.shape
    return {
        "same_shape": matching,
        "same_dtype": current.dtype == reference.dtype,
        "bitwise_identical": matching
        and current.dtype == reference.dtype
        and current.tobytes() == reference.tobytes(),
        "changed_pixel_count": int(np.count_nonzero(current != reference))
        if matching
        else None,
    }


def collect_board(
    directory: Path, position_index: int, capture: Callable, nominal_uv: np.ndarray
) -> dict:
    """Collect twelve immutable-scene observations; persist before the next step.

    capture returns (owned annotator payloads, readback metadata). Extraction
    errors leave explicit NaN corner slots and do not cause extra render steps.
    """
    name = f"board_p{position_index}"
    corners = np.full((CAPTURES, len(nominal_uv), 2), np.nan)
    board = {
        "position_index": position_index,
        "captures": [],
        "corners_file": name + "_corners.npz",
    }
    first = previous = None
    for index in range(CAPTURES):
        frame, metadata = capture()
        rgb = (
            np.asarray(frame["rgb"])
            if frame["rgb"] is not None
            else np.empty((0, 0, 3), np.uint8)
        )
        depth = (
            np.asarray(frame["z"])
            if frame["z"] is not None
            else np.empty((0, 0), np.float32)
        )
        if depth.ndim == 3 and depth.shape[-1] == 1:
            depth = depth[..., 0]
        payload = frame["seg"] if isinstance(frame["seg"], dict) else {}
        segmentation = (
            np.asarray(payload["data"])
            if payload.get("data") is not None
            else np.empty((0, 0), np.uint32)
        )
        record = {
            "capture_index": index,
            "step_since_board_authored": index + 1,
            "raw_file": f"{name}_capture{index:02d}.npz",
            "metadata": metadata,
            "segmentation_info": payload.get("info", {}),
            "measurement_error": None,
            "missing_buffers": {
                "rgb": frame["rgb"] is None,
                "depth": frame["z"] is None,
                "segmentation": payload.get("data") is None,
            },
        }
        # Save the raw inputs even if semantic/corner decoding subsequently fails.
        raw = {"rgb": rgb, "depth_m": depth, "segmentation": segmentation}
        np.savez_compressed(directory / record["raw_file"], **raw)
        # An all-black frame with wholly nonfinite depth is G1's known
        # empty_render_frame. Name it here; a corner error alone describes the
        # semantic ROI, not the render, and would hide it in the record.
        record["render"] = MEASURE.render_diagnostics(rgb, depth)
        mask = np.zeros(rgb.shape[:2] if rgb.ndim >= 2 else (0, 0), bool)
        mask_available = False
        try:
            if (
                rgb.ndim != 3
                or rgb.shape[-1] not in (3, 4)
                or not rgb.size
                or rgb.dtype != np.uint8
            ):
                raise ValueError("RGB buffer is unavailable or malformed")
            mask = MEASURE.semantic_mask(payload, "calibration_board")
            if mask.shape != rgb.shape[:2]:
                raise ValueError("Segmentation/RGB shape mismatch")
            mask_available = True
            corners[index] = MEASURE.checkerboard_corners(rgb, mask)
        except Exception as exc:
            # Includes cv2.error; a missing measurement must not shift the
            # subsequent observation indices by forcing a retry or early exit.
            record["measurement_error"] = f"{type(exc).__name__}: {exc}"
        record["mask_available"] = mask_available
        roi = mask_geometry(mask)
        record["roi"] = roi
        raw.update(
            mask=mask,
            mask_available=np.array(mask_available),
            corners_uv_px=corners[index],
            nominal_uv_px=nominal_uv,
        )
        for key in ("bbox_xyxy_exclusive", "crop_xyxy_exclusive", "crop_origin_uv"):
            raw[key] = np.asarray(
                roi[key] if roi[key] is not None else [], dtype=np.int64
            )
        np.savez_compressed(directory / record["raw_file"], **raw)
        current = {
            "rgb": rgb,
            "mask": mask,
            "mask_available": mask_available,
            "segmentation": segmentation,
            "depth": depth.copy(),
        }
        if first is None:
            first = current
        record["vs_nominal"] = displacement_statistics(corners[index], nominal_uv)
        record["vs_first"] = displacement_statistics(corners[index], corners[0])
        record["vs_previous"] = displacement_statistics(
            corners[index],
            corners[index - 1] if index else np.full_like(nominal_uv, np.nan),
        )
        for label, reference in (("first", first), ("previous", previous)):
            # The claim under test is that RGB alone changes while depth and
            # segmentation stay bit-identical. Record all three the same way,
            # so the record answers it without reloading every capture NPZ.
            record[f"rgb_vs_{label}"] = (
                array_difference(rgb, reference["rgb"])
                if reference is not None
                else None
            )
            record[f"mask_vs_{label}"] = (
                array_difference(mask, reference["mask"])
                if reference is not None
                and mask_available
                and reference["mask_available"]
                else None
            )
            record[f"segmentation_vs_{label}"] = (
                array_difference(segmentation, reference["segmentation"])
                if reference is not None
                else None
            )
            record[f"depth_vs_{label}"] = None
            record[f"target_depth_vs_{label}"] = None
            if (
                reference is not None
                and depth.ndim == 2
                and depth.shape == reference["depth"].shape
            ):
                record[f"depth_vs_{label}"] = depth_difference(
                    depth, reference["depth"]
                )
                if first["mask_available"] and first["mask"].shape == depth.shape:
                    record[f"target_depth_vs_{label}"] = depth_difference(
                        depth, reference["depth"], first["mask"]
                    )
        previous = current
        board["captures"].append(record)
        np.savez_compressed(
            directory / board["corners_file"],
            corners_uv_px=corners,
            nominal_uv_px=nominal_uv,
            capture_indices=np.arange(CAPTURES),
            captured_count=index + 1,
        )
        write_record(directory / f"{name}.json", board)
    return board


def write_curves(directory: Path, boards: list[dict]) -> None:
    """Write signed curve data and dependency-free, shared-scale SVG small multiples."""
    rows = []
    labels = ("nominal", "first", "previous")
    for board in boards:
        for capture in board["captures"]:
            row = {
                "position_index": board["position_index"],
                "capture_index": capture["capture_index"],
                "step_since_board_authored": capture["capture_index"] + 1,
            }
            for label in labels:
                stats = capture[f"vs_{label}"]
                mean = stats["mean_delta_uv_px"] or [None, None]
                row.update(
                    {
                        f"{label}_mean_du_px": mean[0],
                        f"{label}_mean_dv_px": mean[1],
                        f"{label}_centered_p95_px": stats["centered_euclidean_p95_px"],
                    }
                )
                coef = stats["residual_affine_coefficients"]
                for axis, component in enumerate(("du", "dv")):
                    for column, variable in enumerate(("intercept", "du", "dv")):
                        row[f"{label}_{component}_{variable}"] = (
                            coef[axis][column] if coef else None
                        )
            rows.append(row)
    if not rows:
        return
    with (directory / "curves.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    for label in (*labels, "nominal_affine"):
        keys = (
            ["nominal_du_du", "nominal_du_dv", "nominal_dv_du", "nominal_dv_dv"]
            if label == "nominal_affine"
            else [
                f"{label}_mean_du_px",
                f"{label}_mean_dv_px",
                f"{label}_centered_p95_px",
            ]
        )
        values = [row[key] for row in rows for key in keys if row[key] is not None]
        low, high = min([0.0, *values]), max([0.0, *values])
        span = high - low or 1.0
        low, high = low - 0.08 * span, high + 0.08 * span
        colors = ("#0072b2", "#d55e00", "#009e73", "#cc79a7")
        svg = [
            '<svg xmlns="http://www.w3.org/2000/svg" width="1260" height="960" viewBox="0 0 1260 960">',
            '<rect width="1260" height="960" fill="white"/>',
            f'<text x="30" y="26" font-size="18">{label}: observations only; x = step since board authored (1..12)</text>',
        ]
        for i, key in enumerate(keys):
            svg.append(
                f'<text x="30" y="{48 + i * 18}" fill="{colors[i]}" font-size="14">{key}</text>'
            )
        for position in range(9):
            x0, y0 = 65 + 415 * (position % 3), 160 + 265 * (position // 3)

            def y(value, origin=y0, upper=high, lower=low):
                return origin + 180 * (upper - value) / (upper - lower)

            svg.extend(
                [
                    f'<text x="{x0}" y="{y0 - 20}" font-size="16">position {position}</text>',
                    f'<path d="M{x0},{y0} V{y0 + 180} H{x0 + 320}" fill="none" stroke="#777"/>',
                    f'<path d="M{x0},{y(0):.2f} H{x0 + 320}" stroke="#bbb"/>',
                    f'<text x="{x0 - 58}" y="{y0 + 5}" font-size="11">{high:.4g}</text>',
                    f'<text x="{x0 - 58}" y="{y0 + 180}" font-size="11">{low:.4g}</text>',
                    f'<text x="{x0}" y="{y0 + 200}" font-size="12">1</text>',
                    f'<text x="{x0 + 308}" y="{y0 + 200}" font-size="12">12</text>',
                ]
            )
            selected = [row for row in rows if row["position_index"] == position]
            for key, color in zip(keys, colors, strict=False):
                path, pen = [], "M"
                for row in selected:
                    value = row[key]
                    if value is None:
                        pen = "M"
                        continue
                    x = x0 + 320 * row["capture_index"] / (CAPTURES - 1)
                    path.append(f"{pen}{x:.2f},{y(value):.2f}")
                    svg.append(
                        f'<circle cx="{x:.2f}" cy="{y(value):.2f}" r="2.5" fill="{color}"/>'
                    )
                    pen = "L"
                svg.append(
                    f'<path d="{" ".join(path)}" fill="none" stroke="{color}" stroke-width="1.5"/>'
                )
        svg.append("</svg>")
        (directory / f"curves_{label}.svg").write_text(
            "\n".join(svg) + "\n", encoding="utf-8"
        )


def run(app, args: argparse.Namespace, result: dict) -> None:
    """Set up identical history, then only step/read while each board is present."""
    import carb
    import cv2
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.sensors.camera import Camera

    import forklift_core

    # Reuse only scene authoring and SDK-free utilities, never G1 judgments.
    existing = load_module(
        "rgb_drift_scene", ROOT / "sim/isaac/verify_perception_camera.py"
    )
    adapter = load_module("rgb_drift_adapter", ROOT / "sim/isaac/perception_adapter.py")
    rig = load_module("rgb_drift_rig", ROOT / "tools/scene_rig.py")
    settings = carb.settings.get_settings()
    result["render_settings_at_start"] = read_render_settings(settings)
    # Once, before the stage is opened and before any render product exists, so
    # all twelve captures of every board see the same setting. A-D pass an empty
    # map and write nothing.
    result["render_setting_overrides"] = apply_render_settings(
        settings, result["protocol"]["render_setting_overrides"]
    )
    result["render_settings_after_overrides"] = read_render_settings(settings)
    result["source_sha256"] = existing.RUNNER.source_sha256(
        ROOT, Path(forklift_core.__file__).parent
    )
    cv2.ocl.setUseOpenCL(False)
    result["opencv"] = {
        "version": cv2.__version__,
        "use_opencl": bool(cv2.ocl.useOpenCL()),
    }
    write_record(args.output / "diagnostic.json", result)
    if result["opencv"]["use_opencl"]:
        raise RuntimeError("OpenCV OpenCL disable did not take effect")
    if not omni.usd.get_context().open_stage(args.base_scene):
        raise RuntimeError("Cannot open base scene")
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    base_path = "/World/Forklift/base_link"
    if not stage.GetPrimAtPath(base_path).IsValid():
        raise ValueError(f"Base scene must contain {base_path}")
    scene = existing.CalibrationScene(stage, base_path)
    nominal_mount = adapter.default_base_from_optical()
    nominal_matrix = MEASURE.mount_matrix(nominal_mount)
    raw_k = rig.intrinsics()
    nominal_k = adapter.normalize_isaac_intrinsics(raw_k).integer_index
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    world.scene.add(
        Robot(
            prim_path="/World/Forklift",
            name="forklift",
            position=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
    )
    camera_path = base_path + "/PerceptionCamera"
    camera = Camera(
        prim_path=camera_path, frequency=-1, resolution=(raw_k.width, raw_k.height)
    )
    camera.set_local_pose(
        translation=np.asarray(nominal_mount.translation_m),
        orientation=np.asarray(adapter.xyzw_to_wxyz(rig.OPTICAL_QUATERNION_XYZW)),
        camera_axes="ros",
    )
    camera.set_projection_mode("perspective")
    camera.set_lens_distortion_model("pinhole")
    camera.set_focal_length(1.0)
    camera.set_horizontal_aperture(raw_k.width / raw_k.fx, maintain_square_pixels=True)
    world.reset()
    camera.initialize()
    annotators = {}
    try:
        MEASURE.attach_annotators(rep, camera, annotators=annotators)
        for _ in range(60):
            world.step(render=True)
        world.pause()
        timeline = omni.timeline.get_timeline_interface()
        result["hidden_gprims"] = scene.hide_existing_geometry()
        result["camera"] = {
            "nominal_raw_sdk_k": asdict(raw_k),
            "nominal_integer_index_k": asdict(nominal_k),
            "actual_raw_sdk_k": np.asarray(camera.get_intrinsics_matrix()).tolist(),
            "clipping_range_m": list(map(float, camera.get_clipping_range())),
            "render_product": str(camera.get_render_product_path()),
            "world_from_base": scene.world_matrix(base_path).tolist(),
            "world_from_camera_usd": scene.world_matrix(camera_path).tolist(),
        }
        for visit_index, position in enumerate(result["protocol"]["positions"]):
            geometry = MEASURE.checkerboard_layout(
                result["protocol"]["distance_m"],
                position["centre_uv"],
                raw_k,
                nominal_matrix,
            )
            path = scene.board(geometry)
            truth_world = scene.vertices_world(path)[geometry["corner_indices"]]
            wc = scene.world_matrix(camera_path)
            truth_optical = MEASURE.transform_points(
                np.linalg.inv(wc @ np.diag([1, -1, -1, 1])), truth_world
            )
            nominal_uv = MEASURE.project(truth_optical, nominal_k)
            np.savez_compressed(
                args.output / f"board_p{position['position_index']}_geometry.npz",
                truth_world_m=truth_world,
                truth_optical_m=truth_optical,
                nominal_uv_px=nominal_uv,
                **geometry,
            )

            capture = make_capture(
                scene,
                {
                    "world_from_base": base_path,
                    "world_from_camera_usd": camera_path,
                    "world_from_board": path,
                },
                timeline,
                rep,
                annotators,
                result["protocol"]["step"],
                settings,
                camera,
            )
            board = collect_board(
                args.output, position["position_index"], capture, nominal_uv
            )
            board.update(
                visit_index=visit_index,
                centre_uv=position["centre_uv"],
                anchor_horizontal_m=result["protocol"]["distance_m"],
            )
            result["boards"].append(board)
            stage.RemovePrim(path)
            write_record(args.output / "diagnostic.json", result)
            write_curves(args.output, result["boards"])
        result["collection_complete"] = True
    finally:
        # camera remains alive until all attached annotators are released.
        write_record(args.output / "diagnostic.json", result)
        result["annotator_cleanup"] = MEASURE.detach_annotators(annotators)
        write_record(args.output / "diagnostic.json", result)


def build_parser() -> argparse.ArgumentParser:
    """One list of conditions, so the CLI cannot drift from condition_config."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--condition", choices=CONDITIONS, required=True)
    parser.add_argument("--base-scene", required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main() -> int:
    args, unknown = build_parser().parse_known_args()
    # Preserve Isaac's launcher flags, as the existing scripts do.
    sys.argv = [sys.argv[0], *unknown]
    result = {
        "purpose": "RGB drift observation; not a G1 result",
        "collection_complete": False,
        "input_provenance": "synthetic",
        "protocol": condition_config(args.condition),
        "pid": os.getpid(),
        "base_scene": args.base_scene,
        "boards": [],
        "coordinate_convention": "integer pixel index centres",
        "regression_columns": [
            "1",
            "reference_u - mean(reference_u)",
            "reference_v - mean(reference_v)",
        ],
        "regression_rows": ["mean_removed_du", "mean_removed_dv"],
        "depth_comparison": "same integer pixels, both finite and positive; target mask fixed at capture 0",
    }
    app, owns_output = None, False
    try:
        args.output.mkdir(parents=True, exist_ok=False)
        owns_output = True
        write_record(args.output / "diagnostic.json", result)
        from isaacsim import SimulationApp

        app = SimulationApp(result["protocol"]["simulation_app"])
        run(app, args, result)
    except Exception as exc:
        result["execution_error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc(file=sys.stderr)
    finally:
        if owns_output:
            write_record(args.output / "diagnostic.json", result)
        print(
            json.dumps(
                {
                    "condition": args.condition,
                    "collection_complete": result["collection_complete"],
                    "output": str(args.output),
                    "execution_error": result.get("execution_error"),
                }
            ),
            flush=True,
        )
        if app is not None:
            app.close()
    return 1 if "execution_error" in result else 0


if __name__ == "__main__":
    raise SystemExit(main())
