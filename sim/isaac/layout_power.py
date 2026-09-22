"""Calibration-layout detection-power diagnostic (2026-09-21 report, section 13.5).

This module RECORDS numbers. It emits no PASS/FAIL, no threshold and no gate,
and nothing here is an input to the G1 judgment. The judging layout stays
exactly as authored; section 13 rejected changing it, because tilting couples
RGB corner error into depth and moves the bias from a gate production does not
use into the gates that mimic production's depth back-projection.

The question is narrow: does an alternative board layout still detect a KNOWN
intrinsics error as well as the current one? Not "does it pass".

Method. For one layout arm, take the corners that were actually measured, add a
known principal-point perturbation to the truth projection (cx += delta and
cy += delta), refit with the unchanged estimator, and record

  * the recovered fraction of delta, defined as (after - before) / delta, which
    is a positive control on the wiring - never a deviation from nominal;
  * the deviation from nominal AFTER the injection, because gate 2a compares
    against nominal and an arm's own bias can cancel an injection of the
    opposite sign (a -0.06 px bias plus +0.05 px looks BETTER than before);
  * the dispersion of that deviation across the arm's fits, reported apart from
    the OLS standard errors - the two are different uncertainties and must not
    be added.

Limits, stated because the numbers do not state them:
  * The injection is applied AFTER corner measurement. It therefore cannot show
    how a real K change would propagate through rendering and detection.
  * A principal-point (or focal) injection enters the estimator linearly, so a
    full-rank layout recovers it exactly. The recovered fraction is a wiring
    check; it is not by itself evidence of detection power.
  * Repeats of one authored board are not independent observations, and the
    delta ladder is a fully paired contrast on the same corners, not new
    samples. Arm comparisons without replication are observations, not a
    statistical demonstration of equal detection power.

Rendering extra arms is a separate, opt-in step: sim/isaac/diagnose_layout_power.py
runs in its own Isaac process. This module needs neither Isaac nor OpenCV.
"""

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


MEASURE = load_module(
    "layout_power_camera_calibration", Path(__file__).with_name("camera_calibration.py")
)

# Predeclared before any layout is compared. Four magnitudes bracketing the
# 0.1 px gate 2a tolerance; they are injection sizes, never tolerances.
INJECTION_LADDER_PX = (0.05, 0.1, 0.2, 0.5)
INJECTED_PARAMETERS = ("cx", "cy")
# Sub-pixel board phases, one per screen placement. Eleven keeps every offset
# away from 0 and from exactly 0.5, where rounding to the nearest pixel is a
# tie. Every judging board that the authored rule does not tilt sits at one
# and the same phase, so that layout never samples any other phase.
PHASE_DENOMINATOR = 11
DIAGNOSTIC_TILT_DEG = MEASURE.FAR_CENTRAL_TILT_DEG

LAYOUT_VARIANTS = {
    "production": {
        "tilt": "authored_rule",
        "phase": "grid_aligned",
        "evidence": "measured",
        "note": (
            "The judging layout, reproduced exactly: 16.000 px squares whose "
            "checker boundaries sit at one single pixel phase on every board, "
            "tilted only at r=5.0, u=cx. This is the baseline arm and the "
            "layout section 13.5 decided to keep."
        ),
    },
    "tilted": {
        "tilt": "all_boards",
        "tilt_deg": DIAGNOSTIC_TILT_DEG,
        "phase": "grid_aligned",
        "evidence": "measured",
        "note": (
            "Every board rotated about the optical Y axis. Section 12.3 "
            "measured dv=+0.011 on the nine already-tilted boards against "
            "-0.061 elsewhere, so this arm has direct evidence behind it."
        ),
        "known_cost": (
            "Front-parallel boards make the depth gates structurally "
            "insensitive to RGB corner error (dZ/du=0). Section 13.1 measured "
            "545x bias and 997x p95 growth in gate 4 on the tilted boards. "
            "This arm is diagnostic only; it is not a proposal to adopt."
        ),
    },
    "phase_varied": {
        "tilt": "authored_rule",
        "phase": "per_position",
        "evidence": "untested",
        "note": (
            "Whole boards translated by a declared fraction of a pixel so the "
            "checker edges no longer land on the grid. This arm has no "
            "supporting evidence at all in the corpus: every untilted board "
            "measured so far sits at one fixed phase, so nothing is known "
            "about how the render behaves at any other phase."
        ),
    },
}


def phase_offset_px(position_index: int) -> tuple[float, float]:
    """Declared sub-pixel offset of one screen placement, u then v."""
    count = len(MEASURE.SCREEN_CENTRES_UV)
    index = int(position_index) % count
    return (
        (index + 1) / PHASE_DENOMINATOR,
        ((4 * index) % count + 1) / PHASE_DENOMINATOR,
    )


def variant_layout(
    distance_m: float,
    centre_uv,
    calibration,
    nominal_mount: np.ndarray,
    *,
    variant: str,
    position_index: int,
) -> dict:
    """Author one board of one arm; "production" reproduces the judging layout.

    Only the rotation and the sub-pixel placement change. Square size, corner
    count, screen centres and the mount are the authored ones, so an arm
    differs from the baseline in exactly the property being varied.
    """
    if variant not in LAYOUT_VARIANTS:
        raise ValueError(f"Unknown layout variant: {variant}")
    spec = LAYOUT_VARIANTS[variant]
    centre = (float(centre_uv[0]), float(centre_uv[1]))
    tilt_deg = MEASURE.default_tilt_deg(distance_m, centre, calibration)
    if spec["tilt"] == "all_boards":
        tilt_deg = spec["tilt_deg"]
    if spec["phase"] == "per_position":
        offset = phase_offset_px(position_index)
        centre = (centre[0] + offset[0], centre[1] + offset[1])
    return MEASURE.checkerboard_layout(
        distance_m, centre, calibration, nominal_mount, tilt_deg=tilt_deg
    )


def board_phase(uv: np.ndarray) -> dict:
    """Where one board's projected corners sit relative to the integer grid.

    A spread of zero with a nonzero offset is a whole-board translation; a
    nonzero spread means the boundary walks across pixel phases instead.
    """
    uv = np.asarray(uv, dtype=float)
    if uv.ndim != 2 or uv.shape[1] != 2 or not len(uv) or not np.isfinite(uv).all():
        raise ValueError("Board phase needs finite (N,2) pixel coordinates")
    offset = uv - np.round(uv)
    return {
        "corner_count": int(len(uv)),
        "mean_fractional_uv_px": (offset.mean(axis=0) % 1.0).tolist(),
        "grid_offset_spread_uv_px": np.ptp(offset, axis=0).tolist(),
        "max_abs_grid_offset_px": float(np.abs(offset).max()),
    }


def dispersion(values, *, label: str) -> dict:
    """Summarize one arm's values, refusing to invent an unestimable spread."""
    values = np.asarray(list(values), dtype=float)
    record = {
        "label": label,
        "count": int(len(values)),
        "mean": None,
        "sd": None,
        "sd_unavailable_reason": None,
        "max_abs": None,
        "p95_abs": None,
    }
    if not len(values):
        record["sd_unavailable_reason"] = "no fits"
        return record
    record["mean"] = float(values.mean())
    record["max_abs"] = float(np.abs(values).max())
    record["p95_abs"] = float(np.percentile(np.abs(values), 95))
    if len(values) < 2:
        # A single fit has no sample spread. Zero would claim noise-free.
        record["sd_unavailable_reason"] = "fewer than two fits"
        return record
    record["sd"] = float(values.std(ddof=1))
    return record


def _effect_size(delta: float, spread: dict) -> dict:
    """delta / sd, with the reason recorded whenever it is not computable.

    This is a standardized effect size under the arm's observed spread. It is
    not a detection probability and carries no threshold.
    """
    if spread["sd"] is None:
        return {"value": None, "unavailable_reason": spread["sd_unavailable_reason"]}
    if spread["sd"] == 0.0:
        return {
            "value": None,
            "unavailable_reason": (
                "observed spread is exactly zero; this is not proof of a "
                "noise-free layout"
            ),
        }
    return {"value": float(delta / spread["sd"]), "unavailable_reason": None}


def _fit_one(observation: dict, nominal, *, offsets=(0.0, 0.0)) -> dict:
    """Fit a copy of one observation, optionally with an injected offset.

    The offset is added to a copy. Nothing the judging path holds is mutated.
    """
    uv = np.asarray(observation["uv"], dtype=float) + np.asarray(offsets, dtype=float)
    return MEASURE.fit_intrinsics(
        np.asarray(observation["points_optical"], dtype=float),
        uv,
        np.asarray(observation["holdout"], dtype=bool),
        nominal,
    )


def arm_power(
    observations: list,
    nominal,
    *,
    layout: str,
    deltas=INJECTION_LADDER_PX,
    provenance: dict | None = None,
) -> dict:
    """Injected-error record for one layout arm. Pure computation, no render."""
    spec = LAYOUT_VARIANTS.get(layout, {})
    fits, baseline = [], []
    for observation in observations:
        fit = _fit_one(observation, nominal)
        baseline.append(fit)
        fits.append(
            {
                "anchor_horizontal_m": observation.get("anchor_horizontal_m"),
                "repeat": observation.get("repeat"),
                "position_count": observation.get("position_count"),
                "corner_count": int(len(np.asarray(observation["uv"]))),
                "cx_minus_nominal_px": fit["estimated"]["cx"] - nominal.cx,
                "cy_minus_nominal_px": fit["estimated"]["cy"] - nominal.cy,
                "fx_relative_error": fit["estimated"]["fx"] / nominal.fx - 1.0,
                "fy_relative_error": fit["estimated"]["fy"] / nominal.fy - 1.0,
                "ols_standard_errors_px": dict(fit["standard_errors_px"]),
                "heldout_p95_px": fit["heldout_p95_px"],
            }
        )
    deviation = {
        axis: dispersion(
            (f[f"{axis}_minus_nominal_px"] for f in fits),
            label=f"{axis} minus nominal, across fits",
        )
        for axis in INJECTED_PARAMETERS
    }
    record = {
        "layout": layout,
        "evidence": spec.get("evidence"),
        "note": spec.get("note"),
        "known_cost": spec.get("known_cost"),
        "provenance": provenance or {},
        "fit_count": len(fits),
        "fit_structure": {
            "anchors_horizontal_m": sorted(
                {
                    f["anchor_horizontal_m"]
                    for f in fits
                    if f["anchor_horizontal_m"] is not None
                }
            ),
            "repeats": sorted({f["repeat"] for f in fits if f["repeat"] is not None}),
            "independence_caveat": (
                "Repeats of one authored board are not independent, and every "
                "fit at one anchor shares the same nine authored boards. These "
                "fits are not N independent replications of the layout."
            ),
        },
        "completeness": {
            "expected_positions_per_fit": len(MEASURE.SCREEN_CENTRES_UV),
            "fits_below_expected_positions": sum(
                1
                for f in fits
                if f["position_count"] is not None
                and f["position_count"] < len(MEASURE.SCREEN_CENTRES_UV)
            ),
            "corner_count_min": min((f["corner_count"] for f in fits), default=None),
            "corner_count_max": max((f["corner_count"] for f in fits), default=None),
            "selection_bias_note": (
                "Dropping boards whose corners were not detected shrinks the "
                "observed spread. Missing placements are counted here so an "
                "arm cannot look quieter by measuring less."
            ),
        },
        "fits": fits,
        "baseline_deviation_from_nominal_px": deviation,
        "baseline_focal_relative_error": {
            axis: dispersion(
                (f[f"{axis}_relative_error"] for f in fits),
                label=f"{axis} relative error, across fits",
            )
            for axis in ("fx", "fy")
        },
        "mean_ols_standard_error_px": {
            axis: float(np.mean([f["ols_standard_errors_px"][axis] for f in fits]))
            if fits
            else None
            for axis in ("fx", "fy", "cx", "cy")
        },
        "ols_standard_error_note": (
            "Conditional uncertainty of one fit given its own residuals. It is "
            "reported beside, never added to, the across-fit spread, and a "
            "common DC bias does not appear in it at all."
        ),
        "heldout_p95_px": dispersion(
            (f["heldout_p95_px"] for f in fits), label="held-out p95, across fits"
        ),
        "injections": [],
    }
    for delta in deltas:
        record["injections"].append(
            _injection_record(observations, baseline, fits, nominal, deviation, delta)
        )
    return record


def _injection_record(observations, baseline, fits, nominal, deviation, delta) -> dict:
    """One rung of the ladder: refit every observation with cx,cy += delta."""
    injected = [
        _fit_one(observation, nominal, offsets=(delta, delta))
        for observation in observations
    ]
    entry = {
        "delta_px": float(delta),
        "applied_to": list(INJECTED_PARAMETERS),
        "recovery": {},
        "post_injection_deviation_from_nominal_px": {},
        "masking": {},
        "standardized_effect_size": {},
        "heldout_p95_max_abs_shift_px": max(
            (
                abs(after["heldout_p95_px"] - before["heldout_p95_px"])
                for before, after in zip(baseline, injected, strict=True)
            ),
            default=None,
        ),
        "focal_max_abs_relative_shift": max(
            (
                abs(after["estimated"][axis] / before["estimated"][axis] - 1.0)
                for before, after in zip(baseline, injected, strict=True)
                for axis in ("fx", "fy")
            ),
            default=None,
        ),
    }
    for axis in INJECTED_PARAMETERS:
        recovered = np.array(
            [
                after["estimated"][axis] - before["estimated"][axis]
                for before, after in zip(baseline, injected, strict=True)
            ]
        )
        after_nominal = np.array(
            [after["estimated"][axis] - getattr(nominal, axis) for after in injected]
        )
        before_nominal = np.array([f[f"{axis}_minus_nominal_px"] for f in fits])
        spread = dispersion(
            after_nominal, label=f"{axis} minus nominal after injection, across fits"
        )
        entry["recovery"][axis] = {
            "definition": "(estimate after injection - estimate before) / delta",
            "mean_fraction": float(recovered.mean() / delta)
            if len(recovered)
            else None,
            "max_abs_fraction_error": float(np.abs(recovered / delta - 1.0).max())
            if len(recovered)
            else None,
        }
        entry["post_injection_deviation_from_nominal_px"][axis] = spread
        masked = int((np.abs(after_nominal) < np.abs(before_nominal)).sum())
        entry["masking"][axis] = {
            "count": masked,
            "fraction": float(masked / len(fits)) if fits else None,
            "meaning": (
                "Fits whose distance from nominal SHRANK although a real error "
                "was injected: the arm's own bias absorbed it."
            ),
        }
        entry["standardized_effect_size"][axis] = _effect_size(
            delta, deviation[axis]
        ) | {
            "definition": "delta / across-fit sd of the arm's deviation from nominal",
            "not_a_detection_probability": True,
        }
    return entry


def compare_layout_power(
    arms: dict, nominal, *, deltas=INJECTION_LADDER_PX, reference: str = "production"
) -> dict:
    """Put the arms side by side. No arm is judged; nothing here is a gate."""
    records = {
        layout: arm_power(
            payload["observations"],
            nominal,
            layout=layout,
            deltas=deltas,
            provenance=payload.get("provenance"),
        )
        for layout, payload in arms.items()
    }
    by_delta = []
    for index, delta in enumerate(deltas):
        row = {"delta_px": float(delta)}
        for axis in INJECTED_PARAMETERS:
            reference_effect = (
                records[reference]["injections"][index]
                .get("standardized_effect_size", {})
                .get(axis, {})
                .get("value")
                if reference in records
                else None
            )
            row[axis] = {
                "standardized_effect_size": {
                    layout: record["injections"][index]["standardized_effect_size"][
                        axis
                    ]["value"]
                    for layout, record in records.items()
                },
                "relative_to_reference": {
                    layout: (
                        record["injections"][index]["standardized_effect_size"][axis][
                            "value"
                        ]
                        / reference_effect
                        if reference_effect
                        and record["injections"][index]["standardized_effect_size"][
                            axis
                        ]["value"]
                        is not None
                        else None
                    )
                    for layout, record in records.items()
                },
                "post_injection_max_abs_px": {
                    layout: record["injections"][index][
                        "post_injection_deviation_from_nominal_px"
                    ][axis]["max_abs"]
                    for layout, record in records.items()
                },
                "masked_fraction": {
                    layout: record["injections"][index]["masking"][axis]["fraction"]
                    for layout, record in records.items()
                },
            }
        by_delta.append(row)
    return {
        "question": (
            "Does an alternative layout still detect a known intrinsics error "
            "as well as the current one?"
        ),
        "judgment": (
            "none - this diagnostic records numbers. No PASS/FAIL, no "
            "threshold, no gate, and no input to the G1 result."
        ),
        "nominal_intrinsics": {
            "fx": nominal.fx,
            "fy": nominal.fy,
            "cx": nominal.cx,
            "cy": nominal.cy,
            "coordinate_convention": "integer_index_centers",
        },
        "injection": {
            "applied_to": list(INJECTED_PARAMETERS),
            "ladder_px": [float(d) for d in deltas],
            "stage": (
                "after corner measurement, on a copy of the stored corners; it "
                "cannot show how a real K change propagates through rendering "
                "and detection"
            ),
            "paired": (
                "every rung reuses the same corners, so the ladder is a paired "
                "contrast, not four independent samples"
            ),
            "linearity": (
                "a principal-point injection enters the estimator linearly, so "
                "any full-rank layout recovers it exactly; the recovered "
                "fraction is a wiring control, not evidence of detection power"
            ),
        },
        "reference_layout": reference,
        "arms": records,
        "by_delta": by_delta,
        "caveats": [
            "Arms captured in different processes or runs differ by run "
            "conditions as well as by layout; section 12.5 found a mod-4 "
            "authoring modulation present in one run and absent in another.",
            "Without replication this is an observation, not a statistical "
            "demonstration that two layouts have equal detection power.",
            "The judging layout is unchanged and stays the judging baseline "
            "(section 13.5). Nothing here proposes adopting an arm.",
        ],
    }


def observations_from_samples(directory: Path, *, measured_only: bool = True) -> dict:
    """Group stored per-board corner NPZs into one observation set per fit.

    Files are named board_r<anchor>_p<position>_<suffix>_samples.npz, exactly as
    the G1 run and the layout capture script write them. Warm-up captures are
    excluded by default for the same reason the judging run excludes them.
    """
    import re

    pattern = re.compile(
        r"^board_r(?P<anchor>[0-9.]+)_p(?P<position>\d+)_(?P<suffix>warmup|repeat\d+)_samples\.npz$"
    )
    groups, skipped = {}, {"warmup": 0, "unparsed": 0}
    for path in sorted(Path(directory).glob("*_samples.npz")):
        match = pattern.match(path.name)
        if match is None:
            skipped["unparsed"] += 1
            continue
        if match["suffix"] == "warmup":
            skipped["warmup"] += 1
            if measured_only:
                continue
            repeat = None
        else:
            repeat = int(match["suffix"].removeprefix("repeat"))
        key = (float(match["anchor"]), repeat)
        with np.load(path) as arrays:
            groups.setdefault(key, []).append(
                (
                    int(match["position"]),
                    np.asarray(arrays["truth_optical_m"], dtype=float),
                    np.asarray(arrays["uv"], dtype=float),
                    np.asarray(arrays["holdout"], dtype=bool),
                )
            )
    observations = []
    for (anchor, repeat), entries in sorted(
        groups.items(), key=lambda item: (item[0][0], item[0][1] is None, item[0][1])
    ):
        entries.sort()
        observations.append(
            {
                "anchor_horizontal_m": anchor,
                "repeat": repeat,
                "position_count": len(entries),
                "points_optical": np.concatenate([e[1] for e in entries]),
                "uv": np.concatenate([e[2] for e in entries]),
                "holdout": np.concatenate([e[3] for e in entries]),
            }
        )
    return {
        "observations": observations,
        "provenance": {
            "directory": str(directory),
            "measured_only": measured_only,
            "skipped": skipped,
            "file_count": sum(len(entries) for entries in groups.values()),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--arm",
        action="append",
        default=[],
        metavar="LAYOUT=DIRECTORY",
        help="Directory of *_samples.npz for one layout arm; repeat per arm",
    )
    parser.add_argument("--reference", default="production")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rig = load_module("layout_power_rig", ROOT / "tools/scene_rig.py")
    adapter = load_module(
        "layout_power_adapter", ROOT / "sim/isaac/perception_adapter.py"
    )
    nominal = adapter.normalize_isaac_intrinsics(rig.intrinsics()).integer_index
    arms = {}
    for entry in args.arm:
        layout, _, directory = entry.partition("=")
        if not directory:
            raise SystemExit(f"--arm needs LAYOUT=DIRECTORY, got {entry!r}")
        arms[layout] = observations_from_samples(Path(directory))
    if not arms:
        raise SystemExit("No arms given; nothing to compare")
    record = compare_layout_power(arms, nominal, reference=args.reference)
    args.output.write_text(
        json.dumps(record, indent=2, allow_nan=False, default=_native) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output": str(args.output), "arms": sorted(arms)}), flush=True)
    return 0


def _native(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Unsupported record type: {type(value).__name__}")


if __name__ == "__main__":
    raise SystemExit(main())
