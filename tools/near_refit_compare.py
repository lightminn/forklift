"""Registered comparison of the near-capture plane refit arms (docs/plans/2026-11-06-near-capture-plane-refit.md).

    python -m tools.near_refit_compare manifest --root test_runs --e1-run R --e1-mission m2 --e1-attempt 1 --out manifest.json
    python -m tools.near_refit_compare replay --manifest manifest.json --root test_runs --arms A0 --out gate.json
    python -m tools.near_refit_compare gate --manifest manifest.json --replay gate.json   # before any arm is run
    python -m tools.near_refit_compare replay --manifest manifest.json --root test_runs --out replay_a.json
    python -m tools.near_refit_compare seeds --manifest manifest.json --root test_runs --replay replay_a.json --out seeds.json
    python -m tools.near_refit_compare negatives --dataset <dir> --config config --out e3.json
    python -m tools.near_refit_compare toy --data <d435i_noise/date> --out e4.json
    python -m tools.near_refit_compare judge --replay replay_a.json --again replay_b.json --seeds seeds.json \\
        --negatives e3.json --toy e4.json --out judge.json

The saved depth of every observation attempt is replayed through the path the run took: the first call restored from
the attempt record, the lower waiver's conditions restored from the records, and whether the first call succeeded and
whether the waiver's second call ran computed again per arm (plan section 3). The judge only applies section 4's rules
to these outputs; its thresholds are the constants below, registered before any result was read.
"""

from __future__ import annotations

import argparse
import dataclasses
import glob
import hashlib
import json
import math
import re
import statistics
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np

from forklift_core.geometry import RigidTransform
from forklift_core.perception import pocket_detector as det
from forklift_core.perception.pallet_prior import PalletPrior, load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, ExpectedSite
from forklift_core.perception.recognition_dataset import load_recognition_sample, read_meta
from forklift_core.perception.scene_dataset import SceneInput
from forklift_core.sensors.rgbd import PinholeIntrinsics
from tools import diagnose_detection as DD

ARMS = {"A0": (0, 200), "A1": (5, 200), "A2": (0, 400), "A3": (5, 400)}  # (plane_refit_iterations, ransac_iterations)
CANDIDATES = ("A1", "A2", "A3")
NEAR_WAIVER_RADIUS_M = 0.15
NEAR_WAIVER_AXIS_RAD = math.radians(15.0)

# Section 4, registered before any result
E1_POSITION_M, E1_YAW_RAD = 0.005, 0.002
P95_INCREASE_M, P95_INCREASE_RAD = 0.001, 0.001
POINT_WORSENING_M = 0.005
NEW_VALID_LATERAL_M, NEW_VALID_ALONG_M, NEW_VALID_YAW_RAD = 0.010, 0.008, 0.010
LARGE_ERROR_M = 0.03
MATCH_M = 0.30
TIME_P95_RATIO, TIME_MAX_RATIO = 1.5, 2.0
WARMUP_ATTEMPTS = 10
MIN_STRATUM = 20
SEED_EXTRA_FAILURES = 5
E1_SEEDS = tuple(20260913 + i for i in range(200))
FRAME_SEEDS = tuple(range(100))
UNIFORM_FRAMES, RESIDUAL_FRAMES = 10, 10
TOY_STD_WORSENING_M = 0.002
# Only the checks the plan says report, never judge
DETECTION_DELTA_M, DETECTION_DELTA_RAD = 0.002, 0.002
SUPPORT_LOSS_REPORT = 0.5

POINTS = ("centre_left", "centre_right", "along_left", "along_right", "wall_left_outer", "wall_left_inner",
          "wall_right_outer", "wall_right_inner")
LATERAL = ("centre_left", "centre_right", "wall_left_outer", "wall_left_inner", "wall_right_outer", "wall_right_inner")
ALONG = ("along_left", "along_right")
WALLS = LATERAL[2:]


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# --- what the run did, from its records ---------------------------------------------------------------------------

def far_waiver_refusal(state: dict) -> str | None:
    """The mission core's condition 1 (MissionCore._far_waiver_refusal) from the records: the refusal or None."""
    near = state.get("near_capture") or {}
    number = near.get("far_attempt")
    attempts = state.get("observation_attempts") or []
    if not isinstance(number, int) or not 1 <= number <= len(attempts):
        return "far_attempt_missing"
    far = attempts[number - 1]
    detector = far.get("detector_reference", far.get("pocket_observation")) or {}
    if detector.get("status") != "valid":
        return f"far_detector:{detector.get('status')}"
    if far.get("lower_evidence", True) is not True:
        return "far_lower_waived"
    if far.get("expected") is not None or far.get("far_expected") is not None:
        return "far_restricted"
    site = far.get("detector_site")
    if site is None:
        return "far_detector_site_missing"
    estimate = near.get("far_estimate_m")
    if estimate is None:
        return "far_estimate_missing"
    link_m = math.hypot(float(site[0]) - float(estimate["x_m"]), float(site[1]) - float(estimate["y_m"]))
    link_rad = abs(wrap(float(site[2]) - float(estimate["yaw_rad"])))
    if link_m > NEAR_WAIVER_RADIUS_M or link_rad > NEAR_WAIVER_AXIS_RAD:
        return "far_link"
    return None


def replay_path(attempt: dict, state: dict, arguments: dict) -> dict:
    """How the attempt's detection was called: kind, first-call site and lower evidence, and whether a failed first
    call could go on to the lower waiver (a refusal the records prove, or allowed)."""
    waiver_record = attempt.get("lower_waiver")
    expected = attempt.get("expected")
    if expected is not None:
        first_call = expected
        lower = attempt.get("lower_evidence", True)
        if waiver_record is not None and waiver_record.get("applied"):
            first_call = waiver_record["first"]["expected"]
            lower = True  # attempt["lower_evidence"] False was written by the waiver itself
        path = {"kind": "near_expected", "site": [float(v) for v in first_call["site"]],
                "radius_m": float(first_call["radius_m"]), "axis_rad": float(first_call["axis_tolerance_rad"]),
                "lower_evidence": bool(lower), "waiver": "off"}
        if lower and arguments.get("near_capture_lower", "gate") == "waive_far_confirmed":
            why = None if waiver_record is None else waiver_record.get("why", "")
            if why is not None:  # the run judged it: a far refusal, or the far condition held and the second call ran
                path["waiver"] = f"refused:{why}" if why.startswith("far") else "allowed"
            elif "near_capture" not in state:  # a live run's last mission: no state record and no waiver reached
                path["waiver"] = "unknown"
            else:
                refusal = far_waiver_refusal(state)
                path["waiver"] = "allowed" if refusal is None else f"refused:{refusal}"
        return path
    far = attempt.get("far_expected")
    if far is not None:
        return {"kind": "far_expected", "site": [float(v) for v in far["site"]], "radius_m": float(far["radius_m"]),
                "axis_rad": float(far["axis_tolerance_rad"]), "lower_evidence": bool(attempt.get("lower_evidence", True)),
                "waiver": "off", "outside_zone_recorded": far.get("outside_zone") is not None}
    return {"kind": "plain", "lower_evidence": bool(attempt.get("lower_evidence", True)), "waiver": "off"}


def attempt_role(index: int, attempt: dict, state: dict, arguments: dict) -> str:
    """first_near / second / reobserve for the near expected-site branch; far, or near without a site in old runs."""
    attempts = state.get("observation_attempts") or []
    if attempt.get("expected") is not None:
        earlier = sum(1 for a in attempts[:index] if a.get("expected") is not None)
        if earlier == 0:
            return "first_near"
        return "second" if arguments.get("near_recapture_gap_m") is not None else "reobserve"
    far_attempt = (state.get("near_capture") or {}).get("far_attempt")
    if isinstance(far_attempt, int) and index + 1 > far_attempt:
        return "near_no_site"
    return "far"


# --- truth ----------------------------------------------------------------------------------------------------------

def yaw_of(q_wxyz) -> float:
    w, x, y, z = (float(v) for v in q_wxyz)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def truth_in_base(pallet_xy, pallet_yaw, accepted_pose, geometry: dict) -> dict:
    """The target pallet in the capture's true base frame: the entry face (the insertion axis pointing away from the
    camera), its pockets' centre and wall offsets across the face, and the pallet centre."""
    (bx, by, _), q = accepted_pose
    base_yaw = yaw_of(q)
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    dx, dy = float(pallet_xy[0]) - float(bx), float(pallet_xy[1]) - float(by)
    centre = (c * dx + s * dy, -s * dx + c * dy)
    axis = wrap(float(pallet_yaw) - base_yaw)
    if abs(axis) > math.pi / 2:
        axis = wrap(axis + math.pi)
    depth, width = float(geometry["overall_depth_m"]), float(geometry["overall_width_m"])
    blocks = geometry["block_widths_m"]
    outer, inner = width / 2 - float(blocks[0]), float(blocks[1]) / 2
    return {"centre": list(centre), "yaw": axis, "depth_m": depth, "outer_m": outer, "inner_m": inner}


def pose_errors(record: dict, truth: dict) -> dict:
    """Absolute errors against the entry face of ``truth``; ``record`` holds a valid observation's left, right (xy in
    base), widths and yaw. Lateral along the true face, along its normal, walls as near_tracking.carried_face."""
    yaw = truth["yaw"]
    d, l = np.array((math.cos(yaw), math.sin(yaw))), np.array((-math.sin(yaw), math.cos(yaw)))
    face = np.asarray(truth["centre"]) - truth["depth_m"] / 2 * d
    middle = (truth["outer_m"] + truth["inner_m"]) / 2
    left, right = np.asarray(record["left"]), np.asarray(record["right"])
    psi = record["yaw"]
    d_seen, l_seen = np.array((math.cos(psi), math.sin(psi))), np.array((-math.sin(psi), math.cos(psi)))
    w_left, w_right = record["widths"]
    seen = {"wall_left_outer": left + w_left / 2 * l_seen, "wall_left_inner": left - w_left / 2 * l_seen,
            "wall_right_outer": right - w_right / 2 * l_seen, "wall_right_inner": right + w_right / 2 * l_seen}
    true = {"wall_left_outer": face + truth["outer_m"] * l, "wall_left_inner": face + truth["inner_m"] * l,
            "wall_right_outer": face - truth["outer_m"] * l, "wall_right_inner": face - truth["inner_m"] * l}
    out = {name: float(abs((seen[name] - true[name]) @ l)) for name in WALLS}
    true_left, true_right = face + middle * l, face - middle * l
    out.update(centre_left=float(abs((left - true_left) @ l)), centre_right=float(abs((right - true_right) @ l)),
               along_left=float(abs((left - true_left) @ d)), along_right=float(abs((right - true_right) @ d)),
               yaw=float(abs(wrap(psi - yaw))))
    centre_seen = (left + right) / 2 + truth["depth_m"] / 2 * d_seen
    out["match_m"] = float(np.linalg.norm(centre_seen - np.asarray(truth["centre"])))
    return out


def quality_pass(errors: dict) -> bool:
    """The one quality function (plan section 3): new valids and the seed check both use it."""
    return (errors["match_m"] <= MATCH_M
            and all(errors[name] <= NEW_VALID_LATERAL_M for name in LATERAL)
            and all(errors[name] <= NEW_VALID_ALONG_M for name in ALONG)
            and errors["yaw"] <= NEW_VALID_YAW_RAD)


# --- one detection through the run's path -----------------------------------------------------------------------------

def arm_call(params: DetectorParams, arm: str) -> tuple[DetectorParams, dict]:
    refit, ransac = ARMS[arm]
    return dataclasses.replace(params, ransac_iterations=ransac), ({"plane_refit_iterations": refit} if refit else {})


def detect_path(scene, prior, params, path: dict, arm: str, *, log=None, detector=det.detect_pockets_expected):
    """The attempt's detection under one arm: (DetectionResult, whether the waiver's second call ran). Mirrors the
    mission core: the second call runs only in the near branch after an invalid first call with the waiver allowed;
    it stands only if valid."""
    params_a, kw = arm_call(params, arm)
    if log is not None:
        kw = {**kw, "refit_log": log}
    site = None
    if path["kind"] != "plain":
        site = ExpectedSite(*path["site"], path["radius_m"], path["axis_rad"])
    first, _ = detector(scene, prior, params_a, site, lower_evidence=path["lower_evidence"], **kw)
    if path["waiver"] != "allowed" or first.observation.status == "valid":
        return first, False
    narrow = ExpectedSite(site.x_m, site.y_m, site.yaw_rad, NEAR_WAIVER_RADIUS_M, NEAR_WAIVER_AXIS_RAD)
    second, _ = detector(scene, prior, params_a, narrow, lower_evidence=False, **kw)
    return (second, True) if second.observation.status == "valid" else (first, True)


def canonical(observation) -> str:
    return json.dumps(dataclasses.asdict(observation), sort_keys=True, default=list)


def detection_record(detection, second_called: bool, wall_s: float | None = None, log=None) -> dict:
    obs, diag = detection.observation, detection.diagnostics
    record = {"status": obs.status, "reason": obs.reason, "second_called": second_called,
              "canonical_sha256": hashlib.sha256(canonical(obs).encode()).hexdigest(),
              "residual_p95_m": diag.plane_residual_p95_m, "plane_inlier_count": diag.plane_inlier_count}
    if obs.status == "valid":
        record.update(left=[float(v) for v in obs.left.center_m[:2]], right=[float(v) for v in obs.right.center_m[:2]],
                      widths=[float(obs.left.width_m), float(obs.right.width_m)], yaw=float(obs.insertion_yaw_rad))
    if wall_s is not None:
        record["wall_s"] = wall_s
    if log is not None:
        record["refit_rounds"] = [{k: r.get(k) for k in ("stop", "iterations", "support_loss", "inflow", "sizes",
                                                         "normal_change_deg", "offset_change_m",
                                                         "removed_from_remaining")} for r in log]
    return record


# --- manifest ---------------------------------------------------------------------------------------------------------

def run_attempts(run: Path, result: dict):
    """(mission, index, attempt, state) of every observation attempt: a live run's per mission, a batch run's own. A
    live run's result keeps the attempts of its last mission, whose own records may never have been written (the run
    ended inside it): those come with mission "last", failed captures among them."""
    missions = sorted(run.glob("missions/m*/records.json"))
    seen = set()
    for records in missions:
        try:
            state = json.loads(records.read_text())["state"]
        except (OSError, ValueError, KeyError):
            continue
        for index, attempt in enumerate(state.get("observation_attempts") or []):
            seen.add(attempt.get("depth_sha256"))
            yield records.parent.name, index, attempt, state
    for index, attempt in enumerate(result.get("observation_attempts") or []):
        if not missions:
            yield None, index, attempt, result
        elif attempt.get("depth_sha256") not in seen:
            yield "last", index, attempt, result


def run_samples(run: Path, result: dict) -> list:
    samples = list(result.get("samples") or [])
    if samples:
        return samples
    for name in sorted(glob.glob(str(run / "spool/state.samples/*.jsonl"))):
        samples += [json.loads(line) for line in open(name)]
    return samples


def capture_time(attempt: dict) -> float | None:
    control = attempt.get("capture_control")
    if control is not None:
        return float(control["time_s"])
    t = attempt.get("t_before_capture_s")
    return None if t is None else float(t)


def pallet_at(samples: list, t: float | None):
    """The pallet in the last sample at or before ``t``."""
    if t is None:
        return None
    best = None
    for sample in samples:
        if sample.get("pallet_position_m") is not None and float(sample["time_s"]) <= t:
            best = sample
    return best


def build_manifest(root: Path, e1: dict) -> dict:
    files = sorted(root.glob("**/perception_capture_*_depth_m.npy"))
    entries, excluded, runs = [], Counter(), {}
    for path in files:
        run = path.parent
        if run not in runs:
            try:
                result = json.loads((run / "result.json").read_text())
            except (OSError, ValueError):
                runs[run] = None
            else:
                by_sha = {}
                for mission, index, attempt, state in run_attempts(run, result):
                    if attempt.get("depth_sha256"):
                        by_sha.setdefault(attempt["depth_sha256"], []).append((mission, index, attempt, state))
                runs[run] = (result, by_sha, run_samples(run, result))
        if runs[run] is None:
            excluded["no_result"] += 1
            continue
        result, by_sha, samples = runs[run]
        depth = np.load(path)
        sha = DD.depth_array_sha256(depth)
        matches = by_sha.get(sha)
        if not matches:
            excluded["no_attempt_match"] += 1
            continue
        arguments = result.get("arguments") or {}
        if not arguments.get("pallet_prior_loaded") or not arguments.get("pallet_geometry_loaded"):
            excluded["no_prior_or_geometry"] += 1
            continue
        for mission, index, attempt, state in matches:
            params = state.get("detector_params") or result.get("detector_params")
            if not params:
                excluded["no_params"] += 1
                continue
            sample = pallet_at(samples, capture_time(attempt))
            truth = None
            pose = (attempt.get("capture_diagnostics") or {}).get("accepted_pose")
            if sample is not None and pose is not None:
                truth = truth_in_base(sample["pallet_position_m"][:2], sample["pallet_yaw_rad"], pose,
                                      arguments["pallet_geometry_loaded"])
            is_e1 = (e1 and str(run.relative_to(root)) == e1["run"] and mission == e1["mission"]
                     and index == e1["attempt"])
            recorded = DD.recorded_observation(attempt)
            entries.append({"id": f"{run.relative_to(root)}#{mission}#{index}", "file": str(path.relative_to(root)),
                            "depth_sha256": sha, "run": str(run.relative_to(root)), "mission": mission, "attempt": index,
                            "role": attempt_role(index, attempt, state, arguments), "path": replay_path(attempt, state, arguments),
                            "truth": truth, "target_position_m": None if sample is None else sample["pallet_position_m"][:2],
                            "shape": arguments["pallet_prior_loaded"]["catalogue_version"],
                            "proposals": arguments.get("recognition_proposals"), "recognition": arguments.get("recognition"),
                            "recorded_status": recorded.get("status"), "recorded_reason": recorded.get("reason"),
                            "recorded_left": (recorded.get("left") or {}).get("center_m"), "params": params,
                            "e1": bool(is_e1)})
    return {"entries": entries, "files": len(files), "excluded": dict(excluded)}


def load_frame(root: Path, entry: dict, cache: dict):
    """(scene, prior, params) of a manifest entry; the attempt record comes back through its identifiers."""
    run = root / entry["run"]
    if run not in cache:
        cache.clear()  # one run's records at a time: the sweep walks the manifest in run order
        cache[run] = json.loads((run / "result.json").read_text())
    result = cache[run]
    attempts = {(m, i): a for m, i, a, _ in run_attempts(run, result)} if "_attempts" not in cache else cache["_attempts"]
    cache["_attempts"] = attempts
    attempt = attempts[(entry["mission"], entry["attempt"])]
    depth = np.load(root / entry["file"])
    scene = DD.attempt_scene(result, attempt, depth)
    return scene, PalletPrior(**result["arguments"]["pallet_prior_loaded"]), DetectorParams(**entry["params"])


def classify_mismatch(entry: dict, replay: dict) -> str | None:
    """Why a same-code A0 replay differs from the record; None when it does not. An unexplained one stops the freeze."""
    same = entry["recorded_status"] == replay["status"] and entry["recorded_reason"] == replay["reason"]
    if same and (replay["status"] != "valid" or entry["recorded_left"] is None
                 or np.allclose(entry["recorded_left"][:2], replay["left"], atol=1e-6)):
        return None
    if entry["path"].get("outside_zone_recorded"):
        return "far_outside_zone_not_replayed"
    return "unexplained"


# --- sweeps -----------------------------------------------------------------------------------------------------------

def sweep(manifest: dict, root: Path, names=tuple(ARMS)) -> dict:
    """Every entry under every arm, the order rotating per entry so no arm always runs first (plan 4.4). The freeze
    gate runs ``names`` = A0 alone, so the manifest is checked against the records before any arm is seen."""
    out, cache = [], {}
    names = list(names)
    for number, entry in enumerate(manifest["entries"]):
        scene, prior, params = load_frame(root, entry, cache)
        arms = {}
        for k in range(len(names)):
            arm = names[(number + k) % len(names)]
            log = [] if ARMS[arm][0] else None  # K = 0 stays on the frozen code path, timed as it runs
            start = time.perf_counter()
            detection, second = detect_path(scene, prior, params, entry["path"], arm, log=log)
            wall = time.perf_counter() - start
            arms[arm] = detection_record(detection, second, wall, log)
        out.append({"id": entry["id"], "arms": arms})
    return {"frames": out}


def seed_sample(entries: list, a0: dict) -> list:
    """Plan 4.5: near-branch frames with duplicate depth removed and E1 out: sha-uniform, then the highest A0 residual."""
    unique = {}
    e1_sha = {e["depth_sha256"] for e in entries if e["e1"]}
    for entry in sorted(entries, key=lambda e: e["id"]):
        if entry["path"]["kind"] == "near_expected" and entry["depth_sha256"] not in e1_sha:
            unique.setdefault(entry["depth_sha256"], entry)
    ordered = [unique[sha] for sha in sorted(unique)]
    n = len(ordered)
    chosen = [ordered[(i * n) // UNIFORM_FRAMES] for i in range(UNIFORM_FRAMES)] if n >= UNIFORM_FRAMES else list(ordered)
    taken = {e["id"] for e in chosen}
    ranked = sorted((e for e in ordered if e["id"] not in taken),
                    key=lambda e: (-(a0[e["id"]].get("residual_p95_m") or -1.0), e["depth_sha256"]))
    return chosen + ranked[:RESIDUAL_FRAMES]


def seed_failures(scene, prior, params, entry: dict, seeds, arm: str) -> dict:
    failed = []
    for seed in seeds:
        detection, second = detect_path(scene, prior, dataclasses.replace(params, seed=seed), entry["path"], arm)
        record = detection_record(detection, second)
        if record["status"] != "valid" or not quality_pass(pose_errors(record, entry["truth"])):
            failed.append(seed)
    return {"seeds": len(seeds), "failures": len(failed), "failed": failed}


def seed_sweep(manifest: dict, replay: dict, root: Path) -> dict:
    a0 = {f["id"]: f["arms"]["A0"] for f in replay["frames"]}
    entries = [e for e in manifest["entries"] if e["truth"] is not None]
    sample = seed_sample(entries, a0)
    out = {"e1": {}, "frames": {}}
    e1 = next(e for e in entries if e["e1"])
    cache = {}
    scene, prior, params = load_frame(Path(root), e1, cache)
    for arm in ARMS:
        out["e1"][arm] = seed_failures(scene, prior, params, e1, E1_SEEDS, arm)
    for entry in sample:
        scene, prior, params = load_frame(Path(root), entry, cache)
        out["frames"][entry["id"]] = {arm: seed_failures(scene, prior, params, entry, FRAME_SEEDS, arm) for arm in ARMS}
    return out


def negative_sweep(dataset: Path, config: Path) -> dict:
    """E3: dev-split negatives, every noise level, both priors, both lower-evidence modes. A scene that cannot be read
    is an error here: the judge turns any error into undecidable, never into a smaller denominator."""
    priors = {name: load_pallet_prior(config / f"pallet_prior_{name}.yaml") for name in ("epal6", "t11_06")}
    rows, errors = [], []
    for level in sorted(p.name for p in dataset.glob("isaac_*")):
        for scene_dir in sorted((dataset / level).glob("n_negative_*")):
            try:
                if read_meta(scene_dir)["split"] != "dev":
                    raise ValueError("a non-dev scene in the restored set")
                scene = load_recognition_sample(scene_dir).input.scene
            except (OSError, ValueError, KeyError) as exc:
                errors.append(f"{level}/{scene_dir.name}: {exc}")
                continue
            for name, prior in priors.items():
                base = DetectorParams.derived_for(prior)
                for lower in (True, False):
                    for arm in ARMS:
                        params, kw = arm_call(base, arm)
                        obs = det.detect_pockets_expected(scene, prior, params, None, lower_evidence=lower, **kw)[0].observation
                        rows.append({"level": level, "scene": scene_dir.name, "prior": name, "lower": lower, "arm": arm,
                                     "valid": obs.status == "valid"})
    return {"rows": rows, "errors": errors}


# --- E4 -----------------------------------------------------------------------------------------------------------------

TOY_PRIORS = {
    "toy": PalletPrior(height_m=0.085, deck_bottom_m=0.0, deck_top_m=0.02, opening_height_m=0.065,
                       opening_width_min_m=0.145, opening_width_max_m=0.150, centre_spacer_min_m=0.040,
                       centre_spacer_max_m=0.045, overall_width_m=0.42, overall_depth_m=0.30,
                       source_provenance="evaluation_only_tape_and_photo_reading", catalogue_version="toy42_provisional"),
    "toy30": PalletPrior(height_m=0.085, deck_bottom_m=0.0, deck_top_m=0.02, opening_height_m=0.065,
                         opening_width_min_m=0.085, opening_width_max_m=0.090, centre_spacer_min_m=0.030,
                         centre_spacer_max_m=0.035, overall_width_m=0.30, overall_depth_m=0.42,
                         source_provenance="evaluation_only_tape_and_photo_reading", catalogue_version="toy30_provisional"),
}
LENS_HEIGHT = re.compile(r"lens height ([0-9.]+) ?m")


def toy_input(meta: dict, z16: np.ndarray) -> tuple[SceneInput | None, str | None]:
    """A level camera on a stand: base_link on the floor under the lens, x forward. None and why when the capture
    does not state a rectified grid or its mount."""
    device = meta["device"]
    i = device["intrinsics"]
    if any(float(c) != 0.0 for c in i["coeffs"]) or device.get("post_processing"):
        return None, "not_rectified"
    note = meta["setup"].get("notes", "")
    height = LENS_HEIGHT.search(note)
    if height is None or "level" not in note:
        return None, "mount_unknown"
    depth = z16.astype(float) * float(device["depth_scale_m"])
    depth[z16 == 0] = np.nan
    h, w = z16.shape
    rotation = np.array(((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)))
    mount = RigidTransform("camera_optical_frame", "base_link", rotation, np.array((0.0, 0.0, float(height.group(1)))))
    k = PinholeIntrinsics(w, h, float(i["fx"]), float(i["fy"]), float(i["ppx"]), float(i["ppy"]), "camera_optical_frame")
    return SceneInput(np.zeros((h, w, 3), np.uint8), depth, k, mount, 0, "device", "replay"), None


def toy_sweep(data: Path) -> dict:
    """E4, reported apart: per capture and arm the valid count, the spread of pocket centres and yaw over the frames,
    the detected front against the tape; walls are negatives. Never judged, section 4."""
    from PIL import Image  # noqa: PLC0415 -- this report alone reads the PNG frames

    rows, skipped = [], {}
    for capture in sorted(p for p in data.iterdir() if p.is_dir()):
        kind = capture.name.split("_")[0]
        meta = json.loads((capture / "capture.json").read_text())
        frames = sorted(capture.glob("z16_*.png"))
        if kind not in ("wall", "toy", "toy30"):
            skipped[capture.name] = "other_surface"
            continue
        if float(meta["setup"]["distance_m"]) < DetectorParams().range_min_m:
            skipped[capture.name] = "out_of_range"
            continue
        priors = TOY_PRIORS if kind == "wall" else {kind: TOY_PRIORS[kind]}
        scenes, why = [], None
        for frame in frames:
            scene, why = toy_input(meta, np.asarray(Image.open(frame)))
            if scene is None:
                break
            scenes.append(scene)
        if why is not None:
            skipped[capture.name] = why
            continue
        for name, prior in priors.items():
            for arm in ARMS:
                params, kw = arm_call(DetectorParams.derived_for(prior), arm)
                detections = [det.detect_pockets_expected(s, prior, params, None, lower_evidence=False, **kw)[0]
                              for s in scenes]
                seen = [detection_record(d, False) for d in detections if d.observation.status == "valid"]
                rows.append(toy_row(capture.name, kind, name, arm, len(frames), seen, float(meta["setup"]["distance_m"])))
    return {"rows": rows, "skipped": skipped}


def toy_row(capture, kind, prior, arm, frames, seen, tape_m) -> dict:
    row = {"capture": capture, "kind": kind, "prior": prior, "arm": arm, "frames": frames, "valid": len(seen)}
    if len(seen) >= 2:
        points = np.array([r["left"] + r["right"] for r in seen])
        row.update(centre_std_m=float(points.std(axis=0).max()), yaw_std_rad=float(np.std([r["yaw"] for r in seen])))
    if seen:
        row["front_minus_tape_m"] = float(np.mean([(r["left"][0] + r["right"][0]) / 2 for r in seen]) - tape_m)
    return row


# --- the judge ----------------------------------------------------------------------------------------------------------

def percentile95(values) -> float:
    return float(np.percentile(np.asarray(values, float), 95))


def frame_errors(entry: dict, record: dict) -> dict | None:
    if record["status"] != "valid" or entry["truth"] is None:
        return None
    return pose_errors(record, entry["truth"])


def degradation(rows: list, arm: str) -> dict:
    """Section 4.2 over (entry, arms) rows: the problems of ``arm`` against A0; an empty list passes."""
    problems, common = [], []
    for entry, arms in rows:
        a0, cand = arms["A0"], arms[arm]
        if a0["status"] == "valid" and cand["status"] != "valid":
            problems.append(f"flip:{entry['id']}")
        e0, e1 = frame_errors(entry, a0), frame_errors(entry, cand)
        if a0["status"] == "valid" and cand["status"] == "valid":
            if e0 is None or e1 is None:
                problems.append(f"no_truth:{entry['id']}")
                continue
            common.append((e0, e1))
            worst = max(e1[p] - e0[p] for p in POINTS)
            if worst > POINT_WORSENING_M:
                problems.append(f"point_worsened:{entry['id']}:{worst:.4f}")
            if max(e1[p] for p in POINTS) > LARGE_ERROR_M >= max(e0[p] for p in POINTS):
                problems.append(f"new_large_error:{entry['id']}")
        if a0["status"] != "valid" and cand["status"] == "valid":
            if e1 is None:
                problems.append(f"new_valid_without_truth:{entry['id']}")
            elif not quality_pass(e1):
                problems.append(f"new_valid_quality:{entry['id']}")
            elif max(e1[p] for p in POINTS) > LARGE_ERROR_M:
                problems.append(f"new_large_error:{entry['id']}")
    valid = {a: sum(1 for _, arms in rows if arms[a]["status"] == "valid") for a in ("A0", arm)}
    if valid[arm] < valid["A0"]:
        problems.append("valid_count")
    misses = {a: sum(1 for entry, arms in rows if (e := frame_errors(entry, arms[a])) and e["match_m"] > MATCH_M)
              for a in ("A0", arm)}
    if misses[arm] > misses["A0"]:
        problems.append("match_failures")
    return {"problems": problems, "common_valid": len(common), "p95": p95_rule(common, problems), "valid": valid,
            "match_failures": misses}


def p95_rule(common: list, problems: list) -> dict:
    groups = {"lateral": ("centre_left", "centre_right"), "along": ALONG, "wall": WALLS}
    out = {}
    if not common:
        return out
    for name, points in groups.items():
        before = percentile95([max(e0[p] for p in points) for e0, _ in common])
        after = percentile95([max(e1[p] for p in points) for _, e1 in common])
        out[name] = {"a0": before, "arm": after}
        if after - before > P95_INCREASE_M:
            problems.append(f"p95_{name}")
    before, after = percentile95([e0["yaw"] for e0, _ in common]), percentile95([e1["yaw"] for _, e1 in common])
    out["yaw"] = {"a0": before, "arm": after}
    if after - before > P95_INCREASE_RAD:
        problems.append("p95_yaw")
    return out


def stratum_rule(rows: list, arm: str) -> dict:
    """The p95 rule per shape x role cross stratum with at least MIN_STRATUM common valids; fewer is 'not applied'."""
    strata = {}
    for entry, arms in rows:
        strata.setdefault(f"{entry['shape']}|{entry['role']}", []).append((entry, arms))
    out = {}
    for name, members in sorted(strata.items()):
        common = [(frame_errors(e, a["A0"]), frame_errors(e, a[arm])) for e, a in members
                  if a["A0"]["status"] == "valid" and a[arm]["status"] == "valid" and e["truth"] is not None]
        if len(common) < MIN_STRATUM:
            out[name] = {"applied": False, "common_valid": len(common)}
            continue
        problems = []
        out[name] = {"applied": True, "common_valid": len(common), "p95": p95_rule(common, problems), "problems": problems}
    return out


def rescue(entry: dict, record: dict) -> list:
    if record["status"] != "valid":
        return [f"status:{record['status']}:{record['reason']}"]
    e = frame_errors(entry, record)
    problems = [] if e["match_m"] <= MATCH_M else ["match"]
    problems += [p for p in POINTS if e[p] > E1_POSITION_M]
    return problems + (["yaw"] if e["yaw"] > E1_YAW_RAD else [])


def timing(rows: list, arm: str) -> dict:
    near = [(a["A0"]["wall_s"], a[arm]["wall_s"]) for e, a in rows[WARMUP_ATTEMPTS:]]
    base, cand = [p[0] for p in near], [p[1] for p in near]
    out = {"a0_p95": percentile95(base), "arm_p95": percentile95(cand), "a0_max": max(base), "arm_max": max(cand)}
    out["passed"] = out["arm_p95"] <= TIME_P95_RATIO * out["a0_p95"] and out["arm_max"] <= TIME_MAX_RATIO * out["a0_max"]
    twice = [a[arm]["wall_s"] for _, a in rows if a[arm]["second_called"]]
    out["second_called"] = {"count": len(twice), "max": max(twice) if twice else None}
    return out


def seed_rule(seeds: dict, arm: str) -> dict:
    frames = seeds["frames"]
    a0 = [f["A0"]["failures"] / f["A0"]["seeds"] for f in frames.values()]
    cand = [f[arm]["failures"] / f[arm]["seeds"] for f in frames.values()]
    worse = [k for k, f in frames.items() if f[arm]["failures"] > f["A0"]["failures"] + SEED_EXTRA_FAILURES]
    e1 = (seeds["e1"]["A0"]["failures"], seeds["e1"][arm]["failures"])
    problems = []
    if statistics.fmean(cand) > statistics.fmean(a0):
        problems.append("mean_failure_rate")
    if e1[1] > e1[0]:
        problems.append("e1_failures")
    problems += [f"frame:{k}" for k in worse]
    return {"mean_a0": statistics.fmean(a0), "mean_arm": statistics.fmean(cand), "e1_a0": e1[0], "e1_arm": e1[1],
            "problems": problems}


def negative_rule(e3: dict, arm: str) -> dict:
    if e3["errors"] or not e3["rows"]:
        return {"undecidable": e3["errors"][:20] or ["no rows"]}
    problems, counts = [], {}
    keys = {(r["level"], r["lower"]) for r in e3["rows"]}
    for level, lower in sorted(keys):
        rows = [r for r in e3["rows"] if r["level"] == level and r["lower"] == lower]
        scenes = lambda a, prior=None: {r["scene"] for r in rows if r["arm"] == a and r["valid"]  # noqa: E731
                                        and (prior is None or r["prior"] == prior)}
        for prior in sorted({r["prior"] for r in rows}) + [None]:
            base, cand = scenes("A0", prior), scenes(arm, prior)
            counts[f"{level}|lower={lower}|{prior or 'any'}"] = {"a0": len(base), "arm": len(cand),
                                                                 "new": sorted(cand - base)}
            if len(cand) > len(base):
                problems.append(f"{level}|lower={lower}|{prior or 'any'}")
    return {"counts": counts, "problems": problems}


def toy_flags(e4: dict, arm: str) -> list:
    """Not a rule: what section 4 says goes beside the verdict for the lead."""
    flags = []
    rows = {(r["capture"], r["prior"], r["arm"]): r for r in e4.get("rows", [])}
    for (capture, prior, a), row in rows.items():
        if a != arm:
            continue
        base = rows[(capture, prior, "A0")]
        if row["kind"] == "wall" and row["valid"]:
            flags.append(f"wall_valid:{capture}:{prior}")
        if row["kind"] != "wall":
            if row["valid"] < base["valid"]:
                flags.append(f"valid_fell:{capture}")
            if row.get("centre_std_m", 0.0) > base.get("centre_std_m", 0.0) + TOY_STD_WORSENING_M:
                flags.append(f"spread_grew:{capture}")
    return flags


def reports(rows: list, arm: str) -> dict:
    moved = Counter()
    lost = []
    stops = Counter()
    for entry, arms in rows:
        a0, cand = arms["A0"], arms[arm]
        if a0["status"] == "valid" and cand["status"] == "valid":
            shift = max(np.linalg.norm(np.subtract(a0[s], cand[s])) for s in ("left", "right"))
            if shift > DETECTION_DELTA_M or abs(wrap(a0["yaw"] - cand["yaw"])) > DETECTION_DELTA_RAD:
                moved[entry["role"]] += 1
        for r in cand.get("refit_rounds", []):
            stops[r["stop"]] += 1
            if r["sizes"][-1] == cand["plane_inlier_count"] and (r["support_loss"] or 0.0) > SUPPORT_LOSS_REPORT:
                lost.append(entry["id"])
    return {"detection_moved_by_role": dict(moved), "selected_support_loss": lost, "refit_stops": dict(stops)}


def judge(manifest: dict, replay: dict, again: dict, seeds: dict, e3: dict, e4: dict) -> dict:
    frames = {f["id"]: f["arms"] for f in replay["frames"]}
    rows = [(e, frames[e["id"]]) for e in manifest["entries"]]
    applied = [(e, a) for e, a in rows if e["path"]["kind"] == "near_expected"]
    repeated = {f["id"]: f["arms"] for f in again["frames"]}
    e1 = next(e for e in manifest["entries"] if e["e1"])
    verdicts = {}
    for arm in CANDIDATES:
        checks = {"rescue": rescue(e1, frames[e1["id"]][arm]), "all": degradation(rows, arm),
                  "applied": degradation(applied, arm), "strata": stratum_rule(rows, arm),
                  "negatives": negative_rule(e3, arm), "time": timing(applied, arm), "seeds": seed_rule(seeds, arm),
                  "determinism": [i for i, a in frames.items()
                                  if a[arm]["canonical_sha256"] != repeated[i][arm]["canonical_sha256"]]}
        failed = []
        if checks["rescue"]:
            failed.append("rescue")
        for name in ("all", "applied"):
            if checks[name]["problems"]:
                failed.append(f"degradation_{name}")
        if any(s.get("problems") for s in checks["strata"].values()):
            failed.append("strata")
        if checks["negatives"].get("problems"):
            failed.append("negatives")
        if not checks["time"]["passed"]:
            failed.append("time")
        if checks["seeds"]["problems"]:
            failed.append("seeds")
        if checks["determinism"]:
            failed.append("determinism")
        verdict = "not_adoptable" if failed else "adoptable"
        if not failed and "undecidable" in checks["negatives"]:
            verdict = "undecidable"
        verdicts[arm] = {"verdict": verdict, "failed": failed, "checks": checks, "reports": reports(rows, arm),
                         "toy_flags": toy_flags(e4, arm)}
    adoptable = [a for a in ("A1", "A3", "A2") if verdicts[a]["verdict"] == "adoptable"]
    return {"verdicts": verdicts, "recommended_order": adoptable, "toy_skipped": e4.get("skipped"),
            "frames": len(rows), "applied_frames": len(applied)}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("manifest")
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--e1-run", required=True)
    p.add_argument("--e1-mission", required=True)
    p.add_argument("--e1-attempt", type=int, required=True)
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("gate")  # the freeze gate: A0 against the records
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--replay", type=Path, required=True)
    p = sub.add_parser("replay")
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--arms", default=",".join(ARMS))
    p = sub.add_parser("seeds")
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--replay", type=Path, required=True)
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("negatives")
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("toy")
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("judge")
    for name in ("manifest", "replay", "again", "seeds", "negatives", "toy", "out"):
        p.add_argument(f"--{name}", type=Path, required=True)
    args = parser.parse_args(argv)

    def load(path):
        return json.loads(Path(path).read_text())

    if args.command == "manifest":
        e1 = {"run": args.e1_run, "mission": args.e1_mission, "attempt": args.e1_attempt}
        result = build_manifest(args.root, e1)
        result["commit"] = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
        args.out.write_text(json.dumps(result))
        print("entries", len(result["entries"]), "files", result["files"], "excluded", result["excluded"],
              "e1", sum(e["e1"] for e in result["entries"]), "no truth", sum(e["truth"] is None for e in result["entries"]),
              "roles", dict(Counter(e["role"] for e in result["entries"])),
              "paths", dict(Counter((e["path"]["kind"], e["path"]["waiver"].split(":")[0]) for e in result["entries"])),
              "recorded", dict(Counter(e["recorded_status"] for e in result["entries"])))
    elif args.command == "gate":
        manifest, replay = load(args.manifest), load(args.replay)
        a0 = {f["id"]: f["arms"]["A0"] for f in replay["frames"]}
        kinds = Counter()
        unexplained = []
        for entry in manifest["entries"]:
            why = classify_mismatch(entry, a0[entry["id"]])
            kinds[why] += 1
            if why == "unexplained":
                unexplained.append((entry["id"], entry["recorded_status"], entry["recorded_reason"],
                                    a0[entry["id"]]["status"], a0[entry["id"]]["reason"]))
        print("gate", dict(kinds))
        print("unexplained", json.dumps(unexplained, indent=0))
    elif args.command == "replay":
        args.out.write_text(json.dumps(sweep(load(args.manifest), args.root, args.arms.split(","))))
    elif args.command == "seeds":
        args.out.write_text(json.dumps(seed_sweep(load(args.manifest), load(args.replay), args.root)))
    elif args.command == "negatives":
        args.out.write_text(json.dumps(negative_sweep(args.dataset, args.config)))
    elif args.command == "toy":
        args.out.write_text(json.dumps(toy_sweep(args.data)))
    else:
        result = judge(load(args.manifest), load(args.replay), load(args.again), load(args.seeds),
                       load(args.negatives), load(args.toy))
        args.out.write_text(json.dumps(result, indent=1))
        print({a: (v["verdict"], v["failed"]) for a, v in result["verdicts"].items()})
    return 0


if __name__ == "__main__":
    sys.exit(main())
