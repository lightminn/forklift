"""Registered comparison of the near-capture plane refit arms (docs/plans/2026-11-06-near-capture-plane-refit.md).

    python -m tools.near_refit_compare manifest --root test_runs --e1-run R --e1-mission m2 --e1-attempt 1 --out manifest.json
    python -m tools.near_refit_compare replay --manifest manifest.json --root test_runs --arms A0 --out gate.json
    python -m tools.near_refit_compare gate --manifest manifest.json --replay gate.json   # before any arm is run
    python -m tools.near_refit_compare replay --manifest manifest.json --root test_runs --out replay_a.json
    python -m tools.near_refit_compare seeds --manifest manifest.json --root test_runs --replay replay_a.json --out seeds.json
    python -m tools.near_refit_compare negatives --manifest manifest.json --dataset <dir> --config config --out e3.json
    python -m tools.near_refit_compare toy --manifest manifest.json --data <d435i_noise/date> --out e4.json
    python -m tools.near_refit_compare judge --manifest manifest.json --replay replay_a.json --again replay_b.json \\
        --seeds seeds.json --negatives e3.json --toy e4.json --evidence evidence.json --out judge.json

The saved depth of every observation attempt is replayed through the path the run took: the first call restored from
the attempt record, the lower waiver's conditions restored from the records, and whether the first call succeeded and
whether the waiver's second call ran computed again per arm (plan section 3). Every arm runs on every attempt, but an
arm reaches only the near expected-site calls, so the judged comparison keeps A0 elsewhere and the rest is a reference.
The judge only applies section 4's rules to these outputs; its thresholds are the constants below, registered before
any result was read. evidence.json carries section 4.7's checks, which come from outside this tool.
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
E3_LEVELS = ("isaac_clean", "isaac_high", "isaac_low", "isaac_mid")
E3_SCENES_PER_LEVEL = 48  # the dev negatives without the grounded kind (positive since 2026-10-27)
NEGATIVE_KINDS = ("grounded", "shelf", "blocks", "crates")  # tools/recognition_scenes.py: the kind by rank % 4
# The freeze gate's checks of the truth (A0 only)
GATE_MAX_TILT_RAD = 0.02
GATE_YAW_RAD = 0.1
OBSERVATION_ATOL = 1e-6
EVIDENCE_GATE_RUNS = 29
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


def tilt_of(q_wxyz) -> float:
    _, x, y, _ = (float(v) for v in q_wxyz)
    return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))


def truth_in_base(pallet_xy, pallet_yaw, accepted_pose, geometry: dict, camera_xy=(0.0, 0.0)) -> dict:
    """The target pallet in the capture's true base frame (yaw only: the freeze gate bounds the tilt): the entry face
    is the pallet's x face that looks at the camera, with its pockets' centre and wall offsets and the pallet centre."""
    (bx, by, _), q = accepted_pose
    base_yaw = yaw_of(q)
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    dx, dy = float(pallet_xy[0]) - float(bx), float(pallet_xy[1]) - float(by)
    centre = (c * dx + s * dy, -s * dx + c * dy)
    axis = wrap(float(pallet_yaw) - base_yaw)
    view = (centre[0] - float(camera_xy[0]), centre[1] - float(camera_xy[1]))
    if view[0] * math.cos(axis) + view[1] * math.sin(axis) < 0:  # the insertion axis points away from the camera
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
        record["refit_rounds"] = log  # the plan's whole round record, kept as the detector wrote it
    return record


# --- manifest ---------------------------------------------------------------------------------------------------------

def attempt_identity(attempt: dict) -> tuple:
    """The same attempt in a mission's records and in the run's result; equal depth alone does not make it the same."""
    return attempt.get("depth_sha256"), attempt.get("t_before_capture_s"), attempt.get("loop_step")


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
            seen.add(attempt_identity(attempt))
            yield records.parent.name, index, attempt, state
    for index, attempt in enumerate(result.get("observation_attempts") or []):
        if not missions:
            yield None, index, attempt, result
        elif attempt_identity(attempt) not in seen:
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


def mission_targets(run: Path, result: dict) -> dict:
    """mission id -> the pallet the mission was told to fetch (live runs; the last one also under "last")."""
    targets = {r.get("mission_id"): r.get("pallet_id") for r in result.get("mission_results") or []}
    for path in sorted(run.glob("missions/m*/result.json")):
        try:
            record = json.loads(path.read_text())["result"]
        except (OSError, ValueError, KeyError):
            continue
        targets[record.get("mission_id")] = record.get("pallet_id")
        targets["last"] = record.get("pallet_id")
    return targets


def input_hash(attempt: dict, params: dict, prior: dict, path: dict) -> str:
    """What a replay reads besides the depth: frozen with the manifest, checked at every load."""
    frozen = {"mount": attempt.get("base_from_optical"), "quantize": attempt.get("depth_quantize_mm"), "params": params,
              "intrinsics": (attempt.get("capture_diagnostics") or {}).get("intrinsics"), "prior": prior, "path": path}
    return hashlib.sha256(json.dumps(frozen, sort_keys=True, default=list).encode()).hexdigest()


def observation_fields(record: dict | None) -> dict:
    record = record or {}
    left, right = record.get("left") or {}, record.get("right") or {}
    return {"status": record.get("status"), "reason": record.get("reason"), "yaw": record.get("insertion_yaw_rad"),
            "left": (left.get("center_m") or [None, None])[:2], "right": (right.get("center_m") or [None, None])[:2],
            "widths": [left.get("width_m"), right.get("width_m")]}


def build_manifest(root: Path, e1: dict) -> dict:
    """One entry per recorded attempt whose depth was saved; several attempts may share a depth file."""
    entries, excluded = [], Counter()
    by_run = {}
    paths = sorted(root.glob("**/perception_capture_*_depth_m.npy"))
    for path in paths:
        by_run.setdefault(path.parent, []).append(path)
    for run, files in sorted(by_run.items()):
        try:
            result = json.loads((run / "result.json").read_text())
        except (OSError, ValueError):
            excluded["no_result"] += len(files)
            continue
        by_sha = {}
        for path in files:
            by_sha.setdefault(DD.depth_array_sha256(np.load(path)), path)
        arguments = result.get("arguments") or {}
        if not arguments.get("pallet_prior_loaded") or not arguments.get("pallet_geometry_loaded"):
            excluded["no_prior_or_geometry"] += len(files)
            continue
        samples, targets, used = run_samples(run, result), mission_targets(run, result), set()
        for mission, index, attempt, state in run_attempts(run, result):
            sha = attempt.get("depth_sha256")
            if sha not in by_sha:
                excluded["attempt_without_depth"] += 1
                continue
            used.add(sha)
            params = state.get("detector_params") or result.get("detector_params")
            if not params:
                excluded["no_params"] += 1
                continue
            sample = pallet_at(samples, capture_time(attempt))
            pose = (attempt.get("capture_diagnostics") or {}).get("accepted_pose")
            mount = attempt.get("base_from_optical")
            camera = mount["translation_m"][:2] if mount else DD._adapter().default_base_from_optical().translation_m[:2]
            truth = None
            if sample is not None and pose is not None:
                truth = truth_in_base(sample["pallet_position_m"][:2], sample["pallet_yaw_rad"], pose,
                                      arguments["pallet_geometry_loaded"], camera)
            relative = str(run.relative_to(root))
            path_record = replay_path(attempt, state, arguments)
            prior = arguments["pallet_prior_loaded"]
            entries.append({"id": f"{relative}#{mission}#{index}", "file": str(by_sha[sha].relative_to(root)),
                            "depth_sha256": sha, "run": relative, "mission": mission, "attempt": index,
                            "role": attempt_role(index, attempt, state, arguments), "path": path_record, "truth": truth,
                            "shape": prior["catalogue_version"], "proposals": arguments.get("recognition_proposals"),
                            "recognition": arguments.get("recognition"), "params": params,
                            "input_sha256": input_hash(attempt, params, prior, path_record),
                            "target_id": targets.get(mission), "sample_pallet_id": None if sample is None else sample.get("pallet_id"),
                            "sample_age_s": None if sample is None else capture_time(attempt) - float(sample["time_s"]),
                            "tilt_rad": None if pose is None else tilt_of(pose[1]),
                            "recorded": observation_fields(DD.recorded_observation(attempt)),
                            "e1": (relative, mission, index) == (e1["run"], e1["mission"], e1["attempt"])})
        excluded["no_attempt_match"] += sum(1 for sha in by_sha if sha not in used)
    return {"entries": entries, "files": len(paths), "excluded": dict(excluded)}


def load_frame(root: Path, entry: dict, cache: dict):
    """(scene, prior, params) of a manifest entry, refusing a depth or a record that is not the frozen one."""
    run = root / entry["run"]
    if cache.get("run") != run:
        result = json.loads((run / "result.json").read_text())
        cache.update(run=run, result=result, attempts={(m, i): (a, s) for m, i, a, s in run_attempts(run, result)})
    result = cache["result"]
    attempt, _ = cache["attempts"][(entry["mission"], entry["attempt"])]
    depth = np.load(root / entry["file"])
    prior = result["arguments"]["pallet_prior_loaded"]
    if DD.depth_array_sha256(depth) != entry["depth_sha256"] or input_hash(
            attempt, entry["params"], prior, entry["path"]) != entry["input_sha256"]:
        raise ValueError(f"not the frozen input: {entry['id']}")
    return DD.attempt_scene(result, attempt, depth), PalletPrior(**prior), DetectorParams(**entry["params"])


def classify_mismatch(entry: dict, replay: dict) -> str | None:
    """Why a same-code A0 replay differs from the whole recorded observation; None when it does not."""
    recorded = entry["recorded"]
    same = recorded["status"] == replay["status"] and recorded["reason"] == replay["reason"]
    if same and replay["status"] == "valid":
        same = all(np.allclose(recorded[k], replay[k], rtol=0.0, atol=OBSERVATION_ATOL) for k in ("left", "right", "widths"))
        same = same and abs(recorded["yaw"] - replay["yaw"]) <= OBSERVATION_ATOL
    if same:
        return None
    return "far_outside_zone_not_replayed" if entry["path"].get("outside_zone_recorded") else "unexplained"


def freeze_gate(manifest: dict, replay: dict) -> dict:
    """The check before any arm runs: one entry per attempt, E1 exactly once, the records reproduced in full by A0,
    the truth's target and pose assumptions held. ``problems`` empty is the only way to freeze."""
    entries = manifest["entries"]
    a0 = {f["id"]: f["arms"]["A0"] for f in replay["frames"]}
    problems = []
    ids = Counter(e["id"] for e in entries)
    problems += [f"duplicate_id:{i}" for i, n in ids.items() if n > 1]
    if sum(e["e1"] for e in entries) != 1:
        problems.append("e1_not_exactly_one")
    kinds = Counter()
    for entry in entries:
        record = a0[entry["id"]]
        why = classify_mismatch(entry, record)
        kinds[why] += 1
        if why == "unexplained":
            problems.append(f"unexplained:{entry['id']}:{entry['recorded']['status']}->{record['status']}")
        if entry["truth"] is None:
            problems.append(f"no_truth:{entry['id']}")
            continue
        if (entry["tilt_rad"] or 0.0) > GATE_MAX_TILT_RAD:
            problems.append(f"tilt:{entry['id']}:{entry['tilt_rad']:.4f}")
        if entry["target_id"] is not None and entry["sample_pallet_id"] not in (None, entry["target_id"]):
            problems.append(f"target:{entry['id']}:{entry['target_id']}!={entry['sample_pallet_id']}")
        if record["status"] == "valid":
            errors = pose_errors(record, entry["truth"])
            if errors["yaw"] > GATE_YAW_RAD or errors["match_m"] > MATCH_M:
                problems.append(f"truth_disagrees:{entry['id']}:{errors['match_m']:.3f}:{errors['yaw']:.3f}")
    return {"problems": problems, "mismatch": {str(k): v for k, v in kinds.items()}, "entries": len(entries),
            "max_tilt_rad": max((e["tilt_rad"] or 0.0) for e in entries),
            "max_sample_age_s": max((e["sample_age_s"] or 0.0) for e in entries)}


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
    out = {"e1": {}, "frames": {}, "e1_seeds": list(E1_SEEDS), "frame_seeds": list(FRAME_SEEDS)}
    e1 = next(e for e in entries if e["e1"])
    cache = {}
    scene, prior, params = load_frame(Path(root), e1, cache)
    for arm in ARMS:
        out["e1"][arm] = seed_failures(scene, prior, params, e1, E1_SEEDS, arm)
    for entry in sample:
        scene, prior, params = load_frame(Path(root), entry, cache)
        out["frames"][entry["id"]] = {arm: seed_failures(scene, prior, params, entry, FRAME_SEEDS, arm) for arm in ARMS}
    return out


def negative_kind(meta: dict) -> str:
    return NEGATIVE_KINDS[int(meta["rank"]) % len(NEGATIVE_KINDS)]


def negative_sweep(dataset: Path, config: Path) -> dict:
    """E3: dev-split negatives without the grounded kind, every noise level, both priors, both lower-evidence modes. A
    scene that cannot be read is an error: the judge turns it into undecidable, never into a smaller denominator."""
    priors = {name: load_pallet_prior(config / f"pallet_prior_{name}.yaml") for name in ("epal6", "t11_06")}
    rows, errors, left_out = [], [], Counter()
    for level in sorted(p.name for p in dataset.glob("isaac_*")):
        for scene_dir in sorted((dataset / level).glob("n_negative_*")):
            try:
                meta = read_meta(scene_dir)
                if meta["split"] != "dev":
                    left_out["not_dev"] += 1
                    continue
                if negative_kind(meta) == "grounded":
                    left_out["grounded"] += 1
                    continue
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
                        rows.append({"level": level, "scene": scene_dir.name, "kind": negative_kind(meta), "prior": name,
                                     "lower": lower, "arm": arm, "valid": obs.status == "valid"})
    return {"rows": rows, "errors": errors, "left_out": dict(left_out)}


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
                rows.append(toy_row(capture.name, kind, name, arm, len(frames), seen))
    return {"rows": rows, "skipped": skipped}


def toy_row(capture, kind, prior, arm, frames, seen) -> dict:
    """The tape distance is camera front to pallet front, not this base's lens origin: not compared (plan 3 E4)."""
    row = {"capture": capture, "kind": kind, "prior": prior, "arm": arm, "frames": frames, "valid": len(seen)}
    if len(seen) >= 2:
        points = np.array([r["left"] + r["right"] for r in seen])
        row.update(centre_std_m=float(points.std(axis=0).max()), yaw_std_rad=float(np.std([r["yaw"] for r in seen])))
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
    """Section 4.3. Every scene x level x prior x mode x arm exactly once, or undecidable: a missing arm is not zero."""
    if e3["errors"] or not e3["rows"]:
        return {"undecidable": e3["errors"][:20] or ["no rows"]}
    keys = Counter((r["level"], r["scene"], r["prior"], r["lower"], r["arm"]) for r in e3["rows"])
    levels, priors = sorted({k[0] for k in keys}), sorted({k[2] for k in keys})
    scenes = {level: sorted({k[1] for k in keys if k[0] == level}) for level in levels}
    wanted = {(level, scene, prior, lower, a) for level in levels for scene in scenes[level] for prior in priors
              for lower in (True, False) for a in ARMS}
    shape = [] if tuple(levels) == E3_LEVELS and all(len(v) == E3_SCENES_PER_LEVEL for v in scenes.values()) else ["population"]
    if set(keys) != wanted or max(keys.values()) > 1 or shape:
        return {"undecidable": shape + [f"keys {len(keys)} of {len(wanted)}"]}
    problems, counts = [], {}
    for level in levels:
        for lower in (True, False):
            rows = [r for r in e3["rows"] if r["level"] == level and r["lower"] == lower]
            for prior in [*priors, None]:
                base = {r["scene"] for r in rows if r["arm"] == "A0" and r["valid"] and prior in (None, r["prior"])}
                cand = {r["scene"] for r in rows if r["arm"] == arm and r["valid"] and prior in (None, r["prior"])}
                name = f"{level}|lower={lower}|{prior or 'any'}"
                counts[name] = {"a0": len(base), "arm": len(cand), "new": sorted(cand - base)}
                if len(cand) > len(base):
                    problems.append(name)
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
            if "centre_std_m" in row and "centre_std_m" in base and (
                    row["centre_std_m"] > base["centre_std_m"] + TOY_STD_WORSENING_M):
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
        rounds = [r for r in cand.get("refit_rounds", []) if "stop" in r]
        for r in rounds:
            stops[r["stop"]] += 1
        chosen = [r["selected_plane_index"] for r in cand.get("refit_rounds", []) if "selected_plane_index" in r]
        if chosen and (rounds[chosen[-1]]["support_loss"] or 0.0) > SUPPORT_LOSS_REPORT:
            lost.append(entry["id"])
    return {"detection_moved_by_role": dict(moved), "selected_support_loss": lost, "refit_stops": dict(stops)}


def operational(rows: list, arm: str) -> list:
    """What running the arm would change: only the near expected-site detections; every other call keeps A0."""
    return [(e, {"A0": a["A0"], arm: a[arm] if e["path"]["kind"] == "near_expected" else a["A0"]}) for e, a in rows]


def evidence_problems(evidence: dict) -> list:
    """Section 4.7's evidence, which this tool cannot produce itself; without it no arm reads adoptable."""
    problems = []
    if evidence.get("replay_gate") != [EVIDENCE_GATE_RUNS, EVIDENCE_GATE_RUNS]:
        problems.append("replay_gate")
    if evidence.get("full_pytest_failed") != 0:
        problems.append("full_pytest")
    if evidence.get("stored_eval_changes") != 0:
        problems.append("stored_eval")
    if evidence.get("near_contract_default_match") is not True:
        problems.append("near_contract")
    if not evidence.get("v1_guard_baseline"):
        problems.append("v1_guard_baseline")
    return problems


def input_problems(manifest: dict, replay: dict, again: dict, seeds: dict, stamps: dict, entries: list) -> list:
    """Every output of the same manifest and source, for the exact registered frames and seed lists."""
    problems = [f"stamp:{k}" for k, v in stamps.items() if v != stamps["replay"]]
    ids = {e["id"] for e in entries}
    for name, run in (("replay", replay), ("again", again)):
        if {f["id"] for f in run["frames"]} != ids or any(set(f["arms"]) != set(ARMS) for f in run["frames"]):
            problems.append(f"frames:{name}")
    if seeds["e1_seeds"] != list(E1_SEEDS) or seeds["frame_seeds"] != list(FRAME_SEEDS):
        problems.append("seed_lists")
    a0 = {f["id"]: f["arms"]["A0"] for f in replay["frames"]}
    wanted = {e["id"] for e in seed_sample([e for e in entries if e["truth"] is not None], a0)}
    if set(seeds["frames"]) != wanted or len(wanted) != UNIFORM_FRAMES + RESIDUAL_FRAMES:
        problems.append("seed_frames")
    counts = [r["seeds"] for f in seeds["frames"].values() for r in f.values()]
    if any(c != len(FRAME_SEEDS) for c in counts) or any(seeds["e1"][a]["seeds"] != len(E1_SEEDS) for a in ARMS):
        problems.append("seed_counts")
    return problems


def judge(manifest: dict, replay: dict, again: dict, seeds: dict, e3: dict, e4: dict, evidence: dict,
          stamps: dict | None = None) -> dict:
    frames = {f["id"]: f["arms"] for f in replay["frames"]}
    rows = [(e, frames[e["id"]]) for e in manifest["entries"]]
    applied = [(e, a) for e, a in rows if e["path"]["kind"] == "near_expected"]
    others = [(e, a) for e, a in rows if e["path"]["kind"] != "near_expected"]
    repeated = {f["id"]: f["arms"] for f in again["frames"]}
    e1 = next(e for e in manifest["entries"] if e["e1"])
    unusable = input_problems(manifest, replay, again, seeds, stamps or {"replay": None}, manifest["entries"])
    unproven = evidence_problems(evidence)
    verdicts = {}
    for arm in CANDIDATES:
        whole = operational(rows, arm)
        checks = {"rescue": rescue(e1, frames[e1["id"]][arm]), "all": degradation(whole, arm),
                  "applied": degradation(applied, arm), "strata": stratum_rule(whole, arm),
                  "negatives": negative_rule(e3, arm), "time": timing(applied, arm), "seeds": seed_rule(seeds, arm),
                  "determinism": [i for i, a in frames.items() for name in ("A0", arm)
                                  if a[name]["canonical_sha256"] != repeated[i][name]["canonical_sha256"]]}
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
        undecided = unusable + (["negatives"] if "undecidable" in checks["negatives"] else [])
        verdict = "not_adoptable" if failed else "undecidable" if undecided else "unverified" if unproven else "adoptable"
        verdicts[arm] = {"verdict": verdict, "failed": failed, "undecided": undecided, "unproven": unproven,
                         "checks": checks, "reports": reports(rows, arm), "toy_flags": toy_flags(e4, arm),
                         # the same arm on calls it would not reach: 'had it been applied', never judged; far_expected
                         # here skips the core's zone check after the circle
                         "reference": degradation(others, arm)}
    adoptable = [a for a in ("A1", "A3", "A2") if verdicts[a]["verdict"] == "adoptable"]
    return {"verdicts": verdicts, "recommended_order": adoptable, "toy_skipped": e4.get("skipped"),
            "negatives_left_out": e3.get("left_out"), "frames": len(rows), "applied_frames": len(applied)}


def source_sha256() -> str:
    digest = hashlib.sha256()
    for path in (Path(__file__), Path(det.__file__)):
        digest.update(path.read_bytes())
    return digest.hexdigest()


def stamp_of(manifest_path: Path) -> dict:
    return {"manifest_sha256": hashlib.sha256(Path(manifest_path).read_bytes()).hexdigest(), "source_sha256": source_sha256()}


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
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("toy")
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("judge")
    for name in ("manifest", "replay", "again", "seeds", "negatives", "toy", "evidence", "out"):
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
              "recorded", dict(Counter(e["recorded"]["status"] for e in result["entries"])))
    elif args.command == "gate":
        result = freeze_gate(load(args.manifest), load(args.replay))
        print(json.dumps(result, indent=0))
        return 1 if result["problems"] else 0
    elif args.command == "replay":
        out = sweep(load(args.manifest), args.root, args.arms.split(","))
        args.out.write_text(json.dumps({**out, "stamp": stamp_of(args.manifest)}))
    elif args.command == "seeds":
        out = seed_sweep(load(args.manifest), load(args.replay), args.root)
        args.out.write_text(json.dumps({**out, "stamp": stamp_of(args.manifest)}))
    elif args.command == "negatives":
        out = negative_sweep(args.dataset, args.config)
        args.out.write_text(json.dumps({**out, "stamp": stamp_of(args.manifest)}))
    elif args.command == "toy":
        args.out.write_text(json.dumps({**toy_sweep(args.data), "stamp": stamp_of(args.manifest)}))
    else:
        inputs = {k: load(getattr(args, k)) for k in ("replay", "again", "seeds", "negatives", "toy")}
        stamps = {k: v["stamp"] for k, v in inputs.items()}
        stamps["manifest"] = {**stamps["replay"], "manifest_sha256": stamp_of(args.manifest)["manifest_sha256"]}
        result = judge(load(args.manifest), inputs["replay"], inputs["again"], inputs["seeds"], inputs["negatives"],
                       inputs["toy"], load(args.evidence), stamps)
        args.out.write_text(json.dumps(result, indent=1))
        print({a: (v["verdict"], v["failed"], v["undecided"], v["unproven"]) for a, v in result["verdicts"].items()})
    return 0


if __name__ == "__main__":
    sys.exit(main())
