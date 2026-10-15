# src/forklift_core/perception/evaluation_multi.py
"""Scene-level judgment for multi-instance recognition (design §3.3-3.4).

Units are scenes: any failure in a scene fails that scene. Gates are judged on
pooled populations with one-sided 95 % Clopper-Pearson bounds; per-case cells
are reported only. An applicable gate with no scenes fails. Only accessible
faces are matched. Every pick-permitted candidate is checked for danger.
"""

import itertools
import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from forklift_core.perception.fork_fit import AXIAL_DANGER_M, TIP_LEVER_M
from forklift_core.perception.pocket_observation import yaw_difference_rad
from forklift_core.perception.recognition_stats import allowed_failures, cp_upper

MATCH_DISTANCE_M = 0.20
MATCH_YAW_RAD = 0.35
DECISIONS = ("selected", "ask_operator", "reobserve", "none", "not_pickable")
GATES = {"miss": 0.05, "dangerous": 0.05, "extra": 0.10, "negative_fp": 0.10, "target_error": 0.05,
         "deferred_unambiguous": 0.10, "fit_error": 0.05}
APPLICABLE = {
    "known:epal6": ("miss", "dangerous", "extra", "target_error", "deferred_unambiguous"),
    "known:t11_06": ("miss", "dangerous", "extra", "target_error", "deferred_unambiguous"),
    "nonstandard": ("miss", "dangerous", "extra", "target_error", "deferred_unambiguous", "fit_error"),
    "negative": ("negative_fp",),
}
POSE_P95 = {"lateral": 0.020, "axial": 0.020, "yaw": math.radians(2.0), "z": 0.005}


@dataclass(frozen=True)
class CandidateView:
    candidate_id: str
    insertion_yaw_rad: float
    left_m: tuple
    right_m: tuple
    fitted: bool
    identified: bool
    pickable: bool | None
    reasons: tuple
    layer_abs: int | None

    def __post_init__(self) -> None:
        if len(self.left_m) != 3 or len(self.right_m) != 3:
            raise ValueError("candidate pockets must be finite three-vectors")
        if not all(np.isfinite([self.insertion_yaw_rad, *self.left_m, *self.right_m])):
            raise ValueError("candidate pockets and yaw must be finite three-vectors")

    @property
    def counted(self) -> bool:
        return self.fitted and self.identified


@dataclass(frozen=True)
class TargetDecision:
    status: str
    candidate_id: str | None
    reasons: tuple

    def __post_init__(self) -> None:
        if self.status not in DECISIONS:
            raise ValueError(f"decision status must be one of {DECISIONS}")


@dataclass(frozen=True)
class SceneVerdict:
    scene_id: str
    case: str
    shape_key: str
    split: str
    population: str
    render_source: str
    noisy: bool
    miss: bool | None
    dangerous: bool
    extra: bool
    negative_fp: bool | None
    target_error: bool | None
    deferred_unambiguous: bool | None
    fit_error: bool | None
    errors: tuple
    z_judged: bool
    elapsed_s: float | None


def population_of(meta: dict) -> str:
    if meta["case"] == "N":
        return "negative"
    if meta["case"] == "S7":
        return "nonstandard"
    return f"known:{meta['shape_key']}"


def _face_cost(c: CandidateView, face: dict):
    dyaw = abs(yaw_difference_rad(c.insertion_yaw_rad, face["insertion_yaw_rad"]))
    dl, dr = np.subtract(c.left_m, face["left_m"]), np.subtract(c.right_m, face["right_m"])
    dist = max(float(np.linalg.norm(dl)), float(np.linalg.norm(dr)))
    if not (dyaw <= MATCH_YAW_RAD and dist <= MATCH_DISTANCE_M):
        return None
    a = np.array([math.cos(face["insertion_yaw_rad"]), math.sin(face["insertion_yaw_rad"]), 0.0])
    n = np.array([-a[1], a[0], 0.0])
    # Per pocket, per axis, worst of the two: opposite errors must not cancel.
    lateral = max(abs(float(dl @ n)), abs(float(dr @ n)))
    axial = max(abs(float(dl @ a)), abs(float(dr @ a)))
    return dist, (lateral, axial, dyaw, max(abs(float(dl[2])), abs(float(dr[2]))))


def _best_face(c, inst):
    best = None
    for face in inst["faces"]:
        if not face["accessible"]:
            continue
        fc = _face_cost(c, face)
        if fc and (best is None or fc[0] < best[0][0]):
            best = (fc, face)
    return best


def _match(cands, instances):
    costs = {}
    for ci, c in enumerate(cands):
        for ii, inst in enumerate(instances):
            best = _best_face(c, inst)
            if best is not None:
                costs[ci, ii] = best
    for size in range(min(len(cands), len(instances)), 0, -1):
        best, best_total = None, None
        for chosen in itertools.combinations(range(len(cands)), size):
            for perm in itertools.permutations(range(len(instances)), size):
                if all((c, i) in costs for c, i in zip(chosen, perm)):
                    total = sum(costs[c, i][0][0] for c, i in zip(chosen, perm))
                    if best_total is None or total < best_total:
                        best, best_total = dict(zip(chosen, perm)), total
        if best is not None:
            return best, costs
    return {}, costs


def _danger(errors, face) -> bool:
    lateral, axial, dyaw, _ = errors
    return (not face["fit"]["pickable"] or lateral + TIP_LEVER_M * math.tan(dyaw) > face["danger_threshold_m"]
            or axial > AXIAL_DANGER_M)


INSTRUCTION_KINDS = ("none", "ambiguous", "unambiguous")


def _finite_vector(value, n) -> bool:
    try:
        return len(value) == n and all(np.isfinite(float(v)) for v in value)
    except (TypeError, ValueError):
        return False


def _check_instance_truth(inst) -> None:
    """Every field the judgment reads must be present and finite."""
    if (not isinstance(inst, dict) or not isinstance(inst.get("eligible"), bool)
            or not isinstance(inst.get("layer"), int) or isinstance(inst.get("layer"), bool)
            or not isinstance(inst.get("faces"), list) or not inst["faces"]):
        raise ValueError("instance truth needs eligible, layer and at least one face")
    for face in inst["faces"]:
        fit = face.get("fit") if isinstance(face, dict) else None
        if (not isinstance(face, dict) or not isinstance(face.get("accessible"), bool)
                or not _finite_vector(face.get("left_m"), 3) or not _finite_vector(face.get("right_m"), 3)
                or not _finite_vector([face.get("insertion_yaw_rad")], 1)
                or not _finite_vector([face.get("danger_threshold_m")], 1)
                or not isinstance(fit, dict) or not isinstance(fit.get("pickable"), bool)
                or not isinstance(fit.get("reasons"), list)):
            raise ValueError("face truth needs accessible, finite pockets, yaw and danger threshold, and a fit")


def _check_truth(population, truth, instruction_input) -> None:
    """Refuse corrupt or inconsistent truth instead of scoring it as a clean scene."""
    if not isinstance(truth, dict) or not isinstance(truth.get("instances"), list):
        raise ValueError("truth must hold an instances list")
    itruth = truth.get("instruction_truth")
    if not isinstance(itruth, dict) or itruth.get("kind") not in INSTRUCTION_KINDS:
        raise ValueError("truth must hold instruction_truth with a known kind")
    for inst in truth["instances"]:
        _check_instance_truth(inst)
    if population != "negative" and not truth["instances"]:
        raise ValueError("a positive scene's truth must list at least one instance")
    present = bool(instruction_input.get("present"))
    if present != (itruth["kind"] != "none"):
        raise ValueError("instruction input and instruction truth disagree on whether there is an instruction")
    target = itruth.get("target")
    if itruth["kind"] != "none" and not (isinstance(target, int) and 0 <= target < len(truth["instances"])):
        raise ValueError("instruction truth target must index an instance")


def judge_scene(meta, truth, instruction_input, candidates: Sequence[CandidateView], decision: TargetDecision,
                *, elapsed_s=None) -> SceneVerdict:
    population = population_of(meta)
    _check_truth(population, truth, instruction_input)
    instances = truth["instances"]
    counted = [c for c in candidates if c.counted]
    pairs, costs = _match(counted, instances)
    matched_inst = {i: c for c, i in pairs.items()}
    errors = tuple(costs[c, i][0][1] for c, i in pairs.items())
    dangerous = False
    for ci, cand in enumerate(counted):
        if cand.pickable is not True:
            continue
        if ci in pairs:
            (_, err), face = costs[ci, pairs[ci]]
        else:
            options = [costs[ci, ii] for ii in range(len(instances)) if (ci, ii) in costs]
            if not options:
                dangerous = True
                continue
            (_, err), face = min(options, key=lambda o: o[0][0])
        dangerous = dangerous or _danger(err, face)
    negative = population == "negative"
    miss = None if negative else any(inst["eligible"] and k not in matched_inst for k, inst in enumerate(instances))
    target_error = deferred = fit_error = None
    itruth = truth["instruction_truth"]
    target = itruth.get("target")
    by_id = {c.candidate_id: k for k, c in enumerate(counted)}
    if not negative and itruth["kind"] == "ambiguous":
        target_error = decision.status == "selected"
    elif not negative and itruth["kind"] == "unambiguous":
        tgt = instances[target]
        tgt_pickable = any(f["accessible"] and f["fit"]["pickable"] for f in tgt["faces"])
        deferred = decision.status in ("ask_operator", "reobserve")
        if decision.status == "selected":
            k = by_id.get(decision.candidate_id)
            layer = instruction_input.get("layer")
            target_error = k is None or pairs.get(k) != target or (layer is not None and counted[k].layer_abs != layer)
        elif decision.status == "none":
            target_error = bool(tgt["eligible"])
        elif decision.status == "not_pickable":
            target_error = tgt_pickable
        else:
            target_error = False
    if population == "nonstandard" and instances:
        k_inst = 0 if target is None else target
        if k_inst in matched_inst:
            cand = counted[matched_inst[k_inst]]
            face = costs[matched_inst[k_inst], k_inst][1]
            fit_error = None if cand.pickable is None else cand.pickable != face["fit"]["pickable"]
    return SceneVerdict(
        meta["scene_id"], meta["case"], meta["shape_key"], meta["split"], population,
        meta.get("render_source", "rig"), meta.get("noise") is not None,
        miss, dangerous, len(pairs) < len(counted), bool(counted) if negative else None,
        target_error, deferred, fit_error, errors, meta["case"] in ("S4", "S7"), elapsed_s,
    )


def _gate(values, bound):
    values = [v for v in values if v is not None]
    n, k = len(values), sum(bool(v) for v in values)
    upper = cp_upper(k, n)
    return {"n": n, "failures": k, "upper": upper, "bound": bound,
            "allowed_failures": allowed_failures(n, bound) if n else None,
            "pass": upper is not None and upper <= bound}


def _p95_gate(samples, target):
    value = float(np.percentile(samples, 95)) if len(samples) else None
    return {"n": len(samples), "p95": value, "target": target, "pass": value is not None and value <= target}


def summarize(verdicts: Sequence[SceneVerdict], *, split: str, render_source: str) -> dict:
    chosen = [v for v in verdicts if v.split == split and v.render_source == render_source]
    gates, cells = {}, {}
    for pop, names in APPLICABLE.items():
        mine = [v for v in chosen if v.population == pop]
        for name in names:
            gates[f"{pop}:{name}"] = _gate([getattr(v, name) for v in mine], GATES[name])
        if pop != "negative":
            gates[f"{pop}:dangerous_observed_zero"] = {"n": len(mine), "failures": sum(v.dangerous for v in mine),
                                                       "pass": bool(mine) and not any(v.dangerous for v in mine)}
            errs = [e for v in mine for e in v.errors]
            for j, axis in enumerate(("lateral", "axial", "yaw")):
                gates[f"{pop}:pose_{axis}_p95"] = _p95_gate([e[j] for e in errs], POSE_P95[axis])
            gates[f"{pop}:pose_z_p95"] = _p95_gate([e[3] for v in mine if v.z_judged for e in v.errors], POSE_P95["z"])
        for case in sorted({v.case for v in mine}):
            cell = [v for v in mine if v.case == case]
            cells[f"{pop}:{case}"] = {name: _gate([getattr(v, name) for v in cell], GATES[name]) for name in names}
    return {"split": split, "render_source": render_source, "scenes": len(chosen),
            "official": render_source == "isaac" and bool(chosen) and all(v.noisy for v in chosen),
            "gates": gates, "cells": cells}
