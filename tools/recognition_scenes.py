# tools/recognition_scenes.py
"""Deterministic scene recipes for the recognition generality evaluation (design §5.1)."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

from forklift_core.perception.fork_fit import TEST_FORK_SPACING_M, LoadDeclaration
from forklift_core.perception.pallet_damage import DamageOp
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, PalletSpec, spec_from_geometry
from forklift_core.perception.recognition_stats import eval_count
from tools import measure_pocket_evidence, scene_rig
from tools.recognition_truth import Placement, instance_boxes, placed_boxes

ROOT = Path(__file__).resolve().parents[1]
CASES = ("S1", "S2", "S3", "S4", "S5", "S6", "S7", "N")
SEED_BANDS = {"eval_pool": 1_000_000, "train": 5_000_000}
BAND_WIDTH = 1_000_000
MAX_TRIES = 200
S7_VIOLATIONS = ("fork_spacing_adjust", "lift_alignment_needed", "support_unverified",
                 "payload_limit", "near_tracking_unavailable")


@lru_cache(maxsize=None)
def _geometry(shape_key: str):
    return load_pallet_geometry(ROOT / f"config/pallet_geometry_{shape_key}.yaml")


@lru_cache(maxsize=None)
def known_spec(shape_key: str) -> PalletSpec:
    return spec_from_geometry(_geometry(shape_key), entry_axes=KNOWN_ENTRY_AXES[shape_key])


@dataclass(frozen=True)
class InstanceRecipe:
    spec: PalletSpec
    placement: Placement
    damage: tuple
    layer: int


@dataclass(frozen=True)
class Instruction:
    kind: str
    target: int | None
    x_m: float
    y_m: float
    radius_m: float
    layer: int | None
    load: LoadDeclaration | None


@dataclass(frozen=True)
class SceneRecipe:
    scene_id: str
    case: str
    shape_key: str
    split: str
    rank: int
    seed: int
    fork_spacing_m: float
    touching: bool
    instances: tuple
    distractors: tuple
    instruction: Instruction


def recipe_boxes(recipe: SceneRecipe, k: int) -> list[scene_rig.Box]:
    inst = recipe.instances[k]
    return placed_boxes(instance_boxes(inst.spec, inst.damage), inst.placement)


def _clear(groups, candidate):
    return not any(scene_rig.boxes_interpenetrate(a, b) for g in groups for a in g for b in candidate)


def _base_pose(rng):
    return 0.75 + rng.uniform(1.1, 4.0), rng.uniform(-1.0, 1.0), rng.uniform(-math.radians(30), math.radians(30))


def _axes(yaw):
    a = np.array([math.cos(yaw), math.sin(yaw)])
    return a, np.array([-a[1], a[0]])


def _damage_ops(rng, spec) -> tuple:
    kind = str(rng.choice(["top_board_missing", "top_board_broken", "block_corner_chipped", "block_split"]))
    if kind == "top_board_missing":
        picks = sorted(int(i) for i in rng.choice(spec.top_board_count, size=int(rng.integers(1, 3)), replace=False))
        return tuple(DamageOp(kind, f"top_board_{i}", ()) for i in picks)
    if kind == "top_board_broken":
        return (DamageOp(kind, f"top_board_{int(rng.integers(spec.top_board_count))}",
                         (("fraction", float(rng.uniform(0.2, 0.6))), ("keep", float(rng.integers(2))))),)
    col = int(rng.integers(len(spec.column_widths_m)))
    if kind == "block_corner_chipped":
        return (DamageOp(kind, f"block_x0_y{col}", (("depth", float(rng.uniform(0.01, 0.04))),
                                                     ("width", float(rng.uniform(0.01, 0.04))),
                                                     ("side", float(rng.integers(2))))),)
    return (DamageOp(kind, f"block_x0_y{col}", (("gap", float(rng.uniform(0.005, 0.015))),)),)


def _s7_spec(rng, rank: int, index: int):
    """Returns (spec, spacing, load). Even ranks pickable, odd ranks violate one reason."""
    violation = None if rank % 2 == 0 else S7_VIOLATIONS[(rank // 2) % 5]
    four = violation is None and rank % 4 == 0
    s = 0.3725 if four else float(rng.choice([0.3725, 0.290]))
    if violation == "fork_spacing_adjust":
        w = float(rng.uniform(0.10, 0.18))
        c = float(rng.uniform(0.06, 0.20))
    else:
        w = float(rng.uniform(0.195, 0.2025)) if four else float(rng.uniform(0.195, min(0.28, s - 0.06)))
        c = s - w
    o = float(rng.uniform(0.06, 0.20))
    width = 2 * o + 2 * w + c
    if width < 0.60:
        o += (0.60 - width) / 2
        width = 0.60
    if four:
        inner = (c - 0.05) / 2
        widths = (o, inner, inner, o)
        left = -width / 2
        centres = (left + o / 2, left + o + w + inner / 2, left + o + w + inner + 0.05 + inner / 2, width / 2 - o / 2)
    else:
        widths = (o, c, o)
        centres = (-(c / 2 + w + o / 2), 0.0, c / 2 + w + o / 2)
    depth = float(rng.uniform(0.62, 1.20)) if violation == "support_unverified" else float(rng.uniform(0.50, 0.58))
    row = float(min(rng.uniform(0.08, 0.15), depth / 4))
    lift_case = violation == "lift_alignment_needed"
    deck = 0.010 if lift_case else float(rng.uniform(0.010, 0.025))
    opening = float(rng.uniform(0.045, 0.047)) if lift_case else float(rng.uniform(0.06, 0.12))
    board = float(rng.uniform(0.08, 0.14))
    spec = PalletSpec(
        name=f"nonstandard_{index}", known=False, overall_width_m=width, overall_depth_m=depth,
        deck_bottom_m=deck, opening_height_m=opening, stringer_m=float(rng.uniform(0.012, 0.025)),
        top_thickness_m=float(rng.uniform(0.012, 0.025)), column_centres_y_m=centres, column_widths_m=widths,
        row_centres_x_m=(-(depth - row) / 2, 0.0, (depth - row) / 2), row_depth_m=row,
        top_style="flat" if violation == "near_tracking_unavailable" else "boards",
        top_board_count=max(2, min(9, int(width / (board * 1.4)))), top_board_width_m=board, entry_axes=("x",),
    )
    if violation == "payload_limit":
        pallet, load_mass = float(rng.uniform(3.0, 5.0)), float(rng.uniform(7.5, 9.0))
    else:
        pallet, load_mass = float(rng.uniform(1.0, 4.0)), float(rng.uniform(0.0, 6.0))
    return spec, s, LoadDeclaration(pallet, load_mass, depth / 2, 0.0)


def _instruction_kind(rule: str, rank: int) -> str:
    if rule in ("S2", "S3", "S4"):
        return ("ambiguous", "ambiguous", "unambiguous", "unambiguous", "none")[rank % 5]
    if rule in ("S1", "S6"):
        return "none" if rank % 4 == 3 else "unambiguous"
    if rule == "S7":
        return "unambiguous"
    return "none"


def _place_known(rule, rng, spec, touching):
    x, y, yaw = _base_pose(rng)
    a, n = _axes(yaw)
    count = int(rng.integers(2, 4)) if rule in ("S2", "S3", "S4") else 1
    out = []
    for k in range(count):
        if rule == "S2":
            gap = 0.0 if touching else float(rng.uniform(0.01, 0.30))
            lateral = (k - (count - 1) / 2) * (spec.overall_width_m + gap)
            stagger = 0.0 if touching or rng.random() < 0.5 else float(rng.uniform(-0.15, 0.15))
            pos = np.array([x, y]) + a * stagger + n * lateral
            jitter = 0.0 if touching else float(rng.uniform(-0.087, 0.087))
            out.append((Placement(float(pos[0]), float(pos[1]), yaw + jitter, 0.0), 0))
        elif rule == "S3":
            behind = sum(float(rng.uniform(0.05, 1.0)) + spec.overall_depth_m for _ in range(k))
            side = float(rng.uniform(-0.2, 0.2)) if k else 0.0
            pos = np.array([x, y]) + a * behind + n * side
            out.append((Placement(float(pos[0]), float(pos[1]), yaw, 0.0), 0))
        elif rule == "S4":
            off = rng.uniform(-0.03, 0.03, size=2) if k else np.zeros(2)
            dyaw = float(rng.uniform(-0.052, 0.052)) if k else 0.0
            out.append((Placement(x + float(off[0]), y + float(off[1]), yaw + dyaw, k * spec.overall_height_m), k))
        else:
            out.append((Placement(x, y, yaw, 0.0), 0))
    return out


def _distractors_s6(rng, spec, base):
    x, y, yaw = base
    found = []
    for _ in range(int(rng.integers(2, 6))):
        kind = str(rng.choice(["rack", "crate", "box", "wall"]))
        dx = float(rng.uniform(0.3, 1.5)) * float(rng.choice([-1, 1]))
        dy = float(rng.uniform(0.6, 1.6)) * float(rng.choice([-1, 1]))
        if kind == "wall":
            a, _ = _axes(yaw)
            pos = np.array([x, y]) + a * (spec.overall_depth_m / 2 + float(rng.uniform(0.05, 0.6)) + 0.025)
            cand = [scene_rig.Box((float(pos[0]), float(pos[1]), 0.6), (0.05, 2.0, 1.2), yaw)]
        elif kind == "rack":
            cand = scene_rig.place(measure_pocket_evidence.structure(_geometry("epal6"), "shelf"),
                                   x_m=x + dx, y_m=y + dy, yaw_rad=yaw)
        else:
            size = (0.45, 0.66, 0.19) if kind == "crate" else (0.5, 0.5, 0.25)
            cand = [scene_rig.Box((x + dx, y + dy, size[2] / 2), size, float(rng.uniform(-math.pi, math.pi)))]
        found.append(cand)
    if rng.random() < 0.3:
        size = (spec.overall_depth_m, spec.overall_width_m, float(rng.uniform(0.2, 0.5)))
        found.append([scene_rig.Box((x, y, spec.overall_height_m + size[2] / 2), size, yaw)])
    return found


def _scene(case, shape_key, seed, rank):
    place_seq, instr_seq, _ = np.random.SeedSequence(seed).spawn(3)
    rng = np.random.default_rng(place_seq)
    rule = ("S1", "S2", "S3")[rank % 3] if case == "S5" else case
    touching = rule == "S2" and rank % 4 == 0
    for _ in range(MAX_TRIES):
        if case == "N":
            x, y, yaw = _base_pose(rng)
            kind = ("grounded", "shelf", "blocks", "crates")[rank % 4]
            if kind == "crates":
                boxes = [scene_rig.Box((x, y, 0.095 + 0.19 * i), (0.45, 0.66, 0.19), yaw)
                         for i in range(int(rng.integers(2, 5)))]
            else:
                boxes = scene_rig.place(measure_pocket_evidence.structure(_geometry(("epal6", "t11_06")[rank % 2]), kind),
                                        x_m=x, y_m=y, yaw_rad=yaw)
            return [], boxes, 0.3725, rule, touching, None, instr_seq
        load = None
        if case == "S7":
            spec, spacing, load = _s7_spec(rng, rank, seed)
            x, y, yaw = _base_pose(rng)
            places = [(Placement(x, y, yaw, 0.0), 0)]
        else:
            spec, spacing = known_spec(shape_key), TEST_FORK_SPACING_M[shape_key]
            places = _place_known(rule, rng, spec, touching)
        instances, groups, ok = [], [], True
        for k, (p, layer) in enumerate(places):
            damage = _damage_ops(rng, spec) if case == "S5" and k == 0 else ()
            boxes = placed_boxes(instance_boxes(spec, damage), p)
            if rule != "S4" and not _clear(groups, boxes):
                ok = False
                break
            instances.append(InstanceRecipe(spec, p, damage, layer))
            groups.append(boxes)
        if not ok:
            continue
        distractors = []
        if rule == "S4" and rng.random() < 0.5:
            top = instances[-1].placement
            size = tuple(float(v) for v in (rng.uniform(0.4, 0.6), rng.uniform(0.4, 0.6), rng.uniform(0.2, 0.4)))
            distractors = [scene_rig.Box((top.x_m, top.y_m, top.support_z_m + spec.overall_height_m + size[2] / 2),
                                         size, top.yaw_rad)]
        if case == "S6":
            p0 = instances[0].placement
            for cand in _distractors_s6(rng, spec, (p0.x_m, p0.y_m, p0.yaw_rad)):
                if _clear(groups + [distractors], cand):
                    distractors.extend(cand)
        return instances, distractors, spacing, rule, touching, load, instr_seq
    raise RuntimeError(f"{case} seed {seed}: no non-overlapping placement in {MAX_TRIES} tries")


def _instruction(rule, kind, instances, load, rng):
    if kind == "none" or not instances:
        return Instruction("none", None, 0.0, 0.0, 0.0, None, load)
    target = int(rng.integers(len(instances)))
    t = instances[target].placement
    others = [math.hypot(i.placement.x_m - t.x_m, i.placement.y_m - t.y_m) for k, i in enumerate(instances) if k != target]
    if rule == "S4":
        layer = instances[target].layer if kind == "unambiguous" else None
        return Instruction(kind, target, t.x_m, t.y_m, 0.3, layer, load)
    if kind == "ambiguous":
        return Instruction(kind, target, t.x_m, t.y_m, min(others) + 0.1, None, load)
    radius = 0.3 if not others else min(0.3, min(others) / 2)
    return Instruction(kind, target, t.x_m, t.y_m, radius, None, load)


def generate(case: str, shape_key: str, count: int, *, band: str, first_seed: int) -> list[SceneRecipe]:
    if case not in CASES:
        raise ValueError(f"unknown case {case!r}")
    start = SEED_BANDS[band]
    if first_seed < start or first_seed + count > start + BAND_WIDTH:
        raise ValueError(f"seeds {first_seed}+{count} leave the {band} band")
    order = np.random.default_rng(np.random.SeedSequence([first_seed, 0x5E1F])).permutation(count)
    eval_set = set(order[: eval_count(count)].tolist()) if band != "train" else set()
    seen, recipes = {"eval": 0, "dev": 0, "train": 0}, []
    for i in range(count):
        split = "train" if band == "train" else ("eval" if i in eval_set else "dev")
        rank = seen[split]
        seen[split] += 1
        seed = first_seed + i
        instances, distractors, spacing, rule, touching, load, instr_seq = _scene(case, shape_key, seed, rank)
        kind = _instruction_kind(rule, rank)
        if kind == "ambiguous" and len(instances) < 2:
            kind = "unambiguous"
        instruction = _instruction(rule, kind, instances, load, np.random.default_rng(instr_seq))
        recipes.append(SceneRecipe(f"{case.lower()}_{shape_key}_{seed}", case, shape_key, split, rank, seed,
                                   spacing, touching, tuple(instances), tuple(distractors), instruction))
    return recipes


def assert_disjoint(*lists) -> None:
    seen: dict[int, str] = {}
    for recipes in lists:
        for r in recipes:
            if r.seed in seen:
                raise ValueError(f"seed {r.seed} overlaps between {seen[r.seed]} and {r.scene_id}")
            seen[r.seed] = r.scene_id


def catalogue() -> list[SceneRecipe]:
    out, seed = [], SEED_BANDS["eval_pool"]
    for case in ("S1", "S2", "S3", "S4", "S5", "S6"):
        for shape in ("epal6", "t11_06"):
            out += generate(case, shape, 61, band="eval_pool", first_seed=seed)
            seed += 61
    out += generate("S7", "nonstandard", 194, band="eval_pool", first_seed=seed)
    seed += 194
    out += generate("N", "negative", 94, band="eval_pool", first_seed=seed)
    assert_disjoint(out)
    return out


def catalogue_digest(recipes) -> str:
    """sha256 over the recipes at 12 significant digits, to compare machines."""
    def norm(value):
        if isinstance(value, float):
            return float(f"{value:.12g}")
        if isinstance(value, (list, tuple)):
            return [norm(v) for v in value]
        if isinstance(value, dict):
            return {k: norm(v) for k, v in value.items()}
        return value
    payload = json.dumps([norm(asdict(r)) for r in recipes], sort_keys=True, default=str)
    return hashlib.sha256(payload.encode()).hexdigest()
