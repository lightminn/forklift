#!/usr/bin/env python3
"""Approach sequences on the CPU rig for far-range tracking (stage 2B-3 plan section 6).

A recognition recipe (fresh seeds, disjoint from the catalogue) is the world. One
accessible entry face is the target; the robot drives along its normal from a
camera-face distance of 3.5 m to 1.3 m at 0.6 m/s, one frame every 0.2 s. Each
frame moves the scene into that robot pose, renders depth and instance ids with
`scene_truths` (truth in the true base frame), adds the team noise with its own
seed and fits every visible instance with oracle masks under per-frame random
proposal ids. Records keep the candidates, the scoring labels (instance, face)
and the true poses; odometry, dropouts and the tracker run later
(tools/evaluate_track_sequences.py), so V0/V1/V3/V5 share the straight renders.

Kinds: straight (V0, V1, V3, V5), curve (V2: 0.4 m lateral offset, 15 deg
heading, aligned at 1.3 m), pitch (V4: camera pitch noise sigma 0.3 deg).
"""

from __future__ import annotations

import argparse
import hashlib
import math
import pickle
import subprocess
import sys
from dataclasses import dataclass, replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.fork_fit import ForkSetup
from forklift_core.perception.instance_fit import fit_scene
from forklift_core.perception.instance_proposals import InstanceProposal
from tools import instance_rig, scene_rig
from tools import recognition_scenes as rs
from tools.evaluate_recognition import _known, _load_for
from tools.generate_recognition_scenes import load_site
from forklift_core.perception.pallet_model import face_layout
from tools.recognition_truth import Placement, scene_truths

ROOT = Path(__file__).resolve().parents[1]
NOISE_CONFIG = ROOT / "config/depth_noise_d435i_team.yaml"
# Seeds for sequence recipes: inside the eval_pool band, far from the catalogue (1,000,000-1,001,019).
SEQUENCE_BANDS = {"calibration": 1_500_000, "evaluation": 1_600_000,
                  "evaluation_b": 1_700_000, "evaluation_c": 1_800_000,
                  "evaluation_d": 1_900_000,
                  "evaluation_e": 1_950_000,
                  "evaluation_f": 1_970_000}  # the eval_pool band ends at 2,000,000  # 2B-3b re-judgement: the 2B-3 evaluation set was used for diagnosis
CELLS = tuple((c, s) for c in ("S1", "S2", "S3", "S4", "S5", "S6") for s in ("epal6", "t11_06")) + (("S7", "nonstandard"),)
CELL_STRIDE = 1_000  # x 13 cells stays inside the 100,000 gap between the two bands
START_M, END_M, STEP_M, PERIOD_S = 3.5, 1.3, 0.12, 0.2
CURVE_OFFSET_M, CURVE_HEADING_RAD = 0.4, math.radians(15.0)
PITCH_SIGMA_RAD = math.radians(0.3)
MIN_EXPOSURE = 0.5
WALL = scene_rig.Box((scene_rig.BACK_WALL_X_M + 0.05, 0.0, 1.5), (0.1, 40.0, 3.0))
KINDS = ("straight", "curve", "pitch")


@dataclass(frozen=True)
class Pose:
    """world_from_base, planar."""

    x: float
    y: float
    yaw: float

    def to_base(self, p):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = p[0] - self.x, p[1] - self.y
        return (c * dx + s * dy, -s * dx + c * dy)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def recipe_for(cell: tuple, band: str, index: int):
    case, shape = cell
    seed = SEQUENCE_BANDS[band] + CELLS.index(cell) * CELL_STRIDE + index
    return rs.recipe(case, shape, seed, index, "sequence")  # rank = index: every scene type appears (2B-3b)


def _loads(recipe, site):
    instr = recipe.instruction
    holder = instr.target if instr.target is not None else 0
    return [instr.load if (k == holder and instr.load is not None)
            else (site.get(recipe.shape_key) if inst.spec.known else None)
            for k, inst in enumerate(recipe.instances)]


def render(recipe, site, pose: Pose, *, tilt_rad: float = 0.0):
    """The recipe seen from `pose`: (truths, rendered) in that pose's base frame."""
    instances = []
    for inst, load in zip(recipe.instances, _loads(recipe, site)):
        x, y = pose.to_base((inst.placement.x_m, inst.placement.y_m))
        placement = Placement(x, y, _wrap(inst.placement.yaw_rad - pose.yaw), inst.placement.support_z_m)
        instances.append((inst.spec, placement, inst.damage, load))
    boxes = []
    for b in (*recipe.distractors, WALL):
        x, y = pose.to_base(b.centre_m[:2])
        boxes.append(scene_rig.Box((x, y, b.centre_m[2]), b.size_m, _wrap(b.yaw_rad - pose.yaw)))
    camera = scene_rig.Camera(instance_rig.PERCEPTION_CAMERA.xyz_m, tilt_rad)
    return scene_truths(instances, ForkSetup(recipe.fork_spacing_m), distractors=boxes,
                        layers=[i.layer for i in recipe.instances], camera=camera, back_wall=False)


def _inside_any(recipe, point) -> bool:
    boxes = [b for k in range(len(recipe.instances)) for b in rs.recipe_boxes(recipe, k)] + list(recipe.distractors) + [WALL]
    for b in boxes:
        c, s = math.cos(b.yaw_rad), math.sin(b.yaw_rad)
        dx, dy, dz = point[0] - b.centre_m[0], point[1] - b.centre_m[1], point[2] - b.centre_m[2]
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        if abs(lx) <= b.size_m[0] / 2 and abs(ly) <= b.size_m[1] / 2 and abs(dz) <= b.size_m[2] / 2:
            return True
    return False


def path(face, kind: str) -> list[Pose]:
    """Robot poses (world_from_base) for one approach to `face` (a world-frame FaceTruth)."""
    yaw = face.insertion_yaw_rad
    a = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-a[1], a[0]])
    mid = (np.asarray(face.left_m[:2]) + np.asarray(face.right_m[:2])) / 2
    cam_x = instance_rig.PERCEPTION_CAMERA.xyz_m[0]
    span = START_M - END_M
    steps = int(math.floor(span / STEP_M + 1e-9)) + 1
    poses = []
    for k in range(steps):
        s = k * STEP_M
        if kind == "curve":  # cubic Hermite: offset 0.4 m and slope -tan 15 deg at the start, 0 and 0 at the end
            u = s / span
            h00, h10 = 2 * u**3 - 3 * u**2 + 1, u**3 - 2 * u**2 + u
            d00, d10 = (6 * u**2 - 6 * u) / span, (3 * u**2 - 4 * u + 1)
            lat = h00 * CURVE_OFFSET_M + h10 * span * -math.tan(CURVE_HEADING_RAD)
            slope = d00 * CURVE_OFFSET_M + d10 * -math.tan(CURVE_HEADING_RAD)
        else:
            lat, slope = 0.0, 0.0
        heading = yaw + math.atan(slope)
        cam = mid - (START_M - s) * a + lat * n
        base = cam - cam_x * np.array([math.cos(heading), math.sin(heading)])
        poses.append(Pose(float(base[0]), float(base[1]), _wrap(heading)))
    return poses


def choose_target(recipe, site):
    """(instance, face index) candidates by exposure from the recipe's own camera, best first."""
    truths, _ = render(recipe, site, Pose(0.0, 0.0, 0.0))
    ranked = sorted(((f.exposure, -k, -i, k, i) for k, t in enumerate(truths) for i, f in enumerate(t.faces)
                     if f.accessible and f.exposure > 0), reverse=True)
    return [(k, i, truths[k].faces[i]) for *_, k, i in ranked]


def face_openings(recipe, pose: Pose) -> list:
    """Every opening centre of every face, per instance, in the base frame of `pose` -- the same face
    order and geometry as recognition_truth._faces (truth pockets are two of these)."""
    return face_geometry(recipe, pose)[0]


def face_geometry(recipe, pose: Pose) -> tuple:
    """(openings, centres): every face's opening centres and its face centre (z = the pocket band's
    middle), per instance, in the base frame of `pose`."""
    out, centres = [], []
    for inst in recipe.instances:
        spec, pl = inst.spec, inst.placement
        faces, mids = [], []
        for axis in spec.entry_axes:
            layout = face_layout(spec, axis)
            z0, z1 = layout.pocket_band_m
            z = pl.support_z_m + (z0 + z1) / 2
            base = pl.yaw_rad + (0.0 if axis == "x" else math.pi / 2)
            for sign in (1, -1):
                yaw = base + (0.0 if sign == 1 else math.pi)
                a = np.array([math.cos(yaw), math.sin(yaw)])
                n = np.array([-a[1], a[0]])
                centre = np.array([pl.x_m, pl.y_m]) - a * layout.depth_m / 2
                lats = sorted(sign * (lo + hi) / 2 for lo, hi in layout.opening_intervals_m)
                faces.append([(*pose.to_base(centre + n * lat), z) for lat in lats])
                mids.append((*pose.to_base(centre), z))
        out.append(faces)
        centres.append(mids)
    return out, centres


def _label(cand, proposal_instance: dict, truths, openings) -> tuple:
    """(instance, face index, pocket pair) -- the face within 0.35 rad whose openings take both pockets
    within 0.2 m (the evaluator's matching distance), the pair being the two openings' indices. Face and
    pair are None when no face of the candidate's instance takes it."""
    k = proposal_instance[cand.proposal_id]
    best = None
    for i, f in enumerate(truths[k].faces):
        if abs(_wrap(cand.insertion_yaw_rad - f.insertion_yaw_rad)) > 0.35:
            continue
        pair, total = [], 0.0
        for pocket in (cand.left.centre_m, cand.right.centre_m):
            d = [float(np.linalg.norm(np.subtract(pocket, o))) for o in openings[k][i]]
            j = int(np.argmin(d))
            pair.append(j)
            total += d[j]
            if d[j] > 0.2:
                break
        else:
            if best is None or total < best[0]:
                best = (total, i, tuple(pair))
    return (k, None, None) if best is None else (k, best[1], best[2])


def target_pair(face, openings) -> tuple:
    """The truth face's own pocket pair as opening indices."""
    return tuple(int(np.argmin([np.linalg.norm(np.subtract(p, o)) for o in openings])) for p in (face.left_m, face.right_m))


def fit_frame(recipe, rendered, noise_seed: int | None, level: str, rng):
    """Fit every visible instance with oracle masks; proposal ids are random per frame."""
    scene, ids = rendered.scene, rendered.instance_ids
    if noise_seed is not None:
        model = NoiseModel.from_yaml(NOISE_CONFIG, seed=noise_seed, target_fx=scene.intrinsics.fx, level=level)
        scene, ids = apply_noise(scene, ids, model)
    visible = sorted(int(v) for v in np.unique(ids) if v >= 0)
    labels = {f"p{int(rng.integers(10**12)):012d}": k for k in visible}
    proposals = [InstanceProposal(pid, ids == k, 1.0, "oracle") for pid, k in sorted(labels.items())]
    specs, site = _known()
    instr = {"present": recipe.instruction.kind != "none",
             "load": None if recipe.instruction.load is None else recipe.instruction.load.to_json()}
    fitted = fit_scene(scene, proposals, specs=specs, setup=ForkSetup(recipe.fork_spacing_m),
                       load_for=_load_for(instr, site))
    return fitted.candidates, labels


def sequence(recipe, kind: str, *, level: str, clean: bool, site=None):
    """One approach: a dict with the frames, or the reason it was skipped."""
    site = site or load_site()
    # separate streams: the noise and tilt draws must not depend on whether clean fits are recorded
    rng, id_rng, clean_rng = (np.random.default_rng(s) for s in
                              np.random.SeedSequence([recipe.seed, KINDS.index(kind), 0x2B3]).spawn(3))
    for k, i, face in choose_target(recipe, site):
        poses = path(face, kind)
        cam_x = instance_rig.PERCEPTION_CAMERA.xyz_m
        if any(_inside_any(recipe, (p.x + cam_x[0] * math.cos(p.yaw), p.y + cam_x[0] * math.sin(p.yaw), cam_x[2]))
               for p in poses):
            continue
        tilt = [float(rng.normal(0, PITCH_SIGMA_RAD)) if kind == "pitch" else 0.0 for _ in poses]
        truths, _ = render(recipe, site, poses[0], tilt_rad=tilt[0])
        if truths[k].faces[i].exposure < MIN_EXPOSURE:
            continue
        frames = []
        for j, pose in enumerate(poses):
            truths, rendered = render(recipe, site, pose, tilt_rad=tilt[j])
            openings, centres = face_geometry(recipe, pose)
            noise_seed = int(rng.integers(2**63))
            cands, labels = fit_frame(recipe, rendered, noise_seed, level, id_rng)
            frame = {
                "index": j, "t_s": j * PERIOD_S, "pose": (pose.x, pose.y, pose.yaw), "tilt_rad": tilt[j],
                "candidates": cands,
                "labels": {c.candidate_id: _label(c, labels, truths, openings) for c in cands},
                "openings": openings,
                "face_centres": centres,
                "faces": [[{"left_m": f.left_m, "right_m": f.right_m, "insertion_yaw_rad": f.insertion_yaw_rad,
                            "exposure": f.exposure} for f in t.faces] for t in truths],
                "support_z_m": [t.support_z_m for t in truths],
                "noise_seed": noise_seed,
                "truth_face": truths[k].faces[i],
            }
            if clean:
                clean_cands, clean_labels = fit_frame(recipe, rendered, None, level, clean_rng)
                frame["clean"] = clean_cands
                frame["clean_labels"] = {c.candidate_id: _label(c, clean_labels, truths, openings)
                                         for c in clean_cands}
            frames.append(frame)
        return {"scene_id": recipe.scene_id, "case": recipe.case, "shape_key": recipe.shape_key, "seed": recipe.seed,
                "kind": kind, "level": level, "frames": frames, "skipped": None,
                "target": (k, i, target_pair(frames[0]["truth_face"], frames[0]["openings"][k][i]))}
    return {"scene_id": recipe.scene_id, "case": recipe.case, "shape_key": recipe.shape_key, "seed": recipe.seed,
            "kind": kind, "level": level, "target": None, "frames": [], "skipped": "no_clear_visible_face"}


def provenance() -> dict:
    def sha(p):
        return hashlib.sha256((ROOT / p).read_bytes()).hexdigest()
    try:
        commit = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    except OSError:
        commit = None
    files = ["tools/recognition_sequences.py", "tools/recognition_truth.py", "tools/recognition_scenes.py",
             "src/forklift_core/perception/instance_fit.py", "config/depth_noise_d435i_team.yaml"]
    return {"commit": commit, "sha256": {f: sha(f) for f in files}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--band", choices=tuple(SEQUENCE_BANDS), required=True)
    ap.add_argument("--cell", type=int, required=True, help="index into CELLS")
    ap.add_argument("--index", type=int, required=True, help="recipe index inside the cell")
    ap.add_argument("--kinds", default="straight,curve,pitch")
    ap.add_argument("--level", default="frame_848")
    ap.add_argument("--clean", action="store_true", help="also fit the noise-free render (calibration)")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    recipe = recipe_for(CELLS[args.cell], args.band, args.index)
    site = load_site()
    args.out.mkdir(parents=True, exist_ok=True)
    for kind in args.kinds.split(","):
        rec = sequence(recipe, kind, level=args.level, clean=args.clean, site=site)
        rec["band"], rec["provenance"] = args.band, provenance()
        with open(args.out / f"{recipe.scene_id}_{kind}.pkl", "wb") as fh:
            pickle.dump(rec, fh)
        print(recipe.scene_id, kind, rec["skipped"] or f"{len(rec['frames'])} frames target {rec['target']}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
