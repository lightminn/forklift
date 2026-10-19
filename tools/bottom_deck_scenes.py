# tools/bottom_deck_scenes.py
"""Side-face scenes for the bottom-deck evidence (plan docs/plans/2026-10-19-bottom-deck-side-evidence.md).

One ``SideCase`` is one rendered side observation: a structure (the real pallet or a lookalike) seen from the
mission camera mount with its model +y or -y face at ``theta_e_deg`` off that face's outward normal, plus the
optional clutter the plan's review rounds asked for (coplanar neighbour, low plate in front, ledges in the gaps,
an occluding board, a raised or stacked support). The true camera can carry an extrinsic error while the scene
reports the nominal one, and the target handed to the evidence can carry a candidate pose error.
"""

import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from forklift_core.perception.bottom_deck_evidence import BottomDeckTarget
from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_model import PalletSpec, spec_boxes
from tools import instance_rig, scene_rig
from tools import measure_pocket_evidence as mpe
from tools import recognition_scenes as rs

ROOT = Path(__file__).resolve().parents[1]
NOISE = ROOT / "config/depth_noise_d435i_team.yaml"
TILT_RAD = 0.10                                   # carriage_low_measured mount
CAMERA = scene_rig.Camera((0.619, 0.0, 0.27), TILT_RAD)
STRUCTURES = ("pallet", "grounded", "shelf", "slab", "deckless")


@dataclass(frozen=True)
class Plate:
    """A low plate on the floor in front of the observed face (Codex v3/v4)."""

    height_m: float = 0.02
    length_m: float = 0.8
    width_m: float = 0.15
    near_m: float = 0.1        # from the face to the plate's near edge


@dataclass(frozen=True)
class Ledge:
    """A separate low object in one gap of the observed face (Codex v5/v6): not a pallet member."""

    gap: int                   # 0-based, along model +x
    height_m: float
    setback_m: float = 0.0     # behind the face
    depth_m: float = 0.05
    clearance_m: float = 0.003  # to the blocks on each side


@dataclass(frozen=True)
class SideCase:
    shape: str                       # "epal6" | "t11_06"
    structure: str = "pallet"
    face: int = 1                    # observed model y face, +1 or -1
    theta_e_deg: float = 35.0        # centre-to-camera direction off the face's outward normal
    range_m: float = 1.35            # camera to pallet centre (horizontal)
    neighbour: int = 0               # +1 / -1: a real pallet 50 mm beyond along model x (coplanar side)
    plate: Plate | None = None
    ledges: tuple = ()
    occluder: bool = False           # a board standing in front of the observed face from 30 mm up
    lift_m: float = 0.0              # the structure stands this high on the floor (support error)
    support_z_m: float = 0.0         # the support height the target claims
    stacked: bool = False            # the structure stands on a real pallet of the same shape
    tilt_err_deg: float = 0.0        # true camera tilt minus the nominal one
    height_err_m: float = 0.0
    along_err_m: float = 0.0         # candidate error along model x
    lateral_err_m: float = 0.0       # along model y
    yaw_err_deg: float = 0.0
    noise_seed: int | None = None    # team noise frame_640
    smooth: bool = False             # 5 x 5 median over valid depth (a stereo matcher's window)
    spec: PalletSpec | None = None   # render this spec's boxes instead of the shape's (pallet / grounded only)


def geometry(shape: str):
    return load_pallet_geometry(ROOT / f"config/pallet_geometry_{shape}.yaml")


def model_yaw_for(case: SideCase) -> float:
    """Pallet model-x yaw that shows face ``case.face`` at theta_e to a camera looking along base +x."""
    yaw = math.radians(90.0 - case.theta_e_deg)
    return yaw if case.face > 0 else yaw + math.pi


def _lift(boxes, dz):
    return [scene_rig.Box((b.centre_m[0], b.centre_m[1], b.centre_m[2] + dz), b.size_m, b.yaw_rad) for b in boxes]


def _median5(depth):
    """5 x 5 median with invalid pixels read as 0, kept invalid (the spikes' scipy median_filter, numpy only)."""
    d = np.asarray(depth, float)
    filled = np.where(np.isfinite(d), d, 0.0)
    pad = np.pad(filled, 2, mode="symmetric")   # scipy.ndimage median_filter's default "reflect"
    win = np.lib.stride_tricks.sliding_window_view(pad, (5, 5))
    med = np.median(win.reshape(*d.shape, 25), axis=-1)
    return np.where(np.isfinite(d), med, d)


def spec_structure(spec: PalletSpec, kind: str):
    """``pallet`` or ``grounded`` built from a spec (any row count)."""
    boxes = []
    for b in spec_boxes(spec):
        if kind == "grounded" and b.name.startswith("bottom_board"):
            continue
        if kind == "grounded" and b.name.startswith("block_"):
            top = spec.deck_bottom_m + spec.opening_height_m
            boxes.append(scene_rig.Box((b.centre_m[0], b.centre_m[1], top / 2), (b.size_m[0], b.size_m[1], top)))
            continue
        boxes.append(scene_rig.Box(tuple(b.centre_m), tuple(b.size_m)))
    if kind not in ("pallet", "grounded"):
        raise ValueError(f"spec structures are pallet / grounded, not {kind!r}")
    return boxes


def render_side(case: SideCase):
    """-> (SceneInput with the nominal extrinsic, BottomDeckTarget, RenderedScene)."""
    if case.structure not in STRUCTURES:
        raise ValueError(f"unknown structure {case.structure!r}")
    g = geometry(case.shape)
    myaw = model_yaw_for(case)
    cx = CAMERA.xyz_m[0] + case.range_m
    ex = np.array([math.cos(myaw), math.sin(myaw)])
    ey = np.array([-math.sin(myaw), math.cos(myaw)])
    centre = np.array([cx, 0.0])
    side = ey * case.face                         # outward normal of the observed face
    spec = rs.known_spec(case.shape) if case.spec is None else case.spec
    built = mpe.structure(g, case.structure) if case.spec is None else spec_structure(spec, case.structure)
    target_boxes = scene_rig.place(built, x_m=cx, y_m=0.0, yaw_rad=myaw)
    base = case.lift_m
    instances = []
    if case.stacked:
        instances.append(scene_rig.place(mpe.structure(g, "pallet"), x_m=cx, y_m=0.0, yaw_rad=myaw))
        base += g.overall_height_m
    instances.insert(0, _lift(target_boxes, base))
    if case.neighbour:
        step = (g.overall_depth_m + 0.05) * case.neighbour
        instances.append(scene_rig.place(mpe.structure(g, "pallet"), x_m=cx + step * ex[0], y_m=step * ex[1],
                                         yaw_rad=myaw))
    distractors = []
    half_w = spec.overall_width_m / 2
    if case.plate is not None:
        p = case.plate
        c = centre + side * (half_w + p.near_m + p.width_m / 2)
        distractors.append(scene_rig.Box((c[0], c[1], p.height_m / 2), (p.length_m, p.width_m, p.height_m), myaw))
    gap_w = (g.overall_depth_m - 3 * g.block_depth_m) / 2
    for ledge in case.ledges:
        x_mid = -g.overall_depth_m / 2 + g.block_depth_m + gap_w / 2 + ledge.gap * (g.block_depth_m + gap_w)
        c = centre + ex * x_mid + side * (half_w - ledge.setback_m - ledge.depth_m / 2)
        distractors.append(scene_rig.Box((c[0], c[1], ledge.height_m / 2),
                                         (gap_w - 2 * ledge.clearance_m, ledge.depth_m, ledge.height_m), myaw))
    if case.occluder:
        c = centre + side * (half_w + 0.08)
        distractors.append(scene_rig.Box((c[0], c[1], 0.03 + 0.1), (g.overall_depth_m, 0.02, 0.2), myaw))
    true_cam = scene_rig.Camera((CAMERA.xyz_m[0], 0.0, CAMERA.xyz_m[2] + case.height_err_m),
                                TILT_RAD + math.radians(case.tilt_err_deg))
    out = instance_rig.render_instances(instances, distractors=distractors, camera=true_cam, back_wall=False)
    scene = out.scene
    if case.noise_seed is not None:
        model = NoiseModel.from_yaml(NOISE, seed=case.noise_seed, target_fx=scene.intrinsics.fx, level="frame_640")
        scene, _ = apply_noise(scene, out.instance_ids, model)
    if case.smooth:
        scene = replace(scene, depth_m=_median5(scene.depth_m))
    scene = replace(scene, base_from_optical=CAMERA.base_from_optical())
    c_err = centre + ex * case.along_err_m + ey * case.lateral_err_m
    target = BottomDeckTarget(centre_xy=(float(c_err[0]), float(c_err[1])),
                              model_yaw_rad=myaw + math.radians(case.yaw_err_deg),
                              spec=spec, support_z_m=case.support_z_m)
    return scene, target, out
