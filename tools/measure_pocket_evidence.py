"""Reproduce the pocket-evidence measurements from committed code.

Every table in the restructure plan dated 2026-09-14 came from throwaway
scratch scripts. Reproducing them retracted three findings outright, and four
more turned on definitions no document wrote down: which file a pose came from,
what a "lookalike" is geometrically, whether depth was quantised, where the
camera sat. This tool exists so a reviewer can regenerate a table instead of
rewriting the script that made it.

Each subcommand prints a table and the exact arguments that produced it. The
numbers are not frozen in tests -- the constants that must be frozen belong to
the detector's own tests. What is pinned here is the rig (tools/scene_rig.py).

Rules: ``--rule head`` measures the committed detector. ``variant-c`` is the
restructure's proposed gate and does not exist until Task 2 implements it; this
tool refuses to fake it, because monkey-patching the detector from a scratch
script is what silently produced three of the retracted tables.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import yaml

from forklift_core.perception import pocket_detector as detector
from forklift_core.perception.pallet_geometry import PalletGeometry, load_pallet_geometry
from forklift_core.perception.pallet_prior import PalletPrior, load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.scene_dataset import SceneInput
from tools import scene_rig
from tools.scene_rig import Box, Camera

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GEOMETRY = ROOT / "config/pallet_geometry_t11_06.yaml"
DEFAULT_PRIOR = ROOT / "config/pallet_prior_t11_06.yaml"
DEFAULT_PARAMS = ROOT / "config/detector_params_v1.yaml"
CATALOGUE = ROOT / "sim/gazebo/scenes/catalogue_v1.yaml"


# --------------------------------------------------------------------------
# Structures. These definitions are the point: three review rounds reported
# different false-positive rates because each invented its own lookalikes.
# --------------------------------------------------------------------------


def structure(geometry: PalletGeometry, kind: str) -> list[Box]:
    """Named structures, all sharing the pallet's face dimensions.

    ``pallet``    the real article.
    ``deckless``  the same, with the bottom boards removed. The blocks still
                  start at deck_bottom_m, so the openings keep their height but
                  nothing spans beneath them.
    ``grounded``  no bottom boards and the block columns extended to the floor.
                  This is a rack or a stack of crates, and it is the structure
                  the detector cannot currently tell from a pallet.
    ``shelf``     front-open: full-depth columns to the floor and a solid top
                  deck, with no stringers or bottom boards.
    ``blocks``    nine free-standing blocks, no decks at all.
    ``slab``      the pallet with its bottom boards replaced by one continuous
                  slab spanning the full width, so the openings are closed
                  underneath. Real EPAL and T11 pallets are open to the floor;
                  this is the shape the frozen detector was tuned on, and it is
                  the only one of these that supplies lower-deck evidence.
    """
    boxes = scene_rig.pallet(geometry)
    if kind == "pallet":
        return boxes
    if kind == "deckless":
        return [b for b in boxes if not _is_bottom_board(b, geometry)]
    if kind == "grounded":
        kept = []
        for box in boxes:
            if _is_bottom_board(box, geometry):
                continue
            if _is_block(box, geometry):
                depth, width, _ = box.size_m
                cx, cy, _ = box.centre_m
                top = geometry.deck_bottom_m + geometry.block_height_m
                kept.append(Box((cx, cy, top / 2), (depth, width, top)))
            else:
                kept.append(box)
        return kept
    if kind == "shelf":
        top = geometry.deck_bottom_m + geometry.block_height_m
        columns = [
            Box((0.0, y, top / 2), (geometry.overall_depth_m, width, top))
            for y, width in zip(
                geometry.block_centres_y_m(), geometry.block_widths_m, strict=True
            )
        ]
        deck_z = geometry.overall_height_m - geometry.top_board_thickness_m / 2
        deck = Box(
            (0.0, 0.0, deck_z),
            (
                geometry.overall_depth_m,
                geometry.overall_width_m,
                geometry.top_board_thickness_m,
            ),
        )
        return columns + [deck]
    if kind == "blocks":
        return [b for b in boxes if _is_block(b, geometry)]
    if kind == "slab":
        kept = [b for b in boxes if not _is_bottom_board(b, geometry)]
        kept.append(
            Box(
                (0.0, 0.0, geometry.deck_bottom_m / 2),
                (
                    geometry.overall_depth_m,
                    geometry.overall_width_m,
                    geometry.deck_bottom_m,
                ),
            )
        )
        return kept
    raise ValueError(f"unknown structure: {kind}")


STRUCTURES = ("pallet", "deckless", "grounded", "shelf", "blocks", "slab")


def _is_bottom_board(box: Box, geometry: PalletGeometry) -> bool:
    return math.isclose(box.size_m[2], geometry.deck_bottom_m) and math.isclose(
        box.centre_m[2], geometry.deck_bottom_m / 2
    )


def _is_block(box: Box, geometry: PalletGeometry) -> bool:
    return math.isclose(box.size_m[2], geometry.block_height_m) and math.isclose(
        box.centre_m[2], geometry.opening_centre_height_m
    )


# --------------------------------------------------------------------------
# Running the detector
# --------------------------------------------------------------------------


def detect(
    scene: SceneInput, prior: PalletPrior, params: DetectorParams, rule: str
):
    if rule == "head":
        return detect_pockets(scene, prior, params)
    raise NotImplementedError(
        "rule 'variant-c' does not exist in the detector yet; Task 2 implements it. "
        "This tool will not monkey-patch the detector to imitate it."
    )


def seed_pass(
    scene: SceneInput,
    prior: PalletPrior,
    params: DetectorParams,
    rule: str,
    seeds: int,
    truth: np.ndarray | None = None,
    render_for_seed=None,
) -> tuple[int, float, str | None]:
    """Return (valid seeds, worst position error over valid seeds, top reason).

    ``render_for_seed(seed)`` re-renders the scene per trial.  It is needed only when
    the rig adds noise: without it every trial would share one noise draw, so the
    result would measure RANSAC variance and not the sensor's.  With noise off the
    scene is reused and the figures are byte-identical to before.
    """
    valid = 0
    worst = 0.0
    reasons: dict[str, int] = {}
    for seed in range(seeds):
        trial = render_for_seed(seed) if render_for_seed is not None else scene
        result = detect(trial, prior, dataclasses.replace(params, seed=seed), rule)
        observation = result.observation
        if observation.status == "valid":
            valid += 1
            if truth is not None:
                pockets = (observation.left.center_m, observation.right.center_m)
                worst = max(worst, scene_rig.position_error_m(pockets, truth))
        else:
            key = f"{observation.status}/{observation.reason}"
            reasons[key] = reasons.get(key, 0) + 1
    top = max(reasons, key=reasons.__getitem__) if reasons else None
    return valid, worst, top


# --------------------------------------------------------------------------
# Shared argument plumbing
# --------------------------------------------------------------------------


def add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--geometry", type=Path, default=DEFAULT_GEOMETRY)
    parser.add_argument("--prior", type=Path, default=DEFAULT_PRIOR)
    parser.add_argument("--params", type=Path, default=DEFAULT_PARAMS)
    parser.add_argument("--rule", choices=("head", "variant-c"), default="head")
    parser.add_argument(
        "--seeds",
        type=int,
        default=12,
        help="RANSAC seeds per cell; boundary cells are not binary (default: 12)",
    )
    parser.add_argument("--camera-x", type=float, default=0.75)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument(
        "--camera-z",
        type=float,
        default=0.5,
        help="Camera height. The usable range is a function of this, not of any gate",
    )
    parser.add_argument(
        "--camera-tilt", type=float, default=0.0, help="Radians, positive downwards"
    )
    parser.add_argument(
        "--noise-k",
        type=float,
        default=0.0,
        help="Axial depth noise sigma(z) = k * z^2, applied before quantisation. "
        "0 (default) keeps every committed figure reproducing; 0.002 is 2 mm at 1 m.",
    )
    parser.add_argument("--noise-seed", type=int, default=0)
    parser.add_argument(
        "--no-quantize",
        dest="quantize",
        action="store_false",
        help="Render unrounded depth; the default rounds to 1 mm as the fixture does",
    )
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                        help="Override a detector parameter, e.g. --set floor_z_m=0.005")


def load(args) -> tuple[PalletGeometry, PalletPrior, DetectorParams, Camera]:
    geometry = load_pallet_geometry(args.geometry)
    prior = load_pallet_prior(args.prior)
    data = yaml.safe_load(args.params.read_text())
    for item in args.set:
        name, _, value = item.partition("=")
        if name not in data:
            raise SystemExit(f"unknown detector parameter: {name}")
        data[name] = type(data[name])(value) if not isinstance(data[name], bool) else value
    params = DetectorParams(**data)
    camera = Camera((args.camera_x, args.camera_y, args.camera_z), args.camera_tilt)
    return geometry, prior, params, camera


def header(args, extra: str = "") -> None:
    cam = f"camera=({args.camera_x}, {args.camera_y}, {args.camera_z}) tilt={args.camera_tilt}"
    over = " ".join(args.set) if args.set else "-"
    print(
        f"# rule={args.rule} seeds={args.seeds} quantize={args.quantize} {cam}\n"
        f"# geometry={args.geometry.name} prior={args.prior.name} overrides={over}"
        + (f"\n# {extra}" if extra else "")
    )


def arange(spec: str) -> list[float]:
    """``start:stop:step`` inclusive of stop within a rounding tolerance."""
    start, stop, step = (float(x) for x in spec.split(":"))
    n = int(round((stop - start) / step))
    return [round(start + i * step, 6) for i in range(n + 1)]


# --------------------------------------------------------------------------
# Subcommands
# --------------------------------------------------------------------------


def cmd_grid(args) -> int:
    geometry, prior, params, camera = load(args)
    header(args, f"structure={args.structure}")
    boxes = structure(geometry, args.structure)
    distances = arange(args.distances)
    print(f"{'x_m':>7s} {'y_m':>7s} {'yaw':>6s} {'valid':>7s} {'worst_mm':>9s}  reason")
    dead = partial = full = 0
    for x in distances:
        for y in (float(v) for v in args.lateral.split(",")):
            for yaw in (float(v) for v in args.yaw.split(",")):
                scene = scene_rig.render(
                    scene_rig.place(boxes, x_m=x, y_m=y, yaw_rad=yaw),
                    camera=camera,
                    quantize=args.quantize,
                    noise_k=args.noise_k,
                    noise_seed=args.noise_seed,
                )
                truth = scene_rig.true_pockets(
                    geometry, x_m=x, y_m=y, yaw_rad=yaw
                )
                placed = scene_rig.place(boxes, x_m=x, y_m=y, yaw_rad=yaw)
                per_seed = (
                    (lambda s: scene_rig.render(
                        placed,
                        camera=camera,
                        quantize=args.quantize,
                        noise_k=args.noise_k,
                        noise_seed=args.noise_seed + s,
                    ))
                    if args.noise_k
                    else None
                )
                valid, worst, reason = seed_pass(
                    scene, prior, params, args.rule, args.seeds, truth, per_seed
                )
                if valid == 0:
                    dead += 1
                elif valid == args.seeds:
                    full += 1
                else:
                    partial += 1
                print(
                    f"{x:7.3f} {y:7.3f} {yaw:6.3f} {valid:4d}/{args.seeds:<2d}"
                    f" {worst * 1000:9.1f}  {reason or ''}"
                )
    print(f"# cells: all-seed {full}, partial {partial}, dead {dead}")
    return 0


def cmd_evidence(args) -> int:
    """Dump the gate's own evidence for one scene.

    The plan went two rounds without noticing that ``lower`` is structurally
    zero on a floor-through pallet, because no table ever printed it.
    """
    geometry, prior, params, camera = load(args)
    header(args, f"structure={args.structure} x={args.x} y={args.y} yaw={args.yaw}")
    if args.seed is not None:
        params = dataclasses.replace(params, seed=args.seed)
    if args.distances:
        return _evidence_sweep(args, geometry, prior, params, camera)
    scene = scene_rig.render(
        scene_rig.place(
            structure(geometry, args.structure), x_m=args.x, y_m=args.y, yaw_rad=args.yaw
        ),
        camera=camera,
        quantize=args.quantize,
        noise_k=args.noise_k,
        noise_seed=args.noise_seed,
    )
    points, _ = detector._base_points(scene)
    cam = scene.base_from_optical.translation_m
    workspace = detector._filter_workspace(points, cam, prior, params)
    planes = detector._vertical_plane_candidates(workspace, cam, params)
    print(f"workspace points: {len(workspace)}   plane candidates: {len(planes)}")
    for index, plane in enumerate(planes):
        lateral = plane.points @ plane.left_axis
        local = np.column_stack(
            (np.zeros(len(lateral)), lateral, plane.points[:, 2])
        )
        counts, origin = detector._column_grid(local, prior, params)
        gaps = detector._gap_runs(counts > 0)
        distance = float(np.linalg.norm((plane.point - cam)[:2]))
        print(
            f"\nplane {index}: inliers {len(plane.points)}"
            f" residual_p95 {plane.residual_p95_m * 1000:.2f} mm"
            f" distance {distance:.3f} m  occupied cells {int((counts > 0).sum())}"
            f" gaps {len(gaps)}"
        )
        depth = -(workspace - plane.point) @ plane.normal
        band = (
            np.abs(workspace[:, 2] - prior.deck_bottom_m) <= params.deck_evidence_tol_m
        ) & (depth >= 0) & (depth <= prior.overall_depth_m + params.plane_inlier_m)
        workspace_lateral = workspace @ plane.left_axis
        for first, second in zip(gaps, gaps[1:], strict=False):
            spacer = (second[0] - first[1]) * params.cell_m
            supports = (
                int(counts[: first[0]].sum()),
                int(counts[first[1] : second[0]].sum()),
                int(counts[second[1] :].sum()),
            )
            left_edge = (origin + first[0]) * params.cell_m
            right_edge = (origin + second[1]) * params.cell_m
            lower = int(
                np.count_nonzero(
                    band
                    & (workspace_lateral >= left_edge)
                    & (workspace_lateral <= right_edge)
                )
            )
            over = (lateral >= left_edge) & (lateral <= right_edge)
            upper = int(
                np.count_nonzero(
                    over & (local[:, 2] >= prior.height_m - prior.deck_top_m)
                )
            )
            gate = min(*supports, lower, upper)
            print(
                f"  pattern spacer {spacer * 1000:6.1f} mm  supports {supports}"
                f"  lower {lower:6d}  upper {upper:6d}"
                f"  gate min {gate:6d} vs min_band_points {params.min_band_points}"
                f"  -> {'pass' if gate >= params.min_band_points else 'REJECT'}"
            )
    observation = detect(scene, prior, params, args.rule).observation
    print(f"\nobservation: {observation.status} / {observation.reason}")
    return 0


def _evidence_sweep(args, geometry, prior, params, camera) -> int:
    """One row per distance: the gate's terms for the best plane's best pattern.

    The lower count is taken over the selected pattern's lateral span, which is
    the definition the plan's C-10 table uses. Counting over the whole
    workspace instead gives numbers several times larger and is not comparable.
    """
    boxes = structure(geometry, args.structure)
    print(
        f"{'x_m':>7s} {'planes':>7s} {'supports (l,c,r)':>20s} {'lower':>7s}"
        f" {'u_left':>7s} {'u_right':>8s}  observation"
    )
    for x in arange(args.distances):
        scene = scene_rig.render(
            scene_rig.place(boxes, x_m=x, y_m=args.y, yaw_rad=args.yaw),
            camera=camera,
            quantize=args.quantize,
            noise_k=args.noise_k,
            noise_seed=args.noise_seed,
        )
        points, _ = detector._base_points(scene)
        cam = scene.base_from_optical.translation_m
        workspace = detector._filter_workspace(points, cam, prior, params)
        planes = detector._vertical_plane_candidates(workspace, cam, params)
        terms = _gate_terms(planes[0], workspace, prior, params) if planes else None
        observation = detect(scene, prior, params, args.rule).observation
        if terms is None:
            print(f"{x:7.3f} {0:7d} {'-':>20s} {'-':>7s} {'-':>7s} {'-':>8s}"
                  f"  {observation.status}/{observation.reason}")
            continue
        supports, lower, upper_left, upper_right = terms
        print(
            f"{x:7.3f} {len(planes):7d} {str(supports):>20s} {lower:7d}"
            f" {upper_left:7d} {upper_right:8d}"
            f"  {observation.status}/{observation.reason}"
        )
    return 0


def _gate_terms(plane, workspace, prior, params):
    """(supports, lower, upper_left, upper_right) for the plane's first pattern."""
    lateral = plane.points @ plane.left_axis
    local = np.column_stack((np.zeros(len(lateral)), lateral, plane.points[:, 2]))
    counts, origin = detector._column_grid(local, prior, params)
    gaps = detector._gap_runs(counts > 0)
    if len(gaps) < 2:
        return None
    first, second = gaps[0], gaps[1]
    supports = (
        int(counts[: first[0]].sum()),
        int(counts[first[1] : second[0]].sum()),
        int(counts[second[1] :].sum()),
    )
    left_edge = (origin + first[0]) * params.cell_m
    right_edge = (origin + second[1]) * params.cell_m
    depth = -(workspace - plane.point) @ plane.normal
    workspace_lateral = workspace @ plane.left_axis
    lower = int(
        np.count_nonzero(
            (np.abs(workspace[:, 2] - prior.deck_bottom_m) <= params.deck_evidence_tol_m)
            & (depth >= 0)
            & (depth <= prior.overall_depth_m + params.plane_inlier_m)
            & (workspace_lateral >= left_edge)
            & (workspace_lateral <= right_edge)
        )
    )
    band = (local[:, 2] >= prior.height_m - prior.deck_top_m) & (
        local[:, 2] <= prior.height_m + params.plane_inlier_m
    )
    per = []
    for start, stop in (first, second):
        low = (origin + start) * params.cell_m
        high = (origin + stop) * params.cell_m
        per.append(int(np.count_nonzero(band & (lateral >= low) & (lateral <= high))))
    return supports, lower, per[1], per[0]


def cmd_planes(args) -> int:
    """Plane candidates: how many survive, by how much, and how far they overlap.

    Repairing plane extraction made a confirmed plane collect its inliers from
    the whole cloud rather than from what earlier candidates left. That means
    two candidates can now share points, and a structure beside the pallet on
    the same plane joins its column grid. Neither the overlap nor the residual
    margin was observable before.
    """
    geometry, prior, params, camera = load(args)
    header(args, f"structure={args.structure}")
    boxes = structure(geometry, args.structure)
    print(
        f"{'x_m':>7s} {'cands':>6s} {'idx':>4s} {'inliers':>8s} {'resid_mm':>9s}"
        f" {'margin_mm':>10s} {'dist_m':>7s} {'cells':>6s} {'gaps':>5s} {'overlap':>8s}"
    )
    for x in arange(args.distances):
        scene = scene_rig.render(
            scene_rig.place(boxes, x_m=x, y_m=args.y, yaw_rad=args.yaw),
            camera=camera,
            quantize=args.quantize,
            noise_k=args.noise_k,
            noise_seed=args.noise_seed,
        )
        points, _ = detector._base_points(scene)
        cam = scene.base_from_optical.translation_m
        workspace = detector._filter_workspace(points, cam, prior, params)
        planes = detector._vertical_plane_candidates(workspace, cam, params)
        if not planes:
            print(f"{x:7.3f} {0:6d}   -- no vertical plane candidate --")
            continue
        keys = [{tuple(np.round(p, 6)) for p in plane.points} for plane in planes]
        for index, plane in enumerate(planes):
            lateral = plane.points @ plane.left_axis
            local = np.column_stack(
                (np.zeros(len(lateral)), lateral, plane.points[:, 2])
            )
            counts, _ = detector._column_grid(local, prior, params)
            gaps = detector._gap_runs(counts > 0)
            margin = params.max_plane_residual_m - plane.residual_p95_m
            # Worst pairwise share of this candidate's own inliers.
            overlap = 0.0
            for other in range(len(planes)):
                if other == index or not keys[index]:
                    continue
                overlap = max(
                    overlap, len(keys[index] & keys[other]) / len(keys[index])
                )
            distance = float(np.linalg.norm((plane.point - cam)[:2]))
            print(
                f"{x:7.3f} {len(planes):6d} {index:4d} {len(plane.points):8d}"
                f" {plane.residual_p95_m * 1000:9.2f} {margin * 1000:10.2f}"
                f" {distance:7.3f} {int((counts > 0).sum()):6d} {len(gaps):5d}"
                f" {overlap:8.3f}"
            )
    return 0


def cmd_zcut(args) -> int:
    """An overhead obstruction lowered until it hides the upper deck.

    Two traps, both paid for once already. The shadow height depends on the
    pallet's own depth, so it is computed from overall_depth_m rather than the
    0.300 m that suited EPAL 6 and puts the shadow in the wrong place on a
    660 mm article. And the obstruction is placed by its FRONT face: centring
    it on the nominal x puts its face half a box further forward, where it eats
    the support columns and every count collapses to zero for the wrong reason.
    """
    geometry, prior, params, camera = load(args)
    front_x = args.front_x
    # The pallet's approach face, which is what the shadow is measured against.
    pallet_front_x = args.x - geometry.overall_depth_m / 2
    cam_x, _, cam_z = camera.xyz_m
    lever = (front_x - cam_x) / (pallet_front_x - cam_x)
    header(
        args,
        f"structure={args.structure}; obstruction FRONT FACE at x={front_x} m,"
        f" depth {args.obstruction_depth} m; pallet face at"
        f" x={pallet_front_x:.3f} m (half-depth {geometry.overall_depth_m / 2:.3f} m);"
        f" beam bottom = {cam_z:.3f} + (z_cut - {cam_z:.3f}) * {lever:.4f}",
    )
    boxes = structure(geometry, args.structure)
    print(
        f"{'z_cut_m':>8s} {'beam_z_m':>9s} {'supports (l,c,r)':>20s} {'lower':>7s}"
        f" {'u_left':>7s} {'u_right':>8s}  observation"
    )
    for z_cut in arange(args.cuts):
        # z_cut is the shadow height ON THE PALLET FACE, not the beam's own
        # height. Sweeping the beam directly is meaningless: a beam low enough
        # to shadow a 100 mm deck sits below the camera and hides everything.
        beam_z = cam_z + (z_cut - cam_z) * lever
        obstruction = Box(
            (
                front_x + args.obstruction_depth / 2,
                args.y,
                beam_z + args.obstruction_height / 2,
            ),
            (args.obstruction_depth, args.obstruction_width, args.obstruction_height),
        )
        placed = scene_rig.place(
            boxes, x_m=args.x, y_m=args.y, yaw_rad=args.yaw
        ) + [obstruction]
        scene = scene_rig.render(placed, camera=camera, quantize=args.quantize, noise_k=args.noise_k, noise_seed=args.noise_seed)
        points, _ = detector._base_points(scene)
        cam = scene.base_from_optical.translation_m
        workspace = detector._filter_workspace(points, cam, prior, params)
        planes = detector._vertical_plane_candidates(workspace, cam, params)
        observation = detect(scene, prior, params, args.rule).observation
        terms = _gate_terms(planes[0], workspace, prior, params) if planes else None
        if terms is None:
            print(
                f"{z_cut:8.4f} {beam_z:9.4f} {'-':>20s} {'-':>7s} {'-':>7s} {'-':>8s}"
                f"  {observation.status}/{observation.reason}"
            )
            continue
        supports, lower, upper_left, upper_right = terms
        print(
            f"{z_cut:8.4f} {beam_z:9.4f} {str(supports):>20s} {lower:7d}"
            f" {upper_left:7d} {upper_right:8d}"
            f"  {observation.status}/{observation.reason}"
        )
    return 0


def cmd_structures(args) -> int:
    """Every structure at the same distances, so a table says which is which."""
    geometry, prior, params, camera = load(args)
    header(args)
    distances = arange(args.distances)
    names = args.structures.split(",")
    print(f"{'x_m':>7s} " + " ".join(f"{n:>10s}" for n in names))
    for x in distances:
        row = []
        for name in names:
            scene = scene_rig.render(
                scene_rig.place(structure(geometry, name), x_m=x),
                camera=camera,
                quantize=args.quantize,
                noise_k=args.noise_k,
                noise_seed=args.noise_seed,
            )
            valid, _, _ = seed_pass(scene, prior, params, args.rule, args.seeds)
            row.append(f"{valid:4d}/{args.seeds:<2d}".rjust(10))
        print(f"{x:7.3f} " + " ".join(row))
    return 0


def cmd_fov(args) -> int:
    """Where the approach face falls out of the image, in closed form and measured.

    The detector's near limit is not a gate. It is the last image row: the
    column band drops below it before any threshold is reached.
    """
    geometry, prior, params, camera = load(args)
    header(args)
    spec = scene_rig.intrinsics()
    half_v = (spec.height / 2) / spec.fy
    half_h = (spec.width / 2) / spec.fx
    z_low = geometry.deck_bottom_m
    z_high = geometry.deck_bottom_m + geometry.block_height_m
    print(f"# vertical half-angle tan = {half_v:.5f}, horizontal = {half_h:.5f}")
    for name, z in (("band bottom", z_low), ("band top", z_high)):
        reach = (camera.xyz_m[2] - z) / half_v
        print(
            f"# {name} z={z * 1000:5.1f} mm visible from range {reach:.3f} m"
            f"  -> placement x >= {reach + geometry.overall_depth_m / 2 + camera.xyz_m[0]:.3f} m"
        )
    half_width = geometry.overall_width_m / 2
    reach_h = half_width / half_h
    print(
        f"# full width visible from range {reach_h:.3f} m"
        f"  -> placement x >= {reach_h + geometry.overall_depth_m / 2 + camera.xyz_m[0]:.3f} m"
    )
    boxes = structure(geometry, args.structure)
    print(f"\n{'x_m':>7s} {'valid':>7s} {'band_px':>9s} {'deck_px':>9s}  reason")
    for x in arange(args.distances):
        scene = scene_rig.render(
            scene_rig.place(boxes, x_m=x), camera=camera, quantize=args.quantize,
            noise_k=args.noise_k,
            noise_seed=args.noise_seed,
        )
        points, _ = detector._base_points(scene)
        finite = points[np.isfinite(points).all(axis=1)]
        near = finite[np.abs(finite[:, 1]) <= half_width + 0.05]
        near = near[np.abs(near[:, 0] - (x - geometry.overall_depth_m / 2)) <= 0.7]
        band = int(((near[:, 2] >= z_low) & (near[:, 2] <= z_high)).sum())
        deck = int((near[:, 2] > z_high).sum())
        valid, _, reason = seed_pass(scene, prior, params, args.rule, args.seeds)
        print(f"{x:7.3f} {valid:4d}/{args.seeds:<2d} {band:9d} {deck:9d}  {reason or ''}")
    return 0


def cmd_noise(args) -> int:
    """Axial depth noise with per-pixel sigma = k*d^2, then the rig's quantisation.

    The model is stated here because the plan's noise table was not reproducible
    without it: sigma is per pixel, applied to depth_m, before quantisation.
    """
    geometry, prior, params, camera = load(args)
    header(args, f"sigma = k*d^2 with k set by --sigma-at (metres at {args.sigma_at} m)")
    boxes = structure(geometry, args.structure)
    sigmas = [float(v) for v in args.sigmas.split(",")]
    print(f"{'x_m':>7s} " + " ".join(f"{s * 1000:>8.0f}mm" for s in sigmas))
    for x in arange(args.distances):
        base = scene_rig.render(
            scene_rig.place(boxes, x_m=x), camera=camera, quantize=False
        )
        row = []
        for sigma in sigmas:
            k = sigma / (args.sigma_at**2)
            valid = 0
            for seed in range(args.seeds):
                depth = np.asarray(base.depth_m, dtype=float).copy()
                mask = np.isfinite(depth)
                rng = np.random.default_rng(7919 * seed + 13)
                depth[mask] += rng.normal(0.0, 1.0, depth[mask].shape) * (
                    k * depth[mask] ** 2
                )
                scene = dataclasses.replace(base, depth_m=depth)
                if args.quantize:
                    scene = scene_rig.quantized(scene)
                observation = detect(
                    scene, prior, dataclasses.replace(params, seed=seed), args.rule
                ).observation
                valid += observation.status == "valid"
            row.append(f"{valid:4d}/{args.seeds:<2d}".rjust(10))
        print(f"{x:7.3f} " + " ".join(row))
    return 0


def cmd_poses(args) -> int:
    """The committed catalogue poses, retargeted onto this shape.

    The pose source is an argument because the repository holds two that
    disagree: the generator's own placement in the scene catalogue, and the
    pocket midpoints in each scene's ground truth, which sit half a pallet
    depth behind the face. Reading the wrong one moves the near/far split
    from 17/43 to 29/31.
    """
    geometry, prior, params, camera = load(args)
    header(args, f"poses={args.catalogue.name} category={args.category}")
    catalogue = yaml.safe_load(args.catalogue.read_text())
    poses = [
        (s["id"] if "id" in s else s.get("scene_id", "?"), s["pallet"])
        for s in catalogue["scenes"]
        if s["category"] == args.category
    ]
    boxes = structure(geometry, args.structure)
    near = far = 0
    near_ok = far_ok = 0
    worst_accepted = 0.0
    print(f"{'scene':>8s} {'x_m':>7s} {'y_m':>7s} {'yaw':>6s} {'valid':>7s} {'worst_mm':>9s}")
    for name, pose in poses:
        x, y, yaw = pose["x_m"], pose["y_m"], pose["yaw_rad"]
        scene = scene_rig.render(
            scene_rig.place(boxes, x_m=x, y_m=y, yaw_rad=yaw),
            camera=camera,
            quantize=args.quantize,
            noise_k=args.noise_k,
            noise_seed=args.noise_seed,
        )
        truth = scene_rig.true_pockets(geometry, x_m=x, y_m=y, yaw_rad=yaw)
        valid, worst, _ = seed_pass(
            scene, prior, params, args.rule, args.seeds, truth
        )
        passed = valid == args.seeds
        if x > args.split:
            far += 1
            far_ok += passed
        else:
            near += 1
            near_ok += passed
        if passed:
            worst_accepted = max(worst_accepted, worst)
        print(
            f"{name:>8s} {x:7.3f} {y:7.3f} {yaw:6.3f} {valid:4d}/{args.seeds:<2d}"
            f" {worst * 1000:9.1f}"
        )
    total = near + far
    print(
        f"# all-seed {near_ok + far_ok}/{total}"
        f"  x<={args.split}: {near_ok}/{near}  x>{args.split}: {far_ok}/{far}"
        f"  worst accepted {worst_accepted * 1000:.1f} mm"
    )
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)

    grid = sub.add_parser("grid", help="shape x distance x pose grid")
    add_common(grid)
    grid.add_argument("--structure", choices=STRUCTURES, default="pallet")
    grid.add_argument("--distances", default="2.0:4.8:0.1", metavar="START:STOP:STEP")
    grid.add_argument("--lateral", default="0.0", help="comma-separated y offsets")
    grid.add_argument("--yaw", default="0.0", help="comma-separated yaw values")
    grid.set_defaults(func=cmd_grid)

    ev = sub.add_parser("evidence", help="dump one scene's gate evidence")
    add_common(ev)
    ev.add_argument("--structure", choices=STRUCTURES, default="pallet")
    ev.add_argument("--x", type=float, default=3.0)
    ev.add_argument("--y", type=float, default=0.0)
    ev.add_argument("--yaw", type=float, default=0.0)
    ev.add_argument(
        "--seed",
        type=int,
        default=None,
        help="RANSAC seed; default is the one in --params, not 0. Boundary "
        "counts are seed-sensitive, so a different default silently reports "
        "different evidence for the same scene",
    )
    ev.add_argument(
        "--distances",
        default=None,
        metavar="START:STOP:STEP",
        help="Sweep instead of a single --x, printing one row per distance",
    )
    ev.set_defaults(func=cmd_evidence)

    st = sub.add_parser("structures", help="every structure over the same distances")
    add_common(st)
    st.add_argument("--distances", default="2.4:4.2:0.6")
    st.add_argument("--structures", default=",".join(STRUCTURES))
    st.set_defaults(func=cmd_structures)

    fov = sub.add_parser("fov", help="where the approach face leaves the image")
    add_common(fov)
    fov.add_argument("--structure", choices=STRUCTURES, default="pallet")
    fov.add_argument("--distances", default="1.2:2.6:0.1")
    fov.set_defaults(func=cmd_fov)

    noise = sub.add_parser("noise", help="per-pixel axial depth noise")
    add_common(noise)
    noise.add_argument("--structure", choices=STRUCTURES, default="pallet")
    noise.add_argument("--distances", default="2.5:3.5:0.5")
    noise.add_argument("--sigmas", default="0,0.005,0.010,0.020,0.030")
    noise.add_argument("--sigma-at", type=float, default=3.0)
    noise.set_defaults(func=cmd_noise)

    planes = sub.add_parser("planes", help="plane candidates, margins and overlap")
    add_common(planes)
    planes.add_argument("--structure", choices=STRUCTURES, default="pallet")
    planes.add_argument("--distances", default="2.0:4.0:0.5")
    planes.add_argument("--y", type=float, default=0.0)
    planes.add_argument("--yaw", type=float, default=0.0)
    planes.set_defaults(func=cmd_planes)

    zcut = sub.add_parser("zcut", help="overhead obstruction lowered onto the deck")
    add_common(zcut)
    zcut.add_argument("--structure", choices=STRUCTURES, default="pallet")
    zcut.add_argument("--x", type=float, default=2.5)
    zcut.add_argument("--y", type=float, default=0.0)
    zcut.add_argument("--yaw", type=float, default=0.0)
    zcut.add_argument(
        "--front-x",
        type=float,
        default=1.60,
        help="x of the obstruction's FRONT FACE, not its centre",
    )
    zcut.add_argument("--obstruction-depth", type=float, default=0.10)
    zcut.add_argument("--obstruction-width", type=float, default=2.0)
    zcut.add_argument("--obstruction-height", type=float, default=1.0)
    zcut.add_argument("--cuts", default="0.090:0.120:0.005")
    zcut.set_defaults(func=cmd_zcut)

    poses = sub.add_parser("poses", help="the committed catalogue poses")
    add_common(poses)
    poses.add_argument("--structure", choices=STRUCTURES, default="pallet")
    poses.add_argument("--catalogue", type=Path, default=CATALOGUE)
    poses.add_argument("--category", default="positive")
    poses.add_argument("--split", type=float, default=3.0)
    poses.set_defaults(func=cmd_poses)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
