"""Does Isaac's render geometry overturn the CPU rig's carriage-camera result?

Plan: docs/plans/2026-10-01-carriage-camera-render-check.md (step 4). A
minimal static stage -- floor with its top at z = 0, a wall at base x = 6 m,
the measured truck from its imported USD, and a visual-only EPAL 6 -- is
rendered from a camera on ``fork_carriage`` at (0.61, 0, 0.20), tilt 0, for
every cell of a pre-registered 1 cm face-gap grid. Physics never runs: the
timeline stays stopped and every capture is a paused zero-delta orchestrator
step. Each cell renders three times (all shown, carriage visuals hidden,
pallet visuals hidden); every render is checked against the depth the CPU
rig predicts for the pose read back from USD, including pixels the last
change must have moved, and a render that fails is retried and then marked
unconfirmed rather than counted. The CPU rig renders the same cell with the
same normalised K, and both depths go through the same detector and pixel
tip rule (mount_render_analysis).

This says nothing about the real camera: boxes only, no noise, no stereo
occlusion, no dark-material effect.

    <isaac-python> sim/isaac/render_mount_check.py \\
      --forklift-usd <base_scene>/forklift.usd \\
      --forklift-urdf sim/models/dls08_measured/forklift.urdf \\
      --output <dir>
"""

import argparse
import dataclasses
import hashlib
import json
import os
import sys
import time
import traceback
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CAMERA_XYZ_M = (0.61, 0.0, 0.20)
WALL_X_M = 6.0
RESOLUTION = (640, 480)
APERTURE_CM = 2.0955
CLIPPING_M = (0.03, 100.0)
STEP = {"rt_subframes": 4, "delta_time": 0.0, "pause_timeline": True}
CAPTURE_ATTEMPTS = 3
FORKLIFT = "/World/Forklift"
PALLET = "/World/PalletPose"
CAMERA = FORKLIFT + "/fork_carriage/MountCamera"


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path: Path, record: dict) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(record, indent=2, default=float) + "\n")
    tmp.replace(path)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--forklift-usd", type=Path, required=True)
    parser.add_argument("--forklift-urdf", type=Path, required=True)
    parser.add_argument(
        "--pallet-urdf", type=Path, default=ROOT / "sim/models/epal6_pallet/pallet.urdf"
    )
    parser.add_argument(
        "--pallet-geometry",
        type=Path,
        default=ROOT / "config/pallet_geometry_epal6.yaml",
    )
    parser.add_argument(
        "--pallet-prior", type=Path, default=ROOT / "config/pallet_prior_epal6.yaml"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seeds", type=int, default=12)
    parser.add_argument("--tau", type=float, default=0.02)
    parser.add_argument("--reference-gap", type=float, default=0.30)
    parser.add_argument(
        "--gaps", default="-0.20:0.70:0.01", help="start:stop:step, both ends included"
    )
    args, unknown = parser.parse_known_args()
    args.kit_arguments = unknown
    return args


def matrix_np(gf_matrix) -> np.ndarray:
    """USD's row-vector Matrix4d as a column-vector numpy matrix."""
    return np.array(gf_matrix, dtype=float).T


def matrix_gf(matrix: np.ndarray):
    from pxr import Gf

    return Gf.Matrix4d(np.asarray(matrix, dtype=float).T.tolist())


def set_transform(prim, matrix: np.ndarray) -> None:
    from pxr import UsdGeom

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(matrix_gf(matrix))


def add_cube(stage, path: str, centre, size) -> None:
    from pxr import UsdGeom

    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    matrix = np.diag([*size, 1.0])
    matrix[:3, 3] = centre
    set_transform(cube.GetPrim(), matrix)


def world_matrix(stage, path: str) -> np.ndarray:
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"No prim at {path}")
    return matrix_np(UsdGeom.XformCache().GetLocalToWorldTransform(prim))


def cubes_under(stage, root: str, *, collision: bool) -> list[tuple]:
    """World (centre, size) of every Cube below ``root``, through instance proxies."""
    from pxr import Usd, UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache()
    out = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Cube):
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI) != collision:
            continue
        matrix = matrix_np(cache.GetLocalToWorldTransform(prim))
        scale = np.linalg.norm(matrix[:3, :3], axis=0)
        if not np.allclose(matrix[:3, :3], np.diag(scale), atol=1e-6):
            raise RuntimeError(f"Rotated cube {prim.GetPath()}")
        size = UsdGeom.Cube(prim).GetSizeAttr().Get() * scale
        out.append((tuple(matrix[:3, 3]), tuple(size)))
    return out


def carriage_visual_root(stage) -> str:
    """The one instance root under fork_carriage that holds its visual cubes."""
    from pxr import Usd, UsdGeom, UsdPhysics

    carriage = stage.GetPrimAtPath(FORKLIFT + "/fork_carriage")
    roots = []
    for child in carriage.GetChildren():
        if child.IsInstanceProxy():
            continue
        cubes = [
            p
            for p in Usd.PrimRange(child, Usd.TraverseInstanceProxies())
            if p.IsA(UsdGeom.Cube) and not p.HasAPI(UsdPhysics.CollisionAPI)
        ]
        if cubes:
            roots.append(child)
    if len(roots) != 1:
        raise RuntimeError(f"Expected one carriage visual root, found {roots}")
    return str(roots[0].GetPath())


def import_pallet(args) -> tuple[str, str]:
    """Import the pallet URDF and return (usd file, path of its visuals prim)."""
    import omni.kit.commands
    from pxr import Usd

    ok, config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not ok:
        raise RuntimeError("Cannot create URDF importer")
    config.merge_fixed_joints = False
    config.fix_base = True
    config.collision_from_visuals = False
    config.create_physics_scene = False
    config.distance_scale = 1.0
    config.make_default_prim = True
    target = args.output / "pallet_import" / "pallet.usd"
    target.parent.mkdir()
    ok, _ = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(args.pallet_urdf),
        import_config=config,
        dest_path=str(target),
    )
    if not ok:
        raise RuntimeError("Pallet URDF import failed")
    layer = Usd.Stage.Open(str(target))
    visuals = [str(p.GetPath()) for p in layer.Traverse() if p.GetName() == "visuals"]
    if len(visuals) != 1:
        raise RuntimeError(f"Expected one pallet visuals prim, found {visuals}")
    return str(target), visuals[0]


def build_stage(app, args, record) -> dict:
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import UsdGeom

    from tools import scene_rig

    enable_extension("isaacsim.asset.importer.urdf")
    omni.usd.get_context().new_stage()
    for _ in range(10):
        app.update()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.DefinePrim("/World", "Xform")
    add_cube(stage, "/World/Floor", (0.5, 0.0, -0.005), (11.0, 20.0, 0.01))
    add_cube(stage, "/World/Wall", (WALL_X_M + 0.05, 0.0, 4.99), (0.1, 20.0, 10.0))
    stage.DefinePrim(FORKLIFT, "Xform").GetReferences().AddReference(
        str(args.forklift_usd)
    )
    pallet_usd, visuals = import_pallet(args)
    pose = stage.DefinePrim(PALLET, "Xform")
    set_transform(pose, np.eye(4))
    stage.DefinePrim(PALLET + "/Visual", "Xform").GetReferences().AddReference(
        pallet_usd, visuals
    )
    for _ in range(10):
        app.update()

    # Frames the run depends on, read back rather than assumed.
    base = world_matrix(stage, FORKLIFT + "/base_link")
    carriage = world_matrix(stage, FORKLIFT + "/fork_carriage")
    if not (
        np.allclose(base, np.eye(4), atol=1e-6)
        and np.allclose(carriage, base, atol=1e-6)
    ):
        raise RuntimeError(
            "world<-base and base<-carriage must both be identity at lift 0"
        )
    from chassis_contract import require_scene_matches_model, stage_chassis

    require_scene_matches_model(stage_chassis(stage, FORKLIFT), args.forklift_urdf)
    truck = scene_rig.truck_boxes(args.forklift_urdf)
    visual_root = carriage_visual_root(stage)
    visual_boxes = cubes_under(stage, visual_root, collision=False)
    import mount_render_analysis as analysis

    want = [(b.centre_m, b.size_m) for b in truck]
    if not analysis.boxes_match(visual_boxes, want):
        raise RuntimeError("Carriage visual cubes differ from the URDF carriage boxes")
    pallet_cubes = cubes_under(stage, PALLET, collision=False)
    from forklift_core.perception.pallet_geometry import load_pallet_geometry

    geometry = load_pallet_geometry(args.pallet_geometry)
    rig_pallet = [(b.centre_m, b.size_m) for b in scene_rig.pallet(geometry)]
    if not analysis.boxes_match(pallet_cubes, rig_pallet):
        raise RuntimeError("Pallet visual cubes differ from the rig's pallet boxes")
    if cubes_under(stage, PALLET, collision=True):
        raise RuntimeError("The pallet must be visual only")

    # Camera on the carriage: optical (ROS) axes are USD camera axes x (1,-1,-1).
    camera = UsdGeom.Camera.Define(stage, CAMERA)
    rig_camera = scene_rig.Camera(CAMERA_XYZ_M, 0.0)
    optical = rig_camera.base_from_optical()
    local = np.eye(4)
    local[:3, :3] = np.asarray(optical.rotation) @ np.diag([1.0, -1.0, -1.0])
    local[:3, 3] = optical.translation_m
    set_transform(camera.GetPrim(), local)
    width, height = RESOLUTION
    camera.GetProjectionAttr().Set(UsdGeom.Tokens.perspective)
    camera.GetHorizontalApertureAttr().Set(APERTURE_CM)
    camera.GetVerticalApertureAttr().Set(APERTURE_CM * height / width)
    camera.GetFocalLengthAttr().Set(APERTURE_CM * scene_rig.FOCAL_PX / width)
    camera.GetClippingRangeAttr().Set(CLIPPING_M)
    for _ in range(5):
        app.update()
    base_from_usd_camera = world_matrix(stage, CAMERA)
    base_from_optical = base_from_usd_camera @ np.diag([1.0, -1.0, -1.0, 1.0])
    if not (
        np.allclose(base_from_optical[:3, :3], optical.rotation, atol=1e-6)
        and np.allclose(base_from_optical[:3, 3], optical.translation_m, atol=1e-6)
    ):
        raise RuntimeError("base<-camera read back differs from the planned mount")
    record["frames"] = {
        "world_from_base": base.tolist(),
        "base_from_carriage": carriage.tolist(),
        "base_from_optical_read": base_from_optical.tolist(),
    }
    record["scene"] = {
        "carriage_visual_root": visual_root,
        "carriage_visual_cubes": len(visual_boxes),
        "pallet_visual_prim": visuals,
        "pallet_visual_cubes": len(pallet_cubes),
        "pallet_usd": pallet_usd,
    }
    return {
        "stage": stage,
        "truck": truck,
        "geometry": geometry,
        "rig_camera": rig_camera,
        "visual_root": visual_root,
    }


def raw_intrinsics(stage):
    """Isaac's SDK K from the camera's authored attributes (cx = width / 2)."""
    from pxr import UsdGeom

    from forklift_core.sensors.rgbd import PinholeIntrinsics

    camera = UsdGeom.Camera(stage.GetPrimAtPath(CAMERA))
    width, height = RESOLUTION
    focal = camera.GetFocalLengthAttr().Get()
    fx = width * focal / camera.GetHorizontalApertureAttr().Get()
    fy = height * focal / camera.GetVerticalApertureAttr().Get()
    return PinholeIntrinsics(
        width, height, fx, fy, width / 2, height / 2, "camera_optical_frame"
    )


def run(app, args, record) -> None:
    import mount_render_analysis as analysis
    import omni.replicator.core as rep
    import omni.timeline
    import perception_adapter
    from pxr import UsdGeom

    from forklift_core.perception.pallet_prior import load_pallet_prior
    from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
    from tools import measure_pocket_evidence as mpe
    from tools import scene_rig

    built = build_stage(app, args, record)
    stage, truck, geometry = built["stage"], built["truck"], built["geometry"]
    rig_camera = built["rig_camera"]
    calibration = perception_adapter.normalize_isaac_intrinsics(raw_intrinsics(stage))
    k = calibration.integer_index
    record["intrinsics"] = calibration.to_record()
    prior = load_pallet_prior(args.pallet_prior)
    params = DetectorParams.derived_for(prior, range_min_m=0.1)
    record["detector_params"] = dataclasses.asdict(params)
    side = mpe.blades(truck)
    tip_x = truck[side["left"]].centre_m[0] + truck[side["left"]].size_m[0] / 2
    half_depth = geometry.overall_depth_m / 2
    rig_pallet = scene_rig.pallet(geometry)
    samples = {name: mpe.tip_samples(truck[i]) for name, i in side.items()}
    optical = rig_camera.base_from_optical()

    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    product = rep.create.render_product(CAMERA, RESOLUTION)
    annotator = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    annotator.attach([product])
    rep.orchestrator.set_capture_on_play(False)
    carriage_vis = UsdGeom.Imageable(stage.GetPrimAtPath(built["visual_root"]))
    pallet_vis = UsdGeom.Imageable(stage.GetPrimAtPath(PALLET))
    base_before = world_matrix(stage, FORKLIFT + "/base_link")

    def depth_now() -> np.ndarray:
        rep.orchestrator.step(**STEP)
        data = annotator.get_data()
        if isinstance(data, dict):
            data = data["data"]
        depth = np.array(data, dtype=np.float32, copy=True).reshape(
            RESOLUTION[1], RESOLUTION[0]
        )
        return depth

    def expected(x_m: float, carriage: bool, pallet: bool) -> np.ndarray:
        boxes = [
            *(truck if carriage else []),
            *(scene_rig.place(rig_pallet, x_m=x_m) if pallet else []),
        ]
        scene = scene_rig.render(boxes, camera=rig_camera, quantize=False, intrinsics=k)
        return np.asarray(scene.depth_m, dtype=float)

    previous = {"expected": None}

    def capture(label: str, want: np.ndarray, *, needs_change: bool) -> tuple:
        pixels = analysis.reference_pixels(want, previous["expected"])
        attempts = []
        depth = None
        for _ in range(CAPTURE_ATTEMPTS):
            depth_now()  # the first frame after a change is discarded
            depth = depth_now()
            check = analysis.freshness(depth, want, pixels, needs_change=needs_change)
            attempts.append(check)
            if check["ok"] and not check["unverifiable"]:
                break
        previous["expected"] = want
        final = attempts[-1]
        good = final["ok"] and not final["unverifiable"]
        return depth, {"render": label, "confirmed": good, "attempts": attempts}

    def set_visibility(carriage: bool, pallet: bool) -> None:
        (carriage_vis.MakeVisible if carriage else carriage_vis.MakeInvisible)()
        (pallet_vis.MakeVisible if pallet else pallet_vis.MakeInvisible)()

    def place_pallet(x_m: float) -> tuple[float, float, float]:
        matrix = np.eye(4)
        matrix[0, 3] = x_m
        set_transform(stage.GetPrimAtPath(PALLET), matrix)
        return analysis.pallet_pose_from_matrix(world_matrix(stage, PALLET))

    # Visible -> hidden -> visible on a cell where the fork is plainly in view.
    x_ref, _, _ = place_pallet(tip_x + half_depth + args.reference_gap)
    reference = []
    for carriage in (True, False, True):
        set_visibility(carriage, True)
        _, check = capture(
            f"reference carriage={carriage}",
            expected(x_ref, carriage, True),
            needs_change=bool(reference),
        )
        reference.append(check)
    record["reference_toggle"] = reference
    if not all(c["confirmed"] for c in reference):
        raise RuntimeError("Visibility toggling is not reflected in the render")

    gaps = analysis.face_gap_grid(*(float(v) for v in args.gaps.split(":")))
    cells_dir = args.output / "cells"
    cells_dir.mkdir()
    record["cells"] = []
    for index, gap in enumerate(gaps):
        x_m = tip_x + half_depth + gap
        placed = scene_rig.place(rig_pallet, x_m=x_m)
        cell = {"index": index, "face_gap_m": gap, "pallet_x_m": x_m}
        if any(scene_rig.boxes_interpenetrate(t, p) for t in truck for p in placed):
            cell["status"] = "penetrating"
            record["cells"].append(cell)
            continue
        px, py, pyaw = place_pallet(x_m)
        cell["pallet_pose_read"] = [px, py, pyaw]
        truth = scene_rig.true_pockets(geometry, x_m=px, y_m=py, yaw_rad=pyaw)
        renders, checks, cpu = {}, {}, {}
        # One object changes per render, so each check's changed pixels belong
        # to that object: show all, hide carriage, restore it, hide pallet. The
        # next cell's first render restores the pallet.
        for name, carriage, pallet in (
            ("full", True, True),
            ("no_carriage", False, True),
            ("restored", True, True),
            ("no_pallet", True, False),
        ):
            set_visibility(carriage, pallet)
            cpu[name] = expected(px, carriage, pallet)
            renders[name], checks[name] = capture(name, cpu[name], needs_change=True)
        set_visibility(True, True)
        cell["freshness"] = checks
        full_ok = checks["full"]["confirmed"]
        masks_ok = all(c["confirmed"] for c in checks.values())
        cell["status"] = "valid" if full_ok else "unconfirmed"
        cell["masks_confirmed"] = masks_ok

        triple = ("full", "no_carriage", "no_pallet")
        labels = {
            "isaac": analysis.region_labels(**{k_: renders[k_] for k_ in triple}),
            "cpu": analysis.region_labels(**{k_: cpu[k_] for k_ in triple}),
        }
        boundaries = {key: analysis.boundary_mask(v) for key, v in labels.items()}
        tips = {}
        for renderer, depths in (("isaac", renders), ("cpu", cpu)):
            tips[renderer] = {
                name: analysis.pixel_tip_visibility(
                    depths["full"],
                    labels[renderer],
                    boundaries[renderer],
                    points,
                    optical,
                    k,
                )
                for name, points in samples.items()
            }
            if renderer == "isaac" and not masks_ok:
                for value in tips[renderer].values():
                    value["confirmed"] = False
        tips["cpu_continuous_ray"] = {
            name: mpe.tip_visibility(
                [*truck, *placed], side[name], rig_camera, analysis.MIN_RANGE_M
            )
            for name in side
        }
        cell["tip_visibility"] = tips
        if masks_ok:
            cell["depth_difference"] = analysis.difference_stats(
                renders["full"], cpu["full"], labels["isaac"], boundaries["isaac"]
            )

        detection = {}
        template = scene_rig.render([*truck, *placed], camera=rig_camera, intrinsics=k)
        # cpu32: the CPU depth cast to Isaac's float32, a diagnostic that separates
        # storage precision from renderer geometry when the raw-depth pairs differ.
        for renderer, raw in (
            ("isaac", renders["full"]),
            ("cpu", cpu["full"]),
            ("cpu32", cpu["full"].astype(np.float32)),
        ):
            if renderer == "isaac" and not full_ok:
                continue
            for quantize in (True, False):
                scene = dataclasses.replace(
                    template,
                    depth_m=analysis.detector_depth(raw, quantize=quantize),
                )
                ok = valid = 0
                worst = 0.0
                reasons = {}
                for seed in range(args.seeds):
                    observation = detect_pockets(
                        scene, prior, dataclasses.replace(params, seed=seed)
                    ).observation
                    if observation.status != "valid":
                        key = f"{observation.status}/{observation.reason}"
                        reasons[key] = reasons.get(key, 0) + 1
                        continue
                    valid += 1
                    error = mpe.pocket_error_m(observation, truth)
                    worst = max(worst, error)
                    if error <= args.tau:
                        ok += 1
                    else:
                        reasons["valid/error_over_tau"] = (
                            reasons.get("valid/error_over_tau", 0) + 1
                        )
                detection[f"{renderer}_{'q' if quantize else 'raw'}"] = {
                    "ok": ok,
                    "valid": valid,
                    "seeds": args.seeds,
                    "worst_error_m": worst,
                    "reasons": reasons,
                }
        cell["detection"] = detection
        np.savez_compressed(
            cells_dir / f"cell_{index:03d}.npz",
            isaac_full=renders["full"],
            isaac_no_carriage=renders["no_carriage"],
            isaac_restored=renders["restored"],
            isaac_no_pallet=renders["no_pallet"],
            cpu_full=cpu["full"].astype(np.float32),
            labels_isaac=labels["isaac"],
            labels_cpu=labels["cpu"],
        )
        record["cells"].append(cell)
        print(
            "CELL",
            json.dumps(
                {
                    "gap": gap,
                    "status": cell["status"],
                    "masks": masks_ok,
                    "det": {k_: v["ok"] for k_, v in detection.items()},
                    "tip": {
                        r: {n: v["fraction"] for n, v in tips[r].items()}
                        for r in ("isaac", "cpu")
                    },
                }
            ),
            flush=True,
        )
        write_json(args.output / "result.json", record)

    if not np.allclose(
        world_matrix(stage, FORKLIFT + "/base_link"), base_before, atol=0
    ):
        raise RuntimeError("The truck moved during a physics-free run")
    record["timeline_playing_at_end"] = bool(timeline.is_playing())
    record["summary"] = summarise(record["cells"], args.seeds)
    annotator.detach()


def summarise(cells: list[dict], seeds: int) -> dict:
    """Distance boundaries, claimed only across confirmed cells (plan criterion 3)."""
    import mount_render_analysis as analysis

    measured = [c for c in cells if c["status"] != "penetrating"]
    # Boundaries are claimed against the pre-registered grid: a grid cell that
    # was not run counts as unconfirmed; a penetrating one is not a pose.
    grid = analysis.face_gap_grid()
    excluded = [c["face_gap_m"] for c in cells if c["status"] == "penetrating"]

    def claim(states):
        return analysis.nearest_claim(states, grid=grid, excluded=excluded)

    out = {
        "cells": len(cells),
        "penetrating": sum(c["status"] == "penetrating" for c in cells),
        "valid": sum(c["status"] == "valid" for c in measured),
        "unconfirmed": sum(c["status"] == "unconfirmed" for c in measured),
        "masks_confirmed": sum(bool(c.get("masks_confirmed")) for c in measured),
    }
    for key in ("isaac_q", "isaac_raw", "cpu_q", "cpu_raw", "cpu32_q", "cpu32_raw"):

        def value(cell, test, key=key):
            result = cell["detection"].get(key)
            return None if result is None else test(result["ok"])

        for name, test in (
            ("first_nonzero", lambda ok: ok > 0),
            ("first_all_seed", lambda ok: ok == seeds),
        ):
            states = [(c["face_gap_m"], value(c, test)) for c in measured]
            out[f"{key}_{name}"] = claim(states)
            out[f"{key}_{name}_cells"] = analysis.runs(states)
    for renderer in ("isaac", "cpu"):
        for side in ("left", "right"):
            states = []
            for c in measured:
                tip = c["tip_visibility"][renderer][side]
                states.append(
                    (
                        c["face_gap_m"],
                        tip["fraction"] >= 0.5 if tip["confirmed"] else None,
                    )
                )
            out[f"tip_{renderer}_{side}_visible"] = claim(states)
            out[f"tip_{renderer}_{side}_visible_cells"] = analysis.runs(states)
    return out


def main() -> None:
    args = arguments()
    for name in (
        "forklift_usd",
        "forklift_urdf",
        "pallet_urdf",
        "pallet_geometry",
        "pallet_prior",
    ):
        setattr(args, name, getattr(args, name).resolve())
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    sources = [
        Path(__file__),
        Path(__file__).with_name("mount_render_analysis.py"),
        Path(__file__).with_name("perception_adapter.py"),
        Path(__file__).with_name("chassis_contract.py"),
        ROOT / "tools/scene_rig.py",
        ROOT / "tools/measure_pocket_evidence.py",
        *sorted((ROOT / "src/forklift_core").rglob("*.py")),
    ]
    record = {
        "success": False,
        "plan": "docs/plans/2026-10-01-carriage-camera-render-check.md",
        "arguments": {k: str(v) for k, v in vars(args).items()},
        "camera_xyz_m": CAMERA_XYZ_M,
        "orchestrator_step": STEP,
        "inputs_sha256": {
            str(p): sha256(p)
            for p in (
                args.forklift_usd,
                args.forklift_urdf,
                args.pallet_urdf,
                args.pallet_geometry,
                args.pallet_prior,
            )
        },
        "sources_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in sources},
    }
    sys.path.insert(0, str(ROOT))
    app = None
    started = time.monotonic()
    try:
        from isaacsim import SimulationApp

        app = SimulationApp(
            {
                "headless": True,
                "renderer": "RaytracedLighting",
                "multi_gpu": False,
                "limit_cpu_threads": len(os.sched_getaffinity(0)),
                "disable_viewport_updates": True,
            }
        )
        run(app, args, record)
        record["success"] = True
    except BaseException as exc:
        record["failure_reason"] = f"{type(exc).__name__}: {exc}"
        (args.output / "failure.txt").write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        record["wall_time_s"] = time.monotonic() - started
        write_json(args.output / "result.json", record)
        print(
            "RENDER_MOUNT_RESULT",
            json.dumps(record.get("summary"), default=float),
            flush=True,
        )
        if app is not None:
            app.close()
    sys.exit(0 if record["success"] else 1)


if __name__ == "__main__":
    main()
