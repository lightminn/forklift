#!/usr/bin/env python3
"""Render recognition recipes in Isaac Sim 5.1 into noise-free schema-v2 scenes.

origin = isaac_render_pose; truth comes from the CPU rig at this render's K through
the generator's recipe_record. Noise is applied afterwards by
tools/apply_recognition_noise.py. Run inside a ws1 Slurm GPU job only.
"""

import argparse
import json
import math
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim/isaac"))

import numpy as np


def arguments(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--settings", type=Path, default=ROOT / "config/isaac_perception_camera.yaml")
    p.add_argument("--cases", default="S1,S2,S3,S4,S5,S6,S7,N")
    p.add_argument("--split", choices=("dev", "eval", "all"), default="all")
    p.add_argument("--limit", type=int, default=None, help="first N selected recipes per case and shape")
    p.add_argument("--shard", default="0/1")
    p.add_argument("--render-mode", default="RaytracedLighting")
    p.add_argument("--render-preset", default="default",
                   help="RTX settings on the render products (sim/isaac/render_preset.py); default writes nothing")
    p.add_argument("--max-captures", type=int, default=8)
    # Stage 3 task 3: the learned proposer's training renders (docs/plans/2026-10-19-recognition-stage3-learned-proposals.md)
    p.add_argument("--catalogue", choices=("eval", "train"), default="eval",
                   help="train: tools.recognition_scenes.training_catalogue(--scale), randomized appearance and "
                   "lighting per recipe seed, and the light output (rgb.png, instances.png, meta.json; no truth)")
    p.add_argument("--scale", type=int, default=8)
    p.add_argument("--save-unready", type=Path, default=None,
                   help="diagnosis: write scenes that never became ready, decoded ignoring the token")
    return p.parse_args(argv)


def selected(args):
    from tools import recognition_scenes as rs

    index, count = (int(v) for v in args.shard.split("/"))
    cases, seen, out = set(args.cases.split(",")), {}, []
    recipes = rs.training_catalogue(args.scale) if args.catalogue == "train" else rs.catalogue()
    for recipe in recipes:
        if recipe.case not in cases or (args.catalogue == "eval" and args.split != "all" and recipe.split != args.split):
            continue
        key = (recipe.case, recipe.shape_key)
        seen[key] = seen.get(key, 0) + 1
        if args.limit is None or seen[key] <= args.limit:
            out.append(recipe)
    return out[index::count]


def build(stage, recipe, token, rc, randomize=False):
    """Labelled box prims for one scene under its own root, labels carrying the scene token; with ``randomize``
    the training appearance seeded by the recipe seed."""
    from isaacsim.core.utils.semantics import add_labels
    from pxr import Gf, UsdGeom

    root = rc.scene_root(token)
    specs = rc.prim_specs(recipe, root=root)
    if randomize:
        specs = rc.randomized_specs(specs, np.random.default_rng([recipe.seed, 0xC0105]))
    UsdGeom.Xform.Define(stage, root)
    for spec in specs:
        parent = spec.path.rsplit("/", 1)[0]
        if not stage.GetPrimAtPath(parent):
            UsdGeom.Xform.Define(stage, parent)
        cube = UsdGeom.Cube.Define(stage, spec.path)
        xf = UsdGeom.Xformable(cube)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*spec.centre_m))
        xf.AddRotateZOp().Set(math.degrees(spec.yaw_rad))
        xf.AddScaleOp().Set(Gf.Vec3d(*(s / 2 for s in spec.size_m)))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*spec.colour)])
        add_labels(cube.GetPrim(), labels=[rc.scene_label(spec.base_label, token)], instance_name="class")
    return root


def write_training_scene(scene_dir, rgb, instance_ids, meta) -> None:
    """The light training output: rgb.png, instances.png (0 background, k+1 instance k, 65535 distractor -- the
    dataset's code) and meta.json. No truth.json: the training scenes are never judged."""
    from PIL import Image

    scene_dir.mkdir(parents=True, exist_ok=False)
    ids = np.asarray(instance_ids)
    codes = np.where(ids >= 0, ids + 1, np.where(ids == -2, 65535, 0)).astype(np.uint16)
    Image.fromarray(np.asarray(rgb, dtype=np.uint8)).save(scene_dir / "rgb.png")
    Image.fromarray(codes).save(scene_dir / "instances.png")
    (scene_dir / "meta.json").write_text(json.dumps(meta, indent=1))


def main(argv=None):
    args = arguments(argv)
    args.out.mkdir(parents=True, exist_ok=False)
    status = {"rendered": [], "failed": [], "args": {k: str(v) for k, v in vars(args).items()}}
    app = None
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "renderer": args.render_mode, "multi_gpu": False})
        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.core.api import World
        from pxr import Gf, UsdGeom, UsdLux

        import camera_calibration as cc
        import perception_camera as pc
        import recognition_render_core as rc
        from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
        from forklift_core.perception.recognition_dataset import write_recognition_scene
        from forklift_core.perception.scene_dataset import SceneInput
        from forklift_core.sensors.rgbd import PinholeIntrinsics
        from tools.generate_recognition_scenes import load_site, recipe_record

        settings = pc.load_camera_settings(args.settings)
        omni.usd.get_context().new_stage()
        app.update()
        stage = omni.usd.get_context().get_stage()
        world = World(stage_units_in_meters=1.0)
        UsdGeom.Xform.Define(stage, "/World/Base")
        dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
        dome.CreateIntensityAttr(1000.0)
        sun = UsdLux.DistantLight.Define(stage, "/World/Sun") if args.catalogue == "train" else None
        if sun is not None:
            sun.CreateIntensityAttr(0.0)
            sun_xf = UsdGeom.Xformable(sun)
            sun_rot = sun_xf.AddRotateXYZOp()
        camera = pc.create_perception_camera(args.settings, prim_path="/World/Base/PerceptionCamera")
        world.reset()
        camera.initialize()
        sdk = np.asarray(camera.get_intrinsics_matrix(), dtype=float)
        f = float(settings["focal_px"])
        if max(abs(sdk[0, 0] - f), abs(sdk[1, 1] - f), abs(sdk[0, 2] - 320.0), abs(sdk[1, 2] - 240.0)) > 0.01:
            raise RuntimeError(f"SDK K {sdk.tolist()} does not match the camera settings")
        intrinsics = pc._isaac_intrinsics_type()(
            PinholeIntrinsics(640, 480, sdk[0, 0], sdk[1, 1], sdk[0, 2], sdk[1, 2], "camera_optical_frame")
        ).integer_index
        status["sdk_k"] = sdk.tolist()
        annotators = cc.attach_annotators(rep, camera)
        import render_preset

        status["render_preset"] = render_preset.apply_all(stage, args.render_preset)
        for _ in range(10):
            world.step(render=True)
        world.pause()
        site, root = load_site(), None
        for token, recipe in enumerate(selected(args)):
            try:
                previous, root = root, rc.scene_root(token)  # set first so a failed build is still removed
                if previous and stage.GetPrimAtPath(previous):
                    stage.RemovePrim(previous)
                    # One capture with no labelled prims before the next scene: without it Isaac kept the
                    # previous scene's id->label map for 34 of 1,020 scenes (2026-10-15, jobs 2848/2852/2857),
                    # app updates and fresh prim paths did not help; with it S1 rendered 122/122 (job 2860).
                    cc.capture_static(rep, annotators)
                lighting = None
                if args.catalogue == "train":
                    lighting = rc.training_lighting(np.random.default_rng([recipe.seed, 0x116]))
                    dome.GetIntensityAttr().Set(lighting["dome_intensity"])
                    sun.GetIntensityAttr().Set(lighting["sun_intensity"])
                    sun_rot.Set(Gf.Vec3f(90.0 - lighting["sun_elevation_deg"], 0.0, lighting["sun_azimuth_deg"]))
                build(stage, recipe, token, rc, randomize=args.catalogue == "train")
                frame, semantic, attempt = None, None, 0
                for attempt in range(1, args.max_captures + 1):
                    frame = cc.capture_static(rep, annotators)
                    semantic = rc.decode_semantic(frame.get("seg"), token)
                    if semantic.ready:
                        break
                if not semantic.ready and args.save_unready is not None:
                    loose = rc.decode_semantic(frame.get("seg"), None)
                    if loose.instance_ids is not None:
                        pos, q_wxyz = camera.get_world_pose(camera_axes="ros")
                        diag = SceneInput(
                            rgb=np.asarray(frame["rgb"])[:, :, :3].astype(np.uint8),
                            depth_m=rc.clean_depth(frame["z"]), intrinsics=intrinsics,
                            base_from_optical=RigidTransform(
                                "camera_optical_frame", "base_link",
                                rotation_matrix_from_quaternion_xyzw(np.roll(np.asarray(q_wxyz, dtype=float), -1)),
                                np.asarray(pos, dtype=float)),
                            stamp_ns=2_000_000_000, clock_domain="synthetic", source_provenance="synthetic")
                        record = recipe_record(recipe, site, intrinsics=intrinsics)
                        write_recognition_scene(
                            args.save_unready / recipe.scene_id, diag, loose.instance_ids, origin="isaac_render_pose",
                            meta={**record.meta, "render_source": "isaac", "truth_render": "rig", "noise": None,
                                  "render_token": token, "unready": semantic.status,
                                  "stale_tokens": list(semantic.stale_tokens)},
                            truth=record.truth, instruction=record.instruction)
                if not semantic.ready:
                    raise RuntimeError(f"semantics not ready after {args.max_captures} captures: {semantic.status} "
                                       f"(stale pixels {semantic.stale_pixels}, tokens {semantic.stale_tokens})")
                if args.catalogue == "train":
                    write_training_scene(args.out / recipe.scene_id, np.asarray(frame["rgb"])[:, :, :3].astype(np.uint8),
                                         semantic.instance_ids, {"scene_id": recipe.scene_id, "case": recipe.case,
                                                                 "shape_key": recipe.shape_key, "seed": recipe.seed,
                                                                 "lighting": lighting, "render_token": token})
                    status["rendered"].append({"scene_id": recipe.scene_id, "captures": attempt})
                    continue
                pos, q_wxyz = camera.get_world_pose(camera_axes="ros")
                scene = SceneInput(
                    rgb=np.asarray(frame["rgb"])[:, :, :3].astype(np.uint8), depth_m=rc.clean_depth(frame["z"]),
                    intrinsics=intrinsics,
                    base_from_optical=RigidTransform(
                        "camera_optical_frame", "base_link",
                        rotation_matrix_from_quaternion_xyzw(np.roll(np.asarray(q_wxyz, dtype=float), -1)),
                        np.asarray(pos, dtype=float)),
                    stamp_ns=2_000_000_000, clock_domain="synthetic", source_provenance="synthetic")
                record = recipe_record(recipe, site, intrinsics=intrinsics)
                write_recognition_scene(
                    args.out / recipe.scene_id, scene, semantic.instance_ids, origin="isaac_render_pose",
                    meta={**record.meta, "render_source": "isaac", "truth_render": "rig", "noise": None,
                          "render_token": token},
                    truth=record.truth, instruction=record.instruction)
                status["rendered"].append({"scene_id": recipe.scene_id, "captures": attempt})
            except Exception as exc:  # record the scene and keep going
                status["failed"].append({"scene_id": recipe.scene_id, "error": repr(exc),
                                         "trace": traceback.format_exc()})
        cc.detach_annotators(annotators)
    except Exception as exc:
        status["fatal"] = {"error": repr(exc), "trace": traceback.format_exc()}
    finally:
        # Persist before close: the observed Isaac close path may exit Python.
        if app is not None and status.get("render_preset", {}).get("render_products"):
            import omni.usd
            import render_preset

            status["render_preset"]["drift_at_end"] = render_preset.drifted(
                omni.usd.get_context().get_stage(), status["render_preset"])
        (args.out / "status.json").write_text(json.dumps(status, indent=1))
        if app is not None:
            app.close()
    return 1 if status["failed"] or "fatal" in status else 0


if __name__ == "__main__":
    sys.exit(main())
