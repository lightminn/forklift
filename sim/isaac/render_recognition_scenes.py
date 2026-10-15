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
    p.add_argument("--max-captures", type=int, default=8)
    return p.parse_args(argv)


def selected(args):
    from tools import recognition_scenes as rs

    index, count = (int(v) for v in args.shard.split("/"))
    cases, seen, out = set(args.cases.split(",")), {}, []
    for recipe in rs.catalogue():
        if recipe.case not in cases or (args.split != "all" and recipe.split != args.split):
            continue
        key = (recipe.case, recipe.shape_key)
        seen[key] = seen.get(key, 0) + 1
        if args.limit is None or seen[key] <= args.limit:
            out.append(recipe)
    return out[index::count]


def build(stage, recipe, token, previous_root, rc):
    """Each scene gets its own root: reusing prim paths left stale semantic labels."""
    from isaacsim.core.utils.semantics import add_labels
    from pxr import Gf, UsdGeom

    if previous_root and stage.GetPrimAtPath(previous_root):
        stage.RemovePrim(previous_root)
    root = rc.scene_root(token)
    specs = rc.prim_specs(recipe, root=root)
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
        from pxr import UsdGeom, UsdLux

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
        UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1000.0)
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
        for _ in range(10):
            world.step(render=True)
        world.pause()
        site, root = load_site(), None
        for token, recipe in enumerate(selected(args)):
            try:
                previous, root = root, rc.scene_root(token)  # set first so a failed build is still removed
                build(stage, recipe, token, previous, rc)
                frame, semantic, attempt = None, None, 0
                for attempt in range(1, args.max_captures + 1):
                    frame = cc.capture_static(rep, annotators)
                    semantic = rc.decode_semantic(frame.get("seg"), token)
                    if semantic.ready:
                        break
                if not semantic.ready:
                    raise RuntimeError(f"semantics not ready after {args.max_captures} captures: {semantic.status} "
                                       f"(stale pixels {semantic.stale_pixels}, tokens {semantic.stale_tokens})")
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
        (args.out / "status.json").write_text(json.dumps(status, indent=1))
        if app is not None:
            app.close()
    return 1 if status["failed"] or "fatal" in status else 0


if __name__ == "__main__":
    sys.exit(main())
