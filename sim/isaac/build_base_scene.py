"""Rebuild a transport base scene around another forklift URDF.

The transport and SLAM runs read the truck's physics from the base scene's
`/World/Forklift`, which references an Isaac URDF-importer output. This copies an
existing base scene, removes its `/World/Forklift` prim spec (the reference and
every local override on it or below it), imports the given URDF with the same
importer settings `run_perception_approach.py` uses, and references the result.
Before saving it reads the new truck back from the stage and refuses to write a
scene that does not match the URDF (`chassis_contract`).

It replaces only the truck. The rest of the scene -- warehouse, lights, physics
scene -- is carried over unchanged from the source scene; rebuilding that part
from nothing is still an outstanding ADR 0004 D2 item.

    <isaac-python> sim/isaac/build_base_scene.py \
      --source-scene <dir>/scene.usda \
      --forklift-urdf sim/models/dls08_measured/forklift.urdf \
      --output <new dir>
"""

import argparse
import hashlib
import json
import sys
import traceback
from pathlib import Path

IMPORTER_SETTINGS = {
    "merge_fixed_joints": False,
    "import_inertia_tensor": True,
    "fix_base": False,
    "self_collision": False,
    "collision_from_visuals": False,
    "create_physics_scene": False,
    "distance_scale": 1.0,
    "make_default_prim": True,
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-scene", type=Path, required=True)
    parser.add_argument("--forklift-urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args, unknown = parser.parse_known_args()
    args.kit_arguments = unknown
    return args


def build(app, args: argparse.Namespace) -> dict:
    import omni.kit.commands
    import omni.usd
    from chassis_contract import (
        chassis_record,
        require_scene_matches_model,
        stage_chassis,
    )
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Sdf

    enable_extension("isaacsim.asset.importer.urdf")
    ok, config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not ok:
        raise RuntimeError("Cannot create URDF importer")
    for name, value in IMPORTER_SETTINGS.items():
        setattr(config, name, value)
    robot_usd = args.output / "forklift.usd"
    ok, _ = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(args.forklift_urdf),
        import_config=config,
        dest_path=str(robot_usd),
    )
    if not ok:
        raise RuntimeError("Forklift URDF import failed")

    if not omni.usd.get_context().open_stage(str(args.source_scene)):
        raise RuntimeError("Cannot open source scene")
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    root = stage.GetRootLayer()
    path = Sdf.Path("/World/Forklift")
    if root.GetPrimAtPath(path) is None:
        raise RuntimeError("Source scene has no /World/Forklift spec in its root layer")
    # Drop the whole spec so no override below the old truck survives.
    stage.RemovePrim(path)
    if stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError("/World/Forklift is still composed from another layer")
    stage.DefinePrim(path, "Xform").GetReferences().AddReference(str(robot_usd))
    for _ in range(10):
        app.update()
    scene_chassis = stage_chassis(stage, str(path))
    record = chassis_record(scene_chassis)
    print("SCENE_CHASSIS", json.dumps(record), flush=True)
    require_scene_matches_model(scene_chassis, args.forklift_urdf)
    scene = args.output / "scene.usda"
    root.Export(str(scene))
    return {
        "source_scene": str(args.source_scene),
        "source_scene_sha256": sha256(args.source_scene),
        "forklift_urdf": str(args.forklift_urdf),
        "forklift_urdf_sha256": sha256(args.forklift_urdf),
        "importer_settings": IMPORTER_SETTINGS,
        "scene_chassis": record,
        "scene_chassis_matches_urdf": True,
        "outputs": ["scene.usda", "forklift.usd"],
    }


def main() -> None:
    args = arguments()
    args.output = args.output.resolve()
    args.forklift_urdf = args.forklift_urdf.resolve()
    args.source_scene = args.source_scene.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    app = None
    manifest = {"success": False}
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "multi_gpu": False})
        manifest = {**build(app, args), "success": True}
    except BaseException as exc:
        manifest["failure_reason"] = str(exc)
        (args.output / "failure.txt").write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        (args.output / "base_scene_manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n"
        )
        print("BASE_SCENE_RESULT", json.dumps(manifest), flush=True)
        if app is not None:
            app.close()
    sys.exit(0 if manifest["success"] else 1)


if __name__ == "__main__":
    main()
