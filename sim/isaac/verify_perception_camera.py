"""Render a bright forward marker to check the perception camera axis convention.

Run with Isaac Sim 5.1's Python and an installed forklift-core package. This is
an axis smoke check, not a calibration or a perception validation. The forklift
must already be imported into --base-scene; --forklift-urdf records the expected
source path for CLI compatibility and does not re-import or modify that asset.
"""

import argparse
import importlib.util
import json
import sys
import traceback
from pathlib import Path

import numpy as np


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-scene", required=True)
    parser.add_argument(
        "--forklift-urdf",
        type=Path,
        default=Path(__file__).resolve().parent.parent
        / "models/dls08_provisional/forklift.urdf",
        help="Expected source URDF; forklift must already exist in the base scene",
    )
    parser.add_argument(
        "--camera-axes", choices=("world", "usd", "ros"), default="ros",
        help="ROS optical convention is the working hypothesis, not a verified result",
    )
    parser.add_argument("--output", type=Path, default=Path.cwd())
    args, unknown = parser.parse_known_args()
    # Preserve Kit options, as in run_transport.py.
    sys.argv = [sys.argv[0], *unknown]
    return args


def load_module(name: str, path: Path):
    """Load repository helpers outside the installed src package by path."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def run(app, args: argparse.Namespace, result: dict) -> None:
    """Build the static diagnostic scene and record the rendered centroid."""
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.sensors.camera import Camera
    from PIL import Image
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    from forklift_core.geometry import rotation_matrix_from_quaternion_xyzw

    root = Path(__file__).resolve().parents[2]
    adapter = load_module(
        "perception_camera_check_adapter", root / "sim/isaac/perception_adapter.py"
    )
    rig = load_module("perception_camera_check_rig", root / "tools/scene_rig.py")
    mount = adapter.default_base_from_optical()
    calibration = rig.intrinsics()
    # The former _LEVEL constant is now named OPTICAL_QUATERNION_XYZW.
    # Verify that its rotation still matches the adapter before reordering it.
    quaternion_xyzw = rig.OPTICAL_QUATERNION_XYZW
    if not np.allclose(
        rotation_matrix_from_quaternion_xyzw(quaternion_xyzw), mount.rotation,
        atol=1e-12, rtol=0,
    ):
        raise ValueError("Canonical optical quaternion and adapter mount disagree")
    quaternion_wxyz = np.asarray(adapter.xyzw_to_wxyz(quaternion_xyzw))

    if not omni.usd.get_context().open_stage(args.base_scene):
        raise RuntimeError("Cannot open base scene")
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    base_path = "/World/Forklift/base_link"
    if not stage.GetPrimAtPath(base_path).IsValid():
        raise RuntimeError(f"Base scene must already contain {base_path}")
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    world.scene.add(
        Robot(
            prim_path="/World/Forklift", name="forklift",
            position=np.zeros(3), orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
    )

    # A non-physical child marker expresses the target exactly in base_link,
    # even when the imported stage has an additional root/link transform.
    marker_path = base_path + "/PerceptionCameraCheckMarker"
    sphere = UsdGeom.Sphere.Define(stage, marker_path)
    sphere.CreateRadiusAttr(0.05)
    sphere.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.3))
    # Same UsdPreviewSurface and material binding pattern as scene.add_destination.
    material = UsdShade.Material.Define(stage, marker_path + "Material")
    shader = UsdShade.Shader.Define(stage, marker_path + "Material/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((1, 1, 1))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set((20, 20, 20))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(material)

    camera = Camera(
        prim_path=base_path + "/PerceptionCamera", frequency=-1,
        resolution=(calibration.width, calibration.height),
    )
    # Isaac 5.1 Camera.set_local_pose takes parent-relative translation and wxyz:
    # https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/
    # isaacsim.sensors.camera/docs/index.html
    camera.set_local_pose(
        translation=np.asarray(mount.translation_m),
        orientation=quaternion_wxyz, camera_axes=args.camera_axes,
    )
    camera.set_projection_mode("perspective")
    camera.set_lens_distortion_model("pinhole")
    camera.set_focal_length(1.0)
    camera.set_horizontal_aperture(
        calibration.width / calibration.fx, maintain_square_pixels=True,
    )
    world.reset()
    camera.initialize()
    # world.pause() was tried first to hold the robot at the origin during this
    # axis check, but paused world.step(render=True) never produced a rendered
    # frame here (camera.get_rgba() stayed shape (0,) after 30 steps, with
    # repeated "No adjacent samples found for interpolation" warnings --
    # confirmed empirically on ws1, 2026-09-19). Step with physics running
    # instead; the free-standing forklift only settles slightly under gravity
    # in this short window, which does not move the base_link-mounted camera
    # relative to base_link.
    for _ in range(60):
        world.step(render=True)

    rgba = np.asarray(camera.get_rgba())
    if rgba.shape != (calibration.height, calibration.width, 4):
        raise RuntimeError(f"RGBA frame not ready: shape={rgba.shape}")
    if rgba.dtype != np.uint8:
        raise RuntimeError(f"Unexpected RGBA dtype: {rgba.dtype}")
    image_path = args.output.resolve() / "perception_camera_check.png"
    Image.fromarray(rgba).convert("RGB").save(image_path)
    result["image_path"] = str(image_path)

    brightness = rgba[:, :, :3].astype(float).mean(axis=2)
    # Keep all ties at the 99th percentile, but exclude uniform/background frames
    # and require near-white highlights so a black image cannot pass at its centre.
    threshold = max(240.0, float(np.percentile(brightness, 99)))
    mask = brightness >= threshold
    if np.std(brightness) <= 2.0 or not mask.any() or mask.mean() > 0.05:
        result["reason"] = "No isolated bright marker pixels"
        return
    rows, columns = np.nonzero(mask)
    centroid = [float(columns.mean()), float(rows.mean())]
    result["centroid_uv"] = centroid
    # Correct forward projection is (320, 314.52): camera z=0.50, marker z=0.30,
    # forward range=2.0-0.75=1.25, so dv=465.741156*0.20/1.25=74.52 px.
    # A 100 px radius allows marker rasterization/bloom without demanding that
    # a marker below the optical axis land at the exact principal point.
    distance = float(np.hypot(
        centroid[0] - calibration.cx, centroid[1] - calibration.cy,
    ))
    result["distance_from_center_px"] = distance
    result["status"] = "PASS" if distance <= 100.0 else "FAIL"
    # Brightness alone cannot establish marker identity in every base scene;
    # inspect the saved image when comparing the three camera-axis modes.


def main() -> int:
    args = arguments()
    result = {
        "camera_axes": args.camera_axes, "centroid_uv": None,
        "status": "FAIL", "image_path": None,
    }
    app = None
    try:
        args.output.mkdir(parents=True, exist_ok=True)
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
        run(app, args, result)
    except Exception as exc:
        result["reason"] = str(exc)
        traceback.print_exc(file=sys.stderr)
    finally:
        # SimulationApp.close() does not reliably return to Python afterwards
        # (observed empirically on ws1, 2026-09-19: the process exits during
        # close() and code placed after it never runs). Print the result
        # first, and treat app.close() as a best-effort last step.
        print(json.dumps(result, allow_nan=False), flush=True)
        if app is not None:
            app.close()
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
