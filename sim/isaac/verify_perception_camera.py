"""G1 quantitative camera experiment, plan v11 (2026-09-21).

Run with Isaac Sim 5.1's Python, OpenCV and installed forklift-core. Captures RGB,
semantic identity and axial depth at a paused simulation time. This is synthetic
camera verification, not physical calibration or an all-point 3D error bound.
Use a NEW --output directory. result.json and raw captures survive failed gates.
"""

import argparse
import hashlib
import importlib.util
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
    parser.add_argument("--camera-axes", choices=("world", "usd", "ros"), default="ros")
    parser.add_argument(
        "--output", type=Path, required=True, help="New experiment directory"
    )
    args, unknown = parser.parse_known_args()
    sys.argv = [sys.argv[0], *unknown]
    return args


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


ROOT = Path(__file__).resolve().parents[2]
RUNNER = load_module("perception_camera_runner", ROOT / "sim/isaac/run_transport.py")
MEASURE = RUNNER.CAMERA_CALIBRATION


def write_record(path: Path, value: dict) -> None:
    path.write_text(RUNNER.record_json(value, indent=2) + "\n", encoding="utf-8")


def save_capture(directory: Path, name: str, frame: dict, metadata: dict) -> None:
    from PIL import Image

    seg = frame["seg"] if isinstance(frame["seg"], dict) else {}
    diagnostics = frame.get("capture_diagnostics", {})
    # Persist the diagnosis first, including malformed or absent buffers.
    write_record(
        directory / f"{name}_capture.json",
        {
            **metadata,
            "segmentation_info": seg.get("info", {}),
            "capture_diagnostics": diagnostics,
            "depth_units": "metres",
            "depth_definition": "optical_axis_z",
            "annotators": ["rgb", "semantic_segmentation", "distance_to_image_plane"],
            "orchestrator_step": {
                "rt_subframes": 4,
                "delta_time": 0.0,
                "pause_timeline": True,
                "count": len(diagnostics.get("attempts", [None])),
            },
        },
    )
    rgb, depth = np.asarray(frame["rgb"]), np.asarray(frame["z"])
    if rgb.shape not in ((480, 640, 3), (480, 640, 4)) or rgb.dtype != np.uint8:
        raise ValueError(f"Invalid RGB capture: {rgb.shape}, {rgb.dtype}")
    if depth.shape == (480, 640, 1):
        depth = depth[..., 0]
    if depth.shape != (480, 640):
        raise ValueError(f"Invalid axial depth capture: {depth.shape}")
    frame["z"] = depth
    Image.fromarray(rgb[..., :3]).save(directory / f"{name}_rgb.png")
    arrays = {"depth_m": depth}
    if seg.get("data") is not None:
        arrays["segmentation"] = seg["data"]
    np.savez_compressed(directory / f"{name}_arrays.npz", **arrays)
    if diagnostics.get("ready"):
        mask = MEASURE.semantic_mask(seg, diagnostics["target_label"])
        Image.fromarray(mask.astype(np.uint8) * 255).save(
            directory / f"{name}_target_mask.png"
        )


def render_pipeline_state(
    stage, camera, timeline, rep, *, attached_product: str
) -> dict:
    """Read pipeline state without stepping, pausing, or repairing anything.

    A valid USD RenderProduct is not proof of a working Hydra product. Keep
    unavailable readbacks explicit so diagnostic errors cannot erase RGB-D.
    """
    errors = {}

    def read(name, getter):
        try:
            return getter()
        except Exception as exc:
            errors[name] = f"{type(exc).__name__}: {exc}"
            return None

    path = read("product_path", lambda: str(camera.get_render_product_path()))
    product = stage.GetPrimAtPath(path) if path else None
    valid = bool(product is not None and product.IsValid())
    product_state = {
        "path": path,
        "valid": valid,
        "attached_product_path": attached_product,
        "matches_attached_product": path == attached_product,
    }
    if valid:
        product_state.update(
            active=read("product_active", product.IsActive),
            type=read("product_type", product.GetTypeName),
            resolution=read(
                "product_resolution",
                lambda: list(product.GetAttribute("resolution").Get()),
            ),
            camera_targets=read(
                "product_camera_targets",
                lambda: list(map(str, product.GetRelationship("camera").GetTargets())),
            ),
        )
        product_state["camera_targets_valid"] = [
            bool(stage.GetPrimAtPath(p).IsValid())
            for p in product_state["camera_targets"] or []
        ]
    return {
        "timeline": {
            "time_s": read("timeline_time", lambda: float(timeline.get_current_time())),
            "is_playing": read("timeline_playing", timeline.is_playing),
            "is_stopped": read("timeline_stopped", timeline.is_stopped),
        },
        "orchestrator_status": read(
            "orchestrator_status", lambda: str(rep.orchestrator.get_status())
        ),
        "render_product": product_state,
        "camera_clipping_range_m": read(
            "camera_clipping_range",
            lambda: list(map(float, camera.get_clipping_range())),
        ),
        "readback_errors": errors,
    }


def capture_board_with_clip_check(
    camera, vertices_world, world_from_camera_usd, capture, name, path, result
):
    """A/B an observed clipping defect without changing geometry or timing.

    The original frame is persisted first. Only a fully near-clipped, forward
    board permits a near-only change. Runtime recovery must still be measured;
    a successful setter alone is not evidence that the renderer recovered.
    """
    clipping = MEASURE.target_clipping_diagnostics(
        vertices_world, world_from_camera_usd, camera.get_clipping_range()
    )
    if not clipping["all_before_near"]:
        return capture(name, path, "calibration_board")
    before_name = name + "_before_near_clip"
    experiment = {
        "before_capture": before_name,
        "after_capture": name,
        "target_clipping_before": clipping,
        "observation": "recovery_not_confirmed",
    }
    result.setdefault("near_clip_experiments", []).append(experiment)
    before, _, _ = capture(before_name, path, "calibration_board", validate=False)
    experiment["render_before"] = before["capture_diagnostics"]["attempts"][-1][
        "render"
    ]
    experiment["adjustment"] = MEASURE.lower_near_clip_for_target(camera, clipping)
    experiment["target_clipping_after"] = MEASURE.target_clipping_diagnostics(
        vertices_world, world_from_camera_usd, camera.get_clipping_range()
    )
    after = capture(name, path, "calibration_board")
    last = after[0]["capture_diagnostics"]["attempts"][-1]
    experiment["render_after"] = last["render"]
    experiment["target_depth_finite_fraction_after"] = last.get(
        "target_depth_finite_fraction"
    )
    if (
        last["render"]["status"] == "nonempty"
        and last["render"]["rgb"]["max"] > 0
        and last.get("target_depth_finite_fraction", 0) > 0
    ):
        experiment["observation"] = "target_recovered_after_near_only_change"
    return after


class CalibrationScene:
    """Small USD author/readback boundary, imported only after SimulationApp."""

    def __init__(self, stage, base_path: str):
        from pxr import Usd, UsdGeom

        self.stage, self.base_path = stage, base_path
        self.time_code = Usd.TimeCode.Default()
        self.UsdGeom = UsdGeom
        if abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-12:
            raise ValueError("G1 expects metre stage units")
        self.materials = {
            "white": self.material("/World/G1White", (1.0, 1.0, 1.0), (1.0, 1.0, 1.0)),
            "black": self.material("/World/G1Black", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "marker": self.material(
                "/World/G1MarkerMaterial", (1.0, 1.0, 1.0), (20.0, 20.0, 20.0)
            ),
        }

    def material(self, path, diffuse, emissive):
        from pxr import Sdf, UsdShade

        material = UsdShade.Material.Define(self.stage, path)
        shader = UsdShade.Shader.Define(self.stage, path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(diffuse)
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(emissive)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        return material

    def world_matrix(self, path) -> np.ndarray:
        # Compute fresh after every placement/capture: no stale XformCache.
        prim = self.stage.GetPrimAtPath(path)
        return MEASURE.rigid_matrix(
            np.asarray(
                self.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    self.time_code
                )
            ).T
        )

    def vertices_world(self, path) -> np.ndarray:
        mesh = self.UsdGeom.Mesh(self.stage.GetPrimAtPath(path))
        points = np.asarray(mesh.GetPointsAttr().Get(self.time_code), dtype=float)
        return MEASURE.transform_points(self.world_matrix(path), points)

    def target_diagnostics(self, path: str) -> dict:
        """Read label placement and instancing on composed USD render geometry.

        This is scene-graph evidence, not proof of visible renderer pixels.
        Include ancestors because semantics and visibility can be inherited.
        """
        target = self.stage.GetPrimAtPath(path)
        records = []
        prim = target
        while prim.IsValid() and not prim.IsPseudoRoot():
            imageable = self.UsdGeom.Imageable(prim)
            label_attributes = {}
            for attr in prim.GetAttributes():
                if "semantic" in attr.GetName().lower():
                    value = attr.Get(self.time_code)
                    label_attributes[attr.GetName()] = (
                        list(map(str, value))
                        if value is not None
                        and not isinstance(value, str)
                        and hasattr(value, "__iter__")
                        else str(value)
                    )
            records.append(
                {
                    "path": str(prim.GetPath()),
                    "type": prim.GetTypeName(),
                    "is_renderable_gprim": prim.IsA(self.UsdGeom.Gprim),
                    "is_instance": prim.IsInstance(),
                    "is_instance_proxy": prim.IsInstanceProxy(),
                    "prototype_path": str(prim.GetPrimInPrototype().GetPath())
                    if prim.IsInstanceProxy()
                    else None,
                    "applied_schemas": list(prim.GetAppliedSchemas()),
                    "semantic_attributes": label_attributes,
                    "computed_visibility": str(
                        imageable.ComputeVisibility(self.time_code)
                    )
                    if imageable
                    else None,
                    "computed_purpose": str(imageable.ComputePurpose())
                    if imageable
                    else None,
                }
            )
            prim = prim.GetParent()
        return {
            "path": path,
            "valid": target.IsValid(),
            "target_and_ancestors": records,
        }

    def mesh(self, path, vertices, faces, material, label):
        from isaacsim.core.utils.semantics import add_labels
        from pxr import Gf, UsdShade

        mesh = self.UsdGeom.Mesh.Define(self.stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in vertices])
        mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
        mesh.CreateFaceVertexIndicesAttr(np.asarray(faces).ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        add_labels(mesh.GetPrim(), labels=[label], instance_name="class")
        # No CollisionAPI or RigidBodyAPI: this is only a rendered surface.
        return mesh

    def marker(self) -> str:
        from isaacsim.core.utils.semantics import add_labels
        from pxr import Gf, UsdShade

        path = self.base_path + "/PerceptionCameraCheckMarker"
        sphere = self.UsdGeom.Sphere.Define(self.stage, path)
        sphere.CreateRadiusAttr(0.05)
        sphere.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.3))
        UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(
            self.materials["marker"]
        )
        add_labels(sphere.GetPrim(), labels=["axis_marker"], instance_name="class")
        return path

    def board(self, geometry: dict, *, observe=None) -> str:
        from pxr import UsdShade

        path = self.base_path + "/G1Board"
        mesh = self.mesh(
            path,
            geometry["vertices_base_m"],
            geometry["faces"],
            self.materials["white"],
            "calibration_board",
        )
        if observe is not None:
            observe("board_mesh_authored")
        for name in ("white", "black"):
            subset = self.UsdGeom.Subset.CreateGeomSubset(
                mesh,
                name,
                self.UsdGeom.Tokens.face,
                geometry[name + "_faces"],
                "materialBind",
            )
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(
                self.materials[name]
            )
            if observe is not None:
                observe("board_subset_authored_" + name)
        self.mesh(
            path + "/Border",
            geometry["border_base_m"],
            [[0, 1, 2, 3]],
            self.materials["white"],
            "calibration_board",
        )
        if observe is not None:
            observe("board_border_authored")
        return path

    def height_panel(self, centre, distance, height) -> str:
        x, y, _ = centre
        dx, dy = 0.20 * distance, 0.16 * distance
        vertices = [
            [x - dx, y - dy, height],
            [x + dx, y - dy, height],
            [x + dx, y + dy, height],
            [x - dx, y + dy, height],
        ]
        path = self.base_path + "/G1HeightPanel"
        self.mesh(
            path, vertices, [[0, 1, 2, 3]], self.materials["white"], "height_panel"
        )
        return path

    def hide_existing_geometry(self) -> list[str]:
        # Preserve the original scene for the marker comparison first. Then
        # isolate calibration surfaces: a settled base can put h=0 underneath
        # the world floor. Hide only existing Gprims, not base/camera ancestors;
        # no transforms, physics or on-disk base asset are changed.
        hidden = []
        for prim in self.stage.Traverse():
            if prim.IsA(self.UsdGeom.Gprim):
                self.UsdGeom.Imageable(prim).MakeInvisible()
                hidden.append(str(prim.GetPath()))
        return hidden


def run(app, args: argparse.Namespace, result: dict) -> None:
    import cv2
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.sensors.camera import Camera

    import forklift_core
    from forklift_core.geometry import RigidTransform
    from forklift_core.sensors.rgbd import PinholeIntrinsics

    adapter = load_module(
        "perception_camera_check_adapter", ROOT / "sim/isaac/perception_adapter.py"
    )
    rig = load_module("perception_camera_check_rig", ROOT / "tools/scene_rig.py")
    nominal_mount, nominal_k = adapter.default_base_from_optical(), rig.intrinsics()
    nominal_matrix = MEASURE.mount_matrix(nominal_mount)
    result["source_sha256"] = RUNNER.source_sha256(
        ROOT, Path(forklift_core.__file__).parent
    )
    result["gate_5"] = "PASS"
    result["opencv_version"] = cv2.__version__
    result["nominal_base_from_optical"] = nominal_matrix.tolist()
    RUNNER.require(
        omni.usd.get_context().open_stage(args.base_scene), "Cannot open base scene"
    )
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    base_path = "/World/Forklift/base_link"
    RUNNER.require(
        stage.GetPrimAtPath(base_path).IsValid(), f"Base scene must contain {base_path}"
    )
    scene = CalibrationScene(stage, base_path)
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    world.scene.add(
        Robot(
            prim_path="/World/Forklift",
            name="forklift",
            position=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
    )
    marker_path = scene.marker()
    camera_path = base_path + "/PerceptionCamera"
    camera = Camera(
        prim_path=camera_path,
        frequency=-1,
        resolution=(nominal_k.width, nominal_k.height),
    )
    camera.set_local_pose(
        translation=np.asarray(nominal_mount.translation_m),
        orientation=np.asarray(adapter.xyzw_to_wxyz(rig.OPTICAL_QUATERNION_XYZW)),
        camera_axes=args.camera_axes,
    )
    camera.set_projection_mode("perspective")
    camera.set_lens_distortion_model("pinhole")
    camera.set_focal_length(1.0)
    camera.set_horizontal_aperture(
        nominal_k.width / nominal_k.fx, maintain_square_pixels=True
    )
    world.reset()
    camera.initialize()
    RUNNER.verify_camera_intrinsics(camera, nominal_k, result)
    annotators = MEASURE.attach_annotators(rep, camera)
    attached_product = str(camera.get_render_product_path())
    # Preserve the original smoke-check settling period; all measurement
    # captures below use the required orchestrator zero-delta paused step.
    for _ in range(60):
        world.step(render=True)
    world.pause()
    timeline = omni.timeline.get_timeline_interface()
    RUNNER.require(
        stage.GetPrimAtPath(camera_path).GetParent() == stage.GetPrimAtPath(base_path),
        "Camera must be a direct base_link child",
    )

    def read_transforms():
        wb, wc = scene.world_matrix(base_path), scene.world_matrix(camera_path)
        actual, record = MEASURE.read_mount(camera, wb.T, wc.T, nominal_matrix)
        return wb, wc, actual, record

    wb, wc, actual_matrix, mount_record = read_transforms()
    result["mount"] = mount_record
    result["settled_world_from_base"] = wb.tolist()
    actual_mount = RigidTransform(
        nominal_mount.source_frame,
        nominal_mount.target_frame,
        actual_matrix[:3, :3],
        actual_matrix[:3, 3],
    )
    result["marker"], result["boards"], result["height_panels"] = [], [], []
    result["intrinsics_fits"] = []

    result["segmentation_captures"] = []
    result["annotator_attachment_probes"] = []
    pipeline_trace = []
    result["render_pipeline_trace_file"] = "render_pipeline_trace.json"

    def pipeline_state():
        return render_pipeline_state(
            stage, camera, timeline, rep, attached_product=attached_product
        )

    def trace_pipeline(event):
        pipeline_trace.append({"event": event, "state": pipeline_state()})
        write_record(args.output / "render_pipeline_trace.json", pipeline_trace)

    def probe_attachment(name):
        # Failure evidence only. A new annotator can share internal render
        # nodes, so recovery of both streams cannot prove reattachment needed.
        probe = {
            "capture": name,
            "measurement_eligible": False,
            "existing_product_path": attached_product,
            "fresh_product_path": str(camera.get_render_product_path()),
        }
        result["annotator_attachment_probes"].append(probe)
        fresh = {}
        try:
            fresh = MEASURE.attach_annotators(rep, camera)
            comparison, frames = MEASURE.compare_annotator_streams(
                rep, annotators, fresh, max_steps=8, pipeline_state=pipeline_state
            )
            probe.update(comparison)
            for binding, raw in frames.items():
                raw["capture_diagnostics"] = {
                    "attempts": comparison["attempts"],
                    "ready": False,
                    "diagnostic_only": True,
                }
                save_capture(
                    args.output,
                    name + "_attachment_probe_" + binding,
                    raw,
                    {"diagnostic_only": True, "attachment_comparison": comparison},
                )
        except Exception as exc:
            probe["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            for annotator in fresh.values():
                try:
                    annotator.detach([camera.get_render_product_path()])
                except Exception as exc:
                    probe.setdefault("detach_errors", []).append(str(exc))
            write_record(args.output / f"{name}_attachment_probe.json", probe)

    def capture(
        name, target_path, target_label, *, require_semantics=True, validate=True
    ):
        before_time = float(timeline.get_current_time())
        wb0, wc0, _, _ = read_transforms()
        target0 = scene.world_matrix(target_path)
        frame = MEASURE.capture_semantic_static(
            rep,
            annotators,
            target_label=target_label,
            max_steps=8 if require_semantics else 1,
            pipeline_state=pipeline_state,
        )
        wb1, wc1, _, mount = read_transforms()
        target1 = scene.world_matrix(target_path)
        after_time = float(timeline.get_current_time())
        metadata = {
            "time_before_s": before_time,
            "time_after_s": after_time,
            "usd_time_code": "Default (paused composed stage)",
            "stage_meters_per_unit": 1.0,
            "world_from_base": wb1.tolist(),
            "world_from_camera_usd": wc1.tolist(),
            "world_from_target": target1.tolist(),
            "mount": mount,
            "target_prim": scene.target_diagnostics(target_path),
            "render_product_path": str(camera.get_render_product_path()),
            "render_product_camera_targets": pipeline_state()["render_product"].get(
                "camera_targets"
            ),
            "segmentation_init_params": {"colorize": False},
        }
        if target_label in ("calibration_board", "height_panel"):
            metadata["target_clipping"] = MEASURE.target_clipping_diagnostics(
                scene.vertices_world(target_path), wc1, camera.get_clipping_range()
            )
        diagnostic = frame["capture_diagnostics"]
        result["segmentation_captures"].append(
            {
                "capture": name,
                "diagnostics_file": f"{name}_capture.json",
                "required": require_semantics,
                "ready": diagnostic["ready"],
                "first_ready_step": diagnostic["first_ready_step"],
                "step_count": len(diagnostic["attempts"]),
                "last_attempt": diagnostic["attempts"][-1],
                "failure_reason": diagnostic["failure_reason"],
            }
        )
        # Save even an invalid capture before refusing its timing contract.
        save_capture(args.output, name, frame, metadata)
        RUNNER.require(
            before_time == after_time and not timeline.is_playing(),
            "Static capture advanced the timeline",
        )
        RUNNER.require(
            all(
                np.allclose(a, b, atol=1e-9, rtol=0)
                for a, b in [(wb0, wb1), (wc0, wc1), (target0, target1)]
            ),
            "Transforms changed during static capture",
        )
        if validate and require_semantics:
            if (
                target_label != "axis_marker"
                and diagnostic["failure_reason"] == "empty_render_frame"
            ):
                # The original eight-step failure remains the G1 result even
                # when this diagnostic probe later observes recovery.
                probe_attachment(name)
            MEASURE.require_capture_semantics(frame)
        return frame, wb1, wc1

    # Reproduce the original emissive sphere AND global 99th-percentile method
    # in the original scene, before making a clean calibration-only view.
    for repeat in range(MEASURE.REPEATS):
        frame, wb, wc = capture(f"marker_{repeat}", marker_path, "axis_marker")
        marker_world = scene.world_matrix(marker_path)[:3, 3]
        optical = MEASURE.transform_points(
            np.linalg.inv(wc @ np.diag([1, -1, -1, 1])), marker_world[None, :]
        )
        marker = MEASURE.marker_centroids(
            frame["rgb"],
            MEASURE.semantic_mask(frame["seg"], "axis_marker"),
            MEASURE.project(optical, nominal_k)[0],
            float(optical[0, 2]),
            nominal_k,
        )
        result["marker"].append(marker)
    RUNNER.require(mount_record["status"] == "PASS", "G1③ nominal mount mismatch")
    trace_pipeline("marker_sequence_complete_before_geometry_hidden")
    result["calibration_hidden_gprims"] = scene.hide_existing_geometry()
    trace_pipeline("after_geometry_hidden_before_marker_removed")
    result["calibration_isolation_reason"] = (
        "Hide pre-existing render geometry after marker comparison, including world floor that can occlude base_link h=0; retain actual settled base and camera transforms."
    )
    stage.RemovePrim(marker_path)
    trace_pipeline("after_marker_removed")

    holdout = np.array([(i + j) % 4 == 0 for i in range(7) for j in range(9)])
    fits_by_distance = {}
    for distance in MEASURE.DISTANCES_M:
        samples = [[] for _ in range(MEASURE.REPEATS)]
        for position_index, centre_uv in enumerate(MEASURE.SCREEN_CENTRES_UV):
            geometry = MEASURE.checkerboard_layout(
                distance, centre_uv, nominal_k, nominal_matrix
            )
            trace_pipeline(f"before_board_r{distance:.1f}_p{position_index}_authored")
            path = scene.board(geometry, observe=trace_pipeline)
            for repeat in range(MEASURE.REPEATS):
                name = f"board_r{distance:.1f}_p{position_index}_repeat{repeat}"
                record = {
                    "capture": name,
                    "anchor_horizontal_m": distance,
                    "placement_uv": centre_uv,
                    "repeat": repeat,
                    "status": "FAIL",
                }
                result["boards"].append(record)
                frame, wb, wc = capture_board_with_clip_check(
                    camera,
                    scene.vertices_world(path),
                    scene.world_matrix(camera_path),
                    capture,
                    name,
                    path,
                    result,
                )
                # Use readback authored mesh points, not the placement inputs.
                truth_world = scene.vertices_world(path)[geometry["corner_indices"]]
                truth_base = MEASURE.transform_points(np.linalg.inv(wb), truth_world)
                truth_optical = MEASURE.transform_points(
                    np.linalg.inv(wc @ np.diag([1, -1, -1, 1])), truth_world
                )
                record["expected_corner_count"] = len(truth_base)
                try:
                    mask = MEASURE.semantic_mask(frame["seg"], "calibration_board")
                    uv = MEASURE.checkerboard_corners(frame["rgb"], mask)
                    depth = MEASURE.sample_depth_bilinear(
                        frame["z"], uv, MEASURE.erode_mask(mask)
                    )
                    restored = MEASURE.backproject(uv, depth, nominal_k, nominal_matrix)
                    statistics = MEASURE.board_statistics(
                        truth_base,
                        restored,
                        uv,
                        depth - truth_optical[:, 2],
                        actual_matrix[:3, 3],
                        nominal_k,
                    )
                    record.update(statistics)
                    record["measured_corner_count"] = len(uv)
                    record["missing_depth_count"] = int((~np.isfinite(depth)).sum())
                    record["status"] = (
                        "PASS"
                        if statistics["gate_4"] == statistics["gate_7a"] == "PASS"
                        and np.isfinite(depth).all()
                        else "FAIL"
                    )
                    record["nominal_residual_uv_px"] = (
                        uv - MEASURE.project(truth_optical, nominal_k)
                    ).tolist()
                    samples[repeat].append((truth_optical, uv, holdout))
                    np.savez_compressed(
                        args.output / f"{name}_samples.npz",
                        truth_base_m=truth_base,
                        truth_optical_m=truth_optical,
                        uv=uv,
                        depth_m=depth,
                        restored_base_m=restored,
                        holdout=holdout,
                    )
                except ValueError as exc:
                    record["reason"] = str(exc)
            stage.RemovePrim(path)
            write_record(args.output / "result.json", result)
        for repeat, sets in enumerate(samples):
            record = {
                "anchor_horizontal_m": distance,
                "repeat": repeat,
                "gate_2a": "FAIL",
                "gate_2b": "FAIL",
            }
            result["intrinsics_fits"].append(record)
            if len(sets) != len(MEASURE.SCREEN_CENTRES_UV):
                record["reason"] = (
                    "Missing screen layout; cannot claim full-screen calibration"
                )
                continue
            points, uv, heldout = (
                np.concatenate([s[i] for s in sets]) for i in range(3)
            )
            record.update(MEASURE.fit_intrinsics(points, uv, heldout, nominal_k))
            if repeat == 0:
                fits_by_distance[distance] = PinholeIntrinsics(
                    nominal_k.width,
                    nominal_k.height,
                    frame_id=nominal_k.frame_id,
                    **record["estimated"],
                )

    # Visibility and distance bin assignment use readback geometry and the
    # independent render fit. The measured height always uses nominal K/mount.
    for distance in MEASURE.DISTANCES_M:
        if distance not in fits_by_distance:
            result["height_panels"].append(
                {
                    "anchor_horizontal_m": distance,
                    "status": "FAIL",
                    "reason": "Rendered K unavailable for grid/bin geometry",
                }
            )
            continue
        rendered_k = fits_by_distance[distance]
        for height in MEASURE.HEIGHTS_M:
            for column in (100.0, 320.0, 540.0):
                visibility = MEASURE.height_visibility(
                    distance, column, height, rendered_k, actual_mount
                )
                record = {
                    "anchor_horizontal_m": distance,
                    "column_px": column,
                    "requested_surface_height_base_m": height,
                    "centre_visibility": visibility,
                    "captures": [],
                    "status": "FAIL",
                }
                result["height_panels"].append(record)
                if visibility["point_base_m"] is None:
                    record.update(
                        status="UNOBSERVED",
                        reason="No forward horizontal-range/column intersection",
                    )
                    continue
                path = scene.height_panel(visibility["point_base_m"], distance, height)
                wb = scene.world_matrix(base_path)
                vertices_world = scene.vertices_world(path)
                truth_height = MEASURE.surface_height_base(vertices_world, wb)
                RUNNER.require(
                    abs(truth_height - height) <= 1e-7,
                    "Composed panel surface differs from requested base height",
                )
                vertices_base = MEASURE.transform_points(
                    np.linalg.inv(wb), vertices_world
                )
                selection = MEASURE.height_grid_selection(
                    vertices_base, rendered_k, actual_matrix
                )
                record["surface_vertices_base_m"] = vertices_base.tolist()
                record["selection_groups"] = selection["groups"]
                record["sample_location_source"] = selection["location_source"]
                name_base = f"height_r{distance:.1f}_h{height:.3f}_u{column:.0f}"
                np.savez_compressed(
                    args.output / f"{name_base}_selection.npz",
                    uv=selection["uv"],
                    truth_base_m=selection["truth_base_m"],
                    bins=selection["bins"],
                )
                for repeat in range(MEASURE.REPEATS):
                    name = f"{name_base}_repeat{repeat}"
                    frame, wb, _ = capture(
                        name,
                        path,
                        "height_panel",
                        require_semantics=bool(len(selection["uv"])),
                    )
                    # Re-read the rendered surface, including all ancestors,
                    # after capture too. Never use a depth-fitted plane as truth.
                    current_height = MEASURE.surface_height_base(
                        scene.vertices_world(path), wb
                    )
                    RUNNER.require(
                        abs(current_height - truth_height) <= 1e-9,
                        "Height surface moved during capture",
                    )
                    try:
                        mask = MEASURE.semantic_mask(frame["seg"], "height_panel")
                    except ValueError:
                        # Expected-visible samples will fail; an entirely
                        # out-of-view panel remains UNOBSERVED, never PASS.
                        mask = np.zeros(frame["z"].shape, dtype=bool)
                    stats = MEASURE.height_grid_statistics(
                        selection,
                        frame["z"],
                        MEASURE.erode_mask(mask),
                        nominal_k,
                        nominal_matrix,
                        truth_height,
                    )
                    record["captures"].append({"capture": name, **stats})
                states = [s["status"] for s in record["captures"]]
                record["status"] = (
                    "PASS"
                    if all(s == "PASS" for s in states)
                    else "UNOBSERVED"
                    if all(s == "UNOBSERVED" for s in states)
                    else "FAIL"
                )
                stage.RemovePrim(path)
                write_record(args.output / "result.json", result)
    finish_result(result)
    for annotator in annotators.values():
        annotator.detach([camera.get_render_product_path()])


def finish_result(result: dict) -> None:
    """Fail closed on incomplete measurements; G1⑥ remains an evidence judgment."""
    fits, boards, heights = (
        result["intrinsics_fits"],
        result["boards"],
        result["height_panels"],
    )
    gates = {
        "1": result["perception_camera_intrinsics"]["status"],
        "2a": "PASS"
        if len(fits) == 18 and all(f["gate_2a"] == "PASS" for f in fits)
        else "FAIL",
        "2b": "PASS"
        if len(fits) == 18 and all(f["gate_2b"] == "PASS" for f in fits)
        else "FAIL",
        "3": result["mount"]["status"],
        "4": "PASS"
        if len(boards) == 162 and all(b.get("gate_4") == "PASS" for b in boards)
        else "FAIL",
        "5": result["gate_5"],
        "7a": "PASS"
        if len(boards) == 162 and all(b["status"] == "PASS" for b in boards)
        else "FAIL",
    }
    covered_ranges = set()
    for panel in heights:
        for capture in panel.get("captures", []):
            covered_ranges.update(
                g["distance_bin"]
                for g in capture["per_distance"]
                if g["status"] == "PASS"
            )
    gates["7a_prime"] = (
        "PASS"
        if len(heights) == 54
        and len(covered_ranges) == 6
        and all(h["status"] in ("PASS", "UNOBSERVED") for h in heights)
        else "FAIL"
    )
    gates["7b"] = "RECORD_ONLY"
    numerical_pass = all(value == "PASS" for key, value in gates.items() if key != "7b")
    # No raster silhouette-centre tolerance is smuggled into the G1 table.
    # A centroid outside the identified marker cannot locate that marker;
    # numeric gate failures instead expose a camera/transform mismatch.
    if any(gates[k] != "PASS" for k in ("1", "2a", "2b", "3", "4")):
        conclusion = "camera_model_or_transform_discrepancy"
    elif result["marker"] and all(
        m["bright_centroid_outside_marker_bbox"] for m in result["marker"]
    ):
        conclusion = "global_brightness_method_discrepancy"
    else:
        conclusion = "historical_discrepancy_not_resolved"
    result["marker_diagnosis"] = {
        "conclusion": conclusion,
        "status": "USER_JUDGMENT_REQUIRED",
        "basis": "Compare identified-marker and global-bright centroids with gates 2-4; a camera failure or a method discrepancy may coexist. Bloom itself is not proven by this comparison.",
        "historical_centroids_uv": [[345.95, 289.16], [347.10, 288.11]],
        "historical_nominal_uv": [320.0, 314.51858496],
    }
    gates["6"] = "USER_JUDGMENT_REQUIRED"
    result["gates"] = gates
    result["numerical_status"] = "PASS" if numerical_pass else "FAIL"
    # The plan explicitly leaves ⑥ to user judgment together with ②–④.
    # Never promote quantitative PASS alone into G1 completion/G2 authorization.
    result["status"] = "REVIEW_REQUIRED" if numerical_pass else "FAIL"
    result["g2_allowed"] = False


def main() -> int:
    args = arguments()
    result = {
        "status": "FAIL",
        "g2_allowed": False,
        "camera_axes": args.camera_axes,
        "protocol": MEASURE.protocol(),
        "base_scene": args.base_scene,
        "forklift_urdf_source": str(args.forklift_urdf),
        "input_provenance": "synthetic",
    }
    app, owns_output = None, False
    try:
        args.output.mkdir(parents=True, exist_ok=False)
        owns_output = True
        result["protocol_sha256"] = hashlib.sha256(
            RUNNER.record_json(result["protocol"]).encode()
        ).hexdigest()
        write_record(args.output / "protocol.json", result["protocol"])
        plan = ROOT / "docs/plans/2026-09-21-perception-detection-closeout.md"
        if plan.is_file():
            result["plan_sha256"] = hashlib.sha256(plan.read_bytes()).hexdigest()
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
        run(app, args, result)
    except Exception as exc:
        result["status"], result["reason"] = "FAIL", str(exc)
        traceback.print_exc(file=sys.stderr)
    finally:
        # Persist before close: the observed Isaac close path may exit Python.
        if owns_output:
            write_record(args.output / "result.json", result)
        print(RUNNER.record_json(result), flush=True)
        if app is not None:
            app.close()
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
