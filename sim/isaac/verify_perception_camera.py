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
LAYOUT_POWER = load_module(
    "perception_camera_layout_power", ROOT / "sim/isaac/layout_power.py"
)
# The layout diagnostic writes here and nowhere else. No gate reads this key.
LAYOUT_POWER_KEY = "layout_power_diagnostic"


def write_record(path: Path, value: dict) -> None:
    path.write_text(RUNNER.record_json(value, indent=2) + "\n", encoding="utf-8")


# Measured transient, NOT an understood root cause. Four diagnostic runs
# (docs/validation/2026-09-21-g1-calibration-run.md, sections 9 and 10) showed
# that a calibration board is authored once per placement and then captured
# repeatedly with bit-identical geometry, depth and semantic segmentation, yet
# the first capture after authoring is systematically offset by about 0.12 px
# in the fitted principal point; from the second capture on the offset drops to
# 0.03-0.08 px with no trend. It reproduces with DLSS on and anti-aliasing off,
# at rt_subframes 4 and 16, and with the board visit order reversed, and more
# samples shrink the scatter but not the mean, so it is deterministic rather
# than sampling noise. Every observed gate 2a failure sits at that first
# capture. Taking one warm-up capture after authoring and keeping it out of the
# fits avoids that measured transient; it does not explain or repair it.
WARMUP_EXCLUSION_REASON = (
    "First capture after authoring geometry: measured ~0.12 px principal-point "
    "transient, mechanism unexplained "
    "(docs/validation/2026-09-21-g1-calibration-run.md, sections 9-10)"
)

# Predeclared before the run: the fitted K that height-panel work depends on is
# taken from this measured repeat. A warm-up capture contributes no samples and
# therefore no fit at all, so it can never be selected here.
SELECTION_FIT_REPEAT = 0


def capture_plan(repeats: int) -> list[dict]:
    """Ordered captures taken after authoring one piece of geometry.

    The warm-up is captured and recorded like any other frame but never feeds a
    fit or a gate, so the measured repeat count is unchanged and finish_result's
    162-board and 18-fit requirements still describe measured captures only.
    """
    return [
        {
            "suffix": "warmup",
            "repeat": None,
            "measured": False,
            "exclusion_reason": WARMUP_EXCLUSION_REASON,
        },
        *(
            {"suffix": f"repeat{repeat}", "repeat": repeat, "measured": True}
            for repeat in range(repeats)
        ),
    ]


def file_capture_record(
    record: dict, step: dict, measured: list, excluded: list
) -> dict:
    """Route one capture's record; a warm-up is set aside, never dropped.

    Both collections receive the same measured statistics. Discarding a
    measurement without recording it would conceal it.
    """
    if step["measured"]:
        measured.append(record)
    else:
        record["excluded_from_measurement"] = True
        record["exclusion_reason"] = step["exclusion_reason"]
        excluded.append(record)
    return record


def selection_fit_reference(
    fits: list, index: int, *, distance: float, repeat: int, role: str
) -> dict:
    """Name the fit a height panel depends on, with the gate that validated it.

    One fitted K has two distinct uses - selecting depth samples and physically
    placing the panel. Recording the role keeps them apart so a later K
    comparison cannot move the panels without saying so.
    """
    return {
        "collection": "intrinsics_fits",
        "index": index,
        "anchor_horizontal_m": distance,
        "repeat": repeat,
        "repeat_kind": "measured",
        "warmup_capture_excluded": True,
        "gate_2a": fits[index]["gate_2a"],
        "role": role,
    }


# Machine-readable gate 7a-prime reason for a selection K that never passed
# gate 2a. It is deliberately NOT an abort: this phase exists to find out
# whether the warm-up fixes gate 2a, and a run that stops before
# finish_result() would carry no gate table at all - it would destroy the
# evidence it was launched to collect. The run completes, the panels stay on
# record as diagnostics, and the gate fails with this reason named.
GATE_7A_PRIME_UNVALIDATED_FIT_REASON = "selection_fit_unvalidated"


def unvalidated_selection_fit_marks(reference: dict) -> dict:
    """Mark panels a fit placed and sampled before it ever passed gate 2a.

    An unvalidated fit must never silently become a measurement input. The
    numbers are kept - they are diagnostics - but they carry the defect with
    them so no later reader can mistake them for a validated measurement.
    """
    if reference["gate_2a"] == "PASS":
        return {}
    return {
        "selection_fit_unvalidated": True,
        "selection_fit_unvalidated_reason": GATE_7A_PRIME_UNVALIDATED_FIT_REASON,
        "selection_fit_unvalidated_detail": (
            f"intrinsics_fits[{reference['index']}] (anchor "
            f"{reference['anchor_horizontal_m']} m, repeat {reference['repeat']}) "
            f"has gate_2a={reference['gate_2a']}; this panel's placement and "
            "sample selection are diagnostic, never a validated measurement"
        ),
    }


# Machine-readable gate 7a-prime reason for a height panel whose geometry,
# composed readback or sample selection failed. It is NOT an abort either, and
# for the same reason: one panel's geometry must not take the other 53 panels'
# measurements with it and leave the run without a gate table. The panel is
# recorded as failed, carries its own reason, and gate 7a-prime names it.
GATE_7A_PRIME_PANEL_GEOMETRY_REASON = "panel_geometry_failed"


# Machine-readable gate 7a-prime reason for a panel whose numbers are on
# record without the provenance that justifies them. An ABSENT record is not a
# validated one: the checks below read a fit's gate 2a and the selection's
# coverage status, so a panel carrying neither passes both by omission. Every
# record the gate reads must be present, and must carry the field the gate
# reads, before that panel's numbers may count as a measurement.
GATE_7A_PRIME_MISSING_PROVENANCE_REASON = "panel_provenance_missing"
PANEL_FIT_REFERENCE_KEYS = ("panel_placement_K_reference", "selection_fit_reference")
# Required record -> the field gate 7a-prime actually reads from it. An empty
# dict is as unvalidated as a missing one, so presence alone is not enough.
PANEL_REQUIRED_RECORDS = {
    PANEL_FIT_REFERENCE_KEYS[0]: "gate_2a",
    PANEL_FIT_REFERENCE_KEYS[1]: "gate_2a",
    "selection_coverage": "status",
}


def panel_provenance_complete(panel: dict) -> bool:
    """Judge whether every record gate 7a-prime reads is on this panel."""
    return all(
        isinstance(panel.get(key), dict) and field in panel[key]
        for key, field in PANEL_REQUIRED_RECORDS.items()
    )


def panel_status(states: list) -> str:
    """Judge one height panel from the statuses of its measured captures.

    An empty list is not agreement: `all([])` is True, so a panel that recorded
    no measured capture would be promoted to PASS by a vacuous quantifier.
    Nothing measured is a failure, never a pass.
    """
    if not states:
        return "FAIL"
    if all(state == "PASS" for state in states):
        return "PASS"
    if all(state == "UNOBSERVED" for state in states):
        return "UNOBSERVED"
    return "FAIL"


def panel_geometry_failure_marks(error: Exception) -> dict:
    """Mark the one panel whose geometry, readback or selection failed.

    The failure is this panel's, so it fails here and nowhere else. Nothing is
    relaxed: the panel can never reach PASS, and gate 7a-prime fails with the
    named reason below.
    """
    return {
        "status": "FAIL",
        "panel_geometry_failed": True,
        "panel_geometry_failed_reason": GATE_7A_PRIME_PANEL_GEOMETRY_REASON,
        "panel_geometry_failed_detail": f"{type(error).__name__}: {error}",
    }


# Gates that keep their measured value in the gate table but are excluded from
# the numerical aggregate. 7b has always been recorded only. 2a joins it by the
# USER's decision of 2026-09-21 - it is not a judgment this code made and not a
# measurement change: gate 2a is still measured, still judged against its own
# unchanged tolerance, and still written to the table with its PASS/FAIL. What
# it loses is the direct veto over numerical_status.
DIAGNOSTIC_ONLY_GATES = {
    "7b": "recorded only; never an input to the numerical aggregate",
    "2a": (
        "user decision 2026-09-21: measured and recorded as before, excluded "
        "from the numerical aggregate only. The production detector is "
        "depth-only - pocket_detector.py has no .rgb reference - and the "
        "production K is a FOV-derived constant, so a rendered-RGB intrinsics "
        "fit is not an input to detection; and the gate was measured to have "
        "no detection power at its own tolerance, a delta = 0.05 px injection "
        "being masked in about 90 % of fits. This removes the direct veto "
        "only: gate 7a-prime still requires gate 2a on the fit references it "
        "reads, so a selection or placement anchor that failed 2a still fails "
        "7a-prime and still blocks."
    ),
}


def layout_power_diagnostic(observations: list, nominal, *, provenance: dict) -> dict:
    """Record how well the judging layout recovers an injected K error.

    Pure computation on the corners this run already measured: no extra board
    is authored, no extra frame is captured, and the run is neither slower nor
    riskier for it. Rendering any OTHER layout is a separate, opt-in script in
    its own Isaac process, because a board authored mid-run changes the
    authoring order the measured captures depend on.

    Nothing reads this key. A failure is recorded instead of raised: the gate
    table is the evidence the run exists to produce, and a diagnostic must
    never be able to destroy it.
    """
    try:
        return LAYOUT_POWER.compare_layout_power(
            {"production": {"observations": observations, "provenance": provenance}},
            nominal,
        )
    except Exception as exc:
        # A degraded record carries the same keys as a healthy one, so a reader
        # that indexes by_delta or caveats does not break on it. Empty and None
        # mean "not computed"; no number is invented to fill the hole.
        return {
            "question": (
                "Does an alternative layout still detect a known intrinsics "
                "error as well as the current one?"
            ),
            "judgment": (
                "none - this diagnostic records numbers. No PASS/FAIL, no "
                "threshold, no gate, and no input to the G1 result."
            ),
            "nominal_intrinsics": None,
            "injection": None,
            "reference_layout": None,
            "arms": {},
            "by_delta": [],
            "caveats": [],
            "provenance": provenance,
            "error": f"{type(exc).__name__}: {exc}",
        }


def save_height_selection(
    directory: Path, name: str, selection: dict, fit_reference: dict
) -> dict:
    """Persist the depth-independent selection and fit provenance before capture."""
    record = {
        "selection_K": selection["selection_K"],
        "selection_K_coordinate_convention": "integer_index_centers",
        "selection_fit_reference": fit_reference,
        "selection_groups": selection["groups"],
        "selection_margin": selection["margin"],
        "selection_panel_region": selection["panel_region"],
        "selection_coverage": selection["coverage"],
        "sample_location_source": selection["location_source"],
    }
    write_record(directory / f"{name}_selection.json", record)
    np.savez_compressed(
        directory / f"{name}_selection.npz",
        uv=selection["uv"],
        truth_base_m=selection["truth_base_m"],
        bins=selection["bins"],
    )
    return record


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

    def height_panel(self, vertices) -> str:
        # The caller supplies the measurement rectangle already extended by its
        # viewing-direction margin, so the rendered silhouette sits outside the
        # region the selection measures. Geometry is decided in the SDK-free
        # measurement module; this only authors it.
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

    # G1 measurement runs on this thread. SB's CALIB_CB_ACCURACY warpAffine can
    # otherwise allocate OpenCL buffers on Isaac's GPU and terminate in a C++
    # destructor, beyond Python try/except. Keep this 640x480 workload on CPU.
    cv2.ocl.setUseOpenCL(False)
    result["opencv_version"] = cv2.__version__
    result["opencv_opencl"] = {
        "requested_use_opencl": False,
        "have_opencl": bool(cv2.ocl.haveOpenCL()),
        "use_opencl": bool(cv2.ocl.useOpenCL()),
    }
    # Persist the actual backend before the first measurement, even if a later
    # native crash prevents Python finally blocks from running.
    write_record(args.output / "result.json", result)
    RUNNER.require(
        not result["opencv_opencl"]["use_opencl"], "OpenCV OpenCL must be disabled"
    )
    annotators = {}
    try:
        # Keep the SDK camera alive until its owned annotators are detached.
        # On an exception the traceback retains the measurement frame instead.
        _camera = _run_measurements(app, args, result, annotators)
    finally:
        # Save gates/partial captures before touching the renderer's teardown.
        try:
            write_record(args.output / "result.json", result)
        finally:
            result["annotator_cleanup"] = MEASURE.detach_annotators(annotators)
            write_record(args.output / "result.json", result)


def _run_measurements(app, args: argparse.Namespace, result: dict, annotators):
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
    nominal_mount = adapter.default_base_from_optical()
    nominal_raw_sdk_k = rig.intrinsics()
    nominal_k = adapter.normalize_isaac_intrinsics(nominal_raw_sdk_k).integer_index
    nominal_matrix = MEASURE.mount_matrix(nominal_mount)
    result["source_sha256"] = RUNNER.source_sha256(
        ROOT, Path(forklift_core.__file__).parent
    )
    result["gate_5"] = "PASS"
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
    RUNNER.verify_camera_intrinsics(camera, nominal_raw_sdk_k, result)
    # RGB/depth array coordinates and all projection/height comparisons below
    # use integer-index centres. Raw getter settings were compared separately.
    result["measurement_intrinsics_coordinate_convention"] = "integer_index_centers"
    MEASURE.attach_annotators(rep, camera, annotators=annotators)
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
    # Warm-up captures are kept here with their own measured statistics. They
    # are excluded from every fit and gate; nothing reads this collection.
    result["warmup_boards"] = []
    result["capture_plan"] = {
        "steps": capture_plan(MEASURE.REPEATS),
        "policy": (
            "One warm-up capture follows every authored board and height panel. "
            "It is measured and recorded like a repeat but excluded from all "
            "fits and gates; the measured repeat count is unchanged."
        ),
        "basis": "docs/validation/2026-09-21-g1-calibration-run.md, sections 9-10",
        "mechanism": (
            "unexplained; this avoids a measured transient, it is not a root-cause fix"
        ),
    }

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
            MEASURE.attach_annotators(rep, camera, annotators=fresh)
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
            probe["annotator_cleanup"] = MEASURE.detach_annotators(fresh)
            errors = [
                entry["error"]
                for entry in probe["annotator_cleanup"].values()
                if entry["status"] == "error"
            ]
            if errors:
                probe["detach_errors"] = errors
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
    # Corner sets for the layout diagnostic, collected as the judging fits are
    # built. They are read only after every gate input is already recorded.
    layout_observations = []
    for distance in MEASURE.DISTANCES_M:
        samples = [[] for _ in range(MEASURE.REPEATS)]
        for position_index, centre_uv in enumerate(MEASURE.SCREEN_CENTRES_UV):
            # Preserve the existing authored targets. Only measurement pixel
            # coordinates change convention; do not move the calibration scene.
            geometry = MEASURE.checkerboard_layout(
                distance, centre_uv, nominal_raw_sdk_k, nominal_matrix
            )
            trace_pipeline(f"before_board_r{distance:.1f}_p{position_index}_authored")
            path = scene.board(geometry, observe=trace_pipeline)
            for step in capture_plan(MEASURE.REPEATS):
                name = f"board_r{distance:.1f}_p{position_index}_{step['suffix']}"
                record = file_capture_record(
                    {
                        "capture": name,
                        "anchor_horizontal_m": distance,
                        "placement_uv": centre_uv,
                        "repeat": step["repeat"],
                        "status": "FAIL",
                    },
                    step,
                    result["boards"],
                    result["warmup_boards"],
                )
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
                    if step["measured"]:
                        samples[step["repeat"]].append((truth_optical, uv, holdout))
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
            layout_observations.append(
                {
                    "anchor_horizontal_m": distance,
                    "repeat": repeat,
                    "position_count": len(sets),
                    "points_optical": points,
                    "uv": uv,
                    "holdout": heldout,
                }
            )
            if repeat == SELECTION_FIT_REPEAT:
                rendered_k = PinholeIntrinsics(
                    nominal_k.width,
                    nominal_k.height,
                    frame_id=nominal_k.frame_id,
                    **record["estimated"],
                )
                fits_by_distance[distance] = (
                    rendered_k,
                    len(result["intrinsics_fits"]) - 1,
                    repeat,
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
        rendered_k, fit_index, fit_repeat = fits_by_distance[distance]
        # One fitted K does two jobs here: it places the panel in the scene and
        # it selects the depth samples. Keep the two uses separately recorded.
        # The fit for this anchor is complete before any panel is placed, so its
        # gate 2a status is known here: every board capture ran in the loop
        # above. A failed gate 2a marks the panels and fails gate 7a-prime in
        # finish_result; it never aborts, so the run still produces a gate table.
        placement_reference = selection_fit_reference(
            result["intrinsics_fits"],
            fit_index,
            distance=distance,
            repeat=fit_repeat,
            role="height_panel_physical_placement",
        )
        selection_reference = selection_fit_reference(
            result["intrinsics_fits"],
            fit_index,
            distance=distance,
            repeat=fit_repeat,
            role="height_grid_sample_selection",
        )
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
                    # Each panel owns its copy of both role records: one shared
                    # dict inserted into all nine panels of an anchor would let
                    # a later per-panel mutation corrupt nine records at once.
                    "panel_placement_K_reference": dict(placement_reference),
                    **unvalidated_selection_fit_marks(placement_reference),
                    "captures": [],
                    "warmup_captures": [],
                    "status": "FAIL",
                }
                result["height_panels"].append(record)
                if visibility["point_base_m"] is None:
                    record.update(
                        status="UNOBSERVED",
                        reason="No forward horizontal-range/column intersection",
                    )
                    continue
                try:
                    geometry = MEASURE.height_panel_geometry(
                        visibility["point_base_m"],
                        distance,
                        height,
                        rendered_k,
                        actual_matrix,
                    )
                except np.linalg.LinAlgError:
                    # numpy raises LinAlgError as a ValueError SUBCLASS, so the
                    # handler below would absorb it. A linear-algebra failure
                    # here is not this panel's geometry: the matrix inverted is
                    # the mount, shared by all 54 panels and already validated
                    # globally by read_mount, so there is no per-panel defect to
                    # record and nothing to protect by continuing. It aborts
                    # once, diagnosably, rather than being relabelled 54 times.
                    raise
                except ValueError as exc:
                    record.update(panel_geometry_failure_marks(exc))
                    continue
                record["panel_geometry"] = geometry
                path = scene.height_panel(geometry["panel_vertices_base_m"])
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
                # The measured rectangle keeps the readback surface height, so
                # the margin never moves the plane the heights are judged on.
                measurement_base = np.column_stack(
                    (
                        np.asarray(geometry["measurement_vertices_base_m"])[:, :2],
                        np.full(4, truth_height),
                    )
                )
                try:
                    # 1e-5 m absorbs USD's float32 point storage only; the
                    # margin itself is millimetres, so an authoring mistake
                    # still fails - on this panel, not on the whole run.
                    if not np.allclose(
                        vertices_base,
                        geometry["panel_vertices_base_m"],
                        rtol=0,
                        atol=1e-5,
                    ):
                        raise ValueError(
                            "Composed panel differs from the measurement region "
                            "plus margin"
                        )
                    selection = MEASURE.height_grid_selection(
                        vertices_base,
                        rendered_k,
                        actual_matrix,
                        measurement_vertices_base=measurement_base,
                    )
                except np.linalg.LinAlgError:
                    # Same narrowing as above: the ValueError handler must not
                    # relabel a linear-algebra failure of the shared transform
                    # as this one panel's selection defect.
                    raise
                except ValueError as exc:
                    record.update(panel_geometry_failure_marks(exc))
                    stage.RemovePrim(path)
                    continue
                record["surface_vertices_base_m"] = vertices_base.tolist()
                record["measurement_vertices_base_m"] = measurement_base.tolist()
                name_base = f"height_r{distance:.1f}_h{height:.3f}_u{column:.0f}"
                record.update(
                    save_height_selection(
                        args.output, name_base, selection, dict(selection_reference)
                    )
                )
                write_record(args.output / "result.json", result)
                for step in capture_plan(MEASURE.REPEATS):
                    name = f"{name_base}_{step['suffix']}"
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
                    file_capture_record(
                        {"capture": name, **stats},
                        step,
                        record["captures"],
                        record["warmup_captures"],
                    )
                record["status"] = panel_status(
                    [capture["status"] for capture in record["captures"]]
                )
                stage.RemovePrim(path)
                write_record(args.output / "result.json", result)
    # Last, after every judged measurement is already on record.
    result[LAYOUT_POWER_KEY] = layout_power_diagnostic(
        layout_observations,
        nominal_k,
        provenance={
            "source": "this run's measured calibration boards",
            "extra_captures": 0,
            "warmup_captures_excluded": True,
            "camera_axes": args.camera_axes,
            "protocol_version": result["protocol"]["version"],
        },
    )
    finish_result(result)
    return camera


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
    # A panel's captures supply distance coverage whatever the panel's own
    # status, so "what this gate measured with" is wider than "what passed".
    # Both must carry their provenance; a panel that exited before any capture
    # supplies nothing and needs none.
    measurement_inputs = []
    for panel in heights:
        contributed = {
            g["distance_bin"]
            for capture in panel.get("captures", [])
            for g in capture["per_distance"]
            if g["status"] == "PASS"
        }
        covered_ranges.update(contributed)
        if panel["status"] == "PASS" or contributed:
            measurement_inputs.append(panel)
    # Every defect keeps its own reason: a provenance failure must stay visible
    # beside a coverage or measurement failure instead of absorbing it.
    # One fitted K is recorded once per role - placing the panel and selecting
    # its samples - precisely so a later change can move the two apart. Every
    # recorded role is checked, or a selection-only defect would pass unseen.
    unvalidated_fits = []
    for panel in heights:
        for key in PANEL_FIT_REFERENCE_KEYS:
            reference = panel.get(key)
            if not isinstance(reference, dict) or reference.get("gate_2a") == "PASS":
                continue
            if reference not in unvalidated_fits:
                unvalidated_fits.append(reference)
    reasons = []
    if len(heights) != 54:
        reasons.append("incomplete_panel_set")
    if len(covered_ranges) != 6:
        reasons.append("distance_bin_not_covered")
    if any(panel["status"] not in ("PASS", "UNOBSERVED") for panel in heights):
        reasons.append("panel_measurement_failed")
    if any(panel.get("panel_geometry_failed") for panel in heights):
        reasons.append(GATE_7A_PRIME_PANEL_GEOMETRY_REASON)
    if any(
        isinstance(panel.get("selection_coverage"), dict)
        and panel["selection_coverage"].get("status") == "FAIL"
        for panel in heights
    ):
        reasons.append("selection_coverage_lost")
    # A malformed coverage record reaches the line above as "not FAIL"; it is
    # caught here instead, for every panel whose numbers this gate used.
    if any(not panel_provenance_complete(panel) for panel in measurement_inputs):
        reasons.append(GATE_7A_PRIME_MISSING_PROVENANCE_REASON)
    if unvalidated_fits:
        reasons.append(GATE_7A_PRIME_UNVALIDATED_FIT_REASON)
    gates["7a_prime"] = "FAIL" if reasons else "PASS"
    result["gate_7a_prime_detail"] = {
        "status": gates["7a_prime"],
        "reasons": reasons,
        "unvalidated_selection_fits": unvalidated_fits,
    }
    gates["7b"] = "RECORD_ONLY"
    # The excluded set is named and recorded, so a reader of result.json alone
    # sees which gates were measured but kept out of the aggregate, and why.
    result["diagnostic_only_gates"] = dict(DIAGNOSTIC_ONLY_GATES)
    numerical_pass = all(
        value == "PASS"
        for key, value in gates.items()
        if key not in DIAGNOSTIC_ONLY_GATES
    )
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
