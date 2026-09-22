"""Render alternative calibration-board layouts for the power diagnostic.

OPT-IN. Nothing here runs as part of the G1 judging run, and this script emits
no gate, no PASS/FAIL and no G1 result. It exists because section 13.5 of
docs/validation/2026-09-21-g1-calibration-run.md kept the judging layout and
asked a narrower question instead: does an alternative layout still detect an
injected intrinsics error as well as the current one?

Run it in a NEW Isaac Sim Python process with a NEW output directory:

    python sim/isaac/diagnose_layout_power.py \
        --base-scene /path/to/base.usda --output artifacts/layout_power \
        --layouts production,tilted,phase_varied

Then compare the arms on any CPU, with no simulator:

    python sim/isaac/layout_power.py --output compare.json \
        --arm production=artifacts/layout_power/production \
        --arm tilted=artifacts/layout_power/tilted \
        --arm phase_varied=artifacts/layout_power/phase_varied

Why a separate process rather than extra captures inside the judging run: a
board authored mid-run changes the authoring order the measured captures depend
on, and section 12.5 found a modulation locked to that order in one run and
absent in another. The judging run must not become slower or riskier for a
diagnostic. The "production" arm here is captured for a WITHIN-run reference;
the judging run's own arm is recorded separately, in its result.json, and
comparing across the two runs also compares run conditions.

Each board writes board_r<anchor>_p<position>_<suffix>_samples.npz with the
truth optical points, measured corners and the fixed holdout mask - the same
file the judging run writes - so the CPU analysis reads both without change.
The warm-up capture after authoring is written and kept, and excluded from the
fits exactly as the judging run excludes it.

CPU helpers (arguments, layout_plan) import without Isaac; rendering does not.
"""

import argparse
import importlib.util
import json
import os
import sys
import traceback
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


MEASURE = load_module(
    "layout_capture_measure", Path(__file__).with_name("camera_calibration.py")
)
POWER = load_module("layout_capture_power", Path(__file__).with_name("layout_power.py"))
VERIFY = load_module(
    "layout_capture_verify", Path(__file__).with_name("verify_perception_camera.py")
)
DEFAULT_LAYOUTS = ("production", "tilted", "phase_varied")


def _layout_list(value: str) -> tuple:
    return tuple(name.strip() for name in value.split(",") if name.strip())


def _distance_list(value: str) -> tuple:
    return tuple(float(item) for item in value.split(",") if item.strip())


def arguments(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--base-scene", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--layouts",
        type=_layout_list,
        default=DEFAULT_LAYOUTS,
        help="Comma-separated layout arms to render",
    )
    parser.add_argument(
        "--distances",
        type=_distance_list,
        default=MEASURE.DISTANCES_M,
        help="Comma-separated horizontal anchors; the judging anchors by default",
    )
    parser.add_argument("--repeats", type=int, default=MEASURE.REPEATS)
    parser.add_argument("--camera-axes", choices=("world", "usd", "ros"), default="ros")
    args, unknown = parser.parse_known_args(argv)
    if argv is None:
        # Preserve Isaac's launcher flags, as the existing scripts do.
        sys.argv = [sys.argv[0], *unknown]
    return args


def layout_plan(layouts, distances, repeats: int) -> list[dict]:
    """Ordered board authoring plan, one entry per placement of one arm.

    The capture sequence per placement is the judging run's own plan, so the
    warm-up policy cannot drift apart from the measurement it is compared with.
    """
    unknown = [name for name in layouts if name not in POWER.LAYOUT_VARIANTS]
    if unknown:
        raise ValueError(f"Unknown layout variant(s): {', '.join(unknown)}")
    plan = []
    for layout in layouts:
        for distance in distances:
            for position_index, centre_uv in enumerate(MEASURE.SCREEN_CENTRES_UV):
                name = f"board_r{distance:.1f}_p{position_index}"
                plan.append(
                    {
                        "layout": layout,
                        "anchor_horizontal_m": float(distance),
                        "position_index": position_index,
                        "centre_uv": [float(centre_uv[0]), float(centre_uv[1])],
                        "phase_offset_px": list(POWER.phase_offset_px(position_index))
                        if POWER.LAYOUT_VARIANTS[layout]["phase"] == "per_position"
                        else [0.0, 0.0],
                        "captures": [
                            {
                                "name": f"{name}_{step['suffix']}",
                                "suffix": step["suffix"],
                                "repeat": step["repeat"],
                                "measured": step["measured"],
                            }
                            for step in VERIFY.capture_plan(repeats)
                        ],
                    }
                )
    return plan


def write_record(path: Path, value: dict) -> None:
    path.write_text(
        json.dumps(value, indent=2, allow_nan=False, default=_native) + "\n",
        encoding="utf-8",
    )


def _native(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Unsupported record type: {type(value).__name__}")


def run(app, args: argparse.Namespace, result: dict) -> None:
    """Author each arm's boards in its own pass and store the measured corners."""
    import cv2
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.sensors.camera import Camera

    import forklift_core

    adapter = load_module(
        "layout_capture_adapter", ROOT / "sim/isaac/perception_adapter.py"
    )
    rig = load_module("layout_capture_rig", ROOT / "tools/scene_rig.py")
    result["source_sha256"] = VERIFY.RUNNER.source_sha256(
        ROOT, Path(forklift_core.__file__).parent
    )
    cv2.ocl.setUseOpenCL(False)
    result["opencv"] = {
        "version": cv2.__version__,
        "use_opencl": bool(cv2.ocl.useOpenCL()),
    }
    write_record(args.output / "diagnostic.json", result)
    if result["opencv"]["use_opencl"]:
        raise RuntimeError("OpenCV OpenCL disable did not take effect")
    if not omni.usd.get_context().open_stage(args.base_scene):
        raise RuntimeError("Cannot open base scene")
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    base_path = "/World/Forklift/base_link"
    if not stage.GetPrimAtPath(base_path).IsValid():
        raise ValueError(f"Base scene must contain {base_path}")
    scene = VERIFY.CalibrationScene(stage, base_path)
    nominal_mount = adapter.default_base_from_optical()
    nominal_matrix = MEASURE.mount_matrix(nominal_mount)
    raw_k = rig.intrinsics()
    nominal_k = adapter.normalize_isaac_intrinsics(raw_k).integer_index
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    world.scene.add(
        Robot(
            prim_path="/World/Forklift",
            name="forklift",
            position=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
    )
    camera_path = base_path + "/PerceptionCamera"
    camera = Camera(
        prim_path=camera_path, frequency=-1, resolution=(raw_k.width, raw_k.height)
    )
    camera.set_local_pose(
        translation=np.asarray(nominal_mount.translation_m),
        orientation=np.asarray(adapter.xyzw_to_wxyz(rig.OPTICAL_QUATERNION_XYZW)),
        camera_axes=args.camera_axes,
    )
    camera.set_projection_mode("perspective")
    camera.set_lens_distortion_model("pinhole")
    camera.set_focal_length(1.0)
    camera.set_horizontal_aperture(raw_k.width / raw_k.fx, maintain_square_pixels=True)
    world.reset()
    camera.initialize()
    holdout = np.array([(i + j) % 4 == 0 for i in range(7) for j in range(9)])
    annotators = {}
    try:
        MEASURE.attach_annotators(rep, camera, annotators=annotators)
        for _ in range(60):
            world.step(render=True)
        world.pause()
        timeline = omni.timeline.get_timeline_interface()
        result["hidden_gprims"] = scene.hide_existing_geometry()
        for entry in result["plan"]:
            directory = args.output / entry["layout"]
            directory.mkdir(parents=True, exist_ok=True)
            geometry = POWER.variant_layout(
                entry["anchor_horizontal_m"],
                entry["centre_uv"],
                raw_k,
                nominal_matrix,
                variant=entry["layout"],
                position_index=entry["position_index"],
            )
            path = scene.board(geometry)
            wc = scene.world_matrix(camera_path)
            truth_optical = MEASURE.transform_points(
                np.linalg.inv(wc @ np.diag([1, -1, -1, 1])),
                scene.vertices_world(path)[geometry["corner_indices"]],
            )
            entry["board_phase"] = POWER.board_phase(
                MEASURE.project(truth_optical, nominal_k)
            )
            for capture in entry["captures"]:
                frame = MEASURE.capture_semantic_static(
                    rep, annotators, target_label="calibration_board", max_steps=8
                )
                record = {"capture": capture["name"], "corner_count": 0}
                try:
                    mask = MEASURE.semantic_mask(frame["seg"], "calibration_board")
                    uv = MEASURE.checkerboard_corners(frame["rgb"], mask)
                    record["corner_count"] = int(len(uv))
                    np.savez_compressed(
                        directory / f"{capture['name']}_samples.npz",
                        truth_optical_m=truth_optical,
                        uv=uv,
                        holdout=holdout,
                    )
                except ValueError as exc:
                    # A missed board is recorded, never dropped: an arm must
                    # not look quieter because its hard boards vanished.
                    record["reason"] = f"{type(exc).__name__}: {exc}"
                record["measured"] = capture["measured"]
                record["timeline_static"] = not timeline.is_playing()
                entry.setdefault("captures_observed", []).append(record)
            stage.RemovePrim(path)
            write_record(args.output / "diagnostic.json", result)
        result["collection_complete"] = True
    finally:
        write_record(args.output / "diagnostic.json", result)
        result["annotator_cleanup"] = MEASURE.detach_annotators(annotators)
        write_record(args.output / "diagnostic.json", result)


def main() -> int:
    args = arguments()
    result = {
        "purpose": (
            "Alternative calibration-board layouts for the detection-power "
            "diagnostic; not a G1 result and not a gate"
        ),
        "judgment": "none - this run records corners and numbers only",
        "collection_complete": False,
        "input_provenance": "synthetic",
        "pid": os.getpid(),
        "base_scene": args.base_scene,
        "layouts": list(args.layouts),
        "layout_specification": {
            name: POWER.LAYOUT_VARIANTS[name]
            for name in args.layouts
            if name in POWER.LAYOUT_VARIANTS
        },
        "distances_m": list(args.distances),
        "repeats": args.repeats,
        "coordinate_convention": "integer pixel index centres",
        "warmup_policy": VERIFY.WARMUP_EXCLUSION_REASON,
    }
    app, owns_output = None, False
    try:
        result["plan"] = layout_plan(args.layouts, args.distances, args.repeats)
        args.output.mkdir(parents=True, exist_ok=False)
        owns_output = True
        write_record(args.output / "diagnostic.json", result)
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
        run(app, args, result)
    except Exception as exc:
        result["execution_error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc(file=sys.stderr)
    finally:
        if owns_output:
            write_record(args.output / "diagnostic.json", result)
        print(
            json.dumps(
                {
                    "output": str(args.output),
                    "layouts": list(args.layouts),
                    "collection_complete": result["collection_complete"],
                    "execution_error": result.get("execution_error"),
                }
            ),
            flush=True,
        )
        if app is not None:
            app.close()
    return 1 if "execution_error" in result else 0


if __name__ == "__main__":
    raise SystemExit(main())
