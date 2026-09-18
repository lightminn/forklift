"""Preview Case A docking using prescribed poses and all-box AABB clearances.

No physics integration, contact response, pallet motion or payload is simulated.
MP4/PNG rendering happens only through the CLI, after trajectory validation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING
from xml.etree import ElementTree as ET

import numpy as np
import yaml

from forklift_core.perception.pallet_geometry import (
    check_fork_fit,
    load_pallet_geometry,
    target_insertion_depth_m,
)

if TYPE_CHECKING:
    import mujoco

# 이 값은 EPAL 전용이다 -- T11 표준이 확정되고 preview_docking 이 형상을 매개변수로 받게 되면 이 리터럴도 매개변수가 되어야 한다
REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE = REPO_ROOT / "sim/models/docking_scene.xml"
FORKLIFT = REPO_ROOT / "sim/models/dls08_provisional/forklift.xml"
PALLET = REPO_ROOT / "sim/models/epal6_pallet/pallet.xml"
GEOMETRY = REPO_ROOT / "config/pallet_geometry_epal6.yaml"
PHASES = ("approach", "insert", "lift", "settle")
APPROACH_M = 2.0
STANDOFF_M = 0.10
LIFT_M = 0.040
FPS = 24
DISCLAIMER = "Kinematic preview; contact and payload not simulated"


@dataclass(frozen=True)
class DockFrame:
    """One prescribed pose and its signed geometric distances in metres."""

    index: int
    phase: str
    base_x_m: float
    lift_m: float
    penetration_m: float
    clearance_m: float
    clearance_pair: tuple[str, str]
    insertion_margin_m: float


def _box_bounds(model, data, indices):
    # Include rotated decorative boxes via their conservative world-axis bounds.
    rotations = data.geom_xmat[indices].reshape(-1, 3, 3)
    half = np.einsum("nij,nj->ni", np.abs(rotations), model.geom_size[indices])
    centres = data.geom_xpos[indices]
    return centres - half, centres + half


def _pair_distances(truck_bounds, pallet_bounds):
    """Euclidean separation outside; negative minimum translation inside.

    Arrays have shape (truck boxes, pallet boxes, xyz), in world metres.
    A touching pair has zero distance, including edge/corner contact.
    """
    truck_lo, truck_hi = truck_bounds
    pallet_lo, pallet_hi = pallet_bounds
    gaps = np.maximum(
        pallet_lo[None, :, :] - truck_hi[:, None, :],
        truck_lo[:, None, :] - pallet_hi[None, :, :],
    )
    return np.linalg.norm(np.maximum(gaps, 0), axis=2) + np.minimum(
        np.max(gaps, axis=2), 0
    )


def _insertion_margin(truck_bounds, pallet_bounds):
    """Distance to first box contact during further +x travel at this lift."""
    truck_lo, truck_hi = truck_bounds
    pallet_lo, pallet_hi = pallet_bounds
    yz_overlap = np.all(
        (truck_lo[:, None, 1:] <= pallet_hi[None, :, 1:])
        & (pallet_lo[None, :, 1:] <= truck_hi[:, None, 1:]),
        axis=2,
    )
    ahead = pallet_hi[None, :, 0] >= truck_lo[:, None, 0]
    gaps_x = pallet_lo[None, :, 0] - truck_hi[:, None, 0]
    return float(np.min(gaps_x[yz_overlap & ahead], initial=np.inf))


@dataclass
class _DockScene:
    model: mujoco.MjModel
    data: mujoco.MjData
    truck_boxes: np.ndarray
    pallet_boxes: np.ndarray
    tip_x_m: float
    front_x_m: float

    def pose(self, base_x_m: float, lift_m: float) -> None:
        import mujoco

        mujoco.mj_resetData(self.model, self.data)
        self.data.joint("floating_base").qpos[:] = (base_x_m, 0, 0, 1, 0, 0, 0)
        self.data.joint("fork_lift").qpos[0] = lift_m
        # Kinematics only: do not run mj_forward's collision pass or mj_step.
        mujoco.mj_kinematics(self.model, self.data)
        # Reset clears these world poses; update them without opening a GL context.
        mujoco.mj_camlight(self.model, self.data)

    def measure(self):
        truck = _box_bounds(self.model, self.data, self.truck_boxes)
        pallet = _box_bounds(self.model, self.data, self.pallet_boxes)
        distances = _pair_distances(truck, pallet)
        i, j = np.unravel_index(np.argmin(distances), distances.shape)
        pair = (
            self.model.geom(int(self.truck_boxes[i])).name,
            self.model.geom(int(self.pallet_boxes[j])).name,
        )
        return float(distances[i, j]), pair, _insertion_margin(truck, pallet)


def _load_scene(model_path: Path, pallet_path: Path) -> _DockScene:
    # Set the backend before the first MuJoCo import; no GL context is created.
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco

    parameters = yaml.safe_load((model_path.parent / "parameters.yaml").read_text())
    dimensions = parameters["dimensions"]
    geometry = load_pallet_geometry(GEOMETRY)
    tip_x = dimensions["rear_extent_x_m"] + parameters["catalogue"]["overall_length_m"]
    fit = check_fork_fit(
        geometry,
        fork_spacing_m=dimensions["fork_spacing_m"],
        fork_width_m=dimensions["fork_width_m"],
        fork_thickness_m=dimensions["fork_thickness_m"],
        fork_centre_height_m=dimensions["fork_center_height_m"],
        fork_length_m=tip_x - dimensions["fork_root_x_m"],
        lift_travel_m=dimensions["lift_travel_m"],
    )
    if not fit.fits or fit.lift_required_m > 0:
        raise ValueError("The forks cannot enter the pallet at their lowered height")
    if LIFT_M > dimensions["lift_travel_m"]:
        raise ValueError("The requested lift exceeds the model's lift travel")

    # Resolve the two model references without duplicating any component geometry.
    root = ET.parse(SCENE).getroot()
    root.find("include").set("file", str(model_path.resolve()))
    root.find("asset/model").set("file", str(pallet_path.resolve()))
    front_x = tip_x + APPROACH_M
    root.find("worldbody/body[@name='pallet_placement']").set(
        "pos", f"{front_x + geometry.overall_depth_m / 2:.12g} 0 0"
    )
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    truck_root = model.body("base_link").id
    pallet_root = model.body("pallet").id

    def belongs_to(body_id, ancestor):
        while body_id:
            if body_id == ancestor:
                return True
            body_id = model.body_parentid[body_id]
        return False

    def boxes(ancestor):
        indices = [
            i
            for i in range(model.ngeom)
            if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX
            and model.geom(i).name != "ground"
            and belongs_to(model.geom_bodyid[i], ancestor)
        ]
        if not indices:
            raise ValueError("Both models must contain named box geoms")
        return np.array(indices, dtype=int)

    scene = _DockScene(
        model, data, boxes(truck_root), boxes(pallet_root), tip_x, front_x
    )
    # Refuse stale model files rather than use YAML dimensions to hide their drift.
    for side, sign in (("left", 1), ("right", -1)):
        fork = model.geom(f"{side}_fork_visual").id
        lo, hi = _box_bounds(model, data, np.array([fork]))
        centre = np.array(
            [
                (tip_x + dimensions["fork_root_x_m"]) / 2,
                sign * dimensions["fork_spacing_m"] / 2,
                dimensions["fork_center_height_m"],
            ]
        )
        size = np.array(
            [
                tip_x - dimensions["fork_root_x_m"],
                dimensions["fork_width_m"],
                dimensions["fork_thickness_m"],
            ]
        )
        if not (
            np.allclose((lo[0] + hi[0]) / 2, centre, rtol=0, atol=1e-8)
            and np.allclose(hi[0] - lo[0], size, rtol=0, atol=1e-8)
        ):
            raise ValueError(
                "Forklift XML fork geometry disagrees with parameters.yaml"
            )
    lo, hi = _box_bounds(model, data, scene.pallet_boxes)
    if not np.allclose(
        hi.max(axis=0) - lo.min(axis=0),
        [geometry.overall_depth_m, geometry.overall_width_m, geometry.overall_height_m],
        rtol=0,
        atol=1e-8,
    ):
        raise ValueError("Pallet XML envelope disagrees with the geometry YAML")
    scene.front_x_m = float(lo[:, 0].min())
    return scene


def _plan(scene: _DockScene, frames: int, insertion_m: float) -> list[DockFrame]:
    if isinstance(frames, bool) or not isinstance(frames, int) or frames < 8:
        raise ValueError("frames must be an integer >= 8 (two endpoints per phase)")
    if not math.isfinite(insertion_m) or insertion_m <= 0:
        raise ValueError("insertion_m must be finite and positive")
    scene.pose(0.0, 0.0)
    limit = scene.measure()[2] + scene.tip_x_m - scene.front_x_m
    if insertion_m >= limit:
        raise ValueError(
            f"Requested insertion {insertion_m:.6f} m reaches/exceeds "
            f"the all-box truck limit {limit:.6f} m"
        )
    # Equal phase allocations; any remainder goes to the earlier phases.
    counts = [frames // 4 + (i < frames % 4) for i in range(4)]
    depths = ((-APPROACH_M, -STANDOFF_M), (-STANDOFF_M, insertion_m))
    result = []
    for phase, count in zip(PHASES, counts, strict=True):
        for fraction in np.linspace(0.0, 1.0, count):
            if phase in ("approach", "insert"):
                start, end = depths[PHASES.index(phase)]
                penetration = float(start + fraction * (end - start))
                lift = 0.0
            else:
                penetration = insertion_m
                lift = float(LIFT_M * fraction) if phase == "lift" else LIFT_M
            base_x = scene.front_x_m - scene.tip_x_m + penetration
            scene.pose(base_x, lift)
            clearance, pair, margin = scene.measure()
            if clearance <= 0:
                raise ValueError(
                    f"Frame {len(result)} ({phase}): {pair[0]} / {pair[1]} "
                    f"clearance {clearance:.9f} m is not positive"
                )
            result.append(
                DockFrame(
                    len(result),
                    phase,
                    base_x,
                    lift,
                    penetration,
                    clearance,
                    pair,
                    margin,
                )
            )
    return result


def plan_trajectory(
    model_path: Path,
    pallet_path: Path,
    *,
    frames: int,
    insertion_m: float = target_insertion_depth_m(0.60),
) -> list[DockFrame]:
    """Kinematic dock trajectory. Raises ValueError when the forks cannot fit.

    All visual and collision box geoms are included, except ground. Distances
    use world AABBs; tied pairs are resolved in model order. No renderer is made.
    """
    return _plan(_load_scene(model_path, pallet_path), frames, insertion_m)


def _render(scene: _DockScene, frames: list[DockFrame], output: Path) -> str:
    import mujoco
    from OpenGL import GL
    from PIL import Image, ImageDraw, ImageFont

    width, height = 1280, 720
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    options.sitegroup[:] = 0
    camera = mujoco.MjvCamera()
    # Sit near pocket height: a high camera hides the forks behind the top deck.
    camera.azimuth, camera.elevation = 132, -11
    # The approach reads best from three quarters, but once the forks are inside
    # the only way to see them is through the open pockets from the far face, so
    # the camera swings round across the insert phase and stays there.
    insert_indices = [f.index for f in frames if f.phase == "insert"]
    insert_first = insert_indices[0] if insert_indices else 0
    insert_last = insert_indices[-1] if insert_indices else 0

    def camera_pose(frame):
        if frame.phase == "approach":
            progress = 0.0
        elif frame.phase == "insert" and insert_last > insert_first:
            progress = (frame.index - insert_first) / (insert_last - insert_first)
        else:
            progress = 1.0
        return (
            132.0 + 36.0 * progress,
            -11.0 + 7.0 * progress,
            1.9 - 0.4 * progress,
        )

    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 22)
    except OSError:
        font = ImageFont.load_default()
    command = [
        "ffmpeg",
        "-v",
        "error",
        "-nostdin",
        "-n",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        str(FPS),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-crf",
        "20",
        "-movflags",
        "+faststart",
        str(output / "docking.mp4"),
    ]
    with mujoco.Renderer(scene.model, height=height, width=width) as renderer:
        renderer_name = GL.glGetString(GL.GL_RENDERER)
        name = renderer_name.decode() if renderer_name else "unknown"
        with subprocess.Popen(command, stdin=subprocess.PIPE) as encoder:
            try:
                for frame in frames:
                    scene.pose(frame.base_x_m, frame.lift_m)
                    truck_lo, _ = _box_bounds(
                        scene.model, scene.data, scene.truck_boxes
                    )
                    pallet_lo, pallet_hi = _box_bounds(
                        scene.model, scene.data, scene.pallet_boxes
                    )
                    x_min = truck_lo[:, 0].min()
                    x_max = pallet_hi[:, 0].max()
                    # Aim at the opening band, not the pallet top, and close in
                    # as the gap shuts so the pocket entry fills the frame.
                    camera.lookat[:] = (
                        (x_min + x_max) / 2,
                        0,
                        float(pallet_lo[:, 2].min() + pallet_hi[:, 2].max()) / 2,
                    )
                    azimuth, elevation, near = camera_pose(frame)
                    camera.azimuth, camera.elevation = azimuth, elevation
                    camera.distance = max(near, float(x_max - x_min) * 1.15)
                    renderer.update_scene(
                        scene.data, camera=camera, scene_option=options
                    )
                    image = Image.fromarray(renderer.render())
                    draw = ImageDraw.Draw(image)
                    draw.rectangle((0, height - 110, width, height), fill=(20, 28, 38))
                    lines = (
                        f"{frame.phase} | insertion {frame.penetration_m * 1000:.1f} mm"
                        f" | minimum clearance {frame.clearance_m * 1000:.1f} mm",
                        f"{frame.clearance_pair[0]} / {frame.clearance_pair[1]}",
                        DISCLAIMER,
                    )
                    for row, line in enumerate(lines):
                        draw.text(
                            (20, height - 102 + 32 * row),
                            line,
                            font=font,
                            fill=(235, 240, 245),
                        )
                    if frame.index == len(frames) - 1:
                        image.save(output / "overview.png")
                    encoder.stdin.write(image.tobytes())
            finally:
                encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError("ffmpeg failed to encode docking.mp4")
    return name


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", type=Path, default=FORKLIFT, help="Forklift MJCF path"
    )
    parser.add_argument("--pallet", type=Path, default=PALLET, help="Pallet MJCF path")
    parser.add_argument(
        "--output", type=Path, required=True, help="New output directory"
    )
    parser.add_argument(
        "--frames", type=int, default=96, help="Frames at 24 fps (default: 96)"
    )
    parser.add_argument(
        "--insertion-m",
        type=float,
        default=target_insertion_depth_m(0.60),
        help="Target insertion depth in metres (default: 0.360)",
    )
    parser.add_argument(
        "--backend",
        choices=("egl", "glfw"),
        default="egl",
        help="Rendering backend (default: egl)",
    )
    args = parser.parse_args(argv)
    os.environ["MUJOCO_GL"] = args.backend
    try:
        scene = _load_scene(args.model, args.pallet)
        frames = _plan(scene, args.frames, args.insertion_m)
    except ValueError as error:
        parser.error(str(error))
    # Fail before creating any output or GL context if any pose collides.
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "clearance.json").write_text(
        json.dumps([asdict(frame) for frame in frames], indent=2) + "\n",
        encoding="utf-8",
    )
    renderer_name = _render(scene, frames, args.output)
    import mujoco

    inputs = {
        "geometry": GEOMETRY,
        "parameters": args.model.parent / "parameters.yaml",
        "forklift_model": args.model,
        "pallet_model": args.pallet,
        "scene": SCENE,
        "preview_source": Path(__file__),
    }
    report = {
        "mode": "case_a_kinematic_docking_preview",
        "input": "synthetic_prescribed_poses",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            key: {
                "path": str(path.resolve()),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            for key, path in inputs.items()
        },
        "mujoco_version": mujoco.__version__,
        "backend": args.backend,
        "renderer": renderer_name,
        "video_frames": len(frames),
        "video_fps": FPS,
        "insertion_m": args.insertion_m,
        "pallet_front_x_m": scene.front_x_m,
        "truck_box_count": len(scene.truck_boxes),
        "pallet_box_count": len(scene.pallet_boxes),
        "clearance_method": "signed_world_aabb_separation_all_box_pairs_except_ground",
        "minimum_clearance_m": min(frame.clearance_m for frame in frames),
        "physical_drive_or_payload_validation": False,
        "disclaimer": DISCLAIMER,
    }
    (args.output / "run.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
