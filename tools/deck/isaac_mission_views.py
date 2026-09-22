"""Render presentation views from one copied-back Isaac perception mission.

Usage: python tools/deck/isaac_mission_views.py RUN_DIR OUTPUT_DIR
"""

import argparse
import json
import subprocess
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from forklift_core.geometry import rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.overlay import (
    ESTIMATE_COLOUR,
    opening_corners_m,
    project_point,
)
from forklift_core.perception.pocket_observation import Pocket
from forklift_core.sensors.rgbd import PinholeIntrinsics

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))
from mission_views import (  # noqa: E402
    depth_colormap,
    phase_label,
    project_world_opening,
)

FONT_PATH = "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc"


@lru_cache
def font(size: int) -> ImageFont.FreeTypeFont:
    """Load a Korean-capable font, with a clear error when unavailable."""
    return ImageFont.truetype(FONT_PATH, size)


def opening_world_corners(perception: dict) -> list[list[list[float]]]:
    """Freeze accepted estimated openings in world coordinates."""
    observed = perception["pocket_observation"]
    position, quaternion = perception["capture_diagnostics"]["accepted_pose"]
    w, x, y, z = quaternion
    world_from_base = rotation_matrix_from_quaternion_xyzw([x, y, z, w])
    corners = []
    for name in ("left", "right"):
        pocket = Pocket(**observed[name])
        base_corners = np.asarray(
            opening_corners_m(pocket, observed["insertion_yaw_rad"])
        )
        corners.append((base_corners @ world_from_base.T + position).tolist())
    return corners


def draw_openings(image: Image.Image, polygons: list) -> None:
    """Draw only estimated openings, allowing PIL to clip off-image lines."""
    painter = ImageDraw.Draw(image)
    for polygon in polygons:
        if polygon is not None:
            painter.line([*polygon, polygon[0]], fill=ESTIMATE_COLOUR, width=3)


def draw_label_strip(image: Image.Image) -> None:
    """Place the two required Korean labels over a narrow top strip."""
    painter = ImageDraw.Draw(image)
    painter.rectangle((0, 0, 1279, 35), fill=(12, 18, 24))
    label_font = font(22)
    painter.text((12, 4), "RGB · 검출한 포켓", font=label_font, fill="white")
    painter.text((652, 4), "깊이", font=label_font, fill="white")


def encode_frames(
    source: Path,
    output: Path,
    frames: list[dict],
    *,
    height: int,
    stride: int,
    perception: bool,
    world_openings: list | None = None,
    mount: dict | None = None,
) -> None:
    """Decode source in order and encode selected labelled RGB frames."""
    width = 1280
    decoder = subprocess.Popen(
        [
            "ffmpeg",
            "-v",
            "error",
            "-nostdin",
            "-i",
            str(source),
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-",
        ],
        stdout=subprocess.PIPE,
    )
    fps = 30 if perception else 20
    encoder = subprocess.Popen(
        [
            "ffmpeg",
            "-v",
            "error",
            "-nostdin",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            f"{width}x{height}",
            "-r",
            str(fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-crf",
            "28",
            "-maxrate",
            "500k",
            "-bufsize",
            "1000k",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output),
        ],
        stdin=subprocess.PIPE,
    )
    frame_bytes = width * height * 3
    written = 0
    try:
        for index, entry in enumerate(frames):
            raw = decoder.stdout.read(frame_bytes)
            if len(raw) != frame_bytes:
                raise RuntimeError(f"Video ended before logged frame {index}: {source}")
            if perception:
                selected = entry["phase"] in {"approach", "insert"}
                # The mission may have intermediate labels only if the runner changes.
            else:
                selected = index % stride == 0
            if not selected or (perception and index % stride != 0):
                continue
            image = Image.fromarray(
                np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3).copy()
            )
            if perception:
                polygons = [
                    project_world_opening(
                        corners,
                        entry["base_position_m"],
                        entry["base_orientation_wxyz"],
                        mount,
                        mount["intrinsics"],
                    )
                    for corners in world_openings
                ]
                draw_openings(image, polygons)
                draw_label_strip(image)
            else:
                painter = ImageDraw.Draw(image)
                painter.rectangle((0, 0, 96, 34), fill=(12, 18, 24))
                painter.text(
                    (10, 3), phase_label(entry["phase"]), font=font(22), fill="white"
                )
            encoder.stdin.write(image.tobytes())
            written += 1
        if decoder.stdout.read(1):
            raise ValueError(f"Video has more frames than frames.jsonl: {source}")
        if not written:
            raise ValueError(f"No selected frames for {output.name}")
    finally:
        decoder.stdout.close()
        encoder.stdin.close()
        decoder_code = decoder.wait()
        encoder_code = encoder.wait()
    if decoder_code or encoder_code:
        raise RuntimeError(
            f"ffmpeg failed: decode={decoder_code}, encode={encoder_code}"
        )


def render(run_dir: Path, output_dir: Path) -> None:
    """Create the five presentation artifacts from a matching successful run."""
    result = json.loads((run_dir / "result.json").read_text())
    if not result["success"] or "perception_mount" not in result["extra_views"]:
        raise ValueError("A successful perception run with extra views is required")
    frames = [
        json.loads(line) for line in (run_dir / "frames.jsonl").read_text().splitlines()
    ]
    if not frames or any(frame["frame"] != index for index, frame in enumerate(frames)):
        raise ValueError("frames.jsonl must match video frame order")
    output_dir.mkdir(parents=True, exist_ok=True)
    perception = result["perception"]
    mount = result["extra_views"]["perception_mount"]
    world_openings = opening_world_corners(perception)
    capture_number = len(result["observation_attempts"])
    rgb = Image.open(run_dir / f"perception_capture_{capture_number}_rgb.png").convert(
        "RGB"
    )
    depth = np.load(run_dir / f"perception_capture_{capture_number}_depth_m.npy")
    if rgb.size != (640, 480) or depth.shape != (480, 640):
        raise ValueError("Accepted capture must be 640x480 RGB and depth")
    observed = perception["pocket_observation"]
    from forklift_core.geometry import RigidTransform

    intrinsics = PinholeIntrinsics(**mount["intrinsics"])
    transform = RigidTransform(
        intrinsics.frame_id, "base_link", mount["rotation"], mount["translation_m"]
    )
    polygons = []
    for name in ("left", "right"):
        corners = opening_corners_m(
            Pocket(**observed[name]), observed["insertion_yaw_rad"]
        )
        pixels = [project_point(corner, intrinsics, transform) for corner in corners]
        polygons.append(pixels if all(pixel is not None for pixel in pixels) else None)
    still = Image.new("RGB", (1280, 480))
    still.paste(rgb, (0, 0))
    still.paste(Image.fromarray(depth_colormap(depth)), (640, 0))
    draw_openings(still, polygons)
    draw_label_strip(still)
    still.save(output_dir / "observe_pockets.png")
    encode_frames(
        run_dir / "view_perception.mp4",
        output_dir / "perception_pockets.mp4",
        frames,
        height=480,
        stride=2,
        perception=True,
        world_openings=world_openings,
        mount=mount,
    )
    for name, source in (
        ("quarter", "view_quarter.mp4"),
        ("chase", "view_chase.mp4"),
        ("overhead", "transport.mp4"),
    ):
        encode_frames(
            run_dir / source,
            output_dir / f"{name}.mp4",
            frames,
            height=720,
            stride=3,
            perception=False,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    render(args.run_dir, args.output_dir)


if __name__ == "__main__":
    main()
