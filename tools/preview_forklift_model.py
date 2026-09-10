"""Render a model inspection sheet and optional kinematic pose video.

No time-stepped driving or pallet task is implied by the animated poses.
Select a graphics backend explicitly when the platform requires it.
"""

import argparse
import hashlib
import json
import math
import os
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default()


def frame_label(frame: Image.Image, title: str, detail: str) -> Image.Image:
    draw = ImageDraw.Draw(frame)
    draw.rectangle((0, 0, frame.width, 88), fill=(20, 28, 38))
    draw.text((26, 15), title, fill=(250, 198, 28), font=font(26))
    draw.text((26, 53), detail, fill=(198, 210, 222), font=font(15))
    return frame


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--backend", choices=["egl", "glfw", "osmesa"], default="egl")
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Optional video frames at 24 fps; 0 disables video",
    )
    args = parser.parse_args()
    if not 0 <= args.frames <= 720:
        parser.error("frames must be between 0 and 720")
    os.environ["MUJOCO_GL"] = args.backend
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    options.sitegroup[:] = 0
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.16, 0, 0.44]
    camera.distance = 2.5
    args.output.mkdir(parents=True, exist_ok=False)
    report = {
        "mode": "kinematic_pose_preview",
        "input": "synthetic_joint_positions",
        "mujoco_version": mujoco.__version__,
        "backend": args.backend,
        "scene_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
        "model_sha256": hashlib.sha256(
            (args.model.parent / "forklift.xml").read_bytes()
        ).hexdigest(),
        "video_frames": args.frames,
        "video_fps": 24,
        "physical_drive_or_payload_validation": False,
        "views": [],
    }

    def pose(lift: float, steer: float = 0.0) -> None:
        mujoco.mj_resetData(model, data)
        data.joint("fork_lift").qpos[0] = lift
        data.joint("left_steer").qpos[0] = steer
        data.joint("right_steer").qpos[0] = steer
        mujoco.mj_forward(model, data)

    with mujoco.Renderer(model, height=1000, width=1400) as renderer:
        from OpenGL import GL

        renderer_name = GL.glGetString(GL.GL_RENDERER)
        report["renderer"] = renderer_name.decode() if renderer_name else "unknown"
        pose(0)
        camera.azimuth, camera.elevation = 135, -22
        renderer.update_scene(data, camera=camera, scene_option=options)
        hero = Image.fromarray(renderer.render())
        frame_label(
            hero,
            "DLS08  /  PROVISIONAL SIMULATION MODEL",
            "Catalogue envelope 1.46 x 0.63 x 1.01 m | Part dimensions and dynamics remain estimates",
        )
        hero.save(args.output / "overview.png")

    views = [
        ("FRONT / THREE QUARTER", 135, -22, 0.0, 0.0),
        ("REAR / THREE QUARTER", -35, -23, 0.0, 0.0),
        ("SIDE / WHEELBASE", 90, -3, 0.0, 0.0),
        ("TOP / FORK SPACING", 90, -89, 0.0, 0.0),
        ("FORK RAISED / +0.20 m", 135, -18, 0.20, 0.0),
        ("STEERING POSE / +0.30 rad", 160, -27, 0.08, 0.30),
    ]
    sheet = Image.new("RGB", (1800, 1770), (20, 28, 38))
    with mujoco.Renderer(model, height=590, width=900) as renderer:
        for index, (title, azimuth, elevation, lift, steer) in enumerate(views):
            pose(lift, steer)
            camera.azimuth, camera.elevation = azimuth, elevation
            renderer.update_scene(data, camera=camera, scene_option=options)
            frame = frame_label(
                Image.fromarray(renderer.render()),
                title,
                "Photo-informed geometry | Kinematic inspection pose",
            )
            sheet.paste(frame, ((index % 2) * 900, (index // 2) * 590))
            report["views"].append({"title": title, "lift_m": lift, "steer_rad": steer})
        sheet.save(args.output / "views.png")
    if args.frames:
        command = [
            "ffmpeg",
            "-v",
            "error",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            "960x640",
            "-framerate",
            "24",
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
            str(args.output / "motion_preview.mp4"),
        ]
        with subprocess.Popen(command, stdin=subprocess.PIPE) as encoder:
            with mujoco.Renderer(model, height=640, width=960) as renderer:
                for index in range(args.frames):
                    phase = index / max(1, args.frames - 1)
                    pose(
                        0.20 * math.sin(math.pi * phase) ** 2,
                        0.3 * math.sin(math.tau * phase),
                    )
                    camera.azimuth = 135 + 45 * math.sin(math.tau * phase)
                    camera.elevation = -22
                    renderer.update_scene(data, camera=camera, scene_option=options)
                    frame = frame_label(
                        Image.fromarray(renderer.render()),
                        "DLS08  /  JOINT POSE PREVIEW",
                        "Kinematic animation | Lift/steering assumptions | No autonomous driving",
                    )
                    encoder.stdin.write(frame.tobytes())
            encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError("ffmpeg failed to encode the preview")
    (args.output / "preview.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
