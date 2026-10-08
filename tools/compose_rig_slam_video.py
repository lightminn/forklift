"""Compose the rig SLAM comparison video for one record, synchronised by time.

    python tools/compose_rig_slam_video.py --record <record> \
        --replay "LiDAR (slam_toolbox)=<dir>" --replay "Vision (4 RGB-D)=<dir>" \
        --replay "Fusion=<dir>" --title "..." --output out.mp4

Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md D7. Panels at the same
simulation instant (1920 x 1080):
  top left      the truck from behind (chase.mp4, display only),
  top right     the four rig cameras (front, rear, left, right),
  bottom left   the hall from above: ground-truth outlines of the items, the
                true path and each replay's online estimate,
  bottom right  each estimate's start-aligned position error over time (log
                scale), the number the report tabulates.
Each replay directory holds error_series.npz from tools/evaluate_rig_slam.py.
A sensor blackout of the condition is shown as a banner over the cameras or
the map. Needs ffmpeg and Pillow.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
PANEL = (960, 510)
TOP = 60
# Video panels are dark: the dark steps of the report palette (dataviz
# reference), in --replay order -- pass L-ST, V4, F to match the report's
# orange, aqua and blue.
COLOURS = [
    (217, 89, 38),
    (25, 158, 112),
    (57, 135, 229),
    (201, 133, 0),
    (213, 81, 129),
]
TRUTH = (250, 250, 250)
FONTS = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
)
CAMERA_LABELS = {"front": "전방", "rear": "후방", "left": "좌측", "right": "우측"}


def font(size: int):
    for path in FONTS:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def frames(path: Path, width: int, height: int):
    process = subprocess.Popen(
        [
            "ffmpeg",
            "-loglevel",
            "error",
            "-i",
            str(path),
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-",
        ],
        stdout=subprocess.PIPE,
    )
    size = width * height * 3
    try:
        while True:
            data = process.stdout.read(size)
            if len(data) < size:
                return
            yield np.frombuffer(data, np.uint8).reshape(height, width, 3)
    finally:
        process.kill()
        process.wait()


def latest(stamps: np.ndarray, t: float) -> int:
    return max(0, int(np.searchsorted(stamps, t + 1e-9, side="right")) - 1)


class MapPanel:
    """Top-down hall view: item outlines once, paths drawn per frame."""

    def __init__(self, meta: dict, truth_xy: np.ndarray):
        hall = meta["hall"]
        self.x0, self.x1 = hall["x_min_m"] - 1.0, hall["x_max_m"] + 1.0
        self.y0, self.y1 = hall["y_min_m"] - 1.0, hall["y_max_m"] + 1.0
        pw, ph = PANEL
        self.scale = min(
            (pw - 20) / (self.x1 - self.x0), (ph - 20) / (self.y1 - self.y0)
        )
        self.ox = (pw - self.scale * (self.x1 - self.x0)) / 2
        self.oy = (ph - self.scale * (self.y1 - self.y0)) / 2
        base = Image.new("RGB", PANEL, (28, 31, 38))
        draw = ImageDraw.Draw(base)
        scan_height = meta["laser"]["mount_xyz_m"][2]
        for item in meta["obstacles"]:
            if item["base_m"] > 0:
                continue
            c, s = math.cos(item["yaw_rad"]), math.sin(item["yaw_rad"])
            corners = [
                self.px(
                    item["x_m"]
                    + c * a * item["length_m"] / 2
                    - s * b * item["width_m"] / 2,
                    item["y_m"]
                    + s * a * item["length_m"] / 2
                    + c * b * item["width_m"] / 2,
                )
                for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))
            ]
            tall = any(
                o["base_m"] + o["height_m"] > scan_height
                for o in meta["obstacles"]
                if abs(o["x_m"] - item["x_m"]) < item["length_m"] / 2
                and abs(o["y_m"] - item["y_m"]) < item["width_m"] / 2
            )
            draw.polygon(corners, fill=(70, 74, 84) if tall else (48, 52, 60))
        draw.line(
            [self.px(x, y) for x, y in truth_xy[::5]], fill=(85, 90, 100), width=1
        )
        self.base = base

    def px(self, x: float, y: float) -> tuple[float, float]:
        return (
            self.ox + (x - self.x0) * self.scale,
            PANEL[1] - (self.oy + (y - self.y0) * self.scale),
        )

    def zoom(self, k: int, truth: np.ndarray, estimates: list, half_m: float = 1.5):
        """A 3 m x 3 m window centred on the true pose: the centimetre view."""
        size = 222  # fits the margin right of the hall map
        scale = size / (2 * half_m)
        cx, cy = truth[k, 0], truth[k, 1]
        image = Image.new("RGB", (size, size), (18, 20, 25))
        draw = ImageDraw.Draw(image)

        def px(x, y):
            return size / 2 + (x - cx) * scale, size / 2 - (y - cy) * scale

        for g in np.arange(-half_m, half_m + 1e-9, 0.5):
            draw.line(
                [px(cx + g, cy - half_m), px(cx + g, cy + half_m)], fill=(36, 39, 46)
            )
            draw.line(
                [px(cx - half_m, cy + g), px(cx + half_m, cy + g)], fill=(36, 39, 46)
            )
        start = max(0, k - 200)  # the last 20 s
        paths = [(TRUTH, truth)] + list(zip(COLOURS, estimates, strict=False))
        for colour, path in paths:
            seg = path[start : k + 1]
            seg = seg[np.isfinite(seg).all(axis=1)]
            if len(seg) > 1:
                draw.line([px(x, y) for x, y, _ in seg], fill=colour, width=2)
            if np.isfinite(path[k]).all():
                u, v = px(path[k, 0], path[k, 1])
                draw.ellipse([u - 5, v - 5, u + 5, v + 5], fill=colour)
        draw.rectangle([0, 0, size - 1, size - 1], outline=(120, 124, 135))
        draw.text(
            (8, 6),
            "로봇 주변 3 m × 3 m",
            fill=(200, 200, 200),
            font=font(14),
        )
        return image

    def render(
        self, k: int, truth: np.ndarray, estimates: list, labels: list
    ) -> Image.Image:
        image = self.base.copy()
        draw = ImageDraw.Draw(image)
        draw.line(
            [self.px(x, y) for x, y, _ in truth[: k + 1 : 2]], fill=TRUTH, width=2
        )
        for colour, est in zip(COLOURS, estimates, strict=False):
            path = est[: k + 1 : 2]
            path = path[np.isfinite(path).all(axis=1)]
            if len(path) > 1:
                draw.line([self.px(x, y) for x, y, _ in path], fill=colour, width=2)
        for colour, pose in [(TRUTH, truth[k])] + [
            (c, e[k]) for c, e in zip(COLOURS, estimates, strict=False)
        ]:
            if not np.isfinite(pose).all():
                continue
            u, v = self.px(pose[0], pose[1])
            du, dv = 9 * math.cos(pose[2]), -9 * math.sin(pose[2])
            draw.ellipse([u - 4, v - 4, u + 4, v + 4], fill=colour)
            draw.line([u, v, u + du, v + dv], fill=colour, width=2)
        image.paste(self.zoom(k, truth, estimates), (PANEL[0] - 228, PANEL[1] - 262))
        f = font(19)
        legend = [("정답", TRUTH)] + list(zip(labels, COLOURS, strict=False))
        for i, (name, colour) in enumerate(legend):
            draw.rectangle([14, 12 + 26 * i, 32, 28 + 26 * i], fill=colour)
            draw.text((40, 8 + 26 * i), name, fill=(235, 235, 235), font=f)
        draw.text(
            (PANEL[0] - 220, PANEL[1] - 30),
            "위에서 본 공장 홀 (북쪽 위)",
            fill=(170, 170, 170),
            font=font(16),
        )
        return image


class ErrorPanel:
    LO, HI = 0.005, 3.0

    def __init__(self, stamps, errors, labels, end_t):
        self.stamps, self.errors, self.labels, self.end_t = (
            stamps,
            errors,
            labels,
            end_t,
        )
        self.left, self.right, self.top, self.bottom = (
            80,
            PANEL[0] - 30,
            50,
            PANEL[1] - 50,
        )

    def y(self, e):
        e = np.clip(e, self.LO, self.HI)
        r = (np.log10(e) - math.log10(self.LO)) / (
            math.log10(self.HI) - math.log10(self.LO)
        )
        return self.bottom - r * (self.bottom - self.top)

    def x(self, t):
        return self.left + t / self.end_t * (self.right - self.left)

    def render(self, k: int, windows) -> Image.Image:
        image = Image.new("RGB", PANEL, (22, 24, 30))
        draw = ImageDraw.Draw(image)
        f = font(17)
        for start, end, colour in windows:
            draw.rectangle(
                [self.x(start), self.top, self.x(min(end, self.end_t)), self.bottom],
                fill=colour,
            )
        for value in (0.01, 0.03, 0.1, 0.3, 1.0, 3.0):
            yy = self.y(value)
            draw.line([self.left, yy, self.right, yy], fill=(55, 58, 66))
            draw.text((10, yy - 11), f"{value:g} m", fill=(170, 170, 170), font=f)
        for value in range(0, int(self.end_t) + 1, 60):
            draw.text(
                (self.x(value) - 12, self.bottom + 8),
                f"{value}s",
                fill=(150, 150, 150),
                font=f,
            )
        t = self.stamps[k]
        for colour, err in zip(COLOURS, self.errors, strict=False):
            seg = np.arange(0, k + 1, 3)
            pts = [
                (self.x(self.stamps[i]), self.y(err[i]))
                for i in seg
                if np.isfinite(err[i])
            ]
            if len(pts) > 1:
                draw.line(pts, fill=colour, width=2)
        draw.line([self.x(t), self.top, self.x(t), self.bottom], fill=(200, 200, 200))
        draw.text(
            (self.left, 12),
            "위치 오차 (출발 자세만 맞춤, 로그 축)",
            fill=(235, 235, 235),
            font=font(20),
        )
        for i, (colour, label, err) in enumerate(
            zip(COLOURS, self.labels, self.errors, strict=False)
        ):
            value = err[k]
            text = (
                f"{label}: {value * 100:5.1f} cm"
                if np.isfinite(value)
                else f"{label}: —"
            )
            ty = self.top + 6 + 24 * i
            draw.line(
                [self.right - 400, ty + 11, self.right - 380, ty + 11],
                fill=colour,
                width=4,
            )
            draw.text((self.right - 372, ty), text, fill=(235, 235, 235), font=f)
        return image


def camera_mosaic(
    record: Path, cameras: list[str], index: int, off: bool
) -> Image.Image:
    tile_w, tile_h = PANEL[0] // 2, PANEL[1] // 2
    image = Image.new("RGB", PANEL, (0, 0, 0))
    draw = ImageDraw.Draw(image)
    for i, name in enumerate(cameras):
        x, y = (i % 2) * tile_w, (i // 2) * tile_h
        path = record / "rig" / name / f"{index:06d}.jpg"
        if path.exists() and not off:
            tile = Image.open(path).convert("RGB")
            # 4:3 -> the tile: crop the middle rows, keep the full width.
            crop_h = round(tile.width * tile_h / tile_w)
            top = (tile.height - crop_h) // 2
            tile = tile.crop((0, top, tile.width, top + crop_h)).resize(
                (tile_w, tile_h)
            )
            image.paste(tile, (x, y))
        else:
            draw.rectangle([x, y, x + tile_w - 1, y + tile_h - 1], fill=(15, 15, 18))
        draw.rectangle([x + 6, y + 6, x + 74, y + 34], fill=(0, 0, 0))
        draw.text(
            (x + 12, y + 7),
            CAMERA_LABELS.get(name, name),
            fill=(255, 255, 255),
            font=font(20),
        )
    if off:
        draw.rectangle(
            [
                PANEL[0] // 2 - 230,
                PANEL[1] // 2 - 34,
                PANEL[0] // 2 + 230,
                PANEL[1] // 2 + 34,
            ],
            fill=(150, 20, 20),
        )
        draw.text(
            (PANEL[0] // 2 - 210, PANEL[1] // 2 - 24),
            "카메라 끊김 (악조건 구간)",
            fill=(255, 255, 255),
            font=font(34),
        )
    return image


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument(
        "--replay", action="append", required=True, help="label=directory"
    )
    parser.add_argument("--title", required=True)
    parser.add_argument("--condition", default="nominal")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed-up")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument(
        "--meta",
        type=Path,
        default=None,
        help="meta.json to use (a run that stopped early writes none; the same "
        "seed's recording has the same hall and items)",
    )
    parser.add_argument(
        "--palette",
        default="0,1,2,3,4",
        help="COLOURS index per --replay, so a lone fusion run keeps its blue",
    )
    args = parser.parse_args(argv)
    global COLOURS
    COLOURS = [COLOURS[int(i)] for i in args.palette.split(",")]
    from forklift_core.localization.sensor_conditions import condition

    cond = condition(args.condition)
    meta = json.loads((args.meta or args.record / "meta.json").read_text())
    chase = json.loads((args.record / "chase_frames.json").read_text())
    chase_times = np.array([f["time_s"] for f in chase["frames"]])
    labels, series = [], []
    for item in args.replay:
        label, _, directory = item.partition("=")
        labels.append(label)
        series.append(np.load(Path(directory) / "error_series.npz"))
    stamps = series[0]["stamps_s"]
    truth = series[0]["truth"]
    estimates = [s["estimate"] for s in series]
    errors = [s["error_m"] for s in series]
    cameras = meta["depth_rig"]["config"]["cameras"].keys()
    cameras = [c for c in ("front", "rear", "left", "right") if c in cameras]
    map_panel = MapPanel(meta, truth[:, :2])
    error_panel = ErrorPanel(stamps, errors, labels, float(stamps[-1]))
    windows = [(a, b, (70, 30, 30)) for a, b in cond.lidar_blackout_s] + [
        (a, b, (30, 30, 80)) for a, b in cond.camera_blackout_s
    ]
    fps = chase["fps"]
    encoder = subprocess.Popen(
        [
            "ffmpeg",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            f"{W}x{H}",
            "-r",
            str(fps),
            "-i",
            "-",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            str(args.output),
        ],
        stdin=subprocess.PIPE,
    )
    step = max(1, int(round(args.speed)))
    written = 0
    title_font, small = font(30), font(20)
    for n, chase_rgb in enumerate(frames(args.record / "chase.mp4", 1280, 720)):
        if n >= len(chase_times):
            break
        if n % step:
            continue
        t = chase_times[n]
        k = latest(stamps, t)
        canvas = Image.new("RGB", (W, H), (12, 13, 16))
        draw = ImageDraw.Draw(canvas)
        draw.text((24, 12), args.title, fill=(245, 245, 245), font=title_font)
        lidar_off = not cond.lidar_available(t)
        camera_off = not cond.cameras_available(t)
        status = (
            f"t = {t:6.1f} s"
            + ("   ·   LiDAR 끊김" if lidar_off else "")
            + ("   ·   카메라 끊김" if camera_off else "")
        )
        draw.text(
            (W - 520, 18),
            status,
            fill=(255, 120, 120) if (lidar_off or camera_off) else (200, 200, 200),
            font=small,
        )
        canvas.paste(Image.fromarray(chase_rgb).resize(PANEL), (0, TOP))
        canvas.paste(
            camera_mosaic(args.record, cameras, k, camera_off), (PANEL[0], TOP)
        )
        map_image = map_panel.render(k, truth, estimates, labels)
        if lidar_off:
            md = ImageDraw.Draw(map_image)
            md.rectangle([PANEL[0] - 300, 10, PANEL[0] - 10, 52], fill=(150, 20, 20))
            md.text(
                (PANEL[0] - 286, 14),
                "LiDAR 끊김 구간",
                fill=(255, 255, 255),
                font=font(26),
            )
        canvas.paste(map_image, (0, TOP + PANEL[1]))
        canvas.paste(error_panel.render(k, windows), (PANEL[0], TOP + PANEL[1]))
        encoder.stdin.write(np.asarray(canvas, np.uint8).tobytes())
        written += 1
        if args.max_frames and written >= args.max_frames:
            break
    encoder.stdin.close()
    if encoder.wait() != 0:
        raise SystemExit("ffmpeg failed")
    (args.output.with_suffix(".json")).write_text(
        json.dumps(
            {
                "frames": written,
                "fps": fps,
                "speed": step,
                "replays": args.replay,
                "condition": args.condition,
            },
            indent=1,
        )
    )
    print(f"wrote {written} frames to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
