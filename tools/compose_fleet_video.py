"""Compose the fleet overview video: Isaac's top view plus truck trails, labels and status.

    python tools/compose_fleet_video.py --run <run dir> --output <out.mp4> [--title "..."]

Plan: docs/plans/2026-10-08-fleet-five-forklifts.md F4. Reads overview.mp4,
camera.json, fleet_log.npz and result.json written by sim/isaac/run_fleet.py.
Points are projected with the recorded pinhole camera (eye, target, up, focal
length and apertures), so overlays sit where Isaac drew the floor. Trails are
the true rear-axle positions; a truck waiting on another is ringed and linked
to the one it waits for. 1920 x 1080: the 1280 x 720 view scaled to 1440 x 810
on the left, a status panel on the right, time and legend below.
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
VIEW_W, VIEW_H = 1440, 810
COLOURS = [
    (230, 85, 60),
    (60, 140, 230),
    (245, 180, 40),
    (90, 190, 110),
    (170, 100, 220),
]
STAGE_KO = {
    "observe": "통로로 이동",
    "approach": "팔레트 접근",
    "insert": "포크 삽입",
    "lift": "들기",
    "extract": "빼기",
    "aisle_exit": "통로 후진",
    "transport": "출하장으로",
    "lower": "내리기",
    "withdraw": "빠지기",
    "return_home": "복귀",
    "parked": "주차 완료",
}
ITEMS = [
    "observe",
    "approach",
    "insert",
    "lift",
    "extract",
    "aisle_exit",
    "transport",
    "lower",
    "withdraw",
    "return_home",
]
FONTS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/nanum/NanumSquareB.ttf",
]


def font(size):
    for path in FONTS:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


class Projector:
    """World point -> pixel of the recorded USD camera (looks along -Z, pinhole)."""

    def __init__(self, camera: dict):
        eye = np.asarray(camera["eye_m"], float)
        target = np.asarray(camera["target_m"], float)
        up = np.asarray(camera["up"], float)
        forward = (target - eye) / np.linalg.norm(target - eye)
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)
        self.up = np.cross(right, forward)
        self.eye, self.forward, self.right = eye, forward, right
        width, height = camera["resolution"]
        focal = camera.get("focal_length_read", camera["focal_length"])
        self.fx = width * focal / camera["horizontal_aperture"]
        self.fy = self.fx  # square pixels
        self.cx, self.cy = width / 2, height / 2

    def __call__(self, xy) -> np.ndarray:
        p = np.atleast_2d(np.asarray(xy, float))
        if p.shape[1] == 2:
            p = np.column_stack((p, np.zeros(len(p))))
        d = p - self.eye
        depth = d @ self.forward
        u = self.cx + self.fx * (d @ self.right) / depth
        v = self.cy - self.fy * (d @ self.up) / depth
        return np.column_stack((u, v)) * (VIEW_W / 1280)


def _corners(pose, footprint: dict) -> np.ndarray:
    """Footprint corners (rear-axle pose, front/rear/half-width) in the world."""
    x, y, yaw = pose[:3]
    front, rear, half = (
        footprint["front_m"],
        footprint["rear_m"],
        footprint["half_width_m"],
    )
    local = np.array([[front, half], [front, -half], [-rear, -half], [-rear, half]])
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack(
        (x + c * local[:, 0] - s * local[:, 1], y + s * local[:, 0] + c * local[:, 1])
    )


def frames(path: Path):
    proc = subprocess.Popen(
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
    size = 1280 * 720 * 3
    while True:
        raw = proc.stdout.read(size)
        if len(raw) < size:
            break
        yield np.frombuffer(raw, np.uint8).reshape(720, 1280, 3)
    proc.wait()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--title", default="지게차 5대 동시 운반 · NVIDIA 창고 전체 · 각자 LiDAR SLAM"
    )
    parser.add_argument(
        "--speed", type=int, default=2, help="playback speed-up (frames kept 1 in N)"
    )
    parser.add_argument("--trail-s", type=float, default=30.0)
    parser.add_argument("--max-s", type=float, help="stop at this simulation time")
    parser.add_argument(
        "--preview-png", type=Path, help="write the frame at --preview-s and stop"
    )
    parser.add_argument("--preview-s", type=float, default=100.0)
    args = parser.parse_args(argv)
    camera = json.loads((args.run / "camera.json").read_text())
    result = json.loads((args.run / "result.json").read_text())
    log = np.load(args.run / "fleet_log.npz")
    t_log, truth = log["t_s"], log["truth"]
    stage_item, loaded = log["stage"], log["loaded"]
    # A blocker is named whenever a grant falls short of the lookahead; the
    # truck waits only once the distance left in its grant is under 5 cm.
    blocked = np.where(log["allowed"] < 0.05, log["blocked"], -1)
    project = Projector(camera)
    missions = result["missions"]
    fps = camera["fps"]
    title_font, big, small = font(34), font(26), font(20)
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
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            str(args.output),
        ],
        stdin=subprocess.PIPE,
    )
    written = 0
    for index, frame in enumerate(frames(args.run / "overview.mp4")):
        t = index / fps
        if args.max_s is not None and t > args.max_s:
            break
        if args.preview_png is not None:
            if t < args.preview_s:
                continue
        elif index % args.speed:
            continue
        row = int(np.clip(np.searchsorted(t_log, t), 0, len(t_log) - 1))
        canvas = Image.new("RGB", (W, H), (18, 20, 24))
        view = Image.fromarray(frame).resize((VIEW_W, VIEW_H), Image.BILINEAR)
        canvas.paste(view, (0, 70))
        draw = ImageDraw.Draw(canvas, "RGBA")
        draw.text((24, 16), args.title, font=title_font, fill=(240, 240, 240))
        first = int(np.searchsorted(t_log, t - args.trail_s))
        for k in range(len(missions)):
            colour = COLOURS[k % len(COLOURS)]
            trail = project(truth[first : row + 1, k, :2]) + (0, 70)
            if len(trail) > 1:
                draw.line([tuple(p) for p in trail], fill=(*colour, 200), width=4)
            here = project(truth[row, k, :2])[0] + (0, 70)
            footprint = result["geometry"][
                "loaded_footprint" if loaded[row, k] else "unloaded_footprint"
            ]
            draw.polygon(
                [
                    tuple(p)
                    for p in project(_corners(truth[row, k], footprint)) + (0, 70)
                ],
                outline=(*colour, 255),
                width=3,
            )
            draw.text(
                (here[0] + 12, here[1] - 34),
                f"T{k}",
                font=big,
                fill=(255, 255, 255),
                stroke_width=3,
                stroke_fill=(0, 0, 0),
            )
            waiting_on = int(blocked[row, k])
            if waiting_on >= 0:
                other = project(truth[row, waiting_on, :2])[0] + (0, 70)
                draw.ellipse(
                    [here[0] - 20, here[1] - 20, here[0] + 20, here[1] + 20],
                    outline=(255, 70, 70),
                    width=3,
                )
                draw.line([tuple(here), tuple(other)], fill=(255, 70, 70, 160), width=2)
        x0 = VIEW_W + 20
        draw.text(
            (x0, 90), f"시뮬레이션 시각  {t:6.1f} s", font=big, fill=(240, 240, 240)
        )
        y = 140
        for k, mission in enumerate(missions):
            colour = COLOURS[k % len(COLOURS)]
            item = int(stage_item[row, k])
            stage = ITEMS[item] if 0 <= item < len(ITEMS) else "parked"
            truck = result["trucks"][k]
            if truck.get("failed") and t >= (truck.get("finished_s") or 1e9):
                status = "실패"
            elif stage == "parked":
                status = ""
            elif int(blocked[row, k]) >= 0:
                status = f"대기 → T{int(blocked[row, k])}"
            else:
                status = "적재 주행" if loaded[row, k] else "주행"
            draw.rectangle([x0, y, x0 + 14, y + 60], fill=colour)
            draw.text(
                (x0 + 26, y - 2),
                f"T{k} · {mission['aisle'].replace('aisle_', '통로 ')}",
                font=big,
                fill=(240, 240, 240),
            )
            draw.text(
                (x0 + 26, y + 32),
                " · ".join(x for x in (STAGE_KO.get(stage, stage), status) if x),
                font=small,
                fill=(200, 205, 215),
            )
            y += 92
        draw.text(
            (24, VIEW_H + 84),
            "선: 최근 30 s 실제 궤적 · 빨간 고리: 다른 트럭이 비킬 때까지 대기(교통 관리)",
            font=small,
            fill=(200, 205, 215),
        )
        draw.text(
            (24, VIEW_H + 116),
            "입고장(북) → 랙 통로 → 출하장(남) → 주차장 · 합성 시뮬레이션(Isaac Sim 5.1), 실측 차체 모델",
            font=small,
            fill=(150, 155, 165),
        )
        if args.preview_png is not None:
            canvas.save(args.preview_png)
            written = 1
            break
        encoder.stdin.write(np.asarray(canvas, np.uint8).tobytes())
        written += 1
    encoder.stdin.close()
    encoder.wait()
    print(json.dumps({"frames_written": written, "output": str(args.output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
