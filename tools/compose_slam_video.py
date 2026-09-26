"""Compose one SLAM recording into a three-panel video, synchronised by time.

    python tools/compose_slam_video.py --record <run> --replay <run>_replay \
        --output <run>_replay/slam_three_panel.mp4

Panels, all at the same simulation instant:
  1. the truck moving through the factory (Isaac overview, cropped to the hall;
     red dots are that instant's LiDAR hits),
  2. SLAM: the slam_toolbox map as it stood then, the true path (blue), the
     online SLAM estimate (red) and the current scan placed at that estimate
     (green) -- drawn over the same world square as panel 1,
  3. the truck's forward camera, RGB above and depth below.
Live errors are measured against ground truth with the start pose aligned,
which is the one alignment a robot knows. Inputs: the Isaac run of
sim/isaac/run_slam_drive.py --video --robot-camera and its slam_toolbox replay
with map_history:=true. Needs ffmpeg and Pillow.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

CANVAS = (1920, 1080)
SQUARE = 620
CAMERA = (620, 465)
FONTS = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/nanum/NanumSquareRoundB.ttf",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
)
TRUTH, ESTIMATE, SCAN = (70, 150, 255), (235, 45, 45), (60, 235, 90)
ODOMETRY = (255, 160, 30)
CHART = (30, 872, 1220, 128)  # left, top, width, height


def fit_floor_affine(world_xy: np.ndarray, pixels_uv: np.ndarray) -> np.ndarray:
    """(2, 3) least-squares map from floor x, y to overview pixels u, v."""
    design = np.column_stack((world_xy, np.ones(len(world_xy))))
    solution, *_ = np.linalg.lstsq(design, pixels_uv, rcond=None)
    return solution.T


def latest_index(stamps: np.ndarray, t: float) -> int:
    """Index of the last stamp at or before t, or -1."""
    return int(np.searchsorted(stamps, t + 1e-9, side="right")) - 1


def world_from_map(poses: np.ndarray, start: np.ndarray) -> np.ndarray:
    c, s = math.cos(start[2]), math.sin(start[2])
    return np.column_stack(
        (
            start[0] + c * poses[:, 0] - s * poses[:, 1],
            start[1] + s * poses[:, 0] + c * poses[:, 1],
            poses[:, 2] + start[2],
        )
    )


def map_from_world(points_xy: np.ndarray, start: np.ndarray) -> np.ndarray:
    c, s = math.cos(start[2]), math.sin(start[2])
    dx, dy = points_xy[:, 0] - start[0], points_xy[:, 1] - start[1]
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))


def sample_occupancy(
    grid: np.ndarray,
    *,
    origin_xy: tuple[float, float],
    resolution_m: float,
    xm: np.ndarray,
    ym: np.ndarray,
) -> np.ndarray:
    """Occupancy (-1 unknown, 0..100) at map-frame points; -2 off the grid."""
    col = np.floor((xm - origin_xy[0]) / resolution_m).astype(int)
    row = np.floor((ym - origin_xy[1]) / resolution_m).astype(int)
    inside = (row >= 0) & (row < grid.shape[0]) & (col >= 0) & (col < grid.shape[1])
    values = np.full(xm.shape, -2, dtype=np.int16)
    values[inside] = grid[row[inside], col[inside]]
    return values


def occupancy_colours(values: np.ndarray) -> np.ndarray:
    """RGB for occupancy: unknown and off-grid slate, free light, occupied dark."""
    values = np.asarray(values)
    if np.any(values > 100) or np.any(values < -2):
        raise ValueError("occupancy must be -2 (off grid), -1 or 0..100")
    unknown = np.array([62, 66, 74], float)
    free, occupied = np.array([236, 236, 230], float), np.array([12, 12, 18], float)
    share = np.clip(values, 0, 100)[..., None] / 100.0
    colours = free * (1 - share) + occupied * share
    colours[values < 0] = unknown
    return colours.round().astype(np.uint8)


def chart_points(
    times: np.ndarray, errors: np.ndarray, *, end_t: float, chart_max: float
) -> list[tuple[float, float]]:
    """Pixel polyline of an error history inside the CHART box."""
    left, top, width, height = CHART
    u = left + times / end_t * width
    v = top + height - np.clip(errors / chart_max, 0, 1) * height
    return list(zip(u.tolist(), v.tolist(), strict=True))


PHASE_NAMES = {
    "observe": "팔레트 찾기 (관측 지점으로 이동·카메라 인식)",
    "approach": "인식한 팔레트로 접근",
    "insert": "포크 삽입",
    "lift": "팔레트 들어 올림",
    "extract": "팔레트 빼내기",
    "transport": "목적지로 운반",
    "lower": "목적지에 내려놓기",
    "withdraw": "포크 빼기",
    "settle": "정지",
    "return_home": "출발 지점으로 복귀",
    "home_settle": "정지",
    "complete": "임무 완료",
}


def phase_at(transitions: list[dict], t: float) -> str | None:
    """Mission stage at time t from run_transport's recorded transitions."""
    if not transitions:
        return None
    phase = transitions[0]["from"]
    for item in transitions:
        if item["time_s"] <= t + 1e-9:
            phase = item["to"]
    return phase


def _font(size: int) -> ImageFont.FreeTypeFont:
    for path in FONTS:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def _csv(path: Path) -> np.ndarray:
    with path.open(newline="") as stream:
        rows = list(csv.reader(stream))[1:]
    return np.asarray(rows, dtype=float).reshape(-1, 4)


def _reader(path: Path, width: int, height: int):
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
        # Stop the decoder before closing its pipe, so a shortened render
        # (--max-frames) ends quietly.
        process.kill()
        process.wait()
        process.stdout.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, default=None)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"{args.output} exists; choose a new file")

    meta = json.loads((args.record / "meta.json").read_text())
    video = json.loads((args.record / "video_frames.json").read_text())
    offset = json.loads((args.replay / "replay_manifest.json").read_text())[
        "clock_offset_s"
    ]
    with np.load(args.record / "slam_log.npz") as data:
        joint_t = data["joint_stamps_s"]
        base = data["base_pose_world"].astype(float)
        scan_t = data["scan_stamps_s"]
        scan_ranges = data["scan_ranges_m"]
    w, x, y, z = base[:, 3], base[:, 4], base[:, 5], base[:, 6]
    truth = np.column_stack(
        (
            base[:, 0],
            base[:, 1],
            np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
        )
    )
    start = truth[0]
    travelled = np.concatenate(
        ([0.0], np.cumsum(np.hypot(*np.diff(truth[:, :2], axis=0).T)))
    )
    slam = _csv(args.replay / "slam/slam_trajectory.csv")
    slam_t = slam[:, 0] - offset
    slam_world = world_from_map(slam[:, 1:], start)
    truth_at_slam = truth[
        np.clip(np.searchsorted(joint_t, slam_t - 1e-9), 0, len(joint_t) - 1)
    ]
    slam_error = np.hypot(*(slam_world[:, :2] - truth_at_slam[:, :2]).T)
    odometry = _csv(args.replay / "odometry.csv")
    odometry_error = np.hypot(
        *(world_from_map(odometry[:, 1:], start)[:, :2] - truth[:, :2]).T
    )
    with np.load(args.replay / "slam/map_history.npz") as history:
        map_t = history["stamps_ns"] * 1e-9 - offset
        origins = history["origins"]
        resolutions = history["resolutions_m"]
        grids = [history[f"map_{i:04d}"] for i in range(len(map_t))]

    laser = meta["laser"]
    beam_angles = (
        laser["angle_min_rad"]
        + np.arange(laser["beam_count"]) * laser["angle_increment_rad"]
    )
    mount_x, mount_y = laser["mount_xyz_m"][:2]

    # One world square, centred on the hall, shared by panels 1 and 2.
    hall = meta["hall"]
    side = max(hall["x_max_m"] - hall["x_min_m"], hall["y_max_m"] - hall["y_min_m"]) + 2
    cx = (hall["x_min_m"] + hall["x_max_m"]) / 2
    cy = (hall["y_min_m"] + hall["y_max_m"]) / 2
    x0, y0 = cx - side / 2, cy - side / 2
    affine = fit_floor_affine(
        np.asarray(video["overview_floor_points"]["world_m"])[:, :2],
        np.asarray(video["overview_floor_points"]["pixels_uv"]),
    )
    corners = affine @ np.array([[x0, x0 + side], [y0 + side, y0], [1, 1]])
    crop = (
        int(round(corners[0, 0])),
        int(round(corners[1, 0])),
        int(round(corners[0, 1])),
        int(round(corners[1, 1])),
    )
    pixel_centres = (np.arange(SQUARE) + 0.5) / SQUARE * side
    grid_x, grid_y = np.meshgrid(x0 + pixel_centres, y0 + side - pixel_centres)
    map_xy = map_from_world(np.column_stack((grid_x.ravel(), grid_y.ravel())), start)

    def to_panel(points_xy: np.ndarray) -> list[tuple[float, float]]:
        u = (points_xy[:, 0] - x0) / side * SQUARE
        v = (y0 + side - points_xy[:, 1]) / side * SQUARE
        return list(zip(u.tolist(), v.tolist(), strict=True))

    title, label, text, small = _font(30), _font(20), _font(24), _font(17)
    camera_width, camera_height = video["robot_camera"]["resolution"]
    readers = (
        _reader(args.record / video.get("overview_video", "survey.mp4"), 1280, 720),
        _reader(args.record / "camera_rgb.mp4", camera_width, camera_height),
        _reader(args.record / "camera_depth.mp4", camera_width, camera_height),
    )
    encoder = subprocess.Popen(
        [
            "ffmpeg",
            "-nostdin",
            "-n",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            f"{CANVAS[0]}x{CANVAS[1]}",
            "-r",
            str(video["fps"]),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(args.output),
        ],
        stdin=subprocess.PIPE,
    )
    cached_map, cached_index = None, None
    end_t = float(joint_t[-1])
    chart_max = max(float(odometry_error.max()), float(slam_error.max()), 0.05) * 1.1
    mission = meta.get("mission")
    result = {}
    if mission is not None and (args.record / "result.json").exists():
        result = json.loads((args.record / "result.json").read_text())
    transitions = result.get("transitions", [])
    samples = result.get("samples", [])
    sample_t = np.array([item["time_s"] for item in samples])
    detected_at = next(
        (i["time_s"] for i in transitions if i["from"] == "observe"), None
    )
    reached = {i["to"]: i["time_s"] for i in transitions}
    top_speed = np.maximum.accumulate(
        [abs(item.get("signed_speed_mps", 0.0)) for item in samples] or [0.0]
    )
    frames = video["frames"][: args.max_frames]
    for record, overview, rgb, depth in zip(frames, *readers, strict=False):
        t = record["time_s"]
        canvas = Image.new("RGB", CANVAS, (18, 20, 26))
        draw = ImageDraw.Draw(canvas)
        heading_text = (
            "Isaac Sim 공장 홀 · 팔레트 인식 → 운반 → 하역 → 복귀"
            if mission is not None
            else "Isaac Sim 공장 홀 · 2D LiDAR SLAM (slam_toolbox, 기록 재생)"
        )
        draw.text(
            (20, 14),
            f"{heading_text} · seed {meta['seed']} · t = {t:6.1f} s",
            font=title,
            fill=(235, 235, 235),
        )

        # Panel 1: the truck in the factory.
        robot = Image.fromarray(overview).crop(crop).resize((SQUARE, SQUARE))
        canvas.paste(robot, (20, 70))

        # Panel 2: the map as slam_toolbox had it at t.
        index = latest_index(map_t, t)
        if index != cached_index:
            if index < 0:
                values = np.full(map_xy.shape[0], -2)
            else:
                values = sample_occupancy(
                    grids[index],
                    origin_xy=tuple(origins[index, :2]),
                    resolution_m=float(resolutions[index]),
                    xm=map_xy[:, 0],
                    ym=map_xy[:, 1],
                )
            cached_map = Image.fromarray(
                occupancy_colours(values).reshape(SQUARE, SQUARE, 3)
            )
            cached_index = index
        slam_panel = cached_map.copy()
        pen = ImageDraw.Draw(slam_panel)
        k = latest_index(joint_t, t)
        if k > 0:
            pen.line(to_panel(truth[: k + 1 : 12, :2]), fill=TRUTH, width=5)
        s = latest_index(slam_t, t)
        if s >= 0:
            if s > 0:
                pen.line(to_panel(slam_world[: s + 1, :2]), fill=ESTIMATE, width=2)
            j = latest_index(scan_t, slam_t[s])
            pose = slam_world[s]
            ranges = scan_ranges[j].astype(float)
            hit = np.isfinite(ranges)
            lx = pose[0] + math.cos(pose[2]) * mount_x - math.sin(pose[2]) * mount_y
            ly = pose[1] + math.sin(pose[2]) * mount_x + math.cos(pose[2]) * mount_y
            points = np.column_stack(
                (
                    lx + ranges[hit] * np.cos(pose[2] + beam_angles[hit]),
                    ly + ranges[hit] * np.sin(pose[2] + beam_angles[hit]),
                )
            )
            for u, v in to_panel(points):
                pen.rectangle((u - 1, v - 1, u + 1, v + 1), fill=SCAN)
            (u, v), heading = to_panel(pose[None, :2])[0], pose[2]
            tip = (u + 22 * math.cos(heading), v - 22 * math.sin(heading))
            left = (u + 12 * math.cos(heading + 2.5), v - 12 * math.sin(heading + 2.5))
            right = (u + 12 * math.cos(heading - 2.5), v - 12 * math.sin(heading - 2.5))
            pen.polygon([tip, left, right], fill=ESTIMATE, outline=(255, 255, 255))
        else:
            pen.text((20, 290), "SLAM 초기화 중", font=text, fill=(255, 255, 255))
        if k >= 0:
            tu, tv = to_panel(truth[k : k + 1, :2])[0]
            pen.ellipse((tu - 5, tv - 5, tu + 5, tv + 5), outline=TRUTH, width=2)
        if mission is not None:
            goal = mission["destination"]
            gu, gv = to_panel(np.array([[goal["x_m"], goal["y_m"]]]))[0]
            radius = 0.63 / side * SQUARE
            pen.ellipse(
                (gu - radius, gv - radius, gu + radius, gv + radius),
                outline=(0, 200, 60),
                width=3,
            )
            pen.text(
                (gu + radius + 4, gv - 10), "목적지", font=small, fill=(0, 150, 40)
            )
        canvas.paste(slam_panel, (650, 70))

        # Panel 3: the truck's own camera.
        canvas.paste(Image.fromarray(rgb).resize(CAMERA), (1280, 70))
        canvas.paste(Image.fromarray(depth).resize(CAMERA), (1280, 70 + CAMERA[1] + 10))

        for x, y_, name in (
            (20, 70, "① 로봇 움직임 · Isaac 조감 (빨간 점 = LiDAR 적중)"),
            (650, 70, "② SLAM · slam_toolbox 가 만든 지도와 추정 위치"),
            (1280, 70, "③ 로봇 카메라 RGB (전방, 합성)"),
            (1280, 80 + CAMERA[1], "④ 로봇 카메라 깊이 (가까움 빨강 → 멀리 파랑)"),
        ):
            box = draw.textbbox((x + 8, y_ + 6), name, font=label)
            draw.rectangle(
                (box[0] - 6, box[1] - 4, box[2] + 6, box[3] + 4), fill=(0, 0, 0)
            )
            draw.text((x + 8, y_ + 6), name, font=label, fill=(255, 255, 255))

        # Live figures under panels 1 and 2.
        progress = 100 * travelled[max(k, 0)] / travelled[-1]
        slam_now = f"{slam_error[s] * 100:5.1f} cm" if s >= 0 else "  —"
        lines_left = [
            f"시뮬레이션 시각  {t:6.1f} / {end_t:5.1f} s",
            f"조사 경로 진행   {progress:5.1f} %  ({travelled[max(k, 0)]:5.1f} m)",
            f"처리한 스캔      {max(latest_index(scan_t, t) + 1, 0):4d} / {len(scan_t)}",
        ]
        lines_right = [
            f"SLAM 위치 오차   {slam_now}",
            f"바퀴 오도메트리 오차 {odometry_error[max(k, 0)] * 100:5.1f} cm",
            f"지도 갱신 {max(index + 1, 0):3d} 회",
        ]
        if mission is not None:
            now = samples[max(latest_index(sample_t, t), 0)] if samples else {}
            speed = abs(now.get("signed_speed_mps", 0.0))
            phase = phase_at(transitions, t)
            lines_left = [
                f"단계  {PHASE_NAMES.get(phase, phase)}",
                f"속도  {speed * 3.6:4.1f} km/h ({speed:4.2f} m/s)  ·  "
                f"최고 {top_speed[max(latest_index(sample_t, t), 0)] * 3.6:3.1f} km/h",
                f"포크 높이  {now.get('lift_m', 0.0) * 100:4.1f} cm"
                f"   ·   이동 {travelled[max(k, 0)]:5.1f} m",
            ]
            if detected_at is None or t < detected_at:
                seen = "팔레트 탐색 중"
            else:
                error = result.get("perception", {}).get("perception_error", {})
                seen = f"팔레트 인식됨 · 위치 오차 {error.get('position_m', 0) * 1000:.1f} mm"
            done = []
            if (
                "lift" in reached
                and t >= reached["lift"]
                and "insertion_error" in result
            ):
                done.append(
                    f"삽입 {result['insertion_error']['position_m'] * 1000:.1f}"
                )
            if "withdraw" in reached and t >= reached["withdraw"]:
                if result.get("delivery_error_m") is not None:
                    done.append(f"하역 {result['delivery_error_m'] * 1000:.1f}")
            if "complete" in reached and t >= reached["complete"]:
                home = result.get("return_home_error")
                if home:
                    done.append(f"복귀 {home['position_m'] * 1000:.1f}")
            lines_right = [
                seen,
                f"SLAM {slam_now.strip()}  ·  오도메트리 "
                f"{odometry_error[max(k, 0)] * 100:.1f} cm",
                ("오차(mm)  " + " · ".join(done)) if done else "오차(mm)  —",
            ]
        for row, line in enumerate(lines_left):
            draw.text((30, 704 + 36 * row), line, font=text, fill=(230, 230, 230))
        for row, line in enumerate(lines_right):
            draw.text((660, 704 + 36 * row), line, font=text, fill=(230, 230, 230))
        draw.text(
            (660, 814),
            "파랑 = 실제 경로·위치   빨강 = SLAM 추정   초록 = 현재 스캔",
            font=small,
            fill=(200, 200, 200),
        )
        left, top, width, height = CHART
        draw.rectangle((left, top, left + width, top + height), outline=(90, 90, 100))
        draw.text(
            (left, top - 24),
            f"정답 대비 위치 오차 (세로 0–{chart_max * 100:.0f} cm, 가로 0–{end_t:.0f} s)"
            "   빨강 = SLAM   주황 = 바퀴 오도메트리",
            font=small,
            fill=(200, 200, 200),
        )

        if k > 12:
            draw.line(
                chart_points(
                    joint_t[: k + 1 : 12],
                    odometry_error[: k + 1 : 12],
                    end_t=end_t,
                    chart_max=chart_max,
                ),
                fill=ODOMETRY,
                width=2,
            )
        if s > 0:
            draw.line(
                chart_points(
                    slam_t[: s + 1],
                    slam_error[: s + 1],
                    end_t=end_t,
                    chart_max=chart_max,
                ),
                fill=ESTIMATE,
                width=2,
            )
        now_u = left + t / end_t * width
        draw.line((now_u, top, now_u, top + height), fill=(120, 120, 130))
        draw.text(
            (30, 1020),
            "합성 데이터 · 지게차 제어는 시뮬레이터 정답 자세로 주행 · SLAM 은 같은 "
            "기록을 slam_toolbox 로 재생한 결과 · 오차는 출발 자세만 맞춘 값",
            font=small,
            fill=(160, 160, 170),
        )
        encoder.stdin.write(np.asarray(canvas, np.uint8).tobytes())
    for reader in readers:
        reader.close()
    encoder.stdin.close()
    if encoder.wait() != 0:
        raise RuntimeError("video encoding failed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
