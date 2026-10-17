"""Compose an online-SLAM transport run into a three-panel video (plan S3).

    python tools/compose_online_slam_video.py --run <run> --bridge <bridge> \
        --output <run>/slam_online_three_panel.mp4

The run is sim/isaac/run_transport.py --slam-feedback --record-slam --video
--robot-camera; the bridge directory is isaac_slam_bridge's output_dir (maps.json,
maps.npz). Unlike tools/compose_slam_video.py (an offline replay, start-aligned),
everything here is what the closed loop itself had:

  1. the truck in the factory (Isaac overview, cropped to the hall),
  2. the slam_toolbox map **the run had received by that frame** -- a frame
     records the last scan id exchanged before it (slam_last_scan_id), and only
     maps the bridge stored after an earlier scan id are eligible (a map stored
     after the same scan may have arrived after the reply) -- with the true path
     (blue), the estimate control used (red: applied map<-odom composed with
     odom<-base at each scan) and that scan placed at the estimate (green),
  3. the truck's forward camera, RGB above and depth below.

odom = map = world at the known start, so no alignment is applied to anything:
errors are raw, as in the plan. Writes <output>.frames.json with the map index
each frame used, so the map selection can be checked frame by frame.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
import compose_slam_video as base  # noqa: E402

PHASE_NAMES = getattr(base, "PHASE_NAMES", {})


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def eligible_map(after_scan_ids: np.ndarray, last_scan_id: int) -> int:
    """Index of the newest map stored after a scan id below last_scan_id, or -1."""
    ok = np.flatnonzero(after_scan_ids < last_scan_id)
    return int(ok[-1]) if ok.size else -1


def scan_estimates(records: list[dict]) -> dict:
    """Per answered scan: stamp, estimate (control), odometry and truth base poses."""
    rows = [
        r
        for r in records
        if r.get("status") in ("processed", "skipped") and r.get("applied_map_from_odom")
    ]
    stamps = np.array([r["stamp_s"] for r in rows])
    odom = np.array([r["odom_base"] for r in rows])
    truth = np.array([r["truth_base"] for r in rows])
    estimate = np.array(
        [compose(r["applied_map_from_odom"], r["odom_base"]) for r in rows]
    )
    return {
        "stamps": stamps,
        "scan_ids": np.array([r["scan_id"] for r in rows]),
        "estimate": estimate,
        "odom": odom,
        "truth": truth,
        "processed": np.array([r["status"] == "processed" for r in rows]),
    }


PLAN = (255, 214, 0)  # the path the truck is following now
PLAN_OLD = (150, 150, 160)  # the path it was given before a replan
GRID = (255, 130, 30)  # the LiDAR obstacle grid's OCCUPIED cells
PREVIOUS_S = 4.0  # how long the replaced path stays drawn
BANNER_S = 3.0  # how long a replan / new-obstacle banner stays up


def overlay_at(t: float, plans: list[dict], spawns: list[dict]) -> dict:
    """What the overlay draws at t: the plan given by then, the one it replaced
    (for PREVIOUS_S after a replan), and a banner for a fresh replan or a new
    obstacle (display only; plans come from run_transport's plan_history)."""
    given = [p for p in plans if p["time_s"] <= t]
    current = given[-1] if given else None
    previous = None
    banner = None
    if len(given) >= 2 and t - current["time_s"] <= PREVIOUS_S and current["why"] in ("replan", "backoff"):
        previous = given[-2]
    if current is not None and t - current["time_s"] <= BANNER_S:
        if current["why"] == "replan":
            banner = "장애물 감지 → 경로 재계획"
        elif current["why"] == "backoff":
            banner = "경로 막힘 → 짧게 후진 후 재계획"
    fresh = [s for s in spawns if s.get("action") == "spawn" and 0.0 <= t - s["time_s"] <= BANNER_S]
    if fresh and banner is None:
        banner = "새 장애물 출현 → LiDAR 로 감지 중"
    return {"current": current, "previous": previous, "banner": banner}


def boxes_at(t: float, log: list[dict]) -> list[np.ndarray]:
    """Corner arrays (4, 2) of the new obstacles standing at t (spawned, not removed)."""
    alive = {}
    for item in log:
        if item["time_s"] > t:
            break
        if item.get("action") == "spawn":
            _, x, y, yaw, size = item["detail"][:5]
            alive[item["detail"][0]] = (float(x), float(y), float(yaw), float(size[0]), float(size[1]))
        elif item.get("action") == "remove":
            alive.pop(item["detail"][0], None)
    out = []
    for x, y, yaw, length, width in alive.values():
        c, s = math.cos(yaw), math.sin(yaw)
        local = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]]) * [length / 2, width / 2]
        out.append(np.column_stack((x + local[:, 0] * c - local[:, 1] * s, y + local[:, 0] * s + local[:, 1] * c)))
    return out


def grid_index(times: np.ndarray, t: float) -> int:
    """Index of the latest recorded obstacle grid at or before t, or -1."""
    ok = np.flatnonzero(times <= t)
    return int(ok[-1]) if ok.size else -1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument(
        "--draw-odometry",
        action="store_true",
        help="also draw the wheel-odometry-only path (odom<-base, orange) on the map panel",
    )
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"{args.output} exists; choose a new file")

    meta = json.loads((args.run / "meta.json").read_text())
    video = json.loads((args.run / "video_frames.json").read_text())
    result = json.loads((args.run / "result.json").read_text())
    scans = scan_estimates(json.loads((args.run / "slam_records.json").read_text()))
    with np.load(args.run / "slam_log.npz") as data:
        joint_t = data["joint_stamps_s"]
        base_pose = data["base_pose_world"].astype(float)
        scan_t = data["scan_stamps_s"]
        scan_ranges = data["scan_ranges_m"]
    w, x, y, z = base_pose[:, 3], base_pose[:, 4], base_pose[:, 5], base_pose[:, 6]
    truth = np.column_stack(
        (
            base_pose[:, 0],
            base_pose[:, 1],
            np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
        )
    )
    travelled = np.concatenate(
        ([0.0], np.cumsum(np.hypot(*np.diff(truth[:, :2], axis=0).T)))
    )
    slam_error = np.hypot(*(scans["estimate"][:, :2] - scans["truth"][:, :2]).T)
    odom_error = np.hypot(*(scans["odom"][:, :2] - scans["truth"][:, :2]).T)

    plan_file = args.run / "plan_history.json"
    plan_data = json.loads(plan_file.read_text()) if plan_file.exists() else {"plans": [], "new_obstacles": []}
    plans = plan_data.get("plans", [])
    spawns = plan_data.get("new_obstacles", [])
    grid_file = args.run / "obstacle_grid_frames.npz"
    grid_frames = dict(np.load(grid_file)) if grid_file.exists() else None
    maps = json.loads((args.bridge / "maps.json").read_text())
    after_ids = np.array([m["after_scan_id"] for m in maps])
    grids = np.load(args.bridge / "maps.npz")

    laser = meta["laser"]
    beam_angles = (
        laser["angle_min_rad"] + np.arange(laser["beam_count"]) * laser["angle_increment_rad"]
    )
    mount_x, mount_y = laser["mount_xyz_m"][:2]
    hall = meta["hall"]
    side = max(hall["x_max_m"] - hall["x_min_m"], hall["y_max_m"] - hall["y_min_m"]) + 2
    cx = (hall["x_min_m"] + hall["x_max_m"]) / 2
    cy = (hall["y_min_m"] + hall["y_max_m"]) / 2
    x0, y0 = cx - side / 2, cy - side / 2
    affine = base.fit_floor_affine(
        np.asarray(video["overview_floor_points"]["world_m"])[:, :2],
        np.asarray(video["overview_floor_points"]["pixels_uv"]),
    )
    corners = affine @ np.array([[x0, x0 + side], [y0 + side, y0], [1, 1]])
    crop = tuple(int(round(v)) for v in (corners[0, 0], corners[1, 0], corners[0, 1], corners[1, 1]))
    SQUARE = base.SQUARE

    def to_overview(points_xy: np.ndarray) -> list[tuple[float, float]]:
        uv = (affine @ np.column_stack((points_xy, np.ones(len(points_xy)))).T).T
        sx = SQUARE / (crop[2] - crop[0])
        sy = SQUARE / (crop[3] - crop[1])
        return list(zip(((uv[:, 0] - crop[0]) * sx).tolist(), ((uv[:, 1] - crop[1]) * sy).tolist(), strict=True))
    centres = (np.arange(SQUARE) + 0.5) / SQUARE * side
    grid_x, grid_y = np.meshgrid(x0 + centres, y0 + side - centres)
    world_xy = np.column_stack((grid_x.ravel(), grid_y.ravel()))

    def to_panel(points_xy: np.ndarray) -> list[tuple[float, float]]:
        u = (points_xy[:, 0] - x0) / side * SQUARE
        v = (y0 + side - points_xy[:, 1]) / side * SQUARE
        return list(zip(u.tolist(), v.tolist(), strict=True))

    title, label, text, small = base._font(30), base._font(20), base._font(24), base._font(17)
    camera_width, camera_height = video["robot_camera"]["resolution"]
    readers = (
        base._reader(args.run / video.get("overview_video", "transport.mp4"), 1280, 720),
        base._reader(args.run / "camera_rgb.mp4", camera_width, camera_height),
        base._reader(args.run / "camera_depth.mp4", camera_width, camera_height),
    )
    encoder = subprocess.Popen(
        [
            "ffmpeg", "-nostdin", "-n", "-loglevel", "error", "-f", "rawvideo",
            "-pix_fmt", "rgb24", "-s", f"{base.CANVAS[0]}x{base.CANVAS[1]}",
            "-r", str(video["fps"]), "-i", "-", "-an", "-c:v", "libx264", "-crf", "20",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.output),
        ],
        stdin=subprocess.PIPE,
    )
    transitions = result.get("transitions", [])
    samples = result.get("samples", [])
    sample_t = np.array([item["time_s"] for item in samples])
    end_t = float(joint_t[-1])
    chart_max = max(float(odom_error.max()), float(slam_error.max()), 0.05) * 1.1
    frames = video["frames"][: args.max_frames]
    used = []
    cached_map, cached_index = None, None
    written = 0
    for record, overview, rgb, depth in zip(frames, *readers, strict=False):
        t = record["time_s"]
        last_id = int(record.get("slam_last_scan_id", -1))
        canvas = Image.new("RGB", base.CANVAS, (18, 20, 26))
        draw = ImageDraw.Draw(canvas)
        draw.text(
            (20, 14),
            "Isaac Sim 공장 홀 · 온라인 SLAM 폐루프 (slam_toolbox 추정으로 주행)"
            f" · seed {meta['seed']} · t = {t:6.1f} s",
            font=title,
            fill=(235, 235, 235),
        )
        over = overlay_at(t, plans, spawns)
        view = Image.fromarray(overview).crop(crop).resize((SQUARE, SQUARE))
        if over["current"] is not None:
            vpen = ImageDraw.Draw(view)
            if over["previous"] is not None:
                vpen.line(to_overview(np.asarray(over["previous"]["poses"], float)), fill=PLAN_OLD, width=4)
            vpen.line(to_overview(np.asarray(over["current"]["poses"], float)), fill=PLAN, width=4)
        for corners in boxes_at(t, spawns):
            vpen = ImageDraw.Draw(view)
            pts = to_overview(corners)
            vpen.polygon(pts, outline=(255, 40, 40), width=4)
            u_ = max(p[0] for p in pts) + 6
            if u_ + 92 > SQUARE:  # near the right edge: put the tag on the left
                u_ = min(p[0] for p in pts) - 96
            v_ = max(min(p[1] for p in pts) - 4, 4)
            vpen.rectangle((u_ - 3, v_ - 3, u_ + 86, v_ + 24), fill=(200, 40, 40))
            vpen.text((u_, v_), "새 장애물", font=label, fill=(255, 255, 255))
        canvas.paste(view, (20, 70))

        index = eligible_map(after_ids, last_id)
        used.append({"time_s": t, "slam_last_scan_id": last_id, "map_index": index})
        if index != cached_index:
            if index < 0:
                values = np.full(world_xy.shape[0], -2)
            else:
                info = maps[index]
                values = base.sample_occupancy(
                    grids[f"map_{index:04d}"],
                    origin_xy=tuple(info["origin"][:2]),
                    resolution_m=float(info["resolution_m"]),
                    xm=world_xy[:, 0],
                    ym=world_xy[:, 1],
                )
            cached_map = Image.fromarray(base.occupancy_colours(values).reshape(SQUARE, SQUARE, 3))
            cached_index = index
        panel = cached_map.copy()
        pen = ImageDraw.Draw(panel)
        if grid_frames is not None and len(grid_frames["times"]):
            gi = grid_index(grid_frames["times"], t)
            if gi >= 0:
                cells = grid_frames["cells"][grid_frames["offsets"][gi] : grid_frames["offsets"][gi + 1]]
                res = float(grid_frames["resolution"][gi])
                ox, oy = grid_frames["origins"][gi]
                xy = np.column_stack((ox + (cells[:, 0] + 0.5) * res, oy + (cells[:, 1] + 0.5) * res))
                inside = (xy[:, 0] > x0) & (xy[:, 0] < x0 + side) & (xy[:, 1] > y0) & (xy[:, 1] < y0 + side)
                for u, v in to_panel(xy[inside]):
                    pen.rectangle((u - 1, v - 1, u + 1, v + 1), fill=GRID)
        if over["previous"] is not None:
            pen.line(to_panel(np.asarray(over["previous"]["poses"], float)), fill=PLAN_OLD, width=3)
        if over["current"] is not None:
            pen.line(to_panel(np.asarray(over["current"]["poses"], float)), fill=PLAN, width=3)
        for corners in boxes_at(t, spawns):
            pen.polygon(to_panel(corners), outline=(255, 40, 40), width=3)
        k = base.latest_index(joint_t, t)
        if k > 0:
            pen.line(to_panel(truth[: k + 1 : 12, :2]), fill=base.TRUTH, width=5)
        s = base.latest_index(scans["stamps"], t)
        if s >= 0:
            if s > 0:
                if args.draw_odometry:
                    pen.line(to_panel(scans["odom"][: s + 1, :2]), fill=base.ODOMETRY, width=3)
                pen.line(to_panel(scans["estimate"][: s + 1, :2]), fill=base.ESTIMATE, width=2)
            pose = scans["estimate"][s]
            j = base.latest_index(scan_t, scans["stamps"][s])
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
                pen.rectangle((u - 1, v - 1, u + 1, v + 1), fill=base.SCAN)
            (u, v), heading = to_panel(pose[None, :2])[0], pose[2]
            tip = (u + 22 * math.cos(heading), v - 22 * math.sin(heading))
            left = (u + 12 * math.cos(heading + 2.5), v - 12 * math.sin(heading + 2.5))
            right = (u + 12 * math.cos(heading - 2.5), v - 12 * math.sin(heading - 2.5))
            pen.polygon([tip, left, right], fill=base.ESTIMATE, outline=(255, 255, 255))
        if index < 0:
            pen.text((20, 290), "첫 지도 수신 전", font=text, fill=(255, 255, 255))
        goal = meta["mission"]["destination"]
        gu, gv = to_panel(np.array([[goal["x_m"], goal["y_m"]]]))[0]
        radius = 0.63 / side * SQUARE
        pen.ellipse((gu - radius, gv - radius, gu + radius, gv + radius), outline=(0, 200, 60), width=3)
        canvas.paste(panel, (650, 70))
        canvas.paste(Image.fromarray(rgb).resize(base.CAMERA), (1280, 70))
        canvas.paste(Image.fromarray(depth).resize(base.CAMERA), (1280, 80 + base.CAMERA[1]))

        for px, py, name in (
            (20, 70, "① 로봇 움직임 · Isaac 조감"),
            (650, 70, "② 온라인 SLAM · 이 시각까지 받은 지도와 제어에 쓴 추정"),
            (1280, 70, "③ 로봇 카메라 RGB (전방, 합성)"),
            (1280, 80 + base.CAMERA[1], "④ 로봇 카메라 깊이"),
        ):
            box = draw.textbbox((px + 8, py + 6), name, font=label)
            draw.rectangle((box[0] - 6, box[1] - 4, box[2] + 6, box[3] + 4), fill=(0, 0, 0))
            draw.text((px + 8, py + 6), name, font=label, fill=(255, 255, 255))

        if over["banner"] is not None:
            box = draw.textbbox((0, 0), over["banner"], font=title)
            bw = box[2] - box[0]
            bx = 20 + (SQUARE * 2 + 10 - bw) // 2
            draw.rectangle((bx - 16, 620, bx + bw + 16, 670), fill=(200, 40, 40))
            draw.text((bx, 626), over["banner"], font=title, fill=(255, 255, 255))
        now = samples[max(base.latest_index(sample_t, t), 0)] if samples else {}
        phase = base.phase_at(transitions, t)
        speed = abs(now.get("signed_speed_mps", 0.0))
        processed = int(scans["processed"][: s + 1].sum()) if s >= 0 else 0
        lines_left = [
            f"단계  {PHASE_NAMES.get(phase, phase)}",
            f"속도  {speed:4.2f} m/s   ·   포크 높이 {now.get('lift_m', 0.0) * 100:4.1f} cm",
            f"이동  {travelled[max(k, 0)]:5.1f} m   ·   모드 {record.get('slam_mode', '—')}",
        ]
        slam_now = f"{slam_error[s] * 100:5.1f} cm" if s >= 0 else "  —"
        odom_now = f"{odom_error[s] * 100:5.1f} cm" if s >= 0 else "  —"
        lines_right = [
            f"SLAM 추정 오차(제어 입력) {slam_now}",
            f"바퀴 오도메트리만 썼다면  {odom_now}",
            f"키프레임 {processed:3d} 개 처리 · 지도 {max(index + 1, 0):3d} 번째",
        ]
        for row, line in enumerate(lines_left):
            draw.text((30, 704 + 36 * row), line, font=text, fill=(230, 230, 230))
        for row, line in enumerate(lines_right):
            draw.text((660, 704 + 36 * row), line, font=text, fill=(230, 230, 230))
        draw.text(
            (660, 814),
            "파랑 실제 · 빨강 SLAM 추정 · 초록 스캔 · 노랑 계획 경로 · 회색 직전 경로 · 주황 LiDAR 장애물",
            font=small,
            fill=(200, 200, 200),
        )
        cl, ct, cw, ch = base.CHART
        draw.rectangle((cl, ct, cl + cw, ct + ch), outline=(90, 90, 100))
        draw.text(
            (cl, ct - 24),
            f"정답 대비 위치 오차, 정렬 없음 (세로 0–{chart_max * 100:.0f} cm, 가로 0–{end_t:.0f} s)"
            "   빨강 = SLAM   주황 = 바퀴 오도메트리",
            font=small,
            fill=(200, 200, 200),
        )
        if s > 0:
            for series, colour in ((odom_error, base.ODOMETRY), (slam_error, base.ESTIMATE)):
                draw.line(
                    base.chart_points(scans["stamps"][: s + 1], series[: s + 1], end_t=end_t, chart_max=chart_max),
                    fill=colour,
                    width=2,
                )
        now_u = cl + t / end_t * cw
        draw.line((now_u, ct, now_u, ct + ch), fill=(120, 120, 130))
        draw.text(
            (30, 1020),
            "합성 데이터 · 지게차는 온라인 slam_toolbox 추정 자세로 주행(정답은 안전 검사·평가에만) · 합성 잡음 가정값",
            font=small,
            fill=(160, 160, 170),
        )
        encoder.stdin.write(np.asarray(canvas, np.uint8).tobytes())
        written += 1
    for reader in readers:
        reader.close()
    encoder.stdin.close()
    if encoder.wait() != 0:
        raise RuntimeError("video encoding failed")
    Path(str(args.output) + ".frames.json").write_text(
        json.dumps({"frames_written": written, "frames_recorded": len(video["frames"]), "frames": used})
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
