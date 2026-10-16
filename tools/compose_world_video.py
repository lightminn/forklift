"""Compose a random-factory fleet video: Isaac's top view, outlines, people, status panel.

    python tools/compose_world_video.py --run <run dir> --output <out.mp4> [--title "..."]

Plan: docs/plans/2026-10-09-fleet-random-worlds.md (W6). Reads overview.mp4,
camera.json, world_log.npz, world.json and result.json written by
sim/isaac/run_world.py. Points are projected with the recorded pinhole camera.
1920 x 1080: the view scaled to 1440 x 810 on the left, a panel on the right
(time, jobs done, each truck's stage and whether it waits for traffic or for a
person, the latest events), legend below. Pallet places and holding places
are outlined from world.json. Runs with TA/RA (plan 2026-10-10, log "alert")
show a TA as an amber ring, an RA as a red ring with the higher-ranked route
it treats as a wall, and a CC as a green ring. Runs with time-window tokens
(plan 2026-10-11, windows.json) show each crossing whose token is out as a
diamond in the colour of the truck that goes first (hollow: issued, filled:
its grant is in) and a magenta ring for a while on a truck whose token expired;
the rank badges are left out (no ranks). --planned-route draws each truck's
route still ahead, to the end of its job, as a dashed line in its colour
(routes.npz, moved back from the claim rects to the point the trail follows).
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

W, H = 1920, 1080
VIEW_W, VIEW_H = 1440, 810
TOP = 70
COLOURS = [
    (230, 85, 60), (60, 140, 230), (245, 180, 40), (90, 190, 110), (170, 100, 220),
    (40, 200, 200), (240, 120, 180), (160, 160, 60), (120, 200, 250), (250, 150, 80),
    (150, 110, 70), (200, 200, 200),
]
LOG_STAGES = ("idle", "leave", "to_pickup", "insert", "extract", "to_drop", "withdraw",
              "to_holding", "lift", "lower", "broken")
STAGE_KO = {
    "idle": "대기 칸에서 쉼", "leave": "출발(후진)", "to_pickup": "팔레트로", "insert": "포크 삽입",
    "lift": "들기", "extract": "빼기", "to_drop": "도착지로", "lower": "내리기",
    "withdraw": "빠지기", "to_holding": "대기 칸으로", "broken": "멈춤(작업 포기)",
}
STAGE_KO_SHORT = {"leave": "출발", "to_pickup": "팔레트로", "insert": "삽입", "extract": "빼기",
                  "to_drop": "도착지로", "withdraw": "빠지기", "to_holding": "대기 칸으로"}


def why_ko(why: str) -> str:
    """run_world's failure strings in Korean (unknown ones as they are)."""
    if why == "no route round the spill":
        return "쏟아진 짐을 돌아갈 길 없음"
    for prefix, text in (("timeout in ", "시간 초과"), ("tracking ", "경로 이탈")):
        if why.startswith(prefix):
            stage = why.rsplit(" in ", 1)[-1]
            return f"{STAGE_KO_SHORT.get(stage, stage)} {text}"
    return {"lift_timeout": "들기 시간 초과"}.get(why, why)
STRUCTURE_KO = {"rack": "① 랙 창고", "shop": "② 공정 공장", "multi": "③ 복합동"}
PERSON_STOP_M = 0.3  # protective field allows less than this: shown as stopped for a person
TA_COLOUR, RA_COLOUR, CC_COLOUR = (255, 185, 40), (255, 60, 60), (80, 225, 120)
EXPIRED_COLOUR = (230, 70, 230)
WALL_M, CC_SHOW_S = 20.0, 4.0  # run_world's default --wall-m; how long a CC ring stays
WHY_KO = {"cycle": "서로 기다림", "long_wait": "오래 기다림", "in_the_way": "길 막음", "stall": "선 트럭"}
HOW_KO = {"detour": "우회", "retreat": "후진해 비킴", "failed": "길 없음", "skipped": "지금은 못 함", "stale": "그새 풀림"}
EVENT_ROWS = 6
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
    """World point -> canvas pixel through the recorded USD camera (looks along -Z)."""

    def __init__(self, camera: dict):
        eye = np.asarray(camera["eye_m"], float)
        target = np.asarray(camera["target_m"], float)
        up = np.asarray(camera["up"], float)
        forward = (target - eye) / np.linalg.norm(target - eye)
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)
        self.up = np.cross(right, forward)
        self.eye, self.forward, self.right = eye, forward, right
        self.width, self.height = camera["resolution"]
        focal = camera.get("focal_length_read", camera["focal_length"])
        self.f = self.width * focal / camera["horizontal_aperture"]
        self.scale = VIEW_W / self.width

    def __call__(self, xy) -> np.ndarray:
        p = np.atleast_2d(np.asarray(xy, float))
        if p.shape[1] == 2:
            p = np.column_stack((p, np.zeros(len(p))))
        d = p - self.eye
        depth = d @ self.forward
        u = self.width / 2 + self.f * (d @ self.right) / depth
        v = self.height / 2 - self.f * (d @ self.up) / depth
        return np.column_stack((u, v)) * self.scale + (0, TOP)


def corners(pose, fp) -> np.ndarray:
    x, y, yaw = pose[:3]
    front, rear, half = fp
    local = np.array([[front, half], [front, -half], [-rear, -half], [-rear, half]])
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack((x + c * local[:, 0] - s * local[:, 1], y + s * local[:, 0] + c * local[:, 1]))


def rect_corners(r) -> np.ndarray:
    """Corners of an oriented rect (cx, cy, half length, half width, cos, sin)."""
    cx, cy, hl, hw, c, s = r
    local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    return np.column_stack((cx + c * local[:, 0] - s * local[:, 1], cy + s * local[:, 0] + c * local[:, 1]))


class Tablets:
    """Each truck's claim -- its "tablet" -- from routes.npz and the log: the
    samples of the route traffic control holds for it, from where the truck
    is (progress) to how far it may drive (granted)."""

    def __init__(self, run: Path, log):
        rz = np.load(run / "routes.npz")
        self.start, self.rects, self.g = rz["start"], rz["rects"].astype(float), rz["g"].astype(float)
        self.route, self.progress = log["route"].astype(int), log["progress"]
        self.granted, self.rank = log["granted"], log["rank"]

    def _route(self, rid):
        s0 = self.start[rid]
        s1 = self.start[rid + 1] if rid + 1 < len(self.start) else len(self.g)
        return self.g[s0:s1], self.rects[s0:s1]

    def claim(self, row, k):
        """(claimed rects, centres of the route still ahead beyond the claim)."""
        rid = self.route[row, k]
        if rid < 0:
            return np.zeros((0, 6)), np.zeros((0, 2))
        g, rects = self._route(rid)
        prog, gr = self.progress[row, k], self.granted[row, k]
        held = rects[(g >= prog - 0.2) & (g <= gr + 1e-6)]
        ahead = rects[(g > gr + 1e-6) & (g <= gr + 25.0)][::5, :2]
        return held, ahead

    def remaining(self, row, k) -> np.ndarray:
        """Rects of the route still ahead of truck k (from its progress on)."""
        rid = self.route[row, k]
        if rid < 0:
            return np.zeros((0, 6))
        g, rects = self._route(rid)
        return rects[g >= self.progress[row, k] - 0.2]

    def order(self, row) -> dict[int, int]:
        """Truck -> place in the order (1 goes first) among trucks with a job."""
        ranks = self.rank[row]
        busy = [k for k in range(len(ranks)) if ranks[k] < 1e9]
        return {k: i + 1 for i, k in enumerate(sorted(busy, key=lambda k: ranks[k]))}


def route_points(rects: np.ndarray, margin_m: float, footprints) -> np.ndarray:
    """Reference points along route rects. Each rect is a footprint (front, rear,
    half width) grown by the traffic margin, its centre (front - rear) / 2 ahead
    of the reference point -- the point truth (and so the trail) follows."""
    body = rects[:, 2] - margin_m
    half_lengths = np.array([(front + rear) / 2 for front, rear, _ in footprints])
    rear = np.array([rear for _, rear, _ in footprints])[np.argmin(np.abs(body[:, None] - half_lengths), axis=1)]
    return rects[:, :2] - (body - rear)[:, None] * rects[:, 4:6]


def dashed(draw: ImageDraw.ImageDraw, pts: np.ndarray, colour, width: int, on: int = 5, off: int = 4) -> None:
    """A line through pts drawn on for `on` steps, off for `off` (route samples are 0.2 m apart)."""
    for i in range(0, len(pts) - 1, on + off):
        seg = pts[i : i + on + 1]
        if len(seg) > 1:
            draw.line([tuple(p) for p in seg], fill=colour, width=width)


def paint_union(canvas: Image.Image, polys: list, colour, alpha: int) -> None:
    """Fill the union of polygons once (no darker overlaps) and outline it."""
    if not polys:
        return
    pts = np.vstack(polys)
    x0, y0 = np.floor(pts.min(axis=0)).astype(int) - 2
    x1, y1 = np.ceil(pts.max(axis=0)).astype(int) + 2
    x0, y0 = max(x0, 0), max(y0, TOP)
    x1, y1 = min(x1, VIEW_W), min(y1, TOP + VIEW_H)
    if x1 <= x0 or y1 <= y0:
        return
    mask = Image.new("L", (x1 - x0, y1 - y0), 0)
    md = ImageDraw.Draw(mask)
    for poly in polys:
        md.polygon([(float(px - x0), float(py - y0)) for px, py in poly], fill=255)
    solid = Image.new("RGB", mask.size, colour)
    canvas.paste(solid, (x0, y0), mask.point(lambda v: v * alpha // 255))
    canvas.paste(solid, (x0, y0), mask.filter(ImageFilter.FIND_EDGES))


def event_lines(events: list, jobs: list) -> list[tuple[float, str]]:
    """(time, Korean line) for the panel's event list, in time order."""
    out = []
    for e in events:
        t, kind, truck = e["t_s"], e.get("kind"), e.get("truck", "")
        name = f"T{truck[6:]}" if truck.startswith("truck_") else truck
        if kind == "pause":
            out.append((t, f"{name} 일시 고장 ({e['until'] - t:.0f} s)"))
        elif kind == "spill":
            out.append((t, "통로에 짐이 쏟아짐"))
        elif kind == "replan":
            why = e.get("why", "")
            why = "쏟아진 짐" if why == "spill" else "추종 이탈" if why.startswith("tracking") else why
            out.append((t, f"{name} 재계획({why}) {'성공' if e['status'] == 'success' else '실패'}"))
        elif kind in ("cycle", "stall"):
            other = e.get("blocker", "")
            out.append((t, f"{name} → T{other[6:]} {'서로 기다림' if kind == 'cycle' else '오래 기다림'}"))
        elif kind == "truck_failed":
            out.append((t, f"{name} 멈춤: {why_ko(e.get('why', ''))}"))
        elif kind in ("ta", "ra"):
            other = f"T{e.get('other', '')[6:]}"
            what = f"{other} 몸체 돌아가기" if kind == "ta" else f"{other} 경로 비키기"
            out.append((t, f"{name} {kind.upper()}({WHY_KO.get(e.get('why'), e.get('why'))}) {what}: {HOW_KO.get(e.get('how'), e.get('how'))}"))
        elif kind == "cc":
            out.append((t, f"{name} CC — T{e.get('other', '')[6:]} 경로에서 비킴"))
        elif kind == "window_expired":
            other = f"T{e.get('other', '')[6:]}"
            out.append((t, f"{name} 통표 만료 → " + (f"{other} 먼저" if e.get("turned") else "다시 발급")))
        elif kind == "job_held":
            out.append((t, f"{name} 작업 시작 보류(순서 고리)"))
    for j in jobs:
        if j["status"] == "done":
            out.append((j["t_s"], f"T{j['truck']} 작업 {j['job']} 완료 ({j['took_s']:.0f} s)"))
        elif j["status"] == "plan_failed":
            out.append((j["t_s"], f"T{j['truck']} 작업 {j['job']} 경로 없음"))
    return sorted(out, key=lambda x: x[0])


def ra_spans(events: list) -> dict[int, list]:
    """Truck -> [(from, until, other)]: in RA from a carried-out RA to its CC."""
    spans: dict[int, list] = {}
    for e in sorted(events, key=lambda e: e["t_s"]):
        if e.get("kind") == "ra" and e.get("how") in ("detour", "retreat"):
            spans.setdefault(int(e["truck"][6:]), []).append([e["t_s"], np.inf, int(e["other"][6:])])
        elif e.get("kind") == "cc":
            for span in spans.get(int(e["truck"][6:]), []):
                if span[1] == np.inf:
                    span[1] = e["t_s"]
    return spans


def frames(path: Path, width: int, height: int):
    proc = subprocess.Popen(
        ["ffmpeg", "-loglevel", "error", "-i", str(path), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
        stdout=subprocess.PIPE,
    )
    size = width * height * 3
    while True:
        raw = proc.stdout.read(size)
        if len(raw) < size:
            break
        yield np.frombuffer(raw, np.uint8).reshape(height, width, 3)
    proc.wait()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title")
    parser.add_argument("--speed", type=int, default=3, help="playback speed-up (frames kept 1 in N)")
    parser.add_argument("--trail-s", type=float, default=30.0)
    parser.add_argument("--max-s", type=float)
    parser.add_argument("--preview-png", type=Path)
    parser.add_argument("--preview-s", type=float, default=100.0)
    parser.add_argument("--planned-route", action="store_true", help="dashed route still ahead of each truck")
    args = parser.parse_args(argv)
    camera = json.loads((args.run / "camera.json").read_text())
    result = json.loads((args.run / "result.json").read_text())
    world = json.loads((args.run / "world.json").read_text())
    log = np.load(args.run / "world_log.npz")
    t_log, truth = log["t_s"], log["truth"]
    stage, loaded, blocked = log["stage"].astype(int), log["loaded"], log["blocked"].astype(int)
    field = log["field"] if "field" in log else np.full(blocked.shape, -1.0)
    alert = log["alert"].astype(int) if "alert" in log else None
    real = log["pallets"] if "pallets" in log else None  # each truck's real pallet
    tablets = Tablets(args.run, log) if (args.run / "routes.npz").exists() and "granted" in log else None
    if args.planned_route and tablets is None:
        parser.error("--planned-route needs routes.npz and a log with 'granted'")
    windows = None
    if (args.run / "windows.json").exists():
        snapshots = json.loads((args.run / "windows.json").read_text())
        windows = (np.array([w[0] for w in snapshots]), [w[1] for w in snapshots])
    jobs, people = log["jobs"], log["people"]
    project = Projector(camera)
    width, height = camera["resolution"]
    fps = camera["fps"]
    fp_unloaded, fp_loaded = world["footprints"]["unloaded"], world["footprints"]["loaded"]
    footprints, margin_m = (fp_unloaded, fp_loaded), result["arguments"]["margin_m"]
    n = truth.shape[1]
    title = args.title or (
        f"지게차 {n}대 · {STRUCTURE_KO.get(world['structure'], world['structure'])} "
        f"{world['size_m'][0]:.0f}×{world['size_m'][1]:.0f} m · seed {world['seed']}"
    )
    title_font, big, mid, small = font(32), font(24), font(20), font(18)
    encoder = None
    if args.preview_png is None:
        encoder = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{W}x{H}", "-r", str(fps), "-i", "-", "-c:v", "libx264", "-crf", "20",
             "-pix_fmt", "yuv420p", str(args.output)],
            stdin=subprocess.PIPE,
        )
    events = json.loads((args.run / "events.json").read_text())
    spills = [e for e in world.get("extra", []) if e[0] == "spill"]
    spill_times = [e["t_s"] for e in events if e.get("kind") == "spill"]
    ticker = event_lines(events, result.get("jobs", []))
    in_ra = ra_spans(events)
    clears = [(e["t_s"], int(e["truck"][6:])) for e in events if e.get("kind") == "cc"]
    expiries = [(e["t_s"], int(e["truck"][6:])) for e in events if e.get("kind") == "window_expired"]
    depth, width_p = world["footprints"]["pallet"]
    places = []  # (polygon, colour): pallet places by role, then holding places
    for _, role, x, y, yaw, _ in world["slots"]:
        poly = project(corners((x, y, yaw), (depth / 2, depth / 2, width_p / 2)))
        places.append(([tuple(q) for q in poly], (235, 200, 140, 150) if role == "source" else (120, 220, 235, 170)))
    for x, y, yaw in world["holding"]:
        places.append(([tuple(q) for q in project(corners((x, y, yaw), fp_unloaded))], (190, 195, 205, 110)))
    written = 0
    for index, frame in enumerate(frames(args.run / "overview.mp4", width, height)):
        t = index / fps
        if args.max_s is not None and t > args.max_s:
            break
        if args.preview_png is not None:
            if t < args.preview_s:
                continue
        elif index % args.speed:
            continue
        row = int(np.clip(np.searchsorted(t_log, t), 0, len(t_log) - 1))
        canvas = Image.new("RGB", (W, H), (16, 18, 22))
        canvas.paste(Image.fromarray(frame).resize((VIEW_W, VIEW_H), Image.BILINEAR), (0, TOP))
        draw = ImageDraw.Draw(canvas, "RGBA")
        draw.text((24, 16), title, font=title_font, fill=(240, 240, 240))
        for poly, colour in places:
            draw.polygon(poly, outline=colour, width=1)
        for k, (_, x0, y0, x1, y1) in enumerate(spills):
            if k < len(spill_times) and t >= spill_times[k]:
                pts = project([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
                draw.polygon([tuple(p) for p in pts], outline=(255, 230, 40, 255), width=3)
        if real is not None:
            x0b, x1b, y0b, y1b = world["bounds"]
            for x, y, yaw in real[row]:
                if x0b <= x <= x1b and y0b <= y <= y1b:  # the rest wait outside the floor
                    poly = project(corners((x, y, yaw), (depth / 2, depth / 2, width_p / 2)))
                    draw.polygon([tuple(q) for q in poly], fill=(225, 185, 120, 230), outline=(90, 60, 30, 255))
        order = tablets.order(row) if tablets is not None and windows is None else {}
        if tablets is not None:
            for k in range(n):
                colour = COLOURS[k % len(COLOURS)]
                held, ahead = tablets.claim(row, k)
                paint_union(canvas, [project(rect_corners(r)) for r in held], colour, 95)
                if args.planned_route:
                    continue  # the dashed route below shows what lies ahead
                for p in project(ahead) if len(ahead) else []:
                    draw.ellipse([p[0] - 2, p[1] - 2, p[0] + 2, p[1] + 2], fill=(*colour, 200))
        ra_now = {k: other for k, spans in in_ra.items() for t0, t1, other in spans if t0 <= t < t1}
        if tablets is not None:
            for k, other in ra_now.items():  # the higher-ranked route this truck treats as a wall
                wall = tablets.remaining(row, other)
                wall = wall[np.hypot(*(wall[:, :2] - truth[row, k, :2]).T) <= WALL_M][::3]
                paint_union(canvas, [project(rect_corners(r)) for r in wall], RA_COLOUR, 60)
        if args.planned_route:
            for k in range(n):
                rest = tablets.remaining(row, k)
                if len(rest) < 2:  # idle: the route is the truck's own pose
                    continue
                colour = COLOURS[k % len(COLOURS)]
                pts = project(np.vstack((truth[row, k, :2], route_points(rest, margin_m, footprints))))
                dashed(draw, pts, (*colour, 235), 2)
                end = pts[-1]
                draw.ellipse([end[0] - 6, end[1] - 6, end[0] + 6, end[1] + 6], fill=(*colour, 255), outline=(0, 0, 0), width=2)
        cleared = {k for when, k in clears if 0.0 <= t - when < CC_SHOW_S}
        expired = {k for when, k in expiries if 0.0 <= t - when < CC_SHOW_S}
        if windows is not None:
            at = int(np.searchsorted(windows[0], t, side="right")) - 1
            for first_name, second_name, wx, wy, state in windows[1][at] if at >= 0 else ():
                if state == 0:
                    continue
                f_k, s_k = int(first_name[6:]), int(second_name[6:])
                cx, cy = project([[wx, wy]])[0]
                diamond = [(cx, cy - 11), (cx + 11, cy), (cx, cy + 11), (cx - 11, cy)]
                colour = COLOURS[f_k % len(COLOURS)]
                draw.polygon(diamond, fill=(*colour, 230) if state == 2 else None, outline=(*colour, 255), width=3)
                if blocked[row, s_k] == f_k:
                    there = project(truth[row, s_k, :2])[0]
                    draw.line([(cx, cy), tuple(there)], fill=(*colour, 140), width=2)
        first = int(np.searchsorted(t_log, t - args.trail_s))
        for k in range(n):
            colour = COLOURS[k % len(COLOURS)]
            trail = project(truth[first : row + 1, k, :2])
            if len(trail) > 1:
                draw.line([tuple(p) for p in trail], fill=(*colour, 170), width=3)
            fp = fp_loaded if loaded[row, k] else fp_unloaded
            draw.polygon([tuple(p) for p in project(corners(truth[row, k], fp))], outline=(*colour, 255), width=3)
            here = project(truth[row, k, :2])[0]
            draw.text((here[0] + 8, here[1] - 26), f"T{k}", font=mid, fill=(255, 255, 255), stroke_width=3, stroke_fill=(0, 0, 0))
            if k in order:
                bx, by = here[0] + 44, here[1] - 16
                draw.ellipse([bx - 11, by - 11, bx + 11, by + 11], fill=(*colour, 255), outline=(0, 0, 0), width=2)
                draw.text((bx, by), str(order[k]), font=small, fill=(0, 0, 0), anchor="mm")
            wait_colour = (255, 70, 70) if alert is None else TA_COLOUR
            if k in ra_now:
                other = project(truth[row, ra_now[k], :2])[0]
                draw.ellipse([here[0] - 19, here[1] - 19, here[0] + 19, here[1] + 19], outline=RA_COLOUR, width=5)
                draw.line([tuple(here), tuple(other)], fill=(*RA_COLOUR, 170), width=3)
            elif blocked[row, k] >= 0:
                other = project(truth[row, blocked[row, k], :2])[0]
                draw.ellipse([here[0] - 16, here[1] - 16, here[0] + 16, here[1] + 16], outline=wait_colour, width=3)
                draw.line([tuple(here), tuple(other)], fill=(*wait_colour, 150), width=2)
            elif 0.0 <= field[row, k] < PERSON_STOP_M:
                draw.ellipse([here[0] - 16, here[1] - 16, here[0] + 16, here[1] + 16], outline=(255, 80, 140), width=3)
            if k in cleared:
                draw.ellipse([here[0] - 23, here[1] - 23, here[0] + 23, here[1] + 23], outline=CC_COLOUR, width=4)
            if k in expired:
                draw.ellipse([here[0] - 27, here[1] - 27, here[0] + 27, here[1] + 27], outline=EXPIRED_COLOUR, width=4)
        if people.ndim == 3 and people.shape[1]:
            for p in project(people[row]):
                draw.ellipse([p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6], fill=(255, 80, 140, 255), outline=(255, 255, 255))
        x0 = VIEW_W + 20
        draw.text((x0, 84), f"시뮬레이션 시각  {t:6.0f} s", font=big, fill=(240, 240, 240))
        draw.text((x0, 118), f"완료한 작업  {int(jobs[row].sum())} 건", font=big, fill=(240, 240, 240))
        y = 164
        events_top = TOP + VIEW_H - 28 * (EVENT_ROWS + 1)
        step = min(66, (events_top - 10 - y) // max(1, n))
        for k in range(n):
            colour = COLOURS[k % len(COLOURS)]
            name = LOG_STAGES[stage[row, k]] if 0 <= stage[row, k] < len(LOG_STAGES) else "idle"
            if k in ra_now:
                status = f"RA → T{ra_now[k]} 경로 비킴"
            elif blocked[row, k] >= 0:
                status = f"{'TA · ' if alert is not None else ''}교통 대기 → T{blocked[row, k]}"
            elif k in cleared:
                status = "CC"
            elif k in expired:
                status = "통표 만료"
            elif 0.0 <= field[row, k] < PERSON_STOP_M:
                status = "사람 앞 정지"
            else:
                status = "적재" if loaded[row, k] else ""
            if k in order:
                status = f"순번 {order[k]}" + (f" · {status}" if status else "")
            draw.rectangle([x0, y + 2, x0 + 10, y + step - 10], fill=colour)
            draw.text((x0 + 20, y), f"T{k}  {int(jobs[row, k])}건", font=mid, fill=(235, 235, 235))
            draw.text((x0 + 140, y + 2), " · ".join(s for s in (STAGE_KO.get(name, name), status) if s), font=small, fill=(200, 205, 215))
            y += step
        draw.text((x0, events_top), "최근 사건", font=mid, fill=(235, 235, 235))
        recent = [line for line in ticker if line[0] <= t][-EVENT_ROWS:]
        for i, (when, text) in enumerate(recent):
            draw.text((x0, events_top + 30 + 28 * i), f"{when:5.0f} s  {text}", font=small, fill=(200, 205, 215))
        rest_ko = "점선: 앞으로 갈 계획 경로(작업 끝까지, 점 = 끝)" if args.planned_route else "점: 남은 경로"
        if windows is not None:
            draw.text((24, VIEW_H + TOP + 12), f"색 띠: 점유 구간(그 트럭만 들어가 달릴 수 있는 곳) · 마름모: 교차 통표(색 = 먼저 지나갈 트럭, 빈 = 발급, 채움 = 들어감) · {rest_ko}", font=small, fill=(225, 228, 235))
            draw.text((24, VIEW_H + TOP + 40), "주황 고리·선: 대기 — 누구 때문에 · 자홍 고리: 통표 만료(창 안에 못 들어감) · 빨간 고리·띠: RA · 초록 고리: CC", font=small, fill=(225, 228, 235))
        elif tablets is not None and alert is not None:
            draw.text((24, VIEW_H + TOP + 12), f"색 띠: 통표(그 트럭만 들어가 달릴 수 있는 구간) · {rest_ko} · 동그라미 숫자: 순번(1 이 먼저)", font=small, fill=(225, 228, 235))
            draw.text((24, VIEW_H + TOP + 40), "주황 고리·선: TA — 누구 때문에 대기 · 빨간 고리·띠: RA — 후순위가 선순위 경로(빨간 띠)를 벽으로 보고 비킴 · 초록 고리: CC — 풀림", font=small, fill=(225, 228, 235))
        elif tablets is not None:
            draw.text((24, VIEW_H + TOP + 12), f"색 띠: 통표(점유 구간) — 그 트럭만 들어가 달릴 수 있는 곳 · {rest_ko} · 동그라미 숫자: 순번(1 이 먼저) · 빨간 고리·선: 누구 때문에 대기", font=small, fill=(225, 228, 235))
        else:
            draw.text((24, VIEW_H + TOP + 12), "외곽선: 트럭(실은 팔레트 포함) · 선: 최근 30 s 궤적 · 빨간 고리: 교통 관리로 대기 · 분홍 고리: 사람 앞 정지 · 분홍 점: 사람 · 노란 사각형: 쏟아진 짐", font=small, fill=(200, 205, 215))
        scene_note = (
            "랙·기둥·벽·컨베이어·팔레트·사람: NVIDIA Isaac 에셋(겉모습만 — 충돌·계획은 같은 상자)"
            if result.get("dressing") else "주황: 랙 · 파랑: 기계 · 회색: 벽·기둥"
        )
        below = VIEW_H + TOP + (68 if alert is not None else 40)
        draw.text((24, below), f"가는 선: 팔레트 칸(황토 = 출발, 하늘 = 도착), 대기 칸(회색) · {scene_note}", font=small, fill=(150, 155, 165))
        draw.text((24, below + 28), "합성 시뮬레이션(Isaac Sim 5.1) · 실측 차체 모델 · 정답 위치(SLAM 끔) · 무작위로 만든 공장(실재 도면 아님)", font=small, fill=(150, 155, 165))
        if args.preview_png is not None:
            canvas.save(args.preview_png)
            written = 1
            break
        encoder.stdin.write(np.asarray(canvas, np.uint8).tobytes())
        written += 1
    if encoder is not None:
        encoder.stdin.close()
        encoder.wait()
    print(json.dumps({"frames_written": written, "output": str(args.output), "jobs_done": result.get("jobs_done")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
