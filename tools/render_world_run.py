"""Top-down videos of tools/world_cpu_sim.py runs: one run alone, or several side by side.

    python tools/render_world_run.py single --run <cell dir> --label "Q3 — ..." --output q3.mp4 [--speed 20]
    python tools/render_world_run.py compare --run <dir> ... --label "..." ... --output all.mp4 [--speed 20]

Plan: docs/plans/2026-10-17-fleet-large-random-chained.md (「3,600 s 영상」). Reads each
run's world.json, result.json and trajectory.npz (1 Hz; positions are interpolated
between samples). Pallets are not logged: the floor starts filled as world_mission's
Dispatcher fills it (``od="random"``: every slot with probability ``fill``, from
``default_rng([seed, trucks, 11])``) and a slot empties or fills where a truck's
forks are when its load flag changes (the nearest full or empty slot, which must be
within 1 m). Drawn: racks and walls, pallets, holding places, people, spills and
broken trucks from their event time on, each truck with its last 40 s of track; an
orange ring and line where a truck waits for another (``blocked_by``), amber / red
rings for TA / RA, a magenta ring with its seconds once a truck has stood over 60 s
in a driving stage for any reason. ``compare`` puts the runs in a grid (three a row; two runs
side by side, larger) above two live
charts (jobs done, trucks standing in a driving stage) and runs to the longest of
them: a run that stopped early (gridlock) stays on its last second, dimmed, with
why and when it stopped; ``single`` holds that last second for HOLD_S. Frames go to ffmpeg on PATH;
``--workers N`` renders N stretches of the video at once and joins them (stream copy).
The labels are Korean: a Noto Sans CJK Bold from FONTS, or the file RENDER_FONT names.
"""

from __future__ import annotations

import argparse
import colorsys
import json
import multiprocessing as mp
import os
import subprocess
import tempfile
from itertools import pairwise
from math import cos, pi, sin
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from forklift_core.planning.world_mission import Dispatcher

FPS = 30
FONTS = [
    os.environ.get("RENDER_FONT", ""),
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/google-noto-cjk/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/noto-cjk/NotoSansCJK-Bold.ttc",
]
BG, FLOOR, RACK, WALL, SLOT = (16, 18, 22), (30, 33, 40), (70, 78, 94), (98, 104, 118), (52, 58, 70)
PALLET, HOLD, PERSON = (176, 139, 87), (90, 96, 110), (80, 220, 240)
WAIT, TA, RA, STOOD, SPILL = (255, 150, 40), (250, 200, 60), (240, 70, 70), (230, 80, 230), (230, 120, 40)
TEXT, DIM = (235, 237, 242), (150, 155, 165)
DRIVING = range(1, 8)  # world_cpu_sim LOG_STAGES: idle, leave ... to_holding, lift, lower, broken
BROKEN = 10
TRAIL_S, STOOD_S = 40.0, 60.0
HOLD_S = 3.0  # video seconds a run that stopped early is held at its end


def font(size: int) -> ImageFont.FreeTypeFont:
    for path in FONTS:
        if path and Path(path).exists():
            return ImageFont.truetype(path, size)
    raise SystemExit(f"no CJK font among {FONTS}: set RENDER_FONT")  # the default font has no Hangul


def truck_colours(n: int) -> list[tuple[int, int, int]]:
    return [tuple(int(255 * v) for v in colorsys.hsv_to_rgb((k * 0.618) % 1.0, 0.55, 0.98)) for k in range(n)]


class Run:
    """One run's floor, track and the pallets on it over time."""

    def __init__(self, path: Path) -> None:
        self.world = json.loads((path / "world.json").read_text())
        self.result = json.loads((path / "result.json").read_text())
        z = np.load(path / "trajectory.npz")
        self.trucks, self.people = z["trucks"], z["people"]
        self.n = self.trucks.shape[1]
        self.end_s = float(len(self.trucks) - 1)
        self.done_s = np.sort([j["t_s"] for j in self.result["jobs"] if j.get("status") == "done"])
        self.status, self.sim_s = self.result["status"], float(self.result["sim_s"])
        self.events = self.result.get("events", [])
        slots = self.world["slots"]
        self.slot_xy = np.array([[s[2], s[3]] for s in slots])
        self.slot_yaw = np.array([s[4] for s in slots])
        rng = np.random.default_rng([self.result["seed"], self.result["trucks"], 11])
        fill = Dispatcher.__dataclass_fields__["fill"].default
        random_od = self.result.get("od") == "random"
        occupied = np.array([(random_od or s[1] == "source") and rng.random() < fill for s in slots])
        self.initial = occupied.copy()
        front = self.world["footprints"]["loaded"][0]
        depth = self.world["footprints"]["pallet"][0]
        self.changes = []  # (second, slot, occupied)
        loaded = self.trucks[:, :, 3] > 0.5
        for i, k in zip(*np.nonzero(loaded[1:] != loaded[:-1]), strict=True):
            picked = bool(loaded[i + 1, k])
            x, y, yaw = self.trucks[i + 1 if picked else i, k, :3]
            fork = np.array([x + (front - depth / 2) * cos(yaw), y + (front - depth / 2) * sin(yaw)])
            gap = np.where(occupied == picked, np.hypot(*(self.slot_xy - fork).T), np.inf)
            j = int(np.argmin(gap))
            if gap[j] > 1.0:
                raise ValueError(f"truck {k} at {i + 1} s: no {'full' if picked else 'empty'} slot under its forks")
            occupied[j] = not picked
            self.changes.append((i + 1, j, not picked))
        speed = np.zeros(self.trucks.shape[:2])
        speed[1:] = np.hypot(*(self.trucks[1:, :, :2] - self.trucks[:-1, :, :2]).transpose(2, 0, 1))
        still = (speed < 0.05) & np.isin(self.trucks[:, :, 4], DRIVING)
        self.stood = np.zeros(still.shape)  # seconds a truck has stood so far (0 while it moves)
        for i in range(1, len(still)):
            self.stood[i] = np.where(still[i], self.stood[i - 1] + 1.0, 0.0)
        for e in self.events:  # a fault pause is not standing in traffic
            if e["kind"] == "pause":
                k = int(e["truck"].split("_")[1])
                lo, hi = int(e["t_s"]), min(int(e["until"]) + 1, len(still))
                self.stood[lo:hi, k] = 0.0
                for i in range(hi, len(still)):
                    if not still[i, k]:
                        break
                    self.stood[i, k] = self.stood[i - 1, k] + 1.0 if i > hi else 1.0

    def occupied_at(self, t: float) -> np.ndarray:
        occ = self.initial.copy()
        for s, j, full in self.changes:
            if s > t:
                break
            occ[j] = full
        return occ

    def pose(self, t: float) -> np.ndarray:
        """(trucks, 3) poses at t, linear between the 1 Hz samples."""
        i = min(int(t), len(self.trucks) - 2)
        f = min(max(t - i, 0.0), 1.0)
        a, b = self.trucks[i, :, :3], self.trucks[i + 1, :, :3]
        yaw = a[:, 2] + f * ((b[:, 2] - a[:, 2] + pi) % (2 * pi) - pi)
        return np.column_stack((a[:, :2] + f * (b[:, :2] - a[:, :2]), yaw))

    def state(self, t: float) -> np.ndarray:
        return self.trucks[min(int(t), len(self.trucks) - 1)]

    def jobs_by(self, t: float) -> int:
        return int(np.searchsorted(self.done_s, t, side="right"))

    def ended(self, t: float) -> list[str]:
        """Why and when the run stopped, once t is past its end ([] while it runs or if its time ran out)."""
        if t <= self.end_s or self.status == "time_up":
            return []
        if self.status == "gridlock":
            return [f"{self.sim_s:,.0f} s 에 교착으로 끝남", "작업 중인 트럭이 하나도 나아가지 못함"]
        return [f"{self.sim_s:,.0f} s 에 끝남 ({self.status})"]

    def standing(self, t: float) -> int:
        """Trucks standing in a driving stage at t for any reason."""
        i = min(int(t), len(self.trucks) - 1)
        return int((self.stood[i] > 0).sum())


class View:
    """A run drawn into a box of the frame: metres -> pixels."""

    def __init__(self, run: Run, box: tuple[int, int, int, int]) -> None:
        self.run = run
        x0, y0, x1, y1 = box
        bx0, bx1, by0, by1 = run.world["bounds"]
        self.scale = min((x1 - x0) / (bx1 - bx0), (y1 - y0) / (by1 - by0))
        w, h = (bx1 - bx0) * self.scale, (by1 - by0) * self.scale
        self.ox, self.oy = x0 + ((x1 - x0) - w) / 2 - bx0 * self.scale, y0 + ((y1 - y0) - h) / 2 + by1 * self.scale
        self.colours = truck_colours(run.n)
        self.label_font = font(max(9, int(self.scale * 0.9)))

    def px(self, x, y):
        return self.ox + x * self.scale, self.oy - y * self.scale

    def rect(self, x, y, yaw, front, rear, half):
        c, s = cos(yaw), sin(yaw)
        corners = [(front, half), (front, -half), (-rear, -half), (-rear, half)]
        return [self.px(x + a * c - b * s, y + a * s + b * c) for a, b in corners]

    def background(self, image: Image.Image) -> None:
        d = ImageDraw.Draw(image)
        bx0, bx1, by0, by1 = self.run.world["bounds"]
        d.rectangle([*self.px(bx0, by1), *self.px(bx1, by0)], fill=FLOOR)
        for _, zx0, zy0, zx1, zy1 in self.run.world["zones"]:
            d.rectangle([*self.px(zx0, zy1), *self.px(zx1, zy0)], fill=(36, 40, 48))
        for kind, ox0, oy0, ox1, oy1 in self.run.world["obstacles"]:
            d.rectangle([*self.px(ox0, oy1), *self.px(ox1, oy0)], fill=WALL if kind == "wall" else RACK)
        depth, width = self.run.world["footprints"]["pallet"]
        for (x, y), yaw in zip(self.run.slot_xy, self.run.slot_yaw, strict=True):
            d.polygon(self.rect(x, y, yaw, depth / 2, depth / 2, width / 2), outline=SLOT)
        fp = self.run.world["footprints"]["unloaded"]
        for x, y, yaw in self.run.world["holding"]:
            d.polygon(self.rect(x, y, yaw, *fp), outline=HOLD)

    def pallets(self, image: Image.Image, occupied: np.ndarray) -> None:
        d = ImageDraw.Draw(image)
        depth, width = self.run.world["footprints"]["pallet"]
        for (x, y), yaw in zip(self.run.slot_xy[occupied], self.run.slot_yaw[occupied], strict=True):
            d.polygon(self.rect(x, y, yaw, depth / 2, depth / 2, width / 2), fill=PALLET)

    def dynamic(self, image: Image.Image, t: float) -> None:
        run, d = self.run, ImageDraw.Draw(image, "RGBA")
        for e in run.events:
            if e["kind"] == "spill" and e["t_s"] <= t:
                bx0, by0, bx1, by1 = e["box"]
                d.rectangle([*self.px(bx0, by1), *self.px(bx1, by0)], fill=SPILL + (200,))
        lo = max(0, int(t - TRAIL_S))
        hi = min(int(t), len(run.trucks) - 1)
        for k in range(run.n):
            if hi - lo >= 1:
                pts = [self.px(*run.trucks[i, k, :2]) for i in range(lo, hi + 1)]
                d.line(pts, fill=self.colours[k] + (110,), width=max(1, int(self.scale * 0.25)))
        people = run.people[min(int(t), len(run.people) - 1)]
        r = max(2.0, 0.35 * self.scale)
        for x, y in people:
            cx, cy = self.px(x, y)
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=PERSON)
        poses, state = run.pose(t), run.state(t)
        stood = run.stood[min(int(t), len(run.stood) - 1)]
        unloaded, loaded = run.world["footprints"]["unloaded"], run.world["footprints"]["loaded"]
        depth, width = run.world["footprints"]["pallet"]
        for k in range(run.n):
            x, y, yaw = poses[k]
            carrying, stage, blocker, alert = state[k, 3] > 0.5, int(state[k, 4]), int(state[k, 5]), int(state[k, 6])
            colour = (120, 120, 120) if stage == BROKEN else self.colours[k]
            d.polygon(self.rect(x, y, yaw, *(loaded if carrying else unloaded)), fill=colour, outline=(10, 10, 10))
            if carrying:
                fx = loaded[0] - depth / 2
                d.polygon(self.rect(x + fx * cos(yaw), y + fx * sin(yaw), yaw, depth / 2, depth / 2, width / 2), fill=PALLET)
            cx, cy = self.px(x, y)
            ring = None
            if stage == BROKEN:
                ring = RA
                q = 1.2 * self.scale
                d.line([cx - q, cy - q, cx + q, cy + q], fill=RA, width=3)
                d.line([cx - q, cy + q, cx + q, cy - q], fill=RA, width=3)
            elif stood[k] >= STOOD_S:
                ring = STOOD
            elif alert == 2:
                ring = RA
            elif alert == 1:
                ring = TA
            elif blocker >= 0:
                ring = WAIT
            if blocker >= 0 and stage != BROKEN:
                d.line([cx, cy, *self.px(*poses[blocker, :2])], fill=WAIT + (170,), width=max(1, int(self.scale * 0.2)))
            if ring is not None:
                rr = 2.0 * self.scale
                d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=ring, width=max(2, int(self.scale * 0.3)))
            if self.scale >= 8:
                tag = f"T{k}" + (f" {int(stood[k])}s" if stood[k] >= STOOD_S and stage != BROKEN else "")
                d.text((cx + 1.6 * self.scale, cy - 1.6 * self.scale), tag, font=self.label_font, fill=TEXT)


class Layer:
    """The floor with its pallets, redrawn only when a pallet moves."""

    def __init__(self, view: View, size: tuple[int, int]) -> None:
        self.view, self.base = view, Image.new("RGB", size, BG)
        view.background(self.base)
        self.key, self.image = None, None

    def at(self, t: float) -> Image.Image:
        changes = self.view.run.changes
        key = sum(1 for s, _, _ in changes if s <= t)
        if key != self.key:
            self.key, self.image = key, self.base.copy()
            self.view.pallets(self.image, self.view.run.occupied_at(t))
        return self.image


def encoder(path: Path, size: tuple[int, int]) -> subprocess.Popen:
    cmd = ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{size[0]}x{size[1]}",
           "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-crf", "20", "-preset", "medium", "-pix_fmt", "yuv420p",
           "-movflags", "+faststart", str(path)]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)


def legend(d: ImageDraw.ImageDraw, x: int, y: int, size: int) -> None:
    f, items = font(size), [(PALLET, "팔레트"), (PERSON, "사람"), (WAIT, "다른 트럭 때문에 기다림(선: 막은 트럭)"),
                            (TA, "TA"), (RA, "RA · 고장"), (STOOD, "주행 중 60 s 넘게 멈춤(이유 무관)"), (SPILL, "쏟아진 짐")]
    for colour, text in items:
        d.ellipse([x, y + size * 0.2, x + size * 0.7, y + size * 0.9], fill=colour)
        d.text((x + size, y), text, font=f, fill=DIM)
        x += int(d.textlength(text, font=f)) + int(size * 2.2)


def banner(image: Image.Image, box: tuple[int, int, int, int], lines: list[str], f) -> None:
    """box dimmed, with lines across its middle: a run that has stopped."""
    region = image.crop(box)
    image.paste(Image.blend(region, Image.new("RGB", region.size, BG), 0.55), box[:2])
    d = ImageDraw.Draw(image)
    x0, y0, x1, y1 = box
    y = (y0 + y1) / 2 - len(lines) * f.size * 0.7
    width = max(d.textlength(line, font=f) for line in lines)
    d.rectangle([(x0 + x1 - width) / 2 - 12, y - 6, (x0 + x1 + width) / 2 + 12, y + len(lines) * f.size * 1.4 + 6], fill=BG)
    for line in lines:
        d.text(((x0 + x1 - d.textlength(line, font=f)) / 2, y), line, font=f, fill=RA)
        y += f.size * 1.4


def stretch(frames: int, part: tuple[int, int]) -> range:
    """The frames of part k of n."""
    k, n = part
    return range(frames * k // n, frames * (k + 1) // n)


def single(args, part=(0, 1), output=None) -> None:
    run = Run(args.run[0])
    size = (1920, 1080)
    box = (20, 70, 1900, 1000)
    view = View(run, box)
    layer = Layer(view, size)
    big, mid, small = font(30), font(24), font(20)
    end_s = min(run.end_s, args.max_s or run.end_s)
    held = round(HOLD_S * FPS) if run.ended(end_s + 1) else 0
    frames = int(end_s / args.speed * FPS) + 1 + held
    video = encoder(output or args.output, size)
    for f in stretch(frames, part):
        t = min(f * args.speed / FPS, end_s)
        image = layer.at(t).copy()
        view.dynamic(image, t)
        if f * args.speed / FPS > end_s:
            banner(image, box, run.ended(end_s + 1), big)
        d = ImageDraw.Draw(image)
        d.text((24, 14), args.label[0], font=big, fill=TEXT)
        waiting = int((run.state(t)[:, 5] >= 0).sum())
        long_stood = int((run.stood[min(int(t), len(run.stood) - 1)] >= STOOD_S).sum())
        status = (f"시뮬레이션 {t:,.0f} / {end_s:,.0f} s ({args.speed}배속) · 완료 작업 {run.jobs_by(t)} · "
                  f"다른 트럭 때문에 기다림 {waiting} · 60 s 넘게 멈춤 {long_stood}")
        d.text((1896 - d.textlength(status, font=mid), 18), status, font=mid, fill=TEXT)
        legend(d, 24, 1010, 22)
        if args.note:
            d.text((24, 1046), args.note, font=small, fill=DIM)
        video.stdin.write(image.tobytes())
    video.stdin.close()
    video.wait()


def chart(d, box, series, t, end_s, colours, title, f):
    """Lines of series [(times, values)] up to t in box, with a time axis to end_s."""
    x0, y0, x1, y1 = box
    d.rectangle(box, outline=(60, 64, 74))
    d.text((x0 + 8, y0 + 4), title, font=f, fill=DIM)
    top = max(1.0, max(float(np.max(v)) for _, v in series)) * 1.05
    tags = []  # [y, x, text, colour] of each line's last value
    for (times, values), colour in zip(series, colours, strict=True):
        keep = times <= t
        if keep.sum() < 2:
            continue
        xs = x0 + (times[keep] / end_s) * (x1 - x0)
        ys = y1 - (values[keep] / top) * (y1 - y0 - 30)
        d.line(list(zip(xs.tolist(), ys.tolist(), strict=True)), fill=colour, width=3)
        tags.append([ys[-1] - 12, xs[-1] + 4, f"{values[keep][-1]:.0f}", colour])
    tags.sort()
    for a, b in pairwise(tags):  # stacked, not on top of each other, and inside the box
        b[0] = max(b[0], a[0] + f.size)
    floor = y1 - f.size - 2
    for tag in reversed(tags):
        tag[0] = min(tag[0], floor)
        floor = tag[0] - f.size
    for y, x, text, colour in tags:
        d.text((x, y), text, font=f, fill=colour)
    for s in range(0, int(end_s) + 1, 600):
        xs, tick = x0 + s / end_s * (x1 - x0), f"{s} s"
        width = d.textlength(tick, font=f)
        d.text((min(max(xs - width / 2, x0), x1 - width), y1 + 2), tick, font=f, fill=DIM)


def compare(args, part=(0, 1), output=None) -> None:
    runs = [Run(p) for p in args.run]
    size, cols = (1920, 1080), min(3, len(runs))
    rows = (len(runs) + cols - 1) // cols
    big = cols <= 2  # two runs side by side: larger maps and type
    lab, chart_font, head = font(26 if big else 20), font(22 if big else 18), 64 if big else 52
    bx0, bx1, by0, by1 = runs[0].world["bounds"]
    cell_w = size[0] // cols
    cell_h = min(700 // rows, int((cell_w - 12) * (by1 - by0) / (bx1 - bx0)) + head + 4)
    top = rows * cell_h  # the maps end here; the clock and the charts are below
    boxes = [((i % cols) * cell_w + 6, (i // cols) * cell_h + head, (i % cols + 1) * cell_w - 6, (i // cols + 1) * cell_h - 4)
             for i in range(len(runs))]
    views = [View(r, box) for r, box in zip(runs, boxes, strict=True)]
    palette = [(120, 200, 250), (250, 170, 80), (245, 210, 90), (230, 155, 230), (160, 230, 160), (255, 140, 140)]
    if args.colour:  # the same run keeps its colour across several videos
        palette = [tuple(int(c[k:k + 2], 16) for k in (0, 2, 4)) for c in args.colour]
    base = Image.new("RGB", size, BG)
    for v in views:
        v.background(base)
    end_s = min(max(r.end_s for r in runs), args.max_s or 1e9)
    jobs, standing = [], []
    for r in runs:
        grid = np.arange(0.0, min(r.end_s, end_s) + 1)
        jobs.append((grid, np.array([r.jobs_by(t) for t in grid])))
        counts = np.array([r.standing(t) for t in grid], dtype=float)  # mean of the last 30 s, none ahead of t
        standing.append((grid, np.convolve(counts, np.ones(30))[: len(grid)] / np.minimum(np.arange(1, len(grid) + 1), 30)))
    frames = int(end_s / args.speed * FPS) + 1
    video = encoder(output or args.output, size)
    key = None
    for f in stretch(frames, part):
        t = f * args.speed / FPS
        now = tuple(sum(1 for s, _, _ in r.changes if s <= t) for r in runs)
        if now != key:
            key, floor = now, base.copy()
            for v in views:
                v.pallets(floor, v.run.occupied_at(t))
        image = floor.copy()
        d = ImageDraw.Draw(image)
        for i, (v, name) in enumerate(zip(views, args.label, strict=True)):
            v.dynamic(image, t)
            if lines := v.run.ended(t):
                banner(image, boxes[i], lines, lab)
            x, y = (i % cols) * cell_w + 10, (i // cols) * cell_h + 4
            d.text((x, y), name, font=lab, fill=palette[i % len(palette)])
            d.text((x, y + lab.size + 4), f"완료 작업 {v.run.jobs_by(t)} · 멈춘 트럭 {v.run.standing(t)}", font=chart_font, fill=TEXT)
        clock = f"시뮬레이션 {t:,.0f} / {end_s:,.0f} s · {args.speed}배속"
        d.text((1900 - d.textlength(clock, font=lab), top + 8), clock, font=lab, fill=TEXT)
        if args.note:
            d.text((20, top + 10), args.note, font=chart_font, fill=DIM)
        chart(d, (20, top + 40, 940, 1050), jobs, t, end_s, palette, "완료 작업(누적)", chart_font)
        chart(d, (980, top + 40, 1900, 1050), standing, t, end_s, palette, "주행 단계에서 멈춰 선 트럭(30 s 평균, 이유 무관)", chart_font)
        video.stdin.write(image.tobytes())
    video.stdin.close()
    video.wait()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("mode", choices=("single", "compare"))
    parser.add_argument("--run", type=Path, action="append", required=True)
    parser.add_argument("--label", action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--max-s", type=float)
    parser.add_argument("--preview-png", action="store_true", help="write <output>.png at --max-s instead of a video")
    parser.add_argument("--workers", type=int, default=1, help="stretches rendered at once")
    parser.add_argument("--note", help="a line on the floor and run, drawn small")
    parser.add_argument("--colour", action="append", help="compare: RRGGBB of each run in --run order (default: a fixed palette)")
    args = parser.parse_args(argv)
    if len(args.run) != len(args.label):
        parser.error("one --label per --run")
    draw = single if args.mode == "single" else compare
    if args.preview_png:
        preview(args)
    elif args.workers <= 1:
        draw(args)
    else:
        with tempfile.TemporaryDirectory(prefix="render_", dir=args.output.parent) as tmp:
            parts = [Path(tmp) / f"part{k:03d}.mp4" for k in range(args.workers)]
            with mp.get_context("fork").Pool(args.workers) as pool:
                pool.starmap(draw, [(args, (k, args.workers), path) for k, path in enumerate(parts)])
            listing = Path(tmp) / "parts.txt"
            listing.write_text("".join(f"file '{path}'\n" for path in parts))
            subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(listing),
                            "-c", "copy", "-movflags", "+faststart", str(args.output)], check=True)
    return 0


def preview(args) -> None:
    """The frame nearest --max-s, as a PNG next to --output (layout checks)."""
    global encoder, stretch
    f = round((args.max_s or 0.0) / args.speed * FPS)
    args.max_s = None

    class Grab:
        def __init__(self):
            self.stdin, self.data = self, None

        def write(self, data):
            self.data = data

        def close(self):
            pass

        def wait(self):
            pass

    grab = Grab()
    real = encoder, stretch
    encoder, stretch = (lambda path, size: grab), (lambda frames, part: range(f, f + 1))
    try:
        (single if args.mode == "single" else compare)(args)
    finally:
        encoder, stretch = real
    Image.frombytes("RGB", (1920, 1080), grab.data).save(args.output.with_suffix(".png"))


if __name__ == "__main__":
    raise SystemExit(main())
