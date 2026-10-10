"""A close-up clip of traffic control's "tablets" (claims) being handed on, drawn from a run's logs.

    python tools/explain_tablets.py --run <run dir> --output <clip.mp4> [--t0 S --t1 S] [--span M]

Plan: docs/plans/2026-10-09-fleet-random-worlds.md (W6). Reads world.json,
world_log.npz and routes.npz written by sim/isaac/run_world.py (route,
progress, granted and rank per truck). Without --t0/--t1 it picks the
longest wait in the run, a truck held by another that is moving. The view
follows the two trucks in plan (2D, to scale); each truck's tablet -- the
part of its route it alone may drive, grown by the traffic margin -- is its
coloured band. The caption says why a waiting truck waits: its next stretch
would overlap the other's tablet or body, or it would have to stop on the
remaining route of a truck ahead of it in the order (the safe-stop rule).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

from forklift_core.planning.fleet_traffic import oriented_rects, overlap_matrix
from forklift_core.planning.geometry import Footprint

W, H = 1920, 1080
MAP_W = 1240
MARGIN_M = 0.3
COLOURS = [
    (230, 85, 60), (60, 140, 230), (245, 180, 40), (90, 190, 110), (170, 100, 220),
    (40, 200, 200), (240, 120, 180), (160, 160, 60), (120, 200, 250), (250, 150, 80),
]
KIND = {"wall": (88, 96, 104), "rack": (214, 150, 40), "machine": (80, 125, 180), "column": (130, 138, 146),
        "store": (150, 118, 76), "outside": (40, 44, 48), "spill": (226, 196, 0)}
FONTS = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", "/usr/share/fonts/truetype/nanum/NanumSquareB.ttf"]
FONTS_REG = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", *FONTS]


def font(size, regular=False):
    for path in FONTS_REG if regular else FONTS:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def corners(r) -> np.ndarray:
    cx, cy, hl, hw, c, s = r
    local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    return np.column_stack((cx + c * local[:, 0] - s * local[:, 1], cy + s * local[:, 0] + c * local[:, 1]))


class Run:
    def __init__(self, run: Path):
        self.world = json.loads((run / "world.json").read_text())
        log = np.load(run / "world_log.npz")
        self.t, self.truth, self.loaded = log["t_s"], log["truth"], log["loaded"]
        self.blocked = log["blocked"].astype(int)
        self.route, self.progress = log["route"].astype(int), log["progress"]
        self.granted, self.rank = log["granted"], log["rank"]
        rz = np.load(run / "routes.npz")
        self.start, self.rects, self.g = rz["start"], rz["rects"].astype(float), rz["g"].astype(float)
        f = self.world["footprints"]
        self.fp = (Footprint(*f["unloaded"]), Footprint(*f["loaded"]))
        self.n = self.truth.shape[1]

    def route_of(self, row, k):
        rid = self.route[row, k]
        if rid < 0:
            return np.zeros(0), np.zeros((0, 6))
        s0 = self.start[rid]
        s1 = self.start[rid + 1] if rid + 1 < len(self.start) else len(self.g)
        return self.g[s0:s1], self.rects[s0:s1]

    def body(self, row, k, margin=0.0) -> np.ndarray:
        return oriented_rects(self.truth[row, k][None], self.fp[int(self.loaded[row, k])], margin)

    def tablet(self, row, k) -> np.ndarray:
        g, rects = self.route_of(row, k)
        held = rects[(g >= self.progress[row, k] - 0.2) & (g <= self.granted[row, k] + 1e-6)]
        return np.vstack((self.body(row, k, MARGIN_M), held))

    def ahead(self, row, k, metres=30.0) -> np.ndarray:
        g, rects = self.route_of(row, k)
        return rects[(g > self.granted[row, k] + 1e-6) & (g <= self.granted[row, k] + metres)]

    def order(self, row) -> dict[int, int]:
        busy = [k for k in range(self.n) if self.rank[row, k] < 1e9]
        return {k: i + 1 for i, k in enumerate(sorted(busy, key=lambda k: self.rank[row, k]))}

    def why(self, row, k, b) -> str:
        """'tablet' if k's next stretch touches b's tablet or body, else 'safe_stop'."""
        nxt = self.ahead(row, k, 3.0)
        if len(nxt) and overlap_matrix(nxt, self.tablet(row, b)).any():
            return "tablet"
        return "safe_stop"

    def pick(self, reason: str | None = None, max_wait_s: float = 120.0) -> tuple[float, float, int, int]:
        """Longest wait (up to max_wait_s) on a moving blocker that is not itself waiting
        for the waiting truck (a cycle is a failure, not an example), of one reason if
        given: (t0, t1, waiting truck, blocker)."""
        best = None
        for k in range(self.n):
            b = self.blocked[:, k]
            i = 0
            while i < len(b):
                if b[i] < 0:
                    i += 1
                    continue
                j = i
                while j + 1 < len(b) and b[j + 1] == b[i]:
                    j += 1
                other = b[i]
                moved = float(np.hypot(*(self.truth[j, other, :2] - self.truth[i, other, :2])))
                score = (self.t[j] - self.t[i]) * (moved > 1.0)
                if self.t[j] - self.t[i] > max_wait_s or (self.blocked[i : j + 1, other] == k).any():
                    score = 0.0
                if reason is not None and score > 0 and self.why(i, k, other) != reason:
                    score = 0.0
                if best is None or score > best[0]:
                    best = (score, self.t[i], self.t[j], k, other)
                i = j + 1
        if best is None or best[0] <= 0:
            raise SystemExit("no wait on a moving truck in this run")
        return best[1] - 8.0, best[2] + 12.0, best[3], best[4]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--t0", type=float)
    ap.add_argument("--t1", type=float)
    ap.add_argument("--focus", type=int, nargs=2, metavar=("WAITING", "BLOCKER"))
    ap.add_argument("--span", type=float, default=32.0, help="metres across the map view")
    ap.add_argument("--reason", choices=("tablet", "safe_stop"), help="pick a wait of this kind")
    ap.add_argument("--fps", type=int, default=20, help="log rows are 10 Hz: 20 fps plays at 2x, 30 at 3x")
    ap.add_argument("--preview-png", type=Path)
    ap.add_argument("--preview-s", type=float, help="time of the preview frame (default: 3 s into the wait)")
    a = ap.parse_args(argv)
    run = Run(a.run)
    if a.t0 is None:
        t0, t1, kw, kb = run.pick(a.reason)
    else:
        t0, t1 = a.t0, a.t1
        kw, kb = a.focus if a.focus else (0, 1)
    rows = np.nonzero((run.t >= t0) & (run.t <= t1))[0]
    # The view: centred on the pair at the start of the wait, fixed (so motion reads).
    mid_rows = rows[len(rows) // 3]
    centre = (run.truth[mid_rows, kw, :2] + run.truth[mid_rows, kb, :2]) / 2
    scale = MAP_W / a.span
    view_h = H - 120

    def px(xy):
        xy = np.atleast_2d(xy)
        return np.column_stack((MAP_W / 2 + (xy[:, 0] - centre[0]) * scale, 60 + view_h / 2 - (xy[:, 1] - centre[1]) * scale))

    base = Image.new("RGB", (W, H), (24, 27, 31))
    bd = ImageDraw.Draw(base)
    bd.rectangle([0, 60, MAP_W, 60 + view_h], fill=(52, 57, 62))
    for kind, x0, y0, x1, y1 in run.world["obstacles"] + run.world.get("extra", []):
        p = px([[x0, y0], [x1, y1]])
        bd.rectangle([min(p[:, 0]), min(p[:, 1]), max(p[:, 0]), max(p[:, 1])], fill=KIND.get(kind, (120, 120, 120)))
    dep, wid = run.world["footprints"]["pallet"]
    for _, role, x, y, yaw, _ in run.world["slots"]:
        poly = px(corners((x, y, dep / 2, wid / 2, np.cos(yaw), np.sin(yaw))))
        bd.polygon([tuple(q) for q in poly], outline=(235, 200, 140) if role == "source" else (120, 220, 235), width=2)
    bd.rectangle([MAP_W, 0, W, H], fill=(24, 27, 31))
    bd.rectangle([0, 0, MAP_W, 60], fill=(24, 27, 31))  # obstacles stop at the map's edges
    bd.rectangle([0, 60 + view_h, MAP_W, H], fill=(24, 27, 31))
    # scale bar
    bd.line([(30, 60 + view_h - 30), (30 + 5 * scale, 60 + view_h - 30)], fill=(230, 230, 230), width=4)
    bd.text((30, 60 + view_h - 62), "5 m", font=font(22), fill=(230, 230, 230))
    title_f, big, mid, small, tiny = font(34), font(26), font(22), font(20, True), font(18, True)
    bd.text((24, 12), "통표 주고받기 — 확대 (교통 관리의 점유 구간)", font=title_f, fill=(240, 240, 240))
    structure = {"rack": "랙 창고", "shop": "공정 공장", "multi": "복합동"}.get(run.world["structure"], "")
    bd.text((MAP_W + 24, 70), f"{structure} · seed {run.world['seed']} · 2D 축척 그림(로그에서)", font=small, fill=(170, 176, 184))
    rules = [
        ("통표", "트럭은 자기 경로 앞쪽 일부를 「통표」로 받아, 그 띠 안에서만 달린다. 띠는 몸체보다 0.3 m 넓고, 다른 트럭의 띠·몸체와 겹칠 수 없다."),
        ("순번", "경로를 먼저 받은 트럭부터 띠를 늘린다(동그라미 숫자, 1 이 먼저). 새 작업·재계획은 맨 뒤."),
        ("정차 규정", "앞 순번 트럭의 남은 경로 위에서는 멈추지 않는다. 끝까지 지나갈 띠를 못 받으면 그 앞에서 기다린다."),
    ]
    y = 120
    for head, body in rules:
        bd.text((MAP_W + 24, y), head, font=mid, fill=(245, 190, 60))
        y += 32
        line = ""
        for word in body.split(" "):
            if bd.textlength(line + word + " ", font=tiny) > W - MAP_W - 48:
                bd.text((MAP_W + 24, y), line, font=tiny, fill=(215, 220, 226))
                y += 27
                line = ""
            line += word + " "
        bd.text((MAP_W + 24, y), line, font=tiny, fill=(215, 220, 226))
        y += 40
    panel_top = y + 10
    if a.preview_png is None:
        enc = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
             "-r", str(a.fps), "-i", "-", "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", str(a.output)],
            stdin=subprocess.PIPE,
        )
    base_map = base.crop((0, 60, MAP_W, 60 + view_h))  # dynamic drawing stays inside the map

    def mp(xy):
        return px(xy) - np.array([0.0, 60.0])

    was_waiting = {}
    note, note_until = "", -1.0
    written = 0
    for row in rows:
        m = base_map.copy()
        dm = ImageDraw.Draw(m, "RGBA")
        order = run.order(row)
        for k in range(run.n):
            colour = COLOURS[k % len(COLOURS)]
            polys = [mp(corners(r)) for r in run.tablet(row, k)]
            pts = np.vstack(polys)
            if pts[:, 0].max() < 0 or pts[:, 0].min() > MAP_W or pts[:, 1].max() < 0 or pts[:, 1].min() > view_h:
                continue
            mask = Image.new("L", m.size, 0)
            md = ImageDraw.Draw(mask)
            for poly in polys:
                md.polygon([tuple(q) for q in poly], fill=255)
            solid = Image.new("RGB", m.size, colour)
            m.paste(solid, (0, 0), mask.point(lambda v: v * 110 // 255))
            m.paste(solid, (0, 0), mask.filter(ImageFilter.FIND_EDGES))
            ahead = run.ahead(row, k)
            for q in mp(ahead[::4, :2]) if len(ahead) else []:
                dm.ellipse([q[0] - 3, q[1] - 3, q[0] + 3, q[1] + 3], fill=(*colour, 220))
            body = mp(corners(run.body(row, k)[0]))
            dm.polygon([tuple(q) for q in body], fill=(*colour, 255), outline=(10, 10, 10), width=2)
            c = mp(run.truth[row, k, :2])[0]
            dm.text((c[0] + 14, c[1] - 40), f"T{k}", font=big, fill=(255, 255, 255), stroke_width=3, stroke_fill=(0, 0, 0))
            if k in order:
                dm.ellipse([c[0] + 58, c[1] - 38, c[0] + 86, c[1] - 10], fill=(*colour, 255), outline=(0, 0, 0), width=2)
                dm.text((c[0] + 72, c[1] - 24), str(order[k]), font=mid, fill=(0, 0, 0), anchor="mm")
        for k in range(run.n):
            b = run.blocked[row, k]
            if b >= 0:
                p0, p1 = mp(run.truth[row, k, :2])[0], mp(run.truth[row, b, :2])[0]
                dm.line([tuple(p0), tuple(p1)], fill=(255, 70, 70, 230), width=4)
                dm.ellipse([p0[0] - 22, p0[1] - 22, p0[0] + 22, p0[1] + 22], outline=(255, 70, 70), width=4)
        img = base.copy()
        img.paste(m, (0, 60))
        d = ImageDraw.Draw(img, "RGBA")
        # Waits: arrow to the blocker and the reason.
        y = panel_top
        d.text((MAP_W + 24, y), f"시각 {run.t[row]:6.1f} s", font=big, fill=(240, 240, 240))
        y += 44
        for k in (kw, kb):
            colour = COLOURS[k % len(COLOURS)]
            left = run.granted[row, k] - run.progress[row, k]
            b = run.blocked[row, k]
            state = f"T{b} 때문에 대기" if b >= 0 else "달리는 중"
            d.rectangle([MAP_W + 24, y + 4, MAP_W + 36, y + 26], fill=colour)
            d.text((MAP_W + 46, y), f"T{k}  순번 {order.get(k, '-')}  ·  {state}  ·  통표 {left:4.1f} m", font=small, fill=(225, 230, 236))
            y += 34
        y += 10
        for k in range(run.n):
            b = run.blocked[row, k]
            waiting = b >= 0
            if waiting:
                if k in (kw, kb) and was_waiting.get(k, -1) != b:
                    if run.why(row, k, b) == "tablet":
                        note = f"T{k}: 다음 구간이 T{b}의 통표(또는 몸체)와 겹친다 → T{b}가 지나가며 통표를 놓을 때까지 그 앞에서 멈춘다."
                    else:
                        note = (f"T{k}: 그대로 가면 T{b}(순번 앞)의 남은 경로 위에 멈추게 된다 → 정차 규정에 따라 "
                                f"끝까지 지나갈 통표를 받을 때까지 진입 전에 기다린다.")
                    note_until = run.t[row] + 1e9
            elif was_waiting.get(k, -1) >= 0 and k in (kw, kb):
                note = f"T{k}: 길이 비어 통표가 늘어났다 → 출발."
                note_until = run.t[row] + 6.0
            was_waiting[k] = b
        if note and run.t[row] <= note_until:
            line, lines = "", []
            for word in note.split(" "):
                if d.textlength(line + word + " ", font=small) > W - MAP_W - 48:
                    lines.append(line)
                    line = ""
                line += word + " "
            lines.append(line)
            d.rectangle([MAP_W + 16, y - 6, W - 16, y + 30 * len(lines) + 10], fill=(60, 40, 40, 255))
            for ln in lines:
                d.text((MAP_W + 28, y), ln, font=small, fill=(255, 235, 225))
                y += 30
        d.text((24, H - 46), f"색 띠: 통표(트럭마다 그 트럭만 들어갈 수 있는 구간, 여유 0.3 m 포함) · 점: 남은 경로 · 빨간 선: 대기 대상 · {a.fps // 10} 배속",
               font=tiny, fill=(190, 196, 204))
        if a.preview_png is not None:
            if run.t[row] < (a.preview_s if a.preview_s is not None else t0 + 11.0):
                continue
            img.save(a.preview_png)
            print(json.dumps({"preview": str(a.preview_png), "t": float(run.t[row]), "focus": [int(kw), int(kb)]}))
            return 0
        enc.stdin.write(np.asarray(img, np.uint8).tobytes())
        written += 1
    enc.stdin.close()
    enc.wait()
    print(json.dumps({"frames": written, "t0": float(t0), "t1": float(t1), "focus": [int(kw), int(kb)], "output": str(a.output)}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
