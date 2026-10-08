"""Figures for the rig SLAM comparison report, as SVG, from summary.json files.

    python tools/plot_rig_slam_report.py --summary <summary.json> [--online <online.json>] \
        --series <error_series dir label=dir ...> --output <dir> [--theme static|tokens]

Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md D7. ``static`` writes
light-background SVGs with fixed colours for the repository record;
``tokens`` writes SVG fragments whose colours are CSS variables, for the web
report (light and dark from the page's tokens). One y axis per chart, log
scale where the configurations span two decades, categorical colours in a
fixed order that follows the configuration, never its rank.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

ORDER = ("lidar_st", "lidar", "vision", "fusion", "vision_front", "vision_noodom")
LABEL = {
    "lidar_st": "L-ST (기존 LiDAR)",
    "lidar": "L-RT (LiDAR)",
    "vision": "V4 (카메라 4대)",
    "fusion": "F (융합)",
    "vision_front": "V1 (전방 1대)",
    "vision_noodom": "V4 바퀴 없음",
}
SHORT = {k: v.split(" ")[0] for k, v in LABEL.items()}
# Reference palette (dataviz skill), slots 1-6 in fixed order.
STATIC = {
    # F leads (slot 1); the slot follows the configuration, never its rank.
    "fusion": "#2a78d6",
    "lidar_st": "#eb6834",
    "vision": "#1baf7a",
    "lidar": "#eda100",
    "vision_front": "#e87ba4",
    "vision_noodom": "#008300",
    "ink": "#0b0b0b",
    "ink2": "#52514e",
    "grid": "#e4e3df",
    "surface": "#fcfcfb",
    "band": "#f3d9d6",
    "band2": "#d9e3f3",
}
TOKENS = {k: f"var(--c-{k.replace('_', '-')})" for k in STATIC}
CONDITION_LABEL = {
    "nominal": "기본",
    "lidar_blackout": "LiDAR 끊김",
    "lidar_short": "LiDAR 4 m",
    "camera_blackout": "카메라 끊김",
    "wheel_slip": "바퀴 미끄럼 +5 %",
    "fast": "고속 1.0 m/s",
}


def _style_vars(svg: str) -> str:
    """fill/stroke="var(--x)" -> one style attribute: presentation attributes
    do not take CSS custom properties, style declarations do."""
    import re

    def tag(match):
        text = match.group(0)
        styles = []
        for attr in ("fill", "stroke"):
            found = re.search(rf' {attr}="(var\([^"]*\))"', text)
            if found:
                styles.append(f"{attr}:{found.group(1)}")
                text = text.replace(found.group(0), "")
        if styles:
            text = text.replace("<", "<", 1)
            head, _, rest = text.partition(" ")
            text = f'{head} style="{";".join(styles)}" {rest}'
        return text

    return re.sub(r"<[a-z]+ [^>]*>", tag, svg)


class Svg:
    def __init__(self, width, height, colours):
        self.w, self.h, self.c = width, height, colours
        self.parts = []

    def add(self, text):
        self.parts.append(text)

    def text(self, x, y, s, *, size=13, fill="ink", anchor="start", weight=400):
        s = str(s).replace("&", "&amp;").replace("<", "&lt;")
        self.add(
            f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{self.c[fill]}" '
            f'text-anchor="{anchor}" font-weight="{weight}">{s}</text>'
        )

    def render(self, title):
        body = "".join(self.parts)
        if self.c is not STATIC:
            body = _style_vars(body)
        bg = (
            f'<rect width="{self.w}" height="{self.h}" fill="{self.c["surface"]}"/>'
            if self.c is STATIC
            else ""
        )
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" '
            f'role="img" aria-label="{title}" font-family="Noto Sans KR, Noto Sans CJK KR, sans-serif">'
            f"<title>{title}</title>{bg}{body}</svg>"
        )


def log_axis(svg, *, left, right, top, bottom, lo, hi, ticks, unit="m"):
    def y(v):
        v = min(max(v, lo), hi)
        return bottom - (math.log10(v) - math.log10(lo)) / (
            math.log10(hi) - math.log10(lo)
        ) * (bottom - top)

    for t in ticks:
        yy = y(t)
        svg.add(
            f'<line x1="{left}" x2="{right}" y1="{yy:.1f}" y2="{yy:.1f}" stroke="{svg.c["grid"]}"/>'
        )
        label = f"{t * 100:g} cm" if t < 1 else f"{t:g} {unit}"
        svg.text(left - 8, yy + 4, label, size=12, fill="ink2", anchor="end")
    return y


def dot_plot(rows, configs, condition, noise, *, colours, title):
    """Per configuration: every record as a dot, the mean as a bar tick."""
    data = defaultdict(list)
    for r in rows:
        if (
            r["status"] == "ok"
            and r["condition"] == condition
            and r["noise"] == noise
            and not r["record"].endswith("seed0")
        ):
            data[r["config"]].append(r["first_pose_ate_m"])
    configs = [c for c in configs if data.get(c)]
    width, height = 760, 360
    left, right, top, bottom = 90, width - 20, 30, height - 70
    svg = Svg(width, height, colours)
    y = log_axis(
        svg,
        left=left,
        right=right,
        top=top,
        bottom=bottom,
        lo=0.005,
        hi=2.0,
        ticks=(0.01, 0.03, 0.1, 0.3, 1.0),
    )
    step = (right - left) / len(configs)
    for i, c in enumerate(configs):
        cx = left + step * (i + 0.5)
        values = data[c]
        mean = float(np.mean(values))
        for j, v in enumerate(sorted(values)):
            dx = (j - (len(values) - 1) / 2) * 9
            svg.add(
                f'<circle cx="{cx + dx:.1f}" cy="{y(v):.1f}" r="5" fill="{colours[c]}" stroke="{colours["surface"]}" stroke-width="2"/>'
            )
        svg.add(
            f'<line x1="{cx - 30:.1f}" x2="{cx + 30:.1f}" y1="{y(mean):.1f}" y2="{y(mean):.1f}" stroke="{colours["ink"]}" stroke-width="2"/>'
        )
        svg.text(cx + 34, y(mean) + 4, f"{mean * 100:.1f}", size=12, fill="ink")
        svg.text(cx, bottom + 22, SHORT[c], size=13, anchor="middle", weight=600)
        svg.text(
            cx,
            bottom + 40,
            LABEL[c].split(" ", 1)[1].strip("()"),
            size=11,
            fill="ink2",
            anchor="middle",
        )
    svg.text(
        left,
        bottom + 62,
        "점 = 평가 seed 하나, 가로선 = 평균(cm). 로그 축.",
        size=11,
        fill="ink2",
    )
    return svg.render(title)


def condition_chart(rows, configs, conditions, *, colours, title):
    """Mean first-pose ATE per condition (groups) and configuration (dots)."""
    mean = {}
    for cond in conditions:
        for c in configs:
            values = [
                r["first_pose_ate_m"]
                for r in rows
                if r["status"] == "ok"
                and r["condition"] == cond
                and r["config"] == c
                and r["noise"] != "none"
                and not r["record"].endswith("seed0")
            ]
            if values:
                mean[(cond, c)] = (
                    float(np.mean(values)),
                    float(np.min(values)),
                    float(np.max(values)),
                )
    width, height = 760, 380
    left, right, top, bottom = 90, width - 20, 40, height - 60
    svg = Svg(width, height, colours)
    y = log_axis(
        svg,
        left=left,
        right=right,
        top=top,
        bottom=bottom,
        lo=0.005,
        hi=10.0,
        ticks=(0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0),
    )
    group = (right - left) / len(conditions)
    for i, cond in enumerate(conditions):
        gx = left + group * i
        svg.text(
            gx + group / 2,
            bottom + 22,
            CONDITION_LABEL.get(cond, cond),
            size=13,
            anchor="middle",
            weight=600,
        )
        inner = group / (len(configs) + 1)
        for j, c in enumerate(configs):
            if (cond, c) not in mean:
                continue
            m, lo, hi = mean[(cond, c)]
            x = gx + inner * (j + 1)
            svg.add(
                f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{y(lo):.1f}" y2="{y(hi):.1f}" stroke="{colours[c]}" stroke-width="2"/>'
            )
            svg.add(
                f'<circle cx="{x:.1f}" cy="{y(m):.1f}" r="5.5" fill="{colours[c]}" stroke="{colours["surface"]}" stroke-width="2"/>'
            )
    for j, c in enumerate(configs):
        lx = left + j * 150
        svg.add(f'<circle cx="{lx + 6}" cy="16" r="5.5" fill="{colours[c]}"/>')
        svg.text(lx + 16, 20, LABEL[c], size=12)
    svg.text(
        left,
        bottom + 46,
        "점 = 평가 seed 평균, 세로선 = 최소–최대. 로그 축.",
        size=11,
        fill="ink2",
    )
    return svg.render(title)


def error_timeline(series: dict, *, colours, title, windows=(), window_label=""):
    """Start-aligned error over time for a few configurations of one record."""
    width, height = 760, 320
    left, right, top, bottom = 90, width - 20, 40, height - 50
    svg = Svg(width, height, colours)
    y = log_axis(
        svg,
        left=left,
        right=right,
        top=top,
        bottom=bottom,
        lo=0.002,
        hi=10.0,
        ticks=(0.01, 0.03, 0.1, 0.3, 1.0, 3.0),
    )
    end = max(float(s["stamps_s"][-1]) for s in series.values())

    def x(t):
        return left + t / end * (right - left)

    for a, b in windows:
        svg.add(
            f'<rect x="{x(a):.1f}" y="{top}" width="{x(b) - x(a):.1f}" height="{bottom - top}" fill="{colours["band"]}"/>'
        )
    if windows:
        svg.text(x(windows[0][0]) + 4, top + 14, window_label, size=11, fill="ink2")
    for t in range(0, int(end) + 1, 60):
        svg.text(x(t), bottom + 18, f"{t} s", size=11, fill="ink2", anchor="middle")
    for c, s in series.items():
        t, e = s["stamps_s"], s["error_m"]
        keep = np.arange(0, len(t), 4)
        pts = " ".join(
            f"{x(t[i]):.1f},{y(max(e[i], 0.002)):.1f}"
            for i in keep
            if np.isfinite(e[i])
        )
        svg.add(
            f'<polyline points="{pts}" fill="none" stroke="{colours[c]}" stroke-width="2" stroke-linejoin="round"/>'
        )
    for j, c in enumerate(series):
        lx = left + j * 170
        svg.add(
            f'<line x1="{lx}" x2="{lx + 18}" y1="16" y2="16" stroke="{colours[c]}" stroke-width="3"/>'
        )
        svg.text(lx + 24, 20, LABEL[c], size=12)
    return svg.render(title)


def online_chart(online: list, configs, *, colours, title):
    """Closed-loop raw (unaligned) position RMSE of the pose control used."""
    width, height = 760, 330
    left, right, top, bottom = 90, width - 20, 30, height - 60
    svg = Svg(width, height, colours)
    y = log_axis(
        svg,
        left=left,
        right=right,
        top=top,
        bottom=bottom,
        lo=0.005,
        hi=2.0,
        ticks=(0.01, 0.03, 0.1, 0.3, 1.0),
    )
    configs = [c for c in configs if any(o["config"] == c for o in online)]
    step = (right - left) / max(1, len(configs))
    for i, c in enumerate(configs):
        cx = left + step * (i + 0.5)
        runs = [o for o in online if o["config"] == c]
        for j, o in enumerate(runs):
            dx = (j - (len(runs) - 1) / 2) * 10
            v = o.get("raw_rmse_m")
            if v is None:
                svg.text(cx + dx, bottom - 6, "×", size=14, fill="ink", anchor="middle")
                continue
            ring = colours["surface"] if o.get("completed") else colours["ink"]
            svg.add(
                f'<circle cx="{cx + dx:.1f}" cy="{y(v):.1f}" r="5" fill="{colours[c]}" stroke="{ring}" stroke-width="2"/>'
            )
        values = [o["raw_rmse_m"] for o in runs if o.get("raw_rmse_m") is not None]
        if values:
            m = float(np.mean(values))
            svg.add(
                f'<line x1="{cx - 32:.1f}" x2="{cx + 32:.1f}" y1="{y(m):.1f}" y2="{y(m):.1f}" stroke="{colours["ink"]}" stroke-width="2"/>'
            )
            svg.text(cx + 36, y(m) + 4, f"{m * 100:.1f}", size=12)
        svg.text(cx, bottom + 22, LABEL[c], size=13, anchor="middle", weight=600)
    svg.text(
        left,
        bottom + 46,
        "점 = 평가 주행 하나(제어에 쓴 추정의 정렬 없는 RMSE), 검은 테두리 = 미완주, 가로선 = 평균(cm). 로그 축.",
        size=11,
        fill="ink2",
    )
    return svg.render(title)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--online", type=Path)
    parser.add_argument(
        "--timeline", action="append", default=[], help="name:config=dir,... [;windows]"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--theme", choices=("static", "tokens"), default="static")
    args = parser.parse_args(argv)
    colours = STATIC if args.theme == "static" else TOKENS
    rows = json.loads(args.summary.read_text())["rows"]
    args.output.mkdir(parents=True, exist_ok=True)
    nominal = [r for r in rows if r.get("scenario", "nominal") == "nominal"]
    out = {
        "nominal.svg": dot_plot(
            nominal,
            ORDER,
            "nominal",
            "noisy",
            colours=colours,
            title="기본 조건 위치 오차",
        ),
    }
    fast = [r for r in rows if r.get("scenario") == "fast"]
    if fast:
        out["fast.svg"] = dot_plot(
            fast,
            ORDER,
            "nominal",
            "noisy",
            colours=colours,
            title="고속 조건 위치 오차",
        )
    rows = nominal
    conditions = [
        c
        for c in (
            "nominal",
            "lidar_blackout",
            "lidar_short",
            "camera_blackout",
            "wheel_slip",
            "fast",
        )
        if any(r["condition"] == c for r in rows)
    ]
    out["conditions.svg"] = condition_chart(
        rows,
        ("lidar_st", "lidar", "vision", "fusion"),
        conditions,
        colours=colours,
        title="조건별 위치 오차",
    )
    for spec in args.timeline:
        name, _, rest = spec.partition(":")
        parts, _, window_text = rest.partition(";")
        series = {}
        for item in parts.split(","):
            config, _, directory = item.partition("=")
            series[config] = np.load(Path(directory) / "error_series.npz")
        windows = [tuple(map(float, w.split("-"))) for w in window_text.split("|") if w]
        out[f"{name}.svg"] = error_timeline(
            series,
            colours=colours,
            title=name,
            windows=windows,
            window_label="LiDAR 끊김 구간" if windows else "",
        )
    if args.online:
        # Evaluation runs of the nominal condition only (plan D6).
        online = [
            o
            for o in json.loads(args.online.read_text())
            if not o["development"]
            and o["condition"] == "nominal"
            and o["status"] == "ok"
        ]
        out["online.svg"] = online_chart(
            online,
            ("lidar_st", "vision", "fusion"),
            colours=colours,
            title="폐루프 주행 위치 오차",
        )
    for name, text in out.items():
        (args.output / name).write_text(text + "\n")
        print("wrote", args.output / name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
