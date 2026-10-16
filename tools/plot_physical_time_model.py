"""Draw the forklift's physical time model (plan 2026-10-12) as figures.

    python tools/plot_physical_time_model.py --output docs/design/assets/2026-10-12-physical-time-model

Needs matplotlib (not a project dependency). Every number comes from
forklift_core.planning.fleet_physics on the measured chassis URDF; the
chassis mass split and centre of gravity there are estimates, and so is the
load. Four figures, each as .svg and .png:

- ``blocks``: the model as blocks -- mass properties, physical limits (blue),
  control and operating caps (orange), the speed profile and the windows.
- ``force_balance``: the truck braking with a pallet lifted, its centre of
  gravity, axle loads and the tipping condition, to scale.
- ``limits_vs_load``: the centre of gravity and each acceleration limit
  against load mass, for the fork down and lifted.
- ``profile``: speed and time along a sample path, empty and loaded, and the
  window a crossing gets from each.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from forklift_core.planning.fleet_physics import (  # noqa: E402
    Chassis,
    G,
    Load,
    limits,
    speed_profile,
)

# matplotlib sets a whole line that has $...$ in mathtext, which has no Korean;
# ``rich`` lays such a line out as text and math pieces side by side.
plt.rcParams.update({
    "font.family": ["DejaVu Serif", "NanumMyeongjo"], "font.size": 12,
    "mathtext.fontset": "stix", "svg.fonttype": "none", "axes.unicode_minus": False,
})


def rich(ax, x, y, s, size=12, color=None, ha="left", va="baseline", linespacing=1.5, weight="normal", fig_coords=False):
    """Text with Korean and $math$ on one line: each line split at the $
    pieces, laid out left to right in figure coordinates."""
    color = color or INK
    fig = ax.figure
    renderer = fig.canvas.get_renderer()
    to_fig = fig.transFigure.inverted()
    origin = fig.transFigure.transform((x, y)) if fig_coords else ax.transData.transform((x, y))
    step = size * linespacing / 72 * fig.dpi
    lines = s.split("\n")
    if va == "center":
        first = origin[1] + (len(lines) - 1) * step / 2 - size * 0.35 / 72 * fig.dpi
    elif va == "top":
        first = origin[1] - size / 72 * fig.dpi
    else:
        first = origin[1]
    for i, line in enumerate(lines):
        pieces = [p for p in re.split(r"(\$[^$]*\$)", line) if p]
        made = []
        for piece in pieces:
            t = fig.text(0, 0, piece, fontsize=size, color=color, weight=weight, va="baseline", ha="left")
            made.append((t, t.get_window_extent(renderer).width))
        width = sum(w for _, w in made)
        left = origin[0] - (width / 2 if ha == "center" else width if ha == "right" else 0)
        base = first - i * step
        for t, w in made:
            t.set_position(to_fig.transform((left, base)))
            left += w
INK, PHYS, CTRL, MUTED, FILL = "#1f2d3d", "#274c67", "#b0651c", "#5c6670", "#eef2f6"
URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"
SHARE, CAP, LIFT = 0.5, 0.30, 0.20  # tipping share used, tracker cap (m/s^2), transport lift (m)


def page(title, subtitle, size=(16, 9)):
    fig = plt.figure(figsize=size)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis("off")
    rich(ax, 0.45, 8.42, title, size=21, weight="bold")
    rich(ax, 0.45, 7.98, subtitle, size=11.5, color=MUTED)
    return fig, ax


def block(ax, x, y, w, h, text, color=PHYS, size=13, fill="white"):
    ax.add_patch(Rectangle((x, y), w, h, fill=True, facecolor=fill, edgecolor=color, lw=1.6))
    rich(ax, x + w / 2, y + h / 2, text, size=size, color=color, ha="center", va="center", linespacing=1.4)


def wire(ax, points, color=INK):
    pts = np.asarray(points, float)
    for a, b in zip(pts[:-2], pts[1:-1], strict=True):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=1.4)
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle="-|>", mutation_scale=13, color=color, lw=1.4))


def heading(ax, x, y, text, color=PHYS):
    rich(ax, x, y, text, size=13.5, weight="bold", color=color)


def save(fig, out, name):
    for ext in ("svg", "png"):
        fig.savefig(out / f"{name}.{ext}", dpi=150)
    plt.close(fig)


def blocks_page(out, chassis):
    lim = limits(chassis, Load(10.0), LIFT, SHARE, CAP)
    empty = limits(chassis, None, 0.0, SHARE, CAP)
    fig, ax = page("지게차 시간 모델  |  짐이 무게중심을 옮기고, 한계가 시간 창을 정한다",
                   "준정적 평면 모델(차체 좌표: x 앞, z 위). 파랑 = 물리 한계, 주황 = 제어·운영 설정. 오른쪽 숫자는 10 kg 팔레트를 0.2 m 들었을 때 [m/s²].")
    heading(ax, 0.5, 7.35, "질량과 무게중심")
    rows = [("차체 링크", rf"$\Sigma m_i = {chassis.mass_kg - chassis.carriage.kg:.1f}$ kg"),
            ("캐리지", rf"$m_c = {chassis.carriage.kg:.1f}$ kg,  $z_c + z_{{lift}}$"),
            ("짐(팔레트)", r"$m_L$,  $(x_L,\ z_L(z_{lift}))$")]
    for k, (name, value) in enumerate(rows):
        y = 6.35 - 1.05 * k
        block(ax, 0.5, y, 2.6, 0.8, f"{name}\n{value}", size=11.5)
        if k == 1:
            wire(ax, [(3.1, y + 0.4), (3.8, y + 0.4)])
        else:
            wire(ax, [(3.1, y + 0.4), (3.5, y + 0.4), (3.5, 5.72), (3.8, 5.72)])
    block(ax, 3.8, 4.85, 1.7, 1.3, r"$\Sigma\, m_i\,(x_i,\ z_i)\ /\ \Sigma\, m_i$" + "\n" + r"$\rightarrow (m,\ x_G,\ z_G)$", size=12, fill=FILL)
    rich(ax, 4.65, 4.5, rf"$m={lim.mass_kg:.0f}$ kg,  $x_G={lim.cog_x_m:+.3f}$ m,  $z_G={lim.cog_z_m:.3f}$ m", size=10.5, color=PHYS, ha="center")
    heading(ax, 6.1, 7.35, "물리 한계 (기울어짐 · 미끄럼 · 구동)")
    phys = [
        ("앞 넘어짐 (앞으로 가다 제동)", r"$a^- \leq g\,(x_f - x_G)/z_G$", lim.tip_backward),
        ("뒤 넘어짐 (앞으로 가속)", r"$a^+ \leq g\,(x_G - x_r)/z_G$", lim.tip_forward),
        ("옆 넘어짐 (회전)", r"$a_y \leq g\,(b/2)/z_G$", lim.tip_lateral),
        ("미끄럼 (네 바퀴 구동)", r"$|a| \leq \mu\, g$", lim.traction),
        ("바퀴 토크", r"$|a| \leq n\,\tau/(r\,m)$", lim.drive),
    ]
    for k, (name, law, value) in enumerate(phys):
        y = 6.45 - 0.98 * k
        block(ax, 6.1, y, 3.2, 0.82, f"{name}\n{law}", size=11)
        rich(ax, 9.4, y + 0.33, f"{value:.2f}", size=10.5, color=PHYS)
        wire(ax, [(5.5, 5.5), (5.8, 5.5), (5.8, y + 0.41), (6.1, y + 0.41)])
    heading(ax, 0.5, 1.45, "제어 · 운영 설정", CTRL)
    for k, text in enumerate([rf"기울어짐 한계를 쓰는 몫  $\eta={SHARE}$", rf"추종기 가속 상한  $a_{{cap}}={CAP}$ m/s²", r"단계 순항 속도  $v_{stage}$"]):
        block(ax, 0.5 + 3.55 * k, 0.45, 3.35, 0.72, text, color=CTRL, size=11)
    heading(ax, 10.3, 7.35, "허용 운동")
    block(ax, 10.3, 5.55, 1.1, 1.6, "min", size=16, fill=FILL)
    for k in range(5):
        wire(ax, [(9.95, 6.86 - 0.98 * k), (10.1, 6.86 - 0.98 * k), (10.1, 6.35), (10.3, 6.35)])
    wire(ax, [(5.2, 1.17), (5.2, 1.6), (10.85, 1.6), (10.85, 5.55)], color=CTRL)
    outs = [(r"$a_{up}(\pm x),\ \ a_{brake}(\pm x)$", f"짐 10 kg: 앞 {lim.forward:.2f} · 뒤 {lim.backward:.2f} m/s²"),
            (r"$v_{cap}(s) = \min(v_{stage},\ \omega_{max} r,\ \sqrt{a_y/\kappa})$", f"빈 차: 앞 {empty.forward:.2f} · 뒤 {empty.backward:.2f} m/s²")]
    for k, (a, b) in enumerate(outs):
        y = 6.6 - 1.05 * k
        block(ax, 11.75, y, 3.95, 0.82, f"{a}\n{b}", size=11)
        wire(ax, [(11.4, 6.35), (11.55, 6.35), (11.55, y + 0.41), (11.75, y + 0.41)])
    heading(ax, 10.3, 4.6, "시간 창")
    block(ax, 10.3, 3.25, 2.45, 1.05, "속도 단면\n가속으로 쌓고 제동으로 깎기", size=11, fill=FILL)
    block(ax, 13.1, 3.25, 2.6, 1.05, r"$t(s)=\int ds\,/\,v(s)$" + "\n교차 들어감 · 나옴 시각", size=11, fill=FILL)
    wire(ax, [(13.7, 5.53), (13.7, 4.95), (11.5, 4.95), (11.5, 4.3)])
    wire(ax, [(12.75, 3.78), (13.1, 3.78)])
    block(ax, 10.3, 2.0, 5.4, 0.9, r"창 $[t_{in},\ t_{out}]$,  정지 비용 $v/(2a_{brake}) + v/(2a_{up})$" + "\n→ 교차 순서 최적화 (가중 지연 합)", size=11.5)
    wire(ax, [(14.4, 3.25), (14.4, 2.9)])
    rich(ax, 0.5, 3.75, "읽는 법", size=12, weight="bold")
    rich(ax, 0.5, 3.3, "짐은 앞차축 너머 포크 위에 있다. 무거울수록 무게중심이 앞으로,\n"
         r"들수록 위로 간다 → $x_f - x_G$ 가 줄고 $z_G$ 가 커져" + "\n앞으로 넘어지지 않고 낼 수 있는 제동이 줄어든다.", size=11)
    save(fig, out, "blocks")


def force_page(out, chassis):
    load = Load(10.0)
    lim = limits(chassis, load, LIFT, SHARE, CAP)
    fig, ax = page("제동 중 힘의 균형  |  10 kg 팔레트를 0.2 m 들고 앞으로 가다 멈출 때",
                   r"축척대로 (차체 좌표, m). 관성력 $m a$ 가 무게중심 높이 $z_G$ 에서 앞으로 밀어 앞바퀴로 하중을 옮긴다. 뒷바퀴 하중 $N_r$ 이 0 이 되면 앞으로 넘어진다." + "\n" + rf"힘은 몫 $\eta = {SHARE}$ 만큼의 제동 $a = \eta\,g\,(x_f - x_G)/z_G$ 일 때. 화살표 길이는 힘에 비례 (1 m = 900 N).")
    sub = fig.add_axes([0.03, 0.04, 0.62, 0.78])
    sub.set_aspect("equal")
    sub.set_xlim(-0.8, 1.45)
    sub.set_ylim(-0.42, 1.0)
    sub.axis("off")
    sub.plot([-0.75, 1.45], [0, 0], color=MUTED, lw=1.2)
    r = chassis.wheel_radius_m
    sub.add_patch(Polygon([(-0.51, 0.08), (0.50, 0.08), (0.50, 0.48), (0.25, 0.62), (-0.51, 0.62)], closed=True, facecolor=FILL, edgecolor=PHYS, lw=1.5))
    sub.add_patch(Rectangle((0.54, 0.05), 0.04, 0.67, facecolor="#dde3ea", edgecolor=PHYS, lw=1.2))
    fork_z = chassis.fork_height_m + LIFT
    sub.add_patch(Rectangle((0.59, fork_z - 0.0125), 0.36, 0.025, facecolor=PHYS, edgecolor=PHYS))
    m = load.mass(chassis, LIFT)
    base = fork_z + 0.0125 - load.opening_top_m
    sub.add_patch(Rectangle((m.x_m - load.depth_m / 2, base), load.depth_m, 0.144, facecolor="#e8d3b0", edgecolor="#8a6a3a", lw=1.3))
    for x in (chassis.rear_axle_x_m, chassis.front_axle_x_m):
        sub.add_patch(Circle((x, r), r, facecolor="white", edgecolor=PHYS, lw=1.5))
    parts = [*chassis.parts, chassis.carriage]
    mc = sum(p.kg for p in parts)
    xc = sum(p.kg * p.x_m for p in parts) / mc
    zc = sum(p.kg * (p.z_m + (LIFT if p is chassis.carriage else 0)) for p in parts) / mc
    for (x, z), label, size in (((xc, zc), r"$G_c$", 70), ((m.x_m, m.z_m), r"$G_L$", 70), ((lim.cog_x_m, lim.cog_z_m), r"$G$", 160)):
        sub.scatter([x], [z], s=size, color=INK if label == r"$G$" else MUTED, zorder=5, edgecolor="white")
        sub.text(x + (-0.07 if label == r"$G$" else 0.03), z + 0.035, label, fontsize=14, color=INK)

    def arrow(p, q, color, text, dx=0.02, dy=0.0):
        sub.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=14, color=color, lw=2))
        sub.text(q[0] + dx, q[1] + dy, text, fontsize=13, color=color)

    scale = 1 / 900  # m per N
    arrow((xc, zc), (xc, zc - mc * G * scale), PHYS, rf"$m_c\,g = {mc * G:.0f}$ N", dy=-0.03)
    arrow((m.x_m, m.z_m), (m.x_m, m.z_m - load.kg * G * scale), PHYS, rf"$m_L\,g = {load.kg * G:.0f}$ N", dy=-0.03)
    a = SHARE * lim.tip_backward
    arrow((lim.cog_x_m, lim.cog_z_m), (lim.cog_x_m + lim.mass_kg * a * scale, lim.cog_z_m), CTRL, rf"$m\,a = {lim.mass_kg * a:.0f}$ N", dx=-0.02, dy=0.1)
    L = chassis.front_axle_x_m - chassis.rear_axle_x_m
    n_f = lim.mass_kg * (G * (lim.cog_x_m - chassis.rear_axle_x_m) + a * lim.cog_z_m) / L
    n_r = lim.mass_kg * G - n_f
    arrow((chassis.front_axle_x_m, -n_f * scale), (chassis.front_axle_x_m, 0.0), PHYS, rf"$N_f = {n_f:.0f}$ N", dx=0.05, dy=-0.2)
    arrow((chassis.rear_axle_x_m, -max(n_r, 1.0) * scale), (chassis.rear_axle_x_m, 0.0), PHYS, rf"$N_r = {n_r:.0f}$ N", dx=-0.32, dy=-0.12)
    sub.annotate("", (chassis.front_axle_x_m, 0.9), (lim.cog_x_m, 0.9), arrowprops=dict(arrowstyle="<->", color=INK))
    sub.text((chassis.front_axle_x_m + lim.cog_x_m) / 2, 0.94, rf"$x_f - x_G = {lim.static_margin_m * 100:.1f}$ cm", ha="center", fontsize=12.5)
    sub.annotate("", (-0.72, 0), (-0.72, lim.cog_z_m), arrowprops=dict(arrowstyle="<->", color=INK))
    sub.text(-0.79, lim.cog_z_m + 0.04, rf"$z_G = {lim.cog_z_m:.3f}$ m", fontsize=12.5)
    sub.plot([-0.72, lim.cog_x_m], [lim.cog_z_m] * 2, color=MUTED, lw=0.8, ls="--")
    for x in (lim.cog_x_m, chassis.front_axle_x_m):
        sub.plot([x, x], [0.0, 0.88], color=MUTED, lw=0.8, ls="--")
    rich(sub, chassis.rear_axle_x_m + 0.04, -0.07, "뒤차축", size=10.5, color=MUTED)
    rich(sub, chassis.front_axle_x_m + 0.04, -0.07, "앞차축", size=10.5, color=MUTED)
    heading(ax, 10.7, 7.3, "앞바퀴 접점에 대한 모멘트")
    lines = [
        (r"$N_r\,L = m\,[\,g\,(x_f - x_G) - a\,z_G\,]$", INK, 14.5),
        (r"$N_r \geq 0 \ \Rightarrow\ a \leq g\,(x_f - x_G)\,/\,z_G$", INK, 14.5),
        (rf"$= 9.81 \times {lim.static_margin_m:.3f}\,/\,{lim.cog_z_m:.3f} = {lim.tip_backward:.2f}$ m/s²", INK, 13),
        (rf"쓰는 몫  $\eta = {SHARE}$  →  {SHARE * lim.tip_backward:.2f} m/s²", CTRL, 13),
        (rf"추종기 상한 {CAP} 과 비교  →  $a_{{brake}} = {lim.backward:.2f}$ m/s²", CTRL, 13),
    ]
    for k, (text, color, size) in enumerate(lines):
        rich(ax, 10.7, 6.6 - 0.62 * k, text, size=size, color=color)
    heading(ax, 10.7, 3.1, "무엇이 바뀌나")
    rich(ax, 10.7, 2.55, r"짐이 무거울수록 $x_G$ 가 앞차축에 다가가고," + "\n" + r"들수록 $z_G$ 가 높아진다. 둘 다 앞 넘어짐 한계를" + "\n" + r"줄인다 — 제동 거리 $v^2/2a$ 와 멈췄다 다시 떠나는" + "\n" + r"시간 $v/2a$ 가 늘어난다.", size=12.5)
    rich(ax, 10.7, 0.55, r"$G_c$ 차체+캐리지,  $G_L$ 짐,  $G$ 합친 무게중심.", size=10.5, color=MUTED)
    save(fig, out, "force_balance")


def limits_page(out, chassis):
    kg = np.linspace(0, 14, 281)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(16, 7.4))
    fig.subplots_adjust(left=0.06, right=0.97, top=0.8, bottom=0.12, wspace=0.22)
    rich(a1, 0.06, 0.93, "하중에 따른 무게중심과 가감속 한계  (측정 모델, 질량 분배는 추정)", size=19, weight="bold", fig_coords=True)
    rich(a1, 0.06, 0.875, rf"실선 = 포크 0.2 m (운반 높이), 점선 = 포크 0 m.  기울어짐 한계는 몫 $\eta = {SHARE}$ 를 곱한 값.  회색 띠 = 평가에 쓸 하중 범위 (추정).", size=11.5, color=MUTED, fig_coords=True)
    for lift, ls in ((LIFT, "-"), (0.0, "--")):
        rows = [limits(chassis, Load(k) if k > 0 else None, lift, SHARE) for k in kg]
        a1.plot(kg, [r.cog_x_m for r in rows], ls, color=PHYS, lw=2, label=f"x_G (포크 {lift} m)")
        a1.plot(kg, [r.cog_z_m for r in rows], ls, color=CTRL, lw=2, label=f"z_G (포크 {lift} m)")
        a2.plot(kg, [max(SHARE * r.tip_backward, 0) for r in rows], ls, color=PHYS, lw=2.2, label=f"앞 넘어짐 η·g(x_f−x_G)/z_G (포크 {lift} m)")
        if lift == LIFT:
            a2.plot(kg, [SHARE * r.tip_forward for r in rows], ls, color="#4f7d4f", lw=1.6, label="뒤 넘어짐 η·g(x_G−x_r)/z_G")
            a2.plot(kg, [r.drive for r in rows], ls, color=MUTED, lw=1.6, label="바퀴 토크 n·τ/(r·m)")
            a2.plot(kg, [r.traction for r in rows], ":", color=MUTED, lw=1.4, label="미끄럼 μ·g")
    a1.axhline(chassis.front_axle_x_m, color=INK, lw=1, ls=":")
    a1.text(0.2, chassis.front_axle_x_m + 0.008, "앞차축 x_f", fontsize=11)
    a2.axhline(CAP, color=CTRL, lw=2, ls="-.")
    a2.text(0.2, CAP + 0.07, f"추종기 가속 상한 {CAP} m/s² (제어 설정)", color=CTRL, fontsize=11)
    tip_kg = kg[np.argmax([limits(chassis, Load(k) if k > 0 else None, LIFT).tip_backward <= 0 for k in kg])]
    for a in (a1, a2):
        a.axvspan(5, 13, color="#d9dee4", alpha=0.45, lw=0)
        a.axvline(tip_kg, color="#a33", lw=1.2)
        a.set_xlabel("짐 질량 m_L [kg]")
        a.grid(alpha=0.25)
    a2.text(tip_kg - 0.1, 3.6, f"서 있어도 앞으로\n넘어짐 ≈ {tip_kg:.1f} kg", ha="right", color="#a33", fontsize=11)
    a1.set_ylabel("[m]")
    a1.set_title("무게중심 위치", fontsize=13)
    a2.set_ylabel("[m/s²]")
    a2.set_ylim(0, 4.3)
    a2.set_title("가감속 한계", fontsize=13)
    a1.legend(fontsize=10, loc="upper left")
    a2.legend(fontsize=9.5, loc="upper right")
    save(fig, out, "limits_vs_load")


def profile_page(out, chassis):
    # A sample route: 6 m out, a quarter turn on the 2.73 m minimum radius, 5 m on.
    straight = np.column_stack((np.linspace(0, 6, 121), np.zeros(121), np.zeros(121)))
    radius = 2.728
    th = np.linspace(0, np.pi / 2, 90)[1:]
    arc = np.column_stack((6 + radius * np.sin(th), radius * (1 - np.cos(th)), th))
    on = np.column_stack((np.full(100, 6 + radius), radius + np.linspace(0, 5, 101)[1:], np.full(100, np.pi / 2)))
    path = np.vstack((straight, arc, on))
    cruise = 0.30  # loaded transport speed (isaac_transport_measured.yaml)
    cases = [("빈 차 (접근 0.60 m/s)", None, 0.60, MUTED), ("10 kg, 포크 0.2 m", Load(10.0), cruise, PHYS), ("12.8 kg, 포크 0.2 m", Load(12.8), cruise, CTRL)]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(16, 9), sharex=True)
    fig.subplots_adjust(left=0.07, right=0.97, top=0.85, bottom=0.08, hspace=0.12)
    rich(a1, 0.07, 0.94, "같은 길, 다른 짐  —  속도 단면과 교차 창", size=19, weight="bold", fig_coords=True)
    rich(a1, 0.07, 0.895, rf"직진 6 m → 최소 반경 {radius} m 의 90° 회전 → 직진 5 m.  추종기 상한 {CAP} m/s²,  기울어짐 몫 $\eta = {SHARE}$.  회색 = 예시 교차 구간 (s 12–15 m).", size=11.5, color=MUTED, fig_coords=True)
    s_in, s_out = 12.0, 15.0
    for k, (name, load, v_cruise, color) in enumerate(cases):
        lift = LIFT if load is not None else 0.0
        lim = limits(chassis, load, lift, SHARE, CAP)
        s, v, t = speed_profile(path, v_cruise, lim)
        a1.plot(s, v, color=color, lw=2.2, label=f"{name}: 가속 {lim.forward:.2f}, 제동 {lim.backward:.2f} m/s²")
        a2.plot(s, t, color=color, lw=2.2)
        t_in, t_out = np.interp(s_in, s, t), np.interp(s_out, s, t)
        a2.plot([s_in, s_out], [t_in, t_out], "o", color=color)
        a2.text(s_out + 0.15, t_out + 3 * (k - 1), f"창 [{t_in:.1f}, {t_out:.1f}] s", color=color, fontsize=11, va="center")
    for a in (a1, a2):
        a.axvspan(s_in, s_out, color="#d9dee4", alpha=0.6, lw=0)
        a.grid(alpha=0.25)
    a1.set_ylabel("속도 v(s) [m/s]")
    a2.set_ylabel("도착 시각 t(s) [s]")
    a2.set_xlabel("경로 길이 s [m]")
    a1.legend(fontsize=11, loc="upper right")
    save(fig, out, "profile")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    chassis = Chassis.from_urdf(URDF)
    blocks_page(args.output, chassis)
    force_page(args.output, chassis)
    limits_page(args.output, chassis)
    profile_page(args.output, chassis)
    print(f"wrote 4 figures to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
