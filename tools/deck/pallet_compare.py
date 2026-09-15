"""5쪽: 포크가 닿는 깊이 하나로 팔레트 후보를 비교한다.

바닥선을 맞추고 포크 길이에서 가로선을 한 번 그으면, 어느 규격이 포크에
비해 깊은지가 한눈에 보인다. 수치는 ADR 0002 의 후보 표에서 가져온다.
"""
from __future__ import annotations

import pathlib
import sys

from PIL import ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

FORK_MM = 420.0          # 포크 길이
PAYLOAD_KG = 10.0        # 차체 포크 적재 한도

# (이름, 폭 mm, 깊이 mm, 자중 kg 또는 None, 도달률 %, 채택 여부)
# 도달률은 ADR 0002 의 후보 표 값을 그대로 쓴다.
CANDIDATES = [
    ('T11\n국내 표준', 1100, 1100, 19.5, 38, False),
    ('PA11\nKPP 최소 품목', 1100, 800, 18.0, 53, False),
    ('1회용 수출용', 1100, 740, 6.4, 57, False),
    ('EPAL 6\n유럽 하프', 800, 600, 9.0, 70, False),
    ('T11 × 0.6\n제작', 660, 660, None, 64, True),
]

PXMM = 0.30
GAP = 58
TOP = 150
BASE_PAD = 230           # 바닥선 아래 라벨 영역


def build():
    widths = [int(w * PXMM) for _, w, _, _, _, _ in CANDIDATES]
    W = sum(widths) + GAP * (len(CANDIDATES) - 1) + 120
    tallest = max(d for _, _, d, _, _, _ in CANDIDATES)
    H = TOP + int(tallest * PXMM) + BASE_PAD
    canvas, d, _ = R.titled(
        (W, H), '포크 도달 깊이 기준 팔레트 후보 비교',
        f'가로선은 포크 길이 {FORK_MM:.0f} mm · 선 위쪽은 포크가 도달하지 못하는 구간 · 동일 축척')
    base = TOP + int(tallest * PXMM)
    fork_y = base - int(FORK_MM * PXMM)

    x = 60
    for name, w_mm, depth_mm, mass, pct, adopted in CANDIDATES:
        pw, ph = int(w_mm * PXMM), int(depth_mm * PXMM)
        top = base - ph
        reach = min(FORK_MM, depth_mm)
        ry = base - int(reach * PXMM)
        # 포크가 닿지 않는 부분
        if top < ry:
            d.rectangle([x, top, x + pw, ry], fill=(58, 62, 72))
        # 포크가 닿는 부분
        d.rectangle([x, ry, x + pw, base], fill=(126, 100, 62))
        d.rectangle([x, top, x + pw, base],
                    outline=(120, 220, 150) if adopted else (150, 140, 120),
                    width=4 if adopted else 2)
        label = f'{pct} %'
        lw = d.textlength(label, font=R.F(28))
        d.text((x + (pw - lw) / 2, (ry + base) / 2 - 20), label, font=R.F(28),
               fill=(245, 240, 230))

        y = base + 22
        for i, line in enumerate(name.split('\n')):
            d.text((x, y + i * 30), line, font=R.F(25 if i == 0 else 20),
                   fill=(140, 230, 170) if adopted else R.FG)
        y += 30 * len(name.split('\n')) + 8
        d.text((x, y), f'{w_mm} × {depth_mm} mm', font=R.F(21), fill=R.SUB)
        if mass is None:
            d.text((x, y + 30), '제작 대상 · 무게 조정 가능', font=R.F(21), fill=(140, 230, 170))
        else:
            over = mass > PAYLOAD_KG
            d.text((x, y + 30), f'자중 {mass:g} kg' + (' · 적재 한도 초과' if over else ''),
                   font=R.F(21), fill=R.RED if over else R.SUB)
        x += pw + GAP

    # 포크 길이 가로선
    d.line([(30, fork_y), (W - 30, fork_y)], fill=(120, 220, 150), width=3)
    d.text((30, fork_y - 34), f'포크 길이 {FORK_MM:.0f} mm', font=R.F(24), fill=(120, 220, 150))
    d.line([(30, base), (W - 30, base)], fill=(150, 158, 172), width=2)
    d.text((30, H - 40),
           f'하단 선이 포크 삽입면 · 갈색이 포크 도달 구간 · 차체 적재 한도 {PAYLOAD_KG:.0f} kg',
           font=R.F(21), fill=R.SUB)
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '15_pallet_choice.png')
    print('15_pallet_choice.png', canvas.size)


if __name__ == '__main__':
    build()
