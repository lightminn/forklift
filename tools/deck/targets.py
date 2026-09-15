"""9쪽: 개발 로드맵의 초기 목표와 이번 측정값.

검출률은 높을수록, 오차는 낮을수록 좋다. 한 그림에 두 방향이 섞이면 막대
길이만 보고는 판단할 수 없으므로, 줄마다 '이상/이하'를 적고 충족 여부를
따로 표시한다.
"""
from __future__ import annotations

import pathlib
import sys

from PIL import ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

# (항목, 실측, 목표, 단위, 방향, 보조 설명, 막대 최대값)
ROWS = [
    ('양성 검출률', 100.0, 95.0, '%', '이상', '개발용 42 / 42 · 평가용 18 / 18', 100.0),
    ('위치 오차 p95', 8.17, 20.0, 'mm', '이하', '개발용 기준 · 평가용 5.98 mm', 22.0),
    ('방향 오차 p95', 0.053, 2.0, '°', '이하', '개발용 기준 · 평가용 0.026°', 2.2),
]
GREEN = (76, 205, 120)


def build():
    W, RH, TOP = 1500, 138, 96
    canvas, d, _ = R.titled((W, TOP + RH * len(ROWS) + 44),
                            '개발 로드맵의 초기 목표 대비 측정값',
                            'EPAL 6 촬영 장면 100개 · p95 는 오차의 95 %가 이 값 이하인 경계값')
    x0, x1 = 360, W - 300
    for i, (name, value, target, unit, direction, note, full) in enumerate(ROWS):
        y = TOP + i * RH
        met = value >= target if direction == '이상' else value <= target
        d.text((20, y + 22), name, font=R.F(26), fill=R.FG)
        d.text((20, y + 58), f'목표 {target:g} {unit} {direction}', font=R.F(21), fill=R.AMBER)
        d.text((20, y + 88), note, font=R.F(19), fill=R.SUB)
        d.rounded_rectangle([x0, y + 26, x1, y + 70], 6, fill=(34, 38, 46))
        w = (x1 - x0) * min(value / full, 1.0)
        d.rounded_rectangle([x0, y + 26, x0 + w, y + 70], 6, fill=GREEN)
        tx = x0 + (x1 - x0) * target / full
        d.line([(tx, y + 14), (tx, y + 82)], fill=R.AMBER, width=3)
        d.text((x1 + 18, y + 30), f'{value:g} {unit}', font=R.F(28), fill=GREEN)
        badge = '충족' if met else '미달'
        colour = GREEN if met else R.RED
        bw = d.textlength(badge, font=R.F(21))
        bx = x1 + 18
        d.rounded_rectangle([bx, y + 74, bx + bw + 22, y + 106], 6, outline=colour, width=2)
        d.text((bx + 11, y + 79), badge, font=R.F(21), fill=colour)
    d.text((20, TOP + RH * len(ROWS) + 8),
           '초록 막대는 측정값, 주황 세로선은 목표값 · 항목별 기준 방향이 다르므로 충족 여부를 함께 표시',
           font=R.F(20), fill=R.SUB)
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '17_targets.png')
    print('17_targets.png', canvas.size)


if __name__ == '__main__':
    build()
