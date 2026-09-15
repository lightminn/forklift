"""10쪽: 카메라 설치 높이에 따른 근거리 인식 한계.

거리는 모두 **카메라에서 팔레트 전면까지**로 통일한다. 포크 끝은 모델의
left_fork_tip site (x = 0.95 m) 를 쓰고, 카메라 x 는 측정에 쓴 0.75 m 다.
두 좌표를 섞으면 잔여 거리가 270 mm 씩 어긋난다 — 실제로 한 번 어긋났다.
"""
from __future__ import annotations

import pathlib
import sys

from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

CAM_X = 0.75            # 측정에 쓴 카메라 x (배치 좌표계)
FORK_TIP_X = 0.95       # forklift.xml 의 left_fork_tip site
HALF_DEPTH = 0.30       # EPAL 6 절반 깊이

# (높이 m, 카메라→전면 최근접 m, 검출 자세 수, 이름)
MOUNTS = [
    (0.27, 1.05, 21, '낮게 설치'),
    (0.50, 1.25, 18, '현재 기준선'),
    (0.90, 1.75, 10, '마스트 상단 · 과제 제시안'),
]
SWEEP = 46              # 정면 스윕에서 확인한 자세 수

PW, CROP, BAR = 590, (95, 430), 132


def build():
    panels = []
    for z, limit, points, label in MOUNTS:
        front = CAM_X + limit
        gap_mm = (front - FORK_TIP_X) * 1000
        xml = R.scene_xml(R.EPAL6, (front + HALF_DEPTH, 0, 0), camera_z=z,
                          wedge_len=limit + 0.9, wedge_rgba='1 0.45 0.15 0.9')
        img = R.render(xml, azimuth=90, elevation=-6, distance=5.0,
                       lookat=(1.35, 0, 0.45), width=PW, height=470)
        img = img.crop((0, CROP[0], PW, CROP[1]))
        accent = R.RED if z >= 0.9 else (R.AMBER if z >= 0.5 else R.GREEN)

        bar = Image.new('RGB', (PW, BAR), R.BG)
        d = ImageDraw.Draw(bar)
        d.rectangle([0, 0, PW, 5], fill=accent)
        d.text((14, 16), f'높이 {z:.2f} m', font=R.F(29), fill=R.FG)
        d.text((14, 54), label, font=R.F(21), fill=accent)
        d.text((14, 86), f'검출 가능한 최근접 거리 {limit:.2f} m', font=R.F(22), fill=(192, 198, 210))
        cell = Image.new('RGB', (PW, img.height + BAR), R.BG)
        cell.paste(img, (0, 0))
        cell.paste(bar, (0, img.height))
        panels.append((cell, gap_mm, points, accent))

    GAP, TOP = 8, 96
    W = PW * len(panels) + GAP * (len(panels) - 1)
    body_h = panels[0][0].height
    canvas, d, _ = R.titled((W, TOP + body_h + 108), '카메라 설치 높이별 근거리 인식 한계',
                            '거리는 카메라에서 팔레트 전면까지 · 주황은 카메라의 수직 관측 범위')
    for i, (cell, gap_mm, points, accent) in enumerate(panels):
        x = i * (PW + GAP)
        canvas.paste(cell, (x, TOP))
        y = TOP + body_h + 12
        d.text((x + 14, y), f'검출 중단 시 포크 끝에서 {gap_mm:.0f} mm 잔여',
               font=R.F(23), fill=accent)
        d.text((x + 14, y + 36), f'정면 {SWEEP}자세 중 {points}자세 검출',
               font=R.F(21), fill=R.SUB)
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '13_camera_mount.png')
    print('13_camera_mount.png', canvas.size,
          '잔여 mm:', [f'{g:.0f}' for _, g, _, _ in panels])


if __name__ == '__main__':
    build()
