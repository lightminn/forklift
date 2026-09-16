"""3쪽: 2D LiDAR 의 관측 평면과 팔레트 높이의 관계."""
from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

import mujoco  # noqa: E402

SENSOR_XY = (0.60, 0.0)          # 차체 앞쪽
SENSOR_Z = 0.50                  # 설정한 가정 높이 (실제 장착은 미확정)
PALLET = (2.60, 0.0, 0.0)
SAMPLES, RANGE_M = 720, 8.0

WORLD = f"""<mujoco model="lidar_scene">
  <option><flag contact="disable"/></option>
  <asset>
    <model name="pal" file="{R.EPAL6}"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1=".14 .19 .25" rgb2=".38 .43 .49" width="512" height="3072"/>
  </asset>
  <worldbody>
    <light name="key" pos="1 -1 5" dir="-0.2 0.2 -1" directional="true"/>
    <geom name="ground" type="plane" size="12 12 0.1" rgba="0.28 0.31 0.34 1"/>
    <body name="pal_at" pos="{PALLET[0]} {PALLET[1]} 0"><attach model="pal" body="pallet" prefix="p_"/></body>
    <geom name="wall_front" type="box" pos="5.6 0 0.7" size="0.1 3.2 0.7" rgba="0.45 0.47 0.52 1"/>
    <geom name="wall_left"  type="box" pos="2.6 3.1 0.7" size="3.1 0.1 0.7" rgba="0.45 0.47 0.52 1"/>
    <geom name="crate_a"    type="box" pos="3.4 -1.6 0.3" size="0.35 0.35 0.3" rgba="0.55 0.35 0.3 1"/>
    <geom name="post"       type="cylinder" pos="1.9 1.25 0.6" size="0.07 0.6" rgba="0.42 0.36 0.52 1"/>
  </worldbody>
</mujoco>"""


def scan(model, data, height):
    pnt = np.array([SENSOR_XY[0], SENSOR_XY[1], height], dtype=np.float64)
    geomid = np.zeros(1, dtype=np.int32)
    hits = []
    for i in range(SAMPLES):
        a = 2 * math.pi * i / SAMPLES
        vec = np.array([math.cos(a), math.sin(a), 0.0])
        dist = mujoco.mj_ray(model, data, pnt, vec, None, 1, -1, geomid)
        if 0 < dist < RANGE_M:
            hits.append((pnt[0] + dist * vec[0], pnt[1] + dist * vec[1],
                         model.geom(geomid[0]).name))
    return hits


def build():
    model = mujoco.MjModel.from_xml_string(WORLD)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    hits = scan(model, data, SENSOR_Z)
    on_pallet = sum(1 for _, _, n in hits if n.startswith('p_'))

    # 왼쪽: 관측 평면이 팔레트 위를 지나는 옆모습
    side = R.render(
        R.scene_xml(R.EPAL6, (1.95, 0, 0), scan_plane_z=SENSOR_Z),
        azimuth=90, elevation=-7, distance=4.6, lookat=(1.05, 0, 0.42),
        width=740, height=560).crop((0, 150, 740, 470))

    # 오른쪽: 같은 조건의 360° 스캔을 위에서 본 모습
    PW, PH, PXPM = 640, 320, 52.0
    OX, OY = 96.0, PH / 2
    to_px = lambda x, y: (OX + x * PXPM, OY - y * PXPM)
    top = Image.new('RGB', (PW, PH), (26, 29, 36))
    td = ImageDraw.Draw(top)
    for r in range(1, 8):
        td.ellipse([*to_px(-r, r), *to_px(r, -r)], outline=(44, 48, 58))
        td.text((OX + r * PXPM - 10, OY + 4), f'{r}', font=R.F(14), fill=(64, 70, 82))
    for x, y, name in hits:
        u, v = to_px(x, y)
        td.rectangle([u - 2, v - 2, u + 2, v + 2],
                     fill=R.MAGENTA if name.startswith('p_') else (96, 164, 232))
    su, sv = to_px(*SENSOR_XY)
    td.ellipse([su - 6, sv - 6, su + 6, sv + 6], fill=R.RED)
    td.text((su + 10, sv - 9), 'LiDAR', font=R.F(16), fill=(220, 120, 110))
    pu, pv = to_px(PALLET[0], PALLET[1])
    td.rectangle([pu - 0.30 * PXPM, pv - 0.40 * PXPM, pu + 0.30 * PXPM, pv + 0.40 * PXPM],
                 outline=(212, 170, 96), width=2)
    # 개수는 아래 띠가 '측정점 298개 중 팔레트 0개' 로 말한다. 여기서는 위치만 가리킨다.
    td.text((pu - 0.30 * PXPM, pv + 0.40 * PXPM + 4), '팔레트', font=R.F(16),
            fill=(212, 170, 96))
    # 무엇이 찍힌 점인지 이름을 붙인다
    for wx, wy, text, dx, dy in ((5.6, -0.9, '벽', 10, -8), (2.6, 2.55, '벽', -10, -24),
                                 (3.4, -1.6, '상자', 8, 4), (1.9, 1.25, '기둥', 8, -22)):
        u, v = to_px(wx, wy)
        td.text((u + dx, v + dy), text, font=R.F(17), fill=(150, 190, 235))

    GAP, TOP, BAR = 14, 96, 92
    W = side.width + GAP + PW
    canvas, d, _ = R.titled((W, TOP + max(side.height, PH) + BAR + 36),
                            '2D LiDAR 관측 평면과 팔레트 높이',
                            '수평 360° 가상 거리 측정 (MuJoCo) · 차체 반사 제외')
    canvas.paste(side, (0, TOP))
    canvas.paste(top, (side.width + GAP, TOP))
    d = ImageDraw.Draw(canvas)
    y = TOP + max(side.height, PH)
    d.rectangle([0, y + 8, side.width, y + 12], fill=(96, 164, 232))
    d.text((6, y + 24), '옆에서 본 모습 · 하늘색은 거리 측정 높이', font=R.F(21), fill=R.FG)
    d.text((6, y + 56), f'설치 높이 {SENSOR_Z:.2f} m · 팔레트 높이 0.144 m', font=R.F(21), fill=R.SUB)
    x2 = side.width + GAP
    d.rectangle([x2, y + 8, W, y + 12], fill=R.RED)
    d.text((x2 + 6, y + 24), f'위에서 본 모습 · 측정점 {len(hits)}개 중 팔레트 {on_pallet}개',
           font=R.F(21), fill=R.FG)
    d.text((x2 + 6, y + 56), '벽·기둥·상자는 측정된다', font=R.F(20), fill=R.SUB)
    d.text((6, y + BAR + 4),
           '설치 높이는 가정값 · 지도 작성과 실제 장애물 회피는 미구현',
           font=R.F(20), fill=(212, 150, 80))
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '21_lidar_plane.png')
    print('21_lidar_plane.png', canvas.size, f'hits={len(hits)} pallet={on_pallet}')


if __name__ == '__main__':
    build()
