"""10쪽: 카메라 설치 높이에 따른 근거리 인식.

이전 판은 근접 한계를 1.05 / 1.25 / 1.75 m 로 **상수로 박고** 있었다. 그 값은
배치 좌표 격자(앞면 1.25 m 부터 0.05 m 간격)의 **첫 표본**이지 검출 한계가
아니다 — 그보다 가까운 거리를 잰 적이 없었다. 같은 리그로 0.01 m 간격으로
훑으면 0.50 m 높이에서 0.84 m 까지, 0.27 m 높이에서 0.80 m 까지 검출된다
(2026-09-15 재측정). 또 검출 구간은 연속이 아니라 군데군데 끊긴다.

그래서 값을 박지 않고 여기서 직접 훑는다. 화면에는 "가장 가까운 거리" 를 아예
싣지 않는다 — 그 값은 격자에 딸려 움직인다. 0.50 m 높이에서 0.01 m 간격이면
0.84 m, 0.05 m 간격이면 1.05 m 다(0.84 가 0.05 격자에 없고 0.85~0.88 은
미검출이다). 대신 **정해진 구간의 검출 비율**을 싣는다. 비율은 끊긴 구간을
왜곡 없이 요약하고, 설치가 높을수록 근거리에서 못 본다는 결론도 그대로 읽힌다.

그림은 세 패널 모두 팔레트를 **같은 거리**에 두어 높이 차이만 비교한다.

거리는 모두 **카메라에서 팔레트 전면까지**다. 포크 끝은 모델의 left_fork_tip
site (x = 0.95 m), 카메라 x 는 측정에 쓴 0.75 m 다. 두 좌표를 섞으면 잔여
거리가 270 mm 씩 어긋난다 — 실제로 한 번 어긋났다.
"""
from __future__ import annotations

import pathlib
import sys

from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

sys.path[:0] = [str(R.ROOT / 'src'), str(R.ROOT), str(R.ROOT / 'tools')]
import scene_rig  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.perception.pallet_prior import load_pallet_prior  # noqa: E402
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets  # noqa: E402

CAM_X = 0.75            # 측정에 쓴 카메라 x (배치 좌표계)
FORK_TIP_X = 0.95       # forklift.xml 의 left_fork_tip site
HALF_DEPTH = 0.30       # EPAL 6 절반 깊이

MOUNTS = [
    (0.27, '낮게 설치'),
    (0.50, '현재 기준선'),
    (0.90, '마스트 상단 · 과제 제시안'),
]
NEAR, FAR, STEP = 0.60, 2.30, 0.05      # 훑는 구간과 간격 (카메라–앞면)
SHOWN_M = 1.20                          # 그림에 쓰는 고정 거리 (세 패널 공통)

PW, CROP, BAR = 590, (95, 430), 132


def sweep(height, geometry, prior, params, boxes):
    """이 높이에서 구간을 훑어 (검출 거리 목록, 표본 수) 를 낸다."""
    hits, total = [], 0
    d = NEAR
    while d <= FAR + 1e-9:
        distance = round(d, 3)
        placed = scene_rig.place(boxes, x_m=CAM_X + distance + geometry.overall_depth_m / 2)
        scene = scene_rig.render(placed, camera=scene_rig.Camera(xyz_m=(CAM_X, 0.0, height)))
        if detect_pockets(scene, prior, params).observation.status == 'valid':
            hits.append(distance)
        total += 1
        d += STEP
    return hits, total


def build():
    geometry = load_pallet_geometry(R.ROOT / 'config/pallet_geometry_epal6.yaml')
    prior = load_pallet_prior(R.ROOT / 'config/pallet_prior_epal6.yaml')
    params = DetectorParams.derived_for(prior)
    boxes = scene_rig.pallet(geometry)

    panels, measured = [], []
    for z, label in MOUNTS:
        hits, total = sweep(z, geometry, prior, params, boxes)
        nearest = min(hits) if hits else None
        measured.append((z, len(hits), total, nearest))

        front = CAM_X + SHOWN_M
        xml = R.scene_xml(R.EPAL6, (front + HALF_DEPTH, 0, 0), camera_z=z,
                          wedge_len=SHOWN_M + 0.9, wedge_rgba='1 0.45 0.15 0.9')
        img = R.render(xml, azimuth=90, elevation=-6, distance=5.0,
                       lookat=(1.35, 0, 0.45), width=PW, height=470)
        img = img.crop((0, CROP[0], PW, CROP[1]))
        accent = R.RED if z >= 0.9 else (R.AMBER if z >= 0.5 else R.GREEN)

        bar = Image.new('RGB', (PW, BAR), R.BG)
        d = ImageDraw.Draw(bar)
        d.rectangle([0, 0, PW, 5], fill=accent)
        d.text((14, 16), f'카메라 높이 {z:.2f} m', font=R.F(29), fill=R.FG)
        d.text((14, 54), label, font=R.F(21), fill=accent)
        d.text((14, 86), f'{NEAR:.1f}~{FAR:.1f} m 구간에서 {len(hits)}/{total}개 자세 검출',
               font=R.F(21), fill=(192, 198, 210))
        cell = Image.new('RGB', (PW, img.height + BAR), R.BG)
        cell.paste(img, (0, 0))
        cell.paste(bar, (0, img.height))
        panels.append(cell)

    GAP, TOP = 8, 96
    W = PW * len(panels) + GAP * (len(panels) - 1)
    body_h = panels[0].height
    canvas, d, _ = R.titled((W, TOP + body_h + 16),
                            '설치 높이 0.27 / 0.50 / 0.90 m 의 근거리 관측 비교',
                            # 이 그림만 렌더러 표기가 어디에도 없었다. 본문에서 "합성"
                            # 수식어를 걷어낸 뒤로는 세 높이에 카메라를 달고 잰 실물
                            # 벤치 실험으로 읽힐 수 있어, 출처를 그림 안에 한 번 적는다.
                            f'시뮬레이션 깊이 영상 · 그림은 세 패널 모두 카메라–앞면 {SHOWN_M:.2f} m · '
                            f'수치는 {NEAR:.1f}~{FAR:.1f} m 를 {STEP * 100:.0f} cm 간격으로 훑은 결과 · '
                            '주황색은 카메라의 수직 관측 범위')
    for i, cell in enumerate(panels):
        canvas.paste(cell, (i * (PW + GAP), TOP))
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '13_camera_mount.png')
    print('13_camera_mount.png', canvas.size)
    for z, hit, total, nearest in measured:
        gap = f'{(CAM_X + nearest - FORK_TIP_X) * 1000:.0f} mm' if nearest else '-'
        print(f'  높이 {z:.2f} m: {hit}/{total} 검출 · 최근접 '
              f'{f"{nearest:.2f} m" if nearest else "없음"} · 포크 끝 잔여 {gap}')


if __name__ == '__main__':
    build()
