"""11쪽: 가까워질수록 팔레트 앞면이 화면 밖으로 밀려나는 것을 두 거리로 비교한다.

한 장만 보여 주면 널만 찍힌 화면이 되어 '무엇이 사라졌는지'가 보이지 않는다.
앞면이 보이는 거리와 사라진 거리를 나란히 놓아야 사라졌다는 사실이 읽힌다.

화면과 판정은 **같은 카메라 모형**이어야 한다. 그래서 두 화면 모두 검출
판정에 쓰는 측정 리그(`tools/scene_rig.py`)의 거리 영상으로 그린다. MuJoCo
자유 카메라는 지정한 화각과 실제 투영이 달라 같은 장면에서 다른 결론이
나왔다(2026-09-15 확인).
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

sys.path[:0] = [str(R.ROOT / 'src'), str(R.ROOT), str(R.ROOT / 'tools')]
import scene_rig  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.perception.pallet_prior import load_pallet_prior  # noqa: E402
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets  # noqa: E402

CAM_X, CAM_Z = 0.75, 0.50           # 측정에 쓴 카메라 위치
DISTANCES = (1.20, 0.60)            # 카메라에서 팔레트 앞면까지
PW, PH, BAR = 620, 470, 150
SW = 700                            # 옆모습 도해 폭
GAP = 24
EDGE_M = CAM_Z / R.V_TAN            # 이보다 가까운 바닥은 화면에 안 들어온다


def rig_scene(distance_m, geometry, *, with_pallet=True):
    boxes = scene_rig.place(scene_rig.pallet(geometry),
                            x_m=CAM_X + distance_m + geometry.overall_depth_m / 2)
    return scene_rig.render(boxes if with_pallet else [], camera=scene_rig.Camera())


def shade(distance_m, geometry):
    """거리 영상을 회색조로 칠하고 팔레트 화소만 색으로 표시한다."""
    scene = rig_scene(distance_m, geometry)
    empty = rig_scene(distance_m, geometry, with_pallet=False)
    depth = np.nan_to_num(scene.depth_m, nan=6.0)
    near, far = 0.4, 3.2
    grey = np.clip((far - depth) / (far - near), 0, 1)
    img = np.stack([(40 + 150 * grey)] * 3, axis=-1)
    pallet = np.abs(np.nan_to_num(empty.depth_m, nan=6.0) - depth) > 0.004
    img[pallet] = np.stack([236 + 0 * grey, 170 + 0 * grey, 60 + 0 * grey], axis=-1)[pallet]
    out = Image.fromarray(img.astype(np.uint8)).resize((PW, PH), Image.LANCZOS)
    return out, scene


def side_view(geometry):
    """거리마다 한 줄씩, 화면에 들어오는 구간을 색으로 나눈 옆모습."""
    img = Image.new('RGB', (SW, PH + BAR), (24, 27, 33))
    d = ImageDraw.Draw(img)
    RH = (PH - 20) // 2
    scale = (SW - 150) / 2.10
    h = geometry.overall_height_m

    for row, distance in enumerate(DISTANCES):
        y0 = 22 + row * (RH + 20) + RH - 46
        x0 = 52
        d.line([(x0, y0), (SW - 18, y0)], fill=(96, 102, 114), width=3)
        cy = y0 - CAM_Z * scale
        d.line([(x0, cy), (x0 + EDGE_M * scale, y0)], fill=R.AMBER, width=2)
        d.ellipse([x0 - 8, cy - 8, x0 + 8, cy + 8], fill=R.RED)
        d.text((x0 - 44, cy + 10), f'{CAM_Z:.2f} m', font=R.F(18), fill=R.SUB)
        ex = x0 + EDGE_M * scale
        d.line([(ex, y0 - 7), (ex, y0 + 12)], fill=R.AMBER, width=3)
        if row == 0:
            d.text((x0 + 12, cy - 28), '카메라가 보는 아래쪽 경계', font=R.F(20), fill=R.AMBER)
            d.text((ex - 34, y0 + 15), f'{EDGE_M:.2f} m', font=R.F(19), fill=R.AMBER)

        ax = x0 + distance * scale
        bx = x0 + (distance + geometry.overall_depth_m) * scale
        top = y0 - h * scale
        cut = min(max(ex, ax), bx)
        d.rectangle([ax, top, cut, y0], fill=(74, 58, 52), outline=R.RED, width=2)
        if cut < bx:
            d.rectangle([cut, top, bx, y0], fill=(146, 104, 62), outline=R.GREEN, width=3)
        face_seen = ax >= ex
        colour = R.GREEN if face_seen else R.RED
        note = '앞면이 경계 위' if face_seen else '앞면이 경계 아래'
        d.text((ax, top - 28), f'{distance:.2f} m · {note}', font=R.F(21), fill=colour)

    d.text((16, PH + 18), '왜 사라지는가', font=R.F(31), fill=R.FG)
    d.text((16, PH + 60),
           f'카메라가 {CAM_Z:.2f} m 높이에 있으면 {EDGE_M:.2f} m 보다 가까운 바닥은\n'
           '화면에 안 들어온다 · 팔레트 앞면은 바로 그 바닥에 붙어 있다',
           font=R.F(23), fill=R.SUB)
    return img


def build():
    geometry = load_pallet_geometry(R.ROOT / 'config/pallet_geometry_epal6.yaml')
    prior = load_pallet_prior(R.ROOT / 'config/pallet_prior_epal6.yaml')
    params = DetectorParams.derived_for(prior)

    panels = [side_view(geometry)]
    for distance in DISTANCES:
        shot, scene = shade(distance, geometry)
        found = detect_pockets(scene, prior, params).observation.status == 'valid'

        img = Image.new('RGB', (PW, PH + BAR), R.BG)
        img.paste(shot, (0, 0))
        d = ImageDraw.Draw(img)
        colour = R.GREEN if found else R.RED
        d.rectangle([0, PH, PW, PH + 5], fill=colour)
        d.text((14, 12), '주황 = 팔레트 · 회색 = 바닥과 벽', font=R.F(20), fill=(230, 230, 230))
        d.text((16, PH + 18), f'{distance:.2f} m 에서 본 거리 영상', font=R.F(30), fill=R.FG)
        d.text((16, PH + 58),
               '앞면과 포켓이 화면 안에 들어온다' if found
               else '윗판만 보이고 앞면은 화면 밖이다',
               font=R.F(23), fill=R.SUB)
        badge = '포켓 검출' if found else '미검출'
        bw = d.textlength(badge, font=R.F(24))
        d.rounded_rectangle([PW - bw - 42, PH + 100, PW - 16, PH + 138], 7,
                            outline=colour, width=2)
        d.text((PW - bw - 29, PH + 105), badge, font=R.F(24), fill=colour)
        panels.append(img)

    W = SW + 2 * PW + 2 * GAP
    canvas, d, TOP = R.titled((W, 92 + PH + BAR + 8), '가까워지면 팔레트 앞면이 화면 밖으로 내려간다',
                              f'카메라 높이 {CAM_Z:.2f} m 고정 · 정면 접근 · '
                              '화면과 검출 판정 모두 측정 리그의 같은 카메라')
    x = 0
    for p in panels:
        canvas.paste(p, (x, TOP))
        x += p.width + GAP
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '06_near_field_blind.png')
    print('06_near_field_blind.png', canvas.size, f'아래쪽 경계 {EDGE_M:.3f} m')


if __name__ == '__main__':
    build()
