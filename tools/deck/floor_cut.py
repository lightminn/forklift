"""6쪽: 바닥 근처를 버리는 기준선을 실제 촬영 화면 위에 그린다.

점군 옆모습은 팔레트로 보이지 않았다. 촬영한 영상 위에 기준선을 투영하면
'팔레트의 어느 부분이 버려지는가'가 그대로 보인다.
"""
from __future__ import annotations

import dataclasses
import math
import pathlib
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

sys.path[:0] = [str(R.ROOT / 'src'), str(R.ROOT), str(R.ROOT / 'tools')]
from forklift_core.perception import pocket_detector as detector  # noqa: E402
from forklift_core.perception.overlay import project_point  # noqa: E402
from forklift_core.perception.pallet_prior import load_pallet_prior  # noqa: E402
from forklift_core.perception.pocket_detector import DetectorParams  # noqa: E402
from forklift_core.perception.scene_dataset import load_scene_input  # noqa: E402
from measure_pocket_evidence import _gate_terms  # noqa: E402

SCENE = 's099'                       # 팔레트가 가장 크게 찍힌 장면
DATA = R.ROOT / 'data/synthetic_scenes/catalogue_epal6/scenes'
CAT = R.ROOT / 'sim/gazebo/scenes/catalogue_epal6.yaml'
CROP = (55, 222, 605, 474)
PW, PH, BAR = 760, 318, 128
ZOOM = 3.0            # 바닥 띠 확대 배율


def front_face_line(pose, prior, geometry_depth, geometry_width, z, scene, samples=40):
    # prior 는 쓰지 않지만 호출부 호환을 위해 자리를 남긴다.
    """팔레트 전면을 따라 높이 z 를 지나는 선을 영상에 투영한다."""
    yaw = pose['yaw_rad']
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for t in np.linspace(-geometry_width / 2, geometry_width / 2, samples):
        lx, ly = -geometry_depth / 2, float(t)
        base = (pose['x_m'] + lx * c - ly * s, pose['y_m'] + lx * s + ly * c, z)
        px = project_point(base, scene.intrinsics, scene.base_from_optical)
        if px is not None:
            pts.append(px)
    return pts


def build():
    prior = load_pallet_prior(R.ROOT / 'config/pallet_prior_epal6.yaml')
    frozen = DetectorParams(**yaml.safe_load((R.ROOT / 'config/detector_params_v1.yaml').read_text()))
    derived = DetectorParams.derived_for(prior)
    pose = {s['scene_id']: s['pallet'] for s in yaml.safe_load(CAT.read_text())['scenes']}[SCENE]
    scene = load_scene_input(DATA / SCENE)
    points, _ = detector._base_points(scene)

    def evidence(params):
        cam = scene.base_from_optical.translation_m
        ws = detector._filter_workspace(points, cam, prior, params)
        planes = detector._vertical_plane_candidates(ws, cam, params)
        terms = _gate_terms(planes[0], ws, prior, params) if planes else None
        return (terms[1] if terms else 0), params.min_band_points

    sx, sy = PW / (CROP[2] - CROP[0]), PH / (CROP[3] - CROP[1])
    to_panel = lambda p: ((p[0] - CROP[0]) * sx, (p[1] - CROP[1]) * sy)

    panels = []
    for params, name, accent in ((frozen, '고정 기준값', R.RED), (derived, '치수 기반 산출값', R.GREEN)):
        cut = params.floor_z_m
        got, need = evidence(dataclasses.replace(params, seed=0))
        img = Image.open(DATA / SCENE / 'rgb.png').convert('RGB').crop(CROP).resize((PW, PH), Image.LANCZOS)
        layer = Image.new('RGBA', (PW, PH), (0, 0, 0, 0))
        ld = ImageDraw.Draw(layer)

        floor = [to_panel(p) for p in front_face_line(pose, prior, 0.6, 0.8, 0.0, scene)]
        line = [to_panel(p) for p in front_face_line(pose, prior, 0.6, 0.8, cut, scene)]
        board = [to_panel(p) for p in front_face_line(pose, prior, 0.6, 0.8, prior.deck_bottom_m, scene)]
        if floor and line:
            ld.polygon(line + floor[::-1], fill=accent + (110,))
        for seg, colour, width in ((board, (240, 200, 120, 255), 3), (line, accent + (255,), 4)):
            if len(seg) > 1:
                ld.line(seg, fill=colour, width=width)
        img = Image.alpha_composite(img.convert('RGBA'), layer).convert('RGB')

        # 띠는 실제로 몇 화소 되지 않는다. 그 구간만 확대해 옆에 붙인다.
        d = ImageDraw.Draw(img)
        if line and floor:
            mid = len(line) // 2
            cx, cy = line[mid]
            fy = floor[mid][1]
            half_w, half_h = 62, 30
            box = (max(0, cx - half_w), max(0, min(cy, fy) - half_h),
                   min(PW, cx + half_w), min(PH, max(cy, fy) + half_h))
            crop = img.crop(tuple(int(v) for v in box))
            zw, zh = int(crop.width * ZOOM), int(crop.height * ZOOM)
            zoom = crop.resize((zw, zh), Image.NEAREST)
            zx, zy = PW - zw - 12, 12
            img.paste(zoom, (zx, zy))
            d.rectangle([zx, zy, zx + zw, zy + zh], outline=accent, width=3)
            d.rectangle(box, outline=accent, width=2)
            d.line([(box[2], box[1]), (zx, zy + zh)], fill=accent, width=1)
            d.text((zx, zy + zh + 6), f'{ZOOM:.0f}배 확대', font=R.F(17), fill=accent)
        if board:
            d.text((12, max(4, board[0][1] - 30)), '노란 선: 바닥판 상면 22 mm',
                   font=R.F(18), fill=(240, 200, 120))
        if line:
            d.text((12, min(PH - 30, line[0][1] + 10)), f'바닥에서 {cut*1000:.1f} mm 제외',
                   font=R.F(19), fill=accent)

        bar = Image.new('RGB', (PW, BAR), R.BG)
        bd = ImageDraw.Draw(bar)
        bd.rectangle([0, 0, PW, 4], fill=accent)
        ok = got >= need
        bd.text((14, 16), f'{name} · 바닥에서 {cut*1000:.1f} mm 까지 제외', font=R.F(24), fill=R.FG)
        bd.text((14, 56), '바닥판이 제외 구간에 포함된다' if not ok else '바닥판이 남는다',
                font=R.F(23), fill=accent)
        bd.text((14, 92), '→ 팔레트로 판정하지 못함' if not ok else '→ 팔레트로 판정',
                font=R.F(23), fill=accent)
        cell = Image.new('RGB', (PW, PH + BAR), R.BG)
        cell.paste(img, (0, 0)); cell.paste(bar, (0, PH))
        panels.append(cell)
        print(f'{name}: 증거 {got} / 문턱 {need} · 절단 {cut*1000:.2f} mm')

    GAP, TOP = 12, 92
    W = PW * 2 + GAP
    canvas, d, _ = R.titled((W, TOP + PH + BAR + 44), '제외 구간과 팔레트 바닥판',
                            f'촬영 장면 {SCENE} · 색칠한 띠가 제외 구간 · 노란 선이 바닥판 상면')
    for i, p in enumerate(panels):
        canvas.paste(p, (i * (PW + GAP), TOP))
    d.text((20, TOP + PH + BAR + 12),
           '팔레트 판정에는 바닥판 관측이 필요하며, 좌측은 그 바닥판이 제외 구간에 포함된다',
           font=R.F(21), fill=R.SUB)
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '22_floor_cut.png')
    print('22_floor_cut.png', canvas.size)


if __name__ == '__main__':
    build()
