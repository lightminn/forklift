"""6쪽: 아랫판 증거 측정점이 문턱 100개에 못 미쳐 거부되던 이유.

검출기는 포켓으로 판정하기 전에 아랫판이 실제로 보이는지를 확인한다. 그
증거를 세는 띠는 아랫판 윗면 높이 ±6 mm 로, 아랫판이 얇을수록 띠가 얇고
측정점도 적다. 문턱 100개는 아랫판이 50 mm 이던 초기 팔레트에서 정해진
값이라 22 mm 인 EPAL 6 에서는 성립하지 않는다.

같은 자세(s099)를 두 팔레트로 촬영한 장면을 나란히 놓는다. 카메라도 자세도
같고 팔레트 크기만 다르므로, 개수 차이는 팔레트 두께에서만 온다.
"""
from __future__ import annotations

import dataclasses
import pathlib
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

sys.path[:0] = [str(R.ROOT / 'src'), str(R.ROOT)]
from forklift_core.perception import pocket_detector as detector  # noqa: E402
from forklift_core.perception.pallet_prior import load_pallet_prior  # noqa: E402
from forklift_core.perception.pocket_detector import DetectorParams  # noqa: E402
from forklift_core.perception.scene_dataset import load_scene_input  # noqa: E402

SCENE = 's099'                        # 두 카탈로그가 공유하는 가장 가까운 자세
CASES = (
    ('초기 팔레트', 'catalogue_v1', 'pallet_prior_v1.yaml', R.GREEN),
    ('EPAL 6', 'catalogue_epal6', 'pallet_prior_epal6.yaml', R.RED),
)
CROP = (55, 222, 605, 474)
PW, PH, BAR = 760, 318, 150
ZOOM = 3.0


def evidence(scene, prior, params):
    """게이트가 세는 아랫판 증거 측정점 개수."""
    points, _ = detector._base_points(scene)
    cam = scene.base_from_optical.translation_m
    ws = detector._filter_workspace(points, cam, prior, params)
    planes = detector._vertical_plane_candidates(ws, cam, params)
    if not planes:
        return 0
    plane = planes[0]
    lateral = plane.points @ plane.left_axis
    local = np.column_stack((np.zeros(len(lateral)), lateral, plane.points[:, 2]))
    counts, origin = detector._column_grid(local, prior, params)
    gaps = detector._gap_runs(counts > 0)
    if len(gaps) < 2:
        return 0
    band_lateral = ws @ plane.left_axis
    depth = -(ws - plane.point) @ plane.normal
    band = ((np.abs(ws[:, 2] - prior.deck_bottom_m) <= params.deck_evidence_tol_m)
            & (depth >= 0) & (depth <= prior.overall_depth_m + params.plane_inlier_m))
    first, second = gaps[0], gaps[1]
    left = (origin + first[0]) * params.cell_m
    right = (origin + second[1]) * params.cell_m
    return int(np.count_nonzero(band & (band_lateral >= left) & (band_lateral <= right)))


def build():
    frozen = dataclasses.replace(DetectorParams(**yaml.safe_load(
        (R.ROOT / 'config/detector_params_v1.yaml').read_text())), seed=0)
    need = frozen.min_band_points

    panels = []
    for name, catalogue, prior_file, accent in CASES:
        prior = load_pallet_prior(R.ROOT / 'config' / prior_file)
        data = R.ROOT / 'data/synthetic_scenes' / catalogue / 'scenes' / SCENE
        pose = {s['scene_id']: s['pallet'] for s
                in yaml.safe_load((R.ROOT / f'sim/gazebo/scenes/{catalogue}.yaml').read_text())
                ['scenes'] if s.get('pallet')}[SCENE]
        scene = load_scene_input(data)
        got = evidence(scene, prior, frozen)
        ok = got >= need

        sx, sy = PW / (CROP[2] - CROP[0]), PH / (CROP[3] - CROP[1])
        to_panel = lambda p: ((p[0] - CROP[0]) * sx, (p[1] - CROP[1]) * sy)
        img = Image.open(data / 'rgb.png').convert('RGB').crop(CROP).resize((PW, PH), Image.LANCZOS)

        # 증거를 세는 띠: 아랫판 윗면 ±6 mm
        layer = Image.new('RGBA', (PW, PH), (0, 0, 0, 0))
        ld = ImageDraw.Draw(layer)
        tol = frozen.deck_evidence_tol_m
        lo = [to_panel(p) for p in R.front_face_line(
            pose, prior.overall_depth_m, prior.overall_width_m,
            prior.deck_bottom_m - tol, scene)]
        hi = [to_panel(p) for p in R.front_face_line(
            pose, prior.overall_depth_m, prior.overall_width_m,
            prior.deck_bottom_m + tol, scene)]
        if lo and hi:
            ld.polygon(hi + lo[::-1], fill=R.AMBER + (150,))
        img = Image.alpha_composite(img.convert('RGBA'), layer).convert('RGB')

        # 띠가 얇아 원본 크기로는 보이지 않는다 — 확대해서 같이 보여 준다
        if lo and hi:
            cy = int(np.mean([p[1] for p in lo + hi]))
            cx = int(np.mean([p[0] for p in lo + hi]))
            half_w, half_h = int(PW / (2 * ZOOM)), int(PH / (2 * ZOOM))
            box = (max(0, min(PW - 2 * half_w, cx - half_w)),
                   max(0, min(PH - 2 * half_h, cy - half_h)))
            inset = img.crop((box[0], box[1], box[0] + 2 * half_w, box[1] + 2 * half_h))
            inset = inset.resize((int(PW * 0.46), int(PH * 0.46)), Image.LANCZOS)
            ix, iy = PW - inset.width - 12, 12
            img.paste(inset, (ix, iy))
            dd = ImageDraw.Draw(img)
            dd.rectangle([ix, iy, ix + inset.width, iy + inset.height],
                         outline=R.AMBER, width=3)
            dd.text((ix + 8, iy + inset.height - 30), f'{ZOOM:.0f}배 확대',
                    font=R.F(20), fill=R.AMBER)

        card = Image.new('RGB', (PW, PH + BAR), (18, 20, 24))
        card.paste(img, (0, 0))
        d = ImageDraw.Draw(card)
        d.rectangle([0, PH, PW, PH + 5], fill=accent)
        d.text((14, PH + 16), f'{name} · 아랫판 {prior.deck_bottom_m * 1000:.0f} mm',
               font=R.F(26), fill=R.FG)
        d.text((14, PH + 56), f'주황 띠 안의 측정점 {got:,}개 · 문턱 {need}개',
               font=R.F(23), fill=(206, 212, 224))
        d.text((14, PH + 96), '→ 포켓 검출' if ok else '→ 아랫판 증거 부족으로 미검출',
               font=R.F(23), fill=accent)
        panels.append(card)

    W = 2 * PW + 18
    canvas, d, TOP = R.titled((W, 92 + PH + BAR + 44), '아랫판 증거 측정점과 판정 문턱',
                              f'같은 자세 {SCENE} · 주황 띠는 아랫판 윗면 ±'
                              f'{frozen.deck_evidence_tol_m * 1000:.0f} mm')
    for i, p in enumerate(panels):
        canvas.paste(p, (i * (PW + 18), TOP))
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '22_lower_deck.png')
    print('22_lower_deck.png', canvas.size)


if __name__ == '__main__':
    build()
