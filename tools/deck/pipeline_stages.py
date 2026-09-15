"""4쪽: 포켓 인식 다섯 단계와 단계 사이에 무엇이 넘어가는지."""
from __future__ import annotations

import dataclasses
import pathlib
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

sys.path[:0] = [str(R.ROOT / 'src'), str(R.ROOT)]
from forklift_core.perception import pocket_detector as detector  # noqa: E402
from forklift_core.perception.overlay import opening_corners_m, project_point  # noqa: E402
from forklift_core.perception.pallet_prior import load_pallet_prior  # noqa: E402
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets  # noqa: E402
from forklift_core.perception.scene_dataset import load_scene_sample  # noqa: E402

SCENE = R.ROOT / 'data/synthetic_scenes/catalogue_epal6/scenes/s003'
PW, PH, BAR = 430, 380, 96
ARROW = 150                      # 단계 사이 화살표 칸
CROP = (40, 250, 600, 470)
DIM, POINT = (74, 80, 92), (150, 200, 255)


def panel(title, note, colour=R.BLUE):
    img = Image.new('RGB', (PW, PH + BAR), (26, 29, 36))
    d = ImageDraw.Draw(img)
    d.rectangle([0, PH, PW, PH + BAR], fill=(18, 20, 24))
    d.rectangle([0, PH, PW, PH + 4], fill=colour)
    d.text((12, PH + 14), title, font=R.F(22), fill=R.FG)
    d.text((12, PH + 48), note, font=R.F(18), fill=R.SUB)
    return img, d


def build():
    prior = load_pallet_prior(R.ROOT / 'config/pallet_prior_epal6.yaml')
    params = dataclasses.replace(DetectorParams.derived_for(prior), seed=0)
    sample = load_scene_sample(SCENE)
    scene = sample.input

    points, _ = detector._base_points(scene)
    cam = scene.base_from_optical.translation_m
    workspace = detector._filter_workspace(points, cam, prior, params)
    plane = detector._vertical_plane_candidates(workspace, cam, params)[0]
    lateral = plane.points @ plane.left_axis
    local = np.column_stack((np.zeros(len(lateral)), lateral, plane.points[:, 2]))
    counts, origin = detector._column_grid(local, prior, params)
    gaps = detector._gap_runs(counts > 0)
    obs = detect_pockets(scene, prior, params).observation

    xs, ys = workspace[:, 0], workspace[:, 1]
    xlim = (xs.min() - 0.1, xs.max() + 0.1)
    ylim = (ys.min() - 0.1, ys.max() + 0.1)

    def scatter(d, pts, colour, size=1):
        keep = ((pts[:, 0] >= xlim[0]) & (pts[:, 0] <= xlim[1])
                & (pts[:, 1] >= ylim[0]) & (pts[:, 1] <= ylim[1]))
        pts = pts[keep]
        u = (pts[:, 1] - ylim[0]) / (ylim[1] - ylim[0]) * PW
        v = PH - (pts[:, 0] - xlim[0]) / (xlim[1] - xlim[0]) * PH
        for a, b in zip(PW - u, v):
            d.rectangle([a - size, b - size, a + size, b + size], fill=colour)

    panels = []

    img, d = panel('① 거리 영상', '각 화소에 물체까지의 거리 기록')
    img.paste(Image.open(SCENE / 'depth_preview.png').convert('RGB').crop(CROP)
              .resize((PW, PH), Image.LANCZOS), (0, 0))
    ImageDraw.Draw(img).rectangle([0, PH, PW, PH + 4], fill=R.BLUE)
    panels.append(img)

    img, d = panel('② 팔레트가 있을 범위만 남기기', f'{len(points):,} 점 중 {len(workspace):,} 점')
    scatter(d, points[::3], DIM)
    scatter(d, workspace, (72, 200, 120))
    panels.append(img)

    img, d = panel('③ 수직으로 선 면 찾기', f'점과 면의 어긋남 {plane.residual_p95_m*1000:.1f} mm')
    scatter(d, workspace, DIM)
    scatter(d, plane.points, R.AMBER)
    panels.append(img)

    img, d = panel('④ 기둥과 빈 칸 구분',
                   f'{len(counts)}칸 중 기둥 {int((counts > 0).sum())}칸 · 빈 칸 {len(gaps)}군데')
    n = len(counts); cw = PW / n; top, bottom = 46, PH - 40
    hi = max(1, counts.max())
    gapset = {i for a, b in gaps for i in range(a, b)}
    for i, c in enumerate(counts):
        if c:
            h = (bottom - top) * (c / hi)
            d.rectangle([i * cw + 1, bottom - h, (i + 1) * cw - 1, bottom], fill=(72, 200, 120))
        elif i in gapset:
            d.rectangle([i * cw + 1, bottom - 8, (i + 1) * cw - 1, bottom], fill=R.MAGENTA)
    d.text((10, 12), '초록: 기둥 · 자홍: 빈 칸', font=R.F(16), fill=R.SUB)
    panels.append(img)

    img, d = panel('⑤ 구멍 두 개를 포켓으로 확정', '좌우 포켓 중심과 포크를 넣을 방향', R.GREEN)
    rgb = Image.fromarray(scene.rgb).crop(CROP).resize((PW, PH), Image.LANCZOS)
    sx, sy = PW / (CROP[2] - CROP[0]), PH / (CROP[3] - CROP[1])
    dd = ImageDraw.Draw(rgb)
    if obs.status == 'valid':
        for pocket in (obs.left, obs.right):
            px = [project_point(c, scene.intrinsics, scene.base_from_optical)
                  for c in opening_corners_m(pocket, obs.insertion_yaw_rad)]
            if all(p is not None for p in px):
                pts = [((u - CROP[0]) * sx, (v - CROP[1]) * sy) for u, v in px]
                dd.line([*pts, pts[0]], fill=R.MAGENTA, width=3)
    img.paste(rgb, (0, 0))
    ImageDraw.Draw(img).rectangle([0, PH, PW, PH + 4], fill=R.GREEN)
    panels.append(img)

    carries = ['거리 값', '팔레트 주변 점만', '앞면 한 장', '빈 칸 위치']
    TOP = 96
    W = len(panels) * PW + len(carries) * ARROW
    canvas, d, _ = R.titled((W, TOP + PH + BAR + 46), '포켓 인식 다섯 단계',
                            '왼쪽 결과가 화살표를 따라 오른쪽 단계의 입력이 된다 · 학습 모델을 쓰지 않고 팔레트 치수와 모양만 사용')
    x = 0
    for i, p in enumerate(panels):
        canvas.paste(p, (x, TOP))
        x += PW
        if i < len(carries):
            cy = TOP + PH // 2
            d.line([(x + 26, cy), (x + ARROW - 30, cy)], fill=(150, 158, 172), width=3)
            d.polygon([(x + ARROW - 24, cy), (x + ARROW - 40, cy - 9), (x + ARROW - 40, cy + 9)],
                      fill=(150, 158, 172))
            label = carries[i]
            w = d.textlength(label, font=R.F(19))
            d.text((x + (ARROW - w) / 2, cy - 34), label, font=R.F(19), fill=(180, 188, 202))
            x += ARROW
    d.text((20, TOP + PH + BAR + 12),
           f'촬영 장면 {SCENE.name} 에 인식 프로그램을 적용한 단계별 중간 결과',
           font=R.F(20), fill=R.SUB)
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '20_pipeline_stages.png')
    print('20_pipeline_stages.png', canvas.size, f'gaps={len(gaps)} status={obs.status}')


if __name__ == '__main__':
    build()
