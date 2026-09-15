"""4쪽: 포켓 인식 네 단계와 단계 사이에 무엇이 넘어가는지."""
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
PW, PH, BAR = 385, 330, 112
ARROW = 88                        # 단계 사이 화살표 칸
CROP = (40, 250, 600, 470)
DIM, POINT = (74, 80, 92), (150, 200, 255)


def panel(title, note, colour=R.BLUE):
    img = Image.new('RGB', (PW, PH + BAR), (26, 29, 36))
    d = ImageDraw.Draw(img)
    d.rectangle([0, PH, PW, PH + BAR], fill=(18, 20, 24))
    d.rectangle([0, PH, PW, PH + 5], fill=colour)
    d.text((12, PH + 16), title, font=R.F(25), fill=R.FG)
    d.text((12, PH + 54), note, font=R.F(19), fill=R.SUB)
    return img, d


def tag(d, text, y=10, size=19, fill=(232, 236, 244)):
    """밝은 화면 위에서도 읽히도록 어두운 받침 위에 올린 한 줄."""
    f = R.F(size)
    w = d.textlength(text, font=f)
    d.rectangle([8, y, 8 + w + 16, y + size + 11], fill=(16, 18, 23))
    d.text((16, y + 4), text, font=f, fill=fill)

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

    img, d = panel('① 깊이 영상', '화소마다 카메라까지의 거리')
    img.paste(Image.open(SCENE / 'depth_preview.png').convert('RGB').crop(CROP)
              .resize((PW, PH), Image.LANCZOS), (0, 0))
    ImageDraw.Draw(img).rectangle([0, PH, PW, PH + 4], fill=R.BLUE)
    tag(ImageDraw.Draw(img), '카메라에서 본 모습')
    panels.append(img)

    img, d = panel('② 앞면 추출', '수직인 평면을 찾아 앞면으로 삼는다')
    scatter(d, points[::3], (44, 48, 58))
    scatter(d, workspace, (60, 150, 96))
    scatter(d, plane.points, R.AMBER, size=2)
    tag(d, '위에서 본 모습')
    tag(d, '초록: 작업 범위 · 주황: 앞면 후보', y=46, fill=(190, 198, 212))
    panels.append(img)

    img, d = panel('③ 빈 공간 구분', '측정점이 없는 칸이 포켓 입구')
    tag(d, '앞면을 좌우로 나눈 결과')
    tag(d, '갈색: 측정점 있음 · 자홍 테두리: 빈 공간', y=46, fill=(190, 198, 212))
    # 판정에 쓰는 것은 칸에 측정점이 있느냐 없느냐뿐이다. 개수를 막대 높이로
    # 그리면 아무 뜻도 없는 높낮이가 눈에 먼저 들어오므로, 있고 없음만 띠로 그린다.
    n = len(counts)
    x0, x1 = 18, PW - 18
    top, bot = 132, 236
    cw = (x1 - x0) / n
    for i, c in enumerate(counts):
        a, b = x0 + i * cw, x0 + (i + 1) * cw
        d.rectangle([a, top, b, bot], fill=(150, 108, 64) if c else (24, 26, 32))
    for a, b in gaps:
        ax, bx = x0 + a * cw, x0 + b * cw
        d.rectangle([ax, top, bx, bot], fill=(24, 26, 32), outline=R.MAGENTA, width=3)
        mm = (b - a) * params.cell_m * 1000
        text = f'{mm:.0f} mm'
        tw = d.textlength(text, font=R.F(21))
        d.text(((ax + bx - tw) / 2, top - 32), text, font=R.F(21), fill=R.MAGENTA)
        d.line([(ax + 2, bot + 14), (bx - 2, bot + 14)], fill=R.MAGENTA, width=2)
    d.rectangle([x0, top, x1, bot], outline=(96, 102, 114), width=1)
    panels.append(img)

    img, d = panel('④ 포켓 위치·방향', '두 포켓의 중심과 삽입 방향', R.GREEN)
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
    tag(ImageDraw.Draw(img), '카메라에서 본 모습')
    panels.append(img)

    carries = ['거리 측정점', '앞면 평면', '빈 공간 위치']
    TOP = 100
    W = len(panels) * PW + (len(panels) - 1) * ARROW
    H = TOP + PH + BAR + 10
    canvas, d, _ = R.titled((W, H), '포켓 위치를 찾는 네 단계',
                            '앞 단계의 결과를 다음 단계에 전달 · 학습 모델 대신 팔레트 치수와 형상 규칙 사용')
    for i, p in enumerate(panels):
        x = i * (PW + ARROW)
        canvas.paste(p, (x, TOP))
        if i < len(carries):
            cy = TOP + PH // 2
            ax = x + PW
            d.line([(ax + 10, cy), (ax + ARROW - 22, cy)], fill=(160, 168, 182), width=4)
            d.polygon([(ax + ARROW - 14, cy), (ax + ARROW - 30, cy - 9),
                       (ax + ARROW - 30, cy + 9)], fill=(160, 168, 182))
            label = carries[i]
            w = d.textlength(label, font=R.F(15))
            d.text((ax + (ARROW - w) / 2, cy - 38), label, font=R.F(15), fill=(186, 194, 208))
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '20_pipeline_stages.png')
    print('20_pipeline_stages.png', canvas.size, f'gaps={len(gaps)} status={obs.status}')


if __name__ == '__main__':
    build()
