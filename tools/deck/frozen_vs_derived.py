"""7쪽: 같은 촬영 주행을 두 설정으로 돌린 비교.

앞 장의 아랫판 증거 문턱이 갈리는 항이다. 101장에 항을 하나씩만 바꿔 재보면
문턱만 100 → 15 로 낮춰도 50장이 살아나고, 바닥 제외 높이만 고치면 0장
그대로다(2026-09-15 측정). 그래서 바닥 제외 띠는 더 이상 그리지 않는다.
"""
from __future__ import annotations

import dataclasses
import os
import pathlib
import shutil
import subprocess
import sys

import yaml
from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

sys.path[:0] = [str(R.ROOT / 'src'), str(R.ROOT)]
from forklift_core.perception.overlay import opening_corners_m, project_point  # noqa: E402
from forklift_core.perception.pallet_prior import load_pallet_prior  # noqa: E402
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets  # noqa: E402
from forklift_core.perception.scene_dataset import load_scene_input  # noqa: E402

CAPTURE = R.ROOT / 'artifacts/20260914T150409Z_gazebo_approach_demo/scene_capture/scenes'
CAT = R.ROOT / 'sim/gazebo/scenes/catalogue_approach_demo.yaml'
CAM_X, HALF = 0.75, 0.30
CROP = (20, 215, 620, 470)
PW, PH, BAR, STRIP, GAP = 700, 298, 92, 24, 10
MAGENTA = (255, 64, 255)


def panel(scene, pose, params, name, accent, valid):
    img = Image.open(scene / 'rgb.png').convert('RGB').crop(CROP).resize((PW, PH), Image.LANCZOS)
    inp = load_scene_input(scene)
    sx, sy = PW / (CROP[2] - CROP[0]), PH / (CROP[3] - CROP[1])
    to_panel = lambda p: ((p[0] - CROP[0]) * sx, (p[1] - CROP[1]) * sy)

    d = ImageDraw.Draw(img)
    obs = detect_pockets(inp, PRIOR, params).observation
    if obs.status == 'valid':
        for pocket in (obs.left, obs.right):
            px = [project_point(c, inp.intrinsics, inp.base_from_optical)
                  for c in opening_corners_m(pocket, obs.insertion_yaw_rad)]
            if all(p is not None for p in px):
                pts = [to_panel(p) for p in px]
                d.line([*pts, pts[0]], fill=MAGENTA, width=3)
    return img, obs.status == 'valid'


PRIOR = load_pallet_prior(R.ROOT / 'config/pallet_prior_epal6.yaml')


def build():
    frozen = dataclasses.replace(
        DetectorParams(**yaml.safe_load((R.ROOT / 'config/detector_params_v1.yaml').read_text())), seed=0)
    derived = dataclasses.replace(DetectorParams.derived_for(PRIOR), seed=0)
    poses = {s['scene_id']: s['pallet'] for s in yaml.safe_load(CAT.read_text())['scenes']}
    scenes = sorted(d for d in CAPTURE.iterdir() if (d / 'rgb.png').exists())

    frames_dir = pathlib.Path(os.environ.get('DECK_TMP', '/tmp')) / 'frozen_vs_derived'
    if frames_dir.exists():
        shutil.rmtree(frames_dir)
    frames_dir.mkdir(parents=True)

    rows = []
    for scene in scenes:
        pose = poses[scene.name]
        left, lok = panel(scene, pose, frozen, '종전 고정값', R.RED, None)
        right, rok = panel(scene, pose, derived, '규격에서 산출', R.GREEN, None)
        rows.append((scene.name, pose, left, right, lok, rok))

    W = PW * 2 + GAP
    f_t, f_d = R.F(23), R.F(26)
    for i, (sid, pose, left, right, lok, rok) in enumerate(rows):
        rng = pose['x_m'] - CAM_X - HALF
        canvas = Image.new('RGB', (W, BAR + PH + STRIP), R.BG)
        d = ImageDraw.Draw(canvas)
        label = f'같은 합성 접근 자세 · 카메라–팔레트 앞면 거리 {rng:4.2f} m'
        d.text(((W - d.textlength(label, font=f_d)) / 2, 10), label, font=f_d, fill=R.FG)
        for k, (img, ok, name, accent) in enumerate((
                (left, lok, '① 고정 기준값', R.RED),
                (right, rok, '② 치수 기반 산출 · 채택', R.GREEN))):
            x0 = k * (PW + GAP)
            canvas.paste(img, (x0, BAR))
            d.rectangle([x0, BAR - 4, x0 + PW, BAR], fill=R.GREEN if ok else R.RED)
            d.text((x0 + 12, 52), name, font=f_t, fill=(235, 235, 235))
            verdict = '포켓 검출' if ok else '미검출'
            d.text((x0 + PW - d.textlength(verdict, font=f_t) - 14, 52), verdict,
                   font=f_t, fill=R.GREEN if ok else R.RED)
            y0 = BAR + PH + 6
            cw = PW / len(rows)
            for j, row in enumerate(rows):
                on = row[4] if k == 0 else row[5]
                d.rectangle([x0 + j * cw, y0, x0 + (j + 1) * cw, y0 + 11],
                            fill=R.GREEN if on else (92, 98, 110))
            d.rectangle([x0 + i * cw - 1, y0 - 3, x0 + (i + 1) * cw + 1, y0 + 14],
                        outline=(255, 255, 255))
        canvas.save(frames_dir / f'{i:04d}.png')

    out = R.OUT / '04_frozen_vs_derived.mp4'
    R.OUT.mkdir(parents=True, exist_ok=True)
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', '20',
                    '-i', str(frames_dir / '%04d.png'), '-c:v', 'libx264',
                    '-pix_fmt', 'yuv420p', '-crf', '18',
                    '-vf', 'scale=trunc(iw/2)*2:trunc(ih/2)*2', str(out)], check=True)
    print(out, f'{len(rows)} 프레임 · 고정 {sum(r[4] for r in rows)} · 유도 {sum(r[5] for r in rows)}')


if __name__ == '__main__':
    build()
