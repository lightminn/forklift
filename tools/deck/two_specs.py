"""5쪽: 규격이 다른 두 팔레트를 같은 코드로 검출한다.

채택한 근거가 "새 규격에 코드 수정 없이 적용된다" 이므로, 그 근거를 직접
보이는 화면은 실패한 설정과의 대비가 아니라 **두 규격이 나란히 검출되는
장면**이다. 그래서 같은 접근 자세를 EPAL 6 과 축소 T11 두 형상에 각각 올리고,
같은 검출기에 각 규격의 치수에서 계산한 파라미터만 넣어 돌린다.

자세는 커밋된 catalogue_approach_demo 의 101 개다. 비스듬한 위치에서 정렬하며
가까워지는 연속 동작이라 영상이 부드럽게 이어진다. 흩어진 자세 집합을 쓰면
장면이 매 프레임 튀어서 "같은 코드가 두 규격에서 이어진다" 가 읽히지 않는다.
직선 접근을 거리만 바꿔 훑는 것도 안 된다 — 리그에서는 그 축의 검출이
끊겨서 연속 구간이 0.36 m 뿐이다(2026-09-16 실측).
"""
from __future__ import annotations

import dataclasses
import os
import pathlib
import shutil
import subprocess
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

sys.path[:0] = [str(R.ROOT / 'src'), str(R.ROOT), str(R.ROOT / 'tools')]
import scene_rig  # noqa: E402
from forklift_core.perception.overlay import opening_corners_m, project_point  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.perception.pallet_prior import load_pallet_prior  # noqa: E402
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets  # noqa: E402

CATALOGUE = R.ROOT / 'sim/gazebo/scenes/catalogue_approach_demo.yaml'
SPECS = (
    ('EPAL 6', 'pallet_geometry_epal6.yaml', 'pallet_prior_epal6.yaml', '800 × 600 mm'),
    ('축소 T11', 'pallet_geometry_t11_06.yaml', 'pallet_prior_t11_06.yaml', '660 × 660 mm'),
)
PW, PH, BAR, STRIP, GAP = 700, 234, 96, 48, 12
# 전 자세·두 규격에서 팔레트가 찍히는 행은 286~479 로 화면 아래 40 % 뿐이다
# (2026-09-16 실측). 위쪽은 뒷벽만 있어 잘라낸다 — 자르면 영상이 가로로 길어져
# 슬라이드 폭을 채우고 팔레트도 그만큼 커진다.
CROP_TOP = 266 / 480
MAGENTA = (255, 64, 255)
CAM_X = 0.75


def shade(scene, empty):
    """바닥은 회색조, 팔레트만 주황으로 칠한다.

    팔레트 없는 같은 장면과의 깊이 차로 팔레트 화소를 고른다. 회색조만
    쓰면 팔레트가 바닥과 붙어 보여 무엇이 검출된 것인지 읽히지 않는다.
    """
    depth = np.nan_to_num(scene.depth_m, nan=6.0)
    base = np.nan_to_num(empty.depth_m, nan=6.0)
    near, far = 0.6, 4.2
    grey = np.clip((far - depth) / (far - near), 0, 1)
    img = np.stack([(38 + 150 * grey)] * 3, axis=-1)
    pallet = np.abs(base - depth) > 0.004
    img[pallet] = np.stack([236 + 0 * grey, 170 + 0 * grey, 60 + 0 * grey], axis=-1)[pallet]
    top = int(round(img.shape[0] * CROP_TOP))
    return Image.fromarray(img[top:].astype(np.uint8)).resize((PW, PH), Image.LANCZOS), top


def panel(scene, empty, observation, passed):
    img, top = shade(scene, empty)
    img = img.convert('RGB')
    if passed and observation.status == 'valid':
        d = ImageDraw.Draw(img)
        sx = PW / scene.intrinsics.width
        sy = PH / (scene.intrinsics.height - top)
        for pocket in (observation.left, observation.right):
            px = [project_point(c, scene.intrinsics, scene.base_from_optical)
                  for c in opening_corners_m(pocket, observation.insertion_yaw_rad)]
            if all(p is not None for p in px):
                pts = [(u * sx, (v - top) * sy) for u, v in px]
                d.line([*pts, pts[0]], fill=MAGENTA, width=3)
    return img


def build():
    catalogue = yaml.safe_load(CATALOGUE.read_text())
    # 촬영 순서가 곧 접근 순서다. 정렬하면 동작이 끊긴다.
    poses = [(s.get('id') or s.get('scene_id'), s['pallet']) for s in catalogue['scenes']]

    columns = []
    for name, gfile, pfile, size in SPECS:
        geometry = load_pallet_geometry(R.ROOT / 'config' / gfile)
        prior = load_pallet_prior(R.ROOT / 'config' / pfile)
        params = dataclasses.replace(DetectorParams.derived_for(prior), seed=0)
        boxes = scene_rig.pallet(geometry)
        frames = []
        for _, pose in poses:
            placed = scene_rig.place(
                boxes, x_m=pose['x_m'], y_m=pose['y_m'], yaw_rad=pose['yaw_rad'])
            scene = scene_rig.render(placed)
            empty = scene_rig.render([])
            truth = scene_rig.true_pockets(
                geometry, x_m=pose['x_m'], y_m=pose['y_m'], yaw_rad=pose['yaw_rad'])
            obs = detect_pockets(scene, prior, params).observation
            passed = obs.status == 'valid'
            err = (scene_rig.position_error_m((obs.left.center_m, obs.right.center_m), truth)
                   if passed else 0.0)
            frames.append((panel(scene, empty, obs, passed), passed, err))
        hits = sum(1 for _, ok, _ in frames if ok)
        worst = max((w for _, ok, w in frames if ok), default=0.0)
        columns.append((name, size, geometry, frames, hits, worst))
        print(f'  {name}: {hits}/{len(poses)} · 최악 위치오차 {worst * 1000:.1f} mm')

    frames_dir = pathlib.Path(os.environ.get('DECK_TMP', '/tmp')) / 'two_specs'
    if frames_dir.exists():
        shutil.rmtree(frames_dir)
    frames_dir.mkdir(parents=True)

    W = PW * 2 + GAP
    f_head, f_name, f_note = R.F(26), R.F(25), R.F(21)
    for i, (_, pose) in enumerate(poses):
        canvas = Image.new('RGB', (W, BAR + PH + STRIP), R.BG)
        d = ImageDraw.Draw(canvas)
        head = '같은 자세 · 같은 코드 · 팔레트 규격만 다름'
        d.text(((W - d.textlength(head, font=f_head)) / 2, 8), head, font=f_head, fill=R.FG)
        for k, (name, size, geometry, frames, hits, _) in enumerate(columns):
            img, ok, _ = frames[i]
            x0 = k * (PW + GAP)
            canvas.paste(img, (x0, BAR))
            d.rectangle([x0, BAR - 4, x0 + PW, BAR], fill=R.GREEN if ok else R.RED)
            d.text((x0 + 12, 46), f'{name} · {size}', font=f_name, fill=(235, 235, 235))
            verdict = '포켓 검출' if ok else '미검출'
            d.text((x0 + PW - d.textlength(verdict, font=f_name) - 14, 46), verdict,
                   font=f_name, fill=R.GREEN if ok else R.RED)
            reach = pose['x_m'] - CAM_X - geometry.overall_depth_m / 2
            d.text((x0 + 12, BAR + PH + 5), f'카메라–앞면 {reach:.2f} m', font=f_note, fill=R.SUB)
            y0 = BAR + PH + STRIP - 12
            cw = PW / len(poses)
            for j, (_, on, _) in enumerate(frames):
                d.rectangle([x0 + j * cw, y0, x0 + (j + 1) * cw, y0 + 7],
                            fill=R.GREEN if on else (150, 60, 60))
            d.rectangle([x0 + i * cw - 1, y0 - 3, x0 + (i + 1) * cw + 1, y0 + 10],
                        outline=(255, 255, 255))
        canvas.save(frames_dir / f'{i:04d}.png')

    out = R.OUT / '05_two_specs.mp4'
    R.OUT.mkdir(parents=True, exist_ok=True)
    # 101 자세를 8 fps 로 내보내면 12.6 초다. 발표에서 그 장면 앞에 12 초를
    # 머무르지 않으므로 절반만 보고 넘어간다. 16 fps 로 두 배 빠르게 재생하면
    # 6.3 초라 배정 시간 안에 한 바퀴가 다 돈다 — 접근이 끝까지 이어지는 것이
    # 이 화면의 요지이므로 한 바퀴를 다 보이는 쪽이 맞다.
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', '16',
                    '-i', str(frames_dir / '%04d.png'), '-c:v', 'libx264',
                    '-pix_fmt', 'yuv420p', '-crf', '18',
                    '-vf', 'scale=trunc(iw/2)*2:trunc(ih/2)*2', str(out)], check=True)
    print(out, f'{len(poses)} 접근 자세')


if __name__ == '__main__':
    build()
