"""9쪽: 촬영한 장면의 조건과 그 장면에서 실제로 나온 판정 결과."""
from __future__ import annotations

import dataclasses
import json
import pathlib
import sys

import yaml
from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

sys.path[:0] = [str(R.ROOT / 'src'), str(R.ROOT)]
from forklift_core.perception.pallet_prior import load_pallet_prior  # noqa: E402
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets  # noqa: E402
from forklift_core.perception.scene_dataset import load_scene_input  # noqa: E402

SC = R.ROOT / 'data/synthetic_scenes/catalogue_epal6/scenes'
CAT = R.ROOT / 'sim/gazebo/scenes/catalogue_epal6.yaml'
CROP = (30, 210, 610, 480)
PW, PH, BAR, GAP, COLS = 540, 251, 92, 10, 3
CAM_X, HALF = 0.75, 0.30          # 카메라 x, 팔레트 절반 깊이
GREEN, AMBER, BLUE_, RED = (72, 200, 120), (236, 186, 60), (120, 176, 232), (226, 84, 70)


def build():
    prior = load_pallet_prior(R.ROOT / 'config/pallet_prior_epal6.yaml')
    params = dataclasses.replace(DetectorParams.derived_for(prior), seed=0)
    poses = {s['scene_id']: (s.get('pallet') or {}) for s in yaml.safe_load(CAT.read_text())['scenes']}
    meta = {}
    for d in sorted(SC.iterdir()):
        j = d / 'scene.json'
        if j.exists():
            m = json.loads(j.read_text())
            m['_id'] = d.name
            m['pallet'] = poses.get(d.name, {})
            meta[d.name] = m

    def pick(cat, key=None, rev=False):
        xs = [m for m in meta.values() if m.get('category') == cat]
        if key:
            xs.sort(key=key, reverse=rev)
        return xs[0]['_id']

    pal = lambda m: m.get('pallet', {})
    chosen = [
        (pick('positive', lambda m: pal(m).get('x_m', 0), True), '먼 거리'),
        (pick('positive', lambda m: pal(m).get('x_m', 9)), '가까운 거리'),
        (pick('positive', lambda m: abs(pal(m).get('yaw_rad', 0)), True), '비스듬한 자세'),
        (pick('occluded'), '기둥에 가림'),
        (pick('negative_lookalike'), '비슷하게 생긴 물체'),
        (pick('negative_no_pallet'), '팔레트 없음'),
    ]

    def verdict(sid):
        category = meta[sid]['category']
        status = detect_pockets(load_scene_input(SC / sid), prior, params).observation.status
        if status == 'valid':
            return ('포켓 검출', GREEN) if category in ('positive', 'occluded') else ('잘못 검출', RED)
        if category == 'positive':
            return '미검출', RED
        if category == 'occluded':
            return '판단 불가로 거부', AMBER
        return '검출하지 않음', BLUE_

    rows = (len(chosen) + COLS - 1) // COLS
    W = COLS * PW + (COLS - 1) * GAP
    canvas = Image.new('RGB', (W, rows * (BAR + PH) + (rows - 1) * GAP), R.BG)
    d = ImageDraw.Draw(canvas)
    for i, (sid, label) in enumerate(chosen):
        x0 = (i % COLS) * (PW + GAP)
        y0 = (i // COLS) * (BAR + PH + GAP)
        im = Image.open(SC / sid / 'rgb.png').convert('RGB').crop(CROP).resize((PW, PH), Image.LANCZOS)
        canvas.paste(im, (x0, y0 + BAR))
        d.rectangle([x0, y0 + BAR - 3, x0 + PW, y0 + BAR], fill=(70, 130, 200))
        d.text((x0 + 10, y0 + 6), label, font=R.F(21), fill=R.FG)
        p = pal(sid and meta[sid])
        if p:
            extra = f"팔레트까지 {p.get('x_m', 0) - CAM_X - HALF:.2f} m · 틀어진 각 {p.get('yaw_rad', 0)*57.3:+.0f}°"
            d.text((x0 + 10, y0 + 33), extra, font=R.F(17), fill=(150, 156, 168))
        text, colour = verdict(sid)
        w = d.textlength(text, font=R.F(19))
        d.rectangle([x0 + PW - w - 28, y0 + 10, x0 + PW - 10, y0 + 42], outline=colour, width=2)
        d.text((x0 + PW - w - 19, y0 + 16), text, font=R.F(19), fill=colour)
    R.OUT.mkdir(parents=True, exist_ok=True)
    canvas.save(R.OUT / '03_scene_variety.png')
    print('03_scene_variety.png', canvas.size, [s for s, _ in chosen])


if __name__ == '__main__':
    build()
