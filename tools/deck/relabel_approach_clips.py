"""Replace the week-03 approach clips' technical header with report wording.

The original two clips are read from the presentation repository's pre-edit
Git revision, so rerunning this does not layer a new header over an edited one.
Only the header is changed; the 101 synthetic camera poses and detection frames
remain the original measurements.
"""
from __future__ import annotations

import pathlib
import subprocess
import sys
import tempfile

import cv2
from PIL import Image, ImageDraw

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

PRESENTATION = R.ROOT.parent / 'forklift-presentations'
SOURCE_REV = 'ba3c0b840d75eb1b334a8cc3e9ba93f3efacde98'
CLIPS = (
    ('11_gazebo_approach.mp4', 'Gazebo 합성 카메라 · 초록: 정답 · 자홍: 검출 위치'),
    ('14_external_approach.mp4', 'MuJoCo 외부 시점 · 같은 합성 자세 · 실제 주행 아님'),
)


def build_one(name: str, provenance: str) -> None:
    with tempfile.TemporaryDirectory(prefix='week03-approach-header-') as tmp:
        base = pathlib.Path(tmp)
        source = base / name
        with source.open('wb') as out:
            subprocess.run(
                ['git', '-C', str(PRESENTATION), 'show',
                 f'{SOURCE_REV}:week-03/assets/{name}'], stdout=out, check=True)
        video = cv2.VideoCapture(str(source))
        count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = video.get(cv2.CAP_PROP_FPS)
        assert count == 101 and fps == 20, (name, count, fps)
        frames = base / 'frames'
        frames.mkdir()
        for i in range(count):
            ok, frame = video.read()
            assert ok, (name, i)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            detected = int(rgb[75, 5, 1]) > int(rgb[75, 5, 0])
            img = Image.fromarray(rgb)
            d = ImageDraw.Draw(img)
            d.rectangle((0, 0, img.width, 73), fill=R.BG)
            d.text((18, 18), f'카메라–팔레트 앞면 {2.95 - 0.02*i:.2f} m',
                   font=R.F(25), fill=R.FG)
            status = ('포켓 검출' if detected else '포켓 미검출') if name.startswith('11_') else (
                '카메라 시야에 들어옴' if detected else '카메라 시야 밖')
            d.text((395, 18), status,
                   font=R.F(25), fill=R.GREEN if detected else R.RED)
            d.text((660, 23), provenance, font=R.F(18), fill=R.SUB)
            img.save(frames / f'{i:04d}.png')
        video.release()
        R.OUT.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            ['ffmpeg', '-y', '-loglevel', 'error', '-framerate', '20',
             '-i', str(frames / '%04d.png'), '-c:v', 'libx264', '-pix_fmt',
             'yuv420p', '-crf', '18', str(R.OUT / name)], check=True)
        print(name, count, 'synthetic poses; revised header')


if __name__ == '__main__':
    for clip, source_label in CLIPS:
        build_one(clip, source_label)
