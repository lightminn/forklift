"""11쪽: 포크가 팔레트 구멍에 들어가는 구간의 미리보기 영상.

``tools/preview_docking.py`` 가 만드는 검증용 영상과 같은 자세·같은 거리 계산을
쓰되, 자막만 발표용 한국어로 다시 얹는다. 검증 도구의 출력 형식은 기록물이라
건드리지 않고, 발표 자료에 넣을 영상은 이 생성기가 따로 만든다.
"""
from __future__ import annotations

import pathlib
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import render as R  # noqa: E402

sys.path.insert(0, str(R.ROOT / 'tools'))
import preview_docking as P  # noqa: E402

W, H = 1280, 720
BAND = 132
OUTPUT = R.OUT / '09_docking_preview.mp4'

# 영상 안에서는 영어 단계 이름 대신 무엇을 하는 중인지 한 줄로 말한다.
PHASE_KO = {
    'approach': ('① 접근', '팔레트 전면까지 직진 접근'),
    'insert': ('② 삽입', '좌우 포켓에 포크 삽입'),
    'lift': ('③ 승강', '팔레트를 바닥에서 들어 올림'),
    'settle': ('④ 정지', '적재 자세 유지'),
}


def _lines(frame):
    name, what = PHASE_KO[frame.phase]
    mm = frame.penetration_m * 1000
    if mm < 0:
        second = f'포크 끝에서 팔레트 전면까지 {-mm:,.0f} mm'
    else:
        second = (f'포켓 삽입 깊이 {mm:,.0f} mm'
                  f'  ·  포크와 팔레트의 최소 간격 {frame.clearance_m * 1000:,.0f} mm')
    return f'{name}  ·  {what}', second


def build(frames_n=240):
    scene = P._load_scene(P.FORKLIFT, P.PALLET)
    frames = P._plan(scene, frames_n, 0.360)

    import mujoco
    from PIL import Image, ImageDraw

    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    options.sitegroup[:] = 0
    camera = mujoco.MjvCamera()
    inserts = [f.index for f in frames if f.phase == 'insert']
    first, last = (inserts[0], inserts[-1]) if inserts else (0, 0)

    def pose(frame):
        if frame.phase == 'approach':
            t = 0.0
        elif frame.phase == 'insert' and last > first:
            t = (frame.index - first) / (last - first)
        else:
            t = 1.0
        return 132.0 + 36.0 * t, -11.0 + 7.0 * t, 1.9 - 0.4 * t

    command = ['ffmpeg', '-v', 'error', '-nostdin', '-y', '-f', 'rawvideo',
               '-pixel_format', 'rgb24', '-video_size', f'{W}x{H}',
               '-framerate', str(P.FPS), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
               '-pix_fmt', 'yuv420p', '-crf', '20', '-movflags', '+faststart', str(OUTPUT)]

    R.OUT.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(scene.model, height=H, width=W) as renderer:
        with subprocess.Popen(command, stdin=subprocess.PIPE) as encoder:
            try:
                for frame in frames:
                    scene.pose(frame.base_x_m, frame.lift_m)
                    truck_lo, _ = P._box_bounds(scene.model, scene.data, scene.truck_boxes)
                    pallet_lo, pallet_hi = P._box_bounds(scene.model, scene.data, scene.pallet_boxes)
                    x_min, x_max = truck_lo[:, 0].min(), pallet_hi[:, 0].max()
                    camera.lookat[:] = ((x_min + x_max) / 2, 0,
                                        float(pallet_lo[:, 2].min() + pallet_hi[:, 2].max()) / 2)
                    camera.azimuth, camera.elevation, near = pose(frame)
                    camera.distance = max(near, float(x_max - x_min) * 1.15)
                    renderer.update_scene(scene.data, camera=camera, scene_option=options)

                    image = Image.fromarray(renderer.render())
                    d = ImageDraw.Draw(image)
                    d.rectangle((0, H - BAND, W, H), fill=(16, 18, 23))
                    d.rectangle((0, H - BAND, W, H - BAND + 4), fill=R.GREEN)
                    head, detail = _lines(frame)
                    d.text((26, H - BAND + 20), head, font=R.F(34), fill=R.FG)
                    d.text((26, H - BAND + 66), detail, font=R.F(24), fill=(198, 226, 206))
                    d.text((26, H - 32), '접촉과 화물 하중을 반영하지 않은 동작 미리보기',
                           font=R.F(20), fill=R.SUB)
                    # 진행 막대: 지금이 전체 어디쯤인지 한눈에 보이게.
                    t = frame.index / max(1, len(frames) - 1)
                    d.rectangle((0, 0, W, 7), fill=(40, 44, 52))
                    d.rectangle((0, 0, int(W * t), 7), fill=R.GREEN)
                    encoder.stdin.write(image.tobytes())
            finally:
                encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError('ffmpeg failed')
    print(OUTPUT.name, f'{len(frames)} frames', f'{len(frames) / P.FPS:.1f} s')


if __name__ == '__main__':
    build()
