"""Compose forklift + pallet MuJoCo scenes and render deck stills.

Kept in the repository rather than a scratch directory: the week 3 figures are
generated, and a figure whose generator is lost cannot be corrected later.
"""
from __future__ import annotations

import math
import os
import pathlib

os.environ.setdefault('MUJOCO_GL', 'egl')
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
MODELS = ROOT / 'sim/models'
FORKLIFT = MODELS / 'dls08_provisional/forklift.xml'
EPAL6 = MODELS / 'epal6_pallet/pallet.xml'
OUT = ROOT / 'artifacts/week03_assets'

FONT = '/usr/share/fonts/TTF/NanumGothic.ttf'
try:
    ImageFont.truetype(FONT, 20)
except OSError:  # pragma: no cover - font availability differs per machine
    FONT = '/usr/share/fonts/TTF/NanumGothicLight.ttf'


def F(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT, size)


BG, FG, SUB = (18, 20, 24), (240, 240, 240), (152, 158, 170)
GREEN, AMBER, RED, BLUE, GREY = (52, 200, 96), (241, 194, 50), (219, 68, 55), (84, 132, 190), (86, 92, 104)
MAGENTA = (255, 64, 255)

V_TAN, H_TAN = 0.51531, 0.68708     # measured camera half-angle tangents


def scene_xml(pallet_xml, pallet_pos, *, camera_z=None, wedge_len=3.2,
              wedge_rgba='1 0.45 0.15 0.85', scan_plane_z=None, forklift=True):
    """Forklift at the origin, pallet ahead, optional camera marker and wedge.

    ``scan_plane_z`` draws a thin horizontal slab at that height, which is how
    the 2D LiDAR's single observation plane is shown.
    """
    px, py, pyaw = pallet_pos
    extras = []
    if camera_z is not None:
        cx = 0.48                                   # mast face
        extras.append(f'<body name="cam_mark" pos="{cx} 0 {camera_z}">'
                      f'<geom type="box" size="0.035 0.055 0.03" rgba="0.95 0.2 0.2 1"/></body>')
        for sgn in (-1, 1):
            a = sgn * math.atan(V_TAN)
            extras.append(
                f'<body name="wedge{sgn}" pos="{cx + wedge_len/2*math.cos(a)} 0 '
                f'{camera_z + wedge_len/2*math.sin(a)}" euler="0 {-a} 0">'
                f'<geom type="box" size="{wedge_len/2} 0.004 0.004" rgba="{wedge_rgba}"/></body>')
    if scan_plane_z is not None:
        extras.append(f'<geom name="scan_plane" type="box" pos="{px/2 + 0.3} 0 {scan_plane_z}" '
                      f'size="{px/2 + 0.6} 1.1 0.002" rgba="0.35 0.75 1 0.32"/>')
    include = f'<include file="{FORKLIFT}"/>' if forklift else ''
    return f"""<mujoco model="deck">
  {include}
  <option><flag contact="disable"/></option>
  <asset>
    <model name="pal" file="{pallet_xml}"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1=".14 .19 .25" rgb2=".38 .43 .49" width="512" height="3072"/>
  </asset>
  <worldbody>
    <light name="key" pos="1 -1 4" dir="-0.2 0.25 -1" directional="true"/>
    <geom name="ground" type="plane" size="12 12 0.1" rgba="0.28 0.31 0.34 1"/>
    <body name="pal_at" pos="{px} {py} 0" euler="0 0 {pyaw}">
      <attach model="pal" body="pallet" prefix="p_"/>
    </body>
    {''.join(extras)}
  </worldbody>
</mujoco>"""


def render(xml, *, azimuth, elevation, distance, lookat, width=1280, height=720,
           fork_lift=0.0):
    import mujoco
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    if fork_lift:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'fork_lift')
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = fork_lift
    mujoco.mj_forward(model, data)
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = azimuth, elevation, distance
    cam.lookat[:] = lookat
    with mujoco.Renderer(model, height=height, width=width) as r:
        r.update_scene(data, camera=cam, scene_option=mujoco.MjvOption())
        return Image.fromarray(r.render())


def titled(canvas_size, title, subtitle, *, top=92):
    """A dark canvas with the deck's standard heading block."""
    img = Image.new('RGB', canvas_size, BG)
    d = ImageDraw.Draw(img)
    d.text((20, 20), title, font=F(33), fill=FG)
    if subtitle:
        d.text((20, 62), subtitle, font=F(21), fill=SUB)
    return img, d, top

def front_face_line(pose, geometry_depth, geometry_width, z, scene, samples=40):
    """팔레트 앞면을 따라 높이 z 를 지나는 선을 영상 좌표로 투영한다."""
    import numpy as np
    import sys as _sys
    _sys.path[:0] = [str(ROOT / 'src')]
    from forklift_core.perception.overlay import project_point
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
