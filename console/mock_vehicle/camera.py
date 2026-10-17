"""Synthetic forward camera for the fake vehicle: a pinhole render of the mock bay.

Display only. It is not a sensor model; the console draws detections and overlays
on top of it from the vehicle's messages, never from pixels.
"""

from __future__ import annotations

import io
import math

from PIL import Image, ImageDraw

from console.mock_vehicle.world import CAMERA_MODEL, Pallet, Pose, World

NEAR_M = 0.12
WALL_H_M = 3.0


class _Projector:
    def __init__(self, pose: Pose, width: int, height: int):
        cam = CAMERA_MODEL["base_from_camera"]
        fwd, _, height_m = cam["xyz_m"]
        qy, qw = cam["quaternion_xyzw"][1], cam["quaternion_xyzw"][3]
        self.pitch = 2.0 * math.atan2(qy, qw)
        self.c, self.s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        self.ox = pose.x_m + self.c * fwd
        self.oy = pose.y_m + self.s * fwd
        self.oz = height_m
        scale = width / CAMERA_MODEL["width"]
        self.f = CAMERA_MODEL["fx"] * scale
        self.u0, self.v0 = width / 2, height / 2

    def to_cam(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        dx, dy, dz = x - self.ox, y - self.oy, z - self.oz
        fw = dx * self.c + dy * self.s
        lf = -dx * self.s + dy * self.c
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        return fw * cp - dz * sp, -lf, fw * sp + dz * cp  # depth, right, up

    def proj(self, p: tuple[float, float, float]) -> tuple[float, float]:
        d, r, u = p
        return self.u0 + self.f * r / d, self.v0 - self.f * u / d

    def clip(self, pts: list[tuple[float, float, float]]) -> list[tuple[float, float]]:
        out = []
        for i, a in enumerate(pts):
            b = pts[(i + 1) % len(pts)]
            a_in, b_in = a[0] > NEAR_M, b[0] > NEAR_M
            if a_in:
                out.append(a)
            if a_in != b_in:
                t = (NEAR_M - a[0]) / (b[0] - a[0])
                out.append((NEAR_M, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t))
        return [self.proj(p) for p in out]


def _hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _box(cx, cy, hw, hh, yaw, z0, z1):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for a, b in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)):
        for z in (z0, z1):
            out.append((cx + a * c - b * s, cy + a * s + b * c, z))
    return out


def _pockets(p: Pallet) -> list[list[tuple[float, float, float]]]:
    h = p.pose.yaw_rad
    fx = p.pose.x_m - math.cos(h) * p.depth_m / 2
    fy = p.pose.y_m - math.sin(h) * p.depth_m / 2
    pw = min(0.24, p.width_m * 0.3)
    faces = []
    for sign in (-1, 1):
        lc = sign * p.width_m * 0.25
        face = []
        for lat, z in (
            (lc - pw / 2, 0.025),
            (lc + pw / 2, 0.025),
            (lc + pw / 2, 0.122),
            (lc - pw / 2, 0.122),
        ):
            face.append((fx - math.sin(h) * lat, fy + math.cos(h) * lat, z))
        faces.append(face)
    return faces


def render_jpeg(
    world: World,
    pose: Pose,
    lift_m: float,
    frame_id: int,
    width: int = 640,
    height: int = 400,
) -> bytes:
    """Render one frame and return JPEG bytes (quality 70)."""
    pj = _Projector(pose, width, height)
    img = Image.new("RGB", (width, height), (91, 102, 112))
    draw = ImageDraw.Draw(img)
    horizon = pj.v0 - pj.f * math.tan(pj.pitch)
    draw.rectangle([0, max(0, horizon), width, height], fill=(170, 174, 177))
    w, h = world.size_m
    for x1, y1, x2, y2 in ((0, 0, w, 0), (w, 0, w, h), (w, h, 0, h), (0, h, 0, 0)):
        quad = [
            pj.to_cam(x1, y1, 0),
            pj.to_cam(x2, y2, 0),
            pj.to_cam(x2, y2, WALL_H_M),
            pj.to_cam(x1, y1, WALL_H_M),
        ]
        poly = pj.clip(quad)
        if len(poly) >= 3:
            draw.polygon(poly, fill=(111, 122, 131), outline=(89, 99, 107))

    def seg(a, b, fill, width_px=1):
        pa, pb = pj.to_cam(*a), pj.to_cam(*b)
        if pa[0] <= NEAR_M and pb[0] <= NEAR_M:
            return
        if pa[0] <= NEAR_M or pb[0] <= NEAR_M:
            t = (NEAR_M - pa[0]) / (pb[0] - pa[0])
            mid = (NEAR_M, pa[1] + (pb[1] - pa[1]) * t, pa[2] + (pb[2] - pa[2]) * t)
            pa, pb = (mid, pb) if pa[0] <= NEAR_M else (pa, mid)
        draw.line([pj.proj(pa), pj.proj(pb)], fill=fill, width=width_px)

    for gx in range(int(w) + 1):
        seg((gx, 0, 0), (gx, h, 0), (140, 145, 150))
    for gy in range(int(h) + 1):
        seg((0, gy, 0), (w, gy, 0), (140, 145, 150))
    for d in world.destinations.values():
        ring = [
            (
                d.x_m + 0.55 * math.cos(a / 40 * 2 * math.pi),
                d.y_m + 0.55 * math.sin(a / 40 * 2 * math.pi),
                0.002,
            )
            for a in range(41)
        ]
        for a, b in zip(ring, ring[1:], strict=False):
            seg(a, b, (47, 154, 98), 3)

    objects = []
    for cx, cy, r in world.circles:
        pts = [
            (
                cx + r * math.cos(a / 10 * 2 * math.pi),
                cy + r * math.sin(a / 10 * 2 * math.pi),
                z,
            )
            for a in range(10)
            for z in (0.0, 0.88)
        ]
        objects.append(((cx, cy), pts, (47, 93, 138), None))
    for cx, cy, bw, bh in world.boxes:
        objects.append(
            (
                (cx, cy),
                _box(cx, cy, bw / 2, bh / 2, 0.0, 0.0, 0.8),
                (176, 138, 85),
                None,
            )
        )
    for p in world.pallets:
        if p.state == "carried":
            continue
        pts = _box(
            p.pose.x_m,
            p.pose.y_m,
            p.depth_m / 2,
            p.width_m / 2,
            p.pose.yaw_rad,
            0.0,
            0.144,
        )
        objects.append(((p.pose.x_m, p.pose.y_m), pts, (43, 47, 51), p))
    objects.sort(key=lambda o: -pj.to_cam(o[0][0], o[0][1], 0.0)[0])
    for _, pts, colour, pallet in objects:
        cam = [pj.to_cam(*q) for q in pts]
        if any(q[0] <= NEAR_M for q in cam):
            continue
        hull = _hull([pj.proj(q) for q in cam])
        if len(hull) >= 3:
            draw.polygon(
                hull, fill=colour, outline=tuple(int(c * 0.75) for c in colour)
            )
        if pallet is not None:
            for face in _pockets(pallet):
                fc = [pj.to_cam(*q) for q in face]
                if all(q[0] > NEAR_M for q in fc):
                    draw.polygon([pj.proj(q) for q in fc], fill=(7, 9, 10))
    if any(p.state == "carried" for p in world.pallets):
        top = int(height * 0.9 - lift_m * height * 0.5)
        draw.rectangle(
            [int(width * 0.12), top, int(width * 0.88), height], fill=(30, 34, 38)
        )
    draw.rectangle([6, 6, 150, 24], fill=(0, 0, 0))
    draw.text((10, 9), f"MOCK  frame {frame_id}", fill=(232, 237, 240))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=70)
    return buf.getvalue()
