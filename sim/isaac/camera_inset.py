"""Compose the robot-camera inset shown on the overview video.

This module does not import Isaac Sim, so it runs under plain pytest.

The inset shows three things:

* the perception camera's live picture;
* the pocket openings the detector found, drawn back onto that live picture;
* the heading change still needed to line up with the pallet's insertion axis.

The outlines are **not a live detection**. The detector runs once, at the
observation capture. Each later frame carries that one estimate forward with
the robot's simulator pose and re-projects it. The label says so on screen, so
the video cannot be read as continuous tracking.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from forklift_core.geometry import RigidTransform
from forklift_core.perception.overlay import opening_corners_m, project_point
from forklift_core.perception.pocket_observation import PocketObservation
from forklift_core.planning.pallet_mission import PalletSite
from forklift_core.sensors.rgbd import PinholeIntrinsics

# Camera picture size inside the 1280x720 overview. The inset sits top-left:
# with the overview camera at ~113 px/m that corner covers world x < -1.8 m,
# y > 1.0 m, which no spawn range reaches (start -2.34, 0; pickup x >= 2.6;
# destinations and props x >= -0.8). Every other corner covers a spawn range.
INSET_SIZE = (320, 240)
INSET_MARGIN = 16
PANEL_HEIGHT = 74  # information panel below the camera picture
POCKET_COLOUR = (255, 214, 0)  # amber: stands out on the grey floor and white pallet
PALLET_COLOUR = (0, 230, 118)
TEXT_COLOUR = (255, 255, 255)
BAR_COLOUR = (0, 0, 0, 170)
TURN_SPAN_DEG = 45.0  # gauge half-range; larger requests pin to the edge

# Phases where the pallet still sits where it was detected. From lifting onward
# it rides on the forks, so re-projecting the floor estimate would be wrong.
PALLET_ON_FLOOR_PHASES = ("observe", "approach", "insert")


@dataclass(frozen=True)
class InsetEstimate:
    """The one detection the approach is planned from, with its capture pose."""

    observation: PocketObservation
    capture_pose: tuple[float, float, float]  # base_link x, y, yaw in world
    pallet_site: PalletSite  # estimated centre and insertion yaw in world
    intrinsics: PinholeIntrinsics
    base_from_optical: RigidTransform
    attempt_number: int


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def required_turn_rad(robot_yaw_rad: float, pallet_yaw_rad: float) -> float:
    """Signed heading change that lines the forks up with the insertion axis.

    Positive is counter-clockwise seen from above, which is a left turn.
    """
    return wrap_angle(pallet_yaw_rad - robot_yaw_rad)


def transfer_point(
    point_capture_base: tuple[float, float, float],
    capture_pose: tuple[float, float, float],
    current_pose: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Move a capture-time base_link point into the current base_link frame.

    Planar only: the pallet stays on a level floor and height is unchanged.
    """
    x, y, z = point_capture_base
    cx, cy, cyaw = capture_pose
    wx = cx + math.cos(cyaw) * x - math.sin(cyaw) * y
    wy = cy + math.sin(cyaw) * x + math.cos(cyaw) * y
    nx, ny, nyaw = current_pose
    dx, dy = wx - nx, wy - ny
    return (
        math.cos(nyaw) * dx + math.sin(nyaw) * dy,
        -math.sin(nyaw) * dx + math.cos(nyaw) * dy,
        z,
    )


def projected_openings(
    estimate: InsetEstimate, current_pose: tuple[float, float, float]
) -> list[list[tuple[float, float]]]:
    """Pixel outlines of both pockets in the live image; empty if behind it."""
    outlines = []
    observation = estimate.observation
    for pocket in (observation.left, observation.right):
        corners = opening_corners_m(pocket, observation.insertion_yaw_rad)
        pixels = [
            project_point(
                transfer_point(corner, estimate.capture_pose, current_pose),
                estimate.intrinsics,
                estimate.base_from_optical,
            )
            for corner in corners
        ]
        if all(pixel is not None for pixel in pixels):
            outlines.append(pixels)
    return outlines


def pallet_front_distance_m(
    estimate: InsetEstimate, current_pose: tuple[float, float, float]
) -> float:
    """Forward distance from base_link to the midpoint of the two openings."""
    left = np.asarray(estimate.observation.left.center_m)
    right = np.asarray(estimate.observation.right.center_m)
    mid = tuple(float(v) for v in (left + right) / 2)
    return transfer_point(mid, estimate.capture_pose, current_pose)[0]


def _font(path: str | None, size: int):
    from PIL import ImageFont

    if path:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass
    return ImageFont.load_default()


def render_inset(
    camera_rgb: NDArray[np.uint8] | None,
    *,
    phase: str,
    current_pose: tuple[float, float, float],
    estimate: InsetEstimate | None,
    status: str,
    fork_tip_x_m: float,
    detail: str = "",
    font_path: str | None = None,
) -> NDArray[np.uint8]:
    """Return the annotated camera view with an information panel below it.

    status is the header text when no usable estimate is shown, for example
    while searching or after a rejected observation, and detail is a longer
    second line for the panel, such as the rejection reason. fork_tip_x_m is the fork
    tip's forward offset from base_link, so the distance reads from the forks.
    The panel sits below the picture rather than over it, so a close pallet
    low in the image is never hidden behind text.
    """
    from PIL import Image, ImageDraw

    width, height = INSET_SIZE
    if camera_rgb is None:
        source = Image.new("RGB", (640, 480), (40, 40, 40))
    else:
        source = Image.fromarray(np.ascontiguousarray(camera_rgb[:, :, :3]))
    draw = ImageDraw.Draw(source)
    scale = source.size[0] / width  # draw at source resolution, then shrink

    show_pallet = estimate is not None and phase in PALLET_ON_FLOOR_PHASES
    box = None
    if show_pallet:
        outlines = projected_openings(estimate, current_pose)
        for outline in outlines:
            draw.polygon(outline, outline=POCKET_COLOUR, width=max(2, round(3 * scale)))
        if outlines:
            xs = [u for outline in outlines for u, _ in outline]
            ys = [v for outline in outlines for _, v in outline]
            pad_x = (max(xs) - min(xs)) * 0.18 + 4
            pad_top = (max(ys) - min(ys)) * 1.3 + 4
            box = (min(xs) - pad_x, min(ys) - pad_top, max(xs) + pad_x, max(ys) + 4)
            draw.rectangle(box, outline=PALLET_COLOUR, width=max(3, round(3 * scale)))

    picture = source.resize((width, height), Image.BILINEAR).convert("RGBA")
    layer = Image.new("RGBA", picture.size, (0, 0, 0, 0))
    overlay = ImageDraw.Draw(layer)
    header_font = _font(font_path, 15)
    body_font = _font(font_path, 17)
    small_font = _font(font_path, 12)

    # Header over the top edge: what the camera is doing right now.
    overlay.rectangle((0, 0, width, 24), fill=BAR_COLOUR)
    if show_pallet:
        header = f"팔레트 인식됨 · 관측 #{estimate.attempt_number} 추정치"
    elif estimate is not None:
        header = "팔레트 적재 중 · 포크 위"
    else:
        header = status
    overlay.text((8, 4), f"로봇 카메라  {header}", font=header_font, fill=TEXT_COLOUR)

    gap = None
    if show_pallet:
        gap = pallet_front_distance_m(estimate, current_pose) - fork_tip_x_m
    if box is not None:
        # A name tag above the box keeps a distant pallet findable at inset size.
        left, top = box[0] / scale, box[1] / scale
        tag = "팔레트" if gap is None else f"팔레트 {max(gap, 0.0):.2f} m"
        tag_w = overlay.textlength(tag, font=small_font) + 10
        tag_x = min(max(left, 2), width - tag_w - 2)
        tag_y = max(top - 20, 28)
        overlay.rectangle((tag_x, tag_y, tag_x + tag_w, tag_y + 17), fill=PALLET_COLOUR)
        overlay.text((tag_x + 5, tag_y + 1), tag, font=small_font, fill=(0, 0, 0))
    picture = Image.alpha_composite(picture, layer).convert("RGB")

    panel = Image.new("RGB", (width, PANEL_HEIGHT), (18, 18, 18))
    text = ImageDraw.Draw(panel)
    if show_pallet:
        turn = math.degrees(
            required_turn_rad(current_pose[2], estimate.pallet_site.yaw_rad)
        )
        direction = "왼쪽" if turn > 0.05 else "오른쪽" if turn < -0.05 else "정렬됨"
        amount = "" if direction == "정렬됨" else f" {abs(turn):.1f}°"
        gap_text = f"포크→앞면 {gap:.2f} m" if gap >= 0 else f"포크 진입 {-gap:.2f} m"
        text.text(
            (8, 4),
            f"정렬까지 회전: {direction}{amount}",
            font=body_font,
            fill=TEXT_COLOUR,
        )
        text.text((8, 27), gap_text, font=small_font, fill=TEXT_COLOUR)
        legend = "노랑 포켓 · 초록 팔레트"
        text.text(
            (width - 8 - text.textlength(legend, font=small_font), 27),
            legend,
            font=small_font,
            fill=(190, 190, 190),
        )
        gauge_y = PANEL_HEIGHT - 14
        left, right = 24, width - 24
        centre = (left + right) / 2
        text.line((left, gauge_y, right, gauge_y), fill=(200, 200, 200), width=2)
        for tick in (-TURN_SPAN_DEG, 0.0, TURN_SPAN_DEG):
            x = centre - tick / TURN_SPAN_DEG * (right - left) / 2
            text.line((x, gauge_y - 5, x, gauge_y + 5), fill=(200, 200, 200), width=1)
        # Left turn is drawn to the left, matching the driver's view.
        clamped = max(-TURN_SPAN_DEG, min(TURN_SPAN_DEG, turn))
        marker = centre - clamped / TURN_SPAN_DEG * (right - left) / 2
        colour = PALLET_COLOUR if abs(turn) < 1.0 else POCKET_COLOUR
        text.polygon(
            [
                (marker, gauge_y - 4),
                (marker - 6, gauge_y - 13),
                (marker + 6, gauge_y - 13),
            ],
            fill=colour,
        )
        text.text((left - 16, gauge_y - 8), "L", font=small_font, fill=TEXT_COLOUR)
        text.text((right + 7, gauge_y - 8), "R", font=small_font, fill=TEXT_COLOUR)
    else:
        text.text((8, 4), header, font=body_font, fill=TEXT_COLOUR)
        if estimate is None:
            text.text(
                (8, 28),
                detail or "검출 전에는 회전 각도를 계산하지 않는다",
                font=small_font,
                fill=(190, 190, 190),
            )

    combined = Image.new("RGB", (width, height + PANEL_HEIGHT))
    combined.paste(picture, (0, 0))
    combined.paste(panel, (0, height))
    return np.asarray(combined, dtype=np.uint8)


def compose_frame(
    overview_rgb: NDArray[np.uint8], inset_rgb: NDArray[np.uint8]
) -> NDArray[np.uint8]:
    """Paste the inset at the top-left of the overview with a thin frame."""
    frame = np.array(overview_rgb[:, :, :3], dtype=np.uint8, copy=True)
    inset_h, inset_w = inset_rgb.shape[:2]
    top = INSET_MARGIN
    left = INSET_MARGIN
    border = 2
    frame[
        top - border : top + inset_h + border, left - border : left + inset_w + border
    ] = 255
    frame[top : top + inset_h, left : left + inset_w] = inset_rgb
    return frame
