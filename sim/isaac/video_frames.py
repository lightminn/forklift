"""SDK-free helpers for the recorded robot-camera video."""

import numpy as np

POCKET_RGB = (255, 214, 0)  # the same amber and green as camera_inset.py
PALLET_RGB = (0, 230, 118)

# Polynomial approximation of the Turbo colormap (Mikhail, 2019), t in [0, 1].
_TURBO = np.array(
    [
        [
            0.13572138,
            4.61539260,
            -42.66032258,
            132.13108234,
            -152.94239396,
            59.28637943,
        ],
        [0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604],
        [
            0.10667330,
            12.64194608,
            -60.58204836,
            110.36276771,
            -89.90310912,
            27.34824973,
        ],
    ]
)


def colorize_depth(depth_m: np.ndarray, near_m: float, far_m: float) -> np.ndarray:
    """(H, W) metres -> (H, W, 3) uint8: near red through far blue, clipped.

    Missing depth (NaN, infinite, zero or negative) is black, never a colour
    that could be read as a distance.
    """
    if not (np.isfinite(near_m) and np.isfinite(far_m)) or not 0 < near_m < far_m:
        raise ValueError("depth range needs 0 < near_m < far_m")
    depth = np.asarray(depth_m, dtype=float)
    valid = np.isfinite(depth) & (depth > 0)
    t = np.clip((far_m - np.where(valid, depth, far_m)) / (far_m - near_m), 0, 1)
    # The darkest ends of Turbo read as black; stay on the blue-to-red span.
    t = 0.10 + 0.85 * t
    powers = np.stack([t**k for k in range(6)], axis=-1)
    rgb = np.clip(powers @ _TURBO.T, 0, 1)
    out = (rgb * 255).round().astype(np.uint8)
    out[~valid] = 0
    return out


def annotate_detection(
    rgb: np.ndarray,
    outlines: list[list[tuple[float, float]]],
    label: str,
    *,
    font_path: str | None = None,
) -> np.ndarray:
    """Copy of the frame with pocket outlines, a pallet box around them, a label.

    outlines are pixel polygons already projected from one detection; this
    only draws them. No outline, no drawing: the frame comes back unchanged.
    """
    if not outlines:
        return rgb
    from PIL import Image, ImageDraw, ImageFont

    image = Image.fromarray(np.ascontiguousarray(rgb[:, :, :3]))
    draw = ImageDraw.Draw(image)
    for outline in outlines:
        draw.polygon(outline, outline=POCKET_RGB, width=3)
    us = [u for outline in outlines for u, _ in outline]
    vs = [v for outline in outlines for _, v in outline]
    box = (min(us) - 14, min(vs) - 22, max(us) + 14, max(vs) + 10)
    draw.rectangle(box, outline=PALLET_RGB, width=3)
    try:
        font = ImageFont.truetype(font_path, 22) if font_path else None
    except OSError:
        font = None
    font = font or ImageFont.load_default()
    text_box = draw.textbbox((box[0], box[1] - 30), label, font=font)
    draw.rectangle(text_box, fill=PALLET_RGB)
    draw.text((box[0], box[1] - 30), label, font=font, fill=(0, 0, 0))
    return np.asarray(image)
