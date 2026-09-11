"""Generate the fixed v1 synthetic pocket-evaluation catalogue (100 scenes)."""

import argparse
import itertools
import math
import random
from pathlib import Path

import numpy as np
import yaml

from forklift_core.geometry import rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.pocket_observation import Pocket, PocketObservation

CAMERA = {
    "width": 640,
    "height": 480,
    "horizontal_fov_rad": 1.204,
    "rate_hz": 5,
    "translation_m": [0.75, 0.0, 0.5],
    "optical_quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
}
PALLET = {
    "depth_m": 0.6,
    "width_m": 0.8,
    "height_m": 0.30,
    "deck_m": 0.05,
    "center_spacer_m": 0.10,
    "opening_height_m": 0.20,
}
RANGES = {
    "x_m": (2.0, 4.0),
    "y_m": (-1.0, 1.0),
    "yaw_rad": (-0.52, 0.52),
    "opening_width_m": (0.20, 0.28),
    "occluder_gap_m": (0.3, 0.6),
    "occluded_fraction": (0.2, 0.6),
}
COUNTS = {
    "positive": 60,
    "occluded": 20,
    "negative_no_pallet": 10,
    "negative_lookalike": 10,
}
LIGHTING = [
    ("sun_front", (-1.0, -0.5, -1.0)),
    ("sun_left", (-0.5, 1.0, -1.0)),
    ("sun_top", (0.0, 0.0, -1.0)),
]
DIFFUSE = (0.5, 0.9)
SURFACES = [
    (".35 .4 .4 1", ".15 .2 .25 1"),
    (".6 .6 .55 1", ".8 .85 .9 1"),
    (".25 .25 .3 1", ".3 .3 .3 1"),
    (".5 .45 .35 1", ".55 .65 .75 1"),
]
DISTRACTORS = [
    ("crate_a", (1.5, 1.6, 0.2), (0.4, 0.4, 0.4), ".5 .5 .5 1"),
    ("crate_b", (4.5, -1.8, 0.3), (0.6, 0.6, 0.6), ".2 .3 .7 1"),
    ("post", (3.0, 2.2, 0.5), (0.3, 0.3, 1.0), ".8 .7 .2 1"),
    ("wall_block", (5.5, 0.0, 0.4), (1.0, 0.5, 0.8), ".7 .2 .2 1"),
    ("bin", (2.2, -2.0, 0.15), (0.5, 0.3, 0.3), ".2 .6 .3 1"),
    ("cube", (4.0, 1.9, 0.25), (0.5, 0.5, 0.5), ".9 .9 .9 1"),
]


def pallet_ground_truth(
    x_m: float, y_m: float, yaw_rad: float, opening_width_m: float
) -> PocketObservation:
    """Return timeless synthetic opening centers in base_link, in meters."""
    if not all(math.isfinite(v) for v in (x_m, y_m, yaw_rad, opening_width_m)):
        raise ValueError("Pallet parameters must be finite")
    if (
        not RANGES["opening_width_m"][0]
        <= opening_width_m
        <= RANGES["opening_width_m"][1]
    ):
        raise ValueError("opening_width_m must lie in [0.20, 0.28]")
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    d = PALLET["center_spacer_m"] / 2 + opening_width_m / 2
    front_x = x_m - PALLET["depth_m"] / 2 * c
    front_y = y_m - PALLET["depth_m"] / 2 * s
    pockets = [
        Pocket(
            (front_x - sign * d * s, front_y + sign * d * c, PALLET["height_m"] / 2),
            opening_width_m,
            PALLET["opening_height_m"],
        )
        for sign in (1, -1)
    ]
    return PocketObservation(
        stamp_ns=0,
        clock_domain="synthetic",
        frame_id="base_link",
        source_provenance="synthetic_ground_truth",
        status="valid",
        left=pockets[0],
        right=pockets[1],
        insertion_yaw_rad=yaw_rad,
        position_sigma_m=0.0,
        yaw_sigma_rad=0.0,
        reason=None,
    )


def opening_corners(
    x_m: float, y_m: float, yaw_rad: float, opening_width_m: float
) -> dict[str, list[tuple[float, float, float]]]:
    """Return four front-plane corners per opening, in base_link meters."""
    truth = pallet_ground_truth(x_m, y_m, yaw_rad, opening_width_m)
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return {
        side: [
            (
                pocket.center_m[0] - dy * s,
                pocket.center_m[1] + dy * c,
                pocket.center_m[2] + dz,
            )
            for dy, dz in itertools.product(
                (-pocket.width_m / 2, pocket.width_m / 2),
                (-pocket.height_m / 2, pocket.height_m / 2),
            )
        ]
        for side, pocket in (("left", truth.left), ("right", truth.right))
    }


def project_to_image(
    point_base: tuple[float, float, float], camera: dict = CAMERA
) -> tuple[float, float, float]:
    """Return (u, v, optical depth m); zero depth has infinite pixel coordinates."""
    for key in ("width", "height"):
        if type(camera[key]) is not int or camera[key] <= 0:
            raise ValueError(f"Camera {key} must be a positive integer")
    fov = camera["horizontal_fov_rad"]
    if not math.isfinite(fov) or not 0 < fov < math.pi:
        raise ValueError("Camera horizontal_fov_rad must lie in (0, pi)")
    point = np.asarray(point_base, dtype=float)
    translation = np.asarray(camera["translation_m"], dtype=float)
    if any(v.shape != (3,) or not np.isfinite(v).all() for v in (point, translation)):
        raise ValueError("Point and camera translation must be finite three-vectors")
    rotation = rotation_matrix_from_quaternion_xyzw(camera["optical_quaternion_xyzw"])
    x, y, z = (float(v) for v in rotation.T @ (point - translation))
    if z == 0:
        return math.inf, math.inf, z
    focal = camera["width"] / 2 / math.tan(fov / 2)
    return focal * x / z + camera["width"] / 2, focal * y / z + camera["height"] / 2, z


def openings_in_view(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    opening_width_m: float,
    camera: dict = CAMERA,
    margin_px: float = 4,
    min_z_m: float = 0.3,
) -> dict:
    """Require all four corners of each opening to satisfy depth and image bounds."""
    result = {"corners_px": {}}
    for side, corners in opening_corners(x_m, y_m, yaw_rad, opening_width_m).items():
        projected = [project_to_image(p, camera) for p in corners]
        result[f"{side}_in_view"] = all(
            z >= min_z_m
            and margin_px <= u <= camera["width"] - margin_px
            and margin_px <= v <= camera["height"] - margin_px
            for u, v, z in projected
        )
        result["corners_px"][side] = [[u, v] for u, v, _ in projected]
    return result


def _rounded_values(value):
    # Normalize constants as well as generated values; avoid YAML-specific tuples.
    if isinstance(value, dict):
        return {key: _rounded_values(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_rounded_values(item) for item in value]
    if isinstance(value, float):
        return round(value, 6)
    return value


def _sample_parameters(rng, names):
    # Geometry must be derived from exactly the parameters that will be serialized.
    return {name: round(rng.uniform(*RANGES[name]), 4) for name in names}


def _sample_occluder(rng, pallet, visibility):
    truth = pallet_ground_truth(**pallet)
    c, s = math.cos(pallet["yaw_rad"]), math.sin(pallet["yaw_rad"])
    for _ in range(1000):
        side = rng.choice(["left", "right"])
        gap = round(rng.uniform(*RANGES["occluder_gap_m"]), 4)
        fraction = round(rng.uniform(*RANGES["occluded_fraction"]), 4)
        ox, oy, _ = getattr(truth, side).center_m
        dx, dy = CAMERA["translation_m"][0] - ox, CAMERA["translation_m"][1] - oy
        length = math.hypot(dx, dy)
        center = [ox + gap * dx / length, oy + gap * dy / length, 0.40]
        size = [0.10, fraction * pallet["opening_width_m"], 0.80]
        if center[0] < 1.05:
            continue
        box_u = [
            project_to_image(
                (
                    center[0] + c * dx - s * dy,
                    center[1] + s * dx + c * dy,
                    center[2] + dz,
                )
            )[0]
            for dx, dy, dz in itertools.product(*[(-v / 2, v / 2) for v in size])
        ]
        overlaps = {}
        for name, corners in visibility["corners_px"].items():
            low, high = min(p[0] for p in corners), max(p[0] for p in corners)
            overlaps[name] = max(0.0, min(high, max(box_u)) - max(low, min(box_u))) / (
                high - low
            )
        other = "right" if side == "left" else "left"
        if overlaps[side] >= fraction and overlaps[other] <= 0.10:
            return {
                "side": side,
                "gap_m": gap,
                "fraction": fraction,
                "center_m": center,
                "size_m": size,
                "color": ".45 .3 .5 1",
            }, overlaps[side]
    raise RuntimeError("Could not sample an occluder in 1000 attempts")


def sample_catalogue(seed: int, count: int = 100) -> dict:
    """Return the v1 60/20/10/10 catalogue using one seeded random stream."""
    if type(seed) is not int or type(count) is not int or count != sum(COUNTS.values()):
        raise ValueError("seed must be an integer and v1 count must be 100")
    rng = random.Random(seed)
    categories = [name for name, number in COUNTS.items() for _ in range(number)]
    rng.shuffle(categories)
    splits = ["eval"] * count
    for category in COUNTS:
        indices = [i for i, label in enumerate(categories) if label == category]
        rng.shuffle(indices)
        for i in indices[: len(indices) * 7 // 10]:
            splits[i] = "dev"

    scenes = []
    for index, category in enumerate(categories):
        pallet = lookalike = occluder = None
        occluded_fraction_image = None
        visibility = {"left_in_view": False, "right_in_view": False, "corners_px": {}}
        if category in ("positive", "occluded"):
            for _ in range(1000):
                pallet = _sample_parameters(
                    rng, ("x_m", "y_m", "yaw_rad", "opening_width_m")
                )
                visibility = openings_in_view(**pallet)
                if visibility["left_in_view"] and visibility["right_in_view"]:
                    break
            else:
                raise RuntimeError("Could not sample visible openings in 1000 attempts")
        elif category == "negative_lookalike":
            lookalike = _sample_parameters(rng, ("x_m", "y_m", "yaw_rad"))
        if category == "occluded":
            occluder, occluded_fraction_image = _sample_occluder(
                rng, pallet, visibility
            )

        target = pallet or lookalike
        available = [
            name
            for name, center, *_ in DISTRACTORS
            if target is None
            or math.hypot(center[0] - target["x_m"], center[1] - target["y_m"]) >= 1.0
        ]
        distractors = rng.sample(available, rng.randint(0, 2))
        light_name, direction = rng.choice(LIGHTING)
        diffuse = rng.choice(DIFFUSE)
        floor, background = rng.choice(SURFACES)
        if pallet is not None:
            truth = pallet_ground_truth(**pallet)
        else:
            truth = PocketObservation(
                stamp_ns=0,
                clock_domain="synthetic",
                frame_id="base_link",
                source_provenance="synthetic_ground_truth",
                status="no_pallet",
                left=None,
                right=None,
                insertion_yaw_rad=None,
                position_sigma_m=None,
                yaw_sigma_rad=None,
                reason="no target pallet in scene",
            )
        visibility["occluded_side"] = occluder["side"] if occluder else None
        visibility["occluded_fraction_image"] = occluded_fraction_image
        visibility["corners_px"] = {
            side: [[round(v, 2) for v in point] for point in corners]
            for side, corners in visibility["corners_px"].items()
        }
        scenes.append(
            {
                "scene_id": f"s{index + 1:03d}",
                "split": splits[index],
                "category": category,
                "pallet": pallet,
                "lookalike": lookalike,
                "occluder": occluder,
                "distractors": distractors,
                "lighting": {
                    "name": light_name,
                    "direction": direction,
                    "diffuse": diffuse,
                },
                "surfaces": {"floor": floor, "background": background},
                "visibility": visibility,
                "ground_truth": truth.to_json(),
            }
        )
    return _rounded_values(
        {
            "format_version": 1,
            "catalogue_version": "v1",
            "seed": seed,
            "generator": "tools/generate_scene_catalogue.py",
            "source_provenance": "synthetic",
            "camera": CAMERA,
            "pallet": PALLET,
            "ranges": RANGES,
            "distractor_presets": {
                name: {"center_m": list(center), "size_m": list(size), "color": color}
                for name, center, size, color in DISTRACTORS
            },
            "scenes": scenes,
        }
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    catalogue = sample_catalogue(args.seed, args.count)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(catalogue, sort_keys=False, allow_unicode=True)
    )
