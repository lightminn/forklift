import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
CATALOGUE = ROOT / "sim/gazebo/scenes/catalogue_v1.yaml"


@pytest.fixture
def generator():
    path = ROOT / "tools/generate_scene_catalogue.py"
    assert path.exists(), "implementation is missing"
    spec = importlib.util.spec_from_file_location("scene_catalogue", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "x,y,yaw,w,left,right",
    [  # independently computed from the §5.1 formula, not copied from the generator
        (3.0, 0.4, 0.0, 0.20, (2.7, 0.55, 0.15), (2.7, 0.25, 0.15)),
        (2.5, 0.2, 0.5, 0.24, (2.1552, 0.2054, 0.15), (2.3182, -0.0930, 0.15)),
        (3.5, -0.6, -0.3, 0.28, (3.2695, -0.3298, 0.15), (3.1573, -0.6929, 0.15)),
    ],
)
def test_ground_truth_matches_hand_computed_opening_centres(
    generator, x, y, yaw, w, left, right
):
    truth = generator.pallet_ground_truth(x, y, yaw, w)
    assert truth.status == "valid" and truth.frame_id == "base_link"
    assert truth.insertion_yaw_rad == pytest.approx(yaw)
    assert truth.left.center_m == pytest.approx(left, abs=5e-5)
    assert truth.right.center_m == pytest.approx(right, abs=5e-5)
    assert truth.left.width_m == w and truth.left.height_m == 0.20
    assert (
        truth.position_sigma_m == 0.0
        and truth.source_provenance == "synthetic_ground_truth"
    )


def test_projection_of_a_known_opening_corner(generator):
    # scene (3.0, 0.4, 0, 0.20): left opening top-outer corner at base (2.7, 0.65, 0.25)
    u, v, z = generator.project_to_image((2.7, 0.65, 0.25))
    assert (u, v, z) == pytest.approx((164.8, 299.7, 1.95), abs=0.1)
    view = generator.openings_in_view(3.0, 0.4, 0.0, 0.20)
    assert view["left_in_view"] and view["right_in_view"]
    assert generator.openings_in_view(1.2, 0.0, 0.0, 0.25)["left_in_view"] is False


def test_fixed_catalogue_has_the_approved_composition():
    data = yaml.safe_load(CATALOGUE.read_text())
    scenes = data["scenes"]
    assert data["format_version"] == 1 and data["catalogue_version"] == "v1"
    assert [s["scene_id"] for s in scenes] == [f"s{i:03d}" for i in range(1, 101)]
    by_category = {}
    for s in scenes:
        by_category.setdefault(s["category"], []).append(s)
    assert {k: len(v) for k, v in by_category.items()} == {
        "positive": 60,
        "occluded": 20,
        "negative_no_pallet": 10,
        "negative_lookalike": 10,
    }
    assert sum(s["split"] == "dev" for s in scenes) == 70
    assert sum(s["split"] == "dev" for s in by_category["positive"]) == 42
    for s in scenes:
        if s["category"] in ("positive", "occluded"):
            p = s["pallet"]
            assert 2.0 <= p["x_m"] <= 4.0 and -1.0 <= p["y_m"] <= 1.0
            assert (
                -0.52 <= p["yaw_rad"] <= 0.52 and 0.20 <= p["opening_width_m"] <= 0.28
            )
            assert s["visibility"]["left_in_view"] and s["visibility"]["right_in_view"]
            assert s["ground_truth"]["status"] == "valid"
        else:
            assert s["pallet"] is None and s["ground_truth"]["status"] == "no_pallet"
        if s["category"] == "occluded":
            occ = s["occluder"]
            assert occ["side"] in ("left", "right") and occ["center_m"][0] >= 1.05
            assert 0.2 <= occ["fraction"] <= 0.6
            assert s["visibility"]["occluded_side"] == occ["side"]
            assert s["visibility"]["occluded_fraction_image"] >= occ["fraction"]
            assert occ["size_m"][2] == pytest.approx(0.80)
        else:
            assert s["occluder"] is None
        assert s["category"] != "negative_lookalike" or s["lookalike"] is not None


def test_occlusion_does_not_alter_ground_truth(generator):
    # same pallet, with and without occluder, must give identical ground truth
    data = yaml.safe_load(CATALOGUE.read_text())
    occluded = next(s for s in data["scenes"] if s["category"] == "occluded")
    p = occluded["pallet"]
    truth = generator.pallet_ground_truth(
        p["x_m"], p["y_m"], p["yaw_rad"], p["opening_width_m"]
    )
    stored = occluded["ground_truth"]
    assert stored["left"]["center_m"] == pytest.approx(truth.left.center_m, abs=1e-6)


def test_generation_is_deterministic_and_reproduces_the_fixed_file(generator):
    data = yaml.safe_load(CATALOGUE.read_text())
    regenerated = generator.sample_catalogue(seed=data["seed"], count=100)
    assert regenerated == data


def test_catalogue_ground_truth_loads_as_pocket_observation():
    from forklift_core.perception.pocket_observation import pocket_observation_from_json

    data = yaml.safe_load(CATALOGUE.read_text())
    for s in data["scenes"]:
        pocket_observation_from_json(s["ground_truth"])  # must not raise


@pytest.mark.parametrize("width", [0.20, 0.28])
def test_opening_corners_span_the_approved_width_and_height(generator, width):
    corners = generator.opening_corners(3.0, 0.4, 0.0, width)
    for side, low_y, high_y in (
        ("left", 0.45, 0.45 + width),
        ("right", 0.35 - width, 0.35),
    ):
        assert len(corners[side]) == 4
        actual = sorted(tuple(p) for p in corners[side])
        expected = sorted((2.7, y, z) for y in (low_y, high_y) for z in (0.05, 0.25))
        for point, wanted in zip(actual, expected, strict=True):
            assert point == pytest.approx(wanted)


@pytest.mark.parametrize("width", [0, -0.2, 0.1999, 0.2801, float("nan")])
def test_unapproved_opening_width_is_rejected(generator, width):
    with pytest.raises(ValueError):
        generator.pallet_ground_truth(3.0, 0.0, 0.0, width)


@pytest.mark.parametrize(
    "seed,count", [(True, 100), (1.5, 100), (1, 0), (1, 99), (1, 100.0)]
)
def test_invalid_catalogue_settings_are_rejected(generator, seed, count):
    with pytest.raises(ValueError):
        generator.sample_catalogue(seed, count)


@pytest.mark.parametrize(
    "change",
    [
        {"width": 0},
        {"height": -1},
        {"horizontal_fov_rad": 0},
        {"horizontal_fov_rad": float("nan")},
        {"translation_m": [0, 0]},
        {"optical_quaternion_xyzw": [0, 0, 0, 0]},
    ],
)
def test_invalid_camera_settings_are_rejected(generator, change):
    camera = dict(generator.CAMERA, **change)
    with pytest.raises(ValueError):
        generator.project_to_image((3.0, 0.0, 0.2), camera)


def test_visibility_requires_all_corners_and_positive_depth(generator):
    for x, y in [(0.75, 0.0), (1.05, 0.0), (3.0, 3.0), (3.0, -3.0)]:
        view = generator.openings_in_view(x, y, 0.0, 0.20)
        assert not (view["left_in_view"] and view["right_in_view"])
    assert not generator.openings_in_view(3.0, 0.4, 0, 0.20, min_z_m=2.0)[
        "left_in_view"
    ]
    assert not generator.openings_in_view(3.0, 0.4, 0, 0.20, margin_px=170)[
        "left_in_view"
    ]


def test_visibility_rejection_stops_after_1000_attempts(generator, monkeypatch):
    calls = []

    def never_in_view(*args, **kwargs):
        calls.append(args)
        return {"left_in_view": False, "right_in_view": False, "corners_px": {}}

    monkeypatch.setattr(generator, "openings_in_view", never_in_view)
    with pytest.raises(RuntimeError, match="1000"):
        generator.sample_catalogue(20260911)
    assert len(calls) == 1000


def test_catalogue_rounding_splits_and_distractor_clearance(generator):
    import json
    import math

    data = generator.sample_catalogue(20260911)
    assert json.loads(json.dumps(data, allow_nan=False)) == data
    for category, expected in [
        ("positive", (42, 18)),
        ("occluded", (14, 6)),
        ("negative_no_pallet", (7, 3)),
        ("negative_lookalike", (7, 3)),
    ]:
        scenes = [s for s in data["scenes"] if s["category"] == category]
        assert (
            tuple(sum(s["split"] == split for s in scenes) for split in ("dev", "eval"))
            == expected
        )
    presets = {name: center for name, center, *_ in generator.DISTRACTORS}
    for scene in data["scenes"]:
        p = scene["pallet"] or scene["lookalike"]
        if p:
            assert all(round(v, 4) == v for v in p.values())
            for name in scene["distractors"]:
                center = presets[name]
                assert math.hypot(center[0] - p["x_m"], center[1] - p["y_m"]) >= 1.0
        assert 0 <= len(scene["distractors"]) <= 2
        truth = scene["ground_truth"]
        assert truth["stamp_ns"] == 0 and truth["clock_domain"] == "synthetic"
        assert truth["source_provenance"] == "synthetic_ground_truth"
        if scene["pallet"]:
            yaw, w = p["yaw_rad"], p["opening_width_m"]
            for side, sign in (("left", 1), ("right", -1)):
                # Independent scalar formula from the approved base-frame geometry.
                d = sign * (0.05 + w / 2)
                expected = [
                    p["x_m"] - 0.3 * math.cos(yaw) - d * math.sin(yaw),
                    p["y_m"] - 0.3 * math.sin(yaw) + d * math.cos(yaw),
                    0.15,
                ]
                assert truth[side]["center_m"] == pytest.approx(expected, abs=5.1e-7)
            for corners in scene["visibility"]["corners_px"].values():
                assert all(v == round(v, 2) for point in corners for v in point)
        else:
            assert truth["reason"] == "no target pallet in scene"
            assert truth["left"] is None and truth["right"] is None


def test_occluders_follow_camera_rays_and_only_overlap_the_named_opening():
    import itertools
    import math

    data = yaml.safe_load(CATALOGUE.read_text())
    focal = 320 / math.tan(0.602)
    for scene in data["scenes"]:
        if scene["category"] != "occluded":
            continue
        occ, p = scene["occluder"], scene["pallet"]
        c, s = math.cos(p["yaw_rad"]), math.sin(p["yaw_rad"])
        sign = 1 if occ["side"] == "left" else -1
        d = sign * (0.05 + p["opening_width_m"] / 2)
        ox, oy = p["x_m"] - 0.3 * c - d * s, p["y_m"] - 0.3 * s + d * c
        length = math.hypot(0.75 - ox, -oy)
        assert occ["center_m"] == pytest.approx(
            [
                ox + occ["gap_m"] * (0.75 - ox) / length,
                oy - occ["gap_m"] * oy / length,
                0.4,
            ],
            abs=5.1e-7,
        )
        assert occ["size_m"] == pytest.approx(
            [0.1, occ["fraction"] * p["opening_width_m"], 0.8], abs=5.1e-7
        )
        box_u = []
        for dx, dy, _dz in itertools.product(*[(-v / 2, v / 2) for v in occ["size_m"]]):
            x = occ["center_m"][0] + c * dx - s * dy
            y = occ["center_m"][1] + s * dx + c * dy
            box_u.append(320 - focal * y / (x - 0.75))
        for side, sign in (("left", 1), ("right", -1)):
            opening_u = []
            for offset in (0, p["opening_width_m"]):
                local_y = sign * (0.05 + offset)
                x = p["x_m"] - 0.3 * c - local_y * s
                y = p["y_m"] - 0.3 * s + local_y * c
                opening_u.append(320 - focal * y / (x - 0.75))
            low, high = min(opening_u), max(opening_u)
            overlap = max(0, min(high, max(box_u)) - max(low, min(box_u))) / (
                high - low
            )
            if side == occ["side"]:
                assert overlap >= occ["fraction"]
                assert scene["visibility"]["occluded_fraction_image"] == pytest.approx(
                    overlap, abs=1e-5
                )
            else:
                assert overlap <= 0.1
