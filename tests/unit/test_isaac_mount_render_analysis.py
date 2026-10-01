"""SDK-free rules of the carriage-camera render check (plan 2026-10-01, step 4)."""

import dataclasses
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from tools import measure_pocket_evidence as mpe
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "mount_render_analysis", ROOT / "sim/isaac/mount_render_analysis.py"
)
A = importlib.util.module_from_spec(SPEC)
sys.modules["mount_render_analysis"] = A
SPEC.loader.exec_module(A)

URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"
GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
CAMERA = scene_rig.Camera((0.61, 0.0, 0.20), 0.0)
# Isaac's SDK K (cx = width / 2) normalised to integer-index centres.
K = dataclasses.replace(scene_rig.intrinsics(), cx=319.5, cy=239.5)
TRUCK = scene_rig.truck_boxes(URDF)
SIDE = mpe.blades(TRUCK)
TIP_X = TRUCK[SIDE["left"]].centre_m[0] + TRUCK[SIDE["left"]].size_m[0] / 2


def pallet_x(gap):
    return TIP_X + GEOMETRY.overall_depth_m / 2 + gap


def depth(gap, *, carriage=True, pallet=True):
    boxes = [
        *(TRUCK if carriage else []),
        *(
            scene_rig.place(scene_rig.pallet(GEOMETRY), x_m=pallet_x(gap))
            if pallet
            else []
        ),
    ]
    scene = scene_rig.render(boxes, camera=CAMERA, quantize=False, intrinsics=K)
    return np.asarray(scene.depth_m, dtype=float)


def renders(gap):
    return {
        "full": depth(gap),
        "no_carriage": depth(gap, carriage=False),
        "no_pallet": depth(gap, pallet=False),
    }


def test_grid_is_the_pre_registered_91_cells():
    grid = A.face_gap_grid()
    assert len(grid) == 91
    assert grid[0] == -0.20 and grid[-1] == 0.70
    assert all(
        math.isclose(b - a, 0.01, abs_tol=1e-9)
        for a, b in zip(grid, grid[1:], strict=False)
    )


def test_rig_intrinsics_default_is_unchanged():
    default = scene_rig.render(TRUCK, camera=CAMERA)
    explicit = scene_rig.render(TRUCK, camera=CAMERA, intrinsics=scene_rig.intrinsics())
    np.testing.assert_array_equal(default.depth_m, explicit.depth_m)
    assert default.intrinsics == scene_rig.intrinsics()
    shifted = scene_rig.render(TRUCK, camera=CAMERA, intrinsics=K)
    assert shifted.intrinsics.cx == 319.5
    assert not np.array_equal(
        np.nan_to_num(default.depth_m), np.nan_to_num(shifted.depth_m)
    )


def test_regions_follow_what_each_hidden_render_removes():
    r = renders(0.30)
    labels = A.region_labels(**r)
    assert np.count_nonzero(labels == A.CARRIAGE) > 1000
    assert np.count_nonzero(labels == A.PALLET) > 1000
    assert np.count_nonzero(labels == A.INVALID) == 0
    # A carriage pixel is where hiding the carriage moves the depth away.
    carriage = labels == A.CARRIAGE
    assert np.all(r["full"][carriage] < r["no_carriage"][carriage] - 0.002)


def test_a_pixel_invalid_in_any_render_is_invalid():
    r = renders(0.30)
    r["no_pallet"][10, 20] = 0.0  # a miss
    r["no_carriage"][11, 20] = np.nan
    labels = A.region_labels(**r)
    assert labels[10, 20] == A.INVALID and labels[11, 20] == A.INVALID
    assert A.boundary_mask(labels)[10, 21]


def tips(r, labels=None, boundary=None):
    labels = A.region_labels(**r) if labels is None else labels
    boundary = A.boundary_mask(labels) if boundary is None else boundary
    return {
        name: A.pixel_tip_visibility(
            r["full"],
            labels,
            boundary,
            mpe.tip_samples(TRUCK[index]),
            CAMERA.base_from_optical(),
            K,
        )
        for name, index in SIDE.items()
    }


def test_pixel_rule_agrees_with_the_continuous_ray_where_the_tip_is_open():
    gap = 0.30
    placed = scene_rig.place(scene_rig.pallet(GEOMETRY), x_m=pallet_x(gap))
    for name, result in tips(renders(gap)).items():
        ray = mpe.tip_visibility([*TRUCK, *placed], SIDE[name], CAMERA, 0.175)
        assert ray == 1.0
        assert result["total"] == 25 and result["confirmed"]
        assert result["fraction"] == 1.0


def test_boundary_samples_stay_in_the_denominator_and_invalid_unconfirms():
    r = renders(0.30)
    labels = A.region_labels(**r)
    everywhere = np.ones(labels.shape, dtype=bool)
    left = tips(r, labels, everywhere)["left"]
    assert left["on_boundary"] == 25 and left["total"] == 25
    broken = labels.copy()
    broken[labels == A.CARRIAGE] = A.INVALID
    left = tips(r, broken)["left"]
    assert not left["confirmed"] and left["seen"] == 0 and left["total"] == 25


def test_a_tip_hidden_by_the_pallet_is_not_seen():
    # Deeply inserted: the CPU continuous-ray rule says the tips are hidden.
    gap = -0.20
    placed = scene_rig.place(scene_rig.pallet(GEOMETRY), x_m=pallet_x(gap))
    ray = mpe.tip_visibility([*TRUCK, *placed], SIDE["left"], CAMERA, 0.175)
    assert ray == 0.0
    assert tips(renders(gap))["left"]["fraction"] == 0.0


def test_a_one_cell_lag_fails_freshness():
    """Codex's counterexample: a frame one grid cell stale must not pass."""
    before, now = depth(0.30), depth(0.31)
    pixels = A.reference_pixels(now, before)
    assert len(pixels["changed"]) > 0 and len(pixels["static"]) > 0
    assert A.freshness(now, now, pixels, needs_change=True)["ok"]
    stale = A.freshness(before, now, pixels, needs_change=True)
    assert not stale["ok"]
    assert stale["changed"]["over_tolerance"] == stale["changed"]["count"]
    assert stale["static"]["over_tolerance"] == 0


def test_a_hide_that_did_not_reach_the_render_fails_freshness():
    shown, hidden = depth(0.10), depth(0.10, carriage=False)
    pixels = A.reference_pixels(hidden, shown)
    # The carriage changes thousands of pixels even where the fixed probes do not.
    assert len(pixels["changed"]) > 0
    assert not A.freshness(shown, hidden, pixels, needs_change=True)["ok"]
    assert A.freshness(hidden, hidden, pixels, needs_change=True)["ok"]


def test_a_change_with_nothing_to_check_is_unverifiable():
    now, before = depth(0.30), depth(0.31)
    pixels = A.reference_pixels(now, before)
    assert pixels["expected_changes"] > 0
    pixels["changed"] = pixels["changed"][:0]  # every changed pixel on an edge
    report = A.freshness(now, now, pixels, needs_change=True)
    assert report["ok"] and report["unverifiable"]


def test_hiding_a_part_that_is_already_hidden_is_checked_by_static_pixels():
    # Inserted 0.20 m: the pallet hides the whole carriage from this camera.
    shown, hidden = depth(-0.20), depth(-0.20, carriage=False)
    pixels = A.reference_pixels(hidden, shown)
    assert pixels["expected_changes"] == 0 and len(pixels["changed"]) == 0
    report = A.freshness(hidden, hidden, pixels, needs_change=True)
    assert report["ok"] and not report["unverifiable"]
    assert not A.freshness(depth(-0.19), hidden, pixels, needs_change=True)["ok"]


def test_a_far_floor_is_not_an_edge():
    empty = depth(0.30, carriage=False, pallet=False)
    edges = A.depth_edges(empty)
    # Only the image border and the floor-wall crease.
    assert edges.mean() < 0.02
    assert not edges[300, 320]


def test_a_missing_reference_pixel_fails():
    now = depth(0.30)
    pixels = A.reference_pixels(now, None)
    row, col = pixels["static"][0]
    broken = now.copy()
    broken[row, col] = np.inf
    assert not A.freshness(broken, now, pixels, needs_change=False)["ok"]


@pytest.mark.parametrize("quantize", [True, False])
def test_detector_depth_is_the_rig_order(quantize):
    boxes = [*TRUCK, *scene_rig.place(scene_rig.pallet(GEOMETRY), x_m=pallet_x(0.2))]
    raw = scene_rig.render(boxes, camera=CAMERA, quantize=False, intrinsics=K).depth_m
    rig = scene_rig.render(
        boxes, camera=CAMERA, quantize=quantize, min_range_m=0.175, intrinsics=K
    ).depth_m
    np.testing.assert_array_equal(
        A.detector_depth(np.nan_to_num(raw, nan=np.inf), quantize=quantize), rig
    )


def test_difference_stats_split_regions_and_boundaries():
    r = renders(0.30)
    labels = A.region_labels(**r)
    boundary = A.boundary_mask(labels)
    stats = A.difference_stats(r["full"], r["full"] + 0.001, labels, boundary)
    for name in ("carriage", "pallet", "other"):
        interior, every = stats[f"{name}_interior"], stats[f"{name}_all"]
        assert 0 < interior["pixels"] < every["pixels"]
        assert math.isclose(interior["median_m"], -0.001, abs_tol=1e-9)


def test_a_boundary_is_claimed_only_across_confirmed_cells():
    assert A.nearest_claim([(0.1, False), (0.2, True), (0.3, True)]) == {
        "first_true": 0.2,
        "claimed": 0.2,
        "unconfirmed_up_to": None,
    }
    blocked = A.nearest_claim([(0.1, None), (0.2, False), (0.3, True)])
    assert blocked == {"first_true": 0.3, "claimed": None, "unconfirmed_up_to": 0.1}
    assert A.nearest_claim([(0.2, False), (0.1, None)])["first_true"] is None
    assert A.runs([(0.3, True), (0.1, None), (0.2, False)]) == "?.+"


def test_a_partial_grid_cannot_claim_a_boundary():
    """Codex's counterexample: three successful cells out of 91 claimed 0.30."""
    cells = [(0.30, True), (0.40, True), (0.50, True)]
    grid = A.face_gap_grid()
    result = A.nearest_claim(cells, grid=grid)
    assert result["claimed"] is None and result["first_true"] == 0.30
    assert result["unconfirmed_up_to"] == 0.29
    full = [(g, g >= 0.30) for g in grid]
    assert A.nearest_claim(full, grid=grid)["claimed"] == 0.30
    # A penetrating cell is not a pose and does not block the claim.
    without = [(g, v) for g, v in full if g != -0.20]
    assert A.nearest_claim(without, grid=grid, excluded=[-0.20])["claimed"] == 0.30


def test_restoring_one_object_and_hiding_another_are_checked_apart():
    """Codex's counterexample: at 0.70 a pallet hide that never reached the render
    passed because every changed pixel belonged to the carriage restore."""
    restored, hidden = depth(0.70), depth(0.70, pallet=False)
    pixels = A.reference_pixels(hidden, restored)
    assert len(pixels["changed"]) > 0
    assert not A.freshness(restored, hidden, pixels, needs_change=True)["ok"]


def test_pallet_pose_must_be_upright_on_the_floor():
    matrix = np.eye(4)
    matrix[:3, 3] = (1.5, 0.1, 0.0)
    assert A.pallet_pose_from_matrix(matrix) == pytest.approx((1.5, 0.1, 0.0))
    matrix[2, 3] = 0.015
    with pytest.raises(ValueError, match="upright"):
        A.pallet_pose_from_matrix(matrix)


def test_box_sets_match_regardless_of_order():
    boxes = [((0, 0, 0), (1, 1, 1)), ((1, 2, 3), (0.1, 0.2, 0.3))]
    assert A.boxes_match(boxes[::-1], boxes)
    assert not A.boxes_match(boxes[:1], boxes)
    assert not A.boxes_match([boxes[0], ((1, 2, 3.01), (0.1, 0.2, 0.3))], boxes)


def test_a_change_below_the_probe_threshold_is_unverifiable_not_identical():
    """Codex's counterexample: 4 mm on nine pixels moved the 2 mm mask but passed."""
    before = np.full((9, 9), 1.0)
    now = before.copy()
    now[3:6, 3:6] -= 0.004
    pixels = A.reference_pixels(now, before)
    assert pixels["expected_changes"] == 9 and len(pixels["changed"]) == 0
    report = A.freshness(before, now, pixels, needs_change=True)
    assert report["unverifiable"]
