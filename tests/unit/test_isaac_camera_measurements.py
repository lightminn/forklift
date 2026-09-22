"""G1 quantitative measurement contracts, without importing the simulator."""

import importlib.util
import json
import runpy
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.sensors.rgbd import PinholeIntrinsics

ROOT = Path(__file__).resolve().parents[2]
K = PinholeIntrinsics(640, 480, 465.741156, 465.741156, 320.0, 240.0, "optical")

# Independent v11 contract, never derived from the implementation's constants.
V11_LIMITS = {
    "focal_relative": 0.0012,
    "principal_point_px": 0.1,
    "intrinsics_standard_error_px": 0.1,
    "holdout_euclidean_p95_px": 0.5,
    "mount_translation_m": 0.001,
    "mount_rotation_rad": 0.00022,
    "depth_bias_m": 0.001,
    "depth_centered_p95_m": 0.002,
    "plane_normal_rad": 0.00022,
    "plane_offset_m": 0.001,
    "height_bias_m": 0.0005,
    "height_absolute_p95_m": 0.001,
    "point_reference_p95_m": 0.001,
    "point_reference_max_m": 0.002,
    "usd_getter_matrix_agreement_atol": 1e-6,
}


@pytest.fixture
def measurement():
    path = ROOT / "sim/isaac/camera_calibration.py"
    assert path.exists(), "G1 quantitative measurement module is missing"
    spec = importlib.util.spec_from_file_location("g1_measurement_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_fit_uses_design_leverage_and_actual_residuals(measurement):
    x = np.linspace(-0.687, 0.687, 100)
    y = np.sin(np.arange(100)) * 0.45
    optical = np.column_stack((x, y, np.ones(100)))
    uv = np.column_stack((K.fx * x + K.cx, K.fy * y + K.cy))
    # Orthogonalize known residuals, independently computing OLS covariance.
    noise = np.cos(np.arange(100) * 2.0) * 0.2
    a = np.column_stack((x, np.ones(100)))
    noise -= a @ np.linalg.lstsq(a, noise, rcond=None)[0]
    uv[:, 0] += noise
    holdout = np.arange(100) % 4 == 0
    fit = measurement.fit_intrinsics(optical, uv, holdout, K)
    training = a[~holdout]
    residual = (
        uv[~holdout, 0]
        - training @ np.linalg.lstsq(training, uv[~holdout, 0], rcond=None)[0]
    )
    expected = np.sqrt(
        np.diag(np.linalg.inv(training.T @ training))
        * (residual @ residual)
        / (len(training) - 2)
    )
    assert fit["standard_errors_px"]["fx"] == pytest.approx(expected[0])
    assert fit["standard_errors_px"]["cx"] == pytest.approx(expected[1])
    assert fit["gate_2a"] == "PASS"
    optical[:, 0] *= 0.02 / 0.687
    uv[:, 0] = K.fx * optical[:, 0] + K.cx + noise
    narrow = measurement.fit_intrinsics(optical, uv, holdout, K)
    assert narrow["standard_errors_px"]["fx"] > 1
    assert narrow["gate_2a"] == "FAIL"


def test_holdout_is_not_used_to_fit_and_error_is_euclidean(measurement):
    xy = np.array([(x, y) for x in [-0.5, 0, 0.5] for y in [-0.4, 0, 0.4]])
    points = np.column_stack((xy, np.ones(len(xy))))
    uv = xy * K.fx + [320, 240]
    holdout = np.arange(len(xy)) % 3 == 1
    uv[holdout] += 0.4
    fit = measurement.fit_intrinsics(points, uv, holdout, K)
    assert fit["estimated"]["cx"] == pytest.approx(320)
    assert fit["heldout_p95_px"] == pytest.approx(np.sqrt(0.32))
    assert fit["gate_2b"] == "FAIL"


def test_semantics_resolves_arbitrary_ids_and_missing_labels_fail(measurement):
    payload = {
        "data": np.array([[0, 71], [908, 71]], dtype=np.uint32),
        "info": {
            "idToLabels": {
                "0": {"class": "BACKGROUND"},
                "71": {"class": "calibration_board"},
                "908": {"class": ["other", "calibration_board"]},
            }
        },
    }
    assert measurement.semantic_mask(payload, "calibration_board").tolist() == [
        [False, True],
        [True, True],
    ]
    with pytest.raises(ValueError, match="label"):
        measurement.semantic_mask(payload, "missing")


def test_subpixel_depth_interpolates_without_rounding_and_rejects_edges(measurement):
    v, u = np.indices((8, 8))
    depth = 2 + 0.002 * u + 0.003 * v
    mask = np.ones((8, 8), dtype=bool)
    uv = np.array([[2.25, 3.75]])
    assert measurement.sample_depth_bilinear(depth, uv, mask)[0] == pytest.approx(
        2.01575
    )
    mask[4, 3] = False
    assert np.isnan(measurement.sample_depth_bilinear(depth, uv, mask)[0])


def test_unconstrained_plane_detects_tilt_and_uses_base_origin(measurement):
    truth = np.array([(2.0, y, z) for y in [-0.4, 0, 0.4] for z in [-0.3, 0, 0.3]])
    reconstructed = truth.copy()
    reconstructed[:, 0] += 0.002 * truth[:, 2]
    report = measurement.plane_metrics(reconstructed, truth)
    assert report["normal_angle_rad"] == pytest.approx(np.arctan(0.002))
    assert report["status"] == "FAIL"
    # Sign convention and offset, not the biased visible centroid.
    assert measurement.plane_metrics(truth[::-1], truth)["status"] == "PASS"
    reconstructed = truth.copy()
    reconstructed[:, 0] += 0.0008 + 0.0002 * truth[:, 1]
    origin = measurement.plane_metrics(reconstructed, truth)
    moved = measurement.plane_metrics(reconstructed - [0, 2, 0], truth - [0, 2, 0])
    assert origin["status"] == "PASS"
    assert moved["offset_error_m"] > 0.001
    assert moved["status"] == "FAIL"


def test_mount_read_always_normalizes_ros_and_checks_usd_path(measurement):
    class Camera:
        def get_local_pose(self, camera_axes):
            assert camera_axes == "ros"
            return np.array([0.75, 0, 0.5]), np.array([0.5, -0.5, 0.5, -0.5])

    expected = np.array(
        [[0, 0, 1, 0.75], [-1, 0, 0, 0], [0, -1, 0, 0.5], [0, 0, 0, 1.0]]
    )
    usd = expected @ np.diag([1, -1, -1, 1])
    mount, record = measurement.read_mount(Camera(), np.eye(4), usd.T, expected)
    np.testing.assert_allclose(mount, expected)
    assert record["status"] == "PASS"
    with pytest.raises(ValueError, match="paths disagree"):
        measurement.read_mount(Camera(), np.eye(4), np.eye(4), expected)
    wrong = expected.copy()
    wrong[0, 3] += 0.0011
    assert (
        measurement.read_mount(Camera(), np.eye(4), usd.T, wrong)[1]["status"] == "FAIL"
    )


def test_bin_distance_uses_actual_sample_hypot_and_empty_never_passes(measurement):
    points = np.array([[1.75, 0, 0], [1.75, 1.0, 0], [5.75, 0, 0], [5.76, 0, 0]])
    uv = np.array([[320, 240], [630, 240], [320, 240], [320, 240]])
    indices = measurement.sample_bins(points, uv, np.array([0.75, 0, 0.5]), K)
    assert indices[:, 0].tolist() == [1, 1, 5, -1]
    assert indices[:, 1].tolist() == [1, 2, 1, 1]
    assert measurement.error_metrics([], kind="depth")["status"] == "UNOBSERVED"
    assert measurement.error_metrics([0.004] * 30, kind="depth")["status"] == "FAIL"
    assert measurement.error_metrics([-0.004] * 30, kind="depth")["status"] == "FAIL"
    assert measurement.error_metrics([0.0006] * 30, kind="height")["status"] == "FAIL"
    assert (
        measurement.error_metrics([0.003] * 30, kind="point")["status"] == "RECORD_ONLY"
    )


def test_visibility_varies_with_column_at_same_horizontal_range(measurement):
    mount = RigidTransform(
        "optical", "base_link", [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], [0.75, 0, 0.5]
    )
    central = measurement.height_visibility(1.0, 320.0, 0.022, K, mount)
    edge = measurement.height_visibility(1.0, 630.0, 0.022, K, mount)
    near = measurement.height_visibility(0.8, 320.0, 0.022, K, mount)
    assert central["visible"]
    assert central["v_px"] == pytest.approx(462.62427, abs=0.001)
    assert not edge["visible"] and edge["v_px"] > 507
    assert not near["visible"]


def test_height_truth_comes_from_composed_surface_vertices_not_world_ground(
    measurement,
):
    base = np.eye(4)
    base[2, 3] = -0.013
    world_surface = np.array(
        [[1, 0, 0.009], [2, 0, 0.009], [2, 1, 0.009], [1, 1, 0.009]]
    )
    assert measurement.surface_height_base(world_surface, base) == pytest.approx(0.022)
    world_surface[0, 2] += 0.01
    with pytest.raises(ValueError, match="horizontal"):
        measurement.surface_height_base(world_surface, base)


def test_static_capture_uses_one_product_and_zero_time_step(measurement):
    from types import SimpleNamespace

    events = []

    class Annotator:
        def __init__(self, name):
            self.name = name

        def attach(self, products):
            events.append((self.name, products))

        def get_data(self):
            return np.array([1, 2, 3])

    def get_annotator(name, **kwargs):
        if name == "semantic_segmentation":
            assert kwargs == {"init_params": {"colorize": False}}
        return Annotator(name)

    def step(**kwargs):
        events.append(("step", kwargs))

    rep = SimpleNamespace(
        AnnotatorRegistry=SimpleNamespace(get_annotator=get_annotator),
        orchestrator=SimpleNamespace(
            set_capture_on_play=lambda value: events.append(("play", value)), step=step
        ),
    )
    camera = SimpleNamespace(get_render_product_path=lambda: "/Render/Product")
    annotators = measurement.attach_annotators(rep, camera)
    frame = measurement.capture_static(rep, annotators)
    assert events == [
        ("rgb", ["/Render/Product"]),
        ("semantic_segmentation", ["/Render/Product"]),
        ("distance_to_image_plane", ["/Render/Product"]),
        ("play", False),
        ("step", {"rt_subframes": 4, "delta_time": 0.0, "pause_timeline": True}),
    ]
    assert set(frame) == {"rgb", "seg", "z"}


def test_checkerboard_measurement_keeps_subpixel_coordinates(measurement):
    import cv2

    rgb = np.full((480, 640, 3), 255, dtype=np.uint8)
    for i in range(8):
        for j in range(10):
            rgb[160 + 16 * i : 176 + 16 * i, 240 + 16 * j : 256 + 16 * j] = (
                0 if (i + j) % 2 == 0 else 255
            )
    mask = np.zeros((480, 640), dtype=bool)
    mask[140:308, 220:420] = True
    uv = measurement.checkerboard_corners(rgb, mask)
    assert uv.shape == (63, 2)
    assert uv[0] == pytest.approx([255.5, 175.5], abs=0.12)
    assert np.abs(uv - np.rint(uv)).max() > 0.3
    with pytest.raises(ValueError, match="corners"):
        measurement.checkerboard_corners(np.full_like(rgb, 255), mask)
    assert cv2.CALIB_CB_ACCURACY > 0


def test_board_group_gates_do_not_hide_local_bias_or_gate_point_error(measurement):
    truth = np.array([(2.0, y, z) for y in [-0.3, 0, 0.3] for z in [0.1, 0.3, 0.5]])
    uv = np.array([(u, v) for u in [260, 320, 380] for v in [260, 280, 300]])
    reconstructed = truth.copy()
    reconstructed[:, 1] += 0.003  # tangent translation: plane still exact
    report = measurement.board_statistics(
        truth, reconstructed, uv, np.zeros(9), np.array([0.75, 0, 0.5]), K
    )
    assert report["gate_7a"] == "PASS"
    assert report["point"]["status"] == "RECORD_ONLY"
    assert report["point"]["absolute_p95_m"] > 0.002
    bad = measurement.board_statistics(
        truth, reconstructed, uv, np.full(9, 0.004), np.array([0.75, 0, 0.5]), K
    )
    assert bad["gate_4"] == "FAIL"


def test_height_grid_fixes_samples_before_depth_and_rejects_missing(measurement):
    mount = np.array([[0, 0, 1, 0.75], [-1, 0, 0, 0], [0, -1, 0, 0.5], [0, 0, 0, 1.0]])
    surface = np.array(
        [
            [2.35, -0.32, 0.022],
            [3.55, -0.32, 0.022],
            [3.55, 0.32, 0.022],
            [2.35, 0.32, 0.022],
        ]
    )
    # Both distance bins retain an interior after selection's additional margin.
    selection = measurement.height_grid_selection(surface, K, mount)
    assert len(selection["uv"]) > 20
    assert selection["uv"].dtype.kind in "iu"
    # A physically correct pinhole rendering of the plane, with one hole.
    v, u = np.indices((480, 640))
    depth = np.full((480, 640), np.nan)
    depth[v > 240] = (0.5 - 0.022) * K.fy / (v[v > 240] - 240)
    mask = np.ones((480, 640), dtype=bool)
    good = measurement.height_grid_statistics(selection, depth, mask, K, mount, 0.022)
    assert good["status"] == "PASS"
    u0, v0 = selection["uv"][0]
    depth[v0, u0] = np.nan
    bad = measurement.height_grid_statistics(selection, depth, mask, K, mount, 0.022)
    assert bad["status"] == "FAIL"
    assert bad["missing_count"] == 1


def _horizontal_image_rectangle(bounds):
    # A downward camera one metre above z=0: optical Z=1, base x=(u-cx)/fx,
    # base y=-(v-cy)/fy. Literal pixel bounds define independent edge truth.
    k = PinholeIntrinsics(640, 480, 100.0, 100.0, 319.5, 239.5, "optical")
    mount = np.diag([1.0, -1.0, -1.0, 1.0])
    mount[2, 3] = 1.0
    left, right, top, bottom = bounds
    surface = np.array(
        [
            [(u - 319.5) / 100, -(v - 239.5) / 100, 0.0]
            for u, v in [(left, top), (right, top), (right, bottom), (left, bottom)]
        ]
    )
    return k, mount, surface


@pytest.mark.parametrize(
    "dx,dy", [(-0.1442, 0), (0.1442, 0), (0, -0.1442), (0, 0.1442)]
)
def test_height_selection_survives_subpixel_boundary_shift(measurement, dx, dy):
    # Returning selection erosion to evaluation's 2 px makes this fail: an
    # integer boundary row/column flips even though the shift is below 1 px.
    k, mount, surface = _horizontal_image_rectangle((410.01, 425.99, 230.01, 249.99))
    fitted = replace(k, cx=k.cx + dx, cy=k.cy + dy)
    selection = measurement.height_grid_selection(surface, fitted, mount)
    raw_mask = np.zeros((480, 640), dtype=bool)
    raw_mask[231:250, 411:426] = True
    evaluation = measurement.erode_mask(raw_mask)
    expected = np.zeros_like(raw_mask)
    expected[233:248, 413:424] = True
    np.testing.assert_array_equal(evaluation, expected)  # evaluation stays 2 px
    u, v = selection["uv"].T
    assert len(u) == 20
    assert evaluation[v, u].all()
    stats = measurement.height_grid_statistics(
        selection, np.ones((480, 640)), evaluation, k, mount, 0.0
    )
    assert stats["missing_count"] == 0
    assert stats["status"] == "PASS"


def test_height_selection_records_margin_and_reduced_sample_count(measurement):
    k, mount, surface = _horizontal_image_rectangle((409.9, 426.1, 229.9, 237.1))
    selection = measurement.height_grid_selection(surface, k, mount)
    # Raw integer rectangle [410..426] x [230..237]: 2 px gives 7*2
    # stride-grid centres; 3 px gives 5*1. No depth is supplied to selection.
    assert len(selection["uv"]) == 5
    assert selection["coverage"] == {
        "status": "COMPLETE",
        "baseline_sample_count": 14,
        "sample_count": 5,
        "removed_sample_count": 9,
        "lost_bins": [],
    }
    group = next(g for g in selection["groups"] if g["planned_count"])
    assert group["baseline_geometric_count"] == group["baseline_planned_count"] == 14
    assert group["geometric_count"] == group["required_count"] == 5
    margin = selection["margin"]
    assert margin["principal_point_bound_px"] == 0.1442
    assert margin["additional_selection_erosion_px"] == 1
    assert margin["evaluation_erosion_px"] == 2
    assert margin["selection_erosion_px"] == 3
    assert measurement.protocol()["height_selection_margin"] == margin


def test_height_margin_lost_bin_is_a_separate_coverage_failure(measurement):
    k, mount, surface = _horizontal_image_rectangle((410.01, 426.99, 229.9, 234.1))
    selection = measurement.height_grid_selection(surface, k, mount)
    # Five rows support 2 px erosion and six stride samples - exactly the
    # predeclared baseline floor; 3 px erases the whole panel. This is lost
    # required coverage, not UNOBSERVED/PASS.
    assert measurement.MIN_COVERAGE_BASELINE_SAMPLES == 6
    assert len(selection["uv"]) == 0
    assert selection["coverage"] == {
        "status": "FAIL",
        "baseline_sample_count": 6,
        "sample_count": 0,
        "removed_sample_count": 6,
        "lost_bins": [[1, 1, 1]],
    }
    stats = measurement.height_grid_statistics(
        selection, np.ones((480, 640)), np.ones((480, 640), dtype=bool), k, mount, 0.0
    )
    assert stats["status"] == stats["coverage_status"] == "FAIL"
    assert stats["missing_count"] == 0
    assert stats["lost_bins"] == [[1, 1, 1]]


def test_a_lost_bin_below_the_baseline_floor_is_recorded_and_exempt(measurement):
    # User decision, 2026-09-21: a bin whose baseline holds fewer samples than
    # the floor cannot carry a statistic, so losing it is recorded with its
    # count instead of failing the gate. One pixel at the image border must not
    # fail a run. The same panel one grid column wider still fails (above).
    k, mount, surface = _horizontal_image_rectangle((410.01, 425.99, 229.9, 234.1))
    selection = measurement.height_grid_selection(surface, k, mount)
    assert len(selection["uv"]) == 0
    assert selection["coverage"] == {
        "status": "UNOBSERVED",
        "baseline_sample_count": 5,
        "sample_count": 0,
        "removed_sample_count": 5,
        "lost_bins": [],
    }
    # A reclassification, not a deletion: the bin and its baseline stay visible.
    group = next(g for g in selection["groups"] if g["baseline_geometric_count"])
    assert group["bin"] == [1, 1, 1]
    assert group["coverage_status"] == "BELOW_BASELINE_FLOOR"
    assert group["baseline_geometric_count"] == 5
    assert group["geometric_count"] == 0
    stats = measurement.height_grid_statistics(
        selection, np.ones((480, 640)), np.ones((480, 640), dtype=bool), k, mount, 0.0
    )
    assert stats["status"] == stats["coverage_status"] == "UNOBSERVED"
    assert stats["lost_bins"] == []
    measured = next(g for g in stats["groups"] if g["bin"] == [1, 1, 1])
    assert measured["coverage_status"] == "BELOW_BASELINE_FLOOR"
    assert measured["baseline_geometric_count"] == 5


def test_the_coverage_baseline_floor_is_predeclared_in_the_protocol(measurement):
    # Its own constant, so it can diverge from the plane-fit minimum later.
    record = measurement.protocol()
    assert (
        record["minimum_coverage_baseline_samples"]
        == measurement.MIN_COVERAGE_BASELINE_SAMPLES
        == 6
    )
    assert "minimum_coverage_baseline_samples" in record["grid_selection"]


def test_height_margin_lost_bin_fails_even_with_other_samples(measurement):
    mount = np.array([[0, 0, 1, 0.75], [-1, 0, 0, 0], [0, -1, 0, 0.5], [0, 0, 0, 1.0]])
    surface = np.array(
        [
            [2.35, -0.32, 0.022],
            [3.35, -0.32, 0.022],
            [3.35, 0.32, 0.022],
            [2.35, 0.32, 0.022],
        ]
    )
    selection = measurement.height_grid_selection(surface, K, mount)
    assert len(selection["uv"]) == 20
    assert selection["coverage"]["lost_bins"] == [[3, 1, 2]]
    v, _ = np.indices((480, 640))
    depth = np.ones((480, 640))
    depth[v > 240] = (0.5 - 0.022) * K.fy / (v[v > 240] - 240)
    stats = measurement.height_grid_statistics(
        selection, depth, np.ones((480, 640), dtype=bool), K, mount, 0.022
    )
    assert stats["missing_count"] == 0
    assert stats["per_distance"][2]["status"] == "PASS"
    assert stats["status"] == stats["coverage_status"] == "FAIL"


CANONICAL_MOUNT = np.array(
    [[0, 0, 1, 0.75], [-1, 0, 0, 0], [0, -1, 0, 0.5], [0, 0, 0, 1.0]]
)
# Predeclared measurement rectangle, restated here instead of imported so a
# change to the implementation's half extents fails rather than follows.
MEASUREMENT_HALF_EXTENTS = (0.20, 0.16)
# The distance bins that section 9.6 of the run record shows the selection
# margin emptying, reproduced with this file's K.
LOST_BINS_AT_R3_H22_U320 = [[2, 1, 2], [4, 1, 1]]


def _panel_centre(measurement, distance, height, column):
    mount = RigidTransform(
        "optical", "base_link", CANONICAL_MOUNT[:3, :3], CANONICAL_MOUNT[:3, 3]
    )
    return measurement.height_visibility(distance, column, height, K, mount)[
        "point_base_m"
    ]


def _measurement_rectangle(centre, distance, height):
    forward, lateral = (half * distance for half in MEASUREMENT_HALF_EXTENTS)
    return np.array(
        [
            [centre[0] - forward, centre[1] - lateral, height],
            [centre[0] + forward, centre[1] - lateral, height],
            [centre[0] + forward, centre[1] + lateral, height],
            [centre[0] - forward, centre[1] + lateral, height],
        ]
    )


def test_height_panel_margin_restores_the_lost_distance_bin_rows(measurement):
    distance, height, column = 3.0, 0.022, 320.0
    centre = _panel_centre(measurement, distance, height, column)
    measured = _measurement_rectangle(centre, distance, height)
    # Without margin geometry the panel ends inside the extreme distance bins,
    # which then hold a single stride-grid row that the 3 px selection erases.
    bare = measurement.height_grid_selection(measured, K, CANONICAL_MOUNT)
    assert bare["coverage"]["lost_bins"] == LOST_BINS_AT_R3_H22_U320

    geometry = measurement.height_panel_geometry(
        centre, distance, height, K, CANONICAL_MOUNT
    )
    panel = np.asarray(geometry["panel_vertices_base_m"])
    region = np.asarray(geometry["measurement_vertices_base_m"])
    # The measurement region is untouched: same rectangle, same horizontal z.
    np.testing.assert_allclose(region, measured, rtol=0, atol=1e-12)
    # Only the viewing axis grows; lateral extent and height are identical.
    np.testing.assert_allclose(panel[:, 1], region[:, 1], rtol=0, atol=1e-12)
    np.testing.assert_allclose(panel[:, 2], region[:, 2], rtol=0, atol=1e-12)
    assert panel[:, 0].min() < region[:, 0].min()
    assert panel[:, 0].max() > region[:, 0].max()

    selection = measurement.height_grid_selection(
        panel, K, CANONICAL_MOUNT, measurement_vertices_base=region
    )
    assert selection["coverage"]["lost_bins"] == []
    assert selection["coverage"]["status"] == "COMPLETE"
    # Every previously emptied bin now holds at least two stride-2 grid rows.
    for lost in LOST_BINS_AT_R3_H22_U320:
        rows = selection["uv"][(selection["bins"] == lost).all(axis=1)][:, 1]
        assert len(set(rows.tolist())) >= 2, lost
    # Selection never leaves the measurement region, so the margin adds no
    # measured height and no new bin.
    truth = selection["truth_base_m"]
    assert (truth[:, 0] >= region[:, 0].min() - 1e-9).all()
    assert (truth[:, 0] <= region[:, 0].max() + 1e-9).all()
    assert selection["panel_region"] == {
        "measurement_vertices_base_m": region.tolist(),
        "panel_vertices_base_m": panel.tolist(),
        "selection_confined_to_measurement_region": True,
    }
    # The restored samples come from the measurement region's own edge rows,
    # which the margin takes out of the kernel's reach.
    assert len(selection["uv"]) > len(bare["uv"])


def test_height_panel_margin_is_the_derived_row_shift_not_a_constant(measurement):
    # Evaluation 2 px, predeclared selection-only 1 px, and 1 px because an
    # integer row survives erosion only once ceil() has moved a whole row.
    assert measurement.height_selection_margin()["selection_erosion_px"] == 3
    rows = 4
    for distance, height in [(3.0, 0.022), (5.0, 0.144), (2.0, 0.0)]:
        centre = _panel_centre(measurement, distance, height, 320.0)
        geometry = measurement.height_panel_geometry(
            centre, distance, height, K, CANONICAL_MOUNT
        )
        # v = fy*Y/Z + cy with Y = camera height above the panel: the metres
        # that buy `rows` pixels follow from Z alone, independently derived.
        optical_y = CANONICAL_MOUNT[2, 3] - height
        forward = MEASUREMENT_HALF_EXTENTS[0] * distance
        near_z = centre[0] - forward - CANONICAL_MOUNT[0, 3]
        far_z = centre[0] + forward - CANONICAL_MOUNT[0, 3]
        assert geometry["near_margin_m"] == pytest.approx(
            rows * near_z**2 / (K.fy * optical_y + rows * near_z), rel=1e-9
        )
        assert geometry["far_margin_m"] == pytest.approx(
            rows * far_z**2 / (K.fy * optical_y - rows * far_z), rel=1e-9
        )
        # The margin is a distance-dependent derivation, never a fixed span.
        assert geometry["far_margin_m"] > geometry["near_margin_m"] > 0
    assert "margin" in measurement.protocol()["height_panel_size_m"]


def test_the_margin_records_plus_one_as_the_constant_it_is(measurement):
    # ceil() returns 1 for every envelope up to a whole pixel, so the measured
    # 0.1442 px does not size the margin; it only shows the envelope is below
    # 1. The recorded strings must say so instead of reading like a scaling.
    original = measurement.HEIGHT_PRINCIPAL_POINT_BOUND_PX
    try:
        for envelope in (1e-9, 0.05, 0.1442, 0.5, 0.9999, 1.0):
            measurement.HEIGHT_PRINCIPAL_POINT_BOUND_PX = envelope
            margin = measurement.height_selection_margin()
            assert margin["additional_selection_erosion_px"] == 1
            assert margin["selection_erosion_px"] == 2 + 1
    finally:
        measurement.HEIGHT_PRINCIPAL_POINT_BOUND_PX = original
    margin = measurement.height_selection_margin()
    assert margin["principal_point_bound_px"] == 0.1442
    derivation = margin["derivation"]
    assert "(0, 1]" in derivation
    assert "below 1" in derivation
    geometry = measurement.height_panel_geometry(
        _panel_centre(measurement, 3.0, 0.022, 320.0),
        3.0,
        0.022,
        K,
        CANONICAL_MOUNT,
    )
    assert geometry["margin_rows_px"] == 4
    assert "constant" in geometry["derivation"]


def test_the_protocol_comment_names_only_versions_that_existed(measurement):
    # The executed runs recorded G1-v11-1 and then G1-v11-3; no v11-2 was ever
    # authored or run, so nothing may describe the change as coming from it.
    assert measurement.protocol()["version"] == "G1-v11-3"
    source = (ROOT / "sim/isaac/camera_calibration.py").read_text(encoding="utf-8")
    assert "v11-2" not in source
    assert "v11-1" in source


def test_height_selection_never_samples_the_rendered_margin(measurement):
    # A panel far larger than its measurement region: without confinement the
    # erosion clears the margin and the grid walks straight out of the region.
    k, mount, panel = _horizontal_image_rectangle((360.01, 479.99, 180.01, 299.99))
    _, _, region = _horizontal_image_rectangle((400.01, 439.99, 220.01, 259.99))
    confined = measurement.height_grid_selection(
        panel, k, mount, measurement_vertices_base=region
    )
    unconfined = measurement.height_grid_selection(panel, k, mount)
    u, v = confined["uv"].T
    assert len(u)
    assert u.min() >= 401 and u.max() <= 439
    assert v.min() >= 221 and v.max() <= 259
    assert unconfined["uv"][:, 0].min() < 401
    assert confined["panel_region"]["selection_confined_to_measurement_region"]
    assert not unconfined["panel_region"]["selection_confined_to_measurement_region"]
    # The rendered panel, not the measurement region, supplies both erosions,
    # so the region's own edge rows stay eligible and the coverage rule keeps
    # comparing the same two kernels.
    assert confined["coverage"]["baseline_sample_count"] == len(u)
    assert confined["coverage"]["removed_sample_count"] == 0
    with pytest.raises(ValueError, match="inside the rendered panel"):
        measurement.height_grid_selection(
            region, k, mount, measurement_vertices_base=panel
        )
    tilted = region + [[0, 0, 0], [0, 0, 0], [0, 0, 0.01], [0, 0, 0]]
    with pytest.raises(ValueError, match="horizontal at the panel surface"):
        measurement.height_grid_selection(
            panel, k, mount, measurement_vertices_base=tilted
        )


def test_saved_height_selection_reconstructs_fitted_k_and_coverage(
    measurement, tmp_path
):
    k, mount, surface = _horizontal_image_rectangle((409.9, 426.1, 229.9, 237.1))
    fitted = replace(k, fx=100.01, cx=319.4293, cy=239.4701)
    selection = measurement.height_grid_selection(surface, fitted, mount)
    assert selection.get("selection_K") == asdict(fitted)
    # Recording the fitted K while secretly snapping the selection rays to a
    # nominal centre must also fail, even when stride-grid coordinates agree.
    u, v = selection["uv"].T
    np.testing.assert_allclose(
        selection["truth_base_m"],
        np.column_stack(
            ((u - 319.4293) / 100.01, -(v - 239.4701) / 100, np.zeros(len(u)))
        ),
        rtol=0,
        atol=1e-12,
    )
    save = runpy.run_path(str(ROOT / "sim/isaac/verify_perception_camera.py"))[
        "save_height_selection"
    ]
    reference = {
        "collection": "intrinsics_fits",
        "index": 12,
        "anchor_horizontal_m": 4.0,
        "repeat": 0,
    }
    record = save(tmp_path, "height_test", selection, reference)
    saved = json.loads((tmp_path / "height_test_selection.json").read_text())
    assert saved == record
    assert saved["selection_K"] == asdict(fitted)
    assert saved["selection_fit_reference"] == reference
    assert saved["selection_coverage"] == selection["coverage"]
    assert saved["selection_groups"] == selection["groups"]
    rebuilt = measurement.height_grid_selection(
        surface, PinholeIntrinsics(**saved["selection_K"]), mount
    )
    with np.load(tmp_path / "height_test_selection.npz") as arrays:
        for field in ("uv", "truth_base_m", "bins"):
            np.testing.assert_array_equal(arrays[field], rebuilt[field])


def test_checkerboard_layout_covers_screen_and_uses_horizontal_centres(measurement):
    mount = np.array([[0, 0, 1, 0.75], [-1, 0, 0, 0], [0, -1, 0, 0.5], [0, 0, 0, 1.0]])
    corners = []
    for centre in measurement.SCREEN_CENTRES_UV:
        geometry = measurement.checkerboard_layout(2.0, centre, K, mount)
        points = geometry["vertices_base_m"][geometry["corner_indices"]]
        corners.extend(points)
        assert np.linalg.norm(points[31, :2] - mount[:2, 3]) == pytest.approx(2.0)
    optical = measurement.transform_points(np.linalg.inv(mount), np.array(corners))
    uv = measurement.project(optical, K)
    assert np.ptp(uv[:, 0]) > 560
    assert np.ptp(uv[:, 1]) > 410


def test_marker_centroids_compare_semantic_identity_with_global_highlights(measurement):
    rgb = np.zeros((100, 100, 3), dtype=np.uint8)
    mask = np.zeros((100, 100), dtype=bool)
    mask[45:55, 45:55] = True
    rgb[mask] = 255
    rgb[10:30, 80:90] = 255
    report = measurement.marker_centroids(rgb, mask, np.array([49.5, 49.5]), 1.25, K)
    assert report["semantic_centroid_uv"] == [49.5, 49.5]
    assert report["bright_centroid_uv"][0] > 70
    assert report["bright_centroid_outside_marker_bbox"]
    assert report["centroid_difference_m"][0] > 0.05


def test_far_central_board_has_noncollinear_in_range_corners(measurement):
    mount = np.array([[0, 0, 1, 0.75], [-1, 0, 0, 0], [0, -1, 0, 0.5], [0, 0, 0, 1.0]])
    geometry = measurement.checkerboard_layout(5.0, (320.0, 240.0), K, mount)
    truth = geometry["vertices_base_m"][geometry["corner_indices"]]
    points = measurement.transform_points(np.linalg.inv(mount), truth)
    uv = measurement.project(points, K)
    report = measurement.board_statistics(
        truth, truth, uv, np.zeros(63), mount[:3, 3], K
    )
    assert report["gate_7a"] == "PASS"
    assert np.linalg.norm(truth[31, :2] - mount[:2, 3]) == pytest.approx(5.0)


def test_empty_height_selection_is_unobserved_not_crash_or_pass(measurement):
    mount = np.array([[0, 0, 1, 0.75], [-1, 0, 0, 0], [0, -1, 0, 0.5], [0, 0, 0, 1.0]])
    surface = np.array([[1.4, -0.1, 0], [1.5, -0.1, 0], [1.5, 0.1, 0], [1.4, 0.1, 0]])
    selection = measurement.height_grid_selection(surface, K, mount)
    assert len(selection["uv"]) == 0
    stats = measurement.height_grid_statistics(
        selection, np.ones((480, 640)), np.zeros((480, 640), dtype=bool), K, mount, 0.0
    )
    assert stats["status"] == "UNOBSERVED"
    assert all(group["status"] == "UNOBSERVED" for group in stats["groups"])


def test_geometric_sliver_quota_fixed_before_measuring_depth(measurement):
    from forklift_core.geometry import RigidTransform

    mount_obj = RigidTransform(
        "optical", "base_link", [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], [0.75, 0, 0.5]
    )
    mount = measurement.mount_matrix(mount_obj)
    centre = measurement.height_visibility(3.0, 100.0, 0.0, K, mount_obj)[
        "point_base_m"
    ]
    x, y, _ = centre
    surface = np.array(
        [
            [x - 0.6, y - 0.48, 0],
            [x + 0.6, y - 0.48, 0],
            [x + 0.6, y + 0.48, 0],
            [x - 0.6, y + 0.48, 0],
        ]
    )
    selection = measurement.height_grid_selection(surface, K, mount)
    slivers = [g for g in selection["groups"] if 0 < g["geometric_count"] < 20]
    assert slivers
    assert all(g["required_count"] == g["geometric_count"] for g in slivers)


@pytest.mark.parametrize("sentinel", [np.inf, -np.inf, np.nan, 0.0])
def test_invalid_height_depth_records_failure_without_numerical_warning(
    measurement, sentinel
):
    mount = np.array([[0, 0, 1, 0.75], [-1, 0, 0, 0], [0, -1, 0, 0.5], [0, 0, 0, 1.0]])
    surface = np.array(
        [
            [2.35, -0.32, 0.022],
            [3.35, -0.32, 0.022],
            [3.35, 0.32, 0.022],
            [2.35, 0.32, 0.022],
        ]
    )
    selection = measurement.height_grid_selection(surface, K, mount)
    stats = measurement.height_grid_statistics(
        selection,
        np.full((480, 640), sentinel),
        np.ones((480, 640), dtype=bool),
        K,
        mount,
        0.022,
    )
    assert stats["status"] == "FAIL"
    assert stats["missing_count"] == len(selection["uv"])


def test_height_bias_gate_uses_distance_bins_screen_cells_remain_diagnostics(
    measurement,
):
    mount = np.array([[0, 0, 1, 0.75], [-1, 0, 0, 0], [0, -1, 0, 0.5], [0, 0, 0, 1.0]])
    uv = np.array([[160, 352], [480, 352]])
    selection = {
        "uv": uv,
        "bins": np.array([[2, 0, 2], [2, 2, 2]]),
        "groups": [
            {
                "bin": [2, col, 2],
                "planned_count": 1,
                "required_count": 1,
                "geometric_count": 1,
            }
            for col in [0, 2]
        ],
    }
    depth = np.ones((480, 640))
    # Equal opposite biases within ONE distance bin, all heights within 1 mm.
    depth[352, 160] = (0.5 - 0.022 - 0.0006) * K.fy / (352 - K.cy)
    depth[352, 480] = (0.5 - 0.022 + 0.0006) * K.fy / (352 - K.cy)
    stats = measurement.height_grid_statistics(
        selection, depth, np.ones((480, 640), dtype=bool), K, mount, 0.022
    )
    assert stats["per_distance"][2]["bias_m"] == pytest.approx(0, abs=1e-12)
    assert stats["status"] == "PASS"


def test_protocol_records_the_single_v11_limit_source_without_exposing_it(measurement):
    assert getattr(measurement, "G1_LIMITS", None) == V11_LIMITS
    record = measurement.protocol()
    assert record["limits"] == V11_LIMITS
    record["limits"]["focal_relative"] = 1.0
    assert measurement.protocol()["limits"] == V11_LIMITS


@pytest.mark.parametrize("parameter", ["fx", "fy", "cx", "cy"])
@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("factor,status", [(0.999, "PASS"), (1.001, "FAIL")])
def test_fitted_intrinsics_v11_boundary_in_both_directions(
    measurement, parameter, sign, factor, status
):
    xy = np.array([(x, y) for x in [-0.5, 0, 0.5] for y in [-0.4, 0, 0.4]])
    points = np.column_stack((xy, np.ones(len(xy))))
    values = {name: getattr(K, name) for name in ("fx", "fy", "cx", "cy")}
    limit = values[parameter] * 0.0012 if parameter.startswith("f") else 0.1
    values[parameter] += sign * limit * factor
    uv = xy * [values["fx"], values["fy"]] + [values["cx"], values["cy"]]
    fit = measurement.fit_intrinsics(points, uv, np.arange(len(xy)) % 3 == 1, K)
    assert fit["estimated"][parameter] == pytest.approx(values[parameter], abs=1e-9)
    assert fit["gate_2a"] == status
    assert fit["gate_2b"] == "PASS"


@pytest.mark.parametrize("parameter", ["fx", "fy", "cx", "cy"])
@pytest.mark.parametrize("factor,status", [(0.999, "PASS"), (1.001, "FAIL")])
def test_each_intrinsic_standard_error_v11_boundary(
    measurement, parameter, factor, status
):
    # Four training points: A.T A = diag(4*s*s, 4), residual df = 2.
    # [1,-1,1,-1] is orthogonal to both design columns, so beta is unchanged.
    # s separates focal SE from centre SE to exercise each of the four gates.
    s = 0.25 if parameter.startswith("f") else 2.0
    xy = np.array([[-s, -s], [-s, -s], [s, s], [s, s], [0, 0]])
    points = np.column_stack((xy, np.ones(5)))
    uv = xy * [K.fx, K.fy] + [K.cx, K.cy]
    amplitude = 0.1 * factor * np.sqrt(2) * (s if parameter.startswith("f") else 1)
    uv[:4, 0 if parameter.endswith("x") else 1] += amplitude * np.array([1, -1, 1, -1])
    fit = measurement.fit_intrinsics(points, uv, np.array([False] * 4 + [True]), K)
    assert fit["standard_errors_px"][parameter] == pytest.approx(0.1 * factor)
    assert all(
        value < 0.1
        for name, value in fit["standard_errors_px"].items()
        if name != parameter
    )
    assert fit["gate_2a"] == status


@pytest.mark.parametrize("factor,status", [(0.999, "PASS"), (1.001, "FAIL")])
def test_holdout_euclidean_p95_v11_boundary(measurement, factor, status):
    xy = np.array([[-0.5, -0.4], [-0.5, 0.4], [0.5, -0.4], [0.5, 0.4], [0, 0]])
    points = np.column_stack((xy, np.ones(5)))
    uv = xy * [K.fx, K.fy] + [K.cx, K.cy]
    uv[-1] += 0.5 * factor / np.sqrt(2)
    fit = measurement.fit_intrinsics(points, uv, np.array([False] * 4 + [True]), K)
    assert fit["heldout_p95_px"] == pytest.approx(0.5 * factor)
    assert fit["gate_2a"] == "PASS"
    assert fit["gate_2b"] == status


@pytest.mark.parametrize(
    "component,limit", [("translation", 0.001), ("rotation", 0.00022)]
)
@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("factor,status", [(0.999, "PASS"), (1.001, "FAIL")])
def test_mount_v11_boundaries(measurement, component, limit, sign, factor, status):
    from types import SimpleNamespace

    error = sign * limit * factor
    angle = error if component == "rotation" else 0.0
    position = np.array([error if component == "translation" else 0.0, 0, 0])
    c, s = np.cos(angle), np.sin(angle)
    actual = np.array(
        [[c, -s, 0, position[0]], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    )
    camera = SimpleNamespace(
        get_local_pose=lambda camera_axes: (
            position,
            np.array([np.cos(angle / 2), 0, 0, np.sin(angle / 2)]),
        )
    )
    _, report = measurement.read_mount(
        camera, np.eye(4), (actual @ np.diag([1, -1, -1, 1])).T, np.eye(4)
    )
    assert report["status"] == status


@pytest.mark.parametrize("component,limit", [("normal", 0.00022), ("offset", 0.001)])
@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("factor,status", [(0.999, "PASS"), (1.001, "FAIL")])
def test_plane_v11_boundaries(measurement, component, limit, sign, factor, status):
    truth = np.array([(2.0, y, z) for y in [-0.4, 0, 0.4] for z in [-0.3, 0, 0.3]])
    error = sign * limit * factor
    if component == "normal":
        # Rotate about base origin: normal changes, signed plane offset stays 2 m.
        c, s = np.cos(error), np.sin(error)
        reconstructed = truth @ np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
    else:
        reconstructed = truth + [error, 0, 0]
    report = measurement.plane_metrics(reconstructed, truth)
    assert report[
        "normal_angle_rad" if component == "normal" else "offset_error_m"
    ] == pytest.approx(abs(error))
    assert report["status"] == status


@pytest.mark.parametrize(
    "kind,statistic,limit",
    [
        ("depth", "bias", 0.001),
        ("depth", "spread", 0.002),
        ("height", "bias", 0.0005),
        ("height", "p95", 0.001),
    ],
)
@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("factor,status", [(0.999, "PASS"), (1.001, "FAIL")])
def test_depth_and_height_v11_boundaries(
    measurement, kind, statistic, limit, sign, factor, status
):
    error = sign * limit * factor
    errors = (
        np.full(20, error) if statistic == "bias" else np.array([-error, error] * 10)
    )
    report = measurement.error_metrics(errors, kind=kind)
    assert report["status"] == status


@pytest.mark.parametrize(
    "u,v,visible",
    [
        pytest.param(-1e-6, 240.0, False, id="u-min-outside"),
        pytest.param(0.0, 240.0, True, id="u-min-boundary"),
        pytest.param(1e-6, 240.0, True, id="u-min-inside"),
        pytest.param(640.0 - 1e-6, 240.0, True, id="u-max-inside"),
        pytest.param(640.0, 240.0, False, id="u-max-boundary"),
        pytest.param(640.0 + 1e-6, 240.0, False, id="u-max-outside"),
        pytest.param(320.0, -1e-6, False, id="v-min-outside"),
        pytest.param(320.0, 0.0, True, id="v-min-boundary"),
        pytest.param(320.0, 1e-6, True, id="v-min-inside"),
        pytest.param(320.0, 480.0 - 1e-6, True, id="v-max-inside"),
        pytest.param(320.0, 480.0, False, id="v-max-boundary"),
        pytest.param(320.0, 480.0 + 1e-6, False, id="v-max-outside"),
    ],
)
def test_visibility_image_boundaries(measurement, u, v, visible):
    # Keep the other coordinate centred so only one image edge is under test.
    # fx=320 gives exact x/z=+/-1 at u=0/640; fy=480 gives exact v=0/480.
    k = PinholeIntrinsics(640, 480, 320.0, 480.0, 320.0, 240.0, "optical")
    mount = RigidTransform(
        "optical", "base_link", [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], [0, 0, 0.5]
    )
    height = 0.5 - (v - 240.0) / 480.0
    report = measurement.height_visibility(1.0, u, height, k, mount)
    assert report["u_px"] == pytest.approx(u, abs=1e-10, rel=0)
    assert report["v_px"] == pytest.approx(v, abs=1e-10, rel=0)
    assert report["visible"] is visible
