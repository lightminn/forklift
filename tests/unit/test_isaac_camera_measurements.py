"""G1 quantitative measurement contracts, without importing the simulator."""

import importlib.util
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
            [3.35, -0.32, 0.022],
            [3.35, 0.32, 0.022],
            [2.35, 0.32, 0.022],
        ]
    )
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
