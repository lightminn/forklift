"""CPU regressions for G1 transport refusal and transitive source provenance."""

import hashlib
import runpy
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from forklift_core.sensors.rgbd import PinholeIntrinsics

ROOT = Path(__file__).resolve().parents[2]
RUNNER = runpy.run_path(str(ROOT / "sim/isaac/run_transport.py"))
K = PinholeIntrinsics(640, 480, 465.741156, 465.741156, 320, 240, "optical")


class FakeCamera:
    def __init__(self):
        self.matrix = np.array([[K.fx, 0, K.cx], [0, K.fy, K.cy], [0, 0, 1.0]])
        self.resolution = (640, 480)

    def get_intrinsics_matrix(self):
        return self.matrix

    def get_resolution(self):
        return self.resolution

    def get_render_product_path(self):
        return "/Render/PerceptionCamera"


@pytest.fixture(autouse=True)
def render_stage(monkeypatch):
    """Model USD readback separately from Camera's cached resolution; no Kit."""
    product = SimpleNamespace(resolution=(640, 480), valid=True)

    class Prim:
        def IsValid(self):
            return product.valid

        def GetAttribute(self, name):
            assert name == "resolution"
            return SimpleNamespace(Get=lambda: product.resolution)

    class Stage:
        def GetPrimAtPath(self, path):
            assert path == "/Render/PerceptionCamera"
            return Prim()

    omni, usd = ModuleType("omni"), ModuleType("omni.usd")
    usd.get_context = lambda: SimpleNamespace(get_stage=Stage)
    omni.usd = usd
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.usd", usd)
    return product


def test_getter_accepts_matching_intrinsics_and_records_readback_path():
    state = {}
    RUNNER["verify_camera_intrinsics"](FakeCamera(), K, state)
    record = state["perception_camera_intrinsics"]
    assert record["status"] == "PASS"
    assert "get_intrinsics_matrix" in record["intrinsics_source"]
    assert 'GetAttribute("resolution").Get()' in record["resolution_source"]
    assert "get_resolution" in record["cached_resolution_source"]
    assert "cache" in record["cached_resolution_source"]
    assert record["resolution"] == record["cached_resolution"] == [640, 480]
    assert record["render_product_path"] == "/Render/PerceptionCamera"


def test_g1a_compares_and_records_raw_and_integer_index_conventions():
    camera, state = FakeCamera(), {}
    camera.matrix[0, 2] += 0.0625
    camera.matrix[1, 2] -= 0.0625
    RUNNER["verify_camera_intrinsics"](camera, K, state)
    record = state["perception_camera_intrinsics"]
    for name, convention, cx, cy in (
        ("raw_sdk", "isaac_sdk_half_integer_centers", 320.0, 240.0),
        ("integer_index", "integer_index_centers", 319.5, 239.5),
    ):
        comparison = record[name]
        assert comparison["coordinate_convention"] == convention
        assert comparison["nominal"]["cx"] == cx
        assert comparison["nominal"]["cy"] == cy
        assert comparison["matrix"][0][2] == cx + 0.0625
        assert comparison["matrix"][1][2] == cy - 0.0625
        assert comparison["errors"]["cx_px"] == 0.0625
        assert comparison["errors"]["cy_px"] == 0.0625
        assert comparison["status"] == "PASS"
    RUNNER["record_json"](state)


@pytest.mark.parametrize("resolution", [(320, 240), (639, 480), (640, 481)])
def test_render_product_mismatch_is_refused_even_when_camera_cache_matches(
    render_stage, resolution
):
    render_stage.resolution = resolution
    state = {}
    with pytest.raises(RuntimeError, match="intrinsics"):
        RUNNER["verify_camera_intrinsics"](FakeCamera(), K, state)
    record = state["perception_camera_intrinsics"]
    assert record["status"] == "FAIL"
    assert record["cached_resolution"] == [640, 480]
    assert record["resolution"] == list(resolution)
    RUNNER["record_json"](state)


@pytest.mark.parametrize("missing", ["prim", "attribute"])
def test_render_product_readback_missing_fails_closed(render_stage, missing):
    if missing == "prim":
        render_stage.valid = False
    else:
        render_stage.resolution = None
    state = {}
    with pytest.raises(RuntimeError, match="intrinsics"):
        RUNNER["verify_camera_intrinsics"](FakeCamera(), K, state)
    assert state["perception_camera_intrinsics"]["status"] == "FAIL"
    RUNNER["record_json"](state)


@pytest.mark.parametrize(
    "row,col,delta",
    [
        (0, 0, K.fx * 0.00121),
        (1, 1, -K.fy * 0.00121),
        (0, 2, 0.1001),
        (1, 2, -0.1001),
        (0, 0, np.nan),
    ],
)
def test_getter_refuses_out_of_tolerance_and_retains_failure(row, col, delta):
    camera, state = FakeCamera(), {}
    camera.matrix[row, col] += delta
    with pytest.raises(RuntimeError, match="intrinsics"):
        RUNNER["verify_camera_intrinsics"](camera, K, state)
    assert state["perception_camera_intrinsics"]["status"] == "FAIL"
    # Failure records must remain writable by the transport's strict JSON encoder.
    RUNNER["record_json"](state)


@pytest.mark.parametrize("resolution", [(639, 480), (640, 481)])
def test_getter_refuses_either_resolution_mismatch(resolution):
    camera = FakeCamera()
    camera.resolution = resolution
    with pytest.raises(RuntimeError, match="intrinsics"):
        RUNNER["verify_camera_intrinsics"](camera, K, {})


def test_getter_accepts_values_just_inside_limits():
    camera = FakeCamera()
    camera.matrix[0, 0] *= 1.001199
    camera.matrix[1, 1] *= 0.998801
    camera.matrix[0, 2] += 0.09999
    camera.matrix[1, 2] -= 0.09999
    RUNNER["verify_camera_intrinsics"](camera, K, {})


def test_source_hashes_cover_transitive_core_and_preserve_repository_paths(tmp_path):
    repo = tmp_path / "repo"
    core = tmp_path / "installed/forklift_core"
    sources = {
        core / "__init__.py": b"package",
        core / "perception/pocket_detector.py": b"detector",
        core / "geometry.py": b"geometry",
        core / "sensors/rgbd.py": b"depth",
        core / "_validation.py": b"validation",
        core / "future/nested/dependency.py": b"future transitive dependency",
        repo / "sim/isaac/run_transport.py": b"runner",
        repo / "sim/isaac/perception_adapter.py": b"adapter",
        repo / "sim/isaac/new_helper.py": b"helper",
        repo / "tools/scene_rig.py": b"rig",
    }
    for path, content in sources.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    record = RUNNER["source_sha256"](repo, core)
    assert record == {
        "forklift_core/__init__.py": hashlib.sha256(b"package").hexdigest(),
        "forklift_core/perception/pocket_detector.py": hashlib.sha256(
            b"detector"
        ).hexdigest(),
        "forklift_core/geometry.py": hashlib.sha256(b"geometry").hexdigest(),
        "forklift_core/sensors/rgbd.py": hashlib.sha256(b"depth").hexdigest(),
        "forklift_core/_validation.py": hashlib.sha256(b"validation").hexdigest(),
        "forklift_core/future/nested/dependency.py": hashlib.sha256(
            b"future transitive dependency"
        ).hexdigest(),
        "sim/isaac/run_transport.py": hashlib.sha256(b"runner").hexdigest(),
        "sim/isaac/perception_adapter.py": hashlib.sha256(b"adapter").hexdigest(),
        "sim/isaac/new_helper.py": hashlib.sha256(b"helper").hexdigest(),
        "tools/scene_rig.py": hashlib.sha256(b"rig").hexdigest(),
    }
    (core / "geometry.py").write_bytes(b"changed")
    assert (
        RUNNER["source_sha256"](repo, core)["forklift_core/geometry.py"]
        != record["forklift_core/geometry.py"]
    )


@pytest.mark.parametrize("sign", [-1, 1])
def test_getter_limits_are_inclusive_at_exact_authored_bounds(sign):
    camera = FakeCamera()
    camera.matrix[0, 0] = K.fx * (1 + sign * 0.0012)
    camera.matrix[1, 1] = K.fy * (1 + sign * 0.0012)
    camera.matrix[0, 2] = K.cx + sign * 0.1
    camera.matrix[1, 2] = K.cy + sign * 0.1
    state = {}
    RUNNER["verify_camera_intrinsics"](camera, K, state)
    assert state["perception_camera_intrinsics"]["status"] == "PASS"


@pytest.mark.parametrize(
    "row,col,limit",
    [(0, 0, K.fx * 0.0012), (1, 1, K.fy * 0.0012), (0, 2, 0.1), (1, 2, 0.1)],
    ids=["fx", "fy", "cx", "cy"],
)
@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("factor,status", [(0.999, "PASS"), (1.001, "FAIL")])
def test_getter_v11_boundary_in_both_directions(row, col, limit, sign, factor, status):
    camera, state = FakeCamera(), {}
    camera.matrix[row, col] += sign * limit * factor
    if status == "FAIL":
        with pytest.raises(RuntimeError, match="intrinsics"):
            RUNNER["verify_camera_intrinsics"](camera, K, state)
    else:
        RUNNER["verify_camera_intrinsics"](camera, K, state)
    assert state["perception_camera_intrinsics"]["status"] == status
    assert state["perception_camera_intrinsics"]["limits"] == {
        "focal_relative": 0.0012,
        "principal_point_px": 0.1,
    }
