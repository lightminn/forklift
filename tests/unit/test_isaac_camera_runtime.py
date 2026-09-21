"""CPU regressions for G1 OpenCV policy and owned annotator teardown."""

import json
import runpy
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
VERIFY = runpy.run_path(str(ROOT / "sim/isaac/verify_perception_camera.py"))
MEASURE = VERIFY["MEASURE"]


class FakeOpenCL:
    def __init__(self, available, *, ignore_disable=False):
        self.available = available
        self.enabled = True
        self.ignore_disable = ignore_disable

    def setUseOpenCL(self, enabled):
        if not self.ignore_disable:
            self.enabled = enabled

    def haveOpenCL(self):
        return self.available

    def useOpenCL(self):
        return self.enabled


class FakeAnnotator:
    def __init__(self, *, attached=True, error=None, before_detach=lambda: None):
        self.is_attached = attached
        self.error = error
        self.before_detach = before_detach
        self.detach_calls = 0

    def attach(self, products):
        assert products == ["/Render/Original"]
        self.is_attached = True

    def detach(self, render_products=None):
        # The camera's current product may differ from the original attachment.
        assert render_products is None
        self.detach_calls += 1
        self.before_detach()
        if self.error:
            raise RuntimeError(self.error)
        if not self.is_attached:
            raise RuntimeError("Annotator rgb is not attached to any render products")
        self.is_attached = False


@pytest.mark.parametrize("available", [True, False])
def test_run_disables_opencl_and_persists_actual_state_before_measurement(
    monkeypatch, tmp_path, available
):
    ocl = FakeOpenCL(available)
    monkeypatch.setitem(
        sys.modules, "cv2", SimpleNamespace(ocl=ocl, __version__="fake")
    )
    expected = {
        "requested_use_opencl": False,
        "have_opencl": available,
        "use_opencl": False,
    }
    entered = []

    def measure(app, args, result, annotators):
        assert ocl.enabled is False, "OpenCL must be off before corner detection"
        saved = json.loads((args.output / "result.json").read_text())
        assert saved["opencv_opencl"] == expected
        assert saved["opencv_version"] == "fake"
        entered.append(True)

    monkeypatch.setitem(VERIFY["run"].__globals__, "_run_measurements", measure)
    result = {"status": "FAIL"}
    VERIFY["run"](None, SimpleNamespace(output=tmp_path), result)
    assert entered == [True]
    assert result["opencv_opencl"] == expected


def test_opencl_disable_readback_failure_is_recorded_and_stops_measurement(
    monkeypatch, tmp_path
):
    ocl = FakeOpenCL(True, ignore_disable=True)
    monkeypatch.setitem(
        sys.modules, "cv2", SimpleNamespace(ocl=ocl, __version__="fake")
    )

    def forbidden(*args):
        pytest.fail("measurement started with OpenCL enabled")

    monkeypatch.setitem(VERIFY["run"].__globals__, "_run_measurements", forbidden)
    with pytest.raises(RuntimeError, match="OpenCL"):
        VERIFY["run"](None, SimpleNamespace(output=tmp_path), {"status": "FAIL"})
    saved = json.loads((tmp_path / "result.json").read_text())
    assert saved["opencv_opencl"]["use_opencl"] is True


@pytest.mark.parametrize("measurement_fails", [False, True])
def test_run_saves_result_then_detaches_on_success_and_exception(
    monkeypatch, tmp_path, measurement_fails
):
    monkeypatch.setitem(
        sys.modules, "cv2", SimpleNamespace(ocl=FakeOpenCL(True), __version__="fake")
    )

    def check_saved():
        saved = json.loads((tmp_path / "result.json").read_text())
        assert saved["boards"] == [{"captured": True}]
        if not measurement_fails:
            assert saved["gates"] == {"1": "PASS"}

    rgb = FakeAnnotator(before_detach=check_saved)
    depth = FakeAnnotator(attached=False)

    def measure(app, args, result, annotators):
        annotators.update(rgb=rgb, z=depth)
        result["boards"] = [{"captured": True}]
        if measurement_fails:
            raise ValueError("original measurement failure")
        result.update(gates={"1": "PASS"}, status="REVIEW_REQUIRED")

    monkeypatch.setitem(VERIFY["run"].__globals__, "_run_measurements", measure)
    result = {"status": "FAIL"}
    if measurement_fails:
        with pytest.raises(ValueError, match="original measurement failure"):
            VERIFY["run"](None, SimpleNamespace(output=tmp_path), result)
    else:
        VERIFY["run"](None, SimpleNamespace(output=tmp_path), result)
        assert result["status"] == "REVIEW_REQUIRED"
    assert rgb.detach_calls == 1
    assert depth.detach_calls == 0
    saved = json.loads((tmp_path / "result.json").read_text())
    assert saved["annotator_cleanup"] == {
        "rgb": {"status": "detached"},
        "z": {"status": "already_detached"},
    }


def test_detach_skips_disconnected_and_consumes_ownership_for_repeat_cleanup():
    rgb, depth = FakeAnnotator(attached=False), FakeAnnotator()
    owned = {"rgb": rgb, "z": depth}
    report = MEASURE.detach_annotators(owned)
    assert report == {
        "rgb": {"status": "already_detached"},
        "z": {"status": "detached"},
    }
    assert MEASURE.detach_annotators(owned) == {}
    assert rgb.detach_calls == 0
    assert depth.detach_calls == 1
    assert owned == {}


def test_run_keeps_camera_alive_until_after_annotator_cleanup(monkeypatch, tmp_path):
    monkeypatch.setitem(
        sys.modules, "cv2", SimpleNamespace(ocl=FakeOpenCL(True), __version__="fake")
    )
    events = []
    rgb = FakeAnnotator(before_detach=lambda: events.append("detach"))

    class Camera:
        def __del__(self):
            events.append(("camera_released", rgb.is_attached))

    def measure(app, args, result, annotators):
        annotators["rgb"] = rgb
        return Camera()

    monkeypatch.setitem(VERIFY["run"].__globals__, "_run_measurements", measure)
    VERIFY["run"](None, SimpleNamespace(output=tmp_path), {"status": "FAIL"})
    assert events == ["detach", ("camera_released", False)]


@pytest.mark.parametrize(
    "error,status",
    [
        ("Annotator rgb is not attached to any render products", "already_detached"),
        ("unexpected graph failure", "error"),
    ],
)
def test_detach_records_errors_and_continues_other_channels(error, status):
    # The renderer may drop a connection after the is_attached readback.
    rgb, depth = FakeAnnotator(error=error), FakeAnnotator()
    owned = {"rgb": rgb, "z": depth}
    report = MEASURE.detach_annotators(owned)
    assert report["rgb"]["status"] == status
    assert error in report["rgb"]["error"]
    assert report["z"] == {"status": "detached"}
    assert depth.is_attached is False
    MEASURE.detach_annotators(owned)
    assert rgb.detach_calls == depth.detach_calls == 1


def test_partial_attachment_remains_owned_for_finally_cleanup():
    rgb = FakeAnnotator(attached=False)

    def get_annotator(name, **kwargs):
        if name == "rgb":
            return rgb
        raise RuntimeError("cannot create segmentation annotator")

    rep = SimpleNamespace(
        AnnotatorRegistry=SimpleNamespace(get_annotator=get_annotator)
    )
    camera = SimpleNamespace(get_render_product_path=lambda: "/Render/Original")
    owned = {}
    with pytest.raises(RuntimeError, match="cannot create segmentation"):
        MEASURE.attach_annotators(rep, camera, annotators=owned)
    assert owned == {"rgb": rgb}
    MEASURE.detach_annotators(owned)
    assert rgb.is_attached is False
    assert rgb.detach_calls == 1
