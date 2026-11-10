"""The top-down video cameras' ceiling clip (sim/isaac/overhead_clip.py): the clip height from the environment's
light-holding parts, the near distance of a straight-down camera, and the runners' option (off by default, the near
contract knows it). No Isaac."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
import overhead_clip as oc  # noqa: E402


def test_the_clip_height_is_the_lowest_lamp_less_the_margin_not_below_the_floor_line():
    parts = [(0.0, False), (5.826, True), (5.818, True), (7.88, False), (2.0, False)]
    assert oc.clip_z_from(parts) == pytest.approx(5.518)
    assert oc.clip_z_from([(0.0, False), (6.0, False)]) is None  # no lamp: nothing to clip for
    assert oc.clip_z_from([(3.1, True)]) == oc.FLOOR_M  # never below the hide_overhead line
    assert oc.clip_z_from([(0.2, True), (5.82, True)]) == pytest.approx(5.52)  # a floor light does not count
    assert oc.clip_z_from([(0.2, True)]) is None


def test_a_straight_down_camera_cuts_above_the_plane_only_when_it_cuts_more():
    assert oc.top_down_near(26.0, 5.5, 0.01) == pytest.approx(20.5)
    assert oc.top_down_near(6.0, 5.5, 0.01) == pytest.approx(0.5)
    assert oc.top_down_near(5.0, 5.5, 0.01) is None  # camera under the plane: nothing above it in view
    assert oc.top_down_near(26.0, None, 0.01) is None


class FakeCamera:
    def __init__(self):
        self.range = (0.01, 1.0e6)

    def get_clipping_range(self):
        return self.range

    def set_clipping_range(self, near_distance=None, far_distance=None):
        self.range = (near_distance, far_distance)


def test_apply_sets_only_the_near_distance(monkeypatch):
    monkeypatch.setattr(oc, "read_parts", lambda stage, root="/World/Environment": [(5.82, True), (0.0, False)])
    cam = FakeCamera()
    rec = oc.apply(cam, None, 26.0)
    assert cam.range == (pytest.approx(26.0 - 5.52), 1.0e6)
    assert rec["clip_z_m"] == pytest.approx(5.52) and rec["clipping_range_m"][1] == 1.0e6


def test_read_parts_finds_the_lamps_on_a_small_stage():
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdLux
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World/Environment")
    lamp = UsdGeom.Xform.Define(stage, "/World/Environment/Lamp")
    cube = UsdGeom.Cube.Define(stage, "/World/Environment/Lamp/Shade")
    cube.AddTranslateOp().Set((0.0, 0.0, 6.5))
    light = UsdLux.RectLight.Define(stage, "/World/Environment/Lamp/Light")
    light.AddTranslateOp().Set((0.0, 0.0, 6.0))  # a rect light has extent: it sits in the shade, as the warehouse's do
    floor = UsdGeom.Cube.Define(stage, "/World/Environment/Box")
    floor.AddTranslateOp().Set((0.0, 0.0, 1.0))
    parts = sorted(oc.read_parts(stage))
    assert parts[0] == (pytest.approx(0.0), False) and parts[1][1] is True and parts[1][0] == pytest.approx(5.5)
    assert lamp


def test_the_runners_clip_by_default_and_the_near_contract_leaves_it_out():
    run = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert 'parser.add_argument("--overview-clip", choices=("off", "auto"), default="auto"' in run
    assert 'state["camera"]["overhead_clip"] = overhead_clip.apply(camera, stage, float(eye[2]))' in run
    slam = (ROOT / "sim/isaac/run_slam_drive.py").read_text()
    assert 'parser.add_argument("--overview-clip", choices=("off", "auto"), default="auto"' in slam
    import near_tracking

    assert "overview_clip" in near_tracking.RUN_SPECIFIC_ARGUMENTS  # a video-only setting, not a run condition
    assert "overview_clip" not in near_tracking.POST_MEASUREMENT_ARGUMENTS
