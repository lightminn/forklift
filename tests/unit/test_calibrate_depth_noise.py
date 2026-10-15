# tests/unit/test_calibrate_depth_noise.py
import json

import numpy as np
import pytest
from PIL import Image

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from tools import calibrate_depth_noise as cal
from tools import instance_rig

QUIET = dict(edge_jump_m=1e9, dropout_fraction=0.0, quantize_step_m=0.0, registration_shift_px=(0, 0))
TRUE = dict(axial_k=0.0, components=((0.0008, (1.8, 0.0)), (0.004, (6.0, 4.0))))


def _cache(root, scene="dynamic/000000", frames=4, with_gt=False):
    """A tiny BOP-layout cache: a noisy wall at z = 5.25 m with one masked object per frame."""
    out = instance_rig.render_instances([], floor=False)
    i = out.scene.intrinsics
    base = root / scene
    (base / "depth").mkdir(parents=True)
    (base / "mask_visib").mkdir()
    (root / "models_names.json").write_text(json.dumps({"1": "wooden Euro pallet", "2": "small grey KLT"}))
    names, cams, gts = ["models_names.json", f"{scene}/scene_camera.json"], {}, {}
    for f in range(frames):
        noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=f, **TRUE, **QUIET))
        raw = np.rint(noisy.depth_m * 1000).astype(np.uint16)
        raw[200:240, 300:400] = 0
        Image.fromarray(raw).save(base / "depth" / f"{f:06d}.png")
        mask = np.zeros(raw.shape, np.uint8)
        mask[180:260, 280:420] = 255
        Image.fromarray(mask).save(base / "mask_visib" / f"{f:06d}_000000.png")
        names += [f"{scene}/depth/{f:06d}.png", f"{scene}/mask_visib/{f:06d}_000000.png"]
        cams[str(f)] = {"cam_K": [i.fx, 0, i.cx, 0, i.fy, i.cy, 0, 0, 1], "depth_scale": 1.0}
        gts[str(f)] = [{"obj_id": 1}]
    (base / "scene_camera.json").write_text(json.dumps(cams))
    if with_gt:
        (base / "scene_gt.json").write_text(json.dumps(gts))
        names.append(f"{scene}/scene_gt.json")
    (root / "listing.json").write_text(json.dumps({"revision": cal.REVISION, "files": names}))
    return i.fx


def _frames(root, scene, stride=1):
    return cal.select_frames(cal.read_listing(root), [scene], stride)


def test_frames_come_from_the_listing_not_from_the_cache(tmp_path):
    _cache(tmp_path)
    frames = _frames(tmp_path, "dynamic/000000", 2)
    assert [f["frame"] for f in frames] == [0, 2]
    assert frames[0]["masks"] == ["dynamic/000000/mask_visib/000000_000000.png"]


def test_the_survey_recovers_known_components_and_reports_the_patch_check(tmp_path):
    fx = _cache(tmp_path)
    result = cal.calibrate(tmp_path, _frames(tmp_path, "dynamic/000000"), z_range=(5.0, 6.0))
    short, long = result["survey"]["short"], result["survey"]["long"]
    assert short["k"]["p50"] == pytest.approx(0.0008, rel=0.2) and short["g_u"]["p50"] == pytest.approx(1.8, rel=0.3)
    assert long["k"]["p50"] == pytest.approx(0.004, rel=0.15) and long["g_u"]["p50"] == pytest.approx(6.0, rel=0.25)
    assert result["role"] == "provisional" and result["source_fx"] == pytest.approx(fx) and result["usable"] == 4
    assert 0.85 < result["patch_check"]["ratio"]["p50"] < 1.15
    mid = result["levels"]["mid"]
    assert mid["k_iid"] == 0.0 and [c["k"] for c in mid["components"]] == pytest.approx([0.0008, 0.004], rel=0.2)
    assert result["levels"]["low"]["components"][1]["k"] <= mid["components"][1]["k"] <= \
        result["levels"]["high"]["components"][1]["k"]
    assert result["dropout_fraction"] < 0.01 and result["license"] == "CC-BY-4.0" and result["history"]
    for key in ("u48", "u64", "v48", "v64"):  # large-step diagnostics survive into the output
        assert result["survey"]["diagnostic_ratio"][key]["n"] == 4
        assert 0.7 < result["survey"]["diagnostic_ratio"][key]["p50"] < 1.3

def test_pallet_surface_report_uses_the_pallet_masks(tmp_path):
    _cache(tmp_path, scene="val/000000", with_gt=True)
    report = cal.pallet_surface_report(tmp_path, _frames(tmp_path, "val/000000"), z_range=(5.0, 6.0))
    assert report["scenes"] == ["val/000000"]
    assert report["pallet_dropout_fraction"] == pytest.approx(4000 / 11200, rel=0.01)
    assert set(report["ratio_u"]) == {1, 2, 3, 4, 6, 8}


def test_frames_without_full_plane_patches_are_not_used(tmp_path):
    _cache(tmp_path)
    from PIL import Image as _Image

    for f in range(3):  # holes on a 48 px lattice: enough second differences, no fully valid patch
        path = tmp_path / "dynamic/000000/depth" / f"{f:06d}.png"
        raw = np.asarray(_Image.open(path)).copy()
        raw[24::48, 24::48] = 0
        _Image.fromarray(raw).save(path)
    result = cal.calibrate(tmp_path, _frames(tmp_path, "dynamic/000000"), z_range=(5.0, 6.0))
    assert result["usable"] == 1 and result["rejected"]["patches"] == 3

def test_config_round_trip_gives_a_provisional_noise_model(tmp_path):
    fx = _cache(tmp_path)
    out = tmp_path / "noise.yaml"
    cal.main(["--cache", str(tmp_path), "--no-download", "--calibration-scenes", "dynamic/000000",
              "--pallet-scenes", "", "--stride", "1", "--z-range", "5.0,6.0", "--out", str(out)])
    model = NoiseModel.from_yaml(out, seed=3, target_fx=fx / 2, level="mid")
    (k1, g1), (k2, g2) = model.components
    assert k2 == pytest.approx(0.004, rel=0.15) and g2[0] == pytest.approx(3.0, rel=0.25) and model.seed == 3

def test_a_cache_from_another_revision_is_refused(tmp_path):
    _cache(tmp_path)
    (tmp_path / "listing.json").write_text(json.dumps({"revision": "other", "files": []}))
    with pytest.raises(ValueError, match="revision"):
        cal.read_listing(tmp_path)


def test_the_pallet_comparison_inputs_are_hashed(tmp_path):
    _cache(tmp_path, scene="val/000000", with_gt=True)
    report = cal.pallet_surface_report(tmp_path, _frames(tmp_path, "val/000000"), z_range=(5.0, 6.0))
    assert "val/000000/scene_gt.json" in report["files"] and "val/000000/depth/000000.png" in report["files"]
    before = report["files_sha256"]
    (tmp_path / "val/000000/scene_gt.json").write_text(json.dumps({str(f): [{"obj_id": 2}] for f in range(4)}))
    assert cal.pallet_surface_report(tmp_path, _frames(tmp_path, "val/000000"), z_range=(5.0, 6.0))["files_sha256"] != before


def test_a_noise_free_quantised_frame_is_measured_at_the_dither_level(tmp_path):
    """1 mm steps leave over half the second differences at 0 (MAD = 0); a +-0.5 mm dither unties them."""
    _cache(tmp_path, frames=5)
    path = tmp_path / "dynamic/000000/depth/000004.png"
    Image.fromarray(np.full((480, 640), 5250, np.uint16)).save(path)
    frames = _frames(tmp_path, "dynamic/000000")
    depth, masks, _ = cal._load(tmp_path, frames[4])
    record = cal.frame_structure(depth, masks, z_range=(5.0, 6.0), margin_px=8, seed=0)
    assert "reject" not in record and max(c[0] for c in record["components"]) < 1e-4