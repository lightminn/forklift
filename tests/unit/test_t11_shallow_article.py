"""Plan 2026-10-22: the shallow T11 cycle test article (t11_06_d600) -- a test article only, beside the canonical
T11 x0.6, read by the far recognition as the T11 face with the run's depth."""
import importlib.util
import json
import math
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception import bottom_deck_permit as BP
from forklift_core.perception.pallet_geometry import load_pallet_geometry, target_insertion_depth_m
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, VARIANT_OF, spec_from_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior
from tools import instance_rig
from tools import recognition_truth as rt

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("far_recognition_runtime", ROOT / "sim/isaac/far_recognition_runtime.py")
R = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R)
SHALLOW = load_pallet_geometry(ROOT / "config/pallet_geometry_t11_06_d600.yaml")
T11 = load_pallet_geometry(ROOT / "config/pallet_geometry_t11_06.yaml")
DT = 1 / 120
PALLET = (4.2, 0.05, 0.02)


def test_the_article_is_t11_x06_but_0_600_m_deep():
    assert SHALLOW.geometry_version == "t11_06_d600" and SHALLOW.overall_depth_m == pytest.approx(0.600)
    a, b = asdict(SHALLOW), asdict(T11)
    for key in ("geometry_version", "source_provenance", "overall_depth_m"):
        a.pop(key), b.pop(key)
    assert a == b


def test_the_insertion_rule_passes_its_centre_by_30_mm():
    target = target_insertion_depth_m(SHALLOW.overall_depth_m, 0.346, 0.016)
    assert target == pytest.approx(0.330) and target - SHALLOW.overall_depth_m / 2 == pytest.approx(0.030)


def test_its_prior_is_built_and_read_back(tmp_path):
    from tools import build_pallet_prior

    out = tmp_path / "prior.yaml"
    assert build_pallet_prior.main(["--geometry", str(ROOT / "config/pallet_geometry_t11_06_d600.yaml"),
                                    "--output", str(out)]) == 0
    built = load_pallet_prior(out)
    assert built.overall_depth_m == pytest.approx(0.600)
    committed = load_pallet_prior(ROOT / "config/pallet_prior_t11_06_d600.yaml")
    assert asdict(committed) == asdict(built)


def test_the_far_recognition_takes_the_variant_in_its_parents_place_only_for_its_run():
    assert VARIANT_OF == {"t11_06_d600": "t11_06"} and KNOWN_ENTRY_AXES["t11_06_d600"] == ("x",)
    variant = R.known_specs(run_shape="t11_06_d600")
    assert set(variant) == {"epal6", "t11_06_d600"}
    assert variant["t11_06_d600"].overall_depth_m == pytest.approx(0.600)
    plain = R.known_specs(run_shape="t11_06")
    assert set(plain) == {"epal6", "t11_06"} and plain["t11_06"].overall_depth_m == pytest.approx(0.660)
    assert set(R.known_specs()) == {"epal6", "t11_06"}


def _render(spec, base):
    x, y, yaw = R.relative(base, PALLET)
    placed = [rt.placed_boxes(rt.instance_boxes(spec, ()), rt.Placement(x, y, yaw, 0.0))]
    out = instance_rig.render_instances(placed)
    return out.scene, out.instance_ids


def test_a_far_stop_on_the_article_is_read_as_its_own_shape_and_recomputes(tmp_path):
    from tools import recompute_far_recognition as RC

    spec = spec_from_geometry(SHALLOW, entry_axes=KNOWN_ENTRY_AXES["t11_06_d600"])
    run = R.FarRuntime(fork_spacing=0.29, pallet_depth_m=0.600, pallet_shape="t11_06_d600",
                       zone_centre_xy=PALLET[:2], zone_radius_m=0.6,
                       camera_x_m=instance_rig.PERCEPTION_CAMERA.xyz_m[0], dt=DT, out_dir=tmp_path / "recognition")
    xs = np.linspace(0.0, 0.6, 240)
    for tick, x in enumerate(xs):
        base = (float(x), 0.0, 0.0)
        run.record_step(tick, tick * DT, base, base, PALLET)
        if run.due(tick, observing=True):
            scene, ids = _render(spec, base)
            run.drive_frame(tick, tick * DT, scene.rgb, scene.depth_m, ids, scene.intrinsics, scene.base_from_optical)
    base = (float(xs[-1]), 0.0, 0.0)
    scene, ids = _render(spec, base)
    scene = replace(scene, stamp_ns=int(round((len(xs) * DT + 0.1) * 1e9)))
    obs, rec = run.stop(scene, ids, initial_time_s=0.0, control_base=base, odom_base=base, truth_base=base,
                        truth_pallet=PALLET)
    assert obs.status == "valid" and rec["used"]["hypothesis"] == "t11_06_d600:x"
    assert [RC.differences(*p) for p in RC.recompute(tmp_path, recognition=run.summary())] == [[]]


def test_the_gate_holds_the_article_and_its_parent_apart():
    rec = BP.BottomDeckRecord("present", (), (3.0, 0.0), 0.0, "t11_06_d600", "x", 0.0, "c0", "0", (1.0, 2.0), None,
                              "p")
    assert BP.insert_refusal(rec, (3.0, 0.0), 0.0, "t11_06_d600") is None
    assert BP.insert_refusal(rec, (3.0, 0.0), 0.0, "t11_06") == "bottom_deck_target_mismatch"


def test_the_site_load_default_names_the_article():
    from forklift_core.perception.fork_fit import load_site_defaults

    specs = R.known_specs(run_shape="t11_06_d600")
    assert "t11_06_d600" in load_site_defaults(ROOT / "config/site_load_defaults.yaml", specs)
