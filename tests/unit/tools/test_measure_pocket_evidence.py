"""Smoke the measurement tool: each subcommand runs and prints its table head.

The numbers are deliberately not frozen here. Constants that must not move
belong to the detector's own tests; this tool exists so a reviewer can
regenerate a table, and freezing its output would make every new measurement
look like a regression.
"""

from pathlib import Path

import pytest

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from tools.measure_pocket_evidence import STRUCTURES, main, structure

ROOT = Path(__file__).resolve().parents[3]
GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_t11_06.yaml")

FAST = ["--seeds", "1"]


def test_evidence_prints_the_gate_terms(capsys):
    assert main(["evidence", "--x", "3.0", *FAST]) == 0
    out = capsys.readouterr().out
    # The term that blocks this shape must be visible, not inferred.
    assert "supports" in out and "lower" in out and "upper" in out
    assert "plane candidates:" in out
    assert "observation:" in out


def test_grid_reports_cell_totals(capsys):
    assert main(["grid", "--distances", "3.0:3.2:0.2", *FAST]) == 0
    out = capsys.readouterr().out
    assert "# cells:" in out and "worst_mm" in out


def test_structures_names_every_column(capsys):
    assert main(["structures", "--distances", "3.0:3.0:1", *FAST]) == 0
    out = capsys.readouterr().out
    for name in STRUCTURES:
        assert name in out


def test_fov_states_the_closed_form_and_measures_it(capsys):
    assert main(["fov", "--distances", "2.0:2.2:0.2", *FAST]) == 0
    out = capsys.readouterr().out
    assert "band bottom" in out and "full width visible" in out
    assert "band_px" in out and "deck_px" in out


def test_noise_states_its_model(capsys):
    assert (
        main(["noise", "--distances", "3.0:3.0:1", "--sigmas", "0,0.010", *FAST]) == 0
    )
    out = capsys.readouterr().out
    assert "sigma = k*d^2" in out


def test_poses_reports_the_near_far_split(capsys):
    assert main(["poses", "--category", "positive", *FAST]) == 0
    out = capsys.readouterr().out
    assert "# all-seed" in out and "worst accepted" in out


def test_header_records_the_axes_that_changed_conclusions(capsys):
    main(["grid", "--distances", "3.0:3.0:1", *FAST])
    out = capsys.readouterr().out
    # Quantisation and camera pose are printed because leaving them implicit
    # is what made earlier tables irreproducible.
    assert "quantize=" in out and "camera=" in out and "rule=" in out


def test_variant_c_is_refused_until_the_detector_has_it():
    with pytest.raises(NotImplementedError, match="Task 2"):
        main(["grid", "--rule", "variant-c", "--distances", "3.0:3.0:1", *FAST])


@pytest.mark.parametrize("kind", STRUCTURES)
def test_every_structure_builds_and_differs_from_the_pallet(kind):
    boxes = structure(GEOMETRY, kind)
    assert boxes, f"{kind} produced no boxes"
    if kind != "pallet":
        assert boxes != structure(GEOMETRY, "pallet")


def test_grounded_has_no_gap_beneath_its_columns():
    """This is the structure the detector cannot tell from a pallet."""
    boxes = structure(GEOMETRY, "grounded")
    columns = [b for b in boxes if b.size_m[2] > GEOMETRY.block_height_m]
    assert columns, "grounded must extend its columns"
    for box in columns:
        assert box.centre_m[2] == pytest.approx(box.size_m[2] / 2), (
            "column must reach z=0"
        )


def test_evidence_sweeps_distances_and_names_the_terms(capsys):
    assert main(["evidence", "--distances", "3.0:3.5:0.5", *FAST]) == 0
    out = capsys.readouterr().out
    for column in ("supports (l,c,r)", "lower", "u_left", "u_right", "observation"):
        assert column in out


def test_evidence_defaults_to_the_configured_seed_not_zero(capsys):
    """Boundary counts are seed-sensitive; a zero default reports different
    evidence for the same scene than the configuration it claims to measure."""
    from pathlib import Path

    import yaml

    from tools.measure_pocket_evidence import DEFAULT_PARAMS

    configured = yaml.safe_load(Path(DEFAULT_PARAMS).read_text())["seed"]
    assert configured != 0, "this test is vacuous if the frozen seed is zero"
    main(["evidence", "--x", "3.0", *FAST])
    default_out = capsys.readouterr().out
    main(["evidence", "--x", "3.0", "--seed", str(configured), *FAST])
    explicit_out = capsys.readouterr().out
    assert default_out == explicit_out


def test_planes_reports_margin_and_pairwise_overlap(capsys):
    assert main(["planes", "--distances", "2.5:2.5:0.5", *FAST]) == 0
    out = capsys.readouterr().out
    for column in ("cands", "resid_mm", "margin_mm", "overlap"):
        assert column in out


def test_zcut_derives_the_beam_height_from_the_shadow_height(capsys):
    """The sweep axis is the shadow on the pallet face, not the beam itself.

    A beam low enough to shadow a 100 mm deck sits below the camera and hides
    the whole scene, so sweeping the beam directly measures nothing.
    """
    assert main(["zcut", "--cuts", "0.10:0.11:0.005", *FAST]) == 0
    out = capsys.readouterr().out
    assert "beam bottom =" in out and "beam_z_m" in out
    # The beam must sit well above the shadow it casts.
    rows = [r.split() for r in out.splitlines() if r.startswith("  0.1")]
    assert rows, out
    for row in rows:
        z_cut, beam_z = float(row[0]), float(row[1])
        assert beam_z > z_cut


def test_zcut_places_the_obstruction_by_its_front_face(capsys):
    """Centring it on the nominal x puts the face half a box further forward,
    where it eats the support columns and every count collapses for the wrong
    reason."""
    main(["zcut", "--cuts", "0.13:0.13:0.005", "--front-x", "1.6", *FAST])
    out = capsys.readouterr().out
    assert "FRONT FACE at x=1.6" in out


def test_grid_output_is_byte_identical_to_the_pre_mount_tool(capsys):
    fixtures = ROOT / "tests/fixtures/measure_pocket_evidence"
    args = (fixtures / "grid_epal6_derived.args").read_text().split()
    assert main(args) == 0
    assert capsys.readouterr().out == (fixtures / "grid_epal6_derived.txt").read_text()


def _tip(camera_xyz, shape=None, face_gap=None):
    from tools.measure_pocket_evidence import blades, tip_visibility
    from tools.scene_rig import Camera, pallet, place, truck_boxes

    truck = truck_boxes(ROOT / "sim/models/dls08_measured/forklift.urdf")
    boxes = list(truck)
    if shape is not None:
        geometry = load_pallet_geometry(ROOT / f"config/pallet_geometry_{shape}.yaml")
        boxes += place(
            pallet(geometry), x_m=0.95 + face_gap + geometry.overall_depth_m / 2
        )
    side = blades(truck)
    camera = Camera(camera_xyz)
    return {name: tip_visibility(boxes, i, camera, 0.175) for name, i in side.items()}


def test_low_carriage_mount_sees_both_tips_and_a_high_one_none():
    assert _tip((0.61, 0.0, 0.15)) == {"left": 1.0, "right": 1.0}
    assert _tip((0.75, 0.0, 0.50)) == {"left": 0.0, "right": 0.0}


def test_tips_go_out_of_sight_once_inside_a_t11_pocket():
    # T11 stringers bottom out 60 mm up, so 100 mm in hides the blade tops.
    assert _tip((0.61, 0.0, 0.15), "t11_06", -0.10) == {"left": 0.0, "right": 0.0}
    epal = _tip((0.61, 0.0, 0.15), "epal6", -0.10)
    assert set(epal) == {"left", "right"}  # shape-specific: recorded, not frozen


def test_mount_refuses_cameras_behind_the_carriage_or_looking_back():
    with pytest.raises(SystemExit, match="behind the carriage"):
        main(["mount", "--camera-x", "0.55", "--distances", "2.0:2.0:1", *FAST])
    with pytest.raises(SystemExit, match="backwards"):
        main(["mount", "--camera-tilt", "1.2", "--distances", "2.0:2.0:1", *FAST])


def test_mount_marks_penetrating_cells_and_summarises(capsys):
    # T11 at 10 mm of lift: the blades cut the stringers once inserted.
    assert (
        main(
            [
                "mount",
                "--shape",
                "t11_06",
                "--lift",
                "0.01",
                "--distances",
                "1.24:1.30:0.03",
                *FAST,
            ]
        )
        == 0
    )
    out = capsys.readouterr().out
    assert "penetrating" in out
    # 90 mm T11 face sits below a camera body whose bottom is at 0.1475 m.
    assert "camera_blocks_insertion=False target=0.300 common_target=0.300" in out
    assert "#   first non-zero:" in out and "#   simultaneous:" in out
    assert "#   dead runs (ok=0):" in out


def test_mount_smoke_prints_tip_columns(capsys):
    assert main(["mount", "--distances", "2.20:2.25:0.05", *FAST]) == 0
    out = capsys.readouterr().out
    assert (
        "tipL" in out
        and "tipR" in out
        and "derived_for(pallet_prior_epal6.yaml)" in out
    )
    # The 144 mm EPAL face reaches the camera body (bottom 0.1375 m) at z 0.15.
    assert "camera_limit=0.330" in out
    assert "camera_blocks_insertion=True target=0.284 common_target=0.300" in out


def test_pocket_error_is_m2_left_to_left_in_3d():
    from types import SimpleNamespace

    import numpy as np

    from tools.measure_pocket_evidence import pocket_error_m

    truth = np.array([[1.0, 0.2, 0.05], [1.0, -0.2, 0.05]])
    pocket = lambda *c: SimpleNamespace(center_m=c)  # noqa: E731
    high = SimpleNamespace(left=pocket(1.0, 0.2, 0.15), right=pocket(1.0, -0.2, 0.05))
    assert pocket_error_m(high, truth) == pytest.approx(0.10)
    swapped = SimpleNamespace(
        left=pocket(1.0, -0.2, 0.05), right=pocket(1.0, 0.2, 0.05)
    )
    assert pocket_error_m(swapped, truth) == pytest.approx(0.40)


def _summary(out):
    return [line for line in out.splitlines() if line.startswith("#   ")]


def test_mount_summary_does_not_depend_on_distance_order(capsys):
    main(["mount", "--distances", "2.20:2.40:0.10", *FAST])
    forward = _summary(capsys.readouterr().out)
    main(["mount", "--distances", "2.40:2.20:-0.10", *FAST])
    backward = _summary(capsys.readouterr().out)
    assert forward == backward


def test_mount_needs_at_least_one_seed():
    with pytest.raises(SystemExit, match="seeds"):
        main(["mount", "--seeds", "0", "--distances", "2.2:2.2:1"])


def test_mount_parses_boolean_overrides(capsys):
    assert (
        main(
            [
                "mount",
                "--set",
                "median_plane_offset=true",
                "--distances",
                "2.2:2.2:1",
                *FAST,
            ]
        )
        == 0
    )
    assert '"median_plane_offset": true' in capsys.readouterr().out


def test_camera_beside_the_pallet_does_not_shorten_insertion():
    from types import SimpleNamespace

    from tools.measure_pocket_evidence import camera_blocks_insertion
    from tools.scene_rig import pallet, place

    geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    args = SimpleNamespace(
        camera_size=(0.025, 0.09, 0.025), camera_z=0.15, lift=0.0, camera_y=0.44
    )
    assert not camera_blocks_insertion(
        args, place(pallet(geometry), x_m=2.0, y_m=-0.05)
    )
    assert camera_blocks_insertion(args, place(pallet(geometry), x_m=2.0, y_m=0.1))


def test_yaw_face_gap_uses_the_nearer_pocket_entry(capsys):
    main(
        [
            "mount",
            "--shape",
            "t11_06",
            "--yaw",
            "0.05",
            "--distances",
            "2.5:2.5:1",
            *FAST,
        ]
    )
    row = next(
        line
        for line in capsys.readouterr().out.splitlines()
        if line.lstrip().startswith("2.500")
    )
    assert float(row.split()[1]) == pytest.approx(1.213, abs=0.001)


def test_default_range_reaches_each_poses_own_target(capsys, monkeypatch):
    from types import SimpleNamespace

    import tools.measure_pocket_evidence as mpe

    missing = SimpleNamespace(
        observation=SimpleNamespace(status="invalid", reason="stub")
    )
    monkeypatch.setattr(mpe, "detect_pockets", lambda *a, **k: missing)
    monkeypatch.setattr(mpe, "tip_visibility", lambda *a, **k: 0.0)
    monkeypatch.setattr(mpe.scene_rig, "render", lambda *a, **k: None)
    # Lateral -0.04 clears a camera at y 0.44, so that pose targets 0.300 m while
    # the aligned pose (blocked) would target 0.144 m; the range must start at 0.95.
    assert (
        main(
            [
                "mount",
                "--camera-x",
                "0.75",
                "--camera-y",
                "0.44",
                "--lateral",
                "-0.04",
                *FAST,
            ]
        )
        == 0
    )
    rows = [
        line.split()
        for line in capsys.readouterr().out.splitlines()
        if line[:1] == " " and line.split()[0] != "x_m"
    ]
    assert float(rows[0][0]) == pytest.approx(0.95)
    assert float(rows[0][1]) == pytest.approx(-0.30)
    assert float(rows[-1][1]) >= 2.0
