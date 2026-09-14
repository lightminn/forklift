"""Smoke the measurement tool: each subcommand runs and prints its table head.

The numbers are deliberately not frozen here. Constants that must not move
belong to the detector's own tests; this tool exists so a reviewer can
regenerate a table, and freezing its output would make every new measurement
look like a regression.
"""

import pytest

from tools.measure_pocket_evidence import STRUCTURES, main, structure
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from pathlib import Path

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
    assert main(["noise", "--distances", "3.0:3.0:1", "--sigmas", "0,0.010", *FAST]) == 0
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
        assert box.centre_m[2] == pytest.approx(box.size_m[2] / 2), "column must reach z=0"


def test_evidence_sweeps_distances_and_names_the_terms(capsys):
    assert main(["evidence", "--distances", "3.0:3.5:0.5", *FAST]) == 0
    out = capsys.readouterr().out
    for column in ("supports (l,c,r)", "lower", "u_left", "u_right", "observation"):
        assert column in out


def test_evidence_defaults_to_the_configured_seed_not_zero(capsys):
    """Boundary counts are seed-sensitive; a zero default reports different
    evidence for the same scene than the configuration it claims to measure."""
    import yaml
    from pathlib import Path

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
