"""Plan 2026-10-22 task A: the near capture looks for the pallet where the mission expects it. Candidates whose
implied centre or entry axis is outside the expected site are dropped before the detector's own selection; with no
expected site the detector is unchanged."""

import math
from pathlib import Path

import pytest

from forklift_core.perception import pocket_detector as P
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior
from sim.isaac import perception_adapter as adapter
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[3]
GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
PRIOR = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
PARAMS = P.DetectorParams.derived_for(PRIOR)
CAMERA = scene_rig.Camera((0.619, 0.0, 0.27))


def two_pallets(x_m, neighbour_dy_m, target_y_m=0.0):
    boxes = (scene_rig.place(scene_rig.pallet(GEOMETRY), x_m=x_m, y_m=target_y_m)
             + scene_rig.place(scene_rig.pallet(GEOMETRY), x_m=x_m, y_m=target_y_m + neighbour_dy_m))
    return scene_rig.render(boxes, camera=CAMERA, quantize=True)


def expected_at(x_m, y_m, yaw_rad=0.0, radius_m=0.15, axis_rad=math.radians(15.0)):
    return P.ExpectedSite(x_m, y_m, yaw_rad, radius_m, axis_rad)


def centre(observation):
    return adapter.estimate_pallet_center_m(observation, PRIOR.overall_depth_m)


@pytest.mark.parametrize("dy, ty", [(1.0, -0.1), (-1.0, 0.1)])
def test_the_neighbour_wins_without_an_expected_site_and_the_target_with_it(dy, ty):
    # Rig cases found by a sweep (x 2.4-3.6 m, neighbour 0.95/1.0 m either side, target y -0.1..0.1): the plain
    # detector reports the neighbour; the expected site keeps the target (aisle_out's failure in the plan).
    sc = two_pallets(3.2, dy, ty)
    plain = P.detect_pockets(sc, PRIOR, PARAMS)
    assert plain.observation.status == "valid"
    assert abs(centre(plain.observation)[1] - (ty + dy)) < 0.05
    result, gate = P.detect_pockets_expected(sc, PRIOR, PARAMS, expected_at(3.2, ty))
    assert result.observation.status == "valid"
    cx, cy = centre(result.observation)
    assert math.hypot(cx - 3.2, cy - ty) < 0.02
    assert any(not c.inside for c in gate) and any(c.inside for c in gate)


def test_the_target_missing_from_the_candidates_is_not_replaced_by_the_neighbour():
    # x 3.0: the target's own pattern fails its evidence; the plain detector reports the neighbour, the expected
    # site reports nothing rather than the wrong pallet.
    sc = two_pallets(3.0, 0.95)
    assert abs(centre(P.detect_pockets(sc, PRIOR, PARAMS).observation)[1] - 0.95) < 0.05
    result, _ = P.detect_pockets_expected(sc, PRIOR, PARAMS, expected_at(3.0, 0.0))
    assert (result.observation.status, result.observation.reason) == ("no_pallet", "no_expected_candidate")


def test_no_candidate_inside_the_site_is_named():
    sc = two_pallets(3.0, 0.95)
    result, gate = P.detect_pockets_expected(sc, PRIOR, PARAMS, expected_at(3.0, -0.6))
    assert result.observation.status == "no_pallet"
    assert result.observation.reason == "no_expected_candidate"
    assert gate and not any(c.inside for c in gate)


def test_the_axis_tolerance_drops_a_candidate_at_the_right_place_but_the_wrong_axis():
    sc = two_pallets(3.2, 1.0, -0.1)
    result, gate = P.detect_pockets_expected(sc, PRIOR, PARAMS, expected_at(3.2, -0.1, yaw_rad=math.radians(20.0)))
    assert result.observation.reason == "no_expected_candidate"
    near = [c for c in gate if c.distance_m < 0.05]
    assert near and all(c.axis_error_rad > math.radians(15.0) and not c.inside for c in near)


def test_the_candidate_record_holds_each_candidate_with_its_centre():
    sc = two_pallets(3.2, 1.0, -0.1)
    _, gate = P.detect_pockets_expected(sc, PRIOR, PARAMS, expected_at(3.2, -0.1))
    assert gate
    for c in gate:
        assert c.inside == (c.distance_m <= 0.15 and c.axis_error_rad <= math.radians(15.0))
        assert math.isclose(c.distance_m, math.hypot(c.centre_m[0] - 3.2, c.centre_m[1] + 0.1), abs_tol=1e-12)


def test_the_plain_detector_is_the_expected_detector_without_a_site():
    for sc in (two_pallets(3.0, 0.95), two_pallets(2.6, 0.95), two_pallets(3.4, -0.95, 0.2)):
        plain = P.detect_pockets(sc, PRIOR, PARAMS)
        result, gate = P.detect_pockets_expected(sc, PRIOR, PARAMS, None)
        assert gate == ()
        assert result.observation == plain.observation
        assert {k: v for k, v in vars(result.diagnostics).items() if k != "elapsed_s"} == {
            k: v for k, v in vars(plain.diagnostics).items() if k != "elapsed_s"}


def test_the_waiver_and_the_site_combine():
    sc = two_pallets(3.0, 0.95)  # the target fails only on the lower band here
    result, _ = P.detect_pockets_expected(sc, PRIOR, PARAMS, expected_at(3.0, 0.0), lower_evidence=False)
    assert result.observation.status == "valid"
    assert math.hypot(centre(result.observation)[0] - 3.0, centre(result.observation)[1]) < 0.02


@pytest.mark.parametrize("bad", [
    dict(radius_m=0.0), dict(radius_m=-0.1), dict(radius_m=math.nan), dict(axis_rad=0.0), dict(axis_rad=math.pi),
    dict(x_m=math.inf),
])
def test_an_expected_site_must_be_finite_and_positive(bad):
    args = dict(x_m=3.0, y_m=0.0, yaw_rad=0.0, radius_m=0.15, axis_rad=0.2)
    args.update(bad)
    with pytest.raises(ValueError):
        P.ExpectedSite(args["x_m"], args["y_m"], args["yaw_rad"], args["radius_m"], args["axis_rad"])
