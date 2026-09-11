import json
import math

import numpy as np
import pytest

from forklift_core.perception import pocket_observation as po


def valid_observation(**overrides):
    fields = dict(
        stamp_ns=1_700_000_000_000_000_000,
        clock_domain="ros_sim",
        frame_id="base_link",
        source_provenance="synthetic",
        status="valid",
        left=po.Pocket((1.7, 0.175, 0.15), 0.25, 0.20),
        right=po.Pocket((1.7, -0.175, 0.15), 0.25, 0.20),
        insertion_yaw_rad=0.0,
        position_sigma_m=0.01,
        yaw_sigma_rad=0.02,
        reason=None,
    )
    fields.update(overrides)
    return po.PocketObservation(**fields)


def test_valid_observation_round_trips_through_json():
    observation = valid_observation()
    encoded = observation.to_json()
    assert encoded["left"] == {
        "center_m": [1.7, 0.175, 0.15],
        "width_m": 0.25,
        "height_m": 0.2,
    }
    assert po.pocket_observation_from_json(encoded) == observation
    wire = json.loads(json.dumps(encoded, allow_nan=False))
    assert po.pocket_observation_from_json(wire) == observation


def test_numpy_scalars_are_normalised_to_python_types():
    observation = valid_observation(
        stamp_ns=np.int64(7),
        left=po.Pocket((np.float32(1.7), 0.175, 0.15), np.float32(0.25), 0.2),
        insertion_yaw_rad=np.float64(0.0),
        position_sigma_m=np.float32(0.01),
    )
    assert type(observation.stamp_ns) is int
    assert type(observation.left.width_m) is float
    json.dumps(observation.to_json(), allow_nan=False)
    with pytest.raises(ValueError):
        valid_observation(stamp_ns=np.bool_(True))


def test_non_valid_observation_round_trips_through_json():
    lost = valid_observation(
        status="invalid",
        left=None,
        right=None,
        insertion_yaw_rad=None,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason="both pockets occluded",
    )
    assert (
        po.pocket_observation_from_json(json.loads(json.dumps(lost.to_json()))) == lost
    )


def test_left_and_right_are_defined_by_the_pallet_left_axis_not_by_frame_y():
    # yaw = +0.5 rad: left pocket has larger dot product with l = (-sin, cos, 0).
    left = po.Pocket((2.1552, 0.2054, 0.15), 0.24, 0.20)
    right = po.Pocket((2.3182, -0.0930, 0.15), 0.24, 0.20)
    valid_observation(left=left, right=right, insertion_yaw_rad=0.5)
    with pytest.raises(ValueError):
        valid_observation(left=right, right=left, insertion_yaw_rad=0.5)


def test_half_turn_yaw_swaps_which_pocket_has_larger_frame_y():
    # psi = pi: the pallet faces -x, so the LEFT pocket has the SMALLER frame y.
    # A naive y comparison passes the wrong pair and rejects the right one.
    left = po.Pocket((1.7, -0.175, 0.15), 0.25, 0.2)
    right = po.Pocket((1.7, 0.175, 0.15), 0.25, 0.2)
    valid_observation(left=left, right=right, insertion_yaw_rad=math.pi)
    with pytest.raises(ValueError):
        valid_observation(left=right, right=left, insertion_yaw_rad=math.pi)


def test_overlapping_openings_are_rejected():
    with pytest.raises(ValueError):
        valid_observation(
            left=po.Pocket((1.7, 0.12, 0.15), 0.30, 0.2),
            right=po.Pocket((1.7, -0.12, 0.15), 0.30, 0.2),
        )  # separation 0.24 <= (0.30 + 0.30) / 2


def test_swapped_pockets_at_zero_yaw_are_rejected():
    with pytest.raises(ValueError):
        valid_observation(
            left=po.Pocket((1.7, -0.175, 0.15), 0.25, 0.2),
            right=po.Pocket((1.7, 0.175, 0.15), 0.25, 0.2),
        )


def test_non_valid_status_requires_reason_and_forbids_geometry():
    lost = valid_observation(
        status="no_pallet",
        left=None,
        right=None,
        insertion_yaw_rad=None,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason="no target pallet in scene",
    )
    assert lost.to_json()["left"] is None
    with pytest.raises(ValueError):
        valid_observation(status="invalid", reason="occluded")  # geometry still present
    with pytest.raises(ValueError):
        valid_observation(
            status="no_pallet",
            left=None,
            right=None,
            insertion_yaw_rad=None,
            position_sigma_m=None,
            yaw_sigma_rad=None,
            reason=None,
        )


def test_sigma_none_is_allowed_but_negative_or_nan_is_not():
    assert valid_observation(position_sigma_m=None).position_sigma_m is None
    for bad in (-0.001, math.nan, math.inf):
        with pytest.raises(ValueError):
            valid_observation(position_sigma_m=bad)


@pytest.mark.parametrize(
    "field,value",
    [
        ("frame_id", "camera_optical_frame"),
        ("clock_domain", "wall"),
        ("status", "lost"),
        ("source_provenance", "guess"),
        ("stamp_ns", -1),
        ("stamp_ns", True),
        ("stamp_ns", 1.5),
        ("insertion_yaw_rad", 4.0),
        ("insertion_yaw_rad", math.nan),
    ],
)
def test_enumerations_stamp_and_yaw_range_are_validated(field, value):
    with pytest.raises(ValueError):
        valid_observation(**{field: value})


def test_pocket_dimensions_and_center_must_be_finite_positive():
    with pytest.raises(ValueError):
        po.Pocket((1.7, 0.175, 0.15), 0.0, 0.2)
    with pytest.raises(ValueError):
        po.Pocket((1.7, math.nan, 0.15), 0.25, 0.2)


def test_pocket_separation_bounds():
    with pytest.raises(ValueError):
        valid_observation(
            left=po.Pocket((1.7, 0.01, 0.15), 0.25, 0.2),
            right=po.Pocket((1.7, -0.01, 0.15), 0.25, 0.2),
        )
    with pytest.raises(ValueError):
        valid_observation(
            left=po.Pocket((1.7, 1.5, 0.15), 0.25, 0.2),
            right=po.Pocket((1.7, -1.5, 0.15), 0.25, 0.2),
        )


def test_from_json_rejects_unknown_and_missing_keys():
    encoded = valid_observation().to_json()
    with pytest.raises(ValueError):
        po.pocket_observation_from_json({**encoded, "extra": 1})
    del encoded["reason"]
    with pytest.raises(ValueError):
        po.pocket_observation_from_json(encoded)


@pytest.mark.parametrize(
    "estimate,reference,expected",
    [
        (0.1, -0.1, 0.2),
        (math.pi - 0.1, -math.pi + 0.1, -0.2),
        (-math.pi + 0.1, math.pi - 0.1, 0.2),
        (math.pi, 0.0, math.pi),
        (0.0, math.pi, math.pi),
    ],
)
def test_yaw_difference_wraps_to_half_open_interval(estimate, reference, expected):
    assert po.yaw_difference_rad(estimate, reference) == pytest.approx(expected)


@pytest.mark.parametrize("field", ["left", "right", "insertion_yaw_rad"])
def test_valid_status_requires_each_geometry_field(field):
    with pytest.raises(ValueError):
        valid_observation(**{field: None})


@pytest.mark.parametrize("field", ["left", "right"])
def test_geometry_fields_require_pocket_objects(field):
    with pytest.raises(ValueError):
        valid_observation(**{field: {"center_m": [1, 0, 0]}})


@pytest.mark.parametrize("reason", ["unexpected", "", 1])
def test_valid_status_forbids_reason(reason):
    with pytest.raises(ValueError):
        valid_observation(reason=reason)


def lost_fields():
    return dict(
        status="no_pallet",
        left=None,
        right=None,
        insertion_yaw_rad=None,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason="no target pallet in scene",
    )


@pytest.mark.parametrize("status", ["invalid", "no_pallet"])
@pytest.mark.parametrize(
    "field", ["left", "right", "insertion_yaw_rad", "position_sigma_m", "yaw_sigma_rad"]
)
def test_non_valid_status_forbids_each_geometry_and_sigma_field(status, field):
    fields = lost_fields()
    fields.update(status=status, **{field: getattr(valid_observation(), field)})
    with pytest.raises(ValueError):
        valid_observation(**fields)


@pytest.mark.parametrize("reason", [None, "", "   ", 1, False])
def test_non_valid_status_requires_nonempty_string_reason(reason):
    with pytest.raises(ValueError):
        valid_observation(**{**lost_fields(), "reason": reason})


@pytest.mark.parametrize("field", ["position_sigma_m", "yaw_sigma_rad"])
@pytest.mark.parametrize("bad", [-0.01, math.nan, math.inf, True, "0"])
def test_each_sigma_requires_finite_nonnegative_real_or_none(field, bad):
    with pytest.raises(ValueError):
        valid_observation(**{field: bad})


def test_unknown_sigmas_remain_distinct_from_zero_sigmas_in_json():
    unknown = valid_observation(position_sigma_m=None, yaw_sigma_rad=None).to_json()
    exact = valid_observation(position_sigma_m=0, yaw_sigma_rad=0).to_json()
    assert unknown["position_sigma_m"] is None and unknown["yaw_sigma_rad"] is None
    assert exact["position_sigma_m"] == 0.0 and exact["yaw_sigma_rad"] == 0.0


@pytest.mark.parametrize(
    "center", [[1, 2], [[1, 2, 3]], [1, 2, math.inf], ["1", "2", "3"], [1j, 2, 3]]
)
def test_pocket_center_requires_finite_three_vector(center):
    with pytest.raises(ValueError):
        po.Pocket(center, 0.25, 0.2)


def test_pocket_copies_center_to_python_float_tuple():
    center = np.array([1.7, 0.175, 0.15])
    pocket = po.Pocket(center, np.float32(0.25), np.float64(0.2))
    center[:] = 0
    assert pocket.center_m == (1.7, 0.175, 0.15)
    assert all(type(value) is float for value in pocket.center_m)
    assert type(pocket.width_m) is float and type(pocket.height_m) is float


@pytest.mark.parametrize("field", ["width_m", "height_m"])
@pytest.mark.parametrize("bad", [0, -0.1, math.nan, math.inf, True, "0.2"])
def test_each_pocket_dimension_requires_finite_positive_real(field, bad):
    with pytest.raises(ValueError):
        po.Pocket(
            (1.7, 0.175, 0.15), **{**dict(width_m=0.25, height_m=0.2), field: bad}
        )


@pytest.mark.parametrize("distance", [0.05, 2.0])
def test_pocket_separation_endpoints_are_excluded_without_overlap(distance):
    with pytest.raises(ValueError):
        valid_observation(
            left=po.Pocket((0, distance / 2, 0), 0.01, 0.2),
            right=po.Pocket((0, -distance / 2, 0), 0.01, 0.2),
        )


def test_opening_edges_must_not_touch():
    with pytest.raises(ValueError):
        valid_observation(
            left=po.Pocket((0, 0.125, 0), 0.2, 0.2),
            right=po.Pocket((0, -0.125, 0), 0.3, 0.2),
        )


@pytest.mark.parametrize("yaw", [-math.pi, math.inf, True, "0"])
def test_yaw_rejects_excluded_endpoint_and_nonfinite_or_nonreal_values(yaw):
    with pytest.raises(ValueError):
        valid_observation(insertion_yaw_rad=yaw)


@pytest.mark.parametrize(
    "field,values",
    [
        ("clock_domain", ["ros_sim", "ros_system", "device", "synthetic"]),
        (
            "source_provenance",
            ["synthetic", "replay", "live", "synthetic_ground_truth"],
        ),
    ],
)
def test_all_documented_clock_domains_and_provenances_are_accepted(field, values):
    for value in values:
        assert getattr(valid_observation(**{field: value}), field) == value


@pytest.mark.parametrize(
    "field", ["clock_domain", "status", "source_provenance", "frame_id"]
)
def test_metadata_rejects_nonstring_values_with_value_error(field):
    with pytest.raises(ValueError):
        valid_observation(**{field: []})


def test_zero_stamp_and_normalised_yaw_and_sigmas_are_json_serialisable():
    observation = valid_observation(
        stamp_ns=np.uint64(0),
        insertion_yaw_rad=np.float32(0),
        position_sigma_m=np.float32(0.01),
        yaw_sigma_rad=np.float32(0.02),
    )
    assert observation.stamp_ns == 0 and type(observation.stamp_ns) is int
    for field in ("insertion_yaw_rad", "position_sigma_m", "yaw_sigma_rad"):
        assert type(getattr(observation, field)) is float
    json.dumps(observation.to_json(), allow_nan=False)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("change", ["missing", "extra", "not_object"])
def test_from_json_rejects_malformed_nested_pockets(side, change):
    encoded = valid_observation().to_json()
    if change == "missing":
        del encoded[side]["height_m"]
    elif change == "extra":
        encoded[side]["extra"] = 1
    else:
        encoded[side] = []
    with pytest.raises(ValueError):
        po.pocket_observation_from_json(encoded)


@pytest.mark.parametrize("obj", [None, [], "{}"])
def test_from_json_requires_object(obj):
    with pytest.raises(ValueError):
        po.pocket_observation_from_json(obj)


@pytest.mark.parametrize("bad", [math.nan, math.inf, True, "0"])
def test_yaw_difference_rejects_nonfinite_and_nonreal_arguments(bad):
    with pytest.raises(ValueError):
        po.yaw_difference_rad(bad, 0)
    with pytest.raises(ValueError):
        po.yaw_difference_rad(0, bad)


def test_yaw_difference_handles_multiple_turns():
    assert po.yaw_difference_rad(4 * math.pi + 0.1, -4 * math.pi) == pytest.approx(0.1)
