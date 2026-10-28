import pytest

from forklift_core.perception.near_field_bounds import (
    NearFieldBounds,
    bin_containing,
    interval_bound,
)

TICK = 1.0 / 120.0


def cell(hi, lateral=0.004, along=0.001, yaw=0.001, unbounded=False):
    lo = round(hi - 0.1, 3)
    if unbounded:
        return {"bin_m": [hi, lo], "valid": 0, "unbounded": True}
    return {
        "bin_m": [hi, lo],
        "valid": 5,
        "lateral_m": lateral,
        "along_m": along,
        "yaw_rad": yaw,
        "wall_m": lateral,
        "width_m": 0.0075,
    }


def data(**changes):
    out = {
        "schema": "near_field_bounds/v1",
        "latency": {
            "align_s": 0.0917,
            "max_s": 0.0989,
            "read_delay_s": TICK,
            "render_period_s": 0.1,
        },
        "observation": {
            "front": [cell(0.6), cell(0.5), cell(0.4, unbounded=True)],
            "roof": [cell(0.2, along=0.02), cell(0.1, along=0.02)],
        },
        "odometry": {
            "age_s": [TICK, 2 * TICK, 0.1, 0.2],
            "e_m": [0.0005, 0.001, 0.0035, 0.005],
            "psi_rad": [0.0002, 0.0004, 0.0019, 0.0035],
        },
        "near_capture": {
            "bound": {
                "lateral_m": 0.0062,
                "along_m": 0.0003,
                "yaw_rad": 0.0007,
                "wall_m": 0.0097,
                "width_m": 0.0125,
            }
        },
    }
    out.update(changes)
    return out


def test_the_odometry_lookup_takes_the_first_stored_age_at_or_above_the_query():
    b = NearFieldBounds.from_dict(data())
    assert b.odometry(0.0) == (0.0, 0.0)
    assert b.odometry(TICK) == (0.0005, 0.0002)
    assert b.odometry(0.05) == (
        0.0035,
        0.0019,
    )  # between 2 ticks and 0.1 s: the 0.1 s value
    assert b.odometry(0.2) == (0.005, 0.0035)
    assert b.odometry(0.21) is None
    with pytest.raises(ValueError):
        b.odometry(-0.1)


def test_the_observation_lookup_refuses_an_interval_that_reaches_an_unmeasured_bin():
    b = NearFieldBounds.from_dict(data())
    assert b.observation("front", 0.45)["lateral_m"] == 0.004
    assert (
        b.observation("front", 0.405) is None
    )  # +-11 mm reaches the unmeasured 0.3-0.4 bin
    assert b.observation("front", 0.35) is None
    assert b.observation("roof", 0.005) is not None  # clipped at the face plane


def test_a_file_that_is_not_a_running_maximum_or_has_a_bad_band_is_refused():
    with pytest.raises(ValueError, match="running maximum"):
        NearFieldBounds.from_dict(
            data(
                odometry={"age_s": [0.1, 0.2], "e_m": [0.004, 0.003], "psi_rad": [0, 0]}
            )
        )
    with pytest.raises(ValueError, match="increase"):
        NearFieldBounds.from_dict(
            data(odometry={"age_s": [0.2, 0.1], "e_m": [0, 0], "psi_rad": [0, 0]})
        )
    with pytest.raises(ValueError, match="band"):
        NearFieldBounds.from_dict(
            data(
                latency={
                    "align_s": 0.1,
                    "max_s": 0.09,
                    "read_delay_s": 0,
                    "render_period_s": 0.1,
                }
            )
        )
    with pytest.raises(ValueError, match="near_field_bounds"):
        NearFieldBounds.from_dict(data(schema="other"))


def test_the_interval_spans_two_bins_when_the_along_bound_is_wide():
    rows = [
        cell(0.3, lateral=0.0003, along=0.02),
        cell(0.2, lateral=0.0001, along=0.02),
        cell(0.1, lateral=0.0002, along=0.02),
    ]
    assert (
        interval_bound(rows, 0.19)["lateral_m"] == 0.0003
    )  # 0.16-0.22 touches 0.2-0.3
    assert (
        interval_bound(rows, 0.11)["lateral_m"] == 0.0002
    )  # 0.08-0.14 touches 0.0-0.1


def test_a_bin_with_a_large_along_error_is_reached_from_its_neighbour():
    # Codex re-review counterexample: true 0.490 m in a bin whose along bound is 40 mm,
    # observed 0.530 m in a bin whose along bound is 1 mm.
    rows = [
        cell(0.6, lateral=0.001, along=0.001),
        cell(0.5, lateral=0.009, along=0.040),
        cell(0.4, lateral=0.002, along=0.001),
        cell(0.3, lateral=0.002, along=0.001),
    ]
    assert interval_bound(rows, 0.53)["lateral_m"] == 0.009


def test_a_grid_that_does_not_start_at_the_face_hides_nothing_below_it():
    rows = [cell(0.5), cell(0.6)]
    assert interval_bound(rows, 0.405) is None  # below 0.4 m is missing, not the face
    assert interval_bound(rows, 0.45) is not None


def test_bin_lookup_uses_half_open_bins():
    rows = [{"bin_m": [1.0, 0.9]}, {"bin_m": [0.9, 0.8]}]
    assert bin_containing(rows, 0.9)["bin_m"] == [1.0, 0.9]
    assert bin_containing(rows, 0.85)["bin_m"] == [0.9, 0.8]
    assert bin_containing(rows, 1.0) is None


def test_the_committed_bounds_load_with_the_d8b_values():
    from pathlib import Path

    path = (
        Path(__file__).resolve().parents[3] / "config/near_field_bounds_measured.json"
    )
    b = NearFieldBounds.load(path)
    # The EPAL 6 fork spacing plan's S2 re-measure after fixes (1) and (3) (0.3725 m forks, 2026-10-14,
    # batch s2v3 with the correction-run odometry supplement sup: e and psi only).
    assert b.align_latency_s == pytest.approx(0.091668, abs=1e-6)
    assert b.max_latency_s == pytest.approx(0.098850, abs=1e-6)
    assert b.read_delay_s == pytest.approx(
        1 / 120
    ) and b.render_period_s == pytest.approx(0.1)
    assert b.odometry(0.1) == pytest.approx((0.003879, 0.001798), abs=1e-6)
    # The supplement's long ages: S2 alone saturated at 12.233 mrad and 30.74 mm.
    assert b.odometry(30.0) == pytest.approx((0.031382, 0.014579), abs=1e-6)
    assert b.observation("front", 2.5)["lateral_m"] == pytest.approx(0.006191, abs=1e-6)
    # 0.411 m now returns a bound: the lookup's widened edge (along bound + 10 mm) moved from 0.411173
    # to 0.410968 m; the 0.5-0.4 m bin itself had valid reads in both measurements.
    assert b.observation("front", 0.411)["lateral_m"] == pytest.approx(
        0.003769, abs=1e-6
    )
    assert b.observation("front", 0.39) is None and b.observation("roof", 1.76) is None
    assert b.near_capture["lateral_m"] == pytest.approx(0.006191, abs=1e-6)
    assert b.condition["video"] is False


# docs/plans/2026-10-28-near-front-range.md 3.2: the front's operating range ------------------------------------
def ranged(min_m=0.4, **extra):
    d = data()
    d["observation"]["front"] = [cell(0.6), cell(0.5)]
    d["observation"]["front_range"] = {"min_distance_m": min_m, "rule": "fork tip, up to the grid",
                                       "inputs": {"fork_tip_x_m": 0.95, "camera_x_m": 0.619}, "excluded": {}, **extra}
    return d


def test_below_the_floor_is_outside_the_range_not_missing():
    rows = [cell(0.5), cell(0.6)]
    assert interval_bound(rows, 0.405, floor_m=0.4) is not None
    assert interval_bound(rows, 0.405) is None  # floor 0: as today
    assert interval_bound(rows, 0.395, floor_m=0.4) is None  # below the floor: no bin can hold it
    # a hole above the floor is still a hole
    assert interval_bound([cell(0.7), cell(0.5)], 0.505, floor_m=0.4) is None


def test_the_front_range_is_read_and_applied_to_the_front_only():
    b = NearFieldBounds.from_dict(ranged())
    assert b.front_min_distance_m == pytest.approx(0.4)
    assert b.observation("front", 0.405) is not None
    assert NearFieldBounds.from_dict(data()).front_min_distance_m is None  # no range: no range watch


@pytest.mark.parametrize("min_m", [0.35, 0.6, -0.1, float("nan")])
def test_a_front_range_off_the_grid_or_above_the_first_row_is_refused(min_m):
    with pytest.raises(ValueError, match="front_range"):
        NearFieldBounds.from_dict(ranged(min_m=min_m))


def test_a_front_row_below_the_range_is_refused():
    d = ranged()
    d["observation"]["front"].append(cell(0.4))  # the 0.3-0.4 bin under a 0.4 m range
    with pytest.raises(ValueError, match="front_range"):
        NearFieldBounds.from_dict(d)
