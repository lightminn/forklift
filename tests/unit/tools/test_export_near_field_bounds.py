import math

import numpy as np
import pytest

from tools import export_near_field_bounds as EX

TICK = 1.0 / 120.0


def stats(value):
    return {"n": 1, "max": value, "p95": value}


def side(lateral, along, width, wall_moving, wall_standing=None):
    return {
        "lateral_m": stats(lateral),
        "along_m": stats(along),
        "width_m": stats(width),
        "wall_m_moving": stats(wall_moving),
        "wall_m_standing": None if wall_standing is None else stats(wall_standing),
    }


def entry(valid, yaw=0.001, left=None, right=None):
    if not valid:
        return {"observed": 3, "valid": 0, "unbounded": True}
    return {
        "observed": valid,
        "valid": valid,
        "yaw_rad": stats(yaw),
        "left": left or side(0.004, 0.0003, 0.0075, 0.005),
        "right": right or side(0.003, 0.0002, 0.0025, 0.004),
    }


def summary(runs):
    return {
        "runs": {
            f"run{i}": {"bins": bins, "noisy_reference": {"bins": bins}}
            for i, bins in enumerate(runs)
        }
    }


def test_bins_take_the_largest_absolute_error_over_runs_pockets_and_motion():
    a = [
        {
            "bin_m": [1.0, 0.9],
            "frames": 4,
            "front": entry(4, yaw=0.0005),
            "roof": entry(0),
        }
    ]
    standing_wall = side(0.006, 0.0001, 0.0125, 0.003, 0.008)
    b = [
        {
            "bin_m": [1.0, 0.9],
            "frames": 4,
            "front": entry(2, yaw=0.0009, right=standing_wall),
            "roof": entry(0),
        }
    ]
    out = EX.observation_bounds(summary([a, b]))
    cell = out["front"][0]
    assert cell["valid"] == 6
    assert (
        cell["lateral_m"] == 0.006
        and cell["width_m"] == 0.0125
        and cell["yaw_rad"] == 0.0009
    )
    assert cell["wall_m"] == 0.008  # the standing wall of run b's right pocket
    assert cell["along_m"] == 0.0003
    assert out["roof"][0] == {"bin_m": [1.0, 0.9], "valid": 0, "unbounded": True}
    assert (
        "width_m"
        not in EX.observation_bounds(
            summary(
                [
                    [
                        {
                            "bin_m": [1.0, 0.9],
                            "frames": 1,
                            "front": entry(0),
                            "roof": entry(1),
                        }
                    ]
                ]
            )
        )["roof"][0]
    )


def test_an_empty_bin_without_source_entries_is_unbounded():
    out = EX.observation_bounds(
        summary([[{"bin_m": [0.3, 0.2], "frames": 0, "unbounded": True}]])
    )
    assert out["front"][0]["unbounded"] and out["roof"][0]["unbounded"]


def test_runs_over_different_ranges_merge_by_bin_edges_high_first():
    a = [
        {
            "bin_m": [2.6, 2.5],
            "frames": 2,
            "front": entry(2, yaw=0.0007),
            "roof": entry(0),
        },
        {
            "bin_m": [2.5, 2.4],
            "frames": 2,
            "front": entry(2, yaw=0.0001),
            "roof": entry(0),
        },
    ]
    b = [
        {
            "bin_m": [2.5, 2.4],
            "frames": 3,
            "front": entry(3, yaw=0.0004),
            "roof": entry(0),
        },
        {"bin_m": [0.2, 0.1], "frames": 1, "front": entry(0), "roof": entry(1)},
    ]
    out = EX.observation_bounds(summary([a, b]))
    assert [c["bin_m"] for c in out["front"]] == [[2.6, 2.5], [2.5, 2.4], [0.2, 0.1]]
    assert out["front"][1]["valid"] == 5 and out["front"][1]["yaw_rad"] == 0.0004
    assert out["front"][2]["unbounded"] and out["roof"][2]["valid"] == 1
    with pytest.raises(ValueError, match="grid"):
        EX.observation_bounds(summary([[{"bin_m": [0.25, 0.15], "frames": 0}]]))


def control_rows(n, drift=lambda k: (0.0, 0.0, 0.0), start=10.0):
    t = start + np.arange(n) * TICK
    truth = np.stack((0.05 * np.arange(n) * TICK, np.zeros(n), np.zeros(n)), axis=1)
    control = truth + np.array([drift(k) for k in range(n)])
    return np.column_stack((t, control, truth))


def test_exact_odometry_gives_zero_and_a_linear_drift_grows_with_age():
    e, psi = EX.odometry_running_max(control_rows(50), 10.0, 10.0 + 49 * TICK)
    assert np.allclose(e, 0) and np.allclose(psi, 0)
    rows = control_rows(50, drift=lambda k: (0.0, 0.001 * k, 0.0))
    e, _ = EX.odometry_running_max(rows, 10.0, 10.0 + 49 * TICK)
    assert e[0] == 0 and e[1] == pytest.approx(0.001) and e[10] == pytest.approx(0.010)


def test_the_window_maximum_finds_a_jump_anywhere_in_the_span():
    rows = control_rows(40, drift=lambda k: (0.0, 0.004 if k >= 30 else 0.0, 0.0))
    e, _ = EX.odometry_running_max(rows, 10.0, 10.0 + 39 * TICK)
    assert e[1] == pytest.approx(
        0.004
    )  # the pair (29, 30), not one anchored at the start


def test_a_gap_in_the_control_record_is_refused():
    rows = np.delete(control_rows(30), 12, axis=0)
    with pytest.raises(ValueError, match="one row per tick"):
        EX.odometry_running_max(rows, 10.0, 10.0 + 29 * TICK)


def test_the_age_grid_is_a_running_maximum_and_never_understates_a_lookup():
    e = np.zeros(200)
    e[73] = 0.02  # a peak between two coarse grid ages (0.6 and 0.7 s)
    merged = EX.merge_max([e, np.zeros(150)])
    grid = EX.age_grid(merged, merged)
    assert grid["age_s"][:2] == [pytest.approx(TICK), pytest.approx(2 * TICK)]
    assert grid["age_s"][59] == pytest.approx(0.5)
    ages = np.array(grid["age_s"])
    for k in range(1, 200):
        first = int(np.searchsorted(ages, k * TICK - 1e-9))
        assert grid["e_m"][first] >= merged[k]
    assert grid["age_s"][-1] == pytest.approx(199 * TICK)


def test_the_cadence_must_be_one_period_and_one_delay():
    reads = [
        {
            "phase": "approach",
            "result": "ok",
            "stamp_s": 1.0 + 0.1 * i,
            "read_s": 1.0 + 0.1 * i + TICK,
        }
        for i in range(5)
    ]
    assert EX.cadence({"a": reads}) == {
        "reads": 5,
        "render_period_ticks": 12,
        "read_delay_ticks": 1,
    }
    late = [dict(r) for r in reads]
    late[3]["read_s"] += TICK
    with pytest.raises(ValueError, match="not fixed"):
        EX.cadence({"a": late})


def test_build_refuses_a_failed_calibration_or_a_foreign_summary():
    cal = {"verdict": {"pass": False}}
    with pytest.raises(ValueError, match="did not pass"):
        EX.build(cal, "x", {}, "y", [], None, None, None, 0.0)
    cal = {"verdict": {"pass": True}}
    with pytest.raises(ValueError, match="another calibration"):
        EX.build(cal, "x", {"calibration_sha256": "z"}, "y", [], None, None, None, 0.0)
    assert math.isclose(EX.TICK_S, TICK)


def test_the_interval_bound_spans_the_bins_the_true_distance_can_reach():
    def cell(hi, lat, along, unbounded=False):
        if unbounded:
            return {"bin_m": [hi, round(hi - 0.1, 3)], "valid": 0, "unbounded": True}
        return {
            "bin_m": [hi, round(hi - 0.1, 3)],
            "valid": 3,
            "lateral_m": lat,
            "along_m": along,
            "yaw_rad": 0.001,
            "wall_m": lat,
            "width_m": 0.0075,
        }

    rows = [
        cell(0.6, 0.004, 0.001),
        cell(0.5, 0.003, 0.001),
        cell(0.4, 0, 0, unbounded=True),
    ]
    # 0.45 m: +-(1 mm + 10 mm) stays inside 0.4-0.5 -> its own bound only
    assert EX.interval_bound(rows, 0.45)["lateral_m"] == 0.003
    # 0.505 m: reaches 0.4-0.5 and 0.5-0.6 -> the larger
    assert EX.interval_bound(rows, 0.505)["lateral_m"] == 0.004
    # 0.405 m: reaches the unmeasured 0.3-0.4 bin -> unusable
    assert EX.interval_bound(rows, 0.405) is None
    # 0.595 m: reaches past the grid's top edge -> unusable
    assert EX.interval_bound(rows, 0.595) is None
    # a large along bound widens the interval over two bins
    wide = [cell(0.9, 0.0002, 0.02), cell(0.8, 0.0001, 0.02), cell(0.7, 0.0003, 0.02)]
    assert (
        EX.interval_bound(wide, 0.79)["lateral_m"] == 0.0002
    )  # 0.76-0.82: the 0.8-0.7 and 0.9-0.8 bins
    assert (
        EX.interval_bound(wide, 0.71)["lateral_m"] == 0.0003
    )  # 0.68-0.74: the 0.8-0.7 and 0.7-0.6 bins
    # the interval is clipped at the face plane (the grid's low edge), not refused
    low = [cell(0.2, 0.0001, 0.004), cell(0.1, 0.0002, 0.004)]
    assert EX.interval_bound(low, 0.005)["lateral_m"] == 0.0002


def test_the_near_capture_bound_covers_every_capture_and_every_bin_bound():
    def check(err, bound):
        return {
            "error": dict(zip(EX.COMPONENTS, err, strict=True)),
            "front_bound": dict(zip(EX.COMPONENTS, bound, strict=True)),
        }

    got = EX.near_capture_bound(
        [
            check(
                (0.004, 0.00034, 0.00065, 0.005, 0.0025),
                (0.0053, 0.0003, 0.00062, 0.0096, 0.0125),
            ),
            check(
                (0.002, 0.0001, 0.0001, 0.006, 0.0125),
                (0.0059, 0.00033, 0.00072, 0.0097, 0.0125),
            ),
        ]
    )
    assert got == {
        "lateral_m": 0.0059,
        "along_m": 0.00034,
        "yaw_rad": 0.00072,
        "wall_m": 0.0097,
        "width_m": 0.0125,
    }


def test_a_record_that_misses_either_end_of_the_span_is_refused():
    rows = control_rows(30, drift=lambda k: (0.0, 0.02 if k == 0 else 0.0, 0.0))
    e, _ = EX.odometry_running_max(rows, 10.0, 10.0 + 29 * TICK)
    assert e.max() == pytest.approx(0.02)
    with pytest.raises(ValueError, match="both ends"):
        EX.odometry_running_max(rows[1:], 10.0, 10.0 + 29 * TICK)
    with pytest.raises(ValueError, match="both ends"):
        EX.odometry_running_max(rows[:-1], 10.0, 10.0 + 29 * TICK)


def test_a_delay_or_period_off_the_tick_grid_is_refused():
    reads = [
        {
            "phase": "approach",
            "result": "ok",
            "stamp_s": 1.0 + 0.1 * i,
            "read_s": 1.0 + 0.1 * i + TICK,
        }
        for i in range(5)
    ]
    late = [dict(r) for r in reads]
    late[2]["read_s"] += 0.004  # 12.3 ms: rounds to one tick but is not on the grid
    with pytest.raises(ValueError, match="tick grid"):
        EX.cadence({"a": late})
    slow = [
        dict(r, stamp_s=r["stamp_s"] + 0.0004 * i, read_s=r["read_s"] + 0.0004 * i)
        for i, r in enumerate(reads)
    ]
    with pytest.raises(ValueError, match="tick grid"):
        EX.cadence({"a": slow})


def test_the_anchor_table_matches_the_calibrations_definition():
    from tools import d8b_calibration as CAL

    rows = control_rows(40, drift=lambda k: (0.0, 0.0005 * k, 0.0001 * k))
    span = (10.0, 10.0 + 39 * TICK)
    mine = EX.anchor_table(rows, *span)
    theirs = CAL.odometry_tables(rows, *span)["anchor"]
    for name in ("age_s", "distance_m", "e_m", "psi_rad"):
        assert mine[name] == pytest.approx(theirs[name], abs=1e-12)


def test_the_frame_manifest_changes_when_one_depth_frame_does(tmp_path):
    from types import SimpleNamespace

    (tmp_path / "pocket_frames").mkdir()
    for i in range(3):
        np.savez(
            tmp_path / f"pocket_frames/frame_{i:05d}.npz",
            depth_m=np.full((2, 2), float(i)),
        )
    reads = [{"file": f"frame_{i:05d}.npz"} for i in range(3)] + [{"file": None}]
    run = SimpleNamespace(directory=tmp_path, reads=reads, key="run")
    first = EX.frame_manifest(run)
    assert first["count"] == 3 and first["files"][0] == "pocket_frames/frame_00000.npz"
    np.savez(tmp_path / "pocket_frames/frame_00001.npz", depth_m=np.full((2, 2), 9.0))
    assert EX.frame_manifest(run)["sha256"] != first["sha256"]
    run.reads = reads[:1] * 2
    with pytest.raises(ValueError, match="twice"):
        EX.frame_manifest(run)


def test_recorded_paths_start_at_the_artifacts_directory():
    assert EX.artifact_path("/srv/x/artifacts/20261008_d8b_isaac/third/seed_1/run") == (
        "artifacts/20261008_d8b_isaac/third/seed_1/run"
    )
    assert EX.artifact_path("relative/run") == "relative/run"
