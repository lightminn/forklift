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


def test_the_standing_bound_is_the_largest_change_between_any_two_instants():
    # D5 v11 V13: zero command from 0.0 s to 0.05 s; the rear axle goes +0.19 mm then -0.19 mm
    # (0.38 mm apart -- a first-tick reference would read 0.19 mm), heading +-0.01 mrad; the
    # command turns on at 0.05 s and the motion after it is not read.
    t = np.arange(0.0, 0.1, 1 / 120)
    base = np.tile([0.34, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0], (len(t), 1))
    base[2, 1], base[4, 1] = 0.00019, -0.00019
    for k, yaw in ((3, 1e-5), (5, -1e-5)):
        base[k, 3], base[k, 6] = math.cos(yaw / 2), math.sin(yaw / 2)
    base[8:, 0] += 0.01  # moving after the command
    applied = np.where(t < 0.05 - 1e-9, 0.0, 0.05)
    out = EX.standing_motion(t, base, applied, 0.0, -0.34)
    assert out["ticks"] == 6 and out["span_s"] == [0.0, pytest.approx(5 / 120)]
    assert out["d_m"] == pytest.approx(0.00038 + 0.34 * 2e-5, abs=1e-9)  # base + offset * heading
    assert out["phi_rad"] == pytest.approx(2e-5, rel=1e-6)
    assert EX.standing_motion(t, base, np.full(len(t), 0.05), 0.0, -0.34) is None


# --- the correction-run odometry supplement (fork spacing plan, (3) (b) 11th draft) ---------------


def anchor_at(t, control=(0.0, 0.0, 0.0), truth=(0.0, 0.0, 0.0)):
    return {"time_s": t, "control_rear": list(control), "truth_rear": list(truth)}


def test_the_anchor_sample_pairs_sit_at_the_trackers_lookup_age():
    rows = control_rows(40, drift=lambda k: (0.0, 0.0, 0.0001 * k))
    # Capture window [9.95, 10.0 - TICK/3]: the anchor sample sits between two ticks, its control
    # heading 0.002 off the truth's.
    anchor = anchor_at(10.0 - TICK / 3, control=(0.0, 0.0, 0.002))
    e, psi = EX.anchor_running_max(rows, anchor, 9.95, 10.0 + 39 * TICK)
    # The first tick after the sample (10.0) is looked up at 10.0 - 9.95 = 0.05 s -> index 6.
    assert psi[6] == pytest.approx(0.002)
    assert psi[: 6].max() == 0.0
    # The last tick: heading error 0.0039 off, the anchor's 0.002 the other way.
    k = int(math.ceil((39 * TICK + 0.05) / TICK - 1e-6))
    assert psi[k] == pytest.approx(abs(0.0039 - 0.002))
    # Ages are not the tick difference from the first tick: the capture window counts.
    assert len(psi) == k + 1


def test_the_anchor_pairs_refuse_a_gap_a_late_record_or_an_anchor_before_its_window():
    rows = control_rows(30)
    end = 10.0 + 29 * TICK
    with pytest.raises(ValueError, match="one row per tick"):
        EX.anchor_running_max(np.delete(rows, 12, axis=0), anchor_at(10.0 - TICK / 2), 9.9, end)
    with pytest.raises(ValueError, match="both ends"):
        EX.anchor_running_max(rows[5:], anchor_at(10.0 - TICK / 2), 9.9, end)
    with pytest.raises(ValueError, match="before its aligned time"):
        EX.anchor_running_max(rows, anchor_at(9.8), 9.9, end)


class _Truth:
    def __init__(self, phases, t):
        self.phases = ["other", "observe", "approach", "insert"]
        self.phase = np.array([self.phases.index(p) for p in phases])
        self.t = np.asarray(t, dtype=float)


class _Run:
    def __init__(self, phases, starts, attempts, failure=None):
        self.key = "/runs/x"
        self.truth = _Truth(phases, 10.0 + np.arange(len(phases)) * TICK)
        self.result = {
            "near_tracking": {"starts": starts},
            "observation_attempts": attempts,
            "failure_reason": failure,
        }


def test_episodes_follow_tracker_builds_and_report_how_they_ended():
    t = lambda k: 10.0 + k * TICK  # noqa: E731
    phases = ["observe"] * 5 + ["approach"] * 10 + ["observe"] * 5 + ["approach"] * 5 + ["insert"] * 5 + ["other"] * 3
    attempts = [
        {"t_before_capture_s": t(1), "capture_control": anchor_at(t(2) - TICK / 2)},
        {"t_before_capture_s": t(3), "capture_control": anchor_at(t(4) - TICK / 2)},
        {"t_before_capture_s": t(16), "capture_control": anchor_at(t(19) - TICK / 2)},
    ]
    starts = [
        {"episode": 2, "attempt": 2, "aligned_s": t(3), "built_s": t(4)},
        {"episode": 3, "attempt": 3, "aligned_s": t(16), "built_s": t(19)},
    ]
    first, second = EX.supplement_episodes(_Run(phases, starts, attempts))
    assert first["span_s"] == [pytest.approx(t(5)), pytest.approx(t(14))]
    assert first["truncated"] and first["reason"] == "retry or realignment"
    assert second["span_s"] == [pytest.approx(t(20)), pytest.approx(t(29))]
    assert not second["truncated"] and second["ended"] == "insert"
    # A run that ends in the approach is truncated with its failure.
    (only,) = EX.supplement_episodes(
        _Run(phases[:12], starts[:1], attempts, failure="near_field_mismatch")
    )
    assert only["truncated"] and only["reason"] == "near_field_mismatch"
    # The episode's aligned time must be its capture's start.
    bad = [dict(starts[0], aligned_s=t(2))]
    with pytest.raises(ValueError, match="aligned time"):
        EX.supplement_episodes(_Run(phases, bad, attempts))


def test_the_supplement_specs_are_both_kinds_per_held_out_seed_signed_by_parity():
    assert len(EX.SUPPLEMENT_SPECS) == 18
    assert {s for s, _, _ in EX.SUPPLEMENT_SPECS} == set(range(9, 18))
    assert (10, "lateral", 0.02) in EX.SUPPLEMENT_SPECS and (10, "yaw", -0.015) in EX.SUPPLEMENT_SPECS
    assert (9, "lateral", -0.02) in EX.SUPPLEMENT_SPECS and (9, "yaw", 0.015) in EX.SUPPLEMENT_SPECS


def test_a_supplement_run_must_share_the_s2_contract_and_be_a_correction_run(monkeypatch):
    monkeypatch.setattr(EX.CAL, "check_hashes", lambda run: None)
    contract = {k: f"c-{k}" for k in (*EX.CAL.CONTRACT_KEYS, *EX.CAL.HASH_KEYS)}

    def run(**changes):
        args = {"near_tracking": True, "near_inject": "correction", "approach_straight_speed_mps": 0.055,
                # As the runner records it: the axis not injected is 0.0.
                "near_inject_lateral_m": 0.02, "near_inject_yaw_rad": 0.0, "pocket_check": False,
                "slam_noise_seed": 10}
        near = {**EX.SUPPLEMENT_INPUT_SHA256, "contract_problems": [], "tracker_config_problems": []}
        meta = {k: contract[k] for k in EX.CAL.CONTRACT_KEYS}
        result = {**{k: contract[k] for k in EX.CAL.HASH_KEYS}, "seed": 10}
        for key, value in changes.items():
            target = {"args": args, "near": near, "meta": meta, "result": result}[key.split("__")[0]]
            target[key.split("__")[1]] = value

        class Run:
            pass

        Run.key = "/runs/s"
        Run.meta = meta
        Run.result = {**result, "arguments": args, "near_tracking": near}
        return Run

    assert EX.check_supplement_run(run(), contract) == (10, "lateral", 0.02)
    assert EX.check_supplement_run(
        run(args__near_inject_lateral_m=0.0, args__near_inject_yaw_rad=-0.015), contract
    ) == (10, "yaw", -0.015)
    for changes, match in (
        ({"meta__fps": 1}, "meta fps"),
        ({"args__near_inject": "mismatch"}, "correction run"),
        ({"args__near_inject_yaw_rad": 0.015}, "one correction kind"),
        ({"args__near_inject_lateral_m": 0.0}, "one correction kind"),
        ({"args__slam_noise_seed": 0}, "slam noise seed"),
        ({"near__bounds_sha256": "0" * 64}, "fixed input"),
        ({"near__contract_problems": ["x differs"]}, "contract_problems"),
        ({"near__tracker_config_problems": None}, "tracker_config_problems"),
    ):
        with pytest.raises(ValueError, match=match):
            EX.check_supplement_run(run(**changes), contract)


def test_an_s2_run_needs_its_accepted_near_capture_to_anchor_the_odometry():
    class Run:
        key = "/runs/s2"
        result = {"near_capture": {"status": "failed", "near_attempt": None}}

    with pytest.raises(ValueError, match="accepted near capture"):
        EX.s2_anchor(Run)


def test_approach_ticks_before_the_first_tracker_build_are_refused():
    t = lambda k: 10.0 + k * TICK  # noqa: E731
    phases = ["observe"] * 3 + ["approach"] * 6 + ["insert"] * 3 + ["other"] * 2
    attempts = [{"t_before_capture_s": t(4), "capture_control": anchor_at(t(5) - TICK / 2)}]
    late = [{"episode": 2, "attempt": 1, "aligned_s": t(4), "built_s": t(5)}]
    with pytest.raises(ValueError, match="before the first tracker build"):
        EX.supplement_episodes(_Run(phases, late, attempts))


def test_the_table_reaches_the_runtime_row_of_the_oldest_anchor_pair():
    # A capture window starting 0.600000004 s before the span's last tick: the runtime reads that
    # pair at the first age at or above 0.600000003 s, which must exist.
    rows = control_rows(2, start=10.0)
    anchor = anchor_at(10.0 - TICK / 3)
    aligned = 10.0 + TICK - 0.600000004
    e, psi = EX.anchor_running_max(rows, anchor, aligned, 10.0 + TICK)
    grid = EX.age_grid(EX.merge_max([e]), EX.merge_max([psi]))
    pairs = [EX.anchor_pairs(rows, anchor, aligned, 10.0 + TICK)]
    assert EX.table_misses(grid, pairs) == []
    assert grid["age_s"][-1] >= 0.600000003


def test_table_misses_finds_a_pair_past_the_end_or_over_its_row():
    grid = {"age_s": [TICK, 2 * TICK], "e_m": [0.001, 0.002], "psi_rad": [0.001, 0.002]}
    ok = (np.array([TICK, 2 * TICK]), np.array([0.001, 0.002]), np.array([0.0, 0.0]))
    assert EX.table_misses(grid, [ok]) == []
    past = (np.array([2 * TICK + 1e-6]), np.array([0.0]), np.array([0.0]))
    over = (np.array([TICK]), np.array([0.0015]), np.array([0.0]))
    assert len(EX.table_misses(grid, [past, over])) == 2
