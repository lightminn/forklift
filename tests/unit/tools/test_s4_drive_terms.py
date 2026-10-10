import copy
from types import SimpleNamespace

import numpy as np
import pytest

from tools import s4_drive_terms as DT

TICK = 1.0 / 120.0


def run(n, v_true, v_cmd, ctrl_scale=1.0):
    """Control rows at loop ticks t_i; the applied command of interval [t_i, t_i+1] is
    recorded at t_i+1, as the runner's recorder writes it after the physics step."""
    t = np.arange(n) * TICK
    x_true = np.concatenate(([0.0], np.cumsum(np.full(n - 1, v_true) * TICK)))
    control = np.column_stack(
        (
            t,
            x_true * ctrl_scale,
            np.zeros(n),
            np.zeros(n),
            x_true,
            np.zeros(n),
            np.zeros(n),
        )
    )
    return control, t + TICK, np.full(n, v_cmd)


def test_the_window_maximum_is_a_running_maximum_over_window_lengths():
    out = DT.running_window_max(np.array([0.0, 2.0, -1.0, 3.0]), 4)
    assert out.tolist() == [0.0, 3.0, 3.0, 4.0, 4.0]


def test_exact_odometry_and_an_exact_drive_give_zero_terms():
    control, t, cmd = run(200, 0.06, 0.06)
    terms = DT.drive_terms(control, t, cmd, 0.0, control[-1, 0])
    assert max(terms["delta_len_m"]) == pytest.approx(0.0, abs=1e-12)
    assert max(terms["delta_s_m"]) == pytest.approx(0.0, abs=1e-12)
    assert terms["delta_v_mps"] == pytest.approx(0.0, abs=1e-12)


def test_odometry_that_undercounts_shows_in_delta_len_and_a_fast_drive_in_delta_s():
    control, t, cmd = run(200, 0.06, 0.05, ctrl_scale=0.9)
    terms = DT.drive_terms(control, t, cmd, 0.0, control[-1, 0], max_window_s=0.5)
    k = 12  # 0.1 s
    # the control path is 0.9 of the truth: 10 % of 0.06 m/s over 0.1 s
    assert terms["delta_len_m"][k] == pytest.approx(0.1 * 0.06 * k * TICK, rel=1e-6)
    assert terms["delta_s_m"][k] == pytest.approx(0.01 * k * TICK, rel=1e-6)
    assert terms["delta_v_mps"] == pytest.approx(0.01, rel=1e-6)


def test_each_interval_is_paired_with_the_command_recorded_at_its_end():
    # The command steps from 0.05 to 0.06 in the record stamped t_60 + tick, which drove
    # [t_60, t_61]; the truck follows exactly from that interval on. Paired with the start
    # stamp instead, interval 59 would carry 0.06 against a truth of 0.05 and interval 60
    # 0.05 against 0.06.
    n = 120
    t = np.arange(n) * TICK
    speed = np.where(np.arange(n - 1) >= 60, 0.06, 0.05)
    x = np.concatenate(([0.0], np.cumsum(speed * TICK)))
    control = np.column_stack(
        (t, x, np.zeros(n), np.zeros(n), x, np.zeros(n), np.zeros(n))
    )
    stamps = t + TICK
    cmd = np.where(np.arange(n) >= 60, 0.06, 0.05)
    terms = DT.drive_terms(control, stamps, cmd, 0.0, t[-1])
    assert max(terms["delta_s_m"]) == pytest.approx(0.0, abs=1e-12)
    assert terms["delta_v_mps"] == pytest.approx(0.0, abs=1e-12)


def test_the_speed_is_the_unsigned_path_speed_and_delta_v_is_never_negative():
    # Reversing: the truth moves backwards at 0.05 under a -0.05 command -- no excess.
    control, t, cmd = run(100, -0.05, -0.05)
    terms = DT.drive_terms(control, t, cmd, 0.0, control[-1, 0])
    assert terms["speed_true_max_mps"] == pytest.approx(0.05)
    assert max(terms["delta_s_m"]) == pytest.approx(0.0, abs=1e-12)
    assert terms["delta_v_mps"] == pytest.approx(0.0, abs=1e-12)
    # A truck slower than its command everywhere gives delta_v 0, not a negative bound.
    control, t, cmd = run(100, 0.04, 0.05)
    assert DT.drive_terms(control, t, cmd, 0.0, control[-1, 0])["delta_v_mps"] == 0.0


def test_a_command_drop_is_covered_by_the_recent_maximum_not_the_current_command():
    control, t, cmd = run(120, 0.055, 0.055)
    cmd[60:] = (
        0.0  # the command drops to zero, the truck still moves at 0.055 for the rest
    )
    terms = DT.drive_terms(control, t, cmd, 0.0, control[-1, 0], max_window_s=0.5)
    # 0.3 s after the drop the recent maximum is zero too: the over-speed shows from there.
    assert terms["delta_v_mps"] == pytest.approx(0.055, rel=1e-6)
    assert max(terms["delta_s_m"]) > 0.0


def test_a_missing_command_or_a_gap_in_the_control_record_is_refused():
    control, t, cmd = run(50, 0.05, 0.05)
    with pytest.raises(ValueError, match="applied command"):
        DT.drive_terms(control, t[::2], cmd[::2], 0.0, control[-1, 0])
    with pytest.raises(ValueError, match="one row per tick"):
        DT.drive_terms(np.delete(control, 10, axis=0), t, cmd, 0.0, control[-1, 0])


def test_a_control_path_running_ahead_of_the_command_shows_in_the_control_excess():
    # Odometry reads 10 % long against a truck that follows its 0.05 command exactly: the
    # control path gains 0.005 m/s on the command, which d must carry (S4a-1 smoke).
    control, t, cmd = run(200, 0.05, 0.05, ctrl_scale=1.1)
    terms = DT.drive_terms(control, t, cmd, 0.0, control[-1, 0], max_window_s=0.5)
    assert terms["control_excess_m"][12] == pytest.approx(0.005 * 12 * TICK, rel=1e-6)
    assert max(terms["delta_len_m"]) == pytest.approx(
        0.0, abs=1e-12
    )  # truth is shorter
    zero = [0.0] * 200
    excess = [0.005 * k * TICK for k in range(200)]
    assert DT.cruise(zero, excess, zero, 0.0, 0.10727, 0.1, v_max=0.2) < DT.cruise(
        zero, zero, zero, 0.0, 0.10727, 0.1, v_max=0.2
    )


def test_the_budget_is_checked_over_the_whole_render_cycle():
    zero = [0.0] * 200
    period, age = 0.1, 0.10727
    # Odometry error that grows with the pixel age: the worst tick is late in the cycle,
    # not the arrival.
    growing = [0.0005 * k for k in range(200)]
    worst, u = DT.budget_cycle(0.05, growing, zero, zero, 0.0, age, period)
    at_arrival = (
        0.05 * age
        + DT.term_at(growing, age)
        + 0.05 * (period + DT.STOP_LATENCY_S)
        + 0.05**2 / 3
    )
    assert worst > at_arrival
    assert u == pytest.approx(period)
    # Without drive terms the cycle's worst tick is the arrival (vbar = v, d and travel
    # trade one for one) and equals the arrival budget.
    worst, u = DT.budget_cycle(0.05, zero, zero, zero, 0.0, age, period)
    assert worst == pytest.approx(
        0.05 * (age + period + DT.STOP_LATENCY_S) + 0.05**2 / 3
    )


def test_the_cruise_is_the_largest_step_that_keeps_the_budget_and_never_past_the_cap():
    zero = [0.0] * 200
    v = DT.cruise(zero, zero, zero, 0.0, 0.10727, 0.1, v_max=0.2)
    assert DT.budget_cycle(v, zero, zero, zero, 0.0, 0.10727, 0.1)[0] <= 0.03
    assert (
        DT.budget_cycle(v + DT.STEP_MPS, zero, zero, zero, 0.0, 0.10727, 0.1)[0] > 0.03
    )
    assert v == pytest.approx(0.075)  # the D8c 3판 value without the drive terms
    assert (
        DT.cruise([0.004] * 200, zero, [0.002] * 200, 0.01, 0.10727, 0.1, v_max=0.2) < v
    )
    assert DT.cruise(zero, zero, zero, 0.0, 0.10727, 0.1, v_max=0.055) == pytest.approx(
        0.055
    )


# The measurement matrix -------------------------------------------------------------
SOURCES = {
    "forklift_core/localization/slam_pose.py": "a" * 64,
    "forklift_core/perception/near_field_tracking.py": "b" * 64,
    "sim/isaac/run_transport.py": "c" * 64,
}
REVIEWED = {
    "forklift_core/perception/near_field_tracking.py": "7" * 64,
    "forklift_core/perception/near_field_bounds.py": "8" * 64,
    "sim/isaac/run_transport.py": "9" * 64,
}
MANIFEST = {"snapshot": "p5_s4m_test", "allowed_sources": dict(REVIEWED)}
CONDITION = {
    "video": False,
    "fps": 60,
    "mount": {"depth_quantize_mm": 1},
    "camera": {"fx": 1.0},
    "pallet_geometry": "config/pallet_geometry_epal6.yaml",
    "forklift_urdf": "f.urdf",
    "rear_axle_offset_m": -0.34,
    "source_sha256": {
        "source_sha256": dict(SOURCES),
        "pallet_geometry_sha256": "d" * 64,
        "forklift_urdf_sha256": "e" * 64,
        "pallet_urdf_sha256": "f" * 64,
    },
}
BOUNDS = {"condition": CONDITION}
SPAN = (0.0, 20.0)
# Four stops, each 0.6 s standing, 1 s apart from 2 s on; the command is 0.05 elsewhere.
STOPS = [
    {
        "gap_m": g,
        "trigger_s": 2.0 + 3 * i,
        "d_est_m": g - 0.0004,
        "still_since_s": 2.2 + 3 * i,
        "end_s": 2.8 + 3 * i,
    }
    for i, g in enumerate(DT.REQUIRED_GAPS_M)
]


def commands(stops):
    stamps = np.arange(1, int(SPAN[1] / TICK) + 1) * TICK
    applied = np.full(len(stamps), 0.05)
    for stop in stops:
        held = (stamps >= stop["trigger_s"] + TICK - 1e-9) & (
            stamps <= stop["end_s"] + 1e-9
        )
        applied[held] = 0.0
    return stamps, applied


# Two snapshot trees with every required input (filled by the autouse fixture below).
SNAPSHOTS = {}
INPUTS = {
    "base_scene": "scene/scene.usda",
    "pallet_urdf": "sim/models/epal6_pallet/pallet.urdf",
    "pallet_geometry": "config/pallet_geometry_epal6.yaml",
    "forklift_urdf": "sim/models/dls08_measured/forklift.urdf",
    "settings": "config/isaac_transport_measured.yaml",
    "factory_layout": "config/factory_south_hall.yaml",
    "lidar": "config/isaac_slam_lidar.yaml",
    "obstacle_layer": "config/obstacle_layer_single.yaml",
    "pallet_prior": "config/pallet_prior_epal6.yaml",
}
ABSOLUTE = (
    "factory_layout",
    "lidar",
)  # given as absolute snapshot paths, as the runs do


@pytest.fixture(autouse=True)
def snapshot_trees(tmp_path):
    for name in ("p5_d3e5b36", "p5_s4m_wip2"):
        root = tmp_path / "snapshots" / name
        for rel in INPUTS.values():
            (root / rel).parent.mkdir(parents=True, exist_ok=True)
            (root / rel).write_text(f"{rel}: 1\n")
        (root / "config/obstacle_layer_single.yaml").write_text(
            "# everything else is config/obstacle_layer.yaml\nage_table: config/obstacle_odometry_age.json\n"
        )
        (root / "config/obstacle_layer.yaml").write_text("clear_max_height_m: 1.1\n")
        (root / "config/obstacle_odometry_age.json").write_text(
            '{"position_m": 0.001}\n'
        )
        (root / "config/factory_south_hall.yaml").write_text(
            "# Simple_Warehouse/full_warehouse.usd\n"
        )
        SNAPSHOTS[name] = root
    yield
    SNAPSHOTS.clear()


def input_arguments(snapshot):
    root = SNAPSHOTS[snapshot]
    return {k: str(root / v) if k in ABSOLUTE else v for k, v in INPUTS.items()}


def settings(seed, speed):
    return {
        "settings_synthetic": {"wheel_torque_nm": 3.0},
        "detector_params": {"x": 1},
        "tracker_configs": {"approach": {"cruise_speed_mps": speed}},
        "planner_config": {"p": 1},
        "travel_planner_config": {"p": 2},
        "slam_feedback": {
            "socket": f"/a/{seed}/slam.sock",
            "noise_seed": seed,
            "noise_spec": None,
        },
        "arguments": {
            "slam_noise_seed": seed,
            "approach_straight_speed_mps": speed,
            **input_arguments("p5_d3e5b36"),
            "output": "/d8b/run",
            "record_dwell_gaps": [2.25] if speed == 0.055 else None,
        },
    }


def fake_run(seed, speed):
    meta = {k: copy.deepcopy(CONDITION[k]) for k in DT.CONDITION_KEYS}
    meta.update(
        pocket_check=False,
        approach_straight_speed_mps=speed,
        safety_stop_gaps_m=list(DT.REQUIRED_GAPS_M),
        safety_stops=copy.deepcopy(STOPS),
    )
    result = {
        "seed": seed,
        "transitions": [
            {"from": "approach", "to": "insert"},
            {"from": "insert", "to": "lift"},
        ],
        "source_sha256": {**SOURCES, **REVIEWED},
        "script_sha256": "9" * 64,
        **{k: CONDITION["source_sha256"][k] for k in DT.ASSET_HASH_KEYS},
        **copy.deepcopy(settings(seed, speed)),
    }
    result["slam_feedback"]["socket"] = "/s4/slam.sock"
    result["arguments"].update(
        **input_arguments("p5_s4m_wip2"),
        output="/s4/run",
        record_dwell_gaps=None,
        measure_safety_stops_m=list(DT.REQUIRED_GAPS_M),
    )
    return SimpleNamespace(
        meta=meta,
        result=result,
        dwells=[],
        key=f"s{seed}_{speed}",
        phase_span=lambda: SPAN,
    )


def matrix():
    return {
        f"s{s}_{v}": fake_run(s, v)
        for s in DT.REQUIRED_SEEDS
        for v in DT.REQUIRED_SPEEDS
    }


def references():
    return {
        (s, v): settings(s, v) for s in DT.REQUIRED_SEEDS for v in DT.REQUIRED_SPEEDS
    }


def problems_of(runs, refs=None, manifest=MANIFEST):
    cmds = {
        k: commands(STOPS) for k in runs
    }  # the commands as made; the records may be spoiled
    return DT.matrix_problems(
        runs, BOUNDS, references() if refs is None else refs, manifest, cmds
    )


@pytest.fixture(autouse=True)
def lenient_contract(monkeypatch):
    from tools import d8b_calibration as CAL

    def check(runs):  # the cross-run code hashes, by content
        first = runs[0].result
        for r in runs[1:]:
            if r.result["source_sha256"] != first["source_sha256"]:
                raise ValueError(f"{r.key}: source_sha256 differs")

    monkeypatch.setattr(CAL, "check_contract", check)


def test_the_full_matrix_has_no_problems():
    assert problems_of(matrix()) == []


def test_only_the_manifest_files_may_differ_and_only_at_their_reviewed_hashes():
    runs = matrix()
    for r in runs.values():  # an unreviewed runner revision, the same in all six
        r.result["source_sha256"]["sim/isaac/run_transport.py"] = "0" * 64
    assert any(
        "run_transport.py is not the reviewed revision" in p for p in problems_of(runs)
    )
    runs = matrix()
    for r in runs.values():  # a new module the manifest does not name
        r.result["source_sha256"][
            "forklift_core/perception/near_field_new_runtime.py"
        ] = "1" * 64
    assert any("near_field_new_runtime.py differs" in p for p in problems_of(runs))
    runs = matrix()
    for r in runs.values():
        r.result["source_sha256"]["forklift_core/localization/slam_pose.py"] = "3" * 64
    assert any("slam_pose.py differs" in p for p in problems_of(runs))


def test_arguments_and_settings_must_match_the_d8b_run_of_the_same_seed_and_speed():
    runs = matrix()
    runs["s3_0.08"].result["slam_feedback"]["noise_spec"] = "3sigma"
    runs["s1_0.055"].result["settings_synthetic"]["wheel_torque_nm"] = 0.01
    runs["s5_0.055"].result["tracker_configs"]["approach"]["cruise_speed_mps"] = 0.001
    runs["s5_0.08"].result["arguments"]["lidar"] = (
        "/home/projects/forklift/snapshots/x/config/other.yaml"
    )
    problems = problems_of(runs)
    for message in (
        "s3_0.08: slam_feedback",
        "s1_0.055: settings_synthetic",
        "s5_0.055: tracker_configs",
        "s5_0.08: argument lidar",
    ):
        assert any(message in p for p in problems), (message, problems)
    refs = references()
    del refs[(1, 0.08)]
    assert any(
        "no D8b calibration run at seed 1, 0.08" in p
        for p in problems_of(matrix(), refs)
    )


def test_files_named_by_arguments_are_compared_by_content():
    assert problems_of(matrix()) == []
    (SNAPSHOTS["p5_s4m_wip2"] / "config/isaac_slam_lidar.yaml").write_text(
        "beams: 100\n"
    )
    (SNAPSHOTS["p5_s4m_wip2"] / "config/pallet_prior_epal6.yaml").write_text("x: 2\n")
    problems = problems_of(matrix())
    assert any("argument lidar differs" in p for p in problems)
    assert any("argument pallet_prior differs" in p for p in problems)


def test_a_required_input_missing_on_either_or_both_sides_is_refused():
    (SNAPSHOTS["p5_s4m_wip2"] / "config/pallet_prior_epal6.yaml").unlink()
    assert any(
        "argument pallet_prior cannot be read" in p for p in problems_of(matrix())
    )
    # Codex 4th review: both sides gone must not pass either.
    (SNAPSHOTS["p5_d3e5b36"] / "config/pallet_prior_epal6.yaml").unlink()
    assert any(
        "argument pallet_prior cannot be read" in p for p in problems_of(matrix())
    )


def test_files_a_config_names_as_values_are_followed_and_required():
    # Codex 4th review: obstacle_layer_single.yaml reads config/obstacle_odometry_age.json.
    (SNAPSHOTS["p5_s4m_wip2"] / "config/obstacle_odometry_age.json").write_text(
        '{"position_m": 0.1}\n'
    )
    problems = problems_of(matrix())
    assert any("file config/obstacle_odometry_age.json differs" in p for p in problems)
    # Codex 5th review: gone on both sides must not pass.
    for name in SNAPSHOTS:
        (SNAPSHOTS[name] / "config/obstacle_odometry_age.json").unlink()
    assert any(
        "file config/obstacle_odometry_age.json cannot be read" in p
        for p in problems_of(matrix())
    )


def test_paths_mentioned_only_in_comments_are_not_inputs():
    files = DT.input_files(input_arguments("p5_s4m_wip2"), SNAPSHOTS["p5_s4m_wip2"])
    assert "file config/obstacle_odometry_age.json" in files
    assert not any("obstacle_layer.yaml" in k or "full_warehouse" in k for k in files)
    (SNAPSHOTS["p5_s4m_wip2"] / "config/obstacle_layer.yaml").unlink()
    assert problems_of(matrix()) == []
    assert DT.config_paths(
        {"a": ["x.json", {"b": "dir/y.yaml"}], "c": "not a path", "d": 3}
    ) == [
        "x.json",
        "dir/y.yaml",
    ]


def test_recorded_lidar_factory_and_scene_must_match_the_d8b_run():
    runs, refs = matrix(), references()
    for ref in refs.values():
        ref.update(
            lidar_synthetic={"beam_count": 1600},
            factory={"layout": "/a/x.yaml", "layout_sha256": "1"},
        )
    for r in runs.values():
        r.result.update(
            lidar_synthetic={"beam_count": 1600},
            factory={"layout": "/b/x.yaml", "layout_sha256": "1"},
        )
    assert problems_of(runs, refs) == []
    runs["s3_0.055"].result["lidar_synthetic"] = {"beam_count": 100}
    runs["s5_0.08"].result["factory"]["layout_sha256"] = "2"
    problems = problems_of(runs, refs)
    assert any("s3_0.055: lidar_synthetic differs" in p for p in problems)
    assert any("s5_0.08: factory differs" in p for p in problems)


def test_the_snapshot_part_of_an_argument_path_is_ignored():
    assert (
        DT.snapshot_relative("/home/p/snapshots/abc/config/x.yaml") == "config/x.yaml"
    )
    assert DT.snapshot_relative("config/x.yaml") == "config/x.yaml"


def spoil_stop(index, **changes):
    def spoil(runs):
        runs["s1_0.055"].meta["safety_stops"][index].update(changes)

    return spoil


@pytest.mark.parametrize(
    "spoil, message",
    [
        (lambda r: r.update(drop="s3_0.08"), "seed 3 at 0.08 m/s: 0 runs"),
        (lambda r: r["s1_0.055"].meta.update(pocket_check=True), "D5 is on"),
        (lambda r: r["s1_0.055"].dwells.append({"kind": "gap"}), "dwells"),
        (
            lambda r: r["s1_0.055"].result.update(transitions=[]),
            "insertion did not finish",
        ),
        (spoil_stop(2, end_s=None), "the 1.0 m stop is incomplete"),
        (spoil_stop(1, gap_m=9.0), "safety stops at"),
        (spoil_stop(3, still_since_s=12.0, end_s=10.0), "out of order"),
        (spoil_stop(0, end_s=2.5), "did not stand"),
        (
            lambda r: r["s1_0.055"].meta.update(safety_stop_gaps_m=[2.0, 1.0]),
            "safety stop gaps",
        ),
        (
            lambda r: r["s1_0.055"].result["arguments"].update(slam_noise_seed=7),
            "slam noise seed",
        ),
        (
            lambda r: r["s1_0.055"].meta["mount"].update(depth_quantize_mm=None),
            "meta mount differs",
        ),
        (
            lambda r: r["s1_0.055"].result.update(pallet_urdf_sha256="0" * 64),
            "pallet_urdf_sha256 differs",
        ),
        (
            lambda r: r["s5_0.08"].meta.update(approach_straight_speed_mps=0.15),
            "not in the matrix",
        ),
    ],
)
def test_a_run_outside_the_plan_matrix_makes_the_result_diagnostic(spoil, message):
    runs = matrix()
    spoil(runs)
    runs.pop(runs.pop("drop", None) or "none", None)
    problems = problems_of(runs)
    assert any(message in p for p in problems), problems


def test_a_stop_with_a_command_left_on_or_no_restart_is_refused():
    runs = matrix()
    stamps, applied = commands(STOPS)
    leak = applied.copy()
    leak[np.argmin(np.abs(stamps - (STOPS[1]["trigger_s"] + 0.1)))] = 0.01
    cmds = {k: (stamps, leak if k == "s3_0.055" else applied) for k in runs}
    problems = DT.matrix_problems(runs, BOUNDS, references(), MANIFEST, cmds)
    assert any(
        "s3_0.055: the command was not zero through the 1.5 m stop" in p
        for p in problems
    )
    dead = applied.copy()
    dead[stamps > STOPS[3]["end_s"] + 1e-9] = 0.0
    cmds = {k: (stamps, dead if k == "s5_0.055" else applied) for k in runs}
    problems = DT.matrix_problems(runs, BOUNDS, references(), MANIFEST, cmds)
    assert any("s5_0.055: no restart after the 0.7 m stop" in p for p in problems)
    # Codex 3rd review: standing from the first stop to the last, moving only after it --
    # the last restart must not count for the first three.
    still = applied.copy()
    still[(stamps >= STOPS[0]["trigger_s"]) & (stamps <= STOPS[3]["end_s"] + 1e-9)] = (
        0.0
    )
    cmds = {k: (stamps, still if k == "s1_0.08" else applied) for k in runs}
    problems = DT.matrix_problems(runs, BOUNDS, references(), MANIFEST, cmds)
    for gap in (2.0, 1.5, 1.0):
        assert any(
            f"s1_0.08: no restart after the {gap} m stop" in p for p in problems
        ), gap
    assert not any("s1_0.08: no restart after the 0.7 m stop" in p for p in problems)


def test_the_recent_window_holds_only_the_commands_of_the_last_three_tenths():
    # Codex 2nd review: the command drops 0.055 -> 0 for the interval starting at 0.5 s;
    # the truck rolls at 0.01 m/s only in [0.791667, 0.8]. Every command of the intervals
    # ending in (0.5, 0.8] is zero, so delta_v is 0.01.
    n = 97  # rows at 0 .. 0.8 s
    t = np.arange(n) * TICK
    speed = np.zeros(n - 1)
    speed[95] = 0.01  # the interval [95, 96] = [0.791667, 0.8]
    x = np.concatenate(([0.0], np.cumsum(speed * TICK)))
    control = np.column_stack(
        (t, x, np.zeros(n), np.zeros(n), x, np.zeros(n), np.zeros(n))
    )
    stamps = t + TICK
    cmd = np.where(
        np.arange(n) < 60, 0.055, 0.0
    )  # interval 59 = [0.4917, 0.5] is the last at 0.055
    terms = DT.drive_terms(control, stamps, cmd, 0.0, t[-1])
    assert terms["delta_v_mps"] == pytest.approx(0.01)
