"""The obstacle layer's latency budget (docs/plans/2026-11-11-layer-latency-budget.md, the lead's decisions): the config
closed as #417 does (stopping latency + B, use age L = round(0.2 + B)), and the watchdog -- (i) fixed per-scan
deadlines a new scan never pushes, (ii) no motion on a snapshot first used with D > B (a late install right before the
permission included), the in-place path never caught, one judgement per tick, the stop over overdue_end_s in a row
reported as layer_latency_exceeded, nothing without a budget. No Isaac."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

from forklift_core.perception.obstacle_grid import AgeErrorTable
from forklift_core.planning.geometry import Bounds, Footprint

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
SPEC = importlib.util.spec_from_file_location("obstacle_layer", ROOT / "sim/isaac/obstacle_layer.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["obstacle_layer"] = MODULE
SPEC.loader.exec_module(MODULE)

CONFIG = MODULE.load_layer_config(ROOT / "config/obstacle_layer_single_team_spec.yaml")
AGE = json.loads((ROOT / CONFIG["odometry_age"]).read_text())
TABLE = AgeErrorTable(tuple(AGE["ages_s"]), tuple(AGE["cumulative_position_m"]), tuple(AGE["cumulative_yaw_rad"]))


def layer(budget=0.10, end=1.0):
    cfg = MODULE.budget_config({**CONFIG, "overdue_end_s": end}, budget)
    return MODULE.ObstacleLayer(
        cfg, hall=Bounds(-5, 10, -5, 5), error_table=TABLE,
        unloaded=Footprint(1.29, 0.17, 0.36), loaded=Footprint(1.53, 0.17, 0.40),
        body_front_m=0.884, rear_axle_x_in_base_m=-0.34, noise_seed=1,
    )


def scan(lay, t):
    lay._scan_stamps.append(t)


def install(lay, newest):
    lay._note_install({"scan_stamps": {} if newest is None else {"lidar": newest}})


def test_the_budget_closes_the_stopping_latency_and_the_use_age_and_reads_the_0_3_row():
    cfg = MODULE.budget_config(CONFIG, 0.10)
    assert cfg["permission"]["latency_s"] == pytest.approx(0.25) and cfg["grid"]["free_use_age_s"] == 0.3
    assert TABLE.at(cfg["grid"]["free_use_age_s"]) == TABLE.at(0.3)  # 0.2 + 0.1 would read the 0.5 row unrounded
    assert TABLE.at(0.2 + 0.1) != TABLE.at(0.3)
    assert cfg["latency_budget_applied_s"] == 0.10 and CONFIG["permission"]["latency_s"] == 0.15  # a copy
    off = MODULE.budget_config(CONFIG, 0.0)
    assert off["permission"] is CONFIG["permission"] and off["latency_budget_applied_s"] == 0.0
    assert MODULE.budget_config({**CONFIG, "latency_budget_s": 0.05})["latency_budget_applied_s"] == 0.05
    with pytest.raises(ValueError):
        MODULE.budget_config(CONFIG, -0.01)


def test_no_budget_no_watchdog():
    lay = layer(0.0)
    scan(lay, 0.0)
    assert lay.watchdog(5.0) is None and lay.latency_log["scans"] == []


def test_the_in_place_path_is_never_caught():
    lay = layer()
    for i in range(30):
        t = i * 0.1
        scan(lay, t)
        install(lay, t)
        for j in range(12):
            assert lay.watchdog(t + j / 120) is None
    assert max(d for _, _, d in lay.latency_log["scans"]) == pytest.approx(0.0)


def test_a_scan_past_its_deadline_stops_and_a_new_scan_does_not_push_it():
    lay = layer()
    scan(lay, 0.0)
    install(lay, 0.0)
    assert lay.watchdog(0.0) is None
    scan(lay, 0.1)  # not installed
    assert lay.watchdog(0.19) is None  # 0.1 + B not passed yet
    scan(lay, 0.2)  # a newer scan arrives: scan 0.1's deadline (0.2) stays
    assert lay.watchdog(0.2 + 2e-6) == "layer_overdue"
    assert lay.latency_log["overdue_ticks"][-1][4] == "deadline"
    install(lay, 0.2)  # first used 0.2 s... later than B: (ii) holds the stop
    assert lay.watchdog(0.3 + 3e-6) == "layer_overdue" and lay.latency_log["overdue_ticks"][-1][4] == "late"


def test_a_late_install_right_before_the_permission_does_not_drive():
    lay = layer()
    scan(lay, 0.0)
    install(lay, 0.0)
    lay.watchdog(0.0)
    scan(lay, 0.1)
    install(lay, 0.1)  # installed just now, first use 0.15 s after its scan: D > B
    assert lay.watchdog(0.25) == "layer_overdue"
    scan(lay, 0.2)
    install(lay, 0.2)  # a timely one: D 0.02
    assert lay.watchdog(0.22) is None


def test_a_barrier_install_ahead_of_its_deadline_drives():
    lay = layer()
    scan(lay, 0.0)
    install(lay, 0.0)
    assert lay.watchdog(0.05) is None  # D 0.05 <= B: an early (barrier) install is in time


def test_one_judgement_per_tick_and_the_stop_over_the_limit_is_named():
    lay = layer(end=1.0)
    scan(lay, 0.0)
    install(lay, 0.0)
    lay.watchdog(0.0)
    for i in range(1, 30):
        scan(lay, 0.1 * i)  # nothing installed again
    t0 = 0.1 + 0.1 + 2e-6
    assert lay.watchdog(t0) == "layer_overdue"
    assert lay.watchdog(t0) == "layer_overdue" and lay.overdue_stats["ticks"] == 1  # the same tick, counted once
    assert lay.watchdog(t0 + 1.0) == "layer_overdue"  # exactly 1.0 s held
    assert lay.watchdog(t0 + 1.0 + 1 / 120) == "layer_latency_exceeded"
    assert lay.latency_log["overdue_ticks"][-1][5:] == ["layer_latency_exceeded", pytest.approx(1.0 + 1 / 120)]
    assert [r[5] for r in lay.latency_log["overdue_ticks"]].count("layer_latency_exceeded") == 1
    assert lay.overdue_stats["events"] == 1 and lay.overdue_stats["longest_s"] > 1.0
    assert lay.overdue_stats["total_s"] == pytest.approx(lay.overdue_stats["longest_s"])


def test_a_stop_released_within_the_limit_goes_on():
    lay = layer(end=1.0)
    scan(lay, 0.0)
    install(lay, 0.0)
    lay.watchdog(0.0)
    scan(lay, 0.1)
    assert lay.watchdog(0.25) == "layer_overdue"
    scan(lay, 0.3)
    install(lay, 0.3)
    assert lay.watchdog(0.3 + 1 / 120) is None and lay._overdue_since is None


def test_limit_returns_the_watchdogs_stop():
    lay = layer()
    scan(lay, 0.0)
    scan(lay, 0.1)  # nothing installed at all
    assert lay.limit(0.25, current_pose=(0, 0, 0), curvature_inv_m=0.0, direction=1, loaded=False,
                     cap_mps=0.3) == (0.0, "layer_overdue")


def test_the_real_trucks_budget_equals_the_simulators_and_carries_its_provenance():
    import yaml

    real = yaml.safe_load((ROOT / "config/real/obstacle_layer.yaml").read_text())
    sim = MODULE.load_budget_file(ROOT / "config/layer_latency_budget.yaml")
    assert real["schema"] == "real_obstacle_layer/1"
    assert real["latency_budget"]["seconds"] == sim["latency_budget_s"] == 0.10
    assert real["overdue_end"]["seconds"] == sim["overdue_end_s"] == 1.0
    for entry in (real["latency_budget"], real["overdue_end"]):
        assert set(entry["provenance"]) == {"source", "measured", "until"} and entry["provenance"]["measured"] is False


def test_a_budgeted_run_still_matches_a_survey_bundle():
    """The budget lives outside the survey bundles' shared inputs and scene arguments: a run that passes it matches a
    bundle made without it (the registered bundles' manifests hold the unchanged obstacle-layer file's sha256)."""
    from forklift_core.localization import slam_map

    assert "layer_latency_budget" not in slam_map.SHARED_INPUTS and "layer_latency_budget" not in slam_map.SHARED_ARGUMENTS
    assert "layer_latency_budget_s" not in slam_map.SHARED_ARGUMENTS
    files = {"obstacle_layer": ROOT / "config/obstacle_layer_single_team_spec.yaml",
             "lidar": ROOT / "config/isaac_slam_lidar.yaml"}
    sha = {name: (slam_map.file_sha256(files[name]) if name in files else "x" * 64)
           for name in slam_map.SHARED_INPUTS + (slam_map.SCENE_INPUT,)}
    survey_args = {"layout": "scene", "prism_colliders": False, "min_obstacle_height_m": None,
                   "slam_noise_seed": 0, "slam_noise_spec": None}
    manifest = {"seed": 0, "run": {"arguments": survey_args, "scene_sha256": "s"},
                "inputs": {name: {"sha256": h} for name, h in sha.items()}}
    run_args = {**survey_args, "layer_latency_budget": "config/layer_latency_budget.yaml", "layer_latency_budget_s": None}
    assert slam_map.run_mismatches(manifest, seed=0, input_sha256=sha, scene_sha256="s", arguments=run_args) == []
    # the registered bundles' recorded obstacle-layer hash (ws1 manifests, read 2026-11-11) is the file's
    assert slam_map.file_sha256(files["obstacle_layer"]) == \
        "73e35f8fee45ad5547df851cb799528b5b25d89657bcca3697607d84af5d9db0"


def test_a_reinstallation_without_a_newer_scan_is_not_judged_again():
    lay = layer()
    scan(lay, 0.0)
    install(lay, 0.0)
    assert lay.watchdog(0.01) is None
    install(lay, 0.0)  # a correction's re-projection 0.09 s later: the same newest scan, not a new first use
    assert lay.watchdog(0.099) is None and len(lay.latency_log["scans"]) == 1
