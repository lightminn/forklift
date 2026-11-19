"""docs/design/2026-11-19-real-safety-wiring.md 1-2 (stage 5a): the real obstacle layer -- configs with B and the
scan timing error, scans placed or dropped, a refresh as a request, installs keyed on the whole request, the
watchdog's deadlines, the exemption, the planning grid's pallet split."""

import math
from types import SimpleNamespace

import numpy as np
import yaml

from forklift_core.live.real_layer import LayerKey, ManualWorker, RealLayer, RealScan, real_layer_configs
from forklift_core.perception.obstacle_grid import OCCUPIED, AgeErrorTable
from forklift_core.planning.geometry import Footprint

from test_teleop_bridge_cycle import MS  # noqa: F401 -- the shared test conventions

ROOT = __import__("pathlib").Path(__file__).resolve().parents[3]
LAYER = yaml.safe_load((ROOT / "config/real/obstacle_layer_real.yaml").read_text())
TABLE = AgeErrorTable((0.1, 0.2, 0.3, 1.0, 3.0), (0.024, 0.030, 0.039, 0.091, 0.141),
                      (0.014, 0.022, 0.026, 0.039, 0.059))
HALL = SimpleNamespace(x_min_m=-2.0, x_max_m=6.0, y_min_m=-3.0, y_max_m=3.0)
ANGLES = np.linspace(-math.pi, math.pi, 720, endpoint=False)
B, PERIOD = 0.10, 0.10
BODY = Footprint(0.30, 0.20, 0.25)


def configs(**kw):
    args = dict(error_table=TABLE, budget_s=B, scan_period_s=PERIOD, state_delay_s=0.03, v_scan_mps=0.225,
                kappa_max_inv_m=0.5, hall=HALL)
    args.update(kw)
    return real_layer_configs(LAYER, **args)


def layer(placer=None):
    grid, perm = configs()
    return RealLayer(grid, perm, sensors={"lidar": ((0.0, 0.0, 0.0), True)}, unloaded=BODY, loaded=BODY, body=BODY,
                     placer=placer or (lambda a, b: (0.0, 0.0, 0.0)), budget_s=B, overdue_end_s=1.0,
                     scan_period_s=PERIOD, scan_max_age_s=0.35, worker=ManualWorker())


def wall(first_s, x=3.0, valid=None):
    r = np.full(len(ANGLES), np.inf)
    front = np.abs(ANGLES) < math.radians(40)
    r[front] = x / np.cos(ANGLES[front])
    v = np.ones(len(ANGLES), bool) if valid is None else valid
    return RealScan("lidar", first_s, first_s + PERIOD, ANGLES, r, v, np.zeros(len(ANGLES), bool))


def ask(lay, t, version=0, path=None):
    lay.refresh(t, (0.0, 0.0, 0.0), version, current_pose=(0.0, 0.0, 0.0),
                path_ahead=np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]]) if path is None else path, loaded=False)


def test_the_configs_take_b_and_the_scan_timing_error():
    grid, perm = configs()
    assert math.isclose(grid.use_age_s, LAYER["grid"]["free_max_age_s"] + B)
    assert perm.evidence_max_age_s == grid.use_age_s
    assert math.isclose(perm.stopping.latency_s, LAYER["permission"]["latency_s"] + PERIOD + B)
    dt = 2 * PERIOD + 0.03
    assert math.isclose(grid.error.position_m[0], TABLE.position_m[0] + 0.225 * dt)
    assert math.isclose(grid.error.yaw_rad[0], TABLE.yaw_rad[0] + 0.225 * 0.5 * dt)


def test_refresh_is_a_request_and_a_result_installs_only_on_the_current_key():
    lay = layer()
    lay.add_scan(wall(1.0), 1.0)
    assert lay.refresh(1.1, (0.0, 0.0, 0.0), 0, current_pose=(0, 0, 0), path_ahead=None, loaded=False) is None
    assert lay.limit(1.15, current_pose=(0, 0, 0), curvature_inv_m=0.0, direction=1, loaded=False,
                     cap_mps=0.1)[1] == "snapshot_installing"
    lay.worker.step()
    lay.exempt = (2.9, 0.0, 0.2, 0.4, 0.0)  # the key moves on before the result is installed
    lay.install_ready(1.2)
    assert lay.counts["stale_results"] == 1 and lay.installing()
    ask(lay, 1.2)
    lay.worker.step()
    assert lay.install_ready(1.21) == 1 and not lay.installing()
    assert lay.installed_key() == LayerKey(0, 1, None, (), False)


def test_the_first_scan_asks_for_the_first_snapshot_itself():
    lay = layer()
    assert lay.current_key() is None
    lay.add_scan(wall(1.0), 1.0)
    lay.worker.step()
    assert lay.install_ready(1.15) == 1 and lay.snapshot is not None


def test_a_scan_not_placed_or_with_too_few_valid_beams_is_dropped():
    lay = layer(placer=lambda a, b: None)
    assert lay.add_scan(wall(1.0), 1.0) is None  # waiting for the odometry to reach past its last beam
    lay.install_ready(1.05)
    assert lay.counts["scans_dropped"] == 0
    lay.install_ready(1.2)  # waited over place_wait_s: dropped
    assert lay.counts["scans_dropped"] == 1 and not lay.grid.scans
    lay = layer()
    valid = np.zeros(len(ANGLES), bool)
    valid[:100] = True
    assert lay.add_scan(wall(1.0, valid=valid), 1.0) == "too_few_valid_beams"
    assert lay.counts["scans_dropped"] == 1 and not lay.grid.scans


def test_invalid_beams_neither_mark_nor_clear():
    lay = layer()
    valid = np.ones(len(ANGLES), bool)
    valid[np.abs(ANGLES) < math.radians(20)] = False  # the beams straight ahead read nothing (wider than closing)
    lay.add_scan(wall(1.0, valid=valid), 1.0)
    lay.worker.step()
    lay.install_ready(1.15)
    snap = lay.snapshot
    i = int((3.0 - snap.origin_x_m) / snap.resolution_m)
    j = int((0.0 - snap.origin_y_m) / snap.resolution_m)
    assert snap.state[i, j] != OCCUPIED


def test_a_dropped_scan_keeps_its_deadline_until_a_newer_result_is_installed():
    calls = {"n": 0}

    def placer(a, b):
        calls["n"] += 1
        return None if calls["n"] == 2 else (0.0, 0.0, 0.0)

    lay = layer(placer)
    lay.add_scan(wall(1.0), 1.0)
    lay.worker.step()
    lay.install_ready(1.12)
    assert lay.watchdog(1.12) is None
    lay.add_scan(wall(1.1), 1.2)  # dropped: its deadline 1.2 + 0.1
    assert lay.watchdog(1.31) == "layer_overdue"
    lay.add_scan(wall(1.2), 1.32)
    lay.worker.step()
    lay.install_ready(1.33)
    assert lay.watchdog(1.34) is None


def test_a_scan_that_never_arrives_holds_the_wait_until_a_later_result_is_installed():
    lay = layer()
    lay.add_scan(wall(1.0), 1.0)
    lay.worker.step()
    lay.install_ready(1.12)
    assert lay.watchdog(1.25) is None
    assert lay.watchdog(1.31) == "layer_overdue"  # expected by 1.1 + 0.1 + 0.1
    lay.add_scan(wall(1.2), 1.33)  # a late receipt alone does not release it
    assert lay.watchdog(1.34) == "layer_overdue"
    lay.worker.step()
    lay.install_ready(1.35)
    assert lay.watchdog(1.36) is None


def test_a_result_first_used_late_waits_and_the_next_timely_one_releases():
    lay = layer()
    lay.add_scan(wall(1.0), 1.0)
    lay.worker.step()
    lay.install_ready(1.25)
    assert lay.watchdog(1.25) == "layer_overdue"  # first used 0.25 s after its first beam (> period + B)
    lay.add_scan(wall(1.2), 1.3)
    lay.worker.step()
    lay.install_ready(1.31)
    assert lay.watchdog(1.32) is None


def test_an_overdue_held_too_long_is_the_end():
    lay = layer()
    lay.add_scan(wall(1.0), 1.0)
    lay.worker.step()
    lay.install_ready(1.12)
    assert lay.watchdog(1.31) == "layer_overdue"
    assert lay.watchdog(2.4) == "layer_latency_exceeded"


def test_a_dead_worker_is_the_end_at_once():
    lay = layer()
    lay.worker.alive = False
    assert lay.watchdog(1.0) == "layer_worker_dead"


def test_the_exemption_frees_its_region_and_a_narrowing_waits():
    lay = layer()
    lay.exempt = (3.0, 0.0, 0.4, 0.6, 0.0)
    lay.add_scan(wall(1.0), 1.0)
    lay.worker.step()
    lay.install_ready(1.12)
    snap = lay.snapshot
    i = int((3.0 - snap.origin_x_m) / snap.resolution_m)
    j = int((0.0 - snap.origin_y_m) / snap.resolution_m)
    assert snap.state[i, j] != OCCUPIED
    lay.exempt = None  # the D5 region goes: the installed result no longer answers
    assert lay.installing() and lay.ok(1.13)  # a wait, not a fault


def test_the_planning_grid_drops_the_frozen_pallet_and_the_driving_snapshot_keeps_it():
    lay = layer()
    # a face wide enough that the grid's gap closing (0.25 m) does not fill it from the wall either side
    rect = (3.0, 0.0, 0.0, 0.3, 1.5)
    lay.planning_pallet = (rect, ("P1", 0))
    lay.add_scan(wall(1.0), 1.0)
    lay.worker.step()
    lay.install_ready(1.12)
    occ = lay.planning_grid(("P1", 0))
    assert occ is not None and lay.planning_grid(("P1", 1)) is None
    snap = lay.snapshot
    i = int((3.0 - snap.origin_x_m) / snap.resolution_m)
    j = int((0.0 - snap.origin_y_m) / snap.resolution_m)
    assert snap.state[i, j] == OCCUPIED  # the driving grid keeps the pallet
    i2, j2 = occ.cell_of(3.0, 0.0)
    assert not occ.occupied[i2, j2]


def test_the_install_counts_the_distance_driven_since_the_request():
    lay = layer()
    lay.odom_now = lambda: (0.0, 0.0, 0.0)
    lay.add_scan(wall(1.0), 1.0)
    ask(lay, 1.05)
    lay.worker.step()
    lay.install_ready(1.12, odom_pose=(0.03, 0.04, 0.0))
    assert math.isclose(lay.permission._driven_since_m, 0.05)  # noqa: SLF001


def test_a_scan_waits_for_the_odometry_past_its_last_beam_then_is_placed():
    """Design 1.1: the history must reach past the last beam -- the placement happens at a later tick head."""
    reached = {"t": 0.0}
    lay = layer(placer=lambda a, b: (0.0, 0.0, 0.0) if reached["t"] >= b else None)
    lay.add_scan(wall(1.0), 1.1)
    assert not lay.grid.scans
    reached["t"] = 1.12
    lay.install_ready(1.12)
    assert len(lay.grid.scans) == 1 and lay.counts["scans_dropped"] == 0
