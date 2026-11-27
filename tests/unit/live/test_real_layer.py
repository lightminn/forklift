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
    return RealLayer(grid, perm, sensors={"lidar": ((0.0, 0.0, 0.0), True, {"measured": False})}, unloaded=BODY,
                     loaded=BODY, body=BODY,
                     placer=placer or (lambda a, b: (0.0, 0.0, 0.0)), budget_s=B, overdue_end_s=1.0,
                     scan_period_s=PERIOD, scan_max_age_s=0.35, worker=ManualWorker())


def wall(first_s, x=3.0, valid=None):
    r = np.full(len(ANGLES), np.inf)
    front = np.abs(ANGLES) < math.radians(40)
    r[front] = x / np.cos(ANGLES[front])
    v = np.ones(len(ANGLES), bool) if valid is None else valid
    return RealScan("lidar", first_s, first_s + PERIOD, ANGLES, r, v, np.zeros(len(ANGLES), bool),
                    np.full(len(ANGLES), np.inf))


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


def test_the_install_counts_the_path_driven_since_the_request():
    """Codex 5a impl P2-3: the path length (a there-and-back counts both ways), not the straight line."""
    lay = layer()
    lay.odom_now = lambda: 1.00
    lay.add_scan(wall(1.0), 1.0)
    ask(lay, 1.05)
    lay.worker.step()
    lay.install_ready(1.12, odom_travel=1.08)
    assert math.isclose(lay.permission._driven_since_m, 0.08)  # noqa: SLF001
    lay.advance(0.01)
    assert math.isclose(lay.permission._driven_since_m, 0.09)  # noqa: SLF001


def test_the_path_check_runs_in_the_worker():
    """Codex 5a impl P2-5: the control thread installs the worker's check; it does not walk the path itself."""
    lay = layer()
    calls = {"n": 0}
    lay.permission.check_path = lambda *a, **k: calls.__setitem__("n", calls["n"] + 1)
    lay.add_scan(wall(1.0), 1.0)
    lay.worker.step()
    lay.install_ready(1.12)
    assert calls["n"] == 0 and lay.permission.path_check is not None


def test_the_original_grid_keeps_only_what_a_snapshot_may_use():
    lay = layer()
    for k in range(30):
        lay.add_scan(wall(1.0 + 0.1 * k), 1.1 + 0.1 * k)
    assert len(lay.grid.scans) <= 5  # occupied_max_age_s 0.35 at 10 Hz


def test_a_late_receipt_does_not_erase_the_missed_expectation():
    """Codex 5a impl P1-2: the expectation passed before this receipt is held until its result is installed."""
    lay = layer()
    lay.add_scan(wall(1.0), 1.0)
    lay.worker.step()
    lay.install_ready(1.12)
    assert lay.watchdog(1.29) is None
    lay.add_scan(wall(1.2), 1.31)  # expected by 1.1 + 0.1 + 0.1 = 1.30; no watchdog call in between
    assert lay.watchdog(1.311) == "layer_overdue"
    lay.worker.step()
    lay.install_ready(1.32)
    assert lay.watchdog(1.33) is None


def test_first_use_is_late_from_the_last_beam():
    """Design 9 / Codex 5a impl P1-2: a short revolution (last beam 1.05) first used 0.12 s after it is late (B 0.10),
    though it is within a nominal period + B of the first beam."""
    lay = layer()
    scan = wall(1.0)
    scan.last_s = 1.05
    lay.add_scan(scan, 1.05)
    lay.worker.step()
    lay.install_ready(1.17)
    assert lay.watchdog(1.17) == "layer_overdue"


def test_nan_and_zero_ranges_are_not_valid_beams_whatever_the_flag():
    """Codex 5a impl P2-6: a NaN never counts toward the valid fraction; a zero range never marks."""
    lay = layer()
    scan = wall(1.0)
    scan.ranges_m = np.full(len(ANGLES), np.nan)
    assert lay.add_scan(scan, 1.0) == "too_few_valid_beams"
    lay = layer()
    scan = wall(1.0)
    scan.ranges_m = scan.ranges_m.copy()
    scan.ranges_m[np.abs(ANGLES) < math.radians(5)] = 0.0
    lay.add_scan(scan, 1.0)
    lay.worker.step()
    lay.install_ready(1.12)
    snap = lay.snapshot
    i = int((0.0 - snap.origin_x_m) / snap.resolution_m)
    j = int((0.0 - snap.origin_y_m) / snap.resolution_m)
    assert snap.state[i, j] != OCCUPIED


def test_a_clearing_sensor_needs_its_beam_limit_and_a_sensor_needs_its_provenance():
    """Design 1.1 / Codex 5a impl P2-6."""
    lay = layer()
    scan = wall(1.0)
    scan.limit_m = None
    assert lay.add_scan(scan, 1.0) == "no_beam_limit"
    grid, perm = configs()
    for bad in ({}, {"lidar": ((0.0, 0.0, 0.0), True)}, {"lidar": ((0.0, 0.0, 0.0), True, {})},
                {"lidar": ((0.0, 0.0, float("nan")), True, {"measured": False})}):
        try:
            RealLayer(grid, perm, sensors=bad, unloaded=BODY, loaded=BODY, body=BODY, placer=lambda a, b: None,
                      budget_s=B, overdue_end_s=1.0, scan_period_s=PERIOD, scan_max_age_s=0.35, worker=ManualWorker())
        except ValueError as exc:
            assert str(exc).startswith("sensor_")
        else:
            raise AssertionError(bad)


def test_a_scan_waiting_past_its_limit_is_dropped_even_if_now_placeable():
    """Codex 5a impl P3: the wait's expiry is read before the placement."""
    reached = {"ok": False}
    lay = layer(placer=lambda a, b: (0.0, 0.0, 0.0) if reached["ok"] else None)
    lay.add_scan(wall(1.0), 1.1)
    reached["ok"] = True
    lay.install_ready(1.25)
    assert lay.counts["scans_dropped"] == 1 and not lay.grid.scans


def test_placements_wait_for_flush_when_deferred():
    """Codex 5a impl P2-1: in the loop a placement only marks; the computation goes out after this tick's track()."""
    lay = layer()
    lay.defer_submit = True
    lay.add_scan(wall(1.0), 1.0)
    assert lay.worker.pending is None
    lay.track((0.2, 0.0, 0.0), np.array([[0.2, 0.0, 0.0], [0.7, 0.0, 0.0]]))
    lay.flush(1.01)
    assert lay.worker.pending is not None and lay.worker.pending[0]["current_pose"] == (0.2, 0.0, 0.0)


def test_a_scan_waits_for_the_odometry_past_its_last_beam_then_is_placed():
    """Design 1.1: the history must reach past the last beam -- the placement happens at a later tick head."""
    reached = {"t": 0.0}
    lay = layer(placer=lambda a, b: (0.0, 0.0, 0.0) if reached["t"] >= b else None)
    lay.add_scan(wall(1.0), 1.1)
    assert not lay.grid.scans
    reached["t"] = 1.12
    lay.install_ready(1.12)
    assert len(lay.grid.scans) == 1 and lay.counts["scans_dropped"] == 0


def test_a_too_close_return_marks():
    """Codex 5a impl round 2 P1-1: REP-117 -inf (too close) is a valid reading the shared grid marks."""
    lay = layer()
    scan = wall(1.0)
    scan.ranges_m = scan.ranges_m.copy()
    scan.ranges_m[np.argmin(np.abs(ANGLES))] = -np.inf
    lay.add_scan(scan, 1.0)
    lay.worker.step()
    lay.install_ready(1.12)
    ref = layer()
    ref.add_scan(wall(1.0), 1.0)
    ref.worker.step()
    ref.install_ready(1.12)
    assert np.count_nonzero(lay.snapshot.state == OCCUPIED) > np.count_nonzero(ref.snapshot.state == OCCUPIED)


def test_an_end_reached_at_the_tick_a_timely_result_arrives_is_still_the_end():
    """Codex 5a impl round 2 P1-2: the deadlines are read before the install; the end is sticky until taken."""
    lay = layer()
    lay.add_scan(wall(1.0), 1.0)
    lay.worker.step()
    lay.install_ready(1.12)
    assert lay.watchdog(1.31) == "layer_overdue"  # the next scan expected by 1.30
    t = 1.31
    while t < 2.31:  # scans keep coming, the results are held back
        t = round(t + 0.1, 6)
        lay.add_scan(wall(t - 0.1), t)
        lay.watchdog(t)
    lay.worker.step()
    lay.install_ready(2.32)  # past the 1.0 s end at this very tick
    assert lay.watchdog(2.32) == "layer_latency_exceeded"
    assert lay.watchdog(2.40) == "layer_latency_exceeded"
    lay.end_taken()
    lay.add_scan(wall(2.31), 2.41)
    lay.worker.step()
    lay.install_ready(2.42)
    assert lay.watchdog(2.42) is None


def test_a_re_projection_of_a_scan_first_used_in_time_is_not_late():
    """Codex 5a impl round 2 P2-2: first use is judged once per scan."""
    lay = layer()
    lay.add_scan(wall(1.0), 1.1)
    lay.worker.step()
    lay.install_ready(1.12)
    ask(lay, 1.24, version=1)  # a correction change re-projects the same scan
    lay.worker.step()
    assert lay.install_ready(1.24) == 1
    assert lay.watchdog(1.24) is None


def test_a_late_scan_within_the_end_keeps_the_layer_ok():
    """Codex 5a impl round 2 P2-1: readiness reads a late source as the watchdog's wait, not a missing layer."""
    lay = layer()
    lay.add_scan(wall(1.0), 1.1)
    lay.worker.step()
    lay.install_ready(1.12)
    assert lay.watchdog(1.46) == "layer_overdue" and lay.ok(1.46)
    assert lay.watchdog(2.5) == "layer_latency_exceeded" and not lay.ok(2.5)


def test_the_configs_take_the_configured_range_model():
    """Codex 5a impl round 5 P2-2: the real YAML's a2_spec range bound, not the constant."""
    grid, _ = configs()
    assert LAYER["range_noise"]["model"] == grid.range_bound_model == "a2_spec"
    assert float(grid.sensor_bound_at(np.array([5.0]))[0]) > float(grid.sensor_bound_at(np.array([1.0]))[0])


def test_an_older_waiting_scan_expires_after_a_newer_one_was_installed():
    """Codex 5a impl round 5 P2-1: its deadline already held by the newer result -- dropped without an error."""
    lay = layer(placer=lambda a, b: None if a < 1.05 else (0.0, 0.0, 0.0))
    lay.add_scan(wall(1.0), 1.1)  # waits (not placeable)
    lay.add_scan(wall(1.1), 1.2)  # placed
    lay.worker.step()
    lay.install_ready(1.21)
    lay.install_ready(1.25)  # the first one's wait ran out
    assert lay.counts["scans_dropped"] == 1 and lay.watchdog(1.25) is None


def test_the_timing_bypass_skips_the_deadlines_and_the_scan_age_but_not_the_worker_or_the_source():
    """Real safety wiring design 5 (5c-1): layer_timing bypassed."""
    lay = layer()
    lay.add_scan(wall(1.0), 1.1)
    lay.worker.step()
    lay.install_ready(1.12)
    assert lay.watchdog(1.5) == "layer_overdue"
    lay.timing_bypassed = True
    assert lay.watchdog(1.5) is None and lay.ok(5.0)
    assert lay.limit(5.0, current_pose=(0, 0, 0), curvature_inv_m=0.0, direction=1, loaded=False,
                     cap_mps=0.1)[1] != "scan_stale"
    lay.source_lost("lidar")
    assert lay.watchdog(5.1) == "scan_source_lost" and not lay.ok(5.1)
    lay = layer()
    lay.timing_bypassed = True
    lay.worker.alive = False
    assert lay.watchdog(1.0) == "layer_worker_dead"


def test_installing_may_ignore_only_the_correction_generation():
    """5c-1 r2 P2-3: the snapshot bypass waits for no correction-only install, but still for another key's."""
    lay = layer()
    lay.add_scan(wall(1.0), 1.1)
    lay.worker.step()
    lay.install_ready(1.12)
    ask(lay, 1.13, version=1)  # a new correction generation with the same values, not installed
    assert lay.installing() and not lay.installing(ignore_correction=True)
    lay.exempt = (2.0, 0.0, 0.4, 0.6, 0.0)  # another exemption too
    assert lay.installing(ignore_correction=True)
    lay2 = layer()
    lay2.add_scan(wall(1.0), 1.1)
    lay2.worker.step()
    lay2.install_ready(1.12)
    lay2.refresh(1.13, (0.0, 0.25, 0.0), 1, current_pose=(0, 0, 0), path_ahead=None, loaded=False)
    assert lay2.installing(ignore_correction=True)  # 5c-1 r3 P1: other correction values -- still waited for


def test_the_path_end_slack_reaches_the_permission_after_the_layers_own_checks():
    """U3e-1b (docs/plans/2026-11-27-real-path-end-hold.md 3): ``path_end_slack_m`` goes to ``allowed_speed`` only when
    given, and only after the layer's own refusals -- an old scan at the evidence instant is still ``scan_stale``."""
    lay = layer()
    seen = []
    lay.permission.allowed_speed = lambda *a, **k: seen.append(k) or (0.3, "ok")
    lay.installing = lambda ignore_correction=None: False
    lay._installed = {"newest_scan_s": 1.0}
    lay.watchdog = lambda now_s: None
    kw = dict(current_pose=(0.0, 0.0, 0.0), curvature_inv_m=0.0, direction=1, loaded=False, cap_mps=0.05)
    assert lay.limit(1.0, evidence_s=1.035, **kw) == (0.3, "ok") and "path_end_slack_m" not in seen[-1]
    assert lay.limit(1.0, evidence_s=1.035, path_end_slack_m=0.008, **kw) == (0.3, "ok")
    assert seen[-1]["path_end_slack_m"] == 0.008 and "path_end_ahead_m" not in seen[-1]
    lay.limit(1.0, evidence_s=1.035, path_end_slack_m=0.008, path_end_ahead_m=0.005, **kw)
    assert seen[-1]["path_end_ahead_m"] == 0.005
    n = len(seen)
    assert lay.limit(1.0, evidence_s=1.0 + lay.scan_max_age_s + 0.01, path_end_slack_m=0.008, **kw)[1] == "scan_stale"
    assert len(seen) == n
