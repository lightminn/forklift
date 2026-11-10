"""Real-time plan 5.28, the 1e-0 fix (docs/validation/2026-11-10-g3-1e0-run.md): with a layer worker the FREE evidence
may be used up to L = free_max_age_s + (k-1)/120 -- the grid's FREE shrink is sized for L, the permission's age checks
take L, the shadow memory's and the known pallets' horizons take L; what accepts evidence (fresh-scan selection, the
planning memory's clears, the exit-prefix verification) keeps free_max_age_s, and the exit-prefix cells it frees are
stamped (k-1)/120 earlier so they expire when they do without a worker. Off: nothing changes. No Isaac."""

import copy
import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.control.drive_permission import DrivePermission
from forklift_core.control.exit_prefix_exempt import VerifiedCorridor, apply_verified
from forklift_core.perception.obstacle_grid import (FREE, OCCUPIED, AgeErrorTable, GridConfig, GridSnapshot, ObstacleGrid,
                                                    ObstacleScan)
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
F = float(CONFIG["grid"]["free_max_age_s"])


def layer(config):
    return MODULE.ObstacleLayer(
        config, hall=Bounds(-5, 10, -5, 5), error_table=TABLE,
        unloaded=Footprint(1.29, 0.17, 0.36), loaded=Footprint(1.53, 0.17, 0.40),
        body_front_m=0.884, rear_axle_x_in_base_m=-0.34, noise_seed=1,
    )


@pytest.mark.parametrize("k", [2, 3, 4])
def test_the_worker_config_adds_the_delay_to_the_stopping_latency_and_the_use_age(k):
    before = copy.deepcopy(CONFIG)
    out = MODULE.worker_config(CONFIG, k)
    s = (k - 1) / 120.0
    assert CONFIG == before  # the file's config is not touched (the truth evaluation keeps it)
    assert out["permission"]["latency_s"] == float(CONFIG["permission"]["latency_s"]) + s
    assert out["grid"]["free_use_age_s"] == F + s
    assert out["grid"]["free_max_age_s"] == F  # acceptance unchanged
    assert {key: v for key, v in out["grid"].items() if key != "free_use_age_s"} == CONFIG["grid"]


def test_off_every_use_reads_free_max_age_s():
    lay = layer(CONFIG)
    assert lay.grid_config.use_age_s == F and lay.grid_config.free_use_age_s is None
    assert lay.permission.config.evidence_max_age_s == F
    assert lay.shadow is None or lay.shadow.evidence_max_age_s == F
    assert lay.exit_stamp_back_s == 0.0


def test_with_a_worker_every_use_reads_l_and_acceptance_keeps_f():
    k = 3
    lay = layer(MODULE.worker_config(CONFIG, k))
    L = F + (k - 1) / 120.0
    assert lay.grid_config.use_age_s == L and lay.grid_config.free_max_age_s == F
    assert lay.permission.config.evidence_max_age_s == L
    assert lay.shadow is None or lay.shadow.evidence_max_age_s == L
    assert lay.exit_stamp_back_s == pytest.approx((k - 1) / 120.0, abs=1e-15)
    # the sensor silence and every other permission value are the file's
    off = layer(CONFIG).permission.config
    on = lay.permission.config
    assert on.sensor_timeout_s == off.sensor_timeout_s
    assert on.stopping.latency_s == pytest.approx(off.stopping.latency_s + (k - 1) / 120.0, abs=1e-15)


def test_the_known_pallets_take_the_permission_age_as_their_horizon(monkeypatch):
    """apply_known's horizon is the permission config's evidence_max_age_s (obstacle_layer refresh_compute): L with a
    worker. Read off the source so a refactor that takes the horizon from elsewhere fails here."""
    src = (ROOT / "sim/isaac/obstacle_layer.py").read_text()
    assert "horizon_s=float(pconf.evidence_max_age_s)" in src


def _scan(stamp):
    angles = np.linspace(-1.0, 1.0, 181)
    zero = (0.0, 0.0, 0.0)
    return ObstacleScan(stamp_s=stamp, sensor="high", odom_rear=zero, laser_in_rear=zero, angles_rad=angles,
                        ranges_m=np.full(181, 3.0), self_hit=np.zeros(181, bool))


def _grid(use_age=None):
    cfg = GridConfig(-5, 5, -5, 5, TABLE, free_max_age_s=F, free_use_age_s=use_age)
    return ObstacleGrid(cfg)


@pytest.mark.parametrize("kernel", [False, True])
def test_the_free_shrink_is_sized_for_the_use_age(monkeypatch, kernel):
    from forklift_core.perception import obstacle_grid

    monkeypatch.setattr(obstacle_grid, "CLEAR_KERNEL", kernel)
    for use_age, want in ((None, F), (F + 2 / 120.0, F + 2 / 120.0)):
        grid = _grid(use_age)
        ages = []
        real = grid.radius_m

        def spy(age_s, rho_m, *, sensor=True, _real=real):
            if not sensor and rho_m > 0.0:  # the FREE shrink's band edges (the marks' pose error is at rho 0)
                ages.append(age_s)
            return _real(age_s, rho_m, sensor=sensor)

        monkeypatch.setattr(grid, "radius_m", spy)
        grid.add_scan(_scan(0.0))
        grid.snapshot(0.05, (0.0, 0.0, 0.0))
        assert ages and set(ages) == {want}
    # the 0.3 s row of the table for L = 0.2167 s, the 0.2 s row for F
    assert TABLE.at(F + 2 / 120.0) == TABLE.at(0.3) != TABLE.at(F)


def test_a_scan_older_than_f_gives_no_free_evidence_even_with_the_use_age():
    grid = _grid(F + 2 / 120.0)
    grid.add_scan(_scan(0.0))
    snap = grid.snapshot(F + 1 / 120.0, (0.0, 0.0, 0.0))  # older than F, within L
    assert not (snap.state == FREE).any()


def _snapshot(stamp):
    state = np.full((40, 40), FREE, dtype=np.uint8)
    return GridSnapshot(state=state, free_stamp=np.full((40, 40), stamp), stamp_s=stamp, correction=(0.0, 0.0, 0.0),
                        correction_version=0, scans_used=1, origin_x_m=-1.0, origin_y_m=-1.0, resolution_m=0.05)


def test_the_permission_age_check_passes_between_f_and_l_only_with_the_worker():
    L = F + 2 / 120.0
    for cfg, expect_stale in ((layer(CONFIG).permission.config, True),
                              (layer(MODULE.worker_config(CONFIG, 3)).permission.config, False)):
        perm = DrivePermission(cfg)
        snap = _snapshot(0.0)
        path = np.column_stack((np.linspace(0, 0.5, 26), np.zeros(26), np.zeros(26)))
        own = Footprint(0.3, 0.1, 0.2)
        perm.update(snap, path, own, own, current_pose=(0.0, 0.0, 0.0), direction=1)
        now = 0.5 * (F + L)  # between the two limits
        speed, why = perm.allowed_speed(now, {}, current_pose=(0.0, 0.0, 0.0), curvature_inv_m=0.0, direction=1,
                                        footprint=own, own_footprint=own, speed_cap_mps=0.3)
        assert (why == "evidence_stale") is expect_stale


def test_the_age_comparisons_are_the_registered_ones():
    """The sites that compare a FREE evidence age with evidence_max_age_s, all reading a permission config (so all take
    L with a worker): drive_permission.allowed_speed, the teleop bundle, the candidate's zone, the cusp's next leg. A new
    one must be added here and judged against the plan."""
    pattern = re.compile(r"now_s?_? - .*oldest_free_s.*>.*evidence_max_age_s|age_ <= pconf_\.evidence_max_age_s")
    found = []
    for path in sorted((ROOT / "src").rglob("*.py")) + sorted((ROOT / "sim" / "isaac").glob("*.py")):
        for n, line in enumerate(path.read_text().splitlines(), 1):
            if pattern.search(line):
                found.append(str(path.relative_to(ROOT)))
    assert sorted(found) == sorted([
        "src/forklift_core/control/drive_permission.py",
        "src/forklift_core/safety/teleop_drive.py",
        "sim/isaac/obstacle_layer.py",
        "src/forklift_core/mission/core.py",
    ])


def _corridor_snapshot(stamp):
    state = np.full((60, 60), OCCUPIED, dtype=np.uint8)
    state[:, :5] = FREE
    stamps = np.full((60, 60), np.nan)
    stamps[:, :5] = stamp - 0.05
    return GridSnapshot(state=state, free_stamp=stamps, stamp_s=stamp, correction=(0.0, 0.0, 0.0),
                        correction_version=0, scans_used=1, origin_x_m=-1.5, origin_y_m=-1.5, resolution_m=0.05)


def test_the_exit_prefix_cells_are_stamped_earlier_by_the_delay_and_expire_as_without_a_worker():
    s = 2 / 120.0
    tau = 1.0
    verified = VerifiedCorridor(stamp_s=tau, correction_version=0, poses=np.array([[0.0, 0.0, 0.0]]))
    fp = Footprint(0.3, 0.1, 0.2)
    snap = _corridor_snapshot(tau + 0.01)
    off, n_off, why_off = apply_verified(snap, verified, fp, 0.06, now_s=tau + 0.01, max_age_s=F)
    on, n_on, why_on = apply_verified(snap, verified, fp, 0.06, now_s=tau + 0.01, max_age_s=F, stamp_back_s=s)
    assert (why_off, why_on) == (None, None) and n_off == n_on > 0
    freed = (snap.state == OCCUPIED) & (off.state == FREE)
    assert np.array_equal(off.state, on.state)
    assert np.all(off.free_stamp[freed] == tau) and np.allclose(on.free_stamp[freed], tau - s, rtol=0, atol=1e-15)
    untouched = ~freed
    assert np.array_equal(off.free_stamp[untouched], on.free_stamp[untouched], equal_nan=True)
    assert np.array_equal(on.free_stamp[untouched], snap.free_stamp[untouched], equal_nan=True)
    assert verified.stamp_s == tau  # the real verification time stays
    # expiry: off with limit F at tau + F; on with limit F + s at the same tau + F
    stamp_on = float(on.free_stamp[freed][0])
    for t, ok in ((tau + F - 1e-6, True), (tau + F + 1e-6, False)):
        assert (t - tau <= F) is ok
        assert (t - stamp_on <= F + s + 1e-12) is ok
    # acceptance keeps F: a verification older than F is stale either way
    stale = apply_verified(snap, verified, fp, 0.06, now_s=tau + F + 0.01, max_age_s=F, stamp_back_s=s)
    assert stale[2] == "stale"


def test_the_runner_builds_the_worker_config_through_the_one_helper():
    src = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert "obstacle_module.worker_config(layer_config, int(args.layer_worker_k))" in src
    assert 'layer_config["permission"]["latency_s"] = ' not in src  # the D5 latency is the helper's alone
