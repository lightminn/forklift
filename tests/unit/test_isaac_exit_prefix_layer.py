"""Exit-prefix exemption in the obstacle layer (docs/plans/2026-10-28-exit-prefix-extract-rules.md D1, D2; plan C1
"layer computation"): a real ObstacleLayer with the single-LiDAR team-spec config -- scans in through add_scans with
the ranges the grid gets, the watch placed with the scan's own correction, the one transform in refresh."""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.control import exit_prefix_exempt as xe
from forklift_core.perception.obstacle_grid import OCCUPIED, AgeErrorTable
from forklift_core.planning.geometry import Bounds, Footprint

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("obstacle_layer", ROOT / "sim/isaac/obstacle_layer.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["obstacle_layer"] = MODULE
SPEC.loader.exec_module(MODULE)

CONFIG = MODULE.load_layer_config(ROOT / "config/obstacle_layer_single_team_spec.yaml")
AGE = json.loads((ROOT / "config/obstacle_odometry_age_team_spec.json").read_text())
TABLE = AgeErrorTable(tuple(AGE["ages_s"]), tuple(AGE["cumulative_position_m"]), tuple(AGE["cumulative_yaw_rad"]))
LOADED = Footprint(1.53, 0.17, 0.40)
P0 = (0.0, 0.0, 0.0)
PATH = np.column_stack((-np.linspace(0, 1.0, 51), np.zeros(51), np.zeros(51)))  # a straight reverse
WALL_Y = 0.58  # side walls 0.18 m beside the loaded outline, as the rack posts and the trailer walls


def layer():
    return MODULE.ObstacleLayer(CONFIG, hall=Bounds(-5, 10, -5, 5), error_table=TABLE,
                                unloaded=Footprint(1.29, 0.17, 0.36), loaded=LOADED, body_front_m=0.884,
                                rear_axle_x_in_base_m=-0.34, noise_seed=1)


def scan(lay, *, y_wall=WALL_Y, box=None, given_shift=None, own_box=False):
    """Side walls at +-y_wall over x -1..1.6 m and optionally a box (x0, x1, y0, y1) in the rear-axle frame; the
    true distances, and (given_shift) shared ranges drawn as the true ones plus a shift."""
    raw = {}
    for s in CONFIG["sensors"]:
        ox, oy = s.xyz_m[0] + 0.34, s.xyz_m[1]
        ang = lay.beam_angles + s.yaw_rad
        c, sn = np.cos(ang), np.sin(ang)
        d = np.full(len(ang), np.inf)
        own = np.zeros(len(ang), bool)
        with np.errstate(divide="ignore", invalid="ignore"):
            for yw in (y_wall, -y_wall):
                dd = (yw - oy) / sn
                xx = ox + dd * c
                ok = np.isfinite(dd) & (dd > 0) & (xx > -1.0) & (xx < 1.6)
                d = np.where(ok & (dd < d), dd, d)
            if box is not None:
                x0, x1, y0, y1 = box
                for edge in (x0, x1):
                    dd = (edge - ox) / c
                    yy = oy + dd * sn
                    ok = np.isfinite(dd) & (dd > 0) & (yy >= y0) & (yy <= y1) & (dd < d)
                    own = np.where(ok, own_box, own)
                    d = np.where(ok, dd, d)
        hit = np.isfinite(d)
        dist = np.where(hit, d, np.nan)
        if given_shift is None:
            raw[s.name] = (dist, hit, own)
        else:
            given = np.where(hit, dist + given_shift(ang, dist), np.inf)
            raw[s.name] = (dist, hit, own, np.full(len(ang), np.inf), given)
    return raw


def set_region(lay, length=0.4):
    lay.exit_exempt_set(PATH[: int(round(length / 0.02)) + 1], measured_kappa=0.0, commanded_kappa=0.0, direction=-1)


def run(lay, *, exempt=True, correction=(0.0, 0.0, 0.0), version=0, refresh_at=9.95, refresh_version=0, **kw):
    if exempt:
        set_region(lay)
    for k in range(3):
        lay.add_scans(9.7 + 0.1 * k, scan(lay, **kw), odom_rear=P0, loaded=True, correction=correction,
                      version=version)
    lay.refresh(refresh_at, (0.0, 0.0, 0.0), refresh_version, current_pose=P0, path_ahead=PATH, loaded=True,
                direction=-1)
    return lay.limit(refresh_at, current_pose=P0, curvature_inv_m=0.0, direction=-1, loaded=True, cap_mps=0.18)


def test_the_halo_beside_the_walls_holds_the_reverse_start_and_the_verified_corridor_releases_it():
    assert run(layer(), exempt=False) == (0.0, "occupied")
    lay = layer()
    speed, why = run(lay)
    st = lay.exit_exempt_status()
    assert why == "ok" and speed > 0.2
    assert st["verified_version"] == 0 and st["verified_s"] == pytest.approx(9.9)
    assert st["transformed"] and st["skipped"] is None and st["freed"] > 0
    assert st.get("rang") is None


def test_only_occupied_cells_inside_the_corridor_are_freed():
    lay = layer()
    run(lay)
    snap = lay.snapshot
    occ = np.argwhere(snap.state == OCCUPIED)
    y = np.abs(snap.origin_y_m + (occ[:, 1] + 0.5) * snap.resolution_m)
    x = snap.origin_x_m + (occ[:, 0] + 0.5) * snap.resolution_m
    # the walls' own cells, beyond the outline + margin, stay marked
    assert ((y > WALL_Y - 0.05) & (x > -0.5) & (x < 1.4)).any()


def test_a_box_behind_in_the_region_rings_and_nothing_is_freed():
    lay = layer()
    speed, why = run(lay, box=(-0.17 - 0.30, -0.17 - 0.15, -0.15, 0.15))
    st = lay.exit_exempt_status()
    assert st["rang"]["hits"] > 0 and st["rang"]["first"][0]["x_m"] < -0.17
    assert lay.exit_verified is None and st["skipped"] == "unverified"
    assert speed == 0.0


def test_its_own_hits_do_not_ring_but_what_they_hide_stays_unknown():
    # an own part in the plane does not ring the watch; the cells behind it were never seen, and unknown cells are not
    # exempt (plan D2 "가림"): the permission still stops
    lay = layer()
    speed, why = run(lay, box=(-0.17 - 0.30, -0.17 - 0.15, -0.15, 0.15), own_box=True)
    st = lay.exit_exempt_status()
    assert st.get("rang") is None and st["transformed"]
    assert (speed, why) == (0.0, "unknown")


def test_the_watch_reads_the_ranges_the_grid_got_not_the_true_distances():
    # Codex 1st review P2-5: the shared scan's drawn ranges, not the cast's true ones. Walls 0.30 m out (well clear),
    # drawn 0.27 m short on the left (0.03 m beside the outline): the drawn points sit inside the margin and ring.
    lay = layer()
    shift = lambda ang, d: np.where(np.sin(ang) > 0.3, -0.27 / np.sin(ang).clip(0.3), 0.0)  # noqa: E731
    run(lay, y_wall=0.70, given_shift=shift)
    assert lay.exit_exempt_status()["rang"]["hits"] > 0
    lay2 = layer()
    run(lay2, y_wall=0.70)
    assert lay2.exit_exempt_status().get("rang") is None


def test_a_scan_without_its_correction_verifies_nothing():
    lay = layer()
    set_region(lay)
    for k in range(3):
        lay.add_scans(9.7 + 0.1 * k, scan(lay), odom_rear=P0, loaded=True)
    lay.refresh(9.95, (0.0, 0.0, 0.0), 0, current_pose=P0, path_ahead=PATH, loaded=True, direction=-1)
    st = lay.exit_exempt_status()
    assert st["unplaced_scans"] == 3 and st["skipped"] == "unverified"
    assert lay.limit(9.95, current_pose=P0, curvature_inv_m=0.0, direction=-1, loaded=True, cap_mps=0.18)[1] == "occupied"


def test_a_stale_or_other_version_verification_is_skipped():
    lay = layer()
    assert run(lay, refresh_at=9.9 + 0.25) [1] != "ok"
    assert lay.exit_exempt_status()["skipped"] == "stale"
    lay = layer()
    run(lay, refresh_version=1)
    assert lay.exit_exempt_status()["skipped"] == "version"


def test_the_scan_is_placed_with_its_own_correction():
    # the same scan under a correction that moves it 0.3 m sideways: the region goes with it (same verification),
    # and the refresh with that correction frees the same cells relative to the truck
    lay = layer()
    set_region(lay)
    for k in range(3):
        lay.add_scans(9.7 + 0.1 * k, scan(lay), odom_rear=P0, loaded=True, correction=(0.0, 0.3, 0.0), version=4)
    lay.refresh(9.95, (0.0, 0.3, 0.0), 4, current_pose=(0.0, 0.3, 0.0), path_ahead=PATH + [0.0, 0.3, 0.0],
                loaded=True, direction=-1)
    st = lay.exit_exempt_status()
    assert st["verified_version"] == 4 and st["transformed"]
    assert lay.limit(9.95, current_pose=(0.0, 0.3, 0.0), curvature_inv_m=0.0, direction=-1, loaded=True,
                     cap_mps=0.18)[1] == "ok"


def test_clear_ends_it_and_a_new_latch_starts_clean():
    lay = layer()
    run(lay, box=(-0.47, -0.32, -0.15, 0.15))
    assert lay.exit_exempt_status()["rang"]
    lay.exit_exempt_clear("watch")
    assert lay.exit_exempt_status() == {"closed": "watch"}
    lay.refresh(9.96, (0.0, 0.0, 0.0), 0, current_pose=P0, path_ahead=PATH, loaded=True, direction=-1)
    assert lay.exit_exempt_status() == {"closed": "watch"}  # untransformed, nothing recorded
    set_region(lay)
    assert lay.exit_exempt_status() == {} and lay.exit_verified is None


def test_a_call_without_any_scan_verifies_nothing():
    # Codex impl review P1-1: an empty raw (a silenced sensor) is not an observation
    lay = layer()
    for k in range(3):
        lay.add_scans(9.7 + 0.1 * k, scan(lay), odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0), version=0)
    set_region(lay)
    lay.add_scans(10.0, {}, odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0), version=0)
    lay.refresh(10.05, (0.0, 0.0, 0.0), 0, current_pose=P0, path_ahead=PATH, loaded=True, direction=-1)
    st = lay.exit_exempt_status()
    assert st["empty_calls"] == 1 and st.get("verified_s") is None and st["skipped"] == "unverified"
    assert lay.limit(10.05, current_pose=P0, curvature_inv_m=0.0, direction=-1, loaded=True, cap_mps=0.18)[0] == 0.0


def _with_beam(raw, beam, value, own=False):
    out = {}
    for name, (dist, hit, own_, *rest) in raw.items():
        dist, hit, own_ = dist.copy(), hit.copy(), own_.copy()
        dist[beam], hit[beam], own_[beam] = value, True, own
        out[name] = (dist, hit, own_, *rest)
    return out


def test_a_non_own_return_inside_the_minimum_range_rings():
    # Codex impl review round 2 P1: ranges() turns it into -inf, the grid marks near occupancy; the watch dropped it
    lay = layer()
    for k in range(3):
        lay.add_scans(9.7 + 0.1 * k, scan(lay), odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0), version=0)
    set_region(lay)
    lay.add_scans(10.0, _with_beam(scan(lay), 800, 0.10), odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0),
                  version=0)
    st = lay.exit_exempt_status()
    assert st["rang"]["near"] and st["rang"]["hits"] == 1 and lay.exit_verified is None
    lay2 = layer()
    set_region(lay2)
    lay2.add_scans(10.0, _with_beam(scan(lay2), 800, 0.10, own=True), odom_rear=P0, loaded=True,
                   correction=(0.0, 0.0, 0.0), version=0)
    assert lay2.exit_exempt_status().get("rang") is None and lay2.exit_verified is not None  # its own part


def test_a_scan_with_beams_that_read_nothing_verifies_nothing():
    lay = layer()
    set_region(lay)
    raw = {name: (d, h, o, np.full(len(d), np.inf), np.full(len(d), np.nan))
           for name, (d, h, o) in scan(lay).items()}
    lay.add_scans(10.0, raw, odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0), version=0)
    st = lay.exit_exempt_status()
    assert st["invalid_scans"] == 1 and st.get("verified_s") is None and lay.exit_verified is None


def test_a_beam_that_read_nothing_does_not_hide_a_danger_in_the_same_scan_nor_keep_the_last_verification():
    # Codex impl review round 3 P1: NaN on one beam and a near or in-region hit on another, after a valid verification
    lay = layer()
    set_region(lay)
    lay.add_scans(10.0, scan(lay), odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0), version=0)
    assert lay.exit_verified is not None
    raw = _with_beam(_with_beam(scan(lay), 800, 0.10), 100, np.nan)
    raw = {n: (d, h, o, np.full(len(d), np.inf), np.where(np.arange(len(d)) == 100, np.nan,
                                                         np.where(h, np.where(np.arange(len(d)) == 800, -np.inf, d), np.inf)))
           for n, (d, h, o) in raw.items()}
    lay.add_scans(10.1, raw, odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0), version=0)
    assert lay.exit_exempt_status()["rang"]["near"] and lay.exit_verified is None
    # a box in the region with a NaN beam rings too
    lay = layer()
    set_region(lay)
    lay.add_scans(10.0, scan(lay), odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0), version=0)
    boxed = scan(lay, box=(-0.47, -0.32, -0.15, 0.15))
    raw = {n: (d, h, o, np.full(len(d), np.inf), np.where(np.arange(len(d)) == 100, np.nan, np.where(h, d, np.inf)))
           for n, (d, h, o) in boxed.items()}
    lay.add_scans(10.1, raw, odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0), version=0)
    assert lay.exit_exempt_status()["rang"]["hits"] > 0 and lay.exit_verified is None
    # NaN alone: no ring, and the last verification is dropped
    lay = layer()
    set_region(lay)
    lay.add_scans(10.0, scan(lay), odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0), version=0)
    clean = scan(lay)
    raw = {n: (d, h, o, np.full(len(d), np.inf), np.where(np.arange(len(d)) == 100, np.nan, np.where(h, d, np.inf)))
           for n, (d, h, o) in clean.items()}
    lay.add_scans(10.1, raw, odom_rear=P0, loaded=True, correction=(0.0, 0.0, 0.0), version=0)
    st = lay.exit_exempt_status()
    assert st.get("rang") is None and st["invalid_scans"] == 1 and lay.exit_verified is None
