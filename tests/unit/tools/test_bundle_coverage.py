"""Teleop bundle margin plan V1: the coverage and residual tools on a synthetic run whose path curves 0.02 1/m less
than its steering joints say (the M-c evidence 1.5 shape)."""

import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from tools import bundle_coverage as bc
from tools import curvature_residual as cr

WHEELBASE, TRACK, OFFSET = 0.66, 0.53, 0.34


def steering_for(kappa):
    """Joint angles whose held_kappa is kappa (the Ackermann inverse of the runner's formula)."""
    half = TRACK / 2
    left = math.atan(WHEELBASE * kappa / (1 - kappa * half))
    right = math.atan(WHEELBASE * kappa / (1 + kappa * half))
    return left, right


def write_run(path, k_joint=-0.1, k_true=-0.08, speed=0.3, seconds=12.0, joint_wobble=0.0, k_out=None,
              scan_hz=0.0, joint_end=None, arguments=None):
    """No scans by default: installs then live until the refresh distance (the longest a cached check is used)."""
    dt = 1 / 120
    n = int(seconds / dt)
    t = np.arange(n) * dt
    s = speed * t
    yaw = k_true * s
    rx, ry = np.sin(yaw) / k_true, (1 - np.cos(yaw)) / k_true
    bx, by = rx + OFFSET * np.cos(yaw), ry + OFFSET * np.sin(yaw)
    quat = np.column_stack((np.cos(yaw / 2), 0 * yaw, 0 * yaw, np.sin(yaw / 2)))
    joint = k_joint + joint_wobble * np.sin(2 * np.pi * t / 0.1)
    if joint_end is not None:  # the measured curvature drifts linearly to joint_end
        joint = joint + (joint_end - k_joint) * t / t[-1]
    steer = np.array([steering_for(k) for k in joint])
    np.savez(path / "slam_log.npz", joint_stamps_s=t, base_pose_world=np.column_stack((bx, by, 0 * t, quat)),
             steering_rad=steer, scan_stamps_s=np.arange(0.0, seconds, 1 / scan_hz) if scan_hz else np.zeros(0))
    result = {"arguments": {"drive_geometry": {"wheelbase_m": WHEELBASE, "track_m": TRACK},
                            "rear_axle_offset_m": -OFFSET, **(arguments or {})},
              "geometry": {"unloaded_footprint": {"front_m": 1.29, "rear_m": 0.17, "half_width_m": 0.36},
                           "loaded_footprint": {"front_m": 1.56, "rear_m": 0.17, "half_width_m": 0.4},
                           "axle_to_fork_tip_m": 1.29, "carriage_limit_m": 0.346},
              "samples": [{"time_s": float(x), "phase": "observe",
                           "curvature_inv_m": k_joint if k_out is None else k_out} for x in t[::12]]}
    (path / "result.json").write_text(json.dumps(result))


def coverage_args(**k):
    base = dict(rule="new", margin_inv_m=0.02, refresh_m=0.25, envelope_m=0.01, latency_s=0.15, decel_mps2=1.5,
                stop_margin_m=0.05, arc_step_m=0.02, every=24, coarse_every=2, refine_within_m=0.004,
                speed_ahead_s=0.3, min_speed_mps=0.02, urdf_root=None,
                teleop_forward_mps=0.5, teleop_reverse_mps=0.25, teleop_loaded_mps=0.30)
    base.update(k)
    return SimpleNamespace(**base)


def test_the_residual_tool_reads_the_curvature_offset(tmp_path):
    write_run(tmp_path)
    args = SimpleNamespace(min_travel_m=0.03, max_window_s=0.5, steady_inv_m=0.005, wheelbase_m=0.66, track_m=0.53,
                           rear_axle_offset_m=0.34)
    _, rows, geometry = cr.run_rows(tmp_path, args)
    residual = np.array([r[4] for r in rows])
    assert geometry == (WHEELBASE, TRACK, OFFSET)
    assert np.median(residual) == pytest.approx(0.02, abs=1e-4)
    assert max(abs(r[5]) for r in rows) < 1e-6  # no sideways slip in a pure arc


def test_the_m_c_bundle_lets_the_body_out_and_the_margin_keeps_it_in(tmp_path):
    """0.04 1/m off at 0.5 m/s: over the 0.21 m stop the front corner leaves the 10 mm envelope (about 12 mm)."""
    write_run(tmp_path, k_joint=-0.1, k_true=-0.06, speed=0.5, seconds=6.0)
    old = bc.run_coverage(tmp_path, coverage_args(rule="old"))["modes"]["auto"]
    new = bc.run_coverage(tmp_path, coverage_args())["modes"]["auto"]
    assert old["checks"] > 10 and old["violation"] >= old["checks"] - 2  # the last checks reach the record's end
    assert new["checks"] == old["checks"] and new["violation"] == 0
    assert new["worst"]["arcs"] == pytest.approx([-0.15, -0.1, -0.05])


def test_a_scan_reinstalls_and_shortens_what_an_install_is_relied_on_for(tmp_path):
    """Every new scan is a new snapshot key: at 10 Hz the install a stop relies on is at most 0.1 s old."""
    write_run(tmp_path, scan_hz=10.0)
    out = bc.run_coverage(tmp_path, coverage_args(rule="old"))
    assert out["installs"] >= 115
    assert out["modes"]["auto"]["worst"]["install_age_m"] <= 0.031


def test_the_margin_must_reach_the_offset(tmp_path):
    """The grid's outward rounding is not a margin: an end just inside a grid point gets none of it."""
    write_run(tmp_path, k_joint=-0.051, k_true=-0.011, speed=0.5, seconds=6.0)
    assert bc.run_coverage(tmp_path, coverage_args(margin_inv_m=0.0))["modes"]["auto"]["violation"] > 0
    assert bc.run_coverage(tmp_path, coverage_args(margin_inv_m=0.02))["modes"]["auto"]["violation"] == 0


def test_a_reinstall_does_not_end_the_stop_the_last_install_promised(tmp_path):
    """Codex review r1 P1: the measured curvature wobbling across a grid point reinstalls every few cycles; the
    first tool ended each check there and saw nothing. A stop begun now is still the install's to keep."""
    write_run(tmp_path, k_joint=-0.121, k_true=-0.06, speed=0.5, joint_wobble=0.002, k_out=-0.121)
    out = bc.run_coverage(tmp_path, coverage_args())
    assert out["installs"] > 50
    assert out["modes"]["auto"]["violation"] > 0


def test_a_new_input_ends_the_check(tmp_path):
    """The output curvature leaving the install's arcs is a new input (design 4.4): that check stops there."""
    write_run(tmp_path, k_out=0.3)  # the command far from the arcs of the joints' -0.1
    out = bc.run_coverage(tmp_path, coverage_args(rule="old"))
    assert out["cuts"]["input"] == 0  # the old bundle spans joint to command, so the command stays inside
    write_run(tmp_path)
    log = dict(np.load(tmp_path / "slam_log.npz"))
    result = json.loads((tmp_path / "result.json").read_text())
    for row in result["samples"][30:]:
        row["curvature_inv_m"] = 0.3  # the operator steers the other way at 3 s
    (tmp_path / "result.json").write_text(json.dumps(result))
    np.savez(tmp_path / "slam_log.npz", **log)
    assert bc.run_coverage(tmp_path, coverage_args())["cuts"]["input"] > 0


def test_the_relied_distance_keeps_the_grid_gaps_inside_the_envelope():
    """Plan D3: between two grid arcs 0.05 1/m apart, an arc's body pokes out of the pair's union by less than the
    10 mm envelope over refresh 0.25 m + the stop from 0.5 m/s + a step (0.48 m), for both footprints."""
    stop = 0.5 * 0.15 + 0.5 ** 2 / 3.0 + 0.05
    length = 0.25 + stop + 0.02
    for front, rear, half in ((1.29, 0.17, 0.36), (1.56, 0.17, 0.40)):
        parts = [(front, rear, half, 0.0, 0.0)]
        pts = bc.outline(parts, 0.05)
        worst = 0.0
        for k1 in (-0.4, -0.15, -0.05, 0.0, 0.1, 0.35):
            from forklift_core.control.drive_permission import arc_poses

            pair = np.vstack([arc_poses((0.0, 0.0, 0.0), k, d, length, 0.02)[0] for k in (k1, k1 + 0.05)
                              for d in (1,)])
            for f in (0.25, 0.5, 0.75):
                mid = arc_poses((0.0, 0.0, 0.0), k1 + f * 0.05, 1, length, 0.02)[0]
                worst = max(worst, float(bc.outside_m(bc.placed(mid, pts), pair, parts).max()))
        assert worst < 0.01


def test_a_downward_old_bundle_keeps_its_checks(tmp_path):
    """Codex review r2 P1: the M-c bundle runs from measured to output; a steady -0.1 -> -0.12 is no new input."""
    write_run(tmp_path, k_joint=-0.1, k_true=-0.1, k_out=-0.12)
    out = bc.run_coverage(tmp_path, coverage_args(rule="old"))
    assert out["cuts"]["input"] == 0 and out["modes"]["auto"]["checks"] > 10


def test_the_blades_need_the_recorded_urdf_hash():
    """Codex review r2 P2: no recorded hash is no proof the URDF is the run's."""
    result = {"geometry": {"unloaded_footprint": {"front_m": 1.29, "rear_m": 0.17, "half_width_m": 0.36},
                           "loaded_footprint": {"front_m": 1.56, "rear_m": 0.17, "half_width_m": 0.4},
                           "axle_to_fork_tip_m": 1.29, "carriage_limit_m": 0.346},
              "arguments": {"forklift_urdf": "sim/models/dls08_measured/forklift.urdf"}}
    assert bc.shapes_of(result, 0.34, bc.ROOT)["parts_source"] == "hull (URDF hash not recorded)"


def test_the_end_stop_arcs_are_asked_at_the_curvatures_passed_now(tmp_path):
    """Codex review r3 P1: within one install the old bundle's ends drift, and BundlePermit asks the stop arcs at
    the current ones; asking them at the install's let a violation through."""
    write_run(tmp_path, k_joint=-0.092, joint_end=-0.097, k_out=-0.108, k_true=-0.062, speed=0.5, seconds=2.0)
    out = bc.run_coverage(tmp_path, coverage_args(rule="old", every=12))
    # 16 of 20 at the curvatures passed now; the install's ends gave 11 (Codex r4: ">= 1" passed with the defect)
    assert out["modes"]["auto"]["checks"] == 20 and out["modes"]["auto"]["violation"] >= 15


def test_a_silenced_scan_window_installs_nothing_new(tmp_path):
    """Codex review r3 P2: --live-test-scan-silence records the raw scans but keeps them from the obstacle layer."""
    write_run(tmp_path, scan_hz=10.0)
    loud = bc.run_coverage(tmp_path, coverage_args(rule="old"))["installs"]
    write_run(tmp_path, scan_hz=10.0, arguments={"live_test_scan_silence": "2:8"})
    quiet = bc.run_coverage(tmp_path, coverage_args(rule="old"))["installs"]
    assert quiet < loud - 40  # 60 scans silenced; the 0.25 m refresh still installs every 0.83 s


def test_a_forbidden_drive_cycle_is_neither_installed_nor_checked(tmp_path):
    """Codex review r4 P2: no arcs are asked while the drive is forbidden; the tool must not invent the check."""
    write_run(tmp_path, seconds=6.0)
    t = np.arange(int(6.0 * 120)) / 120
    rows = [{"teleop_k": float(x), "measured_kappa": -0.1, "output_kappa": -0.1, "speed_mps": 0.3,
             "permit_asked": bool(x < 3.0)} for x in t]
    (tmp_path / "live_log.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    out = bc.run_coverage(tmp_path, coverage_args())
    assert out["forbidden_cycles"] > 0
    assert out["modes"]["teleop"]["checks"] > 0 and out["modes"]["teleop"]["worst"]["t"] < 3.0
    assert out["modes"]["auto"]["checks"] == 0
