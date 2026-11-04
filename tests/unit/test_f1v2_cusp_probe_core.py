"""F1 v2 cusp arrival probe core (plan section 3, amendment 2): the registered path versions, the synthetic paths and
one cusp leg's trial against a kinematic truck -- the inferred endpoint judgement equals the tracker's own, and the
events change no command (the instrumentation equivalence)."""

import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim" / "isaac"))

import f1v2_probe_core as P  # noqa: E402

from forklift_core.control.path_tracking import AckermannGeometry, RearAxlePathTracker, TrackerConfig  # noqa: E402

GEOMETRY = AckermannGeometry(wheelbase_m=0.5, track_m=0.32, wheel_radius_m=0.08, max_steering_rad=0.6,
                             max_wheel_rate_rad_s=30.0)
TRANSPORT = TrackerConfig(cruise_speed_mps=0.3, max_curvature_inv_m=0.36, max_acceleration_mps2=0.3, lookahead_m=0.28,
                          position_tolerance_m=0.008, yaw_tolerance_rad=0.02, stop_speed_mps=0.012,
                          max_cross_track_error_m=0.35, cusp_position_tolerance_m=0.03, cusp_yaw_tolerance_rad=0.05,
                          cusp_brake_window_m=0.008, overshoot_tolerance_m=0.03)
OBSERVE = replace(TRANSPORT, cruise_speed_mps=0.6, position_tolerance_m=0.03, yaw_tolerance_rad=0.05,
                  cusp_heading_pending_rad=0.08)
DT = 1 / 120


def _drive(trial, start, *, yaw_bias=0.0, wrap=None, max_ticks=40000):
    """A kinematic rear-axle truck: the steering follows its command (tau 0.04 s), the speed the wheel command
    (tau 0.08 s); ``yaw_bias`` adds a yaw rate per unit speed within the leg's last 0.25 m (a heading miss at the
    cusp the tracker has no room to correct)."""
    x, y, yaw = start
    v, steer = 0.0, np.zeros(2)
    for k in range(max_ticks):
        planar = abs(v)
        kappa = math.tan(float(np.mean(steer))) / GEOMETRY.wheelbase_m
        near = math.hypot(trial.goal[0] - x, trial.goal[1] - y) < 0.25
        yaw_rate = v * kappa + (yaw_bias * abs(v) if near else 0.0)
        out = trial.step(k * DT, (x, y, yaw), v, planar, yaw_rate, steer)
        if wrap is not None:
            wrap(k)
        if out is None:
            return k
        rates, steer_cmd = out
        wheel = float(np.mean(rates[2:])) * GEOMETRY.wheel_radius_m  # the rear wheels' mean
        v += (wheel - v) * DT / 0.08
        steer = steer + (steer_cmd - steer) * DT / 0.04
        x, y, yaw = x + v * math.cos(yaw) * DT, y + v * math.sin(yaw) * DT, yaw + yaw_rate * DT
    raise AssertionError("the trial never ended")


def _trial(path, config, leg=0, record=True):
    return P.CuspTrial(path, config, leg, geometry=GEOMETRY, decel_mps2=1.5, accel_mps2=0.8, steering_rate_rad_s=1.2,
                       dt=DT, steering_now=np.zeros(2), record=record)


def test_synthetic_path_shape():
    p = P.synthetic_path((1.0, 2.0, 0.3), -0.36)
    assert len(P.leg_bounds(p["directions"])) == 2
    s, e = P.leg_bounds(p["directions"])[0]
    poses = np.asarray(p["poses"])
    length = np.linalg.norm(np.diff(poses[s:e + 1, :2], axis=0), axis=1).sum()
    assert length == pytest.approx(1.6, abs=1e-4)  # chords of the arc
    assert poses[e, 2] == pytest.approx(0.3 - 0.36 * 0.6)
    assert max(abs(c) for c in p["curvatures_inv_m"]) == pytest.approx(0.36)
    RearAxlePathTracker(p["poses"], p["directions"], p["curvatures_inv_m"], TRANSPORT)  # the tracker accepts it


def _path_record(start, curvature):
    p = P.synthetic_path(start, curvature)
    return {**p, "length_m": 2.4}


def test_registered_paths_rule_and_dedupe():
    a, b, c = (_path_record((0, 0, 0), k) for k in (0.2, 0.36, -0.2))
    results = {
        "r1": {"cusp_replans": [{"replaced_path": a, "path": b}], "paths_replaced": [], "paths": {"transport": c}},
        "r2": {"cusp_replans": [], "paths_replaced": [
            {"leg": "transport", "by": "scene_replan", "path": c, "time_s": 10.0},
            {"leg": "transport", "by": "delivery_straight", "path": a, "time_s": 90.0}], "paths": {"transport": b}},
        "r3": {"cusp_replans": [], "paths_replaced": [], "paths": {"transport": c}},
    }
    reg = P.registered_paths(results)
    assert [v["id"] for v in reg["versions"]] == ["r1/first", "r1/replan0", "r3/first"]
    assert reg["versions"][0]["source"] == "cusp_replans[0].replaced_path"
    why = {o["id"]: o["why"] for o in reg["omitted"]}
    assert why["r2/first"] == "duplicate" and why["r2/scene_replan@10.0"] == "replaced before the first registered plan"
    assert reg["versions"][2]["source"] == "paths.transport" and reg["versions"][0]["cusp_legs"] == 1


def _endpoint_truth(trial):
    """Wrap the trial's tracker so every update also records the tracker's own at_endpoint (path_tracking.update's
    formula on its state)."""
    tracker, truth = trial.tracker, []
    original = tracker.update

    def update(pose, speed, dt):
        endpoint = tracker.leg_end_index
        out = original(pose, speed, dt)
        cfg = tracker.config
        tol = cfg.cusp_position_tolerance_m if endpoint != len(tracker._poses) - 1 else cfg.position_tolerance_m
        window = min(cfg.cusp_brake_window_m, tol)
        remaining = max(0.0, float(tracker._distance[endpoint] - tracker._progress))
        goal = tracker._poses[endpoint]
        pose = np.asarray(pose, float)
        at = remaining <= window and (float(np.linalg.norm(goal[:2] - pose[:2])) <= tol
                                      or tracker._overshoot_accepted(pose, goal, endpoint, tol))
        truth.append(bool(at))
        return out

    tracker.update = update
    return truth


@pytest.mark.parametrize("config,bias,curvature", [(TRANSPORT, 0.0, 0.36), (TRANSPORT, 0.6, -0.2),
                                                   (OBSERVE, 0.10, 0.2), (OBSERVE, 0.6, -0.36)])
def test_inferred_endpoint_equals_the_trackers(config, bias, curvature):
    path = P.synthetic_path((0.0, 0.0, 0.0), curvature)
    trial = _trial(path, config)
    truth = _endpoint_truth(trial)
    _drive(trial, trial.start_pose(), yaw_bias=bias)
    assert trial.done and trial.outcome != "timeout"
    first = next((i for i, a in enumerate(truth) if a), None)
    got = trial.events["first_at_endpoint"]
    assert first is not None and got is not None and got["tick"] == first
    assert trial.stop is not None and trial.e_stop == pytest.approx(trial.yaw_error(trial.stop["rear"]))
    if trial.outcome == "release":
        assert trial.events["leg_switch"]["via"] == "arrival"
    assert trial.outcome in ("release", "failed_endpoint_heading", "pending", "stopped_not_released")


def test_heading_miss_outcomes_by_config():
    path = P.synthetic_path((0.0, 0.0, 0.0), 0.2)
    t1 = _trial(path, TRANSPORT)
    _drive(t1, t1.start_pose(), yaw_bias=0.6)
    t2 = _trial(path, OBSERVE)
    _drive(t2, t2.start_pose(), yaw_bias=0.6)
    # the same miss: transport latches endpoint_heading, observe holds it pending (cusp_heading_pending_rad)
    assert 0.05 < abs(t1.events["first_at_endpoint"]["tracker_yaw_error_rad"]) <= 0.08
    assert t1.outcome == "failed_endpoint_heading" and t1.events["first_failed"]["failure"] == "endpoint_heading"
    assert t2.outcome == "pending" and t2.events["first_pending"] is not None
    assert t1.events["first_exceed_after_endpoint"]["tick"] == t1.events["first_at_endpoint"]["tick"]


def test_record_changes_no_command_or_trajectory():
    for config, bias in ((TRANSPORT, 0.0), (OBSERVE, 0.6), (TRANSPORT, 0.6), (OBSERVE, 0.1)):
        path = P.synthetic_path((0.5, -1.0, 0.7), -0.36)
        on, off = _trial(path, config, record=True), _trial(path, config, record=False)
        _drive(on, on.start_pose(), yaw_bias=bias)
        _drive(off, off.start_pose(), yaw_bias=bias)
        assert on.trajectory_sha256() == off.trajectory_sha256() and on.outcome == off.outcome
        assert off.events is None and on.events["first_at_endpoint"] is not None


def test_release_holds_zero_until_the_stop_count():
    path = P.synthetic_path((0.0, 0.0, 0.0), 0.36)
    trial = _trial(path, TRANSPORT)
    _drive(trial, trial.start_pose())
    assert trial.outcome == "release"
    rows = np.asarray(trial.rows)
    col = {c: i for i, c in enumerate(P.TICK_COLUMNS)}
    after = rows[rows[:, col["advanced"]] == 1]
    assert len(after) >= P.CUSP_STOP_NEEDED - 1
    assert np.all(np.isnan(after[1:, col["trk_speed"]]))  # no tracker update after its own switch
    held = after[:, [col["steer_cmd_l"], col["steer_cmd_r"]]]
    assert np.all(held == held[0])
    assert np.all(np.abs(after[:, col["wheel_speed_cmd"]]) <= 0.02)


def test_skip_legs_starts_at_the_named_cusp_leg_and_time_cap():
    a = P.synthetic_path((0.0, 0.0, 0.0), 0.2)
    b = P.synthetic_path(a["poses"][-1], -0.2)  # a second cusp after the first path's reverse straight
    path = {"poses": a["poses"] + b["poses"][1:], "directions": a["directions"] + b["directions"][1:],
            "curvatures_inv_m": a["curvatures_inv_m"] + b["curvatures_inv_m"][1:]}
    bounds = P.leg_bounds(path["directions"])
    assert len(bounds) == 4
    trial = _trial(path, TRANSPORT, leg=1)
    assert trial.start_pose() == pytest.approx(path["poses"][bounds[1][0]]) and trial.end_index == bounds[1][1]
    assert trial.time_limit_s == pytest.approx(trial.leg_length_m / P.CUSP_TIME_SPEED_MPS)
    with pytest.raises(ValueError):
        _trial(path, TRANSPORT, leg=3)  # the last leg ends at the goal, not a cusp
    # a truck that never moves times out without a stop value
    k = 0
    while trial.step(k * DT, trial.start_pose(), 0.0, 0.0, 0.0, np.zeros(2)) is not None:
        k += 1
        if k * DT > trial.time_limit_s + 1:
            raise AssertionError("no time cap")
    # a standing truck at the start: its tracker drives, never terminal -> the time cap
    assert trial.outcome == "timeout" and trial.e_stop is None


def test_an_overshot_cusp_stall_ends_at_the_cores_stop():
    # observe at 0.6 m/s with this truck's lag overshoots the cusp beyond its tolerance: the tracker then commands 0 in
    # status tracking (a stall); the core's stop condition (speed 0, standing) confirms it, not the time cap
    path = P.synthetic_path((0.0, 0.0, 0.0), 0.2)
    trial = _trial(path, OBSERVE)
    _drive(trial, trial.start_pose(), yaw_bias=0.1)
    assert trial.outcome == "stopped_not_released" and trial.stop["position_error_m"] > 0.03
    assert trial.events["first_at_endpoint"] is not None and trial.stop["tick"] < 1000
