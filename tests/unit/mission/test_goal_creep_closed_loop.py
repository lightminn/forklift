"""The goal creep watch in a closed loop (docs/plans/2026-10-24-goal-creep-stall.md §3): the real tracker, the core's own
goal_creep() on its output, a kinematic plant with an optional deadband, the runner's stop detector, and three odometry
models -- ideal, team-spec (two rear wheel encoders with the MT6701 count and hysteresis, the odometry the mean of their
counted travel) and reported (wheel-rate noise). Every arrival the plan's reviews found (Codex rounds 1-8) arrives as
it did without the watch; the stops the old tracker never left are taken."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control import RearAxlePathTracker, TrackerConfig
from forklift_core.localization.encoders import QuadratureWheelEncoder, encoder_model_spec
from forklift_core.localization.slam_pose import StopDetector
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState

DT = 1 / 120
# The measured runner's transport / return followers (as tests/unit/control/test_leg_end_min_speed.py), goal 8 mm.
BASE = dict(cruise_speed_mps=0.6, max_curvature_inv_m=0.36, max_acceleration_mps2=0.3, lookahead_m=0.28,
            position_tolerance_m=0.008, yaw_tolerance_rad=0.05, stop_speed_mps=0.012, max_cross_track_error_m=0.35,
            cusp_position_tolerance_m=0.03, cusp_yaw_tolerance_rad=0.05, cusp_brake_window_m=0.008,
            overshoot_tolerance_m=0.03)
GOAL = TrackerConfig(**BASE)


def straight(direction=1, n=26):
    x = np.linspace(0.0, 1.0, n) * direction
    return np.column_stack((x, np.zeros(n), np.zeros(n))), np.full(n, direction, np.int8), np.zeros(n)


def along(upto, lateral, yaw, direction=1):
    return [[direction * x, lateral, yaw] for x in np.linspace(0.0, upto, 40)]

RADIUS, TRACK = 0.125, 0.53
SPEC = encoder_model_spec("team_spec_quantized")


def _watch(tracker):
    """A core with only what goal_creep() reads."""
    ns = SimpleNamespace(ms=MissionState(phase="return_home"), slam={}, trackers={"return_home": tracker}, state={},
                         goal_creep_watch={"key": None, "last_t": None, "window": [], "latched": False})
    for name in dir(MissionCore):
        if name.startswith("GOAL_CREEP_"):
            setattr(ns, name, getattr(MissionCore, name))
    return ns


class _TeamSpec:
    def __init__(self, reversed_before):
        self.encoders = [QuadratureWheelEncoder(SPEC.wheel_counts_per_rev, hysteresis_rad=SPEC.wheel_hysteresis_rad)
                         for _ in range(2)]
        q = 2 * math.pi / SPEC.wheel_counts_per_rev
        self.angles = [5 * q, 5 * q]
        for e, a in zip(self.encoders, self.angles):
            e.update_continuous(a + (3 * q if reversed_before else -3 * q))
            e.update_continuous(a)

    def step(self, ds, curvature):
        counted = []
        for i, side in enumerate((-1, 1)):
            self.angles[i] += ds * (1 + side * curvature * TRACK / 2) / RADIUS
            counted.append(self.encoders[i].update_continuous(self.angles[i]) * RADIUS)
        return float(np.mean(counted))


def run(path, config, setup, start, *, odometry="ideal", deadband=0.0, v0=0.0, hold=(0, 0), watch=True,
        reversed_before=False, seconds=20.0, seed=0):
    """Returns ("arrived" | "end" | "taken" | "stuck", time): "end" is the existing receivers' state (remaining
    within 1 um, command zero, stopped); "taken" is an R2 latch (its time) that then reached the stall rules' state
    -- the command held at zero, the stop detector stopped for 120 ticks."""
    tracker = RearAxlePathTracker(*path, config)
    for pose in setup:
        tracker.update(pose, 0.1, 0.02)
    tracker.restart_speed_slew(v0)
    core = _watch(tracker)
    truth = np.array(start, float)
    odom = truth.copy()
    stop = StopDetector(tick_s=DT)
    team = _TeamSpec(reversed_before) if odometry == "team_spec" else None
    rng = np.random.default_rng(seed)
    applied, odom_speed = v0, v0
    taken, standing = None, 0
    for k in range(int(seconds / DT)):
        t = k * DT
        c = tracker.update(odom, stop.tracker_speed(0.012), DT)
        if c.status == "arrived":
            return "arrived", round(t, 3)
        command = c.speed_mps
        if watch:
            core.slam.update(applied_speed=applied, odom_speed=odom_speed)
            verdict = MissionCore.goal_creep(core, t, c, DT)
            if verdict == "r2":
                taken = taken if taken is not None else t
                command = 0.0  # the core's zero command (creep_band / dock_stop)
            if verdict == "r1":
                command = 0.0
        if c.status == "tracking" and tracker.remaining_to_goal_m() <= 1e-6 and command == 0.0 and stop.stopped:
            return "end", round(t, 3)
        # After an R2 latch: the stall rules take it once the stop detector has seen 120 standing ticks (the existing
        # slam_stall_ticks count), the receivers' state -- what the stall replan and the docking stop then do is theirs.
        standing = standing + 1 if (taken is not None and command == 0.0 and stop.stopped) else 0
        if standing >= 120:
            return "taken", round(taken, 3)
        applied = 0.0 if hold[0] <= k < hold[1] else command
        v = applied if abs(applied) >= deadband else 0.0
        step = v * DT
        truth = truth + [step * math.cos(truth[2]), step * math.sin(truth[2]), step * c.curvature_inv_m]
        if odometry == "ideal":
            ds = step
        elif odometry == "team_spec":
            ds = team.step(step, c.curvature_inv_m)
        else:
            ds = (v + rng.normal(0.0, 0.2 * RADIUS / math.sqrt(2))) * DT
        odom_speed = ds / DT
        odom = odom + [ds * math.cos(odom[2]), ds * math.sin(odom[2]), ds * c.curvature_inv_m]
        stop.update(commanded_speed=applied, speed=odom_speed, yaw_rate=ds * c.curvature_inv_m / DT)
    return "stuck", seconds


def _arc(theta, radius_inv=0.29):
    r = 1 / radius_inv
    start = np.array([-r * math.sin(theta), r * (1 - math.cos(theta)), -theta])
    return (np.array([start, [0.0, 0.0, 0.0]]), np.ones(2, np.int8), np.full(2, radius_inv)), start


def _arrivals():
    cases = []
    for d in (1, -1):
        for lat, yaw in ((0.008000845, 0.0), (0.0079427, 0.02 * d)):  # Codex round 1
            cases.append((f"r1 d{d} {lat}", (straight(d), GOAL, along(0.997, lat, yaw, d), [0.997 * d, lat, yaw]), {}))
    cases.append(("r2 behind", (straight(), GOAL, along(0.9995, 0.00801, -0.02), [0.9986, 0.00801, -0.02]), {}))
    cases.append(("r2 hold", (straight(), GOAL, along(0.997, 0.008000845, 0.0), [0.997, 0.008000845, 0.0]),
                  {"hold": (0, 114)}))
    yaw03 = TrackerConfig(**{**BASE, "yaw_tolerance_rad": 0.03})
    path, p0 = _arc(2.0)
    start = [0.005428873102837442, 0.00818, -0.029]
    cases.append(("r3 arc", (path, yaw03, [p0 + f * (np.array(start) - p0) for f in np.linspace(0, 1, 80)], start), {}))
    s1 = (np.array([[-1.0, 0, 0], [0, 0, 0]]), np.ones(2, np.int8), np.zeros(2))
    no_over = TrackerConfig(**{**BASE, "yaw_tolerance_rad": 0.07, "overshoot_tolerance_m": None})
    start = [-0.003, 0.007806670864, 0.069]
    cases.append(("r3 no overshoot", (s1, no_over, [[x, start[1], start[2]] for x in np.linspace(-0.5, -0.003, 40)],
                                      start), {}))
    cases.append(("r4 16 um", (straight(), no_over, along(0.999984, 0.009, 0.0), [0.9991, 0.00795180125925386, 0.069]),
                  {}))
    cases.append(("r5 behind 10 mm", (straight(), GOAL, along(0.9999995, 0.009, 0.0), [0.990, 0.0075, 0.0]), {}))
    cases.append(("r5 behind 0.13 mm", (straight(), GOAL, along(0.9999995, 0.009, 0.0), [0.99987, 0.007999, 0.0]), {}))
    path, p0 = _arc(0.27, 0.22)
    path = (np.array([[0.008047167814230316, 0.10955704849724766, -0.22], [1.0, 0.0, 0.0]]), np.ones(2, np.int8),
            np.full(2, 0.22))
    start = [0.999995, 0.008228, -0.049]
    setup = [[*(path[0][0][:2] * (1 - f) + path[0][1][:2] * f), -0.049] for f in np.linspace(0, 1 - 5e-7, 60)]
    cases.append(("r6 braking", (path, GOAL, setup, start), {"v0": 0.055}))
    path = (np.array([[-2.3291833812108655, 5.991012812211191, -2.4], [0.0, 0.0, 0.0]]), np.ones(2, np.int8),
            np.full(2, 0.29))
    start = np.array([0.010705244400598066, 0.0084, -0.049])
    cases.append(("r7 reported entry", (path, GOAL, [path[0][0] + f * (start - path[0][0]) for f in np.linspace(0, 1, 80)],
                                         start), {"v0": 0.03}))
    path = (np.array([[-1.7775909373153937, 6.403064666789474, -2.6], [0.0, 0.0, 0.0]]), np.ones(2, np.int8),
            np.full(2, 0.29))
    start = np.array([0.02486704704807527, 0.00818, -0.049])
    cases.append(("r8 reversal", (path, GOAL, [path[0][0] + f * (start - path[0][0]) for f in np.linspace(0, 1, 80)],
                                   start), {"reversed_before": True}))
    return cases


@pytest.mark.parametrize("name,args,kw", _arrivals(), ids=[c[0] for c in _arrivals()])
@pytest.mark.parametrize("odometry", ["ideal", "team_spec", "reported"])
def test_every_arrival_the_reviews_found_is_kept(name, args, kw, odometry):
    seeds = range(10) if odometry == "reported" else (0,)
    for seed in seeds:
        before = run(*args, odometry=odometry, watch=False, seed=seed, **kw)
        after = run(*args, odometry=odometry, watch=True, seed=seed, **kw)
        if before[0] == "arrived":
            assert after == before, (name, odometry, seed)


@pytest.mark.parametrize("across", [0.0082, 0.0084])
@pytest.mark.parametrize("progress", [0.99996, 0.999])
def test_a_deadband_stop_the_old_tracker_never_left_is_taken(across, progress):
    args = (straight(), GOAL, along(progress, across, 0.0), [progress, across, 0.0])
    assert run(*args, odometry="team_spec", deadband=0.001, watch=False)[0] == "stuck"
    outcome, t = run(*args, odometry="team_spec", deadband=0.001)
    assert outcome == "taken" and t < 20.0  # 0.8 mm of a 60 um/s creep: 13.3 s


def test_a_stop_after_real_motion_is_taken_from_the_window_after_it():
    r = 20 * 0.0001917476 + 24e-6
    args = (straight(), GOAL, along(1 - r, 0.0084, 0.0), [1 - r, 0.0084, 0.0])
    assert run(*args, odometry="team_spec", deadband=4e-5, watch=False, seconds=60)[0] == "stuck"
    outcome, t = run(*args, odometry="team_spec", deadband=4e-5, seconds=60)
    assert outcome == "taken" and t < 30.0


def test_a_sub_micron_creep_at_the_progress_point_reaches_the_receivers():
    args = (straight(), GOAL, along(1 - 0.0001922476, 0.0084, 0.0), [1 - 0.0001922476, 0.0084, 0.0])
    assert run(*args, odometry="team_spec", deadband=1e-4, watch=False)[0] == "stuck"
    assert run(*args, odometry="team_spec", deadband=1e-4)[0] == "end"


def test_the_window_is_long_enough_for_the_team_spec_counts():
    # Plan G2 R2: over 0.8 mm of commanded travel the counted travel is at least 0.599 of it (worst start phase,
    # just reversed, the inner wheel at curvature 0.36), well above the 0.25 ratio; 0.4 mm would give 0.240.
    q = 2 * math.pi / SPEC.wheel_counts_per_rev

    def counted_min(travel):
        worst = math.inf
        for phase in np.linspace(0, q, 200, endpoint=False):
            e = QuadratureWheelEncoder(SPEC.wheel_counts_per_rev, hysteresis_rad=SPEC.wheel_hysteresis_rad)
            a0 = phase + 5 * q
            e.update_continuous(a0 + 3 * q)
            e.update_continuous(a0)
            total = sum(e.update_continuous(a0 + travel / RADIUS * k / 200) for k in range(1, 201))
            worst = min(worst, total * RADIUS)
        return worst

    window = MissionCore.GOAL_CREEP_WINDOW_M
    bent = 0.36 * TRACK / 2
    ratio = (counted_min(window * (1 - bent)) + counted_min(window * (1 + bent))) / 2 / window
    assert ratio > 0.5 > MissionCore.GOAL_CREEP_RATIO
