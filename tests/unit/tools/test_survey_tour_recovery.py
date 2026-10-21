"""The survey tour's recoveries after V-G2 attempt 2 (gyro-heading plan docs/plans/2026-10-21-gyro-heading-odometry.md
4.5): F1 arm again after a disarm, F2 plan the coverage inside a margin, F3 replans that keep the judged cells
covered."""

import json

import numpy as np
import yaml

from tools import survey_tour as st

SCENE = {
    "scenario": {"seed": 0, "start_rear": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
                 "pickup": {"x_m": 1.8, "y_m": 0.0, "yaw_rad": 0.0}, "destination": {"x_m": 1.8, "y_m": 0.0, "yaw_rad": 0.0},
                 "props": [], "bounds": {"x_min_m": -3.0, "x_max_m": 3.0, "y_min_m": -3.0, "y_max_m": 3.0}},
    "geometry": {"unloaded_footprint": {"front_m": 1.29, "rear_m": 0.17, "half_width_m": 0.36}, "pallet_depth_m": 0.6,
                 "pallet_width_m": 0.8, "inserted_offset_m": 1.26, "prealign_offset_m": 3.79},
    "travel_planner_config": {"clearance_m": 0.1},
}


def _straight_with_cusp():
    fwd = np.column_stack((np.linspace(0, 2, 41), np.zeros(41), np.zeros(41)))
    back = np.column_stack((np.linspace(2, 1, 21)[1:], np.zeros(20), np.zeros(20)))
    poses = np.vstack((fwd, back))
    return poses, np.r_[np.ones(41), -np.ones(20)].astype(np.int8), np.zeros(len(poses))


def _state(seq, x=0.0, speed=0.0, armed=True, stopped=None, latch=None):
    return {"state_seq": seq, "mode": "TELEOP", "armed": armed, "rear_axle_speed_mps": speed,
            "stopped_confirmed": abs(speed) < 0.01 if stopped is None else stopped, "feedback_ok": True,
            "stop_latch": latch, "applied": {"speed_mps": speed, "curvature_inv_m": 0.0}, "steering_rad": [0.0, 0.0],
            "pose": {"x_m": x, "y_m": 0.0, "yaw_rad": 0.0, "correction_version": 7},
            "pacing": {"target_factor": 0.3, "ok": True}}


def _departed(**kw):
    poses = np.column_stack((np.linspace(0, 2, 41), np.zeros(41), np.zeros(41)))
    driver = st.TourDriver(poses, np.ones(41, dtype=np.int8), np.zeros(41), waypoints=[], **kw)
    driver.start(0.0)
    t, seq = 0.0, 0
    for x in np.linspace(0.0, 0.6, 12):
        seq, t = seq + 1, t + 0.1
        driver.step(t, _state(seq, x=x, speed=0.2))
    return driver, t, seq


# --- F1 ------------------------------------------------------------------------------------------------------------

def test_a_disarm_mid_tour_is_armed_again_and_the_tour_arrives():
    """V-G2 attempt 2 SEED 1: an expired deadline disarmed the teleop mid-drive and the tour never armed again."""
    out = st.simulate_tour(*_straight_with_cusp(), max_sim_s=300.0, disarm_at_s=(2.0, 8.0))
    names = [e["event"] for e in out["events"]]
    assert out["end"] == "arrived"
    assert names.count("disarmed") == 2 and names.count("rearm_sent") == 2 and names.count("rearmed") == 2
    assert "steer_timeout" not in names and "stall_replan" not in names


def test_without_arming_again_the_same_disarm_never_arrives():
    """The attempt 2 failure itself, on the CPU model: no arm sent, the truck stands until the rearm budget ends."""
    driver, t, seq = _departed(rearm_wait_s=5.0)
    end = None
    while end is None and t < 60.0:
        seq, t = seq + 1, t + 0.1
        _, end = driver.step(t, _state(seq, x=0.6, armed=False))
    assert end == "disarmed" and driver.rearm_request and 4.9 < t - 1.3 < 5.5


def test_arm_is_asked_only_once_the_stop_is_confirmed_and_once_per_disarm():
    driver, t, seq = _departed()
    seq, t = seq + 1, t + 0.1
    command, end = driver.step(t, _state(seq, x=0.6, speed=0.15, armed=False, latch={"kind": "preempt"}))
    assert command == (0.0, 0.0) and end is None and not driver.rearm_request  # still moving, latched
    seq, t = seq + 1, t + 0.1
    driver.step(t, _state(seq, x=0.62, armed=False, latch={"kind": "preempt"}))
    assert not driver.rearm_request  # standing but the latch holds
    seq, t = seq + 1, t + 0.1
    driver.step(t, _state(seq, x=0.62, armed=False, stopped=False))
    assert not driver.rearm_request  # the vehicle has not confirmed the stop
    seq, t = seq + 1, t + 0.1
    driver.step(t, _state(seq, x=0.62, armed=False))
    assert driver.rearm_request
    driver.rearm_sent(t, {"t": "ack", "arm_gen": 2})
    for _ in range(30):  # the old disarmed state still showing after the ack: no second arm
        seq, t = seq + 1, t + 0.1
        assert driver.step(t, _state(seq, x=0.62, armed=False)) == ((0.0, 0.0), None)
        assert not driver.rearm_request
    seq, t = seq + 1, t + 0.1
    command, end = driver.step(t, _state(seq, x=0.62))
    assert end is None and command[0] == 0.0  # armed: the leg starts again by steering standing
    rearmed = [e for e in driver.events if e["event"] == "rearmed"]
    assert len(rearmed) == 1 and driver.rearms == 1
    seq, t = seq + 1, t + 0.1
    command, end = driver.step(t, _state(seq, x=0.62))
    assert end is None and command[0] > 0.0


def test_a_refused_arm_is_asked_again_and_the_requests_are_capped():
    driver, t, seq = _departed(max_rearms=3, rearm_retry_s=1.0, rearm_wait_s=100.0)
    end = None
    while end is None and t < 50.0:
        seq, t = seq + 1, t + 0.1
        _, end = driver.step(t, _state(seq, x=0.6, armed=False))
        if driver.rearm_request:
            driver.rearm_sent(t, {"t": "reject", "reason": "invalid"})
    assert end == "disarmed" and driver.rearms == 3
    sent = [e["wall"] for e in driver.events if e["event"] == "rearm_sent"]
    assert all(b - a >= 1.0 - 1e-9 for a, b in zip(sent, sent[1:]))
    assert driver.events[-1]["event"] == "rearm_exhausted"


def test_disarmed_time_spends_neither_the_stall_nor_the_steering_budget():
    driver, t, seq = _departed(stall_s=5.0, replan_stall_s=3.0, steer_wait_s=2.0, rearm_wait_s=100.0)
    for _ in range(400):  # 40 s disarmed, refused every time (so it stays disarmed)
        seq, t = seq + 1, t + 0.1
        _, end = driver.step(t, _state(seq, x=0.6, armed=False))
        assert end is None
        if driver.rearm_request:
            driver.rearm_sent(t, {"t": "reject", "reason": "not_stopped"})
            driver.rearms = 0  # keep the cap out of this test
    names = [e["event"] for e in driver.events]
    assert "stall_replan" not in names and "stalled" not in names and "steer_timeout" not in names
    assert driver.drivable_idle == 0.0


def test_a_mode_that_leaves_teleop_still_fails_while_disarmed():
    driver, t, seq = _departed()
    seq, t = seq + 1, t + 0.1
    driver.step(t, _state(seq, x=0.6, armed=False))
    seq, t = seq + 1, t + 0.1
    assert driver.step(t, {**_state(seq, x=0.6, armed=False), "mode": "STOPPED"})[1] == "left_teleop"


def test_the_driven_poses_are_kept_at_their_spacing_with_their_state():
    driver, _, _ = _departed()
    xs = [d[0] for d in driver.driven]
    assert xs[0] == 0.0 and all(b - a >= 0.25 - 1e-9 for a, b in zip(xs, xs[1:]))
    assert driver.driven[0][2] == 1 and driver.driven[0][3] == 7


# --- F2 / F3 -------------------------------------------------------------------------------------------------------

def test_the_distance_is_the_same_whatever_the_chunks():
    rng = np.random.default_rng(3)
    points, sites = rng.uniform(-5, 5, (700, 2)), rng.uniform(-5, 5, (900, 2))
    full = np.sqrt(((points[:, None, :] - sites[None, :, :]) ** 2).sum(axis=2)).min(axis=1)
    assert np.array_equal(st.nearest_distance(points, sites, chunk=128), full)
    assert np.array_equal(st.nearest_distance(points, sites), full)


def test_a_replan_that_would_skip_a_judged_cell_adds_a_waypoint_for_it():
    """V-G2 attempt 2 gyro SEED 0: the replan through the waypoints left skipped a stretch the first path covered."""
    judged = np.array([[-2.0, 2.0]])
    start = (0.0, -2.0, 0.0)
    plain = st.plan_tour(SCENE, [], start=start)
    assert plain.success and st.region_coverage(judged, np.asarray(plain.poses)[:, :2], 1.0)["uncovered"] == 1
    log = []
    tour, plan, coverage = st.replan_tour(SCENE, [], start, driven=[(0.0, -2.0, 1, 0)], judged=judged, radius_m=1.0,
                                          log=log)
    assert plan.success and coverage["ok"] and len(tour) == 1 and any("added" in line for line in log)
    # the added waypoint stays for the next replan (the scene's planner default curvature is above the tour's 0.36)
    driver = st.TourDriver(*_straight_with_cusp(), waypoints=[], kappa=float(np.abs(plan.curvatures_inv_m).max()))
    driver.replan_request = (start, [])
    assert driver.adopt(1.0, plan, tour, coverage) and driver.waypoints == [tuple(tour[0])]


def test_the_poses_already_driven_count_as_covered():
    judged = np.array([[-2.0, 2.0]])
    tour, plan, coverage = st.replan_tour(SCENE, [], (0.0, -2.0, 0.0), driven=[(-2.0, 1.6, 3, 1)], judged=judged,
                                          radius_m=1.0)
    assert coverage["ok"] and tour == []


def test_a_cell_no_waypoint_can_cover_fails_the_replan_instead_of_dropping_it():
    judged = np.array([[-2.95, -2.95]])  # in the corner: no pose within 0.3 m of it keeps the clearance
    tour, plan, coverage = st.replan_tour(SCENE, [], (0.0, -2.0, 0.0), judged=judged, radius_m=0.3)
    assert plan.success and not coverage["ok"] and coverage["uncovered"] == 1
    driver = st.TourDriver(*_straight_with_cusp(), waypoints=[])
    driver.replan_request = ((0.0, -2.0, 0.0), [])
    assert not driver.adopt(1.0, plan, tour, coverage)
    assert driver.step(1.1, _state(1))[1] == "coverage_unrecoverable"
    assert driver.events[-1]["event"] == "coverage_unrecoverable" and driver.events[-1]["uncovered"] == 1


def test_without_judged_cells_a_replan_is_the_plain_plan():
    tour, plan, coverage = st.replan_tour(SCENE, [[-1.0, -1.5, 0.0]], (0.0, -2.0, 0.0))
    assert coverage is None and tour == [[-1.0, -1.5, 0.0]] and plan.success


def test_the_plan_with_a_margin_carries_its_judged_cells_and_fails_when_it_misses_them(tmp_path):
    result = tmp_path / "result.json"
    result.write_text(json.dumps({**SCENE, "arguments": {"rear_axle_offset_m": -0.34}}))
    config = tmp_path / "tour.yaml"
    config.write_text(yaml.safe_dump({"aisles": [[[-1.0, -1.5], [0.0, -1.5]]], "tour": [[-1.0, -1.5, 0.0]]}))
    out = tmp_path / "tour.npz"
    assert st.main(["plan", "--result", str(result), "--tour", str(config), "--out", str(out), "--margin-m", "0.3"]) == 0
    report = json.loads(out.with_suffix(".json").read_text())
    assert report["judged_margin"]["ok"] and report["judged_margin"]["radius_m"] == st.CELL_RADIUS_M - 0.3
    with np.load(out) as data:
        assert len(data["judged"]) == report["judged"]["cells"]
        assert float(data["coverage_radius_m"]) == st.CELL_RADIUS_M - 0.3
        assert json.loads(str(data["judged_source"]))["margin_m"] == 0.3
    # a margin no tour reaches: the plan succeeds but the tour is not ready
    assert st.main(["plan", "--result", str(result), "--tour", str(config), "--out", str(out), "--margin-m",
                    str(st.CELL_RADIUS_M - 0.01)]) == 1
    # without a margin the tour file is as before (no judged cells: replans are plain)
    assert st.main(["plan", "--result", str(result), "--tour", str(config), "--out", str(out)]) == 0
    with np.load(out) as data:
        assert "judged" not in data.files
