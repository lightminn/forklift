"""L1′ single-configuration study (plan v10): helpers and one synthetic replay end to end."""

import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("p0b_sensor_study", ROOT / "tools/p0b_sensor_study.py")
P0B = importlib.util.module_from_spec(SPEC)
sys.modules["p0b_sensor_study"] = P0B
SPEC.loader.exec_module(P0B)
sys.path.insert(0, str(ROOT / "tests/unit"))
from pallet_urdf_fixture import write_pallet_urdf  # noqa: E402

URDF = "sim/models/dls08_measured/forklift.urdf"


def prop(x, y, length, width, height, yaw=0.0):
    return {"asset": {"uri": "x.usd", "length_m": length, "width_m": width, "height_m": height},
            "rectangle": {"x_m": x, "y_m": y, "length_m": length, "width_m": width, "yaw_rad": yaw}}


def test_prism_tops_come_from_the_loads_and_the_column_count_is_checked():
    result = {"prism_colliders": {"root": "/World/PrismColliders", "columns": 2},
              "scenario": {"props": [prop(0, 0, 1.2, 1.0, 0.2), prop(5, 0, 1.0, 1.0, 1.4)]}}
    meta = {"obstacles": [
        {"x_m": 0.1, "y_m": 0.0, "length_m": 0.8, "width_m": 0.6, "yaw_rad": 0, "height_m": 0.5, "base_m": 0.2},
        {"x_m": 0.1, "y_m": 0.0, "length_m": 0.8, "width_m": 0.6, "yaw_rad": 0, "height_m": 0.5, "base_m": 0.7},
        {"x_m": 9.0, "y_m": 0.0, "length_m": 0.8, "width_m": 0.6, "yaw_rad": 0, "height_m": 3.0, "base_m": 0.2},
    ]}
    (r0, top0), (r1, top1) = P0B.run_prisms(result, meta)
    assert top0 == pytest.approx(1.2) and top1 == pytest.approx(1.4)  # the far load sits on neither
    assert (r0.length_m, r0.width_m) == (1.2, 1.0)
    with pytest.raises(SystemExit):
        P0B.run_prisms({**result, "prism_colliders": {"columns": 3}}, meta)
    with pytest.raises(SystemExit):
        P0B.run_prisms({**result, "prism_colliders": None}, meta)


def test_spawned_boxes_live_until_removed_and_silence_is_refused():
    log = [{"time_s": 5.0, "action": "spawn", "detail": ["n1", 1.0, 2.0, 0.3, [0.4, 0.4, 1.3]]},
           {"time_s": 9.0, "action": "remove", "detail": ["n1"]},
           {"time_s": 7.0, "action": "spawn", "detail": ["n2", 3.0, 2.0, 0.0, [0.4, 0.4, 0.5]]}]
    boxes = {b[0]: b for b in P0B.run_new_obstacles({"new_obstacles": {"log": log}})}
    assert boxes["n1"][2:] == (1.3, 5.0, 9.0) and boxes["n2"][4] == math.inf
    with pytest.raises(SystemExit):
        P0B.run_new_obstacles({"new_obstacles": {"log": [{"time_s": 1.0, "action": "silence", "detail": ["high"]}]}})


def test_injected_boxes_sit_on_the_path_about_to_be_driven():
    t = np.arange(0.0, 20.0, 0.1)
    v = np.where(t < 10.0, 0.3, -0.3)  # 3 m forward, then 3 m back
    x = np.cumsum(v * 0.1)
    yaw = np.zeros_like(x)
    rear = np.column_stack((x, np.zeros_like(x), yaw))
    poses = P0B.injection_poses(t, rear, v, np.ones(len(t), bool), 1.0, ahead_m=(2.5,), lead_m=(1.29, 0.17))
    xs = [p[0] for p in poses]
    assert all(p[3] < len(t) and t[p[3]] >= 3.0 for p in poses)  # placed when 1 m, 2 m, ... were driven
    # Forward: 2.5 m of arc on, cut at the cusp (3 m) -- the first box goes in at 1 m of
    # travel, so it sits at the cusp, past the front face.
    assert xs[0] == pytest.approx(3.0 + 1.29 + 0.25, abs=0.05)
    assert max(xs) <= 3.0 + 1.29 + 0.25 + 0.05  # never past the cusp
    # Reverse: behind the rear face of where the leg goes.
    assert any(p[0] < 2.0 - 0.17 - 0.25 + 0.05 for p in poses[-3:])


def test_a_class_without_events_does_not_pass():
    classes = {"unloaded_forward_straight": {"moving": 100, "covered": 95, "event_objects": 31, "unpermitted": 0},
               "loaded_reverse_curve": {"moving": 50, "covered": 50, "event_objects": 0, "unpermitted": 0}}
    out = P0B.judge_classes(classes, coverage_min=0.8, objects_min=30)
    assert out["classes"]["unloaded_forward_straight"]["verdict"] == "pass"
    assert out["classes"]["loaded_reverse_curve"]["verdict"] == "insufficient"  # no evidence, no failure
    assert out["classes"]["loaded_forward_curve"]["verdict"] == "absent"
    assert out["status"] == "incomplete" and not out["pass"]
    classes["loaded_reverse_curve"]["unpermitted"] = 1
    assert P0B.judge_classes(classes, coverage_min=0.8, objects_min=30)["status"] == "fail"
    classes["loaded_reverse_curve"]["unpermitted"] = 0
    classes["loaded_reverse_curve"]["event_objects"] = 30
    # Two passing classes and six absent ones: no evidence for the six (Codex L1′ 1st P1-2).
    out = P0B.judge_classes(classes, coverage_min=0.8, objects_min=30)
    assert out["status"] == "incomplete" and not out["pass"] and len(out["absent"]) == 6
    for cls in P0B.MOTION_CLASSES:
        classes[cls] = {"moving": 10, "covered": 9, "event_objects": 30, "unpermitted": 0}
    assert P0B.judge_classes(classes, coverage_min=0.8, objects_min=30)["status"] == "pass"


def synthetic_run(tmp_path: Path, *, speed=0.3, seconds=6.0, prism_x=1.0) -> Path:
    """A truck driving straight along +x at constant speed into one tall prism (its fork tips
    reach the face at about 5.8 s; the replay drives on through it)."""
    run = tmp_path / "run"
    run.mkdir()
    write_pallet_urdf(run / "pallet_with_synthetic_inertia.urdf")
    stamps = np.arange(0.0, seconds, 1.0 / 120.0)
    x = -2.0 + speed * stamps
    pose = np.column_stack((x, np.zeros_like(x), np.zeros_like(x), np.ones_like(x), np.zeros((len(x), 3))))
    rates = np.zeros((len(stamps), 4))
    rates[:, 2:4] = speed / 0.125
    scans = np.arange(0.0, seconds - 0.05, 0.1)
    np.savez(run / "slam_log.npz", joint_stamps_s=stamps, base_pose_world=pose, wheel_rates_rad_s=rates,
             steering_rad=np.zeros((len(stamps), 2)), scan_stamps_s=scans, silenced_scan_stamps_s=np.zeros(0))
    meta = {
        "odometry_geometry": {"wheelbase_m": 0.66, "track_m": 0.53, "wheel_radius_m": 0.125, "rear_axle_x_in_base_m": -0.34},
        "obstacles": [{"x_m": prism_x, "y_m": 0.0, "length_m": 0.6, "width_m": 0.6, "yaw_rad": 0.0, "height_m": 0.2, "base_m": 0.0},
                      {"x_m": prism_x, "y_m": 0.0, "length_m": 0.5, "width_m": 0.5, "yaw_rad": 0.0, "height_m": 1.1, "base_m": 0.2}],
        "hall": {"x_min_m": -10, "x_max_m": 10, "y_min_m": -10, "y_max_m": 10},
        "laser": {"beam_count": 1600, "range_min_m": 0.2, "range_max_m": 12.0, "mount_xyz_m": [-0.12, 0.0, 1.05]},
    }
    (run / "meta.json").write_text(json.dumps(meta))
    samples = [{"time_s": float(t), "phase": "observe", "lift_m": 0.0, "pallet_position_m": [8.0, 8.0, 0.0],
                "pallet_yaw_rad": 0.0} for t in np.arange(0.0, seconds, 0.1)]
    result = {
        "seed": 1, "arguments": {"forklift_urdf": URDF, "rear_axle_offset_m": -0.34},
        "forklift_urdf_sha256": hashlib.sha256((ROOT / URDF).read_bytes()).hexdigest(),
        "geometry": {"unloaded_footprint": {"front_m": 1.29, "rear_m": 0.17, "half_width_m": 0.36},
                     "loaded_footprint": {"front_m": 1.56, "rear_m": 0.17, "half_width_m": 0.4},
                     "axle_to_fork_tip_m": 1.29, "carriage_limit_m": 0.346, "pallet_depth_m": 0.6, "pallet_width_m": 0.8},
        "scenario": {"props": [prop(prism_x, 0.0, 0.6, 0.6, 0.2)],
                     "bounds": {"x_min_m": -10, "x_max_m": 10, "y_min_m": -10, "y_max_m": 10}},
        "prism_colliders": {"root": "/World/PrismColliders", "columns": 1},
        "chassis_model": {"forklift_urdf": URDF}, "scene_chassis_matches_urdf": True,
        "feedback": "simulator_ground_truth",
        "samples": samples, "transitions": [],
    }
    (run / "result.json").write_text(json.dumps(result))
    return run


def single_args(run, **kw):
    base = dict(run=run if isinstance(run, list) else [run], layer_config=ROOT / "config/obstacle_layer_single.yaml", phases="observe", moving_mps=0.05,
                inject_every_m=0.0, free_check_every=5, noise_seed=None, max_scans=None, coverage_min=0.8,
                min_event_objects=1, tick_every=1)
    base.update(kw)
    return SimpleNamespace(**base)


def test_a_tall_prism_ahead_is_seen_and_the_truck_is_never_let_into_it(tmp_path):
    out = P0B.single_command(single_args(synthetic_run(tmp_path)))
    run = out["runs"][0]
    row = run["classes"]["unloaded_forward_straight"]
    assert run["prisms"] == {"columns": 1, "min_top_m": pytest.approx(1.3)}  # top from its load
    assert row["moving"] > 0 and row["objects"] == 1 and row["events"] > 0
    assert row["unpermitted"] == 0
    # Judged every control tick, not only at the 10 Hz scans (Codex L1′ 1st P1-1).
    assert run["ticks"] == row["moving"] > 10 * run["scans"]
    assert run["self_hit_beams_max"] == 0  # nothing of the measured chassis at 1.05 m
    assert run["free_centres_in_prisms"] == 0
    assert run["face_moving"]["front"] == 0.0  # the plane sees over the truck
    assert out["verdict"]["classes"]["unloaded_forward_straight"]["verdict"] == "pass"
    assert out["verdict"]["status"] == "incomplete"  # seven classes have no motion here


def test_known_pallets_on_or_a_foreign_urdf_are_refused(tmp_path):
    run = synthetic_run(tmp_path)
    cfg = (ROOT / "config/obstacle_layer_single.yaml").read_text().replace("  enabled: false", "  enabled: true")
    (tmp_path / "on.yaml").write_text(cfg)
    with pytest.raises(SystemExit):
        P0B.single_command(single_args(run, layer_config=tmp_path / "on.yaml"))
    result = json.loads((run / "result.json").read_text())
    result["forklift_urdf_sha256"] = "0" * 64
    (run / "result.json").write_text(json.dumps(result))
    with pytest.raises(SystemExit):
        P0B.single_command(single_args(run))


def test_a_permission_that_always_lets_go_is_caught(tmp_path, monkeypatch):
    # Negative control: the truth side counts the permitted events, so a grid that sees
    # nothing (every snapshot cell FREE, fresh) shows up as unpermitted entries and fails
    # the class -- judged on the stopping arc, whatever the path check gets.
    import obstacle_layer
    from forklift_core.perception.obstacle_grid import FREE

    refresh = obstacle_layer.ObstacleLayer.refresh

    def blind(self, now_s, correction, version, **kw):
        out = refresh(self, now_s, correction, version, **kw)
        self.snapshot.state[:] = FREE
        self.snapshot.free_stamp[:] = now_s
        self.permission.update(self.snapshot, kw["path_ahead"], *self.footprints(kw["loaded"], kw["direction"]),
                               current_pose=kw["current_pose"], direction=kw["direction"])
        return out

    monkeypatch.setattr(obstacle_layer.ObstacleLayer, "refresh", blind)
    out = P0B.single_command(single_args(synthetic_run(tmp_path)))
    row = out["runs"][0]["classes"]["unloaded_forward_straight"]
    assert row["unpermitted"] > 0
    assert out["verdict"]["classes"]["unloaded_forward_straight"]["verdict"] == "fail"


def test_the_face_band_stops_at_the_first_observed_cell():
    from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN
    from forklift_core.planning.geometry import Footprint

    res = 0.05
    state = np.full((80, 80), FREE, dtype=np.int8)
    stamp = np.full((80, 80), 10.0)
    snap = SimpleNamespace(state=state, free_stamp=stamp, resolution_m=res, origin_x_m=-2.0, origin_y_m=-2.0)
    body = Footprint(1.0, 0.2, 0.3)
    assert P0B.face_unobserved_m(snap, (0.0, 0.0, 0.0), body, 10.0, 0.2)["front"] == 0.0
    # Two unknown cells against the front face, then an obstacle with an unknown inside:
    # the band is the two cells, not the inside.
    i0 = int((1.0 + 2.0) / res)
    state[i0 : i0 + 2, :] = UNKNOWN
    state[i0 + 2, :] = OCCUPIED
    state[i0 + 3 : i0 + 8, :] = UNKNOWN
    assert P0B.face_unobserved_m(snap, (0.0, 0.0, 0.0), body, 10.0, 0.2)["front"] == pytest.approx(0.101)
    # A band thinner than the probe step still shows (Codex L1′ 1st P3-10: 9.8 mm read 0).
    state[i0 : i0 + 2, :] = FREE
    state[i0, :] = UNKNOWN
    assert 0.0 < P0B.face_unobserved_m(snap, (0.0, 0.0, 0.0), body, 10.0, 0.2)["front"] <= 0.052
    # A rotated face that only clips a cell corner (Codex L1′ 2nd P3-5: 25 mm probes read 0).
    state[:] = FREE
    ci, cj = int((1.20 + 2.0) / res), int((0.50 + 2.0) / res)
    state[ci, cj] = UNKNOWN
    # The face corner (v = 0.36) lands at (1.205, 0.508): inside the cell for the last 8 mm of the face.
    clip = P0B.face_unobserved_m(snap, (0.0, 0.0, 0.12), Footprint(1.257, 0.2, 0.36), 10.0, 0.2)
    assert clip["front"] > 0.0
    stamp[:] = 9.0  # every FREE cell expired: the rear face sees nothing fresh up to 0.5 m
    assert P0B.face_unobserved_m(snap, (0.0, 0.0, 0.0), body, 10.0, 0.2)["rear"] is None


def test_merge_sums_the_parts_and_refuses_mixed_configs(tmp_path):
    def part(name, sha, objects, result_sha=None, every=2.0):
        totals = {c: {"moving": 0, "covered": 0, "permitted": 0, "event_objects": 0, "unpermitted": 0}
                  for c in P0B.MOTION_CLASSES}
        totals["unloaded_forward_straight"] = {"moving": 10, "covered": 9, "permitted": 5,
                                               "event_objects": objects, "unpermitted": 0}
        path = tmp_path / f"{name}.json"
        path.write_text(json.dumps({"layer_config_sha256": sha, "odometry_age_sha256": "t", "code_sha256": {"t": "c"},
                                    "settings": {"inject_every_m": every}, "totals": totals,
                                    "runs": [{"run": name, "run_id": result_sha or name, "result_sha256": result_sha or name}]}))
        return path

    args = SimpleNamespace(inputs=[part("a", "x", 20), part("b", "x", 15)], coverage_min=0.8, min_event_objects=30)
    old = json.loads(args.inputs[0].read_text())
    old.pop("code_sha256")
    (tmp_path / "old.json").write_text(json.dumps({**old, "runs": [{"run": "z", "result_sha256": "z"}]}))
    with pytest.raises(SystemExit):  # a part without a code version (Codex L1′ 5th P2-3)
        P0B.merge_command(SimpleNamespace(**{**vars(args), "inputs": args.inputs + [tmp_path / "old.json"]}))
    out = P0B.merge_command(args)
    row = out["verdict"]["classes"]["unloaded_forward_straight"]
    assert row["event_objects"] == 35 and row["coverage"] == pytest.approx(0.9) and row["verdict"] == "pass"
    for bad in (part("c", "y", 1), part("d", "x", 1, every=1.0), part("e", "x", 15, result_sha="a")):
        # A different config or setting, or the same run under another path (Codex L1′ 1st P2-6).
        with pytest.raises(SystemExit):
            P0B.merge_command(SimpleNamespace(**{**vars(args), "inputs": args.inputs + [bad]}))


def test_low_objects_duplicates_and_the_provisional_chassis_are_refused(tmp_path):
    run = synthetic_run(tmp_path)
    with pytest.raises(SystemExit):  # the same run twice cannot fill the event bound
        P0B.single_command(single_args([run, run]))
    result = json.loads((run / "result.json").read_text())
    (run / "result.json").write_text(json.dumps({**result, "chassis_model": {"forklift_urdf": "sim/models/dls08_provisional/forklift.urdf"}}))
    with pytest.raises(SystemExit):
        P0B.single_command(single_args(run))
    meta = json.loads((run / "meta.json").read_text())
    meta["obstacles"] = meta["obstacles"][:1]  # no load: the prism tops out at 0.2 m, below h_det
    (run / "meta.json").write_text(json.dumps(meta))
    (run / "result.json").write_text(json.dumps(result))
    with pytest.raises(SystemExit):
        P0B.single_command(single_args(run))


def test_the_leg_ends_at_a_cusp_and_phases_come_from_the_transitions():
    t = np.arange(0.0, 4.0, 0.1)
    v = np.where(t < 2.0, 0.3, -0.3)
    x = np.cumsum(v * 0.1)
    rear = np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))
    leg, direction = P0B.leg_ahead(rear, v, 0, 5.0)
    assert direction == 1 and len(leg) == 20  # stops where the truck reverses
    leg, direction = P0B.leg_ahead(rear, v, 25, 5.0)
    assert direction == -1
    transitions = [{"from": "observe", "to": "approach", "time_s": 1.0}, {"from": "approach", "to": "insert", "time_s": 2.0}]
    assert [P0B.phase_at(transitions, x) for x in (0.5, 1.0, 1.5, 2.5)] == ["observe", "approach", "approach", "insert"]


def test_recorded_slam_inputs_are_replayed_and_a_release_refreshes(tmp_path):
    # The same motion fed through recorded SLAM inputs: a map frame shifted and turned,
    # whose estimate moves at the 2.9 s scan and is applied by a release at 2.95 s (the
    # tracker applies what it received last, not the next scan's value).
    run = synthetic_run(tmp_path)
    log = np.load(run / "slam_log.npz")
    result = json.loads((run / "result.json").read_text())
    c1, c2 = (0.5, -0.3, 0.1), (0.52, -0.31, 0.104)

    def comp(a, b):
        c, s = math.cos(a[2]), math.sin(a[2])
        return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])

    stamps, base = log["joint_stamps_s"], log["base_pose_world"]
    recs = []
    for ts in log["scan_stamps_s"]:
        j = int(np.searchsorted(stamps, ts))
        recs.append({"stamp_s": float(ts), "odom_base": [float(base[j, 0]), float(base[j, 1]), 0.0],
                     "map_from_odom": list(c2 if ts >= 2.9 - 1e-9 else c1),
                     "applied_map_from_odom": list(c2 if ts >= 2.95 else c1)})
    control = np.array([[t, *comp(c2 if t >= 2.95 else c1, (base[j, 0] - 0.34, base[j, 1], 0.0)),
                         base[j, 0] - 0.34, base[j, 1], 0.0] for j, t in enumerate(stamps)])
    (run / "slam_records.json").write_text(json.dumps(recs))
    np.save(run / "slam_control.npy", control)
    result.update(feedback="slam_estimate", slam_summary={"holds": [{"time_s": 2.95, "event": "release"}]})
    (run / "result.json").write_text(json.dumps(result))
    out = P0B.single_command(single_args(run))
    x = out["runs"][0]
    row = x["classes"]["unloaded_forward_straight"]
    assert row["objects"] == 1 and row["events"] > 0 and row["unpermitted"] == 0
    assert x["free_centres_in_prisms"] == 0
    assert x["release_refreshes"] == 1 and x["releases"] == {"count": 1, "matched_next_scan": 1}
    with pytest.raises(SystemExit):  # a run whose inputs are not recorded
        result["feedback"] = "something_else"
        (run / "result.json").write_text(json.dumps(result))
        P0B.single_command(single_args(run))


def test_skipped_ticks_or_a_cut_record_never_pass(tmp_path):
    out = P0B.single_command(single_args(synthetic_run(tmp_path), tick_every=12))
    assert out["verdict"]["status"] == "diagnostic" and not out["verdict"]["pass"]


def test_motion_before_the_first_scan_is_uncovered(tmp_path):
    run = synthetic_run(tmp_path)
    log = dict(np.load(run / "slam_log.npz"))
    log["scan_stamps_s"] = log["scan_stamps_s"][log["scan_stamps_s"] >= 3.0]  # moving from 0 s, scans from 3 s
    np.savez(run / "slam_log.npz", **log)
    out = P0B.single_command(single_args(run))
    row = out["runs"][0]["classes"]["unloaded_forward_straight"]
    assert row["moving"] > 0 and row["covered"] / row["moving"] < 0.6  # the first 3 s count, uncovered


def test_the_truth_sweep_covers_between_samples():
    # Codex L1′ 4th P1: a loaded forward curve grazing a post between 25 mm samples.
    from forklift_core.control.drive_permission import arc_poses, shape_meets
    from forklift_core.planning.geometry import Footprint, Rectangle

    loaded = Footprint(1.56, 0.17, 0.40)
    post = Rectangle(1.660025, -0.589250, 0.4, 0.4, -1.180716)
    stop = 0.3 * 0.15 + 0.3 ** 2 / (2 * 1.5) + 0.05  # v = 0.3 m/s on the fixed stopping model
    vol, _ = arc_poses((0.0, 0.0, 0.0), 0.29, 1, stop, 0.025)
    fine, _ = arc_poses((0.0, 0.0, 0.0), 0.29, 1, stop, 0.0005)
    assert any(shape_meets(post, loaded, tuple(p), 0.01) for p in fine[1:])  # the sweep does meet it
    assert not any(shape_meets(post, loaded, tuple(p), 0.01) for p in vol[1:])  # 25 mm poses miss it
    assert P0B.sweep_meets(post, loaded, vol, 0.01, 1)
    # And a post well clear of the sweep is not met.
    assert not P0B.sweep_meets(Rectangle(0.0, 3.0, 0.4, 0.4, 0.0), loaded, vol, 0.01, 1)
    # Codex L1′ 5th P2-1: a post 5 mm behind the rear face is not met driving forward away
    # from it (the trailing face never grows), but it is reversing into it.
    behind = Rectangle(-0.17 - 0.005 - 0.1, 0.0, 0.2, 0.4, 0.0)
    straight, _ = arc_poses((0.0, 0.0, 0.0), 0.0, 1, 0.125, 0.025)
    assert not P0B.sweep_meets(behind, loaded, straight, 0.01, 1)
    back, _ = arc_poses((0.0, 0.0, 0.0), 0.0, -1, 0.125, 0.025)
    assert P0B.sweep_meets(behind, loaded, back, 0.01, -1)


def test_a_box_due_on_a_present_box_is_dropped(tmp_path):
    # Codex L1′ 5th P2-2: a reused leg end must not put two boxes on one spot.
    run = synthetic_run(tmp_path, seconds=12.0, prism_x=9.0)
    out = P0B.single_command(single_args(run, inject_every_m=0.5))
    x = out["runs"][0]
    assert x["injected_boxes"] >= 1
    assert x.get("injected_duplicates", 0) >= 1


def test_a_box_on_a_prism_is_dropped_and_a_changed_code_discards_the_result(tmp_path, monkeypatch):
    # Codex L1′ 6th P2-1: the one box (placed at 2 m of travel, at the record's end + the
    # front face + 0.25 m = x 2.80) lands on the prism there -- one collision shape, one object.
    run = synthetic_run(tmp_path, seconds=12.0, prism_x=2.80)
    x = P0B.single_command(single_args(run, inject_every_m=2.0))["runs"][0]
    assert x["injected_boxes"] == 1 and x["injected_duplicates"] == 1
    assert x["classes"]["unloaded_forward_straight"]["objects"] == 1
    # Codex L1′ 6th P2-2: hashed before and after the run; a change in between discards it.
    seq = iter([{"f": "a"}, {"f": "b"}])
    monkeypatch.setattr(P0B, "code_hashes", lambda: next(seq))
    with pytest.raises(SystemExit):
        P0B.single_command(single_args(run))


def test_injection_cycles_its_distances():
    t = np.arange(0.0, 40.0, 0.1)
    v = np.full(len(t), 0.3)
    rear = np.column_stack((np.cumsum(v * 0.1), np.zeros(len(t)), np.zeros(len(t))))
    poses = P0B.injection_poses(t, rear, v, np.ones(len(t), bool), 1.0, lead_m=(1.29, 0.17))
    gaps = [p[0] - rear[p[3], 0] - 1.29 - 0.25 for p in poses[:3]]
    assert gaps == pytest.approx([2.5, 1.75, 1.0], abs=0.04)


def test_a_spawn_on_a_present_box_is_one_object(tmp_path):
    # Codex L1′ 7th P2-1: the box (placed at 2 m of travel, at x 2.80) and a spawn on the
    # same spot after it -- one collision shape, one object, whichever came first.
    run = synthetic_run(tmp_path, seconds=12.0, prism_x=9.0)
    result = json.loads((run / "result.json").read_text())
    result["new_obstacles"] = {"log": [{"time_s": 8.0, "action": "spawn", "detail": ["n1", 2.80, 0.0, 0.0, [0.4, 0.4, 1.3]]}]}
    (run / "result.json").write_text(json.dumps(result))
    x = P0B.single_command(single_args(run, inject_every_m=2.0))["runs"][0]
    assert x["injected_duplicates"] == 1
    assert x["classes"]["unloaded_forward_straight"]["objects"] == 1


def test_a_spawn_covering_two_boxes_replaces_both(tmp_path):
    # Codex L1′ 8th P2: box0 (2.2025, 0) and box2 (1.7025, -0.25) under one 1.1 x 1.2 m spawn.
    run = synthetic_run(tmp_path, seconds=12.0, prism_x=9.0)
    base = P0B.single_command(single_args(run, inject_every_m=0.5))["runs"][0]
    result = json.loads((run / "result.json").read_text())
    result["new_obstacles"] = {"log": [{"time_s": 6.0, "action": "spawn", "detail": ["big", 1.9525, 0.0, 0.0, [1.1, 1.2, 1.3]]}]}
    (run / "result.json").write_text(json.dumps(result))
    x = P0B.single_command(single_args(run, inject_every_m=0.5))["runs"][0]
    assert x["injected_duplicates"] >= base.get("injected_duplicates", 0) + 2


def test_spawns_on_a_prism_or_on_each_other_are_one_object(tmp_path):
    # Codex L1′ 9th P2: twenty-nine spawns on the prism the truck drives into are one object.
    run = synthetic_run(tmp_path)
    result = json.loads((run / "result.json").read_text())
    result["new_obstacles"] = {"log": [{"time_s": 1.0 + 0.01 * k, "action": "spawn",
                                        "detail": [f"n{k}", 1.0, 0.0, 0.0, [0.6, 0.6, 1.3]]} for k in range(29)]}
    (run / "result.json").write_text(json.dumps(result))
    x = P0B.single_command(single_args(run))["runs"][0]
    assert x["classes"]["unloaded_forward_straight"]["objects"] == 1


def test_a_loaded_truck_gets_its_box_past_the_loaded_face():
    # Codex L1′ 9th P3: the lead is the outline the truck has at placement.
    t = np.arange(0.0, 20.0, 0.1)
    v = np.full(len(t), 0.3)
    rear = np.column_stack((np.cumsum(v * 0.1), np.zeros(len(t)), np.zeros(len(t))))
    poses = P0B.injection_poses(t, rear, v, np.ones(len(t), bool), 1.0, ahead_m=(2.5,), lead_m=(1.29, 0.17),
                                loaded_lead_m=(1.56, 0.17), loaded=np.ones(len(t), bool))
    p = poses[0]
    assert p[0] - rear[p[3], 0] == pytest.approx(2.5 + 1.56 + 0.25, abs=0.04)


def test_two_spawns_on_a_box_are_one_object(tmp_path):
    # Codex L1′ 10th P2: new:n1 -> new:n0 -> box0 resolves to one object.
    run = synthetic_run(tmp_path, seconds=12.0, prism_x=9.0)
    result = json.loads((run / "result.json").read_text())
    result["new_obstacles"] = {"log": [{"time_s": 8.0, "action": "spawn", "detail": [f"n{k}", 2.80, 0.0, 0.0, [0.4, 0.4, 1.3]]}
                                       for k in range(2)]}
    (run / "result.json").write_text(json.dumps(result))
    x = P0B.single_command(single_args(run, inject_every_m=2.0))["runs"][0]
    assert x["classes"]["unloaded_forward_straight"]["objects"] == 1


def test_a_config_changed_during_the_run_is_not_used(tmp_path, monkeypatch):
    # Codex L1′ 11th P2: the config is parsed from the bytes it is named by; a change later
    # does not reach the run and the recorded hash is that of what was used.
    import hashlib

    run = synthetic_run(tmp_path)
    cfg = tmp_path / "layer.yaml"
    cfg.write_text((ROOT / "config/obstacle_layer_single.yaml").read_text())
    used = hashlib.sha256(cfg.read_bytes()).hexdigest()
    orig = P0B.urdf_triangles

    def touch(path, joints):
        cfg.write_text(cfg.read_text().replace("latency_s: 0.15", "latency_s: 0.30"))
        return orig(path, joints)

    monkeypatch.setattr(P0B, "urdf_triangles", touch)
    out = P0B.single_command(single_args(run, layer_config=cfg))
    assert out["layer_config_sha256"] == used


def test_inputs_changed_during_the_run_are_not_used(tmp_path, monkeypatch):
    # Codex L1′ 12th/13th P2: each run input is read once, hashed and parsed from a private
    # copy -- a change to the original afterwards neither reaches the run nor its hashes.
    import hashlib

    run = synthetic_run(tmp_path)
    before = {n: hashlib.sha256((run / n).read_bytes()).hexdigest() for n in ("meta.json", "slam_log.npz")}
    orig = P0B.urdf_triangles

    def touch(path, joints):
        meta = run / "meta.json"
        m = json.loads(meta.read_text())
        m["obstacles"] = m["obstacles"][:1]  # would make the prism low (refused) if it were read
        meta.write_text(json.dumps(m))
        return orig(path, joints)

    monkeypatch.setattr(P0B, "urdf_triangles", touch)
    out = P0B.single_command(single_args(run))
    x = out["runs"][0]
    assert x["inputs_sha256"][str(run / "meta.json")] == before["meta.json"]
    assert x["inputs_sha256"][str(run / "slam_log.npz")] == before["slam_log.npz"]
    assert x["prisms"]["min_top_m"] == pytest.approx(1.3)


def test_a_copy_with_other_whitespace_is_the_same_run(tmp_path):
    # Codex L1′ 14th P2: the identity is the recorded motion, not the file bytes.
    import shutil

    run = synthetic_run(tmp_path)
    copy = tmp_path / "copy"
    shutil.copytree(run, copy)
    (copy / "result.json").write_text((run / "result.json").read_text() + "   ")
    with pytest.raises(SystemExit):
        P0B.single_command(single_args([run, copy]))
