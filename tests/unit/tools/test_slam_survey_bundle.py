"""The SLAM survey bundle and its checks (SLAM localization plan docs/plans/2026-10-20-slam-localization-session.md L2,
S2, V2'), and the survey job's runner held to the console's."""

import json
import math
from pathlib import Path

import numpy as np

from tools import slam_survey_bundle as sb
from tools import survey_tour as st

ROOT = Path(__file__).resolve().parents[3]


def quat(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def nodes_of(rows):
    keys = ("unique_id", "state_id", "stamp_s", "x", "y", "yaw", "odom_x", "odom_y", "odom_yaw")
    return {k: np.asarray([float(r[i]) for r in rows]) for i, k in enumerate(keys)}


def test_the_map_id_names_the_three_files_and_does_not_depend_on_their_order():
    a = sb.map_id({"map.posegraph": "1", "map.data": "2", "grid.npz": "3"})
    assert a == sb.map_id({"grid.npz": "3", "map.data": "2", "map.posegraph": "1"})
    assert a != sb.map_id({"map.posegraph": "1", "map.data": "2", "grid.npz": "4"})


def test_karto_cells_read_as_ros_map_values():
    cells = np.array([[0, 100, 255]], dtype=np.uint8)
    assert sb.karto_to_map(cells).tolist() == [[-1, 100, 0]]


def test_the_world_check_compares_each_node_to_the_truth_at_its_stamp_without_alignment():
    stamps = np.arange(0.0, 1.0, 1 / 60)
    base = np.array([[t, 0.0, 0.1, *quat(math.pi - 0.001)] for t in stamps])
    # node 1 sits 0.05 m off; node 2's yaw wraps through pi (0.002 rad off, not 2 pi)
    nodes = nodes_of([(0, 0, stamps[6], stamps[6], 0.0, math.pi - 0.001, 0, 0, 0),
                      (1, 1, stamps[30], stamps[30], 0.05, math.pi - 0.001, 0, 0, 0),
                      (2, 2, stamps[42], stamps[42], 0.0, -math.pi + 0.001, 0, 0, 0)])
    out = sb.world_check(nodes, stamps, base, 0.5 / 60)
    assert out["paired"] == 3 and abs(out["position_max_m"] - 0.05) < 1e-12
    assert abs(out["yaw_max_rad"] - 0.002) < 1e-9 and out["ok"]
    nodes["x"][1] += 0.10  # now 0.112 m off
    assert not sb.world_check(nodes, stamps, base, 0.5 / 60)["ok"]


def test_a_node_with_no_truth_at_its_stamp_fails_the_world_check():
    stamps = np.arange(0.0, 1.0, 1 / 60)
    base = np.array([[0.0, 0.0, 0.1, *quat(0.0)] for _ in stamps])
    nodes = nodes_of([(0, 0, 5.0, 0, 0, 0, 0, 0, 0)])
    out = sb.world_check(nodes, stamps, base, 0.5 / 60)
    assert out["paired"] == 0 and not out["ok"]


def test_the_start_check_reads_the_first_nodes_odometry_against_the_truth():
    stamps = np.arange(0.0, 1.0, 1 / 60)
    base = np.array([[1.0, 2.0, 0.1, *quat(0.3)] for _ in stamps])
    nodes = nodes_of([(1, 1, stamps[20], 9, 9, 0, 1.0, 2.0, 0.3), (0, 0, stamps[3], 9, 9, 0, 1.0, 2.004, 0.3)])
    out = sb.start_check(nodes, stamps, base, 0.5 / 60)
    assert out["ok"] and abs(out["stamp_s"] - stamps[3]) < 1e-12  # the earliest node, not the first line
    nodes["odom_y"][1] = 2.02
    assert not sb.start_check(nodes, stamps, base, 0.5 / 60)["ok"]


def test_coverage_finds_the_farthest_path_point():
    nodes = np.array([[0.0, 0.0], [10.0, 0.0]])
    path = np.array([[7.5, 0.0], [0.0, 2.0], [10.0, 3.5]])
    out = sb.coverage_check(path, nodes, 3.0, chunk=2)
    assert not out["ok"] and out["outside"] == 1 and out["farthest_point"] == [10.0, 3.5]
    assert abs(out["farthest_m"] - 3.5) < 1e-12
    assert sb.coverage_check(path[:2], nodes, 3.0)["ok"]


def test_the_cost_check_records_processed_keyframes_only():
    records = [{"status": "processed", "wall_s": 0.05}] * 9 + [{"status": "skipped", "wall_s": 5.0},
                                                                 {"status": "processed", "wall_s": 0.31}]
    out = sb.cost_check(records)
    assert out["processed"] == 10 and abs(out["max_s"] - 0.31) < 1e-12 and out["over_0_3_s"] == 1
    assert out["ok"]  # recorded, not judged (L2 v2.5: V1' and V5 judge the deployment cost)
    assert abs(out["last10_mean_s"] - 0.31) < 1e-12


def test_what_moves_besides_truck_and_pallet_and_reaches_the_lidar_plane_fails_the_moving_check():
    rigid = {"rigid_bodies": [{"path": "/World/Forklift/body", "root": "/World/Forklift", "z_min_m": 0, "z_max_m": 2.0},
                              {"path": "/World/Pallet/p", "root": "/World/Pallet", "z_min_m": 0, "z_max_m": 0.15},
                              {"path": "/World/Factory/box", "root": "/World/Factory", "z_min_m": 0, "z_max_m": 0.9}],
             "time_varying": []}
    out = sb.moving_check(rigid, {})
    assert out["ok"] and out["others"] == 1 and out["pallet_z_max_m"] == 0.15
    rigid["rigid_bodies"][2]["z_max_m"] = 1.2  # a box as tall as the plane (a kinematic one counts the same)
    assert not sb.moving_check(rigid, {})["ok"]
    rigid["rigid_bodies"][2]["z_max_m"] = 0.9
    rigid["time_varying"] = [{"path": "/World/Lift", "root": "/World/Lift", "z_min_m": 0, "z_max_m": 0.1}]
    assert not sb.moving_check(rigid, {})["ok"]  # an animated prim, low at the default time
    rigid["time_varying"] = [{"path": "/World/Mesh", "root": "/World/Mesh", "z_min_m": None, "z_max_m": None}]
    assert not sb.moving_check(rigid, {})["ok"]  # geometry only in time samples: an empty default bound
    rigid["time_varying"] = [{"path": "/World/Forklift/mast", "root": "/World/Forklift", "z_min_m": 0, "z_max_m": 2}]
    assert sb.moving_check(rigid, {})["ok"]  # the truck's own
    rigid["rigid_bodies"][2]["z_max_m"] = None
    assert not sb.moving_check(rigid, {})["ok"]  # a body of unknown height
    rigid["rigid_bodies"][2]["z_max_m"] = 0.9
    rigid["time_varying"] = []
    assert not sb.moving_check(rigid, {"new_obstacles": "events.yaml"})["ok"]  # obstacles the runner spawns
    assert sb.moving_check(None, {})["ok"] is False  # not run is no pass


def test_every_processed_reply_must_have_reached_the_runner():
    bridge = [{"scan_id": 1, "status": "processed"}, {"scan_id": 2, "status": "skipped"},
              {"scan_id": 3, "status": "processed"}]
    runner = [{"scan_id": 1, "status": "processed"}, {"scan_id": 2, "status": "skipped"}]
    out = sb.received_check(bridge, runner)
    assert not out["ok"] and out["not_received"] == [3]  # the last reply lost to the closing socket
    assert sb.received_check(bridge[:2], runner)["ok"]


def test_every_input_file_the_arguments_name_is_hashed_but_not_the_runs_own_output(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "settings.yaml").write_text("a: 1\n")
    (tmp_path / "scene.usda").write_text("#usda 1.0\n")
    (tmp_path / "port.json").write_text("{}")
    out = sb.input_hashes({"settings": "config/settings.yaml", "base_scene": str(tmp_path / "scene.usda"),
                           "live_port_file": str(tmp_path / "port.json"), "seed": 0, "missing": "config/none.yaml",
                           "asset_root": "https://example.org/props"}, tmp_path)
    assert sorted(out) == ["base_scene", "settings"]
    assert out["settings"]["sha256"] == sb.sha256(tmp_path / "config" / "settings.yaml")


def test_the_graphs_scans_must_be_the_epochs_processed_scans():
    records = [{"scan_id": i, "stamp_s": i * 0.1, "status": s} for i, s in
               enumerate(["processed", "skipped", "processed", "processed"])]
    nodes = nodes_of([(0, 0, 0.0, 0, 0, 0, 0, 0, 0), (1, 1, 0.2, 0, 0, 0, 0, 0, 0), (2, 2, 0.3, 0, 0, 0, 0, 0, 0)])
    out = sb.nodes_check(nodes, records)
    assert out["ok"] and out["last_processed"] == {"scan_id": 3, "stamp_s": 0.30000000000000004}
    nodes["stamp_s"][1] = 0.1  # a skipped scan in the graph
    assert not sb.nodes_check(nodes, records)["ok"]


SCENE = {
    "scenario": {"seed": 0, "start_rear": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
                 "pickup": {"x_m": 1.8, "y_m": 0.0, "yaw_rad": 0.0}, "destination": {"x_m": 1.8, "y_m": 0.0, "yaw_rad": 0.0},
                 "props": [], "bounds": {"x_min_m": -3.0, "x_max_m": 3.0, "y_min_m": -3.0, "y_max_m": 3.0}},
    "geometry": {"unloaded_footprint": {"front_m": 1.29, "rear_m": 0.17, "half_width_m": 0.36}, "pallet_depth_m": 0.6,
                 "pallet_width_m": 0.8, "inserted_offset_m": 1.26, "prealign_offset_m": 3.79},
    "travel_planner_config": {"clearance_m": 0.1},
}


def _epoch(tmp_path, *, serialized=True, grid_matches=True, grid_resolution=0.05, lost_reply=False):
    epoch, readback = tmp_path / "epoch_1", tmp_path / "readback"
    bridge, run = epoch / "bridge", epoch / "run"
    (bridge / "bundle").mkdir(parents=True)
    for name in ("slam_log.joint", "slam_log.scan", "slam.records"):
        (run / "spool" / name).mkdir(parents=True)
    (readback / "dump").mkdir(parents=True)
    (bridge / "bundle" / "map.posegraph").write_bytes(b"graph")
    (bridge / "bundle" / "map.data").write_bytes(b"data")
    (bridge / "bridge_status.json").write_text(json.dumps({"ok": True, "failed": 0, "records_dropped": 0,
                                                           "records_unsettled": 0}))
    (bridge / "serialize.log").write_text("response:\n  result=0)\nserialize exit 0\n" if serialized else "")
    stamps = np.arange(0.0, 2.0, 1 / 60)
    base = np.array([[0.5 * t, 0.0, 0.1, *quat(0.0)] for t in stamps])
    np.savez(run / "spool" / "slam_log.joint" / "000000.npz", joint_stamps_s=stamps, base_pose_world=base)
    np.savez(run / "spool" / "slam_log.scan" / "000000.npz", scan_stamps_s=stamps[::6])
    scans = [0, 12, 30, 60, 90]  # every 0.2-0.5 s: the processed keyframes
    lines = [json.dumps({"scan_id": i, "stamp_s": float(stamps[i]), "status": "processed", "wall_s": 0.04})
             for i in scans]
    (bridge / "records.jsonl").write_text("\n".join(lines) + "\n")
    received = scans[:-1] if lost_reply else scans
    (run / "spool" / "slam.records" / "000000.jsonl").write_text(
        "".join(json.dumps({"scan_id": i, "status": "processed"}) + "\n" for i in received))
    (run / "spool" / "manifest.json").write_text(json.dumps({
        "complete": True, "slam_log.joint": {"items": len(stamps)}, "slam_log.scan": {"items": len(stamps[::6])},
        "slam.records": {"items": len(received)}}))
    (run / "meta.json").write_text(json.dumps({"spool": {"joint": "slam_log.joint", "scan": "slam_log.scan",
                                                         "silenced": "slam_log.silenced", "root": "spool"}}))
    (run / "exit_code").write_text(json.dumps({"code": 0, "reason": None}))
    site = {"x_m": 1.8, "y_m": 0.0, "yaw_rad": 0.0}  # its approach: rear axle x 0.54 .. -1.99
    (run / "result.json").write_text(json.dumps({"seed": 0, "scene_sha256": "s", "arguments": {
        "seed": 0, "slam_noise_seed": 0, "settings": str(tmp_path / "slam.yaml"), "rear_axle_offset_m": -0.34},
        "factory": {"layout_version": "v2", "layout_sha256": "f"}, **SCENE}))
    (epoch / "bridge_container_exit").write_text("0\n")
    with open(readback / "dump" / "nodes.csv", "w") as handle:
        handle.write("unique_id,state_id,stamp_s,x,y,yaw,odom_x,odom_y,odom_yaw\n")
        for k, i in enumerate(scans):
            t = stamps[i]
            handle.write(f"{k},{k},{float(t)!r},{float(0.5 * t + 0.01)!r},0.0,0.0,{float(0.5 * t)!r},0.0,0.0\n")
    cells = np.array([[0, 100], [255, 255]], dtype=np.uint8)
    cells.tofile(readback / "dump" / "grid.u8")
    (readback / "dump" / "graph.json").write_text(json.dumps({"scans": 5, "vertices": 5, "edges": 4, "width": 2,
                                                              "height": 2, "origin": [-1.0, -2.0],
                                                              "resolution_m": grid_resolution}))
    data = sb.karto_to_map(cells)
    if not grid_matches:
        data[0, 0] = 0
    np.savez(readback / "map.npz", data=data, origin=np.array([-1.0, -2.0, 0.0]), resolution_m=0.05)
    (readback / "map_grab.json").write_text(json.dumps({"settled": True, "maps_received": 3}))
    (readback / "slam_toolbox_version.txt").write_text("2.8.5-1noble\n")
    params = tmp_path / "slam.yaml"
    params.write_text("slam_toolbox: {}\n")
    (tmp_path / "rigid.json").write_text(json.dumps({"scene": str(run / "scene.usda"), "rigid_bodies": [],
                                                     "time_varying": []}))
    (tmp_path / "tour.yaml").write_text("aisles:\n  - [[0.0, 0.0], [0.5, 0.0]]\ntour: []\n")
    return epoch, readback, params


def _main(tmp_path, epoch, readback, params):
    report = tmp_path / "report.json"
    code = sb.main(["--epoch", str(epoch), "--readback", str(readback), "--slam-params", str(params),
                    "--image", "sha256:x", "--rigid-bodies", str(tmp_path / "rigid.json"), "--root", str(tmp_path),
                    "--tour-config", str(tmp_path / "tour.yaml"),
                    "--out", str(tmp_path / "bundle"), "--report", str(report)])
    return code, json.loads(report.read_text())


def test_a_clean_survey_writes_a_bundle_whose_manifest_rehashes(tmp_path):
    code, report = _main(tmp_path, *_epoch(tmp_path))
    assert code == 0 and report["ok"] and report["bundle_written"]
    bundle = tmp_path / "bundle"
    manifest = json.loads((bundle / "manifest.json").read_text())
    hashes = {name: sb.sha256(bundle / name) for name in sb.BUNDLE_FILES}
    assert manifest["files"] == hashes and manifest["map_id"] == sb.map_id(hashes) == report["map_id"]
    assert manifest["graph"] == {"scans": 5, "vertices": 5, "edges": 4}
    assert manifest["last_processed"]["scan_id"] == 90 and manifest["slam_toolbox_version"] == "2.8.5-1noble"
    with np.load(bundle / "grid.npz") as grid:
        assert grid["data"].tolist() == [[-1, 100], [0, 0]]
    assert abs(report["world"]["position_max_m"] - 0.01) < 1e-9 and report["coverage"]["ok"]
    assert report["coverage"]["judged"]["uncovered"] == 0
    assert manifest["inputs"]["settings"]["sha256"] == sb.sha256(tmp_path / "slam.yaml")
    assert manifest["noise"] == {"slam_noise_seed": 0} and manifest["factory"]["layout_sha256"] == "f"


def test_no_bundle_without_the_graph_written_or_when_the_readback_grid_is_not_the_graphs(tmp_path):
    code, report = _main(tmp_path / "a", *_epoch(tmp_path / "a", serialized=False))
    assert code == 1 and not report["bundle_written"] and not (tmp_path / "a" / "bundle").exists()
    code, report = _main(tmp_path / "b", *_epoch(tmp_path / "b", grid_matches=False))
    assert code == 1 and report["finalize"]["readback_grid_is_the_graphs"] is False and not report["bundle_written"]
    code, report = _main(tmp_path / "c", *_epoch(tmp_path / "c", grid_resolution=0.5))  # same cells, other scale
    assert code == 1 and report["finalize"]["readback_grid_is_the_graphs"] is False
    code, report = _main(tmp_path / "d", *_epoch(tmp_path / "d", lost_reply=True))
    assert code == 1 and not report["received"]["ok"] and not report["bundle_written"]


def test_a_bridge_container_that_did_not_end_cleanly_writes_no_bundle(tmp_path):
    epoch, readback, params = _epoch(tmp_path)
    (epoch / "bridge_container_exit").write_text("1\n")
    code, report = _main(tmp_path, epoch, readback, params)
    assert code == 1 and not report["finalize"]["ok"] and not report["bundle_written"]


def test_an_incomplete_spool_fails_the_truth_checks(tmp_path):
    epoch, readback, params = _epoch(tmp_path)
    manifest = epoch / "run" / "spool" / "manifest.json"
    manifest.write_text(json.dumps({**json.loads(manifest.read_text()), "complete": False}))
    code, report = _main(tmp_path, epoch, readback, params)
    assert code == 1 and not report["world"]["ok"] and not report["start"]["ok"] and not report["received"]["ok"]


def _heredoc(text, name):
    start = text.index(f'cat > "$OUT/{name}" <<\'EOT\'')
    return text[start:text.index("\nEOT\n", start)]


def test_the_survey_job_runs_the_consoles_runner_word_for_word():
    live = (ROOT / "deploy" / "console" / "isaac_live.sbatch").read_text()
    survey = (ROOT / "deploy" / "console" / "slam_survey.sbatch").read_text()
    assert _heredoc(live, "runner.sh") == _heredoc(survey, "runner.sh")
    bridge = _heredoc(survey, "bridge.sh")
    assert "-e SLAM_SERIALIZE=/out/bundle/map" in bridge
    assert '/run/slam/slam.sock /out "-0.12,0.0,1.05,0.0" persistent' in bridge
    assert "--max-epochs 1" in survey and "--end-session" in survey


def test_the_bridge_container_writes_the_graph_after_the_bridge_and_before_slam_toolbox_stops():
    script = (ROOT / "tools" / "slam_bridge_container.sh").read_text()
    bridge = script.index("python3 -m forklift_ros.isaac_slam_bridge")
    serialize = script.index("/slam_toolbox/serialize_map")
    assert bridge < serialize < script.index("kill $SLAM")
    assert 'if [ -n "${SLAM_SERIALIZE:-}" ]' in script


def test_a_far_aisle_leaves_the_judged_region_uncovered(tmp_path):
    epoch, readback, params = _epoch(tmp_path)
    (tmp_path / "tour.yaml").write_text("aisles:\n  - [[0.0, 0.0], [0.5, 0.0]]\n  - [[0.0, 2.5], [2.5, 2.5]]\ntour: []\n")
    code, report = _main(tmp_path, epoch, readback, params)
    assert code == 1 and not report["coverage"]["ok"] and report["coverage"]["judged"]["uncovered"] > 0
    assert report["bundle_written"]  # coverage judges the survey, it does not stop the bundle


def test_the_judged_region_is_the_bands_approaches_and_home_away_from_walls_and_props():
    result = {**SCENE, "scenario": {**SCENE["scenario"], "props": [
        {"rectangle": {"x_m": 0.0, "y_m": 1.0, "length_m": 0.4, "width_m": 0.4, "yaw_rad": 0.0}}]}}
    cells = st.judged_region(result, [((-2.0, -2.0), (2.0, -2.0))], [(2.0, 2.0)])
    d = lambda x, y: np.min(np.hypot(cells[:, 0] - x, cells[:, 1] - y))  # noqa: E731
    assert d(0.0, -2.0) < 0.2 and d(2.0, 2.0) < 0.2 and d(-1.0, 0.0) < 0.2  # band, observation disc, approach
    assert d(0.0, 1.0) > 0.3  # under the prop (0.27 from it at least)
    assert cells[:, 1].min() >= -3.0 + 0.27 - 1e-9  # the rear axle cannot be nearer the wall than rear + clearance
    assert d(-2.5, 2.5) > 1.0  # nothing judged there


def test_a_base_pose_moves_back_to_its_rear_axle():
    x, y = st.base_to_rear(np.array([1.0]), np.array([2.0]), np.array([np.pi / 2]), -0.34)
    assert abs(x[0] - 1.0) < 1e-12 and abs(y[0] - 1.66) < 1e-12


def test_the_tour_records_the_scene_it_was_planned_on():
    a = st.scenario_sha256(SCENE)
    assert a == st.scenario_sha256(json.loads(json.dumps(SCENE)))
    other = {**SCENE, "scenario": {**SCENE["scenario"], "seed": 1}}
    assert a != st.scenario_sha256(other)


def _straight_with_cusp():
    """2 m forward along x, then 1 m back: one gear cusp."""
    fwd = np.column_stack((np.linspace(0, 2, 41), np.zeros(41), np.zeros(41)))
    back = np.column_stack((np.linspace(2, 1, 21)[1:], np.zeros(20), np.zeros(20)))
    poses = np.vstack((fwd, back))
    directions = np.r_[np.ones(41), -np.ones(20)].astype(np.int8)
    return poses, directions, np.zeros(len(poses))


def _state(seq, mode="TELEOP", x=0.0, speed=0.0, pacing_ok=True, forbidden=None):
    return {"state_seq": seq, "mode": mode, "armed": True, "rear_axle_speed_mps": speed,
            "pose": {"x_m": x, "y_m": 0.0, "yaw_rad": 0.0}, "pacing": {"target_factor": 0.3, "ok": pacing_ok},
            "teleop_forbidden_reason": forbidden}


def test_a_tour_with_a_cusp_completes_through_the_teleop_slews():
    out = st.simulate_tour(*_straight_with_cusp(), max_sim_s=200.0)
    assert out["end"] == "arrived" and out["progress_m"] > 2.8


def test_the_state_from_before_teleop_is_not_a_departure():
    driver = st.TourDriver(*_straight_with_cusp())
    driver.start(0.0)
    assert driver.step(0.05, _state(1, mode="PAUSED")) == (None, None)  # Codex v2.5 P2-4: not left_teleop
    command, end = driver.step(0.15, _state(2))
    assert end is None and driver.departed and command[0] == 0.0  # the first leg's start: its command judged next
    command, end = driver.step(0.25, _state(3))
    assert end is None and command[0] > 0
    driver2 = st.TourDriver(*_straight_with_cusp(), enter_s=1.0)
    driver2.start(0.0)
    assert driver2.step(1.5, _state(1, mode="PAUSED"))[1] == "teleop_not_entered"


def test_pacing_time_does_not_spend_the_drivable_stall_budget_and_silence_is_a_failure():
    driver = st.TourDriver(*_straight_with_cusp(), stall_s=5.0, pacing_wait_s=100.0, stale_s=2.0)
    driver.start(0.0)
    seq, t = 0, 0.0
    for _ in range(700):  # 70 s with teleop forbidden for pacing, standing
        seq, t = seq + 1, t + 0.1
        assert driver.step(t, _state(seq, pacing_ok=False, forbidden="pacing"))[1] is None
    for _ in range(20):  # recovered: 2 s of drivable standing is within the 5 s budget (Codex v2.5 P2-5)
        seq, t = seq + 1, t + 0.1
        assert driver.step(t, _state(seq))[1] is None
    assert driver.step(t + 2.5, _state(seq))[1] == "state_stale"  # the same state for 2.5 s


def test_the_known_fraction_reads_the_grid_at_each_cell():
    grid = {"data": np.array([[-1, 0], [100, -1]], dtype=np.int8), "origin": np.array([0.0, 0.0, 0.0]),
            "resolution_m": 1.0}
    cells = np.array([[0.5, 0.5], [1.5, 0.5], [0.5, 1.5], [1.5, 1.5], [5.0, 5.0]])
    assert abs(sb.known_fraction(cells, grid) - 0.4) < 1e-12  # row 0 is y in [0, 1): (1.5, 0.5) free, (0.5, 1.5) occupied


def test_states_without_pose_or_speed_are_no_feedback():
    driver = st.TourDriver(*_straight_with_cusp(), stale_s=2.0)
    driver.start(0.0)
    assert driver.step(0.1, _state(1))[1] is None  # departed
    bare = {"state_seq": 2, "mode": "TELEOP", "armed": True, "rear_axle_speed_mps": None, "pose": None}
    assert driver.step(1.0, bare)[1] is None
    assert driver.step(2.5, {**bare, "state_seq": 3})[1] == "feedback_missing"


def test_a_tour_that_did_not_arrive_fails_the_survey(tmp_path):
    epoch, readback, params = _epoch(tmp_path)
    np.savez(tmp_path / "tour.npz", poses=np.zeros((2, 3)), scenario_sha256=np.asarray(st.scenario_sha256(SCENE)))
    for end, ok in (("stalled", False), ("arrived", True)):
        (tmp_path / "probe.json").write_text(json.dumps({"tour": {"end": end, "progress_m": 2.8, "events": []}}))
        report = tmp_path / f"report_{end}.json"
        sb.main(["--epoch", str(epoch), "--readback", str(readback), "--slam-params", str(params), "--image", "x",
                 "--rigid-bodies", str(tmp_path / "rigid.json"), "--root", str(tmp_path), "--tour-config",
                 str(tmp_path / "tour.yaml"), "--tour-path", str(tmp_path / "tour.npz"), "--probe",
                 str(tmp_path / "probe.json"), "--out", str(tmp_path / f"bundle_{end}"), "--report", str(report)])
        out = json.loads(report.read_text())
        assert out["tour"]["ok"] is ok and out["ok"] is ok and out["coverage"]["tour_scene_matches"]


def test_the_tour_plans_around_the_permits_prior_pickup_zone():
    """ws1 job 6807 stood on pallet:P-1: an unrecognised pallet is the whole prior zone to the teleop permit."""
    z = st.permit_pickup_zone(SCENE)
    # BAY_PICKUP_ZONE x 2.6-3.6, y -0.2-1.5, grown by the pallet's half diagonal 0.5 + 0.182 + 0.01 + 0.05
    assert abs((z.x_m - z.length_m / 2) - (2.6 - 0.742)) < 1e-9 and abs((z.y_m + z.width_m / 2) - (1.5 + 0.742)) < 1e-9


def test_a_leg_starts_by_steering_standing():
    """ws1 job 6826: moving while the teleop curvature still slewed left the truck off its leg."""
    poses = np.column_stack((np.linspace(0, 2, 41), np.zeros(41), np.zeros(41)))
    driver = st.TourDriver(poses, np.ones(41, dtype=np.int8), np.zeros(41))
    driver.start(0.0)
    # the steering left hard over from the last leg: the straight leg's ~0 curvature is asked standing
    driver.step(0.05, {**_state(0), "applied": {"curvature_inv_m": -0.3}, "steering_rad": [-0.2, -0.2]})  # leg starts
    command, end = driver.step(0.1, {**_state(1), "applied": {"curvature_inv_m": -0.3}, "steering_rad": [-0.2, -0.2]})
    assert end is None and command[0] == 0.0 and abs(command[1]) < 0.1
    state = {**_state(2), "applied": {"curvature_inv_m": 0.0}, "steering_rad": [-0.05, -0.05]}
    assert driver.step(0.2, state)[0][0] == 0.0  # the joints still turning
    state = {**_state(3), "applied": {"curvature_inv_m": 0.0}, "steering_rad": [-0.05, -0.05]}
    assert driver.step(0.3, state)[0][0] > 0.0  # settled: drive


def test_a_slow_steering_plant_still_arrives():
    out = st.simulate_tour(*_straight_with_cusp(), max_sim_s=300.0, plant_curvature_rate=0.25)
    assert out["end"] == "arrived"


def _drive_to(driver, poses_xy, t0=0.0, seq0=0, speed=0.2):
    """Feed the driver states along ``poses_xy`` (on the path, heading 0), then return (t, seq)."""
    t, seq = t0, seq0
    for x, y in poses_xy:
        seq, t = seq + 1, t + 0.1
        driver.step(t, {**_state(seq, speed=speed), "pose": {"x_m": x, "y_m": y, "yaw_rad": 0.0}})
    return t, seq


def test_standing_short_of_a_leg_end_outside_its_tolerance_replans():
    """ws1 job 6828: 0.24 m beside a cusp waypoint the braking profile reached zero and the tour stood 60 s."""
    poses, directions, curv = _straight_with_cusp()
    driver = st.TourDriver(poses, directions, curv, waypoints=[[2.0, 0.0, 0.0]], release_s=3.0)
    driver.start(0.0)
    t, seq = _drive_to(driver, [(x, 0.0) for x in np.linspace(0.0, 1.85, 40)] + [(1.85 + 0.024 * k, 0.04 * k)
                                                                                  for k in range(1, 6)])
    request = None
    while request is None and t < 30.0:
        seq, t = seq + 1, t + 0.1
        # short of the cusp (2.0) by more than its release window, as 6828 stood
        driver.step(t, {**_state(seq), "pose": {"x_m": 1.97, "y_m": 0.24, "yaw_rad": 0.0}, "rear_axle_speed_mps": 0.0})
        request = driver.replan_request
    events = [e["event"] for e in driver.events]
    assert request is not None and "stuck" in events and "tracker_failed" not in events
    assert request[1] == []  # the cusp waypoint beside (0.24 m) counts as passed


def test_standing_short_of_the_last_goal_beyond_the_slack_replans():
    """Codex v2.5 round 4 P2-1: the last goal 0.4 m away (beyond the 0.3 m slack) must not stall the tour."""
    poses = np.column_stack((np.linspace(0, 2, 41), np.zeros(41), np.zeros(41)))
    driver = st.TourDriver(poses, np.ones(41, dtype=np.int8), np.zeros(41), waypoints=[], release_s=3.0)
    driver.start(0.0)
    t, seq = _drive_to(driver, [(x, 0.0) for x in np.linspace(0.0, 1.9, 40)] + [(1.9 + 0.02 * k, 0.02 * k)
                                                                                 for k in range(1, 6)])
    request = None
    while request is None and t < 30.0:
        seq, t = seq + 1, t + 0.1
        _, end = driver.step(t, {**_state(seq), "pose": {"x_m": 2.0, "y_m": 0.4, "yaw_rad": 0.0},
                                 "rear_axle_speed_mps": 0.0})
        assert end is None
        request = driver.replan_request
    assert request is not None and "stuck" in [e["event"] for e in driver.events]


def test_planning_time_is_neither_idle_nor_stale_and_a_truck_that_never_stands_fails():
    poses, directions, curv = _straight_with_cusp()
    driver = st.TourDriver(poses, directions, curv, waypoints=[], stall_s=5.0, stale_s=3.0, stop_wait_s=2.0)
    driver.start(0.0)
    driver.step(0.1, _state(1))
    plan = st.SimpleNamespace(success=True, poses=poses, directions=directions, curvatures_inv_m=curv, length_m=3.0)
    driver.adopt(70.0, plan, [])  # 70 s of planning (Codex v2.5 round 3 P2-2)
    assert driver.step(70.1, _state(2))[1] is None and driver.drivable_idle < 1.0
    driver._replan_wait = []  # a stop asked for, the truck still creeping (P2-3)
    t = 70.1
    for seq in range(3, 40):
        t += 0.1
        _, end = driver.step(t, _state(seq, speed=0.05))
        if end:
            break
    assert end == "stop_timeout"


def test_a_listed_scene_from_another_run_must_share_this_stage(tmp_path):
    result = {**SCENE, "scene_sha256": {"scene.usda": "a", "animated.usda": "z"}, "forklift_urdf_sha256": "u",
              "pallet_urdf_sha256": "p", "arguments": {"base_scene": "/base/scene.usda"},
              "script_sha256": "s", "prism_colliders": {"columns": 37}, "low_obstacles_removed": {"props_kept": 37},
              "source_sha256": {"sim/isaac/scene.py": "b"}, "factory": {"layout_sha256": "l"}}
    run, ref = tmp_path / "run", tmp_path / "ref"
    run.mkdir(); ref.mkdir()
    (ref / "result.json").write_text(json.dumps(result))
    rigid = {"scene": str(ref / "scene.usda"), "rigid_bodies": []}
    assert sb.rigid_provenance(rigid, run, result, ref)["ok"]
    other = {**result, "source_sha256": {"sim/isaac/scene.py": "c"}}
    (ref / "result.json").write_text(json.dumps(other))
    out = sb.rigid_provenance(rigid, run, result, ref)
    assert not out["ok"] and out["differ"] == ["scene_builder_sha256"]
    assert not sb.rigid_provenance({**rigid, "scene": "/elsewhere/scene.usda"}, run, result, ref)["ok"]
    # the same directory's other file opened (Codex v2.5 round 4 P2-2)
    (ref / "result.json").write_text(json.dumps({**result, "arguments": {"base_scene": "/base/animated.usda"}}))
    out = sb.rigid_provenance(rigid, run, result, ref)
    assert not out["ok"] and "base_scene_file" in out["differ"]


def test_no_progress_replans_before_it_fails_the_tour():
    """ws1 job 6831: the pallet permit held the truck mid-leg; a new path from there before the 60 s stall."""
    poses, directions, curv = _straight_with_cusp()
    driver = st.TourDriver(poses, directions, curv, waypoints=[[2.0, 0.0, 0.0]], replan_stall_s=5.0, stall_s=60.0)
    driver.start(0.0)
    t, seq = _drive_to(driver, [(x, 0.0) for x in np.linspace(0.0, 0.5, 10)])
    while driver.replan_request is None and t < 30.0:
        seq, t = seq + 1, t + 0.1
        _, end = driver.step(t, {**_state(seq, speed=0.0), "pose": {"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0},
                                 "permit": {"limit_mps": 0.0, "reason": "pallet:P-1"}})
        assert end is None
    assert driver.replan_request is not None and "stall_replan" in [e["event"] for e in driver.events]


def test_a_stand_the_pacing_forces_spends_no_replan():
    """Codex v2.5 round 5 P2: 0.4 m before a leg end, teleop forbidden for pacing, the truck standing."""
    poses = np.column_stack((np.linspace(0, 2, 41), np.zeros(41), np.zeros(41)))
    driver = st.TourDriver(poses, np.ones(41, dtype=np.int8), np.zeros(41), waypoints=[], pacing_wait_s=900.0)
    driver.start(0.0)
    t, seq = _drive_to(driver, [(x, 0.0) for x in np.linspace(0.0, 1.6, 30)])
    for _ in range(700):  # 70 s
        seq, t = seq + 1, t + 0.1
        _, end = driver.step(t, {**_state(seq, x=1.6, speed=0.0, pacing_ok=False, forbidden="pacing")})
        assert end is None
    assert driver.replans == 0 and driver.replan_request is None


def test_the_tour_keeps_its_clearance_and_a_replan_never_drops_below_the_planners():
    """r6 (ws1 job 6837): at 0.10 m, then 0, every replan planned the leg the permit held; now 0.30 m, the run's 0.10 m
    once for a start that cannot keep 0.30, and nothing below."""
    scene = {**SCENE, "scenario": {**SCENE["scenario"], "props": [
        {"rectangle": {"x_m": 0.0, "y_m": 1.5, "length_m": 1.0, "width_m": 1.0, "yaw_rad": 0.0}}]}}
    tour = [[-1.0, -1.5, 0.0]]
    assert st.plan_tour(scene, tour).clearance_m == st.SURVEY_CLEARANCE_M
    near = st.plan_tour(scene, tour, start=(0.0, 0.45, 0.0))  # the truck's side 0.19 m from the prop
    assert near.success and near.clearance_m == 0.1
    assert not st.plan_tour(scene, tour, start=(0.0, 0.6, 0.0)).success  # 0.04 m: no clearance-0 retry


def test_each_permit_block_is_recorded_once_with_the_pose_it_held():
    poses = np.column_stack((np.linspace(0, 2, 41), np.zeros(41), np.zeros(41)))
    driver = st.TourDriver(poses, np.ones(41, dtype=np.int8), np.zeros(41), waypoints=[])
    driver.start(0.0)
    t, seq = _drive_to(driver, [(x, 0.0) for x in np.linspace(0.0, 0.5, 10)])
    for limit in (0.0, 0.0, 0.0, 0.5, 0.0):
        seq, t = seq + 1, t + 0.1
        driver.step(t, {**_state(seq, x=0.5, speed=0.0), "permit": {"limit_mps": limit, "reason": "occupied"}})
    blocks = [e for e in driver.events if e["event"] == "permit_blocked"]
    assert len(blocks) == 2 and blocks[0]["pose"]["x_m"] == 0.5 and blocks[0]["permit"]["reason"] == "occupied"
