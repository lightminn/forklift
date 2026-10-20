"""The SLAM map bundle of a survey epoch and the survey's checks (SLAM localization plan
docs/plans/2026-10-20-slam-localization-session.md L2, S2, V2').

deploy/console/slam_survey.sbatch runs one epoch of the live runner in mapping mode, has slam_toolbox write its pose
graph once the runner closed its socket, and reads the graph back in a fresh container (tools/slam_bundle_container.sh:
the graph's nodes and karto grid by tools/slam_graph_dump.cpp, the /map slam_toolbox publishes from it with no scan
sent). This writes, from those:

- ``<out>/``: ``map.posegraph``, ``map.data`` (copied), ``grid.npz`` (the read-back /map: ``data`` int8 -1/0/100,
  ``origin`` [x, y, yaw], ``resolution_m``) and ``manifest.json`` (``map_id`` = SHA-256 over the three files' names
  and SHA-256, the run's scene/URDF/source hashes and arguments, seeds, slam_toolbox version, node and edge counts,
  the last processed scan, the survey's stamps, every input file the run's arguments name, hashed). No bundle is
  written unless the finalize, received and nodes checks pass.
- ``--report``: the checks, each with its numbers and ``ok``:
  ``finalize`` (the session ended, the bridge container and its verdict clean, the graph written, the read-back /map
  settled and the same as the karto grid of the dumped graph, resolution and origin too), ``received`` (every scan the
  bridge answered processed reached the runner), ``nodes`` (the graph's scans are the epoch's processed scans, by
  stamp), ``world`` (V2': every node's corrected base pose against the Isaac truth at its stamp, unaligned),
  ``start`` (map <- world is the identity: the first node's odometric pose is the truth), ``coverage`` (every path
  point of the reference run within the radius of a node), ``cost`` (the survey's keyframe processing seconds),
  ``moving`` (what can move and reach the LiDAR plane besides the truck and the carried pallet; not run is a fail).

    python tools/slam_survey_bundle.py --epoch OUT/epoch_1 --readback OUT/readback --slam-params YAML \\
        --image IMAGE_ID [--reference-run RUN] [--rigid-bodies RIGID.json] --out OUT/bundle --report REPORT.json
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from forklift_core.live.spool import load_list, load_slam_log  # noqa: E402

# Provisional (plan section 5: Claude's proposals, to be changed only in the plan's table with a reason).
WORLD_POSITION_MAX_M = 0.10
WORLD_YAW_MAX_RAD = 0.03
COVERAGE_RADIUS_M = 3.0
SURVEY_KEYFRAME_MAX_S = 0.3
START_POSITION_MAX_M = 0.01
START_YAW_MAX_RAD = 0.005
LIDAR_PLANE_M = 1.05  # the laser's height above base_link (laser_xyz_yaw), the base on the floor
STAMP_TOLERANCE_S = 1e-6
BUNDLE_FILES = ("map.posegraph", "map.data", "grid.npz")
# run arguments that name a file but are this run's own output or socket, not an input to hash
NOT_INPUTS = ("output", "live_port_file", "slam_feedback", "slam_map_dir", "live_web_root")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def map_id(hashes: dict) -> str:
    """The bundle's identity: SHA-256 over ``name sha256`` lines of its three files, in name order."""
    text = "".join(f"{name} {hashes[name]}\n" for name in sorted(hashes))
    return hashlib.sha256(text.encode()).hexdigest()


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def yaw_of(quaternion_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion_wxyz, float).T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rows_at(stamps: np.ndarray, table: np.ndarray, tolerance_s: float) -> np.ndarray:
    """Index in ``table`` of each stamp (nearest, within ``tolerance_s``), else -1."""
    order = np.argsort(table)
    ordered = table[order]
    right = np.clip(np.searchsorted(ordered, stamps), 0, len(ordered) - 1)
    left = np.clip(right - 1, 0, len(ordered) - 1)
    nearest = np.where(np.abs(ordered[left] - stamps) < np.abs(ordered[right] - stamps), left, right)
    return np.where(np.abs(ordered[nearest] - stamps) <= tolerance_s, order[nearest], -1)


def read_nodes(path: Path) -> dict:
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    return {key: np.asarray([float(r[key]) for r in rows]) for key in
            ("unique_id", "state_id", "stamp_s", "x", "y", "yaw", "odom_x", "odom_y", "odom_yaw")}


def read_records(bridge: Path) -> list:
    """The bridge's reply records: ``records.jsonl`` (a persistent bridge) or ``bridge_records.json``."""
    if (bridge / "records.jsonl").exists():
        return [json.loads(line) for line in (bridge / "records.jsonl").read_text().splitlines() if line]
    return json.loads((bridge / "bridge_records.json").read_text())


def karto_to_map(cells: np.ndarray) -> np.ndarray:
    """karto GridStates (0 unknown, 100 occupied, 255 free) as /map values (slam_toolbox toNavMap)."""
    out = np.full(cells.shape, -1, dtype=np.int8)
    out[cells == 100] = 100
    out[cells == 255] = 0
    return out


def world_check(nodes: dict, joint_stamps: np.ndarray, base_pose_world: np.ndarray, tolerance_s: float) -> dict:
    """V2': each node's corrected base pose against the truth at its stamp, no alignment."""
    rows = rows_at(nodes["stamp_s"], joint_stamps, tolerance_s)
    paired = rows >= 0
    truth = base_pose_world[rows[paired]]
    dx = nodes["x"][paired] - truth[:, 0]
    dy = nodes["y"][paired] - truth[:, 1]
    position = np.hypot(dx, dy)
    yaw = np.abs(wrap(nodes["yaw"][paired] - yaw_of(truth[:, 3:7])))
    out = {"nodes": int(len(rows)), "paired": int(paired.sum()),
           "position_max_m": float(position.max()) if len(position) else None,
           "position_rmse_m": float(np.sqrt(np.mean(position**2))) if len(position) else None,
           "yaw_max_rad": float(yaw.max()) if len(yaw) else None,
           "worst_node_stamp_s": float(nodes["stamp_s"][paired][int(np.argmax(position))]) if len(position) else None,
           "limits": {"position_max_m": WORLD_POSITION_MAX_M, "yaw_max_rad": WORLD_YAW_MAX_RAD}}
    out["ok"] = bool(len(rows) and paired.all() and out["position_max_m"] <= WORLD_POSITION_MAX_M
                     and out["yaw_max_rad"] <= WORLD_YAW_MAX_RAD)
    return out


def start_check(nodes: dict, joint_stamps: np.ndarray, base_pose_world: np.ndarray, tolerance_s: float) -> dict:
    """map <- world is the identity: the first node's odometric pose (the runner's odometry starts at the world
    start, run_transport slam_start) is the truth there."""
    first = int(np.argmin(nodes["stamp_s"]))
    row = int(rows_at(nodes["stamp_s"][first:first + 1], joint_stamps, tolerance_s)[0])
    if row < 0:
        return {"ok": False, "reason": "no truth at the first node's stamp"}
    truth = base_pose_world[row]
    position = math.hypot(nodes["odom_x"][first] - truth[0], nodes["odom_y"][first] - truth[1])
    yaw = abs(float(wrap(nodes["odom_yaw"][first] - yaw_of(truth[None, 3:7])[0])))
    return {"ok": position <= START_POSITION_MAX_M and yaw <= START_YAW_MAX_RAD, "stamp_s": float(nodes["stamp_s"][first]),
            "odom_position_error_m": position, "odom_yaw_error_rad": yaw,
            "limits": {"position_m": START_POSITION_MAX_M, "yaw_rad": START_YAW_MAX_RAD}}


def coverage_check(path_xy: np.ndarray, node_xy: np.ndarray, radius_m: float, chunk: int = 4096) -> dict:
    """Every path point within ``radius_m`` of a node (the LiDAR sees all round, so no heading condition)."""
    nearest = np.empty(len(path_xy))
    for start in range(0, len(path_xy), chunk):
        part = path_xy[start:start + chunk]
        d2 = ((part[:, None, :] - node_xy[None, :, :]) ** 2).sum(axis=2)
        nearest[start:start + chunk] = np.sqrt(d2.min(axis=1))
    worst = int(np.argmax(nearest)) if len(nearest) else None
    return {"ok": bool(len(nearest) and nearest.max() <= radius_m), "points": int(len(nearest)),
            "farthest_m": float(nearest.max()) if len(nearest) else None,
            "farthest_point": path_xy[worst].tolist() if worst is not None else None,
            "outside": int((nearest > radius_m).sum()), "radius_m": radius_m}


def path_points(run: Path) -> np.ndarray:
    """Every physics step's base position of a run (Isaac truth; a spooled log is refused when incomplete)."""
    return np.asarray(load_slam_log(run)["base_pose_world"], float)[:, :2]


def cost_check(records: list) -> dict:
    wall = np.asarray([r["wall_s"] for r in records if r.get("status") == "processed"], float)
    if not len(wall):
        return {"ok": False, "processed": 0}
    tail = wall[-max(1, len(wall) // 10):]
    return {"ok": bool(wall.max() <= SURVEY_KEYFRAME_MAX_S), "processed": int(len(wall)),
            "mean_s": float(wall.mean()), "p99_s": float(np.percentile(wall, 99)), "max_s": float(wall.max()),
            "last10_mean_s": float(tail.mean()), "limit_max_s": SURVEY_KEYFRAME_MAX_S}


def moving_check(rigid: dict | None, arguments: dict) -> dict:
    """What can move in the survey's stage and reach the LiDAR plane, other than the truck and the carried pallet:
    rigid bodies (kinematic ones too: a script can drive them) and prims whose transform has time samples
    (tools/list_rigid_bodies.py), and obstacles the runner spawns (--new-obstacles). The runner's own scripted moves
    are the truck, the pallet, the cameras and the path and destination markers, none a collider. Not run (no list)
    is no pass."""
    if rigid is None:
        return {"ok": False, "reason": "no rigid body list (tools/list_rigid_bodies.py did not run or failed)"}
    allowed = ("/World/Forklift", "/World/Pallet")
    movers = [r for r in rigid["rigid_bodies"] + rigid.get("time_varying", []) if r["root"] not in allowed]
    reaching = [r for r in movers if r["z_max_m"] is not None and r["z_max_m"] >= LIDAR_PLANE_M]  # no bound: no geometry
    spawned = arguments.get("new_obstacles")
    return {"ok": not reaching and not spawned, "roots": sorted({r["root"] for r in rigid["rigid_bodies"]}),
            "others": len(movers), "reaching_lidar_plane": reaching, "new_obstacles": spawned,
            "pallet_z_max_m": max((r["z_max_m"] for r in rigid["rigid_bodies"]
                                   if r["root"] == "/World/Pallet" and r["z_max_m"] is not None), default=None)}


def received_check(records: list, runner_records: list) -> dict:
    """Every scan the bridge answered processed reached the runner (L2's finalize order: the replies all received,
    nothing in flight when the graph is written). A reply lost to the runner's closing socket is still in the graph,
    but then the order was not kept."""
    bridge = {r["scan_id"] for r in records if r.get("status") == "processed"}
    runner = {r["scan_id"] for r in runner_records if r.get("status") == "processed"}
    missing = sorted(bridge - runner)
    return {"ok": not missing, "bridge_processed": len(bridge), "runner_processed": len(runner),
            "not_received": missing[:20], "not_received_count": len(missing)}


def input_hashes(arguments: dict, root: Path) -> dict:
    """SHA-256 of every input file the run's arguments name (relative paths from the snapshot root)."""
    out = {}
    for key, value in sorted(arguments.items()):
        if key in NOT_INPUTS or not isinstance(value, str) or "://" in value:
            continue
        path = Path(value) if Path(value).is_absolute() else root / value
        if path.is_file():
            out[key] = {"path": value, "sha256": sha256(path)}
    return out


def finalize_check(epoch: Path, readback: Path) -> tuple[dict, np.ndarray | None]:
    bridge = epoch / "bridge"
    out: dict = {}
    exit_file = epoch / "run" / "exit_code"
    out["runner_exit"] = json.loads(exit_file.read_text()) if exit_file.exists() else None
    container = epoch / "bridge_container_exit"
    out["bridge_container_exit"] = container.read_text().strip() if container.exists() else None
    status = json.loads((bridge / "bridge_status.json").read_text()) if (bridge / "bridge_status.json").exists() else None
    out["bridge_status"] = status
    serialize = (bridge / "serialize.log").read_text() if (bridge / "serialize.log").exists() else ""
    out["serialized"] = "result=0" in serialize and "serialize exit 0" in serialize
    files = {name: bridge / "bundle" / name for name in ("map.posegraph", "map.data")}
    out["graph_files"] = {name: path.stat().st_size if path.exists() else None for name, path in files.items()}
    grab = json.loads((readback / "map_grab.json").read_text()) if (readback / "map_grab.json").exists() else None
    out["map_grab"] = grab
    graph = json.loads((readback / "dump" / "graph.json").read_text()) if (readback / "dump" / "graph.json").exists() else None
    out["graph"] = graph
    grid = None
    same = None
    if grab and grab.get("settled") and (readback / "map.npz").exists() and graph and "width" in graph:
        with np.load(readback / "map.npz") as m:
            grid = {"data": m["data"].copy(), "origin": m["origin"].copy(), "resolution_m": float(m["resolution_m"])}
        cells = np.fromfile(readback / "dump" / "grid.u8", dtype=np.uint8).reshape(graph["height"], graph["width"])
        same = bool(cells.shape == grid["data"].shape and np.array_equal(karto_to_map(cells), grid["data"])
                    and np.allclose(graph["origin"], grid["origin"][:2], rtol=0.0, atol=1e-9)
                    and abs(float(grid["origin"][2])) <= 1e-12
                    and abs(float(graph["resolution_m"]) - grid["resolution_m"]) <= 1e-9)
    out["readback_grid_is_the_graphs"] = same
    out["ok"] = bool(
        out["runner_exit"] is not None and out["runner_exit"].get("code") == 0
        and out["bridge_container_exit"] == "0"
        and status is not None and status.get("ok") and not status.get("pending")
        and not status.get("records_dropped") and not status.get("records_unsettled")
        and out["serialized"] and all(out["graph_files"].values()) and same)
    return out, grid


def nodes_check(nodes: dict, records: list) -> dict:
    processed = [r for r in records if r.get("status") == "processed"]
    stamps = np.asarray([r["stamp_s"] for r in processed], float)
    rows = rows_at(nodes["stamp_s"], stamps, STAMP_TOLERANCE_S) if len(stamps) else np.full(len(nodes["stamp_s"]), -1)
    last = processed[-1] if processed else None
    return {"ok": bool(len(nodes["stamp_s"]) == len(processed) and (rows >= 0).all()
                       and len(set(rows.tolist())) == len(rows)),
            "graph_scans": int(len(nodes["stamp_s"])), "processed": len(processed),
            "graph_scans_not_processed": int((rows < 0).sum()),
            "last_processed": {"scan_id": last["scan_id"], "stamp_s": last["stamp_s"]} if last else None,
            "first_stamp_s": float(stamps.min()) if len(stamps) else None,
            "last_stamp_s": float(stamps.max()) if len(stamps) else None}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--epoch", type=Path, required=True)
    ap.add_argument("--readback", type=Path, required=True)
    ap.add_argument("--slam-params", type=Path, required=True, help="the mapping params the survey ran with")
    ap.add_argument("--image", default=None, help="the ros2-dev image id the survey ran in")
    ap.add_argument("--reference-run", type=Path, default=None,
                    help="a run directory of operational missions (its path is checked for coverage); the survey's "
                         "own path otherwise")
    ap.add_argument("--rigid-bodies", type=Path, default=None, help="tools/list_rigid_bodies.py's output")
    ap.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1],
                    help="the snapshot the run ran from (its relative argument paths)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--report", type=Path, required=True)
    args = ap.parse_args(argv)

    report: dict = {"epoch": str(args.epoch), "readback": str(args.readback)}
    finalize, grid = finalize_check(args.epoch, args.readback)
    report["finalize"] = finalize
    run = args.epoch / "run"
    records = read_records(args.epoch / "bridge")
    report["cost"] = cost_check(records)
    result = json.loads((run / "result.json").read_text()) if (run / "result.json").exists() else {}
    arguments = result.get("arguments") or {}
    try:
        report["received"] = received_check(records, load_list(run / "spool", "slam.records"))
    except (OSError, ValueError) as exc:
        report["received"] = {"ok": False, "reason": str(exc)}
    nodes_csv = args.readback / "dump" / "nodes.csv"
    nodes = read_nodes(nodes_csv) if nodes_csv.exists() else None
    if nodes is not None:
        report["nodes"] = nodes_check(nodes, records)
        try:
            log = load_slam_log(run)
        except (OSError, ValueError) as exc:  # the truth itself incomplete: neither check can pass
            report["world"] = report["start"] = {"ok": False, "reason": str(exc)}
        else:
            joint_stamps = np.asarray(log["joint_stamps_s"], float)
            base = np.asarray(log["base_pose_world"], float)
            tolerance = 0.5 * float(np.median(np.diff(joint_stamps)))  # half a physics step
            report["world"] = world_check(nodes, joint_stamps, base, tolerance)
            report["start"] = start_check(nodes, joint_stamps, base, tolerance)
        reference = args.reference_run or run
        try:
            report["coverage"] = {"reference": str(reference),
                                  **coverage_check(path_points(reference), np.column_stack((nodes["x"], nodes["y"])),
                                                   COVERAGE_RADIUS_M)}
        except (OSError, ValueError) as exc:
            report["coverage"] = {"reference": str(reference), "ok": False, "reason": str(exc)}
    rigid = json.loads(args.rigid_bodies.read_text()) if args.rigid_bodies and args.rigid_bodies.exists() else None
    report["moving"] = moving_check(rigid, arguments)

    written = False
    if finalize["ok"] and report["received"]["ok"] and nodes is not None and report["nodes"]["ok"]:
        args.out.mkdir(parents=True, exist_ok=True)
        for name in ("map.posegraph", "map.data"):
            shutil.copyfile(args.epoch / "bridge" / "bundle" / name, args.out / name)
        np.savez_compressed(args.out / "grid.npz", **grid)
        hashes = {name: sha256(args.out / name) for name in BUNDLE_FILES}
        version = args.readback / "slam_toolbox_version.txt"
        manifest = {
            "format": "slam_map_bundle/1",
            "map_id": map_id(hashes),
            "files": hashes,
            "map_from_world": [0.0, 0.0, 0.0],
            "map_from_world_check": report["start"],
            "grid": {"shape": list(grid["data"].shape), "origin": grid["origin"].tolist(),
                     "resolution_m": grid["resolution_m"], "source": "readback /map (slam_bundle_container.sh)"},
            "graph": {"scans": finalize["graph"]["scans"], "vertices": finalize["graph"]["vertices"],
                      "edges": finalize["graph"]["edges"]},
            "last_processed": report["nodes"]["last_processed"],
            "survey_stamps_s": [report["nodes"]["first_stamp_s"], report["nodes"]["last_stamp_s"]],
            "seed": result.get("seed"),
            "run": {key: result.get(key) for key in ("scene_sha256", "forklift_urdf_sha256", "pallet_urdf_sha256",
                                                     "pallet_geometry_sha256", "script_sha256", "source_sha256",
                                                     "scene_built_from_urdf_sha256", "arguments")},
            "inputs": input_hashes(arguments, args.root),
            "factory": {key: (result.get("factory") or {}).get(key) for key in ("layout_version", "layout_sha256")},
            "settings_synthetic": result.get("settings_synthetic"),
            "lidar_synthetic": result.get("lidar_synthetic"),
            "noise": {key: arguments.get(key) for key in sorted(arguments) if "noise" in key},
            "slam_params_sha256": sha256(args.slam_params),
            "slam_toolbox_version": version.read_text().strip() if version.exists() else None,
            "image": args.image,
            "survey": str(args.epoch),
        }
        (args.out / "manifest.json").write_text(json.dumps(manifest, indent=1))
        report["map_id"] = manifest["map_id"]
        written = True
    report["bundle_written"] = written
    report["ok"] = bool(written and all(report[k]["ok"] for k in ("world", "start", "coverage", "cost", "moving")))
    args.report.write_text(json.dumps(report, indent=1))
    print(json.dumps({k: (v.get("ok") if isinstance(v, dict) else v) for k, v in report.items()}))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
