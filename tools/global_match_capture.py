"""Pose lists for the global matcher's scan capture (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md
B4a), from a survey bundle's grid and the world's own low obstacles; never reads a matcher output.

    python tools/global_match_capture.py poses --bundle BUNDLE --keep-out SCENE_OR_RESULT --split dev --map-index 0 \\
        --seed 1027 --sep-m 1.5 --count 40 --out dev_line_m.json
    python tools/global_match_capture.py poses --bundle HALL --keep-out RESULT --split val --map-index 2 --seed 1028 \\
        --sep-m 3.0 --fill --taken dev_line_m.json --taken dev_random_m.json --out val_hall.json
    python tools/global_match_capture.py command --bundle BUNDLE --poses P.json --out-npz CAP.npz --output RUN

Starts are drawn in the bundle's map frame and written with their hall (world) pose too (the bundle's
``map_from_world``): the capture teleports in the world, and ``--taken`` lists from other maps are compared in the
hall frame. Keep-outs: a session scene's static props and pallets, or a run record's scenario props and pickup pallet
(a pallet as a 1.3 m square, either orientation); every wall visual of a scene file too, whatever its height.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "sim" / "isaac"))

from forklift_core.localization import relocalization_samples as S  # noqa: E402
from forklift_core.localization.slam_map import Grid, file_sha256  # noqa: E402
from forklift_core.localization.slam_pose import compose, invert  # noqa: E402
from forklift_core.planning.geometry import Rectangle  # noqa: E402

PALLET_SQUARE_M = 1.3
SPLITS = {"dev": 0, "val": 1}


def keep_out_rectangles(path: Path) -> list:
    """Rectangles (as corner arrays) of the low obstacles the survey grid does not show."""
    record = json.loads(Path(path).read_text())
    rects: list = []
    if "scenario" in record and "arguments" in record:  # a run's result.json
        sc = record["scenario"]
        rects += [Rectangle(**p["rectangle"]) for p in sc.get("props", [])]
        for site in (sc["pickup"],):
            rects.append(Rectangle(site["x_m"], site["y_m"], PALLET_SQUARE_M, PALLET_SQUARE_M, site["yaw_rad"]))
    else:  # a scene file
        import scene_file  # noqa: PLC0415

        if record.get("format") == scene_file.SESSION_FORMAT:
            scene = scene_file.load_session(path)
            rects += [p.rectangle for p in scene.static_props]
            rects += [Rectangle(p.truth.x_m, p.truth.y_m, PALLET_SQUARE_M, PALLET_SQUARE_M, p.truth.yaw_rad)
                      for p in scene.pallets]
        else:
            scene = scene_file.load_scene(path)
            rects += [p.rectangle for p in scene.scenario.props]
            pk = scene.scenario.pickup
            rects.append(Rectangle(pk.x_m, pk.y_m, PALLET_SQUARE_M, PALLET_SQUARE_M, pk.yaw_rad))
        rects += [v.rectangle for v in scene.visuals if v.kind == "wall"]
    return [S.rectangle_corners(r) for r in rects]


def bundle_frame(bundle: Path) -> tuple:
    manifest = json.loads((bundle / "manifest.json").read_text())
    return manifest["map_id"], tuple(float(v) for v in manifest.get("map_from_world", (0.0, 0.0, 0.0)))


def taken_in_map(files, map_from_world) -> list:
    """Base poses of earlier lists, in this map's frame (through the hall frame)."""
    out = []
    for f in files:
        for row in json.loads(Path(f).read_text())["poses"]:
            out.append(compose(map_from_world, tuple(row["base_world"])))
    return out


def capture_argv(manifest: dict, *, bundle: Path, poses: Path, out_npz: Path, output_dir: Path) -> list:
    """The runner arguments of a capture on the world a survey bundle saw: the input files by the paths the survey
    recorded (so their hashes match), the world arguments as recorded (layout, scene file, seed, colliders, minimum
    prop height, the scene's own keep-out and exit clearance), the SLAM LiDAR and the shared obstacle layer."""
    a = manifest["run"]["arguments"]
    inputs = manifest["inputs"]
    argv = ["--base-scene", inputs["base_scene"]["path"], "--forklift-urdf", inputs["forklift_urdf"]["path"],
            "--factory-layout", inputs["factory_layout"]["path"], "--lidar", inputs["lidar"]["path"],
            "--obstacle-layer", inputs["obstacle_layer"]["path"], "--pallet-urdf", a["pallet_urdf"],
            "--pallet-geometry", a["pallet_geometry"], "--settings", a["settings"],
            "--encoder-model", a["encoder_model"], "--seed", str(a["seed"]), "--layout", a["layout"],
            "--record-slam", "--output", str(output_dir),
            "--scan-capture-poses", str(poses), "--scan-capture-out", str(out_npz), "--scan-capture-bundle", str(bundle)]
    if a["layout"] == "scene":
        argv += ["--scene-file", inputs["scene_file"]["path"], "--near-capture-redraw", "always"]
    if a.get("prism_colliders"):
        argv.append("--prism-colliders")
    for key, flag in (("min_obstacle_height_m", "--min-obstacle-height-m"), ("lidar_keepout_m", "--lidar-keepout-m"),
                      ("exit_clearance_m", "--exit-clearance-m")):
        if a.get(key) is not None:
            argv += [flag, str(a[key])]
    return argv


def cmd_command(a) -> int:
    manifest = json.loads((a.bundle / "manifest.json").read_text())
    for word in capture_argv(manifest, bundle=a.bundle, poses=a.poses, out_npz=a.out_npz, output_dir=a.output):
        print(word)
    return 0


def cmd_poses(a) -> int:
    grid = Grid.from_npz(a.bundle / "grid.npz")
    map_id, map_from_world = bundle_frame(a.bundle)
    world_from_map = invert(map_from_world)
    keep = keep_out_rectangles(a.keep_out) if a.keep_out else []
    taken = taken_in_map(a.taken, map_from_world)
    report: dict = {}
    starts = S.sample_starts(grid, count=a.count if not a.fill else 1_000_000, seed=a.seed, keep_out=keep,
                             taken=taken, sep_m=a.sep_m, max_attempts=a.max_attempts, fill=a.fill, report=report)
    record = {"format": "scan_capture_poses/1", "split": SPLITS[a.split], "map_index": a.map_index,
              "map_id": map_id, "map_from_world": list(map_from_world), "bundle": str(a.bundle),
              "keep_out": None if not a.keep_out else {"path": str(a.keep_out), "sha256": file_sha256(a.keep_out)},
              "taken": [{"path": str(f), "sha256": file_sha256(Path(f))} for f in a.taken],
              "seed": a.seed, "sep_m": a.sep_m, "fill": a.fill, "draw": report,
              "poses": [{"id": f"{a.split}{a.map_index}_{p.index:03d}", "stratum": p.stratum, "base": list(p.base),
                         "base_world": list(compose(world_from_map, p.base))} for p in starts]}
    a.out.write_text(json.dumps(record, indent=1) + "\n")
    strata = {k: sum(p.stratum == k for p in starts) for k in "ABC"}
    print(json.dumps({"out": str(a.out), "poses": len(starts), "strata": strata, "stop": report["stop_reason"]}))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("poses")
    p.add_argument("--bundle", type=Path, required=True)
    p.add_argument("--keep-out", type=Path, default=None, help="a scene file or a run's result.json on this world")
    p.add_argument("--split", choices=sorted(SPLITS), required=True)
    p.add_argument("--map-index", type=int, required=True)
    p.add_argument("--seed", type=int, required=True)
    p.add_argument("--sep-m", type=float, required=True)
    p.add_argument("--count", type=int, default=0)
    p.add_argument("--fill", action="store_true", help="draw until the map holds no more")
    p.add_argument("--taken", action="append", default=[], help="earlier pose lists (any map), kept apart")
    p.add_argument("--max-attempts", type=int, default=200_000)
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("command", help="the runner arguments of a capture, one per line")
    p.add_argument("--bundle", type=Path, required=True)
    p.add_argument("--poses", type=Path, required=True)
    p.add_argument("--out-npz", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True, help="the runner's output folder")
    a = ap.parse_args(argv)
    if a.cmd == "command":
        return cmd_command(a)
    if not a.fill and a.count <= 0:
        ap.error("--count or --fill")
    return cmd_poses(a)


if __name__ == "__main__":
    raise SystemExit(main())


