#!/usr/bin/env python3
"""The survey map bundle registry (live-default plan docs/plans/2026-10-26-slam-localization-default.md D1/D2).

``config/slam_map_bundles.yaml`` lists the accepted survey bundles, one per scene file (by content) and odometry model:
the bundle's folder name under the launcher's ``SLAM_BUNDLE_ROOT`` (no host path in the repository), its full map id
(the supervisor holds the folder to it), its start pose as map <- base_link (the survey's truth base pose at its first
node: map <- world is the identity the survey's start check confirmed) and the survey's V2' figures. Only a survey
whose checks all passed (``survey_report.json`` ok, the world check included) enters it (D5).

    python tools/slam_bundle_registry.py add REGISTRY --name NAME --bundle DIR --report SURVEY_REPORT
        --survey-run RUN --nodes NODES_CSV
    python tools/slam_bundle_registry.py lookup REGISTRY --scene-file FILE --encoder-model MODEL
        (prints "folder map_id x,y,yaw"; exit 3 when the registry has none, any other failure 1)
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import yaml

SCENE_INPUT = "scene_file"
NONE = 3  # lookup: no row (an error exits 1, so the launcher can tell the two apart)


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def start_pose(base_pose_row) -> list[float]:
    """map <- base_link from a truth base pose row (x, y, z, qw, qx, qy, qz)."""
    row = np.asarray(base_pose_row, float)
    w, x, y, z = row[3:7]
    return [float(row[0]), float(row[1]), float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))]


def entry(manifest: dict, report: dict, *, folder: str, start, name: str) -> dict:
    """The registry row of an accepted survey bundle (ValueError when the survey was not accepted or has no scene)."""
    world = report.get("world") or {}
    if not report.get("map_id") or report["map_id"] != manifest.get("map_id"):  # the writer always records it
        raise ValueError(f"{name}: the report's map_id {str(report.get('map_id'))[:12]} is not the bundle's "
                         f"{str(manifest.get('map_id'))[:12]}")
    if not (report.get("ok") is True and world.get("ok") is True):
        raise ValueError(f"{name}: the survey was not accepted (report ok {report.get('ok')}, world ok {world.get('ok')})")
    scene = (manifest.get("inputs") or {}).get(SCENE_INPUT) or {}
    if not scene.get("sha256"):
        raise ValueError(f"{name}: the bundle names no scene file (a factory survey is not registered here)")
    model = ((manifest.get("run") or {}).get("arguments") or {}).get("encoder_model")
    return {"name": name, "scene_sha256": scene["sha256"], "scene": Path(scene.get("path", "")).name,
            "encoder_model": model, "folder": folder, "map_id": manifest["map_id"],
            "start_pose": [float(v) for v in start],
            "v2": {"position_max_m": float(world["position_max_m"]), "yaw_max_rad": float(world["yaw_max_rad"])}}


def check_sources(report: dict, survey_run: Path, nodes_csv: Path) -> None:
    """The survey run and read-back nodes the start pose is computed from are the report's own (ValueError else)."""
    if Path(survey_run).resolve() != (Path(report["epoch"]) / "run").resolve():
        raise ValueError(f"the survey run {survey_run} is not the report's ({report['epoch']}/run)")
    if Path(nodes_csv).resolve() != (Path(report["readback"]) / "dump" / "nodes.csv").resolve():
        raise ValueError(f"the nodes {nodes_csv} are not the report's read-back ({report['readback']})")


def put(book: dict, row: dict) -> dict:
    """The registry with ``row`` in place of any row of the same scene and model."""
    kept = [r for r in book.get("bundles") or []
            if (r["scene_sha256"], r["encoder_model"]) != (row["scene_sha256"], row["encoder_model"])]
    return {**book, "bundles": kept + [row]}


def lookup(book: dict, scene_sha256: str, encoder_model: str) -> dict | None:
    rows = [r for r in book.get("bundles") or [] if r["scene_sha256"] == scene_sha256 and r["encoder_model"] == encoder_model]
    if len(rows) > 1:
        raise ValueError(f"two registry rows for scene {scene_sha256[:12]} and {encoder_model}")
    return rows[0] if rows else None


def read(path: Path, *, missing_ok: bool = False) -> dict:
    """The registry; a file that is not there is empty only for ``add`` (``missing_ok``), an error for a lookup."""
    if not Path(path).exists():
        if missing_ok:
            return {"bundles": []}
        raise FileNotFoundError(f"no SLAM bundle registry at {path}")
    return yaml.safe_load(Path(path).read_text()) or {"bundles": []}


def write(path: Path, book: dict) -> None:
    header = ("# Accepted survey map bundles (docs/plans/2026-10-26-slam-localization-default.md D1), written by\n"
              "# tools/slam_bundle_registry.py add: folder is under the launcher's SLAM_BUNDLE_ROOT.\n")
    Path(path).write_text(header + yaml.safe_dump(book, sort_keys=False))


def _survey_start(run: Path, nodes_csv: Path) -> list[float]:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from forklift_core.live.spool import load_slam_log  # noqa: PLC0415

    import slam_survey_bundle as sb  # noqa: PLC0415

    nodes = sb.read_nodes(nodes_csv)
    log = load_slam_log(run)
    stamps = np.asarray(log["joint_stamps_s"], float)
    first = int(np.argmin(nodes["stamp_s"]))
    row = int(sb.rows_at(nodes["stamp_s"][first:first + 1], stamps, 0.5 * float(np.median(np.diff(stamps))))[0])
    if row < 0:
        raise ValueError("no truth at the survey's first node")
    return start_pose(np.asarray(log["base_pose_world"], float)[row])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    add = sub.add_parser("add")
    add.add_argument("registry", type=Path)
    add.add_argument("--name", required=True)
    add.add_argument("--bundle", type=Path, required=True)
    add.add_argument("--report", type=Path, required=True)
    add.add_argument("--survey-run", type=Path, required=True)
    add.add_argument("--nodes", type=Path, required=True, help="the survey's read-back nodes.csv")
    look = sub.add_parser("lookup")
    look.add_argument("registry", type=Path)
    look.add_argument("--scene-file", type=Path, required=True)
    look.add_argument("--encoder-model", required=True)
    a = ap.parse_args(argv)
    if a.cmd == "add":
        manifest = json.loads((a.bundle / "manifest.json").read_text())
        report = json.loads(a.report.read_text())
        check_sources(report, a.survey_run, a.nodes)
        row = entry(manifest, report, folder=a.bundle.name, start=_survey_start(a.survey_run, a.nodes), name=a.name)
        write(a.registry, put(read(a.registry, missing_ok=True), row))
        print(json.dumps(row))
        return 0
    row = lookup(read(a.registry), sha256(a.scene_file), a.encoder_model)
    if row is None:
        return NONE
    print(row["folder"], row["map_id"], ",".join(repr(float(v)) for v in row["start_pose"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
