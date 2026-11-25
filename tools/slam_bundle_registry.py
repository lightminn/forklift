#!/usr/bin/env python3
"""The survey map bundle registry (live-default plan docs/plans/2026-10-26-slam-localization-default.md D1/D2).

``config/slam_map_bundles.yaml`` lists the accepted survey bundles, one per scene file (by content) and odometry model:
the bundle's folder name under the launcher's ``SLAM_BUNDLE_ROOT`` (no host path in the repository), its full map id
(the supervisor holds the folder to it), its start pose as map <- base_link (the survey's truth base pose at its first
node: map <- world is the identity the survey's start check confirmed) and the survey's V2' figures. Only a survey
whose checks all passed (``survey_report.json`` ok, the world check included) enters it (D5).

Each row also says whether the truck can be relocalized at that start (``start_relocalizable``, with the Isaac smoke
that showed it in ``start_reloc_evidence``: run and date; lead 2026-10-31, start relocalization plan 7.1 (a)): the
operator console relocalizes by default only on a row marked yes. A new row, and a new survey that replaces one, is
"no" until ``mark`` says otherwise; a yes needs its evidence.

    python tools/slam_bundle_registry.py add REGISTRY --name NAME --bundle DIR --report SURVEY_REPORT
        --survey-run RUN --nodes NODES_CSV
    python tools/slam_bundle_registry.py mark REGISTRY --name NAME --relocalizable yes|no --evidence TEXT
    python tools/slam_bundle_registry.py lookup REGISTRY --scene-file FILE --encoder-model MODEL
        (prints "folder map_id x,y,yaw yes|no"; exit 3 when the registry has none, any other failure 1)

Real-site rows (docs/design/2026-11-25-real-sources-pr7.md 6.1 item 5, PR 7c-2a) sit in the same list. They have no
``scene_sha256``: they are found by ``(site_record_sha256, encoder_model)``, the hash of the site record the
registration command reads itself. One site survey report binds the row together (``site_survey/1``, below): the site
record's hash, the bundle's ``map_id``, the sensor and odometry settings' hashes, the survey run's raw records, the start
pose in the map and the five-mark site check (observed figures against limits fixed beforehand). The registered start
must agree with the S10-A taped start (``start_rear`` taken to base_link) within the start tolerance, and the S10-A
settings must name the same map. A new site row is not relocalizable at its start.

    python tools/slam_bundle_registry.py add-site REGISTRY --name NAME --bundle DIR --report SITE_REPORT
        --site-record SITE_RECORD --s10a S10A_YAML --vehicle VEHICLE_YAML
    python tools/slam_bundle_registry.py lookup-site REGISTRY --site-record SITE_RECORD --encoder-model MODEL
        (prints as lookup)

The site survey report (JSON, written by the survey, never by this tool)::

    {"kind": "site_survey/1", "ok": true, "site": "<name>", "site_record_sha256": "<64 hex>",
     "map_id": "<the bundle's>", "encoder_model": "<model>", "start_pose": [x, y, yaw],   # map <- base_link
     "config_sha256": {"lidar": "<64 hex>", "chassis": "<64 hex>"},
     "raw": [{"path": "<record>", "sha256": "<64 hex>"}, ...],   # the survey drive's records and .ok files
     "site_check": {"points": 5, "position_max_m": m, "yaw_max_rad": r,
                    "limits": {"position_m": m, "yaw_rad": r}}}
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

SCENE_INPUT = "scene_file"
SITE_REPORT_KIND = "site_survey/1"
SITE_CHECK_POINTS = 5  # the five floor marks (design 6.1 item 5)
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
            "v2": {"position_max_m": float(world["position_max_m"]), "yaw_max_rad": float(world["yaw_max_rad"])},
            "start_relocalizable": False, "start_reloc_evidence": None}


def start_relocalizable(row: dict) -> bool:
    """The row's mark: a row written before the mark is no; a mark that is not a bool, or a yes without its evidence,
    does not read (ValueError)."""
    mark = row.get("start_relocalizable", False)
    if not isinstance(mark, bool):
        raise ValueError(f"{row.get('name')}: start_relocalizable is {mark!r}, not true or false")
    if mark and not str(row.get("start_reloc_evidence") or "").strip():
        raise ValueError(f"{row.get('name')}: start_relocalizable is true without its evidence")
    return mark


def mark(book: dict, name: str, relocalizable: bool, evidence: str | None) -> dict:
    """The registry with row ``name`` marked (a copy); a yes needs its evidence (the smoke's run and date)."""
    if relocalizable and not str(evidence or "").strip():
        raise ValueError(f"{name}: marking a start relocalizable needs its evidence (the Isaac smoke's run and date)")
    rows = book.get("bundles") or []
    if sum(r["name"] == name for r in rows) != 1:
        raise ValueError(f"no registry row named {name} (or more than one)")
    return {**book, "bundles": [{**r, "start_relocalizable": bool(relocalizable),
                                 "start_reloc_evidence": str(evidence).strip() if evidence else None}
                                if r["name"] == name else r for r in rows]}


def check_sources(report: dict, survey_run: Path, nodes_csv: Path) -> None:
    """The survey run and read-back nodes the start pose is computed from are the report's own (ValueError else)."""
    if Path(survey_run).resolve() != (Path(report["epoch"]) / "run").resolve():
        raise ValueError(f"the survey run {survey_run} is not the report's ({report['epoch']}/run)")
    if Path(nodes_csv).resolve() != (Path(report["readback"]) / "dump" / "nodes.csv").resolve():
        raise ValueError(f"the nodes {nodes_csv} are not the report's read-back ({report['readback']})")


def is_site(row: dict) -> bool:
    """A real-site row (no scene file: keyed by its site record)."""
    return "site_record_sha256" in row


def _key(row: dict) -> tuple:
    return ("site", row["site_record_sha256"], row["encoder_model"]) if is_site(row) else (
        "scene", row["scene_sha256"], row["encoder_model"])


def put(book: dict, row: dict) -> dict:
    """The registry with ``row`` in place of any row of the same kind, key and model (scene rows by the scene file,
    site rows by the site record)."""
    kept = [r for r in book.get("bundles") or [] if _key(r) != _key(row)]
    return {**book, "bundles": kept + [row]}


def lookup_site(book: dict, site_record_sha256: str, encoder_model: str) -> dict | None:
    rows = [r for r in book.get("bundles") or [] if is_site(r) and r["site_record_sha256"] == site_record_sha256
            and r["encoder_model"] == encoder_model]
    if len(rows) > 1:
        raise ValueError(f"two registry rows for site record {site_record_sha256[:12]} and {encoder_model}")
    return rows[0] if rows else None


def _hex64(v) -> bool:
    return isinstance(v, str) and len(v) == 64 and all(c in "0123456789abcdef" for c in v)


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v))


def site_entry(manifest: dict, report: dict, *, name: str, folder: str, report_name: str, report_sha256: str,
               site_record_sha256: str, s10a_map_id, s10a_start_base, tol_m: float, tol_rad: float) -> dict:
    """The registry row of a real site survey (ValueError naming the first problem). ``site_record_sha256`` is the
    hash of the site record file itself; ``s10a_start_base`` the S10-A taped start as base_link."""
    if report.get("kind") != SITE_REPORT_KIND:
        raise ValueError(f"{name}: the report is not a {SITE_REPORT_KIND} report (kind {report.get('kind')!r})")
    if report.get("ok") is not True:
        raise ValueError(f"{name}: the site survey was not accepted (report ok {report.get('ok')!r})")
    if report.get("site_record_sha256") != site_record_sha256:
        raise ValueError(f"{name}: the site record {site_record_sha256[:12]} is not the report's "
                         f"{str(report.get('site_record_sha256'))[:12]}")
    if not manifest.get("map_id") or report.get("map_id") != manifest["map_id"]:
        raise ValueError(f"{name}: the report's map_id {str(report.get('map_id'))[:12]} is not the bundle's "
                         f"{str(manifest.get('map_id'))[:12]}")
    if s10a_map_id != manifest["map_id"]:
        raise ValueError(f"{name}: the S10-A settings name map {str(s10a_map_id)[:12]}, not the bundle's "
                         f"{manifest['map_id'][:12]}")
    model = report.get("encoder_model")
    bundle_model = ((manifest.get("run") or {}).get("arguments") or {}).get("encoder_model")
    if not model or (bundle_model is not None and bundle_model != model):
        raise ValueError(f"{name}: the report's encoder model {model!r} is not the bundle's {bundle_model!r}")
    cfg = report.get("config_sha256") or {}
    if not (_hex64(cfg.get("lidar")) and _hex64(cfg.get("chassis"))):
        raise ValueError(f"{name}: the report has no lidar and chassis settings hashes")
    raw = report.get("raw")
    if not raw or not all(isinstance(r, dict) and r.get("path") and _hex64(r.get("sha256")) for r in raw):
        raise ValueError(f"{name}: the report lists no survey records with their hashes")
    start = report.get("start_pose")
    if not (isinstance(start, list) and len(start) == 3 and all(_finite(v) for v in start)):
        raise ValueError(f"{name}: the report's start_pose is not three finite numbers")
    tape = [float(v) for v in s10a_start_base]
    d = math.hypot(float(start[0]) - tape[0], float(start[1]) - tape[1])
    dyaw = abs(math.remainder(float(start[2]) - tape[2], math.tau))
    if d > tol_m or dyaw > tol_rad:
        raise ValueError(f"{name}: the surveyed start is {d:.3f} m / {math.degrees(dyaw):.1f} deg from the S10-A "
                         f"taped start (tolerance {tol_m} m / {math.degrees(tol_rad):.1f} deg)")
    chk = report.get("site_check") or {}
    lim = chk.get("limits") or {}
    if not (isinstance(chk.get("points"), int) and chk["points"] >= SITE_CHECK_POINTS):
        raise ValueError(f"{name}: the site check has {chk.get('points')!r} marks, not {SITE_CHECK_POINTS}")
    if not all(_finite(v) and v > 0 for v in (lim.get("position_m"), lim.get("yaw_rad"))):
        raise ValueError(f"{name}: the site check's limits are not set (position_m, yaw_rad)")
    obs = (chk.get("position_max_m"), chk.get("yaw_max_rad"))
    if not all(_finite(v) and v >= 0 for v in obs):
        raise ValueError(f"{name}: the site check's observed figures are missing")
    if obs[0] > lim["position_m"] or obs[1] > lim["yaw_rad"]:
        raise ValueError(f"{name}: the site check observed {obs[0]:.3f} m / {obs[1]:.4f} rad, over its limits "
                         f"{lim['position_m']} m / {lim['yaw_rad']} rad")
    return {"name": name, "site": str(report.get("site") or name), "site_record_sha256": site_record_sha256,
            "survey_report": {"path": report_name, "sha256": report_sha256}, "encoder_model": model,
            "folder": folder, "map_id": manifest["map_id"], "start_pose": [float(v) for v in start],
            "v2": {"position_max_m": float(obs[0]), "yaw_max_rad": float(obs[1])},
            "site_check_limits": {"position_m": float(lim["position_m"]), "yaw_rad": float(lim["yaw_rad"])},
            "start_relocalizable": False, "start_reloc_evidence": None}


def lookup(book: dict, scene_sha256: str, encoder_model: str) -> dict | None:
    """An Isaac scene row (site rows have no scene file and are never found here)."""
    rows = [r for r in book.get("bundles") or [] if not is_site(r) and r.get("scene_sha256") == scene_sha256
            and r["encoder_model"] == encoder_model]
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


def _s10a_start(s10a: Path, vehicle: Path) -> dict:
    """The S10-A settings' map id and taped start as base_link (its rear axle offset from the chassis model), with the
    registered start's tolerance (the live loop's ``START_TOL_M``/``START_TOL_RAD``)."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from forklift_core.live.real_autonomy import load_s10a  # noqa: PLC0415
    from forklift_core.live.real_loop import START_TOL_M, START_TOL_RAD  # noqa: PLC0415
    from forklift_core.live.real_run import chassis_reference  # noqa: PLC0415
    from forklift_core.localization.start_reloc import base_from_rear  # noqa: PLC0415
    from forklift_core.plant.real_vehicle import chassis_model_path  # noqa: PLC0415

    cfg = load_s10a(s10a)
    _tip, rear_offset, _carriage = chassis_reference(chassis_model_path(vehicle).parent / "forklift.urdf")
    rear = tuple(float(v) for v in cfg.start_rear)
    if len(rear) != 3 or not all(math.isfinite(v) for v in rear):
        raise ValueError(f"{s10a}: start_rear is not three finite numbers")
    return {"s10a_map_id": cfg.map_id, "s10a_start_base": base_from_rear(rear, rear_offset),
            "tol_m": START_TOL_M, "tol_rad": START_TOL_RAD}


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
    mk = sub.add_parser("mark")
    mk.add_argument("registry", type=Path)
    mk.add_argument("--name", required=True)
    mk.add_argument("--relocalizable", choices=("yes", "no"), required=True)
    mk.add_argument("--evidence", default=None, help="the Isaac smoke that showed it: run and date")
    site = sub.add_parser("add-site", help="register a real site survey's bundle (design 6.1 item 5)")
    site.add_argument("registry", type=Path)
    site.add_argument("--name", required=True)
    site.add_argument("--bundle", type=Path, required=True)
    site.add_argument("--report", type=Path, required=True, help="the site survey report (site_survey/1)")
    site.add_argument("--site-record", type=Path, required=True, help="the site record itself (hashed here)")
    site.add_argument("--s10a", type=Path, required=True, help="the S10-A settings (its start_rear and map_id)")
    site.add_argument("--vehicle", type=Path, required=True, help="config/real/vehicle.yaml (the rear axle offset)")
    lsite = sub.add_parser("lookup-site")
    lsite.add_argument("registry", type=Path)
    lsite.add_argument("--site-record", type=Path, required=True)
    lsite.add_argument("--encoder-model", required=True)
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
    if a.cmd == "add-site":
        row = site_entry(json.loads((a.bundle / "manifest.json").read_text()), json.loads(a.report.read_text()),
                         name=a.name, folder=a.bundle.name, report_name=a.report.name, report_sha256=sha256(a.report),
                         site_record_sha256=sha256(a.site_record), **_s10a_start(a.s10a, a.vehicle))
        write(a.registry, put(read(a.registry, missing_ok=True), row))
        print(json.dumps(row))
        return 0
    if a.cmd == "lookup-site":
        row = lookup_site(read(a.registry), sha256(a.site_record), a.encoder_model)
        if row is None:
            return NONE
        print(row["folder"], row["map_id"], ",".join(repr(float(v)) for v in row["start_pose"]),
              "yes" if start_relocalizable(row) else "no")
        return 0
    if a.cmd == "mark":
        write(a.registry, mark(read(a.registry), a.name, a.relocalizable == "yes", a.evidence))
        return 0
    row = lookup(read(a.registry), sha256(a.scene_file), a.encoder_model)
    if row is None:
        return NONE
    print(row["folder"], row["map_id"], ",".join(repr(float(v)) for v in row["start_pose"]),
          "yes" if start_relocalizable(row) else "no")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
