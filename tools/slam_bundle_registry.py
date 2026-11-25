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
must agree with the S10-A taped start within the start tolerance, compared at the rear axle as the live loop compares
its start (the report's base_link start taken to the rear axle by the chassis's offset), and the S10-A
settings must name the same map. The command also checks the bundle's files against its manifest (``load_bundle``),
the raw records against their listed hashes, and the lidar and chassis hashes against the files it reads; the
report lives under the bundle root and the row keeps its path relative to that root. A new site row is not
relocalizable at its start.

    python tools/slam_bundle_registry.py add-site REGISTRY --name NAME --bundle DIR --report SITE_REPORT
        --site-record SITE_RECORD --s10a S10A_YAML --vehicle VEHICLE_YAML --lidar LIDAR_YAML
    python tools/slam_bundle_registry.py lookup-site REGISTRY --site-record SITE_RECORD --encoder-model MODEL
        (prints as lookup)

The site survey report (JSON, written by the survey, never by this tool)::

    {"kind": "site_survey/1", "ok": true, "site": "<name>", "site_record_sha256": "<64 hex>",
     "map_id": "<the bundle's>", "encoder_model": "<model>", "start_pose": [x, y, yaw],   # map <- base_link
     "config_sha256": {"lidar": "<64 hex>", "chassis": "<64 hex>"},   # lidar.yaml, the chassis URDF
     "vehicle_calibration_id": "<16 hex>",                           # load_real_vehicle(...).calibration_id
     "raw": [{"kind": "observe" | "teleop" | "ok", "path": "<file>", "sha256": "<64 hex>"}, ...],
            # the survey drive's records, paths relative to the report's folder, each once: at least one
            # observation record and one teleop record; each observation record with its .ok listed
            # (real_observe.record_complete); kinds are checked against the files' own headers
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
# The registered start against the S10-A taped start, compared at the rear axle as the live loop compares its start
# (real_loop._off_start): the loop's START_TOL_M / START_TOL_RAD, provisional until bench 7 (Codex 7c-2a round 1).
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
               site_record_sha256: str, s10a_map_id, s10a_start_rear, rear_offset_m: float, tol_m: float,
               tol_rad: float) -> dict:
    """The registry row of a real site survey (ValueError naming the first problem). ``site_record_sha256`` is the
    hash of the site record file itself; ``s10a_start_rear`` the S10-A taped start (rear axle), compared with the
    report's base_link start taken to the rear axle by ``rear_offset_m``. The report's files are checked apart
    (``check_site_sources``)."""
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
    if not (isinstance(model, str) and model.strip()) or (bundle_model is not None and bundle_model != model):
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
    tape = [float(v) for v in s10a_start_rear]
    x, y, yaw = (float(v) for v in start)
    rear = (x - rear_offset_m * math.cos(yaw), y - rear_offset_m * math.sin(yaw), yaw)  # start_reloc.rear_from_base
    d = math.hypot(rear[0] - tape[0], rear[1] - tape[1])
    dyaw = abs(math.remainder(rear[2] - tape[2], math.tau))
    if d > tol_m or dyaw > tol_rad:
        raise ValueError(f"{name}: the surveyed start is {d:.3f} m / {math.degrees(dyaw):.1f} deg from the S10-A "
                         f"taped start at the rear axle (tolerance {tol_m} m / {math.degrees(tol_rad):.1f} deg)")
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


OBSERVE_SCHEMA = "observe_record/1"  # tools/real_observe.py's record header
TELEOP_SCHEMA = "teleop_bridge_record/1"  # tools/teleop_bridge.py's record header


def _header(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        try:
            row = json.loads(f.readline())
        except ValueError:
            return {}
    return row if isinstance(row, dict) and row.get("t") == "header" else {}


def _kind_of(path: Path) -> str | None:
    """A survey record's kind by its own content (never by the report's tag, Codex 7c-2a round 3)."""
    if path.name.endswith(".ok"):
        return "ok"
    schema = _header(path).get("schema")
    return {OBSERVE_SCHEMA: "observe", TELEOP_SCHEMA: "teleop"}.get(schema)


def check_site_sources(report: dict, report_dir: Path, *, lidar_sha256: str, chassis_sha256: str,
                       calibration_id: str) -> None:
    """The report binds one real survey (design 6.1 item 5, Codex 7c-2a rounds 1-3). Its raw records are files under
    the report's folder (relative paths, each listed once) with the hashes it lists. Each file's kind is read from the
    file -- an observation record (``observe_record/1``), a teleop record (``teleop_bridge_record/1``) or an ``.ok``
    -- and must be the kind the report gives. It needs at least one observation record and one teleop record; each
    observation record is complete by its own listed ``.ok`` (``real_observe.record_complete``), and every ``.ok`` is
    an observation record's. No record may be synthetic (a rehearsal's: ``synthetic.refuse_or_mark``). A teleop record
    was not refused and names the vehicle calibration given here. The report's lidar and chassis hashes and its
    vehicle calibration are those of the files this registration reads. ValueError naming the first problem."""
    import os  # noqa: PLC0415

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from forklift_core.live.synthetic import SyntheticRefused, refuse_or_mark  # noqa: PLC0415
    from real_observe import record_complete  # noqa: PLC0415

    raw = report.get("raw") or []
    base = Path(report_dir).resolve()
    seen: dict = {}
    for r in raw:
        rel = Path(os.path.normpath(str(r.get("path", ""))))
        f = (base / rel).resolve()
        if rel.is_absolute() or base not in f.parents:
            raise ValueError(f"the survey record {rel} is not under the report's folder")
        if f in seen.values():
            raise ValueError(f"the survey record {rel} is listed twice")
        if not f.is_file():
            raise ValueError(f"the survey record {rel} is not there")
        if sha256(f) != r.get("sha256"):
            raise ValueError(f"the survey record {rel} differs from the report's hash")
        kind = _kind_of(f)
        if kind is None or kind != r.get("kind"):
            raise ValueError(f"the survey record {rel} is a {kind} record by its content, not {r.get('kind')!r}")
        seen[(kind, str(rel))] = f
    observe = [f for (k, _), f in seen.items() if k == "observe"]
    teleop = [f for (k, _), f in seen.items() if k == "teleop"]
    oks = [f for (k, _), f in seen.items() if k == "ok"]
    if not observe or not teleop:
        raise ValueError("the report needs the survey's observation and teleop records")
    for f in observe + teleop:
        try:
            refuse_or_mark([_header(f)], allow=False)
        except SyntheticRefused as exc:
            raise ValueError(f"the survey record {f.name} is a rehearsal's: {exc}") from exc
    for f in observe:
        if Path(str(f) + ".ok") not in oks:
            raise ValueError(f"the observation record {f.name} has no .ok listed")
        if not record_complete(f):
            raise ValueError(f"the observation record {f.name} is not complete by its .ok")
    for ok in oks:
        if Path(str(ok)[: -len(".ok")]) not in observe:
            raise ValueError(f"the marker {ok.name} is not a listed observation record's")
    for f in teleop:
        if ((_header(f).get("config") or {}).get("vehicle") or {}).get("calibration_id") != calibration_id:
            raise ValueError(f"the teleop record {f.name} names another vehicle calibration")
        with open(f, encoding="utf-8") as fh:
            for line in fh:
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if isinstance(row, dict) and row.get("t") == "refused":
                    raise ValueError(f"the teleop record {f.name} was refused (no survey drive in it)")
    cfg = report.get("config_sha256") or {}
    if cfg.get("lidar") != lidar_sha256:
        raise ValueError("the report's lidar settings hash is not the lidar settings given here")
    if cfg.get("chassis") != chassis_sha256:
        raise ValueError("the report's chassis hash is not the chassis model this registration reads")
    if report.get("vehicle_calibration_id") != calibration_id:
        # the odometry: the URDF alone does not carry the gears and counts (Codex 7c-2a round 2)
        raise ValueError("the report's vehicle calibration is not the vehicle settings given here")


def report_path(report: Path, bundle: Path) -> str:
    """The report's path relative to the bundle root (the bundle folder's parent, the launcher's SLAM_BUNDLE_ROOT): no
    host path in the repository, and the row finds its report again (Codex 7c-2a round 1 P3)."""
    root, f = Path(bundle).resolve().parent, Path(report).resolve()
    if root not in f.parents:
        raise ValueError(f"the report {report} is not under the bundle root {root}")
    return str(f.relative_to(root))


def lookup(book: dict, scene_sha256: str, encoder_model: str) -> dict | None:
    """An Isaac scene row (site rows have no scene file and are never found here; a scene row without its
    ``scene_sha256`` is an error, as before -- never read as no row, Codex 7c-2a round 1)."""
    rows = [r for r in book.get("bundles") or [] if not is_site(r) and r["scene_sha256"] == scene_sha256
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
    """The S10-A settings' map id and taped start (rear axle), the chassis model's rear axle offset, the registered
    start's tolerance (the live loop's ``START_TOL_M``/``START_TOL_RAD``), the chassis URDF's hash and the vehicle's
    calibration id."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from forklift_core.live.real_autonomy import load_s10a  # noqa: PLC0415
    from forklift_core.live.real_loop import START_TOL_M, START_TOL_RAD  # noqa: PLC0415
    from forklift_core.live.real_run import chassis_reference  # noqa: PLC0415
    from forklift_core.plant.real_vehicle import chassis_model_path, load_real_vehicle  # noqa: PLC0415

    cfg = load_s10a(s10a)
    urdf = chassis_model_path(vehicle).parent / "forklift.urdf"
    _tip, rear_offset, _carriage = chassis_reference(urdf)
    rear = tuple(float(v) for v in cfg.start_rear)
    if len(rear) != 3 or not all(math.isfinite(v) for v in rear):
        raise ValueError(f"{s10a}: start_rear is not three finite numbers")
    return {"s10a_map_id": cfg.map_id, "s10a_start_rear": rear, "rear_offset_m": float(rear_offset),
            "tol_m": START_TOL_M, "tol_rad": START_TOL_RAD, "chassis_sha256": sha256(urdf),
            "calibration_id": load_real_vehicle(vehicle).calibration_id}


def _verify_bundle(folder: Path, map_id) -> None:
    """The bundle's files are its manifest's and its map is the report's (slam_map.load_bundle, Codex 7c-2a round 1:
    a manifest left over from another bundle must not carry other files in)."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from forklift_core.localization.slam_map import load_bundle  # noqa: PLC0415

    if not map_id:
        raise ValueError("the report names no map_id")
    load_bundle(folder, expected_map_id=map_id)


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
    site.add_argument("--lidar", type=Path, required=True, help="config/real/lidar.yaml (its hash is the report's)")
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
        a.report = a.report.resolve()  # one path for the sources and the row (Codex 7c-2a round 2 P3)
        report_bytes = a.report.read_bytes()  # one read for the checks and the row's hash (round 3 P3)
        report = json.loads(report_bytes)
        _verify_bundle(a.bundle, report.get("map_id"))
        start = _s10a_start(a.s10a, a.vehicle)
        check_site_sources(report, a.report.parent, lidar_sha256=sha256(a.lidar),
                           chassis_sha256=start.pop("chassis_sha256"), calibration_id=start.pop("calibration_id"))
        row = site_entry(json.loads((a.bundle / "manifest.json").read_text()), report, name=a.name,
                         folder=a.bundle.name, report_name=report_path(a.report, a.bundle),
                         report_sha256=hashlib.sha256(report_bytes).hexdigest(),
                         site_record_sha256=sha256(a.site_record), **start)
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
