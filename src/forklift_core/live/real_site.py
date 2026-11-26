"""The operating REAL run's site bundle (docs/plans/2026-11-26-real-operating-assembly.md 2.8, C6; unlock U2b).

The operating assembly takes its map bundle from the registry alone (no bundle given by hand), found by the site record
it reads itself and the one odometry model it builds:

- ``resolve_site`` (before the map is loaded): the site record's bytes hashed, the registry's row for that hash and
  ``REAL_ENCODER_MODEL`` (``slam_bundle_registry.lookup_site``: none, or two, refuse), the bundle folder
  ``bundle_root / row["folder"]`` (a plain name), and the row's survey report -- read once under the bundle root, its
  sha256 the row's;
- ``check_site`` (once prep knows the chassis and the vehicle): the report against the files this run reads -- its
  site record hash, encoder model, ``config_sha256.lidar`` and ``.chassis`` (``lidar.yaml`` and the chassis URDF as
  hashed at the start of the preparation), ``vehicle_calibration_id``, its ``map_id`` (the row's and S10-A's), and its
  start pose (map <- base_link) taken to the rear axle within the live loop's start tolerance of S10-A's taped start.

Every refusal is ``precondition:site_bundle:<what>``, never bypassable: the way out is a new survey and registration
(plan 2.8; the survey follows the calibration, plan 4)."""

from __future__ import annotations

import hashlib
import importlib.util
import io
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
PREFIX = "precondition:site_bundle"
# The one odometry model the operating executor builds (``plant.real_vehicle.RealOdometry`` on the measured vehicle
# file, ``real_run.assemble_run``): the identifier a real site survey's report and its registry row carry
# (``encoder_model``) and the key the operating assembly looks its row up by (plan 2.8). Never a free string: a survey
# made with another odometry model has another row or none. (Kept here, not in ``real_vehicle``, which the simulation's
# mission core imports.)
REAL_ENCODER_MODEL = "real_wheel_odometry/1"
SITE_REPORT_KIND = "site_survey/1"


class SiteRefused(ValueError):
    def __init__(self, what: str) -> None:
        super().__init__(what)
        self.reason = f"{PREFIX}:{what}"


@dataclass(frozen=True)
class SiteBundle:
    row: dict
    bundle: Path
    report: dict
    hashes: dict = field(default_factory=dict)  # site_record, registry, site_report: the bytes judged


def _registry_module():
    """``tools/slam_bundle_registry.py`` (its ``lookup_site``, the registration's own rule). A missing or broken file
    refuses (``registry_tool``) and leaves no half-made module behind for a retry."""
    name = "slam_bundle_registry"
    if name in sys.modules and hasattr(sys.modules[name], "lookup_site"):
        return sys.modules[name]
    path = REPO / "tools/slam_bundle_registry.py"
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    except Exception as err:  # noqa: BLE001 -- any failure to load the rule is a refusal, never an escape
        sys.modules.pop(name, None)
        raise SiteRefused(f"registry_tool:{type(err).__name__}") from err
    return module


def _read(path, what: str) -> bytes:
    try:
        return Path(path).read_bytes()
    except OSError as err:
        raise SiteRefused(f"{what}_unreadable:{type(err).__name__}") from err


def _hex64(v) -> bool:
    return isinstance(v, str) and len(v) == 64 and all(c in "0123456789abcdef" for c in v)


def resolve_site(*, site_record, registry, bundle_root, encoder_model: str) -> SiteBundle:
    """The row, bundle folder and report (``SiteRefused`` naming the first problem)."""
    import yaml

    if site_record is None:
        raise SiteRefused("site_record_unset")
    if bundle_root is None:
        raise SiteRefused("bundle_root_unset")
    rec = _read(site_record, "site_record")
    rec_sha = hashlib.sha256(rec).hexdigest()
    raw = _read(registry, "registry")
    try:
        book = yaml.safe_load(raw.decode("utf-8"))
    except (UnicodeDecodeError, yaml.YAMLError) as err:
        raise SiteRefused(f"registry_unreadable:{type(err).__name__}") from err
    if not isinstance(book, dict) or not isinstance(book.get("bundles"), list) \
            or not all(isinstance(r, dict) for r in book["bundles"]):
        raise SiteRefused("registry_unreadable:shape")
    registry_tool = _registry_module()  # its own refusal (registry_tool:...), never read as two rows (round 2 P3)
    try:
        row = registry_tool.lookup_site(book, rec_sha, encoder_model)
    except (KeyError, TypeError) as err:
        raise SiteRefused(f"registry_unreadable:{type(err).__name__}") from err
    except ValueError as err:
        raise SiteRefused("two_rows") from err
    if row is None:
        raise SiteRefused("no_row")
    folder, rep = row.get("folder"), row.get("survey_report")
    if not (isinstance(folder, str) and folder and folder not in (".", "..") and "/" not in folder
            and "\\" not in folder):
        raise SiteRefused("row_folder")
    if not (isinstance(rep, dict) and isinstance(rep.get("path"), str) and rep["path"] and _hex64(rep.get("sha256"))):
        raise SiteRefused("row_report")
    root = Path(bundle_root).resolve()
    bundle = (root / folder).resolve()
    if bundle.parent != root:
        raise SiteRefused("row_folder_outside_root")  # a link out of the root is no folder of it
    report_path = (root / rep["path"]).resolve()
    if root not in report_path.parents:
        raise SiteRefused("row_report_outside_root")
    rbytes = _read(report_path, "report")
    if hashlib.sha256(rbytes).hexdigest() != rep["sha256"]:
        raise SiteRefused("report_sha")
    try:
        report = json.loads(rbytes.decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as err:
        raise SiteRefused("report_unreadable") from err
    if not isinstance(report, dict) or report.get("kind") != SITE_REPORT_KIND or report.get("ok") is not True:
        raise SiteRefused("report_not_accepted")
    return SiteBundle(row=row, bundle=bundle, report=report,
                      hashes={"site_record": rec_sha, "registry": hashlib.sha256(raw).hexdigest(),
                              "site_report": rep["sha256"]})


def _pose3(v) -> bool:
    return (isinstance(v, list) and len(v) == 3
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v))


def read_site_map(site: SiteBundle, expected_map_id):
    """``(map_id, Grid)`` of the row's bundle, every file read once and hashed and parsed from the same bytes (the
    shared ``slam_map.load_bundle`` reads the grid twice: Codex U2b round 1 P2-3). ``SiteRefused`` naming the first
    problem; a bundle whose map is not S10-A's is ``map_id``."""
    from forklift_core.localization import slam_map as sm

    folder = site.bundle
    try:
        manifest = json.loads(_read(folder / "manifest.json", "bundle_manifest").decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as err:
        raise SiteRefused("bundle_manifest_unreadable") from err
    if not isinstance(manifest, dict) or not isinstance(manifest.get("files"), dict) \
            or manifest.get("format") != sm.FORMAT:
        raise SiteRefused("bundle_manifest_unreadable")
    raw = {name: _read(folder / name, "bundle_file") for name in sm.BUNDLE_FILES}
    hashes = {name: hashlib.sha256(b).hexdigest() for name, b in raw.items()}
    if hashes != manifest.get("files"):
        raise SiteRefused("bundle_files")
    actual = sm.map_id_of(hashes)
    if actual != manifest.get("map_id"):
        raise SiteRefused("bundle_files")
    if expected_map_id is None or actual != expected_map_id:
        raise SiteRefused("map_id")
    try:
        with np.load(io.BytesIO(raw["grid.npz"])) as z:
            grid = sm.Grid(np.asarray(z["data"], np.int8), tuple(float(v) for v in z["origin"]),
                           float(z["resolution_m"]))
    except (OSError, ValueError, KeyError, TypeError) as err:
        raise SiteRefused("bundle_grid") from err
    if (grid.data.ndim != 2 or not grid.data.size or len(grid.origin) != 3 or not all(map(math.isfinite, grid.origin))
            or not (math.isfinite(grid.resolution_m) and grid.resolution_m > 0)):
        raise SiteRefused("bundle_grid")
    return actual, grid


def check_site(site: SiteBundle, *, encoder_model: str, lidar_sha256, chassis_sha256, calibration_id, map_id,
               start_rear, rear_offset_m: float, tol_m: float, tol_rad: float) -> list:
    """Every ``precondition:site_bundle:<what>`` the report and the row show against this run (empty: they agree)."""
    r, row = site.report, site.row
    out = []

    def bad(what):
        out.append(f"{PREFIX}:{what}")

    if r.get("site_record_sha256") != site.hashes.get("site_record"):
        bad("site_record")
    if r.get("encoder_model") != encoder_model or row.get("encoder_model") != encoder_model:
        bad("encoder_model")
    cfg = r.get("config_sha256") if isinstance(r.get("config_sha256"), dict) else {}
    if lidar_sha256 is None or cfg.get("lidar") != lidar_sha256:
        bad("lidar")
    if chassis_sha256 is None or cfg.get("chassis") != chassis_sha256:
        bad("chassis")
    if calibration_id is None or r.get("vehicle_calibration_id") != calibration_id:
        bad("vehicle_calibration")
    if not map_id or r.get("map_id") != map_id or row.get("map_id") != map_id:
        bad("map_id")
    start = r.get("start_pose")
    ok_start = _pose3(start)
    if not ok_start:
        bad("start_pose")
    elif not (_pose3(row.get("start_pose")) and all(abs(float(a) - float(b)) <= 1e-9
                                                     for a, b in zip(row["start_pose"], start))):
        bad("row_start")  # the row's start is the report's (registered from it): the hello and relocalization read it
    else:
        x, y, yaw = (float(v) for v in start)
        rear = (x - rear_offset_m * math.cos(yaw), y - rear_offset_m * math.sin(yaw), yaw)  # start_reloc.rear_from_base
        tape = [float(v) for v in start_rear]
        d = math.hypot(rear[0] - tape[0], rear[1] - tape[1])
        dyaw = abs(math.remainder(rear[2] - tape[2], math.tau))
        if d > tol_m or dyaw > tol_rad:
            bad("start")
    return out


__all__ = ["PREFIX", "REAL_ENCODER_MODEL", "SITE_REPORT_KIND", "SiteBundle", "SiteRefused", "check_site",
           "read_site_map", "resolve_site"]
