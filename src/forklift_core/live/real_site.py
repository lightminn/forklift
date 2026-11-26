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
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

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
    name = "slam_bundle_registry"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, REPO / "tools/slam_bundle_registry.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
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
    try:
        row = _registry_module().lookup_site(book, rec_sha, encoder_model)
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
    return SiteBundle(row=row, bundle=root / folder, report=report,
                      hashes={"site_record": rec_sha, "registry": hashlib.sha256(raw).hexdigest(),
                              "site_report": rep["sha256"]})


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
    ok_start = (isinstance(start, list) and len(start) == 3
                and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in start))
    if not ok_start:
        bad("start_pose")
    else:
        x, y, yaw = (float(v) for v in start)
        rear = (x - rear_offset_m * math.cos(yaw), y - rear_offset_m * math.sin(yaw), yaw)  # start_reloc.rear_from_base
        tape = [float(v) for v in start_rear]
        d = math.hypot(rear[0] - tape[0], rear[1] - tape[1])
        dyaw = abs(math.remainder(rear[2] - tape[2], math.tau))
        if d > tol_m or dyaw > tol_rad:
            bad("start")
    return out


__all__ = ["PREFIX", "REAL_ENCODER_MODEL", "SITE_REPORT_KIND", "SiteBundle", "SiteRefused", "check_site", "resolve_site"]
