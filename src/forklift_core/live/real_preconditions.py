"""S10's preconditions for the operating REAL assembly (docs/plans/2026-11-26-real-operating-assembly.md 2.4, 3).

``operating_preconditions(facts)`` is the one judgement the assembly and the console share: a pure function of the
``Facts`` gathered from the settings, the board and the record file, returning ``precondition:<C>:<item>:<why>``
reasons (empty: every precondition holds). The record file ``config/real/s10_preconditions.yaml`` keeps what only a
person can confirm; every item is bound (``binds``) to what it was measured on -- the board's USB serial and product
string, the stop and timing files actually read, the vehicle calibration, the lift settings, the S10 limits -- so a
later change voids it. The settings themselves (LiDAR bound and mount, provenance of the vehicle, layer and watch
values) are read directly, never copied into the record.

``UNLOCK_INCOMPLETE`` stays in the list until the last unlock PR (U3) removes it: until then the operating assembly is
refused whatever the record says (plan 2.4, 6).

IMU qualification (C7) is not an assembly refusal: it is returned apart (``imu_qualified``) for the runtime readiness
(``imu:unqualified``, bypassable for the session, plan 2.7)."""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
RECORD_YAML = REPO / "config" / "real" / "s10_preconditions.yaml"
SCHEMA = "s10_preconditions/1"
UNLOCK_INCOMPLETE = "precondition:unlock_incomplete"
STATUSES = ("pass", "approved_limited")

# record item -> (precondition, the binds it must carry); every item needs status, evidence and date
ITEMS = {
    "s5_safety": ("C2", ("board_serial", "product", "stop_files")),
    "iwdg_evidence": ("C2", ("commit", "board_serial")),
    "fault_detection": ("C3", ("board_serial", "product")),
    "port_review": ("C4", ()),
    "scan_load": ("C5", ("lidar_sha256", "scan_image")),
    "scan_coverage": ("C5", ("lidar_sha256",)),
    "s10_speed": ("C9", ("approved_max_speed_mps", "tested_speed_mps")),
    "start_taped": ("C10", ("start_rear",)),
    "lift": ("C11", ("board_serial", "lift_target_m", "lift_rate_mps", "test_pallet", "lift_mechanism")),
    "firmware_build": ("C12", ("product", "firmware_vehicle_calibration_id", "readback_sha256",
                               "board_lift_rate_max_mps")),
    "stop_model": ("C13", ("stop_files",)),
    "jetson_load": ("C14", ("stop_files",)),
}
# the files whose values the stop model, the timing and the watches read (plan 2.4 binds)
STOP_FILES = ("layer", "budget", "odometry_age", "vehicle", "watch")
PROVENANCE_FILES = ("vehicle", "budget", "watch")  # their measured: false entries need a measurement or an exception
# entries the autonomy never reads (the teleop bridge's own): not S10 preconditions
TELEOP_ONLY = {("vehicle", "teleop_speed_limit"), ("watch", "tilt")}
START_TOL_M = 0.001  # the taped start written in the record is the S10-A one (a copy, not a tolerance)


@dataclass
class Facts:
    """Everything the judgement reads, gathered once (``gather``) so the judgement stays pure."""

    record: dict | None  # the parsed record file (None: unreadable -- ``record_error`` says why)
    record_error: str | None
    repo: Path
    lidar: dict  # lidar.yaml as parsed
    lidar_sha256: str
    imu: dict  # imu.yaml as parsed
    settings: dict  # autonomy.yaml as parsed
    s10a: object  # real_autonomy.S10AConfig
    stop_files: dict  # name -> sha256 (STOP_FILES)
    provenance: dict  # name -> [(key path, provenance dict)] for PROVENANCE_FILES
    vehicle_calibration_id: str | None
    board: dict = field(default_factory=dict)  # usb_serial, usb_product, drive_refusal (None: none), first STATE seen


def _sha(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _provenances(node, path=()) -> list:
    """Every ``provenance`` mapping in a parsed YAML with the key path to it."""
    out = []
    if isinstance(node, dict):
        for k, v in node.items():
            if k == "provenance" and isinstance(v, dict):
                out.append((".".join(map(str, path)), v))
            else:
                out += _provenances(v, path + (k,))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            out += _provenances(v, path + (i,))
    return out


def load_record(path=RECORD_YAML) -> tuple:
    """``(record, None)`` or ``(None, why)``: the schema and every item's shape (the values are judged later)."""
    try:
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as err:
        return None, f"unreadable:{type(err).__name__}"
    if not isinstance(raw, dict) or raw.get("schema") != SCHEMA:
        return None, "schema"
    items = raw.get("items")
    if not isinstance(items, dict) or set(items) - set(ITEMS):
        return None, "items"
    for name, item in items.items():
        if item is not None and not isinstance(item, dict):
            return None, f"item_shape:{name}"
    exc = raw.get("config_exceptions") or []
    if not isinstance(exc, list) or any(not isinstance(e, dict) or set(e) != {"file", "key", "decision_id"}
                                        for e in exc):
        return None, "config_exceptions"
    return raw, None


def gather(*, record_path=RECORD_YAML, lidar_path, imu_path, settings_path, s10a_path, layer_path, budget_path,
           vehicle_path, watch_path, repo=REPO, board=None) -> Facts:
    """The facts for ``operating_preconditions`` (the paths are the run's; ``board`` the first STATE's facts)."""
    from forklift_core.live.real_autonomy import load_s10a
    from forklift_core.plant.real_vehicle import load_real_vehicle

    record, record_error = load_record(record_path)
    lidar = yaml.safe_load(Path(lidar_path).read_text(encoding="utf-8"))
    layer = yaml.safe_load(Path(layer_path).read_text(encoding="utf-8"))
    odo = layer.get("odometry_age")
    odo_path = (Path(repo) / odo) if odo else None
    files = {"layer": layer_path, "budget": budget_path, "vehicle": vehicle_path, "watch": watch_path}
    stop = {name: _sha(p) for name, p in files.items()}
    stop["odometry_age"] = _sha(odo_path) if odo_path is not None and odo_path.is_file() else None
    prov = {name: _provenances(yaml.safe_load(Path(files[name]).read_text(encoding="utf-8")))
            for name in PROVENANCE_FILES}
    try:
        calib = load_real_vehicle(vehicle_path).calibration_id
    except (OSError, ValueError, KeyError, TypeError):
        calib = None
    return Facts(record=record, record_error=record_error, repo=Path(repo), lidar=lidar or {},
                 lidar_sha256=_sha(lidar_path), imu=yaml.safe_load(Path(imu_path).read_text(encoding="utf-8")) or {},
                 settings=yaml.safe_load(Path(settings_path).read_text(encoding="utf-8")) or {},
                 s10a=load_s10a(s10a_path), stop_files=stop, provenance=prov, vehicle_calibration_id=calib,
                 board=dict(board or {}))


# --- the judgement ---------------------------------------------------------------------------------------------------
def _num_eq(a, b, tol=1e-9) -> bool:
    try:
        return abs(float(a) - float(b)) <= tol
    except (TypeError, ValueError):
        return False


def _item_reasons(f: Facts, name: str) -> list:
    """The record item's own shape, its evidence and its binds against the facts."""
    c, need = ITEMS[name]
    tag = f"precondition:{c}:{name}"
    item = (f.record.get("items") or {}).get(name)
    if item is None or item.get("status") is None:
        return [f"{tag}:unset"]
    if item.get("status") not in STATUSES:
        return [f"{tag}:status"]
    if item.get("status") == "approved_limited" and not item.get("decision_id"):
        return [f"{tag}:decision_id"]
    ev = item.get("evidence")
    if not isinstance(ev, str) or not ev or Path(ev).is_absolute() or ".." in Path(ev).parts \
            or not (f.repo / ev).is_file():
        return [f"{tag}:evidence"]
    if not item.get("date"):
        return [f"{tag}:date"]
    binds = item.get("binds") or {}
    missing = [k for k in need if k not in binds]
    if missing:
        return [f"{tag}:binds_missing:{missing[0]}"]
    b = f.board
    out = []
    for k in need:
        v = binds[k]
        if k == "board_serial" and v != b.get("usb_serial"):
            out.append(f"{tag}:board_serial")
        elif k == "product" and v != b.get("usb_product"):
            out.append(f"{tag}:product")
        elif k == "stop_files" and (not isinstance(v, dict) or any(v.get(n) != f.stop_files.get(n)
                                                                    for n in STOP_FILES)):
            out.append(f"{tag}:stop_files")
        elif k == "lidar_sha256" and v != f.lidar_sha256:
            out.append(f"{tag}:lidar_sha256")
        elif k == "lift_target_m" and not _num_eq(v, f.settings.get("lift_target_m")):
            out.append(f"{tag}:lift_target_m")
        elif k == "lift_rate_mps" and not _num_eq(v, f.settings.get("lift_rate_mps")):
            out.append(f"{tag}:lift_rate_mps")
        elif k == "test_pallet" and v != f.s10a.pallet_id:
            out.append(f"{tag}:test_pallet")
        elif k == "start_rear" and (not isinstance(v, (list, tuple)) or len(v) != 3
                                    or any(not _num_eq(a, e, START_TOL_M) for a, e in zip(v, f.s10a.start_rear))):
            out.append(f"{tag}:start_rear")
        elif k == "firmware_vehicle_calibration_id" and v != f.vehicle_calibration_id:
            out.append(f"{tag}:firmware_vehicle_calibration_id")
    return out


def _c1_board(f: Facts) -> list:
    b = f.board
    out = []
    if not b.get("state_seen"):
        out.append("precondition:C1:board:no_state")
    elif b.get("drive_refusal"):
        out.append(f"precondition:C1:board:{b['drive_refusal']}")
    product = b.get("usb_product")
    if not product:
        out.append("precondition:C1:board:no_usb_product")
    elif "-dirty" in str(product):
        out.append("precondition:C1:board:dirty_image")
    if not b.get("usb_serial"):
        out.append("precondition:C1:board:no_usb_serial")
    return out


def _c5_scan(f: Facts) -> list:
    t = f.lidar.get("timing") or {}
    m = f.lidar.get("mount") or {}
    out = []
    if t.get("first_beam_err_s") is None:
        out.append("precondition:C5:lidar:first_beam_err_s")
    if t.get("basis") is None:
        out.append("precondition:C5:lidar:basis")
    if not t.get("applies_to"):
        out.append("precondition:C5:lidar:applies_to")
    doc = t.get("basis_doc")
    if t.get("basis") is not None and (not doc or not (f.repo / str(doc)).is_file()):
        out.append("precondition:C5:lidar:basis_doc")
    if m.get("measured") is not True:
        out.append("precondition:C5:lidar:mount_unmeasured")
    return out


def _provenance_reasons(f: Facts) -> list:
    """C12 (vehicle) / C13 (layer budget, watches): an unmeasured value needs a lead exception in the record."""
    exc = {(e["file"], e["key"]) for e in (f.record or {}).get("config_exceptions") or () if e.get("decision_id")}
    out = []
    for name in PROVENANCE_FILES:
        c = "C12" if name == "vehicle" else "C13"
        for key, prov in f.provenance.get(name, ()):
            if prov.get("measured") is not True and (name, key) not in exc and (name, key) not in TELEOP_ONLY:
                out.append(f"precondition:{c}:{name}:{key or '-'}:unmeasured")
    return out


def _limits(f: Facts) -> list:
    """C9 speed against its approval, C11 lift rate against the board's limit (values in the record's binds)."""
    out = []
    items = (f.record or {}).get("items") or {}
    sp = (items.get("s10_speed") or {}).get("binds") or {}
    approved, tested = sp.get("approved_max_speed_mps"), sp.get("tested_speed_mps")
    try:
        if approved is not None and float(f.s10a.max_speed_mps) > float(approved) + 1e-9:
            out.append("precondition:C9:s10_speed:over_approved")
        if approved is not None and tested is not None and float(approved) > float(tested) + 1e-9:
            out.append("precondition:C9:s10_speed:over_tested")
    except (TypeError, ValueError):
        out.append("precondition:C9:s10_speed:values")
    fw = (items.get("firmware_build") or {}).get("binds") or {}
    board_max = fw.get("board_lift_rate_max_mps")
    rate = f.settings.get("lift_rate_mps")
    try:
        if board_max is not None and (rate is None or float(rate) > float(board_max) + 1e-9):
            out.append("precondition:C11:lift_rate_over_board")
    except (TypeError, ValueError):
        out.append("precondition:C11:lift_rate_values")
    lift = items.get("lift") or {}
    if lift.get("status") is not None:
        out += _lift_checks(f, lift)
    return out


def _finite(*vals) -> bool:
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in vals)


def _lift_checks(f: Facts, lift: dict) -> list:
    """C11 (plan 2.10): the hold at the target height with no support, under a lead-approved sag limit, and the
    origin/scale against a tape (the host's ``lift_origin_ok`` only means a finite, in-range reading)."""
    out = []
    target = f.settings.get("lift_target_m")
    rows = lift.get("hold") or []
    limit = lift.get("sag_limit_cm")
    if not isinstance(rows, list) or not rows or not _finite(limit, target) \
            or any(not isinstance(r, dict) or not _finite(r.get("raised_cm"), r.get("sag_cm_after_60s")) for r in rows):
        out.append("precondition:C11:lift:hold_values")
    else:
        top = float(target) * 100.0
        if not any(r["raised_cm"] >= top - 0.5 for r in rows):
            out.append("precondition:C11:lift:hold_no_target_height")
        doc = lift.get("self_locking_evidence")
        has_doc = isinstance(doc, str) and bool(doc) and not Path(doc).is_absolute() and ".." not in Path(doc).parts \
            and (f.repo / doc).is_file()
        worst = [r for r in rows if r.get("worst") is True and isinstance(r.get("worst_reason"), str)
                 and r["worst_reason"].strip() and 0.0 < r["raised_cm"] <= top + 0.5]
        if not has_doc and not worst:
            # a stop mid-lift is a DISARM too: the height where holding is weakest, chosen from the mechanism with its
            # reason written down (Codex plan v8 review rounds 2-3)
            out.append("precondition:C11:lift:hold_no_worst_height")
        if any(r["sag_cm_after_60s"] > limit for r in rows):
            out.append("precondition:C11:lift:sag_over_limit")
    if not lift.get("sag_limit_decision_id"):
        out.append("precondition:C11:lift:sag_limit_decision_id")
    if not (lift.get("binds") or {}).get("lift_mechanism"):
        out.append("precondition:C11:lift:lift_mechanism")
    org = lift.get("origin") or {}
    floor, tape, at_target = (org.get("state_lift_m_at_floor"), org.get("tape_cm_at_target"),
                              org.get("state_lift_m_at_target"))
    if not _finite(floor, tape, at_target, target):
        out.append("precondition:C11:lift:origin_values")
    else:
        if abs(floor) > 0.005:
            out.append("precondition:C11:lift:origin_floor")
        if abs(tape / 100.0 - at_target) > 0.01:
            out.append("precondition:C11:lift:origin_scale")
        if abs(at_target - float(target)) > 0.005:
            out.append("precondition:C11:lift:origin_target")
    return out


def imu_qualified(f: Facts, opened_profile_fits: bool | None = None) -> bool:
    """C7 (plan 2.7): imu.yaml measured with a delay bound, the opened profile fitting the watch (None: not opened
    yet -- not qualified), the watch's level reference measured. Not an assembly refusal."""
    t = f.imu.get("timing") or {}
    level = [prov for key, prov in f.provenance.get("watch", ()) if key == "imu"]  # safety_watch.yaml imu.level_ref
    return bool(t.get("measured") is True and t.get("delay_bound_s") is not None and opened_profile_fits is True
                and level and all(prov.get("measured") is True for prov in level))


def operating_preconditions(f: Facts) -> list:
    """Every reason the operating REAL assembly is refused (plan 3: the non-bypassable rows). Empty: it may."""
    out = [UNLOCK_INCOMPLETE]
    out += _c1_board(f)
    out += _c5_scan(f)
    if f.record is None:
        out.append(f"precondition:record:{f.record_error}")
    else:
        for name in ITEMS:
            out += _item_reasons(f, name)
        out += _limits(f)
    out += _provenance_reasons(f)
    if f.stop_files.get("odometry_age") is None:
        out.append("precondition:C13:odometry_age:missing")
    return out


__all__ = ["Facts", "ITEMS", "RECORD_YAML", "SCHEMA", "STOP_FILES", "UNLOCK_INCOMPLETE", "gather", "imu_qualified",
           "load_record", "operating_preconditions"]
