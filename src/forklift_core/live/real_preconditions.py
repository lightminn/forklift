"""S10's preconditions for the operating REAL assembly (docs/plans/2026-11-26-real-operating-assembly.md 2.4, 2.10, 3).

``operating_preconditions(facts)`` is the one judgement the assembly and the console share: a pure function of the
``Facts`` gathered from the settings, the board and the record file, returning ``precondition:<C>:<item>:<why>``
reasons (empty: every precondition holds). The record file ``config/real/s10_preconditions.yaml`` keeps what only a
person can confirm; every item is bound (``binds``) to what it was measured on -- the board's USB serial and product
string, the stop and timing files actually read, the vehicle calibration, the lift settings and load, the S10 limits and
placement, the scan image -- so a later change voids it. The settings themselves (LiDAR bound and mount, provenance of
the vehicle, layer and watch values) are read directly, never copied into the record.

Every file is parsed from the very bytes its sha256 is taken of; a loader that reads a path itself (the S10-A config,
the vehicle with its chassis model, the LiDAR validator) is built from those same bytes (U3e-2); ``files_changed`` stays
for a change the caller reports. The assembly also compares these hashes with the ones ``prepare_run`` loaded.

The unlock's lock (``precondition:unlock_incomplete``, plan 2.4) was removed by its last PR (U3e-2): an empty list now
lets the operating assembly go on.

IMU qualification (C7) is not an assembly refusal: it is returned apart (``imu_qualified``) for the runtime readiness
(``imu:unqualified``, bypassable for the session, plan 2.7); the watch's ``imu`` provenance is C7's, not C13's."""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field
from pathlib import Path

import xml.etree.ElementTree as ET

import yaml

REPO = Path(__file__).resolve().parents[3]
RECORD_YAML = REPO / "config" / "real" / "s10_preconditions.yaml"
SCHEMA = "s10_preconditions/1"
STATUSES = ("pass", "approved_limited")
# items a limited approval cannot stand in for: C2's safety measurements must pass (S10 procedure, Codex U3e-2 P1)
PASS_ONLY = frozenset({"s5_safety", "iwdg_evidence"})
CORRIDOR_HALF_M = 0.10  # plan 2.4: the approved corridor's half width (the coverage record is bound to it)
APPROVED_CASES = ["A"]  # S10: case A only (plan 2.4)

# record item -> (precondition, the binds it must carry); every item needs status, evidence and date
ITEMS = {
    "s5_safety": ("C2", ("board_serial", "product", "stop_files")),
    "iwdg_evidence": ("C2", ("commit", "board_serial")),
    "fault_detection": ("C3", ("board_serial", "product")),
    "port_review": ("C4", ()),
    "scan_load": ("C5", ("lidar_sha256", "scan_image")),
    "scan_coverage": ("C5", ("lidar_sha256", "start_rear", "check_pallet_pose", "corridor_half_width_m", "map_id",
                             "inserted_offset_m")),
    "s10_speed": ("C9", ("approved_max_speed_mps", "tested_speed_mps", "approved_cases")),
    "start_taped": ("C10", ("start_rear",)),
    "lift": ("C11", ("board_serial", "lift_target_m", "lift_rate_mps", "test_pallet", "lift_mechanism",
                     "pallet_mass_kg")),
    "firmware_build": ("C12", ("product", "firmware_vehicle_calibration_id", "readback_sha256",
                               "board_lift_rate_max_mps")),
    # the stop model's latency takes lidar.yaml's scan_period_s too: its approval is bound to that file (U3e-1)
    "stop_model": ("C13", ("stop_files", "lidar_sha256")),
    "jetson_load": ("C14", ("stop_files",)),
}
# the preconditions the assembly judges (plan 3): every one but the run-time C7 (IMU) and C8 (D5), which readiness and
# the submit judge -- an assembled operating run passed these (the console's summary, plan 2.9)
ASSEMBLY_CHECKS = ("C1", "C2", "C3", "C4", "C5", "C6", "C9", "C10", "C11", "C12", "C13", "C14")
# the files whose values the stop model, the timing and the watches read (plan 2.4 binds)
STOP_FILES = ("layer", "budget", "odometry_age", "vehicle", "watch")
# their measured: false entries need a measurement or an exception; "layer" (obstacle_layer_real.yaml): the stop
# model the run's layer uses (permission.latency_s / decel_mps2, C13 -- operating plan 6.1, U3e-1)
PROVENANCE_FILES = ("vehicle", "budget", "watch", "layer")
# entries the autonomy never reads as a C12/C13 precondition: the teleop bridge's own, and the IMU watch (C7, runtime)
NOT_C12_C13 = {("vehicle", "teleop_speed_limit"), ("watch", "tilt"), ("watch", "imu")}
# entries that must carry a provenance at all (a deleted or null provenance is not a pass)
REQUIRED_PROVENANCE = {
    "vehicle": ("wheel_counts.left", "wheel_counts.right", "wheel_radius", "steering_map", "wheel_rate_limit",
                "steering_control"),
    "budget": ("latency_budget", "overdue_end"),
    "watch": ("current", "wheel"),
    "layer": ("permission.stop_model.latency_s", "permission.stop_model.decel_mps2"),
}
GATHER_ERRORS = (OSError, ValueError, KeyError, TypeError, AttributeError, UnicodeDecodeError, yaml.YAMLError,
                 ET.ParseError)
REQUIRED_HASHES = ("s10a", "settings", "vehicle", "watch", "layer", "budget", "lidar", "imu", "chassis_params",
                   "chassis_urdf")
HASHED = ("s10a", "settings", "vehicle", "watch", "layer", "budget", "lidar", "imu")
START_TOL_M = 0.001  # the taped start written in the record is the S10-A one (a copy, not a tolerance)


@dataclass
class Facts:
    """Everything the judgement reads, gathered once (``gather``) so the judgement stays pure."""

    record: dict | None  # the parsed record file (None: unreadable -- ``record_error`` says why)
    record_error: str | None
    repo: Path
    lidar: dict  # lidar.yaml as parsed
    lidar_sha256: str
    lidar_error: str | None  # the LiDAR loader's own validation (None: valid)
    imu: dict  # imu.yaml as parsed
    settings: dict  # autonomy.yaml as parsed
    s10a: object  # real_autonomy.S10AConfig
    stop_files: dict  # name -> sha256 (STOP_FILES)
    provenance: dict  # name -> [(key path, provenance value)] for PROVENANCE_FILES
    vehicle_calibration_id: str | None
    inserted_offset_m: float | None = None  # the corridor's end: the insertion end's rear axle behind the pallet centre
    file_hashes: dict = field(default_factory=dict)  # name -> sha256 (HASHED), from the bytes parsed
    changed: list = field(default_factory=list)  # files that changed while gathered
    board: dict = field(default_factory=dict)  # usb_serial, usb_product, drive_refusal (None: none), first STATE seen
    layer_permission: dict = field(default_factory=dict)  # obstacle_layer_real.yaml permission: (the stop model, C13)
    budget: dict = field(default_factory=dict)  # obstacle_layer.yaml as parsed (the latency budget, C13)


def _read(path) -> tuple:
    """(bytes, sha256, parsed YAML) from one read."""
    data = Path(path).read_bytes()
    return data, hashlib.sha256(data).hexdigest(), yaml.safe_load(data.decode("utf-8"))


def _sha(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _sha_or_none(path) -> str | None:
    try:
        return _sha(path)
    except OSError:
        return None


def _provenances(node, path=()) -> list:
    """Every ``provenance`` key in a parsed YAML with the key path to it (its value as is: a null is kept)."""
    out = []
    if isinstance(node, dict):
        for k, v in node.items():
            if k == "provenance":
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
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as err:
        return None, f"unreadable:{type(err).__name__}"
    if not isinstance(raw, dict) or raw.get("schema") != SCHEMA:
        return None, "schema"
    items = raw.get("items")
    if not isinstance(items, dict) or set(items) - set(ITEMS):
        return None, "items"
    for name, item in items.items():
        if item is None:
            continue
        if not isinstance(item, dict):
            return None, f"item_shape:{name}"
        if "binds" in item and not isinstance(item["binds"], dict):
            return None, f"binds_shape:{name}"
    lift = items.get("lift") or {}
    if "origin" in lift and not isinstance(lift["origin"], dict):
        return None, "origin_shape:lift"
    exc = raw.get("config_exceptions") or []
    if not isinstance(exc, list) or any(
            not isinstance(e, dict) or set(e) != {"file", "key", "decision_id"} or e["file"] not in PROVENANCE_FILES
            or not isinstance(e["key"], str) or not isinstance(e["decision_id"], str) or not e["decision_id"]
            for e in exc):
        return None, "config_exceptions"
    return raw, None


def gather(*, record_path=RECORD_YAML, lidar_path, imu_path, settings_path, s10a_path, layer_path, budget_path,
           vehicle_path, watch_path, repo=REPO, board=None) -> Facts:
    """The facts for ``operating_preconditions`` (the paths are the run's; ``board`` the first STATE's facts). Raises
    on an unreadable settings file (``GATHER_ERRORS``): the caller turns it into a reason."""

    record, record_error = load_record(record_path)
    paths = {"s10a": s10a_path, "settings": settings_path, "vehicle": vehicle_path, "watch": watch_path,
             "layer": layer_path, "budget": budget_path, "lidar": lidar_path, "imu": imu_path}
    parsed, hashes = {}, {}
    for name, p in paths.items():
        _data, hashes[name], parsed[name] = _read(p)
    changed = []

    # parsed from the very bytes hashed above (U3e-2 Codex round 3 P2-1: a re-read could judge a middle version)
    from forklift_core.live.real_autonomy import s10a_from_mapping
    from forklift_core.plant.real_vehicle import real_vehicle_from_mapping
    from forklift_core.sensors.lidar_cfg import lidar_config_from

    from forklift_core.plant.real_vehicle import chassis_path_from

    s10a = s10a_from_mapping(parsed["s10a"])
    try:  # one chassis path, from the vehicle bytes hashed, for its parameters and its URDF (Codex U3e-2 r4 P2-1)
        params = chassis_path_from(vehicle_path, parsed["vehicle"])
    except (ValueError, TypeError):
        params = None
    try:
        chassis_raw = params.read_bytes()
        hashes["chassis_params"] = hashlib.sha256(chassis_raw).hexdigest()
        calib = real_vehicle_from_mapping(parsed["vehicle"], yaml.safe_load(chassis_raw.decode("utf-8"))).calibration_id
    except (OSError, ValueError, KeyError, TypeError, AttributeError, yaml.YAMLError):
        calib = None  # params None (no chassis named) lands here too
    try:
        lidar_config_from(parsed["lidar"])
        lidar_error = None
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as err:
        lidar_error = f"{type(err).__name__}"
    inserted = None
    try:
        from forklift_core.live.real_run import mission_geometry

        urdf = params.parent / "forklift.urdf"
        hashes.setdefault("chassis_params", _sha_or_none(params))
        hashes["chassis_urdf"] = None  # unread: a missing fact, never compared away
        urdf_raw = urdf.read_bytes()  # hashed and parsed once (U3e-2 Codex round 3)
        hashes["chassis_urdf"] = hashlib.sha256(urdf_raw).hexdigest()
        if s10a.pallet_depth_m is not None and s10a.pallet_width_m is not None:
            inserted = float(mission_geometry(urdf_raw, s10a.pallet_depth_m, s10a.pallet_width_m).inserted_offset_m)
    except (OSError, ValueError, KeyError, TypeError, AttributeError, ET.ParseError):
        inserted = None
    layer = parsed["layer"] if isinstance(parsed["layer"], dict) else {}
    odo = layer.get("odometry_age")
    odo_path = (Path(repo) / odo) if odo else None
    stop = {name: hashes[name] for name in ("layer", "budget", "vehicle", "watch")}
    stop["odometry_age"] = _sha(odo_path) if odo_path is not None and odo_path.is_file() else None
    prov = {name: _provenances(parsed[name] if isinstance(parsed[name], dict) else {}) for name in PROVENANCE_FILES}
    return Facts(record=record, record_error=record_error, repo=Path(repo),
                 lidar=parsed["lidar"] if isinstance(parsed["lidar"], dict) else {},
                 lidar_sha256=hashes["lidar"], lidar_error=lidar_error,
                 imu=parsed["imu"] if isinstance(parsed["imu"], dict) else {},
                 settings=parsed["settings"] if isinstance(parsed["settings"], dict) else {}, s10a=s10a,
                 stop_files=stop, provenance=prov, vehicle_calibration_id=calib, inserted_offset_m=inserted,
                 file_hashes=hashes, changed=changed, board=dict(board or {}),
                 layer_permission=dict((layer.get("permission") or {}) if isinstance(layer.get("permission"), dict)
                                       else {}),
                 budget=parsed["budget"] if isinstance(parsed["budget"], dict) else {})


# --- the judgement ---------------------------------------------------------------------------------------------------
def _named(v) -> bool:
    """A decision id: a non-blank string (Codex U3e-1 P2-5 for exceptions, U3e-2 P2-2 for C4 and C11)."""
    return isinstance(v, str) and bool(v.strip())


def _finite(*vals) -> bool:
    try:
        return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in vals)
    except OverflowError:  # an integer no float can hold (Codex U3e-1 impl round 2 P3)
        return False


def _positive(*vals) -> bool:
    return _finite(*vals) and all(v > 0 for v in vals)


def _num_eq(a, b, tol=1e-9) -> bool:
    return _finite(a, b) and abs(float(a) - float(b)) <= tol


def _inside_repo(f: Facts, rel) -> bool:
    """A repo-relative path to an existing file whose resolved location (links followed) stays in the repository."""
    if not isinstance(rel, str) or not rel or Path(rel).is_absolute():
        return False
    try:
        target = (f.repo / rel).resolve(strict=True)
    except (OSError, RuntimeError):
        return False
    root = f.repo.resolve()
    return target.is_file() and root in target.parents


def _pose_eq(v, pose, tol) -> bool:
    return isinstance(v, (list, tuple)) and pose is not None and len(v) == 3 and len(pose) == 3 \
        and all(_num_eq(a, e, tol) for a, e in zip(v, pose))


def _is_hex(v, n) -> bool:
    return isinstance(v, str) and len(v) == n and all(c in "0123456789abcdef" for c in v)


def _bind_bad(f: Facts, k: str, v) -> bool:
    """Whether one bind does not match what it must be now."""
    b = f.board
    lidar_t = f.lidar.get("timing") if isinstance(f.lidar.get("timing"), dict) else {}
    if k == "board_serial":
        return not v or v != b.get("usb_serial")
    if k == "product":
        return not v or v != b.get("usb_product")
    if k == "stop_files":
        return not isinstance(v, dict) or any(not _is_hex(v.get(n), 64) or v.get(n) != f.stop_files.get(n)
                                              for n in STOP_FILES)
    if k == "lidar_sha256":
        return v != f.lidar_sha256
    if k == "scan_image":
        return not v or v != (lidar_t.get("applies_to") or {}).get("image_id")
    if k == "lift_target_m":
        return not _num_eq(v, f.settings.get("lift_target_m"))
    if k == "lift_rate_mps":
        return not _num_eq(v, f.settings.get("lift_rate_mps"))
    if k == "test_pallet":
        return v != f.s10a.pallet_id
    if k == "lift_mechanism":
        return not isinstance(v, str) or not v.strip()
    if k in ("pallet_mass_kg", "board_lift_rate_max_mps", "approved_max_speed_mps", "tested_speed_mps"):
        return not _positive(v)
    if k == "start_rear":
        return not _pose_eq(v, f.s10a.start_rear, START_TOL_M)
    if k == "check_pallet_pose":
        return not _pose_eq(v, f.s10a.check_pallet_pose, START_TOL_M)
    if k == "corridor_half_width_m":
        return not _num_eq(v, CORRIDOR_HALF_M)
    if k == "map_id":
        return not v or v != f.s10a.map_id
    if k == "inserted_offset_m":
        return f.inserted_offset_m is None or not _num_eq(v, f.inserted_offset_m, 0.001)
    if k == "firmware_vehicle_calibration_id":
        return not v or v != f.vehicle_calibration_id
    if k == "readback_sha256":
        return not _is_hex(v, 64)
    if k == "commit":
        # the IWDG evidence's commit is the final image's: the product string's git describe abbreviation, whole
        abbrev = image_commit(b.get("usb_product"))
        return not (abbrev and isinstance(v, str) and len(v) >= len(abbrev)
                    and all(ch in "0123456789abcdef" for ch in v) and v.startswith(abbrev))
    if k == "approved_cases":
        return v != APPROVED_CASES
    return True  # an unknown bind never passes


def image_commit(product) -> str | None:
    """The commit abbreviation in a USB product string ``forklift-mcu <kind> <outputs> <git describe>...`` (the
    firmware's ``git describe --always --dirty --abbrev=12 --long``: a bare abbreviation when no tag is reachable,
    else ``<tag>-<n>-g<abbrev>`` even on the tagged commit itself -- the abbreviation after the last ``-g``, whatever
    the tag is named); None when absent, dirty or not hex. An image built before ``--long`` (U3d) from an exactly
    tagged commit shows the bare tag: refused unless the tag itself looks like the long form or like a bare 12-40 hex
    abbreviation (``deadbeefdead``), which only the C12 record's exact product string and the operator's readback
    comparison then catch."""
    import re

    parts = str(product or "").split()
    if len(parts) < 4 or parts[0] != "forklift-mcu":
        return None
    d = parts[3]
    m = re.fullmatch(r"(?:.+-\d+-g)?([0-9a-f]{12,40})", d)
    # a dirty build, or a pre-U3d exactly tagged build ("<tag>" alone) whose tag looks like neither form, carries no
    # checkable abbreviation: refused (safe side) -- see the docstring for the tags that do look like one
    return m.group(1) if m else None


def _item_reasons(f: Facts, name: str) -> list:
    """The record item's own shape, its evidence and its binds against the facts."""
    c, need = ITEMS[name]
    tag = f"precondition:{c}:{name}"
    item = (f.record.get("items") or {}).get(name)
    if item is None or item.get("status") is None:
        return [f"{tag}:unset"]
    if item.get("status") not in STATUSES or (name in PASS_ONLY and item.get("status") != "pass"):
        return [f"{tag}:status"]
    if item.get("status") == "approved_limited" and not _named(item.get("decision_id")):
        return [f"{tag}:decision_id"]
    if not _inside_repo(f, item.get("evidence")):
        return [f"{tag}:evidence"]
    if not item.get("date"):
        return [f"{tag}:date"]
    binds = item.get("binds") or {}
    missing = [k for k in need if k not in binds]
    if missing:
        return [f"{tag}:binds_missing:{missing[0]}"]
    return [f"{tag}:{k}" for k in need if _bind_bad(f, k, binds[k])]


def _c1_board(f: Facts) -> list:
    b = f.board
    out = []
    if not b.get("state_seen"):
        out.append("precondition:C1:board:no_state")
    elif b.get("drive_refusal"):
        out.append(f"precondition:C1:board:{b['drive_refusal']}")
    if b.get("bad_build_seen"):
        out.append("precondition:C1:board:bad_build_seen")
    product = b.get("usb_product")
    if not product:
        out.append("precondition:C1:board:no_usb_product")
    elif "-dirty" in str(product):
        out.append("precondition:C1:board:dirty_image")
    if not b.get("usb_serial"):
        out.append("precondition:C1:board:no_usb_serial")
    return out


def _c5_scan(f: Facts) -> list:
    from forklift_core.sensors.lidar_cfg import APPLIES_KEYS, BASES

    t = f.lidar.get("timing") if isinstance(f.lidar.get("timing"), dict) else {}
    m = f.lidar.get("mount") if isinstance(f.lidar.get("mount"), dict) else {}
    out = []
    if f.lidar_error is not None:
        out.append(f"precondition:C5:lidar:invalid:{f.lidar_error}")
    err = t.get("first_beam_err_s")
    if not _finite(err) or err < 0:
        out.append("precondition:C5:lidar:first_beam_err_s")
    if t.get("basis") not in BASES:
        out.append("precondition:C5:lidar:basis")
    applies = t.get("applies_to")
    if not isinstance(applies, dict) or set(applies) != set(APPLIES_KEYS) \
            or any(v in (None, "") for v in applies.values()):
        out.append("precondition:C5:lidar:applies_to")
    if t.get("basis") is not None and not _inside_repo(f, t.get("basis_doc")):
        out.append("precondition:C5:lidar:basis_doc")
    if m.get("measured") is not True:
        out.append("precondition:C5:lidar:mount_unmeasured")
    return out


def _provenance_reasons(f: Facts) -> list:
    """C12 (vehicle) / C13 (layer budget, watches): an unmeasured or missing provenance needs a lead exception."""
    # a lead exception names its decision: a blank decision_id is none (Codex U3e-1 implementation round 1 P2-5)
    exc = {(e["file"], e["key"]) for e in (f.record or {}).get("config_exceptions") or ()
           if isinstance(e.get("decision_id"), str) and e["decision_id"].strip()}
    out = []
    for name in PROVENANCE_FILES:
        c = "C12" if name == "vehicle" else "C13"
        found = set()
        for key, prov in f.provenance.get(name, ()):
            found.add(key)
            if (name, key) in NOT_C12_C13 or (name, key) in exc:
                continue
            if not isinstance(prov, dict) or prov.get("measured") is not True:
                out.append(f"precondition:{c}:{name}:{key or '-'}:unmeasured")
        for key in REQUIRED_PROVENANCE.get(name, ()):
            if key not in found and (name, key) not in exc:
                out.append(f"precondition:{c}:{name}:{key}:provenance_missing")
    return out


def _limits(f: Facts) -> list:
    """C4/C9 speed against every approval, C11 lift rate against the board's limit (values in the record's binds)."""
    out = []
    items = (f.record or {}).get("items") or {}
    limits = []
    sp = items.get("s10_speed") or {}
    spb = sp.get("binds") or {}
    if sp.get("status") is not None and _positive(spb.get("approved_max_speed_mps"), spb.get("tested_speed_mps")):
        limits.append(float(spb["approved_max_speed_mps"]))
        if float(spb["approved_max_speed_mps"]) > float(spb["tested_speed_mps"]) + 1e-9:
            out.append("precondition:C9:s10_speed:over_tested")
    pr = items.get("port_review") or {}
    if pr.get("status") == "approved_limited":
        prb = pr.get("binds") or {}
        if not _positive(prb.get("approved_max_speed_mps")):
            out.append("precondition:C4:port_review:approved_max_speed_mps")
        else:
            limits.append(float(prb["approved_max_speed_mps"]))
        if prb.get("approved_cases") != APPROVED_CASES:
            out.append("precondition:C4:port_review:approved_cases")
    if limits and (not _finite(f.s10a.max_speed_mps) or float(f.s10a.max_speed_mps) > min(limits) + 1e-9):
        out.append("precondition:C9:s10_speed:over_approved")
    fw = items.get("firmware_build") or {}
    board_max = (fw.get("binds") or {}).get("board_lift_rate_max_mps")
    rate = f.settings.get("lift_rate_mps")
    if fw.get("status") is not None and _positive(board_max) and (not _positive(rate) or rate > board_max + 1e-9):
        out.append("precondition:C11:lift_rate_over_board")
    lift = items.get("lift") or {}
    if lift.get("status") is not None:
        out += _lift_checks(f, lift)
    return out


def _lift_checks(f: Facts, lift: dict) -> list:
    """C11 (plan 2.10): the hold at the target height and at the weakest height with no support, under a
    lead-approved sag limit, and the origin/scale against a tape (the host's ``lift_origin_ok`` only means a finite,
    in-range reading)."""
    out = []
    target = f.settings.get("lift_target_m")
    rows = lift.get("hold") or []
    limit = lift.get("sag_limit_cm")
    if not isinstance(rows, list) or not rows or not _positive(limit, target) \
            or any(not isinstance(r, dict) or not _finite(r.get("raised_cm"), r.get("sag_cm_after_60s"))
                   or r["raised_cm"] <= 0 or r["sag_cm_after_60s"] < 0 for r in rows):
        out.append("precondition:C11:lift:hold_values")
    else:
        top = float(target) * 100.0
        if not any(abs(r["raised_cm"] - top) <= 0.5 for r in rows):
            out.append("precondition:C11:lift:hold_no_target_height")
        worst = [r for r in rows if r.get("worst") is True and isinstance(r.get("worst_reason"), str)
                 and r["worst_reason"].strip() and r["raised_cm"] <= top + 0.5]
        if not _inside_repo(f, lift.get("self_locking_evidence")) and not worst:
            # a stop mid-lift is a DISARM too: the height where holding is weakest, chosen from the mechanism with its
            # reason written down (Codex plan v8 review rounds 2-3)
            out.append("precondition:C11:lift:hold_no_worst_height")
        if any(r["sag_cm_after_60s"] > limit for r in rows):
            out.append("precondition:C11:lift:sag_over_limit")
    if not _named(lift.get("sag_limit_decision_id")):
        out.append("precondition:C11:lift:sag_limit_decision_id")
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


def operator_load(f: Facts) -> dict | None:
    """What the pallet confirmation's fourth and sixth lines show (plan 2.6): ``lift_target_m`` from the settings and
    the C11 record's test pallet and mass -- the bytes the assembly judged; None when the record has no such item."""
    lift = ((f.record or {}).get("items") or {}).get("lift")
    binds = lift.get("binds") if isinstance(lift, dict) else None
    if not isinstance(binds, dict):
        return None
    target = f.settings.get("lift_target_m")
    return {"lift_target_m": float(target) if _finite(target) else None, "test_pallet": binds.get("test_pallet"),
            "pallet_mass_kg": float(binds["pallet_mass_kg"]) if _finite(binds.get("pallet_mass_kg")) else None}


def imu_qualified(f: Facts, opened_profile_fits: bool | None = None) -> bool:
    """C7 (plan 2.7): imu.yaml measured with a delay bound, the opened profile fitting the watch (None: not opened
    yet -- not qualified), the watch's level reference (its ``imu`` provenance) measured. Not an assembly refusal."""
    t = f.imu.get("timing") or {}
    level = [prov for key, prov in f.provenance.get("watch", ()) if key == "imu"]
    return bool(t.get("measured") is True and t.get("delay_bound_s") is not None and opened_profile_fits is True
                and level and all(isinstance(prov, dict) and prov.get("measured") is True for prov in level))


def _stop_model_values(f: Facts) -> list:
    """C13 (operating plan 6.1, Codex U3e-1 round 3 P1): the stop model's own values, whatever their provenance or a
    lead exception says -- the latency a finite number >= 0 and the deceleration a finite number > 0 (the stopping
    distance grows with the one and divides by the other; a bool is not a number here)."""
    out = []
    p = f.layer_permission if isinstance(f.layer_permission, dict) else {}
    for key, ok in (("latency_s", lambda v: v >= 0.0), ("decel_mps2", lambda v: v > 0.0)):
        v = p.get(key)
        if not (_finite(v) and ok(float(v))):
            out.append(f"precondition:C13:layer:{key}:invalid")
    return out


def budget_reasons(budget: dict) -> list:
    """C13 (Codex U3e-2 rounds 1-2): the latency budget and the overdue end each a finite positive number as written
    (a bool is not one; a .nan or non-positive end would never end a sustained overdue). One rule for the judgement
    and the operating preparation."""
    out = []
    for key in ("latency_budget", "overdue_end"):
        node = budget.get(key) if isinstance(budget, dict) else None
        v = node.get("seconds") if isinstance(node, dict) else None
        if not (isinstance(v, (int, float)) and not isinstance(v, bool) and _finite(v) and float(v) > 0.0):
            out.append(f"precondition:C13:budget:{key}:invalid")
    return out


def operating_preconditions(f: Facts) -> list:
    """Every reason the operating REAL assembly is refused (plan 3: the non-bypassable rows). Empty: it may."""
    out = [f"precondition:files_changed:{name}" for name in f.changed]
    # every file the judgement binds must have been read (a missing hash is never compared away)
    out += [f"precondition:facts_missing:{name}" for name in REQUIRED_HASHES if not f.file_hashes.get(name)]
    out += _c1_board(f)
    out += _c5_scan(f)
    if f.record is None:
        out.append(f"precondition:record:{f.record_error}")
    else:
        for name in ITEMS:
            try:
                out += _item_reasons(f, name)
            except (TypeError, AttributeError, ValueError):
                out.append(f"precondition:{ITEMS[name][0]}:{name}:malformed")
        try:
            out += _limits(f)
        except (TypeError, AttributeError, ValueError):
            out.append("precondition:record:limits_malformed")
    out += _provenance_reasons(f)
    out += _stop_model_values(f)
    out += budget_reasons(f.budget)
    if f.stop_files.get("odometry_age") is None:
        out.append("precondition:C13:odometry_age:missing")
    return out


__all__ = ["Facts", "GATHER_ERRORS", "HASHED", "ITEMS", "image_commit", "RECORD_YAML", "SCHEMA", "STOP_FILES", "gather",
           "imu_qualified", "load_record", "operating_preconditions"]
