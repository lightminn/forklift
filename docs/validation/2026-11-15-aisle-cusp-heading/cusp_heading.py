"""Baseline priority 4, records only: every aisle_rack transport cusp heading judgement (passes, failures recovered by a
cusp replan, final failures), measured and split by the rules of docs/plans/2026-11-15-aisle-cusp-heading.md (2.2, 2.3,
3; fixed before the results).

    python cusp_heading.py --fields FILE.json.gz --out OUT.json                 # after the fix (K1 re-check, primary)
    python cusp_heading.py --raw-root DIR [--raw-root DIR ...] --out OUT.json
    python cusp_heading.py --fields tdep_fields.json.gz --period prefix --out OUT.json   # before the fix (records only)

--raw-root DIR holds <name>/run/{result.json, slam_control.npy}. --fields is {key: {"result": {...},
"slam_control_rows": [[t, est x, y, yaw, true x, y, yaw], ...] | "slam_control_rows_same_as": "<other key or name>",
"slam_control_sha256": ...}} (keys are "<name>" or "<root>/<name>"). Only runs whose name holds --name-filter (default
"aisle_rack") are read. Reads only.

Reuses the baseline priority 3 tool (2026-11-14-transport-departure/transport_departure.py) by path: Path2, wrap,
row_exact, saturation, primary_cross, wheel_curvatures and its constants. Its installed_paths and leg_start are NOT used
(plan 2.2: the scene replan record swaps old and new, and leg_start ignores the serial); this tool matches each tracker
serial to a recorded path by its cusp poses instead.

Period "postfix" (default) needs tracker_versions. Period "prefix" (the priority 3 fields file, no tracker_versions)
places failures on the nearest leg end of the replaced path (priority 3's method), places passes on the recorded path
whose cusp at endpoint_index has the leg switch's endpoint_pose, and leaves the serial link unresolved (plan 1).
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import importlib.util
import json
import math
import statistics
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TD_FILE = HERE.parent / "2026-11-14-transport-departure" / "transport_departure.py"
TDEP_JSON = HERE.parent / "2026-11-14-transport-departure" / "tdep.json"


def _load_td():
    spec = importlib.util.spec_from_file_location("transport_departure_p3", TD_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


td = _load_td()
Path2, wrap, row_exact, saturation, primary_cross, wheel_curvatures = (
    td.Path2, td.wrap, td.row_exact, td.saturation, td.primary_cross, td.wheel_curvatures)
SAT, S_SHARE, P_RATIO, M_MARGIN = td.SAT, td.S_SHARE, td.P_RATIO, td.M_MARGIN
O_LAT, O_YAW, SHORT_LEG_M, END_WINDOW_M, PAIR_TOL_S = td.O_LAT, td.O_YAW, td.SHORT_LEG_M, td.END_WINDOW_M, td.PAIR_TOL_S

TAU_CUSP = 0.05  # active cusp yaw tolerance (plan 2.3)
CUSP_POS_TOL_M = 0.03  # cusp position tolerance: the reconstructed judgement tick's radius (plan 2.3)
LOOKAHEAD_M = 0.28  # the end stretch of the steering residual (plan 2.3, agy 4)
POSE_TOL = 1e-6  # serial <-> path cusp pose match (plan 2.2)
TIME_TOL = 1e-6
BAND = (0.05, 0.08)  # plan 4: |e_E| between the cusp tolerance and the observe/return_home pending cap
# 확인 필요: the mission path's tracker is installed about one tick after the extract->transport transition (K1
# re-check: +0.0083 s) without its own record; a serial installed within this slack of the transition (or a pre-built
# transport serial still active then) is labelled "mission". Not a plan value.
MISSION_SLACK_S = 0.05
# 확인 필요: "the same corner" (plan 3) is not defined numerically in the plan; judged cusp end poses are grouped
# greedily when within these of a group's first pose. Only used for the corner counts.
CORNER_POS_M, CORNER_YAW_RAD = 0.10, 0.10
LABEL_KO = {  # plan 3 table names for the tool's labels (priority 3 doc 3)
    "unresolved": "미결", "localisation_decision": "위치 추정(판정 시점)",
    "unresolved_saturated_no_curvature": "미결(포화, 곡률 판정 불가)",
    "mixed_plan_margin_and_plant": "혼합(경로 여유 + plant)", "plant_shortfall": "plant",
    "plan_margin": "경로·곡률(여유)", "unresolved_saturated_margins_ok": "미결(포화, 여유 정상)",
    "plan_short_leg": "경로·곡률(짧은 다리)", "tracking_unsaturated": "실제 추종(비포화)"}


def r4(x, n=4):
    return None if x is None else round(float(x), n)


# ---- input ---------------------------------------------------------------------------------------------------------


class Run:
    def __init__(self, root: str, name: str, result: dict, sc, sha_ok=None):
        self.root, self.name, self.result, self.sc, self.sha_ok = root, name, result, sc, sha_ok


def rows_sha(sc) -> str:
    return hashlib.sha256(np.ascontiguousarray(np.round(np.asarray(sc, dtype=float), 6)).tobytes()).hexdigest()


def runs_from_fields(path: Path, name_filter: str) -> list[Run]:
    with gzip.open(path, "rt") as f:
        blob = json.load(f)
    by_name = {k.split("/")[-1]: k for k in blob}

    def rows_of(v, depth=0):
        if "slam_control_rows" in v:
            return v["slam_control_rows"]
        ref = v.get("slam_control_rows_same_as")
        if ref is None or depth > 5:
            return None
        return rows_of(blob[ref] if ref in blob else blob[by_name[ref]], depth + 1)

    out = []
    for key in sorted(blob):
        name = key.split("/")[-1]
        if name_filter and name_filter not in name:
            continue
        v = blob[key]
        rows = rows_of(v)
        sc = np.asarray(rows, dtype=float).reshape(-1, 7) if rows else None
        sha_ok = None
        if sc is not None and v.get("slam_control_sha256"):
            sha_ok = rows_sha(sc) == v["slam_control_sha256"]
        out.append(Run(key.split("/")[0] if "/" in key else path.name, name, v["result"], sc, sha_ok))
    return out


def runs_from_raw(roots: list[Path], name_filter: str) -> list[Run]:
    out = []
    for root in roots:
        for d in sorted(root.iterdir()):
            p = d / "run" / "result.json"
            if not p.is_file() or (name_filter and name_filter not in d.name):
                continue
            scp = d / "run" / "slam_control.npy"
            out.append(Run(root.name, d.name, json.loads(p.read_text()), np.load(scp) if scp.is_file() else None))
    return out


# ---- per-run context -----------------------------------------------------------------------------------------------


def transport_window(r: dict):
    tr = r.get("transitions") or []
    t0 = next((float(t["time_s"]) for t in tr if t.get("to") == "transport"), None)
    if t0 is None:
        return None, None
    t1 = next((float(t["time_s"]) for t in tr if t.get("from") == "transport" and float(t["time_s"]) >= t0), None)
    return t0, t1


def phase_at(r: dict, t: float):
    ph = None
    for x in r.get("transitions") or []:
        if float(x["time_s"]) <= t + 1e-9:
            ph = x.get("to")
    return ph


def _paths_in(obj, out, origin, t, depth=0):
    """Every path_record (poses/directions/curvatures) inside a D7 record."""
    if depth > 6:
        return
    if isinstance(obj, dict):
        if {"poses", "directions", "curvatures_inv_m"} <= set(obj):
            out.append((origin, t, obj))
            return
        for v in obj.values():
            _paths_in(v, out, origin, obj.get("time_s", t) if isinstance(obj.get("time_s"), (int, float)) else t,
                      depth + 1)
    elif isinstance(obj, list):
        for v in obj:
            _paths_in(v, out, origin, t, depth + 1)


def candidate_paths(r: dict) -> dict:
    """Plan 2.2 candidates: digest -> {"path": Path2, "origins": [[origin, time], ...]}."""
    raw = []
    raw.append(("paths.transport", None, (r.get("paths") or {}).get("transport")))
    for e in r.get("paths_replaced") or []:
        if e.get("leg") == "transport":
            raw.append((f"paths_replaced:{e.get('by')}", e.get("time_s"), e.get("path")))
    for c in r.get("cusp_replans") or []:
        raw.append(("cusp_replans.path", c.get("time_s"), c.get("path")))
        raw.append(("cusp_replans.replaced_path", c.get("time_s"), c.get("replaced_path")))
    for c in r.get("stall_replans") or []:
        raw.append(("stall_replans.path", c.get("time_s"), c.get("path")))
        raw.append(("stall_replans.replaced_path", c.get("time_s"), c.get("replaced_path")))
    for c in r.get("scene_replans") or []:
        if c.get("phase") == "transport":
            for k in ("path", "replaced_path"):
                raw.append((f"scene_replans.{k}", c.get("time_s"), c.get(k)))
    tf = r.get("tracking_failure") or {}
    raw.append(("tracking_failure.path", tf.get("time_s"), tf.get("path")))
    d7 = []
    for k in ("d7_docking", "d7_reapproach", "docking_retries"):
        _paths_in(r.get(k), d7, k, None)
    raw += d7
    out: dict = {}
    for origin, t, p in raw:
        if not isinstance(p, dict) or not p.get("poses") or len(p["poses"]) < 2:
            continue
        P = Path2(p)
        d = P.digest()
        out.setdefault(d, {"path": P, "origins": []})["origins"].append([origin, None if t is None else r4(t, 6)])
    return out


def install_events(r: dict, t0, t1) -> list[dict]:
    """Path installs in the transport window, own list (not priority 3's): mission start, cusp replans (ordinal),
    paths_replaced (by), scene replan records, stall and obstacle replans."""
    def inside(t):
        return t0 is not None and t0 - 1e-9 <= t <= (math.inf if t1 is None else t1) + 1e-9

    ev = []
    if t0 is not None:
        ev.append({"time_s": t0, "kind": "mission"})
    k = 0
    for c in r.get("cusp_replans") or []:
        t = float(c["time_s"])
        if inside(t) and phase_at(r, t) == "transport":
            k += 1
            ev.append({"time_s": t, "kind": "cusp_replan", "ordinal": k})
    for e in r.get("paths_replaced") or []:
        if e.get("leg") == "transport" and inside(float(e["time_s"])):
            ev.append({"time_s": float(e["time_s"]), "kind": str(e.get("by"))})
    for e in r.get("scene_replans") or []:
        if e.get("phase") == "transport" and e.get("outcome") == "installed" and inside(float(e["time_s"])):
            ev.append({"time_s": float(e["time_s"]), "kind": "scene_replan_record"})
    for e in r.get("stall_replans") or []:
        if inside(float(e["time_s"])):
            ev.append({"time_s": float(e["time_s"]), "kind": "stall_replan"})
    ob = r.get("obstacle_layer") or {}
    for key in ("replans", "mission_replans"):
        for e in ob.get(key) or []:
            if isinstance(e, dict) and e.get("phase") == "transport" and "time_s" in e and inside(float(e["time_s"])):
                ev.append({"time_s": float(e["time_s"]), "kind": f"obstacle_{key}"})
    return sorted(ev, key=lambda e: e["time_s"])


# paths_replaced "by" values seen in the records and what their "path" holds (core.py, checked 2026-11-15):
#   scene_replan      -> the NEW path (written after paths[phase] was overwritten, core.py 8980-8986)
#   delivery_straight -> the OLD (replaced) path (core.py 7804); the install ends the search path at its cut end
#   reapproach (D7)   -> the OLD (replaced) path (core.py 7236)
# Either way the record is only a candidate geometry here; the serial is matched by its cusp poses.
SOURCE_OF_BY = {"scene_replan": "scene_replan", "scene_replan_record": "scene_replan", "reapproach": "d7_reapproach",
                "delivery_straight": "other:delivery_straight"}


def _cusp_list(P: Path2):
    return [(int(i), P.poses[i]) for i in P.cusps]


def _pose_eq(a, b) -> bool:
    return (abs(float(a[0]) - float(b[0])) <= POSE_TOL and abs(float(a[1]) - float(b[1])) <= POSE_TOL
            and abs(wrap(float(a[2]) - float(b[2]))) <= POSE_TOL)


def cusps_prefix(serial_cusps: list, P: Path2) -> bool:
    """The serial's cusps (index, pose) equal the path's first cusps (an arm_docking cut keeps the front ones)."""
    pc = P.cusps
    if len(pc) < len(serial_cusps):
        return False
    return all(int(c["index"]) == pc[i] and _pose_eq(c["pose"], P.poses[pc[i]]) for i, c in enumerate(serial_cusps))


def _same_front(a: Path2, b: Path2, last: int) -> bool:
    if len(a.poses) <= last or len(b.poses) <= last:
        return False
    return (np.max(np.abs(a.poses[: last + 1] - b.poses[: last + 1])) <= POSE_TOL
            and list(a.directions[: last + 1]) == list(b.directions[: last + 1])
            and np.max(np.abs(a.kappa[: last + 1] - b.kappa[: last + 1])) <= POSE_TOL)


def match_serial(cusps: list, cands: dict):
    """(digest or None, note). Plan 2.2: the candidate whose cusp poses all equal the serial's (an arm_docking cut:
    the serial's are the front ones). 확인 필요: several digests that agree on poses, directions and curvatures up to
    the serial's last cusp (a cut path and its uncut source) are one path for every cusp leg and are taken as one;
    any other second match leaves the serial unresolved."""
    if not cusps:
        return None, "serial has no cusps"
    hits = [d for d, c in cands.items() if cusps_prefix(cusps, c["path"])]
    if not hits:
        return None, "no recorded path has these cusp poses"
    last = int(cusps[-1]["index"])
    ref = cands[hits[0]]["path"]
    if any(not _same_front(ref, cands[d]["path"], last) for d in hits[1:]):
        return None, f"ambiguous: {len(hits)} recorded paths with these cusp poses differ before the last cusp"
    best = min(hits, key=lambda d: (abs(len(cands[d]["path"].cusps) - len(cusps)), d))
    return best, ("unique" if len(hits) == 1 else f"{len(hits)} recorded paths, same up to the last cusp")


class Ctx:
    def __init__(self, run: Run, period: str):
        self.run, self.period = run, period
        r = self.r = run.result
        self.sc = run.sc
        self.t0, self.t1 = transport_window(r)
        self.events = install_events(r, self.t0, self.t1)
        self.cands = candidate_paths(r)
        self.tv = {int(v["serial"]): v for v in r.get("tracker_versions") or []}
        self.samples = [s for s in r.get("samples") or [] if s.get("phase") == "transport"]
        self.serial_info: dict = {}
        if period == "postfix":
            self._serials()

    def in_window(self, t: float) -> bool:
        return self.t0 is not None and self.t0 - 1e-9 <= t <= (math.inf if self.t1 is None else self.t1) + 1e-9

    def _serials(self):
        trans = sorted((v for v in self.tv.values() if v.get("phase") == "transport"), key=lambda v: int(v["serial"]))
        for v in trans:
            s = int(v["serial"])
            d, note = match_serial(v.get("cusps") or [], self.cands)
            inst = v.get("installed_s")
            pred = None
            if inst is not None and v.get("cusps"):
                for p in trans:
                    if (int(p["serial"]) < s and p.get("replaced_s") is not None
                            and abs(float(p["replaced_s"]) - float(inst)) <= TIME_TOL and p.get("cusps")
                            and len(p["cusps"]) >= len(v["cusps"])
                            and all(int(a["index"]) == int(b["index"]) and _pose_eq(a["pose"], b["pose"])
                                    for a, b in zip(v["cusps"], p["cusps"]))):
                        pred = int(p["serial"])
            self.serial_info[s] = {"serial": s, "installed_s": inst, "replaced_s": v.get("replaced_s"),
                                   "cusps": [int(c["index"]) for c in v.get("cusps") or []], "path_digest": d,
                                   "match": note, "truncation_of": pred}
        for s in sorted(self.serial_info):
            self.serial_info[s].update(self._source(s))

    def _source(self, s: int) -> dict:
        info = self.serial_info[s]
        if info["truncation_of"] is not None and info["truncation_of"] in self.serial_info:
            src = self.serial_info[info["truncation_of"]]
            return {"source": src.get("source"), "replan_ordinal": src.get("replan_ordinal"),
                    "source_note": "arm_docking cut of serial %d" % info["truncation_of"]}
        inst = info["installed_s"]
        if inst is None:
            return {"source": "other", "replan_ordinal": None, "source_note": "no installed_s"}
        inst = float(inst)
        for e in self.events:
            if abs(e["time_s"] - inst) <= TIME_TOL and e["kind"] != "mission":
                if e["kind"] == "cusp_replan":
                    return {"source": "cusp_replan", "replan_ordinal": e["ordinal"], "source_note": "install time"}
                return {"source": SOURCE_OF_BY.get(e["kind"], "other:" + e["kind"]), "replan_ordinal": None,
                        "source_note": "install time"}
        if self.t0 is not None and inst <= self.t0 + MISSION_SLACK_S:
            return {"source": "mission", "replan_ordinal": None, "source_note": "installed at or before transport start"}
        return {"source": "other", "replan_ordinal": None, "source_note": "no install record at installed_s"}

    def chain(self, s: int) -> list[int]:
        """The serial and the serials it is an arm_docking cut of (latest first)."""
        out = [s]
        while self.serial_info.get(out[-1], {}).get("truncation_of") is not None:
            out.append(self.serial_info[out[-1]]["truncation_of"])
        return out

    def chain_start(self, s: int):
        """Leg 0 start (plan 2.2: the serial's installed_s), walked back over arm_docking cuts installed in transport."""
        start = None
        for x in self.chain(s):
            inst = self.serial_info[x]["installed_s"]
            if inst is None:
                break
            if start is not None and self.t0 is not None and float(inst) < self.t0 - 1e-9:
                break  # a pre-built transport serial: its install is not when the truck started the leg
            start = float(inst)
        return start

    def active_serial(self, t: float):
        best = None
        for s, v in self.serial_info.items():
            inst, rep = v["installed_s"], v["replaced_s"]
            if inst is None or float(inst) >= t - 1e-9:
                continue
            if rep is not None and float(rep) < t - TIME_TOL:
                continue
            if best is None or s > best:
                best = s
        return best

    def latest_install(self, t: float):
        before = [e for e in self.events if e["time_s"] <= t + 1e-9]
        return before[-1] if before else None


# ---- the reconstructed judgement tick and the leg measures (plan 2.3) -----------------------------------------------


def nearest_end(P: Path2, rear) -> int:
    return min(P.ends, key=lambda i: math.hypot(P.poses[i, 0] - rear[0], P.poses[i, 1] - rear[1]))


def judgement_tick(sc, P: Path2, end: int, start: float, t_rec: float) -> dict:
    """Plan 2.3: the first slam_control row of the leg whose estimated rear axle is within 0.03 m of the leg end. A
    reconstruction, not the tracker's tick (at_endpoint also reads the remaining progress and the overshoot
    acceptance)."""
    goal = P.poses[end]
    m = (sc[:, 0] >= start - PAIR_TOL_S) & (sc[:, 0] <= t_rec + PAIR_TOL_S)
    rows = sc[m]
    out = {"found": False, "t": None, "e_E": None, "e_T": None, "overshoot_possible": False, "rows": int(len(rows)),
           "pre_entry": None}
    if not len(rows):
        out["why"] = "no slam_control rows in the leg"
        return out
    d = np.hypot(rows[:, 1] - goal[0], rows[:, 2] - goal[1])
    inside = np.nonzero(d <= CUSP_POS_TOL_M)[0]
    k = int(inside[0]) if len(inside) else None
    # the tracker's overshoot acceptance (path_tracking._overshoot_accepted): along the end heading in the end's
    # direction, across within the position tolerance; a row past the end outside 0.03 m before the reconstructed
    # tick means the tracker may have judged earlier there
    heading = np.array([math.cos(goal[2]), math.sin(goal[2])])
    off = rows[:, 1:3] - goal[:2]
    along = float(P.directions[end]) * (off @ heading)
    across = np.abs(off @ np.array([-heading[1], heading[0]]))
    lim = len(rows) if k is None else k
    past = (along[:lim] >= 0.0) & (across[:lim] <= CUSP_POS_TOL_M) & (d[:lim] > CUSP_POS_TOL_M)
    out["overshoot_possible"] = bool(np.any(past))
    if past.any():
        out["overshoot_first_t"] = r4(rows[int(np.nonzero(past)[0][0]), 0], 6)
    if k is None:
        out["why"] = "the estimate never came within 0.03 m of the leg end"
        out["min_distance_m"] = r4(float(np.min(d)))
        return out
    row = rows[k]
    out.update(found=True, t=r4(row[0], 6), _t=float(row[0]), e_E=r4(wrap(goal[2] - row[3]), 5), e_T=r4(wrap(goal[2] - row[6]), 5),
               position_error_m=r4(d[k]))
    # the tick before it: the pose's yaw error and the row-to-row speed (truth displacement)
    j = int(np.searchsorted(sc[:, 0], row[0] - PAIR_TOL_S, side="left")) - 1
    if j >= 0:
        prev = sc[j]
        dt = float(row[0] - prev[0])
        out["pre_entry"] = {"t": r4(prev[0], 6), "e_E": r4(wrap(goal[2] - prev[3]), 5),
                            "e_T": r4(wrap(goal[2] - prev[6]), 5),
                            "truth_speed_mps": r4(math.hypot(row[4] - prev[4], row[5] - prev[5]) / dt) if dt > 0 else None}
    return out


def leg_measures(ctx: Ctx, P: Path2, leg: int, end: int, start, t_rec: float, e_E_rec) -> dict:
    sc = ctx.sc
    lo, hi = P.leg_range(leg)
    leg_len = float(P.dist[hi] - P.dist[lo])
    out = {"leg": leg, "leg_range": [int(lo), int(hi)], "end_index": int(end),
           "end_pose": [r4(v, 6) for v in P.poses[end]], "leg_length_m": r4(leg_len, 3),
           "Short": bool(leg_len < SHORT_LEG_M), "kappa_end": r4(P.kappa[end]),
           "kappa_leg_max": r4(float(np.max(np.abs(P.kappa[lo + 1: hi + 1]))) if hi > lo else 0.0),
           "tau": TAU_CUSP, "leg_start_s": r4(start, 6), "_start": start}
    # recorded (post-stop) values
    row = row_exact(sc, t_rec) if sc is not None else None
    e_T_rec = None if row is None else wrap(P.poses[end, 2] - row[6])
    out["recorded"] = {"t": r4(t_rec, 6), "e_E": r4(e_E_rec, 5), "e_T": r4(e_T_rec, 5),
                       "margin": None if e_E_rec is None else r4(abs(e_E_rec) / TAU_CUSP),
                       "row_found": row is not None}
    if sc is None or start is None:
        out["tick"] = {"found": False, "why": "no slam_control" if sc is None else "leg start unknown"}
        out["sat_end"] = None
        out["leg_start_truth"] = out["leg_start_estimate"] = None
        out["O"] = None
        out["steer_residual_end"] = None
        out["pre_entry_sample"] = None
        out["margin_star"] = None
        return out
    tick = judgement_tick(sc, P, end, start, t_rec)
    out["tick"] = tick
    out["margin_star"] = None if tick.get("e_E") is None else r4(abs(tick["e_E"]) / TAU_CUSP)
    # leg start offsets (truth and estimate) on the tracker's walk, as priority 3
    a = row_exact(sc, start)
    if a is None:
        out["leg_start_truth"] = out["leg_start_estimate"] = None
        out["O"] = None
    else:
        lat, yaw = P.frame(a[4:7], P.forward([a[4:7]], lo, hi)[0][2])
        e_lat, e_yaw = P.frame(a[1:4], P.forward([a[1:4]], lo, hi)[0][2])
        out["leg_start_truth"] = {"lateral_m": r4(lat), "yaw_rad": r4(yaw)}
        out["leg_start_estimate"] = {"lateral_m": r4(e_lat), "yaw_rad": r4(e_yaw)}
        out["O"] = bool(abs(lat) > O_LAT or abs(yaw) > O_YAW)
    # the leg's transport samples (copies: the same sample can be in several judgements' windows)
    samples = []
    for s in ctx.samples:
        t = float(s["time_s"])
        if start - 1e-9 <= t <= t_rec + 1e-9:
            c = dict(s)
            sg = (s.get("tracking") or {}).get("segment_index")
            c["_row"] = row_exact(sc, t)
            c["_seg"] = int(sg) if sg is not None and lo <= int(sg) < hi and c["_row"] is not None else None
            samples.append(c)
    end_dist = P.dist[end]
    sat = saturation(samples, sc, P, t_rec, lambda s: s.get("_seg") is not None
                     and end_dist - P.dist[s["_seg"]] <= END_WINDOW_M)
    out["sat_end"] = sat
    out["samples_window"] = len(samples)
    out["samples_unpaired"] = sum(1 for s in samples if s["_seg"] is None)
    # the steering residual (command - actual, [left, right]) over the end 0.28 m, up to the reconstructed tick
    t_cut = tick["_t"] if tick.get("found") else t_rec
    sel = [s for s in samples if s["_seg"] is not None and end_dist - P.dist[s["_seg"]] <= LOOKAHEAD_M
           and float(s["time_s"]) <= t_cut + 1e-9 and s.get("steering_command_rad") and s.get("steering_actual_rad")]
    if sel:
        res = np.array([[float(c) - float(x) for c, x in zip(s["steering_command_rad"], s["steering_actual_rad"])]
                        for s in sel])
        wheel = np.array([wheel_curvatures(s["steering_actual_rad"]) for s in sel])
        out["steer_residual_end"] = {"n": len(sel), "mean": [r4(v) for v in res.mean(axis=0)],
                                     "max_abs": [r4(v) for v in np.abs(res).max(axis=0)],
                                     "wheel_curvature_mean": [r4(v) for v in wheel.mean(axis=0)]}
    else:
        out["steer_residual_end"] = None
    # the last sample before the reconstructed tick: speed and the tracker's yaw error
    pre = [s for s in samples if tick.get("found") and float(s["time_s"]) < tick["_t"] - 1e-9]
    if pre:
        s = pre[-1]
        out["pre_entry_sample"] = {"t": r4(s["time_s"], 6), "signed_speed_mps": r4(s.get("signed_speed_mps")),
                                   "speed_command_mps": r4(s.get("speed_command_mps")),
                                   "tracking_yaw_error_rad": r4((s.get("tracking") or {}).get("yaw_error_rad"), 5)}
    else:
        out["pre_entry_sample"] = None
    return out


# ---- classification (plan 3 table, first matching row) -------------------------------------------------------------


def classify(j: dict, e_E, e_T) -> tuple[str, list]:
    why = []
    if not j.get("is_cusp"):
        why.append("judged end is not a cusp")
    if j.get("path_digest") is None:
        why.append("window path unknown")
    if j.get("leg_start_s") is None:
        why.append("leg start unknown")  # 확인 필요: read as "window path unknown" (the window has no start)
    if e_T is None:
        why.append("e_T unavailable")
    if j.get("installs_inside_window"):
        why.append("another path install inside the window")
    if why:
        return "unresolved", why
    if e_E is None or abs(e_E) <= TAU_CUSP:
        return "unresolved", ["e_E missing or |e_E| <= tau"]
    if abs(e_T) <= TAU_CUSP:
        return "localisation_decision", ["D"]
    sat = j.get("sat_end") or {}
    S = sat.get("S")
    if S is None:
        # 확인 필요: the table has no row for an unknown S_end (no moving sample in the end metre); priority 3's code
        # read it as false. Left unresolved here.
        return "unresolved", ["S_end unknown"]
    if S:
        if sat.get("P") is None or sat.get("M") is None:
            return "unresolved_saturated_no_curvature", ["S_end, P/M unknown"]
        return primary_cross(True, None, sat["P"], sat["M"], False), ["S_end"]
    if j.get("Short"):
        return "plan_short_leg", ["not S_end, Short"]
    return "tracking_unsaturated", ["otherwise"]


# ---- judgements ----------------------------------------------------------------------------------------------------


def _switch_start(ctx: Ctx, P: Path2, leg: int, t: float, serials=None, after=None):
    """Plan 2.2: a leg > 0 starts at the switch that ended the previous leg (same serial chain when known)."""
    prev_end = P.ends[leg - 1]
    best = None
    for s in ctx.r.get("leg_switches") or []:
        ts = float(s["time_s"])
        if s.get("phase") != "transport" or ts >= t - 1e-9 or (after is not None and ts < after - 1e-9):
            continue
        if serials is not None and int(s.get("serial", -1)) not in serials:
            continue
        if int(s["endpoint_index"]) != prev_end or not _pose_eq(s["endpoint_pose"], P.poses[prev_end]):
            continue
        if best is None or ts > best:
            best = ts
    return best


def build_judgements(ctx: Ctx) -> tuple[list, dict]:
    r, post = ctx.r, ctx.period == "postfix"
    excluded: dict = {}
    js = []

    def ex(kind, why):
        excluded.setdefault(kind, {}).setdefault(why, 0)
        excluded[kind][why] += 1

    if ctx.t0 is None:
        for s in r.get("leg_switches") or []:
            ex("leg_switch", f"phase:{s.get('phase')}" if s.get("phase") != "transport" else "no transport window")
        return [], excluded
    # passes
    for s in r.get("leg_switches") or []:
        t = float(s["time_s"])
        if s.get("phase") != "transport":
            ex("leg_switch", f"phase:{s.get('phase')}")
            continue
        if not ctx.in_window(t):
            ex("leg_switch", "outside transport window")
            continue
        if s.get("via") not in ("arrival", "release"):
            ex("leg_switch", f"via:{s.get('via')}")
            continue
        serial = int(s["serial"]) if s.get("serial") is not None else None
        end = int(s["endpoint_index"])
        j = {"kind": "pass", "via": s["via"], "release": s["via"] == "release", "time_s": r4(t, 6), "_t": t,
             "serial": serial, "recorded_position_error_m": r4(s.get("position_error_m")),
             "switch_leg": s.get("leg")}
        P = None
        if post:
            info = ctx.serial_info.get(serial)
            if info is None:
                ex("leg_switch", "serial not a transport serial in tracker_versions")
                continue
            if end not in info["cusps"]:
                ex("leg_switch", "endpoint_index not a cusp of the serial")
                continue
            j.update(is_cusp=True, path_digest=info["path_digest"], path_match=info["match"],
                     source=info["source"], replan_ordinal=info["replan_ordinal"], serial_link="tracker_versions")
            if info["path_digest"] is None:
                j["unresolved_why"] = "serial path: " + info["match"]
            else:
                P = ctx.cands[info["path_digest"]]["path"]
        else:
            # before the fix: no tracker_versions; the recorded path whose cusp at endpoint_index has this pose
            hits = [d for d, c in ctx.cands.items() if end < len(c["path"].poses)
                    and _pose_eq(s["endpoint_pose"], c["path"].poses[end])]
            cusp_hits = [d for d in hits if end in ctx.cands[d]["path"].cusps]
            if hits and not cusp_hits:
                ex("leg_switch", "endpoint_index not a cusp of the matched path")
                continue
            ev = ctx.latest_install(t)
            j.update(is_cusp=True, serial_link="unresolved (no tracker_versions)",
                     source=_event_source(ev), replan_ordinal=(ev or {}).get("ordinal"))
            if len(cusp_hits) == 1:
                j.update(path_digest=cusp_hits[0], path_match="endpoint pose")
                P = ctx.cands[cusp_hits[0]]["path"]
            else:
                j.update(path_digest=None, path_match=f"{len(cusp_hits)} recorded paths with this cusp pose")
                j["unresolved_why"] = j["path_match"]
        if P is not None:
            leg = P.ends.index(end)
            if post:
                start = ctx.chain_start(serial) if leg == 0 else _switch_start(ctx, P, leg, t, set(ctx.chain(serial)))
            else:
                inst = ctx.latest_install(t)
                start = (inst["time_s"] if inst else None) if leg == 0 else _switch_start(
                    ctx, P, leg, t, {serial} if serial is not None else None, inst["time_s"] if inst else None)
            j.update(leg_measures(ctx, P, leg, end, start, t, s.get("yaw_error_rad")))
            j["leg_number_agrees"] = s.get("leg") == leg
        else:
            j.update(leg=None, end_index=end, leg_start_s=None, _start=None, recorded={"t": r4(t, 6), "e_E": r4(s.get("yaw_error_rad"), 5)})
        js.append(j)
    # failures: cusp replans and the final endpoint_heading failure
    fails = []
    for k, c in enumerate(r.get("cusp_replans") or []):
        t = float(c["time_s"])
        if not ctx.in_window(t) or phase_at(r, t) != "transport":
            ex("cusp_replan", "outside transport window" if not ctx.in_window(t) else f"phase:{phase_at(r, t)}")
            continue
        fails.append(("fail_replan", t, c.get("replaced_path"), c.get("rear_pose"), c.get("yaw_error_rad"), c))
    tf = r.get("tracking_failure") or {}
    trk = tf.get("tracking") or {}
    if tf:
        t = float(tf.get("time_s", math.nan))
        if trk.get("failure") != "endpoint_heading":
            ex("tracking_failure", f"failure:{trk.get('failure')}")
        elif not trk.get("at_cusp"):
            ex("tracking_failure", "not at a cusp")
        elif not ctx.in_window(t):
            ex("tracking_failure", "outside transport window")
        elif tf.get("phase") is not None and tf["phase"] != "transport":
            # a recorded phase must be transport; a record without one is judged by the transport window alone
            # (the window is the plan's own test: plan 2.2 "운반 구간 안의 최종 tracking_failure")
            ex("tracking_failure", f"phase {tf['phase']}")
        else:
            fails.append(("fail_final", t, tf.get("path"), tf.get("rear_pose"), trk.get("yaw_error_rad"), tf))
    for kind, t, pd, rear, e_E_rec, rec in fails:
        j = {"kind": kind, "time_s": r4(t, 6), "_t": t, "recorded_position_error_m": r4(
            rec.get("position_error_m", (rec.get("tracking") or {}).get("position_error_m")))}
        if kind == "fail_replan":
            j["status"] = rec.get("status")
            j["new_path_digest"] = Path2(rec["path"]).digest() if isinstance(rec.get("path"), dict) and rec["path"].get("poses") else None
        P = Path2(pd)
        j["path_digest"] = P.digest()
        end = nearest_end(P, rear)
        leg = P.ends.index(end)
        if post:
            serial = ctx.active_serial(t)
            info = ctx.serial_info.get(serial) if serial is not None else None
            j.update(serial=serial, serial_link="tracker_versions" if info else "no active transport serial")
            j["is_cusp"] = bool(info and end in info["cusps"])
            if info:
                j["serial_path_consistent"] = cusps_prefix([{"index": i, "pose": ctx.tv[serial]["cusps"][n]["pose"]}
                                                            for n, i in enumerate(info["cusps"])], P)
                j.update(source=info["source"], replan_ordinal=info["replan_ordinal"])
            start = None
            if info:
                start = ctx.chain_start(serial) if leg == 0 else _switch_start(ctx, P, leg, t, set(ctx.chain(serial)))
        else:
            ev = ctx.latest_install(t - 1e-6)
            j.update(serial=None, serial_link="unresolved (no tracker_versions)", is_cusp=end in P.cusps,
                     source=_event_source(ev), replan_ordinal=(ev or {}).get("ordinal"))
            start = (ev["time_s"] if ev else None) if leg == 0 else _switch_start(
                ctx, P, leg, t, None, ev["time_s"] if ev else None)
        j.update(leg_measures(ctx, P, leg, end, start, t, e_E_rec))
        js.append(j)
    js.sort(key=lambda j: j["_t"])
    for n, j in enumerate(js):
        j["run"] = ctx.run.name
        j["order"] = n
        j["installs_inside_window"] = [] if j.get("_start") is None else [
            e["kind"] for e in ctx.events if j["_start"] + 1e-9 < e["time_s"] < j["_t"] - 1e-9]
        j["replan_leg"] = bool(j.get("source") == "cusp_replan" and j.get("leg") == 0)
        j["first_leg_after_install"] = j.get("source") if j.get("leg") == 0 else None
    return js, excluded


def _event_source(ev):
    if ev is None:
        return None
    if ev["kind"] in ("mission", "cusp_replan"):
        return ev["kind"]
    return SOURCE_OF_BY.get(ev["kind"], "other:" + ev["kind"])


def link(ctx: Ctx, js: list) -> None:
    """Neighbours, the plan 2.2 outcome after each failure, anomalies and the classification."""
    post = ctx.period == "postfix"
    cusp_js = [j for j in js if j.get("leg") is not None]
    for n, j in enumerate(js):
        prev = js[n - 1] if n else None
        nxt = js[n + 1] if n + 1 < len(js) else None
        j["prev"] = None if prev is None else {
            "dt_s": r4(j["_t"] - prev["_t"], 4), "kind": prev["kind"],
            "e_E_recorded": (prev.get("recorded") or {}).get("e_E"),
            "position_error_m": prev.get("recorded_position_error_m")}
        j["next_dt_s"] = None if nxt is None else r4(nxt["_t"] - j["_t"], 4)
    for j in js:
        if j["kind"] == "pass":
            continue
        # anomaly: a pass of the same leg (same path, same end) between the leg start and the failure
        j["anomaly_pass_same_leg"] = any(
            p["kind"] == "pass" and p.get("path_digest") == j.get("path_digest") and p.get("end_index") == j.get("end_index")
            and j.get("_start") is not None and j["_start"] - 1e-9 <= p["_t"] <= j["_t"] + 1e-9
            for p in cusp_js)
        j["outcome"], j["outcome_next"] = outcome(ctx, j, js)
        if not post and j["kind"] == "fail_replan":
            # 확인 필요: before the fix the serial link is unresolved (plan 1); the outcome below is by time order only
            j["outcome_time_order"], j["outcome"] = j["outcome"], None
    for j in js:
        tick = j.get("tick") or {}
        if j["kind"] == "pass":
            j["reconstruction_mismatch"] = bool(tick.get("found") and abs(tick["e_E"]) > TAU_CUSP)
            continue
        j["D"] = None if tick.get("e_E") is None or tick.get("e_T") is None else bool(
            abs(tick["e_E"]) > TAU_CUSP and abs(tick["e_T"]) <= TAU_CUSP)
        j["classification"], j["classification_why"] = classify(j, tick.get("e_E"), tick.get("e_T"))
        rec = j.get("recorded") or {}
        j["D_recorded"] = None if rec.get("e_E") is None or rec.get("e_T") is None else bool(
            abs(rec["e_E"]) > TAU_CUSP and abs(rec["e_T"]) <= TAU_CUSP)
        j["classification_recorded"], _ = classify(j, rec.get("e_E"), rec.get("e_T"))


def outcome(ctx: Ctx, j: dict, js: list):
    if j["kind"] == "fail_final":
        return "final_failure", None
    if j.get("status") != "success":
        return "replan_failed", None
    t = j["_t"]
    post = ctx.period == "postfix"
    cands = []
    for x in js:
        if x["_t"] <= t + 1e-9:
            continue
        if x["kind"] == "pass":
            if post:
                inst = ctx.serial_info.get(x.get("serial"), {}).get("installed_s")
                if inst is None or float(inst) < t - TIME_TOL:
                    continue  # not the new serial
            cands.append((x["_t"], 0, "recovered", f"pass at {x['time_s']}"))
        else:
            cands.append((x["_t"], 0, "refailed", f"{x['kind']} at {x['time_s']}"))
    for e in ctx.events:
        if e["time_s"] <= t + 1e-9 or e["kind"] in ("cusp_replan", "mission"):
            continue
        if e["kind"] == "delivery_straight":
            # Evidence of arrival (Codex tool review 1 P2): the delivery straight replaces the transport path only
            # after the docking match at the search path's end accepted a straight (core.py 7785-7810, docking runs at
            # the cut search path's end, arm_docking). So the install itself records that the replan path's tracker
            # reached its end: plan 2.2 "도착".
            cands.append((e["time_s"], 1, "recovered", f"final arrival (delivery_straight at {r4(e['time_s'], 4)})"))
        else:
            cands.append((e["time_s"], 1, f"cut:{SOURCE_OF_BY.get(e['kind'], e['kind'])}", f"install at {r4(e['time_s'], 4)}"))
    if ctx.t1 is not None:
        cands.append((ctx.t1, 2, "recovered", f"transport ended normally at {r4(ctx.t1, 4)} (final arrival)"))
    if not cands:
        why = (ctx.r.get("failure_reason") or "")[:80]
        return "cut:transport_ended", f"run ended in transport: {why}"
    first = min(cands, key=lambda c: (c[0], c[1]))
    return first[2], first[3]


# ---- cases (plan 1: identical arms) --------------------------------------------------------------------------------


# 확인 필요: fields that are not trajectory values and differ between identical arms (K1 re-check, checked 2026-11-15):
# cusp_replans.planning_wall_s is the planner's wall-clock time, scene_replans.tracker_serial an object id. They are left
# out of the plan 1 comparison; any difference in them is listed per case.
CASE_IGNORE = {"cusp_replans": ("planning_wall_s",), "scene_replans": ("tracker_serial",)}


def case_key(ctx: Ctx) -> tuple[str, str]:
    r = ctx.r
    t0 = ctx.t0
    t1 = math.inf if ctx.t1 is None else ctx.t1

    def win(xs, phase_field=False):
        out = []
        for x in xs or []:
            if phase_field:
                if x.get("phase") == "transport":
                    out.append(x)
            elif t0 is not None and t0 - 1e-9 <= float(x["time_s"]) <= t1 + 1e-9:
                out.append(x)
        return out

    sc = ctx.sc
    rows = None
    if sc is not None and t0 is not None:
        rows = rows_sha(sc[(sc[:, 0] >= t0 - 1e-9) & (sc[:, 0] <= t1 + 1e-9)])
    parts = {"leg_switches": win(r.get("leg_switches")), "cusp_replans": win(r.get("cusp_replans")),
             "cusp_stalls": win(r.get("cusp_stalls"), True), "scene_replans": win(r.get("scene_replans"), True),
             "samples": win(r.get("samples")), "slam_control": rows}
    ignored = {}
    for k, drop in CASE_IGNORE.items():
        ignored[k] = [{f: x.get(f) for f in drop} for x in parts[k]]
        parts[k] = [{f: v for f, v in x.items() if f not in drop} for x in parts[k]]
    key = hashlib.sha256(json.dumps(parts, sort_keys=True).encode()).hexdigest()
    return key, hashlib.sha256(json.dumps(ignored, sort_keys=True).encode()).hexdigest()


# ---- aggregates (plan 3, 4; after the fix only) --------------------------------------------------------------------


def _axis(j, axis):
    if axis == "leg_length_m":
        return j.get("leg_length_m")
    if axis == "leg_start_yaw_rad":
        return (j.get("leg_start_truth") or {}).get("yaw_rad")
    if axis == "leg_start_lateral_m":
        return (j.get("leg_start_truth") or {}).get("lateral_m")
    if axis == "S_end_share":
        return (j.get("sat_end") or {}).get("share")
    if axis == "replan_leg":
        return j.get("replan_leg")
    if axis == "margin_star":
        return j.get("margin_star")
    raise KeyError(axis)


AXES = ("leg_length_m", "leg_start_yaw_rad", "leg_start_lateral_m", "S_end_share", "replan_leg", "margin_star")


def ranges(passes, fails) -> dict:
    out = {}
    for ax in AXES:
        pv = [_axis(j, ax) for j in passes]
        fv = [_axis(j, ax) for j in fails]
        pk, fk = [v for v in pv if v is not None], [v for v in fv if v is not None]
        if ax == "replan_leg":
            ps, fs = sorted({bool(v) for v in pk}), sorted({bool(v) for v in fk})
            out[ax] = {"pass": {"true": sum(map(bool, pk)), "false": sum(not v for v in pk), "missing": len(pv) - len(pk)},
                       "fail": {"true": sum(map(bool, fk)), "false": sum(not v for v in fk), "missing": len(fv) - len(fk)},
                       "separated": bool(pk and fk and not set(ps) & set(fs))}
            continue
        out[ax] = {"pass": {"n": len(pk), "missing": len(pv) - len(pk), "min": r4(min(pk)) if pk else None,
                            "max": r4(max(pk)) if pk else None},
                   "fail": {"n": len(fk), "missing": len(fv) - len(fk), "min": r4(min(fk)) if fk else None,
                            "max": r4(max(fk)) if fk else None}}
        out[ax]["separated"] = bool(pk and fk and (max(pk) < min(fk) or max(fk) < min(pk)))
    return out


def per_run(js_by_case) -> dict:
    out = {}
    for head, js in js_by_case.items():
        row = {}
        for ax in AXES:
            if ax == "replan_leg":
                continue
            for side, sel in (("pass", [j for j in js if j["kind"] == "pass"]), ("fail", [j for j in js if j["kind"] != "pass"])):
                v = [_axis(j, ax) for j in sel]
                v = [x for x in v if x is not None]
                row.setdefault(ax, {})[side] = {"n": len(v), "max": r4(max(v)) if v else None,
                                                "median": r4(statistics.median(v)) if v else None,
                                                "max_abs": r4(max(map(abs, v))) if v else None}
        out[head] = row
    return out


def corners(js) -> list:
    groups = []
    for j in sorted(js, key=lambda j: (j["run"], j["time_s"])):
        p = j.get("end_pose")
        if p is None:
            continue
        for g in groups:
            q = g["pose"]
            if abs(p[0] - q[0]) <= CORNER_POS_M and abs(p[1] - q[1]) <= CORNER_POS_M and abs(wrap(p[2] - q[2])) <= CORNER_YAW_RAD:
                break
        else:
            g = {"pose": p, "pass": 0, "fail": 0, "runs": set(), "members": []}
            groups.append(g)
        g["pass" if j["kind"] == "pass" else "fail"] += 1
        g["runs"].add(j["run"])
        g["members"].append([j["run"], j["time_s"], j["kind"]])
    out = [{**g, "runs": sorted(g["runs"])} for g in groups]
    return sorted(out, key=lambda g: (-g["fail"], -g["pass"]))


def counts(js) -> dict:
    passes = [j for j in js if j["kind"] == "pass"]
    fails = [j for j in js if j["kind"] != "pass"]
    out = {"judgements": len(js), "passes": len(passes), "passes_release": sum(j["release"] for j in passes),
           "passes_path_unresolved": sum(j.get("path_digest") is None for j in passes),
           "failures_replan": sum(j["kind"] == "fail_replan" for j in fails),
           "failures_final": sum(j["kind"] == "fail_final" for j in fails),
           "classification": {}, "classification_recorded": {}, "outcome": {}, "outcome_time_order": {},
           "anomalies_pass_same_leg": sum(bool(j.get("anomaly_pass_same_leg")) for j in fails)}
    for j in fails:
        for key, field in (("classification", "classification"), ("classification_recorded", "classification_recorded"),
                           ("outcome", "outcome"), ("outcome_time_order", "outcome_time_order")):
            v = j.get(field)
            if v is not None:
                out[key][v] = out[key].get(v, 0) + 1
    found_p = [j for j in passes if (j.get("tick") or {}).get("found")]
    mism = [j for j in found_p if j.get("reconstruction_mismatch")]
    out["reconstruction"] = {
        "pass_tick_found": len(found_p), "pass_mismatch": len(mism),
        "pass_mismatch_rate": r4(len(mism) / len(found_p)) if found_p else None,
        "pass_mismatch_at": [[j["run"], j["time_s"], j["tick"]["e_E"]] for j in mism],
        "tick_not_found": {"pass": sum(not (j.get("tick") or {}).get("found") for j in passes),
                           "fail": sum(not (j.get("tick") or {}).get("found") for j in fails)},
        "overshoot_possible": {"pass": sum(bool((j.get("tick") or {}).get("overshoot_possible")) for j in passes),
                               "fail": sum(bool((j.get("tick") or {}).get("overshoot_possible")) for j in fails)},
        "note": "the reconstructed tick is a sample, not the tracker's tick; the pass mismatch rate checks the pass "
                "side only and does not vouch for the failure side's reconstruction (plan 2.3)"}
    return out


def plan4(fails) -> dict:
    def band(vals):
        v = [abs(x) for x in vals if x is not None]
        inb = [x for x in v if BAND[0] < x <= BAND[1]]
        return {"n": len(v), "in_band": len(inb), "share": r4(len(inb) / len(v)) if v else None}

    nonshort = [j for j in fails if j.get("Short") is False]
    known = [j for j in fails if j.get("Short") is not None]
    cls: dict = {}
    for j in nonshort:
        cls[j["classification"]] = cls.get(j["classification"], 0) + 1
    return {"band_rad": list(BAND),
            "e_E_star_in_band": band([(j.get("tick") or {}).get("e_E") for j in fails]),
            "e_E_recorded_in_band": band([(j.get("recorded") or {}).get("e_E") for j in fails]),
            "non_short_failures": {"n": len(nonshort), "of": len(known),
                                   "share": r4(len(nonshort) / len(known)) if known else None, "classification": cls}}


# ---- priority 3's five cases (before the fix) ----------------------------------------------------------------------


def tdep_check(ctxs: dict, js_by_run: dict, tdep_path: Path) -> list:
    """Plan 2.3: the priority 3 endpoint cases again, with the post-stop values priority 3 used (agreement check) and
    with the reconstructed tick's values. Not a check of the priority 4 rules."""
    if not tdep_path.is_file():
        return [{"error": f"{tdep_path} not found"}]
    p3 = json.loads(tdep_path.read_text())
    out = []
    for c in p3.get("cases") or []:
        if c.get("kind") != "endpoint_heading":
            continue
        name = c["runs"][0]
        js = js_by_run.get(name)
        row = {"runs": c["runs"], "p3_primary": c.get("primary"), "p3_label_ko": LABEL_KO.get(c.get("primary")),
               "p3": {k: c.get(k) for k in ("t_judged", "leg", "leg_length_m", "e_E", "e_T", "D", "Short", "path_sha")},
               "p3_S_end_share": (c.get("sat_end") or {}).get("share"),
               "p3_PM": [(c.get("sat_end") or {}).get("P"), (c.get("sat_end") or {}).get("M")]}
        if js is None:
            row["error"] = "run not in the input"
            out.append(row)
            continue
        first = next((j for j in js if j["kind"] == "fail_replan"), None) or next(
            (j for j in js if j["kind"] == "fail_final"), None)
        if first is None:
            row["error"] = "no failure judgement"
            out.append(row)
            continue
        rec, tick = first.get("recorded") or {}, first.get("tick") or {}
        sat = first.get("sat_end") or {}
        row["p4"] = {"time_s": first["time_s"], "leg": first.get("leg"), "leg_length_m": first.get("leg_length_m"),
                     "path_digest": first.get("path_digest"), "leg_start_s": first.get("leg_start_s"),
                     "S_end_share": sat.get("share"), "PM": [sat.get("P"), sat.get("M")], "Short": first.get("Short"),
                     "recorded": {"e_E": rec.get("e_E"), "e_T": rec.get("e_T"), "D": first.get("D_recorded")},
                     "tick": {"t": tick.get("t"), "e_E": tick.get("e_E"), "e_T": tick.get("e_T"), "D": first.get("D")}}
        row["p4_recorded_classification"] = first["classification_recorded"]
        row["p4_tick_classification"] = first["classification"]
        row["agrees_with_p3_recorded"] = first["classification_recorded"] == c.get("primary")
        row["tick_changes_classification"] = first["classification"] != first["classification_recorded"]
        row["same_leg_and_path"] = (first.get("leg") == c.get("leg") and first.get("path_digest") == c.get("path_sha"))

        def close(a, b, tol):
            return a is not None and b is not None and abs(float(a) - float(b)) <= tol

        p3sat = c.get("sat_end") or {}
        row["values_agree_with_p3"] = bool(
            close(rec.get("e_E"), c.get("e_E"), 1e-4) and close(rec.get("e_T"), c.get("e_T"), 1e-4)
            and close(first.get("leg_length_m"), c.get("leg_length_m"), 1e-3)
            and close(sat.get("share"), p3sat.get("share"), 1e-3)
            and [sat.get("P"), sat.get("M")] == [p3sat.get("P"), p3sat.get("M")] and first.get("Short") == c.get("Short"))
        out.append(row)
    return out


# ---- main ----------------------------------------------------------------------------------------------------------


def analyse(runs: list[Run], period: str, tdep_path: Path | None = None) -> dict:
    ctxs, rows, js_by_run, keys, ign = {}, [], {}, {}, {}
    for run in runs:
        ctx = Ctx(run, period)
        ctxs[run.name] = ctx
        info = {"name": run.name, "root": run.root, "success": run.result.get("success"),
                "failure_reason": (run.result.get("failure_reason") or "")[:100] or None,
                "transport_window_s": [r4(ctx.t0, 6), r4(ctx.t1, 6)], "slam_control_rows": None if run.sc is None else len(run.sc),
                "slam_control_sha_ok": run.sha_ok}
        if run.sc is None or ctx.t0 is None:
            info["skipped"] = "no slam_control" if run.sc is None else "no transport"
            rows.append(info)
            continue
        js, excluded = build_judgements(ctx)
        link(ctx, js)
        js_by_run[run.name] = js
        keys[run.name], ign[run.name] = case_key(ctx)
        info.update(judgements=len(js), excluded=excluded,
                    cusp_stalls_transport=[{"time_s": r4(c.get("time_s"), 4), "leg_index": c.get("leg_index"),
                                            "disposition": c.get("disposition"),
                                            "position_error_m": r4(c.get("position_error_m")),
                                            "yaw_error_rad": r4(c.get("yaw_error_rad"), 5)}
                                           for c in ctx.r.get("cusp_stalls") or [] if c.get("phase") == "transport"],
                    install_events=[{k: (r4(v, 6) if k == "time_s" else v) for k, v in e.items()} for e in ctx.events])
        if period == "postfix":
            info["serials"] = [ctx.serial_info[s] for s in sorted(ctx.serial_info)]
        rows.append(info)
    groups: dict = {}
    for name in sorted(keys):
        groups.setdefault(keys[name], []).append(name)
    cases = [{"head": min(names), "runs": names,
              "ignored_fields_differ": len({ign[n] for n in names}) > 1} for names in groups.values()]
    cases.sort(key=lambda c: c["head"])
    case_js = {c["head"]: js_by_run[c["head"]] for c in cases}
    all_js = [j for c in cases for j in js_by_run[c["head"]]]
    for c in cases:
        for j in js_by_run[c["head"]]:
            j["case_runs"] = c["runs"]
    excluded_total: dict = {}
    for c in cases:
        for kind, d in next(r for r in rows if r["name"] == c["head"]).get("excluded", {}).items():
            for why, n in d.items():
                excluded_total.setdefault(kind, {}).setdefault(why, 0)
                excluded_total[kind][why] += n
    out = {"period": period, "plan": "docs/plans/2026-11-15-aisle-cusp-heading.md",
           "constants": {"TAU_CUSP": TAU_CUSP, "CUSP_POS_TOL_M": CUSP_POS_TOL_M, "LOOKAHEAD_M": LOOKAHEAD_M,
                         "END_WINDOW_M": END_WINDOW_M, "SHORT_LEG_M": SHORT_LEG_M, "SAT": SAT, "S_SHARE": S_SHARE,
                         "P_RATIO": P_RATIO, "M_MARGIN": M_MARGIN, "O_LAT": O_LAT, "O_YAW": O_YAW,
                         "POSE_TOL": POSE_TOL, "PAIR_TOL_S": PAIR_TOL_S, "BAND": list(BAND),
                         "MISSION_SLACK_S": MISSION_SLACK_S, "CORNER_POS_M": CORNER_POS_M,
                         "CORNER_YAW_RAD": CORNER_YAW_RAD},
           "label_ko": LABEL_KO,
           "n_runs": len(runs), "n_runs_with_judgements": len(js_by_run), "n_cases": len(cases), "cases": cases,
           "case_ignored_fields": {k: list(v) for k, v in CASE_IGNORE.items()},
           "counts": {**counts(all_js), "excluded": excluded_total}, "runs": rows, "judgements": all_js}
    passes = [j for j in all_js if j["kind"] == "pass"]
    fails = [j for j in all_js if j["kind"] != "pass"]
    if period == "postfix":
        out["aggregate"] = {"ranges_pass_vs_fail": ranges(passes, fails), "per_case": per_run(case_js),
                            "corners": corners(all_js), "plan4": plan4(fails),
                            "corner_grouping_note": ("reference only: judged cusp end poses grouped greedily, in "
                                                     "(run, time) order, within CORNER_POS_M / CORNER_YAW_RAD of a "
                                                     "group's first pose; not a plan value"),
                            "note": "descriptive only: 'separated' means the two ranges do not overlap in this batch; "
                                    "judgements within a run are not independent; no statistical difference is claimed. "
                                    "margin_star (|e_E*|/tau) separates by construction for a consistent reconstruction "
                                    "(a pass needs |e_E*| <= tau); it is listed only to set the two sides' tick values "
                                    "side by side (plan 2.3)"}
    else:
        out["tdep_check"] = tdep_check(ctxs, js_by_run, tdep_path or TDEP_JSON)
        out["prefix_note"] = (
            "before the fix: the runs were picked by priority 3 as transport failures, so passes are the early part of "
            "failing trajectories and their denominator is incomplete. Per-judgement values can be reported; shares, "
            "ranges and distributions cannot (plan 3). No tracker_versions: the serial link is unresolved, failures "
            "sit on the nearest leg end of the replaced path, passes on the recorded path holding their endpoint pose; "
            "the outcome after a cusp replan is given by time order only (outcome_time_order). slam_control rows are "
            "at the sample times (0.1 s), so the reconstructed tick can be up to 0.1 s late.")
    return out


def strip_private(x):
    if isinstance(x, dict):
        return {k: strip_private(v) for k, v in x.items() if not str(k).startswith("_")}
    if isinstance(x, list):
        return [strip_private(v) for v in x]
    return x


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw-root", action="append", type=Path, default=[])
    ap.add_argument("--fields", type=Path)
    ap.add_argument("--period", choices=("postfix", "prefix"), default="postfix",
                    help="postfix: K1 re-check with tracker_versions; prefix: priority 3 fields (no tracker_versions)")
    ap.add_argument("--name-filter", default="aisle_rack")
    ap.add_argument("--tdep-json", type=Path, default=TDEP_JSON, help="priority 3 output (prefix check)")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    if bool(a.raw_root) == bool(a.fields):
        raise SystemExit("give --raw-root DIR... or --fields FILE")
    runs = runs_from_fields(a.fields, a.name_filter) if a.fields else runs_from_raw(a.raw_root, a.name_filter)
    out = analyse(runs, a.period, a.tdep_json)
    out["inputs"] = {"fields": str(a.fields) if a.fields else None, "raw_root": [str(p) for p in a.raw_root],
                     "name_filter": a.name_filter}
    a.out.write_text(json.dumps(strip_private(out), indent=1, default=float, allow_nan=False) + "\n")
    c = out["counts"]
    print(f"period={a.period} runs={out['n_runs']} cases={out['n_cases']} judgements={c['judgements']} "
          f"passes={c['passes']} (release {c['passes_release']}) fail_replan={c['failures_replan']} "
          f"fail_final={c['failures_final']}")
    print("classification", json.dumps(c["classification"]), "outcome", json.dumps(c["outcome"] or c["outcome_time_order"]))
    print("reconstruction", json.dumps({k: v for k, v in c["reconstruction"].items() if k != "note"}))
    for x in out.get("tdep_check") or []:
        print("p3", x.get("runs"), x.get("p3_primary"), "->", x.get("p4_recorded_classification"), "/",
              x.get("p4_tick_classification"), "agree", x.get("agrees_with_p3_recorded"), "values",
              x.get("values_agree_with_p3"))


if __name__ == "__main__":
    main()
