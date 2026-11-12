#!/usr/bin/env python3
"""Teleop real time in Isaac live sessions, read from their records (docs/validation/2026-11-12-teleop-realtime.md).

Reads, per session folder (``<name>/s``): ``probe.json`` (the console probe's states, its scripted teleop holds and
their start/end times), ``live/epoch_1/run/live_log.jsonl`` (the runner's per-tick records: mode, sim time, wall
time, pacing_ok, the arbiter's speed target; status records with timers) and, beside the session, ``<name>_ps.jsonl``
(the other users' CPU sampler). Probe and runner share one host's CLOCK_MONOTONIC.

Per session:
  modes   AUTO / TELEOP: sim/wall over consecutive same-mode ticks (wall gaps >= 1 s left out and reported), the
          ratio over whole mode stretches, pacing_ok by ticks and by wall time, the longest not-ok stretch
  holds   each scripted teleop hold: the wall share with pacing ok and with a nonzero arbiter speed target, the time
          to the first ok tick and to the first output, the console's refusal reasons before that output, the lift
          change (the arbiter refuses driving and lifting while pacing is not ok -- arbiter.py 'pacing')
  timers  per mode and key: seconds per wall second, calls per wall second, mean ms per call, max ms
          (nested timers overlap: never add them)
  overlap the share of sampler rows with other users' CPU over 1 core
  windows (with ``--factor`` and ``tick_trace.npz``) the pacer's 1 s window (steps over step-end wall times) in
          TELEOP: its minimum and the steps whose window fell below 0.9 x factor
  safety  ``result.json``'s safety records (scene damage, forbidden pocket contacts, the runner's unpermitted entries
          -- AUTO only: its evaluator does not run while the mission is held) and, from the spool's ``slam.control``
          (truth rear pose, columns 4-6), the hull's clearance to every scene obstacle at every TELEOP control row

The summary leaves out sessions whose overlap is over ``--overlap-max`` (0.05: the latency plan's 1e (c) rule).
``--verdict --slots NAME,...`` judges the sessions by the smoothness confirmation's registered rules
(docs/plans/2026-11-12-teleop-smooth-confirmation.md).

    python tools/teleop_realtime_report.py SESSION_DIR... [--overlap-max 0.05] [--factor F --verdict] [--json out]
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
GAP_S = 1.0
OVERLAP_CORES = 1.0
DT = 1 / 120
# The registered rules (docs/plans/2026-11-12-teleop-smooth-confirmation.md 3)
DRIVE_HOLDS = ("loaded_forward", "forward", "reverse", "left", "right")
TURN_HOLDS = ("left", "right")
LIFT_HOLDS = ("lift_up", "lift_down")
SLOTS_PER_ARM = 3
OUTPUT_MIN = 0.95
GAP_MAX_MS = 100.0  # the longest pacing not-ok stretch in a direct hold, and the longest no-output stretch after output
TURN_FIRST_MAX_MS = 100.0


def _load(path: Path) -> list:
    out = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def _modes(ticks: list) -> dict:
    out = {}
    for m in ("AUTO", "TELEOP"):
        sim = wall = gap = ok_wall = tot_wall = span_sim = span_wall = longest = cur = 0.0
        n = ok_n = 0
        seg = None
        for a, b in zip(ticks, ticks[1:]):
            if a["mode"] != m:
                seg, cur = None, 0.0
                continue
            seg = seg or a
            n += 1
            ok_n += 1 if a["pacing_ok"] else 0
            dw = max(b["mono"] - a["mono"], 0.0)
            if b["mode"] != m:
                span_sim += a["tick_t"] - seg["tick_t"]
                span_wall += a["mono"] - seg["mono"]
                seg, cur = None, 0.0
                continue
            if dw < GAP_S:
                sim += b["tick_t"] - a["tick_t"]
                wall += dw
            else:
                gap += dw
            tot_wall += dw
            ok_wall += dw if a["pacing_ok"] else 0.0
            cur = 0.0 if a["pacing_ok"] else cur + dw
            longest = max(longest, cur)
        if seg is not None and ticks:
            span_sim += ticks[-1]["tick_t"] - seg["tick_t"]
            span_wall += ticks[-1]["mono"] - seg["mono"]
        drops = sum(1 for a, b in zip(ticks, ticks[1:]) if a["mode"] == m and b["mode"] == m
                    and a["pacing_ok"] and not b["pacing_ok"])
        # entering the mode not ok (pacing carried over from the mode before, Codex registration P3): how long until
        # the first ok tick of each stretch, the longest of them
        entry, never_ok = 0.0, 0
        for k, r in enumerate(ticks):
            if r["mode"] == m and (k == 0 or ticks[k - 1]["mode"] != m) and not r["pacing_ok"]:
                rest = ticks[k:]
                end = next((i for i, x in enumerate(rest) if x["mode"] != m or x["pacing_ok"]), len(rest) - 1)
                entry = max(entry, rest[end]["mono"] - r["mono"])
                never_ok += 0 if (rest[end]["mode"] == m and rest[end]["pacing_ok"]) else 1  # left (or ended) not ok
        out[m] = {"pacing_drops": drops, "entry_not_ok_s": entry, "stretches_never_ok": never_ok,
                  "ratio": sim / wall if wall else None, "ratio_span": span_sim / span_wall if span_wall else None,
                  "sim_s": sim, "gap_wall_s": gap, "ok_ticks": ok_n / n if n else None,
                  "ok_wall": ok_wall / tot_wall if tot_wall else None, "longest_not_ok_s": longest}
    return out


def _segments(ticks: list, monos: list, t0: float, t1: float) -> list | None:
    """[t0, t1) as (start, end, tick in force) pieces: the state of the last tick at or before each instant (Codex
    registration P2-1: the hold's own edges, not its first and last inner ticks). None when no tick is at or before
    t0 or none at or after t1 (the record does not cover the hold)."""
    i = bisect.bisect_right(monos, t0) - 1
    if i < 0 or monos[-1] < t1:
        return None
    segs, start, cur = [], t0, ticks[i]
    for r in ticks[i + 1:]:
        if r["mono"] >= t1:
            break
        if r["mono"] > start:
            segs.append((start, r["mono"], cur))
            start = r["mono"]
        cur = r
    segs.append((start, t1, cur))
    return segs


def _holds(probe: dict, ticks: list) -> list:
    states = sorted(probe.get("states") or [], key=lambda s: s["mono"])
    monos = [r["mono"] for r in ticks]
    out = []
    for h in (probe.get("confirm") or {}).get("holds") or []:
        t0, t1 = h.get("start_mono"), h.get("end_mono")
        if t0 is None or t1 is None or t1 <= t0:
            continue
        ss, es = h.get("start_state") or {}, h.get("end_state") or {}
        row = {"label": h.get("label"), "lift_dir": h.get("lift_dir"), "mode_kind": h.get("mode"), "wall_s": t1 - t0,
               "lift_delta_m": (es["lift_m"] - ss["lift_m"]
                                if ss.get("lift_m") is not None and es.get("lift_m") is not None else None)}
        segs = _segments(ticks, monos, t0, t1) if ticks else None
        if segs is None:
            out.append(dict(row, covered=False, ok_at_start=None, ok_share=None, output_share=None, first_ok_ms=None,
                            first_output_ms=None, longest_not_ok_ms=None, longest_quiet_after_output_ms=None,
                            pacing_drops=None, reasons_before_output=[]))
            continue

        def moving(r):
            return abs(r.get("speed_mps") or 0.0) > 1e-6

        wall = t1 - t0
        first_ok = next((a for a, _, r in segs if r["pacing_ok"]), None)
        first_out = next((a for a, _, r in segs if moving(r)), None)
        until = first_out if first_out is not None else t1
        reasons = sorted({str(s.get("teleop_forbidden")) for s in states
                          if t0 <= s["mono"] <= until and s.get("teleop_forbidden")})
        longest = quiet = cur = cur_q = 0.0
        started = False
        for a, b, r in segs:
            cur = 0.0 if r["pacing_ok"] else cur + (b - a)
            longest = max(longest, cur)
            started = started or moving(r)
            cur_q = 0.0 if (moving(r) or not started) else cur_q + (b - a)
            quiet = max(quiet, cur_q)
        drops = sum(1 for (_, _, x), (_, _, y) in zip(segs, segs[1:]) if x["pacing_ok"] and not y["pacing_ok"])
        out.append(dict(row, covered=True, ok_at_start=segs[0][2]["pacing_ok"],
                        ok_share=sum(b - a for a, b, r in segs if r["pacing_ok"]) / wall,
                        output_share=sum(b - a for a, b, r in segs if moving(r)) / wall,
                        first_ok_ms=None if first_ok is None else (first_ok - t0) * 1e3,
                        first_output_ms=None if first_out is None else (first_out - t0) * 1e3,
                        longest_not_ok_ms=longest * 1e3, longest_quiet_after_output_ms=quiet * 1e3,
                        pacing_drops=drops, reasons_before_output=reasons))
    return out


def _timers(recs: list, ticks: list) -> dict:
    mode_at = {round(r["tick_t"], 4): r["mode"] for r in ticks}
    acc: dict = {}
    prev = None
    for r in recs:
        if not r.get("status") or "timers" not in r or "mono" not in r:
            continue
        m = mode_at.get(round(r.get("tick_t", -1.0), 4))
        if m is not None and prev is not None and 0 < r["mono"] - prev < 2 * GAP_S:
            a = acc.setdefault(m, {"wall": 0.0, "keys": {}})
            a["wall"] += r["mono"] - prev
            for k, v in r["timers"].items():
                e = a["keys"].setdefault(k, [0.0, 0, 0.0])
                e[0] += v[0] or 0.0
                e[1] += v[1] or 0
                e[2] = max(e[2], v[2] or 0.0)
        prev = r["mono"]
    return {m: {k: {"s_per_s": e[0] / a["wall"], "calls_per_s": e[1] / a["wall"],
                    "mean_ms": e[0] / e[1] * 1e3 if e[1] else None, "max_ms": e[2] * 1e3}
                for k, e in a["keys"].items()} for m, a in acc.items() if a["wall"]}


def _spans(ticks: list, mode: str) -> list:
    """(first, last) sim time of each run of consecutive ticks in mode."""
    out, prev = [], None
    for r in sorted(ticks, key=lambda r: r["tick_t"]):
        if r["mode"] == mode:
            if prev == mode:
                out[-1][1] = r["tick_t"]
            else:
                out.append([r["tick_t"], r["tick_t"]])
        prev = r["mode"]
    return [tuple(x) for x in out]


def _windows(run: Path, ticks: list, factor: float) -> dict | None:
    path = run / "tick_trace.npz"
    if not path.exists():
        return None
    sys.path.insert(0, str(REPO / "sim" / "isaac"))
    from tick_trace import COLUMNS

    c = {n: i for i, n in enumerate(COLUMNS)}
    with np.load(path) as z:
        data = z["data"]
    data = data[np.isfinite(data[:, c["S"]])]
    sim = data[:, c["tick"]] * DT
    end = np.where(np.isfinite(data[:, c["step_end"]]), data[:, c["step_end"]], data[:, c["S"]])
    win = np.full(len(end), np.nan)
    j = 0
    for i in range(len(end)):  # the pacer's window: steps over the wall between step ends, at least half full
        while end[i] - end[j] > 1.0:
            j += 1
        if end[i] - end[j] >= 0.5:
            win[i] = (i - j) * DT / (end[i] - end[j])
    tele = np.zeros(len(end), dtype=bool)
    for a, b in _spans(ticks, "TELEOP"):
        tele |= (sim >= a - 1e-9) & (sim <= b + 1e-9)
    w = win[tele & np.isfinite(win)]
    thr = 0.9 * factor
    return {"threshold": thr, "teleop_trace_rows": int(tele.sum()),
            "teleop_ticks": sum(1 for r in ticks if r["mode"] == "TELEOP"),
            "teleop_steps": int(len(w)), "teleop_min": float(w.min()) if len(w) else None,
            "teleop_p1": float(np.percentile(w, 1)) if len(w) else None,
            "teleop_below": int((w < thr).sum())}


def _safety(run: Path, ticks: list) -> dict:
    out = {"result": False, "teleop_rows": 0, "teleop_overlap_ticks": 0, "teleop_min_clearance_m": None,
           "teleop_min_clearance_loaded_m": None, "unevaluated": []}
    try:
        res = json.loads((run / "result.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        out["unevaluated"].append("no result.json")
        return out
    # every safety field must be there in its form (Codex registration round 3 P2-2); scene_damaged alone may be
    # absent or null: the runner writes it only when the scene was damaged
    lay = res.get("obstacle_layer")
    g = res.get("geometry") or {}
    missing = [k for k, ok in (
        ("forbidden_pocket_contacts", isinstance(res.get("forbidden_pocket_contacts"), list)),
        ("obstacle_layer.unpermitted_entries", isinstance(lay, dict) and isinstance(lay.get("unpermitted_entries"), int)),
        ("obstacle_layer.unpermitted_dir_entries", isinstance(lay, dict)
         and isinstance(lay.get("unpermitted_dir_entries"), int)),
        ("scene_layout.obstacles", isinstance((res.get("scene_layout") or {}).get("obstacles"), list)
         and len(res["scene_layout"]["obstacles"]) > 0),
        ("geometry footprints", isinstance(g.get("unloaded_footprint"), dict) and isinstance(g.get("loaded_footprint"),
                                                                                            dict))) if not ok]
    if missing:
        out["unevaluated"].append(f"result.json lacks {missing}")
        return out
    out["result"] = True
    out.update(scene_damaged=res.get("scene_damaged"), forbidden_pocket_contacts=len(res["forbidden_pocket_contacts"]),
               unpermitted_auto=lay["unpermitted_entries"] + lay["unpermitted_dir_entries"], success=res.get("success"))
    sys.path.insert(0, str(REPO / "src"))
    from forklift_core.control.low_clearance import shape_clearance
    from forklift_core.live.spool import load_list
    from forklift_core.planning.geometry import Footprint, Rectangle

    try:
        rows = load_list(run / "spool", "slam.control")
    except (OSError, ValueError, KeyError) as e:
        out["unevaluated"].append(f"slam.control: {e}")
        return out
    hull, loaded = Footprint(**g["unloaded_footprint"]), Footprint(**g["loaded_footprint"])  # checked above
    rects = [Rectangle(float(o["x_m"]), float(o["y_m"]), float(o["length_m"]), float(o["width_m"]),
                       float(o["yaw_rad"])) for o in (res.get("scene_layout") or {}).get("obstacles") or []]
    spans = _spans(ticks, "TELEOP")
    lo = lo_l = math.inf
    for r in rows:
        t = float(r[0])
        if not any(a - 1e-9 <= t <= b + 1e-9 for a, b in spans):
            continue
        out["teleop_rows"] += 1
        if not rects:
            continue
        pose = (float(r[4]), float(r[5]), float(r[6]))
        d = shape_clearance(pose, hull, rects)
        lo, lo_l = min(lo, d), min(lo_l, shape_clearance(pose, loaded, rects))
        out["teleop_overlap_ticks"] += 1 if d <= 0.0 else 0
    if out["teleop_rows"]:
        out["teleop_min_clearance_m"] = None if math.isinf(lo) else float(lo)
        out["teleop_min_clearance_loaded_m"] = None if math.isinf(lo_l) else float(lo_l)
    return out


def session(sd: Path, factor: float | None = None) -> dict:
    sd = Path(sd)
    name = sd.parent.name if sd.name == "s" else sd.name
    run = sd / "live" / "epoch_1" / "run"
    sys.path.insert(0, str(REPO / "src"))
    from forklift_core.live.spool import read_live_log

    log_error = None
    try:
        recs = read_live_log(run)  # refuses a log that lost lines or parts (Codex registration P2-2)
    except ValueError as e:
        log_error = str(e)
        recs = _load(run / "live_log.jsonl")
    ticks = sorted((r for r in recs if {"tick_t", "mode", "pacing_ok", "mono"} <= set(r)), key=lambda r: r["mono"])
    probe = json.loads((sd / "probe.json").read_text(encoding="utf-8"))
    ps = sd.parent.parent / f"{name}_ps.jsonl" if sd.name == "s" else sd.parent / f"{name}_ps.jsonl"
    rows = [r for r in _load(ps) if "other_cores" in r] if ps.exists() else []
    overlap = sum(1 for r in rows if r["other_cores"] > OVERLAP_CORES) / len(rows) if rows else None
    return {"session": name, "modes": _modes(ticks), "holds": _holds(probe, ticks), "timers": _timers(recs, ticks),
            "overlap": overlap, "log_error": log_error, "safety": _safety(run, ticks),
            "windows": None if factor is None else _windows(run, ticks, factor)}


def slot_of(name: str) -> str:
    """The registered slot a session fills: a rerun is named after its original with ``_re``."""
    return name[:-3] if name.endswith("_re") else name


def _session_problems(s: dict) -> list:
    """What keeps a session from being judged at all (Codex registration P2-2): a missing or incomplete record, a
    missing hold, a hold its ticks do not cover."""
    out = []
    if s.get("overlap") is None:
        out.append("no CPU sampler rows")
    if s.get("log_error"):
        out.append(f"live log: {s['log_error']}")
    w = s.get("windows")
    if not w:
        out.append("no pacing windows (tick_trace.npz)")
    elif w.get("teleop_min") is None or not w.get("teleop_ticks") or w["teleop_trace_rows"] < 0.95 * w["teleop_ticks"]:
        out.append(f"pacing windows do not cover TELEOP ({w.get('teleop_trace_rows')} trace rows, "
                   f"{w.get('teleop_ticks')} ticks)")
    direct = [h for h in s["holds"] if h.get("mode_kind") == "direct"]
    for kind in DRIVE_HOLDS + LIFT_HOLDS:
        if not any(h["label"] == kind for h in direct):
            out.append(f"missing hold {kind}")
    out += [f"hold {h['label']} not covered by ticks" for h in direct if not h.get("covered", True)]
    safe = s.get("safety") or {}
    if not safe.get("result") or not safe.get("teleop_rows") or safe.get("unevaluated"):
        out.append(f"safety: {safe.get('unevaluated') or 'no truth rows in TELEOP'}")
    return out


def _session_failures(s: dict) -> list:
    name, out = s["session"], []
    tele = (s.get("modes") or {}).get("TELEOP", {})
    if tele.get("pacing_drops"):
        out.append({"rule": "pacing_drops", "session": name, "value": tele["pacing_drops"]})
    if (tele.get("entry_not_ok_s") or 0.0) * 1e3 > GAP_MAX_MS or tele.get("stretches_never_ok"):
        out.append({"rule": "entry_not_ok", "session": name, "value": tele.get("entry_not_ok_s", 0.0) * 1e3,
                    "never_ok": tele.get("stretches_never_ok", 0)})
    for h in (h for h in s["holds"] if h.get("mode_kind") == "direct" and h.get("covered", True)):
        lab = h["label"]
        if lab in DRIVE_HOLDS and (h["output_share"] is None or h["output_share"] < OUTPUT_MIN):
            out.append({"rule": "hold_output", "session": name, "hold": lab, "value": h["output_share"]})
        if lab in DRIVE_HOLDS and (h.get("longest_quiet_after_output_ms") or 0.0) > GAP_MAX_MS:
            out.append({"rule": "output_stutter", "session": name, "hold": lab,
                        "value": h["longest_quiet_after_output_ms"]})
        if h.get("longest_not_ok_ms") is None or h["longest_not_ok_ms"] > GAP_MAX_MS:
            out.append({"rule": "pacing_gap", "session": name, "hold": lab, "value": h.get("longest_not_ok_ms")})
        if lab in TURN_HOLDS and h["ok_at_start"] and (h["first_output_ms"] is None
                                                       or h["first_output_ms"] > TURN_FIRST_MAX_MS):
            out.append({"rule": "turn_first_output", "session": name, "hold": lab, "value": h["first_output_ms"]})
    safe = s.get("safety") or {}
    bad = {k: safe.get(k) for k in ("scene_damaged", "forbidden_pocket_contacts", "unpermitted_auto",
                                    "teleop_overlap_ticks") if safe.get(k)}
    if bad:
        out.append({"rule": "safety", "session": name, "value": bad})
    return out


def verdict(sessions: list, *, factor: float, overlap_max: float = 0.05, slots: list | None = None) -> dict:
    """The registered rules (plan 3). Every slot (a registered session; its one rerun, ``<name>_re``, stands in only
    when the original is over the overlap rule) must hold one session that is valid and judged, and ``slots`` of
    them must be judged; a judged session passes when TELEOP has no pacing failure (an ok tick followed by a not-ok
    one) and enters not ok for at most 100 ms, every direct drive hold outputs at least 0.95 of [start, end) and,
    once it has output, never stops for over 100 ms, no direct hold (driving or lifting) has a pacing not-ok stretch
    over 100 ms, a turn hold that starts ok answers within 100 ms, and no safety record. Anything unevaluated never
    passes."""
    failures, unevaluated, used, left_out = [], [], {}, []
    by_slot: dict = {}
    for s in sessions:
        by_slot.setdefault(slot_of(s["session"]), []).append(s)
    if slots is not None and (len(set(slots)) != SLOTS_PER_ARM or len(slots) != SLOTS_PER_ARM):
        unevaluated.append({"problem": f"{SLOTS_PER_ARM} distinct registered names needed, given {list(slots)}"})
    if slots is not None:  # exactly the registered names: no other arm's, no stray rerun, none missing
        for slot in sorted(set(by_slot) - set(slots)):
            unevaluated.append({"slot": slot, "problem": "not a registered slot of this arm"})
        for slot in sorted(set(slots) - set(by_slot)):
            unevaluated.append({"slot": slot, "problem": "no session"})
    for slot, group in sorted(by_slot.items()):
        if slots is not None and slot not in slots:
            continue
        names = sorted(s["session"] for s in group)
        if names not in ([slot], [slot, f"{slot}_re"]):
            unevaluated.append({"slot": slot, "problem": f"sessions {names}: the original and at most one rerun"})
            continue
        group.sort(key=lambda s: s["session"].endswith("_re"))  # the original first
        chosen = None
        for s in group:
            ov = s.get("overlap")
            if ov is not None and ov > overlap_max:
                left_out.append(s["session"])
                continue
            chosen = s
            break
        if chosen is None:
            unevaluated.append({"slot": slot, "problem": "every run of the slot over the overlap rule"})
            continue
        problems = _session_problems(chosen)
        if problems:
            unevaluated.append({"slot": slot, "session": chosen["session"], "problems": problems})
            continue
        used[slot] = chosen["session"]
        failures += _session_failures(chosen)
    if not used:
        unevaluated.append({"problem": "no slot judged"})
    judged = [s for s in sessions if s["session"] in used.values()]
    return {"factor": factor, "pass": not failures and not unevaluated, "failures": failures,
            "unevaluated": unevaluated, "judged": used, "left_out": left_out,
            "windows": {s["session"]: s.get("windows") for s in judged}}


def _med(values):
    v = [x for x in values if x is not None]
    return None if not v else {"median": statistics.median(v), "min": min(v), "max": max(v), "n": len(v)}


def summarize(sessions: list, overlap_max: float = 0.05) -> dict:
    kept = [s for s in sessions if (s["overlap"] or 0.0) <= overlap_max]
    out = {"sessions": len(sessions), "kept": [s["session"] for s in kept],
           "left_out": {s["session"]: s["overlap"] for s in sessions if s not in kept}, "modes": {}, "holds": {},
           "timers": {}}
    for m in ("AUTO", "TELEOP"):
        out["modes"][m] = {f: _med([s["modes"][m][f] for s in kept])
                           for f in ("ratio", "ratio_span", "ok_ticks", "ok_wall", "longest_not_ok_s", "gap_wall_s",
                                     "sim_s")}
    labels = sorted({h["label"] for s in kept for h in s["holds"]})
    for lab in labels:
        hs = [h for s in kept for h in s["holds"] if h["label"] == lab]
        out["holds"][lab] = {"n": len(hs), "ok_share": _med([h["ok_share"] for h in hs]),
                             "output_share": _med([h["output_share"] for h in hs]),
                             "first_output_ms": _med([h["first_output_ms"] for h in hs]),
                             "no_output": sum(1 for h in hs if h["first_output_ms"] is None),
                             "started_not_ok": sum(1 for h in hs if h["ok_at_start"] is False),
                             "output_at_first_ok": sum(1 for h in hs if h["ok_at_start"] is False
                                                       and h["first_output_ms"] is not None
                                                       and h["first_output_ms"] == h["first_ok_ms"]),
                             "lift_delta_m": _med([h["lift_delta_m"] for h in hs]) if hs and hs[0]["lift_dir"] else None}
    for m in ("AUTO", "TELEOP"):
        keys = sorted({k for s in kept for k in s["timers"].get(m, {})})
        out["timers"][m] = {k: {f: _med([s["timers"][m][k][f] for s in kept if k in s["timers"].get(m, {})])
                                for f in ("s_per_s", "calls_per_s", "mean_ms", "max_ms")} for k in keys}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--overlap-max", type=float, default=0.05)
    ap.add_argument("--factor", type=float, help="the sessions' declared factor (pacing windows; needed by --verdict)")
    ap.add_argument("--verdict", action="store_true", help="judge by the smoothness confirmation's registered rules")
    ap.add_argument("--slots", help="the arm's registered session names, comma separated (--verdict; required)")
    ap.add_argument("--json")
    args = ap.parse_args(argv)
    if args.verdict and (args.factor is None or not args.slots):
        ap.error("--verdict needs --factor and --slots")
    per = [session(Path(p), args.factor) for p in args.sessions]
    out = {"summary": summarize(per, args.overlap_max), "sessions": per}
    if args.verdict:
        out["verdict"] = verdict(per, factor=args.factor, overlap_max=args.overlap_max,
                                 slots=[x for x in args.slots.split(",") if x])
        # the summary of the sessions the verdict judged (a rerun it did not pick is left out; Codex round 3 P3)
        picked = set(out["verdict"]["judged"].values())
        out["summary"] = summarize([s for s in per if s["session"] in picked], args.overlap_max)
    text = json.dumps(out, indent=1, default=float)
    if args.json:
        Path(args.json).write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
