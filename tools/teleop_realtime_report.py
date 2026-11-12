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

The summary leaves out sessions whose overlap is over ``--overlap-max`` (0.05: the latency plan's 1e (c) rule).

    python tools/teleop_realtime_report.py SESSION_DIR... [--overlap-max 0.05] [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

GAP_S = 1.0
OVERLAP_CORES = 1.0


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
        out[m] = {"ratio": sim / wall if wall else None, "ratio_span": span_sim / span_wall if span_wall else None,
                  "sim_s": sim, "gap_wall_s": gap, "ok_ticks": ok_n / n if n else None,
                  "ok_wall": ok_wall / tot_wall if tot_wall else None, "longest_not_ok_s": longest}
    return out


def _holds(probe: dict, ticks: list) -> list:
    states = sorted(probe.get("states") or [], key=lambda s: s["mono"])
    out = []
    for h in (probe.get("confirm") or {}).get("holds") or []:
        t0, t1 = h.get("start_mono"), h.get("end_mono")
        if t0 is None or t1 is None or t1 <= t0:
            continue
        win = [r for r in ticks if t0 <= r["mono"] <= t1]
        pairs = list(zip(win, win[1:]))
        wall = sum(b["mono"] - a["mono"] for a, b in pairs)
        first_out = next((r for r in win if abs(r.get("speed_mps") or 0.0) > 1e-6), None)
        first_ok = next((r for r in win if r["pacing_ok"]), None)
        until = first_out["mono"] if first_out else t1
        reasons = sorted({str(s.get("teleop_forbidden")) for s in states
                          if t0 <= s["mono"] <= until and s.get("teleop_forbidden")})
        ss, es = h.get("start_state") or {}, h.get("end_state") or {}

        def ms(r):
            return None if r is None else (r["mono"] - t0) * 1e3

        out.append({"label": h.get("label"), "lift_dir": h.get("lift_dir"), "mode_kind": h.get("mode"),
                    "wall_s": wall, "ok_at_start": win[0]["pacing_ok"] if win else None,
                    "ok_share": (sum(b["mono"] - a["mono"] for a, b in pairs if a["pacing_ok"]) / wall
                                 if wall else None),
                    "output_share": (sum(b["mono"] - a["mono"] for a, b in pairs
                                         if abs(a.get("speed_mps") or 0.0) > 1e-6) / wall if wall else None),
                    "first_ok_ms": ms(first_ok), "first_output_ms": ms(first_out), "reasons_before_output": reasons,
                    "lift_delta_m": (es["lift_m"] - ss["lift_m"]
                                     if ss.get("lift_m") is not None and es.get("lift_m") is not None else None)})
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


def session(sd: Path) -> dict:
    sd = Path(sd)
    name = sd.parent.name if sd.name == "s" else sd.name
    recs = _load(sd / "live" / "epoch_1" / "run" / "live_log.jsonl")
    ticks = sorted((r for r in recs if {"tick_t", "mode", "pacing_ok", "mono"} <= set(r)), key=lambda r: r["mono"])
    probe = json.loads((sd / "probe.json").read_text(encoding="utf-8"))
    ps = sd.parent.parent / f"{name}_ps.jsonl" if sd.name == "s" else sd.parent / f"{name}_ps.jsonl"
    rows = [r for r in _load(ps) if "other_cores" in r] if ps.exists() else []
    overlap = sum(1 for r in rows if r["other_cores"] > OVERLAP_CORES) / len(rows) if rows else None
    return {"session": name, "modes": _modes(ticks), "holds": _holds(probe, ticks), "timers": _timers(recs, ticks),
            "overlap": overlap}


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
    ap.add_argument("--json")
    args = ap.parse_args(argv)
    per = [session(Path(p)) for p in args.sessions]
    out = {"summary": summarize(per, args.overlap_max), "sessions": per}
    text = json.dumps(out, indent=1, default=float)
    if args.json:
        Path(args.json).write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
