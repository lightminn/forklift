"""Real-time plan 5.22: the G2-0 / G3-1a / G5-0 measurement run's measures M1-M7 (descriptive; no verdict but M7's
1 % line). Run from a checkout (its sim/isaac and src give the trace and the live-log reader):

    PYTHONPATH=SNAP/src python g2m_analyze.py OUT_DIR --list OUT_DIR.jsonl [--json out.json]

Clocks: the tick trace, the long-call record and the live log's mono are one clock (perf_counter / CLOCK_MONOTONIC on
Linux). Rows: a long call's "row" is the trace row its work belongs to (a control tick's is the row of the step it
precedes; plan 5.18 G2-0). W_i = q[i+1] - S[i] holds step i and the control tick before step i+1, so a start gap
S[i+1] - S[i] is step i's parts, the next row's control tick, the wait and eps.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
sys.path.insert(0, str(ROOT / "src"))
from tick_trace import COLUMNS, FLAGS, work  # noqa: E402
from forklift_core.live.spool import read_live_log  # noqa: E402

DT = 1 / 120
C = {n: i for i, n in enumerate(COLUMNS)}
BIT = {n: 1 << i for i, n in enumerate(FLAGS)}
GAP_S = 0.20
EDGE_S = 0.25


def read_conditions(out: Path) -> dict:
    a = {}
    for name in ("rt_conditions.txt", "args.txt"):
        f = out / name
        if f.exists():
            for line in f.read_text().splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    if k.replace("_", "").isalnum():
                        a.setdefault(k, v)
    return a


class Session:
    def __init__(self, out: Path, entry: dict):
        self.out, self.name, self.env = out, entry["name"], entry["env"]
        self.set = "H" if "LEVEL" in self.env else "F"
        self.instr = self.env.get("INSTR", "off")
        self.warm = self.name.endswith("_warm")
        self.run = out / "live" / "epoch_1" / "run"
        self.invalid = []
        self.cond = read_conditions(out)
        want_on = self.instr == "on"
        has_on = "FORKLIFT_LIVE_LONG_CALLS_MS" in self.cond
        if want_on != has_on:
            self.invalid.append(f"INSTR {self.instr} but the recorded environment {'has' if has_on else 'lacks'} it")
        try:
            self.log = read_live_log(self.run)
        except (OSError, ValueError) as exc:
            self.log, _ = [], self.invalid.append(f"live log: {exc}")
        self.cmd = [r for r in self.log if "pacing_ok" in r and "speed_mps" in r and r.get("mono") is not None]
        self.timers = [r for r in self.log if r.get("status") and isinstance(r.get("timers"), dict)]
        res = self.run / "result.json"
        self.result = json.loads(res.read_text()) if res.exists() else {}
        tt = (self.result or {}).get("tick_trace") or {}
        self.trace = None
        if tt.get("error") or tt.get("dropped", 1) != 0 or not (self.run / "tick_trace.npz").exists():
            self.invalid.append("tick trace missing, errored or dropped")
        else:
            with np.load(self.run / "tick_trace.npz") as z:
                self.trace = z["data"]
        self.long = None
        if want_on:
            f = self.run / "long_calls.json"
            if not f.exists():
                self.invalid.append("no long_calls.json")
            else:
                lc = json.loads(f.read_text())
                if lc.get("dropped", 0):
                    self.invalid.append(f"long calls dropped {lc['dropped']}")
                self.long = lc
        self.lifecycle = next((r.get("marks") for r in self.log if r.get("event") == "lifecycle"), None)

    # --- helpers -------------------------------------------------------------------------------------------------
    def auto_rows(self) -> np.ndarray:
        steps = {int(r["step"]) for r in self.cmd if r.get("mode") == "AUTO" and r.get("step") is not None}
        live = self.trace[:, C["live_step"]]
        return np.array([np.isfinite(s) and int(s) in steps for s in live])

    def first_ctrl_row(self) -> int:
        return int(np.argmax(np.isfinite(self.trace[:, C["ctrl_start"]])))

    # --- measures ------------------------------------------------------------------------------------------------
    def m1_gaps(self) -> list:
        D = self.trace
        S = D[:, C["S"]]
        first = self.first_ctrl_row()
        by_row = defaultdict(list)
        for r in (self.long or {}).get("records", []):
            if r.get("thread") is None and r.get("row") is not None:
                by_row[int(r["row"])].append(r)
        out = []
        for i in range(first, len(D) - 1):
            gap = S[i + 1] - S[i]
            if not np.isfinite(gap) or gap <= GAP_S:
                continue
            parts = {"world": D[i, C["world_end"]] - D[i, C["world_start"]],
                     "step_rest": D[i, C["step_end"]] - D[i, C["world_end"]],
                     "after": D[i, C["after_end"]] - D[i, C["step_end"]],
                     "control_next": D[i + 1, C["ctrl_end"]] - D[i + 1, C["ctrl_start"]]}
            parts = {k: (float(v) if np.isfinite(v) else None) for k, v in parts.items()}
            known = sum(v for v in parts.values() if v is not None)
            calls = [r for r in by_row.get(i, []) if r.get("phase") == "step"] + \
                    [r for r in by_row.get(i + 1, []) if r.get("phase") == "control"]
            top = sorted(calls, key=lambda r: -r["self_s"])[:5]
            out.append({"row": i, "t_from_first_s": float(S[i] - S[first]), "gap_s": float(gap),
                        "edge_margin_s": float(gap - EDGE_S), "parts": parts,
                        "unexplained_s": float(gap - known),
                        "top_calls": [{k: r.get(k) for k in ("name", "path", "self_s", "cpu_s", "phase",
                                                              "run_delay_s_since_mark")} for r in top]})
        return out

    def m2_calls(self) -> dict:
        if self.long is None:
            return {}
        auto = self.auto_rows()
        agg = defaultdict(lambda: {"n": 0, "self_s": 0.0, "max_self_s": 0.0, "run_delay_s": 0.0, "auto_n": 0,
                                   "control": 0, "step": 0, "other_thread": 0})
        for r in self.long["records"]:
            a = agg[r["name"]]
            a["n"] += 1
            a["self_s"] += r["self_s"]
            a["max_self_s"] = max(a["max_self_s"], r["self_s"])
            a["run_delay_s"] += r.get("run_delay_s_since_mark") or 0.0
            if r.get("thread"):
                a["other_thread"] += 1
            else:
                a[r.get("phase", "step")] += 1
                row = r.get("row")
                if row is not None and 0 <= row < len(auto) and auto[row]:
                    a["auto_n"] += 1
        return dict(sorted(agg.items(), key=lambda kv: -kv[1]["self_s"]))

    def m3_scan(self) -> dict:
        D = self.trace
        auto = self.auto_rows()
        bits = np.nan_to_num(D[:, C["flags"]]).astype(int)
        scan = (bits & BIT["scan"]) != 0
        w = work(D)
        sim = auto.sum() * DT
        rest = D[:, C["step_end"]] - D[:, C["world_end"]]
        ws = w[auto & scan & np.isfinite(w)]
        out = {"scan_step_W_ms": {"p50": float(np.median(ws) * 1e3), "p99": float(np.percentile(ws, 99) * 1e3),
                                  "max": float(ws.max() * 1e3)} if len(ws) else None,
               "scan_step_rest_s_per_sim_s": float(np.nansum(rest[auto & scan]) / sim) if sim else None}
        # the status line's totals over the session (the inner steps split by parent; plan 5.20)
        totals = defaultdict(float)
        for r in self.timers:
            for k, v in r["timers"].items():
                totals[k] += v[0]
        span = (self.timers[-1]["tick_t"] - self.timers[0]["tick_t"]) if len(self.timers) > 1 else None
        if span:
            keep = {k: v / span for k, v in totals.items()
                    if k.startswith(("grid.", "shadow.", "layer.", "lidar.", "permission.update<refresh"))}
            parent = keep.get("layer.refresh<obstacle_scan")
            inner = {k: v for k, v in keep.items() if k.endswith("<layer.refresh<obstacle_scan")}
            out["timers_s_per_sim_s"] = dict(sorted(keep.items(), key=lambda kv: -kv[1]))
            if parent is not None:
                out["refresh_unattributed_s_per_sim_s"] = parent - sum(inner.values())
        return out

    def m4_lidar(self) -> dict:
        totals, scans = defaultdict(float), 0
        for r in self.timers:
            for k, v in r["timers"].items():
                if k.startswith("lidar.split."):
                    totals[k] += v[0]
                    if k == "lidar.split.scans":
                        scans += v[1]
        return {k: totals[k] / scans * 1e3 for k in totals} if scans else {}

    def m6_start(self) -> dict:
        S = self.trace[:, C["S"]]
        first_tick = float(S[self.first_ctrl_row()])
        ok = next((r["mono"] for r in self.cmd if r.get("pacing_ok")), None)
        marks = {m["mark"]: m["mono"] for m in (self.lifecycle or [])}
        runner = marks.get("runner_main")
        return {"marks_from_runner_s": {k: v - runner for k, v in marks.items()} if runner else None,
                "first_tick_from_runner_s": first_tick - runner if runner else None,
                "first_ok_from_runner_s": ok - runner if (ok and runner) else None,
                "first_ok_from_first_tick_s": ok - first_tick if ok else None}

    def m7_work(self) -> dict:
        D = self.trace
        auto = self.auto_rows()
        sim = auto.sum() * DT
        w = work(D)
        nx = D[np.r_[np.arange(1, len(D)), len(D) - 1]]
        parts = {"W": w, "world": D[:, C["world_end"]] - D[:, C["world_start"]],
                 "step_rest": D[:, C["step_end"]] - D[:, C["world_end"]],
                 "after": D[:, C["after_end"]] - D[:, C["step_end"]],
                 "control": nx[:, C["ctrl_end"]] - nx[:, C["ctrl_start"]]}
        out = {k: float(np.nansum(v[auto]) / sim) for k, v in parts.items()} if sim else {}
        if self.long is not None:
            out["long_overhead_s_per_sim_s"] = None  # filled from the end line below when present
        end = next((r for r in self.log if r.get("event") == "end"), {})
        lc = end.get("long_calls") or {}
        if "overhead_s" in lc and sim:
            out["long_overhead_s_per_sim_s"] = lc["overhead_s"] / (len(D) * DT)
        return out

    def analyse(self) -> dict:
        row = {"name": self.name, "set": self.set, "instr": self.instr, "warm": self.warm, "invalid": self.invalid}
        if self.trace is None or self.invalid:
            return row
        row.update({"M1": self.m1_gaps(), "M2": self.m2_calls(), "M3": self.m3_scan(), "M4": self.m4_lidar(),
                    "M6": self.m6_start(), "M7": self.m7_work()})
        return row


def m7_compare(rows) -> dict:
    """Per set, the mean of each AUTO work part on vs off and the relative difference of W; off-off spread beside."""
    out = {}
    for set_ in ("H", "F"):
        on = [r["M7"] for r in rows if r["set"] == set_ and r["instr"] == "on" and "M7" in r and not r["warm"]]
        off = [r["M7"] for r in rows if r["set"] == set_ and r["instr"] == "off" and "M7" in r and not r["warm"]]
        if not on or not off:
            out[set_] = {"verdict": "incomplete", "on": len(on), "off": len(off)}
            continue
        mean = lambda xs, k: float(np.mean([x[k] for x in xs]))  # noqa: E731
        rel = (mean(on, "W") - mean(off, "W")) / mean(off, "W")
        spread = (max(x["W"] for x in off) - min(x["W"] for x in off)) / mean(off, "W")
        out[set_] = {"on": {k: mean(on, k) for k in ("W", "world", "step_rest", "after", "control")},
                     "off": {k: mean(off, k) for k in ("W", "world", "step_rest", "after", "control")},
                     "W_rel_diff": rel, "off_off_spread": spread,
                     "verdict": "cost <= 1 %" if rel <= 0.01 else "cost > 1 %: see the keeping time and wrapper counts"}
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--list", type=Path, required=True)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    entries = [json.loads(line) for line in a.list.read_text().splitlines() if line.strip()]
    rows = []
    for e in entries:
        d = a.out_dir / e["name"]
        if not d.exists():
            rows.append({"name": e["name"], "invalid": ["missing"]})
            continue
        rows.append(Session(d, e).analyse())
    summary = {"sessions": rows, "M7": m7_compare([r for r in rows if "set" in r])}
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("name", "set", "instr", "invalid")}))
    print("M7", json.dumps(summary["M7"]))
    if a.json:
        a.json.write_text(json.dumps(summary, indent=1, default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
