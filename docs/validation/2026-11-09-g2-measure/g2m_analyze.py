"""Real-time plan 5.22: the G2-0 / G3-1a / G5-0 measurement run's measures M1-M7 (descriptive; M3's completion line and
M7's cost line are the only verdicts). Run from a checkout (its sim/isaac and src give the trace and the live-log reader):

    PYTHONPATH=SNAP/src python g2m_analyze.py OUT_DIR --list OUT_DIR.jsonl --bench BENCH.json [--json out.json]

Clocks: the tick trace, the long-call record and the live log's mono are one clock (perf_counter / CLOCK_MONOTONIC on
Linux). Rows: a long call's "row" is the trace row its work belongs to (a control tick's is the row of the step it
precedes; plan 5.18 G2-0). A start gap S[i+1] - S[i] holds step i's parts (world, after-step processing, camera read),
the control tick of row i+1, the wait and eps.

Timer names in the measurement mode: ``name<caller`` (by_caller) and ``name|parent`` (by_parent, the innermost
enclosing region's full key); a parent's direct children are the keys whose part after the first ``|`` is that parent.
"""

from __future__ import annotations

import argparse
import glob
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
START_WINDOW_S = 10.0  # M6: a start stall is a gap over EDGE_S within this of the first control tick
INSTR_ON = {"FORKLIFT_LIVE_LONG_CALLS_MS": "20", "FORKLIFT_LIVE_LIDAR_PROFILE_EVERY": "20"}
FORBIDDEN = ("FORKLIFT_PERF_DUMP", "FORKLIFT_KERNELS_DUAL")
REFRESH = "layer.refresh<obstacle_scan"
REGISTERED = {"H": {"on": 3, "off": 2}, "F": {"on": 3, "off": 2}}


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


def head(key: str) -> str:
    return key.split("|", 1)[0]


def parent_of(key: str) -> str | None:
    return key.split("|", 1)[1] if "|" in key else None


class Session:
    def __init__(self, out: Path, entry: dict):
        self.out, self.name, self.env = out, entry["name"], entry["env"]
        self.set = "H" if "LEVEL" in self.env else "F"
        self.instr = self.env.get("INSTR", "off")
        self.warm = self.name.endswith("_warm")
        self.run = out / "live" / "epoch_1" / "run"
        self.invalid = []
        self.cond = read_conditions(out)
        self._check_conditions()
        try:
            self.log = read_live_log(self.run)
        except (OSError, ValueError) as exc:
            self.log = []
            self.invalid.append(f"live log: {exc}")
        self.cmd = [r for r in self.log if "pacing_ok" in r and "speed_mps" in r and r.get("mono") is not None]
        self.timers = [r for r in self.log if r.get("status") and isinstance(r.get("timers"), dict)
                       and r.get("mono") is not None]
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
        if self.instr == "on":
            f = self.run / "long_calls.json"
            if not f.exists():
                self.invalid.append("no long_calls.json")
            else:
                self.long = json.loads(f.read_text())
                if self.long.get("dropped", 0):
                    self.invalid.append(f"long calls dropped {self.long['dropped']}")
        rc = (out / "probe_rc").read_text().strip() if (out / "probe_rc").exists() else None
        if rc != "0":
            self.invalid.append(f"probe exit {rc}")
        self._check_monitor()
        self.lifecycle = next((r.get("marks") for r in self.log if r.get("event") == "lifecycle"), None)
        self.mission_done = self._mission_done()

    def _check_conditions(self):
        c = self.cond
        for k, v in INSTR_ON.items():
            got = c.get(k)
            if self.instr == "on" and got != v:
                self.invalid.append(f"{k}={got}, registered {v}")
            if self.instr != "on" and got is not None:
                self.invalid.append(f"{k} set in an INSTR {self.instr} session")
        for k in c:
            if k.startswith(FORBIDDEN):
                self.invalid.append(f"{k} set (dumps and the dual run stay out of the measurement)")
        if self.set == "H":
            want = {"FACTOR": "1.0", "CATCHUP": "6", "PHASE": "6", "LEVEL": "off"}
        else:
            want = {"SET": "far", "ARM": "K", "FACTOR": "0.4", "FORKLIFT_LIVE_CATCHUP_STEPS": "6"}
        for k, v in want.items():
            if c.get(k) != v:
                self.invalid.append(f"{k}={c.get(k)}, registered {v}")

    def _check_monitor(self):
        f = self.out / "monitor" / "host.jsonl"
        if not f.exists():
            self.invalid.append("no monitor record")
            return
        for line in f.read_text().splitlines():
            r = json.loads(line)
            for p in r.get("gpu_procs") or []:
                if len(p) > 3 and p[3] is False:
                    self.invalid.append(f"another GPU process {p[2]} (pid {p[0]})")
                    return

    def _mission_done(self) -> bool | None:
        report = self.out / "probe.json"
        if not report.exists():
            return None
        j = json.loads(report.read_text())
        res = (j.get("confirm") or {}).get("mission_result") or j.get("result") or {}
        outcome = res.get("outcome") if isinstance(res, dict) else None
        return None if outcome is None else outcome == "done"

    # --- helpers -------------------------------------------------------------------------------------------------
    def auto_rows(self) -> np.ndarray:
        steps = {int(r["step"]) for r in self.cmd if r.get("mode") == "AUTO" and r.get("step") is not None}
        live = self.trace[:, C["live_step"]]
        return np.array([np.isfinite(s) and int(s) in steps for s in live])

    def first_ctrl_row(self) -> int:
        return int(np.argmax(np.isfinite(self.trace[:, C["ctrl_start"]])))

    def auto_windows(self):
        """Status-line windows (between consecutive timer lines) whose every command line is AUTO: the timers'
        population matched to AUTO (plan 5.22 review P2-2). Returns [(start mono, end mono, sim s, timers)]."""
        out = []
        cmd_mono = np.array([r["mono"] for r in self.cmd])
        cmd_auto = np.array([r.get("mode") == "AUTO" for r in self.cmd])
        for a, b in zip(self.timers, self.timers[1:]):
            sel = (cmd_mono > a["mono"]) & (cmd_mono <= b["mono"])
            if sel.any() and cmd_auto[sel].all():
                out.append((a["mono"], b["mono"], b["tick_t"] - a["tick_t"], b["timers"]))
        return out

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
                        # the gap less the four measured parts: the wait, eps and the unmeasured joins between them
                        "outside_parts_s": float(gap - known),
                        "long_calls_self_s": float(sum(r["self_s"] for r in calls if not r["path"])),
                        "top_calls": [{k: r.get(k) for k in ("name", "path", "self_s", "cpu_s", "phase",
                                                              "run_delay_s_since_mark", "since_mark_s")}
                                      for r in top]})
        return out

    def m2_calls(self) -> dict:
        if self.long is None:
            return {}
        auto = self.auto_rows()
        groups = {"auto": defaultdict(list), "rest": defaultdict(list)}
        for r in self.long["records"]:
            row = r.get("row")
            key = "auto" if (r.get("thread") is None and row is not None and 0 <= row < len(auto) and auto[row]) \
                else "rest"
            groups[key][r["name"]].append(r)
        out = {}
        for g, by_name in groups.items():
            rows = {}
            for name, rs in by_name.items():
                delays = [r["run_delay_s_since_mark"] for r in rs if r.get("run_delay_s_since_mark") is not None]
                rows[name] = {"n": len(rs), "self_s": float(sum(r["self_s"] for r in rs)),
                              "max_self_s": float(max(r["self_s"] for r in rs)),
                              # the wait since the phase's mark up to the call's end: a median and a maximum,
                              # never a sum (one wait is counted again by every call of its phase)
                              "run_delay_since_mark_median_s": float(np.median(delays)) if delays else None,
                              "run_delay_since_mark_max_s": float(max(delays)) if delays else None,
                              "control": sum(r.get("phase") == "control" for r in rs),
                              "step": sum(r.get("phase") == "step" for r in rs),
                              "other_thread": sum(bool(r.get("thread")) for r in rs)}
            out[g] = dict(sorted(rows.items(), key=lambda kv: -kv[1]["self_s"]))
        return out

    def m3_scan(self) -> dict:
        D = self.trace
        bits = np.nan_to_num(D[:, C["flags"]]).astype(int)
        scan = (bits & BIT["scan"]) != 0
        rest = D[:, C["step_end"]] - D[:, C["world_end"]]
        w = work(D)
        auto = self.auto_rows()
        ws = w[auto & scan & np.isfinite(w)]
        out = {"scan_step_W_ms": {"p50": float(np.median(ws) * 1e3), "p99": float(np.percentile(ws, 99) * 1e3),
                                  "max": float(ws.max() * 1e3)} if len(ws) else None}
        win = self.auto_windows()
        if not win:
            out["windows"] = 0
            return out
        S = D[:, C["S"]]
        sim = sum(x[2] for x in win)
        rows = np.zeros(len(D), bool)
        totals = defaultdict(float)
        for a, b, _, timers in win:
            rows |= (S > a) & (S <= b)
            for k, v in timers.items():
                totals[k] += v[0]
        per = {k: v / sim for k, v in totals.items()}
        share = float(np.nansum(rest[rows & scan]) / sim)
        refresh_keys = [k for k in per if head(k) == REFRESH]
        refresh = sum(per[k] for k in refresh_keys)
        children = {k: per[k] for k in per if parent_of(k) in refresh_keys}
        lidar = sum(v for k, v in per.items() if head(k) == "lidar.cast")
        add = sum(v for k, v in per.items() if head(k).startswith("layer.add_scans"))
        out.update({
            "windows": len(win), "sim_s": sim,
            # the AUTO windows' scan steps: step_end - world_end (5.20's denominator)
            "scan_step_rest_s_per_sim_s": share,
            "refresh_s_per_sim_s": refresh, "lidar_cast_s_per_sim_s": lidar, "add_scans_s_per_sim_s": add,
            "refresh_children_s_per_sim_s": dict(sorted(children.items(), key=lambda kv: -kv[1])),
            "refresh_self_s_per_sim_s": refresh - sum(children.values()),
            # 5.20 G3-1a's completion: the scan-step share less what the timers attribute (LiDAR, adding the scans,
            # the refresh and its children are inside the refresh) at most 0.01 s per sim s
            "scan_step_unattributed_s_per_sim_s": share - (refresh + lidar + add),
        })
        out["g3_1a_split_complete"] = abs(out["scan_step_unattributed_s_per_sim_s"]) <= 0.01
        for k in list(children):  # one level further: each child's own direct children
            grand = {g: per[g] for g in per if parent_of(g) == k}
            if grand:
                out.setdefault("refresh_grandchildren_s_per_sim_s", {})[k] = dict(
                    sorted(grand.items(), key=lambda kv: -kv[1]))
        return out

    def m4_lidar(self) -> dict:
        totals, scans = defaultdict(float), 0
        plain_s, plain_n = 0.0, 0
        for r in self.timers:
            for k, v in r["timers"].items():
                if head(k).startswith("lidar.split."):
                    totals[head(k)] += v[0]
                    if head(k) == "lidar.split.scans":
                        scans += v[1]
                elif head(k) == "lidar.cast":
                    plain_s, plain_n = plain_s + v[0], plain_n + v[1]
        out = {k: totals[k] / scans * 1e3 for k in totals} if scans else {}
        if scans and plain_n:
            # lidar.cast holds every scan, the profiled ones too: their extra time is their mean less the others'
            others = (plain_s - totals["lidar.split.scans"]) / max(plain_n - scans, 1)
            out["profiled_extra_s_total"] = totals["lidar.split.scans"] - scans * others
        return out

    def m5_corrections(self) -> dict:
        """The share of scans whose layer correction version differs from the scan before (obstacle.scans), and of SLAM
        records whose applied correction differs. A cache keyed on the correction can hit at most on the rest
        (1 - share); its real rate also depends on its other keys (plan 5.22 review P3)."""

        def items(group):
            out = []
            for f in sorted(glob.glob(str(self.run / "spool" / group / "*.jsonl"))):
                out += [json.loads(line) for line in open(f)]
            return sorted(out, key=lambda r: r.get("stamp_s", 0.0))

        scans, recs = items("obstacle.scans"), items("slam.records")
        v = [r.get("version") for r in scans]
        ap = [tuple(r.get("applied_after") or ()) for r in recs]
        return {"scans": len(scans),
                "version_change_share": (sum(a != b for a, b in zip(v, v[1:])) / len(v)) if v else None,
                "slam_records": len(recs),
                "applied_change_share": (sum(a != b for a, b in zip(ap, ap[1:])) / len(ap)) if ap else None}

    def m6_start(self) -> dict:
        S = self.trace[:, C["S"]]
        first = self.first_ctrl_row()
        first_tick = float(S[first])
        ok = next((r["mono"] for r in self.cmd if r.get("pacing_ok")), None)
        marks = {m["mark"]: m["mono"] for m in (self.lifecycle or [])}
        runner = marks.get("runner_main")
        stalls = [g for g in self.m1_gaps() if g["gap_s"] > EDGE_S and g["t_from_first_s"] <= START_WINDOW_S]
        return {"marks_from_runner_s": {k: v - runner for k, v in marks.items()} if runner else None,
                "first_tick_from_runner_s": first_tick - runner if runner else None,
                "first_ok_from_runner_s": ok - runner if (ok and runner) else None,
                "first_ok_from_first_tick_s": ok - first_tick if ok else None,
                "start_stalls": stalls}

    def m7_work(self, bench: dict | None) -> dict:
        D = self.trace
        auto = self.auto_rows()
        sim_auto = auto.sum() * DT
        sim_all = len(D) * DT
        w = work(D)
        nx = D[np.r_[np.arange(1, len(D)), len(D) - 1]]
        parts = {"W": w, "world": D[:, C["world_end"]] - D[:, C["world_start"]],
                 "step_rest": D[:, C["step_end"]] - D[:, C["world_end"]],
                 "after": D[:, C["after_end"]] - D[:, C["step_end"]],
                 "control": nx[:, C["ctrl_end"]] - nx[:, C["ctrl_start"]]}
        out = {"auto_s_per_sim_s": {k: float(np.nansum(v[auto]) / sim_auto) for k, v in parts.items()},
               "whole_W_s_per_sim_s": float(np.nansum(w) / sim_all), "mission_done": self.mission_done}
        if self.long is not None and bench:
            # the accounting bound, per whole-trace sim s: every entered region call's added cost (measured on this
            # node by g2m_bench.py, the larger of the on-record and the long-only added costs), the records' keeping
            # and the profiled LiDAR scans' extra time
            per_call = max(bench["added_on_vs_off_us"], bench["long_only_added_us"]) * 1e-6
            calls = self.long.get("region_calls", 0)
            lidar = self.m4_lidar().get("profiled_extra_s_total", 0.0)
            bound = calls * per_call + self.long.get("overhead_s", 0.0) + max(lidar, 0.0)
            out["accounting"] = {"region_calls": calls, "per_call_us": per_call * 1e6,
                                 "keeping_s": self.long.get("overhead_s"), "lidar_extra_s": lidar,
                                 "bound_s_per_sim_s": bound / sim_all,
                                 "bound_share_of_W": bound / sim_all / out["whole_W_s_per_sim_s"]}
        return out

    def analyse(self, bench) -> dict:
        row = {"name": self.name, "set": self.set, "instr": self.instr, "warm": self.warm, "invalid": self.invalid}
        if self.trace is None or self.invalid:
            return row
        row.update({"M1": self.m1_gaps(), "M2": self.m2_calls(), "M3": self.m3_scan(), "M4": self.m4_lidar(),
                    "M5": self.m5_corrections(), "M6": self.m6_start(), "M7": self.m7_work(bench)})
        return row


def m7_verdict(rows, names_by_set) -> dict:
    """Per set: complete only with every registered session valid and its mission done (3 on, 2 off). The verdict is
    the accounting bound (each on session's bound <= 1 % of W); the A/B mean difference is reported beside it with the
    off-off spread, and flagged when it exceeds the largest bound by more than that spread (plan 5.22 review P2-3/4)."""
    out = {}
    for set_, want in REGISTERED.items():
        sel = {i: [r for r in rows if r.get("set") == set_ and r.get("instr") == i and not r.get("warm")]
               for i in ("on", "off")}
        missing = [n for n in names_by_set[set_] if not any(r["name"] == n and not r["invalid"] for r in rows)]
        failed = [r["name"] for r in sel["on"] + sel["off"] if r.get("M7", {}).get("mission_done") is False]
        if missing or failed or any(len([r for r in sel[i] if not r["invalid"]]) != want[i] for i in want):
            out[set_] = {"verdict": "incomplete", "missing_or_invalid": missing, "mission_failed": failed}
            continue
        on, off = [r["M7"] for r in sel["on"]], [r["M7"] for r in sel["off"]]
        bounds = [r["accounting"]["bound_share_of_W"] for r in on if "accounting" in r]
        w_on = [r["auto_s_per_sim_s"]["W"] for r in on]
        w_off = [r["auto_s_per_sim_s"]["W"] for r in off]
        ab = (np.mean(w_on) - np.mean(w_off)) / np.mean(w_off)
        spread = (max(w_off) - min(w_off)) / np.mean(w_off)
        verdict = "cost <= 1 %" if bounds and max(bounds) <= 0.01 else (
            "undetermined (no accounting)" if not bounds else "cost > 1 % by the accounting bound")
        out[set_] = {"verdict": verdict, "bound_share_of_W": bounds, "ab_rel_diff": float(ab),
                     "off_off_spread": float(spread),
                     "ab_inconsistent": bool(bounds) and ab > max(bounds) + spread}
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--list", type=Path, required=True)
    ap.add_argument("--bench", type=Path, help="g2m_bench.py's JSON from the same node")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    bench = json.loads(a.bench.read_text()) if a.bench else None
    entries = [json.loads(line) for line in a.list.read_text().splitlines() if line.strip()]
    names_by_set = {"H": [e["name"] for e in entries if "LEVEL" in e["env"] and not e["name"].endswith("_warm")],
                    "F": [e["name"] for e in entries if "LEVEL" not in e["env"]]}
    rows = []
    for e in entries:
        d = a.out_dir / e["name"]
        rows.append(Session(d, e).analyse(bench) if d.exists() else {"name": e["name"], "invalid": ["missing"]})
    summary = {"sessions": rows, "M7": m7_verdict(rows, names_by_set)}
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("name", "set", "instr", "invalid")}))
    print("M7", json.dumps(summary["M7"]))
    if a.json:
        a.json.write_text(json.dumps(summary, indent=1, default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
