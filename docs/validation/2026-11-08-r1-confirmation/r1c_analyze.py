"""Real-time plan 5.17 R1 confirmation: the registered measures T1-T6 per session and the per-branch rules.

    PYTHONPATH=SNAP/src python r1c_analyze.py SNAP R1C_DIR [--json out.json]

R1C_DIR holds one folder per session (r1c_session.sh's OUT, with args.txt naming SET, ARM and FACTOR) and the
bundle's results under */bundle/results.jsonl. The runner's records are read from OUT/live/epoch_1/run: the live log
(read_live_log: status lines per control tick with speed_mps, pacing_ok, mode, armed; the link log's processed
requests), the tick trace (each physics step's paced start S, world.step's end, the step's sim tick) and the spool
(slam_log.joint: per physics step the wheel rates and the true base pose). The probe's report holds its STOP send
times, teleop hold intervals and the expiry test (tools/live_console_probe.py confirm scenario).

T1 teleop: per hold, the control ticks inside it -- the share with pacing_ok false (tick count and wall weighted),
   the share with a non-zero output speed -- and the true motion over it (distance, heading change, the largest
   lateral departure from the start-end chord, the lift change); TELEOP entries refused for pacing.
T2 expiry: expired / stale_video seen in the probe's states during normal holds; the boundary test's wall from the
   silence's start to the first state showing `expired` and to the first zero-speed status line.
T3 STOP (A1, A2, T): (1) send -> processed (the link log's line of the same request id); (2) processed -> command zero
   as a bracket [the last status line with a non-zero speed, the first with zero]; (3) processed -> stop complete:
   the first physics step of the first 0.2 sim s window in which every wheel's |rate| stays below 0.01 rad/s, at that
   step's world.step end. Whether the processing fell in a catch-up burst (a step started early) is tagged.
T4 pixel (mark sessions): tools/live_marker_lag.py's pixel-inclusive latency of the conforming frames.
T5 AUTO: the mission result, the ratio and ok over AUTO control ticks only (STOP and teleop out), the 0.25 s gaps
   of AUTO steps, the safety counts (forbidden contacts, unpermitted entries) from the run's result.json.
T6 validity: instrumentation / infrastructure failures (trace dropped, spool or live log refused, launcher or probe
   error) make a session invalid; functional failures stay in the denominator.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

SNAP = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(SNAP / "sim" / "isaac"))
sys.path.insert(0, str(SNAP / "src"))
from tick_trace import COLUMNS  # noqa: E402
from forklift_core.live.spool import load_group, read_live_log  # noqa: E402

DT = 1 / 120
C = {n: i for i, n in enumerate(COLUMNS)}
STILL_RAD_S, STILL_SIM_S = 0.01, 0.2
PIXEL_LAG_S_DEFAULT_PLUS_STEP = 0.2 + 1 / 120  # --live-camera-pixel-lag-s default 0.2 + one step: 0.520833 s at 0.4


def read_args(out: Path) -> dict:
    a = {}
    for line in (out / "args.txt").read_text().splitlines():
        if "=" in line and not line.startswith(("EXTRA_ARGS", "PROBE")):
            k, v = line.split("=", 1)
            a[k] = v
    return a


class Session:
    def __init__(self, out: Path):
        self.out = out
        self.args = read_args(out)
        self.set, self.arm, self.factor = self.args.get("SET"), self.args.get("ARM"), float(self.args.get("FACTOR", "nan"))
        self.run = out / "live" / "epoch_1" / "run"
        self.invalid = []
        self.report = json.loads((out / "probe.json").read_text()) if (out / "probe.json").exists() else {}
        self.confirm = self.report.get("confirm") or {}
        try:
            self.log = read_live_log(self.run)
        except (OSError, ValueError) as exc:
            self.log = []
            self.invalid.append(f"live log: {exc}")
        self.status = [r for r in self.log if "pacing_ok" in r and r.get("mono") is not None and "speed_mps" in r]
        self.links = [r for r in self.log if r.get("where") is not None and r.get("t") is not None]
        self.result = None
        if (self.run / "result.json").exists():
            self.result = json.loads((self.run / "result.json").read_text())
        saved = (self.result or {}).get("tick_trace") or {}
        if saved.get("error") or saved.get("dropped", 1) != 0 or not (self.run / "tick_trace.npz").exists():
            self.invalid.append("tick trace missing, errored or dropped")
            self.trace = None
        else:
            with np.load(self.run / "tick_trace.npz") as z:
                self.trace = z["data"]
        self.joint = None
        try:
            self.joint = load_group(self.run / "spool", "slam_log.joint")
        except (OSError, ValueError) as exc:
            if self.set in ("far", "off"):
                self.invalid.append(f"spool joint: {exc}")

    # --- clocks: wall <-> sim through the tick trace -----------------------------------------------------------------
    def row_at_mono(self, mono):
        S = self.trace[:, C["S"]]
        i = int(np.searchsorted(S, mono, side="right")) - 1
        return max(0, min(i, len(S) - 1))

    def sim_of_row(self, i):
        return float(self.trace[i, C["tick"]] * DT)

    def row_of_sim(self, sim):
        ticks = self.trace[:, C["tick"]]
        i = int(np.searchsorted(ticks, round(sim / DT), side="left"))
        return max(0, min(i, len(ticks) - 1))

    def early(self, i):
        if i <= 0:
            return False
        S = self.trace[:, C["S"]]
        return bool(S[i] - S[i - 1] < DT / self.factor - 1e-9)

    # --- T3 ----------------------------------------------------------------------------------------------------------
    def stop_measures(self, rec):
        out = {"label": rec.get("label"), "moving_at_send": rec.get("moving_at_send"), "reply": rec.get("reply")}
        rid = rec.get("request_id")
        proc = next((r["mono"] for r in self.links if r.get("t") == "stop" and r.get("request_id") == rid), None)
        if proc is None or rec.get("sent_mono") is None:
            out["error"] = "no processed stop line"
            return out
        out["send_to_processed_s"] = proc - rec["sent_mono"]
        after = [r for r in self.status if r["mono"] >= proc]
        first_zero = next((r for r in after if (r.get("speed_mps") or 0.0) == 0.0), None)
        if first_zero is not None:
            before = [r for r in self.status if r["mono"] < first_zero["mono"]]
            lower = max(before[-1]["mono"], proc) if before else proc
            out["processed_to_zero_s"] = [lower - proc, first_zero["mono"] - proc]
        if self.trace is not None and self.joint is not None and "wheel_rates_rad_s" in self.joint:
            i0 = self.row_at_mono(proc)
            out["in_burst"] = self.early(i0)
            t_proc = self.sim_of_row(i0)
            stamps = np.asarray(self.joint["joint_stamps_s"], float)
            rates = np.abs(np.asarray(self.joint["wheel_rates_rad_s"], float)).reshape(len(stamps), -1).max(axis=1)
            idx = np.flatnonzero(stamps >= t_proc)
            still = rates < STILL_RAD_S
            for k in idx:
                win = (stamps >= stamps[k]) & (stamps < stamps[k] + STILL_SIM_S)
                if still[win].all() and stamps[win].max() >= stamps[k] + STILL_SIM_S - 2 * DT:
                    r = self.row_of_sim(stamps[k])
                    out["processed_to_still_s"] = float(self.trace[r, C["world_end"]] - proc)
                    break
        return out

    # --- T1 ----------------------------------------------------------------------------------------------------------
    def hold_measures(self, rec):
        a, b = rec.get("start_mono"), rec.get("end_mono")
        ticks = [r for r in self.status if a <= r["mono"] <= b]
        out = {"label": rec.get("label"), "mode": rec.get("mode"), "ticks": len(ticks)}
        if ticks:
            blocked = np.array([not r.get("pacing_ok") for r in ticks])
            monos = np.array([r["mono"] for r in ticks] + [b])
            w = np.diff(monos)
            out["pacing_blocked_tick_share"] = float(blocked.mean())
            out["pacing_blocked_wall_share"] = float((w * blocked).sum() / w.sum()) if w.sum() > 0 else None
            out["moving_output_share"] = float(np.mean([(r.get("speed_mps") or 0.0) != 0.0 for r in ticks]))
        if self.trace is not None and self.joint is not None and "base_pose_world" in self.joint:
            s0, s1 = self.sim_of_row(self.row_at_mono(a)), self.sim_of_row(self.row_at_mono(b))
            stamps = np.asarray(self.joint["joint_stamps_s"], float)
            pose = np.asarray(self.joint["base_pose_world"], float).reshape(len(stamps), -1)
            m = (stamps >= s0) & (stamps <= s1 + 3.0)  # the stand after the hold included (it ends on zero commands)
            if m.sum() >= 2:
                xy, yaw = pose[m, :2], pose[m, 2]
                chord = xy[-1] - xy[0]
                n = np.linalg.norm(chord)
                out["distance_m"] = float(n)
                out["heading_change_rad"] = float(math.atan2(math.sin(yaw[-1] - yaw[0]), math.cos(yaw[-1] - yaw[0])))
                if n > 1e-6:
                    rel = xy - xy[0]
                    out["lateral_max_m"] = float(np.max(np.abs(rel[:, 0] * chord[1] - rel[:, 1] * chord[0]) / n))
        s_a, s_b = rec.get("start_state") or {}, rec.get("end_state") or {}
        if s_a.get("lift_m") is not None and s_b.get("lift_m") is not None:
            out["lift_change_m"] = float(s_b["lift_m"] - s_a["lift_m"])
        if "stop" in rec:
            out["stop"] = self.stop_measures(rec["stop"])
        return out

    # --- T2 ----------------------------------------------------------------------------------------------------------
    def expiry_measures(self):
        rec = self.confirm.get("expiry")
        if not rec:
            return None
        t0 = rec["silence_start_mono"]
        states = self.report.get("states") or []
        first_exp = next((s.get("mono") for s in states if (s.get("mono") or 0) >= t0 and s.get("reject") == "expired"),
                         None)
        first_zero = next((r["mono"] for r in self.status if r["mono"] >= t0 and (r.get("speed_mps") or 0.0) == 0.0),
                          None)
        return {"moving_before": rec.get("moving_before"), "silence_s": rec.get("silence_s"),
                "to_expired_s": None if first_exp is None else first_exp - t0,
                "to_zero_s": None if first_zero is None else first_zero - t0,
                "armed_after": rec.get("armed_after"), "reject_after": rec.get("last_teleop_reject_after")}

    def normal_expiries(self):
        holds = [(h["start_mono"], h["end_mono"]) for h in self.confirm.get("holds") or []]
        n = {"expired": 0, "stale_video": 0}
        prev = None
        for s in self.report.get("states") or []:
            r = s.get("reject")  # the probe's state record: last_teleop_reject as "reject"
            mono = s.get("mono")
            if r in n and r != prev and mono is not None and any(a <= mono <= b for a, b in holds):
                n[r] += 1
            prev = r
        return n

    # --- T5 ----------------------------------------------------------------------------------------------------------
    def auto_measures(self):
        auto = [r for r in self.status if r.get("mode") == "AUTO" and r.get("tick_t") is not None]
        out = {}
        if len(auto) > 1:
            sim = wall = 0.0
            for r0, r1 in zip(auto, auto[1:]):
                d_s, d_w = r1["tick_t"] - r0["tick_t"], r1["mono"] - r0["mono"]
                if 0 < d_s < 0.5 and 0 < d_w < 5.0:  # consecutive AUTO ticks only (a STOP or teleop breaks the run)
                    sim, wall = sim + d_s, wall + d_w
            out["auto_ratio"] = sim / wall if wall > 0 else None
            oks = [bool(r["pacing_ok"]) for r in auto]
            out["auto_ok_share_after_first"] = float(np.mean(oks[oks.index(True):])) if True in oks else None
        if self.trace is not None and auto:
            S = self.trace[:, C["S"]]
            itv = np.r_[S[1:] - S[:-1], np.nan]
            lo, hi = auto[0]["mono"], auto[-1]["mono"]
            m = (S >= lo) & (S <= hi) & (itv > 0.25)
            out["gaps_over_0_25"] = int(m.sum())
        res = self.confirm.get("mission_result") or {}
        out["mission"] = {"outcome": res.get("outcome"), "reason": res.get("reason")}
        steps = {s["step"]: s["ok"] for s in self.confirm.get("steps") or []}
        out["failed_after_resume"] = res.get("outcome") not in (None, "done") and bool(steps.get("resumed"))
        r = self.result or {}
        ol = r.get("obstacle_layer") or {}
        out["safety"] = {"forbidden_contacts": len(r.get("forbidden_pocket_contacts") or []),
                         "unpermitted_entries": ol.get("unpermitted_entries"),
                         "unpermitted_dir_entries": ol.get("unpermitted_dir_entries"),
                         "result_json": self.result is not None}
        return out

    def analyse(self):
        out = {"name": self.out.name, "set": self.set, "arm": self.arm, "factor": self.factor, "invalid": self.invalid,
               "steps": self.confirm.get("steps")}
        if self.set in ("far", "off"):
            out["T3"] = [self.stop_measures(s) for s in self.confirm.get("stops") or [] if s.get("label") in ("A1", "A2")]
            out["T1"] = [self.hold_measures(h) for h in self.confirm.get("holds") or []]
            out["T1_entries"] = [{k: e.get(k) for k in ("entered", "pacing_refusals", "not_stopped_refusals")}
                                 for e in self.confirm.get("teleop_entries") or []]
            out["T2"] = {"boundary": self.expiry_measures(), "normal": self.normal_expiries()}
            out["T5"] = self.auto_measures()
        elif self.set == "mark":
            out["T4"] = marker(self)
        elif self.set == "wall":
            out["wall"] = wall_measures(self)
        return out


def marker(s: Session):
    frames = s.out / "frames"
    if not frames.is_dir():
        return {"error": "no frames"}
    try:
        p = subprocess.run([sys.executable, str(SNAP / "tools" / "live_marker_lag.py"), "--report", str(s.out / "probe.json"),
                            "--frames-dir", str(frames), "--live-log", str(s.run / "live_log.jsonl")],
                           capture_output=True, text=True, timeout=600)
        res = json.loads(p.stdout)
    except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
        return {"error": repr(exc)}
    freeze = (s.report.get("freeze") or {})
    disarmed = any(x.get("armed") is False and x.get("reject") == "stale_video" for x in freeze.get("samples") or [])
    return {"pixel": res.get("pixel_latency_wall_s_conforming_frames"), "lag_sim_s": res.get("pixel_lag_sim_s"),
            "frames": res.get("frames"), "freeze_disarmed_stale_video": disarmed}


def wall_measures(s: Session):
    """The approach probe's wall stop (R-g): per forward leg whether the vehicle held the truck short of the wall with
    the command still asking to go, the permission it showed, and the true base x where the leg ended (the leg's
    leg_done event mapped to sim time through the tick trace; the base pose from the spool)."""
    legs = s.report.get("approach") or []
    done = {e.get("leg"): e.get("mono") for e in s.report.get("events") or [] if e.get("event") == "leg_done"}
    out = []
    for leg in legs:
        row = {k: leg.get(k) for k in ("leg", "held", "moved", "permit", "forbidden")}
        mono = done.get(leg.get("leg"))
        if mono is not None and s.trace is not None and s.joint is not None and "base_pose_world" in s.joint:
            sim = s.sim_of_row(s.row_at_mono(mono))
            stamps = np.asarray(s.joint["joint_stamps_s"], float)
            pose = np.asarray(s.joint["base_pose_world"], float).reshape(len(stamps), -1)
            k = int(np.clip(np.searchsorted(stamps, sim), 0, len(stamps) - 1))
            row["base_x_m"] = float(pose[k, 0])
        out.append(row)
    return out


def bracket_rule(k_rows, l_rows, h):
    """R-b (1)+(2): pass when the candidate's upper bound max <= the baseline's lower bound max + h; fail when the
    candidate's lower bound max > the baseline's upper bound max + h; otherwise hold (resolution)."""
    def bounds(rows):
        lo = [r["send_to_processed_s"] + r["processed_to_zero_s"][0] for r in rows if "processed_to_zero_s" in r]
        hi = [r["send_to_processed_s"] + r["processed_to_zero_s"][1] for r in rows if "processed_to_zero_s" in r]
        return (max(lo), max(hi)) if lo else (None, None)
    k_lo, k_hi = bounds(k_rows)
    l_lo, l_hi = bounds(l_rows)
    if None in (k_lo, l_lo):
        return "hold (no measure)"
    if k_hi <= l_lo + h:
        return "pass"
    if k_lo > l_hi + h:
        return "fail"
    return "hold (resolution)"


FORWARD_LABELS = {"forward": 0.45, "reverse": 0.45, "video_forward": 0.45, "video_reverse": 0.45}  # 0.15 m/s x 3 s
STEER_LABELS = {"left": 0.18 * 0.30, "right": 0.18 * 0.30}  # curvature x distance (0.15 m/s x 2 s)
LIFT_LABELS = {"lift_up": 0.02 * 2.0, "lift_down": 0.02 * 2.0}  # the manual lift rate x 2 s


def verdict(rows, branch, factor, candidate):
    """Plan 5.17's rules for one branch and factor: baseline validity, function (R-a..R-f), gain; 'change',
    'no gain', 'hold' or 'undecidable' with the reasons."""
    set_ = "far" if branch == "far" else "off"
    sess = [r for r in rows if r["set"] == set_ and r["factor"] == factor and not r["invalid"]]
    L = [r for r in sess if r["arm"] == "L"]
    K = [r for r in sess if r["arm"] == candidate]
    why = []
    done = lambda rs: sum((r.get("T5") or {}).get("mission", {}).get("outcome") == "done" for r in rs)  # noqa: E731
    need = 2 if set_ == "far" else 1
    if done(L) < need:
        return {"verdict": "undecidable", "why": [f"baseline completed {done(L)} < {need}"]}

    def holds(rs, label):
        return [h for r in rs for h in r.get("T1") or [] if h["label"] == label]

    for label, cmd in {**FORWARD_LABELS}.items():
        if not holds(L, label) or np.median([abs(h.get("distance_m") or 0) for h in holds(L, label)]) < 0.5 * cmd:
            return {"verdict": "undecidable", "why": [f"baseline exposure short: {label}"]}
    for label, cmd in STEER_LABELS.items():
        if not holds(L, label) or np.median([abs(h.get("heading_change_rad") or 0) for h in holds(L, label)]) < 0.5 * cmd:
            return {"verdict": "undecidable", "why": [f"baseline exposure short: {label}"]}
    for label, cmd in LIFT_LABELS.items():
        if not holds(L, label) or np.median([abs(h.get("lift_change_m") or 0) for h in holds(L, label)]) < 0.5 * cmd:
            return {"verdict": "undecidable", "why": [f"baseline exposure short: {label}"]}
    # exposure of the candidate's required checks: at least two sessions each
    stops_k = [x for r in K for x in r.get("T3") or [] if "send_to_processed_s" in x]
    tstops_k = [h["stop"] for h in holds(K, "forward") if "stop" in h]
    expiry_k = [r["T2"]["boundary"] for r in K if (r.get("T2") or {}).get("boundary")]
    if min(len({x["label"] for x in stops_k}) and len(stops_k), len(tstops_k), len(expiry_k), len(holds(K, "forward"))) < 2:
        return {"verdict": "hold", "why": ["candidate exposure short (fewer than two sessions of a required check)"]}
    # R-a safety
    for r in K:
        sf = (r.get("T5") or {}).get("safety") or {}
        if sf.get("forbidden_contacts") or sf.get("unpermitted_entries") or sf.get("unpermitted_dir_entries"):
            why.append(f"R-a safety in {r['name']}")
    if any("processed_to_still_s" not in x for x in stops_k + tstops_k):
        why.append("R-a a STOP without a stop-complete")
    # R-b
    h = 1 / (120 * factor)
    stops_l = [x for r in L for x in r.get("T3") or [] if "send_to_processed_s" in x] + \
        [hh["stop"] for hh in holds(L, "forward") if "stop" in hh]
    rb = bracket_rule(stops_k + tstops_k, stops_l, h)
    if rb != "pass":
        why.append(f"R-b {rb}")
    still = lambda xs: max((x["processed_to_still_s"] for x in xs if "processed_to_still_s" in x), default=None)  # noqa
    if still(stops_k + tstops_k) is not None and still(stops_l) is not None and \
            still(stops_k + tstops_k) > still(stops_l) + 0.1 / factor:
        why.append("R-b stop-complete")
    # R-c
    mos = lambda rs, label: [hh.get("moving_output_share") for hh in holds(rs, label) if hh.get("moving_output_share") is not None]  # noqa
    for label in FORWARD_LABELS:
        if mos(K, label) and mos(L, label) and np.median(mos(K, label)) < np.median(mos(L, label)):
            why.append(f"R-c moving output {label}")
        dl = np.median([abs(hh.get("distance_m") or 0) for hh in holds(L, label)])
        if any(abs(abs(hh.get("distance_m") or 0) - dl) > 0.25 * dl for hh in holds(K, label)):
            why.append(f"R-c distance {label}")
        ll = max((hh.get("lateral_max_m") or 0 for hh in holds(L, label)), default=0)
        if any((hh.get("lateral_max_m") or 0) > ll + 0.03 for hh in holds(K, label)):
            why.append(f"R-c lateral {label}")
    for label in STEER_LABELS:
        hl = np.median([abs(hh.get("heading_change_rad") or 0) for hh in holds(L, label)])
        if any(abs(abs(hh.get("heading_change_rad") or 0) - hl) > 0.25 * hl for hh in holds(K, label)):
            why.append(f"R-c heading {label}")
    for label in LIFT_LABELS:
        ml = np.median([abs(hh.get("lift_change_m") or 0) for hh in holds(L, label)])
        if any(abs(abs(hh.get("lift_change_m") or 0) - ml) > 0.10 * ml for hh in holds(K, label)):
            why.append(f"R-c lift {label}")
    # R-d
    norm = lambda rs: sum(sum((r.get("T2") or {}).get("normal", {}).values()) for r in rs)  # noqa: E731
    if norm(K) > norm(L):
        why.append("R-d normal expiries")
    bound = (0.30 - (6 if candidate in ("K", "KP") else 0) / 120) / factor + 2 * h
    for e in expiry_k:
        if e.get("to_expired_s") is None or e.get("to_zero_s") is None or e["to_expired_s"] > bound:
            why.append("R-d boundary expiry")
            break
    # R-e pixel (mark sessions of the candidate at this factor): p99 within the runner's configured bound,
    # (live_camera_pixel_lag_s + 1/120) / f (sim/isaac/run_transport.py live_arbiter_config; the default lag 0.2 s)
    marks = [r for r in rows if r["set"] == "mark" and r["arm"] == candidate and r["factor"] == factor and not r["invalid"]]
    if not marks:
        why.append("R-e no marker session")
    for r in marks:
        px = ((r.get("T4") or {}).get("pixel") or {})
        if px.get("p99") is None or px["p99"] > PIXEL_LAG_S_DEFAULT_PLUS_STEP / factor:
            why.append(f"R-e pixel p99 {px.get('p99')}")
        if not (r.get("T4") or {}).get("freeze_disarmed_stale_video"):
            why.append("R-e/R-d freeze not refused stale_video")
    # R-g wall stop (0.4 only): the candidate held short of the wall on the forward legs, its end x within 0.05 m of L
    if factor == 0.4:
        walls = {r["arm"]: r.get("wall") or [] for r in rows if r["set"] == "wall" and not r["invalid"]}
        wl, wk = walls.get("L"), walls.get(candidate)
        if not wl or not wk:
            why.append("R-g no wall session")
        else:
            for leg in ("forward_to_wall", "forward_again"):
                a_ = next((x for x in wl if x.get("leg") == leg), {})
                b_ = next((x for x in wk if x.get("leg") == leg), {})
                if not b_.get("held") or a_.get("base_x_m") is None or b_.get("base_x_m") is None or \
                        abs(b_["base_x_m"] - a_["base_x_m"]) > 0.05:
                    why.append(f"R-g {leg}")
    # R-f
    if done(K) < done(L):
        why.append("R-f fewer completions")
    reasons = lambda rs: {(r.get("T5") or {}).get("mission", {}).get("reason") for r in rs} - {None}  # noqa
    if reasons(K) - reasons(L):
        return {"verdict": "hold", "why": why + [f"R-f new failure reasons {sorted(reasons(K) - reasons(L))}"]}
    if why:
        return {"verdict": "hold" if any("hold" in w for w in why) else "no change (function)", "why": why}
    # gain
    med = lambda rs, f: np.median([f(r) for r in rs if f(r) is not None]) if [f(r) for r in rs if f(r) is not None] else None  # noqa
    ok = lambda r: (r.get("T5") or {}).get("auto_ok_share_after_first")  # noqa: E731
    ratio = lambda r: (r.get("T5") or {}).get("auto_ratio")  # noqa: E731
    blocked = lambda r: np.mean([hh["pacing_blocked_tick_share"] for hh in r.get("T1") or [] if "pacing_blocked_tick_share" in hh]) if r.get("T1") else None  # noqa
    gains = []
    if None not in (med(K, ok), med(L, ok)) and med(K, ok) >= med(L, ok) + 0.02:
        gains.append("ok")
    if None not in (med(K, blocked), med(L, blocked)) and med(K, blocked) <= med(L, blocked) - 0.02:
        gains.append("pacing blocked")
    if None not in (med(K, ratio), med(L, ratio)) and med(K, ratio) >= med(L, ratio) + 0.01 * factor:
        gains.append("ratio")
    return {"verdict": "change" if gains else "no gain", "gains": gains, "why": why}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("snap")
    ap.add_argument("r1c", type=Path)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    rows = [Session(p).analyse() for p in sorted(a.r1c.iterdir()) if p.is_dir() and (p / "args.txt").exists()]
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("name", "set", "arm", "factor", "invalid")}),
              json.dumps(r.get("T5") or r.get("T4") or r.get("wall"), default=str)[:300])
    verdicts = {}
    for branch, factor, cand in (("far", 0.4, "K"), ("far", 0.3, "K"), ("off", 0.4, "KP")):
        v = verdict(rows, branch, factor, cand)
        verdicts[f"{branch}@{factor}"] = v
        print("verdict", branch, factor, json.dumps(v, default=str))
    if a.json:
        a.json.write_text(json.dumps({"sessions": rows, "verdicts": verdicts}, indent=1, default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
