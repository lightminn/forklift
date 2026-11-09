"""Real-time plan 5.17 R1 confirmation: the registered measures T1-T6 per session and the per-branch rules.

    PYTHONPATH=SNAP/src python SNAP/docs/validation/2026-11-08-r1-confirmation/r1c_analyze.py R1C_DIR --list R1C.jsonl \
        [--json out.json]

The snapshot is the checkout this file sits in (its tick_trace, forklift_core and tools/live_marker_lag.py are used).

R1C_DIR holds one folder per session (r1c_session.sh's OUT, args.txt naming SET, ARM and FACTOR); the bundle list
names the registered sessions -- a verdict is given only when every registered session of its branch is there and
valid (an invalid one waits for its supplement). The runner's records are read from OUT/live/epoch_1/run: the live log
(read_live_log: the status line of every control tick with speed_mps, pacing_ok, mode; the link log's processed
requests; the arbiter's events, a disarm with its reason among them), the tick trace (each physics step's paced start
S, world.step's end, the step's tick counter -- the step simulates [tick dt, (tick + 1) dt]) and the spool
(slam_log.joint: per physics step, stamped at the step's end, the wheel rates and the true base pose x, y, z, qw, qx,
qy, qz; read only when the spool's manifest is complete and holds every item). The probe's report holds the STOP send
times, the teleop holds (start, end, stand) and the expiry test (tools/live_console_probe.py confirm scenario).

T1 teleop: per hold, the control ticks between its start and end -- the share with pacing_ok false (tick count and wall
   weighted) and the share with a non-zero output speed -- and the true motion from its start to its stand (distance,
   heading change from the quaternion's yaw, the largest lateral departure from the start-stand chord, the lift
   change from the probe's states); TELEOP entries refused for pacing.
T2 expiry: the arbiter's disarm events with reason expired / stale_video inside normal holds; the boundary test's wall
   from the silence's start to the first `expired` disarm and to the first zero-speed status line.
T3 STOP (A1, A2, T): (1) send -> processed (the link log's line of the same request id); (2) processed -> command zero
   as a bracket [the last status line before the first zero one (not before processing), the first zero one];
   (3) processed -> stop complete: the first physics step of the first window of 0.2 sim s (both ends observed) in
   which every wheel's |rate| stays below 0.01 rad/s, at that step's world.step end; whether the processing fell in a
   catch-up burst (a step started early) is tagged.
T4 pixel (mark sessions): tools/live_marker_lag.py's whole result (lag, fit, nonconforming frames, the conforming
   frames' pixel-inclusive latency) and whether video commands across the camera freeze were refused stale_video.
T5 AUTO: the mission result; the ratio over pairs of adjacent AUTO status lines (a STOP or a teleop changes the mode
   and so leaves the pairs it touches out; a long AUTO tick stays in); ok over AUTO ticks after the first ok; the
   0.25 s start gaps of steps between AUTO status lines; safety -- forbidden contacts of the run and of every mission
   record (missions/*/records.json), unpermitted entries from the run's result.
T6 validity: instrumentation or infrastructure failures (trace dropped or missing, spool incomplete, live log refused,
   the probe's own error exit, no args) make a session invalid; functional failures (a mission failing, a step not
   reached because of it) stay in its denominator.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import re
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

SNAP = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SNAP / "sim" / "isaac"))
sys.path.insert(0, str(SNAP / "src"))
from tick_trace import COLUMNS  # noqa: E402
from forklift_core.live.spool import load_group, read_live_log  # noqa: E402

DT = 1 / 120
C = {n: i for i, n in enumerate(COLUMNS)}
STILL_RAD_S, STILL_SIM_S = 0.01, 0.2
PIXEL_LAG_S_DEFAULT_PLUS_STEP = 0.2 + 1 / 120  # (--live-camera-pixel-lag-s 0.2 + one step) / f: 0.520833 s at 0.4
# the registered holds (sim seconds x speed): forward stops at 1.5 sim s (STOP-T)
DISTANCE = {"forward": 0.15 * 1.5, "reverse": 0.15 * 3.0, "video_forward": 0.15 * 3.0, "video_reverse": 0.15 * 3.0}
HEADING = {"left": 0.18 * 0.15 * 2.0, "right": 0.18 * 0.15 * 2.0}
LIFT = {"lift_up": 0.02 * 2.0, "lift_down": 0.02 * 2.0}
HEADING_SIGN = {"left": 1.0, "right": -1.0}  # a positive curvature turns left (yaw grows)
LIFT_SIGN = {"lift_up": 1.0, "lift_down": -1.0}
REQUIRED_HOLDS = ("loaded_forward",) + tuple(DISTANCE) + tuple(HEADING) + tuple(LIFT)
# the arm's profile as the wrapper pins it (plan 5.17): catch-up, camera phase ("" = unset, the legacy rule)
PROFILE = {"L": ("0", ""), "K": ("6", ""), "KP": ("6", "6")}


def read_args(out: Path) -> dict:
    """args.txt's KEY=value lines, EXTRA_ARGS and PROBE included (the freeze window is read from EXTRA_ARGS; Codex
    r1c mark/wall review round 2 P2: dropping them left every real session without it)."""
    a = {}
    for line in (out / "args.txt").read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            if k.replace("_", "").isalnum():
                a[k] = v
    return a


def yaw_of(q):
    qw, qx, qy, qz = q
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def spool_group(root: Path, name: str):
    """A spooled group, refused unless the manifest says complete and its item count matches what was read."""
    manifest = json.loads((root / "manifest.json").read_text())
    if not manifest.get("complete"):
        raise ValueError("spool manifest not complete")
    group = load_group(root, name)
    if not group:
        raise ValueError(f"{name}: not in the spool")
    written = (manifest.get(name) or {}).get("items")
    n = len(next(iter(group.values())))
    if written is not None and n != written:
        raise ValueError(f"{name}: {n} items read, {written} written")
    return group


class Session:
    def __init__(self, out: Path):
        self.out = out
        self.invalid = []
        try:
            self.args = read_args(out)
        except OSError:
            self.args = {}
            self.invalid.append("no args.txt")
        self.set, self.arm = self.args.get("SET"), self.args.get("ARM")
        self.factor = float(self.args.get("FACTOR", "nan"))
        self.run = out / "live" / "epoch_1" / "run"
        rc = (out / "probe_rc").read_text().strip() if (out / "probe_rc").exists() else None
        if rc != "0":
            self.invalid.append(f"probe exit {rc}")
        self.report = json.loads((out / "probe.json").read_text()) if (out / "probe.json").exists() else {}
        self.confirm = self.report.get("confirm") or {}
        try:
            self.log = read_live_log(self.run)
        except (OSError, ValueError) as exc:
            self.log = []
            self.invalid.append(f"live log: {exc}")
        self.status = [r for r in self.log if "pacing_ok" in r and r.get("mono") is not None and "speed_mps" in r]
        self.links = [r for r in self.log if r.get("where") is not None and r.get("t") is not None]
        self.disarms = [r for r in self.log if r.get("arbiter") and r.get("event") == "disarm"]
        self.arbiter_events = [r for r in self.log if r.get("arbiter")]
        self.result = json.loads((self.run / "result.json").read_text()) if (self.run / "result.json").exists() else None
        saved = (self.result or {}).get("tick_trace") or {}
        self.trace = None
        if saved.get("error") or saved.get("dropped", 1) != 0 or not (self.run / "tick_trace.npz").exists():
            self.invalid.append("tick trace missing, errored or dropped")
        else:
            with np.load(self.run / "tick_trace.npz") as z:
                self.trace = z["data"]
        self.joint = None
        try:
            self.joint = spool_group(self.run / "spool", "slam_log.joint")
        except (OSError, ValueError) as exc:
            self.invalid.append(f"spool: {exc}")
        want = PROFILE.get(self.arm)
        got = (self.args.get("FORKLIFT_LIVE_CATCHUP_STEPS", ""), self.args.get("FORKLIFT_LIVE_CAMERA_PHASE", ""))
        if want is None or got != want:
            self.invalid.append(f"profile {got} is not arm {self.arm}'s {want}")
        self.mission_records = None
        mid = (self.confirm.get("submit") or {}).get("mission_id")
        if self.set in ("far", "off"):
            rec = self.run / "missions" / str(mid) / "records.json"
            try:
                self.mission_records = json.loads(rec.read_text()) if mid else None
            except (OSError, ValueError):
                self.mission_records = None
            if mid and self.mission_records is None:
                self.invalid.append(f"mission {mid}: records.json missing or unreadable")

    # --- clocks ------------------------------------------------------------------------------------------------------
    def row_at_mono(self, mono):
        S = self.trace[:, C["S"]]
        return int(np.clip(np.searchsorted(S, mono, side="right") - 1, 0, len(S) - 1))

    def start_sim(self, i):
        return float(self.trace[i, C["tick"]] * DT)

    def row_of_end_stamp(self, stamp):
        """The trace row of the physics step whose end is at sim ``stamp`` (its tick is stamp/dt - 1)."""
        ticks = self.trace[:, C["tick"]]
        return int(np.clip(np.searchsorted(ticks, round(stamp / DT) - 1, side="left"), 0, len(ticks) - 1))

    def early(self, i):
        S = self.trace[:, C["S"]]
        return bool(i > 0 and S[i] - S[i - 1] < DT / self.factor - 1e-9)

    def joint_arrays(self):
        stamps = np.asarray(self.joint["joint_stamps_s"], float)
        rates = np.abs(np.asarray(self.joint["wheel_rates_rad_s"], float)).reshape(len(stamps), -1).max(axis=1)
        pose = np.asarray(self.joint["base_pose_world"], float).reshape(len(stamps), -1)
        return stamps, rates, pose

    # --- T3 ----------------------------------------------------------------------------------------------------------
    def stop_measures(self, rec):
        out = {"label": rec.get("label"), "moving_at_send": rec.get("moving_at_send"), "reply": rec.get("reply")}
        rid = rec.get("request_id")
        proc = next((r["mono"] for r in self.links if r.get("t") == "stop" and r.get("request_id") == rid), None)
        if proc is None or rec.get("sent_mono") is None:
            out["error"] = "no processed stop line"
            return out
        out["send_to_processed_s"] = proc - rec["sent_mono"]
        first_zero = next((r for r in self.status if r["mono"] >= proc and (r.get("speed_mps") or 0.0) == 0.0), None)
        if first_zero is not None:
            before = [r["mono"] for r in self.status if r["mono"] < first_zero["mono"]]
            lower = max(before[-1], proc) if before else proc
            out["processed_to_zero_s"] = [lower - proc, first_zero["mono"] - proc]
        # the search closes at the rearm: a still window after it belongs to later driving, not to this STOP (R1
        # results: r1c_02's STOP-T rearmed 0.76 s after processing and its "still" came 48.6 s later)
        rearm = next((e.get("t", e.get("mono")) for e in self.arbiter_events
                      if e.get("event") == "mode" and e.get("reason") == "rearm"
                      and (e.get("t", e.get("mono")) or -1) >= proc), None)
        out["rearm_after_processed_s"] = None if rearm is None else rearm - proc
        if self.trace is not None and self.joint is not None:
            i0 = self.row_at_mono(proc)
            out["in_burst"] = self.early(i0)
            stamps, rates, _ = self.joint_arrays()
            still = rates < STILL_RAD_S
            out["still_before_rearm"] = False
            for k in np.flatnonzero(stamps > self.start_sim(i0)):
                r = self.row_of_end_stamp(stamps[k])
                if not self.trace[r, C["world_end"]] >= proc:
                    continue  # a step that ended before the STOP was processed is not its stop (Codex round 2 P2)
                if rearm is not None and self.trace[r, C["world_end"]] > rearm:
                    break  # rearmed before the wheels stood still: not this STOP's stop
                end = stamps[k] + STILL_SIM_S
                if stamps[-1] < end - 1e-9:
                    break  # the window's end was never observed
                if rearm is not None and self.trace[self.row_of_end_stamp(end), C["world_end"]] > rearm:
                    break  # the still window would end after the rearm: not seen before it
                win = (stamps >= stamps[k]) & (stamps <= end + 1e-9)
                if still[win].all():
                    v = float(self.trace[r, C["world_end"]] - proc)
                    if math.isfinite(v) and v >= 0:
                        out["processed_to_still_s"] = v
                        out["still_before_rearm"] = True
                    break
        return out

    # --- T1 ----------------------------------------------------------------------------------------------------------
    def hold_measures(self, rec):
        a, b = rec.get("start_mono"), rec.get("end_mono")
        stand = rec.get("stand_mono") or b
        ticks = [r for r in self.status if a <= r["mono"] <= b]
        out = {"label": rec.get("label"), "mode": rec.get("mode"), "ticks": len(ticks)}
        if ticks:
            blocked = np.array([not r.get("pacing_ok") for r in ticks])
            w = np.diff(np.array([r["mono"] for r in ticks] + [b]))
            out["pacing_blocked_tick_share"] = float(blocked.mean())
            out["pacing_blocked_wall_share"] = float((w * blocked).sum() / w.sum()) if w.sum() > 0 else None
            out["moving_output_share"] = float(np.mean([(r.get("speed_mps") or 0.0) != 0.0 for r in ticks]))
        if self.trace is not None and self.joint is not None:
            s0, s1 = self.start_sim(self.row_at_mono(a)), self.start_sim(self.row_at_mono(stand)) + DT
            stamps, _, pose = self.joint_arrays()
            m = (stamps > s0) & (stamps <= s1)
            if m.sum() >= 2:
                xy = pose[m, :2]
                yaw0, yaw1 = yaw_of(pose[m][0, 3:7]), yaw_of(pose[m][-1, 3:7])
                chord = xy[-1] - xy[0]
                n = float(np.linalg.norm(chord))
                out["distance_m"] = n
                out["heading_change_rad"] = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
                if n > 1e-6:
                    rel = xy - xy[0]
                    out["lateral_max_m"] = float(np.max(np.abs(rel[:, 0] * chord[1] - rel[:, 1] * chord[0]) / n))
        s_a, s_b = rec.get("start_state") or {}, rec.get("stand_state") or rec.get("end_state") or {}
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
        exp = next((d["t"] for d in self.disarms if d.get("t") is not None and d["t"] >= t0
                    and d.get("reason") == "expired"), None)
        zero = next((r["mono"] for r in self.status if r["mono"] >= t0 and (r.get("speed_mps") or 0.0) == 0.0), None)
        return {"moving_before": rec.get("moving_before"), "silence_s": rec.get("silence_s"),
                "to_expired_s": None if exp is None else exp - t0, "to_zero_s": None if zero is None else zero - t0}

    def normal_expiries(self):
        holds = [(h["start_mono"], h["end_mono"]) for h in self.confirm.get("holds") or []]
        n = {"expired": 0, "stale_video": 0}
        for d in self.disarms:
            if d.get("reason") in n and any(x <= (d.get("t") or -1) <= y for x, y in holds):
                n[d["reason"]] += 1
        return n

    # --- T5 ----------------------------------------------------------------------------------------------------------
    def auto_measures(self):
        st = [r for r in self.status if r.get("tick_t") is not None]
        out = {}
        sim = wall = 0.0
        for r0, r1 in zip(st, st[1:]):
            if r0.get("mode") == "AUTO" and r1.get("mode") == "AUTO":
                sim, wall = sim + (r1["tick_t"] - r0["tick_t"]), wall + (r1["mono"] - r0["mono"])
        out["auto_ratio"] = sim / wall if wall > 0 else None
        auto = [r for r in st if r.get("mode") == "AUTO"]
        oks = [bool(r["pacing_ok"]) for r in auto]
        out["auto_ok_share_after_first"] = float(np.mean(oks[oks.index(True):])) if True in oks else None
        if self.trace is not None and len(st) > 1:
            S = self.trace[:, C["S"]]
            itv = np.r_[S[1:] - S[:-1], np.nan]
            gaps = 0
            for r0, r1 in zip(st, st[1:]):
                if r0.get("mode") == "AUTO" and r1.get("mode") == "AUTO":
                    gaps += int(np.sum((S >= r0["mono"]) & (S < r1["mono"]) & (itv > 0.25)))
            out["gaps_over_0_25"] = gaps
        res = self.confirm.get("mission_result") or {}
        out["mission"] = {"outcome": res.get("outcome"), "reason": res.get("reason")}
        steps = {s["step"]: s["ok"] for s in self.confirm.get("steps") or []}
        out["failed_after_resume"] = res.get("outcome") not in (None, "done") and bool(steps.get("resumed"))
        r = self.result or {}
        ol = r.get("obstacle_layer") or {}
        mission_contacts = None
        if self.mission_records is not None:
            mission_contacts = len(((self.mission_records.get("state") or {}).get("forbidden_pocket_contacts")) or [])
        out["safety"] = {"forbidden_contacts_run": len(r.get("forbidden_pocket_contacts") or []),
                         "forbidden_contacts_missions": mission_contacts,
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
    return {"marker_lag": res, "pixel": res.get("pixel_latency_wall_s_conforming_frames"), **freeze_check(s)}


FREEZE_FRAME_SLACK_S = 0.15  # the probe's own rule (tools/live_console_probe.py): one 10 Hz camera period


def freeze_window(args: dict):
    m = re.search(r"camera-freeze\\? +([0-9.]+):([0-9.]+)", args.get("EXTRA_ARGS", ""))
    return (float(m.group(1)), float(m.group(2))) if m else None


def freeze_check(s):
    """R-e's freeze refusal, inside the freeze (Codex r1c mark/wall review P2): a stale_video disarm sample whose last
    frame is the one before the freeze and whose instant falls in the freeze's simulated window (the trace row at
    its wall time); and the session's end asked only after the last sample."""
    freeze = s.report.get("freeze") or {}
    samples = freeze.get("samples") or []
    window = freeze_window(s.args)
    hits = [x for x in samples if x.get("armed") is False and x.get("reject") == "stale_video"]
    out = {"freeze_window_sim_s": window, "stale_video_samples": len(hits)}
    inside = []
    if window is not None and s.trace is not None:
        for x in hits:
            t = s.start_sim(s.row_at_mono(x["mono"]))
            if window[0] <= t <= window[1] and (x.get("last_frame_sim") or 0.0) >= window[0] - FREEZE_FRAME_SLACK_S:
                inside.append(t)
    out["sample_disarm_sim_s"] = inside[0] if inside else None  # the probe's samples (kept as information)
    # R1 results: the judgement reads the runner's own events -- armed when the freeze began (the last arm or disarm
    # before it is an arm), then a stale_video disarm inside the freeze. The probe's samples miss it when the state's
    # last reject is overwritten (r1c_18), and a session disarmed before the freeze proves nothing.
    # Each event at its occurrence ("t", the monotonic clock), put on the simulated clock through the trace: "tick_t"
    # is the tick the event was flushed with, so a disarm at 29.99 sim s can carry 30.0 (Codex tools review P2-1).
    seq = []
    if s.trace is not None:
        for e in getattr(s, "arbiter_events", []):
            if e.get("event") in ("arm", "disarm") and e.get("t") is not None:
                seq.append({**e, "sim": s.start_sim(s.row_at_mono(e["t"]))})
    armed_at = None
    hit = None
    if window is not None and seq:
        before = [e for e in seq if e["sim"] < window[0]]
        armed_at = bool(before) and before[-1]["event"] == "arm"
        hit = next((e["sim"] for e in seq if e["event"] == "disarm" and e.get("reason") == "stale_video"
                    and window[0] <= e["sim"] <= window[1]), None)
    out["armed_at_freeze_start"] = armed_at
    out["freeze_disarm_sim_s"] = hit
    out["freeze_disarmed_stale_video"] = bool(armed_at) and hit is not None
    end = s.report.get("end_requested_mono")
    last = max((x["mono"] for x in samples), default=None)
    out["ended_after_measures"] = None if end is None or last is None else bool(end > last)
    return out


def wall_measures(s: Session):
    """The approach probe's wall stop (R-g): per leg whether it moved, whether the vehicle held it short with the
    command still asking to go, the permission's reason, and the true base x where the leg ended."""
    done = {e.get("leg"): e.get("mono") for e in s.report.get("events") or [] if e.get("event") == "leg_done"}
    rearms = defaultdict(int)
    for e in s.report.get("events") or []:
        if e.get("event") == "rearmed_in_leg":
            rearms[e.get("leg")] += 1
    disarms = [e for e in getattr(s, "arbiter_events", []) if e.get("event") == "disarm" and e.get("t") is not None]
    out = []
    prev_end = -math.inf
    for leg in s.report.get("approach") or []:
        row = {k: leg.get(k) for k in ("leg", "held", "moved", "forbidden", "armed")}
        end_ = done.get(leg.get("leg"))
        # the runner's disarms inside this leg and the probe's re-arms (Codex tools review P2-2): a leg recovered
        # after a disarm is not a continuous approach
        inside = [e.get("reason") for e in disarms if prev_end < e["t"] <= (end_ if end_ is not None else math.inf)]
        row["disarms"] = inside
        row["rearms"] = rearms.get(leg.get("leg"), 0)
        prev_end = end_ if end_ is not None else prev_end
        row["permit_reason"] = (leg.get("permit") or {}).get("reason")
        end = s.report.get("end_requested_mono")
        row["ended_after"] = None if end is None or done.get(leg.get("leg")) is None else bool(end > done[leg["leg"]])
        mono = done.get(leg.get("leg"))
        if mono is not None and s.trace is not None and s.joint is not None:
            stamps, _, pose = s.joint_arrays()
            k = int(np.clip(np.searchsorted(stamps, s.start_sim(s.row_at_mono(mono)) + DT), 0, len(stamps) - 1))
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


def holds(rs, label):
    return [h for r in rs for h in r.get("T1") or [] if h["label"] == label]


def stops(rs, label):
    if label == "T":
        return [h["stop"] for h in holds(rs, "forward") if "stop" in h and "send_to_processed_s" in h["stop"]]
    return [x for r in rs for x in r.get("T3") or [] if x["label"] == label and "send_to_processed_s" in x]


def verdict(rows, expected, branch, factor, candidate):
    """Plan 5.17's rules for one branch and factor: the fixed session set, baseline validity, the candidate's exposure,
    function (R-a..R-g), gain -> 'change', 'no gain', 'hold', 'no change (function)' or 'undecidable'."""
    set_ = "far" if branch == "far" else "off"
    want = [n for n, (s_, a_, f_) in expected.items() if s_ == set_ and float(f_) == factor and a_ in ("L", candidate)]
    want += [n for n, (s_, a_, f_) in expected.items() if s_ == "mark" and float(f_) == factor and a_ == candidate]
    if factor == 0.4:
        want += [n for n, (s_, a_, f_) in expected.items() if s_ == "wall" and a_ in ("L", candidate)]
    by = {r["name"]: r for r in rows}
    mismatch = [n for n in want if n in by and (by[n]["set"], by[n]["arm"], by[n]["factor"])
                != (expected[n][0], expected[n][1], float(expected[n][2]))]
    if mismatch:
        return {"verdict": "invalid (conditions differ from the list)", "why": mismatch}
    missing = [n for n in want if n not in by or by[n]["invalid"]]
    if missing:
        return {"verdict": "pending (supplement)", "why": missing}
    sess = [by[n] for n in want if by[n]["set"] == set_]
    L = [r for r in sess if r["arm"] == "L"]
    K = [r for r in sess if r["arm"] == candidate]
    done = lambda rs: sum((r.get("T5") or {}).get("mission", {}).get("outcome") == "done" for r in rs)  # noqa: E731
    need = 2 if set_ == "far" else 1
    if done(L) < need:
        return {"verdict": "undecidable", "why": [f"baseline completed {done(L)} < {need}"]}
    for label in REQUIRED_HOLDS:
        if len(holds(L, label)) < need:
            return {"verdict": "undecidable", "why": [f"baseline hold missing: {label}"]}
    for label, cmd in DISTANCE.items():
        if np.median([abs(h.get("distance_m") or 0) for h in holds(L, label)]) < 0.5 * cmd:
            return {"verdict": "undecidable", "why": [f"baseline exposure short: {label}"]}
    for label, cmd in HEADING.items():
        if np.median([HEADING_SIGN[label] * (h.get("heading_change_rad") or 0) for h in holds(L, label)]) < 0.5 * cmd:
            return {"verdict": "undecidable", "why": [f"baseline exposure short: {label}"]}
    for label, cmd in LIFT.items():
        if np.median([LIFT_SIGN[label] * (h.get("lift_change_m") or 0) for h in holds(L, label)]) < 0.5 * cmd:
            return {"verdict": "undecidable", "why": [f"baseline exposure short: {label}"]}
    # the candidate's exposure: every required check in at least two sessions
    exp_k = [r["T2"]["boundary"] for r in K if ((r.get("T2") or {}).get("boundary") or {}).get("moving_before")]
    short = [lab for lab in ("A1", "A2", "T") if len([x for x in stops(K, lab) if x.get("moving_at_send")]) < 2]
    short += [lab for lab in REQUIRED_HOLDS if len(holds(K, lab)) < 2]
    short += ["expiry"] if len(exp_k) < 2 else []
    if short:
        return {"verdict": "hold", "why": [f"candidate exposure short: {short}"]}
    why = []
    # R-a
    for r in K:
        sf = (r.get("T5") or {}).get("safety") or {}
        if sf.get("forbidden_contacts_run") or sf.get("forbidden_contacts_missions") or \
                sf.get("forbidden_contacts_missions") is None or sf.get("unpermitted_entries") or \
                sf.get("unpermitted_dir_entries"):
            why.append(f"R-a safety in {r['name']}")
    k_stops = stops(K, "A1") + stops(K, "A2") + stops(K, "T")
    l_stops = stops(L, "A1") + stops(L, "A2") + stops(L, "T")
    if any("processed_to_still_s" not in x for x in k_stops):
        why.append("R-a a STOP without a stop-complete")
    # R-b
    h = 1 / (120 * factor)
    rb = bracket_rule(k_stops, l_stops, h)
    if rb != "pass":
        why.append(f"R-b {rb}")
    ks = [x["processed_to_still_s"] for x in k_stops if "processed_to_still_s" in x]
    ls = [x["processed_to_still_s"] for x in l_stops if "processed_to_still_s" in x]
    if ks and ls and max(ks) > max(ls) + 0.1 / factor:
        why.append("R-b stop-complete")
    # R-c, every block and every measure (Codex round 3 P2): distance and signed heading within 25 % of the
    # baseline's median (or within 0.02 m / 0.02 rad of it when that median is that small), the moving-output share
    # not lower, the lateral departure no more than the baseline's largest + 0.03 m, the signed lift change within 10 %
    for label in REQUIRED_HOLDS:
        kh, lh = holds(K, label), holds(L, label)
        for key, floor in (("distance_m", 0.02), ("heading_change_rad", 0.02)):
            ref = float(np.median([x.get(key) or 0.0 for x in lh]))
            tol = max(0.25 * abs(ref), floor)
            if any(x.get(key) is None or abs(x[key] - ref) > tol for x in kh):
                why.append(f"R-c {key} {label}")
        if label in LIFT:
            ml = float(np.median([x.get("lift_change_m") or 0.0 for x in lh]))
            if any(x.get("lift_change_m") is None or abs(x["lift_change_m"] - ml) > 0.10 * abs(ml) for x in kh):
                why.append(f"R-c lift {label}")
            continue
        mk = [x.get("moving_output_share") for x in kh if x.get("moving_output_share") is not None]
        ml_ = [x.get("moving_output_share") for x in lh if x.get("moving_output_share") is not None]
        if not mk or (ml_ and np.median(mk) < np.median(ml_)):
            why.append(f"R-c moving output {label}")
        ll = max((x.get("lateral_max_m") or 0.0 for x in lh), default=0.0)
        if any(x.get("lateral_max_m") is None or x["lateral_max_m"] > ll + 0.03 for x in kh):
            why.append(f"R-c lateral {label}")
    # R-d
    norm = lambda rs: sum(sum((r.get("T2") or {}).get("normal", {}).values()) for r in rs)  # noqa: E731
    if norm(K) > norm(L):
        why.append("R-d normal expiries")
    bound = (0.30 - 6 / 120) / factor + 2 * h
    if any(e.get("to_expired_s") is None or e.get("to_zero_s") is None or e["to_expired_s"] > bound for e in exp_k):
        why.append("R-d boundary expiry")
    # R-e
    for r in [by[n] for n in want if by[n]["set"] == "mark"]:
        px = (r.get("T4") or {}).get("pixel") or {}
        if px.get("p99") is None or px["p99"] > PIXEL_LAG_S_DEFAULT_PLUS_STEP / factor:
            why.append(f"R-e pixel p99 {px.get('p99')}")
        if not (r.get("T4") or {}).get("freeze_disarmed_stale_video"):
            why.append("R-e freeze not refused stale_video")
    # R-g (0.4 only)
    if factor == 0.4:
        walls = {by[n]["arm"]: by[n].get("wall") or [] for n in want if by[n]["set"] == "wall"}
        for leg in ("forward_to_wall", "forward_again"):
            a_ = next((x for x in walls.get("L", []) if x.get("leg") == leg), {})
            b_ = next((x for x in walls.get(candidate, []) if x.get("leg") == leg), {})
            if not (b_.get("moved") and b_.get("held") and b_.get("permit_reason") == "occupied") or \
                    a_.get("base_x_m") is None or b_.get("base_x_m") is None or abs(b_["base_x_m"] - a_["base_x_m"]) > 0.05:
                why.append(f"R-g {leg}")
            elif a_.get("disarms") or b_.get("disarms"):
                why.append(f"R-g {leg} recovered after a disarm")  # not a continuous approach in one arm
    # R-f
    if done(K) < done(L):
        why.append("R-f fewer completions")
    reasons = lambda rs: {(r.get("T5") or {}).get("mission", {}).get("reason") for r in rs} - {None}  # noqa: E731
    if reasons(K) - reasons(L):
        return {"verdict": "hold", "why": why + [f"R-f new failure reasons {sorted(reasons(K) - reasons(L))}"]}
    if why:
        return {"verdict": "hold" if any("hold" in w for w in why) else "no change (function)", "why": why}

    # gain
    def med(rs, f):
        v = [f(r) for r in rs if f(r) is not None]
        return float(np.median(v)) if v else None
    ok = lambda r: (r.get("T5") or {}).get("auto_ok_share_after_first")  # noqa: E731
    ratio = lambda r: (r.get("T5") or {}).get("auto_ratio")  # noqa: E731

    def blocked(r):  # pooled over every hold's ticks (the registered share of control ticks)
        n = sum(x["ticks"] for x in r.get("T1") or [] if "pacing_blocked_tick_share" in x)
        b = sum(x["ticks"] * x["pacing_blocked_tick_share"] for x in r.get("T1") or [] if "pacing_blocked_tick_share" in x)
        return b / n if n else None
    gains = []
    if None not in (med(K, ok), med(L, ok)) and med(K, ok) >= med(L, ok) + 0.02:
        gains.append("ok")
    if None not in (med(K, blocked), med(L, blocked)) and med(K, blocked) <= med(L, blocked) - 0.02:
        gains.append("pacing blocked")
    if None not in (med(K, ratio), med(L, ratio)) and med(K, ratio) >= med(L, ratio) + 0.01 * factor:
        gains.append("ratio")
    return {"verdict": "change" if gains else "no gain", "gains": gains}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("r1c", type=Path)
    ap.add_argument("--list", type=Path, required=True, help="the registered bundle list (r1c_lists.py)")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    expected = {}
    for line in a.list.read_text().splitlines():
        e = json.loads(line)
        expected[e["name"]] = (e["env"]["SET"], e["env"]["ARM"], e["env"]["FACTOR"])
    rows = [Session(a.r1c / n).analyse() for n in expected if (a.r1c / n).is_dir()]
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("name", "set", "arm", "factor", "invalid")}))
    verdicts = {}
    for branch, factor, cand in (("far", 0.4, "K"), ("far", 0.3, "K"), ("off", 0.4, "KP")):
        verdicts[f"{branch}@{factor}"] = verdict(rows, expected, branch, factor, cand)
        print("verdict", branch, factor, json.dumps(verdicts[f"{branch}@{factor}"], default=str))
    if a.json:
        a.json.write_text(json.dumps({"sessions": rows, "verdicts": verdicts}, indent=1, default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
