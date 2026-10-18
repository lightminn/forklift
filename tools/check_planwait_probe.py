"""Real-time plan V8-5 (docs/plans/2026-10-17-realtime-sim.md): judge a live run made with
``tools/live_console_probe.py --scenario planwait``.

Usage: python tools/check_planwait_probe.py RUN_DIR   (RUN_DIR holds probe.json and run/); exit 0 only on PASS.

Times are the runner's: each request is placed by the server's own record of the request the probe got its ack for
(``live_log.jsonl`` lines with ``where`` and ``request_id``; the probe asks again after some refusals, so the first
send and the ack may be different requests), and the first command line at or after that record is the tick that
handled it. Each of pause, STOP and cancel must

- be answered ack, the probe having reached every step, with no time-out and no control connection lost before the
  cancel's ack (the runner ends the run on a cancel, so its connection closing after that ack is the expected end);
- be handled while a plan is computing (asked before that tick, finished at or after it) and leave that plan
  untaken (a pause or a cancel drops it as ``paused``);
- turn the mode within a wall second (pause -> PAUSED, STOP -> STOPPED);
- leave the truck standing until the resume, judged on fresh vehicle samples only: status lines with
  ``feedback_ok``, each feedback stamp counted once and at most ``FRESH_S`` old at its status line, at least
  ``MIN_SAMPLES`` of them, no gap over ``MAX_GAP_S`` between them or to either end of the interval, every speed
  finite and under ``STANDING_MPS``;
- ask nothing while paused or stopped and ask again after the resume; after the cancel ask nothing, command only
  zero speed and end the run as ``mission_cancelled``.

(Codex real-time D8 rounds 10-11 P1: earlier versions passed requests made after the plans, stale or repeated
samples, a late handling and a lost connection.)"""

from __future__ import annotations

import bisect
import json
import math
import sys

STEPS = ("plan_wait_1", "paused_in_wait", "plan_wait_2", "stopped_in_wait", "plan_wait_3", "cancelled_in_wait")
STANDING_MPS = 0.05  # the mission core's own "moving while it waits" threshold (measured speed noise reaches 0.03)
MIN_SAMPLES = 5
FRESH_S = 0.5
MAX_GAP_S = 1.5
KINDS = {"pause": "mission_pause", "stop": "stop", "cancel": "mission_cancel"}


def judge(probe: dict, result: dict, log_lines: list[dict]) -> tuple[list[str], list[str]]:
    """(failures, notes) for one run."""
    cmd_mono, cmd_tick, cmd_mode, cmd_speed, fb, server = [], [], [], [], [], {}
    for d in log_lines:
        if d.get("status"):
            f = d.get("feedback")
            if f and d.get("feedback_ok"):
                fb.append((float(f[0]), float(f[1]), float(d.get("mono", float("nan")))))
        elif d.get("request_id") and d.get("where"):
            server.setdefault(d["request_id"], d)
        elif "tick_t" in d and "mono" in d:
            cmd_mono.append(d["mono"])
            cmd_tick.append(d["tick_t"])
            cmd_mode.append(d.get("mode"))
            cmd_speed.append(d.get("speed_mps"))
    plans = result.get("async_plans", [])
    events = probe.get("events", [])
    fails: list[str] = []
    notes: list[str] = []

    def need(cond, msg):
        if not cond:
            fails.append(msg)
        return bool(cond)

    def ack_mono(kind, request_id):
        for e in events:
            if e["event"] == "reply" and e.get("kind") == kind and e.get("request_id") == request_id:
                return e["mono"]
        return None

    cancel_reply = probe.get("cancel_reply") or {}
    cancel_ack = (ack_mono("mission_cancel", cancel_reply.get("request_id"))
                  if cancel_reply.get("t") == "ack" else None)
    cancelled = result.get("failure_reason") == "mission_cancelled"

    def expected_end(e):
        return cancel_ack is not None and cancelled and e["mono"] >= cancel_ack

    reached = {e.get("label") for e in events if e["event"] == "reached"}
    if any(e["event"] == "closed_while_waiting" and e.get("label") == "cancelled_in_wait" and expected_end(e)
           for e in events):
        reached.add("cancelled_in_wait")
    for step in STEPS:
        need(step in reached, f"probe did not reach {step}")
    bad = [e for e in events
           if (e["event"] in ("timeout", "reply_timeout")
               or (e["event"] == "closed_while_waiting" and not expected_end(e))
               or (e["event"] == "control_closed" and e.get("current") and not expected_end(e)))]
    need(not bad, f"probe problems {bad}")
    resumes = [e for e in events
               if e["event"] == "reply" and e.get("kind") == "mission_resume" and e.get("t") == "ack"]
    need(len(resumes) >= 2, f"resume acks {len(resumes)} (want 2)")

    def tick_index(mono):
        i = bisect.bisect_left(cmd_mono, mono)
        return i if i < len(cmd_mono) else None

    def handled(kind, reply):
        """The server's record of the acked request: its mono, or None."""
        rid = (reply or {}).get("request_id")
        rec = server.get(rid)
        if not need(rec is not None and rec.get("t") == kind, f"{kind}: no server record for the acked {rid}"):
            return None
        return float(rec["mono"])

    def standing(start, end, label):
        seen, samples = set(), []
        for stamp, speed, at in fb:
            if start <= stamp <= end and stamp not in seen and math.isfinite(speed) and at - stamp <= FRESH_S:
                seen.add(stamp)
                samples.append((stamp, abs(speed)))
        samples.sort()
        need(len(samples) >= MIN_SAMPLES, f"{label}: {len(samples)} fresh samples (want {MIN_SAMPLES})")
        if samples:
            edges = ([samples[0][0] - start] + [b[0] - a[0] for a, b in zip(samples, samples[1:])]
                     + [end - samples[-1][0]])
            need(max(edges) <= MAX_GAP_S, f"{label}: a {max(edges):.2f} s gap in the samples")
            top = max(v for _, v in samples)
            need(top < STANDING_MPS, f"{label}: max speed {top}")
            notes.append(f"{label}: max speed {top:.4f} over {len(samples)} fresh samples")

    at = {}
    for ev, mode in (("pause", "PAUSED"), ("stop", "STOPPED"), ("cancel", None)):
        reply = probe.get(f"{ev}_reply") or {}
        if not need(reply.get("t") == "ack", f"{ev} reply {reply.get('t')}/{reply.get('reason')}"):
            continue
        m = handled(KINDS[ev], reply)
        i = tick_index(m) if m is not None else None
        if not need(i is not None, f"{ev}: no tick after its handling"):
            continue
        at[ev] = (m, cmd_tick[i], i)
        t = cmd_tick[i]
        computing = [p for p in plans if p["requested_s"] < t <= (p.get("finished_s") or float("inf"))]
        notes.append(f"{ev} handled at sim {t:.3f}: plans computing "
                     f"{[(p['site'], p['job'], p.get('dropped')) for p in computing]}")
        if need(len(computing) == 1, f"{ev}: {len(computing)} plans computing at sim {t:.3f} (want 1)"):
            p = computing[0]
            need(p.get("dropped") is not None, f"{ev}: plan {p['job']} was taken")
            if ev in ("pause", "cancel"):
                need(p.get("dropped") == "paused", f"{ev}: plan {p['job']} dropped as {p.get('dropped')}")
        if mode is not None:
            j = i
            while j < len(cmd_mono) and cmd_mono[j] <= m + 1.0 and cmd_mode[j] != mode:
                j += 1
            need(j < len(cmd_mono) and cmd_mode[j] == mode and cmd_mono[j] <= m + 1.0,
                 f"{ev}: no {mode} within a second")

    if len(resumes) >= 2:
        for ev, resume, label in (("pause", resumes[0], "paused"), ("stop", resumes[1], "stopped")):
            if ev not in at:
                continue
            rm = handled("mission_resume", {"request_id": resume.get("request_id")})
            if rm is None:
                continue
            standing(at[ev][0], rm, label)
            i1 = tick_index(rm)
            t0, t1 = at[ev][1], cmd_tick[i1] if i1 is not None else None
            if t1 is not None:
                # the command is applied at its tick's start, so a plan asked later in that tick is asked while
                # paused; the resume's tick may ask (Codex real-time D8 round 12 P1)
                asked = [p["job"] for p in plans if t0 <= p["requested_s"] < t1]
                need(not asked, f"{label}: plans asked while {label}: {asked}")
                need(any(p["requested_s"] >= t1 for p in plans), f"{label}: nothing asked after the resume")
    if "cancel" in at:
        m, t, i = at["cancel"]
        after = [p["job"] for p in plans if p["requested_s"] >= t]
        need(not after, f"plans asked after the cancel: {after}")
        speeds = cmd_speed[i:]
        need(len(speeds) > 0 and all(v is not None and abs(v) < 1e-9 for v in speeds),
             f"cancelled: commanded speeds after the cancel {sorted(set(map(str, speeds)))[:5]}")
        measured = [abs(v) for stamp, v, _ in fb if stamp >= m]
        need(all(v < STANDING_MPS for v in measured), f"cancelled: measured speeds {measured}")
        notes.append(f"cancelled: {len(speeds)} command lines after it, all zero speed; measured {measured}")
    need(cancelled, f"run ended {result.get('success')}/{result.get('failure_reason')} (want mission_cancelled)")
    return fails, notes


def main(run: str) -> int:
    run = run.rstrip("/")
    probe = json.load(open(f"{run}/probe.json"))
    result = json.load(open(f"{run}/run/result.json"))
    with open(f"{run}/run/live_log.jsonl") as f:
        log_lines = [json.loads(line) for line in f]
    fails, notes = judge(probe, result, log_lines)
    for n in notes:
        print(n)
    print("drops", result.get("async_plan_drops"), "worker", result.get("async_plan_worker"))
    for p in result.get("async_plans", []):
        print("  plan", {k: p.get(k) for k in ("site", "job", "requested_s", "finished_s", "dropped", "ok")})
    for f in fails:
        print("FAIL:", f)
    print("V8-5", "PASS" if not fails else "FAIL")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
