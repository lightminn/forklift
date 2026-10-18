"""Real-time plan V8-5 (docs/plans/2026-10-17-realtime-sim.md): judge a live run made with
``tools/live_console_probe.py --scenario planwait``.

Usage: python tools/check_planwait_probe.py RUN_DIR   (RUN_DIR holds probe.json and run/)

Each pause, STOP and cancel must be answered ack, land inside a plan wait (the probe's send time read on the run's
clock through live_log.jsonl: both are the host's monotonic clock) and leave that plan untaken; the truck must stand
while paused, stopped and cancelled; nothing is asked after the cancel."""
import bisect, json, sys
run = sys.argv[1].rstrip("/")
probe = json.load(open(f"{run}/probe.json"))
result = json.load(open(f"{run}/run/result.json"))
monos, ticks, modes = [], [], []
for line in open(f"{run}/run/live_log.jsonl"):
    d = json.loads(line)
    if "tick_t" in d and "mono" in d and not d.get("status"):
        monos.append(d["mono"]); ticks.append(d["tick_t"]); modes.append(d.get("mode"))
def tick_at(mono):
    i = min(bisect.bisect_left(monos, mono), len(monos) - 1)
    return ticks[i]
plans = result.get("async_plans", [])
print("success", result.get("success"), "failure", result.get("failure_reason"))
print("drops", result.get("async_plan_drops"), "worker", result.get("async_plan_worker"))
for p in plans:
    print("  plan", p)
ok_all = True
for ev in ("pause", "stop", "cancel"):
    m = probe.get(f"{ev}_sent_mono")
    if m is None:
        print(ev, "NOT SENT"); ok_all = False; continue
    t = tick_at(m)
    reply = probe.get(f"{ev}_reply") or {}
    inside = [p for p in plans if p["requested_s"] <= t and (p.get("finished_s") is None or t <= p["finished_s"] + 0.05)]
    dropped = [p.get("dropped") for p in inside]
    taken_after = [p for p in plans if p in inside and p.get("dropped") is None]
    print(f"{ev:6s} sent at sim {t:.2f} reply={reply.get('t')}/{reply.get('reason')} in_wait={bool(inside)} "
          f"drop={dropped}")
    ok = reply.get("t") == "ack" and bool(inside) and not taken_after
    ok_all &= ok
for key in ("paused_max_speed_mps", "stopped_max_speed_mps", "cancelled_max_speed_mps"):
    v = probe.get(key)
    print(key, v); ok_all &= v is not None and v < 0.02
# nothing asked while paused / stopped: every request after a drop comes after the resume
cancel_t = tick_at(probe["cancel_sent_mono"]) if probe.get("cancel_sent_mono") else None
after_cancel = [p for p in plans if cancel_t is not None and p["requested_s"] > cancel_t]
print("plans asked after the cancel", after_cancel); ok_all &= not after_cancel
print("final", probe.get("final_state"))
events = [e for e in probe.get("events", []) if e["event"] in ("timeout", "reply_timeout", "control_closed")]
print("probe problems", events)
print("V8-5", "PASS" if ok_all else "FAIL")
