"""(가) records diagnosis v3 (records only). Hold intervals come from the runner's own hold/release events
(result.json slam_summary.holds), not from slam.records modes (Codex P2: mode is the response-application state,
stamp_s the scan time). Per hold the applied correction is fixed, so the estimate is H o odom inside it.

HOME (every done mission): the return's final hold [t_h, t_arr]. Start: the slam.control row of the hold tick itself
(appended after that tick's SLAM application, before the hold; it equals holds[*].error). End: the last row strictly
before the arrival transition (before any release overwrite). Errors in the goal (return path end) frame.
Propagation split: P = est(t_h) o (truth(t_h)^-1 o truth(t_arr)) is where the estimate would be with exact relative
motion; the end error is B o A with B = truth(t_arr)^-1 o P (the propagated start error) and A = P^-1 o est(t_arr)
(the hold's relative-motion mismatch: rolling distance, kinematics, slip, integration), A's translation shown in the
end truth body axes.
Rotation inside the hold from slam.records pairs (odom_base and truth_base share the scan stamp): first and last record
with stamp in (t_h, t_arr]; signed yaw change, truth distance, residual (truth - odom) x sign(odom) / distance [rad/m].
STALL (each return stall replan): the leg before it = from the last truth-velocity reversal (|v| > 0.02) after the
return's start (or the previous stall) to the event; direction, truth/odom turn from records inside it.
DOCK (line_m): the transport docking hold before the first match; est - truth in the goal-line-free truck frame at
the transport release (first release event after the transport transition), the hold start and the match; the same
propagation split; full D7 'replaces' strings."""
import glob, json, math, sys
from collections import defaultdict
import numpy as np

out = sys.argv[1]
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def comp(a, b):
    c, s = math.cos(a[2]), math.sin(a[2]); return [a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])]
def inv(a):
    c, s = math.cos(a[2]), math.sin(a[2]); return [-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]]
def rel(p, ref): return comp(inv(ref), p)
home, stalls, docks, cover = [], [], [], []
for sess in sorted(glob.glob(f"{out}/*_s[0-9]")):
    run = f"{sess}/live/epoch_1/run"
    try:
        rr = json.load(open(f"{run}/result.json"))
    except Exception:
        continue
    H = (rr.get("slam_summary") or {}).get("holds") or []
    ctrl = []
    for f in sorted(glob.glob(f"{run}/spool/slam.control/*.jsonl")):
        ctrl += [json.loads(l) for l in open(f)]
    if not ctrl:
        continue
    C = np.array(ctrl, float)
    recs = []
    for f in sorted(glob.glob(f"{run}/spool/slam.records/*.jsonl")):
        for l in open(f):
            x = json.loads(l)
            if isinstance(x, dict) and "stamp_s" in x and x.get("odom_base") and x.get("truth_base"):
                recs.append((x["stamp_s"], x["odom_base"], x["truth_base"]))
    recs.sort(key=lambda r: r[0]); RT = np.array([r[0] for r in recs])
    cover.append((sess.split("/")[-1], len(H), H[-1]["time_s"] if H else None, rr.get("simulated_time_s")))
    def row(t):  # last control row strictly before t
        k = int(np.searchsorted(C[:, 0], t, side="left")) - 1
        return C[k], t - C[k][0]
    def row_at(t):  # the control row of the tick at t (appended after that tick's SLAM application, before its hold)
        k = int(np.searchsorted(C[:, 0], t - 1e-6, side="left"))
        return C[k], C[k][0] - t
    def hold_start(t):  # the hold covering t: last 'hold' event <= t with no release event between
        hs = None
        for e in H:
            if e["time_s"] > t:
                break
            if e["event"] == "hold":
                hs = e["time_s"]
            elif str(e["event"]).startswith("release"):
                hs = None
        return hs
    def rot(t0, t1):
        i0 = int(np.searchsorted(RT, t0, side="right")); i1 = int(np.searchsorted(RT, t1, side="right")) - 1
        if i1 <= i0:
            return None
        a, b = recs[i0], recs[i1]
        seg = recs[i0:i1 + 1]
        dist = float(sum(math.hypot(q[2][0] - p[2][0], q[2][1] - p[2][1]) for p, q in zip(seg[:-1], seg[1:])))
        ot = wrap(b[1][2] - a[1][2]); tt = wrap(b[2][2] - a[2][2])
        d = [b[2][0] - a[2][0], b[2][1] - a[2][1]]; fwd = d[0] * math.cos(a[2][2]) + d[1] * math.sin(a[2][2])
        return {"t0": a[0], "t1": b[0], "lead_s": a[0] - t0, "tail_s": t1 - b[0], "odom_turn": ot, "truth_turn": tt, "dist": dist,
                "direction": 1 if fwd >= 0 else -1,
                "res_per_m": ((tt - ot) * math.copysign(1, ot) / dist) if dist > 0.05 and abs(ot) > 1e-4 else None,
                "ratio": (ot / tt) if abs(tt) > 0.05 else None}
    def split(t_h, t_e, ref):
        (r0, dt0), (r1, dt1) = row_at(t_h), row(t_e)
        e0, g0, e1, g1 = list(r0[1:4]), list(r0[4:7]), list(r1[1:4]), list(r1[4:7])
        P = comp(e0, rel(g1, g0))
        return {"dt0": dt0, "dt1": dt1, "start_err": rel(e0, ref) if ref else None, "start_truth": rel(g0, ref) if ref else None,
                "end_err": rel(e1, ref) if ref else None, "end_truth": rel(g1, ref) if ref else None,
                "est_minus_truth_start_body": rel(e0, g0), "est_minus_truth_end_body": rel(e1, g1),
                "odometry_own_goal": [x - y for x, y in zip(rel(e1, ref), rel(P, ref))] if ref else None,
                "propagated_goal": [x - y for x, y in zip(rel(P, ref), rel(g1, ref))] if ref else None,
                "odometry_own_body": rel(e1, P), "propagated_body": rel(P, g1),
                # A = P^-1 o est_e (P axes) shown in the end truth body axes: rotate by B's yaw
                "relmotion_in_end_body": (lambda A, B: [math.cos(B[2]) * A[0] - math.sin(B[2]) * A[1], math.sin(B[2]) * A[0] + math.cos(B[2]) * A[1], A[2]])(rel(e1, P), rel(P, g1))}
    cur, tr = None, {}
    for e in rr.get("transitions") or []:
        cur = e.get("mission_id", cur); tr.setdefault(cur, []).append(e)
    for md in sorted(glob.glob(f"{run}/missions/m*")):
        mid = md.split("/")[-1]
        try:
            res = json.load(open(f"{md}/result.json"))["result"]; st = json.load(open(f"{md}/records.json"))["state"]
        except Exception:
            continue
        T = tr.get(mid, [])
        name = sess.split("/")[-1]
        goal = (((st.get("paths") or {}).get("return_home") or {}).get("poses") or [None])[-1]
        if res.get("outcome") == "done" and goal:
            t_arr = next((e["time_s"] for e in reversed(T) if e.get("to") == "home_settle"), None)
            t_h = hold_start(t_arr - 1e-6) if t_arr else None
            if t_arr and t_h:
                s = split(t_h, t_arr, goal)
                hev = next((e for e in H if e["event"] == "hold" and abs(e["time_s"] - t_h) < 1e-6), {})
                home.append({"session": name, "mission": mid, "hold_s": t_arr - t_h, "hold_event_error": hev.get("error"), "rot": rot(t_h, t_arr),
                             "stalls": sum(1 for e in st.get("stall_replans") or [] if e.get("phase") == "return_home"), **s})
            else:
                home.append({"session": name, "mission": mid, "missing": True})
            t_rh = next((e["time_s"] for e in T if e.get("to") == "return_home"), None)
            prev = t_rh
            for e in st.get("stall_replans") or []:
                if e.get("phase") != "return_home":
                    continue
                te = float(e["time_s"])
                seg = C[(C[:, 0] >= prev) & (C[:, 0] < te)]
                v = np.zeros(len(seg))
                if len(seg) > 2:
                    d = np.diff(seg[:, 4:6], axis=0); dt = np.maximum(np.diff(seg[:, 0]), 1e-6)
                    v[1:] = (d[:, 0] * np.cos(seg[1:, 6]) + d[:, 1] * np.sin(seg[1:, 6])) / dt
                sig = np.where(np.abs(v) > 0.02, np.sign(v), 0); nz = np.nonzero(sig)[0]; i0 = 0
                for a, b in zip(nz[:-1], nz[1:]):
                    if sig[a] != sig[b]:
                        i0 = b
                t0 = seg[i0][0] if len(seg) else prev
                stalls.append({"session": name, "mission": mid, "time_s": te, "leg_start": t0, "leg_dir": int(sig[nz[-1]]) if len(nz) else 0,
                               "rot": rot(t0, te), "hold_at_event": hold_start(te - 1e-6)})
                prev = te
        if "line_m" in name:
            dock = st.get("delivery_docking") or {}; r1 = (dock.get("rounds") or [None])[0]
            t_tr = next((e["time_s"] for e in T if e.get("to") == "transport"), None)
            if r1 and r1.get("time_s") and t_tr:
                tm = r1["time_s"]
                t_rel = next((e["time_s"] for e in H if e["time_s"] >= t_tr and str(e["event"]).startswith("release")), None)
                t_h = hold_start(tm - 1e-6)
                s = split(t_h, tm, None) if t_h else {}
                rel_row = rel(list(row(t_rel + 0.02)[0][1:4]), list(row(t_rel + 0.02)[0][4:7])) if t_rel and t_rel < tm else None
                docks.append({"session": name, "mission": mid, "dest": res.get("destination_id"), "t_transport": t_tr, "t_release": t_rel,
                              "t_hold": t_h, "t_match": tm, "after_release_body": rel_row, "rot": rot(t_h, tm) if t_h else None,
                              "eetf": r1.get("estimate_error_truck_frame"), **s,
                              "replaces": [str(e.get("replaces")) for e in st.get("docking_retries") or [] if e.get("counted")]})
def q(v):
    v = np.asarray([x for x in v if x is not None], float)
    if not len(v): return "n 0"
    return f"n {len(v)} med {np.median(v):+.4f} min {v.min():+.4f} max {v.max():+.4f} neg {int((v<0).sum())}"
print("coverage (session, hold events, last event, end):", [(c[0], c[1], round(c[2] or 0, 1), round(c[3] or 0, 1)) for c in cover][:4], "... all events end within 0.3 s of run end:",
      all(c[2] is not None and c[3] and c[3] - c[2] < 0.3 for c in cover))
hm = [h for h in home if not h.get("missing")]
print(f"== HOME done {len(home)}, with a hold at arrival {len(hm)}, missing {len(home) - len(hm)}")
print("hold-start row offset s (should be 0):", q([h["dt0"] for h in hm]))
print("hold before arrival s:", q([h["hold_s"] for h in hm]), "| row dt start/end:", q([h["dt0"] for h in hm]), q([h["dt1"] for h in hm]))
for i, n in enumerate(("along", "lateral", "yaw")):
    print(f" goal frame {n}: est(arrival) {q([h['end_err'][i] for h in hm])} | truth(arrival) {q([h['end_truth'][i] for h in hm])}")
    print(f"   est-truth at hold start (truth body) {q([h['est_minus_truth_start_body'][i] for h in hm])} | at arrival (truth body) {q([h['est_minus_truth_end_body'][i] for h in hm])}")
    print(f"   per-mission change of est-truth in the goal frame {q([(h['end_err'][i]-h['end_truth'][i])-(h['start_err'][i]-h['start_truth'][i]) for h in hm])}")
    print(f"   relative-motion mismatch (end truth body) {q([h['relmotion_in_end_body'][i] for h in hm])} | propagated start error {q([h['propagated_body'][i] for h in hm])}")
    print(f"   |propagated| > |mismatch|: {sum(1 for h in hm if abs(h['propagated_body'][i]) > abs(h['relmotion_in_end_body'][i]))}/{len(hm)}")
for d in (1, -1):
    sel = [h for h in hm if h["rot"] and h["rot"]["direction"] == d]
    if sel:
        print(f" hold rotation direction {d:+d}: n {len(sel)} odom {q([h['rot']['odom_turn'] for h in sel])} truth {q([h['rot']['truth_turn'] for h in sel])} dist {q([h['rot']['dist'] for h in sel])}")
        print(f"   residual (truth-odom) x sign(odom) / m {q([h['rot']['res_per_m'] for h in sel])} | ratio odom/truth (|truth|>0.05) {q([h['rot']['ratio'] for h in sel])} | lead/tail s {q([h['rot']['lead_s'] for h in sel])} {q([h['rot']['tail_s'] for h in sel])}")
        for sgn in (1, -1):
            ss = [h for h in sel if math.copysign(1, h["rot"]["odom_turn"]) == sgn]
            if ss: print(f"   turn {'left' if sgn > 0 else 'right'}: n {len(ss)} residual/m {q([h['rot']['res_per_m'] for h in ss])}")
print("hold event error minus start-row est-truth (truth body): along", q([h["hold_event_error"]["along_m"] - h["est_minus_truth_start_body"][0] for h in hm if h.get("hold_event_error")]),
      "lateral", q([h["hold_event_error"]["lateral_m"] - h["est_minus_truth_start_body"][1] for h in hm if h.get("hold_event_error")]),
      "yaw", q([h["hold_event_error"]["yaw_rad"] - h["est_minus_truth_start_body"][2] for h in hm if h.get("hold_event_error")]))
print(f"== STALL events {len(stalls)}")
for s in stalls:
    r = s["rot"] or {}
    print(f"  {s['session']} {s['mission']} t {s['time_s']:.1f} leg dir {s['leg_dir']:+d} len {s['time_s']-s['leg_start']:.1f}s hold_from {None if s['hold_at_event'] is None else round(s['time_s']-s['hold_at_event'],2)}s "
          f"odom {r.get('odom_turn', float('nan')):+.4f} truth {r.get('truth_turn', float('nan')):+.4f} dist {r.get('dist', float('nan')):.3f} res/m {r.get('res_per_m')}")
print(f"== DOCK line_m {len(docks)}")
by = defaultdict(list)
for d in docks: by[d["dest"]].append(d)
for dest in sorted(by):
    sel = by[dest]
    print(f" {dest}: n {len(sel)} release after transport s {q([(d['t_release'] or float('nan')) - d['t_transport'] for d in sel])} | hold before match s {q([d['t_match'] - d['t_hold'] for d in sel if d['t_hold']])}")
    for i, n in enumerate(("along", "lateral", "yaw")):
        print(f"   {n}: est-truth body after transport release {q([d['after_release_body'][i] if d['after_release_body'] else None for d in sel])} | at hold start {q([d['est_minus_truth_start_body'][i] for d in sel if d.get('est_minus_truth_start_body')])} | at match {q([d['est_minus_truth_end_body'][i] for d in sel if d.get('est_minus_truth_end_body')])} | eetf {q([d['eetf'][i] for d in sel])}")
        print(f"     hold split: relative-motion mismatch (end body) {q([d['relmotion_in_end_body'][i] for d in sel if d.get('relmotion_in_end_body')])} | propagated start {q([d['propagated_body'][i] for d in sel if d.get('propagated_body')])}")
        print(f"     |propagated| > |mismatch|: {sum(1 for d in sel if d.get('propagated_body') and abs(d['propagated_body'][i]) > abs(d['relmotion_in_end_body'][i]))}/{len(sel)} | start row offset {q([d.get('dt0') for d in sel])}")
    print(f"   hold rotation residual/m {q([d['rot']['res_per_m'] for d in sel if d['rot']])} dir {[d['rot']['direction'] for d in sel if d['rot']]}")
for d in docks:
    for r in d["replaces"]:
        print(" D7", d["session"], d["mission"], r)
json.dump({"home": home, "stalls": stalls, "docks": docks}, open(sys.argv[2], "w"), indent=1)
