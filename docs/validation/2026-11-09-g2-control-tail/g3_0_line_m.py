"""G3-0 (plan 5.19): the line_m re-check from the r1c S-far / S-off records -- the work breakdown on AUTO steps and
the f 1.0 counterfactuals through the real Pacer (catch-up 6), with the lead's two side-by-side rows (2026-11-09):
the tail flattening and the control-mean cut on the same reduced mix.

usage: g3_0_line_m.py SNAP R1C_DIR RUN [RUN ...]
  SNAP: a checkout whose sim/isaac and src give tick_trace / Pacer (the r1c snapshot or a worktree at the same code).
  RUN: r1c session names under R1C_DIR (each with live/epoch_1/run/{tick_trace.npz,live_log.jsonl}).

Caveats printed with the table (plan 5.19 G3-0): the records ran at f 0.4 / 0.3, so the f 1.0 replay assumes the
recorded per-step work W does not change with the factor -- an extrapolation (G1 supported it only inside hall
f 1.0), for ranking only. AUTO steps are the command lines' mode AUTO mapped by live step; the replay runs the whole
trace (the pacer's state carries across STOP and TELEOP) and the shares are read over AUTO steps only. Phase split
("split") is a reference row: with far recognition the phase model has not been validated (5.16).
"""
import json
import sys
from pathlib import Path

import numpy as np

SNAP = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(SNAP / "sim" / "isaac"))
sys.path.insert(0, str(SNAP / "src"))
from tick_trace import COLUMNS, FLAGS, work  # noqa: E402
from forklift_core.live.pacing import Pacer  # noqa: E402

DT = 1 / 120
C = {n: i for i, n in enumerate(COLUMNS)}
BIT = {n: 1 << i for i, n in enumerate(FLAGS)}
TAIL = 0.05


def load(run_dir: Path):
    run = run_dir / "live" / "epoch_1" / "run"
    with np.load(run / "tick_trace.npz") as z:
        data = z["data"]
    w = work(data)
    S, q, d = data[:, C["S"]], data[:, C["q"]], data[:, C["d"]]
    # the start's own overshoot past its wait (Codex G3-0 review P2: S[i+1]-(q+d)[i+1] was one step late)
    eps = S - (q + d)
    nx = data[np.r_[np.arange(1, len(data)), len(data) - 1]]
    parts = {"to_step_end": data[:, C["step_end"]] - S,
             "world": data[:, C["world_end"]] - data[:, C["world_start"]],
             "step_rest": data[:, C["step_end"]] - data[:, C["world_end"]],
             "after": data[:, C["after_end"]] - data[:, C["step_end"]],
             "control": nx[:, C["ctrl_end"]] - nx[:, C["ctrl_start"]]}
    bits = np.nan_to_num(data[:, C["flags"]]).astype(int)
    auto_steps, factor, cmd = set(), None, []
    for line in open(run / "live_log.jsonl"):
        r = json.loads(line)
        if "mode" in r and "step" in r and "pacing_ok" in r and r.get("mono") is not None:
            cmd.append(r)
            if r["mode"] == "AUTO":
                auto_steps.add(int(r["step"]))
        if r.get("status") and isinstance(r.get("pacing"), dict) and factor is None:
            factor = r["pacing"].get("target_factor")
    # the measured AUTO ratio and ok share (the check row replays the record at its own factor and catch-up)
    msim = mwall = 0.0
    for r0, r1 in zip(cmd, cmd[1:]):
        if r0["mode"] == "AUTO" and r1["mode"] == "AUTO":
            msim, mwall = msim + (r1["tick_t"] - r0["tick_t"]), mwall + (r1["mono"] - r0["mono"])
    oks = [bool(r["pacing_ok"]) for r in cmd if r["mode"] == "AUTO"]
    measured = (msim / mwall if mwall > 0 else None,
                float(np.mean(oks[oks.index(True):])) if True in oks else None)
    catchup = 0
    args_txt = run_dir / "args.txt"
    if args_txt.exists():
        for line in args_txt.read_text().splitlines():
            if line.startswith("FORKLIFT_LIVE_CATCHUP_STEPS="):
                catchup = int(line.split("=", 1)[1] or 0)
    live_step = data[:, C["live_step"]]
    auto = np.array([int(s) in auto_steps if np.isfinite(s) else False for s in live_step])
    keep = np.isfinite(w) & np.isfinite(eps)
    idx = np.flatnonzero(keep)
    return {"w": w[idx], "eps": eps[idx], "auto": auto[idx], "factor": factor, "measured": measured,
            "catchup": catchup,
            "parts": {k: np.nan_to_num(v[idx]) for k, v in parts.items()},
            "rendered": ((bits & BIT["rendered"]) != 0)[idx], "scan": ((bits & BIT["scan"]) != 0)[idx]}


def transform(r, split=False, scan=1.0, body=1.0, tail="keep", render=1.0, body_save=None, render_save=None):
    """(W, to_step_end). Scan processing and the render's world.step share sit before step_world's end; the camera
    read and the control tick after it. body halves only control ticks <= 50 ms; tail 'flat' caps the rest at 50 ms;
    tail 'scale' scales the long ticks by ``body`` too (pacer_replay's ctrl factor)."""
    P = r["parts"]
    w, tse = r["w"].copy(), P["to_step_end"].copy()
    if scan != 1.0:
        cut = np.where(r["scan"], (1 - scan) * P["step_rest"], 0.0)
        w -= cut
        tse -= cut
    c = P["control"]
    short = c <= TAIL
    sim_all = len(r["w"]) * DT
    if body_save is not None:  # an absolute cut, s per sim s over the whole trace (Codex P2: equal savings compared)
        body = 1.0 - body_save * sim_all / max(c[short].sum(), 1e-9)
    w -= np.where(short, (1 - body) * c, 0.0)
    if tail == "flat":
        w -= np.where(~short, c - TAIL, 0.0)
    elif tail == "scale":
        w -= np.where(~short, (1 - body) * c, 0.0)
    plain_world = np.median(P["world"][~r["rendered"]]) if (~r["rendered"]).any() else 0.0
    render_part = np.where(r["rendered"], np.maximum(P["world"] - plain_world, 0.0), 0.0)
    if render_save is not None:
        render = 1.0 - render_save * sim_all / max(render_part.sum(), 1e-9)
    if render != 1.0:
        cut = (1 - render) * render_part
        w -= cut
        tse -= cut
        render_part = render * render_part
    if split:
        rs = r["rendered"] & r["scan"]
        rp = np.where(rs, render_part, 0.0)
        read = np.where(rs, P["after"], 0.0)
        w -= rp + read
        tse -= rp
        w[6:] += rp[:-6] + read[:-6]
        tse[6:] += rp[:-6]
    return w, tse


def replay(r, w, tse, catchup=6, factor=1.0):
    p = Pacer(factor, DT, catchup_steps=catchup)
    t, oks, starts = 0.0, [], []
    for i, (wi, ei) in enumerate(zip(w, r["eps"])):
        sleep = p.wait_s(t)
        t += sleep + (ei if sleep > 0 else 0.0)
        p.started(t)
        starts.append(t)
        p.after_step(t + min(max(tse[i], 0.0), wi))
        oks.append(p.ok)
        t += wi
    oks, starts = np.array(oks), np.r_[np.array(starts), t]
    a = r["auto"]
    first = int(np.argmax(oks & a)) if (oks & a).any() else None
    ok = None if first is None else float(oks[first:][a[first:]].mean())
    wall = np.diff(starts)[a].sum()
    return a.sum() * DT / wall, ok


ROWS = [
    ("base", {}),
    ("scan0.5", {"scan": 0.5}),
    ("scan0.5 + body0.5 (tail kept)  [lead: control-mean cut]", {"scan": 0.5, "body": 0.5}),
    ("scan0.5 + tail flat            [lead: tail flattening]", {"scan": 0.5, "tail": "flat"}),
    ("scan0.5 + ctrl0.5 (all ticks)", {"scan": 0.5, "body": 0.5, "tail": "scale"}),
    ("scan0.5 + render0.5", {"scan": 0.5, "render": 0.5}),
    ("scan0.5 + render0.5 + tail flat", {"scan": 0.5, "render": 0.5, "tail": "flat"}),
    ("scan0.5 + body cut 0.05 s/sim s (equal saving)", {"scan": 0.5, "body_save": 0.05}),
    ("scan0.5 + render cut 0.05 s/sim s (equal saving)", {"scan": 0.5, "render_save": 0.05}),
    ("ref split", {"split": True}),
    ("ref split + scan0.5", {"split": True, "scan": 0.5}),
    ("ref split + scan0.5 + tail flat", {"split": True, "scan": 0.5, "tail": "flat"}),
]
GRID_SCAN = (1.0, 0.75, 0.5, 0.35, 0.25)
GRID_BODY = (1.0, 0.75, 0.5)
GRID_RENDER = (1.0, 0.75, 0.5)


def main() -> int:
    root = Path(sys.argv[2])
    print("f 1.0 replay of f 0.4/0.3 records (W held fixed): for ranking only, see the module docstring")
    for name in sys.argv[3:]:
        r = load(root / name)
        a = r["auto"]
        P = r["parts"]
        sim = a.sum() * DT
        c = P["control"]
        print(f"\n{name}: factor {r['factor']}, AUTO steps {int(a.sum())} ({sim:.0f} sim s)")
        print("  AUTO work per sim s: " + " ".join(f"{k} {P[k][a].sum() / sim:.3f}"
                                                   for k in ("world", "step_rest", "after", "control"))
              + f" | W {r['w'][a].sum() / sim:.3f}; control > 50 ms {int((c[a] > TAIL).sum())}")
        rr, ro = replay(r, r["w"], r["parts"]["to_step_end"], catchup=r["catchup"], factor=r["factor"])
        mr, mo = r["measured"]
        print(f"   check: replay at the record's own f {r['factor']} k {r['catchup']}: ratio {rr:.3f} ok {ro:.3f}"
              f" | measured AUTO ratio {mr:.3f} ok {mo if mo is None else round(mo, 3)}")
        base_w = r["w"].sum()
        for label, kw in ROWS:
            ratio, ok = replay(r, *transform(r, **kw))
            saved = (base_w - transform(r, **kw)[0].sum()) / (len(r["w"]) * DT)
            print(f"   {label:58s} ratio {ratio:.3f} ok {ok if ok is None else round(ok, 3)}  (saves {saved:.3f} s/sim s)")
        best = []
        for tail in ("keep", "flat"):
            for s in GRID_SCAN:
                for b in GRID_BODY:
                    for rd in GRID_RENDER:
                        ratio, ok = replay(r, *transform(r, scan=s, body=b, tail=tail, render=rd))
                        if ok is not None and ok >= 0.95 and ratio >= 0.95:
                            # ranked by the work removed (s per sim s), tail flattening included (Codex P2)
                            w_ = transform(r, scan=s, body=b, tail=tail, render=rd)[0]
                            cost = (base_w - w_.sum()) / (len(r["w"]) * DT)
                            best.append((cost, s, b, rd, tail, ok, ratio))
        best.sort()
        print("   least removed work (s/sim s) reaching ok >= 0.95 and ratio >= 0.95, no phase split (grid, not an optimum):")
        for cost, s, b, rd, tail, ok, ratio in best[:5]:
            print(f"     scan {s} body {b} render {rd} tail {tail}: saves {cost:.3f}, ok {ok:.3f} ratio {ratio:.3f}")
        if not best:
            print("     none on the grid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
