"""Applied-values table (plan 2.1, 3.1.1): this batch's DRY_RUN vs the comparators' recorded arguments."""
import json, re, sys
from pathlib import Path
DRY = Path(sys.argv[1]); PRE = Path(sys.argv[2])
ITEMS = ["deterministic", "near_end_stand", "near_observe_arrival", "layer_latency_budget", "return_cusp_replans",
         "return_start_backoff", "exit_prefix_exempt", "low_clearance_guard", "restart_confirm_scans", "d7_reapproach_hold",
         "docking_degenerate_continue", "lidar_keepout_m", "encoder_model", "path_display", "far_capture_expected",
         "near_capture_lower", "near_capture_redraw", "near_capture_redraw_lateral_m", "recognition_proposals",
         "recognition_pixel_lag_steps", "live_factor", "slam_map_bundle", "max_epochs"]
def runner_args(stdout):
    lines = stdout.splitlines()
    if "RUNNER_ARGS" not in lines: return None
    a = lines[lines.index("RUNNER_ARGS") + 1:]
    out, i = {}, 0
    while i < len(a):
        t = a[i]
        if t.startswith("--"):
            k = t[2:].replace("-", "_")
            if "=" in k: k, v = k.split("=", 1); out[k] = v; i += 1; continue
            if i + 1 < len(a) and not a[i + 1].startswith("--"): out[k] = a[i + 1]; i += 2; continue
            out[k] = True
        i += 1
    return out
def recorded(p):
    try: return json.loads(Path(p).read_text()).get("arguments") or {}
    except Exception: return None
rows = {}
for d in sorted(DRY.iterdir()):
    ra = runner_args((d / "dry_stdout.txt").read_text())
    rd = (d / "riib_defaults.txt").read_text() if (d / "riib_defaults.txt").exists() else ""
    m = re.search(r"RIIB_DEFAULTS .*", (d / "dry_stdout.txt").read_text())
    rows[d.name] = {"args": ra, "riib_defaults": rd.strip(), "live_defaults": m.group(0) if m else None}
cmp = {"lcr_B0": "test_runs/lcr/cmp_a2b91d2d/lcr_B0_{sc}_s0/run/result.json",
       "k1c_K1": "test_runs/k1/cmp_c7701296/k1c_K1_{sc}_s0/run/result.json",
       "k1r_K1": "test_runs/k1r/k1r_7744f285/k1r_K1_{sc}_s0/run/result.json",
       "cts_D": "test_runs/ct_survey/cts_dfa9aaf2/D_{sc}_s0/live/epoch_1/run/result.json",
       "cts_O6": "test_runs/ct_survey/cts_dfa9aaf2/O6_{sc}_s0/live/epoch_1/run/result.json"}
def show(v):
    if v is None: return "—"
    return str(v)
print("## riib (R / S)\n")
for sc in ("random_1", "line_2", "aisle_rack"):
    new = rows[f"pfb_R_{sc}_s0"]["args"]; st = rows.get(f"pfb_S_{sc}_s0", {}).get("args")
    pre = {k: recorded(PRE / v.format(sc=sc)) for k, v in cmp.items() if k.startswith(("lcr", "k1"))}
    print(f"### {sc}\n\n| 항목 | R DRY_RUN | S DRY_RUN | lcr_B0 기록 | k1c_K1 기록 | k1r_K1 기록 |\n|---|---|---|---|---|---|")
    for it in ITEMS:
        vals = [new.get(it, "(러너 기본)") if new else "?", (st or {}).get(it, "(러너 기본)") if st else "—"] + \
               [show((pre[k] or {}).get(it)) for k in ("lcr_B0", "k1c_K1", "k1r_K1")]
        if all(v in ("(러너 기본)", "—", "None") for v in vals): continue
        print("| " + it + " | " + " | ".join(map(str, vals)) + " |")
    print("\nR riib_defaults.txt: `" + rows[f"pfb_R_{sc}_s0"]["riib_defaults"].replace("\n", " / ") + "`\n")
print("## live (L)\n")
for sc in ("line_m", "random_m"):
    ld, lo = rows[f"LD_{sc}_s0"]["args"], rows[f"LO_{sc}_s0"]["args"]
    d_, o_ = recorded(PRE / cmp["cts_D"].format(sc=sc)), recorded(PRE / cmp["cts_O6"].format(sc=sc))
    print(f"### {sc}\n\n| 항목 | L-D DRY_RUN | L-O DRY_RUN | cts D 기록 | cts O6 기록 |\n|---|---|---|---|---|")
    for it in ITEMS:
        vals = [ld.get(it, "(러너 기본)"), lo.get(it, "(러너 기본)"), show((d_ or {}).get(it)), show((o_ or {}).get(it))]
        if all(v in ("(러너 기본)", "—", "None") for v in vals): continue
        print("| " + it + " | " + " | ".join(map(str, vals)) + " |")
    print("\nL-D RIIB_DEFAULTS: `" + str(rows[f"LD_{sc}_s0"]["live_defaults"]) + "`\n")
# every recorded-argument difference between the comparators (not this batch: it has no record yet)
print("## 비교 대상끼리 기록 인자 차이(장면 random_1 / line_m s0)\n")
a, b, c = (recorded(PRE / cmp[k].format(sc="random_1")) for k in ("lcr_B0", "k1c_K1", "k1r_K1"))
d_, o_ = (recorded(PRE / cmp[k].format(sc="line_m")) for k in ("cts_D", "cts_O6"))
for lab, x, y in (("lcr_B0 vs k1r_K1", a, c), ("k1c_K1 vs k1r_K1", b, c), ("cts_D vs cts_O6 (line_m)", d_, o_)):
    norm = lambda v: re.sub(r"/snapshots/[^/]+/", "/snapshots/<snap>/", v) if isinstance(v, str) else v
    diff = sorted(k for k in set(x) | set(y) if norm(x.get(k)) != norm(y.get(k)) and k not in ("output", "seed"))
    print(f"- {lab}: " + ", ".join(f"{k}={x.get(k)!r}→{y.get(k)!r}" for k in diff)[:3000])
