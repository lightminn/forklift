"""line_2 delivery robustness V3 verdict (docs/plans/2026-11-04-line2-delivery-robustness.md §4 V3, §8), from the batch
folder and its low-clearance report. Registered before the results were read.
    python v3_sum.py OUT LCR_JSON   (LCR_JSON: tools/low_clearance_report.py over every run folder)
Per run: the last attempt in b*/results.jsonl; a pair is left out only when either run's last attempt is the bundle's
infra_failed; stalled, a timeout and docking_retry_match_refused are failures. Success: result.json success true (the
delivery verdict and the whole run, return included)."""
import glob, json, os, sys
from collections import defaultdict

OUT, LCR = sys.argv[1], sys.argv[2]
rows = [json.loads(l) for l in open(f"{OUT}/RUNS.jsonl")]
last = {}
for f in sorted(glob.glob(f"{OUT}/b*/results.jsonl")):
    for l in open(f):
        x = json.loads(l)
        if x["name"] not in last or x["attempt"] >= last[x["name"]]["attempt"]:
            last[x["name"]] = x
lcr = {os.path.basename(os.path.dirname(r["run"].rstrip("/"))) if r["run"].rstrip("/").endswith("/run") else
       os.path.basename(r["run"].rstrip("/")): r for r in json.load(open(LCR))["runs"]}


def info(name):
    d = None
    p = f"{OUT}/{name}/run/result.json"
    if os.path.exists(p):
        d = json.load(open(p))
    b = last.get(name) or {}
    dd = (d or {}).get("delivery_docking") or {}
    rounds = dd.get("rounds") or []
    r1 = [r for r in rounds if r.get("round") == 1]
    first = r1[0] if r1 else {}
    eligible = bool((first.get("degenerate_continue") or {}).get("eligible"))
    installed = eligible and first.get("chosen") == "round2_stop"
    # what was installed: the whole straight's check, and the stop's only when the stop path was installed (a refused
    # stop drives the checked whole straight -- plan D2 -- and is not an installed path's failure; Codex results P3)
    installed_fail = [r for r in rounds if r.get("chosen") in ("round2_stop", "straight") and (
        (r.get("dry_run") or {}).get("swept_clear") is False
        or (r.get("chosen") == "round2_stop" and (r.get("prefix_dry_run") or {}).get("swept_clear") is False))]
    lr = lcr.get(name) or {}
    phys = [v.get("min_m") for k, v in (lr.get("rows") or {}).items() if "physical" in k and v.get("min_m") is not None]
    return {
        "name": name, "bundle": b.get("status"), "infra": b.get("status") == "infra_failed",
        # never ran to an end here (no attempt, or the bundle left it unstarted): unfinished, not a failure
        "unfinished": b.get("status") in (None, "not_started", "time_left", "time_left_retry", "start_failed"),
        "ok": bool(d and d.get("success") is True), "failure": (d or {}).get("failure_reason") if d else "no result.json",
        "r1_rejected": bool(first) and not first.get("accepted"), "r1_reason": first.get("reason"),
        "eligible": eligible, "installed": installed,
        "retry_b": sum(1 for r in rounds if r.get("chosen") == "retry_b"),
        "d5": bool(d and str(d.get("failure_reason") or "").startswith("docking_retry_match_refused")),
        "first_stop_est_minus_truth": first.get("estimate_vs_truth_at_match"),
        "r2_goal_error": [r.get("goal_error_vs_truth") for r in rounds if r.get("round") == 2 and r.get("accepted")],
        "forbidden": bool(d and ((d.get("forbidden_pocket_contacts") or [])
                                 or str(d.get("failure_reason") or "").startswith("Forbidden"))),
        "physical_min_m": min(phys) if phys else None, "lcr_error": lr.get("error"),
        "installed_check_fail": len(installed_fail),
    }


parts = defaultdict(lambda: defaultdict(dict))
for r in rows:
    _, arm, part, *rest = r["name"].split("_")
    seed = r["name"].rsplit("_s", 1)[1]
    parts[part][seed][arm] = info(r["name"])

MIN_PAIRS = {"v3a": 10, "v3b": 6, "v3c": 8}
report = {}
for part, seeds in sorted(parts.items()):
    pairs = {s: p for s, p in seeds.items() if "on" in p and "off" in p}
    valid = {s: p for s, p in pairs.items() if not (p["on"]["infra"] or p["off"]["infra"]
                                                    or p["on"]["unfinished"] or p["off"]["unfinished"])}
    on_only = [s for s, p in valid.items() if p["on"]["ok"] and not p["off"]["ok"]]
    off_only = [s for s, p in valid.items() if p["off"]["ok"] and not p["on"]["ok"]]
    rescued = [s for s in on_only if valid[s]["on"]["installed"] and valid[s]["off"]["r1_rejected"]]
    on = [p["on"] for p in valid.values()]
    safety = {"forbidden": sum(x["forbidden"] for x in on),
              "physical_contact": sum(1 for x in on if x["physical_min_m"] is not None and x["physical_min_m"] <= 0.0),
              "installed_check_fail": sum(x["installed_check_fail"] for x in on),
              "lcr_errors": sum(1 for x in on if x["lcr_error"])}
    rep = {"pairs": len(pairs), "valid_pairs": len(valid), "min_pairs": MIN_PAIRS[part],
           "unfinished_pairs": sum(1 for p in pairs.values() if p["on"]["unfinished"] or p["off"]["unfinished"]),
           "infra_pairs": sum(1 for p in pairs.values() if p["on"]["infra"] or p["off"]["infra"]),
           "on_ok": sum(x["ok"] for x in on), "off_ok": sum(p["off"]["ok"] for p in valid.values()),
           "on_only": sorted(on_only, key=int), "off_only": sorted(off_only, key=int), "rescued": sorted(rescued, key=int),
           "d1_eligible_on": sum(x["eligible"] for x in on), "installed_on": sum(x["installed"] for x in on),
           "retry_b_on": sum(x["retry_b"] for x in on), "d5_on": sum(x["d5"] for x in on),
           "off_r1_rejected": sum(p["off"]["r1_rejected"] for p in valid.values()),
           "pallet_missed_on": sum(1 for x in on if "Pallet missed" in str(x["failure"])),
           "pallet_missed_off": sum(1 for p in valid.values() if "Pallet missed" in str(p["off"]["failure"])),
           "safety_on": safety}
    # the registered safety rule: an installed path's check failure, a forbidden contact or a physical contact on the on
    # arm fails the part; a run the clearance tool could not read is reported (lcr_errors) and measured apart
    safe = all(v == 0 for k, v in safety.items() if k != "lcr_errors")
    enough = len(valid) >= MIN_PAIRS[part]
    if part == "v3a":
        passed = (enough and rep["d1_eligible_on"] >= 5 and rep["installed_on"] >= 4 and len(on_only) >= 3
                  and len(off_only) <= 1 and len(rescued) >= 2 and safe)
        verdict = ("FAIL (safety)" if not safe else "PASS" if passed else
                   "UNDECIDABLE (branch or pairs short)" if not enough or rep["d1_eligible_on"] < 5
                   or rep["installed_on"] < 4 else "FAIL")
    elif part == "v3b":
        passed = enough and safe and rep["on_ok"] >= rep["off_ok"] - 1
        verdict = "FAIL (safety)" if not safe else "PASS" if passed else "UNDECIDABLE (pairs short)" if not enough else "FAIL"
    else:
        passed = enough and safe and rep["on_ok"] >= rep["off_ok"] - 1 and rep["pallet_missed_on"] <= rep["pallet_missed_off"]
        verdict = "FAIL (safety)" if not safe else "PASS" if passed else "UNDECIDABLE (pairs short)" if not enough else "FAIL"
    rep["verdict"] = verdict
    rep["runs"] = {s: {a: v for a, v in p.items()} for s, p in sorted(pairs.items(), key=lambda kv: int(kv[0]))}
    report[part] = rep
overall = ("FAIL" if any(r["verdict"].startswith("FAIL") for r in report.values()) else
           "PASS" if all(r["verdict"] == "PASS" for r in report.values()) else "UNDECIDABLE")
print(json.dumps({"overall": overall, **{k: {kk: vv for kk, vv in v.items() if kk != "runs"} for k, v in report.items()}},
                 indent=1))
json.dump({"overall": overall, "parts": report}, open(f"{OUT}/v3_verdict.json", "w"), indent=1, default=str)
