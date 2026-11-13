"""The layer budget diagnosis' perturbation batch judgement (docs/plans/2026-11-13-layer-budget-diagnosis.md 3,
fixed before the results).

    python lat_diag_judge.py OUT_ROOT --snap SNAP --batch58 OUT58 [--json OUT.json]

OUT_ROOT holds RUNS_diag.jsonl (and RUNS_rerun.jsonl), the run folders and the bundles' b*/results.jsonl; OUT58 is the
58-run check's out root (the canaries' references). Per run, latency test A's / the budget check's validity, safety,
mission and operating reads (lat_budget_analyze, lat_analyze). Then:

- Pn exposure (3.2): the recorded arguments (seed s + 1000, slam_noise_seed s); the slam_control.npy hash against the
  same seed's B0 (a Pn with an equal hash is "no control/truth pose difference", kept in the test). Two or more
  unexposed Pn in a scene: that scene is not judged.
- Triplets (3.3): a scene-seed's B0, Pn and Bb10, all valid and Pn exposed. (ga) mission failure on triplets where B0
  completed; (na) operating exceedance only on triplets where all three completed (A's widths). Per endpoint, b =
  seeds where only Bb10 worsened, c = seeds where only Pn worsened, one-sided sign test p = P(X >= b | b + c, 1/2).
  p <= 0.10: "budget's excess worsening confirmed"; otherwise "not confirmed against Pn". Pooled over the two scenes
  (disjoint seeds), with each scene's b, c and p beside it.
- Report only: improvements (B0 failed, the arm completed), approach-entry divergence over 5 s, the canaries' match
  with the 58-run B0 (exceptions 1 and 2 applied; a mismatch stops exploratory pooling, never the test).
- Safety violations are P1 whatever the verdict.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ALPHA = 0.10
PN_OFFSET = 1000
DIVERGE_S = 5.0
ARMS = ("B0", "Pn", "Bb10")
# results-document sentences fixed before the results (plan 3.6)
CONFIRMED = ("In these two scenes and seeds, Bb10 worsened {endpoint} more often than the noise resample Pn "
             "(b = {b}, c = {c}, one-sided p = {p:.3f}): the budget's excess worsening is confirmed for this endpoint.")
NOT_CONFIRMED = ("Bb10 did not worsen {endpoint} significantly more often than the noise resample Pn (b = {b}, c = {c}, "
                 "one-sided p = {p}). This is not evidence that the budget costs nothing: with at most 16 pairs the "
                 "test detects only large differences.")


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


G = _load("lat_budget_analyze", HERE / "lat_budget_analyze.py")
LA = G.LA


def sign_test(b: int, c: int) -> float | None:
    """One-sided P(X >= b) for X ~ Binomial(b + c, 1/2); None without discordant pairs."""
    n = b + c
    if n == 0:
        return None
    return sum(math.comb(n, k) for k in range(b, n + 1)) / 2 ** n


def pn_exposure(result: dict | None, seed: int, b0_control: str | None, pn_control: str | None) -> dict:
    """3.2: the arguments decide exposure; the slam_control hash is the result-side check (report)."""
    args = (result or {}).get("arguments") or {}
    exposed = args.get("seed") == seed + PN_OFFSET and args.get("slam_noise_seed") == seed
    same = None if (b0_control is None or pn_control is None) else b0_control == pn_control
    return {"exposed": bool(exposed), "args_seed": args.get("seed"), "args_slam_noise_seed": args.get("slam_noise_seed"),
            "control_equal_to_b0": same}


def worsened(arm_row: dict, b0_row: dict) -> dict:
    """(ga) mission failure given B0 completed; (na) operating exceedance given both completed (None: not counted)."""
    a, b = arm_row["mission"], b0_row["mission"]
    ga = None if not b["done"] else (not a["done"])
    na = None
    if a["done"] and b["done"]:
        op = LA.operating_within(a, b)
        na = not all(v["within"] for v in op.values())
    return {"ga": ga, "na": na}


def judge_endpoint(triplets: list, key: str) -> dict:
    """b, c and the sign test over triplets where the endpoint is defined for both arms."""
    usable = [t for t in triplets if t["Bb10"][key] is not None and t["Pn"][key] is not None]
    b = sum(1 for t in usable if t["Bb10"][key] and not t["Pn"][key])
    c = sum(1 for t in usable if t["Pn"][key] and not t["Bb10"][key])
    both = sum(1 for t in usable if t["Bb10"][key] and t["Pn"][key])
    p = sign_test(b, c)
    return {"n": len(usable), "b": b, "c": c, "both": both, "p": p,
            "verdict": "confirmed" if p is not None and p <= ALPHA else "not_confirmed"}


def sentence(endpoint: str, res: dict) -> str:
    if res["verdict"] == "confirmed":
        return CONFIRMED.format(endpoint=endpoint, b=res["b"], c=res["c"], p=res["p"])
    p = "n/a (no discordant pair)" if res["p"] is None else f"{res['p']:.3f}"
    return NOT_CONFIRMED.format(endpoint=endpoint, b=res["b"], c=res["c"], p=p)


def triplets_for(rows: dict, scene: str, seeds) -> tuple[list, list]:
    """The scene's valid exposed triplets and the seeds left out (with why)."""
    out, left = [], []
    for s in seeds:
        r = {arm: rows.get(f"{arm}:{scene}:{s}") for arm in ARMS}
        pn = r["Pn"]
        if pn is not None and "exposure" in pn and not pn["exposure"]["exposed"]:
            # counted before the other arms' validity: an unexposed Pn counts towards the scene's limit whatever else
            # happened at that seed (Codex tools review P2)
            left.append({"seed": s, "unexposed": pn["exposure"]})
            continue
        why = [arm for arm, x in r.items() if x is None or x.get("invalid") or "mission" not in x]
        if why:
            left.append({"seed": s, "invalid": why})
            continue
        out.append({"seed": s, "Pn": worsened(r["Pn"], r["B0"]), "Bb10": worsened(r["Bb10"], r["B0"])})
    return out, left


def conclusions(rows: dict, seeds_by_scene: dict) -> dict:
    per_scene, all_t = {}, []
    for scene, seeds in seeds_by_scene.items():
        t, left = triplets_for(rows, scene, seeds)
        unexposed = sum(1 for x in left if "unexposed" in x)
        judged = unexposed < 2
        per_scene[scene] = {"triplets": len(t), "left_out": left, "judged": judged,
                            "ga": judge_endpoint(t, "ga"), "na": judge_endpoint(t, "na")}
        if not judged:  # the statistics stay for the record; no verdict (Codex tools review P3)
            for key in ("ga", "na"):
                per_scene[scene][key]["verdict"] = "unjudged"
        if judged:
            all_t += t
    pooled = {"ga": judge_endpoint(all_t, "ga"), "na": judge_endpoint(all_t, "na")}
    return {"per_scene": per_scene, "pooled": pooled,
            "sentences": {"mission failure": sentence("mission failure", pooled["ga"]),
                          "operating exceedance": sentence("operating exceedance", pooled["na"])}}


def file_hash(path: Path) -> str | None:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def approach_time(result: dict) -> float | None:
    for t in result.get("transitions") or []:
        if (t.get("to") or t.get("phase")) == "approach":
            return float(t["time_s"])
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out_root", type=Path)
    ap.add_argument("--snap", type=Path, required=True)
    ap.add_argument("--batch58", type=Path, required=True)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    lists = _load("lat_lists", HERE / "lat_lists.py")
    low_mod = _load("low_clearance_report", ROOT / "tools/low_clearance_report.py")
    sys.path.insert(0, str(ROOT))
    from tools import deterministic_compare as dc

    rows_in = [json.loads(x) for x in (a.out_root / "RUNS_diag.jsonl").read_text().splitlines() if x.strip()]
    if {r["name"] for r in rows_in} != {r["name"] for r in lists.diag_rows(str(a.snap), str(a.out_root))}:
        print("list_not_registered")
        return 2
    rerun_file = a.out_root / "RUNS_rerun.jsonl"
    reruns = ({json.loads(x)["name"]: json.loads(x) for x in rerun_file.read_text().splitlines() if x.strip()}
              if rerun_file.exists() else {})
    attempts = LA.bundle_records(a.out_root)
    rows, by_key = {}, {}

    def make_row(use: dict) -> dict:
        d = Path(use["argv"][-1])
        run = d / "run"
        res = LA._json(run / "result.json")
        env = use["env"]
        arm, scene, seed = env["LAT_ARM"], env["SCENE"], int(env["SEED"])
        problems = LA.environment_problems(attempts.get(use["name"], []), run, "B0", res)
        problems += G.budget_problems("B0" if arm == "Pn" else arm, res, run)
        row = {"name": use["name"], "arm": arm, "scene": scene, "seed": seed, "dir": str(d), "invalid": problems,
               "control_sha256": file_hash(run / "slam_control.npy")}
        if res is not None:
            row["mission"] = LA.mission_of(res)
            row["stop"] = LA.stop_or_end(res)
            row["approach_s"] = approach_time(res)
            try:
                stopping = LA.stopping_from(a.snap, res, low_mod)
                low = low_mod.run_report(str(run), stopping, 0.01)
            except Exception as exc:  # noqa: BLE001 -- unreadable: the run is the environment's
                low = {"error": repr(exc)[:300]}
            row["safety"] = LA.safety_of(res, low)
            if row["safety"]["violations"] is None:
                row["invalid"].append("safety record unreadable")
            row["_result"] = res
        return row

    for r in rows_in:
        name = r["name"]
        rr = reruns.get(f"lat_r_{name}")
        rows[name] = make_row(r)
        if rr is not None:
            rows[rr["name"]] = {**make_row(rr), "rerun_of": name}
    for name, row in rows.items():  # the effective run of each scene-seed-arm
        if name.endswith("_can") or (f"lat_r_{name}" in rows):
            continue
        by_key[f"{row['arm']}:{row['scene']}:{row['seed']}"] = row
    for row in by_key.values():
        if row["arm"] == "Pn":
            b0 = by_key.get(f"B0:{row['scene']}:{row['seed']}")
            row["exposure"] = pn_exposure(row.get("_result"), row["seed"], (b0 or {}).get("control_sha256"),
                                          row["control_sha256"])
    seeds_by_scene = {sc: list(seeds) for sc, seeds in lists.DIAG_SEEDS.items()}
    concl = conclusions(by_key, seeds_by_scene)
    # report only: improvements, approach divergence
    report = {"improved": [], "diverged": {"Pn": 0, "Bb10": 0, "pairs": {"Pn": 0, "Bb10": 0}}}
    for sc, seeds in seeds_by_scene.items():
        for s in seeds:
            b0 = by_key.get(f"B0:{sc}:{s}")
            for arm in ("Pn", "Bb10"):
                x = by_key.get(f"{arm}:{sc}:{s}")
                if not b0 or not x or "mission" not in b0 or "mission" not in x:
                    continue
                if not b0["mission"]["done"] and x["mission"]["done"]:
                    report["improved"].append(x["name"])
                if b0.get("approach_s") is not None and x.get("approach_s") is not None:
                    report["diverged"]["pairs"][arm] += 1  # each arm its own comparable pairs (Codex P2)
                    report["diverged"][arm] += abs(x["approach_s"] - b0["approach_s"]) > DIVERGE_S
    # canaries against the 58-run B0 (exceptions 1 and 2), exploratory linking only
    LA.strip_layer_timing(dc)
    G.strip_affinity(dc)
    canaries = {}
    for scene, seed in lists.DIAG_CANARIES:
        name = f"lbd_B0_{scene}_s{seed}_can"
        mine = rows.get(f"lat_r_{name}") or rows.get(name)  # the effective run (Codex tools review P2)
        ref = a.batch58 / f"lbg_B0_{scene}_s{seed}" / "run"
        if mine is None or mine["invalid"] or not ref.exists():
            canaries[f"{scene}_s{seed}"] = None
            continue
        r = dc.compare(Path(mine["dir"]) / "run", ref)
        canaries[f"{scene}_s{seed}"] = {"equal": bool(r["equal"]), "differences": r["differences"][:5]}
    safety = sorted(n for n, r in rows.items() if (r.get("safety") or {}).get("violations"))
    for row in rows.values():
        row.pop("_result", None)
    rerun_needed = sorted(n for n, r in rows.items() if r["invalid"] and not r.get("rerun_of")
                          and f"lat_r_{n}" not in rows)
    out = {"rows": rows, "conclusions": concl, "report": report, "canaries": canaries,
           "safety_violations": safety, "rerun_needed": rerun_needed,
           "to_lead": sorted(n for n, r in rows.items() if r["invalid"] and r.get("rerun_of"))}
    if a.json is not None:
        a.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
    print(json.dumps({k: out[k] for k in ("conclusions", "report", "canaries", "safety_violations", "rerun_needed",
                                          "to_lead")}, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
