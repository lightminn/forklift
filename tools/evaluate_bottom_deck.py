# tools/evaluate_bottom_deck.py
"""CPU evaluation of the bottom-deck side evidence under the two-face policy (plan 2026-10-19 task 2).

A scene is the unit: one structure of one shape under one entry hypothesis, its candidate pose error, and for each of
the two evidence faces (the hypothesis' model +y and -y faces) an arrival error, an extrinsic error and a noise seed.
Both faces are rendered and judged; the scene's verdict is the policy's: both ``present`` -> present, any ``absent``
-> absent, otherwise unobserved. A face whose actual stand fails the arrival check counts as unobserved.

  run:      evaluate_bottom_deck.py run --band dev|eval --groups epal6:x,t11_06:x,t11_06:y --n 299 --records 32 \\
                --out rows.jsonl [--shard i --of k] [--workers w] [--kinds a,b] [--dims field=delta,...]
  summary:  evaluate_bottom_deck.py summary rows*.jsonl [--json out.json]

The configuration (smoothing off / on, rig K / D435i 640 K) is a separate population each; the pass criteria are
per group x configuration. Scenes are the same across configurations (same draws). ``--dims`` builds the article
off its nominal dimensions (the judge keeps the nominal spec): the dimension-tolerance record, dev only.
"""

import argparse
import hashlib
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from forklift_core.perception import bottom_deck_evidence as B  # noqa: E402
from tools import bottom_deck_scenes as S  # noqa: E402

# Seed bases; dev tunes, eval is opened once. v1 (dev, eval) is the task 2 model, its eval opened at ce0f714; v2
# (dev2, eval2) widens the face-normal candidate error to +-100 mm and adds a per-capture pose error -- the Isaac
# side stands put the target 4-9 cm off along the face normal (plan task 5). A band never changes model.
BANDS = {"dev": 300_000, "eval": 700_000, "dev2": 400_000, "eval2": 900_000}
MODEL = {"dev": "v1", "eval": "v1", "dev2": "v2", "eval2": "v2"}
DEV_BANDS = ("dev", "dev2")
JUDGE_SHA256 = hashlib.sha256(Path(B.__file__).read_bytes()).hexdigest()
# The evaluator and the scene recipe as well: a band's rows must all come from one judge, one recipe, one model.
TOOLS_SHA256 = hashlib.sha256(Path(__file__).read_bytes() + Path(S.__file__).read_bytes()).hexdigest()
GROUPS = ("epal6:x", "t11_06:x", "t11_06:y")
THETA_E = {"x": 35.0, "y": 0.0}
# The negatives of the bottom-deck-required policy (--bottom-deck-check on, the site policy for bottom-deck pallets
# only). Not the general negative population: since 2026-10-27 grounded is a pick target (plan 2026-10-27-
# bottomless-and-bundled-pallets section 2.1 o); do not mix these counts with the recognition negatives.
NEG_KINDS = ("grounded", "shelf", "deckless", "grounded+nb", "grounded+plate", "grounded+ledge1", "grounded+ledge2",
             "grounded+ledge4")
REAL_KINDS = ("pallet", "pallet+nb")
RECORD_KINDS = ("slab", "pallet+plate", "pallet+ledge4", "pallet+occluder", "pallet+stacked", "damaged+ledge")
CONFIGS = (("off", "rig"), ("on", "rig"), ("off", "d435i"), ("on", "d435i"))
BAND_MM = (-9.0, 11.0)       # separate board-height objects inside this are not told apart (plan section 5)
FACES = (1, -1)
CAPTURES = 3                 # per face: a failed arrival or an unobserved frame sends the robot back to the stand


def _kind_id(kind):
    return (NEG_KINDS + REAL_KINDS + RECORD_KINDS).index(kind)


def scene_draws(group: str, kind: str, index: int, band: str) -> dict:
    """All random draws of one scene, deterministic in (band, group, kind, index)."""
    key = [BANDS[band], GROUPS.index(group), _kind_id(kind), index]
    rng = np.random.default_rng(key)
    lateral_m = 0.03 if MODEL[band] == "v1" else 0.10  # one draw either way: v1's stream is unchanged
    d = {"seed_key": key, "model": MODEL[band], "along": rng.uniform(-0.1, 0.1),
         "lateral": rng.uniform(-lateral_m, lateral_m), "yaw": rng.uniform(-5.0, 5.0)}
    for f in FACES:
        d[f"face{f}"] = {"dx": rng.uniform(-0.1, 0.1), "dy": rng.uniform(-0.1, 0.1), "heading": rng.uniform(-5.0, 5.0),
                         "tilt": rng.uniform(-1.0, 1.0), "height": rng.uniform(-0.01, 0.01),
                         "noise": int(rng.integers(0, 2**31 - 1))}
    d["neighbour"] = int(rng.choice((-1, 1)))
    d["plate"] = {"h": rng.uniform(0.01, 0.03), "len": rng.uniform(0.6, 1.1), "w": rng.uniform(0.1, 0.42),
                  "near": rng.uniform(0.03, 0.3), "face": int(rng.choice(FACES))}
    d["occluder_face"] = int(rng.choice(FACES))
    lrng = np.random.default_rng([BANDS[band], GROUPS.index(group), _kind_id(kind), index, 7])
    pick_face, pick_gap = int(lrng.choice(FACES)), int(lrng.integers(0, 2))
    ledges = []
    for f in FACES:
        for gap in (0, 1):
            ledges.append({"face": f, "gap": gap, "h": lrng.uniform(0.004, 0.040), "setback": lrng.uniform(0.0, 0.02)})
    if kind.endswith("+ledge1"):
        ledges = [x for x in ledges if x["face"] == pick_face and x["gap"] == pick_gap]
    elif kind.endswith("+ledge2"):
        ledges = [x for x in ledges if x["face"] == pick_face]
    elif kind == "damaged+ledge":
        ledges = [x for x in ledges if x["face"] == pick_face and x["gap"] == 0]
    elif not kind.endswith("+ledge4"):
        ledges = []
    d["ledges"] = ledges
    d["damaged_face"] = pick_face
    return d


def ledge4_inside_band(group: str, kind: str, draws: dict) -> bool:
    """Four-gap ledges all at an indistinguishable height: outside the operating assumption (plan section 5)."""
    if kind != "grounded+ledge4":
        return False
    db_mm = 1e3 * S.geometry(group.split(":")[0]).deck_bottom_m
    return all(BAND_MM[0] <= 1e3 * x["h"] - db_mm <= BAND_MM[1] for x in draws["ledges"])


def face_case(group: str, kind: str, draws: dict, face: int, smooth: bool, k_name: str,
              dims: tuple = (), variant: dict | None = None) -> S.SideCase:
    """The candidate error is in the evidence-face frame for both hypotheses: along the face (model x) +-100 mm, along
    its normal (model y) +-30 mm. For x that is insertion axis / lateral; for T11 y the insertion axis is the normal."""
    variant = variant or {}
    shape, axis = group.split(":")
    base = kind.split("+")[0]
    extra = kind.split("+")[1] if "+" in kind else ""
    fd = draws[f"face{face}"]
    kw = dict(face=face, theta_e_deg=THETA_E[axis], range_m=1.35,
              stand_dx_m=fd["dx"], stand_dy_m=fd["dy"], heading_err_deg=fd["heading"],
              tilt_err_deg=fd["tilt"], height_err_m=fd["height"], noise_seed=fd["noise"], smooth=smooth,
              intrinsics=S.D435I_640 if k_name == "d435i" else None,
              along_err_m=draws["along"], lateral_err_m=draws["lateral"], yaw_err_deg=draws["yaw"], dims_delta_m=dims)
    if "loc" in fd:  # v2: this capture's pose error on top of the target's
        kw.update(along_err_m=kw["along_err_m"] + fd["loc"][0], lateral_err_m=kw["lateral_err_m"] + fd["loc"][1],
                  yaw_err_deg=kw["yaw_err_deg"] + fd["loc"][2])
    if variant.get("noise") == "off":
        kw["noise_seed"] = None
    if "normal_err_m" in variant:      # the face-normal (model y) candidate error at a fixed size, sign drawn
        kw["lateral_err_m"] = math.copysign(variant["normal_err_m"], draws["lateral"])
    if extra == "nb":
        kw["neighbour"] = draws["neighbour"]
    if extra == "plate":
        p = draws["plate"]
        kw["plate"] = S.Plate(p["h"], p["len"], p["w"], p["near"], face=p["face"])
    if extra.startswith("ledge") or kind == "damaged+ledge":
        kw["ledges"] = tuple(S.Ledge(x["gap"], x["h"], x["setback"], face=x["face"]) for x in draws["ledges"])
    if extra == "occluder":                 # one physical board, present in both faces' renders
        kw.update(occluder=True, occluder_face=draws["occluder_face"])
    if extra == "stacked":
        kw.update(stacked=True, support_z_m=S.geometry(shape).overall_height_m)
    if base == "damaged":
        kw["damaged_face"] = draws["damaged_face"]
    return S.SideCase(shape, base, **kw)


def retry_stand(draws: dict, face: int, attempt: int) -> dict:
    """A fresh arrival (and a fresh frame) for a retried stand; the calibration error stays."""
    fd = draws[f"face{face}"]
    if attempt > 0:
        rng = np.random.default_rng([*draws["seed_key"], 11, face + 1, attempt])
        fd = {**fd, "dx": rng.uniform(-0.1, 0.1), "dy": rng.uniform(-0.1, 0.1), "heading": rng.uniform(-5.0, 5.0),
              "noise": int(rng.integers(0, 2**31 - 1))}
    if draws.get("model", "v1") == "v2":
        # The localisation at this capture: the target in the camera's frame moves by it (along, normal, yaw).
        lrng = np.random.default_rng([*draws["seed_key"], 13, face + 1, attempt])
        fd = {**fd, "loc": (lrng.uniform(-0.02, 0.02), lrng.uniform(-0.02, 0.02), lrng.uniform(-1.0, 1.0))}
    return fd


def judge_face(group, kind, draws, face, smooth, k_name, dims=(), variant=None) -> dict:
    """The face as the mission sees it: drive to the stand and judge the frame; a failed arrival check or an
    ``unobserved`` frame is captured again from a fresh arrival, up to CAPTURES; the first ``present`` / ``absent``
    stops it (it does not keep looking for ``present``)."""
    for attempt in range(CAPTURES):
        rec = _judge_once(group, kind, {**draws, f"face{face}": retry_stand(draws, face, attempt)}, face, smooth,
                          k_name, dims, variant)
        rec["attempts"] = attempt + 1
        if rec["verdict"] != "unobserved":
            break
    return rec


def _judge_once(group, kind, draws, face, smooth, k_name, dims=(), variant=None) -> dict:
    axis = group.split(":")[1]
    case = face_case(group, kind, draws, face, smooth, k_name, dims, variant)
    scene, target, _ = S.render_side(case)
    ev = B.evaluate_side(scene, target, face=face)
    arrived = B.arrival_ok(scene, target, face=face, axis=axis)
    cam = np.asarray(scene.base_from_optical.translation_m)[:2]
    true_c, true_yaw = S.true_pose(case)
    truth = replace(target, centre_xy=true_c, model_yaw_rad=true_yaw)
    verdict = ev.verdict if arrived else "unobserved"
    return {"face": face, "view_face": S.view_face(case), "seen_face": ev.face, "verdict": verdict, "reason": ev.reason if arrived else "arrival",
            "evidence_verdict": ev.verdict, "evidence_reason": ev.reason, "arrived": arrived,
            "theta_e_deg": B.stand_angle_deg(cam, truth, face=face),           # the stand actually reached
            "range_m": float(np.hypot(*(np.asarray(truth.centre_xy) - cam))),
            "theta_e_est_deg": B.stand_angle_deg(cam, target, face=face),      # as the arrival check sees it
            "range_est_m": float(np.hypot(*(np.asarray(target.centre_xy) - cam))),
            "gaps": [[g.state, g.board, g.open, g.occluded] for g in ev.gaps]}


def policy(faces) -> str:
    verdicts = [f["verdict"] for f in faces]
    if "absent" in verdicts:
        return "absent"
    if all(v == "present" for v in verdicts):
        return "present"
    return "unobserved"


def job_list(groups, n, records, band):
    """Per group: n counted negatives, n real, ``records`` of each record kind. Negatives whose four-gap ledges fall
    inside the band are outside the operating assumption: they are kept and recorded apart, and the counted
    negatives continue past them so the bound still has n scenes."""
    jobs = []
    for group in groups:
        counted, i = 0, 0
        while counted < n:
            kind = NEG_KINDS[i % len(NEG_KINDS)]
            jobs.append((group, kind, i))
            counted += not ledge4_inside_band(group, kind, scene_draws(group, kind, i, band))
            i += 1
        for i in range(n):
            jobs.append((group, REAL_KINDS[i % len(REAL_KINDS)], i))
        for kind in RECORD_KINDS:
            for i in range(records):
                jobs.append((group, kind, i))
    return jobs


def variant_name(dims=(), variant=None) -> str:
    """'' for the nominal population; records off it are named by what they change."""
    parts = [f"{k}={v:+.4f}" for k, v in dims] + [f"{k}={v}" for k, v in sorted((variant or {}).items())]
    return ",".join(parts)


def run_job(args):
    """args = (group, kind, index, band[, dims, variant items, (n, records)])."""
    group, kind, index, band, *rest = args
    dims = tuple(tuple(d) for d in rest[0]) if rest else ()
    variant = dict(rest[1]) if len(rest) > 1 else {}
    plan_n = list(rest[2]) if len(rest) > 2 else None
    draws = scene_draws(group, kind, index, band)
    rows = []
    for smooth_name, k_name in CONFIGS:
        faces = [judge_face(group, kind, draws, f, smooth_name == "on", k_name, dims, variant) for f in FACES]
        rows.append({"band": band, "model": MODEL[band], "seed_base": BANDS[band], "judge_sha256": JUDGE_SHA256,
                     "tools_sha256": TOOLS_SHA256,
                     "group": group, "kind": kind, "index": index, "smooth": smooth_name, "K": k_name,
                     "variant": variant_name(dims, variant), "faces": faces, "final": policy(faces),
                     "opposite_faces": sorted(f["view_face"] for f in faces) == [-1, 1],
                     "outside_assumption": ledge4_inside_band(group, kind, draws), "plan_n": plan_n,
                     "ledges_m": [x["h"] for x in draws["ledges"]]})
    return rows


def _dims(text):
    if not text:
        return ()
    out = []
    for item in text.split(","):
        k, v = item.split("=")
        if k not in S.VERTICAL_FIELDS:
            raise SystemExit(f"--dims: {k} is not one of {S.VERTICAL_FIELDS}")
        out.append((k, float(v)))
    return tuple(out)


def cmd_run(a):
    dims = _dims(a.dims)
    variant = {}
    if a.noise_off:
        variant["noise"] = "off"
    if a.normal_err_m is not None:
        variant["normal_err_m"] = a.normal_err_m
    if (dims or variant) and a.band not in DEV_BANDS:
        raise SystemExit("--dims / --noise-off / --normal-err-m are dev records; eval is opened once, nominal")
    jobs = job_list(a.groups.split(","), a.n, a.records, a.band)
    if a.kinds:
        jobs = [j for j in jobs if j[1] in a.kinds.split(",")]
    jobs = [j + (a.band, dims, tuple(variant.items()), (a.n, a.records))
            for i, j in enumerate(jobs) if i % a.of == a.shard]
    with open(a.out, "w") as fh, ProcessPoolExecutor(max_workers=a.workers) as pool:
        for rows in pool.map(run_job, jobs, chunksize=1):
            for r in rows:
                fh.write(json.dumps(r) + "\n")
            fh.flush()


# -- statistics (exact binomial, no scipy) ---------------------------------------------------------------------------

def _binom_cdf(k, n, p):
    if p <= 0.0:
        return 1.0
    if p >= 1.0:
        return 0.0 if k < n else 1.0
    return sum(math.comb(n, i) * p**i * (1 - p) ** (n - i) for i in range(k + 1))


def upper_bound(k, n, alpha=0.05):
    """One-sided (1 - alpha) Clopper-Pearson upper bound on a rate with k events in n."""
    if k >= n:
        return 1.0
    lo, hi = 0.0, 1.0
    for _ in range(80):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _binom_cdf(k, n, mid) > alpha else (lo, mid)
    return hi


def lower_bound(k, n, alpha=0.05):
    return 1.0 - upper_bound(n - k, n, alpha)


def _scene_set_complete(sel, band, group) -> bool:
    """Every scene the run planned is here, no more (Codex task-2 P2: a lost shard or row must not pass)."""
    plans = {tuple(r["plan_n"]) if r.get("plan_n") else None for r in sel}
    if len(plans) != 1 or None in plans:
        return False
    (n, records), = plans
    want = {(k, i) for _, k, i in job_list([group], n, records, band)}
    return {(r["kind"], r["index"]) for r in sel} == want


def summarise(rows):
    """Per band x group x variant x configuration. A scene counted twice would shrink the bound: refused."""
    seen = set()
    for r in rows:
        key = (r["band"], r["group"], r["kind"], r["index"], r["variant"], r["smooth"], r["K"])
        if key in seen:
            raise ValueError(f"duplicate scene row {key}")
        seen.add(key)
    for band in {r["band"] for r in rows}:
        # Shards and partial reruns merged into one bound must be one population (Codex anchor round 1 P2 2): the
        # band's model and seed base, one judge and one evaluator/recipe. v1 rows from before these fields carry none.
        sel = [r for r in rows if r["band"] == band]
        fields = ("model", "seed_base", "judge_sha256", "tools_sha256")
        # Only v1 rows from before the fields go unchecked: every row without any of them (Codex anchor round 3).
        if MODEL[band] != "v1" or any(f in r for r in sel for f in fields):
            for field, want in (("model", MODEL[band]), ("seed_base", BANDS[band])):
                got = {r.get(field) for r in sel}
                if got != {want}:
                    raise ValueError(f"band {band}: {field} {sorted(map(str, got))}, expected {want!r}")
            for field in ("judge_sha256", "tools_sha256"):
                got = {r.get(field) for r in sel}
                if len(got) != 1 or None in got:
                    raise ValueError(f"band {band}: rows from {len(got)} {field} values")
    out = {}
    for band, group, variant in sorted({(r["band"], r["group"], r["variant"]) for r in rows}):
        for smooth, k_name in CONFIGS:
            sel = [r for r in rows if r["band"] == band and r["group"] == group and r["variant"] == variant
                   and r["smooth"] == smooth and r["K"] == k_name]
            if not sel:
                continue
            neg = [r for r in sel if r["kind"] in NEG_KINDS and not r["outside_assumption"]]
            outside = [r for r in sel if r["outside_assumption"]]
            real = [r for r in sel if r["kind"] in REAL_KINDS]
            real_all = [r for r in sel if r["kind"] in REAL_KINDS + ("slab", "pallet+plate", "pallet+ledge4",
                                                                      "pallet+occluder", "pallet+stacked")]
            k_fp = sum(r["final"] == "present" for r in neg)
            k_tp = sum(r["final"] == "present" for r in real)
            k_fn = sum(r["final"] == "absent" for r in real_all)
            res = {
                "neg_n": len(neg), "neg_present": k_fp, "neg_upper": upper_bound(k_fp, len(neg)) if neg else None,
                "real_n": len(real), "real_present": k_tp, "real_lower": lower_bound(k_tp, len(real)) if real else None,
                "real_all_n": len(real_all), "real_absent": k_fn,
                "real_absent_upper": upper_bound(k_fn, len(real_all)) if real_all else None,
                "outside_assumption_n": len(outside),
                "outside_assumption_present": sum(r["final"] == "present" for r in outside),
                "opposite_faces_ok": all(r["opposite_faces"] for r in sel),
                "by_kind": {},
            }
            for kind in NEG_KINDS + REAL_KINDS + RECORD_KINDS:
                ks = [r for r in sel if r["kind"] == kind]
                if ks:
                    res["by_kind"][kind] = {v: sum(r["final"] == v for r in ks) for v in B.VERDICTS} | {"n": len(ks)}
            reasons = {}
            for r in sel:
                for f in r["faces"]:
                    if f["verdict"] == "unobserved":
                        key = f"{r['kind']}:{f['reason']}"
                        reasons[key] = reasons.get(key, 0) + 1
            res["unobserved_reasons"] = dict(sorted(reasons.items(), key=lambda kv: -kv[1]))
            complete = bool(not variant and neg and real and _scene_set_complete(sel, band, group)
                            and all(kind in res["by_kind"] for kind in NEG_KINDS + REAL_KINDS + RECORD_KINDS))
            res["complete"] = complete
            res["pass"] = bool(complete and res["opposite_faces_ok"]
                               and res["neg_upper"] <= 0.01 and res["real_lower"] >= 0.90
                               and res["real_absent_upper"] <= 0.01
                               and res["by_kind"]["pallet+occluder"]["present"] == 0
                               and res["by_kind"]["pallet+stacked"]["present"] == 0)
            out[f"{band} {group} smooth={smooth} K={k_name}" + (f" [{variant}]" if variant else "")] = res
    return out


def cmd_summary(a):
    rows = []
    for path in a.rows:
        with open(path) as fh:
            rows.extend(json.loads(line) for line in fh if line.strip())
    out = summarise(rows)
    for key, res in out.items():
        fmt = lambda v: "-" if v is None else f"{v:.4f}"
        print(f"{key}: neg present {res['neg_present']}/{res['neg_n']} (upper {fmt(res['neg_upper'])}), "
              f"real present {res['real_present']}/{res['real_n']} (lower {fmt(res['real_lower'])}), "
              f"real absent {res['real_absent']}/{res['real_all_n']}, "
              f"4-gap band {res['outside_assumption_present']}/{res['outside_assumption_n']}, pass={res['pass']}")
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--band", choices=tuple(BANDS), required=True)
    r.add_argument("--groups", default=",".join(GROUPS))
    r.add_argument("--n", type=int, default=299)
    r.add_argument("--records", type=int, default=32)
    r.add_argument("--out", required=True)
    r.add_argument("--shard", type=int, default=0)
    r.add_argument("--of", type=int, default=1)
    r.add_argument("--workers", type=int, default=1)
    r.add_argument("--kinds", default="", help="only these kinds (records, the tolerance run)")
    r.add_argument("--dims", default="", help="field=delta,... off-nominal article (dev only)")
    r.add_argument("--noise-off", action="store_true", help="no team noise (dev record)")
    r.add_argument("--normal-err-m", type=float, help="face-normal candidate error at this size (dev record)")
    s = sub.add_parser("summary")
    s.add_argument("rows", nargs="+")
    s.add_argument("--json")
    a = p.parse_args(argv)
    cmd_run(a) if a.cmd == "run" else cmd_summary(a)


if __name__ == "__main__":
    main()
