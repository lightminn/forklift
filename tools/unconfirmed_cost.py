"""The unconfirmed position plan's cost measurement (docs/plans/2026-11-02-unconfirmed-position.md 1.3), offline, before
any implementation.

For each believed start (one JSON line ``{"id", "base": [x, y, yaw]}``, map frame) on one world's survey bundle and
one stationary capture of those starts (B4a's capture, the start's index by id), the start check of plan 1.2
(``forklift_core.localization.twin_check``): the global search on the capture's scans gives the pool (its kept
candidates) and the pre-suppression leaves scoring >= R_RESID of its best; with the synthetic twins they form the
hypothesis set; the residual ambiguity is judged conservatively. Recorded per start: confirmed or not and why, the
hypothesis set's size and sources, impossible regions, whether the truth (the capture's settled base) lies within a
hypothesis region (completeness), whether the straight reverse corridor behind the start is clear of occupied survey
cells for ``--rear-m`` (self-lock, obstacles only: the pallet zones are not in the survey map, so this is a lower
bound), and the time and work.

    python tools/unconfirmed_cost.py run --bundle B --capture C --believed starts.jsonl --out rows.jsonl
    python tools/unconfirmed_cost.py summary --rows rows.jsonl [--rows rows2.jsonl ...] --out summary.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))

import global_match_eval as GE  # noqa: E402

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import local_match as L  # noqa: E402
from forklift_core.localization import twin_check as T  # noqa: E402
from forklift_core.localization.relocalization_samples import (  # noqa: E402
    FOOTPRINT,
    REAR_AXLE_OFFSET_M,
    _overlap,
    rect_corners,
)
from forklift_core.localization.slam_map import Grid  # noqa: E402
from forklift_core.localization.slam_pose import compose  # noqa: E402

E_MIN = 0.80  # B-P1b, frozen
REAR_M = 0.62  # plan 3: D_MIN + the stop + the zone growth over D_MIN (the estimate; plan 1.4 computes the real one)


def rear_clear(grid: Grid, base, rear_m: float = REAR_M) -> bool:
    """No occupied survey cell in the straight corridor the truck sweeps backing up ``rear_m`` from ``base`` (behind
    its footprint, its width). Unknown cells are not obstacles here: the live obstacle layer sees only what is there."""
    x, y, yaw = base
    rx, ry = x - REAR_AXLE_OFFSET_M * math.cos(yaw), y - REAR_AXLE_OFFSET_M * math.sin(yaw)
    rect = rect_corners(rx, ry, yaw, -FOOTPRINT.rear_m - rear_m, -FOOTPRINT.rear_m, FOOTPRINT.half_width_m)
    res, (ox, oy, _) = grid.resolution_m, grid.origin
    c0, c1 = int(math.floor((rect[:, 0].min() - ox) / res)), int(math.floor((rect[:, 0].max() - ox) / res))
    r0, r1 = int(math.floor((rect[:, 1].min() - oy) / res)), int(math.floor((rect[:, 1].max() - oy) / res))
    h, w = grid.data.shape
    for r in range(max(r0, 0), min(r1, h - 1) + 1):
        for c in range(max(c0, 0), min(c1, w - 1) + 1):
            if grid.data[r, c] < 50:
                continue
            cell = rect_corners(ox + (c + 0.5) * res, oy + (r + 0.5) * res, 0.0, -res / 2, res / 2, res / 2)
            if _overlap(rect, cell):
                return False
    return True


def pool_and_leaves(grid: Grid, pts, cfg: G.MatchConfig, index: G.MapIndex, r_resid: float) -> tuple:
    """The global search's kept candidates (base poses) and its pre-suppression leaves >= r_resid of its best."""
    match = G.global_match(grid, pts, cfg, index=index)
    pool = [L._bases(np.array([c.pose]))[0] for c in match.candidates] if match.status == "ok" else []
    leaves = []
    if match.status == "ok":
        s = G._Search(grid, G._thin(np.asarray(pts, float).reshape(-1, 2), grid.resolution_m), cfg, index,
                      G._Budget(cfg))
        h, r, c, sc = G._lattice_leaves(s, cfg)
        if len(sc):
            keep = sc >= r_resid * float(sc.max())
            lasers = np.array([s.pose(h[k], r[k], c[k]) for k in np.flatnonzero(keep)])
            leaves = list(L._bases(lasers)) if len(lasers) else []
    return match.status, pool, leaves


def _in_region(truth, pose) -> bool:
    d, a = T._gap(truth, pose)
    return d <= T.REGION_M and a <= T.REGION_RAD


def cmd_run(a) -> int:
    cap = np.load(a.capture, allow_pickle=False)
    meta = json.loads(str(cap["meta"]))
    manifest = json.loads((a.bundle / "manifest.json").read_text())
    if meta["map_id"] != manifest["map_id"]:
        raise SystemExit("the capture was taken for another map than this bundle's")
    mfw = tuple(float(v) for v in manifest.get("map_from_world", (0.0, 0.0, 0.0)))
    grid = Grid.from_npz(a.bundle / "grid.npz")
    cfg = G.MatchConfig(**json.loads(a.config)) if a.config else G.MatchConfig()
    index, cons = G.MapIndex(grid, cfg), T.ConservativeIndex(grid, cfg)
    ids = list(map(str, cap["id"]))
    believed = [json.loads(line) for line in Path(a.believed).read_text().splitlines() if line.strip()]
    with open(a.out, "w") as f:
        for row in believed:
            sid = str(row["id"])
            out = {"id": sid, "kind": row.get("kind", "start"), "base": row["base"]}
            if sid not in ids:
                out["status"] = "not_captured"
                f.write(json.dumps(out) + "\n")
                continue
            i = ids.index(sid)
            if not bool(cap["valid"][i]):
                out["status"] = "invalid_capture"
                f.write(json.dumps(out) + "\n")
                continue
            truth = tuple(compose(mfw, tuple(cap["base_truth"][i])))
            b = tuple(float(v) for v in row["base"])
            t0 = time.monotonic()
            pts = GE.capture_points(cap, i)
            status, pool, leaves = pool_and_leaves(grid, pts, cfg, index, T.R_RESID)
            t_search = time.monotonic() - t0
            hyps = T.hypotheses(b, pool=pool, leaves=leaves, synthetic=T.synthetic_twins(b))
            res = T.residual_ambiguity(grid, pts, b, hyps, cfg, e_min=E_MIN, index=index, cons=cons)
            regions = [b] + [p for p, _ in hyps]
            out.update(status="ok", search=status, confirmed=res.confirmed, reasons=list(res.reasons),
                       s_b=res.s_b, a_b=res.a_b, n_pool=len(pool), n_leaves=len(leaves), n_hyps=len(hyps),
                       sources=sorted({src.split(" ")[0] for _, src in hyps}),
                       competitors=[{"pose": list(p), "score": s, "source": src} for p, s, src in res.competitors[:20]],
                       n_competitors=len(res.competitors), impossible=len(res.impossible),
                       truth=list(truth), truth_in_set=any(_in_region(truth, p) for p in regions),
                       believed_is_truth=_in_region(truth, b), rear_clear=rear_clear(grid, b, a.rear_m),
                       search_s=round(t_search, 3), check_s=round(res.wall_s, 3), work=int(res.work))
            f.write(json.dumps(out) + "\n")
    return 0


def _q(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, math.ceil(q * (len(v) - 1)))] if v else None


def summarize(rows: list) -> dict:
    ok = [r for r in rows if r.get("status") == "ok"]
    unconf = [r for r in ok if not r["confirmed"]]
    reasons: dict = {}
    for r in unconf:
        for why in r["reasons"]:
            reasons[why.split(":")[0] if why.startswith("search") else why] = reasons.get(why, 0) + 1
    times = [r["search_s"] + r["check_s"] for r in ok]
    return {"rows": len(rows), "checked": len(ok), "skipped": {s: sum(r.get("status") == s for r in rows)
                                                              for s in ("not_captured", "invalid_capture")},
            "unconfirmed": len(unconf), "unconfirmed_share": len(unconf) / len(ok) if ok else None,
            "reasons": reasons, "truth_in_set": sum(r["truth_in_set"] for r in ok),
            "believed_is_truth": sum(r["believed_is_truth"] for r in ok),
            "rear_blocked": sum(not r["rear_clear"] for r in ok),
            "hyps": {"median": _q([r["n_hyps"] for r in ok], 0.5), "max": max((r["n_hyps"] for r in ok), default=None)},
            "leaves": {"median": _q([r["n_leaves"] for r in ok], 0.5),
                       "max": max((r["n_leaves"] for r in ok), default=None)},
            "time_s": {"p50": _q(times, 0.5), "p95": _q(times, 0.95), "max": max(times, default=None)},
            "work_max": max((r["work"] for r in ok), default=None)}


def cmd_summary(a) -> int:
    out = {}
    for path in a.rows:
        rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
        out[Path(path).name] = summarize(rows)
    Path(a.out).write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--bundle", type=Path, required=True)
    r.add_argument("--capture", type=Path, required=True)
    r.add_argument("--believed", type=Path, required=True, help="JSON lines {id, base, [kind]}")
    r.add_argument("--config", default=None, help="MatchConfig overrides (JSON), as the frozen match_config.json")
    r.add_argument("--rear-m", type=float, default=REAR_M)
    r.add_argument("--out", type=Path, required=True)
    s = sub.add_parser("summary")
    s.add_argument("--rows", type=Path, action="append", required=True)
    s.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    return cmd_run(a) if a.cmd == "run" else cmd_summary(a)


if __name__ == "__main__":
    sys.exit(main())
