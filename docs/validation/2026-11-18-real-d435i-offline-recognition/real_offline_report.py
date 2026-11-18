#!/usr/bin/env python3
"""Tables of real_offline.py's JSON (the P0-5b offline check): per capture pair (640 and 848 together) and arm.

  python real_offline_report.py real_offline.json[.xz] [--json report.json]
"""

import argparse
import collections
import json
import lzma
import sys
from pathlib import Path

import numpy as np


def group_of(name: str) -> str:
    return name.rsplit("_", 1)[0]


def pct(values, q):
    return float(np.percentile(values, q)) if len(values) else None


def summarize(data: dict) -> dict:
    rows = collections.defaultdict(lambda: collections.defaultdict(list))
    for cap in data["captures"]:
        g = group_of(cap["capture"])
        for arm, body in cap["arms"].items():
            for fr in body["frames"]:
                rows[(g, arm)]["frames"].append(fr)
    out = []
    for (g, arm), d in sorted(rows.items()):
        frames = d["frames"]
        rec = {"group": g, "arm": arm, "frames": len(frames)}
        if g.startswith("wall"):
            rec["wall_candidates"] = sum(len(f.get("wall_candidates", [])) for f in frames)
            rec["frames_with_candidate"] = sum(1 for f in frames if f.get("wall_candidates"))
            rec["wall_hypotheses"] = dict(collections.Counter(c["hypothesis"] for f in frames
                                                               for c in f.get("wall_candidates", [])))
            out.append(rec)
            continue
        rec["toy"] = dict(collections.Counter(f["toy"] for f in frames))
        comp = [f["toy_component"] for f in frames if "toy_component" in f]
        if comp:
            rec["component_stage"] = dict(collections.Counter(c["stage"] for c in comp))
            rec["component_face_stage"] = dict(collections.Counter(c.get("first_face", {}).get("stage", "-")
                                                                   for c in comp))
            rec["component_width_m_p50"] = pct([c["width_m"] for c in comp if "width_m" in c], 50)
            rec["component_purity_p50"] = pct([c["purity"] for c in comp if "purity" in c], 50)
            rec["component_cover_p50"] = pct([c["cover"] for c in comp if "cover" in c], 50)
            faces = [c["first_face"] for c in comp if "scale_m" in c.get("first_face", {})]
        else:
            faces = [f["toy_face"] for f in frames if "scale_m" in f.get("toy_face", {})]
            rec["roi_face_stage"] = dict(collections.Counter(f["toy_face"]["stage"] for f in frames
                                                             if "toy_face" in f))
        if faces:
            rec["face_scale_mm_p50"] = 1e3 * pct([f["scale_m"] for f in faces], 50)
            rec["face_scale_mm_p90"] = 1e3 * pct([f["scale_m"] for f in faces], 90)
            rec["face_scale_limit_mm"] = 1e3 * pct([f["scale_limit_m"] for f in faces], 50)
        cands = [f["toy_candidate"] for f in frames if "toy_candidate" in f]
        rec["candidates"] = len(cands)
        rec["consistent"] = sum(1 for c in cands if c["consistent"])
        rec["spacing_ok"] = sum(1 for c in cands if c["consistent"] and c["spacing_ok"])
        rec["hypothesis"] = dict(collections.Counter(c["hypothesis"] for c in cands))
        rec["identity"] = dict(collections.Counter(c["identity"] for c in cands))
        rec["fit_reasons"] = dict(collections.Counter(r for c in cands for r in c["fit_reasons"]))
        rec["reasons"] = dict(collections.Counter(r for c in cands for r in c["reasons"]))
        for key in ("x_m", "y_m", "yaw_deg", "spacing_m"):
            v = [c["err"][key] for c in cands]
            rec[f"err_{key}_p50"] = pct(v, 50)
            rec[f"err_{key}_absmax"] = float(np.max(np.abs(v))) if v else None
        good = [c["err"] for c in cands if c["consistent"]]
        rec["geometry"] = dict(collections.Counter(c["geometry"] for c in cands))
        if good:  # the consistent readings alone (the document's table)
            rec["consistent_x_mm"] = [1e3 * min(e["x_m"] for e in good), 1e3 * max(e["x_m"] for e in good)]
            rec["consistent_abs_y_mm_max"] = 1e3 * max(abs(e["y_m"]) for e in good)
            rec["consistent_yaw_deg_max"] = max(e["yaw_deg"] for e in good)
            rec["consistent_spacing_mm"] = [1e3 * min(e["spacing_m"] for e in good),
                                            1e3 * max(e["spacing_m"] for e in good)]
        rec["inconsistent_spacing_mm"] = [1e3 * c["err"]["spacing_m"] for c in cands if not c["consistent"]]
        rec["decision"] = dict(collections.Counter(f["decision"]["status"] + ":" + ",".join(f["decision"]["reasons"])
                                                   for f in frames if "decision" in f))
        rec["decision_toy"] = sum(1 for f in frames if f.get("decision", {}).get("toy"))
        rec["others"] = sum(len(f.get("others", [])) for f in frames)
        rec["proposer_failures"] = dict(collections.Counter(r for f in frames for r in f["proposer_failures"]))
        rec["proposer_ms_p50"] = pct([f["proposer_ms"] for f in frames], 50)
        rec["fit_ms_p50"] = pct([f["fit_ms"] for f in frames], 50)
        out.append(rec)
    return {"meta": data["meta"], "rows": out}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("result", type=Path)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args(argv)
    raw = lzma.open(a.result).read() if a.result.suffix == ".xz" else a.result.read_bytes()
    rep = summarize(json.loads(raw))
    for r in rep["rows"]:
        print(json.dumps(r, ensure_ascii=False))
    if a.json:
        a.json.write_text(json.dumps(rep, indent=1, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
