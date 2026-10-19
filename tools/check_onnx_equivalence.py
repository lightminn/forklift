#!/usr/bin/env python3
"""ONNX equivalence of the learned proposer (stage 3 plan, task 5): for each image, the Ultralytics PyTorch
prediction of the checkpoint against forklift_learned.proposer on the exported ONNX -- same instance count,
scores within --score-tol, masks (camera grid) IoU >= --iou-min after matching by IoU. Exit 0 when all hold.
Runs in the learned venv (ultralytics + onnxruntime)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT)]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("onnx", type=Path)
    ap.add_argument("images", nargs="+", type=Path)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--score-tol", type=float, default=1e-3)
    ap.add_argument("--iou-min", type=float, default=0.99)
    args = ap.parse_args(argv)
    from PIL import Image
    from ultralytics import YOLO

    from forklift_learned.proposer import LearnedProposer

    torch_model = YOLO(str(args.checkpoint))
    onnx_model = LearnedProposer(args.onnx, conf=args.conf, providers=["CPUExecutionProvider"])
    rows, ok = [], True
    for path in args.images:
        rgb = np.asarray(Image.open(path).convert("RGB"))
        res = torch_model.predict(rgb[:, :, ::-1].copy(), imgsz=(rgb.shape[0], rgb.shape[1]), conf=args.conf,
                                  iou=0.6, max_det=8, retina_masks=True, verbose=False)[0]
        t_masks = [] if res.masks is None else [m.astype(bool) for m in res.masks.data.cpu().numpy()]
        t_scores = [] if res.boxes is None else res.boxes.conf.cpu().numpy().tolist()
        scene = type("S", (), {"rgb": rgb})()
        o = onnx_model.propose(scene)
        o_masks, o_scores = [p.mask for p in o], [p.score for p in o]
        ious, diffs = [], []
        for tm, ts in zip(t_masks, t_scores):
            best = max(range(len(o_masks)), key=lambda j: (tm & o_masks[j]).sum() / max(1, (tm | o_masks[j]).sum()),
                       default=None)
            if best is None:
                ious.append(0.0)
                continue
            ious.append(float((tm & o_masks[best]).sum() / max(1, (tm | o_masks[best]).sum())))
            diffs.append(abs(ts - o_scores[best]))
        good = len(t_masks) == len(o_masks) and all(i >= args.iou_min for i in ious) and all(
            d <= args.score_tol for d in diffs)
        ok &= good
        rows.append({"image": path.name, "torch": len(t_masks), "onnx": len(o_masks), "min_iou": min(ious, default=None),
                     "max_score_diff": max(diffs, default=None), "ok": good})
    print(json.dumps({"ok": ok, "rows": rows}, indent=1))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
