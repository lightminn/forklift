#!/usr/bin/env python3
"""Does the learned proposer give the same answer for the same RGB? (deterministic-run plan, step 1)

Starts the learned worker (``forklift_learned.worker``, the learned venv) with the given providers, sends each RGB
of the given .npz files (every array of shape (H, W, 3) uint8) and hashes the reply's masks and scores. Two processes
with the same inputs are then compared by hash alone; ``--repeat`` also sends each image again in the same process.
Writes one JSON to ``--out``.

    python tools/learned_determinism_probe.py --python LEARNED_PY --onnx best.onnx --providers CUDAExecutionProvider \
        --out a.json frames.npz [...]
"""

import argparse
import hashlib
import io
import json
import subprocess
from pathlib import Path

import numpy as np

from forklift_learned.worker import _recv, _send


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--python", required=True)
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--providers", default="CPUExecutionProvider")
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--conf", type=float, default=0.05)
    ap.add_argument("--repeat", type=int, default=2)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("images", nargs="+", type=Path)
    args = ap.parse_args()
    proc = subprocess.Popen([args.python, "-m", "forklift_learned.worker", args.onnx, str(args.conf), args.providers,
                             str(args.threads), "0"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    _send(proc.stdin, b"")
    active = [str(p) for p in np.load(io.BytesIO(_recv(proc.stdout)))["providers"]]
    out = {"providers_asked": args.providers, "providers_active": active, "replies": []}
    for path in args.images:
        z = np.load(path)
        for name in z.files:
            rgb = z[name]
            if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
                continue
            buf = io.BytesIO()
            np.save(buf, np.ascontiguousarray(rgb))
            hashes = []
            for _ in range(args.repeat):
                _send(proc.stdin, buf.getvalue())
                reply = np.load(io.BytesIO(_recv(proc.stdout)))
                h = hashlib.sha256()
                for key in ("masks", "scores"):
                    h.update(np.ascontiguousarray(reply[key]).tobytes())
                hashes.append({"hash": h.hexdigest()[:16], "scores": [round(float(s), 6) for s in reply["scores"]]})
            out["replies"].append({"image": f"{path.name}:{name}", "runs": hashes})
    proc.stdin.close()
    proc.wait(timeout=30)
    args.out.write_text(json.dumps(out, indent=1) + "\n")
    same = sum(len({r["hash"] for r in x["runs"]}) == 1 for x in out["replies"])
    print("LEARNED_PROBE", args.providers, active, f"{same}/{len(out['replies'])} images same within the process")


if __name__ == "__main__":
    main()
