"""Normalised comparison of two mission run directories (M-a plan D2, docs/plans/2026-10-15-m-a-mission-core.md).

Two runs of the same mission (same arguments, seed and inputs; the source may differ) are equal when
every recorded file matches after normalisation:

- run-specific paths (the run's own directory, its socket and map directory) and snapshot names in
  paths are replaced by placeholders;
- wall-clock fields are dropped (keys ending in ``_wall_s``, ``wall_time_s``, detection ``elapsed_s``,
  wall statistics ``wall_s_*`` such as the obstacle layer's ``wall_s_per_scan``);
- fields that differ between runs of the same source (2026-10-15: three runs of batch R seed 0 and two
  record-off near-tracking runs of seed 1 -- the SLAM map version a grid plan read (``slam_map.index`` /
  ``after_scan_id``) and the planning memory revision that counts its installs, the camera readiness
  bookkeeping of a capture, and the renderer frame numbers of pocket frame reads) are reported under
  ``nondeterministic``, not as differences;
- source provenance (``source_sha256``, ``script_sha256``) is reported under ``provenance``;
- the boundary record directory (``boundary/``), recording-only arguments (``record_boundary``,
  ``record_boundary_format``, in ``arguments`` and at the top of ``meta.json``) and the recording-only result keys
  (``setup_snapshot``, ``boundary_record_error``) are skipped;
- JSON is compared by value (key order ignored, floats exactly); arrays (``.npz`` / ``.npy``) bit for
  bit (dtype, shape and bytes); other files byte for byte; scene exports and images are skipped.

    python tools/mission_ab_compare.py RUN_A RUN_B [--json OUT]
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

SKIP_SUFFIXES = {".usd", ".usda", ".usdc", ".png", ".jpg", ".mp4", ".log", ".txt"}
SKIP_DIRS = ("boundary/",)
RECORDING_ONLY_ARGUMENTS = {"record_boundary", "record_boundary_format"}  # arguments that only add records  # the M-a0 boundary record is a record of the run, not its output
# Result keys only a boundary-recording run writes (M-a1 replay plan R3: the setup snapshot's audit, a recording
# failure note) -- skipped like the record itself.
RECORDING_ONLY_RESULT_KEYS = {"setup_snapshot", "boundary_record_error"}
WALL_KEY = re.compile(r"(_wall_s$|^wall_time_s$|^elapsed_s$|^wall_s_)")
PROVENANCE_KEYS = {"source_sha256", "script_sha256"}
NONDETERMINISTIC = (
    re.compile(r"\.slam_map\.(index|after_scan_id)$"),
    # The planning memory revision counts every memory update (SLAM map installs, hits, FREE updates,
    # retractions), so the map-install timing moves it between runs of one source (2026-10-15 near runs);
    # in this comparison the memory's content is compared only where a run saves it (grid_mission_plan_*.npz,
    # the mission plans); the replay evidence covers every grid preparation, whose responses the boundary
    # record keeps by content.
    re.compile(r"\.memory_revision$"),
    re.compile(r"\.capture_diagnostics\.(rejection_counts|sensor_frame_id)(\.|$)"),
    re.compile(r"^reads\[\d+\]\.rendering_frame"),  # renderer frame bookkeeping in pocket_frames/index.json
)
SNAPSHOT = re.compile(r"/snapshots/[^/]+/")


def _normalise_str(value: str, run: Path) -> str:
    for root, token in ((run.parent, "<RUN>"),):
        value = value.replace(str(root), token)
    return SNAPSHOT.sub("/snapshots/<SNAP>/", value)


def _walk(a, b, path: str, run_a: Path, run_b: Path, out: dict, name: str) -> None:
    if isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(set(a) | set(b), key=str):
            sub = f"{path}.{key}" if path else str(key)
            if WALL_KEY.search(str(key)) or (path in ("", "arguments") and key in RECORDING_ONLY_ARGUMENTS):
                continue
            if path == "" and name == "result.json" and key in RECORDING_ONLY_RESULT_KEYS:
                continue
            if key in PROVENANCE_KEYS:
                if a.get(key) != b.get(key):
                    out["provenance"].append(f"{name}: {key}" + _first_difference(a.get(key), b.get(key)))
                continue
            if key not in a or key not in b:
                out["differences"].append(f"{name}: {sub} only in {'A' if key in a else 'B'}")
                continue
            _walk(a[key], b[key], sub, run_a, run_b, out, name)
        return
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            _record(out, name, path, f"length {len(a)} vs {len(b)}")
        for i, (x, y) in enumerate(zip(a, b)):
            _walk(x, y, f"{path}[{i}]", run_a, run_b, out, name)
        return
    if isinstance(a, str) and isinstance(b, str):
        a, b = _normalise_str(a, run_a), _normalise_str(b, run_b)
    if type(a) is not type(b) or a != b:
        _record(out, name, path, f"{str(a)[:60]} vs {str(b)[:60]}")


def _first_difference(a, b) -> str:
    if isinstance(a, dict) and isinstance(b, dict):
        keys = sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))
        return f".{keys[0]} differs" if keys else ""
    return " differs"


def _record(out: dict, name: str, path: str, what: str) -> None:
    line = f"{name}: {path}: {what}"
    if any(p.search(path) for p in NONDETERMINISTIC):
        out["nondeterministic"].append(line)
    else:
        out["differences"].append(line)


def _arrays_equal(x: np.ndarray, y: np.ndarray) -> bool:
    return x.dtype == y.dtype and x.shape == y.shape and x.tobytes() == y.tobytes()


def compare(run_a: Path, run_b: Path) -> dict:
    run_a, run_b = Path(run_a), Path(run_b)
    out = {"run_a": str(run_a), "run_b": str(run_b), "identical": [], "differences": [], "nondeterministic": [],
           "provenance": [], "skipped": []}
    names = sorted({p.relative_to(run_a).as_posix() for p in run_a.rglob("*") if p.is_file()}
                   | {p.relative_to(run_b).as_posix() for p in run_b.rglob("*") if p.is_file()})
    for name in names:
        fa, fb = run_a / name, run_b / name
        if Path(name).suffix in SKIP_SUFFIXES or name.startswith(SKIP_DIRS):
            out["skipped"].append(name)
            continue
        if not fa.exists() or not fb.exists():
            out["differences"].append(f"{name}: only in {'A' if fa.exists() else 'B'}")
            continue
        before = len(out["differences"])
        if name.endswith(".json"):
            _walk(json.loads(fa.read_text()), json.loads(fb.read_text()), "", run_a, run_b, out, name)
        elif name.endswith(".npz"):
            za, zb = np.load(fa, allow_pickle=False), np.load(fb, allow_pickle=False)
            for key in sorted(set(za.files) | set(zb.files)):
                if key not in za.files or key not in zb.files:
                    out["differences"].append(f"{name}: {key} only in {'A' if key in za.files else 'B'}")
                elif not _arrays_equal(za[key], zb[key]):
                    out["differences"].append(f"{name}: {key} bits differ")
        elif name.endswith(".npy"):
            if not _arrays_equal(np.load(fa, allow_pickle=False), np.load(fb, allow_pickle=False)):
                out["differences"].append(f"{name}: bits differ")
        elif fa.read_bytes() != fb.read_bytes():
            out["differences"].append(f"{name}: bytes differ")
        if len(out["differences"]) == before:
            out["identical"].append(name)
    out["equal"] = not out["differences"]
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("run_a", type=Path)
    parser.add_argument("run_b", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    rep = compare(args.run_a, args.run_b)
    if args.json:
        args.json.write_text(json.dumps(rep, indent=1) + "\n")
    print(f"{'EQUAL' if rep['equal'] else 'DIFFERENT'}: {len(rep['identical'])} identical, "
          f"{len(rep['differences'])} differences, {len(rep['nondeterministic'])} nondeterministic, "
          f"{len(rep['provenance'])} provenance, {len(rep['skipped'])} skipped")
    for key in ("differences", "nondeterministic", "provenance"):
        for line in rep[key][:40]:
            print(f"  [{key}] {line}")
    return 0 if rep["equal"] else 1


if __name__ == "__main__":
    sys.exit(main())
