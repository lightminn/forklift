"""Write a boundary record's non-code setup inputs back onto a copy of the gate snapshot (the replay gate's
SETUP_INPUTS=recorded step, deploy/slurm/replay_gate.sbatch).

The replay refuses a record whose setup read a file that has changed since (``boundary_replay.build_inputs``: the
setup snapshot keeps each file's path and sha256). A record reads its code inside its own recording snapshot (absolute
paths) but its configuration by relative paths, so a tree that changes a setup file -- #254's pallet masses -- makes
every record unreplayable on it. This step takes each relative setup input whose bytes differ in the gate copy, finds
the recorded bytes in the record's recording snapshot (the one directory its absolute code entries sit in), checks
their sha256 and writes them into the copy. The gate then checks the code against the inputs the run was recorded on.

It writes only into a directory holding the copy marker (the gate script makes a private copy per record), only data
files (``DATA_SUFFIXES``), and only relative paths inside the copy; anything else that differs is refused before a
byte is written, and the replay's own check reports it as before. Absolute inputs (artifacts outside the snapshot)
are left to the replay.

    python tools/gate_setup_overlay.py RUN_DIR COPY_DIR [--json OUT]   # exit 0 written/nothing to do, 2 refused
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path, PurePosixPath

MARKER = ".gate_setup_copy"
# Files the setup reads as data. A path with any other suffix (.py, .pyc, .so, .sh, none) is code or unknown and is
# never written.
DATA_SUFFIXES = frozenset({".yaml", ".yml", ".json", ".urdf", ".xml", ".usd", ".usda", ".usdc", ".csv", ".txt",
                           ".npz", ".npy", ".stl", ".obj", ".dae", ".png"})


class OverlayRefused(Exception):
    pass


def _sha(path: Path) -> str | None:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def recording_snapshot(entries: list[dict]) -> Path:
    """The one directory the record's absolute in-repository entries sit in (path = ROOT/repo_relative)."""
    roots = set()
    for item in entries:
        path, rel = item.get("path") or "", item.get("repo_relative")
        if rel and PurePosixPath(path).is_absolute() and path.endswith("/" + rel):
            roots.add(path[: -len(rel) - 1])
    if len(roots) != 1:
        raise OverlayRefused(f"the record names {len(roots)} recording snapshot directories: {sorted(roots)[:3]}")
    return Path(roots.pop())


def overlay(run_dir: Path, copy_dir: Path) -> dict:
    if not (copy_dir / MARKER).is_file():
        raise OverlayRefused(f"{copy_dir} has no {MARKER}: only a gate copy is written")
    entries = json.loads((run_dir / "boundary/setup_snapshot.json").read_text()).get("files", [])
    root = recording_snapshot(entries)
    plan, unchanged, seen = [], 0, set()
    for item in entries:
        rel = item["path"]
        if PurePosixPath(rel).is_absolute():
            continue  # inside the recording snapshot (code) or outside any snapshot: the replay checks it
        if rel in seen:
            continue
        seen.add(rel)
        parts = PurePosixPath(rel).parts
        if ".." in parts:
            raise OverlayRefused(f"{rel} leaves the copy")
        target = copy_dir / rel
        now = _sha(target)
        if now == item["sha256"]:
            unchanged += 1
            continue
        if PurePosixPath(rel).suffix not in DATA_SUFFIXES:
            raise OverlayRefused(f"{rel} differs and is not a setup data file (only {sorted(DATA_SUFFIXES)})")
        source = root / rel
        if _sha(source) != item["sha256"]:
            raise OverlayRefused(f"the recording snapshot {root} no longer holds the recorded {rel}")
        plan.append({"path": rel, "recorded_sha256": item["sha256"], "gate_sha256": now, "source": str(source)})
    for w in plan:  # every check passed: only now is anything written
        target = copy_dir / w["path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(Path(w["source"]).read_bytes())
    return {"run": str(run_dir), "copy": str(copy_dir), "recording_snapshot": str(root), "written": plan,
            "unchanged": unchanged}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("copy_dir", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    try:
        out, code = overlay(args.run_dir, args.copy_dir), 0
    except OverlayRefused as exc:
        out, code = {"run": str(args.run_dir), "copy": str(args.copy_dir), "refused": str(exc)}, 2
    text = json.dumps(out, indent=1)
    if args.json:
        args.json.write_text(text + "\n")
    print(text)
    return code


if __name__ == "__main__":
    sys.exit(main())
