"""Rewrite the truth of a rendered recognition dataset after a truth-contract change, without re-rendering.

The truth is computed on the CPU rig (scene.json truth_render "rig") from the recipe and the camera K,
so the current recipe_record reproduces it. This tool writes a new run folder whose scene folders link
every input file of the source and carry a recomputed truth.json, and refuses unless the only change is
the one the contract change allows (plan docs/plans/2026-10-17-recognition-s7-side-entry.md):

- a scene that is not S7: the new truth is identical to the old one (the old file is linked);
- an S7 scene: per instance the spec is identical except entry_axes, damage, layer, support height and
  support visibility are identical, the old faces are exactly the new x faces, instruction_truth is
  identical; only entry_axes, the added y faces, eligible and lane_reachable may differ;
- every scene: the recomputed input instruction equals instruction.json, and across the noise levels
  the scene id, split, camera_info.json, tf.json, instruction.json and truth.json are identical.

Usage: python tools/retruth_recognition_dataset.py --source <run> --out <new run> [--levels clean low mid high]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "sim/isaac") not in sys.path:
    sys.path.insert(0, str(ROOT / "sim/isaac"))

SHARED_INPUTS = ("camera_info.json", "tf.json", "instruction.json", "truth.json")
S7_MAY_CHANGE = {"spec", "faces", "eligible", "lane_reachable"}


class RetruthError(ValueError):
    pass


def canon(obj):
    """JSON value as written and read back (tuples become lists, floats round-trip)."""
    return json.loads(json.dumps(obj))


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_unchanged(old: dict, new: dict, scene_id: str) -> None:
    if canon(new) != old:
        raise RetruthError(f"{scene_id}: truth changed for a scene the contract change does not touch")


def check_s7(old: dict, new: dict, scene_id: str) -> list:
    """The allowed S7 change; returns the changed fields per instance, raises on anything else."""
    old, new = canon(old), canon(new)
    if old.get("instruction_truth") != new.get("instruction_truth"):
        raise RetruthError(f"{scene_id}: instruction_truth changed")
    if len(old["instances"]) != len(new["instances"]):
        raise RetruthError(f"{scene_id}: instance count changed")
    changes = []
    for a, b in zip(old["instances"], new["instances"]):
        for key in set(a) | set(b):
            if key not in S7_MAY_CHANGE and a.get(key) != b.get(key):
                raise RetruthError(f"{scene_id}: instance {a.get('instance_id')} field {key} changed")
        sa, sb = dict(a["spec"]), dict(b["spec"])
        if sa.pop("entry_axes") != ["x"] or sb.pop("entry_axes") != ["x", "y"] or sa != sb:
            raise RetruthError(f"{scene_id}: instance {a['instance_id']} spec changed beyond entry_axes")
        if [f for f in b["faces"] if f["axis"] == "x"] != a["faces"]:
            raise RetruthError(f"{scene_id}: instance {a['instance_id']} x faces changed")
        if any(f["axis"] not in ("x", "y") for f in b["faces"]):
            raise RetruthError(f"{scene_id}: unexpected face axis")
        changes.append({"instance_id": a["instance_id"], "y_faces": sum(f["axis"] == "y" for f in b["faces"]),
                        "eligible": [a["eligible"], b["eligible"]],
                        "lane_reachable": [a["lane_reachable"], b["lane_reachable"]]})
    return changes


def check_levels(dirs: dict, scene_id: str) -> None:
    """The four noise levels of one scene share every input the truth depends on."""
    first = next(iter(dirs.values()))
    meta0 = json.loads((first / "scene.json").read_text())
    for level, d in dirs.items():
        meta = json.loads((d / "scene.json").read_text())
        if (meta["scene_id"], meta["split"]) != (meta0["scene_id"], meta0["split"]) or meta["scene_id"] != scene_id:
            raise RetruthError(f"{scene_id}: scene id or split differs at level {level}")
        for name in SHARED_INPUTS:
            if (d / name).read_bytes() != (first / name).read_bytes():
                raise RetruthError(f"{scene_id}: {name} differs at level {level}")


def _intrinsics(camera_info: dict):
    import perception_camera as pc

    from forklift_core.sensors.rgbd import PinholeIntrinsics

    k = camera_info["k"]
    # The renderer built K from the SDK matrix (principal point 320, 240) through the integer-index
    # convention; camera_info.json stores the result (319.5, 239.5).
    return pc._isaac_intrinsics_type()(
        PinholeIntrinsics(int(camera_info["width"]), int(camera_info["height"]), k[0], k[4], k[2] + 0.5, k[5] + 0.5,
                          "camera_optical_frame")).integer_index


def retruth(source: Path, out: Path, levels, *, revision: str = "") -> dict:
    from tools import recognition_scenes as rs
    from tools.generate_recognition_scenes import load_site, recipe_record

    site = load_site()
    catalogue = {r.scene_id: r for r in rs.catalogue()}
    roots = {lv: source / f"isaac_{lv}" for lv in levels}
    scene_ids = sorted(p.name for p in roots[levels[0]].iterdir() if p.is_dir())
    manifest = {"source": str(source), "levels": list(levels), "revision": revision,
                "rule": "S7 entry axes x and y; y faces judged with the face-relative load", "scenes": {}}
    counts = {"unchanged": 0, "s7_rewritten": 0, "eligible_changed": 0}
    for sid in scene_ids:
        dirs = {lv: roots[lv] / sid for lv in levels}
        check_levels(dirs, sid)
        first = dirs[levels[0]]
        old = json.loads((first / "truth.json").read_text())
        record = recipe_record(catalogue[sid], site,
                               intrinsics=_intrinsics(json.loads((first / "camera_info.json").read_text())))
        if canon(record.instruction) != json.loads((first / "instruction.json").read_text()):
            raise RetruthError(f"{sid}: recomputed instruction differs from instruction.json")
        meta = json.loads((first / "scene.json").read_text())
        if meta["case"] == "S7":
            changes = check_s7(old, record.truth, sid)
            text = json.dumps(record.truth, indent=1) + "\n"
            counts["s7_rewritten"] += 1
            counts["eligible_changed"] += sum(c["eligible"][0] != c["eligible"][1] for c in changes)
        else:
            check_unchanged(old, record.truth, sid)
            changes, text = None, None
            counts["unchanged"] += 1
        for lv, d in dirs.items():
            target = out / f"isaac_{lv}" / sid
            target.mkdir(parents=True, exist_ok=True)
            for f in d.iterdir():
                if f.name == "truth.json" and text is not None:
                    continue
                link = target / f.name
                if not link.exists():
                    link.symlink_to(f.resolve())
            if text is not None:
                (target / "truth.json").write_text(text)
        entry = {"split": meta["split"], "case": meta["case"], "before_sha256": _sha(first / "truth.json")}
        if text is not None:
            entry.update(after_sha256=hashlib.sha256(text.encode()).hexdigest(), changes=changes)
        manifest["scenes"][sid] = entry
    manifest["counts"] = counts
    (out / "truth_revision.json").write_text(json.dumps(manifest, indent=1) + "\n")
    return manifest


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--levels", nargs="+", default=["clean", "low", "mid", "high"])
    a = ap.parse_args(argv)
    if a.out.exists() and any(a.out.iterdir()):
        raise SystemExit(f"{a.out} is not empty")
    try:
        revision = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True, text=True,
                                  check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        revision = ""
    m = retruth(a.source, a.out, a.levels, revision=revision)
    print(json.dumps(m["counts"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
