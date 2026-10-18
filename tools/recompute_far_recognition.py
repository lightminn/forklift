"""Recompute a --recognition far run's far decisions from its saved frames (plan v2, 6).

Reads <run>/recognition/frame_*.npz (driving and stop frames: depth, instance ids, stamp, odom, the map
correction or the control pose, intrinsics and mount) and <run>/result.json's ``recognition`` block (zone,
fork spacing, pallet depth), replays every leg through recognition_bridge.FarRecognition and compares each
stop's status, reason, source, selected track and used candidate with the recorded attempt. Exit 0 when
every stop matches exactly.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from forklift_core.geometry import RigidTransform  # noqa: E402
from forklift_core.perception import recognition_bridge as B  # noqa: E402
from forklift_core.perception.scene_dataset import SceneInput  # noqa: E402
from forklift_core.sensors.rgbd import PinholeIntrinsics  # noqa: E402


def _runtime_module():
    spec = importlib.util.spec_from_file_location("far_recognition_runtime", ROOT / "sim/isaac/far_recognition_runtime.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _scene(z) -> SceneInput:
    w, h, fx, fy, cx, cy = (float(v) for v in z["intrinsics"])
    k = PinholeIntrinsics(int(w), int(h), fx, fy, cx, cy, str(z["intrinsics_frame"]))
    mount = RigidTransform(str(z["intrinsics_frame"]), "base_link", tuple(map(tuple, z["mount_rotation"])),
                           tuple(float(v) for v in z["mount_translation"]))
    depth = np.asarray(z["depth_m"], dtype=float)
    return SceneInput(np.zeros((*depth.shape, 3), np.uint8), depth, k, mount, int(z["stamp_ns"]), "synthetic",
                      "synthetic")


def recompute(run_dir: Path, *, recognition: dict | None = None) -> list:
    """[(recorded attempt, recomputed record)] per stop, in order."""
    R = _runtime_module()
    rec = recognition if recognition is not None else json.loads((run_dir / "result.json").read_text())["recognition"]
    runtime = R.FarRuntime(fork_spacing=rec["fork_spacing_m"], pallet_depth_m=rec["pallet_depth_m"],
                           pallet_shape=rec["pallet_shape"], zone_centre_xy=rec["zone"]["centre_xy"],
                           zone_radius_m=rec["zone"]["radius_m"], camera_x_m=rec["camera_x_m"], dt=1 / 120)
    far = runtime.far
    far.start_leg()
    out, attempts = [], iter(rec["attempts"])
    for path in sorted((run_dir / "recognition").glob("frame_*.npz")):
        z = np.load(path)
        scene, ids, odom = _scene(z), np.asarray(z["instance_ids"]), tuple(float(v) for v in z["odom"])
        if str(z["kind"]) == "drive":
            far.frame(scene, ids, R.planar(*odom), runtime.instruction(R.compose(tuple(z["map_from_odom"]), odom)))
            continue
        _, again = far.stop(scene, ids, R.planar(*odom), runtime.instruction(tuple(z["control_base"])),
                            pallet_depth_m=runtime.pallet_depth_m)
        out.append((next(attempts), again))
        far.start_leg()
    return out


def differences(recorded: dict, again: dict) -> list:
    keys = ("status", "reason", "source", "selected_track", "observations")
    diffs = [k for k in keys if recorded.get(k) != again.get(k)]
    if (recorded.get("used") or {}) != (again.get("used") or {}):
        diffs.append("used")
    return diffs


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir", type=Path)
    args = ap.parse_args(argv)
    bad = 0
    for i, (recorded, again) in enumerate(recompute(args.run_dir)):
        diffs = differences(recorded, again)
        bad += bool(diffs)
        print(f"stop {i}: {'same' if not diffs else 'DIFFERS ' + ','.join(diffs)}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
