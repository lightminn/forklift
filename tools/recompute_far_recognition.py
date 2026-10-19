"""Recompute a --recognition far run's far decisions from its saved frames (plan v2, 6).

Reads <run>/recognition/frame_*.npz (driving and stop frames: depth, instance ids, stamp, odom, the map
correction or the control pose, intrinsics and mount) and <run>/result.json's ``recognition`` block (zone,
fork spacing, pallet depth), replays every leg through recognition_bridge.FarRecognition and compares each
stop's status, reason, source, selected track and used candidate with the recorded attempt. Exit 0 when
every stop matches exactly.

A learned run (stage 3, ``--recognition-proposals learned``) also saved each frame's RGB and the proposals the
decision used (the model's, or the geometric fallback's with its failures); the replay feeds those back with the
run's tau, so the decision is recomputed exactly whatever device ran the model. ``--onnx`` additionally reruns
the model on the saved RGB and compares its proposals (count, score within 1e-3, mask IoU >= 0.99).
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
    rgb = np.asarray(z["rgb"], np.uint8) if "rgb" in z.files else np.zeros((*depth.shape, 3), np.uint8)
    return SceneInput(rgb, depth, k, mount, int(z["stamp_ns"]), "synthetic", "synthetic")


def saved_proposals(z):
    """A learned frame's recorded proposals and proposer failures."""
    from forklift_core.perception.instance_fit import FitFailure  # noqa: PLC0415
    from forklift_core.perception.instance_proposals import InstanceProposal  # noqa: PLC0415

    props = [InstanceProposal(str(i), np.asarray(m, bool), float(sc), str(src)) for m, sc, i, src in
             zip(z["proposal_masks"], z["proposal_scores"], z["proposal_ids"], z["proposal_sources"])]
    failures = tuple(FitFailure(f[0], f[1], tuple(f[2]) if f[2] is not None else None, float(f[3]))
                     for f in json.loads(str(z["extra_failures"])))
    return props, failures


class _Replay:
    """FarRecognition's proposer for a learned run's replay: the current frame's recorded proposals, or, with a
    ``model``, that model rerun on the frame's saved RGB (frames that fell back keep their recorded geometric
    proposals -- the failure is not reproducible). The rerun recomputes the whole decision, so a score that crosses
    tau shows as a decision difference (Codex review 1), not only as a numeric one."""

    def __init__(self, model=None):
        self.frame, self.model = None, model

    def __call__(self, scene, ids):
        if self.model is not None and str(self.frame["proposer_source"]) == "learned":
            return list(self.model.propose(scene)), ()
        return saved_proposals(self.frame)


def recompute(run_dir: Path, *, recognition: dict | None = None, model=None) -> list:
    """[(recorded attempt, recomputed record)] per stop, in order. ``model``: a learned run's model, rerun."""
    R = _runtime_module()
    rec = recognition if recognition is not None else json.loads((run_dir / "result.json").read_text())["recognition"]
    proposer = rec.get("proposer") or {"kind": "oracle"}
    if model is not None and proposer["kind"] != "learned":
        raise ValueError("a model rerun needs a learned run")
    replay = _Replay(model) if proposer["kind"] == "learned" else None
    runtime = R.FarRuntime(fork_spacing=rec["fork_spacing_m"], pallet_depth_m=rec["pallet_depth_m"],
                           pallet_shape=rec["pallet_shape"], zone_centre_xy=rec["zone"]["centre_xy"],
                           zone_radius_m=rec["zone"]["radius_m"], camera_x_m=rec["camera_x_m"], dt=1 / 120,
                           proposer=replay, identity_threshold=proposer.get("tau"))
    far = runtime.far
    far.start_leg()
    out, attempts, leg = [], iter(rec["attempts"]), 0
    for path in sorted((run_dir / "recognition").glob("frame_*.npz")):
        z = np.load(path)
        scene, ids, odom = _scene(z), np.asarray(z["instance_ids"]), tuple(float(v) for v in z["odom"])
        if replay is not None:
            if "proposal_masks" not in z.files:
                out.append(({"frame": path.name}, {"status": "learned_frame_without_proposals"}))
                continue
            replay.frame = z
        if int(z["leg"]) != leg:  # the recorded leg boundaries must be the ones this replay makes
            out.append(({"leg": int(z["leg"]), "frame": path.name}, {"leg": leg, "status": "leg_boundary_differs"}))
        if str(z["kind"]) == "drive":
            again = far.frame(scene, ids, R.planar(*odom),
                              runtime.instruction(R.compose(tuple(z["map_from_odom"]), odom)))
            if again["leg_lost"]:  # A': the leg ends here, the next frame starts a re-observation leg
                far.start_leg()
                leg += 1
            continue
        _, again = far.stop(scene, ids, R.planar(*odom), runtime.instruction(tuple(z["control_base"])),
                            pallet_depth_m=runtime.pallet_depth_m)
        out.append((next(attempts, {"status": "unrecorded_stop"}), again))
        far.start_leg()
        leg += 1
    # every recorded frame replayed and every recorded stop compared exactly once (Codex: a missing frame or
    # directory must not pass as "same")
    expected = sum(1 for f in rec["frames"] if f.get("kind") == "stop"
                   or (f.get("kind") == "drive" and "skipped" not in f))
    found = len(list((run_dir / "recognition").glob("frame_*.npz")))
    if found != expected:
        out.append(({"frames": expected}, {"frames": found, "status": "saved_frames_missing"}))
    for left in attempts:
        out.append((left, {"status": "stop_not_replayed"}))
    return out


def model_check(run_dir: Path, model, *, score_tol: float = 1e-3, min_iou: float = 0.99) -> list:
    """[(frame, problem)] where rerunning ``model`` on a learned frame's saved RGB does not give its recorded
    proposals (frames that fell back to the geometric proposer are skipped)."""
    problems = []
    for path in sorted((run_dir / "recognition").glob("frame_*.npz")):
        z = np.load(path)
        if "proposer_source" not in z.files or str(z["proposer_source"]) != "learned":
            continue
        want, _ = saved_proposals(z)
        got = list(model.propose(_scene(z)))
        if len(got) != len(want):
            problems.append((path.name, f"count {len(got)} != {len(want)}"))
            continue
        for g, w in zip(got, want):
            iou = (g.mask & w.mask).sum() / max(1, (g.mask | w.mask).sum())
            if abs(g.score - w.score) > score_tol or iou < min_iou:
                problems.append((path.name, f"{w.proposal_id}: score {g.score:.4f}/{w.score:.4f} iou {iou:.3f}"))
    return problems


def differences(recorded: dict, again: dict) -> list:
    keys = ("status", "reason", "source", "selected_track", "observations")
    diffs = [k for k in keys if recorded.get(k) != again.get(k)]
    if (recorded.get("used") or {}) != (again.get("used") or {}):
        diffs.append("used")
    return diffs


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--onnx", type=Path, default=None,
                    help="a learned run: also rerun this model on the saved RGB (the [learned] extra)")
    args = ap.parse_args(argv)
    bad = 0
    for i, (recorded, again) in enumerate(recompute(args.run_dir)):
        diffs = differences(recorded, again)
        bad += bool(diffs)
        print(f"stop {i}: {'same' if not diffs else 'DIFFERS ' + ','.join(diffs)}")
    if args.onnx is not None:
        from forklift_learned.proposer import LearnedProposer  # noqa: PLC0415

        rec = json.loads((args.run_dir / "result.json").read_text())["recognition"]["proposer"]
        model = LearnedProposer(args.onnx, conf=rec.get("conf", 0.05))
        problems = model_check(args.run_dir, model)
        for name, what in problems:
            print(f"model {name}: {what}")
        print(f"model check: {'same' if not problems else f'{len(problems)} differ'}")
        bad += bool(problems)
        for i, (recorded, again) in enumerate(recompute(args.run_dir, model=model)):
            diffs = differences(recorded, again)
            bad += bool(diffs)
            print(f"stop {i} (model rerun): {'same' if not diffs else 'DIFFERS ' + ','.join(diffs)}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
