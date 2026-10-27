"""Recompute a --bottom-deck-check on run's side decisions from its saved frames (plan 2026-10-19 task 4).

Reads <run>/bottom_deck/side_NN.npz (depth, intrinsics, mount, the judged target in the capture's base frame,
face, axis, shape, the judge's parameters and the decision) and <run>/result.json's ``bottom_deck`` block, judges
every capture again (the arrival check, then the side evidence) and the two-face policy over the recorded faces,
and compares them with the record. Exit 0 when every capture and the record's verdict match exactly.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from forklift_core.geometry import RigidTransform  # noqa: E402
from forklift_core.localization.slam_pose import compose  # noqa: E402
from forklift_core.perception import bottom_deck_evidence as E  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, spec_from_geometry  # noqa: E402
from forklift_core.perception.scene_dataset import SceneInput  # noqa: E402
from forklift_core.sensors.rgbd import PinholeIntrinsics  # noqa: E402


def scene_of(z) -> SceneInput:
    w, h, fx, fy, cx, cy = (float(v) for v in z["intrinsics"])
    k = PinholeIntrinsics(int(w), int(h), fx, fy, cx, cy, str(z["intrinsics_frame"]))
    mount = RigidTransform(str(z["intrinsics_frame"]), "base_link", tuple(map(tuple, z["mount_rotation"])),
                           tuple(float(v) for v in z["mount_translation"]))
    depth = np.asarray(z["depth_m"], dtype=float)
    return SceneInput(np.zeros((*depth.shape, 3), np.uint8), depth, k, mount, int(z["stamp_ns"]), "synthetic",
                      "synthetic")


def params_of(text: str) -> E.BottomDeckParams:
    return E.BottomDeckParams(**{k: tuple(v) if isinstance(v, list) else v for k, v in json.loads(text).items()})


def judge(z, spec) -> tuple[str, str]:
    scene = scene_of(z)
    x, y, yaw, support = (float(v) for v in z["target_base"])
    target = E.BottomDeckTarget((x, y), yaw, spec, support)
    face, axis = int(z["face"]), str(z["axis"])
    if not E.arrival_ok(scene, target, face=face, axis=axis):
        return "unobserved", "arrival"
    ev = E.evaluate_side(scene, target, face=face, params=params_of(str(z["params"])))
    return ev.verdict, ev.reason


def policy(faces: list, mode: str = "on") -> str:
    """The two-face verdict the mission writes (bottom_deck_permit.two_face_verdict): ``on`` -- bottom boards
    required; ``structure`` -- both faces present or absent (plan 2026-10-27-bottomless-and-bundled-pallets 2.1 g)."""
    from forklift_core.perception.bottom_deck_permit import two_face_verdict  # noqa: PLC0415

    return two_face_verdict(faces, mode)


def recompute(run: Path) -> dict:
    """Judge every saved capture again, rebuild each face's conclusion from the recomputed captures (the capture
    policy: at most three, the first present or absent ends the face; a face that ended otherwise -- blocked, no
    path -- is unobserved), the two-face policy over them, and compare all of it and the record's shape, axis and
    parameter digest with what the run wrote (Codex implementation round 1 P2: the record is checked against the
    frames, not against itself)."""
    from forklift_core.perception.bottom_deck_permit import params_digest  # noqa: PLC0415

    result = json.loads((run / "result.json").read_text())
    deck = result["bottom_deck"]
    # the run's mode; a record from before structure ran on
    mode = (result.get("arguments") or {}).get("bottom_deck_check", "on")
    record = deck.get("record") or {}
    specs: dict = {}
    rows, problems, per_face = [], [], {}
    carried = []  # (attempt, the judged target carried to the record's frame)
    releases = deck.get("releases", [])
    for n, a in enumerate(deck["attempts"], start=1):
        if "npz" not in a:  # a capture failure saves no frame: only an unobserved capture failure stands so
            got, saved = (a["verdict"], a["reason"]), None
            if a["verdict"] != "unobserved" or not str(a["reason"]).startswith("capture_failed"):
                problems.append(f"attempt {n}: {a['verdict']}/{a['reason']} without a saved frame")
        else:
            with np.load(run / a["npz"], allow_pickle=False) as z:
                shape, axis = str(z["shape"]), str(z["axis"])
                if shape not in specs:
                    specs[shape] = spec_from_geometry(
                        load_pallet_geometry(ROOT / f"config/pallet_geometry_{shape}.yaml"),
                        entry_axes=KNOWN_ENTRY_AXES[shape])
                got = judge(z, specs[shape])
                saved = (str(z["verdict"]), str(z["reason"]))
                if record and (shape, axis) != (record.get("shape"), record.get("axis")):
                    problems.append(f"{a['npz']}: shape/axis {shape}/{axis} differ from the record")
                if record and params_digest(params_of(str(z["params"]))) != record.get("params_sha256"):
                    problems.append(f"{a['npz']}: the judge's parameters differ from the record's digest")
                pose = tuple(float(v) for v in z["target_map"])
                for r in releases:  # releases after this capture that carried the target (PR #68 records did)
                    if r.get("carried", True) and r["after_attempts"] >= n:
                        pose = compose(tuple(r["frame_change"]), pose)
                carried.append((n, pose))
                if int(z["face"]) != int(a["face"]):
                    problems.append(f"{a['npz']}: face {int(z['face'])} differs from the attempt's {a['face']}")
        match = list(got) == [a["verdict"], a["reason"]] and (saved is None or saved == tuple(got))
        rows.append({"npz": a.get("npz"), "recorded": [a["verdict"], a["reason"]],
                     "saved": None if saved is None else list(saved), "recomputed": list(got), "match": match})
        per_face.setdefault(int(a["face"]), []).append(got[0])
    faces = []
    for f in deck.get("faces", []):
        caps = per_face.get(int(f["face"]), [])
        decided = [i for i, v in enumerate(caps) if v in ("present", "absent")]
        if decided and decided[0] != len(caps) - 1:
            problems.append(f"face {f['face']}: captured again after a decision")
        if len(caps) > 3:
            problems.append(f"face {f['face']}: {len(caps)} captures")
        verdict = caps[decided[0]] if decided else "unobserved"
        faces.append({"face": int(f["face"]), "verdict": verdict})
        if verdict != f["verdict"]:
            problems.append(f"face {f['face']}: recorded {f['verdict']}, recomputed {verdict}")
    verdict = policy(faces, mode)
    if record and record.get("centre_xy") is not None:
        want = (*record["centre_xy"], record["model_yaw_rad"])
        for n, pose in carried:  # the record binds the target that was judged (the insert gate reads it)
            d_yaw = (pose[2] - want[2] + np.pi) % (2 * np.pi) - np.pi
            if np.hypot(pose[0] - want[0], pose[1] - want[1]) > 1e-6 or abs(d_yaw) > 1e-6:
                problems.append(f"attempt {n}: the judged target {pose} is not the record's {want}")
    if record.get("verdict") != verdict:
        problems.append(f"record verdict {record.get('verdict')}, recomputed {verdict}")
    if record and [(int(f["face"]), f["verdict"]) for f in record.get("faces", [])] != [
            (f["face"], f["verdict"]) for f in faces]:
        problems.append("the record's faces differ from the recomputed ones")
    return {"captures": rows, "faces": faces, "mode": mode, "policy": verdict, "record_verdict": record.get("verdict"),
            "problems": problems, "match": all(r["match"] for r in rows) and not problems}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    a = ap.parse_args(argv)
    out = recompute(a.run)
    print(json.dumps(out, indent=2))
    return 0 if out["match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
