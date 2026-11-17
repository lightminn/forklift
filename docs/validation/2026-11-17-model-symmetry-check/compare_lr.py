"""Left/right and expected-value check of a steering probe's model dump (docs/plans/2026-11-17-model-symmetry-check.md
3; thresholds fixed before the results).

    python compare_lr.py --dump steering_probe_model.json --urdf forklift.urdf [--expect-carriage expect.json]
        [--out OUT.json]

Pairs are made by the physical side (the sign of y in the base frame at the dump), not by names. Causes (the engine's
masses, centres of mass and inertias; the colliders' shapes and zero-pose positions; joints' frames, limits, drives,
armature, friction; collider offsets) are compared left against right and against the expected value (the URDF; the
carriage of an arm that merges a load is compared against --expect-carriage, the arm's own record). Results (the
stand's floor forces, wheel-centre heights, tilt) are reported, never classified as a defect.

Verdict: "defect" when any cause item crosses its threshold, "undecided" when an engine query failed, else
"no_mismatch_in_scope" (the cause is then left open, plan 3 (나)).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

MASS_REL = 0.01
COM_M = 0.001
INERTIA_REL = 0.01
SHAPE_M = 0.0005
FLOAT_EQ = 1e-6

PAIRS = (("front_left_wheel", "front_right_wheel"), ("rear_left_wheel", "rear_right_wheel"),
         ("left_steering_carrier", "right_steering_carrier"))
JOINT_PAIRS = (("left_steer", "right_steer"), ("front_left_spin", "front_right_spin"),
               ("rear_left_spin", "rear_right_spin"))
JOINT_FIELDS = ("lower_deg", "upper_deg", "armature", "joint_friction", "max_joint_velocity")
DRIVE_FIELDS = ("stiffness", "damping", "max_force")


def quat_matrix(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def urdf_inertials(path: Path) -> dict:
    out = {}
    for link in ET.parse(path).getroot().findall("link"):
        i = link.find("inertial")
        if i is None:
            continue
        o = i.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        t = i.find("inertia")
        inertia = None if t is None else [[float(t.get("ixx")), float(t.get("ixy")), float(t.get("ixz"))],
                                          [float(t.get("ixy")), float(t.get("iyy")), float(t.get("iyz"))],
                                          [float(t.get("ixz")), float(t.get("iyz")), float(t.get("izz"))]]
        out[link.get("name")] = {"mass_kg": float(i.find("mass").get("value")), "com_m": xyz, "inertia": inertia}
    return out


def _rel(a, b):
    return abs(a - b) / max(abs(a), abs(b), 1e-9)


def _side(y: float) -> str:
    return "left" if y > 0 else "right"


def base_frame(dump: dict) -> tuple:
    p = np.asarray(dump["body_poses"]["base_link"]["position_m"], float)
    r = quat_matrix(dump["body_poses"]["base_link"]["quat_wxyz"])
    return p, r


def check(dump: dict, urdf: dict, expect_carriage: dict | None = None) -> dict:
    items, undecided = [], []
    eng = dump["engine"]
    if eng.get("errors"):
        undecided += [f"engine:{k}" for k in eng["errors"]]
    bodies = eng.get("bodies", {})
    p0, r0 = base_frame(dump)

    def add(name, value, limit, kind):
        items.append({"item": name, "value": value, "limit": limit, "kind": kind,
                      "crosses": value is not None and value > limit})

    # 1. engine mass properties against the expected value (URDF, or the arm's merged carriage)
    for n, b in bodies.items():
        want = expect_carriage if (n == "fork_carriage" and expect_carriage) else urdf.get(n)
        if want is None or b.get("mass_kg") is None:
            continue
        add(f"mass {n} vs expected", _rel(b["mass_kg"], want["mass_kg"]), MASS_REL, "expected")
        if b.get("com_m") is not None:
            add(f"com {n} vs expected (m)", float(np.linalg.norm(np.subtract(b["com_m"], want["com_m"]))), COM_M,
                "expected")
        if b.get("inertia") is not None and want.get("inertia") is not None:
            got, exp = np.asarray(b["inertia"], float).reshape(3, 3), np.asarray(want["inertia"], float)
            add(f"inertia {n} vs expected", float(np.abs(got - exp).max() / max(np.abs(exp).max(), 1e-12)),
                INERTIA_REL, "expected")
    # 2. left/right pairs by physical side: engine mass, COM mirrored in the base frame, inertia mirrored
    poses = dump["body_poses"]
    mirror = np.diag([1.0, -1.0, 1.0])
    for a, b in PAIRS:
        if a not in bodies or b not in bodies or a not in poses or b not in poses:
            continue
        ya = (r0.T @ (np.asarray(poses[a]["position_m"]) - p0))[1]
        yb = (r0.T @ (np.asarray(poses[b]["position_m"]) - p0))[1]
        if _side(ya) == _side(yb):
            undecided.append(f"pair {a}/{b}: same side")
            continue
        left, right = (a, b) if ya > 0 else (b, a)
        L, R = bodies[left], bodies[right]
        if L.get("mass_kg") is not None and R.get("mass_kg") is not None:
            add(f"mass {left}~{right}", _rel(L["mass_kg"], R["mass_kg"]), MASS_REL, "pair")

        def com_base(name):
            ra = quat_matrix(poses[name]["quat_wxyz"])
            return r0.T @ (np.asarray(poses[name]["position_m"]) + ra @ np.asarray(bodies[name]["com_m"]) - p0)

        if L.get("com_m") is not None and R.get("com_m") is not None:
            add(f"com mirror {left}~{right} (m)", float(np.linalg.norm(com_base(left) - mirror @ com_base(right))),
                COM_M, "pair")
        if L.get("inertia") is not None and R.get("inertia") is not None:
            def in_base(name):
                rb = r0.T @ quat_matrix(poses[name]["quat_wxyz"])
                return rb @ np.asarray(bodies[name]["inertia"], float).reshape(3, 3) @ rb.T

            il, ir = in_base(left), mirror @ in_base(right) @ mirror
            add(f"inertia mirror {left}~{right}", float(np.abs(il - ir).max() / max(np.abs(il).max(), 1e-12)),
                INERTIA_REL, "pair")
    # composite COM in the base frame
    tot, mc = 0.0, np.zeros(3)
    for n, b in bodies.items():
        if b.get("mass_kg") is None or b.get("com_m") is None or n not in poses:
            continue
        c = r0.T @ (np.asarray(poses[n]["position_m"]) + quat_matrix(poses[n]["quat_wxyz"]) @ np.asarray(b["com_m"]) - p0)
        tot += b["mass_kg"]
        mc += b["mass_kg"] * c
    if tot > 0:
        add("composite com |y| (m)", abs(float(mc[1] / tot)), COM_M, "pair")
    # 3. tyre colliders: shape and zero-pose position mirrored (world, about the base)
    tyres = {k: v for k, v in dump["colliders"].items() if "_wheel" in k and v.get("type") == "Cylinder"}
    by_wheel = {}
    for path, c in tyres.items():
        wheel = next((w for pair in PAIRS[:2] for w in pair if f"/{w}/" in path), None)
        if wheel:
            by_wheel.setdefault(wheel, []).append(c)
    for a, b in PAIRS[:2]:
        if a in by_wheel and b in by_wheel and len(by_wheel[a]) == len(by_wheel[b]) == 1:
            ca, cb = by_wheel[a][0], by_wheel[b][0]
            for f in ("radius", "height"):
                add(f"tyre {f} {a}~{b} (m)", abs(ca[f] - cb[f]), SHAPE_M, "pair")
            add(f"tyre axis {a}~{b}", 0.0 if ca["axis"] == cb["axis"] else 1.0, 0.5, "pair")
            pa = r0.T @ (np.asarray(ca["world_center_m"]) - p0)
            pb = r0.T @ (np.asarray(cb["world_center_m"]) - p0)
            add(f"tyre centre mirror {a}~{b} (m)", float(np.linalg.norm(pa - mirror @ pb)), SHAPE_M, "pair")
            for f in ("contact_offset", "rest_offset"):
                va, vb = ca.get(f), cb.get(f)
                if va is not None or vb is not None:
                    add(f"tyre {f} {a}~{b}", 0.0 if va == vb else 1.0, 0.5, "pair")
        else:
            undecided.append(f"tyre colliders {a}/{b}: {len(by_wheel.get(a, []))}/{len(by_wheel.get(b, []))}")
    # 4. joints: limits, drives, armature, friction, frames mirrored
    js = dump["joints"]
    for a, b in JOINT_PAIRS:
        if a not in js or b not in js:
            undecided.append(f"joint pair {a}/{b}: missing")
            continue
        ja, jb = js[a], js[b]
        for f in JOINT_FIELDS:
            va, vb = ja.get(f), jb.get(f)
            if va is None and vb is None:
                continue
            add(f"joint {f} {a}~{b}", 0.0 if (va is not None and vb is not None and abs(va - vb) <= FLOAT_EQ * max(1, abs(va)))
                else 1.0, 0.5, "pair")
        da, db = ja.get("drive") or {}, jb.get("drive") or {}
        for f in DRIVE_FIELDS:
            va, vb = da.get(f), db.get(f)
            if va is None and vb is None:
                continue
            add(f"drive {f} {a}~{b}", 0.0 if (va is not None and vb is not None and _rel(va, vb) <= FLOAT_EQ) else 1.0,
                0.5, "pair")
        for f in ("local_pos0", "local_pos1"):
            va, vb = ja.get(f), jb.get(f)
            if va is not None and vb is not None:
                add(f"joint {f} mirror {a}~{b} (m)", float(np.linalg.norm(np.asarray(va) - mirror @ np.asarray(vb))),
                    SHAPE_M, "pair")
    crossing = [i for i in items if i["crosses"]]
    verdict = "defect" if crossing else ("undecided" if undecided else "no_mismatch_in_scope")
    return {"verdict": verdict, "crossing": crossing, "undecided": undecided, "items": items,
            "settled": dump.get("settled")}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", type=Path, required=True)
    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--expect-carriage", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    exp = json.loads(a.expect_carriage.read_text()) if a.expect_carriage else None
    out = check(json.loads(a.dump.read_text()), urdf_inertials(a.urdf), exp)
    text = json.dumps(out, indent=1, default=float)
    if a.out:
        a.out.write_text(text + "\n")
    print(json.dumps({k: out[k] for k in ("verdict", "crossing", "undecided")}, indent=1, default=float))
    print("settled", json.dumps(out["settled"], default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
