"""Left/right and expected-value check of a steering probe's model dump (docs/plans/2026-11-17-model-symmetry-check.md
3; thresholds fixed before the results).

    python compare_lr.py --dump steering_probe_model.json --urdf forklift.urdf --settings settings.yaml
        [--expect-carriage expect.json] [--out OUT.json]

Causes only, in the zero-pose geometry (independent of the stand's pose):
- engine mass properties per body against the expected value (the URDF; the carriage of an arm that merges a load
  against --expect-carriage, that arm's own record), and left against right in the bodies' own frames (the URDF's
  wheel and carrier frames are parallel, so the right body mirrored about its x-z plane must equal the left);
- tyre colliders: shape, axis, and centre in the wheel body's frame mirrored;
- joints: body0 frame position mirrored, rotations, axis, limits against the URDF, drive type/stiffness/damping/max
  force against the settings (scene.configure_drives), armature/friction/max velocity left against right;
- collider offsets against configure_drives' values, tyre materials left against right, solver iterations against the
  settings.
The stand's floor forces, wheel-centre heights and tilt are results: reported, never a defect.

Verdict: "defect" when any cause item crosses its threshold, else "undecided" when a required engine value is missing,
not finite or a query failed, else "no_mismatch_in_scope" (the cause is then left open, plan 3 (나)).
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
EQ_REL = 1e-6

REQUIRED_BODIES = ("base_link", "front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel",
                   "left_steering_carrier", "right_steering_carrier", "fork_carriage")
BODY_PAIRS = (("front_left_wheel", "front_right_wheel"), ("rear_left_wheel", "rear_right_wheel"),
              ("left_steering_carrier", "right_steering_carrier"))
JOINT_PAIRS = (("left_steer", "right_steer"), ("front_left_spin", "front_right_spin"),
               ("rear_left_spin", "rear_right_spin"))
MIRROR = np.diag([1.0, -1.0, 1.0])
CONTACT_OFFSET, REST_OFFSET = 0.002, 0.0  # scene.configure_drives


def urdf_model(path: Path) -> dict:
    root = ET.parse(path).getroot()
    inertials, limits = {}, {}
    for link in root.findall("link"):
        i = link.find("inertial")
        if i is None:
            continue
        o = i.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        t = i.find("inertia")
        inertia = None if t is None else [[float(t.get("ixx")), float(t.get("ixy")), float(t.get("ixz"))],
                                          [float(t.get("ixy")), float(t.get("iyy")), float(t.get("iyz"))],
                                          [float(t.get("ixz")), float(t.get("iyz")), float(t.get("izz"))]]
        inertials[link.get("name")] = {"mass_kg": float(i.find("mass").get("value")), "com_m": xyz, "inertia": inertia}
    for j in root.findall("joint"):
        lim = j.find("limit")
        if j.get("type") == "revolute" and lim is not None:
            limits[j.get("name")] = (math.degrees(float(lim.get("lower"))), math.degrees(float(lim.get("upper"))))
    return {"inertials": inertials, "limits": limits}


def expected_drives(settings: dict) -> dict:
    """What scene.configure_drives authors (the stage's units: the settings' numbers as written)."""
    return {"steer": {"type": "force", "stiffness": float(settings["steering_stiffness"]),
                      "damping": float(settings["steering_damping"]), "max_force": float(settings["steering_torque_nm"])},
            "spin": {"type": "force", "stiffness": 0.0, "damping": float(settings.get("wheel_drive_damping", 20.0)),
                     "max_force": float(settings["wheel_torque_nm"])},
            "solver": (int(settings.get("solver_position_iterations", 32)),
                       int(settings.get("solver_velocity_iterations", 8)))}


def _finite(*vals) -> bool:
    for v in vals:
        if v is None:
            return False
        if not np.all(np.isfinite(np.asarray(v, float))):
            return False
    return True


def _rel(a, b):
    return abs(a - b) / max(abs(a), abs(b), 1e-9)


def _inertia(v):
    return None if v is None else np.asarray(v, float).reshape(3, 3)


def check(dump: dict, urdf: dict, settings: dict, expect_carriage: dict | None = None) -> dict:
    items, undecided = [], []

    def add(name, value, limit, kind):
        items.append({"item": name, "value": value, "limit": limit, "kind": kind, "crosses": value > limit})

    def same(name, a, b, kind="pair"):
        if isinstance(a, (int, float)) and isinstance(b, (int, float)):
            ok = _rel(float(a), float(b)) <= EQ_REL
        else:
            ok = a == b
        add(name, 0.0 if ok else 1.0, 0.5, kind)

    eng = dump.get("engine") or {}
    for k in (eng.get("errors") or {}):
        undecided.append(f"engine query failed: {k}")
    bodies = eng.get("bodies") or {}
    for n in REQUIRED_BODIES:
        b = bodies.get(n)
        if b is None or not _finite(b.get("mass_kg"), b.get("com_m"), b.get("inertia")):
            undecided.append(f"engine value missing or not finite: {n}")
    # 1. engine mass properties against the expected value
    for n, b in bodies.items():
        want = expect_carriage if (n == "fork_carriage" and expect_carriage) else urdf["inertials"].get(n)
        if want is None or not _finite(b.get("mass_kg")):
            continue
        add(f"mass {n} vs expected", _rel(b["mass_kg"], want["mass_kg"]), MASS_REL, "expected")
        if _finite(b.get("com_m")) and want.get("com_m") is not None:
            add(f"com {n} vs expected (m)", float(np.linalg.norm(np.subtract(b["com_m"], want["com_m"]))), COM_M,
                "expected")
        if _finite(b.get("inertia")) and want.get("inertia") is not None:
            got, exp = _inertia(b["inertia"]), _inertia(want["inertia"])
            add(f"inertia {n} vs expected", float(np.abs(got - exp).max() / max(np.abs(exp).max(), 1e-12)),
                INERTIA_REL, "expected")
    # 2. left/right pairs in their own frames (mirror about the frame's x-z plane)
    for left, right in BODY_PAIRS:
        L, R = bodies.get(left), bodies.get(right)
        if not L or not R:
            continue
        if _finite(L.get("mass_kg"), R.get("mass_kg")):
            add(f"mass {left}~{right}", _rel(L["mass_kg"], R["mass_kg"]), MASS_REL, "pair")
        if _finite(L.get("com_m"), R.get("com_m")):
            add(f"com mirror {left}~{right} (m)",
                float(np.linalg.norm(np.asarray(L["com_m"]) - MIRROR @ np.asarray(R["com_m"]))), COM_M, "pair")
        if _finite(L.get("inertia"), R.get("inertia")):
            il, ir = _inertia(L["inertia"]), MIRROR @ _inertia(R["inertia"]) @ MIRROR
            add(f"inertia mirror {left}~{right}", float(np.abs(il - ir).max() / max(np.abs(il).max(), 1e-12)),
                INERTIA_REL, "pair")
    # 3. tyre colliders: one cylinder per wheel, compared in the wheel's own frame
    tyres = {}
    for path, c in (dump.get("colliders") or {}).items():
        for w in ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"):
            if f"/{w}/" in path + "/" and c.get("type") == "Cylinder":
                tyres.setdefault(w, []).append(c)
    for left, right in BODY_PAIRS[:2]:
        a, b = tyres.get(left, []), tyres.get(right, [])
        if len(a) != 1 or len(b) != 1:
            undecided.append(f"tyre colliders {left}/{right}: {len(a)}/{len(b)}")
            continue
        ca, cb = a[0], b[0]
        for f in ("radius", "height"):
            add(f"tyre {f} {left}~{right} (m)", abs(ca[f] - cb[f]), SHAPE_M, "pair")
        same(f"tyre axis {left}~{right}", ca.get("axis"), cb.get("axis"))
        if ca.get("in_body_center_m") is None or cb.get("in_body_center_m") is None:
            undecided.append(f"tyre centre in body {left}/{right}")
        else:
            add(f"tyre centre in body mirror {left}~{right} (m)",
                float(np.linalg.norm(np.asarray(ca["in_body_center_m"]) - MIRROR @ np.asarray(cb["in_body_center_m"]))),
                SHAPE_M, "pair")
        ma, mb = ca.get("material") or {}, cb.get("material") or {}
        same(f"tyre material {left}~{right}", (ma.get("path"), ma.get("static_friction"), ma.get("dynamic_friction")),
             (mb.get("path"), mb.get("static_friction"), mb.get("dynamic_friction")))
    for path, c in (dump.get("colliders") or {}).items():
        if not path.startswith("/World/Forklift"):
            continue
        for f, want in (("contact_offset", CONTACT_OFFSET), ("rest_offset", REST_OFFSET)):
            v = c.get(f)
            if v is None:
                undecided.append(f"{f} unread: {path}")
            elif abs(v - want) > 1e-9:
                add(f"{f} {path} vs expected", 1.0, 0.5, "expected")
    # 4. joints
    exp = expected_drives(settings)
    js = dump.get("joints") or {}
    for left, right in JOINT_PAIRS:
        ja, jb = js.get(left), js.get(right)
        if ja is None or jb is None:
            undecided.append(f"joint pair {left}/{right}: missing")
            continue
        kind = "steer" if "steer" in left else "spin"
        for j, name in ((ja, left), (jb, right)):
            d = j.get("drive") or {}
            for f in ("type", "stiffness", "damping", "max_force"):
                want, got = exp[kind][f], d.get(f)
                ok = got is not None and (got == want if f == "type" else _rel(float(got), want) <= EQ_REL)
                add(f"drive {f} {name} vs settings", 0.0 if ok else 1.0, 0.5, "expected")
            if name in urdf["limits"]:
                lo, hi = urdf["limits"][name]
                ok = j.get("lower_deg") is not None and abs(j["lower_deg"] - lo) < 1e-3 and abs(j["upper_deg"] - hi) < 1e-3
                add(f"limits {name} vs urdf", 0.0 if ok else 1.0, 0.5, "expected")
        for f in ("armature", "joint_friction", "max_joint_velocity", "axis", "local_rot0", "local_rot1"):
            if ja.get(f) is None and jb.get(f) is None:
                continue
            same(f"joint {f} {left}~{right}", ja.get(f), jb.get(f))
        for f in ("local_pos0", "local_pos1"):
            if ja.get(f) is not None and jb.get(f) is not None:
                add(f"joint {f} mirror {left}~{right} (m)",
                    float(np.linalg.norm(np.asarray(ja[f]) - MIRROR @ np.asarray(jb[f]))), SHAPE_M, "pair")
    art = dump.get("articulation") or {}
    got = (art.get("solver_position_iterations"), art.get("solver_velocity_iterations"))
    if None in got:
        undecided.append("solver iterations unread")
    else:
        add("solver iterations vs settings", 0.0 if tuple(int(v) for v in got) == exp["solver"] else 1.0, 0.5,
            "expected")
    crossing = [i for i in items if i["crosses"]]
    verdict = "defect" if crossing else ("undecided" if undecided else "no_mismatch_in_scope")
    return {"verdict": verdict, "crossing": crossing, "undecided": undecided, "items": items,
            "settled": dump.get("settled")}


def main(argv=None) -> int:
    import yaml

    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", type=Path, required=True)
    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--settings", type=Path, required=True)
    ap.add_argument("--expect-carriage", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    expc = json.loads(a.expect_carriage.read_text()) if a.expect_carriage else None
    out = check(json.loads(a.dump.read_text()), urdf_model(a.urdf), yaml.safe_load(a.settings.read_text()), expc)
    text = json.dumps(out, indent=1, default=float)
    if a.out:
        a.out.write_text(text + "\n")
    print(json.dumps({k: out[k] for k in ("verdict", "crossing", "undecided")}, indent=1, default=float))
    print("settled", json.dumps(out["settled"], default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
