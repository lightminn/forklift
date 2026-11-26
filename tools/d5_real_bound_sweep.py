"""The real D5's passable pallet-truck relative error, on synthetic depth (PR 7 7e-2a, plan 6.2 / 9.2).

The real path treats a voxel inside the error band as visible (refutation catches what is stuck to a surface), so
even an aligned empty pocket stops near the insertion end once the band is wide. This drives the measured chassis
(``sim/models/dls08_measured``) down the docking straight of an EPAL 6 with the real generator
(``real_depth.make_pocket_check``) at the D435i's 640x480 (fx 388.8, the team capture) and 15 Hz, for each setting
of ``pallet_rel_error_m``:

* ``aligned``: the pallet where the estimate puts it, no noise and the team noise shape (``frame_640`` correlated
  components, 5 % dropouts, a few +-4 sigma tails);
* ``offset``: the real pallet moved by the setting along, across and in yaw (a corner by the setting) -- an empty
  pocket and a 3 cm bar in the left pocket (the stop must come before the blade reaches it).

A synthetic, provisional figure for this shape, mount (Isaac's ``carriage_low_measured``: the D435i mount is
unmeasured, B6) and path: S10 needs the error injection on the real truck first (plan 6.2). Prints one JSON line
per run and, with ``--out``, writes them all.

    python tools/d5_real_bound_sweep.py --errors 0,5,10,15,20 --out sweep.json [--jobs 8]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"
NOISE = ROOT / "config/depth_noise_d435i_team.yaml"
FX = 388.81207275390625
INTR = {"serial": "synthetic", "firmware": "-", "stream": "depth", "width": 640, "height": 480, "fx": FX, "fy": FX,
        "ppx": 320.0, "ppy": 240.0, "model": "brown_conrady", "coeffs": [0.0] * 5}
MOUNT = ((0.619, 0.0, 0.27), 0.10)  # Isaac's carriage_low_measured (perception_adapter.PERCEPTION_MOUNTS)
PALLET = {"kind": "epal6", "depth_m": 0.6, "width_m": 0.8}
HZ = 15.0
QUERY_S = 0.035  # the query after the frame (the send margin, plan 9.2)
STANDOFF_M = 2.2  # fork tips before the face at the first frame (Isaac's stand-off)
CRUISE, INSERT, INSERT_LEN = 0.15, 0.055, 0.46  # S10's cap; the insertion speed over its last 0.46 m


def base_from_optical(xyz=MOUNT[0], tilt=MOUNT[1]):
    c, s = math.cos(tilt), math.sin(tilt)
    z = np.array([c, 0.0, -s])
    x = np.array([0.0, -1.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z]), np.array(xyz, dtype=float)


def budget(error_m: float, cam: str) -> dict:
    cam_t, cam_r = (0.0, 0.0) if cam == "zero" else (0.005, 0.5)
    return {"sigma_a": 0.0058, "surface_estimate_m": 0.02, "lifetime_s": 20.0, "view_min_px": 2000,
            "view_valid_min": 0.5,
            "error": {"operator_m": error_m, "slam_m": 0.0, "own_m": 0.002, "cam_trans_m": cam_t, "cam_rot_deg": cam_r}}


def make_check(error_m: float, cam: str):
    from forklift_core.control.drive_permission import StoppingModel
    from forklift_core.live.real_depth import make_pocket_check

    return make_pocket_check(estimate=(0.0, 0.0, 0.0), axis_yaw=0.0, pallet=PALLET, urdf=URDF, intrinsics=INTR,
                             base_from_optical=base_from_optical(), stopping=StoppingModel(0.30, 0.5, 0.05),
                             pocket=budget(error_m, cam), distortion_max_px=0.5, geometry_dir=ROOT / "config")


def true_solids(check, dx: float, dy: float, dyaw: float):
    """The real pallet's boxes: the estimate's moved by (dx, dy, dyaw) about the pallet centre."""
    from forklift_core.perception.insertion_clearance import Box

    c, s = math.cos(dyaw), math.sin(dyaw)
    return [Box((b.center[0] * c - b.center[1] * s + dx, b.center[0] * s + b.center[1] * c + dy, b.center[2]),
                b.half, b.yaw + dyaw) for b in check.solids]


def render(check, rear, boxes, cam, lift_m=0.0):
    """Ray-cast depth (optical z, metres; NaN no return) of the floor and the yawed boxes."""
    rot, trans = check.optical_from_insertion(rear)
    pos = -rot.T @ trans
    u, w = np.meshgrid(np.arange(cam.width), np.arange(cam.height))
    d_opt = np.stack([(u - cam.cx) / cam.fx, (w - cam.cy) / cam.fy, np.ones_like(u, dtype=float)], axis=-1)
    d = d_opt @ rot  # insertion-frame ray per unit optical z
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(d[..., 2] < -1e-9, -pos[2] / d[..., 2], np.inf)
    t = np.where(t > 0, t, np.inf)
    for b in list(boxes) + list(check.own_boxes(check.to_insertion(*rear), lift_m)):
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        r = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        ol = r @ (pos - np.asarray(b.center))
        dl = d @ r.T
        half = np.asarray(b.half)
        if np.all(np.abs(ol) <= half):
            continue  # the camera inside a box of its own carriage sees out of it
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (-half - ol) / dl, (half - ol) / dl
        tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= tmin) & (tmax > 0)
        t = np.where(hit, np.minimum(t, np.maximum(tmin, 0.0)), t)
    return np.where(np.isfinite(t), t, np.nan)


class TeamNoise:
    """The team capture's frame_640 shape: correlated k z^2 components, dropouts, a few +-4 sigma tails."""

    def __init__(self, seed: int) -> None:
        from forklift_core.perception.depth_noise import NoiseModel

        self.model = NoiseModel.from_yaml(NOISE, seed=seed, target_fx=FX, level="frame_640")
        self.rng = np.random.default_rng([seed, 7])

    def __call__(self, depth):
        from forklift_core.perception.depth_noise import correlated_field

        out = depth.copy()
        z2 = np.nan_to_num(depth) ** 2
        for k, widths in self.model.components:
            out = out + k * z2 * correlated_field(depth.shape, widths, self.rng)
        sigma = math.sqrt(sum(k * k for k, _ in self.model.components))
        tails = self.rng.integers(0, depth.size, 20)
        flat = out.reshape(-1)
        flat[tails] += self.rng.choice((-4.0, 4.0), 20) * sigma * z2.reshape(-1)[tails]
        out[self.rng.random(depth.shape) < self.model.dropout_fraction] = np.nan
        return out


def drive(check, boxes, *, noise=None, max_s=60.0, standoff_m=None):
    """15 Hz frames down the straight; at each, the query QUERY_S later caps the speed. Returns the run's facts:
    stopped (tips before the insertion end, m), the reason, the first stop's refuted count."""
    cam = check.camera
    tip = max(x1 for _x0, x1, _y0, _y1 in check.blades_rear)
    face = check.face_x
    end = face + check.insertion_depth_m - tip
    x = face - (STANDOFF_M if standoff_m is None else standoff_m) - tip
    t, n = 0.0, 0
    dt = 1.0 / HZ
    while x < end - 1e-9:
        rear = (x, 0.0, 0.0)
        depth = render(check, rear, boxes, cam)
        if noise is not None:
            depth = noise(depth)
        cruise = INSERT if x > end - INSERT_LEN else CRUISE
        check.add_frame(t, depth, rear, speed_mps=cruise, frame_key=(1, (), n))
        allowed, why = check.limit(t + QUERY_S, rear, 0.0, 1, cruise, 0.0)
        v = min(cruise, allowed)
        if v <= 0.0:
            fail = getattr(check, "last_fail", None) or {}
            return {"passed": False, "reason": why, "short_of_end_m": round(end - x, 3), "time_s": round(t, 2),
                    "tip_past_face_m": round(x + tip - face, 3), "refuted": int(check.memory.refuted),
                    "last_fail": {k: fail.get(k) for k in ("box", "arc_s", "voxels", "outside_v", "never_certified",
                                                           "expired", "examples_insertion", "examples_last_code")},
                    "fail_detail": fail_detail(check, fail)}
        x = min(end, x + v * dt)
        t += dt
        n += 1
        if t > max_s:
            return {"passed": False, "reason": "timeout", "short_of_end_m": round(end - x, 3)}
    return {"passed": True, "time_s": round(t, 2), "refuted": int(check.memory.refuted)}


def fail_detail(check, fail):
    """The failing box's example voxels: refuted now (certification cleared), needing a recent one, their age."""
    ex = fail.get("examples_insertion") or []
    if not ex:
        return None
    v = check.memory.volume
    idx = np.floor((np.asarray(ex) - v.lo) / v.voxel_m).astype(int)
    cert = check.memory.certified_s[idx[:, 0], idx[:, 1], idx[:, 2]]
    need = check.memory.need_recent[idx[:, 0], idx[:, 1], idx[:, 2]]
    return {"cleared": int(np.isneginf(cert).sum()), "need_recent": int(need.sum()), "n": len(ex)}


def cases(errors_mm, cams):
    out = []
    for cam in cams:
        for e in errors_mm:
            em = e / 1000.0
            out.append({"error_mm": e, "cam": cam, "scene": "aligned", "noise": False})
            out.append({"error_mm": e, "cam": cam, "scene": "aligned", "noise": True})
            for name, off in (("along+", (em, 0, 0)), ("along-", (-em, 0, 0)), ("across+", (0, em, 0)),
                              ("across-", (0, -em, 0)), ("yaw+", (0, 0, math.atan2(em, 0.4))),
                              ("yaw-", (0, 0, -math.atan2(em, 0.4)))):
                if e == 0:
                    continue
                out.append({"error_mm": e, "cam": cam, "scene": name, "offset": off, "noise": False})
                out.append({"error_mm": e, "cam": cam, "scene": name, "offset": off, "noise": False, "bar": True})
            if e == 0:
                out.append({"error_mm": e, "cam": cam, "scene": "aligned", "noise": False, "bar": True})
    return out


def run_case(case: dict) -> dict:
    from forklift_core.perception.insertion_clearance import Box

    import dataclasses

    t0 = time.monotonic()
    check = make_check(case["error_mm"] / 1000.0, case["cam"])
    if case.get("mode") == "isaac":  # the same check with the Isaac rules (no real options, no synthetic noise)
        check = dataclasses.replace(check, real=None, synthetic_noise=False, records=[])
    boxes = true_solids(check, *case.get("offset", (0.0, 0.0, 0.0)))
    bar_x = None
    if case.get("bar"):
        # a 3 cm bar standing in the left pocket, 0.20 m in (as the Isaac test's), clear of the surfaces
        yc = sum(check.blades_rear[0][2:]) / 2
        bar_x = check.face_x + 0.20
        boxes = boxes + [Box((bar_x, yc, 0.04), (0.015, 0.015, 0.04))]
    res = drive(check, boxes, noise=TeamNoise(seed=case["error_mm"]) if case["noise"] else None,
                standoff_m=case.get("standoff_m"))
    if bar_x is not None:
        tip_x = check.face_x + res.get("tip_past_face_m", math.inf)
        res["bar_tip_gap_m"] = None if res["passed"] else round(bar_x - 0.015 - tip_x, 3)
        res["stopped_before_bar"] = (not res["passed"]) and res["bar_tip_gap_m"] > 0
    return {**{k: v for k, v in case.items() if k != "offset"}, **res, "wall_s": round(time.monotonic() - t0, 1)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--errors", default="0,5,10,15,20", help="pallet_rel_error_m settings, mm")
    ap.add_argument("--cams", default="zero,nominal", help="camera ray budgets: zero, nominal (5 mm, 0.5 deg)")
    ap.add_argument("--scenes", default="", help="only these scenes (comma list; empty: all)")
    ap.add_argument("--noise", choices=("both", "off", "on"), default="both")
    ap.add_argument("--bar", choices=("both", "off", "on"), default="both")
    ap.add_argument("--mode", choices=("real", "isaac"), default="real", help="isaac: the same drive, Isaac rules")
    ap.add_argument("--standoff", type=float, default=STANDOFF_M)
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--out", type=Path)
    a = ap.parse_args(argv)
    todo = cases([int(v) for v in a.errors.split(",")], a.cams.split(","))
    keep = set(filter(None, a.scenes.split(",")))
    for c in todo:
        c["mode"], c["standoff_m"] = a.mode, a.standoff
    todo = [c for c in todo if (not keep or c["scene"] in keep)
            and (a.noise == "both" or c["noise"] == (a.noise == "on"))
            and (a.bar == "both" or bool(c.get("bar")) == (a.bar == "on"))]
    rows = []
    with ProcessPoolExecutor(a.jobs) as ex:
        for row in ex.map(run_case, todo):
            print(json.dumps(row), flush=True)
            rows.append(row)
    if a.out:
        a.out.write_text(json.dumps({"mount": MOUNT, "urdf": str(URDF.relative_to(ROOT)), "intrinsics": INTR,
                                     "hz": HZ, "speeds": [CRUISE, INSERT, INSERT_LEN], "rows": rows}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
