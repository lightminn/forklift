"""The transport-start stall in a tight slot (riib aisle_rack, trailer), reproduced on the CPU from a run's obstacle dump
and grid frames, with the existing counter-measures and two candidate additions put in (diagnosis, 2026-10-27; no
runner change).

A failing run stands right after the extract with its loaded outline within a few centimetres of the LiDAR marks of a
tall prop's halo (a rack upright, a trailer wall) whose real surface is about 0.2 m away: the permission holds the
transport at 0 ("occupied"), the P6 scene replan's start is invalid and every escape candidate is refused on its grid
checks. From the first transport-phase dump this tool rebuilds the permission (the run's layer config and loaded
outline) and reports, for each variant:

- ``stand``: the permission at the stand (the transport leg ahead, the stop arc at the steering used) -- m/s and reason;
- ``escapes``: the P6 escape candidates' grid checks (``escape.evaluate``'s path, ramped path and the three stop arcs
  at kappa, kappa/2, 0) -- each candidate ``ok`` or which check holds it; ``ok`` here leaves out the rectangle sweep and
  the end region, which the grid variants below do not change (the record's sweep values are printed beside them);
- ``start_to_occupied_m``: the outline's distance to the nearest occupied cell left (the P6 replan's grid start).

Variants:

- ``off``: every counter-measure off, as the run;
- ``band``: the start-band release (#128, docs/plans/2026-10-24-start-band-release.md) at the launcher's width
  (envelope + step / 2) and wider;
- ``zero``: (i) the steering set to 0 before the start (the stop arcs at kappa 0; a stand-still presteer), with the band;
- ``corridor``: (ii) the extract leg's swept outline as known-clear: an occupied cell is read as free when the truck's
  outline covered it during the extract and the cell was already occupied at that moment (a mark the truck stood in is
  not an obstacle); a cell first marked after the outline left it is kept (it may be new);
- ``corridor+band``, ``corridor+zero``: the two together.

The escape-start release (#151 E2) acts on the rectangle sweep only, so it changes none of these grid checks; its row is
the ``off`` row by construction.

    python tools/transport_start_stall.py RUN_DIR [RUN_DIR ...] [--json OUT]
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / "src", ROOT / "sim/isaac"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from forklift_core.control import escape as esc  # noqa: E402
from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel, in_shape  # noqa: E402
from forklift_core.control.start_release import cell_distances, footprint_rect_distance  # noqa: E402
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot  # noqa: E402
from forklift_core.planning import Footprint, Rectangle  # noqa: E402

ESCAPE_SPEED_MPS = 0.1


def layer_permission_config(path: Path) -> PermissionConfig:
    """The permission config the run's obstacle layer built (sim/isaac/obstacle_layer.py)."""
    cfg = yaml.safe_load(path.read_text())
    p, g = cfg["permission"], cfg["grid"]
    return PermissionConfig(StoppingModel(float(p["latency_s"]), float(p["decel_mps2"]), float(p["margin_m"])),
                            envelope_offset_m=float(p["envelope_m"]), evidence_max_age_s=float(g["free_max_age_s"]),
                            envelope_ramp_m=float(p["envelope_ramp_m"]), sensor_timeout_s=float(p["sensor_timeout_s"]),
                            lookahead_m=float(p["lookahead_m"]), shadow_band_m=float(p.get("shadow_band_m", 0.0)))


def first_transport_dump(run: Path):
    for f in sorted(glob.glob(str(run / "run/obstacle_dump_*.npz")), key=lambda s: int(s.split("_")[-1][:-4])):
        d = np.load(f, allow_pickle=True)
        if str(d["phase"]) == "transport":
            return d
    return None


def snapshot_of(dump, state=None) -> GridSnapshot:
    return GridSnapshot(state=np.asarray(dump["state"]).copy() if state is None else state,
                        free_stamp=np.asarray(dump["free_stamp"]), stamp_s=float(dump["t"]),
                        correction=tuple(float(v) for v in dump["correction"]),
                        correction_version=int(dump["version"]), scans_used=1,
                        origin_x_m=float(dump["origin"][0]), origin_y_m=float(dump["origin"][1]),
                        resolution_m=float(dump["res"]))


def extract_corridor_release(run: Path, record: dict, dump, footprint: Footprint) -> tuple[np.ndarray, dict]:
    """(ii): the dump's occupied cells to read as free -- covered by the loaded outline at some pose of the extract
    and already occupied in the grid frame at that moment. The extract's poses are its planned path, timed linearly
    between the extract's and the transport's start transitions (the truck drives it at a near-constant speed)."""
    times_ = {(t.get("phase") or t.get("to")): float(t.get("time_s", t.get("t", 0.0))) for t in record["transitions"]}
    t0, t1 = times_["extract"], times_["transport"]
    poses = np.asarray(record["paths"]["extract"]["poses"], dtype=float)
    seg = np.r_[0.0, np.cumsum(np.hypot(*np.diff(poses[:, :2], axis=0).T))]
    when = t0 + (t1 - t0) * (seg / seg[-1] if seg[-1] > 0 else np.zeros_like(seg))
    frames = np.load(run / "run/obstacle_grid_frames.npz", allow_pickle=True)
    ftimes, forigins, fres, offs, fcells = (frames["times"], frames["origins"], frames["resolution"], frames["offsets"],
                                            frames["cells"])
    state = np.asarray(dump["state"])
    ox, oy, res = float(dump["origin"][0]), float(dump["origin"][1]), float(dump["res"])
    occ = np.argwhere(state == OCCUPIED)
    cx, cy = ox + (occ[:, 0] + 0.5) * res, oy + (occ[:, 1] + 0.5) * res
    release = np.zeros(len(occ), dtype=bool)
    stats = {"occupied": int(len(occ)), "covered": 0, "covered_new": 0}

    def occupied_in_frame(k, x, y):
        cells = fcells[offs[k]:offs[k + 1]]
        if not len(cells):
            return np.zeros(len(x), dtype=bool)
        r = float(fres[k])
        i = np.floor((x - forigins[k][0]) / r).astype(int)
        j = np.floor((y - forigins[k][1]) / r).astype(int)
        have = set(map(tuple, cells.tolist()))
        return np.array([(a, b) in have for a, b in zip(i, j)])

    covered_at = np.full(len(occ), np.nan)
    for pose, t in zip(poses, when):
        inside = in_shape(cx, cy, tuple(pose), footprint)
        covered_at[inside] = t  # the last moment the outline covered the cell's centre
    covered = ~np.isnan(covered_at)
    stats["covered"] = int(covered.sum())
    for idx in np.flatnonzero(covered):
        k = int(np.searchsorted(ftimes, covered_at[idx], side="right") - 1)
        if k < 0:
            continue
        if occupied_in_frame(k, cx[idx:idx + 1], cy[idx:idx + 1])[0]:
            release[idx] = True
        else:
            stats["covered_new"] += 1  # first marked after the outline left: kept
    return occ[release], stats


def evaluate(run: Path) -> dict:
    record = json.loads((run / "run/result.json").read_text())
    dump = first_transport_dump(run)
    if dump is None:
        return {"run": run.name, "error": "no transport-phase dump"}
    base_cfg = layer_permission_config(ROOT / record["obstacle_layer"]["config"])
    lf = record["geometry"]["loaded_footprint"]
    fp = Footprint(float(lf["front_m"]), float(lf["rear_m"]), float(lf["half_width_m"]))
    pose = tuple(float(v) for v in dump["rear"])
    leg = np.asarray(dump["leg"], dtype=float)
    direction, kappa, t = int(dump["direction"]), float(dump["kappa"]), float(dump["t"])
    version = int(dump["version"])
    band_launcher = base_cfg.envelope_offset_m + base_cfg.step_m / 2.0
    released, corridor_stats = extract_corridor_release(run, record, dump, fp)
    corridor_state = np.asarray(dump["state"]).copy()
    for i, j in released:
        corridor_state[i, j] = FREE
    free_stamp_t = t
    stop_len = base_cfg.stopping.distance_m(ESCAPE_SPEED_MPS) + base_cfg.step_m

    def variant(band: float, zero, corridor: bool) -> dict:
        """``zero``: False (the held steering), True (0) or a curvature the steering is set to first."""
        cfg = replace(base_cfg, start_band_m=band)
        snap = snapshot_of(dump)
        if corridor:
            fs = np.asarray(dump["free_stamp"]).copy()
            for i, j in released:
                fs[i, j] = free_stamp_t
            snap = replace(snap, state=corridor_state.copy(), free_stamp=fs)
        k_now = kappa if zero is False else (0.0 if zero is True else float(zero))

        def fresh():
            perm = DrivePermission(cfg)
            if band > 0:
                perm.note_standing(True, pose, version)
            return perm

        perm = fresh()
        perm.update(snap, leg, fp, fp, current_pose=pose, direction=direction)
        allowed, why = perm.allowed_speed(t, {}, current_pose=pose, curvature_inv_m=k_now, direction=direction,
                                          footprint=fp, own_footprint=fp, speed_cap_mps=0.12)
        escapes = {}
        for d in (1, -1):
            arcs = [fresh().check_arc(snap, pose, k, d, fp, fp, ESCAPE_SPEED_MPS).blocked
                    for k in (k_now, k_now / 2.0, 0.0)]
            for length in esc.LENGTHS_M:
                ahead = esc.straight(pose, d, length + stop_len)
                path = fresh().check_path(snap, ahead, fp, fp, current_pose=pose, direction=d).blocked
                ramped = fresh().check_path(snap, ahead, fp, fp, current_pose=pose, direction=d,
                                            extra_margin_m=esc.EXTRA_MARGIN_M, extra_ramp_m=length).blocked
                ok = path is None and ramped is None and all(a is None for a in arcs)
                escapes[f"{d:+d}x{length:g}"] = "ok" if ok else (
                    "path" if path else "ramped" if ramped else "arc")
        occ = np.argwhere(np.asarray(snap.state) == OCCUPIED)
        near = occ[np.hypot(snap.origin_x_m + (occ[:, 0] + 0.5) * snap.resolution_m - pose[0],
                            snap.origin_y_m + (occ[:, 1] + 0.5) * snap.resolution_m - pose[1]) < 4.0]
        start_d = float(cell_distances(snap, near, pose, fp).min()) if len(near) else math.inf
        return {"stand": [round(float(allowed), 3), why], "escapes": escapes,
                "escapes_ok": sum(v == "ok" for v in escapes.values()), "start_to_occupied_m": round(start_d, 4)}

    variants = {"off": variant(0.0, False, False)}
    for band in (band_launcher, 0.03):
        variants[f"band{band:g}"] = variant(band, False, False)
        variants[f"zero+band{band:g}"] = variant(band, True, False)
    variants["corridor"] = variant(0.0, False, True)
    variants[f"corridor+band{band_launcher:g}"] = variant(band_launcher, False, True)
    variants[f"corridor+zero+band{band_launcher:g}"] = variant(band_launcher, True, True)
    variants["corridor+zero"] = variant(0.0, True, True)
    # a presteer to a set curvature (plan 2026-10-27 escape presteer): the escapes' arcs at that steering
    for k_set in (-0.05, -0.02, 0.02, 0.05):
        for band in (band_launcher, 0.03):
            variants[f"steer{k_set:+g}+band{band:g}"] = variant(band, k_set, False)
    tall = [pp for pp in record["scenario"]["props"] if float(pp["asset"].get("height_m", 0.0)) > 1.05]
    truth = tuple(float(v) for v in dump["truth_rear"])
    gap = min(((footprint_rect_distance(truth, fp, Rectangle(**pp["rectangle"])), pp["asset"]["uri"]) for pp in tall),
              default=(math.inf, None))
    record_sweep = {}
    for e in (record.get("scene_escape_candidates") or [])[:1]:
        record_sweep = {f"{c['direction']:+d}x{c['length_m']:g}": c.get("sweep_free") for c in e["candidates"]}
    return {"run": run.name, "t": round(t, 2), "rear": [round(v, 3) for v in pose], "kappa": round(kappa, 4),
            "direction": direction, "band_launcher_m": band_launcher, "corridor": corridor_stats,
            "corridor_released": int(len(released)), "tall_gap_truth_m": [round(gap[0], 4), gap[1]],
            "permit_threshold_m": base_cfg.envelope_offset_m + base_cfg.stopping.margin_m,
            "record_sweep_free": record_sweep, "variants": variants}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    out = []
    for run in args.runs:
        try:
            out.append(evaluate(run))
        except Exception as exc:  # noqa: BLE001 -- one run's failure is reported, the others go on
            out.append({"run": run.name, "error": repr(exc)})
        r = out[-1]
        if "error" in r:
            print(r)
            continue
        print(f"{r['run']}: kappa {r['kappa']} corridor released {r['corridor_released']} "
              f"(covered {r['corridor']['covered']}, new {r['corridor']['covered_new']}) gap {r['tall_gap_truth_m']}")
        for name, v in r["variants"].items():
            print(f"  {name:22s} stand {v['stand']} escapes_ok {v['escapes_ok']}/8 start->occ {v['start_to_occupied_m']}")
    if args.json:
        args.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
