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

# the scene replan's escape speed: options.backoff_speed_mps = run_transport.BACKOFF_SPEED_MPS (Codex diagnosis
# review P2: 0.10 here before, a shorter stop arc and path extension than the runner checks)
ESCAPE_SPEED_MPS = 0.15


def threshold_args(record: dict):
    """(planner clearance, psi, cell) that give the record's escape end thresholds (escape.end_threshold_m is linear in
    the length: clearance + 2 sin(psi/2) L + cell); None without a recorded escape evaluation."""
    events = record.get("scene_escape_candidates") or []
    if not events:
        return None
    by_len = sorted({float(c["length_m"]): float(c["threshold_m"]) for c in events[0]["candidates"]}.items())
    (l0, t0), (l1, t1) = by_len[0], by_len[-1]
    slope = (t1 - t0) / (l1 - l0)
    cell = 0.05
    return t0 - slope * l0 - cell, 2.0 * math.asin(slope / 2.0), cell


def full_escape(dump, record, base_cfg, fp, *, band: float, kappa_arc: float, kappa_measured: float,
                shift=None, anchor_at_shift: bool = False) -> dict:
    """The scene replan's escape_plan (src/forklift_core/mission/core.py) on the dump: the trigger's rectangles
    (permit_stop.current_blocked_rectangles at the held steering), the transport leg's rectangles (the scene props with
    the run's KEEPOUT), the start-band split of the trigger's rectangles with the release on, every check of
    escape.evaluate at ``kappa_arc`` (the stop arcs at kappa, kappa/2, 0), the choice, and candidate_permit at the
    measured steering ``kappa_measured`` -- the escape is installed only when that permit is ok."""
    from forklift_core.mission.permit_stop import current_blocked_rectangles
    from forklift_core.planning.geometry import FootprintCollisionChecker
    from forklift_core.planning.pallet_mission import LIDAR_PLANE_M, scenario_props
    from obstacle_layer import ObstacleLayer
    from scene_file import scenario_from_record

    cfg = replace(base_cfg, start_band_m=band)
    snap = snapshot_of(dump)
    anchor = tuple(float(v) for v in dump["rear"])
    # an escape presteer's stand (escape presteer C4): evaluated from the stand shifted by the presteer's own move, the
    # start-band anchor at the first stand (the implementation) or at the shifted one (re-latched)
    pose = anchor if shift is None else tuple(a + d for a, d in zip(anchor, shift))
    if anchor_at_shift:
        anchor = pose
    leg = np.asarray(dump["leg"], dtype=float)
    direction = int(dump["direction"]) or -1
    layer = ObstacleLayer.__new__(ObstacleLayer)  # the layer's check methods on the dump's processed snapshot
    layer.permission, layer.snapshot, layer.last_scan_s = DrivePermission(cfg), snap, {}
    layer.loaded = layer.unloaded = layer.body = layer.unloaded_shape = fp
    layer.shape_mode = "hull"
    layer.permission.update(snap, leg, fp, fp, current_pose=pose, direction=direction)
    if band > 0:
        layer.permission.note_standing(True, anchor, int(dump["version"]))  # the device's _note_release_standing
    cruise_ = (record.get("tracker_configs") or {}).get("transport", 0.3)
    cruise = float(cruise_["cruise_speed_mps"] if isinstance(cruise_, dict) else cruise_)  # the trigger's arc cap
    extra = current_blocked_rectangles(layer, rear=pose, leg=leg, direction=direction, loaded=True,
                                       kappa=float(dump["kappa"]), speed_cap_mps=cruise)
    scenario = scenario_from_record(record["scenario"])
    rects = scenario_props(scenario, keepout_m=float(record["geometry"].get("lidar_keepout_m", 0.0)),
                           plane_m=LIDAR_PLANE_M)
    checker = FootprintCollisionChecker(list(rects) + list(extra), fp, scenario.bounds)
    if band > 0:
        near, far = esc.start_band_rectangles(list(extra), fp, pose)
        sweep_checker = FootprintCollisionChecker(list(rects) + far, fp, scenario.bounds)
    else:
        near, sweep_checker = [], checker
    targs = threshold_args(record)
    if targs is None:
        return {"error": "no recorded escape evaluation"}
    clear, psi, cell = targs
    stop_len = cfg.stopping.distance_m(ESCAPE_SPEED_MPS) + cfg.step_m
    cands = esc.evaluate(
        pose,
        sweep_free=lambda poses, d: esc.rectangles_sweep_free(sweep_checker, poses, near, fp),
        path_blocked=lambda poses, d, x, r: layer.check_candidate(
            poses, current_pose=pose, direction=d, loaded=True, extra_margin_m=x, extra_ramp_m=r).blocked,
        arc_blocked=lambda k, d: layer.check_arc_candidate(
            current_pose=pose, curvature_inv_m=k, direction=d, loaded=True, cap_mps=ESCAPE_SPEED_MPS).blocked,
        end_clearance=lambda region: (lambda v: (min(v) >= 0.0, min(v)))(
            [esc.clearance_m(checker.free, p) for p in region]),
        kappa_now=kappa_arc, stop_length_m=stop_len, planner_clearance_m=clear, psi_rad=psi, cell_m=cell)
    choice = esc.choose(cands, direction)
    permit = None
    if choice is not None:
        permit = layer.candidate_permit(
            float(dump["t"]), esc.straight(pose, choice.direction, choice.length_m), current_pose=pose,
            direction=int(choice.direction), loaded=True, curvature_inv_m=0.0,
            measured_curvature_inv_m=kappa_measured, cap_mps=ESCAPE_SPEED_MPS)

    def code(c):
        if c.qualifies:
            return "ok"
        if not c.sweep_free:
            return "sweep"
        if c.path is not None:
            return "path"
        if c.path_ramped is not None:
            return "ramped"
        if any(b is not None for _, b in c.arcs):
            return "arc"
        return f"end{c.end_clearance_m:.3f}<{c.threshold_m:.3f}" if c.end_free else "end_hit"

    # which rectangles hold a sweep (the separate diagnosis of the stands no variant frees): the scene props with
    # KEEPOUT alone, the trigger's rectangles alone, and the props at their own size (KEEPOUT 0)
    plain = scenario_props(scenario, keepout_m=0.0, plane_m=LIDAR_PLANE_M)
    split = {}
    for c in cands:
        poses = esc.straight(pose, c.direction, c.length_m)
        split[f"{c.direction:+d}x{c.length_m:g}"] = [
            esc.rectangles_sweep_free(FootprintCollisionChecker(list(rects), fp, scenario.bounds), poses),
            esc.rectangles_sweep_free(FootprintCollisionChecker(list(extra), fp, scenario.bounds), poses),
            esc.rectangles_sweep_free(FootprintCollisionChecker(list(plain), fp, scenario.bounds), poses)]
    return {"candidates": {f"{c.direction:+d}x{c.length_m:g}": code(c) for c in cands},
            "sweep_free_props_trigger_plain": split,
            "qualifying": sum(c.qualifies for c in cands),
            "chosen": None if choice is None else f"{choice.direction:+d}x{choice.length_m:g}",
            "permit": None if permit is None else permit["why"],
            "installs": bool(permit is not None and permit["ok"]),
            "extra_rects": len(extra), "released_rects": len(near),
            # the recorded evaluation's checks, to compare the off variant with (the record's pose and time may differ)
            "record": {f"{c['direction']:+d}x{c['length_m']:g}": [c["sweep_free"], c["path"], c["path_ramped"],
                                                                   [b for _, b in c["arcs"]]]
                       for c in (record.get("scene_escape_candidates") or [{}])[0].get("candidates", [])}}


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
    # the driving tick's cap: max(|requested|, |truth speed|) (core.py; Codex diagnosis review P2)
    stand_cap = max(abs(float(dump["requested"])), abs(float(dump["truth_speed"])))

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
                                          footprint=fp, own_footprint=fp, speed_cap_mps=stand_cap)
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
    # the full escape_plan (escape.evaluate with the rectangle sweep and the end region, and the install permit)
    full = {}
    for name_, band_, k_arc_, k_meas_ in (
            ("off", 0.0, kappa, kappa), (f"band{band_launcher:g}", band_launcher, kappa, kappa),
            ("band0.03", 0.03, kappa, kappa), ("zero+band0.03", 0.03, 0.0, 0.0),
            ("steer-0.05", 0.0, -0.05, -0.05), ("steer+0.05", 0.0, 0.05, 0.05),
            ("steer-0.02+band0.03", 0.03, -0.02, -0.02), ("steer+0.02+band0.03", 0.03, 0.02, 0.02),
            ("steer-0.05+band0.03", 0.03, -0.05, -0.05), ("steer+0.05+band0.03", 0.03, 0.05, 0.05)):
        full[name_] = full_escape(dump, record, base_cfg, fp, band=band_, kappa_arc=k_arc_, kappa_measured=k_meas_)
    # an escape presteer's record (C4 runs): its move while steering and the steering measured after it
    presteer = {}
    for r_ in record.get("scene_replans") or []:
        ep_ = r_.get("escape_presteer") or {}
        if r_.get("phase") == "transport" and ep_.get("rear_start") and ep_.get("rear_end"):
            shift_ = [b - a for a, b in zip(ep_["rear_start"], ep_["rear_end"])]
            k_m_ = next((e.get("kappa_measured") for e in record.get("scene_escape_candidates") or []
                         if e.get("presteered")), None)
            if k_m_ is None:
                continue
            presteer = {"shift": shift_, "kappa_measured": k_m_,
                        "anchor_first": full_escape(dump, record, base_cfg, fp, band=0.03, kappa_arc=k_m_,
                                                    kappa_measured=k_m_, shift=shift_),
                        "anchor_relatched": full_escape(dump, record, base_cfg, fp, band=0.03, kappa_arc=k_m_,
                                                        kappa_measured=k_m_, shift=shift_, anchor_at_shift=True),
                        "no_move": full_escape(dump, record, base_cfg, fp, band=0.03, kappa_arc=k_m_,
                                               kappa_measured=k_m_)}
            break
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
            "record_sweep_free": record_sweep, "variants": variants, "full": full,
            "record_escape_t": (record.get("scene_escape_candidates") or [{}])[0].get("time_s"), "presteer": presteer}


def escape_replay(run: Path) -> dict:
    """An installed presteered escape that stopped at once (C4'): the driving permission re-checked where it stood.
    The grid at that moment is approximated -- the first transport dump's grid frame with every occupied cell cleared to
    fresh free and the occupied cells of the stored grid frame nearest the stop (0.5 s frames) marked again -- and the
    permission walks the escape's straight from the stop pose with the start-band anchor where the re-evaluation latched
    it (the presteered evaluation's pose). Reported per variant: the allowed speed and its reason, the walk's verified
    length and blocked cells, the release counts."""
    record = json.loads((run / "run/result.json").read_text())
    drives = record.get("escape_presteer_drives") or []
    evals = [e for e in record.get("scene_escape_candidates") or [] if e.get("presteered") and e.get("pose")]
    dump = first_transport_dump(run)
    if not drives or not evals or dump is None:
        return {"run": run.name, "error": "no presteered escape drive, evaluation or dump"}
    ev, drive = evals[-1], drives[-1]
    anchor = tuple(float(v) for v in ev["pose"])
    kappa_m = float(ev["kappa_measured"])
    chosen = ev["chosen"]
    base_cfg = replace(layer_permission_config(ROOT / record["obstacle_layer"]["config"]), start_band_m=0.03)
    lf = record["geometry"]["loaded_footprint"]
    fp = Footprint(float(lf["front_m"]), float(lf["rear_m"]), float(lf["half_width_m"]))
    t_eval = float(ev["time_s"])
    # where it stood: the samples (10 Hz) between the install and the drive's end
    samples = [x for x in record.get("samples") or [] if t_eval <= float(x["time_s"]) <= float(drive["time_s"])]
    moved = [x for x in samples if abs(float(x.get("signed_speed_mps", 0.0))) > 1e-3]
    stop = samples[-1] if samples else None
    frames = np.load(run / "run/obstacle_grid_frames.npz", allow_pickle=True)
    ftimes = np.asarray(frames["times"], dtype=float)
    out = {"run": run.name, "anchor": [round(v, 4) for v in anchor], "kappa_measured": kappa_m,
           "chosen": [chosen["direction"], chosen["length_m"]], "t_eval": round(t_eval, 2),
           "drive_end": drive["end"], "samples_moving": len(moved), "variants": {}}
    for label, t_at in (("eval", t_eval), ("stop", float(drive["time_s"]) - 1.0)):
        k = int(np.searchsorted(ftimes, t_at, side="right") - 1)
        if k < 0:
            continue
        cells = frames["cells"][frames["offsets"][k]:frames["offsets"][k + 1]]
        fres, forg = float(frames["resolution"][k]), frames["origins"][k]
        state = np.asarray(dump["state"]).copy()
        state[state == OCCUPIED] = FREE
        free_stamp = np.full(state.shape, t_at)
        ox, oy, res = float(dump["origin"][0]), float(dump["origin"][1]), float(dump["res"])
        for i, j in np.asarray(cells, dtype=int).reshape(-1, 2):
            x, y = forg[0] + (i + 0.5) * fres, forg[1] + (j + 0.5) * fres
            a, b = int(np.floor((x - ox) / res)), int(np.floor((y - oy) / res))
            if 0 <= a < state.shape[0] and 0 <= b < state.shape[1]:
                state[a, b] = OCCUPIED
        snap = replace(snapshot_of(dump, state=state), free_stamp=free_stamp, stamp_s=t_at)
        for where, pose in (("at_anchor", anchor),
                            ("at_stop", tuple(float(v) for v in stop["rear_pose"]) if stop else anchor)):
            for band in (0.0, 0.03):
                perm = DrivePermission(replace(base_cfg, start_band_m=band))
                if band > 0:
                    perm.note_standing(True, anchor, int(dump["version"]))
                    perm.note_standing(False, anchor, int(dump["version"]))  # moving off: the anchor stays
                leg = esc.straight(pose, int(chosen["direction"]), float(chosen["length_m"]))
                check = perm.update(snap, leg, fp, fp, current_pose=pose, direction=int(chosen["direction"]))
                allowed, why = perm.allowed_speed(t_at, {}, current_pose=pose, curvature_inv_m=kappa_m,
                                                  direction=int(chosen["direction"]), footprint=fp,
                                                  own_footprint=fp, speed_cap_mps=ESCAPE_SPEED_MPS)
                occ = np.argwhere(state == OCCUPIED)
                near = occ[np.hypot(ox + (occ[:, 0] + 0.5) * res - pose[0], oy + (occ[:, 1] + 0.5) * res - pose[1]) < 4.0]
                d = cell_distances(snap, near, pose, fp) if len(near) else np.zeros(0)
                out["variants"][f"{label}_grid/{where}/band{band:g}"] = {
                    "allowed": round(float(allowed), 3), "why": why, "verified_m": round(float(check.verified_m), 3),
                    "blocked": check.blocked, "release": dict(getattr(perm, "release_stats", {})),
                    "nearest_occ_m": round(float(d.min()), 4) if len(d) else None,
                    "pose": [round(v, 4) for v in pose], "frame_t": round(float(ftimes[k]), 2)}
    return out


def _rect_point_distance(rect, x, y):
    """Distance from points (x, y) to a Rectangle's area (0 inside)."""
    c, s_ = math.cos(rect.yaw_rad), math.sin(rect.yaw_rad)
    dx, dy = x - rect.x_m, y - rect.y_m
    u, v = np.abs(c * dx + s_ * dy) - rect.length_m / 2.0, np.abs(-s_ * dx + c * dy) - rect.width_m / 2.0
    return np.hypot(np.maximum(u, 0.0), np.maximum(v, 0.0))


def alternatives(run: Path) -> dict:
    """The lead's comparison (2026-10-28) on a failing stand's first transport dump:
    (b) the known tall props judged by their shape -- every occupied cell within ``halo`` of a tall prop cleared, the prop
        grown by a margin m marked occupied -- and the transport leg's stand permit at the held steering;
    (c) the extract carried on straight from the stand: the shortest extension s whose straight (plus the stop length) the
        permission passes with the band off, at whose end the outline is at least ``clear`` from every occupied cell and
        the props at their size are not met.
    Also the stand's control-frame and truth distances to the nearest tall prop."""
    record = json.loads((run / "run/result.json").read_text())
    dump = first_transport_dump(run)
    if dump is None:
        return {"run": run.name, "error": "no transport-phase dump"}
    base_cfg = layer_permission_config(ROOT / record["obstacle_layer"]["config"])
    lf = record["geometry"]["loaded_footprint"]
    fp = Footprint(float(lf["front_m"]), float(lf["rear_m"]), float(lf["half_width_m"]))
    pose = tuple(float(v) for v in dump["rear"])
    truth = tuple(float(v) for v in dump["truth_rear"])
    leg = np.asarray(dump["leg"], dtype=float)
    direction, kappa, t = int(dump["direction"]) or -1, float(dump["kappa"]), float(dump["t"])
    props = record["scenario"]["props"]
    tall = [Rectangle(**pp["rectangle"]) for pp in props if float(pp["asset"].get("height_m", 0.0)) > 1.05]
    plain = [Rectangle(**pp["rectangle"]) for pp in props]
    out = {"run": run.name,
           "tall_ctrl_m": round(min((footprint_rect_distance(pose, fp, r) for r in tall), default=math.inf), 4),
           "tall_truth_m": round(min((footprint_rect_distance(truth, fp, r) for r in tall), default=math.inf), 4),
           "b": {}, "c": None}
    state0 = np.asarray(dump["state"])
    ox, oy, res = float(dump["origin"][0]), float(dump["origin"][1]), float(dump["res"])
    ii, jj = np.meshgrid(np.arange(state0.shape[0]), np.arange(state0.shape[1]), indexing="ij")
    cx, cy = ox + (ii + 0.5) * res, oy + (jj + 0.5) * res
    near_tall = np.zeros(state0.shape, dtype=bool)
    for r in tall:
        near_tall |= _rect_point_distance(r, cx, cy) <= 0.30
    cap = max(abs(float(dump["requested"])), abs(float(dump["truth_speed"])), 0.15)
    for m in (0.03, 0.05, 0.10):
        state = state0.copy()
        state[near_tall & (state0 == OCCUPIED)] = FREE
        for r in tall:
            state[_rect_point_distance(r, cx, cy) <= m] = OCCUPIED
        snap = replace(snapshot_of(dump, state=state), free_stamp=np.where(state != state0, t, dump["free_stamp"]))
        perm = DrivePermission(base_cfg)
        check = perm.update(snap, leg, fp, fp, current_pose=pose, direction=direction)
        allowed, why = perm.allowed_speed(t, {}, current_pose=pose, curvature_inv_m=kappa, direction=direction,
                                          footprint=fp, own_footprint=fp, speed_cap_mps=cap)
        out["b"][f"m{m:g}"] = {"allowed": round(float(allowed), 3), "why": why,
                               "verified_m": round(float(check.verified_m), 3), "blocked": check.blocked}
    # (c): the extract straight carried on, band off, on the dump's own grid
    snap = snapshot_of(dump)
    ext_dir = -1  # the extract backs out; its continuation is straight back along the heading
    stop_len = base_cfg.stopping.distance_m(ESCAPE_SPEED_MPS) + base_cfg.step_m
    occ = np.argwhere(state0 == OCCUPIED)
    for s_ in np.arange(0.1, 3.01, 0.1):
        straight_ = esc.straight(pose, ext_dir, float(s_) + stop_len)
        perm = DrivePermission(base_cfg)
        blocked = perm.check_path(snap, straight_, fp, fp, current_pose=pose, direction=ext_dir).blocked
        end = tuple(esc.straight(pose, ext_dir, float(s_))[-1])
        near = occ[np.hypot(ox + (occ[:, 0] + 0.5) * res - end[0], oy + (occ[:, 1] + 0.5) * res - end[1]) < 4.0]
        d_cells = float(cell_distances(snap, near, end, fp).min()) if len(near) else math.inf
        sweep_ok = all(min((footprint_rect_distance(tuple(p_), fp, r) for r in plain), default=math.inf) > 0.0
                       for p_ in esc.straight(pose, ext_dir, float(s_)))
        if d_cells >= 0.10 and blocked is None and sweep_ok:
            out["c"] = {"extension_m": round(float(s_), 2), "end_to_cells_m": round(d_cells, 3)}
            break
    if out["c"] is None:
        out["c"] = {"extension_m": None, "note": "no extension up to 3 m with the straight permitted, the end 0.10 m "
                                                 "from every mark and the props at their size not met"}
    return out


def extract_extension(run: Path) -> dict:
    """(c) under the extract's own rules (Codex comparison review P2: the drive permission does not act on the extract):
    the extract carried on straight back by s, its sweep against the scene props at their size and with the run's
    KEEPOUT (the smallest clearance over 1 cm samples), and at its end the transport direction's start permit on the dump
    grid (band off) at the planner's largest curvature both ways and at 0, the leg ahead a straight of 1 m. The first s
    whose end permits a start at every one of the three curvatures, and the first at any."""
    record = json.loads((run / "run/result.json").read_text())
    dump = first_transport_dump(run)
    if dump is None:
        return {"run": run.name, "error": "no transport-phase dump"}
    base_cfg = layer_permission_config(ROOT / record["obstacle_layer"]["config"])
    lf = record["geometry"]["loaded_footprint"]
    fp = Footprint(float(lf["front_m"]), float(lf["rear_m"]), float(lf["half_width_m"]))
    pose = tuple(float(v) for v in dump["rear"])
    t = float(dump["t"])
    direction = int(dump["direction"]) or -1
    kmax = float((record.get("planner_config") or {}).get("curvature_limit_inv_m", 0.29))
    keepout = float(record["geometry"].get("lidar_keepout_m", 0.0))
    from forklift_core.planning.pallet_mission import LIDAR_PLANE_M, scenario_props
    from scene_file import scenario_from_record

    scenario = scenario_from_record(record["scenario"])
    plain = scenario_props(scenario, keepout_m=0.0, plane_m=LIDAR_PLANE_M)
    grown = scenario_props(scenario, keepout_m=keepout, plane_m=LIDAR_PLANE_M)
    snap = snapshot_of(dump)
    rows, first_all, first_any = [], None, None
    for s_ in np.arange(0.0, 3.01, 0.1):
        poses = esc.straight(pose, -1, float(s_), step_m=0.01) if s_ > 0 else np.asarray([pose])
        clear_plain = min(min(footprint_rect_distance(tuple(p_), fp, r) for r in plain) for p_ in poses)
        clear_grown = min(min(footprint_rect_distance(tuple(p_), fp, r) for r in grown) for p_ in poses) if grown \
            else math.inf
        end = tuple(poses[-1])
        allowed = {}
        for k in (-kmax, 0.0, kmax):
            perm = DrivePermission(base_cfg)
            perm.update(snap, esc.straight(end, direction, 1.0), fp, fp, current_pose=end, direction=direction)
            a_, why_ = perm.allowed_speed(t, {}, current_pose=end, curvature_inv_m=k, direction=direction,
                                          footprint=fp, own_footprint=fp, speed_cap_mps=0.3)
            allowed[f"{k:+.2f}"] = [round(float(a_), 3), why_]
        ok_all = all(v[0] > 0 for v in allowed.values()) and clear_plain > 0
        ok_any = any(v[0] > 0 for v in allowed.values()) and clear_plain > 0
        rows.append({"s": round(float(s_), 2), "clear_plain_m": round(float(clear_plain), 3),
                     "clear_keepout_m": round(float(clear_grown), 3), "start": allowed})
        if ok_all and first_all is None:
            first_all = round(float(s_), 2)
        if ok_any and first_any is None:
            first_any = round(float(s_), 2)
    plan = {}
    if first_all is not None:
        from forklift_core.planning import Pose2D
        from forklift_core.planning.hybrid_astar import PlannerConfig
        from forklift_core.planning.pallet_mission import ExitPrefix, SyntheticMissionGeometry, plan_transport_leg

        g = record["geometry"]
        geometry = SyntheticMissionGeometry(
            unloaded_footprint=Footprint(**g["unloaded_footprint"]), **{
                k: g[k] for k in ("pallet_depth_m", "pallet_width_m", "axle_to_fork_tip_m", "approach_gap_m",
                                  "alignment_straight_m", "delivery_straight_m", "extraction_m", "withdrawal_m",
                                  "spawn_clearance_m", "carriage_limit_m", "insertion_reserve_m", "lidar_keepout_m")
                if k in g})
        config = PlannerConfig(**record["planner_config"])
        from forklift_core.planning.pallet_mission import keepout_exit_prefix

        ex = record.get("exit_prefix")
        if not ex:
            exit_prefix = None
        elif ex.get("source") == "lidar_keepout":  # the keepout's own prefix, as the runner builds it
            exit_prefix = keepout_exit_prefix(scenario, geometry, float(config.clearance_m))
        else:
            exit_prefix = ExitPrefix(tuple(Rectangle(**w) for w in ex["walls"]), float(ex["clearance_m"]),
                                     float(ex.get("step_m", 0.05)), float(ex.get("cap_m", 1.0)))
        for s_ in (0.0, first_all, round(first_all + 0.2, 2)):
            end = tuple(esc.straight(pose, -1, float(s_))[-1]) if s_ > 0 else pose
            res = plan_transport_leg(scenario, Pose2D(*end), config, geometry=geometry, exit_prefix=exit_prefix)
            plan[f"{s_:g}"] = res.status
    return {"run": run.name, "first_all_m": first_all, "first_any_m": first_any, "plan": plan, "rows": rows}


def low_overlap(run: Path, prop_index: int) -> dict:
    """The return-home overlap with a low prop (2026-10-28 lead, priority): around each unpermitted tick, the truth pose
    (frames.jsonl) and the control pose (samples) against the prop -- the hull the truth evaluator checks, the body and
    the fork blades (what can touch a 0.144 m block) -- and the control pose's distance from the planned return path."""
    from insertion_geometry import read_chassis_reference_m, read_fork_blades_m

    record = json.loads((run / "run/result.json").read_text())
    args_ = record.get("arguments") or {}
    urdf = Path(args_.get("forklift_urdf") or "")
    rect = Rectangle(**record["scenario"]["props"][prop_index]["rectangle"])
    g = record["geometry"]
    hull = Footprint(**g["unloaded_footprint"])
    tip, offset = read_chassis_reference_m(urdf)
    body = Footprint(float(g["axle_to_fork_tip_m"]) - float(g["carriage_limit_m"]), hull.rear_m, hull.half_width_m)
    blades = [(x0 + abs(offset), x1 + abs(offset), y0, y1) for x0, x1, y0, y1 in read_fork_blades_m(urdf)]
    blade_fps = [(Footprint((x1 - x0) / 2, (x1 - x0) / 2, (y1 - y0) / 2), (y0 + y1) / 2, (x0 + x1) / 2)
                 for x0, x1, y0, y1 in blades]

    def parts_distance(pose):
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        d = [footprint_rect_distance(pose, body, rect)]
        for fp_, lat, lon in blade_fps:
            d.append(footprint_rect_distance((pose[0] + lon * c - lat * s_, pose[1] + lon * s_ + lat * c, pose[2]),
                                             fp_, rect))
        return min(d)

    u = record["obstacle_layer"]["unpermitted"]
    if not u:
        return {"run": run.name, "error": "no unpermitted ticks"}
    t0, t1 = float(u[0]["time_s"]), float(u[-1]["time_s"])
    truth = []
    for line in (run / "run/frames.jsonl").read_text().splitlines():
        f = json.loads(line)
        ts = float(f["simulation_time_s"])
        if t0 - 2.0 <= ts <= t1 + 2.0:
            w, qx, qy, qz = f["base_orientation_wxyz"]
            yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            bx, by = f["base_position_m"][:2]
            rear = (bx - abs(offset) * math.cos(yaw), by - abs(offset) * math.sin(yaw), yaw)
            truth.append((ts, rear))
    path = np.asarray((record.get("paths") or {}).get("return_home", {}).get("poses") or [], dtype=float)
    ctrl = [(float(x["time_s"]), tuple(x["rear_pose"])) for x in record["samples"]
            if t0 - 2.0 <= float(x["time_s"]) <= t1 + 2.0]
    out = {"run": run.name, "prop": record["scenario"]["props"][prop_index]["asset"], "rect": [round(v, 3) for v in
           (rect.x_m, rect.y_m, rect.length_m, rect.width_m, rect.yaw_rad)], "unpermitted_t": [round(t0, 2), round(t1, 2)],
           "truth_min": {"hull": min((footprint_rect_distance(p, hull, rect), t) for t, p in truth),
                         "parts": min((parts_distance(p), t) for t, p in truth)},
           "ctrl_min": {"hull": min((footprint_rect_distance(p, hull, rect), t) for t, p in ctrl),
                        "parts": min((parts_distance(p), t) for t, p in ctrl)},
           "max_cross_track_m": None}
    if len(path):
        dev = [float(np.min(np.hypot(path[:, 0] - p[0], path[:, 1] - p[1]))) for _, p in ctrl]
        out["max_cross_track_m"] = round(max(dev), 3)
        out["path_hull_min"] = round(min(footprint_rect_distance(tuple(p), hull, rect) for p in path), 3)
        out["path_parts_min"] = round(min(parts_distance(tuple(p)) for p in path), 3)
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--alternatives", action="store_true",
                        help="the lead's comparison: (b) tall props by their shape, (c) the extract carried on")
    parser.add_argument("--extract-extension", action="store_true",
                        help="(c) under the extract's own rules: scene sweep and the start permit at the end")
    parser.add_argument("--low-overlap", type=int, default=None, metavar="PROP",
                        help="the return-home overlap with a low prop: truth/control hull and parts distances")
    parser.add_argument("--escape-replay", action="store_true",
                        help="re-check an installed presteered escape that stopped (C4') instead of the stand")
    args = parser.parse_args(argv)
    out = []
    if args.low_overlap is not None:
        for run in args.runs:
            try:
                r = low_overlap(run, args.low_overlap)
            except Exception as exc:  # noqa: BLE001
                r = {"run": run.name, "error": repr(exc)}
            print(json.dumps(r, default=str))
        return 0
    if args.extract_extension:
        for run in args.runs:
            try:
                r = extract_extension(run)
            except Exception as exc:  # noqa: BLE001
                r = {"run": run.name, "error": repr(exc)}
            out.append(r)
            print(json.dumps({k: v for k, v in r.items() if k != "rows"}, default=str))
        if args.json:
            args.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
        return 0
    if args.alternatives:
        for run in args.runs:
            try:
                r = alternatives(run)
            except Exception as exc:  # noqa: BLE001
                r = {"run": run.name, "error": repr(exc)}
            out.append(r)
            print(json.dumps(r, default=str))
        if args.json:
            args.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
        return 0
    if args.escape_replay:
        for run in args.runs:
            try:
                r = escape_replay(run)
            except Exception as exc:  # noqa: BLE001
                r = {"run": run.name, "error": repr(exc)}
            out.append(r)
            print(json.dumps(r, indent=1, default=str))
        if args.json:
            args.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
        return 0
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
        if r.get("presteer"):
            ps = r["presteer"]
            print(f"  presteer replay: shift {[round(v, 5) for v in ps['shift']]} kappa_measured {ps['kappa_measured']:.4f}")
            for k in ("no_move", "anchor_first", "anchor_relatched"):
                v = ps[k]
                print(f"    {k:18s} qualifying {v.get('qualifying')}/8 installs {v.get('installs')} {v.get('candidates')}")
        print(f"  full escape_plan (record escape at t {r['record_escape_t']}):")
        for name, v in r["full"].items():
            if "error" in v:
                print(f"    {name:22s} {v['error']}")
                continue
            print(f"    {name:22s} qualifying {v['qualifying']}/8 chosen {v['chosen']} permit {v['permit']} "
                  f"installs {v['installs']} extra {v['extra_rects']} released {v['released_rects']} {v['candidates']}")
    if args.json:
        args.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
