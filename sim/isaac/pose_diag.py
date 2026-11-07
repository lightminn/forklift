"""Record-only diagnostics of the unconfirmed position check (the lead's decision after the second B, 2026-11-07, item 3):
nothing here decides anything the run did not decide before.

- ``walk_rear``: design 3's rear check (``pose_rear_blocked`` in run_transport) as it always ran -- for each curvature of
  the straight reverse's bundle, in order, the union zones' first contact, then the obstacle layer's verified distance;
  the first short one blocks and ends the walk -- returning, beside the verdict, which branch blocked, at which
  curvature, the required and verified distances, the zone, the obstacle's blocked kind and cells, and every later
  branch marked unchecked.
- ``layers_arrays``: a planning grid's layers (``ObstacleLayer.planner_layers``) as plain arrays for an npz.
- ``save_premove``: item 3b, the state before the truck moves (after the start check's result, before the reverse):
  the applied correction C, the odometry O, the grid's correction, and the observe plan's grid by layers, built on a
  copy of the obstacle layer (run_transport ``grid_kwargs`` scratch).
- ``save_prepare``: item 3c, each mission preparation's input fixed when it was submitted -- the worker payload as
  pickled then (world with its obstacles and bounds, waypoints = every target candidate, config, geometry,
  pickup_bounds, grid_kwargs with pickup_obstacle and the occupancy) and the mission inputs -- with the grid by layers,
  the start pose (rear; base by the caller's offset offline), C and O; written for the missions whose result is a
  preparation failure.
"""

from __future__ import annotations

import math
import pickle

import numpy as np

UNCHECKED = "unchecked"


def _num(v):
    return None if v is None else (float(v) if math.isfinite(float(v)) else ("inf" if v > 0 else "-inf"))


def walk_rear(kappas, length_m: float, zones, zone_contact, obstacle_check) -> tuple[bool, list]:
    """(blocked, arcs): ``zone_contact(kappa) -> (distance, zone_id)`` is asked only when there are zones,
    ``obstacle_check(kappa) -> Check`` only when the zones let the arc through, in this order per curvature, the
    first short distance ending the walk (the check's own early return). ``arcs`` holds one entry per curvature;
    those after the blocking one are ``unchecked``."""
    arcs = []
    kappas = list(kappas)
    for i, kappa in enumerate(kappas):
        arc = {"kappa": float(kappa), "zone": UNCHECKED if zones else "no_zones", "obstacle": UNCHECKED}
        arcs.append(arc)
        if zones:
            d, which = zone_contact(kappa)
            arc["zone"] = {"contact_m": _num(d), "zone_id": which}
            if d < length_m:
                arcs += [{"kappa": float(k), "zone": UNCHECKED, "obstacle": UNCHECKED} for k in kappas[i + 1:]]
                return True, arcs
        check = obstacle_check(kappa)
        arc["obstacle"] = {"verified_m": _num(check.verified_m), "blocked": check.blocked,
                           "cells": [list(map(int, c)) for c in (check.blocked_cells or ())]}
        if check.verified_m < length_m:
            arcs += [{"kappa": float(k), "zone": UNCHECKED, "obstacle": UNCHECKED} for k in kappas[i + 1:]]
            return True, arcs
    return False, arcs


def blocking_branch(arcs, length_m: float) -> tuple:
    """(branch, kappa) where the walk stopped: the last checked arc's short distance; (None, None) if clear."""
    for arc in arcs:
        z, o = arc["zone"], arc["obstacle"]
        if isinstance(z, dict) and _short(z["contact_m"], length_m):
            return "zone", arc["kappa"]
        if isinstance(o, dict) and _short(o["verified_m"], length_m):
            return "obstacle", arc["kappa"]
    return None, None


def _short(d, length_m: float) -> bool:
    if d in ("inf", None):
        return False
    if d == "-inf":
        return True
    return float(d) < length_m


def layers_arrays(layers: dict, prefix: str = "") -> dict:
    """``ObstacleLayer.planner_layers``'s output as npz arrays (the occupancy grid apart: its cells and frame)."""
    occ = layers.get("occupancy")
    if occ is None:  # no grid plan (e.g. --layout scene): nothing but the record says so
        return {}
    out = {f"{prefix}occupied": np.asarray(occ.occupied, bool),
           f"{prefix}origin": np.asarray([occ.origin_x_m, occ.origin_y_m], float),
           f"{prefix}resolution_m": np.asarray(occ.resolution_m, float)}
    for name in ("live", "memory_observed", "slam_layer", "own_left_out"):
        if layers.get(name) is not None:
            out[f"{prefix}{name}"] = np.asarray(layers[name], bool)
    for name in ("snapshot_stamp_s", "correction_version", "use_slam"):
        if layers.get(name) is not None:
            out[f"{prefix}{name}"] = np.asarray(layers[name])
    return out


def _meta(cap: dict) -> dict:
    out = {k: v for k, v in cap.items() if k not in ("layers", "payload", "inputs")}
    out["grid"] = "grid_plan" if (cap.get("layers") or {}).get("occupancy") is not None else "no_grid_plan"
    return out


def save_premove(path, premove: dict) -> None:
    """The pre-move state: its layers as arrays and the rest (t, C, O, the grid's correction, the tracker's mode) as a
    JSON string in ``meta``."""
    import json

    np.savez_compressed(path, meta=json.dumps(_meta(premove)), **layers_arrays(premove["layers"]))


def save_prepare(out_dir, captures, results) -> list:
    """Each capture of a mission whose result reason starts with ``prepare:`` (every capture when the results could not
    be read): ``prepare_input_<k>.pkl`` (``{"meta", "payload", "inputs"}``, the payload and inputs as the pickled
    bytes taken at the submit) and ``prepare_input_<k>.npz`` (the layers). Returns what was written."""
    import json
    from pathlib import Path

    failed = None if results is None else {r.get("mission_id") for r in results
                                           if str(r.get("reason") or "").startswith("prepare:")}
    out = []
    for k, cap in enumerate(captures, 1):
        if failed is not None and cap.get("mission_id") not in failed:
            continue
        stem = Path(out_dir) / f"prepare_input_{k}"
        with open(f"{stem}.pkl", "wb") as fh:
            pickle.dump({"meta": _meta(cap), "payload": cap["payload"], "inputs": cap.get("inputs")}, fh)
        np.savez_compressed(f"{stem}.npz", meta=json.dumps(_meta(cap)), **layers_arrays(cap["layers"]))
        out.append({"file": stem.name, "mission_id": cap.get("mission_id"), "t": cap.get("t"),
                    "layers_match": cap.get("layers_match"), "results_read": results is not None})
    return out


__all__ = ["UNCHECKED", "blocking_branch", "layers_arrays", "save_premove", "save_prepare", "walk_rear"]
