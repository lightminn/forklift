"""The runner's start relocalization phase (docs/plans/2026-10-29-start-relocalization.md 1.2 steps 3-10), Isaac-free.

Before the SLAM link opens: the truck settles where it was placed, its first stationary scans decide whether the start
is physically valid (the only use of the simulator's truth here, ``scan_capture.scan_set``), then each attempt's five
scans go to ``relocalize`` on a worker thread while the main thread keeps stepping the stopped truck. All attempts
share one session deadline (``StartRelocalizer``). A confirmed estimate is written for the supervisor
(``reloc_pose.json``), which starts the SLAM bridge at it and answers ``bridge_go``. Unattended (B-P2b), a start that
needs an operator ends the phase; the operator's pick comes with the console (B-P2c).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import scan_capture

from forklift_core.localization import global_match as G
from forklift_core.localization import start_reloc as R


@dataclass
class RelocInputs:
    engine: scan_capture.Engine  # step/pose/cast/noise of the stopped truck (teleport unused)
    target: tuple  # the placed start (world base): physical validity only
    grid: object
    index: G.MapIndex
    match_cfg: G.MatchConfig
    reloc_cfg: R.RelocConfig
    angle_min_rad: float
    angle_increment_rad: float
    epoch_dir: Path
    settle_steps: int = 120  # 1 s of physics before the first scan (plan 1.2 step 3)
    bridge_go_timeout_s: float = 120.0
    pace_s: float = 0.005  # the main thread's step period while the worker matches
    unattended: bool = True
    clock: object = time.monotonic
    cancel: threading.Event = field(default_factory=threading.Event)


def _points(inputs: RelocInputs, sent) -> object:
    scans = [((0.0, 0.0, 0.0), ranges, inputs.angle_min_rad, inputs.angle_increment_rad) for ranges in sent]
    return G.stack_scans(scans, range_min_m=inputs.engine.range_min_m, range_max_m=inputs.engine.range_max_m,
                         match_range_m=inputs.match_cfg.match_range_m)


def _attempt(inputs: RelocInputs, relocalizer: R.StartRelocalizer, points) -> tuple:
    """One relocalize() on a worker; the stopped truck keeps stepping meanwhile. Returns (match, decision, wall_s)."""
    box = {}

    def work():
        t0 = time.monotonic()
        box["result"] = G.relocalize(inputs.grid, points, inputs.match_cfg, e_min=inputs.reloc_cfg.e_min,
                                     r_max=inputs.reloc_cfg.r_max, r_struct=inputs.reloc_cfg.r_struct,
                                     index=inputs.index, clock=inputs.clock, deadline_at=relocalizer.deadline_at,
                                     cancel=inputs.cancel.is_set)
        box["wall_s"] = time.monotonic() - t0

    worker = threading.Thread(target=work, name="start-relocalize", daemon=True)
    worker.start()
    while worker.is_alive():
        inputs.engine.step()
        worker.join(timeout=inputs.pace_s)
    if "result" not in box:
        raise RuntimeError("the relocalization worker ended without a result")
    return (*box["result"], box["wall_s"])


def _candidate(c) -> dict:
    return {"base": list(R.base_from_laser(c.pose)), "score": c.search_score,
            "endpoint_fraction": c.endpoint_fraction}


def _wait_bridge_go(inputs: RelocInputs):
    until = time.monotonic() + inputs.bridge_go_timeout_s
    while time.monotonic() < until and not inputs.cancel.is_set():
        go = R.read_bridge_go(inputs.epoch_dir)
        if go is not None:
            return go
        inputs.engine.step()
        time.sleep(max(inputs.pace_s, 0.001))
    return None


def run(inputs: RelocInputs) -> dict:
    """status: chosen (with ``bridge_go``) | invalid | needs_operator | failed | bridge_timeout | cancelled. The
    result is plain JSON: the runner records it as is."""
    out = {"status": None, "reason": "", "estimate": None, "attempts": [], "truth": None, "bridge_go": None,
           "generation": None, "shown": []}
    if inputs.cancel.is_set():
        out["status"] = "cancelled"
        return out
    relocalizer = R.StartRelocalizer(inputs.reloc_cfg, inputs.clock)
    for _ in range(inputs.settle_steps):  # settle where it was placed (plan 1.2 step 3)
        inputs.engine.step()
    relocalizer.start()  # the first scan is being collected: the session deadline starts (plan 1.2 step 6)
    first = scan_capture.scan_set(inputs.engine, inputs.target, _own_noise, settle_steps=0)
    out["truth"] = list(first["base_truth"])
    if not first["valid"]:  # judged before any matching (plan 1.2 step 4)
        out["status"], out["reason"] = "invalid", first["reason"]
        return out
    sent, step = first["sent"], "rescan"
    while step == "rescan":
        if inputs.cancel.is_set():
            relocalizer.cancel()
            out["status"] = "cancelled"
            return out
        match, decision, wall_s = _attempt(inputs, relocalizer, _points(inputs, sent))
        step = relocalizer.record(match, decision)
        out["attempts"].append({"reason": decision.reason, "ratio": decision.ratio, "status": match.status,
                                "work": int(match.work), "wall_s": round(wall_s, 3),
                                "top": _candidate(match.candidates[0]) if match.candidates else None})
        if step == "rescan":  # a static rescan: the truck stays where it is
            sent = scan_capture.scan_set(inputs.engine, inputs.target, _own_noise, settle_steps=0)["sent"]
    out["generation"] = relocalizer.generation
    if step != "chosen":
        out["status"] = {"awaiting_pick": "needs_operator", "cancelled": "cancelled"}.get(step, "failed")
        out["shown"] = [_candidate(c) for c in relocalizer.shown]
        return out
    est = relocalizer.estimate
    out["estimate"] = list(est.base)
    R.write_reloc_pose(inputs.epoch_dir, est, {"attempts": out["attempts"]})
    go = _wait_bridge_go(inputs)
    if go is None:
        out["status"] = "cancelled" if inputs.cancel.is_set() else "bridge_timeout"
        return out
    out["status"], out["bridge_go"] = "chosen", go
    return out


def _own_noise(scan_index: int):
    """None: the obstacle layer's own generator draws the scan noise, as on the live send path."""
    return None


__all__ = ["RelocInputs", "run"]
