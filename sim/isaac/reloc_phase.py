"""The runner's start relocalization phase (docs/plans/2026-10-29-start-relocalization.md 1.2 steps 3-10), Isaac-free.

Before the SLAM link opens: the truck settles where it was placed, its first stationary scans decide whether the start
is physically valid (the only use of the simulator's truth here, ``scan_capture.scan_set``), then each attempt's five
scans go to ``relocalize`` on a worker thread while the main thread keeps stepping the stopped truck. All attempts
share one session deadline (``StartRelocalizer``). A confirmed estimate is written for the supervisor
(``reloc_pose.json``), which starts the SLAM bridge at it and answers ``bridge_go``. Unattended (B-P2b), a start that
needs an operator ends the phase; the operator's pick comes with the console (B-P2c).
"""

from __future__ import annotations

import hashlib
import json
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import scan_capture

from forklift_core.localization import global_match as G
from forklift_core.localization import local_match as LM
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
    noise_for: object = lambda attempt, scan: None  # noqa: E731 -- the generator of an attempt's scan (None: the engine's)
    # B-P2c: the early console (reloc_console.RelocConsole), serviced at every step and wait; None: no operator
    console: object = None
    operator_deadline_at: float | None = None  # the operator's whole budget (clock time); None: none
    # F1 (plan 2026-10-31-operator-placed-start 1.3): the local re-match's window, R_AMB and deadline
    place_cfg: LM.PlaceConfig = field(default_factory=LM.PlaceConfig)


def _points(inputs: RelocInputs, sent) -> object:
    scans = [((0.0, 0.0, 0.0), ranges, inputs.angle_min_rad, inputs.angle_increment_rad) for ranges in sent]
    return G.stack_scans(scans, range_min_m=inputs.engine.range_min_m, range_max_m=inputs.engine.range_max_m,
                         match_range_m=inputs.match_cfg.match_range_m)


def _attempt(inputs: RelocInputs, relocalizer: R.StartRelocalizer, points, step) -> tuple:
    """One relocalize() on a worker; the stopped truck keeps stepping (and the console is serviced) meanwhile. The
    worker stops at a session cancel or when a STOP interrupted the attempt. Returns (match, decision, wall_s)."""
    box = {}
    generation = relocalizer.generation

    def cancelled() -> bool:
        return inputs.cancel.is_set() or relocalizer.generation != generation

    def work():
        t0 = time.monotonic()
        box["result"] = G.relocalize(inputs.grid, points, inputs.match_cfg, e_min=inputs.reloc_cfg.e_min,
                                     r_max=inputs.reloc_cfg.r_max, r_struct=inputs.reloc_cfg.r_struct,
                                     index=inputs.index, clock=inputs.clock, deadline_at=relocalizer.deadline_at,
                                     cancel=cancelled)
        box["wall_s"] = time.monotonic() - t0

    worker = threading.Thread(target=work, name="start-relocalize", daemon=True)
    worker.start()
    while worker.is_alive():
        step("matching")
        worker.join(timeout=inputs.pace_s)
    if "result" not in box:
        raise RuntimeError("the relocalization worker ended without a result")
    return (*box["result"], box["wall_s"])


def _candidate(c) -> dict:
    return {"base": list(R.base_from_laser(c.pose)), "score": c.search_score,
            "endpoint_fraction": c.endpoint_fraction}


def _wait_bridge_go(inputs: RelocInputs, step):
    until = time.monotonic() + inputs.bridge_go_timeout_s
    while time.monotonic() < until and not inputs.cancel.is_set():
        go = R.read_bridge_go(inputs.epoch_dir)
        if go is not None:
            return go
        step("bridge")
        time.sleep(max(inputs.pace_s, 0.001))
    return None


WAITING = ("awaiting_pick", "reloc_failed", "stopped", "place_preview")
PREVIEW_POINTS = 360  # at most this many scan endpoints drawn over the map for the operator's preview


def _place(inputs: RelocInputs, relocalizer: R.StartRelocalizer, points, step, deadline_at: float) -> tuple:
    """One local re-match on a worker (as ``_attempt``): cancelled at a session cancel or once the relocalizer left
    this placement (a STOP); ``deadline_at`` is T_LOC from the placement's scan start (clock time). Returns
    (LocalResult, wall_s)."""
    box = {}
    places = relocalizer.places
    pc = inputs.place_cfg

    def cancelled() -> bool:
        return inputs.cancel.is_set() or relocalizer.state != "placing" or relocalizer.places != places

    def work():
        w0 = time.monotonic()
        window = LM.Window(base=relocalizer.placed, radius_m=pc.radius_m, yaw_rad=pc.yaw_rad, edge_m=pc.edge_m,
                           edge_rad=pc.edge_rad)
        box["result"] = LM.local_match(inputs.grid, points, window, inputs.match_cfg, e_min=inputs.reloc_cfg.e_min,
                                       r_amb=pc.r_amb, index=inputs.index, clock=inputs.clock,
                                       deadline=deadline_at, cancel=cancelled)
        box["wall_s"] = time.monotonic() - w0

    worker = threading.Thread(target=work, name="start-place", daemon=True)
    worker.start()
    while worker.is_alive():
        step("placing")
        worker.join(timeout=inputs.pace_s)
    if "result" not in box:
        raise RuntimeError("the placement worker ended without a result")
    return box["result"], box["wall_s"]


def _preview_points(points, base) -> list:
    """The scan's endpoints at the previewed base pose (map), at most PREVIEW_POINTS, for the operator's overlay."""
    import numpy as np  # noqa: PLC0415

    pts = np.asarray(points, float).reshape(-1, 2)
    if len(pts) > PREVIEW_POINTS:
        pts = pts[np.linspace(0, len(pts) - 1, PREVIEW_POINTS).astype(int)]
    lx, ly, yaw = R.laser_from_base(base)
    c, s_ = np.cos(yaw), np.sin(yaw)
    return [[round(float(lx + c * x - s_ * y), 3), round(float(ly + s_ * x + c * y), 3)] for x, y in pts]


def run(inputs: RelocInputs) -> dict:
    """status: chosen (with ``bridge_go``) | invalid (the first scan set) | unsteady (a later one) | needs_operator |
    failed | operator_timeout | bridge_timeout | cancelled. The result is plain JSON: the runner records it as is. With a console the phase waits in awaiting_pick /
    reloc_failed / stopped for the operator's pick or retry (unless unattended); the console's exception (the
    operator's session end) leaves through here with the matcher cancelled."""
    try:
        return _run(inputs)
    except BaseException:
        inputs.cancel.set()  # a worker still matching stops at its next check
        raise


def _run(inputs: RelocInputs) -> dict:
    out = {"status": None, "reason": "", "estimate": None, "attempts": [], "truth": None, "bridge_go": None,
           "generation": None, "shown": [], "places": []}
    if inputs.cancel.is_set():
        out["status"] = "cancelled"
        return out
    console = inputs.console
    unattended = inputs.unattended or console is None
    engine_step = inputs.engine.step

    def step(stage: str) -> None:
        if console is not None:
            console.gate(stage)  # a STOP queued meanwhile is taken before the step (Codex B-P2c impl review P2)
        engine_step()
        if console is not None:
            console.service(stage)

    engine = replace(inputs.engine, step=lambda: step("scan"))
    relocalizer = R.StartRelocalizer(inputs.reloc_cfg, inputs.clock)
    if console is not None:
        console.attach(relocalizer)
        console.note(operator_deadline_at=inputs.operator_deadline_at)
    relocalizer.expires_at = inputs.operator_deadline_at  # no pick or retry after it, in the handler too
    g = inputs.grid  # F1: a placement off the survey map is refused (Codex F1 implementation review P2)
    h_, w_ = g.data.shape
    relocalizer.place_bounds = (g.origin[0], g.origin[1], g.origin[0] + w_ * g.resolution_m,
                                g.origin[1] + h_ * g.resolution_m)
    for _ in range(inputs.settle_steps):  # settle where it was placed (plan 1.2 step 3)
        step("settle")
    scan_key = 0  # every scan set of the session its own noise (a retry never reuses one, Codex B-P2c design P3)
    while True:
        if inputs.cancel.is_set():
            relocalizer.cancel()
            out["status"] = "cancelled"
            return out
        state = relocalizer.state
        if state == "chosen":
            break  # a pick taken before the operator's deadline (the handler refuses one after it)
        if inputs.operator_deadline_at is not None and inputs.clock() > inputs.operator_deadline_at:
            relocalizer.cancel()  # nothing can be chosen from here on
            out["status"] = "operator_timeout"
            break
        if state == "relocalizing":
            relocalizer.start()  # the generation's first scan: its session deadline (plan 1.2 step 6)
            generation = relocalizer.generation  # the scans and the match belong to it (Codex impl round 2 P2)
            got = scan_capture.scan_set(engine, inputs.target, lambda s, k=scan_key: inputs.noise_for(k, s),
                                        settle_steps=0)
            scan_key += 1
            if out["truth"] is None:
                out["truth"] = list(got["base_truth"])
            if not got["valid"]:
                # the session's input check is its first scan set only (plan 1.2 step 3, 4.0: fixed before any
                # matching); a later set finding the truck moved fails a valid session (Codex B-P2d freeze review P2)
                out["status"] = "invalid" if scan_key == 1 else "unsteady"
                out["reason"] = got["reason"]
                return out
            if relocalizer.state != "relocalizing" or relocalizer.generation != generation:
                continue  # a STOP (and maybe a retry) during the scans: they go, the new generation scans anew
            match, decision, wall_s = _attempt(inputs, relocalizer, _points(inputs, got["sent"]), step)
            if console is not None:
                console.service("matched")  # the STOPs queued meanwhile, before the result counts
            # from the generation's first scan to this return: the shared session deadline's own time axis (B-P2d)
            stage_s = (None if relocalizer.deadline_at is None  # a STOP and retry reset it: interrupted below
                       else inputs.clock() - (relocalizer.deadline_at - inputs.reloc_cfg.deadline_s))
            record = {"reason": decision.reason, "ratio": decision.ratio, "status": match.status,
                      "work": int(match.work), "wall_s": round(wall_s, 3),
                      "stage_s": None if stage_s is None else round(stage_s, 3),
                      "top": _candidate(match.candidates[0]) if match.candidates else None,
                      # plan 7 F1: every candidate of the matcher (not only those shown), for B-P2d's record
                      "candidates": [_candidate(c) for c in match.candidates]}
            if relocalizer.state != "relocalizing" or relocalizer.generation != generation:
                # a STOP (and maybe a retry since) came while it matched: the result is its old generation's
                out["attempts"].append({**record, "reason": "interrupted", "matcher_reason": decision.reason})
                continue
            relocalizer.record(match, decision)
            if decision.confirmed and relocalizer.state != "chosen":
                record["reason"] = "deadline"  # confirmed too late: the session deadline refused it (no confirmation)
            out["attempts"].append(record)
            if console is not None:
                console.note(last_reason=decision.reason)
            continue
        if state == "placing":
            # F1: fresh scans where the truck stands (it may have been pushed while the operator looked), the stillness
            # check again, then the local re-match around the placed pose (plan 1.3)
            places = relocalizer.places
            t0 = inputs.clock()
            deadline_at = t0 + inputs.place_cfg.deadline_s  # T_LOC from the scan start (Codex F1 impl review P2)
            got = scan_capture.scan_set(engine, inputs.target, lambda s, k=scan_key: inputs.noise_for(k, s),
                                        settle_steps=0)
            scan_key += 1
            record = {"click": list(relocalizer.placed), "places": places}
            if not got["valid"]:
                out["places"].append({**record, "reason": "unsteady", "matcher_reason": got["reason"]})
                out["status"], out["reason"] = "unsteady", got["reason"]
                return out
            if relocalizer.state != "placing" or relocalizer.places != places:
                out["places"].append({**record, "reason": "interrupted"})
                continue
            points = _points(inputs, got["sent"])
            res, wall_s = _place(inputs, relocalizer, points, step, deadline_at)
            if res.confirmed and inputs.clock() > deadline_at:  # nothing confirmed past T_LOC
                res = replace(res, confirmed=False, reason="timeout", status="timeout")
            if console is not None:
                console.service("placed")
            record.update(reason=res.reason, status=res.status, base=None if res.base is None else list(res.base),
                          amb_ratio=res.amb_ratio, endpoint_fraction=res.endpoint_fraction, leaves=res.leaves,
                          work=int(res.work), wall_s=round(wall_s, 3), stage_s=round(inputs.clock() - t0, 3))
            if relocalizer.state != "placing" or relocalizer.places != places:
                out["places"].append({**record, "reason": "interrupted", "matcher_reason": res.reason})
                continue
            relocalizer.placed_result(res.confirmed, res.base, res.reason)
            out["places"].append(record)
            if console is not None:
                console.note(place_reason=None if res.confirmed else res.reason,
                             preview_points=_preview_points(points, res.base) if res.confirmed else None)
            continue
        if state in WAITING:
            if unattended:
                break
            step(state)  # the operator's pick or retry comes through the console
            time.sleep(max(inputs.pace_s, 0.001))
            continue
        out["status"] = "cancelled"
        return out
    out["generation"] = relocalizer.generation
    if relocalizer.state != "chosen":
        if out["status"] is None:
            out["status"] = {"awaiting_pick": "needs_operator", "stopped": "needs_operator"}.get(
                relocalizer.state, "failed")
        out["shown"] = [_candidate(c) for c in relocalizer.shown]
        return out
    est = relocalizer.estimate
    out["estimate"] = list(est.base)
    out["source"] = est.source
    R.write_reloc_pose(inputs.epoch_dir, est, {"attempts": out["attempts"]})
    if console is not None:
        console.localizing()
    go = _wait_bridge_go(inputs, step)
    if go is None:
        out["status"] = "cancelled" if inputs.cancel.is_set() else "bridge_timeout"
        return out
    out["status"], out["bridge_go"] = "chosen", go
    return out


def failure_reason(out: dict) -> str | None:
    """The session failure an ending other than ``chosen`` is (plan 1.2's reasons; ``reloc_needs_operator`` is the
    unattended B-P2b ending of what the console's pick resolves from B-P2c on)."""
    status = out["status"]
    if status == "chosen":
        return None
    if status == "invalid":
        return f"reloc_invalid_input: {out.get('reason')}"
    if status == "unsteady":
        return f"reloc_unsteady: {out.get('reason')} after {len(out.get('attempts') or ())} attempt(s)"
    if status == "needs_operator":
        return f"reloc_needs_operator: {len(out.get('shown') or ())} candidates, no operator (unattended)"
    if status == "failed":
        return "reloc_failed: no candidate an operator could pick"
    if status == "bridge_timeout":
        return "reloc_bridge_timeout: no bridge_go within the wait"
    if status == "operator_timeout":
        return "reloc_operator_timeout: no pick or retry within the operator's time"
    return "reloc_cancelled"


MATCH_CONFIG = "docs/validation/assets/b-p1b/match_config.json"
DEV_CHOICE = "docs/validation/assets/b-p1b/dev_choice.json"
PLACE_CONFIG = "docs/validation/assets/f1/place_config.json"  # F1's first freeze (window, R_AMB, deadline)


def frozen_settings(root: Path) -> tuple:
    """B-P1b's frozen search budget and choice (plan 1.2 step 5), with the files' hashes for the record."""
    files = {"match_config": Path(root) / MATCH_CONFIG, "dev_choice": Path(root) / DEV_CHOICE}
    raw = {name: path.read_bytes() for name, path in files.items()}
    cfg = G.MatchConfig(**json.loads(raw["match_config"]))
    choice = json.loads(raw["dev_choice"])["choice"]
    reloc = R.RelocConfig(e_min=float(choice["e_min"]), r_max=float(choice["r_max"]),
                          r_struct=float(choice["r_struct"]))
    record = {name: {"file": str(files[name].relative_to(root)), "sha256": hashlib.sha256(data).hexdigest()}
              for name, data in raw.items()}
    return cfg, reloc, record


def place_settings(root: Path) -> tuple:
    """F1's placement settings (plan 2026-10-31-operator-placed-start 2, 4.6): the frozen file with its hash, or the
    plan's provisional values (R_AMB not yet chosen) marked as such."""
    path = Path(root) / PLACE_CONFIG
    if not path.exists():
        return LM.PlaceConfig(), {"file": None, "provisional": True}
    data = path.read_bytes()
    return LM.PlaceConfig(**json.loads(data)), {"file": PLACE_CONFIG, "sha256": hashlib.sha256(data).hexdigest()}


__all__ = ["DEV_CHOICE", "MATCH_CONFIG", "PLACE_CONFIG", "RelocInputs", "failure_reason", "frozen_settings",
           "place_settings", "run"]
