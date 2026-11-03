"""The runner's side of the unconfirmed position check (docs/plans/2026-11-02-unconfirmed-runner.md 2.2, 3.3), Isaac-free.

The runner hands in every scan it sent the SLAM link (``on_scan``) and, every control tick, the odometry (base_link),
the applied correction and the stand (``tick``); this module keeps the live scan sets, runs the start check once the
SLAM tracks, drives the re-match generation (the frozen relocalizer's attempts, then the decision) through a worker,
and gives the pallet zones in odometry coordinates while the start is not confirmed. The judgement itself is
``forklift_core.localization.pose_check.PoseCheck``; the heavy work (``run_job``) runs in a worker **process**
(``ProcessWorker``, off the control thread's GIL, plan 3.3), or inline in tests.
"""

from __future__ import annotations

import math
import multiprocessing
import queue
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from forklift_core.localization import global_match as G
from forklift_core.localization import pose_check as P
from forklift_core.localization import start_reloc as R
from forklift_core.localization import twin_check as T

SCANS = 5  # scan_capture.SCANS: one set, as the start relocalization takes it
DEADLINE_S = 30.0  # the start check's and a re-match generation's deadline (the matcher's, plan 3.2)
ZONE_BIN_M = 0.05  # the net displacement's bin: zones grown at its upper end, fixed within it (plan 3.3)


# --- the worker's jobs ------------------------------------------------------------------------------------------------
class JobContext:
    """What every job needs, built once in the worker: the survey grid, its indices and the frozen settings."""

    def __init__(self, grid, cfg: G.MatchConfig, reloc_cfg: R.RelocConfig, bounds: P.Bounds):
        self.grid, self.cfg, self.reloc_cfg, self.bounds = grid, cfg, reloc_cfg, bounds
        self.index = G.MapIndex(grid, cfg)
        self.cons = T.ConservativeIndex(grid, cfg)


def run_job(kind: str, p: dict, ctx: JobContext) -> dict:
    kw = {"index": ctx.index, "clock": time.monotonic}
    e_min = ctx.reloc_cfg.e_min
    if kind == "start_check":
        o0, o_scan = tuple(p["o0"]), tuple(p["o_scan"])
        t0 = time.monotonic()
        out = T.start_check(ctx.grid, p["points"], tuple(p["b_scan"]), pool=p["pool"], gen_points=p["gen_points"],
                            carry=lambda q: T.moved(q, o0, o_scan), cfg=ctx.cfg, e_min=e_min, cons=ctx.cons,
                            deadline=p["deadline_at"], **kw)
        res = out.residual
        return {"confirmed": bool(res.confirmed), "base": None if res.base is None else list(res.base),
                "reasons": list(res.reasons), "hyps": [[list(q), src] for q, src in out.hyps],
                "statuses": out.statuses, "s_b": res.s_b, "a_b": res.a_b, "impossible": len(res.impossible),
                "competitors": len(res.competitors), "work": int(res.work), "wall_s": time.monotonic() - t0}
    if kind == "relocalize":
        rc = ctx.reloc_cfg
        match, decision = G.relocalize(ctx.grid, p["points"], ctx.cfg, e_min=rc.e_min, r_max=rc.r_max,
                                       r_struct=rc.r_struct, deadline_at=p["deadline_at"], **kw)
        return {"match": match, "decision": decision}
    if kind == "rematch":
        b, o0, ot, g = tuple(p["b"]), tuple(p["o0"]), tuple(p["ot"]), tuple(p["g"])
        hyps = [(tuple(q), src) for q, src in p["hyps"]]
        bounds = ctx.bounds
        if p["phase"] == "unconfirmed":
            out = T.rematch_decision(ctx.grid, p["points"], b, hyps, o0, ot, decision=p["decision"], g=g,
                                     cfg=ctx.cfg, e_min=e_min, dp=bounds.dp, dth=bounds.dth, cons=ctx.cons,
                                     deadline=p["deadline_at"], **kw)
            return {"outcome": out.outcome, "matched": out.matched, "agree_m": out.agree_m,
                    "agree_rad": out.agree_rad, "g": list(g),
                    "residual": None if out.residual is None else list(out.residual.reasons)}
        # confirmed (plan 1.5): the conservative check at this place with every hypothesis carried here
        here = T.hypotheses(g, synthetic=[(T.moved(b, o0, ot), "believed-start")]
                            + [(T.moved(q, o0, ot), src) for q, src in hyps])
        res = T.residual_ambiguity(ctx.grid, p["points"], g, here, ctx.cfg, e_min=e_min, cons=ctx.cons,
                                   deadline=p["deadline_at"], **kw)
        return {"outcome": "confirm" if res.confirmed else "none", "g": list(g), "residual": list(res.reasons)}
    raise ValueError(f"unknown job {kind!r}")


class InlineWorker:
    """Runs each job at submit (tests, and a runner that cannot start a process)."""

    def __init__(self, ctx: JobContext):
        self.ctx, self._done, self._n = ctx, [], 0

    def submit(self, kind, payload) -> int:
        self._n += 1
        try:
            self._done.append((self._n, run_job(kind, payload, self.ctx)))
        except Exception as exc:  # noqa: BLE001 -- a job's error is its result
            self._done.append((self._n, {"error": f"{type(exc).__name__}: {exc}"}))
        return self._n

    def poll(self) -> list:
        out, self._done = self._done, []
        return out

    def alive(self) -> bool:
        return True

    def close(self) -> None:
        pass


def _serve(grid_path, cfg, reloc_cfg, bounds, requests, results) -> None:
    from forklift_core.localization.slam_map import Grid

    ctx = JobContext(Grid.from_npz(grid_path), cfg, reloc_cfg, bounds)
    while True:
        item = requests.get()
        if item is None:
            return
        job, kind, payload = item
        try:
            results.put((job, run_job(kind, payload, ctx)))
        except Exception as exc:  # noqa: BLE001
            results.put((job, {"error": f"{type(exc).__name__}: {exc}"}))


class ProcessWorker:
    """One worker process (spawned) holding the survey grid and its indices; jobs and results over queues."""

    def __init__(self, grid_path, cfg, reloc_cfg, bounds):
        mp = multiprocessing.get_context("spawn")
        self._req, self._res, self._n = mp.Queue(), mp.Queue(), 0
        self._proc = mp.Process(target=_serve, args=(str(grid_path), cfg, reloc_cfg, bounds, self._req, self._res),
                                name="pose-check-worker", daemon=True)
        self._proc.start()

    def submit(self, kind, payload) -> int:
        self._n += 1
        self._req.put((self._n, kind, payload))
        return self._n

    def poll(self) -> list:
        out = []
        while True:
            try:
                out.append(self._res.get_nowait())
            except queue.Empty:
                return out

    def alive(self) -> bool:
        return self._proc.is_alive()

    def close(self) -> None:
        try:
            self._req.put(None)
            self._proc.join(timeout=2.0)
        finally:
            if self._proc.is_alive():
                self._proc.terminate()
                self._proc.join(timeout=2.0)
            self._req.close()
            self._res.close()


# --- the runtime ------------------------------------------------------------------------------------------------------
@dataclass
class _Scan:
    stamp: float
    ranges: np.ndarray
    o: tuple
    standing: bool


@dataclass
class _Rematch:
    phase: str  # unconfirmed | confirmed
    o_begin: tuple
    epoch: object
    control_gen: object
    relocalizer: R.StartRelocalizer
    stage: str = "scans"  # scans | relocalize | decide
    job: int | None = None
    points: object = None
    o_scan: tuple | None = None
    attempts: list = field(default_factory=list)


class PoseRuntime:
    def __init__(self, *, bounds: P.Bounds, worker, estimate, o0, pool, gen_points, reloc_cfg: R.RelocConfig, angles,
                 range_min_m: float, range_max_m: float, match_range_m: float, rear_offset_m: float, stop_m: float,
                 tick_m: float, clock=time.monotonic, worker_factory=None, deadline_s: float = DEADLINE_S):
        self.pc = P.PoseCheck(bounds, rear_offset_m=rear_offset_m, stop_m=stop_m, tick_m=tick_m)
        self.bounds, self.worker, self.worker_factory, self.restarts = bounds, worker, worker_factory, 0
        self.b, self.o0 = tuple(float(v) for v in estimate), tuple(float(v) for v in o0)
        self.pool, self.gen_points, self.reloc_cfg = [tuple(p) for p in pool], gen_points, reloc_cfg
        self.angles, self.rmin, self.rmax, self.mrange = tuple(angles), range_min_m, range_max_m, match_range_m
        self.rear_offset_m, self.clock, self.deadline_s = rear_offset_m, clock, deadline_s
        self.buf: deque = deque(maxlen=SCANS)
        self.stage = "await_start"  # await_start | start_running | started
        self.start_job = None
        self.start = None  # {o_scan, b_scan, result}
        self.hyps_s: list = []  # the start check's hypotheses at O_s (the believed start excluded)
        self.o_s = None
        self.o_last = None
        self.rm: _Rematch | None = None
        self.rematches: list = []
        self._zones_cache = None

    # --- inputs --------------------------------------------------------------------------------------------------------
    def on_scan(self, stamp: float, ranges, o_base, standing: bool) -> None:
        """A scan the SLAM link was sent (its ranges as sent), the odometry at its stamp and the stand then."""
        self.buf.append(_Scan(float(stamp), np.asarray(ranges, float), tuple(float(v) for v in o_base),
                              bool(standing)))

    def _still_set(self):
        """(points, the last scan's odometry) of the last SCANS scans when all stood and the odometry did not move
        across them (0.002 m / 0.002 rad on the base and the rear axle), else None."""
        if len(self.buf) < SCANS or not all(s.standing for s in self.buf):
            return None
        first = self.buf[0].o
        for s in self.buf:
            m, rad = P.gap(s.o, first, self.rear_offset_m)
            if m > P.STILL_M or rad > P.STILL_RAD:
                return None
        scans = [((0.0, 0.0, 0.0), s.ranges, *self.angles) for s in self.buf]
        pts = G.stack_scans(scans, range_min_m=self.rmin, range_max_m=self.rmax, match_range_m=self.mrange)
        return pts, self.buf[-1].o

    # --- every tick ----------------------------------------------------------------------------------------------------
    def tick(self, t: float, o, C, *, standing: bool, deadman: bool, epoch, control_gen, mission_active: bool = False,
             near: bool = False) -> P.View:
        o = tuple(float(v) for v in o)
        self.o_last = o
        self._check_worker(t)
        for job, result in self.worker.poll():
            self._result(t, job, result, o, C, epoch, control_gen)
        if C is None:  # no SLAM correction yet: a scan before it is not the start's (plan 2.2)
            self.buf.clear()
            return self.pc.update(t, o, (0.0, 0.0, 0.0), standing=standing, deadman=deadman)
        if self.stage == "await_start":
            got = self._still_set()
            if got is not None:
                pts, o_scan = got
                b_scan = T.moved(self.b, self.o0, o_scan)
                self.start = {"o_scan": list(o_scan), "b_scan": list(b_scan)}
                self.start_job = self.worker.submit("start_check", {
                    "points": pts, "b_scan": b_scan, "pool": self.pool, "gen_points": self.gen_points,
                    "o0": self.o0, "o_scan": o_scan, "deadline_at": self.clock() + self.deadline_s})
                self.stage = "start_running"
                self.start["submitted_t"] = t
        view = self.pc.update(t, o, C, standing=standing, deadman=deadman, mission_active=mission_active, near=near)
        if view.want_rematch and self.rm is None:
            phase = self.pc.status
            self.pc.begin_rematch(t, epoch=epoch, control_gen=control_gen)
            self.rm = _Rematch(phase, o, epoch, control_gen, R.StartRelocalizer(self.reloc_cfg, self.clock))
            self.buf.clear()
            view = self.pc.update(t, o, C, standing=standing, deadman=deadman, mission_active=mission_active,
                                  near=near)
        if self.rm is not None and self.rm.stage == "scans":
            m, rad = P.gap(o, self.rm.o_begin, self.rear_offset_m)
            if not standing or m > P.STILL_M or rad > P.STILL_RAD:  # it moved before its scans: no evidence
                self._finish_rematch(t, "none" if self.rm.phase == "confirmed" else "unconfirmed", None,
                                     self.rm.o_begin, o, C, epoch, control_gen, why="moved")
                view = self.pc.update(t, o, C, standing=standing, deadman=deadman, mission_active=mission_active,
                                      near=near)
            else:
                got = self._still_set()
                if got is not None:
                    self.rm.points, self.rm.o_scan = got
                    if self.rm.relocalizer.deadline_at is None:
                        self.rm.relocalizer.start()
                    self.rm.job = self.worker.submit("relocalize", {
                        "points": self.rm.points, "deadline_at": self.rm.relocalizer.deadline_at})
                    self.rm.stage = "relocalize"
        return view

    def _result(self, t, job, result, o, C, epoch, control_gen) -> None:
        if job == self.start_job and self.stage == "start_running":
            self.stage = "started"
            self.start["result"] = {k: v for k, v in result.items() if k != "hyps"}
            self.start["result_t"] = t
            o_scan = tuple(self.start["o_scan"])
            self.o_s = o_scan
            if "error" in result:
                self.hyps_s = []
                self.pc.start(t, confirmed=False, r_scan=tuple(self.start["b_scan"]), o_scan=o_scan, C=C, o_now=o,
                              reasons=("worker_error",))
                return
            self.hyps_s = [(tuple(q), src) for q, src in result["hyps"]]
            r_scan = tuple(result["base"]) if result["base"] is not None else tuple(self.start["b_scan"])
            self.pc.start(t, confirmed=bool(result["confirmed"]), r_scan=r_scan, o_scan=o_scan, C=C, o_now=o,
                          reasons=tuple(result["reasons"]), hyps=self.hyps_s)
            return
        rm = self.rm
        if rm is None or job != rm.job:
            return  # a job of an abandoned re-match
        if "error" in result:
            self._finish_rematch(t, "none" if rm.phase == "confirmed" else "unconfirmed", None, rm.o_scan, o, C,
                                 epoch, control_gen, why=result["error"])
            return
        if rm.stage == "relocalize":
            state = rm.relocalizer.record(result["match"], result["decision"])
            rm.attempts.append({"reason": result["decision"].reason, "state": state})
            if state == "chosen":
                g = rm.relocalizer.estimate.base
                rm.job = self.worker.submit("rematch", {
                    "phase": rm.phase, "points": rm.points, "b": T.moved(self.b, self.o0, self.o_s), "hyps":
                    self.hyps_s, "o0": self.o_s, "ot": rm.o_scan, "decision": "confirmed", "g": g,
                    "deadline_at": rm.relocalizer.deadline_at})
                rm.stage = "decide"
            elif state == "rescan":
                rm.stage = "scans"
                self.buf.clear()
            else:  # no unattended confirmation in this generation: no evidence
                self._finish_rematch(t, "none" if rm.phase == "confirmed" else "unconfirmed", None, rm.o_scan, o, C,
                                     epoch, control_gen, why=state)
            return
        if rm.stage == "decide":
            g = result.get("g")
            self._finish_rematch(t, result["outcome"], None if g is None else tuple(g), rm.o_scan, o, C, epoch,
                                 control_gen, why=result.get("residual"))

    def _finish_rematch(self, t, outcome, g, o_scan, o, C, epoch, control_gen, *, why=None) -> None:
        rm, self.rm = self.rm, None
        self.rematches.append({"t": t, "phase": rm.phase, "outcome": outcome, "g": None if g is None else list(g),
                               "attempts": rm.attempts, "why": why})
        self.buf.clear()
        self.pc.rematch_result(t, outcome, g, tuple(o_scan), o_now=o, C=C if C is not None else (0.0, 0.0, 0.0),
                               epoch=epoch, control_gen=control_gen)

    def _check_worker(self, t) -> None:
        if self.worker.alive():
            return
        if self.worker_factory is not None and self.restarts < 1:
            self.restarts += 1
            self.worker = self.worker_factory()
            if self.stage == "start_running":  # its job went with the worker: the start check runs again
                self.stage, self.start_job = "await_start", None
            if self.rm is not None:
                self.rm.stage, self.rm.job = "scans", None
            return
        self.pc.end(t, "pose_worker_dead")

    # --- outputs -------------------------------------------------------------------------------------------------------
    def view_dict(self, view: P.View | None = None) -> dict:
        view = view or P.View(self.pc.status, reason="pose_checking" if self.pc.status == "checking" else None)
        return {"status": view.status, "drive_ready": bool(view.drive_ready), "reason": view.reason}

    def zones(self, map_zones) -> tuple:
        """(pallet zones in odometry coordinates, their cache key) while the start is not confirmed -- the union over
        the believed start and every hypothesis at O_s, grown at the net displacement's bin (plan 3.3) -- or
        (None, None): the normal zones."""
        status = self.pc.status
        if self.o_s is None or status == "confirmed" or status.startswith("ended:") or self.o_last is None:
            return None, None
        if status == "checking" and self.pc.c_int is not None:
            return None, None  # a confirmed re-match in progress: the confirmed zones
        d = math.hypot(self.o_last[0] - self.o_s[0], self.o_last[1] - self.o_s[1])
        upper = (math.floor(d / ZONE_BIN_M) + 1) * ZONE_BIN_M
        key = (tuple(map_zones), round(upper / ZONE_BIN_M))
        if self._zones_cache is None or self._zones_cache[0] != key:
            hyps = [T.moved(self.b, self.o0, self.o_s)] + [q for q, _ in self.hyps_s]
            rho = T.odom_zones(list(map_zones), hyps, self.o_s)
            self._zones_cache = (key, T.grown_zones(rho, upper, self.bounds))
        return self._zones_cache[1], ("pose_union", key[1], hash(key[0]))

    def record(self) -> dict:
        return {"start": self.start, "events": self.pc.events, "rematches": self.rematches,
                "hypotheses": len(self.hyps_s), "status": self.pc.status, "worker_restarts": self.restarts}

    def close(self) -> None:
        self.worker.close()


__all__ = ["DEADLINE_S", "InlineWorker", "JobContext", "PoseRuntime", "ProcessWorker", "SCANS", "ZONE_BIN_M",
           "run_job"]
