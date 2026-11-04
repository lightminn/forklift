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
from forklift_core.localization.slam_pose import compose

SCANS = 5  # scan_capture.SCANS: one set, as the start relocalization takes it
DEADLINE_S = 30.0  # the start check's and a re-match generation's deadline (the matcher's, plan 3.2)
ZONE_BIN_M = 0.05  # the net displacement's bin: zones grown at its upper end, fixed within it (plan 3.3)
DISPLAY_ZONES = 64  # the console is sent the nearest ones, and the count left out (Codex 2c P2)
DISPLAY_PERIOD_S = 0.1  # the state's publish period
RESULT_GRACE_S = 5.0  # past a job's own deadline, the time its result may take to come back before it counts as lost


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
def _tally() -> dict:
    return {"success": 0, "imprecise": 0, "wrong": 0, "max_m": 0.0, "max_rad": 0.0}


def _classify(pose, truth) -> tuple:
    """B-P2 4.0 on the base link: success <= 0.10 m and <= 0.05 rad, wrong > 0.30 m or > 0.10 rad, else imprecise."""
    m = math.hypot(pose[0] - truth[0], pose[1] - truth[1])
    rad = abs(math.atan2(math.sin(pose[2] - truth[2]), math.cos(pose[2] - truth[2])))
    if m <= 0.10 and rad <= 0.05:
        return "success", m, rad
    if m > 0.30 or rad > 0.10:
        return "wrong", m, rad
    return "imprecise", m, rad
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
    stamps: list = field(default_factory=list)  # each attempt's scan stamps


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
        self.rear = None  # None: not asked; "pending"; {"t", "blocked"}: design 3's start check, once
        self.start_deadline = None  # the start check's absolute deadline, from the first tracking tick (plan 3.2)
        self.ticks = {"all": _tally(), "open": _tally()}  # plan 3.4: every control tick's class (simulation truth)
        self.samples: list = []  # 10 Hz: (t, status, open, applied, truth)
        self.evidence: list = []  # every C_int's evidence against the truth (design 4)
        self._seen_events = 0
        self._display = None  # (t, the last view sent to the console)
        self._truths: deque = deque(maxlen=1800)  # (t, truth) at 20 Hz: the evidence's scan time looked up (90 s)
        self._evidence_t = None  # the last scan time of the evidence the next agreement uses

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
        if not self.pc.status.startswith("ended:"):
            self._check_worker(t)
        if self.pc.status.startswith("ended:"):  # Codex 2b re-review P2: nothing more to ask a closed worker
            return self.pc.update(t, o, C if C is not None else (0.0, 0.0, 0.0), standing=standing, deadman=deadman)
        for job, result in self.worker.poll():
            self._result(t, job, result, o, C, epoch, control_gen)
        if C is None:  # no SLAM correction yet: a scan before it is not the start's (plan 2.2)
            self.buf.clear()
            return self.pc.update(t, o, (0.0, 0.0, 0.0), standing=standing, deadman=deadman)
        if self.stage == "await_start" and self.start_deadline is None:
            self.start_deadline = self.clock() + self.deadline_s
        if self.stage == "start_running" and self.clock() > self.start_deadline + RESULT_GRACE_S:
            self.stage = "started"  # a result that never came (Codex 2b re-review P2)
            self.pc.end(t, "pose_start_failed")
        if self.stage == "await_start" and self.clock() > self.start_deadline:
            # Codex 2b P2-4: the scans' wait counts; past it no start check can be had (design 1.4: no reverse)
            self.stage = "started"
            self.pc.end(t, "pose_start_failed")
        if self.stage == "await_start":
            got = self._still_set()
            if got is not None:
                pts, o_scan = got
                b_scan = T.moved(self.b, self.o0, o_scan)
                self.start = {"o_scan": list(o_scan), "b_scan": list(b_scan), "scan_t": self.buf[-1].stamp}
                self.start_job = self.worker.submit("start_check", {
                    "points": pts, "b_scan": b_scan, "pool": self.pool, "gen_points": self.gen_points,
                    "o0": self.o0, "o_scan": o_scan, "deadline_at": self.start_deadline})
                self.stage = "start_running"
                self.start["submitted_t"] = t
        view = self.pc.update(t, o, C, standing=standing, deadman=deadman, mission_active=mission_active, near=near)
        if view.want_rematch and self.rm is None:
            phase = self.pc.status
            self.pc.begin_rematch(t, epoch=epoch, control_gen=control_gen)
            self.rm = _Rematch(phase, o, epoch, control_gen, R.StartRelocalizer(self.reloc_cfg, self.clock))
            self.rm.relocalizer.start()  # its 30 s from now: the scans' wait counts (Codex 2b re-review P2)
            self.buf.clear()
            view = self.pc.update(t, o, C, standing=standing, deadman=deadman, mission_active=mission_active,
                                  near=near)
        if self.rm is not None and self.clock() > self.rm.relocalizer.deadline_at + (
                0.0 if self.rm.stage == "scans" else RESULT_GRACE_S):
            self._finish_rematch(t, "none" if self.rm.phase == "confirmed" else "unconfirmed", None,
                                 self.rm.o_scan or self.rm.o_begin, o, C, epoch, control_gen, why="deadline")
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
                    self.rm.stamps.append([s.stamp for s in self.buf])
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
            statuses = result.get("statuses") or {}
            if "error" in result or any(v != "ok" for v in statuses.values()):
                # Codex 2b P1: without every hypothesis the union zones miss pallets -- no reverse can be opened
                # (design 1.4), so the session ends
                self.hyps_s = []
                self.pc.end(t, "pose_start_failed")
                return
            self.hyps_s = [(tuple(q), src) for q, src in result["hyps"]]
            r_scan = tuple(result["base"]) if result["base"] is not None else tuple(self.start["b_scan"])
            self._evidence_t = self.start["scan_t"]
            self.pc.start(t, confirmed=bool(result["confirmed"]), r_scan=r_scan, o_scan=o_scan, C=C, o_now=o,
                          reasons=tuple(result["reasons"]), hyps=self.hyps_s)
            if self.pc.status == "unconfirmed":
                self.rear = "pending"  # design 3: can it back up D_MIN at all? (the runner judges, rear_checked)
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
            self._evidence_t = rm.stamps[-1][-1] if rm.stamps else t
            self._finish_rematch(t, result["outcome"], None if g is None else tuple(g), rm.o_scan, o, C, epoch,
                                 control_gen, why=result.get("residual"))

    def _finish_rematch(self, t, outcome, g, o_scan, o, C, epoch, control_gen, *, why=None) -> None:
        rm, self.rm = self.rm, None
        self.rematches.append({"t": t, "phase": rm.phase, "outcome": outcome, "g": None if g is None else list(g),
                               "attempts": rm.attempts, "why": why, "o_begin": list(rm.o_begin),
                               "o_scan": None if o_scan is None else list(o_scan), "scan_stamps": rm.stamps})
        self.buf.clear()
        self.pc.rematch_result(t, outcome, g, tuple(o_scan), o_now=o, C=C if C is not None else (0.0, 0.0, 0.0),
                               epoch=epoch, control_gen=control_gen)

    def _check_worker(self, t) -> None:
        if self.worker.alive():
            return
        try:
            self.worker.close()  # Codex 2b P3: the dead one's queues go too
        except Exception:  # noqa: BLE001, S110 -- a dead worker's close may fail; it is replaced either way
            pass
        if self.worker_factory is not None and self.restarts < 1:
            self.restarts += 1
            try:
                self.worker = self.worker_factory()
            except Exception as exc:  # noqa: BLE001 -- Codex 2b P2-6: a failed restart ends the check too
                self.pc.events.append({"t": t, "kind": "worker_restart_failed", "error": repr(exc)})
                self.pc.end(t, "pose_worker_dead")
                return
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

    def display(self, view: P.View, C, map_zones, *, t: float) -> dict:
        """What the console is sent (plan 4): the status, why nothing opens, the stroke left, and the union zones as
        map polygons (C o Z_h: where each hypothesis's pallet is relative to the truck, around the estimate) -- the
        DISPLAY_ZONES nearest the truck and the count left out (the permit keeps them all); recomputed at most every
        DISPLAY_PERIOD_S or on a new status (Codex 2c P2)."""
        last = self._display
        if last is not None and t - last[0] < DISPLAY_PERIOD_S and last[1]["status"] == view.status:
            return last[1]
        stroke = None if view.stroke is None else {
            "state": view.stroke, "left_m": view.stroke_left_m, "total_left_m": view.total_left_m}
        odom, _ = self.zones(map_zones) if C is not None else (None, None)
        odom = list(odom or ())
        here = self.o_last or (0.0, 0.0, 0.0)
        odom.sort(key=lambda z: math.hypot(z.x_m - here[0], z.y_m - here[1]))
        zones = []
        for z in odom[:DISPLAY_ZONES]:
            c, s_ = math.cos(z.yaw_rad), math.sin(z.yaw_rad)
            corners = []
            for a, w in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                cx = z.x_m + c * a * z.length_m / 2 - s_ * w * z.width_m / 2
                cy = z.y_m + s_ * a * z.length_m / 2 + c * w * z.width_m / 2
                mx, my, _ = compose(tuple(C), (cx, cy, 0.0))
                corners.append([round(mx, 3), round(my, 3)])
            zones.append(corners)
        out = {"status": view.status, "reason": view.reason, "stroke": stroke, "zones": zones,
               "zones_omitted": max(0, len(odom) - DISPLAY_ZONES)}
        self._display = (t, out)
        return out

    def rear_pending(self) -> bool:
        return self.rear == "pending" and self.pc.status == "unconfirmed"

    def rear_checked(self, t: float, blocked: bool) -> None:
        """The runner's verdict on the straight reverse to D_MIN and its stop (design 3): blocked ends the session."""
        self.rear = {"t": t, "blocked": bool(blocked)}
        if blocked:
            self.pc.end(t, "pose_rear_blocked")

    def rear_zones(self, map_zones) -> list:
        """The union zones in odometry coordinates grown for the whole move to D_MIN and the stop (design 3)."""
        upper = P.D_MIN + self.pc.stop_m + ZONE_BIN_M
        hyps = [T.moved(self.b, self.o0, self.o_s)] + [q for q, _ in self.hyps_s]
        return T.grown_zones(T.odom_zones(list(map_zones), hyps, self.o_s), upper, self.bounds)

    def note_tick(self, t: float, *, applied, truth, open_: bool) -> None:
        """Plan 3.4: this control tick's applied pose against the simulation's truth (base link, B-P2 4.0 classes),
        counted for every tick and for the ticks with forward or mission permission open; 10 Hz samples; and each
        new C_int's evidence against the truth (design 4). The judgement only -- nothing decides on it."""
        if not self._truths or t - self._truths[-1][0] >= 0.05:
            self._truths.append((t, tuple(truth)))
        cls, m, rad = _classify(applied, truth)
        for key in ("all", "open") if open_ else ("all",):
            tally = self.ticks[key]
            tally[cls] += 1
            tally["max_m"], tally["max_rad"] = max(tally["max_m"], m), max(tally["max_rad"], rad)
        if not self.samples or t - self.samples[-1][0] >= 0.1:
            self.samples.append([t, self.pc.status, bool(open_), list(applied), list(truth)])
        for e in self.pc.events[self._seen_events:]:
            if e["kind"] in ("slam_agreement", "rematch_agreement") and e.get("ok"):
                # the evidence is the scan's: judged against the truth at that scan (Codex 2b re-review P3)
                at = self._evidence_t if self._evidence_t is not None else t
                truth_t, truth_at = min(self._truths, key=lambda r: abs(r[0] - at))
                if abs(truth_t - at) > 0.1:  # no truth near the scan: not judged (Codex 2b re-review 3 P3)
                    self.evidence.append({"t": t, "kind": e["kind"], "class": "unknown", "scan_t": at,
                                          "truth_t": truth_t})
                    continue
                ecls, em, erad = _classify(e["evidence"], truth_at)
                self.evidence.append({"t": t, "kind": e["kind"], "class": ecls, "m": em, "rad": erad,
                                      "truth_t": truth_t, "scan_t": at})
        self._seen_events = len(self.pc.events)

    def record(self) -> dict:
        return {"start": self.start, "events": self.pc.events, "rematches": self.rematches, "rear": self.rear,
                "hyps": [[list(q), src] for q, src in self.hyps_s], "status": self.pc.status,
                "worker_restarts": self.restarts, "ticks": self.ticks, "samples": self.samples,
                "evidence": self.evidence}

    def close(self) -> None:
        self.worker.close()


__all__ = ["DEADLINE_S", "InlineWorker", "JobContext", "PoseRuntime", "ProcessWorker", "SCANS", "ZONE_BIN_M",
           "run_job"]
