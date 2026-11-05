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
from forklift_core.localization.slam_pose import compose, invert

SCANS = 5  # scan_capture.SCANS: one set, as the start relocalization takes it
DEADLINE_S = 30.0  # the start check's and a re-match generation's deadline (the matcher's, plan 3.2)
ZONE_BIN_M = 0.05  # the net displacement's bin: zones grown at its upper end, fixed within it (plan 3.3)
DISPLAY_ZONES = 64  # the console is sent the nearest ones, and the count left out (Codex 2c P2)
DISPLAY_PERIOD_S = 0.1  # the state's publish period
RESULT_GRACE_S = 5.0  # past a job's own deadline, the time its result may take to come back before it counts as lost
STAND_SETS_MAX = 20  # plan 2026-11-06 1.5: the mission stands whose scan sets are kept, a session
STAND_SET_COUNT = 3  # and the scan sets kept at each
TRUTH_KEEP = 120 * 90  # every control tick's truth for 90 s (plan 2026-11-06 1.3): a result's scan is looked up in it
# the shadow's provisional verdicts (plan 2026-11-06 1.1): it takes no development values, and what these say is only
# recorded -- neither the development values nor the samples' selection read it
SHADOW_BOUNDS = P.Bounds(odom=(0.0, 0.0, 0.0, 0.0), e_still=(P.INT_M, P.INT_RAD), e_slam=(P.REMATCH_M, P.INT_RAD))


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
    g: tuple | None = None  # the generation's unattended confirmation, once chosen


class PoseRuntime:
    def __init__(self, *, bounds: P.Bounds, worker, estimate, o0, pool, gen_points, reloc_cfg: R.RelocConfig, angles,
                 range_min_m: float, range_max_m: float, match_range_m: float, rear_offset_m: float, stop_m: float,
                 tick_m: float, clock=time.monotonic, worker_factory=None, deadline_s: float = DEADLINE_S,
                 shadow: bool = False, contacts=None):
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
        self._truths: deque = deque(maxlen=TRUTH_KEEP)  # (t, truth) every tick: the evidence's scan time looked up
        self._evidence_t = None  # the last scan time of the evidence the next agreement uses
        self.worker_lost = False  # no worker any more (pose_worker_dead): nothing more is asked
        self._c0 = None  # the start evidence's own correction r o O_scan^-1: the samples' delta before C_int
        self._last = None  # this tick's (o, delta, its reference) for the samples
        self.confirm_tick = self.first_open_tick = None  # plan 2026-11-06 1.3: the applied pose's class at each
        # the shadow (plan 2026-11-06 1.1): the same computation on the operating path, nothing gated or ended; its
        # measurements do not lean on the provisional verdicts -- a stroke meter of its own, a generation at every
        # stand past D_MIN, the raw start and re-match agreements and every stand's largest correction move
        self.shadow = bool(shadow)
        self.contacts = contacts  # pose_contacts.ContactLog (plan 2026-11-06 1.4), the runner feeding it; None: not kept
        self.meter = None  # a PoseCheck held unconfirmed from the start check's scan: its strokes and stands only
        self._meter_view = None
        self.raw = {"start": None, "rematches": [], "stands": []} if self.shadow else None
        self._stand = None
        self._raw_pending: list = []  # (raw record, its scan time, its evidence): judged against the truth there
        # plan 2026-11-06 1.5: at a mission stand (the near field left out) where the applied pose has moved over 0.10 m
        # from C_int's, the inputs of one generation -- up to STAND_SET_COUNT sets of SCANS scans in a row while it
        # stands -- for the offline re-match (a snapshot approximation); past STAND_SETS_MAX stands only counted
        self.stand_sets: list = []
        self.stand_sets_skipped = 0
        self._capture = None
        self._was_standing = False
        self._stand_considered = False

    # --- inputs --------------------------------------------------------------------------------------------------------
    def on_scan(self, stamp: float, ranges, o_base, standing: bool) -> None:
        """A scan the SLAM link was sent (its ranges as sent), the odometry at its stamp and the stand then."""
        self.buf.append(_Scan(float(stamp), np.asarray(ranges, float), tuple(float(v) for v in o_base),
                              bool(standing)))
        cap = self._capture
        if cap is not None and standing and len(cap["sets"]) < STAND_SET_COUNT:
            cap["cur"].append(self.buf[-1])
            if len(cap["cur"]) == SCANS:
                cap["sets"].append({"stamps": [s.stamp for s in cap["cur"]], "o": list(cap["cur"][-1].o),
                                    "ranges": np.stack([s.ranges for s in cap["cur"]])})
                cap["cur"] = []

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
        view = self._tick(t, o, C, standing=standing, deadman=deadman, epoch=epoch, control_gen=control_gen,
                          mission_active=mission_active, near=near)
        if self.shadow:
            self._shadow_stand(t, o, C, standing, mission_active, near)
        self._last = None
        if C is not None:
            ref, kind = (self.pc.c_int, "int") if self.pc.c_int is not None else (self._c0, "start")
            if ref is not None:
                self._last = (o, P.gap(compose(C, o), compose(ref, o), self.rear_offset_m), kind)
        self._stand_capture(t, standing, mission_active, near)
        return view

    def _stand_capture(self, t, standing, mission_active, near) -> None:
        if standing and not self._was_standing:
            self._stand_considered = False
        self._was_standing = standing
        if self._capture is not None and not standing:
            self._capture["t1"], self._capture = t, None
            return
        if (standing and self._capture is None and not self._stand_considered and mission_active and not near
                and self._last is not None and self._last[1][0] > P.REMATCH_M):
            self._stand_considered = True
            if len(self.stand_sets) >= STAND_SETS_MAX:
                self.stand_sets_skipped += 1
                return
            self._capture = {"t0": t, "t1": None, "delta": list(self._last[1]), "ref": self._last[2], "sets": [],
                             "cur": []}
            self.stand_sets.append(self._capture)

    def save_stand_scans(self, path) -> None:
        """The kept stands' scan sets (s<stand>_<set>: SCANS x beams ranges as sent) with the scan pattern."""
        arrays = {f"s{i}_{k}": s["ranges"].astype(np.float32) for i, cap in enumerate(self.stand_sets)
                  for k, s in enumerate(cap["sets"])}
        np.savez_compressed(path, angles=np.asarray(self.angles, float),
                            ranges_m=np.asarray([self.rmin, self.rmax, self.mrange], float), **arrays)

    def _stopped(self) -> bool:
        """Nothing more to ask the worker: it is lost, or (on) the check ended (Codex 2b re-review P2)."""
        return self.worker_lost or (not self.shadow and self.pc.status.startswith("ended:"))

    def _tick(self, t, o, C, *, standing, deadman, epoch, control_gen, mission_active, near) -> P.View:
        if not self._stopped():
            self._check_worker(t)
        if self._stopped():  # Codex 2b re-review P2: nothing more to ask a closed worker
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
        want = view.want_rematch
        if self.shadow:  # its own trigger: every stand past D_MIN outside a mission and the near field
            self._meter_view = None if self.meter is None else self.meter.update(t, o, C, standing=standing,
                                                                                 deadman=deadman)
            want = self._meter_view is not None and self._meter_view.want_rematch and not mission_active and not near
        if want and self.rm is None:
            if self.shadow:
                phase = "confirmed" if self.pc.c_int is not None else "unconfirmed"
                self.meter.rematched_at = self.meter.stand_id
            else:
                phase = self.pc.status
            self.pc.begin_rematch(t, epoch=epoch, control_gen=control_gen)  # unconfirmed or confirmed only
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
            if result.get("base") is not None:
                self._c0 = compose(tuple(result["base"]), invert(o_scan))
            if self.shadow:
                self._shadow_start(t, result, o_scan, o, C)
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
                rm.g = tuple(g)
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
        if self.shadow:
            self._shadow_rematch(t, rm, outcome, g, o_scan, o, C)
        self.pc.rematch_result(t, outcome, g, tuple(o_scan), o_now=o, C=C if C is not None else (0.0, 0.0, 0.0),
                               epoch=epoch, control_gen=control_gen)

    # --- the shadow's measurements (plan 2026-11-06 1.1) ---------------------------------------------------------------
    def _shadow_start(self, t, result, o_scan, o, C) -> None:
        """The start check's local r and its agreement with the correction then, with no threshold; the stroke meter
        from the start's scan."""
        r = result.get("base")
        raw = {"t": t, "scan_t": self.start["scan_t"], "o_scan": list(o_scan), "confirmed": bool(result.get("confirmed")),
               "r": None if r is None else list(r), "C": None if C is None else list(C), "error": result.get("error"),
               "statuses": result.get("statuses")}
        if r is not None and C is not None:
            raw["agree_m"], raw["agree_rad"] = P.agreement(tuple(r), o_scan, C, o, self.rear_offset_m)
            self._raw_pending.append((raw, raw["scan_t"], tuple(r)))
        self.raw["start"] = raw
        self.meter = P.PoseCheck(self.bounds, rear_offset_m=self.rear_offset_m, stop_m=self.pc.stop_m,
                                 tick_m=self.pc.tick_m)
        self.meter.status, self.meter.origin = "unconfirmed", tuple(o_scan)

    def _shadow_rematch(self, t, rm, outcome, g, o_scan, o, C) -> None:
        """A generation's raw values: its g (the decision's, else the relocalizer's) against b' and every h' carried to
        its scan, the conservative outcome, and its agreement with the correction at the result's tick."""
        g = g if g is not None else rm.g
        raw = {"t": t, "phase": rm.phase, "outcome": outcome, "g": None if g is None else list(g),
               "o_begin": list(rm.o_begin), "o_scan": None if o_scan is None else list(o_scan),
               "scan_t": rm.stamps[-1][-1] if rm.stamps else None, "attempts": len(rm.attempts)}
        if g is not None and o_scan is not None:
            g, o_scan = tuple(g), tuple(o_scan)
            raw["to_b"] = list(P.gap(g, T.moved(self.b, self.o0, o_scan), self.rear_offset_m))
            raw["to_h"] = [[*P.gap(g, T.moved(q, self.o_s, o_scan), self.rear_offset_m), src]
                           for q, src in self.hyps_s]
            raw["moved"] = list(P.gap(o, o_scan, self.rear_offset_m))
            if C is not None:
                raw["agree_m"], raw["agree_rad"] = P.agreement(g, o_scan, C, o, self.rear_offset_m)
                raw["C"] = list(C)
            if raw["scan_t"] is not None:
                self._raw_pending.append((raw, raw["scan_t"], g))
        self.raw["rematches"].append(raw)

    def _shadow_stand(self, t, o, C, standing, mission_active, near) -> None:
        """Each stand's largest move of the applied pose by the correction against the stand's first (Delta_still)."""
        if standing and C is not None:
            if self._stand is None:
                self._stand = {"t0": t, "c_still": tuple(C), "max_m": 0.0, "max_rad": 0.0, "mission":
                               bool(mission_active), "near": bool(near), "status": self.pc.status, "t1": t}
                return
            m, rad = P.gap(compose(C, o), compose(self._stand["c_still"], o), self.rear_offset_m)
            st = self._stand
            st["max_m"], st["max_rad"], st["t1"] = max(st["max_m"], m), max(st["max_rad"], rad), t
            st["mission"] = st["mission"] or bool(mission_active)
            st["near"] = st["near"] or bool(near)
        elif self._stand is not None:
            self._close_stand()

    def _close_stand(self) -> None:
        st, self._stand = self._stand, None
        self.raw["stands"].append({**st, "c_still": list(st["c_still"])})

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
                self.worker_lost = True
                self.pc.end(t, "pose_worker_dead")
                return
            if self.stage == "start_running":  # its job went with the worker: the start check runs again
                self.stage, self.start_job = "await_start", None
            if self.rm is not None:
                self.rm.stage, self.rm.job = "scans", None
            return
        self.worker_lost = True
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
        if self.shadow:
            return None, None  # the shadow gates nothing (plan 2026-11-06 1.1)
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
        key = (view.status, None if C is None else tuple(round(float(v), 9) for v in C))
        if last is not None and t - last[0] < DISPLAY_PERIOD_S and last[2] == key:
            return last[1]  # the same status and correction (Codex 2c re-review P2: a new C redraws at once)
        if self.shadow:  # the meter's stroke, for the probe's same pressing rule (plan 2026-11-06 1.1, 2.1)
            mv = self._meter_view
            out = {"status": f"shadow:{view.status}", "reason": None, "zones": [], "zones_omitted": 0,
                   "stroke": None if mv is None or mv.stroke is None else {
                       "state": mv.stroke, "left_m": mv.stroke_left_m, "total_left_m": mv.total_left_m},
                   "rematch": None if self.rm is None else self.rm.stage, "generations": len(self.raw["rematches"])}
            self._display = (t, out, key)
            return out
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
        self._display = (t, out, key)
        return out

    def rear_pending(self) -> bool:
        return self.rear == "pending" and self.pc.status == "unconfirmed"

    def rear_checked(self, t: float, blocked: bool) -> None:
        """The runner's verdict on the straight reverse to D_MIN and its stop (design 3): blocked ends the session."""
        self.rear = {"t": t, "blocked": bool(blocked)}
        if blocked and not self.shadow:
            self.pc.end(t, "pose_rear_blocked")

    def rear_zones(self, map_zones) -> list:
        """The union zones in odometry coordinates grown for the whole move to D_MIN and the stop (design 3)."""
        upper = P.D_MIN + self.pc.stop_m + ZONE_BIN_M
        hyps = [T.moved(self.b, self.o0, self.o_s)] + [q for q, _ in self.hyps_s]
        return T.grown_zones(T.odom_zones(list(map_zones), hyps, self.o_s), upper, self.bounds)

    def ends(self) -> bool:
        """Whether the runner ends the session on this check's end: never in the shadow."""
        return not self.shadow and self.pc.status.startswith("ended:")

    def _truth_at(self, at):
        """(truth time, truth) nearest ``at``, or None when no truth is within 0.1 s of it."""
        if not self._truths:
            return None
        truth_t, truth = min(self._truths, key=lambda r: abs(r[0] - at))
        return None if abs(truth_t - at) > 0.1 else (truth_t, truth)

    def note_tick(self, t: float, *, applied, truth, open_: bool) -> None:
        """Plan 3.4: this control tick's applied pose against the simulation's truth (base link, B-P2 4.0 classes),
        counted for every tick and for the ticks with forward or mission permission open (the tick's final output,
        plan 2026-11-06 1.3); the confirming tick and the first open one after it apart; 10 Hz samples; and each new
        C_int's evidence (and the shadow's raw evidence) against the truth at its scan (design 4). The truth is kept
        every tick. The judgement only -- nothing decides on it."""
        self._truths.append((t, tuple(truth)))
        cls, m, rad = _classify(applied, truth)
        if self.confirm_tick is None and self.pc.status == "confirmed":
            self.confirm_tick = {"t": t, "class": cls, "m": m, "rad": rad, "open": bool(open_)}
        if self.confirm_tick is not None and self.first_open_tick is None and open_:
            self.first_open_tick = {"t": t, "class": cls, "m": m, "rad": rad}
        for key in ("all", "open") if open_ else ("all",):
            tally = self.ticks[key]
            tally[cls] += 1
            tally["max_m"], tally["max_rad"] = max(tally["max_m"], m), max(tally["max_rad"], rad)
        if not self.samples or t - self.samples[-1]["t"] >= 0.1:
            o, delta, ref = self._last or (None, None, None)
            self.samples.append({"t": t, "status": self.pc.status, "open": bool(open_), "applied": list(applied),
                                 "truth": list(truth), "o": None if o is None else list(o),
                                 "delta": None if delta is None else list(delta), "ref": ref})
        for raw, at, pose in self._raw_pending:
            near_ = self._truth_at(at)
            if near_ is None:
                raw["truth"] = {"class": "unknown"}
                continue
            rcls, rm_, rrad = _classify(pose, near_[1])
            raw["truth"] = {"class": rcls, "m": rm_, "rad": rrad, "truth_t": near_[0]}
        self._raw_pending = []
        for e in self.pc.events[self._seen_events:]:
            if e["kind"] in ("slam_agreement", "rematch_agreement") and e.get("ok"):
                # the evidence is the scan's: judged against the truth at that scan (Codex 2b re-review P3)
                at = self._evidence_t if self._evidence_t is not None else t
                near_ = self._truth_at(at)
                if near_ is None:  # no truth near the scan: not judged (Codex 2b re-review 3 P3)
                    self.evidence.append({"t": t, "kind": e["kind"], "class": "unknown", "scan_t": at})
                    continue
                truth_t, truth_at = near_
                ecls, em, erad = _classify(e["evidence"], truth_at)
                self.evidence.append({"t": t, "kind": e["kind"], "class": ecls, "m": em, "rad": erad,
                                      "truth_t": truth_t, "scan_t": at})
        self._seen_events = len(self.pc.events)

    def record(self) -> dict:
        if self.shadow and self._stand is not None:
            self._close_stand()
        out = {"mode": "shadow" if self.shadow else "on", "start": self.start, "events": self.pc.events,
               "rematches": self.rematches, "rear": self.rear, "hyps": [[list(q), src] for q, src in self.hyps_s],
               "status": self.pc.status, "worker_restarts": self.restarts, "ticks": self.ticks, "samples": self.samples,
               "evidence": self.evidence, "confirm_tick": self.confirm_tick, "first_open_tick": self.first_open_tick,
               "stand_sets": [{**{k: v for k, v in cap.items() if k not in ("sets", "cur")},
                               "sets": [{k: v for k, v in s.items() if k != "ranges"} for s in cap["sets"]]}
                              for cap in self.stand_sets],
               "stand_sets_skipped": self.stand_sets_skipped,
               "bounds": {"odom": list(self.bounds.odom), "e_still": list(self.bounds.e_still),
                          "e_slam": list(self.bounds.e_slam)}}
        if self.contacts is not None:
            out["contacts"] = self.contacts.record()
        if self.shadow:
            out["raw"] = self.raw
            out["meter_events"] = None if self.meter is None else self.meter.events
        return out

    def close(self) -> None:
        self.worker.close()


__all__ = ["DEADLINE_S", "InlineWorker", "JobContext", "PoseRuntime", "ProcessWorker", "SCANS", "SHADOW_BOUNDS",
           "STAND_SETS_MAX", "STAND_SET_COUNT", "ZONE_BIN_M", "run_job"]
