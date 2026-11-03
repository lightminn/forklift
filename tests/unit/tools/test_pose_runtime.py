"""The runner's side of the unconfirmed position check (sim/isaac/pose_runtime.py, docs/plans/2026-11-02-unconfirmed-runner.md
2.2, 3.3), Isaac-free: the live scan sets (five still, standing scans the SLAM link was sent), the start check after the
SLAM tracks, the re-match generation (the frozen relocalizer's attempts, then the decision), the worker's jobs, and
the pallet zones in odometry coordinates."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))
sys.path.insert(0, str(ROOT / "tests/unit/localization"))

import pose_runtime as PR  # noqa: E402
import test_global_match as TG  # noqa: E402

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import pose_check as P  # noqa: E402
from forklift_core.localization import start_reloc as R  # noqa: E402
from forklift_core.localization.slam_pose import compose, invert  # noqa: E402
from forklift_core.safety.pallet_permit import Zone  # noqa: E402

BOUNDS = P.Bounds(odom=(0.0, 0.0, 0.0, 0.0), e_still=(0.03, 0.01), e_slam=(0.05, 0.025))
I = (0.0, 0.0, 0.0)
CFG = G.MatchConfig(yaw_step_rad=math.radians(2.0))
RELOC = R.RelocConfig(e_min=0.8, r_max=0.9, r_struct=0.8)
ANGLES = (0.0, 2 * math.pi / 360)


class Scripted:
    """A worker whose results the test hands in: every submitted job is kept until ``answer``."""

    def __init__(self):
        self.jobs, self._done, self._n = [], [], 0

    def submit(self, kind, payload):
        self._n += 1
        self.jobs.append((self._n, kind, payload))
        return self._n

    def answer(self, result, job=None):
        job = job or self.jobs[-1][0]
        self._done.append((job, result))

    def poll(self):
        out, self._done = self._done, []
        return out

    def alive(self):
        return True

    def close(self):
        pass


def _ranges(grid, base):
    return np.asarray(TG._scan(grid, R.laser_from_base(base), beams=360)[0], float)


def _rt(worker, b=(2.42, 1.7, 0.6), o0=None, pool=(), gen_points=None, **kw):
    o0 = b if o0 is None else o0  # the odometry starts at the estimate (B-P2b)
    return PR.PoseRuntime(bounds=BOUNDS, worker=worker, estimate=b, o0=o0, pool=list(pool), gen_points=gen_points,
                          reloc_cfg=RELOC, angles=ANGLES, range_min_m=0.2, range_max_m=12.0, match_range_m=8.0,
                          rear_offset_m=0.34, stop_m=0.068, tick_m=0.001, **kw)


def _feed(rt, grid, base, o, n=5, standing=True, t0=0.0, moving=0.0):
    for k in range(n):
        rt.on_scan(t0 + 0.1 * k, _ranges(grid, base), compose(o, (moving * k, 0.0, 0.0)), standing)


def _tick(rt, o, C=I, **kw):
    kw.setdefault("standing", True)
    kw.setdefault("deadman", False)
    return rt.tick(0.0, o, C, epoch=1, control_gen=1, **kw)


def test_the_start_check_waits_for_the_slam_and_five_still_standing_scans():
    grid = TG._grid(*TG.ROOM)
    b = (2.42, 1.7, 0.6)
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)))
    _feed(rt, grid, b, b)
    assert _tick(rt, b, C=None).status == "checking" and not w.jobs  # no SLAM yet: scans before it do not count
    _feed(rt, grid, b, b, n=4, t0=1.0)
    _tick(rt, b)
    assert not w.jobs
    _feed(rt, grid, b, b, n=1, t0=1.5)
    _tick(rt, b)
    [(_, kind, payload)] = w.jobs
    assert kind == "start_check" and payload["b_scan"] == pytest.approx(b) and payload["o_scan"] == pytest.approx(b)


def test_a_scan_set_that_moved_or_did_not_stand_is_not_used():
    grid = TG._grid(*TG.ROOM)
    b = (2.42, 1.7, 0.6)
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)))
    _tick(rt, b)
    _feed(rt, grid, b, b, moving=0.001)  # 4 mm over the set: over 0.002 m
    _tick(rt, b)
    _feed(rt, grid, b, b, t0=1.0, standing=False)
    _tick(rt, b)
    assert not w.jobs
    _feed(rt, grid, b, b, t0=2.0)
    _tick(rt, b)
    assert len(w.jobs) == 1


def _started(confirmed, b=(2.42, 1.7, 0.6), hyps=()):
    grid = TG._grid(*TG.ROOM)
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)))
    _tick(rt, b)
    _feed(rt, grid, b, b)
    _tick(rt, b)
    w.answer({"confirmed": confirmed, "base": list(b), "reasons": [] if confirmed else ["competitor"],
              "hyps": [[list(p), s] for p, s in hyps], "statuses": {}, "wall_s": 1.0})
    return grid, w, rt


def test_a_confirmed_start_promotes_through_the_slam_agreement():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(True, b)
    C = compose(b, invert(b))  # SLAM agrees: C o O = b
    assert _tick(rt, b, C).status == "confirmed"


def test_an_unconfirmed_start_reverses_then_rematches_and_confirms():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(False, b)
    o = b
    assert _tick(rt, o).status == "unconfirmed"
    while _tick(rt, o, standing=False, deadman=True).drive_ready:  # one stroke, straight back
        o = compose(o, (-0.01, 0.0, 0.0))
    _tick(rt, o, standing=False)
    view = _tick(rt, o)
    assert view.status == "checking"  # the re-match: scans first
    _feed(rt, grid, o, o, t0=5.0)
    _tick(rt, o)
    _, kind, payload = w.jobs[-1]
    assert kind == "relocalize" and payload["deadline_at"] > 0
    top = G.Candidate(R.laser_from_base(o), 0.95, 0.97)
    w.answer({"match": G.GlobalMatch("ok", (top,)), "decision": G.Decision(True, "confirmed", 0.0)})
    _tick(rt, o)
    _, kind, payload = w.jobs[-1]
    assert kind == "rematch" and payload["phase"] == "unconfirmed" and payload["decision"] == "confirmed"
    assert payload["g"] == pytest.approx(o)
    w.answer({"outcome": "confirm", "g": list(o)})
    assert _tick(rt, o, compose(o, invert(o))).status == "confirmed"


def test_a_rematch_whose_scans_are_interrupted_by_a_move_is_dropped():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(False, b)
    o = b
    _tick(rt, o)
    while _tick(rt, o, standing=False, deadman=True).drive_ready:
        o = compose(o, (-0.01, 0.0, 0.0))
    _tick(rt, o, standing=False)
    assert _tick(rt, o).status == "checking"
    o2 = compose(o, (-0.05, 0.0, 0.0))
    view = _tick(rt, o2, standing=False)  # moved before its scans
    assert view.status == "unconfirmed" and rt.pc.events[-1]["kind"] == "rematch_discarded"


def test_a_refused_relocalization_leaves_it_unconfirmed():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(False, b)
    o = b
    _tick(rt, o)
    while _tick(rt, o, standing=False, deadman=True).drive_ready:
        o = compose(o, (-0.01, 0.0, 0.0))
    _tick(rt, o, standing=False)
    _tick(rt, o)
    for k in range(RELOC.attempts):  # competitor every time: rescans, then no estimate
        _feed(rt, grid, o, o, t0=5.0 + k)
        _tick(rt, o)
        top = G.Candidate(R.laser_from_base(o), 0.95, 0.97)
        w.answer({"match": G.GlobalMatch("ok", (top, G.Candidate((9.0, 9.0, 0.0), 0.94, 0.97))),
                  "decision": G.Decision(False, "competitor", 0.99)})
        _tick(rt, o)
    assert rt.pc.status == "unconfirmed" and [j[1] for j in w.jobs].count("relocalize") == RELOC.attempts


def test_the_pallet_zones_are_in_odometry_coordinates_and_fixed_within_a_bin():
    b = (2.42, 1.7, 0.6)
    twin = (8.42, 1.7, 0.6)
    grid, w, rt = _started(False, b, hyps=[(twin, "shift x+1")])
    o = b
    _tick(rt, o)
    z = Zone("p", 4.0, 1.7, 1.2, 1.0, 0.0)
    zones, key = rt.zones([z])
    assert len(zones) == 2  # the believed start's and the twin's
    # under the twin, the pallet 4.42 m behind it ... seen from the truck, the same as the map pallet from the twin
    rel = compose(invert(twin), (4.0, 1.7, 0.0))
    want = compose(b, rel)  # O_s o h^-1 o z with O_s = b
    assert any(abs(q.x_m - want[0]) < 1e-9 and abs(q.y_m - want[1]) < 1e-9 for q in zones)
    o2 = compose(o, (-0.02, 0.0, 0.0))
    assert rt.zones([z])[1] == key  # within the 0.05 m bin: the same zones
    _tick(rt, o2, standing=False, deadman=True)
    o3 = compose(o, (-0.06, 0.0, 0.0))
    _tick(rt, o3, standing=False, deadman=True)
    assert rt.zones([z])[1] != key


def test_a_confirmed_pose_has_no_union_zones():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(True, b)
    _tick(rt, b, compose(b, invert(b)))
    assert rt.zones([Zone("p", 4.0, 1.7, 1.2, 1.0, 0.0)]) == (None, None)


def test_the_jobs_themselves_on_a_room():
    # the worker's functions on a real grid: the start check confirms an asymmetric room's start
    grid = TG._grid(*TG.ROOM)
    b = (2.42, 1.7, 0.6)
    pts = G.stack_scans([(I, _ranges(grid, b), *ANGLES)], range_min_m=0.2, range_max_m=12.0)
    ctx = PR.JobContext(grid, CFG, RELOC, BOUNDS)
    out = PR.run_job("start_check", {"points": pts, "b_scan": b, "pool": [], "gen_points": pts, "o0": b,
                                     "o_scan": b, "deadline_at": None}, ctx)
    assert out["confirmed"] and out["base"] == pytest.approx(b, abs=0.1)
    out = PR.run_job("relocalize", {"points": pts, "deadline_at": None}, ctx)
    assert out["decision"].confirmed


def test_the_process_worker_runs_a_job(tmp_path):
    grid = TG._grid(*TG.ROOM)
    path = tmp_path / "grid.npz"
    np.savez(path, data=grid.data, origin=np.array(grid.origin), resolution_m=grid.resolution_m)
    w = PR.ProcessWorker(path, CFG, RELOC, BOUNDS)
    try:
        b = (2.42, 1.7, 0.6)
        pts = G.stack_scans([(I, _ranges(grid, b), *ANGLES)], range_min_m=0.2, range_max_m=12.0)
        job = w.submit("relocalize", {"points": pts, "deadline_at": None})
        got = []
        for _ in range(600):
            got += w.poll()
            if got:
                break
            import time
            time.sleep(0.05)
        assert got and got[0][0] == job and got[0][1]["decision"].confirmed
    finally:
        w.close()


def test_a_dead_worker_is_started_again_once_and_then_ends_the_check():
    b = (2.42, 1.7, 0.6)
    grid = TG._grid(*TG.ROOM)

    class Dying(Scripted):
        def __init__(self):
            super().__init__()
            self.dead = False

        def alive(self):
            return not self.dead

    first, second = Dying(), Dying()
    made = iter([second])
    rt = _rt(first, b=b, gen_points=np.zeros((40, 2)), worker_factory=lambda: next(made))
    _tick(rt, b)
    _feed(rt, grid, b, b)
    _tick(rt, b)
    assert first.jobs and rt.stage == "start_running"
    first.dead = True
    _tick(rt, b)
    # its start check runs again on the new worker, from the still set the runtime holds
    assert rt.worker is second and rt.stage == "start_running" and second.jobs[-1][1] == "start_check"
    second.dead = True
    assert _tick(rt, b).status == "ended:pose_worker_dead"


def test_the_confirmed_rematch_job_judges_the_place_with_every_hypothesis_carried():
    grid = TG._grid(*TG.ROOM)
    b = (2.42, 1.7, 0.6)
    pts = G.stack_scans([(I, _ranges(grid, b), *ANGLES)], range_min_m=0.2, range_max_m=12.0)
    ctx = PR.JobContext(grid, CFG, RELOC, BOUNDS)
    out = PR.run_job("rematch", {"phase": "confirmed", "points": pts, "b": b, "hyps": [], "o0": I, "ot": I,
                                 "decision": "confirmed", "g": b, "deadline_at": None}, ctx)
    assert out["outcome"] == "confirm"


def test_the_rear_check_is_asked_once_for_an_unconfirmed_start_and_a_block_ends_it():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(False, b)
    _tick(rt, b)
    assert rt.rear_pending()
    z = Zone("p", 1.0, 1.7, 1.2, 1.0, 0.0)
    near = rt.rear_zones([z])
    far = rt.zones([z])[0]
    assert near[0].length_m > far[0].length_m  # grown for D_MIN and the stop, not the first bin
    rt.rear_checked(0.0, blocked=False)
    assert not rt.rear_pending() and rt.pc.status == "unconfirmed"
    grid, w, rt = _started(False, b)
    _tick(rt, b)
    rt.rear_checked(0.0, blocked=True)
    assert rt.pc.status == "ended:pose_rear_blocked" and not rt.rear_pending()


def test_a_confirmed_start_needs_no_rear_check():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(True, b)
    _tick(rt, b, compose(b, invert(b)))
    assert not rt.rear_pending()


# --- Codex piece 2b review ---
def _answer_start(w, b, **fields):
    w.answer({"confirmed": False, "base": list(b), "reasons": ["competitor"], "hyps": [], "statuses":
              {"generation": "ok", "live": "ok"}, "wall_s": 1.0, **fields})


@pytest.mark.parametrize("fields", [{"error": "RuntimeError: x"}, {"statuses": {"generation": "timeout", "live": "ok"}},
                                    {"statuses": {"generation": "ok", "live": "overflow"}}])
def test_a_start_check_without_a_whole_hypothesis_set_ends_instead_of_opening_the_reverse(fields):
    # Codex 2b P1: with the hypotheses not all built, the union zones miss pallets -- no reverse (design 1.4)
    grid = TG._grid(*TG.ROOM)
    b = (2.42, 1.7, 0.6)
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)))
    _tick(rt, b)
    _feed(rt, grid, b, b)
    _tick(rt, b)
    _answer_start(w, b, **fields)
    view = _tick(rt, b)
    assert view.status == "ended:pose_start_failed" and not view.drive_ready


def test_the_start_checks_deadline_runs_from_the_first_tracking_tick_and_survives_a_restart():
    # Codex 2b P2-4: the scans' wait counts, and a new worker keeps the first deadline
    grid = TG._grid(*TG.ROOM)
    b = (2.42, 1.7, 0.6)
    now = {"t": 100.0}
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)), clock=lambda: now["t"])
    _tick(rt, b)  # tracking from here: the deadline is 130
    now["t"] = 120.0
    _feed(rt, grid, b, b)
    _tick(rt, b)
    assert w.jobs[-1][2]["deadline_at"] == 130.0
    now["t"] = 131.0
    rt.stage, rt.start_job = "await_start", None  # as after a worker restart: the job is gone
    assert _tick(rt, b).status == "ended:pose_start_failed"


def test_a_worker_that_cannot_be_started_again_ends_the_check_and_the_old_one_is_closed():
    b = (2.42, 1.7, 0.6)

    class Dead(Scripted):
        closed = False

        def alive(self):
            return False

        def close(self):
            self.closed = True

    old = Dead()

    def refuse():
        raise OSError("spawn refused")

    rt = _rt(old, b=b, gen_points=np.zeros((40, 2)), worker_factory=refuse)
    assert _tick(rt, b).status == "ended:pose_worker_dead" and old.closed


def test_the_record_holds_what_the_judgement_needs():
    # Codex 2b P3-8 (plan 3.4): the hypotheses, the re-matches' scans and odometry, every tick's class
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(False, b, hyps=[((8.42, 1.7, 0.6), "shift x+1")])
    _tick(rt, b)
    rt.note_tick(0.0, applied=b, truth=(2.42 + 0.35, 1.7, 0.6), open_=True)
    rt.note_tick(0.01, applied=b, truth=(2.42 + 0.05, 1.7, 0.6), open_=False)
    rec = rt.record()
    assert rec["hyps"] == [[[8.42, 1.7, 0.6], "shift x+1"]]
    assert rec["ticks"]["open"]["wrong"] == 1 and rec["ticks"]["all"]["success"] == 1
    assert rec["ticks"]["open"]["max_m"] == pytest.approx(0.35)
