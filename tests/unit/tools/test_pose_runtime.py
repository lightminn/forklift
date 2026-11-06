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


# --- Codex piece 2b re-review ---
def test_an_ended_check_no_longer_polls_its_worker():
    b = (2.42, 1.7, 0.6)

    class Closed(Scripted):
        def alive(self):
            return False

        def poll(self):
            raise ValueError("Queue is closed")

    rt = _rt(Closed(), b=b, gen_points=np.zeros((40, 2)))  # no factory: dead at once
    assert _tick(rt, b).status == "ended:pose_worker_dead"
    assert _tick(rt, b).status == "ended:pose_worker_dead"  # no poll on the closed queue


def test_a_start_result_that_never_comes_ends_at_the_deadline():
    grid = TG._grid(*TG.ROOM)
    b = (2.42, 1.7, 0.6)
    now = {"t": 100.0}
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)), clock=lambda: now["t"])
    _tick(rt, b)
    _feed(rt, grid, b, b)
    _tick(rt, b)
    assert rt.stage == "start_running"
    now["t"] = 130.0 + PR.RESULT_GRACE_S + 0.1
    assert _tick(rt, b).status == "ended:pose_start_failed"


def test_a_rematch_generations_deadline_counts_its_scans():
    b = (2.42, 1.7, 0.6)
    grid = TG._grid(*TG.ROOM)
    now = {"t": 100.0}
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)), clock=lambda: now["t"])
    _tick(rt, b)
    _feed(rt, grid, b, b)
    _tick(rt, b)
    _answer_start(w, b)
    o = b
    _tick(rt, o)
    while _tick(rt, o, standing=False, deadman=True).drive_ready:
        o = compose(o, (-0.01, 0.0, 0.0))
    _tick(rt, o, standing=False)
    now["t"] = 200.0
    assert _tick(rt, o).status == "checking" and rt.rm.relocalizer.deadline_at == 230.0  # from the begin
    now["t"] = 231.0
    view = _tick(rt, o)  # no scans in its 30 s: no evidence
    assert view.status == "unconfirmed" and rt.rematches[-1]["why"] == "deadline"


def test_the_evidence_is_judged_against_the_truth_at_its_scans():
    # Codex 2b re-review P3: the truck may have moved between the scan and the result's tick
    b = (2.42, 1.7, 0.6)
    grid = TG._grid(*TG.ROOM)
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)))
    _tick(rt, b)
    for k in range(5):  # the scans at 1.0-1.4 s, the truth there the estimate
        rt.on_scan(1.0 + 0.1 * k, _ranges(grid, b), b, True)
        rt.note_tick(1.0 + 0.1 * k, applied=b, truth=b, open_=False)
    _tick(rt, b)
    w.answer({"confirmed": True, "base": list(b), "reasons": [], "hyps": [], "statuses": {}, "wall_s": 1.0})
    _tick(rt, b, compose(b, invert(b)))
    moved = (b[0] + 0.35, b[1], b[2])
    rt.note_tick(2.0, applied=moved, truth=moved, open_=True)
    [ev] = rt.evidence
    assert ev["class"] == "success" and ev["truth_t"] == pytest.approx(1.4)


def test_evidence_without_a_truth_near_its_scan_is_unknown_not_misjudged():
    # Codex 2b re-review 3 P3: the nearest truth seconds away judged a right evidence wrong
    b = (2.42, 1.7, 0.6)
    grid = TG._grid(*TG.ROOM)
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)))
    _tick(rt, b)
    for k in range(5):
        rt.on_scan(1.0 + 0.1 * k, _ranges(grid, b), b, True)  # no truth noted at the scans
    _tick(rt, b)
    w.answer({"confirmed": True, "base": list(b), "reasons": [], "hyps": [], "statuses": {}, "wall_s": 1.0})
    _tick(rt, b, compose(b, invert(b)))
    moved = (b[0] + 0.35, b[1], b[2])
    rt.note_tick(20.0, applied=moved, truth=moved, open_=True)
    [ev] = rt.evidence
    assert ev["class"] == "unknown"


# --- piece 2c: what the console is sent (plan 4) ---
def test_the_console_gets_the_status_the_stroke_and_the_zones_in_the_map():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(False, b, hyps=[((8.42, 1.7, 0.6), "shift x+1")])
    view = _tick(rt, b)
    C = (1.0, 0.0, 0.0)  # the applied correction: the map one metre along x from the odometry
    z = Zone("p", 4.0, 1.7, 1.2, 1.0, 0.0)
    shown = rt.display(view, C, [z], t=0.0)
    assert shown["status"] == "unconfirmed" and shown["stroke"]["state"] == "ready"
    assert shown["stroke"]["left_m"] == pytest.approx(P.D_STROKE) and len(shown["zones"]) == 2
    odom, _ = rt.zones([z])
    corners = shown["zones"][0]
    cx = sum(p[0] for p in corners) / 4
    assert cx == pytest.approx(odom[0].x_m + 1.0)  # C o Z: the odometry zone moved into the map
    import json
    json.dumps(shown)


def test_a_confirmed_check_sends_its_status_without_zones():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(True, b)
    view = _tick(rt, b, compose(b, invert(b)))
    shown = rt.display(view, compose(b, invert(b)), [Zone("p", 4.0, 1.7, 1.2, 1.0, 0.0)], t=0.0)
    assert shown == {"status": "confirmed", "reason": None, "stroke": None, "zones": [], "zones_omitted": 0}


def test_the_console_gets_the_nearest_zones_with_the_rest_counted_at_the_publish_rate():
    # Codex 2c P2: the cap was the page's only; the server sends at most the nearest PR.DISPLAY_ZONES and says how many
    # it left out, and recomputes at most every 0.1 s
    b = (2.42, 1.7, 0.6)
    hyps = [((2.42 + 6.0 * k, 1.7, 0.6), f"shift x+{k}") for k in range(1, 40)]
    grid, w, rt = _started(False, b, hyps=hyps)
    view = _tick(rt, b)
    zones = [Zone(f"p{k}", 4.0, 1.7 + k, 1.2, 1.0, 0.0) for k in range(2)]
    shown = rt.display(view, I, zones, t=0.0)
    assert len(shown["zones"]) == PR.DISPLAY_ZONES and shown["zones_omitted"] == 80 - PR.DISPLAY_ZONES
    again = rt.display(view, (5.0, 0.0, 0.0), zones, t=0.05)
    assert again is not shown  # a new correction: drawn again at once
    assert rt.display(view, (5.0, 0.0, 0.0), zones, t=0.08) is again  # the same one within 0.1 s: the last
    assert rt.display(view, (5.0, 0.0, 0.0), zones, t=0.16) is not again


def test_a_new_correction_redraws_the_zones_at_once():
    # Codex 2c re-review P2: the 0.1 s reuse kept polygons of the old C beside the new pose
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(False, b, hyps=[((8.42, 1.7, 0.6), "shift x+1")])
    view = _tick(rt, b)
    z = [Zone("p", 4.0, 1.7, 1.2, 1.0, 0.0)]
    first = rt.display(view, I, z, t=0.0)
    moved = rt.display(view, (5.0, 0.0, 0.0), z, t=0.05)
    assert moved is not first and moved["zones"][0][0][0] == pytest.approx(first["zones"][0][0][0] + 5.0)


# --- piece 2d: the shadow mode (docs/plans/2026-11-06-unconfirmed-validation.md 1.1) ---
def _shadow(confirmed, b=(2.42, 1.7, 0.6), hyps=(), base=None, **kw):
    """A shadow runtime past its start check (``base``: the start check's evidence, b by default)."""
    grid = TG._grid(*TG.ROOM)
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)), shadow=True, **kw)
    _tick(rt, b)
    _feed(rt, grid, b, b)
    _tick(rt, b)
    w.answer({"confirmed": confirmed, "base": list(base or b), "reasons": [] if confirmed else ["competitor"],
              "hyps": [[list(p), s] for p, s in hyps], "statuses": {}, "wall_s": 1.0})
    return grid, w, rt


def _reverse(rt, o, metres, C=I, **kw):
    """Back straight up ``metres`` with the deadman held, then let go and stand."""
    steps = round(metres / 0.01)
    for _ in range(steps):
        _tick(rt, o, C, standing=False, deadman=True, **kw)
        o = compose(o, (-0.01, 0.0, 0.0))
    _tick(rt, o, C, standing=False, **kw)
    return o


def test_the_shadow_never_gates_zones_or_ends():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _shadow(False, b, hyps=[((8.42, 1.7, 0.6), "shift x+1")])
    view = _tick(rt, b)
    assert rt.zones([Zone("p", 4.0, 1.7, 1.2, 1.0, 0.0)]) == (None, None)  # the permit's zones untouched
    shown = rt.display(view, I, [Zone("p", 4.0, 1.7, 1.2, 1.0, 0.0)], t=0.0)
    assert shown["status"] == "shadow:unconfirmed" and shown["zones"] == [] and shown["stroke"]["state"] == "ready"
    assert not rt.ends()  # the runner ends nothing on a shadow
    rt.rear_checked(0.0, blocked=True)  # design 3's verdict only recorded
    assert rt.pc.status == "unconfirmed" and rt.rear == {"t": 0.0, "blocked": True}


def test_the_shadow_keeps_the_start_evidence_and_its_agreement_without_a_threshold():
    # plan 1.1: the start check's local r and the correction then, whatever the provisional verdict
    b = (2.42, 1.7, 0.6)
    r = (2.72, 1.7, 0.6)  # 0.30 m from where the correction puts the truck: over any E_SLAM
    grid, w, rt = _shadow(True, b, base=r)
    _tick(rt, b, I)  # C o O = b
    raw = rt.raw["start"]
    assert raw["r"] == pytest.approx(r) and raw["agree_m"] == pytest.approx(0.30, abs=1e-6)
    assert raw["confirmed"] is True and raw["C"] == pytest.approx(I)
    assert rt.pc.status == "ended:pose_slam_disagree" and not rt.ends()  # the provisional verdict: recorded only


def test_the_shadow_meters_strokes_whatever_the_provisional_status():
    # Codex draft 2 P2-1: PoseCheck counts no stroke once confirmed; the shadow's meter does
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _shadow(True, b)
    assert _tick(rt, b, I).status == "confirmed"
    o = b
    seen = []
    for _ in range(120):
        view = _tick(rt, o, I, standing=False, deadman=True)
        seen.append(rt.display(view, I, [], t=len(seen))["stroke"]["state"])
        o = compose(o, (-0.01, 0.0, 0.0))
    assert seen[0] == "active" and seen[-1] == "closed"  # closed at D_STROKE less the stop, as when on
    assert rt.display(_tick(rt, o, I, standing=False), I, [], t=999.0)["status"] == "shadow:confirmed"


def test_the_shadow_runs_a_generation_at_every_stand_past_d_min_and_keeps_its_raw_values():
    # plan 1.1: confirmed or not, below the 0.10 m gate or not; g against b' and every h', and the agreement at the
    # result's tick
    b = (2.42, 1.7, 0.6)
    h = (8.42, 1.7, 0.6)
    grid, w, rt = _shadow(True, b, hyps=[(h, "shift x+1")])
    _tick(rt, b, I)
    o = _reverse(rt, b, 0.3)
    _tick(rt, o, I)
    assert rt.rm is None  # under D_MIN: no generation
    o = _reverse(rt, o, 0.3)
    _tick(rt, o, I)
    assert rt.rm is not None and rt.rm.phase == "confirmed"
    _feed(rt, grid, o, o, t0=5.0)
    _tick(rt, o, I)
    top = G.Candidate(R.laser_from_base(o), 0.95, 0.97)
    w.answer({"match": G.GlobalMatch("ok", (top,)), "decision": G.Decision(True, "confirmed", 0.0)})
    _tick(rt, o, I)
    g = (o[0] + 0.04, o[1], o[2])
    w.answer({"outcome": "confirm", "g": list(g), "residual": []})
    _tick(rt, o, I)
    [gen] = rt.raw["rematches"]
    assert gen["g"] == pytest.approx(g) and gen["outcome"] == "confirm" and gen["phase"] == "confirmed"
    assert gen["agree_m"] == pytest.approx(0.04, abs=1e-6)
    assert gen["to_b"][0] == pytest.approx(0.04, abs=1e-6)  # b' is where the odometry carried the start
    assert gen["to_h"][0][0] == pytest.approx(6.0, abs=0.05) and gen["to_h"][0][2] == "shift x+1"
    assert gen["scan_t"] == pytest.approx(5.4)
    _tick(rt, o, I)
    assert rt.rm is None  # one generation a stand


def test_the_shadow_runs_no_generation_in_a_mission_or_near_the_pallet():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _shadow(True, b)
    _tick(rt, b, I)
    o = _reverse(rt, b, 0.6, mission_active=True)
    _tick(rt, o, I, mission_active=True)
    assert rt.rm is None
    o = _reverse(rt, o, 0.1, near=True)
    _tick(rt, o, I, near=True)
    assert rt.rm is None


def test_the_shadow_keeps_each_stands_largest_correction_move():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _shadow(True, b)
    _tick(rt, b, I)
    _tick(rt, b, I, standing=False)
    _tick(rt, b, I)  # a stand: C_still = I
    _tick(rt, b, (0.03, 0.0, 0.0))
    _tick(rt, b, (0.01, 0.0, 0.0))
    _tick(rt, b, (0.01, 0.0, 0.0), standing=False)
    stand = rt.raw["stands"][-1]
    assert stand["max_m"] == pytest.approx(0.03) and stand["mission"] is False


def test_the_shadow_goes_on_after_its_worker_is_lost():
    b = (2.42, 1.7, 0.6)

    class Dead(Scripted):
        def alive(self):
            return False

    rt = _rt(Dead(), b=b, gen_points=np.zeros((40, 2)), shadow=True)
    view = _tick(rt, b)
    assert view.status == "ended:pose_worker_dead" and not rt.ends()
    _tick(rt, b, standing=False)
    _tick(rt, b)  # no poll on the lost worker, the stands still kept
    assert rt.raw["stands"]


def test_the_samples_carry_the_odometry_and_the_diagnostic_delta():
    # plan 1.1: before the confirmation the delta is against the start evidence's own correction (a diagnostic)
    b = (2.42, 1.7, 0.6)
    r = (2.47, 1.7, 0.6)
    grid, w, rt = _shadow(False, b, base=r)
    _tick(rt, b, I)
    rt.note_tick(0.0, applied=b, truth=b, open_=False)
    [sample] = rt.samples
    assert sample["o"] == pytest.approx(b) and sample["delta"][0] == pytest.approx(0.05, abs=1e-6)
    assert sample["ref"] == "start"


def test_every_ticks_truth_is_kept_and_the_confirmation_ticks_are_classed_apart():
    # plan 1.3 (agy 8.2): the truth every control tick; the confirming tick and the first open one recorded
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(True, b)
    for k in range(3):
        rt.note_tick(0.001 * k, applied=b, truth=b, open_=False)
    assert len(rt._truths) == 3
    _tick(rt, b, compose(b, invert(b)))
    rt.note_tick(0.01, applied=b, truth=b, open_=False)
    rt.note_tick(0.02, applied=b, truth=(b[0] + 0.2, b[1], b[2]), open_=True)
    rt.note_tick(0.03, applied=b, truth=b, open_=True)
    rec = rt.record()
    assert rec["confirm_tick"]["t"] == 0.01 and rec["confirm_tick"]["class"] == "success"
    assert rec["first_open_tick"]["t"] == 0.02 and rec["first_open_tick"]["class"] == "imprecise"


def test_the_record_carries_the_contacts_when_kept():
    import pose_contacts

    b = (2.42, 1.7, 0.6)
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)), contacts=pose_contacts.ContactLog())
    rt.contacts.update(1.0, [("/World/Forklift/body", "/World/Box")], status="checking", phase="idle", relation="free")
    assert rt.record()["contacts"]["intervals"][0]["other"] == "/World/Box"
    assert "contacts" not in _rt(Scripted(), b=b, gen_points=np.zeros((40, 2))).record()


# --- plan 2026-11-06 1.5: the mission stands' scan sets ---
def _mission_stand(rt, grid, b, C, scans, t0, near=False):
    _tick(rt, b, C, standing=False, mission_active=True, near=near)
    _tick(rt, b, C, mission_active=True, near=near)  # a new stand
    for k in range(scans):
        rt.on_scan(t0 + 0.1 * k, _ranges(grid, b), b, True)
        _tick(rt, b, C, mission_active=True, near=near)


def test_a_mission_stand_whose_correction_moved_keeps_up_to_three_scan_sets(tmp_path):
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(True, b)
    _tick(rt, b, I)  # confirmed: C_int = I
    C = (0.15, 0.0, 0.0)  # Delta_int 0.15 m: over 0.10
    _mission_stand(rt, grid, b, C, 17, 10.0)
    _tick(rt, b, C, standing=False, mission_active=True)  # the stand ends
    [cap] = rt.record()["stand_sets"]
    assert len(cap["sets"]) == 3 and cap["sets"][0]["o"] == pytest.approx(b) and cap["delta"][0] == pytest.approx(0.15)
    assert cap["sets"][1]["stamps"] == pytest.approx([10.5, 10.6, 10.7, 10.8, 10.9]) and cap["ref"] == "int"
    rt.save_stand_scans(tmp_path / "s.npz")
    data = np.load(tmp_path / "s.npz")
    assert data["s0_2"].shape == (5, 360) and data["angles"] == pytest.approx(ANGLES)


def test_a_stand_that_ends_early_keeps_what_it_got_and_quiet_or_near_stands_keep_none():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(True, b)
    _tick(rt, b, I)
    _mission_stand(rt, grid, b, (0.05, 0.0, 0.0), 6, 1.0)  # Delta 0.05: not kept
    _mission_stand(rt, grid, b, (0.15, 0.0, 0.0), 6, 2.0, near=True)  # the near field: not kept
    _mission_stand(rt, grid, b, (0.15, 0.0, 0.0), 7, 3.0)  # one whole set before it drove on
    _tick(rt, b, (0.15, 0.0, 0.0), standing=False, mission_active=True)
    [cap] = rt.record()["stand_sets"]
    assert len(cap["sets"]) == 1


def test_stands_past_twenty_are_counted_not_kept():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(True, b)
    _tick(rt, b, I)
    for k in range(PR.STAND_SETS_MAX + 2):
        _mission_stand(rt, grid, b, (0.15, 0.0, 0.0), 5, 10.0 * k)
    rec = rt.record()
    assert len(rec["stand_sets"]) == PR.STAND_SETS_MAX and rec["stand_sets_skipped"] == 2


def test_the_record_carries_the_shadow_audit_when_kept():
    import shadow_audit

    b = (2.42, 1.7, 0.6)
    rt = _rt(Scripted(), b=b, gen_points=np.zeros((40, 2)), shadow=True)
    assert rt.audit is None and "audit" not in rt.record()
    rt.audit = shadow_audit.ShadowAudit({"x": lambda: {"a": 1}})
    rt.audit.before()
    rt.audit.after("checkpoint", 0.0)
    assert rt.record()["audit"]["checks"] == 1


# --- Codex 2d review ---
def test_the_samples_carry_the_odometry_before_any_evidence():
    # P2-5: with no correction yet the sample's O was None, and the start's origin slid to after the check
    b = (2.42, 1.7, 0.6)
    w = Scripted()
    rt = _rt(w, b=b, gen_points=np.zeros((40, 2)), shadow=True)
    _tick(rt, b, None)
    rt.note_tick(0.0, applied=b, truth=b, open_=False)
    assert rt.samples[0]["o"] == pytest.approx(b) and rt.samples[0]["delta"] is None


def _shadow_generation(rt, grid, w, o, C, *, g=None, result=None, stamp=5.0):
    """One shadow generation at the stand at ``o``, answered with ``result``."""
    _tick(rt, o, C)
    _feed(rt, grid, o, o, t0=stamp)
    _tick(rt, o, C)
    top = G.Candidate(R.laser_from_base(o), 0.95, 0.97)
    w.answer({"match": G.GlobalMatch("ok", (top,)), "decision": G.Decision(True, "confirmed", 0.0)})
    _tick(rt, o, C)
    return w.jobs[-1]


def test_a_shadow_generation_asks_the_same_conservative_check_whatever_the_provisional_state():
    # P2-3: the sample's eligibility must not lean on the provisional state or delta
    b = (2.42, 1.7, 0.6)
    for confirmed in (True, False):
        grid, w, rt = _shadow(confirmed, b)
        _tick(rt, b, I)
        o = _reverse(rt, b, 0.6)
        _, kind, payload = _shadow_generation(rt, grid, w, o, I)
        assert kind == "rematch" and payload["shadow"] is True
        w.answer({"outcome": "confirm" if confirmed else "out_of_set", "g": list(o), "residual": [],
                  "residual_confirmed": True})
        _tick(rt, o, I)
        assert rt.raw["rematches"][0]["residual_confirmed"] is True


def test_a_shadow_generation_says_whether_the_operating_path_would_have_applied_it():
    # P2-4: a result after the truck moved, or in another epoch, is discarded before the agreement when on
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _shadow(True, b)
    _tick(rt, b, I)
    o = _reverse(rt, b, 0.6)
    _shadow_generation(rt, grid, w, o, I)
    w.answer({"outcome": "confirm", "g": list(o), "residual": [], "residual_confirmed": True})
    _tick(rt, compose(o, (-0.003, 0.0, 0.0)), I, standing=False)
    gen = rt.raw["rematches"][-1]
    assert gen["applicable"] is False and gen["moved"][0] > P.STILL_M
    grid, w, rt = _shadow(True, b)
    _tick(rt, b, I)
    o = _reverse(rt, b, 0.6)
    _shadow_generation(rt, grid, w, o, I)
    w.answer({"outcome": "confirm", "g": list(o), "residual": [], "residual_confirmed": True})
    _tick(rt, o, I)
    assert rt.raw["rematches"][-1]["applicable"] is True


def test_the_shadow_job_returns_the_conservative_check_in_both_phases():
    grid = TG._grid(*TG.ROOM)
    ctx = PR.JobContext(grid, CFG, RELOC, PR.SHADOW_BOUNDS)
    o = (2.42, 1.7, 0.6)
    points = G.stack_scans([((0.0, 0.0, 0.0), _ranges(grid, o), *ANGLES)] * 5, range_min_m=0.2, range_max_m=12.0,
                           match_range_m=8.0)
    start = compose(o, (0.6, 0.0, 0.0))
    for phase in ("unconfirmed", "confirmed"):
        out = PR.run_job("rematch", {"phase": phase, "shadow": True, "points": points, "b": start, "hyps": [],
                                     "o0": start, "ot": o, "decision": "confirmed", "g": o,
                                     "deadline_at": None}, ctx)
        assert isinstance(out["residual_confirmed"], bool)


def test_a_mission_stands_scan_set_holds_only_still_scans():
    # P2-6: the operating scan set's own stillness: a set that moved is not kept, the next five may be
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(True, b)
    _tick(rt, b, I)
    C = (0.15, 0.0, 0.0)
    _mission_stand(rt, grid, b, C, 0, 0.0)
    for k in range(5):
        rt.on_scan(1.0 + 0.1 * k, _ranges(grid, b), compose(b, (0.003 * k, 0.0, 0.0)), True)
    assert rt.stand_sets[0]["sets"] == []
    end = compose(b, (0.012, 0.0, 0.0))
    for k in range(4):
        rt.on_scan(2.0 + 0.1 * k, _ranges(grid, b), end, True)
    assert len(rt.stand_sets[0]["sets"]) == 1 and rt.stand_sets[0]["sets"][0]["stamps"][0] == pytest.approx(1.4)


def test_the_runtime_keeps_its_own_copy_of_a_scan():
    b = (2.42, 1.7, 0.6)
    rt = _rt(Scripted(), b=b, gen_points=np.zeros((40, 2)))
    r = np.ones(360)
    rt.on_scan(0.0, r, b, True)
    assert rt.buf[-1].ranges is not r and not np.shares_memory(rt.buf[-1].ranges, r)


def test_the_runtime_holds_no_control_object():
    # the lead, 2026-11-06 (Codex 2d P2-7): it is handed values only -- no name of the control side in its source
    import ast

    tree = ast.parse((ROOT / "sim/isaac/pose_runtime.py").read_text())
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)} | {
        n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    assert not names & {"arbiter", "core", "live", "obstacle", "layer", "tracker", "spool", "slam", "publish"}


def test_a_shadow_job_runs_the_conservative_check_once_in_either_phase(monkeypatch):
    # Codex 2d re-review P2: the second run kept the result until the deadline in one phase only
    from forklift_core.localization import twin_check as T

    grid = TG._grid(*TG.ROOM)
    ctx = PR.JobContext(grid, CFG, RELOC, PR.SHADOW_BOUNDS)
    o = (2.42, 1.7, 0.6)
    points = G.stack_scans([((0.0, 0.0, 0.0), _ranges(grid, o), *ANGLES)] * 5, range_min_m=0.2, range_max_m=12.0,
                           match_range_m=8.0)
    start = compose(o, (0.6, 0.0, 0.0))
    real, calls = T.residual_ambiguity, []

    def counted(*a, **k):
        calls.append(1)
        return real(*a, **k)

    monkeypatch.setattr(T, "residual_ambiguity", counted)
    for phase in ("unconfirmed", "confirmed"):
        calls.clear()
        out = PR.run_job("rematch", {"phase": phase, "shadow": True, "points": points, "b": start, "hyps": [],
                                     "o0": start, "ot": o, "decision": "confirmed", "g": o, "deadline_at": None}, ctx)
        assert len(calls) == 1, phase
        assert out["outcome"] == ("confirm" if out["residual_confirmed"] else
                                  ("unconfirmed" if phase == "unconfirmed" else "none"))


# --- the monitor revision (docs/plans/2026-11-07-unconfirmed-monitor-revision.md 9.5) ---
def test_the_runtime_hands_the_hold_to_the_stand_gate():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _started(True, b)
    C = compose(b, invert(b))
    _tick(rt, b, C, holding=True)
    C2 = compose((0.0, 0.0, 0.009), C)
    rt.tick(1.0, b, C2, standing=True, deadman=False, epoch=1, control_gen=1, holding=False)
    assert rt.pc.status == "confirmed"  # 0.009 rad at the release: over E_STILL 0.01? under; the reference taken again
    assert rt.pc.c_still == pytest.approx(C2)


def test_the_shadow_keeps_the_stand_rebased_at_a_release_beside_the_plain_value():
    b = (2.42, 1.7, 0.6)
    grid, w, rt = _shadow(True, b)
    _tick(rt, b, I)
    _tick(rt, b, I, standing=False)
    rt.tick(1.0, b, I, standing=True, deadman=False, epoch=1, control_gen=1, holding=True)
    rt.tick(1.1, b, (0.0, 0.0, 0.04), standing=True, deadman=False, epoch=1, control_gen=1, holding=False)
    rt.tick(1.5, b, (0.0, 0.0, 0.041), standing=True, deadman=False, epoch=1, control_gen=1, holding=False)
    rt.tick(1.6, b, (0.0, 0.0, 0.041), standing=False, deadman=False, epoch=1, control_gen=1, holding=False)
    stand = rt.raw["stands"][-1]
    assert stand["max_rad"] == pytest.approx(0.041) and stand["rebased_rad"] == pytest.approx(0.001, abs=1e-6)
