"""The latency-tolerance test's injections (docs/plans/2026-11-11-latency-tolerance.md): the SLAM reply queue (due at
the scan tick + n, order kept, n = 0 at once, jitter reproducible per seed and channel), the per-job layer k, the
control-tick use marks, the planning answers held until their arrival + d (live), and the applied-correction write
counter the layer install respects. No Isaac."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
SPEC = importlib.util.spec_from_file_location("latency_injection", ROOT / "sim/isaac/latency_injection.py")
li = importlib.util.module_from_spec(SPEC)
sys.modules["latency_injection"] = li
SPEC.loader.exec_module(li)


def test_the_specs_parse_and_refuse_what_the_plan_does_not_register():
    assert li.parse_slam("off") is None
    assert li.parse_slam("ticks:0") == ("ticks", 0)
    assert li.parse_slam("ticks:12") == ("ticks", 12)
    assert li.parse_slam("jitter:12") == ("jitter", 12)
    assert li.parse_layer("off") is None
    assert li.parse_layer("undeclared:6") == ("undeclared", 6)
    assert li.parse_layer("declared:12") == ("declared", 12)
    assert li.parse_layer("jitter:12") == ("jitter", 12)
    for bad in ("ticks", "ticks:-1", "ticks:x", "late:3", "jitter:0"):
        with pytest.raises(ValueError):
            li.parse_slam(bad)
    for bad in ("undeclared:13", "declared:-1", "ticks:3", "jitter:13"):
        with pytest.raises(ValueError):
            li.parse_layer(bad)


def test_the_queue_delivers_at_the_scan_tick_plus_n_in_order():
    q = li.SlamReplyQueue(("ticks", 3), seed=0)
    q.push("a", scan_tick=12, moving=True, holding=False)
    assert q.drain(12) == [] and q.drain(14) == []
    assert q.drain(15) == ["a"]
    q.push("b", scan_tick=24, moving=False, holding=True)
    q.push("c", scan_tick=36, moving=True, holding=False)
    assert q.drain(27) == ["b"] and q.drain(38) == [] and q.drain(39) == ["c"]
    rec = q.records
    assert [(r["scan_tick"], r["due_tick"], r["delivered_tick"]) for r in rec] == [(12, 15, 15), (24, 27, 27), (36, 39, 39)]


def test_n_zero_delivers_in_the_same_call():
    q = li.SlamReplyQueue(("ticks", 0), seed=0)
    q.push("a", scan_tick=12, moving=True, holding=False)
    assert q.drain(12) == ["a"] and not q.pending


def test_jitter_keeps_the_order_and_repeats_per_seed():
    def run(seed):
        q = li.SlamReplyQueue(("jitter", 12), seed=seed)
        dues = []
        for i in range(50):
            q.push(i, scan_tick=12 * i, moving=True, holding=False)
            dues.append(q.records[-1]["due_tick"])
        return dues
    a, b = run(3), run(3)
    assert a == b and a != run(4)
    assert all(d2 >= d1 for d1, d2 in zip(a, a[1:]))  # never before the reply ahead of it
    assert all(0 <= d - 12 * i <= 12 for i, d in enumerate(a))  # 0..12 (the one ahead is due by 12 i at the latest)
    # the channels draw from different streams of the same seed
    assert li.channel_rng(3, "slam").integers(0, 13, 20).tolist() != li.channel_rng(3, "layer").integers(0, 13, 20).tolist()


def test_the_queue_truncates_what_is_left_at_the_end():
    q = li.SlamReplyQueue(("ticks", 12), seed=0)
    q.push("a", scan_tick=12, moving=True, holding=False)
    q.push("b", scan_tick=24, moving=True, holding=False)
    assert q.drain(24) == ["a"]
    left = q.truncate()
    assert left == 1 and q.records[-1]["delivered_tick"] is None and q.records[-1]["truncated"] is True


def test_the_layer_k_per_job():
    fixed = li.layer_k_fn(("undeclared", 6), seed=0)
    assert [fixed() for _ in range(3)] == [6, 6, 6]
    j1, j2 = li.layer_k_fn(("jitter", 12), seed=5), li.layer_k_fn(("jitter", 12), seed=5)
    ks = [j1() for _ in range(40)]
    assert ks == [j2() for _ in range(40)] and min(ks) >= 0 and max(ks) <= 12 and len(set(ks)) > 3


def test_the_use_marks_split_used_superseded_and_unused():
    m = li.UseMarks()
    m.delivered("slam", 1, tick=15)
    m.delivered("slam", 2, tick=16)  # before any control tick: 1 is superseded
    m.control_tick(17, 0.1417)
    m.delivered("layer", 7, tick=20)
    m.control_tick(21, 0.175)
    m.delivered("slam", 3, tick=30)
    out = m.result()
    assert out["slam"][1] == {"use": "superseded"}
    assert out["slam"][2] == {"use": "used", "tick": 17, "t": 0.1417}
    assert out["layer"][7] == {"use": "used", "tick": 21, "t": 0.175}
    assert out["slam"][3] == {"use": "unused"}


class FakeJobs:
    def __init__(self):
        self.done, self.arrived, self.discarded, self._dead, self.restarts, self.proc = {}, {}, [], None, 2, "P"

    def submit(self, name, payload):
        return 7

    def arrival(self, job_id):
        return self.arrived.get(job_id)

    def poll(self, job_id):
        self.arrived.pop(job_id, None)
        return self.done.pop(job_id, None)

    def discard(self, job_id):
        self.done.pop(job_id, None)
        self.arrived.pop(job_id, None)
        self.discarded.append(job_id)


def test_delayed_jobs_hold_an_answer_until_its_arrival_plus_d():
    now = [100.0]
    jobs = FakeJobs()
    d = li.DelayedJobs(jobs, 0.05, clock=lambda: now[0])
    assert d.submit("x", {}) == 7
    assert d.poll(7) is None  # not done
    jobs.done[7], jobs.arrived[7] = (True, "v"), 100.01
    now[0] = 100.03  # first seen here; the hold counts from the arrival, not from this poll
    assert d.poll(7) is None
    now[0] = 100.0601
    assert d.poll(7) == (True, "v")
    (rec,) = d.records
    assert rec["job"] == 7 and rec["arrived"] == 100.01 and rec["delivered"] == pytest.approx(100.0601)
    assert rec["injected_s"] == pytest.approx(0.0501)
    assert d.restarts == 2 and d.proc == "P" and d._dead is None  # the rest passes through


def test_delayed_jobs_discard_a_held_answer_and_pass_a_dead_worker():
    now = [0.0]
    jobs = FakeJobs()
    d = li.DelayedJobs(jobs, 0.1, clock=lambda: now[0])
    jobs.done[3], jobs.arrived[3] = (True, "v"), 0.0
    assert d.poll(3) is None
    d.discard(3)
    now[0] = 1.0
    assert d.poll(3) is None and jobs.discarded == [3]
    jobs._dead = "gone"
    assert d._dead == "gone"
    with pytest.raises(ValueError):
        li.DelayedJobs(jobs, -0.01)


def test_the_applied_watch_counts_only_applied_writes():
    o = li.AppliedWatch({"applied": None, "version": 0})
    s0 = o.writes
    o["version"] = 3
    assert o.writes == s0
    o["applied"] = (0.0, 0.0, 0.0)
    assert o.writes == s0 + 1


def test_install_writes_the_jobs_correction_only_without_a_newer_write():
    o = li.AppliedWatch({"applied": (0.0, 0.0, 0.0), "version": 4})
    info = {"correction": (1.0, 0.0, 0.0), "version": 5, "writes": o.writes}
    assert li.install_applied(o, info) is True and o["applied"] == (1.0, 0.0, 0.0) and o["version"] == 5
    info2 = {"correction": (2.0, 0.0, 0.0), "version": 6, "writes": o.writes}
    o["applied"], o["version"] = (3.0, 0.0, 0.0), 6  # a reprojection wrote after the job was prepared
    assert li.install_applied(o, info2) is False and o["applied"] == (3.0, 0.0, 0.0) and o["version"] == 6


class FakeLayer:
    def __init__(self):
        self.installed = []

    def refresh_compute(self, job):
        return {"raw": job, "snapshot": job, "exit_update": None}

    def refresh_install(self, job, result):
        self.installed.append(job)
        return SimpleNamespace(verified_m=1.0, blocked=False)


def worker(k, **kw):
    import layer_worker as lw

    return lw.LayerWorker(FakeLayer(), "delay", k, k_range=(0, 12), **kw)


def test_a_batch_layer_job_installs_at_its_due_tick_and_k_zero_at_once():
    w = worker(0)
    w.submit("j0", 12)
    assert not w.pending and w.layer.installed == ["j0"] and w.records[-1]["reason"] == "immediate"
    w = worker(12)
    w.submit("j1", 12)
    assert w.install_due(23) is None and w.pending
    assert w.install_due(24) is not None and w.records[-1]["install_tick"] == 24
    assert w.records[-1]["k"] == 12


def test_a_batch_layer_job_draws_its_k_and_reports_what_is_left():
    ks = iter([3, 0, 7])
    w = worker(0, k_fn=lambda: next(ks))
    w.submit("a", 12)
    assert w.pending_info()["due_tick"] == 15
    w.install_due(15)
    w.submit("b", 24)  # k 0: installed inside the submit
    assert not w.pending and w.records[-1]["reason"] == "immediate"
    w.submit("c", 36)
    assert w.pending_info()["due_tick"] == 43 and w.pending_info()["k"] == 7


def test_the_live_worker_keeps_its_range():
    import layer_worker as lw

    with pytest.raises(ValueError):
        lw.LayerWorker(FakeLayer(), "delay", 0)
    with pytest.raises(ValueError):
        lw.LayerWorker(FakeLayer(), "thread", 0, k_range=(0, 12))
    w = lw.LayerWorker(FakeLayer(), "delay", 3)
    w.submit("a", 12)
    assert "k" not in w.records[-1] if w.records else "k" not in w.pending_info()


def test_the_declared_layer_adds_k_over_120():
    import obstacle_layer as mod  # sim/isaac is on sys.path (above)

    cfg = mod.load_layer_config(ROOT / "config/obstacle_layer_single_team_spec.yaml")
    base_lat, base_f = float(cfg["permission"]["latency_s"]), float(cfg["grid"]["free_max_age_s"])
    out = mod.worker_config(cfg, 12, added_s=12 / 120)
    assert out["permission"]["latency_s"] == pytest.approx(base_lat + 0.1)
    assert out["grid"]["free_use_age_s"] == pytest.approx(base_f + 0.1)
    assert mod.worker_config(cfg, 3)["permission"]["latency_s"] == pytest.approx(base_lat + 2 / 120)  # live unchanged


def test_a_queued_reply_keeps_its_scan_id():
    q = li.SlamReplyQueue(("ticks", 6), seed=0)
    q.push("a", scan_tick=12, moving=True, holding=False, ident=41)
    assert q.records[-1]["id"] == 41


def test_a_layer_job_replaced_at_once_is_superseded_not_used():
    m = li.UseMarks()
    m.delivered("layer", 12, tick=20)
    m.superseded("layer", 12)
    m.control_tick(21, 0.175)
    assert m.result()["layer"][12] == {"use": "superseded"}


def test_delayed_jobs_record_a_held_answer_dropped_or_cut():
    now = [0.0]
    jobs = FakeJobs()
    d = li.DelayedJobs(jobs, 0.1, clock=lambda: now[0])
    jobs.done[3], jobs.arrived[3] = (True, "v"), 0.0
    now[0] = 0.04
    d.discard(3)
    assert d.records[-1] == {"job": 3, "arrived": 0.0, "delivered": None, "discarded": 0.04, "held_s": 0.04}
    jobs.done[4], jobs.arrived[4] = (True, "w"), 0.05
    now[0] = 0.07
    assert d.held_at_end([4, 9]) == [{"job": 4, "arrived": 0.05, "delivered": None, "cut": 0.07,
                                       "held_s": pytest.approx(0.02)}]


def test_the_lockstep_reply_handling_is_inside_slam_deliver():
    """The refactor's guard: slam_deliver holds the tracker call, the start check and the record (not its docstring
    alone), and the scan block calls it on the path the options leave untouched."""
    import ast

    tree = ast.parse((ROOT / "sim/isaac/run_transport.py").read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "slam_deliver")
    body = ast.unparse(fn)
    assert "slam['tracker'].receive(" in body and "slam_not_localized" in body and "slam['records'].append(" in body
    step = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "step_world")
    assert "slam_deliver(reply, m_)" in ast.unparse(step)


def test_held_at_end_covers_every_open_job_of_the_wrapper():
    now = [0.0]
    jobs = FakeJobs()
    ids = iter([11, 12, 13])
    jobs.submit = lambda name, payload: next(ids)
    d = li.DelayedJobs(jobs, 0.1, clock=lambda: now[0])
    a, b, c = d.submit("observe", {}), d.submit("next_pick", {}), d.submit("dock", {})
    jobs.done.update({a: (True, 1), b: (True, 2)})
    jobs.arrived.update({a: 0.0, b: 0.0})
    now[0] = 0.2
    assert d.poll(a) == (True, 1)  # delivered: no longer open
    out = d.held_at_end()  # b arrived and held (the pick service's), c not arrived yet
    assert [r["job"] for r in out] == [b]
