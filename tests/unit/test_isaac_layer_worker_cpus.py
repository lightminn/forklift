"""1e stage 1 (docs/plans/2026-11-10-g3-1e-process-worker.md 2-4): the thread worker pinned to the session's second
physical core (--layer-worker-cpus auto), its affinity recorded, and each job's scheduler counters (run time, run-queue
delay, the CPU it ended on) recorded beside its compute time. No Isaac."""

import sys
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
import layer_worker as lw  # noqa: E402


def siblings_of(pairs):
    table = {}
    for core in pairs:
        for c in core:
            table[c] = tuple(core)
    return lambda c: table.get(c, (c,))


def test_auto_takes_the_last_whole_core_of_two_or_more():
    sib = siblings_of([(0, 12), (1, 13)])
    assert lw.worker_cpus("auto", [0, 1, 12, 13], sib) == (1, 13)
    assert lw.worker_cpus("auto", [0, 12], sib) is None  # one core: nothing to pin to
    assert lw.worker_cpus("off", [0, 1, 12, 13], sib) is None
    no_smt = siblings_of([(0,), (1,), (2,)])
    assert lw.worker_cpus("auto", [0, 1, 2], no_smt) == (2,)
    with pytest.raises(ValueError):
        lw.worker_cpus("both", [0, 1], sib)


class FakeLayer:
    def __init__(self):
        self.calls = []

    def refresh_compute(self, job):
        self.calls.append(threading.get_ident())
        return {"raw": None, "snapshot": None, "exit_update": None}

    def refresh_install(self, job, result):
        return "check"


def test_the_thread_worker_pins_itself_and_records_its_affinity(monkeypatch):
    seen = []

    def set_aff(pid, cpus):
        seen.append((pid, tuple(sorted(cpus)), threading.get_ident()))

    monkeypatch.setattr(lw.os, "sched_setaffinity", set_aff)
    monkeypatch.setattr(lw.os, "sched_getaffinity", lambda pid: {1, 13})
    w = lw.LayerWorker(FakeLayer(), "thread", 3, cpus=(1, 13))
    w.submit({"j": 1}, 10)
    assert w.install_now("barrier") == "check"
    assert seen and seen[0][:2] == (0, (1, 13)) and seen[0][2] != threading.get_ident()  # from the worker's thread
    assert w.summary()["cpus"] == [1, 13] and w.summary()["affinity"] == [1, 13]


def test_delay_and_unpinned_threads_do_not_touch_affinity(monkeypatch):
    monkeypatch.setattr(lw.os, "sched_setaffinity", lambda *a: (_ for _ in ()).throw(AssertionError("pinned")))
    for mode in ("delay", "thread"):
        w = lw.LayerWorker(FakeLayer(), mode, 3)
        w.submit({"j": 1}, 10)
        w.install_now("barrier")
        assert w.summary()["cpus"] is None


def test_each_job_records_its_scheduler_counters(monkeypatch):
    reads = iter([(100, 5, 1), (900, 25, 3)])  # (run ns, run-queue delay ns, timeslices) before / after
    monkeypatch.setattr(lw, "thread_schedstat", lambda: next(reads))
    monkeypatch.setattr(lw, "thread_cpu", lambda: 13)
    w = lw.LayerWorker(FakeLayer(), "delay", 3)
    w.submit({"j": 1}, 10)
    w.install_now("barrier")
    rec = w.records[0]
    assert (rec["run_ns"], rec["run_delay_ns"], rec["cpu"]) == (800, 20, 13)


def test_unreadable_scheduler_counters_leave_the_fields_none(monkeypatch):
    monkeypatch.setattr(lw, "thread_schedstat", lambda: None)
    monkeypatch.setattr(lw, "thread_cpu", lambda: None)
    w = lw.LayerWorker(FakeLayer(), "delay", 3)
    w.submit({"j": 1}, 10)
    w.install_now("barrier")
    rec = w.records[0]
    assert (rec["run_ns"], rec["run_delay_ns"], rec["cpu"]) == (None, None, None)


def test_the_proc_readers_parse_this_thread():
    s = lw.thread_schedstat()
    assert s is None or (len(s) == 3 and all(isinstance(v, int) for v in s))
    c = lw.thread_cpu()
    assert c is None or isinstance(c, int)


def test_the_runner_and_the_near_contract_know_the_cpus_argument():
    run = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert 'parser.add_argument("--layer-worker-cpus", choices=("off", "auto"), default="off"' in run
    near = (ROOT / "sim/isaac/near_tracking.py").read_text()
    assert '"layer_worker_cpus": "off"' in near
