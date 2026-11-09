"""Real-time plan 5.18 G2-0: the timers' long-call record -- every region call at or over a threshold, with its
caller path, self time (the nested regions' time taken out), thread CPU time and the context it ran in; off by
default, and on it does not change what the timed code returns."""

import pytest

from forklift_core import perf


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


@pytest.fixture
def timers(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(perf.time, "perf_counter", clock)
    monkeypatch.setattr(perf.time, "thread_time", clock)
    t = perf.Timers()
    t.enabled = True
    return t, clock


def test_off_by_default_nothing_is_kept(timers):
    t, clock = timers
    with t.region("a"):
        clock.t += 1.0
    assert t.long_s is None and t.long == []
    assert t.take()["a"][1] == 1


def test_a_long_call_is_kept_with_its_path_self_time_and_context(timers):
    t, clock = timers
    t.long_s = 0.020
    t.where = lambda: {"row": 7}
    with t.region("tick"):
        clock.t += 0.005
        with t.region("capture"):
            clock.t += 0.030
            with t.region("detect"):
                clock.t += 0.150
        with t.region("short"):
            clock.t += 0.001
    names = [r["name"] for r in t.long]
    assert names == ["detect", "capture", "tick"]  # recorded as they end
    detect, capture, tick = t.long
    assert detect["path"] == ["tick", "capture"] and capture["path"] == ["tick"] and tick["path"] == []
    assert abs(detect["self_s"] - 0.150) < 1e-9 and abs(capture["self_s"] - 0.030) < 1e-9
    assert abs(tick["self_s"] - 0.005) < 1e-9  # every nested region is taken out, recorded or not
    assert abs(tick["end"] - tick["start"] - 0.186) < 1e-9
    assert all(r["row"] == 7 for r in t.long)
    assert detect["cpu_s"] == pytest.approx(0.150)


def test_short_calls_are_not_kept_and_the_take_is_unchanged(timers):
    t, clock = timers
    t.long_s = 0.020
    for _ in range(3):
        with t.region("permission"):
            clock.t += 0.001
    assert t.long == []
    assert t.take()["permission"][1] == 3


def test_wrapped_methods_nest_like_regions(timers):
    t, clock = timers
    t.long_s = 0.010

    class Layer:
        def refresh(self):
            clock.t += 0.002
            self.snapshot()
            return "done"

        def snapshot(self):
            clock.t += 0.020
            return 1

    t.wrap(Layer, "refresh", "layer.refresh")
    t.wrap(Layer, "snapshot", "grid.snapshot")
    assert Layer().refresh() == "done"
    snap, refresh = t.long
    assert snap["name"] == "grid.snapshot" and snap["path"] == ["layer.refresh"]
    assert abs(refresh["self_s"] - 0.002) < 1e-9


def test_an_exception_still_pops_the_stack(timers):
    t, clock = timers
    t.long_s = 0.010
    with pytest.raises(RuntimeError):
        with t.region("outer"):
            clock.t += 0.050
            raise RuntimeError("x")
    with t.region("next"):
        clock.t += 0.020
    assert [r["path"] for r in t.long] == [[], []]


def test_the_record_is_bounded_and_counts_what_it_drops(timers):
    t, clock = timers
    t.long_s = 0.0
    t.long_cap = 2
    for _ in range(5):
        with t.region("a"):
            clock.t += 0.001
    assert len(t.long) == 2 and t.long_dropped == 3


def test_a_failing_context_is_recorded_not_raised(timers):
    t, clock = timers
    t.long_s = 0.0

    def broken():
        raise ValueError("no trace yet")

    t.where = broken
    with t.region("a"):
        clock.t += 0.001
    assert t.long[0]["where_error"].startswith("ValueError")


def test_each_thread_keeps_its_own_stack(timers):
    import threading

    t, clock = timers
    t.long_s = 0.0
    with t.region("main_outer"):
        def worker():
            with t.region("worker_call"):
                pass
        th = threading.Thread(target=worker, name="planner")
        th.start()
        th.join()
        clock.t += 0.001
    worker_rec = next(r for r in t.long if r["name"] == "worker_call")
    main_rec = next(r for r in t.long if r["name"] == "main_outer")
    assert worker_rec["path"] == [] and worker_rec["thread"] == "planner"
    assert main_rec["path"] == [] and "thread" not in main_rec
    assert abs(main_rec["self_s"] - 0.001) < 1e-9  # the other thread's call is not nested in it


def test_long_only_regions_stay_out_of_the_totals_and_are_not_entered_off(timers):
    t, clock = timers

    class Core:
        def _inner(self):
            clock.t += 0.030
            return 5

    t.wrap(Core, "_inner", "core._inner", long_only=True)
    assert Core()._inner() == 5  # long-call record off: not even entered
    assert t.take() == {} and t.long == []
    t.long_s = 0.010
    assert Core()._inner() == 5
    assert t.take() == {} and [r["name"] for r in t.long] == ["core._inner"]


def test_the_scheduling_wait_since_the_step_is_kept_when_readable(timers, monkeypatch):
    t, clock = timers
    t.long_s = 0.0
    waits = iter([1.0, 1.25])
    monkeypatch.setattr(perf, "_run_delay_s", lambda: next(waits))
    t.sched_mark()
    clock.t += 0.5
    with t.region("a"):
        clock.t += 0.1
    rec = t.long[0]
    assert rec["run_delay_s_since_mark"] == pytest.approx(0.25) and rec["since_mark_s"] == pytest.approx(0.6)


def test_the_keeping_of_a_childs_record_is_not_the_parents_self_time(timers, monkeypatch):
    t, clock = timers
    t.long_s = 0.010

    def slow_where():
        clock.t += 0.040  # the record's keeping
        return {}

    t.where = slow_where
    with t.region("parent"):
        clock.t += 0.010
        with t.region("child"):
            clock.t += 0.030
    child, parent = t.long
    assert child["self_s"] == pytest.approx(0.030)
    assert parent["self_s"] == pytest.approx(0.010, abs=1e-9)  # not 0.050 (Codex review P2-2)
    assert t.long_overhead_s >= 0.040


def test_the_cap_holds_across_threads(timers):
    import threading

    t, clock = timers
    t.long_s = 0.0
    t.long_cap = 1
    gate = threading.Barrier(4)

    def worker():
        gate.wait()
        with t.region("w"):
            pass

    threads = [threading.Thread(target=worker) for _ in range(4)]
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    assert len(t.long) == 1 and t.long_dropped == 3


def test_by_parent_splits_an_inner_steps_total_by_the_region_it_ran_inside(timers):
    t, clock = timers

    class Grid:
        def snapshot(self):
            clock.t += 0.001
            self._geometry()

        def _geometry(self):
            clock.t += 0.002

    t.wrap(Grid, "snapshot", "grid.snapshot")
    t.wrap(Grid, "_geometry", "grid._geometry", by_parent=True)
    g = Grid()
    g.snapshot()
    g._geometry()  # off the record: no enclosing region known, the plain name
    assert set(t.take()) == {"grid.snapshot", "grid._geometry"}
    t.long_s = 1.0  # the record on (nothing long enough is kept): the stack names the parent
    g.snapshot()
    g._geometry()
    totals = t.take()
    assert totals["grid._geometry|grid.snapshot"][1] == 1 and totals["grid._geometry"][1] == 1


def test_region_calls_are_counted_while_the_record_is_on(timers):
    t, clock = timers
    with t.region("a"):
        pass
    assert t.long_region_calls == 0
    t.long_s = 1.0
    for _ in range(3):
        with t.region("a"):
            pass
    assert t.long_region_calls == 3
