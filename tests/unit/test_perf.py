"""Real-time plan S0: the region timers, the call dumps and the bench's comparison (docs/plans/2026-10-17-realtime-sim.md)."""

import gzip
import pickle
from dataclasses import dataclass, field

import numpy as np
import pytest

from forklift_core import kernels, perf
from tools import perf_bench


def test_timers_off_record_nothing_and_on_record_calls_and_longest():
    t = perf.Timers()
    with t.region("a"):
        pass
    assert t.take() == {}
    t.enabled = True
    t.add("a", 0.002)
    t.add("a", 0.005)
    with t.region("b"):
        pass
    out = t.take()
    assert out["a"] == [0.007, 2, 0.005, None]
    assert out["b"][1] == 1 and out["b"][3] is not None
    assert t.take() == {}


class Box:
    def __init__(self):
        self.items = [1, 2]

    def twice(self, x):
        return 2 * x

    @staticmethod
    def half(x):
        return x / 2


def test_timers_wrap_a_class_times_every_instance_by_caller_and_instances_still_pickle():
    original_twice, original_half = Box.__dict__["twice"], Box.__dict__["half"]
    t = perf.Timers()
    try:
        t.wrap(Box, "twice", "box.twice", by_caller=True)
        t.wrap(Box, "half", "box.half")
        a, b = Box(), Box()
        assert a.twice(3) == 6 and Box.half(4) == 2
        assert t.take() == {}  # off: the wrapper only passes through
        t.enabled = True

        def caller_one():
            return a.twice(1) + b.twice(2)

        assert caller_one() == 6
        out = t.take()
        assert out["box.twice<caller_one"][1] == 2
        assert pickle.loads(pickle.dumps(a)).items == [1, 2]  # nothing was put on the instance
    finally:
        Box.twice, Box.half = original_twice, original_half


def test_every_spec():
    assert perf._every("7") == {"*": 7}
    assert perf._every("check_arc=50,grid_snapshot=5") == {"*": 50, "check_arc": 50, "grid_snapshot": 5}
    assert perf._every("", default=0) == {"*": 0}


def test_changed():
    assert perf.changed(("t", "k"), 1) and not perf.changed(("t", "k"), 1) and perf.changed(("t", "k"), 2)


@dataclass
class Counter:
    seen: list = field(default_factory=list)

    def push(self, value, into=None):
        self.seen.append(value)
        if into is not None:
            into.append(value)
        return len(self.seen)


@pytest.fixture
def dumping(tmp_path, monkeypatch):
    monkeypatch.setattr(perf, "_DUMP_DIR", str(tmp_path))
    monkeypatch.setattr(perf, "_calls", {})
    monkeypatch.setattr(perf, "_strata", {})
    monkeypatch.setattr(perf, "_streams", {})
    return tmp_path


def records(folder):
    out = []
    for f in sorted(folder.glob("*.pkl.gz")):
        with gzip.open(f, "rb") as g:
            out.append(pickle.load(g))
    return out


def test_dumped_writes_every_nth_call_with_the_state_before_and_after(dumping, monkeypatch):
    monkeypatch.setattr(perf, "_DUMP_EVERY", {"*": 2})
    monkeypatch.setattr(perf, "_STREAM", {"*": 0})
    push = perf.dumped("push")(Counter.push)
    c = Counter()
    assert [push(c, v) for v in "abc"] == [1, 2, 3]
    d = records(dumping / "push")
    assert [r["call"] for r in d] == [0, 2]
    assert d[1]["result"] == 3 and d[1]["before"][1] == ("c",)
    assert d[1]["before"][0].seen == ["a", "b"] and d[1]["after"][0].seen == ["a", "b", "c"]


def test_dumped_keeps_an_argument_that_is_the_receivers_own_object(dumping, monkeypatch):
    monkeypatch.setattr(perf, "_DUMP_EVERY", {"*": 1})
    monkeypatch.setattr(perf, "_STREAM", {"*": 0})
    push = perf.dumped("push")(Counter.push)
    c = Counter()
    push(c, "x", c.seen)
    (d,) = records(dumping / "push")
    receiver, args, _ = d["before"]
    assert args[1] is receiver.seen  # one graph: the identity survives the copy


def test_dumped_strata_and_streams(dumping, monkeypatch):
    monkeypatch.setattr(perf, "_DUMP_EVERY", {"*": 0})
    monkeypatch.setattr(perf, "_STRATA_K", 1)
    monkeypatch.setattr(perf, "_STREAM", {"*": 0, "push": 2})
    push = perf.dumped("push", strata=lambda self_, a, k, r: (a[0],))(Counter.push)
    c1, c2 = Counter(), Counter()
    for v in "aab":
        push(c1, v)
    push(c2, "a")
    d = records(dumping / "push")
    streams = sorted(tuple(r["stream"][1:]) for r in d if r["stream"] is not None)
    assert streams == sorted([(id(c1), 0), (id(c1), 1), (id(c2), 0)])
    strata = [r["stratum"][-1] for r in d if r["stratum"] is not None]
    assert "b" in strata


def test_dumped_is_the_function_itself_when_off(monkeypatch):
    monkeypatch.setattr(perf, "_DUMP_DIR", None)
    assert perf.dumped("push")(Counter.push) is Counter.push


def test_caller_name_skips_this_modules_wrappers():
    t = perf.Timers()
    t.enabled = True

    def inner():
        return perf.caller_name()

    def outer():
        return inner()

    assert outer() == "outer"


def test_differences_are_bitwise_and_always_judge():
    a = np.array([1.0, np.nan, 0.0])
    assert perf.differences(a, a.copy()) == []
    assert perf.differences(a, np.array([1.0, np.nan, -0.0]))  # -0.0 is another bit pattern
    assert perf.differences(a, a.astype(np.float32))
    assert perf.differences(0.1 + 0.2, 0.3)
    assert perf.differences(np.float32(0.0), np.float32(-0.0))
    assert perf.differences(np.float64(1.0), np.float32(1.0))
    assert perf.differences(1.0, 2.0, limit=0)  # a display limit of 0 still judges
    assert perf.differences({"x": 1, "y": 2}, {"y": 2, "x": 1})  # dict order counts
    assert perf.differences({1, 2}, {2, 1}) == []
    assert perf.differences(Counter([1]), Counter([1])) == []
    assert perf.differences(Counter([1]), Counter([2]))


class Sloppy:
    """A method that returns the right answer but scribbles on its argument."""

    def apply(self, arr):
        out = int(arr.sum())
        arr[0] = 99
        return out


def test_bench_catches_an_implementation_that_changes_its_argument(dumping, monkeypatch):
    monkeypatch.setattr(perf, "_DUMP_EVERY", {"*": 1})
    monkeypatch.setattr(perf, "_STREAM", {"*": 0})

    def honest(self, arr):
        return int(arr.sum())

    perf.dumped("shadow_apply")(honest)(Sloppy(), np.array([1, 2, 3]))
    (d,) = records(dumping / "shadow_apply")
    state = perf_bench.start_state(d)
    _, result = perf_bench.call(d, state, kernels.mode())  # Sloppy.apply: same result, changed argument
    assert result == d["result"]
    assert perf_bench.judge(d, result, state, 5)


def test_kernel_modes():
    assert kernels._resolve("auto") == "reference"  # a batch run computes as before unless told
    previous = kernels.set_mode("reference")
    try:
        assert kernels.mode() == "reference"
        with pytest.raises(ValueError):
            kernels.set_mode("fast")
    finally:
        kernels.set_mode(previous)


class Holder:
    def __init__(self):
        self.seen = []

    def apply(self, value, into):
        into.append(value)
        return len(self.seen)


def test_a_stream_whose_arguments_share_the_receivers_objects_is_reported_not_compared(dumping, monkeypatch):
    """Codex real-time round 3 P2: a stream cannot carry an argument-is-receiver's-object relation across calls."""
    monkeypatch.setattr(perf, "_DUMP_EVERY", {"*": 0})
    monkeypatch.setattr(perf, "_STREAM", {"*": 0, "shadow_apply": 5})

    push = perf.dumped("shadow_apply")(Holder.apply)
    h = Holder()
    push(h, 1, h.seen)
    push(h, 2, h.seen)
    d = records(dumping / "shadow_apply")
    assert all(perf_bench.shares_state(r) for r in d)
    out = perf_bench.streams(sorted((dumping / "shadow_apply").glob("*.pkl.gz")), [kernels.mode()], 5)
    assert out[kernels.mode()]["shared"] == 1 and out[kernels.mode()]["calls"] == 0


def test_warm_up_compiles_the_kernels_or_does_nothing():
    previous = kernels.set_mode("numpy")
    try:
        assert kernels.warm_up() == 0.0
        if kernels.MODES and __import__("importlib").util.find_spec("numba") is not None:
            kernels.set_mode("numba")
            assert kernels.warm_up() >= 0.0
    finally:
        kernels.set_mode(previous)


class ModeDependent:
    """Returns the right sum under every mode, but under "numpy" also scribbles on its own state."""

    def __init__(self):
        self.calls = 0

    def apply(self, arr):
        self.calls += 1 if kernels.mode() != "numpy" else 2
        return int(arr.sum())


def test_the_dual_run_counts_calls_and_catches_a_candidate_that_differs(monkeypatch):
    monkeypatch.setattr(perf, "_DUAL", "numpy")
    monkeypatch.setattr(perf, "DUAL_STATS", {})
    monkeypatch.setattr(perf, "_dual_carried", {})
    previous = kernels.set_mode("reference")
    try:
        same = perf.dual("same")(Counter.push)
        c = Counter()
        assert same(c, "a") == 1 and c.seen == ["a"]  # the leading run is the real one
        assert perf.DUAL_STATS["same"] == {"calls": 1, "same": 1, "mismatches": []}
        differs = perf.dual("differs", carried=True)(ModeDependent.apply)
        m = ModeDependent()
        for _ in range(3):
            assert differs(m, np.array([1, 2])) == 3
        stats = perf.DUAL_STATS["differs"]
        assert stats["calls"] == 3 and stats["same"] == 0 and m.calls == 3
        assert kernels.mode() == "reference"  # the candidate's mode does not leak
        assert perf.DUAL_STATS["_modes"] == {"leading": "reference", "candidate": "numpy"}
        kernels.set_mode("numpy")
        with pytest.raises(ValueError):
            same(c, "b")  # a candidate compared with itself proves nothing
        kernels.set_mode("reference")
        monkeypatch.setattr(perf, "_DUAL", "auto")  # "auto" is the reference: refused as well
        with pytest.raises(ValueError):
            same(c, "c")
    finally:
        kernels.set_mode(previous)
