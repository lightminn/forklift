"""Choosing tau on dev (stage 3 plan, task 5)."""
from tools import choose_tau as C


def _metrics(miss, neg, extra=0, deferred=0, target=0, n=100):
    def g(f, bound):
        return {"n": n, "failures": f, "bound": bound}
    return {"gates": {"known:epal6:miss": g(miss, 0.05), "negative:negative_fp": g(neg, 0.10),
                      "known:epal6:extra": g(extra, 0.10), "known:epal6:deferred_unambiguous": g(deferred, 0.10),
                      "known:epal6:target_error": g(target, 0.05)}}


def test_the_lowest_tau_meeting_every_bound_is_chosen():
    runs = [(0.2, _metrics(2, 15)), (0.4, _metrics(3, 8)), (0.6, _metrics(4, 5)), (0.8, _metrics(9, 2))]
    out = C.choose(runs)
    assert out["chosen"] == 0.4 and out["met"]


def test_without_a_meeting_tau_the_least_excess_is_chosen_with_its_shortfalls():
    runs = [(0.2, _metrics(10, 30)), (0.5, _metrics(12, 12)), (0.8, _metrics(30, 2))]
    out = C.choose(runs)
    assert out["chosen"] == 0.5 and not out["met"] and {k.rsplit(":", 1)[-1] for k in out["shortfalls"]} == {"miss", "negative_fp"}


def test_the_request_rate_counts_against_a_tau():
    # Codex v2: a tau that defers every unambiguous instruction must not pass
    runs = [(0.3, _metrics(1, 5, deferred=40)), (0.5, _metrics(2, 5, deferred=5))]
    assert C.choose(runs)["chosen"] == 0.5


def test_one_population_over_its_bound_fails_the_tau_even_when_the_pool_is_under():
    # Codex review 1: EPAL6 8/100 missed with two other populations at 0/100 pooled to 2.67 % and passed
    def runs_with(epal_miss):
        m = _metrics(0, 5)
        m["gates"]["known:t11_06:miss"] = {"n": 100, "failures": 0, "bound": 0.05}
        m["gates"]["s7:miss"] = {"n": 100, "failures": 0, "bound": 0.05}
        m["gates"]["known:epal6:miss"] = {"n": 100, "failures": epal_miss, "bound": 0.05}
        return m
    out = C.choose([(0.2, runs_with(8)), (0.4, runs_with(3))])
    assert out["chosen"] == 0.4 and out["met"]
    out = C.choose([(0.2, runs_with(8))])
    assert not out["met"] and "known:epal6:miss" in out["shortfalls"]


def test_a_gate_without_samples_is_a_shortfall_not_a_pass():
    m = _metrics(0, 5)
    del m["gates"]["negative:negative_fp"]
    m["gates"]["known:epal6:target_error"]["n"] = 0
    out = C.choose([(0.5, m)])
    assert not out["met"] and {"negative_fp", "target_error"} <= {k.rsplit(":", 1)[-1] for k in out["missing"]}


def test_a_population_gate_without_samples_is_missing_even_when_another_population_has_it():
    # Codex re-review 1: T11 miss at n=0 vanished while EPAL6's miss stood in for it -> met with nothing missing
    full = _metrics(0, 5)
    full["gates"]["known:t11_06:miss"] = {"n": 100, "failures": 0, "bound": 0.05}
    empty = _metrics(0, 5)
    empty["gates"]["known:t11_06:miss"] = {"n": 0, "failures": 0, "bound": 0.05}
    out = C.choose([(0.3, empty), (0.5, full)])
    assert out["chosen"] == 0.5 and out["met"]
    out = C.choose([(0.3, empty)])
    assert not out["met"] and "known:t11_06:miss" in out["missing"]
