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
    assert out["chosen"] == 0.5 and not out["met"] and set(out["shortfalls"]) == {"miss", "negative_fp"}


def test_the_request_rate_counts_against_a_tau():
    # Codex v2: a tau that defers every unambiguous instruction must not pass
    runs = [(0.3, _metrics(1, 5, deferred=40)), (0.5, _metrics(2, 5, deferred=5))]
    assert C.choose(runs)["chosen"] == 0.5
