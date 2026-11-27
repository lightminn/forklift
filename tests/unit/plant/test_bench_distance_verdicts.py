"""bench6 verdicts shared by the console and the CLI (plan 4.2 A1b-1 decisions 4, 5, 8): the run verdict accepts a
distance run's own ends, the distance verdict apart, a measuring run's displacement and the rest position."""
from __future__ import annotations

from dataclasses import replace

import pytest

from forklift_core.mcu_link.messages import BenchConfirm, BenchRunFlag, BenchState, Mode
from forklift_core.plant import bench_session as bs

F = BenchRunFlag


def test_counts_diff_is_modular():
    assert bs.counts_diff(-0x80000000, 0x7FFFFFFF) == 1
    assert bs.counts_diff(0x7FFFFFFF, -0x80000000) == -1
    assert bs.counts_diff(5, 5) == 0
    assert bs.counts_diff(-5000, -904) == -4096


def verdict(stop, confirm=0, flags=0, stop_counts=-25, ref=30, cut=-6):
    return bs.distance_verdict(stop_reason=stop, confirm=confirm, run_flags=flags, stop_counts=stop_counts,
                               distance_ref=ref, cut_position=cut)


@pytest.mark.parametrize(("stop", "confirm", "flags", "want"), [
    ("distance_reached", 0, 0, "reached"),
    ("distance_provisional", int(BenchConfirm.REACHED), int(F.RES_TENTATIVE), "reached"),
    ("distance_provisional", int(BenchConfirm.HELD), int(F.RES_TENTATIVE), "held"),
    ("distance_provisional", int(BenchConfirm.HELD), int(F.RES_TENTATIVE | F.RES_LOSS), "failed_loss"),
    ("distance_provisional", 0, int(F.RES_TENTATIVE), "waiting"),
    ("no_progress", 0, 0, "stopped:no_progress"),
    ("kth_zero_suspect", 0, 0, "stopped:kth_zero_suspect"),
    ("kth_lost", 0, int(F.RES_UNCLEAN), "failed_continuity"),
    ("end_zero", 0, 0, "missed"),
    ("time_limit", 0, 0, "missed"),
    ("command_timeout", 0, 0, "missed"),
])
def test_the_distance_verdict(stop, confirm, flags, want):
    v = verdict(stop, confirm, flags)
    assert v["result"] == want, v
    assert v["moved_at_cut"] == -36


def test_a_distance_end_is_a_normal_end_of_a_distance_run_only():
    kw = dict(closed="ended", aborted=None, cleanup_confirmed=True, last=None, record_errors=0)
    assert "stop_reason:distance_reached" not in bs.run_verdict(stop_reason="distance_reached", distance=True,
                                                                **kw)["why"]
    assert "stop_reason:distance_reached" in bs.run_verdict(stop_reason="distance_reached", **kw)["why"]
    assert "stop_reason:no_progress" in bs.run_verdict(stop_reason="no_progress", distance=True, **kw)["why"]


@pytest.mark.parametrize(("flags", "status"), [
    (0, "confirmed"),
    (int(F.RES_TENTATIVE), "provisional"),
    (int(F.RES_UNCLEAN | F.RES_TENTATIVE), "withheld:continuity"),
    (int(F.RES_NO_REF), "withheld:no_reference"),
])
def test_the_measuring_verdict(flags, status):
    v = bs.measure_verdict(run_flags=flags, distance_ref=256, cut_position=0)
    assert v == {"moved": -256, "status": status}


BASE = BenchState(pair_seq=1, run_id=7, result_run_id=7, axis=2, source=1, count_state=1, kth_lost=0, raw_angle=900,
                  cumulative=1000, target=0, distance_ref=900, cut_position=1000, stop_reason=8, distance_confirm=0,
                  blocked=0, dropped_pairs=0, evidence_age_ms=0, end_evidence_age_ms=0, mcu_bypass=1, drive=0x13,
                  conn_gen=0, run_flags=0, result_axis=2, result_source=1)


def series(n, step_us=10_000, **change):
    return [(i * step_us, replace(BASE, **change)) for i in range(n)]


def test_the_rest_position():
    assert bs.rest_position(series(51))["result"] == "settled_at_100hz"
    assert bs.rest_position(series(40))["result"] == "unconfirmed"  # 0.39 s only
    moved = series(51)
    moved[30] = (moved[30][0], replace(BASE, cumulative=1001))
    assert bs.rest_position(moved)["why"] == "not_settled"
    assert bs.rest_position(series(51, run_flags=int(F.UNCLEAN)))["why"] == "continuity"
    assert bs.rest_position(series(51, count_state=2))["why"] == "continuity"
    assert bs.rest_position(series(51, evidence_age_ms=150))["why"] == "evidence_age"
    assert bs.rest_position([])["result"] == "unconfirmed"
    assert Mode.DISARMED  # the caller passes samples after DISARMED
