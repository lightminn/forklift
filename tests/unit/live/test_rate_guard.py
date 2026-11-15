"""D5 3.11: the MCU clock rate checked online from the delay bound's anchors (interval arithmetic: path delay and
its asymmetry stay inside the bounds)."""

import random

import pytest

from forklift_core.live.real_teleop import RATE_OUT, RATE_RHO_MAX, RATE_UNVERIFIED, RATE_VERIFIED, RateGuard

MS = 1_000_000


def anchors(rate, *, seconds, every_s=0.5, path_ms=lambda i: (2.0, 8.0), boot=7, t0=50_000 * MS):
    """Anchors of an MCU clock running ``rate`` x host time: each is a frame written at w, its MCU instant at
    w + out (MCU time = rate x host), received at that instant + back; bound = rx - w."""
    out = []
    for i in range(int(seconds / every_s)):
        w = t0 + int(i * every_s * 1e9)
        o, b = path_ms(i)
        inst = w + int(o * MS)
        rx = inst + int(b * MS)
        m = int((inst - t0) * rate) + 1_000_000_000
        out.append((rx - m, rx, m, rx - w, boot))
    return out


def feed(g, seq):
    for a in seq:
        g.add(a)
    return g.verdict


def test_a_true_clock_is_verified_once_the_baseline_is_long_enough():
    g = RateGuard(RATE_RHO_MAX)
    seq = anchors(1.0, seconds=60)
    verdicts = []
    for a in seq:
        g.add(a)
        verdicts.append(g.verdict)
    assert verdicts[-1] == RATE_VERIFIED and RATE_OUT not in verdicts
    first = verdicts.index(RATE_VERIFIED)
    # each side ~ b / baseline: 10 ms / 0.001 = 10 s of baseline before the verdict
    assert 9 <= first * 0.5 <= 13
    have, need = g.progress_s()
    assert have > need


@pytest.mark.parametrize("rate", [0.98, 0.995, 1.01, 1.02])
def test_a_clock_outside_the_bound_is_found_and_stays_found(rate):
    g = RateGuard(RATE_RHO_MAX)
    assert feed(g, anchors(rate, seconds=60)) == RATE_OUT
    assert feed(g, anchors(1.0, seconds=10, t0=200_000 * MS)) == RATE_OUT  # for the rest of the boot


def test_an_hsi_like_error_of_0_4_percent_is_out_of_range():
    assert feed(RateGuard(RATE_RHO_MAX), anchors(1.004, seconds=60)) == RATE_OUT


@pytest.mark.parametrize("rate", [1.0, 1.0001, 0.9999])
def test_asymmetric_and_varying_path_delay_never_makes_a_good_clock_look_bad(rate):
    rnd = random.Random(3)
    path = lambda i: (rnd.uniform(0.5, 40.0), rnd.uniform(0.5, 4.0))  # noqa: E731 -- the send side alone varies
    g = RateGuard(RATE_RHO_MAX)
    for a in anchors(rate, seconds=120, path_ms=path):
        g.add(a)
        assert g.verdict != RATE_OUT
        if g.range is not None:
            assert g.range[0] <= rate <= g.range[1]


def test_a_new_boot_starts_over():
    g = RateGuard(RATE_RHO_MAX)
    assert feed(g, anchors(1.02, seconds=60)) == RATE_OUT
    g.add(anchors(1.0, seconds=1, boot=8)[0])
    assert g.verdict == RATE_UNVERIFIED


def test_the_far_end_is_the_tightest_early_anchor():
    g = RateGuard(RATE_RHO_MAX)
    loose = anchors(1.0, seconds=1, path_ms=lambda i: (10.0, 10.0))[0]
    tight = anchors(1.0, seconds=1, path_ms=lambda i: (1.0, 1.0), t0=50_000 * MS + 500 * MS)[0]
    g.add(loose)
    g.add(tight)
    assert g._ref[2] == tight[3]


def test_rho_bounds():
    for bad in (0.0, -1.0, 0.2):
        with pytest.raises(ValueError):
            RateGuard(bad)
