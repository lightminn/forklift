"""G7 clock fit (docs/design/2026-11-14-teleop-record-state-markers-clock.md 5): the lower delay envelope between a
device clock (µs) and the host monotonic clock (ns), its quality figures, the clean-up rules and the splits."""

from __future__ import annotations

import random

import pytest

from forklift_core.plant import clock_map as cm


def synth(n=500, span_s=10.0, drift_ppm=0.0, offset_ns=5_000_000_000, seed=0, delay=None):
    """Device times 0..span every span/n, host = offset + device*1000*(1+drift) + delay (ns)."""
    rng = random.Random(seed)
    dev, host = [], []
    for i in range(n):
        d_us = int(i * span_s * 1e6 / n)
        lag = delay(i, rng) if delay else 200_000 + int(rng.expovariate(1 / 2_000_000))  # 0.2 ms + exp(2 ms)
        dev.append(d_us)
        host.append(int(offset_ns + d_us * 1000 * (1 + drift_ppm * 1e-6) + lag))
    return dev, host


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
@pytest.mark.parametrize("drift", [-50.0, 0.0, 50.0])
def test_a_steady_delay_recovers_the_drift_and_the_split_quality_agrees(seed, drift):
    dev, host = synth(drift_ppm=drift, seed=seed)
    f = cm.fit_clock(dev, host)
    assert f.valid, f.reason
    assert abs(f.drift_ppm - drift) < 5.0
    # halves of 5 s with millisecond delays: tens of ppm apart at most (a delay trend reads hundreds, below)
    assert abs(f.drift_split_ppm) < 80.0
    # every point is on or above the envelope, and the envelope follows the smallest delay (0.2 ms)
    assert min(f.residuals_ns) >= 0
    assert f.delay_ns["p50"] > 0 and f.delay_ns["max"] >= f.delay_ns["p95"] >= f.delay_ns["p50"]
    mid = dev[len(dev) // 2]
    assert abs(f.to_host_ns(mid)[0] - (5_000_000_000 + mid * 1000 * (1 + drift * 1e-6) + 200_000)) < 1_000_000


def test_a_delay_trend_goes_into_the_drift_and_the_split_figures_show_it():
    # no drift, delay rising 10 -> 15 ms: the envelope takes the trend as drift and nothing tells it apart (design 5);
    # a step in the delay half way is also taken as drift, but the whole then disagrees with its halves
    dev, host = synth(n=501, span_s=5.1, delay=lambda i, rng: 10_000_000 + i * 10_000)
    f = cm.fit_clock(dev, host)
    assert f.valid and f.drift_ppm > 500  # the trend reads as drift ...
    assert max(f.residuals_ns) < 1_000  # ... with nothing above the envelope ...
    # ... and the halves take the same wrong slope: every figure stays near zero (indistinguishable from drift)
    assert abs(f.drift_split_ppm) < 5 and f.drift_vs_halves_ppm < 5 and abs(f.split_end_ns) < 1_000
    dev2, host2 = synth(n=500, span_s=10.0, delay=lambda i, rng: (2_000_000 if i < 250 else 12_000_000)
                        + int(rng.expovariate(1 / 500_000)))
    g = cm.fit_clock(dev2, host2)
    # each half is flat, the whole takes the step as drift: the whole disagrees with both halves
    assert g.drift_ppm > 500 and abs(g.drift_split_ppm) < 80 and g.drift_vs_halves_ppm > 500
    # a steady delay keeps the whole and the halves together
    dev3, host3 = synth(seed=3)
    assert cm.fit_clock(dev3, host3).drift_vs_halves_ppm < 80


def test_clean_up_rules_device_and_host_going_back_and_repeats():
    dev = [10, 8, 9, 11, 12, 12, 13, 15]
    host = [100, 101, 102, 103, 105, 104, 99, 107]
    kept, dropped = cm.clean_points(dev, host)
    # 8 and 9 are below the last accepted device time (10); device 12 repeats -> the earliest host (104) stays;
    # host 99 goes back (device 13 dropped with it)
    assert kept == [(10, 100), (11, 103), (12, 104), (15, 107)]
    assert dropped == {"device_back": 2, "host_back": 1, "device_repeat": 1}


def test_validity_bounds_are_twenty_points_and_five_seconds_after_clean_up():
    dev, host = synth(n=20, span_s=5.0 * 20 / 19)  # 20 points spanning exactly 5 s
    edge = cm.fit_clock(dev, host)
    assert edge.valid
    # its halves (10 points, about 2.4 s each) are too short for the halves' figures: None with a reason
    assert edge.drift_split_ppm is None and edge.drift_vs_halves_ppm is None and "half" in edge.split_reason
    assert not cm.fit_clock(dev[:19], host[:19]).valid
    dev2, host2 = synth(n=40, span_s=4.9)
    f = cm.fit_clock(dev2, host2)
    assert not f.valid and "span" in f.reason
    with pytest.raises(ValueError):
        f.to_host_ns(1_000)


def test_collinear_points_give_one_deterministic_envelope_and_outside_the_span_is_flagged():
    dev = [i * 100_000 for i in range(60)]
    host = [1_000_000 + d * 1000 for d in dev]  # all on one line
    a = cm.fit_clock(dev, host)
    b = cm.fit_clock(list(dev), list(host))
    assert (a.offset_ns, a.slope) == (b.offset_ns, b.slope)
    assert a.to_host_ns(dev[10])[1] is False
    assert a.to_host_ns(dev[-1] + 1_000_000)[1] is True  # extrapolated


def test_rows_split_by_host_boot_and_mcu_boot():
    rows = [{"t": "header", "host_boot_id": "h1"}]
    for boot in (1, 2):
        dev, host = synth(n=100, span_s=6.0, seed=boot, offset_ns=boot * 10**10)
        rows += [{"t": "state", "boot_id": boot, "mcu_time_us": d, "read_ns": h, "host_ns": h - 5} for d, h in
                 zip(dev, host)]
    rows.append({"t": "header", "host_boot_id": "h2"})
    dev, host = synth(n=100, span_s=6.0, seed=9)
    rows += [{"t": "state", "boot_id": 2, "mcu_time_us": d, "read_ns": h, "host_ns": h} for d, h in zip(dev, host)]
    rows.append({"phase": "idle"})  # a cycle row is ignored
    fits = cm.clock_fits_from_rows(rows)
    assert [(f.host_boot, f.boot_id) for f in fits] == [("h1", 1), ("h1", 2), ("h2", 2)]
    assert all(f.valid for f in fits) and all(f.n == 100 for f in fits)
    # a bringup record has no read_ns: host_ns is used there
    plain = [{"t": "state", "boot_id": 1, "mcu_time_us": r["mcu_time_us"], "host_ns": r["read_ns"]} for r in rows
             if r.get("t") == "state" and r["boot_id"] == 1]
    assert cm.clock_fits_from_rows(plain)[0].valid


def test_a_repeated_device_time_never_brings_back_a_host_time_before_the_previous_point():
    # Codex design review 3: a repeat with an earlier host time replaced the last point even when that time went
    # before the point ahead of it -- a host going back slipped through and the drift read -333,333 ppm
    dev, host = synth(n=60, span_s=6.0)
    base = cm.fit_clock(dev, host)
    i = next(k for k, d in enumerate(dev) if d >= 3_000_000)
    dev.insert(i + 1, dev[i])
    host.insert(i + 1, host[i - 1] - 1_000_000)  # earlier than the point before the repeated one
    kept, dropped = cm.clean_points(dev, host)
    assert all(b[1] >= a[1] for a, b in zip(kept, kept[1:]))
    assert dropped["device_repeat"] == 1
    f = cm.fit_clock(dev, host)
    # the repeat is dropped without touching the kept point: the same fit as without it
    assert f.valid and (f.offset_ns, f.slope) == (base.offset_ns, base.slope)


def test_the_bring_up_clock_command_reads_a_record_without_markers(tmp_path, capsys):
    import json
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
    import bringup

    dev, host = synth(n=200, span_s=8.0, drift_ppm=20.0)
    rec = tmp_path / "teleop.jsonl"
    rows = [{"t": "header", "host_boot_id": "hb"}] + [
        {"t": "state", "boot_id": 3, "mcu_time_us": d, "host_ns": h - 7_000_000, "read_ns": h} for d, h in
        zip(dev, host)] + [{"phase": "ready"}]
    rec.write_text("".join(json.dumps(r) + "\n" for r in rows))
    assert bringup.main(["clock", "--record", str(rec)]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["result"] == "ok" and len(out["fits"]) == 1
    f = out["fits"][0]
    assert f["valid"] and f["boot_id"] == 3 and f["host_boot"] == "hb" and abs(f["drift_ppm"] - 20.0) < 5.0
    (tmp_path / "empty.jsonl").write_text(json.dumps({"phase": "idle"}) + "\n")
    assert bringup.main(["clock", "--record", str(tmp_path / "empty.jsonl")]) == 0
    assert json.loads(capsys.readouterr().out)["result"] == "no_state_rows"
