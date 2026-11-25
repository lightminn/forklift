"""PR 7 design 5 (7b): the real D435i accelerometer source -- the callback only queues, the loop reads
``(gen, lo, hi, accel)`` with ``[receipt - bound, receipt]`` intervals, generations on restart / clock mismatch /
receive gap / overflow / loss, a wrap is a forward step, diagnostic until the delay bound is measured, and a record's
raw fields replay to the same rows."""

import json
from dataclasses import replace
from pathlib import Path

import pytest

from forklift_core.safety.real_watch import ImuWatch, load_imu_config
from forklift_core.sensors.imu_source import (
    IMU_YAML,
    WRAP_US,
    ImuSource,
    fits_watch,
    load_imu_source_config,
    replay,
)

MS = 1_000_000
G = (0.0, 0.0, 9.81)
CFG = load_imu_source_config()
MEASURED = replace(CFG, delay_bound_s=0.02, measured=True)
SAFETY = Path(IMU_YAML).with_name("safety_watch.yaml")


class Clock:
    def __init__(self, ns=0):
        self.ns = ns

    def __call__(self):
        return self.ns


def src(cfg=MEASURED, **kw):
    clock = Clock()
    return ImuSource(cfg, operating=cfg.measured, now_ns=clock, **kw), clock


def feed(s, host0_ms, dev0_us, n, *, step_ms=5, dev_step_us=5000):
    for k in range(n):
        s.push((host0_ms + k * step_ms) * MS, "a", (dev0_us + k * dev_step_us) % WRAP_US, "hardware_clock", G)


# --- the shipped config --------------------------------------------------------------------------------------------
def test_the_shipped_config_is_diagnostic_and_no_operating_source_can_be_built():
    assert CFG.delay_bound_s is None and not CFG.measured
    with pytest.raises(ValueError, match="delay_unmeasured"):
        ImuSource(CFG, operating=True)
    s = ImuSource(CFG, operating=False)
    assert s.bound_ns == int(CFG.provisional_delay_bound_s * 1e9)


def test_a_measured_flag_without_a_bound_is_refused(tmp_path):
    raw = Path(IMU_YAML).read_text().replace("measured: false", "measured: true")
    p = tmp_path / "imu.yaml"
    p.write_text(raw)
    with pytest.raises(ValueError, match="measured without"):
        load_imu_source_config(p)


def test_the_bound_must_leave_the_watch_its_gap():
    """delay + one 200 Hz period within the watch's 50 ms: 45 ms fits, 46 ms would make every sample a gap."""
    gap = load_imu_config(SAFETY).gap_max_s
    assert gap == 0.05
    assert fits_watch(CFG, gap, bound_s=0.045) and not fits_watch(CFG, gap, bound_s=0.046)
    assert fits_watch(CFG, gap, bound_s=CFG.provisional_delay_bound_s)
    assert not fits_watch(CFG, gap)  # unmeasured: nothing fits


# --- intervals and order -------------------------------------------------------------------------------------------
def test_intervals_are_receipt_minus_the_bound_to_receipt():
    s, _ = src()
    feed(s, 100, 0, 3)
    out = s.samples(200 * MS, 64)
    assert [(g, lo, hi) for g, lo, hi, _ in out] == [(1, (t - 20) * MS, t * MS) for t in (100, 105, 110)]
    assert out[0][3] == G


def test_only_samples_received_up_to_the_read_and_at_most_limit():
    s, _ = src()
    feed(s, 100, 0, 5)
    assert [hi for _, _, hi, _ in s.samples(110 * MS, 64)] == [100 * MS, 105 * MS, 110 * MS]
    feed(s, 125, 25_000, 5)
    assert len(s.samples(10_000 * MS, 2)) == 2 and len(s.samples(10_000 * MS, 64)) == 5


def test_gyro_and_missing_device_times_are_dropped_and_counted():
    s, _ = src()
    s.push(100 * MS, "g", 0, "hardware_clock", (0.1, 0.0, 0.0))
    s.push(101 * MS, "a", None, "hardware_clock", G)
    s.push(102 * MS, "a", WRAP_US, "hardware_clock", G)
    s.push(103 * MS, "a", 5, "hardware_clock", G)
    assert len(s.samples(200 * MS, 64)) == 1
    assert s.counts["gyro"] == 1 and s.counts["missing_dev"] == 2


def test_a_repeated_device_time_and_a_small_step_back_are_dropped():
    s, _ = src()
    s.push(100 * MS, "a", 50_000, "hardware_clock", G)
    s.push(105 * MS, "a", 50_000, "hardware_clock", G)  # the same sample again
    s.push(106 * MS, "a", 45_000, "hardware_clock", G)  # 5 ms back: late, not a restart
    s.push(110 * MS, "a", 60_000, "hardware_clock", G)
    out = s.samples(200 * MS, 64)
    assert [hi for _, _, hi, _ in out] == [100 * MS, 110 * MS]
    assert s.counts["dup"] == 1 and s.counts["late"] == 1 and s.generation() == 1


def test_a_wrap_is_a_forward_step_in_the_same_generation():
    s, _ = src()
    feed(s, 100, WRAP_US - 7_500, 4)  # ..., 2**32 - 2500, 2500, 7500
    out = s.samples(200 * MS, 64)
    assert len(out) == 4 and {g for g, *_ in out} == {1} and s.counts["generations"] == 0


# --- generations ---------------------------------------------------------------------------------------------------
def test_a_device_restart_opens_a_generation():
    s, _ = src()
    feed(s, 100, 5_000_000, 3)
    feed(s, 115, 0, 2)  # the device time back 5 s: a restart
    out = s.samples(200 * MS, 64)
    assert [g for g, *_ in out] == [1, 1, 1, 2, 2] and s.generation_why == "device_restart"


def test_device_and_host_steps_apart_by_more_than_the_bound_open_a_generation():
    s, _ = src()
    feed(s, 100, 0, 2)
    s.push((105 + 5 + 22) * MS, "a", 10_000, "hardware_clock", G)  # device 5 ms on, host 27 ms: 22 > 20 + 1
    s.push((105 + 5 + 22 + 5) * MS, "a", 15_000, "hardware_clock", G)
    out = s.samples(500 * MS, 64)
    assert [g for g, *_ in out] == [1, 1, 2, 2] and s.generation_why == "clock_mismatch"


def test_jitter_within_the_bound_is_no_generation():
    s, _ = src()
    feed(s, 100, 0, 2)
    s.push((105 + 5 + 19) * MS, "a", 10_000, "hardware_clock", G)  # late by 19 ms, within 20
    s.push((105 + 5 + 19 + 1) * MS, "a", 15_000, "hardware_clock", G)  # a burst: 1 ms after it
    assert {g for g, *_ in s.samples(500 * MS, 64)} == {1}


def test_a_receive_gap_seen_with_no_sample_opens_a_generation_once():
    """design 5: the restart reaches the loop with no new sample; the sample that ends the gap opens none again."""
    s, clock = src()
    feed(s, 100, 0, 3)
    s.samples(110 * MS, 64)
    clock.ns = 110 * MS + 100 * MS
    assert s.generation() == 1  # 100 ms: not over the limit
    clock.ns += 1
    assert s.generation() == 2 and s.generation_why == "receive_gap"
    clock.ns += 50 * MS
    assert s.generation() == 2
    feed(s, 300, 200_000, 2)
    assert [g for g, *_ in s.samples(400 * MS, 64)] == [2, 2] and s.generation() == 2


def test_a_receive_gap_seen_only_when_the_next_sample_is_read_opens_a_generation():
    s, _ = src()
    feed(s, 100, 0, 2)
    feed(s, 300, 200_000, 2)
    assert [g for g, *_ in s.samples(400 * MS, 64)] == [1, 1, 2, 2]


def test_a_queue_overflow_drops_the_oldest_and_opens_a_generation_after_the_hole():
    s, _ = src(replace(MEASURED, queue_max=4))
    feed(s, 100, 0, 6)
    out = s.samples(400 * MS, 64)
    assert [hi // MS for _, _, hi, _ in out] == [110, 115, 120, 125]
    assert s.counts["overflow"] == 2 and [g for g, *_ in out] == [2, 2, 2, 2]


def test_a_reported_loss_opens_a_generation_on_the_next_read():
    s, _ = src()
    feed(s, 100, 0, 2)
    s.samples(200 * MS, 64)
    s.lost("device_removed")
    assert s.generation() == 2 and s.generation_why == "device_removed"


# --- with the watch ------------------------------------------------------------------------------------------------
def _watch_on(bound_s):
    s, clock = src(replace(MEASURED, delay_bound_s=bound_s))
    w = ImuWatch(load_imu_config(SAFETY))
    feed(s, 1000, 0, 20)
    clock.ns = 1095 * MS
    for item in s.samples(clock.ns, 64):
        w.add_interval(*item, clock.ns)
    w.note_generation(s.generation())
    base, why = w.baseline(900 * MS)
    if why is not None:
        return why
    w.arm(base)
    w.settled(1095 * MS)
    feed(s, 1100, 100_000, 20)
    trips = []
    for k in range(20):
        clock.ns = (1100 + 5 * k) * MS
        for item in s.samples(clock.ns, 64):
            w.add_interval(*item, clock.ns)
        w.note_generation(s.generation())
        trips.append(w.judge(clock.ns))
    return trips


def test_an_armed_watch_holds_on_a_source_whose_bound_fits():
    assert set(_watch_on(0.045)) == {None}


def test_a_bound_that_does_not_fit_leaves_no_baseline_to_arm_on():
    assert _watch_on(0.046) == "imu_no_baseline"


# --- records -------------------------------------------------------------------------------------------------------
def _stream():
    s, _ = src(record=True)
    feed(s, 100, WRAP_US - 10_000, 4)  # across a wrap
    s.push(121 * MS, "a", 5_000, "hardware_clock", G)  # dup of the last
    s.push(122 * MS, "a", 1_000, "hardware_clock", G)  # 4 ms back: late
    s.push(123 * MS, "a", None, "hardware_clock", G)
    feed(s, 125, 900_000, 3)  # 895 ms forward on the device, 10 ms on the host: clock mismatch
    feed(s, 140, 0, 3)  # back 900 ms: a restart
    s.samples(1_000 * MS, 64)
    return s.drain_rows()


def test_every_accel_sample_is_a_row_with_its_verdict_and_interval():
    rows = _stream()
    imu = [r for r in rows if r["t"] == "imu"]
    assert [r["verdict"] for r in imu] == ["ok"] * 4 + ["dup", "late", "missing_dev"] + ["ok"] * 6
    assert [r["why"] for r in rows if r["t"] == "imu_gen"] == ["clock_mismatch", "device_restart"]
    assert all((r["lo"], r["hi"]) == (r["host_ns"] - 20 * MS, r["host_ns"]) for r in imu if r["verdict"] == "ok")


def test_a_records_raw_fields_replay_to_the_same_rows(tmp_path):
    rows = _stream()
    path = tmp_path / "observe.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    back = [json.loads(line) for line in path.read_text().splitlines()]
    again = replay(MEASURED, back).drain_rows()
    assert [r for r in again if r["t"] == "imu"] == [r for r in rows if r["t"] == "imu"]
    assert [r["why"] for r in again if r["t"] == "imu_gen"] == ["clock_mismatch", "device_restart"]


def test_an_imu_record_file_replays_its_accel_rows():
    """``tools/imu_record.py`` rows (``s`` names the stream, gyro interleaved)."""
    rows = [{"t": "header", "schema": "imu_record/1"}]
    for k in range(6):
        rows.append({"t": "imu", "s": "g", "dev_us": 2500 * k, "ts_ms": 0.0, "dom": "hardware_clock",
                     "host_ns": (100 + 2.5 * k) * MS, "x": 0.0, "y": 0.0, "z": 0.0})
        if k % 2 == 0:
            rows.append({"t": "imu", "s": "a", "dev_us": 2500 * k, "ts_ms": 0.0, "dom": "hardware_clock",
                         "host_ns": int((100 + 2.5 * k) * MS) + 1, "x": 0.0, "y": -9.81, "z": 0.0})
    s = replay(MEASURED, rows)
    out = [r for r in s.drain_rows() if r["t"] == "imu"]
    assert len(out) == 3 and all(r["verdict"] == "ok" for r in out) and s.counts["gyro"] == 6
