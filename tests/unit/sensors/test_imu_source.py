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
    feed(s, 115, 0, 6)  # the device time back 5 s: a restart, a boundary at the previous receipt (110 ms)
    out = s.samples(200 * MS, 64)
    # the new clock's samples count from lo >= 110 ms (round 3 P1-1): 115-125 ms may have been taken before it
    assert [g for g, *_ in out] == [1, 1, 1, 2, 2, 2] and s.generation_why == "device_restart"
    assert s.counts["before_boundary"] == 3


def test_a_delay_over_the_bound_opens_a_generation_and_gives_nothing_more():
    """Codex 7b round 1 P1-3: device 5 ms on, host 27 ms -- 22 ms over 20 + 1. The interval [receipt - bound,
    receipt] no longer holds: nothing from here on, the rest recorded as bound_broken."""
    s, _ = src()
    feed(s, 100, 0, 2)
    s.push((105 + 5 + 22) * MS, "a", 10_000, "hardware_clock", G)
    feed(s, 137, 15_000, 5)
    out = s.samples(500 * MS, 64)
    assert [g for g, *_ in out] == [1, 1] and s.generation_why == "clock_mismatch" and s.generation() == 2
    assert s.bound_broken and s.counts["bound_broken"] == 6


def test_the_mismatch_threshold_is_the_bound_plus_1_ms_plus_100_ppm():
    """5 ms of device time apart: 20 + 1 ms + 0.5 µs of drift allowed."""
    for extra_ns, broken in ((0, False), (400, False), (1_000, True)):
        s, _ = src()
        s.push(100 * MS, "a", 0, "hardware_clock", G)
        s.push(105 * MS + 21 * MS + extra_ns, "a", 5_000, "hardware_clock", G)
        s.samples(500 * MS, 64)
        assert s.bound_broken is broken, extra_ns


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
    # the hole covers 100-105 ms: samples that may have been taken before its end (lo < 105 ms) do not count
    # (round 2 P1-2, round 3 P2)
    assert [hi // MS for _, _, hi, _ in out] == [125] and s.counts["before_boundary"] == 3
    assert s.counts["overflow"] == 2 and [g for g, *_ in out] == [2]


def test_a_reported_loss_opens_a_generation_on_the_next_read():
    s, _ = src()
    feed(s, 100, 0, 2)
    s.samples(200 * MS, 64)
    s.lost("device_removed")
    assert s.generation() == 2 and s.generation_why == "lost:device_removed"


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
    feed(s, 125, WRAP_US - 100_000, 6)  # 105 ms back: a restart (boundary at 123 ms, the previous receipt)
    feed(s, 177, WRAP_US - 70_000, 2)  # device 5 ms on, host 27 ms: the bound broken
    s.samples(1_000 * MS, 64)
    return s.drain_rows()


def test_every_accel_sample_is_a_row_with_its_verdict_and_interval():
    rows = _stream()
    imu = [r for r in rows if r["t"] == "imu"]
    assert [r["verdict"] for r in imu] == ["ok"] * 4 + ["dup", "late", "missing_dev"] + ["before_boundary"] * 4 \
        + ["ok"] * 2 + ["bound_broken"] * 2
    assert [r["why"] for r in rows if r["t"] == "imu_gen"] == ["device_restart", "clock_mismatch"]
    assert all((r["lo"], r["hi"]) == (r["host_ns"] - 20 * MS, r["host_ns"]) for r in imu if r["verdict"] == "ok")


def test_a_records_raw_fields_replay_to_the_same_rows(tmp_path):
    rows = _stream()
    path = tmp_path / "observe.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    back = [json.loads(line) for line in path.read_text().splitlines()]
    again = replay(MEASURED, back).drain_rows()
    assert [r for r in again if r["t"] == "imu"] == [r for r in rows if r["t"] == "imu"]
    assert [r["why"] for r in again if r["t"] == "imu_gen"] == ["device_restart", "clock_mismatch"]


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


# --- Codex 7b round 1 ------------------------------------------------------------------------------------------------
def test_a_loss_is_a_boundary_after_the_samples_already_received():
    """P1-2: ten samples queued, then the device lost: they keep generation 1; the loss opens 2 with its time."""
    s, clock = src(record=True)
    feed(s, 100, 0, 10)
    clock.ns = 160 * MS
    s.lost("unplug")
    out = s.samples(200 * MS, 64)
    assert [g for g, *_ in out] == [1] * 10 and s.generation() == 2
    gens = [r for r in s.drain_rows() if r["t"] == "imu_gen"]
    assert [(r["why"], r["host_ns"]) for r in gens] == [("lost:unplug", 160 * MS)]


def test_a_loss_with_no_sample_waiting_reaches_generation_at_once():
    s, clock = src()
    feed(s, 100, 0, 2)
    s.samples(200 * MS, 64)
    clock.ns = 110 * MS
    s.lost("unplug")
    assert s.generation() == 2


def test_an_overflow_hole_is_seen_even_when_no_sample_after_it_is_due_yet():
    """P1-2: queue_max 1 -- the 104 ms sample pushed out by the 106 ms one; a read up to 105 ms sees the hole."""
    s, clock = src(replace(MEASURED, queue_max=1))
    s.push(104 * MS, "a", 0, "hardware_clock", G)
    s.push(106 * MS, "a", 2_000, "hardware_clock", G)
    assert s.samples(105 * MS, 64) == [] and s.generation_why == "overflow"
    clock.ns = 105 * MS
    assert s.generation() == 2


def test_a_quiet_tick_with_old_samples_waiting_opens_no_receive_gap_twice():
    s, clock = src()
    feed(s, 100, 0, 1)
    s.samples(100 * MS, 64)
    s.push(300 * MS, "a", 200_000, "hardware_clock", G)
    clock.ns = 250 * MS
    assert s.generation() == 1  # a sample is waiting: its read decides
    assert [g for g, *_ in s.samples(400 * MS, 64)] == [2]
    clock.ns = 320 * MS
    assert s.generation() == 2 and s.counts["generations"] == 1


def test_a_read_takes_at_most_limit_raw_samples_whatever_their_verdict():
    """P2: 400 samples without a device time ahead of a good one: each read takes `limit` and says it was capped."""
    s, _ = src()
    for k in range(399):  # under queue_max: no overflow boundary
        s.push((100 + k) * MS // 10, "a", None, "hardware_clock", G)
    s.push(200 * MS, "a", 5_000, "hardware_clock", G)
    assert s.samples(300 * MS, 1) == [] and s.last_read_capped
    assert s.counts["missing_dev"] == 1
    assert len(s.samples(300 * MS, 1000)) == 1 and not s.last_read_capped


def test_a_receipt_going_back_is_never_given():
    """P2: (100 ms, dev 0) then (99 ms, dev 30 ms) -- the second is not newer, whatever the device time."""
    s, _ = src()
    s.push(100 * MS, "a", 0, "hardware_clock", G)
    s.push(99 * MS, "a", 30_000, "hardware_clock", G)
    assert [hi for _, _, hi, _ in s.samples(200 * MS, 64)] == [100 * MS] and s.counts["not_newer"] == 1


def test_without_metadata_the_device_clock_timestamp_is_the_device_time():
    """The Jetson's RSUSB build gave no frame metadata: get_timestamp() in the device clock domain stands in."""
    s, _ = src()
    for k in range(3):
        s.push((100 + 5 * k) * MS, "a", None, "hardware_clock", G, ts_ms=401491.329 + 5 * k)
    s.push(116 * MS, "a", None, "system_time", G, ts_ms=1.0)  # not the device clock: no device time
    assert len(s.samples(200 * MS, 64)) == 3
    assert s.counts["dev_from_ts"] == 3 and s.counts["missing_dev"] == 1


JETSON = Path(__file__).resolve().parents[3] / "docs/validation/assets/hw-day-2026-11-17/jetson" \
    / "imu_2026-11-17_jetson.jsonl.xz"


def test_the_jetson_record_of_2026_11_17_replays():
    """The real imu_record/1 file (120 s, every dev_us null): every accel sample is taken on its device-clock
    timestamp under the provisional 30 ms bound, one generation; under 10 ms the bound breaks at once."""
    import lzma

    with lzma.open(JETSON, "rt") as f:
        rows = [json.loads(line) for line in f]
    s = replay(CFG, rows)
    assert s.counts["accepted"] == s.counts["dev_from_ts"] == 24060 and s.gen == 1
    assert s.counts["gyro"] == 48071
    assert replay(replace(CFG, provisional_delay_bound_s=0.01), rows).bound_broken


def _ok_gaps(rows, limit_ns=50 * MS):
    ok = [r for r in rows if r["t"] == "imu" and r["verdict"] == "ok"]
    return sum(1 for x, y in zip(ok, ok[1:], strict=False) if y["hi"] - x["lo"] > limit_ns)


def test_the_jetson_record_counts_its_watch_gaps_per_bound():
    """Codex 7b round 2 P2: next hi - previous lo over 50 ms -- 7 times under 30 ms, none under 20 ms."""
    import lzma

    with lzma.open(JETSON, "rt") as f:
        rows = [json.loads(line) for line in f]
    assert _ok_gaps(replay(CFG, rows).drain_rows()) == 7
    s20 = replay(replace(CFG, provisional_delay_bound_s=0.02), rows)
    assert s20.counts["accepted"] == 24060 and _ok_gaps(s20.drain_rows()) == 0


# --- Codex 7b round 2 ------------------------------------------------------------------------------------------------
def test_a_loss_behind_a_waiting_sample_is_in_the_generation_at_once():
    """P1-1: after a read, one old sample is received and then the device is lost: generation() says 2 now (the
    send gate holds), and the old sample still comes as generation 1."""
    s, clock = src()
    feed(s, 100, 0, 2)
    s.samples(105 * MS, 64)
    s.push(105 * MS + 1, "a", 5_001, "hardware_clock", G)
    clock.ns = 105 * MS + 2
    s.lost("unplug")
    assert s.generation() == 2
    assert [g for g, *_ in s.samples(200 * MS, 64)] == [1] and s.gen == 2


def test_a_sample_that_may_predate_a_boundary_is_not_counted_after_it():
    """P1-2: lost at 101 ms; a sample received at 105 ms may have been taken at 85 ms -- before it."""
    s, clock = src()
    feed(s, 80, 0, 5)
    s.samples(100 * MS, 64)
    clock.ns = 101 * MS
    s.lost("unplug")
    s.push(105 * MS, "a", 0, "hardware_clock", G)  # a new device clock
    s.push(121 * MS, "a", 16_000, "hardware_clock", G)
    out = s.samples(200 * MS, 64)
    assert [hi // MS for _, _, hi, _ in out] == [121] and s.counts["before_boundary"] == 1


def test_the_callback_reads_its_time_under_the_queue_lock():
    """P1-2: the receipt time is taken where the sample is queued -- a time read before a boundary is never placed
    after it."""
    s, clock = src()
    seen = []

    def now():
        seen.append(s._lock.locked())  # noqa: SLF001
        return 50 * MS

    s._now = now  # noqa: SLF001
    s.on_sample("a", 0, 0.0, "hardware_clock", G)
    assert seen == [True]


def test_a_delay_building_up_over_several_samples_breaks_the_bound():
    """P1-3: each step 5 ms on the device, 15 ms on the host -- no neighbouring pair is over 21 ms, but the third
    sample is 30 ms later than the first allows."""
    s, _ = src()
    for k in range(5):
        s.push((1000 + 15 * k) * MS, "a", 5_000 * k, "hardware_clock", G)
    out = s.samples(2000 * MS, 64)
    assert len(out) == 3 and s.bound_broken  # 0, 10, 20 ms behind: the 30 ms one breaks it


def test_a_delay_behind_a_receive_gap_is_still_caught():
    """P1-3: the first sample after a 125 ms receipt gap is 5 ms on in device time: 120 ms of delay."""
    s, _ = src()
    s.push(1000 * MS, "a", 0, "hardware_clock", G)
    s.push(1125 * MS, "a", 5_000, "hardware_clock", G)
    s.samples(2000 * MS, 64)
    assert s.bound_broken and s.counts["accepted"] == 1


def test_a_device_clock_timestamp_jumping_a_whole_turn_is_no_step():
    """P1-4: get_timestamp() 1000 ms -> 4295972.296 ms (a 32-bit turn on) in 5 ms of host time breaks the bound;
    a turn back is a restart; a counter folded at 32 bits steps across."""
    s, _ = src()
    s.push(100 * MS, "a", None, "hardware_clock", G, ts_ms=1000.0)
    s.push(105 * MS, "a", None, "hardware_clock", G, ts_ms=1000.0 + 4294967.296)
    s.samples(200 * MS, 64)
    assert s.bound_broken
    s, _ = src()
    s.push(100 * MS, "a", None, "hardware_clock", G, ts_ms=4294967.296 + 1000.0)
    s.push(105 * MS, "a", None, "hardware_clock", G, ts_ms=1005.0)
    s.samples(200 * MS, 64)
    assert s.generation_why == "device_restart" and not s.bound_broken
    s, _ = src()
    s.push(100 * MS, "a", None, "hardware_clock", G, ts_ms=4294964.796)
    s.push(105 * MS, "a", None, "hardware_clock", G, ts_ms=2.5)
    assert len(s.samples(200 * MS, 64)) == 2 and s.gen == 1


def test_a_loss_reported_many_times_is_one_boundary_and_a_read_takes_at_most_limit_items():
    s, clock = src()
    for _ in range(2048):
        s.lost("unplug")
    assert s.counts["lost"] == 2048 and s.generation() == 2
    s, clock = src()
    feed(s, 100, 0, 3)
    clock.ns = 116 * MS
    s.lost("unplug")
    feed(s, 140, 500_000, 3)
    s.samples(200 * MS, 2)
    assert s.last_read_capped and s.gen == 1


# --- Codex 7b round 3 ------------------------------------------------------------------------------------------------
def _gap_stream(s, clock, *, ask_midway):
    feed(s, 100, 0, 1)
    s.samples(100 * MS, 64)
    if ask_midway:
        clock.ns = 201 * MS
        s.generation()
    feed(s, 205, 105_000, 10)
    return [hi // MS for _, _, hi, _ in s.samples(400 * MS, 64)]


def test_a_receive_gap_has_the_same_boundary_whoever_sees_it():
    """P1-1: last receipt 100 ms, the gap is one at 200 ms -- asked on a quiet tick at 201 ms or seen by the next
    sample at 205 ms, the new generation counts from lo >= 200 ms."""
    a = _gap_stream(*src(), ask_midway=True)
    b = _gap_stream(*src(), ask_midway=False)
    assert a == b == [220, 225, 230, 235, 240, 245, 250]


def test_an_old_device_sample_after_a_loss_does_not_anchor_the_new_clock():
    """P2: lost at 101 ms, an old-device sample at 105 ms (dev 32 ms), then the restarted device (dev 40 ms)."""
    s, clock = src()
    feed(s, 80, 12_000, 5)
    s.samples(100 * MS, 64)
    clock.ns = 101 * MS
    s.lost("unplug")
    s.push(105 * MS, "a", 32_000, "hardware_clock", G)
    feed(s, 145, 40_000, 4)
    out = s.samples(300 * MS, 64)
    assert not s.bound_broken and [hi // MS for _, _, hi, _ in out] == [145, 150, 155, 160]


def test_a_late_sample_after_a_fold_is_a_small_step_back():
    """P2: folded ts_ms 4294964.796 -> 2.5 -> 4294964.796 (5 ms late) -> 7.5."""
    s, _ = src()
    for k, ts in enumerate((4294964.796, 2.5, 4294964.796, 7.5)):
        s.push((100 + 5 * k) * MS, "a", None, "hardware_clock", G, ts_ms=ts)
    out = s.samples(200 * MS, 64)
    assert len(out) == 3 and s.counts["late"] == 1 and not s.bound_broken


def test_the_bound_verdict_does_not_depend_on_the_hosts_uptime():
    """P2: 5 ms of device time apart, 21 ms + 500 ns allowed -- 500 ns more is fine, 501 ns breaks, from 0 or 2**53."""
    for base in (0, 1 << 53):
        for extra, broken in ((21_000_500, False), (21_000_501, True)):
            s, _ = src()
            s.push(base + 100 * MS, "a", 0, "hardware_clock", G)
            s.push(base + 105 * MS + extra, "a", 5_000, "hardware_clock", G)
            s.samples(base + 500 * MS, 64)
            assert s.bound_broken is broken, (base, extra)


def test_boundaries_next_to_each_other_are_one_so_the_callback_work_stays_bounded():
    """P2: queue_max 1, push and lost alternating 512 times -- at most one boundary next to the one sample."""
    s, clock = src(replace(MEASURED, queue_max=1))
    for k in range(512):
        clock.ns = (100 + k) * MS
        s.push((100 + k) * MS, "a", 1000 * k, "hardware_clock", G)
        s.lost("unplug")
    assert s._nm <= 2 and s._nq == 1  # noqa: SLF001


def test_a_loss_reported_again_before_any_sample_is_the_same_loss():
    """P3: lost(); generation() three times with no sample in between -- one new generation."""
    s, _ = src()
    for _ in range(3):
        s.lost("unplug")
        s.generation()
    assert s.gen == 2 and s.counts["lost"] == 3
