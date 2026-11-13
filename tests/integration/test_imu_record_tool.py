"""G2 recorder (docs/design/2026-11-14-imu-record-and-mount.md 2, 6) with a fake frame source instead of pyrealsense2:
row format, header, temperature rows, footer, existing file refused, the callback only queues, rate mismatch kept."""

from __future__ import annotations

import io
import json
import sys
import threading
import time
from pathlib import Path

import pytest

from forklift_core.sensors import imu_record as ir

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
import imu_record as tool  # noqa: E402


class FakeSource:
    """Calls ``on_sample`` from its own thread like librealsense does."""

    def __init__(self, n=40, *, actual=None, temps=({"asic_temperature": 41.0},), dom="hardware_clock", fail=None,
                 samples=None):
        self.n, self.dom, self.fail, self.samples = n, dom, fail, samples
        self.actual = actual or {"gyro_hz": 400, "accel_hz": 200}
        self.temps = list(temps)
        self.stopped = False
        self.thread = None

    def info(self):
        return {"device": "Intel RealSense D435I", "serial": "000", "firmware": "5.15.1.55", "usb": "3.2"}

    def profile_info(self):
        return {"actual": dict(self.actual), "imu_intrinsics": {"gyro": {"data": [[1.01, 0, 0, 0.002]] * 3}},
                "rate_fallback": None,
                "options": {"enable_motion_correction": {"supported": True, "set": 0, "read_back": 0.0}}}

    def temperature(self):
        return self.temps.pop(0) if self.temps else None

    def start(self, on_sample):
        if self.fail:
            raise RuntimeError(self.fail)
        if self.samples is not None:
            def run_given():
                for args in self.samples:
                    on_sample(*args)
            self.thread = threading.Thread(target=run_given, daemon=True)
            self.thread.start()
            return

        def run():
            for i in range(self.n):
                if self.stopped:
                    return
                s = "g" if i % 3 else "a"
                on_sample(s, ((1 << 32) - 50 + i * 2500) % (1 << 32), 1000.0 + i * 2.5, self.dom, (0.0, -9.8 if s == "a" else 0.0, 0.0))
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stopped = True
        if self.thread:
            self.thread.join(2)


def read_rows(text):
    return [json.loads(line) for line in text.splitlines() if line.strip()]


def run_record(source, *, seconds=0.3, temp_every_s=0.1, requested=(400, 200), global_time=False, stall_s=2.0):
    sink = io.StringIO()
    sink.close = lambda: None  # keep the text readable after the writer closes it
    footer = tool.record(source, sink, seconds=seconds, temp_every_s=temp_every_s, gyro_hz=requested[0],
                         accel_hz=requested[1], global_time=global_time, stall_s=stall_s)
    return read_rows(sink.getvalue()), footer


def test_rows_header_temperatures_and_footer():
    rows, footer = run_record(FakeSource())
    header, last = rows[0], rows[-1]
    assert header["t"] == "header" and header["schema"] == ir.SCHEMA and header["tool"] == "imu_record.py"
    assert header["host_clock"] == "monotonic" and header["global_time"] is False
    assert header["requested"] == {"gyro_hz": 400, "accel_hz": 200}
    assert header["serial"] == "000" and "host_boot_id" in header
    prof = next(r for r in rows if r["t"] == "profile")
    assert prof["rate_mismatch"] is False and prof["actual"] == {"gyro_hz": 400, "accel_hz": 200}
    assert prof["imu_intrinsics"]["gyro"]["data"][0][0] == 1.01  # a non-identity calibration is kept as given
    assert prof["options"]["enable_motion_correction"]["read_back"] == 0.0
    imu = [r for r in rows if r["t"] == "imu"]
    assert len(imu) == 40
    r = imu[0]
    assert set(r) == {"t", "s", "dev_us", "ts_ms", "dom", "host_ns", "x", "y", "z"}
    assert r["dom"] == "hardware_clock" and isinstance(r["host_ns"], int)
    temps = [r for r in rows if r["t"] == "temp"]
    assert temps and temps[0]["asic_temperature"] == 41.0 and isinstance(temps[0]["host_ns"], int)
    assert last["t"] == "footer" and last["samples"] == {"g": 26, "a": 14}
    assert last["dropped"] == 0 and last["write_errors"] == 0 and footer["samples"] == last["samples"]
    # the rows go through the G7 fit path unchanged
    st = ir.imu_stream(rows, stream="g")
    assert st.host_clock == "monotonic" and (st.dev_us[1:] > st.dev_us[:-1]).all()
    stats = last["streams"]["g"]
    assert stats["n"] == 26 and stats["missing_dev"] == 0 and stats["back"] == 0 and stats["dup"] == 0
    assert stats["max_gap_us"] == 5000  # every third sample went to the accel stream


def test_a_rate_the_device_did_not_give_is_recorded_not_refused():
    rows, _ = run_record(FakeSource(actual={"gyro_hz": 200, "accel_hz": 100}))
    prof = next(r for r in rows if r["t"] == "profile")
    assert prof["rate_mismatch"] is True and prof["actual"] == {"gyro_hz": 200, "accel_hz": 100}


def test_the_callback_only_queues_while_the_disk_stalls():
    gate = threading.Event()

    class SlowSink(io.StringIO):
        def write(self, s):
            gate.wait(5)
            return super().write(s)

        def close(self):
            pass

    sink = SlowSink()
    writer, on_sample, counts = tool.make_callback(sink, now_ns=time.monotonic_ns)
    t0 = time.monotonic()
    for i in range(2000):
        on_sample("g", i, 0.0, "hardware_clock", (0.0, 0.0, 0.0))
    assert time.monotonic() - t0 < 1.0  # nothing waited on the stalled sink
    assert counts["g"] == 2000
    gate.set()
    result = writer.close({"samples": dict(counts)}, timeout_s=10)
    assert result["finished"] and result["dropped"] == 0


def test_the_cli_refuses_an_existing_file_and_creates_the_folder(tmp_path):
    out = tmp_path / "imu" / "check.jsonl"
    assert tool.main(["--out", str(out), "--seconds", "0.2"], source_factory=lambda **kw: FakeSource()) == 0
    rows = read_rows(out.read_text())
    assert rows[0]["t"] == "header" and rows[-1]["t"] == "footer"
    with pytest.raises(SystemExit):
        tool.main(["--out", str(out), "--seconds", "0.2"], source_factory=lambda **kw: FakeSource())
    assert read_rows(out.read_text()) == rows  # untouched


def test_the_cli_passes_rates_and_global_time_to_the_source(tmp_path):
    seen = {}

    def factory(**kw):
        seen.update(kw)
        return FakeSource()

    tool.main(["--out", str(tmp_path / "a.jsonl"), "--seconds", "0.1", "--gyro-hz", "200", "--accel-hz", "100",
               "--global-time", "on"], source_factory=factory)
    assert seen == {"gyro_hz": 200, "accel_hz": 100, "global_time": True}
    header = read_rows((tmp_path / "a.jsonl").read_text())[0]
    assert header["global_time"] is True and header["requested"] == {"gyro_hz": 200, "accel_hz": 100}


def test_two_streams_calling_back_at_once_lose_nothing():
    sink = io.StringIO()
    sink.close = lambda: None
    writer, on_sample, counts = tool.make_callback(sink)

    def burst(s):
        for i in range(3000):
            on_sample(s, i, 0.0, "hardware_clock", (0.0, 0.0, 0.0))

    threads = [threading.Thread(target=burst, args=(s,)) for s in ("g", "a")]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    result = writer.close({}, timeout_s=10)
    imu = [r for r in read_rows(sink.getvalue()) if r["t"] == "imu"]
    assert counts["g"] == counts["a"] == 3000 and len(imu) == 6000 and result["dropped"] == 0


def test_a_full_queue_drops_and_counts_and_a_late_callback_is_dropped():
    gate = threading.Event()

    class SlowSink(io.StringIO):
        def write(self, s):
            gate.wait(5)
            return super().write(s)

        def close(self):
            pass

    sink = SlowSink()
    writer, on_sample, counts = tool.make_callback(sink, max_rows=10)
    for i in range(50):
        on_sample("g", i, 0.0, "hardware_clock", (0.0, 0.0, 0.0))
    gate.set()
    result = writer.close({}, timeout_s=10)
    on_sample("g", 99, 0.0, "hardware_clock", (0.0, 0.0, 0.0))  # after close: dropped, never written
    assert result["finished"] and 30 <= result["dropped"] <= 40
    assert writer.dropped == result["dropped"] + 1


def test_a_start_failure_still_ends_the_record_and_fails_the_run(tmp_path):
    out = tmp_path / "fail.jsonl"
    assert tool.main(["--out", str(out), "--seconds", "1"], source_factory=lambda **kw: FakeSource(fail="busy")) == 1
    rows = read_rows(out.read_text())
    assert rows[0]["t"] == "header" and rows[-1]["t"] == "footer"
    assert rows[-1]["stop"] == "start_failed" and "busy" in rows[-1]["error"]


def test_samples_that_stop_coming_end_the_record_as_stalled():
    rows, footer = run_record(FakeSource(n=5), seconds=10.0, stall_s=0.3)
    assert footer["stop"] == "stalled" and rows[-1]["stop"] == "stalled" and footer["seconds"] < 5


def test_ctrl_c_ends_with_the_footer(monkeypatch):
    calls = {"n": 0}
    real_sleep = time.sleep

    def sleep(s):
        calls["n"] += 1
        if calls["n"] == 3:
            raise KeyboardInterrupt
        real_sleep(s)

    monkeypatch.setattr(tool.time, "sleep", sleep)
    rows, footer = run_record(FakeSource(), seconds=None)
    assert footer["stop"] == "interrupt" and rows[-1]["t"] == "footer" and footer["finished"]


def test_the_footer_counts_missing_device_times_nans_repeats_and_steps_back():
    samples = [("g", 100, 0.0, "hardware_clock", (0.0, 0.0, 0.0)),
               ("g", 2600, 0.0, "hardware_clock", (0.0, 0.0, 0.0)),
               ("g", 2600, 0.0, "hardware_clock", (0.0, 0.0, 0.0)),
               ("g", 2000, 0.0, "hardware_clock", (0.0, 0.0, 0.0)),
               ("g", None, 0.0, "system_time", (0.0, 0.0, 0.0)),
               ("g", 12600, 0.0, "hardware_clock", (float("nan"), 0.0, 0.0))]
    rows, footer = run_record(FakeSource(samples=samples), seconds=0.3)
    st = footer["streams"]["g"]
    assert st == {"n": 6, "missing_dev": 1, "nan": 1, "dup": 1, "back": 1, "max_gap_us": 10000,
                  "domains": ["hardware_clock", "system_time"]}
    assert footer["streams"]["a"]["n"] == 0


def test_no_device_is_a_clean_failure_and_makes_no_file(tmp_path):
    def factory(**kw):
        raise RuntimeError("no RealSense device found")

    out = tmp_path / "none.jsonl"
    assert tool.main(["--out", str(out), "--seconds", "1"], source_factory=factory) == 1
    assert not out.exists()

