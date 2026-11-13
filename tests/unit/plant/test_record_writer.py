"""The teleop record's writer (record design 3): rows in order through a bounded queue and one thread, a full queue
drops and counts instead of blocking, write and close failures are counted, the footer carries the counts."""

from __future__ import annotations

import io
import json
import threading

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag
from forklift_core.plant.bringup import Sample
from forklift_core.plant.record_writer import InlineRecord, RecordWriter


def state(t):
    return McuState(mcu_time_us=t, boot_id=1, arm_gen=0, last_rx_seq=0, mode=Mode.DISARMED,
                    reject_reason=RejectReason.NONE, fault_flags=FaultFlag(0), wheel_count_left=t,
                    wheel_count_right=0, wheel_rate_left_rad_s=0.0, wheel_rate_right_rad_s=0.0, steering_rad=0.0,
                    lift_m=0.0, current_ma=(0, 0, 0, 0), duty_e4=(0, 0, 0, 0), driver_status=(0, 0, 0, 0),
                    rx_errors=0, cycle_overruns=0, flags=StateFlag(0))


class Sink(io.StringIO):
    def __init__(self, gate=None, fail_at=None, fail_close=False):
        super().__init__()
        self.gate, self.fail_at, self.fail_close, self.n = gate, fail_at, fail_close, 0

    def write(self, text):
        if self.gate is not None:
            self.gate.wait()
        self.n += 1
        if self.fail_at is not None and self.n == self.fail_at:
            raise OSError("disk")
        return super().write(text)

    def close(self):
        self.final = self.getvalue()
        if self.fail_close:
            raise OSError("close")
        super().close()


def test_rows_and_states_come_out_in_order_with_a_footer():
    sink = Sink()
    w = RecordWriter(sink)
    w.put_row({"t": "header"})
    w.put_state(Sample(10, state(1)), 12)
    w.put_row({"phase": "idle"})
    w.put_state(Sample(20, state(2)), None)
    w.close({"overruns": 0})
    rows = [json.loads(x) for x in sink.final.splitlines()]
    assert [r.get("t") for r in rows] == ["header", "state", None, "state", "footer"]
    assert rows[1]["read_ns"] == 12 and rows[1]["host_ns"] == 10 and "read_ns" not in rows[3]
    assert rows[-1] == {"t": "footer", "overruns": 0, "dropped": 0, "write_errors": 0}


def test_a_full_queue_drops_and_counts_instead_of_blocking():
    gate = threading.Event()
    sink = Sink(gate=gate)
    w = RecordWriter(sink, max_rows=5)
    for k in range(50):  # the writer is held at its first write: the queue fills
        w.put_row({"k": k})
    assert w.dropped >= 50 - 5 - 1
    gate.set()
    w.close({})
    assert json.loads(sink.final.splitlines()[-1])["dropped"] == w.dropped


def test_write_and_close_failures_are_counted_and_the_rest_goes_on():
    sink = Sink(fail_at=2, fail_close=True)
    w = RecordWriter(sink)
    for k in range(4):
        w.put_row({"k": k})
    w.close({})
    assert w.write_errors == 2  # the second row, and the close
    assert [json.loads(x).get("k") for x in sink.final.splitlines()][:3] == [0, 2, 3]


def test_a_writer_that_does_not_finish_counts_the_rest_as_dropped():
    gate = threading.Event()
    sink = Sink(gate=gate)
    w = RecordWriter(sink, max_rows=100)
    for k in range(10):
        w.put_row({"k": k})
    w.close({}, timeout_s=0.1)
    assert w.dropped >= 9
    gate.set()


def test_the_inline_record_writes_at_once():
    sink = io.StringIO()
    r = InlineRecord(sink)
    r.put_state(Sample(5, state(3)), 6)
    r.close({"x": 1})
    rows = [json.loads(x) for x in sink.getvalue().splitlines()]
    assert rows[0]["t"] == "state" and rows[0]["read_ns"] == 6 and rows[1]["t"] == "footer"
