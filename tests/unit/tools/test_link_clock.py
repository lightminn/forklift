"""D5 3.3b: with a clock the link gives the host the time a read returned and takes each frame's write time before
its first byte is written; without one (every other tool) both stay the cycle time."""

import os
import sys
import tty
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import real_mcu_link as rml  # noqa: E402
from forklift_core.mcu_link.messages import McuState, Mode, RejectReason, FaultFlag, StateFlag, encode  # noqa: E402
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost  # noqa: E402

CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=8)


def _state(t_us):
    return encode(McuState(t_us, 0xB0, 0, 0, Mode.DISARMED, RejectReason.NONE, FaultFlag(0), 0, 0, 0.0, 0.0, 0.0, 0.0,
                           (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0, StateFlag.DRIVERS_READY), 1)


class Clock:
    def __init__(self, *times):
        self.times = list(times)
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.times.pop(0) if len(self.times) > 1 else self.times[0]


def _pair():
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    return master, slave


def _run(clock):
    master, slave = _pair()
    log = []
    host = McuLinkHost(CFG, log=lambda what, now, data, info: log.append((what, now)))
    def opener(path, baud):
        fd = os.dup(slave)
        os.set_blocking(fd, False)  # as open_raw leaves it
        return fd

    link = rml.ReconnectingLink(os.ttyname(slave), opener=opener, clock=clock)
    try:
        os.write(master, _state(1_000_000))
        link.service(host, 1_000)  # connect + delimiter
        os.write(master, _state(1_010_000))
        host.request_disarm(1_000)
        link.service(host, 2_000)
        return host, log
    finally:
        if link.link is not None:
            link.link.close()
        os.close(master)
        os.close(slave)


def test_with_a_clock_the_read_and_write_times_are_the_clocks():
    host, log = _run(Clock(5_000, 6_000, 7_000, 8_000, 9_000))
    assert host.state_rx_ns >= 5_000 and host.state_rx_ns != 2_000  # the read time, not the cycle time
    written = [now for what, now in log if what == "written"]
    assert written and all(w not in (1_000, 2_000) for w in written)  # taken from the clock before the write


def test_without_a_clock_both_stay_the_cycle_time():
    host, log = _run(None)
    assert host.state_rx_ns in (1_000, 2_000)
    assert all(now in (1_000, 2_000) for what, now in log if what == "written")
