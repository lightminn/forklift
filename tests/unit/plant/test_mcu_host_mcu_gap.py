"""D5: link freshness by MCU-time gap (docs/design/2026-11-15-d5-link-gap-mcu-time.md 3.3a-3.11). A host stall with
STATEs queued is not a disconnect; a host stall over the 150 ms cap, an MCU silence or a delayed newest STATE is.
The "host" rule (every other tool) is unchanged -- tests/unit/plant/test_mcu_host.py."""

import pytest

from forklift_core.mcu_link.messages import Drive, LinkDecoder, Mode, StateFlag
from forklift_core.plant.mcu_host import Blocked, HostConfig, McuLinkHost
from test_mcu_host import BOOT, MS, drive_cmd, kinds, state

ARMED_READY = dict(mode=Mode.ARMED, gen=1, flags=StateFlag.DRIVERS_READY | StateFlag.ZERO_SEEN)
MCU_CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.5, zero_valid_ms=100, outbox_max=8,
                     gap_clock="mcu", host_gap_max_s=0.15)


class FakeFreshness:
    """The bridge's side (DelayBound + rate guard): the delay bound of a STATE and the rate verdict."""

    def __init__(self):
        self.delay_ms = 5.0  # the bound every STATE gets, or a callable (state, rx_ns) -> ms | None
        self.verified = True
        self.rho = 0.001
        self.disconnects = 0

    def delay_s(self, st, rx_ns):
        d = self.delay_ms(st, rx_ns) if callable(self.delay_ms) else self.delay_ms
        return None if d is None else d / 1000.0

    def rate_verified(self):
        return self.verified

    def rate_bound(self):
        return self.rho

    def on_disconnect(self):
        self.disconnects += 1


class McuRig:
    def __init__(self, cfg=MCU_CFG):
        self.fresh = FakeFreshness()
        self.host = McuLinkHost(cfg, freshness=self.fresh)
        self.t_us = 1_000_000
        self.now = 10_000 * MS

    def feed(self, dt_ms=10, **kw):
        self.t_us += int(dt_ms * 1000)
        self.now += int(dt_ms * MS)
        return self.host.on_bytes(state(self.t_us, **kw), self.now)

    def burst(self, stall_ms, period_ms=10, mcu_first_ms=None, **kw):
        """The host stalled ``stall_ms``; the MCU kept sending every ``period_ms`` -- all read at once."""
        n = max(1, int(stall_ms // period_ms))
        first = period_ms if mcu_first_ms is None else mcu_first_ms
        data = b""
        for i in range(n):
            self.t_us += int((first if i == 0 else period_ms) * 1000)
            data += state(self.t_us, **kw)
        self.now += int(stall_ms * MS)
        return self.host.on_bytes(data, self.now)

    def tick(self, dt_ms):
        self.now += int(dt_ms * MS)
        return self.host.tick(self.now)

    def sent(self):
        dec, out = LinkDecoder(), []
        while (f := self.host.next_frame(self.now)) is not None:
            self.host.frame_written(f, self.now)
            out += [r.message for r in dec.feed(f.data, self.now)]
        return out

    def ready(self):
        self.feed()
        assert self.host.request_arm(self.now) is None
        self.sent()
        self.feed(mode=Mode.ARMED, gen=1)
        self.sent()
        self.feed(**ARMED_READY)
        assert self.host.ready


def stale_why(events):
    return [e.detail.get("why") for e in events if e.kind == "stale"]


@pytest.mark.parametrize("stall_ms", [30, 50, 70, 80])
def test_a_host_stall_with_states_queued_is_not_stale(stall_ms):
    r = McuRig()
    r.ready()
    ev = r.burst(stall_ms, **ARMED_READY)
    assert "stale" not in kinds(ev) and r.host.ready and r.host.health(r.now).fresh
    assert r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now) is None


@pytest.mark.parametrize("stall_ms", [151, 166, 327.9])
def test_a_host_stall_over_the_cap_is_stale(stall_ms):
    r = McuRig()
    r.ready()
    ev = r.burst(stall_ms, **ARMED_READY)
    assert stale_why(ev) == ["host_gap"] and "authorization_revoked" in kinds(ev)
    assert r.host.drive(drive_cmd(), 0.0, True, r.now + 100 * MS, r.now) == Blocked("not_authorized")


def test_a_host_stall_at_exactly_the_cap_is_not_stale():
    r = McuRig()
    r.ready()
    assert "stale" not in kinds(r.burst(150, **ARMED_READY)) and r.host.ready


def test_an_mcu_silence_of_60ms_is_stale_on_the_host_tick():
    r = McuRig()
    r.ready()
    ev = r.tick(20) + r.tick(20)
    assert "stale" not in kinds(ev)
    ev = r.tick(20)  # 60 ms since the receipt, +5 ms bound: 65 > 50
    assert stale_why(ev) == ["age"] and "authorization_revoked" in kinds(ev)


def test_an_mcu_silence_of_60ms_is_stale_on_receipt_after_a_host_stall():
    r = McuRig()
    r.ready()
    ev = r.burst(60, mcu_first_ms=60, **ARMED_READY)  # one STATE, 60 ms of MCU time after the last
    assert stale_why(ev) == ["mcu_gap"] and not r.host.ready


@pytest.mark.parametrize("gap_ms,stale", [(49.9, False), (49.95, False), (49.96, True), (50.0, True)])
def test_the_mcu_gap_carries_the_rate_margin(gap_ms, stale):
    r = McuRig()
    r.ready()
    ev = r.burst(gap_ms, mcu_first_ms=gap_ms, **ARMED_READY)  # x (1 + 0.001)
    assert ("mcu_gap" in stale_why(ev)) is stale


def test_a_delayed_burst_with_old_mcu_times_is_stale():
    r = McuRig()
    r.ready()
    r.fresh.delay_ms = 60.0  # the path itself held the bytes: the newest STATE is 60 ms old
    ev = r.burst(60, **ARMED_READY)
    assert stale_why(ev) == ["age"] and not r.host.ready


def test_a_delayed_burst_under_the_bound_is_fresh():
    r = McuRig()
    r.ready()
    r.fresh.delay_ms = 40.0
    ev = r.burst(60, **ARMED_READY)
    assert "stale" not in kinds(ev) and r.host.ready
    assert r.host.health(r.now).delay_s == pytest.approx(0.040)


def test_a_persistent_delay_is_stale_even_without_a_gap():
    r = McuRig()
    r.ready()
    r.fresh.delay_ms = 60.0
    assert stale_why(r.feed(**ARMED_READY)) == ["age"]


def test_older_states_in_a_batch_do_not_trip_the_age_check():
    r = McuRig()
    r.ready()
    newest = r.t_us + 70_000
    r.fresh.delay_ms = lambda st, rx: 5.0 if st.mcu_time_us == newest else 70.0
    assert "stale" not in kinds(r.burst(70, **ARMED_READY)) and r.host.ready


def test_without_a_delay_bound_the_host_rule_applies():
    r = McuRig()
    r.ready()
    r.fresh.delay_ms = None
    assert stale_why(r.burst(60, **ARMED_READY)) == ["host_gap"]


def test_before_the_rate_is_verified_the_host_rule_applies():
    r = McuRig()
    r.ready()
    r.fresh.verified = False
    ev = r.burst(60, **ARMED_READY)
    assert "stale" in kinds(ev) and not r.host.ready
    assert stale_why(ev) == [None]  # the host rule's event, unchanged


def test_a_boot_change_after_a_stall_uses_the_host_rule_and_revokes():
    r = McuRig()
    r.ready()
    r.t_us = 2_000  # a new boot's clock
    ev = r.burst(60, boot=BOOT + 1, mode=Mode.DISARMED)
    assert "boot_changed" in kinds(ev) and stale_why(ev) == ["host_gap"] and not r.host.authorized


def test_a_disconnect_resets_the_bridges_delay_bound():
    r = McuRig()
    r.ready()
    r.host.disconnected(r.now)
    assert r.fresh.disconnects == 1 and r.host.health(r.now).delay_s is None
    ev = r.feed(**ARMED_READY)  # an ARMED STATE after the reconnect never restores the authorization
    assert "first_state" in kinds(ev) and not r.host.authorized


def test_a_stale_gap_inside_a_batch_is_not_reauthorized_by_later_states():
    r = McuRig()
    r.ready()
    ev = r.burst(300, **ARMED_READY)  # thirty STATEs read together after a 300 ms stall
    assert stale_why(ev) == ["host_gap"] and not r.host.authorized and not r.host.ready
    ev = r.feed(**ARMED_READY)
    assert not r.host.authorized  # only a new operator ARM


def test_forward_transitions_use_the_newest_state_of_a_batch():
    r = McuRig()
    r.feed(steering=0.01)
    assert r.host.request_arm(r.now) is None
    r.sent()
    data = b""
    for i in range(8):
        r.t_us += 10_000
        data += state(r.t_us, mode=Mode.ARMED, gen=1, steering=0.01 * (i + 1))
    r.now += 80 * MS
    ev = r.host.on_bytes(data, r.now)
    assert kinds(ev).count("zero_sent") == 1
    (zero,) = r.sent()
    assert isinstance(zero, Drive) and zero.echo_mcu_time_us == r.t_us and zero.steering_rad == pytest.approx(0.08)


def test_a_host_clock_regression_is_rejected():
    r = McuRig()
    r.ready()
    rx = r.host.state_rx_ns
    ev = r.host.on_bytes(state(r.t_us + 10_000, **ARMED_READY), rx - 1)
    assert "state_invalid" in kinds(ev) and r.host.state_rx_ns == rx


def test_the_mcu_gap_clock_needs_its_freshness_source_and_a_sane_cap():
    with pytest.raises(ValueError):
        McuLinkHost(MCU_CFG)
    for cap in (0.04, 0.2):
        with pytest.raises(ValueError):
            HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.5, zero_valid_ms=100, outbox_max=8,
                       gap_clock="mcu", host_gap_max_s=cap)
    with pytest.raises(ValueError):
        HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.5, zero_valid_ms=100, outbox_max=8,
                   gap_clock="wall")
    assert HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.5, zero_valid_ms=100, outbox_max=8,
                      gap_clock="mcu").host_gap_s == pytest.approx(0.15)


def test_drive_validity_still_counts_from_the_receipt_after_a_stall():
    r = McuRig()
    r.ready()
    r.burst(70, **ARMED_READY)
    rx, echo = r.now, r.t_us
    assert r.host.drive(drive_cmd(), 0.0, True, rx + 100 * MS, r.now) is None
    (d,) = r.sent()
    assert d.echo_mcu_time_us == echo and d.valid_after_echo_ms == 100


def test_a_stale_event_names_the_other_clocks_gap():
    # a record must tell a host stall (the operating path) from an MCU silence (lead 2026-11-16)
    r = McuRig()
    r.ready()
    (d,) = [e.detail for e in r.burst(166, **ARMED_READY) if e.kind == "stale"]
    assert d["why"] == "host_gap" and d["value_s"] == pytest.approx(0.166) and d["mcu_gap_s"] == pytest.approx(0.010)
    r = McuRig()
    r.ready()
    (d,) = [e.detail for e in r.burst(60, mcu_first_ms=60, **ARMED_READY) if e.kind == "stale"]
    assert d["why"] == "mcu_gap" and d["mcu_gap_s"] == pytest.approx(0.060) and d["host_gap_s"] == pytest.approx(0.060)
    r = McuRig()
    r.ready()
    (d,) = [e.detail for e in r.tick(20) + r.tick(20) + r.tick(20) if e.kind == "stale"]
    assert d["why"] == "age" and d["host_gap_s"] == pytest.approx(0.060) and d["delay_s"] == pytest.approx(0.005)


def test_a_delayed_newest_state_names_its_delay_and_the_gaps():
    r = McuRig()
    r.ready()
    r.fresh.delay_ms = 60.0
    (d,) = [e.detail for e in r.burst(60, **ARMED_READY) if e.kind == "stale"]
    assert d["why"] == "age" and d["delay_s"] == pytest.approx(0.060) and d["n_states"] == 6
    assert d["host_gap_s"] == pytest.approx(0.060) and d["mcu_gap_s"] == pytest.approx(0.060)
