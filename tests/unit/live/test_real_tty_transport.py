"""The operating unlock U1 (docs/plans/2026-11-26-real-operating-assembly.md 2.1): the real link transport -- the lock
on the fd before configuration, the USB identity from that fd, the motion permission (``real_operating``) granted on a
boot and revoked by a lost device, another boot or a HIL/commissioning build -- and the device block's third way."""

import os
import sys
import tty
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.live.real_loop import motion_allowed
from forklift_core.mcu_link.messages import McuState, Mode, RejectReason, StateFlag, encode

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import real_loop_transport as rlt  # noqa: E402


class Host:
    def __init__(self):
        self.rx = []

    def on_bytes(self, data, now_ns):
        self.rx.append(len(data))

    def disconnected(self, now_ns):
        pass


def pair():
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    os.set_blocking(slave, False)
    return master, slave


def state(boot=7, flags=StateFlag(0)):
    s = McuState(1000, boot, 0, 0, Mode.DISARMED, RejectReason.NONE, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, (0,) * 4, (0,) * 4,
                 (0,) * 4, 0, 0, flags)
    return replace(s, boot_id=boot)


def feed(master, transport, s):
    os.write(master, encode(s, 0))
    transport.receive(Host(), 0)


def make(**kw):
    m, s = pair()
    return m, rlt.RealTtyTransport(s, usb_product=kw.get("product", "forklift-mcu normal open v1"),
                                   usb_serial=kw.get("serial", "0123456789ABCDEF01234567"),
                                   locked=kw.get("locked", True))


def test_permission_is_off_until_granted_and_never_a_hil_check():
    m, t = make()
    feed(m, t, state())
    assert t.real_operating is False and t.hil_plant is False
    assert motion_allowed(t) is False
    assert t.grant([], 7) is True
    assert t.real_operating is True and motion_allowed(t) is True
    assert t.hil_plant is False  # never a HIL check, whatever the board says


@pytest.mark.parametrize("flags", [StateFlag.HIL_PLANT, StateFlag.COMMISSION_BUILD])
def test_a_hil_or_commissioning_build_revokes(flags):
    m, t = make()
    t.grant([], 7)
    feed(m, t, state(flags=flags))
    assert t.real_operating is False and t.revoked == "board_build_changed"
    assert t.grant([], 7) is False  # a revocation is final for the run


def test_another_boot_revokes():
    m, t = make()
    feed(m, t, state(boot=7))
    t.grant([], 7)
    feed(m, t, state(boot=7))
    assert t.real_operating
    feed(m, t, state(boot=8))
    assert not t.real_operating and t.revoked == "boot_changed"


def test_a_lost_device_revokes_and_raises():
    m, t = make()
    t.grant([], 7)
    os.close(m)  # the far end gone: the next read sees EIO or a hangup
    with pytest.raises(rlt.DeviceLost):
        for _ in range(3):
            t.receive(Host(), 0)
    assert not t.real_operating and t.revoked.startswith("device_lost:")


def test_no_grant_without_a_boot():
    _m, t = make()
    assert t.grant([], None) is False and t.grant([], 0) is False


def test_the_device_block_takes_only_the_strict_true():
    assert motion_allowed(SimpleNamespace(real_operating=1)) is False
    assert motion_allowed(SimpleNamespace(real_operating="yes")) is False
    assert motion_allowed(SimpleNamespace()) is False
    assert motion_allowed(SimpleNamespace(real_operating=True)) is True


def test_open_takes_the_lock_before_configuring_and_reads_the_identity_from_that_fd():
    order = []
    m, s = pair()

    def opener(path, before_config=None):
        order.append(("open", path))
        before_config(s)
        order.append("configured")
        return s

    t = rlt.RealTtyTransport.open("/dev/forklift-mcu", opener=opener, lock=lambda fd: order.append(("lock", fd)),
                                  product_reader=lambda fd: f"product:{fd}", serial_reader=lambda fd: f"serial:{fd}",
                                  allow_pty=True)
    assert order == [("open", "/dev/forklift-mcu"), ("lock", s), "configured"]
    assert (t.usb_product, t.usb_serial) == (f"product:{s}", f"serial:{s}") and t.locked
    os.close(m)
    os.close(s)


def test_open_closes_the_fd_when_reading_the_identity_fails():
    m, s = pair()

    def boom(fd):
        raise RuntimeError("no sysfs")

    with pytest.raises(RuntimeError):
        rlt.RealTtyTransport.open("/dev/x", opener=lambda p, before_config=None: s, lock=lambda fd: None,
                                  product_reader=boom, serial_reader=lambda fd: None, allow_pty=True)
    with pytest.raises(OSError):
        os.fstat(s)  # closed
    os.close(m)


def test_a_busy_device_is_refused_without_configuring(tmp_path):
    from forklift_core.live.executor_lock import DeviceBusy

    m, s = pair()
    configured = []

    def opener(path, before_config=None):
        before_config(s)
        configured.append(True)
        return s

    def busy(fd):
        raise DeviceBusy("/dev/x", None)

    with pytest.raises(DeviceBusy):
        rlt.RealTtyTransport.open("/dev/x", opener=opener, lock=busy, product_reader=lambda fd: None,
                                  serial_reader=lambda fd: None)
    assert configured == []
    os.close(m)
    os.close(s)


def test_a_pty_is_refused_by_the_opened_fds_own_type():
    m, s = pair()
    with pytest.raises(OSError):
        rlt.RealTtyTransport.open("/dev/forklift-mcu", opener=lambda p, before_config=None: s, lock=lambda fd: None,
                                  product_reader=lambda fd: "p", serial_reader=lambda fd: "s")
    with pytest.raises(OSError):
        os.fstat(s)  # closed on the refusal
    os.close(m)


@pytest.mark.parametrize("why", ["unlocked", "no_state", "judgement", "other_boot", "locked_outputs_seen"])
def test_grant_needs_the_lock_a_first_state_an_empty_judgement_and_the_current_boot(why):
    m, t = make(locked=why != "unlocked")
    if why != "no_state":
        feed(m, t, state(boot=8, flags=StateFlag.OUTPUTS_LOCKED if why == "locked_outputs_seen" else StateFlag(0)))
    if why == "locked_outputs_seen":
        feed(m, t, state(boot=8))  # a good STATE after a bad one in the same session does not clear it
    judgement = ["precondition:unlock_incomplete"] if why == "judgement" else []
    boot = 7 if why == "other_boot" else 8
    assert t.grant(judgement, boot) is False and t.real_operating is False
    os.close(m)


def test_motion_bytes_are_written_only_while_the_permission_holds():
    from forklift_core.mcu_link.messages import Disarm
    from forklift_core.plant.mcu_host import Outgoing

    m, t = make()
    feed(m, t, state())
    drive = Outgoing("DRIVE", 3, b"\x05" * 20 + b"\x00", None, 0)
    disarm = Outgoing("DISARM", 4, encode(Disarm(), 4), None, 0)
    t._poison_at = None
    assert t._write_first(drive, None) == "eagain"  # not granted: not one byte
    assert t._write_first(disarm, None) == "done"  # the DISARM always goes
    assert t.grant([], 7)
    t._cur, t._cur_at = drive, 5  # a half-written DRIVE
    t.revoke("test")
    assert t._write_rest(drive, None) is False and t._cur_at == 5  # no continuation after the revocation
    os.close(m)
