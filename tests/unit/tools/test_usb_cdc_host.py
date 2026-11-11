"""The host's side of the USB CDC link (firmware board design 5, 'host side premises'; lead assignment after #458):
DTR is never lowered after the open, a 0x00 delimiter goes out before the first frame of every opening, ModemManager
is kept off the port by a udev rule, and the operator is told to cycle the main power when STATEs stop (no
automatic USB reset)."""

import os
import re
import sys
import termios
import tty
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import real_mcu_link as rml  # noqa: E402

from forklift_core.plant.mcu_host import HostConfig, McuLinkHost  # noqa: E402

CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=8)


def test_the_open_never_lowers_dtr_and_raises_it(monkeypatch):
    """cdc_acm raises DTR on open; the firmware drops every STATE while DTR is low. The adapter never sets the B0 speed
    (which hangs up) or clears DTR, and asks for DTR high in case it was left low."""
    calls = []
    monkeypatch.setattr(rml.os, "open", lambda *a, **k: 99)
    monkeypatch.setattr(rml.termios, "tcgetattr", lambda fd: [0, 0, termios.CS8, 0, termios.B115200,
                                                             termios.B115200, [0] * 32])
    monkeypatch.setattr(rml.termios, "tcsetattr", lambda fd, when, attrs: calls.append(("tcsetattr", list(attrs))))
    monkeypatch.setattr(rml.fcntl, "ioctl", lambda fd, req, arg=0: calls.append(("ioctl", req, arg)) or arg)
    for baud in (None, 460800):
        calls.clear()
        assert rml.open_raw("/dev/ttyACM9", baud) == 99
        attrs = next(c[1] for c in calls if c[0] == "tcsetattr")
        assert attrs[4] != termios.B0 and attrs[5] != termios.B0
        assert not any(c[0] == "ioctl" and c[1] == termios.TIOCMBIC for c in calls)
        sets = [c for c in calls if c[0] == "ioctl" and c[1] == termios.TIOCMBIS]
        assert sets and int.from_bytes(sets[0][2], sys.byteorder) & termios.TIOCM_DTR


def _pty():
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    os.set_blocking(slave, False)
    return master, slave


def test_a_delimiter_goes_before_the_first_frame_of_an_opening():
    master, slave = _pty()
    try:
        link = rml.TtyLink(master)
        host = McuLinkHost(CFG)
        host.request_disarm(1)
        link.pump_writes(host, 2)
        out = os.read(slave, 256)
        assert out[0] == 0x00 and len(out) > 1  # the delimiter, then the DISARM frame
        host.request_disarm(3)
        link.pump_writes(host, 4)
        assert os.read(slave, 256)[0] != 0x00  # no second delimiter in this opening (a COBS frame never starts with 0)
    finally:
        os.close(master)
        os.close(slave)


def test_every_reopening_starts_with_a_delimiter():
    master, slave = _pty()
    try:
        opened = []

        def opener(path, baud):
            fd = os.dup(master)
            opened.append(fd)
            return fd

        link = rml.ReconnectingLink("/dev/fake", opener=opener)
        host = McuLinkHost(CFG)
        for k in range(2):
            link.receive(host, 10 + k)
            host.request_disarm(20 + k)
            link.transmit(host, 30 + k)
            assert os.read(slave, 256)[0] == 0x00
            link.link.close()
            link.link = None  # as after a lost device
    finally:
        os.close(master)
        os.close(slave)


def test_a_delimiter_that_cannot_be_written_yet_holds_the_frame_back():
    """EAGAIN on the delimiter: the frame is not handed to the adapter, so a DISARM or an expiry can still drop it."""
    link = rml.TtyLink(123)
    host = McuLinkHost(CFG)
    host.request_disarm(1)
    writes = []

    def full(fd, data):
        writes.append(bytes(data))
        raise BlockingIOError

    orig = rml.os.write
    rml.os.write = full
    try:
        link.pump_writes(host, 2)
    finally:
        rml.os.write = orig
    assert writes == [b"\x00"] and link._cur is None and [o.kind for o in host._outbox] == ["DISARM"]


# --- the udev rule (deploy/udev) -----------------------------------------------------------------------------------

RULES = REPO / "deploy" / "udev" / "99-forklift-mcu.rules"


def test_the_udev_rule_matches_the_firmwares_usb_identity_and_keeps_modemmanager_off():
    text = RULES.read_text(encoding="utf-8")
    desc = (REPO / "firmware" / "boards" / "weact_h743" / "src" / "usb_descriptors.c").read_text(encoding="utf-8")
    vid = re.search(r"#define USB_VID 0x([0-9A-Fa-f]{4})u", desc).group(1).lower()
    pid = re.search(r"#define USB_PID 0x([0-9A-Fa-f]{4})u", desc).group(1).lower()
    rules = [ln for ln in text.splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
    assert rules
    for ln in rules:
        assert f'ATTRS{{idVendor}}=="{vid}"' in ln and f'ATTRS{{idProduct}}=="{pid}"' in ln
        assert 'ATTRS{product}=="forklift-mcu *"' in ln  # the build identity's prefix: never the ROM DFU or a NUCLEO
        assert 'ENV{ID_MM_DEVICE_IGNORE}="1"' in ln
    assert any('SUBSYSTEM=="tty"' in ln and 'SYMLINK+="forklift-mcu"' in ln for ln in rules)


def test_the_install_steps_are_written_for_the_user_to_run():
    readme = (REPO / "deploy" / "udev" / "README.md").read_text(encoding="utf-8")
    assert "sudo install -m 644 deploy/udev/99-forklift-mcu.rules /etc/udev/rules.d/" in readme
    assert "udevadm control --reload" in readme and "udevadm trigger" in readme
    assert "ID_MM_DEVICE_IGNORE" in readme and "/dev/forklift-mcu" in readme


@pytest.mark.parametrize("lowered", ["TIOCMBIC", "B0"])
def test_nothing_in_the_adapter_lowers_dtr(lowered):
    src = (REPO / "tools" / "real_mcu_link.py").read_text(encoding="utf-8")
    code = [ln.split("#")[0] for ln in src.splitlines()]
    assert not any(re.search(rf"\b{lowered}\b", ln) for ln in code)


class _Silent:
    """A device that never answers (a stalled USB stack seen from the host)."""

    tx_enabled = True

    def receive(self, host, now_ns):
        return host.tick(now_ns)

    def transmit(self, host, now_ns):
        while (f := host.next_frame(now_ns)) is not None:
            host.frame_written(f, now_ns)
        return []


def test_the_console_is_told_to_cycle_the_power_when_states_stop():
    import teleop_bridge as tb

    b = tb.Bridge(_Silent(), confirmed=set())
    t0 = 1_000_000_000_000
    seen = []
    for k in range(0, 160):  # 3.2 s of cycles
        now = t0 + k * 20_000_000
        b.cycle(now)
        for _conn, msg in b.channels.take_out():
            if msg.get("t") == "state":
                seen.append((now - t0, msg.get("notes")))
    early = [n for t, n in seen if t < 1.9e9]
    late = [n for t, n in seen if t >= 2.0e9]
    assert all(not n for n in early) and late and all(n == [tb.NO_STATE_HINT] for n in late)
    assert "주 전원" in tb.NO_STATE_HINT
