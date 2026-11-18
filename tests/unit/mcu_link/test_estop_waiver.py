"""V1.0 e-stop waiver, host side (docs/plans/2026-11-18-v1-estop-waiver.md 3): the USB product string of the opened
tty says whether the firmware was built with the user's waiver record; then bit 7 (estop_unwired) is a warning, not a
refusal, for the real executor and bring-up --floor. Unreadable, UART7 or any other image keeps the refusal."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

from forklift_core.mcu_link.build_policy import Client, drive_refusal, estop_waived
from forklift_core.mcu_link.messages import StateFlag as F

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import real_mcu_link as rml  # noqa: E402

WAIVED = "forklift-mcu normal OPEN a1287ce5d0e1 rec=0123abcd waiver=89abcdef"
WAIVED_DIRTY = "forklift-mcu normal OPEN a1287ce5d0e1-dirty rec=0123abcd waiver=89abcdef"


@pytest.mark.parametrize("product,waived", [
    (WAIVED, True),
    (WAIVED_DIRTY, True),
    ("forklift-mcu normal OPEN v0.3-12-gabcdef123456 rec=0123abcd waiver=89abcdef", True),
    ("forklift-mcu normal OPEN a1287ce5d0e1 rec=0123abcd", False),  # no waiver record in the image
    ("forklift-mcu commission OPEN a1287ce5d0e1 rec=0123abcd waiver=89abcdef", False),  # not the normal build
    ("forklift-mcu normal locked a1287ce5d0e1 waiver=89abcdef", False),
    ("forklift-mcu normal OPEN a1287ce5d0e1 rec=0123abcd waiver=89abcd", False),  # cut at 126 characters
    ("forklift-mcu normal OPEN a1287ce5d0e1 rec=0123abcd waiver=89ABCDEF", False),  # not lower-case hex
    ("forklift-mcu normal OPEN a1287ce5d0e1 waiver=89abcdef waiver=01234567", False),  # two tokens: not one image
    ("other-device normal OPEN x waiver=89abcdef", False),
    ("", False),
    (None, False),  # unreadable, a UART7 link or a pty
])
def test_the_product_string_says_waived(product, waived):
    assert estop_waived(product) is waived


def test_the_waiver_makes_bit7_a_warning_and_nothing_else():
    unwired = F.DRIVERS_READY | F.ESTOP_UNWIRED
    assert drive_refusal(unwired, Client.REAL) == "estop_unwired"
    assert drive_refusal(unwired, Client.REAL, estop_waived=True) is None
    # the waiver says nothing about locked outputs or the other builds
    assert drive_refusal(unwired | F.OUTPUTS_LOCKED, Client.REAL, estop_waived=True) == "outputs_locked"
    assert drive_refusal(F.COMMISSION_BUILD | F.ESTOP_UNWIRED, Client.REAL, estop_waived=True) == "commission_build"
    assert drive_refusal(unwired, Client.BRINGUP, estop_waived=True) == "not_an_executor"


def _host():
    from forklift_core.plant.mcu_host import HostConfig, McuLinkHost

    return McuLinkHost(HostConfig(state_max_age_s=0.1, cap_ms=150, arm_timeout_s=1.0, zero_valid_ms=100,
                                  outbox_max=16))


def test_the_host_keeps_the_identity_for_one_connection():
    h = _host()
    assert h.estop_waived is False
    events = h.link_identity(WAIVED)
    assert h.estop_waived is True and h.usb_product == WAIVED
    assert [e.kind for e in events] == ["estop_waived"]  # the warning, once per connection
    h.disconnected(1)
    assert h.estop_waived is False and h.usb_product is None
    assert h.link_identity(None) == [] and h.estop_waived is False


def _fake_sysfs(tmp_path, product: str | None, major=166, minor=0):
    """/sys/dev/char/M:m -> .../usb1/1-1/1-1:1.0/tty/ttyACM0, with the USB device's product two levels up."""
    dev = tmp_path / "sys" / "devices" / "pci0000:00" / "usb1" / "1-1"
    tty = dev / "1-1:1.0" / "tty" / "ttyACM0"
    tty.mkdir(parents=True)
    if product is not None:
        (dev / "product").write_text(product + "\n")
    char = tmp_path / "sys" / "dev" / "char"
    char.mkdir(parents=True)
    (char / f"{major}:{minor}").symlink_to(tty)
    return tmp_path / "sys", os.makedev(major, minor)


def test_the_product_is_read_from_the_usb_device_of_the_opened_tty(tmp_path):
    root, rdev = _fake_sysfs(tmp_path, WAIVED)
    assert rml.usb_product_for_rdev(rdev, sys_root=root) == WAIVED


def test_no_usb_parent_or_no_node_reads_none(tmp_path):
    root, rdev = _fake_sysfs(tmp_path, None)  # a UART (ttyS/ttyTHS): no product file above it
    assert rml.usb_product_for_rdev(rdev, sys_root=root) is None
    assert rml.usb_product_for_rdev(os.makedev(4, 64), sys_root=root) is None  # no such node


def test_a_pty_or_a_pipe_has_no_product():
    r, w = os.pipe()
    try:
        assert rml.usb_product_of_fd(r) is None
    finally:
        os.close(r)
        os.close(w)


def test_the_link_reads_the_identity_at_each_opening_and_forgets_it_on_loss():
    h = _host()
    opened = []

    def opener(path, baud):
        r, w = os.pipe()
        opened.append((r, w))
        os.close(w)  # the read end sees EOF at once: the link is lost on the first service after the opening
        return r

    products = iter([WAIVED, None])
    link = rml.ReconnectingLink("/dev/fake", opener=opener, product_reader=lambda fd: next(products), retry_s=0.0)
    events = link.service(h, 1_000)
    kinds = [e.kind for e in events]
    assert kinds[0] == "connected" and "estop_waived" in kinds
    assert "disconnected" in kinds and h.estop_waived is False  # lost in the same service: forgotten
    events = link.service(h, 2_000)
    assert [e.kind for e in events][0] == "connected" and h.estop_waived is False  # the next board says nothing
    for r, _w in opened:
        try:
            os.close(r)
        except OSError:
            pass


def _bringup():
    import bringup  # tools/ is on sys.path (above)

    return bringup


def test_bringup_floor_takes_a_waived_board_and_refuses_the_rest():
    bu = _bringup()
    unwired = F.DRIVERS_READY | F.ESTOP_UNWIRED
    assert bu.floor_refusal(unwired, allow_locked=False, estop_waived=False) is not None
    assert bu.floor_refusal(unwired, allow_locked=False, estop_waived=True) is None
    assert bu.floor_refusal(unwired | F.OUTPUTS_LOCKED, allow_locked=False, estop_waived=True) is not None
    assert bu.floor_refusal(F.DRIVERS_READY, allow_locked=True, estop_waived=True) is not None
    assert bu.floor_refusal(F.DRIVERS_READY, allow_locked=False, estop_waived=False) is None


def test_bringup_cap_takes_the_wheel_radius():
    bu = _bringup()
    from forklift_core.plant.real_vehicle import load_real_vehicle

    cfg = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
    cap = bu._cap()
    assert cap == pytest.approx(min(cfg.wheel_rate_limit_rad_s, 0.2 / max(cfg.wheel_radius_m)))
    assert cap >= 0.132 / max(cfg.wheel_radius_m)  # the 0.132 m/s floor test (1.06 rad/s) is not refused
