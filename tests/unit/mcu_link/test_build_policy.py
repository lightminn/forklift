"""mcu-link/2 build bits on the host side, both ways apart (lead condition 2026-11-11)."""

import pytest

from forklift_core.mcu_link.build_policy import Client, commission_allowed, connect_refusal, drive_refusal
from forklift_core.mcu_link.messages import StateFlag as F

REAL_OK = F.DRIVERS_READY
HIL = F.HIL_PLANT | F.OUTPUTS_LOCKED | F.ESTOP_UNWIRED  # a core board alone: no e-stop pin, outputs locked
COMMISSION = F.COMMISSION_BUILD | F.ESTOP_UNWIRED


@pytest.mark.parametrize("flags,client,connect,drive", [
    (REAL_OK, Client.REAL, None, None),
    (REAL_OK | F.OUTPUTS_LOCKED, Client.REAL, None, "outputs_locked"),  # S2: sensors read, motors locked
    (REAL_OK | F.ESTOP_UNWIRED, Client.REAL, None, "estop_unwired"),
    (HIL, Client.REAL, "hil_board", "hil_board"),  # the real executor never takes a HIL board
    (COMMISSION, Client.REAL, "commission_build", "commission_build"),
    (HIL, Client.HIL, None, None),  # HIL opens teleop and autonomy whatever bits 6-7 say
    (REAL_OK, Client.HIL, "not_hil_board", "not_hil_board"),  # the HIL executor never takes a real board
    (REAL_OK | F.OUTPUTS_LOCKED, Client.HIL, "not_hil_board", "not_hil_board"),
    (COMMISSION | F.HIL_PLANT, Client.HIL, "commission_build", "commission_build"),
    (COMMISSION, Client.BRINGUP, None, "not_an_executor"),
    (HIL, Client.BRINGUP, None, "not_an_executor"),
])
def test_who_connects_and_who_drives(flags, client, connect, drive):
    assert connect_refusal(flags, client) == connect
    assert drive_refusal(flags, client) == drive


def test_the_commissioning_tool_needs_the_commissioning_build():
    assert commission_allowed(COMMISSION) and not commission_allowed(REAL_OK) and not commission_allowed(HIL)


def test_the_golden_state_0103_is_a_commissioning_board():
    from forklift_core.mcu_link.framing import cobs_decode
    from forklift_core.mcu_link.messages import decode

    state_0103 = ("0a02810301443322110201010befbeadde030202010207010a4a01c01dfefff1fb09010103a03f010102bf0103803d"
                  "0108403fdc0538ff1e09a00f8813f0d8c4090104c0c4e00211020205d90275f100")  # contract 6
    body = cobs_decode(bytes.fromhex(state_0103)[:-1])[:-2]
    _, state = decode(body)
    assert connect_refusal(state.flags, Client.REAL) == "commission_build" and commission_allowed(state.flags)
