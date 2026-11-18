"""mcu-link/2 build bits read by the host (contract section 3.3, 'build bits'): who may connect to a board and
whether it may drive. Pure functions of STATE.flags; the executors and the bring-up tools call them."""

from __future__ import annotations

import re
from enum import Enum

from .messages import StateFlag


class Client(str, Enum):
    REAL = "real"  # the real truck's executor (teleop and autonomy)
    HIL = "hil"  # the HIL executor (P0-9): a core board with a model, outputs locked
    BRINGUP = "bringup"  # STATE logging, commissioning and safety-scenario tools -- not an executor


def connect_refusal(flags: StateFlag, client: Client) -> str | None:
    """Why ``client`` must not connect to a board reporting ``flags`` (None: it may). Both ways apart: the real
    executor never takes a commissioning or HIL board, the HIL executor only a HIL board."""
    flags = StateFlag(flags)
    if client is Client.BRINGUP:
        return None  # it records and shows the bits; the commissioning tool checks commission_build itself
    if flags & StateFlag.COMMISSION_BUILD:
        return "commission_build"
    if client is Client.REAL and flags & StateFlag.HIL_PLANT:
        return "hil_board"
    if client is Client.HIL and not flags & StateFlag.HIL_PLANT:
        return "not_hil_board"
    return None


_WAIVER_TOKEN = re.compile(r"waiver=[0-9a-f]{8}")


def estop_waived(product: str | None) -> bool:
    """Whether the board's USB product string names a normal image built with the user's e-stop waiver record
    (docs/plans/2026-11-18-v1-estop-waiver.md 3; firmware identity ``forklift-mcu normal OPEN <describe> rec=<8 hex>
    waiver=<8 hex>``): the first three tokens exactly, and exactly one whole ``waiver=`` token of eight lower-case hex
    digits (a string cut at 126 characters leaves a partial one, which does not count). None (unread, UART7, a pty)
    is not waived. The record itself is checked by the firmware build alone, whose ARM gate stays."""
    if not product:
        return False
    tokens = product.split()
    return tokens[:3] == ["forklift-mcu", "normal", "OPEN"] and \
        sum(1 for t in tokens if _WAIVER_TOKEN.fullmatch(t)) == 1


def drive_refusal(flags: StateFlag, client: Client, *, estop_waived: bool = False) -> str | None:
    """Why teleop and autonomy must stay closed on a connected board (None: they may open). The HIL executor on a
    HIL board ignores outputs_locked and estop_unwired (no PWM, DRVOFF held high: the wheels cannot turn); the real
    executor refuses either -- estop_unwired only while the link's board does not name a waiver image
    (``estop_waived``), when it is a warning the host has already shown."""
    flags = StateFlag(flags)
    refused = connect_refusal(flags, client)
    if refused is not None:
        return refused
    if client is Client.BRINGUP:
        return "not_an_executor"
    if client is Client.HIL:
        return None
    if flags & StateFlag.OUTPUTS_LOCKED:
        return "outputs_locked"
    if flags & StateFlag.ESTOP_UNWIRED and not estop_waived:
        return "estop_unwired"
    return None


def commission_allowed(flags: StateFlag) -> bool:
    """The commissioning tool sends COMMISSION only to a commissioning build."""
    return bool(StateFlag(flags) & StateFlag.COMMISSION_BUILD)
