"""mcu-link/2 build bits read by the host (contract section 3.3, 'build bits'): who may connect to a board and
whether it may drive. Pure functions of STATE.flags; the executors and the bring-up tools call them."""

from __future__ import annotations

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


def drive_refusal(flags: StateFlag, client: Client) -> str | None:
    """Why teleop and autonomy must stay closed on a connected board (None: they may open). The HIL executor on a
    HIL board ignores outputs_locked and estop_unwired (no PWM, DRVOFF held high: the wheels cannot turn); the real
    executor refuses either."""
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
    if flags & StateFlag.ESTOP_UNWIRED:
        return "estop_unwired"
    return None


def commission_allowed(flags: StateFlag) -> bool:
    """The commissioning tool sends COMMISSION only to a commissioning build."""
    return bool(StateFlag(flags) & StateFlag.COMMISSION_BUILD)
