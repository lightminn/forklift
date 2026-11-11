"""The bring-up tool's drive conditions (docs/design/2026-11-12-bringup-host-tools.md 3) without a link."""

import sys
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import bringup  # noqa: E402


def state(flags=StateFlag(0)):
    return McuState(mcu_time_us=1, boot_id=1, arm_gen=0, last_rx_seq=0, mode=Mode.DISARMED,
                    reject_reason=RejectReason.NONE, fault_flags=FaultFlag(0), wheel_count_left=0,
                    wheel_count_right=0, wheel_rate_left_rad_s=0.0, wheel_rate_right_rad_s=0.0, steering_rad=0.25,
                    lift_m=0.0, current_ma=(0, 0, 0, 0), duty_e4=(0, 0, 0, 0), driver_status=(0, 0, 0, 0),
                    rx_errors=0, cycle_overruns=0, flags=flags)


def test_the_commissioning_build_takes_no_drive_and_locked_outputs_stop_a_real_run():
    with pytest.raises(bringup.Refused, match="commission_build"):
        bringup.check_build(state(StateFlag.COMMISSION_BUILD), allow_locked=True)
    with pytest.raises(bringup.Refused, match="outputs_locked"):
        bringup.check_build(state(StateFlag.OUTPUTS_LOCKED), allow_locked=False)
    bringup.check_build(state(StateFlag.OUTPUTS_LOCKED | StateFlag.HIL_PLANT), allow_locked=True)  # model or HIL
    bringup.check_build(state(), allow_locked=False)


def test_the_drive_holds_the_other_axes():
    d = bringup.rear(0.5, 0.0, 0.25)
    assert (d.left_wheel_rad_s, d.right_wheel_rad_s, d.steering_rad) == (0.5, 0.0, 0.25)


def test_the_cap_comes_from_the_vehicle_limit_and_the_unmeasured_radius():
    assert bringup._cap() == pytest.approx(1.0)  # radius unmeasured: 1.0 rad/s, under the 8 rad/s operating limit


def test_an_unconfirmed_operator_refuses(monkeypatch):
    import io

    monkeypatch.setattr(sys, "stdin", io.StringIO("no\n"))
    with pytest.raises(bringup.Refused):
        bringup._confirm(type("A", (), {"yes": False})(), "x")
