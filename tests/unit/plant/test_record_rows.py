"""The state row shared by the bring-up tool and the teleop bridge (teleop record design 3): moved out of
tools/bringup.py unchanged -- the same row for the same STATE, and back."""

from __future__ import annotations

import json

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, ResetCause, StateFlag
from forklift_core.plant.bringup import Sample
from forklift_core.plant.record_rows import sample_from_row, state_row


def state(**kw):
    base = dict(mcu_time_us=123_456, boot_id=7, arm_gen=2, last_rx_seq=9, mode=Mode.ARMED,
                reject_reason=RejectReason.NONE, fault_flags=FaultFlag.ENCODER_GAP, wheel_count_left=-5,
                wheel_count_right=4_294_967_290, wheel_rate_left_rad_s=0.5, wheel_rate_right_rad_s=-0.25,
                steering_rad=0.125, lift_m=0.01, current_ma=(1, 2, 3, 4), duty_e4=(10, -10, 0, 5),
                driver_status=(0, 1, 0, 0), rx_errors=3, cycle_overruns=1, flags=StateFlag(0),
                reset_cause=ResetCause.POWER)
    base.update(kw)
    return McuState(**base)


def test_the_row_is_what_the_bring_up_record_wrote_and_it_comes_back():
    s = Sample(1_000_000_123, state())
    row = state_row(s)
    # the bring-up record's fixed shape: every McuState field, enums by name, flags as name lists, then t and host_ns
    assert row["t"] == "state" and row["host_ns"] == 1_000_000_123 and row["mode"] == "ARMED"
    assert row["fault_flags"] == ["ENCODER_GAP"] and row["flags"] == [] and row["reset_cause"] == "POWER"
    assert set(McuState.__dataclass_fields__) <= set(row)
    back = sample_from_row(json.loads(json.dumps(row)))
    assert back == s


def test_the_bring_up_tool_still_offers_the_same_functions():
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
    import bringup

    assert bringup.state_row is state_row and bringup.sample_from_row is sample_from_row
