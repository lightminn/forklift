"""Record rows shared by the bring-up tool and the teleop bridge (docs/design/2026-11-14-teleop-record-state-markers-
clock.md 3): one STATE as a JSON-ready row and back. ``host_ns`` is the time the host attached when it accepted the
STATE (the teleop bridge: its cycle start -- see ``read_ns`` there for the read time)."""

from __future__ import annotations

from dataclasses import asdict

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, ResetCause, StateFlag
from forklift_core.plant.bringup import Sample


def state_row(s: Sample) -> dict:
    st = s.state
    row = asdict(st)
    row.update(t="state", host_ns=s.host_ns, mode=st.mode.name, reject_reason=st.reject_reason.name,
               fault_flags=[f.name for f in FaultFlag if f in st.fault_flags],
               flags=[f.name for f in StateFlag if f in st.flags], reset_cause=st.reset_cause.name)
    return row


def sample_from_row(row: dict) -> Sample:
    """A recorded state row back into a Sample (analyze)."""
    faults, flags = FaultFlag(0), StateFlag(0)
    for name in row["fault_flags"]:
        faults |= FaultFlag[name]
    for name in row["flags"]:
        flags |= StateFlag[name]
    fields = {k: row[k] for k in McuState.__dataclass_fields__ if k in row}
    fields.update(mode=Mode[row["mode"]], reject_reason=RejectReason[row["reject_reason"]], fault_flags=faults,
                  flags=flags, reset_cause=ResetCause[row["reset_cause"]],
                  current_ma=tuple(row["current_ma"]), duty_e4=tuple(row["duty_e4"]),
                  driver_status=tuple(row["driver_status"]))
    for name in ("kth_frames", "kth_crc_errors", "kth_alt_crc_errors"):  # v3; JSON gives lists
        if name in row:
            fields[name] = tuple(row[name])
    return Sample(int(row["host_ns"]), McuState(**fields))
