"""The ``duty_disarm`` scenario's validity (docs/design/2026-11-25-bench-commission-limits.md 1.3, PR-2b).

On the open commissioning image the wheels' time limit is the validator's ceiling, so the S5 stop measured after the
MCU's own limit (``limit_disarm``) is replaced by a stop after the tool's own DISARM at a chosen duty. The test
counts only when the record shows, in one run, boot and arm generation:

1. the wheel's applied duty reached the target (within 1 e4) and stayed there for ``HOLD_US`` of MCU time;
2. right after, the test DISARM (phase ``test_disarm``) was written, and its acceptance is a transition of
   ``last_rx_seq`` to its seq (``write_log.accepted``);
3. the STATE just before that acceptance was still ARMED at the target duty;
4. from the reach to the acceptance: no command timeout, no commissioning time limit, no fault or e-stop, no new boot.

The physical items (output response, stop) are judged only on an open board: the run's STATEs report a commissioning
build with outputs not locked and no HIL plant. A locked board or the model computes the same duty without driving a
motor, so its run proves the sequence, not a stop.

The functions read STATE rows as the bring-up record writes them (``record_rows.state_row`` plus ``conn``), so the
tool and ``bringup.py analyze`` judge the same thing.
"""

from __future__ import annotations

from forklift_core.plant.write_log import Seen, Write, accepted

HOLD_US = 500_000
DUTY_TOL_E4 = 1
# A provisional bench floor, not a measured visibility bound: 2 e4 would already keep a zero output out of the +-1
# tolerance (and _at_target excludes zero outright); 0.01 is kept so the test drives something a person can see.
MIN_DUTY_E4 = 100
GAP_MAX_NS = 50_000_000  # between consecutive STATEs from the reach to the acceptance (5 STATE periods), host reads
GAP_MAX_US = 50_000  # and the MCU's own times of the same STATEs (a buffered burst reads close but was not)
STATE_PERIOD_S = 0.010
SEND_MARGIN_S = 0.020
WHEEL_SLEW_PER_S = 2.0  # board_config.c (pinned by the bench-limits test)
_BAD_BITS = {"COMMAND_TIMEOUT", "COMMISSION_TIME_LIMIT", "ESTOP_INPUT"}


def min_seconds(duty: float) -> float:
    """The shortest run that can show the hold in STATEs: the ramp, two STATE periods to see the reach, the hold, two
    periods to see its end, and the send margin."""
    return abs(duty) / WHEEL_SLEW_PER_S + 2 * STATE_PERIOD_S + HOLD_US / 1e6 + 2 * STATE_PERIOD_S + SEND_MARGIN_S


def _seen(r: dict) -> Seen:
    return Seen(r["host_ns"], r.get("conn", 0), r["boot_id"], r["mcu_time_us"], r["last_rx_seq"], r.get("arm_gen", 0))


def _at_target(r: dict, axis: int, duty_e4: int) -> bool:
    applied = int(r["duty_e4"][axis])
    return r["mode"] == "ARMED" and applied != 0 and abs(applied - duty_e4) <= DUTY_TOL_E4


def _bad(r: dict) -> set:
    return (set(r.get("fault_flags", ())) & _BAD_BITS) | ({"FAULT"} if r["mode"] in ("FAULT", "ESTOP") else set())


def reach_and_hold(states: list[dict], axis: int, duty_e4: int, start: int = 0) -> tuple[int | None, int | None]:
    """(index of the first STATE at the target, index of the first one HOLD_US later with every STATE between at the
    target); either None when not seen."""
    reach = None
    for i in range(start, len(states)):
        r = states[i]
        if not _at_target(r, axis, duty_e4):
            reach = None
            continue
        if reach is None:
            reach = i
        elif r["mcu_time_us"] - states[reach]["mcu_time_us"] >= HOLD_US:
            return reach, i
    return reach, None


def physical(states: list[dict]) -> bool:
    """An open commissioning board drove a real motor in this run."""
    flags = [set(r.get("flags", ())) for r in states]
    return bool(flags) and all("COMMISSION_BUILD" in f and "OUTPUTS_LOCKED" not in f and "HIL_PLANT" not in f
                               for f in flags)


def validity(states: list[dict], writes: list[Write], *, axis: int, duty_e4: int, synthetic: bool = False) -> dict:
    """The four conditions above; ``{"valid": bool, "reason": str | None, ...}`` with the indices it used.
    ``synthetic``: a model run (a pty) -- never physical whatever its build bits say (Codex PR-2b P1-3)."""
    out = {"valid": False, "reason": None, "reach_index": None, "hold_index": None, "acceptance_index": None,
           "disarm_seq": None, "disarm_done_ns": None, "physical": physical(states) and not synthetic}
    if abs(duty_e4) < MIN_DUTY_E4:
        out["reason"] = "target_too_small"
        return out
    seen = [_seen(r) for r in states]
    # the reach is counted only from the first nonzero COMMISSION of the drive accepted (Codex PR-2b P1-1)
    m_on = accepted(seen, writes, lambda w: w.kind == "COMMISSION" and w.phase == "on" and w.duty_e4 == duty_e4)
    if m_on.index is None:
        out["reason"] = "seq_ambiguous" if m_on.ambiguous_at is not None else "drive_not_accepted"
        return out
    reach, hold = reach_and_hold(states, axis, duty_e4, m_on.index)
    out["reach_index"], out["hold_index"] = reach, hold
    if hold is None:
        ever = any(_at_target(r, axis, duty_e4) for r in states)
        out["reason"] = "hold_not_seen" if ever else "target_not_reached"
        return out
    m = accepted(seen, writes, lambda w: w.kind == "DISARM" and w.phase == "test_disarm", reach)
    if m.ambiguous_at is not None:
        out["reason"] = "seq_ambiguous"
        return out
    if m.index is None:
        out["reason"] = "disarm_not_accepted"
        return out
    i_acc = m.index
    out.update(acceptance_index=i_acc, disarm_seq=m.write.seq, disarm_done_ns=m.write.done_ns)
    prev = next((j for j in range(i_acc - 1, -1, -1) if seen[j].conn == seen[i_acc].conn), None)
    if prev is None or not _at_target(states[prev], axis, duty_e4):
        out["reason"] = "not_at_target_before_disarm"
        return out
    # continuity from the reach to the acceptance (Codex PR-2b P2-1): one connection, no gap, at the target all along
    if any(seen[j].conn != seen[reach].conn for j in range(reach, i_acc + 1)):
        out["reason"] = "connection_changed"
        return out
    if any(seen[j + 1].host_ns - seen[j].host_ns > GAP_MAX_NS or seen[j + 1].mcu_time_us - seen[j].mcu_time_us > GAP_MAX_US
           for j in range(reach, i_acc)):
        out["reason"] = "state_gap"
        return out
    # the STATE reporting the DISARM's seq shows the stop itself: DISARMED with the driven wheel's output at zero
    # (a DISARM taken while ARMED drops to DISARMED in the same step, drive.c) (Codex PR-2b re-review P2)
    acc = states[i_acc]
    if acc["mode"] != "DISARMED" or int(acc["duty_e4"][axis]) != 0:
        out["reason"] = "not_stopped_at_acceptance"
        return out
    if not all(_at_target(states[j], axis, duty_e4) for j in range(reach, i_acc)):
        out["reason"] = "left_the_target_before_disarm"
        return out
    w_dis = m.write
    if w_dis.done_ns < states[hold]["host_ns"]:
        out["reason"] = "disarm_before_the_hold_was_seen"
        return out
    first_on = next((w for w in writes if w.kind == "COMMISSION" and w.phase == "on"), None)
    if first_on is None or any(w.kind == "COMMISSION" and w.duty_e4 != duty_e4
                               for w in writes if first_on.order <= w.order < w_dis.order):
        out["reason"] = "another_command_before_disarm"
        return out
    span = states[reach:i_acc + 1]
    if any(r["boot_id"] != states[reach]["boot_id"] for r in span):
        out["reason"] = "boot_changed"
        return out
    if any(r.get("arm_gen") != states[reach].get("arm_gen") for r in span):
        out["reason"] = "arm_generation_changed"
        return out
    bad = set().union(*(_bad(r) for r in span))
    if bad:
        out["reason"] = "stopped_otherwise:" + ",".join(sorted(bad))
        return out
    out["valid"] = True
    return out
