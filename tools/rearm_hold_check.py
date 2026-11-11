#!/usr/bin/env python3
"""Re-arm hold check (board design 9; lead decision 2026-11-12 (A)): after the commissioning time limit, ARM stays
refused (reject_reason stopping) until the hold ends, and neither DISARM nor CLEAR_FAULTS moves that end -- the rule
the core's regression test v2r6 pins, seen here from the host through STATE and reject codes.

Targets: an outputs-locked commissioning build (FW_BUILD=commission, FW_MOTOR_OUTPUT=0) on the S2 bench with the
sensors connected, or the host model in CI (mcu_model --commission-build 1 --outputs-locked 1). It refuses any board
whose outputs are not locked: nothing it sends can turn a motor. It is not part of the bring-up tool.

usage:
  rearm_hold_check.py --device D --out run.jsonl --stop-grace-ms MS [--axis L|R] [--duty 0.05] [--limit-s 2.0]

What STATE can show (contract 3.3): last_rx_seq names the last *accepted* message only, and reject_reason keeps the
last refusal until something is accepted. A refused ARM is therefore seen by time: it was written, and a STATE
received REFUSAL_SEEN_NS later is still DISARMED with reject_reason stopping, while rx_errors did not move (nothing
was lost on the link). A CLEAR_FAULTS while DISARMED is ignored by the core (it acts on FAULT or ESTOP only); the
check sends one inside the hold and requires the hold to outlast it all the same.

The hold ends stop_grace_ms after the cycle the limit fired in; that cycle lies between the last ARMED STATE and the
first STOPPING STATE, so the end is known to within one STATE period. The DISARM goes a third of the hold after the
limit is seen (still in its STOPPING) and the CLEAR_FAULTS two thirds in, so a hold either of them restarted would
end at least a third of the hold late -- twice the slack (a sixth of the hold). Pass: refusals seen after the DISARM
was taken and after the CLEAR_FAULTS was written, no other refusal reason before the first ARM taken, no ARM taken
before the earliest possible end, and the first one taken no later than the latest possible end plus the slack.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from forklift_core.mcu_link.messages import Arm, ClearFaults, FaultFlag, Mode, RejectReason, StateFlag

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bringup  # noqa: E402
from real_mcu_link import _BAUDS, ReconnectingLink  # noqa: E402

ARM_PERIOD_S = 0.01
REFUSAL_SEEN_NS = 30_000_000  # a written ARM has reached the MCU and its result a STATE (three STATE periods)
GRACE_MIN_MS = 150.0  # the slack (grace / 6) must stay above two STATE periods plus the ARM cadence


def slack_us(grace_ms: float) -> int:
    return int(grace_ms * 1000 / 6)
AXES = {"L": 0, "R": 1}


def judge(samples: list, limit_i: int, grace_ms: float, disarm_seq: int | None, clear_ns: int | None,
          arm_ns: list[int]) -> dict:
    """Pure verdict. ``samples``: the recorded STATEs (``.host_ns``, ``.state``); ``limit_i``: the first STOPPING
    sample with commission_time_limit; ``clear_ns``, ``arm_ns``: host write times of the CLEAR_FAULTS and ARMs."""
    armed = next((samples[i].state for i in range(limit_i - 1, -1, -1) if samples[i].state.mode == Mode.ARMED), None)
    out = {"result": "not_met", "hold_end_earliest_us": None, "hold_end_latest_us": None}
    if armed is None:
        out["why"] = "no ARMED STATE before the limit"
        return out
    earliest = armed.mcu_time_us + int(grace_ms * 1000)
    latest = samples[limit_i].state.mcu_time_us + int(grace_ms * 1000)
    out["hold_end_earliest_us"], out["hold_end_latest_us"] = earliest, latest
    tail = samples[limit_i:]
    disarm = next((s for s in tail if disarm_seq is not None and s.state.last_rx_seq == disarm_seq), None)
    taken = next((s for s in tail if s.state.mode == Mode.ARMED), None)
    out["disarm_taken"] = disarm is not None
    out["first_arm_taken_us"] = None if taken is None else taken.state.mcu_time_us
    end_ns = taken.host_ns if taken is not None else float("inf")

    def refused_after(t_ns) -> bool:
        """An ARM written at or after ``t_ns`` and before the first taken one, with a STATE received
        REFUSAL_SEEN_NS after it that is still DISARMED with stopping."""
        if t_ns is None:
            return False
        for w in arm_ns:
            if t_ns <= w and w + REFUSAL_SEEN_NS < end_ns and any(
                    s.host_ns >= w + REFUSAL_SEEN_NS and s.state.mode == Mode.DISARMED
                    and s.state.reject_reason == RejectReason.STOPPING for s in tail if s.host_ns < end_ns):
                return True
        return False

    # up to the first taken ARM: the ARMs still in flight after it are refused not_disarmed, as they should be
    others = sorted({s.state.reject_reason.name for s in tail if s.host_ns < end_ns
                     and s.state.reject_reason not in (RejectReason.NONE, RejectReason.STOPPING)})
    errors = {s.state.rx_errors for s in tail}
    checks = {
        "refused_after_disarm": refused_after(None if disarm is None else disarm.host_ns),
        "refused_after_clear_faults": refused_after(clear_ns),
        "no_other_refusal": not others,
        "no_link_errors": len(errors) <= 1,
        "not_shortened": taken is not None and taken.state.mcu_time_us >= earliest,
        "not_lengthened": taken is not None and taken.state.mcu_time_us <= latest + slack_us(grace_ms),
    }
    out["checks"] = checks
    if others:
        out["other_refusals"] = others
    out["result"] = "ok" if all(checks.values()) else "not_met"
    return out


def _queue(b, kind: str, msg) -> int | None:
    """Queue ``msg`` through the host so it gets the link's next seq. ARM and CLEAR_FAULTS go out directly, not
    through the host's arming state machine: that one waits out a repeated refusal (arm_timeout), longer than the
    hold this check probes, and refuses CLEAR_FAULTS outside a latch -- which is exactly what is probed here."""
    b.last_queued = None
    b.host._enqueue(kind, msg, time.monotonic_ns())  # noqa: SLF001 -- see above
    return None if b.last_queued is None else b.last_queued["seq"]


def _written_ns(b, seq: int | None) -> int | None:
    return None if seq is None or seq not in b.written else b.written[seq][1]


def run(b, axis: int, duty: float, limit_s: float, grace_ms: float) -> dict:
    session = bringup.CommissionSession(b, axis, None)
    if not b.run_until(lambda: b.state is not None and b.state.mode == Mode.DISARMED, 3.0):
        return {"result": "refused", "why": "no DISARMED STATE"}
    if not b.state.flags & StateFlag.OUTPUTS_LOCKED:
        return {"result": "refused", "why": "outputs not locked: this check runs on a locked build only"}
    session.arm(bringup.PRE_ZERO_S + limit_s + bringup.LIMIT_SLACK_S + 2.0)
    session.hold_duty(0, bringup.PRE_ZERO_S)
    duty_e4 = int(round(duty * 10000))
    limit_i, scanned = None, len(b.samples)
    give_up = time.monotonic() + limit_s + bringup.LIMIT_SLACK_S
    while limit_i is None and time.monotonic() < give_up and session.aborted is None:
        session.send_duty(duty_e4)
        b.cycle()
        for i in range(scanned, len(b.samples)):
            st = b.samples[i].state
            if st.mode == Mode.STOPPING and st.fault_flags & FaultFlag.COMMISSION_TIME_LIMIT:
                limit_i = i
                break
        scanned = len(b.samples)
        session.aborted = session.aborted or (None if limit_i is not None else session._abort_reason())  # noqa: SLF001
        time.sleep(0.01)
    if limit_i is None:
        session.cleanup()
        return {"result": "not_met", "why": session.aborted or "limit_not_reached"}
    # inside the hold: DISARM a third in, then ARM every ARM_PERIOD_S and one CLEAR_FAULTS two thirds in (once a
    # refusal follows the taken DISARM), until an ARM is taken
    seen_ns = b.samples[limit_i].host_ns
    b.run_until(lambda: False, grace_ms / 3000)
    t0 = time.monotonic_ns()
    b.host.request_disarm(t0)
    b.run_until(lambda: any(k == "DISARM" and t >= t0 for k, t, _m in b.written.values()), 0.5)
    disarm_seq = min((q for q, (k, t, _m) in b.written.items() if k == "DISARM" and t >= t0), default=None)
    clear_seq, arm_seqs = None, []
    end = time.monotonic() + grace_ms / 1000 + 1.0
    while time.monotonic() < end and b.state.mode != Mode.ARMED:
        st = b.state
        if clear_seq is None and time.monotonic_ns() >= seen_ns + grace_ms * 2e6 / 3 and \
                st.reject_reason == RejectReason.STOPPING and any(
                    s.state.last_rx_seq == disarm_seq for s in b.samples[limit_i:]):
            clear_seq = _queue(b, "CLEAR_FAULTS", ClearFaults(st.boot_id))
        seq = _queue(b, "ARM", Arm(st.boot_id, st.mcu_time_us))
        if seq is not None:
            arm_seqs.append(seq)
        b.run_until(lambda: False, ARM_PERIOD_S)
    b.run_until(lambda: False, 0.05)  # the STATEs after the last ARM
    arm_ns = sorted(t for t in (_written_ns(b, q) for q in arm_seqs) if t is not None)
    verdict = judge(b.samples, limit_i, grace_ms, disarm_seq, _written_ns(b, clear_seq), arm_ns)
    verdict.update(arms_written=len(arm_ns), clear_faults_written=_written_ns(b, clear_seq) is not None)
    t1 = time.monotonic_ns()
    b.host.request_disarm(t1)  # the MCU may be ARMED by the check's last ARM
    b.run_until(lambda: b.state.mode == Mode.DISARMED and any(
        k == "DISARM" and t >= t1 for k, t, _m in b.written.values()), 1.0)
    verdict["left_disarmed"] = b.state.mode == Mode.DISARMED
    if not verdict["left_disarmed"]:
        verdict["result"] = "not_met"
    return verdict


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--device", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stop-grace-ms", type=float, required=True,
                    help=f"the build's stop_grace_ms (the hold), at least {GRACE_MIN_MS:g}")
    ap.add_argument("--axis", choices=sorted(AXES), default="L")
    ap.add_argument("--duty", type=float, default=0.05)
    ap.add_argument("--limit-s", type=float, default=2.0, help="the build's commission_time_ms in seconds")
    ap.add_argument("--baud", type=int, choices=sorted(_BAUDS))
    args = ap.parse_args(argv)
    if args.stop_grace_ms < GRACE_MIN_MS:
        print(json.dumps({"result": "refused", "why": f"stop-grace-ms under {GRACE_MIN_MS:g}: too short to judge"}))
        return 2
    if not 0 < abs(args.duty) <= bringup.COMMISSION_DUTY_MAX:
        print(json.dumps({"result": "refused", "why": "duty out of the commissioning tool's range"}))
        return 2
    with open(args.out, "a", encoding="utf-8") as out:
        b = bringup.Bench(ReconnectingLink(args.device, baud=args.baud), out, allow_locked=True, commission=True)
        try:
            result = run(b, AXES[args.axis], args.duty, args.limit_s, args.stop_grace_ms)
        except bringup.Refused as exc:
            result = {"result": "refused", "why": str(exc)}
    print(json.dumps(result, default=str))
    return 0 if result["result"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
