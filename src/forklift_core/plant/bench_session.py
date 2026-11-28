"""The bench console's pure rules (docs/plans/2026-11-27-bench-console.md 3, 3.1 B1): what the one writer does on a
stop by the board's drive state, what a reopened host may do by the first sample after the connection boundary, the
clear-faults request bound to the state it was asked on, the run's first verdict, and the window stop with the two
bypass bits. No I/O: the tool feeds it the latest STATE with its BENCH_STATE (``Pair``)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from forklift_core.mcu_link.messages import (BenchConfirm, BenchCountState, BenchDriveState, BenchRunFlag, FaultFlag,
                                             Mode, RejectReason)


@dataclass(frozen=True)
class Pair:
    """One accepted STATE with the BENCH_STATE that came with it (the board's connection generation is only in the
    latter)."""

    boot_id: int
    conn_gen: int
    arm_gen: int
    mode: Mode
    last_rx_seq: int
    drive_state: BenchDriveState
    run_id: int
    result_run_id: int
    stop_reason: int
    fault_flags: FaultFlag
    duty_e4: tuple
    mcu_time_us: int


def pair_of(state, bench) -> Pair | None:
    """The host's kept pair (``bench_state_of``, ``bench_state``); None until one was seen."""
    if state is None or bench is None:
        return None
    return Pair(boot_id=state.boot_id, conn_gen=bench.conn_gen, arm_gen=state.arm_gen, mode=state.mode,
                last_rx_seq=state.last_rx_seq, drive_state=bench.drive_state, run_id=bench.run_id,
                result_run_id=bench.result_run_id, stop_reason=bench.stop_reason, fault_flags=state.fault_flags,
                duty_e4=tuple(state.duty_e4), mcu_time_us=state.mcu_time_us)


def seq_reached(last_rx_seq: int, seq: int) -> bool:
    """Whether ``last_rx_seq`` is ``seq`` or after it -- the firmware's 16-bit modular comparison (plan 3, 10th round
    P1): ``(int16)(last_rx_seq - seq) >= 0``."""
    d = (int(last_rx_seq) - int(seq)) & 0xFFFF
    return d < 0x8000


def stop_action(*, arm_sent: bool, zero_seq: int | None, zero_link: tuple | None, p: Pair | None,
                arming: bool = False, after_arm: bool = True, after_zero: bool = True, board_end: bool = False,
                latest_stopping: bool = False) -> str:
    """The one stop rule (plan 3, 8th-10th rounds), for ``bench_stop``, the shared STOP, a lost lease or connection:

    - ``cancel``: no BENCH_ARM sent yet -- the run is only dropped;
    - ``send_zero``: armed (accepted or not), no zero sent yet;
    - ``resend_zero``: the zero sent but this sample does not show it taken while armed;
    - ``send_disarm``: this sample shows the zero taken (same link and arming, ``last_rx_seq`` at or after it) and the
      run still preparing -- the zero did not end a drive;
    - ``wait``: driving or stopping after the zero (the board stops itself; a DISARM is never sent), or a sample of
      another link or arming;
    - ``closed`` (DISARMED) / ``closed_latched`` (FAULT, ESTOP).

    ``zero_link``: (boot_id, conn_gen, arm_gen) of the sample the zero was sent against. ``arming``: the host's arming
    exchange is still open (plan 3.1, Codex B1 design P1-3): wait for its outcome -- no zero can be queued and no
    sample closes. ``after_arm``: ``p`` was received after the BENCH_ARM's last byte; an older sample (a late DISARMED
    pair) never closes the run. ``after_zero``: ``p`` was received after the stop zero's last byte and made after the
    STATE it echoed (Codex B1 design round 2 P1-3): the 16-bit comparison alone cannot tell an old 'prep' pair kept by
    the host from a new one, so an older sample is never evidence for or against the zero.

    ``board_end``: this arming's pair carries the run's own result (the board ended it: a distance, a hold's watch, a
    limit, an expiry); ``latest_stopping``: the latest STATE of the run's boot is STOPPING. Either way the board is
    stopping by itself -- after the closing samples (DISARMED, FAULT, ESTOP), ``wait``: no zero, no resend, no DISARM
    from an older ready pair (plan 4.4 B2, Codex plan rounds 2-4)."""
    if not arm_sent:
        return "cancel"
    if p is not None and not after_arm:
        p = None  # a sample from before the ARM went out: no evidence of anything since
    if arming:
        return "closed_latched" if p is not None and p.mode in (Mode.FAULT, Mode.ESTOP) else "wait"
    if p is not None and p.mode == Mode.DISARMED:
        return "closed"
    if p is not None and p.mode in (Mode.FAULT, Mode.ESTOP):
        return "closed_latched"
    if board_end or latest_stopping:
        return "wait"
    if zero_seq is None:
        return "send_zero" if p is None or p.mode == Mode.ARMED else "wait"
    if p is None or not after_zero or zero_link is None or (p.boot_id, p.conn_gen, p.arm_gen) != tuple(zero_link):
        return "wait"
    if p.mode == Mode.STOPPING:
        return "wait"
    taken = seq_reached(p.last_rx_seq, zero_seq)
    if p.mode == Mode.ARMED and not taken:
        return "resend_zero"
    if p.mode == Mode.ARMED and p.drive_state == BenchDriveState.PREP:
        return "send_disarm"
    return "wait"


def after_boundary(last_seen: tuple | None, p: Pair) -> bool:
    """Whether ``p`` comes after a connection boundary the host must wait for (plan 3, 7th round P1-2, 8th round P2):
    a first host takes the first pair; otherwise another boot, or the same boot with another connection
    generation. ``last_seen``: (boot_id, conn_gen) of the last pair seen before the reopening."""
    if last_seen is None:
        return True
    return (p.boot_id, p.conn_gen) != tuple(last_seen)


def reopen_action(p: Pair) -> str:
    """What a reopened host does on the first pair after the boundary -- never a DISARM (the board ended its arming
    at the boundary itself): ``boot_wait`` (initialising: no arming until DISARMED), ``go`` (DISARMED), ``wait_brake``
    (ARMED/STOPPING: wait for DISARMED, FAULT or ESTOP within the brake grace + 1 s) or ``latched`` (FAULT/ESTOP:
    reported, a clear may be asked)."""
    if p.mode == Mode.BOOT:
        return "boot_wait"
    if p.mode == Mode.DISARMED:
        return "go"
    if p.mode in (Mode.ARMED, Mode.STOPPING):
        return "wait_brake"
    return "latched"


@dataclass(frozen=True)
class ClearBinding:
    """A clear-faults request kept with the link -- the host's opening and the board's connection generation (Codex
    B1 round 4 P2-3) -- the boot and the fault bits it was asked on (plan 3, 9th/10th round P2): checked before its
    DISARM is written and again right before the CLEAR_FAULTS frame."""

    conn: int
    conn_gen: int
    boot_id: int
    fault_flags: FaultFlag

    def holds(self, conn: int, conn_gen: int, boot_id: int, fault_flags) -> bool:
        return (conn == self.conn and conn_gen == self.conn_gen and boot_id == self.boot_id
                and FaultFlag(fault_flags) == self.fault_flags)


DISTANCE_ENDS = ("distance_reached", "distance_provisional")  # a distance run's own normal ends (bench6)


def run_verdict(*, closed: str, stop_reason: str | None, aborted: str | None, cleanup_confirmed: bool,
                last: Pair | None, record_errors: int, unexpected_faults=(), distance: bool = False) -> dict:
    """Verdict ① (plan 3): the run and its cleanup -- ended by the run's own zero (the board's first cause
    ``end_zero``), no abort, the cleanup confirmed by the board's end sample then DISARMED, no fault outside the bench's
    ordinary ones, every axis at zero duty, no record row lost. ``closed``: ``ended`` (a run that drove, its result
    read) or ``no_drive`` (closed before driving: never a pass)."""
    why = []
    if closed != "ended":
        why.append(closed)
    elif (distance and stop_reason not in DISTANCE_ENDS) or (not distance and stop_reason != "end_zero"):
        why.append(f"stop_reason:{stop_reason}")  # a distance run ends by its distance only (its own verdict apart)
    if aborted:
        why.append(f"aborted:{aborted}")
    if not cleanup_confirmed:
        why.append("cleanup_unconfirmed")
    if last is None:
        why.append("no_last_sample")
    else:
        if last.mode != Mode.DISARMED:
            why.append(f"last_mode:{last.mode.name}")
        if any(int(d) != 0 for d in last.duty_e4):
            why.append("duty_not_zero")
    if record_errors:
        why.append(f"record_errors:{record_errors}")
    if unexpected_faults:
        why.append("faults:" + ",".join(unexpected_faults))
    return {"ok": not why, "why": why}


# the steering and lift status bits: another axis's are the bench's ordinary state (a motor alone, a sensor not fitted)
_AXIS_STATUS = {2: FaultFlag.STEER_SENSOR_INVALID | FaultFlag.STEER_OUT_OF_RANGE,
                3: FaultFlag.LIFT_SENSOR_INVALID | FaultFlag.LIFT_OUT_OF_RANGE}
_RANGE_BIT = {2: ("steer_range", FaultFlag.STEER_OUT_OF_RANGE), 3: ("lift_range", FaultFlag.LIFT_OUT_OF_RANGE)}


def unexpected_faults(fault_flags, axis: int, bypass=()) -> tuple:
    """The fault bits verdict ① counts (plan 4.4 B2, independent review P2-5): every bit but the command timeout, the
    steering and lift status bits of an axis that is not the run's, and the run axis's own range bit when its range
    bypass is on. The firmware keeps those status bits in STATE whatever the run (a motor-only M is out of range, an
    unfitted H's sensor invalid) while its drive protection follows the run's axis and bypass."""
    flags = FaultFlag(int(fault_flags))
    # bench8 (plan 4.5 A1c decision 4): the wheel gap and jump bits are records only -- ENCODER_GAP is shared by both
    # wheels, so the bit cannot say whose loss it was; a loss that ended a run as a fault shows as the MCU's FAULT mode
    expected = FaultFlag.COMMAND_TIMEOUT | FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_L | FaultFlag.ENCODER_JUMP_R
    for ax, bits in _AXIS_STATUS.items():
        if ax != axis:
            expected |= bits
    if axis in _RANGE_BIT and _RANGE_BIT[axis][0] in tuple(bypass):
        expected |= _RANGE_BIT[axis][1]
    return tuple(f.name for f in FaultFlag if f in flags and f not in expected)


def first_answer(evidence, *, seq16: int, run_id: int, write, lim_mcu_us: int, lim_host_ns: int, now_ns: int,
                 stop_zero_done_ns: int | None = None) -> str | None:
    """The answer to a run's one-shot first frame -- a distance run's first non-zero COMMISSION or a hold's first
    BENCH_HOLD (plan 4.3 decision 10, 4.4 B2 decision 3) -- shared by the CLI and the console.

    ``evidence``: ``(kind, STATE, BENCH_STATE or None, received ns[, host connection])`` the host accepted, kind
    ``state`` or ``pair``; with a connection given on both, a sample of another opening than the write's is not
    evidence (Codex B2a-1 impl P2-3).
    They are judged in MCU-time order, a STATE before its own pair (the host calls every STATE hook of a read before
    its pair hooks). Only samples of the write's boot and arming, received after its last byte and made after the STATE
    it echoed, count; the first one past either deadline (``lim_mcu_us`` on the MCU clock, ``lim_host_ns`` on the host
    clock) closes the wait unanswered. ``write``: the frame's write record (``done_ns``, ``boot_id``, ``arm_gen``,
    ``echo_mcu_time_us``), None while not written whole -- then only the host deadline closes it.
    ``stop_zero_done_ns``: once a stop zero was written, a STATE received at or after it no longer speaks for the
    first frame (its ``last_rx_seq`` and reject reason may be the zero's; the same receipt time cannot order them);
    only a pair carrying this run's result does.

    Returns ``accepted``, ``refused:<reason>``, ``confirmed_by_result``, ``unconfirmed`` (a deadline passed), or None
    (still waiting). A refusal is a reject reason on a STATE whose ``last_rx_seq`` is still before the first frame: the
    tool writes the first frame only after its last readiness zero was accepted (plan 4.4, independent round 2
    P2-1), so nothing else sent can have been refused there."""
    if write is None:
        return "unconfirmed" if now_ns > lim_host_ns else None
    order = sorted(range(len(evidence)),
                   key=lambda i: (evidence[i][1].mcu_time_us, 0 if evidence[i][0] == "state" else 1, i))
    write_conn = getattr(write, "conn", None)
    for i in order:
        kind, st, bench, rx, *more = evidence[i]
        conn = more[0] if more else None
        if conn is not None and write_conn is not None and conn != write_conn:
            continue
        if not (rx >= write.done_ns and st.boot_id == write.boot_id and st.arm_gen == write.arm_gen
                and st.mcu_time_us > write.echo_mcu_time_us):
            continue
        if st.mcu_time_us > lim_mcu_us or rx > lim_host_ns:
            return "unconfirmed"  # past a deadline with nothing in time before it
        if kind == "pair":
            if bench is not None and bench.result_run_id == run_id:
                return "confirmed_by_result"
            continue
        if stop_zero_done_ns is not None and rx >= stop_zero_done_ns:
            continue
        if (int(st.last_rx_seq) & 0xFFFF) == seq16:
            return "accepted"
        if st.reject_reason != RejectReason.NONE and not seq_reached(st.last_rx_seq, seq16):
            return f"refused:{st.reject_reason.name.lower()}"
    return "unconfirmed" if now_ns > lim_host_ns else None


def window_stop(value: float, window: tuple, *, kth_invalid_bypass: bool, range_off: bool) -> str | None:
    """The drive-time window stop of a steering or lift run (plan 3, the two bypass bits' table): none with the range
    off; with the KTH-invalid bypass an invalid (NaN) sample is skipped, a valid one outside still stops; without it an
    invalid sample counts as outside (the board faults on it too)."""
    if range_off:
        return None
    lo, hi = window
    if not math.isfinite(value):
        return None if kth_invalid_bypass else "window"
    return None if lo <= value <= hi else "window"  # the CLI's inclusive bounds (bringup early_stop)


def counts_diff(a: int, b: int) -> int:
    """a - b of two modular 32-bit counts as a signed difference (contract 8.5: the count wraps)."""
    d = (int(a) - int(b)) & 0xFFFFFFFF
    return d - (1 << 32) if d >= 1 << 31 else d


_STOPPED = {"no_progress", "kth_zero_suspect", "kth_lost"}


def distance_verdict(*, stop_reason: str | None, confirm: int, run_flags: int, stop_counts: int | None,
                     distance_ref: int, cut_position: int) -> dict:
    """Verdict ② of a distance run (plan 4.2 A1b-1 decision 8), apart from the run and cleanup verdict:
    ``reached`` (stop 8, or 9 confirmed reached), ``held`` (9 held by a rebound), ``waiting`` (9 with no confirmation
    yet), ``failed_loss`` (9 whose confirmation ended by a loss), ``stopped:<reason>`` (the watch or the count),
    ``missed`` (ended before the target: zero, time limit, expiry, ...). The reached position is the one at the cut
    (or the first trusted sample after it), never the final rest."""
    flags = BenchRunFlag(run_flags & 0x1F)
    moved = counts_diff(cut_position, distance_ref)
    out = {"stop_counts": None if stop_counts is None else int(stop_counts), "moved_at_cut": moved}
    if BenchRunFlag.RES_UNCLEAN in flags:
        return {**out, "result": "failed_continuity"}  # never with a distance end (the drive is refused), defensive
    if stop_reason == "distance_reached":
        return {**out, "result": "reached"}
    if stop_reason == "distance_provisional":
        if confirm == int(BenchConfirm.REACHED):
            return {**out, "result": "reached", "confirmed_after_cut": True}
        if confirm == int(BenchConfirm.HELD):
            return {**out, "result": "failed_loss" if BenchRunFlag.RES_LOSS in flags else "held"}
        return {**out, "result": "waiting"}
    if stop_reason in _STOPPED:
        return {**out, "result": f"stopped:{stop_reason}"}
    return {**out, "result": "missed", "ended_by": stop_reason}


def measure_verdict(*, run_flags: int, distance_ref: int, cut_position: int) -> dict:
    """A measuring KTH run's end displacement and how far it is to be trusted (plan 4.2 A1b-1 decision 5): withheld
    without a fresh reference at the drive start (bit4) or with the count broken (bit1), provisional when it ended on
    a 0x0000 frame (bit3), else confirmed."""
    flags = BenchRunFlag(run_flags & 0x1F)
    moved = counts_diff(cut_position, distance_ref)
    if BenchRunFlag.RES_NO_REF in flags:
        return {"moved": moved, "status": "withheld:no_reference"}
    if BenchRunFlag.RES_UNCLEAN in flags:
        return {"moved": moved, "status": "withheld:continuity"}
    if BenchRunFlag.RES_TENTATIVE in flags:
        return {"moved": moved, "status": "provisional"}
    return {"moved": moved, "status": "confirmed"}


REST_SETTLE_US = 500_000  # 0.5 s with no change at 100 Hz
REST_EVIDENCE_MS = 100
REST_GAP_US = 15_000  # 1.5 x the 10 ms STATE period (plan 3): a 100 Hz observation, never 50 Hz


REST_OBSERVE_MT_S = 1.0  # plan 4.5 A1c decision 9: the wheel's inertia may move it past 0.1 s; KTH keeps 0.6 s
MT_SOURCE = 2


def _rest_mt(samples) -> dict:
    """The MT rest position (plan 4.5 A1c decision 9): back from the last pair, the contiguous segment with one count
    where every pair is this run's MT on its axis, VALID, without the live UNCLEAN mark, with a sample accepted at its
    own tick (raw_angle 0 = MT sample age 0) and within REST_GAP_US of the next; 0.5 s of it is ``unchanged_at_100hz``
    -- never ``settled``: a stopped MT count has no read freshness evidence."""
    t_last, last = samples[-1]
    start, why = t_last, None
    for i in range(len(samples) - 1, -1, -1):
        t, b = samples[i]
        if (b.run_id != last.run_id or b.source != MT_SOURCE or b.axis != last.axis
                or getattr(b, "conn_gen", None) != getattr(last, "conn_gen", None)
                or b.count_state != int(BenchCountState.VALID) or b.run_flags & int(BenchRunFlag.UNCLEAN)):
            why = "continuity"
            break
        if b.raw_angle != 0:
            why = "stale_read"  # this pair's read failed: its count is an older sample's
            break
        if b.cumulative != last.cumulative:
            why = "not_settled"
            break
        if i + 1 < len(samples) and not 0 < samples[i + 1][0] - t <= REST_GAP_US:
            why = "sparse"
            break
        start = t
    if t_last - start < REST_SETTLE_US:
        return {"result": "unconfirmed", "cumulative": last.cumulative, "why": why or "too_short"}
    return {"result": "unchanged_at_100hz", "cumulative": last.cumulative,
            "wording": "100 Hz 관측에서 0.5 s 동안 카운트 그대로(MT 는 읽기 신선도를 확인할 수 없음)"}


def rest_position(samples, source: int = 1) -> dict:
    """The position after DISARMED (plan 4.2 A1b-1 decision 4): ``samples`` are (mcu_time_us, BenchState) of the
    run axis after DISARMED, oldest first. Settled only when the live count stayed whole (bit0 clear, VALID), the
    count did not change for 0.5 s of the 100 Hz samples and the evidence is fresh; worded as a 100 Hz observation.
    Otherwise the last value is recorded as unconfirmed. The MT source has its own rule (``_rest_mt``)."""
    if not samples:
        return {"result": "unconfirmed", "why": "no_sample"}
    if source == MT_SOURCE:
        return _rest_mt(samples)
    t_last, last = samples[-1]
    if last.run_flags & int(BenchRunFlag.UNCLEAN) or last.count_state != int(BenchCountState.VALID):
        return {"result": "unconfirmed", "cumulative": last.cumulative, "why": "continuity"}
    if last.evidence_age_ms > REST_EVIDENCE_MS:
        return {"result": "unconfirmed", "cumulative": last.cumulative, "why": "evidence_age"}
    # back from the last sample, the contiguous segment where every sample has the same count, re-anchored on a
    # non-zero frame with fresh evidence, of the same run, each within REST_GAP_US of the next; settled only when
    # that segment itself spans 0.5 s (impl r1 P1-4, r2 P1)
    start, why = t_last, None
    for i in range(len(samples) - 1, -1, -1):
        t, b = samples[i]
        if b.run_id != last.run_id or b.raw_angle == 0 or b.evidence_age_ms > REST_EVIDENCE_MS:
            why = "not_reanchored"
            break
        if b.cumulative != last.cumulative:
            why = "not_settled"
            break
        if i + 1 < len(samples) and samples[i + 1][0] - t > REST_GAP_US:
            why = "sparse"
            break
        start = t
    if t_last - start < REST_SETTLE_US:
        return {"result": "unconfirmed", "cumulative": last.cumulative, "why": why or "too_short"}
    return {"result": "settled_at_100hz", "cumulative": last.cumulative,
            "wording": "100 Hz 관측에서 0.5 s 동안 위치 변화 없음"}


# ---- bench7 hold (plan 4.3 A1b-2 decisions 7 and 9) ------------------------------------------------------------------
HOLD_GAP_US = 15_000  # 1.5 x the 10 ms STATE period: 11-14 ms jitter joins, one lost pair (20 ms) breaks
HOLD_SETTLE_US = 1_000_000
HOLD_SETTLE_MIN = 50
HOLD_INSIDE_RATIO = 0.8
HOLD_TAIL_US = 200_000
HOLD_EVIDENCE_MS = 100
HOLD_EVIDENCE_MT_MS = 1000  # plan 1, 4.5 A1c decision 8: the MT evidence (the last qualified progress) within 1 s


def _inside(b, deadband: int, source: int = 1) -> bool:
    if source == MT_SOURCE:  # a VALID sample accepted at its own pair's tick only (plan 4.5 A1c decisions 8, 10)
        return (abs(counts_diff(b.target, b.cumulative)) <= deadband and b.raw_angle == 0
                and getattr(b, "count_state", int(BenchCountState.VALID)) == int(BenchCountState.VALID))
    return abs(counts_diff(b.target, b.cumulative)) <= deadband and b.raw_angle != 0  # a tentative 0 is never inside


def hold_verdict(pairs, result: dict, *, deadband: int | None, run_id: int, source: int = 1) -> dict:
    """Verdict ② of a hold run (decision 9): failure, then hold-back, then the settling judgement.

    ``pairs``: (mcu_time_us, Mode, BenchState) of the run in receipt order; ``result``: the board's result
    (``result_fields``: stop_reason name, run_flags, end_evidence_age_ms, target, cut_position); ``deadband``: the
    run's first accepted HOLD's (None when only the board's result was recovered)."""
    if deadband is None:
        return {"result": "undecidable", "why": ["no_settings"]}
    flags = BenchRunFlag(int(result.get("run_flags") or 0) & 0x7F)
    stop = result.get("stop_reason")
    if stop != "end_zero":
        return {"result": "failed", "why": [f"stop_reason:{stop}"]}
    if BenchRunFlag.RES_UNCLEAN in flags:
        return {"result": "failed", "why": ["continuity"]}
    age = result.get("end_evidence_age_ms")
    mt = source == MT_SOURCE
    if age is None or int(age) > (HOLD_EVIDENCE_MT_MS if mt else HOLD_EVIDENCE_MS):
        if mt:
            # a settled MT hold has no fresh evidence by nature (plan 4.5 A1c decision 8): held, with what was observed
            seen = _settle_seen(pairs, run_id, deadband, source)
            return {"result": "held", "why": ["mt_evidence_age"], "evidence_age_ms": age,
                    "note": "MT 는 멈춰 있으면 읽기가 살아 있는지 확인할 수 없음", **seen}
        return {"result": "held", "why": ["0 프레임만 — 고착과 구별 불가"]}
    if BenchRunFlag.RES_TENTATIVE in flags:
        return {"result": "held", "why": ["tentative_end"]}
    end_error = counts_diff(int(result["target"]), int(result["cut_position"]))
    out = {"end_error": end_error}
    if abs(end_error) > deadband:
        return {**out, "result": "unmet", "why": ["end_outside"]}
    run = [(t, m, b) for t, m, b in pairs if b.run_id == run_id and b.mode == 2]  # this hold run's pairs only
    driving = [i for i, (t, m, b) in enumerate(run) if m == Mode.ARMED and b.drive_state == BenchDriveState.DRIVING]
    if not driving:
        return {"result": "undecidable", "why": ["no_samples"]}
    last = driving[-1]
    t_last = run[last][0]
    end = next(((t, b) for t, m, b in run[last + 1:]
                if b.drive_state == BenchDriveState.ENDED and b.result_run_id == run_id), None)
    if end is None or end[0] - t_last > HOLD_GAP_US:
        return {**out, "result": "unmet", "why": ["end_gap"]}
    seg = [last]  # back from the last driving pair, the contiguous driving segment
    for i in range(last - 1, -1, -1):
        t, m, b = run[i]
        if not (m == Mode.ARMED and b.drive_state == BenchDriveState.DRIVING) or run[seg[-1]][0] - t > HOLD_GAP_US:
            break
        seg.append(i)
    seg.reverse()
    first_in = next((i for i in seg if _inside(run[i][2], deadband, source)), None)
    if first_in is None:
        return {**out, "result": "unmet", "why": ["never_inside"]}
    settled = [i for i in seg if i >= first_in]
    t0 = run[first_in][0]
    inside = sum(1 for i in settled if _inside(run[i][2], deadband, source))
    out["settle"] = {"from_us": t0, "to_us": t_last, "samples": len(settled), "inside": inside}
    why = []
    if t_last - t0 < HOLD_SETTLE_US:
        why.append("settle_short")
    if len(settled) < HOLD_SETTLE_MIN:
        why.append("settle_few")
    if inside < HOLD_INSIDE_RATIO * len(settled):
        why.append("settle_ratio")
    if any(not _inside(run[i][2], deadband, source) for i in settled if run[i][0] >= t_last - HOLD_TAIL_US):
        why.append("tail_outside")
    if why:
        return {**out, "result": "unmet", "why": why}
    return {**out, "result": "met", "why": []}


def _settle_seen(pairs, run_id: int, deadband: int, source: int) -> dict:
    """What a held MT hold observed: the time from the first pair inside the deadband to the last driving pair of the
    run, and the share inside (shown with the hold-back, plan 4.5 A1c decision 8)."""
    run = [(t, m, b) for t, m, b in pairs if b.run_id == run_id and b.mode == 2]
    drv = [(t, b) for t, m, b in run if m == Mode.ARMED and b.drive_state == BenchDriveState.DRIVING]
    first = next((i for i, (t, b) in enumerate(drv) if _inside(b, deadband, source)), None)
    if first is None:
        return {"inside_s": 0.0}
    tail = drv[first:]
    inside = sum(1 for _t, b in tail if _inside(b, deadband, source))
    # the time is the sum of the gaps between two neighbouring inside pairs no wider than HOLD_GAP_US: a reception
    # gap or a pair outside is never counted as seen inside (Codex A1c-2 r1 P2-3)
    seen = sum(t1 - t0 for (t0, b0), (t1, b1) in zip(tail, tail[1:], strict=False)
               if _inside(b0, deadband, source) and _inside(b1, deadband, source) and 0 < t1 - t0 <= HOLD_GAP_US)
    return {"inside_s": round(seen / 1e6, 3), "inside_share": round(inside / len(tail), 3)}


ENTRY_WINDOW_US = 1_000_000
ENTRY_WARN = 10
ENTRY_GAP_MAX_US = 65_000_000  # a 16-bit count at <= 1 kHz cannot wrap within this


def entry_points(pairs, run_id: int) -> list:
    """The run's (mcu_time_us, deadband_entries) points from (mcu_time_us, Mode, BenchState) pairs of one boot and
    connection, with a zero point (virtual) at the last pair before the run when one was seen: the count starts at 0
    at the ARM, which came after it (rounds 11-12)."""
    pts, before = [], None
    for t, m, b in pairs:
        if b.run_id == run_id and (m in (Mode.ARMED, Mode.STOPPING) or pts):
            if not pts and before is not None:
                pts.append((before, 0, True))
            pts.append((t, int(b.deadband_entries)))
        elif not pts:
            before = t
    return pts


def entry_window(points, t_now: int) -> dict:
    """The last second's deadband entries from the run's modular count (decision 7, rounds 10-12): lower =
    C(now) - C(first point after the window start), upper = C(now) - C(last point at or before it); a zero point
    (virtual) stands for every time up to it. ``label``: ``warn`` (lower >= 10), ``unknown`` (upper >= 10 > lower, or
    no base, or a gap over 65 s), else ``none``. Never blocks anything."""
    if not points:
        return {"lower": None, "upper": None, "label": "unknown"}
    t0 = t_now - ENTRY_WINDOW_US
    c_now = points[-1][1]
    virtual_in = len(points[0]) > 2 and points[0][2] and points[0][0] >= t0  # the ARM came after the window start
    after = (t0, 0) if virtual_in else next((p for p in points if p[0] > t0), points[-1])
    lower = (c_now - after[1]) & 0xFFFF
    before = [p for p in points if p[0] <= t0]
    if before:
        base = before[-1]
    elif len(points[0]) > 2 and points[0][2]:
        base = (t0, 0)  # the ARM came after the window start: the count was 0 there
    else:
        return {"lower": lower, "upper": None, "label": "unknown"}
    span = [p for p in points if p[0] >= base[0]]
    if any(b[0] - a[0] > ENTRY_GAP_MAX_US for a, b in zip(span, span[1:])):
        return {"lower": lower, "upper": None, "label": "unknown"}
    upper = (c_now - base[1]) & 0xFFFF
    label = "warn" if lower >= ENTRY_WARN else ("unknown" if upper >= ENTRY_WARN else "none")
    return {"lower": lower, "upper": upper, "label": label}


__all__ = ["DISTANCE_ENDS", "ClearBinding", "Pair", "after_boundary", "counts_diff", "distance_verdict",
           "entry_points", "entry_window", "hold_verdict", "measure_verdict", "pair_of", "reopen_action",
           "rest_position", "run_verdict", "seq_reached", "stop_action", "window_stop"]
