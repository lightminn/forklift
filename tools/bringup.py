#!/usr/bin/env python3
"""S1–S6 bring-up host tools ① (docs/design/2026-11-12-bringup-host-tools.md): record, cpr, step, scenario, analyze;
and ② (docs/design/2026-11-12-bringup-commission-tool.md): commission, S3 axis by axis on a commissioning build.

Not an executor (mcu-link/2 contract 3.3): it connects to any build and records the build bits, faults and reset
cause; it drives only under the design's section 3 conditions (the host refuses a commissioning build and, unless
allowed for a model or HIL, locked outputs). Everything it measures is software evidence (STATE): the S5 pass
verdict takes external measurements (oscilloscope, floor distance) given on the command line, or leaves an item
'unavailable'.

usage:
  bringup.py record   --device D --out run.jsonl [--seconds 30] [--markers]
  bringup.py cpr      --device D --out run.jsonl --turns-left N --turns-right N [--dir-left 1] [--dir-right 1]
                      [--motor-counts-confirmed]
  bringup.py step     --device D --out run.jsonl --wheel left|right|both --target RAD_S [--yes]
  bringup.py scenario --device D --out run.jsonl --kind tx_cut|disarm|expiry [--rate RAD_S] [--yes]
                      [--floor --power-switch-reachable] [--output-response-ms X|unavailable]
                      [--stop-time-ms X|unavailable] [--stop-distance-mm X|unavailable]
  bringup.py analyze  --record run.jsonl [--marker N] [--kind tx_cut|disarm|expiry|iwdg|power] [external ...]
                      (--marker counts from 0: 0 = the first marker, -1 = the last, the default)
  bringup.py commission --device D --out run.jsonl --axis L|R|M|H --duty X --seconds S [--window LO HI]
                      [--observed pos|neg|none] --power-switch-in-hand [--allow-locked] [--yes] [--markers]
  bringup.py commission --device D --out run.jsonl --axis L|R --duty X --scenario limit_disarm|hold_disarm
                      [--limit-s 2.0] [--disarm-delay-ms 50] --power-switch-in-hand [external ...]
  bringup.py kth      --record run.jsonl --configured-m V --configured-h V --image ID
                      (S2: the KTH CRC variant from the v3 counters; record while moving each axis through its range)
  bringup.py clock    --record run.jsonl
                      (G7: MCU time -> host monotonic time per host boot and MCU boot, the delay envelope fit and its
                      quality figures; a teleop record's read_ns, a bring-up record's host_ns. No marker needed.)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from forklift_core.mcu_link.build_policy import Client, commission_allowed
from forklift_core.mcu_link.messages import (
    Drive,
    FaultFlag,
    LinkDecoder,
    McuState,
    Mode,
    RejectReason,
    ResetCause,  # noqa: F401 -- re-exported for the tool's tests
    StateFlag,
)
from forklift_core.plant.bringup import (
    Sample,
    count_settle,
    step_metrics,
    wheel_cpr,
    wheel_rate_cap,
)
from forklift_core.plant.clock_map import clock_fits_from_rows
from forklift_core.plant.commission import (
    AXES,
    AXIS_HOLD_MAX_S,
    COMMISSION_CAP_MS,
    COMMISSION_DUTY_MAX,
    COMMISSION_HOLD_MAX_S,
    COMMISSION_HOLD_MIN_S,
    commission_metrics,
    sign_verdict,
)
from forklift_core.plant.kth_check import VARIANTS as KTH_VARIANTS
from forklift_core.plant.kth_check import kth_check
from forklift_core.plant.markers import marker_reader
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost
from forklift_core.plant.real_vehicle import RearDrive, load_real_vehicle
from forklift_core.plant.record_rows import (  # noqa: F401 -- moved; kept here by name
    sample_from_row,
    state_row,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from real_mcu_link import _BAUDS, ReconnectingLink  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
STEP_MAX_S = 3.0
CLEANUP_S = 1.0
OBSERVE_S = 2.0
STATE_GAP_S = 0.2  # after an event, two STATEs further apart than this are an observation failure
HANDOVER_S = 0.05  # a nonzero hold stops sending this much before its end, so the next hold's first command lands
# while the last nonzero one is still valid -- and no nonzero command outlives its hold (Codex re-review P2-1)
# plan 3.4 thresholds (provisional, 0.2 m/s empty truck)
OUTPUT_RESPONSE_MS = {"disarm": 100.0, "iwdg": 100.0}
# DISARM: provisional, the link loss's values (lead decision 2026-11-12, plan 3.4) -- reviewed after the S5
# measurements, never relaxed to fit them; a coasting wheel that misses them argues for safe_brake=true on the wheels
WHEEL_STOP_MS = {"tx_cut": 500.0, "expiry": 500.0, "disarm": 500.0}
STOP_DISTANCE_MM = {"tx_cut": 100.0, "expiry": 100.0, "disarm": 100.0}
STAND_NOTE = ("over the threshold on the stand: report the cause (coast, safe_brake) first -- the threshold is not "
              "relaxed; the pass is the floor S5 measurement's")
# ② the DISARMs in a commissioning time limit's STOPPING take the general DISARM's thresholds, provisionally until
# the S5 measurements (lead decision 2026-11-12; plan 3.4) -- whatever the DISARM rows hold, these follow
THRESHOLD_KIND = {"limit_disarm": "disarm", "hold_disarm": "disarm"}
HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=16)
# ② the commissioning host: a shorter validity bounds how long a COMMISSION runs on after the window stop
COMMISSION_HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=COMMISSION_CAP_MS, arm_timeout_s=0.3,
                                 zero_valid_ms=COMMISSION_CAP_MS, outbox_max=16)
STOPPING_WAIT_S = 1.0  # ② the cleanup lets a commissioning STOPPING (four-axis brake) run out before its DISARM


class Refused(Exception):
    """A drive command's safety condition (design 3) refused the run before anything moved."""


@dataclass
class Bench:
    """The link, the host core and the record. Every STATE is kept in memory first (a tap on the received bytes) and
    every frame's write completion noted with its decoded message; the JSONL record is written after, and a record
    failure is counted, never raised -- the safety send path does not depend on the disk (Codex impl review P1-2)."""

    link: object  # ReconnectingLink-like: service(host, now_ns) -> events, attribute tx_enabled
    out: object  # a text file
    allow_locked: bool = False
    commission: bool = False  # the commissioning tool: COMMISSION to a commissioning build only
    host: McuLinkHost = None
    samples: list = field(default_factory=list)
    written: dict = field(default_factory=dict)  # seq -> (kind, host_ns, decoded message or None)
    last_queued: dict | None = None
    boots: list = field(default_factory=list)
    record_errors: int = 0
    poll: object = None  # called every cycle (``--markers``: Enter marks an operator event in the record)

    def __post_init__(self) -> None:
        self._tap = LinkDecoder()
        self.host = McuLinkHost(COMMISSION_HOST_CFG if self.commission else HOST_CFG, log=self._log, client=Client.BRINGUP, allow_locked=self.allow_locked,
                                commission=self.commission)

    def _log(self, what, now_ns, data, info) -> None:
        try:
            if what == "received":
                for r in self._tap.feed(data, now_ns):
                    if isinstance(r.message, McuState):
                        self._sample(Sample(r.received_ns, r.message))
            elif what == "queued" and info:
                self.last_queued = dict(info)
            elif what == "written" and info:
                msg = None
                for r in LinkDecoder().feed(bytes(data), now_ns):
                    msg = r.message
                self.written[info["seq"]] = (info["kind"], now_ns, msg)
                self.emit({"t": "written", "host_ns": now_ns, **info})
        except Exception:  # noqa: BLE001 -- the host's receive and send must go on whatever the record does
            self.record_errors += 1

    def _sample(self, s: Sample) -> None:
        st = s.state
        self.samples.append(s)  # in memory first
        if not self.boots or self.boots[-1] != st.boot_id:
            if self.boots:
                # contract 3.3: a new boot mid-session may be a motor-start brown-out even with reset_cause power
                self.emit({"t": "warning", "what": "boot_changed", "host_ns": s.host_ns, "boot_id": st.boot_id,
                           "reset_cause": st.reset_cause.name})
            self.boots.append(st.boot_id)
        self.emit(state_row(s))

    def emit(self, row: dict) -> None:
        try:
            self.out.write(json.dumps(row, default=str) + "\n")
        except (OSError, ValueError):
            self.record_errors += 1

    def cycle(self) -> None:
        now = time.monotonic_ns()
        for e in self.link.service(self.host, now):
            self.emit({"t": "event", "kind": e.kind, "host_ns": now, **{k: str(v) for k, v in e.detail.items()}})
        if self.poll is not None:
            self.poll()

    def run_until(self, pred, timeout_s: float, *, every=None, period_s: float = 0.002) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.cycle()
            if every is not None:
                every()
            if pred():
                return True
            time.sleep(period_s)
        return False

    @property
    def state(self) -> McuState | None:
        return self.samples[-1].state if self.samples else None


KTH_FIELDS = ("kth_frames", "kth_crc_errors", "kth_alt_crc_errors")


def kth_row_problem(row: dict) -> str | None:
    """A recorded STATE row's v3 KTH counters: present, two each, 16-bit unsigned integers."""
    for name in KTH_FIELDS:
        v = row.get(name)
        if not (isinstance(v, list) and len(v) == 2
                and all(isinstance(x, int) and not isinstance(x, bool) and 0 <= x <= 0xFFFF for x in v)):
            return f"{name} missing or malformed in a STATE row (a v3 record is needed)"
    return None


def check_build(state: McuState, *, allow_locked: bool) -> None:
    """Design 3: the commissioning build takes no DRIVE; outputs locked on the real truck is a stop."""
    if state.flags & StateFlag.COMMISSION_BUILD:
        raise Refused("commission_build takes no DRIVE")
    if state.flags & StateFlag.OUTPUTS_LOCKED and not allow_locked:
        raise Refused("outputs_locked: nothing would turn (--allow-locked for a model or HIL)")


def rear(left: float, right: float, steering: float) -> RearDrive:
    return RearDrive(0.0, 0.0, float(left), float(right), float(steering), False)


class Session:
    """Arm to ready, drive with the other axes held, clean up -- the design's section 3."""

    def __init__(self, bench: Bench, cap_rad_s: float, *, floor: bool = False,
                 power_switch_reachable: bool = False) -> None:
        self.b, self.cap = bench, cap_rad_s
        self.floor, self.power_switch_reachable = floor, power_switch_reachable
        self.steer_hold = None
        self.aborted: str | None = None
        self.events_started = False  # an expected STOPPING after a scenario's event is not an abort
        self.arm_requested = False  # the cleanup runs whatever the last STATE says once an ARM may have gone out
        self.cleanup_confirmed: bool | None = None
        self.test_end_ns: int | None = None

    def arm(self, budget_s: float) -> None:
        """Arm to ready; ``budget_s`` fixes the run's end, past which no command stays valid."""
        b = self.b
        if not b.run_until(lambda: b.state is not None and b.state.mode == Mode.DISARMED, 3.0):
            raise Refused("no DISARMED STATE")
        self.preflight(b.state)
        if self.floor:
            # design 3: a floor run needs real outputs, a wired e-stop and the main switch in reach
            if b.allow_locked or b.state.flags & (StateFlag.OUTPUTS_LOCKED | StateFlag.ESTOP_UNWIRED):
                raise Refused("floor: outputs locked or e-stop unwired (or --allow-locked given)")
            if not self.power_switch_reachable:
                raise Refused("floor: confirm the main power switch is reachable (--power-switch-reachable)")
        self.steer_hold = b.state.steering_rad  # the other axes stay where they are (design 3)
        self.test_end_ns = time.monotonic_ns() + int(budget_s * 1e9)
        self.arm_requested = True
        blocked = b.host.request_arm(time.monotonic_ns())
        if blocked is not None:
            raise Refused(f"arm blocked: {blocked.reason}")
        if not b.run_until(lambda: b.host.ready, 2.0):  # arm_pending -> zero_pending -> ready, no retry
            raise Refused("not ready after ARM")

    def preflight(self, state: McuState) -> None:
        check_build(state, allow_locked=self.b.allow_locked)

    def send(self, left: float, right: float, end_ns: int | None = None) -> int | None:
        """One DRIVE expiring at most at the run's fixed end and at ``end_ns``. Returns its seq when queued."""
        for v in (left, right):
            if abs(v) > self.cap + 1e-12:
                raise Refused(f"wheel target {v} over the bring-up cap {self.cap:.3f} rad/s")
        now = time.monotonic_ns()
        expires = min(self.test_end_ns, now + HOST_CFG.cap_ms * 1_000_000)
        if end_ns is not None:
            expires = min(expires, end_ns)
        self.b.last_queued = None
        blocked = self.b.host.drive(rear(left, right, self.steer_hold), 0.0, True, expires, now)
        if blocked is not None or self.b.last_queued is None:
            return None
        return self.b.last_queued["seq"]

    def hold(self, left: float, right: float, seconds: float) -> int | None:
        """Keep sending the same target for ``seconds``. A nonzero command outlives this hold by at most
        ``HANDOVER_S`` (Codex impl P2-4: not by the run's whole remaining budget); a zero one may stay valid to the
        run's end -- the next hold's first command must not find the MCU already stopping."""
        end_ns = time.monotonic_ns() + int(seconds * 1e9)
        nonzero = bool(left or right)
        bound = end_ns if nonzero else None  # a nonzero command never expires after its hold's end
        stop_ns = end_ns - int(HANDOVER_S * 1e9) if nonzero else end_ns  # hand over early, inside the bound
        first = None
        while time.monotonic_ns() < stop_ns:
            seq = self.send(left, right, bound)
            first = first if first is not None else seq
            self.b.cycle()
            if not self.b.host.ready and not self.events_started:
                self.aborted = "authorization_revoked_before_event"
                return first
            time.sleep(0.01)
        return first

    def cleanup(self) -> bool:
        """Design 3: stop refreshing, drop what is queued, send DISARM, and confirm with a STATE received after the
        DISARM was written (a stale cached STATE never confirms, Codex impl P2-3)."""
        b = self.b
        b.link.tx_enabled = True
        b.host.drop_drives("cleanup")
        t0 = time.monotonic_ns()
        b.host.request_disarm(t0)

        def done() -> bool:
            # the MCU's own word that it took this DISARM: a STATE naming its seq (Codex re-review P2-2), not just
            # any STATE received after the write
            sent = {q: t for q, (k, t, _m) in b.written.items() if k == "DISARM" and t >= t0}
            return any(s.state.last_rx_seq in sent and s.host_ns > sent[s.state.last_rx_seq]
                       and s.state.mode in (Mode.DISARMED, Mode.FAULT, Mode.ESTOP) for s in b.samples[-50:])

        ok = b.run_until(done, CLEANUP_S)
        b.emit({"t": "cleanup", "confirmed": bool(ok)})
        self.cleanup_confirmed = bool(ok)
        return ok


def mcu_time_of_acceptance(samples, seq: int):
    """The MCU time of the first STATE that reports ``seq`` accepted (last_rx_seq), and its received host time."""
    for s in samples:
        if s.state.last_rx_seq == seq:
            return s.state.mcu_time_us, s.host_ns
    return None, None


def _final(result: dict, session: Session) -> dict:
    ok = result.get("result") == "ok" and session.aborted is None and session.cleanup_confirmed
    return {**result, "aborted": session.aborted, "cleanup_confirmed": session.cleanup_confirmed,
            "result": "ok" if ok else ("aborted" if result.get("result") in ("ok", None) else result["result"])}


def run_step(bench: Bench, session: Session, wheel: str, target: float) -> dict:
    session.arm(0.5 + STEP_MAX_S + 1.0 + 1.0)
    session.hold(0.0, 0.0, 0.5)
    left, right = (target if wheel in ("left", "both") else 0.0), (target if wheel in ("right", "both") else 0.0)
    w = 0 if wheel != "right" else 1
    initial = bench.state.wheel_rate_left_rad_s if w == 0 else bench.state.wheel_rate_right_rad_s
    seq = session.hold(left, right, STEP_MAX_S) if session.aborted is None else None
    if session.aborted is None:
        session.hold(0.0, 0.0, 1.0)
    session.cleanup()
    step_us, step_host = mcu_time_of_acceptance(bench.samples, seq) if seq is not None else (None, None)
    if step_us is None:
        return _final({"result": "no_acceptance"}, session)
    m = step_metrics(bench.samples, w, step_us, initial, target, window_s=STEP_MAX_S)
    written = bench.written.get(seq, (None, None, None))[1]
    return _final({"result": "ok", "rise_s": m.rise_s, "overshoot_pct": m.overshoot_pct, "settle_s": m.settle_s,
                   "unsettled": m.unsettled, "gaps": m.gaps,
                   "input_delay_host_s": None if written is None else (step_host - written) / 1e9}, session)


def _observe_stop(b: Bench, kind: str, boot: int | None = None, start_index: int | None = None) -> str | None:
    """After the event: only the expected STOPPING -> DISARMED (DISARM: straight to DISARMED) is the scenario; a
    fault, an e-stop, a new boot, a STATE gap or no DISARMED in time is an abort (Codex impl P2-5). ``boot`` and
    ``start_index`` are the boot and the first sample as the event began: everything since is checked, the wait for
    the last DRIVE's acceptance included (Codex re-review P2-3)."""
    boot = b.state.boot_id if boot is None else boot
    end = time.monotonic() + OBSERVE_S
    allowed = {Mode.ARMED, Mode.DISARMED} | ({Mode.STOPPING} if kind != "disarm" else set())
    i0 = start_index if start_index is not None else len(b.samples)
    prev = b.samples[i0 - 1].host_ns if i0 > 0 else None  # the last sample before the event
    for s in b.samples[i0:]:
        if s.state.boot_id != boot:
            return "boot_changed"
        if s.state.mode not in allowed:
            return f"unexpected_mode_{s.state.mode.name.lower()}"
        if prev is not None and s.host_ns - prev > STATE_GAP_S * 1e9:
            return "state_gap"  # a gap in the acceptance wait is an observation failure too (Codex re-review 3)
        prev = s.host_ns
    last = b.samples[-1].host_ns
    while time.monotonic() < end:
        b.cycle()
        st = b.state
        if st.boot_id != boot:
            return "boot_changed"
        if st.mode not in allowed:
            return f"unexpected_mode_{st.mode.name.lower()}"
        if b.samples[-1].host_ns - last > STATE_GAP_S * 1e9 or time.monotonic_ns() - b.samples[-1].host_ns \
                > STATE_GAP_S * 1e9:
            return "state_gap"
        last = b.samples[-1].host_ns
        if st.mode == Mode.DISARMED:
            return None
        time.sleep(0.002)
    return "no_disarm"


def run_scenario(bench: Bench, session: Session, kind: str, rate: float) -> dict:
    b = bench
    if not rate > 0:
        raise Refused("the scenario rolls forward: --rate must be positive")  # Codex re-review P2-4
    session.arm(0.3 + STEP_MAX_S + 1.0)
    session.hold(0.0, 0.0, 0.3)
    rolling = session.aborted is None and b.run_until(
        lambda: b.state.wheel_rate_left_rad_s > 0.5 * rate and b.state.wheel_rate_right_rad_s > 0.5 * rate, 2.0,
        every=lambda: session.send(rate, rate), period_s=0.01)
    out = {"kind": kind, "rolling_confirmed": bool(rolling), "worst_case": None}
    if not rolling:
        session.aborted = session.aborted or "not_rolling"
        session.cleanup()
        return _final({**out, "result": "aborted"}, session)
    session.events_started = True
    event_ns = None
    boot0, i_event = b.state.boot_id, len(b.samples)  # the boot and the first sample as the event begins
    if kind == "disarm":
        t0 = time.monotonic_ns()
        b.host.request_disarm(t0)
        b.run_until(lambda: any(k == "DISARM" and t >= t0 for k, t, _m in b.written.values()), 0.5)
        event_ns = min((t for k, t, _m in b.written.values() if k == "DISARM" and t >= t0), default=None)
    else:
        # the last DRIVE: on a STATE just received (the largest validity the cap allows), then nothing more
        n0 = len(b.samples)
        fresh_ok = b.run_until(lambda: len(b.samples) > n0, 0.2)
        fresh = b.state
        seq = session.send(rate, rate)
        write_ok = seq is not None and b.run_until(lambda: seq in b.written, 0.2)
        if not write_ok:
            # never cut the TX with a frame half out: an aborted run, cleaned up (Codex impl P2-6)
            out["worst_case"] = {"met": False, "why": "last_drive_not_written"}
            session.aborted = "last_drive_not_written"
            session.cleanup()
            return _final({**out, "result": "aborted", "worst_case_unmet": True}, session)
        if kind == "tx_cut":
            b.link.tx_enabled = False  # at once after the write completes; STATE still read
        kind_, event_ns, msg = b.written[seq]
        accepted = b.run_until(lambda: b.state.last_rx_seq == seq, 0.3)
        if kind == "tx_cut":
            valid = msg.valid_after_echo_ms if isinstance(msg, Drive) else None
            echo = msg.echo_mcu_time_us if isinstance(msg, Drive) else None
            met = bool(fresh_ok and accepted and valid == HOST_CFG.cap_ms and echo == fresh.mcu_time_us)
            out["worst_case"] = {"seq": seq, "write_done_host_ns": event_ns, "accepted": bool(accepted),
                                 "valid_after_echo_ms": valid, "echo_mcu_time_us": echo, "fresh_state": fresh_ok,
                                 "cap_ms": HOST_CFG.cap_ms, "met": met}
            if not met:
                out["worst_case_unmet"] = True
    i0 = len(b.samples)
    why = _observe_stop(b, kind, boot0, i_event)
    if why is not None:
        session.aborted = why
    else:
        b.run_until(lambda: False, 0.6)  # keep observing the counts after DISARMED
    tail = b.samples[i0:]
    mode_change = next((s for s in tail if s.state.mode != Mode.ARMED), None)
    duty_zero = next((s for s in tail if not any(s.state.duty_e4)), None)
    settle = count_settle(b.samples, i0)
    session.cleanup()
    rel = (lambda ns: None if ns is None or event_ns is None else (ns - event_ns) / 1e6)
    return _final({**out, "result": "ok", "event_host_ns": event_ns,
                   "software": {"mode_change_ms": rel(mode_change.host_ns if mode_change else None),
                                "duty_zero_ms": rel(duty_zero.host_ns if duty_zero else None),
                                "count_settle_ms": rel(settle.host_ns), "count_settle_reason": settle.reason},
                   "note": "software evidence only: the S5 verdict takes the external measurements"}, session)


# ---------------------------------------------------------------- ② commissioning (S3)
_AXIS_ENCODER_BITS = {0: FaultFlag.ENCODER_STUCK_L | FaultFlag.ENCODER_POLARITY_L,
                      1: FaultFlag.ENCODER_STUCK_R | FaultFlag.ENCODER_POLARITY_R}
PRE_ZERO_S = 0.3
POST_ZERO_S = 0.5


class CommissionSession(Session):
    """Design ② 3: a commissioning build only, the picked axis's open-loop duty, early stops, the tool ① cleanup."""

    def __init__(self, bench: Bench, axis: int, window: tuple[float, float] | None) -> None:
        super().__init__(bench, 0.0)
        self.axis, self.window = axis, window
        self.sent: list[tuple[int, int]] = []  # (seq, duty_e4) of every COMMISSION queued by the tool
        self.stop_reason: str | None = None
        self.stopping_waited_ms: float | None = None

    def preflight(self, state: McuState) -> None:
        if not commission_allowed(state.flags):
            raise Refused("not a commissioning build (STATE.flags.commission_build off): nothing is sent")
        if state.flags & StateFlag.OUTPUTS_LOCKED and not self.b.allow_locked:
            raise Refused("outputs_locked: nothing would turn (--allow-locked for a model)")
        if self.axis >= 2:
            v = state.steering_rad if self.axis == 2 else state.lift_m
            lo, hi = self.window
            if not (v == v and lo < v < hi):
                raise Refused(f"window: the axis value {v} is not inside ({lo}, {hi})")

    def send(self, left, right, end_ns=None):  # noqa: ARG002 -- a commissioning build takes no DRIVE
        raise Refused("the commissioning tool sends no DRIVE")

    def send_duty(self, duty_e4: int, end_ns: int | None = None) -> int | None:
        if abs(duty_e4) > COMMISSION_DUTY_MAX * 10000:
            raise Refused(f"duty {duty_e4 / 1e4} over the tool limit {COMMISSION_DUTY_MAX}")
        now = time.monotonic_ns()
        expires = min(self.test_end_ns, now + self.b.host.cfg.cap_ms * 1_000_000)
        if end_ns is not None:
            expires = min(expires, end_ns)
        self.b.last_queued = None
        blocked = self.b.host.commission(self.axis, int(duty_e4), expires, now)
        if blocked is not None or self.b.last_queued is None:
            return None
        seq = self.b.last_queued["seq"]
        self.sent.append((seq, int(duty_e4)))
        return seq

    def hold_duty(self, duty_e4: int, seconds: float, stop=None) -> int | None:
        """``hold`` for one axis's duty: the same bound and hand-over; ``stop()`` naming a reason ends a nonzero
        hold at once (the caller then holds 0)."""
        end_ns = time.monotonic_ns() + int(seconds * 1e9)
        nonzero = duty_e4 != 0
        bound = end_ns if nonzero else None
        stop_ns = end_ns - int(HANDOVER_S * 1e9) if nonzero else end_ns
        first = None
        if stop is not None:
            # the latest samples are judged before the first nonzero command goes out (Codex re-review P2-4)
            self.b.cycle()
            if (reason := stop()) is not None:
                self.stop_reason = reason
                return first
        while time.monotonic_ns() < stop_ns:
            seq = self.send_duty(duty_e4, bound)
            first = first if first is not None else seq
            self.b.cycle()
            why = self._abort_reason()
            if why is not None:
                self.aborted = why
                return first
            if stop is not None and (reason := stop()) is not None:
                self.stop_reason = reason
                return first
            time.sleep(0.01)
        return first

    def cleanup(self) -> bool:
        """Design ② 3: the tool ① cleanup, but never a DISARM into a commissioning STOPPING -- the firmware ends it
        on DISARM and its four-axis electric brake with it (Codex design P1-1, against contract 3.4). Queued
        commands are dropped and the STOPPING is let run out first (bounded by ``STOPPING_WAIT_S``)."""
        b = self.b
        b.host.drop_drives("cleanup")
        # the host's own freshness bound (50 ms), not the observation gap (Codex re-review 3 P2)
        stale = not b.samples or time.monotonic_ns() - b.samples[-1].host_ns > b.host.cfg.state_max_age_s * 1e9
        if stale or b.state.mode == Mode.STOPPING:
            # the MCU may be STOPPING unseen (the last STATE is old) or seen: wait for a fresh STATE out of it.
            # Not seen within STOPPING_WAIT_S (or the I/O failing): the DISARM still goes out -- a stop command is
            # never withheld (contract 3.4) -- and the run records that the brake's end was not observed
            # (Codex re-review P2-1, P2-5)
            t0, n0 = time.monotonic(), len(b.samples)
            try:
                seen = b.run_until(lambda: len(b.samples) > n0 and b.state.mode != Mode.STOPPING, STOPPING_WAIT_S)
            except Exception as exc:  # noqa: BLE001 -- the DISARM below must still be tried
                seen = False
                b.emit({"t": "cleanup_error", "where": "stopping_wait", "error": f"{type(exc).__name__}: {exc}"})
            self.stopping_waited_ms = (time.monotonic() - t0) * 1000
            b.emit({"t": "stopping_waited", "ms": self.stopping_waited_ms, "observed_end": seen})
            if not seen:
                self.aborted = self.aborted or "stopping_end_unobserved"
        return super().cleanup()

    def early_stop(self, state: McuState, at_start: FaultFlag) -> str | None:
        """Why a nonzero hold ends now (design ② 3): a stuck/polarity bit of the driven wheel newly on (latched
        while ARMED in a commissioning build), or the steering/lift value out of the window."""
        if self.axis < 2:
            return "encoder_bit" if (state.fault_flags & _AXIS_ENCODER_BITS[self.axis]) & ~at_start else None
        v = state.steering_rad if self.axis == 2 else state.lift_m
        lo, hi = self.window
        return None if v == v and lo <= v <= hi else "window"

    def _abort_reason(self) -> str | None:
        b = self.b
        if not b.host.ready:
            return "authorization_revoked"
        if b.state.reject_reason != RejectReason.NONE:
            return f"rejected_{b.state.reject_reason.name.lower()}"
        return None


def session_abort_reason(samples: list, i_arm: int) -> str | None:
    """A commissioning session's samples from the arm on (the cleanup included): a fault or e-stop latch, or a new
    boot, is an abort whatever the dynamics' window says. STOPPING -> DISARMED (time limit, expiry) is judged by the
    authorization the run lost, not here."""
    if i_arm >= len(samples):
        return None
    boot = samples[max(i_arm - 1, 0)].state.boot_id
    for smp in samples[i_arm:]:
        if smp.state.boot_id != boot:
            return "boot_changed"
        if smp.state.mode in (Mode.FAULT, Mode.ESTOP):
            return f"mode_{smp.state.mode.name.lower()}"
    return None


def run_commission(bench: Bench, session: CommissionSession, duty: float, seconds: float) -> dict:
    b, axis = bench, session.axis
    duty_e4 = int(round(duty * 10000))
    session.arm(PRE_ZERO_S + seconds + POST_ZERO_S + 1.0)
    i_arm = len(b.samples)
    at_start = b.state.fault_flags & _AXIS_ENCODER_BITS.get(axis, FaultFlag(0))

    seen = {"i": len(b.samples) - 1}

    def early_stop() -> str | None:
        # every sample since the last look: an excursion and its return inside one read is not missed
        for smp in b.samples[max(seen["i"], 0):]:
            if (reason := session.early_stop(smp.state, at_start)) is not None:
                return reason
        seen["i"] = len(b.samples)
        return None

    session.hold_duty(0, PRE_ZERO_S)
    n_zero = len(session.sent)
    if session.aborted is None:
        session.hold_duty(duty_e4, seconds, early_stop)
        if session.stop_reason is None and session.aborted is None:
            session.stop_reason = "time"
    n_on = len(session.sent)
    if session.aborted is None:
        session.hold_duty(0, POST_ZERO_S)
    session.cleanup()
    on_seqs = {q for q, d in session.sent[n_zero:n_on] if d != 0}
    off_seqs = {q for q, _d in session.sent[n_on:]}
    on_index = next((i for i in range(i_arm, len(b.samples)) if b.samples[i].state.last_rx_seq in on_seqs), None)
    # the driven span ends at the first sample accepting a later zero, or at the MCU's own stop (it left ARMED:
    # time limit, expiry, fault) -- never at the record's end (Codex re-review P2-3)
    off_index = None if on_index is None else next(
        (i for i in range(on_index, len(b.samples))
         if b.samples[i].state.last_rx_seq in off_seqs or b.samples[i].state.mode != Mode.ARMED), None)
    # the motion window ends with the post-zero hold, before the cleanup's DISARM
    end = len(b.samples)
    disarm_seqs = {q for q, (k, _t, _m) in b.written.items() if k == "DISARM"}
    end = next((i for i in range(i_arm, end) if b.samples[i].state.last_rx_seq in disarm_seqs), end)
    if off_index is not None and off_index >= end:
        end = off_index + 1  # the sample that closes the driven span stays, even when it confirms the DISARM
    metrics = commission_metrics(b.samples[:end], axis=axis, duty_e4=duty_e4, on_index=on_index,
                                 off_index=off_index)
    # the whole session, the cleanup included, apart from the dynamics' cut (Codex re-review P2-2)
    why = session_abort_reason(b.samples, i_arm)
    if why is not None:
        session.aborted = session.aborted or why
    faults = FaultFlag(0)
    for smp in b.samples[i_arm:]:
        faults |= smp.state.fault_flags
    result = "no_acceptance" if on_index is None else ("ok" if metrics.valid else "invalid_measurement")
    return _final({"result": result, "metrics": asdict(metrics), "stop": session.stop_reason,
                   "faults_seen": [f.name for f in FaultFlag if f in faults],
                   "stopping_waited_ms": session.stopping_waited_ms,
                   "build_flags": [f.name for f in StateFlag if f in b.state.flags],
                   "note": "software evidence only: the direction is the operator's, currents against the bench supply"},
                  session)


COMMISSION_SCENARIOS = ("limit_disarm", "hold_disarm")
LIMIT_MAX_S = 3.0  # the scenario drives a wheel up to the build's time limit (proposed 2 s), never past this
LIMIT_SLACK_S = 0.3  # how long past --limit-s the scenario waits for the MCU's limit before giving up


def disarm_acceptance(samples: list, limit_i: int, seq: int, grace_ms: float) -> dict:
    """Design ② 10: where the scenario's DISARM (``seq``) was accepted, and whether that was inside the limit's
    STOPPING -- proved, not guessed from the sample before (Codex scenario review P2-1). The STOPPING began after the
    last ARMED sample before the limit sample, so it cannot end on its own before that sample's MCU time plus the
    build's ``stop_grace_ms``; an acceptance reported by a STATE earlier than that bound, after a STOPPING sample,
    was inside it. A DISARMED sample before the acceptance means it was not; anything else stays unproven (None)."""
    acc = next((i for i in range(limit_i, len(samples)) if samples[i].state.last_rx_seq == seq), None)
    out = {"disarm_accepted": acc is not None, "mode_before_acceptance": None, "mode_at_acceptance": None,
           "disarm_during_stopping": None, "natural_end_bound_mcu_us": None}
    if acc is None:
        return out
    prev = samples[acc - 1].state
    out["mode_before_acceptance"], out["mode_at_acceptance"] = prev.mode.name, samples[acc].state.mode.name
    armed = next((samples[i].state for i in range(limit_i - 1, -1, -1) if samples[i].state.mode == Mode.ARMED), None)
    if armed is None:
        return out
    bound = armed.mcu_time_us + int(grace_ms * 1000)
    out["natural_end_bound_mcu_us"] = bound
    if any(samples[i].state.mode == Mode.DISARMED for i in range(limit_i, acc)):
        out["disarm_during_stopping"] = False  # the STOPPING was seen over before the DISARM landed
    elif prev.mode == Mode.STOPPING and samples[acc].state.mcu_time_us < bound:
        out["disarm_during_stopping"] = True
    return out


def run_commission_scenario(bench: Bench, session: CommissionSession, kind: str, duty: float, limit_s: float,
                            delay_ms: float, grace_ms: float) -> dict:
    """Design ② 10 (lead decision 2026-11-12): drive one wheel until the MCU's own commissioning time limit, then a
    DISARM at once (``limit_disarm``) or ``delay_ms`` into the limit's STOPPING (``hold_disarm``). The record is the
    command order, the DISARM's acceptance (its seq in a STATE) and every STATE; the stop time and distance pass only
    on external measurements."""
    b, axis = bench, session.axis
    duty_e4 = int(round(duty * 10000))
    session.arm(PRE_ZERO_S + limit_s + LIMIT_SLACK_S + 2.0)
    i_arm = len(b.samples)
    session.hold_duty(0, PRE_ZERO_S)
    out = {"kind": kind, "limit_reached": False, "event_host_ns": None, "disarm_seq": None,
           "disarm_accepted": False, "mode_before_acceptance": None, "disarm_during_stopping": None}
    if session.aborted is not None:
        session.cleanup()
        return _final({**out, "result": "aborted"}, session)
    # drive on until the MCU's limit: STOPPING with commission_time_limit, judged over every new sample
    limit_i, on_i, scanned = None, None, len(b.samples)
    at_start = b.state.fault_flags & _AXIS_ENCODER_BITS[axis]
    give_up = time.monotonic() + limit_s + LIMIT_SLACK_S
    on_seqs = set()
    while time.monotonic() < give_up:
        seq = session.send_duty(duty_e4)
        if seq is not None:
            on_seqs.add(seq)
        b.cycle()
        for i in range(scanned, len(b.samples)):
            st = b.samples[i].state
            if on_i is None and st.last_rx_seq in on_seqs:
                on_i = i
            if st.mode == Mode.STOPPING and st.fault_flags & FaultFlag.COMMISSION_TIME_LIMIT:
                limit_i = i
                break
            if session.early_stop(st, at_start) is not None:
                # the driven wheel's new stuck/polarity bit still stops it here: only the 1.5 s bound is lifted
                # (Codex scenario review P2-3)
                session.stop_reason = "encoder_bit"
                session.aborted = "encoder_bit_before_limit"
                break
        scanned = len(b.samples)
        if limit_i is not None or session.aborted is not None:
            break
        why = session._abort_reason()
        if why is not None:
            session.aborted = why
            break
        time.sleep(0.01)
    if limit_i is None:
        session.aborted = session.aborted or "limit_not_reached"
        if session.aborted == "encoder_bit_before_limit":
            session.hold_duty(0, POST_ZERO_S)  # the picked wheel at 0 (electric brake) as in a normal run
        session.cleanup()
        return _final({**out, "result": "aborted"}, session)
    out["limit_reached"] = True
    out["limit_after_ms"] = None if on_i is None else (
        b.samples[limit_i].state.mcu_time_us - b.samples[on_i].state.mcu_time_us) / 1000
    session.events_started = True
    if kind == "hold_disarm":
        # still in the limit's STOPPING after the delay, or the scenario's point is lost
        b.run_until(lambda: False, delay_ms / 1000)
        if b.state.mode != Mode.STOPPING:
            out["hold_ended_before_disarm"] = True
    t0 = time.monotonic_ns()
    b.host.request_disarm(t0)
    b.run_until(lambda: any(k == "DISARM" and t >= t0 for k, t, _m in b.written.values()), 0.5)
    sent = sorted((t, q) for q, (k, t, _m) in b.written.items() if k == "DISARM" and t >= t0)
    if sent:
        out["event_host_ns"], out["disarm_seq"] = sent[0]
        q = out["disarm_seq"]
        b.run_until(lambda: any(s.state.last_rx_seq == q for s in b.samples[limit_i:]), 0.5)
    b.run_until(lambda: False, 0.6)  # keep observing the counts after DISARMED
    if out["disarm_seq"] is not None:
        out.update(disarm_acceptance(b.samples, limit_i, out["disarm_seq"], grace_ms))
    i0 = limit_i
    settle = count_settle(b.samples, i0)
    disarmed = next((s for s in b.samples[i0:] if s.state.mode == Mode.DISARMED), None)
    # both scenarios are about a DISARM inside the STOPPING: proved, or not met
    met = bool(out["disarm_accepted"] and out["disarm_during_stopping"] is True)
    session.cleanup()
    why = session_abort_reason(b.samples, i_arm)  # after the cleanup: its samples count (Codex scenario P2-2)
    if why is not None:
        session.aborted = session.aborted or why
    ev = out["event_host_ns"]
    rel = (lambda ns: None if ns is None or ev is None else (ns - ev) / 1e6)
    return _final({**out, "result": "ok" if met else "not_met",
                   "software": {"limit_seen_to_disarm_write_ms": None if ev is None
                                else (ev - b.samples[limit_i].host_ns) / 1e6,
                                "disarmed_ms": rel(disarmed.host_ns if disarmed else None),
                                "count_settle_ms": rel(settle.host_ns), "count_settle_reason": settle.reason},
                   "note": "software evidence only: stop time and distance pass on external measurements"}, session)


def external_value(name: str, raw) -> float | str | None:
    """An external measurement as given: None, 'unavailable', or a finite value >= 0 -- anything else is refused
    (Codex review: a negative or non-finite value must never pass)."""
    if raw is None or raw == "unavailable":
        return raw
    try:
        v = float(raw)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: {raw!r} is neither a number nor 'unavailable'") from None
    if not (v == v and v >= 0 and v != float("inf")):
        raise ValueError(f"{name}: {raw!r} must be finite and >= 0")
    return v


def verdict(kind: str, external: dict, *, floor: bool = False) -> dict:
    """Design 4: external measurements against the plan 3.4 thresholds. Every value given is kept, with or without
    a threshold (Codex impl P2-10); 'unavailable' stays so. Off the floor (wheels lifted on the stand, the default)
    a wheel's stop time and distance have no body inertia or floor drag: they are shown against the threshold as a
    reference only, the pass left to the floor S5 measurement (lead decision 2026-11-12); the output response is
    judged either way."""
    out = {}
    for name, table in (("output_response_ms", OUTPUT_RESPONSE_MS), ("stop_time_ms", WHEEL_STOP_MS),
                        ("stop_distance_mm", STOP_DISTANCE_MM)):
        value, limit = external_value(name, external.get(name)), table.get(THRESHOLD_KIND.get(kind, kind))
        if value is None and limit is None:
            continue
        if value is None or value == "unavailable":
            out[name] = {"value": "unavailable", "limit": limit, "pass": None}
        else:
            v = float(value)
            if name != "output_response_ms" and not floor:
                over = None if limit is None else v > limit
                out[name] = {"value": v, "limit": limit, "pass": None, "reference": "stand", "over_limit": over}
                if over:
                    out[name]["note"] = STAND_NOTE
            else:
                out[name] = {"value": v, "limit": limit, "pass": None if limit is None else v <= limit}
    return out


def analyze(record: Path, marker: int, kind: str, external: dict, *, floor: bool = False) -> dict:
    """A recorded run around one operator marker (IWDG, e-stop, main power): the software timeline from the marker's
    host time -- the last STATE before a gap or the record's end, the first mode change, the first all-zero duty,
    a new boot and its reset cause, the count settle -- and the external verdict."""
    rows = [json.loads(x) for x in record.read_text(encoding="utf-8").splitlines() if x.strip()]
    markers = [r for r in rows if r.get("t") == "marker"]
    if not markers:
        return {"result": "no_marker"}
    if not -len(markers) <= marker < len(markers):
        return {"result": "no_marker", "markers": len(markers), "asked": marker}
    m_ns = int(markers[marker]["host_ns"])
    samples = [sample_from_row(r) for r in rows if r.get("t") == "state"]
    after = [s for s in samples if s.host_ns >= m_ns]
    before = [s for s in samples if s.host_ns < m_ns]
    ref = before[-1] if before else (after[0] if after else None)  # the state as the event began
    rel = (lambda s: None if s is None else (s.host_ns - m_ns) / 1e6)
    first_mode = next((s for s in after if ref is not None and s.state.mode != ref.state.mode), None)
    new_boot = next((s for s in after if ref is not None and s.state.boot_id != ref.state.boot_id), None)
    gaps = [(a.host_ns, b.host_ns) for a, b in zip(after, after[1:]) if b.host_ns - a.host_ns > STATE_GAP_S * 1e9]
    i0 = samples.index(after[0]) if after else len(samples)
    settle = count_settle(samples, i0) if after else None
    return {"result": "ok", "marker": markers[marker].get("label"), "kind": kind,
            "software": {"mode_change_ms": rel(first_mode),
                         "duty_zero_ms": rel(next((s for s in after if not any(s.state.duty_e4)), None)),
                         "new_boot_ms": rel(new_boot),
                         "new_boot_reset_cause": None if new_boot is None else new_boot.state.reset_cause.name,
                         "last_state_before_gap_ms": None if not gaps else (gaps[0][0] - m_ns) / 1e6,
                         "last_state_ms": rel(after[-1]) if after else None,
                         "count_settle_ms": None if settle is None or settle.host_ns is None
                         else (settle.host_ns - m_ns) / 1e6},
            "external": verdict(kind, external, floor=floor),
            "note": "software evidence only: the S5 verdict takes the external measurements"}


def _cap() -> float:
    cfg = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
    return wheel_rate_cap(cfg.wheel_rate_limit_rad_s, None)  # the radius stays unmeasured until S2


def _confirm(args, what: str) -> None:
    if args.yes:
        return
    print(what + "\nWheels lifted on the stand (or --floor prerequisites met)? Type 'yes': ", end="", flush=True)
    if sys.stdin.readline().strip() != "yes":
        raise Refused("operator did not confirm")


def _marker_reader(bench: Bench, fd: int | None = None):
    """Enter on the terminal marks an operator event (design 2), through the shared reader
    (``forklift_core.plant.markers``: never blocks the drive loop, whole lines only, a bounded amount per poll)."""
    return marker_reader(lambda label, now: bench.emit({"t": "marker", "host_ns": now, "label": label}), fd).poll


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("record", "cpr", "step", "scenario"):
        p = sub.add_parser(name)
        p.add_argument("--device", required=True)
        p.add_argument("--baud", type=int, choices=sorted(_BAUDS))
        p.add_argument("--out", required=True)
        p.add_argument("--allow-locked", action="store_true")
        p.add_argument("--yes", action="store_true")
    r = sub.choices["record"]
    r.add_argument("--seconds", type=float, default=30.0)
    r.add_argument("--markers", action="store_true")
    c = sub.choices["cpr"]
    c.add_argument("--seconds", type=float, default=60.0)
    c.add_argument("--turns-left", type=float, default=0.0)
    c.add_argument("--turns-right", type=float, default=0.0)
    c.add_argument("--dir-left", type=int, default=1, choices=(1, -1))
    c.add_argument("--dir-right", type=int, default=1, choices=(1, -1))
    c.add_argument("--motor-counts-confirmed", action="store_true")
    s = sub.choices["step"]
    s.add_argument("--wheel", choices=("left", "right", "both"), required=True)
    s.add_argument("--target", type=float, required=True)
    sc = sub.choices["scenario"]
    sc.add_argument("--kind", choices=("tx_cut", "disarm", "expiry"), required=True)
    sc.add_argument("--rate", type=float, default=0.5)
    sc.add_argument("--floor", action="store_true")
    sc.add_argument("--power-switch-reachable", action="store_true")
    cm = sub.add_parser("commission")
    cm.add_argument("--device", required=True)
    cm.add_argument("--baud", type=int, choices=sorted(_BAUDS))
    cm.add_argument("--out", required=True)
    cm.add_argument("--allow-locked", action="store_true")
    cm.add_argument("--yes", action="store_true")
    cm.add_argument("--axis", choices=tuple(AXES), required=True)
    cm.add_argument("--duty", type=float, required=True)
    cm.add_argument("--seconds", type=float)
    cm.add_argument("--scenario", choices=COMMISSION_SCENARIOS)
    cm.add_argument("--limit-s", type=float, default=2.0)
    cm.add_argument("--disarm-delay-ms", type=float, default=50.0)
    cm.add_argument("--stop-grace-ms", type=float)
    cm.add_argument("--window", type=float, nargs=2, metavar=("LO", "HI"))
    cm.add_argument("--observed", choices=("pos", "neg", "none"))
    cm.add_argument("--power-switch-in-hand", action="store_true")
    cm.add_argument("--markers", action="store_true",
                    help="Enter marks an operator event (main power cut, NRST) while the wheel turns, in this record")
    kt = sub.add_parser("kth", help="S2: the KTH CRC variant from a record's v3 counters (mcu-link/3 contract 3.3)")
    kt.add_argument("--record", required=True)
    for ax in ("m", "h"):
        kt.add_argument(f"--configured-{ax}", required=True, choices=KTH_VARIANTS,
                        help=f"the CRC variant axis {ax.upper()} is configured with in the image under test")
    kt.add_argument("--image", required=True, help="the firmware image under test (its identity string or hash)")
    ck = sub.add_parser("clock", help="G7: the MCU-to-host clock fit of a record (teleop record design 5)")
    ck.add_argument("--record", required=True)
    an = sub.add_parser("analyze")
    an.add_argument("--record", required=True)
    an.add_argument("--marker", type=int, default=-1,
                    help="which marker, counted from 0: 0 = the first, 1 = the second, -1 = the last (default)")
    an.add_argument("--floor", action="store_true", help="the marker's run was on the floor (else the stand)")
    an.add_argument("--kind", choices=("tx_cut", "disarm", "expiry", "iwdg", "power") + COMMISSION_SCENARIOS,
                    required=True)
    for p in (sc, an, cm):
        for name in ("output-response-ms", "stop-time-ms", "stop-distance-mm"):
            p.add_argument("--" + name)
    args = ap.parse_args(argv)
    if args.cmd == "kth":
        rows = [json.loads(x) for x in Path(args.record).read_text(encoding="utf-8").splitlines() if x.strip()]
        states = [r for r in rows if r.get("t") == "state"]
        bad = next((why for r in states if (why := kth_row_problem(r)) is not None), None)
        if bad is not None:  # never the dataclass defaults (Codex v3 impl P2-2: a missing error count read as zeros)
            print(json.dumps({"result": "invalid", "why": bad, "configured": {"M": args.configured_m,
                                                                                "H": args.configured_h},
                              "image": args.image}))
            return 0
        samples = [sample_from_row(r) for r in states]
        print(json.dumps(kth_check(samples, configured={"M": args.configured_m, "H": args.configured_h},
                                   image=args.image)))
        return 0
    if args.cmd == "clock":
        rows = [json.loads(x) for x in Path(args.record).read_text(encoding="utf-8").splitlines() if x.strip()]
        fits = clock_fits_from_rows(rows)
        print(json.dumps({"result": "ok" if fits else "no_state_rows", "fits": [f.summary() for f in fits]}))
        return 0
    if args.cmd == "analyze":
        try:
            for name in ("output_response_ms", "stop_time_ms", "stop_distance_mm"):
                external_value(name, getattr(args, name))
        except ValueError as exc:
            print(json.dumps({"result": "refused", "why": str(exc)}))
            return 2
        print(json.dumps(analyze(Path(args.record), args.marker, args.kind, {
            "output_response_ms": args.output_response_ms, "stop_time_ms": args.stop_time_ms,
            "stop_distance_mm": args.stop_distance_mm}, floor=args.floor), default=str))
        return 0
    link = ReconnectingLink(args.device, baud=args.baud)
    with open(args.out, "a", encoding="utf-8") as out:
        bench = Bench(link, out, allow_locked=args.allow_locked, commission=args.cmd == "commission")
        if args.cmd in ("record", "cpr"):
            bench.run_until(lambda: False, args.seconds,
                            every=_marker_reader(bench) if args.cmd == "record" and args.markers else None)
            if args.cmd == "cpr":
                res = wheel_cpr(bench.samples, (args.turns_left, args.turns_right), (args.dir_left, args.dir_right),
                                motor_counts_confirmed=args.motor_counts_confirmed)
                print(json.dumps(asdict(res)))
            return 0
        result = execute(args, bench)
        print(json.dumps(result, default=str))
    return 0


def _check_commission_args(args) -> None:
    """Design ② 2 and 10: the tool's own limits, before anything is sent."""
    if not 0 < abs(args.duty) <= COMMISSION_DUTY_MAX:
        raise Refused(f"duty must be nonzero and within +-{COMMISSION_DUTY_MAX}")
    if getattr(args, "scenario", None) is not None:
        if AXES[args.axis] >= 2:
            raise Refused("scenario: a wheel only (steering and lift would run into their mechanical limits)")
        if not COMMISSION_HOLD_MAX_S < args.limit_s <= LIMIT_MAX_S:
            raise Refused(f"limit-s: the build's time limit, in ({COMMISSION_HOLD_MAX_S}, {LIMIT_MAX_S}]")
        if args.seconds is not None:
            raise Refused("seconds: a scenario runs to the build's limit -- give --limit-s, not --seconds")
        if args.stop_grace_ms is None or not 0 < args.stop_grace_ms <= 1000:
            raise Refused("stop-grace-ms: the build's stop_grace_ms is needed to prove the DISARM was in the STOPPING")
        if not 0 <= args.disarm_delay_ms < args.stop_grace_ms:
            raise Refused("disarm-delay-ms: the delay must be in [0, stop_grace_ms)")
        if args.window is not None:
            raise Refused("window: a wheel takes no window")
        if not args.power_switch_in_hand:
            raise Refused("power switch: confirm a person holds the main power switch (--power-switch-in-hand)")
        return
    if args.seconds is None:
        raise Refused("seconds: required without --scenario")
    hold_max = AXIS_HOLD_MAX_S.get(AXES[args.axis], COMMISSION_HOLD_MAX_S)
    if not COMMISSION_HOLD_MIN_S <= args.seconds <= hold_max:
        raise Refused(f"seconds must be in [{COMMISSION_HOLD_MIN_S}, {hold_max}] for axis {args.axis}")
    if not args.power_switch_in_hand:
        # the commissioning build arms without an e-stop pin: a person holds the main power switch (firmware design
        # 2.1, Q3); --yes does not stand in for it (Codex design P2-5)
        raise Refused("power switch: confirm a person holds the main power switch (--power-switch-in-hand)")
    if AXES[args.axis] >= 2:
        if args.window is None or not args.window[0] < args.window[1]:
            raise Refused("window: steering and lift need --window LO HI with LO < HI")
    elif args.window is not None:
        raise Refused("window: a wheel takes no window")


def _observed(args) -> str:
    if args.observed is not None:
        return args.observed
    if args.yes:
        return "unknown"
    print("Physical direction seen (pos = wheel rolling the truck forward / steering left / lift up; neg; none): ",
          end="", flush=True)
    answer = sys.stdin.readline().strip()
    return answer if answer in ("pos", "neg", "none") else "unknown"


def execute(args, bench: Bench) -> dict:
    """A drive command (step, scenario or commission) on a bench, with the cleanup guaranteed once an ARM may have
    gone out."""
    if args.cmd == "commission":
        session = CommissionSession(bench, AXES[args.axis], None if args.window is None else tuple(args.window))
    else:
        session = Session(bench, _cap(), floor=getattr(args, "floor", False),
                          power_switch_reachable=getattr(args, "power_switch_reachable", False))
    result: dict = {"result": "aborted", "why": "not run"}
    try:
        for name in ("output_response_ms", "stop_time_ms", "stop_distance_mm"):
            try:
                external_value(name, getattr(args, name, None))  # before anything is armed
            except ValueError as exc:
                raise Refused(str(exc)) from None
        if args.cmd == "commission":
            _check_commission_args(args)
            if getattr(args, "scenario", None) is not None:
                what = (f"commission scenario {args.scenario}: will ARM and drive wheel {args.axis} at duty {args.duty} "
                        f"up to the build's time limit ({args.limit_s} s, giving up at {args.limit_s + LIMIT_SLACK_S} s),"
                        f" then DISARM {'at once' if args.scenario == 'limit_disarm' else f'{args.disarm_delay_ms} ms in'}"
                        f" (build stop_grace_ms {args.stop_grace_ms}).")
            else:
                what = (f"commission: will ARM and drive axis {args.axis} at duty {args.duty} for at most "
                        f"{args.seconds} s (window {args.window}).")
            _confirm(args, what + " Outputs on only after the IPROPI protection is confirmed (user decision); "
                                  "the window inside the mechanical limits.")
            if getattr(args, "markers", False):
                # after the confirmation: the reader reads stdin and would take the operator's 'yes' line
                bench.poll = _marker_reader(bench)
                print("markers: press Enter (optionally a label first) at each event -- the main power cut, an NRST "
                      "reset -- while the wheel turns", file=sys.stderr, flush=True)
            if getattr(args, "scenario", None) is not None:
                result = run_commission_scenario(bench, session, args.scenario, args.duty, args.limit_s,
                                                 args.disarm_delay_ms, args.stop_grace_ms)
                result["external"] = verdict(args.scenario, {
                    "output_response_ms": args.output_response_ms, "stop_time_ms": args.stop_time_ms,
                    "stop_distance_mm": args.stop_distance_mm})
                result.update(cmd="commission", axis=args.axis, power_switch_in_hand=True)
            else:
                result = run_commission(bench, session, args.duty, args.seconds)
                result["observed"] = _observed(args)
                m = result["metrics"]
                result["verdict"] = sign_verdict(m["duty_e4"], m["measured_sign"], result["observed"],
                                                 duty_sign_in_build=m["duty_sign_in_build"])
                result.update(cmd="commission", axis=args.axis, power_switch_in_hand=True)
        elif args.cmd == "step":
            result = run_step(bench, session, args.wheel, args.target)
        else:
            result = run_scenario(bench, session, args.kind, args.rate)
            result["external"] = verdict(args.kind, {
                "output_response_ms": args.output_response_ms, "stop_time_ms": args.stop_time_ms,
                "stop_distance_mm": args.stop_distance_mm}, floor=args.floor)
    except Refused as exc:
        result = {"result": "refused", "why": str(exc)}
    except BaseException as exc:  # Ctrl-C and anything else: an aborted run (Codex impl review P1-1)
        result = {"result": "aborted", "why": f"{type(exc).__name__}: {exc}"}
        session.aborted = session.aborted or "exception"
    finally:
        # once an ARM may have gone out, clean up whatever the last cached STATE says
        if session.arm_requested and session.cleanup_confirmed is not True:
            try:
                session.cleanup()
            except BaseException as exc:  # noqa: BLE001 -- the result row is still written (Codex re-review P2-5)
                session.cleanup_confirmed = False
                result["cleanup_error"] = f"{type(exc).__name__}: {exc}"
                result["result"] = "aborted" if result.get("result") == "ok" else result.get("result", "aborted")
            result["cleanup_confirmed"] = session.cleanup_confirmed
    result["record_errors"] = bench.record_errors
    bench.emit({"t": "result", **result})
    return result


if __name__ == "__main__":
    sys.exit(main())
