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
                      [--bypass enc_stuck,enc_polarity] [--wheels-lifted] [--vcc-watched]
                      [--profile-tri LO HI PERIOD CYCLES]
                      (L/R past duty 0.15 or 1.5 s: an open commissioning image with lim=lr1, --wheels-lifted and
                      the phrase retyped on a terminal; design 2026-11-25-bench-commission-limits 1.2)
  bringup.py commission --device D --out run.jsonl --axis L|R --duty X --scenario limit_disarm|hold_disarm
                      [--limit-s 2.0] [--disarm-delay-ms 50] --power-switch-in-hand [external ...]
  bringup.py kth      --record run.jsonl --configured-m V --configured-h V --image ID [--axis-absent M|H]
                      (S2: the KTH CRC variant from the v3 counters; record while moving each axis through its range)
  bringup.py clock    --record run.jsonl
                      (G7: MCU time -> host monotonic time per host boot and MCU boot, the delay envelope fit and its
                      quality figures; a teleop record's read_ns, a bring-up record's host_ns. No marker needed.)
"""

from __future__ import annotations

import argparse
import json
import math
import secrets
import random
import sys
import time
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

from forklift_core.live.executor_lock import DeviceBusy
from forklift_core.live.synthetic import (
    OUTPUT_MARKS,
    SyntheticRefused,
    bench_opener,
    device_identity,
    is_synthetic,
    refuse_or_mark,
    writer_marker,
)
from forklift_core.mcu_link.build_policy import Client, commission_allowed
from forklift_core.mcu_link.messages import (
    BenchRunFlag,
    BenchStop,
    Drive,
    FaultFlag,
    LinkDecoder,
    McuState,
    Mode,
    RejectReason,
    ResetCause,  # noqa: F401 -- re-exported for the tool's tests
    StateFlag,
    valid_after_echo_ms,
)
from forklift_core.plant import bench_limits as bl
from forklift_core.plant import bench_session
from forklift_core.plant import duty_disarm as dd
from forklift_core.plant import judgement
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
from forklift_core.plant.markers import marker_input_state, marker_reader
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost
from forklift_core.plant.real_vehicle import RearDrive, load_real_vehicle
from forklift_core.plant.record_rows import (  # noqa: F401 -- moved; kept here by name
    sample_from_row,
    state_row,
)
from forklift_core.plant.write_log import (
    Seen,
    Write,
    accepted,
    runs_of,
    write_row,
    writes_from_rows,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from real_mcu_link import _BAUDS, BenchTtyLink, ReconnectingLink, open_raw  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
STEP_MAX_S = 3.0
CLEANUP_S = 1.0
OBSERVE_S = 2.0
STATE_GAP_S = 0.2  # after an event, two STATEs further apart than this are an observation failure
RESULT_PAIR_WAIT_S = 0.15  # a STATE out of ARMED: how long its result pair may lag
FIRST_NONZERO_SLACK_S = 0.02  # a distance run's first non-zero: its own deadline plus two STATE periods
REST_OBSERVE_S = 0.6  # after DISARMED: the 0.5 s settling window at 100 Hz, and a margin
BOARD_ENDS = ("distance_reached", "distance_provisional", "no_progress", "kth_zero_suspect", "kth_lost")
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
THRESHOLD_KIND = {"limit_disarm": "disarm", "hold_disarm": "disarm", "duty_disarm": "disarm"}
HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=16)
# ② the commissioning host: a shorter validity bounds how long a COMMISSION runs on after the window stop
COMMISSION_HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=COMMISSION_CAP_MS, arm_timeout_s=0.3,
                                 zero_valid_ms=COMMISSION_CAP_MS, outbox_max=16)
STOPPING_WAIT_S = 1.0  # ② the cleanup lets a commissioning STOPPING (four-axis brake) run out before its DISARM


class Refused(Exception):
    """A drive command's safety condition (design 3) refused the run before anything moved."""


class _BenchHost(McuLinkHost):
    """The bring-up host with its link's openings and losses told to the record (design 2026-11-25 1.2, PR-2a): each
    opening is a new connection number, and the record's own decoder drops a half frame of the old one."""

    on_open = None
    on_lost = None

    products_seen: list | None = None

    def link_identity(self, product):
        if self.on_open is not None:
            self.on_open()
        if self.products_seen is None:
            self.products_seen = []
        self.products_seen.append(product)  # every opening's product: a cpr judged by all of them (A1a-2 r2)
        return super().link_identity(product)

    def disconnected(self, now_ns):
        if self.on_lost is not None:
            self.on_lost()
        return super().disconnected(now_ns)


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
    initial_seq: int | None = None  # the host's first seq (tests start it near the 16-bit wrap)
    # design 2026-11-25 1.2 (PR-2a): every write kept in order with its connection and completion time, and the
    # connection each STATE was read on -- the record matches acceptances with these, never with a seq alone
    conn: int = 0
    writes: list = field(default_factory=list)
    sample_conn: list = field(default_factory=list)
    phase: str | None = None  # the tool's current step, given to every frame it queues (kept in its write row)
    # a real device (main sets it): an open commissioning board whose USB product string cannot be read is refused --
    # it could be fw-single, which an L(0) arming zero would fix on L (Codex PR-B P2-1)
    product_required: bool = False
    gate_holder: dict | None = None  # a bench run's link gate state (main): the cleanup cancels it
    _queued_phase: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self._tap = LinkDecoder()
        kw = {} if self.initial_seq is None else {"initial_seq": self.initial_seq}
        self.host = _BenchHost(COMMISSION_HOST_CFG if self.commission else HOST_CFG, log=self._log,
                               client=Client.BRINGUP, allow_locked=self.allow_locked, commission=self.commission, **kw)
        self.host.on_open = self._opened
        self.host.on_lost = self._tap.discard_partial

    def _opened(self) -> None:
        self.conn += 1
        self._tap.discard_partial()
        self.emit({"t": "connection", "conn": self.conn, "host_ns": time.monotonic_ns()})

    def _log(self, what, now_ns, data, info) -> None:
        try:
            if what == "received":
                for r in self._tap.feed(data, now_ns):
                    if isinstance(r.message, McuState):
                        self._sample(Sample(r.received_ns, r.message))
            elif what == "queued" and info:
                self.last_queued = dict(info)
                self._queued_phase[(info["kind"], info["seq"])] = self.phase
            elif what == "written" and info:
                done = time.monotonic_ns()
                msg = None
                for r in LinkDecoder().feed(bytes(data), now_ns):
                    msg = r.message
                self.written[info["seq"]] = (info["kind"], now_ns, msg)
                w = Write(order=len(self.writes), conn=self.conn, kind=info["kind"], seq=info["seq"], start_ns=now_ns,
                          done_ns=done, boot_id=getattr(msg, "boot_id", None),
                          echo_mcu_time_us=getattr(msg, "echo_mcu_time_us", None),
                          valid_ms=getattr(msg, "valid_after_echo_ms", None), arm_gen=getattr(msg, "arm_gen", None),
                          axis=getattr(msg, "axis", None), duty_e4=getattr(msg, "duty_e4", None),
                          phase=self._queued_phase.pop((info["kind"], info["seq"]), self.phase))
                self.writes.append(w)
                # the record contract (PR-2a): order, connection, start and completion, the step it was queued in and
                # the command's fields -- enough to redo every match from the JSONL alone (write_log.writes_from_rows)
                self.emit({"t": "written", "host_ns": now_ns, **info, **write_row(w)})
        except Exception:  # noqa: BLE001 -- the host's receive and send must go on whatever the record does
            self.record_errors += 1

    def _sample(self, s: Sample) -> None:
        st = s.state
        self.samples.append(s)  # in memory first
        self.sample_conn.append(self.conn)
        if not self.boots or self.boots[-1] != st.boot_id:
            if self.boots:
                # contract 3.3: a new boot mid-session may be a motor-start brown-out even with reset_cause power
                self.emit({"t": "warning", "what": "boot_changed", "host_ns": s.host_ns, "boot_id": st.boot_id,
                           "reset_cause": st.reset_cause.name})
            self.boots.append(st.boot_id)
        self.emit({**state_row(s), "conn": self.conn})

    def seen(self, start: int = 0) -> list[Seen]:
        """The samples from ``start`` as the matcher reads them (an index into this list is ``start`` less)."""
        samples = self.samples[start:]
        conns = self.sample_conn[start:]
        conns += [self.conn] * (len(samples) - len(conns))  # samples put by hand have no recorded connection
        return [Seen(s.host_ns, c, s.state.boot_id, s.state.mcu_time_us, s.state.last_rx_seq, s.state.arm_gen)
                for s, c in zip(samples, conns, strict=True)]

    def emit(self, row: dict) -> None:
        try:
            self.out.write(json.dumps(row, default=str) + "\n")
        except (OSError, ValueError):
            self.record_errors += 1

    def flush_record(self) -> bool:
        """Push the record out of this process's buffer now (a row that must exist before the next step)."""
        try:
            self.out.flush()
            return True
        except (OSError, ValueError):
            self.record_errors += 1
            return False

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


CLEAR_ACCEPT_S = 2.0  # each frame's seq echoed in last_rx_seq (the firmware leaves it on a refused CLEAR_FAULTS)
CLEAR_READY_S = 3.0  # then the driver re-initialisation: DISARMED with DRIVERS_READY
CLEAR_DRAIN_S = 0.3  # read out what was already buffered before anything is judged


def _latch_view(st: McuState) -> dict:
    return {"mode": st.mode.name, "reject_reason": st.reject_reason.name,
            "fault_flags": [f.name for f in FaultFlag if f in st.fault_flags],
            "drivers_ready": bool(st.flags & StateFlag.DRIVERS_READY)}


def _echo_index(bench: Bench, kind: str, seq: int, conn: int, boot: int, floor_us: int, after: int = -1) -> int | None:
    """The first STATE that shows this run's frame taken: the frame (kind, seq) written on connection ``conn``, then a
    STATE read after its last byte on the same connection and boot, newer than ``floor_us`` and past sample index
    ``after``, whose last_rx_seq is the frame's seq (Codex PR-A P2-1: never a buffered STATE of an earlier run)."""
    seq &= 0xFFFF
    w = next((w for w in bench.writes if w.kind == kind and (w.seq & 0xFFFF) == seq and w.conn == conn), None)
    if w is None:
        return None
    for i in range(max(after + 1, 0), len(bench.samples)):
        s = bench.samples[i]
        if (bench.sample_conn[i] == conn and s.host_ns >= w.done_ns and s.state.boot_id == boot
                and s.state.mcu_time_us > floor_us and (s.state.last_rx_seq & 0xFFFF) == seq):
            return i
    return None


def clear_faults(bench: Bench) -> dict:
    """One CLEAR_FAULTS, only in FAULT or ESTOP (partial-parts plan 2026-11-25), after an echoed DISARM. Nothing is
    armed after it.

    STATE repeats the last accepted seq, and a bring-up process numbers its frames from its own start, so a STATE made
    before this run's frame arrived (a boot's 0, an earlier run's seqs in flight) could carry the same number (Codex
    PR-A rounds 1-2). The bench is therefore started at a random seq (``main``), and a frame whose seq any STATE of this
    session has already shown is not used. The echo must also be a STATE of the start's connection and boot, read
    after the frame's last byte, and the CLEAR's after the DISARM's. Residual (Codex PR-A round 3): an earlier run on
    the same boot that drew the same start (1 in 65,535) with its STATEs still unread after the drain could supply both
    echoes; "cleared" therefore only lets the operator ask for a new permission -- ARM is gated separately and a board
    still in FAULT/ESTOP refuses it. Cleared means the newest STATE of that connection and boot, after the CLEAR's
    echo, is DISARMED with DRIVERS_READY. A refusal (sensor invalid, e-stop) leaves the seq unechoed and is reported
    with the reject reason; a reconnect or a reboot on the way ends it unconfirmed. A CLEAR already queued when a new
    boot shows up may still be written; it carries the old boot_id and the new boot refuses it (boot mismatch)."""
    if not bench.run_until(lambda: bench.state is not None, 3.0):
        return {"result": "no_state"}
    bench.run_until(lambda: False, CLEAR_DRAIN_S)
    st, conn = bench.state, bench.conn
    boot, floor_us = st.boot_id, st.mcu_time_us
    before = _latch_view(st)
    if st.mode not in (Mode.FAULT, Mode.ESTOP):
        return {"result": "refused", "why": "not_latched", "before": before}

    def seen() -> set:
        return {0} | {smp.state.last_rx_seq & 0xFFFF for smp in bench.samples}

    def broken() -> str | None:
        if bench.conn != conn:
            return "reconnected"
        return "rebooted" if bench.state.boot_id != boot else None

    def wait(kind, seq, after=-1):
        found: list = []

        def done():
            if broken() is not None:
                return True
            i = _echo_index(bench, kind, seq, conn, boot, floor_us, after)
            if i is not None:
                found.append(i)
            return bool(found)
        bench.run_until(done, CLEAR_ACCEPT_S)
        return found[0] if found and broken() is None else None

    bench.host.request_disarm(time.monotonic_ns())
    dseq = bench.host.queued_disarm_seq()
    if dseq in seen():
        return {"result": "seq_collision", "why": f"DISARM seq {dseq} already shown by a STATE", "before": before}
    di = wait("DISARM", dseq)
    if di is None:
        return {"result": broken() or "no_echo", "why": "the DISARM before the clear was not echoed", "before": before}
    blocked = bench.host.request_clear_faults(time.monotonic_ns())
    if blocked is not None:
        return {"result": "refused", "why": blocked.reason, "before": before}
    seq = bench.host.last_queued_seq & 0xFFFF
    if seq in seen():
        bench.host.request_disarm(time.monotonic_ns())  # drops the queued, unwritten CLEAR_FAULTS
        return {"result": "seq_collision", "why": f"CLEAR_FAULTS seq {seq} already shown by a STATE",
                "before": before}
    ci = wait("CLEAR_FAULTS", seq, after=di)
    if ci is None:
        return {"result": broken() or "not_accepted", "seq": seq, "before": before, "after": _latch_view(bench.state)}

    def ready() -> bool:
        # the newest STATE only: a re-latch read in the same batch as a ready STATE is not a clear (Codex round 2)
        last = len(bench.samples) - 1
        s_ = bench.samples[last].state
        return (last > ci and bench.sample_conn[last] == conn and s_.boot_id == boot and s_.mode == Mode.DISARMED
                and bool(s_.flags & StateFlag.DRIVERS_READY))
    bench.run_until(lambda: broken() is not None or ready(), CLEAR_READY_S)
    why = broken()
    ok = why is None and ready()
    return {"result": "cleared" if ok else (why or "not_ready"), "seq": seq, "before": before,
            "after": _latch_view(bench.state)}


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
        self.last_hold_seqs: list[int] = []  # every seq the latest hold queued (a STATE may skip the first)

    def arm(self, budget_s: float) -> None:
        """Arm to ready; ``budget_s`` fixes the run's end, past which no command stays valid."""
        b = self.b
        if not b.run_until(lambda: b.state is not None and b.state.mode == Mode.DISARMED, 3.0):
            raise Refused("no DISARMED STATE")
        if b.conn > 1 or b.host.disconnects:
            # the operator confirmed this run on its first connection; a reconnect before the ARM ends it (real safety
            # wiring design 8.1, Codex design round 3 P1) -- no ARM on a link that came back
            raise Refused("the link reconnected before the ARM: run it again to confirm")
        self.preflight(b.state)
        if self.floor:
            # design 3: a floor run needs real outputs, a wired e-stop and the main switch in reach
            why = floor_refusal(b.state.flags, allow_locked=b.allow_locked, estop_waived=b.host.estop_waived)
            if why is not None:
                raise Refused(why)
            if b.state.flags & StateFlag.ESTOP_UNWIRED:
                print(f"WARNING floor: no e-stop input on this board; waived by its image ({b.host.usb_product}) -- "
                      "the main power switch is the only stop outside the MCU", file=sys.stderr, flush=True)
            if not self.power_switch_reachable:
                raise Refused("floor: confirm the main power switch is reachable (--power-switch-reachable)")
        self.steer_hold = b.state.steering_rad  # the other axes stay where they are (design 3)
        b.phase = "arm"
        self.test_end_ns = time.monotonic_ns() + int(budget_s * 1e9)
        self.arm_requested = True
        blocked = self.request_arm(time.monotonic_ns())
        if blocked is not None:
            raise Refused(f"arm blocked: {blocked.reason}")
        if not b.run_until(lambda: b.host.ready, 2.0):  # arm_pending -> zero_pending -> ready, no retry
            raise Refused("not ready after ARM")
        # the ARM was taken (zero seen): told apart from the pre-ARM check row (Codex PR-1 re-review 3 P2)
        st = b.host.state  # the STATE the host judged ready on, not a later-decoded older one (Codex re-review 4 P3)
        b.emit({"t": "armed", "host_ns": time.monotonic_ns(), "boot_id": st.boot_id, "arm_gen": st.arm_gen})

    def preflight(self, state: McuState) -> None:
        check_build(state, allow_locked=self.b.allow_locked)

    def request_arm(self, now_ns: int):
        return self.b.host.request_arm(now_ns)

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
        self.last_hold_seqs = []
        while time.monotonic_ns() < stop_ns:
            seq = self.send(left, right, bound)
            first = first if first is not None else seq
            if seq is not None:
                self.last_hold_seqs.append(seq)
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
        b.phase = "cleanup"
        o0 = len(b.writes)
        t0 = time.monotonic_ns()
        b.host.request_disarm(t0)

        def done() -> bool:
            # The MCU's own word that it took this DISARM and is stopped: a STATE read on the connection the DISARM
            # was written on, after its last byte went out, naming its seq, in a stopped mode. Not a transition (as
            # the measured span uses): this confirms a stop, and the mode is read from the STATE itself, so a seq
            # from an earlier wrap cannot make a moving MCU look stopped -- and a DISARM written before the first
            # STATE of a new connection is confirmed too (Codex PR-2a re-reviews 2-4). A lost DISARM is re-sent by
            # the run's final cleanup; the MCU's command expiry stops it meanwhile.
            # The STATE the host itself took as the latest valid one (never a sample it refused as out of order, nor a
            # stopped one buffered before a newer ARMED one -- Codex PR-2a re-reviews 5-6): received after the
            # DISARM's last byte on the connection it went out on, naming its seq, in a stopped mode.
            st, rx = b.host.state, b.host.state_rx_ns
            if st is None or rx is None or st.mode not in (Mode.DISARMED, Mode.FAULT, Mode.ESTOP):
                return False
            return any(w.kind == "DISARM" and w.conn == b.conn and w.seq == st.last_rx_seq and w.done_ns <= rx
                       for w in b.writes[o0:])

        ok = b.run_until(done, CLEANUP_S)
        b.emit({"t": "cleanup", "confirmed": bool(ok)})
        self.cleanup_confirmed = bool(ok)
        return ok


def mcu_time_of_acceptance(samples, seqs, start: int = 0):
    """(MCU time, received host time, seq) of the first STATE that reports any of ``seqs`` accepted (last_rx_seq).
    A STATE names only the last DRIVE received, so when two land between STATEs the first seq is never named
    (ws1 19006, under load): any seq of the step's hold is its acceptance. The STATE's time is then up to one DRIVE
    cycle after the step's first DRIVE reached the MCU -- the rise time can read that much short. Only samples from
    ``start`` on count (taken when the hold began), so an older STATE naming a reused seq never answers."""
    wanted = set(seqs)
    for s in samples[start:]:
        if s.state.last_rx_seq in wanted:
            return s.state.mcu_time_us, s.host_ns, s.state.last_rx_seq
    return None, None, None


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
    step_seqs, step_start = [], len(bench.samples)
    if session.aborted is None:
        session.hold(left, right, STEP_MAX_S)
        step_seqs = list(session.last_hold_seqs)
    if session.aborted is None:
        session.hold(0.0, 0.0, 1.0)
    session.cleanup()
    if session.aborted is None and not step_seqs:
        return _final({"result": "step_not_sent"}, session)  # the host queued no step DRIVE at all
    step_us, step_host, seq = mcu_time_of_acceptance(bench.samples, step_seqs, step_start)
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
_NO_BITS = FaultFlag(0)
POST_ZERO_S = 0.5


class CommissionSession(Session):
    """Design ② 3: a commissioning build only, the picked axis's open-loop duty, early stops, the tool ① cleanup."""

    def __init__(self, bench: Bench, axis: int, window: tuple[float, float] | None, *,
                 duty_max: float = COMMISSION_DUTY_MAX, ignore: FaultFlag = _NO_BITS, large: bool = False,
                 scenario: str | None = None, single_axis: bool = False, sensorless: bool = False) -> None:
        super().__init__(bench, 0.0)
        self.axis, self.window = axis, window
        # partial-parts plan 2026-11-25 PR-B: the fw-single request, and a steering/lift run with no window
        self.single_axis, self.sensorless = single_axis, sensorless
        # design 2026-11-25 1.2: the run's own duty bound (0.15, or the large run's), the encoder bits the early
        # stop leaves out (--bypass), whether the board must carry lim=lr1, and a scenario (locked boards only)
        self.duty_max, self.ignore, self.large, self.scenario = duty_max, ignore, large, scenario
        self.preflight_product: str | None = None
        # the bench image (bench console plan A1a): the run's BENCH_SET ask (None: the image's defaults), whether the
        # board is the bench image, the run id the BENCH_SET and BENCH_ARM carry, and what the board took
        self.bench: bl.BenchAsk | None = None
        self.bench_image = False
        self.run_id: int | None = None
        self.bench_taken: dict | None = None
        self._prev_result: tuple | None = None
        self._prev_final: dict | None = None
        self.board_ended: str | None = None  # a distance run the board ended itself (stop 8-12)
        self.bench_start: str | None = None  # a distance run's first non-zero: accepted/refused/unconfirmed/...
        self._bench_pending: dict | None = None
        self._boot0: int | None = None
        self.sent: list[tuple[int, int]] = []  # (seq, duty_e4) of every COMMISSION queued by the tool
        self.holds_sent = 0  # BENCH_HOLDs queued (bench7)
        self.pre_run_pair_t: int | None = None  # the MCU time of the last pair before this run's BENCH_ARM
        self.stop_reason: str | None = None
        self.stopping_waited_ms: float | None = None
        self.last_expires_ns: int | None = None

    def preflight(self, state: McuState) -> None:
        if not commission_allowed(state.flags):
            raise Refused("not a commissioning build (STATE.flags.commission_build off): nothing is sent")
        if state.flags & StateFlag.OUTPUTS_LOCKED and not self.b.allow_locked:
            raise Refused("outputs_locked: nothing would turn (--allow-locked for a model)")
        if self.scenario in LIMIT_SCENARIOS and not state.flags & StateFlag.OUTPUTS_LOCKED:
            # design 2026-11-25 1.3: on an open board the wheels' limit is the validator's ceiling -- the time-limit
            # scenarios run on a locked board or the model; the stop after a tool DISARM is PR-2's duty_disarm
            raise Refused("scenario: the time-limit scenarios run on a locked commissioning board or the model only")
        # the product string the run is armed on (Codex PR-1 P2-2): checked here, kept with the run's conditions
        self.preflight_product = self.b.host.usb_product
        if self.large and (why := bl.board_refusal(self.preflight_product)) is not None:
            raise Refused(why)
        # fw-single is told apart by its product string's exact suffix only, read on this connection just now: a
        # request on another image, or a run on fw-single without the request, is refused before the ARM (UART7 has
        # no product string, so no fw-single run there)
        if (self.b.product_required and self.preflight_product is None
                and not state.flags & StateFlag.OUTPUTS_LOCKED):
            raise Refused("board: the USB product string of this open commissioning board could not be read -- its "
                          "image (fw-single or not) is unknown: nothing is armed")
        single_image = bl.single_axis_image(self.preflight_product)
        if self.single_axis and not single_image:
            raise Refused(f"single-axis: {self.preflight_product!r} is not a fw-single image (no product ending "
                          f"'{' '.join(bl.SINGLE_AXIS_SUFFIX)}')")
        if single_image and not self.single_axis:
            raise Refused("single-axis: the board is fw-single -- give --single-axis (an older tool would arm on L)")
        if single_image and state.flags & StateFlag.OUTPUTS_LOCKED:
            raise Refused("single-axis: a fw-single product with outputs_locked in STATE: the board disagrees")
        # the arming's zero COMMISSION fixes this run's axis on fw-single; a hold run's readiness zero is on its own
        # axis too (the board refuses another axis's, plan 4.3 independent round 2 P1-1); every other path keeps L(0)
        hold_run = self.bench is not None and self.bench.hold_run
        self.b.host.commission_zero_axis = self.axis if (single_image or hold_run) else 0
        # the bench image (plan A1a, contract 8): told apart by its exact suffix; any other bench generation is refused
        # for every run, bench values on another image too. On it every run sends its BENCH_SET (the defaults when
        # none was asked) and arms with BENCH_ARM, so no run inherits another's overrides
        self.bench_image = bl.bench_image(self.preflight_product)
        if bl.any_bench_image(self.preflight_product) and not self.bench_image:
            raise Refused(f"bench: {self.preflight_product!r} is another bench generation than "
                          f"'{' '.join(bl.BENCH_SUFFIX)}': refused for every run")
        if self.bench is not None and not self.bench_image:
            raise Refused(f"bench: {self.preflight_product!r} is not the bench image (no product ending "
                          f"'{' '.join(bl.BENCH_SUFFIX)}')")
        if self.bench_image:
            self._bench_set()
            state = self.b.state
        if self.axis >= 2 and not self.sensorless and self.window is None and not self._range_off():
            raise Refused("window: steering and lift need --window unless the bench run turns their range off")
        if self.axis >= 2 and not self.sensorless and self.window is not None:
            v = state.steering_rad if self.axis == 2 else state.lift_m
            lo, hi = self.window
            if not (v == v and lo < v < hi):
                raise Refused(f"window: the axis value {v} is not inside ({lo}, {hi})")
        # every pre-ARM check passed: the board and conditions the ARM is about to be asked on (a refused board has no
        # row; whether the ARM was then taken is the "armed" row)
        self.b.emit({"t": "pre_arm_checked", "product": self.preflight_product,
                     "lim": self.preflight_product is not None and bl.LIMITS_TOKEN in self.preflight_product.split(),
                     "axis": "LRMH"[self.axis], "large": self.large, "duty_max": self.duty_max,
                     "single_axis": single_image, "sensorless": self.sensorless,
                     "ignored_bits": [f.name for f in FaultFlag if f in self.ignore], "scenario": self.scenario})

    def _range_off(self) -> bool:
        name = {2: "steer_range", 3: "lift_range"}.get(self.axis)
        return self.bench is not None and name in self.bench.names

    def _bench_set(self) -> None:
        """Send this run's BENCH_SET with a fresh run id and read the board's acceptance (its seq in
        STATE.last_rx_seq, no reject); anything else refuses the run before the ARM."""
        b = self.b
        ask = self.bench or bl.BenchAsk(self.axis, (), None, None)
        conn0, disc0 = b.conn, b.host.disconnects
        self._boot0 = boot0 = b.state.boot_id  # the DISARMED STATE's boot, kept to the BENCH_ARM (A1a-2 r3)

        def same_link() -> bool:
            return (b.conn == conn0 and b.host.disconnects == disc0 and b.state is not None
                    and b.state.boot_id == boot0)

        # the board's record of the last run first (a CLI that ended before writing it down, same boot): recovered
        # here, before this run's BENCH_ARM clears it (contract 8.3; A1a-2 P2)
        stopped = (Mode.DISARMED, Mode.FAULT, Mode.ESTOP)
        t_stop = b.state.mcu_time_us  # Session.arm saw this DISARMED STATE

        def stopped_pair() -> bool:
            # a pair that came with an accepted STATE of this boot and arming generation, in a stopped mode, no older
            # than the stop seen now, with the latest STATE still stopped: an older pair (an empty record seen while
            # the last run drove) never stands for the board's record now (A1a-2 r3, r4)
            st, cur = b.host.bench_state_of, b.host.state
            return (st is not None and cur is not None and st.boot_id == cur.boot_id and st.arm_gen == cur.arm_gen
                    and st.mode in stopped and cur.mode in stopped and st.mcu_time_us >= t_stop)

        if not b.run_until(lambda: not same_link() or stopped_pair(), 1.0):
            raise Refused("bench: no BENCH_STATE with the board's current STATE on this connection")
        self._link_check(same_link, conn0, disc0)
        prev = b.host.bench_state
        if b.host.bench_state_of is not None:
            self.pre_run_pair_t = b.host.bench_state_of.mcu_time_us  # the zero base of the entry window (round 12)
        if prev.result_run_id != 0:
            errors0 = b.record_errors
            b.emit({"t": "bench_previous_result", "boot_id": b.state.boot_id, **result_fields(prev)})
            self._prev_result = (b.state.boot_id, prev.result_run_id)  # recorded again right before the BENCH_ARM
            b.flush_record()  # out of this process before the BENCH_ARM can clear the board's copy
            if b.record_errors != errors0:
                # the next ARM clears the board's record: never before it is written down (A1a-2 r2)
                raise Refused("bench: the last run's board record could not be written: nothing is armed")
        self.run_id = secrets.randbits(32) or 1
        # its seq differs from every number this boot's recent STATEs named, so no STATE written before the board
        # took this BENCH_SET can carry it (A1a-2 r2)
        recent = {s.state.last_rx_seq for s in b.samples[-256:] if s.state.boot_id == b.state.boot_id}
        while (b.host.skip_seq(0) & 0xFFFF) in recent:
            b.host.skip_seq(1)
        boot, conn = b.state.boot_id, b.conn
        echo_us = b.state.mcu_time_us
        blocked = b.host.bench_set(self.run_id, ask, time.monotonic_ns(), count_sign=ask.count_sign,
                                   source=ask.source, stop_counts=ask.stop_counts)
        if blocked is not None:
            raise Refused(f"bench: BENCH_SET not queued ({blocked.reason})")
        seq = b.host.last_queued_seq

        def taken() -> bool:
            # a STATE of this boot, received on this connection after the BENCH_SET's last byte went out, naming its
            # seq with no reject (never an older STATE that happens to carry the same number)
            st, rx = b.host.state, b.host.state_rx_ns
            if st is None or rx is None or st.boot_id != boot or not same_link() or st.mcu_time_us <= echo_us:
                return False
            w = next((w for w in reversed(b.writes) if w.kind == "BENCH_SET" and (w.seq & 0xFFFF) == seq
                      and w.conn == conn), None)
            return (w is not None and w.done_ns <= rx and st.last_rx_seq == seq
                    and st.reject_reason == RejectReason.NONE)

        if not b.run_until(taken, 1.0):
            self._link_check(same_link, conn0, disc0)
            why = None if b.state is None else b.state.reject_reason.name
            raise Refused(f"bench: the board did not take BENCH_SET seq {seq} (reject {why})")
        # provisional: the seq is the board's answer only once its arming names this run (confirm_bench_arm)
        self._bench_pending = {"seq": seq, "run_id": self.run_id, "bypass": list(ask.names),
                               "duty_max_e4": list(ask.duty_max_e4()), "time_ms": list(ask.time_ms()),
                               "overrides": ask.overrides(), "source": ask.source, "stop_counts": ask.stop_counts,
                               "count_sign": ask.count_sign, "mode": ask.mode}
        b.emit({"t": "bench_set_sent", **self._bench_pending})

    def _link_check(self, same_link, conn0: int, disc0: int) -> None:
        if same_link():
            return
        b = self.b
        if b.conn != conn0 or b.host.disconnects != disc0:
            raise Refused("the link reconnected before the ARM: run it again to confirm")
        raise Refused("the board rebooted (a new boot id on the same link) before the ARM: run it again to confirm")

    def arm(self, budget_s: float) -> None:
        super().arm(budget_s)
        if self.bench_image:
            self.confirm_bench_arm()

    def confirm_bench_arm(self) -> None:
        """The BENCH_SET consumed: a BENCH_STATE of this boot, paired with an ARMED STATE, naming this run -- before
        the first non-zero COMMISSION (A1a-2 r4). Only then is the board's acceptance recorded."""
        b = self.b

        def named() -> bool:
            bs, st = b.host.bench_state, b.host.bench_state_of
            return (bs is not None and st is not None and st.boot_id == self._boot0 and st.mode == Mode.ARMED
                    and bs.run_id == self.run_id)

        if not b.run_until(named, 0.5):
            raise Refused("bench: the board's arming never named this run's BENCH_SET")
        self.bench_taken = self._bench_pending  # the board took it, whatever happens next
        if not b.host.ready:
            b.emit({"t": "bench_set", **self.bench_taken})
            raise Refused("bench: the arming ended while its BENCH_SET was being confirmed")
        b.emit({"t": "bench_set", **self.bench_taken})

    def request_arm(self, now_ns: int):
        if self.bench_image:
            if self.b.conn > 1 or self.b.host.disconnects:
                raise Refused("the link reconnected before the ARM: run it again to confirm")
            if self.b.state is None or self.b.state.boot_id != self._boot0:
                raise Refused("the board rebooted (a new boot id on the same link) before the ARM: run it again to "
                              "confirm")
            self.b.host.pre_bench_arm = self._final_before_arm
            return self.b.host.request_bench_arm(self.run_id, now_ns)
        return self.b.host.request_arm(now_ns)

    def end_bench_run(self) -> None:
        """The bench image's end (contract 8.3): one zero COMMISSION after driving ends the run and the MCU stops it
        itself (STOPPING -> DISARMED); no more zeros, which it would refuse as stopping."""
        b = self.b
        if self.send_duty(0) is None:
            self.aborted = self.aborted or "bench_end_zero_not_queued"
            return
        b.run_until(lambda: b.state is not None and b.state.mode != Mode.ARMED, POST_ZERO_S + 0.5)

    def bench_result(self) -> dict | None:
        """The board's own record of this run from its latest BENCH_STATE (None off the bench image)."""
        if not self.bench_image:
            return None
        bs = self.b.host.bench_state
        if bs is None or bs.result_run_id != self.run_id:
            # the board's record of this run was not seen (a lost pair, a run that never drove): never another run's
            return {"run_id": self.run_id, "confirmed": False}
        return {"confirmed": True, **result_fields(bs)}

    def _final_before_arm(self) -> bool:
        """Right before the BENCH_ARM's first byte (the host's pre_bench_arm): the last result the host holds of the
        previous run is written down (again when it changed since), or the ARM, which clears it, is dropped (Codex
        A1b-1b round 1 P2-6, round 2 P2-4)."""
        b, prev = self.b, self.b.host.bench_state
        if self._prev_result is None or prev is None or prev.result_run_id != self._prev_result[1]:
            return True
        fields = result_fields(prev)
        if fields == self._prev_final:
            return True
        errors0 = b.record_errors
        b.emit({"t": "bench_previous_result_final", "boot_id": self._prev_result[0], **fields})
        b.flush_record()
        if b.record_errors != errors0:
            self.aborted = self.aborted or "previous_result_not_recorded"
            return False
        self._prev_final = fields
        return True

    def _distance_run(self) -> bool:
        return self.bench_image and self.bench is not None and self.bench.distance

    def _hold_run(self) -> bool:
        return self.bench_image and self.bench is not None and self.bench.hold_run

    def _board_run(self) -> bool:
        """A run the board may end itself (a distance run's 8-12, a hold's 10-12)."""
        return self._distance_run() or self._hold_run()

    def _board_ended_run(self) -> str | None:
        """The board ended this distance or hold run itself (contract 8.3, 8.7): never an authorization revoked."""
        if not self._board_run():
            return None
        br = self.bench_result() or {}
        # any first cause of this run: a distance rule, or an expiry after a lost acceptance sample (round 2 P2-3);
        # the run's verdict classifies it
        return br.get("stop_reason") if br.get("confirmed") else None

    def send(self, left, right, end_ns=None):  # noqa: ARG002 -- a commissioning build takes no DRIVE
        raise Refused("the commissioning tool sends no DRIVE")

    def send_duty(self, duty_e4: int, end_ns: int | None = None) -> int | None:
        if abs(duty_e4) > round(self.duty_max * 10000):
            raise Refused(f"duty {duty_e4 / 1e4} over the run's limit {self.duty_max}")
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
        self.last_expires_ns = expires  # when the host drops this frame if it is still unwritten
        return seq

    def hold_duty(self, duty_e4: int, seconds: float, stop=None, *, end_ns: int | None = None,
                  handover: bool = True, bound_ns: int | None = None) -> int | None:
        """``hold`` for one axis's duty: the same bound and hand-over; ``stop()`` naming a reason ends a nonzero
        hold at once (the caller then holds 0). ``end_ns``: an absolute end (a profile's piece, design 2026-11-25
        1.2) instead of now + seconds; ``handover=False``: the next piece follows at once, so no gap is left;
        ``bound_ns``: how long the commands may stay valid (a profile's own end: a piece's last command must outlast
        the step to the next piece, still within the cap)."""
        end_ns = time.monotonic_ns() + int(seconds * 1e9) if end_ns is None else end_ns
        nonzero = duty_e4 != 0
        bound = (end_ns if bound_ns is None else bound_ns) if nonzero else None
        stop_ns = end_ns - int(HANDOVER_S * 1e9) if nonzero and handover else end_ns
        first = None
        if stop is not None:
            # the latest samples are judged before the first nonzero command goes out (Codex re-review P2-4)
            self.b.cycle()
            if (reason := stop()) is not None:
                self.stop_reason = reason
                return first
        if nonzero and self._distance_run():
            if not self._first_nonzero_answered(duty_e4, bound):
                return None
            if not self.b.host.ready and self.board_ended is None:
                why = self._abort_reason()  # out of ARMED already: its result pair decides (impl r3 P2-2)
                if why is not None:
                    self.aborted = why
                    return None
            if stop is not None and (reason := stop()) is not None:
                self.stop_reason = reason  # the samples seen while its answer was awaited (impl r1 P1-2)
                return None
        while time.monotonic_ns() < stop_ns and self.board_ended is None:
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

    def send_hold(self, end_ns: int | None = None) -> int | None:
        """One BENCH_HOLD (bench7): the run's settings, valid at most the cap and never past ``end_ns``."""
        ask = self.bench
        now = time.monotonic_ns()
        expires = min(self.test_end_ns, now + self.b.host.cfg.cap_ms * 1_000_000)
        if end_ns is not None:
            expires = min(expires, end_ns)
        self.b.last_queued = None
        blocked = self.b.host.request_bench_hold(self.run_id, self.axis, ask.hold, ask.count_sign, expires, now)
        if blocked is not None or self.b.last_queued is None:
            return None
        seq = self.b.last_queued["seq"]
        self.holds_sent += 1
        self.last_expires_ns = expires
        return seq

    def hold_run(self, seconds: float, stop=None) -> None:
        """A hold run's drive (plan 4.3 A1b-2 decision 10): the first BENCH_HOLD once (the distance run's one-shot
        rule), then the same HOLD every cycle with its validity bound by the end T, stopped HANDOVER_S before T so the
        end zero lands before the commands expire; the board's own end (10-12, a limit, an expiry) ends it at once."""
        end_ns = time.monotonic_ns() + int(seconds * 1e9)
        stop_ns = end_ns - int(HANDOVER_S * 1e9)
        if stop is not None:
            self.b.cycle()
            if (reason := stop()) is not None:
                self.stop_reason = reason
                return
        if not self._first_nonzero_answered(0, end_ns, send=lambda: self.send_hold(end_ns), kind="BENCH_HOLD"):
            return
        while time.monotonic_ns() < stop_ns and self.board_ended is None:
            self.send_hold(end_ns)
            self.b.cycle()
            why = self._abort_reason()
            if why is not None:
                self.aborted = why
                return
            if stop is not None and (reason := stop()) is not None:
                self.stop_reason = reason
                return
            time.sleep(0.01)

    def _first_nonzero_answered(self, duty_e4: int, bound: int | None, *, send=None, kind: str = "COMMISSION") -> bool:
        """A distance run's first non-zero COMMISSION goes out once (plan 4.2 A1b-1 decision 3): no other until it
        is accepted or this run's result is latched. The answer is read from the STATEs and pairs the host itself
        accepted (its on_accepted / on_pair hooks), in receipt order, each made before the command's deadline plus
        two STATE periods (MCU clock) and received before the echo's receipt plus the validity plus 50 ms (host
        clock). Refused, or unanswered in time: the run is closed with a written zero, never retried (Codex A1b-1b
        rounds 1-4)."""
        b = self.b
        conn0 = b.conn
        echo_rx_ns = b.host.state_rx_ns  # the STATE the command echoes was received then (the host-clock bound)
        states: list = []  # (STATE, received ns) the host accepted, in order
        pairs: list = []  # (STATE, BENCH_STATE, received ns) the host accepted, in order
        prev_acc, prev_pair = b.host.on_accepted, b.host.on_pair  # a hold run keeps its own pair record going

        def on_acc(st, rx):
            states.append((st, rx))
            if prev_acc is not None:
                prev_acc(st, rx)

        def on_pair(st, bs, rx):
            pairs.append((st, bs, rx))
            if prev_pair is not None:
                prev_pair(st, bs, rx)

        b.host.on_accepted, b.host.on_pair = on_acc, on_pair
        try:
            return self._first_nonzero_wait(duty_e4, bound, conn0, echo_rx_ns, states, pairs, send=send, kind=kind)
        finally:
            b.host.on_accepted, b.host.on_pair = prev_acc, prev_pair

    def _first_nonzero_wait(self, duty_e4, bound, conn0, echo_rx_ns, states, pairs, *, send=None,
                            kind: str = "COMMISSION") -> bool:
        b = self.b
        seq = self.send_duty(duty_e4, bound) if send is None else send()
        if seq is None:
            self.aborted = "bench_start_not_queued"
            return False
        seq16 = seq & 0xFFFF
        verdict = {"v": None}  # accepted / refused:<why> / confirmed, from the first evidence in time
        lim = {}
        # the host-clock bound from the queued frame's own validity (the echo's receipt + validity + 50 ms), fixed now
        # whether or not its bytes ever go out (Codex A1b-1b round 5 P2)
        try:
            q_valid_ms = min(b.host.cfg.cap_ms, valid_after_echo_ms(self.last_expires_ns, echo_rx_ns))
        except (TypeError, ValueError):
            q_valid_ms = b.host.cfg.cap_ms
        host_deadline_ns = (echo_rx_ns or time.monotonic_ns()) + q_valid_ms * 1_000_000 + 50_000_000

        def write():
            return next((w for w in reversed(b.writes) if w.kind == kind and (w.seq & 0xFFFF) == seq16
                         and w.conn == conn0), None)

        def judge() -> bool:
            """True once judged: the first in-time evidence in receipt order, or a deadline passed."""
            if verdict["v"] is not None:
                return True
            w = write()
            if w is None or w.echo_mcu_time_us is None or w.valid_ms is None:
                return time.monotonic_ns() > host_deadline_ns  # never written: closed at the host bound
            if not lim:
                lim["mcu_us"] = w.echo_mcu_time_us + w.valid_ms * 1000 + int(FIRST_NONZERO_SLACK_S * 1e6)
                lim["host_ns"] = host_deadline_ns

            def ours(st, rx) -> bool:
                return (rx >= w.done_ns and st.boot_id == w.boot_id and st.arm_gen == w.arm_gen
                        and st.mcu_time_us > w.echo_mcu_time_us)

            def in_time(st, rx) -> bool:
                return st.mcu_time_us <= lim["mcu_us"] and rx <= lim["host_ns"]

            for st, bs, rx in pairs:  # this run's result in a pair made and received in time: it drove
                if ours(st, rx) and in_time(st, rx) and bs.result_run_id == self.run_id:
                    verdict["v"] = "confirmed"
                    return True
            for st, rx in states:
                if not ours(st, rx):
                    continue
                if not in_time(st, rx):
                    return True  # past a deadline, nothing in time before it (receipt order)
                if st.last_rx_seq == seq16:
                    verdict["v"] = "accepted"
                    return True
                if st.reject_reason != RejectReason.NONE:
                    verdict["v"] = f"refused:{st.reject_reason.name.lower()}"
                    return True
            return time.monotonic_ns() > lim["host_ns"]

        b.run_until(judge, max(0.0, (host_deadline_ns - time.monotonic_ns()) / 1e9))
        judge()
        v = verdict["v"]
        if v == "confirmed":
            self.bench_start = "confirmed_by_result"  # it drove: the board's own end follows
            self.board_ended = (self.bench_result() or {}).get("stop_reason")
        elif v == "accepted":
            self.bench_start = "accepted"
        elif v is not None and v.startswith("refused:"):
            self.bench_start = "refused"
            self.aborted = f"bench_start_refused_{v.split(':', 1)[1]}"
        else:
            self.bench_start = "unconfirmed"
            self.aborted = "bench_start_unconfirmed"
        b.emit({"t": "bench_start", "result": self.bench_start, "seq": seq})
        if self.bench_start in ("accepted", "confirmed_by_result"):
            return True
        if b.host.ready:
            zero = self.send_duty(0)  # closed with a zero (then the cleanup's DISARM), never another non-zero
            if zero is not None:
                # written, not only queued: the cleanup drops what is still queued (Codex round 2 P2-2)
                b.run_until(lambda: any(w.kind == "COMMISSION" and (w.seq & 0xFFFF) == (zero & 0xFFFF)
                                        and w.conn == conn0 for w in b.writes), 0.2)
        return False

    def observe_rest(self, seconds: float = REST_OBSERVE_S) -> dict | None:
        """After DISARMED, the run axis's position at 100 Hz (plan 4.2 A1b-1 decision 4): settled only when the count
        stayed whole and did not change for 0.5 s with fresh evidence; else the last value, unconfirmed."""
        if not (self.bench_image and self.bench is not None and self.bench.source == 1):
            return None
        b, seen, end = self.b, [], time.monotonic() + seconds
        last = None
        while time.monotonic() < end:
            b.cycle()
            bs, st = b.host.bench_state, b.host.bench_state_of
            if bs is not None and st is not None and bs is not last and st.mode == Mode.DISARMED:
                seen.append((st.mcu_time_us, bs))
                last = bs
            time.sleep(0.005)
        out = bench_session.rest_position(seen)
        b.emit({"t": "bench_rest_position", **out})
        return out

    def cleanup(self) -> bool:
        """Design ② 3: the tool ① cleanup, but never a DISARM into a commissioning STOPPING -- the firmware ends it
        on DISARM and its four-axis electric brake with it (Codex design P1-1, against contract 3.4). Queued
        commands are dropped and the STOPPING is let run out first (bounded by ``STOPPING_WAIT_S``)."""
        b = self.b
        b.host.drop_drives("cleanup")
        if b.gate_holder is not None:
            b.gate_holder["cancelled"] = True  # no BENCH_HOLD or non-zero COMMISSION continues from here
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
            watched = _AXIS_ENCODER_BITS[self.axis] & ~self.ignore  # --bypass leaves the named kinds out
            return "encoder_bit" if (state.fault_flags & watched) & ~at_start else None
        if self.sensorless:
            return None  # no KTH: the motor is off the mechanism (--motor-detached), the hold ends by time
        v = state.steering_rad if self.axis == 2 else state.lift_m
        if self.window is None:
            return None  # the bench run turned the range off: the hold ends by time (the board's own checks stay)
        lo, hi = self.window
        return None if v == v and lo <= v <= hi else "window"

    def _abort_reason(self) -> str | None:
        b = self.b
        if not b.host.ready:
            if self._board_run():
                # the STATE and its BENCH_STATE may come in two reads: the result pair decides (impl r1 P2-2)
                b.run_until(lambda: self._board_ended_run() is not None, RESULT_PAIR_WAIT_S)
            if (ended := self._board_ended_run()) is not None:
                self.board_ended = ended
                return None
            return "authorization_revoked"
        if b.state.reject_reason != RejectReason.NONE:
            return f"rejected_{b.state.reject_reason.name.lower()}"
        return None


def result_fields(bs) -> dict:
    """The board's result of a run from a BENCH_STATE (contract 8.3, 8.5): every field, and for a KTH run its
    distance or measuring verdict (plan 4.2 A1b-1 decision 7)."""
    stop = BenchStop(bs.stop_reason).name.lower() if bs.stop_reason <= max(BenchStop) else bs.stop_reason
    kind = ("hold" if bs.run_flags & int(BenchRunFlag.RES_HOLD)
            else "distance" if bs.run_flags & int(BenchRunFlag.RES_DISTANCE) else "duty")
    out = {"run_id": bs.result_run_id, "stop_reason": stop, "kind": kind,
           "axis": "LRMH"[bs.result_axis] if bs.result_axis < 4 else None, "source": bs.result_source,
           "blocked": bs.blocked, "distance_confirm": bs.distance_confirm, "run_flags": bs.run_flags,
           "distance_ref": bs.distance_ref, "cut_position": bs.cut_position,
           "end_evidence_age_ms": bs.end_evidence_age_ms,
           # bench7: the run's own bypass, fixed at its end (the current arming's is mcu_bypass), and a hold's target
           "result_bypass": sorted(n for n, bit in bl.BENCH_BYPASS_BITS.items() if bs.result_bypass & bit)}
    if kind == "hold":
        out.update(target=bs.target, deadband_entries=bs.deadband_entries)
    if bs.result_source == 1:
        # a KTH run: how far its end displacement is trusted, and the distance verdict when the board ended it by a
        # distance rule (the distance asked is not in BENCH_STATE: the verdict's category only)
        out["measure"] = bench_session.measure_verdict(run_flags=bs.run_flags, distance_ref=bs.distance_ref,
                                                       cut_position=bs.cut_position)
        if kind == "distance" and stop in BOARD_ENDS:  # a distance verdict on a distance run's result only
            out["distance"] = bench_session.distance_verdict(
                stop_reason=stop, confirm=bs.distance_confirm, run_flags=bs.run_flags, stop_counts=None,
                distance_ref=bs.distance_ref, cut_position=bs.cut_position)
    return out


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


def run_commission(bench: Bench, session: CommissionSession, duty: float, seconds: float,
                   profile: bl.Profile | None = None) -> dict:
    b, axis = bench, session.axis
    duty_e4 = int(round(duty * 10000))
    if profile is not None:
        seconds = profile.duration_s
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

    o_arm = len(b.writes)  # write orders from here on are this run's
    distance_run = getattr(session, "bench_image", False) and session._distance_run()
    if distance_run and abs(duty) > bl.HIGH_DUTY_DISTANCE:
        b.emit({"t": "bench_advisory", "advisory": "high_duty_distance_unverified", "duty": duty,
                "above": bl.HIGH_DUTY_DISTANCE})  # shown and recorded, never refused (lead 2026-11-28)
        print(f"advisory: a distance stop at duty {duty} is unverified above {bl.HIGH_DUTY_DISTANCE} "
              "(the axis speed may near the counting bound)", file=sys.stderr)
    b.phase = "pre_zero"
    session.hold_duty(0, PRE_ZERO_S)
    b.phase = "on"
    if session.aborted is None:
        if profile is None:
            session.hold_duty(duty_e4, seconds, early_stop)
        else:
            # design 2026-11-25 1.2: each piece ends at its absolute deadline from the profile's start, so the
            # loop's own overhead does not stretch the run (the diagnostic profile ran 31.45 s for 30 s)
            t0 = time.monotonic_ns()
            profile_end = t0 + int(round(len(profile.pieces) * bl.PIECE_S * 1e9))
            for k, piece_e4 in enumerate(profile.pieces):
                # no hand-over even on the last piece: it lasts its whole PIECE_S and the post-zero follows at once;
                # the commands stay valid HANDOVER_S past the profile's end so that zero lands before they expire
                # -- no command outlives profile_end + HANDOVER_S (Codex PR-1 P2-1)
                session.hold_duty(piece_e4, 0.0, early_stop, end_ns=t0 + int(round((k + 1) * bl.PIECE_S * 1e9)),
                                  handover=False, bound_ns=profile_end + int(HANDOVER_S * 1e9))
                if session.stop_reason is not None or session.aborted is not None:
                    break
        if session.board_ended is not None and session.stop_reason is None:
            session.stop_reason = f"board_{session.board_ended}"  # the board ended the distance run itself
        if session.stop_reason is None and session.aborted is None:
            session.stop_reason = "time"
    b.phase = "post_zero"
    if session.aborted is None:
        if getattr(session, "bench_image", False) and session.board_ended is not None:
            # the board ended it and is stopping by itself: a zero now would only be refused (stopping)
            b.run_until(lambda: b.state is not None and b.state.mode != Mode.ARMED, POST_ZERO_S + 0.5)
        elif getattr(session, "bench_image", False):
            session.end_bench_run()  # one zero ends the run; the board stops it itself (contract 8.3)
        else:
            session.hold_duty(0, POST_ZERO_S)
    session.cleanup()
    rest = session.observe_rest() if getattr(session, "bench_image", False) else None
    if getattr(session, "bench_image", False):
        # the board's own first cause decides: a run that ended by expiry, a time limit, a fault or anything but the
        # tool's one zero is not ok, whatever the samples say (A1a-2 P2)
        br = session.bench_result()
        if not br.get("confirmed"):
            b.run_until(lambda: (session.bench_result() or {}).get("confirmed", False), 0.3)
            br = session.bench_result()
        if not br.get("confirmed"):
            session.aborted = session.aborted or "bench_result_unconfirmed"
        elif distance_run and br.get("stop_reason") not in bench_session.DISTANCE_ENDS:
            # ended before its target (its zero, a time limit, an expiry): the run failed (plan 4.2 decision 8)
            session.aborted = session.aborted or f"bench_stop_{br.get('stop_reason')}"
        elif not distance_run and br.get("stop_reason") != "end_zero":
            session.aborted = session.aborted or f"bench_stop_{br.get('stop_reason')}"
    # Acceptances are matched with the write log (order, connection, completion, echo), never with a seq alone: a
    # long run wraps the 16-bit seq and a USB-only reconnect brings the same boot back on a new fd (design 2026-11-25
    # 1.2, PR-2a). An ambiguous sample leaves the run unjudgeable instead of picking a frame.
    seen = b.seen()
    # each command's phase comes from when it was queued, so a nonzero frame finishing during the post-zero still
    # counts as driving (Codex PR-2a P2-4)
    m_on = accepted(seen, b.writes, lambda w: w.kind == "COMMISSION" and w.phase == "on" and w.duty_e4 != 0, i_arm)
    on_index = m_on.index
    ambiguous = m_on.ambiguous_at
    # the driven span ends at the first sample accepting a later command, or at the MCU's own stop (it left ARMED:
    # time limit, expiry, fault) -- never at the record's end (Codex re-review P2-3)
    off_index = None
    if on_index is not None:
        m_off = accepted(seen, b.writes, lambda w: w.kind == "COMMISSION" and w.phase == "post_zero", on_index)
        left = next((i for i in range(on_index, len(b.samples)) if b.samples[i].state.mode != Mode.ARMED), None)
        cands = [i for i in (m_off.index, left) if i is not None]
        off_index = min(cands) if cands else None
        if m_off.ambiguous_at is not None and (off_index is None or m_off.ambiguous_at < off_index):
            ambiguous = m_off.ambiguous_at
    # the motion window ends with the post-zero hold, before the cleanup's DISARM
    end = len(b.samples)
    m_end = accepted(seen, b.writes, lambda w: w.kind == "DISARM" and w.order >= o_arm, i_arm)
    if m_end.index is not None:
        end = m_end.index
    if m_end.ambiguous_at is not None and ambiguous is None:
        ambiguous = m_end.ambiguous_at  # the window's end is unknown too (Codex PR-2a P2-3)
    if off_index is not None and off_index >= end:
        end = off_index + 1  # the sample that closes the driven span stays, even when it confirms the DISARM
    metrics = commission_metrics(b.samples[:end], axis=axis, duty_e4=duty_e4, on_index=on_index,
                                 off_index=off_index, sensorless=getattr(session, "sensorless", False))
    # the whole session, the cleanup included, apart from the dynamics' cut (Codex re-review P2-2)
    why = session_abort_reason(b.samples, i_arm)
    if why is not None:
        session.aborted = session.aborted or why
    faults = FaultFlag(0)
    for smp in b.samples[i_arm:]:
        faults |= smp.state.fault_flags
    result = "no_acceptance" if on_index is None else ("ok" if metrics.valid else "invalid_measurement")
    bench_out = {}
    if getattr(session, "bench_image", False) and session.bench is not None and session.bench.source == 1:
        br = session.bench_result() or {}
        no_drive_sample = on_index is None or b.samples[on_index].state.mode != Mode.ARMED
        if no_drive_sample and br.get("confirmed") and br.get("run_id") == session.run_id:
            # 100 Hz may miss a short distance run's acceptance and cut: the board's latched result says it drove;
            # the duty and current figures then have no sample (plan 4.2 A1b-1 decision 8)
            result = "board_record_only"
            metrics = replace(metrics, valid=False, reason="no_driving_sample")
        if br.get("confirmed"):
            if session.bench.distance:
                bench_out["distance"] = bench_session.distance_verdict(
                    stop_reason=br.get("stop_reason"), confirm=br.get("distance_confirm", 0),
                    run_flags=br.get("run_flags", 0), stop_counts=session.bench.stop_counts,
                    distance_ref=br.get("distance_ref", 0), cut_position=br.get("cut_position", 0))
            else:
                bench_out["measure"] = bench_session.measure_verdict(
                    run_flags=br.get("run_flags", 0), distance_ref=br.get("distance_ref", 0),
                    cut_position=br.get("cut_position", 0))
        if session.bench_start is not None:
            bench_out["start"] = session.bench_start
        if rest is not None:
            bench_out["rest"] = rest
        bench_out["bench_result"] = br  # the one read the verdicts above were made from (impl r1 P2-4)
    if ambiguous is not None:
        # the seq of a sample on the driven span names more than one write: no figure is taken from it
        result = "unjudgeable"
        metrics = replace(metrics, valid=False, reason="seq_ambiguous")
    extra = {}
    if profile is not None:
        # a profile is not a step: its sent and applied peaks instead of the step's steady-state figures
        applied = [s.state.duty_e4[axis] for s in b.samples[i_arm:end]]
        extra["profile"] = {"pieces": len(profile.pieces), "period_s": profile.period_s, "cycles": profile.cycles,
                            # from the frames really written (a queued one dropped on expiry is not sent)
                            "sent_max_e4": max((w.duty_e4 for w in b.writes[o_arm:] if w.kind == "COMMISSION"),
                                               key=abs, default=0),
                            "applied_max_e4": max(applied, key=abs, default=0)}
    m_out = asdict(metrics)
    if profile is not None:
        m_out.update(rate_mean_rad_s=None, rate_peak_rad_s=None, current_mean_ma=None)
    return _final({"result": result, "metrics": m_out, "stop": session.stop_reason, **extra, **bench_out,
                   "faults_seen": [f.name for f in FaultFlag if f in faults],
                   "stopping_waited_ms": session.stopping_waited_ms,
                   "build_flags": [f.name for f in StateFlag if f in b.state.flags],
                   "note": "software evidence only: the direction is the operator's, currents against the bench supply"},
                  session)


def run_hold(bench: Bench, session: CommissionSession, seconds: float) -> dict:
    """A bench7 hold run (plan 4.3 A1b-2 decisions 9-10): armed with its readiness zero on its own axis, the
    BENCH_HOLDs for ``seconds``, the end zero, the cleanup; then verdict ② from every pair of the run and the board's
    result, and the last second's deadband entries (shown, never blocking)."""
    b, ask = bench, session.bench
    h = ask.hold
    pairs: list = []  # (mcu_time_us, Mode, BenchState) of every pair the host accepted, in order

    def keep(st, bs, rx):  # noqa: ARG001
        pairs.append((st.mcu_time_us, st.mode, bs))

    b.host.on_pair = keep
    warned = {"label": None}
    i_arm = len(b.samples)
    try:
        session.arm(PRE_ZERO_S + seconds + POST_ZERO_S + 1.0)
        i_arm = len(b.samples)
        seen = {"i": len(b.samples) - 1}

        def stop() -> str | None:
            for smp in b.samples[max(seen["i"], 0):]:
                if (reason := session.early_stop(smp.state, FaultFlag(0))) is not None:
                    return reason
            seen["i"] = len(b.samples)
            # the last second's deadband entries, shown when they start (decision 7): never a stop
            pts = _entry_points(session, pairs)
            if pts:
                w = bench_session.entry_window(pts, pts[-1][0])
                if w["label"] in ("warn", "unknown") and w["upper"] is not None and w["label"] != warned["label"]:
                    warned["label"] = w["label"]
                    b.emit({"t": "bench_advisory", "advisory": f"deadband_reentry_{w['label']}", **w})
            return None

        b.phase = "pre_zero"
        session.hold_duty(0, PRE_ZERO_S)
        b.phase = "on"
        if session.aborted is None:
            session.hold_run(seconds, stop)
            if session.board_ended is not None and session.stop_reason is None:
                session.stop_reason = f"board_{session.board_ended}"
            if session.stop_reason is None and session.aborted is None:
                session.stop_reason = "time"
        b.phase = "post_zero"
        if session.aborted is None:
            if session.board_ended is not None:
                b.run_until(lambda: b.state is not None and b.state.mode != Mode.ARMED, POST_ZERO_S + 0.5)
            else:
                session.end_bench_run()
        session.cleanup()
        br = session.bench_result()
        if not br.get("confirmed"):
            b.run_until(lambda: (session.bench_result() or {}).get("confirmed", False), 0.3)
            br = session.bench_result()
        if not br.get("confirmed"):
            session.aborted = session.aborted or "bench_result_unconfirmed"
        elif br.get("stop_reason") != "end_zero":
            session.aborted = session.aborted or f"bench_stop_{br.get('stop_reason')}"
    finally:
        b.host.on_pair = None
    why = session_abort_reason(b.samples, i_arm)
    if why is not None:
        session.aborted = session.aborted or why
    verdict2 = (bench_session.hold_verdict(pairs, br, deadband=h.deadband, run_id=session.run_id)
                if br.get("confirmed") else {"result": "undecidable", "why": ["no_result"]})
    pts = _entry_points(session, pairs)
    windows = [bench_session.entry_window(pts[: i + 1], p[0]) for i, p in enumerate(pts) if len(p) == 2]
    entries = {"max_lower": max((w["lower"] or 0 for w in windows), default=0),
               "warned": any(w["label"] == "warn" for w in windows),
               "unknown": any(w["label"] == "unknown" and (w["upper"] or 0) >= bench_session.ENTRY_WARN
                              for w in windows),
               "total": br.get("deadband_entries")}
    entries["label"] = ("데드밴드 반복 진입(최근 1 s 최소 {} 회)".format(entries["max_lower"]) if entries["warned"]
                        else "데드밴드 반복 진입 미상" if entries["unknown"] else None)
    hold_out = {"settings": {"target": args_target(h), "kp": h.kp, "ki": h.ki, "kd": h.kd, "deadband": h.deadband,
                             "min_duty": h.min_duty, "cap": h.cap, "count_sign": ask.count_sign},
                "start": session.bench_start, "holds_sent": session.holds_sent, "verdict": verdict2,
                "entries": entries, "pairs": len(pairs),
                # what the PWM report is and is not (plan 4.3 decision 6), the stationary noise width not yet measured
                # (decision 7), the coast within 200 us of a run's start unverified (independent round 3 P2-2)
                "notes": ["PWM 출력(브리지 미확인)", "정지 폭 미측정", "구간 시작 관성 혼입 미확인"]}
    result = "ok" if session.aborted is None and session.cleanup_confirmed else "aborted"
    return _final({"result": result, "stop": session.stop_reason, "hold": hold_out, "bench_result": br,
                   "build_flags": [f.name for f in StateFlag if f in b.state.flags]}, session)


def args_target(h: bl.HoldAsk) -> str:
    name = {v: k for k, v in bl.HOLD_TARGETS.items()}[h.kind]
    return name if name in ("zero", "here") else f"{name}:{h.value}"


def _entry_points(session: CommissionSession, pairs: list) -> list:
    pre = [(session.pre_run_pair_t, Mode.DISARMED, _NoRun())] if session.pre_run_pair_t is not None else []
    return bench_session.entry_points(pre + pairs, session.run_id)


class _NoRun:
    run_id = 0
    deadband_entries = 0


LIMIT_SCENARIOS = ("limit_disarm", "hold_disarm")  # drive to the MCU's own time limit: locked boards and the model
COMMISSION_SCENARIOS = LIMIT_SCENARIOS + ("duty_disarm",)
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


def run_duty_disarm(bench: Bench, session: CommissionSession, duty: float, seconds: float, *,
                    synthetic: bool = False) -> dict:
    """Design 2026-11-25 1.3 (PR-2b): drive one wheel at ``duty`` until the STATEs show it held at that duty for
    0.5 s, then the tool's DISARM; the stop after it is the S5 item on the open image (its wheel time limit is the
    validator's ceiling). The test counts only as ``duty_disarm.validity`` says, from the record itself."""
    b, axis = bench, session.axis
    duty_e4 = int(round(duty * 10000))
    session.arm(PRE_ZERO_S + seconds + 2.0)
    i_arm = len(b.samples)
    out = {"kind": "duty_disarm", "event_host_ns": None, "disarm_seq": None}
    b.phase = "pre_zero"
    session.hold_duty(0, PRE_ZERO_S)
    if session.aborted is not None:
        session.cleanup()
        return _final({**out, "result": "aborted"}, session)
    b.phase = "on"
    end_ns = time.monotonic_ns() + int(seconds * 1e9)  # the approved nonzero output ends here, ramp included
    held = False
    last_seq, last_expires = None, end_ns
    # the encoder stuck/polarity early stop as in a plain run, --bypass leaving the named kinds out (Codex PR-2b P1-2):
    # the commissioning firmware latches those bits and stays ARMED, so the tool is what stops the drive
    at_start = b.state.fault_flags & _AXIS_ENCODER_BITS[axis]
    scanned = len(b.samples)
    while time.monotonic_ns() < end_ns:
        seq = session.send_duty(duty_e4, end_ns)
        if seq is not None:
            last_seq, last_expires = seq, session.last_expires_ns
        b.cycle()
        why = session._abort_reason()  # noqa: SLF001
        if why is not None:
            session.aborted = why
            break
        stop = next((r for smp in b.samples[scanned:] if (r := session.early_stop(smp.state, at_start)) is not None),
                    None)
        scanned = len(b.samples)
        if stop is not None:
            session.stop_reason = stop
            break
        rows = [{**state_row(s), "conn": c} for s, c in zip(b.samples[i_arm:], b.sample_conn[i_arm:], strict=True)]
        if dd.reach_and_hold(rows, axis, duty_e4)[1] is not None:
            held = True
            break
        time.sleep(0.01)
    if session.aborted is None and held:
        # the last refresh must really be out before the DISARM (request_disarm drops what is still queued), and
        # there must be room left under the approved end for the DISARM itself (design 1.3)
        def refresh_out() -> bool:
            return last_seq is None or any(w.seq == last_seq and w.kind == "COMMISSION" for w in b.writes[-8:])

        def wait_over() -> bool:
            # the STATEs read during the wait get the same early stop and authorization check, and end it at once
            # (Codex PR-2b re-reviews 2-3): a frame dropped on a disconnect is never written, so the wait would
            # otherwise last to the approved end; it is bounded by that frame's own validity (end_ns) anyway
            nonlocal scanned
            stop = next((r for smp in b.samples[scanned:]
                         if (r := session.early_stop(smp.state, at_start)) is not None), None)
            scanned = len(b.samples)
            if stop is not None:
                session.stop_reason = stop
            elif (why := session._abort_reason()) is not None:  # noqa: SLF001
                session.aborted = why
            return session.stop_reason is not None or session.aborted is not None or refresh_out()

        # bounded by the last refresh's own expiry: the host drops an unwritten frame then without revoking anything
        # (Codex PR-2b re-review 4 P2); the room under the approved end is checked separately below
        b.run_until(wait_over, max((min(last_expires, end_ns) - time.monotonic_ns()) / 1e9, 0.0))
        written = refresh_out()
        left = (end_ns - time.monotonic_ns()) / 1e9
        # the latest STATE and the drive authorization looked at again right before the test DISARM: never into a
        # STOPPING (an expiry's electric brake the cleanup waits out) nor after the authorization went (Codex PR-2b
        # P2-2)
        not_armed = b.state.mode != Mode.ARMED or not b.host.ready
        if session.stop_reason is not None or session.aborted is not None:
            pass  # cleaned up below without the test DISARM
        elif not written or left < dd.SEND_MARGIN_S or not_armed:
            out["why_not_sent"] = ("last_refresh_not_written" if not written else
                                   "not_armed_before_the_disarm" if not_armed else "no_room_before_the_approved_end")
        else:
            b.phase = "test_disarm"
            session.events_started = True
            b.host.request_disarm(time.monotonic_ns())
            b.run_until(lambda: any(w.kind == "DISARM" and w.phase == "test_disarm" for w in b.writes), 0.5)
            b.run_until(lambda: False, 0.6)  # keep observing the counts after the DISARM
    rows = [{**state_row(s), "conn": c} for s, c in zip(b.samples[i_arm:], b.sample_conn[i_arm:], strict=True)]
    v = dd.validity(rows, b.writes, axis=axis, duty_e4=duty_e4, synthetic=synthetic)
    out.update(validity=v, event_host_ns=v["disarm_done_ns"], disarm_seq=v["disarm_seq"], physical=v["physical"])
    session.cleanup()
    if session.aborted is not None:
        return _final({**out, "result": "aborted"}, session)
    if session.stop_reason is not None:
        return _final({**out, "result": "not_met", "why": f"early_stop_{session.stop_reason}"}, session)
    if not held:
        return _final({**out, "result": "not_met", "why": "hold_not_seen_before_the_approved_end"}, session)
    if v["reason"] == "seq_ambiguous":
        return _final({**out, "result": "unjudgeable"}, session)
    return _final({**out, "result": "ok" if v["valid"] else "not_met"}, session)


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


TOOL_KINDS = ("tx_cut", "disarm", "expiry") + COMMISSION_SCENARIOS  # events the tool itself issued in a run
OPERATOR_KINDS = ("iwdg", "power")  # events an operator marked (Enter) during a record or commission run


def _run_kind(header: dict) -> str | None:
    if header.get("cmd") == "scenario":
        return header.get("kind")
    if header.get("cmd") == "commission":
        return header.get("scenario")
    return None


def _header(run: list[dict]) -> dict:
    return next((r for r in run if r.get("t") == "header"), {})


def _tool_event(run: list[dict], kind: str, header: dict, synthetic: bool) -> tuple[int | None, dict | None]:
    """The host time of the event the tool itself issued in ``run`` (design 2026-11-25 1.3, Codex PR-2b P2-3): for
    ``duty_disarm`` the test DISARM's write completion, judged again from the run's STATEs and writes; for the other
    tool kinds the ``event_host_ns`` of the run's own result row."""
    if kind == "duty_disarm":
        states = [r for r in run if r.get("t") == "state"]
        duty_e4 = int(round(float(header.get("duty") or 0) * 10000))
        v = dd.validity(states, writes_from_rows(run), axis=AXES.get(header.get("axis"), 0), duty_e4=duty_e4,
                        synthetic=synthetic)
        return v["disarm_done_ns"], v
    res = next((r for r in reversed(run) if r.get("t") == "result"), {})
    ev = res.get("event_host_ns")
    return (None if ev is None else int(ev)), None


def _real_outputs(states: list[dict]) -> bool:
    """The run's STATEs show outputs that drive a motor: some STATE at all, none locked, no HIL plant."""
    return bool(states) and not any({"OUTPUTS_LOCKED", "HIL_PLANT"} & set(r.get("flags", ())) for r in states)


def _tool_run_official(kind: str, res: dict | None, states: list[dict]) -> bool:
    """A run the tool itself issued an event in may give an official pass only when it ended ``ok`` with its stop
    confirmed, ``tx_cut`` met its worst case, and its STATEs show real outputs (Codex PR-2b re-reviews 2-3). The tool's
    own result and ``analyze`` both ask this."""
    if res is None or res.get("result") != "ok" or res.get("cleanup_confirmed") is not True:
        return False
    if kind == "tx_cut" and (res.get("worst_case") or {}).get("met") is not True:
        return False
    return _real_outputs(states)


def analyze(record, marker: int, kind: str, external: dict, *, floor: bool = False, allow_bypassed: bool = False,
            allow_synthetic: bool = False) -> dict:
    """A recorded run around one event: the software timeline from the event's host time -- the last STATE before a
    gap or the record's end, the first mode change, the first all-zero duty, a new boot and its reset cause, the
    count settle -- and the external verdict.

    ``record``: a path, or the rows already read (the CLI reads once). Design 2026-11-25 1.3 (PR-2b):

    - an event the tool issued (``TOOL_KINDS``) is found in the record itself, in the runs of that kind (``marker``
      counts those runs: -1 the last); an older record that names no run kind falls back to its Enter markers and
      gets no official pass;
    - an operator's event (``OPERATOR_KINDS``) is its Enter marker (``marker`` counts markers);
    - the selected run is checked before its event is looked for: its kind, where it ran, its own marks (a bypassed,
      unknown-condition or diagnostic-image run is refused unless ``allow_bypassed``) and whether it is a rehearsal
      (refused unless ``allow_synthetic``) -- not another run appended to the same file (Codex PR-2b P2-4);
    - an official pass needs, for a tool event, the run's own result row ``ok`` with its stop confirmed, and STATEs
      showing real outputs; never from a rehearsal, from the time-limit scenarios (locked boards and the model
      only) or from a ``duty_disarm`` the record does not show valid on an open board (Codex PR-2b P1-3)."""
    rows = record if isinstance(record, list) else [
        json.loads(x) for x in Path(record).read_text(encoding="utf-8").splitlines() if x.strip()]
    runs = runs_of(rows)
    official = True
    validity = None
    m_ns, label = None, None
    kind_runs = [run for run in runs if _run_kind(_header(run)) == kind] if kind in TOOL_KINDS else []
    if kind_runs:
        if not -len(kind_runs) <= marker < len(kind_runs):
            return {"result": "no_event", "runs": len(kind_runs), "asked": marker}
        run = kind_runs[marker]
        header = _header(run)
    else:
        markers = [(run, r) for run in runs for r in run if r.get("t") == "marker"]
        named = sorted({k for run in runs if (k := _run_kind(_header(run))) is not None})
        if kind in TOOL_KINDS and named and not any(_run_kind(_header(run)) is None for run, _ in markers):
            # a record whose runs name other kinds: a kind mismatch, not a missing marker (Codex PR-2b re-review P2)
            return {"result": "refused", "why": f"the record's runs are {named}, not {kind!r}"}
        if not markers:
            return {"result": "no_marker"}
        if not -len(markers) <= marker < len(markers):
            return {"result": "no_marker", "markers": len(markers), "asked": marker}
        run, mrow = markers[marker]
        header = _header(run)
        if kind in TOOL_KINDS:
            if _run_kind(header) is not None:
                return {"result": "refused", "why": f"the marker's run is {_run_kind(header)!r}, not {kind!r}"}
            official = False  # an older record that does not name its kind: read, but no official pass
        m_ns, label = int(mrow["host_ns"]), mrow.get("label")
    run_kind = _run_kind(header)
    if header.get("cmd") == "commission" and floor:
        return {"result": "refused", "why": "a commissioning run is on the stand: --floor does not apply"}
    if header.get("cmd") == "scenario" and "floor" in header and bool(header["floor"]) != floor:
        return {"result": "refused", "why": f"the run was recorded with floor={bool(header['floor'])}"}
    states = [r for r in run if r.get("t") == "state"]
    if floor and any("COMMISSION_BUILD" in r.get("flags", ()) for r in states):
        return {"result": "refused", "why": "a commissioning build is on the stand: --floor does not apply"}
    jmarks = judgement.run_marks(run)
    try:
        judgement.check(jmarks, allow=allow_bypassed)
    except judgement.NotForGate as exc:
        return {"result": "refused", "why": str(exc)}
    synthetic = is_synthetic(header)
    if synthetic and not allow_synthetic:
        return {"result": "refused", "why": "a synthetic (rehearsal) run: not for measurement (give --allow-synthetic "
                                            "to read it; its pass fields stay empty)"}
    smarks = dict(OUTPUT_MARKS) if synthetic else {}
    run_result = None
    if kind_runs:
        m_ns, validity = _tool_event(run, kind, header, synthetic)
        res = next((r for r in reversed(run) if r.get("t") == "result"), None)
        run_result = None if res is None else {"result": res.get("result"),
                                               "cleanup_confirmed": res.get("cleanup_confirmed"),
                                               "worst_case_met": (res.get("worst_case") or {}).get("met")}
        if m_ns is None:
            return {**smarks, **jmarks, "result": "no_event", "kind": kind,
                    "why": "the run shows no event the tool issued", "run_result": run_result,
                    **({} if validity is None else {"validity": validity})}
        # the run's own end: an aborted run or an unconfirmed stop gives no pass, whatever its STATEs show before the
        # event (Codex PR-2b re-review P1)
        # locked outputs, a HIL plant or no STATE: nothing physical to judge (re-review P2)
        if not _tool_run_official(kind, res, states):
            official = False
    if not judgement.eligible(jmarks) or synthetic or kind in LIMIT_SCENARIOS:
        official = False
    if validity is not None and not (validity["valid"] and validity["physical"]):
        official = False
    samples = [sample_from_row(r) for r in run if r.get("t") == "state"]
    after = [s for s in samples if s.host_ns >= m_ns]
    before = [s for s in samples if s.host_ns < m_ns]
    ref = before[-1] if before else (after[0] if after else None)  # the state as the event began
    rel = (lambda s: None if s is None else (s.host_ns - m_ns) / 1e6)
    first_mode = next((s for s in after if ref is not None and s.state.mode != ref.state.mode), None)
    new_boot = next((s for s in after if ref is not None and s.state.boot_id != ref.state.boot_id), None)
    gaps = [(a.host_ns, b.host_ns) for a, b in zip(after, after[1:]) if b.host_ns - a.host_ns > STATE_GAP_S * 1e9]
    i0 = samples.index(after[0]) if after else len(samples)
    settle = count_settle(samples, i0) if after else None
    ext = verdict(kind, external, floor=floor)
    out = {**smarks, **jmarks, "result": "ok", "marker": label, "event_host_ns": m_ns, "kind": kind,
           "run_kind": run_kind, "official": official, "run_result": run_result}
    if validity is not None:
        out["validity"] = validity
    if not official:
        ext = judgement.empty_pass_fields(ext)
    out.update(software={"mode_change_ms": rel(first_mode),
                         "duty_zero_ms": rel(next((s for s in after if not any(s.state.duty_e4)), None)),
                         "new_boot_ms": rel(new_boot),
                         "new_boot_reset_cause": None if new_boot is None else new_boot.state.reset_cause.name,
                         "last_state_before_gap_ms": None if not gaps else (gaps[0][0] - m_ns) / 1e6,
                         "last_state_ms": rel(after[-1]) if after else None,
                         "count_settle_ms": None if settle is None or settle.host_ns is None
                         else (settle.host_ns - m_ns) / 1e6},
               external=ext, note="software evidence only: the S5 verdict takes the external measurements")
    return out


def floor_refusal(flags, *, allow_locked: bool, estop_waived: bool) -> str | None:
    """Why a --floor run must not arm (design 3): real outputs only, and a wired e-stop -- or, on V1.0, a board whose
    USB product string names the waiver image (plan v1 e-stop waiver 3)."""
    if allow_locked or flags & StateFlag.OUTPUTS_LOCKED:
        return "floor: outputs locked (or --allow-locked given)"
    if flags & StateFlag.ESTOP_UNWIRED and not estop_waived:
        return "floor: e-stop unwired and the board's image names no waiver"
    return None


def _cap() -> float:
    cfg = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
    # 0.2 m/s on the larger wheel radius (the configured one: nominal until S8, plan v1 e-stop waiver 3 -- without it
    # the cap stayed 1.0 rad/s and the 0.132 m/s test, 1.06 rad/s, was refused)
    return wheel_rate_cap(cfg.wheel_rate_limit_rad_s, max(cfg.wheel_radius_m))


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


_PART_AXIS = {"steer": "M", "lift": "H"}
_AXIS_PART = {v: k for k, v in _PART_AXIS.items()}


def _kth_absent(rows: list[dict], args) -> list[str]:
    """kth's absent axes: the record headers' declared parts (steer = M, lift = H) with --axis-absent; a flag a
    declaring header does not name is refused, and every fitted axis needs its configured variant. kth reads every
    STATE of the file, so its runs must declare the same parts (Codex PR-A P2-4): one bench set-up per file."""
    headers = [r for r in rows if r.get("t") == "header"]
    decls = {tuple(judgement.parts_absent(h)) if isinstance(h.get("parts_absent"), list) else None for h in headers}
    if len(decls) > 1:
        raise ValueError("the record's runs declare different absent parts (or some none): kth reads the whole file "
                         "-- record each bench set-up to its own file")
    declaring = decls not in (set(), {None})
    declared = {_PART_AXIS[p] for p in (next(iter(decls)) or ()) if p in _PART_AXIS} if declaring else set()
    flags = set(args.axis_absent)
    if declaring and flags - declared:
        raise ValueError(f"--axis-absent {sorted(flags - declared)[0]} disagrees with the record header's "
                         f"parts_absent (axes {sorted(declared)})")
    absent = sorted(declared | flags)
    if len(absent) == 2:
        raise ValueError("both KTH axes absent: nothing to check")
    for ax, v in (("M", args.configured_m), ("H", args.configured_h)):
        if ax in absent and v is not None:
            raise ValueError(f"--configured-{ax.lower()} given for absent axis {ax}")
        if ax not in absent and v is None:
            raise ValueError(f"--configured-{ax.lower()} is required (axis {ax} is fitted)")
    return absent


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("record", "cpr", "step", "scenario", "clear-faults"):
        p = sub.add_parser(name)
        p.add_argument("--device", required=True)
        p.add_argument("--baud", type=int, choices=sorted(_BAUDS))
        p.add_argument("--out", required=True)
        p.add_argument("--allow-locked", action="store_true")
        p.add_argument("--yes", action="store_true")
        p.add_argument("--bench", help="the rehearsal bench's bench.json: required on a pty device (the model MCU), "
                                       "refused otherwise; the record is marked synthetic")
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
    cm.add_argument("--bench", help="as for record: the rehearsal bench's bench.json on a pty device")
    cf = sub.choices["clear-faults"]
    cf.help = "clear a FAULT/ESTOP latch once its cause is fixed (partial-parts plan 2026-11-25); never arms"
    cf.add_argument("--cause", required=True,
                    help="what latched and what was done about it (kept in the header), e.g. 'H KTH unplugged'")
    for p in (*(sub.choices[n] for n in ("record", "cpr", "step", "scenario", "clear-faults")), cm):
        p.add_argument("--absent", action="append", default=[], choices=judgement.PARTS,
                       help="a part not fitted on this bench (repeatable): written into the run's header, which "
                            "first boot and kth read; steer or wheels absent makes the run no gate evidence "
                            "(partial-parts plan 2026-11-25)")
    cm.add_argument("--axis", choices=tuple(AXES), required=True)
    cm.add_argument("--duty", type=float,
                    help="the duty (required); with --hold the hold's duty cap, default 0.15 (plan 4.3 A1b-2)")
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
    cm.add_argument("--bypass", help="enc_stuck,enc_polarity: the wheel's encoder early stop leaves these out "
                                     "(the bits stay in the record; design 2026-11-25 1.2)")
    cm.add_argument("--single-axis", action="store_true",
                    help="the board is fw-single (product ends 'lim=lr1 sa1', checked before ARM): this run's axis is "
                         "fixed for the arming and the other channels stay cut (partial-parts plan 2026-11-25)")
    cm.add_argument("--motor-detached", action="store_true",
                    help="fw-single steering/lift: the motor is off the mechanism (retyped on a terminal, recorded); "
                         "required there, since no end stop protects a coupled axis without its sensor checks")
    # the bench image (bench console plan A1a, contract 8): one run's settings, sent before its BENCH_ARM
    cm.add_argument("--bench-bypass", help="bench image: comma names the board turns off for this run "
                                           f"({', '.join(sorted(bl.BENCH_BYPASS_BITS))}); the stopping side never")
    cm.add_argument("--bench-duty-max", type=float, help="bench image: this axis's duty limit for the run, (0, 1]")
    cm.add_argument("--bench-time-max", type=float, help="bench image: this axis's continuous output limit for the run, s")
    cm.add_argument("--stop-kth-turns", type=float,
                    help="bench image (bench7): steering/lift only -- the board cuts the output after this many KTH "
                         "sensor turns in the duty's direction (plan 4.2 A1b-1)")
    cm.add_argument("--kth-measure", action="store_true",
                    help="bench image (bench7): steering/lift only -- measure the run by the KTH count without a "
                         "distance stop (the end displacement and how far it is trusted)")
    cm.add_argument("--hold", metavar="TARGET",
                    help="bench image (bench7): hold steering/lift at zero | here | abs:V (raw 0..4095) | rel:N "
                         "(counts) for --seconds, closed loop on the KTH count; --duty is the cap (plan 4.3 A1b-2)")
    cm.add_argument("--hold-kp", type=float, default=0.5, help="hold: duty per rad (11-27 measured start 0.5)")
    cm.add_argument("--hold-ki", type=float, default=0.0, help="hold: duty per rad*s")
    cm.add_argument("--hold-kd", type=float, default=0.0,
                    help="hold: duty per rad/s (default 0; untested on hardware -- the bench bundle compares 0 and "
                         "0.02 on shaking)")
    cm.add_argument("--hold-deadband", type=int, default=12, help="hold: counts, |error| <= this gives output 0")
    cm.add_argument("--hold-min-duty", type=float, default=0.12,
                    help="hold: the least duty toward the error outside the deadband")
    cm.add_argument("--count-sign", type=int, choices=(1, -1), default=1,
                    help="bench image: +1 when a positive duty raises the KTH count, -1 when it lowers it")
    cm.add_argument("--sensorless", action="store_true",
                    help="fw-single steering/lift with no KTH window (instead of --window): output and stop only")
    cm.add_argument("--wheels-lifted", action="store_true",
                    help="a large run: the wheels off the floor, the truck fixed, the wires clear of turning parts")
    cm.add_argument("--vcc-watched", action="store_true", help="a large run above duty 0.3: VCC is being watched")
    cm.add_argument("--profile-tri", type=float, nargs=4, metavar=("LO", "HI", "PERIOD", "CYCLES"),
                    help="a triangle LO -> HI -> LO per PERIOD seconds, CYCLES times; |--duty| = HI, its sign")
    kt = sub.add_parser("kth", help="S2: the KTH CRC variant from a record's v3 counters (mcu-link/3 contract 3.3)")
    kt.add_argument("--record", required=True)
    kt.add_argument("--allow-synthetic", action="store_true", help="read a rehearsal record (output not_for_judgement)")
    for ax in ("m", "h"):
        kt.add_argument(f"--configured-{ax}", choices=KTH_VARIANTS,
                        help=f"the CRC variant axis {ax.upper()} is configured with in the image under test "
                             f"(required unless the axis is absent)")
    kt.add_argument("--axis-absent", action="append", default=[], choices=("M", "H"),
                    help="no KTH fitted on this axis: its counters are not read, its verdict is null; a record header "
                         "that declares its parts must name it (steer = M, lift = H)")
    kt.add_argument("--image", required=True, help="the firmware image under test (its identity string or hash)")
    ck = sub.add_parser("clock", help="G7: the MCU-to-host clock fit of a record (teleop record design 5)")
    ck.add_argument("--record", required=True)
    ck.add_argument("--allow-synthetic", action="store_true", help="read a rehearsal record (output not_for_judgement)")
    an = sub.add_parser("analyze")
    an.add_argument("--record", required=True)
    an.add_argument("--allow-synthetic", action="store_true", help="read a rehearsal record (output not_for_judgement)")
    an.add_argument("--marker", type=int, default=-1,
                    help="which event, counted from 0: 0 = the first, 1 = the second, -1 = the last (default); for "
                         "iwdg/power the operator's Enter markers, for an event the tool issued the runs of that kind "
                         "in the record")
    an.add_argument("--floor", action="store_true", help="the run was on the floor (else the stand)")
    an.add_argument("--kind", choices=("tx_cut", "disarm", "expiry", "iwdg", "power") + COMMISSION_SCENARIOS,
                    required=True)
    for p in (sc, an, cm):
        for name in ("output-response-ms", "stop-time-ms", "stop-distance-mm"):
            p.add_argument("--" + name)
    for p in (kt, ck, an):
        p.add_argument("--allow-bypassed", action="store_true",
                       help="read a record with a bypassed protection or unknown run conditions (its pass fields stay "
                            "empty, design 2026-11-25 1.2)")
    args = ap.parse_args(argv)
    if args.cmd == "commission" and args.duty is None:
        if getattr(args, "hold", None) is None:
            ap.error("the following arguments are required: --duty")
        args.duty = HOLD_DUTY_DEFAULT  # the hold's cap (plan 4.3 A1b-2 decision 10)
    kabsent: list = []
    if args.cmd in ("kth", "clock", "analyze"):
        # one read of the file for every check and the judgement (Codex S 2nd P2: no second read that a new run
        # appended in between could change)
        rows_ = [json.loads(x) for x in Path(args.record).read_text(encoding="utf-8").splitlines() if x.strip()]
        try:
            # analyze judges the run it selects (its own marks and rehearsal check, Codex PR-2b P2-4 and re-review);
            # kth and clock read the whole file
            marks = {} if args.cmd == "analyze" else refuse_or_mark(rows_, allow=args.allow_synthetic)
            rmarks = {} if args.cmd == "analyze" else judgement.record_marks(rows_)
            if args.cmd == "kth":
                # the absent axes count for eligibility, a flag on an old record too (Codex PR-A P2-3)
                kabsent = _kth_absent(rows_, args)
                parts = sorted(set(rmarks.get("parts_absent", [])) | {_AXIS_PART[ax] for ax in kabsent})
                if parts:
                    rmarks["parts_absent"] = parts
                    if any(p not in judgement.GATE_NEUTRAL_ABSENT for p in parts):
                        rmarks["not_for_gate"] = True
            jmarks = {} if args.cmd == "analyze" else judgement.check(rmarks, allow=args.allow_bypassed)
        except ValueError as exc:
            print(json.dumps({"result": "refused", "why": str(exc)}))
            return 2
        except (SyntheticRefused, judgement.NotForGate) as exc:
            print(json.dumps({"result": "refused", "why": str(exc)}))
            return 2
        marks = {**marks, **jmarks}
    if args.cmd == "kth":
        rows = rows_
        configured = {ax: v for ax, v in (("M", args.configured_m), ("H", args.configured_h)) if ax not in kabsent}
        states = [r for r in rows if r.get("t") == "state"]
        bad = next((why for r in states if (why := kth_row_problem(r)) is not None), None)
        if bad is not None:  # never the dataclass defaults (Codex v3 impl P2-2: a missing error count read as zeros)
            print(json.dumps({**marks, "result": "invalid", "why": bad, "configured": configured,
                              "image": args.image, **({"absent": kabsent} if kabsent else {})}))
            return 0 if judgement.eligible(jmarks) else 1  # an ineligible record never exits as judged (Codex P3)
        samples = [sample_from_row(r) for r in states]
        # synthetic_input is the synthetic marks' only (a large run's high_limit does not make a record synthetic)
        out = kth_check(samples, configured=configured, image=args.image,
                        synthetic_input=bool(marks.get("synthetic_inputs")), absent=kabsent)
        if not judgement.eligible(jmarks):
            # read with --allow-bypassed: the counts stay, no verdict is official (S2 reads configured_ok and the CRC
            # change reads other_variant from these fields)
            for entry in out.get("axes", {}).values():
                if not entry.get("absent"):
                    # a synthetic entry already holds its computed verdict (Codex PR-A P3)
                    entry.setdefault("computed", entry["verdict"])
                    entry["verdict"] = None
            print(json.dumps({**marks, **out}))
            return 1
        print(json.dumps({**marks, **out}))
        return 0
    if args.cmd == "clock":
        fits = clock_fits_from_rows(rows_)
        print(json.dumps({**marks, "result": "ok" if fits else "no_state_rows", "fits": [f.summary() for f in fits]}))
        return 0
    if args.cmd == "analyze":
        try:
            for name in ("output_response_ms", "stop_time_ms", "stop_distance_mm"):
                external_value(name, getattr(args, name))
        except ValueError as exc:
            print(json.dumps({"result": "refused", "why": str(exc)}))
            return 2
        out = analyze(rows_, args.marker, args.kind, {
            "output_response_ms": args.output_response_ms, "stop_time_ms": args.stop_time_ms,
            "stop_distance_mm": args.stop_distance_mm}, floor=args.floor, allow_bypassed=args.allow_bypassed,
            allow_synthetic=args.allow_synthetic)
        print(json.dumps(out, default=str))
        # refused 2; nothing to judge (no event, no marker) 1; read 0
        return {"refused": 2, "no_event": 1, "no_marker": 1}.get(out.get("result"), 0)
    try:
        synthetic = writer_marker(args.device, args.bench)
    except (SyntheticRefused, OSError, ValueError) as exc:
        print(json.dumps({"result": "refused", "why": str(exc)}))
        return 2
    args.synthetic = synthetic is not None
    # design 2026-11-25 1.2 (Codex S 7th P2): the host is given each read's return time, not the loop's start, so a
    # COMMISSION's validity is counted from when its STATE was really read -- a late read no longer stretches the
    # MCU deadline past the approved end
    # one executor per MCU device, listeners included (real safety wiring design 8.1)
    gate_holder: dict = {"host": None, "cancelled": False}
    link_factory = None
    if args.cmd == "commission" and _bench_args_given(args):
        # a bench run's link (plan 4.3 A1b-2 decision 10): every continuation of a BENCH_HOLD or a non-zero
        # COMMISSION is checked right before it is written -- the session not cancelled, the authorization generation
        # the frame was made under, its deadline -- and abandoned with the poison when one fails
        link_factory = lambda fd: BenchTtyLink(fd, gate=lambda f, write: _cli_gate(gate_holder, f, write))  # noqa: E731
    link = ReconnectingLink(args.device, baud=args.baud, clock=time.monotonic_ns, exclusive=True,
                            opener=open_raw if synthetic is None else bench_opener(open_raw, synthetic["bench"]),
                            link_factory=link_factory)
    markers = None
    if getattr(args, "markers", False):
        # an asked-for marker input that cannot work (no terminal) is said out loud and in the record, not left to
        # a record with no markers in it (rehearsal 2 R2-1)
        active, why = marker_input_state()
        markers = {"requested": True, "active": active, "why": why}
        if not active:
            print(f"bringup.py: --markers is off ({why}): no Enter marks will be recorded", file=sys.stderr)
    with open(args.out, "a", encoding="utf-8") as out:
        # every run's header (rehearsal design 2.3): the device, and the synthetic marker of a model-MCU run
        out.write(json.dumps({"t": "header", "schema": "bringup_record/1", "tool": "bringup.py", "cmd": args.cmd,
                              "device": args.device, "device_id": device_identity(args.device),
                              # the run's kind, which analyze matches a tool-issued event against (design 1.3)
                              **({"kind": args.kind, "floor": args.floor} if args.cmd == "scenario" else {}),
                              **({"cause": args.cause} if args.cmd == "clear-faults" else {}),
                              "synthetic": synthetic, "start_wall": time.time(),
                              # design 2026-11-25 1.2: the run's asked bypass and large-run arguments in its header
                              **({"bypass": _header_bypass(args),
                                  "single_axis": getattr(args, "single_axis", False),
                                  "motor_detached": getattr(args, "motor_detached", False),
                                  "sensorless": getattr(args, "sensorless", False),
                                  "wheels_lifted": getattr(args, "wheels_lifted", False),
                                  "vcc_watched": getattr(args, "vcc_watched", False),
                                  "profile_tri": getattr(args, "profile_tri", None),
                                  "duty": args.duty, "seconds": args.seconds, "axis": args.axis,
                                  "large": _header_large(args), "scenario": getattr(args, "scenario", None)}
                                 if args.cmd == "commission" else {}),
                              "start_mono_ns": time.monotonic_ns(),
                              # partial-parts plan 2026-11-25: always written, so a reader can tell "none absent"
                              # from a record made before the field
                              "parts_absent": list(_absent_of(args)),
                              **({} if markers is None else {"markers": markers})}) + "\n")
        out.flush()
        # clear-faults numbers its frames from a random start, so no STATE of an earlier run can echo them
        bench = Bench(link, out, allow_locked=args.allow_locked, commission=args.cmd == "commission",
                      product_required=_product_required(synthetic),
                      initial_seq=random.randrange(1, 0x10000) if args.cmd == "clear-faults" else None)
        gate_holder["host"] = bench.host
        bench.gate_holder = gate_holder
        try:
            if args.cmd in ("record", "cpr"):
                bench.run_until(lambda: False, args.seconds,
                                every=_marker_reader(bench) if args.cmd == "record" and args.markers else None)
                if args.cmd == "cpr":
                    res = wheel_cpr(bench.samples, (args.turns_left, args.turns_right),
                                    (args.dir_left, args.dir_right), motor_counts_confirmed=args.motor_counts_confirmed)
                    out_ = asdict(res)
                    _mark_absent_result(out_, _absent_of(args))
                    _mark_diagnostic_cpr(out_, bench.host.products_seen or [bench.host.usb_product])
                    print(json.dumps({**(OUTPUT_MARKS if synthetic else {}), **out_}))
                return 0
            if args.cmd == "clear-faults":
                result = clear_faults(bench)
                bench.emit({"t": "result", **result})
                print(json.dumps({**(OUTPUT_MARKS if synthetic else {}), **result}, default=str))
                return 0 if result["result"] == "cleared" else 1
            result = execute(args, bench)
        except DeviceBusy as busy:
            # one executor per MCU device (real safety wiring design 8.1): held at the first opening, nothing written
            refused = {"result": "refused", "why": str(busy), "holder": busy.holder}
            bench.emit({"t": "result", **refused})
            print(json.dumps({**(OUTPUT_MARKS if synthetic else {}), **refused}, default=str))
            return 2
        # a rehearsal run's result says so wherever it is copied (Codex impl review P2)
        print(json.dumps({**(OUTPUT_MARKS if synthetic else {}), **result} if isinstance(result, dict) else result,
                         default=str))
    return 0


HOLD_DUTY_DEFAULT = 0.15  # the hold's cap without --duty (11-27 measured start, lead 2026-11-28)
HOLD_SECONDS_MAX = 60.0  # a hold run's length (its continuous output stays under the run's time limit on the board)


def _bench_args_given(args) -> bool:
    given = [getattr(args, k, None) for k in ("bench_bypass", "bench_duty_max", "bench_time_max", "stop_kth_turns",
                                              "hold")]
    return any(v is not None for v in given) or bool(getattr(args, "kth_measure", False))


def _cli_gate(holder: dict, f, write) -> bool:
    """The bench CLI's send boundary (plan 4.3 A1b-2 decision 10, 2nd/3rd round P1-4): a BENCH_HOLD or a non-zero
    COMMISSION -- its first byte and every continuation -- only while the session is not cancelled, under the
    authorization generation it was made under, before its deadline; everything else (zeros, DISARM, the bench
    SET/ARM) passes."""
    host = holder.get("host")
    if getattr(f, "kind", None) == "BENCH_HOLD" or (getattr(f, "kind", None) == "COMMISSION" and f.nonzero):
        if (holder.get("cancelled") or host is None or f.generation != host.auth_epoch
                or (f.expires_ns is not None and f.expires_ns <= time.monotonic_ns())):
            return False
    write()
    return True


def _bench_ask(args) -> bl.BenchAsk | None:
    """The run's BENCH_SET (plan A1a), or None when no bench argument was given (a bench board then gets its
    defaults)."""
    if not _bench_args_given(args):
        return None
    if getattr(args, "hold", None) is not None:
        return _hold_ask(args)
    try:
        names = bl.parse_bench_bypass(getattr(args, "bench_bypass", None))
    except bl.LimitRefused as exc:
        raise Refused(str(exc)) from None
    for k in ("bench_duty_max", "bench_time_max"):
        v = getattr(args, k, None)
        if v is not None and not math.isfinite(v):
            raise Refused(f"{k.replace('_', '-')}: a finite number")
    turns = getattr(args, "stop_kth_turns", None)
    source, stop, count_sign = 0, 0, int(getattr(args, "count_sign", 1) or 1)
    if turns is not None:
        try:
            stop = bl.kth_stop_counts(turns, float(args.duty), count_sign)
        except bl.LimitRefused as exc:
            raise Refused(str(exc)) from None
        source = 1
    elif getattr(args, "kth_measure", False):
        source = 1  # measuring only (no distance stop)
    ask = bl.BenchAsk(AXES[args.axis], names, getattr(args, "bench_duty_max", None),
                      getattr(args, "bench_time_max", None), source=source, stop_counts=stop, count_sign=count_sign)
    if (why := bl.bench_refusal(ask)) is not None:
        raise Refused(why)
    return ask


def _hold_ask(args) -> bl.BenchAsk:
    """A hold run's BENCH_SET and BENCH_HOLD settings (plan 4.3 A1b-2 decision 10): steering or lift, the KTH
    source, no distance; --duty is the cap."""
    if getattr(args, "stop_kth_turns", None) is not None or getattr(args, "kth_measure", False):
        raise Refused("hold: no --stop-kth-turns or --kth-measure (a hold measures by its own count)")
    try:
        names = bl.parse_bench_bypass(getattr(args, "bench_bypass", None))
        kind, value = bl.parse_hold_target(args.hold)
    except bl.LimitRefused as exc:
        raise Refused(str(exc)) from None
    for k in ("bench_duty_max", "bench_time_max", "hold_kp", "hold_ki", "hold_kd", "hold_min_duty", "duty"):
        v = getattr(args, k, None)
        if v is not None and not math.isfinite(v):
            raise Refused(f"{k.replace('_', '-')}: a finite number")
    if not args.duty > 0:
        raise Refused("hold: --duty is the hold's duty cap, a positive number (the controller picks the sign)")
    hold = bl.HoldAsk(kind, value, kp=float(args.hold_kp), ki=float(args.hold_ki), kd=float(args.hold_kd),
                      deadband=int(args.hold_deadband), min_duty=float(args.hold_min_duty), cap=float(args.duty))
    ask = bl.BenchAsk(AXES[args.axis], names, getattr(args, "bench_duty_max", None),
                      getattr(args, "bench_time_max", None), source=1, count_sign=int(args.count_sign or 1), mode=2,
                      hold=hold)
    if (why := bl.bench_refusal(ask)) is not None:
        raise Refused(why)
    return ask


def _check_commission_args(args) -> tuple[bl.RunAsk, bl.Profile | None, tuple[str, ...]]:
    """Design ② 2 and 10 and 2026-11-25 1.2: the tool's own limits, before anything is sent. Returns what the run
    asks, its profile and its bypass names."""
    axis = AXES[args.axis]
    try:
        names = bl.parse_bypass(getattr(args, "bypass", None))
    except bl.LimitRefused as exc:
        raise Refused(str(exc)) from None
    bench = _bench_ask(args)
    if bench is not None and (getattr(args, "single_axis", False) or getattr(args, "scenario", None) is not None
                              or getattr(args, "profile_tri", None) is not None):
        raise Refused("bench: a plain run only (no --single-axis, --scenario or --profile-tri)")
    if names and axis >= 2:
        raise Refused("bypass: the encoder early stop is a wheel's (L or R)")
    # fw-single (partial-parts plan 2026-11-25 PR-B): steering and lift only with the motor off the mechanism, with
    # a hand-checked KTH window or none (--sensorless); both are bypasses of the firmware's sensor checks
    single = getattr(args, "single_axis", False)
    detached, sensorless = getattr(args, "motor_detached", False), getattr(args, "sensorless", False)
    if (detached or sensorless) and not single:
        raise Refused("motor-detached/sensorless: only with --single-axis (the fw-single image)")
    if (detached or sensorless) and axis < 2:
        raise Refused("motor-detached/sensorless: steering or lift only")
    if single and axis >= 2:
        if not detached:
            raise Refused("single-axis: steering and lift run only with the motor off the mechanism "
                          "(--motor-detached): with the sensor checks off nothing guards the end stops")
        if sensorless == (getattr(args, "window", None) is not None):
            raise Refused("single-axis: steering and lift need either --window LO HI (KTH fitted, checked by hand) "
                          "or --sensorless")
        names = names + bl.SINGLE_BYPASS_NAMES[: 2 if sensorless else 1]
    profile = None
    tri = getattr(args, "profile_tri", None)
    if tri is not None:
        lo, hi, period, cycles = tri
        if not all(math.isfinite(v) for v in tri):
            raise Refused("profile: LO, HI, PERIOD and CYCLES must be finite numbers")
        if axis >= 2 or getattr(args, "scenario", None) is not None:
            raise Refused("profile: a wheel's plain run only")
        if args.seconds is not None:
            raise Refused("profile: its length is PERIOD x CYCLES -- give no --seconds")
        if int(cycles) != cycles:
            raise Refused("profile: CYCLES is a whole number")
        if abs(abs(args.duty) - hi) > 1e-9:
            raise Refused("profile: |--duty| must equal HI (its sign is every piece's)")
        try:
            profile = bl.triangle(lo, hi, period, int(cycles), 1 if args.duty > 0 else -1)
        except bl.LimitRefused as exc:
            raise Refused(str(exc)) from None
        if (why := bl.slope_problem(profile.pieces)) is not None:
            raise Refused(why)
    if not args.power_switch_in_hand:
        # the commissioning build arms without an e-stop pin: a person holds the main power switch (firmware design
        # 2.1, Q3); --yes does not stand in for it (Codex design P2-5)
        raise Refused("power switch: confirm a person holds the main power switch (--power-switch-in-hand)")
    if getattr(args, "scenario", None) == "duty_disarm":
        # design 2026-11-25 1.3: a wheel driven for --seconds (ramp included) and DISARMed by the tool; the large-run
        # rules apply as to a plain run of that duty and length
        if axis >= 2:
            raise Refused("scenario: a wheel only (steering and lift would run into their mechanical limits)")
        if args.window is not None:
            raise Refused("window: a wheel takes no window")
        if args.seconds is None or args.duty == 0:
            raise Refused("duty_disarm: give a nonzero --duty and --seconds (the approved output time, ramp included)")
        if not (math.isfinite(args.duty) and math.isfinite(args.seconds)):
            raise Refused("duty_disarm: --duty and --seconds must be finite numbers")
        if abs(args.duty) > bl.WHEEL_DUTY_MAX_OPEN:
            raise Refused(f"duty_disarm: |duty| must be at most {bl.WHEEL_DUTY_MAX_OPEN:g}")
        if abs(int(round(args.duty * 10000))) < dd.MIN_DUTY_E4:
            # a target within one e4 of zero would take a zero output for the hold (Codex PR-2b P1-1)
            raise Refused(f"duty_disarm: |duty| must be at least {dd.MIN_DUTY_E4 / 1e4:g}")
        need = dd.min_seconds(args.duty)
        if args.seconds < need:
            raise Refused(f"seconds: duty_disarm needs at least {need:.3f} s at duty {abs(args.duty):g} to show the "
                          "0.5 s hold in STATEs before its DISARM")
        ask = bl.RunAsk(axis, args.duty, args.seconds)
        if bl.is_large(ask):
            why = bl.large_refusal(ask, wheels_lifted=getattr(args, "wheels_lifted", False),
                                   vcc_watched=getattr(args, "vcc_watched", False),
                                   synthetic=getattr(args, "synthetic", False))
            if why is not None:
                raise Refused(why)
        elif abs(args.duty) > COMMISSION_DUTY_MAX:
            raise Refused(f"duty must be within +-{COMMISSION_DUTY_MAX}")
        return ask, None, names
    if getattr(args, "scenario", None) is not None:
        if not 0 < abs(args.duty) <= COMMISSION_DUTY_MAX:
            raise Refused(f"duty must be nonzero and within +-{COMMISSION_DUTY_MAX}")
        if axis >= 2:
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
        return bl.RunAsk(axis, args.duty, args.limit_s, args.scenario), None, names
    # a profile's longest non-zero output: its pieces, then up to HANDOVER_S of command validity past its end until the
    # zero lands, and the command's own validity if that zero is lost (Codex PR-1 re-review P2-1)
    seconds = (profile.duration_s + HANDOVER_S + bl.PROFILE_VALIDITY_S) if profile is not None else args.seconds
    if seconds is None:
        raise Refused("seconds: required without --scenario")
    peak = (profile.max_abs_e4 / 10000.0 if profile is not None else abs(args.duty)) * (1 if args.duty > 0 else -1)
    ask = bl.RunAsk(axis, peak, seconds)
    if bench is not None and axis >= 2:
        # steering and lift under the run's own limits (the image's 0.15 / 2 s when not given), the tool's hold a
        # margin under the board's time limit
        duty_lim = bench.duty_max if bench.duty_max is not None else COMMISSION_DUTY_MAX
        t_lim = (bench.time_max_s if bench.time_max_s is not None
                 else bl.BENCH_DEFAULT_TIME_MS[axis] / 1000.0) - bl.TIME_MARGIN_S
        if not 0 < abs(ask.duty) <= duty_lim + 1e-12:
            raise Refused(f"duty must be nonzero and within +-{duty_lim:g} (the bench run's limit)")
        if bench.hold_run:
            # a hold's length is not its continuous output (the board's time limit judges that): 1.2 s for the
            # settling verdict, at most HOLD_SECONDS_MAX
            if not 1.2 <= seconds <= HOLD_SECONDS_MAX:
                raise Refused(f"seconds: a hold runs 1.2 .. {HOLD_SECONDS_MAX:g} s")
        elif not COMMISSION_HOLD_MIN_S <= seconds <= t_lim:
            raise Refused(f"seconds must be in [{COMMISSION_HOLD_MIN_S}, {t_lim:g}] (the bench run's time limit less "
                          f"{bl.TIME_MARGIN_S} s)")
        if (abs(ask.duty) > bl.VCC_WATCH_ABOVE + 1e-12 and not getattr(args, "vcc_watched", False)
                and not getattr(args, "synthetic", False)):
            raise Refused(f"vcc: above duty {bl.VCC_WATCH_ABOVE} watch VCC (--vcc-watched) -- steering and lift too")
        range_off = (axis == 2 and "steer_range" in bench.names) or (axis == 3 and "lift_range" in bench.names)
        if range_off and args.window is not None:
            raise Refused("window: the range is off for this run -- give no --window (it stops by time)")
        if not range_off and not sensorless and (args.window is None or not args.window[0] < args.window[1]):
            raise Refused("window: steering and lift need --window LO HI with LO < HI")
        return ask, profile, names + tuple(f"bench:{n}" for n in bench.names)
    if bl.is_large(ask):
        why = bl.large_refusal(ask, wheels_lifted=getattr(args, "wheels_lifted", False),
                               vcc_watched=getattr(args, "vcc_watched", False),
                               synthetic=getattr(args, "synthetic", False))
        if why is not None:
            raise Refused(why)
        if not COMMISSION_HOLD_MIN_S <= seconds:
            raise Refused(f"seconds must be at least {COMMISSION_HOLD_MIN_S}")
        if args.duty == 0:
            raise Refused("duty must be nonzero")
    else:
        # a profile's effective peak (its pieces), not its nominal HI (Codex PR-1 P3)
        if not 0 < abs(ask.duty) <= COMMISSION_DUTY_MAX:
            raise Refused(f"duty must be nonzero and within +-{COMMISSION_DUTY_MAX}")
        hold_max = AXIS_HOLD_MAX_S.get(axis, COMMISSION_HOLD_MAX_S)
        if single and detached and axis >= 2:
            hold_max = bl.SINGLE_DETACHED_HOLD_MAX_S  # a detached motor: the firmware's 2 s, kept a margin under
        if not COMMISSION_HOLD_MIN_S <= seconds <= hold_max:
            raise Refused(f"seconds must be in [{COMMISSION_HOLD_MIN_S}, {hold_max}] for axis {args.axis}")
    if axis >= 2 and not sensorless:
        if args.window is None or not args.window[0] < args.window[1]:
            raise Refused("window: steering and lift need --window LO HI with LO < HI")
    elif axis < 2 and args.window is not None:
        raise Refused("window: a wheel takes no window")
    if bench is not None:
        # a wheel: the run's own limits may only be lower than the image's (1.0, the validator's ms)
        if bench.duty_max is not None and abs(ask.duty) > bench.duty_max + 1e-12:
            raise Refused(f"duty must be within +-{bench.duty_max:g} (the bench run's limit)")
        if bench.time_max_s is not None and seconds > bench.time_max_s - bl.TIME_MARGIN_S:
            raise Refused(f"seconds must be at most {bench.time_max_s - bl.TIME_MARGIN_S:g} (the bench run's limit)")
        names = names + tuple(f"bench:{n}" for n in bench.names)
    return ask, profile, names


def _mark_commission_result(result: dict, names: tuple, large: bool, product: str | None = None,
                            absent: tuple = ()) -> None:
    """Design 2026-11-25 1.2 (PR-2b): a commissioning result with a bypassed protection, or armed on a known
    diagnostic image (Codex PR-2b P2-5), is not evidence for a gate -- marked, its official pass fields (the S3 sign
    verdict, metrics.valid, external passes) left empty; a large run is marked but judged."""
    if large:
        result["high_limit"] = True
    diag = judgement.diagnostic_image(product)
    absent = tuple(sorted(set(absent)))
    if absent:
        result["parts_absent"] = list(absent)
    gate_absent = any(p not in judgement.GATE_NEUTRAL_ABSENT for p in absent)
    if not names and not diag and not gate_absent:
        return
    result["not_for_gate"] = True
    if names:
        result["bypassed"] = list(names)
    if diag:
        result["diagnostic_image"] = product
    if isinstance(result.get("verdict"), dict):
        # no duty_sign proposal either (partial-parts plan v4): a bypassed or partial run never rewrites the build's
        result["verdict"] = {**result["verdict"], "duty_sign_ok": None, "sensor_sign_ok": None,
                             "suggested_duty_sign": None}
    if isinstance(result.get("metrics"), dict):
        result["metrics"] = {**result["metrics"], "valid": None}
    if isinstance(result.get("external"), dict):
        result["external"] = judgement.empty_pass_fields(result["external"])


def _absent_of(args) -> tuple:
    return tuple(sorted(set(getattr(args, "absent", None) or ())))


def _mark_diagnostic_cpr(result: dict, products) -> None:
    """A cpr measured with any opening on a diagnostic image (the bench image included, plan A1a) keeps its computed
    values but no official pass: S2 reads ``valid``."""
    products = [products] if isinstance(products, str) or products is None else list(products)
    diag = [p for p in products if judgement.diagnostic_image(p)]
    if not diag:
        return
    result["not_for_gate"] = True
    result["board_product"] = diag[0]
    if "valid" in result and "computed_valid" not in result:
        result["computed_valid"] = result["valid"]
    result["valid"] = None


def _mark_absent_result(result: dict, absent: tuple) -> None:
    """Partial-parts plan v4 (Codex PR-A P2-2): a step, scenario or cpr result of a run with steer or wheels absent
    keeps its measured values but no official pass (external passes, cpr valid -> computed_valid)."""
    if not absent:
        return
    result["parts_absent"] = list(absent)
    if all(p in judgement.GATE_NEUTRAL_ABSENT for p in absent):
        return
    result["not_for_gate"] = True
    if isinstance(result.get("external"), dict):
        result["external"] = judgement.empty_pass_fields(result["external"])
    if "valid" in result:
        result["computed_valid"], result["valid"] = result["valid"], None


def _product_required(synthetic) -> bool:
    """Every real device needs a readable product string on an open commissioning board, whatever --baud says: the
    port speed is set on USB CDC too, so it is no evidence of UART7 (Codex PR-B round 2). Only the rehearsal bench's
    model (a pty, no USB) has none by nature."""
    return synthetic is None


def _header_bypass(args) -> str | None:
    """The run's bypass names for its header, the tool-derived fw-single ones included (partial-parts plan v4: in the
    record before the ARM); the typed --bypass when the arguments are refused anyway."""
    try:
        _ask, _profile, names = _check_commission_args(args)
    except Refused:
        return getattr(args, "bypass", None)
    return ",".join(names) or None


def _header_large(args) -> bool | None:
    """The run's large-run verdict for its record header, before anything is connected (None: its arguments are
    refused anyway; the board's product and lim come in the pre_arm_checked row once the link is open)."""
    try:
        ask, _profile, _names = _check_commission_args(args)
    except Refused:
        return None
    return bl.is_large(ask)


def _tty_line(prompt: str) -> str:
    """One line typed on a real terminal (design 2026-11-25 1.2: no pipe stands in for the operator)."""
    if not sys.stdin.isatty():
        raise Refused("large run: the confirmation is typed on a terminal (stdin is not a TTY)")
    print(prompt, end="", flush=True)
    return sys.stdin.readline().strip()


def _confirm_large(ask: bl.RunAsk) -> None:
    """Before and whatever --yes: the operator retypes what the large run will do."""
    phrase = bl.confirmation_phrase(ask)
    answer = _tty_line(f"LARGE RUN on axis {'LRMH'[ask.axis]}: {phrase}. Type exactly '{phrase}': ")
    if answer != phrase:
        raise Refused("large run: the operator did not retype the run")


def _confirm_detached(axis: str) -> None:
    """fw-single steering/lift: the operator retypes that the motor is off the mechanism."""
    phrase = f"motor {axis} detached, clamped alone, nothing near the shaft"
    answer = _tty_line(f"SENSOR CHECKS OFF on axis {axis}: {phrase}. Type exactly '{phrase}': ")
    if answer != phrase:
        raise Refused("motor-detached: the operator did not retype it")


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
        session = CommissionSession(bench, AXES[args.axis], None if args.window is None else tuple(args.window),
                                    single_axis=getattr(args, "single_axis", False),
                                    sensorless=getattr(args, "sensorless", False))
    else:
        session = Session(bench, _cap(), floor=getattr(args, "floor", False),
                          power_switch_reachable=getattr(args, "power_switch_reachable", False))
    result: dict = {"result": "aborted", "why": "not run"}
    names: tuple = ()
    conditions: dict | None = None
    try:
        for name in ("output_response_ms", "stop_time_ms", "stop_distance_mm"):
            try:
                external_value(name, getattr(args, name, None))  # before anything is armed
            except ValueError as exc:
                raise Refused(str(exc)) from None
        if args.cmd == "commission":
            ask, profile, names = _check_commission_args(args)
            session.large = bl.is_large(ask)
            session.duty_max = bl.WHEEL_DUTY_MAX_OPEN if session.large else COMMISSION_DUTY_MAX
            session.ignore = bl.ignored_bits(names, AXES[args.axis])
            session.scenario = getattr(args, "scenario", None)
            session.bench = _bench_ask(args)  # the bench image's run settings (plan A1a)
            if session.bench is not None and session.bench.duty_max is not None:
                session.duty_max = (session.bench.duty_max if AXES[args.axis] >= 2
                                    else min(session.duty_max, session.bench.duty_max))
            conditions = {"bypass": list(names), "large": session.large,
                          "bench": None if session.bench is None else {
                              "bypass": list(session.bench.names), "duty_max": session.bench.duty_max,
                              "time_max_s": session.bench.time_max_s},
                          "asked": {"duty": ask.duty, "seconds": ask.seconds},
                          "profile_tri": None if profile is None else list(args.profile_tri)}
            bench.emit({"t": "run_conditions", **conditions})
            if session.large:
                _confirm_large(ask)  # before --yes is looked at (design 2026-11-25 1.2)
            if getattr(args, "motor_detached", False):
                _confirm_detached(args.axis)  # retyped on a terminal, whatever --yes (partial-parts plan v4)
                bench.emit({"t": "confirmed", "what": "motor_detached", "axis": args.axis})
            if getattr(args, "scenario", None) == "duty_disarm":
                what = (f"commission scenario duty_disarm: will ARM and drive wheel {args.axis} at duty {args.duty}, at "
                        f"most {args.seconds} s, until it has held that duty 0.5 s, then DISARM and watch it stop.")
            elif getattr(args, "scenario", None) is not None:
                what = (f"commission scenario {args.scenario}: will ARM and drive wheel {args.axis} at duty {args.duty} "
                        f"up to the build's time limit ({args.limit_s} s, giving up at {args.limit_s + LIMIT_SLACK_S} s),"
                        f" then DISARM {'at once' if args.scenario == 'limit_disarm' else f'{args.disarm_delay_ms} ms in'}"
                        f" (build stop_grace_ms {args.stop_grace_ms}).")
            else:
                what = (f"commission: will ARM and drive axis {args.axis} at duty {ask.duty} for at most "
                        f"{ask.seconds} s (window {args.window}"
                        f"{'' if profile is None else f', triangle {list(args.profile_tri)}'}"
                        f"{'' if not names else f', checks off: {list(names)}'}"
                        f"{', single-axis image: the other channels stay cut' if getattr(args, 'single_axis', False) else ''}).")
            _confirm(args, what + " Outputs on only after the IPROPI protection is confirmed (user decision); "
                                  "the window inside the mechanical limits.")
            if getattr(args, "markers", False):
                # after the confirmation: the reader reads stdin and would take the operator's 'yes' line
                bench.poll = _marker_reader(bench)
                print("markers: press Enter (optionally a label first) at each event -- the main power cut, an NRST "
                      "reset -- while the wheel turns", file=sys.stderr, flush=True)
            if getattr(args, "scenario", None) == "duty_disarm":
                result = run_duty_disarm(bench, session, args.duty, args.seconds,
                                         synthetic=getattr(args, "synthetic", False))
                ext = verdict("duty_disarm", {
                    "output_response_ms": args.output_response_ms, "stop_time_ms": args.stop_time_ms,
                    "stop_distance_mm": args.stop_distance_mm})
                v = result.get("validity") or {}
                if not (v.get("valid") and v.get("physical")) or result.get("result") != "ok":
                    # no physical pass from an invalid test, a board that drives no motor (locked, the model), or a
                    # run that was aborted or whose stop was not confirmed (Codex PR-2b P1-3, P2-2)
                    ext = judgement.empty_pass_fields(ext)
                result["external"] = ext
                result.update(conditions, cmd="commission", axis=args.axis, power_switch_in_hand=True,
                              board_product=session.preflight_product)
            elif getattr(args, "scenario", None) is not None:
                result = run_commission_scenario(bench, session, args.scenario, args.duty, args.limit_s,
                                                 args.disarm_delay_ms, args.stop_grace_ms)
                # locked boards and the model only: the sequence is shown, no physical pass (Codex PR-2b P1-3)
                result["external"] = judgement.empty_pass_fields(verdict(args.scenario, {
                    "output_response_ms": args.output_response_ms, "stop_time_ms": args.stop_time_ms,
                    "stop_distance_mm": args.stop_distance_mm}))
                result.update(conditions, cmd="commission", axis=args.axis, power_switch_in_hand=True,
                              board_product=session.preflight_product)
            elif session.bench is not None and session.bench.hold_run:
                result = run_hold(bench, session, args.seconds)
                result.update(conditions, cmd="commission", axis=args.axis, power_switch_in_hand=True,
                              board_product=session.preflight_product)
                if session.bench_taken is not None:
                    result["bench_set"] = session.bench_taken
            else:
                result = run_commission(bench, session, args.duty, args.seconds, profile)
                result.update(conditions, board_product=session.preflight_product)
                if session.bench_taken is not None:
                    result["bench_set"] = session.bench_taken  # what the board took, by its seq
                    result["bench_run"] = session.bench_result()
                if getattr(args, "sensorless", False):
                    # no KTH: the run shows the output accepted and the stop confirmed, never a position or a sign
                    # (partial-parts plan v4)
                    result["position"] = "none (sensorless: output accepted / stop confirmed only)"
                if session.arm_requested and session.cleanup_confirmed is not True:
                    # the stop again before the operator is asked anything: a DISARM lost on a disconnect is re-sent
                    # now, not after the direction prompt (Codex PR-2a re-reviews 5-6); a failure here is kept with
                    # the result already measured
                    try:
                        session.cleanup()
                    except BaseException as exc:  # noqa: BLE001 -- Ctrl-C too: the measured result stays, the final
                        # cleanup in execute()'s finally retries (Codex PR-2a re-review 7 P3)
                        session.cleanup_confirmed = False
                        session.aborted = session.aborted or "exception"
                        result["cleanup_error"] = f"{type(exc).__name__}: {exc}"
                        result["result"] = "aborted"
                # still no confirmed stop: nobody is asked to look at a wheel that may not have stopped
                result["observed"] = _observed(args) if session.cleanup_confirmed else "unknown"
                m = result["metrics"]
                # an unjudgeable run (its acceptances could not be told apart) gives no sign verdict (Codex PR-2a P2-3)
                result["verdict"] = None if result["result"] == "unjudgeable" else sign_verdict(
                    m["duty_e4"], m["measured_sign"], result["observed"], duty_sign_in_build=m["duty_sign_in_build"])
                if getattr(args, "sensorless", False) and result["verdict"] is not None:
                    # no sensor: no sign of any kind (partial-parts plan v4)
                    result["verdict"] = {"duty_sign_ok": None, "sensor_sign_ok": None, "suggested_duty_sign": None,
                                         "note": "sensorless"}
                result.update(cmd="commission", axis=args.axis, power_switch_in_hand=True)
        elif args.cmd == "step":
            result = run_step(bench, session, args.wheel, args.target)
        else:
            result = run_scenario(bench, session, args.kind, args.rate)
            result["external"] = verdict(args.kind, {
                "output_response_ms": args.output_response_ms, "stop_time_ms": args.stop_time_ms,
                "stop_distance_mm": args.stop_distance_mm}, floor=args.floor)
            if getattr(args, "synthetic", False) or not _tool_run_official(
                    args.kind, result, [state_row(smp) for smp in bench.samples]):
                # the same qualification analyze asks of the record, a rehearsal never physical (Codex PR-2b
                # re-reviews 3-4)
                result["external"] = judgement.empty_pass_fields(result["external"])
    except Refused as exc:
        result = {"result": "refused", "why": str(exc)}
    except DeviceBusy:
        raise  # held at the first opening: nothing was written, the caller refuses (design 8.1), never an abort
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
    if args.cmd == "commission" and conditions is not None:
        # the run's conditions and marks on every result once its arguments were taken, an exception's or an
        # abort's too (Codex PR-2b P3)
        for k, v in conditions.items():
            result.setdefault(k, v)
        _mark_commission_result(result, names, session.large, session.preflight_product, _absent_of(args))
    elif args.cmd in ("step", "scenario"):
        _mark_absent_result(result, _absent_of(args))
    result["record_errors"] = bench.record_errors
    bench.emit({"t": "result", **result})
    return result


if __name__ == "__main__":
    sys.exit(main())
