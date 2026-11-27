#!/usr/bin/env python3
"""The bench console's back end (docs/plans/2026-11-27-bench-console.md 3, 3.1 B1): the operator console served on
the Jetson for the bench image (``lim=lr1 bench5``), one board writer, the shared lease, STOP and request ledger, and a
run engine that drives one axis by duty for a time (A1a). Hold, distance stops and the MT source come with B2.

usage: bench_console.py (--device /dev/forklift-mcu [--baud B] | --model PATH) [--host 127.0.0.1] [--port 8765]
                        [--console-lan --console-origin-host H] [--web-root console/web] [--runs-root ~/bench_runs]
                        [--duration-s S]

One control thread does everything in order each cycle: the emergency batch and the requests (``BenchLink``), the
board's bytes, the run engine, then the writes (``BenchTtyLink`` behind the send boundary). The server thread only
puts messages in the channels. The board is opened under the executor lock and refused unless its USB product string
is exactly the bench image; the REAL operating console must not run beside this tool (same lock).

Each run has its own exclusive record directory ``<runs-root>/<mode>_<axis>_<time>/`` with ``run.jsonl`` (a row per
accepted STATE, request, frame and event; a write error cancels the run) and ``result.json``.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import secrets
import sys
import time
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))  # console/
sys.path.insert(0, str(Path(__file__).resolve().parent))

import bringup  # noqa: E402
from forklift_core.live.bench_link import BenchChannels, BenchLink  # noqa: E402
from forklift_core.live.real_channels import send_boundary  # noqa: E402
from forklift_core.mcu_link.build_policy import Client  # noqa: E402
from forklift_core.mcu_link.messages import (  # noqa: E402
    BenchDriveState, BenchStop, FaultFlag, LinkDecoder, Mode, RejectReason, StateFlag)
from forklift_core.plant import bench_limits as bl  # noqa: E402
from forklift_core.plant import bench_session as bs  # noqa: E402
from forklift_core.plant.bringup import Sample  # noqa: E402
from forklift_core.plant.mcu_host import McuLinkHost  # noqa: E402
from forklift_core.plant.record_rows import state_row  # noqa: E402
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig  # noqa: E402

CYCLE_S = 0.002
STREAM_S = 0.05
SEND_EVERY_S = 0.01  # a non-zero duty is refreshed this often (bringup's hold)
RESEND_ZERO_S = 0.05
READY_WAIT_S = 3.0  # a run's board must show DISARMED by then (BOOT, a brake running out)
PAIR_WAIT_S = 1.0
SET_WAIT_S = 1.0
ARM_WAIT_S = 2.0
NAMED_WAIT_S = 0.5
PREP_WAIT_S = 0.5
STOP_WAIT_S = bringup.STOPPING_WAIT_S + 1.0 + bringup.POST_ZERO_S  # the brake grace + 1 s, after the zero lands
CLEAR_ECHO_S = bringup.CLEAR_ACCEPT_S
CLEAR_READY_S = bringup.CLEAR_READY_S
# the board's command_timeout_ms (board_config.c, pinned by a unit test): an ARM's echo older than this is refused
BOARD_COMMAND_TIMEOUT_MS = 150
LOOPBACK = ("127.0.0.1", "localhost", "::1")
AXES = "LRMH"
RUN_KINDS = ("BENCH_SET", "BENCH_ARM", "COMMISSION")
A1A_BYPASS = ("steer_range", "lift_range")  # the MCU bits this image has (A1a); kth_invalid is A2's
STOPPED = (Mode.DISARMED, Mode.FAULT, Mode.ESTOP)


class Refused(Exception):
    pass


def welcome(epoch: int, product: str | None) -> dict:
    return {"t": "welcome", "api": "console/1", "vehicle_id": "bench", "session_epoch": epoch, "backend": "real",
            "session": "bench", "product": product, "capabilities": {
                "control": True, "mission": False, "teleop_direct": False, "teleop_video": False, "lift": False,
                "recovery_actions": [], "sim_reset": False, "bench": True,
                "feedback": {"rear_axle_speed": False, "steering": True, "lift_height": True, "load_state": False},
                "media": {}, "provisional_limits": True}}


# --- the run record --------------------------------------------------------------------------------------------------
class RecordError(Exception):
    pass


class RunRecord:
    """One run's directory, created exclusively; ``run.jsonl`` flushed per row (an error raises ``RecordError``) and
    ``result.json`` written through a temporary file."""

    def __init__(self, root: Path, mode: str, axis: str, *, wall=time.time) -> None:
        root = Path(root).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(wall()))
        base = f"{mode}_{axis}_{stamp}"
        for k in range(100):
            d = root / (base if k == 0 else f"{base}_{k}")
            try:
                d.mkdir()  # exclusive: two runs in one second get their own directories
                break
            except FileExistsError:
                continue
        else:
            raise OSError(f"no free record directory under {root}")
        self.dir = d
        self._f = open(d / "run.jsonl", "x", encoding="utf-8")

    def emit(self, row: dict) -> None:
        try:
            self._f.write(json.dumps(row, default=str) + "\n")
            self._f.flush()
        except (OSError, ValueError) as exc:
            raise RecordError(str(exc)) from exc

    def result(self, obj: dict) -> None:
        tmp = self.dir / "result.json.tmp"
        with open(tmp, "x", encoding="utf-8") as f:
            json.dump(obj, f, indent=1, default=str)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.dir / "result.json")

    def close(self) -> None:
        try:
            self._f.close()
        except OSError:
            pass


# --- the request at the web boundary ---------------------------------------------------------------------------------
def _flag(msg: dict, key: str) -> bool:
    v = msg.get(key, False)
    if not isinstance(v, bool):
        raise Refused(f"{key}: true or false")
    return v


def _num(msg: dict, key: str, *, required: bool) -> float | None:
    v = msg.get(key)
    if v is None:
        if required:
            raise Refused(f"{key}: required")
        return None
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise Refused(f"{key}: a finite number")
    return float(v)


@dataclass
class Ask:
    axis: int
    run: object  # bl.RunAsk
    bench: object  # bl.BenchAsk
    names: tuple
    window: tuple | None
    range_off: bool
    large: bool
    source: str
    physical_setup: str | None
    absent: tuple


def parse_run(msg: dict) -> Ask:
    """The web request checked as the CLI's arguments are (plan 3.1): types first, then bringup's own limits; the
    confirmations come only from the request's own booleans; ``synthetic`` and the product are never taken from it."""
    axis_name = msg.get("axis")
    if axis_name not in tuple(AXES):
        raise Refused("axis: one of L, R, M, H")
    axis = AXES.index(axis_name)
    mode = msg.get("mode", "duty")
    if mode == "hold":
        raise Refused("not_yet_a1b: hold comes with the KTH counter (A1b)")
    if mode != "duty":
        raise Refused("mode: duty")
    if msg.get("stop"):
        raise Refused("not_yet_a1b: the distance stop comes with A1b/A1c")
    source = msg.get("source", "kth" if axis >= 2 else "none")
    if source == "mt":
        raise Refused("not_yet_a1c: the MT source comes with A1c")
    if source not in ("kth", "none") or (source == "kth" and axis < 2):
        raise Refused("source: kth for steering or lift, none for any axis")
    bypass = msg.get("bypass", [])
    if not isinstance(bypass, list) or not all(isinstance(n, str) for n in bypass):
        raise Refused("bypass: a list of names")
    if "kth_invalid" in bypass:
        raise Refused("not_yet_a2: kth_invalid comes with the second bypass bits (A2)")
    unknown = [n for n in bypass if n not in A1A_BYPASS]
    if unknown:
        raise Refused(f"bypass: unknown {unknown}; this image has {list(A1A_BYPASS)}")
    absent = msg.get("absent", [])
    if not isinstance(absent, list) or not all(a in tuple(AXES) for a in absent):
        raise Refused("absent: a list of axis letters")
    if axis_name in absent:
        raise Refused(f"absent: axis {axis_name} is marked absent -- nothing to drive")
    window = msg.get("window")
    if window is not None:
        if not (isinstance(window, list) and len(window) == 2 and all(
                isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in window)):
            raise Refused("window: two finite numbers")
        window = (float(window[0]), float(window[1]))
    setup = msg.get("physical_setup")
    if setup is not None and not isinstance(setup, str):
        raise Refused("physical_setup: text")
    large_confirm = msg.get("large_confirm")
    if large_confirm is not None and not isinstance(large_confirm, str):
        raise Refused("large_confirm: text")
    ns = argparse.Namespace(
        cmd="commission", axis=axis_name, duty=_num(msg, "duty", required=True),
        seconds=_num(msg, "seconds", required=True), window=window, bypass=None,
        bench_bypass=",".join(bypass),  # never None: every run builds its BenchAsk (the same limits for [] and none)
        bench_duty_max=_num(msg, "bench_duty_max", required=False),
        bench_time_max=_num(msg, "bench_time_max", required=False),
        power_switch_in_hand=_flag(msg, "power_switch_in_hand"), wheels_lifted=_flag(msg, "wheels_lifted"),
        vcc_watched=_flag(msg, "vcc_watched"), synthetic=False, single_axis=False, motor_detached=False,
        sensorless=False, profile_tri=None, scenario=None, limit_s=None, stop_grace_ms=None, disarm_delay_ms=0)
    try:
        run, _profile, names = bringup._check_commission_args(ns)
        bench = bringup._bench_ask(ns)
    except bringup.Refused as exc:
        raise Refused(str(exc)) from None
    large = bl.is_large(run)
    if large and large_confirm != bl.confirmation_phrase(run):
        raise Refused(f"large run: large_confirm must be exactly {bl.confirmation_phrase(run)!r}")
    range_off = (axis == 2 and "steer_range" in bench.names) or (axis == 3 and "lift_range" in bench.names)
    return Ask(axis, run, bench, tuple(names), window, range_off, large, source, setup, tuple(absent))


# --- the run engine ---------------------------------------------------------------------------------------------------
@dataclass
class Run:
    request_id: str
    ask: Ask
    rec: RunRecord
    lease_id: str
    owner_conn: str | None
    stops0: int
    control_gen: int
    started_ns: int
    phase: str = "recover"
    phase_ns: int = 0
    t_stop_us: int | None = None
    run_id: int | None = None
    set_seq: int | None = None
    set_echo_us: int | None = None
    boot: int | None = None
    conn: int | None = None
    arm_seq: int | None = None
    arm_written_ns: int | None = None
    arm_echo_us: int | None = None  # the STATE the BENCH_ARM echoed (its acceptance window starts there)
    arm_prev_gen: int | None = None
    bench_taken: dict | None = None
    prep_seq: int | None = None
    drive_start_ns: int | None = None
    drive_end_ns: int | None = None
    last_send_ns: int = 0
    zero_seqs: list = field(default_factory=list)  # every stop zero queued, with the echo time it carried
    zero_seq: int | None = None  # the first stop zero actually written: the one the board's answer is read against
    zero_link: tuple | None = None
    zero_written_ns: int | None = None
    zero_echo_us: int | None = None
    zero_sent_ns: int = 0
    disarm_seq: int | None = None  # this run's DISARM (re-issued if it was dropped or abandoned unwritten)
    disarm_written_ns: int | None = None
    cancel: str | None = None
    window_stop: str | None = None
    ended_seen: bool = False
    cleanup_confirmed: bool = False
    link_reopened: bool = False
    stop_entered_ns: int | None = None
    aborted: str | None = None
    previous_result: dict | None = None
    record_errors: int = 0
    faults_at_drive: int = 0
    nonzero_written: bool = False
    peak_current_ma: list = field(default_factory=lambda: [0, 0, 0, 0])


@dataclass
class Clear:
    request_id: str
    cause: str
    binding: bs.ClearBinding
    rec: RunRecord
    started_ns: int
    floor_us: int
    phase: str = "disarm"
    phase_ns: int = 0
    disarm_seq: int | None = None
    clear_seq: int | None = None
    before: dict = field(default_factory=dict)


def _latch_view(st) -> dict:
    return {"mode": st.mode.name, "fault_flags": [f.name for f in FaultFlag if f in st.fault_flags],
            "driver_status": [f"{b:02X}" for b in st.driver_status],
            "drivers_ready": bool(st.flags & StateFlag.DRIVERS_READY), "reject_reason": st.reject_reason.name}


def _finite_or_none(v):
    return float(v) if isinstance(v, (int, float)) and math.isfinite(v) else None


class BenchConsole:
    """The session: the arbiter's lease and ledger, the board host, one run or one clear at a time."""

    def __init__(self, device, *, runs_root: Path, clock=time.monotonic_ns, epoch: int | None = None,
                 initial_seq: int | None = None) -> None:
        self.clock = clock
        self.runs_root = Path(runs_root).expanduser()
        self.epoch = epoch if epoch is not None else secrets.randbits(48)
        self.arbiter = Arbiter(self.epoch, ArbiterConfig(bench_requests=True, request_tokens=True),
                               lambda: secrets.token_hex(16))
        self.arbiter.bench_request = self._on_request
        self.arbiter.on_disarm = self._on_shared_disarm
        self.channels = BenchChannels()
        self.link = BenchLink(self.arbiter, self.channels)
        self.host = McuLinkHost(bringup.COMMISSION_HOST_CFG, client=Client.BRINGUP, commission=True,
                                log=self._host_log, on_state=self._on_state,
                                initial_seq=secrets.randbelow(0x10000) if initial_seq is None else initial_seq)
        self.dev = device
        self.serial = 0
        self.run: Run | None = None
        self.clear: Clear | None = None
        self.last_result: dict | None = None
        self.conn = 0  # board openings seen
        self.connected_ns: int | None = None
        self.boundary_from: tuple | None = None  # (boot_id, conn_gen) last seen before the opening
        self.boundary_ok = False
        self.reopen: str | None = None  # the first post-boundary pair's action
        self.last_seen: tuple | None = None
        self.written: dict = {}  # seq -> (kind, ns after the last byte)
        self.fate: dict = {}  # seq -> "written" | "dropped" | "abandoned" (a frame queued and not here: pending)
        # this session's own stop zeros and DISARMs while queued: written whatever the run's state; a seq leaves the
        # set once written, dropped or abandoned, and the set is emptied at every opening and run end (round 3 P1-1)
        self.cleanup_seqs: set = set()
        self.new_states: list = []  # accepted STATEs since the engine last looked (the window check reads all)
        self.recent_rx: deque = deque(maxlen=256)  # (boot_id, last_rx_seq) of recent STATEs
        self.pair_rx_ns: int | None = None
        self._pair_obj = None
        self._next_stream = 0
        self.messages: list = []  # every bench message put out (tests read it)

    # --- the board's host callbacks -----------------------------------------------------------------------------------
    def _host_log(self, what, now_ns, data, info) -> None:
        fields = {}
        if info and what == "queued":
            seq = int(info["seq"]) & 0xFFFF
            self.fate.pop(seq, None)  # a seq queued again after the ring wrapped starts with no fate (round 4 P2-1)
            self.written.pop(seq, None)
        if info and what in ("written", "dropped", "abandoned"):
            seq = int(info["seq"]) & 0xFFFF
            if what == "written":
                self.written[seq] = (info["kind"], int(self.clock()))
                for rec in LinkDecoder().feed(bytes(data or b""), int(self.clock())):
                    m = rec.message  # the command's own fields in the record (axis and duty of a COMMISSION)
                    fields = {k: getattr(m, k) for k in ("axis", "duty_e4", "arm_gen", "boot_id") if hasattr(m, k)}
            self.fate[seq] = what
            self.cleanup_seqs.discard(seq)
            if what == "written" and info.get("kind") == "COMMISSION" and fields.get("duty_e4"):
                if self.run is not None and fields.get("axis") == self.run.ask.axis:
                    self.run.nonzero_written = True  # a duty actually went out (no_drive vs result_lost)
        r = self.run
        if r is not None and info and what in ("written", "dropped", "abandoned", "event"):
            self._emit(r, {"t": what, "host_ns": int(self.clock()), **dict(info), **fields})

    def _on_state(self, st, received_ns: int) -> None:
        self.new_states.append((st, received_ns))
        self.recent_rx.append((st.boot_id, st.last_rx_seq & 0xFFFF))
        r = self.run
        if r is not None:
            self._emit(r, {**state_row(Sample(received_ns, st)), "conn": self.conn})
            r.peak_current_ma = [max(a, abs(int(b))) for a, b in zip(r.peak_current_ma, st.current_ma)]
        if self.clear is not None:
            try:
                self.clear.rec.emit({**state_row(Sample(received_ns, st)), "conn": self.conn})
            except RecordError:
                pass  # a clear's record is kept best-effort; the clear itself only stops

    def _emit(self, r: Run, row: dict) -> None:
        try:
            r.rec.emit(row)
        except RecordError as exc:
            r.record_errors += 1  # counted in every phase: a row lost in the cleanup fails the verdict (round 3 P2-5)
            if r.cancel is None:
                r.cancel = f"record_error:{exc}"

    # --- the requests (inside BenchLink.begin, after the arbiter's checks) ---------------------------------------------
    def _on_shared_disarm(self, now_s: float) -> None:
        if self.run is not None and self.run.cancel is None:
            self.run.cancel = "disarm"

    def _on_request(self, kind: str, msg: dict, now_s: float):
        now = int(self.clock())
        if kind == "bench_run":
            if self.arbiter.mode == "STOPPED":
                # the STOP is released only once the board shows it stopped (round 6 P2-3): asked again after it
                return "stopped", {"detail": "waiting for the board's stop to be confirmed"}
            if self.run is not None:
                return "busy_running", {}
            if self.clear is not None:
                return "busy_clearing", {}
            why = self.not_ready()
            if why is not None:
                return "board_not_ready", {"detail": why}
            try:
                ask = parse_run(msg)
            except Refused as exc:
                return "bench_refused", {"detail": str(exc)}
            try:
                rec = RunRecord(self.runs_root, "duty", AXES[ask.axis])
            except OSError as exc:
                return "record_unavailable", {"detail": str(exc)}
            lease = self.arbiter.lease
            r = Run(request_id=msg["request_id"], ask=ask, rec=rec, lease_id=lease.lease_id,
                    owner_conn=self.arbiter.conns.get(lease.client_id), stops0=self.arbiter.stops,
                    control_gen=self.arbiter.control_gen, started_ns=now, phase_ns=now)
            self._emit(r, {"t": "request", "host_ns": now, "request_id": r.request_id, "msg": msg,
                           "product": self.host.usb_product, "names": list(ask.names), "large": ask.large,
                           "source_display": ask.source, "source_wire": "none", "physical_setup": ask.physical_setup,
                           "absent": list(ask.absent), "advice": self._advice(ask)})
            if r.cancel is not None:
                rec.close()
                return "record_unavailable", {"detail": r.cancel}
            self.run = r
            return None, {"run": rec.dir.name}
        if kind in ("bench_stop", "bench_disarm"):
            if self.run is not None:
                if self.run.cancel is None:
                    self.run.cancel = kind
                return None, {}
            if kind == "bench_disarm":
                why = self.not_ready()  # this fd's bench5 image and a pair after the boundary, as every command
                if why is not None and why != "board_initialising":
                    return "board_not_ready", {"detail": why}
                self.host.request_disarm(now)
                seq = self.host.queued_disarm_seq()
                if seq is not None:
                    self.cleanup_seqs.add(seq)
            return None, {}
        if kind == "clear_faults":
            return self._start_clear(msg, now)
        return "unsupported", {}

    @staticmethod
    def _advice(ask: Ask) -> list:
        """Plan 0.2: shown and recorded, never enforced."""
        out = []
        if ask.range_off:
            out.append("recommended: detach the motor from the mechanism (partial-parts procedure 3)")
        return out

    def not_ready(self) -> str | None:
        """Why no run or clear may start now (None: it may)."""
        if self.host.state is None:
            return "no_state"
        if not self.boundary_ok or self.pair() is None:
            return "no_pair_after_the_connection_boundary"
        if self.reopen == "boot_wait" and self.host.state.mode == Mode.BOOT:
            return "board_initialising"
        refused = self.host.bench_refusal()
        if refused is not None:
            return f"bench_refused:{refused}"
        return None

    def _start_clear(self, msg: dict, now: int):
        cause = msg.get("cause")
        if not isinstance(cause, str) or not cause.strip():
            return "bench_refused", {"detail": "cause: required"}
        if self.run is not None:
            return "busy_running", {}
        if self.clear is not None:
            return "busy_clearing", {}
        why = self.not_ready()
        if why is not None:  # this fd's bench5 image and a pair after the boundary first (round 3 P1-4)
            return "board_not_ready", {"detail": why}
        st = self.host.state
        if st.mode not in (Mode.FAULT, Mode.ESTOP):
            return "not_latched", {}
        try:
            rec = RunRecord(self.runs_root, "clear", "-")
        except OSError as exc:
            return "record_unavailable", {"detail": str(exc)}
        c = Clear(request_id=msg["request_id"], cause=cause.strip(),
                  binding=bs.ClearBinding(*self._binding_now()), rec=rec,
                  started_ns=now, phase_ns=now, floor_us=st.mcu_time_us, before=_latch_view(st))
        try:
            rec.emit({"t": "clear_request", "host_ns": now, "cause": c.cause, "before": c.before,
                      "advice": "recommended: switch VM off before the clear (weekend handoff 6.5)"})
        except RecordError as exc:
            rec.close()
            return "record_unavailable", {"detail": str(exc)}
        self.clear = c
        return None, {"clear": rec.dir.name}

    def _binding_now(self) -> tuple:
        """(host opening, board connection generation, boot, fault bits) now -- what a clear is bound to."""
        st, bst = self.host.state, self.host.bench_state
        return (self.conn, None if bst is None else bst.conn_gen, None if st is None else st.boot_id,
                FaultFlag(0) if st is None else FaultFlag(st.fault_flags))

    # --- the write boundary ------------------------------------------------------------------------------------------
    def cancel_reason(self, r: Run, now_ns: int) -> str | None:
        if r.cancel is not None:
            return r.cancel
        a = self.arbiter
        if a.stops != r.stops0:
            return "stop"
        lease = a.lease
        if lease is None or lease.lease_id != r.lease_id:
            return "lease_lost"
        if now_ns / 1e9 > lease.renewed_mono + a.config.lease_timeout_s:
            return "lease_expired"
        if a.conns.get(lease.client_id) != r.owner_conn:
            return "disconnected"
        if a.control_gen != r.control_gen:
            return "control_boundary"
        return None

    def frame_allowed(self, f) -> bool:
        now = int(self.clock())
        if not self.boundary_ok or self.host.bench_refusal() is not None:
            # a reopened host sends nothing at all until a pair after the connection boundary (plan 3: the board
            # ended its arming there itself), and nothing to a board that is not this fd's bench5 image (round 3
            # P1-4) -- not a DISARM, not a zero
            return False
        if f.expires_ns is not None and f.expires_ns <= now:
            return False
        seq = int(f.seq) & 0xFFFF
        zero = f.kind == "COMMISSION" and not f.nonzero
        if seq in self.cleanup_seqs and (f.kind == "DISARM" or zero):
            c = self.clear
            if c is not None and f.kind == "DISARM" and seq == c.disarm_seq:
                return c.binding.holds(*self._binding_now())  # the clear's DISARM: its request still holds (r4 P2-3)
            return True  # this session's own stop frame, by its seq and its kind (round 3 P1-1)
        arming_zero = f.echo is not None or (self.host.arming_zero_seq is not None
                                             and seq == self.host.arming_zero_seq & 0xFFFF)
        if zero and arming_zero and f.generation == self.host.auth_epoch:
            # the host's own arming zero (L, 0) for an ARM already out: a stop waits for it to land, then sends its
            # own zero -- refusing it would leave the arming open and the stop zero unqueueable (round 2 P1-2)
            return True
        if f.kind == "DISARM":
            return False  # no DISARM this session did not ask for
        if f.kind == "CLEAR_FAULTS":
            c, st = self.clear, self.host.state
            return (c is not None and c.clear_seq == seq and st is not None
                    and c.binding.holds(*self._binding_now()))
        r = self.run
        if r is None or f.kind not in RUN_KINDS or r.phase in ("stop", "closed"):
            return False
        if self.cancel_reason(r, now) is not None:
            return False
        if f.kind == "COMMISSION" and f.generation != self.host.auth_epoch:
            return False
        return True

    def gate(self, f, write) -> bool:
        return send_boundary(self.channels, self.serial, write, check=lambda: self.frame_allowed(f))

    # --- the cycle ------------------------------------------------------------------------------------------------
    def cycle(self, now_ns: int | None = None) -> None:
        now = int(self.clock()) if now_ns is None else int(now_ns)
        # the serial of the emergency batch handled: anything stop-like after it refuses this cycle's writes
        self.serial = self.link.begin(now / 1e9)
        for e in self.dev.receive(self.host, now):
            self._device_event(e, now)
        if now_ns is None:
            # every decision below is taken after the read (round 4 P1): a STATE read late never lends its command
            # the cycle start's longer validity
            now = int(self.clock())
        self._observe(now)
        if self.run is not None:
            self._step(self.run, now)
        if self.clear is not None:
            self._step_clear(self.clear, now)
        if self.run is None and self.clear is None and self.arbiter.mode == "STOPPED":
            p = self.pair()
            if p is not None and p.mode in STOPPED and self.host.state.mode in STOPPED:
                # nothing running and the board stopped: the screen leaves STOPPED (round 5 P2-2)
                self.arbiter.release_stop(now / 1e9, "bench_stop_confirmed")
        for e in self.dev.transmit(self.host, now):
            self._device_event(e, now)
        self.link.tick(now / 1e9)
        if now >= self._next_stream:
            self._next_stream = now + int(STREAM_S * 1e9)
            self._put(self.state_message(now))

    def _put(self, msg: dict) -> None:
        self.messages.append(msg)
        self.channels.put(None, msg)

    def _device_event(self, e, now: int) -> None:
        kind = getattr(e, "kind", None)
        if kind == "connected":
            self.conn += 1
            self.connected_ns = now
            self.boundary_from = self.last_seen
            self.boundary_ok = False
            self.reopen = None
            self._drop_for_boundary("reopened")  # nothing queued for the old connection is this one's to send
            if self.run is not None:
                self.run.link_reopened = True
                if self.run.cancel is None:
                    self.run.cancel = "link_reopened"
        elif kind == "disconnected" and self.run is not None:
            self.run.link_reopened = True
            if self.run.cancel is None:
                self.run.cancel = "link_lost"
        r = self.run
        if r is not None and kind is not None:
            self._emit(r, {"t": "device", "kind": kind, "host_ns": now,
                           **{k: str(v) for k, v in getattr(e, "detail", {}).items()}})

    def _observe(self, now: int) -> None:
        st, bst = self.host.bench_state_of, self.host.bench_state
        if bst is not None and bst is not self._pair_obj:
            self._pair_obj = bst
            self.pair_rx_ns = self.host.bench_state_rx_ns
        latest = self.host.state
        if self.boundary_ok and latest is not None and self.last_seen is not None \
                and latest.boot_id != self.last_seen[0]:
            # a new boot on the same fd (round 5 P2-1): the host keeps the old boot's pair until a new one comes, so
            # nothing is sent and no clear is bound until a pair of this boot arrives
            self.boundary_ok, self.boundary_from, self.connected_ns = False, self.last_seen, now
            self.reopen = None
            self._drop_for_boundary("rebooted")
            if self.run is not None:
                self.run.link_reopened = True
                if self.run.cancel is None:
                    self.run.cancel = "rebooted"
        p = bs.pair_of(st, bst)
        if p is None or self.connected_ns is None:
            return
        if not self.boundary_ok:
            if self.pair_rx_ns is not None and self.pair_rx_ns >= self.connected_ns \
                    and bs.after_boundary(self.boundary_from, p) \
                    and latest is not None and p.boot_id == latest.boot_id:  # one read may hold two boots (r6 P2-1)
                self.boundary_ok = True
                self.reopen = bs.reopen_action(p)
            else:
                return
        self.last_seen = (p.boot_id, p.conn_gen)

    def _drop_for_boundary(self, why: str) -> None:
        """At a connection or boot boundary: the exceptions end and the frames they were for leave the queue (round 6
        P2-2) -- left there, a refused DISARM would sit at the queue head in front of every later frame."""
        self.cleanup_seqs.clear()
        self.host.drop_frames(("DISARM", "COMMISSION", "CLEAR_FAULTS", "BENCH_SET", "BENCH_ARM"), f"boundary: {why}")

    def pair(self) -> bs.Pair | None:
        """The latest pair after the boundary, only while it is of the current STATE's boot."""
        if not self.boundary_ok:
            return None
        p = bs.pair_of(self.host.bench_state_of, self.host.bench_state)
        st = self.host.state
        if p is None or st is None or p.boot_id != st.boot_id:
            return None
        return p

    # --- one run ------------------------------------------------------------------------------------------------------
    def _phase(self, r: Run, name: str, now: int) -> None:
        r.phase, r.phase_ns = name, now
        self._emit(r, {"t": "phase", "phase": name, "host_ns": now})

    def _over(self, r: Run, now: int, seconds: float) -> bool:
        return now - r.phase_ns > seconds * 1e9

    def _take_states(self) -> list:
        states, self.new_states = self.new_states, []
        return states

    @staticmethod
    def _window_out(r: Run, states: list) -> float | None:
        """The first value of a steering or lift sample outside the user's window (range on), None if none (the CLI's
        inclusive bounds)."""
        a = r.ask
        if a.axis < 2 or a.range_off:
            return None
        for st, _rx in states:
            v = st.steering_rad if a.axis == 2 else st.lift_m
            if bs.window_stop(v, a.window, kth_invalid_bypass=False, range_off=False) is not None:
                return v
        return None

    def _step(self, r: Run, now: int) -> None:
        states = self._take_states()
        if r.phase not in ("stop", "closed"):
            why = self.cancel_reason(r, now)
            if why is not None:
                self._enter_stop(r, why, now)
        elif r.phase == "stop" and r.aborted is None:
            # a STOP, a lost lease or connection during the run's own end changes nothing that is sent, but the run
            # was interrupted: it is not a pass (round 4 P2-4)
            r.aborted = self.cancel_reason(r, now)
        if r.phase in ("recover", "set", "arm", "prep"):
            # every sample from the request to the first non-zero is judged (round 3 P2-4)
            v = self._window_out(r, states)
            if v is not None:
                if r.arm_seq is None:
                    self._refuse(r, f"window_before_drive:{v}", now)
                    return
                self._enter_stop(r, f"window_before_drive:{v}", now)
        if r.phase == "recover":
            self._recover(r, now)
        elif r.phase == "set":
            self._set(r, now)
        elif r.phase == "arm":
            self._arm(r, now)
        elif r.phase == "prep":
            self._prep(r, now)
        elif r.phase == "drive":
            self._drive(r, now, states)
        if r.phase == "stop":
            self._stop(r, now)

    def _refuse(self, r: Run, why: str, now: int) -> None:
        """Closed before any BENCH_ARM went out: nothing of the run is left queued."""
        self.host.drop_frames(RUN_KINDS, f"refused: {why}")
        r.aborted = why
        self._close(r, now, "no_drive")

    def _recover(self, r: Run, now: int) -> None:
        st, p = self.host.state, self.pair()
        if st is None or p is None or self.not_ready() is not None:
            if self._over(r, now, READY_WAIT_S):
                self._refuse(r, f"board_not_ready:{self.not_ready()}", now)
            return
        if st.mode in (Mode.FAULT, Mode.ESTOP):
            self._refuse(r, f"latched:{st.mode.name}", now)
            return
        if st.mode != Mode.DISARMED:
            if self._over(r, now, READY_WAIT_S):
                self._refuse(r, f"not_disarmed:{st.mode.name}", now)
            return
        if r.t_stop_us is None:
            r.t_stop_us = st.mcu_time_us
        if not (p.mode in STOPPED and p.boot_id == st.boot_id and p.arm_gen == st.arm_gen
                and p.mcu_time_us >= r.t_stop_us):
            if self._over(r, now, PAIR_WAIT_S + READY_WAIT_S):
                self._refuse(r, "no_bench_state_with_the_current_state", now)
            return
        if p.result_run_id != 0:
            prev = {"t": "bench_previous_result", "boot_id": p.boot_id, "run_id": p.result_run_id,
                    "stop_reason": _stop_name(p.stop_reason)}
            r.previous_result = prev
            self._emit(r, prev)
            if r.cancel is not None:  # the next ARM clears the board's copy: never before it is written down
                self._refuse(r, "previous_result_not_recorded", now)
                return
        a = r.ask
        if a.axis >= 2 and not a.range_off:
            v = st.steering_rad if a.axis == 2 else st.lift_m
            if bs.window_stop(v, a.window, kth_invalid_bypass=False, range_off=False) is not None:
                self._refuse(r, f"window: the axis value {v} is not inside {a.window}", now)
                return
        r.run_id = secrets.randbits(32) or 1
        recent = {seq for boot, seq in self.recent_rx if boot == st.boot_id}
        while (self.host.skip_seq(0) & 0xFFFF) in recent:
            self.host.skip_seq(1)
        blocked = self.host.bench_set(r.run_id, a.bench, now)
        if blocked is not None:
            self._refuse(r, f"bench_set_not_queued:{blocked.reason}", now)
            return
        r.set_seq = self.host.last_queued_seq & 0xFFFF
        r.set_echo_us, r.boot, r.conn = st.mcu_time_us, st.boot_id, self.conn
        self._phase(r, "set", now)

    def _same_link(self, r: Run) -> bool:
        st = self.host.state
        return self.conn == r.conn and st is not None and st.boot_id == r.boot and not r.link_reopened

    def _set(self, r: Run, now: int) -> None:
        if not self._same_link(r):
            self._refuse(r, "link_changed_before_the_arm", now)
            return
        st, rx = self.host.state, self.host.state_rx_ns
        w = self.written.get(r.set_seq)
        taken = (w is not None and w[0] == "BENCH_SET" and rx is not None and w[1] <= rx
                 and st.mcu_time_us > r.set_echo_us and st.last_rx_seq == r.set_seq
                 and st.reject_reason == RejectReason.NONE)
        if not taken:
            if self._over(r, now, SET_WAIT_S):
                why = st.reject_reason.name if st is not None else None
                self._refuse(r, f"bench_set_not_taken:{why}", now)
            return
        self._emit(r, {"t": "bench_set_sent", "seq": r.set_seq, "run_id": r.run_id,
                       "bypass": list(r.ask.bench.names), "duty_max_e4": list(r.ask.bench.duty_max_e4()),
                       "time_ms": list(r.ask.bench.time_ms()), "overrides": r.ask.bench.overrides()})
        blocked = self.host.request_bench_arm(r.run_id, now)
        if blocked is not None:
            self._refuse(r, f"arm_blocked:{blocked.reason}", now)
            return
        r.arm_seq = self.host.last_queued_seq & 0xFFFF
        r.arm_echo_us, r.arm_prev_gen = st.mcu_time_us, st.arm_gen
        self._phase(r, "arm", now)

    def _arm_written(self, r: Run) -> bool:
        if r.arm_written_ns is None and r.arm_seq is not None:
            w = self.written.get(r.arm_seq)
            if w is not None and w[0] == "BENCH_ARM":
                r.arm_written_ns = w[1]
        return r.arm_written_ns is not None

    def _arm(self, r: Run, now: int) -> None:
        self._arm_written(r)
        if not self.host.ready:
            if not self.host.authorized and not self.host.arming:
                self._enter_stop(r, "arm_refused", now)  # the exchange ended without authorization
            elif self._over(r, now, ARM_WAIT_S):
                self._enter_stop(r, "not_ready_after_arm", now)
            return
        p = self.pair()
        if not (p is not None and p.boot_id == r.boot and p.mode == Mode.ARMED and p.run_id == r.run_id):
            if self._over(r, now, ARM_WAIT_S + NAMED_WAIT_S):
                self._enter_stop(r, "arm_never_named_the_run", now)
            return
        r.bench_taken = {"seq": r.set_seq, "run_id": r.run_id, "bypass": list(r.ask.bench.names),
                         "duty_max_e4": list(r.ask.bench.duty_max_e4()), "time_ms": list(r.ask.bench.time_ms()),
                         "overrides": r.ask.bench.overrides()}
        self._emit(r, {"t": "bench_set", **r.bench_taken})
        # the run axis's own zero, taken before any non-zero (plan 3.1): a steering or lift axis whose KTH is invalid
        # refuses even a zero (contract 3.4), and the run then closes without driving
        blocked = self.host.commission(r.ask.axis, 0, now + self.host.cfg.cap_ms * 1_000_000, now)
        if blocked is not None:
            self._enter_stop(r, f"prep_zero_not_queued:{blocked.reason}", now)
            return
        r.prep_seq = self.host.last_queued_seq & 0xFFFF
        self._phase(r, "prep", now)

    def _prep(self, r: Run, now: int) -> None:
        st = self.host.state
        if st is not None and bs.seq_reached(st.last_rx_seq, r.prep_seq) and st.boot_id == r.boot:
            if st.reject_reason != RejectReason.NONE or st.mode != Mode.ARMED:
                self._enter_stop(r, f"prep_zero_refused:{st.reject_reason.name}", now)
                return
            r.drive_start_ns = now
            r.drive_end_ns = now + int(r.ask.run.seconds * 1e9)
            r.faults_at_drive = int(st.fault_flags)
            self._phase(r, "drive", now)
            return
        if self._over(r, now, PREP_WAIT_S):
            self._enter_stop(r, "prep_zero_not_taken", now)

    def _drive(self, r: Run, now: int, states: list) -> None:
        a = r.ask
        v = self._window_out(r, states)
        if v is not None:
            r.window_stop = f"window:{v}"
        elif a.axis < 2:
            # the CLI's early stop (bringup early_stop): a stuck or polarity bit of the driven wheel newly on (the
            # commissioning build only reports it; A2 brings the choice to bypass it -- round 3 P2-4)
            watched = int(bringup._AXIS_ENCODER_BITS[a.axis])
            for st_, _rx in states:
                if int(st_.fault_flags) & watched & ~r.faults_at_drive:
                    r.window_stop = "encoder_bit"
                    break
        st = self.host.state
        if r.window_stop is not None or not self.host.ready or st is None or st.mode != Mode.ARMED \
                or now >= r.drive_end_ns - int(bringup.HANDOVER_S * 1e9):
            self._end(r, now)
            return
        if now - r.last_send_ns >= SEND_EVERY_S * 1e9:
            expires = min(now + self.host.cfg.cap_ms * 1_000_000, r.drive_end_ns)
            duty_e4 = int(round(a.run.duty * 10000))
            if self.host.commission(a.axis, duty_e4, expires, now) is None:
                r.last_send_ns = now

    def _end(self, r: Run, now: int) -> None:
        """The run's own end: one zero (the board stops itself, contract 8.3), then the stop rule's wait."""
        self._enter_stop(r, None, now)

    def _enter_stop(self, r: Run, why: str | None, now: int) -> None:
        if r.phase in ("stop", "closed"):
            return
        r.aborted = why if why is not None else r.aborted
        self.host.drop_frames(("BENCH_SET",), f"stop: {why}")
        # the queued duties go, the host's own arming zero for an ARM already out stays (round 3 P1-2): dropping it
        # would leave the exchange in zero_pending with nothing to answer it
        self.host.drop_frames(("COMMISSION",), f"stop: {why}", keep_arming_zero=True)
        if not self._arm_written(r):
            # the BENCH_ARM never went out whole (queued, or abandoned behind the poison): no arming can follow it
            self.host.drop_frames(("BENCH_ARM",), f"stop: {why}")
            if self.host.arming:
                self.host.cancel_arm(now, why or "stop")
        r.stop_entered_ns = now
        self._phase(r, "stop", now)
        self._emit(r, {"t": "stop", "why": why, "host_ns": now})

    def _after_arm(self, r: Run, p: bs.Pair | None) -> bool:
        """Whether ``p`` can only have been made after the BENCH_ARM's fate was decided (round 2 P1-2): its arming
        generation moved on (the ARM was taken), or on the same boot its MCU time is past the ARM's acceptance window
        (the echo's age the board allows: its ``command_timeout_ms``, round 3 P1-2). The host's receive time alone
        cannot rule out an earlier DISARMED pair arriving late."""
        if p is None or r.arm_written_ns is None or p.boot_id != r.boot:
            return False
        if p.arm_gen != r.arm_prev_gen:
            return True
        return p.mcu_time_us > r.arm_echo_us + BOARD_COMMAND_TIMEOUT_MS * 1000

    def _zero_written(self, r: Run) -> None:
        """The stop zero the board's answer is read against: the first one actually written (round 2 P1-3)."""
        if r.zero_seq is not None:
            return
        for seq, echo_us, link in r.zero_seqs:
            w = self.written.get(seq)
            if w is not None and w[0] == "COMMISSION":
                r.zero_seq, r.zero_written_ns, r.zero_echo_us, r.zero_link = seq, w[1], echo_us, link
                self._emit(r, {"t": "stop_zero_written", "seq": seq, "host_ns": w[1]})
                return

    def _pending(self, seq: int | None) -> bool:
        """Queued and not yet written, dropped or abandoned."""
        return seq is not None and seq not in self.fate

    def _stop(self, r: Run, now: int) -> None:
        p = self.pair()
        arm_sent = r.arm_written_ns is not None
        if p is not None and r.run_id is not None and p.result_run_id == r.run_id and \
                p.drive_state == BenchDriveState.ENDED:
            r.ended_seen = True  # the board's end of this run (its own record), whatever mode the pair is in
        if r.link_reopened:
            # after a reopening nothing is sent (the board ended its arming at the boundary): wait for the first
            # pair after it to show the stop
            if p is not None and p.mode in STOPPED:
                if p.boot_id != r.boot:
                    r.aborted = r.aborted or "rebooted_result_lost"  # a new boot keeps no record of this run
                elif p.mode == Mode.DISARMED and r.ended_seen:
                    r.cleanup_confirmed = True
                self._close(r, now, None)
            elif now - r.stop_entered_ns > (STOP_WAIT_S + 1.0) * 1e9:
                r.aborted = r.aborted or "stop_unconfirmed_after_reopen"
                self._close(r, now, None)
            return
        self._zero_written(r)
        after_zero = (r.zero_seq is not None and p is not None and self.pair_rx_ns is not None
                      and self.pair_rx_ns > r.zero_written_ns and p.mcu_time_us > r.zero_echo_us)
        arming = self.host.arming or (self.host.authorized and not self.host.ready)
        action = bs.stop_action(arm_sent=arm_sent, zero_seq=r.zero_seq, zero_link=r.zero_link, p=p, arming=arming,
                                after_arm=self._after_arm(r, p), after_zero=after_zero)
        if action == "cancel":
            self._refuse(r, r.aborted or "cancelled", now)
            return
        if action in ("closed", "closed_latched"):
            if p is not None and p.mode == Mode.DISARMED and (r.ended_seen or self._disarm_confirmed(r, p)):
                r.cleanup_confirmed = True
            self._close(r, now, None)
            return
        if action in ("send_zero", "resend_zero"):
            pending = any(self._pending(seq) for seq, _e, _l in r.zero_seqs)
            if pending or now - r.zero_sent_ns < RESEND_ZERO_S * 1e9:
                pass  # one stop zero on its way at a time (round 3 P2-1)
            elif self.host.ready:
                st = self.host.state
                if self.host.commission(r.ask.axis, 0, now + self.host.cfg.cap_ms * 1_000_000, now) is None:
                    seq = self.host.last_queued_seq & 0xFFFF
                    self.cleanup_seqs.add(seq)
                    r.zero_sent_ns = now
                    # the zero's echo and the arming it answers both from the STATE it was queued against; the
                    # connection generation from the latest pair (it changes only at a boundary, which ends sending)
                    link = (st.boot_id, p.conn_gen, st.arm_gen) if p is not None and st is not None else None
                    r.zero_seqs.append((seq, st.mcu_time_us if st is not None else 0, link))
                    self._emit(r, {"t": "stop_zero", "seq": seq, "host_ns": now})
            # else: no authorization left to send a zero -- never a DISARM to a board that may still drive (it would
            # skip the brake, round 3 P1-3): the board's own command expiry stops it, and the wait below sees it
        elif action == "send_disarm":
            if r.disarm_seq is None or self.fate.get(r.disarm_seq) in ("dropped", "abandoned"):
                self._disarm(r, now)  # again only if the last one never went out whole
        if r.disarm_seq is not None and p is not None and self._disarm_confirmed(r, p):
            r.cleanup_confirmed = True
            self._close(r, now, None)
            return
        if now - r.stop_entered_ns > STOP_WAIT_S * 1e9:
            r.aborted = r.aborted or "stop_unconfirmed"
            self._close(r, now, None)

    def _disarm_confirmed(self, r: Run, p: bs.Pair) -> bool:
        """This run's DISARM went out whole and a DISARMED pair of the same boot, received after it, names its seq
        (round 3 P2-1: never the echo or the mode alone)."""
        if r.disarm_seq is None:
            return False
        w = self.written.get(r.disarm_seq)
        if w is None or w[0] != "DISARM":
            return False
        r.disarm_written_ns = w[1]
        return (self.pair_rx_ns is not None and self.pair_rx_ns > w[1] and p.boot_id == r.boot
                and p.mode == Mode.DISARMED and bs.seq_reached(p.last_rx_seq, r.disarm_seq))

    def _disarm(self, r: Run, now: int) -> None:
        self.host.request_disarm(now)
        seq = self.host.queued_disarm_seq()
        if seq is not None:
            self.cleanup_seqs.add(seq)
        r.disarm_seq, r.disarm_written_ns = seq, None
        self._emit(r, {"t": "stop_disarm", "seq": seq, "host_ns": now})

    def _close(self, r: Run, now: int, closed: str | None) -> None:
        p = self.pair()
        # the run's own frames still queued go with it (round 4 P2-2): an unsent stop DISARM would otherwise be
        # refused at every gate and handed back to the queue head, in front of the next run's BENCH_SET
        self.host.drop_frames(("COMMISSION", "DISARM", "BENCH_SET", "BENCH_ARM"), "run closed")
        result_seen = p is not None and r.run_id is not None and p.result_run_id == r.run_id
        stop_reason = _stop_name(p.stop_reason) if result_seen else None
        if closed is None:
            if result_seen:
                closed = "ended"  # the board drove this run and recorded how it ended
            elif r.nonzero_written:
                closed = "result_lost"  # a duty went out but the board's record of the run was not read
            else:
                closed = "no_drive"
        st = self.host.state
        if p is not None and st is not None and st.boot_id == p.boot_id and st.mcu_time_us >= p.mcu_time_us:
            # the verdict reads the latest STATE too: a FAULT after the last pair is not a clean end (round 4 P2-4)
            p = replace(p, mode=st.mode, fault_flags=st.fault_flags, duty_e4=tuple(st.duty_e4))
        if closed == "ended" and r.window_stop is not None:
            r.aborted = r.aborted or r.window_stop
        faults = []
        if p is not None:
            faults = [f.name for f in FaultFlag if f in p.fault_flags and f != FaultFlag.COMMAND_TIMEOUT]
        result = {"t": "bench_result", "request_id": r.request_id, "run": r.rec.dir.name,
                  "record": str(r.rec.dir), "axis": AXES[r.ask.axis], "duty": r.ask.run.duty,
                  "seconds": r.ask.run.seconds, "closed": closed, "run_id": r.run_id, "bench_set": r.bench_taken,
                  "stop_reason": stop_reason, "aborted": r.aborted, "cleanup_confirmed": r.cleanup_confirmed,
                  "previous_result": r.previous_result,
                  "bypass_mcu": list(r.bench_taken["bypass"]) if r.bench_taken else [], "bypass_host": [],
                  "not_for_gate": True, "physical_setup": r.ask.physical_setup,
                  "peak_current_ma": list(r.peak_current_ma),  # from the run's own STATEs, not the screen stream
                  "final": None if p is None else {"mode": p.mode.name, "duty_e4": list(p.duty_e4)}}
        r.phase = "closed"
        try:
            r.rec.emit({**result, "t": "result"})
        except RecordError:
            r.record_errors += 1
        verdict = bs.run_verdict(closed=closed, stop_reason=stop_reason, aborted=r.aborted,
                                 cleanup_confirmed=r.cleanup_confirmed, last=p, record_errors=r.record_errors,
                                 unexpected_faults=tuple(faults))
        result["verdict"] = {"run_cleanup": verdict}
        result["record_errors"] = r.record_errors
        try:
            r.rec.result(result)
        except OSError as exc:
            result["record_error"] = str(exc)
            result["verdict"]["run_cleanup"] = {"ok": False, "why": verdict["why"] + [f"record:{exc}"]}
        r.rec.close()
        self.last_result = result
        self._put(result)
        self.run = None
        self.cleanup_seqs.clear()  # the next run's stop frames are its own
        st = self.host.state
        if st is not None and st.mode in STOPPED and self.arbiter.mode == "STOPPED":
            self.arbiter.release_stop(now / 1e9, "bench_stop_confirmed")

    # --- the clear ----------------------------------------------------------------------------------------------------
    def _seen_after(self, seq: int | None) -> bool:
        """A STATE received after that frame's last byte went out (never one made before it)."""
        w = self.written.get(seq) if seq is not None else None
        return w is not None and self.host.state_rx_ns is not None and self.host.state_rx_ns > w[1]

    def _clear_done(self, c: Clear, result: str, now: int, why: str | None = None) -> None:
        st = self.host.state
        last = c.clear_seq if c.clear_seq is not None and c.clear_seq in self.written else c.disarm_seq
        sent = last is not None and last in self.written
        # the state after the attempt: only one received after its last frame (Codex C round 5 P2-2); before any
        # frame went out, the state as it is
        after = None if st is None or (sent and not self._seen_after(last)) else _latch_view(st)
        out = {"t": "clear_result", "request_id": c.request_id, "clear": c.rec.dir.name, "result": result,
               "why": why, "cause": c.cause, "before": c.before, "after": after}
        self.host.drop_frames(("CLEAR_FAULTS",) if result == "cleared" else ("CLEAR_FAULTS", "DISARM"),
                              f"clear: {result}")  # a refused request's own unsent frames go with it (round 3 P2-3)
        try:
            c.rec.emit(out)
            c.rec.result(out)
        except (RecordError, OSError):
            pass
        c.rec.close()
        self._put(out)
        self.clear = None

    def _echoed(self, seq: int, kind: str, c: Clear) -> bool:
        st, rx = self.host.state, self.host.state_rx_ns
        w = self.written.get(seq)
        return (w is not None and w[0] == kind and rx is not None and w[1] <= rx and st.boot_id == c.binding.boot_id
                and self.conn == c.binding.conn and st.mcu_time_us > c.floor_us and st.last_rx_seq == seq)

    def _step_clear(self, c: Clear, now: int) -> None:
        st = self.host.state
        conn_gen = None if self.host.bench_state is None else self.host.bench_state.conn_gen
        if self.conn != c.binding.conn or conn_gen != c.binding.conn_gen or st is None \
                or st.boot_id != c.binding.boot_id:
            self._clear_done(c, "refused", now, "the link or the boot changed: ask again")
            return
        if c.phase == "disarm":
            if c.disarm_seq is None:
                if not c.binding.holds(*self._binding_now()):
                    # judged before the DISARM too (round 2 P2-3): a request asked on other bits is not run
                    self._clear_done(c, "refused", now, "the fault bits changed before the DISARM: ask again")
                    return
                seen = {s for _b, s in self.recent_rx} | {0}
                while (self.host.skip_seq(0) & 0xFFFF) in seen:  # a number no recent STATE named, before queueing
                    self.host.skip_seq(1)
                self.host.request_disarm(now)
                c.disarm_seq = self.host.queued_disarm_seq()
                self.cleanup_seqs.add(c.disarm_seq)
                c.phase_ns = now
            elif self._pending(c.disarm_seq) and not c.binding.holds(*self._binding_now()):
                # still unsent and the request no longer holds: refused, its DISARM out of the queue (round 4 P2-3)
                self._clear_done(c, "refused", now, "the board changed before the DISARM went out: ask again")
                return
            elif self._echoed(c.disarm_seq, "DISARM", c):
                if not c.binding.holds(*self._binding_now()):
                    self._clear_done(c, "refused", now, "the fault bits changed: ask again")
                    return
                blocked = self.host.request_clear_faults(now)
                if blocked is not None:
                    self._clear_done(c, "refused", now, blocked.reason)
                    return
                c.clear_seq = self.host.last_queued_seq & 0xFFFF
                c.phase, c.phase_ns = "clear", now
            elif now - c.phase_ns > CLEAR_ECHO_S * 1e9:
                self._clear_done(c, "no_echo", now, "the DISARM before the clear was not echoed")
        elif c.phase == "clear":
            if not self._echoed(c.clear_seq, "CLEAR_FAULTS", c) and self._pending(c.clear_seq) \
                    and not c.binding.holds(*self._binding_now()):
                # still unsent and the bits changed: refused for good, the frame out of the queue (round 3 P2-3)
                self._clear_done(c, "refused", now, "the fault bits changed before the CLEAR: ask again")
                return
            if self._echoed(c.clear_seq, "CLEAR_FAULTS", c):
                c.phase, c.phase_ns = "ready", now
            elif now - c.phase_ns > CLEAR_ECHO_S * 1e9:
                if not self._seen_after(c.clear_seq):
                    # no STATE since the CLEAR went out (or it never went whole): nothing says what the board did
                    self._clear_done(c, "unconfirmed", now, "no STATE after the CLEAR")
                else:
                    why = st.reject_reason.name
                    self._clear_done(c, "not_cleared", now, f"the CLEAR was not taken (reject {why})")
        elif c.phase == "ready":
            if st.mode == Mode.DISARMED and st.flags & StateFlag.DRIVERS_READY:
                self._clear_done(c, "cleared", now)
            elif now - c.phase_ns > CLEAR_READY_S * 1e9:
                self._clear_done(c, "not_ready", now, f"after the clear: {st.mode.name}")

    # --- the stream ---------------------------------------------------------------------------------------------------
    def state_message(self, now: int) -> dict:
        a, st, bst = self.arbiter, self.host.state, self.host.bench_state
        lease = a.lease
        board = None
        if st is not None:
            invalid = []
            steer, lift = _finite_or_none(st.steering_rad), _finite_or_none(st.lift_m)
            if steer is None:
                invalid.append("steering_rad:invalid")
            if lift is None:
                invalid.append("lift_m:invalid")
            board = {"mode": st.mode.name, "boot_id": st.boot_id, "arm_gen": st.arm_gen, "duty_e4": list(st.duty_e4),
                     "current_ma": list(st.current_ma), "driver_status": [f"{b:02X}" for b in st.driver_status],
                     "fault_flags": [f.name for f in FaultFlag if f in st.fault_flags],
                     "reject_reason": st.reject_reason.name, "steering_rad": steer, "lift_m": lift,
                     "invalid": invalid, "wheel_count": [st.wheel_count_left, st.wheel_count_right]}
        bench = None
        if bst is not None:
            bench = {"run_id": bst.run_id, "drive_state": bst.drive_state.name, "result_run_id": bst.result_run_id,
                     "stop_reason": _stop_name(bst.stop_reason), "conn_gen": bst.conn_gen,
                     "mcu_bypass": [n for n, mask in bl.BENCH_BYPASS_BITS.items() if bst.mcu_bypass & mask]}
        r = self.run
        run = None if r is None else {
            "request_id": r.request_id, "name": r.rec.dir.name, "phase": r.phase, "axis": AXES[r.ask.axis],
            "duty": r.ask.run.duty, "seconds": r.ask.run.seconds, "bypass_asked": list(r.ask.bench.names),
            "bypass_mcu": list(r.bench_taken["bypass"]) if r.bench_taken else [], "bypass_host": [],
            "advice": self._advice(r.ask)}
        return {"t": "bench_state", "session_epoch": self.epoch, "mode": a.mode, "control_gen": a.control_gen,
                "stops": a.stops, "lease": None if lease is None else {
                    "client_id": lease.client_id,
                    "remaining_s": max(0.0, lease.renewed_mono + a.config.lease_timeout_s - now / 1e9)},
                "product": self.host.usb_product, "ready": self.not_ready() is None, "not_ready": self.not_ready(),
                "reopen": self.reopen, "board": board, "bench": bench, "run": run,
                "clear": None if self.clear is None else {"phase": self.clear.phase, "cause": self.clear.cause},
                "last_result": self.last_result}


def _stop_name(v: int):
    return BenchStop(v).name.lower() if 0 <= v <= max(BenchStop) else v


MODEL_PRODUCT = "forklift-mcu commission OPEN model rec=00000000 lim=lr1 bench5"


class ModelDevice:
    """The MCU model (``mcu_model --commission-build 1 --bench 1``) on this tool's own pty, for trying the console
    without a board (plan 4.1: the screen's check in a browser). Nothing else can open the pty; the product string
    names the model."""

    def __init__(self, exe: str, gate) -> None:
        import os
        import subprocess
        import tty

        from real_mcu_link import BenchTtyLink

        self.master = self.slave = None
        self.proc = None
        try:
            self.master, self.slave = os.openpty()
            tty.setraw(self.slave)
            os.set_blocking(self.master, False)
            # its own session: a terminal Ctrl-C reaches this tool, which then stops the model itself (round 5 P2-1)
            self.proc = subprocess.Popen([str(exe), "--link", os.ttyname(self.slave), "--commission-build", "1",
                                          "--bench", "1"], start_new_session=True)
            self.link = BenchTtyLink(self.master, gate=gate)
        except BaseException:
            self.close()  # neither the child nor the pty outlives a failed construction (round 3 P3-1, 4 P3-2)
            raise
        self.opened = False
        self.dead = False

    def receive(self, host, now):
        from forklift_core.plant.mcu_host import HostEvent

        events = []
        if self.dead:
            return events + host.tick(now)
        if self.proc.poll() is not None:
            # the model ended: a lost device, as a board unplugged (round 3 P2-5) -- the console stops being ready
            self.dead = True
            return events + host.disconnected(now)  # the host's own disconnected event: one per loss
        if not self.opened:
            self.opened = True
            events.append(HostEvent("connected", {"path": "model", "product": MODEL_PRODUCT}))
            events += host.link_identity(MODEL_PRODUCT)
        data = self.link.read_available()
        if data:
            events += host.on_bytes(data, time.monotonic_ns())
        events += host.tick(now)
        return events

    def transmit(self, host, now):
        if not self.dead:
            self.link.pump_writes(host, now)
        return []

    def close(self) -> None:
        import os

        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except Exception:  # noqa: BLE001
                self.proc.kill()
        for fd in (self.master, self.slave):
            if fd is None:
                continue
            try:
                os.close(fd)
            except OSError:
                pass


def shutdown(console: "BenchConsole", *, wait_s: float = STOP_WAIT_S + 1.0, sleep=time.sleep) -> None:
    """The tool's exit: a run is cancelled and stopped by the stop rule, a clear whose CLEAR never went out is closed as
    cancelled; both are given ``wait_s`` to end, and whatever is still open then is closed with a record --
    ``unconfirmed`` / ``unconfirmed_at_exit`` -- never left without one (Codex C round 4 P2-4, round 5 P2-1)."""
    if console.run is not None and console.run.cancel is None:
        console.run.cancel = "tool_exit"
    c = console.clear
    if c is not None and (c.clear_seq is None or console._pending(c.clear_seq)):
        console._clear_done(c, "cancelled", int(console.clock()), "the tool was stopped before the CLEAR")
    stop_end = time.monotonic() + wait_s
    while (console.run is not None or console.clear is not None) and time.monotonic() < stop_end:
        console.cycle()
        sleep(CYCLE_S)
    if console.clear is not None:
        console._clear_done(console.clear, "unconfirmed", int(console.clock()), "the tool was stopped")
    if console.run is not None:
        r = console.run
        r.aborted = r.aborted or "unconfirmed_at_exit"
        console._close(r, int(console.clock()), None)


# --- the CLI -----------------------------------------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawTextHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--device")
    src.add_argument("--model", help="the MCU model executable (mcu_model): the console on its own pty, no board")
    p.add_argument("--baud", type=int, default=None)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--console-lan", action="store_true",
                   help="serve on a LAN address (no authentication: anyone who reaches it may take the lease)")
    p.add_argument("--console-origin-host", default=None, help="the LAN host the page is opened from")
    p.add_argument("--web-root", default=str(REPO / "console" / "web"))
    p.add_argument("--runs-root", default="~/bench_runs")
    p.add_argument("--duration-s", type=float, default=None)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.host not in LOOPBACK and not args.console_lan:
        print("refused: a non-loopback --host needs --console-lan", file=sys.stderr)
        return 2
    if args.console_lan and not args.console_origin_host:
        print("refused: --console-lan needs --console-origin-host", file=sys.stderr)
        return 2
    from console.live.server import LiveServer, MediaStore
    from real_mcu_link import BenchTtyLink, ReconnectingLink

    import signal

    stop_asked = {"v": False}

    def _terminate(signum, frame):  # noqa: ARG001 -- only asks: the loop ends at a cycle's boundary (round 3 P1),
        stop_asked["v"] = True  # never in the middle of a frame's bookkeeping, and the stop cleanup then runs

    signal.signal(signal.SIGTERM, _terminate)
    signal.signal(signal.SIGINT, _terminate)  # Ctrl-C too (round 4 P1): never an exception mid-frame
    holder = {}
    gate = lambda f, w: holder["c"].gate(f, w)  # noqa: E731 -- bound at every opening, a reopening too
    dev = None
    server = None
    try:
        if args.model:
            dev = ModelDevice(args.model, gate)  # the child exists from here: the finally below ends it
            console = BenchConsole(dev, runs_root=Path(args.runs_root))
            holder["c"] = console
            end_wait = time.monotonic() + 3.0
            while console.host.state is None and time.monotonic() < end_wait and not stop_asked["v"]:
                console.cycle()
                time.sleep(CYCLE_S)
            if console.host.state is None:
                print("refused: the model sent no STATE within 3 s", file=sys.stderr)
                return 2
        else:
            dev = ReconnectingLink(args.device, baud=args.baud, exclusive=True, clock=time.monotonic_ns,
                                   link_factory=lambda fd: BenchTtyLink(fd, gate=gate))
            console = BenchConsole(dev, runs_root=Path(args.runs_root))
            holder["c"] = console
            try:
                dev.open_first(console.host, time.monotonic_ns())
            except Exception as exc:  # noqa: BLE001 -- a held device or an unreadable one: refused before the console
                print(f"refused: {exc}", file=sys.stderr)
                return 2
        product = console.host.usb_product
        if bl.any_bench_image(product) and not bl.bench_image(product):
            print(f"refused: {product!r} is another bench generation than '{' '.join(bl.BENCH_SUFFIX)}'",
                  file=sys.stderr)
            return 2
        if not bl.bench_image(product):
            print(f"refused: {product!r} is not the bench image (no product ending '{' '.join(bl.BENCH_SUFFIX)}')",
                  file=sys.stderr)
            return 2
        hosts = ("127.0.0.1", "localhost") + ((args.console_origin_host,) if args.console_lan else ())
        server = LiveServer(console.channels, MediaStore(), welcome(console.epoch, product), host=args.host,
                            port=args.port, web_root=Path(args.web_root), origin_hosts=hosts)
        console.channels.notify = lambda: server.handle.wake() if server.handle is not None else None
        port = server.start()
        print(json.dumps({"t": "started", "port": port, "product": product, "runs_root": str(console.runs_root),
                          "console_lan": bool(args.console_lan)}), flush=True)
        end = None if args.duration_s is None else time.monotonic() + args.duration_s
        try:
            while (end is None or time.monotonic() < end) and not stop_asked["v"]:
                console.cycle()
                time.sleep(CYCLE_S)
        except KeyboardInterrupt:
            pass
        finally:
            shutdown(console)
    finally:
        if server is not None:
            server.stop()
        if isinstance(dev, ModelDevice):
            dev.close()
        elif dev is not None and dev.link is not None:
            dev.link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
