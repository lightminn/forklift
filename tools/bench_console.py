#!/usr/bin/env python3
"""The bench console's back end (docs/plans/2026-11-27-bench-console.md 3, 3.1 B1): the operator console served on
the Jetson for the bench image (``lim=lr1 bench7``), one board writer, the shared lease, STOP and request ledger, and a
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
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))  # console/
sys.path.insert(0, str(Path(__file__).resolve().parent))

import bringup  # noqa: E402
from forklift_core.live.bench_link import BenchChannels, BenchLink  # noqa: E402
from forklift_core.live.real_channels import send_boundary  # noqa: E402
from forklift_core.mcu_link.build_policy import Client  # noqa: E402
from forklift_core.mcu_link.messages import (  # noqa: E402
    BenchDriveState, BenchStop, FaultFlag, LinkDecoder, Mode, RejectReason, StateFlag, valid_after_echo_ms)
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
RUN_KINDS = ("BENCH_SET", "BENCH_ARM", "COMMISSION", "BENCH_HOLD")
BRAKE_NOTICE_S = 1.0  # a STOPPING this long: the brake's end not seen yet, said on the screen (plan 4.4)
STOPPING_LIKE = (Mode.STOPPING, Mode.DISARMED, Mode.FAULT, Mode.ESTOP)
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

    def final_row(self, row: dict) -> bool:
        """The run's final result row (plan 4.4 decision 4): the earlier rows flushed, then this one in a single
        unbuffered write and an fsync, never cut back; a torn row is closed with a line end. True only when it is on
        disk (``record_complete``); the verdict in it is the rows before it."""
        try:
            self._f.flush()
            fd = self._f.fileno()
        except (OSError, ValueError):
            return False
        data = (json.dumps(row, default=str) + "\n").encode("utf-8")
        try:
            if os.write(fd, data) != len(data):
                try:
                    os.write(fd, b"\n")
                except OSError:
                    pass
                return False
            os.fsync(fd)
            return True
        except OSError:
            return False

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


def _finite(v) -> bool:
    """A JSON number that is finite as a float (a huge integer raises in ``math.isfinite``: refused, never raised)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return False
    try:
        return math.isfinite(v) and math.isfinite(float(v))
    except (OverflowError, ValueError):
        return False


def _num(msg: dict, key: str, *, required: bool) -> float | None:
    v = msg.get(key)
    if v is None:
        if required:
            raise Refused(f"{key}: required")
        return None
    if not _finite(v):
        raise Refused(f"{key}: a finite number")
    return float(v)


def _f32(v: float) -> float:
    return struct.unpack("<f", struct.pack("<f", v))[0]


# the 5.1 bundle's checked range for a distance or measuring run's speed exclusion (plan 4.4 decision 2): empty until
# the bundle's results are back -- every such run is 'unchecked' until then. {axis: {"S": ±1, "duty_max": {+1: d,
# -1: d}, "setup": text}}
CHECKED_RANGE: dict = {}


def range_mark(kind: str, axis: int, duty: float, count_sign: int | None, same_setup: bool) -> str | None:
    """The checked-range mark (plan 4.4 decision 2, shown and recorded, never refused): distance and measuring runs
    only, per measured direction."""
    if kind == "hold":
        return "not_applicable"  # a hold's gains and settling are checked apart
    if kind not in ("distance", "measure"):
        return None
    c = CHECKED_RANGE.get(AXES[axis])
    if not c:
        return "unchecked"
    lim = (c.get("duty_max") or {}).get(1 if duty > 0 else -1)
    if lim is None or c.get("excluded"):
        return "outside" if c.get("excluded") else "unchecked"
    if count_sign != c.get("S") or abs(duty) > lim or not same_setup:
        return "outside"
    return "inside"


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
    kind: str = "duty"  # duty | hold | distance | measure (plan 4.4 decision 2)
    source_wire: str = "none"
    advisories: tuple = ()
    range_mark: str | None = None


def _bench_fields(msg: dict, axis: int) -> dict:
    """The B2 request fields checked as a web schema first (plan 4.4 decision 2): what argparse would have done for the
    CLI -- types, choices, the one shape of ``stop`` and ``hold`` -- and the exclusions, before the CLI's own checks.
    Returns the Namespace fields and the run's kind."""
    mode = msg.get("mode", "duty")
    if mode not in ("duty", "hold"):
        raise Refused("mode: duty or hold")
    stop = msg.get("stop")
    turns = None
    if stop is not None:
        if isinstance(stop, dict) and "counts" in stop and "turns" not in stop:
            raise Refused("not_yet_a1c: a distance in MT counts comes with A1c")
        if not (isinstance(stop, dict) and set(stop) == {"turns"}):
            raise Refused("stop: {turns: N}")
        turns = stop["turns"]
        if not _finite(turns):
            raise Refused("stop.turns: a finite number")
    measure = msg.get("measure", False)
    if not isinstance(measure, bool):
        raise Refused("measure: true or false")
    hold = msg.get("hold")
    if hold is not None and mode != "hold":
        raise Refused("hold: only with mode hold")
    if mode == "hold" and not isinstance(hold, dict):
        raise Refused("hold: an object {target, kp, ki, kd, deadband, min_duty}")
    source = msg.get("source", "kth" if axis >= 2 else "none")
    if mode == "hold" and stop is not None:
        raise Refused("hold: no stop (a hold ends by time)")
    if measure and (stop is not None or mode == "hold" or source != "kth"):
        raise Refused("measure: source kth, a duty run with no stop")
    kind = "hold" if mode == "hold" else "distance" if stop is not None else "measure" if measure else "duty"
    if kind != "duty" and source != "kth":
        raise Refused(f"{kind}: source kth (the KTH count; never silently)")
    out = {"kth_measure": kind == "measure", "stop_kth_turns": None if turns is None else float(turns), "hold": None,
           "hold_kp": None, "hold_ki": None, "hold_kd": None, "hold_deadband": None, "hold_min_duty": None,
           "count_sign": 1}
    cs = msg.get("count_sign")
    if kind != "duty":
        # the sign is the operator's, never a default (plan 4.4, independent review P1-2)
        if isinstance(cs, bool) or not isinstance(cs, int) or cs not in (1, -1):
            raise Refused(f"count_sign: 1 or -1, required for a {kind} run")
        out["count_sign"] = int(cs)
    elif cs is not None and (isinstance(cs, bool) or not isinstance(cs, int) or cs not in (1, -1)):
        raise Refused("count_sign: 1 or -1")
    if kind == "hold":
        target = hold.get("target")
        if not isinstance(target, str):
            raise Refused("hold.target: zero, here, abs:N or rel:N")
        out["hold"] = target
        for key, name in (("kp", "hold_kp"), ("ki", "hold_ki"), ("kd", "hold_kd"), ("min_duty", "hold_min_duty")):
            v = hold.get(key)
            if v is None:
                continue
            if not _finite(v):
                raise Refused(f"hold.{key}: a finite number")
            v = float(v)
            if key in ("kp", "ki", "kd") and (v != 0 and _f32(v) == 0 or not math.isfinite(_f32(v))):
                raise Refused(f"hold.{key}: {v} does not survive the wire's float32")  # the board would refuse it
            out[name] = v
        db = hold.get("deadband")
        if db is not None:
            if isinstance(db, bool) or not isinstance(db, int):
                raise Refused("hold.deadband: an integer count")
            out["hold_deadband"] = db
        unknown = set(hold) - {"target", "kp", "ki", "kd", "deadband", "min_duty"}
        if unknown:
            raise Refused(f"hold: unknown {sorted(unknown)}")
    return {**out, "kind": kind}


def parse_run(msg: dict) -> Ask:
    """The web request checked as the CLI's arguments are (plan 3.1): types first, then bringup's own limits; the
    confirmations come only from the request's own booleans; ``synthetic`` and the product are never taken from it."""
    axis_name = msg.get("axis")
    if axis_name not in tuple(AXES):
        raise Refused("axis: one of L, R, M, H")
    axis = AXES.index(axis_name)
    b2 = _bench_fields(msg, axis)
    kind = b2.pop("kind")
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
        if not (isinstance(window, list) and len(window) == 2 and all(_finite(v) for v in window)):
            raise Refused("window: two finite numbers")
        window = (float(window[0]), float(window[1]))
    setup = msg.get("physical_setup")
    if setup is not None and not isinstance(setup, str):
        raise Refused("physical_setup: text")
    large_confirm = msg.get("large_confirm")
    if large_confirm is not None and not isinstance(large_confirm, str):
        raise Refused("large_confirm: text")
    duty = _num(msg, "duty", required=kind != "hold")
    if kind == "hold" and duty is None:
        duty = bringup.HOLD_DUTY_DEFAULT  # the hold's cap, as the CLI's main() fills it
    if kind == "distance" and round(abs(duty) * 10000) < 1:
        raise Refused("duty: a distance run needs a non-zero duty on the wire (0.0001 or more)")
    ns = argparse.Namespace(
        cmd="commission", axis=axis_name, duty=duty,
        seconds=_num(msg, "seconds", required=True), window=window, bypass=None,
        bench_bypass=",".join(bypass),  # never None: every run builds its BenchAsk (the same limits for [] and none)
        bench_duty_max=_num(msg, "bench_duty_max", required=False),
        bench_time_max=_num(msg, "bench_time_max", required=False),
        power_switch_in_hand=_flag(msg, "power_switch_in_hand"), wheels_lifted=_flag(msg, "wheels_lifted"),
        vcc_watched=_flag(msg, "vcc_watched"), synthetic=False, single_axis=False, motor_detached=False,
        sensorless=False, profile_tri=None, scenario=None, limit_s=None, stop_grace_ms=None, disarm_delay_ms=0, **b2)
    try:
        run, _profile, names = bringup._check_commission_args(ns)
        bench = bringup._bench_ask(ns)
    except bringup.Refused as exc:
        raise Refused(str(exc)) from None
    large = bl.is_large(run)
    if large and large_confirm != bl.confirmation_phrase(run):
        raise Refused(f"large run: large_confirm must be exactly {bl.confirmation_phrase(run)!r}")
    range_off = (axis == 2 and "steer_range" in bench.names) or (axis == 3 and "lift_range" in bench.names)
    advisories = []
    if kind == "distance" and abs(duty) > bl.HIGH_DUTY_DISTANCE:
        advisories.append("high_duty_distance_unverified")  # shown and recorded, never refused (lead 2026-11-28)
    same_setup = msg.get("same_setup_as_bundle", False)
    if not isinstance(same_setup, bool):
        raise Refused("same_setup_as_bundle: true or false")
    mark = range_mark(kind, axis, duty, b2["count_sign"] if kind != "duty" else None, same_setup)
    return Ask(axis, run, bench, tuple(names), window, range_off, large, source, setup, tuple(absent), kind=kind,
               source_wire="kth" if kind != "duty" else "none", advisories=tuple(advisories), range_mark=mark)


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
    # B2 (plan 4.4 decision 3)
    pairs: list = field(default_factory=list)  # (mcu_time_us, Mode, BenchState) of every pair while the run is on
    idents: list = field(default_factory=list)  # (boot_id, arm_gen) of each
    evidence: list = field(default_factory=list)  # the first frame's: (kind, STATE, BENCH_STATE, rx ns, conn)
    first_seq: int | None = None
    first_kind: str | None = None
    first_host_deadline_ns: int | None = None
    first_lim_mcu_us: int | None = None
    bench_start: str | None = None
    send_until_ns: int | None = None  # T - HANDOVER: no BENCH_HOLD or non-zero after it
    conn_gen_at_arm: int | None = None
    arm_ident: tuple | None = None  # (boot_id, arm_gen) of the arming that named this run
    pre_run_pair_t: int | None = None  # the last pair's time before the ARM (the entry window's zero base)
    zero_axis_set: bool = False
    board_end: bool = False
    board_cause: str | None = None
    cleanup_state: str | None = None  # confirmed / stop_unconfirmed / ... (apart from aborted, plan 4.4)
    brake_notice: bool = False
    rest_until_ns: int | None = None
    rest_abort: str | None = None
    rest_reopened: bool = False
    late_failures: list = field(default_factory=list)
    labels: list = field(default_factory=list)
    rest_closed: str | None = None
    arm_rx_conn: int | None = None
    pair_wait_noted: bool = False


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
    echoed: set = field(default_factory=set)  # this request's seqs a paired STATE of its epoch echoed (round 9 P2-2)


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
        # the host's opening count at receipt (plan 4.4: a new fd's first bytes reach the host before its connected
        # event reaches this console, so rows and evidence take the number the link itself raised)
        self.rx_conn = 0
        self.write_info: dict = {}  # seq -> the written frame's fields (the first answer's write record)
        orig_identity = self.host.link_identity

        def link_identity(product):
            self.rx_conn += 1
            return orig_identity(product)

        self.host.link_identity = link_identity
        self.host.on_pair = self._on_pair

    # --- the board's host callbacks -----------------------------------------------------------------------------------
    def _host_log(self, what, now_ns, data, info) -> None:
        fields = {}
        if info and what == "queued":
            seq = int(info["seq"]) & 0xFFFF
            self.fate.pop(seq, None)  # a seq queued again after the ring wrapped starts with no fate (round 4 P2-1)
            self.written.pop(seq, None)
            self.write_info.pop(seq, None)  # nor an earlier write's record (B2a-2 impl P2)
        if info and what in ("written", "dropped", "abandoned"):
            seq = int(info["seq"]) & 0xFFFF
            if what == "written":
                done = int(self.clock())
                self.written[seq] = (info["kind"], done)
                msg = None
                for rec in LinkDecoder().feed(bytes(data or b""), done):
                    msg = rec.message  # the command's own fields in the record (axis and duty of a COMMISSION)
                    fields = {k: getattr(msg, k) for k in ("axis", "duty_e4", "arm_gen", "boot_id") if hasattr(msg, k)}
                self.write_info[seq] = SimpleNamespace(
                    kind=info["kind"], seq=seq, done_ns=done, conn=self.rx_conn, boot_id=getattr(msg, "boot_id", None),
                    arm_gen=getattr(msg, "arm_gen", None), echo_mcu_time_us=getattr(msg, "echo_mcu_time_us", None),
                    valid_ms=getattr(msg, "valid_after_echo_ms", None))
            self.fate[seq] = what
            self.cleanup_seqs.discard(seq)
            if what == "written" and info.get("kind") == "COMMISSION" and fields.get("duty_e4"):
                if self.run is not None and fields.get("axis") == self.run.ask.axis:
                    self.run.nonzero_written = True  # a duty actually went out (no_drive vs result_lost)
            if what == "written" and info.get("kind") == "BENCH_HOLD" and self.run is not None:
                self.run.nonzero_written = True  # a hold went out: it may have driven
        r = self.run
        if r is not None and info and what in ("written", "dropped", "abandoned", "event"):
            extra = {}
            w = self.write_info.get(int(info["seq"]) & 0xFFFF) if what == "written" and "seq" in info else None
            if w is not None:
                extra = {"done_ns": w.done_ns, "conn": w.conn, "echo_mcu_time_us": w.echo_mcu_time_us,
                         "valid_ms": w.valid_ms}
            self._emit(r, {"t": what, "host_ns": int(self.clock()), **dict(info), **fields, **extra})

    def _on_state(self, st, received_ns: int) -> None:
        self.new_states.append((st, received_ns))
        self.recent_rx.append((st.boot_id, st.last_rx_seq & 0xFFFF))
        r = self.run
        if r is not None:
            self._emit(r, {**state_row(Sample(received_ns, st)), "conn": self.rx_conn})
            if r.first_seq is not None and r.bench_start is None:
                r.evidence.append(("state", st, None, received_ns, self.rx_conn))
            r.peak_current_ma = [max(a, abs(int(b))) for a, b in zip(r.peak_current_ma, st.current_ma)]
        if self.clear is not None:
            self._note_echo(self.clear, st, received_ns)
            try:
                self.clear.rec.emit({**state_row(Sample(received_ns, st)), "conn": self.conn})
            except RecordError:
                pass  # a clear's record is kept best-effort; the clear itself only stops

    def _on_pair(self, st, bst, received_ns: int) -> None:
        """Every pair the host accepted while a run is on, in the run's record (plan 4.4 decision 3: verdicts 2 and 3
        and the first answer recomputed from the JSONL alone), with the connection number at receipt."""
        r = self.run
        if r is None:
            return
        r.pairs.append((st.mcu_time_us, st.mode, bst))
        r.idents.append((st.boot_id, st.arm_gen, bst.conn_gen, self.rx_conn))
        if r.arm_written_ns is None and st.mode == Mode.DISARMED:
            r.pre_run_pair_t = st.mcu_time_us  # the last pair before the ARM (the entry window's zero base)
        if r.first_seq is not None and r.bench_start is None:
            r.evidence.append(("pair", st, bst, received_ns, self.rx_conn))
        self._emit(r, {"t": "bench_pair", "rx_ns": received_ns, "conn": self.rx_conn, "conn_gen": bst.conn_gen,
                       "mcu_time_us": st.mcu_time_us, "boot_id": st.boot_id, "arm_gen": st.arm_gen,
                       "mode": st.mode.name, "last_rx_seq": st.last_rx_seq, "run_id": bst.run_id,
                       "result_run_id": bst.result_run_id, "drive": bst.drive, "axis": bst.axis, "source": bst.source,
                       "count_state": bst.count_state, "raw_angle": bst.raw_angle, "cumulative": bst.cumulative,
                       "target": bst.target, "distance_ref": bst.distance_ref, "cut_position": bst.cut_position,
                       "stop_reason": bst.stop_reason, "distance_confirm": bst.distance_confirm,
                       "run_flags": bst.run_flags, "deadband_entries": bst.deadband_entries,
                       "evidence_age_ms": bst.evidence_age_ms, "end_evidence_age_ms": bst.end_evidence_age_ms,
                       "result_axis": bst.result_axis, "result_source": bst.result_source,
                       "result_bypass": bst.result_bypass, "blocked": bst.blocked})

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
            except Exception as exc:  # noqa: BLE001 -- one request never ends the console (plan 4.4, P1-4)
                return "bench_refused", {"detail": f"internal: {type(exc).__name__}: {exc}"}
            try:
                rec = RunRecord(self.runs_root, ask.kind, AXES[ask.axis])
            except OSError as exc:
                return "record_unavailable", {"detail": str(exc)}
            lease = self.arbiter.lease
            r = Run(request_id=msg["request_id"], ask=ask, rec=rec, lease_id=lease.lease_id,
                    owner_conn=self.arbiter.conns.get(lease.client_id), stops0=self.arbiter.stops,
                    control_gen=self.arbiter.control_gen, started_ns=now, phase_ns=now)
            self._emit(r, {"t": "request", "host_ns": now, "request_id": r.request_id, "msg": msg,
                           "product": self.host.usb_product, "names": list(ask.names), "large": ask.large,
                           "source_display": ask.source, "source_wire": ask.source_wire, "kind": ask.kind,
                           "advisories": list(ask.advisories), "range_mark": ask.range_mark,
                           "physical_setup": ask.physical_setup,
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
                why = self.not_ready()  # this fd's bench7 image and a pair after the boundary, as every command
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
        if why is not None:  # this fd's bench7 image and a pair after the boundary first (round 3 P1-4)
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
        """(host opening, board connection generation, boot, fault bits) now -- what a clear is bound to -- all from
        one completed pair, never a newer STATE's bits with the previous pair's generation (Codex C round 9 P2-1)."""
        ps, bst = self.host.bench_state_of, self.host.bench_state
        if ps is None or bst is None:
            return (self.conn, None, None, FaultFlag(0))
        return (self.conn, bst.conn_gen, ps.boot_id, FaultFlag(ps.fault_flags))

    def _continuing(self, seq: int) -> bool:
        """The link is in the middle of that frame (some of its bytes already out)."""
        link = getattr(self.dev, "link", None)
        cur = getattr(link, "_cur", None)
        return cur is not None and getattr(link, "_off", 0) > 0 and (int(cur.seq) & 0xFFFF) == seq

    def _holds(self, c: Clear) -> bool | None:
        """The clear's binding against the latest STATE: None while that STATE's pair is not in (its generation is not
        known yet: nothing is sent, nothing refused), else whether it still holds (round 9 P2-1)."""
        st = self.host.state
        if c is None or st is None:
            return False
        if (self.conn != c.binding.conn or st.boot_id != c.binding.boot_id
                or FaultFlag(st.fault_flags) != c.binding.fault_flags):
            return False  # what the STATE alone settles is judged before waiting for its pair (round 11 P2-1)
        if st is not self.host.bench_state_of:
            return None
        return c.binding.holds(*self._binding_now())

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
            # ended its arming there itself), and nothing to a board that is not this fd's bench7 image (round 3
            # P1-4) -- not a DISARM, not a zero
            return False
        if f.expires_ns is not None and f.expires_ns <= now:
            return False
        seq = int(f.seq) & 0xFFFF
        zero = f.kind == "COMMISSION" and not f.nonzero
        if seq in self.cleanup_seqs and (f.kind == "DISARM" or zero):
            c = self.clear
            if c is not None and f.kind == "DISARM" and seq == c.disarm_seq:
                held = self._holds(c)  # the clear's DISARM: its request still holds (r4 P2-3, r9 P2-1, r10 P2-3)
                return held is True or (held is None and self._continuing(seq))
            return True  # this session's own stop frame, by its seq and its kind (round 3 P1-1)
        arming_zero = f.echo is not None or (self.host.arming_zero_seq is not None
                                             and seq == self.host.arming_zero_seq & 0xFFFF)
        if zero and arming_zero and f.generation == self.host.auth_epoch:
            # the host's own arming zero (L, 0) for an ARM already out: a stop waits for it to land, then sends its
            # own zero -- refusing it would leave the arming open and the stop zero unqueueable (round 2 P1-2)
            return True
        if f.kind == "BENCH_ARM":
            r = self.run
            if r is None or r.arm_seq is None or seq != r.arm_seq or not self._final_before_arm(r):
                return False  # not this run's ARM, or the last run's result not written down (A1b-1b r2 P2-4)
            now = int(self.clock())  # the record took time: the lease and the rest judged at the write (r3 P2-3)
        if f.kind == "DISARM":
            return False  # no DISARM this session did not ask for
        if f.kind == "CLEAR_FAULTS":
            c, st = self.clear, self.host.state
            # the first byte on the latest STATE's own pair (round 9 P2-1); the rest of a frame already partly out is
            # not held for a pair still to come (holding it is abandoning it, round 10 P2-3), only stopped when the
            # binding no longer holds
            if c is None or c.clear_seq != seq or st is None:
                return False  # its request already closed: the link abandons the rest (round 11 P2-2)
            held = self._holds(c)
            return held is True or (held is None and self._continuing(seq))
        r = self.run
        if r is None or f.kind not in RUN_KINDS or r.phase in ("stop", "rest", "closed"):
            return False
        if self.cancel_reason(r, now) is not None:
            return False
        if f.kind in ("COMMISSION", "BENCH_HOLD") and f.generation != self.host.auth_epoch:
            return False
        if f.kind == "BENCH_HOLD" or (f.kind == "COMMISSION" and f.nonzero and r.ask.kind in ("hold", "distance")):
            # the one-shot first frame alone in 'first', the renewals in 'drive', none from T - HANDOVER on
            # (plan 4.4 decision 3; the CLI's gate, round 1 P1-1)
            if r.send_until_ns is not None and now >= r.send_until_ns:
                return False
            if r.phase == "first":
                return seq == r.first_seq and f.kind == r.first_kind
            if r.phase != "drive":
                return False
        return True

    def _final_before_arm(self, r: Run) -> bool:
        """Right before the BENCH_ARM's first byte (under the send boundary): the last result the host holds of the
        previous run is written down (again when it changed), else the ARM, which clears it, does not go out."""
        prev, bst = r.previous_result, self.host.bench_state
        if prev is None or bst is None or bst.result_run_id != prev["run_id"]:
            return True
        fields = bringup.recovered_fields(bst)
        if all(prev.get(k) == v for k, v in fields.items()) and prev.get("t") == "bench_previous_result_final":
            return True
        final = {"t": "bench_previous_result_final", "boot_id": prev["boot_id"], **fields}
        self._emit(r, final)
        if r.cancel is not None:
            return False  # not written down: the run is cancelled and the ARM never goes out
        r.previous_result = final  # result.json carries the final value (impl r1 P2-5)
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
        self._restore_zero_axis(r)
        if r.phase == "rest":
            self._rest(r, now)
            return
        if r.phase not in ("stop", "closed"):
            why = self.cancel_reason(r, now)
            if why is not None:
                self._enter_stop(r, why, now)
        elif r.phase == "stop":
            # a STOP, a lost lease or connection during the run's own end changes nothing that is sent, but the run
            # was interrupted: it is not a pass (round 4 P2-4); after another abort it is kept apart, so a late board
            # record never clears it (B2a-2 impl P2)
            why = self.cancel_reason(r, now)
            if why is not None and r.aborted is None:
                r.aborted = why
            elif why is not None and why != r.aborted and why not in r.late_failures:
                r.late_failures.append(why)
        if r.phase in ("recover", "set", "arm", "prep", "first"):
            # every sample from the request to the first non-zero is judged (round 3 P2-4; plan 4.4: 'first' too)
            v = self._window_out(r, states)
            if v is not None:
                if r.arm_seq is None:
                    self._refuse(r, f"window_before_drive:{v}", now)
                    return
                r.window_stop = f"window:{v}"
                r.late_failures.append(r.window_stop)  # a late board record never clears it
                self._enter_stop(r, f"window_before_drive:{v}", now)
        if r.phase == "recover":
            self._recover(r, now)
        elif r.phase == "set":
            self._set(r, now)
        elif r.phase == "arm":
            self._arm(r, now)
        elif r.phase == "prep":
            self._prep(r, now)
        elif r.phase == "first":
            self._first(r, now, states)
        elif r.phase == "drive":
            self._drive(r, now, states)
        if r.phase == "stop":
            self._zero_written(r)  # its write time first: a STATE received after it never speaks for the first frame
            self._classify_first(r, now)
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
            # every result field (a CLI distance run's confirmation, cut and flags too: plan 4.2 A1b-1 decision 7)
            prev = {"t": "bench_previous_result", "boot_id": p.boot_id, "run_id": p.result_run_id,
                    "stop_reason": _stop_name(p.stop_reason)}
            bst = self.host.bench_state
            if bst is not None and bst.result_run_id == p.result_run_id:
                prev.update(bringup.recovered_fields(bst))  # a hold's: undecidable (no settings or samples)
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
        blocked = self.host.bench_set(r.run_id, a.bench, now, count_sign=a.bench.count_sign, source=a.bench.source,
                                      stop_counts=a.bench.stop_counts)
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
        if r.ask.kind == "hold":
            # the hold image refuses L(0) in a hold run: the readiness zero on the run axis (plan 4.3 decision 10)
            self.host.commission_zero_axis = r.ask.axis
            r.zero_axis_set = True
        blocked = self.host.request_bench_arm(r.run_id, now)
        if blocked is not None:
            self._restore_zero_axis(r, force=True)
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
        r.conn_gen_at_arm, r.arm_ident, r.arm_rx_conn = p.conn_gen, (p.boot_id, p.arm_gen), self.rx_conn
        r.bench_taken = {"seq": r.set_seq, "run_id": r.run_id, "bypass": list(r.ask.bench.names),
                         "duty_max_e4": list(r.ask.bench.duty_max_e4()), "time_ms": list(r.ask.bench.time_ms()),
                         "overrides": r.ask.bench.overrides(), "mode": r.ask.bench.mode, "source": r.ask.bench.source,
                         "count_sign": r.ask.bench.count_sign, "stop_counts": r.ask.bench.stop_counts}
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
        if self._over(r, now, PREP_WAIT_S):
            # the deadline before any acceptance (plan 4.4, independent round 3 P2-2)
            self._enter_stop(r, "prep_zero_not_taken", now)
            return
        st, rx = self.host.state, self.host.state_rx_ns
        w = self.write_info.get(r.prep_seq)
        taken = (st is not None and w is not None and w.kind == "COMMISSION" and rx is not None and rx >= w.done_ns
                 and w.conn == self.rx_conn and st.boot_id == r.boot and r.arm_ident is not None
                 and st.arm_gen == r.arm_ident[1] and (st.last_rx_seq & 0xFFFF) == r.prep_seq)
        if not taken:
            return
        if st.reject_reason != RejectReason.NONE or st.mode != Mode.ARMED:
            self._enter_stop(r, f"prep_zero_refused:{st.reject_reason.name}", now)
            return
        r.drive_start_ns = now
        r.drive_end_ns = now + int(r.ask.run.seconds * 1e9)  # T, fixed once (plan 4.4)
        r.send_until_ns = r.drive_end_ns - int(bringup.HANDOVER_S * 1e9)
        r.faults_at_drive = int(st.fault_flags)
        if r.ask.kind in ("hold", "distance"):
            self._send_first(r, now)
        else:
            self._phase(r, "drive", now)

    def _send_first(self, r: Run, now: int) -> None:
        """A hold's first BENCH_HOLD or a distance run's first non-zero, once (plan 4.3 decision 10, 4.4 decision 3):
        its host deadline fixed when it is queued, whether or not a byte goes out."""
        a = r.ask
        if a.kind == "hold":
            errors0 = r.record_errors
            self._emit(r, {"t": "hold_settings", "run_id": r.run_id, **bringup._hold_settings(a.bench)})
            if r.record_errors != errors0:
                self._enter_stop(r, "hold_settings_not_recorded", now)  # verdict 2 needs them: no HOLD without
                return
        echo_rx = self.host.state_rx_ns
        expires = min(now + self.host.cfg.cap_ms * 1_000_000, r.drive_end_ns)
        if a.kind == "hold":
            blocked = self.host.request_bench_hold(r.run_id, a.axis, a.bench.hold, a.bench.count_sign, expires, now)
            kind = "BENCH_HOLD"
        else:
            blocked = self.host.commission(a.axis, int(round(a.run.duty * 10000)), expires, now)
            kind = "COMMISSION"
        if blocked is not None:
            r.aborted = r.aborted or "bench_start_not_queued"
            self._enter_stop(r, f"first_not_queued:{blocked.reason}", now)
            return
        r.first_seq, r.first_kind = self.host.last_queued_seq & 0xFFFF, kind
        try:
            valid = min(self.host.cfg.cap_ms, valid_after_echo_ms(expires, echo_rx))
        except (TypeError, ValueError):
            valid = self.host.cfg.cap_ms
        r.first_host_deadline_ns = (echo_rx or now) + valid * 1_000_000 + 50_000_000
        r.evidence = []
        r.last_send_ns = now
        self._emit(r, {"t": "first_frame", "kind": kind, "seq": r.first_seq, "conn": self.rx_conn,
                       "echo_rx_ns": echo_rx, "host_deadline_ns": r.first_host_deadline_ns})
        self._phase(r, "first", now)

    def _classify_first(self, r: Run, now: int) -> str | None:
        """The first frame's answer from what the host accepted (bench_session.first_answer); judged in 'first' and,
        after a stop, until its deadlines (classified only: nothing more is sent for it)."""
        if r.first_seq is None or r.bench_start is not None:
            return r.bench_start
        w = self.write_info.get(r.first_seq)
        if w is not None and (w.kind != r.first_kind or w.echo_mcu_time_us is None or w.valid_ms is None):
            w = None
        if w is not None and r.first_lim_mcu_us is None:
            r.first_lim_mcu_us = w.echo_mcu_time_us + w.valid_ms * 1000 + int(bringup.FIRST_NONZERO_SLACK_S * 1e6)
        v = bs.first_answer(r.evidence, seq16=r.first_seq, run_id=r.run_id, write=w,
                            lim_mcu_us=r.first_lim_mcu_us or 0, lim_host_ns=r.first_host_deadline_ns, now_ns=now,
                            stop_zero_done_ns=r.zero_written_ns)
        if v is None:
            return None
        r.bench_start = v
        self._emit(r, {"t": "bench_start", "result": v, "seq": r.first_seq, "lim_mcu_us": r.first_lim_mcu_us,
                       "host_ns": now, "stop_zero_done_ns": r.zero_written_ns})
        if v.startswith("refused:"):
            r.aborted = r.aborted or f"bench_start_refused_{v.split(':', 1)[1]}"
        elif v == "unconfirmed":
            r.aborted = r.aborted or "bench_start_unconfirmed"
        return v

    def _first(self, r: Run, now: int, states: list) -> None:  # noqa: ARG002 -- the window is judged in _step
        ans = self._classify_first(r, now)
        if ans is None:
            if r.send_until_ns is not None and now >= r.send_until_ns:
                self._end(r, now)  # the end zero is never held for the first answer (plan 4.4, r1 P1-2)
            return
        if ans in ("accepted", "confirmed_by_result"):
            self._phase(r, "drive", now)
            if r.send_until_ns is not None and now >= r.send_until_ns:
                self._end(r, now)  # the end zero in this same cycle (Codex B2a-2 impl round 2 P1)
            return
        self._enter_stop(r, None, now)

    def _ended_by_board(self, r: Run) -> bool:
        """This arming's pair carries this run's result: the board ended the run itself (plan 4.4 decision 3)."""
        p = self.pair()
        if p is None or r.run_id is None or r.arm_ident is None or (p.boot_id, p.arm_gen) != r.arm_ident:
            return False
        return p.result_run_id == r.run_id and p.drive_state == BenchDriveState.ENDED

    def _drive(self, r: Run, now: int, states: list) -> None:
        a = r.ask
        if a.kind in ("hold", "distance") and self._ended_by_board(r):
            self._end(r, now)  # the board's own end (distance 8-12, hold 10-12, a limit, an expiry)
            return
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
            if a.kind == "hold":
                queued = self.host.request_bench_hold(r.run_id, a.axis, a.bench.hold, a.bench.count_sign, expires,
                                                      now)
            else:
                queued = self.host.commission(a.axis, int(round(a.run.duty * 10000)), expires, now)
            if queued is None:
                r.last_send_ns = now

    def _end(self, r: Run, now: int) -> None:
        """The run's own end: one zero (the board stops itself, contract 8.3), then the stop rule's wait."""
        self._enter_stop(r, None, now)

    def _enter_stop(self, r: Run, why: str | None, now: int) -> None:
        if r.phase in ("stop", "closed"):
            return
        r.aborted = why if why is not None else r.aborted
        self.host.drop_frames(("BENCH_SET", "BENCH_HOLD"), f"stop: {why}")
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

    def _after_arm_state(self, r: Run, st) -> bool:
        """A STATE made after the BENCH_ARM's fate was decided (its arming moved on, or past the ARM's acceptance
        window on the same boot) -- the STATE-only form of ``_after_arm``."""
        if r.arm_written_ns is None or st.boot_id != r.boot:
            return False
        return st.arm_gen != r.arm_prev_gen or st.mcu_time_us > r.arm_echo_us + BOARD_COMMAND_TIMEOUT_MS * 1000

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
                self._emit(r, {"t": "stop_zero_written", "seq": seq, "host_ns": w[1], "echo_us": echo_us,
                               "link": None if link is None else list(link)})
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
        st = self.host.state
        same_arming = p is not None and r.arm_ident is not None and (p.boot_id, p.arm_gen) == r.arm_ident
        if same_arming and r.ended_seen and not r.board_end:
            r.board_end, r.board_cause = True, _stop_name(p.stop_reason)
            self._emit(r, {"t": "board_end", "stop_reason": r.board_cause, "host_ns": now})
        # the latest STATE of the run's boot already out of ARMED (STOPPING, DISARMED, FAULT, ESTOP) after the ARM
        # went out: no zero, resend or DISARM from an older pair (Codex B2a-2 impl P1-2); closing still waits for an
        # end pair
        latest_stopping = (st is not None and arm_sent and st.boot_id == r.boot and st.mode in STOPPING_LIKE
                           and self._after_arm_state(r, st))
        if r.board_end or latest_stopping:
            # the board is ending the run itself: no stop frame of ours goes out any more -- queued ones leave the
            # queue, a partly written one is refused at the gate (abandoned with the poison) (plan 4.4, r2 P1-1)
            if self.cleanup_seqs:
                self.cleanup_seqs.clear()
                self.host.drop_frames(("COMMISSION", "DISARM"), "board_ending", keep_arming_zero=True)
        if (st is not None and st.mode == Mode.STOPPING and not r.brake_notice
                and now - r.stop_entered_ns > BRAKE_NOTICE_S * 1e9):
            r.brake_notice = True
            self._emit(r, {"t": "brake_end_unseen", "host_ns": now})
        if (latest_stopping and not r.ended_seen and not r.pair_wait_noted
                and now - r.stop_entered_ns > bringup.RESULT_PAIR_WAIT_S * 1e9):
            r.pair_wait_noted = True  # recorded only: nothing is sent for it (plan 4.4, the three waits)
            self._emit(r, {"t": "result_pair_absent", "after_s": bringup.RESULT_PAIR_WAIT_S, "host_ns": now})
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
                r.cleanup_state = "stop_unconfirmed_after_reopen"
                r.aborted = r.aborted or "stop_unconfirmed_after_reopen"
                self._close(r, now, None)
            return
        self._zero_written(r)
        after_zero = (r.zero_seq is not None and p is not None and self.pair_rx_ns is not None
                      and self.pair_rx_ns > r.zero_written_ns and p.mcu_time_us > r.zero_echo_us)
        arming = self.host.arming or (self.host.authorized and not self.host.ready)
        action = bs.stop_action(arm_sent=arm_sent, zero_seq=r.zero_seq, zero_link=r.zero_link, p=p, arming=arming,
                                after_arm=self._after_arm(r, p), after_zero=after_zero,
                                board_end=r.board_end, latest_stopping=latest_stopping)
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
                    # connection generation from the latest pair, or the arming's when no pair is in (plan 4.4, r5
                    # P1-1: a pairless zero is still matched against a later pair of the same arming)
                    conn_gen = p.conn_gen if p is not None else r.conn_gen_at_arm
                    link = (st.boot_id, conn_gen, st.arm_gen) if st is not None and conn_gen is not None else None
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
            r.cleanup_state = "stop_unconfirmed"  # apart from the host's abort and the board's cause (plan 4.4)
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

    def _latched_result(self, r: Run):
        """The last recorded BENCH_STATE of this run's boot and arming carrying its result (any host opening)."""
        if r.arm_ident is None or r.run_id is None:
            return None
        return next((b for (_t, _m, b), idn in zip(reversed(r.pairs), reversed(r.idents), strict=True)
                     if idn[:2] == r.arm_ident and b.result_run_id == r.run_id), None)

    def _no_drive_by_end_pair(self, r: Run) -> bool:
        """An end pair (DISARMED, FAULT, ESTOP) of this run's boot and arming with no run result: the arming ended
        without the run having driven (plan 4.4 decision 3, '구동 없음 확인')."""
        if r.arm_ident is None:
            return False
        return any(m in STOPPED and idn[:2] == r.arm_ident and b.result_run_id == 0
                   for (_t, m, b), idn in zip(r.pairs, r.idents, strict=True))

    def _restore_zero_axis(self, r: Run, *, force: bool = False) -> None:
        """The readiness zero's axis back to L once the ARM's exchange is over (plan 4.4, r1 P1-4): never while the
        host is still arming or waiting for that zero -- an L(0) made then would be refused by the hold image."""
        if not r.zero_axis_set:
            return
        if force or not (self.host.arming or (self.host.authorized and not self.host.ready)):
            self.host.commission_zero_axis = 0
            r.zero_axis_set = False

    def _rest(self, r: Run, now: int) -> None:
        """After DISARMED, a KTH run's rest position (plan 4.2 decision 4, 4.4): nothing is sent and nothing cancels
        it -- a STOP, the lease or an exit only noted; a reopening ends it unconfirmed."""
        why = self.cancel_reason(r, now)
        if why is not None and r.rest_abort is None:
            r.rest_abort = why
        if r.link_reopened:
            r.rest_reopened = True
        if r.link_reopened or now >= r.rest_until_ns:
            self._close(r, now, r.rest_closed, rest_done=True)

    def _close(self, r: Run, now: int, closed: str | None, *, rest_done: bool = False) -> None:
        if not rest_done and r.ask.kind != "duty" and r.cleanup_confirmed and not r.link_reopened:
            # the rest position first (0.6 s of DISARMED pairs), the verdicts after it
            self.host.drop_frames(("COMMISSION", "DISARM", "BENCH_SET", "BENCH_ARM", "BENCH_HOLD"), "run resting")
            r.rest_closed = closed
            r.rest_until_ns = now + int(bringup.REST_OBSERVE_S * 1e9)
            self._phase(r, "rest", now)
            return
        p = self.pair()
        latched = self._latched_result(r)
        # the run's own frames still queued go with it (round 4 P2-2): an unsent stop DISARM would otherwise be
        # refused at every gate and handed back to the queue head, in front of the next run's BENCH_SET
        self.host.drop_frames(("COMMISSION", "DISARM", "BENCH_SET", "BENCH_ARM", "BENCH_HOLD"), "run closed")
        self._restore_zero_axis(r, force=True)
        result_seen = latched is not None or (p is not None and r.run_id is not None and p.result_run_id == r.run_id)
        stop_reason = (_stop_name(latched.stop_reason) if latched is not None
                       else _stop_name(p.stop_reason) if result_seen else None)
        if closed is None:
            if result_seen:
                closed = "ended"  # the board drove this run and recorded how it ended
            elif self._no_drive_by_end_pair(r):
                closed = "no_drive"  # this arming's end pair carries no run result: it never drove (plan 4.4)
            elif r.nonzero_written:
                closed = "result_lost"  # a duty or a hold went out but the board's record of the run was not read
            else:
                closed = "no_drive"
        st = self.host.state
        if (p is not None and st is not None and st.boot_id == p.boot_id and st.mcu_time_us >= p.mcu_time_us
                and (r.arm_ident is None or st.arm_gen == p.arm_gen)):
            # the verdict reads the latest STATE of the same arming too: a FAULT after the last pair is not a clean
            # end (round 4 P2-4; plan 4.4: the same arming only)
            p = replace(p, mode=st.mode, fault_flags=st.fault_flags, duty_e4=tuple(st.duty_e4))
        if closed == "ended" and r.window_stop is not None:
            r.aborted = r.aborted or r.window_stop
        faults = []
        if p is not None:
            # the bench's expected status bits left out (plan 4.4 B2, independent review P2-5): a motor-only M's range
            # bit with its bypass, an unfitted H's sensor bit
            faults = list(bs.unexpected_faults(p.fault_flags, r.ask.axis, r.ask.bench.names))
        b2 = self._b2_verdicts(r, p, closed, result_seen)
        if b2.get("closed") is not None:
            closed = b2.pop("closed")
        result = {"t": "bench_result", "request_id": r.request_id, "run": r.rec.dir.name,
                  "record": str(r.rec.dir), "axis": AXES[r.ask.axis], "duty": r.ask.run.duty,
                  "seconds": r.ask.run.seconds, "closed": closed, "run_id": r.run_id, "bench_set": r.bench_taken,
                  "stop_reason": stop_reason, "aborted": r.aborted, "cleanup_confirmed": r.cleanup_confirmed,
                  "previous_result": r.previous_result,
                  "bypass_mcu": list(r.bench_taken["bypass"]) if r.bench_taken else [], "bypass_host": [],
                  "not_for_gate": True, "physical_setup": r.ask.physical_setup,
                  "peak_current_ma": list(r.peak_current_ma),  # from the run's own STATEs, not the screen stream
                  "final": None if p is None else {"mode": p.mode.name, "duty_e4": list(p.duty_e4)},
                  "kind": r.ask.kind, "board_cause": r.board_cause,
                  "cleanup": r.cleanup_state or ("confirmed" if r.cleanup_confirmed else "unconfirmed"),
                  "advisories": list(r.ask.advisories), "range_mark": r.ask.range_mark,
                  "notes": [n for n, on in (("brake_end_unseen", r.brake_notice),
                                            ("result_pair_absent", r.pair_wait_noted)) if on],
                  "late_aborts": list(r.late_failures), **b2}
        result["closed"] = closed
        r.phase = "closed"
        try:
            r.rec._f.flush()  # every row before the verdict counts (plan 4.4 decision 4)
        except (OSError, ValueError):
            r.record_errors += 1
        verdict = bs.run_verdict(closed=closed, stop_reason=stop_reason, aborted=r.aborted,
                                 cleanup_confirmed=r.cleanup_confirmed, last=p, record_errors=r.record_errors,
                                 unexpected_faults=tuple(faults), distance=r.ask.kind == "distance")
        result["verdict"] = {"run_cleanup": {**verdict, "labels": list(r.labels)}}
        result["record_errors"] = r.record_errors
        # the final row after the verdict, in one write; its own fate apart (plan 4.4 decision 4)
        complete = r.rec.final_row({**result, "t": "result"})
        result["record_complete"] = complete
        if not complete:
            result["final_record_error"] = True
        else:
            try:
                r.rec.result(result)  # a copy of the final row, never read for the verdict
            except OSError as exc:
                result["copy_error"] = str(exc)
        r.rec.close()
        self.last_result = result
        self._put(result)
        self.run = None
        self.cleanup_seqs.clear()  # the next run's stop frames are its own
        st = self.host.state
        if st is not None and st.mode in STOPPED and self.arbiter.mode == "STOPPED":
            self.arbiter.release_stop(now / 1e9, "bench_stop_confirmed")

    def _b2_verdicts(self, r: Run, p, closed: str, result_seen: bool) -> dict:
        """Verdicts 2 and 3 and the B2 labels from this run's own record (plan 4.4 decision 4): the pairs of this
        arming, the board's result as last read, the rest position."""
        a, out = r.ask, {}
        if a.kind == "duty":
            return out
        ident = r.arm_ident
        link = (r.conn_gen_at_arm, r.arm_rx_conn)

        def ours(idn) -> bool:
            # this arming on this host opening and board connection (B2a-2 impl P2: a pair after a reopening is
            # never joined to the run's)
            return ident is not None and idn[:2] == ident and idn[2:] == link

        mine = [(t, m, b) for (t, m, b), idn in zip(r.pairs, r.idents, strict=True) if ours(idn)]
        # the board's latched result from the run's record of this boot and arming, on any opening (the host's kept
        # pair may be gone after a loss; the board keeps its record across a reopening) -- Codex B2a-2 r2 P2-2
        res = self._latched_result(r)
        br = {"confirmed": True, **bringup.result_fields(res)} if res is not None else {"confirmed": False}
        out["bench_result"] = br
        out["start"] = r.bench_start
        driving = any(m == Mode.ARMED and (b.drive & 0x3) == int(BenchDriveState.DRIVING) and b.run_id == r.run_id
                      for _t, m, b in mine)
        rest = [(t, b) for t, m, b in mine if m == Mode.DISARMED and b.axis == a.axis and b.source == a.bench.source]
        out["rest"] = None if r.rest_reopened else bs.rest_position(rest)
        if r.rest_abort is not None or r.rest_reopened:
            out["rest_note"] = r.rest_abort or "reopened"
        if (a.kind == "distance" and r.aborted == "bench_start_unconfirmed" and br.get("confirmed")
                and br.get("stop_reason") == "distance_reached" and p is not None and p.mode == Mode.DISARMED
                and ident is not None and (p.boot_id, p.arm_gen) == ident and not r.late_failures):
            r.aborted = None
            r.labels.append("첫 응답 미확인 — 보드 기록으로 끝 확인")
        if a.kind == "distance" and br.get("confirmed") and not driving:
            r.labels.append("보드 기록만(구동 표본 없음)")
            out["drive_samples"] = 0
            out["peak_current_ma"] = None  # the readiness and brake currents are no drive observation (plan 4.4)
            if closed in ("no_drive", "result_lost"):
                out["closed"] = "ended"  # the board's record of this arming says it drove (plan 4.4 decision 4)
        if a.kind == "distance" and br.get("confirmed"):
            out["distance"] = bs.distance_verdict(
                stop_reason=br.get("stop_reason"), confirm=br.get("distance_confirm", 0),
                run_flags=br.get("run_flags", 0), stop_counts=a.bench.stop_counts,
                distance_ref=br.get("distance_ref", 0), cut_position=br.get("cut_position", 0))
        if a.kind == "measure" and br.get("confirmed"):
            out["measure"] = bs.measure_verdict(run_flags=br.get("run_flags", 0), distance_ref=br.get("distance_ref", 0),
                                                cut_position=br.get("cut_position", 0))
        if a.kind == "hold":
            sess = SimpleNamespace(bench=a.bench, run_ident=ident, run_id=r.run_id, pre_run_pair_t=r.pre_run_pair_t)
            v2, entries, n = bringup._hold_verdict2(sess, [x for x, idn in zip(r.pairs, r.idents, strict=True)
                                                           if ours(idn)], [ident] * len(mine), br)
            out["hold"] = {"settings": bringup._hold_settings(a.bench), "verdict": v2, "entries": entries, "pairs": n,
                           "notes": ["듀티는 펌웨어가 낸 명령값 — 실제 PWM·브리지 출력은 미확인", "정지 폭 미측정",
                                     "구간 시작 관성 혼입 미확인"]}
        return out

    # --- the clear ----------------------------------------------------------------------------------------------------
    def _seen_after(self, seq: int | None) -> bool:
        """A STATE received after that frame's last byte went out (never one made before it)."""
        w = self.written.get(seq) if seq is not None else None
        return w is not None and self.host.state_rx_ns is not None and self.host.state_rx_ns > w[1]

    def _same_epoch(self, c: Clear) -> bool:
        """The latest STATE is of the clear's own epoch: paired with its BENCH_STATE (so its board connection generation
        is known, not the previous pair's) on the same host opening, generation and boot (Codex C round 8 P2)."""
        st, bst = self.host.state, self.host.bench_state
        return (st is not None and bst is not None and self.host.bench_state_of is st
                and self.conn == c.binding.conn and bst.conn_gen == c.binding.conn_gen
                and st.boot_id == c.binding.boot_id)

    def _clear_done(self, c: Clear, result: str, now: int, why: str | None = None) -> None:
        st = self.host.state
        last = c.clear_seq if c.clear_seq is not None and c.clear_seq in self.written else c.disarm_seq
        sent = last is not None and last in self.written
        # the state after the attempt: only one received after its last frame (Codex C round 5 P2-2); before any
        # frame went out, the state as it is. An unconfirmed clear has none: a STATE received after the CLEAR may
        # have been made before the board read it (round 6 P2)
        # and never one of another opening, generation or boot, nor a STATE whose pair is not in yet (round 8 P2)
        after = None if st is None or not self._same_epoch(c) or result == "unconfirmed" \
            or (sent and not self._seen_after(last)) else _latch_view(st)
        out = {"t": "clear_result", "request_id": c.request_id, "clear": c.rec.dir.name, "result": result,
               "why": why, "cause": c.cause, "before": c.before, "after": after}
        self.host.drop_frames(("CLEAR_FAULTS",) if result == "cleared" else ("CLEAR_FAULTS", "DISARM"),
                              f"clear: {result}")  # a refused request's own unsent frames go with it (round 3 P2-3)
        if c.disarm_seq is not None and c.disarm_seq not in self.written:
            # its DISARM, even one partly out, loses its leave with the request: the link abandons the rest (r12 P2)
            self.cleanup_seqs.discard(c.disarm_seq)
        try:
            c.rec.emit(out)
            c.rec.result(out)
        except (RecordError, OSError):
            pass
        c.rec.close()
        self._put(out)
        self.clear = None

    def _note_echo(self, c: Clear, st, received_ns: int) -> None:
        """Every STATE the host accepts, each in its read (so two pairs in one read never hide the first one's echo,
        Codex C round 10 P2-2): one received after the request's own frame went out whole, naming its seq, of the
        request's boot and host opening, is that frame's reply, kept (round 9 P2-2). Never one received before the
        frame went out (round 10 P2-1). Its board generation is not asked: a frame the board answered was taken
        whatever the generation, and a generation change is the boundary check's (the clear then unconfirmed)."""
        seq = st.last_rx_seq & 0xFFFF
        if seq not in (c.disarm_seq, c.clear_seq):
            return
        kind = "DISARM" if seq == c.disarm_seq else "CLEAR_FAULTS"
        w = self.written.get(seq)
        if (w is not None and w[0] == kind and received_ns > w[1] and st.boot_id == c.binding.boot_id
                and self.conn == c.binding.conn and st.mcu_time_us > c.floor_us):
            c.echoed.add(seq)

    def _echoed(self, seq: int, kind: str, c: Clear) -> bool:  # noqa: ARG002 -- kind checked when it was noted
        return seq is not None and seq in c.echoed

    def _step_clear(self, c: Clear, now: int) -> None:
        st = self.host.state
        conn_gen = None if self.host.bench_state is None else self.host.bench_state.conn_gen
        if self.conn != c.binding.conn or conn_gen != c.binding.conn_gen or st is None \
                or st.boot_id != c.binding.boot_id:
            if c.clear_seq is not None and self.fate.get(c.clear_seq) == "written":
                # the CLEAR went out whole and its reply was not seen: the board may have taken it (round 7 P2-2)
                self._clear_done(c, "unconfirmed", now, "the link or the boot changed after the CLEAR, no reply seen")
            else:
                self._clear_done(c, "refused", now, "the link or the boot changed: ask again")
            return
        if c.phase == "disarm":
            if c.disarm_seq is None:
                held = self._holds(c)
                if held is False:
                    # judged before the DISARM too (round 2 P2-3): a request asked on other bits is not run
                    self._clear_done(c, "refused", now, "the fault bits changed before the DISARM: ask again")
                    return
                if held is None:  # the latest STATE's pair not in yet: queue nothing this cycle (round 9 P2-1)
                    if now - c.phase_ns > CLEAR_ECHO_S * 1e9:
                        self._clear_done(c, "refused", now, "no complete STATE pair to bind the clear to: ask again")
                    return
                seen = {s for _b, s in self.recent_rx} | {0}
                while (self.host.skip_seq(0) & 0xFFFF) in seen:  # a number no recent STATE named, before queueing
                    self.host.skip_seq(1)
                self.host.request_disarm(now)
                c.disarm_seq = self.host.queued_disarm_seq()
                self.cleanup_seqs.add(c.disarm_seq)
                c.phase_ns = now
            elif self._pending(c.disarm_seq) and self._holds(c) is False:
                # still unsent and the request no longer holds: refused, its DISARM out of the queue (round 4 P2-3)
                self._clear_done(c, "refused", now, "the board changed before the DISARM went out: ask again")
                return
            elif self._echoed(c.disarm_seq, "DISARM", c) and self._holds(c) is not None:
                if not self._holds(c):
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
                    and self._holds(c) is False:
                # still unsent and the bits changed: refused for good, the frame out of the queue (round 3 P2-3)
                self._clear_done(c, "refused", now, "the fault bits changed before the CLEAR: ask again")
                return
            if self._echoed(c.clear_seq, "CLEAR_FAULTS", c):
                c.phase, c.phase_ns = "ready", now
            elif now - c.phase_ns > CLEAR_ECHO_S * 1e9:
                # no echo of this CLEAR: a refusal leaves none of its own (reject_reason is the board's last one,
                # last_rx_seq unchanged), and a board out of FAULT may be an old host's late CLEAR (contract 8.4), so
                # this request is unconfirmed, what was seen only in its why (rounds 6 and 7 P2)
                if not self._seen_after(c.clear_seq):
                    self._clear_done(c, "unconfirmed", now, "no STATE after the CLEAR")
                elif st.mode in (Mode.FAULT, Mode.ESTOP):
                    self._clear_done(c, "unconfirmed", now, f"no reply to the CLEAR (the board still {st.mode.name}, "
                                                            f"its last reject {st.reject_reason.name})")
                else:
                    self._clear_done(c, "unconfirmed", now, f"no echo of this CLEAR; the board is {st.mode.name} "
                                                            "(a CLEAR worked, not shown to be this one)")
        elif c.phase == "ready":
            if st.mode == Mode.DISARMED and st.flags & StateFlag.DRIVERS_READY and self._same_epoch(c):
                self._clear_done(c, "cleared", now)
            elif now - c.phase_ns > CLEAR_READY_S * 1e9:
                self._clear_done(c, "not_ready", now, f"after the clear: {st.mode.name}")

    # --- the stream ---------------------------------------------------------------------------------------------------
    def _b2_stream(self, r: Run, now: int) -> dict:
        """The run's B2 values for the screen (plan 4.4 decision 5): its kind and marks, and the count values only once
        they are this run's -- this arming's pair after its first frame was answered; null before and across a gap."""
        a = r.ask
        out = {"kind": a.kind, "advisories": list(a.advisories), "range_mark": a.range_mark, "start": r.bench_start,
               "target": None, "error_counts": None, "deadband": None, "distance_ref": None, "moved_counts": None,
               "deadband_entries": None, "arming": None, "pair_valid": False}
        if a.kind == "hold":
            out["deadband"] = a.bench.hold.deadband
        p, bst = self.pair(), self.host.bench_state
        started = a.kind == "measure" or r.bench_start in ("accepted", "confirmed_by_result")
        own = (p is not None and bst is not None and r.arm_ident is not None and (p.boot_id, p.arm_gen) == r.arm_ident
               and started and bst.run_id == r.run_id and bst.axis == a.axis and bst.source == a.bench.source
               and (p.drive_state == BenchDriveState.DRIVING
                    or (p.drive_state == BenchDriveState.ENDED and bst.result_run_id == r.run_id))
               and self.pair_rx_ns is not None and now - self.pair_rx_ns <= int(bs.HOLD_GAP_US * 1000) * 2)
        if a.kind in ("hold", "distance", "measure"):
            out["arming"] = list(r.arm_ident) if r.arm_ident else None
        if not own:
            return out
        out["pair_valid"] = True
        if a.kind == "hold":
            out["target"] = bst.target
            out["error_counts"] = bs.counts_diff(bst.target, bst.cumulative)
            out["deadband_entries"] = bst.deadband_entries
        elif a.kind in ("distance", "measure"):
            out["distance_ref"] = bst.distance_ref
            out["moved_counts"] = bs.counts_diff(bst.cumulative, bst.distance_ref)
        return out

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
            "advice": self._advice(r.ask), **self._b2_stream(r, now),
            "notes": [n for n, on in (("brake_end_unseen", r.brake_notice), ("result_pair_absent", r.pair_wait_noted))
                      if on]}
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


MODEL_PRODUCT = "forklift-mcu commission OPEN model rec=00000000 lim=lr1 bench7"


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
        r.aborted = r.aborted or "unconfirmed_at_exit"  # the first cause stays (tool_exit once a cycle ran)
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
