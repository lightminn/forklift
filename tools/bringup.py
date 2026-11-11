#!/usr/bin/env python3
"""S1–S6 bring-up host tool ① (docs/design/2026-11-12-bringup-host-tools.md): record, cpr, step, scenario, analyze.

Not an executor (mcu-link/2 contract 3.3): it connects to any build and records the build bits, faults and reset
cause; it drives only under the design's section 3 conditions. Everything it measures is software evidence (STATE):
the S5 pass verdict takes external measurements (oscilloscope, floor distance) given on the command line, or leaves
an item 'unavailable'.

usage:
  bringup.py record   --device D --out run.jsonl [--seconds 30]
  bringup.py cpr      --device D --out run.jsonl --turns-left N --turns-right N [--dir-left +1] [--dir-right +1]
                      [--motor-counts-confirmed]
  bringup.py step     --device D --out run.jsonl --wheel left|right|both --target RAD_S [--yes]
  bringup.py scenario --device D --out run.jsonl --kind tx_cut|disarm|expiry [--rate RAD_S] [--yes]
                      [--output-response-ms X|unavailable] [--stop-time-ms X|unavailable]
                      [--stop-distance-mm X|unavailable] [--floor]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from forklift_core.mcu_link.messages import LinkDecoder, McuState, Mode, StateFlag
from forklift_core.plant.bringup import Sample, count_settle, step_metrics, wheel_cpr, wheel_rate_cap
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost
from forklift_core.plant.real_vehicle import RearDrive, load_real_vehicle

sys.path.insert(0, str(Path(__file__).resolve().parent))
from real_mcu_link import _BAUDS, ReconnectingLink  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
STEP_MAX_S = 3.0
CLEANUP_S = 1.0
# plan 3.4 thresholds (provisional, 0.2 m/s empty truck)
OUTPUT_RESPONSE_MS = {"disarm": 100.0}
WHEEL_STOP_MS = {"tx_cut": 500.0, "expiry": 500.0}
STOP_DISTANCE_MM = {"tx_cut": 100.0, "expiry": 100.0}
HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=16)


class Refused(Exception):
    """A drive command's safety condition (design 3) refused the run before anything moved."""


@dataclass
class Bench:
    """The link, the host core and the record: every STATE is kept (a tap on the received bytes), every frame's
    write-completion time noted (the host's log), events and samples written to the JSONL file."""

    link: object  # ReconnectingLink-like: service(host, now_ns) -> events, attribute tx_enabled
    out: object  # a text file
    host: McuLinkHost = None
    samples: list = field(default_factory=list)
    written: dict = field(default_factory=dict)  # seq -> (kind, host_ns)
    last_queued: dict | None = None  # the host's last queued frame (kind, seq)
    boots: list = field(default_factory=list)

    def __post_init__(self) -> None:
        self._tap = LinkDecoder()
        self.host = McuLinkHost(HOST_CFG, log=self._log)

    def _log(self, what, now_ns, data, info) -> None:
        if what == "received":
            for r in self._tap.feed(data, now_ns):
                if isinstance(r.message, McuState):
                    self._sample(Sample(r.received_ns, r.message))
        elif what == "queued" and info:
            self.last_queued = dict(info)
        elif what == "written" and info:
            self.written[info["seq"]] = (info["kind"], now_ns)
            self.emit({"t": "written", "host_ns": now_ns, **info})

    def _sample(self, s: Sample) -> None:
        st = s.state
        if not self.boots or self.boots[-1] != st.boot_id:
            if self.boots:
                # contract 3.3: a new boot mid-session may be a motor-start brown-out even with reset_cause power
                self.emit({"t": "warning", "what": "boot_changed", "host_ns": s.host_ns, "boot_id": st.boot_id,
                           "reset_cause": st.reset_cause.name})
            self.boots.append(st.boot_id)
        self.samples.append(s)
        row = asdict(st)
        row.update(t="state", host_ns=s.host_ns, mode=st.mode.name, reject_reason=st.reject_reason.name,
                   fault_flags=[f.name for f in type(st.fault_flags) if f in st.fault_flags],
                   flags=[f.name for f in type(st.flags) if f in st.flags], reset_cause=st.reset_cause.name)
        self.emit(row)

    def emit(self, row: dict) -> None:
        self.out.write(json.dumps(row, default=str) + "\n")

    def cycle(self) -> None:
        now = time.monotonic_ns()
        for e in self.link.service(self.host, now):
            self.emit({"t": "event", "kind": e.kind, "host_ns": now, **{k: str(v) for k, v in e.detail.items()}})

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

    def __init__(self, bench: Bench, cap_rad_s: float, *, allow_locked: bool = False) -> None:
        self.b, self.cap, self.allow_locked = bench, cap_rad_s, allow_locked
        self.steer_hold = None
        self.aborted: str | None = None
        self.events_started = False  # an expected STOPPING after a scenario's event is not an abort
        self.test_end_ns: int | None = None  # the run's fixed end: no DRIVE expires after it (design 3)

    def arm(self, budget_s: float) -> None:
        """Arm to ready; ``budget_s`` fixes the run's end, past which no command stays valid."""
        b = self.b
        self.test_end_ns = time.monotonic_ns() + int(budget_s * 1e9)
        if not b.run_until(lambda: b.state is not None and b.state.mode == Mode.DISARMED, 3.0):
            raise Refused("no DISARMED STATE")
        check_build(b.state, allow_locked=self.allow_locked)
        self.steer_hold = b.state.steering_rad  # the other axes stay where they are (design 3)
        blocked = b.host.request_arm(time.monotonic_ns())
        if blocked is not None:
            raise Refused(f"arm blocked: {blocked}")
        if not b.run_until(lambda: b.host.ready, 2.0):  # arm_pending -> zero_pending -> ready, no retry
            raise Refused("not ready after ARM")

    def send(self, left: float, right: float, end_ns: int | None = None) -> int | None:
        """One DRIVE expiring at most at the run's fixed end (and ``end_ns`` when given). Returns its seq."""
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
        """Keep sending the same target for ``seconds``; returns the first queued seq."""
        end_ns = time.monotonic_ns() + int(seconds * 1e9)
        first = None
        while time.monotonic_ns() < end_ns:
            seq = self.send(left, right)
            first = first if first is not None else seq
            self.b.cycle()
            if self.b.host.ready is False and not self.events_started:
                self.aborted = "authorization_revoked_before_event"
                return first
            time.sleep(0.01)
        return first

    def cleanup(self) -> bool:
        """Design 3: stop refreshing, drop what is queued, send DISARM and confirm DISARMED within 1 s."""
        b = self.b
        b.link.tx_enabled = True
        b.host.drop_drives("cleanup")
        b.host.request_disarm(time.monotonic_ns())
        ok = b.run_until(lambda: b.state is not None and b.state.mode in (Mode.DISARMED, Mode.FAULT, Mode.ESTOP),
                         CLEANUP_S)
        b.emit({"t": "cleanup", "confirmed": bool(ok)})
        return ok


def mcu_time_of_acceptance(samples, seq: int):
    """The MCU time of the first STATE that reports ``seq`` accepted (last_rx_seq), and its received host time."""
    for s in samples:
        if s.state.last_rx_seq == seq:
            return s.state.mcu_time_us, s.host_ns
    return None, None


def run_step(bench: Bench, session: Session, wheel: str, target: float) -> dict:
    session.arm(0.5 + STEP_MAX_S + 1.0 + 1.0)
    session.hold(0.0, 0.0, 0.5)
    left, right = (target if wheel in ("left", "both") else 0.0), (target if wheel in ("right", "both") else 0.0)
    initial = bench.state.wheel_rate_left_rad_s if wheel != "right" else bench.state.wheel_rate_right_rad_s
    seq = session.hold(left, right, STEP_MAX_S)
    session.hold(0.0, 0.0, 1.0)
    session.cleanup()
    step_us, step_host = mcu_time_of_acceptance(bench.samples, seq) if seq is not None else (None, None)
    if step_us is None:
        return {"result": "no_acceptance", "aborted": session.aborted}
    m = step_metrics(bench.samples, 0 if wheel != "right" else 1, step_us, initial, target, window_s=STEP_MAX_S)
    written = bench.written.get(seq, (None, None))[1]
    return {"result": "ok" if session.aborted is None else "aborted", "aborted": session.aborted,
            "rise_s": m.rise_s, "overshoot_pct": m.overshoot_pct, "settle_s": m.settle_s, "unsettled": m.unsettled,
            "gaps": m.gaps, "input_delay_host_s": None if written is None else (step_host - written) / 1e9}


def run_scenario(bench: Bench, session: Session, kind: str, rate: float) -> dict:
    b = bench
    session.arm(0.3 + STEP_MAX_S + 1.0)
    session.hold(0.0, 0.0, 0.3)
    rolling = b.run_until(lambda: b.state.wheel_rate_left_rad_s > 0.5 * rate and b.state.wheel_rate_right_rad_s
                          > 0.5 * rate, 2.0, every=lambda: session.send(rate, rate), period_s=0.01)
    out = {"kind": kind, "rolling_confirmed": bool(rolling), "worst_case": None}
    if not rolling:
        session.cleanup()
        return {**out, "result": "aborted", "aborted": "not_rolling"}
    session.events_started = True
    event_ns, seq = None, None
    if kind == "disarm":
        b.host.request_disarm(time.monotonic_ns())
        b.run_until(lambda: any(k == "DISARM" for k, _ in b.written.values()), 0.5)
        event_ns = max((t for k, t in b.written.values() if k == "DISARM"), default=None)
    else:
        # the last DRIVE: built on a STATE just received (the largest validity the cap allows), then nothing more
        n0 = len(b.samples)
        b.run_until(lambda: len(b.samples) > n0, 0.2)
        seq = session.send(rate, rate)  # the largest validity the cap allows on the STATE just received
        if seq is not None:
            b.run_until(lambda: seq in b.written, 0.2)
            if kind == "tx_cut":
                b.link.tx_enabled = False  # at once after the write completes; STATE still read
        event_ns = b.written.get(seq, (None, None))[1] if seq is not None else None
        accepted = b.run_until(lambda: b.state.last_rx_seq == seq, 0.3) if seq is not None else False
        if kind == "tx_cut":
            out["worst_case"] = {"seq": seq, "write_done_host_ns": event_ns, "accepted": bool(accepted),
                                 "cap_ms": HOST_CFG.cap_ms, "met": bool(seq is not None and accepted)}
            if not out["worst_case"]["met"]:
                out["worst_case_unmet"] = True
    i0 = len(b.samples)
    b.run_until(lambda: b.state.mode in (Mode.DISARMED, Mode.FAULT, Mode.ESTOP), 2.0)  # STOPPING -> DISARMED expected
    b.run_until(lambda: False, 0.6)  # keep observing the counts after it
    tail = b.samples[i0:]
    mode_change = next((s for s in tail if s.state.mode != Mode.ARMED), None)
    duty_zero = next((s for s in tail if not any(s.state.duty_e4)), None)
    settle = count_settle(b.samples, i0)
    session.cleanup()
    rel = (lambda ns: None if ns is None or event_ns is None else (ns - event_ns) / 1e6)
    return {**out, "result": "ok", "event_host_ns": event_ns,
            "software": {"mode_change_ms": rel(mode_change.host_ns if mode_change else None),
                         "duty_zero_ms": rel(duty_zero.host_ns if duty_zero else None),
                         "count_settle_ms": rel(settle.host_ns), "count_settle_reason": settle.reason},
            "note": "software evidence only: the S5 verdict takes the external measurements"}


def verdict(kind: str, external: dict) -> dict:
    """Design 4: external measurements against the plan 3.4 thresholds; 'unavailable' stays so."""
    out = {}
    for name, table in (("output_response_ms", OUTPUT_RESPONSE_MS), ("stop_time_ms", WHEEL_STOP_MS),
                        ("stop_distance_mm", STOP_DISTANCE_MM)):
        value, limit = external.get(name), table.get(kind)
        if limit is None:
            continue
        if value is None or value == "unavailable":
            out[name] = {"value": "unavailable", "limit": limit, "pass": None}
        else:
            out[name] = {"value": float(value), "limit": limit, "pass": float(value) <= limit}
    return out


def _cap() -> float:
    cfg = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
    return wheel_rate_cap(cfg.wheel_rate_limit_rad_s, None)  # the radius stays unmeasured until S2


def _confirm(args, what: str) -> None:
    if args.yes:
        return
    print(what + "\nWheels lifted on the stand (or --floor prerequisites met)? Type 'yes': ", end="", flush=True)
    if sys.stdin.readline().strip() != "yes":
        raise Refused("operator did not confirm")


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
    sub.choices["record"].add_argument("--seconds", type=float, default=30.0)
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
    for name in ("output-response-ms", "stop-time-ms", "stop-distance-mm"):
        sc.add_argument("--" + name)
    args = ap.parse_args(argv)
    link = ReconnectingLink(args.device, baud=args.baud)
    with open(args.out, "a", encoding="utf-8") as out:
        bench = Bench(link, out)
        if args.cmd in ("record", "cpr"):
            bench.run_until(lambda: False, args.seconds)
            if args.cmd == "cpr":
                res = wheel_cpr(bench.samples, (args.turns_left, args.turns_right), (args.dir_left, args.dir_right),
                                motor_counts_confirmed=args.motor_counts_confirmed)
                print(json.dumps(asdict(res)))
            return 0
        session = Session(bench, _cap(), allow_locked=args.allow_locked)
        try:
            _confirm(args, f"{args.cmd}: will ARM and drive")
            if args.cmd == "step":
                result = run_step(bench, session, args.wheel, args.target)
            else:
                result = run_scenario(bench, session, args.kind, args.rate)
                result["external"] = verdict(args.kind, {
                    "output_response_ms": args.output_response_ms, "stop_time_ms": args.stop_time_ms,
                    "stop_distance_mm": args.stop_distance_mm})
        except Refused as exc:
            result = {"result": "refused", "why": str(exc)}
        finally:
            if bench.state is not None and bench.state.mode in (Mode.ARMED, Mode.STOPPING):
                session.cleanup()
        bench.emit({"t": "result", **result})
        print(json.dumps(result, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
