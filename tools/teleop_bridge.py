#!/usr/bin/env python3
"""S6 console teleop bridge (docs/design/2026-11-12-teleop-bridge.md): the operator console -> the arbiter -> the
encoder speed cap -> McuLinkHost, on a real MCU over a serial link, outside the Isaac live runtime.

The floor permission has no scan source yet (bring-up 7 adds the A2M12): it is always 0, so the wheels do not turn
-- the lease, STOP, re-arm, the MCU ARM/DISARM and the zero DRIVEs close now (design 8). Teleop opens only after the
operator confirms four things separately (design 7); ``--yes`` is not one of them.

usage: teleop_bridge.py --device D [--baud B] --empty-forks --forks-clear --lift-low --no-open-recovery
                        [--port 8765] [--host 127.0.0.1] [--web-root console/web] [--record run.jsonl [--markers]]
                        [--duration-s S]

The record (docs/design/2026-11-14-teleop-record-state-markers-clock.md): a header line, every STATE the host took
(``{"t": "state", "host_ns", "read_ns", ...}``, the bring-up record's row), every cycle's row as before (no ``t``),
``--markers`` lines (Enter on the terminal: ``{"t": "marker", "host_ns", "index", "label"}``) and a footer with the
dropped and failed writes. A new path only: an existing file is refused.
"""

from __future__ import annotations

import argparse
import itertools
import json
import secrets
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))  # console/
sys.path.insert(0, str(Path(__file__).resolve().parent))

from console.live.server import LiveServer, MediaStore  # noqa: E402
from forklift_core.live import drive_profile as dprof  # noqa: E402
from forklift_core.live import real_teleop as rt  # noqa: E402
from forklift_core.live.link import Channels, Link  # noqa: E402
from forklift_core.mcu_link.build_policy import Client  # noqa: E402
from forklift_core.mcu_link.messages import McuState  # noqa: E402
from forklift_core.plant.bringup import Sample  # noqa: E402
from forklift_core.plant.kinematics import RealKinematics, teleop_limit_overrides  # noqa: E402
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost  # noqa: E402
from forklift_core.plant.markers import marker_reader  # noqa: E402
from forklift_core.plant.port import RealPlantPort  # noqa: E402
from forklift_core.plant.record_writer import InlineRecord, RecordWriter, host_boot_id  # noqa: E402
from forklift_core.plant.real_vehicle import load_real_vehicle  # noqa: E402
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig  # noqa: E402
from forklift_core.safety.state import state_message  # noqa: E402
from forklift_core.safety.teleop_drive import TeleopLimits  # noqa: E402
from real_mcu_link import _BAUDS, ReconnectingLink  # noqa: E402

CYCLE_S = 0.02

NO_STATE_HINT_S = 2.0  # no STATE this long: tell the operator (no automatic USB reset, lead 2026-11-12)
NO_STATE_HINT = "MCU STATE 없음 — USB 가 멈췄으면 주 전원을 재투입하세요(자동 리셋 없음)"
HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.5, zero_valid_ms=100, outbox_max=16)


def no_scan(curvatures, direction, check_speed):
    """Bring-up 6's floor permission: no scan source, nothing allowed (design 8). Bring-up 7 replaces it."""
    return 0.0, "no_scan"


def welcome(epoch: int) -> dict:
    return {"t": "welcome", "api": "console/1", "vehicle_id": "real-forklift", "session_epoch": epoch,
            "backend": "real", "capabilities": {
                "control": True, "mission": False, "teleop_direct": True, "teleop_video": False, "lift": False,
                "recovery_actions": [], "sim_reset": False,
                "feedback": {"rear_axle_speed": True, "steering": True, "lift_height": True, "load_state": False},
                "media": {}, "provisional_limits": True}}


class ProfileChannels(Channels):
    """The console's channels with a drive profile in them (G6 design 2): each ``teleop`` message the console sent
    gets the profile's two ratios on its way to the arbiter -- nothing else of it changes, and no message is made."""

    def __init__(self, runner, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.runner = runner

    def take_cycle(self):
        conns, stops, ordered, latest = super().take_cycle()
        return conns, stops, ordered, [(c, cl, self.runner.rewrite(m)) for c, cl, m in latest]


class Bridge:
    """The bridge's parts on one device and one console (the CLI and the end-to-end test build it the same way)."""

    def __init__(self, device_link, *, confirmed, floor_permit=no_scan, session_epoch=None, record=None,
                 profile=None):
        self.vehicle = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
        self.params = rt.load_bridge_params(REPO / "config" / "real" / "teleop_bridge.yaml", self.vehicle)
        self.cap = rt.command_cap_mps(self.vehicle.teleop_speed_limit_mps, self.params)
        self.kin = RealKinematics(self.vehicle)
        limits = rt.real_teleop_limits(TeleopLimits(**teleop_limit_overrides(self.kin)), self.cap)
        self.limits = limits
        self.epoch = session_epoch if session_epoch is not None else secrets.randbits(48)
        self.arbiter = Arbiter(self.epoch, ArbiterConfig(tick_s=CYCLE_S, teleop_limits=limits, request_tokens=True),
                               lambda: secrets.token_hex(16), mission=None)
        # G6: a drive profile (its file's dict) replaces the joystick's two ratios; None for the hand alone
        self.profile = None if profile is None else dprof.ProfileRunner(
            dprof.parse_profile(profile, self.profile_limits()), self.profile_limits())
        self.channels = Channels() if self.profile is None else ProfileChannels(self.profile)
        self._still_since: int | None = None
        self._last_cycle_ns: int | None = None
        self.link = Link(self.arbiter, self.channels)
        # a random first seq per session: a STATE an earlier session left in flight is never taken for this one's
        # acknowledgement (the delay bound's causality, design 2)
        self.host = McuLinkHost(HOST_CFG, client=Client.REAL, on_state=self._on_state, log=self._log,
                                initial_seq=secrets.randbelow(0x10000))
        self.port = RealPlantPort(self.host, self.kin)
        self.safety = rt.bench_safety(confirmed)
        self.core = rt.TeleopBridge(link=self.link, host=self.host, port=self.port, kinematics=self.kin,
                                    params=self.params, limit_mps=self.vehicle.teleop_speed_limit_mps,
                                    floor_permit=floor_permit, safety_provider=lambda now: self.safety,
                                    cycle_s=CYCLE_S)
        self.dev = device_link
        self.record = record
        self.marker_poll = None  # set by the CLI's --markers
        self._markers = 0
        self.overruns = 0
        self._started_ns: int | None = None
        self._last_rx_ns: int | None = None

    def profile_limits(self) -> dprof.ProfileLimits:
        """What a profile is checked against: the teleop limits this bridge drives under (design 2.1)."""
        lim, v = self.limits, self.vehicle
        return dprof.ProfileLimits(cap_mps=min(lim.direct_forward_mps, lim.direct_reverse_mps),
                                   kappa_max=lim.curvature_inv_m, kappa_rate=lim.curvature_rate_inv_m_s,
                                   q_rate=self.kin.steering_rate_rad_s, accel_mps2=lim.accel_mps2,
                                   q_of_kappa=v.steering.angle, kappa_of_q=lambda q: self.kin.held_curvature((q,)))

    def profile_start(self, now_ns: int) -> str | None:
        """The operator's start (Enter on the bridge's terminal): None when a trial began, else why not."""
        if self.profile is None:
            return "no_profile"
        a = self.arbiter
        still = 0.0 if self._still_since is None else (now_ns - self._still_since) / 1e9
        why = self.profile.start(now_ns, standstill_s=still, armed=a.armed and a.mode == "TELEOP")
        if self.record is not None:
            self.record.put_row({"t": "profile_start", "host_ns": now_ns, "trial": self.profile.trial,
                                 "profile": self.profile.profile.name, "refused": why})
        return why

    def _profile_step(self, now_ns: int, rec: dict, out) -> None:
        reading = self.core.reading
        valid = reading is not None and reading.speed.valid
        speed = reading.speed.signed_mps if valid else 0.0
        if valid and abs(speed) < self.limits.stop_speed_mps:
            self._still_since = now_ns if self._still_since is None else self._still_since
        else:
            self._still_since = None
        dt = CYCLE_S if self._last_cycle_ns is None else (now_ns - self._last_cycle_ns) / 1e9
        self._last_cycle_ns = now_ns
        obs = dprof.Observation(
            dt_s=dt, speed_valid=valid, speed_mps=speed,
            meas_q=None if reading is None or reading.steering_rad is None else reading.steering_rad,
            armed=bool(rec.get("armed")), mode=str(rec.get("mode")), target_mps=float(rec.get("target_mps") or 0.0),
            applied_mps=float(rec.get("applied_mps") or 0.0), applied_q=self.core.q_cmd, permit=out.permit,
            port=rec.get("port"), direction_cut=rec.get("direction_cut"), stop=rec.get("stop") or rec.get("disarm"),
            # only the command the arbiter accepted (owner, lease, generation, age) carries the operator's deadman:
            # an observer's or a refused message never releases anything (Codex G6 code review P1-1)
            deadman=bool(self.arbiter.teleop and self.arbiter.teleop.get("deadman")), policy=rec.get("policy"))
        row = self.profile.step(now_ns, obs)
        if self.profile.zero_now:
            # the request dropped to zero (an end, or a drive finished): the accepted command's speed goes to zero
            # now, not when the console's next message comes (Codex G6 code review P1-2). Only ever a reduction.
            self.profile.zero_now = False
            if self.arbiter.teleop is not None:
                self.arbiter.teleop["speed_ratio"] = 0.0
        if row is not None and self.record is not None:
            self.record.put_row(row)
        for m in self.profile.take_marks():
            self.mark(m["label"], m["host_ns"])

    def _log(self, what, now_ns, data, info) -> None:
        """Every frame written (or abandoned part-way, counted as possibly delivered): a round-trip anchor's start."""
        if what in ("written", "abandoned") and info is not None and now_ns is not None:
            self.core.frame_written(info["seq"], now_ns)

    def _on_state(self, state: McuState, received_ns: int) -> None:
        """Only the STATEs the host accepted reach the safety speed (Codex impl P2-7)."""
        self._last_rx_ns = received_ns  # kept here: the host forgets its receipt time on a disconnect
        sample = Sample(received_ns, state)
        if self.record is not None:
            # every STATE the host took, before the core filters it (record design 3): queued, written by the writer
            self.record.put_state(sample, getattr(self.dev, "last_read_ns", None))
        self.core.add_sample(sample)

    def mark(self, label: str, host_ns: int) -> None:
        """An operator marker (record design 4): queued like any row."""
        if self.record is not None:
            self.record.put_row({"t": "marker", "host_ns": host_ns, "index": self._markers, "label": label})
        self._markers += 1

    @property
    def record(self):
        return self._record

    @record.setter
    def record(self, value) -> None:
        # a plain file (a test's StringIO) is written at once; the CLI passes a RecordWriter (bounded queue + thread)
        self._record = InlineRecord(value) if value is not None and not hasattr(value, "put_row") else value

    @property
    def record_errors(self) -> int:
        return 0 if self.record is None else self.record.write_errors + self.record.dropped

    def cycle(self, now_ns: int) -> dict:
        """Design 5: receive only, decide, transmit only."""
        events = self.dev.receive(self.host, now_ns)
        rec = self.core.cycle(now_ns)
        out = rec.pop("_out")
        now = now_ns / 1e9
        if self.link.state_due(now):
            reading = self.core.reading
            st = self.host.state
            q = None if reading is None or reading.steering_rad is None else reading.steering_rad
            # the final state and the command actually submitted (Codex impl P3): ``out`` was refreshed after every
            # stop the cycle took; ``applied`` is the port's speed and the steering command's own curvature
            msg = state_message(self.arbiter, out, self.core.safety_provider(now), now_mono=now,
                                rear_axle_speed_mps=None if reading is None or not reading.speed.valid
                                else reading.speed.signed_mps,
                                applied=rec["applied"], steering_rad=None if q is None else [q],
                                lift_m=None if st is None else st.lift_m)
            # the wall escape (standing-direction-rule design 4.1/4.3): the offer the console's button answers, and
            # the escape running
            msg["escape_offer"] = self.core.offer_view()
            msg["escape"] = self.core.escape_view()
            notes = self.notes(now_ns)
            if notes:
                msg["notes"] = notes
            self.channels.put(None, msg)
        for e in rec.get("escape_events", ()):
            # the console logs vehicle events (standing-direction-rule design 4.3; Codex impl review P2-2)
            kind = "escape_stop" if e["event"] == "end" and e.get("stop") else f"escape_{e['event']}"
            self.channels.put(None, {"t": "event", "kind": kind, "reason": e.get("reason"),
                                     **{k: e[k] for k in ("offer_id", "travel_m", "elapsed_s", "after_end_s")
                                        if k in e}})
        events += self.dev.transmit(self.host, now_ns)
        if events:
            rec["link_events"] = [e.kind for e in events]
        if self.record is not None:
            self.record.put_row(rec)
        if self.profile is not None:
            self._profile_step(now_ns, rec, out)
        if self.marker_poll is not None:
            self.marker_poll()
        return rec

    def notes(self, now_ns: int) -> list[str]:
        """The console's session row (state.notes): the power-cycle hint while the MCU sends nothing. The truck is
        already stopping by then -- the arbiter's feedback loss and the MCU's own command expiry -- so this only tells
        the operator what to do."""
        if self._started_ns is None:
            self._started_ns = now_ns
        since = self._started_ns if self._last_rx_ns is None else self._last_rx_ns
        notes = [NO_STATE_HINT] if now_ns - since >= NO_STATE_HINT_S * 1e9 else []
        if self.profile is not None:
            p = self.profile
            seg = p.segment
            notes.append(f"일정표 {p.profile.name}: " + (
                f"시행 {p.trial} {seg.label} ({seg.kind})" if seg is not None else
                f"시행 {p.trial} 끝 {p.result} — deadman 을 놓으면 손 조작으로 돌아갑니다" if p.state == "zeroing"
                else "대기 — 정지·무장 뒤 다리 터미널에서 Enter"))
        return notes

    def close(self, now_ns: int) -> None:
        """Leave the MCU disarmed: a DISARM written, a few cycles to let it out."""
        self.host.request_disarm(now_ns)
        for _ in range(5):
            now_ns = time.monotonic_ns()
            self.dev.receive(self.host, now_ns)
            self.dev.transmit(self.host, now_ns)
            time.sleep(CYCLE_S)


RECORD_SCHEMA = "teleop_bridge_record/1"


OVERRUN_TOLERANCE_NS = 2_000_000  # a cycle starting this much past its slot counts as an overrun


def run(bridge: Bridge, *, duration_s: float | None, clock=time.monotonic_ns, sleep=time.sleep) -> list[int]:
    """The 50 Hz wall-clock loop. Every cycle starts at least a period after the previous one started; a late start
    is counted and the schedule restarts from it -- no catching up (Codex impl P2-6: two cycles 5 ms apart each took
    a 20 ms slew). Returns the start times (tests)."""
    period = int(CYCLE_S * 1e9)
    t0 = clock()
    slot = t0
    starts: list[int] = []
    for _ in itertools.count():
        now = clock()
        if duration_s is not None and now - t0 >= duration_s * 1e9:
            break
        if now - slot > OVERRUN_TOLERANCE_NS:
            bridge.overruns += 1
        starts.append(now)
        bridge.cycle(now)
        slot = now + period  # from this cycle's actual start
        delay = (slot - clock()) / 1e9
        if delay > 0:
            sleep(delay)
    return starts


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--device", required=True)
    ap.add_argument("--baud", type=int, choices=sorted(_BAUDS))
    for name in rt.CONFIRMATIONS:
        ap.add_argument("--" + name.replace("_", "-"), action="store_true")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--web-root", default=str(REPO / "console" / "web"))
    ap.add_argument("--record")
    ap.add_argument("--markers", action="store_true")
    ap.add_argument("--profile", help="drive profile YAML (G6): Enter on this terminal starts a trial; needs --record")
    ap.add_argument("--duration-s", type=float)
    args = ap.parse_args(argv)
    confirmed = {n for n in rt.CONFIRMATIONS if getattr(args, n)}
    missing = sorted(set(rt.CONFIRMATIONS) - confirmed)
    if missing:
        print(f"teleop stays closed until confirmed: {', '.join('--' + m.replace('_', '-') for m in missing)}",
              file=sys.stderr)
    if args.markers and not args.record:
        print("--markers needs --record", file=sys.stderr)
        return 2
    profile = None
    if args.profile:
        if not args.record or args.markers:
            print("--profile needs --record and takes the terminal's Enter (not with --markers)", file=sys.stderr)
            return 2
        import yaml

        profile = yaml.safe_load(Path(args.profile).read_text(encoding="utf-8"))
    record = None
    if args.record:
        try:
            sink = open(args.record, "x", encoding="utf-8")  # noqa: SIM115 -- a new record per run (design 3)
        except FileExistsError:
            print(f"the record {args.record} exists: give a new path", file=sys.stderr)
            return 2
        record = RecordWriter(sink)
        record.put_row({"t": "header", "schema": RECORD_SCHEMA, "tool": "teleop_bridge.py", "device": args.device,
                        "host_boot_id": host_boot_id(), "start_wall": time.time(),
                        "start_mono_ns": time.monotonic_ns()})
    # from the record's creation on, every exit path closes it (Codex code review P2-3: a failed start left the writer
    # and the sink open)
    bridge = server = None
    started = False
    out: dict = {}
    try:
        bridge = Bridge(ReconnectingLink(args.device, baud=args.baud), confirmed=confirmed, record=record,
                        profile=profile)
        if bridge.profile is not None:
            def on_enter(_label, host_ns):
                why = bridge.profile_start(host_ns)
                print(json.dumps({"profile_start": bridge.profile.trial if why is None else None, "refused": why}),
                      flush=True)
            reader = marker_reader(on_enter)
            bridge.marker_poll = reader.poll if reader.active else None
            print(json.dumps({"profile": bridge.profile.profile.name, "planned_s": round(bridge.profile.profile.planned_s, 1),
                              "start": "Enter" if reader.active else f"off: {reader.reason}"}), flush=True)
        if args.markers:
            reader = marker_reader(bridge.mark)
            bridge.marker_poll = reader.poll if reader.active else None
            print(json.dumps({"markers": "on" if reader.active else f"off: {reader.reason}"}), flush=True)
        server = LiveServer(bridge.channels, MediaStore(), welcome(bridge.epoch), host=args.host, port=args.port,
                            web_root=Path(args.web_root))
        server.start()
        started = True
        print(json.dumps({"console": f"http://{args.host}:{server.port}/", "command_cap_mps": round(bridge.cap, 4),
                          "floor_permit": "no_scan (bring-up 7 adds the scan)"}), flush=True)
        try:
            run(bridge, duration_s=args.duration_s)
        except KeyboardInterrupt:
            pass
    finally:
        if bridge is not None:
            bridge.close(time.monotonic_ns())
            out["overruns"] = bridge.overruns
        if started:
            server.stop()
        if record is not None:
            out["record"] = record.close({"overruns": out.get("overruns"),
                                          "markers": 0 if bridge is None else bridge._markers,
                                          "end_mono_ns": time.monotonic_ns()})
        print(json.dumps(out), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
