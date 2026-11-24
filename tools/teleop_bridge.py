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
import gc
import hashlib
import itertools
import json
import os
import secrets
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))  # console/
sys.path.insert(0, str(Path(__file__).resolve().parent))

from console.live.server import LiveServer, MediaStore  # noqa: E402
from forklift_core.live import drive_profile as dprof  # noqa: E402
from forklift_core.live import model_peer  # noqa: E402
from forklift_core.live import real_teleop as rt  # noqa: E402
from forklift_core.live.executor_lock import DeviceBusy  # noqa: E402
from forklift_core.live.synthetic import SyntheticRefused, bench_opener, device_identity, writer_marker  # noqa: E402
from forklift_core.live.link import Channels, Link  # noqa: E402
from forklift_core.mcu_link.build_policy import Client  # noqa: E402
from forklift_core.mcu_link.messages import McuState  # noqa: E402
from forklift_core.plant.bringup import Sample  # noqa: E402
from forklift_core.plant.kinematics import RealKinematics, teleop_limit_overrides  # noqa: E402
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost  # noqa: E402
from forklift_core.plant.markers import marker_input_state, marker_reader  # noqa: E402
from forklift_core.plant.port import RealPlantPort  # noqa: E402
from forklift_core.plant.record_writer import InlineRecord, RecordWriter, host_boot_id  # noqa: E402
from forklift_core.plant.real_vehicle import chassis_model_path, load_real_vehicle  # noqa: E402
from forklift_core.safety.real_watch import SafetyWatch, load_watch_config  # noqa: E402
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig  # noqa: E402
from forklift_core.safety.state import state_message  # noqa: E402
from forklift_core.safety.teleop_drive import TeleopLimits  # noqa: E402
from real_mcu_link import _BAUDS, ReconnectingLink, open_raw  # noqa: E402

CYCLE_S = 0.02

NO_STATE_HINT_S = 2.0  # no STATE this long: tell the operator (no automatic USB reset, lead 2026-11-12)
NO_STATE_HINT = "MCU STATE 없음 — USB 가 멈췄으면 주 전원을 재투입하세요(자동 리셋 없음)"
# D5 (docs/design/2026-11-15-d5-link-gap-mcu-time.md): freshness by MCU-time gap, 150 ms host cap = the DRIVE cap
HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.5, zero_valid_ms=100, outbox_max=16,
                      gap_clock="mcu", host_gap_max_s=0.15)


class LateFreshness:
    """The host is made before the bridge core whose delay bound and rate guard it reads (D5 3.3a): the core is
    bound once it exists."""

    def __init__(self) -> None:
        self.target = None

    def delay_s(self, state, rx_ns):
        return None if self.target is None else self.target.delay_s(state, rx_ns)

    def delay_terms(self, state, rx_ns):
        return None if self.target is None else self.target.delay_terms(state, rx_ns)

    def rate_verified(self):
        return self.target is not None and self.target.rate_verified()

    def rate_bound(self):
        return rt.RATE_RHO_MAX if self.target is None else self.target.rate_bound()

    def on_disconnect(self):
        if self.target is not None:
            self.target.on_disconnect()


def no_scan(curvatures, direction, check_speed):
    """Bring-up 6's floor permission: no scan source, nothing allowed (design 8). Bring-up 7 replaces it."""
    return 0.0, "no_scan"


class SyntheticPermit:
    """The rehearsal's model-only floor permission (docs/design/2026-11-15-arrival-day-rehearsal.md 2.3): open while
    the model-peer check passes on the device -- the full check (with its scan of other processes) once at start, in
    ``main``; here, every call, the path's device, and every ``recheck_s`` the cheap ``model_alive_refusal`` (the
    full scan in the 20 ms loop stalled it past the STATE age gate) -- and closed for good once one fails. Never on a real device: a USB CDC or
    UART link fails the check's first condition before this is ever built."""

    def __init__(self, device: str, bench: dict, *, recheck_s: float = 1.0, clock=time.monotonic_ns,
                 check=model_peer.model_alive_refusal) -> None:
        self.device, self.bench, self.recheck_ns, self.clock, self.check = device, bench, int(recheck_s * 1e9), clock, check
        self.lost: str | None = None
        self.next_ns = 0

    def __call__(self, curvatures, direction, check_speed):
        if self.lost is None:
            # every call: the path still names the bench's pty -- a reopen onto another device (a link the
            # ReconnectingLink just re-established) closes the permit in the same cycle (Codex impl review P1)
            try:
                if os.stat(self.device).st_rdev != self.bench.get("pts_rdev"):
                    self.lost = "device_changed"
            except OSError:
                self.lost = "no_device"
        if self.lost is None:
            now = self.clock()
            if now >= self.next_ns:
                why = self.check(self.device, self.bench)
                if why is not None:
                    self.lost = why
                self.next_ns = now + self.recheck_ns
        if self.lost is not None:
            return 0.0, "synthetic_permit_lost"
        return 1.0, "synthetic_permit"


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
                 profile=None, rate_rho=rt.RATE_RHO_MAX, clock=None, scan_permit=None, safety_watch=True):
        """``rate_rho``: the MCU clock rate bound the arm waits for (D5 3.11; tests widen it to verify quickly).
        ``clock``: the CLI's monotonic clock -- the decision time is taken after the receive (the D5 read time is
        later than the cycle start) and each cycle's stage times and GC pauses are recorded; None keeps the cycle
        start for everything (tests drive the time). ``safety_watch``: the real truck's safety watch
        (docs/design/2026-11-16-real-safety-watch.md, config/real/safety_watch.yaml) -- on unless turned off."""
        self.clock = clock
        # 7b (docs/design/2026-11-16-bridge-scan-permit.md): the scan permit is the floor permission, fed each STATE
        # and polled each cycle
        self.scan_permit = scan_permit
        if scan_permit is not None:
            floor_permit = scan_permit
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
        self._fresh = LateFreshness()
        self.host = McuLinkHost(HOST_CFG, client=Client.REAL, on_state=self._on_state, log=self._log,
                                initial_seq=secrets.randbelow(0x10000), freshness=self._fresh)
        self.port = RealPlantPort(self.host, self.kin)
        self.safety = rt.bench_safety(confirmed)
        self.core = rt.TeleopBridge(link=self.link, host=self.host, port=self.port, kinematics=self.kin,
                                    params=self.params, limit_mps=self.vehicle.teleop_speed_limit_mps,
                                    floor_permit=floor_permit, safety_provider=lambda now: self.safety,
                                    cycle_s=CYCLE_S, rate_rho=rate_rho,
                                    safety_watch=SafetyWatch(load_watch_config(SAFETY_WATCH_YAML),
                                                             wheel_radius_m=self.vehicle.wheel_radius_m)
                                    if safety_watch else None)
        self._fresh.target = self.core
        self._gc_ms = 0.0  # GC pause time since the last cycle record (D5 (나) instrumentation)
        self._gc_t0: int | None = None
        if self.clock is not None:
            gc.callbacks.append(self._gc_cb)
        self.dev = device_link
        self.record = record
        self.marker_poll = None  # set by the CLI's --markers
        self._markers = 0
        self._profile_ended = None  # the last trial whose profile_end row was written
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
            port=rec.get("port"), direction_cut=rec.get("direction_cut"), stop=rec.get("stop") or rec.get("disarm") or self.arbiter.preempt_latch,
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
            if row.get("result") is not None and self._profile_ended != row["trial"]:
                # one row per trial at its end (G6 design 2; arrival-day rehearsal defect D1)
                self._profile_ended = row["trial"]
                self.record.put_row({"t": "profile_end", "host_ns": row["host_ns"], "trial": row["trial"],
                                     "result": row["result"]})
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
        if self.scan_permit is not None:
            self.scan_permit.on_state(sample, self.core.delay_s(state, received_ns))
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

    def _gc_cb(self, phase, info) -> None:
        t = self.clock()
        if phase == "start":
            self._gc_t0 = t
        elif self._gc_t0 is not None:
            self._gc_ms += (t - self._gc_t0) / 1e6
            self._gc_t0 = None

    def cycle(self, now_ns: int) -> dict:
        """Design 5: receive only, decide, transmit only. With a clock (the CLI) the decision time is taken after the
        receive (D5 3.3b) and the stage times go in the record."""
        t0 = now_ns
        events = self.dev.receive(self.host, now_ns)
        t_rx = now_ns
        if self.clock is not None:
            t_rx = self.clock()
            now_ns = max(now_ns, t_rx)
        if self.scan_permit is not None:
            self.scan_permit.poll(now_ns)  # the scans received since the last cycle, before the decision
        rec = self.core.cycle(now_ns)
        if self.scan_permit is not None:
            rec["scan"] = self.scan_permit.record_view()
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
        trip = (rec.get("watch") or {}).get("trip")
        if trip is not None:
            # the safety watch's stop: one kind (the console paints a kind with "stop" red), the reason the check
            self.channels.put(None, {"t": "event", "kind": "watch_stop", "reason": trip})
        for why in ("mcu_stopping", "loop_slow", "clock_rate"):
            # D5 decisions 2-3: the grey zone's MCU stop, a slow loop and a clock out of range are shown, not hidden
            if (why == "mcu_stopping" and rec.get("mcu_stopping")) or rec.get("stop") == why:
                self.channels.put(None, {"t": "event", "kind": why, "reason": why})
        if self.clock is not None:
            t_dec = self.clock()
        events += self.dev.transmit(self.host, now_ns)
        for e in events:
            if e.kind == "device_busy":
                # one executor per device (real safety wiring design 8): shown with its holder, the opening retried
                h = e.detail.get("holder") or {}
                why = f"{e.detail.get('path')} is held by pid {h.get('pid', '?')} ({h.get('tool', '?')})"
                self.channels.put(None, {"t": "event", "kind": "device_busy", "reason": why})
                print(f"teleop_bridge: {why}", file=sys.stderr, flush=True)
            if e.kind == "estop_waived":
                # V1.0 e-stop waiver (plan 3): shown, not hidden -- the console paints a kind with "stop" red
                why = f"no e-stop input; waived by the board image ({e.detail.get('product')})"
                self.channels.put(None, {"t": "event", "kind": "estop_waived", "reason": why})
                print(f"WARNING {why} -- the main power switch is the only stop outside the MCU", file=sys.stderr,
                      flush=True)
        if events:
            rec["link_events"] = [e.kind for e in events]
            stale = [e.detail for e in events if e.kind == "stale"]
            if stale:
                rec["stale_detail"] = stale
        if self.clock is not None:
            t_tx = self.clock()
            rec["cycle_start_ns"] = t0
            rec["stage_ms"] = {"rx": round((t_rx - t0) / 1e6, 3), "dec": round((t_dec - now_ns) / 1e6, 3),
                               "tx": round((t_tx - t_dec) / 1e6, 3)}
            rec["gc_ms"], self._gc_ms = round(self._gc_ms, 3), 0.0
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
        rate = self.core.rate
        if rate.verdict == rt.RATE_OUT:
            notes.append("MCU 시계 속도가 범위 밖입니다 — 이 부팅에서는 무장할 수 없습니다(전원을 다시 켜고 HSE 를 확인)")
        elif rate.verdict != rt.RATE_VERIFIED and self.host.state is not None:
            prog = rate.progress_s()
            notes.append("MCU 시계 확인 중 — 확인 뒤 무장할 수 있습니다" if prog is None else
                         f"MCU 시계 확인 중 {prog[0]:.0f}/{max(prog[1], prog[0]):.0f} s — 확인 뒤 무장할 수 있습니다")
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
        if self._gc_cb in gc.callbacks:
            gc.callbacks.remove(self._gc_cb)
        if getattr(self.dev, "ever_connected", True) is False:
            return  # never opened (or refused at its first opening): nothing to disarm, and no opening now (8.1)
        self.host.request_disarm(now_ns)
        for _ in range(5):
            now_ns = time.monotonic_ns()
            self.dev.receive(self.host, now_ns)
            self.dev.transmit(self.host, now_ns)
            time.sleep(CYCLE_S)


RECORD_SCHEMA = "teleop_bridge_record/1"
VEHICLE_YAML = REPO / "config" / "real" / "vehicle.yaml"
BRIDGE_PARAMS_YAML = REPO / "config" / "real" / "teleop_bridge.yaml"
SAFETY_WATCH_YAML = REPO / "config" / "real" / "safety_watch.yaml"


def _file_identity(path: Path) -> dict:
    data = Path(path).read_bytes()
    try:
        shown = str(Path(path).resolve().relative_to(REPO))
    except ValueError:
        shown = str(path)
    return {"path": shown, "sha256": hashlib.sha256(data).hexdigest()}


def config_identity(profile_path: str | None, profile_bytes: bytes | None = None) -> dict:
    """What the record was made with (D4 design 2, lead 2026-11-15): the vehicle file with its calibration id, the
    bridge's encoder-speed premises and the drive profile, each by content hash -- an analysis given another file
    is refused instead of silently using different numbers."""
    vehicle = {**_file_identity(VEHICLE_YAML), "calibration_id": load_real_vehicle(VEHICLE_YAML).calibration_id,
               "chassis_model": _file_identity(chassis_model_path(VEHICLE_YAML))}  # the file vehicle.yaml names
    out = {"vehicle": vehicle, "bridge_params": _file_identity(BRIDGE_PARAMS_YAML), "profile": None}
    if profile_path is not None:
        import yaml

        # the same bytes give the hash and the parsed name -- the CLI passes the bytes it parsed for the run itself
        # (Codex #530 P2: a file read twice could change between the two reads)
        data = Path(profile_path).read_bytes() if profile_bytes is None else profile_bytes
        ident = _file_identity(Path(profile_path))
        ident["sha256"] = hashlib.sha256(data).hexdigest()
        out["profile"] = {**ident, "name": (yaml.safe_load(data) or {}).get("name")}
    return out


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
        bridge.looped = True  # a control cycle ran: the exit leaves a DISARM (main's finally)
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
    ap.add_argument("--no-safety-watch", action="store_true",
                    help="turn the real truck's safety watch off (config/real/safety_watch.yaml; on by default)")
    ap.add_argument("--profile", help="drive profile YAML (G6): Enter on this terminal starts a trial; needs --record")
    ap.add_argument("--duration-s", type=float)
    ap.add_argument("--bench", help="rehearsal only: the bench's bench.json; required on a pty device (the model "
                                   "MCU), refused on any other; the record is marked synthetic")
    ap.add_argument("--synthetic-permit", action="store_true",
                    help="rehearsal only: open the floor permission on the model MCU (needs --bench and a passing "
                         "model-peer check; refused on a real device). No operating launcher passes it.")
    ap.add_argument("--scan-socket", help="7b: the scan adapter's Unix socket (bind here); with --lidar-mount the "
                                         "floor permission comes from the real scan")
    ap.add_argument("--lidar-mount", help="7b: x,y,z,yaw_deg of the LiDAR in the rear-axle frame (measured, S7)")
    ap.add_argument("--lidar-mask", help="7b: a:b,c:d degree intervals (sensor frame) the truck's body hides")
    args = ap.parse_args(argv)
    if (args.scan_socket is None) != (args.lidar_mount is None):
        print("--scan-socket and --lidar-mount go together (the mount has no default: S7 measures it)",
              file=sys.stderr)
        return 2
    if args.scan_socket is not None and args.synthetic_permit:
        print("--scan-socket is not combined with --synthetic-permit", file=sys.stderr)
        return 2
    scan_args = None
    if args.scan_socket is not None:
        import bridge_scan

        try:
            scan_args = (bridge_scan.parse_mount(args.lidar_mount), bridge_scan.parse_mask(args.lidar_mask))
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 2
    try:
        synthetic = writer_marker(args.device, args.bench)
    except (SyntheticRefused, OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.synthetic_permit and synthetic is None:
        print("--synthetic-permit is for the rehearsal's model MCU only: give a pty device with --bench", file=sys.stderr)
        return 2
    if synthetic is not None:
        synthetic["permit"] = bool(args.synthetic_permit)
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

        try:
            profile_bytes = Path(args.profile).read_bytes()  # read once: the run and the header use these bytes
            profile = yaml.safe_load(profile_bytes)
        except (OSError, yaml.YAMLError) as e:
            print(f"the profile {args.profile} cannot be read: {e}", file=sys.stderr)
            return 2
    record = None
    identity = None
    if args.record:
        # computed before the record exists: a file that cannot be read stops here, leaving no half-made record
        try:
            identity = config_identity(args.profile, profile_bytes if args.profile else None)
        except (OSError, ValueError) as e:
            print(f"the configuration cannot be identified: {e}", file=sys.stderr)
            return 2
    if args.record:
        try:
            sink = open(args.record, "x", encoding="utf-8")  # noqa: SIM115 -- a new record per run (design 3)
        except FileExistsError:
            print(f"the record {args.record} exists: give a new path", file=sys.stderr)
            return 2
        record = RecordWriter(sink)
        marks = None
        if args.markers or args.profile:
            # Enter marks events (--markers) or starts the trial (--profile): whether it can, in the record (R2-1)
            active, why = marker_input_state()
            marks = {"requested": "profile_start" if args.profile else "markers", "active": active, "why": why}
        record.put_row({"t": "header", "schema": RECORD_SCHEMA, "tool": "teleop_bridge.py", "device": args.device,
                        "device_id": device_identity(args.device), "synthetic": synthetic, "host_boot_id": host_boot_id(), "start_wall": time.time(),
                        "start_mono_ns": time.monotonic_ns(), "config": identity,
                        "safety_watch": None if args.no_safety_watch else _file_identity(SAFETY_WATCH_YAML),
                        **({} if marks is None else {"markers": marks})})
    # from the record's creation on, every exit path closes it (Codex code review P2-3: a failed start left the writer
    # and the sink open)
    bridge = server = scan_sock = None
    started = False
    out: dict = {}
    code = 0
    try:
        floor = SyntheticPermit(args.device, synthetic["bench"]) if args.synthetic_permit else no_scan
        scan_permit = None
        if scan_args is not None:
            import bridge_scan

            from forklift_core.sensors.scan_feed import ScanFeed, bind_socket

            scan_sock = bind_socket(args.scan_socket)
            scan_permit = bridge_scan.ScanPermit(
                ScanFeed(sock=scan_sock), mount=scan_args[0], mask_deg=scan_args[1],
                config_path=REPO / "config" / "real" / "obstacle_layer_real.yaml",
                vehicle=load_real_vehicle(VEHICLE_YAML), v_max_mps=load_real_vehicle(VEHICLE_YAML).teleop_speed_limit_mps,
                allow_synthetic=synthetic is not None)
        link = (ReconnectingLink(args.device, baud=args.baud, clock=time.monotonic_ns, exclusive=True)
                if synthetic is None
                else ReconnectingLink(args.device, baud=args.baud, opener=bench_opener(open_raw, synthetic["bench"]),
                                      clock=time.monotonic_ns, exclusive=True))
        bridge = Bridge(link, confirmed=confirmed, record=record, clock=time.monotonic_ns,
                        safety_watch=not args.no_safety_watch,
                        profile=profile, floor_permit=floor, scan_permit=scan_permit)
        if identity is not None:
            # the configuration the bridge loaded is the one the header names (Codex #530 P2: the files are read by
            # the header and by the bridge at two moments)
            changed = [what for what, then, now in (
                ("vehicle", identity["vehicle"]["sha256"], _file_identity(VEHICLE_YAML)["sha256"]),
                ("chassis_model", identity["vehicle"]["chassis_model"]["sha256"],
                 _file_identity(chassis_model_path(VEHICLE_YAML))["sha256"]),
                ("bridge_params", identity["bridge_params"]["sha256"], _file_identity(BRIDGE_PARAMS_YAML)["sha256"]))
                if then != now]
            if bridge.vehicle.calibration_id != identity["vehicle"]["calibration_id"]:
                changed.append("calibration_id")
            if changed:
                print(f"the configuration changed while starting ({', '.join(changed)}): start again", file=sys.stderr)
                record.put_row({"t": "refused", "why": "config_changed", "what": changed})
                return 2
        try:
            # the first opening before anything else starts: a device held by another executor is refused first,
            # whatever else fails later (design 8.1, Codex executor-lock impl round 2 P2)
            link.open_first(bridge.host, time.monotonic_ns())
        except DeviceBusy as busy:
            print(f"teleop_bridge: {busy}", file=sys.stderr, flush=True)
            if record is not None:
                record.put_row({"t": "refused", "why": "device_busy", "holder": busy.holder})
            code = 2
            return code
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
                          "floor_permit": ("synthetic_permit (rehearsal, model MCU only)" if args.synthetic_permit
                                           else "scan (7b, provisional: the memo's operating conditions)"
                                           if scan_permit is not None else "no_scan (bring-up 7 adds the scan)")}),
              flush=True)
        try:
            run(bridge, duration_s=args.duration_s)
        except KeyboardInterrupt:
            pass
        except DeviceBusy as busy:
            # one executor per device (real safety wiring design 8.1): held at the first opening, the link ends
            # refused -- nothing was written, and the close below cannot reopen it
            print(f"teleop_bridge: {busy}", file=sys.stderr, flush=True)
            if record is not None:
                record.put_row({"t": "refused", "why": "device_busy", "holder": busy.holder})
            code = 2
    finally:
        if scan_sock is not None:
            scan_sock.close()
        if bridge is not None and getattr(bridge, "looped", False):
            bridge.close(time.monotonic_ns())
            out["overruns"] = bridge.overruns
        elif bridge is not None:
            # no control cycle ran (the console failed to start, or --duration-s 0): nothing of ours to stop, so no
            # DISARM -- it would cut short a stop the previous run left -- and the fd is closed, releasing the lock
            # (Codex executor-lock impl rounds 3-4)
            if bridge._gc_cb in gc.callbacks:
                gc.callbacks.remove(bridge._gc_cb)
            if getattr(bridge.dev, "link", None) is not None:
                bridge.dev.link.close()
                bridge.dev.link = None
        if started:
            server.stop()
        if record is not None:
            out["record"] = record.close({"overruns": out.get("overruns"),
                                          "markers": 0 if bridge is None else bridge._markers,
                                          "end_mono_ns": time.monotonic_ns()})
        print(json.dumps(out), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
