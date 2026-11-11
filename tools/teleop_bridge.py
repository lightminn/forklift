#!/usr/bin/env python3
"""S6 console teleop bridge (docs/design/2026-11-12-teleop-bridge.md): the operator console -> the arbiter -> the
encoder speed cap -> McuLinkHost, on a real MCU over a serial link, outside the Isaac live runtime.

The floor permission has no scan source yet (bring-up 7 adds the A2M12): it is always 0, so the wheels do not turn
-- the lease, STOP, re-arm, the MCU ARM/DISARM and the zero DRIVEs close now (design 8). Teleop opens only after the
operator confirms four things separately (design 7); ``--yes`` is not one of them.

usage: teleop_bridge.py --device D [--baud B] --empty-forks --forks-clear --lift-low --no-open-recovery
                        [--port 8765] [--host 127.0.0.1] [--web-root console/web] [--record run.jsonl]
                        [--duration-s S]
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
from forklift_core.live import real_teleop as rt  # noqa: E402
from forklift_core.live.link import Channels, Link  # noqa: E402
from forklift_core.mcu_link.build_policy import Client  # noqa: E402
from forklift_core.mcu_link.messages import McuState  # noqa: E402
from forklift_core.plant.bringup import Sample  # noqa: E402
from forklift_core.plant.kinematics import RealKinematics, teleop_limit_overrides  # noqa: E402
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost  # noqa: E402
from forklift_core.plant.port import RealPlantPort  # noqa: E402
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


class Bridge:
    """The bridge's parts on one device and one console (the CLI and the end-to-end test build it the same way)."""

    def __init__(self, device_link, *, confirmed, floor_permit=no_scan, session_epoch=None, record=None):
        self.vehicle = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
        self.params = rt.load_bridge_params(REPO / "config" / "real" / "teleop_bridge.yaml", self.vehicle)
        self.cap = rt.command_cap_mps(self.vehicle.teleop_speed_limit_mps, self.params)
        self.kin = RealKinematics(self.vehicle)
        limits = rt.real_teleop_limits(TeleopLimits(**teleop_limit_overrides(self.kin)), self.cap)
        self.epoch = session_epoch if session_epoch is not None else secrets.randbits(48)
        self.arbiter = Arbiter(self.epoch, ArbiterConfig(tick_s=CYCLE_S, teleop_limits=limits, request_tokens=True),
                               lambda: secrets.token_hex(16), mission=None)
        self.channels = Channels()
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
        self.record_errors = 0
        self.overruns = 0
        self._started_ns: int | None = None
        self._last_rx_ns: int | None = None

    def _log(self, what, now_ns, data, info) -> None:
        """Every frame written (or abandoned part-way, counted as possibly delivered): a round-trip anchor's start."""
        if what in ("written", "abandoned") and info is not None and now_ns is not None:
            self.core.frame_written(info["seq"], now_ns)

    def _on_state(self, state: McuState, received_ns: int) -> None:
        """Only the STATEs the host accepted reach the safety speed (Codex impl P2-7)."""
        self._last_rx_ns = received_ns  # kept here: the host forgets its receipt time on a disconnect
        self.core.add_sample(Sample(received_ns, state))

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
        events += self.dev.transmit(self.host, now_ns)
        if events:
            rec["link_events"] = [e.kind for e in events]
        if self.record is not None:
            try:
                self.record.write(json.dumps(rec, default=str) + "\n")
            except (OSError, ValueError):
                self.record_errors += 1
        return rec

    def notes(self, now_ns: int) -> list[str]:
        """The console's session row (state.notes): the power-cycle hint while the MCU sends nothing. The truck is
        already stopping by then -- the arbiter's feedback loss and the MCU's own command expiry -- so this only tells
        the operator what to do."""
        if self._started_ns is None:
            self._started_ns = now_ns
        since = self._started_ns if self._last_rx_ns is None else self._last_rx_ns
        return [NO_STATE_HINT] if now_ns - since >= NO_STATE_HINT_S * 1e9 else []

    def close(self, now_ns: int) -> None:
        """Leave the MCU disarmed: a DISARM written, a few cycles to let it out."""
        self.host.request_disarm(now_ns)
        for _ in range(5):
            now_ns = time.monotonic_ns()
            self.dev.receive(self.host, now_ns)
            self.dev.transmit(self.host, now_ns)
            time.sleep(CYCLE_S)


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
    ap.add_argument("--duration-s", type=float)
    args = ap.parse_args(argv)
    confirmed = {n for n in rt.CONFIRMATIONS if getattr(args, n)}
    missing = sorted(set(rt.CONFIRMATIONS) - confirmed)
    if missing:
        print(f"teleop stays closed until confirmed: {', '.join('--' + m.replace('_', '-') for m in missing)}",
              file=sys.stderr)
    record = open(args.record, "a", encoding="utf-8") if args.record else None  # noqa: SIM115
    bridge = Bridge(ReconnectingLink(args.device, baud=args.baud), confirmed=confirmed, record=record)
    server = LiveServer(bridge.channels, MediaStore(), welcome(bridge.epoch), host=args.host, port=args.port,
                        web_root=Path(args.web_root))
    server.start()
    print(json.dumps({"console": f"http://{args.host}:{server.port}/", "command_cap_mps": round(bridge.cap, 4),
                      "floor_permit": "no_scan (bring-up 7 adds the scan)"}), flush=True)
    try:
        run(bridge, duration_s=args.duration_s)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close(time.monotonic_ns())
        server.stop()
        if record is not None:
            record.close()
        print(json.dumps({"overruns": bridge.overruns, "record_errors": bridge.record_errors}), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
