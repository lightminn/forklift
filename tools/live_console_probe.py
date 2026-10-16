"""Scripted console/1 client for the live Isaac runner's functional test (M-c plan, completion condition 2).

Reads the runner's port file, connects the control and media sockets as the web console would and walks a scenario,
writing what it sent, what it got and when (its own monotonic clock) to a JSON report. The verdict is read from this
report together with the runner's live_log.jsonl -- this tool does not decide pass or fail on its own.

  python tools/live_console_probe.py --port-file RUN/live_port.json --report RUN/probe.json --scenario main
  python tools/live_console_probe.py ... --scenario slow --old-session RUN1/probe.json
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import struct
import time
from pathlib import Path

from websockets.asyncio.client import connect

API = "console/1"


class Probe:
    def __init__(self, port: int, report: dict):
        self.port, self.report = port, report
        self.ws = None
        self.media = None
        self.client_id = "probe"
        self.epoch = None
        self.lease_id = None
        self.control_gen = 0
        self.arm_gen = 0
        self.tick = None
        self.state = None
        self.targets = None
        self.replies: dict = {}
        self.seq = 0
        self.n = 0
        self.frames: list = []
        self.states: list = []
        self._reader = None
        self._media_reader = None
        self.closed_code = None
        self.last_teleop = None
        self.save_frames = None  # (directory, how many) for the marker scenario

    def event(self, name, **fields):
        self.report.setdefault("events", []).append({"mono": time.monotonic(), "event": name, **fields})

    # --- connection ---------------------------------------------------------------------------------------------
    async def open(self):
        if self._reader is not None:
            self._reader.cancel()  # the previous connection's reader must not write this one's state
        self.closed_code = None
        self.ws = await connect(f"ws://127.0.0.1:{self.port}/api/v1/control", max_size=None)
        await self.ws.send(json.dumps({"t": "hello", "api": API, "client_id": self.client_id}))
        welcome = json.loads(await self.ws.recv())
        self.epoch = welcome["session_epoch"]
        self.event("welcome", session_epoch=self.epoch, capabilities=welcome.get("capabilities"))
        self._reader = asyncio.create_task(self._read(self.ws))
        if getattr(self, "_echoer", None) is None:
            self._echoer = asyncio.create_task(self._echo_loop())

    async def _echo_loop(self):
        """As the web console: answer every tick while connected, whatever the scenario is waiting on."""
        while True:
            try:
                await self.echo()
            except Exception:  # noqa: BLE001 -- a closed socket during a reconnect
                pass
            await asyncio.sleep(0.05)

    async def open_media(self):
        self.media = await connect(f"ws://127.0.0.1:{self.port}/api/v1/media", max_size=None)
        await self.media.send(json.dumps({"t": "media_subscribe", "camera_rgb": {"max_hz": 10},
                                          "scan": {"max_hz": 10}, "grid": {"max_hz": 1}}))
        self._media_reader = asyncio.create_task(self._read_media())

    async def _read(self, ws):
        try:
            async for raw in ws:
                msg = json.loads(raw)
                t = msg.get("t")
                if t == "tick":
                    self.tick = msg
                elif t == "state":
                    self.state = msg
                    self.control_gen = msg.get("control_gen", self.control_gen)
                    self.states.append({"mono": time.monotonic(), "mode": msg.get("mode"),
                                        "stop_kind": msg.get("stop_kind"), "armed": msg.get("armed"),
                                        "applied": msg.get("applied"), "speed": msg.get("rear_axle_speed_mps"),
                                        "mission": msg.get("mission"), "pacing": msg.get("pacing"),
                                        "latch": msg.get("stop_latch"), "reject": msg.get("last_teleop_reject"),
                                        "teleop_forbidden": msg.get("teleop_forbidden_reason")})
                elif t == "targets":
                    self.targets = msg
                elif t in ("ack", "reject"):
                    self.replies[msg.get("request_id")] = (time.monotonic(), msg)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001
            if ws is self.ws:
                self.closed_code = getattr(ws, "close_code", None)
            self.event("control_closed", code=getattr(ws, "close_code", None), error=repr(exc), current=ws is self.ws)
        else:
            if ws is self.ws:
                self.closed_code = ws.close_code
            self.event("control_closed", code=ws.close_code, current=ws is self.ws)

    async def _read_media(self):
        try:
            async for raw in self.media:
                if not isinstance(raw, bytes):
                    continue
                (n,) = struct.unpack(">I", raw[:4])
                head = json.loads(raw[4:4 + n])
                body = raw[4 + n:]
                if head.get("t") == "camera":
                    self.frames.append({"mono": time.monotonic(), "frame_id": head["frame_id"],
                                        "acquired_mono_ns": head["acquired_mono_ns"], "jpeg": body[:2] == b"\xff\xd8",
                                        "bytes": len(body), "stamp_ns": head.get("capture", {}).get("stamp_ns")})
                    if self.save_frames is not None and len(self.frames) <= self.save_frames[1]:
                        (self.save_frames[0] / f"frame_{head['frame_id']:06d}.jpg").write_bytes(body)
        except Exception as exc:  # noqa: BLE001
            self.event("media_closed", error=repr(exc))

    # --- requests -----------------------------------------------------------------------------------------------
    async def request(self, kind, timeout=5.0, **fields):
        reply = await self._request(kind, timeout, **fields)
        if reply is not None and reply.get("reason") in ("no_lease", "stale_lease") and kind != "lease_request":
            # As the console would: the lease was lost (a long tick on the vehicle); take it again, ask once more.
            self.event("lease_lost", kind=kind)
            lease = await self._request("lease_request", timeout)
            if lease is not None and lease.get("t") == "ack":
                self.lease_id = lease["lease_id"]
                self.control_gen = lease.get("control_gen", self.control_gen)
                fields.pop("lease_id", None)
                fields.pop("control_gen", None)
                reply = await self._request(kind, timeout, **fields)
        return reply

    async def _request(self, kind, timeout=5.0, **fields):
        self.n += 1
        rid = f"r{self.n}"
        msg = {"t": kind, "request_id": rid, "session_epoch": self.epoch, **fields}
        if kind not in ("stop", "lease_request"):
            msg.setdefault("lease_id", self.lease_id)
            msg.setdefault("control_gen", self.control_gen)
        sent = time.monotonic()
        await self.ws.send(json.dumps(msg))
        end = sent + timeout
        while time.monotonic() < end:
            if rid in self.replies:
                at, reply = self.replies[rid]
                self.event("reply", kind=kind, request_id=rid, t=reply["t"], reason=reply.get("reason"),
                           detail=reply.get("detail"), rtt_s=at - sent)
                if reply.get("control_gen") is not None:
                    self.control_gen = reply["control_gen"]
                return reply
            await asyncio.sleep(0.01)
        self.event("reply_timeout", kind=kind, request_id=rid)
        return None

    async def request_until_standing(self, kind, tries=20, **fields):
        """A request the vehicle refuses until its own detector confirms standing (not_stopped): asked again."""
        reply = None
        for _ in range(tries):
            reply = await self.request(kind, **fields)
            if reply is None or reply.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed"):
                return reply
            await asyncio.sleep(0.5)
        return reply

    async def echo(self):
        if self.tick is not None and self.lease_id is not None:
            await self.ws.send(json.dumps({"t": "echo", "session_epoch": self.epoch, "lease_id": self.lease_id,
                                           "echo": {"state_seq": self.tick["state_seq"], "token": self.tick["token"]}}))

    async def teleop(self, speed, curvature, deadman=True, mode="direct"):
        if self.tick is None:
            return
        self.seq += 1
        msg = {"t": "teleop", "session_epoch": self.epoch, "lease_id": self.lease_id, "control_gen": self.control_gen,
               "arm_gen": self.arm_gen, "seq": self.seq,
               "echo": {"state_seq": self.tick["state_seq"], "token": self.tick["token"]}, "mode": mode,
               "deadman": deadman, "speed_ratio": speed, "curvature_ratio": curvature, "lift_dir": 0}
        if mode == "video" and self.frames:
            msg["frame_echo"] = {"frame_id": self.frames[-1]["frame_id"]}
        await self.ws.send(json.dumps(msg))
        if (speed, curvature, deadman) != self.last_teleop:
            # The control round trip (plan completion 2, agy G2) is read from these against the states' applied
            # command: same client clock at both ends.
            self.last_teleop = (speed, curvature, deadman)
            self.report.setdefault("teleop_sent", []).append(
                {"mono": time.monotonic(), "seq": self.seq, "speed_ratio": speed, "curvature_ratio": curvature,
                 "deadman": deadman, "mode": mode})

    async def keep_alive(self, seconds, hz=20):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            await self.echo()
            await asyncio.sleep(1 / hz)

    async def until(self, pred, timeout, hz=20, label=""):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.state is not None and pred(self.state):
                self.event("reached", label=label)
                return True
            if self.closed_code is not None:
                self.event("closed_while_waiting", label=label, code=self.closed_code)
                return False
            await asyncio.sleep(1 / hz)  # the background echo answers the ticks
        self.event("timeout", label=label)
        return False

    def standing(self, s):
        v = s.get("rear_axle_speed_mps")
        return v is not None and abs(v) < 0.01


async def main_scenario(p: Probe, args):
    await p.open()
    await p.open_media()
    await p.until(lambda s: True, 10, label="first_state")
    lease = await p.request("lease_request")
    p.lease_id = lease.get("lease_id") if lease else None
    p.report["session"] = {"session_epoch": p.epoch, "lease_id": p.lease_id}
    await p.until(lambda s: p.targets is not None, 10, label="targets")
    await p.until(p.standing, 20, label="standing_at_start")
    submit = None
    for _ in range(20):  # the vehicle confirms standing on its own detector: a not_stopped is asked again
        submit = await p.request("mission_submit", pallet_id="P-1", destination_id="D-1",
                                 targets_revision=p.targets["targets_revision"])
        if submit is None or submit.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed"):
            break
        await asyncio.sleep(0.5)
    mission_id = (submit or {}).get("mission_id")
    await p.until(lambda s: s.get("mode") == "AUTO", 5, label="auto")
    # A STOP inside the first capture: the runner's test option stops it there (a client cannot aim for those
    # steps); without that option the probe sends its own STOP once the mission has driven and stands.
    await p.until(lambda s: abs(s.get("rear_axle_speed_mps") or 0.0) > 0.05, 60, label="moving")
    if args.expect_stop_in_capture:
        await p.until(lambda s: s.get("mode") == "STOPPED", args.observe_timeout_s, label="stopped_in_capture")
    else:
        await p.until(lambda s: (s.get("mission") or {}).get("phase") == "observe" and p.standing(s),
                      args.observe_timeout_s, label="observe_standing")
        await p.request("stop", source="key")
    await p.until(lambda s: s.get("mode") == "STOPPED", 3, label="stopped")
    await p.until(p.standing, 10, label="standing_after_stop")
    await asyncio.sleep(0.5)
    await p.request_until_standing("rearm")
    await p.until(p.standing, 5, label="standing_before_resume")
    await asyncio.sleep(0.4)
    await p.request_until_standing("mission_resume", mission_id=mission_id)
    await p.until(lambda s: s.get("mode") == "AUTO", 5, label="auto_again")
    # Let it drive, then pause and take it by hand.
    await p.keep_alive(args.auto_s)
    await p.request("mode_request", to="PAUSED")
    await p.until(p.standing, 15, label="standing_paused")
    await asyncio.sleep(0.5)
    teleop = None
    for _ in range(40):  # pacing turns ok only after a held second without a stall; standing is the vehicle's call
        teleop = await p.request("mode_request", to="TELEOP")
        if teleop is None or (teleop.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed")
                              and teleop.get("detail") != "pacing"):
            break
        await asyncio.sleep(0.5)
    p.report["teleop_entry"] = teleop
    if teleop and teleop["t"] == "ack":
        arm = await p.request("arm")
        p.arm_gen = (arm or {}).get("arm_gen", 0)
        # 30 s of driving: forward with a curvature sweep, dead man released, reverse, forward again.
        start = time.monotonic()
        first_move = None
        while time.monotonic() - start < 30.0:
            el = time.monotonic() - start
            if el < 8:
                speed, curv, dm = 0.6, math.sin(el * 0.8), True
            elif el < 11:
                speed, curv, dm = 0.0, 0.0, False
            elif el < 19:
                speed, curv, dm = -0.6, 0.3, True
            else:
                speed, curv, dm = 0.5, -0.4, True
            await p.teleop(speed, curv, dm, mode="video" if 22 < el < 26 else "direct")
            if first_move is None and p.state and (p.state.get("applied") or {}).get("speed_mps"):
                first_move = time.monotonic() - start
            await asyncio.sleep(0.05)
        p.report["teleop_first_motion_s"] = first_move
        # Silence: echo only, no command -- the command must expire on the vehicle's clock.
        silent_at = time.monotonic()
        p.report["silence_start_mono"] = silent_at
        await p.until(lambda s: not s.get("armed"), 15.0, label="expired_disarmed")  # the age limit is 0.3/f in Isaac
        await p.request("mode_request", to="PAUSED")
    # Reconnect within the grace: a new lease id; the old one no longer counts.
    old_lease = p.lease_id
    await p.ws.close()
    await asyncio.sleep(0.5)
    await p.open()
    lease = await p.request("lease_request")
    p.lease_id = (lease or {}).get("lease_id")
    p.report["reconnect"] = {"old_lease": old_lease, "new_lease": p.lease_id}
    p.lease_id, saved = old_lease, p.lease_id
    # A negative test: the first answer is the evidence, no lease recovery (Codex M-c round 8 P2).
    p.report["old_lease_after_reconnect"] = await p._request("mode_request", to="PAUSED")
    p.lease_id = saved
    await p.until(p.standing, 10, label="standing_before_final_resume")
    await asyncio.sleep(0.5)
    await p.request_until_standing("mission_resume", mission_id=mission_id)
    end_by = time.monotonic() + args.mission_timeout_s
    while time.monotonic() < end_by and p.closed_code is None:
        await p.until(lambda s: (s.get("mission") or {}).get("status") in ("done", "failed", "cancelled")
                      or s.get("mode") == "STOPPED", end_by - time.monotonic(), label="mission_end_or_stop")
        if p.state is None or p.state.get("mode") != "STOPPED" or p.closed_code is not None:
            break
        # Stopped on the way (a STOP inside a later capture, a fault): stand, rearm and resume, as an operator would.
        await p.until(p.standing, 20, label="standing_again")
        await p.request_until_standing("rearm")
        await p.request_until_standing("mission_resume", mission_id=mission_id)
    end = time.monotonic() + 60
    while p.closed_code is None and time.monotonic() < end:
        await asyncio.sleep(0.05)
    p.report["close_code"] = p.closed_code


async def marker_scenario(p: Probe, args):
    """Plan C5: save the frames (the runner's marker tells when their pixels were drawn); then, with the runner's
    camera freeze (--live-test-camera-freeze FROM:TO, sim s), hold TELEOP and send video-mode commands (dead man
    released: nothing moves) across the freeze -- they must be refused stale_video and disarm."""
    args.frames_dir.mkdir(parents=True, exist_ok=True)
    p.save_frames = (args.frames_dir, args.frames)
    await p.open()
    await p.open_media()

    def sim_now():
        return p.frames[-1]["stamp_ns"] / 1e9 if p.frames and p.frames[-1].get("stamp_ns") else 0.0

    end = time.monotonic() + args.marker_s
    while time.monotonic() < end and len(p.frames) < args.frames and p.closed_code is None:
        await asyncio.sleep(0.2)
    if args.freeze_from is None:
        return
    lease = await p.request("lease_request")
    p.lease_id = (lease or {}).get("lease_id")
    await p.until(p.standing, 30, label="standing")
    for _ in range(40):
        teleop = await p.request("mode_request", to="TELEOP")
        if teleop is None or (teleop.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed")
                              and teleop.get("detail") != "pacing"):
            break
        await asyncio.sleep(0.5)
    arm = await p.request("arm")
    p.arm_gen = (arm or {}).get("arm_gen", 0)
    p.report["freeze"] = {"teleop": teleop, "arm": arm, "samples": []}
    stop_at = time.monotonic() + args.marker_s
    while time.monotonic() < stop_at and p.closed_code is None:
        await p.teleop(0.0, 0.0, deadman=False, mode="video")
        st = p.state or {}
        p.report["freeze"]["samples"].append({"mono": time.monotonic(), "last_frame_sim": sim_now(),
                                              "armed": st.get("armed"), "reject": st.get("last_teleop_reject")})
        if not st.get("armed") and st.get("last_teleop_reject") == "stale_video":
            p.event("stale_video_disarmed", last_frame_sim=sim_now())
            break
        await asyncio.sleep(0.05)


async def slow_scenario(p: Probe, args):
    await p.open()
    await p.open_media()
    if args.old_session:
        old = json.loads(Path(args.old_session).read_text()).get("session", {})
        p.n += 1
        await p.ws.send(json.dumps({"t": "lease_release", "request_id": "old", "session_epoch": old.get("session_epoch"),
                                    "lease_id": old.get("lease_id")}))
        await asyncio.sleep(0.5)
        p.report["old_session_reply"] = p.replies.get("old", (None, None))[1]
    lease = await p.request("lease_request")
    p.lease_id = (lease or {}).get("lease_id")
    await p.until(p.standing, 20, label="standing")
    await p.keep_alive(3.0)
    p.report["teleop_entry_slow"] = await p.request_until_standing("mode_request", to="TELEOP")
    await p.keep_alive(args.auto_s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port-file", type=Path, required=True)
    ap.add_argument("--report", type=Path, required=True)
    ap.add_argument("--scenario", choices=("main", "slow", "marker"), default="main")
    ap.add_argument("--frames-dir", type=Path, default=None)
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--marker-s", type=float, default=600.0)
    ap.add_argument("--freeze-from", type=float, default=None, help="the runner's camera freeze start (sim s)")
    ap.add_argument("--old-session", type=Path, default=None)
    ap.add_argument("--observe-timeout-s", type=float, default=240.0)
    ap.add_argument("--expect-stop-in-capture", action="store_true",
                    help="the runner has --live-test-stop-in-capture: wait for that stop instead of sending one")
    ap.add_argument("--auto-s", type=float, default=15.0)
    ap.add_argument("--mission-timeout-s", type=float, default=900.0)
    ap.add_argument("--wait-port-s", type=float, default=900.0)
    args = ap.parse_args()
    end = time.monotonic() + args.wait_port_s
    info = None
    while time.monotonic() < end:
        if args.port_file.exists():
            info = json.loads(args.port_file.read_text())
            if info.get("state") == "running":
                break
        time.sleep(1.0)
    report = {"port_file": info}
    p = Probe(info["port"], report)
    try:
        scenario = {"main": main_scenario, "slow": slow_scenario, "marker": marker_scenario}[args.scenario]
        asyncio.run(scenario(p, args))
    finally:
        report["frames"] = p.frames
        report["states"] = p.states
        args.report.write_text(json.dumps(report, default=str))


if __name__ == "__main__":
    main()
