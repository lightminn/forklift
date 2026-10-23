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
import signal
import struct
import sys
import time
from pathlib import Path

import websockets
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
        self.missions = None  # the session's results book (persistent console plan D1)
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
        self.queue = None  # the queue scenario's tools/mission_queue.MissionQueue: fed every state and result
        self.queue_has_lease = False  # the lease as console/web/js/protocol.js holds it (a state may take it away)

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
        # persistent console plan D7: the connection's request token, its seq (a reconnect gets a new token)
        self.request_token, self.request_seq = welcome.get("request_token"), 0
        self.event("welcome", session_epoch=self.epoch, capabilities=welcome.get("capabilities"))
        self._reader = asyncio.create_task(self._read(self.ws))
        if getattr(self, "_echoer", None) is None:
            self._echoer = asyncio.create_task(self._echo_loop())

    async def _echo_loop(self):
        """As the web console: answer every tick while connected, whatever the scenario is waiting on."""
        while True:
            try:
                if not getattr(self, "silent", False):  # plan D14 U20 b: the client goes quiet
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
                    if self.queue is not None:  # as console/web/js/queuewire.js onState: state, lease, result
                        now_ms_ = time.monotonic() * 1000.0
                        had_ = self.queue_has_lease
                        if msg.get("lease") and msg["lease"].get("holder") != self.client_id:
                            self.queue_has_lease = False  # protocol.js: the holder is not this client
                        self.queue.on_state(msg, now_ms_)
                        if had_ and not self.queue_has_lease:
                            self.queue.on_lease_lost()
                        if msg.get("last_result"):
                            self.queue.on_result(msg["last_result"], now_ms_)
                    self.states.append({"mono": time.monotonic(), "mode": msg.get("mode"),
                                        "stop_kind": msg.get("stop_kind"), "armed": msg.get("armed"),
                                        "applied": msg.get("applied"), "speed": msg.get("rear_axle_speed_mps"),
                                        "mission": msg.get("mission"), "pacing": msg.get("pacing"),
                                        "latch": msg.get("stop_latch"), "reject": msg.get("last_teleop_reject"),
                                        "teleop_forbidden": msg.get("teleop_forbidden_reason"),
                                        "permit": msg.get("permit"), "last_result": msg.get("last_result")})
                elif t == "targets":
                    self.targets = msg
                elif t == "missions":
                    self.missions = msg
                    if self.queue is not None:  # queuewire onResults: oldest first
                        for r_ in sorted(msg.get("results") or [], key=lambda r: r.get("seq") or 0):
                            self.queue.on_result(r_, time.monotonic() * 1000.0)
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

    async def _request(self, kind, timeout=5.0, request_id=None, **fields):
        self.n += 1
        rid = request_id or f"r{self.n}"
        msg = {"t": kind, "request_id": rid, "session_epoch": self.epoch, **fields}
        if kind != "stop" and getattr(self, "request_token", None) is not None:
            self.request_seq += 1
            msg.update(request_token=self.request_token, request_seq=self.request_seq)
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

    async def request_resent(self, kind, request_id, retry_s=0.5, **fields):
        """protocol.js request() + pump(): the same message resent every retry_s until it is answered (no timeout --
        the caller stops waiting when it no longer needs the answer)."""
        msg = {"t": kind, "request_id": request_id, "session_epoch": self.epoch, **fields}
        if getattr(self, "request_token", None) is not None:
            self.request_seq += 1
            msg.update(request_token=self.request_token, request_seq=self.request_seq)  # a resend is this message
        msg.setdefault("lease_id", self.lease_id)
        msg.setdefault("control_gen", self.control_gen)
        raw = json.dumps(msg)
        sent = time.monotonic()
        while self.closed_code is None:
            await self.ws.send(raw)
            end = time.monotonic() + retry_s
            while time.monotonic() < end:
                if request_id in self.replies:
                    at, reply = self.replies[request_id]
                    self.event("reply", kind=kind, request_id=request_id, t=reply["t"], reason=reply.get("reason"),
                               detail=reply.get("detail"), rtt_s=at - sent)
                    if reply.get("control_gen") is not None:
                        self.control_gen = reply["control_gen"]
                    return reply
                await asyncio.sleep(0.01)
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

    async def teleop(self, speed, curvature, deadman=True, mode="direct", lift_dir=0):
        if self.tick is None:
            return
        self.seq += 1
        msg = {"t": "teleop", "session_epoch": self.epoch, "lease_id": self.lease_id, "control_gen": self.control_gen,
               "arm_gen": self.arm_gen, "seq": self.seq,
               "echo": {"state_seq": self.tick["state_seq"], "token": self.tick["token"]}, "mode": mode,
               "deadman": deadman, "speed_ratio": speed, "curvature_ratio": curvature, "lift_dir": int(lift_dir)}
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
    seen_ = ((p.state or {}).get("last_result") or {}).get("seq", 0)
    await p.request_until_standing("mission_resume", mission_id=mission_id)
    end_by = time.monotonic() + args.mission_timeout_s

    def ended_(s):
        # an ended mission leaves the state's mission slot (persistent console plan D1): its result is the end
        # (U13b pc14: the failure at sim 240 s was missed and the wait ran to its limit)
        r_ = s.get("last_result") or {}
        return (s.get("mission") or {}).get("status") in ("done", "failed", "cancelled") or (
            r_.get("seq", 0) > seen_ and r_.get("mission_id") == mission_id)

    while time.monotonic() < end_by and p.closed_code is None:
        await p.until(lambda s: ended_(s) or s.get("mode") == "STOPPED", end_by - time.monotonic(),
                      label="mission_end_or_stop")
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


async def approach_scenario(p: Probe, args):
    """M-c completion 5: teleoperation up to an obstacle the 2D LiDAR sees. From the start pose (rear axle at
    x -2.34 facing +x) a slight right curve passes the pallet (it sits 1 m to the left; the 1.05 m scan plane does
    not see it -- nor does the teleoperation permission) towards the hall wall at x 4.7: the arc bundle permission must slow the truck and hold it short of
    the wall. Then reverse with the steering swung the other way (a steering change near the obstacle), then
    forward again straight at the wall. The runner's --live-test-permit-compare logs the bundle against a full
    check along the way."""
    await p.open()
    await p.open_media()
    lease = await p.request("lease_request")
    p.lease_id = (lease or {}).get("lease_id")
    await p.until(p.standing, 30, label="standing")
    teleop = None
    for _ in range(60):
        teleop = await p.request("mode_request", to="TELEOP")
        if teleop is None or (teleop.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed")
                              and teleop.get("detail") != "pacing"):
            break
        await asyncio.sleep(0.5)
    p.report["teleop_entry"] = teleop
    if not teleop or teleop.get("t") != "ack":
        return
    arm = await p.request("arm")
    p.arm_gen = (arm or {}).get("arm_gen", 0)
    legs = (("forward_to_wall", 0.6, args.approach_curvature, args.approach_s),
            ("reverse_swing", -0.6, 0.4, args.approach_s / 4), ("forward_again", 0.6, 0.0, args.approach_s))
    if args.approach_weave_s > 0:
        # Teleop bundle margin plan V3: steering swung back and forth before the approach, so the bundle's arcs
        # change while the truck drives (the curvature ratio flips every weave period, wall seconds).
        legs = (("weave", 0.6, "weave", args.approach_weave_s * args.approach_weave_n),) + legs
    p.report["approach"] = []
    for name, speed, curv, limit_s in legs:
        start = time.monotonic()
        still_since = None
        moved = False
        while time.monotonic() - start < limit_s and p.closed_code is None:
            now_curv = curv
            if curv == "weave":
                now_curv = 0.8 if int((time.monotonic() - start) / args.approach_weave_s) % 2 == 0 else -0.8
            try:
                await p.teleop(speed, now_curv, True)
            except websockets.ConnectionClosed:
                break  # the runner ended (its own checks stop a run): recorded below
            st = p.state or {}
            v = abs(st.get("rear_axle_speed_mps") or 0.0)
            moved = moved or v > 0.05
            applied_ = abs((st.get("applied") or {}).get("speed_mps") or 0.0)
            if moved and applied_ == 0.0 and v < 0.03:  # the vehicle holds it (the measured speed jitters ~0.02)
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > args.held_s and name not in ("reverse_swing", "weave"):
                    break  # held short of the obstacle with the command still asking to go
            else:
                still_since = None
            await asyncio.sleep(0.05)
        st = p.state or {}
        p.report["approach"].append({"leg": name, "wall_s": time.monotonic() - start, "moved": moved,
                                     "held": still_since is not None, "armed": st.get("armed"),
                                     "applied": st.get("applied"), "permit": st.get("permit"),
                                     "forbidden": st.get("teleop_forbidden_reason")})
        p.event("leg_done", leg=name)
        if p.closed_code is not None:
            return
    await p.request("mode_request", to="PAUSED")


async def planwait_scenario(p: Probe, args):
    """Real-time plan V8-5: a pause, a STOP and a cancel each sent while the mission stands for its plan.

    The state does not say a plan is being waited for, so the requests are timed: the observation's mission plan
    takes ~12-16 wall s after the truck stands for the far capture (and again after each resume, which captures
    again), longer than any other stand in AUTO observe (a gear change stands well under a second); each request
    goes once the truck has stood ``--wait-offset-s`` there. The run's ``async_plans`` says afterwards whether it
    landed in a wait (tools/check_planwait_probe.py)."""
    await p.open()
    await p.open_media()
    await p.until(lambda s: True, 10, label="first_state")
    lease = await p.request("lease_request")
    p.lease_id = (lease or {}).get("lease_id")
    await p.until(lambda s: p.targets is not None, 10, label="targets")
    await p.until(p.standing, 20, label="standing_at_start")
    submit = None
    for _ in range(20):
        submit = await p.request("mission_submit", pallet_id="P-1", destination_id="D-1",
                                 targets_revision=p.targets["targets_revision"])
        if submit is None or submit.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed"):
            break
        await asyncio.sleep(0.5)
    mission_id = (submit or {}).get("mission_id")
    await p.until(lambda s: s.get("mode") == "AUTO", 5, label="auto")
    await p.until(lambda s: abs(s.get("rear_axle_speed_mps") or 0.0) > 0.05, 60, label="moving")

    async def in_plan_wait(label):
        """AUTO, observe, standing without a break for --wait-offset-s."""
        since, end = None, time.monotonic() + args.observe_timeout_s
        while time.monotonic() < end:
            s = p.state or {}
            # the vehicle's own stand confirmation: the measured speed of a standing truck wanders past 0.01 m/s
            if (s.get("mode") == "AUTO" and (s.get("mission") or {}).get("phase") == "observe"
                    and s.get("stopped_confirmed")):
                since = since or time.monotonic()
                if time.monotonic() - since >= args.wait_offset_s:
                    p.event("reached", label=label)
                    return True
            else:
                since = None
            await asyncio.sleep(0.05)
        p.event("timeout", label=label)
        return False

    async def watch_standing(seconds, key):
        """The truck must not move while paused, stopped or cancelled: the largest speed in the states received
        meanwhile, and how many (the judge, tools/check_planwait_probe.py, reads the vehicle's own record)."""
        start = time.monotonic()
        await asyncio.sleep(seconds)
        speeds = [s["speed"] for s in p.states if s["mono"] >= start]
        p.report[key] = {"max": max((abs(v) for v in speeds if v is not None), default=None),
                         "samples": len(speeds), "missing": sum(v is None for v in speeds)}

    # 1. pause inside the wait, stand, resume
    await in_plan_wait("plan_wait_1")
    p.report["pause_sent_mono"] = time.monotonic()
    p.report["pause_reply"] = await p.request("mission_pause", mission_id=mission_id)
    await p.until(lambda s: s.get("mode") == "PAUSED", 5, label="paused_in_wait")
    await watch_standing(args.hold_s, "paused_max_speed_mps")
    p.report["resume_after_pause"] = await p.request_until_standing("mission_resume", mission_id=mission_id)
    await p.until(lambda s: s.get("mode") == "AUTO", 5, label="auto_after_pause")
    # 2. STOP inside the plan asked again after the resume, rearm, resume
    await in_plan_wait("plan_wait_2")
    p.report["stop_sent_mono"] = time.monotonic()
    p.report["stop_reply"] = await p.request("stop", source="key")
    await p.until(lambda s: s.get("mode") == "STOPPED", 3, label="stopped_in_wait")
    await watch_standing(args.hold_s, "stopped_max_speed_mps")
    await p.request_until_standing("rearm")
    await p.until(p.standing, 5, label="standing_before_resume")
    p.report["resume_after_stop"] = await p.request_until_standing("mission_resume", mission_id=mission_id)
    await p.until(lambda s: s.get("mode") == "AUTO", 5, label="auto_after_stop")
    # 3. cancel inside the next wait: no plan may be taken afterwards
    await in_plan_wait("plan_wait_3")
    p.report["cancel_sent_mono"] = time.monotonic()
    p.report["cancel_reply"] = await p.request("mission_cancel", mission_id=mission_id)
    await p.until(lambda s: (s.get("mission") or {}).get("status") == "cancelled", 10, label="cancelled_in_wait")
    await watch_standing(args.after_cancel_s, "cancelled_max_speed_mps")
    p.report["final_state"] = {k: (p.state or {}).get(k) for k in ("mode", "mission", "rear_axle_speed_mps")}


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


async def teleop_drive(p: Probe, seconds: float, speed: float, curvature: float) -> dict | None:
    """Pause, take the truck by hand, drive for ``seconds`` wall s, stop and hand back (PAUSED)."""
    await p.request("mode_request", to="PAUSED")
    await p.until(p.standing, 20, label="standing_before_teleop")
    teleop = None
    for _ in range(40):
        teleop = await p.request("mode_request", to="TELEOP")
        if teleop is None or (teleop.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed")
                              and teleop.get("detail") != "pacing"):
            break
        await asyncio.sleep(0.5)
    if not teleop or teleop["t"] != "ack":
        return teleop
    arm = await p.request("arm")
    p.arm_gen = (arm or {}).get("arm_gen", 0)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        await p.teleop(speed, curvature, True)
        await asyncio.sleep(0.05)
    stop_by = time.monotonic() + 15.0
    while time.monotonic() < stop_by and not (p.state and p.standing(p.state)):
        await p.teleop(0.0, 0.0, True)
        await asyncio.sleep(0.05)
    await p.request("mode_request", to="PAUSED")
    return teleop


async def back_off_by(p: Probe, low_m: float, high_m: float, max_s: float) -> dict:
    """Reverse into the advised interval [``low_m``, ``high_m``] by the reported rear-axle speed, integrated over the
    states that arrive (a state not renewed adds nothing -- Codex D13 impl round 1 P2), aiming at its middle, crawling
    in the last 0.20 m, the stop included; short of the interval after the stop, crawl again (at most 3 times). Says
    how far it went and whether that is inside the interval -- a fixed tolerance is no proof (Codex round 2 P2)."""
    moved, seen = 0.0, None
    stop_by = time.monotonic() + max_s
    target = 0.5 * (low_m + high_m)

    def take() -> float:
        nonlocal seen
        st = p.state or {}
        stamp = (st.get("state_seq"), id(p.state))
        if p.state is None or stamp == seen:
            return 0.0
        prev, seen = seen, stamp
        now = time.monotonic()
        dt = 0.0 if prev is None or take.at is None else now - take.at
        take.at = now
        v = st.get("rear_axle_speed_mps")
        pacing = st.get("pacing") or {}
        factor = pacing.get("realtime_factor") or pacing.get("target_factor") or 1.0  # sim s per wall s
        return -float(v) * dt * float(factor) if v is not None and v < 0 and dt < 0.5 else 0.0

    async def stop_and_count():
        nonlocal moved
        settle_by = time.monotonic() + 15.0
        while time.monotonic() < settle_by and p.closed_code is None and not (p.state and p.standing(p.state)):
            await p.teleop(0.0, 0.0, True)
            await asyncio.sleep(0.05)
            moved += take()  # the coasting counts

    take.at = None
    goal = target
    for attempt in range(4):
        while moved < goal and time.monotonic() < stop_by and p.closed_code is None:
            left = goal - moved
            ratio = max(0.05, min(0.5, 0.5 * left / 0.20))  # a crawl in the last 0.20 m
            await p.teleop(-ratio, 0.0, True)
            await asyncio.sleep(0.05)
            moved += take()
        out_of_time = time.monotonic() >= stop_by or p.closed_code is not None
        await stop_and_count()
        if moved >= low_m or attempt == 3 or out_of_time:
            break  # the budget spent: no new crawl, and one stop wait only (Codex D13 impl round 3 P3)
        goal = low_m + 0.25 * (high_m - low_m)  # short: crawl again into the interval's first quarter
    return {"moved_m": moved, "target_m": target, "within": low_m <= moved <= high_m}


async def enter_teleop(p: Probe) -> dict | None:
    """PAUSED, standing, then TELEOP (retried while the vehicle has not confirmed standing) and armed."""
    await p.request("mode_request", to="PAUSED")
    await p.until(p.standing, 30, label="standing_before_teleop")
    reply = None
    for _ in range(60):
        reply = await p.request("mode_request", to="TELEOP")
        if reply is None or (reply.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed")
                             and reply.get("detail") != "pacing"):
            break
        await asyncio.sleep(0.5)
    if not reply or reply["t"] != "ack":
        return reply
    arm = await p.request("arm")
    p.arm_gen = (arm or {}).get("arm_gen", 0)
    return reply


async def manual_recovery(p: Probe, args) -> dict:
    """Plan D14 U20 c: an ended recovery_required mission recovered by hand -- lower while only lowering is
    allowed, reverse straight while the drive mode says so, until the recovery stage is gone (normal teleop)."""
    out = {"enter": await enter_teleop(p), "stages": [], "lift_m": []}
    end = time.monotonic() + args.mission_timeout_s
    last = None
    while time.monotonic() < end and p.closed_code is None:
        st = p.state or {}
        rec, dirs, mode = st.get("recovery"), st.get("lift_dirs") or [], st.get("drive_mode")
        key = (None if rec is None else (rec.get("stage"), rec.get("grounded"), rec.get("aligned"), rec.get("evidence")),
               tuple(dirs), mode, st.get("teleop_forbidden_reason"))
        if key != last:
            out["stages"].append({"mono": time.monotonic(), "recovery": rec, "lift_dirs": dirs, "drive_mode": mode,
                                  "forbidden": st.get("teleop_forbidden_reason"), "lift_m": st.get("lift_m")})
            last = key
        if rec is None and mode == "normal":
            break
        if st.get("mode") != "TELEOP" or not st.get("armed"):
            # Not in TELEOP (entry refused while the sim lagged real time, or a preempt disarmed it): as an operator
            # would, ask again rather than send commands nobody takes.
            out.setdefault("reentries", []).append({"mono": time.monotonic(), "mode": st.get("mode"),
                                                    "armed": st.get("armed")})
            await enter_teleop(p)
            continue
        if dirs == [-1]:
            await p.teleop(0.0, 0.0, True, lift_dir=-1)
            await asyncio.sleep(0.05)
        elif mode == "reverse_straight":
            # In short strokes: the free verdict needs the truck standing (Codex D14 impl round 2 P2) -- back off,
            # stand, give the verdict a moment, again.
            stroke_end = time.monotonic() + args.manual_stroke_s
            while time.monotonic() < stroke_end and (p.state or {}).get("drive_mode") == "reverse_straight":
                await p.teleop(-1.0, 0.0, True)
                await asyncio.sleep(0.05)
            stand_by = time.monotonic() + 15.0
            while time.monotonic() < stand_by and not (p.state and p.standing(p.state)):
                await p.teleop(0.0, 0.0, True)
                await asyncio.sleep(0.05)
            settle_end = time.monotonic() + 1.5
            while time.monotonic() < settle_end:
                await p.teleop(0.0, 0.0, True)
                await asyncio.sleep(0.05)
        else:
            await p.teleop(0.0, 0.0, True)
            await asyncio.sleep(0.05)
    out["done"] = (p.state or {}).get("recovery") is None
    # a little further back by hand, normally, so the next preparation has room (plan D4b checks it at the submit)
    stop_by = time.monotonic() + args.manual_extra_s
    while time.monotonic() < stop_by:
        await p.teleop(-0.5, 0.0, True)
        await asyncio.sleep(0.05)
    stop_by = time.monotonic() + 15.0
    while time.monotonic() < stop_by and not (p.state and p.standing(p.state)):
        await p.teleop(0.0, 0.0, True)
        await asyncio.sleep(0.05)
    await p.request("mode_request", to="PAUSED")
    return out


async def lift_scenario(p: Probe, args):
    """Plan D14 U20 a/b: the manual lift in an idle session -- up and down at its rate within the travel; the
    client going quiet mid-lift; the runner stopped (SIGSTOP) while the lease runs out, then continued."""
    import os

    await p.open()
    await p.until(lambda s: True, 10, label="first_state")
    lease = await p.request("lease_request")
    p.lease_id = lease.get("lease_id") if lease else None
    await p.until(p.standing, 30, label="standing_at_start")
    rep_ = p.report.setdefault("lift", {})
    rep_["enter"] = await enter_teleop(p)

    async def hold_lift(direction, seconds):
        samples = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            await p.teleop(0.0, 0.0, True, lift_dir=direction)
            samples.append(((p.state or {}).get("lift_m"), (p.state or {}).get("lift_dirs")))
            await asyncio.sleep(0.05)
        return samples

    rep_["up"] = await hold_lift(1, args.lift_up_s)
    rep_["after_up"] = await hold_lift(0, 3.0)
    # the client goes quiet mid-lift (no echo, no commands) for longer than the lease, then takes it again
    rep_["before_quiet"] = await hold_lift(1, 2.0)
    p.silent = True
    quiet_start = (p.state or {}).get("lift_m")
    await asyncio.sleep(args.quiet_s)
    rep_["quiet"] = {"start_lift_m": quiet_start, "end_lift_m": (p.state or {}).get("lift_m"), "mode": (p.state or {}).get("mode")}
    p.silent = False
    lease = await p.request("lease_request")
    p.lease_id = lease.get("lease_id") if lease else p.lease_id
    rep_["after_quiet"] = await hold_lift(0, 2.0)
    # the runner stopped while the lease runs out (the 10-17 follow-up's path), then continued
    rep_["reenter"] = await enter_teleop(p)
    rep_["before_stop"] = await hold_lift(1, 2.0)
    pid = (p.report.get("port_file") or {}).get("pid")
    if pid and not args.no_sigstop:
        os.kill(int(pid), signal.SIGSTOP)
        stopped_at = (p.state or {}).get("lift_m")
        await asyncio.sleep(args.quiet_s)
        os.kill(int(pid), signal.SIGCONT)
        rep_["sigstop"] = {"pid": pid, "lift_m_at_stop": stopped_at}
        await asyncio.sleep(3.0)
        rep_["after_sigcont"] = {"lift_m": (p.state or {}).get("lift_m"), "mode": (p.state or {}).get("mode")}
    lease = await p.request("lease_request")
    p.lease_id = lease.get("lease_id") if lease else p.lease_id
    rep_["reenter2"] = await enter_teleop(p)
    rep_["down"] = await hold_lift(-1, args.lift_up_s + 6.0)
    await p.request("mode_request", to="PAUSED")
    if args.end_session:
        await p.request_until_standing("session_end")


async def cycle_scenario(p: Probe, args):
    """Persistent console plan U2-U5: missions one after another in one session, each judged at its end (the
    results book); optionally driven by hand first, so the first mission is prepared from elsewhere."""
    await p.open()
    await p.open_media()
    await p.until(lambda s: True, 10, label="first_state")
    lease = await p.request("lease_request")
    p.lease_id = lease.get("lease_id") if lease else None
    p.report["session"] = {"session_epoch": p.epoch, "lease_id": p.lease_id}
    await p.until(lambda s: p.targets is not None, 10, label="targets")
    await p.until(p.standing, 20, label="standing_at_start")
    if args.idle_s > 0:  # plan U1: left up, nothing asked, the video subscribed (the frames are counted)
        p.report["idle_start_mono"] = time.monotonic()
        await p.keep_alive(args.idle_s)
        p.report["idle_end_mono"] = time.monotonic()
    if args.pre_teleop_s > 0:
        p.report["pre_teleop"] = await teleop_drive(p, args.pre_teleop_s, args.pre_teleop_speed,
                                                    args.pre_teleop_curvature)
    p.report["missions_run"] = []
    legs = [leg for leg in args.cycle.split(",") if leg]  # empty: only the teleop (plan U11)
    leg_index, backoffs = 0, 0
    while leg_index < len(legs):
        leg = legs[leg_index]
        pallet_id, destination_id = leg.split(":")
        await p.until(p.standing, 30, label="standing_before_submit")
        await asyncio.sleep(1.0)
        seen = ((p.state or {}).get("last_result") or {}).get("seq", 0)
        submit = None
        for _ in range(40):  # standing on the vehicle's own detector, and the list's revision as it is now
            submit = await p.request("mission_submit", pallet_id=pallet_id, destination_id=destination_id,
                                     targets_revision=p.targets["targets_revision"])
            if submit is None or submit.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed",
                                                              "stale_targets"):
                break
            await asyncio.sleep(0.5)
        if args.manual_recovery and p.report["missions_run"] and \
                (p.report["missions_run"][-1].get("result") or {}).get("outcome") == "recovery_required":
            p.report["manual_recovery"] = await manual_recovery(p, args)  # plan D14 U20 c, before the next submit
            if not p.report["manual_recovery"]["done"]:
                p.event("manual_recovery_not_done")
                break  # the next mission is not asked of a truck still recovering
            await p.until(p.standing, 30, label="standing_after_manual_recovery")
            await asyncio.sleep(1.0)
            seen = ((p.state or {}).get("last_result") or {}).get("seq", 0)
            for _ in range(40):
                submit = await p.request("mission_submit", pallet_id=pallet_id, destination_id=destination_id,
                                         targets_revision=p.targets["targets_revision"])
                if submit is None or submit.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed",
                                                                  "stale_targets"):
                    break
                await asyncio.sleep(0.5)
        entry = {"pallet_id": pallet_id, "destination_id": destination_id, "submit": submit,
                 "submitted_mono": time.monotonic()}
        p.report["missions_run"].append(entry)
        if not submit or submit.get("t") != "ack":
            leg_index += 1  # refused: the next leg (Codex D14 impl round 4 P2-4 -- it asked the same one forever)
            continue
        if args.stop_wait_at and not p.report.get("stop_wait_done"):
            # Persistent console plan U30: STOP the first mission that reaches PHASE[:SECONDS in it] (the near field
            # tracks there), stand STOPPED for --stop-wait-sim-s of sim time (read from the video frames' stamps: the
            # pacing is not the factor), then rearm and cancel it; the cycle goes on with the next leg.
            phase_, _, after_ = args.stop_wait_at.partition(":")
            after_s_ = float(after_ or 0.0)
            mid_ = submit.get("mission_id")

            def at_phase_(s):
                # this submission, running in the phase (a preparing mission shows the core's retained phase; Codex
                # probe review P2-1)
                m_ = s.get("mission") or {}
                return (m_.get("mission_id") == mid_ and m_.get("status") == "running" and m_.get("phase") == phase_
                        and (m_.get("phase_elapsed_s") or 0.0) >= after_s_)

            def ended_(s):
                r_ = s.get("last_result") or {}
                return r_.get("seq", 0) > seen and r_.get("mission_id") == mid_

            reached_ = await p.until(lambda s: at_phase_(s) or ended_(s), args.mission_timeout_s,
                                     label=f"stop_wait:{phase_}")
            # the mission ended before the phase: no STOP, no wait (recorded; the next mission takes it)
            reached_ = bool(reached_) and p.state is not None and at_phase_(p.state)
            p.report["stop_wait_done"] = reached_

            def fresh_sim_(after_mono):
                # the newest frame *acquired* after ``after_mono``: a capture made before the STOP and sent late does not
                # start it (Codex probe review round 2 P2-2). acquired_mono_ns is the runner's time.monotonic(), the
                # probe's clock on the same host (live.sbatch runs both on one node).
                for f_ in reversed(p.frames):
                    if f_["acquired_mono_ns"] / 1e9 <= after_mono:
                        return None
                    if f_.get("stamp_ns"):
                        return f_["stamp_ns"] / 1e9
                return None

            wait_ = entry["stop_wait"] = {"phase_reached": reached_, "mission_id": mid_}
        if args.stop_wait_at and entry.get("stop_wait", {}).get("phase_reached"):
            wait_["stop_reply"] = await p.request("stop", source="key")
            await p.until(lambda s: s.get("mode") == "STOPPED", 5, label="stopped_for_wait")
            stopped_mono_ = time.monotonic()
            wait_["start_mono"], wait_["start_wall"], wait_["start_sim_s"] = stopped_mono_, time.time(), None
            wall_end_ = time.monotonic() + args.stop_wait_wall_s
            wait_["end_reason"] = "wall_limit"
            while time.monotonic() < wall_end_:
                if p.closed_code is not None:
                    wait_["end_reason"] = "closed"
                    break
                if wait_["start_sim_s"] is None:
                    # the sim time starts at the first capture after the STOP (Codex probe review P2-2)
                    wait_["start_sim_s"] = fresh_sim_(stopped_mono_)
                now_ = fresh_sim_(stopped_mono_)
                if now_ is not None and wait_["start_sim_s"] is not None and now_ - wait_["start_sim_s"] >= args.stop_wait_sim_s:
                    wait_["end_reason"] = "sim_time"
                    break
                await p.echo()
                await asyncio.sleep(0.5)
            wait_["end_mono"], wait_["end_wall"] = time.monotonic(), time.time()  # the RSS samples are wall-stamped
            wait_["end_sim_s"] = fresh_sim_(stopped_mono_)
            wait_["mode_at_end"] = (p.state or {}).get("mode")
            wait_["mission_at_end"] = (p.state or {}).get("mission")
            await p.until(p.standing, 20, label="standing_after_wait")
            wait_["rearm"] = await p.request_until_standing("rearm")
            await p.until(lambda s: s.get("mode") == "PAUSED", 5, label="paused_after_rearm")
            wait_["cancel_reply"] = await p.request("mission_cancel", mission_id=submit.get("mission_id"))
            statuses_ = []
            stop_by_ = time.monotonic() + args.mission_timeout_s
            while time.monotonic() < stop_by_ and p.closed_code is None:
                mission_ = (p.state or {}).get("mission") or {}
                status_ = mission_.get("status")
                if status_ and (not statuses_ or statuses_[-1] != status_):
                    statuses_.append(status_)
                if p.state is not None and ended_(p.state):
                    break
                phase_now_ = str(mission_.get("phase") or "")
                if (status_ == "paused" and mission_.get("mission_id") == mid_
                        and (phase_now_.startswith("recover_") or phase_now_ == "clear_out")
                        and (wait_["cancel_reply"] or {}).get("t") == "ack" and "recovery_resume" not in wait_):
                    # Plan D4: the cancel stood with the forks in the pocket (or, at the approach's end, short of the
                    # start check's clearance: clear_out -- Codex U30 review P2), so it became a recovery, and the
                    # PAUSED mode pauses that at once; nothing moves until the operator resumes it (U30 pc15).
                    wait_["recovery_resume"] = await p.request_until_standing("mission_resume",
                                                                              mission_id=submit.get("mission_id"))
                await asyncio.sleep(0.1)
            wait_["after_cancel_statuses"] = statuses_
        if args.cancel_at and len(p.report["missions_run"]) == 1:
            # Persistent console plan U7/U26/U27: the first mission cancelled in a phase (after some seconds in it);
            # the statuses it goes through to its end are kept (recovering, then cancelled or recovery_required).
            # PHASE@METRES: once the follower has at most METRES left in it (a leg's end, plan D4b's approach end).
            if args.cancel_at.endswith("@docked"):
                # Plan U8: on the docked delivery straight (under the docking's SLAM hold) -- the follower stopped
                # short of the destination to look (at most 0.1 m left), then the docked straight was installed
                # (more than 0.3 m left again). The state does not publish the hold, so its leg change is read.
                phase_ = args.cancel_at.partition("@")[0]
                seen_short_ = []

                def due_(s):
                    m_ = s.get("mission") or {}
                    left_ = m_.get("remaining_m")
                    if m_.get("phase") != phase_ or left_ is None:
                        return False
                    if float(left_) <= 0.1:
                        seen_short_.append(True)
                    return bool(seen_short_) and float(left_) > 0.3
            elif "@" in args.cancel_at:
                phase_, _, left_ = args.cancel_at.partition("@")
                left_m_ = float(left_)

                def due_(s):
                    m_ = s.get("mission") or {}
                    return (m_.get("phase") == phase_ and m_.get("remaining_m") is not None
                            and float(m_["remaining_m"]) <= left_m_)
            else:
                phase_, _, after_ = args.cancel_at.partition(":")
                after_s_ = float(after_ or 0.0)

                def due_(s):
                    m_ = s.get("mission") or {}
                    return m_.get("phase") == phase_ and (m_.get("phase_elapsed_s") or 0.0) >= after_s_
            reached_ = await p.until(due_, args.mission_timeout_s, label=f"phase:{phase_}")
            entry["cancel_phase_reached"] = bool(reached_)
            mid_ = submit.get("mission_id")
            if args.cancel_pause_s > 0 and reached_:
                # Plan U27: paused in the phase for a while first (the forks stay where they are), then cancelled
                entry["pause_reply"] = await p.request("mission_pause", mission_id=mid_)
                await p.until(lambda s: (((s.get("mission") or {}).get("mission_id") == mid_
                                          and (s.get("mission") or {}).get("status") == "paused")), 15,
                              label="paused_before_cancel")
                entry["pause_mono"] = time.monotonic()
                await asyncio.sleep(args.cancel_pause_s)
                await p.until(p.standing, 20, label="standing_before_cancel")
            entry["cancel_reply"] = await p.request("mission_cancel", mission_id=mid_)
            entry["cancel_mono"] = time.monotonic()
            statuses_, phases_ = [], []
            stop_by_ = time.monotonic() + args.mission_timeout_s
            while time.monotonic() < stop_by_ and p.closed_code is None:
                mission_ = (p.state or {}).get("mission") or {}
                for key_, seen_ in (("status", statuses_), ("phase", phases_)):
                    if mission_.get(key_) and (not seen_ or seen_[-1] != mission_[key_]):
                        seen_.append(mission_[key_])
                if ((p.state or {}).get("last_result") or {}).get("seq", 0) > seen:
                    break
                phase_now_ = str(mission_.get("phase") or "")
                if (mission_.get("status") == "paused" and mission_.get("mission_id") == mid_
                        and (phase_now_.startswith("recover_") or phase_now_ == "clear_out")
                        and (entry["cancel_reply"] or {}).get("t") == "ack" and "recovery_resume" not in entry):
                    # as in the U30 stop-wait: a cancel from a pause becomes a recovery the PAUSED mode pauses at once
                    entry["recovery_resume"] = await p.request_until_standing("mission_resume", mission_id=mid_)
                await asyncio.sleep(0.1)
            entry["after_cancel"] = {"statuses": statuses_, "phases": phases_}
        end_by = time.monotonic() + args.mission_timeout_s
        while time.monotonic() < end_by and p.closed_code is None:
            await p.until(lambda s: (((s.get("last_result") or {}).get("seq", 0) > seen
                                      and (s.get("last_result") or {}).get("mission_id") == submit.get("mission_id")))
                          or s.get("mode") == "STOPPED", end_by - time.monotonic(), label=f"end:{leg}")
            if p.state is None or p.state.get("mode") != "STOPPED" or p.closed_code is not None:
                break
            await p.until(p.standing, 20, label="standing_again")
            await p.request_until_standing("rearm")
            await p.request_until_standing("mission_resume", mission_id=submit.get("mission_id"))
        entry["result"] = (p.state or {}).get("last_result")
        if (entry["result"] or {}).get("mission_id") != submit.get("mission_id"):
            # not this submission's end (it never came, or a later one is shown): kept, and said (Codex probe P2-1)
            entry["result_not_this_mission"] = True
        entry["ended_mono"] = time.monotonic()
        reason_ = (entry["result"] or {}).get("reason") or ""
        if (args.manual_recovery and "manual_recovery" in p.report and reason_.startswith("prepare:")
                and "invalid_start" in reason_ and backoffs < args.manual_backoffs):
            # Plan D14 U20 c: as an operator would -- the preparation could not start from here (D4b's check at the
            # submit); back off a little more by hand and ask the same mission again.
            backoffs += 1
            advice_ = (entry["result"] or {}).get("advice") or {}
            if advice_ and advice_.get("backoff_m") is None:
                # Plan D13 item 6: the vehicle found no start straight back (blocked, or not within its limit).
                p.report.setdefault("manual_backoffs", []).append({"after": reason_[:80], "advice": advice_,
                                                                   "mono": time.monotonic(), "gave_up": True})
                leg_index += 1
                continue
            await enter_teleop(p)
            if advice_:
                # the middle of the passing interval, measured by the reported rear-axle speed (the map pose moves
                # with SLAM corrections; plan D13 item 6)
                moved_ = await back_off_by(p, float(advice_["backoff_m"]), float(advice_["backoff_max_m"]), max_s=60.0)
                target_ = moved_["target_m"]
            else:
                stop_by = time.monotonic() + args.manual_extra_s
                while time.monotonic() < stop_by:
                    await p.teleop(-0.5, 0.0, True)
                    await asyncio.sleep(0.05)
                target_ = moved_ = None
            stand_by = time.monotonic() + 15.0
            while time.monotonic() < stand_by and not (p.state and p.standing(p.state)):
                await p.teleop(0.0, 0.0, True)
                await asyncio.sleep(0.05)
            await p.request("mode_request", to="PAUSED")
            p.report.setdefault("manual_backoffs", []).append({"after": reason_[:80], "mono": time.monotonic(),
                                                               "advice": advice_ or None, "target_m": target_,
                                                               "moved_m": moved_})
            continue
        leg_index += 1
    p.report["missions_message"] = p.missions
    if args.reconnect_at_end and p.closed_code is None:
        # Plan U2: a page that connects after the missions gets the results book and the last result
        await p.ws.close()
        await asyncio.sleep(0.5)
        p.missions, p.state = None, None
        await p.open()
        await p.until(lambda s: True, 10, label="first_state_after_reconnect")
        await p.until(lambda s: p.missions is not None, 10, label="missions_after_reconnect")
        lease_ = await p.request("lease_request")
        p.lease_id = (lease_ or {}).get("lease_id")
        p.report["after_reconnect"] = {"session_epoch": p.epoch, "missions_message": p.missions,
                                       "last_result": (p.state or {}).get("last_result")}
    if args.end_session and p.closed_code is None:
        # Persistent console plan D7: the session ends on request (the runner exits 0), not at a sim time limit.
        await p.until(p.standing, 30, label="standing_before_end")
        p.report["session_end"] = await p.request_until_standing("session_end")
        end = time.monotonic() + 120
        while p.closed_code is None and time.monotonic() < end:
            await asyncio.sleep(0.1)
    p.report["close_code"] = p.closed_code


async def queue_scenario(p: Probe, args):
    """CT-1 PR 4 (docs/plans/2026-10-22-continuous-transport-ct1.md): the --cycle legs run by the console's own mission
    queue (tools/mission_queue.py, a port of console/web/js/queue.js) -- fed every state, result, welcome and close as
    the page feeds it, asked every 250 ms whether to submit, the operator's resume given once at the start. Any stop
    of the queue is final here: nothing more is submitted, a mission already out runs to its end and is recorded."""
    import mission_queue

    await p.open()
    await p.open_media()
    await p.until(lambda s: True, 10, label="first_state")
    lease = await p.request("lease_request")
    p.lease_id = lease.get("lease_id") if lease else None
    p.report["session"] = {"session_epoch": p.epoch, "lease_id": p.lease_id}
    await p.until(lambda s: p.targets is not None, 10, label="targets")
    q = p.queue = mission_queue.MissionQueue()
    p.queue_has_lease = p.lease_id is not None
    q.on_welcome(p.epoch)
    asking = None  # (key, task) of the submit out
    ids = {"n": 0}

    def new_id():
        ids["n"] += 1
        return f"q{ids['n']}"

    for leg in [leg for leg in args.cycle.split(",") if leg]:
        pallet_id, destination_id = leg.split(":")
        q.add(mission_queue.new_item(pallet_id, destination_id, new_id()))
    p.report["missions_run"] = []
    q.resume(time.monotonic() * 1000.0)
    closed_seen, deadline = False, time.monotonic() + args.mission_timeout_s * max(1, len(q.items) + 1)
    while time.monotonic() < deadline:
        now_ms = time.monotonic() * 1000.0
        if p.closed_code is not None and not closed_seen:
            closed_seen = True
            q.on_disconnect()
        q.tick(now_ms)
        for entry in p.report["missions_run"]:  # each submission's result, as the queue applied it
            if "result" not in entry and (entry.get("submit") or {}).get("t") == "ack":
                mid_ = entry["submit"].get("mission_id")
                got_ = next((f for f in q.finished if f["mission_id"] == mid_), None)
                if got_ is not None:
                    entry["result"] = {"mission_id": mid_, "outcome": got_["outcome"], "reason": got_["reason"]}
                    entry["ended_mono"] = time.monotonic()
        if q.run != "running" and q.current is None and asking is None:
            break  # stopped, nothing out: the queue's end here
        if q.run != "running" and q.phase == "in_mission" and closed_seen:
            break  # its result can no longer come
        if asking is not None and asking[1].done():
            key_, task_, entry_ = asking
            entry_["submit"] = task_.result()
            if entry_["submit"] is not None:
                q.on_answer(key_, entry_["submit"], time.monotonic() * 1000.0)  # a late one is ignored by the queue
            asking = None
        if asking is not None and q.run != "running" and q.current is None:
            asking[1].cancel()  # the queue stopped while it was out (it forgot the request, D6): no answer needed
            asking[2]["submit"] = {"t": "unanswered"}
            asking = None
        ctx = {"connected": p.closed_code is None, "welcomed": p.closed_code is None, "has_lease": p.queue_has_lease,
               "state": p.state, "targets": p.targets}
        n = q.next(ctx, now_ms, new_id)
        if n:
            s_ = n["submit"]
            key = {"epoch": q.epoch, "item_id": s_["item_id"], "request_id": s_["request_id"]}
            entry = {"pallet_id": s_["pallet_id"], "destination_id": s_["destination_id"], "item_id": s_["item_id"],
                     "submitted_mono": time.monotonic()}
            p.report["missions_run"].append(entry)
            task = asyncio.ensure_future(p.request_resent(
                "mission_submit", s_["request_id"], pallet_id=s_["pallet_id"], destination_id=s_["destination_id"],
                targets_revision=s_["targets_revision"]))
            asking = (key, task, entry)
        await p.echo()
        await asyncio.sleep(0.25)
    if asking is not None:
        asking[1].cancel()
    for entry in p.report["missions_run"]:
        if "result" not in entry:
            entry["result_not_this_mission"] = True  # the queue never applied a result for it
    p.report["queue"] = {"stop_reason": q.stop_reason, "stops": q.stops, "counts": q.counts,
                         "items_left": [f"{it['pallet_id']}:{it['destination_id']}" for it in q.items],
                         "uncertain": [it["item_id"] for it in q.items if it.get("uncertain")]}
    p.report["missions_message"] = p.missions
    if args.end_session and p.closed_code is None:
        await p.until(p.standing, 30, label="standing_before_end")
        p.report["session_end"] = await p.request_until_standing("session_end")
        end = time.monotonic() + 120
        while p.closed_code is None and time.monotonic() < end:
            await asyncio.sleep(0.1)
    p.report["close_code"] = p.closed_code


async def restart_scenario(p: Probe, args):
    """Persistent console plan U15/U24: the runner killed (SIGKILL) while the console holds the lease and stands;
    main() then follows the supervisor's next session, resends the old session's request there (its epoch, token
    and lease: refused stale_epoch) and runs the cycle."""
    import os

    await p.open()
    await p.until(lambda s: True, 10, label="first_state")
    lease = await p.request("lease_request")
    p.lease_id = lease.get("lease_id") if lease else None
    await p.until(lambda s: p.targets is not None, 10, label="targets")
    await p.until(p.standing, 30, label="standing_before_kill")
    p.report["session"] = {"session_epoch": p.epoch, "lease_id": p.lease_id, "control_gen": p.control_gen,
                           "request_token": p.request_token, "request_seq": p.request_seq}
    # The port file read again at the kill: the connected session's, running, its pid a runner (Codex probe review
    # P1: the snapshot from before the connection could name an earlier epoch's pid, reused since).
    try:
        info_ = json.loads(args.port_file.read_text())
    except (OSError, ValueError) as exc:
        p.event("kill_refused", why=f"port_file: {exc!r}")
        return
    pid = info_.get("pid")
    if not pid or info_.get("state") != "running" or info_.get("session_epoch") != p.epoch:
        p.event("kill_refused", why="port file not this session", port_file=info_, session_epoch=p.epoch)
        return
    try:
        cmdline_ = Path(f"/proc/{int(pid)}/cmdline").read_bytes()
    except OSError as exc:
        p.event("kill_refused", why=f"no process {pid}: {exc!r}")
        return
    if b"run_transport.py" not in cmdline_:
        p.event("kill_refused", why="not a runner", pid=pid)
        return
    p.report["killed"] = {"pid": pid, "mono": time.monotonic()}
    os.kill(int(pid), signal.SIGKILL)
    await p.until(lambda s: p.closed_code is not None, 30, label="closed_after_kill")
    p.report["killed"]["close_code"] = p.closed_code


async def old_session_requests(p: Probe, old: dict) -> dict:
    """Plan U24: the old session's next request as its console would resend it (its epoch, token, seq and lease) on
    the new session's connection -- a mode change and a submit; both must be refused stale_epoch."""
    await p.open()
    await p.until(lambda s: True, 10, label="first_state_new_session")
    # The epoch, lease and generation go as the request's own fields (the reader keeps updating the shared
    # control_gen); the token and seq are the probe's, swapped and restored whatever happens (Codex probe review P3).
    old_fields = {"session_epoch": old["session_epoch"], "lease_id": old["lease_id"], "control_gen": old["control_gen"]}
    saved = (p.request_token, p.request_seq)
    out = {"new_epoch": p.epoch}
    try:
        p.request_token, p.request_seq = old["request_token"], old["request_seq"]
        out["mode_request"] = await p._request("mode_request", to="PAUSED", **old_fields)
        out["mission_submit"] = await p._request("mission_submit", pallet_id="P-1", destination_id="D-1",
                                                 targets_revision=(p.targets or {}).get("targets_revision"),
                                                 **old_fields)
    finally:
        p.request_token, p.request_seq = saved
    return out


async def reset_scenario(p: Probe, args):
    """Persistent console plan U12: a scene damaged under a mission (the runner's --live-test-push-pallet moves the
    pallet at a phase) is latched; automatic work is refused, manual control is not; a scene reset ends the session --
    main() then follows the supervisor's next session and runs the cycle there."""
    await p.open()
    await p.until(lambda s: True, 10, label="first_state")
    lease = await p.request("lease_request")
    p.lease_id = lease.get("lease_id") if lease else None
    p.report["session"] = {"session_epoch": p.epoch, "lease_id": p.lease_id}
    await p.until(lambda s: p.targets is not None, 10, label="targets")
    await p.until(p.standing, 30, label="standing_at_start")
    await asyncio.sleep(1.0)
    p.report["first_submit"] = await p.request("mission_submit", pallet_id="P-1", destination_id="D-1",
                                                targets_revision=p.targets["targets_revision"])
    await p.until(lambda s: ((s.get("scene") or {}).get("damaged") is not None), args.mission_timeout_s,
                  label="scene_damaged")
    await p.until(lambda s: ((s.get("last_result") or {}).get("outcome") in ("failed", "cancelled",
                                                                               "recovery_required", "done")),
                  120, label="first_end")
    p.report["first_result"] = (p.state or {}).get("last_result")
    p.report["scene"] = (p.state or {}).get("scene")
    await p.until(p.standing, 60, label="standing_after_damage")
    await asyncio.sleep(1.0)
    p.report["refused_submit"] = await p.request("mission_submit", pallet_id="P-1", destination_id="D-1",
                                                  targets_revision=p.targets["targets_revision"])
    # recorded as it is: allowed when the forks' relation to the pallet is known; a push that leaves it unknown is
    # refused recovery:unknown (plan U12 as amended, Codex injection review P2)
    p.report["teleop_allowed_after_damage"] = (p.state or {}).get("teleop_allowed")
    p.report["teleop_after_damage"] = await p.request("mode_request", to="TELEOP")
    await p.request("mode_request", to="PAUSED")
    await p.until(p.standing, 30, label="standing_before_reset")
    p.report["reset"] = await p.request_until_standing("sim_reset")
    await asyncio.sleep(3.0)


async def tour_scenario(p: Probe, args):
    """SLAM localization plan L2 v2.5: the survey epoch's tour. The path tools/survey_tour.py planned is driven by
    teleop with tools/survey_tour.py's TourDriver (the runner's path tracker on the state's map pose and measured
    speed, a bounded cusp release, the stall, staleness and TELEOP checks -- it fails the tour, never skips ahead).
    With --end-session the session ends either way, so the job does not hold the GPU to its time limit (ws1 job 6807
    did); the bundle tool reads the tour's end from this report and fails a survey whose tour did not arrive."""
    import numpy as np

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from survey_tour import TourDriver, replan_tour_bounded

    data = np.load(args.tour_path)
    plan_input = json.loads(str(data["plan_input"])) if "plan_input" in data.files else None
    # a tour planned with a coverage margin carries its judged cells: replans keep them covered (gyro plan 4.5 F3)
    judged = np.asarray(data["judged"]) if "judged" in data.files else None
    radius = float(data["coverage_radius_m"]) if "coverage_radius_m" in data.files else None
    tour = p.report["tour"] = {"path": str(args.tour_path), "samples": int(len(data["poses"])),
                               "coverage_keeping": judged is not None, "coverage_radius_m": radius}
    await p.open()
    await p.until(lambda s: True, 10, label="first_state")
    lease = await p.request("lease_request")
    p.lease_id = lease.get("lease_id") if lease else None
    p.report["session"] = {"session_epoch": p.epoch, "lease_id": p.lease_id}
    await p.until(p.standing, 30, label="standing_at_start")
    limits = (p.state or {}).get("limits") or {}
    driver = TourDriver(data["poses"], data["directions"], data["curvatures_inv_m"],
                        forward_mps=float(limits.get("forward_mps", 0.5)),
                        reverse_mps=float(limits.get("reverse_mps", 0.25)),
                        kappa=float(limits.get("curvature_inv_m", 0.36)), cruise_mps=args.tour_cruise_mps,
                        stall_s=args.tour_stall_s, pacing_wait_s=args.tour_pacing_wait_s,
                        waypoints=data["tour"].tolist() if plan_input is not None else None)
    tour["events"] = driver.events
    await p.request("mode_request", to="PAUSED")
    await p.until(p.standing, 20, label="standing_before_teleop")
    entered = None
    for _ in range(120):
        entered = await p.request("mode_request", to="TELEOP")
        if entered is None or (entered.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed")
                               and entered.get("detail") != "pacing"):
            break
        await asyncio.sleep(0.5)
    tour["teleop"] = entered
    end = None
    if not entered or entered.get("t") != "ack":
        end = "teleop_refused"
    else:
        arm = await p.request("arm")
        p.arm_gen = (arm or {}).get("arm_gen", 0)
        driver.start(time.monotonic())  # a state from before this instant is not a departure (Codex v2.5 P2-4)
        command = None

        async def rearm_if_asked():
            # disarmed after departure (an expired deadline: gyro plan 4.5 F1), the stop confirmed: arm again; the
            # generation every later command carries is the state's once armed (an ack lost to a timeout still
            # armed it), an older one is refused stale_arm
            if driver.rearm_request:
                arm = await p.request("arm")
                if arm is not None and arm.get("t") == "ack":
                    p.arm_gen = arm.get("arm_gen", p.arm_gen)
                driver.rearm_sent(time.monotonic(), arm)
            if driver.arm_gen is not None:
                p.arm_gen, driver.arm_gen = driver.arm_gen, None
            st_ = p.state or {}
            if st_.get("armed") and st_.get("arm_gen") is not None:
                p.arm_gen = st_["arm_gen"]  # the server's generation while armed (a first arm's ack lost too)

        while end is None:
            await asyncio.sleep(0.05)
            if p.closed_code is not None:
                end = "closed"
                break
            new, end = driver.step(time.monotonic(), p.state)
            command = new if new is not None else command
            await rearm_if_asked()
            if command is not None:
                await p.teleop(*command, True)
            if end is None and driver.replan_request is not None:
                # plan off the event loop in a child process killed past its hard limit (the echo task keeps
                # answering ticks), holding zero and still watching the states meanwhile -- staleness, TELEOP left,
                # a disarm and its rearm (Codex F1-F3 review P2)
                start, left = driver.replan_request
                driven, refine_log, asked = list(driver.driven), [], time.monotonic()
                left_in = list(left)
                job = asyncio.get_running_loop().run_in_executor(
                    None, lambda: replan_tour_bounded(plan_input, left, start, driven, judged,
                                                      radius if radius is not None else 0.0, log=refine_log))
                while not job.done() and end is None:
                    if p.closed_code is not None:
                        end = "closed"
                        break
                    _, end = driver.step(time.monotonic(), p.state)
                    if end is None:
                        await rearm_if_asked()
                    await p.teleop(0.0, 0.0, True)
                    await asyncio.sleep(0.05)
                if end is not None:
                    # a failure while planning: the plan (finished or killed at its limit) is not adopted
                    driver.events.append({"wall": time.monotonic(), "event": "replan_abandoned", "end": end,
                                          "start": list(start), "waypoints_in": [list(w) for w in left_in],
                                          "elapsed_s": time.monotonic() - asked})
                    break
                try:
                    left, plan, coverage, error = job.result()
                except Exception as exc:  # noqa: BLE001 -- a planner error is a failed replan, recorded
                    plan, coverage, error = None, None, repr(exc)
                if error is not None:
                    driver.events.append({"wall": time.monotonic(), "event": "replan_error", "error": error})
                for line in refine_log:
                    driver.events.append({"wall": time.monotonic(), "event": "refine", **line})
                # what the replan was given and gave, whole (gyro plan 4.6, Codex P3): its coverage prediction redone
                # from these alone -- every driven pose with its state id and correction version, the full new path
                driver.events.append({
                    "wall": time.monotonic(), "event": "replan_io", "start": list(start),
                    "waypoints_in": [list(w) for w in left_in], "waypoints_out": [list(w) for w in left],
                    "driven": [list(d) for d in driven],
                    "path": None if plan is None or not plan.success else np.asarray(plan.poses).tolist(),
                    "elapsed_s": time.monotonic() - asked, "error": error,
                    "coverage": None if coverage is None else {k: coverage[k] for k in (
                        "ok", "uncovered", "radius_m", "farthest_m", "farthest_cell")}})
                driver.adopt(time.monotonic(), plan, left, coverage)
    tour["end"], tour["progress_m"] = end, float(driver.best)
    stop_by = time.monotonic() + 15.0
    while time.monotonic() < stop_by and not (p.state and p.standing(p.state)) and p.closed_code is None:
        await p.teleop(0.0, 0.0, True)
        await asyncio.sleep(0.05)
    await p.request("mode_request", to="PAUSED")
    if args.end_session and p.closed_code is None:
        await p.until(p.standing, 30, label="standing_before_end")
        p.report["session_end"] = await p.request_until_standing("session_end")
        stop = time.monotonic() + 120
        while p.closed_code is None and time.monotonic() < stop:
            await asyncio.sleep(0.1)
    p.report["close_code"] = p.closed_code


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port-file", type=Path, required=True)
    ap.add_argument("--report", type=Path, required=True)
    ap.add_argument("--scenario", choices=("main", "slow", "marker", "approach", "planwait", "cycle", "queue", "lift", "reset",
                                           "restart", "tour"),
                    default="main")
    ap.add_argument("--manual-recovery", action="store_true",
                    help="cycle scenario (plan D14 U20 c): a mission ending recovery_required is recovered by hand "
                    "before the next submit")
    ap.add_argument("--manual-extra-s", type=float, default=2.0, help="manual recovery: reverse this long after it")
    ap.add_argument("--manual-backoffs", type=int, default=3,
                    help="manual recovery: how often to back off further and ask again when the preparation cannot "
                    "start from where the recovery ended (invalid_start)")
    ap.add_argument("--manual-stroke-s", type=float, default=1.5,
                    help="manual recovery: each straight reverse stroke before standing for the verdict (wall s)")
    ap.add_argument("--lift-up-s", type=float, default=6.0, help="lift scenario: hold the lift up this long (wall s)")
    ap.add_argument("--quiet-s", type=float, default=8.0, help="lift scenario: the client quiet / runner stopped (s)")
    ap.add_argument("--no-sigstop", action="store_true",
                    help="lift scenario: skip the runner SIGSTOP part (until main's SLAM link stall fix)")
    ap.add_argument("--cycle", default="P-1:D-1,P-1:D-0,P-1:D-1", help="cycle scenario: pallet:destination,...")
    ap.add_argument("--pre-teleop-s", type=float, default=0.0, help="cycle scenario: drive by hand first (wall s)")
    ap.add_argument("--end-session", action="store_true", help="cycle scenario: end the session at the end")
    ap.add_argument("--reconnect-at-end", action="store_true",
                    help="cycle scenario: reconnect after the missions and keep the results book it gets (U2)")
    ap.add_argument("--cancel-pause-s", type=float, default=0.0,
                    help="cycle scenario: with --cancel-at, pause the mission this long (wall s) before cancelling (U27)")
    ap.add_argument("--cancel-at", default="", help="cycle scenario: cancel the first mission in PHASE[:SECONDS in it] "
                    "or PHASE@METRES (at most METRES left of its leg) or transport@docked (on the docked straight) "
                    "(plan U7/U26/U27)")
    ap.add_argument("--stop-wait-at", default="", help="cycle scenario: STOP the first mission in PHASE[:SECONDS in "
                    "it], stand --stop-wait-sim-s, rearm and cancel it (plan U30)")
    ap.add_argument("--stop-wait-sim-s", type=float, default=1500.0, help="cycle scenario: the STOP's stand (sim s)")
    ap.add_argument("--stop-wait-wall-s", type=float, default=7200.0,
                    help="cycle scenario: the STOP's stand at most (wall s), when the frames' time does not get there")
    ap.add_argument("--idle-s", type=float, default=0.0, help="cycle scenario: stay connected this long first (wall s)")
    ap.add_argument("--pre-teleop-speed", type=float, default=0.5)
    ap.add_argument("--pre-teleop-curvature", type=float, default=0.3)
    ap.add_argument("--wait-offset-s", type=float, default=4.0,
                    help="planwait scenario: standing this long in AUTO observe is taken as the plan wait")
    ap.add_argument("--hold-s", type=float, default=5.0, help="planwait scenario: standing watched after pause/STOP")
    ap.add_argument("--after-cancel-s", type=float, default=20.0,
                    help="planwait scenario: standing watched after the cancel (the dropped plan would have landed)")
    ap.add_argument("--approach-s", type=float, default=600.0, help="approach scenario: wall seconds per leg at most")
    ap.add_argument("--held-s", type=float, default=15.0, help="approach scenario: standing this long ends a leg")
    ap.add_argument("--approach-weave-s", type=float, default=0.0,
                    help="approach scenario: weave first, flipping the curvature ratio +-0.8 every this many wall s")
    ap.add_argument("--approach-weave-n", type=int, default=6, help="approach scenario: weave periods")
    ap.add_argument("--approach-curvature", type=float, default=-0.3,
                    help="approach scenario: the first leg's curvature ratio (-0.15 passed too close to the pallet the "
                    "scan plane does not see, 2026-10-17)")
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
    ap.add_argument("--tour-path", type=Path, default=None, help="tour scenario: tools/survey_tour.py's path (.npz)")
    ap.add_argument("--tour-cruise-mps", type=float, default=0.30)
    ap.add_argument("--tour-stall-s", type=float, default=60.0, help="tour scenario: no progress this long (wall s)")
    ap.add_argument("--tour-pacing-wait-s", type=float, default=900.0,
                    help="tour scenario: no progress this long while teleop is forbidden for pacing (wall s)")
    args = ap.parse_args()
    # The job's timeout ends a stuck probe with SIGTERM: leave through the finally below, so the report is written.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(124))
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
        scenario = {"main": main_scenario, "slow": slow_scenario, "marker": marker_scenario,
                    "approach": approach_scenario, "planwait": planwait_scenario, "cycle": cycle_scenario,
                    "queue": queue_scenario,
                    "lift": lift_scenario, "reset": reset_scenario, "restart": restart_scenario,
                    "tour": tour_scenario}[args.scenario]
        asyncio.run(scenario(p, args))
        if ((args.scenario == "reset" and (report.get("reset") or {}).get("t") == "ack")
                or (args.scenario == "restart" and "killed" in report)):
            # U12, U15: the supervisor's next session (a new epoch on the same port), the cycle there
            # the successor is newer than the session left behind -- for a restart the one killed, which may be later
            # than the snapshot taken at start (Codex probe review round 2 P2-1)
            old = ((report.get("session") or {}).get("session_epoch") if args.scenario == "restart"
                   else info.get("session_epoch"))
            info2, ready = {}, False
            end = time.monotonic() + args.wait_port_s
            while time.monotonic() < end:
                info2 = json.loads(args.port_file.read_text()) if args.port_file.exists() else {}
                if info2.get("state") == "running" and info2.get("session_epoch") not in (None, old):
                    ready = True
                    break
                time.sleep(2.0)
            # the next session must be up, new, and on the same port (the supervisor's) -- else nothing more is run
            ready = ready and info2.get("port") == info.get("port")
            report["after_reset"] = {"port_file": info2, "ready": ready}
            if ready:
                p2 = Probe(info2["port"], report["after_reset"])
                try:
                    if args.scenario == "restart":
                        async def resend_then_cycle():
                            report["after_reset"]["old_requests"] = await old_session_requests(p2, report["session"])
                            await cycle_scenario(p2, args)

                        asyncio.run(resend_then_cycle())
                    else:
                        asyncio.run(cycle_scenario(p2, args))
                finally:
                    report["after_reset"]["states_n"] = len(p2.states)
                    report["after_reset"]["states"] = p2.states  # Codex probe review P3: the next session's own
                    report["after_reset"]["frames"] = p2.frames
    finally:
        report["frames"] = p.frames
        report["states"] = p.states
        args.report.write_text(json.dumps(report, default=str))


if __name__ == "__main__":
    main()
