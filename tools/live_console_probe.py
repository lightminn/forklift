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
import os
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
        self.queue_asking = None  # {key, entry} of the submit out: its answer goes to the queue as it is received
        self.queue_forgotten: set = set()  # request ids the queue dropped (D6 forget): never sent again
        self.queue_picking = None  # CT-2: {key} of the pick_request whose pick is awaited (cleared by the pick)
        self.queue_pick_answer = None  # CT-2: {key} of the pick_request whose ack / reject is awaited (either order)
        self.capabilities: dict = {}

    def queue_apply(self, out) -> None:
        """queuewire apply(): a stop that forgets the submit out ends its resending at once (Codex PR 4 r3 P2-1)."""
        if out and out.get("forget") is not None:
            self.queue_forgotten.add(out["forget"])
            if self.queue_asking is not None and self.queue_asking["key"]["request_id"] == out["forget"]:
                self.queue_asking["entry"]["submit"] = {"t": "forgotten"}
                self.queue_asking = None

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
        self.capabilities = welcome.get("capabilities") or {}
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
                        out_ = self.queue.on_state(msg, now_ms_)
                        if had_ and not self.queue_has_lease:
                            self.queue_apply(self.queue.on_lease_lost())
                        if msg.get("last_result"):
                            self.queue.on_result(msg["last_result"], now_ms_)
                        self.queue_apply(out_)
                    self.states.append({"mono": time.monotonic(), "mode": msg.get("mode"),
                                        "stop_kind": msg.get("stop_kind"), "armed": msg.get("armed"),
                                        "applied": msg.get("applied"), "speed": msg.get("rear_axle_speed_mps"),
                                        "mission": msg.get("mission"), "pacing": msg.get("pacing"),
                                        "latch": msg.get("stop_latch"), "reject": msg.get("last_teleop_reject"),
                                        "teleop_forbidden": msg.get("teleop_forbidden_reason"),
                                        "permit": msg.get("permit"), "last_result": msg.get("last_result")})
                elif t == "targets":
                    self.targets = msg
                elif t == "pick":
                    picking_ = self.queue_picking
                    self.event("pick", request_id=msg.get("request_id"), reason=msg.get("reason"), pick=msg.get("pick"),
                               plan=msg.get("plan"), targets_revision=msg.get("targets_revision"))
                    if self.queue is not None and picking_ is not None and \
                            msg.get("request_id") == picking_["key"]["request_id"]:
                        held_ = (self.targets or {}).get("targets_revision")
                        self.queue_apply(self.queue.on_pick(picking_["key"], msg, time.monotonic() * 1000.0, held_))
                        # kept while the queue still waits on this request (a none_remaining before its ack may be
                        # followed by a re-computed ok, contract 5.3; Codex CT-2 impl r2 P3); its ack is queue_pick_answer
                        if not (self.queue.phase == "picking"
                                and (self.queue.pick or {}).get("request_id") == picking_["key"]["request_id"]):
                            self.queue_picking = None
                elif t == "missions":
                    self.missions = msg
                    if self.queue is not None:  # queuewire onResults: oldest first
                        for r_ in sorted(msg.get("results") or [], key=lambda r: r.get("seq") or 0):
                            self.queue.on_result(r_, time.monotonic() * 1000.0)
                elif t in ("ack", "reject"):
                    self.replies[msg.get("request_id")] = (time.monotonic(), msg)
                    answer_ = self.queue_pick_answer
                    if self.queue is not None and answer_ is not None and \
                            msg.get("request_id") == answer_["key"]["request_id"]:
                        self.queue_pick_answer = None
                        if t == "reject":
                            self.queue_picking = None  # no pick follows a refusal
                        self.queue_apply(self.queue.on_answer(answer_["key"], msg, time.monotonic() * 1000.0))
                    asking_ = self.queue_asking
                    if self.queue is not None and asking_ is not None and \
                            msg.get("request_id") == asking_["key"]["request_id"]:
                        # the answer into the queue in the order it came, among the states (Codex PR 4 r3 P2-2)
                        asking_["entry"]["submit"] = msg
                        self.queue_asking = None
                        self.queue_apply(self.queue.on_answer(asking_["key"], msg, time.monotonic() * 1000.0))
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
                           detail=reply.get("detail"), rtt_s=at - sent, sent_mono=sent, reply_mono=at)
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
        while self.closed_code is None and request_id not in self.queue_forgotten and request_id not in self.replies:
            await self.ws.send(raw)
            end = time.monotonic() + retry_s
            while time.monotonic() < end and request_id not in self.queue_forgotten:
                if request_id in self.replies:
                    at, reply = self.replies[request_id]
                    self.event("reply", kind=kind, request_id=request_id, t=reply["t"], reason=reply.get("reason"),
                               detail=reply.get("detail"), rtt_s=at - sent, sent_mono=sent, reply_mono=at)
                    if reply.get("control_gen") is not None:
                        self.control_gen = reply["control_gen"]
                    return reply
                await asyncio.sleep(0.01)
        # answered at a resend's boundary (Codex PR 4 r4 P2: never sent again once answered), or forgotten / closed
        if request_id not in self.replies:
            return None
        at, reply = self.replies[request_id]
        self.event("reply", kind=kind, request_id=request_id, t=reply["t"], reason=reply.get("reason"),
                   detail=reply.get("detail"), rtt_s=at - sent, sent_mono=sent, reply_mono=at)  # (Codex r5 P3)
        return reply

    async def request_until_standing(self, kind, tries=20, **fields):
        """A request the vehicle refuses until its own detector confirms standing (not_stopped): asked again. For
        session_end alone a stale_control is asked again too, with the generation the next state carries (_read):
        CT-1 PR 4 q2 had three session_ends refused so and the runner's result.json never written. Any other
        request returns it -- a rearm or resume made before a later STOP must not go through after it (the
        arbiter's boundary; Codex PR 4 doc review P2)."""
        retry = ("not_stopped", "no_feedback:rear_axle_speed") + (("stale_control",) if kind == "session_end" else ())
        reply = None
        for _ in range(tries):
            reply = await self.request(kind, **fields)
            if reply is None or reply.get("reason") not in retry:
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
    if getattr(args, "end_session", False):
        # validation v2 H1: end the session through its own path, so the epoch writes result.json and the tick
        # trace (a launcher TERM lost both in batch 16782's lifecycle 1); a mission still active (the final resume
        # refused, a STOP again) would make session_end refuse it (mission_active): STOP, cancel, its end, then end
        m_ = (p.state or {}).get("mission") or {}
        if m_.get("mission_id") and m_.get("status") not in FINAL_MISSION:
            p.report["end_active"] = await end_active_mission(p, m_.get("mission_id"))
        await _end_session(p)
        return
    end = time.monotonic() + 60
    while p.closed_code is None and time.monotonic() < end:
        await asyncio.sleep(0.05)
    p.report["close_code"] = p.closed_code


FREEZE_FRAME_SLACK_S = 0.15  # the live camera runs at 10 Hz: its last frame before the freeze is at most 0.1 s early


async def _end_if_asked(p, args) -> None:
    """--end-session for the scenarios that leave the session open (marker, approach): close it through its own path
    so the epoch writes its end summary, result.json, the tick trace and the spool -- a launcher TERM lost all of them
    (realtime plan 5.17, ws1 job 16866: the eight hall sessions; validation v2 H1 likewise)."""
    if getattr(args, "end_session", False):
        p.report["end_requested_mono"] = time.monotonic()  # after every measured step (the analysis checks it)
        await _end_session(p)


async def marker_scenario(p: Probe, args):
    await _marker_steps(p, args)
    await _end_if_asked(p, args)


async def approach_scenario(p: Probe, args):
    await _approach_steps(p, args)
    await _end_if_asked(p, args)


async def _marker_steps(p: Probe, args):
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
        in_freeze = sim_now() >= args.freeze_from - FREEZE_FRAME_SLACK_S
        if not st.get("armed") and not in_freeze:
            # Disarmed before the freeze (a stall's stale_video, an expiry): armed again, so the freeze meets live
            # commands (R1 results: three sessions entered it already disarmed and proved nothing).
            p.event("rearmed_before_freeze", reject=st.get("last_teleop_reject"), last_frame_sim=sim_now())
            arm = await p.request("arm")
            p.arm_gen = (arm or {}).get("arm_gen", p.arm_gen)
        elif not st.get("armed"):
            # Disarmed with the frames stopped at the freeze: the steps end here, whatever the state's last reject
            # shows now (it is soon overwritten by not_armed / stale_control: R1 results, r1c_18 waited 600 s). The
            # analysis judges the reason from the runner's own disarm event, not from this sample.
            p.event("disarmed_in_freeze", reject=st.get("last_teleop_reject"), last_frame_sim=sim_now())
            break
        await asyncio.sleep(0.05)


async def _approach_steps(p: Probe, args):
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
            if not st.get("armed"):
                # Disarmed mid-leg (a command expiry after a long step; R1 results: every arm's wall leg ended so and
                # the later legs never moved): armed again and the leg goes on; it is not a hold.
                p.event("rearmed_in_leg", leg=name, reject=st.get("last_teleop_reject"))
                arm = await p.request("arm")
                p.arm_gen = (arm or {}).get("arm_gen", p.arm_gen)
                still_since = None
                await asyncio.sleep(0.05)
                continue
            v = abs(st.get("rear_axle_speed_mps") or 0.0)
            moved = moved or v > 0.05
            applied_ = abs((st.get("applied") or {}).get("speed_mps") or 0.0)
            permit_limit = (st.get("permit") or {}).get("limit_mps")
            # A hold is the permission's: armed, the output zero, the permission's limit zero, the truck still
            # (R1 results: an expiry's zero output was counted as a hold short of the wall).
            if moved and applied_ == 0.0 and v < 0.03 and permit_limit is not None and permit_limit <= 0.0:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > args.held_s and name not in ("reverse_swing", "weave"):
                    break  # held short of the obstacle with the command still asking to go
            else:
                still_since = None
            await asyncio.sleep(0.05)
        st = p.state or {}
        final_limit = (st.get("permit") or {}).get("limit_mps")
        # held: the permission's hold, kept at least held_s and still so at the leg's end (armed, its limit zero;
        # Codex tools review P2-3: a disarm in the last wait left held=true with armed=false)
        final_v = abs(st.get("rear_axle_speed_mps") or 0.0)
        final_applied = abs((st.get("applied") or {}).get("speed_mps") or 0.0)
        held = (still_since is not None and time.monotonic() - still_since >= args.held_s and bool(st.get("armed"))
                and final_limit is not None and final_limit <= 0.0 and final_applied == 0.0 and final_v < 0.03)
        p.report["approach"].append({"leg": name, "wall_s": time.monotonic() - start, "moved": moved,
                                     "held": held, "armed": st.get("armed"),
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


class StallWatch:
    """--stall-recover-s (docs/plans/2026-11-08-live-riib-defaults-validation.md 3.2, the low-clearance guard's target
    sessions): this mission running in AUTO with the truck standing -- reported rear-axle speed under 0.005 m/s, its
    published pose within 0.01 m of one spot -- over fresh states for ``seconds`` wall s in a row. A cap of 0 from the
    guard stands the truck with no result of its own; this is what an operator watching the page would see, and it
    can also be a long legitimate stand (a heuristic: the cause is attributed afterwards from the run's records).
    Only a new state counts (a state object or sequence seen before adds nothing, Codex probe review P2); no new state
    for ``STALE_S`` starts the count again, as anything else does (another mission, a pause, STOPPED, a move);
    ``reset`` for a stop the caller handles."""

    SPEED_MPS, SPOT_M, STALE_S = 0.005, 0.01, 1.0

    def __init__(self, seconds: float, mission_id, clock=None):
        self.seconds, self.mission_id = float(seconds), mission_id
        self.clock = clock if clock is not None else (lambda: time.monotonic())
        self.since = self.spot = self.seen = self.fresh_at = None

    def reset(self) -> None:
        self.since = self.spot = None

    def __call__(self, s) -> bool:
        now = self.clock()
        stamp = None if s is None else (s.get("state_seq"), id(s))
        if stamp is None or stamp == self.seen:
            if self.fresh_at is None or now - self.fresh_at > self.STALE_S:
                self.reset()
            return False
        if self.fresh_at is not None and now - self.fresh_at > self.STALE_S:
            self.reset()  # a silence before this state: its stand is not known to have gone on
        self.seen, self.fresh_at = stamp, now
        m = s.get("mission") or {}
        pose, v = s.get("pose"), s.get("rear_axle_speed_mps")
        if not (m.get("mission_id") == self.mission_id and m.get("status") == "running" and s.get("mode") == "AUTO"
                and v is not None and abs(v) < self.SPEED_MPS and isinstance(pose, dict)
                and pose.get("x_m") is not None and pose.get("y_m") is not None):
            self.reset()
            return False
        xy = (float(pose["x_m"]), float(pose["y_m"]))
        if self.spot is None or math.hypot(xy[0] - self.spot[0], xy[1] - self.spot[1]) >= self.SPOT_M:
            self.since, self.spot = now, xy
            return False
        return now - self.since >= self.seconds


# stall_recovery outcomes after which the cycle goes on (the others end the session)
STALL_GOES_ON = ("recovered", "recovery_required", "completed_before_cancel", "ended_otherwise")


async def stall_recovery(p: Probe, args, mission_id, seen: int) -> dict:
    """The registered recovery from a stand StallWatch saw (validation plan 3.2): cancel the mission and wait for its
    result; an empty truck then backs off straight by hand once in TELEOP (0.05-``--stall-backoff-m``, measured by the
    reported speed, standing at its end) and goes PAUSED. ``outcome``:
    - recovered;
    - recovery_required: the cancel left the forks in a pocket (--manual-recovery takes it at the next submit);
    - completed_before_cancel / ended_otherwise: the mission ended by itself first (done / another result): no move;
    - loaded_hold: cancelled with the pallet on the forks -- a SCENE_FILE session cannot go on (the next submit is
      cargo_held; the registration counts it as an operational gap);
    - cancel_refused, cancel_not_ended, teleop_refused, reverse_out_of_interval, paused_refused: not recovered.
    Every request's reply is checked (Codex probe review P2)."""
    st = p.state or {}
    out = {"mono": time.monotonic(), "mission": st.get("mission"), "pose": st.get("pose"), "mode": st.get("mode"),
           "load": st.get("load")}
    out["cancel"] = await p.request("mission_cancel", mission_id=mission_id)

    def ended(s):
        r = s.get("last_result") or {}
        return r.get("seq", 0) > seen and r.get("mission_id") == mission_id

    if (out["cancel"] or {}).get("t") != "ack":
        if p.state is not None and ended(p.state):  # the mission ended by itself just before
            out["result"] = p.state.get("last_result")
            out["outcome"] = ("completed_before_cancel" if (out["result"] or {}).get("outcome") == "done"
                              else "ended_otherwise")
            return out
        out["outcome"] = "cancel_refused"
        return out
    out["ended"] = await p.until(ended, 120.0, label="stall_cancel_end")
    out["result"] = (p.state or {}).get("last_result")
    if not out["ended"]:
        out["outcome"] = "cancel_not_ended"
        return out
    result_outcome = (out["result"] or {}).get("outcome")
    if result_outcome == "recovery_required":
        out["outcome"] = "recovery_required"
        return out
    if result_outcome == "done":
        out["outcome"] = "completed_before_cancel"
        return out
    if result_outcome != "cancelled":
        out["outcome"] = "ended_otherwise"
        return out
    if ((p.state or {}).get("load") or {}).get("state") == "loaded":
        out["outcome"] = "loaded_hold"
        return out
    out["teleop"] = await enter_teleop(p)
    arm_gen = int(getattr(p, "arm_gen", 0) or 0)  # enter_teleop keeps this arm's generation (0: the arm failed)
    # the replies come before the state that shows them (Codex probe review round 2 P2): wait for it
    out["armed"] = bool(out["teleop"]) and out["teleop"].get("t") == "ack" and arm_gen > 0 and await p.until(
        lambda s: s.get("mode") == "TELEOP" and bool(s.get("armed")) and int(s.get("arm_gen") or 0) >= arm_gen,
        5.0, label="stall_teleop_armed")
    if not out["armed"]:
        out["outcome"] = "teleop_refused"
        return out
    out["backoff"] = await back_off_by(p, 0.05, float(args.stall_backoff_m), max_s=30.0)
    standing = bool(p.state and p.standing(p.state))
    out["paused"] = await p.request("mode_request", to="PAUSED")
    if not (out["backoff"]["within"] and standing):
        out["outcome"] = "reverse_out_of_interval"
        return out
    if (out["paused"] or {}).get("t") != "ack":
        out["outcome"] = "paused_refused"
        return out
    out["outcome"] = "recovered"
    return out


FINAL_MISSION = ("done", "failed", "cancelled", "recovery_required")  # forklift_core.mission.commands.FINAL


def mission_over(s: dict, mission_id) -> bool:
    """This mission has a final status, or the state's mission slot has moved on from it."""
    m = s.get("mission") or {}
    if m.get("mission_id") == mission_id:
        return m.get("status") in FINAL_MISSION
    return True


async def end_active_mission(p: Probe, mission_id) -> dict:
    """Before a session's end with a mission not yet final: out of STOPPED first (a cancel is refused there,
    stopped_locked) -- stand, rearm, leave STOPPED -- then cancel it (unless it is already recovering from an earlier
    cancel) and wait (120 s) for its final status, resuming once a recovery the PAUSED mode paused."""
    out = {}
    if (p.state or {}).get("mode") == "STOPPED":
        out["standing"] = await p.until(p.standing, 20.0, label="end_standing")
        out["rearm"] = await p.request_until_standing("rearm")
        out["rearmed"] = await p.until(lambda s: s.get("mode") != "STOPPED", 5.0, label="end_rearmed")

    def paused_recovery(s):
        # a cancel that stood with the forks in a pocket (or short of clearance: clear_out) became a recovery, which
        # the PAUSED mode pauses at once -- as the cycle path's U30 handling, it is resumed once (Codex v2 H1 P2)
        m = s.get("mission") or {}
        phase = str(m.get("phase") or "")
        return (m.get("mission_id") == mission_id and m.get("status") == "paused"
                and (phase.startswith("recover_") or phase == "clear_out"))

    m0 = (p.state or {}).get("mission") or {}
    # already recovering (an earlier cancel became a recovery, e.g. the stall recovery's): a second cancel is refused
    # (recovering), so no new cancel -- the recovery is resumed and waited for (Codex v2 H1 P2)
    out["in_recovery"] = p.state is not None and m0.get("mission_id") == mission_id and (
        m0.get("status") == "recovering" or paused_recovery(p.state))
    out["cancel_ended"] = False
    if not out["in_recovery"]:
        out["cancel"] = await p.request("mission_cancel", mission_id=mission_id)
        if not (out["cancel"] and out["cancel"].get("t") == "ack"):
            if (out["cancel"] or {}).get("reason") != "recovering":
                return out
            out["in_recovery"] = True

    stop_by = time.monotonic() + 120.0
    while time.monotonic() < stop_by and p.closed_code is None:
        hit = await p.until(lambda s: mission_over(s, mission_id) or (paused_recovery(s) and "recovery_resume" not in out),
                            max(0.0, stop_by - time.monotonic()), label="end_cancel_ended")
        if p.state is not None and mission_over(p.state, mission_id):
            out["cancel_ended"] = True
            break
        if not hit:
            break
        if p.state is not None and paused_recovery(p.state) and "recovery_resume" not in out:
            out["recovery_resume"] = await p.request_until_standing("mission_resume", mission_id=mission_id)
    return out


async def stall_end_session(p: Probe) -> dict:
    """A recovery not made: STOP, one more cancel of a mission still active, and the session's end asked; the wrapper
    ends the launcher after the probe if the session does not end (its grace)."""
    out = {"stop": await p.request("stop", source="key")}
    out["stopped"] = await p.until(lambda s: s.get("mode") == "STOPPED", 10.0, label="stall_end_stopped")
    m = (p.state or {}).get("mission") or {}
    if m.get("mission_id") and m.get("status") not in FINAL_MISSION:
        out["end_active"] = await end_active_mission(p, m.get("mission_id"))  # rearm first: STOPPED refuses a cancel
    out["session_end"] = await p.request("session_end")
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
        stall_ = (StallWatch(args.stall_recover_s, submit.get("mission_id"), clock=getattr(p, "clock", None))
                  if args.stall_recover_s > 0 else None)
        stalled_, span_ = False, None

        def ended_this_(s):
            r_ = s.get("last_result") or {}
            return r_.get("seq", 0) > seen and r_.get("mission_id") == submit.get("mission_id")

        while time.monotonic() < end_by and p.closed_code is None:
            await p.until(lambda s, w_=stall_: ended_this_(s) or s.get("mode") == "STOPPED" or (w_ is not None and w_(s)),
                          end_by - time.monotonic(), label=f"end:{leg}")
            if (stall_ is not None and p.state is not None and not ended_this_(p.state)
                    and p.state.get("mode") != "STOPPED" and stall_.since is not None
                    and stall_.clock() - stall_.since >= stall_.seconds):
                stalled_ = True  # validation plan 3.2: the registered recovery, then the next leg or the end
                # the stand on the watch's clock (time.monotonic live, the runtime's tick-trace clock): the guard's
                # holds are in sim time, so the false-positive check maps this span through the trace's (S, tick)
                span_ = {"clock": "stall_watch", "since": stall_.since, "decided": stall_.clock()}
                break
            if p.state is None or p.state.get("mode") != "STOPPED" or p.closed_code is not None:
                break
            if stall_ is not None:
                stall_.reset()  # a STOP and its rearm are not part of a stand (Codex probe review P2)
            await p.until(p.standing, 20, label="standing_again")
            await p.request_until_standing("rearm")
            await p.request_until_standing("mission_resume", mission_id=submit.get("mission_id"))
        if stalled_:
            entry["stall_recovery"] = await stall_recovery(p, args, submit.get("mission_id"), seen)
            entry["stall_recovery"]["stall_span_mono"] = span_
        entry["result"] = (p.state or {}).get("last_result")
        if stalled_ and entry["stall_recovery"]["outcome"] not in STALL_GOES_ON:
            p.event("stall_recovery_not_done", outcome=entry["stall_recovery"]["outcome"])
            entry["ended_mono"] = time.monotonic()
            p.report["stall_end"] = await stall_end_session(p)
            break  # the next mission is not asked of a truck that could not be got going again
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


async def wait_steady_states(p: Probe, n: int = 20, gap_s: float = 0.5, timeout_s: float = 120.0) -> bool:
    """True once the last n + 1 states came at most gap_s apart and the newest is fresh (about 2 s of steady flow)."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        st = [s_["mono"] for s_ in p.states[-(n + 1):]]
        if len(st) == n + 1 and max(b_ - a_ for a_, b_ in zip(st, st[1:])) <= gap_s \
                and time.monotonic() - st[-1] <= gap_s:
            return True
        await asyncio.sleep(0.1)
    return False


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
    resend = None  # the resend task of the submit out
    ids = {"n": 0}

    def new_id():
        ids["n"] += 1
        return f"q{ids['n']}"

    if args.cycle == "auto":
        # CT-2 D4: one standing "all remaining pallets" item, resolved by the vehicle's pick_request answers
        q.add({"item_id": new_id(), "kind": "auto", "scope": "remaining", "pallet_id": None, "destination_id": None,
               "label": "남은 팔레트 전부", "added_unix": time.time()})
        q.pick_deadline_ms = int(p.capabilities.get("next_pick_deadline_ms") or mission_queue.PICK_DEADLINE_MS)
        expected = sum(1 for x in (p.targets or {}).get("pallets") or [] if x.get("state") == "available")
    else:
        for leg in [leg for leg in args.cycle.split(",") if leg]:
            pallet_id, destination_id = leg.split(":")
            q.add(mission_queue.new_item(pallet_id, destination_id, new_id()))
        expected = len(q.items)
    p.report["missions_run"] = []
    pick_resend = None
    # The operator resumes once the screen is live: states arriving steadily, not inside the runner's start-up pause
    # (2.5-2.9 s without a state just after the session starts, ws1 2026-10-24 CT-2 V3, which stopped the queue on its
    # 2 s silence rule before anything was sent -- in either arm, by timing alone; the PR 4 note).
    started_wait = time.monotonic()
    p.report["start_steady"] = {"ok": await wait_steady_states(p), "waited_s": round(time.monotonic() - started_wait, 2)}
    q.resume(time.monotonic() * 1000.0)
    closed_seen, deadline = False, time.monotonic() + args.mission_timeout_s * max(1, expected + 1)
    while time.monotonic() < deadline:
        now_ms = time.monotonic() * 1000.0
        if p.closed_code is not None and not closed_seen:
            closed_seen = True
            p.queue_apply(q.on_disconnect())
        p.queue_apply(q.tick(now_ms))
        for entry in p.report["missions_run"]:  # each submission's result, as the queue applied it
            if "result" not in entry and (entry.get("submit") or {}).get("t") == "ack":
                mid_ = entry["submit"].get("mission_id")
                got_ = next((f for f in q.finished if f["mission_id"] == mid_), None)
                if got_ is not None:
                    entry["result"] = {"mission_id": mid_, "outcome": got_["outcome"], "reason": got_["reason"]}
                    entry["ended_mono"] = time.monotonic()
        p.queue_forgotten.update(q.drain_forget())
        # (a not_stopped refusal of the session's first submit is the queue's own one resubmit now: plan D5's
        # exception, tools/mission_queue.py -- the probe's operator resume it replaces is refused in main())
        if q.run != "running" and q.current is None and p.queue_asking is None:
            break  # stopped, nothing out: the queue's end here
        if q.run != "running" and q.phase == "in_mission" and closed_seen:
            break  # its result can no longer come
        ctx = {"connected": p.closed_code is None, "welcomed": p.closed_code is None, "has_lease": p.queue_has_lease,
               "state": p.state, "targets": p.targets, "next_pick": p.capabilities.get("next_pick") is True}
        n = q.next(ctx, now_ms, new_id)
        if n and "pick_request" in n:
            r_ = n["pick_request"]
            p.queue_picking = {"key": {"epoch": q.epoch, "item_id": r_["item_id"], "request_id": r_["request_id"]}}
            p.queue_pick_answer = {"key": dict(p.queue_picking["key"])}
            pick_resend = asyncio.ensure_future(p.request_resent("pick_request", r_["request_id"]))
            n = None
        if n:
            s_ = n["submit"]
            key = {"epoch": q.epoch, "item_id": s_["item_id"], "request_id": s_["request_id"]}
            entry = {"pallet_id": s_["pallet_id"], "destination_id": s_["destination_id"], "item_id": s_["item_id"],
                     "submitted_mono": time.monotonic()}
            p.report["missions_run"].append(entry)
            p.queue_asking = {"key": key, "entry": entry}  # answered through _read
            resend = asyncio.ensure_future(p.request_resent(
                "mission_submit", s_["request_id"], pallet_id=s_["pallet_id"], destination_id=s_["destination_id"],
                targets_revision=s_["targets_revision"]))
        await p.echo()
        await asyncio.sleep(0.25)
    for task_ in (resend, pick_resend):
        if task_ is not None and not task_.done():
            task_.cancel()
    for entry in p.report["missions_run"]:
        if "result" not in entry:
            entry["result_not_this_mission"] = True  # the queue never applied a result for it
    p.report["queue"] = {"stop_reason": q.stop_reason, "stops": q.stops, "counts": q.counts,
                         "items_left": [f"{it['pallet_id']}:{it['destination_id']}" if it.get("kind") != "auto"
                                        else "auto" for it in q.items],
                         "uncertain": [it["item_id"] for it in q.items if it.get("uncertain")],
                         "picks": q.picks, "last_pick_blocked": q.last_pick_blocked,
                         "finished": [{"pallet_id": f.get("pallet_id"), "destination_id": f.get("destination_id"),
                                       "outcome": f.get("outcome"), "mission_id": f.get("mission_id")}
                                      for f in reversed(q.finished)]}
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


# --- R1 confirmation (docs/plans/2026-10-17-realtime-sim.md 5.17) --------------------------------------------------
# The teleop limits the ratios scale (forklift_core.safety.teleop_drive.TeleopLimits; the probe does not import the
# package): 0.15 m/s forward and reverse, curvature 0.18 1/m (within the 0.36 limit).
CONFIRM_LIMITS = {"direct_forward_mps": 0.5, "direct_reverse_mps": 0.25, "curvature_inv_m": 0.36, "loaded_mps": 0.30}
CONFIRM_FORWARD_RATIO = 0.3
CONFIRM_REVERSE_RATIO = 0.6
CONFIRM_CURVATURE_RATIO = 0.5
CONFIRM_LOADED_RATIO = 0.5  # 0.15 m/s under the loaded cap


async def _psleep(p, seconds):
    """asyncio.sleep, or the probe's own sleep when it keeps a clock of its own (the tests' fake)."""
    sleep = getattr(p, "sleep", None)
    await (sleep(seconds) if sleep is not None else asyncio.sleep(seconds))


# R1 results: the STOP's completion is the four wheels still for 0.2 sim s; a rearm 0.5 wall s after the rear axle
# stood (0.2 sim s at 0.4) came before it could be seen. The probe now holds the STOP 1 sim s before rearming.
STOP_HOLD_SIM_S = 1.0


async def confirm_stop(p, label, clock=time.monotonic, factor=1.0) -> dict:
    """A STOP sent now (a 'key' source, as the console's button): its send time and request on the probe's clock --
    the runner's link log holds the same request processed -- whether the truck was moving when it was sent, then the
    stop and the stand waited for and a rearm (5.17 T3). A STOP not acknowledged is recorded and not rearmed."""
    s = p.state or {}
    rec = {"label": label, "sent_mono": clock(), "moving_at_send": not p.standing(s), "mode_at_send": s.get("mode"),
           "phase_at_send": (s.get("mission") or {}).get("phase")}
    reply = await p.request("stop", source="key")
    rec["reply"] = None if reply is None else reply.get("t")
    rec["request_id"] = None if reply is None else reply.get("request_id")
    rec["stopped"] = bool(reply) and await p.until(lambda st: st.get("mode") == "STOPPED", 5, label=f"stopped_{label}")
    rec["standing"] = rec["stopped"] and await p.until(p.standing, 20, label=f"standing_{label}")
    rec["standing_mono"] = clock() if rec["standing"] else None
    rec["rearm"] = None
    if rec["standing"]:
        await _psleep(p, max(0.5, STOP_HOLD_SIM_S / factor))
        rearm = await p.request_until_standing("rearm")
        rec["rearm"] = None if rearm is None else rearm.get("t")
    p.report.setdefault("confirm", {}).setdefault("stops", []).append(rec)
    return rec


async def confirm_enter_teleop(p, tries=60, clock=time.monotonic) -> dict:
    """PAUSED, standing, TELEOP and armed, as enter_teleop -- counting the TELEOP entries refused for pacing (5.17 T1:
    the arbiter refuses the mode while the pacer is not ok)."""
    rec = {"start_mono": clock(), "pacing_refusals": 0, "not_stopped_refusals": 0, "entered": False}
    await p.request("mode_request", to="PAUSED")
    await p.until(p.standing, 30, label="standing_before_teleop")
    reply = None
    for _ in range(tries):
        reply = await p.request("mode_request", to="TELEOP")
        if reply is not None and reply.get("detail") == "pacing":
            rec["pacing_refusals"] += 1
        elif reply is not None and reply.get("reason") in ("not_stopped", "no_feedback:rear_axle_speed"):
            rec["not_stopped_refusals"] += 1
        else:
            break
        await _psleep(p, 0.5)
    rec["reply"] = None if reply is None else {"t": reply.get("t"), "reason": reply.get("reason"),
                                               "detail": reply.get("detail")}
    if reply and reply.get("t") == "ack":
        arm = await p.request("arm")
        p.arm_gen = (arm or {}).get("arm_gen", 0)
        rec["entered"] = bool(arm) and arm.get("t") == "ack"
    rec["end_mono"] = clock()
    p.report.setdefault("confirm", {}).setdefault("teleop_entries", []).append(rec)
    return rec


async def hold_teleop(p, label, sim_seconds, speed_ratio, curvature_ratio, mode="direct", lift_dir=0,
                      stop_after_sim_s=None, factor=1.0, clock=time.monotonic) -> dict:
    """A command held for ``sim_seconds`` of simulated time (``sim_seconds / factor`` wall s at the declared factor),
    dead man pressed, every 0.05 wall s (5.17 T1: the interval, its mode and ratios are what the analysis reads the
    control ticks inside); with ``stop_after_sim_s`` a STOP is sent that far in (STOP-T, while moving) and the hold
    ends there. The truck is let come to a stand after it (zero command); ``stand_mono`` is when it stood."""
    rec = {"label": label, "mode": mode, "deadman": True, "speed_ratio": speed_ratio, "curvature_ratio": curvature_ratio,
           "lift_dir": int(lift_dir), "sim_seconds": sim_seconds, "factor": factor, "start_mono": clock(),
           "start_state": _pose_of(p.state)}
    start = rec["start_mono"]
    seconds = sim_seconds / factor
    stop_after_s = None if stop_after_sim_s is None else stop_after_sim_s / factor
    while clock() - start < seconds:
        if stop_after_s is not None and clock() - start >= stop_after_s:
            rec["stop"] = await confirm_stop(p, "T", clock=clock, factor=factor)
            rec["end_mono"] = rec["stop"]["sent_mono"]
            break
        await p.teleop(speed_ratio, curvature_ratio, True, mode=mode, lift_dir=lift_dir)
        await _psleep(p, 0.05)
    else:
        rec["end_mono"] = clock()
    rec["end_state"] = _pose_of(p.state)
    if "stop" not in rec:
        stop_by = clock() + 15.0 / factor
        while clock() < stop_by and not (p.state and p.standing(p.state)):
            await p.teleop(0.0, 0.0, True, mode="direct")
            await _psleep(p, 0.05)
    rec["stand_mono"] = clock()
    rec["stand_state"] = _pose_of(p.state)
    p.report.setdefault("confirm", {}).setdefault("holds", []).append(rec)
    return rec


def _pose_of(s):
    s = s or {}
    pose = s.get("pose") or {}
    return {"x_m": pose.get("x_m"), "y_m": pose.get("y_m"), "yaw_rad": pose.get("yaw_rad"), "lift_m": s.get("lift_m"),
            "mode": s.get("mode"), "armed": s.get("armed")}


async def command_expiry(p, factor, clock=time.monotonic) -> dict:
    """5.17 T2 boundary test, apart from video expiry: drive forward on direct commands until moving, then send no
    teleop command for 0.5/f wall s (the lease is kept by the background echo), and record what the vehicle shows."""
    rec = {"silence_s": 0.5 / factor}
    start = clock()
    while clock() - start < 1.0 / factor:
        await p.teleop(CONFIRM_FORWARD_RATIO, 0.0, True, mode="direct")
        await _psleep(p, 0.05)
    rec["moving_before"] = not p.standing(p.state or {})
    rec["silence_start_mono"] = clock()
    end = rec["silence_start_mono"] + rec["silence_s"]
    while clock() < end:
        await _psleep(p, 0.05)
    rec["silence_end_mono"] = clock()
    s = p.state or {}
    rec["armed_after"] = s.get("armed")
    rec["last_teleop_reject_after"] = s.get("last_teleop_reject")
    rec["standing_after"] = p.standing(s)
    p.report.setdefault("confirm", {})["expiry"] = rec
    return rec


async def confirm_scenario(p: Probe, args):
    """5.17: one AUTO cycle P-1:D-1 with a STOP while moving in transport (then a short teleop while the mission
    lives, loaded) and one while moving on the return, each rearmed and resumed; after the mission's result the
    teleop block -- direct forward (a STOP inside it), reverse, left and right curves, lift up and down, video forward
    and reverse -- and the command-expiry test; then the session ends."""
    rep = p.report.setdefault("confirm", {})
    rep["steps"] = []
    f = args.live_factor  # holds are given in simulated seconds; their wall length is sim / f

    def step(name, ok):
        rep["steps"].append({"step": name, "ok": bool(ok), "mono": time.monotonic()})
        return ok

    await p.open()
    await p.open_media()
    await p.until(lambda s: True, 10, label="first_state")
    lease = await p.request("lease_request")
    p.lease_id = lease.get("lease_id") if lease else None
    p.report["session"] = {"session_epoch": p.epoch, "lease_id": p.lease_id}
    await p.until(lambda s: p.targets is not None, 10, label="targets")
    await p.until(p.standing, 30, label="standing_at_start")
    submit = None
    for _ in range(20):
        submit = await p.request("mission_submit", pallet_id="P-1", destination_id="D-1",
                                 targets_revision=p.targets["targets_revision"])
        if submit is None or submit.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed"):
            break
        await asyncio.sleep(0.5)
    mission_id = (submit or {}).get("mission_id")
    rep["submit"] = submit
    if not step("submitted", mission_id):
        return
    seen = ((p.state or {}).get("last_result") or {}).get("seq", 0)

    def ended(s):
        r_ = s.get("last_result") or {}
        return r_.get("seq", 0) > seen and r_.get("mission_id") == mission_id

    def moving_in(phase):
        return lambda s: (s.get("mission") or {}).get("phase") == phase and not p.standing(s) and s.get("mode") == "AUTO"

    async def resume():
        await p.until(p.standing, 10, label="standing_before_resume")
        await asyncio.sleep(0.4)
        r_ = await p.request_until_standing("mission_resume", mission_id=mission_id)
        return step("resumed", r_ and r_.get("t") == "ack") and await p.until(lambda s: s.get("mode") == "AUTO", 10,
                                                                              label="auto_again")

    timeout = args.mission_timeout_s
    # STOP-A1 in transport, moving; a short loaded teleop while the mission lives; resume
    if step("transport_moving", await p.until(lambda s: moving_in("transport")(s) or ended(s), timeout,
                                              label="transport_moving") and not ended(p.state)):
        a1 = await confirm_stop(p, "A1", factor=f)
        if step("stop_A1", a1["rearm"] == "ack"):
            entry = await confirm_enter_teleop(p)
            if step("loaded_teleop_entry", entry["entered"]):
                await hold_teleop(p, "loaded_forward", 2.0, CONFIRM_LOADED_RATIO, 0.0, factor=f)
                await p.request("mode_request", to="PAUSED")
            await resume()
    # STOP-A2 on the return, moving; resume
    if step("return_moving", await p.until(lambda s: moving_in("return_home")(s) or ended(s), timeout,
                                           label="return_moving") and not ended(p.state)):
        a2 = await confirm_stop(p, "A2", factor=f)
        if step("stop_A2", a2["rearm"] == "ack"):
            await p.request("mode_request", to="PAUSED")
            await resume()
    end_by = time.monotonic() + timeout
    while time.monotonic() < end_by and p.closed_code is None:
        await p.until(lambda s: ended(s) or s.get("mode") == "STOPPED", end_by - time.monotonic(), label="mission_end")
        if p.state is None or ended(p.state) or p.state.get("mode") != "STOPPED":
            break
        await p.until(p.standing, 20, label="standing_again")
        await p.request_until_standing("rearm")
        await p.request_until_standing("mission_resume", mission_id=mission_id)
    rep["mission_result"] = (p.state or {}).get("last_result")
    step("mission_ended", ended(p.state or {}))
    # the teleop block, standing after the mission
    entry = await confirm_enter_teleop(p)
    if step("teleop_entry", entry["entered"]):
        fwd = await hold_teleop(p, "forward", 3.0, CONFIRM_FORWARD_RATIO, 0.0, stop_after_sim_s=1.5, factor=f)
        if "stop" in fwd:
            entry = await confirm_enter_teleop(p)
            step("teleop_reentry_after_stop", entry["entered"])
        await hold_teleop(p, "reverse", 3.0, -CONFIRM_REVERSE_RATIO, 0.0, factor=f)
        await hold_teleop(p, "left", 2.0, CONFIRM_FORWARD_RATIO, CONFIRM_CURVATURE_RATIO, factor=f)
        await hold_teleop(p, "right", 2.0, CONFIRM_FORWARD_RATIO, -CONFIRM_CURVATURE_RATIO, factor=f)
        await hold_teleop(p, "lift_up", 2.0, 0.0, 0.0, lift_dir=1, factor=f)
        await hold_teleop(p, "lift_down", 2.0, 0.0, 0.0, lift_dir=-1, factor=f)
        await hold_teleop(p, "video_forward", 3.0, CONFIRM_FORWARD_RATIO, 0.0, mode="video", factor=f)
        await hold_teleop(p, "video_reverse", 3.0, -CONFIRM_REVERSE_RATIO, 0.0, mode="video", factor=f)
        await command_expiry(p, args.live_factor)
        await p.request("mode_request", to="PAUSED")
    await p.until(p.standing, 20, label="standing_at_end")
    if getattr(args, "end_session", False):
        rep["session_end"] = await p.request_until_standing("session_end")


async def hold_scenario(p, args):
    """B-P2d (start relocalization plan 4.4, 1.2 step 12): a relocalized session that reached running (the port file
    says so only after the localization) stands for --hold-sim-s of simulated time -- wall: that over --live-factor,
    plus a second -- and is then ended. Whether the localization held is read from the runner's own records, not
    here; a session that closes during the hold is left as it ended."""
    await p.open()
    await p.until(lambda s: s is not None and s.get("pose") is not None, 30, label="localized_state")
    await _hold_and_end(p, args)


async def _hold_and_end(p, args):
    """Stand --hold-sim-s of simulated time once localized, then end the session (unless it closed meanwhile)."""
    hold = p.report["hold"] = {"start_mono": time.monotonic(), "hold_sim_s": args.hold_sim_s,
                               "factor": args.live_factor}
    end = time.monotonic() + args.hold_sim_s / args.live_factor + 1.0
    while time.monotonic() < end and p.closed_code is None:
        await asyncio.sleep(0.2)
    hold["end_mono"], hold["closed_during"] = time.monotonic(), p.closed_code
    await _end_session(p)


async def _end_session(p):
    """End the session (unless it closed already) and wait for its close."""
    if p.closed_code is None:
        lease = await p.request("lease_request")
        p.lease_id = (lease or {}).get("lease_id") or p.lease_id  # refused: the one it holds
        p.report["session_end"] = await p.request_until_standing("session_end")
        close_by = time.monotonic() + 120
        while p.closed_code is None and time.monotonic() < close_by:
            await asyncio.sleep(0.1)
    p.report["close_code"] = p.closed_code


HANDOVER_CODE = 1012  # the early console's close when the live runtime adopts it ("session ready")


async def _reopen(p, deadline: float) -> None:
    """Connect again until ``deadline`` (the listening server may refuse for a moment while it changes hands)."""
    while True:
        try:
            await p.open()
            return
        except (OSError, websockets.exceptions.InvalidHandshake):
            if time.monotonic() >= deadline:
                raise
            await asyncio.sleep(0.5)


async def place_scenario(p, args):
    """F1 (docs/plans/2026-10-31-operator-placed-start.md 4.2 b): the probe is the operator. While the candidates are
    shown and placements are left, it places the click model's draw around the truth (--truth, the placed start; the
    seed and the start's index fix the draws, as tools/f1_place_eval.py's sessions), and confirms the first preview it
    sees (the plan's fixed policy). Once localized it stands the hold and ends the session, as the hold scenario; a
    session that never localizes is left to the operator's time."""
    from f1_place_eval import (  # noqa: PLC0415 -- the same clicks as the offline sessions and records
        click as place_click,
        stress_click,
    )


    await p.open()
    lease = await p.request("lease_request")
    p.lease_id = (lease or {}).get("lease_id")
    places = p.report["places"] = []
    if args.click_kind:  # a record click (f1_place_eval STRESS_KINDS) at every placement instead of the click model
        p.report["click_kind"] = args.click_kind
    sent = 0
    acked = 0  # placements the vehicle took: the next one waits until its state shows them
    invalid = 0  # clicks off the map: refused by the vehicle uncounted, each one try of the operator's three (plan 3)
    end = time.monotonic() + args.place_wait_s
    while time.monotonic() < end and p.closed_code is None and sent < 10:
        st = p.state or {}
        if st.get("pose") is not None:
            break
        r = (st.get("slam") or {}).get("reloc") or {}
        state, done = r.get("state"), r.get("places") or 0
        k = done + invalid
        if state in ("awaiting_pick", "reloc_failed") and done >= acked and k < (r.get("max_places") or 0):
            c = (stress_click(args.click_seed, args.click_index, args.click_kind, args.truth) if args.click_kind
                 else place_click(args.click_seed, args.click_index, k, args.truth))
            reply = await p.request("relocalize_place", generation=r["generation"], pose=[float(v) for v in c]) or {}
            sent += 1
            places.append({"k": k, "click": list(c), "reply": reply.get("t"), "reason": reply.get("reason"),
                           "mono": time.monotonic()})
            if reply.get("t") == "ack":
                acked = done + 1
            elif reply.get("reason") == "reloc_place_invalid":
                invalid += 1
        elif state == "place_preview" and "confirm" not in p.report:
            reply = await p.request("relocalize_place_confirm", generation=r["generation"],
                                    preview_id=r["preview_id"]) or {}
            sent += 1
            p.report["confirm"] = {"preview_id": r["preview_id"], "reply": reply.get("t"),
                                   "reason": reply.get("reason"), "mono": time.monotonic()}
        await asyncio.sleep(0.2)
    if p.closed_code == HANDOVER_CODE:
        # the live runtime adopted the early console (console/live/server.py reconnect_clients): hello again and read
        # the session's state there, as the page does after any close but a session end (F1 Isaac development 13450:
        # the probe stopped here, sent no session end, and the launcher had to be ended)
        p.report["handover"] = {"code": p.closed_code, "mono": time.monotonic()}
        epoch = p.epoch
        await _reopen(p, time.monotonic() + 30.0)
        if p.epoch != epoch:  # another session on the port: not this one's to hold or end (Codex F1 dev review P3)
            p.report["handover"]["epoch_changed"] = [epoch, p.epoch]
            p.report["localized"] = False
            p.report["close_code"] = p.closed_code
            return
        await p.until(lambda s: s is not None and s.get("pose") is not None, 30, label="localized_state")
    p.report["localized"] = bool((p.state or {}).get("pose") is not None)
    if p.report["localized"]:
        if getattr(args, "after_place", "hold") == "reverse":  # plan 2026-11-06 2 (reverse_place): V2's placements
            await _reverse_localized(p, args)
        elif getattr(args, "after_place", "hold") == "block_check":  # plan 2026-11-08 5: the vehicle-side F1 block
            await _block_check(p, args)
            await _end_session(p)
        else:
            await _hold_and_end(p, args)
        return
    while p.closed_code is None and time.monotonic() < end:  # the operator's time ends the session
        await asyncio.sleep(0.5)
    p.report["close_code"] = p.closed_code


REVERSE_RATIO = -0.4  # direct_reverse_mps 0.25 x 0.4 = 0.10 m/s: V_REV_U (plan 2026-11-06 2.1)
GENERATION_WAIT_S = 45.0  # wall s: a generation's own 30 s deadline, its 5 s grace and the state's lag
WATCH_WALL_FACTOR = 3.0  # the watch gives up at this many times its wall time at the declared factor
REVERSE_MAX_STROKES = 15  # chatter can reach D_TOTAL
REVERSE_TRIES = 4  # presses a stroke may take: a stalled tick disarms and closes it (no_feedback)
REVERSE_COUNTS_M = 0.05  # a press that moved less takes no stroke of the count
REVERSE_MOVES_M = 0.005  # a press moving less than this for --stroke-stuck-s is held by the permit
REVERSE_BLOCKED_MAX = 2  # such presses in a row end the strokes (the session goes on to its end and record)
REVERSE_STEP_M = {"half": 0.5, "chatter": 0.2}


def _pose_check(s):
    return (s or {}).get("pose_check")


def _pc_status(s):
    """The check's status with the shadow's prefix taken off; None with the check off."""
    pc = _pose_check(s)
    if pc is None:
        return None
    st = pc.get("status") or ""
    return st[len("shadow:"):] if st.startswith("shadow:") else st


def release_due(kind: str, start_left_m, stroke) -> bool:
    """Plan 2026-11-06 2.1, once the vehicle showed the stroke active: let go when it is no longer active (closed by
    the vehicle), and for half / chatter once its distance left went down by 0.5 / 0.2 m."""
    if stroke is None or stroke.get("state") != "active":
        return True
    if kind == "full":
        return False
    left = stroke.get("left_m")
    return left is not None and start_left_m is not None and start_left_m - left >= REVERSE_STEP_M[kind] - 1e-9


async def reverse_scenario(p, args):
    """Plan 2026-11-06-unconfirmed-validation 2.1: a relocalized session with the unconfirmed position check (on or
    shadow). Once the start check is over, a start that is not confirmed (always, in the shadow) is backed straight
    up by hand -- TELEOP, armed, the deadman held at -0.10 m/s with the steering at 0 -- one stroke at a time by the
    pressing rule (--reverse-kind: full lets go when the vehicle closes the stroke, half after 0.5 m, chatter every
    0.2 m), each followed by a stand and the re-match it may start, at most REVERSE_MAX_STROKES. Then PAUSED and a
    stand of --hold-sim-s; with --mission-after-confirm the first --cycle leg once confirmed (or after the strokes,
    in the shadow); then the session is ended. A check that ended (on) ends the session by itself. Reads neither the
    truth nor the scene."""
    await p.open()
    await p.until(lambda s: s is not None and s.get("pose") is not None, 30, label="localized_state")
    await _reverse_localized(p, args)


async def _reverse_localized(p, args):
    """The reverse scenario from a localized session on (after the place scenario's placement too)."""
    rev = p.report["reverse"] = {"kind": args.reverse_kind, "strokes": [], "mission": None}
    if _pose_check(p.state) is None:  # the check off: stand as the hold scenario
        rev["check"] = "off"
        await _hold_and_end(p, args)
        return
    shadow = (_pose_check(p.state).get("status") or "").startswith("shadow:")
    rev["check"] = "shadow" if shadow else "on"
    lease = await p.request("lease_request")
    p.lease_id = (lease or {}).get("lease_id")
    await p.until(lambda s: _pc_status(s) not in (None, "checking"), args.reverse_wait_s, label="start_checked")
    rev["start_status"] = _pc_status(p.state)
    if shadow or rev["start_status"] == "unconfirmed":
        await _reverse_strokes(p, args, rev, shadow)
    status = _pc_status(p.state) or ""
    rev["end_status"] = status
    if not shadow and status.startswith("ended:"):  # the session ends with the check
        close_by = time.monotonic() + 120
        while p.closed_code is None and time.monotonic() < close_by:
            await asyncio.sleep(0.2)
        p.report["close_code"] = p.closed_code
        return
    if p.closed_code is not None:
        p.report["close_code"] = p.closed_code
        return
    await p.request("mode_request", to="PAUSED")
    await p.until(p.standing, 30, label="standing_paused")
    end = time.monotonic() + args.hold_sim_s / args.live_factor + 1.0
    while time.monotonic() < end and p.closed_code is None:
        await asyncio.sleep(0.2)
    if args.mission_after_confirm and p.closed_code is None and (shadow or _pc_status(p.state) == "confirmed"):
        rev["mission"] = await _one_mission(p, args)
        rev["post_mission"] = await _post_mission_watch(p, args, shadow)
    await _end_session(p)


async def _reverse_strokes(p, args, rev, shadow):
    teleop = None
    end = time.monotonic() + args.reverse_wait_s
    while time.monotonic() < end and p.closed_code is None:  # standing and the pacing are the vehicle's call
        teleop = await p.request("mode_request", to="TELEOP")
        if teleop is None or (teleop.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed")
                              and teleop.get("detail") != "pacing"):
            break
        await asyncio.sleep(0.5)
    rev["teleop_entry"] = teleop
    if not teleop or teleop.get("t") != "ack":
        return
    n, blocked = 0, 0
    for _ in range(REVERSE_MAX_STROKES * REVERSE_TRIES):
        if n >= REVERSE_MAX_STROKES:
            break
        status = _pc_status(p.state) or ""
        if p.closed_code is not None or (not shadow and (status.startswith("ended:") or status == "confirmed")):
            break  # a shadow's provisional verdict selects nothing (plan 1.1)
        stroke = (_pose_check(p.state) or {}).get("stroke")
        if stroke is None or stroke.get("state") == "spent":
            break
        if stroke.get("state") != "ready":
            released_ = asyncio.ensure_future(_keep_released(p))
            try:
                ready = await p.until(lambda s: ((_pose_check(s) or {}).get("stroke") or {}).get("state")
                                      in ("ready", "spent"), 30, label="stroke_ready")
            finally:
                released_.cancel()
            if not ready:
                break
            continue
        if not (p.state or {}).get("armed"):  # first, or disarmed by a stalled tick (no_feedback)
            arm = await _arm(p, args)
            rev.setdefault("arms", []).append(arm)
            if not arm or arm.get("t") != "ack":
                break
        rec = {"n": n, "start_left_m": stroke.get("left_m"), "start_total_left_m": stroke.get("total_left_m"),
               "start_mono": time.monotonic()}
        seen_active, released, stk = False, "timeout", None
        end = time.monotonic() + args.stroke_wall_s
        moved_mono, moved_left = time.monotonic(), rec["start_total_left_m"]
        while time.monotonic() < end and p.closed_code is None:
            stk = (_pose_check(p.state) or {}).get("stroke")
            state = (stk or {}).get("state")
            seen_active = seen_active or state == "active"
            # "closed" unseen active: it opened and closed between two published states (Codex 2d round 4 P2)
            if (seen_active or state == "closed") and release_due(args.reverse_kind, rec["start_left_m"], stk):
                released = state or "none"
                break
            left = (stk or {}).get("total_left_m")
            if left is not None and moved_left is not None and moved_left - left >= REVERSE_MOVES_M:
                moved_mono, moved_left = time.monotonic(), left
            if time.monotonic() - moved_mono > args.stroke_stuck_s:
                released = "blocked"  # the permit holds it, or nothing took the press: no use pressing on
                break
            await p.teleop(REVERSE_RATIO, 0.0, True)
            await asyncio.sleep(0.05)
        # the stroke as seen at the release (the stand after it starts the next one)
        rec.update(released=released, end_left_m=(stk or {}).get("left_m"),
                   end_total_left_m=(stk or {}).get("total_left_m"), release_mono=time.monotonic())
        # the deadman let go, the command sent on: one older than 0.30 s disarms (the arbiter's command age)
        released_ = asyncio.ensure_future(_keep_released(p))
        try:
            rec["standing"] = bool(await p.until(p.standing, 30, label="standing_after_stroke"))
            rec["rematch"] = await _await_rematch(p, shadow, args)
        finally:
            released_.cancel()
        rec["status_after"] = _pc_status(p.state)
        rev["strokes"].append(rec)
        moved = (rec["start_total_left_m"] or 0.0) - (rec["end_total_left_m"] or rec["start_total_left_m"] or 0.0)
        n += moved >= REVERSE_COUNTS_M  # a press cut at once takes no stroke of the count
        blocked = blocked + 1 if released == "blocked" else 0
        if blocked >= REVERSE_BLOCKED_MAX:
            rev["blocked"] = True
            break


async def _arm(p, args):
    """Arm once the vehicle lets it (the latch of a no_feedback, the pacing): its reply."""
    reply = None
    end = time.monotonic() + args.reverse_wait_s
    released_ = asyncio.ensure_future(_keep_released(p))
    try:
        while time.monotonic() < end and p.closed_code is None:
            reply = await p.request("arm")
            if reply is not None and reply.get("t") == "ack":
                p.arm_gen = reply.get("arm_gen", 0)
                # the state says so before the press: its first command needs the new generation
                await p.until(lambda s: bool(s.get("armed")), 5, label="armed")
                return reply
            await asyncio.sleep(0.5)
    finally:
        released_.cancel()
    return reply


async def _post_mission_watch(p, args, shadow) -> dict:
    """The revision plan 9.10: the truck stands after its mission (not paused: a paused mission stays active and no
    re-match runs; no SLAM hold) for --post-mission-sim-s of simulated time, an estimate summed at the factor the run
    reports (these sessions carry no frames to read a stamp from). A generation that started is waited for to its result or its
    deadline, whatever the watch has left: ended first, the late and failing ones would be cut off the sample. Its
    deadline is the check's own, on the wall clock (pose_runtime.DEADLINE_S + RESULT_GRACE_S)."""
    start = last = time.monotonic()
    wall_end = start + WATCH_WALL_FACTOR * args.post_mission_sim_s / args.live_factor
    out = {"sim_s": args.post_mission_sim_s, "elapsed_sim_s": 0.0, "started": False, "ended": "wall_limit"}
    seen = None
    while p.closed_code is None:
        now = time.monotonic()
        if p.state is not seen:  # no state, no simulated time counted: a stalled run does not finish the watch
            seen = p.state
            achieved = ((p.state or {}).get("pacing") or {}).get("realtime_factor") or args.live_factor
            out["elapsed_sim_s"] += (now - last) * achieved
        last = now
        if _rematching(p.state, shadow):
            out["started"] = True
            at = time.monotonic()
            done = await p.until(lambda s: not _rematching(s, shadow), GENERATION_WAIT_S, label="post_mission_rematch")
            out.update(done=bool(done), wall_s=time.monotonic() - at, status=_pc_status(p.state),
                       ended="result" if done else ("closed" if p.closed_code is not None else "timeout"))
            return out
        if out["elapsed_sim_s"] >= args.post_mission_sim_s:
            out["ended"] = "watch"
            break
        if now >= wall_end:
            break
        await asyncio.sleep(0.2)
    if p.closed_code is not None:
        out["ended"] = "closed"
    out["wall_s"] = time.monotonic() - start
    return out


def _rematching(s, shadow) -> bool:
    """On: the status checking. The shadow: its generation running."""
    return (_pose_check(s) or {}).get("rematch") is not None if shadow else _pc_status(s) == "checking"


async def _keep_released(p):
    while True:
        await p.teleop(0.0, 0.0, False)
        await asyncio.sleep(0.05)


async def _await_rematch(p, shadow, args) -> dict:
    """The re-match a stand may start (on: the status checking; the shadow: its generation running), to its end."""
    if not await p.until(lambda s: _rematching(s, shadow), max(3.0, 2.0 / args.live_factor), label="rematch_started"):
        return {"started": False}
    t0 = time.monotonic()
    done = await p.until(lambda s: not _rematching(s, shadow), args.reverse_wait_s, label="rematch_done")
    return {"started": True, "done": bool(done), "wall_s": time.monotonic() - t0, "status": _pc_status(p.state)}


BLOCK_REASON = "operator_placed_start"
BLOCK_ACTIVE = ("accepted", "preparing", "running", "resuming", "recovering")


def _xy(s):
    pose = (s or {}).get("pose") or {}
    return (pose.get("x_m"), pose.get("y_m")) if pose.get("x_m") is not None else None


async def _block_check(p, args):
    """docs/plans/2026-11-08-f1-vehicle-block.md 5: once an operator-placed start is localized, what the vehicle refuses
    and keeps open, judged here against --block-expect (block: AUTO, missions and picks refused operator_placed_start,
    the state's start_block, no AUTO, teleop taken and the truck moved; allow: the submit taken -- then cancelled --
    and no start_block). Every check must run: a step that could not is a failure, never a pass. The record's source
    (the runner's start_reloc) is checked afterwards from the run's files."""
    out = p.report["block_check"] = {"expect": args.block_expect, "checks": {}, "modes": []}
    checks = out["checks"]
    first = len(getattr(p, "states", None) or [])  # every state received from here on is judged, not snapshots
    confirm = (p.report.get("confirm") or {}).get("reply")
    checks["placement_confirmed"] = confirm == "ack"
    checks["same_epoch"] = not (p.report.get("handover") or {}).get("epoch_changed")
    lease = await p.request("lease_request")
    p.lease_id = (lease or {}).get("lease_id") or p.lease_id
    standing = await p.until(p.standing, 30, label="standing_before_block_check")
    out["start_block"] = (p.state or {}).get("start_block")
    pallet_id, destination_id = args.cycle.split(",")[0].split(":")
    await p.until(lambda s: p.targets is not None, 10, label="targets")
    submit = None
    for _ in range(40):
        submit = await p.request("mission_submit", pallet_id=pallet_id, destination_id=destination_id,
                                 targets_revision=(p.targets or {}).get("targets_revision"))
        if submit is None or submit.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed",
                                                          "stale_targets"):
            break
        await asyncio.sleep(0.5)
    out["submit"] = submit
    out["modes"].append((p.state or {}).get("mode"))
    if args.block_expect == "allow":
        checks["standing"] = bool(standing)
        checks["submit_taken"] = bool(submit) and submit.get("t") == "ack"
        checks["no_start_block"] = out["start_block"] is None
        if checks["submit_taken"]:
            out["cancel"] = await p.request("mission_cancel", mission_id=submit.get("mission_id"))
            await p.until(lambda s: (s.get("mission") or {}).get("status") not in ("accepted", "preparing", "running",
                                                                                   "cancelling"), 120,
                          label="cancelled")
    else:
        out["auto"] = await p.request("mode_request", to="AUTO")
        out["pick"] = await p.request("pick_request")
        out["modes"].append((p.state or {}).get("mode"))
        checks["standing"] = bool(standing)
        checks["start_block"] = (out["start_block"] or {}).get("reason") == BLOCK_REASON
        for name in ("submit", "auto", "pick"):
            r = out[name] or {}
            checks[f"{name}_refused"] = r.get("t") == "reject" and r.get("reason") == BLOCK_REASON
        before = _xy(p.state)
        out["teleop"] = await teleop_drive(p, args.block_teleop_s, 0.15, 0.0)
        after = _xy(p.state)
        out["moved_m"] = None if before is None or after is None else math.hypot(after[0] - before[0],
                                                                                after[1] - before[1])
        checks["teleop_taken"] = bool(out["teleop"]) and out["teleop"].get("t") == "ack"
        checks["teleop_moved"] = out["moved_m"] is not None and out["moved_m"] > 0.02
        out["modes"].append((p.state or {}).get("mode"))
        seen = (getattr(p, "states", None) or [])[first:]
        out["states_judged"] = len(seen)
        checks["states_seen"] = len(seen) > 0
        checks["never_auto"] = all((x or {}).get("mode") != "AUTO" for x in seen + [p.state])
        checks["no_mission"] = all(((x or {}).get("mission") or {}).get("status") not in BLOCK_ACTIVE
                                   for x in seen + [p.state])
    out["verdict"] = "pass" if all(checks.values()) else "fail"
    out["failed"] = sorted(k for k, v in checks.items() if not v)


async def _one_mission(p, args) -> dict:
    """The first --cycle leg, submitted standing, and its result."""
    pallet_id, destination_id = args.cycle.split(",")[0].split(":")
    await p.until(lambda s: p.targets is not None, 10, label="targets")
    await p.until(p.standing, 30, label="standing_before_submit")
    seen = ((p.state or {}).get("last_result") or {}).get("seq", 0)
    submit = None
    for _ in range(40):
        submit = await p.request("mission_submit", pallet_id=pallet_id, destination_id=destination_id,
                                 targets_revision=(p.targets or {}).get("targets_revision"))
        if submit is None or submit.get("reason") not in ("not_stopped", "no_feedback:rear_axle_speed",
                                                          "stale_targets"):
            break
        await asyncio.sleep(0.5)
    out = {"pallet_id": pallet_id, "destination_id": destination_id, "submit": submit}
    if not submit or submit.get("t") != "ack":
        return out
    mid = submit.get("mission_id")

    def ended(s):
        r = s.get("last_result") or {}
        return r.get("seq", 0) > seen and r.get("mission_id") == mid

    out["result"] = (p.state or {}).get("last_result") if await p.until(ended, args.mission_timeout_s,
                                                                         label="mission_result") else None
    return out


def pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    try:  # a zombie (ended, not yet reaped by its parent) counts as ended
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rsplit(")", 1)[1].split()[0] != "Z"
    except OSError:
        return True


# F1: the early console's port-file states (the start still unknown) and running: the place scenario connects then
RELOC_PORT_STATES = ("relocalizing", "awaiting_pick", "reloc_failed", "stopped", "placing", "place_preview",
                     "localizing", "running")


def wait_port(port_file: Path, wait_s: float, launcher_pid=None, sleep=time.sleep, clock=time.monotonic,
              states=("running",)):
    """The port file of a running session, or (None, why): ``launcher_ended`` when the launcher (isaac_live.sbatch)
    exits before its session runs -- lvram (2026-10-27): it ended on its first line (a required variable) and the
    probe waited out its 900 s holding the job -- else ``no_port_file`` at the deadline."""
    end = clock() + wait_s
    info = None
    while clock() < end:
        # the launcher first: a "running" port file an earlier session left in the output is not this one's (Codex)
        if launcher_pid is not None and not pid_alive(launcher_pid):
            return None, "launcher_ended"
        if port_file.exists():
            info = json.loads(port_file.read_text())
            if info.get("state") in states:
                return info, None
        sleep(1.0)
    return None, "no_port_file"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port-file", type=Path, required=True)
    ap.add_argument("--report", type=Path, required=True)
    ap.add_argument("--scenario", choices=("main", "slow", "marker", "approach", "planwait", "cycle", "queue", "lift", "reset",
                                           "restart", "tour", "hold", "place", "reverse", "confirm"),
                    default="main")
    ap.add_argument("--manual-recovery", action="store_true",
                    help="cycle scenario (plan D14 U20 c): a mission ending recovery_required is recovered by hand "
                    "before the next submit")
    ap.add_argument("--manual-extra-s", type=float, default=2.0, help="manual recovery: reverse this long after it")
    ap.add_argument("--stall-recover-s", type=float, default=0.0,
                    help="cycle scenario (docs/plans/2026-11-08-live-riib-defaults-validation.md 3.2): a running mission "
                    "whose truck stands within 0.01 m this many wall s is cancelled, backed off straight by hand once and "
                    "the cycle goes on; a recovery that cannot be made ends the session. 0: off")
    ap.add_argument("--stall-backoff-m", type=float, default=0.30,
                    help="the stall recovery's hand reverse: from 0.05 m to this many metres (measured by the speed)")
    ap.add_argument("--manual-backoffs", type=int, default=3,
                    help="manual recovery: how often to back off further and ask again when the preparation cannot "
                    "start from where the recovery ended (invalid_start)")
    ap.add_argument("--manual-stroke-s", type=float, default=1.5,
                    help="manual recovery: each straight reverse stroke before standing for the verdict (wall s)")
    ap.add_argument("--lift-up-s", type=float, default=6.0, help="lift scenario: hold the lift up this long (wall s)")
    ap.add_argument("--quiet-s", type=float, default=8.0, help="lift scenario: the client quiet / runner stopped (s)")
    ap.add_argument("--no-sigstop", action="store_true",
                    help="lift scenario: skip the runner SIGSTOP part (until main's SLAM link stall fix)")
    ap.add_argument("--queue-resume-not-stopped", action="store_true",
                    help="superseded and refused: the queue itself now submits the session's first item once more "
                         "after a not_stopped refusal and a 1 s stand (plan D5 exception, user decision 2026-10-25); "
                         "the runs that used this operator resume (CT-2 V3 seed 2 reruns, leg-failures V4) ran "
                         "snapshots from before it")
    ap.add_argument("--cycle", default="P-1:D-1,P-1:D-0,P-1:D-1", help="cycle / queue scenario: pallet:destination,...; queue: \"auto\" for one all-remaining-pallets item (CT-2)")
    ap.add_argument("--pre-teleop-s", type=float, default=0.0, help="cycle scenario: drive by hand first (wall s)")
    ap.add_argument("--end-session", action="store_true",
                    help="cycle, queue, main, confirm, marker and approach scenarios: end the session at the end (main: validation v2 H1; marker and approach: realtime plan 5.17, job 16866)")
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
    ap.add_argument("--launcher-pid", type=int, default=None,
                    help="the session launcher's pid: its end before the session runs ends the probe at once (exit 3)")
    ap.add_argument("--block-expect", choices=("block", "allow"), default="block",
                    help="place scenario with --after-place block_check (plan 2026-11-08 5): what the vehicle's "
                    "F1 setting is, and so what the check expects")
    ap.add_argument("--block-teleop-s", type=float, default=3.0,
                    help="block_check: the teleop drive's wall seconds (0.15 m/s ratio, straight)")
    ap.add_argument("--after-place", choices=("hold", "reverse", "block_check"), default="hold",
                    help="place scenario: once localized, the hold (as before) or the reverse scenario's strokes")
    ap.add_argument("--reverse-kind", choices=("full", "half", "chatter"), default="full",
                    help="reverse scenario (plan 2026-11-06 2.1): when a stroke lets go")
    ap.add_argument("--mission-after-confirm", action="store_true",
                    help="reverse scenario: once confirmed (the shadow: after the strokes), the first --cycle leg")
    ap.add_argument("--reverse-wait-s", type=float, default=300.0,
                    help="reverse scenario: the start check's and each re-match's wait at most (wall s)")
    ap.add_argument("--stroke-wall-s", type=float, default=120.0, help="reverse scenario: one stroke at most (wall s)")
    ap.add_argument("--post-mission-sim-s", type=float, default=0.0,
                    help="reverse scenario: stand this long after the mission (simulated s), or to the end of a "
                    "re-match that started")
    ap.add_argument("--stroke-stuck-s", type=float, default=20.0,
                    help="reverse scenario: let go of a press that has not moved for this long (wall s)")
    ap.add_argument("--hold-sim-s", type=float, default=10.0,
                    help="hold scenario (B-P2d): the stand after the relocalized session runs (simulated s)")
    ap.add_argument("--live-factor", type=float, default=0.4,
                    help="hold scenario: the runner's --live-factor (sim s per wall s), to wait the hold in wall time")
    ap.add_argument("--truth", type=lambda v: tuple(float(c) for c in v.split(",")), default=None,
                    help="place scenario (F1): the placed start x,y,yaw (world = map), the click model's centre")
    ap.add_argument("--click-seed", type=int, default=0, help="place scenario: the click model's seed")
    ap.add_argument("--click-index", type=int, default=0, help="place scenario: the start's index in its list")
    ap.add_argument("--click-kind", default=None, choices=("bay+x", "bay-x", "bay+y", "bay-y", "pos1.5", "yaw0.5",
                                                           "yaw180"),
                    help="place scenario, development only: place this record click (tools/f1_place_eval.py "
                         "STRESS_KINDS) every time instead of the click model's draws -- what a wrong confirmation "
                         "meets after the console")
    ap.add_argument("--place-wait-s", type=float, default=600.0,
                    help="place scenario: how long to act as the operator (wall s; the operator budget ends it too)")
    ap.add_argument("--tour-path", type=Path, default=None, help="tour scenario: tools/survey_tour.py's path (.npz)")
    ap.add_argument("--tour-cruise-mps", type=float, default=0.30)
    ap.add_argument("--tour-stall-s", type=float, default=60.0, help="tour scenario: no progress this long (wall s)")
    ap.add_argument("--tour-pacing-wait-s", type=float, default=900.0,
                    help="tour scenario: no progress this long while teleop is forbidden for pacing (wall s)")
    args = ap.parse_args()
    if not (math.isfinite(args.stall_recover_s) and args.stall_recover_s >= 0):
        ap.error("--stall-recover-s must be a finite number of wall seconds (0: off)")
    if not (math.isfinite(args.stall_backoff_m) and args.stall_backoff_m > 0.05):
        ap.error("--stall-backoff-m must be a finite distance above 0.05 m")
    if args.stall_recover_s > 0 and (args.scenario != "cycle" or args.cancel_at or args.stop_wait_at):
        # the watch runs in the cycle's end wait only; with --cancel-at or --stop-wait-at their waits come first and
        # it would not watch them (Codex probe review)
        ap.error("--stall-recover-s goes with --scenario cycle, not with --cancel-at or --stop-wait-at")
    if args.queue_resume_not_stopped:
        # both would resubmit after the same refusal
        ap.error("--queue-resume-not-stopped is superseded by the queue's own start resubmit (plan D5 exception)")
    # The job's timeout ends a stuck probe with SIGTERM: leave through the finally below, so the report is written.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(124))
    if args.scenario == "place" and args.truth is None:
        ap.error("the place scenario needs --truth x,y,yaw")
    info, why = wait_port(args.port_file, args.wait_port_s, args.launcher_pid,
                          states=RELOC_PORT_STATES if args.scenario == "place" else ("running",))
    if info is None:
        args.report.write_text(json.dumps({"port_file": None, "error": why}, indent=1) + "\n")
        print(f"probe: no running session ({why})", file=sys.stderr)
        sys.exit(3)
    report = {"port_file": info}
    p = Probe(info["port"], report)
    try:
        scenario = {"main": main_scenario, "slow": slow_scenario, "marker": marker_scenario,
                    "approach": approach_scenario, "planwait": planwait_scenario, "cycle": cycle_scenario,
                    "queue": queue_scenario,
                    "lift": lift_scenario, "reset": reset_scenario, "restart": restart_scenario,
                    "tour": tour_scenario, "hold": hold_scenario, "place": place_scenario,
                    "reverse": reverse_scenario, "confirm": confirm_scenario}[args.scenario]
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
    # plan 2026-11-08 5: a block check that did not pass, or never ran (an epoch changed, no placement), fails the run
    if args.scenario == "place" and args.after_place == "block_check" and (
            (report.get("block_check") or {}).get("verdict") != "pass"):
        sys.exit(5)


if __name__ == "__main__":
    main()
