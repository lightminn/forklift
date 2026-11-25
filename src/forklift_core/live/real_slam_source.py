"""The executor's real SLAM source (docs/design/2026-11-25-real-sources-pr7.md 6.1, items 0-4; 7c-1b-2a).

``RealSlamSource`` keeps the loop's epoch contract (``relocalize``, ``seed``, ``request_processed``, ``replies``, and
7c-1b-1's ``can_confirm``, ``can_seed``, ``seed_state``) on a real-mode bridge (``forklift_ros.real_slam_bridge``,
7c-1a) reached through ``RealSlamLink``, a non-blocking stream socket speaking ``slam_real_link``'s frames.

The scan path (6.1, 1): ``offer(SlamScanIn)`` takes each scan the feed accepted. A scan goes to the bridge only with
the odometry over its whole first-beam interval (``slam_real_scan.pick_odom``: tried once more a tick later, else
dropped ``slam_no_odom``), its stamp above every stamp sent this session and -- to a seed's instance -- its interval
starting at or after the seed's acceptance. One send slot (the scan in flight) and one wait slot (the newest scan: an
older one waiting is dropped ``slam_superseded``); when the send slot empties the newest waiting scan goes at once. A
new producer generation cuts what waits, and the reply of a scan sent from the earlier one only frees the slot.

The link (6.1, 2):
- the session's start (``hello``) is in progress from construction and goes at the first poll; a control frame goes
  only whole and only with no scan in flight; no scan goes before ``state(id, ready_for_scans)``, and after a start
  check's seven scans none goes before its end state;
- a seed is accepted at once (``seed`` returns its id) and written when the slot empties; ``seeding`` until its end
  state (``ready`` / ``not_localized``) or ``void``;
- a seed's inputs are judged until its end state -- each poll and each offer (before any write retry), before each
  scan to its instance and before its end state is taken: a boot change, a raised odometry continuity generation (the
  loop's ``odom_gen``), an entry since the acceptance not continuous, or a move past the limit, seen on
  any odometry history entry made since the acceptance (``odom_step``, the loop's ``odom_listeners``: kept even after
  the entry left the history), or any odometry pose since the acceptance past ``SEED_STILL_M`` / ``SEED_STILL_RAD`` from the
  snapshot cancels it. A seed the bridge has not seen a byte of is cancelled here (``void`` at once); otherwise new
  scans stop, the frame being written is finished and ``cancel`` is written once. A ``ready`` / ``not_localized`` /
  ``reply`` of a cancelled or voided seed, or of a stopped instance, is dropped and counted;
- the deadlines are the executor's, judged before the frames are taken: a start or seed with no end state delivered
  ``SEED_WAIT_S`` after its acceptance (on the clock before the read; only a cancel decided within it lifts it), a
  cancel not written whole within ``CANCEL_SEND_S`` of its decision, no ``void`` within ``VOID_WAIT_S`` of the
  cancel's send or no reply within ``REPLY_WAIT_S`` of a scan to a localized instance (on the clock after the read), or
  a frame not written whole within ``WRITE_WAIT_S`` of its first write (before every retry and at its last byte) ends
  the run (``SlamSourceFatal``; the run's guard marks ``fatal_source``); so does a socket error,
  EOF, a malformed or oversized frame, a ``fault``, a state for another id, a void with no cancel, an end state before
  the seventh start reply, a ``skipped`` from a localized instance, a reply that is not the scan in flight's or carries
  a non-finite correction.

The mode (7a-3 plan 3), chosen at construction: ``operating`` sends only scans with a first-beam interval (the
per-frame judgement ``lidar_cfg.frame_bound``, or the HIL's exact cast) and counts only those in a start check;
``diagnostic`` sends every scan and hands no reply over, even of a scan with an interval. A session changes mode only
by starting again (a new instance).

A producer generation change (``generation``, told for every feed scan before it is judged -- plan 3, Codex round 3
P2-3) cuts what waits and makes the scan in flight's reply only free the slot; it also voids a check in progress
(round 4 P2-3): during the session's first start check, once a scan of the earlier generation went into it, the
session ends (``start_check_generation_changed``: the bridge takes no second hello on its one connection, round 5);
a seed in progress is cancelled (``generation_changed``: cancel, void). Only a check begun on the new generation's
scans can then open the confirmation.

The replies (6.1, 0 and 4): only a localized instance's replies of measured scans reach the loop, each with the order
stamp ``stamp_s`` (core seconds of the first-beam estimate), the evidence interval ``evidence_lo_s`` /
``evidence_hi_s``, the epoch it was acquired in (the current one only if its ``Flo`` is at or after the epoch's start
at this source) and a forced request's attempt -- given only to a measured scan, and passed on to the next one when
its reply is dropped. A reply received more than ``LATE_S`` after its ``Flo`` is dropped (``slam_late``). A
diagnostic scan (no measured interval) is sent and its reply recorded, never handed over.
"""

from __future__ import annotations

import math
import socket
import struct
import time
from collections import Counter
from types import SimpleNamespace

from forklift_core.localization import slam_link
from forklift_core.localization import slam_real_link as rl
from forklift_core.localization import slam_real_scan as srs
from forklift_core.localization.start_reloc import base_from_rear

SEED_WAIT_S = 20.0  # a start or seed's end state from its acceptance (provisional; bench item 4 on the Jetson)
CANCEL_SEND_S = 1.0  # a cancel decided -> its frame out whole
VOID_WAIT_S = 5.0  # a cancel sent -> the bridge's void (stop, reap, graph)
REPLY_WAIT_S = 1.0  # a scan to a localized instance -> its reply
WRITE_WAIT_S = 1.0  # any frame, from its first write to its last byte
LATE_S = 0.25  # a reply's receipt at most this after its first beam's earliest instant (design 6.1, 4)
REQUEST_S = 3.0  # a forced request's life from its acceptance (the confirmation's limit)
ODOM_RETRY_NS = 20_000_000  # one tick: a scan whose odometry is not in yet is tried once more after this
SEED_STILL_M = 0.01  # a seed's snapshot -> any odometry pose since beyond this is a move (provisional)
SEED_STILL_RAD = math.radians(1.0)
START_SCANS = 7  # a start check's scans (the bridge's STARTUP_SCANS)
MODES = ("diagnostic", "operating")
MAX_FRAME = 1 << 20  # the largest frame accepted from the bridge
READ_BYTES = 1 << 18  # read per poll at most (the rest next poll)
READ_FRAMES = 64  # frames taken per poll at most


class SlamSourceFatal(RuntimeError):
    """The real SLAM link cannot go on (the run's guard marks ``fatal_source``)."""


class RealSlamLink:
    """The executor end of the bridge's stream socket: one frame written at a time, never interleaved, its completion
    called exactly once; a frame not one byte of which went may be taken back. Frames read whole, bounded per poll.
    Never blocks."""

    def __init__(self, sock: socket.socket, *, clock=time.monotonic_ns) -> None:
        self.sock, self.clock = sock, clock
        sock.setblocking(False)
        self._out = b""
        self._sent = 0
        self.out_kind: str | None = None
        self._done = None
        self._gate = None
        self.started_ns: int | None = None
        self._buffer = b""
        self.closed = False

    @property
    def busy(self) -> bool:
        return bool(self._out)

    def write(self, frame: bytes, kind: str, done=None, gate=None) -> bool:
        """``gate()`` runs right before the frame's first byte goes (each try while none has gone) and may raise."""
        if self._out:
            raise SlamSourceFatal("link: a frame written over another")
        self._out, self._sent, self.out_kind, self._done, self._gate = frame, 0, kind, done, gate
        self.started_ns = int(self.clock())
        return self.flush()

    def flush(self) -> bool:
        """True once nothing is left to write; the frame's completion is called once, when its last byte goes. A frame
        past ``WRITE_WAIT_S`` from its first write is a fault before any retry and at its completion (round 2 P2-2)."""
        if self._out and self.overdue(int(self.clock())):
            raise SlamSourceFatal(f"link: a {self.out_kind} frame not written whole within {WRITE_WAIT_S} s")
        while self._out:
            if self._sent == 0 and self._gate is not None:
                self._gate()  # the frame's deadlines on the clock right before its first byte (round 6 P2)
            try:
                n = self.sock.send(self._out)
            except (BlockingIOError, InterruptedError):
                return False
            except OSError as exc:
                raise SlamSourceFatal(f"link: send: {exc}") from exc
            self._out = self._out[n:]
            self._sent += n
        if self.started_ns is not None and self.overdue(int(self.clock())):
            raise SlamSourceFatal(f"link: a {self.out_kind} frame not written whole within {WRITE_WAIT_S} s")
        done, self._done, self._gate = self._done, None, None
        self.out_kind, self.started_ns = None, None
        if done is not None:
            done()
        return True

    def retract(self, kind: str) -> bool:
        """Take back the frame being written if it is ``kind`` and not one byte of it went."""
        if self._out and self.out_kind == kind and self._sent == 0:
            if self.overdue(int(self.clock())):
                raise SlamSourceFatal(f"link: a {kind} frame not written whole within {WRITE_WAIT_S} s")
            self._out, self.out_kind, self._done, self._gate, self.started_ns = b"", None, None, None, None
            return True
        return False

    def overdue(self, now_ns: int) -> bool:
        return self.started_ns is not None and now_ns - self.started_ns > WRITE_WAIT_S * 1e9

    def _parse(self, out: list) -> None:
        """Take whole frames from the buffer (one slice at the end, not one per frame -- round 2 P2-8)."""
        buf, pos = self._buffer, 0
        while len(buf) - pos >= 4 and len(out) < READ_FRAMES:
            (length,) = struct.unpack(">I", buf[pos:pos + 4])
            if length > MAX_FRAME:
                raise SlamSourceFatal(f"link: a frame of {length} bytes")
            if len(buf) - pos < 4 + length:
                break
            body = bytes(buf[pos + 4:pos + 4 + length])
            pos += 4 + length
            try:
                out.append(rl.decode(body))
            except ValueError as exc:
                raise SlamSourceFatal(f"link: a malformed frame: {exc}") from exc
        if pos:
            self._buffer = buf[pos:]

    def read(self) -> list:
        """The whole frames received (at most ``READ_FRAMES``, the buffered ones first), decoded; at most
        ``READ_BYTES`` read per poll; EOF is reported after them (``closed``)."""
        out: list = []
        self._parse(out)
        budget = READ_BYTES
        try:
            while budget > 0 and len(out) < READ_FRAMES:
                chunk = self.sock.recv(min(65536, budget))
                if not chunk:
                    self.closed = True
                    break
                self._buffer += chunk
                budget -= len(chunk)
                self._parse(out)
        except (BlockingIOError, InterruptedError):
            pass
        except OSError as exc:
            raise SlamSourceFatal(f"link: recv: {exc}") from exc
        if len(self._buffer) > MAX_FRAME + READ_BYTES:
            raise SlamSourceFatal(f"link: {len(self._buffer)} bytes behind")
        return out


def _finite_pose(v) -> bool:
    return (isinstance(v, (list, tuple)) and len(v) == 3
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v))


class RealSlamSource:
    """See the module text. ``odom_history()``: the loop's odometry history (core seconds); ``odom_base()``: the
    odometry base_link pose now; ``odom_gen()``: the loop's odometry continuity generation (raised at every break,
    reboot and device loss; a break clears the history, so the history alone cannot show it); ``boot()``: the MCU boot now; ``start_pose``: map <- base_link of the
    registered start (the session's first instance); ``mode``: ``diagnostic`` or ``operating`` (no default: each caller
    names it, 7a-3 plan 3)."""

    def __init__(self, link: RealSlamLink, *, mode: str, map_id: str, start_ns: int, start_pose, odom_history,
                 odom_base, odom_gen, boot, rear_offset_m: float, session_epoch=0, clock=time.monotonic_ns,
                 events=None) -> None:
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, not {mode!r}")
        self.mode = mode
        self.link, self.map_id, self.start_ns = link, str(map_id), int(start_ns)
        self.start_pose = tuple(float(v) for v in start_pose)
        self.odom_history, self.odom_base, self.odom_gen, self.boot = odom_history, odom_base, odom_gen, boot
        # session_epoch: an int, or a callable read when the hello goes (the run's arbiter is made after the sources)
        self.rear_offset_m, self.session_epoch, self.clock = float(rear_offset_m), session_epoch, clock
        self.events = events if events is not None else []  # the record (observe run, 6.1, 8)
        self.counts: Counter = Counter()
        self.epoch, self._prev_epoch = 0, 0
        self._epoch_start_ns = -(1 << 62)
        self._next_id = 2
        # the session's start, in progress from now on (its acceptance and deadline at the first poll, when it goes)
        self.target: dict | None = self._target(1, "start", self.start_pose, None, None)
        self.seeds: dict = {}  # epoch -> (kind, reason): seed_state's answers
        self._ended: set = set()  # ids of seeds cancelled or voided: their late frames are counted, never taken
        self._discarded: set = set()  # (scan_id, stamp_s) of scans in flight dropped with a void
        self._inflight: dict | None = None
        self._waiting: dict | None = None  # the newest scan ready to send
        self._odom_wait: dict | None = None  # a scan whose odometry is not in yet (one retry)
        self._requests: list = []  # [epoch, attempt, not_before_s, expires_ns]
        self._out: list = []
        self._last_stamp_ns: int | None = None
        self._scan_id = 0
        self._gen = None
        # (bridge scan, reply record) -> None: every reply of a scan sent whole, before its verdict -- the observe
        # run's match quality (PR 7 design 6.1, 8); never raises into the source (its errors are counted)
        self.reply_hook = None

    @staticmethod
    def _target(ident, kind, pose, accepted_ns, epoch, snapshot=None, boot=None, odom_gen=None) -> dict:
        return {"id": int(ident), "kind": kind, "epoch": epoch, "accepted_ns": accepted_ns,
                "deadline_ns": None if accepted_ns is None else accepted_ns + int(SEED_WAIT_S * 1e9),
                "pose": tuple(float(v) for v in pose), "snapshot": snapshot, "boot": boot, "odom_gen": odom_gen,
                "state": "queued", "cancel": None, "start_scans": 0, "moved": False, "broken": False}

    # --- helpers -----------------------------------------------------------------------------------------------------
    def _s(self, ns: int) -> float:
        return (int(ns) - self.start_ns) / 1e9

    def _event(self, **kw) -> None:
        self.events.append({"ns": int(self.clock()), **kw})

    def _drop(self, why: str, **kw) -> None:
        self.counts[why] += 1
        self._event(t="slam_drop", why=why, **kw)

    def _fatal(self, why: str):
        self._event(t="slam_fatal", why=why)
        raise SlamSourceFatal(why)

    def _seeding(self) -> bool:
        t = self.target
        return t is not None and t["state"] in ("queued", "sent", "seeding", "scans")

    def _reinstate(self, attempt, now: int) -> None:
        if attempt is not None and now <= attempt[3] and attempt[0] == self.epoch:
            self._requests.insert(0, attempt)

    # --- the loop's contract -----------------------------------------------------------------------------------------
    def can_confirm(self) -> bool:
        t = self.target
        return t is not None and t["state"] == "ready" and t["cancel"] is None

    def can_seed(self) -> bool:
        t = self.target
        return not self._seeding() and not (t is not None and t["cancel"] is not None)

    def seed_state(self, epoch):
        return self.seeds.get(int(epoch))

    def relocalize(self, epoch) -> None:
        self._new_epoch(int(epoch))

    def _new_epoch(self, epoch: int) -> None:
        self._prev_epoch, self.epoch = self.epoch, int(epoch)
        self._epoch_start_ns = int(self.clock())
        if self._waiting is not None:
            self._drop("slam_epoch_cut", key=self._waiting["key"])
        self._waiting, self._odom_wait = None, None
        self._requests = [r for r in self._requests if r[0] >= self.epoch]

    def seed(self, epoch, base_pose, stamp_s):
        """Accepted at once (6.1, 2): the new epoch, the odometry snapshot, the frame written once the slot is empty.
        Returns the seed id (the frame's)."""
        if not self.can_seed():
            self._fatal("seed while a start or seed has no end state")  # the loop asks can_seed() first
        now = int(self.clock())
        self._new_epoch(int(epoch))
        sid = self._next_id
        self._next_id += 1
        self.target = self._target(sid, "seed", base_pose, now, self.epoch,
                                   tuple(float(v) for v in self.odom_base()), self.boot(), self.odom_gen())
        self.seeds[self.epoch] = ("seeding", None)
        self._event(t="seed_accepted", id=sid, epoch=self.epoch, stamp_s=float(stamp_s))
        self._pump(now)
        return sid

    def odom_step(self, entry) -> None:
        """The loop's every odometry history entry as it is made (its ``odom_listeners``): for a seed in progress, an
        entry since its acceptance that breaks continuity, or a pose past the stillness limit, is kept until its end
        state, whether or not a poll still finds the entry in the history (rounds 3-4). An entry left out of scan
        placement for its STATE's delay bound (``included`` False) is still a continuous odometry pose: the seed's
        inputs are the snapshot, the boot and the continuity (design 6.1, 2), not the placement's time precision --
        normal HIL running has such STATEs (7c-1b-2b pty run). Never raises."""
        t = self.target
        if t is None or t["kind"] != "seed" or t["snapshot"] is None or not self._seeding():
            return
        if float(entry[1]) < self._s(t["accepted_ns"]):
            return
        if not entry[3]:
            t["broken"] = True
        if self._past_still(base_from_rear(entry[2], self.rear_offset_m), t["snapshot"]):
            t["moved"] = True

    @staticmethod
    def _past_still(pose, snapshot) -> bool:
        x, y, yaw = (float(v) for v in pose)
        sx, sy, syaw = snapshot
        dyaw = abs(math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw)))
        return math.hypot(x - sx, y - sy) > SEED_STILL_M or dyaw > SEED_STILL_RAD

    def input_break(self, why: str) -> None:
        """The odometry this source places scans on broke (the observe run's reconnect, boot change or continuity
        break -- the loop opens a relocalization instead): what waits is cut, the scan in flight's reply only frees the
        slot, as for a new producer generation (7c-1c, Codex round 1 P2-1)."""
        for w in (self._waiting, None if self._odom_wait is None else self._odom_wait["scan"]):
            if w is not None:
                self._drop("slam_input_break", key=w["key"] if isinstance(w, dict) else w.key, why_break=why)
        self._waiting, self._odom_wait = None, None
        f = self._inflight
        if f is not None and self.link.retract("scan"):
            # not one byte of it went: never sent at all (7c-1c round 2 P2-1), and not one of a start check's seven
            self._inflight = None
            self._last_stamp_ns = f["prev_stamp_ns"]
            t = self.target
            if t is not None and t["id"] == f["target"] and f["phase"] == "scans":
                t["start_scans"] -= 1
            self._drop("slam_input_break", key=f["key"], why_break=why)
        elif f is not None:
            f["old_gen"] = True

    def request_processed(self, epoch, attempt, not_before) -> None:
        if int(epoch) >= self.epoch:
            self._requests.append([int(epoch), attempt, float(not_before), int(self.clock()) + int(REQUEST_S * 1e9)])

    # --- the scan path -----------------------------------------------------------------------------------------------
    def generation(self, gen) -> None:
        """A feed scan's producer generation, told before the scan is judged (an unqualified scan still changes it)."""
        gen = tuple(gen)
        if self._gen is not None and gen != self._gen:
            self._gen = gen
            self._generation_changed(gen)
        self._gen = gen

    def _generation_changed(self, gen: tuple) -> None:
        now = int(self.clock())
        self._event(t="generation_changed", gen=[None if g is None else int(g) if isinstance(g, int) else str(g)
                                                 for g in gen])
        # what waits is cut, the scan in flight's reply only frees the slot
        for w in (self._waiting, None if self._odom_wait is None else self._odom_wait["scan"]):
            if w is not None:
                self._drop("slam_generation_cut", key=w["key"] if isinstance(w, dict) else w.key)
        self._waiting, self._odom_wait = None, None
        t = self.target
        f = self._inflight
        if f is not None and self.link.retract("scan"):
            # not one byte of it went: never sent, and not one of a start check's seven (Codex 7a-3a round 1 P3-1);
            # nor the newest stamp sent (round 2 P3)
            self._inflight = None
            self._last_stamp_ns = f["prev_stamp_ns"]
            if t is not None and t["id"] == f["target"] and f["phase"] == "scans":
                t["start_scans"] -= 1
            self._drop("slam_generation_cut", key=f["key"])
            self._reinstate(f["attempt"], now)  # a forced request it carried goes on, as with an old reply
        elif f is not None:
            f["old_gen"] = True
        if t is None or t["cancel"] is not None or not self._seeding():
            return  # after a check's end state: only what waits and the reply in flight (as before)
        if t["kind"] == "start":
            if t["start_scans"] > 0:
                # the bridge's start check has scans of the earlier generation and no second hello on this connection
                # (round 5): the session ends, never confirmed on them
                t["state"] = "generation_changed"
                self._fatal("start_check_generation_changed")
            return  # no scan of the earlier generation went into it: its seven come from the new one
        self._decide_cancel(t, "generation_changed", now)

    def offer(self, s: srs.SlamScanIn) -> None:
        now = int(self.clock())
        self._all_deadlines(now)  # an expired deadline is never lifted by what an offer does (rounds 4-5)
        self._watch_seed(now)  # before any write retry: a seed not one byte of which went is taken back (round 2 P2-1)
        self.generation(s.gen)
        if self.mode == "operating" and not s.measured:
            self.counts["slam_not_qualified"] += 1  # counted, no event: a pre-bound real feed sends 10 a second
            return
        if self._odom_wait is not None and s.stamp_ns <= self._odom_wait["scan"].stamp_ns:
            return self._drop("slam_stamp_order", key=s.key)  # the fresher scan keeps its retry (round 3 P2-4)
        if self._odom_wait is not None:
            self._drop("slam_superseded", key=self._odom_wait["scan"].key)
        self._odom_wait = {"scan": s, "retry_ns": now + ODOM_RETRY_NS, "tried": False}
        self._try_odom(now)
        self._pump(now)

    def _try_odom(self, now: int) -> None:
        w = self._odom_wait
        if w is None:
            return
        s = w["scan"]
        if w["tried"] and now < w["retry_ns"]:
            return
        lo, hi, t = self._s(s.flo_ns), self._s(s.fhi_ns), self._s(s.stamp_ns)
        rear, why = srs.pick_odom(self.odom_history(), lo, hi, t)
        if rear is None and why == "not_yet" and not w["tried"]:
            w["tried"] = True
            return
        self._odom_wait = None
        if rear is None:
            return self._drop("slam_no_odom", key=s.key, why_odom=why)
        self._prepare(s, base_from_rear(rear, self.rear_offset_m))

    def _prepare(self, s: srs.SlamScanIn, odom_from_base) -> None:
        if self._last_stamp_ns is not None and s.stamp_ns <= self._last_stamp_ns:
            return self._drop("slam_stamp_order", key=s.key)
        if self._waiting is not None and s.stamp_ns <= self._waiting["stamp_ns"]:
            return self._drop("slam_stamp_order", key=s.key)  # never a fresher waiting scan for an older (P2-7)
        self._scan_id += 1
        scan, why = srs.to_bridge_scan(s, self._scan_id, odom_from_base)
        if scan is None:
            return self._drop(f"slam_{why}", key=s.key)
        epoch = self.epoch if s.flo_ns >= self._epoch_start_ns else self._prev_epoch
        if self._waiting is not None:
            self._drop("slam_superseded", key=self._waiting["key"])
        self._waiting = {"scan": scan, "stamp_ns": int(s.stamp_ns), "flo_ns": int(s.flo_ns), "fhi_ns": int(s.fhi_ns),
                         "measured": bool(s.measured), "epoch": epoch, "key": s.key,
                         "odom_from_base": tuple(odom_from_base)}

    # --- sending -----------------------------------------------------------------------------------------------------
    def _pump(self, now: int) -> None:
        link = self.link
        if not link.flush():
            return  # the frame being written first, whatever comes next
        t = self.target
        if t is not None and t["cancel"] is not None:
            c = t["cancel"]
            if not c["writing"]:
                c["writing"] = True

                def sent(c=c, ident=t["id"]):
                    c["sent_ns"] = int(self.clock())
                    if c["sent_ns"] - c["decided_ns"] > CANCEL_SEND_S * 1e9:  # its last byte late (round 2 P2-2)
                        self._fatal(f"cancel of {ident} not sent within {CANCEL_SEND_S} s")
                    self._event(t="cancel_sent", id=ident)

                link.write(rl.encode_cancel(rl.Cancel(t["id"], c["reason"])), "cancel", sent, self._gate_now)
            return
        if self._inflight is not None:
            return
        if t is not None and t["state"] == "queued":
            if t["accepted_ns"] is None:
                return  # the session's start goes at the first poll
            if t["kind"] == "start":
                epoch = self.session_epoch() if callable(self.session_epoch) else self.session_epoch
                frame = rl.encode_hello(rl.Hello(int(epoch), self.map_id, t["id"], t["pose"], t["deadline_ns"]))
            else:
                frame = rl.encode_seed(rl.Seed(t["id"], t["epoch"], t["pose"], t["snapshot"], int(t["boot"]),
                                               t["deadline_ns"]))
            self._all_deadlines(int(self.clock()))  # right before its first byte (round 5 P2-2)
            t["state"] = "sent"
            link.write(frame, t["kind"], gate=self._gate_now)
            return
        if t is None or t["state"] not in ("scans", "ready") or self._waiting is None:
            return
        if t["state"] == "scans" and t["start_scans"] >= START_SCANS:
            return  # the start check has its seven: nothing more before its end state (round 1 P1-3)
        w = self._waiting
        if t["kind"] == "seed" and w["flo_ns"] < t["accepted_ns"]:
            self._waiting = None
            return self._drop("slam_before_seed", key=w["key"])  # while the seed's instance lives (round 1 P3)
        if t["kind"] == "seed" and t["state"] == "scans":
            why = self._input_broken(t)
            if why is not None:
                return self._decide_cancel(t, why, now)
        self._all_deadlines(int(self.clock()))  # right before its first byte: never a start scan past 20 s (P2-2)
        self._waiting = None
        attempt = None
        if t["state"] == "ready" and w["measured"] and self.mode == "operating" and w["epoch"] == self.epoch:
            for req in list(self._requests):
                if req[0] == self.epoch and self._s(w["flo_ns"]) >= req[2] and now <= req[3]:
                    self._requests.remove(req)
                    attempt = req
                    break
        f = {**w, "target": t["id"], "phase": t["state"], "attempt": attempt, "sent_ns": None, "old_gen": False,
             "prev_stamp_ns": self._last_stamp_ns}  # restored if not one byte of it goes (round 2 P3)
        self._last_stamp_ns = w["stamp_ns"]
        self._inflight = f
        if t["state"] == "scans":
            t["start_scans"] += 1

        def sent(f=f, w=w, ident=t["id"], phase=t["state"]):
            f["sent_ns"] = int(self.clock())
            # recorded at its last byte: what the bridge received (round 4 P3)
            self._event(t="scan_sent", key=w["key"], scan_id=w["scan"].scan_id, stamp_ns=w["stamp_ns"],
                        flo_ns=w["flo_ns"], fhi_ns=w["fhi_ns"], measured=w["measured"],
                        odom_from_base=list(w["odom_from_base"]), target=ident, phase=phase)

        link.write(rl.encode_scan(w["scan"], w["stamp_ns"]), "scan", sent, self._gate_now)

    # --- receiving ---------------------------------------------------------------------------------------------------
    def replies(self, now_ns: int) -> list:
        t = self.target
        if t is not None and t["kind"] == "start" and t["accepted_ns"] is None:
            now = int(self.clock())
            t["accepted_ns"], t["deadline_ns"] = now, now + int(SEED_WAIT_S * 1e9)
        looked = int(self.clock())  # before the read: what is read arrived at or after it
        self._deadlines(looked)
        self._watch_seed(looked)  # before the write retry (round 2 P2-1)
        self.link.flush()
        frames = self.link.read()
        received = int(self.clock())
        self._receipt_deadlines(received, frames)
        for obj in frames:
            if isinstance(obj, slam_link.Reply):
                self._on_reply(obj, received)
            elif isinstance(obj, rl.State):
                self._on_state(obj, received)
            else:
                self._fatal(f"protocol: unexpected frame {type(obj).__name__} from the bridge")
        if self.link.closed:
            self._fatal("the bridge closed the connection")
        self._watch_seed(looked)
        self._try_odom(looked)
        self._pump(looked)
        out, self._out = self._out, []
        return out

    def _deadlines(self, now: int) -> None:
        """The executor's deadlines on the clock before the read, before any frame is taken: an end state delivered
        past the start or seed's deadline is no end state (design 6.1, 2 -- 'from the acceptance to the end state's
        delivery'; Codex 7c-1b-2a round 2 P2-3)."""
        if self.link.overdue(now):
            self._fatal(f"a {self.link.out_kind} frame not written whole within {WRITE_WAIT_S} s")
        t = self.target
        if t is not None and t["cancel"] is not None:
            c = t["cancel"]
            if c["sent_ns"] is None and now - c["decided_ns"] > CANCEL_SEND_S * 1e9:
                self._fatal(f"cancel of {t['id']} not sent within {CANCEL_SEND_S} s")
        elif self._seeding() and t["deadline_ns"] is not None and now > t["deadline_ns"]:
            self._fatal(f"{t['kind']} {t['id']}: no end state within {SEED_WAIT_S} s")

    def _gate_now(self) -> None:
        self._all_deadlines(int(self.clock()))

    def _all_deadlines(self, now: int) -> None:
        """Every deadline on the clock now: outside a poll (an offer, a write) nothing has been read since, so a bound
        on the bridge's answer that has passed is past (round 5 P2-1)."""
        self._deadlines(now)
        self._receipt_deadlines(now)

    def _receipt_deadlines(self, received: int, frames=()) -> None:
        """What is waited for from the bridge, on the clock after the read, before any frame is taken: a void, a reply
        or a start / seed's end state read past its bound is past it (round 2 P2-4, round 3 P2-2)."""
        t = self.target
        if t is not None and t["cancel"] is None and self._seeding() and t["deadline_ns"] is not None \
                and received > t["deadline_ns"]:
            self._fatal(f"{t['kind']} {t['id']}: no end state within {SEED_WAIT_S} s")
        if t is not None and t["cancel"] is not None and t["cancel"]["sent_ns"] is not None \
                and received - t["cancel"]["sent_ns"] > VOID_WAIT_S * 1e9:
            self._fatal(f"no void for {t['id']} within {VOID_WAIT_S} s of its cancel")
        f = self._inflight
        if (f is not None and f["sent_ns"] is not None and f["phase"] == "ready"
                and received - f["sent_ns"] > REPLY_WAIT_S * 1e9):
            for r in frames:  # its reply read past the bound: recorded before the fault (7c-1c round 3 P2)
                if isinstance(r, slam_link.Reply) and self._reply_key_ok(r) \
                        and (int(r.scan_id), float(r.stamp_s)) == (f["scan"].scan_id, float(f["scan"].stamp_s)):
                    self._event(**self._reply_record(r, f, received), passed=False, why="past_reply_bound")
                    break
            self._fatal(f"scan {f['scan'].scan_id}: no reply within {REPLY_WAIT_S} s")

    def _reply_record(self, r, f, received: int) -> dict:
        """The reply's record: its scan in flight's identity, epoch and attempt, and the reply hook's diagnostic."""
        rec = {"t": "reply", "scan_id": int(r.scan_id), "status": r.status, "map_from_odom": list(r.map_from_odom),
               "received_ns": received, "flo_ns": f["flo_ns"], "target": f["target"], "phase": f["phase"],
               "epoch": f["epoch"], "attempt": None if f["attempt"] is None else f["attempt"][1]}
        if self.reply_hook is not None:  # every reply of a scan sent whole, whatever its verdict (7c-1c round 2 P2-3)
            try:
                extra = self.reply_hook(f["scan"], rec)
                if extra:
                    rec.update(extra)
            except Exception as exc:  # noqa: BLE001 -- a diagnostic never ends the source
                self.counts["reply_hook_errors"] += 1
                rec["hook_error"] = f"{type(exc).__name__}: {exc}"
        return rec

    @staticmethod
    def _reply_key_ok(r) -> bool:
        """An integer scan id, a finite stamp and three finite correction numbers -- one check for both paths that match
        a reply to the scan in flight (round 1 P2-6; 7c-1c round 4 P2)."""
        try:
            return (not isinstance(r.scan_id, bool) and isinstance(r.scan_id, int) and not isinstance(r.stamp_s, bool)
                    and isinstance(r.stamp_s, (int, float)) and math.isfinite(r.stamp_s)
                    and _finite_pose(r.map_from_odom))
        except (OverflowError, TypeError, ValueError):
            return False  # a number past a float's range is no reply of ours (7c-1c round 5 P3)

    def _on_reply(self, r, received: int) -> None:
        if not self._reply_key_ok(r):
            self._fatal(f"protocol: a malformed reply {r.scan_id!r}")  # round 1 P2-6
        key = (int(r.scan_id), float(r.stamp_s))
        f = self._inflight
        if f is None or f["sent_ns"] is None or key != (f["scan"].scan_id, float(f["scan"].stamp_s)):
            if key in self._discarded:
                self.counts["slam_discarded_reply"] += 1  # the scan in flight a void dropped (round 1 P2-4)
                self._discarded.discard(key)
                return
            self._fatal(f"protocol: reply for scan {r.scan_id}, not the scan in flight")
        rec = self._reply_record(r, f, received)
        if r.status == "failed":
            self._event(**rec, passed=False, why="failed")
            self._fatal(f"bridge failed scan {r.scan_id}: {r.reason}")
        if r.status != "processed" and f["phase"] == "ready":
            self._event(**rec, passed=False, why="not_processed")
            self._fatal(f"protocol: a {r.status} reply from a localized instance")  # every scan gets a /pose (P2-6)
        self._inflight = None
        t = self.target
        why = None
        if f["old_gen"]:
            why = "old_generation"
        elif t is None or t["id"] != f["target"]:
            why = "old_instance"
        elif t["cancel"] is not None:
            why = "cancelled"
        elif f["phase"] != "ready":
            why = "start_check"
        elif not f["measured"] or self.mode == "diagnostic":
            why = "diagnostic"
        elif received - f["flo_ns"] > LATE_S * 1e9:
            why = "late"
        if why is not None:
            if why in ("old_generation", "old_instance", "cancelled", "late"):
                self.counts[f"slam_{why}_reply" if why != "late" else "slam_late"] += 1
            self._event(**rec, passed=False, why=why)
            self._reinstate(f["attempt"], received)  # a forced request the dropped reply carried goes on
            return
        reply = SimpleNamespace(scan_id=int(r.scan_id), stamp_s=self._s(f["stamp_ns"]), status=r.status,
                                map_from_odom=tuple(float(v) for v in r.map_from_odom), slam_wall_s=None,
                                epoch=f["epoch"], attempt=None if f["attempt"] is None else f["attempt"][1],
                                evidence_lo_s=self._s(f["flo_ns"]), evidence_hi_s=self._s(f["fhi_ns"]))
        self._event(**rec, passed=True)
        self._out.append((reply, {"source": "real_slam"}, f["sent_ns"] / 1e9, received / 1e9))

    def _on_state(self, st, received: int) -> None:
        t = self.target
        self._event(t="state", id=int(st.id), kind=st.kind, reason=st.reason)
        if st.kind == "fault":
            self._fatal(f"bridge fault: {st.reason}")
        if int(st.id) in self._ended and st.kind != "void":
            self.counts["slam_state_after_void"] += 1  # sent before the bridge read the cancel (round 1 P2-4)
            return
        if t is None or int(st.id) != t["id"]:
            self._fatal(f"protocol: state {st.kind} for id {st.id}, not the current start or seed")
        if st.kind == "void":
            if t["cancel"] is None or t["cancel"]["sent_ns"] is None:
                self._fatal(f"protocol: void of {st.id} with no cancel sent")
            f = self._inflight
            if f is not None and f["target"] == t["id"]:
                self._inflight = None  # the seed's scan in flight goes with its instance
                self._discarded.add((f["scan"].scan_id, float(f["scan"].stamp_s)))
            self.seeds[t["epoch"]] = ("void", t["cancel"]["reason"])
            self._ended.add(t["id"])
            self.target = None
            return
        if t["cancel"] is not None:
            self.counts["slam_end_after_cancel"] += 1  # a ready / not_localized sent before the bridge saw the cancel
            return
        if st.kind == "seeding":
            if t["state"] == "sent":
                t["state"] = "seeding"
            return
        if st.kind == "ready_for_scans":
            if t["state"] not in ("sent", "seeding"):
                self._fatal(f"protocol: ready_for_scans in {t['state']}")
            t["state"] = "scans"
            return
        f = self._inflight
        if t["state"] != "scans" or t["start_scans"] != START_SCANS or (f is not None and f["target"] == t["id"]):
            # the bridge answers the seventh scan before its end state (round 2 P2-5)
            self._fatal(f"protocol: {st.kind} in {t['state']} after {t['start_scans']} start scans")
        if t["kind"] == "seed":
            why = self._input_broken(t)  # judged before the end state is taken (round 1 P1-1)
            if why is not None:
                self.counts["slam_end_after_cancel"] += 1
                return self._decide_cancel(t, why, received)
        t["state"] = st.kind  # ready / not_localized
        if t["kind"] == "seed":
            self.seeds[t["epoch"]] = (st.kind, st.reason)

    # --- the seed's inputs -------------------------------------------------------------------------------------------
    def _input_broken(self, t) -> str | None:
        if self.boot() != t["boot"]:
            return "boot_changed"
        if self.odom_gen() != t["odom_gen"]:
            return "odom_break"  # any break since the acceptance, seen or not by a poll (round 2 P1)
        if t["broken"]:
            return "odom_break"
        if t["moved"]:
            return "seed_moved"
        since = self._s(t["accepted_ns"])
        poses = []
        for e in self.odom_history():
            if e[1] < since:
                continue
            if not e[3]:
                return "odom_break"
            poses.append(base_from_rear(e[2], self.rear_offset_m))
        poses.append(tuple(float(v) for v in self.odom_base()))
        for pose in poses:  # every pose since the acceptance still in the history, not the last only (round 1 P1-2)
            if self._past_still(pose, t["snapshot"]):
                return "seed_moved"
        return None

    def _watch_seed(self, now: int) -> None:
        t = self.target
        if t is None or t["kind"] != "seed" or not self._seeding() or t["cancel"] is not None:
            return
        why = self._input_broken(t)
        if why is not None:
            self._decide_cancel(t, why, int(self.clock()))  # the decision's own instant (round 3 P2-2)

    def _decide_cancel(self, t, why: str, now: int) -> None:
        if t["deadline_ns"] is not None and now > t["deadline_ns"]:
            # only a cancel decided within the deadline lifts it (round 2 P2-3)
            self._fatal(f"{t['kind']} {t['id']}: no end state within {SEED_WAIT_S} s")
        self._event(t="cancel_decided", id=t["id"], reason=why, state=t["state"])
        if t["state"] == "queued" or (t["state"] == "sent" and self.link.retract("seed")):
            # the bridge has not seen a byte of it: cancelled here, the bridge never hears of it (round 1 P2-2)
            self.seeds[t["epoch"]] = ("void", why)
            self._ended.add(t["id"])
            self.target = None
            return
        f = self._inflight
        if f is not None and f["target"] == t["id"] and self.link.retract("scan"):
            self._inflight = None  # not one byte of it went: the cancel goes first (round 3 P2-3)
            self._last_stamp_ns = f["prev_stamp_ns"]
            self._drop("slam_cancel_cut", key=f["key"])
        t["cancel"] = {"reason": why, "decided_ns": int(now), "writing": False, "sent_ns": None}
        for w in (self._waiting, None if self._odom_wait is None else self._odom_wait["scan"]):
            if w is not None:
                self._drop("slam_cancel_cut", key=w["key"] if isinstance(w, dict) else w.key)
        self._waiting, self._odom_wait = None, None
        self._pump(now)


__all__ = ["MODES", "RealSlamLink", "RealSlamSource", "SlamSourceFatal"]
