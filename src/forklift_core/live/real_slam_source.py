"""The executor's real SLAM source (docs/design/2026-11-25-real-sources-pr7.md 6.1, items 0-4; 7c-1b-2).

``RealSlamSource`` keeps the loop's epoch contract (``relocalize``, ``seed``, ``request_processed``, ``replies``, and
7c-1b-1's ``can_confirm``, ``can_seed``, ``seed_state``) on a real-mode bridge (``forklift_ros.real_slam_bridge``,
7c-1a) reached through ``RealSlamLink``, a non-blocking stream socket speaking ``slam_real_link``'s frames.

The scan path (6.1, 1): ``offer(SlamScanIn)`` takes each scan the feed accepted. A scan goes to the bridge only with
the odometry over its whole first-beam interval (``slam_real_scan.pick_odom``: tried again once a tick later, else
dropped ``slam_no_odom``), its stamp above every stamp sent this session, and -- for a seed's instance -- its interval
starting at or after the seed's acceptance. One send slot (the scan in flight) and one wait slot (the newest scan:
an older one waiting is dropped ``slam_superseded``); when the send slot empties the newest waiting scan goes at once.

The link (6.1, 2):
- ``hello`` at the session's first poll, then ``seed`` frames; a control frame goes only whole and only with the send
  slot empty, and no scan goes for its id before ``state(id, ready_for_scans)``;
- a seed is accepted at once (``seed`` returns its id) and sent when the slot empties; ``seeding`` until its end state
  (``ready`` / ``not_localized``) or ``void``;
- a seed's inputs (the odometry snapshot taken at acceptance, the boot, the history's continuity) are watched until
  its end state: a move past ``SEED_STILL_M`` / ``SEED_STILL_RAD``, a boot change or a continuity break cancels it --
  locally if its frame has not gone (``void`` at once), else new scans stop, the frame being written is finished and
  ``cancel`` is sent (``CANCEL_SEND_S`` to send it, ``VOID_WAIT_S`` from then to the bridge's ``void``); any
  ``ready`` / ``not_localized`` / ``reply`` of a cancelled seed is dropped and counted;
- the deadlines are the executor's: a start or seed with no end state ``SEED_WAIT_S`` after its acceptance, no cancel
  sent within ``CANCEL_SEND_S`` of its decision, no ``void`` within ``VOID_WAIT_S`` of the send, or no reply within
  ``REPLY_WAIT_S`` of a scan sent to a localized instance ends the run (``SlamSourceFatal`` -- the run's guard marks
  ``fatal_source``); so does any socket error, EOF, malformed frame, ``fault`` or protocol mismatch.

The replies (6.1, 0 and 4): only a localized instance's replies reach the loop, each with the order stamp ``stamp_s``
(core seconds of the first-beam estimate), the evidence interval ``evidence_lo_s`` / ``evidence_hi_s``, the epoch the
scan was acquired in (the current one only if its ``Flo`` is at or after the epoch's start at this source) and the
attempt of a forced request; a reply whose receipt is more than ``LATE_S`` after its ``Flo`` is dropped
(``slam_late``) and a forced request it carried goes to the next scan, within the request's ``REQUEST_S``. A
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
LATE_S = 0.25  # a reply's receipt at most this after its first beam's earliest instant (design 6.1, 4)
REQUEST_S = 3.0  # a forced request's life from its acceptance (the confirmation's limit)
ODOM_RETRY_NS = 20_000_000  # one tick: a scan whose odometry is not in yet is tried once more after this
SEED_STILL_M = 0.01  # a seed's snapshot -> odometry now beyond this is a move (provisional)
SEED_STILL_RAD = math.radians(1.0)


class SlamSourceFatal(RuntimeError):
    """The real SLAM link cannot go on (the run's guard marks ``fatal_source``)."""


class RealSlamLink:
    """The executor end of the bridge's stream socket: one frame written at a time (never interleaved), frames read
    whole. Never blocks."""

    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        sock.setblocking(False)
        self._out = b""
        self.out_kind: str | None = None
        self._buffer = b""
        self.closed = False

    @property
    def busy(self) -> bool:
        return bool(self._out)

    def write(self, frame: bytes, kind: str) -> bool:
        if self._out:
            raise SlamSourceFatal("link: a frame written over another")
        self._out, self.out_kind = frame, kind
        return self.flush()

    def flush(self) -> bool:
        """True once nothing is left to write."""
        while self._out:
            try:
                n = self.sock.send(self._out)
            except (BlockingIOError, InterruptedError):
                return False
            except OSError as exc:
                raise SlamSourceFatal(f"link: send: {exc}") from exc
            self._out = self._out[n:]
        self.out_kind = None
        return True

    def read(self) -> list:
        """The whole frames received, decoded; EOF is reported after them (``closed``)."""
        try:
            while True:
                chunk = self.sock.recv(65536)
                if not chunk:
                    self.closed = True
                    break
                self._buffer += chunk
        except (BlockingIOError, InterruptedError):
            pass
        except OSError as exc:
            raise SlamSourceFatal(f"link: recv: {exc}") from exc
        out = []
        while len(self._buffer) >= 4:
            (length,) = struct.unpack(">I", self._buffer[:4])
            if len(self._buffer) < 4 + length:
                break
            body, self._buffer = self._buffer[4:4 + length], self._buffer[4 + length:]
            try:
                out.append(rl.decode(body))
            except ValueError as exc:
                raise SlamSourceFatal(f"link: a malformed frame: {exc}") from exc
        return out


class RealSlamSource:
    """See the module text. ``odom_history()``: the loop's odometry history (core seconds); ``odom_base()``: the
    odometry base_link pose now; ``boot()``: the MCU boot now; ``start_pose``: map <- base_link of the registered start
    (the session's first instance)."""

    def __init__(self, link: RealSlamLink, *, map_id: str, start_ns: int, start_pose, odom_history, odom_base, boot,
                 rear_offset_m: float, session_epoch: int = 0, clock=time.monotonic_ns, events=None) -> None:
        self.link, self.map_id, self.start_ns = link, str(map_id), int(start_ns)
        self.start_pose = tuple(float(v) for v in start_pose)
        self.odom_history, self.odom_base, self.boot = odom_history, odom_base, boot
        self.rear_offset_m, self.session_epoch, self.clock = float(rear_offset_m), int(session_epoch), clock
        self.events = events if events is not None else []  # the record (observe run, 6.1, 8)
        self.counts: Counter = Counter()
        self.epoch, self._prev_epoch = 0, 0
        self._epoch_start_ns = -(1 << 62)
        self._next_id = 1
        self.target: dict | None = None  # the current start / seed
        self.seeds: dict = {}  # epoch -> (kind, reason): seed_state's answers
        self._inflight: dict | None = None
        self._waiting: dict | None = None  # the newest scan ready to send
        self._odom_wait: dict | None = None  # a scan whose odometry is not in yet (one retry)
        self._requests: list = []  # [epoch, attempt, not_before_s, expires_ns]
        self._out: list = []
        self._last_stamp_ns: int | None = None
        self._scan_id = 0
        self._started = False

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
        """Accepted at once (6.1, 2): the new epoch, the odometry snapshot, the seed frame sent once the slot is
        empty. Returns the seed id (the frame's)."""
        if not self.can_seed():
            self._fatal("seed while another seed has no end state")  # the loop asks can_seed() first
        now = int(self.clock())
        self._new_epoch(int(epoch))
        sid = self._next_id
        self._next_id += 1
        self.target = {"id": sid, "kind": "seed", "epoch": self.epoch, "accepted_ns": now,
                       "deadline_ns": now + int(SEED_WAIT_S * 1e9), "pose": tuple(float(v) for v in base_pose),
                       "snapshot": tuple(float(v) for v in self.odom_base()), "boot": self.boot(), "state": "queued",
                       "cancel": None}
        self.seeds[self.epoch] = ("seeding", None)
        self._event(t="seed_accepted", id=sid, epoch=self.epoch, stamp_s=float(stamp_s))
        self._pump(now)
        return sid

    def request_processed(self, epoch, attempt, not_before) -> None:
        if int(epoch) >= self.epoch:
            self._requests.append([int(epoch), attempt, float(not_before), int(self.clock()) + int(REQUEST_S * 1e9)])

    # --- the scan path -----------------------------------------------------------------------------------------------
    def offer(self, s: srs.SlamScanIn) -> None:
        now = int(self.clock())
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
        self._prepare(s, base_from_rear(rear, self.rear_offset_m), now)

    def _prepare(self, s: srs.SlamScanIn, odom_from_base, now: int) -> None:
        if self._last_stamp_ns is not None and s.stamp_ns <= self._last_stamp_ns:
            return self._drop("slam_stamp_order", key=s.key)
        t = self.target
        if t is not None and t["kind"] == "seed" and self._seeding() and s.flo_ns < t["accepted_ns"]:
            return self._drop("slam_before_seed", key=s.key)
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
        if t is not None and t["cancel"] is not None and t["cancel"]["sent_ns"] is None:
            if self._write(rl.encode_cancel(rl.Cancel(t["id"], t["cancel"]["reason"])), "cancel"):
                t["cancel"]["sent_ns"] = int(self.clock())
                self._event(t="cancel_sent", id=t["id"])
            return
        if self._inflight is not None:
            return
        if t is not None and t["state"] == "queued":
            if t["kind"] == "start":
                frame = rl.encode_hello(rl.Hello(self.session_epoch, self.map_id, t["id"], t["pose"],
                                                 t["deadline_ns"]))
            else:
                frame = rl.encode_seed(rl.Seed(t["id"], t["epoch"], t["pose"], t["snapshot"], int(t["boot"]),
                                               t["deadline_ns"]))
            t["state"] = "sent"
            self._write(frame, t["kind"])
            return
        if t is None or t["state"] not in ("scans", "ready") or t["cancel"] is not None or self._waiting is None:
            return
        w, self._waiting = self._waiting, None
        if t["kind"] == "seed" and t["state"] == "scans" and w["flo_ns"] < t["accepted_ns"]:
            return self._drop("slam_before_seed", key=w["key"])
        attempt = None
        if t["state"] == "ready" and w["epoch"] == self.epoch:
            for req in list(self._requests):
                if req[0] == self.epoch and self._s(w["flo_ns"]) >= req[2] and now <= req[3]:
                    self._requests.remove(req)
                    attempt = req
                    break
        self._last_stamp_ns = w["stamp_ns"]
        self._inflight = {**w, "target": t["id"], "phase": t["state"], "attempt": attempt, "sent_ns": None}
        if self._write(rl.encode_scan(w["scan"], w["stamp_ns"]), "scan"):
            self._inflight["sent_ns"] = int(self.clock())
        self._event(t="scan_sent", key=w["key"], scan_id=w["scan"].scan_id, stamp_ns=w["stamp_ns"],
                    flo_ns=w["flo_ns"], fhi_ns=w["fhi_ns"], measured=w["measured"],
                    odom_from_base=list(w["odom_from_base"]), target=t["id"], phase=t["state"])

    def _write(self, frame: bytes, kind: str) -> bool:
        return self.link.write(frame, kind)

    # --- receiving ---------------------------------------------------------------------------------------------------
    def replies(self, now_ns: int) -> list:
        if not self._started:
            self._started = True
            now = int(self.clock())
            self.target = {"id": self._next_id, "kind": "start", "epoch": self.epoch, "accepted_ns": now,
                           "deadline_ns": now + int(SEED_WAIT_S * 1e9), "pose": self.start_pose, "snapshot": None,
                           "boot": None, "state": "queued", "cancel": None}
            self._next_id += 1
        if self._inflight is not None and self._inflight["sent_ns"] is None and self.link.flush():
            self._inflight["sent_ns"] = int(self.clock())
        looked = int(self.clock())  # before the read: what is read arrived at or after it
        frames = self.link.read()
        received = int(self.clock())
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
        self._deadlines(looked)
        self._try_odom(looked)
        self._pump(looked)
        out, self._out = self._out, []
        return out

    def _on_reply(self, r, received: int) -> None:
        f = self._inflight
        if f is None or f["sent_ns"] is None:
            self._fatal("protocol: a reply with no scan sent whole")
        if int(r.scan_id) != f["scan"].scan_id or float(r.stamp_s) != float(f["scan"].stamp_s):
            self._fatal(f"protocol: reply for scan {r.scan_id} to scan {f['scan'].scan_id}")
        if r.status == "failed":
            self._fatal(f"bridge failed scan {r.scan_id}: {r.reason}")
        self._inflight = None
        t = self.target
        rec = {"t": "reply", "scan_id": int(r.scan_id), "status": r.status, "map_from_odom": list(r.map_from_odom),
               "received_ns": received, "flo_ns": f["flo_ns"], "target": f["target"], "phase": f["phase"]}
        if t is None or t["id"] != f["target"] or t["cancel"] is not None:
            # an instance stopped since the scan went (a later seed) or being cancelled: its correction is no
            # evidence of the current one
            why = "cancelled" if t is not None and t["id"] == f["target"] else "old_instance"
            self.counts[f"slam_{why}_reply"] += 1
            self._event(**rec, passed=False, why=why)
            return
        if f["phase"] != "ready" or not f["measured"]:
            self._event(**rec, passed=False, why="start_check" if f["phase"] != "ready" else "diagnostic")
            return
        if received - f["flo_ns"] > LATE_S * 1e9:
            self.counts["slam_late"] += 1
            self._event(**rec, passed=False, why="late")
            if f["attempt"] is not None and received <= f["attempt"][3]:
                self._requests.insert(0, f["attempt"])  # the next scan carries it (6.1, 2)
            return
        reply = SimpleNamespace(scan_id=int(r.scan_id), stamp_s=self._s(f["stamp_ns"]), status=r.status,
                                map_from_odom=tuple(float(v) for v in r.map_from_odom), slam_wall_s=None,
                                epoch=f["epoch"], attempt=None if f["attempt"] is None else f["attempt"][1],
                                evidence_lo_s=self._s(f["flo_ns"]), evidence_hi_s=self._s(f["fhi_ns"]))
        self._event(**rec, passed=True, epoch=f["epoch"], attempt=reply.attempt)
        self._out.append((reply, {"source": "real_slam"}, f["sent_ns"] / 1e9, received / 1e9))

    def _on_state(self, st, received: int) -> None:
        t = self.target
        self._event(t="state", id=int(st.id), kind=st.kind, reason=st.reason)
        if st.kind == "fault":
            self._fatal(f"bridge fault: {st.reason}")
        if t is None or int(st.id) != t["id"]:
            self._fatal(f"protocol: state {st.kind} for id {st.id}, not the current start or seed")
        if st.kind == "void":
            if t["cancel"] is None or t["cancel"]["sent_ns"] is None:
                self._fatal(f"protocol: void of {st.id} with no cancel sent")
            if self._inflight is not None and self._inflight["target"] == t["id"]:
                self._inflight = None  # the seed's scan in flight goes with its instance
            self.seeds[t["epoch"]] = ("void", t["cancel"]["reason"])
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
        if t["state"] != "scans":
            self._fatal(f"protocol: {st.kind} in {t['state']}")
        t["state"] = st.kind  # ready / not_localized
        if t["kind"] == "seed":
            self.seeds[t["epoch"]] = (st.kind, st.reason)

    # --- the seed's inputs and the deadlines -------------------------------------------------------------------------
    def _watch_seed(self, now: int) -> None:
        t = self.target
        if t is None or t["kind"] != "seed" or not self._seeding() or t["cancel"] is not None:
            return
        why = None
        if self.boot() != t["boot"]:
            why = "boot_changed"
        elif any(not (e[3] and e[4]) for e in self.odom_history() if e[1] >= self._s(t["accepted_ns"])):
            why = "odom_break"
        else:
            x, y, yaw = (float(v) for v in self.odom_base())
            sx, sy, syaw = t["snapshot"]
            dyaw = abs(math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw)))
            if math.hypot(x - sx, y - sy) > SEED_STILL_M or dyaw > SEED_STILL_RAD:
                why = "seed_moved"
        if why is None:
            return
        self._event(t="cancel_decided", id=t["id"], reason=why, state=t["state"])
        if t["state"] == "queued":
            self.seeds[t["epoch"]] = ("void", why)  # never sent: cancelled here, the bridge never hears of it
            self.target = None
            return
        t["cancel"] = {"reason": why, "decided_ns": now, "sent_ns": None}
        if self._waiting is not None:
            self._drop("slam_cancel_cut", key=self._waiting["key"])
        self._waiting, self._odom_wait = None, None

    def _deadlines(self, now: int) -> None:
        t = self.target
        if t is not None and t["cancel"] is not None:
            c = t["cancel"]
            if c["sent_ns"] is None and now - c["decided_ns"] > CANCEL_SEND_S * 1e9:
                self._fatal(f"cancel of {t['id']} not sent within {CANCEL_SEND_S} s")
            if c["sent_ns"] is not None and now - c["sent_ns"] > VOID_WAIT_S * 1e9:
                self._fatal(f"no void for {t['id']} within {VOID_WAIT_S} s of its cancel")
        elif self._seeding() and now > t["deadline_ns"]:
            self._fatal(f"{t['kind']} {t['id']}: no end state within {SEED_WAIT_S} s")
        f = self._inflight
        if (f is not None and f["sent_ns"] is not None and f["phase"] == "ready"
                and now - f["sent_ns"] > REPLY_WAIT_S * 1e9):
            self._fatal(f"scan {f['scan'].scan_id}: no reply within {REPLY_WAIT_S} s")


__all__ = ["RealSlamLink", "RealSlamSource", "SlamSourceFatal"]
