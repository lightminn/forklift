"""A real-mode SLAM bridge stand-in for the pty HIL runs (PR 7 7c-1b-2b): the 7c-1a protocol on a Unix socket in a
thread of the test -- ``hello`` / ``seed`` answered ``seeding`` then ``ready_for_scans`` (a seed's after
``seed_delay_s``), every scan answered ``processed`` at once with the correction its start pose implies on the
instance's first scan's odometry, the seventh start scan followed by the end state (``seed_end`` for seeds),
``cancel`` answered ``void``. ``fault_after`` scans of a localized instance: a ``fault`` state instead of a reply.
``reply_delay_s``: each reply held that long (the next scan waits for it, as the bridge's one scan in flight).
``hold_end``: ids whose end state waits for ``release(id)``. ``hold_replies()`` / ``release_replies()``: scan
replies kept from now until released. ``closing()``: the test is ending the run -- a broken
connection after it ends the thread quietly; before it, it is the bridge's error.
``laser`` (with ``map_id``): the real bridge's comparison (operating plan 2.3, U3e-1) -- a hello whose map or
base_from_laser does not match is a ``fault``; a matching one's digest rides on every ``ready`` of the session
(``laser_checked``), as the operating executor requires. Without it, no comparison and no digest (an older bridge).
Not slam_toolbox: nothing here is evidence of matching or of the bridge's timing."""

from __future__ import annotations

import os
import select
import socket
import threading
import time

from forklift_core.localization import slam_link
from forklift_core.localization import slam_real_link as rl
from forklift_core.localization.slam_pose import compose, invert


class FakeBridge(threading.Thread):
    def __init__(self, path, *, seed_delay_s=0.0, seed_end="ready", fault_after=None, reply_delay_s=0.0,
                 hold_end=(), map_id=None, laser=None):
        super().__init__(daemon=True)
        self.map_id, self.laser = map_id, None if laser is None else tuple(float(v) for v in laser)
        self.digest = None  # the hello's base_from_laser digest once compared (laser mode)
        self.path, self.seed_delay_s, self.seed_end, self.fault_after = str(path), seed_delay_s, seed_end, fault_after
        self.reply_delay_s = float(reply_delay_s)
        self._held = {int(i): threading.Event() for i in hold_end}
        self._holding = threading.Event()
        self.held_replies: list = []
        self._closing = threading.Event()
        self.got: list = []  # (monotonic s, frame type name, id or scan id)
        self.poses: dict = {}  # id -> the start pose its hello / seed carried
        self.stamps: list = []  # (monotonic ns at receipt, the scan's stamp_ns as sent)
        self.sent: list = []  # (monotonic s, kind, id)
        self.error = None
        self._halt = threading.Event()
        self._server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        if os.path.exists(self.path):
            os.unlink(self.path)
        self._server.bind(self.path)
        self._server.listen(1)
        self._conn = None
        self._buf = b""
        self._timers: list = []  # (due monotonic s, fn)
        self._cur = None
        self._ready_scans = 0

    def hold_replies(self):
        self._holding.set()

    def release_replies(self):
        self._timers.append((time.monotonic(), self._release_replies))

    def _release_replies(self):
        self._holding.clear()
        held, self.held_replies = self.held_replies, []
        for fn in held:
            fn()

    def release(self, ident):
        self._held[int(ident)].set()

    def closing(self):
        self._closing.set()

    def stop(self):
        self._halt.set()
        self.join(5)
        for s in (self._conn, self._server):
            if s is not None:
                s.close()
        if os.path.exists(self.path):
            os.unlink(self.path)

    def _send(self, frame):
        self._conn.sendall(frame)

    def _state(self, ident, kind, reason=None):
        self.sent.append((time.monotonic(), kind, ident))
        checked = self.digest if kind == "ready" else None
        self._send(rl.encode_state(rl.State(ident, kind, reason, laser_checked=checked)))

    def run(self):
        try:
            self._server.settimeout(0.2)
            while not self._halt.is_set() and self._conn is None:
                try:
                    self._conn, _ = self._server.accept()
                except socket.timeout:
                    continue
            if self._conn is None:
                return
            while not self._halt.is_set():
                now = time.monotonic()
                for due, fn in list(self._timers):
                    if due <= now:
                        self._timers.remove((due, fn))
                        fn()
                r, _, _ = select.select([self._conn], [], [], 0.005)
                if not r:
                    continue
                chunk = self._conn.recv(1 << 16)
                if not chunk:
                    if not self._closing.is_set():
                        self.error = "the executor closed the connection while the run went on"
                    return
                self._buf += chunk
                while len(self._buf) >= 4:
                    n = int.from_bytes(self._buf[:4], "big")
                    if len(self._buf) < 4 + n:
                        break
                    body, self._buf = self._buf[4:4 + n], self._buf[4 + n:]
                    self._on(rl.decode(body))
        except (BrokenPipeError, ConnectionResetError) as exc:
            if not self._closing.is_set():
                self.error = repr(exc)  # broken while the run went on
        except Exception as exc:  # noqa: BLE001 -- the test reads it
            self.error = repr(exc)

    def _end(self, c):
        held = self._held.get(c["id"])
        if held is not None and not held.is_set():
            self._timers.append((time.monotonic() + 0.01, lambda c=c: self._end(c) if self._cur is c else None))
            return
        self._state(c["id"], c["end"], None if c["end"] == "ready" else "NOT_LOCALIZED: test")
        c["localized"] = c["end"] == "ready"

    def _on(self, f):
        now = time.monotonic()
        if isinstance(f, rl.Hello) and self.laser is not None:
            # the real bridge's hello comparison (real_bridge_core.on_frame): the map, then the mount
            if self.map_id is not None and f.map_id != self.map_id:
                self._state(f.start_id, "fault", f"map_mismatch: {f.map_id}")
                return
            if f.base_from_laser is None or not rl.laser_matches(f.base_from_laser, self.laser):
                self._state(f.start_id, "fault", f"laser_mismatch: {f.base_from_laser}")
                return
            self.digest = rl.laser_digest(f.base_from_laser)
        if isinstance(f, (rl.Hello, rl.Seed)):
            ident = f.start_id if isinstance(f, rl.Hello) else f.seed_id
            self.got.append((now, type(f).__name__, ident))
            self.poses[ident] = tuple(f.start_pose)
            self._cur = {"id": ident, "pose": f.start_pose, "n": 0, "corr": None,
                         "end": "ready" if isinstance(f, rl.Hello) else self.seed_end, "localized": False}
            self._state(ident, "seeding")
            delay = 0.0 if isinstance(f, rl.Hello) else self.seed_delay_s
            self._timers.append((now + delay, lambda i=ident: self._state(i, "ready_for_scans")))
        elif isinstance(f, rl.Cancel):
            self.got.append((now, "Cancel", f.seed_id))
            self._cur = None
            self._state(f.seed_id, "void", f.reason)
        elif isinstance(f, rl.RealScan):
            self.got.append((now, "RealScan", f.scan.scan_id))
            self.stamps.append((time.monotonic_ns(), int(f.stamp_ns)))
            c = self._cur
            if c["localized"]:
                self._ready_scans += 1
                if self.fault_after is not None and self._ready_scans > self.fault_after:
                    self._state(c["id"], "fault", "test: injected")
                    return
            if c["corr"] is None:
                c["corr"] = tuple(compose(c["pose"], invert(f.scan.odom_from_base)))
            c["n"] += 1
            s = f.scan
            frame = slam_link.encode_reply(slam_link.Reply(s.scan_id, s.stamp_s, "processed", c["corr"], 1, 0.0))
            last = c["n"] == 7

            def answer(frame=frame, c=c, last=last):
                if self._holding.is_set():
                    self.held_replies.append(lambda: answer(frame, c, last))
                    return
                if self._cur is not c:
                    return  # the instance went (a seed or a cancel) while the reply was held
                self._send(frame)
                if last:
                    self._end(c)

            if self.reply_delay_s > 0:
                self._timers.append((now + self.reply_delay_s, answer))
            else:
                answer()
