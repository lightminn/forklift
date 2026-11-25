"""A real-mode SLAM bridge stand-in for the pty HIL runs (PR 7 7c-1b-2b): the 7c-1a protocol on a Unix socket in a
thread of the test -- ``hello`` / ``seed`` answered ``seeding`` then ``ready_for_scans`` (a seed's after
``seed_delay_s``), every scan answered ``processed`` at once with the correction its start pose implies on the
instance's first scan's odometry, the seventh start scan followed by the end state (``seed_end`` for seeds),
``cancel`` answered ``void``. ``fault_after`` scans of a localized instance: a ``fault`` state instead of a reply.
``reply_delay_s``: each reply held that long (the next scan waits for it, as the bridge's one scan in flight).
``hold_end``: ids whose end state waits for ``release(id)``. ``closing()``: the test is ending the run -- a broken
connection after it ends the thread quietly; before it, it is the bridge's error.
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
                 hold_end=()):
        super().__init__(daemon=True)
        self.path, self.seed_delay_s, self.seed_end, self.fault_after = str(path), seed_delay_s, seed_end, fault_after
        self.reply_delay_s = float(reply_delay_s)
        self._held = {int(i): threading.Event() for i in hold_end}
        self._closing = threading.Event()
        self.got: list = []  # (monotonic s, frame type name, id or scan id)
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
        self._send(rl.encode_state(rl.State(ident, kind, reason)))

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
        if isinstance(f, (rl.Hello, rl.Seed)):
            ident = f.start_id if isinstance(f, rl.Hello) else f.seed_id
            self.got.append((now, type(f).__name__, ident))
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
                if self._cur is not c:
                    return  # the instance went (a seed or a cancel) while the reply was held
                self._send(frame)
                if last:
                    self._end(c)

            if self.reply_delay_s > 0:
                self._timers.append((now + self.reply_delay_s, answer))
            else:
                answer()
