#!/usr/bin/env python3
"""I/O for the mcu-link host core (sim-to-real plan P0-7): a raw, non-blocking tty (USB CDC, UART or a
pty) feeding ``forklift_core.plant.mcu_host.McuLinkHost``.

Writes go one whole frame at a time: a frame started is finished before the next is taken (partial
writes and EAGAIN keep the rest for the next service call); a frame the device loss interrupted is
logged as abandoned and never replayed. A lost device (EIO, ENODEV, EOF) tells the host it is
disconnected -- which drops everything queued and revokes operator authorization -- and the link is
reopened every ``retry_s``. Nothing here sends ARM or CLEAR_FAULTS; the monitor CLI only listens.

usage: real_mcu_link.py --device /dev/forklift-mcu [--baud 460800] [--seconds 10]
"""

from __future__ import annotations

import argparse
import errno
import fcntl
import os
import select
import stat
import sys
import termios
import time
from pathlib import Path

from forklift_core.live.executor_lock import DeviceBusy, lock_fd
from forklift_core.mcu_link.messages import Disarm, encode
from forklift_core.plant.mcu_host import HostConfig, HostEvent, McuLinkHost, Outgoing

_LOST = {errno.EIO, errno.ENODEV, errno.ENXIO, errno.EBADF}
_BAUDS = {9600: termios.B9600, 115200: termios.B115200, 230400: termios.B230400, 460800: termios.B460800,
          921600: termios.B921600}


class LinkLost(Exception):
    pass


def open_raw(path: str, baud: int | None = None, *, before_config=None) -> int:
    """Open the link raw. The USB CDC firmware drops every STATE while the host's DTR is low (firmware board design
    5): Linux cdc_acm raises DTR on open, and nothing here lowers it -- no zero speed (it hangs the line up) and no
    clearing of the modem bits; DTR (and RTS) are asked high once more in case a previous user left them low. A
    device without modem bits (a pty, a UART adapter that refuses) is fine as it is."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    if before_config is not None:
        # run on the opened fd before anything is set on the port (the executor lock: a refused opening changes
        # nothing on a device another executor holds); if it raises, the fd is closed and the error passes on
        try:
            before_config(fd)
        except BaseException:
            os.close(fd)
            raise
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0  # iflag, oflag, lflag: raw
        attrs[2] = (attrs[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)) | termios.CS8 | termios.CREAD | termios.CLOCAL
        if baud is not None:
            attrs[4] = attrs[5] = _BAUDS[baud]
        attrs[6][termios.VMIN], attrs[6][termios.VTIME] = 0, 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except termios.error:
        pass  # not every endpoint is a tty with settable attributes (a pty master is fine raw already)
    try:
        bits = (termios.TIOCM_DTR | termios.TIOCM_RTS).to_bytes(4, sys.byteorder)
        fcntl.ioctl(fd, termios.TIOCMBIS, bits)
    except OSError:
        pass  # no modem control on this endpoint
    return fd


def usb_product_for_rdev(rdev: int, *, sys_root="/sys") -> str | None:
    """The USB product string of the device a character node belongs to: from ``/sys/dev/char/<major>:<minor>`` up
    the device tree to the nearest USB device (the directory with ``idVendor``) and its own ``product`` -- never a
    hub's above it (Linux omits ``product`` when the device has no string, Codex P2). None for a node with no USB
    device above it (a UART such as ttyS or ttyTHS, a pty), a device without the string, or one that cannot be
    read."""
    root = Path(sys_root)
    try:
        node = (root / "dev" / "char" / f"{os.major(rdev)}:{os.minor(rdev)}").resolve(strict=True)
    except OSError:
        return None
    top = root.resolve()
    for d in (node, *node.parents):
        if d == top or top not in d.parents:
            break
        if (d / "idVendor").is_file():  # the nearest USB device: its own string or none
            try:
                return (d / "product").read_text(encoding="ascii", errors="replace").strip()
            except OSError:
                return None
    return None


def usb_product_of_fd(fd: int) -> str | None:
    """The USB product string of the device ``fd`` is open on (the opened tty itself, never the path looked up again:
    a reconnect or a swapped board must not lend another device's string; plan v1 e-stop waiver 3, Codex)."""
    try:
        st = os.fstat(fd)
    except OSError:
        return None
    if not stat.S_ISCHR(st.st_mode):
        return None
    return usb_product_for_rdev(st.st_rdev)


DELIMITER = b"\x00"
# 6a's poison (docs/design/2026-11-20-real-hil-6a.md 3, real_loop_transport.POISON): more nonzero bytes than either
# decoder's ENCODED_MAX (91) before the delimiter, so whatever half frame the MCU holds is dropped as too long -- a
# half frame missing only its final delimiter would otherwise be completed by a lone 0x00 and act late
POISON = bytes([0x01]) * 96 + DELIMITER


class TtyLink:
    """One open endpoint and the frame being written. On every opening the poison goes out right before the first
    frame (real safety wiring design 8.1): the MCU's receive buffer may hold half a frame from the last connection,
    another executor's included, and the poison drops it instead of completing it. An opening that never sends a
    frame (a listener) writes nothing at all, so the MCU counts no receive error for it."""

    def __init__(self, fd: int) -> None:
        self.fd = fd
        self._cur: Outgoing | None = None
        self._off = 0
        self._cur_t = 0  # the current frame's write time (taken before its first byte)
        self._poison_at: int | None = 0  # the poison's next byte on this opening (None: out)

    def read_available(self) -> bytes:
        """Whatever is waiting. An empty read is "nothing yet" on a raw tty (VMIN=0), not end of file;
        a hangup shows as POLLHUP/POLLERR or as EIO/ENODEV on read (Codex P0-7 round 2 P1)."""
        poller = select.poll()
        poller.register(self.fd, select.POLLIN | select.POLLHUP | select.POLLERR)
        hung = 0
        for _fd, revents in poller.poll(0):
            hung |= revents & (select.POLLHUP | select.POLLERR | select.POLLNVAL)
        chunks = []
        while True:
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                break
            except InterruptedError:
                continue
            except OSError as exc:
                if exc.errno in _LOST:
                    raise LinkLost(str(exc)) from exc
                raise
            if not data:
                break
            chunks.append(data)
        if hung and not chunks:  # a hung-up tty also reports POLLIN and reads empty: deliver data first
            raise LinkLost(f"hangup (poll 0x{hung:x})")
        return b"".join(chunks)

    def pump_writes(self, host: McuLinkHost, now_ns: int, clock=None) -> None:
        """``clock``: the D5 write time (design 3.3b) -- each frame's time is taken just before its first byte is
        written (earlier than the send: a round trip from it is never under-estimated); None: ``now_ns``."""
        while True:
            if self._cur is None:
                self._cur, self._off = host.next_frame(now_ns), 0
                if self._cur is None:
                    return
                if not self._allowed(self._cur):
                    # refused before its first byte -- and before the opening's poison (ObserveTtyLink)
                    host.frame_refused(self._cur, now_ns, "not_allowed_on_this_link")
                    self._cur = None
                    continue
            if self._poison_at is not None:
                # the opening's poison, right before its first frame; until it is all out the frame is handed back
                # untouched, so a DISARM, a revoke or its expiry can still drop it before the next attempt
                try:
                    n = os.write(self.fd, POISON[self._poison_at:])
                except (BlockingIOError, InterruptedError):
                    n = 0
                except OSError as exc:
                    if exc.errno in _LOST:
                        raise LinkLost(str(exc)) from exc
                    raise
                self._poison_at += n
                if self._poison_at < len(POISON):
                    host.unsent(self._cur, now_ns)
                    self._cur = None
                    return
                self._poison_at = None
            if self._off == 0:
                self._cur_t = now_ns if clock is None else clock()
            try:
                n = os.write(self.fd, self._cur.data[self._off:])
            except BlockingIOError:
                if self._off == 0:
                    # Nothing of it is out yet: hand it back so a DISARM, a revoke or its expiry can still
                    # drop it before the next attempt (Codex P0-7 round 2 P1).
                    host.unsent(self._cur, now_ns)
                    self._cur = None
                return
            except InterruptedError:
                continue
            except OSError as exc:
                if exc.errno in _LOST:
                    raise LinkLost(str(exc)) from exc
                raise
            self._off += n
            if self._off == len(self._cur.data):
                host.frame_written(self._cur, self._cur_t)
                self._cur = None

    def _allowed(self, out: Outgoing) -> bool:
        """Whether a frame taken from the host may be written on this link (every frame, here)."""
        return True

    def abandon(self, host: McuLinkHost, now_ns: int) -> None:
        if self._cur is not None:
            host.frame_abandoned(self._cur, self._off, now_ns)
            self._cur = None

    def close(self) -> None:
        try:
            os.close(self.fd)
        except OSError:
            pass


class ObserveTtyLink(TtyLink):
    """The observe run's link (PR 7 design 1.1): a DISARM-only write boundary. A frame is written only if its kind
    is DISARM and its bytes are that seq's canonical encoding; anything else is refused before its first byte -- and
    before the opening's poison, which therefore goes out only ahead of the first allowed DISARM. All that can reach
    the device on one fd is the poison followed by canonical DISARMs, whatever any code queued on the host."""

    def __init__(self, fd: int) -> None:
        super().__init__(fd)
        self.refused = 0

    def _allowed(self, out: Outgoing) -> bool:
        if out.kind == "DISARM" and bytes(out.data) == encode(Disarm(), out.seq):
            return True
        self.refused += 1
        return False


class ReconnectingLink:
    """A device path reopened after loss; ``service`` is called every control cycle."""

    def __init__(self, path: str, *, baud: int | None = None, retry_s: float = 0.5, opener=open_raw,
                 clock=None, product_reader=usb_product_of_fd, exclusive: bool = False,
                 link_factory=None) -> None:
        """``clock``: the teleop bridge's D5 times (docs/design/2026-11-15-d5-link-gap-mcu-time.md 3.3b) -- the host is
        given the time a read returned (later than the bytes' arrival: validity shorter, gaps longer) and each frame's
        write time is taken before its write; None keeps the cycle time for both (every other tool)."""
        self.path, self.baud, self.retry_ns, self.opener = path, baud, int(retry_s * 1e9), opener
        # One executor per device (docs/design/2026-11-19-real-safety-wiring.md 8): an exclusive flock on every fd this
        # link opens, a reconnect included; held elsewhere, the opening is refused (a ``device_busy`` event naming the
        # holder) and tried again later. Off by default (tests and rigs open one pty more than once); every tool that
        # opens the MCU device passes True (design 8.1, pinned by tests/unit/tools/test_executor_lock_tools.py).
        # Held at the very first opening, the link ends refused (design 8.1): the DeviceBusy is raised once and the
        # link never opens nor writes again -- a tool's cleanup cannot reopen it after the holder lets go.
        self.exclusive = bool(exclusive)
        self.busy: DeviceBusy | None = None  # the last refusal (None once an opening succeeded)
        self.ever_connected = False
        self.refused: DeviceBusy | None = None
        self._pending: list[HostEvent] = []  # an early opening's events (open_first), handed out by the next service
        # the link class every opening -- the first and each reopening -- is wrapped in (the observe run's
        # ObserveTtyLink, PR 7 design 1.1); TtyLink otherwise
        self.link_factory = TtyLink if link_factory is None else link_factory
        self.product_reader = product_reader  # the opened device's USB product string, read at each opening
        self.clock = clock
        self.link: TtyLink | None = None
        self._next_try = 0
        # Bring-up tx_cut (docs/design/2026-11-12-bringup-host-tools.md 2): False stops writing to the MCU while
        # its STATE is still read -- the MCU's command-expiry path with the stop kept in view.
        self.tx_enabled = True
        # when the last read returned (monotonic ns): the read time a record keeps beside the cycle time the host
        # is given (teleop record design 3) -- it never feeds a control decision
        self.last_read_ns: int | None = None

    def open_first(self, host: McuLinkHost, now_ns: int) -> None:
        """Try the first opening now, before a tool starts anything else (the teleop bridge: before its console), so a
        device held by another executor is refused first (design 8.1). Its events come out of the next ``service``."""
        if self.link is None and self.refused is None:
            self._pending += self._open(host, now_ns)

    def _open(self, host: McuLinkHost, now_ns: int) -> list[HostEvent]:
        events: list[HostEvent] = []
        if self.link is None and now_ns >= self._next_try:
            try:
                if self.exclusive:  # locked on the opened fd before the port is configured (real safety design 8)
                    fd = self.opener(self.path, self.baud, before_config=lambda fd_: lock_fd(fd_, self.path))
                else:
                    fd = self.opener(self.path, self.baud)
                self.busy = None
                self.ever_connected = True
                self.link = self.link_factory(fd)
                product = self.product_reader(self.link.fd)
                events.append(HostEvent("connected", {"path": self.path, "product": product}))
                events += host.link_identity(product)
            except DeviceBusy as busy:
                if not self.ever_connected:
                    self.refused = busy
                    raise
                if self.busy is None or str(self.busy) != str(busy):
                    events.append(HostEvent("device_busy", {"path": self.path, "holder": busy.holder}))
                self.busy = busy
                self._next_try = now_ns + self.retry_ns
            except OSError:
                self._next_try = now_ns + self.retry_ns
        return events

    def service(self, host: McuLinkHost, now_ns: int) -> list[HostEvent]:
        events, self._pending = self._pending, []
        if self.refused is not None:
            return events + host.tick(now_ns)
        events += self._open(host, now_ns)
        if self.link is not None:
            try:
                data = self.link.read_available()
                if data:
                    self.last_read_ns = time.monotonic_ns() if self.clock is None else self.clock()
                    events += host.on_bytes(data, now_ns if self.clock is None else self.last_read_ns)
                t = now_ns if self.clock is None else self.clock()
                events += host.tick(t)
                if self.tx_enabled:
                    self.link.pump_writes(host, now_ns, self.clock)
                return events
            except LinkLost:
                self.link.abandon(host, now_ns)
                self.link.close()
                self.link = None
                self._next_try = now_ns + self.retry_ns
                events += host.disconnected(now_ns)
                return events
        events += host.tick(now_ns)
        return events

    # The teleop bridge's cycle (docs/design/2026-11-12-teleop-bridge.md 5): receive, decide, then transmit -- a
    # STOP or an expiry judged in between drops what was queued before anything is written.
    def receive(self, host: McuLinkHost, now_ns: int) -> list[HostEvent]:
        """``service`` without the writes."""
        saved, self.tx_enabled = self.tx_enabled, False
        try:
            return self.service(host, now_ns)
        finally:
            self.tx_enabled = saved

    def transmit(self, host: McuLinkHost, now_ns: int) -> list[HostEvent]:
        """The writes only (a lost device is handled as in ``service``)."""
        if self.link is None or not self.tx_enabled:
            return []
        try:
            self.link.pump_writes(host, now_ns, self.clock)
            return []
        except LinkLost:
            self.link.abandon(host, now_ns)
            self.link.close()
            self.link = None
            self._next_try = now_ns + self.retry_ns
            return host.disconnected(now_ns)


def service_fd(link: TtyLink, host: McuLinkHost, now_ns: int, *, tx_enabled: bool = True) -> list[HostEvent]:
    """One cycle on an endpoint that is not reopened (a pty master in tests)."""
    events = []
    data = link.read_available()
    if data:
        events += host.on_bytes(data, now_ns)
    events += host.tick(now_ns)
    if tx_enabled:
        link.pump_writes(host, now_ns)
    return events


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Listen to an MCU over mcu-link/2 and print link events (never arms).")
    ap.add_argument("--device", required=True)
    ap.add_argument("--baud", type=int, choices=sorted(_BAUDS))
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args(argv)
    host = McuLinkHost(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100,
                                  outbox_max=16))
    # a listener takes the lock too: two readers of one tty split its bytes (real safety wiring design 8.1)
    link = ReconnectingLink(args.device, baud=args.baud, exclusive=True)
    end = time.monotonic_ns() + int(args.seconds * 1e9)
    while (now := time.monotonic_ns()) < end:
        try:
            events = link.service(host, now)
        except DeviceBusy as busy:
            print(f"refused: {busy}", file=sys.stderr, flush=True)
            return 2
        for e in events:
            print(f"{now / 1e9:.3f} {e.kind} {e.detail}", flush=True)
        time.sleep(0.005)
    h = host.health(time.monotonic_ns())
    print(f"final: fresh={h.fresh} boot={h.boot_id} mode={h.mode} faults=0x{h.fault_flags:x} "
          f"decode_errors={h.decode_errors}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
