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
import os
import select
import sys
import termios
import time

from forklift_core.plant.mcu_host import HostConfig, HostEvent, McuLinkHost, Outgoing

_LOST = {errno.EIO, errno.ENODEV, errno.ENXIO, errno.EBADF}
_BAUDS = {9600: termios.B9600, 115200: termios.B115200, 230400: termios.B230400, 460800: termios.B460800,
          921600: termios.B921600}


class LinkLost(Exception):
    pass


def open_raw(path: str, baud: int | None = None) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
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
    return fd


class TtyLink:
    """One open endpoint and the frame being written."""

    def __init__(self, fd: int) -> None:
        self.fd = fd
        self._cur: Outgoing | None = None
        self._off = 0

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

    def pump_writes(self, host: McuLinkHost, now_ns: int) -> None:
        while True:
            if self._cur is None:
                self._cur, self._off = host.next_frame(now_ns), 0
                if self._cur is None:
                    return
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
                host.frame_written(self._cur, now_ns)
                self._cur = None

    def abandon(self, host: McuLinkHost, now_ns: int) -> None:
        if self._cur is not None:
            host.frame_abandoned(self._cur, self._off, now_ns)
            self._cur = None

    def close(self) -> None:
        try:
            os.close(self.fd)
        except OSError:
            pass


class ReconnectingLink:
    """A device path reopened after loss; ``service`` is called every control cycle."""

    def __init__(self, path: str, *, baud: int | None = None, retry_s: float = 0.5, opener=open_raw) -> None:
        self.path, self.baud, self.retry_ns, self.opener = path, baud, int(retry_s * 1e9), opener
        self.link: TtyLink | None = None
        self._next_try = 0
        # Bring-up tx_cut (docs/design/2026-11-12-bringup-host-tools.md 2): False stops writing to the MCU while
        # its STATE is still read -- the MCU's command-expiry path with the stop kept in view.
        self.tx_enabled = True

    def service(self, host: McuLinkHost, now_ns: int) -> list[HostEvent]:
        events: list[HostEvent] = []
        if self.link is None and now_ns >= self._next_try:
            try:
                self.link = TtyLink(self.opener(self.path, self.baud))
                events.append(HostEvent("connected", {"path": self.path}))
            except OSError:
                self._next_try = now_ns + self.retry_ns
        if self.link is not None:
            try:
                data = self.link.read_available()
                if data:
                    events += host.on_bytes(data, now_ns)
                events += host.tick(now_ns)
                if self.tx_enabled:
                    self.link.pump_writes(host, now_ns)
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
            self.link.pump_writes(host, now_ns)
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
    link = ReconnectingLink(args.device, baud=args.baud)
    end = time.monotonic_ns() + int(args.seconds * 1e9)
    while (now := time.monotonic_ns()) < end:
        for e in link.service(host, now):
            print(f"{now / 1e9:.3f} {e.kind} {e.detail}", flush=True)
        time.sleep(0.005)
    h = host.health(time.monotonic_ns())
    print(f"final: fresh={h.fresh} boot={h.boot_id} mode={h.mode} faults=0x{h.fault_flags:x} "
          f"decode_errors={h.decode_errors}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
