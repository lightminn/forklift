#!/usr/bin/env python3
"""The real loop's TTY transport and its HIL pty (docs/design/2026-11-20-real-hil-6a.md 1-3).

``LoopTtyTransport`` keeps the 4c loop's transport contract (``receive`` / ``transmit`` / ``resync`` / the partial
frame) on a raw, non-blocking tty fd it is given and never closes or reopens:

- every opening's first write is the poison ``0x01 x 96 + 0x00`` (longer than either decoder's ENCODED_MAX, so a
  half frame the MCU still holds is dropped as an overflow), resumed from its offset after an EAGAIN; no frame goes out
  before its delimiter;
- a frame started is passed through the gate again before each continuation; a refused continuation (or a stuck one,
  ``resync``) is abandoned on the same fd and the poison follows -- the receive side is left as it is;
- the write callback the gate is given is ``write(*, judged_ns=None)``: the judged clock value of the last successful
  write of each frame is kept (``judged``), with the first byte's and the last write's instants;
- a receive reads at most 4096 bytes and gives the host the actual read time;
- ``hil_plant`` is true only while every STATE of the current boot showed HIL_PLANT and OUTPUTS_LOCKED without
  COMMISSION_BUILD (a boot change starts over);
- a lost device (EIO, ENXIO, EBADF, ENODEV) ends the run: ``lost`` keeps the first path that saw it and whether a half
  frame or an unfinished poison was left (``stream_tainted``), and the call raises ``DeviceLost``.

``HilPty`` makes the runner's own pty and model (design 1): ``openpty`` -> the slave locked at once (TIOCEXCL),
non-blocking, close-on-exec, raw -> no other fd on the slave (``device_shared``) -> the master's input drained -> the
model started on the master (``--link-fd``) -> the parent's master closed. The slave's path is never told to anyone.
"""

from __future__ import annotations

import errno
import fcntl
import os
import subprocess
import termios
import time
from pathlib import Path

from forklift_core.mcu_link.messages import LinkDecoder, McuState, StateFlag

POISON = bytes([0x01]) * 96 + b"\x00"
READ_CAP = 4096
_LOST = {errno.EIO, errno.ENXIO, errno.EBADF, errno.ENODEV}
RESERVED_MODEL_ARGS = ("--link", "--link-fd", "--duration-ms", "--hil-plant", "--commission-build",
                       "--outputs-locked")
HIL_FIRST_STATE_S = 2.0


class DeviceLost(Exception):
    pass


class HilRefused(Exception):
    pass


def hil_bits_ok(state: McuState) -> bool:
    f = state.flags
    return bool(f & StateFlag.HIL_PLANT) and bool(f & StateFlag.OUTPUTS_LOCKED) and not f & StateFlag.COMMISSION_BUILD


class LoopTtyTransport:
    fake_mcu = False  # the device block's own flag: never the internal fake MCU

    def __init__(self, fd: int, *, clock=time.monotonic_ns, writer=os.write, reader=os.read) -> None:
        self.fd = int(fd)
        self.clock, self._write, self._read = clock, writer, reader
        self._poison_at: int | None = 0  # the poison's next byte (None: done); every opening starts with it
        self._cur = None  # the frame being written
        self._cur_at = 0  # its next byte
        self.partial_since_ns: int | None = None
        self.lost: dict | None = None
        self._dec = LinkDecoder()
        self._boot: int | None = None
        self._boot_bad = False
        self.hil_plant = False
        self.judged: dict = {}  # seq -> judged clock value of the frame's last successful write
        self.first_byte_ns: dict = {}
        self.written_ns: dict = {}
        self.resyncs = 0
        self.read_done_ns: int | None = None

    # --- the loop's contract -------------------------------------------------------------------------------------
    @property
    def partial_frame(self):
        return self._cur

    def receive(self, host, now_ns: int, limit: int = 64) -> None:
        if self.lost is not None:
            raise DeviceLost(self.lost["reason"])
        try:
            data = self._read(self.fd, READ_CAP)
        except BlockingIOError:
            return
        except OSError as exc:
            if exc.errno in _LOST:
                self._lose("read_eio")
            raise
        if not data:
            self._lose("read_eof")
        rx_ns = int(self.clock())  # the actual read time (design 2): the host's echo validity starts here
        self.read_done_ns = rx_ns
        for rec in self._dec.feed(data, rx_ns):
            if isinstance(rec.message, McuState):
                self._check_hil(rec.message)
        host.on_bytes(data, rx_ns)

    def transmit(self, host, now_ns: int, gate) -> list:
        if self.lost is not None:
            raise DeviceLost(self.lost["reason"])
        out: list = []
        if not self._poison_out():
            return out
        if self._cur is not None:
            f = self._cur
            finished = {}

            def finish(*, judged_ns=None):
                finished["done"] = self._write_rest(f, judged_ns)

            if gate(f, finish):
                if finished.get("done"):
                    out.append((f.kind, f.seq, self.written_ns[int(f.seq) & 0xFFFF]))
                    host.frame_written(f, self.written_ns[int(f.seq) & 0xFFFF])
                    self._cur, self.partial_since_ns = None, None
                else:
                    return out  # still half written (EAGAIN)
            else:
                self.resync(host, now_ns)  # a refused continuation: abandoned on this fd, the poison next
                if not self._poison_out():
                    return out
        while (f := host.next_frame(now_ns)) is not None:
            state = {}

            def write(f=f, *, judged_ns=None):
                state["r"] = self._write_first(f, judged_ns)

            passed = gate(f, write)
            r = state.get("r")
            if not passed:
                host.unsent(f, now_ns)
                if not getattr(gate, "keep", False):
                    host.drop_drives("send_gate")
                break
            if r == "eagain":
                host.unsent(f, now_ns)  # not one byte out: back to the queue head (judged again next time)
                break
            if r == "partial":
                break
            out.append((f.kind, f.seq, self.written_ns[int(f.seq) & 0xFFFF]))
            host.frame_written(f, self.written_ns[int(f.seq) & 0xFFFF])
        return out

    def resync(self, host, now_ns: int) -> None:
        """A half frame abandoned on the same fd (counted as possibly delivered); the poison goes out before anything
        else. The receive side and the HIL check are left as they are (one continuous stream)."""
        if self._cur is not None:
            host.frame_abandoned(self._cur, self._cur_at, now_ns)
        self._cur, self._cur_at, self.partial_since_ns = None, 0, None
        self._poison_at = 0
        self.resyncs += 1

    # --- writing ---------------------------------------------------------------------------------------------------
    def _os_write(self, data: bytes, path: str) -> int | None:
        """Bytes written, or None on EAGAIN; a lost device raises DeviceLost."""
        try:
            return self._write(self.fd, data)
        except BlockingIOError:
            return None
        except OSError as exc:
            if exc.errno in _LOST:
                self._lose(path)
            raise

    def _poison_out(self) -> bool:
        while self._poison_at is not None:
            n = self._os_write(POISON[self._poison_at:], "write_eio")
            if not n:
                return False
            self._poison_at += n
            if self._poison_at >= len(POISON):
                self._poison_at = None
        return True

    def _write_first(self, f, judged_ns) -> str:
        seq = int(f.seq) & 0xFFFF
        self.first_byte_ns[seq] = int(self.clock())
        n = self._os_write(f.data, "write_eio")
        if not n:
            return "eagain"
        self.judged[seq] = judged_ns
        if n < len(f.data):
            self._cur, self._cur_at, self.partial_since_ns = f, n, int(self.clock())
            return "partial"
        self.written_ns[seq] = int(self.clock())
        return "done"

    def _write_rest(self, f, judged_ns) -> bool:
        seq = int(f.seq) & 0xFFFF
        while self._cur_at < len(f.data):
            n = self._os_write(f.data[self._cur_at:], "write_eio")
            if not n:
                return False
            self.judged[seq] = judged_ns
            self._cur_at += n
        self.written_ns[seq] = int(self.clock())
        return True

    # --- the HIL check and the loss ----------------------------------------------------------------------------------
    def _check_hil(self, s: McuState) -> None:
        if s.boot_id != self._boot:
            self._boot, self._boot_bad = s.boot_id, False
        if not hil_bits_ok(s):
            self._boot_bad = True
        self.hil_plant = not self._boot_bad

    def _lose(self, path: str) -> None:
        if self.lost is None:
            tainted = self._cur is not None or self._poison_at is not None
            self.lost = {"first": path, "stream_tainted": tainted,
                         "reason": f"device_lost:first={path}:stream_tainted={int(tainted)}"}
        self.hil_plant = False
        raise DeviceLost(self.lost["reason"])


# --- the runner's own pty and model ---------------------------------------------------------------------------------
def _other_openers(rdev: int, own_fd: int) -> list[int]:
    """Pids (other than this process's own fd) holding an fd on the device ``rdev`` (Linux /proc)."""
    me, out = os.getpid(), []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            fds = os.listdir(f"/proc/{pid}/fd")
        except OSError:
            continue
        for fd in fds:
            try:
                st = os.stat(f"/proc/{pid}/fd/{fd}")
            except OSError:
                continue
            if st.st_rdev == rdev and not (int(pid) == me and int(fd) == own_fd):
                out.append(int(pid))
                break
    return out


def _raw(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = (attrs[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)) | termios.CS8 | termios.CREAD \
        | termios.CLOCAL
    attrs[6][termios.VMIN], attrs[6][termios.VTIME] = 0, 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


class HilPty:
    """The runner's pty and its MCU model child (design 1). ``slave`` is the transport's fd."""

    def __init__(self, model: str | Path, duration_s: float, extra_args=(), *, before_spawn=None) -> None:
        if not 0 < float(duration_s) <= 3600:
            raise HilRefused("duration_s")
        for a in extra_args:
            if str(a) in RESERVED_MODEL_ARGS:
                raise HilRefused(f"hil_model_arg_reserved:{a}")
        master, slave = os.openpty()
        try:
            if before_spawn is not None:
                before_spawn(master, slave)  # a test hook between openpty and the lock (a writer or an opener)
            fcntl.ioctl(slave, termios.TIOCEXCL)  # no later non-root opener
            os.set_blocking(master, False)
            os.set_blocking(slave, False)
            os.set_inheritable(slave, False)
            _raw(slave)
            others = _other_openers(os.fstat(slave).st_rdev, slave)
            if others:
                raise HilRefused(f"device_shared:{others[0]}")
            while True:  # whatever reached the slave before the lock: gone before the model reads
                try:
                    if not os.read(master, 4096):
                        break
                except BlockingIOError:
                    break
            args = [str(model), "--link-fd", str(master), "--hil-plant", "1",
                    "--duration-ms", str(int(round(float(duration_s) * 1000)) + 5000), *map(str, extra_args)]
            self.proc = subprocess.Popen(args, pass_fds=(master,))
        except BaseException:
            os.close(master)
            os.close(slave)
            raise
        os.close(master)
        self.slave = slave

    def poll(self):
        return self.proc.poll()

    def close(self, wait_s: float = 1.0) -> int | None:
        try:
            os.close(self.slave)
        except OSError:
            pass
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=wait_s)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        return self.proc.returncode


__all__ = ["DeviceLost", "HIL_FIRST_STATE_S", "HilPty", "HilRefused", "LoopTtyTransport", "POISON",
           "RESERVED_MODEL_ARGS", "hil_bits_ok"]
