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
  COMMISSION_BUILD (a boot change starts over); ``hil_losses`` counts every loss of it -- a STATE without the bits on a
  boot that had them, a lost device -- so a loss and a new boot in one read are still seen as a loss (PR 7 7r-2);
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
import select
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
HIL_STARTUP_MAX_S = 5.0  # 6a-2b (design 10.3): the model's start to the session's start (first STATE, assembly, console)
HIL_MODEL_EXTRA_S = HIL_STARTUP_MAX_S + 1.0 + 5.0  # + the closing DISARM's confirmation + margin: the model's own end


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
        self.hil_boot: int | None = None  # the boot the HIL check holds for (6a-2b design 10.1)
        # the read time of the first STATE of the current boot: a replay's boot boundary (PR 7 7r) -- nothing acquired
        # before it is given after a boot change
        self.hil_boot_since_ns: int | None = None
        self.hil_losses = 0  # every loss of the check, even one a later STATE of the same read hides (7r-2)
        self.inject = None  # a test hook: inject(fd, data, frame) -> int | None to replace one write (6a-2b 10.2)
        self.on_event = None  # the run record's sink: a half write, its abandonment, the poison finished (10.8)
        self.judged: dict = {}  # seq -> judged clock value of the frame's last successful write
        self.first_byte_ns: dict = {}  # seq -> clock just before its first os.write (D5's round trip)
        self.last_write_ns: dict = {}  # seq -> clock just before the os.write that sent its last byte
        self.written_ns: dict = {}  # seq -> clock after that os.write returned (the book's and the permit's)
        self.written_data: dict = {}  # seq -> the frame's bytes once written whole (the record's E: its echo, validity)
        self._host = None
        self.resyncs = 0
        self.read_done_ns: int | None = None

    # --- the loop's contract -------------------------------------------------------------------------------------
    @property
    def partial_frame(self):
        return self._cur

    def receive(self, host, now_ns: int, limit: int = 64) -> None:
        """One read of at most READ_CAP bytes (``limit`` is not counted here: the byte cap bounds a read to about 45
        STATEs). On a raw tty with VMIN 0 an empty read is "nothing yet"; a hangup shows as POLLHUP/POLLERR or as EIO
        (6a-2a impl round 1 P1-1)."""
        self._host = host
        if self.lost is not None:
            raise DeviceLost(self.lost["reason"])
        try:
            data = self._read(self.fd, READ_CAP)
        except BlockingIOError:
            data = b""
        except OSError as exc:
            if exc.errno in _LOST:
                self._lose("read_eio")
            raise
        if not data:
            if self._hung():
                self._lose("read_hup")
            return
        rx_ns = int(self.clock())  # the actual read time (design 2): the host's echo validity starts here
        self.read_done_ns = rx_ns
        for rec in self._dec.feed(data, rx_ns):
            if isinstance(rec.message, McuState):
                self._check_hil(rec.message)
        host.on_bytes(data, rx_ns)

    def _hung(self) -> bool:
        try:
            poller = select.poll()
            poller.register(self.fd, select.POLLIN | select.POLLHUP | select.POLLERR)
            return any(ev & (select.POLLHUP | select.POLLERR | select.POLLNVAL) for _, ev in poller.poll(0))
        except (OSError, ValueError):
            return True

    def transmit(self, host, now_ns: int, gate) -> list:
        """Frames written whole in this call. A device lost part-way returns the ones finished before it (the loop
        books them); the next call raises (6a-2a impl round 1 P2-5)."""
        self._host = host
        if self.lost is not None:
            raise DeviceLost(self.lost["reason"])
        out: list = []
        try:
            self._transmit(host, now_ns, gate, out)
        except DeviceLost:
            if not out:
                raise
        return out

    def _transmit(self, host, now_ns: int, gate, out: list) -> None:
        if not self._poison_out():
            return
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
                    return  # still half written: the next piece passes the gate again
            else:
                self.resync(host, now_ns)  # a refused continuation: abandoned on this fd, the poison next
                if not self._poison_out():
                    return
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

    def resync(self, host, now_ns: int) -> None:
        """A half frame abandoned on the same fd (counted as possibly delivered); the poison goes out before anything
        else. The receive side and the HIL check are left as they are (one continuous stream)."""
        if self._cur is not None:
            host.frame_abandoned(self._cur, self._cur_at, now_ns)
            self._event({"t": "tx_abandoned", "seq": int(self._cur.seq) & 0xFFFF, "kind": self._cur.kind,
                         "bytes": int(self._cur_at), "ns": int(self.clock())})
        self._cur, self._cur_at, self.partial_since_ns = None, 0, None
        self._poison_at = 0
        self.resyncs += 1

    # --- writing ---------------------------------------------------------------------------------------------------
    def _os_write(self, data: bytes, path: str, frame=None) -> int | None:
        """Bytes written, or None on EAGAIN; a lost device raises DeviceLost."""
        try:
            if self.inject is not None:
                n = self.inject(self.fd, bytes(data), frame if frame is not None else self._cur)
                if n is not None:
                    return n if n > 0 else None
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
                self._event({"t": "tx_poison_done", "ns": int(self.clock())})
        return True

    def _write_first(self, f, judged_ns) -> str:
        seq = int(f.seq) & 0xFFFF
        entry = int(self.clock())
        self.first_byte_ns[seq] = entry
        n = self._os_write(f.data, "write_eio", f)
        if not n:
            return "eagain"
        self.judged[seq], self.last_write_ns[seq] = judged_ns, entry
        if n < len(f.data):
            self._cur, self._cur_at, self.partial_since_ns = f, n, int(self.clock())
            self._event({"t": "tx_partial", "seq": seq, "kind": f.kind, "bytes": int(n), "ns": self.partial_since_ns})
            return "partial"
        self.written_ns[seq] = int(self.clock())
        self.written_data[seq] = bytes(f.data)
        if len(self.written_data) > 4096:
            self.written_data.pop(next(iter(self.written_data)))
        return "done"

    def _write_rest(self, f, judged_ns) -> bool:
        """One os.write per gate pass (6a-2a impl round 1 P1-2): a short write leaves the rest for the next pass."""
        seq = int(f.seq) & 0xFFFF
        entry = int(self.clock())
        n = self._os_write(f.data[self._cur_at:], "write_eio", f)
        if not n:
            return False
        self.judged[seq], self.last_write_ns[seq] = judged_ns, entry
        self._cur_at += n
        if self._cur_at < len(f.data):
            return False
        self.written_ns[seq] = int(self.clock())
        self.written_data[seq] = bytes(f.data)
        if len(self.written_data) > 4096:
            self.written_data.pop(next(iter(self.written_data)))
        return True

    def _event(self, ev: dict) -> None:
        if self.on_event is not None:
            try:
                self.on_event(ev)
            except Exception:  # noqa: BLE001 -- the record never holds a write
                pass

    # --- the HIL check and the loss ----------------------------------------------------------------------------------
    def _check_hil(self, s: McuState) -> None:
        if s.boot_id == 0:
            return  # the host refuses it too (state_invalid): neither opens the check nor resets a boot's record
        if s.boot_id != self._boot:
            self._boot, self._boot_bad = s.boot_id, False
            self.hil_boot_since_ns = self.read_done_ns
        if not hil_bits_ok(s):
            if not self._boot_bad:
                self.hil_losses += 1
            self._boot_bad = True
        self.hil_plant = not self._boot_bad
        self.hil_boot = self._boot if self.hil_plant else None

    def peer_exited(self) -> None:
        """The runner saw the model end (6a-2b design 10.3): the loss fixed now with the half frame / poison state as it
        is at this moment, the first path ``model_poll``; does not raise."""
        try:
            self._lose("model_poll")
        except DeviceLost:
            pass

    def _lose(self, path: str) -> None:
        if self.lost is None:
            tainted = self._cur is not None or self._poison_at is not None
            self.lost = {"first": path, "stream_tainted": tainted,
                         "reason": f"device_lost:first={path}:stream_tainted={int(tainted)}"}
            if self._host is not None:
                try:
                    self._host.disconnected(int(self.clock()))  # the queue and the authorization go (once)
                except Exception:  # noqa: BLE001 -- the loss is raised whatever the host does
                    pass
        if self.hil_plant:
            self.hil_losses += 1
        self.hil_plant = False
        raise DeviceLost(self.lost["reason"])


class RealTtyTransport(LoopTtyTransport):
    """The operating REAL link (docs/plans/2026-11-26-real-operating-assembly.md 2.1): the loop contract of
    ``LoopTtyTransport`` on the real board's USB CDC tty, opened (``open``) with the executor lock taken on that very
    fd before anything is set on the port, a pty refused by the opened fd's own type, and the board's USB product
    string and serial read from that fd (the e-stop waiver, the precondition record's binds). Never a HIL check:
    ``hil_plant`` stays False.

    ``real_operating`` (the device block's third way, ``real_loop.motion_allowed``) is true only after ``grant`` and
    until the first of: a revocation, a lost device (never reopened), a STATE of another boot than the one granted on,
    a STATE showing a HIL or commissioning build or locked outputs. ``grant`` takes the judgement itself and refuses
    unless the lock is held, a first STATE was read on this fd, the judgement is empty, the boot named is the one the
    board last showed and no bad build was ever seen. Every ARM/DRIVE byte -- the first and each continuation -- is
    written only while the permission holds; every other send check (the host's ``drive_refusal``, freshness, the gate)
    stays."""

    _BAD_BUILD = StateFlag.HIL_PLANT | StateFlag.COMMISSION_BUILD | StateFlag.OUTPUTS_LOCKED

    def __init__(self, fd: int, *, usb_product: str | None, usb_serial: str | None, locked: bool = False,
                 **kw) -> None:
        super().__init__(fd, **kw)
        self.usb_product, self.usb_serial = usb_product, usb_serial
        self.locked = bool(locked)
        self._token = None
        self._grant_boot: int | None = None
        self._last_boot: int | None = None
        self.first_state: McuState | None = None  # the first STATE read on this fd (the precondition's board facts)
        self.bad_build_seen = False
        self.revoked: str | None = None

    @classmethod
    def open(cls, path: str, *, opener=None, lock=None, product_reader=None, serial_reader=None, allow_pty=False,
             **kw):
        """Open ``path`` raw with the executor lock on the fd first (``DeviceBusy`` if held; the fd closed on any
        failure), refuse a pty by the opened fd's type, then read the USB identity from the same fd."""
        import stat as stat_

        import real_mcu_link as rml

        from forklift_core.live.executor_lock import lock_fd
        from forklift_core.live.model_peer import PTY_SLAVE_MAJORS

        opener = (lambda p, before_config=None: rml.open_raw(p, before_config=before_config, strict=True)) \
            if opener is None else opener
        lock = (lambda fd: lock_fd(fd, path)) if lock is None else lock
        fd = opener(path, before_config=lock)
        try:
            st = os.fstat(fd)
            if not allow_pty and stat_.S_ISCHR(st.st_mode) and os.major(st.st_rdev) in PTY_SLAVE_MAJORS:
                raise OSError(errno.ENOTTY, "a real MCU never links by pty", path)
            product = (rml.usb_product_of_fd if product_reader is None else product_reader)(fd)
            serial = (rml.usb_serial_of_fd if serial_reader is None else serial_reader)(fd)
            return cls(fd, usb_product=product, usb_serial=serial, locked=True, **kw)
        except BaseException:
            os.close(fd)
            raise

    @property
    def real_operating(self) -> bool:
        return self._token is not None and self.lost is None and self.revoked is None

    def grant(self, judgement, boot_id: int | None) -> bool:
        """The run's permission once the operating preconditions held: ``judgement`` is their list, which must be
        empty. False (nothing granted): not locked, no first STATE, a non-empty judgement, a boot other than the one
        the board last showed, a bad build seen, already revoked or lost."""
        if (not self.locked or self.first_state is None or not isinstance(judgement, list) or judgement
                or not self.usb_product or not self.usb_serial
                or boot_id in (None, 0) or int(boot_id) != int(self.first_state.boot_id)
                or int(boot_id) != self._last_boot or self.bad_build_seen
                or self.revoked is not None or self.lost is not None):
            return False  # the judged boot (the first STATE's) must still be the board's current one
        self._token, self._grant_boot = object(), int(boot_id)
        self._event({"t": "real_operating", "granted": True, "boot": int(boot_id), "ns": int(self.clock())})
        return True

    def revoke(self, reason: str) -> None:
        if self.revoked is None:
            self.revoked = str(reason)
            self._token = None
            self._event({"t": "real_operating", "granted": False, "reason": self.revoked, "ns": int(self.clock())})

    def _check_hil(self, s: McuState) -> None:
        """Not a HIL check (``hil_plant`` stays False): the first STATE is kept; a STATE of a HIL or commissioning
        build or with locked outputs, or of another boot than the granted one, revokes the permission."""
        if s.boot_id == 0:
            return
        if self.first_state is None:
            self.first_state = s
        self._last_boot = int(s.boot_id)
        if s.flags & self._BAD_BUILD:
            self.bad_build_seen = True
            self.revoke("board_build_changed")
        if self._grant_boot is not None and int(s.boot_id) != self._grant_boot:
            self.revoke("boot_changed")

    @property
    def last_boot(self) -> int | None:
        return self._last_boot

    def _motion_frame(self, f) -> bool:
        return getattr(f, "kind", None) in ("ARM", "DRIVE")

    def _write_first(self, f, judged_ns) -> str:
        if self._motion_frame(f) and not self.real_operating:
            return "eagain"  # not one byte: back to the queue (the next gate refuses it without the permission)
        return super()._write_first(f, judged_ns)

    def _write_rest(self, f, judged_ns) -> bool:
        if self._motion_frame(f) and not self.real_operating:
            return False  # no continuation: the next gate refuses it and the half frame is abandoned (resync)
        return super()._write_rest(f, judged_ns)

    def _lose(self, path: str) -> None:
        self.revoke(f"device_lost:{path}")
        super()._lose(path)


# --- the runner's own pty and model ---------------------------------------------------------------------------------
def _other_openers(rdev: int, own_fd: int) -> tuple[list[int], list[int]]:
    """``(holders, uninspected)``: pids (other than this process's own fd) holding an fd on the device ``rdev``, and
    pids of this user whose fds could not be read (non-dumpable agents; recorded, not refused -- the lock comes first
    and the master is drained, so only an opener inside the openpty-to-lock instant could hold it). Another user's
    process cannot open this user's slave read-write (mode 0620) and is skipped; an fd that vanished is skipped."""
    me, uid, out, unseen = os.getpid(), os.getuid(), [], []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            owner = os.stat(f"/proc/{pid}").st_uid
        except FileNotFoundError:
            continue
        except PermissionError:
            unseen.append(int(pid))  # its owner unknown: recorded (impl round 2 P3)
            continue
        try:
            fds = os.listdir(f"/proc/{pid}/fd")
        except FileNotFoundError:
            continue
        except PermissionError:
            if owner == uid:
                unseen.append(int(pid))
            continue
        for fd in fds:
            try:
                st = os.stat(f"/proc/{pid}/fd/{fd}")
            except FileNotFoundError:
                continue
            except PermissionError:
                if owner == uid and int(pid) not in unseen:
                    unseen.append(int(pid))
                continue
            if st.st_rdev == rdev and not (int(pid) == me and int(fd) == own_fd):
                out.append(int(pid))
                break
    return out, unseen


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
            others, self.uninspected = _other_openers(os.fstat(slave).st_rdev, slave)
            if others:
                raise HilRefused(f"device_shared:{others[0]}")
            while True:  # whatever reached the slave before the lock: gone before the model reads
                try:
                    if not os.read(master, 4096):
                        break
                except BlockingIOError:
                    break
            args = [str(model), "--link-fd", str(master), "--hil-plant", "1",
                    "--duration-ms", str(int(round((float(duration_s) + HIL_MODEL_EXTRA_S) * 1000))),
                    *map(str, extra_args)]
            # its own session (process group): a terminal Ctrl-C reaches the runner, which confirms the closing DISARM
            # before it ends the model (6a-2b design 10.3)
            self.proc = subprocess.Popen(args, pass_fds=(master,), start_new_session=True)
        except BaseException:
            os.close(master)
            os.close(slave)
            raise
        os.close(master)
        self.slave = slave

    def poll(self):
        return self.proc.poll()

    def close(self, wait_s: float = 1.0) -> int | None:
        if self.slave >= 0:
            fd, self.slave = self.slave, -1  # released once: a second close never closes a reused number
            try:
                os.close(fd)
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


__all__ = ["DeviceLost", "HIL_FIRST_STATE_S", "HIL_MODEL_EXTRA_S", "HIL_STARTUP_MAX_S", "HilPty", "HilRefused", "LoopTtyTransport", "POISON",
           "RESERVED_MODEL_ARGS", "hil_bits_ok"]
