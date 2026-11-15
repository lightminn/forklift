"""The arrival-day rehearsal's model-peer check (docs/design/2026-11-15-arrival-day-rehearsal.md 2.2).

A synthetic stand-in (the floor permit, the synthetic record marker) may open only when the link device is the slave
of a pty whose master the rehearsal bench's ``mcu_model`` -- and nothing else -- holds. mcu-link STATE cannot tell the
model from a board (its eight flag bits are all used), so the check is on the device and the processes:

1. the device (followed) is a Unix98 pty slave (major 136-143) and the bench's pty (st_rdev);
2. the bench's model pid is alive, started when the bench recorded (no reused pid) and its exe is ``mcu_model``;
3. the model holds a ``/dev/ptmx`` fd whose tty index is the device's pts number;
4. no other process holds that master -- a complete scan of the same uid's processes; a process that exits while
   it is being read is skipped (it holds nothing now), and any other read error twice refuses. Two stated limits: other uids cannot be read, and a same-uid process whose fds
   cannot be read (non-dumpable) is tolerated only if it started before the model -- either could hold this master
   only if one of our processes passed it. Root is refused.

Returns None (may open) or the reason. ``proc`` is injectable; the default reads /proc and stat.
"""

from __future__ import annotations

import errno
import os
import time
from pathlib import Path

PTY_SLAVE_MAJORS = range(136, 144)
BENCH_SCHEMA = "rehearsal_bench/1"
MODEL_EXE_NAME = "mcu_model"


class SystemProc:
    """The live /proc and stat view."""

    @property
    def euid(self) -> int:
        return os.geteuid()

    def device(self, path):
        st = os.stat(path)  # follows links
        import stat as stat_

        return stat_.S_ISCHR(st.st_mode), st.st_rdev

    def pids(self):
        return sorted(int(p) for p in os.listdir("/proc") if p.isdigit())

    def uid(self, pid):
        return os.stat(f"/proc/{pid}").st_uid

    def starttime(self, pid):
        text = Path(f"/proc/{pid}/stat").read_text()
        return int(text[text.rindex(")") + 2:].split()[19])  # field 22: after the comm's closing parenthesis

    def exe(self, pid):
        return os.readlink(f"/proc/{pid}/exe")

    def exists(self, pid):
        return os.path.exists(f"/proc/{pid}")

    def fds(self, pid):
        """{fd: target}; an fd closed between the listing and its read is left out (it holds nothing now -- the
        listing's own directory fd in this process is one). The process gone raises, as the scan rule wants."""
        base = f"/proc/{pid}/fd"
        out = {}
        for fd in os.listdir(base):
            try:
                out[int(fd)] = os.readlink(f"{base}/{fd}")
            except FileNotFoundError:
                if not os.path.exists(f"/proc/{pid}"):
                    raise
        return out

    def tty_index(self, pid, fd):
        for line in Path(f"/proc/{pid}/fdinfo/{fd}").read_text().splitlines():
            if line.startswith("tty-index:"):
                return int(line.split(":", 1)[1])
        return None


class _Incomplete(Exception):
    pass


def _masters(proc, pid) -> dict[int, int | None]:
    """{fd: tty index} of the pty masters ``pid`` holds."""
    out = {}
    for fd, target in proc.fds(pid).items():
        if target == "/dev/ptmx" or target.startswith("/dev/pts/ptmx"):
            out[fd] = proc.tty_index(pid, fd)
    return out


def _gone(proc, pid) -> bool:
    try:
        return not proc.exists(pid)
    except OSError:
        return False


def _other_holders(proc, index: int, model_pid: int, model_start: int, skipped: list) -> list[int]:
    """Same-uid processes other than the model holding the master of pts ``index``. A same-uid process whose fds
    cannot be read (non-dumpable session components: sd-pam, the compositor, polkit agents) is tolerated only if it
    started before the model -- it could hold this master only if one of our processes passed it later -- and is
    listed in ``skipped``; one started after the model, or any other read failure, is _Incomplete. A process that
    exits between the listing and its reads is skipped: gone, it holds no master (a busy host -- parallel test
    workers -- churns short-lived processes faster than two scans can avoid). A not-found error on a pid that
    still exists (reused by a new process) is _Incomplete."""
    holders = []
    for pid in proc.pids():
        if pid == model_pid:
            continue
        try:
            if proc.uid(pid) != proc.euid:
                continue  # another uid: not readable (the module's stated limit)
            try:
                masters = _masters(proc, pid)
            except PermissionError:
                if proc.starttime(pid) < model_start:
                    skipped.append(pid)
                    continue
                raise
            if index in masters.values():
                holders.append(pid)
        except OSError as exc:
            not_found = isinstance(exc, (FileNotFoundError, ProcessLookupError)) or exc.errno in (errno.ENOENT,
                                                                                                    errno.ESRCH)
            if not_found and _gone(proc, pid):
                continue
            if not_found or isinstance(exc, PermissionError) or exc.errno in (errno.EACCES, errno.EPERM):
                raise _Incomplete(pid, exc) from exc
            raise
    return holders


def model_alive_refusal(device: str, bench: dict, *, proc=None) -> str | None:
    """The cheap recheck a control loop can afford (conditions 1-2 and the model's own master, no scan of other
    processes): the full ``model_peer_refusal`` scan takes tens of milliseconds and runs once, at start
    (arrival-day rehearsal: run inside the teleop bridge's 20 ms loop every second it stalled the loop past the
    50 ms STATE age gate)."""
    proc = SystemProc() if proc is None else proc
    try:
        is_chr, rdev = proc.device(device)
    except OSError:
        return "no_device"
    if not is_chr or os.major(rdev) not in PTY_SLAVE_MAJORS:
        return "not_a_pty"
    if rdev != bench.get("pts_rdev"):
        return "not_the_bench_pty"
    pid = bench.get("model_pid")
    try:
        if proc.starttime(pid) != bench.get("model_starttime"):
            return "model_restarted"
        if proc.exe(pid) != bench.get("model_exe"):
            return "not_mcu_model"
        if os.minor(rdev) not in _masters(proc, pid).values():
            return "model_holds_no_master"
    except (FileNotFoundError, ProcessLookupError):
        return "model_gone"
    except OSError:
        return "model_unreadable"
    return None


SCAN_ATTEMPTS = 6  # a full scan repeated this many times before an incomplete scan refuses ...
SCAN_RETRY_S = 0.25  # ... this far apart: a short-lived non-dumpable same-uid process (an sshd session of a status
# probe, seen on ws1 2026-11-15) is gone by then; each attempt is the whole fail-closed scan, so nothing is relaxed


def model_peer_refusal(device: str, bench: dict, *, proc=None, detail: list | None = None,
                       attempts: int = SCAN_ATTEMPTS, sleep=time.sleep) -> str | None:
    """``detail``, if given, receives ``(pid, error)`` for each process that left a scan incomplete."""
    proc = SystemProc() if proc is None else proc
    if not isinstance(bench, dict) or bench.get("schema") != BENCH_SCHEMA or bench.get("synthetic") is not True:
        return "bad_bench"
    if proc.euid == 0:
        return "running_as_root"
    try:
        is_chr, rdev = proc.device(device)
    except OSError:
        return "no_device"
    if not is_chr or os.major(rdev) not in PTY_SLAVE_MAJORS:
        return "not_a_pty"
    if rdev != bench.get("pts_rdev"):
        return "not_the_bench_pty"
    pid = bench.get("model_pid")
    try:
        if proc.starttime(pid) != bench.get("model_starttime"):
            return "model_restarted"
        exe = proc.exe(pid)
        if exe != bench.get("model_exe") or os.path.basename(exe) != MODEL_EXE_NAME:
            return "not_mcu_model"
        index = os.minor(rdev)
        if index not in _masters(proc, pid).values():
            return "model_holds_no_master"
    except (FileNotFoundError, ProcessLookupError):
        return "model_gone"
    except OSError:
        return "model_unreadable"
    for attempt in range(attempts):
        try:
            if _other_holders(proc, index, pid, bench.get("model_starttime"), []):
                return "master_shared"
            return None
        except _Incomplete as inc:
            if detail is not None:
                detail.append((inc.args[0], repr(inc.args[1]) if len(inc.args) > 1 else None))
            if attempt == attempts - 1:
                return "peer_scan_incomplete"
            if attempt:
                sleep(SCAN_RETRY_S)  # the first repeat is immediate (a process vanishing mid-scan)
    return "peer_scan_incomplete"
