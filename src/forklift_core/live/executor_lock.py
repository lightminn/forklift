"""One executor per MCU device (docs/design/2026-11-19-real-safety-wiring.md 8).

The lock is an exclusive ``flock`` on the very file descriptor a link opens, taken at every opening (a reconnect
included), so it always covers the device that fd reaches: an alias (``/dev/serial/by-id/...``) resolves to the same
device inode and meets the same lock, and a reconnect that lands on another device node is locked afresh. No lock
directory and no lock file: the kernel drops the lock when its holder's fd closes or its process dies, and nothing is
left behind to block the next executor. A refusal names the holder (pid and command line) read from ``/proc/locks``. There is no separate start-up probe: its
own open and close would touch DTR on USB CDC (design 8.1); a tool's first opening is its start-up check.
"""

from __future__ import annotations

import fcntl
import os

__all__ = ["DeviceBusy", "holder", "lock_fd"]


class DeviceBusy(OSError):
    """Another open file description holds the device's executor lock."""

    def __init__(self, path: str, holder_: dict | None) -> None:
        who = "unknown holder" if holder_ is None else f"pid {holder_['pid']} ({holder_['tool']})"
        super().__init__(f"{path} is held by another executor: {who}")
        self.path, self.holder = path, holder_


def _tool(pid: int) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            argv = [a.decode(errors="replace") for a in f.read().split(b"\0") if a]
    except OSError:
        return "?"
    # the program and the script it runs, when there is one (python3 tools/teleop_bridge.py ...)
    return " ".join(os.path.basename(a) for a in argv[:2]) or "?"


def holder(fd: int) -> dict | None:
    """The process holding an exclusive flock on the inode the opened ``fd`` reaches, from ``/proc/locks`` (None: none,
    or unreadable). From the fd, not the path: an alias retargeted since the opening never names another node's holder
    (design 8.1)."""
    try:
        st = os.fstat(fd)
        with open("/proc/locks") as f:
            lines = f.read().splitlines()
    except OSError:
        return None
    key = f"{os.major(st.st_dev):02x}:{os.minor(st.st_dev):02x}:{st.st_ino}"
    for line in lines:
        parts = line.split()
        # "1: FLOCK  ADVISORY  WRITE 12345 00:05:1234 0 EOF" (a blocked waiter's line starts "1: ->")
        if len(parts) >= 6 and parts[1] == "FLOCK" and parts[3] == "WRITE" and parts[5] == key:
            try:
                pid = int(parts[4])
            except ValueError:
                continue
            return {"pid": pid, "tool": _tool(pid)}
    return None


def lock_fd(fd: int, path: str) -> None:
    """Take the executor lock on the opened device ``fd`` (non-blocking); ``DeviceBusy`` names the holder."""
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise DeviceBusy(path, holder(fd)) from None
