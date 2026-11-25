"""slam_toolbox instances as the real bridge's direct children (docs/design/2026-11-25-real-sources-pr7.md 6.1, 2).

The node executable is started itself, in a new process group: ``ros2 run`` exits before the node it started, which is
then reparented and left a zombie no one reaps (PR 7c-1a premise test, ws1 jobs 21220-21222). Stopping sends SIGTERM to
the group, waits for every member to leave, then SIGKILL, and reaps the child; the caller's ``graph_has(k)`` must
then turn false and ``drain(k)`` empties the bridge's subscription queues before the next instance starts.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path


def group_members(pgid: int) -> list[int]:
    """Live (not zombie) processes whose process group is ``pgid``."""
    out = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text()
        except OSError:
            continue
        fields = stat[stat.rfind(")") + 2:].split()
        if len(fields) > 2 and fields[2] == str(pgid) and fields[0] != "Z":
            out.append(int(entry.name))
    return out


class SlamInstances:
    def __init__(self, argv_for, *, log_dir: Path, graph_has=lambda k: False, drain=lambda k: None,
                 term_s: float = 1.0, kill_s: float = 1.0, graph_s: float = 2.0) -> None:
        """``argv_for(k, start_pose)``: the command line of instance ``k`` (the node executable first). The bounds
        keep a whole stop -- SIGTERM, SIGKILL, reap, graph -- under the executor's 5 s cancel-to-void bound (design
        6.1, 2; Codex 7c-1a round 1 P2-4)."""
        self.argv_for, self.log_dir = argv_for, Path(log_dir)
        self.graph_has, self.drain = graph_has, drain
        self.term_s, self.kill_s, self.graph_s = term_s, kill_s, graph_s
        self.procs: dict[int, subprocess.Popen] = {}
        self.records: list[dict] = []

    def start(self, k: int, start_pose) -> subprocess.Popen:
        log = open(self.log_dir / f"slam_{k}.log", "w")  # noqa: SIM115 -- the child holds it
        try:
            proc = subprocess.Popen(self.argv_for(k, tuple(start_pose)), stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        finally:
            log.close()
        self.procs[k] = proc
        return proc

    def _wait_empty(self, pgid: int, seconds: float) -> bool:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if not group_members(pgid):
                return True
            time.sleep(0.05)
        return not group_members(pgid)

    def stop(self, k: int) -> bool:
        """Stop instance ``k``: True only when the whole group is gone, the child reaped, the node gone from the graph
        and the queues drained."""
        proc = self.procs.get(k)
        rec = {"k": k, "sigkill": False}
        self.records.append(rec)
        if proc is None:
            rec["ok"] = True
            return True
        t0 = time.monotonic()
        pgid = proc.pid
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        empty = self._wait_empty(pgid, self.term_s)
        if not empty:
            rec["sigkill"] = True
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            empty = self._wait_empty(pgid, self.kill_s)
        try:
            proc.wait(timeout=self.kill_s)
            reaped = True
        except subprocess.TimeoutExpired:
            reaped = False
        if reaped and empty:
            self.procs.pop(k, None)  # kept until the whole group is gone and reaped: stop_all tries again
        gone = True
        deadline = time.monotonic() + self.graph_s
        while self.graph_has(k):
            if time.monotonic() > deadline:
                gone = False
                break
            time.sleep(0.05)
        if empty and reaped and gone:
            self.drain(k)
        rec.update(group_empty=empty, reaped=reaped, graph_gone=gone, stop_s=round(time.monotonic() - t0, 3))
        rec["ok"] = bool(empty and reaped and gone)
        return rec["ok"]

    def stop_all(self) -> None:
        for k in list(self.procs):
            self.stop(k)


__all__ = ["SlamInstances", "group_members"]
