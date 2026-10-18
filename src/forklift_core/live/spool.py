"""The record spool of a long-lived live session (persistent console plan D8).

A live session keeps running for hours; its audit records (the samples, the SLAM log, the scans and plans, the
frames' records) would grow without end in memory. Each registered list keeps its last ``keep`` items -- the ones the
running code reads (``[-1]``) -- and every ``interval_s`` the rest is cut off and handed, as it is, to a writer thread
that stores it under ``<root>/<name>/<n>.jsonl`` (a group of aligned lists: ``.npz``). What was cut off is no longer
reachable from the running code, so nothing changes it while it is written. The control thread only cuts: it never
waits on the disk. A queue over ``queue_items`` drops the chunk (counted); a failed write is counted too. ``close``
writes ``manifest.json``: per name the chunks and items written and the items dropped, and the errors.
"""

from __future__ import annotations

import json
import threading
from collections import deque
from pathlib import Path

import numpy as np


def _jsonable(value):
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return str(value)


class RecordSpool:
    def __init__(self, root, *, interval_s: float = 10.0, queue_items: int = 500_000) -> None:
        self.root = Path(root)
        self.interval_s, self.queue_items = float(interval_s), int(queue_items)
        self._lists: dict = {}
        self._groups: dict = {}
        self._stats: dict = {}
        self._chunk_no: dict = {}
        self._errors: list = []
        self._queue: deque = deque()
        self._queued = 0
        self._cond = threading.Condition()
        self._stop = False
        self._running = threading.Event()
        self._running.set()
        self._last = 0.0
        self._thread = threading.Thread(target=self._write_loop, name="record-spool", daemon=True)
        self._thread.start()

    # --- the control thread -----------------------------------------------------------------------------------------
    def register(self, name: str, items: list, *, keep: int = 1) -> None:
        self._lists[name] = (items, int(keep))
        self._stats.setdefault(name, {"chunks": 0, "items": 0, "dropped_items": 0})

    def register_group(self, name: str, lists: dict, *, keep: int = 1) -> None:
        """Lists appended together (one entry each per step): cut at one length, written as one .npz."""
        self._groups[name] = (lists, int(keep))
        self._stats.setdefault(name, {"chunks": 0, "items": 0, "dropped_items": 0})

    def tick(self, now: float) -> None:
        if now - self._last < self.interval_s:
            return
        self._last = now
        for name, (items, keep) in self._lists.items():
            n = len(items) - keep
            if n > 0:
                cut = items[:n]
                del items[:n]
                self._enqueue(name, "jsonl", cut, n)
        for name, (lists, keep) in self._groups.items():
            n = min((len(v) for v in lists.values()), default=0) - keep
            if n > 0:
                cut = {}
                for key, values in lists.items():
                    cut[key] = values[:n]
                    del values[:n]
                self._enqueue(name, "npz", cut, n)

    def _enqueue(self, name: str, kind: str, payload, n: int) -> None:
        with self._cond:
            if self._queued + n > self.queue_items:
                self._stats[name]["dropped_items"] += n
                return
            self._queue.append((name, kind, payload, n))
            self._queued += n
            self._cond.notify()

    # --- the writer thread ------------------------------------------------------------------------------------------
    def pause_writer(self) -> None:  # tests: a slow disk
        self._running.clear()

    def resume_writer(self) -> None:
        self._running.set()

    def _write_loop(self) -> None:
        while True:
            self._running.wait()
            with self._cond:
                while not self._queue and not self._stop:
                    self._cond.wait(0.5)
                if not self._queue and self._stop:
                    return
                name, kind, payload, n = self._queue.popleft()
            try:
                self._write(name, kind, payload)
                ok = True
            except (OSError, TypeError, ValueError) as exc:
                ok = False
                self._errors.append(f"{name}: {type(exc).__name__}: {exc}")
            with self._cond:
                self._queued -= n
                stats = self._stats[name]
                if ok:
                    stats["chunks"] += 1
                    stats["items"] += n
                else:
                    stats["dropped_items"] += n

    def _write(self, name: str, kind: str, payload) -> None:
        folder = self.root / name
        folder.mkdir(parents=True, exist_ok=True)
        number = self._chunk_no[name] = self._chunk_no.get(name, 0) + 1
        path = folder / f"{number:06d}.{kind}"
        if kind == "jsonl":
            path.write_text("".join(json.dumps(item, default=_jsonable) + "\n" for item in payload))
        else:
            np.savez(path, **{key: np.asarray(values) for key, values in payload.items()})

    def close(self, timeout_s: float = 60.0) -> dict:
        """Everything queued written (within ``timeout_s``), then the manifest; returns it."""
        self._running.set()
        with self._cond:
            self._stop = True
            self._cond.notify()
        self._thread.join(timeout_s)
        with self._cond:
            for name, kind, payload, n in self._queue:  # not written in time: counted as dropped
                self._stats[name]["dropped_items"] += n
            self._queue.clear()
            manifest = {name: dict(stats) for name, stats in self._stats.items()}
        manifest["errors"] = list(self._errors)
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "manifest.json").write_text(json.dumps(manifest, indent=1))
        return manifest
