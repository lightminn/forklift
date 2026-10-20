"""The record spool of a long-lived live session (persistent console plan D8).

A live session keeps running for hours; its audit records (the samples, the SLAM log, the scans and plans, the
frames' records) would grow without end in memory. Each registered list keeps its last ``keep`` items -- the ones the
running code reads (``[-1]``) -- and every ``interval_s`` the rest is cut off and handed, as it is, to a writer thread
that stores it under ``<root>/<name>/<n>.jsonl`` (a group of aligned lists: ``.npz``). What was cut off is no longer
reachable from the running code, so nothing changes it while it is written. The control thread only cuts: it never
waits on the disk. The queue holds at most ``queue_cuts`` cuts not yet written (each one interval's records, so the
backlog in memory is bounded in time -- whatever the records' sizes); a cut past that is dropped whole (counted); a failed write is counted too. A chunk is written to ``<n>.<kind>.tmp`` and renamed, so a chunk file is
always whole. ``close`` cuts everything left (the tails too), waits for the writer up to a deadline and writes
``manifest.json``: per name the chunks and items written, the items dropped and the items still unsettled at the
deadline, and whether the record is complete.
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

ERRORS_KEPT = 10  # the last write errors kept (with the total count): a failing disk does not grow the list


def _jsonable(value):
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return str(value)


class RecordSpool:
    def __init__(self, root, *, interval_s: float = 10.0, queue_cuts: int = 6) -> None:
        self.root = Path(root)
        self.interval_s, self.queue_cuts = float(interval_s), int(queue_cuts)
        self._lists: dict = {}
        self._groups: dict = {}
        self._stats: dict = {}
        self._chunk_no: dict = {}
        self._errors: deque = deque(maxlen=ERRORS_KEPT)
        self._error_count = 0
        self._queue: deque = deque()
        self._unwritten: dict = {}  # cut number -> its chunks not yet settled (taken ones included: still in memory)
        self._cut_no = 0
        self._inflight = None  # (name, n) of the chunk the writer has taken and not yet settled
        self._cond = threading.Condition()
        self._stop = False
        self._closed = False
        self._manifest = None  # the first close's: a later close returns it (it never rewrites a record as whole)
        self._running = threading.Event()
        self._running.set()
        self._last = 0.0
        self._cuts, self._cut_wall_max = 0, 0.0
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
        started = time.perf_counter()
        self._cut_all(keep_override=None)
        self._cuts += 1  # the control thread's cost per cut (the close's own cut is not a tick)
        self._cut_wall_max = max(self._cut_wall_max, time.perf_counter() - started)

    def _cut_all(self, keep_override) -> None:
        """Cut every list; the chunks go to the writer as one cut, or -- the backlog full -- are dropped (counted).
        The close's cut (keep 0) always goes: nothing runs after it."""
        with self._cond:
            admit = keep_override == 0 or len(self._unwritten) < self.queue_cuts
            self._cut_no += 1
        chunks = []
        for name, (items, keep) in self._lists.items():
            n = len(items) - (keep if keep_override is None else keep_override)
            if n > 0:
                cut = items[:n]
                del items[:n]
                chunks.append((name, "jsonl", cut, n))
        for name, (lists, keep) in self._groups.items():
            n = min((len(v) for v in lists.values()), default=0) - (keep if keep_override is None else keep_override)
            if n > 0:
                cut = {}
                for key, values in lists.items():
                    cut[key] = values[:n]
                    del values[:n]
                chunks.append((name, "npz", cut, n))
        with self._cond:
            for name, kind, payload, n in chunks:
                if admit:
                    self._queue.append((self._cut_no, name, kind, payload, n))
                else:
                    self._stats[name]["dropped_items"] += n
            if admit and chunks:
                self._unwritten[self._cut_no] = len(chunks)
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
                cut_no, name, kind, payload, n = self._queue.popleft()
                self._inflight = (name, n)  # still counted against the bounds: its payload is still in memory
            try:
                self._write(name, kind, payload)
                ok = True
            except (OSError, TypeError, ValueError) as exc:
                ok = False
                error = f"{name}: {type(exc).__name__}: {exc}"
            with self._cond:
                self._inflight = None
                self._unwritten[cut_no] -= 1
                if not self._unwritten[cut_no]:
                    del self._unwritten[cut_no]
                if self._closed:  # settled after the manifest: the manifest already said unsettled
                    continue
                stats = self._stats[name]
                if ok:
                    stats["chunks"] += 1
                    stats["items"] += n
                else:
                    stats["dropped_items"] += n
                    self._errors.append(error)
                    self._error_count += 1

    def _write(self, name: str, kind: str, payload) -> None:
        folder = self.root / name
        folder.mkdir(parents=True, exist_ok=True)
        number = self._chunk_no[name] = self._chunk_no.get(name, 0) + 1
        path = folder / f"{number:06d}.{kind}"
        tmp = folder / f"{number:06d}.{kind}.tmp"
        if kind == "jsonl":
            tmp.write_text("".join(json.dumps(item, default=_jsonable) + "\n" for item in payload))
        else:
            with open(tmp, "wb") as handle:
                np.savez(handle, **{key: np.asarray(values) for key, values in payload.items()})
        os.replace(tmp, path)

    def close(self, timeout_s: float = 60.0) -> dict:
        """Everything left cut (the tails too) and written within ``timeout_s``, then the manifest; returns it.
        Once: a second call returns the first manifest."""
        if self._manifest is not None:
            return self._manifest
        misaligned = {}
        for name, (lists, _keep) in self._groups.items():
            lengths = {key: len(values) for key, values in lists.items()}
            if len(set(lengths.values())) > 1:
                misaligned[name] = lengths
        self._cut_all(keep_override=0)
        self._running.set()
        with self._cond:
            self._stop = True
            self._cond.notify()
        self._thread.join(timeout_s)
        with self._cond:
            self._closed = True
            for name, stats in self._stats.items():
                stats["unsettled_items"] = 0
            for _cut, name, _kind, _payload, n in self._queue:  # never taken: not written
                self._stats[name]["unsettled_items"] += n
            self._queue.clear()
            if self._inflight is not None:  # taken and still being written at the deadline
                self._stats[self._inflight[0]]["unsettled_items"] += self._inflight[1]
            manifest = {name: dict(stats) for name, stats in self._stats.items()}
            for name, lengths in misaligned.items():
                manifest[name]["misaligned"] = lengths
            manifest["errors"] = list(self._errors)
            manifest["error_count"] = self._error_count
            manifest["cuts"], manifest["cut_wall_max_s"] = self._cuts, self._cut_wall_max
            manifest["complete"] = not misaligned and all(
                s["dropped_items"] == 0 and s["unsettled_items"] == 0 for s in self._stats.values())
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "manifest.json").write_text(json.dumps(manifest, indent=1))
        self._manifest = manifest
        return manifest


# --- reading a spool back (after the run: replays, analyses, the end-of-run summaries) ---------------------------------
def iter_chunks(root, name: str):
    """The chunks of ``name`` in the order written, one at a time: a list of items (jsonl) or a dict of arrays (npz).
    Only whole chunks: a ``.tmp`` left by a cut-off write is not read."""
    folder = Path(root) / name
    if not folder.is_dir():
        return
    for path in sorted(folder.iterdir()):
        if path.suffix == ".jsonl":
            yield [json.loads(line) for line in path.read_text().splitlines() if line]
        elif path.suffix == ".npz":
            with np.load(path, allow_pickle=False) as data:
                yield {key: data[key] for key in data.files}


def load_group(root, name: str, tail: dict | None = None) -> dict:
    """A group's chunks joined into one array per list, the in-memory ``tail`` (the lists as they are) after them."""
    parts: dict = {}
    for chunk in iter_chunks(root, name):
        for key, values in chunk.items():
            parts.setdefault(key, []).append(values)
    for key, values in (tail or {}).items():
        if len(values):
            parts.setdefault(key, []).append(np.asarray(values))
    return {key: np.concatenate(arrays) for key, arrays in parts.items()}


def _require_complete(root: Path, allow_incomplete: bool) -> dict:
    manifest = json.loads((root / "manifest.json").read_text()) if (root / "manifest.json").exists() else None
    if not allow_incomplete and not (manifest or {}).get("complete"):
        raise ValueError(f"{root}: the spooled record is incomplete (manifest: dropped, unsettled or misaligned, or "
                         "no manifest); pass allow_incomplete to read what was written")
    return manifest or {}


def _require_items(root: Path, manifest: dict, name: str, read: int, allow_incomplete: bool) -> None:
    """The items read against the items the manifest says were written (Codex D8 round 3 P3-1: a copy missing a
    chunk would otherwise read as whole)."""
    written = manifest.get(name, {}).get("items")
    if not allow_incomplete and written is not None and read != written:
        raise ValueError(f"{root}/{name}: {read} items read, {written} written -- chunks missing")


def load_list(root, name: str, *, allow_incomplete: bool = False) -> list:
    """A spooled list, whole (after the run)."""
    manifest = _require_complete(Path(root), allow_incomplete)
    items = [item for chunk in iter_chunks(root, name) for item in chunk]
    _require_items(Path(root), manifest, name, len(items), allow_incomplete)
    return items


def load_slam_log(run, *, allow_incomplete: bool = False) -> dict:
    """A run's SLAM log as arrays: its ``slam_log.npz`` (a batch run), or, for a live run, the spooled groups its
    ``meta.json`` names (persistent console plan D8) -- refused when the spool says it is incomplete."""
    run = Path(run)
    if (run / "slam_log.npz").exists():
        with np.load(run / "slam_log.npz", allow_pickle=False) as data:
            return {key: data[key] for key in data.files}
    names = json.loads((run / "meta.json").read_text()).get("spool")
    if not names:
        raise FileNotFoundError(f"{run}: neither slam_log.npz nor a spooled SLAM log in meta.json")
    root = run / names.get("root", "spool")
    manifest = _require_complete(root, allow_incomplete)
    joint, scan = load_group(root, names["joint"]), load_group(root, names["scan"])
    for name, group in ((names["joint"], joint), (names["scan"], scan)):
        _require_items(root, manifest, name, min((len(v) for v in group.values()), default=0), allow_incomplete)
    log = {**joint, **scan}
    log["silenced_scan_stamps_s"] = np.asarray(
        load_list(root, names["silenced"], allow_incomplete=allow_incomplete), dtype=float)
    return log


def read_live_log(run, *, allow_incomplete: bool = False) -> list[dict]:
    """A live run's ``live_log.jsonl`` entries, oldest first across its rolled files (``.N`` … ``.1``, then the
    current one). Refused when files rolled off (``live_log.index.json``: more rotations than kept), since what is
    left would read as the whole session; and, for a log made by the ordered writer (its first line says
    ``log_writer``, plan U29), when its end summary (``log_end``) is missing or counts lost lines -- unless
    ``allow_incomplete``. A log from before the writer has neither and is read as it is."""
    run = Path(run)
    index = run / "live_log.index.json"
    parts = [run / "live_log.jsonl"]
    if index.exists():
        info = json.loads(index.read_text())
        if info["rotations"] > info["keep"]:
            raise ValueError(f"{run}: the live log rolled {info['rotations']} times and keeps {info['keep']}: "
                             "its oldest part is gone")
        parts = [run / f"live_log.jsonl.{n}" for n in range(info["rotations"], 0, -1)] + parts
        missing = [p.name for p in parts if not p.exists()]
        if missing:
            raise ValueError(f"{run}: live log parts missing: {missing}")
    entries = [json.loads(line) for part in parts if part.exists() for line in part.read_text().splitlines() if line]
    if allow_incomplete or not any(e.get("log_writer") == "ordered" for e in entries[:1]):
        return entries
    end = next((e["log_end"] for e in reversed(entries) if "log_end" in e), None)
    if end is None:
        raise ValueError(f"{run}: the live log has no end summary (the session did not close its writer)")
    if end.get("dropped_lines") or end.get("line_errors"):
        raise ValueError(f"{run}: the live log lost lines ({end['dropped_lines']} dropped, {end['line_errors']} "
                         "failed)")
    return entries
