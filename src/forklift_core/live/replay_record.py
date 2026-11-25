"""The replay run's input (PR 7 7r, docs/plans/2026-11-26-7r-replay-run.md 1, 2, 4): an observe record
(``observe_record/1``) checked, verified and indexed before the HIL starts.

``load_replay_record`` reads the record once, front to back, and refuses it (``ReplayRefused``) unless:

- it is complete (``.ok`` names its sha256 and its last row is a complete ``end``) and ``end.result`` is ``ok``;
- its schema is ``observe_record/1``, it has a ``slam_header``, a ``state`` with a finite ``delay_bound_s`` (the
  anchor), at least one accepted scan, and IMU rows when the IMU is required;
- every accepted scan with acquisition times has them in order on one clock: first <= last <= adapter receipt <= the
  observe tool's receipt; a frame without them stays diagnostic and is not checked;
- it is not synthetic (``synthetic.is_synthetic`` of the header, or a frame marked ``synthetic``) unless allowed;
- it runs ``REPLAY_MIN_S`` past its anchor;
- the **verification pass** reproduces the record's own verdicts: every scan frame through a new ``ScanFeed`` in file
  order with the feed's generations opened where the record opened them (``receive_gap``, ``socket_error``), and every
  IMU row through a new recording ``ImuSource`` the way ``imu_source.replay`` does, losses put back at their own time
  -- a difference is ``replay_mismatch``. The IMU is judged with the source config the record names
  (``header.imu.config``); a record without one (before 7r) is judged with the given config and its differences are
  counted as ``imu_config_unrecorded``, not refused;
- the selected LiDAR config can run the SLAM start check on it: in ``[anchor + BRIDGE_STARTUP_MAX_S, anchor +
  START_WINDOW_END_S]`` at least ``START_SCANS`` scans of one producer and feed generation go to the layer and to the
  operating SLAM (``replay_no_operating_scans``). A floor, not a proof: the run's own start check decides.

The two streams the replay reads are indexed in **file order** (a stream's rows are never re-sorted by time: the
observe tool stamps a ``socket_error`` with its call's start, which may precede the scan before it -- Codex plan round 4
P2-1): LiDAR (``scan`` and ``scan_event`` rows) and IMU (``imu`` rows and ``imu_gen`` losses). Each entry is
``(record ns, file offset)``; the time only says when the row may go (``replay_sources``)."""

from __future__ import annotations

import base64
import collections
import json
import math
import threading
from dataclasses import dataclass, field
from pathlib import Path

from forklift_core.live.observe_record import SCHEMA, file_sha256, record_complete
from forklift_core.live.synthetic import is_synthetic

# provisional (lead 2026-11-26, re-checked on the first bench 1' record): the real bridge starts in 8-12.5 s (design
# 9), the start check has 20 s (real_slam_source.SEED_WAIT_S); a record must outlast both with room
REPLAY_MIN_S = 40.0
BRIDGE_STARTUP_MAX_S = 12.5
START_WINDOW_END_S = 19.0
START_SCANS = 7  # real_slam_source.START_SCANS


# the keys the replay reads, per row kind -- the observe tool's rows and the tests' synthetic writer must both carry
# them (a contract test reads both)
READS = {
    "header": {"t", "schema", "synthetic", "device", "device_id", "imu"},
    "state": {"t", "host_ns", "delay_bound_s"},
    "scan": {"t", "host_recv_ns", "frame", "verdict"},
    "scan_event": {"t", "kind", "host_ns"},
    "imu": {"t", "host_ns", "dev_us", "ts_ms", "dom", "x", "y", "z", "verdict", "gen", "lo", "hi"},
    "imu_gen": {"t", "why", "gen", "host_ns"},
    "slam_header": {"t", "bundle", "map_id", "start_pose", "rear_offset_m", "base_laser"},
    "end": {"t", "result", "complete"},
}


class ReplayRefused(ValueError):
    """The record cannot be replayed; ``reason`` names why (the first word of the message)."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason


@dataclass
class ReplayRecord:
    path: Path
    sha256: str
    header: dict
    slam_header: dict
    end: dict
    anchor_ns: int  # the record's first state with a finite delay bound (host_ns)
    last_ns: int  # the latest row time of either stream
    lidar: list  # [(record ns, file offset)] in file order: scan and scan_event rows
    imu: list  # [(record ns, file offset)] in file order: imu rows and imu_gen losses
    synthetic: bool
    counts: collections.Counter = field(default_factory=collections.Counter)
    imu_config_recorded: bool = False


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _imu_cfg_from_header(imu_head):
    from forklift_core.sensors.imu_source import ImuSourceConfig

    raw = (imu_head or {}).get("config")
    if not isinstance(raw, dict):
        return None
    try:
        return ImuSourceConfig(**raw)
    except TypeError:
        return None


class _ImuVerifier:
    """``imu_source.replay`` row by row, the produced rows compared with the record's as they come."""

    def __init__(self, cfg) -> None:
        from forklift_core.sensors.imu_source import ImuSource

        self.clock = {"ns": 0}
        self.src = ImuSource(cfg, operating=False, record=True, now_ns=lambda: self.clock["ns"])
        self.expected_gen: collections.deque = collections.deque()
        self.mismatches = 0
        self.first_mismatch = None

    def _bad(self, what) -> None:
        self.mismatches += 1
        if self.first_mismatch is None:
            self.first_mismatch = what

    def _compare(self, imu_row=None) -> None:
        for p in self.src.drain_rows():
            if p["t"] == "imu":
                if imu_row is None or any(p.get(k) != imu_row.get(k) for k in ("host_ns", "verdict", "gen", "lo", "hi")):
                    self._bad(("imu", p.get("host_ns"), p.get("verdict"), None if imu_row is None else imu_row.get("verdict")))
                imu_row = None
            else:
                want = self.expected_gen.popleft() if self.expected_gen else None
                if want is None or (p.get("why"), p.get("gen")) != (want.get("why"), want.get("gen")):
                    self._bad(("imu_gen", p.get("why"), p.get("gen"), None if want is None else want.get("why")))

    def row(self, r: dict) -> None:
        if r.get("t") == "imu_gen":
            self.expected_gen.append(r)
            why = str(r.get("why") or "")
            if why == "receive_gap" and r.get("host_ns") is not None:
                self.clock["ns"] = int(r["host_ns"]) + 1
                self.src.generation()
            elif why.startswith("lost:") and r.get("host_ns") is not None:
                at = int(r["host_ns"])
                self.clock["ns"] = max(self.clock["ns"], at)
                self.src.lost(why[len("lost:"):], at_ns=at)
                self.src.generation()
            self._compare()
        elif r.get("t") == "imu":
            host = int(r["host_ns"])
            self.clock["ns"] = host
            self.src.push(host, r.get("s", "a"), r.get("dev_us"), r.get("dom"), (r["x"], r["y"], r["z"]),
                          ts_ms=r.get("ts_ms"))
            self.src.samples(host, 1 << 30)
            self._compare(r)

    def finish(self, last_generation_ns) -> None:
        if last_generation_ns is not None:
            self.clock["ns"] = int(last_generation_ns)
            self.src.generation()
        self._compare()
        if self.expected_gen:
            self._bad(("imu_gen_unproduced", self.expected_gen[0].get("why")))


def load_replay_record(path, *, lidar_cfg, imu_cfg=None, imu_required: bool, allow_synthetic: bool = False,
                       min_s: float = REPLAY_MIN_S) -> ReplayRecord:
    """The record checked, verified and indexed, or ``ReplayRefused`` (see the module text). ``lidar_cfg``: the
    selected ``lidar_cfg.LidarConfig``; ``imu_cfg``: the selected IMU source config (judges a record that names
    none)."""
    from forklift_core.localization.slam_real_scan import from_feed
    from forklift_core.sensors import scan_feed as sf
    from forklift_core.sensors.lidar_cfg import to_layer_scan

    p = Path(path)
    if not record_complete(p):
        raise ReplayRefused("record_incomplete", str(p))
    header = slam_header = end = None
    anchor = None
    lidar, imu = [], []
    counts: collections.Counter = collections.Counter()
    feed = sf.ScanFeed()
    synthetic_frame = False
    operating: list = []  # (record ns, (feed_gen, producer_gen)) of scans the layer and the operating SLAM take
    imu_rows = 0
    imu_ver = None
    last_ns = None
    with open(p, "rb") as f:
        offset = 0
        for raw in f:
            line_at = offset
            offset += len(raw)
            try:
                r = json.loads(raw)
            except ValueError as exc:
                raise ReplayRefused("record_malformed", f"offset {line_at}") from exc
            t = r.get("t") if isinstance(r, dict) else None
            if header is None:
                if t != "header" or r.get("schema") != SCHEMA:
                    raise ReplayRefused("record_schema", f"first row is not an {SCHEMA} header")
                header = r
                cfg = _imu_cfg_from_header(header.get("imu"))
                imu_ver = _ImuVerifier(cfg if cfg is not None else imu_cfg) if (cfg or imu_cfg) else None
                counts["imu_config_recorded"] = int(cfg is not None)
                continue
            if t == "slam_header":
                slam_header = r
            elif t == "end":
                end = r
            elif t == "state":
                if anchor is None and _finite(r.get("delay_bound_s")):
                    anchor = int(r["host_ns"])
            elif t == "scan_event":
                ns = int(r["host_ns"])
                lidar.append((ns, line_at))
                kind = r.get("kind")
                if kind in ("receive_gap", "socket_error"):
                    feed.new_generation(kind)  # where the observe tool opened it (verification, plan 2)
            elif t == "scan":
                ns = int(r["host_recv_ns"])
                lidar.append((ns, line_at))
                last_ns = ns if last_ns is None else max(last_ns, ns)
                data = base64.b64decode(r["frame"])
                scan, why = feed.judge(data, ns)
                if why != r.get("verdict"):
                    raise ReplayRefused("replay_mismatch", f"scan at offset {line_at}: {why} != {r.get('verdict')}")
                if scan is None:
                    continue
                counts["scans_accepted"] += 1
                if scan.meta.get("synthetic"):
                    synthetic_frame = True
                fb, lb = scan.meta.get("first_beam_mono_ns"), scan.meta.get("last_beam_mono_ns")
                if fb is not None and lb is not None and not (fb <= lb <= scan.recv_mono_ns <= ns):
                    raise ReplayRefused("record_clock_order", f"scan at offset {line_at}")
                layer, _lw = to_layer_scan(scan, lidar_cfg, 0)
                s_in, _sw = from_feed(scan, lidar_cfg)
                if layer is not None and s_in is not None and s_in.measured:
                    operating.append((ns, (scan.feed_gen, scan.producer_gen)))
            elif t in ("imu", "imu_gen"):
                if t == "imu" or str(r.get("why") or "").startswith("lost:"):
                    ns = int(r["host_ns"])
                    imu.append((ns, line_at))
                    last_ns = ns if last_ns is None else max(last_ns, ns)
                imu_rows += t == "imu"
                if imu_ver is not None:
                    imu_ver.row(r)
    if header is None or end is None:
        raise ReplayRefused("record_incomplete", "no header or end")
    if end.get("result") != "ok":
        raise ReplayRefused("record_result", str(end.get("result")))
    if slam_header is None:
        raise ReplayRefused("record_no_slam_header")
    if anchor is None:
        raise ReplayRefused("record_no_anchor", "no state with a finite delay bound")
    if counts["scans_accepted"] == 0:
        raise ReplayRefused("record_no_scans")
    if imu_required and imu_rows == 0:
        raise ReplayRefused("record_no_imu")
    synthetic = is_synthetic(header) or synthetic_frame
    if synthetic and not allow_synthetic:
        raise ReplayRefused("record_synthetic", "give --allow-synthetic (the output is then not_for_judgement)")
    if last_ns is None or (last_ns - anchor) / 1e9 < float(min_s):
        raise ReplayRefused("record_too_short", f"{min_s} s past the anchor needed")
    if imu_ver is not None:
        imu_ver.finish((end.get("imu") or {}).get("last_generation_ns"))
        if imu_ver.mismatches:
            if counts["imu_config_recorded"]:
                raise ReplayRefused("replay_mismatch", f"imu {imu_ver.first_mismatch}")
            counts["imu_config_unrecorded"] = imu_ver.mismatches
    lo, hi = anchor + int(BRIDGE_STARTUP_MAX_S * 1e9), anchor + int(START_WINDOW_END_S * 1e9)
    per_gen = collections.Counter(g for ns, g in operating if lo <= ns <= hi)
    if not per_gen or max(per_gen.values()) < START_SCANS:
        raise ReplayRefused("replay_no_operating_scans",
                            f"fewer than {START_SCANS} scans of one generation for the layer and the operating SLAM "
                            f"in [{BRIDGE_STARTUP_MAX_S}, {START_WINDOW_END_S}] s past the anchor")
    counts["scans_operating"] = len(operating)
    return ReplayRecord(path=p, sha256=file_sha256(p), header=header, slam_header=slam_header, end=end,
                        anchor_ns=anchor, last_ns=last_ns, lidar=lidar, imu=imu, synthetic=synthetic, counts=counts,
                        imu_config_recorded=bool(counts["imu_config_recorded"]))


class StreamReader:
    """One stream's rows read ahead on a thread of its own (plan 4: no disk read in a tick): ``entries`` in file
    order, parsed into a bounded queue. ``take(upto_ns)`` gives the queued rows in order while the next one's record
    time is at most ``upto_ns`` (a row never goes before the ones ahead of it in the file); ``t_read`` is the latest
    record time queued so far (infinite once the file's last row is queued); ``done`` once every row is taken."""

    def __init__(self, path, entries, *, capacity: int = 512, start: bool = True) -> None:
        self.path, self.entries, self.capacity = Path(path), list(entries), int(capacity)
        self._q: collections.deque = collections.deque()
        self._cv = threading.Condition()
        self._next = 0  # the next entry to read
        self._taken = 0
        self._t_read = -math.inf
        self._stop = False
        self.error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        if start:
            self._thread.start()

    def _run(self) -> None:
        try:
            with open(self.path, "rb") as f:
                while True:
                    with self._cv:
                        while not self._stop and len(self._q) >= self.capacity:
                            self._cv.wait(0.1)
                        if self._stop or self._next >= len(self.entries):
                            if self._next >= len(self.entries):
                                self._t_read = math.inf
                            return
                        ns, off = self.entries[self._next]
                    f.seek(off)
                    row = json.loads(f.readline())
                    with self._cv:
                        self._q.append((ns, row))
                        self._next += 1
                        self._t_read = max(self._t_read, ns)
                        if self._next >= len(self.entries):
                            self._t_read = math.inf
                        self._cv.notify_all()
        except BaseException as exc:  # noqa: BLE001 -- surfaced to the source, never lost on the thread
            self.error = exc

    @property
    def t_read(self) -> float:
        with self._cv:
            return self._t_read

    @property
    def done(self) -> bool:
        with self._cv:
            return self._taken >= len(self.entries)

    def take(self, upto_ns, limit: int | None = None) -> list:
        """``[(record ns, row)]`` in file order whose turn has come (each row's time at most ``upto_ns``)."""
        if self.error is not None:
            raise RuntimeError(f"replay read: {self.error}") from self.error
        out = []
        with self._cv:
            while self._q and self._q[0][0] <= upto_ns and (limit is None or len(out) < limit):
                out.append(self._q.popleft())
            self._taken += len(out)
            if out:
                self._cv.notify_all()
        return out

    def wait_read(self, timeout: float = 10.0) -> bool:
        """Until the reader has queued every row it may (the file's end, or a full queue); True when it has."""
        import time

        end = time.monotonic() + timeout
        with self._cv:
            while self._t_read != math.inf and len(self._q) < self.capacity and self.error is None:
                left = end - time.monotonic()
                if left <= 0 or not self._thread.is_alive():
                    return False
                self._cv.wait(min(left, 0.05))
            return True

    def peek_ns(self):
        """The record time of the next queued row, or None (none queued yet, or the stream is done)."""
        with self._cv:
            return self._q[0][0] if self._q else None

    def close(self) -> None:
        with self._cv:
            self._stop = True
            self._cv.notify_all()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)


__all__ = ["BRIDGE_STARTUP_MAX_S", "READS", "REPLAY_MIN_S", "START_SCANS", "START_WINDOW_END_S", "ReplayRecord",
           "ReplayRefused", "StreamReader", "load_replay_record"]
