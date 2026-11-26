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
- its depth (7e-1, plan 4.2), when it has any: every side file named by a ``depth_file`` row is there with that
  sha256 (its stat pinned for the readers); every ``depth`` row's array is in its file under its key, ``uint16``, of
  the row's shape (the intrinsics' size), with the row's frame sha256; its units finite and positive and the
  profile's; its ``calib_id`` recomputed from the row's own intrinsics (and the profile's); and the depth source
  re-judging the rows in file order with the header's config gives back every verdict, interval and generation
  (``replay_depth_mismatch`` otherwise);
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
import hashlib
import json
import math
import os
import re
import threading
import zlib
from dataclasses import dataclass, field
from pathlib import Path

from forklift_core.live.observe_record import SCHEMA
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
    # 7e-1: the depth rows (only when the record has depth)
    "depth": {"t", "host_ns", "frame_number", "timestamp", "domain", "units", "intrinsics", "calib_id", "lo", "hi",
              "gen", "verdict", "npz", "key", "sha256", "shape", "dtype"},
    "depth_gen": {"t", "why", "gen", "host_ns"},
    "depth_file": {"t", "name", "frames", "sha256"},
    "depth_profile": {"t", "intrinsics", "calib_id", "depth_scale"},
}
DEPTH_KINDS = ("depth", "depth_gen", "depth_file", "depth_profile")


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
    lidar: list  # [(record ns, file offset, crc32 of the line)] in file order: scan and scan_event rows
    imu: list  # [(record ns, file offset, crc32 of the line)] in file order: imu rows and imu_gen losses
    synthetic: bool
    counts: collections.Counter = field(default_factory=collections.Counter)
    imu_config_recorded: bool = False
    stat: tuple = ()  # (device, inode, size, mtime_ns) of the bytes verified: the readers refuse another file
    # 7e-1: [(record ns, file offset, crc32)] in file order: depth rows and depth losses; the side files verified,
    # {name: (sha256, (device, inode, size, mtime_ns))} -- a reader refuses a file changed since (plan 4.2)
    depth: list = field(default_factory=list)
    depth_files: dict = field(default_factory=dict)


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _imu_cfg_from_header(imu_head):
    """The source config the record names; None when it names none (a record from before 7r); a config that is there
    but not a whole one is refused (Codex 7r-1 round 1 P2-8: an empty one is not "unrecorded")."""
    from forklift_core.sensors.imu_source import ImuSourceConfig

    if not isinstance(imu_head, dict) or "config" not in imu_head:
        return None
    raw = imu_head["config"]
    names = {f.name for f in ImuSourceConfig.__dataclass_fields__.values()}
    if not isinstance(raw, dict) or set(raw) != names:
        raise ReplayRefused("record_imu_config", "the header's imu.config is not a whole source config")
    return ImuSourceConfig(**raw)


class _ImuVerifier:
    """``imu_source.replay`` row by row, the produced rows compared with the record's as they come."""

    def __init__(self, cfg) -> None:
        from forklift_core.sensors.imu_source import ImuSource

        self.clock = {"ns": 0}
        self.src = ImuSource(cfg, operating=False, record=True, now_ns=lambda: self.clock["ns"])
        self.expected_gen: collections.deque = collections.deque()
        self.gap_ns = int(round(cfg.receive_gap_s * 1e9))
        self.last_rx: int | None = None
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
                key = ("why", "gen", "host_ns")  # the boundary's time too (round 1 P2-8)
                if want is None or any(p.get(k) != want.get(k) for k in key):
                    self._bad(("imu_gen", p.get("why"), p.get("gen"), None if want is None else want.get("why")))

    def row(self, r: dict) -> None:
        if r.get("t") == "imu_gen":
            self.expected_gen.append(r)
            why = str(r.get("why") or "")
            if why == "receive_gap" and r.get("host_ns") is not None:
                # the observe tool's cycle: samples (applying what is queued, a loss included), then generation
                self.clock["ns"] = max(self.clock["ns"], int(r["host_ns"]) + 1)
                self.src.samples(self.clock["ns"], 1 << 30)
                self.src.generation()
            elif why.startswith("lost:") and r.get("host_ns") is not None:
                # the device thread queues it; the observe cycle's samples() applies it alone (one generation), or --
                # when it came between samples() and a generation() that found a receive gap -- the two boundaries
                # merged into this one row (two generations): the recorded step says which (round 1-2 P2-8, P2-3)
                at = int(r["host_ns"])
                self.clock["ns"] = max(self.clock["ns"], at)
                self.src.lost(why[len("lost:"):], at_ns=at)
                step = int(r.get("gen") or 0) - self.src.gen
                if step >= 2 and self.last_rx is not None:
                    self.clock["ns"] = max(self.clock["ns"], self.last_rx + self.gap_ns + 1)
                    self.src.generation()
                else:
                    self.src.samples(at, 1 << 30)
            self._compare()
        elif r.get("t") == "imu":
            host = int(r["host_ns"])
            self.last_rx = host if self.last_rx is None else max(self.last_rx, host)
            self.clock["ns"] = host
            self.src.push(host, r.get("s", "a"), r.get("dev_us"), r.get("dom"), (r["x"], r["y"], r["z"]),
                          ts_ms=r.get("ts_ms"))
            self.src.samples(host, 1 << 30)
            self._compare(r)

    def finish(self, last_generation_ns) -> None:
        if last_generation_ns is not None:
            self.clock["ns"] = max(self.clock["ns"], int(last_generation_ns))
            self.src.samples(self.clock["ns"], 1 << 30)
            self.src.generation()
        self._compare()
        if self.expected_gen:
            self._bad(("imu_gen_unproduced", self.expected_gen[0].get("why")))


def _depth_cfg_from_header(head):
    from forklift_core.sensors.depth_cfg import DepthConfig

    c = dict(head["config"])
    c["applies_to"] = tuple(tuple(x) for x in (c.get("applies_to") or ()))
    return DepthConfig(**c)


_SIDE_NAME = re.compile(r"depth_\d{4,}\.npz")  # DepthRecorder's names: inside the record's folder, nothing else


class _DepthVerifier:
    """The record's depth rows checked against their side files and re-judged in file order (module text)."""

    def __init__(self, head) -> None:
        from forklift_core.sensors.depth_source import DepthSource

        self.cfg = _depth_cfg_from_header(head)
        self.folder_name = head.get("folder")
        self.src = DepthSource(self.cfg)
        self.seq: list = []  # ("frame"|"gen", row) recorded, in file order
        self.files: dict = {}  # name -> depth_file row
        self.by_file: dict = collections.defaultdict(list)
        self.duplicate_files: set = set()
        self.profile = None

    def row(self, r: dict) -> None:
        t = r.get("t")
        if t == "depth":
            self.seq.append(("frame", r))
            self.by_file[str(r["npz"])].append(r)
        elif t == "depth_gen":
            self.seq.append(("gen", r))
        elif t == "depth_file":
            name = str(r["name"])
            if name in self.files:
                self.duplicate_files.add(name)
            self.files[name] = r
        elif t == "depth_profile":
            self.profile = r

    def finish(self, record_path: Path, end_depth) -> tuple:
        from forklift_core.sensors.depth_cfg import calib_id, frame_sha256
        from forklift_core.sensors.depth_record import read_side_file, side_folder

        def bad(what):
            raise ReplayRefused("replay_depth_mismatch", what)

        # the end's summary and the file list, whether or not the record has frames (Codex 7e-1 round 2 P2-3)
        if not isinstance(end_depth, dict):
            bad("a depth header and no depth summary in the end row")
        ed = end_depth
        for key in ("overflow", "bad_frames", "callback_errors"):
            if int(ed.get(key) or 0) > 0:
                bad(f"raw depth frames lost before they were recorded ({key} {ed.get(key)})")
        if ed.get("record_failed"):
            bad(f"a side file not written whole: {ed.get('record_failed')}")
        if self.duplicate_files:
            bad(f"side files declared twice: {sorted(self.duplicate_files)}")
        bad_names = [n for n in self.files if not _SIDE_NAME.fullmatch(n)]
        if bad_names:
            bad(f"side file names outside the record's folder form: {bad_names}")
        if sorted(self.files) != sorted(str(n) for n in (ed.get("files") or ())):
            bad("the depth_file rows are not the end's file list")
        if int(ed.get("frames") or 0) != sum(1 for k, _r in self.seq if k == "frame"):
            bad("the depth rows are not the end's frame count")
        if not self.seq:
            if self.files:
                bad("side files declared and no depth row")  # nothing to check them against (round 3 P2-3)
            return {}, collections.Counter()
        if self.profile is None:
            bad("no depth_profile row")
        prof_id = calib_id(self.profile["intrinsics"])
        if prof_id != self.profile["calib_id"]:
            bad("the profile's calib_id is not its intrinsics'")
        units = self.profile.get("depth_scale")
        if not _finite(units) or units <= 0:
            bad(f"the profile's depth_scale {units!r} is not a finite positive number")
        folder = side_folder(record_path)
        if self.folder_name is not None and folder.name != self.folder_name:
            bad(f"side folder {folder.name} is not the header's {self.folder_name}")
        unknown = set(self.by_file) - set(self.files)
        if unknown:
            bad(f"depth rows in files with no depth_file row: {sorted(unknown)}")
        pins: dict = {}
        counts: collections.Counter = collections.Counter()
        for name, frow in self.files.items():
            fp = folder / name
            try:
                arrays, pin = read_side_file(fp, str(frow["sha256"]))  # one open: its fstat is the pin
            except (OSError, ValueError) as exc:
                bad(f"{name}: {exc}")
            rows = self.by_file.get(name, [])
            keys = [str(r["key"]) for r in rows]
            if len(rows) != int(frow["frames"]) or len(set(keys)) != len(keys) or set(keys) != set(arrays):
                bad(f"{name}: arrays {sorted(arrays)[:3]}... and rows' keys are not one to one "
                    f"({len(rows)} rows, {frow['frames']} frames recorded)")
            for r in rows:
                a = arrays.get(str(r["key"]))
                intr = r["intrinsics"]
                shape = [int(intr["height"]), int(intr["width"])]
                if a is None or a.dtype.name != "uint16" or r.get("dtype") != "uint16" or list(a.shape) != shape \
                        or list(r["shape"]) != shape:
                    bad(f"{name}/{r['key']}: not a {shape} uint16 frame")
                if frame_sha256(a) != r["sha256"]:
                    bad(f"{name}/{r['key']}: frame sha256 differs")
                u = r["units"]
                if not _finite(u) or u <= 0 or u != units:
                    bad(f"{name}/{r['key']}: units {u}")
                if calib_id(intr) != r["calib_id"] or r["calib_id"] != prof_id:
                    bad(f"{name}/{r['key']}: calib_id is not its intrinsics' or the profile's")
                counts["depth_frames_verified"] += 1
            pins[name] = (str(frow["sha256"]), pin)
        # re-judged in file order with the header's config (losses at their own time)
        got: list = []
        for kind, r in self.seq:
            if kind == "frame":
                self.src.push(int(r["host_ns"]), r["frame_number"], r["timestamp"], r["domain"], r.get("meta"),
                              r["units"], r["intrinsics"], None)
                got += self.src.take(int(r["host_ns"]))
            elif str(r["why"]).startswith("lost:"):
                self.src.lost(str(r["why"])[len("lost:"):], at_ns=int(r["host_ns"]))
                got += self.src.take(int(r["host_ns"]))
        last = (end_depth or {}).get("last_take_ns")
        if last is not None:
            got += self.src.finish(int(last))
        keys = ("host_ns", "frame_number", "lo", "hi", "gen", "verdict")
        want = [(k, r if k == "gen" else {x: r[x] for x in keys}) for k, r in self.seq]
        have = [(k, r if k == "gen" else {x: r[x] for x in keys}) for k, r in got]
        if have != want:
            first = next((i for i, (a, b) in enumerate(zip(have, want)) if a != b), min(len(have), len(want)))
            bad(f"re-judged depth differs at depth item {first}")
        counts["depth_ok"] = sum(1 for k, r in self.seq if k == "frame" and r["verdict"] == "ok")
        return pins, counts


def check_depth_pins(record: "ReplayRecord") -> None:
    """Every verified side file still the bytes verified (``ReplayRefused`` otherwise) -- a reader's check before
    it reads (plan 4.2)."""
    from forklift_core.sensors.depth_record import side_folder

    folder = side_folder(record.path)
    for name, (_sha, pin) in record.depth_files.items():
        try:
            st = os.stat(folder / name)
        except OSError as exc:
            raise ReplayRefused("replay_depth_mismatch", f"{name}: {exc}") from exc
        if (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns) != tuple(pin):
            raise ReplayRefused("replay_depth_mismatch", f"{name} changed since it was verified")


def load_replay_record(path, *, lidar_cfg, imu_cfg=None, imu_required: bool, allow_synthetic: bool = False,
                       min_s: float = REPLAY_MIN_S) -> ReplayRecord:
    """The record checked, verified and indexed, or ``ReplayRefused`` (see the module text). ``lidar_cfg``: the
    selected ``lidar_cfg.LidarConfig``; ``imu_cfg``: the selected IMU source config (judges a record that names
    none)."""
    from forklift_core.localization.slam_real_scan import from_feed
    from forklift_core.sensors import scan_feed as sf
    from forklift_core.sensors.lidar_cfg import to_layer_scan

    p = Path(path)
    try:
        want_sha = Path(str(p) + ".ok").read_text(encoding="ascii").strip()
        st = os.stat(p)
    except OSError as exc:
        raise ReplayRefused("record_incomplete", f"{p}: no .ok marker") from exc
    digest = hashlib.sha256()  # the bytes parsed are the bytes hashed (Codex 7r-1 round 1 P3: one read)
    last_t = None
    malformed_at = None
    deferred: ReplayRefused | None = None
    header = slam_header = end = None
    anchor = None
    lidar, imu = [], []
    counts: collections.Counter = collections.Counter()
    feed = sf.ScanFeed()
    synthetic_frame = False
    operating: list = []  # (record ns, (feed_gen, producer_gen)) of scans the layer and the operating SLAM take
    imu_rows = 0
    imu_ver = None
    depth: list = []
    depth_ver = None
    last_ns = None
    with open(p, "rb") as f:
        offset = 0
        for raw in f:
            digest.update(raw)
            line_at = offset
            offset += len(raw)
            if deferred is not None or malformed_at is not None:
                continue  # hashed to the end: a refusal found in the rows is reported after the hash (round 2 P3)
            try:
                r = json.loads(raw)
            except ValueError:
                # hashed to the end first: a changed file is "incomplete", a sealed one that does not parse "malformed"
                malformed_at = line_at if malformed_at is None else malformed_at
                last_t = None
                continue
            t = r.get("t") if isinstance(r, dict) else None
            last_t = t
            try:
                if header is None:
                    if t != "header" or r.get("schema") != SCHEMA:
                        raise ReplayRefused("record_schema", f"first row is not an {SCHEMA} header")
                    header = r
                    cfg = _imu_cfg_from_header(header.get("imu"))
                    imu_ver = _ImuVerifier(cfg if cfg is not None else imu_cfg) if (cfg or imu_cfg) else None
                    counts["imu_config_recorded"] = int(cfg is not None)
                    if header.get("depth"):
                        depth_ver = _DepthVerifier(header["depth"])
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
                    lidar.append((ns, line_at, zlib.crc32(raw)))
                    kind = r.get("kind")
                    if kind in ("receive_gap", "socket_error"):
                        feed.new_generation(kind)  # where the observe tool opened it (verification, plan 2)
                elif t == "scan":
                    ns = int(r["host_recv_ns"])
                    lidar.append((ns, line_at, zlib.crc32(raw)))
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
                        imu.append((ns, line_at, zlib.crc32(raw)))
                        last_ns = ns if last_ns is None else max(last_ns, ns)
                    imu_rows += t == "imu"
                    if imu_ver is not None:
                        imu_ver.row(r)
                elif t in DEPTH_KINDS:
                    if depth_ver is None:
                        raise ReplayRefused("replay_depth_mismatch", "depth rows with no depth header")
                    if t == "depth" or (t == "depth_gen" and str(r.get("why") or "").startswith("lost:")):
                        # indexed for 7e-2's reader; not the record's length: the replay does not consume depth yet
                        # (Codex 7e-1 round 1 P2-7)
                        depth.append((int(r["host_ns"]), line_at, zlib.crc32(raw)))
                    depth_ver.row(r)
            except ReplayRefused as exc:
                deferred = exc
            except (KeyError, TypeError, ValueError, AttributeError):
                # a row without a key it must have: malformed, reported after the hash (round 3 P3)
                malformed_at = line_at if malformed_at is None else malformed_at
    if digest.hexdigest() != want_sha:
        raise ReplayRefused("record_incomplete", "the .ok marker names other bytes")
    if malformed_at is not None:
        raise ReplayRefused("record_malformed", f"offset {malformed_at}")
    if deferred is not None:
        raise deferred
    st2 = os.stat(p)
    pin = (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns)
    if (st2.st_dev, st2.st_ino, st2.st_size, st2.st_mtime_ns) != pin:
        raise ReplayRefused("record_incomplete", "the record changed while it was read")
    if header is None or end is None or last_t != "end" or end.get("complete") is not True:
        raise ReplayRefused("record_incomplete", "the last row is not a complete end")
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
    # a frame marked synthetic counts whether or not the feed accepted it (round 1 P2-7)
    synthetic = is_synthetic(header) or synthetic_frame or feed.synthetic_frames > 0
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
    depth_pins: dict = {}
    if depth_ver is not None:
        depth_pins, dcounts = depth_ver.finish(p, end.get("depth"))
        counts.update(dcounts)
    lo, hi = anchor + int(BRIDGE_STARTUP_MAX_S * 1e9), anchor + int(START_WINDOW_END_S * 1e9)
    per_gen = collections.Counter(g for ns, g in operating if lo <= ns <= hi)
    if not per_gen or max(per_gen.values()) < START_SCANS:
        raise ReplayRefused("replay_no_operating_scans",
                            f"fewer than {START_SCANS} scans of one generation for the layer and the operating SLAM "
                            f"in [{BRIDGE_STARTUP_MAX_S}, {START_WINDOW_END_S}] s past the anchor")
    counts["scans_operating"] = len(operating)
    return ReplayRecord(path=p, sha256=want_sha, header=header, slam_header=slam_header, end=end,
                        anchor_ns=anchor, last_ns=last_ns, lidar=lidar, imu=imu, synthetic=synthetic, counts=counts,
                        imu_config_recorded=bool(counts["imu_config_recorded"]), stat=pin, depth=depth,
                        depth_files=depth_pins)


class StreamReader:
    """One stream's rows read ahead on a thread of its own (plan 4: no disk read in a tick): ``entries`` in file
    order, parsed into a bounded queue. ``take(upto_ns)`` gives the queued rows in order while the next one's record
    time is at most ``upto_ns`` (a row never goes before the ones ahead of it in the file); ``t_read`` is the latest
    record time queued so far (infinite once the file's last row is queued); ``done`` once every row is taken."""

    def __init__(self, path, entries, *, capacity: int = 512, start: bool = True, pin: tuple = ()) -> None:
        self.path, self.entries, self.capacity = Path(path), entries, int(capacity)
        self.pin = tuple(pin)  # the verified file's identity: another file under the path is an error
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
                if self.pin:
                    st = os.fstat(f.fileno())
                    if (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns) != self.pin:
                        raise RuntimeError("the record is not the file that was verified")
                while True:
                    with self._cv:
                        while not self._stop and len(self._q) >= self.capacity:
                            self._cv.wait(0.1)
                        if self._stop or self._next >= len(self.entries):
                            if self._next >= len(self.entries):
                                self._t_read = math.inf
                            return
                        ns, off, crc = self.entries[self._next]
                    f.seek(off)
                    line = f.readline()
                    if zlib.crc32(line) != crc:
                        # the bytes verified at load are the bytes replayed (round 2 P3: a change in place is caught)
                        raise RuntimeError(f"the record changed at offset {off}")
                    row = json.loads(line)
                    with self._cv:
                        self._q.append((ns, row))
                        self._next += 1
                        self._t_read = max(self._t_read, ns)
                        if self._next >= len(self.entries):
                            self._t_read = math.inf
                        self._cv.notify_all()
        except BaseException as exc:  # noqa: BLE001 -- surfaced to the source, never lost on the thread
            with self._cv:
                self.error = exc
                self._cv.notify_all()

    def check(self) -> None:
        """A read failure surfaces here, before anything else is asked (round 1 P2-3: not mistaken for a slow read)."""
        if self.error is not None:
            raise RuntimeError(f"replay read: {self.error}") from self.error

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
        self.check()
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
            return self.error is None

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


__all__ = ["BRIDGE_STARTUP_MAX_S", "DEPTH_KINDS", "READS", "check_depth_pins", "REPLAY_MIN_S", "START_SCANS", "START_WINDOW_END_S", "ReplayRecord",
           "ReplayRefused", "StreamReader", "load_replay_record"]
