#!/usr/bin/env python3
"""The observe run (docs/design/2026-11-25-real-sources-pr7.md 1.1, 7a-1): the real board and the real sources,
recorded, with nothing that can move the truck.

- The board is opened with the executor lock (real safety wiring design 8.1) and every opening's link is an
  ``ObserveTtyLink``: only the poison and canonically encoded DISARMs can be written, whatever is queued. There is no
  mission core, arbiter or permit here at all.
- DISARMs: a session pair at the start and after every reconnect (the STATE delay bound is built from a pair of this
  session's frames), one each second (the OFF path's refresh), and one at the end, confirmed by a stopped STATE on
  its connection naming its seq.
- Recorded (JSONL, one row per line): the header, every STATE the host accepted with its connection, every frame
  written or refused, the link's events, and -- with ``--scan-socket`` -- every scan frame received (raw, with the
  feed's verdict, its generations and whether it could go to the obstacle layer; without the SDK first-sample time
  (7a-2) none can, and every scan stays diagnostic), and -- with ``--imu`` (7b) -- every D435i accel sample (raw, with
  ``sensors/imu_source.ImuSource``'s verdict, generation and interval under the provisional delay bound until the
  bound is measured) and every IMU generation change, and -- with ``--slam`` (7c-1c) -- the real SLAM source on a
  real-mode bridge (``forklift_core.live.observe_slam``: the scans sent, the SLAM path's drops, the replies with their
  match quality, the bridge's states; diagnostic until the first-beam bound is measured, 7a-3). The last row is
  ``end``: complete only if every row was written.

usage: real_observe.py --device /dev/forklift-mcu --out run.jsonl --seconds 60 [--scan-socket /run/forklift-scan/scan.sock]
       [--imu] [--slam /run/forklift-slam/bridge.sock --slam-bundle <survey bundle folder>]
(the scan socket is the one ``tools/ros2_dev.sh scan-a2m12`` sends to; the teleop bridge binds the same path, and the
executor lock keeps the two from running together)
exit: 0 complete and stopped (and, with --scan-socket, at least one scan accepted; with --imu, at least one accel
sample), 1 otherwise, 2 refused (nothing written to the device)."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import random
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))

from real_mcu_link import _BAUDS, ObserveTtyLink, ReconnectingLink, open_raw  # noqa: E402

from forklift_core.live.executor_lock import DeviceBusy  # noqa: E402
from forklift_core.live.synthetic import (  # noqa: E402
    OUTPUT_MARKS,
    SyntheticRefused,
    bench_opener,
    device_identity,
    writer_marker,
)
from forklift_core.mcu_link.build_policy import Client  # noqa: E402
from forklift_core.live.real_teleop import DelayBound  # noqa: E402
from forklift_core.mcu_link.messages import Mode  # noqa: E402
from forklift_core.plant.bringup import Sample  # noqa: E402
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost  # noqa: E402
from forklift_core.plant.record_rows import state_row  # noqa: E402
from forklift_core.safety.real_watch import load_imu_config  # noqa: E402
from forklift_core.sensors.imu_source import IMU_YAML, ImuSource, fits_watch, load_imu_source_config  # noqa: E402
from forklift_core.sensors.lidar_cfg import LIDAR_YAML, load_lidar_config, to_layer_scan  # noqa: E402
from forklift_core.sensors.scan_feed import RECV_BYTES, ScanFeed, bind_socket  # noqa: E402

SCHEMA = "observe_record/1"
CYCLE_S = 0.02  # 50 Hz, as the executor's loop
DISARM_EVERY_S = 1.0  # the OFF path's refresh (real_autonomy.ANCHOR_DISARM_S)
END_CONFIRM_S = 1.0
SCANS_READ_PER_CYCLE = 8  # the read cost per cycle (design 3.3): 10 Hz scans, 50 Hz cycles -- ample, never drained
MAX_SECONDS = 3600.0
SESSION_STAGE_S = 0.5  # each stage of the session pair is answered within this, or the pair starts again
SCAN_GAP_NS = 350_000_000  # a scan receive gap past the layer's sensor timeout (obstacle_layer_real 0.35 s)
SAFETY_YAML = REPO / "config" / "real" / "safety_watch.yaml"  # the IMU watch's gap limit, for the profile's fit
IMU_READ_PER_CYCLE = 64  # the loop's IMU_BATCH: 200 Hz accel, 50 Hz cycles -- four a cycle
HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=16)


def _sync(f) -> bool:
    try:
        f.flush()
        os.fsync(f.fileno())
        return True
    except (OSError, ValueError):
        return False


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):  # streamed: an hour's record is gigabytes (round 3 P3)
            h.update(chunk)
    return h.hexdigest()


def write_ok_marker(path: Path) -> None:
    """``<record>.ok``: the record's sha256, made only after the record's last row was synced (the completeness
    proof a replay checks). Written to a fresh file of its own (``mkstemp``, never an existing path or a link's
    target), synced, then renamed over the marker (Codex 7a-1 round 3 P2-3)."""
    path = Path(path)
    digest = _sha256(path)
    fd, tmp = tempfile.mkstemp(prefix=path.name + ".ok.", dir=path.parent)
    data = (digest + "\n").encode("ascii")
    try:
        try:
            off = 0
            while off < len(data):  # every byte out before it is published (Codex 7a-1 round 4 P2-2)
                n = os.write(fd, data[off:])
                if n <= 0:
                    raise OSError("short write of the .ok marker")
                off += n
            os.fsync(fd)
        finally:
            os.close(fd)
        os.replace(tmp, Path(str(path) + ".ok"))
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def _last_line(path: Path, block: int = 1 << 16) -> bytes:
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        end = f.tell()
        pos, buf = end, b""
        while pos > 0:
            step = min(block, pos)
            pos -= step
            f.seek(pos)
            buf = f.read(step) + buf
            body = buf.rstrip(b"\n")
            if b"\n" in body or pos == 0:
                return body.rsplit(b"\n", 1)[-1]
    return b""


def record_complete(path: Path) -> bool:
    """A record a replay may take: its .ok names its current sha256 and its last row is a complete end."""
    p = Path(path)
    try:
        want = Path(str(p) + ".ok").read_text(encoding="ascii").strip()
        if _sha256(p) != want:
            return False
        last = _last_line(p)
    except OSError:
        return False
    try:
        end = json.loads(last)
    except ValueError:
        return False
    return end.get("t") == "end" and end.get("complete") is True


class Record:
    """Rows written straight to the file; a failed write is counted and makes the record incomplete."""

    def __init__(self, f) -> None:
        self.f, self.errors, self.rows = f, 0, 0

    def put(self, row: dict) -> None:
        try:
            self.f.write(json.dumps(row, default=str) + "\n")
            self.rows += 1
        except (OSError, ValueError):
            self.errors += 1


class Observer:
    """One observe session on a link: the session DISARM pair (the teleop/auto session rule: the second a random
    1024-64512 seq step after the first is acknowledged, so the STATE delay bound can be built -- ``DelayBound``), a
    DISARM each second, every STATE, write, refusal and event recorded, scans if a socket is given."""

    def __init__(self, link, record: Record, *, openings=None, scan_sock=None, lidar=None, imu=None, slam=None,
                 clock=time.monotonic_ns, rng=None) -> None:
        self.link, self.rec, self.clock = link, record, clock
        self.openings = openings
        self.start_ns = clock()
        self.rng = rng if rng is not None else random.Random()
        # a random first seq: a STATE an earlier session left naming seq 0 never answers this one (Codex 7a-1 P2-1)
        self.host = McuLinkHost(HOST_CFG, client=Client.BRINGUP, log=self._log, on_state=self._on_state,
                                initial_seq=self.rng.randint(1, 0xFFFF))
        self.delay = DelayBound()
        self.scan_sock, self.lidar = scan_sock, lidar
        self.feed = ScanFeed() if scan_sock is not None else None
        self.imu = imu  # an ImuSource with record=True (7b), fed by the device's callback
        self.slam = slam  # an ObserveSlam (7c-1c): fed every STATE and every accepted scan, polled each cycle
        self.counts = {"states": 0, "written": 0, "refused": 0, "scans": 0, "scans_layer_ok": 0, "read_capped": 0,
                       "scan_gaps": 0, "scan_socket_errors": 0, "sessions": 0, "imu_read_capped": 0}
        self._writes: dict[tuple[int, int], int] = {}  # (conn, seq) -> first-byte ns (DISARMs)
        self._done_ns: dict[tuple[int, int], int] = {}  # (conn, seq) -> last-byte ns: an acknowledgement is after it
        if openings is not None:
            openings.on_done = self._done
        self._acks: dict[tuple[int, int, int], int] = {}  # (conn, boot, seq) -> receipt of the STATE stepping to it
        self._prev: tuple[int, int, int] | None = None  # the previous accepted STATE: (conn, boot, last_rx_seq)
        self._session: dict | None = None  # the pair in progress: {"stage", "seq", "first"}
        self.session_ready = False
        self._next_refresh_ns: int | None = None
        self._last_scan_ns: int | None = None
        self._scan_gap_open = False

    # --- host taps -------------------------------------------------------------------------------------------
    def _conn(self) -> int:
        return 0 if self.openings is None else self.openings.count

    def _on_state(self, state, received_ns: int) -> None:
        self.counts["states"] += 1
        conn, boot, seq = self._conn(), int(state.boot_id), int(state.last_rx_seq) & 0xFFFF
        if self._prev is not None and self._prev[:2] == (conn, boot) and self._prev[2] != seq:
            self._acks[(conn, boot, seq)] = int(received_ns)  # a step to ``seq`` on this connection and boot
        if self._prev is not None and self._prev[:2] != (conn, boot):
            self._restart_session("boot_changed" if self._prev[0] == conn else "new_connection")
        self._prev = (conn, boot, seq)
        sample = Sample(int(received_ns), state)
        self.delay.add(sample)
        d = self.delay.delay_s(sample)
        self.rec.put({**state_row(sample), "conn": conn, "delay_bound_s": d})
        if self.slam is not None:
            self.slam.on_state(state, int(received_ns), d, conn)

    def _log(self, what, now_ns, data, info) -> None:
        if what in ("written", "refused") and info:
            self.counts[what] += 1
            self.rec.put({"t": what, "kind": info.get("kind"), "seq": info.get("seq"), "host_ns": int(now_ns),
                          "conn": self._conn(), **({"why": info["why"]} if "why" in info else {})})
            if what == "written" and info.get("kind") == "DISARM":
                seq = int(info["seq"]) & 0xFFFF
                self._writes[(self._conn(), seq)] = int(now_ns)
                self.delay.written(seq, int(now_ns))

    def _done(self, seq: int, ns: int) -> None:
        self._done_ns[(self._conn(), seq & 0xFFFF)] = int(ns)

    def acknowledged(self, seq: int) -> bool:
        """A STATE on the current connection and boot stepped to ``seq`` after ``seq``'s last byte was written on it
        (Codex 7a-1 round 2 P2-1: not after its first byte)."""
        conn = self._conn()
        st = self.host.state
        w = self._done_ns.get((conn, seq & 0xFFFF))
        if st is None or w is None:
            return False
        rx = self._acks.get((conn, int(st.boot_id), seq & 0xFFFF))
        return rx is not None and rx >= w

    # --- the session pair ------------------------------------------------------------------------------------
    def _restart_session(self, why: str) -> None:
        self.delay.restart_check()
        self._session, self.session_ready = None, False
        self.rec.put({"t": "session_restart", "why": why, "host_ns": int(self.clock()), "conn": self._conn()})

    def _disarms(self, now: int) -> None:
        if self.link.link is None:
            return
        sess = self._session
        if sess is not None and now >= sess["deadline"]:
            # a stage not answered in time (a STATE that already named the seq, a lost answer): the pair starts
            # again with fresh seqs (Codex 7a-1 round 2 P2-2)
            self.rec.put({"t": "session_retry", "stage": sess["stage"], "host_ns": int(now), "conn": self._conn()})
            self.delay.restart_check()
            self._session = sess = None
        if not self.session_ready and sess is None:
            self.host.request_disarm(now)
            self._session = {"stage": "first", "seq": self.host.queued_disarm_seq(),
                             "deadline": now + int(SESSION_STAGE_S * 1e9)}
            self.counts["sessions"] += 1
        elif sess is not None and sess["stage"] == "first" and sess["seq"] is not None \
                and self.acknowledged(sess["seq"]):
            first = sess["seq"]
            # a step of 1024-64512 between the two seqs: queuing the first already moved the next seq by one
            self.host.skip_seq(self.rng.randint(1024, 64512) - 1)
            self.host.request_disarm(now)
            second = self.host.queued_disarm_seq()
            self.delay.expect_step(first, second)
            self._session = {"stage": "second", "seq": second, "first": first,
                             "deadline": now + int(SESSION_STAGE_S * 1e9)}
        elif sess is not None and sess["stage"] == "second" and self.delay.verified:
            # done when the delay bound's own session check passed (a STATE stepping from first to second)
            self._session, self.session_ready = None, True
            self.rec.put({"t": "session_ready", "host_ns": int(now), "conn": self._conn(), "verified": True})
            self._next_refresh_ns = now + int(DISARM_EVERY_S * 1e9)
        elif self.session_ready and self._next_refresh_ns is not None and now >= self._next_refresh_ns:
            self.host.request_disarm(now)
            self._next_refresh_ns = now + int(DISARM_EVERY_S * 1e9)

    # --- one cycle -------------------------------------------------------------------------------------------
    def cycle(self, *, refresh: bool = True) -> None:
        now = self.clock()
        for e in self.link.service(self.host, now):
            self.rec.put({"t": "event", "kind": e.kind, "host_ns": int(now), "conn": self._conn(),
                          **{k: str(v) for k, v in e.detail.items()}})
            if e.kind in ("connected", "disconnected"):
                self._restart_session(e.kind)
                if e.kind == "disconnected" and self.slam is not None:
                    self.slam.on_disconnect()
        if refresh:
            self._disarms(now)
        if self.feed is not None:
            self._scans(now)
            if (self._last_scan_ns is not None and not self._scan_gap_open
                    and self.clock() - self._last_scan_ns > SCAN_GAP_NS):
                # no scan for longer than the sensor timeout, seen on the tick it happens -- not only when one comes
                # back (Codex 7a-1 round 2 P2-4): a new generation, once per gap
                self._open_gap(self.clock())
        if self.imu is not None:
            self._imu(self.clock())
        if self.slam is not None:
            self.slam.cycle(self.clock())

    def _imu(self, now: int) -> None:
        """The accel samples received so far, through the source's verdict, generation and interval, into the record;
        then the source's generation (a receive gap with no sample opens one here, design 5)."""
        self.imu.samples(now, IMU_READ_PER_CYCLE)
        if self.imu.last_read_capped:
            self.counts["imu_read_capped"] += 1
        else:
            self.imu.generation()  # as the loop: only after a read that did not stop at its cap
        for row in self.imu.drain_rows():
            self.rec.put(row)

    def drain_imu(self) -> None:
        """After the device stopped: every sample still queued into the record (round 1 P2: not just one read)."""
        if self.imu is None:
            return
        now = self.clock()
        while True:
            self.imu.samples(now, IMU_READ_PER_CYCLE)
            capped = self.imu.last_read_capped
            for row in self.imu.drain_rows():
                self.rec.put(row)
            if not capped:
                return

    def _open_gap(self, now: int) -> None:
        self._scan_gap_open = True
        self.counts["scan_gaps"] += 1
        self.feed.new_generation("receive_gap")
        self.rec.put({"t": "scan_event", "kind": "receive_gap", "since_ns": int(self._last_scan_ns),
                      "host_ns": int(now), "feed_gen": self.feed.generation})

    def _scans(self, now: int) -> None:
        for _k in range(SCANS_READ_PER_CYCLE):
            try:
                data = self.scan_sock.recv(RECV_BYTES)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as exc:  # a socket error: recorded, and a new generation (design 3.2)
                self.counts["scan_socket_errors"] += 1
                self.feed.new_generation("socket_error")
                self.rec.put({"t": "scan_event", "kind": "socket_error", "error": str(exc), "host_ns": int(now),
                              "feed_gen": self.feed.generation})
                return
            if not data:
                break
            recv = self.clock()
            if (not self._scan_gap_open and self._last_scan_ns is not None
                    and recv - self._last_scan_ns > SCAN_GAP_NS):
                # the gap ended on this very read (no quiet tick saw it): opened here, before the receipt is taken
                # as the newest (Codex 7a-1 round 3 P2-1)
                self._open_gap(recv)
            if self._scan_gap_open:
                self._scan_gap_open = False
                self.rec.put({"t": "scan_event", "kind": "receive_resumed", "host_ns": int(recv),
                              "gap_s": (recv - self._last_scan_ns) / 1e9, "feed_gen": self.feed.generation})
            self._last_scan_ns = recv
            scan, why = self.feed.judge(data, recv)
            self.counts["scans"] += 1
            row = {"t": "scan", "host_recv_ns": int(recv), "frame": base64.b64encode(data).decode("ascii"),
                   "verdict": why, "feed_gen": self.feed.generation}
            if scan is not None:
                layer, lwhy = to_layer_scan(scan, self.lidar, self.start_ns)
                row.update(scan_id=scan.scan_id, producer_gen=scan.producer_gen, layer=lwhy or "ok")
                if layer is not None:
                    self.counts["scans_layer_ok"] += 1
            self.rec.put(row)
            if scan is not None and self.slam is not None:
                self.slam.on_scan(scan)  # the same accepted scan as the layer's path (design 6.1, 1)
        else:
            self.counts["read_capped"] += 1  # more may be waiting: left for the next cycle (design 3.3)

    # --- the end ---------------------------------------------------------------------------------------------
    def finish(self) -> bool:
        """The end DISARM, bound to its own (connection, seq): confirmed by a STATE on that connection stepping to
        that seq after it was written, in a stopped mode. A reconnection while waiting asks again on the new link.
        An acknowledgement is a step from an earlier STATE: when the DISARM went out before any STATE was seen on
        the connection and the first one seen already names it (an end straight after the opening, as a failed IMU
        start), no step can show it -- it is asked once more with a new seq (Codex 7b round 1 P1-4)."""
        if not getattr(self.link, "ever_connected", False):
            return False
        end = time.monotonic() + END_CONFIRM_S
        asked: tuple[int, int] | None = None
        unseen = reasked = False
        while time.monotonic() < end:
            conn = self._conn()
            if self.link.link is not None and (asked is None or asked[0] != conn):
                unseen = not (self._prev is not None and self._prev[0] == conn)
                reasked = False
                asked = self._ask_end(conn)
            self.cycle(refresh=False)
            st = self.host.state
            if asked is not None and st is not None and asked[0] == self._conn() and self.acknowledged(asked[1]) \
                    and st.last_rx_seq == asked[1] and st.mode in (Mode.DISARMED, Mode.FAULT, Mode.ESTOP):
                return True
            if asked is not None and unseen and not reasked and st is not None and asked[0] == self._conn() \
                    and self.link.link is not None and st.last_rx_seq == asked[1] \
                    and asked in self._done_ns and not self.acknowledged(asked[1]):
                # only once its last byte is out: a DISARM still queued would be reused with the same seq (Codex
                # 7b round 2 P2)
                reasked = True
                asked = self._ask_end(asked[0])
            time.sleep(CYCLE_S / 2)
        return False

    def _ask_end(self, conn: int):
        self.host.request_disarm(self.clock())
        seq = self.host.queued_disarm_seq()
        return None if seq is None else (conn, seq)


class Openings:
    """The link factory: every opening -- the first and each reopening -- gets an ObserveTtyLink and a number (the
    record's ``conn``), counted as the link is made, before any of its bytes are read."""

    def __init__(self) -> None:
        self.count = 0
        self.on_done = None  # the observer's completion tap, set when it is made

    def __call__(self, fd: int) -> ObserveTtyLink:
        self.count += 1
        return ObserveTtyLink(fd, on_done=lambda seq, ns: self.on_done and self.on_done(seq, ns))


def _start_imu(dev, imu, cfg, gap_max_s, rec) -> str | None:
    """Start the device and record its profile with the watch fit; the failing stage (``start``/``profile``) is
    recorded and returned (Codex 7b round 1 P2)."""
    stage = "start"
    try:
        dev.start(imu.on_sample)  # the options are written here; the callback only queues (design 5)
        stage = "profile"
        prof = dev.profile_info()
    except (RuntimeError, OSError) as exc:
        rec.put({"t": "imu_error", "stage": stage, "error": f"{type(exc).__name__}: {exc}",
                 "host_ns": time.monotonic_ns()})
        return stage
    hz = (prof.get("actual") or {}).get("accel_hz")
    bound = imu.bound_ns / 1e9
    rec.put({"t": "imu_profile", "host_ns": time.monotonic_ns(), **prof,
             "fits_watch": {"gap_max_s": gap_max_s, "accel_hz": hz, "interval_bound_s": bound,
                            "ok": fits_watch(cfg, gap_max_s, bound_s=bound, accel_hz=hz)}})
    return None


def _realsense(cfg):
    from imu_record import RealSenseSource  # the motion module opened directly (2026-11-14 design 2)

    return RealSenseSource(gyro_hz=cfg.gyro_hz, accel_hz=cfg.accel_hz, global_time=False)


def _yaml_error():
    import yaml

    return yaml.YAMLError


def _xml_error():
    from xml.etree.ElementTree import ParseError

    return ParseError


def _timing_header(lidar) -> dict:
    """The bound and what it rests on, as configured (7a-3): whether a frame used it is that frame's own verdict."""
    return {"first_beam_err_s": lidar.first_beam_err_s, "timing_basis": lidar.timing_basis,
            "timing_decision": lidar.timing_decision, "timing_doc": lidar.timing_doc,
            "applies_to": dict(lidar.applies_to) or None}


def _slam_parts(args, lidar) -> dict:
    """What the SLAM path needs, read before anything is opened (a bad file refuses the run): the bundle's grid and
    map id, the registered start (base_link), the vehicle's odometry and the laser in base_link."""
    from forklift_core.live.real_autonomy import load_s10a
    from forklift_core.live.real_run import RUN_STATE_DELAY_S, chassis_reference
    from forklift_core.live.real_teleop import EncoderSpeedParams
    from forklift_core.localization.slam_map import load_bundle
    from forklift_core.localization.slam_real_scan import base_from_laser
    from forklift_core.localization.start_reloc import base_from_rear
    from forklift_core.plant.real_vehicle import chassis_model_path, load_real_vehicle

    import math

    if args.slam_bundle is None:
        raise ValueError("--slam needs --slam-bundle")
    if not args.scan_socket:
        raise ValueError("--slam needs --scan-socket (its scans are the feed's)")
    def named(path, read):
        try:
            return read()
        except Exception as exc:  # noqa: BLE001 -- refused with the file that could not be read (round 2 P2-2)
            raise ValueError(f"{path}: {type(exc).__name__}: {exc}") from exc

    bundle = named(args.slam_bundle, lambda: load_bundle(args.slam_bundle))
    cfg = named(args.s10a, lambda: load_s10a(args.s10a))
    vehicle = named(args.vehicle, lambda: load_real_vehicle(args.vehicle))
    urdf = chassis_model_path(args.vehicle).parent / "forklift.urdf"
    _tip, rear_offset, _carriage = named(urdf, lambda: chassis_reference(urdf))
    kappa = max(abs(k) for k in vehicle.steering.curvatures_inv_m)
    params = EncoderSpeedParams(radius_m=vehicle.wheel_radius_m, counts_per_rev=vehicle.counts_per_wheel_rev,
                                track_m=vehicle.track_m, kappa_max=kappa, axle_accel_mps2=0.5, max_age_s=0.1,
                                dt_min_s=0.02, dt_max_s=0.2)
    start = tuple(float(v) for v in cfg.start_rear)
    if len(start) != 3 or not all(math.isfinite(v) for v in start) or not math.isfinite(float(cfg.max_speed_mps)):
        raise ValueError("s10a: start_rear and max_speed_mps must be finite")
    return {"bundle": bundle, "vehicle": vehicle, "params": params, "rear_offset_m": rear_offset, "kappa": kappa,
            "start_pose": base_from_rear(tuple(float(v) for v in cfg.start_rear), rear_offset),
            "base_laser": base_from_laser(lidar.mount, rear_offset), "state_delay_s": RUN_STATE_DELAY_S,
            "v_scan_mps": 1.5 * float(cfg.max_speed_mps)}


def _slam_start(args, parts, rec, lidar):
    """The source connected to the bridge (a refused connection refuses the run before the board is opened)."""
    import socket

    from forklift_core.live.observe_slam import ObserveSlam
    from forklift_core.live.real_slam_source import RealSlamLink, RealSlamSource
    from forklift_core.plant.real_vehicle import RealOdometry

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.settimeout(5.0)
        sock.connect(str(args.slam))
        start_ns = time.monotonic_ns()
        # diagnostic (7a-3 plan 3): the observe run records the SLAM path, it never hands a reply over as evidence
        src = RealSlamSource(RealSlamLink(sock), mode="diagnostic", map_id=parts["bundle"].map_id, start_ns=start_ns,
                             start_pose=parts["start_pose"], odom_history=None, odom_base=None, odom_gen=None,
                             boot=None, rear_offset_m=parts["rear_offset_m"], session_epoch=0)
    except BaseException:
        sock.close()
        raise
    rec.put({"t": "slam_header", "socket": str(args.slam), "bundle": str(args.slam_bundle),
             "map_id": parts["bundle"].map_id, "start_pose": list(parts["start_pose"]),
             "rear_offset_m": parts["rear_offset_m"], "base_laser": list(parts["base_laser"]), "start_ns": start_ns,
             # the mode only; which frames had an interval is per frame (the end's frames_measured /
             # frames_diagnostic, 7a-3 plan 3, Codex P3)
             "mode": "diagnostic", "timing": _timing_header(lidar)})
    return ObserveSlam(src, rec.put, odometry=RealOdometry(parts["vehicle"], max_gap_s=0.2), params=parts["params"],
                       rear_offset_m=parts["rear_offset_m"], start_ns=start_ns, grid=parts["bundle"].grid,
                       base_laser=parts["base_laser"], lidar=lidar, state_delay_s=parts["state_delay_s"],
                       v_scan_mps=parts["v_scan_mps"], kappa_max=parts["kappa"])


def main(argv=None, *, imu_factory=_realsense) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--device", required=True)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--seconds", type=float, required=True)
    ap.add_argument("--scan-socket", help="bind here for the scan adapter's frames (a socket of its own, PR 7 design 4)")
    ap.add_argument("--lidar-config", type=Path, default=LIDAR_YAML)
    ap.add_argument("--baud", type=int, choices=sorted(_BAUDS))
    ap.add_argument("--bench", help="rehearsal only: the bench's bench.json on a pty device (the model MCU)")
    ap.add_argument("--imu", action="store_true", help="record the D435i accelerometer (PR 7 design 5, 7b)")
    ap.add_argument("--imu-config", type=Path, default=IMU_YAML)
    ap.add_argument("--slam", help="the real-mode SLAM bridge's socket: record the real SLAM source (7c-1c)")
    ap.add_argument("--slam-bundle", type=Path, help="with --slam: the survey bundle the bridge runs on")
    ap.add_argument("--s10a", type=Path, default=REPO / "config" / "real" / "s10a.yaml",
                    help="with --slam: the registered start (start_rear) and the speed the scan placement assumes")
    ap.add_argument("--vehicle", type=Path, default=REPO / "config" / "real" / "vehicle.yaml",
                    help="with --slam: the vehicle file (odometry) and its chassis model (rear axle offset)")
    args = ap.parse_args(argv)
    if not 0 < args.seconds <= MAX_SECONDS:
        print(json.dumps({"result": "refused", "why": f"--seconds must be in (0, {MAX_SECONDS:g}]"}))
        return 2
    try:
        synthetic = writer_marker(args.device, args.bench)
        lidar = load_lidar_config(args.lidar_config)
        imu_cfg = load_imu_source_config(args.imu_config) if args.imu else None
        imu_gap_max_s = load_imu_config(SAFETY_YAML).gap_max_s if args.imu else None
        slam_parts = _slam_parts(args, lidar) if args.slam else None
    except (SyntheticRefused, OSError, ValueError, KeyError, TypeError, AttributeError, _yaml_error(),
            _xml_error()) as exc:
        print(json.dumps({"result": "refused", "why": str(exc)}))
        return 2
    if args.out.exists():
        print(json.dumps({"result": "refused", "why": f"{args.out} exists: every observe run gets its own record"}))
        return 2
    imu_dev = None
    if imu_cfg is not None:
        try:
            imu_dev = imu_factory(imu_cfg)  # finds the device only (no option written before the executor lock); no
            # device, no SDK: refused before anything is opened or written
        except (RuntimeError, ImportError, OSError) as exc:
            print(json.dumps({"result": "refused", "why": f"imu: {exc}"}))
            return 2
    opener = open_raw if synthetic is None else bench_opener(open_raw, synthetic["bench"])
    openings = Openings()
    link = ReconnectingLink(args.device, baud=args.baud, opener=opener, clock=time.monotonic_ns, exclusive=True,
                            link_factory=openings)
    scan_sock = bind_socket(args.scan_socket) if args.scan_socket else None
    args.out.parent.mkdir(parents=True, exist_ok=True)
    stopped = False
    with open(args.out, "x", encoding="utf-8") as f:
        rec = Record(f)
        rec.put({"t": "header", "schema": SCHEMA, "tool": "real_observe.py", "device": args.device,
                 "device_id": device_identity(args.device), "synthetic": synthetic, "start_wall": time.time(),
                 "start_mono_ns": time.monotonic_ns(), "seconds": args.seconds, "scan_socket": args.scan_socket,
                 "lidar": {"path": str(args.lidar_config), "mount_measured": lidar.mount_measured,
                           **_timing_header(lidar)},
                 "imu": None if imu_cfg is None else {
                     "path": str(args.imu_config), "measured": imu_cfg.measured, "delay_bound_s": imu_cfg.delay_bound_s,
                     "interval_bound_s": imu_cfg.delay_bound_s if imu_cfg.delay_bound_s is not None
                     else imu_cfg.provisional_delay_bound_s, "operating": False}})
        imu = None if imu_cfg is None else ImuSource(imu_cfg, operating=False, record=True)
        slam = None
        if slam_parts is not None:
            try:
                slam = _slam_start(args, slam_parts, rec, lidar)
            except OSError as exc:
                rec.put({"t": "end", "result": "refused", "why": f"slam_bridge: {exc}", "complete": False})
                print(json.dumps({**(OUTPUT_MARKS if synthetic else {}), "result": "refused",
                                  "why": f"slam_bridge: {exc}"}))
                return 2
        obs = Observer(link, rec, openings=openings, scan_sock=scan_sock, lidar=lidar, imu=imu, slam=slam)
        try:
            link.open_first(obs.host, time.monotonic_ns())
        except DeviceBusy as busy:
            if slam is not None:  # the bridge was connected first: closed here, its state in the end row (P3-1)
                slam.flush()
                slam.src.link.sock.close()
            rec.put({"t": "end", "result": "refused", "why": str(busy), "holder": busy.holder, "complete": False,
                     "slam": None if slam is None else slam.summary()})
            print(json.dumps({**(OUTPUT_MARKS if synthetic else {}), "result": "refused", "why": str(busy)}))
            return 2
        result = "aborted"
        imu_started = False
        try:
            imu_error = None
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:
                t0 = time.monotonic()
                if imu_dev is not None and not imu_started and link.link is not None:
                    # the D435i starts (its options written) only once the locked board link is actually open --
                    # open_first may only have scheduled a retry (Codex 7b round 2 P2)
                    imu_started = True  # set first: a start interrupted inside the SDK is stopped too
                    imu_error = _start_imu(imu_dev, imu, imu_cfg, imu_gap_max_s, rec)
                    if imu_error is not None:
                        break
                    end = time.monotonic() + args.seconds  # --seconds of sensor observation, from its start
                obs.cycle()
                time.sleep(max(0.0, CYCLE_S - (time.monotonic() - t0)))
            if imu_error is not None:
                result = f"imu_{imu_error}_failed"
            elif imu_dev is not None and not imu_started:
                result = "imu_not_started"  # the board link never opened
            else:
                result = "ok"
        except KeyboardInterrupt:
            result = "interrupted"
        except DeviceBusy as busy:
            # held at a retried first opening (the device was missing at open_first): refused as at the first one,
            # nothing ever written to it (Codex 7b round 3 P2)
            result = "refused"
            rec.put({"t": "refused", "why": str(busy), "holder": busy.holder, "host_ns": time.monotonic_ns()})
        finally:
            try:
                stopped = obs.finish()
            except Exception as exc:  # noqa: BLE001 -- the end row is still written
                rec.put({"t": "finish_error", "error": f"{type(exc).__name__}: {exc}"})
            imu_stopped = True
            if imu_started:
                try:
                    imu_dev.stop()  # its callbacks have ended when it returns; the last samples are read below
                except Exception as exc:  # noqa: BLE001 -- callbacks may still run: the record cannot be complete
                    imu_stopped = False
                    rec.put({"t": "imu_error", "stage": "stop", "error": f"{type(exc).__name__}: {exc}",
                             "host_ns": time.monotonic_ns()})
                    if result == "ok":
                        result = "imu_stop_failed"
                obs.drain_imu()
            # every row before the end row is on disk before the end row says so; the end row's own write and sync
            # count too, and only then the sidecar <out>.ok (the record's sha256) is made -- a replay takes the
            # record only with a matching .ok, so a file that claims more than was kept is never read as complete
            # (Codex 7a-1 P2-5, round 2 P2-5)
            synced = _sync(f)
            imu_overflow = imu is not None and imu.counts["overflow"] > 0
            if imu_overflow and result == "ok":
                # raw samples lost before they were recorded (round 2 P2): before no_imu and no_scans (round 4 P2)
                result = "imu_overflow"
            if obs.feed is not None and obs.feed.accepted == 0 and result == "ok":
                result = "no_scans"  # a scan socket nobody sent to is not an observed source
            if imu is not None and imu.counts["accepted"] == 0 and result == "ok":
                result = "no_imu"
            if slam is not None:
                slam.flush()
                if slam.fatal is not None and result == "ok":
                    result = "slam_failed"
                elif slam.replies() == 0 and result == "ok":
                    result = "no_slam_replies"  # a bridge that answered nothing is no observed source (P2-3)
            errors_before = rec.errors
            rec.put({"t": "end", "result": result, "stopped_confirmed": stopped,
                     "complete": errors_before == 0 and synced and imu_stopped and not imu_overflow
                     and result != "refused",
                     "record_errors": errors_before,
                     "counts": obs.counts,
                     "feed": None if obs.feed is None else {"accepted": obs.feed.accepted,
                                                            "refused": dict(obs.feed.refused),
                                                            "id_gaps": obs.feed.id_gaps,
                                                            "generation": obs.feed.generation,
                                                            "generation_why": obs.feed.generation_why},
                     "imu": None if imu is None else {"counts": dict(imu.counts), "generation": imu.gen,
                                                      "generation_why": imu.generation_why,
                                                      "stopped": imu.stopped,  # round 5 P3: a restart is needed
                                                      "last_generation_ns": imu.last_generation_ns},
                     "slam": None if slam is None else slam.summary(),
                     "end_mono_ns": time.monotonic_ns()})
            complete = errors_before == 0 and synced and rec.errors == 0 and _sync(f) and imu_stopped \
                and not imu_overflow and result != "refused"
            if link.link is not None:
                link.link.close()
                link.link = None
            if scan_sock is not None:
                scan_sock.close()
            if slam is not None:
                slam.src.link.sock.close()
    if complete:
        try:
            write_ok_marker(args.out)
        except OSError:
            complete = False
    out = {**(OUTPUT_MARKS if synthetic else {}), "result": result, "stopped_confirmed": stopped,
           "complete": complete, "record": str(args.out), "counts": obs.counts}
    if imu is not None:
        out["imu_stopped"] = imu.stopped  # the source stopped (a new device clock, an overflow, a broken bound)
    print(json.dumps(out))
    if result == "refused":
        return 2  # never connected: nothing written to the device
    return 0 if (result == "ok" and stopped and complete) else 1


if __name__ == "__main__":
    sys.exit(main())
