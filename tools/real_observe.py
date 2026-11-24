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
  (7a-2) none can, and every scan stays diagnostic). The last row is ``end``: complete only if every row was written.

usage: real_observe.py --device /dev/forklift-mcu --out run.jsonl --seconds 60 [--scan-socket /run/forklift-scan/scan.sock]
(the scan socket is the one ``tools/ros2_dev.sh scan-a2m12`` sends to; the teleop bridge binds the same path, and the
executor lock keeps the two from running together)
exit: 0 complete and stopped (and, with --scan-socket, at least one scan accepted), 1 otherwise, 2 refused (nothing
written to the device)."""

from __future__ import annotations

import argparse
import base64
import json
import os
import random
import sys
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
from forklift_core.sensors.lidar_cfg import LIDAR_YAML, load_lidar_config, to_layer_scan  # noqa: E402
from forklift_core.sensors.scan_feed import RECV_BYTES, ScanFeed, bind_socket  # noqa: E402

SCHEMA = "observe_record/1"
CYCLE_S = 0.02  # 50 Hz, as the executor's loop
DISARM_EVERY_S = 1.0  # the OFF path's refresh (real_autonomy.ANCHOR_DISARM_S)
END_CONFIRM_S = 1.0
SCANS_READ_PER_CYCLE = 8  # the read cost per cycle (design 3.3): 10 Hz scans, 50 Hz cycles -- ample, never drained
MAX_SECONDS = 3600.0
SCAN_GAP_NS = 350_000_000  # a scan receive gap past the layer's sensor timeout (obstacle_layer_real 0.35 s)
HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=16)


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

    def __init__(self, link, record: Record, *, openings=None, scan_sock=None, lidar=None,
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
        self.counts = {"states": 0, "written": 0, "refused": 0, "scans": 0, "scans_layer_ok": 0, "read_capped": 0,
                       "scan_gaps": 0, "scan_socket_errors": 0, "sessions": 0}
        self._writes: dict[tuple[int, int], int] = {}  # (conn, seq) -> written ns (DISARMs)
        self._acks: dict[tuple[int, int, int], int] = {}  # (conn, boot, seq) -> receipt of the STATE stepping to it
        self._prev: tuple[int, int, int] | None = None  # the previous accepted STATE: (conn, boot, last_rx_seq)
        self._session: dict | None = None  # the pair in progress: {"stage", "seq", "first"}
        self.session_ready = False
        self._next_refresh_ns: int | None = None
        self._last_scan_ns: int | None = None

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

    def _log(self, what, now_ns, data, info) -> None:
        if what in ("written", "refused") and info:
            self.counts[what] += 1
            self.rec.put({"t": what, "kind": info.get("kind"), "seq": info.get("seq"), "host_ns": int(now_ns),
                          "conn": self._conn(), **({"why": info["why"]} if "why" in info else {})})
            if what == "written" and info.get("kind") == "DISARM":
                seq = int(info["seq"]) & 0xFFFF
                self._writes[(self._conn(), seq)] = int(now_ns)
                self.delay.written(seq, int(now_ns))

    def acknowledged(self, seq: int) -> bool:
        """A STATE on the current connection and boot stepped to ``seq`` after ``seq`` was written on it."""
        conn = self._conn()
        st = self.host.state
        w = self._writes.get((conn, seq & 0xFFFF))
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
        if not self.session_ready and self._session is None:
            self.host.request_disarm(now)
            self._session = {"stage": "first", "seq": self.host.queued_disarm_seq()}
            self.counts["sessions"] += 1
        elif self._session is not None and self._session["seq"] is not None \
                and self.acknowledged(self._session["seq"]):
            if self._session["stage"] == "first":
                first = self._session["seq"]
                self.host.skip_seq(self.rng.randint(1024, 64512))
                self.host.request_disarm(now)
                second = self.host.queued_disarm_seq()
                self.delay.expect_step(first, second)
                self._session = {"stage": "second", "seq": second, "first": first}
            else:
                self._session, self.session_ready = None, True
                self.rec.put({"t": "session_ready", "host_ns": int(now), "conn": self._conn(),
                              "verified": self.delay.verified})
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
        if refresh:
            self._disarms(now)
        if self.feed is not None:
            self._scans(now)

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
            if self._last_scan_ns is not None and recv - self._last_scan_ns > SCAN_GAP_NS:
                # a receive gap past the sensor timeout: the stream before it says nothing about after (design 3.2)
                self.counts["scan_gaps"] += 1
                self.feed.new_generation("receive_gap")
                self.rec.put({"t": "scan_event", "kind": "receive_gap", "gap_s": (recv - self._last_scan_ns) / 1e9,
                              "host_ns": int(recv), "feed_gen": self.feed.generation})
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
        else:
            self.counts["read_capped"] += 1  # more may be waiting: left for the next cycle (design 3.3)

    # --- the end ---------------------------------------------------------------------------------------------
    def finish(self) -> bool:
        """The end DISARM, bound to its own (connection, seq): confirmed by a STATE on that connection stepping to
        that seq after it was written, in a stopped mode. A reconnection while waiting asks again on the new link."""
        if not getattr(self.link, "ever_connected", False):
            return False
        end = time.monotonic() + END_CONFIRM_S
        asked: tuple[int, int] | None = None
        while time.monotonic() < end:
            if self.link.link is not None and (asked is None or asked[0] != self._conn()):
                self.host.request_disarm(self.clock())
                seq = self.host.queued_disarm_seq()
                asked = None if seq is None else (self._conn(), seq)
            self.cycle(refresh=False)
            st = self.host.state
            if asked is not None and st is not None and asked[0] == self._conn() and self.acknowledged(asked[1]) \
                    and st.last_rx_seq == asked[1] and st.mode in (Mode.DISARMED, Mode.FAULT, Mode.ESTOP):
                return True
            time.sleep(CYCLE_S / 2)
        return False


class Openings:
    """The link factory: every opening -- the first and each reopening -- gets an ObserveTtyLink and a number (the
    record's ``conn``), counted as the link is made, before any of its bytes are read."""

    def __init__(self) -> None:
        self.count = 0

    def __call__(self, fd: int) -> ObserveTtyLink:
        self.count += 1
        return ObserveTtyLink(fd)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--device", required=True)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--seconds", type=float, required=True)
    ap.add_argument("--scan-socket", help="bind here for the scan adapter's frames (a socket of its own, PR 7 design 4)")
    ap.add_argument("--lidar-config", type=Path, default=LIDAR_YAML)
    ap.add_argument("--baud", type=int, choices=sorted(_BAUDS))
    ap.add_argument("--bench", help="rehearsal only: the bench's bench.json on a pty device (the model MCU)")
    args = ap.parse_args(argv)
    if not 0 < args.seconds <= MAX_SECONDS:
        print(json.dumps({"result": "refused", "why": f"--seconds must be in (0, {MAX_SECONDS:g}]"}))
        return 2
    try:
        synthetic = writer_marker(args.device, args.bench)
        lidar = load_lidar_config(args.lidar_config)
    except (SyntheticRefused, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"result": "refused", "why": str(exc)}))
        return 2
    if args.out.exists():
        print(json.dumps({"result": "refused", "why": f"{args.out} exists: every observe run gets its own record"}))
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
                           "timing_measured": lidar.timing_measured, "first_beam_err_s": lidar.first_beam_err_s}})
        obs = Observer(link, rec, openings=openings, scan_sock=scan_sock, lidar=lidar)
        try:
            link.open_first(obs.host, time.monotonic_ns())
        except DeviceBusy as busy:
            rec.put({"t": "end", "result": "refused", "why": str(busy), "holder": busy.holder, "complete": False})
            print(json.dumps({**(OUTPUT_MARKS if synthetic else {}), "result": "refused", "why": str(busy)}))
            return 2
        result = "aborted"
        try:
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:
                t0 = time.monotonic()
                obs.cycle()
                time.sleep(max(0.0, CYCLE_S - (time.monotonic() - t0)))
            result = "ok"
        except KeyboardInterrupt:
            result = "interrupted"
        finally:
            try:
                stopped = obs.finish()
            except Exception as exc:  # noqa: BLE001 -- the end row is still written
                rec.put({"t": "finish_error", "error": f"{type(exc).__name__}: {exc}"})
            # every row before the end row is on disk before the end row says so (Codex 7a-1 P2-5): a sync that
            # fails makes the end row say incomplete; the end row's own sync failing leaves no end row a reader
            # trusts -- either way the file never claims more than was kept
            try:
                f.flush()
                os.fsync(f.fileno())
                synced = True
            except OSError:
                synced = False
            complete = rec.errors == 0 and synced
            if obs.feed is not None and obs.feed.accepted == 0 and result == "ok":
                result = "no_scans"  # a scan socket nobody sent to is not an observed source
            rec.put({"t": "end", "result": result, "stopped_confirmed": stopped, "complete": complete,
                     "record_errors": rec.errors, "counts": obs.counts,
                     "feed": None if obs.feed is None else {"accepted": obs.feed.accepted,
                                                            "refused": dict(obs.feed.refused),
                                                            "id_gaps": obs.feed.id_gaps,
                                                            "generation": obs.feed.generation,
                                                            "generation_why": obs.feed.generation_why},
                     "end_mono_ns": time.monotonic_ns()})
            try:
                f.flush()
                os.fsync(f.fileno())
            except OSError:
                complete = False
            if link.link is not None:
                link.link.close()
                link.link = None
            if scan_sock is not None:
                scan_sock.close()
    out = {**(OUTPUT_MARKS if synthetic else {}), "result": result, "stopped_confirmed": stopped,
           "complete": complete, "record": str(args.out), "counts": obs.counts}
    print(json.dumps(out))
    return 0 if (result == "ok" and stopped and complete) else 1


if __name__ == "__main__":
    sys.exit(main())
