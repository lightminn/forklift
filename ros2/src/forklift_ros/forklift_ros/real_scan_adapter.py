"""The real LiDAR's adapter (docs/design/2026-11-12-real-scan-obstacle-layer.md 1.2): subscribes to the driver's raw
scan and sends one scan/1 frame per message to the host over a Unix datagram socket. It changes nothing in the
values (the host's ScanFeed reads them) and never waits: no receiver, or a full one, drops the scan and counts it.

Each frame carries the driver stamp as given, the adapter's receipt on CLOCK_MONOTONIC and the (system - monotonic)
pair read at that moment, so the host can place the stamp on its monotonic clock (plan 2.5). The upstream node's
stamp is taken before it fetches a scan that may have waited in a buffer -- it is not the acquisition time (design
1.1); the host names the delay stamp-to-receipt.

The ROS imports live in ``main`` so the framing and the sender load and are tested without ROS. The frame layout
matches ``forklift_core.sensors.scan_feed.encode_frame`` (a root test decodes what this module makes)."""

from __future__ import annotations

import errno
import json
import re
import secrets
import socket
import struct
import time
from collections import Counter, OrderedDict

import numpy as np

MAGIC = b"SCN1"
_LEN = struct.Struct("<I")


SOURCES = {  # --source: the topic and what its stamp means (PR 7a-2: the topic decides, so the two never mismatch)
    "raw": ("/sensors/lidar/scan_raw", "call_start"),  # upstream node: stamped before it fetches a scan
    # forklift patches: the SDK's first sample -- v1 (0001, decode time) or v2 (0002, byte arrival), named per frame
    "sdk": ("/sensors/lidar/scan_raw_sdk", "sdk"),
}


SDK_MAX_AGE_NS = 5_000_000_000  # a first-sample time older than this at receipt is not one (a wrong topic, a remap)
SDK_MAX_SPAN_S = 0.5
SDK_FORMAT = "@sdk_mono_v1"  # forklift patch 0001 appends this to the frame_id of every scan_sdk message
SDK_FORMAT_V2 = "@sdk_mono_v2"  # patch 0002: sample times from the receive thread's byte arrival, + "#<run>.<start>"
STAMP_V1, STAMP_V2 = "sdk_first_sample", "sdk_first_sample_rx"  # the host's stamp_kind for each format
INFO_TOPIC = "/sensors/lidar/scan_sdk_info"  # patch 0002: the node's std_msgs/String (JSON), latched, one per start
INFO_FORMAT = "sdk_mono_v2"
_HEX = r"[0-9a-f]{8,32}"
_V2_ID = re.compile(r"^(?P<frame>.*)" + re.escape(SDK_FORMAT_V2) + r"#(?P<run>" + _HEX + r")\.(?P<start>" + _HEX + r")$")
RUN_ID = re.compile(r"^" + _HEX + r"$")


def _time_fields(stamp_ns: int, scan_time: float, recv_mono_ns: int, sys_minus_mono_ns: int) -> dict | None:
    if not (0 < stamp_ns <= recv_mono_ns and recv_mono_ns - stamp_ns <= SDK_MAX_AGE_NS):
        return None
    if not (0.0 < scan_time <= SDK_MAX_SPAN_S):
        return None
    # the span is whole microseconds at the source; scan_time is a float32 on the wire -- round to the microsecond
    # first, so the last sample comes back exactly (round 2 P3)
    last = stamp_ns + int(round(scan_time * 1e6)) * 1000
    if last > recv_mono_ns:
        return None
    return {"first_beam_mono_ns": int(stamp_ns), "last_beam_mono_ns": int(last),
            # the system-clock equivalent the host's checks read (stamp-to-receipt, the clock pair)
            "ros_stamp_ns": int(stamp_ns + sys_minus_mono_ns)}


def sdk_fields(stamp_ns: int, scan_time: float, recv_mono_ns: int, sys_minus_mono_ns: int, *,
               frame_id: str) -> dict | None:
    """The frame fields of a v1 scan_sdk message (forklift patch 0001): its stamp is the SDK's first sample on
    CLOCK_MONOTONIC as it is, its scan_time the same buffer's last - first sample. None when the message does not
    look like one -- an epoch stamp (the upstream topic remapped here), a time in the future or too old, a span that
    is not one rotation -- so a wrong topic gives no scan, never a call-start stamp taken for a first sample (Codex
    7a-2 P2-3)."""
    if not str(frame_id).endswith(SDK_FORMAT):
        return None  # the format's own name is required: any other LaserScan is not a scan_sdk message (round 2 P2-3)
    return _time_fields(stamp_ns, scan_time, recv_mono_ns, sys_minus_mono_ns)


class SdkStream:
    """The patched node's scan_sdk messages, v1 (patch 0001) and v2 (0002), as frame fields with their producer
    generation (7a-3 plan 2026-11-26 2-3):

    - a v2 message names the launcher's run (``run_id``) and the node's start (a new one at every scan start): one of
      another run is dropped (``other_run`` -- another v2 node on the topic), a v2 message with no run given to this
      adapter is dropped (``no_run_id``);
    - a new start (or a format change) is a new producer generation (a new random ``gen``) and retires the start it
      replaced: a retired start's late message is dropped (``retired_source``) and never opens a generation again;
    - ``on_info`` takes the node's ``scan_sdk_info`` (JSON: format, run, start, ans_type, sample_period_us,
      baudrate); a v2 frame carries ``source`` -- the launcher's facts (image_id, usb, host) with ``sdk_info`` only
      when the info of that very start was received -- so the host judges each frame alone;
    - anything else (no format name, times that do not fit) is ``not_sdk``. Each drop is counted in ``dropped``."""

    MAX_INFO = 8

    def __init__(self, run_id: str | None = None, facts: dict | None = None, rand=None) -> None:
        if run_id is not None and not RUN_ID.match(str(run_id)):
            raise ValueError("run id: 8-32 lowercase hex digits")
        self.run_id = run_id
        self.facts = {k: str(v) for k, v in (facts or {}).items() if v}
        self._rand = rand if rand is not None else (lambda: secrets.randbits(62) + 1)
        self.gen = self._rand()  # this adapter's start: the first producer generation
        self._key = None
        self.retired: set = set()
        self.info: OrderedDict = OrderedDict()
        self.dropped: Counter = Counter()

    def on_info(self, text: str) -> str | None:
        """None when taken, else why it was not (counted ``info_<why>``)."""
        try:
            d = json.loads(text)
        except (TypeError, ValueError):
            d = None
        why = None
        if not (isinstance(d, dict) and d.get("format") == INFO_FORMAT and isinstance(d.get("run"), str)
                and isinstance(d.get("start"), str) and RUN_ID.match(d["start"])
                and all(isinstance(d.get(k), int) and not isinstance(d.get(k), bool) and d[k] > 0
                        for k in ("ans_type", "baudrate"))
                and isinstance(d.get("sample_period_us"), (int, float)) and not isinstance(d["sample_period_us"], bool)
                and 0 < float(d["sample_period_us"]) < 1e6):
            why = "malformed"
        elif d["run"] != self.run_id:
            why = "other_run"
        elif d["start"] in self.retired:
            why = "retired_source"
        if why is not None:
            self.dropped[f"info_{why}"] += 1
            return why
        self.info[d["start"]] = {"ans_type": d["ans_type"], "sample_period_us": float(d["sample_period_us"]),
                                 "baudrate": d["baudrate"]}
        self.info.move_to_end(d["start"])
        while len(self.info) > self.MAX_INFO:
            self.info.popitem(last=False)
        return None

    def on_scan(self, stamp_ns: int, scan_time: float, recv_mono_ns: int, sys_minus_mono_ns: int, *,
                frame_id: str) -> tuple[dict | None, str | None]:
        """``({"sdk": frame fields, "stamp_kind": ..., "gen": ...}, None)`` or ``(None, why)``."""
        fid = str(frame_id)
        m = _V2_ID.match(fid)
        if fid.endswith(SDK_FORMAT):
            key, kind, frame, source = ("v1",), STAMP_V1, fid.removesuffix(SDK_FORMAT), None
        elif m is not None:
            if self.run_id is None:
                return self._drop("no_run_id")
            if m["run"] != self.run_id:
                return self._drop("other_run")
            if m["start"] in self.retired:
                return self._drop("retired_source")
            key, kind, frame = ("v2", m["start"]), STAMP_V2, m["frame"]
            source = {"run": m["run"], "start": m["start"], **self.facts}
            if m["start"] in self.info:
                source["sdk_info"] = dict(self.info[m["start"]])
        else:
            return self._drop("not_sdk")
        fields = _time_fields(stamp_ns, scan_time, recv_mono_ns, sys_minus_mono_ns)
        if fields is None:
            return self._drop("not_sdk")  # a frame that does not fit never retires the start still sending
        if self._key is not None and key != self._key:
            if self._key[0] == "v2":
                self.retired.add(self._key[1])
                self.info.pop(self._key[1], None)
            self.gen = self._rand()
        self._key = key
        fields["frame_id"] = frame
        if source is not None:
            fields["source"] = source
        return {"sdk": fields, "stamp_kind": kind, "gen": self.gen}, None

    def _drop(self, why: str):
        self.dropped[why] += 1
        return None, why


def make_frame(msg, *, scan_id: int, recv_mono_ns: int, sys_minus_mono_ns: int, gen: int | None = None,
               first_beam_mono_ns: int | None = None, stamp_kind: str | None = None,
               sdk: dict | None = None) -> bytes:
    """A sensor_msgs/LaserScan (or anything with its fields) as one scan/1 frame. ``gen``: this adapter's start
    nonce (PR 7 design 3.2: the host opens a new generation when it changes); ``first_beam_mono_ns``: the SDK
    first-sample time on the host monotonic clock, when the driver gives it (7a-2)."""
    ranges = np.asarray(msg.ranges, dtype="<f4")
    inten = np.asarray(msg.intensities, dtype="<f4") if len(msg.intensities) == len(ranges) and len(ranges) else None
    head = {
        "scan_id": int(scan_id),
        "ros_stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
        "recv_mono_ns": int(recv_mono_ns),
        "sys_minus_mono_ns": int(sys_minus_mono_ns),
        "angle_min": float(msg.angle_min),
        "angle_increment": float(msg.angle_increment),
        "time_increment": float(msg.time_increment),
        "scan_time": float(msg.scan_time),
        "range_min": float(msg.range_min),
        "range_max": float(msg.range_max),
        "frame_id": str(msg.header.frame_id),
        "n": int(len(ranges)),
        "has_intensities": inten is not None,
    }
    if gen is not None:
        head["gen"] = int(gen)
    if first_beam_mono_ns is not None:
        head["first_beam_mono_ns"] = int(first_beam_mono_ns)
    if stamp_kind is not None:
        head["stamp_kind"] = str(stamp_kind)
    if sdk is not None:
        head.update(sdk)  # a v2 stream's fields give the sensor frame and the source
        head["frame_id"] = head["frame_id"].removesuffix(SDK_FORMAT)  # the sensor frame, as on the upstream topic
    raw = json.dumps(head).encode("utf-8")
    body = ranges.tobytes() + (inten.tobytes() if inten is not None else b"")
    return MAGIC + _LEN.pack(len(raw)) + raw + body


def clock_pair() -> tuple[int, int]:
    """(monotonic, system - monotonic), the system clock read between two monotonic reads."""
    m0 = time.monotonic_ns()
    s = time.time_ns()
    m1 = time.monotonic_ns()
    m = (m0 + m1) // 2
    return m, s - m


class DatagramSender:
    """Sends whole frames to ``path``; anything that would wait or fail drops the frame (counted)."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sent = 0
        self.dropped = 0
        self.last_error: str | None = None

    def send(self, frame: bytes) -> bool:
        try:
            self.sock.sendto(frame, self.path)
        except (BlockingIOError, InterruptedError):
            self.dropped += 1
            self.last_error = "full"
            return False
        except OSError as exc:  # no receiver yet (ENOENT, ECONNREFUSED), or anything else
            self.dropped += 1
            self.last_error = errno.errorcode.get(exc.errno, str(exc.errno))
            return False
        self.sent += 1
        return True

    def close(self) -> None:
        self.sock.close()


def main(argv=None) -> int:
    import argparse

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    ap = argparse.ArgumentParser(description="raw LaserScan -> scan/1 over a Unix datagram socket")
    ap.add_argument("--socket", required=True, action="append",
                    help="a host's bound socket (forklift_core.sensors.scan_feed); give it twice for the layer and "
                         "the SLAM bridge (PR 7 design 4: the adapter is where one scan gets its identity)")
    ap.add_argument("--source", choices=sorted(SOURCES), default="raw",
                    help="raw: the upstream topic (call-start stamps); sdk: the patched node's scan_sdk topic, stamped "
                         "at the SDK's first sample (PR 7a-2)")
    ap.add_argument("--run-id", help="sdk: the launcher's run id (the node's forklift_run_id); v2 scans of another "
                                     "run, or any v2 scan without it, are dropped")
    ap.add_argument("--image-id", help="sdk: the docker image ID the launcher ran (a source fact of every v2 frame)")
    ap.add_argument("--usb-id", help="sdk: the LiDAR's USB adapter, vendor:product:serial from udev (a source fact)")
    ap.add_argument("--host", help="sdk: the host the launcher ran on (a source fact)")
    args, ros_args = ap.parse_known_args(argv)

    class Adapter(Node):
        def __init__(self) -> None:
            super().__init__("real_scan_adapter")
            self.senders = [DatagramSender(p) for p in args.socket]
            self.gen = secrets.randbits(62) + 1  # this start's nonce: a restart is a new producer generation
            self.scan_id = 0
            self.topic, self.stamp_kind = SOURCES[args.source]
            self.stream = SdkStream(args.run_id, {"image_id": args.image_id, "usb": args.usb_id, "host": args.host})
            self.create_subscription(LaserScan, self.topic, self.on_scan, qos_profile_sensor_data)
            if self.stamp_kind == "sdk":
                latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(String, INFO_TOPIC, lambda m: self.stream.on_info(m.data), latched)
            self.create_timer(5.0, self.report)

        def on_scan(self, msg) -> None:
            recv, pair = clock_pair()
            sdk, gen, kind = None, self.gen, self.stamp_kind
            if self.stamp_kind == "sdk":
                stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
                out, _why = self.stream.on_scan(stamp_ns, float(msg.scan_time), recv, pair,
                                                frame_id=str(msg.header.frame_id))
                if out is None:
                    return  # not a scan_sdk message of this run's live start: dropped, counted, never sent
                sdk, gen, kind = out["sdk"], out["gen"], out["stamp_kind"]
            self.scan_id += 1
            # one frame, the same bytes to every receiver: each path keeps its own sent/dropped count
            frame = make_frame(msg, scan_id=self.scan_id, recv_mono_ns=recv, sys_minus_mono_ns=pair, gen=gen,
                               stamp_kind=kind, sdk=sdk)
            for s in self.senders:
                s.send(frame)

        def report(self) -> None:
            parts = [f"{s.path}: sent {s.sent} dropped {s.dropped} last_error {s.last_error}" for s in self.senders]
            self.get_logger().info(f"scans {self.scan_id} gen {self.stream.gen if self.stamp_kind == 'sdk' else self.gen}"
                                   f" dropped {dict(self.stream.dropped)} | " + " | ".join(parts))

    rclpy.init(args=ros_args)
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for s in node.senders:
            s.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
