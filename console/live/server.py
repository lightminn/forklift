"""console/1 server inside the live Isaac runner (M-c plan C3).

The transport is console/wire.py, as for the fake vehicle and the gateway. Behind it there is no vehicle object: a
message goes into the link's channels (forklift_core.live.link) and the runner's control thread answers through
them. The server runs its own asyncio loop on a thread of its own; the runner starts and stops it.

Media come from a store the runner fills (one lock; each kind swapped whole with its sequence number, so a reader
never sees a picture with another frame's id). JPEG is encoded here, once per camera frame.
"""

from __future__ import annotations

import asyncio
import contextlib
import io
import json
import os
import struct
import threading

import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request, Response

from console import wire
from console.wire import CONTROL_PATH, MEDIA_PATH

API = "console/1"
CAPS_HZ = {"camera_rgb": 10.0, "scan": 10.0, "grid": 1.0}


class MediaStore:
    """The latest of each kind with its sequence number, swapped whole under one lock."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._items: dict = {}

    def put(self, kind: str, seq: int, item: dict) -> None:
        with self._lock:
            self._items[kind] = (seq, item)

    def get(self, kind: str):
        with self._lock:
            return self._items.get(kind)


def write_port_file(path: Path, info: dict) -> None:
    """Atomically: a reader never sees half a file (agy A5)."""
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(info))
    os.replace(tmp, path)


def encode_jpeg(rgb, quality: int = 80) -> bytes:
    from PIL import Image

    out = io.BytesIO()
    Image.fromarray(rgb).save(out, format="JPEG", quality=quality)
    return out.getvalue()


class LiveHandle(wire.ControlOutboxMixin, wire.MediaFanoutMixin):
    def __init__(self, channels, store: MediaStore, welcome: dict, web_root: Path | None):
        self.channels, self.store, self.welcome, self.web_root = channels, store, welcome, web_root
        self.server = None
        self.tasks: list[asyncio.Task] = []
        self.control: dict[str, tuple[ServerConnection, str]] = {}
        self.outbox: dict[str, asyncio.Queue] = {}
        self.writers: dict[str, asyncio.Task] = {}
        self.media: dict[str, dict] = {}
        self.media_drops = 0
        self.media_closed = 0
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="live-jpeg")
        self._wake = asyncio.Event()
        self._jpeg: tuple[int, bytes] | None = None  # (frame_id, bytes): one encoding per frame

    def process_request(self, connection: ServerConnection, request: Request) -> Response | None:
        if self.web_root is None:
            return None
        return wire.static_response(connection, request, self.web_root)

    async def handler(self, ws: ServerConnection) -> None:
        path = ws.request.path.split("?", 1)[0]
        if path == CONTROL_PATH:
            await self._control(ws)
        elif path == MEDIA_PATH:
            await self._media(ws)

    async def _control(self, ws: ServerConnection) -> None:
        try:
            first = wire.parse(await asyncio.wait_for(ws.recv(), 5.0))
        except (TimeoutError, ConnectionClosed):
            return
        client_id = first.get("client_id") if first else None
        if first is None or first.get("t") != "hello" or first.get("api") != API or not isinstance(client_id, str) \
                or not client_id:
            await ws.close(1008, "console/1 hello required")
            return
        conn_id = uuid.uuid4().hex
        self.attach_control(conn_id, ws, client_id)
        token = self.channels.issue_token()  # this connection's (persistent console plan D7)
        self.channels.connect(conn_id, client_id, True, token)
        try:
            self._send_control(conn_id, dict(self.welcome, request_token=token))  # a copy: the shared one stays
            for kind in ("targets", "missions"):  # the current list, as the fake vehicle does after welcome, and
                item = self.store.get(kind)  # the session's mission results (persistent console plan D1)
                if item is not None:
                    self._send_control(conn_id, item[1])
            async for raw in ws:
                msg = wire.parse(raw)
                if msg is None:
                    continue
                reply = self.channels.message(conn_id, client_id, msg)
                if reply is not None:
                    self._send_control(conn_id, reply)  # busy: the ordered channel was full
        except ConnectionClosed:
            pass
        finally:
            self.detach_control(conn_id)
            self.channels.disconnect(conn_id, client_id)

    async def _media(self, ws: ServerConnection) -> None:
        conn_id = uuid.uuid4().hex
        sub = {"ws": ws, "rates": {}, "last": {}, "sent": {}}
        self.media[conn_id] = sub
        try:
            async for raw in ws:
                msg = wire.parse(raw)
                if msg is None or msg.get("t") != "media_subscribe":
                    continue
                sub["rates"] = wire.media_rates(msg, CAPS_HZ)
                sub["sent"] = {}  # every subscribe gets the current frames again (contract 10)
        except ConnectionClosed:
            pass
        finally:
            self.media.pop(conn_id, None)

    async def _pump(self) -> None:
        """Messages the control thread put out, to their connection (None: every control connection)."""
        while True:
            await self._wake.wait()
            self._wake.clear()
            for conn_id, msg in self.channels.take_out():
                if conn_id is None:
                    await self._broadcast_control(msg)
                else:
                    self._send_control(conn_id, msg)
                    if msg.get("t") == "reject" and msg.get("of") == "hello":
                        # refused by the request ledger (no room, plan D7): closed once the reject is out
                        ws, _ = self.control.get(conn_id, (None, None))
                        if ws is not None:
                            asyncio.get_running_loop().call_later(
                                0.2, lambda ws_=ws: asyncio.ensure_future(ws_.close(1013, "busy")))

    def wake(self) -> None:
        self._wake.set()

    async def _media_kind(self, kind: str) -> None:
        due = self._due(kind)
        if not due:
            return
        item = self.store.get(kind)
        if item is None:
            return
        seq, content = item
        due = [(c, s) for c, s in due if s["sent"].get(kind) != seq]
        if not due:
            return
        if kind == "camera_rgb":
            if self._jpeg is None or self._jpeg[0] != seq:
                loop = asyncio.get_running_loop()
                self._jpeg = (seq, await loop.run_in_executor(self.executor, encode_jpeg, content["rgb"]))
            jpeg = self._jpeg[1]
            head = dict(content["head"], jpeg_bytes=len(jpeg))
            payload = wire.frame(head, jpeg)
        else:
            payload = wire.frame(content["head"], content["body"])
        for conn_id, sub in due:
            if await self._send_media(conn_id, sub, kind, payload):
                sub["sent"][kind] = seq

    def close(self) -> None:
        for task in self.tasks:
            task.cancel()
        for task in self.writers.values():
            task.cancel()
        if self.server is not None:
            self.server.close()


class LiveServer:
    """The server on its own thread: ``start`` returns once it listens; ``stop`` closes every connection (1001),
    ends the loop and joins the thread (agy A4, T6)."""

    def __init__(self, channels, store: MediaStore, welcome: dict, *, host: str = "127.0.0.1", port: int = 0,
                 web_root: Path | None = None):
        self.channels, self.store, self.welcome = channels, store, welcome
        self.host, self.port, self.web_root = host, port, web_root
        self.loop: asyncio.AbstractEventLoop | None = None
        self.handle: LiveHandle | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._error: BaseException | None = None

    def start(self, timeout_s: float = 10.0) -> int:
        self._thread = threading.Thread(target=self._run, name="live-console", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout_s):
            raise RuntimeError("live console server did not start")
        if self._error is not None:
            raise RuntimeError(f"live console server failed: {self._error!r}")
        return self.port

    def _run(self) -> None:
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._serve())
        except BaseException as exc:  # noqa: BLE001 -- reported to start() or logged by stop()
            self._error = exc
            self._ready.set()
        finally:
            self.loop.close()

    async def _serve(self) -> None:
        h = self.handle = LiveHandle(self.channels, self.store, self.welcome, self.web_root)
        self.channels.notify = lambda: self.loop.call_soon_threadsafe(h.wake)
        h.server = await serve(h.handler, self.host, self.port, process_request=h.process_request,
                               compression=None, close_timeout=1.0, max_size=64 * 1024)
        self.port = h.server.sockets[0].getsockname()[1]
        h.tasks.append(asyncio.create_task(h._pump()))
        for kind, hz in CAPS_HZ.items():
            h.tasks.append(asyncio.create_task(wire.every(hz * 2, lambda k=kind: h._media_kind(k), "live media")))
        self._stopping = asyncio.Event()
        self._ready.set()
        await self._stopping.wait()
        sockets = [ws for ws, _ in list(h.control.values())] + [sub["ws"] for sub in list(h.media.values())]

        async def close(ws):
            with contextlib.suppress(Exception):
                await ws.close(1001, "runner ended")

        # Every connection at once, under one budget (Codex M-c review round 3 P2: one by one, 20 slow
        # closes outlived the join); whatever is left is aborted.
        if sockets:
            _, late = await asyncio.wait([asyncio.create_task(close(ws)) for ws in sockets], timeout=1.5)
            for task in late:
                task.cancel()
            for ws in sockets:
                if ws.transport is not None and not ws.transport.is_closing():
                    ws.transport.abort()
        h.close()
        with contextlib.suppress(Exception):
            await asyncio.wait_for(h.server.wait_closed(), 2.0)
        h.executor.shutdown(wait=False, cancel_futures=True)

    def stop(self, timeout_s: float = 5.0) -> bool:
        """True once the server thread has ended; False when it outlived ``timeout_s`` (the caller must not report
        the server ended)."""
        if self.loop is not None and self._thread is not None and self._thread.is_alive():
            self.loop.call_soon_threadsafe(self._stopping.set)
            self._thread.join(timeout_s)
        return self._thread is None or not self._thread.is_alive()


def camera_item(frame_id: int, rgb, acquired_mono: float, stamp_sim_s: float, session_epoch: int, *,
                calibration_version: int = 0) -> dict:
    h, w = rgb.shape[:2]
    return {"rgb": rgb, "head": {"t": "camera", "session_epoch": session_epoch, "frame_id": frame_id,
                                 "acquired_mono_ns": int(acquired_mono * 1e9),
                                 "capture": {"domain": "sim", "stamp_ns": int(stamp_sim_s * 1e9)},
                                 "calibration_version": calibration_version, "format": "jpeg",
                                 "width": w, "height": h}}


def scan_item(ranges, angle_min_rad: float, angle_increment_rad: float, range_max_m: float, pose: dict,
              session_epoch: int, frame: str, age_s: float = 0.0) -> dict:
    body = struct.pack(f"<{len(ranges)}f", *[float(r) for r in ranges])
    return {"head": {"t": "scan", "session_epoch": session_epoch, "frame": frame, "pose": pose,
                     "pose_at": "scan_stamp", "angle_min_rad": angle_min_rad,
                     "angle_increment_rad": angle_increment_rad, "range_max_m": range_max_m, "dtype": "float32le",
                     "count": len(ranges), "age_s": age_s}, "body": body}


def grid_item(cells: bytes, width: int, height: int, resolution_m: float, origin_m, session_epoch: int,
              frame: str, correction_version: int) -> dict:
    return {"head": {"t": "grid", "session_epoch": session_epoch, "frame": frame, "resolution_m": resolution_m,
                     "origin_m": [float(origin_m[0]), float(origin_m[1])], "width": width, "height": height,
                     "values": {"unknown": 0, "free": 1, "occupied": 2},
                     "correction_version": correction_version}, "body": bytes(cells)}


__all__ = ["LiveServer", "MediaStore", "camera_item", "encode_jpeg", "grid_item", "scan_item", "write_port_file"]

