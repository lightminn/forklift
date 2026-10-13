"""WebSocket server for the fake vehicle (console/1 draft).

Two connections per client: /api/v1/control (JSON) and /api/v1/media (binary frames:
uint32 big-endian header length + UTF-8 JSON header + body). Static web files are served
from web_root with module-friendly MIME types. Development aid only, no authentication.
"""

from __future__ import annotations

import asyncio
import contextlib
import http
import json
import math
import os
import secrets
import struct
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from websockets.asyncio.server import ServerConnection, serve
from websockets.datastructures import Headers
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request, Response

from console.mock_vehicle import camera
from console.mock_vehicle.core import API, MockVehicle
from console.mock_vehicle.world import raycast

CONTROL_PATH = "/api/v1/control"
MEDIA_PATH = "/api/v1/media"
MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
}
CONTROL_SEND_TIMEOUT_S = 0.2
CONTROL_OUTBOX_MAX = 256
MAX_JSON_DEPTH = 8
MEDIA_SEND_TIMEOUT_S = 1.0
MEDIA_SKIP_BYTES = 512 * 1024
MEDIA_CLOSE_BYTES = 4 * 1024 * 1024
STEP_HZ = 120.0
STATE_HZ = 10.0
CAMERA_HZ = 15.0
SCAN_HZ = 10.0
GRID_HZ = 1.0
SCAN_RAYS = 180
SCAN_RANGE_M = 12.0
GRID_RES_M = 0.05


def _reject_constant(name: str):
    raise ValueError(f"non-finite number {name}")


def _finite_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value):  # 1e400 parses as inf without this check
        raise ValueError(f"non-finite number {text}")
    return value


def _parse(text) -> dict | None:
    if not isinstance(text, str):
        return None
    try:
        msg = json.loads(
            text, parse_constant=_reject_constant, parse_float=_finite_float
        )
    except (ValueError, RecursionError):
        return None
    if not isinstance(msg, dict) or _depth(msg) > MAX_JSON_DEPTH:
        return None
    return msg


def _depth(value) -> int:
    """Nesting depth without recursion (console/1 messages are shallow)."""
    deepest, stack = 0, [(value, 1)]
    while stack:
        node, d = stack.pop()
        deepest = max(deepest, d)
        if d > MAX_JSON_DEPTH:
            return d
        if isinstance(node, dict):
            children = node.values()
        elif isinstance(node, list):
            children = node
        else:
            children = ()
        stack.extend((c, d + 1) for c in children if isinstance(c, (dict, list)))
    return deepest


def _encode(msg: dict) -> str | None:
    """Standard JSON only (no NaN/Infinity), or None if the message cannot be sent."""
    try:
        return json.dumps(msg, allow_nan=False)
    except (ValueError, RecursionError):
        return None


def _media_rates(msg: dict) -> dict[str, float]:
    """Subscribed rates per kind, capped to the vehicle's; bad values drop that kind."""
    caps = {"camera_rgb": CAMERA_HZ, "scan": SCAN_HZ, "grid": GRID_HZ}
    rates = {}
    for kind, cap in caps.items():
        want = msg.get(kind)
        if not isinstance(want, dict):
            continue
        hz = want.get("hz", cap)
        if isinstance(hz, bool) or not isinstance(hz, (int, float)):
            continue
        if isinstance(hz, int) and not 0 < hz <= 1000:
            continue  # refuses huge ints before any float conversion
        if isinstance(hz, float) and not (math.isfinite(hz) and 0 < hz <= 1000):
            continue
        rates[kind] = min(float(hz), cap)
    return rates


def _frame(head: dict, body: bytes) -> bytes:
    raw = json.dumps(head, separators=(",", ":")).encode("utf-8")
    return struct.pack(">I", len(raw)) + raw + body


class Handle:
    """Running fake-vehicle server; close() cancels the server and every loop."""

    def __init__(self, vehicle: MockVehicle, web_root: Path, pad_bytes: int):
        self.vehicle = vehicle
        self.web_root = web_root
        self.pad_bytes = pad_bytes
        self.server = None
        self.tasks: list[asyncio.Task] = []
        self.control: dict[str, tuple[ServerConnection, str]] = {}
        self.outbox: dict[str, asyncio.Queue] = {}
        self.writers: dict[str, asyncio.Task] = {}
        self.media: dict[str, dict] = {}
        self.media_drops = 0
        self.media_closed = 0
        self.frame_id = 0
        self.executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="mock-camera"
        )
        self._targets_revision = None

    # ------------------------------------------------------------- lifecycle
    def close(self) -> None:
        for task in self.tasks:
            task.cancel()
        for task in self.writers.values():
            task.cancel()
        for sub in self.media.values():
            for pending in sub.get("pending", {}).values():
                pending.cancel()
        if self.server is not None:
            self.server.close()

    async def wait_closed(self) -> None:
        with contextlib.suppress(asyncio.CancelledError):
            await asyncio.gather(*self.tasks, return_exceptions=True)
        if self.server is not None:
            try:
                await asyncio.wait_for(self.server.wait_closed(), 1.5)
            except TimeoutError:
                for ws, _ in list(self.control.values()):
                    ws.transport.abort()
                for sub in list(self.media.values()):
                    sub["ws"].transport.abort()
                await self.server.wait_closed()
        self.executor.shutdown(wait=False, cancel_futures=True)

    # ---------------------------------------------------------------- static
    def process_request(
        self, connection: ServerConnection, request: Request
    ) -> Response | None:
        path = request.path.split("?", 1)[0]
        if path in (CONTROL_PATH, MEDIA_PATH):
            return None
        if path in ("/", "/index.html"):
            rel = "index.html"
        elif path == "/app.css" or (
            path.startswith("/js/") and path.endswith(".js") and ".." not in path
        ):
            rel = path.lstrip("/")
        else:
            return connection.respond(http.HTTPStatus.NOT_FOUND, "not found\n")
        target = self.web_root / rel
        if not target.is_file():
            return connection.respond(http.HTTPStatus.NOT_FOUND, "not found\n")
        body = target.read_bytes()
        headers = Headers(
            [
                ("Content-Type", MIME[target.suffix]),
                ("Content-Length", str(len(body))),
                ("Cache-Control", "no-store"),
            ]
        )
        return Response(200, "OK", headers, body)

    # ------------------------------------------------------------ connection
    async def handler(self, ws: ServerConnection) -> None:
        path = ws.request.path.split("?", 1)[0]
        if path == CONTROL_PATH:
            await self._control(ws)
        elif path == MEDIA_PATH:
            await self._media(ws)

    def attach_control(self, conn_id: str, ws, client_id: str) -> None:
        """Register a control connection with its own outbox and writer.

        Every message to a connection goes through its queue, so one slow reader never
        delays ticks or replies to the others; its writer aborts it on a stuck send.
        """
        self.control[conn_id] = (ws, client_id)
        self.outbox[conn_id] = asyncio.Queue(maxsize=CONTROL_OUTBOX_MAX)
        self.writers[conn_id] = asyncio.create_task(self._writer(conn_id, ws))

    def detach_control(self, conn_id: str) -> None:
        self.control.pop(conn_id, None)
        self.outbox.pop(conn_id, None)
        task = self.writers.pop(conn_id, None)
        if task is not None:
            task.cancel()

    async def _writer(self, conn_id: str, ws) -> None:
        queue = self.outbox[conn_id]
        while True:
            text = await queue.get()
            try:
                await asyncio.wait_for(ws.send(text), CONTROL_SEND_TIMEOUT_S)
            except (TimeoutError, ConnectionClosed):
                ws.transport.abort()  # its teleop expires; other clients are unaffected
                return

    def _send_control(self, conn_id: str, msg: dict) -> None:
        queue = self.outbox.get(conn_id)
        text = _encode(msg)
        if queue is None or text is None:
            return
        try:
            queue.put_nowait(text)
        except asyncio.QueueFull:
            ws, _ = self.control.get(conn_id, (None, None))
            if ws is not None:
                ws.transport.abort()

    async def _control(self, ws: ServerConnection) -> None:
        try:
            first = _parse(await asyncio.wait_for(ws.recv(), 5.0))
        except (TimeoutError, ConnectionClosed):
            return
        client_id = first.get("client_id") if first else None
        if (
            first is None
            or first.get("t") != "hello"
            or first.get("api") != API
            or not isinstance(client_id, str)
            or not client_id
        ):
            await ws.close(1008, "console/1 hello required")
            return
        conn_id = uuid.uuid4().hex
        welcome = self.vehicle.connect(client_id, conn_id)
        self.attach_control(conn_id, ws, client_id)
        try:
            self._send_control(conn_id, welcome)
            self._send_control(conn_id, self.vehicle.make_targets())
            async for raw in ws:
                msg = _parse(raw)
                if msg is None:
                    continue
                reply = self.vehicle.handle(client_id, msg, conn_id=conn_id)
                if reply is not None:
                    self._send_control(conn_id, reply)
        except ConnectionClosed:
            pass
        finally:
            self.detach_control(conn_id)
            self.vehicle.disconnect(client_id, conn_id)

    async def _media(self, ws: ServerConnection) -> None:
        conn_id = uuid.uuid4().hex
        sub = {"ws": ws, "rates": {}, "last": {}}
        self.media[conn_id] = sub
        try:
            async for raw in ws:
                msg = _parse(raw)
                if msg is None or msg.get("t") != "media_subscribe":
                    continue
                rates = _media_rates(msg)
                sub["rates"] = rates
        except ConnectionClosed:
            pass
        finally:
            self.media.pop(conn_id, None)

    async def _send_media(
        self, conn_id: str, sub: dict, kind: str, payload: bytes
    ) -> None:
        """Latest-only: one send in flight per subscriber; a frame due while it is busy is dropped.

        websockets' send() waits for the socket to drain, so a reader that stopped would
        otherwise hold frames back instead of filling the transport buffer.
        """
        ws = sub["ws"]
        pending = sub.setdefault("pending", {}).get(kind)  # one in flight per kind
        if pending is not None and not pending.done():
            if time.monotonic() - sub["pending_since"][kind] > MEDIA_SEND_TIMEOUT_S:
                self._close_media(conn_id, ws)
            else:
                self.media_drops += 1
            return
        buffered = ws.transport.get_write_buffer_size() if ws.transport else 0
        if buffered > MEDIA_CLOSE_BYTES:
            self._close_media(conn_id, ws)
            return
        if buffered > MEDIA_SKIP_BYTES:
            self.media_drops += 1
            return
        sub["last"][kind] = time.monotonic()
        sub.setdefault("pending_since", {})[kind] = time.monotonic()
        sub["pending"][kind] = asyncio.create_task(
            self._media_send_task(conn_id, ws, payload)
        )

    async def _media_send_task(
        self, conn_id: str, ws: ServerConnection, payload: bytes
    ) -> None:
        try:
            await asyncio.wait_for(ws.send(payload), MEDIA_SEND_TIMEOUT_S)
        except (TimeoutError, ConnectionClosed):
            self._close_media(conn_id, ws)

    def _close_media(self, conn_id: str, ws: ServerConnection) -> None:
        if self.media.pop(conn_id, None) is not None:
            self.media_closed += 1
        if ws.transport is not None:
            ws.transport.abort()

    def _due(self, kind: str) -> list[tuple[str, dict]]:
        now = time.monotonic()
        out = []
        for conn_id, sub in list(self.media.items()):
            hz = sub["rates"].get(kind)
            if hz and now - sub["last"].get(kind, 0.0) >= 1.0 / hz - 1e-3:
                out.append((conn_id, sub))
        return out

    # ----------------------------------------------------------------- loops
    async def _every(self, hz: float, fn) -> None:
        period = 1.0 / hz
        nxt = time.monotonic()
        while True:
            try:
                await fn()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # one bad step must not end a shared loop
                print(f"mock server: {fn.__name__} failed: {exc!r}", flush=True)
            nxt += period
            delay = nxt - time.monotonic()
            if delay < -period:  # fell behind: skip, never burst
                nxt = time.monotonic()
                delay = 0.0
            await asyncio.sleep(max(0.0, delay))

    async def _step(self) -> None:
        now = time.monotonic()
        dt = min(0.05, now - getattr(self, "_last_step", now)) or 1.0 / STEP_HZ
        self._last_step = now
        self.vehicle.step(dt)

    async def _broadcast_control(self, msg: dict) -> None:
        for conn_id in list(self.control):
            self._send_control(conn_id, msg)

    async def _tick(self) -> None:
        if self.control:
            await self._broadcast_control(self.vehicle.make_tick())

    async def _state(self) -> None:
        if not self.control:
            return
        await self._broadcast_control(self.vehicle.make_state())
        events, self.vehicle.events[:] = list(self.vehicle.events), []
        for ev in events:
            await self._broadcast_control(ev)
        if self.vehicle.targets_revision != self._targets_revision:
            self._targets_revision = self.vehicle.targets_revision
            await self._broadcast_control(self.vehicle.make_targets())

    async def _camera(self) -> None:
        due = self._due("camera_rgb")
        if not due:
            return
        self.frame_id += 1
        frame_id = self.frame_id
        self.vehicle.record_frame(frame_id)
        acquired = time.monotonic()
        v = self.vehicle
        loop = asyncio.get_running_loop()
        jpeg = await loop.run_in_executor(
            self.executor, camera.render_jpeg, v.world, v.pose, v.lift_m, frame_id
        )
        body = jpeg + (os.urandom(self.pad_bytes) if self.pad_bytes else b"")
        head = {
            "t": "camera",
            "session_epoch": v.session_epoch,
            "frame_id": frame_id,
            "acquired_mono_ns": int(acquired * 1e9),
            "capture": {"domain": "sim", "stamp_ns": int(acquired * 1e9)},
            "calibration_version": camera.CAMERA_MODEL["calibration_version"],
            "format": "jpeg",
            "width": camera.CAMERA_MODEL["width"],
            "height": camera.CAMERA_MODEL["height"],
            "jpeg_bytes": len(jpeg),
        }
        payload = _frame(head, body)
        await asyncio.gather(
            *(self._send_media(c, s, "camera_rgb", payload) for c, s in due)
        )

    async def _scan(self) -> None:
        due = self._due("scan")
        if not due:
            return
        v = self.vehicle
        p = v.pose
        inc = 2 * math.pi / SCAN_RAYS
        ranges = []
        for i in range(SCAN_RAYS):
            a = p.yaw_rad - math.pi + i * inc
            d = raycast(v.world, p.x_m, p.y_m, math.cos(a), math.sin(a), SCAN_RANGE_M)
            ranges.append(d if d < SCAN_RANGE_M - 1e-6 else float("nan"))
        head = {
            "t": "scan",
            "session_epoch": v.session_epoch,
            "frame": "base_link",
            "pose": {"x_m": p.x_m, "y_m": p.y_m, "yaw_rad": p.yaw_rad},
            "angle_min_rad": -math.pi,
            "angle_increment_rad": inc,
            "range_max_m": SCAN_RANGE_M,
            "dtype": "float32le",
            "count": SCAN_RAYS,
        }
        payload = _frame(head, struct.pack(f"<{SCAN_RAYS}f", *ranges))
        await asyncio.gather(*(self._send_media(c, s, "scan", payload) for c, s in due))

    async def _grid(self) -> None:
        due = self._due("grid")
        if not due:
            return
        v = self.vehicle
        w_m, h_m = v.world.size_m
        nx, ny = round(w_m / GRID_RES_M), round(h_m / GRID_RES_M)
        cells = bytearray([1]) * (nx * ny)  # 0 unknown, 1 free, 2 occupied

        def mark(x0, y0, x1, y1, inside):
            for j in range(
                max(0, int(y0 / GRID_RES_M)), min(ny, int(y1 / GRID_RES_M) + 1)
            ):
                for i in range(
                    max(0, int(x0 / GRID_RES_M)), min(nx, int(x1 / GRID_RES_M) + 1)
                ):
                    if inside((i + 0.5) * GRID_RES_M, (j + 0.5) * GRID_RES_M):
                        cells[j * nx + i] = 2

        for cx, cy, r in v.world.circles:
            mark(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                lambda x, y, cx=cx, cy=cy, r=r: (x - cx) ** 2 + (y - cy) ** 2 <= r * r,
            )
        for cx, cy, bw, bh in v.world.boxes:
            mark(cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2, lambda x, y: True)
        for pal in v.world.pallets:
            if pal.state == "carried":
                continue
            c, s = math.cos(pal.pose.yaw_rad), math.sin(pal.pose.yaw_rad)
            rad = math.hypot(pal.depth_m, pal.width_m) / 2

            def inside(x, y, pal=pal, c=c, s=s):
                lx = (x - pal.pose.x_m) * c + (y - pal.pose.y_m) * s
                ly = -(x - pal.pose.x_m) * s + (y - pal.pose.y_m) * c
                return abs(lx) <= pal.depth_m / 2 and abs(ly) <= pal.width_m / 2

            mark(
                pal.pose.x_m - rad,
                pal.pose.y_m - rad,
                pal.pose.x_m + rad,
                pal.pose.y_m + rad,
                inside,
            )
        head = {
            "t": "grid",
            "session_epoch": v.session_epoch,
            "frame": "map",
            "resolution_m": GRID_RES_M,
            "origin_m": [0.0, 0.0],
            "width": nx,
            "height": ny,
            "values": {"unknown": 0, "free": 1, "occupied": 2},
            "correction_version": 1,
        }
        payload = _frame(head, bytes(cells))
        await asyncio.gather(*(self._send_media(c, s, "grid", payload) for c, s in due))


async def start(
    host: str, port: int, web_root: Path, *, test_frame_pad_bytes: int = 0
) -> Handle:
    vehicle = MockVehicle(time.monotonic, lambda: secrets.token_hex(8))
    h = Handle(vehicle, Path(web_root), test_frame_pad_bytes)
    h.server = await serve(
        h.handler,
        host,
        port,
        process_request=h.process_request,
        compression=None,
        close_timeout=1.0,
        max_size=64 * 1024,
    )
    for hz, fn in (
        (STEP_HZ, h._step),
        (h.vehicle.cfg.tick_hz, h._tick),
        (STATE_HZ, h._state),
        (CAMERA_HZ, h._camera),
        (SCAN_HZ, h._scan),
        (GRID_HZ, h._grid),
    ):
        h.tasks.append(asyncio.create_task(h._every(hz, fn)))
    return h


async def main(host: str, port: int, web_root: Path) -> None:
    h = await start(host, port, web_root)
    print(
        f"fake vehicle: http://{host}:{port}/  (session_epoch {h.vehicle.session_epoch})",
        flush=True,
    )
    try:
        await asyncio.Future()
    finally:
        h.close()
        await h.wait_closed()
