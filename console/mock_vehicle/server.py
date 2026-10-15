"""WebSocket server for the fake vehicle (console/1 draft).

The transport (connections, framing, static files) is console/wire.py, shared with the
gateway. Development aid only, no authentication.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import os
import secrets
import struct
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request, Response

from console import wire
from console.mock_vehicle import camera
from console.mock_vehicle.core import API, MockVehicle
from console.mock_vehicle.world import raycast
from console.wire import CONTROL_PATH, MEDIA_PATH

STEP_HZ = 120.0
STATE_HZ = 10.0
CAMERA_HZ = 15.0
SCAN_HZ = 10.0
GRID_HZ = 1.0
SCAN_RAYS = 180
SCAN_RANGE_M = 12.0
GRID_RES_M = 0.05

# the shared transport rules (console/wire.py), under the names the tests use
_parse = wire.parse
_encode = wire.encode
_frame = wire.frame


def _media_rates(msg: dict) -> dict[str, float]:
    return wire.media_rates(
        msg, {"camera_rgb": CAMERA_HZ, "scan": SCAN_HZ, "grid": GRID_HZ}
    )


class Handle(wire.ControlOutboxMixin, wire.MediaFanoutMixin):
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
        return wire.static_response(connection, request, self.web_root)

    # ------------------------------------------------------------ connection
    async def handler(self, ws: ServerConnection) -> None:
        path = ws.request.path.split("?", 1)[0]
        if path == CONTROL_PATH:
            await self._control(ws)
        elif path == MEDIA_PATH:
            await self._media(ws)

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

    # ----------------------------------------------------------------- loops
    async def _every(self, hz: float, fn) -> None:
        await wire.every(hz, fn, "mock server")

    async def _step(self) -> None:
        now = time.monotonic()
        dt = min(0.05, now - getattr(self, "_last_step", now)) or 1.0 / STEP_HZ
        self._last_step = now
        self.vehicle.step(dt)

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
