"""console/1 monitor-only server for the gateway (design C2a).

Same transport as the fake vehicle (console/wire.py). Control connection: hello ->
welcome with capabilities.control false, then state at STATE_HZ; every request that
carries a request_id is answered reject(unsupported) -- STOP included, because nothing
behind this server can stop the truck (design D1). Media connection: grid, scan and
camera frames from the store, latest only, each frame once per subscriber.
"""

from __future__ import annotations

import asyncio
import contextlib
import time
import uuid
from pathlib import Path

from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request, Response

from console import wire
from console.gateway import convert
from console.gateway.store import Store
from console.mock_vehicle.core import API

STATE_HZ = 5.0
CAMERA_HZ = 15.0
SCAN_HZ = 10.0
GRID_HZ = 1.0
MEDIA_LOOP_HZ = 30.0


class Gateway(wire.ControlOutboxMixin, wire.MediaFanoutMixin):
    def __init__(
        self,
        store: Store,
        web_root: Path,
        vehicle_id: str,
        backend: str,
        epoch: int,
        rear_axle_x_m: float,
        clock=time.monotonic,
        capture_domain: str = "sensor",
        stop_instruction: str | None = None,
    ):
        self.store = store
        self.web_root = web_root
        self.clock = clock
        self.epoch = epoch
        self.rear_axle_x_m = rear_axle_x_m
        self.capture_domain = capture_domain  # "sim" when the stamps are sim time
        self.welcome_msg = convert.welcome(
            vehicle_id, backend, epoch, stop_instruction=stop_instruction
        )
        self.state_seq = 0
        self.server = None
        self.tasks: list[asyncio.Task] = []
        self.control: dict[str, tuple[ServerConnection, str]] = {}
        self.outbox: dict[str, asyncio.Queue] = {}
        self.writers: dict[str, asyncio.Task] = {}
        self.media: dict[str, dict] = {}
        self.media_drops = 0
        self.media_closed = 0
        self.frame_id = 0
        self._camera_seq_sent = 0

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

    def process_request(
        self, connection: ServerConnection, request: Request
    ) -> Response | None:
        return wire.static_response(connection, request, self.web_root)

    async def handler(self, ws: ServerConnection) -> None:
        path = ws.request.path.split("?", 1)[0]
        if path == wire.CONTROL_PATH:
            await self._control(ws)
        elif path == wire.MEDIA_PATH:
            await self._media(ws)

    # --------------------------------------------------------------- control
    async def _control(self, ws: ServerConnection) -> None:
        try:
            first = wire.parse(await asyncio.wait_for(ws.recv(), 5.0))
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
        self.attach_control(conn_id, ws, client_id)
        try:
            self._send_control(conn_id, self.welcome_msg)
            self._send_control(conn_id, self.make_state())
            async for raw in ws:
                msg = wire.parse(raw)
                if msg is None:
                    continue
                reply = self.reply(msg)
                if reply is not None:
                    self._send_control(conn_id, reply)
        except ConnectionClosed:
            pass
        finally:
            self.detach_control(conn_id)

    @staticmethod
    def reply(msg: dict) -> dict | None:
        """Every request is refused: this connection has no control path (design D1)."""
        rid = msg.get("request_id")
        if not isinstance(rid, str):
            return None  # echo, input_state, teleop ticks: nothing to answer
        return {"t": "reject", "request_id": rid, "reason": "unsupported"}

    def make_state(self) -> dict:
        self.state_seq += 1
        now = self.clock()
        return convert.state(
            self.store.snapshot(now),
            now=now,
            epoch=self.epoch,
            seq=self.state_seq,
            rear_axle_x_m=self.rear_axle_x_m,
        )

    async def _state(self) -> None:
        if self.control:
            await self._broadcast_control(self.make_state())

    # ----------------------------------------------------------------- media
    async def _media(self, ws: ServerConnection) -> None:
        conn_id = uuid.uuid4().hex
        sub = {"ws": ws, "rates": {}, "last": {}, "sent_seq": {}}
        self.media[conn_id] = sub
        try:
            async for raw in ws:
                msg = wire.parse(raw)
                if msg is None or msg.get("t") != "media_subscribe":
                    continue
                sub["rates"] = wire.media_rates(
                    msg, {"camera_rgb": CAMERA_HZ, "scan": SCAN_HZ, "grid": GRID_HZ}
                )
                # a (re)subscribe gets the current frames again: the page may have
                # dropped earlier ones (e.g. before it knew the session epoch)
                sub["sent_seq"] = {}
        except ConnectionClosed:
            pass
        finally:
            self.media.pop(conn_id, None)

    def _payload(self, kind: str, value):
        if kind == "grid":
            try:
                head, body = convert.grid_frame(
                    value,
                    self.epoch,
                    self.store.snapshot(self.clock()).correction_version,
                )
            except convert.GridRotated as exc:
                self.store.note("grid", f"지도 원점이 회전되어 표시하지 않음 ({exc})")
                return wire.frame(*convert.withdrawn_grid(self.epoch, "rotated"))
            except ValueError as exc:
                self.store.note("grid", f"지도 형식 오류 ({exc})")
                return wire.frame(*convert.withdrawn_grid(self.epoch, "invalid"))
            self.store.note("grid", None)
            return wire.frame(head, body)
        if kind == "scan":
            scan, laser, pose_at, delay_s = value
            age = self.clock() - scan.received_s
            out = convert.scan_frame(scan, laser, self.epoch, pose_at, delay_s, age)
            return None if out is None else wire.frame(*out)
        # camera: JPEG passed through; one frame_id per capture (the store drops re-sends)
        head = convert.camera_head(
            self.epoch,
            self.frame_id,
            value.stamp_ns,
            value.received_s,
            len(value.jpeg),
            self.capture_domain,
        )
        return wire.frame(head, value.jpeg)

    async def _media_kind(self, kind: str) -> None:
        due = self._due(kind)
        if not due:
            return
        seq, value = self.store.latest(kind)
        if value is None:
            return
        if kind == "camera_rgb" and seq != self._camera_seq_sent:
            self._camera_seq_sent = seq
            self.frame_id += 1  # a new image gets a new id, the same image keeps it
        fresh = [(c, s) for c, s in due if s["sent_seq"].get(kind) != seq]
        if not fresh:
            return
        payload = self._payload(kind, value)
        if payload is None:
            return
        sent = await asyncio.gather(
            *(self._send_media(c, s, kind, payload) for c, s in fresh)
        )
        for (_, s), ok in zip(fresh, sent, strict=True):
            if ok:  # a frame dropped on a busy socket is tried again on the next tick
                s["sent_seq"][kind] = seq

    async def _media_tick(self) -> None:
        for kind in ("grid", "scan", "camera_rgb"):
            await self._media_kind(kind)


async def start(
    store: Store,
    host: str,
    port: int,
    web_root: Path,
    vehicle_id: str,
    backend: str,
    epoch: int,
    rear_axle_x_m: float,
    capture_domain: str = "sensor",
    stop_instruction: str | None = None,
) -> Gateway:
    gw = Gateway(
        store,
        web_root,
        vehicle_id,
        backend,
        epoch,
        rear_axle_x_m,
        capture_domain=capture_domain,
        stop_instruction=stop_instruction,
    )
    gw.server = await serve(
        gw.handler,
        host,
        port,
        process_request=gw.process_request,
        compression=None,
        close_timeout=1.0,
        max_size=64 * 1024,
    )
    gw.tasks = [
        asyncio.create_task(wire.every(STATE_HZ, gw._state, "gateway")),
        asyncio.create_task(wire.every(MEDIA_LOOP_HZ, gw._media_tick, "gateway")),
    ]
    return gw
