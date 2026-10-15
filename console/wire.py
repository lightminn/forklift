"""console/1 transport shared by the fake vehicle and the gateway.

Two connections per client: /api/v1/control (JSON) and /api/v1/media (binary frames:
uint32 big-endian header length + UTF-8 JSON header + body). Static web files are served
from a web root with module-friendly MIME types. No authentication (design 7.4).

The rules here came out of the C1 reviews: strict JSON (no NaN/Infinity, no 1e400, no
deep nesting), every control connection with its own bounded outbox and writer so one
slow reader never delays the others, and latest-only media with one send in flight per
kind and subscriber.
"""

from __future__ import annotations

import asyncio
import http
import json
import math
import struct
import time
from pathlib import Path

from websockets.asyncio.server import ServerConnection
from websockets.datastructures import Headers
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request, Response

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


def _reject_constant(name: str):
    raise ValueError(f"non-finite number {name}")


def _finite_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value):  # 1e400 parses as inf without this check
        raise ValueError(f"non-finite number {text}")
    return value


def parse(text) -> dict | None:
    if not isinstance(text, str):
        return None
    try:
        msg = json.loads(
            text, parse_constant=_reject_constant, parse_float=_finite_float
        )
    except (ValueError, RecursionError):
        return None
    if not isinstance(msg, dict) or depth(msg) > MAX_JSON_DEPTH:
        return None
    return msg


def depth(value) -> int:
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


def encode(msg: dict) -> str | None:
    """Standard JSON only (no NaN/Infinity), or None if the message cannot be sent."""
    try:
        return json.dumps(msg, allow_nan=False)
    except (ValueError, RecursionError):
        return None


def media_rates(msg: dict, caps: dict[str, float]) -> dict[str, float]:
    """Subscribed rates per kind, capped to the vehicle's; bad values drop that kind."""
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


def frame(head: dict, body: bytes) -> bytes:
    raw = json.dumps(head, separators=(",", ":")).encode("utf-8")
    return struct.pack(">I", len(raw)) + raw + body


def static_response(
    connection: ServerConnection, request: Request, web_root: Path
) -> Response | None:
    """index.html, app.css and js/*.js from web_root; None lets the WebSocket through."""
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
    target = web_root / rel
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


class ControlOutboxMixin:
    """Per-connection control outboxes. Needs self.control, self.outbox, self.writers."""

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
        text = encode(msg)
        if queue is None or text is None:
            return
        try:
            queue.put_nowait(text)
        except asyncio.QueueFull:
            ws, _ = self.control.get(conn_id, (None, None))
            if ws is not None:
                ws.transport.abort()

    async def _broadcast_control(self, msg: dict) -> None:
        for conn_id in list(self.control):
            self._send_control(conn_id, msg)


class MediaFanoutMixin:
    """Latest-only media fan-out. Needs self.media, self.media_drops, self.media_closed."""

    async def _send_media(
        self, conn_id: str, sub: dict, kind: str, payload: bytes
    ) -> bool:
        """Latest-only: one send in flight per subscriber; a frame due while it is busy is dropped.

        True when the frame went out (a send started); False when it was dropped or the
        connection closed, so a caller that sends each frame once can try again.

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
            return False
        buffered = ws.transport.get_write_buffer_size() if ws.transport else 0
        if buffered > MEDIA_CLOSE_BYTES:
            self._close_media(conn_id, ws)
            return False
        if buffered > MEDIA_SKIP_BYTES:
            self.media_drops += 1
            return False
        sub["last"][kind] = time.monotonic()
        sub.setdefault("pending_since", {})[kind] = time.monotonic()
        sub["pending"][kind] = asyncio.create_task(
            self._media_send_task(conn_id, ws, payload)
        )
        return True

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


async def every(hz: float, fn, label: str) -> None:
    """Run fn at hz; one failing step never ends the loop, and a late loop skips, never bursts."""
    period = 1.0 / hz
    nxt = time.monotonic()
    while True:
        try:
            await fn()
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(f"{label}: {fn.__name__} failed: {exc!r}", flush=True)
        nxt += period
        delay = nxt - time.monotonic()
        if delay < -period:
            nxt = time.monotonic()
            delay = 0.0
        await asyncio.sleep(max(0.0, delay))
