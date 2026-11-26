"""The operating REAL run's console (operating plan 2026-11-26 6.1, Codex U3e-1 round 2 P1-2): a WebSocket whose
Origin is not one of the run's own pages (missing included) is refused before the handshake; HIL keeps no check."""

import asyncio
import json

import pytest

pytest.importorskip("websockets")

from websockets.asyncio.client import connect  # noqa: E402
from websockets.exceptions import InvalidStatus  # noqa: E402

from console.live.server import API, LiveServer, MediaStore  # noqa: E402
from console.wire import CONTROL_PATH  # noqa: E402
from forklift_core.live.link import Channels  # noqa: E402


async def _hello(port, origin):
    headers = {} if origin is None else {"Origin": origin}
    async with connect(f"ws://127.0.0.1:{port}{CONTROL_PATH}", additional_headers=headers, origin=None) as ws:
        await ws.send(json.dumps({"t": "hello", "api": API, "client_id": "c1"}))
        return True


def _run(server, origin):
    try:
        return asyncio.run(_hello(server.port, origin))
    except InvalidStatus as err:
        return err.response.status_code


@pytest.fixture
def real_server():
    s = LiveServer(Channels(), MediaStore(), {"t": "welcome"}, origin_hosts=("127.0.0.1", "localhost"))
    s.start(timeout_s=5)
    yield s
    s.stop()


def test_the_runs_own_page_is_accepted_others_refused(real_server):
    port = real_server.port
    assert _run(real_server, f"http://127.0.0.1:{port}") is True
    assert _run(real_server, f"http://localhost:{port}") is True
    assert _run(real_server, None) == 403  # no Origin at all
    assert _run(real_server, "http://evil.example") == 403
    assert _run(real_server, f"http://127.0.0.1:{port + 1}") == 403  # another port is another page


def test_a_lan_host_named_by_the_operator_is_accepted():
    s = LiveServer(Channels(), MediaStore(), {"t": "welcome"}, origin_hosts=("127.0.0.1", "192.168.4.1"))
    s.start(timeout_s=5)
    try:
        assert _run(s, f"http://192.168.4.1:{s.port}") is True
        assert _run(s, f"http://192.168.4.2:{s.port}") == 403
    finally:
        s.stop()


def test_hil_keeps_no_origin_check():
    s = LiveServer(Channels(), MediaStore(), {"t": "welcome"})
    s.start(timeout_s=5)
    try:
        assert _run(s, None) is True and _run(s, "http://elsewhere") is True
    finally:
        s.stop()
