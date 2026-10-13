import asyncio
import json
import time

import pytest

pytest.importorskip("websockets")
pytest.importorskip("PIL")

from console.mock_vehicle import server  # noqa: E402


class FakeTransport:
    def __init__(self):
        self.aborted = False

    def abort(self):
        self.aborted = True

    def get_write_buffer_size(self):
        return 0


class FakeWS:
    def __init__(self, delay_s=0.0):
        self.delay_s = delay_s
        self.sent = []
        self.transport = FakeTransport()

    async def send(self, data):
        if self.delay_s:
            await asyncio.sleep(self.delay_s)
        self.sent.append(data)


def _handle(tmp_path):
    from console.mock_vehicle.core import MockVehicle

    return server.Handle(MockVehicle(time.monotonic, lambda: "t"), tmp_path, 0)


def test_a_slow_control_connection_does_not_delay_the_others(tmp_path):
    async def body():
        h = _handle(tmp_path)
        fast, slow = FakeWS(), FakeWS(delay_s=1.0)
        h.attach_control("c-fast", fast, "a")
        h.attach_control("c-slow", slow, "b")
        t0 = time.monotonic()
        await h._broadcast_control({"t": "tick", "state_seq": 1})
        assert time.monotonic() - t0 < 0.05
        await asyncio.sleep(0.05)
        assert len(fast.sent) == 1
        await asyncio.sleep(0.3)
        assert slow.transport.aborted  # its own 0.2 s send watchdog fired
        h.close()
        await h.wait_closed()

    asyncio.run(body())


def test_a_pending_scan_does_not_drop_the_grid(tmp_path):
    async def body():
        h = _handle(tmp_path)
        ws = FakeWS(delay_s=0.3)
        sub = {"ws": ws, "rates": {"scan": 10, "grid": 1}, "last": {}}
        h.media["m1"] = sub
        await h._send_media("m1", sub, "scan", b"scan")
        await h._send_media("m1", sub, "grid", b"grid")
        await asyncio.sleep(0.4)
        assert h.media_drops == 0 and ws.sent.count(b"grid") == 1
        h.close()
        await h.wait_closed()

    asyncio.run(body())


@pytest.mark.parametrize(
    "text",
    ['{"t": "stop", "request_id": "s", "source": 1e400}', "[" * 1500 + "]" * 1500],
)
def test_parse_refuses_non_finite_numbers_and_deep_nesting(text):
    assert server._parse(text) is None


def test_outgoing_json_is_strict():
    assert server._encode({"t": "event", "x": float("inf")}) is None
    assert json.loads(server._encode({"t": "event", "x": 1.0})) == {
        "t": "event",
        "x": 1.0,
    }
