"""M-c3: the console server inside the live runner, against a stand-in control loop (plan C3)."""

import asyncio
import itertools
import json
import threading
import time

import numpy as np
import pytest

pytest.importorskip("websockets")
pytest.importorskip("PIL")

from websockets.asyncio.client import connect  # noqa: E402

from console.live.server import LiveServer, MediaStore, camera_item, write_port_file  # noqa: E402
from forklift_core.live.link import Channels, Link  # noqa: E402
from forklift_core.mission.commands import Context, MissionCommands  # noqa: E402
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig, Feedback  # noqa: E402

EPOCH = 77
SAFE = {"load": {"loaded": False}, "fork_in_pocket": {"value": False}, "near": None,
        "frame": {"hold": "none", "slam": True}, "mission": {"phase": "observe"}}


class StandIn:
    """The runner's control thread: the link's cycle every 10 ms, a camera frame every 100 ms."""

    def __init__(self):
        counter = itertools.count(1)
        self.m = MissionCommands()
        self.a = Arbiter(EPOCH, ArbiterConfig(), lambda: f"tok{next(counter)}", mission=self.m)
        self.ch = Channels()
        self.link = Link(self.a, self.ch)
        self.store = MediaStore()
        self.welcome = {"t": "welcome", "api": "console/1", "vehicle_id": "isaac", "session_epoch": EPOCH,
                        "backend": "isaac", "capabilities": {"control": True}}
        self.server = LiveServer(self.ch, self.store, self.welcome)
        self._stop = threading.Event()
        self.frames = 0

    def run(self):
        k = 0
        while not self._stop.is_set():
            now = time.monotonic()
            stop_kind = self.link.begin(now, Feedback(now, 0.0))
            records = self.m.apply(k / 100, Context(stop_kind=stop_kind))
            self.link.end(now, records, SAFE, (0.0, 0.0))
            if k % 10 == 0:
                self.frames += 1
                rgb = np.full((40, 64, 3), self.frames % 255, dtype=np.uint8)
                self.store.put("camera_rgb", self.frames, camera_item(self.frames, rgb, now, k / 100, EPOCH))
            k += 1
            time.sleep(0.01)

    def __enter__(self):
        self.port = self.server.start()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self.thread.join(2)
        self.server.stop()


async def recv_until(ws, pred, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        msg = json.loads(await asyncio.wait_for(ws.recv(), end - time.monotonic()))
        if pred(msg):
            return msg
    raise TimeoutError


def test_hello_lease_stop_and_a_camera_frame_through_the_live_server(tmp_path):
    async def client(port):
        async with connect(f"ws://127.0.0.1:{port}/api/v1/control") as ws:
            await ws.send(json.dumps({"t": "hello", "api": "console/1", "client_id": "laptop"}))
            welcome = json.loads(await ws.recv())
            assert welcome["session_epoch"] == EPOCH
            await recv_until(ws, lambda m: m.get("t") == "tick")
            await ws.send(json.dumps({"t": "lease_request", "request_id": "L", "session_epoch": EPOCH}))
            lease = await recv_until(ws, lambda m: m.get("of") == "lease_request")
            assert lease["t"] == "ack" and lease["lease_id"]
            await ws.send(json.dumps({"t": "stop", "request_id": "S", "source": "key"}))
            stop = await recv_until(ws, lambda m: m.get("of") == "stop")
            assert stop["t"] == "ack"
        async with connect(f"ws://127.0.0.1:{port}/api/v1/media") as media:
            await media.send(json.dumps({"t": "media_subscribe", "camera_rgb": {"max_hz": 10}}))
            return await asyncio.wait_for(media.recv(), 3.0)

    with StandIn() as s:
        write_port_file(tmp_path / "port.json", {"port": s.port, "session_epoch": EPOCH, "state": "running"})
        assert json.loads((tmp_path / "port.json").read_text())["port"] == s.port
        data = asyncio.run(client(s.port))
        assert isinstance(data, bytes) and b'"frame_id"' in data and b"\xff\xd8" in data
        assert s.a.mode == "STOPPED"


def test_stopping_the_server_closes_connections_with_1001():
    async def client(port, stopped):
        async with connect(f"ws://127.0.0.1:{port}/api/v1/control") as ws:
            await ws.send(json.dumps({"t": "hello", "api": "console/1", "client_id": "laptop"}))
            await ws.recv()
            stopped.set()
            try:
                while True:
                    await asyncio.wait_for(ws.recv(), 5.0)
            except Exception:  # noqa: BLE001
                return ws.close_code

    with StandIn() as s:
        stopped = threading.Event()
        result = {}
        t = threading.Thread(target=lambda: result.setdefault("code", asyncio.run(client(s.port, stopped))))
        t.start()
        assert stopped.wait(3)
        s._stop.set()
        s.thread.join(2)
        assert s.server.stop() is True
        t.join(5)
    assert result["code"] == 1001


def test_many_clients_are_closed_within_the_stop_budget():
    async def clients(port, n, ready):
        conns = [await connect(f"ws://127.0.0.1:{port}/api/v1/control") for _ in range(n)]
        for k, ws in enumerate(conns):
            await ws.send(json.dumps({"t": "hello", "api": "console/1", "client_id": f"c{k}"}))
            await ws.recv()
        ready.set()
        await asyncio.sleep(10)  # never read again: closes must not wait on them one by one

    with StandIn() as s:
        ready = threading.Event()
        threading.Thread(target=lambda: asyncio.run(clients(s.port, 20, ready)), daemon=True).start()
        assert ready.wait(5)
        s._stop.set()
        s.thread.join(2)
        start = time.monotonic()
        assert s.server.stop(timeout_s=5.0) is True
        assert time.monotonic() - start < 4.0
