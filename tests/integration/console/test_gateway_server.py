import asyncio
import json
import math
import struct
import time

import pytest

websockets = pytest.importorskip("websockets")

from console.gateway import server  # noqa: E402
from console.gateway.convert import GridMessage, PoseSample, ScanMessage  # noqa: E402
from console.gateway.store import CameraFrame, Store  # noqa: E402

OPENED = []


async def _open(port, path):
    ws = await websockets.connect(f"ws://127.0.0.1:{port}{path}")
    OPENED.append(ws)
    return ws


async def _close_clients():
    while OPENED:
        ws = OPENED.pop()
        try:
            await asyncio.wait_for(ws.close(), 1.0)
        except (TimeoutError, websockets.ConnectionClosed):
            ws.transport.abort()


async def _until(ws, t, pred=lambda m: True):
    while True:
        msg = json.loads(await ws.recv())
        if msg.get("t") == t and pred(msg):
            return msg


def _parse_frame(data: bytes):
    n = struct.unpack(">I", data[:4])[0]
    return json.loads(data[4 : 4 + n]), data[4 + n :]


async def _media_until(ws, t):
    while True:
        head, body = _parse_frame(await ws.recv())
        if head["t"] == t:
            return head, body


def run(coro):
    return asyncio.run(asyncio.wait_for(coro, 20))


def filled_store():
    s = Store()
    now = time.monotonic()
    s.set_base(PoseSample("map", 3.0, 2.0, 0.0, received_s=now))
    s.set_map_to_odom((0.1, 0.0, 0.0), now)
    s.set_speed(0.25, now)
    s.set_grid(
        GridMessage(
            width=3,
            height=2,
            resolution_m=0.5,
            origin_x_m=-1.0,
            origin_y_m=-0.5,
            origin_yaw_rad=0.0,
            frame="map",
            data=[0, 100, -1, 0, 0, 50],
            received_s=now,
        )
    )
    s.set_scan(
        ScanMessage(-0.5, 0.5, 0.1, 10.0, [1.0, math.inf, 2.0], received_s=now),
        PoseSample("map", 2.9, 2.0, 0.0, received_s=now),
    )
    s.set_camera(CameraFrame(b"\xff\xd8\xff\xe0jpegdata", 123, now))
    return s


async def _start(store, port, tmp_path, backend="isaac"):
    return await server.start(
        store, "127.0.0.1", port, tmp_path, "fl-1", backend, 77, -0.34
    )


async def _hello(port, cid="a"):
    c = await _open(port, "/api/v1/control")
    await c.send(json.dumps({"t": "hello", "api": "console/1", "client_id": cid}))
    return c


def test_welcome_declares_a_monitor_only_connection_and_state_flows(
    tmp_path, unused_tcp_port
):
    async def body():
        gw = await _start(filled_store(), unused_tcp_port, tmp_path)
        try:
            c = await _hello(unused_tcp_port)
            w = await _until(c, "welcome")
            assert w["capabilities"]["control"] is False
            assert w["backend"] == "isaac" and w["session_epoch"] == 77
            s1 = await _until(c, "state")
            s2 = await _until(c, "state")
            assert s2["state_seq"] > s1["state_seq"]
            assert s2["mode"] is None and s2["permit"] is None
            # base_link (3, 2) facing +x; the rear axle is 0.34 m behind
            assert s2["pose"]["x_m"] == pytest.approx(2.66)
            assert s2["pose"]["frame"] == "map"
            assert s2["rear_axle_speed_mps"] == pytest.approx(0.25)
        finally:
            await _close_clients()
            gw.close()
            await gw.wait_closed()

    run(body())


def test_every_request_is_refused_stop_included(tmp_path, unused_tcp_port):
    async def body():
        gw = await _start(filled_store(), unused_tcp_port, tmp_path)
        try:
            c = await _hello(unused_tcp_port)
            await _until(c, "welcome")
            for i, t in enumerate(
                [
                    "stop",
                    "lease_request",
                    "mode_request",
                    "arm",
                    "mission_submit",
                    "recovery",
                    "targets_request",
                ]
            ):
                await c.send(json.dumps({"t": t, "request_id": f"r{i}"}))
                r = await _until(c, "reject", lambda m, i=i: m["request_id"] == f"r{i}")
                assert r["reason"] == "unsupported"
            # messages without a request id (echo, teleop) get no answer and change nothing
            await c.send(json.dumps({"t": "teleop", "speed_ratio": 1.0}))
            await c.send(json.dumps({"t": "stop", "request_id": "last"}))
            r = await _until(c, "reject")
            assert r["request_id"] == "last"
        finally:
            await _close_clients()
            gw.close()
            await gw.wait_closed()

    run(body())


def test_a_bad_hello_is_closed(tmp_path, unused_tcp_port):
    async def body():
        gw = await _start(filled_store(), unused_tcp_port, tmp_path)
        try:
            c = await _open(unused_tcp_port, "/api/v1/control")
            await c.send(
                json.dumps({"t": "hello", "api": "console/0", "client_id": "a"})
            )
            with pytest.raises(websockets.ConnectionClosed):
                await asyncio.wait_for(c.recv(), 3)
        finally:
            await _close_clients()
            gw.close()
            await gw.wait_closed()

    run(body())


def test_media_frames_carry_grid_scan_and_camera(tmp_path, unused_tcp_port):
    async def body():
        gw = await _start(filled_store(), unused_tcp_port, tmp_path)
        try:
            m = await _open(unused_tcp_port, "/api/v1/media")
            await m.send(
                json.dumps(
                    {
                        "t": "media_subscribe",
                        "client_id": "a",
                        "camera_rgb": {"hz": 15},
                        "scan": {"hz": 10},
                        "grid": {"hz": 1},
                    }
                )
            )
            head, cells = await _media_until(m, "grid")
            assert head["origin_m"] == [-1.0, -0.5]
            assert list(cells) == [1, 2, 0, 1, 1, 0]
            head, ranges = await _media_until(m, "scan")
            vals = struct.unpack("<3f", ranges)
            assert vals[0] == pytest.approx(1.0) and math.isnan(vals[1])
            assert head["pose"]["x_m"] == pytest.approx(2.9)
            head, jpeg = await _media_until(m, "camera")
            assert jpeg.startswith(b"\xff\xd8") and head["frame_id"] == 1
            assert head["capture"]["stamp_ns"] == 123
        finally:
            await _close_clients()
            gw.close()
            await gw.wait_closed()

    run(body())


def test_the_same_frame_is_sent_once_and_a_new_image_gets_a_new_id(
    tmp_path, unused_tcp_port
):
    async def body():
        store = filled_store()
        gw = await _start(store, unused_tcp_port, tmp_path)
        try:
            m = await _open(unused_tcp_port, "/api/v1/media")
            await m.send(
                json.dumps({"t": "media_subscribe", "client_id": "a", "camera_rgb": {}})
            )
            head, _ = await _media_until(m, "camera")
            assert head["frame_id"] == 1
            with pytest.raises(TimeoutError):  # nothing new: nothing sent
                await asyncio.wait_for(m.recv(), 0.4)
            store.set_camera(CameraFrame(b"\xff\xd8next", 124, time.monotonic()))
            head, _ = await _media_until(m, "camera")
            assert head["frame_id"] == 2
        finally:
            await _close_clients()
            gw.close()
            await gw.wait_closed()

    run(body())


def test_a_rotated_map_is_withheld_and_reported(tmp_path, unused_tcp_port):
    async def body():
        store = filled_store()
        store.set_grid(
            GridMessage(2, 1, 0.5, 0.0, 0.0, 0.2, "map", [0, 0], time.monotonic())
        )
        gw = await _start(store, unused_tcp_port, tmp_path)
        try:
            m = await _open(unused_tcp_port, "/api/v1/media")
            await m.send(
                json.dumps({"t": "media_subscribe", "client_id": "a", "grid": {}})
            )
            head, body = await _media_until(m, "grid")
            assert head["withdrawn"] is True and body == b""  # the page drops its copy
            c = await _hello(unused_tcp_port)
            st = await _until(c, "state", lambda s: s["notes"])
            assert "회전" in st["notes"][0]
        finally:
            await _close_clients()
            gw.close()
            await gw.wait_closed()

    run(body())


def test_the_gateway_never_claims_to_be_the_fake(tmp_path, unused_tcp_port):
    async def body():
        with pytest.raises(ValueError):
            await _start(Store(), unused_tcp_port, tmp_path, backend="mock")

    run(body())
