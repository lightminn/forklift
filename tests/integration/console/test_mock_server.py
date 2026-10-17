import asyncio
import json
import struct

import pytest

websockets = pytest.importorskip("websockets")
pytest.importorskip("PIL")

from console.mock_vehicle import server  # noqa: E402

OPENED = []


async def _open(port, path, **kw):
    ws = await websockets.connect(f"ws://127.0.0.1:{port}{path}", **kw)
    OPENED.append(ws)
    return ws


async def _close_clients():
    """Close every client this test opened; a client that stopped reading is aborted."""
    while OPENED:
        ws = OPENED.pop()
        try:
            await asyncio.wait_for(ws.close(), 1.0)
        except (TimeoutError, websockets.ConnectionClosed):
            ws.transport.abort()


async def _until(ws, t, pred=lambda m: True, echo=None):
    """Read until a message of type t that satisfies pred; answer ticks with an echo if given."""
    while True:
        msg = json.loads(await ws.recv())
        if echo and msg.get("t") == "tick":
            await ws.send(
                json.dumps(
                    dict(
                        echo,
                        echo={"state_seq": msg["state_seq"], "token": msg["token"]},
                    )
                )
            )
        if msg.get("t") == t and pred(msg):
            return msg


async def _keepalive(ws, keep, sink):
    async for raw in ws:
        msg = json.loads(raw)
        sink.append(msg)
        if msg.get("t") == "tick":
            await ws.send(
                json.dumps(
                    dict(
                        keep,
                        echo={"state_seq": msg["state_seq"], "token": msg["token"]},
                    )
                )
            )


def run(coro):
    return asyncio.run(asyncio.wait_for(coro, 20))


def test_second_client_is_observer_but_can_stop(tmp_path, unused_tcp_port):
    async def body():
        srv = await server.start("127.0.0.1", unused_tcp_port, tmp_path)
        try:
            a = await _open(unused_tcp_port, "/api/v1/control")
            await a.send(
                json.dumps({"t": "hello", "api": "console/1", "client_id": "a"})
            )
            wa = await _until(a, "welcome")
            await a.send(
                json.dumps(
                    {
                        "t": "lease_request",
                        "request_id": "1",
                        "session_epoch": wa["session_epoch"],
                    }
                )
            )
            lease_ack = await _until(a, "ack")
            assert lease_ack["request_id"] == "1"
            keep = {
                "t": "echo",
                "session_epoch": wa["session_epoch"],
                "lease_id": lease_ack["lease_id"],
            }
            seen_a = []
            pump_a = asyncio.create_task(
                _keepalive(a, keep, seen_a)
            )  # answers every tick for the whole test
            b = await _open(unused_tcp_port, "/api/v1/control")
            await b.send(
                json.dumps({"t": "hello", "api": "console/1", "client_id": "b"})
            )
            wb = await _until(b, "welcome")
            await b.send(
                json.dumps(
                    {
                        "t": "lease_request",
                        "request_id": "2",
                        "session_epoch": wb["session_epoch"],
                    }
                )
            )
            assert (await _until(b, "reject"))["reason"] == "busy"
            await b.send(
                json.dumps(
                    {
                        "t": "stop",
                        "request_id": "3",
                        "session_epoch": wb["session_epoch"],
                        "source": "console",
                    }
                )
            )
            assert (await _until(b, "ack"))["request_id"] == "3"
            for _ in range(50):
                if any(
                    m.get("t") == "state" and m["mode"] == "STOPPED" for m in seen_a
                ):
                    break
                await asyncio.sleep(0.05)
            stopped = next(
                m for m in seen_a if m.get("t") == "state" and m["mode"] == "STOPPED"
            )
            assert stopped["stop_kind"] == "safety_preempt"
            pump_a.cancel()
        finally:
            await _close_clients()
            srv.close()
            await srv.wait_closed()

    run(body())


def test_wrong_api_version_is_refused(tmp_path, unused_tcp_port):
    async def body():
        srv = await server.start("127.0.0.1", unused_tcp_port, tmp_path)
        try:
            a = await _open(unused_tcp_port, "/api/v1/control")
            await a.send(
                json.dumps({"t": "hello", "api": "console/0", "client_id": "a"})
            )
            with pytest.raises(websockets.ConnectionClosed):
                await _until(a, "welcome")
        finally:
            await _close_clients()
            srv.close()
            await srv.wait_closed()

    run(body())


def test_control_ticks_keep_flowing_while_a_media_client_never_reads(
    tmp_path, unused_tcp_port
):
    async def body():
        # 1 MB padding per camera frame fills the socket buffers within about a second
        srv = await server.start(
            "127.0.0.1", unused_tcp_port, tmp_path, test_frame_pad_bytes=1_000_000
        )
        try:
            m = await _open(
                unused_tcp_port,
                "/api/v1/media",
                max_queue=1,
                max_size=None,
                compression=None,
            )
            await m.send(
                json.dumps(
                    {
                        "t": "media_subscribe",
                        "client_id": "slow",
                        "camera_rgb": {"hz": 15},
                    }
                )
            )
            a = await _open(unused_tcp_port, "/api/v1/control")
            await a.send(
                json.dumps({"t": "hello", "api": "console/1", "client_id": "a"})
            )
            await _until(a, "welcome")
            loop = asyncio.get_running_loop()
            start, ticks = loop.time(), 0
            while loop.time() - start < 2.0:
                if json.loads(await a.recv()).get("t") == "tick":
                    ticks += 1
            assert (
                ticks >= 30
            )  # 20 Hz nominal; the unread media socket must not stall control
            assert (
                srv.media_drops > 0 or srv.media_closed > 0
            )  # congestion was really reached
        finally:
            await _close_clients()
            srv.close()
            await srv.wait_closed()

    run(body())


def test_static_files_are_served_with_module_mime_types(tmp_path, unused_tcp_port):
    import urllib.request

    (tmp_path / "js").mkdir()
    (tmp_path / "index.html").write_text("<!doctype html>", encoding="utf-8")
    (tmp_path / "js" / "main.js").write_text("export {};", encoding="utf-8")

    async def body():
        srv = await server.start("127.0.0.1", unused_tcp_port, tmp_path)
        try:

            def fetch(path):
                with urllib.request.urlopen(
                    f"http://127.0.0.1:{unused_tcp_port}{path}"
                ) as r:
                    return r.headers["Content-Type"]

            assert (await asyncio.to_thread(fetch, "/js/main.js")).startswith(
                "text/javascript"
            )
            assert (await asyncio.to_thread(fetch, "/")).startswith("text/html")
        finally:
            await _close_clients()
            srv.close()
            await srv.wait_closed()

    run(body())


def test_media_frames_carry_frame_id_and_jpeg(tmp_path, unused_tcp_port):
    async def body():
        srv = await server.start("127.0.0.1", unused_tcp_port, tmp_path)
        try:
            m = await _open(unused_tcp_port, "/api/v1/media")
            await m.send(
                json.dumps(
                    {"t": "media_subscribe", "client_id": "a", "camera_rgb": {"hz": 10}}
                )
            )
            for _ in range(20):
                data = await m.recv()
                if isinstance(data, bytes):
                    (n,) = struct.unpack(">I", data[:4])
                    head = json.loads(data[4 : 4 + n])
                    if head["t"] == "camera":
                        assert head["frame_id"] >= 1 and isinstance(
                            head["session_epoch"], int
                        )
                        assert data[4 + n : 4 + n + 2] == b"\xff\xd8"
                        return
            pytest.fail("no camera frame")
        finally:
            await _close_clients()
            srv.close()
            await srv.wait_closed()

    run(body())
