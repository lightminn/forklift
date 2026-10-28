"""A3 (docs/plans/2026-10-28-learned-worker-shared.md): one learned worker shared by several sessions over a Unix
socket -- the handshake refuses another model or settings, each client gets its own replies, a connection that
breaks ends that connection only, and a lost server raises learned_server_lost."""
import os
import socket
import threading
import time

import numpy as np
import pytest

from forklift_core.perception.instance_proposals import InstanceProposal
from forklift_learned import worker as W

INFO = {"protocol": W.PROTOCOL, "model_sha256": "ab" * 32, "input_shape": [1, 3, 48, 64], "conf": 0.05, "iou": 0.6,
        "max_det": 8, "providers": ["CUDAExecutionProvider", "CPUExecutionProvider"], "device_id": 0}


class Echo:
    """Proposes one mask whose score encodes the frame's marker pixel, so a reply can be traced to its request."""

    def __init__(self, delay_s=0.0):
        self.delay_s, self.active = delay_s, 0
        self.max_active, self.lock = 0, threading.Lock()

    def propose(self, scene):
        with self.lock:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
        try:
            time.sleep(self.delay_s)
            marker = int(scene.rgb[0, 0, 0])
            m = np.zeros(scene.rgb.shape[:2], bool)
            m[:2, :marker % 60 + 1] = True
            return [InstanceProposal(f"l{marker}", m, marker / 255.0, "learned")]
        finally:
            with self.lock:
                self.active -= 1


class Scene:
    def __init__(self, marker):
        self.rgb = np.full((48, 64, 3), marker, np.uint8)


@pytest.fixture
def server(tmp_path):
    started = []

    def start(proposer=None, lock="serial", info=INFO):
        path = str(tmp_path / f"s{len(started)}.sock")
        stop, ready = threading.Event(), threading.Event()
        logs = []
        t = threading.Thread(target=W.serve_socket, args=(proposer or Echo(), path, info),
                             kwargs={"lock": lock, "ready": ready.set, "stop": stop, "log": logs.append}, daemon=True)
        t.start()
        assert ready.wait(5)
        started.append((stop, t))
        return path, stop, logs

    yield start
    for stop, t in started:
        stop.set()
        t.join(5)


def test_each_client_gets_the_replies_to_its_own_frames(server):
    path, _, _ = server(Echo(delay_s=0.01), lock="concurrent")
    clients = [W.SocketProposer(path, INFO) for _ in range(4)]
    errors = []

    def run(k, c):
        for i in range(20):
            marker = 10 * k + i % 10
            out = c.propose(Scene(marker))
            if out[0].proposal_id != f"l{marker}" or out[0].mask.sum() != 2 * (marker % 60 + 1):
                errors.append((k, i, out[0].proposal_id))

    threads = [threading.Thread(target=run, args=(k, c)) for k, c in enumerate(clients)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(20)
    assert errors == []
    for c in clients:
        c.close()


def test_serial_mode_runs_one_call_at_a_time_and_reports_the_wait(server):
    echo = Echo(delay_s=0.05)
    path, _, _ = server(echo, lock="serial")
    clients = [W.SocketProposer(path, INFO) for _ in range(3)]
    waits = []
    threads = [threading.Thread(target=lambda c=c: (c.propose(Scene(1)), waits.append(c.last_queue_ms)))
               for c in clients]
    for t in threads:
        t.start()
    for t in threads:
        t.join(10)
    assert echo.max_active == 1 and max(waits) >= 40  # the last of three waited for the others
    assert all(c.last_infer_ms >= 45 for c in clients)
    for c in clients:
        c.close()


def test_concurrent_mode_lets_calls_overlap(server):
    echo = Echo(delay_s=0.1)
    path, _, _ = server(echo, lock="concurrent")
    clients = [W.SocketProposer(path, INFO) for _ in range(3)]
    threads = [threading.Thread(target=c.propose, args=(Scene(1),)) for c in clients]
    for t in threads:
        t.start()
    for t in threads:
        t.join(10)
    assert echo.max_active >= 2
    for c in clients:
        c.close()


@pytest.mark.parametrize("change, word", [({"model_sha256": "cd" * 32}, "model_sha256"), ({"conf": 0.8}, "conf"),
                                          ({"providers": ["CPUExecutionProvider"]}, "providers"),
                                          ({"protocol": 99}, "protocol"), ({"input_shape": [1, 3, 480, 640]}, "input")])
def test_the_handshake_refuses_another_model_or_settings(server, change, word):
    path, _, logs = server()
    with pytest.raises(RuntimeError, match=word):
        W.SocketProposer(path, {**INFO, **change})
    c = W.SocketProposer(path, INFO)  # the server goes on for a matching client
    assert c.propose(Scene(3))[0].proposal_id == "l3"
    c.close()


def test_a_client_that_says_nothing_of_a_key_is_refused(server):
    path, _, _ = server()
    with pytest.raises(RuntimeError, match="device_id: the client did not say"):
        W.SocketProposer(path, {k: v for k, v in INFO.items() if k != "device_id"})


def test_a_broken_connection_ends_only_itself(server):
    path, _, logs = server(Echo(delay_s=0.05), lock="serial")
    other = W.SocketProposer(path, INFO)
    # a client that sends half a frame and vanishes, and one that never reads its reply
    raw = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    raw.connect(path)
    f = raw.makefile("rwb")
    W._send(f, __import__("json").dumps(INFO).encode())
    W._recv(f)
    f.write((10 ** 6).to_bytes(8, "little") + b"x" * 100)
    f.flush()
    f.close()  # the file object holds the descriptor too
    raw.close()
    slow = W.SocketProposer(path, INFO)
    import io

    buf = io.BytesIO()
    np.save(buf, Scene(7).rgb)
    W._send(slow.f, buf.getvalue())  # and never read
    for i in range(5):
        assert other.propose(Scene(i))[0].proposal_id == f"l{i}"
    slow.close()
    other.close()


def test_a_lost_server_raises_learned_server_lost(server):
    path, stop, _ = server()
    c = W.SocketProposer(path, INFO, timeout_s=2.0)
    assert c.propose(Scene(1))
    stop.set()
    time.sleep(0.5)  # the server ends and closes its connections
    try:
        with pytest.raises(RuntimeError, match="learned_server_lost"):
            c.propose(Scene(2))
    finally:
        c.close()


def test_no_server_raises_learned_server_lost(tmp_path):
    with pytest.raises(RuntimeError, match="learned_server_lost"):
        W.SocketProposer(str(tmp_path / "none.sock"), INFO, connect_timeout_s=0.5)


def test_a_socket_path_too_long_for_af_unix_is_refused(tmp_path):
    path = tmp_path / ("x" * max(1, 120 - len(str(tmp_path))))
    with pytest.raises(ValueError, match="AF_UNIX"):
        W.serve_socket(Echo(), str(path), INFO)
    assert not os.path.exists(path)


def test_a_message_cut_short_by_the_end_of_the_stream_is_an_error():
    # Codex: read(n) returns fewer bytes at EOF; a truncated reply must not pass for a whole one
    import io

    with pytest.raises(RuntimeError, match="short message"):
        W._recv(io.BytesIO((10).to_bytes(8, "little") + b"abc"))
    assert W._recv(io.BytesIO((3).to_bytes(8, "little") + b"abc")) == b"abc"


def test_the_handshake_compares_every_key():
    assert W.handshake_problems(INFO, INFO) == []
    assert set(W.HANDSHAKE_KEYS) == set(INFO)
