"""The learned proposer in its own process (stage 3 plan, task 6): the Isaac interpreter has no onnxruntime, so the
mission runs the model in the learned venv and talks to it over stdin/stdout.

Protocol: each request is an 8-byte little-endian length and an .npy of the RGB (H, W, 3) uint8; each reply is an
8-byte length and an .npz with ``masks`` (n, H, W) bool, ``scores`` (n,) float and ``ids`` (n,) str, or ``error``;
either way ``infer_ms``, the worker's own wall time for the model call (a client that does not read it is unaffected).
An empty request asks for the session's active providers (reply ``providers``).

A3 (docs/plans/2026-10-28-learned-worker-shared.md): ``--serve-socket PATH`` serves the same requests to several
sessions over a Unix socket. A connection opens with a handshake -- a JSON object of what the client expects (protocol,
model sha256, input shape, post-processing, providers, device); any difference refuses the connection -- and each
reply adds ``queue_ms`` (the wait for the model when calls are serialised). A connection that breaks ends that
connection only. ``SocketProposer`` is the client; it raises ``learned_server_lost:`` when the server is gone.
"""
from __future__ import annotations

import hashlib
import io
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time

import numpy as np

from forklift_core.perception.instance_proposals import InstanceProposal


def _send(stream, payload: bytes) -> None:
    stream.write(struct.pack("<Q", len(payload)))
    stream.write(payload)
    stream.flush()


MAX_MESSAGE = 1 << 28  # 256 MB: a 640 x 480 frame and eight masks are a few MB


def _recv(stream) -> bytes | None:
    head = stream.read(8)
    if len(head) < 8:
        return None
    (n,) = struct.unpack("<Q", head)
    if n > MAX_MESSAGE:
        raise RuntimeError(f"implausible message length {n} (stray output on the protocol stream?)")
    payload = stream.read(n)
    if len(payload) != n:  # the stream ended inside a message (Codex, A3 P0 review)
        raise RuntimeError(f"short message: {len(payload)} of {n} bytes before the end of the stream")
    return payload


def protocol_stdout():
    """The protocol's binary stream: the real stdout, kept on its own descriptor; fd 1 and ``sys.stdout`` then go to
    stderr, so nothing else the process prints (onnxruntime's preload diagnostics, C libraries) reaches the client
    (Codex review, stage 3b). Call before importing or building the model."""
    sys.stdout.flush()
    fd = os.dup(1)
    os.dup2(2, 1)
    sys.stdout = sys.stderr
    return os.fdopen(fd, "wb")


def serve(proposer, stdin=None, stdout=None) -> None:
    stdin = stdin or sys.stdin.buffer
    stdout = stdout or sys.stdout.buffer
    if hasattr(proposer, "warm_up"):
        proposer.warm_up()  # inside the first request's (longer) deadline, before any frame waits on it
    while True:
        raw = _recv(stdin)
        if raw is None:
            return
        buf = io.BytesIO()
        if not raw:
            np.savez(buf, providers=np.array(list(getattr(proposer, "active_providers", [])), dtype=str))
            _send(stdout, buf.getvalue())
            continue
        rgb = np.load(io.BytesIO(raw))
        t0 = time.perf_counter()
        try:
            props = proposer.propose(type("Scene", (), {"rgb": rgb})())
            infer_ms = 1e3 * (time.perf_counter() - t0)
            masks = np.stack([p.mask for p in props]) if props else np.zeros((0, *rgb.shape[:2]), bool)
            np.savez(buf, masks=masks, scores=np.array([p.score for p in props], float),
                     ids=np.array([p.proposal_id for p in props]), infer_ms=np.array(infer_ms))
        except Exception as exc:  # noqa: BLE001 -- reported to the client, which falls back
            np.savez(buf, error=np.array(f"{type(exc).__name__}: {exc}"),
                     infer_ms=np.array(1e3 * (time.perf_counter() - t0)))
        _send(stdout, buf.getvalue())


class WorkerProposer:
    """The client: ``propose(scene)`` as LearnedProposer, the model in ``python`` (the learned venv).

    Every request has a deadline (``first_timeout_s`` for the first, which includes the model load, ``timeout_s``
    after): past it the worker is killed and reaped and the call raises, so the caller falls back (Codex review 1:
    a silent worker otherwise blocked the mission loop). A dead worker raises at once on every later call."""

    def __init__(self, python, onnx_path, *, conf: float = 0.05, providers: str = "CPUExecutionProvider", cwd=None,
                 env=None, command=None, timeout_s: float = 10.0, first_timeout_s: float = 120.0, threads: int = 4,
                 device_id: int = 0):
        # threads: onnxruntime's pool, pinned (unpinned it sized itself to the machine, not the Slurm allocation:
        # 830 ms a round trip on ws1 against 106 ms at 4 threads)
        cmd = command or [str(python), "-m", "forklift_learned.worker", str(onnx_path), str(conf), providers,
                          str(int(threads)), str(int(device_id))]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, cwd=cwd, env=env)
        self.timeout_s, self.first_timeout_s, self.calls = float(timeout_s), float(first_timeout_s), 0
        self.last_infer_ms = None  # the worker's own time for the latest call (None: no reply, or an older worker)

    def _exchange(self, payload: bytes) -> bytes | None:
        out: dict = {}

        def run():
            try:
                _send(self.proc.stdin, payload)
                out["raw"] = _recv(self.proc.stdout)
            except Exception as exc:  # noqa: BLE001 -- broken pipe, closed file, a corrupt stream
                out["error"] = exc

        t = threading.Thread(target=run, daemon=True)
        t.start()
        deadline = self.first_timeout_s if self.calls == 0 else self.timeout_s
        t.join(deadline)
        self.calls += 1
        if t.is_alive():
            self._kill()
            t.join(5)
            raise RuntimeError(f"learned worker missed its deadline ({deadline:g} s); killed")
        if isinstance(out.get("error"), RuntimeError):  # the stream is out of step: no later reply can be trusted
            self._kill()
            raise RuntimeError(f"learned worker protocol broken: {out['error']}")
        return out.get("raw")

    def providers(self) -> list:
        """The worker session's active onnxruntime providers (what really runs, not what was asked)."""
        if self.proc.poll() is not None:
            raise RuntimeError(f"learned worker ended (code {self.proc.returncode})")
        raw = self._exchange(b"")
        if raw is None:
            raise RuntimeError(f"learned worker ended (code {self.proc.poll()})")
        return [str(p) for p in np.load(io.BytesIO(raw))["providers"]]

    def propose(self, scene) -> list:
        self.last_infer_ms = None
        if self.proc.poll() is not None:
            raise RuntimeError(f"learned worker ended (code {self.proc.returncode})")
        buf = io.BytesIO()
        np.save(buf, np.asarray(scene.rgb, dtype=np.uint8))
        raw = self._exchange(buf.getvalue())
        if raw is None:
            raise RuntimeError(f"learned worker ended (code {self.proc.poll()})")
        z = np.load(io.BytesIO(raw))
        if "infer_ms" in z.files:
            self.last_infer_ms = float(z["infer_ms"])
        if "error" in z.files:
            raise RuntimeError(str(z["error"]))
        return [InstanceProposal(str(i), m, float(s), "learned") for m, s, i in zip(z["masks"], z["scores"], z["ids"])]

    def _kill(self) -> None:
        if self.proc.poll() is None:
            self.proc.kill()
        self.proc.wait()
        self._release()

    def _release(self) -> None:
        """Close the pipes of an ended worker (stdin off-thread: a stuck send may still hold its lock briefly)."""
        threading.Thread(target=_quiet_close, args=(self.proc.stdin,), daemon=True).start()
        _quiet_close(self.proc.stdout)

    def close(self, timeout_s: float = 10.0) -> None:
        """End the worker within ``timeout_s``: its stdin is closed off-thread (a send stuck in the pipe holds the
        writer's lock -- Codex re-review 1), and a worker still alive at the deadline is killed and reaped. The
        pipes are closed either way."""
        if self.proc.poll() is None:
            threading.Thread(target=_quiet_close, args=(self.proc.stdin,), daemon=True).start()
            try:
                self.proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                pass
        self._kill()


def _quiet_close(stream) -> None:
    try:
        stream.close()
    except (OSError, ValueError):
        pass


# --- A3: one worker shared by several sessions over a Unix socket ------------------------------------------------

PROTOCOL = 1
HANDSHAKE_KEYS = ("protocol", "model_sha256", "input_shape", "conf", "iou", "max_det", "providers", "device_id")
SOCKET_PATH_MAX = 107  # AF_UNIX sun_path is 108 bytes with its NUL


def file_sha256(path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def describe(proposer, model_sha256: str, device_id: int) -> dict:
    """What a server runs, as the handshake compares it: the model by content, its input, the post-processing and the
    providers it really runs on."""
    shape = [int(d) if isinstance(d, int) else str(d) for d in proposer.session.get_inputs()[0].shape]
    return {"protocol": PROTOCOL, "model_sha256": model_sha256, "input_shape": shape, "conf": float(proposer.conf),
            "iou": float(proposer.iou), "max_det": int(proposer.max_det),
            "providers": list(getattr(proposer, "active_providers", [])), "device_id": int(device_id)}


def handshake_problems(expected: dict, server: dict) -> list[str]:
    """Why a client's expectation and a server differ (empty: they match). Every key is compared, providers as the
    server really runs them (a client asking for CUDA needs CUDA active)."""
    out = []
    for k in HANDSHAKE_KEYS:
        if k not in expected:
            out.append(f"{k}: the client did not say")
        elif k == "providers":
            if list(expected[k]) != list(server.get(k, [])):
                out.append(f"providers: the client expects {expected[k]}, the server runs {server.get(k)}")
        elif expected[k] != server.get(k):
            out.append(f"{k}: the client expects {expected[k]!r}, the server has {server.get(k)!r}")
    return out


def _reply(proposer, rgb, lock, t_recv: float) -> bytes:
    buf = io.BytesIO()
    t_wait = time.perf_counter()
    if lock is not None:
        lock.acquire()
    t0 = time.perf_counter()
    queue_ms = 1e3 * (t0 - t_wait)
    try:
        props = proposer.propose(type("Scene", (), {"rgb": rgb})())
        infer_ms = 1e3 * (time.perf_counter() - t0)
        masks = np.stack([p.mask for p in props]) if props else np.zeros((0, *rgb.shape[:2]), bool)
        np.savez(buf, masks=masks, scores=np.array([p.score for p in props], float),
                 ids=np.array([p.proposal_id for p in props]), infer_ms=np.array(infer_ms),
                 queue_ms=np.array(queue_ms), decode_ms=np.array(1e3 * (t_wait - t_recv)))
    except Exception as exc:  # noqa: BLE001 -- reported to that client only
        np.savez(buf, error=np.array(f"{type(exc).__name__}: {exc}"),
                 infer_ms=np.array(1e3 * (time.perf_counter() - t0)), queue_ms=np.array(queue_ms))
    finally:
        if lock is not None:
            lock.release()
    return buf.getvalue()


def _handle(conn, proposer, info: dict, lock, log) -> None:
    f = conn.makefile("rwb")
    try:
        raw = _recv(f)
        if raw is None:
            return
        try:
            expected = json.loads(raw.decode())
        except (ValueError, UnicodeDecodeError):
            expected = {}
        problems = handshake_problems(expected if isinstance(expected, dict) else {}, info)
        _send(f, json.dumps({"ok": not problems, "problems": problems, "server": info}).encode())
        if problems:
            log(f"refused a client: {'; '.join(problems)}")
            return
        while True:
            raw = _recv(f)
            if raw is None:
                return
            t_recv = time.perf_counter()
            if not raw:
                buf = io.BytesIO()
                np.savez(buf, providers=np.array(info["providers"], dtype=str))
                _send(f, buf.getvalue())
                continue
            _send(f, _reply(proposer, np.load(io.BytesIO(raw)), lock, t_recv))
    except (OSError, RuntimeError, ValueError) as exc:  # this connection only: the server and the others go on
        log(f"a connection ended: {type(exc).__name__}: {exc}")
    finally:
        _quiet_close(f)
        _quiet_close(conn)


def serve_socket(proposer, path, info: dict, *, lock: str = "serial", ready=None, log=None, stop=None,
                 warm: bool = True) -> None:
    """Serve ``proposer`` to every client that connects to the Unix socket ``path`` (one thread a connection).
    ``lock``: "serial" runs one call at a time (``queue_ms`` is the wait), "concurrent" lets onnxruntime run them
    together (its sessions take concurrent run calls). ``ready`` is called once the socket listens -- after the warm-up
    -- so a session starts only on a warmed server. ``stop``: an Event that ends the loop (tests)."""
    if len(os.fsencode(str(path))) > SOCKET_PATH_MAX:
        raise ValueError(f"socket path {path} is longer than AF_UNIX allows ({SOCKET_PATH_MAX} bytes)")
    log = log or (lambda m: print(f"learned server: {m}", file=sys.stderr, flush=True))
    if warm and hasattr(proposer, "warm_up"):
        proposer.warm_up()
    lk = threading.Lock() if lock == "serial" else None
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass
    srv.bind(str(path))
    srv.listen(16)
    srv.settimeout(0.2)
    conns = []
    if ready is not None:
        ready()
    try:
        while stop is None or not stop.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            conns.append(conn)
            threading.Thread(target=_handle, args=(conn, proposer, info, lk, log), daemon=True).start()
    finally:
        srv.close()
        for conn in conns:  # as a server that ends: its clients see the connection close
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass


class SocketProposer:
    """The client of a shared server: ``propose(scene)`` as WorkerProposer. The handshake refuses a server that runs
    another model or settings; any later failure -- the server gone, a broken stream, a missed deadline -- raises
    ``learned_server_lost:`` (the shared mode ends the session instead of starting a worker of its own, plan 2)."""

    def __init__(self, path, expected: dict, *, timeout_s: float = 10.0, connect_timeout_s: float = 30.0):
        self.timeout_s, self.calls = float(timeout_s), 0
        self.last_infer_ms = self.last_queue_ms = None
        deadline = time.monotonic() + connect_timeout_s
        while True:
            try:
                self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.sock.connect(str(path))
                break
            except OSError as exc:
                self.sock.close()
                if time.monotonic() > deadline:
                    raise RuntimeError(f"learned_server_lost: no server at {path} ({exc})") from exc
                time.sleep(0.2)
        self.sock.settimeout(connect_timeout_s)
        self.f = self.sock.makefile("rwb")
        _send(self.f, json.dumps(expected).encode())
        raw = _recv(self.f)
        answer = json.loads(raw.decode()) if raw else {"ok": False, "problems": ["no answer"]}
        if not answer.get("ok"):
            self.close()
            raise RuntimeError(f"learned server refused the handshake: {'; '.join(answer.get('problems', []))}")
        self.server = answer.get("server", {})
        self.sock.settimeout(self.timeout_s)

    def _exchange(self, payload: bytes) -> bytes:
        try:
            _send(self.f, payload)
            raw = _recv(self.f)
        except (OSError, RuntimeError, ValueError) as exc:
            raise RuntimeError(f"learned_server_lost: {type(exc).__name__}: {exc}") from exc
        if raw is None:
            raise RuntimeError("learned_server_lost: the server closed the connection")
        self.calls += 1
        return raw

    def providers(self) -> list:
        return [str(p) for p in np.load(io.BytesIO(self._exchange(b"")))["providers"]]

    def propose(self, scene) -> list:
        self.last_infer_ms = self.last_queue_ms = None
        buf = io.BytesIO()
        np.save(buf, np.asarray(scene.rgb, dtype=np.uint8))
        raw = self._exchange(buf.getvalue())
        try:  # every array read inside: a damaged member fails only when it is read (Codex)
            z = np.load(io.BytesIO(raw))
            arrays = {k: z[k] for k in z.files}
        except Exception as exc:  # noqa: BLE001 -- a reply that does not decode: the stream cannot be trusted
            raise RuntimeError(f"learned_server_lost: undecodable reply ({type(exc).__name__}: {exc})") from exc
        if "infer_ms" in arrays:
            self.last_infer_ms = float(arrays["infer_ms"])
        if "queue_ms" in arrays:
            self.last_queue_ms = float(arrays["queue_ms"])
        if "error" in arrays:
            raise RuntimeError(str(arrays["error"]))
        return [InstanceProposal(str(i), m, float(s), "learned")
                for m, s, i in zip(arrays["masks"], arrays["scores"], arrays["ids"])]

    def close(self, timeout_s: float = 0.0) -> None:
        _quiet_close(self.f)
        _quiet_close(self.sock)


def serve_socket_main(argv) -> int:
    """``--serve-socket PATH [--lock serial|concurrent] ONNX CONF PROVIDERS THREADS DEVICE``: prints READY <info> on
    stdout once the warmed server listens."""
    path, rest = argv[1], argv[2:]
    lock = "serial"
    if rest[:1] == ["--lock"]:
        lock, rest = rest[1], rest[2:]
    from forklift_learned import proposer  # noqa: PLC0415

    onnx, conf = rest[0], float(rest[1]) if len(rest) > 1 else 0.05
    providers = rest[2].split(",") if len(rest) > 2 else None
    threads = int(rest[3]) if len(rest) > 3 else None
    device_id = int(rest[4]) if len(rest) > 4 else 0
    p = proposer.LearnedProposer(onnx, conf=conf, providers=providers, threads=threads, device_id=device_id)
    p.warm_up()  # before describing it: the providers the warmed-up session really runs
    info = describe(p, file_sha256(onnx), device_id)
    serve_socket(p, path, info, lock=lock, warm=False, ready=lambda: print("READY " + json.dumps(info), flush=True))
    return 0


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["--serve-socket"]:
        return serve_socket_main(argv)
    out = protocol_stdout()
    from forklift_learned import proposer  # noqa: PLC0415

    providers = argv[2].split(",") if len(argv) > 2 else None
    threads = int(argv[3]) if len(argv) > 3 else None
    device_id = int(argv[4]) if len(argv) > 4 else 0
    serve(proposer.LearnedProposer(argv[0], conf=float(argv[1]) if len(argv) > 1 else 0.05, providers=providers,
                                   threads=threads, device_id=device_id), stdout=out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
