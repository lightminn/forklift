"""The learned proposer in its own process (stage 3 plan, task 6): the Isaac interpreter has no onnxruntime, so the
mission runs the model in the learned venv and talks to it over stdin/stdout.

Protocol: each request is an 8-byte little-endian length and an .npy of the RGB (H, W, 3) uint8; each reply is an
8-byte length and an .npz with ``masks`` (n, H, W) bool, ``scores`` (n,) float and ``ids`` (n,) str, or ``error``.
"""
from __future__ import annotations

import io
import struct
import subprocess
import sys
import threading

import numpy as np

from forklift_core.perception.instance_proposals import InstanceProposal


def _send(stream, payload: bytes) -> None:
    stream.write(struct.pack("<Q", len(payload)))
    stream.write(payload)
    stream.flush()


def _recv(stream) -> bytes | None:
    head = stream.read(8)
    if len(head) < 8:
        return None
    (n,) = struct.unpack("<Q", head)
    return stream.read(n)


def serve(proposer, stdin=None, stdout=None) -> None:
    stdin = stdin or sys.stdin.buffer
    stdout = stdout or sys.stdout.buffer
    while True:
        raw = _recv(stdin)
        if raw is None:
            return
        rgb = np.load(io.BytesIO(raw))
        buf = io.BytesIO()
        try:
            props = proposer.propose(type("Scene", (), {"rgb": rgb})())
            masks = np.stack([p.mask for p in props]) if props else np.zeros((0, *rgb.shape[:2]), bool)
            np.savez(buf, masks=masks, scores=np.array([p.score for p in props], float),
                     ids=np.array([p.proposal_id for p in props]))
        except Exception as exc:  # noqa: BLE001 -- reported to the client, which falls back
            np.savez(buf, error=np.array(f"{type(exc).__name__}: {exc}"))
        _send(stdout, buf.getvalue())


class WorkerProposer:
    """The client: ``propose(scene)`` as LearnedProposer, the model in ``python`` (the learned venv).

    Every request has a deadline (``first_timeout_s`` for the first, which includes the model load, ``timeout_s``
    after): past it the worker is killed and reaped and the call raises, so the caller falls back (Codex review 1:
    a silent worker otherwise blocked the mission loop). A dead worker raises at once on every later call."""

    def __init__(self, python, onnx_path, *, conf: float = 0.05, providers: str = "CPUExecutionProvider", cwd=None,
                 env=None, command=None, timeout_s: float = 10.0, first_timeout_s: float = 120.0):
        cmd = command or [str(python), "-m", "forklift_learned.worker", str(onnx_path), str(conf), providers]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, cwd=cwd, env=env)
        self.timeout_s, self.first_timeout_s, self.calls = float(timeout_s), float(first_timeout_s), 0

    def _exchange(self, payload: bytes) -> bytes | None:
        out: dict = {}

        def run():
            try:
                _send(self.proc.stdin, payload)
                out["raw"] = _recv(self.proc.stdout)
            except (OSError, ValueError) as exc:  # broken pipe, closed file
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
        return out.get("raw")

    def propose(self, scene) -> list:
        if self.proc.poll() is not None:
            raise RuntimeError(f"learned worker ended (code {self.proc.returncode})")
        buf = io.BytesIO()
        np.save(buf, np.asarray(scene.rgb, dtype=np.uint8))
        raw = self._exchange(buf.getvalue())
        if raw is None:
            raise RuntimeError(f"learned worker ended (code {self.proc.poll()})")
        z = np.load(io.BytesIO(raw))
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


def main(argv=None) -> int:
    from forklift_learned.proposer import LearnedProposer  # noqa: PLC0415

    argv = sys.argv[1:] if argv is None else argv
    providers = argv[2].split(",") if len(argv) > 2 else None
    serve(LearnedProposer(argv[0], conf=float(argv[1]) if len(argv) > 1 else 0.05, providers=providers))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
