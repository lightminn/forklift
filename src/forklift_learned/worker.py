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
    """The client: ``propose(scene)`` as LearnedProposer, the model in ``python`` (the learned venv)."""

    def __init__(self, python, onnx_path, *, conf: float = 0.05, cwd=None, env=None, command=None):
        cmd = command or [str(python), "-m", "forklift_learned.worker", str(onnx_path), str(conf)]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, cwd=cwd, env=env)

    def propose(self, scene) -> list:
        buf = io.BytesIO()
        np.save(buf, np.asarray(scene.rgb, dtype=np.uint8))
        _send(self.proc.stdin, buf.getvalue())
        raw = _recv(self.proc.stdout)
        if raw is None:
            raise RuntimeError(f"learned worker ended (code {self.proc.poll()})")
        z = np.load(io.BytesIO(raw))
        if "error" in z.files:
            raise RuntimeError(str(z["error"]))
        return [InstanceProposal(str(i), m, float(s), "learned") for m, s, i in zip(z["masks"], z["scores"], z["ids"])]

    def close(self) -> None:
        if self.proc.poll() is None:
            self.proc.stdin.close()
            self.proc.wait(timeout=10)


def main(argv=None) -> int:
    from forklift_learned.proposer import LearnedProposer  # noqa: PLC0415

    argv = sys.argv[1:] if argv is None else argv
    serve(LearnedProposer(argv[0], conf=float(argv[1]) if len(argv) > 1 else 0.05))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
