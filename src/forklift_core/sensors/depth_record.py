"""The depth frames' side files (PR 7 7e, docs/plans/2026-11-26-7e-depth-pocket-check.md 4.1-4.2).

A record ``<out>`` keeps its depth pixels in ``<out>.depth/``, made exclusively (an existing folder refuses the record:
two records never share pixel files, Codex plan round 2 P2-5). The folder holds ``depth_NNNN.npz`` files: uncompressed
``np.savez``, one ``uint16`` array per frame named ``fNNNNNN`` (the frame's index in the record), at most
``chunk_frames`` frames each. A file is written whole (built in memory, then written to a fresh ``x`` file, flushed and
fsynced) and its sha256 is reported as a ``depth_file`` row; the last partial chunk is closed the same way at the end
(``close``). Any failure sets ``failed`` -- the record is then incomplete and gets no ``.ok`` (plan 4.1).

``read_side_file(path, sha256, pin)`` reads one file back for the replay: opened once, its stat taken from that
descriptor (and checked against ``pin``), its bytes hashed, then parsed from those very bytes (``allow_pickle=False``)."""

from __future__ import annotations

import hashlib
import io
import os
from pathlib import Path

import numpy as np

from forklift_core.sensors.depth_cfg import frame_sha256

SUFFIX = ".depth"


def side_folder(record_path) -> Path:
    p = Path(record_path)
    return p.with_name(p.name + SUFFIX)


class DepthRecorder:
    def __init__(self, folder, chunk_frames: int) -> None:
        self.folder = Path(folder)
        os.mkdir(self.folder)  # exclusive: an existing folder raises FileExistsError
        self.chunk = int(chunk_frames)
        self._arrays: dict = {}
        self._file_index = 0
        self.frames = 0
        self.failed: str | None = None
        self._rows: list = []
        self.files: list = []  # names written whole

    def add(self, z16) -> tuple:
        """One frame's pixels: ``(file name, array key, frame sha256)``. Raises on a frame that is not a 2-D uint16
        array; returns the place even after a failure (the record is incomplete anyway)."""
        a = np.asarray(z16)
        if a.dtype != np.uint16 or a.ndim != 2:
            raise ValueError(f"depth frame must be a 2-D uint16 array, not {a.dtype} {a.shape}")
        a = np.ascontiguousarray(a, dtype="<u2")
        key = f"f{self.frames:06d}"
        name = f"depth_{self._file_index:04d}.npz"
        self._arrays[key] = a
        self.frames += 1
        sha = frame_sha256(a)
        if len(self._arrays) >= self.chunk:
            self._flush()
        return name, key, sha

    def _flush(self) -> None:
        if not self._arrays:
            return
        name = f"depth_{self._file_index:04d}.npz"
        arrays, self._arrays = self._arrays, {}
        self._file_index += 1
        if self.failed is not None:
            return
        try:
            buf = io.BytesIO()
            np.savez(buf, **arrays)
            data = buf.getvalue()
            with open(self.folder / name, "xb") as f:
                f.write(data)
                f.flush()
                os.fsync(f.fileno())
            self.files.append(name)
            self._rows.append({"t": "depth_file", "name": name, "frames": len(arrays), "bytes": len(data),
                               "sha256": hashlib.sha256(data).hexdigest()})
        except (OSError, ValueError) as exc:
            self.failed = f"{name}: {type(exc).__name__}: {exc}"

    def take_rows(self) -> list:
        out, self._rows = self._rows, []
        return out

    def close(self) -> list:
        """The last partial chunk written and the folder synced; the ``depth_file`` rows not yet taken."""
        self._flush()
        if self.failed is None:
            try:
                fd = os.open(self.folder, os.O_RDONLY)
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
            except OSError as exc:
                self.failed = f"folder sync: {exc}"
        return self.take_rows()


def read_side_file(path, sha256: str, pin: tuple | None = None) -> tuple:
    """``(arrays, pin)``: the file opened once, its ``(device, inode, size, mtime_ns)`` taken from that very
    descriptor (and compared with ``pin`` when given), its bytes hashed and parsed from those bytes
    (``ValueError`` when the pin or the hash differs; Codex 7e-1 round 1 P3-1)."""
    with open(path, "rb") as f:
        st = os.fstat(f.fileno())
        got = (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns)
        if pin is not None and tuple(pin) != got:
            raise ValueError(f"{Path(path).name}: not the file verified")
        data = f.read()
    if hashlib.sha256(data).hexdigest() != sha256:
        raise ValueError(f"{Path(path).name}: sha256 differs from the record")
    with np.load(io.BytesIO(data), allow_pickle=False) as z:
        return {k: z[k] for k in z.files}, got


__all__ = ["DepthRecorder", "SUFFIX", "read_side_file", "side_folder"]
