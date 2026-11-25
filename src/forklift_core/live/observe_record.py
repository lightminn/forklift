"""The observe run's record (``observe_record/1``, tools/real_observe.py): its schema name and the completeness proof a
replay checks -- shared by the observe tool, which writes it, and the replay run (PR 7 7r), which reads it."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

SCHEMA = "observe_record/1"


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):  # streamed: an hour's record is gigabytes (round 3 P3)
            h.update(chunk)
    return h.hexdigest()


def last_line(path: Path, block: int = 1 << 16) -> bytes:
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        end = f.tell()
        pos, buf = end, b""
        while pos > 0:
            step = min(block, pos)
            pos -= step
            f.seek(pos)
            buf = f.read(step) + buf
            body = buf.rstrip(b"\n")
            if b"\n" in body or pos == 0:
                return body.rsplit(b"\n", 1)[-1]
    return b""


def record_complete(path: Path) -> bool:
    """A record a replay may take: its .ok names its current sha256 and its last row is a complete end."""
    p = Path(path)
    try:
        want = Path(str(p) + ".ok").read_text(encoding="ascii").strip()
        if file_sha256(p) != want:
            return False
        last = last_line(p)
    except OSError:
        return False
    try:
        end = json.loads(last)
    except ValueError:
        return False
    return isinstance(end, dict) and end.get("t") == "end" and end.get("complete") is True


__all__ = ["SCHEMA", "file_sha256", "last_line", "record_complete"]
