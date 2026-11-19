"""Synthetic (rehearsal) records: marked where they are written, refused where they are measured
(docs/design/2026-11-15-arrival-day-rehearsal.md 2.3).

- ``writer_marker(device, bench_path)``: what a record writer puts in its header as ``synthetic``. A pty device needs
  the rehearsal bench's record and a passing model-peer check (a real MCU never links through a pty); a bench on a
  real device is refused; a real device without a bench gets no marker (None).
- ``is_synthetic(header)``: marked, or a header whose device is a pty (an unmarked rehearsal record).
- ``refuse_or_mark(rows, allow)``: measurement entry points -- SyntheticRefused unless ``allow``; with it, the marks
  every output then carries (``synthetic_inputs``, ``not_for_judgement``).
"""

from __future__ import annotations

import json
import os
import re
import stat

from . import model_peer

PTY_PATH = re.compile(r"^/dev/pts/\d+$")
OUTPUT_MARKS = {"synthetic_inputs": True, "not_for_judgement": True}


class SyntheticRefused(Exception):
    pass


def _is_pty(device: str) -> bool:
    try:
        st = os.stat(device)
    except OSError:
        return False
    return stat.S_ISCHR(st.st_mode) and os.major(st.st_rdev) in model_peer.PTY_SLAVE_MAJORS


def _load_bench(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def writer_marker(device: str, bench_path: str | None) -> dict | None:
    pty = _is_pty(device)
    if not pty:
        if bench_path is not None:
            raise SyntheticRefused(f"--bench given but {device} is not a pty: the bench is for the model MCU only")
        return None
    if bench_path is None:
        raise SyntheticRefused(f"{device} is a pty: a rehearsal link needs --bench (a real MCU never links by pty)")
    bench = _load_bench(bench_path)
    why = model_peer.model_peer_refusal(device, bench)
    if why is not None:
        raise SyntheticRefused(f"the model-peer check refused {device}: {why}")
    return {"bench": bench}


def device_identity(device: str) -> dict | None:
    """What a record header keeps about its link device: the character device's major and minor (None: not one)."""
    try:
        st = os.stat(device)
    except OSError:
        return None
    if not stat.S_ISCHR(st.st_mode):
        return None
    return {"major": os.major(st.st_rdev), "minor": os.minor(st.st_rdev)}


def is_synthetic(header: dict) -> bool:
    """Marked, or linked through a pty -- by the header's device identity, or (older records) its path."""
    if header.get("synthetic"):
        return True
    ident = header.get("device_id") or {}
    if ident.get("major") in model_peer.PTY_SLAVE_MAJORS:
        return True
    return bool(PTY_PATH.match(str(header.get("device") or "")))


def refuse_or_mark(rows, *, allow: bool) -> dict:
    synthetic = any(r.get("t") == "header" and is_synthetic(r) for r in rows)
    if not synthetic:
        return {}
    if not allow:
        raise SyntheticRefused("a synthetic (rehearsal) record: not for measurement (give --allow-synthetic to read "
                               "it for a rehearsal; the output is then marked not_for_judgement)")
    return dict(OUTPUT_MARKS)


def bench_opener(opener, bench: dict):
    """Wrap a link opener so a rehearsal link uses only the bench's pty: the opened fd itself (fstat, not the path
    checked earlier) must be that character device, else it is closed and the open fails -- a path retargeted
    between the check and an open or a reopen never reaches another device (Codex rehearsal review P1)."""
    want = bench.get("pts_rdev")

    def open_checked(path, baud=None, **kw):
        fd = opener(path, baud, **kw)  # kw: the link's before_config (the executor lock) passes through
        st = os.fstat(fd)
        if not stat.S_ISCHR(st.st_mode) or st.st_rdev != want:
            os.close(fd)
            raise OSError(f"{path} opened a device that is not the bench's pty (rdev {st.st_rdev} != {want})")
        # the same pts number reused by another pty after the model went: the model must still hold its master
        why = model_peer.model_alive_refusal(path, bench)
        if why is not None:
            os.close(fd)
            raise OSError(f"{path}: the bench's model is not on this pty ({why})")
        return fd

    return open_checked

