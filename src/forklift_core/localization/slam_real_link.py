"""The real-mode SLAM link's frames (docs/design/2026-11-25-real-sources-pr7.md 6.1, 2 -- 'the link frames', defined
here once).

The same framing as ``slam_link`` (a 4-byte big-endian length, a JSON header line, an optional payload). The executor
sends ``hello`` (the session start), ``seed`` (an operator seed), ``cancel`` (a seed whose inputs broke) and ``scan``;
the bridge sends ``reply`` (one per scan sent) and ``state`` (a start or seed's progress -- never a scan reply).

- ``hello{session_epoch, map_id, start_id, start_pose, deadline_ns}`` and ``seed{seed_id, epoch, start_pose,
  odom_snapshot, boot, deadline_ns}``: ``start_pose`` is map <- base_link (the instance's ``map_start_pose``);
  ``deadline_ns`` the host-monotonic instant after which the bridge sends no end state for that start check (the
  executor alone judges the deadline).
- ``cancel{seed_id, reason}``.
- ``scan``: ``slam_link``'s scan with ``stamp_ns`` (integer host-monotonic ns of the first beam estimate; the bridge
  stamps ROS messages with it, and the session's stamps strictly increase).
- ``state{id, kind, reason}``: ``kind`` one of ``seeding``, ``ready_for_scans``, ``ready``, ``not_localized``, ``void``,
  ``fault`` (the bridge cannot go on: the connection ends after it).
"""

from __future__ import annotations

import json
from dataclasses import dataclass

import numpy as np

from forklift_core.localization import slam_link

STATE_KINDS = ("seeding", "ready_for_scans", "ready", "not_localized", "void", "fault")
END_KINDS = ("ready", "not_localized", "void", "fault")


@dataclass(frozen=True)
class Hello:
    session_epoch: int
    map_id: str
    start_id: int
    start_pose: tuple
    deadline_ns: int


@dataclass(frozen=True)
class Seed:
    seed_id: int
    epoch: int
    start_pose: tuple
    odom_snapshot: tuple
    boot: int
    deadline_ns: int


@dataclass(frozen=True)
class Cancel:
    seed_id: int
    reason: str


@dataclass(frozen=True)
class State:
    id: int
    kind: str
    reason: str | None = None

    def __post_init__(self) -> None:
        if self.kind not in STATE_KINDS:
            raise ValueError(f"unknown state kind {self.kind!r}")


@dataclass(frozen=True)
class RealScan:
    scan: slam_link.Scan
    stamp_ns: int


def _pose(v, n: int = 3) -> tuple:
    out = tuple(float(x) for x in v)
    if len(out) != n or not all(np.isfinite(out)):
        raise ValueError("a pose of finite numbers expected")
    return out


def _int(v, name: str) -> int:
    if isinstance(v, bool) or not isinstance(v, int):
        raise ValueError(f"{name}: an integer expected")
    return v


def encode_hello(h: Hello) -> bytes:
    return slam_link._frame({"type": "hello", "session_epoch": int(h.session_epoch), "map_id": str(h.map_id),  # noqa: SLF001
                             "start_id": int(h.start_id), "start_pose": list(_pose(h.start_pose)),
                             "deadline_ns": int(h.deadline_ns)})


def encode_seed(s: Seed) -> bytes:
    return slam_link._frame({"type": "seed", "seed_id": int(s.seed_id), "epoch": int(s.epoch),  # noqa: SLF001
                             "start_pose": list(_pose(s.start_pose)), "odom_snapshot": list(_pose(s.odom_snapshot)),
                             "boot": int(s.boot), "deadline_ns": int(s.deadline_ns)})


def encode_cancel(c: Cancel) -> bytes:
    return slam_link._frame({"type": "cancel", "seed_id": int(c.seed_id), "reason": str(c.reason)})  # noqa: SLF001


def encode_state(s: State) -> bytes:
    return slam_link._frame({"type": "state", "id": int(s.id), "kind": s.kind,  # noqa: SLF001
                             **({"reason": str(s.reason)} if s.reason is not None else {})})


def encode_scan(scan: slam_link.Scan, stamp_ns: int) -> bytes:
    """``slam_link``'s scan frame with the integer stamp added (its float ``stamp_s`` is ``stamp_ns`` / 1e9)."""
    body = slam_link.encode_scan(scan)[4:]
    line, _, payload = body.partition(b"\n")
    header = json.loads(line)
    header["stamp_ns"] = _int(stamp_ns, "stamp_ns")
    return slam_link._frame(header, payload)  # noqa: SLF001


def decode(body: bytes):
    """One frame's object: ``Hello``, ``Seed``, ``Cancel``, ``State``, ``RealScan`` or a ``slam_link.Reply``. Anything
    else, or a field of the wrong kind, raises ``ValueError``."""
    line = body.partition(b"\n")[0]
    try:
        header = json.loads(line)
    except json.JSONDecodeError as exc:
        raise ValueError(f"not a frame header: {exc}") from None
    kind = header.get("type")
    try:
        if kind == "hello":
            return Hello(_int(header["session_epoch"], "session_epoch"), str(header["map_id"]),
                         _int(header["start_id"], "start_id"), _pose(header["start_pose"]),
                         _int(header["deadline_ns"], "deadline_ns"))
        if kind == "seed":
            return Seed(_int(header["seed_id"], "seed_id"), _int(header["epoch"], "epoch"),
                        _pose(header["start_pose"]), _pose(header["odom_snapshot"]), _int(header["boot"], "boot"),
                        _int(header["deadline_ns"], "deadline_ns"))
        if kind == "cancel":
            return Cancel(_int(header["seed_id"], "seed_id"), str(header["reason"]))
        if kind == "state":
            return State(_int(header["id"], "id"), str(header["kind"]), header.get("reason"))
        if kind == "scan":
            scan = slam_link.decode_scan(body)
            return RealScan(scan, _int(header["stamp_ns"], "stamp_ns"))
        if kind == "reply":
            return slam_link.decode_reply(body)
    except KeyError as exc:
        raise ValueError(f"{kind}: missing {exc}") from None
    raise ValueError(f"unknown frame type {kind!r}")


__all__ = ["Cancel", "END_KINDS", "Hello", "RealScan", "STATE_KINDS", "Seed", "State", "decode", "encode_cancel",
           "encode_hello", "encode_scan", "encode_seed", "encode_state"]
