#!/usr/bin/env python3
"""PR 7c-1a on the real SLAM stack (docs/design/2026-11-25-real-sources-pr7.md 6.1, 9): drive
``forklift_ros.real_slam_bridge`` + slam_toolbox 2.8.5 through one session with frames, on a survey bundle, scans
ray-cast from its grid at the truck's (still) pose.

Scenarios, in one session, each start with the operational 20 s deadline: hello at A -> ready, three ordinary scans;
seed B -> ready; a seed at a wrong pose with scans taken at B -> not_localized; seed B again -> ready; seed A cancelled
right after start scan 5 is sent (its /pose pending) -> void within 5 s; then seed A -> ready. Each start's states, replies, latencies and the reply corrections' distance from the expected pose go to
OUT/stack_check.json; exit 0 when every scenario ends as expected.

    python3 tools/real_slam_stack_check.py SOCKET BUNDLE_DIR OUT_DIR
"""

from __future__ import annotations

import json
import math
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))
from slam_instance_premise import free_at, raycast  # noqa: E402

from forklift_core.localization import slam_link, slam_map  # noqa: E402
from forklift_core.localization import slam_real_link as rl  # noqa: E402

A = (-1.7529200315475464, 0.0, 0.0)  # random_m registry start


class Client:
    def __init__(self, path: str) -> None:
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        deadline = time.monotonic() + 120
        while True:
            try:
                self.sock.connect(path)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.5)
        self.buf = b""
        self.sid = 0
        self.last_ns = 0

    def send(self, frame: bytes) -> None:
        self.sock.sendall(frame)

    def recv(self, timeout: float):
        self.sock.settimeout(timeout)
        while len(self.buf) < 4 or len(self.buf) < 4 + struct.unpack(">I", self.buf[:4])[0]:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("bridge closed")
            self.buf += chunk
        (n,) = struct.unpack(">I", self.buf[:4])
        body, self.buf = self.buf[4:4 + n], self.buf[4 + n:]
        return rl.decode(body)

    def wait_state(self, ident: int, kinds, timeout: float) -> rl.State:
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise TimeoutError(f"no {kinds} for {ident} within {timeout} s")
            obj = self.recv(left)
            if time.monotonic() > deadline:  # read again after the frame (Codex 7c-1a round 2 P3-2)
                raise TimeoutError(f"no {kinds} for {ident} within {timeout} s")
            if isinstance(obj, rl.State) and obj.id == ident and obj.kind in kinds:
                return obj
            if isinstance(obj, rl.State) and obj.kind == "fault":
                raise RuntimeError(f"bridge fault: {obj.reason}")

    def scan(self, grid, pose, timeout: float = 5.0):
        self.sid += 1
        ns = max(time.monotonic_ns(), self.last_ns + 1)
        self.last_ns = ns
        s = slam_link.Scan(self.sid, ns / 1e9, (0.0, 0.0, 0.0), raycast(grid, pose).astype(np.float32), -math.pi,
                           2 * math.pi / 360, 0.1, 12.0)
        t0 = time.monotonic()
        self.send(rl.encode_scan(s, ns))
        while True:
            obj = self.recv(timeout)
            if isinstance(obj, slam_link.Reply):
                return obj, time.monotonic() - t0
            if isinstance(obj, rl.State) and obj.kind == "fault":
                raise RuntimeError(f"bridge fault: {obj.reason}")


def start(cl: Client, grid, frame: bytes, ident: int, scan_pose, expect_pose, *, cancel_at=None) -> dict:
    """One start or seed (its frame carries the operational 20 s deadline). ``cancel_at``: send that start scan and
    cancel at once, while its /pose is pending -- void must come within the executor's 5 s."""
    t0 = time.monotonic()
    cl.send(frame)
    cl.wait_state(ident, ("seeding",), 10)
    cl.wait_state(ident, ("ready_for_scans",), 20)
    up = time.monotonic() - t0
    replies = []
    for i in range(1, 8):
        if cancel_at is not None and i == cancel_at:
            cl.sid += 1
            ns = max(time.monotonic_ns(), cl.last_ns + 1)
            cl.last_ns = ns
            s = slam_link.Scan(cl.sid, ns / 1e9, (0.0, 0.0, 0.0), raycast(grid, scan_pose).astype(np.float32),
                               -math.pi, 2 * math.pi / 360, 0.1, 12.0)
            cl.send(rl.encode_scan(s, ns))
            tc = time.monotonic()
            cl.send(rl.encode_cancel(rl.Cancel(ident, "test")))
            st = cl.wait_state(ident, ("void",), 5.0)
            return {"end": st.kind, "bring_up_s": round(up, 2), "void_s": round(time.monotonic() - tc, 3),
                    "replies": replies}
        r, dt = cl.scan(grid, scan_pose)
        err = None
        if r.status == "processed":
            pose = slam_map_pose(r.map_from_odom)
            err = round(math.hypot(pose[0] - expect_pose[0], pose[1] - expect_pose[1]), 4)
        replies.append({"i": i, "status": r.status, "latency_s": round(dt, 3), "error_m": err})
    st = cl.wait_state(ident, ("ready", "not_localized"), 10)
    return {"end": st.kind, "reason": st.reason, "bring_up_s": round(up, 2), "total_s": round(time.monotonic() - t0, 2),
            "replies": replies}


def slam_map_pose(map_from_odom):
    return tuple(float(v) for v in map_from_odom)  # odom is identity here: map <- base = map <- odom


def main(argv) -> int:
    sock, bundle_dir, out = argv[1], Path(argv[2]), Path(argv[3])
    grid = slam_map.load_bundle(bundle_dir).grid
    b = next(p for p in ((A[0] + 0.3, A[1], 0.1), (A[0] + 0.4, A[1] + 0.2, 0.15)) if free_at(grid, p))
    wrong = (b[0] + 1.5, b[1] + 0.5, b[2] + 0.8)
    cl = Client(sock)
    def deadline():
        return time.monotonic_ns() + 20 * 10**9  # the operational seed/start deadline (design 6.1, 2)
    res: dict = {"a": A, "b": b, "wrong": wrong}
    bundle_id = slam_map.load_bundle(bundle_dir).map_id
    ok = True
    try:
        res["hello_a"] = start(cl, grid, rl.encode_hello(rl.Hello(1, bundle_id, 1, A, deadline())), 1, A, A)
        res["ready_scans"] = [{"status": r.status, "latency_s": round(dt, 3),
                               "error_m": round(math.hypot(r.map_from_odom[0] - A[0], r.map_from_odom[1] - A[1]), 4)}
                              for r, dt in (cl.scan(grid, A) for _ in range(3))]
        seed = lambda i, pose: rl.encode_seed(rl.Seed(i, i, pose, (0.0, 0.0, 0.0), 1, deadline()))  # noqa: E731
        res["seed_b"] = start(cl, grid, seed(2, b), 2, b, b)
        res["seed_wrong"] = start(cl, grid, seed(3, wrong), 3, b, wrong)
        res["seed_b_again"] = start(cl, grid, seed(4, b), 4, b, b)
        res["seed_a_cancelled"] = start(cl, grid, seed(5, A), 5, A, A, cancel_at=5)  # scan 5's /pose pending
        res["seed_a"] = start(cl, grid, seed(6, A), 6, A, A)
        want = {"hello_a": "ready", "seed_b": "ready", "seed_wrong": "not_localized", "seed_b_again": "ready",
                "seed_a_cancelled": "void", "seed_a": "ready"}
        res["ends"] = {k: res[k]["end"] for k in want}
        ok = res["ends"] == want and all(r["status"] == "processed" for r in res["ready_scans"])
    except Exception as exc:  # noqa: BLE001 -- recorded
        res["error"] = f"{type(exc).__name__}: {exc}"
        ok = False
    res["ok"] = ok
    (out / "stack_check.json").write_text(json.dumps(res, indent=1, default=str))
    print(json.dumps({"ok": ok, "ends": res.get("ends"), "error": res.get("error")}))
    cl.sock.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
