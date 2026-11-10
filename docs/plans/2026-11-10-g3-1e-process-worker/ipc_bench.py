"""Local IPC micro-measurement for the 1e process-worker plan (docs/plans/2026-11-10-g3-1e-process-worker.md, 5).

A job dict the size of 1e-0's refresh job (1,600-beam scan arrays, a 200-pose path, a 240 x 240 bool support) goes to
a child over an inherited pipe; the child answers with a raw and a final 240 x 240 snapshot (uint8 state, float64
free stamps). Prints the main side's pickle + send time, the round trip and the reply's unpickle time.

    /home/light/anaconda3/bin/python -I ipc_bench.py
"""

import os
import pickle
import subprocess
import sys
import time
from multiprocessing.connection import Connection

import numpy as np

N = 240


def child(rfd: int, wfd: int) -> None:
    r, w = Connection(rfd, writable=False), Connection(wfd, readable=False)
    state = np.zeros((N, N), np.uint8)
    stamp = np.full((N, N), np.nan)
    while True:
        b = r.recv_bytes()
        if b == b"q":
            return
        pickle.loads(b)
        out = {"raw": (state, stamp), "snapshot": (state.copy(), stamp.copy()), "exit": None}
        w.send_bytes(pickle.dumps(out, protocol=5))


def main() -> None:
    a, b = os.pipe()
    c, d = os.pipe()
    p = subprocess.Popen([sys.executable, __file__, str(a), str(d)], pass_fds=(a, d))
    os.close(a)
    os.close(d)
    s, r = Connection(b, readable=False), Connection(c, writable=False)
    job = {"ranges": np.random.rand(1600), "self_hit": np.zeros(1600, bool), "limit": np.random.rand(1600),
           "pose": (1.0, 2.0, 0.3), "path": np.random.rand(200, 3), "support": np.zeros((N, N), bool)}
    trip, send = [], []
    bs = b""
    for _ in range(300):
        t0 = time.perf_counter()
        bs = pickle.dumps(job, protocol=5)
        s.send_bytes(bs)
        t1 = time.perf_counter()
        pickle.loads(r.recv_bytes())
        t2 = time.perf_counter()
        trip.append(t2 - t0)
        send.append(t1 - t0)
    s.send_bytes(b"q")
    p.wait()
    trip_ms = np.array(trip[20:]) * 1e3
    send_ms = np.array(send[20:]) * 1e3
    print(f"job bytes {len(bs)}; round trip ms mean {trip_ms.mean():.3f} p50 {np.median(trip_ms):.3f} "
          f"p99 {np.percentile(trip_ms, 99):.3f}; send side ms mean {send_ms.mean():.3f}")
    reply = {"raw": (np.zeros((N, N), np.uint8), np.full((N, N), np.nan)),
             "snapshot": (np.zeros((N, N), np.uint8), np.full((N, N), np.nan))}
    rb = pickle.dumps(reply, protocol=5)
    t = time.perf_counter()
    for _ in range(200):
        pickle.loads(rb)
    print(f"reply unpickle ms {(time.perf_counter() - t) / 200 * 1e3:.3f}; reply bytes {len(rb)}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        child(int(sys.argv[1]), int(sys.argv[2]))
    else:
        main()
