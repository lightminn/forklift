import os, pickle, subprocess, sys, time
from multiprocessing.connection import Connection
import numpy as np
N = 240
def child(rfd, wfd):
    r, w = Connection(rfd, writable=False), Connection(wfd, readable=False)
    state = np.zeros((N, N), np.uint8); stamp = np.full((N, N), np.nan)
    while True:
        b = r.recv_bytes()
        if b == b"q": return
        job = pickle.loads(b)
        out = {"raw": (state, stamp), "snapshot": (state.copy(), stamp.copy()), "exit": None}
        w.send_bytes(pickle.dumps(out, protocol=5))
if len(sys.argv) > 1:
    child(int(sys.argv[1]), int(sys.argv[2])); sys.exit()
a, b = os.pipe(); c, d = os.pipe()
p = subprocess.Popen([sys.executable, __file__, str(a), str(d)], pass_fds=(a, d))
os.close(a); os.close(d)
s, r = Connection(b, readable=False), Connection(c, writable=False)
scan = {"ranges": np.random.rand(1600), "self_hit": np.zeros(1600, bool), "limit": np.random.rand(1600),
        "pose": (1.0, 2.0, 0.3), "path": np.random.rand(200, 3), "support": np.zeros((N, N), bool)}
ts = []; tsend = []; trecv = []
for i in range(300):
    t0 = time.perf_counter(); bs = pickle.dumps(scan, protocol=5); s.send_bytes(bs); t1 = time.perf_counter()
    out = pickle.loads(r.recv_bytes()); t2 = time.perf_counter()
    ts.append(t2 - t0); tsend.append(t1 - t0)
s.send_bytes(b"q"); p.wait()
ts = np.array(ts[20:]) * 1e3; tsend = np.array(tsend[20:]) * 1e3
print(f"job bytes {len(bs)} reply ~{2*N*N*9} ; roundtrip ms mean {ts.mean():.3f} p50 {np.median(ts):.3f} p99 {np.percentile(ts,99):.3f}; send-side ms mean {tsend.mean():.3f}")
# main-side cost only: pickle.loads of reply
rep = {"raw": (np.zeros((N,N),np.uint8), np.full((N,N),np.nan)), "snapshot": (np.zeros((N,N),np.uint8), np.full((N,N),np.nan))}
rb = pickle.dumps(rep, protocol=5)
t=time.perf_counter()
for _ in range(200): pickle.loads(rb)
print(f"reply unpickle ms {(time.perf_counter()-t)/200*1e3:.3f}; reply bytes {len(rb)}")
