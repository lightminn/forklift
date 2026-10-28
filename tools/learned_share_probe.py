"""A3 P0 (docs/plans/2026-10-28-learned-worker-shared.md, section 3): one shared learned worker against four clients,
without Isaac, in one GPU job -- the pre-registered screening: the server's VRAM over an own worker (delta), the round
trip of four synchronised clients against one (ABAB phases on the same server), the replies against a serial
reference, the server's CPU and RSS, and a broken or slow client not ending the others.

    LEARNED_PY tools/learned_share_probe.py --onnx BEST.onnx --frames FRAMES.npy --out DIR \
        [--locks serial,concurrent] [--pairs 4] [--phase-s 90] [--rate 2.0] [--threads 4]

Run it with the learned venv's python (onnxruntime-gpu) inside a Slurm job holding one shard. FRAMES.npy is an
(N, 480, 640, 3) uint8 stack of recorded far-recognition RGB frames.
"""
from __future__ import annotations

import argparse
import io
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

from forklift_learned import worker as W

CONF, IOU, MAX_DET = 0.05, 0.6, 8
PROVIDERS = "CUDAExecutionProvider,CPUExecutionProvider"
# pre-registered (plan section 3, P0): a pair is left out when another process's GPU use in either of its phases is
# more than 30 % (and 5 points) away from the median of the phase means (a median: a few loaded phases must not move
# the reference and drop every pair); fewer than 3 pairs left -> undecided
EXCLUDE_REL, EXCLUDE_ABS, MIN_PAIRS = 0.30, 5.0, 3
DELTA_MAX_MIB, P95_RATIO_MAX = 693.0, 2.0
SCORE_TOL, MASK_IOU_MIN = 1e-3, 0.99


class Scene:
    def __init__(self, rgb):
        self.rgb = rgb


def gpu_index() -> str:
    """The job's GPU as nvidia-smi inside the job numbers it. Under the job's device constraint nvidia-smi lists only
    the allocated GPU, as index 0 -- the node-wide SLURM_JOB_GPUS (1 for the second GPU) made every query fail with
    exit 6 in the first P0 run (12080). With one GPU listed, that one; else the node-wide number."""
    try:
        listed = subprocess.run(["nvidia-smi", "-L"], capture_output=True, text=True, timeout=10).stdout
        if len([ln for ln in listed.splitlines() if ln.startswith("GPU ")]) == 1:
            return "0"
    except (OSError, subprocess.SubprocessError):
        pass
    gpus = [g for g in (os.environ.get("SLURM_JOB_GPUS") or "0").split(",") if g.strip()]
    return gpus[0]


class Sampler(threading.Thread):
    """Every ``period_s``: each GPU process's memory and SM use (nvidia-smi pmon), and the watched pids' RSS and CPU
    time. Rows go to ``rows`` with the phase label current at the time."""

    def __init__(self, gpu: str, period_s: float = 1.0):
        super().__init__(daemon=True)
        self.gpu, self.period_s, self.rows, self.phase, self.pids = gpu, period_s, [], "start", {}
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            t = time.time()
            row = {"t": t, "phase": self.phase, "apps": {}, "sm": {}, "procs": {}}
            try:
                p = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits",
                                    "-i", self.gpu], capture_output=True, text=True, timeout=10)
                if p.returncode != 0:  # a failed query is a missing sample, not an empty GPU (Codex)
                    raise ValueError(f"compute-apps exit {p.returncode}")
                for line in p.stdout.strip().splitlines():
                    pid, mib = [x.strip() for x in line.split(",")]
                    row["apps"][pid] = int(mib)
                p = subprocess.run(["nvidia-smi", "pmon", "-c", "1", "-s", "u", "-i", self.gpu],
                                   capture_output=True, text=True, timeout=10)
                if p.returncode != 0 or "#" not in p.stdout:  # pmon always prints its header
                    raise ValueError(f"pmon exit {p.returncode}")
                for line in p.stdout.splitlines():
                    parts = line.split()
                    if parts and parts[0].isdigit() and len(parts) > 3:
                        if parts[3] == "-":
                            # "-" is no value (unsupported or not sampled), not zero: the sample's other-process load
                            # is unknown and its phase may lose the pair (Codex)
                            row["sm_unknown"] = row.get("sm_unknown", []) + [parts[1]]
                        else:
                            row["sm"][parts[1]] = float(parts[3])
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                row["error"] = repr(exc)
                row["apps"], row["sm"] = {}, None
            for name, pid in self.pids.items():
                try:
                    stat = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
                    status = Path(f"/proc/{pid}/status").read_text()
                    rss = {k: int(v.split()[0]) for k, v in (ln.split(":", 1) for ln in status.splitlines())
                           if k in ("VmRSS", "VmHWM")}
                    row["procs"][name] = {"cpu_s": (int(stat[11]) + int(stat[12])) / os.sysconf("SC_CLK_TCK"),
                                          "rss_kib": rss.get("VmRSS"), "hwm_kib": rss.get("VmHWM"), "pid": pid}
                except (OSError, ValueError, IndexError):
                    pass
            self.rows.append(row)
            self.stop.wait(max(0.0, self.period_s - (time.time() - t)))


def reference(onnx: str, frames: np.ndarray, threads: int, sampler: Sampler, py: str) -> tuple[list, float]:
    """The serial reference replies (an own worker over the pipe, as a session runs it today) and that worker's VRAM
    peak (the plan's 514 MiB, measured here in the same job)."""
    w = W.WorkerProposer(py, onnx, conf=CONF, providers=PROVIDERS, threads=threads, device_id=0)
    sampler.pids["own"] = w.proc.pid
    sampler.phase = "own"
    try:
        ref = [w.propose(Scene(f)) for f in frames]
        for f in frames[:40]:  # a little more of the steady state for the VRAM peak
            w.propose(Scene(f))
    finally:
        pid = str(w.proc.pid)
        w.close()
    time.sleep(1.0)
    seen = [r["apps"][pid] for r in sampler.rows if r["phase"] == "own" and pid in r["apps"]]
    peak = max(seen) if seen else 0  # 0: never seen -- judge() then leaves delta undecided
    sampler.pids.pop("own", None)
    return ref, float(peak)


def same(props, ref) -> str | None:
    """None when a reply matches the serial reference: the same ids in order, scores within SCORE_TOL, each mask's
    IoU with the reference's at least MASK_IOU_MIN; else why not."""
    if [p.proposal_id for p in props] != [p.proposal_id for p in ref]:
        return f"ids {[p.proposal_id for p in props]} vs {[p.proposal_id for p in ref]}"
    for a, b in zip(props, ref):
        if abs(a.score - b.score) > SCORE_TOL:
            return f"score {a.score} vs {b.score}"
        union = np.logical_or(a.mask, b.mask).sum()
        if union and np.logical_and(a.mask, b.mask).sum() / union < MASK_IOU_MIN:
            return "mask IoU below the floor"
    return None


def client_loop(path, expected, frames, ref, rate, until, tick0, k, out):
    """One client: a frame every 1/rate s on a common clock (all clients fire on the same ticks -- the synchronised
    worst case), cycling through the frames from an offset of its own. A client that cannot connect leaves an error
    row, so a phase that ran with fewer clients does not pass (Codex)."""
    try:
        c = W.SocketProposer(path, expected, timeout_s=10.0)
    except RuntimeError as exc:
        out.append({"client": k, "n": None, "frame": None, "rtt_ms": None, "error": f"connect: {exc}",
                    "mismatch": None})
        return
    n = 0
    try:
        while True:
            t_due = tick0 + n / rate
            if t_due >= until:
                return
            time.sleep(max(0.0, t_due - time.monotonic()))
            i = (17 * k + n) % len(frames)
            t0 = time.monotonic()
            try:
                props = c.propose(Scene(frames[i]))
                err = None
            except RuntimeError as exc:
                props, err = None, str(exc)
            rtt = 1e3 * (time.monotonic() - t0)
            out.append({"client": k, "n": n, "frame": i, "rtt_ms": rtt, "infer_ms": c.last_infer_ms,
                        "queue_ms": c.last_queue_ms, "late_ms": 1e3 * (t0 - t_due), "error": err,
                        "mismatch": None if props is None else same(props, ref[i])})
            n += 1
    finally:
        c.close()


def expected_calls(rate, seconds) -> int:
    """The calls one client makes in a phase: ticks at 0, 1/rate, ... below ``seconds``."""
    return int(np.ceil(seconds * rate - 1e-9))


def phase(path, expected, frames, ref, clients, rate, seconds, sampler, label):
    sampler.phase = label
    out, tick0 = [], time.monotonic() + 0.5
    threads = [threading.Thread(target=client_loop, args=(path, expected, frames, ref, rate, tick0 + seconds, tick0, k, out))
               for k in range(clients)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(seconds + 60)
    sampler.phase = label + "_end"  # the next phase's label comes only with its start
    return {"label": label, "rows": out, "clients": clients, "expected": clients * expected_calls(rate, seconds),
            "hung": sum(1 for t in threads if t.is_alive())}


def faults(path, expected, frames, ref) -> dict:
    """A client that sends half a frame and vanishes, and one that never reads its replies, beside a normal client:
    the normal one must go on (plan section 3, P0 faults)."""
    raw = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    raw.connect(path)
    f = raw.makefile("rwb")
    W._send(f, json.dumps(expected).encode())
    W._recv(f)
    f.write((10 ** 6).to_bytes(8, "little") + b"x" * 1000)
    f.flush()
    f.close()  # the file object holds the descriptor too: close both, or the server never sees the end (Codex)
    raw.close()
    slow = W.SocketProposer(path, expected)
    buf = io.BytesIO()
    np.save(buf, frames[0])

    def send_unread():  # its own thread: once the server blocks on the unread reply, a further frame blocks the sender
        try:
            for _ in range(3):
                W._send(slow.f, buf.getvalue())
        except OSError:
            pass

    threading.Thread(target=send_unread, daemon=True).start()
    time.sleep(1.0)
    normal = W.SocketProposer(path, expected)
    errors = 0
    for i in range(20):
        try:
            if same(normal.propose(Scene(frames[i % len(frames)])), ref[i % len(frames)]) is not None:
                errors += 1
        except RuntimeError:
            errors += 1
    slow.close()
    normal.close()
    return {"normal_client_errors": errors, "calls": 20}


def pct(v, q):
    return float(np.percentile(v, q)) if len(v) else float("nan")


def _is_ab(label, mode):
    rest = label[len(mode) + 1:] if label.startswith(mode + "_") else ""
    return rest[:1] in ("A", "B") and rest[1:].isdigit()


def judge(mode, sampler_rows, own_peak, server_pid, phases):
    """The pre-registered P0 verdict for one lock mode. A measurement that is missing makes its verdict undecided
    (None), never a pass (Codex): no VRAM sample of the server or the own worker, a phase without SM samples (its
    pair is left out), fewer than two CPU samples in every B phase, no RSS. The server's cores are its CPU time over
    the wall time between the same samples (Codex)."""
    spid = str(server_pid)
    mine = [r for r in sampler_rows if r["phase"].startswith(mode + "_")]
    seen = [r["apps"][spid] for r in mine if spid in r["apps"]]
    peak = max(seen) if seen else None
    delta = (peak - own_peak) if peak is not None and own_peak else None
    ours = {spid}
    per_phase = {}
    for r in mine:
        # a sample with another process's load unknown ("-") or a failed query does not count (Codex)
        unknown = [q for q in r.get("sm_unknown", []) if q not in ours]
        if _is_ab(r["phase"], mode) and r["sm"] is not None and "error" not in r and not unknown:
            per_phase.setdefault(r["phase"], []).append(sum(v for p, v in r["sm"].items() if p not in ours))
    means = {k: float(np.mean(v)) for k, v in per_phase.items() if v}
    overall = float(np.median(list(means.values()))) if means else None

    def off(label):
        if overall is None or label not in means:
            return True  # no load record for the phase: the pair cannot be shown clean
        return abs(means[label] - overall) > max(EXCLUDE_REL * overall, EXCLUDE_ABS)

    pairs, kept = [], []
    for j, (a, b) in enumerate(phases):
        ra = [x["rtt_ms"] for x in a["rows"] if x["error"] is None]
        rb = [x["rtt_ms"] for x in b["rows"] if x["error"] is None]
        row = {"pair": j, "a_p95": pct(ra, 95), "b_p95": pct(rb, 95), "a_n": len(ra), "b_n": len(rb),
               "other_sm_a": means.get(a["label"]), "other_sm_b": means.get(b["label"]),
               "excluded": off(a["label"]) or off(b["label"])}
        row["ratio"] = row["b_p95"] / row["a_p95"] if row["a_p95"] else float("nan")
        pairs.append(row)
        if not row["excluded"]:
            kept.append(row)
    sides = [side for ph in phases for side in ph]
    all_rows = [x for side in sides for x in side["rows"]]
    mismatches = sum(1 for x in all_rows if x["mismatch"])
    errors = sum(1 for x in all_rows if x["error"])
    short = sum(max(0, side.get("expected", 0) - sum(1 for x in side["rows"] if x["n"] is not None)) for side in sides)
    hung = sum(side.get("hung", 0) for side in sides)
    a_all = [x["rtt_ms"] for a, _ in phases for x in a["rows"] if x["error"] is None]
    b_all = [x["rtt_ms"] for _, b in phases for x in b["rows"] if x["error"] is None]
    cpu_s = wall_s = 0.0
    for lab in sorted({r["phase"] for r in mine if _is_ab(r["phase"], mode) and r["phase"][len(mode) + 1] == "B"}):
        seg = [(r["t"], r["procs"]["server"]["cpu_s"]) for r in mine if r["phase"] == lab and "server" in r["procs"]]
        if len(seg) > 1:
            cpu_s += seg[-1][1] - seg[0][1]
            wall_s += seg[-1][0] - seg[0][0]
    cores = cpu_s / wall_s if wall_s > 0 else None
    hwm = [r["procs"]["server"].get("hwm_kib") for r in mine if "server" in r["procs"]]
    hwm = [h for h in hwm if h]
    rss_gib = max(hwm) / 2 ** 20 if hwm else None
    verdict = {
        "delta_ok": None if delta is None else delta <= DELTA_MAX_MIB,
        "latency_ok": None if len(kept) < MIN_PAIRS else all(r["ratio"] <= P95_RATIO_MAX for r in kept),
        "correct_ok": mismatches == 0 and errors == 0 and short == 0 and hung == 0,
        # plan 1: 4 shared + 3 own on one node -- CPU 18 + C <= 20, RAM 24 + R + 18.5 <= 56.6
        "cpu_ok": None if cores is None else 18 + int(np.ceil(cores)) <= 20,
        "ram_ok": None if rss_gib is None else 24 + rss_gib + 18.5 <= 56.6,
    }
    verdict["pass"] = all(v is True for v in verdict.values())
    return {"server_vram_peak_mib": peak, "own_worker_peak_mib": own_peak, "delta_mib": delta,
            "rtt_a": {"p50": pct(a_all, 50), "p95": pct(a_all, 95), "n": len(a_all)},
            "rtt_b": {"p50": pct(b_all, 50), "p95": pct(b_all, 95), "p99": pct(b_all, 99), "n": len(b_all)},
            "pairs": pairs, "kept_pairs": len(kept), "other_sm_median": overall, "mismatches": mismatches,
            "errors": errors, "short_calls": short, "hung_clients": hung, "server_cores_b": cores,
            "server_rss_hwm_gib": rss_gib, "verdict": verdict}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--frames", required=True)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--locks", default="serial,concurrent")
    ap.add_argument("--pairs", type=int, default=4)
    ap.add_argument("--phase-s", type=float, default=90.0)
    ap.add_argument("--rate", type=float, default=2.0, help="requests a second a client (5 Hz sim x pace 0.4)")
    ap.add_argument("--threads", type=int, default=4)
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    frames = np.load(a.frames)
    py = sys.executable
    sampler = Sampler(gpu_index())
    sampler.start()
    ref, own_peak = reference(a.onnx, frames, a.threads, sampler, py)
    summary = {"own_worker_peak_mib": own_peak, "frames": int(len(frames)), "rate": a.rate, "phase_s": a.phase_s,
               "pairs": a.pairs, "gpu": gpu_index(), "modes": {}}
    sock = f"/tmp/flw_{os.environ.get('SLURM_JOB_ID', os.getpid())}.sock"
    for lock in a.locks.split(","):
        srv = subprocess.Popen([py, "-m", "forklift_learned.worker", "--serve-socket", sock, "--lock", lock, a.onnx,
                                str(CONF), PROVIDERS, str(a.threads), "0"], stdout=subprocess.PIPE, text=True)
        line = srv.stdout.readline()
        while line and not line.startswith("READY "):
            line = srv.stdout.readline()
        if not line:
            raise SystemExit(f"the server did not come up (exit {srv.poll()})")
        info = json.loads(line[len("READY "):])
        sampler.pids["server"] = srv.pid
        phases = []
        for j in range(a.pairs):
            pa = phase(sock, info, frames, ref, 1, a.rate, a.phase_s, sampler, f"{lock}_A{j}")
            pb = phase(sock, info, frames, ref, 4, a.rate, a.phase_s, sampler, f"{lock}_B{j}")
            phases.append((pa, pb))
        sampler.phase = f"{lock}_faults"
        fault = faults(sock, info, frames, ref)
        time.sleep(2.0)
        result = judge(lock, sampler.rows, own_peak, srv.pid, phases)
        result["faults"] = fault
        result["verdict"]["faults_ok"] = fault["normal_client_errors"] == 0
        result["verdict"]["pass"] = result["verdict"]["pass"] and result["verdict"]["faults_ok"]
        summary["modes"][lock] = result
        with open(a.out / f"calls_{lock}.jsonl", "w") as f:
            for pa, pb in phases:
                for side in (pa, pb):
                    for x in side["rows"]:
                        f.write(json.dumps({"phase": side["label"], **x}) + "\n")
        srv.send_signal(signal.SIGTERM)
        try:
            srv.wait(10)
        except subprocess.TimeoutExpired:
            srv.kill()
        sampler.pids.pop("server", None)
    sampler.stop.set()
    sampler.join(5)
    with open(a.out / "samples.jsonl", "w") as f:
        f.write("".join(json.dumps(r) + "\n" for r in sampler.rows))
    (a.out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    for lock, r in summary["modes"].items():
        def num(v, fmt):  # an undecided measurement is None (Codex)
            return "n/a" if v is None else format(v, fmt)

        print(lock, json.dumps(r["verdict"]), f"delta {num(r['delta_mib'], '.0f')} MiB",
              f"B p95 {num(r['rtt_b']['p95'], '.1f')} ms", f"A p95 {num(r['rtt_a']['p95'], '.1f')} ms",
              f"cores {num(r['server_cores_b'], '.2f')}", f"rss {num(r['server_rss_hwm_gib'], '.2f')} GiB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
