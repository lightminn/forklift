"""Several Isaac runs in one Slurm job (docs/plans/2026-10-24-ws1-isaac-bundle.md): a run list, N slots, staggered starts.

    python3 tools/isaac_bundle.py --list RUNS.jsonl --out OUT [--slots N] [--shards N] [--stagger-s 60]
        [--session-vram-mib 3950] [--run-timeout-s S] [--grace-s 110] [--retries 1] [--pace-floor R]
        [--pace-gap-p99-s S] [--no-vram] [--no-cleanup] [--allow-multi-gpu]

A run is one JSON line: {"name": .., "argv": [..], "env": {..}, "cwd": .., "scan": [globs], "retry_move": [paths],
"pace_log": glob} (all but name and argv optional). Runs start in
list order whenever

- a slot is free: at most --slots runs at once, and never more slots than the job's shards (the rule: as many shards
  as Isaac sessions running in the job);
- the previous start is at least --stagger-s old (a live session reaches its steady GPU memory 31-40 s after it
  starts, measured on ws1);
- the GPU memory allows one more session, on a sample taken after the latest start: the job's runs, each counted at
  least --session-vram-mib (a run still warming up holds its whole share), plus one more fit in --shards x
  --shard-mib, and the GPU's free memory, less what the job's warming runs will still take, holds one more session.
  Shards are a scheduling count, not a partition: the second check keeps a start from landing on a GPU that other
  jobs have filled. When nvidia-smi cannot be read the start waits, for at most --vram-unreadable-s, then goes on by
  the slots alone and says so. A run alone always starts.

Each run gets SESSION_ID=<job>s<index> (the launchers name their docker containers with it), BRIDGE_CPU (a CPU of the
job, a different one per slot, for the SLAM bridge container), PORT (a free port, 9000-9499, from an offset by job),
BUNDLE_SLOT, BUNDLE_JOB, BUNDLE_RUN and BUNDLE_TOKEN. The list may set PORT (each value once) but none of the others.
Each run is its own process group with its log in OUT/runs/<name>.log; one that fails leaves the others running.

A run past --run-timeout-s, and every run when the bundle gets TERM/INT, is ended like Slurm's --signal=B:TERM: TERM
to its top process only (the launchers end their supervisor, runner and bridge in that order), then KILL after
--grace-s to every process carrying its BUNDLE_TOKEN -- found through /proc/<pid>/environ, because `timeout` and
similar tools move their child into a process group of its own. Processes a finished run left behind get TERM, then
KILL. Docker containers named slam_bridge_<SESSION_ID>_* (outside the run's processes and the job's cgroup) are
removed.

A run that failed for the infrastructure -- an OOM kill in the job's cgroup while it ran (and it did not exit 0),
or a GPU allocation failure in its log or in the files its "scan" globs name (`bfc_arena`, `OUT_OF_DEVICE`, CUDA out
of memory) -- is recorded as status "infra_failed" and, with --retries left, run again: the paths in its
"retry_move" are first renamed to <path>.attempt<n> (so the launcher's output folder keeps its name), and "{attempt}"
in its argv, env, scan and pace_log is the attempt number. A failure of the run itself is never retried.

Real-time back-off (live runs): a run's "pace_log" glob names its live_log.jsonl (the newest match is read). Every
--pace-period-s the bundle takes, for each run older than --pace-warmup-s, simulated over wall time across the last
--pace-window-s of tick records and the 99th percentile tick gap. When the median of those ratios falls below
--pace-floor, or a run's gap p99 exceeds --pace-gap-p99-s, the bundle starts no new run until fewer than (slots - 1)
run -- one step back -- and waits a window before it steps back again. OUT/pace.jsonl keeps every evaluation.

Records: OUT/results.jsonl (a line per run), OUT/vram.jsonl (the job's GPU memory, per run, every --vram-period-s),
OUT/summary.json. A run's exit code is its top process's: the launcher has to end with the code of what it ran.
Exit 0 when every run exited 0, 1 otherwise, 2 for a refused configuration.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import secrets
import signal
import socket
import subprocess
import sys
import threading
import time
import zlib
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

SHARD_MIB = 4076  # 16,303 MiB / 4 shards per GPU on ws1 (gres.conf)
SESSION_VRAM_MIB = 3950  # a live session with the learned worker: 3,886-3,915 MiB measured, + ~40 MiB driver share
PORTS = (9000, 9500)
RESERVED_ENV = ("SESSION_ID", "BRIDGE_CPU", "BUNDLE_SLOT", "BUNDLE_JOB", "BUNDLE_RUN", "BUNDLE_TOKEN")
INFRA_PATTERNS = re.compile(rb"bfc_arena|BFCArena|OUT_OF_DEVICE|CUDA out of memory|CUDA_ERROR_OUT_OF_MEMORY")
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$")


@dataclass
class Run:
    name: str
    argv: list
    env: dict = field(default_factory=dict)
    cwd: str | None = None
    scan: list = field(default_factory=list)
    retry_move: list = field(default_factory=list)
    pace_log: str | None = None

    def at(self, attempt) -> Run:
        """This run with "{attempt}" filled in."""
        f = (lambda x: x.replace("{attempt}", str(attempt)))  # noqa: E731
        return Run(self.name, [f(a) for a in self.argv], {k: f(v) for k, v in self.env.items()}, self.cwd,
                   [f(g) for g in self.scan], [f(m) for m in self.retry_move],
                   f(self.pace_log) if self.pace_log else None)

    @property
    def retryable(self) -> bool:
        texts = [*self.argv, *self.env.values(), *self.scan, self.pace_log or ""]
        return bool(self.retry_move) or any("{attempt}" in t for t in texts)


def load_runs(path) -> list[Run]:
    """The run list: one JSON object per non-empty line; refuses unknown keys, bad names, duplicate names or ports
    and the environment names the bundle sets itself."""
    runs, names, ports = [], set(), set()
    for n, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.strip():
            continue
        obj = json.loads(line)
        if not isinstance(obj, dict):
            raise ValueError(f"line {n}: not an object")
        unknown = set(obj) - {"name", "argv", "env", "cwd", "scan", "retry_move", "pace_log"}
        if unknown:
            raise ValueError(f"line {n}: unknown keys {sorted(unknown)}")
        name = obj.get("name")
        if not isinstance(name, str) or not _NAME.match(name):
            raise ValueError(f"line {n}: bad name {name!r} (letters, digits, . _ -)")
        if name in names:
            raise ValueError(f"line {n}: the name {name} is used twice")
        names.add(name)
        argv = obj.get("argv")
        if not isinstance(argv, list) or not argv or not all(isinstance(a, str) for a in argv):
            raise ValueError(f"line {n}: argv must be a non-empty list of strings")
        env = obj.get("env", {})
        if not isinstance(env, dict) or not all(isinstance(k, str) and isinstance(v, str) for k, v in env.items()):
            raise ValueError(f"line {n}: env must map strings to strings")
        reserved = sorted(set(env) & set(RESERVED_ENV))
        if reserved:
            raise ValueError(f"line {n}: env sets {reserved}, which the bundle sets per run")
        if "PORT" in env:
            if not env["PORT"].isdigit() or env["PORT"] in ports:
                raise ValueError(f"line {n}: PORT {env['PORT']!r} is not a number or is used twice")
            ports.add(env["PORT"])
        cwd = obj.get("cwd")
        if cwd is not None and not isinstance(cwd, str):
            raise ValueError(f"line {n}: cwd must be a string")
        lists = {}
        for key in ("scan", "retry_move"):
            value = obj.get(key, [])
            if not isinstance(value, list) or not all(isinstance(x, str) for x in value):
                raise ValueError(f"line {n}: {key} must be a list of strings")
            lists[key] = value
        pace_log = obj.get("pace_log")
        if pace_log is not None and not isinstance(pace_log, str):
            raise ValueError(f"line {n}: pace_log must be a string")
        runs.append(Run(name=name, argv=argv, env=env, cwd=cwd, pace_log=pace_log, **lists))
    return runs


def visible_gpus(env) -> list[str]:
    """The GPUs the job sees: CUDA_VISIBLE_DEVICES, else SLURM_JOB_GPUS."""
    value = env.get("CUDA_VISIBLE_DEVICES") or env.get("SLURM_JOB_GPUS") or ""
    return [g.strip() for g in value.split(",") if g.strip()]


def _stat(pid):
    """(state, ppid, pgid) from /proc/<pid>/stat, or None."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()
        return fields[0], int(fields[1]), int(fields[2])
    except (OSError, ValueError, IndexError):
        return None


def alive(pid) -> bool:
    """A process that exists and is not a zombie."""
    st = _stat(pid)
    return st is not None and st[0] != "Z"


def _pids():
    return [int(d) for d in os.listdir("/proc") if d.isdigit()]


def token_pids(token) -> list[int]:
    """Live processes whose environment carries BUNDLE_TOKEN=<token> (the run's whole tree, whatever its groups)."""
    needle = f"BUNDLE_TOKEN={token}".encode()
    found = []
    for pid in _pids():
        try:
            with open(f"/proc/{pid}/environ", "rb") as f:
                env = f.read()
        except OSError:
            continue
        if needle in env.split(b"\0") and alive(pid):
            found.append(pid)
    return found


def _kill(pids, sig):
    for pid in pids:
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


def _ancestors(pid, limit=64):
    out = []
    while pid and pid > 1 and len(out) < limit:
        out.append(pid)
        st = _stat(pid)
        pid = st[1] if st else None
    return out


def nvidia_smi_probe(leaders=None):
    """GPU compute processes as [(pid, MiB, gpu index)] and free MiB per GPU index; None when nvidia-smi fails."""
    try:
        gpus = subprocess.run(
            ["nvidia-smi", "--query-gpu=index,uuid,memory.free", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10, check=True,
        ).stdout
        apps = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid,gpu_uuid,used_memory", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10, check=True,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    index, free = {}, {}
    for line in gpus.splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) == 3 and parts[0].isdigit() and parts[2].isdigit():
            index[parts[1]] = int(parts[0])
            free[int(parts[0])] = int(parts[2])
    if not free:
        return None
    out = []
    for line in apps.splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) == 3 and parts[0].isdigit() and parts[2].isdigit():
            out.append((int(parts[0]), int(parts[2]), index.get(parts[1])))
    return out, free


def docker_cleanup(session_id):
    """Remove the docker containers named slam_bridge_<session_id>_*; return their names."""
    try:
        names = subprocess.run(
            ["docker", "ps", "-a", "--filter", f"name=^slam_bridge_{session_id}_", "--format", "{{.Names}}"],
            capture_output=True, text=True, timeout=30, check=True,
        ).stdout.split()
    except (OSError, subprocess.SubprocessError):
        return []
    for name in names:
        subprocess.run(["docker", "rm", "-f", name], capture_output=True, timeout=60, check=False)
    return names


def infra_signs(paths) -> list[str]:
    """The GPU allocation failure lines (at most 3, cut to 200 characters) in these files."""
    found = []
    for path in paths:
        try:
            with open(path, "rb") as f:
                for line in f:
                    if INFRA_PATTERNS.search(line):
                        found.append(f"{path}: {line[:200].decode(errors='replace').strip()}")
                        if len(found) >= 3:
                            return found
        except OSError:
            continue
    return found


def job_oom_kills():
    """The job cgroup's oom_kill count (memory.events, all its steps), or None outside a Slurm job cgroup."""
    try:
        with open("/proc/self/cgroup") as f:
            path = f.read().strip().split("::", 1)[-1]
    except OSError:
        return None
    m = re.match(r"(.*/job_\d+)(/|$)", path)
    if not m:
        return None
    try:
        with open(f"/sys/fs/cgroup{m.group(1)}/memory.events") as f:
            for line in f:
                if line.startswith("oom_kill "):
                    return int(line.split()[1])
    except (OSError, ValueError):
        return None
    return None


class _Pace:
    """Tick records ({tick_t, mono}) of one live log, the last window of them."""

    def __init__(self, window_s):
        self.window_s = window_s
        self.path, self.offset, self.buf = None, 0, b""
        self.ticks = deque()

    def read(self, pattern):
        matches = sorted(glob.glob(pattern), key=lambda p: (os.path.getmtime(p), p)) if pattern else []
        if not matches:
            return
        if matches[-1] != self.path:  # a new epoch's log
            self.path, self.offset, self.buf = matches[-1], 0, b""
            self.ticks.clear()
        try:
            with open(self.path, "rb") as f:
                f.seek(self.offset)
                data = f.read()
        except OSError:
            return
        self.offset += len(data)
        lines = (self.buf + data).split(b"\n")
        self.buf = lines.pop()
        for line in lines:
            if b'"tick_t"' not in line or b'"pacing_ok"' not in line:
                continue
            try:
                e = json.loads(line)
                self.ticks.append((float(e["mono"]), float(e["tick_t"])))
            except (ValueError, KeyError, TypeError):
                continue
        while self.ticks and self.ticks[-1][0] - self.ticks[0][0] > self.window_s:
            self.ticks.popleft()

    def measure(self):
        """(sim over wall, tick gap p99) across the window, or None until it spans 80 % of the window."""
        if len(self.ticks) < 3 or self.ticks[-1][0] - self.ticks[0][0] < 0.8 * self.window_s:
            return None
        (w0, s0), (w1, s1) = self.ticks[0], self.ticks[-1]
        gaps = sorted(b[0] - a[0] for a, b in zip(self.ticks, list(self.ticks)[1:], strict=False))
        return (s1 - s0) / (w1 - w0), gaps[min(len(gaps) - 1, int(0.99 * len(gaps)))]


def _free_port(job_id, taken):
    """A port that binds now, from an offset by job so that bundles on one node seldom try the same one first."""
    lo, hi = PORTS
    first = lo + zlib.crc32(job_id.encode()) % 100 * 5
    for k in range(hi - lo):
        port = lo + (first - lo + k) % (hi - lo)
        if port in taken:
            continue
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(("127.0.0.1", port))
            except OSError:
                continue
        return port
    raise RuntimeError(f"no free port in {lo}-{hi - 1}")


def admits(current_mib, unattributed_mib, *, budget_mib, session_mib, free_mib=None) -> str:
    """Whether one more session may start: 'ok', 'over' (the job's shards) or 'gpu_full' (the device's free memory).

    current_mib: the GPU memory each running run of the job holds now; a run below session_mib (warming up) counts as
    session_mib, and the device must still give it the difference. unattributed_mib: the job's GPU memory no run owns.
    """
    used = sum(max(c, session_mib) for c in current_mib) + unattributed_mib
    if used + session_mib > budget_mib:
        return "over"
    if free_mib is not None and free_mib - sum(max(0, session_mib - c) for c in current_mib) < session_mib:
        return "gpu_full"
    return "ok"


@dataclass
class _Live:
    run: Run
    index: int
    slot: int
    proc: subprocess.Popen
    start: float
    start_wall: float
    env: dict
    log: object
    timeout: bool = False
    term_at: float | None = None
    killed: bool = False
    vram_max: int = 0
    vram_now: int = 0
    gpus: set = field(default_factory=set)
    attempt: int = 1
    oom_at_start: int | None = None
    pace: _Pace | None = None
    pace_min: float | None = None


def run_bundle(runs, out, *, slots, stagger_s=60.0, grace_s=110.0, run_timeout_s=None, shards=None,
               shard_mib=SHARD_MIB, session_vram_mib=SESSION_VRAM_MIB, vram_probe=None, vram_period_s=1.0,
               vram_unreadable_s=120.0, job_id=None, cpus=None, cleanup=docker_cleanup, stop=None, poll_s=0.5,
               base_env=None, retries=1, oom_counter=job_oom_kills, pace_floor=None, pace_gap_p99_s=None,
               pace_window_s=300.0, pace_warmup_s=240.0, pace_period_s=60.0):
    """Run the list; return the summary (also written to out/summary.json)."""
    out = Path(out)
    if (out / "results.jsonl").exists() or (out / "summary.json").exists():
        raise FileExistsError(f"{out} holds an earlier bundle")
    if slots < 1:
        raise ValueError("slots must be at least 1")
    if shards is not None and slots > shards:
        raise ValueError(f"{slots} slots but {shards} shards: a shard per Isaac session running in the job")
    (out / "runs").mkdir(parents=True, exist_ok=True)
    job_id = str(job_id or os.environ.get("SLURM_JOB_ID") or f"local{os.getpid()}")
    cpus = sorted(cpus if cpus is not None else os.sched_getaffinity(0))
    base_env = dict(os.environ if base_env is None else base_env)
    stop = stop or threading.Event()
    budget = shards * shard_mib if shards else None
    gate_on = vram_probe is not None and budget is not None
    nonce = secrets.token_hex(4)

    queue = deque((i, r, 1) for i, r in enumerate(runs))  # (index, run, attempt)
    pace_on = pace_floor is not None or pace_gap_p99_s is not None
    pace_path = out / "pace.jsonl"
    live: dict[int, _Live] = {}  # top pid -> run
    leftovers = []  # (token, kill_at)
    records = []
    st = {"last_start": None, "sampled_at": None, "unreadable_since": None, "job_total": None, "unattributed": 0,
          "free": None, "job_vram_max": 0, "vram_waits": 0, "gpu_full_waits": 0, "unreadable_starts": 0,
          "waiting": None, "cap": slots, "backoffs": [], "backoff_at": None, "pace_next": 0.0, "infra": 0}
    t_begin, wall_begin = time.monotonic(), time.time()
    stopping = False
    results_path, vram_path = out / "results.jsonl", out / "vram.jsonl"

    def record(rec):
        records.append(rec)
        with results_path.open("a") as f:
            f.write(json.dumps(rec) + "\n")

    def finish(pid, status, rusage):
        r = live.pop(pid)
        r.log.close()
        rc = os.waitstatus_to_exitcode(status) if os.WIFEXITED(status) else None
        sig = os.WTERMSIG(status) if os.WIFSIGNALED(status) else None
        r.proc.returncode = rc if rc is not None else -(sig or 0)
        token = r.env["BUNDLE_TOKEN"]
        left = token_pids(token)
        if left:
            _kill(left, signal.SIGTERM)
            leftovers.append((token, time.monotonic() + min(grace_s, 10.0)))
        oom_now = oom_counter() if oom_counter else None
        ooms = (oom_now - r.oom_at_start) if (oom_now is not None and r.oom_at_start is not None) else 0
        signs = infra_signs([out / "runs" / f"{r.run.name}.log",
                             *(p for g in r.run.scan for p in sorted(glob.glob(g)))])
        if ooms and (rc != 0):
            signs.insert(0, f"oom_kill +{ooms} in the job cgroup while it ran")
        infra = bool(signs)
        retry = infra and r.run.retryable and r.attempt <= retries and not stopping
        record({
            "name": r.run.name, "index": r.index, "slot": r.slot, "attempt": r.attempt,
            "status": "infra_failed" if infra else "ended", "infra": signs, "retried": retry,
            "oom_kills_during": ooms, "pace_min": r.pace_min,
            "start": r.start_wall, "end": time.time(), "wall_s": round(time.monotonic() - r.start, 3),
            "rc": rc, "signal": sig, "timeout": r.timeout, "killed": r.killed,
            "session_id": r.env["SESSION_ID"], "bridge_cpu": r.env["BRIDGE_CPU"], "port": r.env["PORT"],
            "cpu_s": round(rusage.ru_utime + rusage.ru_stime, 2) if rusage else None,
            "maxrss_kib": rusage.ru_maxrss if rusage else None,
            "vram_max_mib": r.vram_max, "gpus": sorted(g for g in r.gpus if g is not None),
            "leftover_ended": bool(left), "containers_removed": list(cleanup(r.env["SESSION_ID"]) or []),
        })
        if infra:
            st["infra"] += 1
        if retry:
            for path in r.run.retry_move:
                if os.path.lexists(path):
                    os.rename(path, f"{path}.attempt{r.attempt}")
            queue.appendleft((r.index, runs[r.index], r.attempt + 1))

    def start(index, template, attempt):
        run = template.at(attempt)
        slot = min(set(range(slots)) - {r.slot for r in live.values()})
        env = dict(base_env)
        taken = {int(r.env["PORT"]) for r in live.values() if r.env["PORT"].isdigit()}
        taken |= {int(x.env["PORT"]) for _, x, _ in queue if "PORT" in x.env}
        env.update({
            "SESSION_ID": f"{job_id}s{index}" + (f"r{attempt}" if attempt > 1 else ""), "BRIDGE_CPU": str(cpus[slot % len(cpus)]),
            "PORT": str(_free_port(job_id, taken)), "BUNDLE_SLOT": str(slot), "BUNDLE_JOB": job_id,
            "BUNDLE_RUN": run.name, "BUNDLE_TOKEN": f"{job_id}-{index}-{attempt}-{nonce}",
        })
        env.update(run.env)
        log = (out / "runs" / f"{run.name}.log").open("ab")
        log.write(f"=== bundle attempt {attempt} {time.strftime('%Y-%m-%dT%H:%M:%S%z')}\n".encode())
        log.flush()
        try:
            proc = subprocess.Popen(run.argv, cwd=run.cwd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
        except OSError as exc:
            log.close()
            record({"name": run.name, "index": index, "slot": slot, "attempt": attempt, "status": "start_failed",
                    "error": str(exc),
                    "start": time.time(), "rc": None, "signal": None})
            return
        now = time.monotonic()
        live[proc.pid] = _Live(run, index, slot, proc, now, time.time(), env, log, attempt=attempt,
                               oom_at_start=oom_counter() if oom_counter else None,
                               pace=_Pace(pace_window_s) if run.pace_log else None)
        st["last_start"] = now

    def sample_vram(now):
        leaders = [pid for pid, _ in sorted(live.items(), key=lambda kv: kv[1].slot)]
        got = vram_probe(leaders)
        if got is None:
            st["job_total"] = None
            if st["unreadable_since"] is None:
                st["unreadable_since"] = now
            return
        st["unreadable_since"] = None
        apps, free = got
        per = dict.fromkeys(live, 0)
        job_total, job_gpus = 0, set()
        me = os.getpid()
        for pid, mib, gpu in apps:
            chain = _ancestors(pid)
            owner = next((p for p in chain if p in live), None)
            if owner is None and me not in chain:
                continue  # another job's process
            job_total += mib
            job_gpus.add(gpu)
            if owner is not None:
                per[owner] += mib
                live[owner].gpus.add(gpu)
        for pid, mib in per.items():
            live[pid].vram_now = mib
            live[pid].vram_max = max(live[pid].vram_max, mib)
        st["job_total"], st["unattributed"] = job_total, max(0, job_total - sum(per.values()))
        mine = [g for g in job_gpus if g is not None]
        st["free"] = min(free[g] for g in mine if g in free) if any(g in free for g in mine) else (
            min(free.values()) if len(free) == 1 else None)
        st["job_vram_max"] = max(st["job_vram_max"], job_total)
        st["sampled_at"] = now
        with vram_path.open("a") as f:
            f.write(json.dumps({"t": round(time.time(), 2), "job_mib": job_total, "free_mib": st["free"],
                                "runs": {live[p].run.name: m for p, m in per.items()}}) + "\n")

    def verdict(now):
        if not gate_on or not live:
            return "ok"
        if st["unreadable_since"] is not None:
            if now - st["unreadable_since"] >= vram_unreadable_s:
                return "unreadable_go"
            return "stale"
        if st["sampled_at"] is None or st["sampled_at"] <= st["last_start"]:
            return "stale"  # judge only on a sample taken after the latest start
        return admits([r.vram_now for r in live.values()], st["unattributed"], budget_mib=budget,
                      session_mib=session_vram_mib, free_mib=st["free"])

    def evaluate_pace(now):
        rows = {}
        for r in live.values():
            if r.pace is None:
                continue
            r.pace.read(r.run.pace_log)
            got = r.pace.measure() if now - r.start >= pace_warmup_s else None
            if got is not None:
                rows[r.run.name] = {"ratio": round(got[0], 4), "gap_p99_s": round(got[1], 4)}
                r.pace_min = round(min(got[0], r.pace_min if r.pace_min is not None else got[0]), 4)
        reason = None
        if rows:
            ratios = sorted(v["ratio"] for v in rows.values())
            median = ratios[len(ratios) // 2] if len(ratios) % 2 else (ratios[len(ratios) // 2 - 1]
                                                                          + ratios[len(ratios) // 2]) / 2
            worst_gap = max(v["gap_p99_s"] for v in rows.values())
            if pace_floor is not None and median < pace_floor:
                reason = f"median sim/wall {median:.4f} < {pace_floor}"
            elif pace_gap_p99_s is not None and worst_gap > pace_gap_p99_s:
                reason = f"tick gap p99 {worst_gap:.3f} s > {pace_gap_p99_s}"
        cooled = st["backoff_at"] is None or now - st["backoff_at"] >= pace_window_s
        stepped = False
        if reason and cooled and st["cap"] > 1:
            st["cap"] = min(st["cap"], len(live)) - 1 if len(live) > 1 else 1
            st["cap"] = max(1, st["cap"])
            st["backoff_at"] = now
            st["backoffs"].append({"t": round(time.time(), 1), "reason": reason, "cap": st["cap"]})
            stepped = True
        with pace_path.open("a") as f:
            f.write(json.dumps({"t": round(time.time(), 1), "runs": rows, "cap": st["cap"], "reason": reason,
                                "stepped_back": stepped}) + "\n")

    vram_next = 0.0
    while queue or live or leftovers:
        now = time.monotonic()
        for pid in list(live):
            try:
                got, status, rusage = os.wait4(pid, os.WNOHANG)
            except ChildProcessError:
                got, status, rusage = pid, 0, None
            if got == pid:
                finish(pid, status, rusage)
        for item in list(leftovers):
            token, kill_at = item
            left = token_pids(token)
            if not left:
                leftovers.remove(item)
            elif now >= kill_at:
                _kill(left, signal.SIGKILL)
                leftovers.remove(item)
        if stop.is_set() and not stopping:
            stopping = True
            for r in live.values():
                if r.term_at is None:
                    _kill([r.proc.pid], signal.SIGTERM)
                    r.term_at = now
            while queue:
                index, run, attempt = queue.popleft()
                record({"name": run.name, "index": index, "slot": None, "attempt": attempt, "status": "not_started",
                        "rc": None, "signal": None})
        for r in live.values():
            if run_timeout_s is not None and r.term_at is None and now - r.start > run_timeout_s:
                r.timeout, r.term_at = True, now
                _kill([r.proc.pid], signal.SIGTERM)
            if r.term_at is not None and not r.killed and now - r.term_at > grace_s:
                r.killed = True
                _kill(set(token_pids(r.env["BUNDLE_TOKEN"])) | {r.proc.pid}, signal.SIGKILL)
        if vram_probe is not None and now >= vram_next:
            vram_next = now + vram_period_s
            sample_vram(now)
        if pace_on and now >= st["pace_next"]:
            st["pace_next"] = now + pace_period_s
            evaluate_pace(now)
        if (not stopping and queue and len(live) < min(slots, st["cap"])
                and (st["last_start"] is None or now - st["last_start"] >= stagger_s)):
            v = verdict(now)
            if v in ("ok", "unreadable_go"):
                st["waiting"] = None
                if v == "unreadable_go":
                    st["unreadable_starts"] += 1
                index, run, attempt = queue.popleft()
                start(index, run, attempt)
                continue
            if v in ("over", "gpu_full") and st["waiting"] != v:
                st["vram_waits" if v == "over" else "gpu_full_waits"] += 1
                st["waiting"] = v
        time.sleep(poll_s)

    final = [r for r in records if not r.get("retried")]  # the last attempt of each run
    failed = sum(1 for r in final if r["status"] in ("ended", "infra_failed") and (r["rc"] != 0 or r["status"] ==
                                                                                    "infra_failed"))
    not_started = sum(1 for r in final if r["status"] in ("not_started", "start_failed"))
    summary = {
        "runs": len(runs), "failed": failed, "not_started": not_started, "slots": slots, "stagger_s": stagger_s,
        "shards": shards, "vram_gate": gate_on, "vram_waits": st["vram_waits"],
        "gpu_full_waits": st["gpu_full_waits"], "unreadable_starts": st["unreadable_starts"],
        "infra_failed_attempts": st["infra"], "retries": retries, "backoffs": st["backoffs"],
        "final_cap": st["cap"], "vram_job_max_mib": st["job_vram_max"], "budget_mib": budget, "job_id": job_id, "cpus": cpus,
        "start": wall_begin, "wall_s": round(time.monotonic() - t_begin, 3),
        "exit_code": 0 if failed == 0 and not_started == 0 else 1,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    return summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--list", required=True, help="the run list (JSON lines)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--slots", type=int, help="runs at once (default: the shards)")
    ap.add_argument("--shards", type=int, help="the job's shards (default: SLURM_SHARDS_ON_NODE)")
    ap.add_argument("--stagger-s", type=float, default=60.0)
    ap.add_argument("--session-vram-mib", type=int, default=SESSION_VRAM_MIB)
    ap.add_argument("--shard-mib", type=int, default=SHARD_MIB)
    ap.add_argument("--vram-period-s", type=float, default=1.0)
    ap.add_argument("--vram-unreadable-s", type=float, default=120.0)
    ap.add_argument("--run-timeout-s", type=float)
    ap.add_argument("--grace-s", type=float, default=110.0)
    ap.add_argument("--retries", type=int, default=1, help="reruns of a run that failed for the infrastructure")
    ap.add_argument("--pace-floor", type=float, help="step back when the median sim/wall of live runs is below")
    ap.add_argument("--pace-gap-p99-s", type=float, help="step back when a live run's tick gap p99 is above")
    ap.add_argument("--pace-window-s", type=float, default=300.0)
    ap.add_argument("--pace-warmup-s", type=float, default=240.0)
    ap.add_argument("--pace-period-s", type=float, default=60.0)
    ap.add_argument("--no-vram", action="store_true", help="no GPU memory samples and no memory gate (slots only)")
    ap.add_argument("--no-cleanup", action="store_true", help="do not look for leftover bridge containers")
    ap.add_argument("--allow-multi-gpu", action="store_true")
    a = ap.parse_args(argv)
    gpus = visible_gpus(os.environ)
    if len(gpus) > 1 and not a.allow_multi_gpu:
        print(f"refusing: the job sees {len(gpus)} GPUs ({','.join(gpus)}) and every session uses device 0 -- its "
              "shards may be split across GPUs (--allow-multi-gpu to run anyway)", file=sys.stderr)
        return 2
    env_shards = os.environ.get("SLURM_SHARDS_ON_NODE", "")
    shards = a.shards or (int(env_shards) if env_shards.isdigit() else None)
    slots = a.slots or shards
    if not slots:
        print("refusing: no --slots and no shard count (--shards or SLURM_SHARDS_ON_NODE)", file=sys.stderr)
        return 2
    if shards is not None and slots > shards:
        print(f"refusing: {slots} slots but {shards} shards (a shard per Isaac session)", file=sys.stderr)
        return 2
    try:
        runs = load_runs(a.list)
    except (OSError, ValueError) as exc:
        print(f"refusing the run list: {exc}", file=sys.stderr)
        return 2
    probe = None if a.no_vram else nvidia_smi_probe
    if probe is not None and probe() is None:
        print("refusing: nvidia-smi cannot be read (--no-vram runs by the slots alone)", file=sys.stderr)
        return 2
    if probe is not None and shards is None:
        print("no shard count: GPU memory is sampled but not gated", file=sys.stderr)
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())
    summary = run_bundle(
        runs, a.out, slots=slots, stagger_s=a.stagger_s, grace_s=a.grace_s, run_timeout_s=a.run_timeout_s,
        shards=shards, shard_mib=a.shard_mib, session_vram_mib=a.session_vram_mib, vram_probe=probe,
        vram_period_s=a.vram_period_s, vram_unreadable_s=a.vram_unreadable_s,
        cleanup=(lambda sid: []) if a.no_cleanup else docker_cleanup, stop=stop, retries=a.retries,
        pace_floor=a.pace_floor, pace_gap_p99_s=a.pace_gap_p99_s, pace_window_s=a.pace_window_s,
        pace_warmup_s=a.pace_warmup_s, pace_period_s=a.pace_period_s,
    )
    print(json.dumps(summary))
    return summary["exit_code"]


if __name__ == "__main__":
    sys.exit(main())
