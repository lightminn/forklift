"""Several Isaac runs in one Slurm job (docs/plans/2026-10-24-ws1-isaac-bundle.md): a run list, N slots, staggered starts.

    python3 tools/isaac_bundle.py check --list RUNS.jsonl --runner SNAP/sim/isaac/run_transport.py
    sbatch $(python3 tools/isaac_bundle.py sizes live|riib|bd [--slots N]) deploy/slurm/isaac_bundle.sbatch RUNS OUT KIND
    python3 tools/isaac_bundle.py --list RUNS.jsonl --out OUT [--slots N] [--shards N] [--stagger-s 60]
        [--session-vram-mib 4100] [--run-timeout-s S] [--grace-s 110] [--retries 1] [--pace-floor R]
        [--pace-gap-p99-s S] [--no-vram] [--no-cleanup] [--allow-multi-gpu]

A run is one JSON line: {"name": .., "argv": [..], "env": {..}, "cwd": .., "scan": [globs], "retry_move": [paths],
"pace_log": glob} (all but name and argv optional). Runs start in list order whenever

- a slot is free: at most --slots runs at once, and never more slots than the job's shards (the rule: as many shards
  as Isaac sessions running in the job). A run holds its slot until every process it started has ended, not only its
  top process;
- the previous start is at least --stagger-s old (a live session reaches its steady GPU memory 31-40 s after it
  starts, measured on ws1);
- the GPU memory allows one more session, on a sample taken after the latest start: the job's runs, each counted at
  least --session-vram-mib (a run still warming up holds its whole share), plus one more fit in --shards x
  --shard-mib, and the GPU's free memory, less what the job's warming runs will still take, holds one more session
  (checked for the first run too). Shards are a scheduling count, not a partition: the second check keeps a start
  from landing on a GPU that other jobs have filled. When nvidia-smi cannot be read the start waits, for at most
  --vram-unreadable-s, then goes on by the slots alone and says so;
- the node's MemAvailable is at least --node-ram-min-mib (a session and its bridge container): the job's cgroup is
  reserved by Slurm, but the SLAM bridges and other users' containers are not.

Each run gets SESSION_ID=<job>s<index>[r<attempt>] (the launchers name their docker containers with it), BRIDGE_CPU
(the SLAM bridge container's cpuset: with --bridge-cpus a core set apart for every run's bridge while the sessions are
pinned to the other CPUs with taskset -- the bridge's keyframe replies slow down as its graph grows and must not
queue behind Isaac's thread pools -- else a CPU of the job per slot, shared), PORT (a free port, 9000-9499, from an
offset by job), BUNDLE_SLOT, BUNDLE_JOB, BUNDLE_RUN and BUNDLE_TOKEN. The list may set PORT (each value once) but none
of the others. Each run is its own process group with its log in OUT/runs/<name>.<attempt>.log; one that fails leaves
the others running.

A run past --run-timeout-s, and every run when the bundle gets TERM/INT, is ended like Slurm's --signal=B:TERM: TERM
to its top process only (the launchers end their supervisor, runner and bridge in that order), then KILL after
--grace-s to every process carrying its BUNDLE_TOKEN -- found through /proc/<pid>/environ, because `timeout` and
similar tools move their child into a process group of its own. Processes a finished run left behind get TERM, then
KILL after --leftover-grace-s; a run is over when two scans 1 s apart find none of its processes. Docker containers named slam_bridge_<SESSION_ID>_* (outside the run's processes and
the job's cgroup) are removed. A failing cleanup step is recorded and does not stop the other runs.

A run failed for the infrastructure when its own attempt log, or a file its "scan" globs name, has a GPU allocation
failure (ONNX Runtime's BFCArena, Vulkan's ERROR_OUT_OF_DEVICE_MEMORY, CUDA out of memory), or when it was killed
(SIGKILL, or exit 137 from its launcher) while the job cgroup's oom_kill count rose. An oom_kill while a run failed
otherwise is recorded beside it, not blamed on it. An infrastructure failure is recorded as status "infra_failed" and,
with --retries left, the run goes again once all its processes have ended: the paths in its "retry_move" are renamed
to <path>.attempt<n> first (so the launcher's output folder keeps its name), and "{attempt}" in its argv, env, scan and
pace_log is the attempt number. A failure of the run itself is never retried.

Real-time back-off (live runs): a run's "pace_log" glob names its live_log.jsonl (the newest match is read). Every
--pace-period-s the bundle takes, for each run older than --pace-warmup-s, simulated over wall time across the last
--pace-window-s of tick records up to now (a run whose ticks stopped counts its silence as a gap) and the 99th
percentile tick gap. When the median of those ratios falls below --pace-floor, or a run's gap p99 exceeds
--pace-gap-p99-s (either for --pace-confirm evaluations in a row), or a run has written no tick for --pace-stall-s (at
once), the bundle starts no new run until fewer than (running - 1) run -- one step back -- and waits a window before
it steps back again. The warm-up (540 s) is a session's start-up, 0.22-0.33 for its first four minutes on ws1, plus a
window, so that no judged window holds it: with 240 s every bundle of rh (2026-10-25) stepped back on its own
start-up and stayed at half its slots. Stalls are judged earlier, from --pace-stall-warmup-s. After --pace-recover clean
evaluations in a row (every running live run past its warm-up, measured and at pace) and a window since the last
back-off and the last step-up, the cap goes up by one again: a back-off that was a false alarm costs minutes, not the
job. A step-up does not delay the next back-off. OUT/pace.jsonl keeps every evaluation, runs it could not measure included.

Several bundle jobs can share one list (--claim-dir): each run is claimed with an atomic mkdir before its first
attempt, so a job with a free slot takes the next unclaimed run and the jobs' last waves end together instead of one
job holding its GPU for a single straggler. A job refuses to start while another bundle on this node runs the same
list without the same --claim-dir (both would run every run into the same folders; --list-registry-dir).

Time limit: with --run-est-min (default the kind's measured p95 run) and inside a Slurm job, the runner reads the
job's time left (squeue %L, again every 5 min: scontrol can shorten it) and starts no run with less than that plus
--grace-s and a minute left; such runs are recorded as time_left and stay unclaimed for another job on the list.
`sizes KIND --runs R --jobs J [--run-min M]` prints a --time to match.

Stall watchdog: a run whose progress files -- its "watch" globs, else its pace_log, else the kind's file under its first
retry_move (riib: run/frames.jsonl) -- have not changed for --stall-min (default the kind's) is first looked at --
tools/stall_watch.py's capture_diagnostics (py-spy --native, every thread's state/utime/stime/wchan, /proc status,
nvidia-smi) into its output folder's stall_diagnostics.txt, then SIGUSR1 to its run_transport processes (their
faulthandler writes every thread's Python stack to <output>.stall_stack.txt) -- then, --stack-wait-s later, ended like
a timeout (TERM, KILL after --grace-s) and recorded as stalled; it is never retried (a hang is not the infrastructure,
and a rerun would blur a deterministic comparison). Before the first progress file exists the silence counts from
the run's start, and is judged only while a Python process runs --stack-match (run_transport) under the run,
from the later of the run's and that runner's start -- between a live session's epochs the supervisor runs alone for
up to its backoff plus 300 s of standby; a run with no progress file named is not watched.

Records: OUT/results.jsonl (a line per attempt), OUT/vram.jsonl (the job's GPU memory, per run, every
--vram-period-s), OUT/pace.jsonl, OUT/summary.json. A run's exit code is its top process's: the launcher has to end
with the code of what it ran. Exit 0 when every run's last attempt exited 0, 1 otherwise, 2 for a refused
configuration.
"""

from __future__ import annotations

import argparse
import fcntl
import glob
import json
import os
import re
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
import zlib
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

SHARD_MIB = 4076  # 16,303 MiB / 4 shards per GPU on ws1 (gres.conf)
SESSION_VRAM_MIB = 4100  # a live session with the learned worker at its peak (3,890-4,047 MiB) + ~50 MiB driver share: KINDS live
PORTS = (9000, 9500)
RESERVED_ENV = ("SESSION_ID", "BRIDGE_CPU", "BUNDLE_SLOT", "BUNDLE_JOB", "BUNDLE_RUN", "BUNDLE_TOKEN")
# The failures themselves, not summary lines such as riib_scene's "OUT_OF_DEVICE lines: 0".
INFRA_PATTERNS = re.compile(
    rb"BFCArena::Allocate|bfc_arena\.cc:\d+|ERROR_OUT_OF_DEVICE_MEMORY|CUDA out of memory|CUDA_ERROR_OUT_OF_MEMORY"
    rb"|cudaErrorMemoryAllocation"
)
# Per-session sizes measured on ws1 (2026-10-24/25, docs/plans/2026-10-24-ws1-isaac-bundle.md 1 and 2.4). RAM is the
# job cgroup's anon memory (the page cache of Isaac's libraries is shared by a bundle's sessions and reclaimable, so
# it is counted once, SHARED_FILE_GIB); VRAM is per session with its learned worker; max_slots is how many fit one
# GPU's 16,303 MiB with the device check's margin.
KINDS = {
    # live console sessions (isaac_live / ct1_queue), paced at 0.4: anon <= 5.48 GiB, 1.0-1.2 cores, 3,886-3,915 MiB
    # run_min: a run's p95 wall time -- live ct1_queue 29.6 (rh/sf, 39 runs, max 30.5), riib 62.4 (lcg, 32 runs; rcf
    # 46.6, dv3's trailer up to 77), bd 12.1 (bd_isaac, 151 jobs); a batch with longer runs passes --run-min
    # stall_min: no change of a run's progress file for this long ends it (status stalled, not retried). live: the
    # live_log.jsonl of 30 runs (rh/sf) never went quiet for more than 3.3 s, its first record came 38-50 s after
    # the start. riib (run/frames.jsonl, watch_rel; its outputs carry no wall-clock timeline): 86 runs on ws1
    # (2026-10-26) took 10-136 s from run.log to frames.jsonl (median 44), waited at most 1.0 s for a SLAM reply and
    # 0.5 s for an obstacle cast per scan, and wrote result.json at most 3.8 s after their last frame; planning is not
    # timed in its records and has no wall-clock bound (pallet_mission's retry ladder: up to 390,000 hybrid-A*
    # expansions, no deadline outside the grid path -- Codex), so riib is measured, not acted on: every watched run's
    # results line carries silence_max_s; set stall_min from those (dv3_trailer_det2 hung in render for 45 min).
    # live VRAM (2026-10-27): a GPU gives 16,303 - 464 (the driver's reservation) = 15,839 MiB; sessions with their
    # learned worker peak at 3,890-4,047 MiB (3,762-3,821 early) and take ~50 MiB of driver share each, so a fourth
    # session does not fit once they have grown (11596_2 never started; four ran only while small). 4,100 a session:
    # 3 a GPU, 3 sessions on 4 shards, 1 on 2 -- what the shards tell other jobs -- until session VRAM shrinks
    # (docs/plans/2026-10-26-ws1-isaac-array.md 5.4).
    "live": {"session_gib": 5.5, "cores": 2, "max_slots": 3, "vram_mib": 4100, "run_min": 30, "stall_min": 10},
    # batch riib_scene with video: anon 17.8 GiB for 3 sessions (5.93 each), 4,145-4,366 MiB each -> 3 per GPU
    "riib": {"session_gib": 6.0, "cores": 2, "max_slots": 3, "vram_mib": 4500, "run_min": 63, "stall_min": 0,
             "watch_rel": "run/frames.jsonl"},
    # batch bd/jao without the learned worker: cgroup peak <= 5.82 GiB single, 1.8-2.2 cores
    "bd": {"session_gib": 5.0, "cores": 2, "max_slots": 4, "vram_mib": 3500, "run_min": 13},
}
BRIDGE_CORE_CPUS = 2  # one physical core (its two hyperthreads) for the SLAM bridges, apart from the Isaac sessions
SHARED_FILE_GIB = 1.5  # file pages the bundle's sessions share (10793/10794: 1.36-1.49 GiB)
MARGIN_GIB = 0.5
TERM_EARLY_S = 180  # --signal=B:TERM@120 reaches the batch shell up to 60 s before that (Slurm's signal resolution)
BRIDGE_MIB = 760  # a SLAM bridge container, outside the job's cgroup: 0.12-0.74 GiB measured (se* pilot, 16 bridges)
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
    watch: list = field(default_factory=list)

    def at(self, attempt) -> Run:
        """This run with "{attempt}" filled in."""
        f = (lambda x: x.replace("{attempt}", str(attempt)))  # noqa: E731
        return Run(self.name, [f(a) for a in self.argv], {k: f(v) for k, v in self.env.items()}, self.cwd,
                   [f(g) for g in self.scan], [f(m) for m in self.retry_move],
                   f(self.pace_log) if self.pace_log else None, [f(w) for w in self.watch])

    @property
    def retryable(self) -> bool:
        """A run can go again when its outputs move aside or depend on the attempt (judged on the list's entry)."""
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
        unknown = set(obj) - {"name", "argv", "env", "cwd", "scan", "retry_move", "pace_log", "watch"}
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
        for key in ("scan", "retry_move", "watch"):
            value = obj.get(key, [])
            if not isinstance(value, list) or not all(isinstance(x, str) for x in value):
                raise ValueError(f"line {n}: {key} must be a list of strings")
            lists[key] = value
        pace_log = obj.get("pace_log")
        if pace_log is not None and not isinstance(pace_log, str):
            raise ValueError(f"line {n}: pace_log must be a string")
        runs.append(Run(name=name, argv=argv, env=env, cwd=cwd, pace_log=pace_log, **lists))
    return runs


def defined_flags(script) -> set:
    """The --flags a Python script defines (argparse add_argument("--x", ...) and the like, read from its source)."""
    return set(re.findall(r"""["'](--[A-Za-z0-9][A-Za-z0-9_-]*)["']""", Path(script).read_text()))


def _dry_run_launcher(argv, cwd):
    """The launcher script in argv (resolved in the run's cwd) that has a DRY_RUN mode printing RUNNER_ARGS, or None
    (then nothing is run: a launcher without it, such as isaac_live.sbatch, would start Isaac for real)."""
    for a in argv:
        path = os.path.join(cwd or ".", a)
        if a.endswith((".sbatch", ".sh")) and os.path.isfile(path):
            text = Path(path).read_text(errors="replace")
            return path if "DRY_RUN" in text and "RUNNER_ARGS" in text else None
    return None


def _scratch_argv(argv, env, cwd, outputs, scratch):
    """argv with every argument naming one of the outputs (compared as real paths, relative ones resolved in cwd)
    replaced by a scratch path; None unless every output was replaced and no output path is left anywhere else in
    argv or env (inside an --opt=PATH, say), since the DRY_RUN could then write that real output."""
    real = {os.path.realpath(o): k for k, o in enumerate(outputs)}
    out, hit = [], set()
    for a in argv:
        k = real.get(os.path.realpath(os.path.join(cwd or ".", a))) if a else None
        if k is not None:
            out.append(os.path.join(scratch, f"out{k}"))
            hit.add(k)
        else:
            out.append(a)
    if len(hit) != len(outputs):
        return None
    left = [o for o in {*outputs, *real} for text in (*out, *env.values()) if o and o in text]
    return None if left else out


def _end_token(token, quiet_s=0.5, limit_s=10.0):
    """KILL every process carrying the token until two scans quiet_s apart find none (a process may fork between a
    scan's listing and its reads), for at most limit_s."""
    deadline, empty_since = time.monotonic() + limit_s, None
    while time.monotonic() < deadline:
        left = token_pids(token)
        if left:
            _kill(left, signal.SIGKILL)
            empty_since = None
        elif empty_since is None:
            empty_since = time.monotonic()
        elif time.monotonic() - empty_since >= quiet_s:
            return True
        time.sleep(0.05)
    return False


def check_runs(runs, runner, timeout_s=120.0) -> list[str]:
    """Problems found before a submission: a run whose output (retry_move) already exists, has no output named, or
    whose argv does not name it (none of these is run), a launcher without a DRY_RUN mode (not run either), a DRY_RUN
    that fails, hangs or prints no RUNNER_ARGS line, and runner arguments whose --flags `runner` does not define (an
    option the snapshot's code lacks fails only once the job runs). The DRY_RUN writes to a scratch copy of the
    output; on a timeout every process carrying its token is killed, whatever its process group."""
    import tempfile

    known = defined_flags(runner)
    problems = []
    for n, run in enumerate(runs):
        r = run.at(1)
        if not r.retry_move:
            problems.append(f"{r.name}: no retry_move output named (not run: its DRY_RUN could write a real output)")
            continue
        existing = [path for path in r.retry_move if os.path.lexists(path)]
        if existing:
            problems.append(f"{r.name}: {', '.join(existing)} exists (not run)")
            continue
        if _dry_run_launcher(r.argv, r.cwd) is None:
            problems.append(f"{r.name}: no launcher with a DRY_RUN mode printing RUNNER_ARGS (not run)")
            continue
        with tempfile.TemporaryDirectory(prefix="bundle_check_") as scratch:
            merged = {**os.environ, **r.env}  # what the launcher will see: the inherited environment too
            argv = _scratch_argv(r.argv, merged, r.cwd, r.retry_move, scratch)
            if argv is None:
                problems.append(f"{r.name}: its outputs {r.retry_move} are not each a whole argument of its argv, or "
                                "appear elsewhere in its argv or env (not run)")
                continue
            token = f"check-{os.getpid()}-{n}-{secrets.token_hex(4)}"
            env = {**merged, "DRY_RUN": "1", "BUNDLE_TOKEN": token}
            out_path = os.path.join(scratch, "stdout.txt")
            with open(out_path, "w") as out_file:  # a file, not a pipe: a straggler cannot hold the read open
                try:
                    p = subprocess.Popen(argv, cwd=r.cwd, env=env, stdout=out_file, stderr=subprocess.DEVNULL,
                                         stdin=subprocess.DEVNULL, start_new_session=True)
                except OSError as exc:
                    problems.append(f"{r.name}: DRY_RUN did not start ({exc})")
                    continue
                hung, ended = False, False
                try:
                    p.wait(timeout=timeout_s)
                except subprocess.TimeoutExpired:
                    hung = True
                finally:  # a timeout, an interrupt or a clean end: nothing it started may outlive the check
                    if p.poll() is None:
                        _kill([p.pid], signal.SIGKILL)
                    ended = _end_token(token)
                    try:
                        p.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        pass
            if not ended:
                problems.append(f"{r.name}: processes of its DRY_RUN survive SIGKILL (check stopped)")
                return problems  # do not start another launcher beside them
            if hung:
                problems.append(f"{r.name}: DRY_RUN did not end in {timeout_s} s (killed)")
                continue
            lines = Path(out_path).read_text(errors="replace").splitlines()
        if p.returncode != 0 or "RUNNER_ARGS" not in lines:
            problems.append(f"{r.name}: DRY_RUN exit {p.returncode}, no RUNNER_ARGS line")
            continue
        args = " ".join(lines[lines.index("RUNNER_ARGS") + 1:]).split()
        unknown = sorted({a.split("=", 1)[0] for a in args if a.startswith("--")} - known)
        if unknown:
            problems.append(f"{r.name}: {runner} does not define {unknown}")
    return problems


def job_gpu_index(env) -> int | None:
    """The job's GPU as nvidia-smi numbers it: SLURM_JOB_GPUS is the node-wide index; CUDA_VISIBLE_DEVICES is the
    job's own numbering under ConstrainDevices (0 for whichever GPU it got) and is not used. None unless exactly one."""
    gpus = [g.strip() for g in (env.get("SLURM_JOB_GPUS") or "").split(",") if g.strip()]
    return int(gpus[0]) if len(gpus) == 1 and gpus[0].isdigit() else None


def sbatch_sizes(kind, slots=None, runs=None, jobs=1, run_min=None) -> dict:
    """The job's resources for `slots` sessions of `kind` (default its max per GPU): shards for the sessions' GPU
    memory (a riib session with its video holds 4.47 GB, more than a 4,076 MiB shard, so 3 of them take 4 shards --
    the shards are what other jobs on the GPU rely on), 2 cores per session, RAM = slots x session +
    the shared file pages + a margin, rounded up to a GiB. The runner is told the kind (deploy/slurm/
    isaac_bundle.sbatch's third argument, --kind) and takes the slots and the session's GPU memory from it."""
    k = KINDS[kind]
    slots = slots or k["max_slots"]
    if not 1 <= slots <= k["max_slots"]:
        raise ValueError(f"{kind}: 1-{k['max_slots']} sessions per GPU, not {slots}")
    shards = -(-slots * k["vram_mib"] // SHARD_MIB)
    if shards > 4:
        raise ValueError(f"{kind}: {slots} sessions need {shards} shards, more than a GPU's 4")
    mem = -(-(slots * k["session_gib"] + SHARED_FILE_GIB + MARGIN_GIB) // 1)
    # no core for the SLAM bridges by default: set apart (P2c) they answered keyframes on the same curve as on a shared
    # CPU (scans 4,500-4,999: median 0.32 s, p95 0.77-0.82 s -- the graph's size sets it, not contention), so the two
    # CPUs go to more sessions; --bridge-cpus N with a bigger --cpus-per-task still sets one apart
    out = {"slots": slots, "gres": f"shard:{shards}", "cpus": slots * k["cores"], "mem_gib": int(mem)}
    if runs:
        out["time"] = time_limit(runs, slots, jobs, run_min or k["run_min"])
    return out


# One run per Slurm array task (deploy/slurm/isaac_array.sbatch, docs/plans/2026-10-26-ws1-isaac-array.md), sized
# as a single job, not as a bundle's share: a task cannot borrow a neighbour's idle CPUs or page cache. The pilot
# (11590, 2026-10-26) ran live tasks at CPU 2 / 7G: 3 of 3 ended slam_link_failed (no bridge reply in 1 s) -- the two
# CPUs are one physical core, the session and its bridge on it, bridge median 0.174-0.178 s against 0.132-0.136 s for
# the same scenes in a 4-slot bundle -- and the cgroup met its 7G limit 590 and 707 times. cpus: live 4 (the runner's
# --bridge-cpus auto then gives the bridge one core and the session the other), riib 6 (a riib session used 3.1 of 4
# CPUs alone: two cores for it, one for the bridge), bd 6 (its single job); task_gib: live and riib 8 (their single
# jobs'; S3 runner RSS 6.6 GiB), bd 7 (its single job); array_k: 20 CPUs give riib 3, bd 3; live 4 -- a live task
# takes 2 shards since its VRAM (KINDS) is over one, so a GPU holds 2 of them: 3-slot bundles hold 3 a GPU.
ARRAY = {"live": {"cpus": 4, "task_gib": 8.0, "array_k": 4}, "riib": {"cpus": 6, "task_gib": 8.0, "array_k": 3},
         "bd": {"cpus": 6, "task_gib": 7.0, "array_k": 3}}
RETRY_ROOM_MIN = 10  # a GPU allocation failure comes in a run's first minutes: room for the runner's retry in the task


def array_sizes(kind, runs, run_min):
    """One task's sbatch options for a run list of `runs` runs of `kind`, one run per array task. run_min is the
    batch's own p95 run length -- required: one kind's runs differ by hours (S3: 21 to 209 min)."""
    if not runs or runs < 1:
        raise ValueError("--array needs --runs (the list's runs)")
    if not run_min:
        raise ValueError("--array needs --run-min: the batch's own p95 run length in minutes (one kind's runs differ "
                         "by hours: S3 21-209 min); give the runner the same with --run-est-min")
    k, a = KINDS[kind], ARRAY[kind]
    shards = -(-k["vram_mib"] // SHARD_MIB)
    minutes = int(-(-((run_min + RETRY_ROOM_MIN + 5) * 1.15) // 5) * 5)
    return {"array": f"0-{runs - 1}%{min(a['array_k'], runs)}", "gres": f"shard:{shards}", "cpus": a["cpus"],
            "mem_mib": int(a["task_gib"] * 1024), "time": f"{minutes // 60:02d}:{minutes % 60:02d}:00"}


def collect(out, expect):
    """Each array task's final state under out/task_<i>/bundle: the last attempt's status per task, and the tasks with
    no record at all (never started, cancelled before the runner wrote). summary_exit is the task summary's exit_code
    (None when the summary is missing or unreadable). Returns (rows, missing)."""
    rows, missing = [], []
    for i in range(expect):
        path = Path(out) / f"task_{i}" / "bundle" / "results.jsonl"
        summary = Path(out) / f"task_{i}" / "bundle" / "summary.json"
        recs = []
        if path.exists():
            with open(path) as f:
                recs = [json.loads(x) for x in f if x.strip()]
        if not recs:
            missing.append(i)
            continue
        last = recs[-1]
        try:  # the runner's own verdict: a run that ended rc 0 can still leave errors (a bridge cleanup that failed)
            sm = json.loads(summary.read_text())
            verdict = sm["exit_code"] if isinstance(sm, dict) else None
        except (OSError, ValueError, KeyError):
            verdict = None
        rows.append({"task": i, "name": last.get("name"), "status": last.get("status"), "rc": last.get("rc"),
                     "attempts": len(recs), "summary_exit": verdict})
    return rows, missing


def time_limit(runs, slots, jobs=1, run_min=30.0, stagger_min=1.0):
    """--time for `runs` runs on `jobs` jobs of `slots` sharing one list: the waves they make together times a run's
    p95, the stagger of one wave, 5 min of start and settling, 15 % on top, rounded up to 5 min. A job that started
    late or ran short of time leaves runs for the others: the runner starts no run its time limit would cut."""
    waves = -(-runs // (slots * jobs))
    minutes = (waves * run_min + (slots - 1) * stagger_min + 5) * 1.15
    minutes = int(-(-minutes // 5) * 5)
    return f"{minutes // 60:02d}:{minutes % 60:02d}:00"


def _cmdline(pid) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read().replace(b"\0", b" ").decode(errors="replace")
    except OSError:
        return ""


def stall_targets(pids, match):
    """The pids of Python interpreters running `match` that catch SIGUSR1 (tools/stall_watch.py's test, inlined: this
    file runs alone). The interpreter test keeps out `timeout`, which names the script too and forwards SIGUSR1."""
    out = []
    for p in pids:
        if not match or match not in _cmdline(p):
            continue
        try:
            if not os.path.basename(os.readlink(f"/proc/{p}/exe")).startswith("python"):
                continue
        except OSError:
            continue
        try:
            with open(f"/proc/{p}/status") as f:
                cgt = next((int(x.split()[1], 16) for x in f if x.startswith("SigCgt:")), 0)
        except (OSError, ValueError):
            continue
        if cgt >> (signal.SIGUSR1 - 1) & 1:
            out.append(p)
    return out


def stall_diagnostics(pids, r, out):
    """tools/stall_watch.py's capture_diagnostics for a stalled run, into its output folder (its first retry_move) or
    else beside the bundle's run log; the path, or the error -- never raised, the run is ended either way."""
    folder = r.run.retry_move[0] if r.run.retry_move and os.path.isdir(r.run.retry_move[0]) else None
    path = (os.path.join(folder, "stall_diagnostics.txt") if folder
            else str(Path(out) / "runs" / f"{r.run.name}.{r.attempt}.stall_diagnostics.txt"))
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        try:
            from stall_watch import capture_diagnostics
        finally:
            sys.path.pop(0)
        return capture_diagnostics(pids, path)
    except Exception as exc:  # noqa: BLE001
        return f"<{exc!r}>"


def watch_globs(run, watch_rel=None):
    """A run's progress files: its "watch" globs, else its pace_log, else watch_rel under its first retry_move."""
    if run.watch:
        return list(run.watch)
    if run.pace_log:
        return [run.pace_log]
    if watch_rel and run.retry_move:
        return [os.path.join(run.retry_move[0], watch_rel)]
    return []


def _runner_start(pids, match):
    """The latest start (wall clock) of the Python processes among pids running `match`, or None when none runs."""
    try:
        with open("/proc/stat") as f:
            btime = next(int(x.split()[1]) for x in f if x.startswith("btime "))
    except (OSError, ValueError, StopIteration):
        return None
    starts = []
    for p in pids:
        if not match or match not in _cmdline(p):
            continue
        try:
            if not os.path.basename(os.readlink(f"/proc/{p}/exe")).startswith("python"):
                continue
            with open(f"/proc/{p}/stat") as f:
                starts.append(btime + int(f.read().rsplit(")", 1)[1].split()[19]) / os.sysconf("SC_CLK_TCK"))
        except (OSError, ValueError, IndexError):
            continue
    return max(starts) if starts else None


def progress_silence(run, start_wall, watch_rel=None, now=time.time, runner_start=None):
    """Seconds since the run's progress files last changed, counted at the earliest from the later of its start and
    its runner's start (runner_start: between a live session's epochs, and while a new runner starts up, there is no
    record to wait for); None when the run has no progress file named, so nothing is judged."""
    globs = watch_globs(run, watch_rel)
    if not globs:
        return None
    latest = max(start_wall, runner_start or 0.0)
    for pattern in globs:
        for path in glob.glob(pattern):
            try:
                latest = max(latest, os.stat(path).st_mtime)
            except OSError:
                continue
    return now() - latest


def job_time_left(env, run=subprocess.run, now=time.time):
    """Seconds left before the job's end: the smaller of SLURM_JOB_END_TIME (set at the start, always there in a
    Slurm job) and squeue's %L (which follows a limit shortened by scontrol since); None outside Slurm."""
    left = []
    end = env.get("SLURM_JOB_END_TIME", "")
    if end.isdigit():
        left.append(int(end) - now())
    if env.get("SLURM_JOB_ID"):
        q = slurm_time_left(env["SLURM_JOB_ID"], run)
        if q is not None:
            left.append(q)
    return min(left) if left else None


def slurm_time_left(job_id, run=subprocess.run):
    """Seconds left in the Slurm job's time limit (squeue %L), or None when unknown or unlimited."""
    try:
        text = run(["squeue", "-h", "-j", str(job_id), "-o", "%L"], capture_output=True, text=True,
                   timeout=20).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.fullmatch(r"(?:(\d+)-)?(?:(\d+):)?(\d+):(\d+)", text)
    if not m:
        return None
    d, h, mi, se = (int(x) if x else 0 for x in m.groups())
    return ((d * 24 + h) * 60 + mi) * 60 + se


def wave_warning(n_runs, kind, jobs=1):
    """A warning when the list's last wave leaves half or more of the slots empty, else None. N runs on `jobs` bundle
    jobs of `kind` (its slots per GPU) run in waves of jobs x slots; a last wave of half of them or fewer keeps the
    others' GPUs held and idle for about one run length (rcq: 20 runs on 2 x 4 slots, a last wave of 4 of 8)."""
    per = KINDS[kind]["max_slots"]
    total = per * jobs
    if n_runs < 1 or jobs < 1:
        return None
    last = n_runs - (-(-n_runs // total) - 1) * total
    if last * 2 > total:
        return None
    sizes = [n for n in (n_runs - last, n_runs - last + total) if n > 0]
    fewer = f", or {n_runs} runs on {-(-n_runs // per)} job(s)" if -(-n_runs // per) < jobs else ""
    return (f"warning: {n_runs} runs on {jobs} {kind} job(s) of {per} slots: the last wave runs {last} of {total} "
            f"slots, the rest idle for about a run -- make it {' or '.join(map(str, sizes))} runs{fewer}, and put "
            "the longest runs first")


def kind_slots(kind, shards) -> int:
    """How many sessions of `kind` the job's shards hold (at most the kind's per-GPU limit)."""
    k = KINDS[kind]
    return min(k["max_slots"], shards * SHARD_MIB // k["vram_mib"])  # 0: not even one session fits


def _siblings(cpu):
    try:
        text = Path(f"/sys/devices/system/cpu/cpu{cpu}/topology/thread_siblings_list").read_text().strip()
    except OSError:
        return (cpu,)
    out = set()
    for part in text.split(","):
        a, _, b = part.partition("-")
        out.update(range(int(a), int(b or a) + 1))
    return tuple(sorted(out))


def auto_bridge_cpus(n_cpus, slots) -> int:
    """A core for the bridges when the job has one beyond 2 CPUs per session (what `sizes` asks for), else none."""
    return BRIDGE_CORE_CPUS if n_cpus >= 2 * slots + BRIDGE_CORE_CPUS else 0


def split_bridge_cpus(cpus, n, siblings=_siblings):
    """(bridge CPUs, session CPUs): whole cores from the end of the job's CPUs until at least n CPUs are set apart for
    the SLAM bridge containers; the sessions keep the rest. ([], cpus) when n is 0 or nothing would be left."""
    cpus = sorted(cpus)
    if n <= 0:
        return [], cpus
    cores, seen = [], set()
    for c in cpus:
        if c in seen:
            continue
        group = [x for x in siblings(c) if x in cpus] or [c]
        seen.update(group)
        cores.append(group)
    bridge = []
    while cores and len(bridge) < n:
        bridge = sorted(bridge + cores.pop())
    rest = sorted(c for core in cores for c in core)
    return (bridge, rest) if rest else ([], cpus)


def node_available_mib():
    """MemAvailable of the node in MiB (the bridges and other jobs' containers live outside Slurm's accounting)."""
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) // 1024
    except (OSError, ValueError):
        return None
    return None


class NodeLedger:
    """Node-wide RAM reservations for starts that have not grown into their memory yet, shared by every bundle job on
    the node through a directory under a file lock: a start is admitted while MemAvailable minus the live
    reservations still holds one more session, and its reservation lasts hold_s (a session's memory grows over its
    first minute)."""

    def __init__(self, directory, hold_s=120.0):
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.hold_s = hold_s

    def admit(self, key, need_mib, available) -> bool:
        with open(self.dir / ".lock", "a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            now, reserved = time.time(), 0
            for f in self.dir.glob("r-*"):
                try:
                    mib, until = (float(x) for x in f.read_text().split())
                except (OSError, ValueError):
                    continue
                if until > now:
                    reserved += mib
                else:
                    f.unlink(missing_ok=True)
            avail = available()
            if avail is not None and avail - reserved < need_mib:
                return False
            (self.dir / f"r-{key}").write_text(f"{need_mib} {now + self.hold_s}\n")
            return True

    def release(self, key):
        """Give back a reservation whose session did not start."""
        with open(self.dir / ".lock", "a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            (self.dir / f"r-{key}").unlink(missing_ok=True)


def _proc_identity(pid):
    """[boot id, start time in clock ticks] of a process, or None: a pid alone is reused after a SIGKILL."""
    try:
        with open("/proc/sys/kernel/random/boot_id") as f:
            boot = f.read().strip()
        with open(f"/proc/{pid}/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()
        return [boot, int(fields[19])]  # stat field 22 (fields[0] here is field 3)
    except (OSError, ValueError, IndexError):
        return None


def _runner_alive(entry) -> bool:
    """The registered runner itself is still running: an isaac_bundle with the same boot and start time."""
    pid = entry.get("pid")
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            if b"isaac_bundle" not in f.read():
                return False
    except OSError:
        return False
    return entry.get("ident") is not None and _proc_identity(pid) == entry.get("ident")


def register_list(directory, list_path, claim_dir, job_id, pid=None, alive=_runner_alive, ident=None):
    """Record that this runner runs list_path with claim_dir; return (the conflict, or None; the entry to remove at
    exit, or None). A conflict is another live runner on this node with the same list and not the same claim dir:
    without a shared claim dir both would run every run into the same folders (rcq, 2026-10-25, two jobs on one list
    with none)."""
    d = Path(directory)
    d.mkdir(parents=True, exist_ok=True)
    pid = os.getpid() if pid is None else pid
    key = zlib.crc32(os.path.realpath(list_path).encode())
    mine = os.path.realpath(claim_dir) if claim_dir else None
    with open(d / ".lock", "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        for f in d.glob(f"l-{key:08x}-*"):
            try:
                other = json.loads(f.read_text())
            except (OSError, ValueError):
                continue
            if other.get("list") != os.path.realpath(list_path) or other.get("pid") == pid:
                continue
            if not alive(other):
                f.unlink(missing_ok=True)
                continue
            if mine is None or other.get("claim_dir") != mine:
                return (f"job {other.get('job')} (pid {other.get('pid')}) runs {list_path} with claim dir "
                        f"{other.get('claim_dir')}, this one with {mine}: two jobs on one list need the same "
                        "--claim-dir", None)
        entry = d / f"l-{key:08x}-{job_id}-{pid}"
        entry.write_text(json.dumps({"list": os.path.realpath(list_path), "claim_dir": mine, "job": job_id,
                                     "pid": pid, "ident": ident or _proc_identity(pid)}) + "\n")
        return None, entry


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


def token_pids(token) -> list[int]:
    """Live processes whose environment carries BUNDLE_TOKEN=<token> (the run's whole tree, whatever its groups)."""
    needle = f"BUNDLE_TOKEN={token}".encode()
    found = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/environ", "rb") as f:
                env = f.read()
        except OSError:
            continue
        if needle in env.split(b"\0") and alive(int(d)):
            found.append(int(d))
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
    """Remove the docker containers named slam_bridge_<session_id>_*; return their names (raises on a failure)."""
    names = subprocess.run(
        ["docker", "ps", "-a", "--filter", f"name=^slam_bridge_{session_id}_", "--format", "{{.Names}}"],
        capture_output=True, text=True, timeout=30, check=True,
    ).stdout.split()
    for name in names:
        subprocess.run(["docker", "rm", "-f", name], capture_output=True, timeout=30, check=True)
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
    """Tick records ({tick_t, mono}) of one live log, the last window of them (mono is CLOCK_MONOTONIC, like ours)."""

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

    def measure(self, now):
        """(sim over wall up to now, tick gap p99 counting the silence since the last tick, silence s) over the window
        ending now; None when no tick was ever read. A window that the log does not cover yet gives ratio None."""
        while len(self.ticks) > 1 and now - self.ticks[1][0] >= self.window_s:
            self.ticks.popleft()
        if not self.ticks:
            return None
        silence = max(0.0, now - self.ticks[-1][0])
        start = self.ticks[0][0]
        covered = now - start >= 0.8 * self.window_s
        w0 = max(start, now - self.window_s)
        ratio = (self.ticks[-1][1] - self.ticks[0][1]) / (now - w0) if covered and now > w0 else None
        gaps = [b[0] - a[0] for a, b in zip(self.ticks, list(self.ticks)[1:], strict=False)] + [silence]
        gaps.sort()
        return ratio, gaps[min(len(gaps) - 1, int(0.99 * len(gaps)))], silence


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
    if current_mib and used + session_mib > budget_mib:  # a run alone always fits the job's own count
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
    log_path: Path
    attempt: int = 1
    timeout: bool = False
    stalled: bool = False
    silence_max: float = 0.0
    stack_term_at: float | None = None  # a stalled run's TERM, stack_wait_s after its SIGUSR1
    no_runner_since: float | None = None
    diagnostics: str | None = None
    diag_thread: object = None  # the stalled run's diagnostics, gathered off the loop
    diag_deadline: float | None = None
    stack_pids: list = field(default_factory=list)
    term_at: float | None = None
    killed: bool = False
    vram_max: int = 0
    vram_now: int = 0
    gpus: set = field(default_factory=set)
    oom_at_start: int | None = None
    pace: _Pace | None = None
    pace_min: float | None = None
    # set when the top process has ended; the slot is held until the run's other processes are gone
    exited: dict | None = None
    leftover_kill_at: float | None = None
    empty_since: float | None = None  # the first of the empty token scans that settle the run
    ram_key: str | None = None  # its node-ledger reservation, given back when the run is over


def run_bundle(runs, out, *, slots, stagger_s=60.0, grace_s=110.0, run_timeout_s=None, shards=None,
               shard_mib=SHARD_MIB, session_vram_mib=SESSION_VRAM_MIB, vram_probe=None, vram_period_s=1.0,
               vram_unreadable_s=120.0, job_id=None, cpus=None, cleanup=docker_cleanup, stop=None, poll_s=0.5,
               base_env=None, retries=1, oom_counter=job_oom_kills, pace_floor=None, pace_gap_p99_s=None,
               pace_window_s=300.0, pace_warmup_s=540.0, pace_period_s=60.0, pace_stall_s=120.0, pace_confirm=2,
               pace_recover=10, pace_stall_warmup_s=240.0, run_est_s=None, time_left=None, time_check_s=300.0,
               stall_s=None, stall_check_s=30.0, watch_rel=None, stack_match="run_transport.py", stack_wait_s=3.0,
               diag_wait_s=60.0, no_runner_s=900.0,
               leftover_grace_s=10.0, settle_quiet_s=1.0, gpu_index=None, node_ram_min_mib=None,
               node_ram=node_available_mib, claim_dir=None, node_ledger_dir=None, bridge_cpus=0,
               siblings=_siblings, gpu_full_give_up_s=None):
    """Run the list; return the summary (also written to out/summary.json)."""
    out = Path(out)
    if claim_dir is not None:
        Path(claim_dir).mkdir(parents=True, exist_ok=True)
    ledger = NodeLedger(node_ledger_dir or Path(out) / "node_ledger") if node_ram_min_mib is not None else None
    if (out / "results.jsonl").exists() or (out / "summary.json").exists():
        raise FileExistsError(f"{out} holds an earlier bundle")
    if slots < 1:
        raise ValueError("slots must be at least 1")
    if shards is not None and slots > shards:
        raise ValueError(f"{slots} slots but {shards} shards: a shard per Isaac session running in the job")
    (out / "runs").mkdir(parents=True, exist_ok=True)
    job_id = str(job_id or os.environ.get("SLURM_JOB_ID") or f"local{os.getpid()}")
    cpus = sorted(cpus if cpus is not None else os.sched_getaffinity(0))
    # bridge_cpus: a whole core (or more) for every run's SLAM bridge, the sessions pinned to the other CPUs; "auto"
    # does so when the job has a core beyond 2 CPUs per slot (isaac_bundle.py sizes asks for it), else bridges share
    if bridge_cpus == "auto":
        bridge_cpus = auto_bridge_cpus(len(cpus), slots)
    bridge_set, session_cpus = split_bridge_cpus(cpus, int(bridge_cpus), siblings)
    base_env = dict(os.environ if base_env is None else base_env)
    stop = stop or threading.Event()
    budget = shards * shard_mib if shards else None
    gate_on = vram_probe is not None and budget is not None
    pace_on = pace_floor is not None or pace_gap_p99_s is not None
    nonce = secrets.token_hex(4)

    queue = deque((i, r, 1) for i, r in enumerate(runs))  # (index, list entry, attempt)
    live: dict[int, _Live] = {}  # top pid -> run (until all its processes have ended)
    records = []
    st = {"last_start": None, "sampled_at": None, "unreadable_since": None, "unattributed": 0, "free": None,
          "job_vram_max": 0, "vram_waits": 0, "gpu_full_waits": 0, "unreadable_starts": 0, "waiting": None,
          "waiting_since": None, "gpu_full_gave_up": 0, "busy_at": None,
          "cap": slots, "backoffs": [], "backoff_at": None, "pace_next": 0.0, "infra": 0, "errors": [],
          "pace_below": 0, "pace_clean": 0, "stepups": [], "stepup_at": None, "deadline": None, "time_next": 0.0,
          "time_left": 0, "time_left_retry": [], "stall_next": 0.0, "stalled": 0,
          "node_ram_waits": 0, "claimed_elsewhere": 0}
    t_begin, wall_begin = time.monotonic(), time.time()
    stopping = False
    results_path, vram_path, pace_path = out / "results.jsonl", out / "vram.jsonl", out / "pace.jsonl"

    def record(rec):
        records.append(rec)
        with results_path.open("a") as f:
            f.write(json.dumps(rec) + "\n")

    def claimed_elsewhere(run, attempt):
        """A first attempt another job has claimed (a retry belongs to the job that ran the first)."""
        if claim_dir is None or attempt != 1:
            return False
        try:
            return Path(claim_dir, run.name, "by").read_text().strip() != job_id
        except OSError:
            return os.path.isdir(os.path.join(claim_dir, run.name))  # claimed, owner not written yet

    def drop_unstarted(status):
        while queue:
            index, run, attempt = queue.popleft()
            if claimed_elsewhere(run, attempt):
                st["claimed_elsewhere"] += 1
                continue
            record({"name": run.name, "index": index, "slot": None, "attempt": attempt, "status": status,
                    "rc": None, "signal": None, "retried": False})

    def note_error(where, exc):
        st["errors"].append({"t": round(time.time(), 1), "where": where,
                             "error": exc if isinstance(exc, str) else repr(exc)})

    def top_exited(pid, status, rusage):
        """The run's top process ended: note it, end what it left behind; the slot stays held."""
        r = live[pid]
        r.log.close()
        rc = os.waitstatus_to_exitcode(status) if os.WIFEXITED(status) else None
        sig = os.WTERMSIG(status) if os.WIFSIGNALED(status) else None
        r.proc.returncode = rc if rc is not None else -(sig or 0)
        oom_now = oom_counter() if oom_counter else None
        r.exited = {
            "rc": rc, "signal": sig, "end": time.time(), "wall_s": round(time.monotonic() - r.start, 3),
            "cpu_s": round(rusage.ru_utime + rusage.ru_stime, 2) if rusage else None,
            "maxrss_kib": rusage.ru_maxrss if rusage else None,
            "ooms": (oom_now - r.oom_at_start) if (oom_now is not None and r.oom_at_start is not None) else 0,
        }
        left = token_pids(r.env["BUNDLE_TOKEN"])
        r.exited["leftover"] = len(left)
        if left:
            _kill(left, signal.SIGTERM)
            r.leftover_kill_at = time.monotonic() + leftover_grace_s

    def settle(pid):
        """Every process of the run has ended: classify, clean up, record, retry; the slot frees."""
        r = live.pop(pid)
        if ledger is not None and r.ram_key is not None:
            try:  # its memory is free again, however soon it ended (taskset exec failures too)
                ledger.release(r.ram_key)
            except Exception as exc:  # noqa: BLE001 -- the run is still cleaned up and recorded
                note_error(f"release {r.run.name}", exc)
        x = r.exited
        signs = infra_signs([r.log_path, *(p for g in r.run.scan for p in sorted(glob.glob(g)))])
        killed = x["signal"] == signal.SIGKILL or x["rc"] in (137, -9)
        if x["ooms"] and killed:
            signs.insert(0, f"oom_kill +{x['ooms']} in the job cgroup and the run was killed")
        infra = bool(signs)
        removed, cleanup_error = [], None
        try:
            removed = list(cleanup(r.env["SESSION_ID"]) or [])
        except Exception as exc:  # noqa: BLE001 -- a cleanup failure must not stop the other runs
            cleanup_error = repr(exc)
        retry = (infra and not r.stalled and runs[r.index].retryable and r.attempt <= retries and not stopping
                 and cleanup_error is None)  # a bridge that could not be removed may still hold the run's state
        moved, move_error = [], None
        if retry:
            try:
                for path in r.run.retry_move:
                    if os.path.lexists(path):
                        dest = f"{path}.attempt{r.attempt}"
                        if os.path.lexists(dest):
                            raise FileExistsError(dest)
                        os.rename(path, dest)
                        moved.append(dest)
            except OSError as exc:
                move_error, retry = repr(exc), False
        record({
            "name": r.run.name, "index": r.index, "slot": r.slot, "attempt": r.attempt,
            "status": "infra_failed" if infra else ("stalled" if r.stalled else "ended"), "infra": signs,
            "retried": retry, "stalled": r.stalled, "silence_max_s": round(r.silence_max, 1),
            "stall_diagnostics": r.diagnostics,
            "oom_kills_during": x["ooms"], "pace_min": r.pace_min, "start": r.start_wall, **x,
            "timeout": r.timeout, "killed": r.killed, "session_id": r.env["SESSION_ID"],
            "bridge_cpu": r.env["BRIDGE_CPU"], "port": r.env["PORT"], "vram_max_mib": r.vram_max,
            "gpus": sorted(g for g in r.gpus if g is not None), "log": str(r.log_path),
            "containers_removed": removed, "cleanup_error": cleanup_error, "moved": moved, "move_error": move_error,
        })
        if infra:
            st["infra"] += 1
        if r.stalled:
            st["stalled"] += 1
        if cleanup_error or move_error:
            note_error(f"settle {r.run.name} attempt {r.attempt}", cleanup_error or move_error)
        if retry:
            queue.appendleft((r.index, runs[r.index], r.attempt + 1))

    def start(index, template, attempt, ram_key=None):
        run = template.at(attempt)
        slot = min(set(range(slots)) - {r.slot for r in live.values()})
        env = dict(base_env)
        taken = {int(r.env["PORT"]) for r in live.values() if r.env["PORT"].isdigit()}
        taken |= {int(x.env["PORT"]) for _, x, _ in queue if "PORT" in x.env}
        env.update({
            "SESSION_ID": f"{job_id}s{index}" + (f"r{attempt}" if attempt > 1 else ""),
            "BRIDGE_CPU": ",".join(map(str, bridge_set)) if bridge_set else str(cpus[slot % len(cpus)]),
            "PORT": str(_free_port(job_id, taken)),
            "BUNDLE_SLOT": str(slot), "BUNDLE_JOB": job_id, "BUNDLE_RUN": run.name,
            "BUNDLE_TOKEN": f"{job_id}-{index}-{attempt}-{nonce}",
        })
        env.update(run.env)
        log_path = out / "runs" / f"{run.name}.{attempt}.log"
        log = log_path.open("ab")
        argv = (["taskset", "-c", ",".join(map(str, session_cpus))] + run.argv) if bridge_set else run.argv
        try:
            proc = subprocess.Popen(argv, cwd=run.cwd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
        except OSError as exc:
            log.close()
            record({"name": run.name, "index": index, "slot": slot, "attempt": attempt, "status": "start_failed",
                    "error": str(exc), "start": time.time(), "rc": None, "signal": None, "retried": False})
            return False
        now = time.monotonic()
        live[proc.pid] = _Live(run, index, slot, proc, now, time.time(), env, log, log_path, attempt=attempt,
                               oom_at_start=oom_counter() if oom_counter else None,
                               pace=_Pace(pace_window_s) if run.pace_log else None)
        live[proc.pid].ram_key = ram_key
        st["last_start"] = now
        return True

    def sample_vram(now):
        leaders = [pid for pid, _ in sorted(live.items(), key=lambda kv: kv[1].slot)]
        got = vram_probe(leaders)
        if got is None:
            if st["unreadable_since"] is None:
                st["unreadable_since"] = now
            return
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
        st["unattributed"] = max(0, job_total - sum(per.values()))
        mine = [g for g in job_gpus if g is not None and g in free]
        # the job's GPU: the one its processes use; before any, the only one it can see (ConstrainDevices), else
        # the one Slurm names; a GPU that cannot be told is treated as an unreadable sample (the start waits)
        if mine:
            st["free"] = min(free[g] for g in mine)
        elif len(free) == 1:
            st["free"] = next(iter(free.values()))
        elif gpu_index is not None and gpu_index in free:
            st["free"] = free[gpu_index]
        else:
            st["free"] = None
            if gate_on and st["unreadable_since"] is None:
                st["unreadable_since"] = now
            return
        st["unreadable_since"] = None
        st["job_vram_max"] = max(st["job_vram_max"], job_total)
        st["sampled_at"] = now
        with vram_path.open("a") as f:
            f.write(json.dumps({"t": round(time.time(), 2), "job_mib": job_total, "free_mib": st["free"],
                                "runs": {live[p].run.name: m for p, m in per.items()}}) + "\n")

    def verdict(now):
        if not gate_on:
            return "ok"
        if st["unreadable_since"] is not None:
            return "unreadable_go" if now - st["unreadable_since"] >= vram_unreadable_s else "stale"
        if st["sampled_at"] is None or (st["last_start"] is not None and st["sampled_at"] <= st["last_start"]):
            return "stale"  # judge only on a sample taken after the latest start
        return admits([r.vram_now for r in live.values() if r.exited is None], st["unattributed"],
                      budget_mib=budget, session_mib=session_vram_mib, free_mib=st["free"])

    def evaluate_pace(now):
        # a run's pace is judged from pace_warmup_s (its start-up runs slow), its stalls already from
        # pace_stall_warmup_s: a session that hangs in its first minutes must not wait 9 min to be seen
        rows, unmeasured, early, warming, blind = {}, [], {}, 0, 0
        for r in live.values():
            if r.exited is not None:
                continue
            if r.pace is None:  # running without a pace log: load nothing can measure
                blind += 1
                continue
            age = now - r.start
            if age < pace_warmup_s:
                warming += 1
                if age < pace_stall_warmup_s:
                    continue
                r.pace.read(r.run.pace_log)
                got = r.pace.measure(now)
                if got is not None:
                    early[r.run.name] = round(got[2], 1)
                elif age >= pace_stall_warmup_s + pace_stall_s:
                    early[r.run.name] = round(age, 1)  # never ticked
                continue
            r.pace.read(r.run.pace_log)
            got = r.pace.measure(now)
            if got is None:
                unmeasured.append(r.run.name)
                if age >= pace_warmup_s + pace_stall_s:
                    rows[r.run.name] = {"ratio": None, "gap_p99_s": None, "silence_s": round(age, 1),
                                        "no_tick": True}
                continue
            ratio, gap99, silence = got
            rows[r.run.name] = {"ratio": None if ratio is None else round(ratio, 4), "gap_p99_s": round(gap99, 4),
                                "silence_s": round(silence, 1)}
            if ratio is not None:
                r.pace_min = round(min(ratio, r.pace_min if r.pace_min is not None else ratio), 4)
        # a stall is judged first and on its own: it steps back at once, whatever the pace of the others
        stall = None
        silences = {**early, **{k: v["silence_s"] for k, v in rows.items()}}
        if silences:
            worst = max(silences.items(), key=lambda kv: kv[1])
            if worst[1] >= pace_stall_s:
                stall = f"{worst[0]} has written no tick for {worst[1]} s"
        slow = None
        ratios = sorted(v["ratio"] for v in rows.values() if v["ratio"] is not None)
        if ratios:
            k = len(ratios) // 2
            median = ratios[k] if len(ratios) % 2 else (ratios[k - 1] + ratios[k]) / 2
            if pace_floor is not None and median < pace_floor:
                slow = f"median sim/wall {median:.4f} < {pace_floor}"
        if slow is None and pace_gap_p99_s is not None and any(v["gap_p99_s"] is not None for v in rows.values()):
            worst = max(((k, v) for k, v in rows.items() if v["gap_p99_s"] is not None), key=lambda kv: kv[1]["gap_p99_s"])
            if worst[1]["gap_p99_s"] > pace_gap_p99_s:
                slow = f"{worst[0]} tick gap p99 {worst[1]['gap_p99_s']} s > {pace_gap_p99_s}"
        reason = stall or slow
        cooled = st["backoff_at"] is None or now - st["backoff_at"] >= pace_window_s
        running = sum(1 for r in live.values() if r.exited is None)
        stepped = False
        # a stall steps back at once; a slow window only after pace_confirm evaluations in a row (one window dipping
        # below the floor is within a single session's own spread: lfv4 single runs' 300 s minima 0.3467-0.3648)
        st["pace_below"] = st["pace_below"] + 1 if slow else 0
        confirmed = stall is not None or (slow is not None and st["pace_below"] >= pace_confirm)
        if confirmed and cooled and st["cap"] > 1:
            st["cap"] = max(1, min(st["cap"], running) - 1)
            st["backoff_at"] = now
            st["backoffs"].append({"t": round(time.time(), 1), "reason": reason, "cap": st["cap"]})
            stepped = True
        # back up by one after pace_recover clean evaluations in a row: a back-off is otherwise for the rest of the
        # job, and one taken on a false alarm halves the bundle for hours. Clean = every running run has a pace log,
        # is past its warm-up and measured, none slow or stalled -- a start, a run still warming, one not measured
        # or one without a pace log restarts the count, so the cap only goes up on the load that is running. The
        # step-up has its own wait: it does not delay the next back-off.
        whole = (bool(rows) and not warming and not unmeasured and not blind
                 and all(v["ratio"] is not None for v in rows.values()))
        st["pace_clean"] = st["pace_clean"] + 1 if whole and reason is None else 0
        stepped_up = False
        up_cooled = cooled and (st["stepup_at"] is None or now - st["stepup_at"] >= pace_window_s)
        if not stepped and st["cap"] < slots and st["pace_clean"] >= pace_recover and up_cooled:
            st["cap"] += 1
            st["stepup_at"] = now
            st["pace_clean"] = 0
            st["stepups"].append({"t": round(time.time(), 1), "cap": st["cap"]})
            stepped_up = True
        with pace_path.open("a") as f:
            f.write(json.dumps({"t": round(time.time(), 1), "runs": rows, "unmeasured": unmeasured, "early": early,
                                "cap": st["cap"],
                                "reason": reason, "stepped_back": stepped,
                                "stepped_up": stepped_up}) + "\n")

    def end_all(now):
        for r in live.values():
            if r.exited is None and r.term_at is None:
                _kill([r.proc.pid], signal.SIGTERM)
                r.term_at = now

    def loop():
        nonlocal stopping
        vram_next = 0.0
        while queue or live:
            now = time.monotonic()
            if live:  # the job's own runs (ending ones too) may still free the device: no give-up counts from before
                st["busy_at"] = now
            for pid, r in list(live.items()):
                if r.exited is None:
                    try:
                        got, status, rusage = os.wait4(pid, os.WNOHANG)
                    except ChildProcessError:
                        got, status, rusage = pid, 0, None
                    if got == pid:
                        top_exited(pid, status, rusage)
                if r.exited is not None:
                    # a process can fork between a scan's listing and its reads: one empty scan settles nothing,
                    # two at least settle_quiet_s apart do
                    left = token_pids(r.env["BUNDLE_TOKEN"])
                    if left:
                        r.empty_since = None
                        r.exited["leftover"] = max(r.exited["leftover"], len(left))
                        if r.leftover_kill_at is None:
                            _kill(left, signal.SIGTERM)
                            r.leftover_kill_at = now + leftover_grace_s
                        elif now >= r.leftover_kill_at:
                            _kill(left, signal.SIGKILL)
                    elif r.empty_since is None:
                        r.empty_since = now
                    elif now - r.empty_since >= settle_quiet_s:
                        settle(pid)
            if stop.is_set() and not stopping:
                stopping = True
                end_all(now)
                drop_unstarted("not_started")
            for r in live.values():
                if r.exited is not None:
                    continue
                if run_timeout_s is not None and r.term_at is None and now - r.start > run_timeout_s:
                    r.timeout, r.term_at = True, now
                    _kill([r.proc.pid], signal.SIGTERM)
                if (not r.stalled and r.term_at is None and now >= st["stall_next"]
                        and watch_globs(r.run, watch_rel)):
                    # measured for every watched run (silence_max_s), acted on with stall_s; judged only while its
                    # runner (run_transport) runs, from that runner's start
                    started = _runner_start(token_pids(r.env["BUNDLE_TOKEN"]), stack_match) if stack_match else 0.0
                    silence, limit = None, stall_s
                    if started is not None:
                        r.no_runner_since = None
                        silence = progress_silence(r.run, r.start_wall, watch_rel, runner_start=started)
                        r.silence_max = max(r.silence_max, silence)
                    elif no_runner_s:
                        # no runner: a live session between epochs (backoff, bridge start, standby: ~7 min at most)
                        # is no stall, a run stuck without a runner is, after no_runner_s
                        r.no_runner_since = now if r.no_runner_since is None else r.no_runner_since
                        silence, limit = now - r.no_runner_since, no_runner_s
                    if stall_s and silence is not None:
                        if silence >= limit:
                            # first what it was doing: diagnostics in a thread (py-spy, /proc, nvidia-smi -- they
                            # must not hold up the other runs), at most diag_wait_s
                            r.stalled = True
                            r.stack_pids = [p for p in token_pids(r.env["BUNDLE_TOKEN"])
                                            if stack_match and stack_match in _cmdline(p)]
                            r.diag_thread = threading.Thread(target=lambda r=r: setattr(
                                r, "diagnostics", stall_diagnostics(r.stack_pids, r, out)), daemon=True)
                            r.diag_thread.start()
                            r.diag_deadline = now + diag_wait_s
                if (r.stalled and r.stack_term_at is None and r.term_at is None
                        and (not r.diag_thread.is_alive() or now >= r.diag_deadline)):
                    # then the stack: SIGUSR1 only to processes that catch it (run_transport's faulthandler)
                    _kill(stall_targets(r.stack_pids, stack_match), signal.SIGUSR1)
                    r.stack_term_at = now + stack_wait_s
                if r.stalled and r.term_at is None and r.stack_term_at is not None and now >= r.stack_term_at:
                    r.term_at = now  # then ended like a timeout: TERM, KILL after the grace, never retried
                    _kill([r.proc.pid], signal.SIGTERM)
                if r.term_at is not None and not r.killed and now - r.term_at > grace_s:
                    r.killed = True
                    _kill(set(token_pids(r.env["BUNDLE_TOKEN"])) | {r.proc.pid}, signal.SIGKILL)
            if now >= st["stall_next"]:
                st["stall_next"] = now + stall_check_s
            if vram_probe is not None and now >= vram_next:
                vram_next = now + vram_period_s
                sample_vram(now)
            if pace_on and now >= st["pace_next"]:
                st["pace_next"] = now + pace_period_s
                evaluate_pace(now)
            while queue and claimed_elsewhere(queue[0][1], queue[0][2]):  # before any wait: nothing to wait for
                queue.popleft()
                st["claimed_elsewhere"] += 1
            # the job's time limit (re-read: scontrol can shorten it): a run that would not end before it is not
            # started -- left unclaimed for another job on the list instead of being cut by Slurm's TERM
            if run_est_s and time_left is not None and now >= st["time_next"]:
                st["time_next"] = now + time_check_s
                left = time_left()
                if left is not None:
                    st["deadline"] = now + left
            if (not stopping and queue and st["deadline"] is not None
                    and st["deadline"] - now < run_est_s + TERM_EARLY_S + grace_s + 60):
                # a first attempt goes back unclaimed for another job on the list; a retry of ours keeps its claim
                # (its outputs are this job's attempts) and is recorded apart: someone has to run it again
                while queue:
                    index, run, attempt = queue.popleft()
                    if claimed_elsewhere(run, attempt):
                        st["claimed_elsewhere"] += 1
                        continue
                    status = "time_left" if attempt == 1 else "time_left_retry"
                    record({"name": run.name, "index": index, "slot": None, "attempt": attempt, "status": status,
                            "rc": None, "signal": None, "retried": False})
                    st["time_left"] += 1
                    if attempt > 1:
                        st["time_left_retry"].append(run.name)
            if (not stopping and queue and len(live) < min(slots, st["cap"])
                    and (st["last_start"] is None or now - st["last_start"] >= stagger_s)):
                v = verdict(now)
                index, run, attempt = queue[0]
                key = f"{job_id}-{index}-{attempt}-{nonce}"
                if v in ("ok", "unreadable_go") and ledger is not None:
                    # the node's own memory: Slurm reserves the cgroups, not the bridge containers or what others
                    # run outside Slurm; the ledger keeps two jobs from both spending the same free memory
                    if not ledger.admit(key, node_ram_min_mib, node_ram):
                        v = "node_ram"
                if v in ("ok", "unreadable_go"):
                    st["waiting"] = None
                    if v == "unreadable_go":
                        st["unreadable_starts"] += 1
                    queue.popleft()
                    mine, started = None, False
                    try:
                        if attempt == 1 and claim_dir is not None:
                            try:  # several bundle jobs share one list: the first to claim a run runs it
                                os.mkdir(os.path.join(claim_dir, run.name))
                            except FileExistsError:
                                st["claimed_elsewhere"] += 1
                                continue
                            mine = os.path.join(claim_dir, run.name)
                            Path(mine, "by").write_text(job_id + "\n")
                        started = start(index, run, attempt, key if ledger is not None else None)
                    except Exception as exc:  # noqa: BLE001 -- recorded; the claim goes back for another job
                        note_error(f"start {run.name}", exc)
                        if mine is not None:
                            shutil.rmtree(mine, ignore_errors=True)
                        record({"name": run.name, "index": index, "slot": None, "attempt": attempt,
                                "status": "start_failed", "error": repr(exc), "rc": None, "signal": None,
                                "retried": False})
                    finally:
                        if not started and ledger is not None:
                            try:  # nothing started: the memory was not taken
                                ledger.release(key)
                            except Exception as exc:  # noqa: BLE001
                                note_error(f"release {run.name}", exc)
                    continue
                if v in ("over", "gpu_full", "node_ram") and st["waiting"] != v:
                    st[{"over": "vram_waits", "gpu_full": "gpu_full_waits", "node_ram": "node_ram_waits"}[v]] += 1
                    st["waiting"] = v
                    st["waiting_since"] = now
                elif v not in ("over", "gpu_full", "node_ram"):  # an unreadable or stale sample breaks the wait
                    st["waiting"], st["waiting_since"] = None, None
                if (v == "gpu_full" and gpu_full_give_up_s and not live
                        and now - max(st["waiting_since"], st["busy_at"] or 0.0) >= gpu_full_give_up_s):
                    # nothing of this job runs, so nothing of ours will free the device: other jobs' sessions hold
                    # it (11596_2, 2026-10-27: three S3 sessions grown to 3,920-4,018 MiB, 464 MiB the driver
                    # reserves, 3,837 MiB free -- the fourth shard of a GPU without the memory for a fourth session;
                    # the task held its shard, CPUs and memory for 54 min). Slurm hands shards out from the first
                    # GPU, so a requeued task would likely land on the same device: end, the list's owner resubmits.
                    while queue:
                        index, run, attempt = queue.popleft()
                        if claimed_elsewhere(run, attempt):  # another job of the list has it: not ours to record
                            st["claimed_elsewhere"] += 1
                            continue
                        record({"name": run.name, "index": index, "slot": None, "attempt": attempt,
                                "status": "gpu_full", "rc": None, "signal": None, "retried": False,
                                "free_mib": st["free"], "need_mib": session_vram_mib, "gpu": gpu_index,
                                "waited_s": round(now - max(st["waiting_since"], st["busy_at"] or 0.0), 1)})
                        st["gpu_full_gave_up"] += 1
            time.sleep(poll_s)

    try:
        loop()
    except BaseException as exc:
        # an error of the bundle itself: end every run (TERM, then KILL by token after the grace) before leaving
        note_error("loop", exc)
        traceback.print_exc()
        stop.set()
        now = time.monotonic()
        end_all(now)
        deadline = now + grace_s
        while any(alive(r.proc.pid) or token_pids(r.env["BUNDLE_TOKEN"]) for r in live.values()):
            if time.monotonic() > deadline:
                for r in live.values():
                    _kill(set(token_pids(r.env["BUNDLE_TOKEN"])) | {r.proc.pid}, signal.SIGKILL)
                break
            time.sleep(poll_s)
        for r in live.values():
            try:
                got, status = os.waitpid(r.proc.pid, 0 if not alive(r.proc.pid) else os.WNOHANG)
                if got == r.proc.pid:
                    r.proc.returncode = os.waitstatus_to_exitcode(status)
            except ChildProcessError:
                r.proc.returncode = r.proc.returncode if r.proc.returncode is not None else -1
            r.log.close()
            removed, cleanup_error = [], None
            try:
                removed = list(cleanup(r.env["SESSION_ID"]) or [])
            except Exception as cexc:  # noqa: BLE001
                cleanup_error = repr(cexc)
            record({"name": r.run.name, "index": r.index, "slot": r.slot, "attempt": r.attempt,
                    "status": "aborted", "rc": None, "signal": None, "retried": False,
                    "containers_removed": removed, "cleanup_error": cleanup_error})
        drop_unstarted("not_started")

    final = [r for r in records if not r.get("retried")]  # the last attempt of each run
    failed = sum(1 for r in final if r["status"] in ("infra_failed", "stalled")
                 or (r["status"] == "ended" and r["rc"] != 0))
    not_started = sum(1 for r in final if r["status"] in ("not_started", "start_failed", "aborted", "time_left",
                                                          "time_left_retry", "gpu_full"))
    summary = {
        "runs": len(runs) - st["claimed_elsewhere"], "failed": failed, "not_started": not_started, "slots": slots, "stagger_s": stagger_s,
        "shards": shards, "vram_gate": gate_on, "vram_waits": st["vram_waits"],
        "gpu_full_waits": st["gpu_full_waits"], "gpu_full_gave_up": st["gpu_full_gave_up"], "unreadable_starts": st["unreadable_starts"],
        "node_ram_waits": st["node_ram_waits"], "claimed_elsewhere": st["claimed_elsewhere"],
        "infra_failed_attempts": st["infra"], "retries": retries, "backoffs": st["backoffs"], "stepups": st["stepups"],
        "time_left": st["time_left"], "time_left_retry": st["time_left_retry"], "run_est_s": run_est_s,
        "stalled": st["stalled"], "stall_s": stall_s,
        "final_cap": st["cap"], "vram_job_max_mib": st["job_vram_max"], "budget_mib": budget, "job_id": job_id,
        "node_gpu": gpu_index, "cpus": cpus, "bridge_cpus": bridge_set, "session_cpus": session_cpus if bridge_set else cpus,
        "errors": st["errors"], "start": wall_begin, "wall_s": round(time.monotonic() - t_begin, 3),
        "exit_code": 0 if failed == 0 and not_started == 0 and not st["errors"] else 1,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    return summary


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv[:1] == ["check"]:
        cp = argparse.ArgumentParser(prog="isaac_bundle.py check",
                                     description="check a run list before submitting it (see check_runs)")
        cp.add_argument("--list", required=True)
        cp.add_argument("--runner", required=True, help="the snapshot's runner, e.g. SNAP/sim/isaac/run_transport.py")
        cp.add_argument("--kind", choices=sorted(KINDS), help="with --jobs: warn when the last wave is half empty")
        cp.add_argument("--jobs", type=int, default=1, help="bundle jobs sharing the list (--claim-dir)")
        a = cp.parse_args(argv[1:])
        runs = load_runs(a.list)
        problems = check_runs(runs, a.runner)
        for line in problems:
            print(line)
        warning = wave_warning(len(runs), a.kind, a.jobs) if a.kind else None
        if warning:
            print(warning)
        print(f"{len(problems)} problems")
        return 1 if problems else 0
    if argv[:1] == ["collect"]:
        cp = argparse.ArgumentParser(prog="isaac_bundle.py collect",
                                     description="the final state of every task of an array (deploy/slurm/isaac_array.sbatch)")
        cp.add_argument("out", help="the array's OUT")
        cp.add_argument("--expect", type=int, required=True, help="the list's runs (the array's tasks)")
        a = cp.parse_args(argv[1:])
        rows, missing = collect(a.out, a.expect)
        for r in rows:
            print(json.dumps(r))
        for i in missing:
            print(json.dumps({"task": i, "status": "missing"}))
        bad = missing + [r["task"] for r in rows if r["status"] != "ended" or r["rc"] != 0 or r["summary_exit"] != 0]
        print(f"{len(rows)} of {a.expect} tasks recorded, {len(missing)} missing, {len(bad)} not ended with rc 0 "
              "and a clean summary")
        return 1 if bad else 0
    if argv[:1] == ["sizes"]:
        sp = argparse.ArgumentParser(prog="isaac_bundle.py sizes",
                                     description="sbatch options for a bundle of measured session kinds")
        sp.add_argument("kind", choices=sorted(KINDS))
        sp.add_argument("--slots", type=int)
        sp.add_argument("--runs", type=int, help="the list's runs: also print --time")
        sp.add_argument("--jobs", type=int, default=1, help="bundle jobs sharing the list")
        sp.add_argument("--run-min", type=float, help="a run's p95 wall minutes (default: the kind's measured)")
        sp.add_argument("--array", action="store_true",
                        help="one run per array task (deploy/slurm/isaac_array.sbatch); needs --runs and --run-min")
        a = sp.parse_args(argv[1:])
        if a.array:
            try:
                z = array_sizes(a.kind, a.runs, a.run_min)
            except ValueError as exc:
                print(exc, file=sys.stderr)
                return 2
            print(f"--array={z['array']} --gres={z['gres']} --cpus-per-task={z['cpus']} --mem={z['mem_mib']}M "
                  f"--time={z['time']}")
            print(f"(then: deploy/slurm/isaac_array.sbatch RUNS OUT {a.kind}, with BUNDLE_ARGS=\"--run-est-min "
                  f"{a.run_min:g}\")", file=sys.stderr)
            if a.kind == "live":
                print("note: a live task takes 2 shards (2 a GPU); 3-slot bundles (sizes live --slots 3) hold 3 a GPU",
                      file=sys.stderr)
            return 0
        try:
            z = sbatch_sizes(a.kind, a.slots, a.runs, a.jobs, a.run_min)
        except ValueError as exc:
            print(exc, file=sys.stderr)
            return 2
        print(f"--gres={z['gres']} --cpus-per-task={z['cpus']} --mem={z['mem_gib']}G"
              + (f" --time={z['time']}" if "time" in z else ""))
        print(f"(then: deploy/slurm/isaac_bundle.sbatch RUNS OUT {a.kind})", file=sys.stderr)
        if a.run_min is not None:
            print(f"(and BUNDLE_ARGS=\"... --run-est-min {a.run_min:g}\": the runner starts no run with less than its "
                  "estimate left, and its default is the kind's)", file=sys.stderr)
        return 0
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--list", required=True, help="the run list (JSON lines)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--kind", choices=sorted(KINDS),
                    help="the sessions' kind: its measured GPU memory per session and the slots the shards hold")
    ap.add_argument("--slots", type=int, help="runs at once (default: what the shards hold for --kind, else shards)")
    ap.add_argument("--shards", type=int, help="the job's shards (default: SLURM_SHARDS_ON_NODE)")
    ap.add_argument("--stagger-s", type=float, default=60.0)
    ap.add_argument("--session-vram-mib", type=int, help=f"default: --kind's, else {SESSION_VRAM_MIB}")
    ap.add_argument("--shard-mib", type=int, default=SHARD_MIB)
    ap.add_argument("--vram-period-s", type=float, default=1.0)
    ap.add_argument("--vram-unreadable-s", type=float, default=120.0)
    ap.add_argument("--run-timeout-s", type=float)
    ap.add_argument("--grace-s", type=float, default=110.0)
    ap.add_argument("--retries", type=int, default=1, help="reruns of a run that failed for the infrastructure")
    ap.add_argument("--pace-floor", type=float, help="step back when the median sim/wall of live runs is below")
    ap.add_argument("--pace-gap-p99-s", type=float, help="step back when a live run's tick gap p99 is above")
    ap.add_argument("--pace-window-s", type=float, default=300.0)
    ap.add_argument("--pace-warmup-s", type=float, default=540.0,
                    help="a run is judged from this age: its start-up (~240 s) plus a window")
    ap.add_argument("--pace-period-s", type=float, default=60.0)
    # 120 s: healthy live runs went 36-42 s without a tick (escb 2026-10-26: four back-offs at 36.6-42.5 s; rcq up
    # to 41 s), so 30 s took a session away from bundles that were at pace; a real hang is the stall watchdog's
    ap.add_argument("--pace-stall-s", type=float, default=120.0)
    ap.add_argument("--pace-stall-warmup-s", type=float, default=240.0,
                    help="a run's stalls are judged from this age (its pace from --pace-warmup-s)")
    ap.add_argument("--pace-confirm", type=int, default=2,
                    help="evaluations in a row a slow window or tick gap must persist before a step back")
    ap.add_argument("--pace-recover", type=int, default=10,
                    help="clean evaluations in a row (and a window since the last change) before the cap goes up one")
    ap.add_argument("--run-est-min", type=float,
                    help="a run's p95 wall minutes: no run starts with less than this (+ grace) left in the job's "
                         "time limit (default: --kind's measured; 0: off)")
    ap.add_argument("--stall-min", type=float,
                    help="end a run whose progress files (its watch globs, else its pace_log, else the kind's file "
                         "under its output) have not changed for this long, status stalled, not retried (default: "
                         "--kind's; 0: off)")
    ap.add_argument("--list-registry-dir", default=f"/tmp/isaac_bundle_lists_{os.getuid()}",
                    help="where this node's bundle runners record their run list and claim dir")
    ap.add_argument("--node-ledger-dir", default=f"/tmp/isaac_bundle_node_ledger_{os.getuid()}",
                    help="node-wide RAM reservations shared by this user's bundle jobs")
    ap.add_argument("--gpu-full-give-up-min", type=float, default=0.0,
                    help="with nothing of the job running, stop waiting for a GPU that has stayed full this long and "
                         "record the unstarted runs as gpu_full (0: wait; the array script gives 10)")
    ap.add_argument("--bridge-cpus", default="auto",
                    help="CPUs set apart for the SLAM bridges, the sessions pinned to the rest (auto: a core when the "
                         "job has one beyond 2 CPUs per slot; 0: bridges share)")
    ap.add_argument("--claim-dir", help="share the list with other bundle jobs: a run goes to the first to claim it")
    ap.add_argument("--node-ram-min-mib", type=int, default=KINDS["riib"]["session_gib"] * 1024 + BRIDGE_MIB,
                    help="start only while the node's MemAvailable is at least this (0: off)")
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
    if a.session_vram_mib is None:
        a.session_vram_mib = KINDS[a.kind]["vram_mib"] if a.kind else SESSION_VRAM_MIB
    if a.slots:
        slots = a.slots
    elif a.kind and shards:
        slots = kind_slots(a.kind, shards)
    else:  # no kind: as many sessions of --session-vram-mib as the shards hold, at most one a shard
        slots = min(shards, shards * a.shard_mib // a.session_vram_mib) if shards else None
    if not slots:
        print("refusing: no --slots and no shard count (--shards or SLURM_SHARDS_ON_NODE), or the shards hold no "
              "session of --kind", file=sys.stderr)
        return 2
    if shards is not None and slots > shards:
        print(f"refusing: {slots} slots but {shards} shards (a shard per Isaac session)", file=sys.stderr)
        return 2
    if shards is not None and slots * a.session_vram_mib > shards * a.shard_mib:
        need = -(-slots * a.session_vram_mib // a.shard_mib)
        print(f"refusing: {slots} sessions of {a.session_vram_mib} MiB need {need} shards, the job has {shards}",
              file=sys.stderr)
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
    job_id = os.environ.get("SLURM_JOB_ID") or f"local{os.getpid()}"
    est_min = a.run_est_min if a.run_est_min is not None else (KINDS[a.kind]["run_min"] if a.kind else 0)
    left = job_time_left(os.environ) if est_min and os.environ.get("SLURM_JOB_ID") else None
    if left is not None and left < 60 * est_min + TERM_EARLY_S + a.grace_s + 60:
        print(f"warning: the job has {left / 60:.0f} min left and a run is estimated at {est_min:g} min: no run will "
              "start (each is left as time_left) -- pass --run-est-min with the runs' real length, or a longer --time",
              file=sys.stderr)
    conflict, entry = register_list(a.list_registry_dir, a.list, a.claim_dir, job_id)
    if conflict:
        print(f"refusing: {conflict}", file=sys.stderr)
        return 2
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())
    try:
        summary = _run_main(a, runs, slots, shards, probe, stop)
    finally:
        entry.unlink(missing_ok=True)
    print(json.dumps(summary))
    return summary["exit_code"]


def _run_main(a, runs, slots, shards, probe, stop):
    return run_bundle(
        runs, a.out, slots=slots, stagger_s=a.stagger_s, grace_s=a.grace_s, run_timeout_s=a.run_timeout_s,
        shards=shards, shard_mib=a.shard_mib, session_vram_mib=a.session_vram_mib, vram_probe=probe,
        vram_period_s=a.vram_period_s, vram_unreadable_s=a.vram_unreadable_s,
        cleanup=(lambda sid: []) if a.no_cleanup else docker_cleanup, stop=stop, retries=a.retries,
        pace_floor=a.pace_floor, pace_gap_p99_s=a.pace_gap_p99_s, pace_window_s=a.pace_window_s,
        pace_warmup_s=a.pace_warmup_s, pace_period_s=a.pace_period_s, pace_stall_s=a.pace_stall_s,
        pace_confirm=a.pace_confirm, pace_recover=a.pace_recover, pace_stall_warmup_s=a.pace_stall_warmup_s,
        run_est_s=60 * (a.run_est_min if a.run_est_min is not None else (KINDS[a.kind]["run_min"] if a.kind else 0)),
        time_left=(lambda: job_time_left(os.environ)) if os.environ.get("SLURM_JOB_ID") else None,
        stall_s=60 * (a.stall_min if a.stall_min is not None else (KINDS[a.kind].get("stall_min", 0) if a.kind else 0)),
        watch_rel=KINDS[a.kind].get("watch_rel") if a.kind else None,
        gpu_index=job_gpu_index(os.environ), node_ram_min_mib=a.node_ram_min_mib or None, claim_dir=a.claim_dir,
        node_ledger_dir=a.node_ledger_dir,
        bridge_cpus=a.bridge_cpus if a.bridge_cpus == "auto" else int(a.bridge_cpus),
        gpu_full_give_up_s=60 * a.gpu_full_give_up_min or None,
    )


if __name__ == "__main__":
    sys.exit(main())
