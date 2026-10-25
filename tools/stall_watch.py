"""Progress watchdog for a single Isaac job (deploy/console/isaac_live.sbatch starts it beside its supervisor).

While --term-pid lives, every --period-s: the silence is now minus the newest mtime of the --glob matches (the job's
start while none exists yet). Once it reaches --stall-min, for the descendants of --term-pid whose command line holds
--stack-match: capture_diagnostics (py-spy --native, threads' state/times/wchan, /proc status, nvidia-smi) to
--diagnostics, then SIGUSR1 (sim/isaac/run_transport.py dumps every thread's Python stack on it), --stack-wait-s later
SIGTERM to
--term-pid (the supervisor ends its runner and bridge, as on Slurm's TERM), --grace-s later SIGKILL to what is left
of its tree. --marker gets a JSON line saying so. Standard library only.

Why: 2026-10-26 two sessions hung -- dv3_trailer_det2 in world.render() for 45 min, lfsolo 11119 silent for 34 min
mid-transport until its time limit -- each holding a GPU. The bundle runner (tools/isaac_bundle.py) has the same
watchdog for its runs; a single job had none.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import signal
import sys
import time


def newest_mtime(patterns, since):
    latest = since
    for pattern in patterns:
        for path in glob.glob(pattern):
            try:
                latest = max(latest, os.stat(path).st_mtime)
            except OSError:
                continue
    return latest


def descendants(root):
    """The pids under `root` (not root itself), from /proc."""
    children = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/stat") as f:
                ppid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            continue
        children.setdefault(ppid, []).append(int(entry))
    out, todo = [], [root]
    while todo:
        for c in children.get(todo.pop(), []):
            out.append(c)
            todo.append(c)
    return out


def cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read().replace(b"\0", b" ").decode(errors="replace")
    except OSError:
        return ""


def alive(pid):
    """Running, not a zombie (an ended process its parent has not reaped yet still answers kill 0)."""
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rsplit(")", 1)[1].split()[0] != "Z"
    except (OSError, IndexError):
        return False


PY_SPY_CANDIDATES = ("py-spy", "/home/projects/forklift/.venv/bin/py-spy")


def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError as exc:
        return f"<{exc}>"


def _run(argv, timeout=20):
    import subprocess

    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return f"(exit {r.returncode})\n{r.stdout}{r.stderr}"
    except (OSError, subprocess.SubprocessError) as exc:
        return f"<{exc!r}>"


def capture_diagnostics(pids, out_path, py_spy=None, now=time.time, spy_timeout=20, smi_timeout=10):
    """Before a stalled run is ended: what its processes were doing, written to out_path. For each pid: py-spy dump
    --native (sim/isaac/run_transport.py allows it with PR_SET_PTRACER_ANY: Yama scope 1 lets only an ancestor attach,
    and py-spy is not one), then every thread's state, utime, stime and wchan, and /proc/<pid>/status; last the GPU's
    per-process memory and SM use (nvidia-smi). Every step is tried on its own; a failure is written, not raised."""
    import shutil

    spy = py_spy or next((p for p in (shutil.which(c) or c for c in PY_SPY_CANDIDATES) if os.path.exists(p)), None)
    lines = [f"stall diagnostics {time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(now()))} pids {list(pids)}"]
    for pid in pids:
        lines.append(f"\n=== pid {pid}: {cmdline(pid)[:300]}")
        lines.append("--- py-spy dump --native" + ("" if spy else ": py-spy not found"))
        if spy:
            lines.append(_run([spy, "dump", "--native", "--pid", str(pid)], spy_timeout))
        lines.append("--- threads: tid comm state utime stime (clock ticks) wchan")
        try:
            tids = sorted(os.listdir(f"/proc/{pid}/task"), key=int)
        except OSError as exc:
            tids = []
            lines.append(f"<{exc}>")
        for tid in tids:
            stat = _read(f"/proc/{pid}/task/{tid}/stat")
            try:
                comm = stat[stat.index("(") + 1:stat.rindex(")")]
                rest = stat.rsplit(")", 1)[1].split()
                state, utime, stime = rest[0], rest[11], rest[12]
            except (ValueError, IndexError):
                comm, state, utime, stime = "?", "?", "?", "?"
            lines.append(f"{tid} {comm} {state} {utime} {stime} {_read(f'/proc/{pid}/task/{tid}/wchan').strip()}")
        lines.append("--- /proc/status")
        lines.append(_read(f"/proc/{pid}/status"))
    lines.append("--- nvidia-smi compute apps (pid, used memory)")
    lines.append(_run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader"], smi_timeout))
    lines.append("--- nvidia-smi pmon (one sample: sm % per process)")
    lines.append(_run(["nvidia-smi", "pmon", "-c", "1", "-s", "um"], smi_timeout))
    with open(out_path, "a") as f:
        f.write("\n".join(lines) + "\n")
    return out_path


def catches(pid, sig):
    """The process has a handler for `sig` (/proc/<pid>/status SigCgt): SIGUSR1 goes only to such processes -- to
    one without (bash, timeout, a helper) it would be the end of it."""
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("SigCgt:"):
                    return bool(int(line.split()[1], 16) >> (int(sig) - 1) & 1)
    except (OSError, ValueError, IndexError):
        pass
    return False


def is_python(pid):
    """The process's executable is a Python interpreter (timeout, which names run_transport.py in its argv and
    forwards SIGUSR1 to its group, is not)."""
    try:
        return os.path.basename(os.readlink(f"/proc/{pid}/exe")).startswith("python")
    except OSError:
        return False


def stack_targets(pids, match):
    """The pids of Python interpreters that run `match` (sim/isaac/run_transport.py) and catch SIGUSR1 (its
    faulthandler)."""
    return [p for p in pids if match and match in cmdline(p) and is_python(p) and catches(p, signal.SIGUSR1)]


def identity(pid):
    """(pid, start time): a pid alone may be reused once its process is gone."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            return pid, int(f.read().rsplit(")", 1)[1].split()[19])
    except (OSError, ValueError, IndexError):
        return None


def kill_left(identities, sig):
    """Signal the captured processes that still run as the same process."""
    for ident in identities:
        if ident is not None and identity(ident[0]) == ident:
            try:
                os.kill(ident[0], sig)
            except OSError:
                pass


def signal_all(pids, sig):
    for pid in pids:
        try:
            os.kill(pid, sig)
        except OSError:
            pass


def watch(patterns, stall_s, term_pid, *, stack_match="run_transport.py", stack_wait_s=3.0, grace_s=110.0,
          period_s=30.0, marker=None, diag_path=None, start=None, now=time.time, sleep=time.sleep):
    """Returns "ended" when term_pid went away by itself, "stalled" after ending it."""
    start = now() if start is None else start
    while alive(term_pid):
        silence = now() - newest_mtime(patterns, start)
        if silence >= stall_s:
            tree = descendants(term_pid)
            owned = [identity(p) for p in [term_pid, *tree]]  # the tree now: what is left of it is killed at the end
            matched = [p for p in tree if stack_match and stack_match in cmdline(p)]
            stacked = stack_targets(matched, stack_match)  # SIGUSR1 only to those with the handler
            diag = None
            if diag_path:
                try:
                    diag = capture_diagnostics(matched, diag_path, now=now)
                except Exception as exc:  # noqa: BLE001 -- the run is ended whatever the diagnostics did
                    diag = f"<{exc!r}>"
            signal_all(stacked, signal.SIGUSR1)  # faulthandler in run_transport: every thread's Python stack
            if marker:
                try:  # a marker that cannot be written must not keep the stalled session alive
                    with open(marker, "a") as f:
                        f.write(json.dumps({"wall": round(now(), 1), "event": "stalled",
                                            "silence_s": round(silence, 1), "globs": list(patterns),
                                            "stack_pids": stacked, "diagnostics": diag}) + "\n")
                except OSError:
                    pass
            sleep(stack_wait_s)
            signal_all([term_pid], signal.SIGTERM)
            deadline = now() + grace_s
            while alive(term_pid) and now() < deadline:
                sleep(1.0)
            # what is left of the tree captured before TERM -- reparented children included -- and anything new under
            # term_pid
            kill_left(owned + [identity(p) for p in descendants(term_pid)], signal.SIGKILL)
            return "stalled"
        sleep(period_s)
    return "ended"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--glob", action="append", required=True, help="a progress file glob (repeatable)")
    ap.add_argument("--stall-min", type=float, required=True, help="0: do nothing")
    ap.add_argument("--term-pid", type=int, required=True)
    ap.add_argument("--stack-match", default="run_transport.py")
    ap.add_argument("--stack-wait-s", type=float, default=3.0)
    ap.add_argument("--grace-s", type=float, default=110.0)
    ap.add_argument("--period-s", type=float, default=30.0)
    ap.add_argument("--marker")
    ap.add_argument("--diagnostics", help="where capture_diagnostics writes (default: stall_diagnostics.txt beside "
                                          "--marker)")
    a = ap.parse_args(argv)
    if a.stall_min <= 0:
        return 0
    result = watch(a.glob, 60 * a.stall_min, a.term_pid, stack_match=a.stack_match, stack_wait_s=a.stack_wait_s,
                   grace_s=a.grace_s, period_s=a.period_s, marker=a.marker,
                   diag_path=a.diagnostics or (os.path.join(os.path.dirname(a.marker), "stall_diagnostics.txt")
                                               if a.marker else None))
    print(f"stall_watch: {result}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
