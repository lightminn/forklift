"""The real-time runner's planning worker (real-time plan D8a, docs/plans/2026-10-17-realtime-sim.md).

A real-time run asks for its long plans here instead of computing them in its control loop: one worker process,
started with ``python -m forklift_core.planning.plan_worker`` (not multiprocessing's spawn, which would import the
Isaac runner's main module again), talking over two inherited pipes, so whatever a planner prints does not mix
with the messages. A batch run never starts one.

Jobs (each a module-level planner and a copy of its arguments):

- ``plan_transport``: ``pallet_mission.plan_transport(*args, **kwargs)``; a ``trace`` list in the keyword
  arguments comes back filled.
- ``leg``: ``plan_transport_leg`` or ``plan_return_leg``, then once more with the tight arguments when the first
  status is one of ``retry_on`` -- the caller's own fallback, run here so both share one deadline.
- ``dock``: ``scan_docking.dock(*args, **kwargs)`` and its wall time (real-time plan D9b).
- ``observe_ladder``: a live mission's first leg (persistent console plan D2, agreed with the main session
  2026-10-18): the setup's candidate ladder -- every waypoint with ``extended=False``, then, only if none plans,
  every one with ``extended=True`` -- stopping at the first success or, between candidates, at the payload's
  absolute ``deadline`` (this host's monotonic clock), which every call also gets. The batch setup keeps its own
  loop: the replay gate records each ``plan_observation_leg`` call, so this is an intended duplicate.

``PlanWorker`` is the runner's side. The control thread never waits on the worker (Codex real-time D8 round 4
P1): ``submit`` pickles the request at once (its inputs are fixed from that moment) and queues the bytes;
``poll`` only looks at finished answers; ``discard`` only marks. One daemon thread sends, receives whole messages and
restart the worker -- when the job it has is discarded (so a new request does not wait behind it) or when it
died (the jobs it had and nobody discarded are written to the new one).
"""

from __future__ import annotations

import os
import pickle
import queue
import subprocess
import sys
import threading
import time
import traceback
from multiprocessing.connection import Connection, wait


def worker_env(mode: str) -> dict:
    """The environment for a worker that runs the kernels ``mode`` (persistent console plan D11)."""
    return {**os.environ, "FORKLIFT_KERNELS": str(mode)}


class WorkerFailure(RuntimeError):
    """The worker could not be started."""


def _run(name: str, payload: dict):
    from forklift_core.planning import pallet_mission as pm

    if name == "plan_transport":
        kwargs = dict(payload["kwargs"])
        trace = kwargs.get("trace")
        result = pm.plan_transport(*payload["args"], **kwargs)
        return {"result": result, "trace": trace}
    if name == "leg":
        fn = {"transport": pm.plan_transport_leg, "return": pm.plan_return_leg}[payload["kind"]]
        result = fn(*payload["args"], **payload["kwargs"])
        tight = False
        if result.status in payload.get("retry_on", ()):
            result = fn(*payload["tight_args"], **payload["tight_kwargs"])
            tight = True
        return {"result": result, "tight": tight}
    if name == "dock":  # real-time plan D9b: the delivery docking match
        from forklift_core.localization.scan_docking import dock

        started = time.monotonic()
        result = dock(*payload["args"], **payload["kwargs"])
        return {"result": result, "wall_s": time.monotonic() - started}
    if name == "observe_ladder":
        return _observe_ladder(pm, payload)
    if name == "echo":  # the tests' job
        if payload.get("sleep_s"):
            time.sleep(payload["sleep_s"])
        if payload.get("raise"):
            raise ValueError(payload["raise"])
        return {"result": payload.get("value")}
    raise ValueError(f"unknown job {name!r}")


def _observe_ladder(pm, payload: dict) -> dict:
    """The setup's ladder (run_transport, ``initial_candidates``), each candidate recorded with the keys of
    ``state["observation_candidates"]``; every candidate tried comes back, failed or not."""
    from forklift_core.planning.geometry import Pose2D

    started = time.monotonic()
    world = payload["world"]
    kwargs = dict(payload["grid_kwargs"])
    # One deadline for every call: the payload's whole-ladder one, else what the grid arguments carried (Codex P2:
    # grid_kwargs("observe") returns obstacle["deadline"] when one is set, so passing both raised a TypeError).
    deadline = payload.get("deadline") if payload.get("deadline") is not None else kwargs.get("deadline")
    kwargs["deadline"] = deadline
    start = world.start_rear
    candidates, result, selected, timed_out = [], None, None, False
    for extended in (False, True):
        for index, coordinates in enumerate(payload["waypoints"]):
            if deadline is not None and time.monotonic() >= deadline:
                timed_out = True
                break
            waypoint = Pose2D(*coordinates)
            plan = pm.plan_observation_leg(world, waypoint, payload["config"], geometry=payload["geometry"],
                                           pickup_bounds=payload["pickup_bounds"], extended=extended, **kwargs)
            candidates.append({"candidate_index": index, "pose": [waypoint.x_m, waypoint.y_m, waypoint.yaw_rad],
                               "success": plan.success, "status": plan.status,
                               "analytic_expansion_interval": plan.analytic_expansion_interval,
                               "search_attempts": [list(entry) for entry in plan.search_attempts],
                               "start_rear": [start.x_m, start.y_m, start.yaw_rad], "extended_ladder": extended})
            if plan.success:
                result, selected = plan, index
                break
        if result is not None or timed_out:
            break
    out = {"result": result, "selected": selected, "candidates": candidates, "timed_out": timed_out}
    if result is None and not timed_out and candidates and all(c["status"] == "invalid_start" for c in candidates):
        # Persistent console plan D13 item 6: how far straight back the start check passes -- the planner's own
        # check, built from this job's own frozen inputs, within the same deadline (Codex D13 rounds 1-2).
        free, clearance = pm.observation_start_checker(world, payload["config"], geometry=payload["geometry"],
                                                       pickup_bounds=payload["pickup_bounds"], **kwargs)
        out["advice"] = pm.backoff_advice(free, clearance, start, deadline=deadline)
    out["wall_s"] = time.monotonic() - started
    return out


def main(argv: list[str]) -> int:
    """The worker: a reader thread takes every request off the pipe at once (so the runner's writes never wait for
    a computation), the main thread computes them in order and writes the answers."""
    conn_in, conn_out = Connection(int(argv[0]), writable=False), Connection(int(argv[1]), readable=False)
    inbox: queue.Queue = queue.Queue()

    def read() -> None:
        while True:
            try:
                inbox.put(pickle.loads(conn_in.recv_bytes()))
            except (EOFError, OSError):
                inbox.put(None)
                return

    threading.Thread(target=read, name="plan-worker-read", daemon=True).start()
    # Persistent console plan D11: the kernels the environment names (the live runner passes its own), compiled
    # before the first job -- a new interpreter would otherwise plan on the reference kernels, slower than the live
    # one and so judged differently against the candidates' wall-clock deadline.
    from forklift_core import kernels

    if kernels.mode() != "reference":
        kernels.warm_up()
    conn_out.send_bytes(pickle.dumps(("ready", os.getpid(), kernels.mode())))
    while True:
        message = inbox.get()
        if message is None:
            return 0
        job_id, name, payload = message
        try:
            answer = (job_id, True, _run(name, payload))
            out = pickle.dumps(answer, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as exc:  # noqa: BLE001 -- the job's failure (or an unpicklable answer) goes back as text
            out = pickle.dumps((job_id, False, f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}"))
        conn_out.send_bytes(out)


class PlanWorker:
    """The runner's handle on the worker: one job computes at a time, later ones wait in order.

    One daemon I/O thread owns the pipes and the process: it writes the queued requests (the worker's reader takes
    them at once, so a write does not wait for a computation), reads whole answers, and restarts the worker when the
    job it is computing is discarded (so a new request does not wait behind it) or when it died. Jobs the old
    process had and nobody discarded are written again to the new one. (Codex real-time D8 implementation rounds
    5-6: two threads sharing the pipes raced on their lifetime.)"""

    def __init__(self, *, ready_timeout_s: float = 120.0, env: dict | None = None):
        self.ready_timeout_s = ready_timeout_s
        self.env = env
        self.kernels: str | None = None  # what the process said it runs (plan D11)
        self.restarts = 0
        self._lock = threading.Lock()  # guards the bookkeeping below, never held across pipe or process work
        self._outbox: queue.Queue = queue.Queue()  # (job_id, bytes)
        self._blobs: dict[int, bytes] = {}  # every job not yet answered nor discarded, to write again after a restart
        self._done: dict[int, tuple] = {}
        self._discarded: set[int] = set()
        self._sent: list[int] = []  # written to the current process, unanswered, in order
        self._restart = threading.Event()
        self._stopping = threading.Event()
        self._dead: str | None = None  # why there is no worker any more: every job from then on fails at once
        self._next = 0
        self.proc: subprocess.Popen | None = None
        self._send = self._recv = None
        self._start_process()
        self._thread = threading.Thread(target=self._io, name="plan-worker-io", daemon=True)
        self._thread.start()

    # --- the control thread's side: none of these waits on the worker ---------------------------------------------
    def submit(self, name: str, payload: dict) -> int:
        """Queue a job; its inputs are pickled now, so later changes to them do not reach the job."""
        with self._lock:
            self._next += 1
            job_id = self._next
        blob = pickle.dumps((job_id, name, payload), protocol=pickle.HIGHEST_PROTOCOL)
        with self._lock:
            if self._dead is not None:  # no worker to run it (Codex real-time D8 implementation round 7 P2)
                self._done[job_id] = (False, self._dead)
                return job_id
            self._blobs[job_id] = blob
        self._outbox.put(job_id)
        return job_id

    def poll(self, job_id: int):
        """None while the job runs (or waits); else ``(ok, value)`` -- the job's dict, or the failure text."""
        with self._lock:
            return self._done.pop(job_id, None)

    def discard(self, job_id: int) -> None:
        """Forget a job; if the worker is computing it, the I/O thread restarts the worker."""
        with self._lock:
            self._done.pop(job_id, None)
            self._blobs.pop(job_id, None)
            self._discarded.add(job_id)
            if self._sent and self._sent[0] == job_id:
                self._restart.set()

    def close(self) -> None:
        self._stopping.set()
        self._thread.join(timeout=self.ready_timeout_s + 15)
        self._stop_process()

    # --- the I/O thread only ---------------------------------------------------------------------------------------
    def _start_process(self) -> None:
        fds: list[int] = []
        try:
            to_child_r, to_child_w = os.pipe()
            fds += [to_child_r, to_child_w]
            from_child_r, from_child_w = os.pipe()
            fds += [from_child_r, from_child_w]
            proc = subprocess.Popen(
                [sys.executable, "-m", "forklift_core.planning.plan_worker", str(to_child_r), str(from_child_w)],
                pass_fds=(to_child_r, from_child_w), env=self.env, stdin=subprocess.DEVNULL)
        except Exception as exc:  # noqa: BLE001 -- no process: its pipes closed, the same failure as not ready
            for fd in fds:
                os.close(fd)
            raise WorkerFailure(f"plan worker could not start: {exc}") from exc
        os.close(to_child_r)
        os.close(from_child_w)
        self.proc = proc
        self._send, self._recv = Connection(to_child_w, readable=False), Connection(from_child_r, writable=False)
        deadline = time.monotonic() + self.ready_timeout_s
        while not wait([self._recv], timeout=0.1):
            if self._stopping.is_set() or time.monotonic() > deadline:
                self._stop_process()
                raise WorkerFailure("plan worker not ready")
        try:
            tag, _pid, *rest = pickle.loads(self._recv.recv_bytes())
            self.kernels = rest[0] if rest else None
        except (OSError, EOFError) as exc:
            self._stop_process()
            raise WorkerFailure(f"plan worker gone before ready: {exc}") from exc
        if tag != "ready":
            self._stop_process()
            raise WorkerFailure(f"plan worker said {tag!r} first")

    def _stop_process(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait(timeout=10)
        for conn in (self._send, self._recv):
            if conn is not None:
                conn.close()
        self._send = self._recv = None

    def _restart_process(self, died: str | None = None) -> None:
        """A new process; the jobs the old one had and nobody discarded are written to it again, first -- except,
        when it died (``died``), the one it was computing, which fails (it may be what killed it)."""
        with self._lock:
            again = [j for j in self._sent if j not in self._discarded]
            self._sent = []
            if died is not None and again:
                culprit = again.pop(0)
                self._blobs.pop(culprit, None)
                self._done[culprit] = (False, died)
        self._stop_process()
        try:
            self._start_process()
        except WorkerFailure as exc:
            with self._lock:
                self._dead = f"plan worker restart failed: {exc}"
                for job_id in again + list(self._blobs):
                    self._done.setdefault(job_id, (False, self._dead))
                self._blobs.clear()
            raise
        self.restarts += 1
        for job_id in again:
            self._write(job_id)

    def _write(self, job_id: int) -> None:
        with self._lock:
            blob = self._blobs.get(job_id)
            if blob is None:  # discarded meanwhile
                return
            self._sent.append(job_id)
        self._send.send_bytes(blob)  # the worker's reader takes it at once

    def _io(self) -> None:
        try:
            while not self._stopping.is_set():
                if self._restart.is_set():
                    self._restart.clear()
                    self._restart_process()
                    continue
                try:
                    job_id = self._outbox.get_nowait()
                except queue.Empty:
                    job_id = None
                try:
                    if job_id is not None:
                        self._write(job_id)
                    if self._recv.poll(0.005 if job_id is None else 0):
                        got, ok, value = pickle.loads(self._recv.recv_bytes())
                        with self._lock:
                            if got in self._sent:
                                self._sent.remove(got)
                            self._blobs.pop(got, None)
                            if got not in self._discarded:
                                self._done[got] = (ok, value)
                    elif self.proc.poll() is not None:
                        raise EOFError("exited")
                except (OSError, EOFError):
                    if not self._stopping.is_set():
                        code = self.proc.wait(timeout=10) if self.proc is not None else None
                        self._restart_process(died=f"plan worker exited with {code}")
        except Exception as exc:  # noqa: BLE001 -- whatever ended this thread, no job may wait for it any more
            with self._lock:
                if self._dead is None:
                    self._dead = f"plan worker I/O ended: {type(exc).__name__}: {exc}"
                for job_id in list(self._sent) + list(self._blobs):
                    self._done.setdefault(job_id, (False, self._dead))
                self._sent, self._blobs = [], {}
            return
        finally:
            if self._stopping.is_set() and self._send is not None:
                try:
                    self._send.send_bytes(pickle.dumps(None))
                except OSError:
                    pass


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
