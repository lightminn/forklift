"""The obstacle layer's scan refresh off the control thread (real-time plan 5.28, docs/plans/2026-10-17-realtime-sim.md).

A live run with ``--layer-worker delay|thread`` submits each scan's refresh here instead of computing it in place:
the layer prepares the job on the main thread (``ObstacleLayer.refresh_prepare(detach=True)``), the computation
(``refresh_compute``) runs at once (``delay``) or on this worker's thread (``thread``), and the result is installed on
the main thread (``refresh_install``) at the earlier of

- the scheduled tick: the control tick of physics tick ``scan_tick + k`` (``due``), waiting there if it is not done;
- a barrier: the main thread is about to touch what the computation owns or reads (``install_now(reason)``, called by
  the layer before a synchronous refresh, a planner grid, a new scan, a copy, the exemption's or the corridor's
  change, the exit latch's close; and by the runner before a permission reset and at the session's end).

``delay`` and ``thread`` share the pending state, the barriers and the install rule; only where the computation runs
differs (plan 5.28 D1). One job is pending at a time.

While a thread job runs, the grid and the shadow memory are the worker's (D3). Its timers go to a list of its own
(``perf.Timers.capture``) and are added to the shared timers at the install, on the main thread (D7).

A failure (the computation raised, or it did not end within ``timeout_s``) raises ``LayerWorkerFailure`` at the
install; the runner ends the session (D6). The thread cannot be cancelled: after a failure the layer is not touched
again (``failed``).
"""

from __future__ import annotations

import threading
import time

from forklift_core.perf import TIMERS


class LayerWorkerFailure(RuntimeError):
    """The layer worker's job raised or timed out (its message starts ``layer_worker_failed``)."""


class LayerWorker:
    def __init__(self, layer, mode: str, k: int, *, clock=time.perf_counter, timeout_s: float = 0.5,
                 on_install=None):
        if mode not in ("delay", "thread"):
            raise ValueError(f"layer worker mode {mode!r}: delay or thread")
        if not 1 <= int(k) <= 11:
            raise ValueError(f"layer worker k {k}: 1..11 (below the scan period of 12 ticks)")
        self.layer, self.mode, self.k = layer, mode, int(k)
        self.clock, self.timeout_s, self.on_install = clock, float(timeout_s), on_install
        self.records: list[dict] = []
        self._slack: list[dict] = []  # the recent jobs' records waiting for their next control ticks' times
        self.failed: str | None = None
        self._job = None
        self._info: dict | None = None
        self._result = None
        self._error: BaseException | None = None
        self._done = threading.Event()
        self._go = threading.Event()
        self._thread = None
        if mode == "thread":
            self._thread = threading.Thread(target=self._loop, name="layer-worker", daemon=True)
            self._thread.start()

    @property
    def pending(self) -> bool:
        return self._job is not None

    def submit(self, job: dict, scan_tick: int, *, extra: dict | None = None) -> None:
        """A scan's refresh (prepared, detached) for physics tick ``scan_tick``; installed at ``scan_tick + k`` or
        at an earlier barrier."""
        if self.failed:
            raise LayerWorkerFailure(f"layer_worker_failed: submit after {self.failed}")
        if self._job is not None:
            self.install_now("submit")  # never two pending (a scan inside a capture's steps)
        self._job = job
        self._info = {"scan_tick": int(scan_tick), "due_tick": int(scan_tick) + self.k, "submit_t": self.clock(),
                      **(extra or {})}
        self._slack.append(self._info)
        self._result, self._error = None, None
        self._done.clear()
        if self.mode == "delay":
            self._run()
        else:
            self._go.set()

    def due(self, tick: int) -> bool:
        return self._job is not None and int(tick) >= self._info["due_tick"]

    def install_due(self, tick: int):
        """At a control tick's start: install when the scheduled tick has come (None otherwise)."""
        if self.due(tick):
            return self.install_now("scheduled", tick=tick)
        return None

    def tick_start(self, tick: int):
        """A control tick begins (before any mission work): its time is kept for the jobs of the last few scans --
        1e-0's paired slack, the time from a submit to the control start of scan_tick + 1..4 -- and the pending job
        is installed when its tick has come."""
        now = self.clock()
        keep = []
        for rec in self._slack:
            offset = int(tick) - rec["scan_tick"]
            if 1 <= offset <= 4:
                rec.setdefault("tick_t", {})[offset] = now
            if offset < 4:
                keep.append(rec)
        self._slack = keep
        return self.install_due(tick)

    def install_now(self, reason: str, *, tick: int | None = None):
        """Wait for the pending job (up to ``timeout_s``), then install it on this (the main) thread."""
        if self._job is None:
            return None
        if self.failed:
            raise LayerWorkerFailure(f"layer_worker_failed: {self.failed}")
        start = self.clock()
        if not self._done.wait(self.timeout_s):
            self.failed = f"timeout after {self.timeout_s} s ({reason})"
            raise LayerWorkerFailure(f"layer_worker_failed: {self.failed}")
        waited = self.clock() - start
        if self._error is not None:
            self.failed = f"{type(self._error).__name__}: {self._error}"
            raise LayerWorkerFailure(f"layer_worker_failed: {self.failed}") from self._error
        job, info, result = self._job, self._info, self._result
        self._job = self._info = self._result = None
        for entry in result.pop("timers", ()):
            TIMERS.add(*entry)  # the worker's timers, on the main thread (D7)
        t0 = self.clock()
        check = self.layer.refresh_install(job, result)
        info.update(reason=reason, install_tick=tick, wait_s=waited, install_start_t=start,
                    install_s=self.clock() - t0)
        self.records.append(info)
        if self.on_install is not None:
            self.on_install(job, check, info)
        return check

    def _run(self) -> None:
        info = self._info
        t0, c0 = self.clock(), time.thread_time()
        try:
            if self.mode == "thread":
                with TIMERS.capture() as captured:
                    result = self.layer.refresh_compute(self._job)
                result["timers"] = captured
            else:
                result = self.layer.refresh_compute(self._job)
            self._result = result
        except BaseException as exc:  # noqa: BLE001 -- raised at the install, on the main thread
            self._error = exc
        finally:
            info["compute_s"] = self.clock() - t0
            info["compute_cpu_s"] = time.thread_time() - c0
            info["done_t"] = self.clock()
            self._done.set()

    def _loop(self) -> None:
        while True:
            self._go.wait()
            self._go.clear()
            self._run()

    def summary(self) -> dict:
        reasons: dict = {}
        for r in self.records:
            reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
        return {"mode": self.mode, "k": self.k, "installs": len(self.records), "reasons": reasons,
                "failed": self.failed}
