"""Pure planning calls in forked processes, for runs with cores to spare.

``Forked(fn, *args)`` starts ``fn(*args)`` in a forked child at once (the
arguments are inherited, not pickled); ``result()`` waits for its value and
returns it (or raises with the child's traceback); ``cancel()`` stops a child
whose value is no longer wanted. Only for pure functions: whatever the child
changes in its copy of the parent's state is lost. Linux (fork) only, and
for a parent without threads: run it with OMP_NUM_THREADS=1 and
OPENBLAS_NUM_THREADS=1 (as the gate1 workers do) so that NumPy's BLAS starts
none -- a child forked from a threaded process can deadlock.
"""

from __future__ import annotations

import multiprocessing as mp
import traceback

_CTX = mp.get_context("fork")


def _run(conn, fn, args) -> None:
    try:
        conn.send((True, fn(*args)))
    except BaseException:  # noqa: BLE001 -- handed to the parent as text
        conn.send((False, traceback.format_exc()))
    finally:
        conn.close()


class Forked:
    """fn(*args) running in a forked child."""

    def __init__(self, fn, *args) -> None:
        self._recv, send = _CTX.Pipe(duplex=False)
        self._process = _CTX.Process(target=_run, args=(send, fn, args))
        self._process.start()
        send.close()
        self._value = None

    def ready(self, timeout_s: float = 0.0) -> bool:
        """Has the value arrived (waiting up to timeout_s for it)?"""
        return self._value is not None or self._recv.poll(timeout_s)

    def result(self):
        if self._value is None:
            self._value = self._recv.recv()
            self._recv.close()
            self._process.join()
        ok, value = self._value
        if not ok:
            raise RuntimeError(f"forked planning call failed:\n{value}")
        return value

    def cancel(self) -> None:
        if self._value is None:
            self._process.terminate()
            self._recv.close()
        self._process.join()
