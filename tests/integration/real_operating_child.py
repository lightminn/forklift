"""TEST ONLY: the operating REAL run (``tools/real_autonomy.run_real``) as a child of the end-to-end test
(tests/integration/test_real_run_operating.py, operating plan 2026-11-26 6.1).

The command line refuses a pty, so this child calls ``run_real`` itself with the test's transport factory. The test
hands the child its pty's slave fd (``fd``, inherited); the child takes that fd only -- never a path -- and only when
it is a pty slave: anything else (a real tty, a file) and the child refuses before the lock is dropped or any test
identity is given (Codex U3e-1 implementation round 1 P1). On that fd, ``RealTtyTransport.open`` runs as on the board
(the executor lock taken first, the fd set raw) with the operating product string and serial the test gives. Every write the transport makes -- each frame's first write, each continuation and
the frame-less resync bytes -- is logged with its byte count (``writes.jsonl``), so "no motion byte" is judged on
what reached the device. With ``drop_lock`` the precondition judgement is run as it is and only
``precondition:unlock_incomplete`` is taken out of its list (pytest's stand-in for U3e-2); every other reason stays.

usage: python real_operating_child.py <config.json>"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))

import real_autonomy as tool  # noqa: E402
import real_loop_transport as rlt  # noqa: E402

from forklift_core.live import real_preconditions as rp  # noqa: E402


def _the_tests_pty(fd: int) -> bool:
    import os
    import stat

    from forklift_core.live.model_peer import PTY_SLAVE_MAJORS

    try:
        st = os.fstat(fd)
    except OSError:
        return False
    return stat.S_ISCHR(st.st_mode) and os.major(st.st_rdev) in PTY_SLAVE_MAJORS


def main() -> int:
    cfg = json.loads(Path(sys.argv[1]).read_text())
    fd = int(cfg["fd"])
    if not _the_tests_pty(fd):
        print(json.dumps({"run": False, "refused": ["child_fd_not_a_pty"]}), flush=True)
        return 2  # before the judgement is touched or any identity handed out
    if cfg.get("drop_lock"):
        judged = rp.operating_preconditions

        def without_lock(facts):
            return [r for r in judged(facts) if r != rp.UNLOCK_INCOMPLETE]

        rp.operating_preconditions = without_lock
    log = open(cfg["writes_log"], "a", buffering=1)  # noqa: SIM115 -- open for the run's whole life

    class LoggedTransport(rlt.RealTtyTransport):
        def _os_write(self, data, path, frame=None):
            n = super()._os_write(data, path, frame)
            log.write(json.dumps({"ns": time.monotonic_ns(), "path": path, "kind": getattr(frame, "kind", None),
                                  "seq": getattr(frame, "seq", None), "n": int(n or 0)}) + "\n")
            return n

    def opener(path, before_config=None):
        if before_config is not None:
            before_config(fd)  # the executor lock on this very fd first, as on the board
        rlt._raw(fd)  # noqa: SLF001
        os.set_blocking(fd, False)  # as the board's open_raw leaves it: a full pty is EAGAIN, never a wait
        return fd

    def factory(device):
        return LoggedTransport.open(device, opener=opener, allow_pty=True, product_reader=lambda _fd: cfg["product"],
                                    serial_reader=lambda _fd: cfg["serial"])

    args = tool.build_parser().parse_args(cfg["argv"])
    try:
        return tool.run_real(args, transport_factory=factory)
    finally:
        log.close()


if __name__ == "__main__":
    sys.exit(main())
