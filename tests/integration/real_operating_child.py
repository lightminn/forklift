"""TEST ONLY: the operating REAL run (``tools/real_autonomy.run_real``) as a child of the end-to-end test
(tests/integration/test_real_run_operating.py, operating plan 2026-11-26 6.1).

The command line refuses a pty, so this child calls ``run_real`` itself with the test's transport factory: the
test's pty to the MCU model opened by ``RealTtyTransport.open(..., allow_pty=True)`` with the operating product
string and serial the test gives. Every write the transport makes -- each frame's first write, each continuation and
the frame-less resync bytes -- is logged with its byte count (``writes.jsonl``), so "no motion byte" is judged on
what reached the device. With ``drop_lock`` the precondition judgement is run as it is and only
``precondition:unlock_incomplete`` is taken out of its list (pytest's stand-in for U3e-2); every other reason stays.

usage: python real_operating_child.py <config.json>"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))

import real_autonomy as tool  # noqa: E402
import real_loop_transport as rlt  # noqa: E402

from forklift_core.live import real_preconditions as rp  # noqa: E402


def main() -> int:
    cfg = json.loads(Path(sys.argv[1]).read_text())
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

    def factory(device):
        return LoggedTransport.open(device, allow_pty=True, product_reader=lambda fd: cfg["product"],
                                    serial_reader=lambda fd: cfg["serial"])

    args = tool.build_parser().parse_args(cfg["argv"])
    try:
        return tool.run_real(args, transport_factory=factory)
    finally:
        log.close()


if __name__ == "__main__":
    sys.exit(main())
