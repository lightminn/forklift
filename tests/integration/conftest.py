"""Integration-test hooks: the teleop end-to-end harness's loop evidence (D5 loop_slow hypothesis, lead 2026-11-16).

The harness turns loop_slow off (it is not real time) and records, for every arm attempt, whether loop_slow WOULD
have fired. Workers (xdist) append those lines to one file named in the environment; the terminal summary prints
them so they land in the run's log.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

ENV = "FORKLIFT_LOOP_EVIDENCE"


def pytest_configure(config):
    if ENV not in os.environ and not hasattr(config, "workerinput"):  # the controller only; workers inherit it
        fd, path = tempfile.mkstemp(prefix="forklift_loop_evidence_", suffix=".jsonl")
        os.close(fd)
        os.environ[ENV] = path


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    path = os.environ.get(ENV)
    if not path or hasattr(config, "workerinput") or not Path(path).exists():
        return
    lines = [x for x in Path(path).read_text(encoding="utf-8").splitlines() if x.strip()]
    if lines:
        terminalreporter.section("teleop harness loop evidence (loop_slow off)")
        for line in lines:
            terminalreporter.write_line("LOOP_EVIDENCE " + line)
