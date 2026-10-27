"""The probe's wait for a running session: it ends at once when the session's launcher has ended (lvram, 2026-10-27)."""
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


def test_a_launcher_that_ended_before_its_session_ran_ends_the_wait(tmp_path):
    done = subprocess.Popen([sys.executable, "-c", "pass"])
    done.wait()
    clock = _Clock()
    info, why = lcp.wait_port(tmp_path / "live_port.json", 900.0, done.pid, sleep=clock.sleep, clock=clock)
    assert info is None and why == "launcher_ended" and clock.t <= 1.0
    # a "running" port file an earlier session left behind does not stand in for the ended launcher's session
    (tmp_path / "live_port.json").write_text(json.dumps({"state": "running", "port": 8800}))
    assert lcp.wait_port(tmp_path / "live_port.json", 900.0, done.pid, sleep=clock.sleep, clock=clock) == (
        None, "launcher_ended")


def test_a_running_session_is_returned_and_a_missing_one_times_out(tmp_path):
    port = tmp_path / "live_port.json"
    port.write_text(json.dumps({"state": "running", "port": 8800}))
    clock = _Clock()
    assert lcp.wait_port(port, 900.0, None, sleep=clock.sleep, clock=clock) == ({"state": "running", "port": 8800},
                                                                                 None)
    clock = _Clock()
    assert lcp.wait_port(tmp_path / "none.json", 5.0, None, sleep=clock.sleep, clock=clock) == (None, "no_port_file")


def test_the_probe_exits_3_with_its_reason_when_the_launcher_ended(tmp_path):
    done = subprocess.Popen([sys.executable, "-c", "pass"])
    done.wait()
    report = tmp_path / "probe.json"
    p = subprocess.run([sys.executable, str(ROOT / "tools" / "live_console_probe.py"), "--port-file",
                        str(tmp_path / "live_port.json"), "--report", str(report), "--scenario", "cycle",
                        "--launcher-pid", str(done.pid)], capture_output=True, text=True, timeout=60)
    assert p.returncode == 3 and json.loads(report.read_text()) == {"port_file": None, "error": "launcher_ended"}
