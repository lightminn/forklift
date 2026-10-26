import json
import os
import subprocess
import sys
import time
from pathlib import Path

from tools import stall_watch as sw

# a stand-in for sim/isaac/run_transport.py: its name is what the watchdogs match, it registers the same faulthandler
# dump on SIGUSR1 and then hangs
HANG = (
    "import faulthandler,signal,sys,time\n"
    "faulthandler.register(signal.SIGUSR1, file=open(sys.argv[1], 'w'), all_threads=True)\n"
    "print('ready', flush=True)\n"
    "def stuck():\n"
    "    time.sleep(60)\n"
    "stuck()\n"
)


def fake_runner(tmp_path):
    script = tmp_path / "run_transport.py"
    script.write_text(HANG)
    return script


def test_capture_diagnostics_writes_threads_and_status(tmp_path):
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
        time.sleep(0.3)
        out = sw.capture_diagnostics([child.pid], tmp_path / "diag.txt", py_spy="/nonexistent/py-spy")
        text = Path(out).read_text()
    finally:
        child.kill()
        child.wait()
    assert f"=== pid {child.pid}" in text and "threads: tid comm state utime stime" in text
    assert f"{child.pid} python" in text or f"{child.pid} " in text
    assert "--- /proc/status" in text and "VmRSS" in text
    assert "nvidia-smi" in text  # tried; its failure is written, not raised


def test_a_stalled_session_leaves_its_stack_and_is_ended(tmp_path):
    script = fake_runner(tmp_path)
    stack = tmp_path / "run.stall_stack.txt"
    sup = subprocess.Popen(["bash", "-c", f"{sys.executable} {script} {stack}; true"])
    time.sleep(1.0)
    marker = tmp_path / "stall.jsonl"
    result = sw.watch([str(tmp_path / "never.jsonl")], 0.5, sup.pid, stack_wait_s=0.5, grace_s=3.0, period_s=0.1,
                      marker=str(marker), diag_path=str(tmp_path / "diag.txt"))
    sup.wait(timeout=10)
    assert result == "stalled"
    assert "stuck" in stack.read_text()  # faulthandler's dump of the hung frame
    rec = json.loads(marker.read_text())
    assert rec["event"] == "stalled" and rec["stack_pids"] and rec["diagnostics"].endswith("diag.txt")
    assert "threads:" in (tmp_path / "diag.txt").read_text()


def test_progress_keeps_the_session(tmp_path):
    log = tmp_path / "live_log.jsonl"
    writer = subprocess.Popen([sys.executable, "-c",
                               f"import time\nfor _ in range(15):\n    open({str(log)!r},'a').write('x')\n"
                               "    time.sleep(0.1)"])
    result = sw.watch([str(log)], 0.6, writer.pid, period_s=0.1)
    writer.wait()
    assert result == "ended" and writer.returncode == 0


def test_zero_minutes_is_off(tmp_path):
    assert sw.main(["--glob", str(tmp_path / "x"), "--stall-min", "0", "--term-pid", str(os.getpid())]) == 0


def test_sigusr1_goes_only_to_processes_that_catch_it(tmp_path):
    # Codex: timeout's argv also names run_transport.py, and SIGUSR1 would end a process without a handler
    script = fake_runner(tmp_path)
    # the real thing: timeout (it catches SIGUSR1 to forward it) -> the runner (faulthandler)
    timed = subprocess.Popen(["timeout", "60", sys.executable, str(script), str(tmp_path / "s.txt")],
                             stdout=subprocess.PIPE)
    without = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)", "run_transport.py"])
    try:
        timed.stdout.readline()
        runner = sw.descendants(timed.pid)
        assert len(runner) == 1
        assert sw.stack_targets([timed.pid, *runner, without.pid], "run_transport.py") == runner
    finally:
        for p in (timed, without):
            p.kill()
            p.wait()
        timed.stdout.close()


def test_a_marker_that_cannot_be_written_does_not_keep_the_session(tmp_path):
    script = fake_runner(tmp_path)
    sup = subprocess.Popen(["bash", "-c", f"{sys.executable} {script} {tmp_path / 's.txt'}; true"])
    time.sleep(1.0)
    result = sw.watch([str(tmp_path / "never")], 0.3, sup.pid, stack_wait_s=0.1, grace_s=2.0, period_s=0.1,
                      marker=str(tmp_path / "missing_dir" / "stall.jsonl"))
    sup.wait(timeout=10)
    assert result == "stalled"


def test_a_child_left_behind_by_the_supervisor_is_killed(tmp_path):
    # Codex: the supervisor ends on TERM, its child ignores TERM and is reparented -- it must not outlive the watch
    pid_file = tmp_path / "child.pid"
    child_py = tmp_path / "run_transport.py"  # a runner, without the SIGUSR1 handler
    child_py.write_text("import os,signal,sys,time\nsignal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
                        "open(sys.argv[1], 'w').write(str(os.getpid()))\ntime.sleep(60)\n")
    sup = subprocess.Popen(["bash", "-c", f"{sys.executable} {child_py} {pid_file} & trap 'exit 0' TERM; wait"])
    for _ in range(50):
        if pid_file.exists() and pid_file.read_text():
            break
        time.sleep(0.1)
    child = int(pid_file.read_text())
    result = sw.watch([str(tmp_path / "never")], 0.3, sup.pid, stack_wait_s=0.1, grace_s=1.0, period_s=0.1)
    sup.wait(timeout=10)
    time.sleep(0.5)
    assert result == "stalled" and not sw.alive(child)


def test_nothing_is_judged_while_no_runner_runs(tmp_path):
    # the supervisor alone (between epochs): long silences are not a stall
    sup = subprocess.Popen(["bash", "-c", "sleep 1.5"])
    result = sw.watch([str(tmp_path / "never")], 0.3, sup.pid, period_s=0.1)
    sup.wait(timeout=10)
    assert result == "ended"


def test_silence_counts_from_the_runner_s_start(tmp_path):
    # a new epoch's runner: the old epoch's last record does not count against it
    old = tmp_path / "epoch_1.jsonl"
    old.write_text("x")
    os.utime(old, (time.time() - 3600, time.time() - 3600))
    script = fake_runner(tmp_path)
    sup = subprocess.Popen(["bash", "-c", f"{sys.executable} {script} {tmp_path / 's.txt'}; true"])
    time.sleep(1.0)
    t0 = time.time()
    result = sw.watch([str(tmp_path / "epoch_*.jsonl")], 2.0, sup.pid, stack_wait_s=0.1, grace_s=2.0, period_s=0.1)
    sup.wait(timeout=10)
    assert result == "stalled" and time.time() - t0 >= 0.9  # not at once on the hour-old record


def test_a_supervisor_stuck_without_a_runner_is_a_stall_after_its_own_limit(tmp_path):
    # Codex: a bridge stop that never returns leaves no runner -- it must not hold the job to its time limit
    sup = subprocess.Popen(["bash", "-c", "sleep 30"])
    result = sw.watch([str(tmp_path / "never")], 100.0, sup.pid, stack_wait_s=0.1, grace_s=2.0, period_s=0.1,
                      no_runner_s=0.5, marker=str(tmp_path / "stall.jsonl"))
    sup.wait(timeout=10)
    assert result == "stalled" and json.loads((tmp_path / "stall.jsonl").read_text())["runner"] is False
