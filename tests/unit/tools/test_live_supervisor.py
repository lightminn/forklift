"""The live session's supervisor (persistent console plan D9): epoch after epoch by the runner's exit code."""

import json
import sys

from tools import live_supervisor as sup

PY = sys.executable


def runner_exiting(*codes):
    """A runner command that exits with codes[epoch-1] and writes its port file running first."""
    table = json.dumps(list(codes))
    return [PY, "-c", "import json,sys,pathlib; e=int(sys.argv[1]); p=pathlib.Path(sys.argv[2]);"
            "p.write_text(json.dumps({'state':'running','epoch':e})); sys.exit(json.loads(sys.argv[3])[e-1])",
            "{epoch}", "{port_file}", table]


def run(tmp_path, runner, **k):
    return sup.supervise(out=tmp_path, runner=runner, port_file=tmp_path / "live_port.json", backoff_s=0.0, **k)


def test_an_ended_session_stops_the_supervisor(tmp_path):
    result = run(tmp_path, runner_exiting(0))
    assert result == {"epochs": 1, "end": "ended", "codes": [0]}
    assert (tmp_path / "epoch_1").is_dir()


def test_a_scene_reset_starts_a_new_epoch_at_once(tmp_path):
    result = run(tmp_path, runner_exiting(3, 3, 0))
    assert result["codes"] == [3, 3, 0] and result["end"] == "ended"
    assert all((tmp_path / f"epoch_{n}").is_dir() for n in (1, 2, 3))


def test_a_crash_marks_the_port_file_and_three_in_a_row_stop_it(tmp_path):
    result = run(tmp_path, runner_exiting(1, 1, 1, 0))
    assert result["codes"] == [1, 1, 1] and result["end"] == "failing"
    assert json.loads((tmp_path / "live_port.json").read_text())["state"] == "crashed"
    log = [json.loads(line) for line in (tmp_path / "supervisor.jsonl").read_text().splitlines()]
    assert [e["code"] for e in log if e["event"] == "exit"] == [1, 1, 1]


def test_a_failure_then_a_reset_does_not_count_against_the_limit(tmp_path):
    result = run(tmp_path, runner_exiting(1, 3, 1, 1, 0))
    assert result["codes"] == [1, 3, 1, 1, 0] and result["end"] == "ended"


def test_the_bridge_starts_before_and_stops_after_each_epoch(tmp_path):
    marks = tmp_path / "marks.txt"
    bridge = [PY, "-c", "import sys,pathlib; pathlib.Path(sys.argv[1]).open('a').write('start {epoch}\\n')",
              str(marks)]
    cleanup = [PY, "-c", "import sys,pathlib; pathlib.Path(sys.argv[1]).open('a').write('stop {epoch}\\n')",
               str(marks)]
    result = run(tmp_path, runner_exiting(3, 0), bridge=bridge, bridge_stop=cleanup)
    assert result["end"] == "ended"
    assert marks.read_text().split("\n")[:4] == ["start 1", "stop 1", "start 2", "stop 2"]


def test_the_epoch_limit_stops_it(tmp_path):
    result = run(tmp_path, runner_exiting(3, 3, 3, 3), max_epochs=2)
    assert result == {"epochs": 2, "end": "epoch_limit", "codes": [3, 3]}


def test_no_runner_starts_without_its_bridge_ready(tmp_path):
    ran = tmp_path / "ran.txt"
    runner = [PY, "-c", "import sys,pathlib; pathlib.Path(sys.argv[1]).write_text('ran'); sys.exit(0)", str(ran)]
    result = sup.supervise(out=tmp_path, runner=runner, port_file=tmp_path / "live_port.json", backoff_s=0.0,
                           bridge=[PY, "-c", "pass"], bridge_ready="{epoch_dir}/bridge/bridge_ready",
                           ready_timeout_s=0.2)
    assert result["end"] == "failing" and result["codes"] == [1, 1, 1] and not ran.exists()


def test_the_runners_own_exit_file_wins_over_the_process_code(tmp_path):
    """Isaac's app.close() ended the process with 0 before the runner's SystemExit (ws1 2026-10-18: a SLAM link
    failure and an obstacle failure both exited 0); the runner writes its code first and that is what counts."""
    runner = [PY, "-c", "import sys,pathlib; e=int(sys.argv[1]); d=pathlib.Path(sys.argv[2]) / 'run'; d.mkdir();"
              "(d / 'exit_code').write_text(str([3, 1, 0][e - 1])); sys.exit(0)", "{epoch}", "{epoch_dir}"]
    result = run(tmp_path, runner, exit_file="{epoch_dir}/run/exit_code")
    assert result["codes"] == [3, 1, 0] and result["end"] == "ended"


def test_a_process_that_ends_without_its_exit_file_is_a_crash(tmp_path):
    result = run(tmp_path, [PY, "-c", "pass"], exit_file="{epoch_dir}/run/exit_code", max_epochs=1)
    assert result["codes"] == [1]
