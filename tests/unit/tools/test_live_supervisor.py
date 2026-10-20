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


# --- Codex stage 4b round 1 ---------------------------------------------------------------------------------------
def writes_exit(code_by_epoch, reasons=None, process_code=0):
    """A runner that writes its exit file {code, reason} and then ends the process with process_code (Isaac: 0)."""
    table = json.dumps([list(code_by_epoch), list(reasons or [None] * len(code_by_epoch)), process_code])
    return [PY, "-c", "import json,sys,pathlib; e=int(sys.argv[1]); d=pathlib.Path(sys.argv[2]) / 'run'; d.mkdir();"
            "c, r, p = json.loads(sys.argv[3]); (d / 'exit_code').write_text(json.dumps({'code': c[e-1],"
            " 'reason': r[e-1]})); sys.exit(p)", "{epoch}", "{epoch_dir}", table]


def test_a_process_that_really_crashed_is_a_failure_whatever_its_file_says(tmp_path):
    result = run(tmp_path, writes_exit([0], process_code=139), exit_file="{epoch_dir}/run/exit_code", max_epochs=1)
    assert result["codes"] == [139] and result["end"] == "epoch_limit"
    assert json.loads((tmp_path / "live_port.json").read_text())["state"] == "crashed"


def test_three_failures_of_one_cause_stop_it_and_a_different_cause_starts_the_count_again(tmp_path):
    reasons = ["slam_link_failed: scan 1", "Tracking failed in observe", "slam_link_failed: scan 9",
               "slam_link_failed: scan 12", "slam_link_failed: scan 30", None]
    result = run(tmp_path, writes_exit([1, 1, 1, 1, 1, 0], reasons), exit_file="{epoch_dir}/run/exit_code")
    assert result["codes"] == [1, 1, 1, 1, 1] and result["end"] == "failing"


def test_a_start_that_raises_is_a_failed_epoch_and_is_retried(tmp_path):
    result = run(tmp_path, [str(tmp_path / "no-such-runner")], max_epochs=2)
    assert result["codes"] == [1, 1] and result["end"] == "epoch_limit"
    log = [json.loads(line) for line in (tmp_path / "supervisor.jsonl").read_text().splitlines()]
    assert any(e["event"] == "start_failed" for e in log)


def test_a_term_ends_the_runner_removes_the_bridge_and_starts_nothing_more(tmp_path):
    import signal
    import subprocess
    import time

    marks = tmp_path / "marks.txt"
    script = tmp_path / "sup.py"
    script.write_text(
        "import sys, json, pathlib\n"
        f"sys.path.insert(0, {str(sup.__file__.rsplit('/', 2)[0])!r})\n"
        "from tools import live_supervisor as sup\n"
        f"out = pathlib.Path({str(tmp_path / 'out')!r})\n"
        "py = sys.executable\n"
        f"marks = {str(marks)!r}\n"
        # a runner that ignores TERM: the supervisor must KILL it after the grace (Codex 4b round 2 P2)
        "r = sup.supervise(out=out, runner=[py, '-c', 'import signal, time; signal.signal(signal.SIGTERM,"
        " signal.SIG_IGN); time.sleep(60)'], port_file=out / 'p.json', backoff_s=0.0, grace_s=0.5,"
        " bridge=[py, '-c', 'pass'],"
        " bridge_stop=[py, '-c', 'import sys; open(sys.argv[1], \"a\").write(\"stop {epoch}\\\\n\")', marks])\n"
        "print(json.dumps(r))\n")
    proc = subprocess.Popen([PY, str(script)], stdout=subprocess.PIPE, text=True)
    end = time.monotonic() + 20
    while not (tmp_path / "out" / "supervisor.jsonl").exists() and time.monotonic() < end:
        time.sleep(0.1)
    time.sleep(0.5)
    proc.send_signal(signal.SIGTERM)
    out, _ = proc.communicate(timeout=20)
    result = json.loads(out.strip().splitlines()[-1])
    assert result["end"] == "terminated" and result["epochs"] == 1
    assert marks.read_text().split() == ["stop", "1"]


def test_a_failing_cleanup_is_logged(tmp_path):
    result = run(tmp_path, runner_exiting(0), bridge=[PY, "-c", "pass"], bridge_stop=[PY, "-c", "raise SystemExit(9)"])
    assert result["end"] == "ended"
    log = [json.loads(line) for line in (tmp_path / "supervisor.jsonl").read_text().splitlines()]
    assert any(e["event"] == "bridge_stop_failed" and e.get("code") == 9 for e in log)


# --- SLAM localization plan L4 -------------------------------------------------------------------------------------
def _bundle(tmp_path):
    import numpy as np

    from forklift_core.localization import slam_map as sm

    folder = tmp_path / "bundle"
    folder.mkdir()
    (folder / "map.posegraph").write_bytes(b"g")
    (folder / "map.data").write_bytes(b"d")
    np.savez(folder / "grid.npz", data=np.zeros((2, 2), np.int8), origin=np.zeros(3), resolution_m=0.05)
    hashes = {n: sm.file_sha256(folder / n) for n in sm.BUNDLE_FILES}
    (folder / "manifest.json").write_text(json.dumps({"format": sm.FORMAT, "map_id": sm.map_id_of(hashes),
                                                      "files": hashes}))
    return folder, sm.map_id_of(hashes)


def test_the_session_s_map_id_fills_every_epoch_s_commands(tmp_path):
    folder, map_id = _bundle(tmp_path)
    seen = tmp_path / "seen.txt"
    runner = [PY, "-c", "import sys,pathlib; pathlib.Path(sys.argv[1]).open('a').write(sys.argv[2] + ' ' + sys.argv[3]"
              " + '\\n'); sys.exit(3 if int(sys.argv[4]) == 1 else 0)", str(seen), "{expected_map_id}", "{map_bundle}",
              "{epoch}"]
    result = run(tmp_path / "out", runner, max_epochs=2, map_bundle=folder)
    assert result["codes"] == [3, 0]
    assert seen.read_text().split("\n")[:2] == [f"{map_id} {folder.resolve()}"] * 2


def test_a_map_mismatch_stops_at_once_and_its_reason_reaches_the_port_file(tmp_path):
    folder, _ = _bundle(tmp_path)
    reason = "slam_map_mismatch: seed 1 is not the survey's 0"
    result = run(tmp_path / "out", writes_exit([1, 0], [reason, None]), exit_file="{epoch_dir}/run/exit_code",
                 map_bundle=folder)
    assert result["epochs"] == 1 and result["end"] == "map_mismatch"
    port = json.loads((tmp_path / "out" / "live_port.json").read_text())
    assert port["state"] == "crashed" and port["reason"] == reason


def test_a_bundle_that_does_not_load_starts_no_epoch(tmp_path):
    folder, _ = _bundle(tmp_path)
    (folder / "map.data").write_bytes(b"changed")
    result = run(tmp_path / "out", runner_exiting(0), map_bundle=folder)
    assert result["epochs"] == 0 and result["end"] == "map_mismatch" and result["reason"].startswith("slam_map_mismatch")
    assert not (tmp_path / "out" / "epoch_1").exists()


def test_a_failed_process_keeps_its_own_code_and_its_written_reason(tmp_path):
    result = run(tmp_path, writes_exit([1], ["slam_not_localized: fractions"], process_code=1),
                 exit_file="{epoch_dir}/run/exit_code", max_epochs=1)
    assert result["codes"] == [1]
    assert json.loads((tmp_path / "live_port.json").read_text())["reason"] == "slam_not_localized: fractions"
