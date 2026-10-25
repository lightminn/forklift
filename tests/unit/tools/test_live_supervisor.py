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


def test_an_epoch_without_a_reason_does_not_show_the_previous_one(tmp_path):
    """Codex S3 review P3: epoch 2's crash showed epoch 1's localization reason."""
    result = run(tmp_path, writes_exit([1, 1], ["slam_not_localized: x", None], process_code=0),
                 exit_file="{epoch_dir}/run/exit_code", max_epochs=2)
    assert result["codes"] == [1, 1]
    assert "reason" not in json.loads((tmp_path / "live_port.json").read_text())


# --- the standby (contract section 6: a session that fails its start checks before its server opens) ---------------
def _free_port() -> int:
    import socket

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _get(port: int, path: str):
    import urllib.error
    import urllib.request

    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=2) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()
    except OSError:
        return None, None


def _watch(port: int, until, then_term: bool = True, timeout_s: float = 20.0):
    """In a thread: poll /api/v1/session until ``until(state)``, keep that state, then TERM this process (the
    supervisor runs on the main thread, where its signal handlers are)."""
    import os
    import signal
    import threading

    seen = {}

    def poll():
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            code, body = _get(port, "/api/v1/session")
            if code == 200:
                state = json.loads(body)
                if until(state):
                    seen["state"] = state
                    seen["page"] = _get(port, "/")
                    seen["control"] = _get(port, "/api/v1/control")
                    break
            time.sleep(0.05)
        if then_term:
            os.kill(os.getpid(), signal.SIGTERM)

    thread = threading.Thread(target=poll, daemon=True)
    thread.start()
    return seen, thread


def _web(tmp_path):
    web = tmp_path / "web"
    (web / "js").mkdir(parents=True)
    (web / "index.html").write_text("<html>console</html>")
    (web / "js" / "main.js").write_text("// main")
    return web


import time  # noqa: E402 -- used by the standby helpers above


def test_the_standby_shows_a_start_failure_and_hands_the_port_to_the_runner(tmp_path):
    port, web = _free_port(), _web(tmp_path)
    bound = tmp_path / "bound.txt"
    # a runner that binds the console's port (the standby must have let it go), then fails its start checks
    # before writing its port file running
    runner = [PY, "-c", "import json,socket,sys,pathlib; s=socket.socket(); "
              "s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); s.bind(('127.0.0.1', int(sys.argv[1]))); "
              "pathlib.Path(sys.argv[2]).write_text('bound'); s.close(); "
              "pathlib.Path(sys.argv[3]).parent.mkdir(parents=True, exist_ok=True); "
              "pathlib.Path(sys.argv[3]).write_text(json.dumps({'code': 1, 'reason': 'slam_not_localized: 0.4 m'}))",
              str(port), str(bound), "{epoch_dir}/run/exit_code"]
    seen, thread = _watch(port, lambda s: s["phase"] == "stopped")
    result = sup.supervise(out=tmp_path / "out", runner=runner, port_file=tmp_path / "live_port.json", backoff_s=0.0,
                           max_epochs=1, exit_file="{epoch_dir}/run/exit_code", standby_port=port, web_root=web,
                           standby_linger_s=30.0)
    thread.join(5.0)
    assert bound.read_text() == "bound"  # the port was the runner's while it ran
    state = seen["state"]
    assert state["end"] == "epoch_limit" and state["retry"] is False and state["next_epoch"] is None
    assert state["last_exit"] == {"epoch": 1, "exit_code": 1, "reason": "slam_not_localized: 0.4 m",
                                  "stage": "start"}
    assert seen["page"] == (200, b"<html>console</html>") and seen["control"][0] == 503
    assert result["end"] == "epoch_limit"
    assert _get(port, "/api/v1/session") == (None, None)  # closed with the supervisor


def test_a_run_failure_is_told_apart_and_the_backoff_names_the_next_epoch(tmp_path):
    port, web = _free_port(), _web(tmp_path)
    # epoch 1 opens its server (port file running, started now) then fails; epoch 2 ends the session
    runner = [PY, "-c", "import json,os,sys,time,pathlib; e=int(sys.argv[1]); "
              "pathlib.Path(sys.argv[2]).write_text(json.dumps({'state':'running','started':time.time(),"
              "'pid':os.getpid()})); "
              "sys.exit(1 if e == 1 else 0)", "{epoch}", "{port_file}"]
    seen, thread = _watch(port, lambda s: s["phase"] == "backoff", then_term=False)
    result = sup.supervise(out=tmp_path / "out", runner=runner, port_file=tmp_path / "live_port.json", backoff_s=2.0,
                           standby_port=port, web_root=web)
    thread.join(5.0)
    state = seen["state"]
    assert state["retry"] is True and state["next_epoch"] == 2 and state["last_exit"]["stage"] == "run"
    assert result["end"] == "ended" and result["codes"] == [1, 0]


def test_a_refused_map_bundle_is_shown_until_a_term_ends_the_linger(tmp_path):
    port, web = _free_port(), _web(tmp_path)
    (tmp_path / "bundle").mkdir()  # no manifest: refused
    seen, thread = _watch(port, lambda s: s["phase"] == "stopped")
    began = time.monotonic()
    result = sup.supervise(out=tmp_path / "out", runner=[PY, "-c", "pass"], port_file=tmp_path / "live_port.json",
                           map_bundle=tmp_path / "bundle", standby_port=port, web_root=web, standby_linger_s=60.0)
    thread.join(5.0)
    assert time.monotonic() - began < 20.0  # the TERM ended the linger
    state = seen["state"]
    assert state["end"] == "map_mismatch" and state["last_exit"]["epoch"] == 0
    assert state["last_exit"]["stage"] == "start" and state["last_exit"]["reason"]
    assert result["end"] == "map_mismatch"


def test_the_live_job_has_the_supervisor_serve_the_consoles_port_between_runners():
    """Contract section 6: the standby on the console's own port, with the page; the linger can be turned off."""
    from pathlib import Path

    live = (Path(__file__).resolve().parents[3] / "deploy" / "console" / "isaac_live.sbatch").read_text()
    assert ('--standby-port "${PORT:-8798}" --web-root "$ROOT/console/web" '
            '--standby-linger-s "${STANDBY_LINGER_S:-300}"') in live


def test_a_run_ended_through_the_runtimes_close_is_a_run_and_one_without_its_server_a_start(tmp_path):
    """Codex standby review P2-2: LiveRuntime.close rewrote the port file without its start time, so every failure
    read as a start failure. The close keeps it (when the server opened); the pid ties it to this runner."""
    import os

    port = _free_port()
    web = _web(tmp_path)
    # epochs 1 and 3: opened (the close's ended file keeps started and the runner's pid); epoch 2: closed
    # without opening (no started) and failed; epoch 3 ends the session
    runner = [PY, "-c", "import json,os,sys,time,pathlib; e=int(sys.argv[1]); p=pathlib.Path(sys.argv[2]); "
              "info={'state':'ended','pid':os.getpid()}; "
              "info.update({'started':time.time()} if e != 2 else {}); p.write_text(json.dumps(info)); "
              "sys.exit(0 if e == 3 else 1)", "{epoch}", "{port_file}"]
    stages = []
    orig = sup.Standby.update

    def spy(self, **fields):
        if fields.get("last_exit"):
            stages.append(fields["last_exit"]["stage"])
        return orig(self, **fields)

    sup.Standby.update = spy
    try:
        result = sup.supervise(out=tmp_path / "out", runner=runner, port_file=tmp_path / "live_port.json",
                               backoff_s=0.0, standby_port=port, web_root=web)
    finally:
        sup.Standby.update = orig
    assert result["end"] == "ended" and stages == ["run", "start", "run"]
    assert os.getpid()  # (the runner's pid is its own, not this process's)


def test_an_epoch_limit_after_a_scene_reset_lingers_too(tmp_path):
    """Codex standby review P2-3: whether the standby lingers follows the end, not the last runner's code."""
    port, web = _free_port(), _web(tmp_path)
    seen, thread = _watch(port, lambda s: s["phase"] == "stopped")
    result = sup.supervise(out=tmp_path / "out", runner=runner_exiting(3), port_file=tmp_path / "live_port.json",
                           backoff_s=0.0, max_epochs=1, standby_port=port, web_root=web, standby_linger_s=30.0)
    thread.join(5.0)
    assert result["end"] == "epoch_limit" and seen.get("state", {}).get("end") == "epoch_limit"


def test_the_live_runner_takes_the_encoder_model_with_its_own_age_table():
    """Plan 2026-10-24-insertion-depth-margin 5: ENCODER_MODEL picks the SLAM odometry's encoders, and a team-spec
    model the team-spec obstacle-layer age table -- the runner refuses the reported table for it. The default is the
    joint-angle model (plan 2026-10-25-team-spec-default), reported on request, nothing else."""
    from pathlib import Path

    live = (Path(__file__).resolve().parents[3] / "deploy" / "console" / "isaac_live.sbatch").read_text()
    assert "ENCODER_MODEL=${ENCODER_MODEL:-team_spec_quantized}" in live
    assert 'case "$ENCODER_MODEL" in reported|team_spec_quantized) ;;' in live
    assert "printf 'ENCODER_MODEL=%q\\n' \"$ENCODER_MODEL\"" in live
    runner = live[live.index('cat > "$OUT/runner.sh"'):]
    runner = runner[: runner.index("EOT\n")]
    assert ('LAYER=config/obstacle_layer_single.yaml\n'
            '[ "$ENCODER_MODEL" = reported ] || LAYER=config/obstacle_layer_single_team_spec.yaml') in runner
    assert '--obstacle-layer "$LAYER" --encoder-model "$ENCODER_MODEL" --obstacle-act' in runner
    assert "--obstacle-layer config/obstacle_layer_single.yaml" not in runner
