"""B-P2d's session wrapper (deploy/console/reloc_session.sh): the console launcher and the probe's hold; a probe that
fails ends the launcher at once (nothing else would end a live session), and a launcher that outlives a normal end is
ended after a grace (Codex B-P2d tooling review P2)."""

import os
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / "deploy/console/reloc_session.sh"


def _fakes(tmp_path, probe_rc: int, launcher_s: float):
    # The fake launcher marks itself ready only after its TERM trap is in place, and the fake probe waits for that mark
    # before it ends: a probe failing in milliseconds could otherwise deliver the wrapper's TERM to a bash that had not
    # yet run its trap line, which ends it without the record the tests look for (seen on ws1 under xdist load).
    launcher = tmp_path / "launcher.sh"
    launcher.write_text("#!/bin/bash\ntrap 'echo terminated >> \"$1/launcher_term\"; exit 143' TERM\n"
                        "touch \"$1/launcher_ready\"\n"
                        f"echo \"$RELOC_UNATTENDED $MAX_EPOCHS $START_POSE\" > \"$1/launcher_env\"\n"
                        f"for i in $(seq 1 {int(launcher_s * 10)}); do sleep 0.1; done\nexit 0\n")
    python = tmp_path / "python"
    python.write_text(f"#!/bin/bash\ncase \"$1\" in *f1_block_judge.py) echo \"$@\" > {tmp_path}/judge_args; "
                      f"exit ${{JUDGE_RC:-0}};; esac\necho \"$@\" > {tmp_path}/probe_args\n"
                      f"for i in $(seq 1 400); do [ -e {tmp_path}/out/launcher_ready ] && break; sleep 0.05; done\n"
                      f"exit {probe_rc}\n")
    python.chmod(0o755)
    return launcher, python


def _run(tmp_path, probe_rc, launcher_s, grace="1"):
    launcher, python = _fakes(tmp_path, probe_rc, launcher_s)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "RELOC_END_GRACE_S": grace, "FACTOR": "0.3"}
    t0 = time.monotonic()
    p = subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=60)
    return p, time.monotonic() - t0, tmp_path / "out"


def test_a_failing_probe_ends_the_launcher_at_once(tmp_path):
    p, took, out = _run(tmp_path, probe_rc=1, launcher_s=60)
    assert took < 15 and (out / "launcher_term").exists()
    assert p.returncode != 0 and "probe failed" in p.stdout


def test_a_normal_end_waits_for_the_launcher_and_passes_its_code(tmp_path):
    p, took, out = _run(tmp_path, probe_rc=0, launcher_s=1, grace="10")  # well past the launcher's 1 s
    assert p.returncode == 0 and not (out / "launcher_term").exists()
    assert (out / "launcher_env").read_text().split() == ["1", "1", "1,2,3"]  # unattended, one epoch, the start
    args = (tmp_path / "probe_args").read_text().split()
    assert args[args.index("--scenario") + 1] == "hold" and args[args.index("--live-factor") + 1] == "0.3"


def test_a_dry_run_is_the_launchers_with_the_sessions_environment_and_no_probe(tmp_path):
    # B-P2d development: the bundle refuses a list without isaac_bundle.py check's preflight proof, and check runs only
    # a launcher with a DRY_RUN mode printing RUNNER_ARGS -- the wrapper hands its DRY_RUN to the console launcher's
    launcher = tmp_path / "launcher.sh"
    launcher.write_text("#!/bin/bash\necho \"$DRY_RUN $RELOC_UNATTENDED $MAX_EPOCHS $START_POSE $1\"\n"
                        "echo RUNNER_ARGS\necho --start-pose=1,2,3\necho ARGS_OK\nexit 0\n")
    python = tmp_path / "python"
    python.write_text(f"#!/bin/bash\ntouch {tmp_path}/probe_ran\nexit 0\n")
    python.chmod(0o755)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "DRY_RUN": "1"}
    p = subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=60)
    assert p.returncode == 0 and not (tmp_path / "probe_ran").exists()
    assert p.stdout.splitlines()[0].split() == ["1", "1", "1", "1,2,3", str(tmp_path / "out")]
    assert "RUNNER_ARGS" in p.stdout.splitlines() and "ARGS_OK" in p.stdout.splitlines()
    assert "RUNNER_ARGS" in WRAPPER.read_text()  # what isaac_bundle.py's _dry_run_launcher looks for


def test_a_launcher_that_outlives_a_normal_end_is_ended_after_the_grace(tmp_path):
    p, took, out = _run(tmp_path, probe_rc=0, launcher_s=60, grace="1")
    assert took < 15 and (out / "launcher_term").exists()


def test_a_place_session_is_attended_and_its_probe_places_around_the_truth(tmp_path):
    # F1 (plan 2026-10-31-operator-placed-start 4.2 b): the console waits for the operator, the probe is the operator
    launcher, python = _fakes(tmp_path, probe_rc=0, launcher_s=1)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "RELOC_END_GRACE_S": "10", "FACTOR": "0.3", "RELOC_PROBE": "place", "CLICK_SEED": "11", "CLICK_INDEX": "4"}
    env["START_POSE"] = "-1,2,3"  # a negative x is one word (Codex F1 implementation review P2 4)
    p = subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=60)
    assert p.returncode == 0, p.stdout + p.stderr
    assert (tmp_path / "out" / "launcher_env").read_text().split() == ["1", "-1,2,3"]  # no RELOC_UNATTENDED
    args = (tmp_path / "probe_args").read_text().split()
    assert args[args.index("--scenario") + 1] == "place" and "--truth=-1,2,3" in args
    assert args[args.index("--click-seed") + 1] == "11" and args[args.index("--click-index") + 1] == "4"
    assert "--click-kind" not in args  # the click model, unless a record click is named


def test_a_place_session_can_name_a_record_click(tmp_path):
    # the lead's safety-net check (2026-10-31): does the bridge's start check stop a wrong placement an operator confirms
    launcher, python = _fakes(tmp_path, probe_rc=0, launcher_s=1)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "RELOC_END_GRACE_S": "10", "FACTOR": "0.3", "RELOC_PROBE": "place", "CLICK_KIND": "bay+y"}
    p = subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=60)
    assert p.returncode == 0, p.stdout + p.stderr
    args = (tmp_path / "probe_args").read_text().split()
    assert args[args.index("--click-kind") + 1] == "bay+y"


def test_an_unknown_probe_mode_is_refused(tmp_path):
    launcher, python = _fakes(tmp_path, probe_rc=0, launcher_s=1)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "RELOC_PROBE": "both"}
    p = subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=60)
    assert p.returncode == 2 and "RELOC_PROBE" in p.stdout + p.stderr


def _session(tmp_path, **extra):
    launcher, python = _fakes(tmp_path, probe_rc=0, launcher_s=1)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "RELOC_END_GRACE_S": "10", "FACTOR": "0.3", **extra}
    return subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                          text=True, timeout=60)


def test_a_reverse_session_is_unattended_and_presses_by_its_rule(tmp_path):
    # plan 2026-11-06-unconfirmed-validation 2.1
    p = _session(tmp_path, RELOC_PROBE="reverse", POSE_CHECK="shadow", REVERSE_KIND="chatter",
                 MISSION_AFTER_CONFIRM="1")
    assert p.returncode == 0, p.stdout + p.stderr
    assert (tmp_path / "out" / "launcher_env").read_text().split() == ["1", "1", "1,2,3"]  # RELOC_UNATTENDED=1
    args = (tmp_path / "probe_args").read_text().split()
    assert args[args.index("--scenario") + 1] == "reverse" and args[args.index("--reverse-kind") + 1] == "chatter"
    assert "--mission-after-confirm" in args and args[args.index("--post-mission-sim-s") + 1] == "20"


def test_a_reverse_place_session_places_then_reverses(tmp_path):
    p = _session(tmp_path, RELOC_PROBE="reverse_place", POSE_CHECK="on", CLICK_SEED="5")
    assert p.returncode == 0, p.stdout + p.stderr
    assert (tmp_path / "out" / "launcher_env").read_text().split() == ["1", "1,2,3"]  # attended
    args = (tmp_path / "probe_args").read_text().split()
    assert args[args.index("--scenario") + 1] == "place" and args[args.index("--after-place") + 1] == "reverse"
    assert args[args.index("--reverse-kind") + 1] == "full" and "--mission-after-confirm" not in args
    assert "--click-kind" not in args


def test_a_reverse_place_session_takes_a_record_click(tmp_path):
    # the F1 path's reach (the lead's decision of 2026-11-07, A): the wrong click, then the strokes
    p = _session(tmp_path, RELOC_PROBE="reverse_place", POSE_CHECK="on", CLICK_KIND="yaw180")
    assert p.returncode == 0, p.stdout + p.stderr
    args = (tmp_path / "probe_args").read_text().split()
    assert args[args.index("--click-kind") + 1] == "yaw180" and args[args.index("--after-place") + 1] == "reverse"


def test_a_reverse_session_needs_the_check_and_a_known_rule(tmp_path):
    for extra in ({"RELOC_PROBE": "reverse"}, {"RELOC_PROBE": "reverse", "POSE_CHECK": "off"},
                  {"RELOC_PROBE": "reverse", "POSE_CHECK": "on", "REVERSE_KIND": "long"}):
        p = _session(tmp_path, **extra)
        assert p.returncode == 2, (extra, p.stdout + p.stderr)


def test_a_place_block_session_places_then_checks_the_block(tmp_path):
    # docs/plans/2026-11-08-f1-vehicle-block.md 5: attended, the placement, then the block check with the vehicle's
    # setting as the expectation and the first CYCLE leg as the mission tried
    p = _session(tmp_path, RELOC_PROBE="place_block", CLICK_SEED="5")
    assert p.returncode == 0, p.stdout + p.stderr
    assert (tmp_path / "out" / "launcher_env").read_text().split() == ["1", "1,2,3"]  # attended
    args = (tmp_path / "probe_args").read_text().split()
    assert args[args.index("--after-place") + 1] == "block_check" and args[args.index("--block-expect") + 1] == "block"
    assert args[args.index("--cycle") + 1] == "P-1:D-1"
    (tmp_path / "b").mkdir()
    p = _session(tmp_path / "b", RELOC_PROBE="place_block", F1_AUTONOMY="allow", CYCLE="P-2:D-1")
    assert p.returncode == 0, p.stdout + p.stderr
    args = (tmp_path / "b" / "probe_args").read_text().split()
    assert args[args.index("--block-expect") + 1] == "allow" and args[args.index("--cycle") + 1] == "P-2:D-1"


def test_a_place_block_session_ends_with_the_vehicles_verdict(tmp_path):
    p = _session(tmp_path, RELOC_PROBE="place_block")
    assert p.returncode == 0, p.stdout + p.stderr
    judge = (tmp_path / "judge_args").read_text().split()
    assert judge[judge.index("--expect") + 1] == "block" and judge[1] == str(tmp_path / "out")
    (tmp_path / "f").mkdir()
    q = _session(tmp_path / "f", RELOC_PROBE="place_block", JUDGE_RC="1")
    assert q.returncode == 6 and "block check failed" in q.stdout
    (tmp_path / "h").mkdir()
    assert _session(tmp_path / "h", RELOC_PROBE="place")  # no judge for the other modes
    assert not (tmp_path / "h" / "judge_args").exists()
