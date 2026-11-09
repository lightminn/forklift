"""The live launcher's START_POSE_KNOWN (docs/plans/2026-11-09-l4c-repro-cancel-other-target.md 1.3, the repro batch's
H0): the truck placed at a pose and its start known there -- the runner's --start-pose/--start-known, the bridge's
start pose the same, no relocalization and no F1 block; refused with START_POSE, START_RELOC=on or no survey map."""
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_isaac_live_preflight import (  # noqa: E402,F401 -- tmp_path: the short /tmp root the launcher needs
    _bridge, _fake_python, _launch, _preflight, _registry_launch, _relocalizing, _supervisor, tmp_path)

POSE = "-0.934,0.9106,0.1706"


def _env(tmp_path, name: str) -> str:
    """A value of the session's env.sh as the epoch scripts read it (printf %q escapes the commas)."""
    return subprocess.run(["bash", "-c", f'. "$0"; printf %s "${name}"', str(tmp_path / "out" / "env.sh")],
                          capture_output=True, text=True, check=True).stdout


def test_a_known_start_places_the_truck_and_starts_the_slam_there(tmp_path):
    p, env_sh = _registry_launch(tmp_path, BUNDLE_TOKEN="tok", START_POSE_KNOWN=POSE)
    assert p.returncode == 0, p.stdout + p.stderr
    pre = _preflight(tmp_path)
    i = pre.index(f"--start-pose={POSE}")  # one word: a negative x is no option
    assert pre[i + 1] == "--start-known"
    assert not {"--start-relocalize", "--reloc-dir", "--reloc-unattended", "--operator-placed-auto"} & set(pre)
    assert "start: known at START_POSE_KNOWN (diagnostic: placed there, no relocalization)" in p.stdout
    assert _env(tmp_path, "MAP_START_POSE") == POSE and _env(tmp_path, "START_POSE_KNOWN") == POSE
    assert _env(tmp_path, "START_POSE") == "" and _env(tmp_path, "START_RELOC") == "off"
    supervisor = _supervisor(tmp_path)
    assert "--relocalize" not in supervisor.split() and "map_start_pose" not in supervisor
    (tmp_path / "started").unlink()
    assert _bridge(tmp_path).returncode == 0  # the bridge needs no estimate from the runner
    assert f"SLAM_MAP_START_POSE={POSE}" in (tmp_path / "started").read_text()  # not the registry's start


def test_an_operators_session_with_a_known_start_does_not_relocalize(tmp_path):
    p, env_sh = _registry_launch(tmp_path, START_POSE_KNOWN=POSE)  # no bundle token: the default would relocalize
    assert p.returncode == 0, p.stdout + p.stderr
    pre = _preflight(tmp_path)
    assert "--start-known" in pre and "--start-relocalize" not in pre
    assert "--relocalize" not in _supervisor(tmp_path).split()


def test_the_known_start_refuses_what_it_cannot_do(tmp_path):
    p, _ = _registry_launch(tmp_path / "a", START_POSE_KNOWN=POSE, START_POSE="-4.0,3.0,-2.0")
    assert p.returncode == 2 and "two starts" in p.stdout + p.stderr
    for bad in ("on", ""):
        q, _ = _registry_launch(tmp_path / f"b{len(bad)}", START_POSE_KNOWN=POSE, START_RELOC=bad)
        assert q.returncode == 2 and "contradicts START_POSE_KNOWN" in q.stdout + q.stderr
    q, _ = _registry_launch(tmp_path / "c", START_POSE_KNOWN=POSE, START_RELOC="off")
    assert q.returncode == 0, q.stdout + q.stderr
    (tmp_path / "d").mkdir()
    r = _launch(tmp_path / "d", START_POSE_KNOWN=POSE)  # MAP_BUNDLE=none: online mapping
    assert r.returncode == 2 and "needs one" in r.stderr
    s = _relocalizing(tmp_path / "e", START_POSE_KNOWN=POSE, POSE_CHECK="shadow")  # the check follows a relocalization
    assert s.returncode == 2 and "needs START_POSE" in s.stdout + s.stderr


def test_without_it_the_session_starts_as_before(tmp_path):
    p, env_sh = _registry_launch(tmp_path, BUNDLE_TOKEN="tok")
    assert p.returncode == 0, p.stdout + p.stderr
    assert "--start-known" not in _preflight(tmp_path) and _env(tmp_path, "START_POSE_KNOWN") == ""
    assert _env(tmp_path, "MAP_START_POSE") == "1.5,-2.0,0.25"  # the registry row's start


def test_a_map_folder_given_by_hand_takes_the_known_start_as_its_start_pose(tmp_path):
    # Codex fix A review P3: the folder's MAP_START_POSE requirement comes before the start choice
    bundle = tmp_path / "bundle"
    bundle.mkdir(parents=True)
    p = _launch(tmp_path, venv_py=_fake_python(tmp_path), DRY_RUN="0", MAP_BUNDLE=str(bundle),
                START_POSE_KNOWN=POSE)
    assert p.returncode == 0, p.stdout + p.stderr
    assert _env(tmp_path, "MAP_START_POSE") == POSE and "--start-known" in _preflight(tmp_path)
