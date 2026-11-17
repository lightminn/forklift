"""The F2 tyre contact on by default in the riib and live launchers (user decision 2026-11-18,
docs/validation/2026-11-18-f2-i-reregistration.md): f2 unless told otherwise, an EXTRA_ARGS --tyre-contact (in full)
wins and is recorded, a --near-tracking run defaults it off and refuses f2 (its contract was measured rigid)."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_live_riib_45_defaults as LV  # noqa: E402
import test_riib_scene_launcher as RB  # noqa: E402

tmp_live = LV.tmp_path  # the short /tmp root the live launcher needs


def tyre_record(tmp_path, out="out"):
    return [l for l in (tmp_path / out / "riib_defaults.txt").read_text().splitlines() if l.startswith("TYRE_CONTACT=")]


# --- riib ------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("scene, env", [("line_2", {}), ("aisle", {}), ("trailer", {}), ("line_2", {"MODE": "truth"})])
def test_riib_turns_f2_on_for_every_scene_and_mode(tmp_path, scene, env):
    out = RB.dry_run(tmp_path, scene=scene, **env)
    assert out.returncode == 0, out.stdout + out.stderr
    args = RB.args_of(out)
    assert RB.contains(args, ["--tyre-contact", "f2"]) and args.count("--tyre-contact") == 1
    assert tyre_record(tmp_path) == ["TYRE_CONTACT=f2 TYRE_CONTACT_FROM=launcher"]


@pytest.mark.parametrize("env, extra, value, src", [({"TYRE_CONTACT": "off"}, "", "off", "launcher"),
                                                    ({}, "--tyre-contact off", "off", "EXTRA_ARGS"),
                                                    ({}, "--tyre-contact=off", "off", "EXTRA_ARGS"),
                                                    ({"TYRE_CONTACT": "off"}, "--tyre-contact f2", "f2", "EXTRA_ARGS")])
def test_riib_tyre_can_be_switched_and_extra_args_wins(tmp_path, env, extra, value, src):
    out = RB.dry_run(tmp_path, EXTRA_ARGS=extra, **env) if extra else RB.dry_run(tmp_path, **env)
    assert out.returncode == 0, out.stdout + out.stderr
    args = RB.args_of(out)
    assert sum(a == "--tyre-contact" or a.startswith("--tyre-contact=") for a in args) == 1
    assert tyre_record(tmp_path) == [f"TYRE_CONTACT={value} TYRE_CONTACT_FROM={src}"]


def test_riib_near_tracking_keeps_the_tyre_rigid(tmp_path):
    near = dict(EXTRA_ARGS="--near-tracking", RETURN_CUSP_REPLANS="0", EXIT_EXEMPT="off",
                DOCKING_DEGENERATE_CONTINUE="off", GOAL_CREEP_BAND="off")
    out = RB.dry_run(tmp_path, out=tmp_path / "a", **near)
    assert out.returncode == 0, out.stdout + out.stderr
    assert RB.contains(RB.args_of(out), ["--tyre-contact", "off"])
    out = RB.dry_run(tmp_path, out=tmp_path / "b", TYRE_CONTACT="f2", **near)
    assert out.returncode == 2 and "needs TYRE_CONTACT=off" in out.stdout + out.stderr
    out = RB.dry_run(tmp_path, out=tmp_path / "c", TYRE_CONTACT="soft")
    assert out.returncode == 2 and "TYRE_CONTACT must be f2 or off" in out.stdout + out.stderr


# --- live ------------------------------------------------------------------------------------------------------------

def test_live_turns_f2_on_by_default(tmp_live):
    p = LV._launch(tmp_live)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "TYRE_CONTACT=f2 TYRE_CONTACT_FROM=launcher" in p.stdout
    e = LV._env_sh(tmp_live)
    assert (e["TYRE_CONTACT"], e["TYRE_CONTACT_APPLIED"], e["TYRE_CONTACT_FROM"]) == ("f2", "f2", "launcher")
    assert LV._values(tmp_live, "--tyre-contact") == ["f2"]


def test_live_extra_args_comes_later_and_wins(tmp_live):
    p = LV._launch(tmp_live, EXTRA_ARGS="--tyre-contact off")
    assert p.returncode == 0, p.stdout + p.stderr
    assert "TYRE_CONTACT=off TYRE_CONTACT_FROM=EXTRA_ARGS" in p.stdout
    assert LV._values(tmp_live, "--tyre-contact")[-1] == "off"


def test_live_off_and_abbreviation(tmp_live):
    p = LV._launch(tmp_live, TYRE_CONTACT="off")
    assert p.returncode == 0 and LV._values(tmp_live, "--tyre-contact") == ["off"]
    p = LV._launch(tmp_live, EXTRA_ARGS="--tyre off")
    assert p.returncode == 2 and "write --tyre-contact in full" in p.stdout + p.stderr
