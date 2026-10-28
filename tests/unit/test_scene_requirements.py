"""Scene requirements (docs/plans/2026-10-27-scene-requirements.md): the arguments a scene needs and the argument sets a
purpose compares against, declared in config/scene_requirements.yaml and checked by the runner before Isaac."""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))

import scene_requirements as sr  # noqa: E402
from tools import riibotics_isaac_scenes as tool  # noqa: E402

RUN = ("import runpy, sys; sys.modules['isaacsim'] = None; sys.argv = sys.argv[1:]; "
       "runpy.run_path(sys.argv[0], run_name='__main__')")


def _base(out):
    return ["--base-scene", "scene.usda", "--output", str(out), "--seed", "0",
            "--forklift-urdf", str(ROOT / "sim/models/dls08_measured/forklift.urdf"),
            "--pallet-urdf", str(ROOT / "sim/models/epal6_pallet/pallet.urdf"),
            "--pallet-geometry", str(ROOT / "config/pallet_geometry_epal6.yaml"),
            "--settings", str(ROOT / "config/isaac_transport_measured.yaml"),
            "--use-perception", "--planning-target", "perception", "--record-slam",
            "--pallet-prior", str(ROOT / "config/pallet_prior_epal6.yaml"),
            "--obstacle-layer", str(ROOT / "config/obstacle_layer_single.yaml"), "--obstacle-act"]


def _check(*argv):
    env = {**os.environ, "PYTHONPATH": str(ROOT / "src")}
    return subprocess.run([sys.executable, "-c", RUN, str(ROOT / "sim/isaac/run_transport.py"), *argv, "--check-args"],
                          capture_output=True, text=True, timeout=120, env=env, cwd=ROOT)


def _scene(tmp_path, name, built):
    path = tmp_path / f"{name}.json"
    path.write_text(json.dumps(tool.scene_record(name, built)))
    return path


def _parser_dests():
    """Every argparse dest of the runner with its default (the runner parses sys.argv; give it the minimum)."""
    import run_transport

    argv, sys.argv = sys.argv, ["run_transport.py", *_base(Path("/nonexistent/out"))]
    try:
        return vars(run_transport.arguments())
    finally:
        sys.argv = argv


# --- the file ----------------------------------------------------------------------------------------------------


def test_the_file_names_only_runner_arguments_and_values_their_parser_takes():
    data = sr.load(sr.DEFAULT_PATH)
    dests = _parser_dests()
    keys = {k for e in data["scenes"].values() for option in e["any_of"] for k in option}
    keys |= {k for e in data["profiles"].values() for k in e["args"]}
    assert keys <= set(dests), keys - set(dests)
    # every declared value is one the runner can hold: run the profile's own set through the check
    for name, entry in data["profiles"].items():
        values = {**dests, **entry["args"]}
        assert sr.check(values, data, profile=name)["mismatches"] == []


@pytest.mark.parametrize("text, word", [
    ("format: other\n", "not a"),
    ("format: scene_requirements_v1\nscenes: {t: {any_of: [{a: 1}]}}\n", "why"),
    ("format: scene_requirements_v1\nscenes: {t: {any_of: [], why: x}}\n", "any_of"),
    ("format: scene_requirements_v1\nprofiles: {p: {args: {near_capture_expected: on}, why: x}}\n", "boolean"),
    ("format: scene_requirements_v1\nprofiles: {p: {args: {a: 1}, recommends: {fps: 60}, why: x}}\n", "recommends"),
    ("format: scene_requirements_v1\nscenes: {t: {any_of: [{a: {lt: 1}}], why: x}}\n", "gt"),
    ("format: scene_requirements_v1\nprofiles: {p: {args: {recognition_tau: .nan}, why: x}}\n", "finite"),
    ("format: scene_requirements_v1\nscenes: {t: {any_of: [{a: {gt: .inf}}], why: x}}\n", "finite"),
])
def test_a_malformed_file_is_refused(tmp_path, text, word):
    path = tmp_path / "r.yaml"
    path.write_text(text)
    with pytest.raises(ValueError, match=word):
        sr.load(path)


def test_values_compare_as_declared():
    assert sr.holds(0.30000000001, 0.30) and not sr.holds(0.31, 0.30)
    assert not sr.holds(None, 0.30) and sr.holds(None, None) and not sr.holds(0.0, None)  # null is "not given"
    assert sr.holds(0.15, {"gt": 0.0}) and not sr.holds(0.0, {"gt": 0.0}) and not sr.holds(None, {"gt": 0.0})
    assert sr.holds("always", "always") and not sr.holds("on", "always") and not sr.holds(True, 1)


def test_an_argument_the_runner_lacks_or_a_profile_the_file_lacks_is_an_error():
    data = {"format": sr.FORMAT, "scenes": {"t": {"any_of": [{"no_such_flag": 1}], "why": "x"}}, "profiles": {}}
    with pytest.raises(ValueError, match="no_such_flag"):
        sr.check({"a": 1}, data, {"name": "t"})
    with pytest.raises(ValueError, match="no profile"):
        sr.check({"a": 1}, data, profile="ct_baselin")


# --- scenes ------------------------------------------------------------------------------------------------------


def test_trailer_needs_an_exit_prefix(tmp_path):
    # 82ffffa8: a riib trailer batch without --exit-clearance-m 0.30 went 0/10, rc 0 and nothing refused
    scene = _scene(tmp_path, "trailer", tool.trailer_scene())
    base = [*_base(tmp_path / "run"), "--layout", "scene", "--scene-file", str(scene)]
    p = _check(*base)
    assert p.returncode == 2 and "scene trailer needs one of" in p.stderr and "ARGS_OK" not in p.stdout
    assert _check(*base, "--exit-clearance-m", "0.30").returncode == 0
    assert _check(*base, "--lidar-keepout-m", "0.15").returncode == 0  # the keepout's own prefix
    # both: the runner would refuse it after Isaac starts (0.30 below the keepout's 0.385 m), now before
    assert _check(*base, "--exit-clearance-m", "0.30", "--lidar-keepout-m", "0.15").returncode == 2
    p = _check(*base, "--scene-requirements-override", "exit prefix off on purpose (comparison)")
    assert p.returncode == 0 and "overridden (exit prefix off on purpose (comparison))" in p.stderr
    assert _check(*base, "--scene-requirements-override", "   ").returncode == 2  # a blank reason is none


def test_the_rack_aisle_needs_the_keepout_only_with_posts_at_their_size(tmp_path):
    real = _scene(tmp_path, "aisle_rack", tool.rack_scene())
    assert json.loads(real.read_text())["source"]["upright_keepout_m"] == 0.0
    base = [*_base(tmp_path / "run"), "--layout", "scene", "--scene-file", str(real)]
    assert _check(*base).returncode == 2
    assert _check(*base, "--lidar-keepout-m", "0.15").returncode == 0
    # the PR #106 copy widened the posts in the file itself: same name, not this entry
    wide = tmp_path / "wide"
    wide.mkdir()
    widened = _scene(wide, "aisle_rack", tool.rack_scene(keepout=0.20))
    assert _check(*_base(tmp_path / "run"), "--layout", "scene", "--scene-file", str(widened)).returncode == 0


def test_a_copy_without_the_match_key_is_not_matched_and_said_so():
    data = sr.load(sr.DEFAULT_PATH)
    report = sr.check({"lidar_keepout_m": 0.0}, data, {"name": "aisle_rack", "source": {}})
    assert report["matched"] is False and "absent" in report["not_matched"] and report["mismatches"] == []


def test_a_scene_without_an_entry_needs_nothing(tmp_path):
    scene = _scene(tmp_path, "line_2", tool.line_scene(1))
    assert _check(*_base(tmp_path / "run"), "--layout", "scene", "--scene-file", str(scene)).returncode == 0


# --- profiles ------------------------------------------------------------------------------------------------------


def test_a_profile_applies_only_when_named():
    data = sr.load(sr.DEFAULT_PATH)
    values = {**_parser_dests()}
    assert sr.check(values, data)["mismatches"] == []  # no FAR: fine without a profile (d7oor)
    report = sr.check({**values, **data["profiles"]["ct_baseline"]["args"], "recognition_pixel_lag_steps": 24},
                      data, profile="ct_baseline", final={"live_factor": 0.3, "encoder_model": "team_spec_quantized",
                                                          "slam": "localization"})
    assert [m["arg"] for m in report["mismatches"]] == ["recognition_pixel_lag_steps"]
    assert [w["recommends"] for w in report["warnings"]] == ["live_factor"]  # a warning only


def test_the_runner_refuses_an_unknown_profile(tmp_path):
    p = _check(*_base(tmp_path / "run"), "--requirements-profile", "ct_baselin")
    assert p.returncode == 2 and "no profile" in p.stderr


# --- the record --------------------------------------------------------------------------------------------------


def test_the_options_are_recorded_only_when_given_and_are_run_specific_for_the_near_field_contract():
    import run_transport
    from near_tracking import RUN_SPECIFIC_ARGUMENTS

    keys = ("scene_requirements", "requirements_profile", "scene_requirements_override")
    values = _parser_dests()
    import argparse

    args = argparse.Namespace(**values, scene_requirements_report={"scene": None})
    recorded = run_transport.recorded_arguments(args)
    assert not set(keys) & set(recorded) and "scene_requirements_report" not in recorded
    args.scene_requirements_override = "a comparison"
    assert run_transport.recorded_arguments(args)["scene_requirements_override"] == "a comparison"
    assert set(keys) <= set(RUN_SPECIFIC_ARGUMENTS)


def test_the_preflight_proof_rests_on_the_requirements(tmp_path):
    from tools import isaac_bundle as ib

    assert "config/scene_requirements.yaml" in ib.SNAPSHOT_INPUTS
    assert "sim/isaac/scene_requirements.py" in ib.SNAPSHOT_INPUTS
