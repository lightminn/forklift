"""The runner's contract for a live far recognition (docs/plans/2026-10-22-live-far-recognition.md task 1): a live
run takes --recognition far (the learned proposals too); the combinations refused before stay refused."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))


def _args(*extra):
    import run_transport

    argv = sys.argv
    sys.argv = ["run_transport.py", "--base-scene", "scene.usda", "--output", "out", "--seed", "0",
                "--forklift-urdf", str(ROOT / "sim/models/dls08_measured/forklift.urdf"),
                "--pallet-urdf", str(ROOT / "sim/models/epal6_pallet/pallet.urdf"),
                "--pallet-geometry", str(ROOT / "config/pallet_geometry_epal6.yaml"),
                "--settings", str(ROOT / "config/isaac_transport_measured.yaml"),
                "--use-perception", "--planning-target", "perception", "--record-slam",
                "--pallet-prior", str(ROOT / "config/pallet_prior_epal6.yaml"),
                "--slam-feedback", "/tmp/slam.sock", "--recognition", "far", *extra]
    try:
        return run_transport.arguments()
    finally:
        sys.argv = argv


def test_a_live_run_takes_the_far_recognition():
    args = _args("--live")
    assert args.live and args.recognition == "far"


LAYER = ("--obstacle-layer", str(ROOT / "config/obstacle_layer_single.yaml"), "--obstacle-act")


@pytest.mark.parametrize("extra, message", [
    (("--live", "--pocket-check", *LAYER), "--recognition far is not defined with --pocket-check"),
    (("--pocket-check", *LAYER), "--recognition far is not defined with --pocket-check"),
    (("--live", "--bottom-deck-check", "on"), "--bottom-deck-check on is a batch run"),
])
def test_the_refused_combinations_stay_refused_live(extra, message, capsys):
    with pytest.raises(SystemExit):
        _args(*extra)
    assert message in capsys.readouterr().err


def test_near_tracking_with_the_far_recognition_is_left_to_the_drive_terms_condition():
    # Near tracking re-measure (docs/plans/2026-10-27-insertion-lateral-drift.md 5.3): the parser takes the pair; the
    # drive terms' measured condition decides at run time (tests/unit/test_isaac_near_tracking.py).
    args = _args("--near-tracking", "--near-bounds", str(ROOT / "config/near_field_bounds_measured.json"),
                 "--near-drive-terms", str(ROOT / "config/near_field_drive_terms.json"),
                 "--approach-straight-speed-mps", "0.08")
    assert args.near_tracking and args.recognition == "far"


def test_the_live_far_failures_are_the_missions_and_the_console_names_them():
    from forklift_core.mission.failures import MISSION, classify

    assert classify("recognition_error:ValueError") == MISSION and classify("recognition_timeout") == MISSION
    js = (ROOT / "console/web/js/results.js").read_text()
    assert '"recognition_error:"' in js and '"recognition_timeout"' in js


def test_the_missions_far_summary_is_a_mission_key_and_the_session_labels_are_not():
    from forklift_core.mission.baseline import CORE_FIELDS, MISSION_KEYS

    assert {"recognition_mission", "recognition_discarded"} <= set(MISSION_KEYS["state"])
    assert "recognition" not in MISSION_KEYS["state"]  # the session's labels (setup) stay
    assert {"observe_failed_needed", "far_leg_recaptures"} <= set(CORE_FIELDS)


def test_the_rejection_tests_zone_offset_is_a_live_far_option(capsys):
    assert _args("--live", "--live-test-far-zone-offset-m", "3.0").live_test_far_zone_offset_m == 3.0
    with pytest.raises(SystemExit):
        _args("--live-test-far-zone-offset-m", "3.0")
    assert "--live-test-far-zone-offset-m needs --live and --recognition far" in capsys.readouterr().err
