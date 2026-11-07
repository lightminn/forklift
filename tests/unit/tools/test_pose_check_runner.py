"""The unconfirmed position check in the runner (docs/plans/2026-11-02-unconfirmed-runner.md 3): ``--pose-check``
(off by default, not recorded off, in the near-field contract) and its development values, required to turn it on."""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))
sys.path.insert(0, str(ROOT / "tests/unit/localization"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402
from test_start_reloc_runner import _live, _reloc  # noqa: E402

BOUNDS = {"odom": [0.02, 0.02, 0.005, 0.01], "e_still": [0.03, 0.01], "e_slam": [0.05, 0.025],
          "e_window": [0.153, 0.103], "e_release": [0.065, 0.076]}


def test_the_check_is_off_by_default_and_not_recorded_off(tmp_path):
    import run_transport

    off = _runner_args(_live(tmp_path, *_reloc(tmp_path)))
    assert off.pose_check == "off" and off.pose_check_bounds is None
    assert not {"pose_check", "pose_check_bounds"} & set(run_transport.recorded_arguments(off))


def test_on_needs_the_relocalization_and_every_development_value(tmp_path):
    import run_transport

    on = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "on", "--pose-check-bounds",
                            json.dumps(BOUNDS)))
    assert on.pose_check == "on" and on.pose_check_bounds.e_slam == (0.05, 0.025)
    rec = run_transport.recorded_arguments(on)
    assert rec["pose_check"] == "on" and rec["pose_check_bounds"] == BOUNDS
    json.dumps(rec["pose_check_bounds"])  # plain lists in the record
    for extra in (["--pose-check", "on"],  # no development values (plan 0: no half state)
                  ["--pose-check", "on", "--pose-check-bounds", json.dumps({**BOUNDS, "e_still": [0.5, 0.01]})],
                  ["--pose-check", "on", "--pose-check-bounds", json.dumps({"odom": [0.0] * 4})],
                  ["--pose-check-bounds", json.dumps(BOUNDS)]):  # values without the check
        with pytest.raises(SystemExit):
            _runner_args(_live(tmp_path, *_reloc(tmp_path), *extra))
    with pytest.raises(SystemExit):  # not without the start relocalization
        _runner_args(_live(tmp_path, "--pose-check", "on", "--pose-check-bounds", json.dumps(BOUNDS)))


def test_the_bounds_may_come_from_a_file(tmp_path):
    f = tmp_path / "bounds.json"
    f.write_text(json.dumps(BOUNDS))
    on = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "on", "--pose-check-bounds", str(f)))
    assert on.pose_check_bounds.odom == (0.02, 0.02, 0.005, 0.01)


def test_a_bounds_file_without_the_release_limit_is_refused(tmp_path):
    no_release = {k: v for k, v in BOUNDS.items() if k != "e_release"}
    f = tmp_path / "bounds.json"
    f.write_text(json.dumps(no_release))
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "on", "--pose-check-bounds", str(f)))


def test_a_bounds_file_without_the_window_limit_is_refused(tmp_path):
    no_window = {k: v for k, v in BOUNDS.items() if k != "e_window"}
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "on", "--pose-check-bounds",
                           json.dumps(no_window)))


def test_the_arguments_are_in_the_near_field_contract():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["pose_check"] == "off"
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["pose_check_bounds"] is None


def _source():
    return (ROOT / "sim/isaac/run_transport.py").read_text()


def test_the_check_sits_where_the_plan_puts_it_in_the_tick():
    # plan 3.3 (Codex draft 1 P1-1): after the SLAM replies are applied and the control pose read, before any mission
    # work; again right before the output; the check's end before a session end request can make it a normal end
    src = _source()
    replies = src.index("core.slam_replies(t, replies_")
    first = src.index("pose_checkpoint(t)  # plan 3.3 (2)")
    work = src.index("if obstacle is not None:\n                    obstacle[\"control_rear\"]")
    assert replies < first < work
    second = src.index("pose_checkpoint(t)  # plan 3.3 (3)")
    assert second < src.index("live_safety_ = core.safety_state(t)\n                    out_ = live.tick_end(")
    top_end = src.index('if pose_box["end"] is not None:\n                # plan 3.3: the check ended')
    assert top_end < src.index("if live is not None and live.arbiter.session_request is not None:")


def test_every_new_path_is_behind_the_check():
    # the off path makes exactly the recorded calls (the replay gate): each new call is under pose_box
    src = _source()
    for needle in ("pose_box[\"rt\"].on_scan(", "pose_checkpoint(t)  # plan 3.3 (2)", "pose_checkpoint(t)  # plan 3.3 (3)",
                   "pose_box[\"rt\"] = make_pose_runtime(args"):
        at = src.index(needle)
        assert 'pose_box["rt"] is not None' in src[at - 260:at] or 'pose_box["sink"] is not None' in src[at - 260:at], \
            needle
    assert '**({} if pose_box["rt"] is None' in src  # tick_start's pose_ok only when on


def test_the_end_of_the_check_is_a_mission_failure_in_its_tick_and_a_session_failure_after():
    from forklift_core.mission.failures import classify

    assert classify("pose_ending:pose_revoked:int", strict=True) == "mission"
    assert classify("pose_revoked:int", strict=True) == "session"
    assert 'raise RuntimeError(f"pose_ending:{reason_}")' in _source()


def test_a_setup_recorded_before_the_check_replays_with_it_off():
    from tools import boundary_replay

    assert boundary_replay.SETUP_NAMES_ADDED["pose_box"] == ("pose_check", "off", {"rt": None, "sink": None,
                                                                                  "end": None})
    assert 'if "pose_box" not in _bound:' in _source()


def test_the_end_tick_refuses_this_ticks_submit_and_resume_on_copies():
    # Codex 2b P2-2: the boundary alone left an applied submit answered ack
    src = _source()
    at = src.index('if reason_.startswith("pose_ending:"):')
    assert 'dict(r_, accepted=False, reason="pose_ended")' in src[at:at + 600]
    assert src.index("live.end_mission(t_,", at) > at


def test_the_rear_check_walks_the_straight_reverses_own_bundle():
    # Codex 2b P2-3: kappa 0 alone passed where the permit's +-0.05 arcs stopped
    from forklift_core.safety.teleop_drive import TeleopLimits, bundle

    assert bundle(0.0, 0.0, TeleopLimits().curvature_margin_inv_m) == [-0.05, 0.0, 0.05]
    src = _source()
    at = src.index("def pose_rear_blocked(rt_)")
    assert "for kappa_ in bundle(0.0, 0.0, live.arbiter.config.teleop_limits.curvature_margin_inv_m):" in src[at:at + 2500]
    assert "with permission_kept(perm_):" in src[at:at + 3000]  # a check only (plan 2026-11-06 1.1)


def test_the_base_link_from_the_rear_axle():
    import math

    import run_transport

    assert run_transport.base_from_rear_axle((1.0, 2.0, math.pi / 2), 0.34) == pytest.approx((1.0, 2.34, math.pi / 2))


def test_the_state_carries_the_checks_view_for_the_console():
    # plan 4: LiveRuntime.publish adds pose_check when the runner set it (absent with the check off)
    live = (ROOT / "sim/isaac/live_runtime.py").read_text()
    assert 'if self.pose_info is not None:  # the unconfirmed position check' in live
    assert 'msg["pose_check"] = dict(self.pose_info)' in live
    src = _source()
    assert "live.pose_info = rt_.display(view_, None if applied_ is None else applied_[0], live_pallet_zones(), t=t_)" in src


# --- piece 2d (docs/plans/2026-11-06-unconfirmed-validation.md 1.1, 1.3) ---
def test_the_shadow_needs_the_relocalization_takes_no_development_values_and_is_recorded(tmp_path):
    import run_transport

    shadow = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "shadow"))
    assert shadow.pose_check == "shadow" and shadow.pose_check_bounds is None
    assert run_transport.recorded_arguments(shadow)["pose_check"] == "shadow"
    with pytest.raises(SystemExit):  # the provisional verdicts take no development values (plan 1.1)
        _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "shadow", "--pose-check-bounds",
                           json.dumps(BOUNDS)))
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path, "--pose-check", "shadow"))


def test_the_shadow_gates_nothing_in_the_runner():
    # plan 1.1: no arbiter.pose, no pose_ok, no zones (the runtime's), no ending
    src = _source()
    assert "pose_box[\"sink\"] = {}" in src and 'if getattr(args, "pose_check", "off") != "off":' in src
    at = src.index('pose_box["rt"] = make_pose_runtime(args')
    assert 'if not pose_box["rt"].shadow:' in src[at:at + 400]
    assert '**({} if pose_box["rt"] is None or pose_box["rt"].shadow' in src
    at = src.index("def pose_checkpoint(t_):")
    body = src[at:src.index("def pose_rear_blocked(rt_)")]
    assert "if not rt_.shadow:\n                a_.pose = rt_.view_dict(view_)" in body
    assert "if rt_.ends() and pose_box[\"end\"] is None:" in body
    at = src.index("def make_pose_runtime(")
    assert "shadow=args.pose_check == \"shadow\"" in src[at:at + 3000]
    assert "pose_runtime.SHADOW_BOUNDS if args.pose_check == \"shadow\"" in src[at:at + 3000]


def test_the_ticks_permission_is_the_final_output_after_tick_end():
    # plan 1.3: AUTO is the output's source (a zero candidate too); TELEOP armed with drive allowed in normal mode
    import run_transport
    from forklift_core.safety.arbiter import Output
    from forklift_core.safety.teleop_policy import Policy

    def out(source, armed=True, allowed=True, mode="normal"):
        return Output("X", "none", source, 0.0, 0.0, Policy(allowed, False, None, drive_mode=mode), 1, armed)

    assert run_transport.pose_open(out("auto", armed=False, allowed=False))
    assert run_transport.pose_open(out("teleop"))
    assert not run_transport.pose_open(out("teleop", armed=False))
    assert not run_transport.pose_open(out("teleop", allowed=False))
    assert not run_transport.pose_open(out("teleop", mode="reverse_straight"))
    assert not run_transport.pose_open(out("zero")) and not run_transport.pose_open(out("core_stop"))
    src = _source()
    end = src.index("live_safety_ = core.safety_state(t)\n                    out_ = live.tick_end(")
    note = src.index("pose_box[\"rt\"].note_tick(")
    assert note > end and "open_=pose_open(out_)" in src[note:note + 500]


def test_the_injection_is_for_unattended_relocalization_batches_only(tmp_path):
    # plan 1.2 (the lead, 2026-11-06): test-only, off by default, recorded only when given; not in an attended session
    import near_tracking
    import run_transport

    off = _runner_args(_live(tmp_path, *_reloc(tmp_path)))
    assert off.reloc_inject is None and "reloc_inject" not in run_transport.recorded_arguments(off)
    on = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--reloc-unattended", "--reloc-inject", "shift x+1"))
    assert run_transport.recorded_arguments(on)["reloc_inject"] == "shift x+1"
    for extra in (["--reloc-inject", "shift x+1"], ["--reloc-unattended", "--reloc-inject", "shift x+9"]):
        with pytest.raises(SystemExit):
            _runner_args(_live(tmp_path, *_reloc(tmp_path), *extra))
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path, "--reloc-inject", "turn"))
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["reloc_inject"] is None
    assert "inject=getattr(args, \"reloc_inject\", None)" in _source()


def test_the_contact_record_goes_with_the_check_and_reads_every_physics_step():
    # plan 1.4: every forklift rigid body reports; read after each physics step while the check's runtime exists
    import near_tracking
    import run_transport

    off = _runner_args(_live(tmp_path_ := Path(__import__("tempfile").mkdtemp()), *_reloc(tmp_path_)))
    assert off.pose_check_contacts is False and "pose_check_contacts" not in run_transport.recorded_arguments(off)
    on = _runner_args(_live(tmp_path_, *_reloc(tmp_path_), "--pose-check", "shadow", "--pose-check-contacts"))
    assert run_transport.recorded_arguments(on)["pose_check_contacts"] is True
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path_, *_reloc(tmp_path_), "--pose-check-contacts"))
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["pose_check_contacts"] is False
    src = _source()
    at = src.index("def step_world(render: bool)")
    body = src[at:src.index("if args.tyre_contact_log and tick % 12 == 0:", at)]
    assert 'pose_box["rt"] is not None and pose_box["rt"].contacts is not None' in body
    assert body.index("world.step(render=") < body.index("pose_box[\"rt\"].contacts.update(")
    assert 'if getattr(args, "pose_check_contacts", False):' in src
    at = src.index("def make_pose_runtime(")
    assert 'contacts=pose_contacts.ContactLog() if getattr(args, "pose_check_contacts", False) else None' in \
        src[at:at + 3500]


def test_the_mission_stands_scan_sets_are_saved_with_the_record():
    src = _source()
    at = src.index('state["pose_check"] = pose_box["rt"].record()')
    assert 'pose_box["rt"].save_stand_scans(args.output / "pose_check_stand_scans.npz")' in src[at:at + 400]
    assert 'if pose_box["rt"].stand_sets or pose_box["rt"].gen_points:' in src[at:at + 400]  # a generation's points too


def test_the_rear_checks_walk_leaves_the_permit_as_it_was_with_the_start_band_on():
    # the lead, 2026-11-06: the shadow writes nothing the control path reads -- a walk raises the start-band cells'
    # best distances and the counts; the whole permit is compared, not only those two
    import run_transport
    import shadow_audit

    sys.path.insert(0, str(ROOT / "tests/unit/control"))
    from test_start_release import FOOT, P0, SIDE, grid, path, perm

    snap, away = grid(SIDE), (0.0, -0.018, 0.0)
    plain = perm(True)
    before = shadow_audit.fingerprint(plain)
    plain.check_path(snap, path(-1, start=away), FOOT, FOOT, current_pose=away, direction=-1)
    assert shadow_audit.fingerprint(plain) != before  # without the guard the walk does change it
    kept = perm(True)
    before = shadow_audit.fingerprint(kept)
    with run_transport.permission_kept(kept):
        kept.check_path(snap, path(-1, start=away), FOOT, FOOT, current_pose=away, direction=-1)
    assert shadow_audit.fingerprint(kept) == before
    with pytest.raises(ValueError), run_transport.permission_kept(kept):  # put back on the way out of an error too
        kept.release_anchor = None
        raise ValueError
    assert shadow_audit.fingerprint(kept) == before and P0 == kept.release_anchor["pose"]


def test_the_shadow_audit_is_a_shadow_option_recorded_only_when_given(tmp_path):
    import near_tracking
    import run_transport

    off = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "shadow"))
    assert off.pose_shadow_audit is False and "pose_shadow_audit" not in run_transport.recorded_arguments(off)
    on = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "shadow", "--pose-shadow-audit"))
    assert run_transport.recorded_arguments(on)["pose_shadow_audit"] is True
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-shadow-audit"))
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "on", "--pose-check-bounds", json.dumps(BOUNDS),
                           "--pose-shadow-audit"))
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["pose_shadow_audit"] is False


def test_the_audit_brackets_each_place_the_shadow_computes():
    src = _source()
    at = src.index("def pose_checkpoint(t_):")
    body = src[at:src.index("def pose_rear_blocked(rt_)")]
    assert "if audit_on_:\n                rt_.audit.before()" in body
    assert 'rt_.audit.after("checkpoint", t_)' in body
    note = src.index('pose_box["rt"].note_tick(')
    assert 'pose_box["rt"].audit.before()' in src[note - 200:note]
    assert 'pose_box["rt"].audit.after("note_tick", t)' in src[note:note + 700]
    at = src.index('pose_box["rt"] = make_pose_runtime(args')
    block = src[at:at + 900]
    assert 'quiet_ = bool(a_.stop_confirmed) and a_.mode != "TELEOP"' in body
    assert "audit_on_ = rt_.audit is not None and rt_.audit.due(quiet=quiet_)" in body
    assert "if rt_.rear_pending():\n                if rt_.audit is not None and not audit_on_:" in body  # every rear check
    assert 'pose_box["rt"].audit.due(quiet=bool(live.arbiter.stop_confirmed) and live.arbiter.mode != "TELEOP")' in \
        src[note - 400:note]


def test_the_shadows_places_in_the_tick_write_only_their_own():
    # the lead, 2026-11-06 (Codex 2d P2-7): the writes listed -- the arbiter only when on, the published view, the end
    # only when on; nothing into state, the spool or the core
    src = _source()
    body = src[src.index("def pose_checkpoint(t_):"):src.index("def pose_rear_blocked(rt_)")]
    shadow_part = body[:body.index("if rt_.ends() and pose_box[\"end\"] is None:")]
    assert "state[" not in shadow_part and "core." in shadow_part  # core read only:
    assert "core.commands.status in ACTIVE" in shadow_part and "core.near_active()" in shadow_part
    assert "core.commands.status =" not in shadow_part
    assert shadow_part.count("a_.pose = ") == 1 and "if not rt_.shadow:\n                a_.pose = " in shadow_part
    assert "live.pose_info = " in shadow_part  # the published view
    rear = src[src.index("def pose_rear_blocked(rt_)"):src.index("def live_failed(t_, reason_")]
    assert "state[" not in rear and "with permission_kept(perm_):" in rear
    # Codex 2d re-review: slam_rear() writes the tracker's hold record; the check reads C o O itself
    assert "slam_rear(" not in rear and "compose(applied_[0], slam_odom_base())" in rear
    at = src.index('pose_box["rt"].on_scan(')
    assert "sent" in src[at:at + 120]
    at = src.index('pose_box["rt"] = make_pose_runtime(args')
    block = src[at:at + 1100]
    for name in ("arbiter", "layer", "commands", "cargo", "tracker"):
        assert f'"{name}": lambda:' in block, name


def test_the_check_is_told_the_trackers_hold():
    src = _source()
    at = src.index("def pose_checkpoint(t_):")
    body = src[at:src.index("def pose_rear_blocked(rt_)")]
    assert body.count('holding=slam["tracker"].mode == "holding"') == 2


def test_the_correction_injection_is_a_test_of_a_confirmed_session(tmp_path):
    # the revision plan 2 D: test only, unattended, with the check, recorded only when given
    import near_tracking
    import run_transport

    off = _runner_args(_live(tmp_path, *_reloc(tmp_path)))
    assert off.correction_inject is None and "correction_inject" not in run_transport.recorded_arguments(off)
    on = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--reloc-unattended", "--pose-check", "shadow",
                            "--correction-inject", "0.35,0,0,200"))
    assert on.correction_inject == (0.35, 0.0, 0.0, 200.0)
    assert run_transport.recorded_arguments(on)["correction_inject"] == [0.35, 0.0, 0.0, 200.0]
    held = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--reloc-unattended", "--pose-check", "shadow",
                              "--correction-inject", "0,0,0.12,0,hold"))
    assert held.correction_inject == (0.0, 0.0, 0.12, 0.0, "hold")
    assert run_transport.recorded_arguments(held)["correction_inject"] == [0.0, 0.0, 0.12, 0.0, "hold"]
    for extra in (["--pose-check", "shadow", "--correction-inject", "0.35,0,0,200"],  # attended
                  ["--reloc-unattended", "--correction-inject", "0.35,0,0,200"],  # no check
                  ["--reloc-unattended", "--pose-check", "shadow", "--correction-inject", "0.35,0,0"],
                  ["--reloc-unattended", "--pose-check", "shadow", "--correction-inject", "0,0,0.12,0,soon"]):
        with pytest.raises(SystemExit):
            _runner_args(_live(tmp_path, *_reloc(tmp_path), *extra))
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["correction_inject"] is None
    src = _source()
    assert "def correction_injection(t_):" in src
    assert 'wait_ and slam["tracker"].mode != "holding"' in src  # a hold's jump waits for the hold
    at = src.index("correction_injection(t)\n")
    assert 'if args.correction_inject is not None:' in src[at - 120:at]
    assert 'pose_box["rt"] is not None' in src[at - 200:at]
