"""The operating unlock U3c (docs/plans/2026-11-26-real-operating-assembly.md 2.9): real_status.preconditions on the
operating run -- the assembly's passed items and the run-time rows (C7 IMU, C8 D5 and, without depth, the per-pallet
confirmation, the scan source's newest refusal), each pass / blocked / bypassed (the scan row a note)."""

from forklift_core.live.real_preconditions import ASSEMBLY_CHECKS

from real_loop_rig import Rig
from test_real_loop_console import ask
from test_real_loop_u3b2 import _confirmed, register, rig


def rows(r):
    pre = r.loop.real_status(r.now)["preconditions"]
    return {(x["id"], x["item"]): (x["state"], x["why"]) for x in pre["runtime"]}


def test_only_the_operating_run_has_precondition_rows():
    r = Rig(real_requests=True)
    r.start()
    assert r.loop.real_status(r.now)["preconditions"] is None  # HIL: nothing the operating run judges


def test_without_depth_the_d5_rows_follow_the_bypass_and_the_confirmation():
    r = rig()
    assert rows(r) == {("C7", "imu"): ("pass", None),
                       ("C8", "d5_pocket"): ("blocked", "d5_pocket:no_depth_source"),
                       ("C8", "pallet_confirm"): ("blocked", "operator_start:unconfirmed"),
                       ("C5", "scan"): ("note", None)}
    _confirmed(r)  # the D5 bypass, a registration and its confirmation
    got = rows(r)
    assert got[("C8", "d5_pocket")] == ("bypassed", None) and got[("C8", "pallet_confirm")] == ("pass", None)
    register(r)  # registering again voids it
    assert rows(r)[("C8", "pallet_confirm")] == ("blocked", "operator_start:unconfirmed")


def test_a_run_with_depth_has_no_confirmation_row():
    r = rig(no_depth=False)
    got = rows(r)
    assert got[("C8", "d5_pocket")] == ("pass", None) and ("C8", "pallet_confirm") not in got


def test_an_unqualified_imu_is_blocked_until_bypassed():
    r = rig(imu_unqualified=True)
    assert rows(r)[("C7", "imu")] == ("blocked", "imu:unqualified")
    reply, _ = ask(r, "check_bypass", check="imu", on=True)
    assert reply["t"] == "ack", reply
    r.tick()
    assert rows(r)[("C7", "imu")] == ("bypassed", "imu:unqualified")


def test_the_scan_row_tells_the_newest_refusal_without_blocking():
    r = rig()
    r.loop.scan_refusal = lambda: "frame:stale"
    assert rows(r)[("C5", "scan")] == ("note", "frame:stale")


def test_the_summary_and_the_load_are_the_runs():
    r = rig()
    assert r.loop.real_status(r.now)["preconditions"]["passed"] == []  # a rig the assembly did not judge
    r.loop.preconditions_passed = ASSEMBLY_CHECKS
    r.loop.operator_load = {"lift_target_m": 0.2, "test_pallet": "P-S10", "pallet_mass_kg": 4.5}
    pre = r.loop.real_status(r.now)["preconditions"]
    assert pre["passed"] == list(ASSEMBLY_CHECKS)
    assert pre["load"] == {"lift_target_m": 0.2, "test_pallet": "P-S10", "pallet_mass_kg": 4.5}
