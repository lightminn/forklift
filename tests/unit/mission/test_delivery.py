"""The delivery check moved into a function (docs/plans/2026-10-18-persistent-live-console.md D1 step 6): the batch
run calls it where it always judged the delivery, a live run at each mission's end. The messages and their order
are the batch run's."""

import math

import pytest

from forklift_core.mission.delivery import delivery_verdict

DEST = (5.0, 2.0, 0.0)  # x, y, yaw: the pallet's approach axis points along +x
K = dict(pallet_depth_m=1.2, axle_to_fork_tip_m=1.6, rear_axle_offset_m=0.3, exit_clearance_m=0.08)


def verdict(pallet=(5.0, 2.0, 0.0), base=(3.0, 2.0), yaw=0.0, dest=DEST, reads=None):
    def base_pose():
        if reads is not None:
            reads.append(1)
        return base, yaw

    return delivery_verdict(pallet, base_pose, dest, **K)


def test_a_centred_grounded_pallet_with_the_forks_out_is_delivered():
    v = verdict()
    assert v.ok and v.reason is None and v.delivery_error_m == pytest.approx(0.0)


@pytest.mark.parametrize("pallet, base, reason", [
    ((5.09, 2.0, 0.0), (3.0, 2.0), "Pallet missed the green destination center"),
    ((5.0, 2.0, 0.01), (3.0, 2.0), "Delivered pallet is not grounded"),
    # tip = base + (1.6 - 0.3) along +x = 4.3 + 0.1; pallet - tip = 0.6 < 0.6 + 0.08
    ((5.0, 2.0, 0.0), (3.1, 2.0), "Fork still inside delivered pallet"),
])
def test_each_failure_names_the_batch_runs_message(pallet, base, reason):
    v = verdict(pallet, base)
    assert not v.ok and v.reason == reason


def test_the_first_failure_in_the_batch_runs_order_is_named():
    v = verdict((5.2, 2.0, 0.05), (4.5, 2.0))
    assert v.reason == "Pallet missed the green destination center" and v.delivery_error_m == pytest.approx(0.2)


def test_the_fork_exit_is_measured_along_the_destination_axis():
    dest = (5.0, 2.0, math.pi / 2)  # the axis along +y: a truck below the pallet
    assert verdict((5.0, 2.0, 0.0), (5.0, 0.0), math.pi / 2, dest).ok
    assert verdict((5.0, 2.0, 0.0), (5.0, 0.2), math.pi / 2, dest).reason == "Fork still inside delivered pallet"


def test_the_base_pose_is_read_only_after_the_first_two_checks_pass():
    """The batch run read the truck's pose after them: its port calls (compared by the replay gate) stay the same."""
    reads = []
    verdict((5.2, 2.0, 0.0), reads=reads)
    verdict((5.0, 2.0, 0.05), reads=reads)
    assert reads == []
    verdict(reads=reads)
    assert reads == [1]
