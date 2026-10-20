"""The delivery check on the simulation's truth (docs/plans/2026-10-18-persistent-live-console.md D1 step 6).

A batch run calls it where it always judged the delivery, after the mission loop; a live run at each mission's end.
The checks, their limits, messages and order are the batch run's: the pallet's centre within 0.08 m of the
destination, the pallet grounded (|z| < 0.008 m), and the fork tip out of it along the destination's axis by half the
pallet depth plus the exit clearance.
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Callable

import numpy as np

MISSED = "Pallet missed the green destination center"
NOT_GROUNDED = "Delivered pallet is not grounded"
FORK_INSIDE = "Fork still inside delivered pallet"


def delivery_verdict(final_pallet, base_pose: Callable[[], tuple], destination, *, pallet_depth_m: float,
                     axle_to_fork_tip_m: float, rear_axle_offset_m: float, exit_clearance_m: float) -> SimpleNamespace:
    """``final_pallet`` (x, y, z) is the pallet's true world position; ``base_pose()`` gives base_link's true (x, y)
    and yaw -- read only once the first two checks pass, as the batch run read it; ``destination`` is the true
    destination (x, y, yaw). Returns ``ok``, ``reason`` (the first failed check's message, or None) and
    ``delivery_error_m``."""
    pallet = np.asarray(final_pallet, dtype=float)
    delivery_error = float(np.linalg.norm(pallet[:2] - np.array([destination[0], destination[1]], dtype=float)))
    reason = None
    if not delivery_error < 0.08:
        reason = MISSED
    elif not abs(pallet[2]) < 0.008:
        reason = NOT_GROUNDED
    else:
        # final_base is base_link, not the rear axle -- axle_to_fork_tip_m is measured from the axle, so subtract
        # the axle-to-base_link offset first.
        final_base, final_yaw = base_pose()
        tip = np.asarray(final_base, dtype=float)[:2] + (axle_to_fork_tip_m - abs(rear_axle_offset_m)) * np.array(
            [math.cos(final_yaw), math.sin(final_yaw)])
        axis = np.array([math.cos(destination[2]), math.sin(destination[2])])
        if not float(np.dot(pallet[:2] - tip, axis)) > pallet_depth_m / 2 + exit_clearance_m:
            reason = FORK_INSIDE
    return SimpleNamespace(ok=reason is None, reason=reason, delivery_error_m=delivery_error)
