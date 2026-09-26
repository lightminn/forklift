"""Qualify current known-model roof tracking against directly seen pockets.

This adapter never extrapolates a missing observation. After qualification the
roof stream is authoritative: its loss must reach the servo and stop motion.
"""

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.perception.pocket_observation import PocketObservation


@dataclass(frozen=True)
class HandoffSelection:
    observation: PocketObservation
    mode: str
    confirmed_frames: int
    agreement_position_m: float | None
    agreement_yaw_rad: float | None
    agreement_size_m: float | None


class RoofHandoff:
    """Three consecutive, paired observations establish the model continuation."""

    def __init__(
        self,
        *,
        start_front_x_m=1.4,
        confirmation_frames=3,
        position_tolerance_m=0.025,
        yaw_tolerance_rad=0.03,
        size_tolerance_m=0.02,
    ):
        if type(confirmation_frames) is not int or confirmation_frames < 1:
            raise ValueError("confirmation_frames must be a positive integer")
        for value in (
            start_front_x_m,
            position_tolerance_m,
            yaw_tolerance_rad,
            size_tolerance_m,
        ):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError("handoff limits must be finite and positive")
        self.start_front_x_m = start_front_x_m
        self.confirmation_frames = confirmation_frames
        self.position_tolerance_m = position_tolerance_m
        self.yaw_tolerance_rad = yaw_tolerance_rad
        self.size_tolerance_m = size_tolerance_m
        self.confirmed = 0
        self.qualified = False
        self.last_stamp = None

    def select(self, front, roof):
        position_error = yaw_error = size_error = None
        if not self.qualified:
            paired = (
                front.status == roof.status == "valid"
                and front.source_provenance != "synthetic_ground_truth"
                and roof.source_provenance != "synthetic_ground_truth"
                and front.frame_id == roof.frame_id == "base_link"
                and front.clock_domain == roof.clock_domain
                and front.stamp_ns == roof.stamp_ns
                and (self.last_stamp is None or front.stamp_ns > self.last_stamp)
            )
            if paired:
                midpoint = (np.array(front.left.center_m) + front.right.center_m) / 2
                pairs = ((front.left, roof.left), (front.right, roof.right))
                position_error = max(
                    float(np.linalg.norm(np.array(a.center_m) - b.center_m))
                    for a, b in pairs
                )
                size_error = max(
                    max(abs(a.width_m - b.width_m), abs(a.height_m - b.height_m))
                    for a, b in pairs
                )
                delta = front.insertion_yaw_rad - roof.insertion_yaw_rad
                yaw_error = abs(math.atan2(math.sin(delta), math.cos(delta)))
                paired = (
                    midpoint[0] <= self.start_front_x_m
                    and position_error <= self.position_tolerance_m
                    and yaw_error <= self.yaw_tolerance_rad
                    and size_error <= self.size_tolerance_m
                )
            self.confirmed = self.confirmed + 1 if paired else 0
            self.last_stamp = front.stamp_ns
            self.qualified = self.confirmed >= self.confirmation_frames
        return HandoffSelection(
            roof if self.qualified else front,
            "roof_model" if self.qualified else "front_pockets",
            self.confirmed,
            position_error,
            yaw_error,
            size_error,
        )
