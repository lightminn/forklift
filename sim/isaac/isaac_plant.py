"""The Isaac plant's actuation, extracted from drive() (sim-to-real plan P0-4 step 4a).

A mechanical extraction: every method makes exactly the robot calls drive() made at its sites, in the
same order with the same arguments, so a recorded run replays unchanged (the "Vehicle" and "Controller"
ports see the same calls). drive() builds one after its robot is wrapped for the record and rebinds
``robot`` whenever drive() rebinds its own.

TEMPORARY SIGNATURE: the common contract the real-vehicle port shares (decision time, absolute expiry,
generation, lift_hold, a queued / refused / blocked result, one owner of the DRIVE frame) is fixed in
step 4a' together with the vehicle kinematics injection; nothing here is that contract yet. The gain
methods are Isaac-only (the standstill wheel lock emulates a hold the real MCU does with its own PI).
"""

from __future__ import annotations

import numpy as np


class IsaacPlant:
    def __init__(self, robot, action, wheels, steers, lift_index) -> None:
        self.robot = robot  # drive()'s current robot (the record proxy when recording)
        self.action = action  # isaac.action: ArticulationAction, or a replay's CPU stand-in
        self.wheels, self.steers, self.lift_index = wheels, steers, lift_index

    def zero_wheels(self) -> None:
        """Every wheel's velocity target to zero at once; steering and lift untouched."""
        self.robot.apply_action(self.action(joint_velocities=np.zeros(len(self.wheels)), joint_indices=self.wheels))

    def hold_lift(self, position) -> None:
        """The lift's position target."""
        self.robot.apply_action(self.action(joint_positions=np.array([position]), joint_indices=self.lift_index))

    def drive_wheels(self, wheel_rates, lock_q) -> None:
        """The wheels' velocity targets with the standstill lock's positions (None when unlocked)."""
        self.robot.apply_action(self.action(joint_velocities=wheel_rates, joint_positions=lock_q,
                                            joint_indices=self.wheels))

    def steer(self, steering) -> None:
        """The two front steering joints' position targets."""
        self.robot.apply_action(self.action(joint_positions=steering, joint_indices=self.steers))

    def controller(self):
        """Isaac only: the articulation controller whose drive gains the wheel lock reads and sets."""
        return self.robot.get_articulation_controller()


__all__ = ["IsaacPlant"]
