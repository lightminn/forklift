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

    def command(self, cmd, lock_q=None):
        """The shared contract (4a', forklift_core.plant.port.PlantCommand) on this plant: the wheels (with the
        Isaac-only standstill lock), the steering and the lift target, then ``queued``. drive() keeps calling the
        three methods itself until 4e-1 (a write sits between them); this is the contract's Isaac side."""
        from forklift_core.plant.port import PortResult

        self.drive_wheels(np.asarray(cmd.drive.wheel_rates_rad_s), lock_q)
        self.steer(np.asarray(cmd.steering_command))
        if cmd.lift_mode == "target":
            self.hold_lift(cmd.lift_m)
        return PortResult("queued")

    def sample(self, t=None):
        """P0-4 4b: one joint read as a measurement -- exactly one ``get_joint_positions()``, the steering and lift
        indexed as drive() did (``q[steers]``, ``q[lift_index[0]]``), the array kept (the slip watch reuses it).
        ``t`` is the caller's simulation time: the plant reads no clock (the world is a recorded port)."""
        from forklift_core.plant.port import PlantSample

        q = self.robot.get_joint_positions()
        return PlantSample(steering=q[self.steers], lift_m=q[self.lift_index[0]], sample_time_s=t, raw_joints=q)

    def joint_velocities(self):
        """Isaac only: the joint velocity read drive() makes (one ``get_joint_velocities()``)."""
        return self.robot.get_joint_velocities()

    def controller(self):
        """Isaac only: the articulation controller whose drive gains the wheel lock reads and sets."""
        return self.robot.get_articulation_controller()


__all__ = ["IsaacPlant"]
