"""Host side of the Jetson <-> MCU link (mcu-link/1): frames and messages only.

No serial port, ROS or other forklift_core module is imported here; an adapter owns
the transport. Contract: ``docs/interfaces/2026-10-22-mcu-link-contract.md``.
"""
