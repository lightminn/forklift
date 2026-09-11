# Local Sensor Core Implementation Plan

**Goal:** Run reproducible, chassis-independent depth and LiDAR geometry tests on the local machine.

**Architecture:** Separate frame geometry, rectified depth conversion, and scan conversion. A synthetic example composes the same public functions tested by pytest; no device or ROS adapter is included.

**Tech Stack:** Python >= 3.10, NumPy >= 1.23, pytest >= 7.

**Spec:** `docs/design/2026-09-10-local-sensor-core-design.md`

## Global constraints

Retain the confirmed D435i and RPLIDAR selections and provisional Jetson direction. Defer chassis, simulator and ROS distribution selection. Do not change the presentation repository, initialize Git, commit, install packages, or access robot hardware. Use the existing local Python environment for execution; repository instructions use portable `python` commands.

## Task 1: Frame geometry and sensor conversion

Files: `forklift_core/__init__.py`, `forklift_core/geometry.py`, `forklift_core/rgbd.py`, `forklift_core/lidar.py`, `tests/test_geometry.py`, `tests/test_rgbd.py`, `tests/test_lidar.py`, `pyproject.toml`.

- [x] Write literal expected-coordinate tests for the interfaces specified in the design, including zero/invalid samples and malformed metadata. Examples: depth 2000 with scale 0.001 gives z=2; a 90-degree LiDAR beam at range 2 gives (0,2,0); an optical-forward point (0,0,2) transformed by the example R and t=(0.2,0,0.5) gives (2.2,0,0.5).
- [x] Run `python -m pytest tests -q` and retain the failing result before adding implementation modules.
- [x] Implement validated `FramePoints` and `RigidTransform.apply`, then `PinholeIntrinsics` and `deproject_depth_pixels`, then `scan_to_points`. Preserve invalid sample positions and reject inconsistent frames.
- [x] Run the same suite and correct production defects until every declared contract passes.

## Task 2: Runnable synthetic example and documentation

Files: `forklift_core/demo.py`, `tests/test_demo.py`, `README.md`, `docs/LOCAL_VALIDATION.md`.

- [x] First add a subprocess test running `python -m forklift_core.demo`, requiring valid JSON, the known transformed point, and a preserved unavailable LiDAR beam.
- [x] Confirm that this command fails before implementation.
- [x] Implement a deterministic example with explicit fictional calibration; output coordinates and validity masks, not a robot-success verdict.
- [x] Update the README with hardware decisions, current scope, commands and next work. Keep the assignment's reference hardware distinct from selected equipment.
- [x] Run `python -m pytest tests -q` and `python -m forklift_core.demo`; record actual results and unverified boundaries in `docs/LOCAL_VALIDATION.md`.

Execution stays in this workspace; no Git repository is available for branch/worktree operations. Completion means the two local deliverables pass, not that the robot or sensor drivers have been implemented.
