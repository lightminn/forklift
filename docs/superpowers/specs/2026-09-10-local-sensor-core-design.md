# Local sensor core design

## Confirmed direction

User decisions on 2026-09-10:

- RGB-D camera: RealSense D435i, confirmed. Gemini 335Le belongs to the original assignment brief, not the selected camera.
- LiDAR: SLAMTEC RPLIDAR, confirmed at family level; the follow-up about using the supplied brief's model is interpreted as **A2 planned**, not a confirmed A2 subvariant. Do not assume serial baud rate, scan frequency, or range specification before identifying that subvariant.
- Upper controller: a Jetson-series SBC is anticipated; the model and JetPack version are unselected. An existing machine named Jetson is not thereby assigned to this project.
- Chassis: defer identification and motion modelling until the hardware arrives.
- ROS 2 remains the project direction; local development and code tests can precede chassis selection.

## First deliverable

A NumPy-only, hardware-independent sensor geometry package, automated tests, and a runnable synthetic-input example. This is an initial code-testing foundation, not a forklift simulator. It does not choose a simulator or create a ROS workspace.

The five assignment module boundaries remain detection → robot-base pose → obstacle-aware path → following/insertion/loading → transport/unloading. This increment implements numerical primitives used by the second stage and LiDAR input conversion; it does not implement the entire pipeline.

## Contracts

- `FramePoints(frame_id, xyz_m)` contains N × 3 points in metres. Each row is either entirely finite or entirely NaN for an unavailable measurement; infinity and partially invalid rows are rejected. `valid` preserves the input sample positions.
- `RigidTransform(source_frame, target_frame, rotation, translation_m).apply(points)` requires an explicit proper rotation, translation, and matching source frame. No real sensor-to-base calibration is invented. The transform is p_target = R × p_source + t.
- `PinholeIntrinsics(width, height, fx, fy, cx, cy, frame_id)` describes an explicitly rectified depth pixel grid. Focal lengths must be positive and calibration values finite.
- `deproject_depth_pixels(depth, pixels_uv, intrinsics, *, meters_per_unit, pixel_frame, rectified)` accepts only integer pixel indices in the supplied depth grid. The caller explicitly supplies the depth scale, matching frame, and rectification status. Zero, negative, NaN and infinite depths yield invalid points; an image/grid mismatch, out-of-bounds pixel, unsupported unrectified input or invalid calibration raises `ValueError`. A colour detection must first be registered into this grid by a future sensor adapter. Image shape alone is not alignment evidence.
- `scan_to_points(ranges_m, *, angle_min_rad, angle_increment_rad, range_min_m, range_max_m, frame_id)` uses ROS LaserScan conventions: metres, radians, zero angle along +x, positive angle toward +y. Inclusive range limits come from input metadata, not an assumed RPLIDAR model. Invalid beams remain NaN rows, never zero-distance points or evidence of free space. Negative nonzero angle increments are supported; metadata must be finite and bounds valid.
- No live device discovery, network access, GPU, ROS, or camera SDK is required by this increment. Python >= 3.10, NumPy >= 1.23; development tests use pytest >= 7.

## Evidence and limitations

Fixtures use small hand-computable depth grids and scans, with a labelled synthetic camera mounting transform. They do not model D435i noise, minimum range, stereo holes, optical distortion, RPLIDAR timing, or actual mounting geometry. NaN samples mean unknown, not clear space. Scan motion compensation is not implemented. Neither points nor this example authorize motion.

Tests must independently check units, axis direction, transformed origin, invalid data preservation, frame rejection, calibration validation, and malformed metadata. A subprocess test exercises the documented example command. Record the failing run before implementing and the passing run afterwards.

No actual RGB-D detector, pocket tracker, SLAM, planner, controller, loading physics, or A–D task success is claimed. Future live/replay adapters must preserve acquisition timestamps and coordinate frames, synchronize RGB/depth/TF, and use runtime calibration. A separate observation watchdog/state-machine design follows that input contract. Chassis dimensions, steering, turning radius, fork motion, and loaded footprint remain deferred.

## Reference interfaces

- RealSense ROS wrapper: https://github.com/realsenseai/realsense-ros (optical frames, camera information and aligned depth interfaces).
- SLAMTEC ROS 2 driver: https://github.com/Slamtec/sllidar_ros2 (model-specific launch configuration).
- ROS LaserScan message: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg.

Sensor and SBC selections above come from the user; driver execution and Jetson deployment are not verified by reading these sources.
