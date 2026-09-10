# Local sensor core validation

Date: 2026-09-10.

## Scope

This is an offline numerical test increment. Input depth grids, scan values, intrinsic calibration and mounting transforms are synthetic fixtures, not recordings from D435i or RPLIDAR A2. The user-selected camera is D435i; RPLIDAR is confirmed with A2 planned; Jetson is provisional and chassis identification deferred.

## Executed checks

| Check | Observed result |
|---|---|
| RED run before implementation | 62 failed: the package/entrypoint did not exist. Tests collected and ran; no missing test dependency |
| GREEN run after implementation | 62 passed; warnings treated as errors |
| Review regression tests before correction | 2 failed, 12 passed, 50 deselected; overflow warnings reproduced for extreme finite metadata |
| Final suite after correction | **64 passed**; warnings treated as errors |
| `python -m forklift_core.demo` | Exit 0, valid JSON; metric coordinates and missing-sample masks emitted |

Reproduce from the project root with the project Python dependencies installed:

```bash
python -m pytest tests -q -p no:cacheprovider -W error
python -m forklift_core.demo
```

The tests were executed with NumPy 2.4.6 and pytest 9.1.0. The declared minimum dependency versions have not been separately tested.

An independent read-only code review found no major implementation defect, but identified arithmetic-overflow warnings escaping metadata validation. Both cases were reproduced as failing regression tests and corrected so invalid metadata raises `ValueError`. The final 64-test run includes the example subprocess check.

## Verified behavior

- Rectified depth-grid pixels deproject with an explicit metres-per-unit scale; optical +x is right, +y down, +z forward.
- A supplied proper rotation and translation map points into the declared target frame. Incorrect source frames, reflection/scaling matrices and invalid calibration are rejected.
- LiDAR ranges convert in the laser frame with an explicit angular origin, signed angular step and supplied range bounds.
- Missing/out-of-range samples retain their indices as all-NaN points. The example serializes these as JSON null with false validity flags.
- RGB pixels from another named grid are rejected. This name check cannot prove real image alignment; acquisition adapters must perform and verify registration.

## Not verified or implemented

- Live D435i/RPLIDAR drivers, IMU data, actual camera distortion correction, image registration, calibration, minimum sensing distance, range accuracy or sensor mounting.
- Timestamp synchronization, rolling scan motion compensation, TF lookup at capture time, perception freshness or actuator watchdogs.
- Pallet/pocket detection, tracking, SLAM, navigation or A–D task acceptance.
- Robot geometry, steering, fork insertion/loading contact, chassis simulation or real motion.
- Jetson/JetPack deployment, ROS distribution choice or integrated simulator rendering.

A missing beam is unknown, not free space. A successful transform is not a valid pallet pose by itself. A zero-contact insertion requirement does not prohibit the intended support contact when lifting; insertion and loading must be evaluated separately in later work.
