# Isaac Sim Hybrid A* transport preview

`run_transport.py` places official NVIDIA Simple Warehouse barrel, plastic crate,
and cardboard-box assets on a flat warehouse bay. A seeded pallet pickup and
destination are connected by forward/reverse Hybrid A* paths. The destination
is a green ground ring; a fixed overhead camera records the whole bay at 60fps.

Install this repository into the Isaac Sim Python environment with
`python -m pip install -e '.[dev]'`. Run inside an allocated GPU job:

```bash
python sim/isaac/run_transport.py \
  --base-scene /path/to/imported-forklift-warehouse/scene.usda \
  --pallet-urdf sim/models/epal6_pallet/pallet.urdf \
  --settings config/isaac_transport.yaml \
  --output /path/to/new-run --seed 0 --video
```

Use Isaac Sim's Python launcher in place of `python`. The base scene must contain
this project's imported provisional forklift at `/World/Forklift` and NVIDIA's
warehouse environment. The output directory must not already exist. Kit options,
such as a writable portable cache, can be appended. Outputs include scenario,
paths, sampled physical state, source hashes, scene USD, screenshots, and video.
Inspect `result.json` for success or failure; a video alone does not prove success.

Physics runs at 120Hz, recording at 60fps by simulation time. This does not promise
60fps real-time server throughput. Steering force and gains in the YAML are
synthetic simulation settings; the original model parameters remain unchanged.
Pose and obstacle feedback use simulator ground truth. The pallet remains a free
rigid body lifted by contact, with no attachment or pose changes during driving.

Cruise speeds are explicit YAML settings: approach 0.60 m/s, transport 0.30 m/s,
extraction 0.18 m/s and withdrawal 0.12 m/s. Insertion remains 0.055 m/s.
Braking and measured steering response can reduce the commanded speed.
The first baseline used approach 0.14 m/s and transport 0.12 m/s.

The user resumed the physical test bench after the preview. Retained records and
limits are described in the dated validation notes; this remains a synthetic
controller experiment, not hardware acceptance.

## Camera-guided alignment approach

`run_perception_approach.py` constructs a fresh warehouse and imports the URDFs;
it does not require a saved base scene. It uses the existing EPAL 6 pocket detector
on Isaac RGB-D images, transforms the observation at its acquisition time, and
drives to the estimated alignment point. It stops there; insertion requires the
next continuous pocket-tracking stage.

```bash
python sim/isaac/run_perception_approach.py \
  --output /path/to/new-approach-run --seed 0 --video
```

The synthetic fixed mount is in `config/isaac_perception_camera.yaml`. RGB and
optical-axis Z depth in metres share one undistorted render product. Physics stays
frozen during capture, detection and planning, pairing the image with the exact
robot transform. No cached SDK rendering timestamp is relabelled as acquisition
time. Raw RGB, floating depth, calibration, both transforms and acquisition time
are saved in `observation/`; the existing detector uses the EPAL 6 derived
parameters without a new detector or depth quantization. The factory-specific
`config/isaac_perception_detector.yaml` raises the candidate-plane budget from
three to twelve: intervening prop faces exhausted smaller budgets in the saved
seed 0 and seed 17 captures. Geometric acceptance gates and the 5 m camera-range limit remain
unchanged. This setting has a recorded positive/negative replay check; it is not
a general detector success-rate claim.

Robot localization and factory-prop bounds still use simulator information.
Pallet position enters the planner only through detection and the prior; a
downstream evaluator uses scene truth after driving to check alignment error and
pallet displacement. As in the existing transport approach, planner clearance is
capped at 0.05 m and uses the existing
unloaded footprint, with an alignment endpoint nominally 0.10 m before the pallet.
These are synthetic experiment margins, not calibrated sensor uncertainty bounds.

`--hide-pallet` supplies a negative rendering experiment; `--capture-only` records
detection without driving. Rejected observations produce `stopped_no_target`
with `success: false` and no wheel motion command. `result.json` distinguishes
arrival at the observed goal from the downstream evaluated `aligned` result.
`stop.json` verifies measured stopping after a started approach, including its
failure path. Inspect result files rather than relying on the process exit code,
because application shutdown may override it. Video remains a 60fps overview by
simulation time. This mode does not yet handle a moving pallet or continuous
reacquisition during approach.

## Continuous pocket observation and fail-stop insertion experiment

```bash
python sim/isaac/run_pocket_insertion.py \
  --output /path/to/new-pocket-run --seed 0 --video
python sim/isaac/run_pocket_insertion.py \
  --output /path/to/new-pocket-loss-run --seed 0 --video --drop-after-s 2
```

The second command injects missing depth two simulated seconds after the first
tracking motion command. Both runs first use the original detector and Hybrid A*
to reach an earlier observation handoff. `config/isaac_pocket_insertion.yaml`
defines the synthetic servo envelope. New depth observations arrive every 0.1 s
of simulation time; physics is frozen during rendering/detection. The video is
60fps by simulation time, not a real-time perception throughput measurement.
The requested free-approach speed is 0.60 m/s, with acceleration limited to
0.60 m/s² in the pocket servo. At a measured front-plane distance of 1.35 m,
the servo requests 0.055 m/s before the 0.95 m fork tip reaches the pallet.
Loss and lower speed requests brake immediately; they do not wait for a command
ramp. These are simulator settings, not measured hardware braking limits.

Near observations must remain within 30 mm and 0.03 rad of the object initially
estimated from the camera. Only this associated stage enables the detector's
`median_plane_offset` option. It corrects plane-offset bias but can also admit
column/shelf lookalikes, so it is not bottom-board or semantic certification.
The default acquisition detector retains its original mean plane offset.

Three fresh, consistent observations while stopped are required to begin.
Loss, stale data, association failure, excessive motion or insufficient nominal
fork clearance command zero. A watchdog also checks held commands between
captures. The core permits confirmed reacquisition; this experiment latches the
first loss for the trial. Simulator pallet geometry only supplies an independent
abort guard and downstream evaluation, never the tracking target.

`pocket_observations.json` and `pocket_frames/` retain observations, commands,
transforms and compressed raw depth. `pocket_braking.json` verifies linear and
angular stopping even after capture/detection exceptions. `pocket_stop.json`
reports the trial outcome and measured displacement after loss. Inspect
`result.json`; a stopped trial is not a successful insertion.

The fixed camera loses the front pocket face at close range. For the known empty
EPAL 6 model, `roof_tracking.py` measures the current roof rear edge and board-gap
pattern, then infers the hidden pocket positions. Three consecutive paired frames
must agree on each pocket center (25 mm), heading (0.03 rad), and opening dimensions
(20 mm) before `pocket_tracking_handoff.py` selects `roof_model`. Missing roof depth
then stops the trial; it never substitutes a cached pose. The initial camera target
only bounds association. A solid slab, unknown rear/grooves, mismatched board layout
and foreground rear occlusion are rejected by focused tests.

`pocket_observations.json` distinguishes `front_pockets` from `roof_model`, records
both detections and qualification, and stores pallet truth explicitly for offline
evaluation. This continuation requires an empty rigid slatted pallet; it cannot
observe a new obstruction in a hidden pocket. It is not general semantic recognition
or measured camera calibration. See
[`2026-09-18-roof-assisted-insertion.md`](../../docs/validation/2026-09-18-roof-assisted-insertion.md)
for actual insertion and stopping evidence. To inject missing depth after the forks
have entered in the validated seed-0 configuration, use `--drop-after-s 10`.
