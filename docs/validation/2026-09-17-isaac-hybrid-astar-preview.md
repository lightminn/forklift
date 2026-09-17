# Hybrid A* first video checkpoint

The user requested the first video only and explicitly deferred the main test
campaign. This checkpoint is **not** a successful transport acceptance result.

Artifact: `artifacts/20260916T151004Z_hybrid_transport/preview02_seed0/run/transport.mp4`.
`ffprobe` reports 1280×720, 60/1 fps, 5,041 frames, 84.016667 seconds.
The first and last decoded frames were visually inspected. The whole bay,
NVIDIA barrel/crates/cardboard box, random pickup pallet, forklift and green
destination ring are visible. Seed 0 contains four official warehouse props.

Observed transitions from the retained Slurm log:

- Approach → insertion, then contact lift.
- Lift → extraction at simulation time 44.525 s.
- Extraction → transport at 52.167 s.
- The final frame shows the lifted pallet approaching the green ring.

At 84 s, a NumPy `int64` in a tracking sample could not be serialized by JSON.
The same issue prevented saving `result.json`; Isaac then segfaulted during
interpreter finalization. The encoder finalized a readable MP4. No successful
lowering/delivery claim is made. Preserve the log and fix serialization before
the deferred main campaign; do not discard this failed attempt.

The first camera attempt (`preview01`, cancelled) was obstructed by the 9 m
warehouse ceiling. The delivered preview uses a fixed camera at 6 m and wider
focal length. Physics uses 120 Hz; video timestamps refer to simulation time.
Steering is a synthetic 20 Nm / stiffness 1000 / damping 100 setting, changed
from the provisional 4 Nm after a separate steering response probe. No hardware
actuation or calibration is claimed. Pose feedback is simulator ground truth.

Earlier relevant planning/control tests passed, including targeted fixes for
steered tire width, actuator-lag braking and curved cusps. The complete suite was
not rerun after these fixes. An earlier full run had 736 passes, 5 skips and 6
failures caused by missing optional MuJoCo; the dependency was subsequently
installed. Final full tests and multiple random physical runs remain deferred.
The review also identified missing forbidden-contact/pocket-clearance checks
for insertion, which remain open. Actual pallet pose collision checks and a
0.36 m steered tire half-width were added before this recording.
