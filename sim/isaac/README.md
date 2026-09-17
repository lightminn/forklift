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

Cruise speeds are explicit YAML settings: approach 0.35 m/s, transport 0.30 m/s,
extraction 0.18 m/s and withdrawal 0.12 m/s. Insertion remains 0.055 m/s.
Braking and measured steering response can reduce the commanded speed.
The first baseline used approach 0.14 m/s and transport 0.12 m/s.

The user resumed the physical test bench after the preview. Retained records and
limits are described in the dated validation notes; this remains a synthetic
controller experiment, not hardware acceptance.
