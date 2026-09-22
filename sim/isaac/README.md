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
  --pallet-geometry config/pallet_geometry_epal6.yaml \
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
# `sim/isaac/`

Isaac Sim work for this project. [ADR 0004](../../docs/decisions/0004-simulation-engine-and-insertion-depth.md)
makes Isaac the designated engine for new dynamic simulation (M4–M6); Gazebo is
frozen rather than deleted, and `tools/scene_rig.py` and MuJoCo stay as CPU
fixtures. Nothing in this directory is an approved milestone baseline yet.

The `bench03` mission code came in with PR #1 (`a3eb2c1`, 2026-09-17), so it can
be reviewed and its CPU tests run from a clone. **D2 is not finished.** Still
outstanding: `deploy/isaac/`, the procedure for rebuilding the external base
scene this directory expects, and the manifest D2 asks for -- interpreter path,
extension cache, asset revision, driver and GPU SKU, and hashes of every
transitively imported source (`run_transport.py` currently hashes the three
adapters plus `control/` and `planning/`, which leaves out `forklift_core`'s own
`geometry.py` and `_validation.py`). There is also **no negative-scene capture
path**: the transport run writes an overhead RGBA video, not an RGB-D/TF dataset
in the `scene_dataset` v1 format that ADR 0004 D5 and D6 need.

## What is here

| 파일 | 하는 일 |
|---|---|
| `determinism_probe.py` | 같은 장면의 깊이 영상이 프로세스를 새로 띄워도 비트 단위로 같은지 잰다 |
| `run_transport.py` | Hybrid A* 운반 임무를 Isaac 에서 실행하고 기록한다 (PR #1) |
| `scene.py` | 운반 장면을 구성한다 — 외부 base scene 을 전제로 한다 (PR #1) |
| `insertion_geometry.py` | 포크·팔레트 박스의 **순간** 여유를 기하로 검사한다. PhysX 접촉력도, 갱신 사이의 연속 비접촉도 증명하지 않는다 (PR #1) |
| `perception_camera.py` | 물리를 멈춘 채 같은 render product 에서 RGB 와 광축 Z 깊이를 읽는다 (PR #2) |
| `run_perception_approach.py` | 카메라 추정만으로 정렬 지점까지 주행하고 멈춘다 (PR #2) |
| `approach_evaluation.py` | 주행이 끝난 뒤에만 정답과 비교해 정렬·검출 오차를 채점한다 (PR #2) |
| `run_pocket_insertion.py` | 10Hz 연속 깊이로 삽입하고, 관측을 잃으면 멈춘다 (PR #2) |
| `pocket_tracking_handoff.py` | 직접 포켓 검출과 윗판 추정이 3회 일치해야 추적을 인계한다 (PR #2) |

## 실행

```bash
/opt/isaacsim/python.sh sim/isaac/determinism_probe.py --out /tmp/det --tag runA
/opt/isaacsim/python.sh sim/isaac/determinism_probe.py --out /tmp/det --tag runB
# 비교: 두 JSON 의 captures[].sha256 이 같은가
```

⚠️ **인터프리터 경로가 실험의 일부다.** 원격에 Isaac 설치가 두 벌 있고 `VERSION`
문자열이 `5.1.0-rc.19+release.26219.9c81211b.gl` 로 **똑같은데 한쪽만 돈다** —
`/opt/isaacsim` 은 실행되고 `~/isaacsim` 은 `isaacsim.core.api` import 에서
`typing_extensions` 없음으로 죽는다. 결과를 적을 때 경로를 같이 적는다.

## 2026-09-17 측정

원격 `kang-MS-7D77`, RTX 5070 12227 MiB, driver 580.126.09, `/opt/isaacsim` 5.1.0-rc.19.

| | waited | sha256 (앞 24) | finite | sum |
|---|---|---|---|---|
| runA first | 2 | `36e768b2b5ea2dba2329c522` | 135360 | 780902.875 |
| runA second | 0 | 같음 | 135360 | 780902.875 |
| runB first | 2 | 같음 | 135360 | 780902.875 |
| runB second | 0 | 같음 | 135360 | 780902.875 |

**실행 안에서도, 프로세스를 새로 띄워도 비트 단위로 같았다.** 채워지기까지 걸린
스텝 수까지 두 실행이 일치했다.

### 이 결과가 덮지 않는 것

같은 기계·같은 GPU·같은 드라이버·같은 빌드에서, **정지한 장면**을 기본 렌더러로
찍은 결과다. 다음은 **다시 재야 한다.**

- **기계 간 재현성** — 개발 PC 는 RTX 5070 Ti, 원격은 5070 이다. 둘이 갈리면
  누가 돌렸는지에 따라 수치가 달라진다. **팀에 제일 중요한 미측정 항목이다.**
- 움직이는·물리 구동 장면. NVIDIA 문서가 물리 재개의 비결정성을 명시한다.
- `PathTracing` 렌더러.
- `SingleViewDepthSensor` 같은 **잡음 경로** — 자체 RNG 가 들어간다.

### 왜 쟀는가

`tools/scene_rig.py` 를 남길 근거 중 하나가 "순수 numpy 라야 비트 재현이 된다"
였는데 **이 측정이 그것을 반증했다.** Isaac 도 재현한다. `scene_rig` 를 남길
근거로 남은 것은 ① GPU·EULA 없이 도는 CPU CI ② 기존 ADR 0003 수치가 그 리그의
양자화·좌표 모델 위에 있다는 이행 비용, 둘뿐이다.
