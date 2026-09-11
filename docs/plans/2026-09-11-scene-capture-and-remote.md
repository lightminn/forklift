# M1-b 3단계: 장면 캡처·원격 `scenes` mode·세트 manifest 구현 계획

> **실행자:** 구현은 Codex에 **두 번 나눠** 위임한다(A: Task 1–3 캡처 계층, B: Task 4–5 원격·병합 계층). 샌드박스 제약(`.git`·anaconda3·네트워크·Docker 쓰기 불가, 커밋·stage 금지)은 1·2단계와 같다. Docker 이미지 안 시험·원격 실행·시간 측정은 Claude가 수행한다. 상태: **v2 — 2026-09-11 Codex 검토 반영.**

**목표:** 카탈로그 장면마다 Gazebo에서 동기화된 RGB/depth/CameraInfo 1세트를 v1 데이터 세트 형식으로 캡처하는 노드와 runner, 이를 원격 Slurm에서 batch로 실행하는 `scenes` mode, batch 결과를 합쳐 100개 완비를 검증하는 세트 manifest 도구를 만든다. 로드맵 M1의 "100개 장면 데이터 생성"을 닫는다.

**아키텍처:** `forklift_ros/scene_files.py`(ROS 의존 없는 인코딩·선택 순수 함수) ← `forklift_ros/scene_capture.py`(rclpy 노드) ← `sim/gazebo/capture_scenes.py`(장면 루프·프로세스 관리; `run_sensor_smoke.py`의 `Children`·`wait_ready`·`wait_clock`을 spec 로더로 재사용) ← `tools/remote_model_job.py`/`submit_model_check.py`/`model_check.sbatch`의 `scenes` mode ← `tools/merge_scene_batches.py`(세트 manifest). 캡처 노드·runner는 `forklift_core`를 import하지 않는다(gazebo 이미지에 코어 미설치). 로더 계약 `docs/interfaces/scene-dataset.md`가 파일 형식의 정본이며 캡처는 그 형식을 **생산**한다.

**기술 스택:** ROS 2 Jazzy(rclpy, sensor_msgs, rosgraph_msgs, tf2_msgs), Gazebo Harmonic, Pillow(gazebo 이미지의 `python3-pil`; 개발 이미지에는 없음), numpy, PyYAML, Slurm/Docker, pytest.

**Spec:** 설계 §7·§8·§10. 2단계 산출 `build_scene_world.py`의 `scene.yaml`(항목 + `catalogue_version` + `camera`; 최상위 provenance 없음)을 입력으로 쓴다.

## 전역 제약

- 기준: `main` `d80d94f`. 브랜치 `feat/scene-capture`(생성됨).
- 저장 형식은 `docs/interfaces/scene-dataset.md` v1 그대로. `stamp_ns = sec × 1_000_000_000 + nanosec`(정수 연산; bag 저장 시각이 아니라 **header stamp**). `scene.json`: `scene_id`, `catalogue_version`, `category`, `split`, `stamp_ns`, `clock_domain: "ros_sim"`, `source_provenance: "synthetic"` + `camera`, `image_id`, `source_snapshot_sha256`, `run_id`, `visibility`, `wall_times_s`. `ground_truth.json`: 카탈로그 GT 전체 사본에서 `stamp_ns`·`clock_domain`만 치환(`source_provenance: synthetic_ground_truth` 유지). `tf.json`: **header.frame_id=`base_link`, child_frame_id=`camera_optical_frame`인 실제 수신 변환**을 선택(`origin: received_tf_static`; static TF의 stamp 0은 영상 stamp와 일치를 요구하지 않음).
- Depth: 수신 encoding `32FC1`(little-endian, step = width×4)만 허용, RGB는 `rgb8`(step = width×3)만 허용. step·data 길이·endianness·frame_id를 검사한다. `depth_to_millimetre_png_array`: float32/64 (H,W) → NaN/±Inf/≤0 → 0(unknown), 유한 양수는 `round(v×1000)`, **유한 양수 중** 65.535 m 초과는 `ValueError`(float32 `65.535` 표현 오차를 피하려면 float64로 올려 비교). 전부 unknown인 영상도 저장 가능해야 한다(preview는 검정).
- 캡처 순서: 노드 구독 준비(`ready.json`) → bridge → Gazebo → 실제 `/clock` 수신 → static TF → simulation stamp ≥ 2 s의 **같은 header stamp RGB·depth·CameraInfo** 첫 세트 저장. 검정(전부 0) RGB는 건너뛴다. TF가 늦게 와도 저장 조건을 재평가한다. 벽시계 진행·timeout은 `time.monotonic()` 기준이며 **장면 전체 deadline의 잔여 시간**을 각 대기에 전달한다(ready 30 + clock 30 + capture 120을 더하지 않는다). JSON은 임시 파일 후 교체로 쓴다. stamp 버퍼는 최근 50개로 제한한다. 장면별 프로세스 정리는 `finally`에서 한다.
- colcon build/test는 batch당 1회, 설치 setup을 source한 env를 모든 자식에 전달하고 새 모듈의 설치 경로 import를 확인한다. `.runtime/install`은 장면 간 재사용(원격 wrapper가 종료 시 `.runtime`을 삭제하므로 batch 간 캐시는 없다).
- **실패 정책(Codex 검토 반영):** runner는 장면 실패 시에도 manifest를 갱신하고 계속 진행한 뒤 nonzero로 끝난다. 기존 `collect`는 실패 batch를 회수하지 않으므로, **실패 batch 전체를 새 실행 ID로 재제출**하고 원래 batch는 병합에서 제외한다. 실패 ID만 재제출하지 않는다(부분 회수 규약이 없음).
- **시간 제한:** 현재 `model_check.sbatch`는 `--time=00:20:00`이다. 장면당 120 s × 25 = 50 min이므로 `scenes` mode는 제출 시 `sbatch --time`을 mode별로 넘긴다(scenes 기본 `01:30:00`, 기존 mode는 변경 없음). batch 크기와 시간은 2장면 spike 결과로 확정한다.
- `--source-sha256`은 원격 runner가 `verify_source()`로 얻은 snapshot digest, `--image-id`는 inspect한 immutable ID, `--run-id`는 제출 시 run ID. runner → node → scene.json/batch manifest까지 전달한다.
- 기존 `run_sensor_smoke.py`·`sensor_validator.py`·gazebo mode 동작과 시험은 바꾸지 않는다. 원격 실행은 새 snapshot·새 실행 ID. 기존 시험 422개 보존, Ruff 통과.

## 파일 구조

| 파일 | 책임 |
|---|---|
| `ros2/src/forklift_ros/forklift_ros/scene_files.py` | 순수 함수: `stamp_to_ns(stamp) -> int`, `depth_to_millimetre_png_array(depth_m) -> uint16`, `image_to_array(msg_like, encoding) -> ndarray`(step·길이·endianness·encoding 검사), `camera_info_to_json(msg_like) -> dict`, `tf_static_to_json(transform_like) -> dict`, `select_synchronized_set(buffers, warmup_ns) -> stamp_ns | None`(rgb/depth/info 모두 있고 검정 아님), `write_scene_files(scene_dir, *, rgb, depth_m, camera_info, tf, ground_truth, scene) -> dict[str, str]`(파일 7개 + `depth_preview.png`, 파일별 SHA-256) |
| `ros2/src/forklift_ros/forklift_ros/scene_capture.py` | rclpy 노드 console script `scene_capture --output <dir> --entry <scene.yaml> --image-id <id> --source-sha256 <hex> --run-id <id> --warmup-s 2 --deadline-s <n>` |
| `ros2/src/forklift_ros/test/test_scene_files.py` | 인코딩·선택·JSON·저장 시험. **import는 기존 시험처럼 `importlib.util.spec_from_file_location`으로 파일을 직접 로드**(`from forklift_ros import …`는 호스트 루트 실행에서 해결되지 않음) |
| `ros2/src/forklift_ros/setup.py` | `scene_capture` entry point |
| `sim/gazebo/capture_scenes.py` | `--catalogue --scenes sNNN-sMMM --output --image-id --source-sha256 --run-id [--scene-deadline-s 120]`; 장면 루프, 장면마다 갱신되는 `manifest.json`, exit 0은 실패 0일 때만 |
| `tests/simulation/test_capture_scenes.py` | 범위 파싱(순서·형식·카탈로그 ID 존재), manifest 조립·실패 집계, 시작 순서·자식 조기 종료·timeout·정리(fake process/clock) |
| `tests/integration/test_scene_roundtrip.py` | **생산자 → 로더 왕복**: `write_scene_files`로 유효한 640×480 장면을 쓰고 `load_scene_sample()`이 읽어 GT·stamp·intrinsics가 맞는지 |
| `tools/remote_model_job.py`, `tools/submit_model_check.py`, `deploy/slurm/model_check.sbatch` | `scenes` mode(`--catalogue`, `--scene-range`), sbatch 인자 6 또는 8개(`${7:--}`, `${8:--}`), mode별 `--time` |
| `tests/integration/test_remote_model_jobs.py` | scenes argv·검증·sbatch 인자·`--time`·dry-run 시험 추가, gazebo 기존 시험 불변 |
| `tools/merge_scene_batches.py` | `--catalogue --batches <dir>... --output <set dir>`: 복사·해시·완비·정합 검증, 세트 `manifest.json`, 기존 출력 덮어쓰기 거부 |
| `tests/integration/test_merge_scene_batches.py` | 합성 batch(3+2 장면, ID 매개변수화된 fixture) 병합·거부 시험 |
| `docs/development.md`, `sim/gazebo/README.md`, `docs/interfaces/scene-dataset.md` | scenes mode 명령·batch 정책·병합, 캡처 절차, 생산자 절 |

---

## 위임 A: Task 1–3 (캡처 계층)

### Task 1: `scene_files.py` (ROS 의존 없음)

- [ ] **Step 1 (RED):** `ros2/src/forklift_ros/test/test_scene_files.py`. 파일 로드는 `test_observation.py`와 같은 spec 방식. 시험 목록(각 1규칙 1시험): `stamp_to_ns(sec=2, nanosec=400_000_000) == 2_400_000_000`(float 경유 없음, `sec*10**9+nanosec`), depth 반올림·NaN/±Inf/≤0 → 0, 전부 unknown 영상 저장, 유한 양수 65.536 → `ValueError`이고 `+inf`는 실패가 아님, float32 65.535 정확 비교, 잘못된 dtype·1차원 거부, `image_to_array`의 encoding 불일치·step 불일치·data 길이 불일치·big-endian 거부, `camera_info_to_json` 키 12개와 roi 5키(`json.dumps(allow_nan=False)` 가능), `tf_static_to_json`이 frame 쌍이 다른 변환을 거부, `select_synchronized_set`이 warmup 전 stamp·검정 RGB·불완전 세트를 건너뛰고 첫 완전 세트를 고르는지, `write_scene_files`가 파일 8개와 해시를 돌려주고 `depth_mm.png`가 **PNG IHDR 16-bit grayscale**이며 픽셀값이 mm인지(`image.mode` 단정은 쓰지 않음; 로더가 `I` 모드도 허용하므로).
- [ ] **Step 2:** 호스트에서 `python -m pytest ros2/src/forklift_ros/test/test_scene_files.py -q -p no:cacheprovider -W error` 실패 확인(Pillow는 conda base에 있음).
- [ ] **Step 3:** 구현. `write_scene_files`는 `Image.fromarray(uint16)`(mode 인자 금지)로 저장하고 `depth_preview.png`는 0–5 m를 8-bit로 사상(unknown 검정).
- [ ] **Step 4 (GREEN):** 통과.

### Task 2: 캡처 노드 `scene_capture.py`

- 구독 QoS는 `sensor_validator.py`와 동일(영상·info·clock RELIABLE depth 50, `/tf_static` RELIABLE+TRANSIENT_LOCAL). `use_sim_time=True`. 구독 생성 직후 `ready.json`, 1 s마다 `progress.json`(`counts` 딕셔너리에 `clock` 키 포함 — `wait_clock` 재사용 조건).
- 수신 세트를 stamp_ns 키로 버퍼(최근 50개). `select_synchronized_set` 조건 + TF 수신이 충족되면 `--entry`의 scene.yaml에서 GT·scene_id·category·split·visibility·camera·catalogue_version을 읽어 파일을 쓰고 `result.json`(`passed`, stamp, counts, wall_times)과 함께 exit 0. `--deadline-s` 초과·저장 실패·encoding 위반은 `result.json`에 원인을 쓰고 exit 1(저장 실패 → nonzero 시험을 Task 1의 순수 함수 수준에서 `write_scene_files` 예외로 검증).
- 노드 로직 중 ROS 없이 시험 가능한 부분(세트 선택·메타 조립 `build_scene_json(entry, stamp_ns, image_id, source_sha, run_id, wall_times)`·GT 치환 `ground_truth_for_capture(entry, stamp_ns)`)은 `scene_files.py`에 두고 Task 1 시험에 포함한다.

### Task 3: runner `capture_scenes.py`

- `parse_scene_range(text, catalogue_ids) -> list[str]`: `^s\d{3}-s\d{3}$`, 시작 ≤ 끝, 모든 ID가 카탈로그에 존재해야 함(아니면 `ValueError`).
- 출력 `/output/scene_capture/`(fresh), runtime `/output/.runtime`(smoke와 같은 env 구성). colcon build → colcon test(`--python-testing pytest`) → 설치 경로 import 확인(`forklift_ros.scene_files`) 1회.
- 장면마다(전체 deadline `--scene-deadline-s`): `build_scene_world.py --catalogue --scene --output <scene_dir>/world` → `scene_capture`(ready 대기) → bridge → `gz sim -s -r --headless-rendering` → `wait_clock(progress)` → `synthetic_tf --config <scene_dir>/world/transforms.yaml` → 노드 종료 대기 → `finally`에서 gazebo/bridge/tf 정지 → 장면 결과 기록 후 **manifest 즉시 갱신**.
- `manifest.json`: `catalogue_version`, `catalogue_sha256`(입력 파일 해시), `camera`, `image_id`, `source_snapshot_sha256`, `run_id`, `requested_scenes`, `scenes: {id: {passed, error, files: {name: sha256}, wall_times_s: {build_world, ready, first_clock, captured, stopped}}}`, `failed_count`.
- 시험 `tests/simulation/test_capture_scenes.py`: 범위 파싱 정상/역순/형식/미존재 ID, `build_manifest`·`failed_count`·exit 판정, fake `Children`/fake clock으로 시작 순서(ready 전 Gazebo 시작 금지), 자식 조기 종료·timeout 시 실패 기록과 정리, 정리 실패가 결과에 반영됨. 실제 `Children` 내부는 재시험하지 않는다.
- 통합: `tests/integration/test_scene_roundtrip.py`(생산자 → `load_scene_sample`).

**위임 A 완료 확인:** 호스트 회귀(기존 422 + 신규), Ruff, 직접 실행 `python3 sim/gazebo/capture_scenes.py --help`. Claude: gazebo 이미지에서 `python3 -m pytest ros2/src/forklift_ros/test`(scene_files 포함, Pillow 있음)와 colcon test.

## 위임 B: Task 4–5 (원격·병합 계층)

### Task 4: 원격 `scenes` mode

- `SubmitRequest`에 `catalogue: str | None = None`, `scene_range: str | None = None`, `time_limit: str | None = None`(기본 mode별: scenes `01:30:00`, 그 외 None → sbatch 스크립트 기본). CLI `--catalogue`, `--scene-range`, `--time-limit`.
- `_validate_request`: scenes는 `image`·`catalogue`·`scene_range` 필수. `catalogue`는 정규화된 상대 POSIX 경로(`..`·절대·숨김 금지)이고 **`discover_snapshot(source).files`에 실제 포함**돼야 한다(`_is_allowed`만으로는 존재하지 않는 경로도 통과). `scene_range`는 형식·순서·카탈로그 ID 존재(로컬 YAML 읽어 확인). dry-run·제출 기록 JSON에 세 값 포함.
- sbatch argv: `sbatch --parsable [--time <limit>] <script> MODE SOURCE OUTPUT PYTHON IMAGE DURATION [CATALOGUE RANGE]`. `model_check.sbatch`는 `${7:--}`, `${8:--}`로 선택 인자를 읽고 6 또는 8개만 허용. 기존 mode 6개 인자 유지.
- `remote_model_job.py`: parser `--catalogue`, `--scene-range`, `--run-id`; `build_command("scenes")`는 gazebo와 같은 docker argv에 명령 `python3 sim/gazebo/capture_scenes.py --catalogue /workspace/<catalogue> --scenes <range> --output /output --image-id <immutable> --source-sha256 <verify_source digest> --run-id <run id>`; `execute_job`는 `mode in {"gazebo", "scenes"}`에서 이미지 ID 고정·`_ContainerSignalCleanup`; 원격에서도 catalogue 경로가 검증된 manifest 파일 집합에 있는지 확인.
- 시험: scenes argv(catalogue·range·image ID·source sha·run id), catalogue 미포함/`..`/미존재 거부, range 오류 거부, sbatch argv 6/8 + `--time`, dry-run 출력, gazebo 기존 시험 불변. `model_check.sbatch`를 `bash -n`과 함께 8인자 전달 시험(스크립트를 실제로 실행하되 `remote_model_job.py`를 stub으로 대체).

### Task 5: 세트 manifest `tools/merge_scene_batches.py`

- 입력: 카탈로그, batch 회수 디렉터리들(`scene_capture/manifest.json` + `scene_capture/scenes/sNNN/`). 출력 디렉터리는 **존재하면 거부**.
- 검증: 모든 batch의 `catalogue_version`·`catalogue_sha256`(입력 카탈로그 파일의 실제 해시와도 대조)·`camera`·`image_id`·`source_snapshot_sha256` 동일; `failed_count == 0`이고 장면별 `passed`; 요청 ID·결과 키·디렉터리명·`scene.json.scene_id` 일치; 요청 ID 합집합 == 카탈로그 ID 집합(중복·누락 0); 파일 해시 재계산 == batch manifest; 장면별 `category`/`split`/`catalogue_version`/`camera`/`image_id`/`source_snapshot_sha256`이 카탈로그·batch와 일치; `ground_truth.json`이 카탈로그 GT에서 `stamp_ns`·`clock_domain`만 치환한 값과 동일; 양성·가림 GT `status == "valid"`, 음성 두 범주 `status == "no_pallet"`; 각 장면 `load_scene_sample` 성공.
- 세트 `manifest.json`: 메타 + `scenes: {id: {batch_run_id, files}}` + 범주·split 집계.
- 시험(`tests/integration/test_merge_scene_batches.py`): ID를 매개변수화한 fixture로 batch 3+2 병합 성공; 중복 ID·누락·해시 불일치·image_id 불일치·GT 불일치·status 불일치·실패 batch·기존 출력 존재 각각 거부.

### Task 6: 문서

- `docs/development.md`: scenes mode 명령(`--catalogue`, `--scene-range`, `--time-limit`), batch 분할, 실패 batch 전체 재제출 정책, 병합 명령. `sim/gazebo/README.md`: 캡처 절차·산출물. `docs/interfaces/scene-dataset.md`: 생산자 절.

## Task 7: 검증·실행 (Claude)

- [ ] gazebo 이미지에서 ROS 패키지 시험(scene_files 포함)·colcon. 호스트 회귀·Ruff.
- [ ] spike: `--mode scenes --scene-range s001-s002 --wait`. `manifest.json`의 `wall_times_s`로 장면당 시간을 재고 batch 크기·`--time-limit`을 확정. 같은 2장면을 한 번 더 캡처해 `depth_mm.png` 해시 동일 여부 기록(결정론 관찰).
- [ ] 100개: 25개씩 4 batch(spike 결과에 따라 조정) 제출·회수. 실패 batch는 전체 재제출.
- [ ] `merge_scene_batches.py`로 `data/synthetic_scenes/catalogue_v1/` 생성, 로더로 100개 읽어 status 분포(valid 80, no_pallet 20) 확인, RGB·preview 몇 장 직접 확인.
- [ ] 검증 기록 `docs/validation/2026-09-1x-scene-dataset-v1.md`, 체크포인트 갱신, 커밋·병합·push·원격 repo 갱신. 데이터는 Git 밖(로컬 `data/`, 원격 `data/`).

## 자체 검토

- Codex 검토 반영: header stamp 정수 계산, scene.json provenance 명시, 왕복 통합시험, depth 경계(+Inf 비실패·전부 unknown·dtype), 메시지 step/길이/endianness 검사, TF 선택 기준, monotonic deadline 잔여 전달, 임시파일 JSON, 버퍼 제한, 늦은 TF 재평가, finally 정리, `wait_clock` counts 형식, sbatch `--time`·`${7:--}`, catalogue 실제 포함 검증, 범위 ID 존재 검사, 실패 batch 전체 재제출, source/image/run id 전달 경로, manifest 정합 검증 항목, status 명시, `batch_run_id` 공급, 시험 import 방식, gazebo 이미지에서 ROS 시험, 두 번 위임.
- 미포함(의도): 부분 회수, LiDAR, 시퀀스, GPU, 320×240 비교.
