# 포켓 관측 계약과 합성 평가 장면 세트 (M1-b) 설계

작성일: 2026-09-11. 상태: **v2 승인(2026-09-11).** Codex 교차검증을 반영한 뒤 사용자가 §12의 결정 5개를 권장안대로 승인했다(640×480·69°, 조건 비율·범위, 70/30, 장면당 1세트, 물리 폭 기준 가림). 구현은 §11의 계획 3개로 진행한다. 로드맵 M1 후반부와 M2 진입 조건([로드맵](../plans/2026-09-11-development-roadmap.md) §2·§3 M1–M3·§6·§8)을 구현 가능한 계약과 데이터 세트로 구체화한다. 실물 센서·팔레트 인식 알고리즘·주행은 범위가 아니다.

## 1. 목표와 완료 조건

- **관측 계약:** 팔레트 두 포켓의 `base_link` 기준 위치·삽입 방향·개구 치수·상태·불확실성·시각·clock domain·출처를 표현하는 코어 타입과 문서. M2 인식기의 출력, M3 추적기의 입력, 평가기의 정답이 같은 기하 타입을 쓴다.
- **평가 장면 세트:** 양성·가림·음성을 포함한 **독립 합성 장면 100개**를 seed·장면 ID와 함께 저장소에 고정하고, 각 장면의 RGB/depth/CameraInfo/TF와 정답을 원격 Gazebo에서 생성해 개발/평가로 분리한다.
- **완료 조건(로드맵 M1):** 카탈로그 100개 고정, 원격 일괄 생성으로 100개 전부의 관측 파일·정답·해시 manifest 완비, 코어 로더가 각 장면을 기존 `forklift_core` 타입으로 읽음. 기존 고정 reference experiment(`scene_config.yaml`, `run_sensor_smoke.py`)의 검사는 완화하지 않는다.
- **이 계약이 보장하지 않는 것:** M5 삽입 여유 계산에는 삽입 깊이별 통로 치수·실제 포크·처짐·보정 오차가 추가로 필요하다. 이 타입은 M2 평가와 M3 입력까지만 충분하다고 본다.

## 2. 현재 상태와 제약

- `sim/gazebo/build_sensor_world.py`는 단일 reference experiment이며 검증기가 설정 변경을 거부한다. 팔레트 0.6(x)×0.8(y)×0.30(z) m, 덱 두께 0.05, 지지대 3개(폭 0.1, y=−0.35/0/0.35) → 개구부 두 개 폭 0.25·높이 0.20, 중심 y=±0.175. 카메라 320×240·HFOV 90°·5 Hz, base 기준 (0.75, 0, 0.5) m, optical 회전 고정. 조명·장애물·음성 조건 없음.
- 잠정 포크: 간격 0.29·폭 0.055·두께 0.024 m(추정값). 실물 팔레트·포크 치수는 미확인이며 아래 범위는 **설계 가정**이다.
- 원격 실행은 `tools/submit_model_check.py`의 `gazebo` mode(읽기 전용 snapshot, Docker `forklift/gazebo:jazzy-harmonic`, CPU llvmpipe). `deploy/slurm/model_check.sbatch`는 현재 인자 6개 고정이다.
- snapshot 허용 목록은 `tests/`에서 `.py`만 보낸다 → 바이너리 fixture는 원격에 가지 않는다.
- Gazebo RGB-D 센서는 영상 구독자가 없으면 렌더를 건너뛴다. 기존 runner도 validator 구독 후 Gazebo를 시작한다.
- D435i 공식 사양: depth FOV 87°×58°, RGB FOV 69°×42°. 아래 카메라 설정은 이를 흉내 낸 **합성 설정**이지 보정값이 아니다.
- CONTRIBUTING §5: SI, base x전방/y좌측/z위, optical x우/y아래/z전방, 시각+clock domain, `_ns`/`_s`, NaN=unknown, 잘못된 입력은 예외·정상 소실은 상태. §7: 기대값을 시험 대상 구현으로 재계산하지 않는다. §8: 데이터는 Git 밖, manifest·출처·해시 기록.

## 3. 대안 비교

| 접근 | 판단 |
|---|---|
| A. reference experiment 생성기를 파라미터화해 장면마다 30초 bag 기록 | 기각. 장면당 30초+초기화, bag 100개 수 GB. 정적 장면에 시간 축 불필요 |
| **B. 별도 생성 경로: 카탈로그(YAML) → 장면별 SDF → 짧은 캡처(warmup 후 동기화 1세트) → 파일 데이터 세트** | **채택.** 정답은 카탈로그 기하에서 해석적으로 계산해 렌더러와 독립. 기존 smoke 경로 유지 |
| C. Gazebo 없이 직접 투영으로 depth 합성 | 기각(평가 입력용). 단위시험 fixture 생성에는 사용 |

## 4. 관측 계약 — `src/forklift_core/perception/pocket_observation.py`

**가정(명시):** 팔레트는 바닥에 수평으로 놓이고 두 개구부는 수직 직사각형이며 같은 삽입축을 공유한다. 기울어진 팔레트 지원은 이후 자세 필드를 추가할 때 다룬다.

```python
@dataclass(frozen=True)
class Pocket:
    center_m: tuple[float, float, float]  # 개구부 중심(전면 기준면 위), 관측 frame
    width_m: float  # 개구부 자체의 폭(수평)
    height_m: float  # 개구부 자체의 높이(수직)


@dataclass(frozen=True)
class PocketObservation:
    stamp_ns: int  # 촬영/측정 시각, 0 이상 정수
    clock_domain: str  # "ros_sim" | "ros_system" | "device" | "synthetic"
    frame_id: str  # 이 단계에서는 "base_link"만 허용
    source_provenance: str  # "synthetic" | "replay" | "live" | "synthetic_ground_truth"
    status: str  # "valid" | "no_pallet" | "invalid"
    left: Pocket | None  # 접근 방향 기준 왼쪽(frame +y 쪽). valid일 때만
    right: Pocket | None
    insertion_yaw_rad: (
        float | None
    )  # 삽입축 a=(cos ψ, sin ψ, 0)의 ψ, (−π, π]. 전면 바깥 법선은 −a
    position_sigma_m: (
        float | None
    )  # 두 중심에 공통으로 적용하는 등방 1σ 상한. None=미상(0으로 취급 금지)
    yaw_sigma_rad: float | None
    reason: str | None  # status != "valid"일 때 필수
```

- **좌우 정의:** 삽입축 a와 왼쪽축 ℓ=(−sin ψ, cos ψ, 0)에 대해 `left`는 ℓ·(center_left − center_right) > 0인 쪽이다. 검증은 이 내적으로 하며 단순 y 비교를 쓰지 않는다(yaw가 커도 성립).
- **방향 정본:** `insertion_yaw_rad` 하나만 저장한다. 포켓별 축 벡터·정면 법선은 파생값이며 저장하지 않는다.
- **검증(`ValueError`):** 좌표·치수·yaw 유한, 폭·높이 > 0, σ ≥ 0 또는 None, `stamp_ns` 정수 ≥ 0, `clock_domain`/`status`/`source_provenance`는 허용 집합, `frame_id == "base_link"`, `status == "valid"`면 left/right/yaw 필수·`reason` None, 아니면 left/right/yaw/σ 모두 None·`reason` 필수, 두 중심 간 거리가 (0.05, 2.0) m 안이고 개구부가 겹치지 않음: `ℓ·(left.center − right.center) > (left.width_m + right.width_m)/2` (2026-09-11 Codex 검토로 정정; 이전 "두 폭의 합보다 큼"은 중심 간격 0.10+w < 2w라 승인 범위 전체를 거부하는 모순이었다).
- **frame 정책:** 공개 타입은 `base_link` 출력으로 제한한다. optical 기준 점·벡터는 `RigidTransform.apply`(점)와 회전만 적용하는 방향 변환으로 먼저 base로 옮긴 뒤 관측을 구성한다. 별도 `transform_observation` 헬퍼는 만들지 않는다(YAGNI).
- **정답 σ:** 합성 정답은 `position_sigma_m = 0.0`, `yaw_sigma_rad = 0.0`이며 이는 모델 기하가 정확하다는 뜻이지 렌더·양자화 오차가 없다는 뜻이 아니다.
- **평가 규약(M2가 사용):** 포켓별 위치 오차 ‖ĉ − c‖(좌/우 각각), yaw 오차는 (−π, π]로 wrap한 차이, 장면별로 두 포켓 중 최대를 그 장면의 위치 오차로 집계. 검출률 분모는 `positive`와 `occluded`를 분리하고 음성은 오검출률로 센다. 인식기가 `invalid`를 내면 실패 표본으로 별도 집계한다.
- **JSON:** `to_json()`/`from_json()`은 필드명 그대로, NaN 금지. `docs/interfaces/pocket-observation.md`에 의미·단위·좌표계·검증 규칙·예시.

## 5. 장면 카탈로그 — `tools/generate_scene_catalogue.py` → `sim/gazebo/scenes/catalogue_v1.yaml`

결정론적 생성(`--seed`, `--count 100`)으로 만든 YAML을 **저장소에 고정**한다. 재생성은 새 버전 파일로만 한다.

### 5.1 팔레트 기하(정답과 SDF의 공통 정의)

팔레트 원점 C = 바닥 위 footprint 중심(z=0). 삽입축 a=(cos ψ, sin ψ, 0), 왼쪽축 ℓ=(−sin ψ, cos ψ, 0), 위축 u=(0,0,1). 외형은 깊이 0.6(a 방향)·폭 0.8(ℓ 방향)·높이 0.30, 덱 두께 0.05로 고정한다. 따라서 **개구 높이는 0.20으로 고정**(0.30 − 2×0.05)하고 개구 폭 w만 변화시킨다. 중앙 지지대 폭 s_c = 0.10 고정, 바깥 지지대 폭 s_o = (0.8 − 2w − 0.1)/2, 유효 범위 w ∈ [0.20, 0.28] (s_o ∈ [0.07, 0.15]).

- 개구부 중심: `C − 0.3·a ± d·ℓ + 0.15·u`, d = 0.05 + w/2 (∈ [0.15, 0.19]; 포크 y=±0.145는 폭 0.055로 항상 개구 안).
- `insertion_yaw_rad = ψ`, 전면 바깥 법선 −a, 전면 기준면은 C − 0.3·a를 지나는 ℓ–u 평면.
- SDF는 같은 정의로 덱·지지대 상자를 회전·배치한다. 정답 생성과 SDF 생성이 같은 회전 실수를 공유할 위험은 §9의 독립 기대값 시험으로 막는다.

### 5.2 항목

| 필드 | 내용 |
|---|---|
| `scene_id` | `s001`…`s100` |
| `split` | `dev` 70 / `eval` 30. seed 기반 셔플, 범주별 비율 유지. 장면은 서로 독립 표본(공유 기하 없음)이므로 누출 그룹은 두지 않는다 |
| `category` | `positive` 60 / `occluded` 20 / `negative_no_pallet` 10 / `negative_lookalike` 10 |
| `pallet` | 양성·가림만: `x_m`(C의 x)∈[2.0, 4.0], `y_m`∈[−1.0, 1.0], `yaw_rad`∈[−0.52, 0.52], `opening_width_m`∈[0.20, 0.28]. 두 음성 범주는 `pallet: null`(목표 팔레트 부재를 명시) |
| `lookalike` | `negative_lookalike`만: 개구부 없는 0.6×0.8×0.30 상자, 같은 위치 범위. 쉬운 음성 기준선이며 "잘못된 두 포켓 패턴"에 대한 강건성 주장은 아니다 |
| `occluder` | `occluded`만: 한쪽 포켓(좌/우 명시) 전면에서 a 방향으로 0.3–0.6 m 앞, 카메라보다 최소 0.3 m 앞(x ≥ 1.05 m)에 놓인 상자. `occluded_fraction_nominal` = 상자 폭 / 개구 폭 ∈ [0.2, 0.6]**(물리 폭 기준이며 영상 면적이 아님)**, 높이는 개구 높이 이상 |
| `distractors` | 프리셋 배치 목록(6종)에서 0–2개 선택. 팔레트·가림 상자와 겹치지 않음 |
| `lighting` | 방향 프리셋 3종 × diffuse 강도 {0.5, 0.9} |
| `surfaces` | 바닥·배경 색 프리셋 4종 |
| `visibility` | 생성기가 계산한 평가 메타데이터: 각 개구부 4모서리의 영상 투영 좌표, `left_in_view`/`right_in_view`(모서리 전부 영상 안·near clip 이상), `occluded_side` |
| `ground_truth` | `PocketObservation` JSON. `clock_domain: synthetic`, `stamp_ns: 0`, `source_provenance: synthetic_ground_truth`, σ=0. 음성은 `status: no_pallet`, `reason: "no target pallet in scene"`. **가림은 정답을 바꾸지 않는다**(물체 기하 보존; 가시성은 `visibility`에만) |

- **시야 거부 샘플링:** 양성·가림 장면은 두 개구부의 4모서리가 모두 영상 안(여백 4 px)이고 depth > near clip일 때만 채택한다. 카메라 높이 0.5 m·수직 반시야 약 27°(§6 설정)에서는 x ≥ 2.0 m가 필요하며 그 아래 근접 조건은 M3 시퀀스 범위로 미룬다.
- **정답 시각 의미:** 카탈로그의 GT는 시각이 없는 기하다. 캡처는 장면 GT 사본에 캡처 stamp(`ros_sim`)를 붙여 저장해 관측과 정답의 시각을 명시적으로 연결한다.

## 6. 장면 월드 생성 — `sim/gazebo/sdf_parts.py` + `sim/gazebo/build_scene_world.py`

- `build_sensor_world.py`의 헬퍼(`element`, `box`, `rotation_rpy`, `add_urdf_visuals`, 센서 rig·bridge·TF 출력)를 `sdf_parts.py`로 옮긴다. 기존 생성기는 SDF·bridge·TF 출력 **바이트 동일**과 고정 config 거부를 회귀 시험으로 보호하고, 직접 실행과 `spec_from_file_location` 시험 양쪽의 import를 확인한다.
- 새 생성기는 카탈로그 항목 하나를 받아 팔레트(회전·폭 반영)·유사물·가림 상자·distractor·조명·표면색·센서 rig를 넣은 SDF와 bridge/TF 설정을 만든다. LiDAR는 이 경로에 넣지 않는다(M1-b/M2 소비자 없음).
- **카메라 설정(사용자 결정 1):** 권장 640×480, HFOV 1.204 rad(69°, 정사각 픽셀이면 VFOV ≈ 54.6°) — D435i RGB 시야를 흉내 낸 합성 설정. 대안: 기존 320×240·90°. 어느 쪽이든 §8의 시간 측정 뒤 확정한다.

## 7. 캡처 — `forklift_ros/scene_capture.py` + `sim/gazebo/capture_scenes.py`

- 순서(장면마다): 캡처 노드 시작·구독 준비(`ready.json`) → Gazebo headless·bridge·static TF 시작 → simulation time ≥ 2 s warmup → **같은 header stamp의 RGB·depth·CameraInfo 1세트** 수신 확인 → 저장 → 프로세스 그룹 종료. 벽시계 timeout(장면당 120 s). colcon build/test는 batch당 한 번.
- 캡처 노드는 ROS·numpy·Pillow만 쓰고 `forklift_core`를 import하지 않는다(gazebo 이미지에 코어 설치 단계가 없음).
- 저장 형식(`scenes/s001/`):
  - `rgb.png` 8-bit RGB.
  - `depth_mm.png` 16-bit 단일 채널, 값 = round(depth_m × 1000)(최근접), 0 = unknown(NaN·±Inf·≤0), 65.535 m 초과는 캡처 실패로 거부. `depth_meta.json`: `{"unit": "mm", "meters_per_unit": 0.001, "unknown_value": 0, "kind": "optical_axis_z"}`. 시각화용 8-bit PNG는 별도 이름(`depth_preview.png`)이며 데이터가 아니다.
  - `camera_info.json`: 수신 메시지 값 그대로(K, D, R, P, width, height, binning, roi, frame_id). 반 픽셀 보정 없음.
  - `tf.json`: 실제 수신한 `/tf_static`에서 `target_frame: base_link`, `source_frame: camera_optical_frame`, `translation_m`, `quaternion_xyzw`, `origin: "received_tf_static"`.
  - `ground_truth.json`: 카탈로그 GT 사본에 `stamp_ns`(캡처 stamp)·`clock_domain: ros_sim`을 넣은 것.
  - `scene.json`: 카탈로그 항목·stamp·이미지 ID·소스 snapshot 해시·카메라 설정.
  - batch 최상위 `manifest.json`: 카탈로그 버전·센서 설정·소스/이미지 ID·요청 장면 ID·장면별 성공/실패(원인)·파일별 SHA-256.
- 실패 정책: 한 장면이라도 실패하면 runner는 manifest를 쓰고 nonzero로 끝난다. 기존 `collect`는 실패 batch를 회수하지 않으므로 실패 증거는 원격 `artifacts/`에서 별도로 읽고, 실패 장면 ID만 새 batch로 재제출한다.
- 결정론: 같은 장면을 두 번 캡처해 depth가 동일한지 보는 반복 캡처 실험을 §8에서 한 번 수행한다(렌더러·이미지 고정 조건에서의 관찰이며 일반 증명이 아니다).

## 8. 원격 실행 — `scenes` mode

- `submit --mode scenes --image <IMAGE> --catalogue sim/gazebo/scenes/catalogue_v1.yaml --scene-range s001-s025`. `SubmitRequest`·CLI·검증·dry-run·제출 기록·`model_check.sbatch`(현재 6인자 고정 → 가변 인자)·원격 parser까지 전달한다. 이미지 ID 고정과 컨테이너 신호 정리는 gazebo mode와 같은 분기에서 적용한다. 기존 gazebo argv·이미지 ID 시험은 유지하고 scenes 시험을 추가한다.
- **시간 측정 spike를 먼저 한다:** 2개 장면을 320×240과 640×480으로 각각 캡처해 시작·warmup·캡처·종료 시간과 메모리를 기록하고, 그 결과로 카메라 설정(§6)과 batch 크기(기본 25)를 확정한다. 장면당 10–15초는 현재 미검증 추정이다.
- 100개는 batch 4회로 나눠 제출한다. 세트 manifest(`data/synthetic_scenes/catalogue_v1/manifest.json`)는 4개 batch manifest를 합쳐 카탈로그 버전·센서 설정·소스/이미지 ID 동일성, 예상 ID 집합과의 일치(중복·누락·실패 0), 파일 해시 재계산 일치를 검증한다. 데이터는 Git 밖(로컬 `data/`, 원격 `data/`), manifest 해시와 실행 ID는 검증 기록에 남긴다.

## 9. 데이터 세트 로더 — `src/forklift_core/perception/scene_dataset.py`

- `SceneInput`: `rgb uint8[H,W,3]`, `depth_m float64[H,W]`(NaN=unknown), `intrinsics: PinholeIntrinsics`, `base_from_optical: RigidTransform`, `stamp_ns`, `clock_domain`, `rectified=True`, `pixel_frame`, `rgb_registered_to_depth_grid=True`. **인식기는 이 타입만 받는다.**
- `SceneSample = SceneInput + ground_truth: PocketObservation + catalogue entry`. **평가기만 소유**한다(정답 누출 방지).
- CameraInfo → `PinholeIntrinsics` 변환은 좁은 계약: `D` 전부 0, `R = I`, `P[:, :3] = K`이고 `P[:, 3] = 0`(단안, 기준선 없음), skew 0, binning 0/1, roi 없음. 벗어나면 `ValueError`(조용히 K만 뽑지 않음). 값은 그대로 보존한다.
- depth는 `meters_per_unit`을 적용해 m로 만들며, 이후 `deproject_depth_pixels(depth_m, …, meters_per_unit=1.0)`로 호출한다(이중 스케일 방지를 문서·시험에 명시).
- 시험 fixture는 파일이 아니라 시험 코드가 tmp 디렉터리에 생성한다(8×6 픽셀, 손으로 정한 값; snapshot 허용 목록이 `tests/`의 `.py`만 보내므로). 비중앙 픽셀의 역투영 기대값을 독립 계산으로 대조한다.
- 의존성: Pillow를 `dataset` extra로 추가하고 `dev` extra에도 포함한다. 로더 안에서 지연 import.

## 10. 시험 계획

- 코어: 관측 타입 검증(좌우 내적·yaw wrap·σ None·상태별 필수/금지 필드·frame 제한)·JSON 왕복·로더(생성 fixture, 좁은 CameraInfo 계약, 이중 스케일 방지). 회귀 146 유지.
- 카탈로그: 같은 seed → 바이트 동일; 개수·범주·split 비율; **yaw=0, +ψ, −ψ 세 장면을 의도적으로 골라 비영점 위치·다른 폭에서 정답을 독립 계산한 문자 그대로의 기대값과 대조**; 음성·치수 경계(w=0.20/0.28)·시야 밖 거부·가림 상자 위치(카메라 앞)·잘못된 설정 거부.
- SDF: 기존 생성기 바이트 동일 회귀, 장면 SDF의 팔레트 회전·지지대 폭·가림 상자 위치 검사.
- 원격 도구: `scenes` 명령 구성·범위 파싱·sbatch 인자·manifest 검증 시험(기존 패턴).
- 컨테이너: 캡처 노드 시험(동기화 stamp, depth 변환·거부 규칙, 저장 형식)은 ROS 패키지 `test/`.
- 원격: 시간 측정 spike → 반복 캡처 동일성 → 25개×4 → 세트 manifest → 검증 기록.

## 11. 구현 단계 (계획 3개, 각각 별도 검증)

1. **계약+로더** — 코어 타입·문서·로더·패키징. 원격 model-cpu 1회로 확인.
2. **카탈로그+SDF** — 생성기·카탈로그 v1 고정·SDF 리팩터링과 회귀. 호스트 시험으로 확인.
3. **캡처+원격** — 캡처 노드·runner·`scenes` mode·sbatch·시간 측정 spike·100개 생성·세트 manifest·검증 기록.

## 12. 사용자 결정 필요 항목

1. 카메라: **640×480·HFOV 69°(권장, D435i RGB 시야 모사)** vs 320×240·90°. 시간 측정 spike 결과에 따라 최종 확정.
2. 조건 비율 60/20/10/10과 범위: x 2.0–4.0 m, y ±1.0 m, yaw ±30°, 개구 폭 0.20–0.28 m(높이 0.20 고정).
3. dev/eval 70/30.
4. 장면당 프레임 1세트(시퀀스는 M3).
5. 가림 정의: 물리 폭 비율 20–60 %(영상 면적 아님), 가림은 정답을 바꾸지 않고 가시성 메타데이터로만 기록.
