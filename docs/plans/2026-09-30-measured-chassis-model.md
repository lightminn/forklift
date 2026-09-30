# 실측 차체 모델 `dls08_measured` — 계획

작성일: 2026-09-30 · 개정 2026-10-01 두 차례 (독립 검토 반영, 아래 자체 검토 절)

## 요청과 사용자 결정

사용자 요청(2026-09-30): 「1번 진행해」. 1번은 같은 날 Codex 교차검증과 수렴한 다음 작업 목록의
첫 항목으로, **팀 실측값을 문서뿐 아니라 실행 경로(생성 모델·Isaac 장면·구동 기하·삽입 한계)까지
전달한다**는 뜻이다. 같은 날 사용자 답변:

| 질문 | 답 |
|---|---|
| 최대 조향각 15° 의 정의 | **한쪽 바퀴를 직접 잼** (안쪽/바깥쪽 구분 없음) |
| 전장·전폭·전고·질량, 승강 범위 "동일" | **직접 재서 같았음** — 실측값으로 승격 |
| 포크 뿌리·캐리지·차축–포크 끝 거리 미측정 | **잠정 규칙으로 진행** — 계산으로 옮기고 "추정" 표시 |
| 모델 위치 | **새 모델 디렉터리** — `dls08_provisional` 과 Gazebo 기준선은 그대로 |

## 실측 출처

`forklift-presentations/week-05/build_deck.py` `MEASURE_ROWS` 와 `SCRIPT.md` 2·3쪽(팀 실측,
2026-09-29–30):

| 항목 | 실측 | 잠정 모델 |
|---|---|---|
| 축간 거리 · 윤거 | 0.66 · 0.53 m | 0.64 · 0.51 m |
| 바퀴 반지름 | 0.125 m | 0.135 m |
| 최대 조향각 | 15° (한쪽 바퀴) | 0.45 rad |
| 포크 길이 · 폭 · 두께 | 360 · 55 · 25 mm | 420 · 55 · 24 mm |
| 포크 중심 간격 | 55–380 mm 수동 조절 | 290 mm 고정 |
| 전장 × 전폭 × 전고 · 질량 | 1.46 × 0.63 × 1.01 m · 24 kg | 같음(카탈로그) |
| 승강 범위 | 280 mm | 280 mm |

**측정 기준이 적히지 않은 값 세 개.** 윤거(중심 간/외측 간), 포크 길이(뿌리 뒷면부터/캐리지
앞면부터의 노출 길이), 15°(어느 바퀴, 무엇에 대한 각). 아래 해석을 쓰고 README 와 결과 표에
"해석"으로 적는다. 다음 실측 요청 목록(§범위 밖)에 넣는다.

## 설계

### D1. 새 모델 `sim/models/dls08_measured/`

`dls08_provisional` 과 그 산출물은 Gazebo 동결 기준선([ADR 0004](../decisions/0004-simulation-engine-and-insertion-depth.md))과
과거 기록이 해시로 참조하므로 **바이트 단위로 그대로 둔다.** 같은 생성기로 새 디렉터리를 만든다.
생성기의 `model_name` 검사를 두 이름 허용으로 넓힌다.

`parameters.yaml` 값(★ 실측, △ 실측과 잠정 배치에서 계산, ▽ 추정 유지, ◇ 시험 설정값):

| 필드 | 값 | 근거 |
|---|---|---|
| catalogue 4개 | 1.46 · 0.63 · 1.01 · 24.0 | ★ |
| rear_axle_x_m | −0.34 | ▽ base_link 기준 뒤차축 위치는 미측정 — 기준으로 고정 |
| front_axle_x_m | 0.32 | △ −0.34 + 0.66 |
| rear_extent_x_m | −0.51 | ▽ 뒤차축–후단 0.17 m 미측정 |
| wheel_radius_m | 0.125 | ★ |
| wheel_track_m | 0.53 | ★ **중심 간으로 해석** |
| wheel_width_m | 0.08 | ▽ 0.10 → 0.08. 볼트 장식 외곽이 `track/2 + width/2 + 0.009` 라 0.10 이면 0.324 m 로 전폭 반값 0.315 를 **9 mm** 넘는다(envelope 검사 실패, 시험 생성으로 확인). 0.082 이하여야 하며 0.08 에서 0.314 m |
| fork_root_x_m | 0.59 | △ 포크 끝(−0.51 + 1.46 = 0.95) − 0.36. **포크 끝 = 전장 끝**(생성기 `tip = rear + overall_length`, envelope 이 강제) 그리고 **360 mm = 뿌리부터**로 해석 |
| body_front_x_m · mast_x_m | 0.50 · 0.54 | △ 잠정값 + 0.06. 포크 뿌리 이동분만큼 전면 묶음(차체 앞면·마스트)을 함께 옮긴다. 그대로 두면 마스트 앞면(0.505)과 힐 뒷면(0.577) 사이에 7 cm 빈 공간이 생기고 앞바퀴(0.195–0.445)가 차체 앞면 0.44 를 넘는다 |
| fork_thickness_m | 0.025 | ★ |
| fork_width_m | 0.055 | ★ |
| fork_spacing_m | 0.29 | ◇ 조절 범위(중심 55–380 mm, 안쪽 0–325 mm, 수동) 안의 EPAL 6 시험 설정 |
| lift_travel_m | 0.28 | ★ |
| steering_limit_rad | 0.2617993878 | ★ 측정값 15° 를 **두 바퀴 모두의 한계**로 배정(보수적 배정) |
| 나머지 | 잠정 모델과 같음 | ▽ |

생성기에 하드코딩된 `fork_carriage` 관성 위치 `(0.59, 0, 0.15)` 는 `fork_root_x_m` 에서 파생하도록
바꾼다 — **잠정 모델에서 값이 같게 나오는지**(0.53 + 0.06 = 0.59) 시험으로 고정해 잠정 산출물의
바이트 보존을 지킨다. 생성기에는 부품 간 겹침 검사가 없다(뒤 타이어–counterweight 교차는 잠정
모델에도 있는 기존 상태). 이 계획은 envelope 통과만 주장한다.

**조향각 배정.** `ackermann_command()` 는 두 앞바퀴가 모두 한계 안에 있도록 곡률을 자른다
(`path_tracking.py:405–412`). 두 바퀴에 15° 를 주면 뒤차축 중심 최소 반경은
0.66/tan15° + 0.53/2 = **2.728 m**(κ 0.36655 m⁻¹)이다. 발표의 약 2.5 m 는 자전거 모델
0.66/tan15° = 2.463 m 이고, 15° 가 바깥 바퀴였다면 2.198 m 다. 세 값은 전제가 다를 뿐 모두 맞는
계산이다 — 회전 원 실측(다음 작업 2)이 가른다.

### D2. 새 모델의 Isaac base scene

운반(`run_transport.py`)과 SLAM(`run_slam_drive.py`)은 `--base-scene` 의 `/World/Forklift` 를 쓰고
`--forklift-urdf` 로 차체를 다시 가져오지 않는다(`run_transport.py:674`, `run_slam_drive.py:249`).
ws1 의 기존 base scene 은 `artifacts/base_scene_warehouse_forklift/scene.usda` 가
`forklift.usd`(잠정 URDF 의 importer 출력)를 참조하는 구조다. **URDF 만 바꾸면 물리는 반지름 0.135,
계산은 0.125 로 도는 불일치(이동거리 1.08 배)가 생긴다.**

- `sim/isaac/build_base_scene.py`(신규): 기존 base scene 을 열고, 주어진 URDF 를
  `run_perception_approach.py:100–118` 과 같은 importer 설정으로 가져와 `/World/Forklift` 참조만
  바꿔 새 디렉터리에 저장한다. `base_scene_manifest.json` 에 원본 scene·URDF 의 SHA-256 과
  importer 설정을 적는다. 이것은 README 의 D2 미완 항목 "외부 base scene 재생성 절차" 중 **차체
  교체 부분**만 채운다.
- **장면–URDF 일치 검사**(`sim/isaac/chassis_contract.py`, 두 실행기가 stage 를 연 직후):
  ① 네 관절(`left_steer`·`right_steer`·`rear_left_spin`·`rear_right_spin`)의 위치 — USD 의
  `physics:localPos0` 는 `physics:body0` 링크 프레임 기준이므로 body0 의 월드 변환과 base_link
  역변환으로 **base_link 좌표로 합성**한다 ② 조향 한계 — USD 는 degree 라 radian 으로 바꾼다
  ③ 네 타이어 충돌 원통의 실효 반지름(스케일 반영, Z 축·xy 등방만 허용) ④ `fork_carriage` 의
  모든 충돌 상자(이름·중심·반치수, 회전 불가). 관절 z 와 원통 반지름은 USD 에서 독립이라
  관절만으로는 반지름을 증명하지 못하기 때문에 ③④ 를 둔다. 허용치는 1 mm·0.001 rad 이고
  **초과**하면 거부한다. 장면 단위(metersPerUnit = 1)도 확인한다.
  ws1 base scene 을 로컬로 복사해 Isaac 5.1 importer 출력 구조(`/<root>/joints/<name>`,
  `<link>/collisions/<name>_collision/{box,cylinder}` 인스턴스 프록시, 스케일된 단위 Cube)를
  확인했고, 이 읽기가 잠정 장면을 잠정 URDF 와 일치로, 실측 URDF 와 24.5 mm 불일치로 판정함을
  확인했다(2026-10-01).
- builder 는 기존 `/World/Forklift` **prim spec 을 통째로 지운 뒤** 새 참조를 정의한다. 참조만
  바꾸면 기존 하위 override 가 새 참조보다 우선할 수 있다(Codex 가 USD fixture 로 재현). 저장 전에
  같은 일치 검사를 돌려 통과한 장면만 쓴다.

### D3. URDF 독자

`sim/isaac/insertion_geometry.py` 에 추가:

- `read_drive_geometry_m(urdf, max_wheel_rate)` → `AckermannGeometry`. 축간 거리
  (`left_steer` x − `rear_left_spin` x), 윤거(`left_steer` y − `right_steer` y), 바퀴 반지름
  (네 타이어 충돌 원통 반지름이 같고 x 축 π/2 회전), 조향 한계(`left_steer` 와 `right_steer`
  의 limit 가 ±같음).
- `read_carriage_limit_m(urdf)` → 포크 끝 조인트 x − `carriage_cross_*` 충돌 상자 앞면 최대 x.

**계약.** 조인트 `origin` 은 부모 링크 기준이다. 독자는 부모 체인을 따라 **평행이동을 합성**하고,
체인에 회전이 있거나 조향 관절 축이 +z·spin 축이 +y 가 아니거나 조향이 앞차축이 아니거나 좌우가
대칭(1 mm)이 아니면 거부한다. 기존 `read_chassis_reference_m` 도 같은 합성으로 바꾼다
(`fork_lift` 원점을 옮긴 반례에서 1.29 대신 1.39 를 내야 한다 — 단위시험).

**부모 관절은 fixed 만** 허용한다. 예외는 `fork_lift`(내린 자세)와 조향 관절(직진 자세)뿐이다
— 원점 rpy 가 0 인 revolute 중간 부모도 평행이동 합성은 통과하지만, 회전하면 좌우 조향 중심이
101 mm 어긋나 상수 Ackermann 기하가 성립하지 않는다(Codex 계산).

하드코딩 교체: `run_transport.py:1073`·`438`, `run_slam_drive.py:481`·`732`. 두 실행기의
`--forklift-urdf` 는 **필수 인자**로 바꿔 기본값이 조용히 한 모델을 고르지 않게 한다.

### D4. 삽입 한계를 모델에서 받는다

`target_insertion_depth_m(depth, carriage_limit_m=CARRIAGE_INSERTION_LIMIT_M)` 로 인자를 열고
`carriage_limit_m` 은 유한값이고 `INSERTION_RESERVE_M` 보다 커야 한다(아니면 `ValueError`).
`SyntheticMissionGeometry` 에 `carriage_limit_m` 필드(기본 = 상수)를 둔다. 운반·SLAM 실행기는
`read_carriage_limit_m` 결과를 넘긴다.

**기본값을 잠정 상수로 남기는 범위.** `preview_docking`·`build_t11_drawing`·`summarise_sweep`·
`deck`·`factory_planning_sweep`·`benchmark_transport_planning`·`run_perception_approach` 는 잠정 모델의
CPU 기준선으로 **명시적으로 남긴다**(각 파일 docstring 에 한 줄). 이 도구들은 공용 설정도 잠정
설정을 읽으므로(D5) 두 모델이 섞이지 않는다. `test_preview_docking` 의 0.406/0.360 은 잠정 회귀로
유지한다.

새 모델에서 규칙 `min(깊이 × 0.6, 한계 − 0.046)`, 한계 0.95 − (0.59 + 0.014) = 0.346 m:

| 형상 | 목표 | 목표 − 깊이/2 | 삽입 후 차축–팔레트 중심 | 적재 전방 외곽 |
|---|---:|---:|---:|---:|
| EPAL 6 (600) | 0.360 → **0.300** | **0.000** | 1.230 → 1.290 | 1.530 → **1.590** |
| T11 × 0.6 (660) | 0.360 → **0.300** | **−0.030** | 1.260 → 1.320 | 1.590 → **1.650** |

두 형상 모두 캐리지 항이 지배한다. 360 mm 가 캐리지 앞면부터의 노출 길이였다면 한계는 0.360,
목표는 0.314 가 된다(14 mm 차). 깊이/2 는 종방향 대용값이라 전도를 단정하지 않는다. **46 mm
정책과 T11 지지의 양립 판단은 캐리지 실측 뒤 별도로 한다**(다음 작업 3).

### D5. 곡률 설정은 새 파일로

`config/isaac_transport.yaml`·`isaac_transport_fast.yaml` 은 **바꾸지 않는다** — CPU 도구가 공용으로
읽어 잠정 형상과 섞이기 때문이다. 새 `config/isaac_transport_measured.yaml` 을 만든다. 잠정 모델
설정은 기구 한계 0.63295 m⁻¹ 대비 추종기 0.62(98 %)·계획기 0.50(79 %)였고, 같은 비율을 새 한계
0.36655 에 적용해 **추종기 0.36 · 계획기 0.29** 로 둔다. 나머지는 기본 설정과 같게 두되
바퀴 반지름에 기대는 주석(최고 속도 8 × 0.125 = 1.00 m/s)을 고친다. 빠른 설정의 실측판은 만들지
않는다(속도 작업은 우선순위 밖, 사용자 2026-09-28).

두 실행기는 **설정 곡률 > URDF 기구 한계**면 시작 전에 거부한다(잠정 설정 + 실측 URDF 조합이
여기서 걸린다). 곡률 값의 타당성은 A–D·운반 재검증(다음 작업 2)의 몫이다.

`SyntheticMissionGeometry` 무부하 외곽(앞 1.29·뒤 0.17·반폭 0.36)은 새 기하에서도 유효하다 —
조향 타이어 외곽 0.265 + 0.125 sin15° + 0.04 cos15° = 0.3360 m(볼트 장식 0.3336 m) < 0.36.
운반·SLAM 실행기는 이미 URDF 에서 차축–포크 끝을 읽어 넘긴다. 1.29 는 "포크 끝 = 전장 끝"·"뒤차축
−0.34" 두 추정 위의 값이다.

### D6. 이번에 전환하지 않는 경로

- `run_pocket_insertion.py`: 자체 설정 `target_front_x_m 0.59`(삽입 0.360 → 새 캐리지 앞면 0.604
  보다 14 mm 뒤), 포크 높이 `[0.028, 0.052]` 고정, 그리고 `SyntheticMissionGeometry(approach_offset_m=…)`
  가 `init=False` 필드를 넘겨 **지금도 `TypeError`** 로 시작하지 못한다(Codex 재현). PR #2 갈래의
  재측정 대기 경로이므로 이번에 전환하지 않고, 결함은 현황 문서에 적는다.
- `run_perception_approach.py`·`verify_perception_camera.py`: 잠정 모델 기본값을 유지한다. G1 은
  base scene 의 차체를 쓰며 `--forklift-urdf` 는 출처 기록일 뿐이라, 기본값만 바꾸면 잠정 장면에
  실측 출처를 붙이게 된다. G1 결과는 잠정 장면 기준선으로 유지한다.
- 센서 장착 기준은 움직이지 않는다: 운반·G1 카메라 base_link (0.75, 0, 0.50), 접근·포켓 실험
  카메라 z 0.27, LiDAR (−0.12, 0, 1.05) — 전고 1.01 이 같아 차체 위 40 mm 가 유지된다(렌더
  self-hit 는 미검증). 팔레트 prior 는 차체와 무관하다.

### D7. 문서

- `docs/hardware.md` 차체 행, `docs/validation/2026-09-23-chassis-intake.md` 에 실측 절 추가(§3 표에서
  측정된 항목을 옮기고 남은 항목을 남긴다).
- `sim/models/dls08_measured/README.md`: 필드별 출처 표(★△▽◇), 해석 세 개, 포크 간격 범위와 설정의
  구분.
- `CLAUDE.md`·`AGENTS.md` 의 삽입 깊이 절: "360 mm for both shapes today" 를 모델별 결과(잠정 360 ·
  실측 300)로 고친다(두 파일 동일).
- 현황 문서에 `run_pocket_insertion` 결함과 이번 전환 범위를 적는다.

## 범위 밖

A–D·운반·SLAM 재실행과 성적 갱신(다음 작업 2), 캐리지 실측과 46 mm 정책 판단(3), 동봉 팔레트 형상
계약(4), Gazebo 기준선·`dls08_provisional`·과거 기록 수정, CPU 도구의 모델 선택 기능, G1 재실행·G2
개방, `run_pocket_insertion` 수정. **다음 실측 요청**: 윤거·포크 길이·15° 의 측정 기준, 뒤차축–포크
끝 거리, 캐리지 가로대 앞면 위치, 포크 최저·최고 높이, 뒤차축–후단 거리.

## 완료 기준

1. `build_forklift_model.py` 가 `dls08_measured` 를 만들고 envelope 검사를 통과한다.
   `dls08_provisional` 의 네 산출물을 다시 생성하면 바이트 단위로 같다(관성 위치 파생 변경 포함).
2. `read_drive_geometry_m` 이 실측 URDF 에서 (0.66, 0.53, 0.125, 15°), 잠정 URDF 에서
   (0.64, 0.51, 0.135, 0.45). 부모 링크 이동 반례 두 개(조향 중간 링크 +0.10 → 0.74, `fork_lift`
   +0.10 → 1.39)를 맞게 합성하고, 회전 체인·축 방향·비대칭은 거부한다 — 단위시험.
3. `read_carriage_limit_m` 이 잠정 0.406 · 실측 0.346. `target_insertion_depth_m` 이 기본 인자에서
   기존과 같고, 0.346 에서 EPAL 6 · T11 모두 0.300, NaN·0.046 이하는 거부 — 단위시험.
4. 곡률 설정이 기구 한계를 넘으면 거부. 장면–URDF 비교가 0.9 mm 는 통과·1.1 mm 는 거부하고,
   관절은 같고 타이어 반지름만 또는 가로대 위치만 다른 importer 형식 USD 를 거부하며, body0 가
   중간 링크인 경우를 올바르게 합성한다 — 단위시험(`pxr` 필요).
5. 두 실행기에 `AckermannGeometry(0.64` 리터럴이 없고 `--forklift-urdf` 가 필수다.
6. `pytest tests --ignore=tests/simulation` 새 실패 없음, `tests/simulation/test_forklift_model.py`
   통과, 변경 파일 Ruff 통과.
7. ws1(Slurm): `build_base_scene.py` 로 실측 base scene 을 만들고(검사 통과가 저장 조건), 운반
   seed 0 정답 좌표 1회와 SLAM 조사 1회가 장면–URDF 검사를 통과해 시작하는지 확인한다(완주 여부는
   기록만 — 재검증은 다음 작업 2). 기존 잠정 장면 + 실측 URDF 조합이 거부되는지도 1회 확인한다.
   실제 prim 경로·body0·단위와 합성 전후 값을 결과에 남긴다.

## 자체 검토

**① 직접 재계산 (2026-10-01).** 원자료(발표 `MEASURE_ROWS`, 생성기, 잠정 URDF)에서 다시 뽑았다.
κ 0.366548 · R 2.728 m, 잠정 κ 0.632951, 추종·계획 비율 적용 0.3590·0.2896, 한계 0.346, 목표 0.300,
반폭 0.3360/0.3450. 시험 생성(스크래치)에서 바퀴 폭 0.10 은 envelope 실패, 0.08 은 통과하고 생성
URDF 의 `carriage_cross_0` 가 x 0.59 · 두께 0.028 로 한계 0.346 을 확인했다.

**② Codex 적대적 검토 (1차).** 수정 필요 6건, 모두 원자료로 확인 후 반영:
운반·SLAM 이 base scene 의 차체를 써 URDF 교체만으로는 물리 불일치(→ D2), `run_pocket_insertion` 의
별도 목표·포크 높이·기존 `TypeError`(→ D6 범위 밖으로 명시), 공용 설정 변경이 CPU sweep 을 두 모델로
섞음(→ D5 새 설정 파일), URDF 부모 체인 합성과 구조 검증(→ D3), G1 기본값이 출처만 바꿈(→ D6),
envelope 외곽 수치 0.316 → 0.324 정정과 겹침 검사 부재·시험 경계(→ D1, 완료 기준 1·6).

**③ Claude 서브에이전트 독립 검토 (Codex 결론 비공개).** 9건. 반영: 윤거 envelope 실패(①에서 이미
발견), 포크 길이 측정 기준에 따른 한계 0.346/0.360 분기(→ 해석 명시·D4), 포크 끝=전장 끝·뒤차축
고정의 겹친 가정(→ D1·D5 에 추정으로 명시), 전면 부품 간극과 관성 하드코딩(→ D1 전면 묶음 이동·
관성 파생), 조용히 옛 값을 쓰는 경로(→ D4·D6 명시), 빠른 설정 동작 변화(→ D5 에서 빠른 설정 미변경),
발표 2.46 과 2.728 의 전제 차이(→ D1). 반폭 볼트 포함 0.3544 는 바퀴 폭 0.08 기준 0.3450 으로 다시
계산했다. 시나리오가 반경 2 m 전제라는 지적은 다음 작업 2 에서 잠정 결과를 비교 기준으로 남기는
것으로 넘긴다.

**② Codex 적대적 검토 (2차, 개정본 대상).** D1 생성·바이트 보존, D4 계산, D5 설정 분리는 성립한다고
확인. 수정 필요 4건, 모두 반영: localPos0 는 body0 프레임 기준이라 base 합성이 필요하고 한계는
degree(→ D2 ①②), 관절만으로는 반지름·캐리지 충돌기하를 증명 못 하고 참조 교체만으로는 하위
override 가 남음(→ D2 ③④·spec 삭제), revolute 중간 부모 제한(→ D3), 허용치 경계와 완료 기준 4·7
보강. 외곽 수치 정정(볼트 0.3336, 이전 마스트 간극 0.068 → 이동 후 0.008 m)도 반영했다.

## 결과

[2026-10-01 실행 기록](../validation/2026-10-01-measured-chassis-model.md). 완료 기준 1–7 충족. ws1 스모크에서
실측 장면이 만들어지고 운반·SLAM 이 장면 검사를 통과해 시작했으며, 잠정 장면 + 실측 URDF 는 거부됐다.
운반 seed 0 은 삽입까지 진행한 뒤 들기 중 팔레트 기울기로 중단됐다(D4 의 "목표 − 깊이/2 = 0" 이 드러난 것으로
읽음, 다음 작업 3 의 입력).
