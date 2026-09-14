# 검증 기록: EPAL 6 캡처와 평가 1 회 (2026-09-14)

**무엇을 했는가.** 재타깃한 epal6 카탈로그 100 장면을 Gazebo Harmonic 으로 캡처하고, 유도 파라미터로 dev·eval 을 각각 한 번 평가했다. **이것이 채택한 유도가 합성 해석 리그가 아니라 렌더된 깊이 영상에서 검증된 첫 사례다.**

⚠️ **무엇을 하지 않았는가 — 먼저 읽을 것.**
- **실센서가 아니다.** Gazebo 렌더이고 깊이는 시뮬레이터가 낸다. D435i 의 실오차(거리의 약 2 %, 2.5 m 에서 ~50 mm)는 여기에 없다.
- **채택 시험 형상(T11 × 0.6)이 아니라 EPAL 6 다.** 실물로 만들 팔레트가 아니다(ADR 0002). 이 표는 **유도 규칙이 렌더 데이터에서 동작하는지**를 말하고, 채택 형상의 성능은 말하지 않는다.
- **카메라는 (0.75, 0, 0.5) 고정이다.** §C-19 가 잰 대로 이 장착에서는 **1.90 m 아래가 전 자세 미검출**이고 브리프 Case C·D 가 사정권 밖이다. 카탈로그 자세는 2.0 ~ 4.0 m 라 그 구간을 **구조적으로 볼 수 없다.**
- **§A ⓪ 는 (c)"재캡처 안 함"이었다.** 사용자가 "계획서 모두 구현" 을 지시해 그 결정을 뒤집고 캡처했다. 재개 조건(차체 입고·카메라 장착 확정·실물 팔레트)은 **여전히 충족되지 않았다.**

## 1. 캡처

| | 값 |
|---|---|
| 카탈로그 | `sim/gazebo/scenes/catalogue_epal6.yaml` (v1 100 자세 재타깃) |
| 장면 | **100 / 100 성공, 실패 0** |
| 이미지 | `sha256:1ce743e9…` (`deploy/gazebo/Dockerfile` 로 빌드) |
| 스냅샷 | `5832ef5e…` |
| 장면당 소요 | 약 7.7 초 |
| 병합 결과 | dev 70 / eval 30, positive 60 · occluded 20 · negative 20 |

**Task 4.5 스모크(2 장면)를 전체 캡처 전에 돌렸다.** 22 상자 URDF 경로가 실제 Gazebo 에서 처음 도는 지점이라, 되돌릴 수 없는 100 장면 캡처에서 처음 드러나는 것을 막는 것이 그 Task 의 목적이었다.

## 2. 스모크에서 바로 나온 결과 — 동결은 0, 유도는 검출

캡처한 두 장면에 두 파라미터를 걸었다:

| | s001 | s002 |
|---|---|---|
| 동결 | `no_pallet/no_opening_pattern` | 같음 |
| **유도** | **`valid`** (lower 39) | **`valid`** (lower 49) |

**`lower_band_is_clipped` 가 그대로 예측했다** — 동결 True, 유도 False. 동결 `floor_z_m` 20 mm 가 EPAL 6 의 22 mm 바닥판을 z 필터에서 지워 `lower` 가 0 이 되고, 유도값 7.33 mm 에서는 바닥판이 살아남는다. **계획이 §C-2·§C-18 에서 해석 리그로만 보던 것이 렌더된 깊이에서 재현됐다.**

## 3. 평가 1 회 — 유도 파라미터

파라미터는 prior 에서 유도했다(`--derive-params`): `plane_inlier_m` **7.80 mm**, `min_band_points` **15**, `floor_z_m` **7.33 mm**. 두 실행 모두 git `7ac4530`, dirty **false**.

| | dev (70) | eval (30) | 목표 |
|---|---|---|---|
| **positive 검출률** | **1.00** (42/42) | **1.00** (18/18) | ≥ 0.95 ✅ |
| **위치 p95** | **8.17 mm** | **5.98 mm** | ≤ 20 mm ✅ |
| 위치 최대 | 8.36 mm | 6.16 mm | (상한 미정, §D-2 (a)) |
| **yaw p95** | **0.00093 rad** (0.053°) | **0.00045 rad** (0.026°) | ≤ 2° ✅ |
| 음성 위양성 | **0 / 14** | **0 / 6** | — |
| 음성 `invalid` | **0** | **0** | — |
| `targets.met` | **3/3 True** | **3/3 True** | |
| occluded 검출률 | 0.21 (3/14) | 0.50 (3/6) | (예산 없음) |

**positive 60 장면 전부 검출, 오위치 0, 위양성 0.** `wrong_pose` 도 0 이다.

**occluded 는 낮고 그것이 정상이다** — 가림 장면은 `invalid` 로 거부되는 것이 설계다(가림을 뚫고 추정하지 않는다). dev 11 / eval 3 이 `invalid` 이고 **위양성으로 새지 않았다**.

## 4. 이 실행이 M2 에 대해 말하는 것과 말하지 않는 것

로드맵 M2 의 합성 탐색 목표(위치 p95 ≤ 20 mm, yaw p95 ≤ 2°, 검출률 ≥ 95 %)를 **두 분할 모두에서 통과**한다. 다만 M2 완료로 적지 않는다:

- **거리 구간을 붙여야 참이다** — 이 카탈로그는 2.0 ~ 4.0 m 다. **1.90 m 아래는 전 자세 0** 이고(§C-19) 삽입 마지막 구간이 거기 있다.
- **이 세트도 홀드아웃이 아니다** — 규칙·문턱·유도 공식이 전부 v1·EPAL 6 를 미리 훑어 정해졌다(§H 의 마지막 행과 같은 의미로 오염됐다).
- **채택 형상이 아니다.** T11 × 0.6 의 캡처·평가는 실물 제작 이후다.
- **`invalid` 예산이 아직 비어 있다**(§D-2 (b)) — 음성 0/20 이지만 0/20 의 단측 95 % 상한은 **13.9 %** 다. "위양성이 낮다" 를 이 세트로 주장할 수 없다.

## 5. 재현

```bash
docker build -t forklift/gazebo:jazzy -f deploy/gazebo/Dockerfile deploy/gazebo
docker run --rm --network none -v "$PWD":/workspace/source:ro -v <out>:/workspace/out \
  -u 1000:1000 -w /workspace forklift/gazebo:jazzy bash -lc \
  'python3 /workspace/source/sim/gazebo/capture_scenes.py \
     --catalogue /workspace/source/sim/gazebo/scenes/catalogue_epal6.yaml \
     --scenes s001-s100 --output /workspace/out --image-id <id> \
     --source-sha256 <snap> --run-id epal6_full --scene-deadline-s 300'
python tools/merge_scene_batches.py --catalogue sim/gazebo/scenes/catalogue_epal6.yaml \
  --batches <out> --output data/synthetic_scenes/catalogue_epal6
python tools/evaluate_pocket_detector.py --dataset data/synthetic_scenes/catalogue_epal6 \
  --prior config/pallet_prior_epal6.yaml --split dev --derive-params --output <run>
```

**데이터셋과 실행 산출물은 `.gitignore` 대상이다**(`/data/`·`/artifacts/`). 위 명령으로 재생성한다.
