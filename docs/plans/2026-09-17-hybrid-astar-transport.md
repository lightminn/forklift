# Hybrid A* Transport Implementation Plan

> **For agentic workers:** Use the executing-plans/subagent-driven workflow task by task. Planner and simulator work have disjoint ownership; integration is sequential.

**Goal:** Implement and physically test random pallet pickup and transport through random official warehouse props to a green destination marker, with a global 60fps camera.

**Architecture:** Hardware-independent rear-axle Hybrid A* and path tracking; Isaac Sim adapter owns asset geometry, random scenario, lift/contact mission and recording.

**Tech Stack:** Python >=3.10, NumPy >=1.23, pytest >=7, Isaac Sim 5.1.0, USD/PhysX.

**Spec:** [Design](../design/2026-09-17-hybrid-astar-transport.md)

## Global Constraints

- Keep unmeasured provisional chassis and synthetic dynamics distinct from hardware.
- Flat floor; official NVIDIA warehouse props; randomized pallet, obstacles and destination.
- Full-scene camera; 60fps output, 120Hz physics; green ground circle marks pallet delivery center.
- Use simulator pose feedback; no pose teleportation after initialization, no attached pallet.
- No commits/push requested. Keep prior validation and artifacts intact. Host facts stay outside tracked guidance.

## Tasks

- [x] Planner: failing tests; continuous forward/reverse search and exact feasible goal connector; footprint collision tests and fixed-seed regression.
- [x] Tracking: failing tests for forward, reverse and cusps; measured wheel/steer diagnostics; bounded physical command conversion.
- [x] Scene: discover official props; randomized valid map; green circle and full-view camera; reproducible CLI/source snapshot.
- [x] Mission: unloaded and loaded plans; controlled insertion/lift/withdrawal; obstacle transport; grounded delivery and fork clearance verification.
- [ ] Run local tests and multiple deterministic HPC scenarios, retaining every failure. Record 60fps representative missions and verify video.
- [ ] Review code, run final required checks, save validation with results and source hashes, return code/video links.

## Video-first checkpoint *(그 시점의 기록 — 아래 「검증 재개」가 후속이다)*

The user requested the first result video only and deferred the main tests.
Keep the multi-seed physical campaign and final acceptance review pending.
The first recording uses seed 0 and four official warehouse props. Preview
completion must not be reported as full validation. The reviewer identified
missing insertion contact/pocket-clearance validation; this remains open.
Measured-pallet obstacle checks and the steered-tire envelope were corrected
before recording. The command-history ramp and curved-endpoint lookahead have
targeted regression evidence; the complete suite has not been rerun on this
final preview revision.

## 검증 재개

사용자가 기록 버그 수정 후 테스트 벤치와 대규모 검증을 요청했다.
JSON 수치형 버그와 삽입 기하 검사 보완 후 로컬 778개 시험이 통과했다.
물리 100개 seed 작업1025와 계획1000개 seed 작업1026을 제출했다.
대규모 최종 집계·분석은 아직 완료되지 않았다. 실행 기록은
`docs/validation/2026-09-17-isaac-hybrid-astar-bench.md`를 따른다.


## 형상 일반화 — 미완 (2026-09-17, 3 라운드 교차검증)

⚠️ **이 코드는 EPAL 6 을 가정한다. ADR 0002 는 EPAL 6 과 T11 ×0.6 을 모두 요구한다.**
그리고 [ADR 0004 D3](../decisions/0004-simulation-engine-and-insertion-depth.md) 은 삽입 깊이를
**값이 아니라 규칙**으로 들고 다니라고 적었는데, 지금은 규칙의 **EPAL 계산 결과**가 오프셋에
굳어 있다.

⚠️ **이 결함은 새로 발견된 것이 아니다.** [2026-09-17 WebGPT 검토](../validation/2026-09-17-dual-pallet-webgpt-review.md)의
첫 항목(`:20-27`)이 **머지 전에 같은 것을 지적했다** — 인자 없는 `SyntheticMissionGeometry()`,
`pallet_mission.py:75-95` 의 EPAL 기본값, T11 에서 1.26·1.59 가 되는 유도, 실행 루프에 박힌
`(0.3, 0.3, 0.4)` 와 `0.89` 까지 전부 적혀 있다. **그럼에도 그대로 머지됐다.** 지적이 추적 항목으로
남지 않으면 통과한다는 뜻이고, 그래서 이 절을 계획서에 **미완 작업**으로 남긴다.

### 무엇이 깨지는가 — 실행으로 확인했다

`SyntheticMissionGeometry`(`src/forklift_core/planning/pallet_mission.py:85-95`)는
`pallet_depth_m = 0.60` 과 `inserted_offset_m = 1.23` 을 **서로 독립된 필드**로 들고 있고
`__post_init__`(`:97-114`)은 양수·오프셋 순서만 검사한다. 축 x = −0.34, 포크끝 x = 0.95 이므로
F = 1.29 이고 삽입 깊이 = 깊이/2 + (F − `inserted_offset`) 이다.

| 형상 | 코드가 만드는 삽입 깊이 | D3 규칙 |
|---|---:|---:|
| EPAL 6 (600 mm) | 360 mm | 360 mm |
| **T11 ×0.6 (660 mm)** | **390 mm** | **360 mm** |
| 합성 (500 mm) | **310 mm** | **300 mm** |

`SyntheticMissionGeometry(pallet_depth_m=0.66, pallet_width_m=0.66)` 은 **아무 오류 없이
생성된다.** 46 mm 예비가 강제되지 않는다.

**오프셋만 고쳐도 부족하다.** 후축 기준 x = 1.57 에 10 mm 장애물을 놓고 실제 충돌 함수를 돌리면
기존 `loaded_footprint`(front 1.53)는 `collision_free = True`, T11 유도 외곽(1.59)은 `False` 다.
적재 외곽의 앞쪽 60 mm 를 통째로 놓친다.

### 사양

**입력**: `pallet_depth_m` D · `pallet_width_m` W · **`axle_to_fork_tip_m` F** · 접근 간격(0.10) ·
정렬 직선(0.80) · 배송 직선(0.70) · 추출(0.65) · 인출(0.55) · spawn 여유(0.12) · 차체 반폭(0.36) ·
차체 후방(0.17).

⚠️ **F 는 `unloaded_footprint.front_m` 과 값이 같지만 같은 양이 아니다.** F 는 **포크끝 위치**이고
`Footprint.front_m` 은 **충돌 외곽**이다(`planning/geometry.py:47`). 외곽에 보수적 여유를 더했을 때
삽입 위치가 따라 움직이면 안 된다. 근거는 URDF 다 — `left_fork_collision` 중심 x = 0.74 · 길이
0.42 → 끝 0.950, `left_fork_tip_fixed` 조인트도 x = 0.95, `rear_left_spin` 이 x = −0.34
(`sim/models/dls08_provisional/forklift.urdf`).

**유도** — `init=False` dataclass 필드로 둔다. ⚠️ **property 로 만들면 안 된다**:
`run_transport.py:177` 이 `asdict(geometry)` 로 기록하는데 property 는 거기 안 들어가 실행 기록에서
유도값이 사라진다.

- `d = min(D × 0.6, 0.406 − 0.046)`
- `inserted_offset = F + D/2 − d`
- `approach_offset = F + D/2 + 접근 간격` — 삽입 직선이 두 형상 모두 **0.46** 으로 보존된다
- `prealign_offset = approach_offset + 정렬 직선`
- `predelivery_offset = inserted_offset + 배송 직선`
- `loaded_footprint` = **비적재 외곽과 적재 팔레트 외곽의 합집합**:
  `Footprint(max(unloaded.front_m, inserted_offset + D/2), unloaded.rear_m, max(unloaded.half_width_m, W/2))`
  ⚠️ **팔레트 쪽만 쓰면 안 된다** — 비적재 외곽에 보수적 여유를 더했을 때 적재 외곽이 그보다
  작아질 수 있다. 실측 반례: F 를 1.29 로 둔 채 비적재 front 를 1.60 으로 늘리고 후축 x = 1.57 에
  10 mm 장애물을 놓으면 **비적재는 충돌, 팔레트 쪽만 쓴 적재 외곽(1.53)은 통과**한다.
- `unloaded_footprint` 는 그대로 — 팔레트와 무관하다
- `extracted`·`withdrawn` 은 이미 `inserted` 에 거리를 더하므로 자동으로 따라간다(`:140-147`)

| 형상 | inserted | approach | prealign | predelivery | loaded front | 반폭 | 삽입 직선 |
|---|---:|---:|---:|---:|---:|---:|---:|
| EPAL 6 | 1.23 | 1.69 | 2.49 | 1.93 | 1.53 | 0.40 | 0.46 |
| T11 ×0.6 | 1.26 | 1.72 | 2.52 | 1.96 | 1.59 | 0.36 | 0.46 |

**규칙 상수**(0.6 / 0.406 / 0.046)는 코어 한 곳에 두고 `pallet_mission` 과 `build_t11_drawing` 이
참조한다. ⚠️ **`build_t11_drawing.py:40` 의 `DEFAULT_SCALE = 0.6` 과 `:42` 의
`INSERTION_FRACTION = 0.6` 을 합치지 말 것** — 앞은 ADR 0002 의 **축척**, 뒤는 **삽입 정책**이다.
숫자가 같을 뿐이다.

**`run_transport.py` 의 잔존 상수를 geometry 에서 끌어온다.**

| 위치 | 현재 | 바꿀 것 |
|---|---|---|
| `:306-308` | `Rectangle(..., 0.6, 0.8, ...)` | `(D, W)` |
| `:432` | `Footprint(0.3, 0.3, 0.4)` | `Footprint(D/2, D/2, W/2)` |
| `:441` | `abs(... − 0.89) < 0.08` | `inserted_offset − 0.34` (T11 이면 0.92 — 허용폭 ±0.08 안에 숨어 **30 mm 편향**된다) |

⚠️ **`:432` 검사 자체는 유지한다** — 실제로 움직인 팔레트의 중심·yaw 기준이라, 후축 자세 기준의
차체 검사가 못 잡는 미끄러짐·회전을 잡는다. 없애면 안 된다.

**실행기 입력 연결이 사양의 일부다.** 지금 CLI 에는 `--pallet-urdf` 만 있고 mission 치수 입력이
없으며(`:45-62`) `:168` 이 인자 없는 `SyntheticMissionGeometry()` 를 만든다. **기본 생성 + 일치
검증만 넣으면 T11 은 올바르게 거부되지만 실행할 수가 없다.** 흐름을 명시한다 —
**선택한 mission 설정의 D/W → geometry 생성 → 입력 URDF 와 대조 → 같은 geometry 를 spawn·계획·
실행 검사·기록에 전달.**

**인터페이스를 여기서 닫는다 (구현자가 고르게 두지 않는다).**

| 항목 | 결정 |
|---|---|
| D/W 정본 | **`config/pallet_geometry_*.yaml`** 의 `overall_depth_m`·`overall_width_m`. 이미 이 저장소의 형상 정본이고 `build_pallet_model.py` 가 URDF 를 만들 때 쓰는 입력이다 |
| CLI | `--pallet-geometry <yaml>` 을 **필수**로 추가한다. `--pallet-urdf` 는 그대로 두고 **둘을 대조**한다 |
| 중복 입력 우선순위 | 없음 — 둘이 어긋나면 **우선순위를 정하지 말고 거부**한다. 조용히 한쪽을 이기게 하면 이 수정의 목적이 사라진다 |
| 대조 허용오차 | **절대 1 mm.** 모델은 mm 단위로 생성되므로 상대오차는 쓰지 않는다 |
| T11 시험 자산 | **저장소에 커밋하지 않는다.** T11 형상이 미확정이라 커밋하면 350 해석을 박는 셈이 된다. 시험은 **합성 D/W 로 fixture 를 만들어** 유도·대조 경로만 고정한다 |

⚠️ **URDF 에서 D/W 를 뽑아 그것으로 geometry 를 만들면 안 된다** — 그러면 같은 URDF 에서 두 번 뽑은
값을 비교하는 것이라 **형상 선택을 검증하지 않는다.** YAML 이 독립적인 기대값이고 URDF 가 검사
대상이다.

**⚠️ 코드를 고쳐도 T11 을 못 돌린다 — T11 팔레트 모델이 없다.** `sim/models/` 에 있는 팔레트는
`epal6_pallet/pallet.urdf` **하나뿐**이다(실측: 상자 22 개, 외곽 0.600 × 0.800 × 0.144). T11 은
`config/pallet_geometry_t11_06.yaml` 과 `pallet_prior_t11_06.yaml` 만 있고 모델이 없다.
`tools/build_pallet_model.py --geometry config/pallet_geometry_t11_06.yaml` 로 생성할 수는 있으나,
**그 YAML 이 이미 350 해석을 박아 두었다** — [T11 도면](../design/2026-09-17-t11-test-article-drawing.md)은
235 와 350 **어느 쪽도 채택하지 않았다.** 따라서 **T11 Isaac 실행은 코드 수정이 아니라 표준
번호·판본·도면 확정에 막혀 있다.** 코드 수정은 그것과 **독립적으로** 진행할 수 있고 해야 한다 —
합성 D 로 규칙 분기를 시험하면 T11 모델 없이도 유도 경로를 고정할 수 있다.

**URDF 검증은 치수만으로 부족하다 — 원점도 본다.** `PalletSite` 는 팔레트 **중심**이고
(`pallet_mission.py:46`) Isaac 은 rigid-body 원점을 그 위치에 둔다(`scene.py:251`). 그런데 URDF
판독(`insertion_geometry.py:32,83`)은 단일 링크·박스·무회전만 보고 박스 중심을 그대로 받는다.
**모든 박스를 x 로 +30 mm 옮기면 D·W 도 collision 개수도 그대로인데 삽입 목표와 실제 앞면이
30 mm 어긋난다.** 허용오차를 정해 collision 외곽이 `xmin = −D/2, xmax = D/2, ymin = −W/2,
ymax = W/2` 인지 검사하고, 어긋나면 거부한다(원점 오프셋 지원은 모든 위치 변환에 반영해야
하므로 이번 범위 밖이다).

⚠️ **z 까지 중심 정렬을 요구하면 현행 자산을 거부한다.** `build_pallet_model.py` 의 계약은
**"원점은 바닥 높이의 외곽 중심"**(그 파일 docstring)이고, `epal6_pallet/pallet.urdf` 실측도
중심 x = 0.0000 · y = 0.0000 · **z = +0.0720**(높이 0.144 의 절반, 즉 바닥이 z = 0)이다.
검사는 **xy 는 원점 중심, z 는 하단이 0** 으로 적는다.

**`scene.py:327` 의 22 박스 assertion 은 삭제하지 않는다.** 목적이 다르다 — 삽입 정책 상수가
아니라 **모델 무결성**(collision prim 이 제대로 import 됐는가)이다. 다만 "모든 팔레트가 22 개"
라는 계약으로 남기면 다른 구성을 막으므로, **입력 URDF 의 기대 collision 구성과 대조**하도록
바꾸고 **개수 일치가 치수·배치 검증은 아니라는 것**을 주석에 적는다.

### 먼저 쓸 실패 시험 (`CONTRIBUTING.md:168`)

| 시험 | 기대값·실패 조건 |
|---|---|
| T11 유도값과 직렬화 | D = W = 0.66 에서 d 0.36 · inserted 1.26 · approach 1.72 · prealign 2.52 · predelivery 1.96 · loaded front 1.59 · 반폭 0.36. **`asdict()` 에도 유도 필드가 있어야 한다.** 현재는 EPAL 고정이라 실패 |
| T11 외곽 반례 | 후축 x = 1.57 의 10 mm 장애물을 T11 geometry 가 **충돌로** 판정해야 한다. `margin_m = 0` 으로 외곽 오류 자체를 고정 |
| 삽입 규칙의 두 분기 | 합성 D = 0.50 → d = 0.30, D = 0.66 → d = 0.36. **두 실물 형상만 시험하면 `d = 0.36` 하드코딩도 통과**하므로 합성 입력이 따로 필요하다 |
| F 와 외곽의 독립성 | F 고정 · unloaded front 만 늘려도 inserted·approach 불변. 반대로 F 증가분만큼 두 오프셋이 증가. **그리고 늘린 비적재 외곽이 적재 외곽에도 보존되어야 한다**(합집합) |
| 실행 입력 연결·불일치 거부 | T11 설정 + T11 URDF 는 구성 성공, EPAL 설정 + T11 URDF 는 **spawn 전에** 실패. Isaac 없이 순수 입력 검증 경계에서 |
| 비중심 URDF 거부 | 박스 전체를 x 로 +0.03 옮긴 fixture 는 D·W·개수가 같아도 거부 |

⚠️ **기존 EPAL 좌표·직선 길이 시험과 `margin_m = 0.10` 추가는 RED 가 아니다** — 회귀 고정이고
현행 코드에서 이미 통과한다. 그것만으로 "실패 시험을 먼저 썼다" 고 보고하지 말 것.

⚠️ **T11 fixture 의 미확정 포켓 폭(235/350)을 이 시험들의 기대 상수로 쓰지 않는다.** 여기서
필요한 것은 외곽 660 × 660 뿐이다.

### 아직 확인하지 않은 것

유도한 EPAL 오프셋을 기존 객체에 대입해 seed 0·7·12·17 의 시나리오가 **기존과 동일함**을 읽기
전용 계산으로 확인했다. **전체 planner 회귀는 다시 돌리지 않았다.** T11 은 예약 구간과 외곽이
달라지므로 같은 seed 라도 배치가 달라질 수 있다.
