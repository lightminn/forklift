# 밑판 옆면 `structure` 검사를 riib 운영 기본으로 — 계획 (2026-11-18)

총괄 배정(2026-11-18, 사용자 "모두 해"): CLAUDE.md 에 "결정됐으나 안 만듦"으로 적힌 `--bottom-deck-check structure` 를 계획(Codex 수렴) → 구현(러너 PR 이라 재생 게이트) → 운영 런처 기본 순서로 만든다. 시간 비용이 커지면 범위를 줄일 안을 함께 보고한다.

**총괄 결정(2026-11-18): 이 계획은 단계 1 만 한다.** 단계 1 은 batch 장면 경계 수정, riib 런처 기본 `structure`(옆면을 볼 수 없는 장면은 끔), Isaac S+·S− 다.

- 보류: D-F′ 면별 면제와 live·대기열(콘솔 런처)이다. 실차 인식이 더 급하기 때문이다(§6).
- 시간 비용을 줄이는 안(§5)은 단계 1 결과와 함께 판단한다.
- 총괄 조건:
  - 장면 상자를 넓힌 뒤 장면 도구 `--plan` 결과가 24 장면 모두 같을 때만 원본 장면에 적용하고, 하나라도 다르면 넓힌 사본으로 한다.
  - 재생 게이트 29/29 는 그대로 요구한다.
  - S− 선반 시험체는 작게 되면 넣고, 커지면 CPU 결과로 갈음한다(판단 위임).

앞선 결정은 [하판 없는 팔레트 계획](2026-10-27-bottomless-and-bundled-pallets.md)의 D-F·D-F′·D-G, §2.1 g·g′, §3.3, §7, §8 의 PR ⑤·⑥ 이다.

## 0. 지금 있는 것과 없는 것 (main `25631735`)

| 항목 | 상태 | 근거 |
|---|---|---|
| 러너 인자 `--bottom-deck-check {off,on,structure}`, 기본 `off` | **있음**(PR ②, `22eb0f66`) | `run_transport.py` 1071 행. CLAUDE.md·AGENTS.md 의 "러너는 `off`/`on` 만 받는다"는 낡은 문장이다 |
| `structure` 판정: 서로 다른 두 면이 각각 present 또는 absent 일 때만 허가, 첫 면이 absent 여도 둘째 면을 봄 | 있음 | `bottom_deck_permit.two_face_verdict`·`insert_refusal`(mode) |
| 옆면 단계: 두 촬영 위치·복귀까지 세 경로를 **먼저 모두** 계획하고 하나라도 실패하면 미션 전체를 거부(`bottom_deck_unobserved`, `stand_unreachable`) | 있음 | `core._bottom_deck_begin` |
| 실행 범위 | batch 한 미션 전용 | `--use-perception`·`--planning-target perception` 필수. `--near-tracking`·`--pocket-check`·`--live`·`--repeat-captures`·`--near-recapture-gap-m`·세션 장면 파일과 함께면 인자 오류(`run_transport.py` 1720–1733, 1960, 2084 행) |
| 켜면 바뀌는 다른 입력 | 인식 카메라의 근거리 절단 1.0 → 0.28 m(미션 전체) | `run_transport.py` 4739 행. 옆면 이동 전의 원거리 단계도 입력이 달라진다 |
| D-F′ 면별 면제, 사람 확인·무인 건너뛰기, live·세션 생명주기 | 없음 | 보류(§6) |
| Isaac 확인(§7 S+·S−) | 없음 | |
| Isaac 미션용 선반 시험체 | 없음 | `--pallet-article` 은 `nominal`·`grounded` 만. 러너의 이름 붙은 충돌 상자 검사(`insertion_geometry.assert_pallet_urdf_matches_named_boxes` 415 행)도 둘만 안다. 선반 모양은 CPU 상자(`tools/negative_structures.py` 의 `shelf_closed`)로만 있다 |
| 운영 런처 기본 | 둘 다 `off` | `riib_scene.sbatch`, `isaac_live.sbatch` |

## 1. 운영 장면 점검 (CPU)

[`side_census.py`](2026-11-18-bottom-deck-structure-default/side_census.py)(`--plan`)는 riib 장면 18 개마다 옆면 단계를 본다. 18 개는 plain·`_m` line/random 12 개와 `aisle`·`aisle_out`·두 `_rack`·`trailer`·`conveyor` 다. 보는 것은 셋이다.

- 집을 팔레트의 두 옆면 촬영 위치(`bottom_deck_legs.stand`, x 가설)가 다른 물체와 겹치는지;
- 카메라에서 옆면 가운데까지의 시선을 무엇이 가리는지;
- 코어가 하는 대로 원거리 관측 지점 → 첫 옆면 → 둘째 옆면 → 복귀를 장면 도구의 계획기 설정으로 계획한 결과.

결과는 [`side_census.json`](2026-11-18-bottom-deck-structure-default/side_census.json)이다.

| 장면 | 옆면 촬영 위치 | 세 경로 | 지금 코드로 켜면 |
|---|---|---|---|
| `random_2`·`random_3` (+`_m`) | 두 면 다 비어 있음 | 모두 성공 | 두 면을 보고 진행 |
| `line_1`·`line_2`·`line_3`·`random_1` (+`_m`) | 두 면 다 비어 있음 | 한 경로가 `invalid_goal` | **거부**(`stand_unreachable`). 한쪽 촬영 위치가 장면 상자(`scenario.bounds`) 밖이다. 남쪽 홀 안이고 벽은 없다 |
| `aisle`·`aisle_out`·`aisle_rack`·`aisle_out_rack` | 두 면 다 이웃 랙 팔레트와 겹침(칸 사이 간격 plain 0.20 m, `_rack` 0.58 m). 카메라는 팔레트 중심에서 1.35 m, 뒤차축 목표점은 약 2.31 m 이다. `aisle_out_rack` 은 한 면 시선을 랙 기둥이 가림 | 실패 | **거부**. D-F′(보류)가 있어야 열린다 |
| `trailer` | 두 면 다 트레일러 벽 | 실패 | **거부**. D-F′ 로도 비팔레트라 사람 확인 |
| `conveyor` | 한 면 컨베이어 | 실패 | **거부**. 위와 같음 |

이 점검의 범위는 **정적 경로의 필요조건**까지다.

- riib 장면 실행은 사전 지도(장면 props)로 계획하고 `--grid-planning` 을 쓰지 않는다. LiDAR 장애물 층은 주행 허가에 따로 작용한다.
- 점검은 x 가설만 본다(EPAL 6 의 등록 진입축은 x 하나).
- 인식 추정 자리에 참 팔레트를 넣었다.
- 원거리 정지 자리는 장면 도구가 처음 닿는 관측 후보다.
- 실제 인식 자세, 정지 위치, 옆면 프레임 판정은 Isaac 에서 본다(§3).

## 2. 장면 상자를 어떻게 넓히나

러너는 장면 상자(`scenario.bounds`)를 홀로 쓴다.

- 계획기의 차체 검사, 옆면 단계 경계(`side_bounds` 의 hall), LiDAR 장애물 층의 격자(`ObstacleLayer(hall=scenario.bounds)`, `run_transport.py` 5158 행), 계획 메모리, 오버뷰 카메라, 기록의 `hall` 이 모두 이 상자다.
- 그래서 옆면 단계만 경계를 넓히면 그 밖은 격자에 없다. 장면 상자 자체를 넓혀야 한다.

**홀 좌표에서 +y 만 넓힌다.**

- 장면 생성기의 상자는 `to_hall()` 에서 −90° 돌고, 동쪽 끝(장면 `y_max` → 홀 x 4.6)과 출발점(홀 y 0)에 맞춰 옮겨진다. 그래서 생성기의 `y_max` 여유를 늘리면 장면 전체가 옮겨진다(Codex 재현: line_1 출발 x −2.952 → −4.352 m).
- 홀 +y 는 생성기의 `min(xs)` 쪽(`bounds.y_max = −b.x_min + ty`)이고, 평행이동은 `b.y_max` 와 출발점에만 달려 있다. 그래서 `to_hall()` 결과의 `bounds.y_max_m` 만 늘리면 팔레트·출발점·props·visuals 는 그대로다.

**필요한 양**([`need_margin.py`](2026-11-18-bottom-deck-structure-default/need_margin.py)): 옆면 촬영·후퇴 위치의 차체 윤곽 + 계획 여유 0.10 m + 이동 여유 0.3 m 가 들어가는 상자다. 팔레트를 ±0.15 m(대조 허용)·±5° 움직인 최악값도 함께 본다. 필요한 쪽은 모두 홀 +y 다.

| 장면 | 정위치 | ±0.15 m·±5° 최악 | 넣을 값 |
|---|---:|---:|---:|
| line_1·2·3 | +1.12·+1.35·+1.25 m | +1.39·+1.59·+1.51 m | +1.7 m(홀 y_max 1.60 → 3.30) |
| random_1 | +0.73 m | +1.02 m | +1.1 m(3.05 → 4.15) |
| random_2·3 | 0 | 0·+0.10 m | +1.1 m(같은 시퀀스라 함께) |

넓힌 상자는 남쪽 홀(y ≤ 8.3) 안이다. 생성기에 이 홀 경계 검사가 이미 있다.

**넓힌 사본으로 한다(총괄 조건).** Codex 가 장면 도구 `--plan` 으로 넓힌 상자를 계산했다. 검사 끔에서도 `line_3`·`line_3_m` 의 복귀 길이가 17.04 → 16.36 m 로 바뀌었다. 그래서 원본은 두고 사본을 만든다.

- 장면 도구에 `--side-room` 을 더한다. line·random 마다 `<name>_side.json` 사본을 쓰고, `_m` 사본에는 `<name>_m_side.json` 을 쓴다. 사본의 이름(`name`)도 `_side` 가 붙는다. 사본은 홀 상자 +y 만 위 값만큼 넓히고, 나머지 필드는 원본과 같다.
- 바이트 비교로 확인할 것:
  - 원본 장면 파일 18 개가 바뀌지 않는다.
  - 콘솔 세션 장면(`line_m_session.json`·`random_m_session.json`)도 바뀌지 않는다. 이 둘은 지도 묶음 레지스트리의 해시로 묶여 있고, 바뀌면 콘솔이 매핑으로 바뀌거나 멈춘다.
- 사본 안 팔레트·출발점·props·visuals·관측 후보가 원본과 같은지 시험으로 고정한다.
- `PICKUP_ERRORS` 조회는 `_side` 도 떼어 원본과 같은 오차를 쓴다.
- 장면 요구 사항(`config/scene_requirements.yaml`)은 장면 파일의 `name` 으로 찾는다. 지금 항목은 `trailer` 뿐이라 사본과 겹치지 않는다.
- 근접 추적 계약 파일(`config/near_field_*_riib_line_1_m.json`)은 입력 해시로 원본을 묶는다. `structure` 는 근접 추적과 함께 쓰지 못하므로 사본과 엮이지 않는다.

## 3. 단계 1 의 할 일

1. **장면 도구:** `--side-room` 과 사본, 시험.
   - 사본 좌표 불변, 홀 안, 이름·오차 조회.
   - 원본과 세션 파일 바이트 불변.
   - `side_census` 기준으로 12 사본 모두 세 경로 성공. Codex 가 같은 조건에서 12 변형 모두 성공을 확인했다.
2. **riib 런처**(`deploy/riibotics/riib_scene.sbatch`):
   - `SIDE=on|off`(기본 `on`)를 더한다. `MARKERS`·`RACK` 처럼 line·random 장면을 `_side` 사본 이름으로 바꾸는 스위치다.
   - `BOTTOM_DECK=structure|on|off` 를 더한다. 런처가 정하는 기본은 아래 조건을 모두 만족할 때만 `structure` 이고, 아니면 `off` 와 그 이유를 `riib_defaults.txt` 에 적는다.
     - 장면이 `_side` 사본이다.
     - `MODE=slam` 이다.
     - `NEAR_RECAPTURE_GAP` 이 비어 있다.
     - `EXTRA_ARGS` 에 `--near-tracking`·`--pocket-check`·`--repeat-captures`·`--live` 가 없다.
   - `BOTTOM_DECK=structure` 를 사람이 직접 줬는데 위 조건과 맞지 않으면, Isaac 을 띄우기 전에 거부한다.
   - 장면별 허용 목록이 최종 `NAME` 으로 정해지는 곳(`line_2_m` 의 degenerate-continue, `aisle_rack` 의 short cusp·keepout, `trailer` 의 exit clearance)은 `_side` 를 뗀 이름으로 비교한다. 그래야 사본에서도 같은 기본값이 붙는다.
   - 시험: 런처 인자 조합(`DRY_RUN`/러너 `--check-args`)으로 18 장면 × `MODE=slam/truth`, `NEAR_RECAPTURE_GAP`, `EXTRA_ARGS=--near-tracking` 의 최종 인자와 거부를 확인한다. Codex 가 `line_2_m`·`random_2_m` 기본 SLAM 인자 + `structure` 를 `--check-args` 로 돌려 `ARGS_OK` 를 확인했다.
3. **선반 시험체(S−)를 넣는다.**
   - `tools/build_grounded_pallet.py` 처럼 `negative_structures` 의 `shelf_closed` 상자로 URDF 를 쓰는 도구를 더한다.
   - 상자 검사 쪽에 `shelf_closed` 상자 목록을 연결하고, `--pallet-article` 에 `shelf_closed` 를 더한다(평가 전용, 러너 기본 불변).
   - 러너가 팔레트 URDF 의 상자 이름으로 다른 값(지지대·접촉 판정)을 읽는 곳이 미션 시작 전에 깨지면, 이 일은 커진 것으로 본다. 그때는 S− 를 PR ③ CPU 결과(닫힌 선반 넷 72/72 거부)로 갈음하고 그 사실과 이유를 기록에 적는다.
4. **Isaac 확인.** 운영 기본(learned 원거리, 결정 모드 아님)으로 돌린다. 결정 모드는 learned 제안을 oracle 로, 픽셀 지연을 0 으로, SLAM 정적 층을 끔으로 바꿔 운영 스택을 보지 못한다. 벽시계 측정이 아니므로 다른 잡과 겹쳐도 된다. 크기는 `tools/isaac_bundle.py sizes`·`check` 로 정하고 제출 전에 총괄에 알린다.

| 묶음 | 장면·시험체 | 실행 | 합격 / 기록 |
|---|---|---|---|
| S+ | `line_2_m_side`, `random_1_m_side`(둘 다 경계 수정이 필요한 장면), 진짜 EPAL 6 | seed 0–2, `structure` 와 `off` 짝(같은 사본) | 장면마다 3 회 모두 두 면 판정까지 가서 허가, 완주 ≥ 2. 단계별 시각 기록 |
| S+g | `random_1_m_side`, grounded EPAL 6 | seed 0–2, `structure` | 3 회 모두 두 면 판정 `absent`/`present` 로 허가(`no_bottom_deck` 0), 완주 ≥ 2 |
| S− | 이웃 팔레트 없는 배치(기본 bay 또는 이웃 없는 장면 — 이웃은 미션 시험체를 복제하므로), `shelf_closed` | seed 0–2(원거리에서 걸러진 실행은 따로 적고 다음 seed 로, 최대 6) | 각 실행이 원거리 채택 → 옆면 단계 → 저장 프레임 재계산에서 `closed_face`(`unobserved`) → 그 판정으로 거부 → `insert` 전환 없음 |

   - 단계별 시각: 첫 원거리 채택, 옆면 이동·촬영, 복귀·재관측, 첫 접근 시작, 완주.
   - 근거리 절단 변화(§0)는 옆면 이동 전 원거리 단계에서도 차이를 만들 수 있다. 그래서 첫 원거리 채택 시각까지의 차이는 옆면 비용과 따로 적는다.

5. **회귀:**
   - 재생 게이트 29/29: 옛 기록 재생이라 `structure` 기록은 거부되고, 기존 경로의 회귀만 본다.
   - ws1 전체 pytest.
   - S+ 의 learned 실행이 바뀐 런처 경로를 지나는 smoke 를 겸한다(CONTRIBUTING §7).
6. **문서:** CLAUDE.md·AGENTS.md 의 같은 문단을 같이 고친다.
   - 러너는 `off`/`on`/`structure` 를 받고 기본은 `off` 다.
   - riib 런처는 `_side` 사본에서 `structure` 를 기본으로 쓴다(Isaac 확인 뒤).
   - D-F′·콘솔 적용은 보류다.
   - 결과는 `docs/validation/` 에 적는다.

머지 순서: (1)–(3) 코드와 시험을 한 PR 로 낸다. 이 PR 에서 런처 기본은 아직 `off` 로 둔다. Isaac (4) 결과로 합격하면 런처 기본을 켜는 작은 PR 과 결과 문서를 낸다. 불합격이면 기본을 켜지 않고 원인과 함께 총괄에 보고한다.

## 4. 시간 비용

2026-10-21 검사 켬 기록은 공장 홀, 옛 차체 설정, `on` 모드다. 끝난 사이클이 325–391 s(끔 214–259 s, 시뮬 시간)였고, 첫 접근 시작은 140–205 s(끔 20–40 s)였다. 이 근사로 늘어나는 시간은 사이클당 약 110–130 s(약 +50 %)다. 그러나 이번 riib 장면·`structure` 에서 다시 잰다. 어느 구간을 줄일지는 S+ 의 단계별 시각으로 정한다.

## 5. 시간 비용이 클 때 줄이는 안 (결정 아님)

| 안 | 줄어드는 것 | 잃는 것 | 종류 |
|---|---|---|---|
| ㉠ **한 면만 본다**(먼저 닿는 면) | 옆면 다리 하나와 그 이동. 얼마나 주는지는 S+ 의 구간 시각으로 정한다 | 한쪽만 막힌 선반은 못 거른다. 전 깊이로 막힌 선반은 어느 면에서나 `closed_face` 라 걸린다(PR ③ CPU 의 72/72 는 면마다 판정한 값) | D-F 계약(두 면 필수) 변경, 총괄 결정 |
| ㉡ **복귀 재관측을 근거리 재관측으로 대신한다** | 첫 원거리 지점으로 돌아가는 이동 | 옆면 뒤 SLAM 연결 오차(PR #72 가 막은 것)가 근거리로 넘어갈 수 있다. 지금 재관측이 하는 목표 대조(형상·0.15 m·15°)를 접근 계획 **전에** 어디서 할지 새로 정해야 한다 | 흐름 변경 |
| ㉢ **같은 팔레트 열은 한 번만** | 연속 이송에서 이미 확인한 열의 다음 팔레트 | 열 안에 다른 물체가 섞이면 놓친다 | 판정 정책 변경. 단계 3(보류)에서만 의미 있음 |
| ㉣ **장면별 끔 유지** | 검사 비용 0 | 그 장면의 선반 방어도 함께 없다 | 지금 `aisle*`·`trailer`·`conveyor` 처리 |

## 6. 보류한 단계 (재개할 때를 위한 기록)

- **단계 2 — D-F′ 면별 면제.** [§3.3](2026-10-27-bottomless-and-bundled-pallets.md) 의 판정 계약 그대로다.
  - 면마다 관측 판정은 그대로 두고, 면제는 정책 기록(`waived: pallet_row` + 근거 블록)으로 따로 붙인다.
  - 이번 미션의 원거리 `fitted`·`confirmed`·바닥 후보가 가린 면만 면제한다.
  - 한 면 면제 + 반대 면 `closed_face` 는 거부다.
  - 옆면 계획을 "먼저 모두 계획"에서 면별 계획으로 바꾼다.
  - 이것이 있어야 `aisle*` 를 켤 수 있다.
- **단계 3 — live·세션·대기열.** §2.1 g′ 의 생명주기, 사람 확인과 무인 건너뛰기(§8 대기열 계약), §7 C 와 콘솔 런처 기본이다. 콘솔 세션과 함께 정한다.

## 7. 묻지 않고 진행하는 것과 올릴 것

- 진행(문서·보고): 장면 사본, 런처 스위치·기본·장면별 끔, Isaac 확인 범위, 선반 시험체, CLAUDE.md·AGENTS.md 수정.
- 총괄 결정: ㉠ 한 면(D-F 계약), 단계 2·3 재개, S+ 결과 시간 비용을 받아들일지, S− 를 CPU 로 갈음할 때.

## 8. 자체 검토 (독립 검토 3 회)

### ① 원자료 재계산 (2026-11-18)

- 시간 비용 수치(325–391 s, 끔 214–259 s, 첫 접근 140–205 s, 끔 20–40 s)를 [2026-10-21 기록](../validation/2026-10-21-bottom-deck-cycle.md) 12 행에서 다시 읽었다. 같다. 다만 그 기록은 공장 홀·`on` 모드라 근사로만 쓴다(§4).
- 랙 칸 사이 간격을 `tools/riibotics_isaac_scenes.py` 에서 다시 읽었다. plain 0.20 m(`aisle_scene` 기본), `_rack` 0.58 m(`RACK_SLOT_GAP_M`). 초안의 "간격 0.20 m" 를 고쳤다.
- 장면 상자 여유를 `line_defs`·`random_defs` 에서 다시 읽었다(1.6·1.2 m).
- 장면 상자가 러너에서 홀로 쓰이는 곳은 `run_transport.py` 2877·5158 행에서 확인했다.

### ② Codex 적대적 검토 (2026-11-18, 스레드 01a1ef2a)

| 등급 | 지적 | 확인 | 처리 |
|---|---|---|---|
| P1 | 생성기 `y_max` 여유를 바꾸면 장면이 옮겨지고 옆면은 안 열린다. `to_hall()` 결과의 `bounds.y_max_m` 만 넓히면 열린다. 그래도 `line_3`·`_m` 은 끔에서도 계획이 바뀐다 | `to_hall()` 452 행 읽음. 홀 +y = 생성기 −x 쪽, 이동은 `b.y_max`·출발점에만 의존 | §2: 홀 상자 +y 만, 넓힌 사본 |
| P1 | 장면 이름만으로 `structure` 를 붙이면 `MODE=truth`·`NEAR_RECAPTURE_GAP`·`--pocket-check`·`--repeat-captures` 실행이 깨진다 | 러너 1720·1960·2084 행 | §3-2: 조건이 다 맞을 때만 기본, 명시 요청은 사전 거부, `riib_defaults.txt` 에 최종값 |
| P1 | 선반 URDF·CLI 만으로는 상자 검사기가 거부한다. 이웃은 미션 시험체를 복제하니 이웃 없는 장면이어야 한다. CPU 로 갈음하면 범위 변경으로 올려야 한다 | `insertion_geometry.py` 415 행, `scene.py` 257 행 | §3-3·§3-4 S− |
| P2 | 점검 분모는 18 이고 성공은 4. 범위는 정적 경로 필요조건(사전 지도, grid planning 없음, x 가설) | `side_census.json` | §1 |
| P2 | 경계 변경은 격자·기록·해시에 퍼진다. 세션 장면 해시가 레지스트리와 묶여 있다 | `slam_choice.sh` 32 행 | §2: 원본·세션 바이트 불변 확인 |
| P2 | 결정 모드 짝 비교는 운영 스택을 보지 못한다. `random_2_m` 대신 경계 수정이 필요한 `random_1_m`. grounded 는 3 seed | 러너 1734 행 | §3-4 |
| P2 | 시간 표는 근사이고, 한 면 절반 절약 근거 없음. 근거리 절단 변화를 옆면 비용에 섞지 말 것 | 러너 4739 행 | §4·§5 |
| P3 | 1.35 m 는 카메라 거리, 뒤차축 목표는 약 2.31 m. CLAUDE.md·AGENTS.md 같이 | | §1·§3-6 |

### ③ 다른 관점 독립 검토 — agy(Gemini 4 Hydrogen High), 2026-11-18

질문은 "적히지 않은 전제"였고, ② 의 결과는 보여 주지 않았다.

| 지적 | 확인 | 처리 |
|---|---|---|
| 결정 모드는 learned 를 oracle 로, SLAM 정적 층을 끔으로 바꿔 운영 스택을 시험하지 못한다 | ② 와 같은 결론 | §3-4: 운영 기본으로 |
| CPU 72/72 는 Isaac 0.28 m 절단 렌더·learned 채택·상자 검사를 거치지 않는다 | 맞음 | §3-3: 시험체를 넣는 쪽을 먼저, 못 하면 기록 |
| 생성기 여유가 `to_hall()` 회전 뒤 어느 쪽인지(장면 이동), 세션 장면이 함께 바뀜 | ② P1 과 같음 | §2 |
| `structure` 와 런처 인자 비호환, 근거리 절단이 미션 전체를 바꿈 | 러너 4739 행 | §0·§3-2·§3-4 |
| 정위치만 보면 여유가 빠듯하다(line_2 0.05 m) → 팔레트 ±0.15 m·±5° 로 다시 | `need_margin.py` 에 넣어 다시 계산: 최악 +1.59 m | §2 표: +1.7·+1.1 m |
