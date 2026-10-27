# 채택 단계 구역 검사 (총괄 결정 2026-10-28)

관련: `docs/plans/2026-10-27-bottomless-and-bundled-pallets.md` §3.3(다리형 선반 받아들인 위험, PR #191). 받아들인 위험은 방어 ①이 실제로 성립할 때만 유효하다. 지금은 원거리 후보 선택만 지시 반경(외접원)에 묶여 있다. 근거리 캡처·재관측·추적 재작도는 구역 검사 없이 추정을 계획 대상으로 채택한다.

## 1. 결정

- **검사:** 팔레트 추정이 계획 대상으로 채택되기 직전마다, 채택될 **유효 목표**(`hooks.planning_target` 을 거친 값)의 중심 (x, y) 가 **미션 채택 사각형 + 여유 m** 안인지 본다. 밖이면 그 미션을 `adoption_outside_zone` 으로 끝내고 기록한다. 다른 후보로 넘어가지 않는다. 기록에는 유효 목표와 원시 추정(`target_pickup`)을 함께 남긴다.
- **원거리 선택기는 그대로 둔다.** `select_target` 의 원(유일성·`multiple_in_radius`·`reobserve`)은 바꾸지 않는다. 원 안·사각형 밖 후보가 뽑히면 채택 검사에서 거부된다. 그 결과 사각형 안 팔레트를 끝내 못 집는 경우가 생길 수 있다(원 모서리에 다른 물체가 있을 때 `ask_operator`·거부). 이것은 가용성 한계이고 안전성은 나빠지지 않는다(Codex CPU 확인). 사각형 밖 확인 후보를 선택 전에 빼는 개선은 후속 항목으로 둔다.
- **콘솔 경로(`--recognition off`) 문서 문구:** 이 경로는 장면이 준 팔레트 자세(사전 정보)와 기하 검출기로 집는 **시뮬 전용 경로**다. 실차는 인식 경로(`--recognition far`)를 쓴다. 구역 검사는 채택 지점에 있으므로 이 경로에도 똑같이 걸린다.

## 2. 미션 채택 사각형

미션을 준비할 때 한 번 정하고, 미션 기록(`state["adoption_zone"]["zone"]`)에 남긴다. 모두 지도 축에 맞춘 **팔레트 중심 범위**다. 장애물용 `zone_rect`·러너의 `pickup_zone`(팔레트 반대각선만큼 넓힌 사각형, `scene_file.py:117`, `run_transport.py:3863`)은 쓰지 않는다. 그것에 m 을 또 더하면 허용 범위가 사방으로 0.5 m 넘게 커진다.

| 경우 | 사각형 |
|---|---|
| 배치 실행, 장면 파일 있음 | `scene_layout.pickup_zone` 의 `[x_min, x_max] × [y_min, y_max]` |
| 배치 실행, 장면 파일 없음(bay·factory) | `planning/observation_viewpoints.BAY_PICKUP_ZONE` 의 중심 범위(러너 `_ZONE`, `run_transport.py:3857`) |
| 세션, 팔레트가 처음 자리에 있음 | 그 팔레트의 `SessionZone` 중심 범위 |
| 세션, 하역된 팔레트를 다시 집음(`delivered`, 처음 자리 아님) | 내려놓은 중심을 가운데로 하는 정사각형, 반폭 0.10 m(Isaac 하역 오차 0.02–0.10 m, `docs/validation/2026-10-22-session-scene-pallet-props.md:28`). 내려놓은 중심은 **정상 하역 때 따로 남기는 스냅숏**(`put_down_xy`: 그 하역의 목적지 추정, `core.py:5261` 에서 `TargetsBook` 에 쓰는 값)이다. `TargetsBook.pallets[pid].pose` 를 그대로 읽지 않는다. 그 값은 재집기 관측 뒤 새 인식 추정으로 바뀌는데 `state="delivered"` 는 남아(`core.py:5266`, `targets.py:88`) 실패 뒤 재제출 때 마지막 내려놓은 중심이 아니게 된다. truth 실측 중심도 아니다 |
| 그 밖(복구 뒤 재제출 등으로 중심을 모름) | 사각형을 정하지 않는다. 검사 켬이면 그 미션은 시작 전에 `adoption_zone_undefined` 로 거부된다 |

yaw 범위는 기록만 하고 거부 조건에 넣지 않는다. 지도 축 사각형은 구역 정의 그대로다(`SessionZone` 의 `yaw_centre_rad` 는 heading 범위의 중심이지 사각형의 회전이 아니다).

## 3. 채택 지점 (모두 `src/forklift_core/mission/core.py`)

| 단계 | 함수 | 검사할 값과 순서 |
|---|---|---|
| 원거리 관측·근거리 캡처·재관측(retry·realign 자리) | `observe_succeeded` (3199) | `target_pickup` 을 `hooks.planning_target` 에 넣은 유효 목표. `target_pickup` 계산(3244) 직후, 옆면 단계 대조(3296·3302)·`set_inset_estimate`·`known_pallet` 등록·옆면 주행 시작(`_bottom_deck_begin`)보다 **먼저**. 거부면 그 뒤 부작용은 하나도 일어나지 않는다 |
| 둘째 근거리 캡처(C3) | `second_near_capture` (4333) | `new_pickup_`(4369). 새 캡처가 박스·dry-run 을 통과해 **실제로 채택될 때만** 검사한다. 채택되지 않고 옛 경로로 돌아가는 경우는 검사하지 않는다(새 추정을 쓰지 않으므로) |
| 추적 기반 재작도 | `near_redraw` (8361) | `target_`(8386, `G2.planning_target` 뒤). `near_install` 전 |
| 삽입 전 대조 | `approach` → `insert` 전이(504–531) | `pocket["planning_pickup"]`. 근접 추적을 켠 실행은 그 시점 최신 추적 관측(`latest_`)의 중심도 검사한다(삽입은 추적 포켓을 따라가므로). `bottom_deck_on()` 과 무관하게 돈다 |

`near_capture_redraw`(8421)는 채택 지점이 아니다. 재작도는 이미 채택된 `pocket["planning_pickup"]`(8443)을 쓰고, `raw_`(8445)는 진단 기록에만 쓰인다. live 의 `observation_take` 결과도 `observe_succeeded` 로 들어간다(`run_transport.py:8285`, Codex 확인).

검사는 순수 함수(`forklift_core/mission/adoption_zone.py`: `check(site_xy, zone, margin_m) -> record`)로 두고 위 네 곳에서 부른다. 통과도 기록한다(단계·유효 목표·원시 추정·사각형·m·경계까지의 부호 거리). 비교는 계획기가 쓰는 같은 제어 좌표(SLAM hold 중이면 hold 된 지도 추정)에서 한다. hold 동안의 오도메트리 드리프트는 4 의 오차 예산에 들어간다.

## 4. 거부 처리 계약

- 사유 `adoption_outside_zone`(과 `adoption_zone_undefined`)을 `forklift_core/mission/failures.py` 에 **MISSION** 으로 등록한다. 미등록 사유는 SESSION 으로 분류되어 live 세션 전체가 끝난다(`failures.py:88`, `run_transport.py:9034`).
- `state["adoption_zone"]` 을 `baseline.py` 의 `MISSION_KEYS["state"]` 에 넣는다. 넣지 않으면 기록이 미션 결과로 넘어가지 않고 다음 미션까지 남는다(Codex 가 capture → 추가 → restore 로 재현).
- 미션은 다른 미션 실패와 같은 길로 끝난다(`require(False, ...)`). 대기열은 다른 미션 실패 때와 같이 동작한다. 거부된 팔레트는 자동으로 다시 제출하지 않는다.
- 남는 위험: 삽입 직전에 거부되면 로봇이 대상 바로 앞에 서 있다. 다음 미션 준비가 `invalid_start` 로 막힐 수 있다. 시험 3 에서 확인하고, 막히면 따로 다룬다.

## 5. 여유 m

중심 추정 오차는 로봇 자세 오차 + 인식 오차 + 진입 yaw 오차가 중심으로 옮겨 가는 몫이다. 지금 근거는 다음뿐이다.

- 원거리 관측 중 미션별 로봇 최대 위치 오차: 108 미션 중앙값 6.4 cm, p95 19.3 cm, 최대 21.8 cm(`docs/validation/2026-10-24-far-target-occupancy.md:53`, 잡 10657). 팔레트 중심 오차의 상한은 아니다.
- 근거리가 원거리보다 나빠진 미션이 있다(line_m s0 진행축 −0.4 → +3.9 cm, `docs/validation/2026-10-24-d1-clearance-overlap.md:4`).
- 인식 중심 오차 ≤ 3.7 mm 는 Isaac 오라클 분할의 값이다. 학습 제안기·기하 검출기·실차 D435i 의 값이 아니다.

그래서 m 은 **기존 기록에서 먼저 잰다**(시험 0). 그때까지 구현의 기본값은 임시 0.25 m 다.

- **시험 0(dev, 판정 규칙을 정한 뒤 실행):** ws1 에 남아 있는 Isaac 기록(근접 추적·C3·live 세션 포함, 목록은 실행 전에 이 문서에 적는다)에서 각 채택 지점의 유효 목표와 같은 시각 팔레트 truth 중심의 차이를 축별로 모은다. 단계별 최대와 p99 를 적는다. **m = 전 단계 축별 최대를 5 cm 단위로 올린 값**으로 정한다. 그 m 이 0.35 m 를 넘으면 구현을 멈추고 총괄에 올린다(그만큼 넓은 사각형은 이웃 칸을 덮어 방어가 약해진다).
- m 은 `--adoption-zone-margin-m` 으로 바꿀 수 있다. 실차 SLAM·D435i 중심 오차를 잰 뒤 다시 정한다.
- m 이 커질수록 현장 규칙("사각형 + m 안에 선반을 두지 않는다")이 미치는 공간도 커진다. 오차 방향에 따라 사각형 밖 선반의 추정이 사각형 + m 안으로 들어올 수도 있다. 이것은 받아들인 위험의 범위에 든다.

## 6. 인자·기본값

- 러너 `--adoption-zone-check {off,on}`(기본 `off` — 인자 없는 옛 기록의 읽기), `--adoption-zone-margin-m`. `sim/isaac/near_tracking.py` 의 RUN_SPECIFIC 계약에 등록한다(등록하지 않으면 근접 추적 실행이 시작되지 않는다).
- 운영 런처(콘솔 `isaac_live.sbatch`, 리보틱스 `riib_scene.sbatch`)의 기본은 8 의 Isaac 확인을 통과한 뒤 `on` 으로 바꾼다.

## 7. 시험 (먼저 쓴다)

1. `adoption_zone.check`: 경계 안·경계 위·여유 안·여유 밖, 원 안·사각형 + m 밖 모서리 점 거부, NaN·빈 사각형은 오류. 2 의 사각형 고르기(처음 자리·재집기·장면 없음·모름)도 시험한다.
2. 미션 코어 CPU 시험: 네 채택 지점마다, 검사 켬에서 구역 밖 유효 목표가 `adoption_outside_zone` 으로 끝나고, `observe_succeeded` 에서는 그 뒤 부작용(`known_pallet`·옆면 주행·inset)이 없다. 검사 끔이면 지금과 같다(재생 게이트 기록 동일). C3 는 채택될 때만 검사한다.
3. live 계약: 거부 뒤 세션이 살아 있다(`classify` = MISSION). 대기열은 멈춘다. 명시적 새 제출은 받는다. 다음 미션의 `state["adoption_zone"]` 은 비어서 시작한다.
4. CPU 리그 반례: `shelf_legs` 를 사각형 밖·외접원 안(1.0 × 1.7 m 구역 모서리 쪽)에 두면 원거리 선택은 뽑지만 채택에서 거부된다.

## 8. Isaac 확인 (판정 규칙은 결과 전에 고정)

기능 확인이다. 오거절률의 상한 근거가 아니다.

- **장면·팔:** 검사 켬, m 은 시험 0 에서 정한 값. (a) 콘솔 live line_m·random_m 각 3 seed, 세션마다 처음 자리 집기와 하역 뒤 재집기 미션을 하나 이상 포함 (b) 리보틱스 `NEAR=on`(근거리 캡처 기대 위치·`near_capture_redraw` — 근접 추적은 켜지 않는다, `riib_scene.sbatch:199`) aisle_rack 포함 3 seed (c) C3(둘째 근거리 캡처)를 켠 팔 3 seed (d) **근접 추적 팔** 3 seed — `near_redraw` 와 삽입 전 `latest_` 검사는 `--near-tracking` 에서만 돈다. 리보틱스 `recognition far` 와 C3 는 `--near-tracking` 과 함께 쓰면 러너가 거부하므로(`run_transport.py:1373`, `1287`), (B) 근접 추적 재측정과 같은 line_1_m 구성(호환되는 bounds·drive terms)을 쓴다.
- **분모:** 지시된 진짜 팔레트에 대해 `observe_succeeded` 채택 검사에 도달한 미션. 단계별로 도달한 미션 수를 적는다. 도달 전에 끝난 미션(관측 실패·조기 종료)은 분모에서 빼고 따로 센다. 채택 기록이 없거나 깨진 미션은 판정 불가로 센다(통과 아님). 단계별로 최소 1 미션이 도달해야 한다. 도달하지 못한 단계는 "Isaac 미확인"으로 적는다.
- **오거절:** 지시된 진짜 팔레트(truth)에 대한 `adoption_outside_zone`. 구역 밖의 다른 물체·팔레트를 거부한 것은 올바른 거절이다.
- **판정:** 네 채택 단계가 모두 최소 1 미션 도달했고, 판정 불가 기록이 없고, 오거절이 0 이면 런처 기본을 `on` 으로 바꾼다. 도달하지 못한 단계가 있으면 바꾸지 않고 그 단계의 팔을 다시 짠다. 1 건이라도 나오면 바꾸지 않고 그 사례의 오차를 분해한다. 결과를 보고 m 을 올리지 않는다.
- 기록: 모든 채택 기록의 경계까지 부호 거리 분포(최대값)를 문서에 적는다. 크기는 `isaac_bundle` 계산(GPU 당 3 세션)을 따른다.

## 9. 남는 위험·전제

- 사각형 + m 안에 다리형 선반이 있으면 여전히 막지 못한다(현장 규칙 전제).
- yaw 는 검사하지 않는다.
- 실차 전제(재지 않음): 구역이 서베이 지도 좌표로 주어진다는 것, 그 정의 오차, 작업자가 구역선 밖에 내려놓은 팔레트. 이런 오차가 m 을 넘으면 정상 팔레트를 거부한다.
- 하역 뒤 재집기 사각형의 반폭 0.10 m 는 Isaac 하역 오차에서 온 임시값이다.

## 자체 검토

- **① 직접 재확인(Claude):** 계획의 행 번호와 수치를 소스에서 다시 확인했다. 삽입 게이트를 처음에 3903 으로 적은 것은 틀렸다(3903 은 옆면 재관측 대조). `approach` → `insert` 전이(504–531)로 고쳤다. 21.8 cm·6.4 cm·19.3 cm(`far-target-occupancy.md:53`), 3.7 mm(같은 문서 88), −0.162 m(`joint-angle-promotion-failures.md:55`), 0.1576 m(`joint-angle-odometry-rejudge.md:69`)는 원문 그대로다. 외접원 반경 0.986 m = hypot(0.5, 0.85).
- **② Codex 적대적 검토(1차, 73381ccd):** P1 없음, P2 3·P3 2.
  - P2 재집기에 처음 구역을 쓰면 정상 재집기를 거부한다(처음 구역 중심과 하역 뒤 탐색 중심이 9.216 m 떨어진 기록) → 2 에 미션 채택 사각형 표(처음 자리·재집기·장면 없음·모름)를 넣었다. `zone_rect` 를 쓰지 않는다고 적었다.
  - P2 live 거부 계약(실패 분류·`MISSION_KEYS`) → 4 와 시험 3 을 넣었다.
  - P2 §6 이 도달 없이 0 건으로 통과한다 → 8 에 분모·단계별 도달·판정 불가·재집기·C3 팔을 고정했다. 오거절을 "지시된 진짜 팔레트"로 정의했다.
  - P3 m 근거가 임시 출발값 수준이다 → 5 에 시험 0(기존 기록에서 단계별 중심 오차를 재서 m 을 정함)을 넣었다. "오집기는 아님" 문구를 고쳤다.
  - P3 검사 값이 실제 계획 대상과 다르다(`oracle_nominal` 치환, `near_capture_redraw` 의 `raw_`) → 유효 목표를 검사하고 원시 추정은 기록하도록 바꿨다. `near_capture_redraw` 는 채택 지점에서 뺐다.
- **② Codex 2차(b48f0d54):** P1 없음, P2 1·P3 1. 대기열이 미션 실패 때 멈추고 명시적 재개 뒤에만 제출한다는 것은 실제 동작과 같다(`queue.js:632`·`382`, CPU 네 경우 확인).
  - P2 `NEAR=on` 은 근접 추적을 켜지 않아 `near_redraw`·삽입 전 `latest_` 검사가 검증되지 않는다. 리보틱스 `recognition far`·C3 는 `--near-tracking` 과 함께 쓸 수 없다 → 8 에 근접 추적 팔 (d)(line_1_m 구성)을 넣고 `NEAR=on` 설명을 고쳤다. 승격 조건을 "모든 단계 도달 + 판정 불가 없음 + 오거절 0"으로 적었다.
  - P3 재집기 중심의 원천 → `TargetsBook` 의 `pose` 가 아니라 정상 하역 때 따로 남기는 `put_down_xy` 스냅숏으로 적었다.
- **③ 독립 검토(agy Gemini 4 Argon, ①·② 결론을 보여주지 않음, "적히지 않은 전제"):** 13 개 전제를 들었다. 반영한 것: 재집기 구역·`zone_rect` 이중 확장(2), `near_capture_redraw` 의 `raw_`·C3 는 채택 때만·`observe_succeeded` 의 부작용 순서·추적 포켓(3), 실패 분류·`MISSION_KEYS`·정지 위치(4), 여유 산출 근거와 비오라클 인식 오차·yaw 레버암(5), 드문 경로가 3 seed 에서 안 나오는 문제(8 의 단계별 도달·C3 팔), 실차 구역 정의(9). 반영하지 않은 것: "SLAM hold 추정과 지도 구역을 직접 비교하면 안 된다" — 계획기 자체가 같은 제어 좌표로 움직이므로 같은 좌표에서 비교하고, 드리프트는 오차 예산으로 다룬다(3 끝). "축 정렬 사각형이 회전된 구역에서 죽은 공간을 만든다" — 지금 구역 정의가 축 정렬 중심 범위라 검사가 정의와 같다(2 끝). "사각형 밖 확인 후보를 선택 전에 빼라" — 안전성 악화가 아니라 가용성 문제라서 후속 항목으로 남겼다(1).
