# 채택 단계 구역 검사 (총괄 결정 2026-10-28)

관련: `docs/plans/2026-10-27-bottomless-and-bundled-pallets.md` §3.3(다리형 선반 받아들인 위험, PR #191). 받아들인 위험은 방어 ①이 실제로 성립할 때만 유효하다. 지금은 원거리 후보 선택만 지시 반경(외접원)에 묶여 있다. 근거리 캡처·재관측·재작도는 구역 검사 없이 추정을 계획 대상으로 채택한다.

## 1. 결정

- **검사:** 추정이 계획 대상이 되기 직전마다 그 팔레트 중심 추정 (x, y) 가 **지시 배치 사각형 + 여유 m** 안인지 본다. 밖이면 그 미션을 `adoption_outside_zone` 으로 멈추고 기록한다. 다른 후보로 넘어가지 않는다.
- **사각형:** 장면·세션이 준 팔레트 중심 범위 `[x_min, x_max] × [y_min, y_max]`(`scene_file.SessionZone`·`PickupZone`, 배치 실행은 `scene_layout.pickup_zone`, 세션은 미션 팔레트의 `zone`)를 m 만큼 넓힌 것. 외접원은 쓰지 않는다. yaw 범위는 기록만 하고 거부 조건에 넣지 않는다.
- **원거리 선택기는 그대로 둔다.** `select_target` 의 원(유일성·`multiple_in_radius`·`reobserve` 판정)은 바꾸지 않는다. 원 안·사각형 밖 후보가 뽑히면 채택 검사에서 거부된다. 사각형 안 팔레트와 원 안의 다른 물체가 함께 있으면 기존대로 운영자에게 묻는다.
- **콘솔 경로(`--recognition off`) 문서 문구:** 이 경로는 장면이 준 팔레트 자세(사전 정보)와 기하 검출기로 집는 **시뮬 전용 경로**다. 실차는 인식 경로(`--recognition far`)를 쓴다. 이 경로에도 같은 검사가 걸린다(검사는 채택 지점에 있으므로 인식 방식과 무관하다).

## 2. 채택 지점 (모두 `src/forklift_core/mission/core.py`)

| 단계 | 함수 | 추정이 채택되는 곳 |
|---|---|---|
| 원거리 관측·근거리 캡처·재관측(retry·realign 자리) | `observe_succeeded` (3199) | `target_pickup` → `hooks.planning_target` (3244–3313). 옆면 단계의 재관측 대조(`_bottom_deck_reobserved`, 3903)·근거리 대조(`_bottom_deck_near_check`, 4021) 바로 뒤 |
| 둘째 근거리 캡처(C3) | `second_near_capture` (4333) | `target_` → `new_pickup_` (4365–4369) |
| 추적 기반 재작도 | `near_redraw` (8361) | `site_` → `target_` (8385–8386) |
| 근거리 캡처 재작도 | `near_capture_redraw` (8421) | `raw_` (8445) |
| 삽입 전 대조 | `approach` → `insert` 전이(504–531). 지금 이 자리의 게이트는 `bottom_deck_on()` 일 때만 돈다 — 구역 검사는 그와 무관하게 돈다 | 그때의 `pocket["planning_pickup"]`(근접 추적 보정이 쌓인 값) |

검사는 순수 함수 하나(`forklift_core/mission/adoption_zone.py`: `check(site_xy, zone, margin_m) -> record`)로 두고 다섯 곳에서 부른다. 기록은 `state["adoption_zone"]` 목록에 단계·추정·사각형·m·바깥 거리(안이면 음의 여유)를 남긴다. 통과도 기록한다(오탐 여유를 재기 위해).

## 3. 여유 m = 0.25 m (임시값)

중심 추정 오차는 로봇 자세 오차 + 인식 오차다.

- 원거리 관측 중 미션별 최대 위치 오차: 108 미션 중앙값 6.4 cm, p95 19.3 cm, **최대 21.8 cm**(`docs/validation/2026-10-24-far-target-occupancy.md:53`, 잡 10657). 여기에 yaw 오차 × 관측 거리가 더해진다(같은 문서).
- 원거리 인식의 중심 오차: 정답 팔레트 14 건에서 측면 ≤ 3.7 mm, 진행 방향 ≤ 0.9 mm(같은 문서 88행, Isaac 오라클 분할 — 실차 값이 아님).
- 근거리 단계: 삽입 끝 SLAM 측면 오차 −0.162 m(`2026-10-25-joint-angle-promotion-failures.md:55`), 근거리 대조 거리 0.1576 m(`2026-10-23-joint-angle-odometry-rejudge.md:69`).

관측된 최대 0.22 m 위에 yaw × 거리 몫(약 0.03 m)을 얹어 0.25 m 로 둔다. **실차 SLAM·D435i 중심 오차를 잰 뒤 다시 정한다.** 이 값이 커지면 현장 규칙("선반을 두지 않는다")이 미치는 공간도 커진다는 점을 문서에 함께 적는다.

## 4. 인자·기본값

- 러너 `--adoption-zone-check {off,on}`(기본 `off` — 인자 없는 옛 기록의 읽기), `--adoption-zone-margin-m`(기본 0.25). `sim/isaac/near_tracking.py` 의 RUN_SPECIFIC 계약에 등록한다(등록하지 않으면 근접 추적 실행이 시작되지 않는다).
- 운영 런처(콘솔 `isaac_live.sbatch`, 리보틱스 `riib_scene.sbatch`)의 기본은 6 의 Isaac 확인을 통과한 뒤 `on` 으로 바꾼다.

## 5. 시험 (먼저 쓴다)

1. `adoption_zone.check`: 경계 안·경계 위·여유 안·여유 밖, 사각형 모서리(원 안·사각형+m 밖 점은 거부), NaN·빈 구역은 오류.
2. 미션 코어 CPU 시험: 다섯 채택 지점마다, 검사 켬에서 구역 밖 추정이 `adoption_outside_zone` 으로 멈추고 기록이 남는다. 검사 끔이면 지금과 같다(재생 게이트 기록 동일).
3. CPU 리그 반례: `shelf_legs` 를 사각형 밖·외접원 안(예 1.0 × 1.7 m 구역 모서리 쪽)에 두면 원거리 선택은 뽑지만 채택에서 거부된다.

## 6. Isaac 확인 (판정 규칙은 결과 전에 고정)

- **오탐 0:** 검사 켬, 콘솔 line_m·random_m 각 3 seed + 리보틱스 장면(aisle_rack 포함) 3 seed. 진짜 팔레트에 대한 `adoption_outside_zone` 이 한 건도 없어야 한다. 한 건이라도 나오면 런처 기본을 바꾸지 않고 그 사례의 오차를 분해한다(m 을 결과 보고 올리지 않는다).
- **여유 기록:** 모든 채택 기록의 바깥 거리 분포(최대값)를 문서에 적는다.
- 크기는 `isaac_bundle` 계산(GPU 당 3 세션)을 따른다.

## 7. 남는 위험

- 사각형 + m 안에 다리형 선반이 있으면 여전히 막지 못한다(현장 규칙 전제).
- yaw 는 검사하지 않는다.
- 위치 오차가 m 을 넘는 실차 상황에서는 진짜 팔레트를 거부한다(작업 실패이지 오집기는 아님).

## 자체 검토

(검토 뒤 채운다.)
