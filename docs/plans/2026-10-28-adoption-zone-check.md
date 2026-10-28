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
| 세션, 하역된 팔레트를 다시 집음(`delivered`, 처음 자리 아님) | 내려놓은 중심을 가운데로 하는 정사각형, 반폭 0.10 m(Isaac 하역 오차 0.02–0.10 m, `docs/validation/2026-10-22-session-scene-pallet-props.md:28`). 내려놓은 중심은 **`TargetsBook` 의 `Pallet.put_down_pose` 스냅숏**이다. 모든 `put_down`(하역·복구로 되돌려 놓기)이 정하고, 좌표계를 다시 표현할 때(`set_correction_version` 의 `reexpress`) 함께 옮겨진다. 같은 내려놓기를 다시 표현하는 갱신(그 하역 미션 자신의 갱신, 그 hold 의 해제, 복구 되돌림의 hold 해제)은 `update_pose(..., carry_put_down=True)` 로 스냅숏도 옮긴다. 뒤 미션의 관측 추정은 `pose` 만 바꾸고 스냅숏은 그대로 둔다. 그래서 첫 자리 여부를 자세 일치로 판단하지 않는다: 스냅숏이 없으면(이 세션에서 한 번도 내려놓지 않음) 첫 자리의 구역을 쓴다. `at`→`destinations[at]` 는 쓰지 않는다(뒤에 목적지 추정이 바뀌면 움직이지 않은 팔레트의 사각형이 따라 움직인다, Codex 구현 1차 P3). 스냅숏은 targets 메시지에 넣지 않는다 그 값은 재집기 관측 뒤 새 인식 추정으로 바뀌는데 `state="delivered"` 는 남아(`core.py:5266`, `targets.py:88`) 실패 뒤 재제출 때 마지막 내려놓은 중심이 아니게 된다. truth 실측 중심도 아니다 |
| 그 밖(복구 뒤 재제출 등으로 중심을 모름) | 사각형을 정하지 않는다. 검사 켬이면 그 미션은 첫 채택 검사에서 `adoption_zone_undefined` 로 끝난다(추정을 쓰기 전) |

yaw 범위는 기록만 하고 거부 조건에 넣지 않는다. 지도 축 사각형은 구역 정의 그대로다(`SessionZone` 의 `yaw_centre_rad` 는 heading 범위의 중심이지 사각형의 회전이 아니다).

## 3. 채택 지점 (모두 `src/forklift_core/mission/core.py`)

| 단계 | 함수 | 검사할 값과 순서 |
|---|---|---|
| 원거리 관측·근거리 캡처·재관측(retry·realign 자리) | `observe_succeeded` (3199) | `target_pickup` 을 `hooks.planning_target` 에 넣은 유효 목표. `target_pickup` 계산(3244) 직후, 옆면 단계 대조(3296·3302)·`set_inset_estimate`·`known_pallet` 등록·옆면 주행 시작(`_bottom_deck_begin`)보다 **먼저**. 거부면 그 뒤 부작용은 하나도 일어나지 않는다 |
| 둘째 근거리 캡처(C3) | `second_near_capture` (4333) | `new_pickup_`(4369). 새 캡처가 박스·dry-run 을 통과해 **실제로 채택될 때만** 검사한다. 채택되지 않고 옛 경로로 돌아가는 경우는 검사하지 않는다(새 추정을 쓰지 않으므로) |
| 추적 기반 재작도 | `near_redraw` (8361) | `target_`(8386, `G2.planning_target` 뒤). `near_install` 전 |
| 삽입 전 대조 | `approach` → `insert` 전이(504–531) | `pocket["planning_pickup"]`. 근접 추적을 켠 실행은 그 시점 최신 추적 관측(`latest_`)의 중심도 검사한다(삽입은 추적 포켓을 따라가므로). `bottom_deck_on()` 과 무관하게 돈다 |

`near_capture_redraw`(8421)는 채택 지점이 아니다. 재작도는 이미 채택된 `pocket["planning_pickup"]`(8443)을 쓰고, `raw_`(8445)는 진단 기록에만 쓰인다. live 의 `observation_take` 결과도 `observe_succeeded` 로 들어간다(`run_transport.py:8285`, Codex 확인).

검사는 순수 함수(`forklift_core/mission/adoption_zone.py`: `check(site_xy, zone, margin_m) -> record`)로 두고 위 네 곳에서 부른다. 기록의 단계 이름은 `far`·`near`·`retry`·`realign`(observe_succeeded), `second_near`, `tracked_redraw`, `insert`·`insert_tracked` 이고, 거부 사유는 `adoption_outside_zone:<단계>` 다. 통과도 기록한다(단계·유효 목표·원시 추정·사각형·m·경계까지의 부호 거리). 비교는 계획기가 쓰는 같은 제어 좌표(SLAM hold 중이면 hold 된 지도 추정)에서 한다. hold 동안의 오도메트리 드리프트는 4 의 오차 예산에 들어간다.

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

그래서 m 은 **기존 기록에서 먼저 잰다**(시험 0). 그 전 구현의 기본값은 임시 0.25 m 였고, 시험 0 뒤 0.30 m 다(아래).

- **시험 0(dev, 판정 규칙을 정한 뒤 실행):** ws1 에 남아 있는 Isaac 기록(근접 추적·C3·live 세션 포함, 목록은 실행 전에 이 문서에 적는다)에서 각 채택 지점의 유효 목표와 같은 시각 팔레트 truth 중심의 차이를 축별로 모은다. 단계별 최대와 p99 를 적는다. **m = 전 단계 축별 최대를 5 cm 단위로 올린 값**으로 정한다. 그 m 이 0.35 m 를 넘으면 구현을 멈추고 총괄에 올린다(그만큼 넓은 사각형은 이웃 칸을 덮어 방어가 약해진다).
- **시험 0 의 기록 목록(2026-10-28 ws1 `test_runs` 에서 고정, 결과를 보기 전):**
  - 리보틱스(`riibotics/`): `t2d_1b60df21`(45 실행, B0·C3·C1 팔 — C3 의 둘째 캡처는 `near_capture["second"]`), `rsbr_b35d7f53`(32), `rsbt_b35d7f53`(5), `tra_fdf0e789`(10), `trb_e560bc00`(10), `trc_fdf0e789`(5), `trd_b35d7f53`(5), `trf_36b9b7de`(4), `trx_b35d7f53`(10), `rp_c9fad6e8`(6), `p3a_*`(10). 모두 근거리 캡처가 있다.
  - live 세션(`live/`): `ct2_07ca86aa_*`(3)·`ct2_41a2a7e8_*`(1)·`ct2_624d0616_*`(3), `s3conf2_3a37e216`.
  - 근접 추적(추적 재작도·삽입 전 추적 관측): 지금 남은 기록이 없다. (B) 근접 추적 재측정의 line_1_m S2·S3 실행이 끝나면 그것을 쓴다. 그 전에는 이 두 단계의 m 근거가 없다고 적는다.
  - 단계별로 모으는 값: 관측 시도마다 `pocket_observation`·`base_from_optical`·`capture_control` 로 다시 계산한 세계 중심(유효 목표), C3 의 `near_capture["second"]["target"]`, 근접 추적의 재작도 `target`, 삽입 시작 때 `perception_pickup_estimate_m`. truth 는 들기 전 정지한 팔레트의 `initial_pallet_m`(live 는 미션별 팔레트 truth). 세계 축 x·y 별 차이.
- **시험 0 결과(2026-10-28, `tools/adoption_zone_errors.py`, 표본은 `docs/validation/2026-10-28-adoption-zone-test0/` 의 매니페스트 — 경로·SHA-256):**
  - 방법: 유효한 관측 시도마다 추정을 `observe_succeeded` 와 같은 식으로 다시 놓고(앞면 중점 + 깊이/2, 캡처 제어 뒤차축 + |rear_axle_offset_m|), 미션 자신의 팔레트 truth(`handoff.pallet_position_m` — 들기 전 팔레트는 움직이지 않는다, Codex 확인 최대 0.95 µm; 없으면 `initial_pallet_m`)와 축별로 비교한다. 기록된 추정(`perception_pickup_estimate_m`, `far_estimate_m`, 채택된 둘째 캡처의 `target`)이 1e-6 m 안에서 재현되지 않거나 유효 시도에 캡처 자세가 없으면 판정 불가로 뺀다. live 실행은 미션별 기록(`missions/*/records.json`)과, 그 기록이 없는 마지막 미션은 `result.json` 에서 읽는다.
  - 리보틱스 배치(학습 원거리 인식, team-spec·reported 엔코더 섞임): 153 기록, 판정 불가 0. 단계별 축별 최대 — 원거리 0.057 m(n 153), 근거리 0.065(n 151), 둘째 근거리 0.054(n 15), 기타 0.105(n 1, reported `p3a_a2_aisle_s0` 시도 4).
  - live 세션(`ct2_*` — 원거리 인식·reported·mapping, `s3conf2`): 29 기록, 판정 불가 7(`s3conf2` 의 마지막 미션, 기록된 추정 없음). 원거리 0.225 m(n 22), **근거리 0.254 m**(n 22, `ct2_07ca86aa_auto_random_m_s3` 마지막 미션 시도 2, dy −0.254).
  - **m = 0.30 m**(전체 최대 0.254 → 5 cm 올림). 0.35 m 를 넘지 않아 진행한다. 러너 기본값을 0.30 으로 바꿨다.
  - 표본에 없는 것: 지금 콘솔 구성(`--recognition off`·team-spec·서베이 번들 위치 추정)과 근접 추적 두 단계. 콘솔 구성은 8 의 실행에서 검사 기록의 truth(`hooks.adoption_truth`, 시뮬 전용, 판정에는 쓰지 않음, 거부된 검사에도 남음)로 잰다. 그 실행으로 m 을 다시 고르면 그 실행은 dev 가 되고, m 을 고정한 뒤 확인 실행을 다시 한다.
  - 처음 목록의 `trf_36b9b7de`(4)는 결과 보기 전 적은 수다. 커밋한 매니페스트에는 15 기록이다(같은 폴더에 실행이 늘었다). 이후 같은 표본은 매니페스트로 다시 읽는다(`--manifest-in`: 적힌 경로만 직접 읽고, 파일이 없거나 해시가 다르면 거부한다 — 그 뒤 생긴 파일은 표본을 바꾸지 못한다).
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

## 8a. 경로별 여유 측정 (총괄 2026-10-28, dev — 결과 전에 고정)

총괄 결정: 0.30 m 는 잠정값이다. 큰 오차(22–25 cm)는 live 의 지도 작성(mapping)·reported 경로에서 나왔다. 운영 기본은 위치 추정(서베이 번들, A)이므로 그 경로의 단계별 오차를 재서 경로마다 여유를 정하는 안을 올린다. 검사는 그때까지 기본 off 다.

- **팔:** 둘 다 위치 추정(registry 번들, `MAP_BUNDLE` 미설정)·team-spec(런처 기본)·live 콘솔 대기열.
  - **A-far:** `ct1_queue.sbatch` 그대로(학습 원거리 인식·근거리 캡처 기대 위치·재작도).
  - **A-off:** `--recognition off`. 세션 장면은 `--near-capture-redraw always` 를 요구하므로(러너가 거부) 근거리 캡처 기대 위치·재작도(0.10 m)는 A-far 와 같게 두고, 원거리 확정을 전제로 한 `--near-capture-lower waive_far_confirmed` 만 뺀다(`test_runs/live/az_queue.sbatch` 의 `RECOG=off`).
- **장면·seed·대기열:** line_m(`P-1:D-1,P-2:D-2,P-1:D-3`)·random_m(`P-4:D-1,P-5:D-2,P-4:D-3`) 각 seed 3 개(0·1·2). 셋째 미션은 하역한 팔레트를 다시 집는다(재집기 사각형). 팔 × 장면 × seed = 12 세션.
- **검사:** `--adoption-zone-check on --adoption-zone-margin-m 5.0` — 측정용이라 거부가 나지 않을 만큼 넓힌다. 기록에는 검사마다 유효 목표와 시뮬 truth(`hooks.adoption_truth`)가 남는다.
- **값:** 미션 기록의 `state.adoption_zone.checks` 에서 단계별 `max(|site − truth|)`(축별). 팔마다 m_path = 그 팔 전 단계 최대를 5 cm 단위로 올린 값. 이 오차 지표와 함께, 실제 판정량인 **`max(0, edge_m)` 의 최대**와 **제안 m_path 및 0.30 m 에서 거부됐을 검사 수**도 적는다 — 재집기 사각형은 앞 미션의 추정 하역 위치가 중심이라, 그 스냅숏의 오차가 추정 오차에 더해진다(Codex 8a P3: 추정 오차 0.01 m 인데 경계 초과 0.06 m 인 반례).
- **분모:** 단계별 검사 수와 그 단계에 도달한 미션 수, **재집기 사각형(`zone_source == "put_down"`)까지 도달한 세션 수**를 적는다(앞 미션이 실패하면 대기열이 멈춘다). truth 가 없는 검사·깨진 기록은 판정 불가로 따로 센다. 실행마다 `slam_choice`·map id·엔코더를 확인하고, 위치 추정이 아니라 지도 작성으로 떨어진(fallback) 실행은 A 표본에서 뺀다.
- **성격:** 팔마다 6 세션이고 한 세션 안의 검사는 서로 상관되며, 장면 파일의 팔레트 배치는 seed 와 무관하게 고정이다. 그래서 m_path 는 "관측 최대를 올린 잠정값, 상한 미확립"으로 제안한다. 근접 추적 두 단계는 세션 장면이 근접 추적을 막고, C3 는 live 를 지원하지 않아(`run_transport.py:1308`) 이 측정에 들지 않는다.
- **다음:** 경로별 m 안을 총괄에 올린다. 채택되면 그 값으로 고정한 확인 실행(§8)을 따로 돈다 — 이 측정 실행은 dev 다.
- 크기: `isaac_bundle` live 크기(GPU 당 3 세션), 두 묶음 잡이 claim 목록을 공유한다.

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
- **② Codex 3차(24551b3e):** 새 P1/P2 없음(수렴). 팔 (d)는 (B) 의 새 bounds·drive terms 와 조건부 계약 변경이 끝나야 돌릴 수 있다(`2026-10-27-insertion-lateral-drift.md:165`).
- **구현 검토(Codex, 5회, 수렴):**
  - 1차: 첫 자리 판정이 자세 완전 일치라 관측 뒤 구역을 잃음(P2) → `Pallet.put_down_pose` 스냅숏으로 바꿈. 인식 없는 배치에서 삽입 검사가 계획 대상을 못 찾음(P2) → 러너가 `--use-perception` 없는 켬을 거부. `at`→목적지가 목적지 갱신에 끌려감(P3) → 스냅숏으로 해소.
  - 2차: SLAM 해제 때 스냅숏이 옛 좌표에 남음(P2) → 같은 내려놓기를 다시 표현하는 갱신에서 `carry_put_down`.
  - 3차: 수동 복구의 지연 해제(`pallet_held`)가 스냅숏을 안 옮김(P2) → 4차에서 자세 값 비교로는 hold 를 증명하지 못한다는 반례(P2) → 5차 구현: 코어가 hold 중 내려놓은 팔레트를 기록(`put_downs_in_hold`, 해제마다 비움)하고 그 기록으로만 옮긴다.
  - 5차: 새 P1/P2 없음.
- **③ 독립 검토(agy Gemini 4 Argon, ①·② 결론을 보여주지 않음, "적히지 않은 전제"):** 13 개 전제를 들었다. 반영한 것: 재집기 구역·`zone_rect` 이중 확장(2), `near_capture_redraw` 의 `raw_`·C3 는 채택 때만·`observe_succeeded` 의 부작용 순서·추적 포켓(3), 실패 분류·`MISSION_KEYS`·정지 위치(4), 여유 산출 근거와 비오라클 인식 오차·yaw 레버암(5), 드문 경로가 3 seed 에서 안 나오는 문제(8 의 단계별 도달·C3 팔), 실차 구역 정의(9). 반영하지 않은 것: "SLAM hold 추정과 지도 구역을 직접 비교하면 안 된다" — 계획기 자체가 같은 제어 좌표로 움직이므로 같은 좌표에서 비교하고, 드리프트는 오차 예산으로 다룬다(3 끝). "축 정렬 사각형이 회전된 구역에서 죽은 공간을 만든다" — 지금 구역 정의가 축 정렬 중심 범위라 검사가 정의와 같다(2 끝). "사각형 밖 확인 후보를 선택 전에 빼라" — 안전성 악화가 아니라 가용성 문제라서 후속 항목으로 남겼다(1).
