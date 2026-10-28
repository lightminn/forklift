# 원거리 단계를 지시 대상에 묶기 — 재집기 대상 선택 (총괄 2026-10-28)

관련: `docs/validation/2026-10-28-adoption-zone-path-margin.md`(8a 측정), `docs/plans/2026-10-28-adoption-zone-check.md`. 총괄 결정: 콘솔 경로 재집기 결함(과제 1)과 칸이 붙은 배치의 재집기(과제 2)를 한 계획으로 묶는다. 진단과 계획을 먼저 올리고, 수정 뒤 같은 12 세션 구성으로 재집기와 구역 검사의 재집기 단계를 함께 확인한다. 콘솔(대기열 auto·TargetsBook)과 계획 단계에서 맞춘다.

## 1. 진단 (8a, 위치 추정·team-spec, 12 세션)

| 장면 | 팔 | 재집기(m3) 결과 | 원인 |
|---|---|---|---|
| line_m s0–s2 | A-far | 원거리 `uncertain_in_radius` → 관측 후보 소진 | **대기열 구성.** 하역 칸이 접근축(x) 위에 일렬이다(D-1 x 3.00, D-2 1.94, D-3 0.88). D-2 의 P-2 가 D-1 의 P-1 앞을 막고 가린다. 기준 검출기의 진행축 오차가 1.063 m 로, 앞의 P-2 를 봤다. 인식 켬은 옳게 멈췄다 |
| line_m s0–s2 | A-off | **이웃 P-2 채택** → 계획 `approach:straight_collision` | **D1: 인식 끔의 원거리 관측이 지시 대상에 묶이지 않는다.** 원거리 단계는 기대 위치 없이 `detect_pockets` 를 불러 화면에서 가장 강한 패턴을 채택한다. `--near-capture-expected on` 이었지만 그 기대 위치는 근거리 캡처에만 걸린다 |
| random_m s2 | 두 팔 | 근거리 `no_expected_candidate` | **D2: 원인 미상.** 원거리 단계는 맞는 대상(오차 ≤ 3 cm)을 잡았다. 근거리 캡처 자리(대상 앞 3.42 m)는 성공한 세션과 같다. 대상 앞면 평면이 `no_opening_pattern` 으로 거부되고, 남은 후보는 옆 0.43–1.36 m 다. RGB 로는 s1(성공)과 구별되지 않는다 |
| random_m s0(A-far)·s1(A-off) | | 관측 추종 실패(`Tracking failed in observe`) | **D3: 미분석** |

- `next_pick` 은 delivered 팔레트를 후보로 삼지 않으므로 line_m 의 막힌 재집기를 내지 않는다(콘솔 세션 확인). 이런 요청은 대기열 수동 항목에서만 나온다. 지금은 막힘 판정 없이 제출되고, 막힘은 계획 단계에서야 드러난다.
- random_m 의 하역 칸은 y 방향으로 1.29 m 간격이다. 이웃은 옆에 있고 앞을 막지 않는다.

## 2. 고칠 것

### F1. 인식 끔의 원거리 관측에 기대 위치 (D1)

- 원거리 캡처(`capture_and_detect` 의 원거리 갈래)에서 인식이 꺼져 있으면, `detect_pockets` 대신 `detect_pockets_expected` 를 부른다. 기대 위치는 이번 미션의 채택 사각형이다(구역 검사 2 절, `options.adoption_zone`):
  - 중심 = 사각형 중심. 처음 자리는 배치 구역(세션은 `SessionZone`), 재집기는 `put_down_pose` 정사각형이다.
  - 반경 = 사각형 반대각선 + 경로 여유(`adoption_zone_margin_m`). line_m·random_m 의 처음 자리 0.5 × 0.5 m 구역은 0.354 + 0.10 = 0.454 m 이고, 칸 간격의 절반(0.53 m, 0.64 m)보다 작다. 재집기는 0.141 + 0.10 = 0.241 m 다. 실제 구역 크기·간격은 구현 전에 장면 파일에서 다시 잰다.
  - 축 허용 = 구역 yaw 범위 + 5°(구역이 없으면 15°). 재집기는 `put_down_pose` 의 yaw 를 중심으로 둔다.
  - 배치(batch) 실행의 `scenario.pickup` 은 truth 이므로 쓰지 않는다. 사각형만 쓴다.
- 안에 후보가 없으면 `no_expected_candidate` 다. 그 관측은 실패로 끝나고 다음 관측 자리로 간다. 모두 실패하면 미션이 `perception_no_pallet` 으로 끝난다. 이웃을 채택하지 않는다.
- **ID 대조(콘솔과 합의):** 비전 결과에는 ID 가 없다. 그래서 원거리 단계를 미션 팔레트 ID 의 TargetsBook 자세(처음 자리는 등록 자세·구역, 재집기는 `put_down_pose`)에 묶는 것을 위치로 하는 대조로 삼는다. TargetsBook 자세는 해제 때의 프레임 이동을 반영한, 그때그때의 재표현 값이다(구역 검사의 스냅숏 이동 규칙과 같다).
- 인자 `--far-capture-expected {off,on}`(기본 off). 운영 런처는 4 의 확인 뒤 on 으로 한다. 인식 켬(`--recognition far`)은 이미 지시 반경(`instruction_from_zone`)으로 묶여 있으므로 이 인자와 상관없다.

### F2. 원거리 인식 켬 — 바꾸지 않음

line_m 의 `uncertain_in_radius` 는 막힌 배치에서 옳은 동작이다. random_m 에서는 원거리 단계가 맞았다. 반경을 바꾸는 근거가 없다.

### F3. 막힌 수동 재집기의 사전 거부 — 콘솔 세션 담당

콘솔 세션이 맡는다. 사유는 `reject(blocked)` 이고, `detail` 에 막는 쪽을 싣는다(예: `pickup:D-2`, `delivery:P-3`). 표기는 `next_pick` 의 blocked 목록과 같다. 범위는 `mission_submit` 의 pallet_id 가 delivered(재집기)일 때만이다. 정본은 `docs/interfaces/2026-10-13-operator-console-contract-draft.md` 의 mission_submit 행이다. 이 계획과는 별개로 진행한다.

### D2·D3 진단 (수정 전)

- D2: s2 와 s1 의 근거리 캡처에서 depth 를 비교한다(`perception_capture_2_depth_m.npy`, 대상 앞면 영역의 유효 픽셀·평면 잔차). 그리고 검출기 진단(`rejected_plane_reasons`, `opening_rays`)을 대조한다. 원인이 나오면 별도 항목으로 올린다.
- D3: 관측 추종 실패 두 건의 경로·추종 기록을 본다. 재집기와 관계가 있는지만 먼저 가린다.

## 3. 시험 (먼저 쓴다)

1. 원거리 기대 위치 만들기: 사각형 중심·반경·축 허용이 위 규칙대로 나온다. 처음 자리·재집기·장면 없음(BAY)·사각형 없음(→ 기대 위치 없이 끔과 같게 하지 않고 미션을 끝낸다: 구역 검사의 `adoption_zone_undefined` 와 맞춘다)을 다룬다.
2. 미션 코어: `--far-capture-expected on`·인식 끔이면 원거리 캡처가 `detect_pockets_expected` 를 부르고, 이웃만 보이면 `no_expected_candidate` 로 그 관측이 실패한다. 끔이면 지금과 같다(재생 게이트).
3. 러너 인자를 근접 추적 계약(RUN_SPECIFIC 또는 POST_MEASUREMENT)에 등록한다.

## 4. Isaac 확인 (판정 규칙은 결과 전에 고정)

- 팔: 8a 와 같은 A-far·A-off(위치 추정·team-spec), `--far-capture-expected on`, `--adoption-zone-check on`(경로 기본 여유).
- **재집기 가능한 대기열:**
  - line_m `P-1:D-1,P-1:D-2,P-2:D-3`. D-1 에 놓은 P-1 을 D-2 가 빈 채로 다시 집는다. D-2 로 옮긴 뒤 P-2 는 맨 앞 D-3 에 놓는다.
  - random_m `P-4:D-1,P-5:D-2,P-4:D-3`. 이웃은 옆에 있다.
  - seed 6·7·8(8a·8b 와 겹치지 않음)로 12 세션을 돈다.
- **막힌 대기열 회귀(A-off 만):** line_m `P-1:D-1,P-2:D-2,P-1:D-3` seed 6–8 의 3 세션. 기대 결과: m3 가 이웃을 채택하지 않는다(`no_expected_candidate` 끝에 `perception_no_pallet`, 또는 구역 검사 거부). P-2 를 채택하면 F1 실패다.
- 판정:
  - 재집기 가능한 대기열에서 재집기 완료율을 8a(1/6)와 비교해 보고한다.
  - 지시된 팔레트의 오거절 0. 대상은 구역 검사 8b 의 0.50 m 규칙으로 가른다.
  - 막힌 대기열 회귀에서 이웃 채택 0/3.
  - 구역 검사의 재집기(`put_down`) 단계에 도달한 세션 수를 적는다. 1 세션 이상이면 그 단계를 확인한 것으로 본다.
- 크기: live 묶음, 15 세션.

## 5. 남는 위험

- F1 은 기대 위치 안의 다른 물체를 막지 못한다. 사각형 + 여유 안의 이웃이나 다리형 선반이 그런 경우다(현장 규칙 전제).
- D2·D3 는 원인을 모른다. 재집기 완료율을 F1 만으로 올린다고 기대하지 않는다.

## 자체 검토

(검토 뒤 채운다.)
