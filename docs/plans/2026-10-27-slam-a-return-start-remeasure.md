# SLAM A 전환 뒤 하역 줄 치우침과 복귀 출발 물러남 재측정 (2026-10-27, 측정 계획)

지시: 총괄(2026-10-27). live SLAM 기본이 A(조사 지도 위 위치 추정, PR #167)로 바뀐 뒤 첫 random_m 배치에서 두 지표를 다시 잰다. **판정이 아니라 측정이다.** 지표와 비교 기준은 결과 전에 이 문서에 적는다.
배경: `docs/validation/2026-10-27-return-start-backoff.md`(PR #176). mapping 에서는 random_m 하역 줄의 위치 추정이 y 로 0.12–0.23 m 치우쳤다. 그 만큼 저장된 이웃 추정이 밀려, 다음 미션의 복귀 출발이 막혔다(`return_home:invalid_start`).

## 배치

- 스냅숏: #167·#176 이 들어간 main.
- 장면·순서: `ct1_queue.sbatch`, `scenes_ct1_a770031/random_m_session.json`, SCENARIO=queue, CYCLE P-4:D-1, P-5:D-2, P-3:D-3, seed 0–11(3 슬롯 묶음의 빈 슬롯을 없애려고 12 실행).
- 설정:
  - MAP_BUNDLE 은 기본이다(registry 위 위치 추정).
  - 런처 기본 그대로다(team-spec, D7_HOLD on).
  - `MORE=--return-start-backoff on` 한 팔이다(발동 빈도를 재려면 켜야 한다).
- 제출: live 3 슬롯 묶음(GPU 당 3 세션, PR #161), 의존성 없음. FIFO 로 trailer 진단·lvram 뒤에 선다.
- 제출 전 runner 인자 사전 검사(`isaac_bundle.py check`)를 거친다.

## 지표

**M1 하역 줄 위치 추정 치우침**: 하역 도킹 1 회차의 `delivery_docking.rounds[0].estimate_vs_truth_at_match` 의 (dx, dy)다. 미션별, 목적지(D-1·D-2·D-3)별로 본다.
- 분포: 개수, |dy| 중앙값·최대, dy 부호별 개수, |e| 최대.
- 보조: 다음 미션의 `mission_scenario` 에 저장된 이웃 `@search` 의 칸 대비 dy(같은 런에서 팔레트별 하나). 경계 0.21 m 이상인 개수.

**M2 복귀 출발 물러남 발동 빈도**: `state.return_start_backoffs` 를 센다. 실행 폴더 전수 키 검사로 빠진 것이 없는지 함께 본다.
- 이웃이 있는 미션(둘째·셋째) 가운데 시도·채택이 있는 미션 수
- 시도가 나온 곳(mission_plan / release_replan / live_leg)
- `return_home:invalid_start`·`slam_release_replan_failed:invalid_start` 로 끝난 미션 수

## 비교 기준 (A 전 mapping, 결과 전에 적음)

rh_5d280e80 의 random_m 실행이다(off_random_m_s5·s8, on_random_m_s4·s6, on_tq_random_m_s2). F2′ 이전이라 M2 는 "복귀 출발 무효" 수로 대신한다. 원자료는 `gdrive:forklift-archive/ws1-cleanup-20261027-d/` `d27_rh_01`·`02` 이고, 추출본에서 다시 셌다.

| 지표 | rh(mapping) |
|---|---|
| M1 1 회차 dy 개수 | 9(D-1 5, D-2 4) |
| M1 \|dy\| 중앙값 / 최대 | 0.190 / 0.219 m(모두 dy < 0, 칸 쪽) |
| M1 보조: 저장된 이웃 추정 dy | 9 개, −0.134 – −0.247 m, 0.21 m 이상 4 개 |
| M2 대신: 이웃 있는 미션 중 복귀 출발 무효 | 4 / 9 |

참고(콘솔 S3 2 차, A, D7_HOLD off): random_m 하역 줄 \|e\| 최대 0.044–0.045 m.

## 읽는 법 (판정 아님)

- 두 지표를 rh 와 나란히 적는다. A 에서 M1 의 |dy| 가 0.21 m 경계보다 충분히 작고 M2 가 0 이면, F2′ 는 A 에서 거의 쓰이지 않는 예비 장치로 기록한다.
- 경계에 닿는 사례가 나오면 그 미션의 기록(도킹 라운드, 저장 추정, 후진 설치·주행)을 따로 적는다.
- 런처 기본값은 바꾸지 않는다. 판단은 총괄이 한다.

## 결과 (2026-10-27, 잡 11825, 스냅숏 `sar_05d6faa6`)

12 실행이 모두 끝났다(묶음 기록 `ended` 12, 세션 중단 0). 36 미션 모두 완주했다. 모든 실행의 supervisor `slam_choice` 는 `localization (registry: random_m_team_spec_quantized_0facac3eea66)` 이고, 러너 기록 인자에 `return_start_backoff: on` 이 있다. 집계는 ws1 `test_runs/live/sar_05d6faa6/sar_sum.py`(`sar_sum.txt`)로 했다.

| 지표 | A(이번) | rh(mapping) |
|---|---|---|
| M1 1 회차 dy 개수 | 36(D-1·D-2·D-3 각 12) | 9(D-1 5, D-2 4) |
| M1 \|dy\| 중앙값 / 최대 | 0.004 / 0.012 m | 0.190 / 0.219 m |
| M1 dy 부호 | 칸 쪽(dy < 0) 23, 반대 11, 0 2 | 모두 칸 쪽 |
| M1 \|e\| 최대 | 0.038 m | — |
| M1 보조: 저장된 이웃 추정 dy | 24 개, −0.022 – +0.009 m, 0.21 m 이상 0 | 9 개, −0.134 – −0.247 m, 0.21 m 이상 4 |
| M2 F2′ 시도 / 채택 | 0 / 0(실행 폴더 전수 키 검사 0) | (F2′ 이전) |
| M2 `invalid_start` 로 끝난 미션 | 0 | 이웃 있는 미션 중 4 / 9 |

- 목적지별 \|dy\| 최대: D-1 0.012, D-2 0.008, D-3 0.009 m.
- dx 는 −0.037 – −0.024 m 로 세 칸 모두 같은 쪽으로 조금 치우친다. 콘솔 S3 의 A 값(\|e\| 최대 0.044–0.045 m)과 같은 크기다.

## 읽음 (판정 아님)

- A 에서 하역 줄의 y 치우침은 rh(mapping)의 약 1/16(최대 기준 0.012 대 0.219 m)이다. F2′ 가 걸리는 경계(이웃 추정이 칸 쪽으로 약 0.21 m)보다 한 자릿수 이상 작다.
- 이번 36 미션에서 F2′ 는 한 번도 시도되지 않았다. A 에서는 F2′ 가 거의 쓰이지 않는 예비 장치로 기록한다. mapping 대체(등록부 행이 없는 장면, reported)나 위치 추정이 크게 틀어진 경우에 대비한 장치로 남는다.
- 런처 기본값(F2′ off)은 바꾸지 않는다. 판단은 총괄이 한다.
