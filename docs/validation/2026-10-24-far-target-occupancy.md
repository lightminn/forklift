# 원거리 대상 점유 배제와 CT-1 두 팔 삽입 깊이 측정 (2026-10-24)

다룬 계획은 두 개다.
- `docs/plans/2026-10-24-far-target-occupancy.md`(3판): O1 바닥 계약 아래의 점유 배제.
- `docs/plans/2026-10-24-insertion-depth-margin.md`(3판) 5 절: CT-1 에서 reported·team-spec 두 팔을 측정한다.

브랜치는 `plan/far-target-occupancy` 다.

## 1. 바꾼 것

| 커밋 | 내용 |
|---|---|
| 5c4ede3f | `select_target` 의 점유 배제(`occupancy_margin_m`, 상수 `MIN_HALF_SHORT_SIDE_M` 0.25), 브리지·런타임·재계산 연결, 러너 인자 `--recognition-occupancy-margin-m`(Isaac 기본 0.05, `none` 끔), 근접 계약 등록 |
| 41302c77 | 콘솔 런처 `ENCODER_MODEL`(기본 reported). team-spec 이면 team-spec 장애물 층 나이 표를 쓴다. 지금까지 CT-1 은 모든 팔이 reported 였다 |
| 13d07d37 | m 이 꺼져 있으면 기록 요약에 키를 넣지 않는다(옛 기록 형태, ec4bece 골든) |
| 4c53486 | 측량 잡이 콘솔 러너와 글자 그대로 같도록 맞춤. 재계산이 프레임 파일을 닫는다 |

2026-10-24 사용자 정책(총괄 경유)에 따라 판정 규칙과 런처 기본값은 이 문서에 남기고 진행했다.
- 러너 기본 m = 0.05 는 Isaac 전용 경험값이다.
- 실차 D435i 구성에서는 원거리 중심 오차를 측정하기 전까지 끈다(계획 O1 의 남는 위험).

## 2. 시험

- **단위·연결**: 대상 선택 13 경우(Codex 반례 포함)를 시험했다.
  - 조건마다 돌연변이 9 개를 넣었고, 9 개 모두 시험이 잡았다. 층 지시, 바닥, 알려진 형상, 경쟁 중심, 깊이 선분, `_decide`, h_min, m, 후보 하나 조건이다.
  - 브리지(실제 맞춤), 기록·재계산, 러너 연결, 런처 연결, 프레임 파일 닫기도 시험했다.
- **ws1 전체 pytest**(4c53486): 6451 통과, 33 건너뜀, 10 실패다. 실패 10 건은 알려진 ws1-only 다(mujoco 없음 6, 두 통합 시험, `diagnose_detection` 2).
- **재생 게이트**(`runs29.txt`, 스냅숏 occ_5c4ede3): 29/29 `ok` 다. 기록된 실패도 같은 틱·사유로 재현됐다.

## 3. 기록 재계산(회귀)

ws1 의 모든 원거리 인식 기록 427 곳(`far_regress.py`, 잡 10400)을 m 없음(기록 그대로)과 m = 0.05 로 다시 계산해 정지마다 비교했다.

- 3 곳은 `result.json` 이 없는 미션이다(세션이 미션 도중 끝남). 나머지 424 곳을 비교했다.
- 판정 전환은 다음과 같다.

| 전환 | 정지 수 |
|---|---|
| selected → selected | 408 |
| reobserve → selected(`occupancy_excluded`) | 353 |
| reobserve → reobserve | 13 |
| none → none | 10 |
| 그 밖 | 0 |

- 바뀐 353 곳은 모두 최종 관측이 `valid` 다. 쓴 후보의 두 포켓 중심과 정답 포켓(`truth_face`)의 차이는 최대 7.4 mm 다.
  - 고른 팔레트는 지시 대상이고 그 중심 오차는 m(50 mm)보다 작다. 그러니 바닥 계약 아래에서는 반경 안에 다른 바닥 팔레트가 있을 수 없다. 애매한 지시를 자동 선택한 경우는 없다.
- 대상: CT-1 대기열·연속 이송 회차, 리보틱스 장면(통로·트레일러·line·random·컨베이어 대체), 관절각 비교(bd_isaac jao2, jao3).
  - 리보틱스 장면도 팔레트는 모두 홀 바닥에 놓인다. 블록은 팔레트 높이의 상자 소품이다.
- random_m P-4 실패 기록도 다시 계산했다(계획 1 절의 42 정지와 총괄이 넘긴 `lec_le_b9106d0_random_m_s0·s2·s3`). 모두 첫 정지에서 P-4 를 골랐다.

## 4. Isaac 세 팔(CT-1 대기열, 같은 시간대)

- 스냅숏 옛 `q2_0136fc5`(reported)와 새 `occ_4c53486`(reported / team_spec_quantized)을 썼다.
- seed 0–4 × line_m·random_m 을 seed·장면마다 세 팔 번갈아 돌렸다(잡 10572–10601).
- 시나리오는 #119 의 대기열이다(line_m P-1·P-2·P-3, random_m P-4·P-5·P-3).
- 분석: ws1 `test_runs/live/occ_analyze.py`·`occ_age.py`(잡 10654).

### 실행

| 팔 | 세션 | 제출 미션 | 완료 | 실패 |
|---|---|---|---|---|
| 옛 reported | 10 | 23 | 19 | P-3 near_capture_misaligned, P-2 transport 추종 시간 초과, P-3 return_home:invalid_start, P-4 transport 추종 시간 초과 |
| 새 reported | 10 | 20 | 18 | P-2 Pallet slipped off forks, P-3 Excessive body tilt in transport |
| 새 team-spec | 10 | 27 | 23 | P-3 return_home 추종 시간 초과 ×2, P-3 transport_recovery_after_docking, P-5 Pallet slipped off forks |

- 세 세션(옛 line_m s2, 새 reported line_m s3·random_m s1)은 40 초 만에 미션 없이 끝났다. #119 의 random_m s4 와 같은 대기열 시작 침묵이고 팔과 무관하다.
- 네 세션은 마지막 미션 도중 runner 가 새 epoch 로 넘어가 그 미션 결과가 없다(`result_not_this_mission`). 옛 1, 새 reported 1, team-spec 2 다.

### 점유 배제: random_m 첫 원거리 관측

| 팔 | 시작한 random_m 세션 | 첫 관측 소진 실패 | P-4 를 고를 때까지 정지 |
|---|---|---|---|
| 옛 reported | 5 | 0 | 3, 3, 1, 3, 5 |
| 새 reported | 4 | 0 | 1, 1, 1, 1 |
| 새 team-spec | 5 | 0 | 1, 1, 1, 1, 1 |

- 새 스냅숏 두 팔의 원거리 정지 47 건은 모두 첫 정지에서 선택됐고 재관측은 0 이다.
  - 그중 14 건이 `occupancy_excluded` 다(random_m 의 P-4 와 P-3). 14 건 모두 정답 팔레트이고, 측면 오차는 1.6–5.7 mm, 진행 방향 오차는 0.4 mm 이하다(`detector_error`).
- 이번 회차에서는 옛 스냅숏도 소진 실패가 없었다. #119 두 회차의 5/9 와 다르다. Isaac 은 비결정적이어서 실패 빈도 비교에는 표본이 작다.
  - 확실히 말할 수 있는 것은 이렇다. 옛 규칙은 재관측을 되풀이했고(정지 1–5 번), 새 규칙은 그 재관측을 첫 정지에서 없앴다. 회귀 재계산(3 절)에서도 옛 소진 실패 정지가 모두 첫 정지 선택으로 바뀌었다.
- 주행 중 대상 잃음(`target_lost_while_driving`)은 새 규칙으로 줄지 않았다. P-4 미션당 1–2 번이다.

### 삽입 깊이(D3 규칙·예비 16 mm 그대로)

| 팔 | 삽입 | Δ(근접 촬영 → 삽입 끝, mm) | 최대 \|Δ\| | 중심 초과 최소 | 캐리지 간격 최소 | 도착 오차 |
|---|---|---|---|---|---|---|
| 옛 reported | 21 | −14.3…+10.9 (sd 7.3) | 14.3 | +11.0 | 8.2 | 6.2–8.0 |
| 새 reported | 20 | −4.0…+23.4 (sd 7.1) | 23.4 | −1.1 | 16.9 | 5.0–7.8 |
| 새 team-spec | 27 | −0.8…+5.2 (sd 1.1) | 5.2 | +19.7 | 21.7 | 7.6–7.8 |

- **항 분해**(hold 구간, `odom_drift_terms`):

| 팔 | `noise`(mm) | `rate_angle` / `quantize`(mm) | `angle_truth`(mm) |
|---|---|---|---|
| 옛 reported | −7.6…+7.7 | `rate_angle` −12.6…+9.1 | +0.9…+3.0 |
| 새 reported | −8.1…+12.6 | `rate_angle` −2.8…+28.4 | +0.7…+2.5 |
| 새 team-spec | 0 | `quantize` −0.3…0 | +1.1…+2.8 |

- 삽입 깊이 계획 3 절의 가설("team-spec 에서는 얇지 않다")이 폐루프에서 맞았다. team-spec 의 Δ 는 거의 `angle_truth` 뿐이고, 양쪽 여유 22 mm 안에 넉넉히 든다(최대 5.2 mm).
- 새 reported 의 실패 두 건은 삽입 여유 실패다.
  - line_m s1 P-2: 중심 초과 +2.1 mm, Δ +18.8 mm. 미끄러졌다.
  - line_m s2 P-3: 중심 초과 −1.1 mm, Δ +23.4 mm. 기울었다.
  - 두 건 모두 `rate_angle`(+20.8 / +28.4 mm)이 지배한다.
- **나이 표 넘침**(`occ_age.py`, 사용 나이 FREE 0.2 s·OCCUPIED 0.3 s, 각 팔의 표):
  - 미션을 돌린 세션은 모두 VALID 이고 넘침 0 이다. 최악 비율은 옛 0.78, 새 reported 0.78, team-spec 0.47 이다.
  - 미션 없이 끝난 세 세션은 덮음 0 이라 INVALID 다(판정 대상 아님).

## 5. 관절각(team-spec) 승격 판정

재판정 문서(`docs/validation/2026-10-23-joint-angle-odometry-rejudge.md` 3 절)의 조건을 이 측정에 적용했다.

| 조건 | 결과 |
|---|---|
| 1. hold 드리프트의 \|중앙값\|·max(\|·\|) 가 reported 보다 작다 | 충족. \|중앙값\| 1.4 대 7.9 mm, 최대 5.2 대 23.4 mm(새 reported). 유효 수 27 대 20 은 같지 않다 |
| 2. 실패가 늘지 않고 새 유형이 없다 | **불충족.** team-spec 4/27 대 새 reported 2/20. 다른 팔에 없던 유형이 셋이다: return_home 추종 시간 초과 ×2(line_m s0·s1 P-3), transport_recovery_after_docking(random_m s1 P-3), 그리고 깊이와 무관한 미끄럼(random_m s3 P-5, 중심 초과 +20.6 mm, Δ +0.5 mm) |
| 3. 성공 → 실패 쌍의 원인이 오도메트리 조합이 아니다 | **미판정.** 같은 seed·장면·미션이 새 reported 에서 성공하고 team-spec 에서 실패한 쌍은 random_m s3 P-5 하나다. 원인을 아직 진단하지 않았다. 나머지 셋은 짝이 되는 reported 미션이 돌지 않았다 |
| 4. 나이 표 넘침 0 | 충족. 10/10 VALID, 최악 0.47 |

- 조건 2·3 을 채우지 못했으므로 기본값은 reported 로 둔다(정책: 조건 미충족이면 유지).
- 삽입 깊이의 근거는 충분하다(조건 1). 남은 것은 team-spec 에서 새로 나온 세 유형의 원인이다.
  - line_m P-3 의 return_home 추종 시간 초과는 두 seed 에서 같은 모양이었다(복귀 210 s, 정체 재계획·cusp 정체·최종 목표 크리프 기록 없음).
  - 이것이 team-spec 오도메트리(정지 중 고정 계수, 양자화)와 관련된 결함인지가 다음 진단 대상이다.

## 6. 한계

- Isaac 은 비결정적이다. 세 팔의 실패 수 차이(4/23, 2/20, 4/27)는 이 표본 크기에서 구분되지 않는다. seed 당 한 번씩만 돌렸다.
- 점유 배제의 m = 0.05 는 Isaac(깊이 1 mm 양자화, 잡음 모형 없음)의 경험값이다. 실차 D435i 에서는 끈다.
- 삽입 깊이의 시뮬레이션 결과는 실차 미끄럼·타이어 변형·δ_cam·하중 편심을 포함하지 않는다(M6 의 측정 과제). T11 × 0.6 은 다루지 않았다.
