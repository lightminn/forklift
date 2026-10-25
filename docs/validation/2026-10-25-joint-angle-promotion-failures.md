# 관절각(team-spec) 오도메트리 승격: team-spec 팔 실패 세 유형의 진단 (2026-10-25)

총괄 지시(2026-10-25): PR #127 의 CT-1 세 팔 측정(`docs/validation/2026-10-24-far-target-occupancy.md` 4–5 절)에서 team-spec 팔에서만 나온 실패를 진단한다. 그것이 관절각 고유 결함인지, 비결정성이나 다른 결함인지 가린다. 결과로 승격 조건 1–4 를 다시 판정한다.

## 1. 반복 실행

- PR #127 회차에서 team-spec 이 실패한 seed·장면 넷(line_m s0·s1, random_m s1·s3)을 다시 돌렸다.
  - 스냅숏은 같은 `occ_4c53486`, 두 팔(reported·team_spec_quantized)을 같은 시간대에 번갈아 3 번씩, 24 세션이다(ws1 잡 10694–10717, `rep_submit.sh`).
  - 총괄이 다른 잡을 먼저 돌리려 중간에 보류·해제했다. 그래서 r1 일부와 나머지가 시간대가 다르다(예: r1 random_m s3 두 팔의 시작이 58 분 51 초 떨어졌다). 보류 전후 모두 두 팔이 있어 완전히 갈린 것은 아니지만, 시간대 교란은 배제하지 못한다.
  - 팔은 세션 이름(rep/new 뒤의 rp·tq)으로 나눴다. 분석기(`occ_analyze.py`)의 `encoder` 필드는 마지막 epoch 의 meta 로 덮여, 빈 epoch_2 가 있는 두 세션(newtq line_m s2, random_m s4)에 reported 로 잘못 붙는다.
- 합친 표본(PR #127 의 새 스냅숏 두 팔 + 반복, 같은 스냅숏):

| 팔 | 세션 | 미션 없이 끝난 세션 | 미션 | 완료 | 실패 |
|---|---|---|---|---|---|
| reported | 22 | 2 | 53 | 46 | 7 |
| team-spec | 22 | 2 | 56 | 49 | 7 |

- reported 실패(7):
  - return_home 추종 실패 ×2(line_m s0 r1·r3 P-3, yaw 오차 0.05–0.07 rad)
  - return_home:no_path(random_m s1 r1 P-3)
  - observe 추종 시간 초과(random_m s1 r2 P-5)
  - docking_retry_match_refused(random_m s3 r3 P-3)
  - 미끄럼(line_m s1 P-2)
  - 운반 중 기울기(line_m s2 P-3)
- team-spec 실패(7):
  - return_home 추종 시간 초과 ×2(PR #127 line_m s0·s1 P-3)
  - return_home 추종 실패(line_m s1 r2 P-3)
  - transport_recovery_after_docking ×2(random_m s1 P-3, s3 r3 P-3)
  - 미끄럼 ×2(random_m s3 P-5, s1 r2 P-5)
- PR #127 에서 team-spec 이 실패한 seed 에서, 반복해 보면 team-spec 이 완주하고 reported 가 실패한 경우가 있다(line_m s0 r1, random_m s1 r1). 같은 seed 라도 실패가 팔을 옮겨 다닌다.
- 다만 같은 스냅숏 표본 안에서 유형별로 보면 다음과 같다. 장면별 실패도 line_m 4 → 3, random_m 3 → 4 로 방향이 갈린다.
  - `Tracking timeout in return_home`: team-spec 2, reported 0
  - `transport_recovery_after_docking`: team-spec 2, reported 0
  - 미끄럼: team-spec 2, reported 1
  - 둘을 합친 4/56 대 0/53 은 이 표본에서 우연과 구분되지 않는다(단측 Fisher p ≈ 0.07). 그러나 차이가 없다는 증거도 아니다.

## 2. 세 유형의 원인

ws1 에 남은 live 기록 전부(reported 185 미션, team-spec 27 미션, PR #127 시점, `fail_census.py`)에서 세 유형은 모두 reported 에도 있다. return_home 추종 시간 초과 3, transport_recovery_after_docking 3, Pallet slipped off forks 2 다. 다른 스냅숏·다른 세션의 기록이라, "team-spec 에서만 가능한 결함은 아니다"를 뒷받침할 뿐 이 비교의 유형 기준을 대신하지는 않는다.

### 2.1 return_home 추종 시간 초과(line_m P-3)

- 하역 뒤 복귀 첫 회전에서, 허가층이 D-2 칸 표지(x 1.226–2.126, y −7.446, 높이 1.3 m) 끝의 칸을 `occupied` 로 보고 속도를 0 으로 막았다(`obstacle_dump_*.npz`, why occupied).
- 3 s 뒤 P6 `permit_stop` 장면 재계획이 `scene_replan_failed:permit_stop:invalid_start` 로 실패했다.
  - 여유 0.2 m 로는 출발 자세 자체가 막혀 있었다.
  - 탈출 후보 8 개(전진 0.3–1.5 m, 후진 0.3–1.5 m)가 모두 막혔다.
  - 한 다리당 요청은 한 번이라, 그 뒤로는 추종 시간 초과까지 기다렸다(73 s).
- 같은 지점·같은 모양이 reported 에도 있다. `sr_0f88b91_line_m_s1` m3 P-3 은 P6 이전 스냅숏이라 재계획 시도 없이 시간 초과로 끝났다.
- `rh_compare.py` 로 SLAM 제어 자세 1 m 앞 점과 표지 끝점의 거리를 2 s 표본에서 쟀다. 멈춘 실행은 0.68–0.73 m(team-spec 둘, reported 하나), 같은 정도로 가깝고도 지나간 실행은 0.73–0.74 m(reported 둘)였다. 제어 좌표의 표본 점거리라 truth 차체의 연속 최단 거리는 아니다. 같은 물리 경계라는 증명이 아니라, 두 팔 모두 그 근처를 지난다는 정도다.
- 원인은 계획 여유와 허가층 사이의 차이, 그리고 탈출 실패다. 계획 여유–허가 작업(`docs/plans/2026-10-23-planner-permission-clearance.md`, #128 출발 띠 해제)의 범위다.

### 2.2 transport_recovery_after_docking

- team-spec random_m s1 P-3 은 운반 시작 도킹이 `docking_unaligned`(측면 0.201 m)였다. 재시도(trigger b) 뒤 운반 다리 끝에서 정체했고, 도킹이 받아들여진 상태라 `require(not (transport and accepted_any))` 에서 끝났다.
- 삽입 끝 SLAM 측면 오차가 −0.162 m 였다.
- sw-연속이송 세션이 reported 의 같은 사유 두 건을 같은 경로로 확정했다. 도킹 측면 0.19–0.22 m 어긋남 → D7 재접근 → 곡률 상한 → cusp 정체다. 그 세션이 D7 재접근 곡률을 낮추는 수정을 맡았다.

### 2.3 "Pallet slipped off forks"(random_m P-5)

- 실제 미끄럼이 아니다. 판정(`run_transport.py`, extract·transport 단계)은 팔레트 원점(차체 앞 0.93 m)이 truth 차체 진행축에서 측면 0.04 m 안에 있는지를 본다.
  - 실패 두 건은 측면 −0.0476, −0.0509 m 였다. 원래 측면 약 27 mm 에, 팔레트–차체 yaw 차 0.016 rad × 0.93 m ≈ 15 mm 가 더해졌다.
  - 팔레트는 포크와 함께 올라갔다(pz 0.118 m).
- 판정 여유는 두 팔 모두 얇다(`slip_margin.py`, 꺼내기 첫 틱).
  - reported 135 건: |측면| 중앙값 13.4 mm, p90 24.6 mm, 최대 36.9 mm(넘음 0).
  - team-spec 58 건: 중앙값 14.1 mm, p90 28.2 mm, 최대 50.9 mm(넘음 2).
- yaw 차의 출처(`p5_yaw.py`, `odom_drift_terms` 를 근접 촬영 → 삽입 끝 창에 적용):
  - 삽입 끝 차체–팔레트 yaw 차는 hold 구간 yaw 드리프트와 부호만 반대로 같다.
  - team-spec 두 건의 드리프트는 거의 다 `angle_truth` 항이다(−0.0153, −0.0163 rad). `quantize` 항은 0.0002 rad 이하다.
  - `angle_truth` 는 관절각으로 굴린 운동과 차체 실제 운동의 차이(Isaac 의 바퀴·조향 접촉)로, 두 모델에 똑같이 들어간다. reported P-5 에서도 −0.0092 rad 까지 나왔다.
- 팔레트별 삽입 끝 |yaw 차|(`ins_yaw.py`):
  - team-spec P-5 만 p90 0.0157, 최대 0.0163 rad 다(11 건 중 2 건). reported P-5 는 최대 0.0078 rad(23 건)다.
  - 전체로는 두 팔이 같다: 중앙값 0.0041 rad, p90 0.0071 대 0.0078 rad.
- 기록된 궤적에서 관절각 사슬의 직접 몫(양자화)은 작다. 그러나 이것은 장부상 분해다(`odom_drift_terms` 의 설명대로 인과 분해가 아니다).
  - 조합이 바뀌면 SLAM·추종·조향이 바뀌고, 그 결과 다른 궤적·접촉이 생겨 `angle_truth` 가 커질 수 있다. 이 경로는 배제하지 못했다.
  - P-5 꼬리 2/11 대 0/23 도 차이의 증명은 아니지만 무관하다는 근거도 아니다.
  - reported 에서 `rate_angle` 이 `angle_truth` 와 부호가 반대일 때가 있었다(예: +0.0036 대 −0.0092 rad). 우연인지 구조적인지는 확인하지 않았다.
- 판정 여유(13 mm)가 얇아서 이 yaw 가 실패로 드러났다는 것은 확실하다.

## 3. 승격 조건 다시 판정

재판정 문서(`docs/validation/2026-10-23-joint-angle-odometry-rejudge.md` 3 절)의 조건이다.

| 조건 | 결과 |
|---|---|
| 1. hold 드리프트의 \|중앙값\|·max(\|·\|) 가 reported 보다 작다(유효 수 같음) | 원래 기준으로 **불충족**: 유효 hold 는 reported 51, team-spec 56 이다. 보조 분석으로 공통 42 쌍(장면·seed·반복·epoch·미션)에서 \|중앙값\| 1.40 대 3.92 mm, 최대 3.32 대 19.76 mm 로 team-spec 이 작다. 수치상 개선은 뚜렷하지만 요건 충족과는 구분한다 |
| 2. 실패가 늘지 않고 새 유형이 없다 | **불충족.** 총 실패는 7/56 대 7/53 으로 같다. 그러나 같은 스냅숏 표본에서 return_home 추종 시간 초과(2 대 0)와 transport_recovery_after_docking(2 대 0)은 team-spec 에만 있다. 다른 스냅숏의 reported 사례는 그 유형이 reported 에서도 생길 수 있다는 근거일 뿐이다 |
| 3. 성공 → 실패 쌍의 원인이 오도메트리 조합이 아니다 | **미판정.** 세 유형의 기전(2 절)은 관절각 사슬의 직접 결함이 아니다. 하지만 조합 변경이 폐루프에서 궤적·접촉을 바꿔 그 기전을 더 자주 부를 가능성은 남는다 |
| 4. 나이 표 넘침 0 | 충족. 미션을 돌린 세션은 모두 VALID, 넘침 0(team-spec 최악 0.97). INVALID 둘은 미션 없이 끝난 세션이다(저장 로그 기준, 전량 재적분은 하지 않음) |

- **판정: 조건 1·2 를 채우지 못하고 3 은 미판정이라 기본값은 reported 로 둔다.**
- 삽입 깊이의 근거는 분명하다. 이 판정을 가를 다음 단계는 5 절이다.
- 삽입 깊이 쪽 근거(`docs/plans/2026-10-24-insertion-depth-margin.md`)도 합친 표본에서 그대로다.
  - team-spec 의 Δ 는 −0.8…+6.7 mm, 중심 초과 최소 +19.0 mm, 캐리지 간격 최소 21.7 mm 다.
  - reported 의 Δ 는 최대 +23.4 mm, 중심 초과 최소 −1.1 mm 다.

### 3.1 결정(2026-10-25, 총괄)

- 위 판정 뒤 총괄이 근거를 따져 **관절각(team_spec_quantized)을 운영 기본으로 승격**했다.
- **사전 조건을 채우지 못한 상태에서 총괄이 근거를 따져 내린 예외 결정이다.** 조건은 바꾸지도, 폐기하지도 않았다. 판정은 조건 1·2 불충족, 3 미판정 그대로다.
- 이번 자료로 판정하지 못한 이유: Isaac 이 같은 seed 에서도 결정적이지 않아, 실패가 반복마다 팔을 옮겨 다녔다(1 절). 그래서 개별 반사실은 재현할 수 없다. 빈도 비교(조건 2)는 더 큰 무작위 표본으로 할 수 있지만 하지 않았다(5 절).
- 근접 구간 비교의 team-spec 표본(14 미션)은 reported(148)의 10 분의 1 이라, 그 범위는 기술 통계일 뿐이다.
- 결정의 근거:
  - reported 는 엔진 관절 속도를 적분한 시뮬의 산물이고, 과도 구간에서 실제 바퀴 회전과 계통적으로 어긋난다.
  - 근접 구간 오도메트리 오차: reported −21.9 ~ +25.1 mm(148 미션), team-spec +1.0 ~ +2.1 mm(조사한 14 미션). 과삽입 금지 접촉 두 건이 둘 다 reported 였다(`docs/plans/2026-10-25-insertion-depth-tail.md`).
  - hold 드리프트와 삽입 깊이 Δ(3 절 표와 그 아래).
  - team-spec 에만 보였던 세 유형은 관절각 사슬의 결함이 아니었다(2 절, 미끄럼 오탐은 PR #135 로 해소).
  - 실차는 각도 엔코더(MT6701·KTH7801)를 단다.
- 롤아웃: `docs/plans/2026-10-25-team-spec-default.md`.

## 4. 남는 것(다른 작업)

- 2.1 return_home 정체: 계획 여유–허가의 탈출(출발 자세 무효)이다. 두 모델 공통이다.
- 2.2 도킹 측면 어긋남: sw-연속이송 세션이 D7 재접근 곡률을 맡았다. 측면 0.2 m 의 원천(삽입 끝 SLAM 측면 오차 0.16 m)은 별건이다.
- 2.3 미끄럼 판정: 시뮬레이션 판정 규칙이다.
  - 팔레트 원점의 절대 측면 대신, 꺼내기 시작 때의 포크–팔레트 상대 자세에서 얼마나 움직였는지로 보는 안이 있다.
  - 실제 미끄럼과 처음부터 치우친 적재를 함께 막으려면 두 조건을 나눠야 한다.
- 승격의 영향 범위(기본값을 바꿀 때 함께 바꿀 것):
  - 러너 `--encoder-model` 기본값.
  - 콘솔 런처 `ENCODER_MODEL` 기본값.
  - ws1 의 런처들(riib_scene·ct1_far·bd 등). reported 나이 표를 명시로 넘겨, 러너 기본값만 바꾸면 거부된다.
  - 근접 추적 계약(`POST_MEASUREMENT_ARGUMENTS` 의 `encoder_model`): 측정 실행과 같아야 해서, 근접 추적 실행은 reported 를 명시해야 한다.
  - 진행 중인 다른 세션의 두 팔 비교가 스냅숏 갱신으로 섞이지 않게 시점을 맞춘다.

## 5. 다음 단계(제안)

- 조건 2 를 가르려면, 두 유형이 같은 스냅숏에서 팔별로 어떤 빈도인지 더 큰 표본이 필요하다.
  - 필요한 크기는 효과 크기와 검정력 가정에 달려 있어 여기서 정하지 않았다. 위 Fisher 값도 세션 안 미션의 의존성과 유형의 사후 선택을 고려하지 않은 참고값이다.
  - 묶음 제출(PR #130, 동시 세션 8)로 seed 를 넓혀 두 팔을 같은 방식으로 돌린다. 예: seed 5–14 × line_m·random_m × 두 팔 = 40 세션, 팔마다 약 60 미션.
- 표본 확대는 빈도를 알려 줄 뿐 조건 3 의 인과 경로를 배제하지 않는다. 조건 3 은 실패 사례마다 같은 시점의 두 팔 궤적·조향·SLAM 을 맞대어 보거나, 결정 모드(총괄 공지: RGB 렌더 비결정성을 막는 모드)가 생기면 같은 조건에서 오도메트리만 바꿔 다시 돌려 가린다.
- 조건 1 은 유효 수를 맞추려면, 같은 seed·반복의 두 팔 미션이 둘 다 삽입에 이른 것만 세도록 판정 규칙을 정의해야 한다. 지금 기준의 의미("유효 수 같음")를 공통 쌍으로 바꾸는 것은 판정 규칙 변경이라 이 문서에서 하지 않았다.
- 2.1·2.2·2.3 의 결함 자체는 모델과 상관없이 고칠 대상이다(4 절).

## 6. 한계

- Isaac 은 비결정적이다. 실패 수 7 대 7 은 이 표본에서 차이가 없다는 뜻이지, 같은 빈도의 증명은 아니다.
- team-spec P-5 의 yaw 꼬리(11 건 중 2 건)는 표본이 작다. 장부상 `angle_truth`(접촉 물리) 몫이지만, 모델과 무관한지는 미판정이다(조건 3).
- 실차의 관절각 사슬(엔코더 장착·미끄럼·타이어)은 M6 측정 과제다. 이 판정은 시뮬레이션 기본값에 대한 것이다.
- 원자료(ws1 `test_runs/live/`): `rep*_occ_4c53486_*`, `newtq_occ_4c53486_*`, 분석 `rep_analyze.jsonl`·`rep_analyze-11018.log`, `p5_yaw-11039.log`, `slip_margin-*.log`. 분석 스크립트는 `occ_analyze.py`, `occ_age.py`, `fail_census.py`, `slip_margin.py`, `ins_yaw.py`, `p5_yaw.py`, `rh_compare.py`.

## 7. 검토 기록

- Codex 1 차: P2 3·P3 2 → 조건 1·2 불충족·3 미판정으로 판정 정정, 팔 표지·시간대·거리 해석 보정.
- Codex 2 차: 새 P1/P2 없음(수렴). P3 두 건(6 절의 단정, 표본 확대로 조건 3 을 가를 수 없다는 점·Fisher 값의 한계) 반영.
