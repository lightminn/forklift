# 자이로 방향 오도메트리 — G2 실행 안전 설계 (v2)

작성일: 2026-10-21 · 상위: [자이로 방향 오도메트리 검증](2026-10-21-gyro-heading-odometry.md) v4 의 G2·G3(요구사항)과 V-G0 합격(4.1)
· 이 문서는 그 요구사항의 **구체 설계**다: 러너에서 자이로 방향 오도메트리가 hold·고장·정지 판정·장애물 층·근거리·기록·재생과 어떻게 맞물리는지.
구현은 이 문서가 3 회 검토(재계산·Codex·agy)를 통과한 뒤 G1–G3 로 한다. 기본값(`--odometry-heading steering`)의 실행은 지금과 비트 단위로 같아야 한다.
· v1 → v2: Codex 1 회차(P1 4·P2 7·P3 2)와 agy 독립 검토(5)로 **설계를 줄였다** — 세션 안에서 조향 모드로 갈아타기(v1 D5)와 출발 때 강제 검증 scan(v1 D4)을
뺐다. 둘 다 임무 종료·복구·장애물 기억·오차 표·재생·비동기 기한과 얽혀 닫히지 않았다(5.2). 상위 계획 G2 의 그 두 요구도 이에 맞춰 고친다.

## 1. 지금 구조 (main e3868a2 기준)

- 오도메트리 상태 `slam["odom_rear"]`(후축, 연속 yaw)는 물리 틱마다 `IncrementalWheelOdometry.update_increments`(팀 인코더) 또는 `update`(보고값 +
  잡음)로 나아간다(`run_transport.py` 5570–5583). yaw rate `slam["odom_yaw_rate"]` = (새 yaw − 앞 yaw) × 120.
- yaw rate 소비자: 러너의 `StopDetector.update(yaw_rate=…)`(5596), 근거리 추적의 정지 프레임 승인과 불확실성(3672·5501, 3927), 라이브 피드백(6071–6073 —
  안전 중재기에 `yaw_rate_radps` 로 가고 중재기의 **자기 StopDetector**(`safety/arbiter.py` 545)가 쓴다). 경계 재생은 `slam` 의 `odom_yaw_rate` 등을 상태 키로
  복원한다(6298).
- `SlamPoseTracker`: warming_up → tracking → holding → release. release 는 받은 보정을 적용하며, 자세 점프가 `release_limits`(0.02 m·0.02 rad,
  `state["slam_feedback"]`)를 넘으면 `slam_release_*` 로 임무가 실패한다(MISSION). 임무의 release gate 는 `mission/core.py` 3998, constrained/unknown 화물은
  해제 불가(847).
- 장애물 층: 관측 셀 나이마다 오도메트리 오차 표로 팽창·FREE 축소를 정한다; 적용 검사 `encoders.py` 289 `odometry_table_problem` 은 인코더 모형과 바퀴
  반지름만 본다. 표를 만드는 측정은 `tools/p0a_measure.py`.
- prewarm: live 이고 비동기 링크가 켜졌거나 localization 일 때만(6374). mapping 은 5 번째 시작 scan(약 0.408 s, 49 틱)에 tracking.
- Isaac 차체 각속도: `robot.get_angular_velocity()`(월드 3 축).

## 2. 설계

### D1 자이로 모형 — 기록되는 서비스 (G1)

- 자이로 표본은 **어댑터 포트** `isaac.gyro_sample()` 이 낸다(물리 틱마다 한 번, 3 축 측정값 rad/s). 안에서: `w_body = R(q)ᵀ·w_world`(`get_angular_velocity`,
  `get_world_pose`), `w_imu = R_trueᵀ·w_body`(R_true = 주입 장착 오차 · 실행의 실제 카메라 장착 회전), 축마다 (1 + s)·w + b₀ + β·t + 랜덤워크 + 백색 N/√dt,
  양자화. RNG·적분 상태는 서비스 안에만 있고 러너의 `slam`·`state` 에는 값(표본)만 들어온다 — 경계 기록은 포트의 반환값을 기록하므로 재생은 기록값을
  그대로 받는다(Codex P2-10: 상태에 RNG 객체를 두면 codec 이 `$opaque` 로 거부한다).
- 매개변수 `config/gyro_model_*.yaml`(형식 `gyro_model/1`). 결과 `gyro_model` 에 내용·해시·실제 b₀·온도 부호를 남긴다. 표본은 spool `slam_log.gyro` 에
  남기고 `meta.json` 의 spool 그룹에 등록하며, `load_slam_log` 가 읽는다(지금은 joint·scan·silenced 만 읽는다).

### D2 방향 적분과 편향

- 알고리즘은 아는 회전(실제 카메라 장착 회전만)으로 `w_imu` 를 차체로 되돌린 수직 성분 ω_raw 를 쓴다. 방향 증분 = (ω_raw − b̂)·dt, 거리 = 팀 인코더 증분
  평균 × 반지름, 위치는 구간 중간 방향. `IncrementalWheelOdometry.update_increments_heading(stamp, increments, dyaw)` 을 더한다(조향 곡률 상태는 쓰지 않음).
- 정지 블록: 두 뒤 바퀴 계수 변화 0 **그리고** 그 틱의 적용 명령 속도 0 **그리고** 블록 안 ω_raw 분산 ≤ k·N²/dt 인 연속 60 틱(T_zupt 0.5 s)이면
  b̂ ← 블록 평균. 블록은 겹치지 않는다. 마지막 갱신의 끝 시각 `t_b` 를 둔다.
- **편향 준비 전 구동 금지**: b̂ 가 아직 없으면 러너는 바퀴 명령을 0 으로 둔다 — prewarm 이 도는 경로든 아니든(batch, 비동기 끔 mapping 포함; Codex
  P2-7). warm-up 이 블록보다 먼저 끝나면(mapping 은 49 틱) 구동이 블록까지 늦춰진다. 상한은 `slam_prewarm_timeout` 과 같은 60 s.
- **편향 준비 전 yaw rate 소비자**(러너·중재기의 StopDetector, 근거리, 피드백)는 지금의 조향 차분 값을 받는다 — 정지 중이면 계수 변화 0 이라 0 이다.
  준비 뒤에는 보정된 자이로(ω_raw − b̂)를 받는다. 둘 다 같은 `slam["odom_yaw_rate"]` 하나로 흐르므로 소비자 쪽은 바뀌지 않는다.
- 전제(계획 G2): 뒤 바퀴 계수가 변하지 않고 명령이 0 이면 차체는 돌지 않는다.

### D3 hold 의 방향 오차 상한 (편향 나이 포함)

- hold 진입 시각 t₀ 의 **편향 나이** a = t₀ − t_b + T_zupt/2(블록 평균은 블록 가운데 시각의 편향). hold 진입은 a ≤ `bias_age_max_s`(기본 60 s)일 때만
  허용한다: 넘으면 hold 직전 정지(도킹은 근접 촬영에서 이미 선다)에서 블록 하나를 기다린 뒤 hold 한다 — 그동안 바퀴 0.
- hold 동안 방향 오차의 보수적 상한 U(t), t = 지금 − t₀, 구간 i 는 hold 안의 편향 갱신으로 나눈 구간(각각 길이 tᵢ, 시작 나이 aᵢ — 첫 구간 a, 이후 T_zupt/2):
  - 확률 부분 σᵢ = √(N²·tᵢ + σ_b²·tᵢ² + σ_rw²·[(aᵢ + tᵢ)³ − aᵢ³]/3), σ_b = N/√T_zupt. 구간끼리는 **상관이 있다**(한 블록의 잡음이 앞 구간 적분과 다음
    편향에 함께 든다, Codex P2-11) — 그래서 분산을 더하지 않고 **표준편차를 더한다**(완전 상관 상한): S = Σ σᵢ.
  - 체계 부분 C = Σ [ s_max·|Δψᵢ| + β_max·(aᵢ·tᵢ + tᵢ²/2) ](선형 온도 기울기가 나이 aᵢ 의 편향 위에 더하는 오차, Codex P1-2).
  - 판정: C + 3·S > `hold_heading_limit_rad`(기본 0.02). 단위: N rad/√s, σ_b rad/s, σ_rw rad/s/√s, β rad/s².
  - 3·S 는 "가정한 모형에서 고정 시각의 영평균 정규 꼬리" 의 상한이지 실행 전체 최대 초과율도, 결정론적 안전 상한도 아니다(문서·상태에 그렇게 적는다).
- 넘으면 `slam_hold_heading_bound`: 바퀴 0, **세션 실패**(D5·D6). hold 를 풀지 않고 수동 복구 절차도 이 세션 안에서 열지 않는다(Codex P2-6 — 세션 실패는
  제어 루프를 끝내므로 D14 를 이 세션에서 쓸 수 없다). 시뮬은 감독자가 새 에폭으로 장면을 초기화하고, 실차는 사람이 복구한다.

### D4 흡수된 회전의 감시 — 기존 장치로 (v1 D4 의 강제 검증 scan 대신)

- tracking 중에는 SLAM 보정이 매 keyframe 방향을 고친다 — 편향이 실제 회전을 흡수해 틀어진 방향은 다음 보정에서 지워진다. 정지 블록 갱신 뒤 첫 processed
  보정의 방향 점프 |Δψ_corr| 를 기록하고 `bias_check_rad`(기본 0.02) 를 넘으면 사건 `bias_suspect` 와 함께 그 갱신을 되돌린다(앞 b̂ 로). 구동은 막지 않는다.
- holding 중에는 보정을 받되 적용하지 않으므로 같은 점프가 쌓인다 — 기존 release 점프 한도(0.02 rad)가 release 에서 잡아 임무를 실패시킨다(MISSION, 지금
  그대로). 이 동안의 방향 오차는 D3 가 모형 오차만 세므로 **흡수된 회전은 D3 에 들지 않는다** — 남는 위험이다: 정지 중 미끄러지거나 밀린 회전이 hold 직전
  편향에 흡수되면 삽입 중 방향 오차가 release 까지 보이지 않는다. 삽입은 근거리 포켓 추적(카메라)이 직접 맞추므로 포크 진입은 그 추적이 막는다는 것이
  남는 위험의 경계다. 이 위험은 V-G1 의 고장 주입(정지 중 밀림)과 실차 G4(포크 끼임·밀기, agy 5)로 잰다.
- 이렇게 하면 비동기 링크·keyframe gate·응답 기한은 바뀌지 않는다(Codex P2-5 를 피한다).

### D5 자이로 고장 — 세션 실패 (v1 D5 의 조향 모드 전환 대신)

- 고장: 자이로 표본 시각 간격 > `gyro_gap_s`(기본 50 ms, 모형 파일; 실차 USB 지터는 G4 로 잰 뒤 다시 정한다, agy 4), NaN, |ω| > 범위, 시각 역행.
- 처리: 그 틱부터 바퀴 0, `slam_gyro_fault`(SESSION). 끊긴 구간을 다른 공급원으로 잇지 않는다 — 세션이 끝나므로 이을 필요가 없다(Codex P2-8). 장애물
  기억·오차 표·근거리 경계를 갈아 끼우지 않는다(Codex P1-3, P1-4, P2-9). 시뮬은 고장 주입 인자(`--gyro-fault-at <sim s>`)로만 생긴다.
- 감독자는 `slam_gyro_fault`·`slam_hold_heading_bound` 를 지금의 3 회 규칙으로 다룬다(일시 고장일 수 있다). 새 에폭은 새 b̂ 를 잡는다.

### D6 실패 분류

- `slam_gyro_fault`, `slam_hold_heading_bound`: SESSION. `bias_suspect`: 사건(실패 아님). `failures.py` 표와 콘솔 사유 표에 넣는다.

### D7 계약 (G3)

- 러너 인자: `--odometry-heading {steering,gyro}`(기본 steering), `--gyro-model <yaml>`(gyro 일 때 필수, `--encoder-model team_spec_quantized` 필수),
  `--gyro-fault-at <s>`(시험용). 셋 다 `POST_MEASUREMENT_ARGUMENTS`(steering·None·None), 셋 다 기본값이면 결과에 기록하지 않는다(Codex P3).
- 기본 steering 의 비트 동일: 자이로 서비스는 gyro 일 때만 만들고 부르지 않는다(RNG 도 만들지 않는다); 어댑터 포트를 기본 경로에서 부르지 않으므로
  경계 기록도 같다; 새 `slam`·`state` 키는 gyro 일 때만.
- 장애물 나이 표: 머리에 `odometry_heading`·`gyro_model_sha256`·`zupt` 를 더하고 `odometry_table_problem` 이 대조한다(조향 표에는 `steering` 이 기본). 자이로
  표는 `tools/p0a_measure.py` 로 자이로 오도메트리에서 잰다. 표 없는 자이로 실행은 장애물 층과 함께 쓸 수 없다. 조향 대체 표는 필요 없다(D5).
- 근거리 경계·구동 항: 계약 비교가 `odometry_heading` 을 보므로 자이로 조건의 측정 전까지 근접 추적 실행은 거부된다.
- spool `slam_log.gyro`, `slam.heading`(편향 갱신·`bias_suspect`·hold 상한 추이); 결과 `heading` 요약.
- 경계 재생: 어댑터 포트 `gyro_sample` 을 재생 adapters 에 기록값 반환으로 둔다; 새 setup 이름(자이로 서비스·설정)은 `SETUP_NAMES_ADDED`(off = steering).
  자이로 실행 기록 하나의 재생 사례(V-G1).
- 콘솔: 상태 `slam.heading`(선택): `{"mode": "gyro", "bias_age_s", "hold_bound_rad"}`. 계약 문서 추가만.

## 3. 검증 (구현 단계, V-G1 상세)

1. 모형 통계: 잡음 밀도·편향·축척·랜덤워크를 표본에서 재추정(허용 5 %), 장착 회전(차량 yaw → 센서 −y, 되돌린 수직 성분 = ω).
2. 정지 블록: 회전 + 바퀴 계수 변화 → 갱신 없음; 명령 ≠ 0 틱 → 블록 버림; 분산 초과 → 버림; 겹치지 않음.
3. 편향 준비 전 구동 금지: mapping·batch·비동기 끔 경로 각각에서 첫 블록 전 바퀴 명령 0, 소비자 입력이 조향 차분과 같음.
4. hold 상한: 몬테카를로(같은 모형, 고정 시각 t = 5·10·20 s, hold 1 000 회, 나이 a = 0·30·60 s 각각)에서 실제 |오차| 가 C + 3·S 를 넘는 비율 ≤ 1 %
   (보수 상한이라 0.27 % 아래가 기대지만 1 000 회의 표본 흔들림을 두고 1 % 로 판정한다).
5. 감시: 정지 중 0.01 rad/s 흡수 회전 CPU 반례에서 첫 보정 점프로 `bias_suspect`·되돌림.
6. 고장: 주입 → 바퀴 0 → `slam_gyro_fault` 세션 끝(결과·exit 파일의 이유).
7. 기본값 steering: 지금의 오도메트리·두 StopDetector 입력·결과와 비트 단위 같음, replay gate 29/29.
8. Isaac: V-G2(조사 3 회)·V-G3(임무 정확도) 는 자이로 계획대로.

## 4. 남는 위험·열린 것

- D4 의 남는 위험(흡수된 회전이 hold 안에서 release 까지 안 보임) — V-G1 고장 주입과 G4 로 잰다.
- 긴 연속 주행 중 편향 갱신 없음(agy 3): tracking 은 SLAM 보정이 방향을 고치므로 지도 위치에는 크게 영향이 없고, hold 는 D3 의 나이 조건이 막는다.
  조사·임무의 정지 간격 분포를 V-G2 에서 적는다.
- 마스트 기울기·진동(agy 1)과 실차 USB 간격(agy 4), 정지 상태 정합 흔들림(agy 2, D4 의 한도 0.02 rad 근거)은 G4 와 실차 시험에서 잰다. 시뮬은 기울기
  0.001° 이하라 보이지 않는다.
- 기본 한도(hold 0.02 rad, bias_check 0.02 rad, 나이 60 s, 간격 50 ms)는 가정이다 — V-G2 결과와 G4 로 다시 정한다.

## 5. 자체 검토

### 5.1 재계산 (2026-10-21)

1 장의 행 번호를 main(e3868a2)에서 다시 확인했다(release gate 3998·847, 나이 표 `encoders.py` 289, 측정 도구 `tools/p0a_measure.py`, prewarm 조건 6374,
중재기 StopDetector `arbiter.py` 545). 50 ms = 120 Hz 의 6 틱, mapping 의 tracking 진입 0.408 s = 49 틱 < 블록 60 틱(편향 준비 전 구동 금지가 필요한 이유),
localization 의 7 번째 응답 0.608 s = 73 틱(S5 연기 실행 job 7587). Codex 의 온도 예(나이 1 000 s, t = 10 s, β = 0.15 °/s / 1 000 s = 2.618e-6 rad/s²):
β·(a·t + t²/2) = 2.618e-6 × (10 002.5 + 50) = 0.0263 rad — v1 식(β·t²/2 = 0.00013 rad)이 크게 과소였다.

### 5.2 Codex 1 회차 (v1, P1 4·P2 7·P3 2) → v2

| 지적 | 처리 |
|---|---|
| P1 D4 검증을 출발까지 미룸 | 강제 검증 scan 을 빼고 D4 를 기존 장치(tracking 보정 점프 → 되돌림, holding 은 release 점프 한도)로; 남는 위험 명시 |
| P1 D3 편향 나이 무시 | 나이 a 를 넣은 온도·랜덤워크 식, hold 진입 나이 상한 60 s |
| P1 D5 추정 무효가 종료·복구와 안 맞음 | 세션 안 전환을 빼고 자이로 고장은 세션 실패 |
| P1 장애물 기억 초기화 전제 | 기억을 초기화하지 않음(세션 실패) |
| P2 비동기 1 s 는 검증 기한이 아님, force_keyframe 경로 | 강제 검증 scan 을 쓰지 않음 |
| P2 D14 와 SESSION 양립 불가 | hold 상한 초과는 세션 실패, 이 세션 안의 수동 복구 없음 |
| P2 초기 보정 대기가 모든 경로를 못 막음, 소비자 불완전 | 편향 준비 전 구동 금지(모든 경로), 준비 전 소비자는 조향 차분(중재기 detector·피드백 포함) |
| P2 끊긴 구간 적분 상태 | 세션 실패로 이을 필요 없음 |
| P2 조향 대체 표 | 필요 없음 |
| P2 재생 codec 이 RNG 를 못 담음 | 자이로를 기록되는 어댑터 포트로 |
| P2 독립 가정·3σ 의미 | 표준편차 합(완전 상관 상한), 3σ 의 의미를 고정 시각 꼬리로 명시, 시험 판정 고정 |
| P3 `--gyro-fault-at` 기록, wire 필드 | 세 인자 기본값 미기록, 기본 경로에서 포트·RNG 없음 |
| P3 `odometry_age_table.py` 없음 | `tools/p0a_measure.py` 로 |

### 5.3 agy 독립 검토 (Gemini 3.8 Flash High, v1 본문만)

마스트 기울기·진동(실차 G4), 정지 정합 흔들림 0.01 rad(한도를 0.02 rad 로 올리고 실패가 아니라 사건으로 낮춤), 긴 연속 주행(D3 나이 조건, V-G2 기록),
USB 간격 50 ms(모형 파일 값, G4 로 다시 정함), 포크 끼임·밀기에 의한 흡수 회전(D4 남는 위험, V-G1 고장 주입·G4).
