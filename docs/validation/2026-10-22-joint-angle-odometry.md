# 관절각 오도메트리 전후 비교 (2026-10-22)

계획은 `docs/plans/2026-10-22-joint-angle-odometry.md`(3판), 브랜치는 `feat/joint-angle-odometry` 다.

**결론: 기준선 승격 보류.**
- ws1 `bd.sbatch` 의 기본값은 `reported` 로 둔다. `ENCODER_MODEL=team_spec_quantized` 는 선택으로만 쓴다.
- 승격 조건 네 가지를 모두 충족하지 못했다(아래 "승격 판정").
- 막는 것 두 가지:
  - 추종기 다리 끝 교착: 정지 중 오도메트리가 고정되면 드러난다.
  - 팀 규격 나이 표가 미션 주행을 덮지 못한다.

## 무엇을 비교했나

- 장면: EPAL 6, 포크 간격 0.3725 m, 기본 장면, 공장 홀.
- 조건: `--bottom-deck-check` 끔·켬 × seed 0–5, 그리고 `--near-capture-redraw always`.
- 스냅숏: `jao_f7f68be`. 실행당 `MAX_SIM=1500`.
- 두 조합을 seed 마다 짝으로 돌렸다(24 회).

| 조합 | 엔코더 | 장애물 층 나이 표 |
|---|---|---|
| reported (`jao_rp_*`) | `reported`: 엔진 바퀴 각속도 + 가우스 잡음 | `config/obstacle_odometry_age.json` |
| team-spec (`jao_tq_*`) | `team_spec_quantized`: MT6701 4096 계수/회전·히스테리시스 0.088°, KTH7801 12비트 조향, 가우스 잡음 없음 | `config/obstacle_odometry_age_team_spec.json` |

- 이 비교는 엔코더 하나가 아니라 조합의 전후다. 바퀴 입력, 조향 양자화, 잡음 제거, 나이 표가 함께 바뀐다.
- 시험 실행 1 회(`jao_smoke_tq_off_s0`, team-spec 검사 끔 seed 0)를 먼저 돌렸다.
  - 사이클을 완주했다.
  - 정지·저속 구간에서 속도 명령의 부호 반전은 0 회였다.
  - 0 ↔ 0 아님 전환은 reported 실행보다 적었다.

## 결과

### 사이클

| 조건 | reported | team-spec |
|---|---|---|
| 검사 끔 | 5/6 완주. s5 는 운반 중 `cusp_position_stall` | 5/6 완주. s5 는 observe 추종 시간 초과 |
| 검사 켬 | 5/6 완주. s0 은 observe 추종 시간 초과 | 4/6 완주. s1 은 return_home `obstacle_no_progress`, s2 는 근거리 `bottom_deck_target_mismatch` |

같은 seed 에서 reported 성공 → team-spec 실패로 바뀐 쌍은 두 개다.

- **`on_s1` — 오도메트리 조합이 원인이다(추종기 다리 끝 교착).**
  - return_home 다리 끝에서 차가 truth (−2.0117, −0.0693) 에 멈췄다.
  - 속도 명령은 6.14×10⁻⁵ m/s 로 57 s 동안 고정됐다. 추종기 속도 상한 `1.5 × remaining` 에서 나온 값이고, 오도메트리상 남은 거리는 약 0.04 mm 다.
  - 도착 판정은 명령이 정확히 0 이어야 한다. 이 명령으로는 차가 움직이지 않는다.
  - 그동안 team-spec 계수는 1–2 계수만 바뀌어 오도메트리가 고정됐고, 남은 거리가 줄지 않았다. 60 s 진행 감시가 실패를 냈다.
  - reported 에서는 바퀴 각속도 잡음이 추정 위치를 흔들어 이 교착이 가려졌던 것으로 본다.
  - 실차 엔코더도 정지 중 계수가 고정이므로 같은 일이 날 수 있다.
- **`on_s2` — 두 모델 공통의 근거리 대조 편향이 경계를 넘었다.**
  - 근거리 대조 목표가 truth 에서 0.14 m(대부분 y), yaw 로 −0.12 rad 떨어져 있었다. 기록 중심과의 거리는 0.157 m 로 허용 0.15 m 를 넘었다.
  - 같은 seed 의 reported 실행도 같은 쪽으로 0.12 m, yaw −0.11 rad 떨어져 있었고, 기록 중심과 0.110 m 로 통과했다.
  - 셋째 촬영의 SLAM 측면 오차는 team-spec −4.9 cm, reported −3.8 cm 였다.
  - 편향 자체는 오도메트리와 무관하다. 오도메트리 차이가 경계를 넘긴 몫은 분리하지 못했다.

### 삽입 깊이와 SLAM

완주했거나 들기까지 간 실행의 수치다.

| | reported | team-spec |
|---|---|---|
| 포크가 중심을 넘은 깊이(좌우 평균, truth) | 6.8–30.4 mm | 19.4–22.0 mm |
| 들기 기울기 최댓값 | 0.000031–0.000120 rad | 0.000036–0.000112 rad |
| SLAM 원시 위치 RMSE | 4.0–8.8 cm | 2.1–5.7 cm |

team-spec 은 깊이가 고르다. reported 의 얕은 쪽(6.8 mm `on_s3`, 7.5 mm `off_s1`)은 hold 구간 과대계상과 같은 방향이다.

### hold 구간 전진 드리프트

hold 구간은 채택된 근접 촬영부터 삽입 끝까지다. `tools/odom_drift_terms.py` 로 쟀고, 모든 실행이 기록 `odom_base` 와 오차 0 으로 재현됐다.

| | 유효 실행 | 부호 있는 범위 | \|중앙값\| | max(\|드리프트\|) |
|---|---|---|---|---|
| reported | 11 | −7.2…+16.8 mm | 0.04 mm | 16.8 mm |
| team-spec | 10 | +0.8…+3.5 mm | 1.9 mm | 3.5 mm |

- team-spec 의 드리프트는 거의 모두 `angle_truth` 항이다(바퀴 관절각과 차체 움직임의 차이, +1.0…+3.6 mm). `quantize` 항은 ±0.3 mm 안이다.
- reported 의 |중앙값|이 작은 것은 잡음의 부호가 상쇄되기 때문이다. 최악값은 team-spec 이 1/5 이다.

### 정지 상태에서 정지 감지기에 들어가는 속도

곁 세션 요청 항목이다. 정지 틱은 truth 후륜축 0.1 s 평균 속도가 1 mm/s 미만이고 속도 명령이 0 인 틱으로 잡았다.

| | 원시 σ | 원시 0.012 m/s 초과 | 0.1 s 평균 σ | 0.1 s 평균 초과 |
|---|---|---|---|---|
| reported | 0.017–0.018 m/s | 49–52 % | 0.0048–0.0055 | 1.6–6.4 % |
| team-spec | 0.0011–0.0023 m/s | 0–0.45 % | 0.0005–0.0007 | 0 % |

- team-spec `off_s5` 는 observe 에서 일찍 끝나 정지 틱이 59 개뿐이고 모두 0 이라, 표의 범위에서 뺐다.

team-spec 의 값은 이산이다. 한 바퀴 1 계수는 0.0115 m/s, 두 바퀴 또는 2 계수는 0.023 m/s 이고, 나머지 틱은 0 이다. 0 이 아닌 틱은 정지 틱의 0.9–3.7 % 였다.

### 나이 표 확인

장애물 층이 실제로 쓰는 나이 범위만 봤다. 각 스캔 시작에서 FREE 0.2 s, OCCUPIED 0.3 s 까지의 매 틱 오차를 그 틱 나이보다 크거나 같은 첫 표 값과 비교했다(ws1 `age_use.py`).

| | 표를 넘은 실행 | FREE 범위 초과 스캔(위치 / yaw) | 최악 비율 |
|---|---|---|---|
| reported, reported 표 | 0/12 | 0 / 0 | 0.82 |
| team-spec, team-spec 표 | 12/12 | 0–3 / 0–202 | FREE 1.55(위치 11.3 mm 대 표 7.8 mm, 검사 켬 observe), OCCUPIED 1.91(14.9 mm 대 7.8 mm, return_home 후진 선회) |

- 초과가 나는 곳은 return_home 의 0.6 m/s 후진 선회(조향 ±0.25 rad)와 검사 켬 observe 의 옆면 다리다.
- team-spec 표는 S2 조사 주행 r6/r7 만으로 만들었다. r6/r7 의 후진 속도는 −0.26 m/s 까지였다.
- 기존 reported 미션 실행 29 개를 팀 규격으로 다시 세어도 모두 이 표를 넘었다.
- 따라서 표를 넘는 원인은 표본 범위다. 엔코더 재계수와 적분 경로는 기록과 정확히 일치했다.
- 이 오차는 FREE 침식 반경을 바꾼다. 거리 4 m, 나이 0.2 s 에서 현재 표의 반경은 19.6 mm 다. 시험 실행의 최대값을 덮도록 키우면 30.2 mm 가 되고, reported 표는 119.1 mm 다. 실제 격자·허가 영향은 표를 바꿀 때 같은 입력으로 따로 재야 한다.

## 승격 판정

| 조건 | 결과 |
|---|---|
| 1. hold 드리프트의 \|중앙값\|·max(\|·\|) 가 reported 보다 작다(유효 수 같음) | 불충족. max 는 3.5 대 16.8 mm 로 작지만 \|중앙값\|은 1.9 대 0.04 mm 로 크고, 유효 수도 10 대 11 로 다르다 |
| 2. 조건별 실패 횟수가 늘지 않고 새 유형이 없다 | 불충족. 검사 켬에서 1 → 2 회, 새 유형 두 가지 |
| 3. 성공 → 실패 쌍의 원인이 오도메트리 조합이 아니다 | 불충족. `on_s1` 이 다리 끝 교착 |
| 4. 나이 표 초과 0 | 불충족. team-spec 12/12 |

## 다음에 필요한 것

결정은 총괄에 넘긴다.

1. **추종기 다리 끝 교착.** 다리 끝에서 명령이 0 으로 가는 조건이 필요하다. 지금은 남은 거리에 비례하는 속도 상한만 있어, 오도메트리가 정지 중 고정되면 끝나지 않는다. 같은 계열의 다리 끝 크리프는 곁 세션 `sw-연속이송-재관측` 이 다루고 있다. 그쪽은 관절각 전환 뒤에 범위를 정하기로 해서, 두 작업의 순서를 정해야 한다.
2. **팀 규격 나이 표 보완.** 후보 표를 만들었다. 입력은 r6/r7 + reported 미션 29 개를 팀 규격으로 다시 센 것이고, ws1 `test_runs/bd_isaac/jao_table_candidate.json` 에 있다.

   | 나이 | 0.1 s | 0.2 s | 0.3 s | 0.5 s | 1 s | 10 s |
   |---|---|---|---|---|---|---|
   | 후보 위치 오차 | 17.7 mm | 21.8 | 21.8 | 21.9 | 33.1 | 400.0 |
   | 현재 team-spec 표 | 7.7 mm | 7.8 | 7.8 | 7.8 | 14.1 | 144.5 |
   | reported 표 | 24.1 mm | 31.5 | 42.1 | 58.6 | 91.5 | 448.4 |

   - 공유 표(V-G2 조사도 쓴다)를 바꾸는 일이다. 표 출처·검증 절차를 계획에 먼저 적고 Codex 검토를 거친다.
   - 그다음 새 표로 team-spec 비교 실행을 새로 해야 한다. 이번 12 회는 옛 표로 만든 궤적이라, 새 표의 검증 자료가 될 수 없다.
   - 0.1 s 값이 0.2 s 의 80 % 로 튀는 원인도 그때 본다.
3. **근거리 대조 목표 편향**(`on_s2`, 두 모델 공통): PR #79 의 검사 켬 근거리 대조 경로다. 별건으로 기록한다.

## 재현

- ws1 `test_runs/bd_isaac/bd.sbatch`
  - `TAG=jao_{rp,tq}_{off,on}_s<seed> SEED=<seed> MAX_SIM=1500 ENCODER_MODEL={reported,team_spec_quantized} EXTRA_ARGS="--bottom-deck-check {off,on} --near-capture-redraw always"`, 스냅숏 `jao_f7f68be`
  - team-spec 이면 `config/obstacle_layer_single_team_spec.yaml` 을 고른다.
- 측정(ws1 `test_runs/bd_isaac/`):
  - `jao_compare.py`: 사이클·깊이·기울기·SLAM 오차
  - `jao_measure.sbatch`: `tools/odom_drift_terms.py`, `standstill_noise.py`, `age_use.py`
  - `age_where.py`, `age_where.sbatch`: 기존 미션의 재계수 초과
  - `table_candidate.sbatch`: 후보 표
  - `cmd_osc.py`: 저속 명령 반전
- 결과 파일: `jao_drift.json`, `jao_age_use_{tq,rp}.txt`, `jao_where/`, `jao_table_candidate.json`

## 검토 기록

- 계획: Codex 3 회 수렴, `agy` 독립 검토(Gemini 3.8 Flash Medium), 직접 재계산.
- 진단 도구: Codex 1 차 P2 1·P3 3. 반영 뒤 2 차에서 "새 P1/P2 없음".
- 나이 표 진단 상의(Codex): 승격 보류와 표본 범위 진단에 동의했다. 지적 네 가지를 이 문서에 반영했다.
  - 새 표로는 비교를 다시 해야 한다.
  - 검증 실행을 보고 배율을 고르지 않는다.
  - 초과는 장애물 층이 실제 쓰는 나이 범위로 센다.
  - FREE 영향은 격자로 잰다.
