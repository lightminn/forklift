# 관절각 오도메트리 승격 재판정 (2026-10-23)

사용자 결정(2026-10-22, 총괄 경유): `team_spec_quantized` 오도메트리의 기준선 승격을 다시 판정한다.

순서는 다음과 같다.
1. team-spec 나이 표를 보완한다(`docs/plans/2026-10-22-team-spec-age-table.md`).
2. `on_s2` 근거리 SLAM 오차를 분해한다.
3. #105 위에서 24 회 비교를 다시 돌려 승격 조건 넷을 판정한다.

조건을 모두 채우면 기본값 변경은 사용자 결정으로 올린다.

첫 판정은 `docs/validation/2026-10-22-joint-angle-odometry.md` 다.

## 1. team-spec 나이 표

계획 `docs/plans/2026-10-22-team-spec-age-table.md`(6판, Codex 5 회 수렴)대로 `config/obstacle_odometry_age_team_spec.json` 을 바꿨다(커밋 8c80683a, sha256 `b853b34a…`).

- 입력은 194 개다. SLAM 로그가 있는 reported Isaac 미션 192 개(ws1 `test_runs/bd_isaac`, `test_runs/riibotics`)와 S2 조사 r6/r7 이다. 기록 바퀴 관절각을 `team_spec_quantized` 로 다시 셌다. 배율 1.0 은 검증 전에 정했다.
- 옛 표(r6/r7 만, `0bcf52a4…`)는 team-spec 미션 12 회가 모두 넘었다.

| 나이 s | 0.1 | 0.2 | 0.3 | 0.5 | 1 | 2 | 3 | 5 | 10 |
|---|---|---|---|---|---|---|---|---|---|
| 새 표 위치 mm | 19.3 | 23.6 | 25.7 | 28.6 | 33.1 | 43.4 | 50.0 | 134.6 | 421.2 |
| 새 표 yaw rad | 0.0080 | 0.0137 | 0.0142 | 0.0153 | 0.0189 | 0.0367 | 0.0544 | 0.0893 | 0.1379 |
| 옛 표 위치 mm | 7.7 | 7.8 | 7.8 | 7.8 | 14.1 | 15.8 | 22.2 | 38.2 | 144.5 |

- 짧은 나이 yaw 는 `riibotics/rk3_aisle_out_s0` 에서 나왔다. 적재 상태로 0.30 m/s 선회하다가 다리 끝에서 0.25 s 만에 서는 동안, 뒤 바퀴는 거의 멈췄는데 차체는 계속 돌았다.
- FREE 0.2 s 반경은 거리 4 m 에서 19.6 → 78.5 mm 다. FREE 침식 이웃은 9 → 21–25 개다(`free_r_cap_m` 0.20 m 안).
- V1(옛 표로 돈 team-spec 12 회를 새 표에 댐): 12/12 VALID, 넘침 0, 최악 0.58.

## 2. on_s2 근거리 SLAM 오차 분해

`jao_tq_on_s2` 는 셋째 촬영에서 근거리 대조 목표가 기록 중심에서 0.1576 m 떨어져 허용 0.15 m 를 넘었다(`bottom_deck_target_mismatch`). 그 목표는 SLAM 제어 자세로 관측을 세계 좌표에 놓아 만든다.

셋째 촬영 순간의 제어 자세 오차를 둘로 나눴다(ws1 `bd_isaac/slam_split.py`, `slam_records.json`).
- **hold 때 붙잡은 보정:** SLAM 이 hold 를 시작한 순간의 보정(`applied_map_from_odom`)을 그대로 두고, 오도메트리가 truth 와 똑같이 움직였다고 할 때의 오차다.
- **hold 뒤 오도메트리 드리프트:** 나머지다. 그 동안 오도메트리가 truth 와 다르게 움직인 몫이다.

| 실행(검사 켬, 마지막 촬영) | 전체 측면·yaw | hold 보정 | hold 뒤 드리프트 | hold 뒤 이동 |
|---|---|---|---|---|
| team-spec s2 | −5.8 cm, −0.024 rad | −4.2 cm, −0.0117 | −1.6 cm, −0.0127 | 1.03 m |
| reported s2 | −4.5 cm, −0.020 | −2.8 cm, −0.0069 | −1.6 cm, −0.0135 | 1.10 m |

첫 판정(`jao_*`) 검사 켬 12 회의 마지막 촬영으로 넓혀 보면:
- **hold 보정 측면 오차:** reported −3.8…−1.0 cm(중앙값 −2.4), team-spec −4.2…+0.4 cm(중앙값 −1.6). team-spec 이 계통적으로 나쁘지 않다. s2 의 −4.2 cm 는 꼬리다.
- **hold 뒤 yaw 드리프트:** 두 모델 모두 0.6–1.1 m 이동에 최대 0.015–0.019 rad 다. 이 기록 궤적에서는 바퀴 관절각과 차체 움직임의 차이(`angle_truth` 항, 조향 기하·미끄럼)가 지배적이고, 양자화 단독 결함으로는 설명되지 않는다. 고정된 궤적의 오차 항 분해라, 폐루프에서 오도메트리 조합 전체가 미친 영향까지 배제하지는 않는다.

그러니 `on_s2` 는 보정 오차의 꼬리와 yaw 드리프트가 더해져 근거리 대조 허용을 넘은 경우다. 양자화 결함으로는 설명되지 않지만, 오도메트리 조합과 무관하다는 증명도 아니다(3 절 조건 3).
- 고정 궤적 반사실 계산(Codex, 2026-10-23): hold 뒤 양자화만 없애면 거리 0.1586 m 로 여전히 넘고, 그 구간 이동을 truth 로 바꾸면 0.0976 m 로 통과한다. 새 폐루프 실행은 아니다.
- 첫 판정에서 허용 0.15 m 에 대한 여유는 두 모델 모두 작았다. reported s2 도 0.110 m 였다. 이번(`jao2_*`) reported s2 는 0.0246 m 다(3 절).
- 이번 `jao2_*` 검사 켬 마지막 촬영의 hold 보정 측면 중앙값은 reported −1.54 cm, team-spec −1.94 cm 다.
- 이 표의 전체 오차는 기록 `slam_error` 와 근사하게 맞는다. 재계산은 가장 가까운 SLAM 기록 시각의 base 기준이라 약간 다르다(예: −5.8 대 −4.9 cm).

## 3. 24 회 비교와 승격 판정

### 실행

- 스냅숏 `jao2_8c80683a` 는 #105 머지(689a020) 위에 새 표를 더한 커밋이다. 24 회 모두 이 스냅숏에서 돌았다. 스냅숏의 표 SHA-256 은 team-spec `b853b34a…`, reported `0e07ba94…` 다(측정 잡이 기록).
- 조건은 첫 판정과 같다: `--bottom-deck-check` 끔·켬 × seed 0–5, `--near-capture-redraw always`, `MAX_SIM=1500`. 태그는 `jao2_{rp,tq}_{off,on}_s<seed>`, ws1 job 9534–9557 이다.

### 사이클

| 조건 | reported | team-spec |
|---|---|---|
| 검사 끔 | 5/6. s5 는 운반 중 `cusp_position_stall`(첫 판정과 같음) | **6/6** |
| 검사 켬 | **6/6** | 5/6. s2 는 근거리 `bottom_deck_target_mismatch`(첫 판정과 같음) |

- 첫 판정의 team-spec 실패 셋 중 둘이 없어졌다. `off_s5`(observe 추종 시간 초과)와 `on_s1`(return_home 다리 끝 교착, #105 가 고친 기전)이다.
- `tq_on_s2` 는 첫 판정의 `jao_tq_on_s2` 와 `slam_log.npz` 가 SHA-256 까지 같고, 촬영 시각·SLAM 오차·`bottom_deck` 기록·주행 샘플도 같다(SLAM JSON 의 `slam_wall_s` 같은 벽시계 값은 다르다). 새 표와 #105 가 이 실행의 셋째 촬영까지는 영향이 없다. 2 절의 분해가 그대로 맞는다: hold 보정 −4.2 cm, −0.0117 rad, hold 뒤 드리프트 −1.6 cm, −0.0127 rad, 근거리 대조 거리 0.1576 m(허용 0.15 m, 7.6 mm 초과).
- 같은 seed 의 reported 는 이번에 셋째 촬영 SLAM 오차가 측면 −1.8 cm, yaw +0.001 rad 로 작아 통과했다(첫 판정 −3.8 cm, −0.020 rad). 근거리 대조 거리는 0.0246 m 다. reported 물리 궤적은 10.74 s 부터 첫 판정과 갈라진다. 그 순간 남은 길이 2.9 mm·종점 횡단 오차 83 mm 로 #105 의 다리 끝 하한 조건에 들어, 명령이 0.0043 → 0.010 m/s 가 됐다. 셋째 촬영 개선 전체를 #105 단독 효과로 확정하지는 않는다.

### 삽입·SLAM

| | reported | team-spec |
|---|---|---|
| 포크가 중심을 넘은 깊이(완주·들기까지, truth) | 7.5–25.2 mm | 19.4–22.0 mm |
| SLAM 원시 위치 RMSE | 3.7–8.8 cm | 1.6–5.5 cm |
| 삽입 끝 SLAM 오차 측면 | −5.1…+3.5 cm | −8.6…+3.2 cm |

### hold 구간 전진 드리프트(`tools/odom_drift_terms.py`)

| | 유효 실행 | \|중앙값\| | max(\|드리프트\|) |
|---|---|---|---|
| reported | 12 | 2.9 mm | 15.1 mm |
| team-spec | 11 | 1.5 mm | 3.5 mm |

- 모든 유효 실행이 기록 `odom_base` 와 오차 0 으로 재현됐다. team-spec 의 드리프트는 거의 `angle_truth` 항(+0.9…+3.6 mm)이고 `quantize` 항은 ±0.3 mm 안이다.
- team-spec 이 11 인 것은 `on_s2` 가 삽입 전에 끝나 구간이 없어서다. 공통 11 쌍만 보면 reported \|중앙값\| 은 2.9 mm 다.

### 나이 표 넘침(`age_use.py`, 사용 나이 FREE 0.2 s, OCCUPIED 0.3 s)

- team-spec 12/12 VALID(덮음 1.000), 넘침 0. 최악은 OCCUPIED 0.59(`on_s5`, observe, 위치 15.3 mm 대 25.7 mm)다.
- reported 12/12 VALID, 넘침 0, 최악 0.84.

### 가용성(계획 T3 의 A1–A4, V2 전에 정한 규칙)

| 규칙 | 결과 |
|---|---|
| A1 완료 시각 비 중앙값 ≤ 1.10 | 통과. 10 쌍, 0.913 |
| A2 occupied/unknown 제한 판정 시간 ≤ reported × 1.5 + 5 s | 통과. 5.3 s 대 18.4 s |
| A3 재계획 수 ≤ reported × 1.5 + 2 | 통과. 6 대 14 |
| A4 계획 실패 ≤ reported, 새 종류 없음 | 통과. 둘 다 0, 계획 전 종료 0 |

- 새 표로 FREE 침식이 커졌지만, 이 장면에서는 제한 판정·재계획이 reported 보다 적었다. 조합 비교이므로 이것을 표만의 효과로 읽지 않는다.

### 정지 상태 속도(`standstill_noise.py`)

- team-spec 정지 틱의 원시 속도 σ 는 0.0011–0.0023 m/s, 0.1 s 평균이 정지 문턱을 넘은 비율은 0 % 다. reported 는 σ 0.0173–0.0184, 0.1 s 평균 넘음 1.5–5.6 % 다.

### 승격 판정

| 조건 | 결과 |
|---|---|
| 1. hold 드리프트의 \|중앙값\|·max(\|·\|) 가 reported 보다 작다(유효 수 같음) | 값은 충족(1.5 대 2.9 mm, 3.5 대 15.1 mm). 유효 수가 11 대 12 로 달라 비교 요건은 불충족이다. 차이는 조건 2·3 의 `on_s2` 하나다. 공통 11 쌍의 reported 중앙값은 2.92 mm(보조 분석) |
| 2. 조건별 실패 횟수가 늘지 않고 새 유형이 없다 | 불충족. 검사 끔은 1 → 0 이지만, 검사 켬이 0 → 1(`bottom_deck_target_mismatch`) |
| 3. 성공 → 실패 쌍의 원인이 오도메트리 조합이 아니다 | 불충족. `on_s2` 한 쌍. 두 팔은 소스·장면·초기 상태가 같고 인자 차이는 엔코더와 장애물 층 설정뿐이다. 그 조합이 셋째 촬영의 붙잡은 보정과 hold 뒤 yaw 드리프트를 만들었다. 양자화 단독 결함은 아니지만(2 절), 그것이 조합과 무관하다는 뜻은 아니다 |
| 4. 나이 표 넘침 0 | 충족. 12/12 VALID, 넘침 0 |

넷을 모두 채우지 못했으므로 기본값 변경은 올리지 않는다. 남은 것은 `on_s2` 하나다.

- 이 실패는 근거리 대조 허용(0.15 m)에 대한 여유 문제다. 첫 판정에서 reported 도 같은 seed 에서 0.110 m 였다. 고정 궤적에서 hold 구간 이동을 truth 로 바꾸면 통과한다(2 절).
- 결정은 총괄·사용자에게 넘긴다. 선택지는 다음과 같다.
  - (a) 근거리 대조 목표를 SLAM 제어 자세 대신 다른 기준(예: 근거리 관측 자체의 팔레트 기준)으로 놓는 별건.
  - (b) hold 중 yaw 드리프트(`angle_truth`)를 줄이는 별건(자이로 헤딩, V-G2 계열).
  - (c) seed 를 늘려 조건 2 의 실패율을 더 정확히 재는 것. 다만 seed 를 늘려도 기존 `on_s2` 가 조건 3 을 막는 것은 그대로다.
  - 조건이나 허용값을 바꾸는 것은 별도 결정이다.

## 재현

- ws1 `test_runs/bd_isaac/bd.sbatch`, 스냅숏 `jao2_8c80683a`
  - `TAG=jao2_{rp,tq}_{off,on}_s<seed> SEED=<seed> MAX_SIM=1500 ENCODER_MODEL={reported,team_spec_quantized} EXTRA_ARGS="--bottom-deck-check {off,on} --near-capture-redraw always"`
- 측정: ws1 `test_runs/bd_isaac/jao2_measure.sbatch jao2_8c80683a`(표 SHA, `age_use.py`, `odom_drift_terms.py`, `jao_compare.py`, `jao2_avail.py`, `standstill_noise.py`). 로그 `jao2_meas-<job>.log`, 결과 `jao2_age_use_{tq,rp}.txt`, `jao2_drift.json`.
- on_s2 분해: ws1 `test_runs/bd_isaac/slam_split.py`.

## 검토 기록

- 나이 표 계획: 계획서 6 절(Codex 5 회 수렴, agy 독립 검토, 직접 재계산).
- 이 문서와 판정(Codex 새 스레드): ws1 접속이 중간에 막혀 원자료를 로컬로 복사해 넘겼다. 24 회를 독립 재계산해 판정표 수치가 반올림 안에서 맞음을 확인했고, "새 P1/P2 없음, 승격 보류 결론 타당" 이었다. P3 3 건을 반영했다: 근거리 거리 0.1576 m 표기, 2 절 분포가 첫 판정 실행임을 명시(이번 실행 값 추가), "엔코더 모형과 무관" 을 "이 궤적에서 양자화 단독 결함으로 설명되지 않음" 으로 좁힘. 조건 3 해석과 seed 확대의 한계, reported 궤적 분기(10.74 s, #105 하한)도 Codex 의 확인을 반영했다.
