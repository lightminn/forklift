# 관절각(team-spec) 오도메트리를 기본으로 (2026-10-25)

총괄 결정(2026-10-25): SLAM 오도메트리의 기본 엔코더 모형을 `reported` 에서 `team_spec_quantized` 로 바꾼다. 롤아웃 지시는 셋이다.
1. 영향 목록 → 계획서 → Codex 수렴 → PR.
2. 진행 중인 비교는 지금 설정 그대로 끝까지 돌리고, 새 배치부터 전환한다.
3. 머지 때 모든 세션에 공지한다.

## 1. 결정과 근거

### 사전 등록 조건은 채우지 못했다

재판정 문서(`docs/validation/2026-10-23-joint-angle-odometry-rejudge.md` 3 절)의 승격 조건 1–4 를 PR #132(`docs/validation/2026-10-25-joint-angle-promotion-failures.md` 3 절)에서 판정했다. 결과는 조건 1·2 불충족, 3 미판정, 4 충족이다.

**이 결정은 사전 조건을 채우지 못한 상태에서 총괄이 근거를 따져 내린 예외 결정이다.** 조건은 바꾸지도, 폐기하지도 않았다. 판정 결과(불충족·미판정)도 그대로 남는다.

- 조건 1·2 는 이번 자료로 채우지 못했고, 조건 3 은 이번 자료로 판정하지 못했다.
  - Isaac 이 같은 seed 에서도 결정적이지 않아, 실패가 반복마다 팔을 옮겨 다녔다. 그래서 개별 실패의 반사실(같은 조건에서 오도메트리만 바꾸면 어떻게 되는가)은 재현할 수 없다.
  - 실패 빈도나 평균 효과의 비교(조건 2)는 더 큰 무작위 배정 표본으로 할 수 있지만, 하지 않았다. 조건 3 은 결정 모드나 쌍 궤적 대조가 필요하다.
  - 조건 1 은 원래 기준(유효 수 같음)으로 불충족이다. 공통 42 쌍의 보조 분석에서는 team-spec 이 작다. 이것은 보조 분석이지 요건 충족이 아니다.
- 총괄은 이 불확실성을 받아들이고 아래 근거로 결정했다.

### 근거

- **reported 는 시뮬의 산물이다.**
  - reported 는 엔진의 바퀴 관절 속도에 가정 잡음(σ 0.2 rad/s, 데이터시트 값 아님)을 더해 적분한다.
  - 과도 구간에서 실제 바퀴 회전과 계통적으로 어긋난다. 회전 중 가속에서 0.1 s 마다 3–6 mm 를 덜 세고, 이는 가정 잡음의 5–12σ 다(`docs/plans/2026-10-25-insertion-depth-tail.md` 1 절).
  - 실차는 각도 엔코더를 단다. 뒤바퀴에 MT6701(4,096 계수), 조향에 KTH7801 이다. team-spec 이 그 부품을 모형으로 삼은 것이다.
- **근접 구간(약 2.5 m, 14 s)의 오도메트리 앞뒤 오차**(같은 문서):
  - reported 63 실행 148 미션: −21.9 ~ +25.1 mm.
  - team-spec 5 실행 14 미션: +1.0 ~ +2.1 mm. 표본이 reported 의 10 분의 1 이라 이 범위는 기술 통계일 뿐이고, team-spec 의 꼬리는 아직 관측되지 않았을 수 있다.
  - 과삽입 금지 접촉 두 건은 둘 다 reported 다(오도메트리 −21.9 / −20.5 mm).
- **hold 드리프트**(PR #132, 공통 42 쌍): |중앙값| 1.40 대 3.92 mm, 최대 3.32 대 19.76 mm.
- **삽입 깊이 Δ**(PR #132): team-spec −0.8 ~ +6.7 mm, reported 최대 +23.4 mm.
- **team-spec 에만 보였던 실패 세 유형**(PR #132 2 절):
  - return_home 정체: 칸 표지 칠함과 P6 출발.
  - 도킹 뒤 복구: 0.2 m 측면 어긋남.
  - 미끄럼 판정 오탐: PR #135 로 해소.
  - 셋 다 관절각 사슬의 결함이 아니고, 다른 스냅숏의 reported 에서도 나왔다.
- 나이 표 넘침 0(조건 4).

## 2. 바꾸는 것

| 곳 | 지금 | 바꿈 |
|---|---|---|
| `deploy/console/isaac_live.sbatch` | `ENCODER_MODEL` 기본 reported | 기본 team_spec_quantized(층은 이미 모형을 따름). reported 는 명시로 계속 쓸 수 있다 |
| `deploy/console/slam_survey.sbatch` | 같음 | 같게 바꿈 |
| `deploy/riibotics/riib_scene.sbatch` | 엔코더 인자 없음(러너 기본 reported), 층은 NOISE 로만 | `ENCODER_MODEL`(기본 team_spec_quantized)을 넘기고, 층을 NOISE × 모형으로 고른다 |
| `config/obstacle_layer_single_a2_team_spec.yaml` | 없음 | 새로 둔다: `obstacle_layer_single_a2.yaml` 에 team-spec 나이 표(`obstacle_odometry_age_team_spec.json`) |
| ws1 `test_runs/bd_isaac/bd.sbatch` (저장소 밖, 유일한 bd 사본 — `jao3/bd.sbatch` 는 이제 없다) | 기본 reported | 머지 뒤 기본 team_spec_quantized |
| ws1 `test_runs/live/ct1_*.sbatch` (저장소 밖) | `isaac_live.sbatch` 를 부름 | 바꿀 것 없음. 스냅숏의 런처 기본을 따른다 |
| 러너 `--encoder-model` 기본 | reported | **그대로 둔다**(아래) |

riib_scene 의 층 선택:

| NOISE \ 모형 | reported | team_spec_quantized |
|---|---|---|
| const | obstacle_layer_single.yaml | obstacle_layer_single_team_spec.yaml |
| a2 | obstacle_layer_single_a2.yaml | obstacle_layer_single_a2_team_spec.yaml |

`ENCODER_MODEL` 값은 reported 와 team_spec_quantized 만 받는다. 다른 값은 러너 인자 검사 전에 런처가 거절한다. `team_spec_typical_inl` 은 해당 나이 표가 없으므로 받지 않는다.

### 러너 기본은 그대로 둔다

- team-spec 은 `--record-slam` 을 요구하고 `--pocket-check` 를 거절한다(인자 검사). 러너 기본을 바꾸면 SLAM 없이 러너를 직접 부르는 실행이 깨진다(도구·시험·게이트 재기록 스크립트). D5 포켓 검사 실행도 마찬가지다.
- 기록에 인자가 없으면 reported 로 읽는 계약은 그대로다(근접 계약 `POST_MEASUREMENT_ARGUMENTS`, `encoders.py`). 재생은 기록된 인자 이름공간을 되살리므로, 파서 기본이 바뀌어도 게이트는 영향을 받지 않는다. 그래도 러너 기본을 바꾸면 "인자 없음 = reported" 라는 두 계약과 어긋나는 혼동만 생긴다.
- 운영 실행(콘솔 live, CT-1·CT-2 배치, 리보틱스 장면, bd)은 모두 런처를 거친다. 런처 기본이 곧 실질 기본이다.
- 러너를 직접 부를 때는 지금처럼 모형을 명시한다. 러너 `--encoder-model` 도움말에 "운영 런처의 기본은 team_spec_quantized" 라고 적는다.

## 3. 섞지 않을 것

- 실행마다 `meta.json` 의 `encoder_model` 에 모형이 남는다. 집계 도구는 이 값으로 나눈다(`tools/p0a_measure.py` 는 모형이 다른 실행을 섞으면 거절한다).
- 새 기본으로 낸 결과를 판정하려면 같은 모형의 기준선을 새로 잰다(비교의 두 팔은 같은 모형). 기존 reported 기준선과의 직접 비교는 하지 않는다.
- reported 로 잰 기준선과 비교는 그대로 reported 기준선이다(게이트 f_p2b, CT-1·CT-2 V3/V4, 리보틱스 장면 결과 등). 새 기본으로 낸 결과와 한 표에 섞지 않는다. 비교의 두 팔은 같은 모형이어야 한다.
- 진행 중인 배치는 스냅숏 사본으로 돌아서 머지의 영향을 받지 않는다(두 세션 확인).
  - sw-연속이송-재관측: V4 hold 묶음 11205·11206, `lf_5d280e80`. 후속도 같은 스냅숏으로 낸다.
  - v-g2 조사 잡 재개: rcf 11088·11089, rcq 11090·11091, `rc_fdf0e789`. 추가가 필요하면 새 스냅숏에서 두 팔을 다시 같은 시간대에 낸다.
- riib `MODE=truth` 도 나이 표가 바뀐다(허가층이 나이 표를 쓴다). 이전 truth 결과와 같은 기준선으로 보지 않는다.
- **survey 지도 번들은 그것을 만든 층에 묶인다.** 지도 계약이 장애물 층 해시도 비교하므로, reported 층으로 만든 번들(ws1 `test_runs/live/s2_survey/r6`·`r7`, `live/s5_v5*.sbatch` 가 씀)은 새 기본(team-spec 층)으로 쓰면 러너가 `slam_map_mismatch` 로 시작을 거절한다(Codex 재현).
  - 옛 번들을 쓰는 실행은 `ENCODER_MODEL=reported` 를 명시한다.
  - team-spec 으로 쓰려면 `slam_survey.sbatch`(새 기본)로 다시 survey 해 새 번들을 만든다.
  - 머지 공지에 넣는다.
- 재생 게이트 29 건은 기록의 인자로 돈다. 바뀌지 않으므로 f_p2b 절차(게이트 기준 교체)는 필요 없다. 머지 전 게이트로 확인한다.

## 4. 검증

- riib 은 DRY_RUN 으로 MODE × NOISE × 모형 조합을 러너 파서까지 통과시키는 시험으로 고정한다(`tests/unit/tools/test_riib_scene_launcher.py`). isaac_live·slam_survey 에는 DRY_RUN 이 없으므로, 기본·거절·층 선택을 소스 시험으로 고정하고(`test_live_supervisor.py`, `test_slam_survey_bundle.py`) Isaac 연기 시험으로 실제 기동을 본다.
- 새 층 파일과 a2 층이 나이 표를 빼고 같은지 확인한다(시험).
- ws1 전체 pytest, 재생 게이트 29/29.
- Isaac 연기 시험(묶음):
  - isaac_live 기본(team-spec)으로 CT-1 queue line_m 한 세션.
  - riib_scene NOISE=a2 기본으로 한 실행.
  - slam_survey 기본(team-spec)으로 새 번들을 만들고, 그 번들로 isaac_live localization 한 세션(`MAP_BUNDLE`).
  - 나이 표 VALID, 미션이 도는지 본다. 성능 비교가 아니라 배선 확인이다.

## 5. 머지 뒤

- 모든 세션 공지: "이제 운영 런처의 기본 오도메트리는 team_spec_quantized. reported 기준선과 섞지 말 것. reported 가 필요하면 `ENCODER_MODEL=reported`."
- ws1 `bd_isaac/bd.sbatch` 기본을 바꾼다(사본을 남긴다).
- 프로젝트 지침(`CLAUDE.md`·`AGENTS.md`)의 "Current implementation decisions" 에 한 줄을 둔다.

## 6. 자체 검토·검토 기록

- ① 직접 재계산: 근거 수치는 과삽입 분석(`ins_tail/split.json`)과 PR #132 표에서 다시 옮겼다. 런처·층·나이 표 조합은 시험으로 확인했다.
- ② Codex 1 차(P1 없음, P2 2·P3 2):
  - 러너 기본 유지에 찬성했다. 세 런처의 MODE × NOISE × 모형 조합 전부가 파서와 `age_table_problem` 을 통과했고, 재생·근접 계약에 영향 없음도 확인됐다.
  - P2: survey 번들의 층 해시 → 3 절에 절차를 넣었다.
  - P2: "표본으로 판정 불가" 단정 → 1 절 서술을 "이번 자료로 판정 못 함, 불확실성을 받아들인 예외 결정"으로 좁혔다.
  - P3: DRY_RUN 없음 → 검증 방법을 고쳤다. P3: README → 고쳤다. jao3 bd 사본 → ws1 에 없음.
- ③ agy 독립 검토(Gemini 3.8 Flash, 1·2 를 보지 않음):
  - "조건을 바꾼 것이 아니다" 는 방어처럼 읽힌다 → 예외 결정이라고 바로 적었다.
  - 표본 크기 차(14 대 148)와 미관측 꼬리 → 1 절에 적었다.
  - 결과에 모형 태그가 있는지 → `meta.json` 의 `encoder_model` 과 집계 도구의 거절을 3 절에 적었다.
  - 새 기본의 기준선 → 같은 모형으로 새로 잰다고 적었다.
  - 반영하지 않은 것: 러너 기본도 바꾸라는 취지 → 기록 계약과 SLAM 없는 실행 때문에 두고, 도움말·지침에 적었다.

## 7. 연기 시험 결과 (ws1 묶음 11258, 스냅숏 `tsd_a024b404`, 2026-10-25)

4 절의 "survey + localization" 은 투어 파일이 지워져 다음 두 시험으로 대신했다. 옛 번들을 reported 로 쓰는 경우와, 새 기본에서 거절되는 경우다. team-spec 재survey 는 머지 뒤 localization 사용자의 운영 절차로 남긴다(3 절).

| 실행 | 모형·층 | 결과 |
|---|---|---|
| isaac_live 기본, CT-1 line_m P-1:D-1 | team_spec_quantized, single_team_spec | 미션 완료 |
| riib_scene NOISE=a2 기본, line_2 | team_spec_quantized, single_a2_team_spec | 하역까지 진행, `Pallet missed the green destination center`(도킹 fallback). line_2 하역 정밀도의 기존 약점이고 배선 문제가 아니다 |
| s5_v5(r7 번들) + `ENCODER_MODEL=reported` | reported | localization 미션 완료 |
| s5_v5(r7 번들) 기본 | team_spec_quantized | `slam_map_mismatch: obstacle_layer differs from the survey's` 로 시작 거절(기대대로) |

- 출력: ws1 `test_runs/live/tsd_smoke/`.
- 추가 근거: riib `rp_c9fad6e8/rp_line_1_default` 접촉도 같은 유형이다(reported, 오도메트리 −20.0 mm, 겨냥 0.3 mm). 짝 실행 fast 는 +6.8 mm 로 완주했다(`test_runs/live/ins_tail/depth_split_batch.py`).
