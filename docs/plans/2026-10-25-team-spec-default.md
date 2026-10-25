# 관절각(team-spec) 오도메트리를 기본으로 (2026-10-25)

총괄 결정(2026-10-25): SLAM 오도메트리의 기본 엔코더 모형을 `reported` 에서 `team_spec_quantized` 로 바꾼다. 롤아웃 지시는 셋이다.
1. 영향 목록 → 계획서 → Codex 수렴 → PR.
2. 진행 중인 비교는 지금 설정 그대로 끝까지 돌리고, 새 배치부터 전환한다.
3. 머지 때 모든 세션에 공지한다.

## 1. 결정과 근거

### 사전 등록 조건은 채우지 못했다

재판정 문서(`docs/validation/2026-10-23-joint-angle-odometry-rejudge.md` 3 절)의 승격 조건 1–4 를 PR #132(`docs/validation/2026-10-25-joint-angle-promotion-failures.md` 3 절)에서 판정했다. 결과는 조건 1·2 불충족, 3 미판정, 4 충족이다. **이 결정은 조건을 바꿔 통과시킨 것이 아니다.** 조건은 그대로 두고, 근거를 따져 총괄이 결정했다.

조건 2·3 은 표본으로 가를 수 없다는 것이 드러났다.
- Isaac 은 같은 seed 라도 결정적이지 않다.
  - 같은 seed 반복에서 실패가 팔을 옮겨 다녔다. PR #127 에서 team-spec 이 실패한 seed 를 다시 돌리면 team-spec 이 완주하고 reported 가 실패했다.
  - 그래서 "team-spec 에만 있는 유형"(조건 2)은 표본을 늘려도 빈도만 알려 줄 뿐이다.
  - 조건 3(오도메트리 조합이 그 기전을 더 자주 부르는가)은 결정 모드가 있어야 가를 수 있다.
- 조건 1 은 원래 기준(유효 수 같음)으로 불충족이다. 공통 42 쌍의 보조 분석에서는 team-spec 이 뚜렷이 작다(아래).

### 근거

- **reported 는 시뮬의 산물이다.**
  - reported 는 엔진의 바퀴 관절 속도에 가정 잡음(σ 0.2 rad/s, 데이터시트 값 아님)을 더해 적분한다.
  - 과도 구간에서 실제 바퀴 회전과 계통적으로 어긋난다. 회전 중 가속에서 0.1 s 마다 3–6 mm 를 덜 세고, 이는 가정 잡음의 5–12σ 다(`docs/plans/2026-10-25-insertion-depth-tail.md` 1 절).
  - 실차는 각도 엔코더를 단다. 뒤바퀴에 MT6701(4,096 계수), 조향에 KTH7801 이다. team-spec 이 그 부품을 모형으로 삼은 것이다.
- **근접 구간(약 2.5 m, 14 s)의 오도메트리 앞뒤 오차**(같은 문서):
  - reported 63 실행 148 미션: −21.9 ~ +25.1 mm.
  - team-spec 5 실행 14 미션: +1.0 ~ +2.1 mm(표본의 기술 통계).
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
| ws1 `test_runs/bd_isaac/bd.sbatch` (저장소 밖) | 기본 reported | 머지 뒤 기본 team_spec_quantized |
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

- reported 로 잰 기준선과 비교는 그대로 reported 기준선이다(게이트 f_p2b, CT-1·CT-2 V3/V4, 리보틱스 장면 결과 등). 새 기본으로 낸 결과와 한 표에 섞지 않는다. 비교의 두 팔은 같은 모형이어야 한다.
- 진행 중인 배치는 스냅숏 사본으로 돌아서 머지의 영향을 받지 않는다(두 세션 확인).
  - sw-연속이송-재관측: V4 hold 묶음 11205·11206, `lf_5d280e80`. 후속도 같은 스냅숏으로 낸다.
  - v-g2 조사 잡 재개: rcf 11088·11089, rcq 11090·11091, `rc_fdf0e789`. 추가가 필요하면 새 스냅숏에서 두 팔을 다시 같은 시간대에 낸다.
- 재생 게이트 29 건은 기록의 인자로 돈다. 바뀌지 않으므로 f_p2b 절차(게이트 기준 교체)는 필요 없다. 머지 전 게이트로 확인한다.

## 4. 검증

- 런처 DRY_RUN 으로 모형·층 조합 넷(riib NOISE × 모형)과 isaac_live·slam_survey 기본을 확인하고, 시험으로 고정한다(`tests/unit/tools/test_riib_scene_launcher.py` 등).
- 새 층 파일과 a2 층이 나이 표를 빼고 같은지 확인한다(시험).
- ws1 전체 pytest, 재생 게이트 29/29.
- Isaac 연기 시험(묶음):
  - isaac_live 기본(team-spec)으로 CT-1 queue line_m 한 세션.
  - riib_scene NOISE=a2 기본으로 한 실행.
  - 나이 표 VALID, 미션이 도는지 본다. 성능 비교가 아니라 배선 확인이다.

## 5. 머지 뒤

- 모든 세션 공지: "이제 운영 런처의 기본 오도메트리는 team_spec_quantized. reported 기준선과 섞지 말 것. reported 가 필요하면 `ENCODER_MODEL=reported`."
- ws1 `bd_isaac/bd.sbatch` 기본을 바꾼다(사본을 남긴다).
- 프로젝트 지침(`CLAUDE.md`·`AGENTS.md`)의 "Current implementation decisions" 에 한 줄을 둔다.

## 6. 자체 검토·검토 기록

(Codex·agy 검토 뒤 적는다.)
