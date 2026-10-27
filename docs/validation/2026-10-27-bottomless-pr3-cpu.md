# 하판 없는 팔레트 PR ③ CPU 부분 — 새 음성·양성 집단과 선반 판별 측정 (2026-10-27)

계획: `docs/plans/2026-10-27-bottomless-and-bundled-pallets.md`(PR #168) §3.1·§4.2·§4.3. 이 문서는 PR ③ 중 **CPU 부분**만 다룬다. Isaac 렌더·학습 제안기 평가·eval 읽기는 하지 않았다(맨 끝 "Isaac 단계에 남은 것").
branch `feat/bottomless-pr3-cpu`. 측정은 전부 **dev** 이고 CPU 장면 장치(`tools/scene_rig.py`) 렌더다. eval 은 읽지 않았다.

## 결론

질문: 하판 없는 팔레트를 양성으로 받은 뒤, 선반 계열·상자 받침을 grounded 와 가르는 증거가 CPU 장치에서 확실히 있는가. **모든 선반 변형을 가르는 하나의 증거는 없었다.**

- **정면 상부 띠 비 (ADR 0003 B-16, 개구부별 형태)**
  - 무잡음: 스트링거 띠가 빈 선반(`shelf`·`shelf_closed`·`shelf_legs`, EPAL `shelf_raised`)은 모든 거리(1.2–3.5 m)에서 양성보다 낮았다. 칸별 여유는 EPAL 6 ≥ +0.311, T11×0.6 ≥ +0.348 이었다.
  - 무잡음: 앞 레일 선반(`shelf_rail`)은 양성보다 **높았다**. 여유는 0.04(EPAL)·0.05(T11)뿐이고, 거리를 합친 범위는 양성과 겹친다. 문턱보다 낮으면 거부하는 형태로는 이 형상을 거르지 못한다.
  - 팀 잡음(1.2–1.8 m): EPAL 6 은 띠가 빈 선반 넷이 칸마다 양성 밖이었다(1.6 m 까지 ≥ +0.21, 1.8 m +0.04). 레일 선반과 모서리 기둥 선반은 양성 구간 안에 들어간 칸이 있었다. T11×0.6 은 띠가 빈 선반도 1.4·1.6 m 에서 양성 구간 안에 들어간 칸이 있었다(`shelf_legs` −0.37).
  - 팀 잡음(2.0–3.5 m): 양성 자체가 거의 검출되지 않아(EPAL 팔레트 7/36, T11 4/36) 판정할 수 없다.
- **옆면 구조 검사 (`bottom_deck_evidence`, 팀 잡음)**
  - 막힌 옆면 선반 넷(`shelf`·`shelf_closed`·`shelf_rail`·`shelf_raised`)은 `closed_face` 로 72/72 씩 거부했다.
  - `shelf_uprights`·`box_stand` 는 `rows_not_located` 등으로 거부했다(판별이 아니라 블록 열을 못 찾은 거부다).
  - 다리형 선반 `shelf_legs` 는 grounded 와 같은 `absent:open_gaps` 로 **72/72 허가**했다.
- **두 증거를 겹치면**
  - 레일 선반은 정면에서 못 거르고 옆면에서 거른다.
  - 다리형 선반은 옆면에서 못 거른다. 정면 비로 갈린 곳은 무잡음과 EPAL 6 팀 잡음 1.2–1.8 m 뿐이고, T11 팀 잡음에서는 갈리지 않았다.
  - 이 장치의 측정 범위에서 다리형 선반이 남는 구멍이다. 계획 §3.3 의 다른 방어(작업 지시 구역, LiDAR 평면)에 기대야 한다. LiDAR 평면은 다리형 받침이 1.05 m 아래로 끝나면 거르지 못한다.
- **데이터 경로 (dev)**
  - 근거리 검출기를 그대로 쓴 `baseline` 은 무잡음에서 N2 어려운 음성 89 장 중 64 장을 후보로 냈다(옛 N `shelf` 14/16, `grounded` 16/17).
  - 학습 제안기는 CPU 에서 돌지 않는다. `geometric-fit`·`oracle-fit` 은 음성 오검출을 재지 못한다(아래 "CPU 로 잴 수 없는 것").

## 무엇을 넣었나

### 새 음성 형상 (`tools/negative_structures.py`)

옛 `measure_pocket_evidence.structure` 의 `shelf`·`blocks`·`crates` 는 그대로 두고, 형상 치수(폭·깊이·높이·블록 열)를 팔레트 형상에서 받아 새 종류를 만든다. 상자 치수는 단위 시험(`tests/unit/tools/test_negative_structures.py`, 10 건)으로 고정했다.

| 종류 | 형상 | 어려운 음성 |
|---|---|---|
| `shelf` | 옛 형상. 기둥 위로 상판이 스트링거 높이만큼 떠 있다(상자 모형의 산물) | 예 |
| `shelf_closed` | 기둥을 상판 밑면까지 올림(뜬 틈 없음) | 예 |
| `shelf_rail` | 차선 천장에서 상판까지 앞·뒤 레일(폭 전체). 팔레트의 스트링거 띠와 같은 높이를 덮는다 | 예 |
| `shelf_raised` | 바닥판(10–30 mm) 위에 기둥과 상판 | 예 |
| `shelf_legs` | 다리 9 개(블록 열 × 3 행)와 통판 상판 | 예 |
| `shelf_uprights` | `shelf_rail` + 폭 바깥 모서리 기둥 4 개(1.15 m, LiDAR 평면 1.05 m 를 지남) + 0.55 m 윗단 | 예 |
| `box_stand` | 다리 4–9 개(2×2·2×3·3×3) 위 닫힌 상자 | 예 |
| `block_pile` | 바닥 위 서로 겹치지 않는 블록 4–8 개 | 아니오 |
| `blocks` | 바닥에 선 블록 9 개(옛 `blocks` 는 15 mm 떠 있다. 옛 N 은 그대로 둔다) | 아니오 |

### N2 — 새 음성 카탈로그 (`tools/recognition_scenes.catalogue_bottomless`)

- 153 장, 시드 1,100,000–1,100,152(eval_pool 대역). dev 107 · eval 46(`eval_count(153)`). eval 46 장은 오검출 1 건까지 단측 95 % 상한 9.9 %(`min_scenes(1, 0.10)` = 46).
- 구성은 17 칸 주기의 상수(`negative_structures.N2_CYCLE`)로 고정했다. 17 칸 중 14 칸이 어려운 음성이다. 실제 분할별 비율: dev 89/107, eval 39/46(둘 다 2/3 이상, 시험으로 고정).
- 종류별 장수 — dev: `shelf_legs` 19, `shelf_rail` 19, `shelf_closed` 13, `box_stand` 13, `shelf_raised` 12, `shelf` 7, `shelf_uprights` 6, `block_pile` 6, `blocks` 6, `crates` 6. eval: `shelf_rail` 9, `shelf_legs` 8, `box_stand` 6, `shelf_closed` 5, `shelf_raised` 5, `shelf` 3, `shelf_uprights` 3, `block_pile` 3, `blocks` 2, `crates` 2.
- 치수 모체는 순위에 따라 EPAL 6·T11×0.6 을 번갈아 쓴다.
- 평가 모집단 `negative_n2` 는 옛 `negative`(옛 N)와 합치지 않는다. 옛 카탈로그는 recipe digest 를 고정해 바뀌지 않음을 시험한다.

### S8 — 하판 없는 양성 (같은 카탈로그)

- `grounded_epal6` 266 장(시드 1,100,153–1,100,418), `grounded_t11_06` 266 장(1,100,419–1,100,684). 형상마다 dev 186 · eval 80.
- 지시는 S1 과 같은 단일 대상 규칙이라 eval 80 장 중 지시가 있는 장면이 60 장이다. 대상 오류·지시 보류 게이트의 분모는 이 60 장이고 `min_scenes(0, 0.05)` = 59 를 넘는다. 놓침·위험·여분 게이트는 80 장 전부가 분모다(Codex 1 차 P2: 처음 196 장은 지시 분모가 45 였다).
- T11 은 홀수 순위를 90° 돌려 y 면을 본다(133/266).
- 하판 변형은 `InstanceRecipe.bottom = "none"` 이 렌더·정답·시퀀스 렌더까지 따라간다: 하판 상자가 없고 블록이 바닥까지 내려온다. 현장 적재물은 모체 형상의 값을 쓴다.
- **slab 은 넣지 않았다.** 계획 §2.1 i(면 배치 수정)가 먼저다.
- 학습용 짝은 train 대역 5,100,000 부터(5,480 장, `training_catalogue_bottomless(scale=8)`). eval_pool 장면은 학습 입력이 아니다.
- 시드 대역 검사: 옛 카탈로그(1,000,000–1,001,019), `recognition_sequences.SEQUENCE_BANDS`(1,500,000–), 옛 train 목록과 겹치지 않음을 시험으로 고정했다.

### 평가 계약 (`src/forklift_core/perception/evaluation_multi.py`)

- 모집단 `known:grounded_epal6`·`known:grounded_t11_06`(놓침·위험·여분·대상 오류·지시 보류)과 `negative_n2`(오검출).
- S8 에는 높이 게이트가 없다(S1 처럼 바닥 팔레트, 높이를 판정하는 사례가 없다).
- **카탈로그별 필수 모집단**(`CATALOGUE_POPULATIONS`): `main` = 옛 네 모집단, `bottomless` = 위 셋. `summarize(..., catalogue=)` 는 그 카탈로그의 게이트만 출력하고(빈 게이트는 여전히 불합격), 다른 카탈로그의 장면은 거부한다. `evaluate_recognition --catalogue`. 이것이 없으면 `choose_tau` 가 다른 카탈로그의 n = 0 게이트를 누락으로 세어 τ 를 못 고른다(Codex 2 차 P2).

## 측정 조건

- UBAI `cpu2`, `tools/hpc_sweep.sh`(공유 체크아웃을 건드리지 않게 `HPC_REMOTE` 별도 폴더). UBAI venv 의 numpy 는 2.5.3 으로 ws1(1.26)과 다르다. 이 문서의 수치는 비트 단위 기준선 비교가 아니다.
- 실행과 그때의 코드:
  - 데이터 경로: run-id `20261027T060358Z`(slurm 1050387), `a81c5bf9`(S8 266 장).
  - 옆면: `20261027T055110Z`, `0d073f9c`. 옆면 경로는 그 뒤 바뀌지 않았다.
  - 앞면 2.0–3.5 m: `20261027T061147Z`(slurm 1050394), `63261b79`(선택 패턴 재현).
  - 앞면 1.2–1.8 m: `20261027T072355Z`(slurm 1050580). `63261b79` 에 `--distances` 만 더했다.
  - 실행 `20261027T060358Z`·`20261027T055110Z` 에 함께 들어 있던 앞면 행은 고치기 전 코드(가장 큰 평면의 첫 두 틈 판독)의 결과라 쓰지 않았다.
- **잡음 조건은 섞지 않는다.** 표마다 하나다:
  - 무잡음: 렌더 깊이를 1 mm 로 반올림만 한다.
  - 팀 잡음: `config/depth_noise_d435i_team.yaml` 의 `frame_640` 수준(공간 상관 성분·결측·날림 화소). 옆면·앞면에서 쓴다.
  - 생성기 기본 잡음: `generate_recognition_scenes --noise default` = `NoiseModel()` 기본값이다. 팀 표가 **아니다**. 데이터 경로 표의 "기본 잡음" 열이 이것이다.
  - iid σ = k·d²(k 0.006): 앞면에서 진단으로만 쓴다.

## 결과 1 — 데이터 경로 (dev, 장면 단위)

검출기 셋을 돌렸다.
- `baseline`: 근거리 포켓 검출기(형상 사전마다, lower 관문 켬)를 원거리 장면에 그대로 쓴다. 정체 판정이 없어 valid 면 곧 후보다.
- `geometric-fit`: 기하 제안 + `instance_fit`.
- `oracle-fit`: 정답 인스턴스만 제안하고 `instance_fit` 이 맞춘다.

**CPU 로 잴 수 없는 것**:
- 학습 제안기(YOLO11s)는 RGB 모델이라 이 장치에서 돌리지 않았다. 계획 §3.1 의 "제안이 있었는가, 점수는" 항목은 비어 있다.
- `geometric-fit` 의 후보는 정체가 `unconfirmed` 로 남는다(정체 확정은 학습 점수 τ 가 한다). 그래서 이 경로는 오검출도 재현율도 재지 못한다 — 아래 표의 fp 0 과 놓침 100 % 는 측정이 아니라 구조다. 맞춤 사유만 진단으로 쓴다.
- `oracle-fit` 은 정답 인스턴스만 제안하므로 음성 장면에는 후보가 없다. 오검출 0 은 구성상 0 이다.

### N2 dev 의 오검출 (`baseline`, 장면 수 기준)

| 종류 (dev 장수) | 무잡음 | 기본 잡음 |
|---|---|---|
| `shelf` (7) | 6 | 4 |
| `shelf_closed` (13) | 12 | 4 |
| `shelf_rail` (19) | 15 | 4 |
| `shelf_raised` (12) | 4 | 3 |
| `shelf_legs` (19) | 17 | 6 |
| `shelf_uprights` (6) | 6 | 1 |
| `box_stand` (13) | 4 | 2 |
| `block_pile` (6) · `blocks` (6) · `crates` (6) | 0 · 0 · 0 | 0 · 0 · 0 |
| 어려운 음성 합 (89) | **64** | **24** |

- 같은 실행에서 옛 N dev(무잡음): `grounded` 16/17, `shelf` 14/16 이 `baseline` 의 후보가 됐다. `blocks`·`crates` 0/16. 기본 잡음에서는 `grounded` 7/17, `shelf` 2/16.
- 무잡음에서 근거리 검출기의 정면 관문은 어려운 음성 89 장 중 64 장을 후보로 냈다. 기본 잡음에서는 24 장으로 줄었지만, 같은 잡음에서 S8 의 선택도 EPAL 6 155 → 69, T11 170 → 50 으로 함께 줄었다(아래). 이 숫자로 잡음 아래에서 판별이 나아졌다고 읽을 수 없다.

### S8 dev 재현 (`selected` = 대상 선택, 놓침 = 정답 인스턴스에 맞는 후보 없음)

| 형상 (dev 186) | 경로 | 무잡음 선택 / 놓침 | 기본 잡음 선택 / 놓침 |
|---|---|---|---|
| grounded EPAL 6 | `baseline` | 155 / 31 | 69 / 117 |
| grounded EPAL 6 | `oracle-fit` | 176 / 10 | 177 / 9 |
| grounded T11×0.6 | `baseline` | 170 / 16 | 50 / 136 |
| grounded T11×0.6 | `oracle-fit` | 91 / 26 | 90 / 34 |

- 선택과 놓침의 합이 186 보다 작은 나머지는 대상을 고르지 않은 장면(보류·재관측 요청 등)이다.
- `oracle-fit` T11: 정체·맞춤이 된 후보(`counted`)는 무잡음 160, 기본 잡음 152 인데 대상 선택은 91·90 이다. 남는 후보의 사유는 `competing:t11_06:x`(T11 의 x·y 두 가설이 경쟁, 무잡음 20 건)가 가장 많다. y 면 장면(계획 §2.1 a′ 의 면 점유 틀 차이)이 이 경쟁과 관련이 있는지는 장면 단위로 가르지 않았다. 학습 제안기 경로에서 같은 현상이 나오는지와 함께 Isaac 단계에서 본다.
- 이 수치는 dev CPU 렌더의 진단이다. 게이트 판정이 아니다.

## 결과 2 — 옆면 구조 검사 (`bottom_deck_evidence`, 팀 잡음 frame_640)

옆면 받침대(`tools/bottom_deck_scenes`)에서 면마다 판정했다. 시드 12 × 양면 = 묶음당 24 회. 거리 1.35 m, 비스듬히 보는 각 x 면 35°·T11 y 면 0°.

| 형상 | 판정 (EPAL 6 x · T11 x · T11 y) | `structure` 모드에서 |
|---|---|---|
| 팔레트 | present 23 · 24 · 24 (EPAL 1 회 `board_band_unseen`) | 허가 |
| grounded | absent:`open_gaps` 24 · 24 · 24 | 허가 |
| `shelf`·`shelf_closed`·`shelf_rail`·`shelf_raised` | unobserved:`closed_face` 전부 24 | 거부 |
| `shelf_uprights` | unobserved:`rows_not_located` 전부 24 | 거부 |
| `box_stand` | unobserved:`rows_not_located` 24 · 22 · 24 (T11 x 2 회 `face_ambiguous`) | 거부 |
| **`shelf_legs`** | **absent:`open_gaps` 24 · 24 · 24** | **허가 — 거르지 못한다** |

- 다리형 선반은 옆에서 보면 블록 열 사이가 비어 grounded 와 같은 판정이 나온다. 이 형상은 옆면 검사로 거를 수 없다.
- `shelf_uprights`·`box_stand` 의 거부는 "구조가 막혔다" 가 아니라 "블록 열을 찾지 못했다" 는 이유다. 거부라는 결과는 같지만 판별 근거는 아니다.

## 결과 3 — 앞면 관문 항목 (근거리 검출기, 장착 `carriage_low_measured`)

근거리 재관측 설정(`DetectorParams.derived_for`)의 검출기를 정면에 세운 형상에 돌렸다.
- 격자: 거리 × yaw 0·0.13·0.26 rad. 거리 x 는 카메라에서 형상 **중심**까지다. 진입면은 x − 깊이/2 이다(EPAL 6 깊이 0.60 m, T11×0.6 0.66 m).
- 조건마다 lower 관문 켬(`gate`)·면제(`waive`, `structure` 모드의 근거리 관문)를 따로 돌렸다.
- 관문 항목은 **검출기가 실제로 고른 패턴**의 것이다. 선택을 재현하고 검출기 진단과 대조했다. 이번 실행의 3,024 회(행 1,512 × 두 모드)가 모두 일치했다.
- 판별 통계량은 (upper_left + upper_right) / Σsupports 로, ADR 0003 B-16 비의 개구부별 형태다.
- 표의 "비"는 `waive` 에서 valid 인 칸의 범위다. "칸별 최소 여유"는 같은 칸(거리·yaw·잡음 시드)에서 양성 둘(팔레트·grounded)의 비가 이루는 구간 밖으로 음성이 떨어진 거리의 최소값이다. 음수면 구간 안에 들어갔다.

### 무잡음, 중심 거리 2.0–3.5 m (칸 12)

| 형상 | EPAL 6 valid gate / waive | EPAL 6 비 | 칸별 최소 여유 | T11 valid gate / waive | T11 비 | 칸별 최소 여유 |
|---|---|---|---|---|---|---|
| 팔레트 | 10 / 12 | 0.720–0.859 | | 12 / 12 | 1.065–1.362 | |
| grounded | 10 / 12 | 0.720–0.859 | | 12 / 12 | 1.065–1.362 | |
| `shelf` | 11 / 12 | 0.331–0.454 | +0.311 | 12 / 12 | 0.580–0.886 | +0.348 |
| `shelf_closed` | 11 / 12 | 0.331–0.454 | +0.311 | 12 / 12 | 0.580–0.886 | +0.348 |
| `shelf_legs` | 11 / 12 | 0.331–0.454 | +0.311 | 12 / 12 | 0.571–0.886 | +0.348 |
| `shelf_raised` | 12 / 12 | 0.331–0.444 | +0.311 | 0 / 0 | — | — |
| `shelf_rail` | 10 / 12 | 0.771–0.906 | +0.040 (양성 **위**) | 12 / 12 | 1.159–1.476 | +0.050 (양성 **위**) |
| `shelf_uprights` | 11 / 12 | 0.623–0.755 | +0.081 | 12 / 12 | 0.879–1.099 | +0.161 |
| `box_stand` | 0 / 0 | — | — | 0 / 0 | — | — |

- 무잡음에서 팔레트와 grounded 는 정면 관문 항목이 칸마다 같다. 정면만으로는 둘을 가를 수 없다(가를 필요도 없다 — 둘 다 양성이다).
- 스트링거 띠가 빈 선반(`shelf`·`shelf_closed`·`shelf_legs`, EPAL `shelf_raised`)은 비가 양성보다 낮다. 칸별 여유는 EPAL 0.311·T11 0.348 이고, 칸을 합친 범위도 겹치지 않는다.
- 앞 레일 선반(`shelf_rail`)은 비가 양성보다 **높다**. 칸마다 양성 밖이지만 여유는 0.040·0.050 이다. 칸을 합친 범위는 겹친다(EPAL 0.771–0.906 대 0.720–0.859). "비가 문턱보다 낮으면 거부" 형태의 B-16 판별은 이 형상을 거르지 못한다. 거리·yaw 조건부 양쪽 구간 판정이어야 하고, 그 여유는 무잡음에서도 0.04 다.
- 모서리 기둥 선반(`shelf_uprights`)은 레일이 있는데도 비가 양성보다 낮다(여유 0.081·0.161). 왜 낮은지는 가르지 않았다.
- `box_stand` 와 T11 `shelf_raised` 는 이 격자에서 검출기가 패턴을 찾지 못했다(`no_opening_pattern`·`opening_width_mismatch`).

### 팀 잡음 frame_640, 중심 거리 2.0–3.5 m (칸 36 = 12 × 시드 3)

| 형상 | EPAL 6 valid (waive) | EPAL 6 비 | T11 valid (waive) | T11 비 |
|---|---|---|---|---|
| 팔레트 | 7 / 36 | 0.330–1.047 | 4 / 36 | 0.486–1.182 |
| grounded | 5 / 36 | 0.668–1.126 | 3 / 36 | 0.778–1.236 |
| `shelf` | 6 / 36 | 0.196–0.588 | 2 / 36 | 0.728–1.226 |
| `shelf_closed` | 6 / 36 | 0.282–1.168 | 3 / 36 | 0.244–0.548 |
| `shelf_legs` | 5 / 36 | 0.191–0.412 | 6 / 36 | 0.376–1.321 |
| `shelf_raised` | 4 / 36 | 0.219–0.434 | 0 / 36 | — |
| `shelf_rail` | 6 / 36 | 0.288–1.563 | 3 / 36 | 1.064–1.786 |
| `shelf_uprights` | 7 / 36 | 0.242–0.856 | 4 / 36 | 0.614–1.546 |
| `box_stand` | 0 / 36 | — | 0 / 36 | — |

- 이 거리에서는 팀 잡음 한 프레임으로 **양성도 거의 검출되지 않는다.** 거리별 팔레트 valid 는 EPAL 6 이 2.0 m 5/9, 2.5 m 2/9, 3.0·3.5 m 0/9 이다. T11 은 2.0 m 4/9 이고 그 밖은 0/9 다. 그래서 이 표로는 판별을 판정할 수 없다.
- 판정은 못 하지만 범위는 이렇다. 양성의 비 범위가 넓어서(EPAL 팔레트 0.330–1.047), 칸을 합쳐 양성과 겹치지 않는 선반 형상은 하나도 없다.
- 팀 잡음 실측 문서(`docs/validation/2026-10-17-recognition-measured-noise.md`)도 한 프레임 자세 오차가 1.5 m 안에서만 합격선 안이라고 적었다. 그래서 아래 근거리 격자를 따로 쟀다.
- iid σ = k·d² 진단 결과는 팀 잡음과 다르다(valid 가 더 많다: EPAL 팔레트 23/36). 이 문서의 판단에 쓰지 않는다.

### 근거리 격자 — 중심 거리 1.2·1.4·1.6·1.8 m (진입면 EPAL 6 0.9–1.5 m, T11 0.87–1.47 m)

run-id `20261027T072355Z`(slurm 1050580). 행 1,512, 재현 불일치 0. 거리마다 무잡음 칸 3(yaw 3), 팀 잡음 칸 9(yaw 3 × 시드 3)이다. 칸 값은 `waive` valid 수 / 칸, [비 범위], 칸별 최소 여유다. `gate` 의 valid 수는 모든 칸에서 `waive` 와 같았다 — 이 장치에서는 lower 관문이 아무 형상도 추가로 거르지 않았다.

**무잡음**

| 형상 | EPAL 6 (거리 4 개 합쳐) | T11×0.6 (거리 4 개 합쳐) |
|---|---|---|
| 팔레트·grounded | 12/12, 0.72–0.77 | 12/12, 1.14–1.32 |
| `shelf`·`shelf_closed`·`shelf_legs` | 12/12, 0.39–0.43, 여유 ≥ +0.31 | 12/12, 0.62–0.73, 여유 ≥ +0.44 |
| `shelf_raised` | 12/12, 0.39–0.42, 여유 ≥ +0.31 | 0/12 |
| `shelf_rail` | 12/12, 0.76–0.82(양성 위), 여유 +0.04 | 12/12, 1.23–1.47(양성 위), 여유 ≥ +0.05 |
| `shelf_uprights` | 12/12, 0.62–0.67, 여유 ≥ +0.09 | 12/12, 0.92–1.10, 여유 ≥ +0.18 |
| `box_stand` | 0/12 | 0/12 |

무잡음 근거리는 2.0–3.5 m 와 같은 모양이다.

**팀 잡음 frame_640 — EPAL 6**

| 형상 | 1.2 m | 1.4 m | 1.6 m | 1.8 m |
|---|---|---|---|---|
| 팔레트 | 9/9 [0.69–0.80] | 9/9 [0.64–0.92] | 9/9 [0.52–0.91] | 7/9 [0.52–1.19] |
| grounded | 9/9 [0.71–0.80] | 9/9 [0.66–0.92] | 9/9 [0.58–0.85] | 8/9 [0.44–1.06] |
| `shelf` | 9/9 [0.39–0.45] +0.28 | 9/9 [0.34–0.55] +0.28 | 9/9 [0.30–0.46] +0.21 | 9/9 [0.25–0.77] +0.04 |
| `shelf_closed` | 9/9 [0.38–0.46] +0.31 | 9/9 [0.34–0.53] +0.30 | 9/9 [0.20–0.51] +0.21 | 8/9 [0.29–0.64] +0.09 |
| `shelf_legs` | 9/9 [0.39–0.44] +0.30 | 9/9 [0.33–0.54] +0.26 | 9/9 [0.29–0.45] +0.21 | 9/9 [0.25–0.59] +0.10 |
| `shelf_raised` | 9/9 [0.38–0.44] +0.29 | 3/9 [0.37–0.48] +0.32 | 1/9 [0.33] +0.31 | 3/9 [0.42–0.54] +0.36 |
| `shelf_rail` | 9/9 [0.70–0.84] −0.00 | 9/9 [0.66–0.94] −0.01 | 9/9 [0.56–0.90] −0.05 | 7/9 [0.31–1.21] −0.02 |
| `shelf_uprights` | 9/9 [0.59–0.75] +0.05 | 9/9 [0.55–0.83] +0.05 | 8/9 [0.49–0.84] −0.01 | 8/9 [0.46–0.82] +0.01 |
| `box_stand` | 0/9 | 0/9 | 0/9 | 0/9 |

**팀 잡음 frame_640 — T11×0.6**

| 형상 | 1.2 m | 1.4 m | 1.6 m | 1.8 m |
|---|---|---|---|---|
| 팔레트 | 5/9 [0.93–1.41] | 6/9 [0.89–1.38] | 7/9 [0.42–2.25] | 4/9 [0.20–1.84] |
| grounded | 4/9 [0.93–1.49] | 8/9 [0.97–1.36] | 9/9 [0.40–1.63] | 2/9 [0.79–1.25] |
| `shelf` | 9/9 [0.51–1.10] +0.26 | 8/9 [0.40–0.97] +0.06 | 7/9 [0.36–1.15] −0.09 | 5/9 [0.37–0.87] +0.21 |
| `shelf_closed` | 9/9 [0.55–0.92] +0.07 | 8/9 [0.46–0.83] +0.15 | 7/9 [0.22–1.09] −0.04 | 3/9 [0.30–0.89] +0.30 |
| `shelf_legs` | 8/9 [0.49–0.99] +0.10 | 8/9 [0.37–1.11] −0.23 | 6/9 [0.74–1.12] −0.37 | 2/9 [0.72–1.05] +0.78 |
| `shelf_raised` | 0/9 | 0/9 | 0/9 | 0/9 |
| `shelf_rail` | 5/9 [1.23–1.61] −0.08 | 8/9 [0.95–2.14] −0.04 | 5/9 [0.37–1.47] +0.03 | 4/9 [0.86–2.10] −0.07 |
| `shelf_uprights` | 5/9 [0.83–1.15] −0.16 | 8/9 [0.82–1.49] −0.02 | 7/9 [0.37–1.52] −0.73 | 4/9 [0.59–1.20] +0.09 |
| `box_stand` | 0/9 | 0/9 | 0/9 | 0/9 |

칸별 여유는 같은 칸에 양성과 그 음성이 둘 다 valid 인 칸에서만 셌다. 그래서 칸 수가 적은 칸(T11 1.8 m 등)의 여유는 1–2 칸 값이다.

- EPAL 6, 팀 잡음, 1.2–1.8 m: 스트링거 띠가 빈 선반 넷(`shelf`·`shelf_closed`·`shelf_legs`·`shelf_raised`)은 모든 칸에서 양성 구간 밖이다. 칸별 최소 여유는 1.6 m 까지 +0.21 이상이고, 1.8 m 에서 +0.04(`shelf`)로 좁아진다. 앞 레일(`shelf_rail`)은 네 거리 모두 양성 구간 안에 들어간 칸이 있다(−0.00 ~ −0.05). 모서리 기둥(`shelf_uprights`)은 +0.05 ~ −0.01 이다.
- T11×0.6, 팀 잡음: 양성 자체가 칸의 22–100 % 만 valid 이고 비 범위가 넓다(1.6 m 팔레트 0.42–2.25). 띠가 빈 선반도 1.4·1.6 m 에서 양성 구간 안에 들어간 칸이 있다(`shelf_legs` −0.23·−0.37, `shelf` 1.6 m −0.09).
- 이 장치에서 검출기는 선반 계열을 팔레트만큼 또는 그보다 자주 valid 로 낸다(예: EPAL `shelf` 36/36, 팔레트 34/36). 정면 관문의 valid 자체는 정체 판별이 아니다.

## 재현

저장소 루트에서 `PYTHONPATH=src:.` 로 실행한다. UBAI 에서는 아래 명령을 한 줄씩 작업 목록(`이름<TAB>명령`)에 넣어 `HPC_REMOTE=<전용 폴더> HPC_TIME=00:20:00 tools/hpc_sweep.sh <목록> cpu2 16` 으로 보냈다.

```bash
# 앞면: 형상 2 × 구조 9 (FRONT_STRUCTURES), 거리 격자 둘
python -m tools.shelf_discrimination_study front --shape epal6 --structure shelf_rail                          # 2.0-3.5 m
python -m tools.shelf_discrimination_study front --shape epal6 --structure shelf_rail --distances 1.2 1.4 1.6 1.8
python -m tools.shelf_discrimination_study front-table out/front_*.txt     # 표의 valid 수·비 범위·칸별 최소 여유
# 옆면: 묶음 epal6:x · t11_06:x · t11_06:y × 구조 9
python -m tools.shelf_discrimination_study side --group t11_06:y --structure shelf_legs
# 데이터 경로: 카탈로그마다 무잡음·기본 잡음 데이터셋을 만들고 dev 를 평가한 뒤 종류별로 묶는다
python tools/generate_recognition_scenes.py --catalogue bottomless --cases N2,S8 --noise none --out D/ds_none
python tools/evaluate_recognition.py --dataset D/ds_none --split dev --detector baseline --catalogue bottomless \
    --output D/none_baseline --diagnostics
python -m tools.shelf_discrimination_study summarise --evaluation D/none_baseline --catalogue bottomless
```

`geometric-fit`·`oracle-fit` 은 `--segmentation-truth D/ds_none` 을 더 준다. 데이터 경로 실행(1050387)은 `--catalogue` 인자가 생기기 전 코드였다. 종류별 표는 `verdicts.csv`·`candidates.jsonl` 에서 나오므로 그 인자의 영향을 받지 않는다.

## 한계

- CPU 장치의 상자 렌더다. Isaac 렌더·실센서와 같지 않다. 무잡음 결과는 잡음 결과로 옮겨 읽지 않는다.
- 학습 제안기는 돌리지 않았다. 계획 §4.4 의 재학습 판단(grounded 재현율 5 %p 기준)은 이 문서로 할 수 없다.
- LiDAR 1.05 m 평면 점유 대조(계획 §3.3 보강 2)는 재지 않았다. `shelf_uprights` 가 그 평면을 지나도록 만들어 두었을 뿐이다.
- `shelf` 의 뜬 상판(15 mm)은 상자 모형의 산물이다. 그 형상의 결과를 현실 선반으로 읽지 않는다.

## Isaac 단계에 남은 것

- N2·S8 Isaac 렌더. `sim/isaac/render_recognition_scenes.py --catalogue` 에 bottomless 선택이 아직 없다(렌더 코어 `recipe_boxes` 는 `bottom` 을 이미 따른다).
- 학습 제안기(τ 0.3)로 dev 지표: N2 종류별 오검출, S8 재현율·대상 오류, `competing:t11_06` 경쟁. 그 뒤 설정을 고정해 문서에 적고 eval 을 한 번 읽는다(`evaluate_recognition --catalogue bottomless`).
- LiDAR 평면 대조로 `shelf_uprights` 거부율.
- `shelf_legs` 는 옆면 검사를 통과한다 — 다른 방어(작업 지시 구역, LiDAR 평면은 다리형 받침대가 낮으면 못 거름)에 기대는 범위를 총괄에 올린다.
- slab 은 §2.1 i 수정 뒤.
