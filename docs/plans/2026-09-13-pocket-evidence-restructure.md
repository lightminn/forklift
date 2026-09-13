# 포켓 증거 구조 변경과 M2 재평가 계획 (v6)

> 상태: **v6 — 독립 검토 15회 반영.** v1 `22e967b`, v2 `1644063`, v3 `f633b7a`, v4 `41d4e52`, v5 `3154aea`.
> **이 문서가 정본이다.** `docs/plans/2026-09-13-epal6-scene-set-and-m2-rerun.md` 는 폐기.
> **§9 의 결정을 받기 전에는 Task 4 이후를 착수하지 않는다.**

**목표:** 실물 EPAL 6 형상에서 포켓을 검출하지 못하는 현재 상태를 풀고, EPAL 6 장면 세트로 M2 를 다시 평가한다.

---

## 1. 핵심 변경 — 변이 C

판정 증거를 **각 개구 바로 위**(`u1`·`u2`)로 옮긴다. 아래 덱 증거는 관문에서 뺀다.

**현행 `upper` 는 개구 위를 보지 않는다.** 횡방향 창 `over_openings`(`pocket_detector.py:332`)가 두 개구와 중앙 지지대를 통째로 훑는다. 블록만 아홉 개 있는 구조는 **개구 위가 2.2~3.5 m 전 거리에서 `u1 = u2 = 0`** 인데 현행 `upper` 는 중앙 지지대 위 점으로 채워져 통과 가능해진다.

### 규칙

1. **`supports_ok = min(지지대 3 개) >= min_band_points` 인 패턴만 후보로 만든다.** 아래 덱 증거는 관문에서 뺀다.
2. 후보에 `u1`·`u2` 와 `upper_ok = min(u1, u2) >= min_band_points` 를 기록한다. **`u1`·`u2` 의 횡방향 구간은 `_gap_runs` 가 돌려준 셀 경계 그대로다** — 보고용 반 셀 보정(`params.cell_m` 가산)을 적용하지 않는다. 실측: 같은 장면에서 경계 정의에 따라 117 / 119 / 123 / 137 로 갈리고 문턱 100 이 그 사이에 있다.
3. `upper_ok` 가 거짓인 후보도 **버리지 않는다.** 다만 **절대 `valid` 이 될 수 없다.**
4. 후보 정렬의 **1순위 키는 `upper_ok`**, 2순위가 기존 점수, 3순위가 기존 `-distance`.
5. 선택된 후보가 `upper_ok` 가 아니면:
   - **`max(u1, u2) >= min_band_points`** 이면(한쪽 개구만 증거가 없다 = 팔레트의 한쪽이 가려진 모양) 기존 폭 검사·광선 분류 결과를 **그대로 쓰되, `valid` 만 `no_pallet/no_upper_deck` 로 바꾼다.**
   - **`max(u1, u2) < min_band_points`** 이면(양쪽 다 증거가 없다) 결과와 무관하게 **`no_pallet/no_upper_deck`** 를 낸다.

**5 의 두 갈래가 이 계획에서 가장 중요한 수정이다.** 이것이 없으면 변이 C 는 **팔레트가 없는 구조에 `pocket_occluded` 를 붙인다** — 블록 아홉 개 앞에 장애물 하나를 놓으면 `invalid/pocket_occluded:left` 가 나온다. 현행 HEAD 는 패턴 자체가 후보가 안 되므로 이 라벨을 만들 수 없다. 게다가 음성 범주에서 `invalid` 는 `false_positive_rate` 에 안 잡히고(`evaluation.py:101-103, 170-173`) 예산 없는 `counts.*.invalid` 로 사라진다.

**분리가 성립함을 실측했다.** v1 의 가림 아홉 장면은 **막히지 않은 쪽이 전부 상한 증거를 유지**한다 — `max(u1,u2)` 가 1341·817·761·479·368·405·1105·679·3265 로 최소가 368 이다. 블록만 있는 구조는 2.2·2.5·2.75·3.0 m 에서 **전부 0/0** 이다. 문턱 100 이 그 사이에 넉넉히 들어간다.

**5 에 `front_fraction` 조건을 다시 쓰지 말 것.** `_build_observation` 은 `valid` 를 내기 **전에** 이미 양면의 `front_fraction > occluded_front_frac` 을 검사해 빠져나간다(`:406-411`). "`valid` 인데 `front_fraction` 초과"는 성립하지 않는다. v5 는 그 조건을 규칙에 적었고 그것은 **도달 불가능한 죽은 분기**였다.

### 규칙 4 가 깨뜨리는 진단 하나를 같이 고친다

`detect_pockets:513-515` 는 선택되지 않은 평면을 전부 `lower_pattern_score` 로 표시한다. HEAD 에서는 선택이 곧 점수 argmax 라 참이었다. 규칙 4 는 점수와 무관하게 `upper_ok` 를 먼저 보므로 **더 높은 점수의 평면이 "점수가 낮아서 떨어졌다"고 기록된다** — 독립 검토가 20 조합 중 8 개에서 재현했다(예: 점수 7083 인 평면 0 이 7060 인 평면 1 에게 지고 `lower_pattern_score` 로 기록). **`upper_evidence_absent` 같은 별도 사유를 추가한다.**

### 세 규칙의 값 (독립 재현 3회 일치)

| 규칙 | v1 재현 | EPAL 6 | 블록 아홉 개 | 유령(y 0.80) | 후보 수 |
|---|---|---|---|---|---|
| 현행 HEAD | 기준 100/100 | **전 거리 미검출** | 거부 | **valid, 337~368 mm 오위치** | 82 |
| `min(supports, u1, u2)` | 91/100 | 검출 | 거부 | 거부 | 70 |
| `u1 + u2` | 100/100 | 검출 | 거부 | **valid, 327 mm 오위치** | 79 |
| **채택: 변이 C** | **99/100** | **검출** | **거부** | **거부** | **95** |

**EPAL 6 자세 범위 전수 확인:** 125 자세(x 2.0~4.0 × y ±1.0 × yaw ±0.52)에서 **95 개 `valid`**, 위치 오차 p50 3.6 mm / p95 6.3 mm / 최대 8.5 mm. 실패 30 개는 전부 x 2.0 의 큰 측방 오프셋이나 x 2.5·y ±1.0 에서 **화각 잘림**(`no_opening_pattern`/`opening_width_mismatch`)이고 **`no_upper_deck` 은 하나도 없다.** 같은 125 자세에서 HEAD 는 **0 개** 검출한다.

### 왜 합계가 아니라 최솟값인가

팔레트를 화각 가장자리에 두고 **화각 중심 쪽**에 같은 평면 상자를 놓으면 [이웃 상자 | 공백 | 팔레트 바깥 블록 | 진짜 포켓] 유령 쌍이 성립한다. 유령 개구 위는 비어 있어 `u1 = 0` 인데 진짜 개구가 `u2` 를 대서 합으로 통과한다.

**가설이 아니다.** v1 기하 `y = 0.80` 에서 **현행 HEAD 가 이미 `valid` 로 내고** 337~368 mm 오위치를 보고한다(독립 재현 3회). `y >= 0.75` 에서 36 조합 중 33 개가 유령을 만들고, **`y = 0.70` 에서는 12 조합 전부 정상**이라 시험 자세를 0.70 으로 잡으면 조용히 통과한다.

---

## 2. 합격 조건

### 2a. v1 회귀 가드 (합격 조건이 아니라 비회귀 시험이다)

> v1 카탈로그 100 장면을 동결 파라미터로 재생했을 때 HEAD 와 다른 장면은 **`s056` 하나뿐**이다(`negative_lookalike`, `no_pallet/no_opening_pattern` → `invalid/opening_width_mismatch`). 나머지 99 장면은 `diagnostics` 를 제외한 전 필드가 동일하고 **포켓 좌표·yaw·σ 는 100 장면 전부 불변**이다. `summarize` 에서 움직이는 값은 **`counts.negative_lookalike.true_negative` 10→9, `.invalid` 0→1 둘뿐**이다. `detection_rate`(0.9833 / 0.25), `false_positive_rate`(0.0, 0.0), 전 분위수, `targets.met`(3×True), `scene_count`, `splits` 는 불변이고 **`occluded.invalid` 는 15 로 유지된다.**

독립 재현 **3회 전부 일치**했다. 주의할 점 둘:

- `s056` 은 `true_negative` 에서 **예산 없는 `invalid` 버킷으로** 옮겨간다(§9 ①과 같은 구멍).
- **`no_upper_deck` 은 이 100 장면 중 0 개에서 나온다.** 새 사유의 유일한 커버리지는 Task 2 가 쓰는 새 시험이다.

### 2b. 새 세트의 합격 조건 (캡처 전에 정한다)

**v1 은 이번 변경을 판정할 수 없다.** v1 팔레트는 개구 200 mm·덱 50 mm 라 **고치려는 실패(얇은 덱 증거 고갈)도, 새로 들이는 실패(앞면 개방 구조)도 v1 에서는 발생할 수 없다.** 게다가 재생 입력과 기대값이 둘 다 gitignore 라 새 clone 에서는 **조용히 skip** 된다. 이것을 합격 조건으로 부른 것이 v5 의 잘못이다.

새 세트의 합격 조건은 **§9 의 답을 받은 뒤 여기에 수치로 적고 캡처를 시작한다.** 최소한 다음 네 줄이 수치로 채워져야 한다.

- (a) EPAL 6 양성: 카탈로그 전 거리에서 위치 p95 ≤ 20 mm, yaw p95 ≤ 2°, 검출률 ≥ 95 %(로드맵 M2 목표).
- (b) `negative_block_row`: 위양성률 ≤ ____ (§9 ①에서 정한다).
- (c) `negative_open_bay`: 알려진 한계로 **개수와 함께 기록**하고 (b) 의 예산에서 제외한다 — 또는 §9 ①(b) 를 택하면 이 범주를 만들지 않는다.
- (d) 음성 범주의 `invalid` 를 별도 집계해 함께 보고한다(§1 규칙 5 때문에 여기가 새는 구멍이다).

---

## 3. 이 변경이 새로 들여오는 것

**앞면이 팔레트와 같은 평면에 맞물린(flush) 개방 구조는 위양성이다.** 실질적으로 "바닥판을 뺀 EPAL 6" 이고, 2.5·3.0·3.5 m 에서 `u` 가 1060/1033 · 543/520 · 358/358 로 **문턱의 3.5~10 배**다. 임계 사고가 아니라 구조적 통과다.

**앞면이 물러난 변형은 결과가 리그 구성에 따라 갈린다. 어느 쪽도 주장하지 않는다.** 독립 리그 두 개가 다른 답을 냈다 — 하나는 2.2~2.75 m·후퇴 0.10~0.23 m 에서 위양성을 재현했고, 다른 하나는 같은 후퇴 범위에서 지지대 자체가 문턱을 못 넘어 패턴이 서지 않았다. 상판과 기둥의 깊이·폭을 어떻게 잡느냐로 갈린다. **v5 가 "앞면 정렬이 이 음성의 전부다"라고 단정한 것은 단일 측정의 과잉 일반화였다.** 따라서 음성 세트에 **flush 와 후퇴 두 변형을 모두 넣고** 측정이 답하게 한다.

**기둥 아래 바닥판만 따로 세는 우회는 안 된다.** 바닥판 대역(`|z − 0.022| <= 0.006`)에 **블록 앞면 하단이 같이 들어온다**(블록이 z 0.022 에서 시작). 2 m 에서 EPAL 6 이 기둥당 84 점인데 바닥판을 뺀 구조도 51 점이고, 3.5 m 부터 한 자릿수다.

**정직한 진술:** 이 변경은 판별 근거를 "개구 아래 재료"에서 "개구 위 재료"로 **옮긴다**. 현행은 실물 팔레트를 거부하는 대가로 개방 구조를 거부했고, 변경 후는 실물 팔레트를 검출하는 대가로 앞면 개방 구조를 통과시킨다. 이 프레임 안에 둘 다 잡는 규칙은 없다 — 연결성 검사나 조사(`docs/references/pallet-detection-survey.md` §5.1)의 다중 특징 점수 구조로 가야 풀린다(§9 ⓪).

---

## 4. 문턱 자체가 네 번의 설계 회차에서 공통 원인이다

`min_band_points = 100` 은 **절대 점 개수**다. 얇은 덱 v1→v4, 실기하 재측정(2/3/4 m 에서 286/70/29), floor-through 설계, 그리고 이번 Task 8 의 "4 m 에서 지지대가 문턱의 1.44 배" — 전부 같은 절벽이다.

**직접 재봤다. 거리 정규화는 1/d² 이 아니다.** EPAL 6 지지대 최솟값은 2.0~4.5 m 에서 1484 → 626 → 349 → 220 → 144 → **104** 로, 1/d² 예측치(4.5 m 에서 293)를 한참 밑돈다. 지수는 약 **d^-3.3** 이다 — 원근 단축 때문이다. 예측 점수를 쓰려면 대역 사각형을 실제로 투영해야 하고, 한 줄짜리 닫힌 식이 아니다.

**대신 비는 거의 불변이다.** `u / 지지대` 가 2.0~4.5 m 에서 1.5~1.8 이다. 상대 기준이 더 싼 길이다.

**그리고 선언된 범위가 실제로 도달 불가능하다.** `range_max_m = 5.0` 인데 EPAL 6 지지대는 **4.5 m 에서 이미 104** 로 문턱에 붙는다. 카탈로그가 4.0 m 까지라 평가가 이걸 못 본다. §9 ②와 직결된다.

이 절을 `docs/design/2026-09-13-thin-deck-evidence.md` 에 옮겨 적는다(Task 2). 다음 회차가 같은 절벽을 다시 발견하지 않도록.

---

## 5. 포켓 깊이 관문을 넣지 않는다 — 구현 기각, 요구는 존치

광선은 터널을 관통하지 않고 **내려가다 터널 바닥에 닿는다.**

```
p_max = (개구 대역 상단 z − 터널 바닥 z) × R_perp / (카메라 높이 − 개구 대역 상단 z)
```

EPAL 6 은 터널 바닥이 실제 바닥(z=0), 대역 여유를 뺀 개구 상단이 0.090 m, 카메라가 0.5 m 라 계수가 **0.2195** 다. `range_min_m` 0.8 m 에서 **176 mm** 가 상한이다. 0.36 m 를 요구하면 v1 검출률 0.800 → 0.362, EPAL 6 75 자세 중 9 개만 남는다. 안전한 값 0.09 m 는 실질 이득이 **40 mm** 이고 이름이 360 mm 를 약속한다.

**요구는 죽지 않는다.** 기각된 것은 *이 계측기로 재는 구현*이다. 조사가 기록한 업계 방식(포크 장착 2D LiDAR·레이저)은 **이 프로젝트 하드웨어 계획에 없다** — `docs/hardware.md` 는 D435i·RPLIDAR 만 확정이고 추가 센서는 H1 미결이다. Task 10 에서 `docs/hardware.md` 와 로드맵 위험표에 **"삽입 중 독립 접촉/근접 감지가 필요하며 H0/H1 에서 결정한다"** 를 남긴다.

---

## 6. 전역 제약

- **터널 너머에 의존하는 신호를 만들지 않는다.** 관통 한계는 `arctan(227.5/600) = 20.77°` 다.
- **v1 카탈로그·데이터 세트·prior 는 읽기만 한다.** `COUNTS` 에 범주를 더해 재생성하면 `rng.shuffle`(`generate_scene_catalogue.py:237`)이 밀려 **기존 100 장 중 0 장만 동일**하다(범주 유지 28, 자세 유지 4). 반대로 `sample_catalogue(20260911, 100)` 은 커밋된 YAML 과 같다.
- dev 에서만 튜닝, **eval 은 한 번**.
- `ruff check .` · `ruff format --check .` 통과.
- **기준 회귀:** `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` → **759 passed, 1 deselected, 실패 0**(독립 확인 3회). deselect 되는 1 개는 `test_preview_renders_distinct_views_and_records_evidence`(`rendering` 마크). `test_transport_timeout_kills_orphan_*` 는 부하가 높을 때 **간헐 실패**할 수 있으나 기준선은 초록이다.
- **`artifacts/` 와 `data/` 는 git 에 없다**(`.gitignore:24`). 위임자에게 `data/synthetic_scenes/catalogue_v1` 과 `artifacts/20260912T170442Z_pocket_eval_dev_02`·`..._eval_01` 을 **별도로 전달**한다. 안 하면 `test_detector_v1_replay.py:24` 가 **조용히 skip** 되어 초록으로 보인다.

---

## Task 0: 기준 확인 (Claude)
- [ ] 트리 깨끗, 기준 회귀 초록.
- [ ] 아래 덱 증거의 프로덕션 경로 값 2.0/3.0/4.0 m = **0/12/9**(독립 재현 2회 일치)를 기록한다.

## Task 0.5: §9 결정 수령 (Claude) — **Task 4 착수 조건**
- [ ] §9 의 ⓪①②③④ 를 받아 이 문서에 적는다. §2b 의 (b) 를 수치로 채운다.
- [ ] **받기 전에 Task 4 를 시작하지 않는다.** ②를 택하면 Task 4 의 "v1 자세 100 장 완전 일치" 시험이 무효가 되고, ①(b) 를 택하면 20 장분 카탈로그·월드 배출 경로가 폐기된다. Task 6 이후에 결정이 오면 140 장 재캡처다.

## Task 1: 원격 경로 수리 (위임)

**Files:** `tools/submit_model_check.py`, `tools/remote_model_job.py`, `tests/integration/test_evaluate_cli.py`, `tools/evaluate_pocket_detector.py`, `tests/integration/test_remote_model_jobs.py`

- [ ] snapshot 허용 목록에 `config` 와 `tests` 의 `.yaml` 을 추가한다. 실측: 정확히 **6 개**(91 → 97), 비밀 파일 없음. 현재는 `tests/fixtures/thin_deck_legacy_*.yaml` 이 **모듈 import 시점**에 로드되므로 원격 pytest 가 수집 단계에서 죽는다.
- [ ] **`git_revision` 을 고친다.** `test_evaluate_cli.py:313` 이 길이 40 을 요구하는데 스냅샷에 `.git` 이 없어 `_git_state` 가 `None` 을 준다. **assert 를 완화하지 말고 스냅샷 생성 시 `git_revision` 을 메타데이터로 주입한다.** 완화하면 `run.json` 에 `(None, None)` 이 실려 Task 10 의 "고정 revision" 을 산출물로 증명할 수 없다.
- [ ] **`scenes` 모드에 코어 패키지를 설치하지 않는다.** `core_environment_builder` 는 `model-cpu`/`model-render` 에서만 돈다(`remote_model_job.py:494-506`). Task 3 이 `forklift_core` 를 import 하지 않으므로 이대로 둔다. **결정 완료 항목이다.**
- [ ] **완료 조건:** ① 원격 `model-cpu` job 초록 ② `scenes` 모드 v1 1~2 장면 스모크 성공. **②는 v1 5 상자 경로만 검증한다** — epal6 22 상자 경로의 게이트는 Task 4.5 다.

## Task 2: 변이 C 구현과 명시 진단 (위임)

**Files:** `src/forklift_core/perception/pocket_detector.py`, `tools/evaluate_pocket_detector.py`, `tests/integration/test_evaluate_cli.py`, `tests/unit/perception/test_thin_deck_evidence.py`, `tests/unit/perception/test_pocket_detector.py`, Create `tests/unit/perception/test_opening_evidence_cases.py`, `tests/integration/test_detector_v1_replay.py`, `docs/design/2026-09-13-pocket-detector-baseline.md`, `docs/design/2026-09-13-thin-deck-evidence.md`, `docs/validation/2026-09-13-thin-deck-evidence.md`, `docs/interfaces/pocket-observation.md`

- [ ] §1 의 규칙 1–5 와 `upper_evidence_absent` 진단을 구현한다.
- [ ] **`no_upper_deck` 의 status 를 `no_pallet` 으로 하는 이유를 주석과 설계 문서에 적는다.** 음성 범주에서 `no_pallet` 은 `true_negative`, `invalid` 는 예산 없는 `invalid` 로 집계된다(`evaluation.py:101-108`). 이웃한 `opening_width_mismatch` 가 `invalid` 인 것과 갈리므로 의도임을 명시한다.
- [ ] `_Pattern` 에 `upper_ok: bool` 과 `lower`·`upper_left`·`upper_right`·`support_min` 을 **스칼라**로 추가한다. `support_count` 는 합계라 최솟값 지표로 쓸 수 없다.
- [ ] **`_Pattern.deck_count = lower + upper_left + upper_right` 로 재정의하고 점수식 `score = support_count + deck_count` 는 그대로 둔다.** 실측: 점수는 바뀌지만(2.0 m 에서 16740 → 15726) **v1 100 장면에서 선택 결과는 한 장면도 안 바뀐다.** 이 재정의를 하면 `test_thin_deck_evidence.py` 의 시험 **3 개**가 실패한다(`-231 == 0` 형태) — 하니스가 `deck_count − upper` 로 `lower` 를 역산하기 때문이다. **현행에서 그 역산은 정확하다**(2.0 m 에서 16740 − 7866 = 8874 = 실제 `lower`). 즉 이 결함은 **재정의가 만드는 것**이고, 하니스를 명시 필드 직독으로 바꾸는 것이 그 수선이다. *(v5 가 "재정의를 안 하면 틀린 값을 잰다"고 적은 것은 인과가 거꾸로였다.)*
- [ ] `DetectionDiagnostics` 의 새 필드는 **`exception_traceback` 앞**에 넣는다. 뒤에 붙이면 클래스 생성 시점에 `TypeError` 로 import 가 깨진다. 두 dataclass 모두 위치 인자 생성이다(`:347`, `:537-551`).
- [ ] `SCENE_COLUMNS` 에 새 진단 이름을 올린다. 안 올리면 조용히 버려진다. **`tests/integration/test_evaluate_cli.py:110-134` 가 `scenes.csv` 헤더를 리터럴 완전일치로 검사하므로 같은 커밋에서 깨진다.**
- [ ] **bool `DetectorParams` 항목을 만들지 않는다.** `_finite_scalar` 가 bool 을 거부해 import 가 깨진다.
- [ ] **`test_detector_v1_replay.py`:** 기대 델타는 `s056` 한 장면. **보관된 `run.json`·관측 파일을 고치지 말 것**(과거 실행 기록이다). 예상 델타를 **시험 파일 안의 추적되는 상수**로 넣는다.
- [ ] **새 시험 `test_opening_evidence_cases.py`** — §4 의 리그를 커밋한다. `tests/fixtures/synthetic_scene.py` 와 `pallet_boxes(load_pallet_geometry(...))` 로 만든다. 최소 다섯 가지:
  - **EPAL 6** 2.0~4.0 m: `valid`, 포켓 y = ±0.186 ± 0.01.
  - **블록 아홉 개**: `no_pallet/no_upper_deck`, `u1 = u2 = 0`.
  - **블록 아홉 개 + 앞쪽 장애물**: `no_pallet/no_upper_deck` — **규칙 5 의 `max(u)` 갈래가 없으면 여기서 `pocket_occluded` 가 나온다.** 이 시험이 그 회귀를 막는다.
  - **flush 앞면 개방 구조**(바닥판만 뺀 EPAL 6): `valid`. **이름을 `test_known_limitation_flush_front_open_bay_is_accepted` 로 하고** 검증 기록의 한계 절에 올린다. 의도된 동작으로 오독되면 안 된다.
  - **유령 패턴**: 팔레트 x 2.5 / **y = 0.80**(0.70 은 두 규칙 모두 정상 통과라 조용히 통과한다), 같은 평면 상자를 **화각 중심 쪽(−y)** 바깥 블록에서 0.22 m. **사용 기하를 시험 안에 명시한다.** 합계 규칙에서 `valid`(320~370 mm), 변이 C 에서 `no_pallet`.
- [ ] 무효가 되는 계약을 같은 커밋에서 갱신한다: `baseline.md:66`("위·아래 덱 대역에 점유"), **`:68`("후보가 여럿이면 최고 점수" — 규칙 4 가 깨뜨린다)**, **`:70-76` 의 순서화된 판정 목록, 특히 `:74`("두 개구부 모두 열림 → `valid`" — 이제 `upper_ok` 조건부다)**, `thin-deck-evidence.md` 의 `min(지지대 3개, lower, upper)`, `validation/thin-deck-evidence.md` 의 "deck_count 에서 upper 를 뺀다".
- [ ] §4(문턱)와 §5(깊이 닫힌 식)를 `thin-deck-evidence.md` 에 남긴다.
- [ ] **인터페이스 문서:** `pocket-observation.md:39` 에는 사유 어휘가 **없고** 기존 여섯 개도 안 적혀 있다. 검증하는 코드도 없다(`pocket_observation.py:93-94` 가 문자열·공백만 본다). **일곱 개를 전부 적거나, 이 항목을 빼고 설계 문서에만 적는다.** 하나만 적힌 절을 만들지 않는다.
- [ ] **인터페이스 문서에 한 줄:** `center_m` 의 **z 는 prior 유래 상수**(`:421`)이고 정답도 같은 값이라 **`position_error_m` 의 z 성분은 구조적으로 항상 0** 이다. 보고 오차는 사실상 평면 2 차원이다.

## Task 3: 월드 생성기가 두 기하를 받게 한다 (위임)

**Files:** `sim/gazebo/build_scene_world.py`, `tests/simulation/test_build_scene_world.py`

- [ ] **새 장면 키를 만들지 않는다.** `_require_keys`(`:24-26`, `:59-71`)는 카탈로그 공용이고 집합 동일성이라, 키 하나를 더하면 v1 100 장면이 전부 거부된다. 새 음성은 **기존 `lookalike` 블록의 `{x_m, y_m, yaw_rad}`** 를 그대로 쓰고 구조 종류는 **`category`** 로 구분한다. 고칠 곳은 범주 화이트리스트(`:78-83`)와 `("lookalike", category == "negative_lookalike")` 존재 규칙(`:88`) 두 줄뿐이다.
- [ ] **치수 출처는 `sim/models/epal6_pallet/pallet.urdf` 직독이다**(22 visual, 검증됨). 이 경로는 Task 1 없이도 이미 스냅샷 허용 목록에 있다. `config/pallet_geometry_epal6.yaml` 직독은 `block_centres_x_m/y_m`·`top_board_centres_y_m`·`top_board_pitch_m`·`opening_centre_height_m`·`deck_top_m` 다섯 개를 재구현해야 해서 `pallet_boxes()` 와 중복 구현이 된다. `forklift_core` 는 import 하지 않는다(`scenes` 모드에 코어 설치가 없다).
- [ ] **경로는 `Path(__file__).resolve().parents[N]` 기준으로 한다.** `capture_scenes.py:226-236` 이 `build_scene_world.py` 를 **`cwd=runtime`**(스냅샷 밖)에서 띄우므로 cwd 상대경로는 로컬만 통과하고 원격에서만 죽는다.
- [ ] **v1 경로는 그대로 둔다.** `catalogue_version == "v1"` 이면 현행 5 상자 배출과 `pallet` 완전일치 검사를 유지한다. `"epal6"` 일 때만 22 상자 경로를 탄다. `tests/simulation/test_build_scene_world.py:37`(collision 5), `:40`(outer 2), `:144`(스페이서 이름 집합)이 **계속 통과해야 한다.**
- [ ] **계속 통과해야 하는 3 파일:** `tests/simulation/test_build_scene_world.py`, `test_capture_scenes.py`, `test_scene_catalogue.py`.
- [ ] **`negative_lookalike` 의 솔리드 상자가 `[0.6, 0.8, 0.30]`·z 0.15 로 하드코딩되어 있다**(`:260`). EPAL 6 세트에서 그대로 두면 144 mm 팔레트 옆에 300 mm 덩어리가 서서 "팔레트를 닮은 음성"이 아니게 된다. §9 ④의 답대로 처리한다.
- [ ] `sdf_parts.add_urdf_visuals` 는 **재사용 불가**다: 시그니처 `(world, path)` 에 pose 인자가 없고, 모델명이 하드코딩(`:69`)이며, visual 만 내고 collision 이 없다.
- [ ] **시험: 정답 포켓 중심을 지나는 광선이 팔레트 상자 어느 것과도 교차하지 않는다.** 상자 개수·크기만 보는 시험은 블록 아홉 개를 같은 x 에 놓아도 통과한다.

## Task 4: 카탈로그 변환과 새 음성 (위임) — **Task 0.5 이후**

**Files:** Create `tools/retarget_scene_catalogue.py`, `tests/unit/test_retarget_scene_catalogue.py`, `sim/gazebo/scenes/catalogue_epal6.yaml`; Modify `src/forklift_core/perception/evaluation.py`, `tools/merge_scene_batches.py`, `tools/evaluate_pocket_detector.py`, `sim/gazebo/build_scene_world.py`, `tests/simulation/test_scene_catalogue.py`, `docs/interfaces/scene-dataset.md`, `docs/design/2026-09-11-pocket-observation-and-scene-set.md`, `docs/validation/2026-09-11-scene-catalogue-and-world.md`

- [ ] v1 의 `x_m`·`y_m`·`yaw_rad`·범주·분할·조명·표면·distractor 를 그대로 옮긴다. **자세 100 장 완전 일치를 시험으로 고정한다**(approx 아님).
- [ ] **재계산·재설정 대상 전수:**
  - 헤더: `catalogue_version` → `epal6`(Task 3 이 이것으로 분기한다), `generator`, `seed`, `source_provenance`, `ranges.opening_width_m`
  - 헤더 `pallet` 6 필드 — `load_catalogue:174-181` 이 v1 값과 **완전일치**를 요구한다. epal6 분기에서 **무엇을 넣을지 정한다**(v1 값 복사는 문서가 기하와 모순된다)
  - `pallet.opening_width_m` → **0.2275 고정**(v1 은 80 장면에 78 가지 값 0.2007~0.2794). 검증기 허용 범위 `0.20 <= w <= 0.28`(`:108`) 안이고 epal6 prior 범위 `[0.2075, 0.2475]` 안이다
  - `ground_truth.{left,right}.center_m` — **x 성분도 바뀐다**(정답은 `pallet + R(yaw)·(−0.3, ±offset)` 이고 offset 이 장면마다 달랐다). y 오프셋 ±0.18625, **z 0.15 → 0.061**
  - `ground_truth.{left,right}.width_m` → 0.2275, `.height_m` **0.20 → 0.078**
  - `occluder.center_m`·`size_m` — `size[1] = fraction × opening_width_m`(`generate_scene_catalogue.py:200`)
  - `visibility.corners_px`, `occluded_fraction_image`(**이미 있는 필드다** — s001 = 0.431511), **`left_in_view`·`right_in_view`·`occluded_side`**(포켓 중심이 움직이므로 판정이 바뀔 수 있다)
- [ ] **`fraction` 은 물리 폭 비율이지 영상 가림률이 아니다**(s001: 0.3553 × 0.2474 = 0.087901). 같은 fraction 이 같은 난이도가 아님을 인터페이스 문서에 적는다.
- [ ] 새 음성 두 종을 **종별 20 장**(`s101`–`s140`) 덧붙인다. 분할은 `len(indices) * 7 // 10` 으로 dev 14 / eval 6. `_scene_ids` 정규식 `s[0-9]{3}` 이 받는다.
  - `negative_block_row`: 블록 아홉 개만.
  - `negative_open_bay`: **flush 앞면과 후퇴 앞면 두 변형을 각각 10 장씩**(§3 — 후퇴 쪽은 결과가 갈리므로 측정이 답하게 한다).
- [ ] 데이터 세트는 `data/synthetic_scenes/catalogue_epal6/`. **`catalogue_v1` 을 덮지 않는다** — 두 `run.json` 의 `dataset_dir` 가 그 경로라 재현 시험의 **주 경로**가 깨진다.
- [ ] 범주 등록 **네 곳 전부**: `evaluation.py:18`, `merge_scene_batches.py:41-46`(새 두 종의 `ground_truth.status` 는 **`no_pallet`**), `build_scene_world.py:78-83`, `evaluate_pocket_detector.py:292-296`.

## Task 4.5: epal6 스모크 게이트 (Claude + 원격)
- [ ] **epal6 카탈로그로 1~2 장면을 원격 캡처한다.** Task 1 의 스모크는 v1 경로만 검증했다. 이 게이트가 없으면 22 상자 경로·URDF 직독·`cwd=runtime` 경로 문제가 **되돌릴 수 없는 140 장면 캡처에서 처음** 드러난다.

## Task 5: 동결 게이트 (Claude)
- [ ] **캡처는 되돌릴 수 없다.** `merge_batches` 는 배치 manifest 를 `expected_metadata`(`:174-187`)와 완전일치시키는데 거기에 `catalogue_sha256` 뿐 아니라 **`source_snapshot_sha256`·`image_id`** 가 들어 있다. 배치 1 부터 병합까지 **스냅샷 허용 목록의 어떤 파일도** 건드리면 전 배치가 폐기된다. 전수 커버리지도 요구한다(`:220-222`).
- [ ] **확인 명령이 있다:** `python tools/submit_model_check.py submit --host <h> --remote-root <r> --source . --mode scenes --image <id> --catalogue <c> --scene-range s001-s002 --run-id probe --dry-run` 이 원격 접속 없이 `source.snapshot_sha256` 을 출력한다. 캡처 시작 전과 각 배치 전에 같은 값인지 본다.
- [ ] 배치 수는 코드 요구가 아니라 우리 선택이다(`--batches` 는 `nargs="+"`). **중간 발견이 한 배치만 버리도록 작게 나눈다.**
- [ ] **캡처 중 Task 7 의 파라미터 파일 작업을 시작하지 않는다.**

## Task 6: 캡처·병합 (Claude + 원격)
- [ ] 140 장면 캡처, 병합, set manifest 기록, 그룹·권한 확인. 재캡처 예산을 미리 잡는다(공유 `kang` 계정·GPU 대기).

## Task 7: 동결 파라미터 파일 정리 (Claude)
- [ ] **Task 2 뒤, Task 8 앞.**
- [ ] `config/detector_params_v1.yaml` 은 **16 키**, `DetectorParams` 는 **17 필드**(`deck_evidence_tol_m` 누락, 기본 0.006). `_load_params` 는 모르는 키는 거부하고 **없는 키는 조용히 기본값으로 채운다.**
- [ ] **동결 파일을 고치지 않는다.** 두 과거 `run.json` 도 16 키라 `test_detector_v1_replay.py:31` 의 `params_data == run["params"]` 가 지금 True 이고 키를 더하는 순간 False 가 된다. 시험을 **"파일 ⊆ run.params, 나머지는 코드 기본값"** 으로 바꾼다.

## Task 8: dev 튜닝 (Claude)
- [ ] 기준선 1 회 후 한 번에 하나씩.
- [ ] 지표에 **지지대 최솟값**을 넣는다. 4 m 에서 `min()` 을 결정하는 것은 `upper`(692)도 `u1/u2`(256/268)도 아닌 **지지대**(144/207/144, 최소 1.44 배)다.
- [ ] **위양성 지표에 `invalid` 를 병기한다.** `false_positive_rate` 는 `valid` 만 세고 `evaluate_scene` 은 `invalid` 를 음성 분기보다 먼저 처리한다.
- [ ] **`floor_z_m >= deck_bottom_m` 을 설정 오류로 거부**한다. 현행 `floor_z_m` 은 0.020, EPAL 6 `deck_bottom_m` 은 0.022 — 2 mm 차다. `DetectorParams.__post_init__` 은 prior 를 못 보므로 **`detect_pockets` 진입부와 `evaluate_pocket_detector._run` 두 곳**에 넣는다.
- [ ] **처리 시간은 합격 조건이 아니다.** 변이 C 는 후보가 82 → 95 로 는다. 같은 기계에서 순차 best-of-3 는 +56 %, 장면별 교차 best-of-3 는 −0.3 % 를 준다 — 부하 드리프트가 신호를 압도한다. 재려면 교차 best-of-3 로만 재고 목표로 쓰지 않는다.

## Task 9: 동결 게이트 · Task 10: eval 1 회와 기록 (Claude)
- [ ] 고정 revision·clean 트리에서 eval 1 회. **§2b 의 (a)~(d) 로 합불을 적는다.**
- [ ] 검증 기록: 위양성률과 `invalid` 병기 / **앞면 개방 구조 위양성이 이 변경으로 새로 생긴다는 사실과 §3 의 "못 없앤다" 측정** / `position_error_m` 의 **z 성분 구조적 0** / `POSITION_TOLERANCE_M` 0.20 m 는 대응 문턱이지 도킹 허용치가 아니며 포크 좌우 45 mm·수직 6 mm 와 무관하다는 점 / §4 의 문턱 절벽과 `range_max_m` 5.0 의 도달 불가 / 해결하지 않은 것들.
- [ ] `docs/hardware.md` 와 로드맵 위험표에 §5 의 접촉/근접 감지 요구를 남긴다.

---

## 9. 사용자 결정 (Task 0.5 에서 받는다)

**⓪ 이 재평가를 지금 하는가.** 로드맵은 **M2 를 이미 완료로 표시**했다(`2026-09-11-development-roadmap.md:221`, 합성 한정 명시). 개발 2주차(09/18–09/24)는 "M2 기준선 + **H0/H1 조사**", 3주차가 "M2 오차 평가 + **M3 추적·소실 회귀**" 다. **M3 는 "실물 없이 먼저 검증"** 이라고 로드맵에 적혀 있고(`:67`) 코드가 0 줄이며, 브리프가 명시한 "삽입 중 포켓 추적"이 거기 있다. 로드맵 자신이 "인식 개발이 빨라도 실물 선행 경로를 건너뛸 수 없다"(`:171`)고 적었다. 선택: (a) 이 계획을 지금 실행 (b) M2 를 합성 한정으로 봉인하고 M3·H0/H1 을 먼저 (c) 축소판(§9 ②)만 실행.

**⓪′ 게이트 아키텍처를 그대로 두는가.** v5 는 조사 §5.1 의 다중 특징 점수 구조를 "M2 가 포켓 관측을 산출물로 정의하므로" 물리쳤는데 **그것은 로드맵 오독이었다.** 로드맵 M2 는 "팔레트 전면 후보와 두 포켓의 경계·배치를 **함께 사용해**"(`:61`) 추정하라고 하고, 무효 반환도 "부족하면"이라고만 한다 — 점수 문턱으로도 충족된다. 비용 차이는 Task 2·8 에 갇히고(1·3·4·5·6·7·9·10 은 그대로), `pocket_detector.py:501` 에 이미 점수가 있고 포켓 치수는 이미 prior 에서 온다. §3 의 위양성이 사라지는 길이기도 하다.

**① `negative_open_bay` 를 찍는가.** `targets.met` 에는 **위양성 목표가 없다**(`evaluation.py:210-225`). 선택: (a) `negative_block_row` 위양성률 **≤ 5 %** 를 `targets` 에 추가하고 `negative_open_bay` 는 예산 밖에서 개수만 보고 (b) 앞면 개방 구조를 운용 범위 밖으로 선언하고 **20 장을 안 찍는다** (c) 지금처럼 별도 보고만. **두 음성은 분리해서 각각 정할 수 있다.**

**② 자세 범위를 좁히는가.** 범위 `x 2–4 m, y ±1.0, yaw ±0.52` 는 브리프 요구가 아니라 2026-09-11 에 정한 값이고 `build_scene_world.py:102-106` 에 하드코딩돼 있다. **커밋된 v1 카탈로그 80 장면의 실제 입사각은 중앙값 16.3°, 최대 51.5°, 터널 한계 20.77° 초과가 30/80(38 %)**(양성만 23/60)이다 — 카메라 (0.75,0,0.5)→팔레트 앞면 기준이며, v5 가 적은 26.6° 는 base 원점→중심으로 잰 틀린 값이었다. 조사의 상용 제품은 1–3 m 에 대략 위치까지 준다. **§4 의 `range_max_m` 5.0 도달 불가와 같이 본다.**

**③ MR6D 를 어떻게 하는가.** CC-BY-4.0, **D435i(우리 카메라)**, 유로 팔레트 6D 정답. 선택: (a) 140 장 캡처에 **추가** (b) 140 장 캡처를 **대체** (c) 미룬다. 비용을 정직하게: MR6D 는 **EPAL 6 half 가 아니어서** 별도 prior 가 필요하고, 공개 RGB-D 세트에 **3D 포켓 라벨이 없으므로** 포켓 정답은 6D 자세 + 규격에서 유도해야 한다.

**④ `negative_lookalike` 상자를 EPAL 6 규격으로 낮추는가.** 지금은 `[0.6, 0.8, 0.30]`·z 0.15 하드코딩이다. 낮추지 않으면 그 10 장면은 "닮은 음성"이 아니게 된다.

## 10. 범위 밖 (의도)

터널 너머 신호(금지), 포켓 깊이 관문(구현 기각 — 요구는 §5 존치), 형판 정합·PnP, 딥러닝, 마커, 다중 시점 융합(M3), `s009` 진단 분리(별건 — 다만 이것은 **양성**을 해치는 구조적 결함이고 두 번째로 미뤄지는 중이다). 게이트 아키텍처 재설계는 **§9 ⓪′ 의 결정 사항이지 범위 밖이 아니다.**

---

## 11. 자체 검토 기록

독립 검토 **15회**(v1~v6). 각 회차는 서로의 결론을 보지 않았다.

**5회차(v5 대상, 3건)가 잡은 것과 처리:**

| 지적 | 처리 |
|---|---|
| 변이 C 가 **팔레트 없는 구조에 `pocket_occluded`** 를 붙인다(블록 아홉 개 + 장애물) | **규칙 5 에 `max(u)` 갈래 추가.** 분리 성립을 실측(가림 9 장면 최소 368 대 블록 0/0). 회귀 시험 추가 |
| 규칙 5 의 `pocket_occluded` 분기가 **도달 불가능**하고 100 장면에서 규칙 5 자체가 0 회 발화 | 규칙 재작성. 가림 라벨은 규칙 1–3 이 보존한다 |
| 규칙 4 가 `lower_pattern_score` 진단을 **거짓으로 만든다**(20 조합 중 8) | `upper_evidence_absent` 사유 추가 |
| §3 "후퇴 앞면은 위양성이 아니다" 가 **단일 측정의 과잉 일반화**, 리그 둘이 다른 답 | 어느 쪽도 주장하지 않고 **두 변형을 모두 음성 세트에** |
| 합격 조건이 비회귀 시험이지 합격 기준이 아님 — v1 은 이 변경을 판정할 수 없다 | §2 를 2a(회귀 가드)·2b(새 세트 합격 조건)로 분리 |
| §10 의 아키텍처 기각 사유가 **로드맵 오독** | §9 ⓪′ 로 승격, 비용이 Task 2·8 에 갇힘을 명시 |
| **M2 재평가를 하는지 자체**가 조용히 결정됨 | §9 ⓪ 로 승격 |
| §9 결정에 **게이트가 없어** 위임 Task 4 가 버려질 수 있음 | Task 0.5 신설, Task 4 착수 조건 |
| 장면 키 추가가 v1 100 장면을 전부 거부시킴 | **키를 안 늘린다** — `lookalike` + `category` 재사용 |
| Task 1 스모크가 **v1 경로만** 검증 | Task 4.5 epal6 스모크 게이트 신설 |
| `deck_count` 재정의 근거가 **거꾸로** | 인과 수정(7860 은 재정의 *이후* 값) |
| 내 입사각 26.6° 가 틀림 — 실제 중앙값 16.3°·최대 51.5°·38 % 초과 | §9 ② 수정 |
| `test_evaluate_cli.py` 가 Task 2 Files 에 없음(CSV 헤더 완전일치) | 추가 |
| `baseline.md:68`·`:70-76` 이 계약 갱신 목록에 없음 | 추가 |
| 인터페이스 문서에 사유 어휘가 **없다** | "일곱 개 전부 또는 빼기" |
| `no_upper_deck` 의 `no_pallet` 선택이 집계에 영향(true_negative vs invalid) | 이유를 적도록 지시 |
| `negative_lookalike` 상자 하드코딩 | §9 ④ |
| Task 번호 오기(캡처 Task 6, 파라미터 Task 7) | 수정 |
| Task 4 Files 에 시험 파일 없음 | 추가 |
| 동결 게이트 확인 명령 부재 | `--dry-run` 명령 추가 |
| `u1`/`u2` 구간 정의 미지정(117/119/123/137) | 규칙 2 에 못 박음 |
| "3 m 중앙 위 1056" 재현 안 됨 | 재현되는 진술(블록 아홉 개 `u1=u2=0`)로 교체 |
| 문턱 100 이 네 회차의 공통 원인 | §4 신설. **거리 정규화가 1/d² 이 아님(d^-3.3)** 과 `range_max_m` 5.0 도달 불가를 직접 측정 |

**내가 직접 재계산한 것:** `max(u)` 분리(가림 9 장면 × 블록 구조 4 거리), 변이 C 의 v1 100 장면·집계 델타, EPAL 6/블록/개방 구조/유령 케이스 행렬, 거리별 지지대 감쇠(d^-3.3)와 `range_max_m` 도달 불가, 입사각 통계(30/80), 카탈로그 키 집합·`lookalike` 검증·하드코딩 상자, `_build_observation` 의 검사 순서, `centre[2]` prior 유래, 로드맵 M2·일정 원문.

**반박되지 않은 것:** §2a 의 합격 조건은 세 검토가 독립적으로 재계산해 **전부 일치**했고(후보 수 82/70/95 포함), §3 의 flush 앞면 위양성과 §1 의 유령 위양성도 각각 독립 재현되었다.
