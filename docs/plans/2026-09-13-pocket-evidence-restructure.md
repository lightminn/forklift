# 포켓 증거 구조 변경과 M2 재평가 계획 (v5)

> 상태: **v5 — 독립 검토 12회 반영.** v1 `22e967b`, v2 `1644063`, v3 `f633b7a`, v4 `41d4e52`.
> **이 문서가 정본이다.** `docs/plans/2026-09-13-epal6-scene-set-and-m2-rerun.md` 는 폐기.
> **실행자:** Task 1–4 위임, 나머지 Claude.

**목표:** 실물 EPAL 6 형상에서 포켓을 검출하지 못하는 현재 상태를 풀고, EPAL 6 장면 세트로 M2 를 다시 평가한다.

---

## 1. 핵심 변경

판정 증거를 **각 개구 바로 위**(`u1`·`u2`)로 옮긴다. 아래 덱 증거는 관문에서 뺀다.

**현행 `upper` 는 개구 위를 보지 않는다.** 횡방향 창 `over_openings` 가 두 개구와 중앙 지지대를 통째로 훑어서(`pocket_detector.py:332`), 바닥에 놓인 상자 세 개가 **가운데 상자 자신으로** 상한 증거를 다 채운다. 3 m 에서 개구 위 0+0, 중앙 위 1056.

### 세 가지 규칙과 그 값 (전부 재현함)

| 규칙 | v1 재현 | EPAL 6 | 상자 세 개 | 유령(y 0.80) | 후보 수 |
|---|---|---|---|---|---|
| 현행 HEAD | 기준 100/100 | **전 거리 미검출** | 거부 | **valid, 368 mm 오위치** | 82 |
| `min(supports, u1, u2)` | 91/100 | 검출 | 거부 | 거부 | 70 |
| `u1 + u2` | 100/100 | 검출 | 거부 | **valid, 327 mm 오위치** | 79 |
| **채택: 변이 C** | **99/100** | **검출** | **거부** | **거부** | **95** |

### 변이 C — 게이트는 유지하고 라벨만 되살린다

`min(u1, u2)` 만 쓰면 게이트 안에서 떨어지는데, **광선 분류는 게이트를 통과한 패턴에만 돈다**(`pocket_detector.py:486-491`). 그래서 가림 장면의 `invalid/pocket_occluded:*` 가 `no_pallet/no_opening_pattern` 으로 뭉개진다. 설계 문서가 v3 에서 같은 모양을 보고 **"명백한 퇴행이다"** 라고 적었던 것이고(`thin-deck-evidence.md:63-66`), 로드맵 M3 의 `lost` 상태와 브리프 Case C/D(막히면 재배치)가 쓰는 구분이다.

**규칙:**

1. `supports_ok = min(지지대 3 개) >= min_band_points` 를 만족하는 패턴만 후보로 만든다.
2. 후보에 `upper_ok = min(u1, u2) >= min_band_points` 를 **불리언으로 기록**한다.
3. `upper_ok` 가 거짓인 후보도 **버리지 않는다.** 다만 **절대 `valid` 이 될 수 없다.**
4. 후보 정렬의 **1순위 키는 `upper_ok`**, 2순위가 기존 점수, 3순위가 기존 `-distance` 다. `upper_ok` 를 점수에 녹이지 말고 별도 키로 둔다.
5. 선택된 후보가 `upper_ok` 가 아니면 기존 폭 검사와 광선 분류를 그대로 돌리고, 그 결과가 `valid` 이 될 경우에만 가로채서 — 어느 쪽이든 `front_fraction > occluded_front_frac` 이면 `invalid/pocket_occluded:<side>`, 아니면 `no_pallet/no_upper_deck` 를 낸다. 기존 검사가 이미 비-`valid` 를 냈으면 그대로 둔다.

4번이 없으면 상한 실패 후보가 진짜 후보를 argmax 에서 밀어낼 수 있다. v1 100 장면에서는 실제로 물지 않았지만(정렬 키를 넣은 판과 안 넣은 판이 100/100 동일) 규칙으로 박는다.

### 왜 합계가 아니라 최솟값인가

**합계는 실재하는 버그를 남긴다.** 팔레트를 화각 가장자리에 두고 **화각 중심 쪽**에 같은 평면 상자를 놓으면, 먼 쪽 바깥 블록이 잘려 진짜 패턴이 탈락하고 [이웃 상자 | 공백 | 팔레트 바깥 블록 | 진짜 포켓] 유령 쌍이 성립한다. 유령 개구 위는 비어 있어 `u1 = 0` 인데 진짜 개구가 `u2` 를 대서 합으로 통과한다.

**가설이 아니다.** v1 기하 `y = 0.80` 에서 **현행 HEAD 가 이미 이 유령을 `valid` 로 내고** 336~368 mm 오위치를 보고한다. 독립 재현 두 건이 각각 EPAL 6 기하에서 327 mm, v1 기하에서 338~342 mm 를 얻었다. `y` 를 0.70·0.75·0.78·0.80, 간격 0.20·0.22·0.24, 유령 폭 0.06~0.30 으로 쓸어 보면 **`y >= 0.75` 에서 24 조합 중 22 개**가 유령을 만들고 **`y = 0.70` 에서는 12 조합 전부 정상**이다.

---

## 2. 합격 조건 (실측으로 확정)

> v1 카탈로그 100 장면을 동결 파라미터로 재생했을 때, HEAD 와 다른 장면은 **`s056` 하나뿐**이다(`negative_lookalike`, `no_pallet/no_opening_pattern` → `invalid/opening_width_mismatch` — 둘 다 올바른 거부). 나머지 99 장면은 `diagnostics` 를 제외한 전 필드가 동일하고, **포켓 좌표·yaw·σ 는 100 장면 전부 불변**이다.
>
> `summarize` 에서 움직이는 값은 **`counts.negative_lookalike` 둘뿐**이다: `true_negative` 10→9, `invalid` 0→1. `detection_rate`(positive 0.9833 / occluded 0.25), `false_positive_rate`(둘 다 0.0), `position_error_m`·`yaw_error_rad` 전 분위수, `targets.met`(셋 다 True), `scene_count`, `splits` 는 **전부 불변**이다. 특히 `occluded.invalid` 는 **15 로 유지된다** — 이것이 `min` 단독안과 변이 C 를 가르는 지점이다.
>
> `elapsed_s` 는 합격 조건에서 **제외한다**(§7 참조).

v3 의 *"어떤 집계 지표도 움직이지 않는다"* 도, v4 의 *"`occluded.invalid` 15→6, `false_negative` 0→9"* 도 변이 C 에는 해당하지 않는다. 후자는 `min` 단독안의 값이다.

---

## 3. 이 변경이 새로 들여오는 것 — 없앨 수 없음을 실측으로 확인했다

**앞면이 같은 평면에 맞물린(flush) 개방 구조는 위양성이 된다.** 실질적으로 "바닥판을 뺀 EPAL 6" 이다. 2.5·3.0·3.5 m 에서 전부 `valid` 이고, 기둥 세 개에 상판 하나만 얹은 구조도 같다.

**앞면이 물러난 선반은 위양성이 아니다.** 기둥이 상판보다 뒤에 있으면 RANSAC 이 평면을 둘로 쪼개서(상판 앞면 x=2.70, 기둥 앞면 x=2.93) 개구 위 증거가 44 점밖에 안 잡혀 **전혀 다른 이유로** 거부된다. **그래서 "선반 음성"이라고만 적으면 구현자가 만든 장면이 아무것도 시험하지 않고 통과한다.** 음성 정의에 앞면 정렬 조건을 반드시 넣는다.

**기둥 아래 바닥판만 따로 세는 방식으로는 못 가린다.** 재봤다. 바닥판 대역(`|z − 0.022| <= 0.006`)에 **블록 앞면 하단이 같이 들어온다** — 블록이 z 0.022 에서 시작하기 때문이다. 2 m 에서 EPAL 6 이 기둥당 84 점인데 바닥판을 뺀 구조도 51 점이고, 3.5 m 부터 한 자릿수로 붕괴한다. 문턱 100 근처에도 못 간다.

**정직한 진술:** 이 변경은 판별 근거를 "개구 아래 재료"에서 "개구 위 재료"로 **옮긴다**. 현행은 실물 팔레트를 거부하는 대신 개방 구조를 거부했고, 변경 후는 실물 팔레트를 검출하는 대신 앞면 정렬 개방 구조를 통과시킨다. 이 프레임 안에 둘 다 잡는 규칙은 없다. 연결성 검사나 조사(`docs/references/pallet-detection-survey.md` §5.1)의 다중 특징 점수 구조로 가야 풀린다.

---

## 4. 재현 불가능한 수치를 계획에서 뺀다

v4 가 인용한 `EPAL 6 5/40·34/40`, `상자 세 개 33/40`, `선반 0/21 → 15/21`, `46.4 %(701/1512)`, `0.5 mm 이동에 852 ↔ 168`, `EPAL 6 75 자세` 표는 전부 **gitignore 된 `artifacts/` 안의 리그** 산물이다. 위임 실행자는 재현도 반박도 못 한다.

**처리:** Task 2 가 그 리그를 `tests/unit/perception/test_opening_evidence_cases.py` 로 **커밋한다.** `tests/fixtures/synthetic_scene.py`(추적됨)와 `tools/build_pallet_model.pallet_boxes()`(추적됨)로 EPAL 6·상자 세 개·앞면 정렬 개방 구조·유령 장면을 전부 만들 수 있음을 확인했다. 숫자는 그 시험이 내는 값으로 대체하고, 위 인용은 §1 표에 남은 것만 쓴다. **커밋되지 않은 리그의 숫자는 이 계획의 근거로 쓰지 않는다.**

`3.17 m`(광선 30 %가 360 mm 초과) 는 계획 자신의 닫힌 식에서 나오지 않는다 — 그 식대로면 2.1 m 쯤이다. 삭제한다.

---

## 5. 포켓 깊이 관문을 넣지 않는다 — 측정으로 기각, 요구사항은 존치

광선은 터널을 관통하지 않고 **내려가다 터널 바닥에 닿는다.** 읽을 수 있는 최대 깊이는 닫힌 식으로 고정된다.

```
p_max = (개구 대역 상단 z − 터널 바닥 z) × R_perp / (카메라 높이 − 개구 대역 상단 z)
```

EPAL 6 은 터널 바닥이 실제 바닥(z = 0)이고 대역 여유를 뺀 개구 상단이 0.090 m, 카메라가 0.5 m 라 계수가 **0.2195** 다. `range_min_m` 0.8 m 에서 **176 mm** 가 상한이다. 0.36 m 를 요구하면 v1 검출률 0.800 → **0.362**, EPAL 6 75 자세 중 9 개만 남고 3 m 이내는 전멸한다. 안전한 값 0.09 m 는 현행 대비 실질 이득이 **40 mm** 이고, 그 이름과 값이 **360 mm 를 보장한다는 거짓 확신**을 만든다.

**닫힌 식을 `docs/design/2026-09-13-thin-deck-evidence.md` 에 남긴다**(Task 2). 다음 사람이 0.36 을 다시 넣지 않게.

**단, 요구사항은 죽지 않는다.** 기각된 것은 *이 계측기로 그것을 재는 구현*이지 *막힌 포켓을 거부해야 한다는 요구*가 아니다. 조사가 기록한 업계 방식(포크 장착 2D LiDAR·레이저)은 **이 프로젝트의 하드웨어 계획에 없다** — `docs/hardware.md` 는 D435i 와 RPLIDAR 만 확정이고 추가 센서는 H1 미결이다. Task 10 에서 `docs/hardware.md` 와 로드맵 위험표에 **"삽입 중 독립 접촉/근접 감지가 필요하며 H0/H1 에서 결정한다"** 를 한 줄로 남긴다. 지금은 하류 소비자가 없어(`PocketObservation` 은 `evaluation.py`·`overlay.py` 만 읽는다) 실제 위험은 아니지만, "측정으로 기각"으로만 적어 두면 닫힌 항목으로 읽힌다.

---

## 6. 전역 제약

- **터널 너머에 의존하는 신호를 만들지 않는다.** 시선각 `arctan(227.5/600) = 20.78°` 를 넘으면 관통이 불가능하다. 설계 결정으로 고정한다.
- **v1 카탈로그·데이터 세트·prior 는 읽기만 한다.** 새 음성은 재생성이 아니라 변환 도구로 덧붙인다. 실측: `COUNTS` 에 범주를 더하면 `rng.shuffle`(`generate_scene_catalogue.py:237`)이 밀려 **기존 100 장 중 0 장만 동일**하고, 범주 유지 28 장·자세 유지 4 장이다. 반대로 `sample_catalogue(20260911, 100)` 은 커밋된 YAML 과 비트 단위로 같다 — 재생성 자체는 안전하다.
- dev 에서만 튜닝, **eval 은 한 번**.
- `ruff check .` 와 `ruff format --check .` 통과.
- **기준 회귀:** `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` → **759 passed, 1 deselected, 실패 0**(독립 확인 2회). deselect 되는 1 개는 `tests/simulation/test_forklift_model.py::test_preview_renders_distinct_views_and_records_evidence`(`rendering` 마크). `test_transport_timeout_kills_orphan_*` 는 부하가 높을 때 **간헐적으로** 실패할 수 있으나 기준선은 초록이다. 실패를 보면 그것이 이번 변경 탓인지 먼저 의심한다.
- **`artifacts/` 와 `data/` 는 git 에 없다.** `.gitignore:24`. 위임 실행자에게는 `data/synthetic_scenes/catalogue_v1` 과 `artifacts/20260912T170442Z_pocket_eval_dev_02`·`..._eval_01` 을 **별도로 전달**한다. 전달하지 않으면 `test_detector_v1_replay.py:24` 가 **조용히 skip** 되어 초록으로 보인다.

---

## Task 0: 기준 확인 (Claude)
- [ ] 트리 깨끗, 위 기준 회귀 초록 확인.
- [ ] 아래 덱 증거가 평면 배치에 **조건수가 나쁘다**는 사실을 기록한다. 프로덕션 경로 값은 2.0/3.0/4.0 m 에서 **0/12/9**(독립 재현 일치).

## Task 1: 원격 경로 수리 (위임)

**Files:** `tools/submit_model_check.py`, `tools/remote_model_job.py`, `tests/integration/test_evaluate_cli.py`, `tools/evaluate_pocket_detector.py`, `tests/integration/test_remote_model_jobs.py`

- [ ] snapshot 허용 목록에 `config` 와 `tests` 의 `.yaml` 을 추가한다. 실측: 정확히 **6 개** 파일이 새로 포함되고(91 → 97) 비밀 파일은 없다. 현재는 `tests/fixtures/thin_deck_legacy_*.yaml` 이 안 가서 — 두 파일이 **모듈 import 시점**에 로드되므로 — **원격 pytest 가 수집 단계에서 죽는다.**
- [ ] **`git_revision` 을 고친다.** `test_evaluate_cli.py:313` 이 길이 40 을 요구하는데 snapshot 에 `.git` 이 없어 `_git_state` 가 `None` 을 돌려주고 `TypeError` 가 난다. **방향을 여기서 정한다: assert 를 완화하지 말고, 스냅샷 생성 시 `git_revision` 을 메타데이터로 주입한다.** 완화하면 `run.json` 에 `(None, None)` 이 그대로 실려 Task 10 의 "고정 revision 에서 eval 1 회"를 **산출물로 증명할 방법이 사라진다.**
- [ ] **`scenes` 모드에 코어 패키지를 설치하지 않는다.** `remote_model_job.py:494-506` 의 `core_environment_builder` 는 `model-cpu`/`model-render` 에서만 돈다. Task 3 이 `forklift_core` 를 import 하지 않으므로 이대로 둔다. **이 항목은 결정 완료이며 실행자가 다시 고를 사항이 아니다.**
- [ ] **완료 조건 두 가지:** ① 원격 `model-cpu` job 초록 ② **`scenes` 모드로 1~2 장면 스모크 캡처 성공.** ②가 없으면 `scenes` 경로가 처음 도는 시점이 Task 7 의 되돌릴 수 없는 140 장면 캡처가 된다.
- [ ] v1 재현 시험 2 개는 원격에서 skip 된다(`artifacts/` 제외). 기록만 한다.

## Task 2: 변이 C 구현과 명시 진단 (위임)

**Files:** `src/forklift_core/perception/pocket_detector.py`, `tools/evaluate_pocket_detector.py`, `tests/unit/perception/test_thin_deck_evidence.py`, `tests/unit/perception/test_pocket_detector.py`, Create `tests/unit/perception/test_opening_evidence_cases.py`, `tests/integration/test_detector_v1_replay.py`, `docs/design/2026-09-13-pocket-detector-baseline.md`, `docs/design/2026-09-13-thin-deck-evidence.md`, `docs/validation/2026-09-13-thin-deck-evidence.md`, `docs/interfaces/pocket-observation.md`

- [ ] §1 의 규칙 1–5 를 구현한다. `no_upper_deck` 을 새 `reason` 으로 추가하고 인터페이스 문서에 올린다.
- [ ] **`_Pattern` 에 `upper_ok: bool` 과 `lower`·`upper_left`·`upper_right`·`support_min` 을 스칼라로 추가한다.** `support_count` 는 **합계**라 최솟값 지표로 쓸 수 없다.
- [ ] **`_Pattern.deck_count` 와 점수식을 이렇게 정한다:** `deck_count = lower + upper_left + upper_right`(현행은 `lower + upper` 전 구간), 점수식 `score = support_count + deck_count` 는 **그대로 둔다**. 정렬만 §1 규칙 4 로 바꾼다. *이 정의를 안 바꾸고 게이트만 바꾸면 `test_thin_deck_evidence.py` 23 개가 전부 통과해 버려서, 하니스가 `deck_count − upper` 로 틀린 `lower` 를 계속 재는 결함(2.0 m 에서 7860 대 실제 8874)이 신호 없이 남는다.* 바꾸면 시험 3 개(`test_a_24mm_strip_ahead...`, `test_a_box_800mm_beyond_the_rear...`, `test_the_floor_still_supplies_zero_lower_evidence`)가 실패하며, 하니스를 명시 필드 직독으로 고치는 것이 그 수정이다.
- [ ] `DetectionDiagnostics` 의 새 필드는 **`exception_traceback` 앞**에 넣는다. 뒤에 붙이면 기본값 순서 때문에 클래스 생성 시점에 `TypeError` 로 import 가 깨진다. 두 dataclass 모두 위치 인자로 생성된다(`:347`, `:537-551`). 패턴 미선택 시 `None`.
- [ ] `SCENE_COLUMNS` 에 새 진단 이름을 올린다. 안 올리면 **조용히 버려진다.**
- [ ] **bool `DetectorParams` 항목을 만들지 않는다.** `_finite_scalar` 가 bool 을 거부해 모듈 import 가 깨진다. (`_Pattern.upper_ok` 는 파라미터가 아니라 무방하다.)
- [ ] **`test_detector_v1_replay.py` 를 고친다.** 기대 델타는 `s056` 한 장면이다. **보관된 `run.json`·관측 파일을 고치지 말 것** — 그것은 과거 실행 기록이고, 덮어쓰면 이 회귀를 영원히 반증 불가능하게 만든다. 대신 **예상 델타를 시험 파일 안의 추적되는 상수로** 넣고 `assert differences == EXPECTED_DELTA` 형태로 바꾼다.
- [ ] **새 시험 파일 `test_opening_evidence_cases.py`** — §4 의 리그를 커밋한다. `tests/fixtures/synthetic_scene.py` 의 `make_pallet_scene`·`_box_depth` 와 `pallet_boxes(load_pallet_geometry(...))` 로 만든다. 최소 네 가지:
  - **EPAL 6** 2.0~4.0 m: `valid`, 포켓 y = ±0.186 ± 0.01.
  - **상자 세 개**(블록 아홉 개만, 바닥판·스트링거·상판 없음): `no_pallet`.
  - **앞면 정렬 개방 구조**(바닥판만 뺀 EPAL 6): `valid` — **알려진 한계로 고정**하고 "연결성 검사를 넣으면 뒤집히는 것이 정상"이라고 주석에 적는다. 앞면이 물러난 변형도 같이 넣어 **거부 이유가 다르다는 것**(`u = 44`, 평면이 둘로 쪼개짐)을 못 박는다.
  - **유령 패턴**: 팔레트 x 2.5 / **y = 0.80**(0.70 아님 — v1 기하 0.70 은 두 규칙 모두 정상 통과라 시험이 조용히 통과한다), 같은 평면 상자를 **화각 중심 쪽(−y)** 바깥 블록에서 0.22 m. **사용 기하를 시험 안에 명시한다.** 합계 규칙에서 `valid`(320~370 mm 오위치), 변이 C 에서 `no_pallet` 이어야 한다.
- [ ] 무효가 되는 계약을 같은 커밋에서 갱신한다: `baseline.md` 의 "위·아래 덱 대역에 점유가 있어야", `thin-deck-evidence.md` 의 `min(지지대 3개, lower, upper)`, `validation/thin-deck-evidence.md` 의 "deck_count 에서 upper 를 뺀다".
- [ ] **포켓 깊이 닫힌 식**(§5)을 `thin-deck-evidence.md` 에 남긴다.
- [ ] **인터페이스 문서에 한 줄:** `center_m` 의 **z 는 prior 유래 상수**(`pocket_detector.py:421`)이고 정답도 같은 값이라 **`position_error_m` 의 z 성분은 구조적으로 항상 0** 이다. 보고되는 오차는 사실상 평면 2 차원이다.

## Task 3: 월드 생성기가 두 기하를 받게 한다 (위임)

**Files:** `sim/gazebo/build_scene_world.py`, `tests/simulation/test_build_scene_world.py`

- [ ] **치수 출처는 `config/pallet_geometry_epal6.yaml` 직독이다.** Task 1 이 그 파일을 스냅샷에 넣으므로 원격에서도 읽을 수 있다. **`forklift_core` 는 import 하지 않는다**(`scenes` 모드에 코어 설치가 없다). 카탈로그 헤더는 **쓰지 않는다** — `pallet` 블록은 6 필드(`depth_m·width_m·height_m·deck_m·center_spacer_m·opening_height_m`)인데 22 상자에는 13 필드가 필요하다. 13 필드짜리 헤더 스키마를 **발명하지 말 것.**
- [ ] **v1 경로는 그대로 둔다.** `catalogue_version == "v1"` 이면 현행 5 상자 배출과 현행 `pallet` 완전일치 검사를 유지한다. `catalogue_version == "epal6"` 일 때만 22 상자 경로를 탄다. 그래서 `tests/simulation/test_build_scene_world.py:37`(collision 5 개), `:40`(outer 2 개), `:144`(스페이서 이름 집합)이 **계속 통과해야 한다.**
- [ ] **계속 통과해야 하는 3 파일:** `tests/simulation/test_build_scene_world.py`, `tests/simulation/test_capture_scenes.py`, `tests/simulation/test_scene_catalogue.py`.
- [ ] 22 상자 배치를 **여기서 다시 유도하지 말 것.** `tools/build_pallet_model.pallet_boxes()` 와 중복 구현이 되면 둘이 갈라진다. `sim/models/epal6_pallet/pallet.urdf`(22 visual, 검증됨)를 읽거나, 같은 YAML 에서 같은 식으로 뽑되 **`pallet_boxes()` 결과와 이름·크기·중심이 일치함을 확인하는 시험**을 넣는다.
- [ ] `sdf_parts.add_urdf_visuals` 는 **재사용 불가**다: 시그니처 `(world, path)` 로 pose 인자가 없고, 모델명이 `"provisional_forklift_visuals"` 로 하드코딩되며(`:69`), visual 만 내고 collision 이 없다(생성 월드에서 193 visual / 0 collision).
- [ ] 새 음성 두 종의 배출 경로를 만든다. `_require_keys`(`:24-26`)가 **집합 동일성**이므로 새 기하 필드를 넣으려면 140 장면 전부와 그 집합에 키를 추가해야 한다.
- [ ] **시험: 정답 포켓 중심을 지나는 광선이 팔레트 상자 어느 것과도 교차하지 않는다**(개구가 실제로 비어 있음). 상자 개수·크기만 보는 시험은 블록 아홉 개를 같은 x 에 놓아도 통과한다.

## Task 4: 카탈로그 변환과 새 음성 (위임)

**Files:** Create `tools/retarget_scene_catalogue.py`, `sim/gazebo/scenes/catalogue_epal6.yaml`; Modify `src/forklift_core/perception/evaluation.py`, `tools/merge_scene_batches.py`, `tools/evaluate_pocket_detector.py`, `sim/gazebo/build_scene_world.py`, `docs/interfaces/scene-dataset.md`, `docs/design/2026-09-11-pocket-observation-and-scene-set.md`, `docs/validation/2026-09-11-scene-catalogue-and-world.md`

- [ ] v1 의 `x_m`·`y_m`·`yaw_rad`·범주·분할·조명·표면·distractor 를 **그대로** 옮긴다. **자세 100 장 완전 일치를 시험으로 고정한다**(approx 아님).
- [ ] **재계산 대상 필드를 전부 열거한다.** v1 의 `opening_width_m` 은 80 장면에 걸쳐 **78 가지 값(0.2007~0.2794)** 인데 EPAL 6 은 기하가 **0.2275 로 고정**한다(prior 허용 범위 `[0.2075, 0.2475]` 안). 자세만 옮기면 상당수가 `opening_width_mismatch` 가 된다. 바꿔야 하는 것:
  - `pallet.opening_width_m` → 0.2275 고정
  - `ground_truth.{left,right}.center_m` — y 오프셋 ±0.18625, **z 0.15 → 0.061**
  - `ground_truth.{left,right}.width_m` → 0.2275, `.height_m` **0.20 → 0.078**
  - `occluder.center_m`·`size_m` — `size[1] = fraction × opening_width_m` 로 재계산(`generate_scene_catalogue.py:200`)
  - `visibility.corners_px`, `visibility.occluded_fraction_image` — **이미 있는 필드다**(s001 = 0.431511). 발명이 아니라 새 기하로 재계산이다.
- [ ] **occluder 의 `fraction` 은 물리 폭 비율이지 영상 가림률이 아니다**(s001: 0.3553 × 0.2474 = 0.087901). 같은 fraction 이 같은 난이도를 뜻하지 않는다는 것을 인터페이스 문서에 적는다.
- [ ] 새 음성 두 종을 **종별 20 장**(`s101`–`s140`) 덧붙인다. 분할은 `len(indices) * 7 // 10` 이 적용되어 dev 14 / eval 6 이다. `_scene_ids` 정규식 `s[0-9]{3}` 이 받는다.
  - `negative_block_row`: 블록 아홉 개만. 바닥판·스트링거·상판 없음.
  - `negative_open_bay`: **앞면이 팔레트와 같은 평면에 맞물린** 개방 구조(상판 + 기둥, 바닥판 없음). **앞면 정렬이 이 음성의 전부다** — 물러나면 다른 이유로 거부되어 아무것도 시험하지 않는다(§3).
- [ ] 데이터 세트 디렉터리는 `data/synthetic_scenes/catalogue_epal6/`. **`catalogue_v1` 을 덮지 않는다** — 두 `run.json` 의 `dataset_dir` 가 그 경로를 가리키므로 재현 시험의 **주 경로**가 깨진다(폴백은 같은 경로라 안 뜬다).
- [ ] 범주 등록 **네 곳 전부**: `evaluation.py:18 NEGATIVE_CATEGORIES`, `merge_scene_batches.py:41-46 CATEGORY_STATUS`(새 두 종의 `ground_truth.status` 는 **`no_pallet`**), `build_scene_world.py:78-83` 화이트리스트, `evaluate_pocket_detector.py:292-296`. 미등록이면 전부 `ValueError` 다.

## Task 5: 동결 게이트 (Claude)
- [ ] **캡처는 되돌릴 수 없다.** `merge_batches` 는 배치 manifest 를 `expected_metadata` 와 완전일치시키는데(`:174-187`), 거기에는 `catalogue_sha256` 뿐 아니라 **`source_snapshot_sha256`·`image_id`** 가 들어 있다. 즉 배치 1 부터 병합까지 **스냅샷 허용 목록에 속한 어떤 파일도**(Task 1 이 새로 넣은 `config/*.yaml`·`tests/**/*.yaml` 포함) 건드리면 전 배치가 폐기된다. 전수 커버리지도 요구한다(`:220-222`).
- [ ] 배치 수는 **코드 요구가 아니라 우리 선택**이다(`--batches` 는 `nargs="+"`).
- [ ] 카탈로그·월드 생성기·prior·`config/*.yaml`·`tests/**/*.yaml` 을 커밋하고 그 revision·카탈로그 해시·스냅샷 해시를 기록한 뒤 캡처한다. **캡처 중 Task 8 의 파라미터 파일 작업을 먼저 손대지 않는다.**

## Task 6: 캡처·병합 (Claude + 원격)
- [ ] 140 장면 캡처, 병합, set manifest 기록, 그룹·권한 확인.

## Task 7: 동결 파라미터 파일 정리 (Claude)
- [ ] **Task 2 뒤, Task 8 앞.** Task 2 가 진단 필드를 추가하고 Task 8 이 값을 정하므로 앞에 두면 두 번 한다.
- [ ] `config/detector_params_v1.yaml` 은 키 **16 개**인데 `DetectorParams` 는 **17 개**다(`deck_evidence_tol_m` 누락, 기본값 0.006). `_load_params` 는 모르는 키는 거부하고 **없는 키는 조용히 코드 기본값으로 채운다.**
- [ ] **동결 파일을 고치지 않는다.** 두 과거 `run.json` 도 16 키라 `test_detector_v1_replay.py:31` 의 `params_data == run["params"]` 가 지금은 True 이고 키를 추가하는 순간 False 가 된다(직접 로드해 확인). 시험을 **"파일 ⊆ run.params 이고 나머지는 코드 기본값과 같다"** 로 바꾸는 편이 정직하다. 근거를 기록한다.
- [ ] EPAL 6 튜닝 결과는 Task 8 이 끝난 뒤 `config/detector_params_epal6.yaml` 로 저장한다.

## Task 8: dev 튜닝 (Claude)
- [ ] 기준선 1 회 후 한 번에 하나씩.
- [ ] 지표에 **지지대 최솟값**을 넣는다. 4 m 에서 `min()` 을 결정하는 것은 `upper`(692, 문턱의 6.9 배)도 `u1/u2`(256/268)도 아니고 **지지대**(144/207/144, 최소 1.44 배)다.
- [ ] **위양성 지표에 `invalid` 를 함께 적는다.** `false_positive_rate` 는 `valid` 만 세고(`evaluation.py:170-173`), `evaluate_scene` 은 `invalid` 를 음성 분기보다 **먼저** 처리한다(`:101-103`).
- [ ] **`floor_z_m >= deck_bottom_m` 을 설정 오류로 거부**한다. 현행 `floor_z_m` 는 기본·동결 모두 **0.020** 이고 EPAL 6 `deck_bottom_m` 는 **0.022** — 2 mm 차다. `DetectorParams.__post_init__` 은 prior 를 못 보므로 거기 넣을 수 없다. **`detect_pockets` 진입부와 `evaluate_pocket_detector._run` 두 곳**에 넣는다.
- [ ] **처리 시간은 합격 조건이 아니다.** 변이 C 는 후보가 82 → **95** 로 늘어 광선 분류가 더 돈다. 그런데 같은 기계에서 순차 best-of-3 는 +56 %, 장면별 교차 best-of-3 는 −0.3 % 를 준다 — **부하 드리프트가 신호를 압도한다.** 재려면 교차 best-of-3 로만 재고, 수치를 목표로 쓰지 않는다. v4 의 "p50 −5.7 %" 는 재현되지 않았다.

## Task 9: 동결 게이트 · Task 10: eval 1 회와 기록 (Claude)
- [ ] 고정 revision·clean 트리에서 eval 1 회. 목표 도달 여부를 있는 그대로.
- [ ] 검증 기록에 적을 것: 위양성률과 `invalid` 병기 / **앞면 정렬 개방 구조 위양성이 이 변경으로 새로 생긴다는 사실과 §3 의 "못 없앤다" 측정** / `position_error_m` 의 **z 성분이 구조적으로 0** 이라는 점 / `POSITION_TOLERANCE_M` 0.20 m 는 대응 문턱이지 도킹 허용치가 아니며 포크 좌우 여유 45 mm·수직 여유 6 mm 와 무관하다는 점 / 해결하지 않은 것들(§9).
- [ ] `docs/hardware.md` 와 로드맵 위험표에 §5 의 접촉/근접 감지 요구를 한 줄 남긴다.

---

## 결정 지점

새 음성 종별 20 장, dev 14 / eval 6. dev 해상도 1/14 ≈ 7 % 이므로 **`negative_block_row` dev 위양성 2 장 이상(≈14 %)** 이면 규칙을 다시 본다.

`negative_open_bay` 는 **통과가 예상된다**(§3). 그래서 이 20 장을 캡처 슬롯에 쓸지 자체가 결정 사항이다 — 아래 §9 ①.

## 9. 사용자 결정이 필요한 것 (이 계획이 혼자 정하지 않는다)

① **`negative_open_bay` 20 장을 실제로 찍을 것인가.** 통과가 예상되고, 현재 `targets.met` 에는 **위양성 목표가 아예 없다**(`evaluation.py:210-225` — `position_p95`·`yaw_p95`·`detection_rate` 셋뿐). 문턱을 걸지 않으면 "70 % 위양성을 보고 적고 모든 성문 합격 조건을 만족한 채" 끝낼 수 있다. 셋 중 하나: (a) 위양성 예산을 `targets` 에 추가하고 합불 판정한다 (b) 이 구조를 운용 범위 밖으로 선언하고 20 장을 **안 찍는다** (c) 지금처럼 별도 보고만 한다.

② **140 장을 다 찍을 것인가.** 자세 범위 `x 2–4 m, y ±1.0, yaw ±0.52` 는 브리프 요구가 아니라 2026-09-11 에 카탈로그와 함께 정해진 값이다. 조사에 따르면 상용 제품은 1–3 m 에 **대략적 사전 위치까지 주고** 동작하고, 2 m 전방·1 m 측방은 이미 26.6° 로 §6 의 20.8° 터널 한계를 넘는다. 운용 범위를 좁혀 더 작은 세트로 가는 선택지가 있다 — **데이터를 답에 맞추는 것이 아니라 요구를 명시하는 것**이다.

③ **MR6D 실데이터를 M2 안으로 당길 것인가.** CC-BY-4.0, **D435i(우리 카메라)**, 유로 팔레트 6D 정답. 하드웨어 입고 전에 쓸 수 있는 **유일한 실측 깊이 증거**이고, 설계 문서가 스스로 나열한 실센서 위험 다섯 가지(스침각, 거리별 깊이 잡음, flying pixel, 휜 덱, 근거리 자기가림)는 **합성 깊이에서 전부 안 보인다.**

## 10. 범위 밖 (의도)

터널 너머 신호(금지), 포켓 깊이 관문(구현 기각 — 요구는 §5 로 존치), 형판 정합·PnP, 딥러닝, 마커, 다중 시점 융합(M3), `evaluation.py` 의 **목표 상수** 변경(범주·위양성 예산 추가는 §9 ①의 결정 사항), `s009` 진단 분리(별건), 게이트 아키텍처 자체의 재설계(조사 §5.1 의 "포켓 관측을 필수 관문에서 빼고 선택적 증거로 강등" — 이번 범위에서 다루지 않으며, 그 선택을 하지 않은 이유는 **M2 가 포켓 관측을 산출물로 정의**하기 때문이다).

---

## 11. 자체 검토 기록

독립 검토 **12회**(v1~v5). 각 회차는 서로의 결론을 보지 않았다.

**4회차(v4 대상, 3건)가 잡은 것과 처리:**

| 지적 | 처리 |
|---|---|
| `min(u1,u2)` 가 `pocket_occluded` 를 `no_pallet` 으로 뭉갠다 — 설계 문서가 같은 모양을 "명백한 퇴행"이라 적었다 | **변이 C 채택.** 직접 구현·실측해 v1 99/100, 가림 라벨 15 건 전부 보존 확인 |
| 기준 회귀가 "758 passed, 1 failed" 가 아니라 **759 passed, 0 failed** | 수정. flake 는 "간헐적"으로 낮춤 |
| 후보 수 82→79 는 합계 규칙 값, 채택안은 82→70 | 수정. 변이 C 는 82→**95**(직접 측정) |
| 처리 시간 p50 −5.7 % 재현 안 됨(교차 best-of-3 로 −0.3 %, 순차로 +56 %) | 삭제. 시간을 합격 조건에서 뺌 |
| 유령 시험 자세 y 0.70 은 v1 기하에서 **조용히 통과** | y 0.80 으로 바꾸고 기하 명시 |
| 카탈로그 헤더는 6 필드, 22 상자는 13 필드 — Task 3 제약이 불가능 | 치수 출처를 `config/*.yaml` 직독으로 변경 |
| "22 상자로 바꾼다" 가 v1 시험 3 개와 모순 | v1 5 상자 경로 존치, epal6 만 22 상자 |
| `scenes` 모드가 Task 7 캡처에서 처음 돈다 | Task 1 완료 조건에 스모크 캡처 추가, 코어 설치 문제는 계획이 결정 |
| 재현 시험 기대값·입력이 untracked → 새 clone 은 **skip 초록** | 전달 의무를 전역 제약에 명시, 기대 델타를 추적 상수로 |
| `deck_count`·점수식 미지정 → 시험 3 개가 안 울린다 | 정의를 Task 2 에 못 박음 |
| 리그가 gitignore → 계획의 수치를 재현·반박 불가 | §4. 리그를 `test_opening_evidence_cases.py` 로 커밋 |
| v1 개구 폭 78 가지 vs EPAL 6 고정 0.2275 | Task 4 에 재계산 필드 전수 열거 |
| 동결 게이트가 `source_snapshot_sha256` 을 빠뜨림 | Task 5 에 추가 |
| `position_error_m` 의 z 는 구조적으로 0 | Task 2·10 에 기록 |
| 포크 장착 센서는 이 프로젝트 하드웨어 계획에 없다 | §5. "요구는 존치" 로 분리 |
| 위양성 목표가 `targets` 에 없어 새 음성이 판정을 못 움직인다 | §9 ① 사용자 결정 |
| 자세 범위·140 장이 근거 없이 정해졌고 MR6D 가 이유 없이 범위 밖 | §9 ②③ 사용자 결정 |
| `3.17 m` 가 자신의 닫힌 식과 불일치 | 삭제 |

**내가 직접 재계산한 것:** 변이 C 의 v1 100 장면·집계 델타, EPAL 6/상자 세 개/개방 구조/유령 케이스 행렬, 앞면 정렬 여부에 따른 거부 이유 차이, 기둥별 바닥판 증거(못 가림), 후보 수 82/70/95, `targets.met` 구성, 게이트-광선분류 순서, `centre[2]` prior 유래, 카탈로그 헤더 6 필드, v1 개구 폭 78 가지.

**반박되지 않은 것:** §1 의 규칙과 §2 의 합격 조건은 세 검토 중 둘이 독립적으로 재계산해 일치했다.
