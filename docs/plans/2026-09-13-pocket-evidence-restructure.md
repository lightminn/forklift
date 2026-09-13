# 포켓 증거 구조 변경과 M2 재평가 계획 (v3)

> 상태: **v3 — 독립 검토 6회 반영.** v1 초안 `22e967b`, v2 `1644063`. 이번 라운드는 Codex 없이 서브에이전트 3회를 서로 모르게 돌렸고, 그 결과 v2 의 핵심 선택을 뒤집었다.
> **실행자:** Task 1–5 위임, 나머지 Claude.

**목표:** 실물 EPAL 6 형상에서 포켓을 검출하지 못하는 현재 상태를 풀고, EPAL 6 장면 세트로 M2 를 다시 평가한다.

## 검토 6회가 확정한 것

**표는 독립 재현됐다.** 검증자가 카탈로그 카메라 기록에서 광선추적기를 새로 쓰고, 장면을 새로 구성하고, 실제 `detect_pockets` 와 매 장면 대조하며 20 칸을 모두 재현했다. 불일치 장면 ID 목록까지 같다.

| 규칙 | v1 재현 | EPAL 6 | 상자 세 개 | 상자 세 개(높음) | 선반 |
|---|---|---|---|---|---|
| 현행 | 100/100 | 5/40 | 1/40 | 1/40 | 0/40 |
| `upper` 전 구간 + lower 관문 제거 | 99/100 | 34/40 | 33/40 | 34/40 | 35/40 |
| **개구별 AND `min(u1,u2)`** | **91/100** | **34/40** | **0/40** | **0/40** | 35/40 |
| 개구 위 합계 `u1+u2` | 100/100 | 34/40 | 0/40 | 0/40 | 35/40 |

**v2 는 합계를 골랐다. 그것을 뒤집는다.**

## v2 의 선택을 뒤집는 두 측정

**1. 합계는 위험한 위양성을 통과시킨다.** 팔레트를 화각 가장자리에 두고 옆에 같은 평면인 상자를 놓으면, 먼 쪽 바깥 블록이 잘려 진짜 패턴이 지지대 74 점으로 탈락하고, 대신 [이웃 상자 | 공백 | 팔레트 바깥 블록 | 진짜 포켓] 쌍이 성립한다. 가짜 개구 위는 비어 있어 `u1 = 0` 인데 진짜 개구가 `u2 = 1105` 를 대서 **합으로 관문을 통과한다.** 결과는 미검출이 아니라 **확신에 찬 오위치**다. 한쪽 포크는 진짜 포켓으로, 다른 포크는 팔레트 옆 빈 공간으로 들어간다. `min(u1, u2)` 는 이것을 거부한다.

**2. `min` 의 91/100 은 지표상 영향이 없다.** 깨지는 9 장면은 전부 `occluded` 이고 `invalid/pocket_occluded:*` → `no_pallet/no_opening_pattern` 으로 바뀐다. 둘 다 `valid` 가 아니다. `summarize` 의 `detection_rate` 는 TP 만, `false_positive_rate` 는 음성 범주만, 오차 통계는 `valid` 만 본다 — **셋 다 불변**이다. 바뀌는 것은 `Outcome` 라벨과 이유 문자열뿐이다.

**따라서 `min(u1, u2)` 를 채택한다.** 대가는 가림 장면 9 개의 라벨이 덜 구체적으로 바뀌는 것이고(안전한 실패), 이득은 포크를 빈 공간에 밀어 넣는 경로를 막는 것이다.

**합격 조건을 다시 쓴다.** "v1 100/100" 이 아니라 **"v1 에서 바뀌는 것은 그 9 개 `occluded` 장면의 라벨뿐이고, 나머지 91 장면은 전 필드 동일하며, 어떤 집계 지표도 움직이지 않는다"** 이다.

## 바로잡는 서술

- v2 의 *"현행은 실물 EPAL 6 을 한 장면도 검출하지 못한다"* 는 **정면 3 거리에 한정된 문장**이었다. 40 자세에서는 5/40 이 통과한다. 아래 덱 증거가 자세에 따라 0~288 로 널뛰기 때문이고, 통과 장면은 정확히 `lower >= 100` 인 장면이다.
- v2 의 *"점수식에서 `lower` 를 빼면 마진이 반감하므로 남긴다"* 는 제약은 **실측상 무의미**하다. 점수 구성 세 변형에서 260 장면 중 상태가 바뀌는 장면이 0 이었다. 그래도 남기는 쪽이 정보를 잃지 않으므로 유지한다.
- **선반은 팔레트보다 자주 통과한다**(35/40 대 34/40). 규칙 변경으로 줄지 않는다. 알려진 한계로 고정한다.

## 아직 없는 관문 — 포켓 깊이

검토가 만든 위양성 중 **가장 현실적인 것들이 전부 같은 구멍을 통과한다.** 화물이 흘러내려 포켓을 막은 팔레트, 받침목 위 화물, 100 mm 뒤가 막힌 구멍, 발 셋 달린 플린스. **인식기에 포켓 깊이 검사가 아예 없다.** 광선 판정의 "뒤" 기준이 `front_margin_m` 0.05 라 5 cm 만 뒤면 뚫린 것으로 센다. 포크는 360 mm 를 넣어야 한다.

Task 2 에서 이것을 다룬다. 이 계획에서 가장 실질적인 안전 개선이다.

## 전역 제약

- **터널 너머에 의존하는 신호를 만들지 않는다.** 포켓은 폭 227.5 mm·깊이 600 mm 터널이라 시선각 `arctan(227.5/600)` = **20.8°** 를 넘으면 관통이 불가능하다. 설계 결정으로 고정한다.
- **v1 카탈로그·데이터 세트·prior 는 읽기만 한다.** 새 음성은 `sample_catalogue` 재생성이 아니라 변환 도구로 덧붙인다. 실측: `COUNTS` 에 범주를 더하면 `rng.shuffle` 이 밀려 **기존 100 장 중 0 장만 동일**하다.
- dev 에서만 튜닝, **eval 은 한 번**.
- **기준 회귀는 `tests` 686 + `ros2/src/forklift_ros/test` 73 = 759 passed, 1 deselected.** `tests` 만 돌리면 686 이다.
- `tests/integration/test_remote_model_jobs.py::test_transport_timeout_kills_orphan_...` 는 **부하 시 간헐 실패하는 기존 flake** 다. 이번 변경 탓으로 오인하지 않는다.

---

### Task 0: 기준 확인 (Claude)
- [ ] 트리 깨끗. 회귀 759(=686+73) passed, 1 deselected.
- [ ] 아래 덱 증거가 **평면 배치에 조건수가 나쁘다**는 사실을 기록한다(0.5 mm 이동에 852 ↔ 168). 이전에 나온 서로 다른 표들은 어느 쪽이 맞는 문제가 아니라 전부 리그 산물이었다.

### Task 1: 동결 파라미터 파일 재생성 (위임, 동작 변화 0)

**Files:** Modify `config/detector_params_v1.yaml`

실측: 이 파일은 키 16 개인데 `DetectorParams` 는 필드 17 개다. `deck_evidence_tol_m` 이 빠져 있고 `_load_params` 가 **누락 키를 조용히 코드 기본값으로 채운다.** 검증 기록의 "유효 파라미터 전체가 이 파일에 있다"는 서술이 지금 거짓이다.

- [ ] `asdict(DetectorParams())` 로 재생성한다. **동작은 변하지 않는다**(값이 같다).
- [ ] `_load_params` 가 불완전한 매핑을 거부하도록 하거나, 최소한 시험으로 키 집합 일치를 고정한다.
- [ ] 이것을 안 하면 Task 6·7 의 동결 게이트가 의미가 없다.

### Task 2: 원격 경로 수리 (위임)

**Files:** Modify `tools/submit_model_check.py`, `tests/integration/test_evaluate_cli.py`

- [ ] snapshot 허용 목록에 `config` 와 `tests` 의 `.yaml` 을 추가한다. 실측: 정확히 6 개 파일이 새로 포함되고 빠지는 것은 없으며 비밀 파일도 없다. 현재는 `tests/fixtures/thin_deck_legacy_*.yaml` 이 안 가서 **원격 pytest 가 수집 단계에서 죽는다.**
- [ ] **`git_revision` 문제를 같이 고친다.** `test_run_metadata_...` 가 `len(git_revision) == 40` 을 요구하는데 snapshot 에는 `.git` 이 절대 안 들어가 `None` 이 되고 `TypeError` 가 난다. 이 assert 는 `3465c8d` 에서 들어왔고 **그 이후 원격 pytest 가 한 번도 안 돌았다.** 시험이 revision 미상을 허용하게 한다.
- [ ] v1 재현 시험 2 개는 원격에서 **skip** 된다(`artifacts/` 가 snapshot 제외). 정상이며 기록만 한다.

### Task 3: 관문 변경과 명시 진단 (위임)

**Files:** Modify `src/forklift_core/perception/pocket_detector.py`, `tools/evaluate_pocket_detector.py`, `docs/design/2026-09-13-pocket-detector-baseline.md`, `docs/design/2026-09-13-thin-deck-evidence.md`, 관련 시험

- [ ] 판정을 **`min(지지대 3 개, u1, u2) >= min_band_points`** 로 바꾼다. `u1`·`u2` 는 **각 개구 바로 위** 구간의 상한 증거다. 중앙 지지대 위는 세지 않는다. 아래 덱 증거는 관문에서 뺀다.
- [ ] `_Pattern` 과 `DetectionDiagnostics` 에 **`lower`, `upper_per_opening`, `support_per_section`** 을 명시 필드로 추가한다. 현재 시험 하니스는 `deck_count − upper` 로 `lower` 를 역산하는데, 집계가 바뀌면 **시험 3 개가 실패하고 다른 3 개는 통과하면서 틀린 값을 잰다**(2.0 m 에서 7860 대 실제 8874). 하니스를 명시 필드 직독으로 바꾼다.
- [ ] **`DetectionDiagnostics` 의 새 필드는 `exception_traceback` 앞에 넣는다.** 뒤에 붙이면 기본값 순서 때문에 `TypeError` 로 import 가 깨진다. 두 dataclass 모두 위치 인자로 생성되므로 생성 지점을 같은 커밋에서 고친다. 패턴 미선택 시 `None`.
- [ ] **`support_per_section` 을 지금 넣는다.** `_Pattern.support_count` 는 **합계**지 최솟값이 아니고 진단에 지지대 필드가 아예 없다. Task 6 에서 지지대 최솟값을 지표로 쓰려면 필요하다. 안 넣으면 dataclass 를 두 번 갈아엎는다.
- [ ] `tools/evaluate_pocket_detector.py` 의 `SCENE_COLUMNS` 에 새 진단 이름을 올린다. 안 올리면 **조용히 버려진다.**
- [ ] **bool 파라미터를 넣지 않는다.** `_finite_scalar` 가 bool 을 거부해 모듈 import 가 깨진다.
- [ ] 두 설계 문서의 무효가 되는 계약을 같은 커밋에서 갱신한다(`baseline.md` 의 "위·아래 덱 대역에 점유가 있어야", `thin-deck-evidence.md` 의 `min(지지대 3개, lower, upper)`).
- [ ] **합격 조건:** v1 에서 바뀌는 것은 9 개 `occluded` 장면의 라벨뿐이고 나머지 91 장면은 전 필드 동일, 집계 지표 불변.
- [ ] 시험: 상자 세 개가 거부되고 EPAL 6 이 통과한다. **유령 포켓 반례**(팔레트를 화각 가장자리에, 옆에 같은 평면 상자)가 거부되는 것을 고정한다. **선반은 통과한다는 사실**을 알려진 한계로 고정한다. EPAL 6 fixture 는 커밋된 `sim/models/epal6_pallet/pallet.urdf` 를 추적해 만든다 — `artifacts/` 의 옛 `epal6_boxes` 는 **연속 통판이라 이 문제를 숨긴다.**

### Task 4: 포켓 깊이 관문 (위임)

**Files:** Modify `src/forklift_core/perception/pocket_detector.py`, 관련 시험

- [ ] 광선 "뒤" 판정 기준을 `front_margin_m` 0.05 에서 **실제 삽입 깊이 기준**으로 바꾼다. 새 파라미터 `min_pocket_depth_m`, 기본값은 도킹 계획의 삽입 깊이 0.36 m 에서 정한다.
- [ ] **먼저 측정한다.** 이 관문이 v1 100 장면과 EPAL 6 정상 장면을 떨어뜨리지 않는지 확인하고, 떨어뜨리면 기본값을 낮춰 잡되 **그 값과 근거를 기록한다.**
- [ ] 시험: 깊이 60·100·200 mm 로 막힌 포켓이 거부되고, 뚫린 포켓은 통과한다.

### Task 5: 월드 생성기와 카탈로그 변환 (위임, 둘로 쪼갬)

**5a — 월드 생성기가 두 기하를 받게 한다**
- [ ] `build_scene_world.load_catalogue` 의 `catalogue_version == "v1"` 고정과 `catalogue["pallet"]` **완전일치** 검사를 두 기하 모두 받도록 바꾼다.
- [ ] 팔레트 배출을 **22 상자**로 바꾼다(현재 5 상자 하드코딩). 새 음성 두 종의 배출 경로를 만든다. `sdf_parts.add_urdf_visuals` 는 **재사용 불가**다(모델명 하드코딩, pose 인자 없음, collision 없음).
- [ ] `_require_keys` 가 **집합 동일성**이므로 새 기하 필드를 넣으려면 140 장면 전부와 그 집합에 키를 추가해야 한다.
- [ ] **`catalogue_v1` 시험 3 파일이 계속 통과해야 한다**(`test_build_scene_world.py`, `test_capture_scenes.py`, `test_scene_catalogue.py`).

**5b — 변환 도구와 새 음성**
- [ ] v1 자세·범주·분할·조명·표면·distractor 를 그대로 옮기고 형상 의존 항목만 재계산한다.
- [ ] 새 음성 `negative_block_row`·`negative_shelf` 를 **종별 20 장** 덧붙인다(분할 dev 14 / eval 6).
- [ ] 범주 등록: `evaluation.py` 의 `NEGATIVE_CATEGORIES`, `merge_scene_batches.py` 의 `CATEGORY_STATUS`, 월드 생성기 화이트리스트, `evaluate_pocket_detector.py` 의 카테고리 검사. **미등록이면 `ValueError` 로 죽는다.**
- [ ] 문서 갱신: `docs/interfaces/scene-dataset.md`, `docs/design/2026-09-11-pocket-observation-and-scene-set.md`, `docs/validation/2026-09-11-scene-catalogue-and-world.md`.
- [ ] `merge_batches` 는 **카탈로그 전수 커버리지**를 요구하므로 140 장 전체를 새 세트로 캡처한다.

### Task 6: 캡처·병합 (Claude + 원격)
- [ ] 140 장면 캡처, 병합, manifest 기록, 그룹·권한 확인.

### Task 7: dev 튜닝 (Claude)
- [ ] 기준선 1 회 후 한 번에 하나씩.
- [ ] 지표에 **지지대 최솟값**을 넣는다. 4 m 에서 `min()` 을 결정하는 것은 `upper`(562~692, 문턱의 5.6 배)가 아니라 지지대(120~172, 문턱의 1.2~1.7 배)다.
- [ ] **위양성 지표에 `invalid` 를 함께 적는다.** `false_positive_rate` 는 `valid` 만 세고 `evaluate_scene` 은 `invalid` 를 음성 분기보다 먼저 처리한다.
- [ ] 처리 시간을 EPAL 6 장면에서 잰다. v1 에서는 p50 +5.8 %·p95 +4.1 % 이고 **후보 수는 오히려 줄었다**(82→79). EPAL 6 에서는 늘어날 수 있다.

### Task 8: 동결 게이트 · Task 9: eval 1 회와 기록 (Claude)
- [ ] 고정 revision·clean 트리에서 eval 1 회. 목표 도달 여부를 있는 그대로.
- [ ] 검증 기록에 위양성률을 `invalid` 와 함께, 그리고 **선반이 통과한다는 한계**를 명시한다. 해결했다고 주장하지 않는다.

## 결정 지점

새 음성 종별 20 장, dev 14 / eval 6. dev 해상도 1/14 ≈ 7 % 이므로 **결정 지점은 `dev 위양성 2 장 이상`(≈14 %)** 이다. 선반은 이 집계에서 제외하고 따로 보고한다.

## 범위 밖 (의도)

터널 너머 신호(금지), 형판 정합·PnP, 딥러닝, 마커, 다중 시점 융합(M3), MR6D 실데이터 평가, `evaluation.py` 의 **목표 상수** 변경(범주 등록은 범위 안).
