# 포켓 증거 구조 변경과 M2 재평가 계획 (v4)

> 상태: **v4 — 독립 검토 9회 반영.** v1 `22e967b`, v2 `1644063`, v3 `f633b7a`.
> **이 문서가 정본이다.** `docs/plans/2026-09-13-epal6-scene-set-and-m2-rerun.md`(v2)는 **폐기**한다. 그 문서의 11 상자·100 장면 전제는 이 계획의 22 상자·140 장면과 충돌한다.
> **실행자:** Task 2–6 위임, 나머지 Claude.

**목표:** 실물 EPAL 6 형상에서 포켓을 검출하지 못하는 현재 상태를 풀고, EPAL 6 장면 세트로 M2 를 다시 평가한다.

## 핵심 변경 하나

판정을 **`min(지지대 3 개, u1, u2) >= min_band_points`** 로 바꾼다. `u1`·`u2` 는 **각 개구 바로 위** 구간의 상한 증거다(중앙 지지대 위는 세지 않는다). 아래 덱 증거는 관문에서 뺀다.

**현행 `upper` 는 개구 위를 보지 않는다.** 횡방향 창이 두 개구와 중앙 지지대를 통째로 훑어서, 바닥에 놓인 상자 세 개가 **가운데 상자 자신으로** 상한 증거를 전부 채운다(3 m 에서 개구 위 0 + 0, 중앙 위 1056).

### 측정 (검토 2회가 독립 재현)

| 규칙 | v1 재현 | EPAL 6 | 상자 세 개 | 선반 |
|---|---|---|---|---|
| 현행 | 100/100 | 5/40 | 1/40 | **0/40** |
| `upper` 전 구간 + lower 관문 제거 | 99/100 | 34/40 | 33/40 | 35/40 |
| **채택: `min(u1, u2)`** | **91/100** | **34/40** | **0/40** | **35/40** |
| `u1 + u2` | 100/100 | 34/40 | 0/40 | 35/40 |

### 왜 합계가 아니라 `min` 인가

**합계는 실재하는 버그를 남긴다.** 팔레트를 화각 가장자리에 두고 **화각 중심 쪽**에 같은 평면 상자를 놓으면, 먼 쪽 바깥 블록이 잘려 진짜 패턴이 지지대 56 점으로 탈락하고 [이웃 상자 | 공백 | 팔레트 바깥 블록 | 진짜 포켓] 유령 쌍이 성립한다. 유령 개구 위는 비어 있어 `u1 = 0` 인데 진짜 개구가 `u2 = 1021` 을 대서 합으로 통과한다. 결과는 좌 368 mm·우 321 mm **오위치**다.

**이것은 가설이 아니다.** `y = 0.80` 에서는 **현행 HEAD 규칙도 이 유령을 `valid` 로 낸다**(368 mm 오차). 지금 코드에 있는 버그이고 `min` 이 제거하며 합계는 제거하지 못한다.

**`min` 의 91/100 은 라벨 변경이다.** 9 장면 전부 `occluded` 이고 `invalid/pocket_occluded:*` → `no_pallet/no_opening_pattern` 이다. 좌표·yaw·σ 는 9 장면 모두 원래 `null` 이었고 그대로다.

## 합격 조건 (검토가 검증한 정확한 문안)

> v1 에서 바뀌는 것은 9 개 `occluded` 장면(`s007 s027 s039 s051 s060 s063 s076 s090 s100`)의 `status`·`reason` 뿐이다(`invalid/pocket_occluded:*` → `no_pallet/no_opening_pattern`). 나머지 91 장면은 `diagnostics` 를 제외한 전 필드가 동일하고, 포켓 좌표·yaw·σ 는 **100 장면 전부 불변**이다. `summarize` 의 `detection_rate`·`false_positive_rate`·`position_error_m`·`yaw_error_rad`·`targets`·`scene_count`·`splits` 는 불변이다. **`counts` 는 설계대로 움직인다**(`occluded.invalid` 15→6, `occluded.false_negative` 0→9). `elapsed_s` 는 벽시계 측정이라 잡음이 델타보다 크므로 **합격 조건에서 제외한다.**

v3 의 *"어떤 집계 지표도 움직이지 않는다"* 는 **거짓**이었다.

## 정직하게 적어야 할 두 가지

1. **선반 위양성은 이 변경이 새로 들여온다.** 현행 0/21 → 변경 후 15/21 이다. v3 의 *"규칙 변경으로 줄지 않는다"* 는 이미 있던 것처럼 읽히게 쓴 것이고 틀렸다. 검증 기록에 **새로 생긴다**고 적는다.
2. **`min` 과 합계의 차이는 EPAL 6 가림 장면에서 46.4 %(701/1512)에 걸린다.** v1 의 9 장면 수준이 아니다. 차이는 전부 같은 성격이고(`no_pallet` 대 `invalid`), `valid` 판정과 좌표는 양쪽이 비트 단위로 같다. 그러나 EPAL 6 재평가 표에서 두 규칙이 겉보기에 크게 달라 보이므로 Task 7 의 `invalid` 병기가 필수다.

## Task 4(포켓 깊이 관문)를 뺀다 — 측정으로 기각

v3 는 광선 "뒤" 기준을 0.05 → 0.36 m 로 올려 막힌 포켓을 거부하자고 했다. **기하학적으로 불가능하다.**

광선은 터널을 관통하지 않고 **내려가다 터널 바닥에 닿는다.** 읽을 수 있는 최대 깊이는 닫힌 식으로 고정된다.

```
p_max = (개구 대역 상단 z − 터널 바닥 z) × R_perp / (카메라 높이 − 개구 대역 상단 z)
```

EPAL 6 은 터널 바닥이 실제 바닥(z = 0)이고 개구가 78 mm 라 계수가 **0.2195** 다. 검출기 자신의 `range_min_m` 0.8 m 에서 최대 176 mm 이고, 광선의 30 % 가 360 mm 를 넘으려면 팔레트가 3.17 m 보다 멀어야 한다.

실측 결과다.

| 문턱 | v1 검출률 | EPAL 6 75 자세 |
|---|---|---|
| 0.05(현행) | 0.800 | 65/75 |
| 0.10 | 0.800 | 64/75 |
| 0.20 | 0.650 | 47/75 |
| **0.36** | **0.362** | **9/75**(x ≤ 3.0 m 45 자세 전부 미검출) |

그리고 **이 데이터 세트에서 이득이 0 이다.** v1 위양성은 이미 0 이고 어떤 문턱에서도 안 움직인다. 새로 넣을 음성 두 종도 뒤가 뚫려 있어 못 잡는다. 작동을 보여 줄 장면이 하나도 없다.

권장 기본값은 0.09 m(v1 최솟값 0.1093, EPAL 6 최솟값 0.0957)이고 현행 대비 실질 이득이 **40 mm** 다. 그 값으로 잡히는 것은 **90 mm 보다 얕은 막힘뿐**이다. `min_pocket_depth_m` 이라는 이름과 0.36 이라는 값은 **360 mm 를 보장한다는 거짓 확신**을 만든다. 없는 것보다 나쁘다.

**따라서 이 계획에서 뺀다.** 위 닫힌 식을 `docs/design/2026-09-13-thin-deck-evidence.md` 에 남겨 다음 사람이 또 0.36 을 넣지 않게 한다(Task 3 에서). 막힌 포켓 문제는 실재하지만 이 계측기로는 못 잰다 — 조사가 보여 준 대로 삽입 중에는 포크 장착 센서가 업계 방식이다.

## 전역 제약

- **터널 너머에 의존하는 신호를 만들지 않는다.** 시선각 `arctan(227.5/600)` = **20.8°** 를 넘으면 관통이 불가능하다. 설계 결정으로 고정한다. v3 의 Task 4 가 이 제약을 어겼다.
- **v1 카탈로그·데이터 세트·prior 는 읽기만 한다.** 새 음성은 `sample_catalogue` 재생성이 아니라 변환 도구로 덧붙인다. 실측: `COUNTS` 에 범주를 더하면 `rng.shuffle` 이 밀려 **기존 100 장 중 0 장만 동일**하다.
- dev 에서만 튜닝, **eval 은 한 번**.
- `ruff check .` 와 `ruff format --check .` 통과.
- **기준 회귀:** `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` → **758 passed, 1 failed, 1 deselected**. 그 1 failed 는 `test_transport_timeout_kills_orphan_*` 로 **부하 시 간헐 실패하는 기존 flake** 다. 이번 변경 탓으로 오인하지 않는다. 샌드박스에 따라 `test_prepare_core_environment_installs_snapshot_wheel_*` 도 실패할 수 있다(네트워크).

---

### Task 0: 기준 확인 (Claude)
- [ ] 트리 깨끗. 위 기준 회귀 확인.
- [ ] 아래 덱 증거가 **평면 배치에 조건수가 나쁘다**는 사실을 기록한다(0.5 mm 이동에 852 ↔ 168). 앞서 나온 서로 다른 표들은 전부 리그 산물이었고 프로덕션 경로 값은 2.0/3.0/4.0 m 에서 0/12/9 다.

### Task 1: 원격 경로 수리 (위임)

**Files:** `tools/submit_model_check.py`, `tests/integration/test_evaluate_cli.py`, `tools/remote_model_job.py`

- [ ] snapshot 허용 목록에 `config` 와 `tests` 의 `.yaml` 을 추가한다. 실측: 정확히 6 개 파일이 새로 포함되고 비밀 파일은 없다. 현재는 `tests/fixtures/thin_deck_legacy_*.yaml` 이 안 가서 **원격 pytest 가 수집 단계에서 죽는다.**
- [ ] **`git_revision` 문제를 같이 고친다.** `test_run_metadata_...` 가 길이 40 을 요구하는데 snapshot 에 `.git` 이 없어 `None` → `TypeError`. 이 assert 는 `3465c8d` 이후 **원격에서 한 번도 안 돌았다.**
- [ ] **`scenes` mode 의 코어 패키지 문제를 결정한다.** `remote_model_job.py` 의 `scenes` 분기는 `pip install -e .` 없이 `capture_scenes.py` 를 바로 실행한다. 월드 생성기가 `forklift_core` 를 import 하면 원격 캡처가 죽는다. **설치를 추가할지, import 없이 동작하게 할지 택하고 보고한다.**
- [ ] **완료 조건: 원격 `model-cpu` job 을 한 번 돌려 green 을 확인한다.** 안 하면 Task 6 까지 검증이 미뤄진다.
- [ ] v1 재현 시험 2 개는 원격에서 skip 된다(`artifacts/` 제외). 기록만 한다.

### Task 2: 관문 변경과 명시 진단 (위임)

**Files:** `src/forklift_core/perception/pocket_detector.py`, `tools/evaluate_pocket_detector.py`, `tests/integration/test_detector_v1_replay.py`, `tests/unit/perception/test_thin_deck_evidence.py`, `tests/unit/perception/test_pocket_detector.py`, `docs/design/2026-09-13-pocket-detector-baseline.md`, `docs/design/2026-09-13-thin-deck-evidence.md`, `docs/validation/2026-09-13-thin-deck-evidence.md`

- [ ] 판정을 `min(지지대 3 개, u1, u2) >= min_band_points` 로 바꾸고 아래 덱 증거를 관문에서 뺀다.
- [ ] `_Pattern` 과 `DetectionDiagnostics` 에 **`lower`, `upper_left`, `upper_right`, `support_min`** 을 **스칼라**로 추가한다. 튜플로 두면 CSV 에 파이썬 repr 이 들어간다. `_Pattern.support_count` 는 **합계**라 최솟값 지표로 쓸 수 없다.
- [ ] **새 필드는 `DetectionDiagnostics` 의 `exception_traceback` 앞에 넣는다.** 뒤에 붙이면 기본값 순서 때문에 `TypeError` 로 import 가 깨진다. 두 dataclass 모두 위치 인자로 생성되므로 생성 지점을 같은 커밋에서 고치고, 패턴 미선택 시 `None`.
- [ ] 현재 시험 하니스는 `deck_count − upper` 로 `lower` 를 역산한다. **시험 3 개가 실패하고 다른 3 개는 통과하면서 틀린 값을 잰다**(2.0 m 에서 7860 대 실제 8874). 하니스를 명시 필드 직독으로 바꾼다.
- [ ] **`test_detector_v1_replay.py` 는 반드시 깨진다.** `assert not differences` 가 0 건을 요구한다. 9 장면 기대값을 같은 커밋에서 갱신한다. 실측: 패치 후 dev 64/70·eval 27/30 으로 두 파라미터화 모두 FAILED.
- [ ] `SCENE_COLUMNS` 에 새 진단 이름을 올린다. 안 올리면 **조용히 버려진다.**
- [ ] **bool 파라미터를 넣지 않는다.** `_finite_scalar` 가 bool 을 거부해 모듈 import 가 깨진다.
- [ ] 무효가 되는 계약을 같은 커밋에서 갱신한다: `baseline.md` 의 "위·아래 덱 대역에 점유가 있어야", `thin-deck-evidence.md` 의 `min(지지대 3개, lower, upper)`, `validation/thin-deck-evidence.md` 의 "deck_count 에서 upper 를 뺀다".
- [ ] **포켓 깊이 닫힌 식**(위 §)을 `thin-deck-evidence.md` 에 남긴다.
- [ ] **유령 포켓 반례 시험.** 팔레트 x 2.5 / y +0.70, 같은 평면 상자를 **화각 중심 쪽(−y)** 바깥 블록에서 0.22 m. **`+y` 쪽에 놓으면 상자가 화면 밖으로 투영되어 유령 패턴이 생기지 않고 시험이 조용히 통과한다.** 합계에서는 `valid`(368 mm 오위치), `min` 에서는 `no_pallet` 이어야 한다.
- [ ] 상자 세 개 거부(63/63), EPAL 6 통과, **선반 통과는 알려진 한계로 고정**(주석에 "연결성 검사를 넣으면 뒤집히는 것이 정상").

### Task 3: 월드 생성기가 두 기하를 받게 한다 (위임)

**Files:** `sim/gazebo/build_scene_world.py`, `sim/gazebo/sdf_parts.py`, `tests/simulation/test_build_scene_world.py`

- [ ] **치수 출처를 카탈로그 헤더 `pallet` 블록으로 한다.** `forklift_core` 를 import 하지 않는다(원격에 코어 설치가 없다). 형상 YAML 도 직접 읽지 않는다.
- [ ] `load_catalogue` 의 `catalogue_version == "v1"` 고정과 `pallet` 블록 **완전일치** 검사를 두 기하 모두 받도록 바꾼다.
- [ ] 팔레트 배출을 **22 상자**로 바꾼다(현재 5 상자 하드코딩). `sdf_parts.add_urdf_visuals` 는 **재사용 불가**다(모델명 하드코딩, pose 인자 없음, collision 없음).
- [ ] 새 음성 두 종의 배출 경로를 만든다. `_require_keys` 가 **집합 동일성**이므로 새 기하 필드를 넣으려면 140 장면 전부와 그 집합에 키를 추가해야 한다.
- [ ] **시험: 정답 포켓 중심을 지나는 광선이 팔레트 상자 어느 것과도 교차하지 않는다**(개구가 실제로 비어 있음). 상자 개수·크기만 보는 시험은 9 개를 같은 x 에 놓아도 통과한다.
- [ ] **`catalogue_v1` 시험 3 파일이 계속 통과해야 한다.**

### Task 4: 카탈로그 변환과 새 음성 (위임)

**Files:** Create `tools/retarget_scene_catalogue.py`, `sim/gazebo/scenes/catalogue_epal6.yaml`; Modify `src/forklift_core/perception/evaluation.py`, `tools/merge_scene_batches.py`, `tools/evaluate_pocket_detector.py`, `docs/interfaces/scene-dataset.md`, `docs/design/2026-09-11-pocket-observation-and-scene-set.md`, `docs/validation/2026-09-11-scene-catalogue-and-world.md`

- [ ] v1 자세·범주·분할·조명·표면·distractor 를 그대로 옮기고 형상 의존 항목만 재계산한다. **자세 100 장 완전 일치를 시험으로 고정한다**(approx 아님).
- [ ] **occluder 의 `fraction` 은 물리 폭 비율이지 영상 가림률이 아니다.** 변환 도구가 **실제 영상 가림률을 계산해 기록**한다. 같은 fraction 이 같은 난이도를 뜻하지 않는다.
- [ ] 새 음성 `negative_block_row`·`negative_shelf` 를 **종별 20 장**(scene ID `s101`–`s140`) 덧붙인다. 분할 dev 14 / eval 6.
- [ ] 데이터 세트 디렉터리는 `data/synthetic_scenes/catalogue_epal6/`. **`catalogue_v1` 을 덮지 않는다** — `test_detector_v1_replay.py` 의 폴백이 깨진다.
- [ ] 범주 등록: `evaluation.py` 의 `NEGATIVE_CATEGORIES`, `merge_scene_batches.py` 의 `CATEGORY_STATUS`, 월드 생성기 화이트리스트, `evaluate_pocket_detector.py` 의 카테고리 검사. **미등록이면 `ValueError` 로 죽는다.**

### Task 5: 카탈로그 동결 게이트 (Claude)
- [ ] **캡처는 되돌릴 수 없다.** `merge_batches` 는 배치 manifest 의 `catalogue_sha256` 완전일치와 140 장 전수 커버리지를 요구한다. 캡처 후 카탈로그를 1 바이트만 고쳐도 4 배치 전부 폐기다.
- [ ] 카탈로그·월드 생성기·prior 를 커밋하고 그 revision 과 카탈로그 해시를 기록한 뒤 캡처한다.

### Task 6: 캡처·병합 (Claude + 원격)
- [ ] 140 장면 4 배치 캡처, 병합, set manifest 기록, 그룹·권한 확인.

### Task 7: 동결 파라미터 파일 정리 (Claude)
- [ ] **여기서 한다. Task 0 이 아니다.** Task 2 가 파라미터를 추가하므로 앞에서 하면 두 번 만든다.
- [ ] `config/detector_params_v1.yaml` 은 키 16 개인데 `DetectorParams` 는 17 개다(`deck_evidence_tol_m` 누락). `_load_params` 가 누락 키를 조용히 코드 기본값으로 채운다.
- [ ] **`test_detector_v1_replay.py:31` 의 `params_data == run["params"]` 가 깨진다.** 과거 `run.json` 도 16 키다. 동결 파일을 고치면 "두 실행이 실제로 쓴 집합" 이 아니게 된다. **파일을 고치지 말고 시험이 "파일 ⊆ run.params 이고 나머지는 코드 기본값과 같다" 를 확인하도록 바꾸는 편이 정직하다.** 어느 쪽을 택하든 근거를 기록한다.
- [ ] EPAL 6 튜닝 결과는 `config/detector_params_epal6.yaml` 로 저장한다.

### Task 8: dev 튜닝 (Claude)
- [ ] 기준선 1 회 후 한 번에 하나씩.
- [ ] 지표에 **지지대 최솟값**을 넣는다. 4 m 에서 `min()` 을 결정하는 것은 `upper`(562~692, 문턱의 5.6 배)가 아니라 지지대(120~172, 문턱의 1.2~1.7 배)다.
- [ ] **위양성 지표에 `invalid` 를 함께 적는다.** `false_positive_rate` 는 `valid` 만 세고 `evaluate_scene` 은 `invalid` 를 음성 분기보다 먼저 처리한다. EPAL 6 에서는 `min` 이 가림 장면 46 %를 `invalid` 에서 `no_pallet` 으로 옮기므로 병기가 필수다.
- [ ] 처리 시간을 EPAL 6 에서 잰다. v1 에서는 장면별 best-of-3 로 **오히려 빠르다**(p50 −5.7 %). 후보 수도 줄었다(82→79).
- [ ] **`floor_z_m >= deck_bottom_m` 을 설정 오류로 거부**한다. EPAL 6 은 0.020 대 0.022 로 2 mm 차다.

### Task 9: 동결 게이트 · Task 10: eval 1 회와 기록 (Claude)
- [ ] 고정 revision·clean 트리에서 eval 1 회. 목표 도달 여부를 있는 그대로.
- [ ] 검증 기록에 적을 것: 위양성률과 `invalid` 병기, **선반 위양성이 이 변경으로 새로 생긴다는 사실**, `POSITION_TOLERANCE_M` 0.20 m 가 포크 좌우 여유 45 mm 와 무관하다는 점(`true_positive` 딱지가 삽입 가능성을 뜻하지 않는다), 그리고 해결하지 않은 것들.

## 결정 지점

새 음성 종별 20 장, dev 14 / eval 6. dev 해상도 1/14 ≈ 7 % 이므로 **결정 지점은 `dev 위양성 2 장 이상`(≈14 %)** 이다. 선반은 이 집계에서 제외하고 따로 보고한다(통과가 예상되고 규칙이 아니라 기하의 문제).

## 범위 밖 (의도)

터널 너머 신호(금지), 포켓 깊이 관문(측정으로 기각), 형판 정합·PnP, 딥러닝, 마커, 다중 시점 융합(M3), MR6D 실데이터 평가, `evaluation.py` 의 **목표 상수** 변경(범주 등록은 범위 안), `s009` 진단 분리(별건).
