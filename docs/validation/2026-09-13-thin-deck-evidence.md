# 얇은 아래 덱 증거 v4 구현 검증 (2026-09-13)

상태: **합격. v1 최종 관측 100/100 일치, 전체 753 passed / 1 deselected.**
기준은 `main`, HEAD `c14a038`이며 결과는 미커밋 작업 트리에 대한 것이다.
커밋·스테이지는 하지 않았다. 이전 실행의 구현과 시험이 작업 트리에 남아 있어
이를 보존하고, 사용자가 정정한 거리 조건으로 시험을 고쳐 재검증했다.
이번 재개에서는 남아 있던 제품 코드에 추가 변경이 필요하지 않았다.

증거는 합성 최초 충돌 광선과 저장 데이터 재생이다. 새 Gazebo 캡처,
실물 D435i 또는 로봇 주행 검증은 아니다.

## 1. 만든 파일과 고친 파일

HEAD 대비 최종 변경 목록이다. 이전 실행에서 남아 있던 변경도 포함한다.

| 구분 | 파일 | 내용 |
|---|---|---|
| 신규 | `tests/unit/perception/test_thin_deck_evidence.py` | 거리·전면 앞 띠·후면 상자·대역 경계·바닥·분리 받침·파라미터 시험 |
| 신규 | `tests/integration/test_detector_v1_replay.py` | 저장된 dev 70개와 eval 30개 관측의 전체 필드 비교 |
| 신규 | `docs/validation/2026-09-13-thin-deck-evidence.md` | 이 검증 기록 |
| 수정 | `src/forklift_core/perception/pocket_detector.py` | 작업 공간에서 아래 덱 증거를 집계, `deck_evidence_tol_m=0.006` 추가·검증 |
| 수정 | `src/forklift_core/perception/pallet_prior.py` | 필수 양수 `overall_depth_m` 추가 |
| 수정 | `src/forklift_core/perception/pallet_geometry.py` | 형상 깊이를 prior에 전달 |
| 수정 | `tools/build_pallet_prior.py` | 형상에서 얻은 깊이를 YAML에 기록 |
| 수정 | `config/pallet_prior_v1.yaml` | 깊이 0.6 m 명시 |
| 수정 | `config/pallet_prior_epal6.yaml` | 생성 prior에도 깊이 0.6 m 반영 |
| 수정 | `tests/unit/perception/test_pallet_prior.py` | 깊이 누락·잘못된 값 거부, 생성·변환 시험 |
| 수정 | `docs/design/2026-09-13-thin-deck-evidence.md` | 사용자 정정대로 반례 시험 2와 남아 있던 0점 문제 진술 수정 |

제품 코드의 변경 범위는 아래 덱 증거 위치와 필요한 설정 전달이다.
연속 횡범위, 위 덱 집계, 지지대 집계, 최소 점 수 문턱, 점수식,
광선 분류, 상태 판정 순서를 그대로 유지한다. 깊이는 카메라 쪽을 향하는
전면 법선에 대해 `-(workspace - plane.point) @ plane.normal`으로 계산하며,
`0 <= depth <= overall_depth_m + plane_inlier_m`를 적용한다.
바닥 필터를 통과한 작업 공간에서만 `abs(z - deck_bottom_m) <= 0.006`을 검사한다.
0.006 m는 dev 튜닝 출발값이며 평면 잔차의 유도값이 아니다.
EPAL 6에서 기존 바닥 필터와 겹친 실효 하향 여유는 2 mm 미만이다.

## 2. 실패 확인 → 구현 검증

이미 있던 제품 코드를 되돌려 덮어쓰지 않도록 `c14a038`의 원래 모듈을
별도로 로드했다. `_opening_candidates`에는 새 workspace 인자를 무시하고
원래 3개 인자 함수에 전달하는 호출 어댑터만 두었다. 알고리즘 본문은
HEAD 그대로다. 필수 깊이 누락 시험도 HEAD의 원래 prior 로더를 사용했다.
프로세스가 끝나면 이 연결은 사라지며 작업 트리 제품 파일은 바뀌지 않는다.

수정된 시험으로 재확인한 RED는 **4 failed, 1 passed, 43 deselected**다.

| 실패 시험 | 실제 메시지 |
|---|---|
| 3.7 m 아래 덱 증거 | `assert 0 > 100` |
| 4.0 m 아래 덱 증거 | `assert 0 > 100` |
| 분리된 받침의 알려진 한계 | `assert 'no_pallet' == 'valid'`, 사유 `no_opening_pattern` |
| `overall_depth_m` 누락 | `Failed: DID NOT RAISE ValueError` |

2.0 m 대조군은 HEAD에서도 통과했다. 기존 구현을 사용하는 정상 프로세스에서
동일 시험과 prior·v1 재생 시험을 실행한 GREEN은 **50 passed (25.04초)**다.
이전 실행의 구현 전 RED 기록은
`artifacts/20260913T110645Z_thin_deck_v4_validation/red.log`에 보존돼 있다.

앞쪽 허용량을 메모리에서만 `depth >= -plane_inlier_m`로 바꾼 별도 변이 시험은
`assert 252 == 0`으로 실패했다. 앞쪽 띠 시험이 해당 경계 오류를 실제로 잡는다.
변이는 파일에 적용하지 않았다.

## 3. v1 100장면 재현

| 저장 실행 | 관측 수 | 정확히 일치 | 불일치 |
|---|---:|---:|---:|
| `20260912T170442Z_pocket_eval_dev_02` | 70 | 70 | 0 |
| `20260912T170558Z_pocket_eval_eval_01` | 30 | 30 | 0 |
| 합계 | **100** | **100** | **0** |

`config/pallet_prior_v1.yaml` + 변경하지 않은 `config/detector_params_v1.yaml`을
사용했다. 지정 파라미터 YAML과 저장 `run.json`의 값도 일치한다.
저장 `observations/*.json`에서 **`diagnostics`만 제외**하고 모든 최종 관측 필드를
정확히 비교했다. 관측 개수와 scene ID 집합도 확인했다. 저장 기대값은 수정하지 않았다.
신규 tolerance는 기본값 0.006을 사용한다.

두 저장 실행을 이 머신에서 실제로 재생했으며 skip하지 않았다. 다른 호스트에서
실행 디렉터리 자체가 없으면 해당 재생 시험은 skip 사유를 표시한다.
실행 디렉터리는 있는데 관측이나 입력 데이터가 불완전하면 실패한다.

## 4. 거리별 아래 덱 증거량

| 팔레트 중심 x | 수정 전 | 수정 후 | 판정 |
|---|---:|---:|---|
| 2.0 m | 1,085 | 8,874 | 전후 모두 100 이상 |
| 3.7 m | 0 | 1,102 | 전 100 미만, 후 100 초과 |
| 4.0 m | 0 | 960 | 전 100 미만, 후 100 초과 |

이 값은 사용자 표의 이상적 정면 계산 1,647 / 98 / 88과 동일한 수치를
주장하지 않는다. 시험은 정정된 **문턱 조건**을 고정하며 수정 전 0점을 요구하지 않는다.

재현 입력은 EPAL 6 형상 YAML의 두 덱·9개 블록, 카메라 (0.75, 0, 0.5) m,
640×480, fx=fy=465.741156, cx=320, cy=240이다.
`tests/fixtures/synthetic_scene.py`의 독립 박스 교차로 최초 충돌을 구하고
깊이를 **1 mm로 반올림**한 뒤 실제 검출기의 적합 전면과 격자 범위를 사용한다.
3.7·4.0 m에서 전면 inlier에 남는 덱 부근의 가장 낮은 z는
**0.02205211600410939 m**다. 22 mm보다 약 0.052 mm 높아
기존 `z <= 0.022`에서 빠진다. 이는 이번 양자화 입력의 결과이며,
3·4 m에서 구조적으로 항상 0이라는 주장을 되살리는 근거가 아니다.

수정 전 수는 같은 전면과 횡범위의 기존 집계식으로 읽고,
수정 후 수는 실제 `_Pattern.deck_count`에서 변경하지 않은 위 덱 수를 뺀다.

⚠️ **개정 (2026-09-14): 이 역산은 더 이상 성립하지 않고, 성립했던 것도 우연이었다.** `deck_count`가 `lower + upper`였을 때는 위 덱 수를 빼면 정확히 `lower`가 나왔지만, `deck_count`가 **`lower + upper_left + upper_right`**(개구별)로 바뀌면서 두 항이 상쇄하지 않는다 — 하니스가 **음수 `lower`(−231)** 를 보고했다. **`_Pattern.lower`를 직접 읽는다.** 같은 값이 `DetectionDiagnostics.selected_lower`로도 나온다.
위 RED에서 HEAD 원래 함수로도 수정 전 수가 동일함을 확인했다.
0점 측정 시에만 비공개 후보 함수의 count gate를 0으로 내려 관찰하고,
공개 파라미터와 최종 검출의 `min_band_points=100`은 유지한다.

## 5. 반례·가정·계획과 달라진 시험 해석

- **앞쪽 띠 0점은 띠 자체의 기여를 뜻한다고 해석했다.** 기둥 3개와 위 판에
  x=2.6805–2.6995 m, y 중심 ±0.18 m, 폭 260 mm, 높이 24 mm 띠를 놓았다.
  관측된 띠 252점의 아래 증거는 **0**이다. 같은 점을 전면 뒤 10 mm로 옮기면
  **252점**이 인정된다. 시험에서는 출처를 정확히 분리하기 위해 양자화 없는
  깊이를 쓰고, 별도 재현 스크립트로 1 mm 양자화에서도 띠 0점을 확인했다.
- **전체 장면 증거는 0점이라고 요구하지 않는다.** 그대로 유지해야 하는
  연속 횡범위가 중앙 기둥도 포함한다. 전체 아래 증거는 무양자화 62점,
  1 mm 양자화 38점이다. 둘 다 최종 `no_pallet/no_opening_pattern`이다.
  이전 시험의 전체 0점 기대를 띠의 기여 0점으로 바로잡았으며,
  제품 코드에서 중앙 기둥을 빼거나 개구별 집계로 바꾸지 않았다.
  전면 기준 -19.5 / -10.0 / -0.5 mm의 점군 경계 시험도 모두 0점이다.
- **후면 뒤 0.8 m 상자:** 팔레트 후면 x=3.3 m, 상자 전면 x=4.1 m로 해석했다.
  높이 24 mm 상자의 윗면 800점을 명시적으로 공급해도 아래 증거는 0점이다.
  위 덱에 가려 입력 자체가 없어지는 시험을 피하려고 점군 수준에서 검사한다.
- **바닥과 높이·깊이 경계:** z=0 및 z=20 mm 바닥, 높이 허용 대역과
  후면 깊이 상한의 양쪽을 검사했다. 바닥 필터는 수정하지 않았다.
- **필수 설정:** 깊이 누락, 0, 음수, NaN을 거부한다. 형상→prior와 생성 YAML도 검사했다.
- **알려진 연결성 한계:** 기둥 3개 + 위 판 + 중심 (2.89, ±0.18) m의
  160×200×24 mm 받침 2개는 기둥과 40 mm 떨어져도 **`valid`**다.
  연결성 검사를 구현하면 이 시험을 뒤집고 기대를 바꾸라는 주석을 유지했다.
- **이번 재개의 절차 차이:** 기존 구현이 이미 있으므로 RED는 HEAD 함수를
  메모리에 별도로 로드해 확인했다. 기존 파일은 보존하고 검증을 이어갔다.
  알고리즘 설계 v4와 다른 구현, 문턱 조정, 관측 기대값 변경은 없다.
  추가 독립 에이전트 호출은 하지 않았으며 앞서 제공된 교차검토와 직접 실행으로 확인했다.

## 6. 최종 회귀와 Ruff

사용 인터프리터: `/home/light/anaconda3/bin/python`.
`forklift-core`는 이 저장소를 가리키는 editable 설치임을 확인했다.

```bash
python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error
python -m ruff check .
python -m ruff format --check .
```

- 전체 회귀: **753 passed, 1 deselected (84.95초)**. 기존 724개를 보존하고
  추가 29개가 통과했다. 제외 1개는 요청한 `rendering` 마커다.
- Ruff lint: **All checks passed!**
- Ruff format check: **118 files already formatted**.
- `git diff --check`: 통과. 스테이징된 변경 없음, HEAD `c14a038` 유지.

이번 재검증의 로그, 실행 요약·설정·소스 해시, 시작 시점 작업 트리 사본은
`artifacts/20260913T113911Z_thin_deck_v4_completion/`에 보관했다.
재현 스크립트는 그 안의 `run_head_red.py`와 `measure_evidence.py`이고,
핵심 로그는 `red.log`, `targeted.log`, `regression.log`, `evidence.json`,
`front_guard_mutation.log`, `ruff_check.log`, `ruff_format.log`다.
