# 포켓 관측 계약·장면 로더 검증 — 2026-09-11 (M1-b 1단계)

**범위:** [1단계 계획](../plans/2026-09-11-pocket-observation-contract.md) Task 5. 커밋 `2907a1b`(브랜치 `feat/pocket-observation-contract`, 기준 `main` `743c008`)의 코어 타입·로더·문서가 호스트와 원격에서 시험을 통과하는지 확인했다. 이는 계약과 파일 형식의 검증이며 팔레트 인식 성능·실물 센서·캡처 실행의 검증이 아니다.

## 구현 범위

- `src/forklift_core/perception/pocket_observation.py`: `Pocket`, `PocketObservation`(상태·좌우 내적 판정·비중첩·yaw 범위·σ·시각·열거값 검증, numpy 스칼라 정규화, JSON 왕복), `yaw_difference_rad`.
- `src/forklift_core/perception/scene_dataset.py`: `SceneInput`/`SceneSample` 분리, CameraInfo 좁은 계약, 16-bit mm depth 복원(`meters_per_unit` 정확히 0.001, unknown 0), PNG IHDR 비트 깊이 검사, tf.json·scene.json·정답 시각 교차 검증.
- `src/forklift_core/geometry.py`: `rotation_matrix_from_quaternion_xyzw`(허용오차 1e-6 안 정규화).
- `pyproject.toml`: `dataset` extra(Pillow ≥ 10), `dev`에 Pillow 포함.
- `docs/interfaces/pocket-observation.md`, `docs/interfaces/scene-dataset.md`, README 갱신. 설계 문서의 비중첩 조건 정정.

Codex가 구현 중 자체 리뷰로 찾은 결함 2건(고정 메타데이터의 미지 키 무시, Pillow가 16-bit RGB PNG를 uint8로 열어도 통과하던 문제)은 실패 시험 5개를 추가한 뒤 수정했다.

## 호스트 (conda base Python 3.11.7, Pillow 12.2.0)

| 검사 | 결과 |
|---|---|
| 기준 회귀(구현 전) | 146 passed, 1 deselected |
| Task 1 RED → GREEN | 신규 시험 11개 실패 → geometry 시험 31개 통과 |
| Task 2 RED → GREEN | `forklift_core.perception` 부재 실패 → 관측 계약 시험 106개 통과 |
| Task 3 RED → GREEN | `scene_dataset` import 실패 → 로더 시험 104개 통과 |
| 전체 회귀 `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` | **372 passed, 1 deselected** |
| 문서의 코어 명령 `--ignore=tests/simulation` | 320 passed |
| `ruff check .` / `ruff format --check .` | 통과 / 69 files |
| `/tmp`에서 `from forklift_core.perception import pocket_observation, scene_dataset` | `<checkout>/src/forklift_core/perception/scene_dataset.py` |
| Markdown 상대 링크 | 0개(설계 문서에서 대괄호 뒤에 괄호가 바로 이어져 링크로 오인되던 표기는 공백을 넣어 해소) |
| wheel `python -m build --wheel` METADATA | `Provides-Extra: dataset`, `Requires-Dist: Pillow>=10; extra == "dataset"`, `forklift_core/perception/*` 3개 파일 포함 |

계획의 수치 기대값(yaw 0.5 좌표, yaw 차이 5쌍, 역투영 (2.2, −1.0, 1.5), quaternion 회전행렬)은 구현 전 독립 계산으로 확인했고, Codex 계획 검토도 같은 값을 재계산했다.

## 원격 model-cpu (읽기 전용 snapshot → wheel → 실행별 venv)

run ID `20260911T081911Z_pocket_contract_model_cpu_01`, **Slurm 984 `COMPLETED 0:0`**, 실행 13초. snapshot SHA-256 `ff0c586e14e0cc36…`(제출 시점은 커밋 직전 작업 트리라 revision `b799838` dirty로 기록됐으며, 허용 목록 파일 내용은 커밋 `2907a1b`와 같다). 명령 5개 모두 exit 0, pytest **344 tests, failures 0, errors 0, skipped 0**(호스트 372에서 ROS 패키지 시험 28을 뺀 수와 일치). import 경로는 실행별 `.runtime/venv`, venv pip 24.0 · setuptools 79.0.1, wheel SHA-256 `ffe85bdfe956aefd…`. 결과 8개 파일 해시가 회수본과 일치. 결과: `artifacts/20260911T081911Z_pocket_contract_model_cpu_01/`.

원격 venv는 base 환경의 Pillow 12.2.0을 `--system-site-packages`로 보므로, 이 실행은 `dataset` extra의 설치 동작을 검증하지 않는다(extra 선언은 wheel METADATA로 확인).

## 경계

- `PocketObservation`은 M2 평가·M3 입력용이며 M5 삽입 여유 계산에는 불충분하다(문서에 명시).
- 로더는 v1 파일 형식만 받는다. 실제 캡처 파일은 3단계에서 생성되며, 그때 로더가 실제 Gazebo CameraInfo·TF를 읽는지 다시 확인한다.
- 시험 fixture는 8×6 픽셀 합성 값이며 D435i 특성이나 실물 정합을 나타내지 않는다.
