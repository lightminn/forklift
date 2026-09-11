# 장면 카탈로그·장면 월드 생성 검증 — 2026-09-11 (M1-b 2단계)

**범위:** [2단계 계획](../plans/2026-09-11-scene-catalogue-and-world.md) Task 5. 브랜치 `feat/scene-catalogue`의 기능 커밋이 카탈로그 100개를 고정하고, 장면 월드 생성기와 공용 SDF 헬퍼가 기존 reference experiment를 깨지 않는지 호스트에서 확인했다. Gazebo 렌더링·캡처·원격 실행은 3단계 범위이며 여기서 검증하지 않았다.

## 구현 범위와 작업 방식

- `tools/generate_scene_catalogue.py`(결정론 생성기, `forklift_core` 정답 타입 사용), `sim/gazebo/scenes/catalogue_v1.yaml`(고정 산출물), `sim/gazebo/sdf_parts.py`(공용 헬퍼), `sim/gazebo/build_sensor_world.py`(헬퍼 사용으로 리팩터링), `sim/gazebo/build_scene_world.py`(장면 월드 생성기, 코어 미의존), 시험 3파일, `sim/gazebo/README.md` 절.
- Codex 위임 세션이 노트북 메모리 부족으로 두 번 강제 종료됐다(사용자 게임 실행 중 여유 < 1 GB). 클라이언트 종료 후에도 app-server 쪽 작업이 이어져 Task 1·2와 Task 3 대부분을 남겼고, 중복 실행된 job은 취소했다. Claude가 이어서 distractor 프리셋을 카탈로그 헤더(`distractor_presets`)로 옮겨 빌더가 그 정본을 읽도록 조정하고 README 절을 썼다. 이 부분은 Codex 교차검증 없이 Claude 판단으로 진행했다(계획 자체는 Codex 2차 검토를 거침).

## 카탈로그 v1

| 항목 | 값 |
|---|---|
| 파일 | `sim/gazebo/scenes/catalogue_v1.yaml`, 121,599 bytes, seed 20260911 |
| 결정론 | 같은 seed로 두 번 생성해 바이트 동일 |
| 범주 | positive 60 · occluded 20 · negative_no_pallet 10 · negative_lookalike 10 |
| split | dev 70 / eval 30 (positive 42/18, occluded 14/6, 음성 각 7/3) |
| 가림 | `occluded_fraction_image` 0.396–1.0(물리 폭 비율 0.2–0.6 이상) |
| 헤더 | camera(640×480·1.204 rad·5 Hz·(0.75, 0, 0.5)), pallet 상수, ranges, distractor_presets 6종 |

`tests/simulation/test_scene_catalogue.py` 14개: yaw 0/+0.5/−0.3의 정답 좌표를 독립 계산값과 대조, 투영 기대값 (164.8, 299.7, 1.95), 시야 밖(x=1.2) 거부, 구성·범위·split, 가림이 정답을 바꾸지 않음, 재생성 동일성, 정답 100개가 `PocketObservation`으로 로드됨.

## 장면 월드 생성기와 회귀

- `tests/simulation/test_gazebo_sensor_world.py`: 기존 `build_sensor_world.py` 출력 3개의 SHA-256(`75dda4e7…`, `3b83e534…`, `018b2669…`)이 리팩터링 전후 동일. 직접 실행(`python sim/gazebo/build_sensor_world.py`)도 같은 해시.
- `tests/simulation/test_build_scene_world.py` 8개: 회전 팔레트의 지지대 5개와 폭, 지지대 위치에서 개구 중심을 재구성해 정답과 1e-6 이내 일치, 가림 상자 pose, 음성 장면의 팔레트 부재·유사물 1상자, 조명·표면, 센서 rig(카메라 1개·LiDAR 없음), distractor 기하가 프리셋과 일치, 잘못된 카탈로그 거부.
- CLI 직접 실행 `build_scene_world.py --scene s001` → 모델 7개(floor, forklift visuals, synthetic_pallet, occluder, distractor 2, sensor rig), bridge 4토픽, transforms 카메라 1개.

## 호스트 결과

| 검사 | 결과 |
|---|---|
| 전체 회귀 `-m 'not rendering'` | **422 passed, 1 deselected** (1단계 372 + 50) |
| 문서의 코어 명령 | 320 passed |
| `ruff check .` / `ruff format --check .` | 통과 / 76 files |
| Markdown 상대 링크 | 0개 |

## 경계

- 카탈로그의 가시성·가림 비율은 핀홀 투영 계산이며 Gazebo 렌더 결과가 아니다. 실제 영상에서의 가림·시야는 3단계 캡처에서 확인한다.
- 정답은 카탈로그 기하에서 해석적으로 계산한 값이며, 렌더·양자화 오차나 실물 팔레트 치수를 반영하지 않는다.
- 가림 상자 높이 0.80 m와 시선 위 배치는 설계 검토의 반례를 막기 위한 합성 설정이다.
