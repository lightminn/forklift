# src 레이아웃 구조 전환 검증 — 2026-09-11

**범위:** [구조 전환 계획](../plans/2026-09-11-src-layout-migration.md)의 Task 7. 전환 커밋 `2e9be5a`(브랜치 `refactor/src-layout`, 기준 `main` `1beea12`)가 기존 동작을 보존하고, 호스트·wheel·개발 컨테이너·원격 Slurm에서 같은 결과를 내는지 확인했다. 구조 전환은 기능 변경이 아니며 실물 센서·팔레트 인식·주행 검증이 아니다.

## 호스트 (conda base Python 3.11.7)

editable 재설치: `pip uninstall forklift-sensor-core` 후 `pip install -e '.[dev]'`. `/tmp`에서 `import forklift_core.sensors.rgbd`가 `<checkout>/src/forklift_core/sensors/rgbd.py`를 가리킨다. 루트의 옛 `forklift_core/`와 `forklift_sensor_core.egg-info/`는 없다.

| 검사 | 결과 |
|---|---|
| `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` | **146 passed, 1 deselected** (전환 전 145 + `prepare_core_environment` 시험 1) |
| `python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error` | 94 passed (코어 단위 63 + 예제 1 + 원격 도구 30) |
| `python -m ruff check .` / `python -m ruff format --check .` | 통과 / 58 files already formatted |
| `cd /tmp && python <checkout>/examples/sensor_geometry.py` | JSON 출력, cwd 무관 |
| 옛 경로 `git grep` (날짜 기록·계획·§10 이력표 제외) | 잔여 참조 없음 |
| Markdown 상대 링크 검사 | 깨진 링크 0개 |

이동한 코어·시험 파일은 import 줄과 docstring 외에 이전 revision과 바이트 동일함을 `diff`로 확인했다. `_validation.py`는 `geometry.py`의 검증 함수 3개와 `Real` import를 그대로 옮긴 것이다.

## wheel (source 밖 설치)

`python -m build --wheel` → `forklift_core-0.1.0-py3-none-any.whl`, SHA-256 `c3ee53ed89aead085f70fc24560892d433be60da42a791f83719eca4c85c3502`.

| 환경 | 결과 |
|---|---|
| 깨끗한 Python 3.11.7 venv(`--system-site-packages` 없음) | import가 venv `site-packages`, `pip check` 통과, 예제 실행, `tests/unit` **63 passed** (`--import-mode=importlib`, cwd `/tmp`) |
| `python:3.10-slim` (digest `sha256:fd76ade0…`) 컨테이너 venv, Python 3.10.21 | 동일: import·`pip check`·예제·**63 passed** |

## 개발 컨테이너 (`forklift/ros2-dev:jazzy` 재빌드)

새 이미지 ID `sha256:dbc0929b8839cae1d424bd87d91c9d2b9aba683811efcfd8018e8a6c481ad5d1`(이전 `4fe07bbd…`). `/opt/forklift/venv`의 `__editable__.forklift_core-0.1.0.pth`는 `/workspace/src` 한 줄이다.

| 검사 | 결과 |
|---|---|
| `bash tools/ros2_dev.sh python -c 'import forklift_core.sensors.rgbd as m; print(m.__file__)'` | `/workspace/src/forklift_core/sensors/rgbd.py` |
| `bash tools/ros2_dev.sh bash -c 'cd /tmp && python -c "import forklift_core; print(forklift_core.__file__)"'` | `/workspace/src/forklift_core/__init__.py` (cwd 무관) |
| `bash tools/ros2_dev.sh python -m pytest tests --ignore=tests/simulation …` | 94 passed |
| `bash tools/ros2_dev.sh python -m pytest ros2/src/forklift_ros/test …` | 28 passed |
| `bash tools/ros2_dev.sh python examples/sensor_geometry.py` | JSON 출력 |
| 호스트에서 `src/forklift_core/__init__.py`에 `PROBE = 1` 추가 후 컨테이너 import | `PROBE 1` 출력, 재빌드 없이 반영. 되돌린 뒤 `git diff`가 직전과 동일 |

## 원격 model-cpu (읽기 전용 snapshot → wheel → 실행별 venv)

로컬 사전 검사(Codex 구현 세션): 실제 snapshot 사본에 `chmod -R a-w`를 적용하고 `tools/remote_model_job.py --mode model-cpu`를 실행해 **118 passed, 1 deselected**, `core_environment.import_path`가 `<output>/.runtime/venv/…/forklift_core/__init__.py`, wheel SHA-256 `0c67c7a3…`, snapshot 내용·권한 보존과 `.runtime/` 삭제를 확인했다.

원격 실행: run ID `20260911T063812Z_src_layout_model_cpu_01`, **Slurm 980 `COMPLETED 0:0`**, 실행 12초. snapshot SHA-256 `08772037a816133c1ef7e58dc7b55d1025c2d3e9831c6561dfd14cd9e4f537ee`, revision `2e9be5a` clean. 명령 5개(`core-wheel`, `core-venv`, `core-install`, `core-import-check`, `model-cpu`) 모두 exit 0. pytest **118 tests, failures 0, errors 0, skipped 0**(호스트 146에서 ROS 패키지 시험 28을 뺀 수와 일치). `core_environment`: import 경로 `<output>/.runtime/venv/lib/python3.11/site-packages/forklift_core/__init__.py`, venv pip 24.0 · setuptools 79.0.1, wheel SHA-256 `3d5f47085a0488ee35bdcc4c92e69aee51426e9ed0405564148594d1017949f9`(로컬 wheel과는 빌드 환경이 달라 해시가 다르며 `wheels/`에 회수). 원격 결과 8개 파일의 SHA-256이 회수본과 일치했다. 결과: `artifacts/20260911T063812Z_src_layout_model_cpu_01/`.

## 원격 gazebo (센서 기준선 재실행)

첫 실행 run ID `20260911T063817Z_src_layout_gazebo_01`, Slurm 981은 **FAILED `1:0`**(83초)로 끝났고 도구는 회수를 거부했다(실패 산출물은 원격 `artifacts/`에 보존). 원격 `sensor_smoke/result.json` 확인 결과 live(센서 4종 각 161개/32.0초, clock 3202개)와 저장 bag(센서 4종 각 162개/32.2초, clock 3226개)은 통과했고, 새 프로세스 replay도 센서 4종은 162개/32.2초로 저장 bag과 정확히 일치했으나 `/clock`이 3225개로 1개 적어 `Fresh replay does not reproduce stored stream counts/time range: clock`으로 거부됐다. `sim/gazebo/`, `ros2/`, `deploy/gazebo/`는 이번 전환에서 바뀌지 않았고(`git diff 2e0d588 2e9be5a` 해당 경로 변경 없음) 2026-09-10의 성공 실행은 clock 3226/3226이었으므로 구조 전환의 회귀로 볼 근거는 약하며, 100 Hz clock 재생의 간헐 불일치로 별도 추적한다(아래 경계). 재실행 한 번의 성공이 이 간헐 현상의 해결을 뜻하지는 않는다.

재실행 run ID `20260911T064208Z_src_layout_gazebo_02`, **Slurm 982 `COMPLETED 0:0`**, 실행 82초, 같은 snapshot SHA-256 `08772037…`(revision `2e9be5a`; 이 검증 기록 등 미커밋 문서는 snapshot 허용 목록 밖이라 `dirty`로 표시됨), 이미지 ID `sha256:489f4aa6…`(변경 없음). `sensor_smoke/result.json` `passed: true`. live 센서 4종 각 161개/32.0초, 저장 bag과 새 프로세스 replay 각각 센서 4종 162개/32.2초·clock 3225개/32.24초로 **모든 스트림이 정확히 일치**했다. 컨테이너 colcon 시험 28 passed. 원격 결과 **51개 파일**의 SHA-256이 회수본과 일치했고 live/replay의 RGB·depth·scan PNG가 생성됐다. 결과: `artifacts/20260911T064208Z_src_layout_gazebo_02/`. 첫 회수 시도는 노트북 메모리 부족으로 중단돼 부분 디렉터리를 `…_02_partial_killed_collect/`로 옮겨 두고 다시 회수했다.

model-cpu(`forklift_core` 사용)와 gazebo(코어 미사용) 두 경로가 같은 snapshot에서 성공했으므로 원격 동등성 조건을 충족한다.

## 경계

- 이 기록은 파일 배치·패키징·실행 경로의 동등성만 증명한다. 코어 함수의 수치 동작은 기존 합성 시험이 보증하며 새 시험을 추가하지 않았다(`prepare_core_environment` 시험은 원격 도구 시험이다).
- Gazebo 경로는 `forklift_core`를 사용하지 않으므로 model-cpu와 gazebo 두 결과를 함께 확인해야 원격 동등성이 성립한다.
- 개발 컨테이너의 editable 구성은 기본 editable 모드와 단순 `src/` 구조에서 정적 `.pth`가 `/workspace/src`를 가리키는 것에 의존한다. `pyproject.toml`을 바꾸면 이미지를 다시 빌드한다.
- 원격 venv의 pip·setuptools는 base 환경과 다른 버전이며 `job_result.json`에 기록된다. 원격 GitHub 인증, GPU, Jetson은 이번 범위가 아니다.
- Gazebo 기준선의 replay 비교는 `/clock`까지 개수 완전 일치를 요구하며, 이번 첫 실행에서 clock 1개 차이로 실패했다. 검증기는 `/clock`도 센서 4종과 같은 reliable·depth 50으로 구독하고, bag metadata의 `/clock` offered QoS도 reliable/volatile이므로 reliability 불일치는 아니다(2026-09-11 Codex 교차검증). 원인은 미확정이며 rosbag2 player가 저장된 QoS를 적용할 때 history를 기본 `KEEP_LAST` depth 10으로 두는 점, 재생 종료 후 고정 1초 대기만 하는 종료 동기화가 가설이다. 비교 기준에서 clock 개수를 빼는 방식은 누락을 허용하므로 채택하지 않는다. 실패 bag을 고정해 반복 재생하며 누락 시각과 endpoint QoS를 확인한 뒤 `fix(sim)` 별도 커밋으로 처리한다. live clock 3202개와 저장 bag 3226개의 차이는 live 관측 종료 후에도 recorder와 Gazebo가 약 0.24초 더 기록하기 때문이며 warmup 처리 차이가 아니다.
