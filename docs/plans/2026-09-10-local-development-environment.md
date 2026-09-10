# 로컬 개발 환경 구성 계획

작성일: 2026-09-10

**후속 계획:** 사용자가 원격 워크스테이션에서 시뮬레이션하는 방향을 제안하여 [노트북·원격 분담 구성안](2026-09-10-local-and-remote-development-environment.md)이 실행 위치와 우선순위를 대체한다. 아래는 최초 계획 기록이며, 구조 전환을 원격 실행의 선행 조건으로 적용하지 않는다.

**상태:** 계획 수립 단계. 설치·패키지 업데이트·소스 이동·ROS 작업공간 생성·장치 연결은 실행하지 않았다.

**목표:** 기존 개발용 Python 환경에서 코드를 작성하고, 장치가 없는 깨끗한 Docker 환경에서도 설치·시험을 재현할 수 있게 한다.

**구조:** 호스트는 편집·수학·영상 처리·빠른 테스트를 담당한다. 별도 컨테이너는 패키징과 지원 Python 버전을 검사한다. ROS 통합과 Jetson 배포는 타깃이 정해진 뒤 별도 단계에서 추가한다.

**도구:** Python >= 3.10, NumPy, pytest, Ruff, build, pip-tools, Docker. 개발용 Python 선택은 사용자의 환경 지침을 따른다. ROS·센서 SDK·시뮬레이터는 코어 필수 의존성에 넣지 않는다.

**기준:** [개발 컨벤션](../../CONTRIBUTING.md), [현재 상태와 실행법](../../README.md).

## 1. 완료 범위와 작업 경계

초기 환경 구성은 아래 2–5절까지다. 완료 시 할 수 있어야 하는 일은 다음과 같다.

- 컨벤션에 맞게 배치된 패키지를 editable 설치하고 임의 작업 디렉터리에서 import한다.
- Ruff lint·format check와 기존 64개 테스트를 실행한다.
- wheel을 만들고 Python 3.10·3.11의 별도 Linux x86_64 환경에서 설치·검사한다.
- 설치 버전, 실행 명령, 입력 출처, 실패·skip 여부를 재현 기록에 남긴다.

시뮬레이터 선정, ROS 배포판 확정, MCU 툴체인 설치, Jetson 접속·플래시는 이 초기 단계의 완료 조건이 아니다. 해당 도구의 설치가 코어 환경 구성을 막게 하지 않는다. 하드웨어 동작 검증은 별도다.

기존 Python 환경을 개발 환경으로 유지한다. 운영체제 교체, 전역 ROS 설치, 일괄 패키지 업그레이드를 계획하지 않는다. 다른 프로젝트 이미지·컨테이너를 정리하지 않는다. 프로젝트 정본에는 개인 설치 경로와 설치 목록을 복제하지 않고, 실제 머신 변경은 개인 환경 기록에 남긴다.

## 2. 개발 도구 보강

**대상:** 개발용 Python의 Ruff, build, pip-tools. 이미 충족하는 NumPy·pytest 등은 재설치 대상으로 삼지 않는다.

- [x] 지정 Python의 버전·모듈과 외부 도구를 읽기 전용으로 조사한다. 현재 관측과 과거 성공 기록을 구분한다.
- [x] Docker 이미지 목록을 확인한다. 샌드박스의 소켓 접근 거부는 설치 부재와 구분한다.
- [ ] 설정 작업 시작 시 새로운 기록 디렉터리를 만든다. 기존 결과 디렉터리를 재사용하지 않는다.

```bash
mkdir -p artifacts
SETUP_OUTPUT="artifacts/$(date -u +%Y%m%dT%H%M%SZ)_environment_setup"
mkdir "$SETUP_OUTPUT"
python -m pip list --format=json > "$SETUP_OUTPUT/packages_before.json"
python -m pip check > "$SETUP_OUTPUT/pip_check_before.txt" 2>&1
```

이후 셸 예제는 같은 셸의 `SETUP_OUTPUT`을 사용하며, `python`은 지정된 개발 환경의 인터프리터다. 초기 `pip check`가 실패하면 기존 문제로 기록한다. 그 결과만으로 다른 프로젝트의 의존성을 수정하지 않는다.

기존 환경의 교체를 방지하는 임시 constraints를 만든다. 이는 개인 환경의 변경 범위를 제한하는 기록이며, 프로젝트의 재현용 requirements로 커밋하지 않는다.

```bash
python - "$SETUP_OUTPUT/packages_before.json" "$SETUP_OUTPUT/host_constraints.txt" <<'PY'
import json
from pathlib import Path
import sys

packages = json.loads(Path(sys.argv[1]).read_text())
Path(sys.argv[2]).write_text("".join(
    f"{package['name']}=={package['version']}\n" for package in packages
    if package['name'].lower().replace('_', '-') not in {'forklift-core', 'forklift-sensor-core'}
))
PY
```

- [ ] 설치 시점의 resolver 결과를 먼저 확인한다.

```bash
python -m pip install --dry-run --constraint "$SETUP_OUTPUT/host_constraints.txt" \
  --report "$SETUP_OUTPUT/tools_install_plan.json" ruff build pip-tools
```

검토 조건은 누락 도구와 필요한 의존성만 추가되는지다. 기존 수학·영상·학습 패키지의 교체가 필요하면 해당 호스트 설치 경로를 진행하지 않고, 충돌하는 도구만 이후의 독립 컨테이너에서 실행하도록 조정한다. `pip-sync`를 공유 개발 환경에 실행하지 않는다.

- [ ] 확인한 도구 버전과 의존성을 기록해 같은 버전으로 설치한다. 새 버전 전체 업데이트 옵션을 쓰지 않는다.

```bash
python - "$SETUP_OUTPUT/tools_install_plan.json" "$SETUP_OUTPUT/tools_install.txt" <<'PY'
import json
from pathlib import Path
import sys

report = json.loads(Path(sys.argv[1]).read_text())
Path(sys.argv[2]).write_text("".join(
    f"{item['metadata']['name']}=={item['metadata']['version']}\n"
    for item in report['install']
))
PY
python -m pip install --constraint "$SETUP_OUTPUT/host_constraints.txt" \
  --requirement "$SETUP_OUTPUT/tools_install.txt"
```

- [ ] 아래 도구 실행과 기존 라이브러리 import를 확인한다.

```bash
python -m ruff --version
python -m build --version
python -m piptools --version
python -c 'import numpy, pytest, cv2, pyrealsense2; print("host imports OK")'
```

**완료 조건:** 검사·빌드·의존성 고정 도구가 실행되고 기존 라이브러리의 import가 유지된다. RealSense SDK import는 카메라 스트리밍 성공으로 기록하지 않는다.

## 3. 저장소 구조와 개발 설치 연결

이 단계는 환경 구성에 필요한 선행 구조 정리다. 새로운 센서·로봇 기능을 추가하지 않는다.

**파일 범위:** `CONTRIBUTING.md` 마지막 절의 이동 목록, `pyproject.toml`, `.editorconfig`, `.gitignore`, `README.md`, `AGENTS.md`, `CLAUDE.md`.

- [ ] 기존 코드와 테스트의 상태를 기록한다. 직전 검증의 기준은 64개 통과이며, 이동 전에 한 번 확인한다.
- [ ] 코드를 `src/forklift_core/`, 시험을 `tests/unit/`·`tests/integration/`, 합성 예제를 `examples/sensor_geometry.py`로 이동한다. 공유 입력 검증은 `_validation.py`로 옮기고 import 경로를 고친다.
- [ ] 관련 문서·과제 링크를 컨벤션의 목표 위치로 옮기고 실제 참조도 함께 갱신한다. 발표 저장소 링크와 기존 ZIP은 이동 대상에서 제외한다.
- [ ] 패키지명은 `forklift-core`, import명은 `forklift_core`로 맞추고 아래 설정을 기존 `pyproject.toml`에 반영한다. 기존 항목 전체를 덮어쓰지 않는다.

```toml
[project.optional-dependencies]
test = ["pytest>=7"]
dev = ["pytest>=7", "ruff", "build", "pip-tools"]

[tool.setuptools.packages.find]
where = ["src"]
include = ["forklift_core*"]

[tool.ruff]
line-length = 88
target-version = "py310"
extend-exclude = ["presentation", "artifacts", "data", "ros2/build", "ros2/install", "ros2/log"]

[tool.ruff.lint]
select = ["E4", "E7", "E9", "F", "I", "UP", "B"]

[tool.ruff.format]
quote-style = "double"
indent-style = "space"
line-ending = "lf"

[tool.pytest.ini_options]
testpaths = ["tests"]
addopts = ["--import-mode=importlib"]
```

기존 `test` extra는 호환성을 위해 유지한다. `dev`는 일상 개발용, `test`는 배포 wheel의 검사용이며 동일한 pytest 최소 버전을 유지한다.

- [ ] `.editorconfig`에 UTF-8·LF·파일 끝 개행·Python 공백 4칸을 설정한다. `.gitignore`에는 캐시, 생성물, editable의 egg-info, `artifacts/` 결과, `data/` 대형 원본, ROS의 build/install/log를 추가한다. 기존 파일은 삭제하지 않는다.
- [ ] 2절에서 확인한 의존성을 보존하는 constraints와 함께 editable 설치의 dry-run을 검토한 후 설치한다.

```bash
python -m pip install --dry-run --constraint "$SETUP_OUTPUT/host_constraints.txt" -e '.[dev]'
python -m pip install --constraint "$SETUP_OUTPUT/host_constraints.txt" -e '.[dev]'
python -m ruff check src tests examples
python -m ruff format --check src tests examples
python -m pytest tests -q -p no:cacheprovider -W error
python examples/sensor_geometry.py
```

위 두 설치 명령에는 동일한 constraints를 적용한다. 검사 전에는 실제 포맷을 적용하고 lint 지적의 원인을 수정한다. 검사 대상에 발표 링크를 넣지 않는다.

**완료 조건:** 64개 기존 동작 시험 통과, Ruff 두 검사 통과, 새 예제 명령의 JSON 출력 확인. 소스 루트 밖의 새 Python 프로세스에서도 `forklift_core`를 import할 수 있어야 한다. 지원되지 않는 경로를 `sys.path` 수정으로 숨기지 않는다.

## 4. 의존성 고정과 깨끗한 설치 검사

**새 파일:** `requirements/dev_py311.txt`, `requirements/test_py310.txt`, `requirements/test_py311.txt`, `requirements/README.md`, `deploy/Dockerfile.core_check`, `.dockerignore`.

- [ ] `pyproject.toml`에서 필요한 extra만 입력으로 사용한다. 전체 개인 환경의 `pip freeze`를 프로젝트 의존성으로 삼지 않는다.
- [ ] 개발용 고정 목록은 확인된 기존 직접 의존성 버전을 constraints로 보존하며 생성한다. 예를 들어 개발 환경의 NumPy·pytest 버전을 변경하기 위한 재해석을 하지 않는다.

```bash
python -m piptools compile --extra dev --generate-hashes \
  --constraint "$SETUP_OUTPUT/host_constraints.txt" \
  --constraint "$SETUP_OUTPUT/tools_install.txt" \
  --output-file requirements/dev_py311.txt pyproject.toml
```

두 constraints로 기존 의존성과 이번에 설치한 도구 버전을 함께 보존한다. `pip-compile`은 설치된 모든 버전을 자동 보존하는 명령이 아니므로 출력 차이를 검토한다. [pip-tools 공식 설명](https://pip-tools.readthedocs.io/en/stable/)

- [ ] 시험용 고정 목록은 각 대상 Python의 깨끗한 컨테이너에서 `--extra test`로 생성한다. Python 3.11에서 만든 결과를 파일명만 바꾸어 3.10 결과로 사용하지 않는다.
- [ ] 별도 기록 디렉터리에 wheel을 만든다.

```bash
python -m build --wheel --outdir "$SETUP_OUTPUT/wheels"
```

- [ ] `deploy/Dockerfile.core_check`는 다음 계약으로 구성한다.

| 항목 | 설정 |
|---|---|
| 기본 이미지 | `python:3.10-slim-bookworm`, `python:3.11-slim-bookworm` 두 대상; 구성 시 실제 digest 기록·고정 |
| 설치 입력 | 대상별 해시 포함 test 고정 목록 + 이번 실행에서 생성한 wheel 한 개 |
| 설치 방식 | requirements는 `--require-hashes`, 프로젝트 wheel은 의존성 설치 뒤 `--no-deps` |
| 런타임 입력 | `/verification/tests`, `/verification/examples`, pytest 설정만 제공 |
| 소스 노출 | `src/`와 editable 설치 정보는 이미지·런타임 mount에서 제외 |
| 검사 | `pip check`, 설치 위치 출력, 전체 로컬 시험, 합성 예제 |
| 자원 | 최초 실행은 CPU 4개·메모리 4GiB 제한 |

wheel 경로는 해당 실행의 새 출력 폴더에서 한 개임을 확인한다. 과거 wheel과 섞인 전역 `dist/`의 임의 파일을 선택하지 않는다. 이미지 빌드에 테스트 코드를 복사하더라도 패키지 소스는 넣지 않는다. 테스트의 예제 경로는 `/verification` 구조에서도 동작해야 한다.

빌드 입력은 `$SETUP_OUTPUT/context/`에 선택한 wheel·시험용 고정 목록·tests·examples·pytest 설정만 모아서 구성한다. 저장소 전체를 빌드 컨텍스트로 보내지 않는다. Dockerfile은 `deploy/Dockerfile.core_check`를 명시적으로 지정한다. `.dockerignore`의 생성물 제외 규칙 때문에 검사할 wheel까지 빠지는 구성은 피한다.

- [ ] 패키지를 설치하는 이미지 빌드 단계에는 다운로드를 허용하되, 완성된 검증 컨테이너의 시험은 `--network none`으로 실행한다. 호스트 홈 디렉터리나 Docker 소켓은 검증 컨테이너에 mount하지 않는다.
- [ ] 각 대상에서 결과와 실제 import 위치를 기록한다. 필요한 의존성이 호스트에만 있어서 가려지는 경우가 없는지 확인한다.

**완료 조건:** Python 3.10과 3.11의 두 환경에서 설치된 wheel로 시험·예제가 통과한다. 이는 해당 두 환경의 검사이며 NumPy·pytest의 선언된 모든 최소 버전 조합을 검증했다는 뜻은 아니다. ARM64·Jetson 검증도 별도다.

## 5. 환경 설정 완료 기록

**대상:** `README.md`, `requirements/README.md`, `docs/validation/2026-09-10-environment-setup.md`, 사용자 개인 환경 기록.

- [ ] 전후 패키지 목록과 `pip check`를 비교해 새 충돌 여부를 기록한다. 이미 있던 다른 프로젝트 문제를 이번 변경의 성공/실패와 혼동하지 않는다.
- [ ] 개인 환경 기록에는 실제 설치한 버전·인터프리터·확인일·명령·결과를 쓴다. 이 계획 작성만으로 설치 완료를 기록하지 않는다.
- [ ] 프로젝트 검증 기록에는 소스/설정 식별자, 두 Python 대상, 고정 목록, wheel 해시, 컨테이너 digest, 검사 결과와 미검증 항목을 쓴다.
- [ ] 재시작한 새 셸에서 README 명령을 실행해 일시적인 PATH나 우연한 작업 디렉터리에 의존하지 않는지 확인한다.
- [ ] 새 이미지·캐시·결과의 실제 디스크 증가량을 확인한다. **초기 추가 사용량 10GiB 이내**를 계획 예산으로 두며, 더 큰 설치는 원인을 확인하고 범위를 조정한다. 이 수치는 다운로드 크기 실측값이 아니다.

**초기 단계 완료 보고:** 호스트 검사 결과, Python별 wheel 검사 결과, 설치한 도구, 변경 파일, 추가 디스크 사용량, 재실행 명령을 짧게 제시한다.

## 6. ROS 통합 환경은 타깃 확정 후 추가

이 절은 조건부 후속 계획이다. 지금 ROS 작업공간이나 통합 시뮬레이터를 생성하지 않는다. 기존에 캐시된 ROS 이미지가 최종 배포판을 결정하지 않는다.

| 실제 타깃 기반 OS | ROS 후보 | 결정 조건 |
|---|---|---|
| Ubuntu 22.04 기반 JetPack | ROS 2 Humble | 해당 Jetson·JetPack과 D435i/RPLIDAR 드라이버 조합 확인 |
| Ubuntu 24.04 기반 JetPack | ROS 2 Jazzy | 같은 드라이버 및 배포 라이브러리 지원 확인 |
| 그 밖의 타깃 | 다시 검토 | 후보 표를 억지로 적용하지 않음 |

JetPack 6.2.2는 Ubuntu 22.04 기반이고, 현재 JetPack 7은 Ubuntu 24.04 기반이다. 최신 공식 안내에는 Orin도 JetPack 7 지원 대상에 포함되므로 **Orin이라는 이름만으로 Humble을 고정하지 않는다**. [NVIDIA JetPack 6.2.2](https://developer.nvidia.com/embedded/jetpack-sdk-622), [NVIDIA JetPack](https://developer.nvidia.com/embedded/jetpack)

ROS의 Ubuntu 대응은 공식 설치 문서에서 다시 확인한다. [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), [Jazzy](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html)

타깃 확정 뒤 할 작업:

1. `deploy/`에 프로젝트 전용 ROS 이미지를 만든다. ROS base와 colcon·rosdep을 시작점으로 하고, 센서·TF·rosbag·RViz·SLAM/Nav2는 해당 기능을 통합할 때 추가한다.
2. `ros2/src/`에 필요한 프로젝트 패키지만 생성한다. 호스트 conda를 ROS 런타임에 mount하거나 `rclpy`를 호스트 conda에 혼합 설치하지 않는다.
3. 메시지 송수신 → TF → 기록/재생 → RViz 순서로 검사한다. GUI는 이 머신의 실제 세션과 Intel 그래픽 렌더러를 확인한 뒤 연결한다. 센서 없는 headless 성공과 GUI 성공을 별도 기록한다.
4. D435i는 `realsense2_camera`, RPLIDAR는 공식 SLAMTEC 드라이버를 연결한다. A2 세부형과 실제 USB/시리얼 장치를 확인하고 필요한 장치만 컨테이너에 연결한다. host udev 설정은 이미 있는 규칙과 비교한 뒤 필요한 변경만 한다.
5. Jetson용 이미지는 실제 ARM64·JetPack/L4T와 CUDA 런타임에 맞춰 별도 검사한다. x86 이미지 실행이나 QEMU 실행을 Jetson 성능·GPU·USB 검증으로 보고하지 않는다.

RealSense의 Jetson 설치 경로는 JetPack에 따라 달라질 수 있으므로 SDK의 해당 가이드를 따른다. [RealSense Jetson 가이드](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)

## 7. 시뮬레이션·실물 준비의 후속 순서

차체가 정해지기 전에는 합성 센서 입력·기록 재생·코어 시험을 진행한다. 설치된 물리 엔진의 간단한 실행 확인을 프로젝트 시뮬레이터 선정으로 해석하지 않는다.

차체 확인 후에는 다음 순서로 통합 환경을 정한다.

1. 차체 치수·조향·회전반경·포크 구조를 모델로 표현한다.
2. 작은 환경에서 운동학/물리 계산과 화면 렌더링을 각각 시험한다.
3. 필요한 RGB-D·LiDAR 출력, ROS 연동, 로컬 메모리·실행 속도를 비교해 엔진을 선택한다.
4. A–D 시나리오와 삽입/승강/적재 상태를 연결한다.

새 대형 시뮬레이터, Isaac 계열 구성, Open3D, MCU IDE·크로스 컴파일러는 그 기능과 타깃이 실제로 필요해질 때 추가한다. 기존에 설치된 패키지는 이 계획 때문에 제거하지 않는다.

## 8. 실행 순서 요약

| 단계 | 결과물 | 다음 단계로 넘어가는 기준 |
|---|---|---|
| 개발 도구 보강 | Ruff·build·pip-tools의 실행 경로 | 실행 성공, 기존 import 유지 |
| 구조·개발 설치 | `src` 배치·editable 설치·Ruff 설정 | 기존 64개 시험 + 두 정적 검사 + 예제 통과 |
| 의존성·wheel 검사 | 대상별 고정 목록·검증 이미지 | Python 3.10/3.11의 깨끗한 설치 검사 통과 |
| 기록·재실행 | README·검증 기록·개인 환경 기록 | 새 셸에서 동일 절차 재현 |
| ROS/Jetson | 선택한 타깃의 통합 환경 | 하드웨어·OS·드라이버 조합 확정 후 시작 |
| 차체 시뮬레이션 | 모델·환경·시나리오 | 차체 확인과 엔진 비교 후 시작 |

예상 작업량은 초기 네 단계 합계 약 2–3시간이다. 이는 작업 계획상의 추정이며 다운로드 속도·의존성 충돌·구조 전환 중 발견되는 문제에 따라 달라진다. ROS 통합과 물리 시뮬레이션 시간은 이 추정에 포함하지 않는다.
