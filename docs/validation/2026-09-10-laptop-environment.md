# 노트북 개발 환경 검증

검증일: 2026-09-10. 상태: **노트북 개발 도구·패키징·ROS 2 개발 컨테이너 검증 완료**.

하드웨어 권장안은 [하드웨어 정본](../hardware.md), 개발·시뮬레이션 역할은 [ADR 0001](../decisions/0001-development-and-deployment-platforms.md), 실제 사용법은 [개발 환경](../development.md)을 따른다.

## 변경과 범위

- 기존 root `forklift_core/` 구조와 `forklift-sensor-core` 배포명을 유지했다. `dev` extra, Ruff, editor/ignore 설정과 ROS 개발용 Dockerfile·launcher를 추가했다.
- Python 7개 파일은 Ruff 포맷과 표준 라이브러리 import 정렬만 변경했다. 별도 리뷰에서 AST 동일성을 확인했고 전체 기존 시험을 재실행했다.
- 호스트 개발 도구 4개(Ruff, build, pip-tools, pyproject_hooks)와 프로젝트 editable 패키지를 추가했다. 설치 전후 목록 비교에서 기존 패키지 버전 변경은 0개다.
- 원격 시뮬레이터·Slurm·Jetson 설치, 센서 드라이버 연결, 차체 제어 기능은 이번 변경 범위에 포함하지 않았다.

## 실행 결과

| 검사 | 환경과 명령 | 실제 결과 |
|---|---|---|
| 호스트 전체 시험 | Python 3.11.7, `FORKLIFT_RENDER_TEST=1 python -m pytest tests -q -p no:cacheprovider -W error` | **81 passed**, skip 없음; 합성 센서 기하와 잠정 모델 시험 |
| Ruff | `python -m ruff check forklift_core tests tools`, `python -m ruff format --check forklift_core tests tools` | lint 통과, Python 13개 파일 포맷 통과 |
| 패키지 빌드 | `python -m build --wheel --outdir <scratch>/wheels` | 격리 빌드 성공, `forklift_sensor_core-0.1.0-py3-none-any.whl` |
| wheel 최소 Python | 별도 Python 3.10.21 컨테이너의 새 venv, source 밖 `/tmp`에서 import·demo·pytest | 설치 경로 import, `pip check` 통과, **64 passed** |
| wheel 호스트 계열 | 별도 Python 3.11.7 venv, source 밖 `/tmp`에서 import·demo·pytest | 설치 경로 import, `pip check` 통과, **64 passed** |
| ROS 개발 이미지 | `docker build -f deploy/ros2/Dockerfile --build-arg DEV_UID=... --build-arg DEV_GID=... -t forklift/ros2-dev:jazzy .` | Ubuntu 24.04.4 / ROS 2 Jazzy / Python 3.12.3 빌드 성공 |
| ROS 컨테이너 코어 시험 | `bash tools/ros2_dev.sh python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error` | **64 passed** |
| ROS 프로세스 간 통신 | 한 컨테이너의 `demo_nodes_cpp` talker/listener | 메시지 수신 확인 |
| ROS 컨테이너 간 통신 | 기본 bridge의 두 별도 컨테이너, 같은 임시 ROS domain | **6개 메시지 수신** 확인, 설정된 timeout으로 종료 |
| launcher 입출력·권한 | heredoc 표준입력, checkout import 위치, 결과 파일 소유권, `bash -n tools/ros2_dev.sh` | non-root 실행, host UID/GID 일치, `/workspace` 코드 import, 문법 통과 |
| 새 셸 editable | 선택한 개발 환경을 활성화한 새 zsh에서 source 밖 import·Ruff 조회 | 성공; 환경 자동 활성화를 전제로 하지 않음 |

wheel 시험은 `--import-mode=importlib`로 코어 시험만 실행했다. 별도의 source 밖 import와 demo로 설치된 wheel 경로도 확인했다. MuJoCo·렌더링 시험을 wheel 최소 의존성 시험에 포함한 것은 아니다.

## 의존성·실행 제한

- 공유 호스트의 `pip check`는 설치 전부터 기존 conda 관련 의존성 문제 9행으로 실패했다. 설치 후 출력이 완전히 같음을 확인했으며, 이 호스트 전체를 깨끗한 환경이라고 판정하지 않았다. 별도 wheel venv의 `pip check`는 통과했다.
- 비격리 호스트 wheel 빌드는 기존 conda build entry point 경고를 출력했다. 최종 산출물은 별도 빌드 환경을 사용하는 기본 `python -m build`로 다시 생성했다. 호스트의 다른 프로젝트 패키지는 수정하지 않았다.
- Python 3.10 이미지의 첫 pull은 연결 reset으로 실패했고 같은 공식 이미지의 재시도는 성공했다.
- 초기 컨테이너 간 검증 스크립트는 ROS 로그를 stdout에서만 찾았으나 메시지는 stderr에 있었다. stdout+stderr를 함께 수집하도록 검사 코드를 수정한 뒤 두 컨테이너 시험을 다시 통과했다. 이는 ROS 전달 실패가 아니었다.
- Docker 빌드는 기존 legacy builder로 성공했다. 출력의 buildx 설치 권고는 기록했으며, 이번 작업에서 Docker daemon이나 다른 프로젝트 이미지를 변경하지 않았다.
- 개발 컨테이너는 기본 bridge와 저장소 마운트로 실행한다. GUI·USB 센서·GPU 전달, 노트북–원격 DDS, ARM64 실행은 검증하지 않았다.

## 재현 식별자와 원본 기록

- 실행 당시 Git: 초기 커밋 전 작업 트리. 소스 manifest SHA-256: `5cbeae7bbd659c2280e66dc39ceeff0f8aad804c3fa8a94632beee137960deb2`.
- wheel SHA-256: `0f089b0814465ad67263282346f7308ffd4662592584e7f93b009638da8d639f`.
- ROS base digest: `sha256:386d06ec6d4188f731bae5678e07b4cb64a4e4d4152090c0bd1f881dcf7706f5`.
- 검증한 개발 이미지 ID: `sha256:4fe07bbd6d6ddaf585776de8f991763ddb985db603b84a82600a8d331fff999f`.
- 원본 기록은 Git 제외 `artifacts/20260910T082554Z_environment_setup_01/`에 있다. `validated_source_sha256.json`, 패키지 전후 목록·constraints·resolver 보고서, `host_pytest_final.txt`, `container_pytest_final.txt`, `wheel_py310.txt`, `wheel_py311_*.txt`, `ros_bridge_*_final.txt`, `docker_build_final.txt`, 리뷰 보고서를 포함한다.
- 기본 이미지 digest는 고정했지만 apt 저장소 전체 snapshot을 고정한 것은 아니다. 다시 빌드할 때 동일 binary image ID를 보장하지 않으며 버전 목록과 위 시험을 재확인한다.

코어·컨테이너 시험 성공은 D435i/RPLIDAR 실제 수집, 팔레트 인식·SLAM, Gazebo 통합, 지게차 주행 또는 안전 정지 검증을 뜻하지 않는다.
