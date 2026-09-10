# 노트북 개발 · 원격 워크스테이션 시뮬레이션 구성안

작성일: 2026-09-10. **상태: 개발·배포 기준 선택 및 노트북 기본 구성 검증 완료. Docker 이미지 빌드, 컨테이너 코어 시험 64개와 ROS 2 프로세스 간 통신, 호스트 기준 시험 81개를 확인했다. 원격 동기화·시뮬레이션 작업 제출과 Jetson 실물 검증은 실행하지 않았다.**

사용자가 시뮬레이션을 별도 고성능 워크스테이션에서 실행하는 방향을 제안했다. 이 문서가 [기존 단일 머신 환경 계획](2026-09-10-local-development-environment.md)의 실행 위치와 우선순위를 대체한다. 기존 계획의 의존성 보존·wheel 검사 원칙은 유지한다. 호스트명·SSH 별칭·개인 경로·설치 사양과 실행 당시 대기열은 개인 환경 기록에 둔다.

**목표:** 노트북에서 편집·빠른 검사를 하고, 원격에서 재현 가능한 모델 검사·렌더링·장시간 실험을 실행하며 결과를 노트북으로 가져온다.

**선택한 구성:** 노트북의 Ubuntu 24.04 / ROS 2 Jazzy 개발 컨테이너 + 원격 Gazebo Harmonic 통합 시뮬레이션 + 기존 MuJoCo 빠른 모델 검사. 원격은 시뮬레이션·렌더링·학습을 맡고 실제 제어는 온보드에서 실행한다. Jetson 후보는 JetPack 7.2.1 / Ubuntu 24.04 / Jazzy이며 D435i 실물 검증 전에는 최종 버전으로 동결하지 않는다.

## 1. 역할 분담

| 위치 | 맡는 일 | 환경 |
|---|---|---|
| 노트북 | 코드·모델 설정 편집, Ruff, 코어 시험, ROS 2 개발, 간단한 모델 자세 확인, 결과 열람 | 호스트 편집 도구 + Ubuntu 24.04 / ROS 2 Jazzy 컨테이너 |
| 원격 CPU 작업 | 모델 생성, 비렌더링 시험, 향후 물리 적분·파라미터 반복 시험 | 프로젝트 전용 Python, Slurm CPU 자원 |
| 원격 GPU 작업 | EGL 렌더링, 향후 RGB-D 렌더·영상 인식 등 GPU가 필요한 작업 | 같은 모델 환경, Slurm GPU 할당 |
| 원격 ROS 통합 | 센서 메시지·TF·rosbag·SLAM·계획/제어 연결과 장시간 실험 | Ubuntu 24.04 / ROS 2 Jazzy + Gazebo Harmonic 계획 |
| 향후 Jetson | 실제 D435i/RPLIDAR·구동기와 연결되는 상위 제어 | Orin Nano Super / JetPack 7.2.1 후보, 실물 ARM64 검증 후 동결 |

현재 모델 검사는 노트북에서도 짧게 실행할 수 있다. 원격 GPU 대기열이 길 때 이 경로를 유지한다. 현재 `mujoco.mj_step` 검사에는 GPU 요청이 필요하지 않으며 EGL 렌더링 작업과 구분한다. 여러 물리 실험을 GPU에서 자동 병렬화하는 MJX/Warp 구현은 현재 없다.

코어 알고리즘과 센서 관측·명령 계약은 두 머신에서 같은 코드를 사용한다. 원격 시뮬레이터가 미래 실물 로봇의 실시간 제어 루프를 네트워크 너머에서 맡도록 설계하지 않는다.

## 2. 대안 비교

| 구성 | 판단 |
|---|---|
| 노트북 컨테이너 개발/검사 + 원격 배치 실행 | **선택.** 오프라인 개발 가능, 원격 GPU 대기 중에도 진전 가능 |
| 모든 편집·시험을 Remote SSH에서 수행 | 원격 디버깅에는 편리하지만 접속·대기열에 개발 전체가 의존. 필요한 디버깅 때만 사용 |
| 노트북 호스트에 ROS·대형 시뮬레이터 직접 설치 | 호스트 환경과 결합되므로 기본 경로로 사용하지 않음. 로컬 센서 연결이나 GUI 요구가 생기면 필요 범위를 다시 결정 |

## 3. 노트북: 개발 도구와 ROS 컨테이너

**변경할 파일:** `pyproject.toml`의 `dev` extra와 Ruff 설정, `deploy/ros2/Dockerfile`, `tools/ros2_dev.sh`, 실행 안내 `docs/development.md`와 `README.md`. 기존 `test`·`model` extra를 보존한다. `src/` 이동은 별도 작업이며 컨테이너 구성의 선행 조건으로 두지 않는다.

- [x] Ubuntu 24.04 / ROS 2 Jazzy 컨테이너를 노트북 ROS 개발 기준으로 선택했다.
- [x] 개발 이미지 빌드, 워크스페이스 마운트, ROS 환경과 talker/listener 프로세스 간 통신을 확인했다.
- [x] 기존 패키지 목록·`pip check`를 `artifacts/20260910T082554Z_environment_setup_01/`에 보관했다.
- [x] resolver 계획을 확인하고 **Ruff·build·pip-tools**만 보강했다. 기존 설치 의존성은 변경하지 않았다.
- [x] `dev` extra와 Ruff 설정을 반영했다. Ruff 줄 길이 88, Python 3.10 대상, `E4/E7/E9/F/I/UP/B` 규칙을 적용한다.
- [x] Ruff lint·format 검사를 통과했다. Python 7개 파일은 포맷과 표준 라이브러리 import 정렬만 변경했으며, 구조 이동 없이 기존 전체 시험 81개를 다시 통과했다.
- [x] 호스트에서 현재 전체 시험 81개, 컨테이너에서 코어 시험 64개를 확인했다. 모델 검사는 선택 의존성으로 유지한다.

```bash
python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error
python -m forklift_core.demo
FORKLIFT_RENDER_TEST=1 python -m pytest tests/simulation -q -p no:cacheprovider -W error
```

현재 기준은 코어 64개·모델 17개다. 이 구성 단계에서는 노트북 OS 교체나 호스트 Python 환경에 ROS를 혼합 설치하지 않는다. 실제 명령·환경·결과는 [노트북 환경 검증 기록](../validation/2026-09-10-laptop-environment.md)에 남긴다. 이 결과는 원격 Gazebo, Jetson ARM64 또는 실물 센서 검증을 대신하지 않는다.

## 4. 원격: 전용 환경과 첫 재현

원격의 기존 연구 환경을 재사용하거나 수정하지 않고 프로젝트 전용 Python 3.11 환경을 만든다. 노트북과 같은 Python minor·MuJoCo 버전으로 첫 재현의 변수를 줄인다. ROS 런타임은 이 환경과 분리한다.

**구현할 파일:** `requirements/model_py311.txt`, `requirements/README.md`, `deploy/slurm/model_cpu.sbatch`, `deploy/slurm/model_render.sbatch`. Python 의존성 정본은 `pyproject.toml`이고 lock은 그중 `test`·`model` extra에서 생성한다. 개인 전체 `pip freeze`를 프로젝트 lock으로 쓰지 않는다.

- [ ] 전용 환경을 만들고 MuJoCo 3.10.0을 첫 재현 기준으로 삼아 `test`·`model` 의존성을 고정한다. Python 환경 생성 위치는 개인 환경 기록에서 지정한다.
- [ ] 첫 소스 스냅샷을 보내고 설치 후 `pip check`, `forklift_core` import 위치, 모델 생성 결과를 확인한다.
- [ ] CPU job은 4 CPU·8GiB·10분을 초기 요청값으로 삼고 `python -m pytest tests -m 'not rendering' -q -p no:cacheprovider -W error`를 실행한다. 렌더 시험 1개는 deselect되므로 기대값은 현재 **80개 통과·1개 deselect**다. 모델 의존성이 없어 발생한 skip을 정상 결과로 받지 않는다.
- [ ] GPU job은 4 CPU·8GiB·GPU 1개·10분으로 전체 시험과 아래 미리보기를 실행한다. 자원 요청값은 실행 후 최대 사용량에 맞춰 조정한다.

```bash
FORKLIFT_RENDER_TEST=1 python -m pytest tests -q -p no:cacheprovider -W error
python tools/preview_forklift_model.py \
  --model sim/models/dls08_provisional/scene.xml \
  --output "$FORKLIFT_OUTPUT_DIR" --backend egl --frames 96
```

`FORKLIFT_OUTPUT_DIR`는 제출 시 정한 해당 실행의 새 결과 디렉터리다. job은 비어 있는 변수, 기존 결과 디렉터리, 없는 checkout·Python 실행 경로를 시작 전에 거부한다. GPU 렌더러 문자열·드라이버·설정/소스 해시를 결과에 남겨 CPU 소프트웨어 렌더링으로 바뀌지 않았는지 확인한다. `sbatch --parsable`의 job ID, Slurm 종료 상태와 exit code, 실제 pytest/영상 출력을 함께 확인한다. Accounting이 비활성화된 대상에서는 `sacct` 대신 `scontrol show job`을 완료 직후 확인한다.

**첫 완료 조건:** 원격에서 81개 시험 통과, 실제 NVIDIA EGL 렌더 확인, PNG 및 4초 MP4 생성, 노트북으로 결과 회수·직접 확인. SSH 접속·GPU 목록 조회는 이 완료 조건을 대신하지 않는다.

## 5. 소스와 결과의 전달

**구현할 도구:** `tools/submit_model_check.py` 하나에 CPU/GPU 모드, 명시적 SSH host·remote root·Python 경로·출력 ID를 받도록 한다. 개인 기본 SSH 별칭이나 경로를 코드에 고정하지 않는다. 내부적으로 `rsync`와 `sbatch`를 호출하며 `--dry-run`은 원격 변경 없이 전송 파일·제출 내용을 보여준다.

- [ ] 원격 실행에는 동일 Git commit을 사용한다. 2026-09-10 사용자의 별도 업로드 요청으로 로컬 Git을 초기화했다. 미커밋 검증은 소스 SHA-256 스냅샷으로 식별하며, 이후 commit/push는 사용자가 승인한 범위에서 수행한다.
- [ ] 실행마다 **새 원격 소스 디렉터리**에 `forklift_core/`, `tools/`, `tests/`, `sim/models/`, `pyproject.toml`, 필요한 lock·job 파일만 보낸다. `.git/`, `presentation/`, `data/`, 기존 `artifacts/`, 캐시·비밀 파일은 제외한다.
- [ ] 편집 중인 공유 checkout 위에서 job을 실행하지 않는다. 전송 후 소스 해시를 확인하고 스냅샷을 고정한다. 기존 원격 트리에 `rsync --delete`를 적용하지 않는다.
- [ ] 출력은 source와 별도 디렉터리에 둔다. 소스/설정 해시, 환경 버전, 명령, job ID, 종료 코드, 입력 종류, 사용 seed, 결과를 회수한다.
- [ ] 큰 rosbag·데이터셋은 원격에 보관하고 필요한 로그·지표·대표 PNG/MP4만 회수한다. 나중에 데이터 재생을 추가하면 파일 해시로 참조한다.

제출 도구를 구현할 때는 전송 제외·기존 출력 거부·Slurm 실패 전파를 자동시험으로 먼저 확인하고, 마지막에 짧은 실제 원격 job으로 검증한다. 외부 명령 mock 통과를 원격 실행 성공으로 기록하지 않는다.

## 6. 화면 보기와 SSH 단절

초기 기본 경로는 **원격 headless 실행 → 결과 회수 → 노트북에서 PNG/MP4 열람**이다. `sbatch`로 제출된 작업은 노트북 SSH 연결 유지에 의존하지 않는다. 긴 실행을 편집기의 터미널 세션에만 매달지 않는다.

Remote SSH는 로그 확인·원격 디버깅에 사용한다. 실시간 3D 조작이 필요해지면 원격 데스크톱이나 웹 뷰어 하나를 추가로 선정하고 NVIDIA 렌더러를 직접 확인한다. 단순 X11 포워딩을 성능 검증으로 간주하지 않는다. 웹 뷰어를 쓰면 SSH 로컬 포트 포워딩으로 접근하며 공개 포트 개방을 전제로 하지 않는다.

## 7. ROS 2 Jazzy와 Gazebo Harmonic 통합

개발·원격 통합의 선택 조합은 **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**이다. [ROS 지원 플랫폼](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html), [Gazebo 공식 ROS 설치 안내](https://gazebosim.org/docs/harmonic/ros_installation/)

현재 MJCF 모델 검사를 먼저 원격에서 재현한다. 이후 RGB-D·LiDAR 출력, `ros_gz`·TF·`/clock`·rosbag·제어기 요구를 작은 장면에서 검사한 뒤 통합 엔진을 확정한다. URDF는 형상·운동학 교환용이므로 Gazebo의 접촉·관절 구동·센서·관성을 따로 검사한다. 모델 변환만으로 동역학이 같다고 간주하지 않는다.

원격에 기존 Isaac Sim/Isaac Lab 환경이 있으면 새로 설치하기 전에 별도 실행 검사 대상으로 둔다. 설치 디렉터리·버전 파일의 존재만으로 지게차 프로젝트에서 사용 가능하다고 결론내리지 않는다. 기존 연구 환경을 수정하지 않고 모델 가져오기·센서 출력·GPU 메모리 사용량을 비교한 뒤 재사용 여부를 정한다.

ROS 노드·시뮬레이터·RViz는 우선 원격 안에서 연결한다. 노트북과 원격 사이에 ROS DDS 발견·센서 토픽 전달을 필수로 만들지 않는다. 실제 로봇의 제어 루프는 온보드에서 실행한다. Jetson의 JetPack 7.2.1 / Ubuntu 24.04 / Jazzy 후보 조합은 D435i와 RPLIDAR를 ARM64에서 직접 검증한 뒤 동결한다.

ROS 컨테이너를 추가할 때 NVIDIA의 기본 `compute,utility`만으로 EGL이 된다고 가정하지 않는다. 렌더 작업은 `graphics` capability를 포함하고, Slurm이 할당한 GPU만 전달되도록 확인한다. 시스템 전체 GPU를 무조건 노출하는 `--gpus all`을 제출 도구 기본값으로 쓰지 않는다. [NVIDIA 런타임 설정](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html), [Slurm GPU 할당](https://slurm.schedmd.com/gres.html)

## 8. 권장 착수 순서

1. 노트북 도구 보강·Ruff 설정·ROS 2 Jazzy 개발 컨테이너 빌드와 현 구조의 editable 설치 확인.
2. 원격 전용 모델 환경·소스 스냅샷·CPU job으로 기존 검사 재현.
3. GPU 할당을 받아 EGL·81개 시험·미리보기 영상 생성과 회수.
4. 제출/결과 수집을 도구화하고 새 셸에서 다시 실행.
5. 별도 구조 전환, ROS/센서 통합, A–D 시나리오 개발을 각각 검증하며 진행.

이번 조사에서는 원격 실행 중인 작업, 드라이버, Slurm 설정, 기존 컨테이너·연구 환경을 변경하지 않았다. 원격 시뮬레이션 성능·렌더링·설치 성공은 아직 미검증이다.
