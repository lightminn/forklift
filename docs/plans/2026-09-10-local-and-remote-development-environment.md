# 노트북 개발 · 원격 워크스테이션 시뮬레이션 구성안

작성일: 2026-09-10. **상태(2026-09-11 갱신): 노트북 개발 환경과 ROS 2 Jazzy 컨테이너, 원격 팀 작업 공간·전용 모델 환경·Slurm 제출/회수 도구, 원격 Gazebo 정적 합성 센서 관측과 30초 이상 기록/재생 검증까지 완료. 최신 회귀 결과와 증거는 [개발 중간 정리](../validation/2026-09-11-development-checkpoint.md)를 따르며, 본문에 남긴 시험 개수는 각 실행 시점의 기록이다. GPU EGL·자율 주행·Jetson 실물 검증은 미완료다.**

사용자가 시뮬레이션을 별도 고성능 워크스테이션에서 실행하는 방향을 제안했다. 이 문서가 [기존 단일 머신 환경 계획](2026-09-10-local-development-environment.md)의 실행 위치와 우선순위를 대체한다. 기존 계획의 의존성 보존·wheel 검사 원칙은 유지한다. 호스트명·SSH 별칭·개인 경로·설치 사양과 실행 당시 대기열은 개인 환경 기록에 둔다.

**목표:** 노트북에서 편집·빠른 검사를 하고, 원격에서 재현 가능한 모델 검사·렌더링·장시간 실험을 실행하며 결과를 노트북으로 가져온다.

**선택한 구성:** 노트북의 Ubuntu 24.04 / ROS 2 Jazzy 개발 컨테이너 + 원격 Gazebo Harmonic 통합 시뮬레이션 + 기존 MuJoCo 빠른 모델 검사. 원격은 시뮬레이션·렌더링·학습을 맡고 실제 제어는 온보드에서 실행한다. Jetson 후보는 JetPack 7.2.1 / Ubuntu 24.04 / Jazzy이며 D435i 실물 검증 전에는 최종 버전으로 동결하지 않는다.

## 1. 역할 분담

| 위치 | 맡는 일 | 환경 |
|---|---|---|
| 노트북 | 코드·모델 설정 편집, Ruff, 코어 시험, ROS 2 개발, 간단한 모델 자세 확인, 결과 열람 | 호스트 편집 도구 + Ubuntu 24.04 / ROS 2 Jazzy 컨테이너 |
| 원격 CPU 작업 | 모델 생성, 비렌더링 시험, 향후 물리 적분·파라미터 반복 시험 | 프로젝트 전용 Python, Slurm CPU 자원 |
| 원격 GPU 작업 | EGL 렌더링, 향후 RGB-D 렌더·영상 인식 등 GPU가 필요한 작업 | 같은 모델 환경, Slurm GPU 할당 |
| 원격 ROS 통합 | 센서 메시지·TF·rosbag·SLAM·계획/제어 연결과 장시간 실험 | Ubuntu 24.04 / ROS 2 Jazzy + Gazebo Harmonic 센서 기준선 확인 |
| 향후 Jetson | 실제 D435i/RPLIDAR·구동기와 연결되는 상위 제어 | Orin Nano Super / JetPack 7.2.1 후보, 실물 ARM64 검증 후 동결 |

현재 모델 검사는 노트북에서도 짧게 실행할 수 있다. 원격 GPU 대기열이 길 때 이 경로를 유지한다. 현재 `mujoco.mj_step` 검사에는 GPU 요청이 필요하지 않으며 EGL 렌더링 작업과 구분한다. 여러 물리 실험을 GPU에서 자동 병렬화하는 MJX/Warp 구현은 현재 없다.

코어 알고리즘과 센서 관측·명령 계약은 두 머신에서 같은 코드를 사용한다. 원격 시뮬레이터가 미래 실물 로봇의 실시간 제어 루프를 네트워크 너머에서 맡도록 설계하지 않는다.

### 1.1. 학기·기업·팀 단위의 공용 작업 공간

2026-09-10 사용자 최종 정정: 이 프로젝트는 **2026-2 임베설의 리보틱스 팀** 작업이며 워크스테이션은 다른 팀도 사용한다. 사용자가 제공한 [과목 공유 Drive](https://drive.google.com/drive/folders/1NUFh5lr1UhUKqeia3k8B6X_eoAgmFgGj)의 분류를 원격 작업 공간에도 맞춘다.

같은 날 로그인된 브라우저에서 확인한 최상위 폴더와 우리 과제의 하위 구조는 다음과 같다. 다른 기업 폴더 내부는 이번 확인 범위에 포함하지 않았다.

```text
2026 2학기 임베설/
├── 다우테크놀로지/
├── 리보틱스/
│   └── 발표자료/
│       └── 2주차 발표자료.txt
├── 장자동화 + 두루기계/
├── 파워크래프트(매트릭스배터리)/
├── 파워크래프트(태양광)/
├── 팜테크/
└── 플라즈마 큐어링/
```

**원격 배치 완료 — 2026-09-10:** 사용자 승인 후 기업·과제 폴더 7개를 Drive와 같은 이름으로 생성했다. 팀명 정정에 따라 중간 팀 계층을 제거하고 `리보틱스/forklift/`를 우리 실행 공간으로 사용한다. Drive의 `리보틱스/` 폴더가 이미 우리 팀 분류이므로 팀 이름을 중복해서 넣지 않는다. 실행용 하위 폴더는 서버에만 추가했다.

```text
<COURSE_ROOT>/                       # Drive의 "2026 2학기 임베설"에 대응
└── 리보틱스/
    ├── 발표자료/                    # Drive와 같은 분류; 공유용 결과·링크
    └── forklift/                    # 팀 프로젝트 작업 공간
        ├── repo/                   # 실행 준비용 Git checkout
        ├── snapshots/              # 제출할 때 고정한 소스
        ├── artifacts/              # 실행별 로그·지표·PNG·MP4
        ├── data/                   # 원본 rosbag·데이터셋·가중치
        └── cache/                  # 다시 생성할 수 있는 캐시
```

- 과목 공용 루트의 실제 절대 경로·서버 계정·그룹 이름은 환경 기록에서 지정한다. 원격 제출 도구는 프로젝트 루트를 인자로 받고 개인 홈이나 팀 경로를 코드에 고정하지 않는다.
- 공용 과목 루트와 아직 배정하지 않은 다른 기업 폴더는 관리자 소유다. 우리 팀의 `리보틱스/`에는 팀 그룹·setgid·기본 ACL을 적용했다. 팀명에 맞춰 그룹 이름을 정정하되 GID와 기존 구성원은 유지한다. 새 SSH 연결의 그룹 적용과 보수적인 umask에서도 새 파일의 팀 그룹·공동 쓰기 권한이 상속되는 것을 검증한다. 다른 팀의 구성원·쓰기 권한은 아직 배정하지 않았다. 개인 개발 checkout·Python 환경과 버전을 고정한 팀별 컨테이너 구성은 후속 단계다.
- 초기 `repo/`는 노트북의 Git bundle에서 복제하고 origin을 기존 GitHub 저장소로 지정했다. 초기 commit 위의 미커밋 팀·폴더 문서 변경도 전달했다. commit과 원격 파일 해시를 검증했으며, 원격 GitHub 인증·fetch는 이번 검사에 포함하지 않았다. 이는 실행별 스냅샷 생성·자동 동기화 도구의 구현 완료를 뜻하지 않는다.
- 실행은 §5의 새 소스 스냅샷에서 수행한다. 결과는 기존 컨벤션대로 `artifacts/<UTC시각>_<시나리오>_<구분자>/`에 남기고 작성자·팀·job ID를 함께 기록한다. 팀별 결과 보관 기간과 용량 기준은 공용 운영 설정 단계에서 정한다.
- 이 계층은 저장소 **밖의 과목 작업 공간**이다. 코드 저장소 내부의 폴더·네이밍은 `CONTRIBUTING.md`를 유지한다. 실행 도구는 한글·공백·괄호가 있는 상위 경로를 지원하도록 인자 전달과 셸 인용을 검증한다.
- Drive와 동일한 분류를 쓰는 것이 자동 파일 동기화를 뜻하지는 않는다. 로봇 코드 정본은 Git, 발표 소스 정본은 별도 발표 저장소이며 Drive에는 선택한 공유 자료·링크를 둔다. 이번 원격 폴더 구성에서는 Drive의 폴더·파일, 기존 연구 환경, Slurm 설정과 실행 중인 작업을 변경하지 않았다.

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

**구현한 파일:** `requirements/model_py311.txt`, `requirements/README.md`, 공통 배치 `deploy/slurm/model_check.sbatch`와 `tools/remote_model_job.py`. CPU 시험·CPU 렌더·Gazebo는 명시적인 mode로 구분한다. GPU mode는 아직 추가하지 않았다. Python 의존성 정본은 `pyproject.toml`이고 lock은 그중 `test`·`model` extra에서 생성한다. 개인 전체 `pip freeze`를 프로젝트 lock으로 쓰지 않는다.

- [x] Python 3.11.16 전용 환경에 MuJoCo 3.10.0과 `test`·`model` 의존성을 설치했다. Linux x86_64/Python 3.11 대상 wheel 16개를 SHA-256으로 고정해 실행 기록에 보관했다. 환경 생성 위치는 개인 환경 기록에 둔다.
- [x] 첫 소스 스냅샷 21개 파일을 고정하고 설치 후 `pip check`, source 밖 wheel import, 모델 생성·로딩 시험을 통과했다.
- [x] 첫 CPU job은 가용 자원에 맞춰 **2 CPU·4GiB·10분**을 요청했다. `python -m pytest tests -m 'not rendering' -q -p no:cacheprovider -W error` 결과 **80개 통과·1개 deselect, skip 0개**다. 같은 자원으로 20초 물리 적분과 별도 4초 OSMesa 영상도 생성했다. 자원 요청은 후속 실행의 실제 사용량에 맞춰 조정한다.
- [ ] GPU job은 4 CPU·8GiB·GPU 1개·10분으로 전체 시험과 아래 미리보기를 실행한다. 자원 요청값은 실행 후 최대 사용량에 맞춰 조정한다.

첫 테스트런의 GPU job은 2 CPU·4GiB·GPU 1개·5분으로 제출했으나, 예상 시작이 실행 기한을 넘어 미실행 상태로 취소했다. NVIDIA EGL·전체 81개 시험 조건은 미완료다. [원격 검증 기록](../validation/2026-09-10-remote-model-smoke.md)에 실제 종료 상태와 증거를 남겼다. 첫 테스트런의 일회성 도구는 당시 `artifacts/`에 보존했다. 이후 lock과 공통 배치를 저장소에 추가하고 아래 표준 제출 도구로 새 CPU 작업을 실행·회수했다.

```bash
FORKLIFT_RENDER_TEST=1 python -m pytest tests -q -p no:cacheprovider -W error
python tools/preview_forklift_model.py \
  --model sim/models/dls08_provisional/scene.xml \
  --output "$FORKLIFT_OUTPUT_DIR" --backend egl --frames 96
```

`FORKLIFT_OUTPUT_DIR`는 제출 시 정한 해당 실행의 새 결과 디렉터리다. job은 비어 있는 변수, 기존 결과 디렉터리, 없는 checkout·Python 실행 경로를 시작 전에 거부한다. GPU 렌더러 문자열·드라이버·설정/소스 해시를 결과에 남겨 CPU 소프트웨어 렌더링으로 바뀌지 않았는지 확인한다. `sbatch --parsable`의 job ID, Slurm 종료 상태와 exit code, 실제 pytest/영상 출력을 함께 확인한다. Accounting이 비활성화된 대상에서는 `sacct` 대신 `scontrol show job`을 완료 직후 확인한다.

**첫 완료 조건:** 원격에서 81개 시험 통과, 실제 NVIDIA EGL 렌더 확인, PNG 및 4초 MP4 생성, 노트북으로 결과 회수·직접 확인. SSH 접속·GPU 목록 조회는 이 완료 조건을 대신하지 않는다.

## 5. 소스와 결과의 전달

**현재 도구:** `tools/submit_model_check.py`의 `submit`, `status`, `collect`를 사용한다. `model-cpu`, `model-render`(CPU OSMesa), `gazebo` mode와 SSH host·remote root·Python 또는 image를 명시한다. 개인 기본 SSH 별칭이나 경로는 코드에 고정하지 않는다. 실행 ID를 생략하면 UTC 기반 새 ID를 만든다. `--dry-run`은 원격 접속·파일 쓰기 없이 계획을 출력한다.

- 실행마다 허용 목록의 일반 파일만 새 원격 snapshot에 보내고, 파일별 SHA-256과 전체 manifest를 검증해 read-only로 고정한다. `.git`, 발표 링크, data, 기존 artifacts, cache, 비밀명 파일과 symlink는 제외한다.
- 편집 중인 공유 checkout에서 job을 실행하지 않으며 기존 snapshot·output·job record를 덮어쓰지 않는다. Git revision과 미커밋 snapshot 해시는 별도로 기록한다. 이후 commit/push는 사용자가 승인한 범위에서만 수행한다.
- 출력은 source와 별도 디렉터리에 둔다. 실제 명령·job ID·종료 코드·source hash·image ID와 관측 결과를 보존한다. Slurm 완료 상태와 결과 exit code를 함께 확인하고 회수 파일 SHA-256을 대조한다.
- 현재 collector는 성공 실행의 전체 결과를 회수한다. 30초 통합 검사의 bag도 회수 대상이다. 큰 실험 데이터에 대한 선택 회수는 후속 요구가 생기면 별도로 추가한다.

실제 Slurm 970의 CPU 100개 시험과 수정 후 새 Slurm 972의 102개 시험·결과 회수까지 확인했다. 제출 도구의 추가 오류 처리 시험은 [센서 관측 검증 기록](../validation/2026-09-10-gazebo-sensor-baseline.md)을 따른다. 실패 작업 로그는 원격에 보존하며 성공 회수와 구분한다. 명령은 [개발 안내](../development.md#원격-slurm-모델gazebo-검사)를 따른다.

## 6. 화면 보기와 SSH 단절

초기 기본 경로는 **원격 headless 실행 → 결과 회수 → 노트북에서 PNG/MP4 열람**이다. `sbatch`로 제출된 작업은 노트북 SSH 연결 유지에 의존하지 않는다. 긴 실행을 편집기의 터미널 세션에만 매달지 않는다.

Remote SSH는 로그 확인·원격 디버깅에 사용한다. 실시간 3D 조작이 필요해지면 원격 데스크톱이나 웹 뷰어 하나를 추가로 선정하고 NVIDIA 렌더러를 직접 확인한다. 단순 X11 포워딩을 성능 검증으로 간주하지 않는다. 웹 뷰어를 쓰면 SSH 로컬 포트 포워딩으로 접근하며 공개 포트 개방을 전제로 하지 않는다.

## 7. ROS 2 Jazzy와 Gazebo Harmonic 통합

개발·원격 통합의 선택 조합은 **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**이다. [ROS 지원 플랫폼](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html), [Gazebo 공식 ROS 설치 안내](https://gazebosim.org/docs/harmonic/ros_installation/)

MJCF 원격 재현 후 정적 장면의 RGB-D·LiDAR, `ros_gz`·TF·`/clock`·rosbag 기록과 새 프로세스 재생을 확인했다. [센서 관측 검증 기록](../validation/2026-09-10-gazebo-sensor-baseline.md)에 확인 범위를 남긴다. 제어기 연결은 후속 작업이다. URDF는 형상·운동학 교환용이므로 Gazebo의 접촉·관절 구동·센서·관성을 따로 검사한다. 모델 변환만으로 동역학이 같다고 간주하지 않는다.

원격에 기존 Isaac Sim/Isaac Lab 환경이 있으면 새로 설치하기 전에 별도 실행 검사 대상으로 둔다. 설치 디렉터리·버전 파일의 존재만으로 지게차 프로젝트에서 사용 가능하다고 결론내리지 않는다. 기존 연구 환경을 수정하지 않고 모델 가져오기·센서 출력·GPU 메모리 사용량을 비교한 뒤 재사용 여부를 정한다.

ROS 노드·시뮬레이터·RViz는 우선 원격 안에서 연결한다. 노트북과 원격 사이에 ROS DDS 발견·센서 토픽 전달을 필수로 만들지 않는다. 실제 로봇의 제어 루프는 온보드에서 실행한다. Jetson의 JetPack 7.2.1 / Ubuntu 24.04 / Jazzy 후보 조합은 D435i와 RPLIDAR를 ARM64에서 직접 검증한 뒤 동결한다.

ROS 컨테이너를 추가할 때 NVIDIA의 기본 `compute,utility`만으로 EGL이 된다고 가정하지 않는다. 렌더 작업은 `graphics` capability를 포함하고, Slurm이 할당한 GPU만 전달되도록 확인한다. 시스템 전체 GPU를 무조건 노출하는 `--gpus all`을 제출 도구 기본값으로 쓰지 않는다. [NVIDIA 런타임 설정](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html), [Slurm GPU 할당](https://slurm.schedmd.com/gres.html)

## 8. 권장 착수 순서

1. 노트북 도구 보강·Ruff 설정·ROS 2 Jazzy 개발 컨테이너 빌드와 현 구조의 editable 설치 확인.
2. 원격 전용 모델 환경·소스 스냅샷·CPU job으로 기존 검사 재현.
3. GPU 할당을 받아 EGL·81개 시험·미리보기 영상 생성과 회수.
4. 제출/결과 수집을 도구화하고 새 셸에서 다시 실행.
5. 별도 구조 전환, ROS/센서 통합, A–D 시나리오 개발을 각각 검증하며 진행.

첫 원격 테스트런에서는 새 프로젝트 전용 환경과 작업만 추가했다. 기존 연구 작업·드라이버·Slurm 설정·컨테이너는 변경하지 않았다. MuJoCo CPU 물리 계산·소프트웨어 렌더링과 Gazebo 합성 센서 전달·재생을 확인했다. GPU·실물 센서·자율주행 성능은 아직 미검증이다.
