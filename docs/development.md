# 개발 환경

이 문서는 현재 root의 `forklift_core/` 배치를 유지한 채 노트북에서 코어를
개발하고, 격리된 ROS 2 Jazzy 컨테이너에서 ROS 명령을 확인하는 방법을 설명한다.
정적 Gazebo 장면과 ROS 센서 검증 패키지는 아래 별도 통합 이미지로 실행한다. 실제 장치 드라이버와 자율 주행은 아직 구현하지 않았다.

## 호스트 Python 개발

Python 3.10 이상 환경에서 저장소 루트를 editable로 설치한다. 코어 검사만
필요하면 `test`, 모델 생성·렌더 검사까지 필요하면 `model` extra를 함께 쓴다.

```bash
python -m pip install -e '.[dev]'
python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error
python -m forklift_core.demo
python -m ruff check .
python -m ruff format --check .
```

첫 pytest 명령은 코어 시험과 원격 제출 도구의 로컬 시험을 함께 실행한다. Ruff는
`pyproject.toml`의 제외 목록을 적용해 `forklift_core`, `tests`, `tools`뿐 아니라
`ros2/`, `sim/`의 Python도 검사한다. 경로를 일부만 지정하면 새 코드가 검사에서
빠진다.

모델 의존성이 필요한 개발 환경은 다음처럼 설치한다.

```bash
python -m pip install -e '.[dev,model]'
FORKLIFT_RENDER_TEST=1 python -m pytest tests/simulation -q \
  -p no:cacheprovider -W error
```

원격 실행 전이나 인계 전에는 렌더링 시험을 제외한 전체 호스트 회귀를 한 번에
실행한다. ROS 패키지의 `test/`는 ROS 없이 실행되는 메시지 계약 시험이며, 같은
파일을 Jazzy 컨테이너에서 colcon으로 다시 실행한 결과와 중복되므로 두 수를
합산하지 않는다.

```bash
python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q \
  -p no:cacheprovider -W error
```

Ruff lint 설정은 Python 3.10, 88열과 `E4`, `E7`, `E9`, `F`, `I`, `UP`, `B`
규칙을 적용한다. 규칙 전체를 숨기는 ignore는 없으며 lint와 format 검사를 모두
통과해야 한다.

## ROS 2 Jazzy 개발 이미지

컨테이너는 Ubuntu 24.04용 공식 `ros:jazzy-ros-base-noble` 이미지의 확인된
digest를 기본값으로 사용한다. Conda, Gazebo, Isaac Sim, CUDA, D435i/RPLIDAR
SDK는 포함하지 않는다. 호스트와 같은 UID/GID로 이미지를 빌드하면 bind mount에
생기는 파일 소유권이 유지된다.

```bash
docker build \
  -f deploy/ros2/Dockerfile \
  --build-arg DEV_UID="$(id -u)" \
  --build-arg DEV_GID="$(id -g)" \
  -t forklift/ros2-dev:jazzy .
```

다른 검증된 base image가 필요할 때만 `--build-arg ROS_BASE_IMAGE=<image>`를
추가한다. 이미지에는 빌드 시점의 코어 snapshot이 `/opt/forklift/venv`에
일반 설치된다. launcher로 실행하면 현재 checkout을 `/workspace`에 mount하고 그
위치에서 명령을 시작하므로, 개발 중인 `forklift_core/`가 snapshot보다 먼저
import된다. 실행할 때 editable 재설치 없이 checkout 코드를 우선 사용한다.
`/workspace` 밖에서 실행하면 이미지에 설치된 snapshot을 사용할 수 있으므로,
그 경로에서 최신 코드를 사용하려면 이미지를 다시 빌드한다.

저장소 어느 작업 디렉터리에서든 launcher를 실행할 수 있다.

```bash
bash tools/ros2_dev.sh ros2 doctor --report
bash tools/ros2_dev.sh python -m pytest tests --ignore=tests/simulation \
  -q -p no:cacheprovider -W error
bash tools/ros2_dev.sh python -m forklift_core.demo
```

ROS publish/subscribe 확인은 두 터미널에서 실행한다.

```bash
# terminal 1
bash tools/ros2_dev.sh ros2 run demo_nodes_cpp listener

# terminal 2
bash tools/ros2_dev.sh ros2 run demo_nodes_cpp talker
```

이미지 이름을 바꿨다면 실행할 때 환경변수로 지정한다.

```bash
FORKLIFT_ROS2_IMAGE=example/ros2-dev:jazzy \
  bash tools/ros2_dev.sh ros2 topic list
```

launcher는 컨테이너를 `--rm`, non-root, 기본 bridge network로 실행한다. host
network, privileged mode, Docker socket, host home은 사용하지 않으며, checkout
하나만 `/workspace`에 mount한다. USB 센서·GPU·GUI가 필요한 단계에서는 필요한
장치와 권한을 확인한 뒤 별도 실행 구성을 추가한다.

## 역할 경계

노트북은 편집, Ruff, 빠른 코어 시험과 ROS 명령 smoke test를 맡는다.
MuJoCo는 로컬 빠른 모델 검사에 유지하고, Gazebo 센서 관측·기록·재생은
아래 원격 Slurm 실행 경로를 사용한다.
이 컨테이너가 실제 센서 연결, 자율 제어, 시뮬레이터 선정 또는 Jetson 배포를
검증하지는 않는다.

## 원격 Slurm 모델·Gazebo 검사

원격 실행은 편집 중인 checkout을 직접 사용하지 않는다. 제출 도구가 명시된 소스에서
허용한 코어·도구·시험·모델·Gazebo/ROS 파일만 새 snapshot에 복사하고 SHA-256
manifest를 만든다. `.git`, `presentation`, `artifacts`, `data`, cache, 비밀명 파일과
심볼릭 링크는 전송하지 않는다. 같은 실행 ID의 snapshot·결과·job 기록이 하나라도
있으면 덮어쓰지 않고 실패한다.

호스트·원격 프로젝트 root·실행 환경은 항상 인자로 지정한다. 다만
`deploy/slurm/model_check.sbatch`는 Slurm `compute` partition과 원격 시스템
`/usr/bin/python3`(runner 실행용, Python 3.12 확인)을 전제하며 옵션화하지
않았다. 다른 클러스터에서는 이 두 값을 먼저 확인하고 배치 스크립트를 조정한다.
`model-cpu`와 `model-render`는 준비된 Python 3.11 interpreter를 사용한다. 현재 `model-render`는
OSMesa CPU 렌더링만 수행하며 NVIDIA GPU 검사가 아니다. `gazebo`는 제출 시 지정한
이미지 tag를 job 시작 때 immutable image ID로 해석해 기록하고 그 ID로 실행한다.

```bash
python tools/submit_model_check.py submit \
  --host <SSH_HOST> \
  --remote-root '<REMOTE_PROJECT_ROOT>' \
  --source . \
  --mode model-cpu \
  --python '<REMOTE_PYTHON>' \
  --run-id 20260910T150000Z_model_cpu_01

python tools/submit_model_check.py status \
  --host <SSH_HOST> --remote-root '<REMOTE_PROJECT_ROOT>' \
  --run-id 20260910T150000Z_model_cpu_01

python tools/submit_model_check.py collect \
  --host <SSH_HOST> --remote-root '<REMOTE_PROJECT_ROOT>' \
  --run-id 20260910T150000Z_model_cpu_01 \
  --output artifacts/20260910T150000Z_model_cpu_01
```

`submit --wait`는 종료 상태를 기다린 뒤 성공한 결과만 회수한다. `--output`을 생략하면
`<source>/artifacts/<run-id>`를 사용한다. 먼저 계획만 확인할 때는 `submit` 인자에
`--dry-run`을 추가한다. 이 모드는 SSH·rsync·디렉터리 생성 없이 JSON 계획만 출력한다.
SSH는 비대화형 연결, 10초 연결 제한과 keepalive를 사용하고 기존 SSH master를
재사용하지 않는다. 각 SSH 조회는 최대 60초, snapshot·결과 rsync는 최대 600초다.
transport timeout이면 이 CLI가 만든 process group만 종료하고 실패로 반환한다. 다른
SSH 세션·원격 job을 종료하거나 성공으로 판정하지 않는다.

Gazebo 제출은 `--mode gazebo --image <IMAGE> --duration 30`을 사용한다. Slurm job은
2 CPU와 4GiB를 요청한다. 컨테이너에는 Slurm 프로세스 affinity의 CPU 집합과 4GiB
memory/swap 상한을 그대로 적용하고, snapshot은 `/workspace` read-only,
실행별 결과는 `/output` read-write로만 mount한다. `job_result.json`에는 실제 명령,
종료 코드, 소스 해시, 선택한 Slurm 환경과 컨테이너 image ID가 남는다. `collect`는
`COMPLETED`, Slurm `0:0`, job 결과 `0`이 모두 확인된 경우에만 원격/로컬 파일 해시를
대조하고, 확인한 `scontrol` 원문도 함께 저장한다.


완료 상태를 오래 보관하지 않는 Slurm 설치에서는 `--wait`를 권장한다. accounting이
없고 `scontrol`의 작업 기록도 만료됐으면 이 도구는 성공 여부를 추정하지 않는다.
작업 종료 직후 `collect`로 상태 원문을 보존한다. 실패 작업의 로그는 원격
`artifacts/<run-id>/`와 `jobs/<run-id>-<job-id>.log`에 남으며 현재 `collect`는 성공
작업만 회수한다. 실패 진단 파일은 해당 실행 디렉터리에서 별도로 읽거나 복사한다.

## Gazebo 센서 통합 이미지와 실행

`deploy/gazebo/Dockerfile`은 ROS 2 Jazzy와 Gazebo Harmonic, bridge, rosbag2,
센서 검증 의존성만 설치한다. 소스는 실행마다 검증된 read-only snapshot으로
전달하므로 코드를 수정할 때 이미지를 다시 빌드할 필요는 없다. 의존성 변경 시에는
별도 이미지 빌드와 검증을 수행한다. 워크스테이션에서는 빌드도 Slurm CPU 할당
안에서 실행하고 Docker에 해당 CPU 집합과 메모리 제한을 적용한다.

```bash
# Run inside a 2-CPU / 4-GiB Slurm allocation.
TASK_CPUSET="$(python3 -c 'import os; print(*sorted(os.sched_getaffinity(0)), sep=",")')"
DOCKER_BUILDKIT=0 docker build \
  --cpuset-cpus "$TASK_CPUSET" --memory 4g --memory-swap 4g \
  -f deploy/gazebo/Dockerfile -t forklift/gazebo:jazzy-harmonic .

# From the laptop checkout, after the image exists on the selected host.
python tools/submit_model_check.py submit \
  --host <SSH_HOST> --remote-root '<REMOTE_PROJECT_ROOT>' \
  --source . --mode gazebo --image forklift/gazebo:jazzy-harmonic \
  --duration 30 --wait
```

`--duration 30`은 각 필수 센서 스트림이 관측해야 하는 simulation time의 최소
범위다. 초기화·기록 종료·재생·검증 때문에 실제 벽시계 실행 시간은 더 길다.
장면의 센서 위치·주기·노이즈 없는 영상은 합성 설정이며 실제 D435i/RPLIDAR의
교정값이나 성능을 나타내지 않는다. 장면과 검사 방법은
[`sim/gazebo/README.md`](../sim/gazebo/README.md), 확인한 실행 결과는
[센서 관측 검증 기록](validation/2026-09-10-gazebo-sensor-baseline.md)을 따른다.
