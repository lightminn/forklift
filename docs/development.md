# 개발 환경

이 문서는 현재 root의 `forklift_core/` 배치를 유지한 채 노트북에서 코어를
개발하고, 격리된 ROS 2 Jazzy 컨테이너에서 ROS 명령을 확인하는 방법을 설명한다.
ROS 패키지·장치 드라이버·통합 시뮬레이터는 아직 이 저장소에 구현되지 않았다.

## 호스트 Python 개발

Python 3.10 이상 환경에서 저장소 루트를 editable로 설치한다. 코어 검사만
필요하면 `test`, 모델 생성·렌더 검사까지 필요하면 `model` extra를 함께 쓴다.

```bash
python -m pip install -e '.[dev]'
python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error
python -m forklift_core.demo
python -m ruff check forklift_core tests tools
python -m ruff format --check forklift_core tests tools
```

모델 의존성이 필요한 개발 환경은 다음처럼 설치한다.

```bash
python -m pip install -e '.[dev,model]'
FORKLIFT_RENDER_TEST=1 python -m pytest tests/simulation -q \
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
MuJoCo는 로컬 빠른 모델 검사에 유지하고, 향후 장시간 렌더링과 Gazebo 통합
시뮬레이션은 별도 원격 실행 계획을 따른다.
이 컨테이너가 실제 센서 연결, 자율 제어, 시뮬레이터 선정 또는 Jetson 배포를
검증하지는 않는다.
