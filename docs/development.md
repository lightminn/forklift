# 개발 환경

이 문서는 `src/forklift_core/` 코어와 `examples/` 예제, `tests/unit/`·
`tests/integration/` 시험 배치를 기준으로 노트북 개발과 격리된 ROS 2 Jazzy
컨테이너에서의 실행 방법을 설명한다. 실행 전 대상 Python 환경에 editable로 설치한다.
정적 Gazebo 장면과 ROS 센서 검증 패키지는 아래 별도 통합 이미지로 실행한다. 실제 장치 드라이버와 자율 주행은 아직 구현하지 않았다.

2026-09-11 구조 전환의 호스트·wheel·재빌드한 개발 컨테이너·원격 Slurm(model-cpu,
gazebo) 동등성 검사는 [구조 전환 검증 기록](validation/2026-09-11-src-layout-migration.md)에
있다. 이전 날짜의 컨테이너·원격 결과는 당시 배치의 기록이다.

## 새 checkout에서 시작하기

Python 3.10 이상 환경에서 다음 순서로 설치하고 로컬 코어를 확인한다.

```bash
git clone https://github.com/lightminn/forklift.git
cd forklift
python -m pip install -e '.[dev]'
python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error
python examples/sensor_geometry.py
python tools/submit_model_check.py submit \
  --host <SSH_HOST> --remote-root '<REMOTE_PROJECT_ROOT>' \
  --source . --mode model-cpu --python '<REMOTE_PYTHON>' --dry-run
```

마지막 명령은 SSH 없이 제출 계획만 출력한다. 실제 원격 사용 권한과 환경은 팀
계정을 배정받은 뒤 별도로 확인한다.

## 호스트 Python 개발

Python 3.10 이상 환경에서 저장소 루트를 editable로 설치한다. 코어 검사만
필요하면 `test`, 모델 생성·렌더 검사까지 필요하면 `model` extra를 함께 쓴다.

```bash
python -m pip install -e '.[dev]'
python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error
python examples/sensor_geometry.py
python -m ruff check .
python -m ruff format --check .
```

첫 pytest 명령은 코어 시험과 원격 제출 도구의 로컬 시험을 함께 실행한다. Ruff는
`pyproject.toml`의 제외 목록을 적용해 `src/`, `examples/`, `tests/`, `tools/`뿐 아니라
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

## 포켓 인식 평가 실행

합성 장면 데이터 세트 v1의 RGB·깊이 입력으로 포켓을 인식하고, 정답과 비교한
지표·장면별 진단·overlay를 저장한다. `.[dev]`에 포함된 NumPy, Pillow, PyYAML이
필요하고 MP4 생성에는 별도의 ffmpeg가 필요하다. 데이터 세트는 **Git 밖**에서
관리하므로 clone만으로 `data/synthetic_scenes/catalogue_v1/`이 생기지 않는다.
세트를 별도로 준비하고 원본은 읽기 전용으로 유지한다.
[데이터 세트 계약](interfaces/scene-dataset.md)과
[포켓 인식 설계](design/2026-09-13-pocket-detector-baseline.md)를 함께 따른다.

파라미터와 문턱은 **dev에서만 튜닝하고 eval은 확정 후 한 번** 실행한다.
아래 명령은 실행 예시이며, 데이터 세트 성능 검증이 끝났다는 뜻이 아니다.

```bash
# Development: repeated runs are allowed, with a fresh output directory each time.
python tools/evaluate_pocket_detector.py \
  --dataset data/synthetic_scenes/catalogue_v1 --split dev \
  --prior config/pallet_prior_v1.yaml \
  --output "artifacts/$(date -u +%Y%m%dT%H%M%SZ)_pocket_eval_dev_01"

# Optional: inspect selected dev scenes using explicit parameter overrides.
python tools/evaluate_pocket_detector.py \
  --dataset data/synthetic_scenes/catalogue_v1 --split dev \
  --prior config/pallet_prior_v1.yaml --params config/detector_params_v1.yaml \
  --scenes s003,s006,s010 \
  --output "artifacts/$(date -u +%Y%m%dT%H%M%SZ)_pocket_eval_dev_02"

# Final evaluation: run once, only after the freeze gate described below.
python tools/evaluate_pocket_detector.py \
  --dataset data/synthetic_scenes/catalogue_v1 --split eval \
  --prior config/pallet_prior_v1.yaml --params config/detector_params_v1.yaml \
  --video \
  --output "artifacts/$(date -u +%Y%m%dT%H%M%SZ)_pocket_eval_eval_01"
```

`config/detector_params_v1.yaml`은 dev에서 선택한 뒤 준비하는 파일이다. 아직 없으면
첫 dev 명령처럼 `--params`를 생략하여 기본값을 사용한다. YAML에는 `DetectorParams`의
필드만 허용하며, 생략한 필드는 기본값으로 채운다. eval 전에는 기본값을 선택했더라도
유효 파라미터 전체를 YAML로 저장하고 같은 파일을 dev에서 다시 읽어 확인한다.
구현·시험·prior·params를 커밋해 revision을 고정하고 깨끗한 트리에서 같은 `--params`로
eval 30장면 전체를 한 번 실행한다. eval 결과를 보고 파라미터를 바꾸면 그 실행은 최종
보고가 아니며 검증 기록에 그 사실을 남긴다. 상세 동결 절차는
[실행 계획의 Task 5.5](plans/2026-09-13-pocket-detector-evaluation-run.md)를 따른다.

`--split`은 `dev` 또는 `eval`만 받는다. `--scenes`는 선택한 split의 ID만 허용하며,
빈 선택·없는 ID·다른 split의 ID를 거부한다. 장면은 ID 오름차순으로 평가한다.
출력 디렉터리가 이미 있으면 덮어쓰지 않고 실패하므로 새 UTC 시각 또는 번호를 쓴다.

| 산출물 | 내용 |
|---|---|
| `metrics.json` | `evaluation.summarize`의 범주별 개수·검출률·오차·처리 시간·목표 도달 여부와 `run` 재현 정보 |
| `scenes.csv` | 장면당 18열: 상태·판정·좌우/최대 위치 오차·yaw 오차·사유·시간·평면 진단·좌우 광선 비율 |
| `observations/sNNN.json` | 추정 `PocketObservation`과 진단, 내부 예외의 `diagnostics.exception_traceback` |
| `overlay/sNNN.png` | 초록색 정답·자홍색 추정 사각형과 상태·오차 글자. `--no-overlay`이면 생략 |
| `overlay.mp4` | `--video`일 때 PNG를 5 fps로 묶은 독립 장면 모음. 연속 관측이 아니라는 표시 포함 |
| `run.json` | 입력 경로·manifest/prior SHA-256·split·실제 장면 ID/개수·기본값과 seed를 포함한 params·판정 허용오차·Git revision/dirty·UTC 시작/종료·Python/NumPy 버전. `metrics.json["run"]`과 동일 |

광선 진단이 없으면 CSV는 빈칸, JSON의 비율은 `null`이다. 없는 값을 0 %로 만들지
않는다. CLI가 직접 잡은 장면별 예외의 진단에는 traceback 키만 있을 수 있다.
인식·평가 예외는 해당 장면을 `invalid`로 남기고 다음 장면을 계속 처리한다.
설정·입력 오류, 출력 디렉터리 중복, 필수 산출물 저장 실패는 비정상 종료하며,
실행 중 도구 오류는 stderr JSON의 같은 `diagnostics.exception_traceback` 경로에 남긴다.
목표 미달과 MP4 생성 실패는 정상 종료한다. MP4 실패 사유는 `run.json["video_error"]`에
남고, 저장된 PNG로 인식기를 재실행하지 않고 다시 영상을 만들 수 있다.
`--video --no-overlay` 조합도 영상 입력이 없다는 `video_error`를 기록한다.

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
추가한다. Dockerfile은 코어를 `/workspace/src`에 복사한 뒤 `/opt/forklift/venv`에
`/workspace`를 editable로 설치한다. launcher는 현재 checkout을 `/workspace`에
mount하므로 `/tmp` 등 다른 작업 디렉터리에서도 mount된 소스를 import한다.
코드 편집은 이미지 재빌드 없이 반영된다. mount 없이 실행하면 이미지에 복사된
빌드 시점의 소스를 사용한다.

이 구성은 기본 editable 모드와 단순한 `src/` 구조에서 정적 `.pth`가
`/workspace/src`를 가리키는 방식에 의존한다. strict editable 모드는 사용하지
않는다. 설치 metadata(버전·의존성)는 이미지 빌드 시점에 고정되므로
`pyproject.toml`을 바꾸면 이미지를 다시 빌드하고 import 경로를 확인한다.

저장소 어느 작업 디렉터리에서든 launcher를 실행할 수 있다.

```bash
bash tools/ros2_dev.sh ros2 doctor --report
bash tools/ros2_dev.sh python -m pytest tests --ignore=tests/simulation \
  -q -p no:cacheprovider -W error
bash tools/ros2_dev.sh python examples/sensor_geometry.py
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

model 모드는 검증한 snapshot의 소스를 쓰기 가능한 실행별 복사본에 옮겨 wheel을
만들고, `--system-site-packages` venv에 설치한 Python으로 실행한다. snapshot은
읽기 전용으로 유지한다. `job_result.json`의 `core_environment`에는 wheel 경로와
SHA-256, 실제 import 경로, 실행 Python과 venv의 pip·setuptools 버전을 남긴다.
wheel은 결과의 `wheels/`에 보존하고 실행별 `.runtime/`은 종료 시 삭제한다.

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

원격 `artifacts/` 보관 기준(2026-09-13 기본값 채택): 성공 실행은 로컬 회수·검증 기록 후 30일 뒤 원격 사본을 지울 수 있고, 실패 실행은 원인을 검증 기록에 남긴 뒤 지운다. `snapshots/`는 검증 기록이 참조하는 실행의 것만 유지한다. 삭제 전 로컬 회수본의 manifest 해시가 있는지 확인한다.

### 장면 batch 제출과 병합 (`scenes`)

`scenes`는 카탈로그 범위마다 RGB-D 한 세트씩 캡처한다. 제출 측 Python과 원격
wrapper의 `/usr/bin/python3`에는 카탈로그 검사용 PyYAML이 필요하다. 캡처는 위의
Jazzy/Harmonic 이미지에서 실행하며 코어 패키지 설치는 필요하지 않다.

```bash
python tools/submit_model_check.py submit \
  --host <SSH_HOST> --remote-root '<REMOTE_PROJECT_ROOT>' \
  --source . --mode scenes --image forklift/gazebo:jazzy-harmonic \
  --catalogue sim/gazebo/scenes/catalogue_v1.yaml \
  --scene-range s001-s025 --time-limit 01:30:00 \
  --run-id scenes-v1-batch-01 --wait \
  --output artifacts/scenes-v1-batch-01
```

먼저 같은 명령에 `--dry-run`을 붙이면 JSON에서 `catalogue`, `scene_range`,
`time_limit`을 확인할 수 있다. `--catalogue`는 `--source` 기준 정규화된 상대
POSIX 경로여야 하며 실제 snapshot 파일 목록에 포함돼야 한다. 절대·숨김·`..`·
심볼릭 링크 경로, 미존재 파일은 거부한다. `--scene-range`는 양끝을 포함하는
`sNNN-sMMM` 형식이며 시작 ≤ 끝이고 범위의 모든 ID가 카탈로그에 존재해야 한다.

`scenes`의 기본 Slurm 제한은 `01:30:00`이며 `--time-limit`으로 바꾼다. 기존
model/gazebo mode는 이 옵션을 생략하면 스크립트의 `00:20:00`을 그대로 쓴다.
`--duration`은 기존 gazebo 관측용이며 scenes의 장면 수·제한 시간을 조정하지 않는다.
원격 runner는 검증된 snapshot digest, inspect한 immutable image ID, 제출 run ID를
캡처 노드와 batch manifest까지 전달한다. sbatch의 기존 mode는 6개, scenes는
카탈로그·범위를 더한 8개 위치 인자를 쓴다.

아래 25개씩 4 batch는 실행 예시다. 먼저 `s001-s002` spike를 별도 run ID로 실행하고
`wall_times_s`를 측정해 batch 크기와 `--time-limit`을 확정한다. 2장면 spike 결과를
아래 전체 세트에 함께 넣으면 ID가 중복되므로 병합 입력에서 제외한다.

| run ID 예시 | 범위 | 회수 디렉터리 |
|---|---|---|
| `scenes-v1-batch-01` | `s001-s025` | `artifacts/scenes-v1-batch-01` |
| `scenes-v1-batch-02` | `s026-s050` | `artifacts/scenes-v1-batch-02` |
| `scenes-v1-batch-03` | `s051-s075` | `artifacts/scenes-v1-batch-03` |
| `scenes-v1-batch-04` | `s076-s100` | `artifacts/scenes-v1-batch-04` |

모든 batch는 같은 소스 snapshot·카탈로그 바이트·카메라 설정·immutable image ID를
사용해야 한다. 제출 사이 소스를 수정하거나 이미지 태그의 대상을 바꾸지 않는다.
runner는 장면 실패를 기록하고 다음 장면을 시도하지만 batch 전체는 nonzero로 끝난다.
**실패 batch 전체를 같은 범위·새 run ID로 재제출**한다. 실패 ID만 재제출하거나
원래 batch에서 성공 장면만 병합하지 않는다. `collect`에는 부분 회수 규약이 없으며
실패 batch는 병합 입력에서 제외한다. 예를 들어 batch-02 재실행이 성공하면 아래
입력 중 batch-02 경로를 새 실행의 회수 경로로 바꾼다.

성공한 batch를 모두 회수한 뒤, 코어·Pillow·PyYAML이 설치된 로컬 환경에서 실행한다.

```bash
python tools/merge_scene_batches.py \
  --catalogue sim/gazebo/scenes/catalogue_v1.yaml \
  --batches artifacts/scenes-v1-batch-01 artifacts/scenes-v1-batch-02 \
            artifacts/scenes-v1-batch-03 artifacts/scenes-v1-batch-04 \
  --output data/synthetic_scenes/catalogue_v1
```

병합 도구는 카탈로그의 모든 ID가 중복·누락 없이 한 번씩 있는지, batch·장면 메타데이터와
GT가 정합한지, 파일 해시와 실제 `load_scene_sample` 로딩이 통과하는지 확인한다.
기존 출력은 덮어쓰지 않는다. 세트에는 검증된 장면 파일 8개씩과 집계 manifest를
복사하며 원래 world·로그는 회수 디렉터리에 남는다. 파일 형식과 manifest 필드는
[장면 데이터 세트 생산자 계약](interfaces/scene-dataset.md#캡처-생산자와-batch세트-manifest),
캡처 순서는 [Gazebo 캡처 절](../sim/gazebo/README.md#장면-캡처)을 따른다.

이 명령 안내와 호스트 시험은 원격 캡처 성공이나 100개 데이터 완성을 뜻하지 않는다.
실제 Gazebo/ROS 실행, 2장면 시간·반복 해시 측정, 100장면 생성과 시각 검토는
[계획 Task 7](plans/2026-09-11-scene-capture-and-remote.md#task-7-검증실행-claude)에서 수행한다.

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
