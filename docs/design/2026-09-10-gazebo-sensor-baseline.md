# Gazebo 센서 관측 기준선

사용자가 2026-09-10 승인한 목표: 원격 실행 절차 정식화, 고정 지게차·팔레트·장애물의 Gazebo 최소 장면, RGB-D·2D LiDAR의 ROS 2 전달과 30초 기록·재생·좌표 검증.

## 범위와 결정

- Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic을 별도 Docker 이미지로 사용한다. 기존 연구 환경·Slurm 설정·작업은 유지한다.
- MuJoCo 검사와 Gazebo 통합 검사는 각각 실행·보고한다. CPU 소프트웨어 렌더링을 기본으로 하고 NVIDIA 모드는 Slurm GPU 할당을 확인해야 한다.
- 차체는 현재 잠정 URDF의 시각 형상을 고정 배치한다. 주행·포크 구동·하중 검증은 추가하지 않는다. 팔레트와 센서 위치는 명시적인 합성 설정이다.
- 센서는 범용 RGB-D와 2D scan을 사용한다. D435i/RPLIDAR 드라이버·노이즈·실제 장착 보정·SKU 충실도를 주장하지 않는다.
- 기존 `forklift_core/` 구조를 보존하며 ROS/SDK 의존성을 코어에 추가하지 않는다. 이번 작업에 `src/` 마이그레이션을 섞지 않는다.

## 원격 실행 계약

`tools/submit_model_check.py`는 명시적 SSH host·project root·모드·Python 또는 컨테이너 이미지·실행 ID를 받는다. 기본 ID는 UTC 시각을 사용한다. `--dry-run`은 원격 접속이나 파일 쓰기 없이 계획을 JSON으로 출력한다.

소스는 명시적으로 허용한 프로젝트 경로에서 새 스냅샷으로 복사한다. `.git`, 발표 링크, 캐시, 데이터, 비밀 파일은 제외하고 심볼릭 링크를 따라가지 않는다. 파일 SHA-256과 revision/dirty 상태를 기록한다. 기존 출력·스냅샷을 재사용하지 않는다. 원격 root·한글·공백은 안전한 인자로 전달한다.

Slurm에서 2 CPU·4GiB를 기본 요청한다. Docker 실행도 같은 CPU 집합과 메모리를 제한하고 source read-only/output read-write만 마운트한다. `submit`, `status`, `collect`로 비동기 사용하며 `--wait`로 제출·종료 확인·회수를 이어 실행할 수 있다. 완료는 실제 종료 상태·exit code·산출물로 판정한다. 시간 초과나 실패는 성공으로 바꾸지 않는다.

## 최소 장면과 데이터 계약

- `sim/gazebo/`에 장면 생성·실행·합성 설정·ROS bridge 설정을 둔다. ROS 센서 검사 노드는 `ros2/src/forklift_ros/` 패키지로 설치한다.
- 기본 장면: 바닥, 현재 지게차의 고정 시각 형상, 포켓이 뚫린 합성 팔레트, 별도 거리 검사 장애물. 네트워크 Fuel asset 의존성은 없다.
- 기본 관측: RGB 320×240, 깊이 320×240, CameraInfo, 360도 2D scan, `/clock`, `/tf_static`. 낮은 해상도와 5Hz는 첫 통합 검사 비용을 줄이기 위한 합성 설정이다.
- `base_link`: x 전방/y 좌측/z 위, 카메라 optical: x 우측/y 아래/z 전방. TF는 합성 장착값 정본에서 생성한다. RGB와 depth는 같은 optical grid를 사용한다.
- 알려진 평면까지의 depth, LiDAR의 알려진 장애물 거리, optical→base 변환을 독립적인 장면 진실값과 대조한다. frame ID·해상도·encoding·CameraInfo·timestamp 순서·시간 범위를 검사한다.
- 30초 이상 simulation time의 센서 스트림을 rosbag2로 저장한다. live 관측, 저장된 bag 내용, 프로세스를 재시작한 재생 관측을 별도 결과로 남긴다. RGB/깊이/scan PNG를 직접 확인한다.

## 완료 증거

단위시험 RED→GREEN, lint/format, 기존 CPU 80개 회귀 검사, 새 이미지 빌드, Slurm 실제 실행, Gazebo 센서 생성·bridge·TF 확인, bag 기록·재생 검증, 결과 SHA-256 회수와 대표 PNG 확인. GPU를 배정받지 못하면 CPU 결과만 성공으로 기록하고 GPU 경로는 미검증으로 남긴다.
