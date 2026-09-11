# 자율 지게차 프로젝트

주변 장애물을 고려한 팔레트 핸들링 경로 생성 및 제어 — 임베디드구동 및 실습.

개발 규칙은 **[CONTRIBUTING.md](CONTRIBUTING.md)**를 따른다. 전체 로봇 코드는 이 저장소에서 관리하고 발표는 별도 저장소에 둔다. 2026-09-11에 `src/forklift_core/`, `examples/`, `tests/unit|integration/` 구조를 적용했다. 실행 전에 대상 Python 환경에 editable로 설치한다.

## 현재 개발 상태

2026-09-11 기준, 센서 좌표 처리 코어와 합성 입력 예제, 상품 사진·제조사 카탈로그 기반의 **잠정 지게차 모델(URDF/MJCF)**을 구현했다. 모델의 관절 자세·무부하 안정화·렌더링을 검사했다. 노트북용 ROS 2 Jazzy 개발 이미지를 빌드해 컨테이너 코어 시험 64개와 ROS 2 talker/listener 프로세스 간 통신을 확인했다. 고정 지게차·팔레트·장애물의 Gazebo 장면에서 합성 RGB-D·2D LiDAR를 ROS 2로 전달하고 30초 이상 기록·재생·좌표 검증까지 확인했다. 실제 센서 드라이버·팔레트 인식·SLAM·자율 주행은 아직 구현하지 않았다. 여기 적힌 시험 개수는 각 검증 시점의 기록이며, 최신 전체 회귀 결과와 원본 증거는 [개발 중간 정리](docs/validation/2026-09-11-development-checkpoint.md)를 따른다.

원격 전용 MuJoCo 환경에서도 Slurm CPU 시험 **80개 통과**, **20초 물리 적분**, 포크 승강을 포함한 **4초 소프트웨어 렌더 영상**을 확인했다. GPU EGL 시험은 자원 대기로 실행하지 못했다. 상세 결과와 영상 위치는 [원격 모델 테스트런 기록](docs/validation/2026-09-10-remote-model-smoke.md)에 있다.

| 구성 | 결정 상태 |
|---|---|
| RGB-D 카메라 | **RealSense D435i 확정** |
| LiDAR | **RPLIDAR 확정**, 과제 자료의 **A2 사용 예정**. A2 세부형은 미확인 |
| 상위 제어기 | **NVIDIA 공식 Jetson Orin Nano Super 개발자 키트 8GB + M.2 2280 NVMe 256GB 권장**. 128GB는 보유 중이거나 비용 제약 시 허용. 미구매 |
| 차체 | DLS08 외형 대응 후보로 잠정 모델 생성. SKU 동일성·조향·부품 치수는 실물 수령 후 확인 |
| 소프트웨어 | 노트북 Ubuntu 24.04 컨테이너 + **ROS 2 Jazzy** 개발 기준. 원격 **Gazebo Harmonic** 정적 장면·센서 기록/재생 검증, MuJoCo 빠른 모델 검사 유지 |

과제 원문의 Gemini 335Le는 참고 장비이며 이번 프로젝트의 확정 카메라는 D435i다. 장비 상태와 구매 근거는 [하드웨어 정본](docs/hardware.md), 개발·배포 역할은 [기술 결정](docs/decisions/0001-development-and-deployment-platforms.md)에 기록했다.

## 로컬 실행

Python 3.10 이상, NumPy 1.23 이상이 필요하다. 테스트에는 pytest 7 이상이 필요하다. 해당 의존성이 있는 Python 환경에 아래처럼 설치한 뒤 프로젝트 루트에서 명령을 실행한다. 설치된 코어는 작업 디렉터리와 무관하게 import할 수 있다. ROS·GPU·실물 센서는 필요하지 않다. ROS 작업은 [Ubuntu 24.04 / ROS 2 Jazzy 개발 컨테이너 안내](docs/development.md)를 따른다. 확인한 명령과 증거 경계는 [노트북 환경 검증 기록](docs/validation/2026-09-10-laptop-environment.md)에 있다.

```bash
python -m pip install -e '.[dev]'
python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error
python examples/sensor_geometry.py
```

설치 후 pytest 명령은 자동시험, 예제 명령은 합성 깊이값과 LiDAR 스캔을 실제 변환 함수에 넣는다. 예제는 JSON으로 미터 단위 좌표와 유효 여부를 출력한다. `null`과 `valid: false`는 측정 불가를 뜻하며, 장애물이 없다는 의미가 아니다. 예제의 내·외부 보정값과 센서 수치는 모두 계산 확인용 가상값이다.

현재 구현:

- [`rgbd.py`](src/forklift_core/sensors/rgbd.py): 왜곡이 보정된 깊이 영상의 픽셀 → 카메라 광학 좌표 변환. 깊이 단위·격자·좌표계를 명시적으로 받는다.
- [`geometry.py`](src/forklift_core/geometry.py): 보정 회전·이동을 이용한 센서 → 로봇 기준 좌표 변환. 잘못된 좌표계와 회전행렬을 거부한다.
- [`lidar.py`](src/forklift_core/sensors/lidar.py): 거리·각도 → LiDAR 기준 평면 좌표 변환. 누락된 빔의 위치를 유지한다.
- [`tests/`](tests/): 단위, 축 방향, 누락값, 잘못된 보정값·메타데이터, 실행 예제를 검증한다.

위 명령은 코어 합성 시험 **64개**와 원격 제출 도구의 로컬 시험을 함께 실행한다. 코어 64개 통과는 위 수학·입력 계약의 합성 시험 결과다. D435i/RPLIDAR의 실측 정확도, 포켓 검출 성능, 지게차 A–D 동작 성공을 뜻하지 않는다. [검증 기록](docs/validation/2026-09-10-sensor-core.md)에 확인 범위와 미검증 항목을 구분했다.

## 중간 정리와 전체 로드맵

**[2026-09-11 개발 중간 정리](docs/validation/2026-09-11-development-checkpoint.md)**에서 구현·검증·미완료 범위와 원본 증거를 확인한다. **[전체 개발 로드맵](docs/plans/2026-09-11-development-roadmap.md)**은 15주 수업 중 시험·공휴일을 제외한 유효 개발 약 12주를 기준으로 한다.

구조 전환은 호스트·wheel·컨테이너·원격에서 동등성을 확인했다([검증 기록](docs/validation/2026-09-11-src-layout-migration.md)). 다음 순서는 다양한 합성 장면·포켓 관측 계약 → RGB-D 기반 포켓 위치 추정·추적이다. 실물 조사·센서 보정·하위 제어를 병행하고, 기구·정지 검증 후 주행을 연결한다. A 직진 조건의 삽입·적재·이송·하역을 먼저 완성한 뒤 B 곡선 접근, C 후진 접근, D 후방 장애물 조건으로 확장한다.

실물 제어는 현재 잠정 모델의 조향·치수 가정을 그대로 사용하지 않는다. GPU EGL·실물 센서·자율 제어는 별도 검증 항목이며, 마지막 두 개발 주차는 반복 검증과 시연 준비에 배정한다. 환경별 실행 역할과 준비 이력은 [노트북·원격 개발 계획](docs/plans/2026-09-10-local-and-remote-development-environment.md)을 따른다.

## 원격 센서 시뮬레이션

[`sim/gazebo/README.md`](sim/gazebo/README.md)에서 합성 장면과 검증 계약을 확인한다. 원격 실행은 소스 snapshot을 고정하고 Slurm에서 별도 Docker 이미지로 실행한다. [개발 안내](docs/development.md)의 `tools/submit_model_check.py submit --mode gazebo --duration 30 --wait` 명령으로 제출·완료 확인·결과 회수를 연결한다. SSH host와 원격 project root, image는 명시적으로 지정한다.

결과에는 실제 RGB·depth·scan PNG, ROS bag, live/저장 bag/새 프로세스 replay의 개수·기간·거리·TF 검사, Slurm 상태와 파일 SHA-256이 남는다. [실행·실패 수정·검증 기록](docs/validation/2026-09-10-gazebo-sensor-baseline.md)을 함께 확인한다. CPU llvmpipe로 확인한 결과이며 실물 D435i/RPLIDAR의 성능·잡음 모델이나 주행 성공을 뜻하지 않는다.

## 상품 기반 지게차 모델

**[모델 파일·실행 설명](sim/models/dls08_provisional/README.md)** · [원본 출처와 추정값](docs/references/dls08/README.md) · [모델 검증 기록](docs/validation/2026-09-10-product-forklift-model.md)

DLS08 후보 카탈로그의 전체 크기 **1.46 × 0.63 × 1.01m**, 순중량 **24kg**을 사용했다. 지붕·캐빈·마스트·네 바퀴·두 포크를 구성하고 바퀴 회전 4개, 조향 2개, 포크 승강 1개를 분리했다. 조향 축·휠베이스·포크 간격·승강 범위와 동역학은 실측 전 가정이며 `parameters.yaml`에서 수정한다. 센서 장착 보정은 포함하지 않았다.

```bash
python -m pip install -e '.[model,test]'
python tools/preview_forklift_model.py \
  --model sim/models/dls08_provisional/scene.xml \
  --output "artifacts/$(date -u +%Y%m%dT%H%M%SZ)_dls08_model_preview_01" \
  --backend egl --frames 96
```

PNG 다각도 이미지와 포크 승강·조향 자세 MP4를 만든다. EGL 그래픽 문맥과 ffmpeg가 필요하다. 영상은 관절 위치를 지정한 운동학 미리보기이며 자율 주행 결과가 아니다. MuJoCo는 이번 모델 검사에 사용하고, URDF는 형상·관절 교환용으로 제공한다. URDF 물리 실행에는 가져오는 엔진의 자기 충돌·구동 설정이 추가로 필요하다.

## 과제와 발표자료

- [`quest.txt`](docs/references/quest.txt): 과제 원문 Google Slides 링크
- [`forklift_store_link.txt`](docs/references/forklift_store_link.txt): 개조 후보 전동 지게차 판매 링크
- **[공개 발표자료](https://lightminn.github.io/forklift-presentations/)**: 주차별 웹 발표
- **[발표 전용 GitHub 레포](https://github.com/lightminn/forklift-presentations)**: 로컬 원본은 `../forklift-presentations/`
- [`presentation/`](presentation/README.md): 발표 전용 레포로 연결되는 로컬 바로가기
- [`2주차 발표 원고`](presentation/week-02/SCRIPT.md): 장별 발화와 시간 배분

이 폴더에서 `bash presentation/present.sh`를 실행한 뒤 **http://127.0.0.1:8765**를 엽니다. `F`는 전체화면, 방향키는 슬라이드 이동입니다.

발표는 별도 저장소의 주차별 기록이다. 이전 발표의 장비 후보와 제안은 작성 당시의 상태이며, 현재 선택은 위 표를 따른다. 이번 로봇 코드 작업으로 발표자료를 수정하지 않았다.
