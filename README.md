# 자율 지게차 프로젝트

주변 장애물을 고려한 팔레트 핸들링 경로 생성 및 제어 — 임베디드구동 및 실습.

개발 규칙은 **[CONTRIBUTING.md](CONTRIBUTING.md)**를 따른다. 전체 로봇 코드는 이 저장소에서 관리하고 발표는 별도 저장소에 둔다. 컨벤션의 목표 폴더 구조로 이동하는 작업은 아직 수행하지 않았으며, 아래 실행 명령은 현재 구조 기준이다.

## 현재 개발 상태

2026-09-10 기준, 센서 좌표 처리 코어와 합성 입력 예제, 상품 사진·제조사 카탈로그 기반의 **잠정 지게차 모델(URDF/MJCF)**을 구현했다. 모델의 관절 자세·무부하 안정화·렌더링을 검사했다. 노트북용 ROS 2 Jazzy 개발 이미지를 빌드해 컨테이너 코어 시험 64개와 ROS 2 talker/listener 프로세스 간 통신을 확인했다. 실제 센서 드라이버·팔레트 인식·SLAM·자율 주행 시뮬레이션은 아직 구현하지 않았다.

| 구성 | 결정 상태 |
|---|---|
| RGB-D 카메라 | **RealSense D435i 확정** |
| LiDAR | **RPLIDAR 확정**, 과제 자료의 **A2 사용 예정**. A2 세부형은 미확인 |
| 상위 제어기 | **NVIDIA 공식 Jetson Orin Nano Super 개발자 키트 8GB + M.2 2280 NVMe 256GB 권장**. 128GB는 보유 중이거나 비용 제약 시 허용. 미구매 |
| 차체 | DLS08 외형 대응 후보로 잠정 모델 생성. SKU 동일성·조향·부품 치수는 실물 수령 후 확인 |
| 소프트웨어 | 노트북 Ubuntu 24.04 컨테이너 + **ROS 2 Jazzy** 개발 기준. 원격 **Gazebo Harmonic** 통합 시뮬레이션 예정, MuJoCo 빠른 모델 검사 유지 |

과제 원문의 Gemini 335Le는 참고 장비이며 이번 프로젝트의 확정 카메라는 D435i다. 장비 상태와 구매 근거는 [하드웨어 정본](docs/hardware.md), 개발·배포 역할은 [기술 결정](docs/decisions/0001-development-and-deployment-platforms.md)에 기록했다.

## 로컬 실행

Python 3.10 이상, NumPy 1.23 이상이 필요하다. 테스트에는 pytest 7 이상이 필요하다. 해당 의존성이 있는 Python 환경에서 프로젝트 루트를 작업 디렉터리로 실행한다. ROS·GPU·실물 센서는 필요하지 않다. ROS 작업은 [Ubuntu 24.04 / ROS 2 Jazzy 개발 컨테이너 안내](docs/development.md)를 따른다. 확인한 명령과 증거 경계는 [노트북 환경 검증 기록](docs/validation/2026-09-10-laptop-environment.md)에 있다.

```bash
python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error
python -m forklift_core.demo
```

첫 명령은 자동시험, 둘째는 합성 깊이값과 LiDAR 스캔을 실제 변환 함수에 넣는 예제다. 예제는 JSON으로 미터 단위 좌표와 유효 여부를 출력한다. `null`과 `valid: false`는 측정 불가를 뜻하며, 장애물이 없다는 의미가 아니다. 예제의 내·외부 보정값과 센서 수치는 모두 계산 확인용 가상값이다.

현재 구현:

- [`rgbd.py`](forklift_core/rgbd.py): 왜곡이 보정된 깊이 영상의 픽셀 → 카메라 광학 좌표 변환. 깊이 단위·격자·좌표계를 명시적으로 받는다.
- [`geometry.py`](forklift_core/geometry.py): 보정 회전·이동을 이용한 센서 → 로봇 기준 좌표 변환. 잘못된 좌표계와 회전행렬을 거부한다.
- [`lidar.py`](forklift_core/lidar.py): 거리·각도 → LiDAR 기준 평면 좌표 변환. 누락된 빔의 위치를 유지한다.
- [`tests/`](tests/): 단위, 축 방향, 누락값, 잘못된 보정값·메타데이터, 실행 예제를 검증한다.

**64개 테스트 통과**는 위 수학·입력 계약의 합성 시험 결과다. D435i/RPLIDAR의 실측 정확도, 포켓 검출 성능, 지게차 A–D 동작 성공을 뜻하지 않는다. [검증 기록](docs/LOCAL_VALIDATION.md)에 확인 범위와 미검증 항목을 구분했다.

## 다음 작업

상품 모델은 아래 명령으로 지금 검사할 수 있다. 환경 구성은 **[노트북 개발·원격 시뮬레이션 계획](docs/plans/2026-09-10-local-and-remote-development-environment.md)**을 따른다. 노트북은 컨테이너 기반 ROS 개발과 빠른 검사를 맡고, 원격 워크스테이션은 Gazebo 통합 시뮬레이션·학습·장시간 실행을 맡는다. 원격 작업과 실물 센서 연결은 아직 검증하지 않았다. [컨벤션의 구조 전환 목록](CONTRIBUTING.md#10-현재-파일의-적용-계획)에 따른 기존 코드 이동은 별도 작업이다.

1. 기록 데이터 재생과 ROS 센서 어댑터의 공통 입력 계약: RGB·깊이 정합, 촬영 시각, CameraInfo, TF, LiDAR 스캔 메타데이터.
2. 팔레트·포켓 인식 및 추적과 오래된 관측·추적 상실의 처리. 알려진 포켓 좌표를 넣는 시험과 영상에서 검출하는 시험을 구분한다.
3. 실물 수령 후 잠정 모델의 부품 치수·조향 기구·구동 성능을 보정하고 Gazebo 모델을 검증한다. 이후 A 직진 / B 곡선 접근 / C 후진 후 접근 / D 후방 장애물의 네 조건을 시험한다.

[설계](docs/superpowers/specs/2026-09-10-local-sensor-core-design.md)와 [구현 계획](docs/superpowers/plans/2026-09-10-local-sensor-core.md)에 이번 단계의 계약과 제외 범위를 적었다.

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

- [`quest.txt`](quest.txt): 과제 원문 Google Slides 링크
- [`forklift_store_link.txt`](forklift_store_link.txt): 개조 후보 전동 지게차 판매 링크
- **[공개 발표자료](https://lightminn.github.io/forklift-presentations/)**: 주차별 웹 발표
- **[발표 전용 GitHub 레포](https://github.com/lightminn/forklift-presentations)**: 로컬 원본은 `../forklift-presentations/`
- [`presentation/`](presentation/README.md): 발표 전용 레포로 연결되는 로컬 바로가기
- [`2주차 발표 원고`](presentation/week-02/SCRIPT.md): 장별 발화와 시간 배분

이 폴더에서 `bash presentation/present.sh`를 실행한 뒤 **http://127.0.0.1:8765**를 엽니다. `F`는 전체화면, 방향키는 슬라이드 이동입니다.

발표는 별도 저장소의 주차별 기록이다. 이전 발표의 장비 후보와 제안은 작성 당시의 상태이며, 현재 선택은 위 표를 따른다. 이번 로봇 코드 작업으로 발표자료를 수정하지 않았다.
