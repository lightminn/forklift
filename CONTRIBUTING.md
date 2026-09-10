# 개발 컨벤션

버전: 1 · 작성일: 2026-09-10

이 문서는 지게차 프로젝트의 폴더·네이밍·모듈 경계·코드 스타일·시험·문서 규칙의 정본이다. 상위 제어, ROS 2 연동, 시뮬레이션, 향후 자체 MCU 펌웨어는 **한 저장소**에서 관리한다. 발표자료는 기존 별도 저장소를 유지한다. 저장소 범위는 사용자가 확인했다.

**적용 상태:** 새 작업은 이 규칙을 따른다. 기존 코드는 마지막 절의 전환 대상으로 남아 있으며, `src/` 이동은 아직 적용하지 않았다. 개발 도구·ROS 컨테이너 구성 상태와 실행 결과는 `README.md`와 `docs/development.md`에서 확인한다. 이 문서의 작성 자체가 하드웨어 구매, 실물 연결 또는 로봇 기능 구현을 뜻하지 않는다.

현재 하드웨어 결정은 D435i 확정, RPLIDAR 확정 및 A2 사용 예정, Jetson Orin Nano Super 8GB 개발자 키트와 256GB NVMe 구매 권장, 차체 수령 전 판단 보류다. 구매 권장은 구매 완료가 아니며 상세 상태는 `docs/hardware.md`를 따른다. 미정 사항을 폴더 이름이나 가상의 기본값으로 대신 결정하지 않는다.

## 1. 목표 폴더 구조

아래는 **목표 위치 규칙**이다. 현재 파일은 마지막 절의 전환 목록을 따른다. 아직 구현하지 않은 기능의 빈 폴더·빈 클래스·성공하는 척하는 테스트를 미리 만들지 않는다.

```text
forklift/
├── README.md                         # 프로젝트 상태, 실행법, 다음 작업
├── CONTRIBUTING.md                   # 합의된 개발 컨벤션의 정본
├── AGENTS.md / CLAUDE.md              # 에이전트용 프로젝트 지침과 정본 링크
├── pyproject.toml                    # Python 패키징·의존성·검사 설정
├── .editorconfig                     # 인코딩·줄바꿈·들여쓰기
├── .gitignore
├── src/
│   └── forklift_core/                # ROS·실물 장치에 독립적인 Python 코드
│       ├── geometry.py
│       ├── _validation.py            # 패키지 내부 공통 입력 검증
│       ├── sensors/                  # 깊이·스캔 데이터의 수학적 처리
│       │   ├── rgbd.py
│       │   └── lidar.py
│       ├── perception/               # 검출·포켓 추적·로봇 기준 자세 추정
│       ├── planning/                 # 접근 경로·충돌 검사
│       ├── control/                  # 경로 추종·삽입 제어
│       ├── mission/                  # 적재·이송·하역의 순서와 상태 전이
│       └── safety/                   # 관측·명령 유효성 등 공통 안전 판단
├── ros2/
│   └── src/
│       ├── forklift_ros/             # ROS 메시지 ↔ 코어 입출력, 노드
│       ├── forklift_interfaces/      # 필요한 사용자 정의 msg/srv/action
│       ├── forklift_description/     # 실제 선택한 차체의 URDF·mesh
│       └── forklift_bringup/         # launch와 ROS 파라미터 config
├── firmware/                         # 자체 MCU 코드; 보드 선정 후 생성
├── sim/                              # 선택한 엔진의 모델·환경·어댑터
├── config/                           # ROS와 독립적인 실행·시나리오 설정
├── tests/
│   ├── unit/                         # 코어 경로를 따라 구성
│   │   ├── test_geometry.py
│   │   └── sensors/
│   │       ├── test_rgbd.py
│   │       └── test_lidar.py
│   ├── integration/                  # 모듈 연결·명령행 예제
│   ├── scenarios/                    # A–D와 후속 작업 시나리오
│   └── fixtures/                     # 작고 재현 가능한 공통 시험 입력
├── examples/                         # 사용법·합성 입력 예제
├── tools/                            # 재생·변환·검증 등 개발용 실행 도구
├── requirements/                     # 대상별 재현 가능한 의존성 고정 목록
├── deploy/                           # Dockerfile·배포 실행 설정
├── data/                             # 원본 데이터; 큰 파일은 Git 밖에서 관리
├── artifacts/                        # 실행 로그·영상·결과; 기본 Git 제외
├── docs/
│   ├── architecture.md               # 모듈 경계와 데이터 흐름
│   ├── hardware.md                   # 확정·예정·미정 장비와 근거
│   ├── interfaces/                   # 관측·제어·MCU 통신 계약
│   ├── decisions/                    # 오래 유지할 기술 결정과 이유
│   ├── design/                       # 기능별 설계·제안
│   ├── plans/                        # 실행할 작업 계획
│   ├── validation/                   # 날짜가 있는 검증 기록
│   └── references/                   # 과제·외부 자료의 출처와 원본
└── presentation -> ../forklift-presentations
```

Python 배포 패키지명은 `forklift-core`, import 경로는 `forklift_core`로 통일한다. 현재 `forklift-sensor-core` 배포명은 구조 전환 때 변경한다. root의 `src/`는 Python 소스이고, `ros2/src/`는 colcon 패키지 위치다. 서로 복사하거나 같은 패키지를 두 번 두지 않는다.

`src` 배치는 개발 시 editable 설치가 필요하지만, 현재 디렉터리의 소스가 우연히 import되어 패키징 누락을 가리는 일을 줄인다. 이 비용을 받아들이고 실제 설치 경로도 검사하는 쪽을 선택한다. [PyPA 설명](https://packaging.python.org/en/latest/discussions/src-layout-vs-flat-layout/)

`config/`에는 코어 실행·시나리오 입력을 둔다. ROS의 `ros__parameters` YAML은 `forklift_bringup/config/`에 두고 해당 ROS 패키지의 자원으로 설치한다. 같은 의미의 설정을 두 곳에 수동 복제하지 않는다. 공통 값이 필요하면 한 정본에서 명시적으로 변환·전달한다.

## 2. 모듈 경계와 의존 방향

기본 의존 방향은 **실물/ROS/시뮬레이션 어댑터 → 공통 코어**다. 코어는 `rclpy`, RealSense SDK, 시리얼 포트, 시뮬레이터 엔진을 import하지 않는다. NumPy 같은 수학 라이브러리는 코어 의존성으로 허용하고, OpenCV·학습 모델처럼 일부 기능에만 필요한 의존성은 해당 기능 경계에서 관리한다.

| 과제 단계 | 주 책임 | 경계 |
|---|---|---|
| 1. 팔레트 검출 | `perception` | 영상·깊이 → 검출 후보. 좌표를 미리 알려주는 시험과 구분 |
| 2. 로봇 기준 자세 | `perception` + `geometry` | 검출·깊이·보정 → `base_link` 기준 포켓 위치·방향 |
| 3. 장애물 회피 접근 경로 | `planning` | 지도·자기 위치·목표 자세·차체 제약 → 경로 또는 실패 사유 |
| 4. 추종·삽입·적재 | `control` + `mission` | 포켓 관측을 갱신하며 추종·삽입; 삽입과 승강 전환 조건은 명시 |
| 5. 이송·하역 | `mission` | 주행과 하역 절차를 조합하고 각 단계 결과를 확인 |

`sensors`는 입력의 수학·형식 처리를 맡는다. 실물 드라이버를 이 이름 아래 숨기지 않는다. SLAM 패키지 연동은 ROS 계층에서 시작하며, 공통 위치 추정 알고리즘이 실제로 필요해질 때만 코어 모듈을 추가한다.

노드·launch 파일에 알고리즘을 직접 구현하지 않는다. 반대로 단순 계산 함수가 노드·로거·전역 설정 객체를 필수 인자로 요구하게 만들지 않는다. 안전 판단은 적절한 계층의 입출력에 적용하고, 저수준 타임아웃·정지 경로는 상위 Python 코드에만 의존하지 않도록 실물 제어 단계에서 설계한다.

`utils.py`, `common.py`, `manager.py`처럼 책임이 드러나지 않는 새 모듈을 기본 선택으로 쓰지 않는다. 공통화를 위해 과도한 베이스 클래스·플러그인 등록기·범용 설정 프레임워크를 선행 구현하지 않는다.

## 3. 이름과 언어

| 대상 | 규칙 | 예 |
|---|---|---|
| Python 패키지·모듈·함수·변수 | `snake_case` | `pallet_pose.py`, `estimate_pocket_pose` |
| 클래스·예외·enum 타입 | `PascalCase` | `PocketObservation`, `CalibrationError` |
| 상수·enum 항목 | `UPPER_SNAKE_CASE` | `TRACKING_LOST`, `DEFAULT_TIMEOUT_S` |
| 내부 모듈·내부 함수 | `_` 접두사 | `_validation.py`, `_normalize_depth` |
| bool | 상태·조건이 읽히는 이름 | `is_valid`, `has_depth`, `can_insert` |
| 단위가 있는 물리량 | 단위 접미사 | `distance_m`, `yaw_rad`, `speed_mps`, `timeout_s` |
| 픽셀·원시 값 | 표현을 드러내는 접미사 | `pixels_uv`, `depth_raw`, `depth_scale_m` |
| 좌표가 있는 값 | frame 객체에 포함하거나 이름으로 표시 | `points_base_m`, `rotation_base_from_camera` |
| Python 시험 파일·함수 | `test_*.py`, `test_<behavior>` | `test_stale_depth_is_rejected` |
| ROS 패키지 | `forklift_` + `snake_case` | `forklift_bringup` |
| 자체 ROS 노드·토픽·파라미터 | `snake_case`; 토픽은 상대 이름 | `pocket_tracker`, `pallet_pose`, `timeout_s` |
| 자체 ROS msg/srv/action 타입 | `PascalCase` | `PocketObservation.msg` |
| 자체 C/C++ 파일·함수 | `snake_case` | `motor_driver.c`, `set_target_speed` |
| C 모듈의 외부 함수 | 충돌을 피하는 모듈 접두사 | `motor_set_target_speed` |
| C++ 타입·멤버 | `PascalCase`, 비공개 멤버는 후행 `_` | `MotorState`, `target_speed_mps_` |
| 일반 폴더·설정 파일 | 소문자 ASCII `snake_case` | `sensor_replay.yaml` |
| 설계·검증 문서 | `YYYY-MM-DD-kebab-case.md` | `2026-09-10-sensor-core.md` |
| 기술 결정 문서 | `NNNN-kebab-case.md` | `0001-core-and-adapter-boundaries.md` |
| 표준 진입 문서 | 관례적 대문자 이름 | `README.md`, `CONTRIBUTING.md` |

사람이 읽는 프로젝트 설명은 한국어를 기본으로 한다. 코드 식별자·공개 API docstring·코드 주석은 영어를 기본으로 하되 외부 원본의 언어는 유지한다. 주석은 처리 순서를 다시 읽어주는 대신 이유·단위·제약을 설명한다.

비공개 함수는 다른 형제 모듈에서 직접 가져오지 않는다. 패키지 안에서 공유할 검증은 `_validation.py`의 명시적인 내부 API로 두고, 패키지 밖에서는 사용하지 않는다. 현재 `geometry.py`의 `_finite_scalar` 등을 다른 모듈에서 import하는 부분이 이 이동 대상이다.

벤더 드라이버·자동 생성 코드·외부 메시지의 이름은 원래 규칙을 유지한다. 자체 컨벤션에 맞추기 위해 일괄 개명하지 않는다. 표준 메시지 필드는 재정의하지 않고 어댑터에서 내부 계약으로 변환한다.

## 4. 형식과 코드 품질

- UTF-8, LF 줄바꿈, 파일 끝 개행. Python은 공백 4칸.
- Python 포맷·lint는 **Ruff 하나로 통일**한다. 줄 길이는 **88**, 문자열은 큰따옴표, import 정렬을 적용한다. 사람이 포맷을 수동 협상하지 않는다. Ruff 기본 포맷과 맞춘 선택이다. [Ruff 설정 문서](https://docs.astral.sh/ruff/configuration/)
- 초기 lint 범위는 `E4`, `E7`, `E9`, `F`, `I`, `UP`, `B`. `ALL` 규칙이나 전역 무시 목록을 기본으로 쓰지 않는다. 규칙 예외는 사유가 있는 최소 범위에 둔다.
- 설정은 루트 `pyproject.toml`에 모은다. Python 최소 버전과 Ruff 대상은 우선 기존 계약인 3.10을 유지한다.
- 공개 함수의 인자·반환형에 type hint를 쓴다. NumPy 배열의 shape·dtype·단위·프레임과 NaN 의미는 docstring으로 명시한다. 내부 저장 배열과 생성자 입력의 타입이 다른 경우 구분한다.
- import 시 장치 연결, 모델 다운로드, 파일 쓰기, 스레드 시작을 하지 않는다. 실행 진입점은 명시적으로 둔다.
- 현재 시험의 동적 import는 초기 RED 확인을 위한 형태였다. 구조 정리 시 특별한 이유가 없는 테스트는 일반적인 모듈 상단 import로 바꾼다.
- C/C++은 자체 코드부터 clang-format을 적용한다. 컴파일러·언어 표준·벤더 포맷 예외는 첫 해당 타깃을 추가할 때 설정 파일과 함께 확정한다. 차체·MCU 미정 상태에서 컴파일러 선택을 고정하지 않는다.
- 파일 길이·함수 줄 수·커버리지 수치만으로 자동 불합격시키지 않는다. 서로 다른 책임이 섞이는지와 오류를 실제로 잡는 시험이 있는지로 분리 여부를 판단한다.

## 5. 로봇 데이터 계약

계산은 SI 단위를 기본으로 한다. 길이 m, 각도 rad, 속도 m/s, 각속도 rad/s다. 센서 원시 단위는 어댑터 경계에서 변환하고, 원시 데이터가 필요한 경우 변환 계수와 함께 보존한다.

로봇 좌표는 x 전방·y 좌측·z 위, 카메라 optical 좌표는 x 우측·y 아래·z 전방을 사용한다. 지도·연속 오도메트리·차체 프레임은 `map`, `odom`, `base_link` 역할을 따른다. 이 규칙은 실제 센서 장착 보정값을 대신하지 않는다. [REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst), [REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)

- 변환은 이름 또는 타입에 `target_from_source` 방향을 드러낸다. 표현이 불명확한 `T`, `pose`, `angle`만으로 외부 인터페이스를 만들지 않는다. 짧은 지역 수학 변수는 문맥이 분명하면 허용한다.
- quaternion 배열은 순서를 명시한다. 외부 인터페이스에서 배열을 쓴다면 `quaternion_xyzw`와 같이 표시하고, 라이브러리의 wxyz 형식과 경계에서 변환한다.
- 시간 정보가 있는 **관측·명령 객체**는 시각과 clock domain을 함께 가진다. 계산 전용 점·행렬 타입까지 강제로 타임스탬프를 붙이지는 않는다.
- 촬영/측정 시각과 수신 시각을 구분한다. 정수 나노초를 저장할 때는 `_ns`, 시간 간격을 초로 계산할 때는 `_s`를 쓴다. 장치 시각·ROS 시각·monotonic 시각을 변환 근거 없이 빼지 않는다.
- depth grid와 calibration의 해상도·frame·정합 상태를 확인한다. D435i라는 모델명으로 고정 depth scale이나 외부 보정을 추정하지 않는다.
- NaN·측정 누락은 unknown이다. 장애물 없음, 거리 0, 성공 관측으로 바꾸지 않는다.
- 잘못된 설정·좌표계·shape는 명시적인 예외로 거부한다. 정상적으로 발생 가능한 관측 소실·경로 없음은 상태와 원인으로 표현하며, 호출 계층이 정지·재관측·재계획을 결정한다.
- 관측의 유효성, 안전 정지, 작업 성공은 서로 다른 값이다. 안전 정지 결과를 삽입 성공률에 넣지 않는다.

이 절은 앞으로 만들 관측·명령·통신 계약의 기준이며, 현재 구현되지 않은 동기화·watchdog·제어 기능이 있다는 뜻은 아니다.

## 6. 설정·의존성·실행 환경

보정값·센서/차체 파라미터·시나리오 입력은 YAML, 실행 결과와 스냅샷은 JSON을 기본으로 한다. 로봇 설정과 원시 외부 메시지 사이의 변환은 코드에서 한 번 수행한다. 알 수 없는 필드와 필수값 누락을 조용히 무시하지 않는다.

차체 치수·외부 보정처럼 실물 확인이 필요한 값에는 그럴듯한 기본값을 넣지 않는다. 합성 예제의 가상 설정은 synthetic 입력임을 명시한다. 설정 우선순위는 명시적 CLI 인자 → 지정 설정 파일 → 문서화된 일반 기본값이다. 환경변수는 경로·접속·개인 비밀값처럼 실행 환경에 속하는 값에 한정한다.

Python 패키지 의존성의 정본은 `pyproject.toml`이다. 공통 런타임과 `dev`·향후 기능별 선택 의존성을 분리한다. 재현용 고정 목록은 `requirements/`에 대상 Python/아키텍처와 생성 방법을 기록해 저장한다. 로컬 x86 환경의 고정 목록을 Jetson에 그대로 적용하지 않는다.

구조 전환 이후 기본 개발 설치는 대상 Python 환경에서 `python -m pip install -e '.[dev]'`로 통일한다. `sys.path.insert`, 관행적인 전역 `PYTHONPATH`, 전역 pip 설치를 표준 실행 절차로 삼지 않는다. 구조 적용 단계에서 실제 설치와 import를 검증한다.

README에는 프로젝트가 요구하는 버전과 설치·실행 절차를 적는다. 개인 conda 경로, SSH 별칭, 계정과 해당 머신에 설치된 목록은 전역 환경 기록에 둔다. ROS 런타임은 선정한 ROS/OS Python 환경을 사용하며, 로컬 개인 conda 설정을 배포 환경의 전제로 삼지 않는다.

## 7. 시험과 검증 기록

- 기능·버그 수정은 실패하는 테스트 → 실패 확인 → 구현 → 통과 확인 순서로 진행한다. 파일 이동·문서 편집을 위해 구현을 그대로 복제한 테스트를 추가하지 않는다.
- 단위시험은 외부 장치·네트워크 없이 실행한다. import 시 ROS나 카메라 SDK가 필요하면 단위시험 경계를 다시 나눈다.
- 통합시험은 실제 연결되는 구성요소를 명시한다. ROS 패키지의 자체 시험은 해당 패키지의 `test/`에 두고, 루트 `tests/`에 중복 복사하지 않는다.
- 시나리오 이름은 `case_a_straight`, `case_b_curved`, `case_c_reverse`, `case_d_rear_obstacle`로 통일한다. 입력 조건과 기대 행동도 같이 명시한다.
- 무작위 시험은 seed를 저장한다. 기대값을 구현 함수로 다시 계산하지 않는다. 경계·반례·실패 처리와 사용자가 관찰하는 결과를 검증한다.
- 시뮬레이션·ROS·장치 의존 시험은 `simulation`, `ros`, `hardware` marker 등으로 의존성을 드러내고 별도 명령/CI job에서 실행한다. 필요한 환경이 없으면 skip 사유와 개수를 보고하며 pass로 세지 않는다.
- 기본 검사 흐름은 Ruff lint → Ruff format check → 단위·로컬 통합시험이다. 패키지 레이아웃·의존성·배포 경로를 바꾸면 설치된 wheel을 소스 트리 밖에서 import/실행하는 검사도 한다.
- CI는 로컬과 같은 명령을 사용한다. ROS·Jetson·센서 장치가 필요한 job을 기본 Python job의 성공으로 대체하지 않는다.

실행 산출물은 `artifacts/<UTC시각>_<시나리오>_<구분자>/`에 모은다. 예: `artifacts/20260910T120000Z_case_a_straight_01/`. 결과 디렉터리는 기존 실행을 덮어쓰지 않는다.

재현 기록에는 코드 revision(미커밋/비Git이면 그 상태와 소스 해시), 설정 스냅샷, 입력 출처, seed, 실행 명령, 환경 버전, 실제 결과, 실패·skip 사유를 둔다. 입력 출처 `synthetic`/`replay`/`live`와 검증 종류(수학·운동학·물리·센서 렌더링·실물)를 구분한다. rosbag 재생도 실시간 실물 주행 증거가 되는 것은 아니다.

작은 검증 요약과 대표 결과는 `docs/validation/`에 남기고, 원본 영상·대형 로그는 보관 위치와 식별자로 연결한다. 최소 작업 시간은 경로 길이만으로 판정하지 않고 전후진 전환·정렬·삽입 등 평가에 포함한 시간을 기록한다.

## 8. 데이터와 외부 자료

- 작고 필요한 테스트 fixture만 Git에 포함한다. 원본 rosbag, 녹화 영상, 학습 데이터와 모델 가중치는 기본적으로 외부 저장소에 두고 경로/버전/체크섬/출처를 기록한다.
- `data/`의 원본은 읽기 전용으로 취급한다. 전처리·렌더·평가 결과는 `artifacts/`에 새로 쓴다.
- 모델 가중치와 시뮬레이션 차체 모델을 같은 `models/` 폴더에 섞지 않는다. 전자는 데이터 자산 기록, 후자는 `sim/` 또는 `forklift_description`의 역할에 맞춰 둔다.
- `docs/references/`는 외부 자료와 출처 기록의 위치다. 외부 원본 파일명·내용을 자체 네이밍에 맞추려고 무리하게 바꾸지 않는다.
- 일반 빌드·캐시 산출물과 `ros2/build`, `ros2/install`, `ros2/log`는 Git에서 제외한다.
- 발표 저장소로 향하는 기존 `presentation/` 링크는 호환 경로로 유지하고 검사·포맷·재귀 이동 대상에서 제외한다. 다른 clone에 형제 저장소가 없어도 코어 실행이 가능해야 한다.
- 기존 `forklift-development-web.zip`은 내용을 확인하기 전 이동·삭제하지 않는다. 기존 `.git`, `.codex`, `.agents` 같은 환경 소유 항목도 자동 정리하지 않는다.

## 9. 문서와 Git 운영

`README.md`는 현재 상태·진입 명령·다음 작업을 보여준다. 컨벤션 정본은 `CONTRIBUTING.md`, 하드웨어 선택 정본은 `docs/hardware.md`, 구조 정본은 `docs/architecture.md`로 나눈다. 같은 규칙과 상태를 여러 문서에 전문 복사하지 않고 정본을 링크한다.

`AGENTS.md`와 `CLAUDE.md`는 에이전트별 짝을 유지한다. 두 파일에는 프로젝트 작업 경계와 정본을 읽으라는 지침을 동일하게 적는다. 특정 도구·개인 계정·설치 현황을 프로젝트 컨벤션에 섞지 않는다.

새 문서는 도구 이름을 경로에 넣지 않는다. `docs/superpowers/specs/`와 `docs/superpowers/plans/`는 구조 전환 작업에서 각각 `docs/design/`와 `docs/plans/`로 이동하고 관련 링크도 갱신한다. 과거 검증 결과·원시 기록의 의미는 바꾸지 않는다.

Git 기본 브랜치는 `main`으로 두고, 작업 브랜치는 `feat/sensor-replay`, `fix/depth-scale`, `docs/repository-conventions`처럼 목적을 표시한다. 2026-09-10에 저장소가 초기화됐으며 최초 커밋 전에는 기존 이력에 맞출 커밋 관례가 없다.

초기 커밋 형식은 `type(scope): summary`의 영어 메시지를 사용한다. 예: `feat(sensors): add rectified depth projection`. 유형은 `feat`, `fix`, `refactor`, `test`, `docs`, `build`, `ci`, `chore`에서 선택하고 scope는 실제 변경한 영역을 쓴다.

파일별로 명시적으로 stage하고 무관한 변경이 섞인 파일은 제외한다. 구조·포맷 정리와 기능 변경은 분리한다. 커밋과 push는 사용자가 요청·승인한 범위에서만 한다. 컨벤션 합의는 Git 초기화나 원격 생성·공개의 승인으로 해석하지 않는다.

## 10. 현재 파일의 적용 계획

다음 구조 전환 작업은 아래 변경을 **구조 정리 작업 하나**로 수행하고 기능 추가를 섞지 않는다. 이 컨벤션 문서 작성만으로 파일 이동이나 검사 도구 설정이 완료된 것은 아니다.

| 현재 | 변경안 |
|---|---|
| `forklift_core/geometry.py` | `src/forklift_core/geometry.py`; 공유 검증은 `_validation.py`로 분리 |
| `forklift_core/rgbd.py` | `src/forklift_core/sensors/rgbd.py` |
| `forklift_core/lidar.py` | `src/forklift_core/sensors/lidar.py` |
| `forklift_core/demo.py` | `examples/sensor_geometry.py`; 실행 안내와 통합시험 갱신 |
| `tests/test_geometry.py`, `test_rgbd.py`, `test_lidar.py` | 대응하는 `tests/unit/` 경로 |
| `tests/test_demo.py` | `tests/integration/test_sensor_geometry_example.py` |
| `docs/LOCAL_VALIDATION.md` | `docs/validation/2026-09-10-sensor-core.md` |
| `docs/superpowers/specs/…` | `docs/design/…` |
| `docs/superpowers/plans/…` | `docs/plans/…` |
| `quest.txt`, `forklift_store_link.txt` | 내용은 보존하고 `docs/references/`로 이동; 참조 명령 갱신 |
| `pyproject.toml` | `src` 패키징·배포명·`dev` extra·Ruff 설정 반영 |
| `README.md`, `AGENTS.md`, `CLAUDE.md` | 실제 새 경로·명령과 정본 링크를 함께 갱신 |

현재 64개 시험의 행동을 유지하고, 포맷/정적 검사와 editable·wheel 설치 경로를 확인한다. 미구현 ROS·펌웨어·시뮬레이션 폴더는 생성하지 않는다. 전환 후의 실행 명령은 설치 검증 결과와 함께 다시 안내한다.
