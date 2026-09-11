# 저장소 컨벤션 설계 기록

작성일: 2026-09-10

상태: 컨벤션 v1 문서화. 규칙 정본은 [CONTRIBUTING.md](../../CONTRIBUTING.md)이며 이 파일에 전문을 중복하지 않는다. 현재 소스의 구조 전환은 아직 수행하지 않았다.

2026-09-11 후속: [구조 전환 적용 이력](../../CONTRIBUTING.md#10-구조-전환-이력)을 참고한다. 아래 내용은 작성 당시의 기록이며, 환경별 검증은 아직 별도 완료가 필요하다.

## 확인한 범위

사용자는 개발을 이어가기 전에 저장소 전체의 폴더 구조·네이밍 등을 정하도록 요청했고, 상위 제어·ROS 2 연동·시뮬레이션·향후 MCU 펌웨어를 한 저장소에서 관리하는 범위를 확인했다. 발표자료는 별도 저장소로 유지한다.

## 선택한 구조와 이유

| 비교한 접근 | 판단 |
|---|---|
| 공통 코어와 입출력 계층을 나눈 단일 저장소 | 채택. 장치 없는 로컬 시험과 인터페이스 공동 검토를 함께 지원 |
| 모든 Python 코드를 ROS 패키지 중심으로 배치 | 채택하지 않음. 현재는 ROS가 필요 없는 계산·시험 경계가 유용 |
| 상위 제어·시뮬레이션·펌웨어를 각각 별도 저장소로 분리 | 채택하지 않음. 사용자가 전체 로봇 코드의 단일 저장소 관리를 선택 |

Python 소스는 `src/forklift_core/`, ROS 패키지는 `ros2/src/`, 펌웨어는 `firmware/`, 선택한 시뮬레이터의 자원은 `sim/`에 둔다. 실제 구현이 생기는 시점에만 해당 폴더를 생성한다. 어댑터에서 코어로 의존하고 코어는 ROS·SDK·시뮬레이터 엔진을 직접 import하지 않는다.

사람이 읽는 컨벤션은 `CONTRIBUTING.md`에 모으고 `AGENTS.md`·`CLAUDE.md`는 같은 정본을 가리킨다. 과제 원문·현재 장비 결정·과거 실험 기록을 서로 바꾸어 해석하지 않는다.

## 전환 범위

정본의 마지막 절에 현재 파일 → 목표 파일 대응, 실행 명령 변경, 기존 64개 테스트 보존, editable·wheel 설치 검사 조건을 기록했다. 현재 작업에서는 컨벤션 문서와 진입 링크만 작성했으며 소스 이동·포맷·의존성 설치·Git 초기화는 하지 않았다.

## 근거

- [PyPA의 src 배치 설명](https://packaging.python.org/en/latest/discussions/src-layout-vs-flat-layout/): 설치 경로와 현재 디렉터리의 import를 구분하는 근거.
- [Ruff 설정 문서](https://docs.astral.sh/ruff/configuration/): 88열·4칸·큰따옴표의 기본 포맷과 pyproject 설정 경로.
- [REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst), [REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst): SI 단위와 로봇·광학·지도 좌표계 역할.
