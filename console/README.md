# 운용 콘솔

[설계](../docs/design/2026-10-13-operator-console.md) · [계약 초안 console/1](../docs/interfaces/2026-10-13-operator-console-contract-draft.md)

- `web/`: 브라우저 콘솔(빌드 없음, ES 모듈). 화면 구성은 차량 관제 화면(FMS) 형식을 따른다: 왼쪽 메뉴(모니터링 · 이벤트 기록 · 통신), 가운데 지도(끌기 이동 · 휠/두 손가락 확대 · 차량 따라가기 · 표시 층 선택)와 그 위에 뜬 전방 카메라 창, 오른쪽 차량 상세(진행 막대 · 권한/모드/작업 상태 · 현재 목적지 · 위치 · 작업 단계)와 수동 조종 탭. 작업은 '작업 지정' 창에서 작업 종류 → 대상 설정 → 확인 세 단계로 낸다. 작업 종류는 차량이 `capabilities`에 지원한다고 밝힌 것만 보이며, console/1 에는 '팔레트 운반' 하나뿐이다. 지도에서 대기 중인 팔레트를 누르면 그 팔레트를 고른 채로 2단계가 열린다.
- `gateway/`: **관찰 전용 게이트웨이(C2a)**. ROS 2 의 `/map`·`/scan`·`/odom`·TF(`map`·`odom`·`base_link`)·(선택) 압축 카메라 토픽을 console/1 의 상태·격자·스캔·영상으로 바꾼다. 조종 권한·STOP·모드·수동 조종·작업 지시는 모두 `reject(unsupported)` 로 거부하고, 화면은 '정지 불가'와 차량이 선언한 실제 정지 방법(`stop_instruction`)을 늘 보인다. 차량을 움직이거나 멈추는 경로가 없다. 설계: [C2a](../docs/design/2026-10-15-console-gateway-monitor.md)
- `wire.py`: 가짜 차량과 게이트웨이가 함께 쓰는 전송 규칙(엄격한 JSON, 미디어 프레임, 정적 파일, 연결별 송신 큐, 최신값만 보내는 미디어)
- `mock_vehicle/`: 계약 초안을 흉내 낸 **가짜 차량**. 실차·Isaac·중재기(M-b)가 아니다. 여기서 통과한 규칙 시험은 콘솔 개발용이며 차량 쪽 안전의 증거가 아니다.

## 실행

    python -m pip install -e '.[console]'
    python -m console.mock_vehicle --port 8765
    # 브라우저: http://localhost:8765/

게임패드는 브라우저가 보안 컨텍스트로 보는 `localhost`에서만 확실히 동작한다. 웹 시험에는 Node 18 이상이 필요하다.

가짜 차량의 한계: 정지 확인을 센서가 아니라 자기가 낸 적용 명령으로 하고(실차는 잡음 있는 피드백을 걸러야 한다), 모든 요청을 한 프로세스 안에서 즉시 처리한다(실차는 게이트웨이 → 중재기 → 미션 실행부가 비동기로 나뉘고, 결과 캐시의 주인은 중재기다). 상태의 `applied`도 slew·허가 상한·안전 선점까지만 거친 값이다(실차는 Ackermann 바퀴 속도·조향 속도 한계까지 거친다). 위치는 시뮬레이션 정답(`source: mock_truth`)이다. 이 차이는 C2·M-b에서 다룬다. 다른 기기에서 접속할 때는 `ssh -L 8765:localhost:8765 <호스트>` 뒤 `http://localhost:8765/`를 연다.

### 관찰 전용 게이트웨이 (ROS 2 컨테이너)

    docker build -t forklift/ros2-console:jazzy -f deploy/console/Dockerfile deploy/console
    docker run --rm --network host -v "$PWD":/workspace -w /workspace forklift/ros2-console:jazzy \
        python3 -m console.gateway --backend isaac --vehicle-id fl-1 --port 8765 \
        --ros-args -p use_sim_time:=true -p camera_topic:=/camera/color/image_raw/compressed
    # 브라우저: http://<호스트>:8765/

관찰 전용 화면은 STOP 대신 정지 방법 문장을 보인다. 기본값은 Isaac "시뮬레이션은 Isaac 실행기(잡)를 멈추세요", 실차 "주 전원 스위치를 끄세요"(시험된 비상정지가 아직 없어 사용자가 정한 방법, 2026-10-15)이고, `--stop-instruction` 으로 바꾼다.

`forklift/ros2-console:jazzy` 는 `forklift/ros2-dev:jazzy` 에 `websockets` 만 더한 이미지다(개발 이미지는 바꾸지 않는다). 뒤 차축 위치는 `--rear-axle-x`(기본 −0.34 m, `dls08_measured`)로 바꾼다.

### 기록된 Isaac SLAM bag 으로 보기 (Slurm, CPU)

    sbatch deploy/console/replay_demo.sbatch <bag 디렉터리> [재생 횟수, 기본 3]
    # 잡이 고른 포트: console-replay-<잡 번호>.port
    ssh -N -L 8765:127.0.0.1:<포트> <계산 호스트>   # 그다음 http://localhost:8765/

저장소 루트에서 낸다. bag 은 `python3 -m forklift_ros.slam_replay` 가 만든 것(`/scan`·`/tf`·`/odom`, sim 시각)이다. 재생마다 slam_toolbox(재생 매개변수)를 새로 띄우고 bag 을 `/clock` 과 함께 한 번 튼다. 게이트웨이는 계속 떠 있어서 재생이 끝나면 '위치 n s 전'이, 다음 재생에서는 시각이 뒤로 가는 경우가 화면에 보인다. 컨테이너는 잡의 예약(CPU 1·메모리 512 MB)만큼으로 묶인다 — 도커는 Slurm cgroup 밖에서 돌기 때문이다. 같은 날 실측은 최대 0.23 코어·185 MiB 였다. 2026-10-15 에 공장 임무 bag(214 s)으로 지도·LiDAR·map 위치·후진 속도·끊김 표시·시각 되돌림을 확인했다.

## 시험

    python -m pytest tests/unit/console tests/integration/console -q -p no:cacheprovider -W error
    node --test "console/web/test/*.test.mjs"
    # ROS 노드 시험(호스트에서는 건너뜀):
    docker run --rm --network none -v "$PWD":/workspace -w /workspace forklift/ros2-console:jazzy \
        python3 -m pytest tests/integration/console/test_gateway_ros.py -q -p no:cacheprovider

## 확인한 환경 (2026-10-15, FMS 형식 화면)

- 헤드리스 Chromium(Playwright)에서 1600 px · 1024 px · 400 px 폭으로 확인했다. 확인한 흐름은 권한 → 작업 지정 3단계 → 작업 시작(AUTO, 경로·진행 막대·현재 목적지) → 수동 조종 시작 → 무장 → 키보드 누르고 있기(예상 궤적 표시) → STOP → 재무장 → 재개 체크리스트 → 취소, 지도 휠 확대 · 끌기 · 따라가기 · 맞춤 보기, 지도에서 팔레트 눌러 작업 지정 열기, 새로고침 뒤 조종 권한 회수다. 폰 폭에서 가로 넘침은 없다.

## 확인한 환경 (2026-10-13)

- 가짜 차량 서버: Python 3.11, websockets 16, Pillow 12 (conda base)
- 웹 시험: Node 22 (`node --test`는 디렉터리가 아니라 글롭을 받는다)
- 화면: 헤드리스 Chromium(Playwright)에서 1440 px·400 px 폭으로 렌더와 흐름(권한 → 작업 시작 → 수동 조종 보류 → 무장 → 누르고 있기 → 포커스 잃음 해제 → STOP → 재무장 → 재개 체크리스트)을 확인했다. Firefox·Safari·실제 폰·게임패드는 아직 확인하지 않았다.
