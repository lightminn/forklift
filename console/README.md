# 운용 콘솔

[설계](../docs/design/2026-10-13-operator-console.md) · [계약 초안 console/1](../docs/interfaces/2026-10-13-operator-console-contract-draft.md)

- `web/`: 브라우저 콘솔(빌드 없음, ES 모듈). 화면 구성은 차량 관제 화면(FMS) 형식을 따른다: 왼쪽 메뉴(모니터링 · 이벤트 기록 · 통신), 가운데 지도(끌기 이동 · 휠/두 손가락 확대 · 차량 따라가기 · 표시 층 선택)와 그 위에 뜬 전방 카메라 창, 오른쪽 차량 상세(진행 막대 · 권한/모드/작업 상태 · 현재 목적지 · 위치 · 작업 단계)와 수동 조종 탭. 작업은 '작업 지정' 창에서 작업 종류 → 대상 설정 → 확인 세 단계로 낸다. 작업 종류는 차량이 `capabilities`에 지원한다고 밝힌 것만 보이며, console/1 에는 '팔레트 운반' 하나뿐이다. 지도에서 대기 중인 팔레트를 누르면 그 팔레트를 고른 채로 2단계가 열린다.
  - **대기열과 저장된 운반**([계획](../docs/plans/2026-10-22-console-mission-queue.md)): 작업 지정 3단계의 '운반 저장'으로 자주 쓰는 팔레트 → 하역 칸 조합을 버튼으로 남기고(이 브라우저의 `localStorage`, 차량별), '대기열에 넣기'나 저장된 운반 버튼으로 차량 상세의 '대기열'에 쌓는다. 재개(2단계 확인)하면 앞 항목부터 하나씩 내고, 결과가 `done` 이면 1초 뒤 다음 항목을 낸다. 실패·취소·일시정지·STOP·수동 지시·조종 권한 상실·연결 끊김·새 세션·상태 2초 끊김·안전 선점·차량 피드백 없음 가운데 무엇이든 생기면 대기열은 멈추고, 운용자가 다시 재개할 때까지 다음 미션을 내지 않는다(진행 중 미션은 건드리지 않는다). 대기열은 탭마다 따로 `sessionStorage` 에 두어 새로고침 뒤에도 남지만 늘 멈춘 채 열리며, 냈지만 결과를 모르는 항목은 '불확실'로 표시되어 운용자가 지우거나 다시 내기로 정할 때까지 재개되지 않는다. 대기열은 이 화면에만 있다(차량 쪽 대기열이 아니다).
  - **상태·경보**([계획](../docs/plans/2026-10-22-console-status-panel.md)): 왼쪽 메뉴의 '상태·경보'는 콘솔이 이미 받는 값 — 제어·미디어 연결, 상태·위치·영상·스캔·지도의 나이, 차량 피드백, 정지·안전 선점, 구동 허가, 장면 손상, 대기 서버의 세션 실패, 대기열의 이상 멈춤 — 을 정상·주의·위험·모름·해당 없음으로 보이고, 주의·위험이 0.5초 이어지면 경보를 열어 정상이 1초 이어지면 '복구'로 닫는다. 경보 시각은 이 화면이 처음 본 브라우저 시각이며(차량이 내는 경보 원장이 아니다), 목록은 탭의 `sessionStorage` 에 남아 새로고침 뒤에도 보인다. 메뉴 배지는 열린 경보 수이고, 보지 않는 동안 열렸다 닫힌 위험 경보가 있으면 빨간 점을 단다. 의도한 세션 종료·장면 초기화·대기 서버 기동 중에는 연결 행을 '해당 없음'으로 두되, 대기 서버가 실패를 알리면 그쪽이 우선한다.
- `gateway/`: **관찰 전용 게이트웨이(C2a)**. ROS 2 의 `/map`·`/scan`·`/odom`·TF(`map`·`odom`·`base_link`)·(선택) 압축 카메라 토픽을 console/1 의 상태·격자·스캔·영상으로 바꾼다. 조종 권한·STOP·모드·수동 조종·작업 지시는 모두 `reject(unsupported)` 로 거부하고, 화면은 '정지 불가'와 차량이 선언한 실제 정지 방법(`stop_instruction`)을 늘 보인다. 차량을 움직이거나 멈추는 경로가 없다. 설계: [C2a](../docs/design/2026-10-15-console-gateway-monitor.md)
- `wire.py`: 가짜 차량과 게이트웨이가 함께 쓰는 전송 규칙(엄격한 JSON, 미디어 프레임, 정적 파일, 연결별 송신 큐, 최신값만 보내는 미디어)
- `mock_vehicle/`: 계약 초안을 흉내 낸 **가짜 차량**. 실차·Isaac·중재기(M-b)가 아니다. 여기서 통과한 규칙 시험은 콘솔 개발용이며 차량 쪽 안전의 증거가 아니다. 하역된 팔레트를 다시 집는 미션(예: P-01→D-A 뒤 P-01→D-B)은 접근 중 `occupied` 허가로 멈춰 끝나지 않는다 — 가짜 차량 장면의 인공물이고 막힘 시간 제한이 없어서다(끊김과 무관, 2026-10-22 분류).

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

## 확인한 환경 (2026-10-22, 상태·경보)

- 가짜 차량과 헤드리스 Chromium(Playwright), 1600 px · 400 px. 처음 연결 → 경보 0 / 끊김 시험(3초) → 제어 연결 위험 경보, 재접속 뒤 '복구' / STOP → 정지 경보, 재무장 뒤 '복구' / 모니터링을 보는 동안 열렸다 닫힌 위험 경보 → 배지 빨간 점, 상태·경보를 열면 사라짐 / 장면 손상 주입 → 위험 배지, 장면 초기화 뒤 '새 세션'으로 닫힘(초기화 중 연결 경보 없음) / 열린 경보가 있는 채 새로고침 → '화면 다시 열림 — 그 사이 모름'으로 닫히고 지금도 이상이면 다시 열림 / 진행 중 실패 주입 + 대기열 항목 → 대기열 주의 경보, 대기열을 비우면 '해당 없어짐' / 바뀐 것이 없을 때 행·목록 노드 유지 / 가로 넘침 없음. 실차·Isaac·대기 서버(standby) 경로는 브라우저로 확인하지 않았다(단위 시험만).

## 확인한 환경 (2026-10-22, 대기열)

- 가짜 차량과 헤드리스 Chromium(Playwright), 1600 px · 400 px. 확인한 흐름: 저장된 운반 둘 → 대기열 셋 → 재개(2단계) → 첫 운반 `done` 뒤 둘째가 저절로 나감 / 진행 중 실패 주입(`/mock/inject/fail_running`) → 멈춤, 다음 안 냄 / 진행 중 취소 → 곧바로 멈춤 / 진행 중 STOP → 멈춤, 재무장 뒤에도 멈춤 / 끊김 시험(3초) → 멈춤, 재접속 뒤에도 멈춤, 진행 미션은 계속 / 그 미션 중 새로고침 → 멈춘 채 열리고 그 항목은 '불확실', 미션 결과가 오면 끝낸 항목으로 정리되고 대기열은 멈춘 채 / 두 번째 탭은 빈 대기열, 저장된 운반은 보임 / 거부(`already_there`) → 멈춤, 항목은 맨 앞에 남음. 가로 넘침 없음. 실차·Isaac 연속 운용은 확인하지 않았다.

## 확인한 환경 (2026-10-13)

- 가짜 차량 서버: Python 3.11, websockets 16, Pillow 12 (conda base)
- 웹 시험: Node 22 (`node --test`는 디렉터리가 아니라 글롭을 받는다)
- 화면: 헤드리스 Chromium(Playwright)에서 1440 px·400 px 폭으로 렌더와 흐름(권한 → 작업 시작 → 수동 조종 보류 → 무장 → 누르고 있기 → 포커스 잃음 해제 → STOP → 재무장 → 재개 체크리스트)을 확인했다. Firefox·Safari·실제 폰·게임패드는 아직 확인하지 않았다.
