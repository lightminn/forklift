# 운용 콘솔

[설계](../docs/design/2026-10-13-operator-console.md) · [계약 초안 console/1](../docs/interfaces/2026-10-13-operator-console-contract-draft.md)

- `web/`: 브라우저 콘솔(빌드 없음, ES 모듈). 화면 구성은 차량 관제 화면(FMS) 형식을 따른다: 왼쪽 메뉴(모니터링 · 이벤트 기록 · 통신), 가운데 지도(끌기 이동 · 휠/두 손가락 확대 · 차량 따라가기 · 표시 층 선택)와 그 위에 뜬 전방 카메라 창, 오른쪽 차량 상세(진행 막대 · 권한/모드/작업 상태 · 현재 목적지 · 위치 · 작업 단계)와 수동 조종 탭. 작업은 '작업 지정' 창에서 작업 종류 → 대상 설정 → 확인 세 단계로 낸다. 작업 종류는 차량이 `capabilities`에 지원한다고 밝힌 것만 보이며, console/1 에는 '팔레트 운반' 하나뿐이다. 지도에서 대기 중인 팔레트를 누르면 그 팔레트를 고른 채로 2단계가 열린다.
- `mock_vehicle/`: 계약 초안을 흉내 낸 **가짜 차량**. 실차·Isaac·중재기(M-b)가 아니다. 여기서 통과한 규칙 시험은 콘솔 개발용이며 차량 쪽 안전의 증거가 아니다.

## 실행

    python -m pip install -e '.[console]'
    python -m console.mock_vehicle --port 8765
    # 브라우저: http://localhost:8765/

게임패드는 브라우저가 보안 컨텍스트로 보는 `localhost`에서만 확실히 동작한다. 웹 시험에는 Node 18 이상이 필요하다.

가짜 차량의 한계: 정지 확인을 센서가 아니라 자기가 낸 적용 명령으로 하고(실차는 잡음 있는 피드백을 걸러야 한다), 모든 요청을 한 프로세스 안에서 즉시 처리한다(실차는 게이트웨이 → 중재기 → 미션 실행부가 비동기로 나뉘고, 결과 캐시의 주인은 중재기다). 상태의 `applied`도 slew·허가 상한·안전 선점까지만 거친 값이다(실차는 Ackermann 바퀴 속도·조향 속도 한계까지 거친다). 위치는 시뮬레이션 정답(`source: mock_truth`)이다. 이 차이는 C2·M-b에서 다룬다. 다른 기기에서 접속할 때는 `ssh -L 8765:localhost:8765 <호스트>` 뒤 `http://localhost:8765/`를 연다.

## 시험

    python -m pytest tests/unit/console tests/integration/console -q -p no:cacheprovider -W error
    node --test "console/web/test/*.test.mjs"

## 확인한 환경 (2026-10-15, FMS 형식 화면)

- 헤드리스 Chromium(Playwright)에서 1600 px · 1024 px · 400 px 폭으로 확인했다. 확인한 흐름은 권한 → 작업 지정 3단계 → 작업 시작(AUTO, 경로·진행 막대·현재 목적지) → 수동 조종 시작 → 무장 → 키보드 누르고 있기(예상 궤적 표시) → STOP → 재무장 → 재개 체크리스트 → 취소, 지도 휠 확대 · 끌기 · 따라가기 · 맞춤 보기, 지도에서 팔레트 눌러 작업 지정 열기, 새로고침 뒤 조종 권한 회수다. 폰 폭에서 가로 넘침은 없다.

## 확인한 환경 (2026-10-13)

- 가짜 차량 서버: Python 3.11, websockets 16, Pillow 12 (conda base)
- 웹 시험: Node 22 (`node --test`는 디렉터리가 아니라 글롭을 받는다)
- 화면: 헤드리스 Chromium(Playwright)에서 1440 px·400 px 폭으로 렌더와 흐름(권한 → 작업 시작 → 수동 조종 보류 → 무장 → 누르고 있기 → 포커스 잃음 해제 → STOP → 재무장 → 재개 체크리스트)을 확인했다. Firefox·Safari·실제 폰·게임패드는 아직 확인하지 않았다.
