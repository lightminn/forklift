# 운용 콘솔

[설계](../docs/design/2026-10-13-operator-console.md) · [계약 초안 console/1](../docs/interfaces/2026-10-13-operator-console-contract-draft.md)

- `web/`: 브라우저 콘솔(빌드 없음, ES 모듈)
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

## 확인한 환경 (2026-10-13)

- 가짜 차량 서버: Python 3.11, websockets 16, Pillow 12 (conda base)
- 웹 시험: Node 22 (`node --test`는 디렉터리가 아니라 글롭을 받는다)
- 화면: 헤드리스 Chromium(Playwright)에서 1440 px·400 px 폭으로 렌더와 흐름(권한 → 작업 시작 → 수동 조종 보류 → 무장 → 누르고 있기 → 포커스 잃음 해제 → STOP → 재무장 → 재개 체크리스트)을 확인했다. Firefox·Safari·실제 폰·게임패드는 아직 확인하지 않았다.
