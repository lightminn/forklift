# 운용 콘솔

[설계](../docs/design/2026-10-13-operator-console.md) · [계약 초안 console/1](../docs/interfaces/2026-10-13-operator-console-contract-draft.md)

- `web/`: 브라우저 콘솔(빌드 없음, ES 모듈)
- `mock_vehicle/`: 계약 초안을 흉내 낸 **가짜 차량**. 실차·Isaac·중재기(M-b)가 아니다. 여기서 통과한 규칙 시험은 콘솔 개발용이며 차량 쪽 안전의 증거가 아니다.

## 실행

    python -m pip install -e '.[console]'
    python -m console.mock_vehicle --port 8765
    # 브라우저: http://localhost:8765/

게임패드는 브라우저가 보안 컨텍스트로 보는 `localhost`에서만 확실히 동작한다. 웹 시험에는 Node 18 이상이 필요하다.

가짜 차량의 한계: 정지 확인을 센서가 아니라 자기가 낸 적용 명령으로 하고(실차는 잡음 있는 피드백을 걸러야 한다), 모든 요청을 한 프로세스 안에서 즉시 처리한다(실차는 게이트웨이 → 중재기 → 미션 실행부가 비동기로 나뉘고, 결과 캐시의 주인은 중재기다). 이 차이는 C2·M-b에서 다룬다. 다른 기기에서 접속할 때는 `ssh -L 8765:localhost:8765 <호스트>` 뒤 `http://localhost:8765/`를 연다.

## 시험

    python -m pytest tests/unit/console tests/integration/console -q -p no:cacheprovider -W error
    node --test console/web/test/
