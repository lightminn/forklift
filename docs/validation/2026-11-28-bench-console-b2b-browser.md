# 벤치 콘솔 B2b — 화면의 위치 유지·바퀴 수 정지 칸, 브라우저 재생과 기존 화면 비교 (2026-11-28)

- **무엇**: 계획 [`2026-11-27-bench-console.md`](../plans/2026-11-27-bench-console.md) 4.4 결정 6(화면)과 4 절 완료 조건 2 를 실제 브라우저에서 확인한 기록이다. 방법은 C 의 기록([`2026-11-28-bench-console-c-browser.md`](2026-11-28-bench-console-c-browser.md))과 같다.
- **검증 종류**: 화면 렌더와 프로토콜. 입력 출처는 모형 실시간 실행(`mcu_model --bench 1`, pty)과 스크립트 백엔드(`synthetic`)다. 실물 보드는 쓰지 않았다.
- **코드**: `feat/bench-console-b2b`(origin/main 480342e6 위). 커밋 해시는 PR 본문에 적는다.
- **환경**: 노트북, Chromium(Playwright MCP), 창 1600 × 1000, 백엔드는 모두 `127.0.0.1:8791` 에 차례로 띄웠고 한 탭이 계속 열린 채 다시 붙었다.
- **원자료**: `docs/validation/assets/2026-11-28-bench-console-b2b/` — 스크린샷 10 장, 백엔드가 받은 메시지 JSONL 7 개(`replay_real`·`replay_isaac`·`replay_hil`, `r2_same_a/b`, `r2_epoch_a/b`). 스크립트 백엔드는 C 의 [`2026-11-28-bench-console-c-scripted-vehicle.py`](2026-11-28-bench-console-c-scripted-vehicle.py) 그대로다.

## 1. 실제 벤치 백엔드(B2a + 모형)에서 화면으로 실행

`python tools/bench_console.py --model <감싼 mcu_model> --port 8791 --runs-root <폴더>` 로 띄웠다. 감싼 모형은 HIL 시험처럼 `--kth-start-raw 1000,1000` 을 붙인다(모형의 KTH 가 원시 0 에서 시작하면 hold 의 ARM 이 거부된다 — 모형 조건이지 코드 문제가 아님).

| 확인 | 결과 |
|---|---|
| L 축에서 '위치 유지' 선택지 | 비활성(KTH 원천이 있는 M·H 에서만 열림) |
| M · 위치 유지 · 지금 자리에서 옮겨 고정 60 카운트 · 감쇠 0.02 · 센서 방향 +1 · 3 s | `bench_run: 받음`, '목표에 머묾 · 끝 오차 12 카운트 · 허용 오차 안 100%', 실행·정리 정상, 첫 명령 `accepted`. 그래프에 '목표까지 오차' 선(`after_hold_driving.png`) |
| 위치 유지에서 칸 이름 | 출력 상한(선택)·목표·값·세기(비례)·누적 보정(적분)·감쇠·허용 오차(카운트)·최소 출력, 기본값 한 줄, '감쇠는 실물 미시험 — 0 과 0.02 를 비교해 보세요', 실물 확인 범위 '해당 없음' |
| M · 출력 0.2 · N 바퀴에서 정지 0.5 · 센서 방향 없이 시작 | 화면에서 '입력 확인: count_sign' (보내지 않음) |
| 같은 실행에 센서 방향 +1 | 모형에서 1 s 안에 0.5 바퀴에 못 미쳐 '도달 전에 끝남 · 목표 2048 · 차단 때 123 카운트'(모형의 느린 축, CLI 시험도 0.02 바퀴를 씀) |
| 0.02 바퀴 | '도달 · 목표 82 · 차단 때 82 카운트', 정지 원인 `distance_reached`, 실행·정리 정상, 실물 확인 범위 '아직 없음(5.1 결과 전)' (`after_distance.png`) |
| 이동량만 재기 0.2 · 0.8 s | '97 카운트 (confirmed)', 실행·정리 정상 |
| 축 M → H → L, 원천 KTH → 없음 | 센서 방향·바퀴 수 칸이 비워짐, L 과 원천 없음에서 위치 유지·정지 칸이 닫힘 |

화면 확인 중에 고친 것(같은 PR): 시작 버튼 문구가 새 문구 키에 덮이던 것, 결과의 '허용 오차 안' 비율이 빠지던 것, 해당 없는 결과 줄이 'null' 로 찍히던 것, 그리고 **센서 방향 선택지 순서** — 객체가 정수 같은 키("1")를 빈 키보다 앞에 두어 축을 바꿔도 방향이 +1 로 남던 것을 고정 순서로 바꿨다.

## 2. 같은 탭에서 REAL / Isaac / HIL 로 전환(4 절 (a))

| 백엔드 | 메뉴 | 벤치 DOM(`bn-*`) | STOP 버튼 / Escape | 벤치 메시지 |
|---|---|---|---|---|
| REAL(`real_status`) | 넷 | 0 | `stop` source `console` / `key` | 0 (`replay_real.jsonl`) |
| Isaac(`state`) | 넷 | 0 | `console` / `key` | 0 (`replay_isaac.jsonl`) |
| HIL(`real_status`) | 넷 | 0 | `console` / `key`(재전송 포함) | 0 (`replay_hil.jsonl`) |
| 다시 벤치(B2a + 모형) | 다섯 | 새로 생김(우회·확인·센서 방향 비움, 모드 듀티) | — | — |

## 3. ACK 유실 뒤 재연결(4 절 (a''))

스크립트 백엔드 `bench-mute`(보드 FAULT, `--mute-stop`). 연결 A(epoch 4242, 8 s 뒤 닫음)에서 화면의 시작·고장 해제(원인 'replay')·패널 STOP 을 눌렀다(A 에서 패널 STOP 의 source 는 `bench`). 바로 다음 소켓 B 는 welcome 이 3 s 늦다.

- 같은 epoch(4242, `r2_same_a/b`): B 의 welcome 전에 받은 것은 A 의 그 id 의 `stop` 뿐, B 전체에서 `bench_run`·`clear_faults` 0 건, lease 는 B 에서 다시 요청.
- 바뀐 epoch(5151, `r2_epoch_a/b`): welcome 전에는 같은 id 의 `stop` 뿐, `bench_run`·`clear_faults` 0 건, lease 회수 요청 없음.

C 의 기록과 같다.

## 4. 기존 화면 PR 전후(4 절 (c))

가짜 차량(`python -m console.mock_vehicle --port 8791`)으로 화면 넷(모니터링·이벤트·상태·통신)을 main(480342e6)의 프런트(`--web-root` 로 main 의 `console/web` 을 따로 내보냄)와 이 PR 의 프런트로 찍었다(`before_*.png` / `after_*.png`).

- 배치·요소·문구가 같다. 바뀐 것은 실행마다 다른 값뿐이다: 세션 epoch·control, 경보 목록, 그리고 'PR 전' 가짜 차량이 이미 정지 상태(재무장 필요)였던 것(그래서 '정지' 줄이 두 줄이고 경보가 다름).
- 메뉴에 '벤치' 는 없다.

## 5. 남는 것

- 브라우저 재생은 자동 시험이 아니다(CONTRIBUTING 7). 화면 모델은 node 시험(`console/web/test/bench.test.mjs`, 위치 유지·바퀴 수·측정 요청, 부호 필수, 축·원천 전환 때 비움, 결과 보기, 그래프의 오차 계열)이 자동으로 본다.
- 실물 보드에서 화면으로 하는 실행은 사용자가 요청하는 실보드 확인 묶음(계획 5.1) 때 한다.
