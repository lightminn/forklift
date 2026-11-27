# 벤치 콘솔 C — 브라우저 진입점 재생과 기존 화면 비교 (2026-11-28)

- **무엇**: 계획 [`2026-11-27-bench-console.md`](../plans/2026-11-27-bench-console.md) 4 절 완료 조건 2 의 (a)·(a'')·(c) 를 실제 브라우저에서 확인한 기록이다. (a) 는 4.1 에서 node 시험과 브라우저 재생으로 나눴다.
- **검증 종류**: 화면 렌더와 프로토콜. 입력 출처는 `synthetic`(스크립트 백엔드)과 `replay` 가 아닌 모형 실시간 실행(`mcu_model --bench 1`, pty)이다. 실물 보드는 쓰지 않았다.
- **코드**: `feat/bench-console-c` 의 작업 트리(origin/main 207a8608 위)다. 커밋 해시는 PR 본문에 적는다.
- **환경**: 노트북, Chromium(Playwright MCP), 창 1600 × 1000. 백엔드는 모두 `127.0.0.1:8791` 에 차례로 띄웠다. 한 탭이 계속 열린 채(새로고침 없음) 다음 백엔드로 다시 붙는 경로를 봤다.
- **원자료**: `docs/validation/assets/2026-11-28-bench-console-c/` 의 스크린샷 10 장과 백엔드가 받은 메시지 JSONL 8 개. 스크립트 백엔드는 [`2026-11-28-bench-console-c-scripted-vehicle.py`](2026-11-28-bench-console-c-scripted-vehicle.py) 다.

## 1. 실제 벤치 백엔드(B1 + 모형)에서 화면으로 실행

`python tools/bench_console.py --model <mcu_model> --port 8791 --runs-root <폴더>` 로 띄웠다(`--model` 은 이 PR 이 더한 개발용 옵션: 도구가 자기 pty 에 모형을 띄우고 보드 없이 콘솔을 연다).

| 확인 | 결과 |
|---|---|
| welcome 이 벤치 세션이면 '벤치' 메뉴와 화면 DOM 이 생김 | 생김(`bn-rail`, `v-bench`) |
| L 축 듀티 0.1 · 0.6 s 를 화면에서 시작(조종 권한·주 전원 확인 체크) | `bench_run: 받음`, 결과 `ended` · `end_zero` · 실행·정리 정상 |
| M 축(원천 KTH, 범위 켬)에서 창 칸이 보임, 원천 목록 kth / 없음 / mt(비활성) | 그렇다. 창 −1…1 로 실행 → 정상 |
| M 축 `steer_range` 우회 체크 | 창 칸 숨김, '권장: 기구에서 모터를 분리하세요 … 막지는 않습니다' 안내가 보임. 0.15 · 0.6 s 실행 → 정상 |
| 화면 모습 | `after_bench.png`, `after_bench_driving.png` |

## 2. ACK 유실 뒤 재연결(4 절 (a''))

스크립트 백엔드 `bench-mute`(벤치 welcome·bench_state, 벤치 요청은 기록만 하고 답하지 않음)를 썼다.

1. 첫 서버(epoch 4242, 6 s 뒤 연결 닫음)에서 `bench_run` 을 보냈다. 같은 연결 안에서는 500 ms 마다 같은 id 로 재전송됐다(9 회, `replay1.jsonl`). 이는 계약대로다.
2. 연결이 닫힌 뒤 같은 epoch 의 새 서버를 띄웠다. welcome 을 3 s 늦게 보냈다(재전송 간격보다 길다). 그 서버가 받은 메시지는 `hello`·`lease_request`·`echo`·`media_subscribe` 뿐이었다. **`bench_run` 은 0 건**이다(`replay2.jsonl`).
3. 바뀐 epoch(5151)로 같은 순서를 반복했다(`replay3.jsonl` → `replay4.jsonl`). 새 서버가 받은 것은 `hello`·`media_subscribe` 뿐이었다. **`bench_run` 0 건**이고, 바뀐 epoch 이라 lease 회수 요청도 없다.

## 3. 같은 탭에서 REAL / Isaac / HIL 로 전환(4 절 (a))

벤치 화면을 보던 탭에서 백엔드를 차례로 바꿨다.

| 백엔드 | 메뉴 | 벤치 DOM(`bn-*`) | REAL 패널 | 화면 | STOP 버튼 / Escape | 벤치 메시지 |
|---|---|---|---|---|---|---|
| REAL(`real_status`) | 넷 | 없음 | 보임 | 모니터링으로 복귀(벤치를 보던 중이었음) | `stop` source `console` / `key` | 0 (`replay_real.jsonl`) |
| Isaac(`state`) | 넷 | 없음 | 숨김 | — | `console` / `key` | 0 (`replay_isaac.jsonl`) |
| HIL(`real_status`) | 넷 | 없음 | 보임 | — | `console` / `key` | 0 (`replay_hil.jsonl`) |
| 다시 벤치 | 다섯 | 새로 생김(우회·확인 체크 해제 상태) | — | — | 화면의 큰 STOP → 공용 `stop`, source `bench` | `replay_bstop.jsonl` |

## 4. 기존 화면 PR 전후(4 절 (c))

가짜 차량(`python -m console.mock_vehicle --port 8791`)으로 화면 넷을 PR 전(main 의 프런트)과 후에 찍었다. 지도·격자·LiDAR 점·카메라·측면 패널에 데이터가 있는 상태다.

- 파일: `before_*.png` / `after_*.png`. 화면은 모니터링·이벤트·상태·통신 넷이다.
- 비교 결과: 배치·요소·문구가 같다. 다른 것은 실행마다 바뀌는 값뿐이다.
  - 조종 권한: 전 '없음', 후 '이 기기'. 새로고침한 탭이 회수했다.
  - 세션 epoch, 카메라 프레임 번호, tick 간격.
  - 그 결과로 열리고 닫히는 버튼: 세션 종료·장면 초기화·작업 지정.
- 메뉴에 '벤치' 는 없다.

## 5. 남는 것

- 브라우저 재생은 자동 시험이 아니다(CONTRIBUTING 7: 브라우저 화면은 자동 시험 범위 밖, 렌더 확인을 남긴다). 같은 전환은 node 시험(`console/web/test/bench.test.mjs`)이 실제 `createSession` 으로 자동 재생한다.
- 실물 보드에서 화면으로 하는 실행은 사용자가 요청하는 실보드 확인 묶음 때 한다.
