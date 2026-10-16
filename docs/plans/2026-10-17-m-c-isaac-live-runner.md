# M-c Isaac 실시간 실행기 계획

작성: 2026-10-17 · 상태: **초안(검토 전)** · 근거: [운용 콘솔 설계](../design/2026-10-13-operator-console.md) 5.3·5.4·6·7.2·7.3·9절, [콘솔 계약 초안](../interfaces/2026-10-13-operator-console-contract-draft.md) 1–7·9절, [M-b 계획](2026-10-16-m-b-arbiter.md), [C2a 관찰 게이트웨이 설계](../design/2026-10-15-console-gateway-monitor.md)

## 1. 범위

**목표.** Isaac 안의 지게차를 웹 콘솔(`console/web`)로 실시간 조종·관찰한다. 판정 배치 실행기와 같은 미션 코어·중재기를 쓰되, 벽시계 1배속으로 돌고, 콘솔의 요청을 중재기(M-b)가 처리한다. 설계 6절의 M-c 정의와 계약 9절의 M-c 분담을 이번 판의 할 일로 삼는다.

- 포함
  - 실시간 페이싱: 물리 스텝을 벽시계에 맞춘다. 건너뛰거나 밀린 만큼 몰아서 돌지 않는다. 배속·지연을 재고 미달이면 상태에 싣고 수동 조종을 막는다.
  - 실행기의 실시간 모드(`run_transport.py --live`): 중재기를 주기마다 돌리고(M-b B3 순서), 출력(AUTO 후보, TELEOP 명령, 선점 0)을 바퀴·조향에 넣는다. 판정 배치(`--live` 없음)는 바뀌지 않는다.
  - 콘솔 서버: 같은 프로세스의 별도 스레드에서 console/1(제어·미디어 두 WebSocket)을 연다. 전송 규칙은 `console/wire.py` 를 그대로 쓴다. 노드·포트·`session_epoch` 를 파일로 남긴다(설계 7.2).
  - 미디어: 전방(마스트) 카메라 JPEG(촬영 단위 `frame_id`, 렌더 결과를 받은 차량 단조 시각), LiDAR 스캔, 장애물 격자. 대상 목록(`targets`).
  - `camera_latency_bound_s` 측정: 렌더 요청부터 영상 버퍼를 받기까지의 차량 단조 시각 분포를 실시간·미디어 부하 조건에서 잰다.
  - 잡마다 새 `session_epoch`(예측할 수 없는 값). 같은 잡 안에서 장면을 다시 시작하는 기능은 두지 않는다 — 새 장면은 새 잡이고 새 epoch 다.
- 제외
  - ROS 2 msg/action 과 게이트웨이 노드(C2b, 콘솔 쪽). 이번 콘솔 서버는 실행기 안의 직접 연결이다. C2b 가 생기면 같은 중재기 입출력을 ROS 로 옮긴다.
  - 비동기 포트와 늦은 결과 폐기(M-a 계획 D5 의 "M-c 전 별건"). 이번 판은 동기 포트 그대로 둔다: 계획·캡처가 도는 동안 sim 시각이 멈추고 페이싱이 미달로 표시된다. 만료·lease 는 차량 단조 시계로 계속 흐르므로(설계 5.2) 그 사이 텔레옵은 만료로 멈춘다.
  - 복구 동작 `withdraw_straight`, 수동 승강(M-b 와 같음).
  - 실차(C4)·H3.

**완료 조건.**
1. CPU 시험: 페이싱, 실시간 연결(중재기·코어 연결부), 콘솔 서버(가짜 실행기)의 단위·통합 시험.
2. ws1 Isaac 기능 시험(설계 9절 C3 범위의 메인 쪽 몫, 안전 검증 아님): 스크립트 클라이언트가 콘솔 서버에 붙어 lease → 작업 지시(AUTO) → 일시정지 → TELEOP 진입·무장·주행 → 명령 끊김으로 만료 정지 → STOP → 재무장 → 재개 → 완료를 한 실행에서 거치고, 기록으로 각 전이를 확인한다. 같은 스크립트로 잡을 다시 띄우면 이전 `lease_id`·세대가 `stale_epoch` 로 거부된다. 페이싱을 일부러 늦춘 실행에서 텔레옵 주행이 막힌다.
3. 판정 배치 불변: 재생 29건, ws1 전체 시험(실패 = 알려진 10건).
4. `camera_latency_bound_s` 측정값과 그 조건을 증거 문서에 남긴다.

## 2. 결정

**C1 페이싱(순수, `forklift_core/live/pacing.py`).** `Pacer(target_factor, window_s=1.0, max_lag_s=0.25, min_ratio=0.9)` — `ok` 의 문턱은 `min_ratio × target_factor`.

- 물리 스텝마다 `after_step(sim_t, wall_mono) -> sleep_s`: 목표 벽시각 `anchor_wall + (sim_t − anchor_sim)/target_factor` 까지 남은 시간을 돌려준다(0 이상). 실행기가 그만큼 잔다.
- 뒤처짐(`wall − 목표`)이 `max_lag_s` 를 넘으면 기준점을 지금으로 다시 잡는다(`anchor = (sim_t, wall)`). 그래서 밀린 시간을 따라잡으려 빨리 돌지 않는다(설계 7.3 "몰아서 실행하지 않는다").
- 배속: 최근 `window_s` 벽시간 동안의 `Δsim/Δwall`. `ok` = 창이 차 있고 배속 ≥ `min_ratio × target_factor` 이고 기준점을 다시 잡은 뒤 `window_s` 가 지났다. 상태의 `pacing` 은 `{"realtime_factor", "ok", "lag_s", "resyncs"}`.
- 계획·캡처처럼 오래 걸린 동기 호출 뒤에는 기준점이 다시 잡히고, 창이 다시 찰 때까지 `ok` 가 거짓이다.
- **목표 배속은 설정값이다**(`--live-factor`, 기본 1.0). 실측(검토 ①): 지금 배치 실행 전체의 배속은 0.61 이다(`ma3b_p_obs` 시뮬레이션 223.5 s / 벽시계 368.4 s, 장면 준비·계획·캡처·SLAM 동기 대기 포함). 정상 주행 구간의 배속은 M-c1 의 첫 Isaac 실측으로 정한다. 1.0 을 유지하지 못하면 목표를 낮춰 선언한다. 목표보다 느린 sim 은 같은 벽시간에 덜 움직이므로 조종 지연과 만료(벽시계)에 대해 안전한 쪽이다. 상태에는 목표와 실측을 함께 싣고(`pacing.target_factor`·`realtime_factor`), `ok` 는 선언한 목표 기준으로 판정한다. 목표보다 **빠르게** 도는 일은 페이싱이 막는다.

**C2 중재기 연결부(`sim/isaac/live_link.py`, Isaac 없이 시험 가능).** 실행기 제어 스레드가 소유한다. 콘솔 서버 스레드와는 스레드 안전한 큐 두 개로만 주고받는다.

- 들어오는 것: `(conn_id, client_id, kind, msg)` — 연결·끊김·메시지. 주기 첫머리에 모두 꺼내 중재기에 넣는다(M-b B3 ①). 주기 중간에 도착한 것은 다음 주기로 간다.
- 나가는 것: `(conn_id 또는 broadcast, message)` — 신뢰 요청의 답(같은 연결로), `resolve` 의 미션 요청 답(요청한 연결로; 끊겼으면 버린다 — 재전송하면 저장된 결과를 받는다), `tick`(주기 `tick_period_s`, broadcast), `state`(10 Hz), `targets`(revision 이 바뀔 때).
- 시계: 모든 중재기 호출의 `now_mono` 는 `time.monotonic()`. 미션 코어에는 지금처럼 sim 시각 `t` 를 준다. 둘을 빼지 않는다(계약 1절).
- 주기 순서(M-b B3 그대로): `observe(now, feedback)` → 메시지 처리 → `mission_context(now)` → `core.commands.apply(t, Context(stop_kind, near_tracker))` → 코어 틱 → `resolve` → `step` → 출력 적용.
- 피드백: 바퀴 속도는 실행기가 이미 읽는 값(SLAM 이 있으면 오도메트리 속도 `slam["odom_speed"]`, 없으면 바퀴 관절 속도 × 반경의 평균), 요 각속도는 같은 출처, 측정 시각은 그 값을 읽은 직후의 `time.monotonic()`. 조향 곡률은 실측 조향각에서 Ackermann 역으로.
- 출력 적용:
  - AUTO·PAUSED: 코어가 조립한 명령 그대로(중재기 출력과 같다, M-b B11).
  - TELEOP: 중재기 속도·곡률을 코어와 같은 뒷단(Ackermann 바퀴 목표, 조향 속도 제한)에 넣는다. 그 결과를 `applied` 로 상태에 싣고, 코어 명령 상태 셋(`command_speed`·`applied_command`·`slam["last_command"]`)에 되쓴다(M-b B11).
  - 선점(`STOPPED`·래치): 그 주기부터 물리 스텝마다 바퀴 0, 조향 유지 — 명령 일정의 `preempt` 와 같은 자리(`step_world`)에서 한다.
- 미션 제출: 실시간 모드에서는 배치처럼 스스로 제출하지 않는다. 대상 목록에 시나리오의 팔레트·목적지를 `P-1`·`D-1` 로 올려 두고 조종자의 `mission_submit` 을 기다린다.
- 페이싱 미달: 중재기 정책에 `pacing_ok` 입력을 더한다(M-b 의 작은 확장). 거짓이면 TELEOP 주행 거부(사유 `pacing`), 이미 주행 중이면 최종 상한 0(B8). AUTO 는 막지 않는다 — AUTO 의 안전은 sim 시각 기준의 허가가 지키고, 페이싱은 사람이 화면을 보고 조종할 때의 문제다.

**C3 콘솔 서버(`console/live/`).** 가짜 차량 서버(`console/mock_vehicle/server.py`)의 `Handle` 구조와 `console/wire.py` 를 그대로 쓰고, 차량 객체 자리에 `LiveVehicle` 을 둔다.

- `LiveVehicle.connect/handle/disconnect` 는 큐에 넣고 바로 돌아온다(답은 C2 의 나가는 큐로 온다). `hello` 의 `welcome` 은 서버가 바로 만든다(세션 정적 정보: `session_epoch`, `backend: "isaac"`, `capabilities`).
- `capabilities`: `control: true`, `mission: true`, `teleop_direct`·`teleop_video: true`, `lift: false`, `recovery_actions: []`, `feedback`: 바퀴 속도·조향은 `true`, 승강 높이 `true`(관절 위치), 적재 `false`(단계 추정만 있다), `media` 상한(카메라 10 Hz·640×400, 스캔 10 Hz, 격자 1 Hz), `provisional_limits: true`.
- 미디어는 실행기 스레드가 최신값 저장소에 넣고(카메라 RGB 배열·`frame_id`·`acquired_mono`, 스캔, 격자 스냅숏) 서버 스레드가 구독 상한에 맞춰 꺼내 보낸다. JPEG 인코딩은 서버 쪽 작업 스레드에서 한 번만 한다(설계 7.3). 밀리면 최신 것만 남긴다.
- 포트: `0`(운영체제가 고름)으로 열고 `--live-port-file` 에 `{"host", "port", "session_epoch", "pid", "started"}` 를 쓴다. 노트북 쪽은 이 파일로 `ssh -L` 터널을 만든다(설계 7.2). 웹 정적 파일도 같은 포트에서 준다.

**C4 실행기 통합(`--live`).**

- `--live` 는 `--mission-commands` 와 함께 쓸 수 없다(같은 자리를 다르게 쓴다). `--max-sim-seconds` 는 그대로 상한이다.
- 렌더: 카메라 프레임이 필요한 물리 스텝(구독 상한 10 Hz 이면 12 스텝마다)에만 `render=True`. 영상 기록(`--video`)과 함께 쓸 수 있다.
- `frame_id`: 실행기가 카메라 버퍼를 읽을 때마다 1 씩 늘린다(촬영 단위). `acquired_mono` 는 버퍼를 받은 직후의 `time.monotonic()`. `capture.domain` 은 `sim`, `stamp_ns` 는 그 렌더의 sim 시각.
- 격자는 장애물 층의 스냅숏(코드 0 미확인·1 빈칸·2 점유, `origin_m`), 스캔은 실행기의 2D LiDAR 스캔(미측정은 NaN).
- `state` 의 `pose` 는 제어 자세(뒤 차축, SLAM 이 있으면 `map`·보정 버전), `freshness`(격자 FREE 증거 나이, SLAM 보정 나이), `pacing`.
- 기록: 중재기 이벤트, 받은 요청과 답, 페이싱 표본, 카메라 지연 표본을 `live_log.jsonl` 로 남긴다(실시간 실행은 재생 대상이 아니다 — 경계 기록기를 켜지 않는다).

**C5 카메라 지연 측정.** 렌더 스텝 직전의 `time.monotonic()` 과 버퍼를 받은 직후의 차이를 프레임마다 기록한다. 이것이 "촬영부터 렌더 수신까지"의 Isaac 쪽 값이다. 실시간 실행·미디어 구독이 있는 상태에서 잰 분포의 최댓값에 여유를 더해 `camera_latency_bound_s` 잠정값을 정하고, 그 전에는 M-b 잠정값 0.15 s 를 쓴다. 증거 문서에 분포(중앙값·p99·최댓값)와 조건(해상도·구독 상한·GPU 공유 여부)을 적는다.

**C6 ws1 실행.** Slurm GPU 잡(`--gres=shard:1`, Isaac 하나)으로 띄운다. 판정 배치와 같은 GPU 를 동시에 쓰지 않는다(설계 6절) — 기능 시험 잡은 판정 배치가 없을 때만 낸다. 스크립트 클라이언트(`tools/live_console_probe.py`, `websockets`)는 같은 잡 안에서 실행기 뒤에 띄워 포트 파일을 읽고 시나리오를 돈다. 사람이 노트북에서 붙는 시연은 시험이 아니다.

## 3. 수치

| 이름 | 값 | 근거 |
|---|---|---|
| 물리·제어 주기 | 1/120 s | 실행기 `physics_hz` |
| `tick_period_s` | 0.05 s | ≤ `command_max_age_s/4` = 0.075 s(계약 4.1) |
| 상태 주기 | 10 Hz | 가짜 차량과 같음 |
| 페이싱 `max_lag_s` | 0.25 s | 잠정. 이보다 늦으면 따라잡지 않고 기준점을 다시 잡는다 |
| 페이싱 `min_factor`, 창 | 목표의 0.9 배, 1 s | 잠정 |
| 목표 배속 | 1.0(설정으로 낮춤) | 배치 전체 실측 0.61(위) |
| 카메라 | 10 Hz, 640×400 | 계약 3절 예시 상한 안 |

## 4. 조각

| 조각 | 내용 | 시험 |
|---|---|---|
| M-c1 | `Pacer`, 중재기 정책의 `pacing_ok` | 단위 |
| M-c2 | `live_link.py`(큐·주기·피드백·출력 적용 함수), 코어의 외부 명령 뒷단 함수(Ackermann·조향 속도·되쓰기) | 단위(가짜 코어·가짜 실행기) |
| M-c3 | `console/live/`(LiveVehicle·서버·포트 파일·미디어 저장소) | 통합(실제 WebSocket, 가짜 연결부) |
| M-c4 | `run_transport.py --live` 통합, `tools/live_console_probe.py` | ws1 Isaac 기능 시험, 재생 29건·전체 시험 |
| M-c5 | 카메라 지연 측정과 증거 문서 | ws1 |

각 조각은 Codex 구현 리뷰를 수렴까지 받는다.

## 5. 위험

- 동기 계획·캡처 동안 sim 이 멈춰 텔레옵이 만료로 끊긴다. 이번 판의 의도된 동작이고 화면에 페이싱 미달로 보인다. 근본 해결은 비동기 포트(별건).
- Isaac 이 1배속을 못 낼 수 있다(배치 전체 0.61). 목표 배속을 낮춰 선언하는 길을 C1 에 두었다. 렌더가 10 Hz 에서도 목표를 깨면 카메라 상한을 낮춘다. 측정으로 정한다.
- 콘솔 서버 스레드가 GIL 을 오래 잡으면 제어 주기가 밀린다. JPEG 인코딩은 C 확장(Pillow)에서 GIL 을 놓는다고 보고, 측정한다.
- ws1 터널이 Slurm 실행 노드까지 닿는지(설계 7.2)는 사람이 붙는 시연에서만 필요하다 — 기능 시험은 같은 잡 안 클라이언트로 한다.

## 6. 열린 질문

- 페이싱 기준값(0.9·0.25 s)은 잠정이다. 기능 시험의 배속 분포를 보고 정한다.
- C2b(ROS 게이트웨이)가 생기면 이 직접 연결을 남길지(개발용) 없앨지.

## 7. 자체 검토 기록

(검토 뒤 채운다.)
