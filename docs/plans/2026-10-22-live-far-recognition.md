# live 실행기의 원거리 인식 — 연속 이송 CT-1 PR ① (3판)

브랜치 `feat/continuous-transport`. 상위 계획 `docs/plans/2026-10-22-continuous-transport-ct1.md` 3절 PR ①. 단일 팔레트 장면에서 상시 운용 콘솔(`--live`)이 원거리 인식(`--recognition far`, 학습 제안 포함)을 쓰게 한다. 다중 팔레트는 PR ②·③.

## 1. 사실 (조사·소스)

- 거부: `--recognition far` 는 "batch run (the live capture path is out of its scope)"으로 live 를 거부한다(`run_transport.py` 1100–1101행). 원 계획(`docs/plans/2026-10-18-isaac-recognition-far-capture.md` 128행)은 0.2 s 마다 렌더 + 인식이 실행을 느리게 해 live 를 범위 밖에 두었다. 밑판 검사도 live 를 거부한다(917–918행) — 이번에도 그대로 둔다.
- 캡처:
  - 배치 `perception_capture.capture()` 는 `_paused_capture` 로 감싸 `far_runtime.paused=True` 동안 주행 프레임을 기록하지 않는다(3155–3165행).
  - live 는 `.session()` 으로 틱마다 렌더 한 걸음씩 캡처한다(core 3676–3708행, 러너 7398–7403·7954–7960행). 여기에는 그 보호가 없다.
  - 의미 분할(`semantic_fn`)은 두 경로 모두 채운다.
- 주행 프레임 기록:
  - 조건은 `ms.phase == "observe"`·근접 캡처 아님·밑판 옆면 다리 아님(5774–5777행)이다. 미션 상태(`commands.status`)는 보지 않는다.
  - 미션 끝에 phase 를 되돌리지 않으므로(`phase` 는 `MS_FIELDS` 밖), 쉬거나 멈춘 세션이 "observe" 로 남아 있으면 계속 기록할 수 있다(코드에서 추론, 실행 확인 안 함).
- 정지 판단: `far_hook` → `far_runtime.stop` 이 다리를 넘긴다(`next_leg()`, `far_recognition_runtime.py` 326행). live 의 `observation_take` 가 관측을 버리면(미션·phase·보정 변경, 7341–7364행) 다시 캡처할 때 주행 프레임 없는 새 다리라 "single" 이 된다.
- 시간(지금 코드):
  - 원거리 일은 제어 스레드에서 동기로 돈다. `drive_frame` 은 24 스텝마다, `stop` 은 캡처를 소비하는 틱에서 돈다.
  - 학습 제안은 `WorkerProposer.propose` 가 기한까지 막는다.
    - 기한은 `--recognition-timeout-s` 기본 10 s, 첫 호출은 120 s.
    - 기한을 넘기면 작업자를 죽이고 기하 제안으로 대신하며, 다시 띄우지 않는다.
  - 미션 안 왕복 측정(`docs/validation/2026-10-19-recognition-stage3-learned-proposals.md`): CPU p50 115–129 ms·최대 232 ms, GPU p50 42–44 ms·최대 98–171 ms.
  - live 틱 하나는 1/120 s 물리를 벽시계 (1/120)/배속에 돈다(배속 0.4 에서 20.8 ms). 원거리 호출 동안 틱(링크·게이트·발행·물리)이 멈춘다. `persistent-live-console.md` 514행에 0.54 s "인식" 틱이 박자 지연으로 기록돼 있다.
- 상태 수명:
  - `FarRuntime` 에서 미션과 상관없이 남는 것: 단계 deque, 다리 단계·프레임·기록·시도 목록(상한 없음), 다리 종료·카운터, 구역 중심·반경과 변경 기록, `paused`.
  - `FarRecognition` 에서 남는 것: 트랙·`_chosen`·선택 이력. 이것은 `start_leg` 에서만 비워진다.
  - 그래서 관측 도중 끝난 미션의 선택이 다음 미션의 정지에서 `target_lost`·`target_inconsistent` 를 낼 수 있다.
  - 프레임 파일 이름은 `frame_{len(frame_records):04d}.npz` 다. 목록을 비우면 같은 이름을 덮어쓴다.
  - `MissionBaseline` 에는 인식 키가 없다.
  - 구역은 설정 때 한 번 정해지고, live 미션의 `pickup_zone` 은 FarRuntime 에 가지 않는다.
- 픽셀 지연: 배치 24 스텝(직전 렌더), live 실측 11 스텝(`--live-camera-pixel-lag-s` 0.2 s, 807행)(`docs/validation/2026-10-19-isaac-recognition-far-capture.md` 60행).
- 실패:
  - 원거리 정지의 `no_pallet` 계열은 `observe_failed` 로 다음 후보로 간다(core 2376행). 다 쓰면 `observe:all_candidates_exhausted:...`, `not_pickable:fork_spacing_adjust` 는 바로 끝난다.
  - live 는 RuntimeError 를 잡아 분류한다(`failures.py`). `observe:`·`not_pickable:`·`perception_capture_failed`·`observation_discarded`·`near_capture_` 는 미션 실패이고, 모르는 접두는 세션 실패다.
  - 원거리 코드의 RuntimeError 가 아닌 예외(예: `decode_mission_semantic` 의 ValueError, 160행)는 잡히지 않아 세션을 끝낸다.
  - `observe_failed` 의 나머지 후보 재계획은 live 에서도 틱 안에서 동기로 돈다.
- truth: 원거리 truth·지연 맞춤은 `pallet` 손잡이와 인스턴스 0 을 쓴다(단일 팔레트라 이번엔 맞다).

## 2. 범위

- live 에서 `--recognition far` 허용(`--recognition-proposals learned` 포함). 밑판 검사·`--near-tracking`·`--pocket-check` 와의 조합 거부는 지금 그대로.
- 캡처 보호: live 캡처 세션(`capture_begin` → `capture_stepped` → 끝/버림) 동안 `far_runtime.paused=True`.
- 주행 프레임 기록 조건(`far_due` 와 `record_step` 둘 다): "활성 미션이 observe 단계이고 이 물리 스텝의 게이트가 STOP 을 받지 않음". `record_step` 의 무제한 `leg_steps` 도 이 조건과 다리 수명으로 묶는다(Codex 2차 — `paused`·`due=False` 에서도 1,200 회 호출 뒤 1,200 개). 쉬는·멈춘·취소 중인 세션은 기록하지 않는다(live 만 — 배치는 그대로).
- 미션마다 초기화(live 미션 시작 `live_mission_start`):
  - 새 다리를 연다(트랙·선택 비움).
  - 구역을 그 미션의 구역으로 둔다(`reset_for_mission` 이 구역 초기값과 변경 기록을 새로 정한다 — 지금 `set_zone(centre_xy, radius_m)` 은 초기값 인자가 없다). 사전 팔레트는 장면의 원래 구역, 인식·하역된 팔레트는 현재 추정 위치를 중심으로 같은 반경(상위 계획 Codex 2차 — P-1:D-1 뒤 P-1:D-0 에서 원래 자리를 찾지 않게).
  - 기록 디렉터리를 `recognition/mission_<id>/` 로 바꾼다. 다리·시도 목록은 미션 디렉터리마다 새로 시작한다.
  - 기록 소유:
    - `state["recognition"]` 에는 세션 라벨(설정 때 쓰는 `labels`, 2963행)이 함께 있으므로 키 전체를 미션 키로 만들지 않는다(agy).
    - 미션 요약은 새 미션 키 `state["recognition_mission"]` 이다. 미션 시작에 연결하고, 모든 종료 경로(완료·실패·취소·정지 중 끝)에서 최종 요약을 쓴다. 첫 정지 전에 끝난 미션도 주행 프레임 수를 갖는다(Codex 2차).
    - 학습 통계(`LearnedSource.times_ms`·`fallbacks`)는 미션별로 새 목록을 만들어 넘긴다(넘긴 목록을 `.clear()` 하지 않는다).
    - 프레임 기록은 live 의 제한된 쓰기 스레드(`io_writer`)로 넘긴다. 큐가 차거나 쓰기가 실패하면 그 미션의 `record_error` 다.
- 관측 버림 뒤 재캡처: `observation_take` 가 관측을 버리면 그 원거리 정지를 되돌리지 않고 `recognition_discarded` 로 기록한다. STOP·취소로 관측을 지우는 경로(7325행)도 같이 센다. 다음 캡처는 새 다리라 주행 프레임 없이 판단한다(정지 한 번만으로도 `valid/fused` 가 될 수 있다 — Codex 2차 P3, "반드시 single" 철회).
- **시간 (결정 T1, 2판):** 동기 호출은 쓸 수 없다(Codex 2차 P1).
  - 학습 작업자 대기(`worker.py` 106행, 미션 중 기본 10 s) 동안 STOP 처리와 상태 발행이 멈춘다.
  - 콘솔 대기열은 상태가 2 s 오지 않으면 연결을 잃은 것으로 보고 멈춘다(`queue.js` 8행 `SILENCE_MS`).
  - 그래서 원거리 인식의 무거운 일(제안 + 장면 맞춤 + 프레임 기록)을 제어 스레드 밖에서 돌린다.
  - **(A) 권고 — 별도 스레드 + 꼬리표 적용.**
    - 주행 프레임과 정지 캡처의 일을 단일 작업 스레드에 넘기고, 틱은 계속 돈다. 일에는 (미션 id, 다리 번호, 프레임 순번) 꼬리표를 붙인다.
    - 결과는 틱에서 순서대로 트랙에 적용한다. 지난 미션·다리의 결과는 버리고 그 사실을 센다.
    - 정지 판단은 대기 작업이다. 그 다리의 앞선 주행 프레임 일이 모두 끝나야 결정하고, 그동안 차는 서서 틱을 돈다. 결정이 나면 지금의 `observation_take` 흐름으로 들어간다.
    - 캡처·대기 중 STOP·취소는 그 일을 버림으로 표시한다(작업자는 끝까지 돌 수 있다).
  - **(A) 의 계약(3판, Codex 4차 P2 다섯):**
    1. 관측 문맥 고정:
       - 정지 캡처의 raw 를 받아들인 순간(그 캡처의 SLAM 해제·제어 표본이 확정된 때)에 관측 문맥을 저장하고, 추론을 제출한 뒤 완료까지 그대로 넘긴다. 문맥은 보정 버전·`mission_gen`·취득 시각·제어 표본·취득 자세다.
       - `observation_freeze` 를 완료 뒤에 부르지 않는다. 그 사이 보정이 바뀌면 지금의 D9c 처럼 `correction` 으로 버린다.
       - 꼬리표에는 미션 id 대신 `mission_gen` 을 쓴다. 정지·텔레옵·재개는 같은 미션에서도 세대를 바꾼다(`commands.py` 226행).
    2. 소유 경계:
       - 작업 스레드는 프레임 하나의 순수 일(`recognition_bridge.py` 130행의 제안 + 장면 맞춤)을 하고, 그 프레임의 제안·맞춤·호출 시간·대체 여부·저장 내용을 돌려준다.
       - 트랙·선택·다리·미션 기록·학습 통계 목록은 틱만 바꾼다. `LearnedSource` 의 공유 `last`·`times_ms`·`fallbacks` 갱신을 반환값으로 바꾼다.
       - 취득 자세(`pose_at`)는 제출할 때 계산해 함께 넘긴다. 늦게 계산하면 256 스텝 deque 에서 사라진다.
    3. 다리 전환과 대기 중 정지:
       - 앞선 주행 결과를 적용하다 다리를 잃으면(`leg_lost` → `next_leg`), 그 다리에 걸린 대기 중 정지 판단은 무효다. 정지 일은 버린 것으로 세고, 대기를 끝낸다.
       - 서 있는 그 자리에서 새 다리로 정지 캡처를 한 번 다시 한다(재캡처 1 회, 새 다리 = 주행 프레임 없음). 두 번째도 무효면 관측 실패(`observe_failed` → 다음 후보)로 넘긴다.
       - 정지를 지금 다리로 다시 표기하지는 않는다.
    4. 상한:
       - 입력 큐: 주행 프레임 최대 4 개(배속 0.4 에서 약 1 s 분량). 가득 차면 가장 오래된 아직 안 돈 주행 프레임 일을 버리고 센다. 정지 일은 버리지 않는다.
       - 완료 큐: 틱마다 비운다.
       - STOP·취소·미션 세대 변경: 아직 안 돈 일을 지운다. 이미 돌고 있는 일의 결과는 꼬리표로 버린다.
       - 정지 판단 하나의 전체 대기 기한은 30 s(주행 프레임 4 개 + 정지 일, 학습 RPC 10 s 와 기하 대체·맞춤 포함)다. 넘기면 미션 실패 `recognition_timeout` 이다.
       - 대기는 D8c 처럼 `begin_wait/end_wait` 를 짝짓는다(core 5681행).
    5. 재계산 계약:
       - 미션 기록에 프레임마다 적용·폐기 상태와 이유(늦음·STOP·세대·다리·큐 넘침), 적용 순서, 다리 전환을 남긴다.
       - `tools/recompute_far_recognition.py` 는 미션 디렉터리를 받아 적용된 프레임만 그 순서로 다시 적용한다. 시험 경우: STOP 직전 완료, 이전 미션의 늦은 완료, 다리 경계 폐기.
  - (B) 대안 — 틱 안 짧은 기한(live 0.5 s, 넘기면 작업자 재시작과 그 프레임은 기하 제안). 상태 발행 간격 상한은 0.5 s + 장면 맞춤 시간이다. 시뮬에서는 물리도 멈춰 시뮬 시각상 STOP 지연이 없다. 그러나 벽시계 지연과 프레임 단위 품질 저하가 남고, live 의 디스크 비대기 계약(`io_writer.py` 3행)도 프레임 기록에서 어긴다.
  - 학습 작업자의 첫 호출 기한 120 s 는 지금 초기화의 `providers()` 가 소비한다(`run_transport.py` 3132행, Codex·agy) — 첫 미션 추론은 10 s 기한이다. (A) 에서는 기한이 틱을 막지 않는다.
- 픽셀 지연: 11 스텝은 콘솔 영상 카메라의 live 실측이라 인식 카메라·원거리 렌더 조합으로 옮겨지는지 확인되지 않았다. PR ① 의 Isaac live 에서 원거리 인식의 `lag_fit` 으로 지연을 다시 재고, 그 값으로 실행 스크립트를 정한다(Codex 2차·agy).
- 예외:
  - 예외 경계는 세 곳이다: 정지 캡처의 의미 분할 해석(`perception_adapter.py` 638행 `semantic_fn` — 지금은 `CaptureFailure` 만 잡는다), 주행 프레임의 의미 분할 해석과 `drive_frame`, 작업 스레드.
  - 미션 실패 `recognition_error:<형식>` 으로 바꾸고, 캡처 보호 훅은 반드시 닫는다(`try/finally`).
  - 센서 계약 오류(카메라 메타데이터 없음 등)는 지금처럼 세션 실패로 둔다.
  - 주행 프레임은 물리 스텝 뒤에 처리한다. 그래서 그 예외를 미션 실패로 바꿀 때 `live_stepped_` 를 먼저 세워 물리를 두 번 진행하지 않는다. 생략된 센서·발행 처리를 마무리하는 순서를 시험으로 고정한다(Codex 2차 P2).
  - 새 접두는 `failures.py`(미션)·`results.js`(문구)에 등록한다.
- 관측 실패 뒤 다음 후보 재계획(`observe_failed`, 틱 안 동기)은 live 에서 계획 작업자의 `observe_ladder` 일로 보낸다. 같은 시간 계약 때문이다.
- `paused` 고착 방지: 캡처 보호는 세션 객체의 열림·닫힘에 묶는다. 미션 끝·세션 끝·예외에서 닫히지 않은 세션은 미션 끝 정리(`abandon_jobs`)가 닫는다(agy).

## 3. 작업 (시험 먼저)

1. 러너 parser: live 의 `--recognition far` 거부를 푼다(밑판·근접 추적·포켓 검사 조합 거부는 유지). 새 인자 없음. (A) 의 작업 스레드는 live 에서만 쓰고, 배치는 지금처럼 동기 호출이다.
2. 순수 일과 상태 갱신 분리(`recognition_bridge.py`·`far_recognition_runtime.py`):
   - 프레임 하나의 순수 일 = 제안 + 장면 맞춤 → 반환값(제안·맞춤·호출 시간·대체 여부·저장 내용).
   - 상태 갱신 = 트랙·선택·다리·기록·통계. 틱에서 그 반환값으로만 한다.
   - 배치의 동기 경로는 같은 두 함수를 바로 이어 불러 지금과 같은 결과·기록을 낸다.
3. live 작업 스레드(`FarWorker`):
   - 입력 큐 상한 4, 주행 프레임은 넘치면 가장 오래된 것을 버리고, 정지 일은 버리지 않는다.
   - 꼬리표 (`mission_gen`, 다리, 프레임 순번).
   - STOP·취소·세대 변경에서 아직 안 돈 일을 지운다.
   - 완료 결과는 틱에서 꺼내 적용·폐기한다.
4. 대기 중 정지:
   - raw 를 받아들일 때 관측 문맥을 고정하고 정지 일을 넘긴다.
   - `begin_wait/end_wait`, 30 s 기한(`recognition_timeout`).
   - 다리를 잃으면 재캡처 1 회, 두 번째도 무효면 관측 실패.
   - 결정이 나면 고정한 문맥으로 지금의 관측 채택 흐름(`observation_take`)에 넣는다.
5. 캡처 보호: 캡처 세션 객체의 열림·닫힘에 `far_runtime.paused` 를 묶고, `abandon_jobs` 가 남은 세션을 닫는다.
6. 기록 조건: `far_due`·`record_step` 에 live 면 활성 observe 미션이고 이 물리 스텝의 게이트가 STOP 을 받지 않았다는 조건. `leg_steps` 는 다리 수명으로 묶는다.
7. 미션 초기화 `FarRuntime.reset_for_mission(...)`: 새 다리·구역(사전 = 원래 구역, 인식·하역 = 추정 기준)·미션 디렉터리·새 통계 목록. `live_mission_start` 에서 부르고, 미션 요약 `recognition_mission` 을 모든 종료 경로에서 마감한다.
8. 프레임 기록은 `io_writer` 로, 프레임별 적용·폐기 기록과 재계산 도구의 미션 디렉터리·적용 순서 지원.
9. 예외 경계(정지 캡처의 의미 분할 해석, 주행 프레임 해석과 `drive_frame`, 작업 스레드) → `recognition_error:<형식>`. `live_stepped_` 를 먼저 세운다. `failures.py`·`results.js` 에 등록한다.
10. 관측 실패 뒤 후보 재계획: live 에서 계획 작업자의 `observe_ladder` 일로.
11. 시험:
    - parser: live + far 허용, 밑판·근접 추적·포켓 검사 조합은 그대로 거부.
    - 분리: 배치 동기 경로가 고정 입력(저장 프레임·고정 제안)에서 origin/main 코드와 같은 결과·기록.
    - 작업 스레드: 꼬리표 폐기(늦은 완료·STOP·세대·다리), 큐 넘침, 정지 대기 기한, 다리 손실 → 재캡처 → 관측 실패, 문맥 고정(대기 중 보정이 바뀌면 `correction` 폐기), 취득 자세를 제출 때 계산.
    - 캡처 보호: 열림·닫힘·버림·예외에서 `paused` 짝이 맞음.
    - 기록 조건: STOP 순간·쉬는 세션에서 `record_step` 이 늘지 않음.
    - 초기화: `reset_for_mission` 뒤 트랙·선택·다리·기록·통계가 새것, 미션 디렉터리, 인식·하역 팔레트의 구역이 추정 기준.
    - 예외: `recognition_error:` 는 미션 실패, 물리 한 번만 진행.
    - baseline: `recognition_mission` 은 미션 키, `recognition` 은 세션.
    - 재계산: 적용된 프레임만 그 순서로.
    - 리플레이 게이트 29 건.

## 4. 검증

- ws1 pytest 전체, 리플레이 게이트 29 건.
- Isaac live(ws1, 단일 팔레트 장면 line_1 v1, 학습 제안·SLAM): 콘솔 탐침 cycle P-1:D-1 → P-1:D-0 → P-1:D-1. 같은 세션 또는 별도 세션에서:
  - 캡처 중 STOP(`--live-test-stop-in-capture`) → 재개 → 미션 계속.
  - 관측 중 취소 → 새 제출 성공.
  - 인식 거절(구역 밖 팔레트 — 제출 뒤 원래 구역에서 팔레트를 찾지 못하게 하는 시험 장면) → 미션 실패, 세션 생존, 새 제출 성공.
  - 세 미션 모두 원거리 인식이 다리마다 새로 시작한다(기록이 미션별 디렉터리).
  - 미션 사이 `_chosen` 이 남지 않는다.
  - 원거리 호출 틱 정지 시간 분포를 기록한다.
  - 미션 결과는 기록만 한다.
- 장면 레이아웃용 live sbatch 는 이 PR 에서 만든다(PR ④ 가 이어 쓴다).

## 5. 위험·범위 밖

- 작업 스레드는 학습 RPC 10 s 동안 막힐 수 있다. 그 사이 주행 프레임은 큐 상한으로 버려지고, 정지 판단은 30 s 기한 안에서 기다린다. 처리율이 계속 모자라면 주행 프레임이 줄어 판단 품질이 떨어진다 — 버린 수를 미션마다 기록한다.
- 정지 뒤 관측이 버려지거나 다리가 바뀌어 다시 캡처하면, 그 다리는 주행 프레임 없이 판단한다(정지 한 번만으로 `valid/fused` 가 될 수 있다).
- 밑판 검사의 live 지원은 범위 밖이다.

## 6. 자체 검토 — 직접 재계산

- 소스에서 직접 확인했다.
  - live 거부: `run_transport.py` 1100–1101행.
  - `far_due` 의 조건(5774–5777행): 미션 상태를 보지 않는다.
  - `FarRuntime.due`(204–205행): `paused` 를 본다.
  - `set_zone(centre_xy, radius_m)`(220행): 초기값 인자가 없다.
  - `next_leg`(329행).
  - `failures.py`: `observe:`·`not_pickable:`·`perception_capture_failed`·`near_capture_` 가 미션 실패(44·46·54행)이고, 모르는 접두는 세션 실패.
- 시간 수치(GPU p50 42–44 ms, CPU p50 115–129 ms)는 `docs/validation/2026-10-19-recognition-stage3-learned-proposals.md` 의 기록값을 옮겼다. 다시 재지 않았다 — Isaac live 에서 틱 정지 시간을 잰다.

## 7. 검토 기록

### 2판 → 3판

- **② Codex 재검토(4차): 새 P1 없음, P2 5·P3 2 — 모두 받아들였다.**
  - 관측 문맥을 제출 전에 고정, 꼬리표는 `mission_gen`.
  - 소유 경계를 `recognition_bridge.py` 130행에서 나누고, 학습 통계·`last` 를 반환값으로.
  - 취득 자세를 제출 때 계산.
  - 다리를 잃은 대기 중 정지는 재캡처 1 회 뒤 관측 실패.
  - 큐 상한 4·정지 대기 30 s·`begin_wait/end_wait`.
  - 프레임별 적용·폐기 기록과 재계산 계약.
  - 남은 문구 정리(미션 키 이름, 위험 절의 동기 서술 삭제).
  - 원거리 배치의 고정 입력 비교 시험.

### 1판 → 2판

- **② Codex 적대적 검토(2차): P1 1·P2 6·P3 3 — 모두 받아들였다.**
  - P1 동기 추론 → 결정 T1 (A) 별도 스레드.
  - P2 `record_step` 기록 조건·STOP 순간.
  - P2 의미 분할 예외 경계.
  - P2 예외 뒤 물리 두 번 진행.
  - P2 기록 쓰기 계약(`io_writer`).
  - P2 미션 요약·학습 통계 소유.
  - P2 픽셀 지연 재측정을 PR ① 로.
  - P3 "반드시 single" 철회, STOP·취소 폐기 경로 포함, 재계산 도구(`tools/recompute_far_recognition.py`)가 미션별 디렉터리를 읽게.
- **③ 독립 검토(agy, Gemini 3.8 Flash Medium).** ② 결론을 보이지 않고 1판만 주었다. 8 개 중:
  - 받아들임: 첫 호출 120 s 기한이 초기화에서 소비됨, `state["recognition"]` 의 세션 라벨 → 별도 미션 키, `paused` 고착, 픽셀 지연 11 스텝의 출처(콘솔 영상 카메라), 동기 후보 재계획 → 작업자.
  - 기각: 단일 팔레트 truth(이 PR 은 같은 팔레트를 옮기는 미션 열 — 손잡이가 그 팔레트를 그대로 가리킨다), VRAM 경합(기록만 — Isaac 에서 `nvidia-smi` 를 미션마다 남긴다).

