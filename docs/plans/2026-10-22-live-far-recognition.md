# live 실행기의 원거리 인식 — 연속 이송 CT-1 PR ① (1판)

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
- 시간:
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
- 주행 프레임 기록 조건에 "활성 미션이 observe 단계"(`commands.status` 활성)를 더한다. 쉬는·멈춘·취소 중인 세션은 기록하지 않는다(live 에서만 — 배치는 그대로).
- 미션마다 초기화(live 미션 시작 `live_mission_start`):
  - 새 다리를 연다(트랙·선택 비움).
  - 구역을 그 미션의 구역으로 둔다(`reset_for_mission` 이 구역 초기값과 변경 기록을 새로 정한다 — 지금 `set_zone(centre_xy, radius_m)` 은 초기값 인자가 없다). 사전 팔레트는 장면의 원래 구역, 인식·하역된 팔레트는 현재 추정 위치를 중심으로 같은 반경(상위 계획 Codex 2차 — P-1:D-1 뒤 P-1:D-0 에서 원래 자리를 찾지 않게).
  - 기록 디렉터리를 `recognition/mission_<id>/` 로 바꾼다. 다리·시도 목록은 미션 디렉터리마다 새로 시작한다.
  - 미션의 원거리 요약(`state["recognition"]`)을 미션 키로 옮긴다(`baseline.MISSION_KEYS` 에 `recognition`).
- 관측 버림 뒤 재캡처: `observation_take` 가 관측을 버리면 그 원거리 정지를 되돌리지 않는다. 대신 그 사실(`recognition_discarded`)을 기록한다 — 다음 캡처가 "single" 이 되는 것은 받아들인다(새 주행 프레임을 모으려면 다시 움직여야 한다).
- 시간: 이번에는 동기 호출을 유지하고, 원거리 호출이 막은 벽시계(틱 정지 시간)를 미션별로 기록한다. 실차에서는 인식이 별도 프로세스(Jetson)라 이 지연이 제어 루프를 막지 않는다. 비동기화는 범위 밖(5절).
- 픽셀 지연: live 는 `--recognition-pixel-lag-steps` 를 live 실측(11)으로 쓰도록 실행 스크립트에 둔다(러너 기본값은 그대로). 다시 재는 것은 PR ④ 의 Isaac 에서.
- 예외: 원거리 코드에서 나온 RuntimeError 아닌 예외를 미션 실패 `recognition_error:<형식>` 로 바꿔 세션을 살린다. 새 접두 `recognition_error:` 를 `failures.py`(미션)·`results.js`(문구)에 등록한다.

## 3. 작업 (시험 먼저)

1. 러너 parser: live 의 `--recognition far` 거부를 푼다(밑판·근접 추적·포켓 검사 조합 거부는 유지). 새 인자 없음 — 기존 인자 조합만 바뀐다.
2. 캡처 보호: core 의 `capture_begin`/`capture_stepped`/`capture_abandon` 에 훅 `capture_session_open(bool)` → 러너가 `far_runtime.paused` 를 둔다. 배치 `capture()` 경로는 그대로.
3. 기록 조건: 러너 `far_due` 에 live 면 `core.commands` 활성 조건.
4. `FarRuntime.reset_for_mission(zone_centre_xy, radius, directory)` — 새 다리, 구역 초기값, 기록 디렉터리·목록 초기화. `live_mission_start` 에서 부른다.
5. 예외 감싸기: `far_hook`·`drive_frame` 호출을 감싸 RuntimeError 아닌 예외를 `RuntimeError("recognition_error:<type>")` 로.
6. 시험:
   - parser: live + far 허용, 밑판·근접 추적·포켓 검사 조합은 그대로 거부.
   - core: 캡처 세션 동안 훅이 열림·닫힘·버림에서 정확히 짝을 맞춤.
   - FarRuntime: `reset_for_mission` 뒤 `_chosen`·트랙·다리·기록이 비고 구역이 바뀜, 디렉터리가 미션별, 쉬는 동안 `due()` 거짓.
   - 실패 분류: `recognition_error:` 는 미션 실패.
   - baseline: `recognition` 이 미션 키.
   - 배치 경로 불변(리플레이 게이트 29 건).

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

- 동기 인식이 틱을 막는다(GPU p50 약 43 ms ≈ 배속 0.4 에서 2 틱). 시뮬에서는 물리도 같이 멈춰 물리 결과는 같다. 다만 콘솔 박자·링크 지연 기록이 늘어난다. 실차 인식은 Jetson 의 별도 프로세스다. 비동기화(제안 + 장면 맞춤을 작업자에서, 트랙 갱신·결정을 틱에서)는 필요해지면 따로 한다.
- 원거리 정지 뒤 관측이 버려지면 다시 캡처한 다리는 "single" 판단이 된다(주행 프레임 없음) — 이번에 고치지 않는다.
- `observe_failed` 의 후보 재계획이 틱 안에서 동기로 도는 것은 지금 live 와 같다.
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

(②③)
