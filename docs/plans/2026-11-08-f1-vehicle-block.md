# 차량 측 F1 차단 (2026-11-08, 2판)

확인 전 위치(③)가 개발 보류되면서([개정 계획 9.23](2026-11-07-unconfirmed-monitor-revision.md) 4), F1 운전자 놓기로 정한 출발 위치(`StartRelocalizer` 의 출처 `operator_placed`)는 독립 확인 없이 AUTO·미션에 쓰인다. F1 검증(#249, [검증 문서](../validation/2026-10-31-f1-operator-placed-start.md) 1.3)에서 이웃 칸 오확정 넷이 모두 브리지의 출발 확인을 통과했다. 9.23 은 실차 자율 운용 전에 'F1 끔, 그리고 차량 측 F1 차단 또는 수동 전용 제한의 구현·검증'을 필수 미완료로 등록했다. 이 계획은 그 차량 측 차단이다.

**지킬 것.**

- 사람 확인 뒤 예외는 만들지 않는다(9.23 4). 화면 재확인으로 풀리는 경로가 없어야 한다.
- 텔레옵·수동 운전은 그대로 둔다. 차단은 '지도를 믿고 차가 스스로 움직이는 것'만 막는다.
- 기본값 판단(특히 Isaac)은 권고로 올리고 총괄이 정한다.

## 1. 지금 경로 (코드 확인)

- 출처는 `StartRelocalizer._choose` 에서 정해진다(`start_reloc.py:252`). 매처 확정은 `matcher`, 후보 고르기는 `operator`, 놓기 확정은 `operator_placed` 다(`confirm_place`, `:221`). 한 번 `chosen` 이 되면 그 세대의 재시도·놓기는 거부된다(`retry`·`place` 의 상태 검사).
- 재위치 중에는 아비터가 `reloc` 을 들고 있어, 차를 움직이는 요청을 `relocalizing` 으로 거부한다(`arbiter.py:352`, `RELOC_REFUSED` + AUTO·TELEOP 모드 요청).
- 러너가 미션 코어를 열면 `LiveRuntime.start()` 가 `arbiter.reloc = None` 으로 넘긴다(`live_runtime.py:219`). **이때 출처가 아비터에서 사라진다.** 출처는 러너 기록 `state["start_reloc"]["source"]` 와 에폭 폴더의 `reloc_pose.json` 에만 남는다(`reloc_phase.py:336`, `start_reloc.py:273`).
- 같은 모양의 차단이 이미 둘 있다.
  - ③: 아비터 `_run` 이 `POSE_REFUSED`(mission_submit·mission_resume·pick_request·recovery·sim_reset)와 AUTO 모드 요청을 `pose_unconfirmed` 로 거부한다(`arbiter.py:359`). 미션 명령도 `Context.pose_ok` 로 submit·resume 을 거부한다(`commands.py`).
  - D6 장면 손상: 래치가 서면 `Context.scene_damaged` 로 submit 을 거부하고 상태 메시지에 `scene` 을 싣는다(`live_runtime.py:387`, `:553`).
- 재위치는 `--start-relocalize` 의 라이브 세션에만 있다(`run_transport.py:1633`). 배치 실행·재생 게이트·묶음(`BUNDLE_TOKEN` 은 알려진 출발, `isaac_live.sbatch:117`)에는 없다.

## 2. 결정할 것과 권고

### 2.1 차단 지점: 아비터와 미션 명령 두 곳

③ 과 같은 두 겹으로 한다.

1. **아비터 `_run`.** 래치가 서 있으면 `mission_submit`, `mission_resume`, `pick_request`, `recovery`, `mode_request(to=AUTO)` 를 사유 `operator_placed_start` 로 거부한다. 확인 순서는 `relocalizing` 다음, `pose_*` 앞이다.
2. **미션 명령 `Context`.** 새 불 필드 `start_blocked` 가 참이면 submit·resume 을 고정 코드 `"operator_placed_start"` 로 거부한다. 적재 제출 예외(6c)와 상태 변경보다 먼저 본다. 아비터를 거치지 않는 제출이 생겨도 코어가 막는다. 고정 문자열로 돌려주는 것은 콘솔 거부 사유 시험(`tests/unit/console/test_reject_reasons.py`)이 반환 코드를 추적하기 때문이다.

AUTO 는 미션 제출·재개가 수락될 때만 들어간다(`resolve`, `arbiter.py:740–744`). `mode_request(to=AUTO)` 는 지금도 `invalid` 로 거부되고(`_on_mode_request`, `:897`), 래치가 있으면 그보다 먼저 차단 사유로 답한다(③ 과 같은 방식). 두 겹이면 AUTO 진입 경로가 모두 닫힌다. `resolve` 의 AUTO 진입에 래치 단언을 하나 더 두어, 다른 경로가 생기면 시험이 깨지게 한다.

### 2.2 래치가 서는 곳과 풀리는 조건

- **서는 곳.** 러너는 재위치 기록 `state["start_reloc"]` 를 런타임에 넘긴다. `LiveRuntime.start()` 는 `arbiter.reloc = None` 으로 넘기기 직전, 같은 함수 안에서 그 기록으로 래치를 정한다. 그래서 재위치 중의 `relocalizing` 거부와 래치 사이에 빈 틱이 없다.
  - 근거는 재위치기 객체가 아니라 러너 기록이다. 초기 콘솔이 없는 경로(`parts is None`, 무인 재위치)에서도 같은 값을 본다.
  - **출처를 모르면 막는다(fail-closed).** 재위치를 했는데(`--start-relocalize`) 기록의 `source` 가 없거나 `matcher`·`operator`·`operator_placed` 밖의 값이면 래치를 세운다. 재위치기에 추정이 남아 있으면 기록의 출처와 대조하고, 다르면 역시 세운다.
  - 러너는 정한 값을 `state["start_reloc"]["autonomy"]`(`blocked`·`allowed`·`not_applicable`)에 적는다.
- **출처가 바뀌는 경로 전부** (코드 기준):
  - 같은 에폭 안: **없다.** `chosen` 뒤에는 재시도·놓기·고르기가 모두 거부되고, 런타임이 넘긴 뒤에는 재위치기가 없다. 따라서 래치는 그 러너가 끝날 때까지 간다. STOP·재무장·리스 교체·재접속·텔레옵 주행으로 풀리지 않는다.
  - 새 에폭: 감독자가 다음 에폭을 시작하면(실패 뒤 재시작, 운전자의 새 장면 요청) 새 러너가 새 재위치를 하고 그 출처로 다시 정한다. `clear_epoch` 이 이전 파일을 먼저 지운다(`live_supervisor.py:261`).
  - 세션 끝 → 새 세션: 위와 같다.
  - ③ 의 `confirmed`: 보류 중이고 V 를 하지 않았으므로 **래치를 풀지 않는다.** ③ 이 재개되어 검증되면 그때 해제 경로로 따로 정한다.
  - `sim_reset`: 같은 에폭에서는 래치를 지우지 않는다. 요청이 수락되면 제어를 닫고(`arbiter.py:411`) 러너가 끝나며, 다음 에폭의 재위치가 새 출처로 다시 정한다. 수락 직후 같은 에폭에서 다시 보낸 요청도 계속 막힌다.
- **그래서 풀리는 길은 '새 재위치에서 `operator_placed` 가 아닌 출처로 정해지는 것' 하나다.** 매처 재확정은 지금 같은 에폭 안에서는 할 수 없다. 세션 안 재위치 명령을 새로 만드는 것은 이 계획의 범위 밖이다(필요하면 따로 계획).

### 2.3 허용으로 남기는 것

`mode_request(to=TELEOP)`, `arm`, `rearm`, `disarm`, 텔레옵 주행·승강, `stop`, `session_end`, `mission_cancel`·`mission_pause`(미션이 없어 효과 없음), 리스·대상 요청, `sim_reset` 이다.

- `sim_reset` 은 Isaac 에만 있다(`sim_reset_allowed`). 새 에폭과 새 재위치로 이어지므로 열어 둔다. ③ 이 이것을 막는 이유(위치를 모르는 채 장면 교체)는 F1 차단에는 해당하지 않는다.
- `recovery`(포켓에서 스스로 빠져나오기)는 지도에 기대는 자동 동작이라 목록에 넣는다. 지금은 구현되지 않아 늘 `unsupported` 지만(`arbiter.py:422`), 구현되면 바로 막히게 하려는 것이다. 텔레옵으로 넣은 포크는 텔레옵으로 뺀다.
- `pick_request` 는 추천만 돌려주고 차를 움직이지 않지만 막는다. 추천 자체가 확인되지 않은 지도 위치로 계산되어 수동 작업에도 틀린 안내가 될 수 있고, 대기열의 자동 항목이 이어서 미션을 보내므로 같은 사유로 일찍 멈추게 한다(③ 과 같음).

### 2.4 콘솔 표시와 사유 코드

- **사유 코드:** `operator_placed_start`. 아비터 거부와 미션 명령 거부에 같은 코드를 쓴다.
- **상태 메시지:** `start_block: {"reason": "operator_placed_start", "since_mono": …}`. 래치가 없으면 필드를 싣지 않는다(③ 의 `pose_check` 와 같은 방식).
- **콘솔 문구**(profile.js 의 거부 사유 표 두 곳): "운전자가 놓은 출발 위치입니다 · 자동 운행과 미션은 받지 않습니다(수동 운전만)".
- **페이지:** 래치가 있으면 AUTO·미션 버튼을 비활성으로 두고 위 문구를 띄운다. 지도에도 "운전자가 놓은 위치 · 확인되지 않음 · 지도 표시를 믿고 운전하지 마세요" 띠를 띄운다. 확정한 순간 지도와 센서 겹침이 잘 맞는 것처럼 보여도 실제 위치는 확인되지 않았기 때문이다.
- **모의 차량**(`console/mock_vehicle/core.py`): 놓기 확정 뒤 같은 래치와 `start_block` 을 내도록 맞춘다. 지금은 확정 뒤 출처를 버리고 `reloc=None` 이 되어 미션이 열린다(`core.py:1049`, `:1091`).
- **대기열:** `mission_queue.py`/`queue.js` 는 이 거부를 정해진 거부로 보고 멈춘다. `UNSURE_REJECTS` 에 넣지 않는다.
- **계약 문서:** 거부 사유 코드 목록과 상태 메시지 절에 더한다.

### 2.5 기본값 (권고, 총괄 결정)

- **코어:** 아비터 설정의 기본은 차단 켬이다. 차단을 끄는 것은 명시한 설정으로만 한다.
- **러너:** 새 인자 `--operator-placed-auto {block,allow}`, 기본 `block`. 실차 런처는 아직 없고, 실차 경로는 이 기본을 그대로 쓴다. 덮어쓰기는 실차 런처에서 허용하지 않는다고 계약에 적는다.
- **Isaac `isaac_live.sbatch`: 권고는 `block`**, 환경 `F1_AUTONOMY=allow` 로만 끈다.
  - 근거 1: 기준선과 겹치지 않는다. 재생 게이트·배치·묶음 세션에는 재위치가 없다(1 절). F1 검증 `place` 시나리오는 위치를 정한 뒤 서 있다가 끝나므로(`live_console_probe.py` place, hold) 차단과 무관하다.
  - 근거 2: 실차와 같은 스택이라는 원칙(운용 콘솔 결정 2026-10-13)에 맞고, 운전자가 Isaac 에서 실차와 같은 제약을 본다.
  - 영향받는 것: ③ 개발의 `reverse_place` + `MISSION_AFTER_CONFIRM=1`(놓기 뒤 미션)이다. 보류 중에도 기존 기록 재판정과 총괄이 명시적으로 승인한 개발 실행은 허용된다(9.23 1). 그런 실행과 ③ 재개 뒤의 실행은 `F1_AUTONOMY=allow` 를 명시해야 한다. 이 사실을 `reloc_session.sh` 주석과 개정 계획 9.23 에 적는다.
  - **규칙:** 놓기 뒤 자동 운행을 보는 시험·스모크·배치는 `F1_AUTONOMY=allow`(러너 `--operator-placed-auto allow`)를 명시한다. 명시가 없으면 차단이 맞는 결과다.
  - 대안 `allow`(Isaac 기본 끔): Isaac 콘솔 운용에서 놓기 뒤에도 미션을 돌릴 수 있지만, 실차와 다른 동작을 기본으로 두게 된다.
- 러너 인자는 기록 인자(`arguments`)에 남는다. 근접 계약(`near_tracking.py` RUN_SPECIFIC)에 등록한다.

## 3. 구현 단위

1. `src/forklift_core/safety/arbiter.py`: 래치 필드, `START_REFUSED` 목록, `_run` 확인, `_set_mode("AUTO")` 단언, 이벤트(`start_block`, 거부마다 기존 응답 기록).
2. `src/forklift_core/mission/commands.py`: `Context.start_blocked`, `_submit`·`_resume` 의 거부(고정 코드).
3. `sim/isaac/live_runtime.py`: `start()` 의 래치 설정(설정이 `block` 일 때), 상태 메시지 `start_block`, `Context` 전달.
4. `sim/isaac/run_transport.py`: 인자, `start_relocalize` 와의 조건, 기록, 런타임에 재위치 기록 전달. `sim/isaac/near_tracking.py`: 계약 등록.
5. `deploy/console/isaac_live.sbatch`: `F1_AUTONOMY`(기본은 총괄 결정), DRY_RUN 에 표시.
6. `deploy/console/reloc_session.sh` 와 `tools/live_console_probe.py`: 새 `RELOC_PROBE=place_block` 이 `--scenario place --after-place block_check` 를 넘긴다. 프로브에 `block_check` 분기를 구현한다(5 절의 판정).
7. 콘솔: `console/web/js/profile.js` 문구, 페이지 버튼 상태와 지도 띠, 모의 차량, `queue.js`·`tools/mission_queue.py` 의 거부 처리 확인.
8. 문서: 콘솔 계약(사유 코드, `start_block`, 실차 런처의 덮어쓰기 금지, 6 절의 실차 요구), F1 검증 문서·개정 계획 9.23 의 상태(구현 뒤 '구현됨, 실차 검증 전'으로).

## 4. 시험 (먼저 쓴다)

- **아비터** (`tests/unit/safety/test_arbiter_reloc.py` 에 더함)
  - 래치가 있으면 다섯 요청이 `operator_placed_start` 로 거부되고 상태가 바뀌지 않는다.
  - 2.3 의 허용 목록이 모두 통과한다(TELEOP 진입과 텔레옵 주행 포함).
  - 래치가 STOP·재무장·리스 교체·재접속 뒤에도 남는다.
  - `relocalizing` 이 래치보다 먼저 답한다. 둘이 겹쳐도 사유가 하나로 정해진다.
  - ③ 과 겹칠 때의 우선순위: 래치가 `pose_unconfirmed` 보다 먼저.
  - `_set_mode("AUTO")` 단언.
- **런타임**
  - 출처 `operator_placed` 면 `start()` 직후 래치, `matcher`·`operator` 면 없음, 알려진 출발이면 없음, 설정 `allow` 면 없음(기록에는 남음).
  - 출처가 없거나 모르는 값, 재위치기의 추정과 기록이 다르면 래치(fail-closed). `parts is None` 경로도 같다.
  - 래치와 `reloc=None` 사이에 거부되지 않는 틱이 없다.
  - 상태 메시지의 `start_block`.
- **미션 명령:** `start_blocked` 가 submit·resume 을 거부하고, 적재 제출(6c)도 막는다. 콘솔 거부 사유 시험이 새 코드의 문구를 찾는다.
- **연결:** `sim_reset` 수락 직후 같은 에폭의 재요청이 막히고, 다음 에폭(새 런타임·새 기록)에서 출처에 따라 다시 정해진다.
- **모의 차량:** 놓기 확정 뒤 미션 거부와 `start_block`, 페이지 버튼·띠.
- **러너 인자:** 기본 `block`, `--start-relocalize` 없이 `allow` 는 무의미하므로 그대로 받되 기록, 계약 등록 누락 시험. 새 인자가 없는 옛 기록(재생 Namespace)은 기본값으로 읽힌다.
- **콘솔:** `mission_queue.py` 의 정해진 거부 처리, profile 문구 표에 코드가 있는지(기존 문구 시험 방식).
- **런처:** DRY_RUN 출력에 인자가 나오고, 잘못된 값은 거부.

## 5. 검증

- **재생 게이트 29/29**(러너·미션 코어를 건드림). 배치 실행에는 재위치가 없으므로 판정 불변이 기대값이다.
- **ws1 전체 시험** 0 failed.
- **재생 게이트 29/29 는 F1 차단의 증거가 아니다.** 배치 판정이 바뀌지 않았다는 확인일 뿐이다.
- **Isaac 스모크 2 세션**(`RELOC_PROBE=place_block`, `block` 한 번, `F1_AUTONOMY=allow` 한 번, 같은 출발). 단독 live 세션이라 `isaac_bundle.py sizes live --slots 1` 이 찍는 shard 2 로 예약한다. 세션당 약 15 분이고, ws1 시간은 시뮬-물성·콘솔 세션과 조율한다.
  - **선행 조건(하나라도 없으면 그 세션은 실패):** 놓기 확정 `ack`, 러너 기록의 `source == "operator_placed"`, 같은 에폭으로의 넘김(에폭이 바뀌면 실패), 아래 검사를 모두 실행.
  - **`block` 판정:** `mission_submit`·`mode_request(AUTO)`·`pick_request` 가 정확히 `operator_placed_start` 로 거부, 미션이 만들어지지 않음, AUTO 진입 0, 상태 메시지에 `start_block`. `mode_request(TELEOP)` 수락과 짧은 텔레옵 주행(측정 이동 > 0).
  - **`allow` 판정:** 같은 순서에서 `mission_submit` 이 수락되고 `start_block` 이 없음. 미션은 수락 확인 뒤 취소해도 된다.
  - 매처가 먼저 확정해 놓기를 못 하면 '미도달'로 실패 처리하고, 다른 출발로 다시 돈다.
- 실차 검증은 실차가 준비된 뒤다. 그 전까지 9.23 의 항목은 '구현·시뮬 확인, 실차 검증 전'으로 둔다.

## 6. 범위 밖과 남는 위험

- 매처 무인 출발의 남은 모호성(검증 문서 그대로)과, 운전자가 매처 후보를 잘못 고르는 경우(`operator`)는 막지 않는다. 9.23 이 `operator_placed` 만 지정했다.
- 알려진 출발(`START_POSE` 없이 장면 시작, `START_RELOC=off`)은 재위치가 없어 이 차단과 무관하다.
- 세션 안에서 다시 재위치하는 명령은 없다. 놓기로 시작한 세션은 끝날 때까지 수동 운전만 된다.
- 이 차단은 소프트웨어 래치다. 실차의 하드웨어 비상정지·MCU 쪽 동작과는 별개다.
- **텔레옵 허가는 놓은 좌표계에서 계속 돈다.** 텔레옵 주행은 장애물층 허가를 거치는데, 그 층의 지도 기반 부분(알려진 장애물, 정적 지도층)은 확인되지 않은 위치에 놓인다. 한 칸 밀렸다면 빈 통로에서 허가가 막히거나, 실제 장애물을 놓칠 수 있다. 텔레옵은 이 계획 전에도 열려 있었고 운전자는 눈으로 운전한다는 전제다. 차단 상태에서 허가를 센서 근거만으로 바꾸는 것은 이 계획 밖이며 총괄 판단 항목으로 올린다.

### 실차 전 요구 (구현은 실차 스택이 생길 때, 계약에 등록)

- **출처 보존.** 실차에서 위치를 정하는 모든 입력은 출처를 가진다. 출처를 모르는 초기화(예: ROS 표준 초기 자세 토픽처럼 출처 필드가 없는 경로)는 차단된 상태로 시작한다.
- **재시작.** Isaac 은 러너가 다시 뜨면 장면도 다시 만든다. 실차는 프로세스가 죽어도 차가 그 자리에 있다. 실차에서 재위치 없이 러너를 다시 띄우는 경로(알려진 출발, 저장된 자세 재사용)는 두지 않는다. 생기면 이전 출처를 이어받거나 차단으로 시작한다.
- **덮어쓰기 금지.** 실차 런처는 `--operator-placed-auto allow` 를 받지 않는다.

## 7. 자체 검토

### ① 원자료 재확인

1판의 서술을 코드에서 다시 확인했다. 고친 것은 둘이다.

- `mode_request(to=AUTO)` 는 지금도 `invalid` 로 거부된다(`_on_mode_request`). AUTO 는 미션 제출·재개 수락으로만 들어간다(`resolve`). 서술을 그에 맞췄다.
- `recovery` 는 구현되지 않아 늘 `unsupported` 다. 목록에 두는 이유를 적었다.

그 밖에 확인한 것: 출처가 정해지는 곳과 `chosen` 뒤 거부(`start_reloc.py`), 런타임의 `reloc=None`(`live_runtime.py:219`), ③·D6 의 두 겹 거부, 재위치가 라이브에만 있다는 것, 놓기 뒤 미션을 보내는 실행이 `reverse_place` + `MISSION_AFTER_CONFIRM=1` 뿐이라는 것(`live_console_probe.py`, `reloc_session.sh`). 무인 재위치에는 `arbiter.reloc` 이 붙지 않으므로, 래치 근거를 러너 기록으로 바꿨다.

### ② Codex (새 스레드)

| 회차 | 지적 | 처리 |
|---|---|---|
| 1 | P1 우회 경로 없음 | — |
| | P2 스모크의 미실행·실패 판정 없음, 래퍼 전달 경로 없음 | 선행 조건·판정·미도달 실패, `RELOC_PROBE=place_block` 과 프로브 분기 |
| | P2 shard 1 부족 | `sizes live --slots 1` 의 shard 2 |
| | P3 동적 사유 반환이 거부 사유 시험을 깸 | 불 필드 `start_blocked` + 고정 코드 |
| | P3 모의 차량 불일치 | 범위에 넣음 |
| | P3 9.23 인용이 좁음 | 승인된 개발 실행 포함 |
| | 권고: sim_reset 재요청·다음 에폭 시험, 옛 Namespace, 29/29 는 증거 아님 | 시험·검증 절에 넣음 |
| 2 | **새 P1/P2 없음** | 수렴 |

### ③ 독립 검토 (agy, Gemini 3.8 Flash, 자체 검토 절 없이 전달)

| 지적 | 처리 |
|---|---|
| P1 실차 ROS 표준 초기 자세에는 출처가 없음 | 실차 전 요구(출처 없는 초기화는 차단으로 시작) |
| P1 실차 재시작 때 차는 그대로인데 래치가 사라짐 | 실차 전 요구(재위치 없는 재시작 금지, 생기면 출처 승계나 차단) |
| P1 텔레옵 허가가 놓은 좌표계에서 돎 | 범위 밖 위험으로 적고 총괄 판단 항목으로 올림 |
| P2 지도 표시가 운전자를 오도 | 지도 '위치 미확인' 띠 |
| P2 `pick_request` 는 읽기라 허용해야 | **불채택.** 추천이 확인되지 않은 위치로 계산되어 수동 작업에도 틀린 안내가 된다 |
| P2 `parts is None`·추정 없음 경로 | 근거를 러너 기록으로, 출처를 모르면 차단 |
| P3 놓기를 쓰는 시험의 기본값 | `allow` 명시 규칙 |
