# 실차 안전 배선 — PR 5 (설계)

작성 2026-11-19 · 1판(검토 전) · 상위 [실차 자율 계획](../plans/2026-11-18-real-autonomy-entry.md) PR 5 행 · 앞 단계 [4b 조립](2026-11-19-real-assembly.md)(#599, 14 절의 PR 5 요구), [4c 루프](2026-11-19-real-loop.md)(#602)

## 범위

계획 PR 5 행(4e-2b 안전 배선)과 4b 설계 14 절이 넘긴 '등록 팔레트 반사의 팽창 전 분리'를 한다. 4c 가 넘긴 `--run` 벽시계 배선과 실 `TtyLink` 의 gate 재통과는 6a 그대로다(총괄 2026-11-19).

- **Isaac 기본 동작은 바꾸지 않는다.** 새 코드는 실차 모듈(`src/forklift_core/live/`)과 기본 꺼진 옵션뿐이다. 공유 코드(`ObstacleGrid`, 중재기)를 건드리면 기본값이 지금과 같음을 시험으로 고정하고, 재생 게이트 29/29 와 ws1 전체 pytest 를 거친다.
- 실 MCU 로 AUTO 를 내는 일은 여전히 막혀 있다(4c 장치 차단). PR 5 는 층·감시·복구의 **배선과 판정**을 가짜 원천으로 시험한다. 실센서 입력은 PR 7 이다.

| 항목 | 내용 | 절 |
|---|---|---|
| 실차 층 | `RealLayer`: 실스캔 입력, 비동기 갱신, 설치된 스냅숏의 세대가 맞을 때만 허가 | 1 |
| 팔레트 반사 분리 | 등록 팔레트에서 온 반사는 지우기만 하고 표시하지 않는다(`ObstacleScan.clear_only`, 기본 꺼짐) | 2 |
| 무장 전 층 강제 | 층이 살아 있고 `obstacle_act=True` 일 때만 채택·무장, 실행 중 해제는 래치 | 3 |
| 감시 | `safety_watch.yaml` 로 감시를 조립, D435i IMU 입력과 끊김 | 4 |
| 콘솔·lease | 4b·4c 의 정책을 실제 콘솔 요청으로 끝까지 시험 | 5 |
| 재위치 배선 | 현장 복구 완료 → 옛 증거 폐기 → 새 세대의 재위치 확인 | 6 |
| 포켓 구간 재개 | S10-A 에는 재개가 없어 해당 없음(이유 기록) | 7 |
| 단일 실행기 | 장치마다 한 실행기만 링크를 연다(잠금 파일) | 8 |

## 1. 실차 층 `RealLayer` (`src/forklift_core/live/real_layer.py`)

Isaac 층(`sim/isaac/obstacle_layer.py`)은 레이 캐스팅·합성 잡음·시뮬 시계를 품고 있어 실차가 가져오지 않는다(4c 원칙). 실차 층은 공유 `ObstacleGrid`·`DrivePermission` 위에 S10-A 가 쓰는 부분만 얹는다.

**코어가 읽는 것**(4c 가짜 트럭으로 S10-A 전 구간을 돌려 기록): `refresh`, `limit`, `watchdog`, `permission`(`path_check`·`last_estop`·`config.stopping`), `known_enabled`, 그리고 코어가 쓰는 `exempt`·`depth_support`. 표에 없는 속성을 읽으면 `AttributeError` 로 ①이 되게 둔다(조용한 기본값 없음). 시험으로 이 표를 고정한다(4c 훅 표와 같은 방식).

### 1.1 스캔 입력

- `add_scan(scan: RealScan)`: 실스캔 한 장. 필드는 센서 이름, **취득 시각**(호스트 단조 시계, 브리지가 원 ROS stamp 로 옮긴 값 — PR 7), 각도·거리(REP-117), 자기 차체 반사 마스크다.
- 취득 시각의 후륜축 오도메트리 자세는 루프의 자세 이력에서 보간한다. 그 구간이 연속이 아니면(4c 오도메트리 단절) 그 스캔은 버리고 셈한다(`scans_dropped`).
- 장착 자세는 차량 설정(`config/real/vehicle.yaml` 의 라이다 장착값; 없으면 조립 거부)에서 온다.
- PR 5 에서 스캔 원천은 주입된 가짜다. 실제 A2M12 는 PR 7.

### 1.2 비동기 갱신 (계획 PR 5 행의 계약)

코어는 10 곳에서 `layer.refresh(...)` 를 부른다(보정 변경의 `slam_replies`, `permission_watch` 등). 실차 층의 `refresh` 는 **기다리지 않는다**:

1. `refresh(now_s, correction, version, *, current_pose, path_ahead, loaded, direction)` 은 요청만 기록한다(가장 새 요청 하나, 세대 = `version`). 반환은 `None` 이고, 코어가 그 반환값을 쓰는 곳이 있으면(4c 기록에서는 없음) 시험으로 막는다.
2. 작업자 스레드가 요청을 꺼내 격자 스냅숏·면제 영역·허가 경로 검사를 계산한다. 작업자는 격자의 사본(요청 시점의 스캔 묶음)만 읽고 루프 상태를 바꾸지 않는다.
3. 결과는 **제어 스레드가 틱 머리에서만 설치**한다(`install_ready(now_ns)`). 설치는 요청의 세대가 지금 적용된 보정의 세대와 같을 때만이다. 옛 세대 결과는 버리고 셈한다(`stale_results`).
4. 설치된 스냅숏은 `(보정, 세대, 담긴 가장 새 스캔의 취득 시각)` 을 갖는다. 4b 의 `SnapshotView` 가 이것을 그대로 읽는다(`installing` 은 설치 안 된 더 새 요청이 있을 때 참).

**허가**(`limit`): 다음 중 하나면 `(0, 사유)`.
- 설치된 스냅숏이 없거나 세대·보정이 적용값과 다름 → `snapshot_installing`(대기).
- 담긴 가장 새 스캔이 `SCAN_MAX_AGE_S` 를 넘음 → `scan_stale`.
- `watchdog` 이 정지 → 그 사유.
- 그 밖에는 `DrivePermission.allowed_speed` 그대로.

**watchdog**: 4c 루프가 매 틱 부른다. 스캔이 들어왔는데 지연 상한 B 안에 설치된 스냅숏에 들어가지 못하면 `layer_overdue`, 그것이 `overdue_end_s` 를 넘으면 `layer_latency_exceeded`(①). 작업자 스레드가 죽었으면 즉시 `layer_worker_dead`(①). B 와 `overdue_end_s` 는 Isaac 층 예산 설정값을 잠정으로 쓴다.

**SLAM 무효 뒤 입력만 복구**: 보정이 바뀌거나 SLAM 이 무효가 되면 설치된 스냅숏은 세대가 어긋나 허가가 닫힌다. 입력이 돌아와 새 세대 결과가 설치돼도 **자동 재개는 없다**: SLAM 무효는 4b 준비 판정에서 래치 사유이므로 허가가 이미 래치돼 있고, 재무장 전에는 출력이 없다.

### 1.3 그 밖의 속성

- `exempt`: D5 포켓 영역 면제. 작업자가 계산 때 그 시점 값의 사본으로 `exempt_region`(src 로 옮긴 같은 함수)을 적용한다. 코어가 값을 바꾸면 다음 요청부터 반영되고, 이미 설치된 스냅숏은 그대로다 — 면제가 **좁아지는** 변경(해제)은 즉시 효력이 있어야 하므로, 해제 때는 설치된 스냅숏을 무효로 하고(`snapshot_installing`) 새 결과를 기다린다.
- `depth_support`: 실차는 그림자 띠 기억을 쓰지 않으므로(엄격 규칙) 저장만 하고 읽지 않는다.
- `known_enabled = False`(4a 가 켜짐을 거부), `memory = None`, 출구 면제 없음.
- `planner_grid(...)`: 재계획용 점유 격자. 설치된 스냅숏의 점유 셀을 조사 지도 격자와 합친다.

## 2. 등록 팔레트 반사 분리 (4b 14 절)

문제: 정확히 등록한 팔레트의 전면 반사도 격자 팽창으로 외곽 밖 셀(전면 2.70 m 앞 2.675 m 셀)에 남아 삽입 계획을 막는다.

- `ObstacleScan` 에 `clear_only: np.ndarray | None = None` 을 더한다. None(기본)이면 지금과 바이트까지 같다(시험으로 고정). 참인 빔은 **표시하지 않고 지우기만** 한다: 빔을 따라 거리 − 센서 오차 상한(`sensor_bound_m`)까지 지우고, 끝점과 그 둘레는 표시하지 않는다.
- 실차 층은 스캔마다 각 빔의 끝점을 보정∘오도메트리로 지도 좌표에 놓고, 현재 미션의 **등록 팔레트**(`basis=operator_input`, 미션이 고정한 자세) 외곽을 센서 오차 상한만큼 키운 사각형 안이면 `clear_only` 로 표시한다.
- 그 밖의 반사는 그대로 표시한다. 다른 물체가 팔레트 외곽 안에 있으면 숨겨진다 — 그 영역은 삽입 직선에서 D5 포켓 검사가 따로 본다(4a). 외곽 밖에 있는 물체는 지금처럼 막는다.
- 등록이 없거나 미션 밖이면 분리하지 않는다.
- 회귀 시험: `ObstacleGrid → plan_transport` 경로. 같은 스캔으로 분리 끔 → 계획 실패(지금 증상 재현), 켬 → 성공. 팔레트 옆 상자는 켬에서도 막는다.

## 3. 무장 전 층 강제

- 채택·무장 전 준비 판정(`Evidence.layer_ok`)은 `RealLayer.ok(now)` 를 읽는다: 작업자 생존, 스캔 원천 연결, 설치된 스냅숏 존재.
- `obstacle_act=True` 는 4b 의 필수 옵션이다. 4c 루프는 매 틱 `core.options.obstacle_act` 와 `core.obstacle["layer"] is real_layer` 를 확인하고, 바뀌었으면 ①이다(실행 중 해제 금지).

## 4. 감시

- 조립이 `config/real/safety_watch.yaml` 에서 `SafetyWatch` 를 만들어 루프에 넘긴다. 파일이 없거나 깨지면 조립 거부.
- **시계 둘**: 4c 는 STATE 마다 MCU 시각으로 감시한다(전류·바퀴). IMU 표본은 호스트 시계다. 한 인스턴스에 두 시계를 섞지 않도록 **두 인스턴스**를 둔다.
  - `watch_state`: 지금처럼 STATE 마다, MCU 시각, 기울기 없음.
  - `watch_imu`: 틱마다 호스트 시각으로 가장 새 가속 표본을 준다(`imu_expected=True`). 표본이 `max_age_s` 보다 오래되면 `watch_imu_stale`, 기울기 초과가 `hold_s` 이어지면 `watch_tilt`. 둘 다 ①.
- IMU 원천(D435i 가속)은 주입한다. IMU 가 없다고 조립에서 밝히면(`imu: none`) `watch_imu` 는 만들지 않고 기록에 남긴다 — 실차 S10-A 에서 IMU 를 필수로 할지는 사용자 결정이 아니라 설정값이므로 잠정 `required` 로 두고 총괄에 보고한다.

## 5. 콘솔·lease

4b(중재기 `auto_lease_loss="stop"`, 재무장 거부 조건)와 4c(콘솔 DISARM, 첫 바이트 lease)에서 정책은 끝났다. PR 5 는 계획이 요구한 통합 시험을 더한다: 실제 콘솔 요청(lease → 제출 → 채택·무장 → AUTO)으로 무장하고, STOP·DISARM·센서 무효(스캔 끊김, IMU 끊김, SLAM 무효) 각각 뒤에 금지 명령(비영 DRIVE·ARM)이 0 건이며, 입력만 돌아와서는 재개되지 않음(재무장 요청 전 DRIVE 0).

## 6. 재위치 배선

지금 중재기는 `recovery` 를 `unsupported` 로 거절한다. 실차 설정에서만 다음을 연다(중재기 설정 `real_recovery=False` 기본, Isaac 그대로):

1. **현장 복구 완료**(`recovery`, 콘솔): 중재기가 lease·세션 검사 뒤 루프 콜백에 넘긴다. 루프는 `core.field_recovery_done(t, id, disarmed=, outstanding=)` 를 부른다.
   - `disarmed` = 허가의 DISARM 이 확인됨(`disarm_unconfirmed` 아님, 신선한 DISARMED). `outstanding` = 포트가 `settled` 가 아님.
   - 거절 사유는 그대로 콘솔 응답의 사유가 된다.
2. 완료되면 코어가 재위치 세대를 연다(`relocalize_required = gen`). 루프는 **옛 증거를 버린다**: 정지 표본·자세 이력·D5 프레임·설치된 층 스냅숏(세대를 올려 무효), SLAM 원천에 `relocalize(gen)` 요청(주입 인터페이스, 실물은 PR 7).
3. **재위치 확인**: 운용자가 차를 바닥 표시 시작 자세에 두고 콘솔에서 `relocalize_confirm{generation}` 을 보낸다. 루프는 다음이 모두 맞을 때만 `core.relocalized(t, gen)` 을 부른다:
   - 같은 세대, 요청이 완료 뒤에 옴;
   - SLAM 원천이 그 세대 요청 **뒤에** 계산한 신선한 자세를 냄;
   - 그 자세가 등록된 시작 자세와 허용오차(위치 0.10 m, 각 5°, 잠정) 안;
   - 정지 표본이 그 뒤에 새로 쌓임(차가 서 있음).
   하나라도 아니면 거절 사유를 답한다. 옛 세대·늦은 답은 아무것도 바꾸지 않는다(코어 규칙 그대로).

## 7. 포켓 구간 재개 조건

계획은 "포켓 구간 재개는 D5 가 각 방향을 답한 뒤, 그 전까지 삽입은 감독·저속·빈 포크" 를 요구한다. S10-A 에는 재개가 없다: 일시정지는 래치·수거이고(4c), 포켓 안에서 끝난 미션은 현장 복구로만 끝난다. 그래서 PR 5 에서 구현할 것이 없다. 재개를 여는 PR 에서 이 조건을 넣는다는 줄을 계획 문서에 남긴다.

## 8. 단일 실행기

- `executor_lock(device_path)`: 장치 경로마다 잠금 파일(`$XDG_RUNTIME_DIR/forklift-mcu-<장치 이름>.lock`)에 `flock(LOCK_EX | LOCK_NB)`. 잡지 못하면 링크를 열기 전에 거부한다. 프로세스가 죽으면 커널이 푼다.
- PR 5 는 함수와 시험, 실차 자율 실행기 경로(`tools/real_autonomy.py` — `--run` 은 아직 거부지만 잠금은 장치를 열기 전 단계에 둔다)에 넣는다. 텔레옵 다리에 같은 잠금을 거는 일은 11-20 보드 확인일 뒤(6a)로 미룬다 — 그날 쓰는 도구의 동작을 바꾸지 않기 위해서다.

## 9. 시험

- 층: 코어 접근 속성 표(S10-A 전 구간), 비동기 계약(새 스캔 미설치 → 대기, 옛 세대 결과 늦게 도착 → 버림, 작업자 죽음 → ①, 면제 해제 → 즉시 대기, SLAM 무효 뒤 입력만 복구 → 재개 없음), 단절 구간 스캔 버림.
- 분리: `ObstacleGrid` 기본값 바이트 동일, `clear_only` 빔의 지우기·표시 없음, `ObstacleGrid → plan_transport` 회귀.
- 강제: 층 없음·`obstacle_act` 꺼짐에서 채택 거부, 실행 중 해제 → ①.
- 감시: 설정에서 조립, IMU 끊김·기울기 → ①, 두 시계 분리.
- 통합(4c 가짜 트럭 확장): 실제 콘솔 요청으로 무장, STOP·DISARM·센서 무효 뒤 금지 명령 0, 입력 복구만으로 재개 없음.
- 재위치: 완료 거절 사유(무장 미확인, 명령 남음, 세대 다름), 완료 → 증거 폐기 → 확인 조건 각각, 늦은 답 무시, 확인 뒤 제출 받음.
- 잠금: 두 번째 잠금 거부, 프로세스 종료 뒤 풀림.
- Isaac: 재생 게이트 29/29, ws1 전체 pytest.

## 10. 자체 검토

(검토 뒤 채움)
