# 실차 안전 배선 — PR 5 (설계)

작성 2026-11-19 · 10판(Codex 적대 검토 9 회차 반영, 수렴 대상 5a — 총괄 2026-11-19) · 상위 [실차 자율 계획](../plans/2026-11-18-real-autonomy-entry.md) PR 5 행 · 앞 단계 [4b 조립](2026-11-19-real-assembly.md)(#599, 14 절의 PR 5 요구), [4c 루프](2026-11-19-real-loop.md)(#602)

## 범위

계획 PR 5 행(4e-2b 안전 배선)과 4b 설계 14 절의 '등록 팔레트 반사의 팽창 전 분리'를 한다. 4c 가 넘긴 `--run` 벽시계 배선과 실 `TtyLink` 의 gate 재통과는 6a 그대로다(총괄 2026-11-19).

- **Isaac 기본 동작은 바꾸지 않는다.** 새 코드는 실차 모듈(`src/forklift_core/live/`)과 기본 꺼진 선택 인자뿐이다. 공유 코드(`ObstacleGrid.snapshot`, 중재기, `SlamPoseTracker`)를 건드리는 곳은 기본값이 지금과 같음을 시험으로 고정하고, 재생 게이트 29/29 와 ws1 전체 pytest 를 거친다.
- 실 MCU 로 AUTO 를 내는 일은 여전히 막혀 있다(4c 장치 차단). PR 5 는 층·감시·복구의 **배선과 판정**을 가짜 원천으로 시험한다. 실센서 입력은 PR 7 이다.
- **S10 선행 조건으로 넘기는 것**(PR 5 가 증명하지 않음, 4c 설계 8 절과 같은 범위, 총괄 승인 2026-11-19): DISARM·링크 단절·Jetson 정지 각각의 실차 정지 거리와 포크 거동; 라이다 평면이 차체·포크 이동 공간을 대표하는 범위(빔 높이·거리·방향별 지우기 허용); 스캔 빔별 취득 시각과 시계 변환 오차(PR 7 브리지); Jetson 부하에서 층 갱신 지연(B·`overdue_end_s` 확정); D435i 장착 자세와 기울기 임계값; **승강 공간**(아래 2 절 끝).

| 항목 | 내용 | 절 |
|---|---|---|
| 실차 층 | `RealLayer`: 원시 오도메트리로 스캔 배치, 스캔이 시작하는 비동기 갱신, 결과를 요청 조건 전체에 묶어 설치 | 1 |
| 팔레트 반사 분리 | **계획 격자에서만** 등록 팔레트 반사의 표시를 뺀다(`ObstacleGrid.snapshot(suppress=)`, 기본 없음). 주행 증거는 그대로 | 2 |
| 무장 전 층 강제 | 층 생존·`obstacle_act` 확인, 실행 중 해제는 래치; 대기·무장 금지·고장의 구분 | 3 |
| 감시 | `safety_watch.yaml` 로 조립, IMU 는 별도 판정(실표본·연속성) | 4 |
| 콘솔 | 실차 요청 경로(팔레트 등록·운용자 시작 확인·현장 복구·재위치 확인)와 통합 시험 | 5 |
| 재위치 | 복구 완료 → 옛 증거 폐기 → 새 세대 자세 적용 → 그 뒤의 정지·층 → 확인 | 6 |
| 정지 뒤 출발·포켓 구간 | 두 방향이 가능하면 D5 에 두 방향을 묻는다(계획의 '각 방향 답함') | 7 |
| 단일 실행기 | 링크가 연 장치 fd 의 잠금(별도 PR 로 먼저 머지), 자율·텔레옵 모두 | 8 |

**구현 단계**(4c 와 같은 방식 — 단계마다 Codex 구현 검토 수렴, 단계별 PR):

| 단계 | 절 | 상태 |
|---|---|---|
| 5a | 1, 2, 3 | **수렴 대상**(이 판). 실차 층·계획 격자 분리·출력 구분. 공유 `ObstacleGrid.snapshot(suppress=)`·코어 계획 경계 |
| 5b | 4 | **수렴**(19판, Codex 9 회차) |
| 5c-1 | 5 | **수렴**(29판, Codex 10 회차) — 머지는 11-20 보드 확인 뒤 |
| 5c-1b | 5.1 | **수렴**(13판, Codex 13 회차) — 실차 콘솔 조작 화면, 5c-2 보다 먼저(총괄) |
| 5c-2 | 6 | **수렴**(14판, Codex 14 회차), 구현 수렴(Codex 7 회차, `9321857f`) — 재위치 배선, 가짜 SLAM 원천 계약. 5c 머지 뒤 main 에 맞춰 따로 PR(총괄) |
| 5d | 7 | **수렴**(8판, Codex 8 회차), 구현 수렴(Codex 6 회차, `5dba3140`) — 두 방향 질의, D5 포켓 확인 우회(`d5_pocket`) |

'가장 새 DRIVE 하나' 규칙(3 절)은 4c 의 빈틈이라 **단독 수정 PR**(`fix/real-loop-newest-drive`)로 먼저 머지한다(총괄): 미송신 옛 DRIVE 를 버린다. **반쯤 쓴 옛 DRIVE**(7 회차 P1)는 그 PR 에서 뺐다 — 반쪽을 버리는 재동기화가 실 TTY 에서는 구분자 `0x00` 으로 그 반쪽을 오히려 완성할 수 있고(그 PR 의 Codex 검토), 대기 프레임 경로도 같이 봐야 해서 **실 전송기 계약과 함께 6a** 에서 닫는다. 4c 운영 전송기는 DISARM 만 쓰므로 그동안 실차 경로는 없다. 실차 경계의 세부는 각 단계와 S10 선행 조건에서 닫는다 — 실 MCU AUTO 는 여전히 막혀 있다(총괄).

단일 실행기 잠금(8 절)은 별도 PR 로 11-20 뒤(총괄).

## 1. 실차 층 `RealLayer` (`src/forklift_core/live/real_layer.py`)

Isaac 층(`sim/isaac/obstacle_layer.py`)은 레이 캐스팅·합성 잡음·시뮬 시계를 품고 있어 실차가 가져오지 않는다(4c 원칙). 실차 층은 공유 `ObstacleGrid`·`DrivePermission` 위에 S10-A 가 쓰는 부분만 얹는다.

**코어가 읽고 쓰는 것**: 4c 가짜 트럭의 S10-A 전 구간 기록과 코어 AST 를 합친 표 — `refresh`(코어 호출 12 곳, 모두 반환값을 버림, 5 곳은 `direction` 생략), `limit`, `watchdog`, `permission`(`path_check`·`last_estop`·`config.stopping`), `snapshot`(코어 진단이 정지 때 읽음), `known_enabled`, 그리고 코어가 쓰는 `exempt`·`depth_support`. 표 밖의 속성은 `AttributeError` 로 ①이 된다(조용한 기본값 없음). 표는 시험으로 고정한다.

### 1.1 스캔 입력

- `add_scan(scan: RealScan)`: 센서 이름, **취득 시각**(호스트 단조 시계), 각도·거리, 빔별 유효 마스크, 자기 차체 반사 마스크.
- **빔 유효성**(전제 3): 거리 0·NaN·무효 품질 빔은 표시도 지우기도 하지 않는다. `+Inf` 는 브리지가 "유효한 무반사" 라고 밝힌 빔만이다. `−Inf`(REP-117 너무 가까움)는 유효 빔이며 공유 격자처럼 근거리를 표시한다(구현 2 회차 P1-1). 유효 빔 비율이 하한(잠정 0.5) 미만인 스캔은 격자에 넣지 않고 스캔 신선도도 갱신하지 않는다(`scans_dropped`).
- **센서 설정**: 장착 자세·`may_clear`·빔별 높이 한계(`limit_m`)는 `config/real/layer.yaml` 에서 온다. 측정 전 값은 provenance 를 `measured: false` 로 적는다. 없으면 조립 거부. 5a 의 `RealLayer` 는 센서마다 (장착, `may_clear`, provenance) 가 없으면 생성을 거부하고, `may_clear` 센서의 스캔은 `limit_m` 이 없으면 버린다; 설정 파일에서 읽는 배선은 조립 단계(PR 7 브리지와 함께).
- **원시 오도메트리 이력과 스캔 배치**(적대 1 회차 P1-2, 2 회차 P1-1, 3 회차 P2-7, 4 회차 P1-3, 5 회차 P1-3·P2-1): 루프의 `pose_history` 는 보정이 들어간 제어 자세라 쓰지 않는다. 루프가 STATE 마다 `(계측 구간, RealOdometry 자세, 연속 여부)` 를 따로 쌓는다(`odom_history`). 계측 구간은 `[수신 − 지연 상한, 수신]`, 지연 상한은 텔레옵 다리의 `DelayBound`(왕복 기준) 값이다. **텔레옵 스캔 경로(`tools/bridge_scan.py`)와 같은 방식으로 놓는다**:
  - 스캔의 첫 빔 시각부터 마지막 빔 시각까지를 **양끝 모두 감싸는 연속 이력**이 있어야 한다 — 그 사이에 단절이나 제외된 STATE(`delay_over`: 기준점 없음, 상한이 `state_delay_s` 초과)가 하나라도 있으면 그 스캔은 버린다.
  - **오차표 가정의 확인**(6 회차 P1-2, 7 회차 P2-1): 오차표에 더하는 시각 오차는 **스캔용 이동 상한** `v_scan = 1.5 × 명령 상한`(S10-A 0.15 m/s → 0.225 m/s, 4c 과속 감시와 같은 배수) 이하의 움직임을 가정한다. 명령 상한(새 명령을 자르는 값)과 따로 둔다 — 정상 접근(0.12 m/s)의 엔코더 상한(지연·가속 오차 포함 약 0.15 m/s)이 스캔 판정에 걸리지 않게. 스캔 구간(첫 빔 − dt 부터 마지막 빔까지)의 STATE 마다 엔코더 속도 상한이 `v_scan` 을, 또는 회전율 상한이 `v_scan·κ_max` 를 넘으면 그 스캔은 버린다 — 무장하지 않은 차를 사람이 밀거나 과속한 구간의 스캔은 격자에 들어가지 않는다. 시험: 설정 STATE 간격·지연에서 0.12 m/s 접근의 스캔이 채택됨.
  - **같은 부팅의 오도메트리 단절**(6 회차 P1-3): `RealOdometry` 는 단절 동안의 이동을 버리고 옛 자세 숫자에서 다시 적분한다. 그래서 단절이 생기면 텔레옵 브리지처럼 **층을 새로 만든다**(보관 스캔·설치 결과·계산 중 결과 모두 버림). 권한이 있으면 4c 대로 ①이고, 권한이 없을 때도 위치 증거가 끊겼으므로 **재위치 세대를 연다**(`odometry_break`, 6 절) — 재부팅과 같은 경로.
  - 스캔은 첫 빔 시각의 보간 자세에 놓는다. 시각 불확실성 `dt = 2 × 회전 주기 + state_delay_s`(회전 하나, 스탬프 앞의 시작, STATE 계측 시각)에서 생기는 위치·각 오차 `v_scan·dt`, `v_scan·κ_max·dt` 를 **오도메트리 나이 오차표에 더해** 격자에 준다(브리지와 같은 식). **채택 판정과 같은 `v_scan`** 을 쓴다(8 회차 P2-1: 점유 팽창과 FREE 침식이 채택 조건과 같은 가정을 따름). 격자는 그 오차만큼 FREE 를 줄이므로, 정상 접근 속도(0.12 m/s)의 회전 중 이동(1.2 cm)도 버리지 않고 오차로 흡수한다(고정 1 cm 허용 없음).
  - **시각 기준**: FREE 나이는 첫 빔 시각부터 센다(보수적). 처리 마감(watchdog)은 마지막 빔 시각 + B 다. 시험: 설정 접근 속도로 연속 주행할 때 스캔이 끊기지 않고 설치됨.
  - **전후 판정**(재위치 정지 창 등)은 계측 구간의 하한을 쓴다.
  - 4c 루프에 `DelayBound` 를 텔레옵 다리와 같이 배선하고, 무장하지 않은 동안 기준점이 만료되지 않도록 **기준점 갱신용 DISARM** 을 주기적으로(잠정 1 s) 보낸다 — OFF·LATCHED 에서만, S10 정지 확인(STOP_PENDING·DISARMING) 중에는 보내지 않는다(4 회차 P2-2).
  - 격자는 놓인 자세에 `map_from_odom` 을 한 번만 합성한다.
- 스캔 한 장의 회전 시간(10 Hz 에서 0.1 s) 동안의 움직임 오차는 S10-A 속도 상한(0.15 m/s)에서 1.5 cm 다. PR 5 는 이것을 격자 센서 오차 상한에 더하지 않는다. 빔별 시각 보정은 PR 7 브리지 계약이다(전제 7).
- PR 5 에서 스캔 원천은 주입된 가짜다.

### 1.2 비동기 갱신

1. **요청**: `refresh(now_s, correction, version, *, current_pose, path_ahead, loaded, direction=0)` 은 요청을 기록하고 `None` 을 돌려준다(반환값을 읽는 Isaac 동기 러너는 Isaac 층을 쓰므로 무관). 요청 열쇠는 `(보정 세대, 면제 일련번호, 미션 세대, 경로 식별, 적재)` 다. **경로 식별은 추적기 일련번호와 검사 대상 기어 구간**(`leg_index`)이다(3 회차 P2-2): cusp 에서 구간이 바뀌면 열쇠가 바뀌어 옛 구간의 경로 검사가 쓰이지 않고, 새 결과가 설치될 때까지 `snapshot_installing` 으로 출력은 0 이다. **방향은 열쇠에 넣지 않는다**(2 회차 P2-1): 코어는 한 틱에 앞뒤 두 방향을 같은 스냅숏으로 묻는다. 정지 호 검사는 방향마다 그 스냅숏에서 하고, 경로 검사는 경로의 진행 방향 하나다.
2. **갱신을 시작하는 곳**(2 회차 P2-1): (a) 새 스캔이 들어오면 마지막 요청 열쇠로; (b) 요청이 한 번도 없으면 부팅 첫 스캔에서 초기 열쇠(경로 없음, 빈 포크, 적용 보정)로; (c) 4c 루프가 매 틱 열쇠의 구성 요소(추적기 경로의 일련번호·단계, 미션 세대, 적재, 면제 일련번호, 보정 세대)를 보고 바뀌면 지금 경로로 `refresh` 를 직접 요청한다 — 코어가 보정 변경 때만 부르는 것을 메운다.
3. **합치기**(전제 12): 작업자가 바쁠 때 요청은 가장 새 것 하나로 합치되, **아직 반영 안 된 가장 오래된 스캔의 마감**(취득 + B)은 앞당겨진 채 유지한다. watchdog 은 그 마감으로 판정한다.
4. **계산**: 작업자 스레드가 그 시점 스캔 묶음의 사본으로 스냅숏·면제·경로 검사를 계산한다. 루프 상태는 바꾸지 않는다.
5. **설치**(적대 1 회차 P1-3): 제어 스레드가 틱 머리에서만 한다. 결과의 요청 열쇠가 **지금 열쇠와 모두 같을 때만** 설치하고, 아니면 버리고 셈한다. 면제가 좁아지면(해제) 면제 일련번호가 올라 계산 중인 옛 결과도 버려진다. 설치 때 `DrivePermission` 의 이동 거리는 **요청 때의 원시 오도메트리 자세부터 설치 순간까지 주행한 거리**로 시작하고(2 회차 P2-2: 0 이 아님), 이후 매 틱 그 이동 거리를 `permission.advance()` 에 더한다(Isaac 러너와 같은 규칙).
6. **보기**(4b `SnapshotView`): 설치된 결과의 보정·세대·가장 새 스캔 취득 시각. `installing` 은 **허가를 무효로 만드는 변경**(보정 세대·면제·미션·경로(기어 구간 포함)·적재 — 방향은 아님)이 아직 설치되지 않았을 때만 참이다. 같은 열쇠의 새 스캔 갱신은 `installing` 이 아니다 — 정상 입력에서 대기가 끝나지 않는 일이 없다(전제 12).

**허가**(`limit`): 다음이면 `(0, 사유)`. 설치된 결과의 열쇠가 지금과 다름 → `snapshot_installing`; 담긴 가장 새 스캔이 `SCAN_MAX_AGE_S` 초과 → `scan_stale`; watchdog 정지; 그 밖에는 `DrivePermission.allowed_speed`. 경로·정지 범위의 셀마다 FREE 나이는 `allowed_speed` 가 이미 `evidence_max_age_s` 로 본다(`drive_permission.py` 980 행, 전제 6) — `SCAN_MAX_AGE_S` 는 입력 생존 판정이다.

**지연 상한 B 의 반영**(3 회차 P1-2): 같은 열쇠의 새 스캔이 처리되는 동안 B 까지 옛 결과로 주행하므로, 조립이 정지 모형 지연(`StoppingModel.latency_s`)에 **회전 시간 + B** 를 더하고(6 회차 P2-2: 첫 빔에서 새로 보인 장애물은 마지막 빔까지 한 회전, 그 뒤 처리 B 동안 옛 결과로 주행한다 — 기본 지연은 제어·제동 지연이고 회전 시간을 포함하지 않는다), FREE 사용 나이는 **격자의 `GridConfig.free_use_age_s` 를 B 만큼 늘린 뒤 같은 값을 허가의 `evidence_max_age_s` 로** 넘긴다(4 회차 P1-2: 격자의 FREE 침식 반경이 그 나이로 계산되므로 둘이 같아야 함) — Isaac 층의 예산 적용과 같은 순서다. 시험: 조립된 격자·허가 설정의 세 값.

**watchdog**(Isaac 층과 같은 두 판정, 2 회차 P2-2; **버린 스캔도 마감에 올린다** — 7 회차 P2-2: 버린·도착하지 않은 스캔 때문에 옛 층을 `회전 시간 + B` 넘게 쓰지 않도록, 스캔 원천에 도착한 모든 스캔의 마감(마지막 빔 + B)을 등록하고, 그 마감까지 그보다 새 스캔이 설치되지 않으면 `layer_overdue` 다. **도착하지 않은 스캔**(8 회차 P2-2)도 같은 한도로 막는다: 마지막으로 받은 스캔의 마지막 빔 + 공칭 회전 주기 + B 를 **다음 스캔의 예상 마감**으로 두고, 그때까지 새 스캔이 하나도 도착하지 않으면 `layer_overdue` 다. 마감 해소는 스캔 ID 로 판정한다 — 정상 스캔은 자기 결과가 마감 안에 설치되면, 버린 스캔은 그보다 새 스캔의 결과가 설치되면 해소한다(8 회차 P3). **이미 지난 마감은 그것을 채우는 결과가 설치될 때까지 유지한다**(9 회차 P2): 늦게 도착한 스캔의 수신은 지난 예상 마감을 풀지 않고, 그 스캔(또는 더 새 스캔)의 결과가 설치돼야 풀린다 — 수신과 설치 사이에는 대기다. 시험: 늦은 수신과 설치를 다른 틱에 넣어 그 사이 비영 DRIVE 0 건; 정상 스캔을 늦게 설치해 대기했다가 다음 스캔이 제때 설치되면 풀림(지난 마감이 영구히 남지 않음, 9 회차 P3)): (i) 반영 마감(취득 + B)이 지난 스캔이 설치 결과에 없음, (ii) 설치 결과가 처음 쓰인 시각 − 그것이 담은 가장 새 스캔의 취득 시각 > B. 둘 중 하나면 `layer_overdue`, 이어진 시간이 `overdue_end_s` 를 넘으면 `layer_latency_exceeded`(①). 작업자가 죽었으면 `layer_worker_dead`(①). B·`overdue_end_s` 는 잠정(Isaac 예산값), Jetson 부하 측정으로 확정(S10 선행 조건).

**SLAM 무효 뒤 입력만 복구**: SLAM 무효는 4b 준비 판정의 래치 사유다. 입력이 돌아와 새 결과가 설치돼도 재무장 전에는 출력이 없다.

**계획 경계**(적대 1 회차 P2-1, 2 회차 P2-3):
- 계획 격자(2 절)는 작업자가 **갱신 결과의 일부로** 만든다: 미션이 고정한 팔레트가 있을 때 `(고정 팔레트, 보정 세대)` 열쇠로 분리 스냅숏을 계산해 함께 설치한다. 제어 틱에서 동기로 계산하지 않는다.
- `RealHooks` 의 계획 격자 훅(`grid_kwargs` 가 부르는 `_occupancy()`)은 설치된 계획 격자의 열쇠가 지금 `(고정 팔레트, 적용 보정 세대)` 와 같을 때만 그 격자를 주고, 아니면 `planning_grid_not_ready` 로 거부한다.
- 4c 루프는 `known_request` 를 그 격자가 준비됐을 때만 내고, 그때의 **격자 보정 세대**를 기록한다. 채택(`job_hold`) 전에 지금 적용 세대와 다르면 답을 버리고 다시 요청한다(3 회차 P2-3: 코어의 `_job_invalid` 는 보정 값만 비교해 A→B→A 를 놓친다).
- **채택 전 준비 판정**(3 회차 P2-4): 4c 루프는 `job_hold` 보다 먼저 3 절의 구분(고장·대기)을 그대로 적용한다 — 준비 판정과 층 watchdog(`layer_overdue` 포함)을 함께 읽는다. 고장이면 채택하지 않고 **허가 래치 경로**(`AutoPermit.latch`, OFF 에서도 LATCHED 로 — 재무장 필요)로 보낸다. 허가는 채택 전 미션을 자기가 수거하지 않으므로(`_collected` 가 참), 루프가 같은 틱에 `accepted`·`preparing` 미션을 그 고장 사유로 한 번 끝내고(`_end_now`) 계획 작업과 답을 버린다(4 회차 P2-3). 대기면 그 틱의 채택을 미룬다(답은 버리지 않음).

### 1.3 그 밖의 속성

- `exempt`: D5 포켓 영역 면제. **설치하는 곳**(8 회차 P2-3): 실차 D5 생성 훅(`RealHooks.pocket_check`)이 Isaac 훅과 같은 규칙으로 — 검사기가 유효할 때만(`check.valid`, 아니면 `pocket_check_refused`) `layer.exempt = check.region_control()` 을 설치한다(`depth_support` 는 실차가 쓰지 않음). 5a 는 유효한 가짜 D5 검사기와 그 검사 영역을 함께 배선해, 계획 성공 → 면제 설치 → 주행 격자의 팔레트 점유를 면제 영역에서 넘음을 시험한다(면제 없으면 `occupied` 로 0). 바뀌면 면제 일련번호가 오른다. 좁아지는 변경(해제)은 설치된 결과도 무효로 해 `snapshot_installing` 으로 기다린다 — 층 생존(`layer_ok`)은 그대로라 래치가 아니라 대기다(적대 P1-4).
- `depth_support`: 실차는 그림자 띠 기억을 쓰지 않아(엄격 규칙) 저장만 한다.
- `known_enabled = False`(4a 가 켜짐을 거부), `memory = None`, 출구 면제 없음.
- `planner_grid(...)`: 2 절.

## 2. 등록 팔레트 반사 분리 (4b 14 절)

문제: 정확히 등록한 팔레트의 전면 반사가 격자 팽창으로 외곽 밖 셀(전면 2.70 m 앞 2.675 m 셀)에 남아 **삽입 계획**을 막는다.

적대 P1-1: 주행 증거에서 반사를 빼면 팔레트 앞의 실제 물체도 같이 사라진다. 그래서 **계획 격자에서만 분리한다**.

- `ObstacleGrid.snapshot(now_s, map_from_odom, version, *, suppress=None)` 에 선택 인자를 더한다. `suppress` 는 지도 좌표 사각형 목록이다. 보관된 모든 스캔에서 끝점이 그 사각형 안에 드는 **정상 유한 반사의 표시만** 빼고, 지우기·오차 상한·셀 한 칸 여유·`may_clear`·높이 한계는 그대로다. `None`(기본)이면 지금과 바이트까지 같다(시험으로 고정).
- 보관 스캔 전체를 스냅숏 때 다시 분류하므로, 미션 전에 들어온 스캔의 반사도 빠진다(적대 P2-2).
- `RealLayer.planner_grid(...)` 만 `suppress=[등록 팔레트 외곽 + 센서 오차 상한]` 으로 스냅숏을 만든다. 등록은 현재 미션이 고정한 자세(`FrozenSubmit`, `basis=operator_input`)다. 등록이 없거나 미션 밖이면 분리하지 않는다.
- **주행 허가용 스냅숏은 분리하지 않는다.** 팔레트는 주행 격자에 장애물로 남고, 삽입 직선의 포켓 영역은 지금처럼 D5 면제(`exempt`)가 맡는다(4a). 등록이 틀려 팔레트가 비뚤면 첫 프레임 정지의 D5 검사가 삽입을 거부한다(안전한 실패).
- 계획 격자에서 팔레트 바로 앞의 다른 물체가 외곽+오차 안에 들면 계획에서 빠진다. 주행 격자에는 남아 허가가 막으므로 충돌이 아니라 정지로 끝난다 — **면제 사각형 밖이라면**.
- **면제 영역 안의 독립 물체와 승강 공간**(2 회차 P1-2): `exempt_region` 은 포켓 사각형 안의 점유를 모두 FREE 로 덮는다. 그 안은 D5 가 **현재 포크 높이**에서만 답한다. 그래서 (a) 포켓 입구에서 포크보다 높고 승강 범위 안에 있는 돌출물, (b) 삽입 뒤 승강(운반 높이까지) 중 팔레트 위 공간은 PR 5 의 어떤 센서도 증명하지 않는다. S10-A 에서는 **감독 아래 배치 조건**으로 둔다: 시험 팔레트의 포켓·윗면·주변에서 **조립이 실제로 로드한 승강 목표**(`autonomy.yaml` 의 `lift_target_m`, 지금 0.20 m)에 팔레트 높이를 더한 높이까지 다른 물체가 없음을 운용자가 시작 확인(`operator_start`) 때 확인한다(3 회차 P1-3). 콘솔 문구는 그 값을 조립 기록에서 읽어 보인다. 센서로 증명하는 일은 S10 선행 조건 목록에 남긴다.
- 회귀 시험: `ObstacleGrid → plan_transport`. 같은 스캔으로 분리 끔 → 계획 실패(지금 증상), 켬 → 성공, 팔레트 옆 상자는 켬에서도 막힘, 주행 스냅숏에는 팔레트가 남음.

## 3. 무장 전 층 강제와 출력 구분

- `Evidence.layer_ok` = `RealLayer.ok(now)`: 작업자 생존, 스캔 원천 연결, **한 번 이상** 설치된 결과. 면제 해제 등으로 지금 결과가 무효인 것은 `ok` 를 바꾸지 않는다(대기).
- 4c 루프는 매 틱 `core.options.obstacle_act` 와 `core.obstacle["layer"] is real_layer` 를 확인하고, 바뀌었으면 ①이다.
- **가장 새 DRIVE 하나만**(6 회차 P1-1, 4c 에 있던 빈틈): 4c 루프는 정상 AUTO DRIVE 를 큐에 넣을 때 쓰지 않은 옛 DRIVE 를 남겨 둬, 관문이 0 을 낸 틱에도 옛 비영 DRIVE 가 먼저 나갔다. 루프가 DRIVE 를 큐에 넣기 전에 호스트의 미송신 DRIVE 를 모두 버린다(`drop_drives("superseded")`) — 큐에는 늘 가장 새 결정 하나만 남는다(대기 프레임 `hold_frame` 이 이미 하는 규칙을 정상 출력에도). 시험: 접근 중 송신이 한 틱 밀린 뒤 중간 정지 관문이 닫히면, 관문 뒤 실제로 쓰인 비영 DRIVE 가 0 건.
- **세 가지 구분**(적대 P1-4), 최종 송신 경계(4c 첫 바이트 판정)까지 같다:
  - **고장** → ① DISARM: 층 작업자 죽음, `layer_latency_exceeded`, 스캔 원천 끊김, IMU 필수인데 끊김(4 절), `obstacle_act` 해제.
  - **대기**(무장 유지, 바퀴 0): `wait:` 사유(`snapshot_installing` 등)와 `layer_overdue`.
  - **신규 ARM 금지**: 대기 사유가 하나라도 있으면 ARM 프레임은 첫 바이트에서도 거부한다. 지금 첫 바이트 판정은 대기 사유를 정상 DRIVE 에만 적용하므로, ARM 에도 적용하고 그때는 허가를 래치한다(`arm_wait` — 재무장 필요, 보수적). ARMING 중 호스트의 확인용 0 DRIVE 는 바퀴 0 이라 그대로 둔다.
- **정지 뒤 방향 판정**: 7 절.

## 4. 감시 (5b)

> **5b 수렴**(19판, Codex 9 회차 새 P1/P2 없음): 이 절만 검토했다(총괄 2026-11-19). 5a 의 결정은 바꾸지 않는다.

**설정과 조립**(`src/forklift_core/live/real_assembly.py` 의 `build_watches(autonomy, watch_path, wheel_radius_m)` → `Watches(safety, imu, imu_policy)`; 바퀴 반경은 조립이 읽은 차량 설정의 좌우 값):
- `config/real/safety_watch.yaml` 에서 `SafetyWatch` 를 만든다(`load_watch_config`, 4c 그대로). 파일이 없거나 깨지면 조립 거부(`watch_config:<이유>`).
- IMU 값은 같은 파일의 **새 `imu:` 부분**에 둔다(`tilt:` 은 텔레옵 브리지의 `SafetyWatch` 몫으로 그대로): `limit_deg`(잠정 8), `hold_s`(0.3), `max_age_s`(0.1), `gap_max_s`(0.05), `baseline_min`(10), `baseline_spread_deg`(2.0), `baseline_wait_s`(0.5), `norm_tol_frac`(0.30), `level_ref`(자리표시 `[0, 0, 1]`, S7 측정), `level_tol_deg`(5), `provenance`. 판정이 가능한 범위만 받는다: `0 < limit_deg < 90`, `0 < baseline_spread_deg < limit_deg`, `gap_max_s ≤ max_age_s`, `hold_s > gap_max_s`, `0 < norm_tol_frac < 1`, `baseline_min ≥ 2`, `0 < level_tol_deg < limit_deg`, `level_ref` 가 유한한 수 세 개의 목록(매핑·문자열·불리언·부동소수로 못 바꾸는 큰 정수 아님, 영벡터 아님, 7–8 차); 아니면 조립 거부(`imu_config:<이름>`).
- `config/real/autonomy.yaml` 의 `imu: required | none`(잠정 `required`). 키가 없거나 다른 값이면 조립 거부(`imu_policy`). `none` 이면 `ImuWatch` 를 만들지 않고 조립 기록에 `imu: none`.
- 건식 조립(`tools/real_autonomy.py --dry-assemble`)과 실행 조립이 **같은** `build_watches` 를 부르고, 루프는 그 `Watches` 를 **통째로** 받는다 — `SafetyWatch` 만 따로 넘기는 인자는 없다(4 차 P2-1). `RealLoop(watches=, imu_source=)` 는 `imu_policy == "required"` 인데 `ImuWatch` 나 원천 중 하나라도 없으면 생성 거부(`imu_required`) — 두 주입 인자를 빠뜨려 무감시로 도는 길이 없다. `watches=None`(4c·5a 시험 rig)은 지금과 같음.
- `SafetyWatch` 의 기울기 부분은 실차 자율에서 쓰지 않는다 — 루프는 `accel` 을 넘기지 않는다(적대 P1-5). 전류·바퀴는 4c 그대로.

**IMU 원천 계약**(`imu_source.samples(upto_ns, limit)`; 취득 시각은 정수 호스트 ns — 정수가 아닌 표본은 루프가 그것만 버리고 정수는 정밀도 그대로 정렬·비교한다, 4–5 차 P2): 이미 받아 둔 표본 중 **취득 시각이 `upto_ns` 이하인 것만** 막히지 않고 즉시, 최대 `limit`(64) 개 `(취득 호스트 ns, 가속 3 축)` 를 **취득 시각 오름차순**으로 돌려준다 — `upto_ns` 보다 뒤의 표본은 원천에 남아 다음 조회에 나온다(3 차 P2-3: 조회 중 도착한 표본을 미래로 버리지 않음). 루프는 조회 **직전에** 시계를 읽어 `upto_ns` 로 쓰고 `add` 의 `now_ns` 도 같은 값이다(틱 시작 시각 아님). 루프도 넣기 전에 시각순으로 정렬한다(2 차 P2-3). 장치 읽기는 원천의 자기 스레드가 한다. 예외면 그 회는 표본 없음(끊김은 `judge` 가 잡음). 5b 의 가짜 원천과 PR 7 의 D435i 원천이 이 계약을 따른다(1 차 P1-3).

**`ImuWatch`**(`src/forklift_core/safety/real_watch.py`, 호스트 단조 시계 ns):
- `add(sample_ns, accel, now_ns) -> str | None`: 유효하면 None, 아니면 버린 이유(셈만). 버림: 비유한 값, `sample_ns` 가 지난 유효 표본 이하(재전달·되감김), `sample_ns > now_ns`(미래), `|‖a‖ − 9.81| > norm_tol_frac × 9.81`(비율, m/s² 아님). 유효 표본은 최근 2 s 만 둔다.
- `baseline(still_since_ns) -> ((방향, last_ns), None) | (None, 이유)`: 취득 시각이 `still_since_ns` 이상인 유효 표본 중 **가장 최근의 공백 없는 연속 구간**(이웃 공백 ≤ `gap_max_s`; 앞선 공백은 구간이 늦게 시작했다는 뜻일 뿐)으로 — 정지 아님 `imu_not_still`; 그 구간이 `baseline_min` 미만 `imu_no_baseline`(기한 안이면 대기); 평균 벡터 크기가 단위의 0.9 미만이거나 최근 구간의 어느 표본이든 그 평균 방향에서 `baseline_spread_deg` 넘게 벗어나면 `imu_baseline_unsteady`(그 표본이 창 밖으로 나가면 풀림, 6 차 P2-2); 기준 방향이 수평 참조에서 벗어나면 `imu_not_level`(1 차 P1-2: 섞인 자세·반대 벡터의 평균).
- **수평 참조**(6 차): 기준은 정지 구간의 이력이 아니라 **선 자리**로 확인한다 — 기준 방향이 설정의 `level_ref`(평평한 바닥에서 IMU 좌표의 중력 방향, D435i 장착이 기울어 있어 S7 측정 전에는 자리표시값)에서 `level_tol_deg`(잠정 5°, `limit_deg` 미만)를 넘으면 `imu_not_level`(ARM 전 거부). 정지 중 공백·정지 판정 상실 뒤 바뀐 자세도 선 자리로 잡힌다. 수평 참조 안의 작은 자세 변화는 `limit_deg` 판정의 여유로 남는다(측정 전 잠정, S10 선행 조건).
- **ARM 응답 전 움직임**(6 차 P2-1): `arm()` 부터 허가가 READY 가 될 때까지(`settled()`) 기준에서 `baseline_spread_deg` 를 넘는 표본이 하나라도 오면 지속 없이 `watch_imu_moved` → ① — ARM 의 첫 바이트 판정 포함. 경계는 **표본의 취득 시각**으로 정한다(8 차 P2): 루프가 허가를 처음 READY 로 본 틱의 IMU 조회 시각을 `settled(경계)` 로 한 번 고정하고, 그 시각 이하에 취득한 표본은 틱 머리든 첫 바이트든 어느 조회로 오든 ARMING 규칙, 그 뒤 표본은 지속 규칙이다(READY 는 그보다 늦지 않으므로 보수적). READY 틱에도 판정을 먼저 한다(7 차 P2-1).
- `baseline` 은 방향과 함께 **기준에 쓴 마지막 표본의 시각**을 돌려준다. `arm((direction, last_ns))`: **ARM 요청 전에** 기준을 고정한다(무장 동안 바꾸지 않음). 판정은 `last_ns` 에서 시작한다 — 그 뒤 받아들인 첫 표본과의 공백부터 잰다(2 차 P2-1: 고정 직전 취득·직후 수신 표본이 공백을 숨기지 않음). `disarm()`: 기준·판정 상태·고정된 사유를 버림.
- `judge(now_ns) -> str | None`(고정된 기준이 있을 때만):
  - `watch_imu_stale`: 가장 새 유효 표본이 `now_ns − max_age_s` 보다 오래됨.
  - `watch_imu_gap`: 기준 고정 뒤 이웃 유효 표본 사이 공백이 `gap_max_s` 초과(1 차 P1-1: 드문드문 오는 표본으로는 지속을 판정할 수 없다 — 그 자체가 감시 상실).
  - `watch_tilt`: 기준 대비 각도가 `limit_deg` 를 넘는 표본이 이어진 첫 표본부터 마지막 표본까지가 `hold_s` 이상. 한도 안 표본이 지속을 처음부터 다시 세게 한다. 한 표본은 지속이 아니다.
  - 판정은 새로 받은 표본만 차례로 처리하는 **증분 상태**(직전 표본 시각, 지속 시작 시각)로 한다 — 표본 보관 창과 무관하게 `hold_s` 가 얼마든 지속을 끝까지 잰다(2 차 P1-1).
  - 한 번 낸 사유는 `disarm()` 까지 고정.

**루프 배선**:
- 틱 머리(STATE 처리 뒤)와 첫 바이트 판정 안에서 원천 표본을 `add` 한다. 첫 바이트에서는 **기존 준비 판정과 IMU 표본 수집·`judge(ns)` 를 모두 같은 읽기 예산(`EVIDENCE_READ_MAX_S`) 안에서** 한 뒤 시계를 다시 읽어, 상한 `ns` 를 넘었으면 `evidence_slow` alarm 이다(4c 규칙; 2 차 P1-2).
- **ARM 전**(채택 뒤 ARM 조건에서):
  - 기존 `_pre_arm_refusal` 을 **먼저** 묻는다: `not_still` 이 아닌 거부(`outstanding`·`mcu_not_disarmed`·래치 사유)는 지금처럼 즉시 래치한다(3 차 P2-2). `not_still` 이거나 거부가 없으면 IMU 판정으로 간다 — `not_still` 또는 IMU 의 `imu_no_baseline`·`imu_not_still` 이고 그 미션의 **대기 기한** 전이면 **대기**: `adopted_for` 를 기록하지 않고 ARM 을 다음 틱으로 미룬다(5a 의 층 대기와 같음, 1 차 P2-1). 대기 기한은 미션마다 처음 대기에 들어간 틱에 한 번 `지금 + baseline_wait_s` 로 정하고 정지가 끊겼다 다시 잡혀도 늘리지 않는다(2 차 P2-2).
  - 그 밖의 이유이거나 기한이 지났으면 `pre_arm:<이유>` 로 래치(5a: ARM 전 래치 = 미션 종료).
  - 기준을 얻으면 `arm((방향, last_ns))` — `baseline` 이 돌려준 마지막 기준 표본 시각 — 뒤에 `permit.adopted()` 를 부른다(3 차 P2-1). 그것이 실패하면 `disarm()`.
- **무장 중**(ARMING·READY·DRIVING·STOP_PENDING): 틱 머리의 `judge(now_ns)` 가 사유면 `watch:<사유>` 로 ①. 첫 바이트에서도 `judge(ns)` 가 사유면 `send:watch:<사유>` alarm.
- 허가가 OFF·LATCHED 이면 `disarm()`.
- IMU 원천의 실배선(D435i 가속, PR 7 의 시각 변환)은 이 단계에 없다. 5b 는 가짜 원천으로 판정과 배선만 시험한다.

**고장 구분**(3 절의 셋): `watch_imu_stale`·`watch_imu_gap`·`watch_imu_moved`·`watch_tilt` 는 고장 → ①. `imu_baseline_unsteady`·`imu_not_level`·기한 지난 `imu_no_baseline`·`imu_not_still` 은 ARM 전 거부(래치·미션 종료). 기한 안의 기준 축적은 대기.

- 임계값(기울기·진동 영향·공백·나이·기준 분산)은 측정 전 잠정값이다(S10 선행 조건, 전제 8).

## 5. 콘솔 (5c-1)

> **5c-1 수렴**(29판, Codex 10 회차 새 P1/P2 없음): 이 절만 Codex 검토로 수렴시켰다(총괄 2026-11-19: 5c 를 5c-1 = 5 절, 5c-2 = 6 절로 나눔). 중재기·`validate` 는 텔레옵 브리지의 import 경로라 **머지는 11-20 보드 확인 뒤**(총괄).

4b·4c 에서 lease·STOP·DISARM·재무장 정책은 끝났다. 실차에서 콘솔로 끝까지 가려면 요청이 더 있어야 한다(적대 1 회차 P2-3, 2 회차 P2-5).

**중재기**(`src/forklift_core/safety/arbiter.py`, 공유 — 기본값 불변):
- `ArbiterConfig.real_requests: bool = False`. `False` 면 지금과 **같다**: 새 요청 이름은 신뢰 요청(`RELIABLE`)이 아니라 지금처럼 응답 없이 끝나고, `recovery` 는 지금처럼 `unsupported` 다(2 회차 P3).
- `True`(실차 설정만): `REAL_LEASED = ("pallet_register", "operator_start", "operator_answer", "check_bypass")` 를 신뢰·lease 요청으로 다루고(세션·lease·제어 세대 검사, 요청 저장·원장, `session_ending` 등 기존 순서 그대로), 처리기는 `Arbiter.real_request(kind, msg, now)` 콜백(루프가 설정)을 불러 그 답 `(사유 | None, 필드)` 를 ack/reject 로 돌려준다. `recovery` 도 같은 콜백으로 간다(5c-1 은 `action: "done"` 만). 콜백이 없으면 `unsupported`.
- **공유 `validate.problem` 은 바꾸지 않는다**(1 차 P2-1: 채널이 중재기 전에 검증해 기본값의 응답이 바뀜). 새 요청의 필드 검사는 중재기의 실차 처리기 안에서만(`real_requests` 켬일 때, 세대 검사 뒤): `pallet_register` = `pallet_id` 비지 않은 문자열·`pose` 유한한 수 셋; `operator_answer` = `kind` 문자열·`serial` 정수(불리언 아님)·`verdict ∈ {confirm, reject}`; `recovery` = `recovery_id` 정수·`action == "done"`; `check_bypass` = `check` 문자열·`on` 불리언. 아니면 `invalid`.
- **연결 토큰 원장 필수**(1 차 P2-2): `real_requests` 켬에서 `REAL_LEASED` 와 `recovery` 는 `request_token` 이 없으면 `token_required` 로 거절한다(저장 안 함) — 원장(연결 토큰·요청 번호)을 거쳐서만 실행된다. 기존 요청은 지금 규칙 그대로. 원장 정리(`purge`)는 `request_tokens` 나 `real_requests` 가 켜져 있으면 주기마다 한다(2 차 P2-4). 필드 검사에서 부동소수로 못 바꾸는 큰 정수는 `invalid`(2 차 P3-2).

**루프의 처리**(`RealLoop._real_request`, 콘솔 주기 안 — 4c 루프 스레드, 틱 머리; 모든 실차 요청은 상태를 바꾸기 전에 **시계를 다시 읽어 lease 가 그 시각에 유효한지** 먼저 보고 아니면 `no_lease` — 중재기는 틱 시작 시각으로 본다, 9 차 P2):

| 요청 | 처리 | 거절 사유 |
|---|---|---|
| `pallet_register{pallet_id, pose}` | `register_pallet` → ack 에 새 `targets_revision` | 진행 중 미션(`accepted`…`resuming`·`held`) `mission_active`; 등록 목록에 없는 ID `unknown_pallet`(1 차 P3-2) |
| `operator_start{session_epoch}` | `OperatorStart.confirm(epoch, 지금 lease)` | 세션 다름은 원장의 `stale_epoch`(2 차 P3-1); 무장 중(허가가 OFF·LATCHED 아님) `armed`; 정지 아님 `not_still` |
| `operator_answer{kind, serial, verdict}` | 루프가 `(serial, verdict)` 를 답 상자에 둔다 → 코어 운용자 훅(`operator_check`)이 다음 판정에서 읽고, 지금 요청의 `kind`·`serial` 이 맞을 때만 쓴다(코어 규칙 그대로). ack 는 **접수**다(코어 반영은 `real_status` 로 보임). 한 `serial` 에는 **첫 답만** 받는다(1 차 P2-3) | 열린 확인 요청 없음 `no_operator_request`; `kind`·`serial` 다름 `stale_operator_request`; 그 `serial` 에 이미 답함 `already_answered` |
| `recovery{recovery_id, action:"done"}` | 콜백 안에서 **시계를 다시 읽은** `now` 로(3 차 P2-2) lease 가 그 시각에 유효한지 먼저 보고(아니면 `no_lease`, 7 차 P2-2) `core.field_recovery_done(t, id, disarmed=허가.disarm_confirmed(now), outstanding=포트 outstanding(now) ≠ settled)` → 받아들이면 재위치 세대가 열림(이후는 5c-2) | 코어의 거절 사유 그대로(`no_open_recovery`·`stale_recovery`·`mission_active`·`not_disarmed`·`commands_outstanding`) |

| `check_bypass{check, on}` | 아래 **검사 우회**: 우회 집합에 넣거나 뺀다, 기록 | 우회할 수 없는 검사 `not_bypassable`; 모르는 이름 `unknown_check`; 무장 중(허가가 OFF·LATCHED 아님) `armed`(1 차 P1) |

- `AutoPermit.disarm_confirmed(now_ns)`: 기다리는 DISARM 일화가 없고, **호스트 큐와 전송기의 반쯤 쓴 프레임에 DISARM 이 없으며**(앵커 DISARM 포함, 1 차 P2-4) **받아들였지만 아직 처리 전인 콘솔 DISARM 도, 채널에 들어와 아직 중재기를 거치지 않은 콘솔 DISARM 도 없으며**(2 차 P2-1, 6 차 P2-2), DISARM 을 쓴 부팅과 응답의 부팅이 같고(4 차 P2-2), 가장 새로 **다 쓴** DISARM 의 seq 를 담은 **같은 부팅의 신선한** DISARMED 상태가 있음(적대 P1-6: `disarm_unconfirmed` 플래그는 시한 전까지 거짓이라 쓰지 않음).
- 운용자 시작 확인의 무효화 호출점(1 차 P2-5; 4c 에는 없었음): 틱 머리에서 정지가 끊기면 `void("moved")`, 장치 연결이 끊기면 `void("link_lost")`, lease 가 바뀌면 `ok()` 가 lease 로 거름, `recovery` 가 받아들여지면 `void("field_recovery")`. `operator_start` 를 우회하면 제출 판정과 채택 준비의 `take()` 가 **둘 다** 건너뛴다. `operator_start` 우회를 켜거나 끄면 남아 있는 확인을 `void("bypass_changed")` 로 버린다(2 차 P2-2: 우회 중 남은 확인이 우회를 끈 뒤 다른 제출에 쓰이지 않음). 같은 콘솔 묶음의 시작 확인과 제출은 제출이 그 틱 머리에 계산한 거절로 판정되어 **보수적으로 거절**된다(운용자가 다시 제출; 제출 판정의 순서는 공유 명령 코드라 바꾸지 않음).
- 코어의 열린 확인 요청은 읽기 전용 접근자 `MissionCore.operator_open()`(종류·일련번호·단계, 없으면 None)로 읽는다 — 코어의 판정·상태는 바꾸지 않는다.

**검사 우회**(사용자 지시 2026-11-19, 총괄 전달: "실제 상황에서는 유동적으로 조절할 일 많을테니까 너무 고정적으로 작동하게 설계하지는 마 … imu체크 부분을 잠깐 내려두고 구동 가능하게"): 실차 검사는 조건이 안 맞을 때 운용자가 콘솔에서 **그 검사를 잠깐 내려 두고** 진행할 수 있다. 검사마다 **켬/우회** 둘뿐이다(학교 규모).
- 범위는 **세션 한정**: 루프의 우회 집합은 세션이 시작할 때 비어 있고 세션이 끝나면 사라진다. 켜고 끌 때마다 기록(`bypass` 사건: 검사·켬/끔·lease·시각)하고, `real_status` 에 `bypassed: [검사…]` 를 늘 싣는다(콘솔이 '우회 중: <검사>' 를 계속 보임).
- 우회된 검사는 그 판정을 **통과로 본다**(거부·대기·① 모두 내지 않음). 우회를 켜도 이미 걸린 래치는 그대로이고, 풀려면 재무장(우회할 수 없음)이 필요하다.
- **전환은 다음 틱 머리에 적용**(3 차 P2-1): 받아들인 전환은 (검사, 켬/끔, 요청한 lease) 로 대기열에 두고 다음 틱 머리(제출 판정 전)에 적용한다 — 한 틱의 모든 판정(제출 거절·준비·`take()`·ARM·첫 바이트)은 그 틱 머리의 우회 집합을 쓴다. 전환이 대기 중인 틱에는 ARM 을 내지 않는다.
- **전환의 조건**(4 차 P1·P2-1): 받을 때와 적용할 때 모두 허가가 OFF·LATCHED 이고, **진행 중 미션이 없고**(`accepted`…`recovering`), 요청한 lease 가 지금 lease 이며 중재기 규칙으로 만료되지 않았고 보유자의 **지금 연결**의 끊김이나 **같은 클라이언트의 새 연결**(lease 를 되찾음)이 대기 중이 아니어야 한다(5 차 P2, 6 차 P2-1, 7 차 P3: 교체된 옛 연결의 끊김은 셈하지 않음). 적용 때의 판정과 집합 변경은 채널 잠금 하나 안에서, 잠금을 얻은 뒤 읽은 시계로 한다(7 차 P2-1, 8 차 P2). 받을 때의 판정도 그 순간의 시계로(6 차 P3-1). 받을 때 아니면 그 사유(`armed`·`mission_active`·`lease_changed`)로 거절, 적용할 때 아니면 적용하지 않고 `bypass_dropped:<사유>` 를 기록한다(`real_status.bypassed` 가 실제 집합을 보임) — 같은 묶음의 우회 해제와 제출은 제출이 먼저 받아들여져 해제가 버려지고, 미션은 그 우회 아래 그대로 간다. 기록은 검사·전환(켬/끔)·lease·시각 — 버린 것도(5 차 P3-1).
- **켜고 끄기는 무장하지 않았을 때만**(허가 OFF·LATCHED, 1 차 P1): 무장 중 전환은 `armed` 로 거절한다 — IMU 를 우회한 채 기준 없이 무장한 차가 주행 중 `imu` 를 다시 켜도 감시가 생기지 않는 빈틈을 막는다. 다시 켠 검사는 다음 정상 ARM 에서 기준부터 다시 얻는다.
- 세션: 우회 집합은 그 세션의 `session_epoch` 에 묶인다 — 같은 세션의 재접속에서는 유지되고, 세션 종료 요청(`session_end`)이 받아들여지거나 새 세션이면 비운다(1 차 P3-1).
- **적용점**(1 차 P2-6·P2-7): 우회는 그 사유가 나는 모든 곳에서 같은 규칙으로 건너뛴다 — `imu` 는 ARM 전 기준 관문·틱 머리 판정·첫 바이트 판정(`ImuWatch` 를 무장하지 않음); `layer_timing` 은 층에 `timing_bypassed` 를 켜 watchdog 의 `layer_overdue`·`layer_latency_exceeded`·`limit()` 의 `scan_stale` 과 overdue·`ok()` 의 스캔 나이·준비 판정 `snapshot:scan_stale`·ARM 대기·첫 바이트 판정을 모두 건너뛰되 `layer_worker_dead`·`scan_source_lost`·허가 판정(장애물 속도 제한)은 그대로; 준비 판정 항목은 `auto_ready_refusal(..., bypass=)` 가 그 범주만 건너뜀(틱 머리·채택 전·ARM 대기·첫 바이트 모두 같은 함수); `snapshot` 우회는 **보정 세대 번호만 다르고 보정 값은 같은** 설치 대기만 건너뛴다(준비 판정도 설치·적용 보정 값의 일치는 늘 요구하고 세대 번호 검사만 건너뜀, 6 차 P2-3) — 층의 `installing(ignore_correction=True)` 는 설치 결과가 지금 요청과 **같은 보정 값**으로 만들어졌고 보정 세대를 뺀 나머지 열쇠(면제 일련번호·미션·경로·적재)도 같으면 거짓이다(3 차 P1: 값이 다르면 옛 보정의 격자와 새 보정의 차 자세가 섞여 정지할 곳에서 속도를 냄 — 그때는 그대로 대기). 그리고 ARM 대기·첫 바이트·`limit()` 이 같은 판정을 쓴다(2 차 P2-3: 경로·면제가 바뀐 대기는 그대로); `watch_current`·`watch_wheel` 은 `SafetyWatch.step(..., skip=)`(기본 빈 집합 — 텔레옵 브리지 불변)이 그 판정을 하지 않아 다른 쪽 판정의 지속을 지우지 않음.

| 검사 이름 | 우회 시 건너뛰는 판정 |
|---|---|
| `imu` | `ImuWatch` 전체 — ARM 전 기준(`imu_no_baseline`·`imu_not_still`·`imu_baseline_unsteady`·`imu_not_level`, 수평 참조 미측정 포함)과 무장 중 `watch_imu_stale`·`watch_imu_gap`·`watch_imu_moved`·`watch_tilt`(4 절, 5b 소급) |
| `layer_timing` | 층 마감 `layer_overdue`·`layer_latency_exceeded`, 스캔 신선도 `snapshot:scan_stale`·`layer_ok` 의 스캔 나이(작업자 생존은 아님) |
| `measurement` | 준비 판정 `measurement:*`(조향·승강·승강 원점·카운트) |
| `odometry` | 준비 판정 `odometry:cut` |
| `slam` | 준비 판정 `slam:*`(모드·신선도·hold) |
| `snapshot` | 준비 판정 `wait:snapshot_installing` 의 보정 세대 일치 |
| `operator_start` | 제출의 `operator_start:unconfirmed` |
| `watch_current` / `watch_wheel` | 안전 감시의 전류·바퀴 판정(4c) |

**우회할 수 없는 것**(멈추는 쪽만):

| 항목 | 이유 |
|---|---|
| STOP(콘솔·장치), 콘솔 DISARM | 멈추는 명령 자체 |
| lease 상실·만료 정지, 연결 상실 | 운용자가 없는 차를 움직이지 않음 |
| 래치 해소 절차(재무장) | 우회도 걸린 래치를 풀지 않음 — 운용자의 명시적 재무장 |
| MCU 링크 끊김·STATE 신선도(`measurement_stale`·`link:*`)·출력 정체(`link_stuck`)·첫 바이트 읽기 예산 | MCU 와 통하지 않으면 명령이 의미 없음 |
| 층 작업자 죽음(`layer_worker_dead`)·`obstacle_layer_off` | 층을 쓰지 않으려면 `obstacle_act` 를 끄는 별도 결정(조립)이 필요 |
| 과속(`overspeed`)·느린 루프(`loop_slow`)·오도메트리 단절의 무장 중 ① | 지금 차가 통제 밖 |

**실차 상태 발행**(4c 루프는 아직 상태 메시지를 내지 않는다): 루프가 `link.state_due(now)` 마다 채널에 `{"t": "real_status", "session_epoch", "control_gen", "lease": {"holder": 클라이언트 | null}, ...}` 를 낸다(보유자가 없으면 `null` 을 늘 실음, 3 차 P3) — 허가 상태·사유, 미션 상태·ID, 열린 운용자 확인(`kind`·`serial`·`phase`), 열린 현장 복구 ID, 열린 재위치 세대, 운용자 시작 확인 유무, `targets_revision`, 마지막 거절(요청 종류·사유 — 루프의 거절과 중재기의 세대·토큰 거절 모두, 5 차 P3-2), `bypassed`(우회 중인 검사). `safety/state.py`(텔레옵·Isaac 의 `state`)는 바꾸지 않는다. **콘솔**(1 차 P2-8; 범위 총괄 승인 2026-11-19): 5c-1 은 `console/web/js/protocol.js` 가 `real_status` 를 받아 같은 세션이면 제어 세대와 **lease 보유자**를 갱신하고 마지막 값을 두며, 화면은 `bypassed` 가 비어 있지 않으면 '우회 중: <검사…>' 띠를 늘 보이는 데까지다(표시 문자열은 순수 함수 모듈 `realstatus.js`, Node 시험). 실차 조작 화면(팔레트 등록 뒤 `targets` 갱신과 그 revision 으로 제출, `real_status` 만 받는 실차의 제출 가능 판정, 시작 확인·복구 완료 단추, 우회 패널, 운용자 확인 단추를 화면에 표시한 `(session_epoch, kind, serial)` 에 묶기, lease 반납 → 같은 연결 재획득)은 **5c-1b**(2 차 P2-5·P2-6·P2-7).

**통합 시험**(4c 가짜 트럭 확장, `operator_start_ok`·가짜 운용자 훅 주입 대신 실제 요청): lease → `pallet_register` → `operator_start` → 제출 → 채택·무장 → AUTO → 운반 높이 → `real_status` 의 확인 요청 → `operator_answer` → S10 정지; 각 거절 사유; IMU 기준이 없는(수평 아님) 차가 `check_bypass{imu}` 뒤 출발해 S10 까지, `real_status.bypassed` 표시, 새 세션에서 우회가 사라짐; 우회할 수 없는 이름의 거절; `real_requests=False` 에서 새 이름은 응답 없음·`recovery` 는 `unsupported`(중재기 기존 시험 불변).

STOP·DISARM·센서 무효(스캔·IMU·SLAM) 뒤 금지 명령 0 건과 lease 만료 뒤 재무장은 4c·5a·5b 시험이 이미 덮는다. 재위치 확인(`relocalize_confirm`)과 상태의 재위치 세부는 5c-2.

### 5.1 실차 콘솔 조작 화면 (5c-1b)

> **5c-1b 수렴**(13판, Codex 13 회차 새 P1/P2 없음 — 현행 Chrome 범위): 이 절만 Codex 검토로 수렴시켰다(총괄 2026-11-19: 5c-1 의 브라우저 지적 2 차 P2-5·P2-6·P2-7 을 여기서 닫고, 5c-2 보다 먼저 — 현장에서 운용자가 우회를 직접 누를 수 있어야 함). 머지 동결(11-20 보드 확인)은 5c-1 과 같다.

**지원 브라우저는 현행 Chrome(터치 포함)** 이다(운용자 단말은 노트북 Chrome, 사용자 결정 2026-11-16; 범위 총괄 2026-11-19). 클릭에 `pointerId` 가 없는 브라우저(Safari 18.2 이전 등)의 다중 포인터 경계는 **지원하지 않으며 수렴 대상에서 뺀다** — 최종 판정(제어 세대·lease·세션)은 서버가 하므로 안전 경계는 서버에 있다. 그런 브라우저에서는 첫 포인터 클릭 때 패널 위에 '지원하지 않는 브라우저' 경고 띠를 띄운다(판정 로직은 안전한 쪽으로 거절하는 지금 그대로).

`real_status` 를 받은 뒤에만 보이는 **실차 패널**(자율 실행기만 낸다 — 텔레옵 브리지도 `backend: real` 이지만 `real_status` 가 없음). 페이지 위층에 고정 위치로 두어 어느 화면에서도 보이되, 데스크톱 상단 STOP 막대와 폰 하단 STOP 막대 **둘 다보다 아래 층**이고 폰 폭에서는 하단 STOP 막대 위에 놓인다(STOP 을 가리지 않음, 2–3 차 P1). 배선은 `console/web/js/realpanel_dom.js`, 판단은 `realpanel.js` 의 순수 함수(Node 시험). 기존 마법사(가짜 차량·Isaac 의 `state` 기반)는 바꾸지 않는다.

- **이 콘솔의 상태일 때만**(1 차 P2-2): 단추는 `real_status` 가 **이 세션**의 것이고 **이 클라이언트를 lease 보유자**로 적었으며 **지금 lease 를 가진 동안 받은** 것일 때만 열린다(`protocol.js` 가 받는 순간의 lease 를 `realStatusLease` 로 기록; 2 차 P2-2: 같은 클라이언트가 lease 를 되찾은 직후의 옛 상태로는 안 열림). 모든 단추는 **누르기 시작한 때 그려져 있던 것**(단추·입력마다 기록, 취소된 포인터·거절된 클릭은 기록을 버림, 반복 Enter 는 어느 단추에서도 무시, 클릭은 자기 누름에 **한 번만** 묶고 포인터 클릭은 자기 누름 기록이 있어야 함 — 지금 그림으로 대신하지 않음; 포인터 id 없는 click(옛 Safari)은 그 단추의 포인터 누름이 **하나일 때만** 거기에 묶고, 모호하면 거절하고 그 단추를 **다른 포인터가 모두 떨어진 뒤의 새 누름**까지 막음; 단추 밖에서 뗀·취소된 포인터의 누름 기록은 지움 — 브라우저가 탭으로 보고 그 단추에 click 을 주는 경우(Chrome 의 터치 탭 허용 거리 안)는 의도한 누름으로 본다: 누름 때 그려진 세션·lease·제어 세대가 지금과 같아야 하는 검사는 그대로 — 5–12 차)과 마지막으로 그린 때의 세션·lease 와 **클릭이 기대는 상태 내용**(제어 세대·보유자·허가·미션·모드·열린 확인 요청·복구·우회 집합·revision 의 서명 — 제어 세대는 두 번째 STOP 처럼 모드가 같아도 바뀜, 4 차 P2-1)이 그대로인지 본다(3 차 P2-1: 같은 내용의 새 상태는 클릭을 막지 않음).
- **팔레트 등록과 작업 지시**(2 차 P2-5, 1 차 P2-4·P2-5): 목록(`targets`)이 이 세션에 아직 없으면 패널이 `targets_request` 를 한 번에 하나 보내고, 거절되면 이유를 보이고 다시 요청한다(2 차 P2-5). 팔레트를 고르고 지도 자세 `x, y, yaw`(m, m, °)를 넣어 `pallet_register` → ack 의 `targets_revision` 을 **세션에 묶어** 보관하고 목록을 새로 받는다. '작업 지시'는 **이 세션의 등록 revision** 으로만, 그리고 입력란이 **등록한 팔레트·자세 그대로**일 때만 보낸다(7 차 P2-1: 등록 뒤 고친 자세가 옛 등록으로 나가지 않음; ack 를 기다리는 사이 입력을 바꿨으면 ack 때 바로 재등록 안내, 8 차 P3) — `real_status.targets_revision` 이나 받은 `targets` 의 revision 이 그보다 앞서거나(2 차 P2-4) `stale_targets` 를 받거나 세션이 바뀌면 다시 등록해야 한다.
- **제출 가능**(1 차 P2-6·P2-3): 허가 OFF·진행 중 미션 없음·lease, 그리고 **래치(`LATCHED`)나 중재기 `STOPPED` 가 아님**(재무장이 먼저 — 그때는 패널에 '재무장' 단추; `real_status.mode` 는 중재기 모드), 그리고 **보낸 우회 전환이 `real_status.bypassed` 에 보이기 전이 아님** — 전환은 한 번에 하나(세션·lease 에 묶음 — 바뀌면 그 안내도 지우고 늦게 온 그 응답도 보이지 않음, 3–5 차 P3), **보내는 순간부터** 제출·시작 확인을 막고, ack 뒤 2 s 안에 보이면 '적용됐습니다', 안 보이면 '적용되지 않았습니다' 를 보이고 푼다(거절되면 바로 풂, 2 차 P2-3·P3). 시작 확인 단추도 같은 조건.
- **운용자 확인 답**(2 차 P2-7, 1 차 P2-1): 누르기 **시작한** 순간(포인터 누름·처음 Enter — 반복 키는 무시, Space 는 텔레옵 데드맨이라 쓰지 않음, 2 차 P2-1·P3; 단추마다·입력(포인터 id·키보드)마다 따로 기록하고 클릭은 자기 입력의 기록만 씀, 3–4 차 P2-2)의 `(session_epoch, kind, serial)` 을 잡고, 떼는 순간 최신 `real_status` 의 열쇠와 같을 때만 보낸다(그사이 #7→#8 이면 보내지 않음).
- **현장 복구 완료**: `real_status.field_recovery` 가 있을 때만 단추. 보낼 때 최신 상태의 번호가 그린 번호와 같고 그린 때의 세션·lease 가 같아야 한다(1 차 P2-2).
- **검사 우회**: 5 절의 우회 가능한 검사마다 켬/우회 단추 → `check_bypass{check, on}`. 허가가 OFF·LATCHED 가 아니거나 진행 중 미션이 있거나 lease 가 없으면 단추를 막고 이유를 보인다(이유가 바뀌면 다시 그림, 1 차 P3). 지금 우회 상태는 `real_status.bypassed` 만 따른다.
- **연결 끊김**(12–13 차 P3): 끊겼거나 다시 연결된 뒤 이 lease 의 새 `real_status` 가 아직 없으면 '연결 끊김·새 상태 대기 · 마지막 수신 값' 으로 표시하고 흐리게 그린다(단추는 닫힘).
- **lease**(2 차 P2-6): 5c-1 의 `protocol.js` 가 `real_status.lease.holder` 로 보유를 맞추므로 반납·만료 뒤 같은 연결에서 다시 `lease_request` 를 보낸다(Node 시험: 반납 → 같은 연결 재획득).
- 거절은 `rejectText` 로 한국어 사유(`not_ready:` 접두 포함, 1 차 P3).

**시험**: Node 로 `realpanel.js` — 등록 revision 의 세션 묶음·앞선 목록에서 재등록, 이 콘솔의 상태 판정, 제출 가능(래치·STOPPED·전환 대기), 전환 표시 확인, 답 열쇠, 우회 단추, 자세 변환, 우회 목록이 루프와 같음; `protocol.js` 반납 → 같은 연결 재획득. 화면 배선은 DOM 함수 단위(학교 규모).

## 6. 재위치 배선 (5c-2)

> **수렴**(5c-2, 14판 — Codex 14 회차 새 P1/P2 없음; Codex 1 회차 P1 2·P2 7, 2 회차 P1 3·P2 6, 3 회차 P1 2·P2 5, 4 회차 P1 1·P2 4, 5 회차 P1 1·P2 2, 6 회차 P1 1·P2 2, 7 회차 P2 2, 8 회차 P2 4, 9 회차 P2 3, 10 회차 P1 1·P2 3, 11 회차 P2 1, 12 회차 P2 2, 13 회차 P2 1 반영; 총괄의 운용자 선언 우회 7b 포함). 예전 판(7 회차 지적 반영, 미수렴)을 5c-1 이 실제로 만든 요청 경로에 맞춰 줄였다: 보류 원장(`PENDING`·`settle_pending`) 대신 **즉시 답 + `real_status` 로 판정 결과**. 확인과 선언은 같은 **시도** 틀을 쓰고 자세의 출처만 다르다(3판). 실 SLAM 링크는 PR 7 이고, 이 단계는 가짜 원천 계약으로 배선과 판정만 시험한다(총괄 2026-11-19).

재위치 세대는 이미 코어에서 열린다(바꾸지 않음): 현장 복구 완료(`field_recovery_done`), MCU 재부팅·같은 부팅의 오도메트리 끊김·hold 한도 초과(`require_relocalization`). 세대가 열려 있는 동안 제출은 코어의 `relocalize_required` 로 거절되고(`core_refusal`, 우회 불가 — 5 절 목록의 `core`), **실차 텔레옵도 주행·승강 모두 닫힌다**(`real_teleop` 의 `relocalize_required`, `test_core_field_recovery` 가 고정 — 1 회차 P1-2). 그래서 운용자는 차를 손으로 바닥 표시 시작 자세에 옮긴다(현장 복구 절차의 모터 전원 끔 상태 그대로). 세대를 닫는 길은 시도(5)의 두 종류 — 확인과 운용자 선언(7b) — 이다(둘 다 `core.relocalized(t, gen)`).

**프레임**(2 회차 P1-2): `start_rear` 는 지도의 후륜축 자세, SLAM 보정은 `map_from_odom`, 오도메트리는 `odom_from_base`(`base_link`)다. 자세 비교는 `rear_from_base(compose(보정, odom_from_base), rear_offset_m)` 와 `start_rear` 로, 선언의 목표 `base_link` 자세는 `base_from_rear(start_rear, rear_offset_m)`(`start_reloc.rear_from_base` 의 역, 새 함수)로 한다. `odom_from_base` 는 `hooks.slam_odom_base()`, `rear_offset_m` 은 차체 모델의 후륜축 오프셋(실 SLAM 링크와 같은 값).

1. **세대를 루프에 맞춤** `_sync_relocalization(now_ns, t)`: `core.relocalize_generation()` 이 None 이 아니고 루프가 연 세대와 다르면 그 세대를 연다(2). 부르는 자리(1 회차 P2-1): 콘솔 묶음의 종료 수거 뒤, `_odometry_head` 뒤, 재위치 요청 처리의 첫머리, 준비 단계(계획 요청·채택·ARM 판단) 직전, 제어 단계 끝(hold 한도 초과가 세대를 연 경우). 한 번에 세대가 둘 올라도 마지막 것 하나만 연다. 확인·선언 요청은 이 맞춤 뒤 **루프가 연 세대 = 코어 세대이고 추적기 초기화(2)가 끝났을 때만** 받는다.
2. **옛 증거 폐기**(세대를 열 때 한 번):
   - 루프: 진행 중 시도(`refused`, `relocalize_superseded`), 정지 시작(`still_since_ns`·3 의 `stand_since_ns`), 운용자 시작 확인(`operator_start.void("relocalize")`), 적용 보정 출처(4). D5 프레임은 5d 에서 더한다.
   - 층: `_reset_layer(now_ns)`(오도메트리 끊김과 같은 것: 새 격자, 설치·대기 결과 없음, 미배치 스캔과 오도메트리 이력 비움, `exempt_serial` 증가로 작업자에게 이미 넘어간 옛 결과도 키에서 걸러짐). 차를 손으로 옮기는 동안 쌓인 스캔은 시도를 받을 때 다시 자른다(5).
   - **재위치 리셋과 층 감시**(3 회차 P2-1·P2-2, 4 회차 P2-1·P2-2로 줄임): 모든 격자 리셋(`_reset_layer`, 5a 의 끊김 리셋 포함)은 `_placed_id` 도 비운다 — 리셋 뒤 빈 작업이 새 `exempt_serial` 과 옛 스캔 id 를 함께 달고 설치돼 옛 마감을 해소하는 일이 없게(`_placed_id` 가 없으면 작업의 `scan_id` 는 0). 마감·`_missed_expected`·`_overdue_since`·`_first_use_late`·원천 단절 같은 감시 상태는 **바꾸지 않는다**. 리셋 뒤 첫 결과까지는 대략 0.3 s(배치에 필요한 이력 `2·주기 + 지연` 과 계산)라 그동안 `layer_overdue`(바퀴·승강 정지 유지 — 허가 OFF·LATCHED 인 재위치 중에는 바뀌는 것 없음)가 나고, `overdue_end`(1.0 s) 안에 스캔이 든 결과가 설치돼 풀린다. 1 s 안에 시도를 거듭 보내 고장 한도를 넘기면 그것은 기존 처리 그대로 `layer_latency_exceeded`(래치, 재무장 절차)이고 그 시도는 무효(5)다.
   - SLAM 원천: `slam_source.relocalize(epoch)` — **원천 회차** `epoch` 는 루프가 원천에 `relocalize`·`seed` 를 부를 때마다 하나씩 올리는 정수다(코어 세대와 별개 — 2 회차 P2-2: 한 세대 안의 선언이 원천을 다시 시작해도 구별되게). 루프는 받아들이는 회차를 그것으로, 회차 시작 시각 `t_epoch` 를 지금 코어 시각으로 둔다(올리기만 함).
   - 추적기·코어(**미션이 끝난 뒤**: 미션 상태가 `idle` 이나 종료 상태가 될 때까지 미룸 — 그 사이 제출은 이미 거절된다): `tracker.reset_for_relocalization()`(새 메서드, 실차만 부름: hold 를 버리고 `warming_up` 으로, `applied`·`received` 는 남긴다 — 제어 자세 계산과 응답 시각 단조 검사가 끊기지 않게. 그 뒤의 `processed` 는 scan id 와 무관하게 `applied` 가 되지만 **세대가 닫힐 때까지 `warming_up` 에 머문다** — `warming_up` 은 나이를 보지 않으므로 세대가 열린 동안 `hooks.slam_rear` 의 나이 예외가 없고(6 회차 P2-1: 층을 기다리는 동안 응답이 끊겨도 시도 판정이 `relocalize_stale`·시간 초과로 끝까지 감), 출력은 이미 닫혀 있다. 세대를 닫을 때 `tracker.relocalization_done()`(새 메서드)으로 `tracking` 에 들어가고, 그 뒤 나이는 보통대로 본다), `core.relocalization_reset(t)`(새 메서드: `in_hold`·`put_downs_in_hold`·`_last_pose_hold`·`end_release_due` 를 지우고 기록). 둘 다 같은 틱에, 같이. 남은 옛 `applied` 는 출처가 비어 있으므로(4) 시도 조건을 채우지 못한다.
   - 세대가 열려 있는 동안 `operator_start` 요청은 `relocalize_required` 로 거절하고, 세대를 닫을 때도 한 번 더 무효로 한다(2 회차 P2-5).
2b. **출력이 닫힌 동안의 자세 읽기**(7 회차 P2-1·P2-2, 8 회차 P2-1~4 로 고침): 지금 루프는 틱 안의 어떤 예외도 비상 경로(래치·STOP)로 보내고, 추적기의 `map_from_base` 는 적용 보정이 없거나 `tracking` 의 나이가 지나면 `LocalizationStale` 을 낸다. 그래서 첫 보정 전에 재부팅으로 열린 세대나 선언 뒤 원천이 응답하지 않는 경우, 출력이 이미 닫힌 차가 래치된다. 8판의 '자세 없음 틱'(틱 일부를 건너뜀)은 층 작업 제출·콘솔 정산·hold 한도 경로를 함께 건너뛰는 문제가 있어 철회하고, **예외가 나지 않게 읽는다**:
   - 실차 `hooks.slam_rear`·`slam_odom_base` 배선 계약(PR 7 이 붙임, 5c-2 의 가짜 훅은 실제 추적기로 이를 지킴): 루프가 매 틱 넘기는 `outputs_closed`(허가 OFF·LATCHED 이고 미션 상태가 `idle`·종료)가 참이면 **나이 비교와 hold 이동 한도의 예외를 건너뛰고**(이동량 누적은 그대로) 마지막 적용 보정으로 자세를 내며, 적용 보정이 아직 없으면 항등 보정(지도 = 오도메트리)으로 낸다. hold 이동 한도 초과는 예외 대신 루프 검사가 맡는다 — 자세를 읽은 뒤 `release_gate` 전에, 출력이 닫혀 있고 SLAM 이 `holding` 이며 `hold_travel_m > HOLD_MAX_M` 이면 **미뤄 둔 해제가 있든 없든**(10 회차 P2-4: 화물 상태가 해제를 막아 예약이 없는 hold 도) `require_relocalization("hold_too_long")`(9 회차 P2-2: 예외로 빠지면 그 검사에 닿지 못함). 지금의 예약 있는 경우 검사는 그대로 둔다. `outputs_closed` 가 거짓이면 지금 그대로(나이 초과 = 예외 = 비상).
   - 그 밖에는 틱을 건너뛰지 않는다 — 층 작업 제출·감시, 콘솔 정산(`tick_end`), 시도 판정이 모두 보통대로 돈다(8 회차 P2-1·P2-4). 출력이 닫혀 있으므로 그 자세는 층 투영·표시에만 쓰이고, 세대를 닫는 시도는 지금 회차의 출처(4)를 요구하므로 항등·낡은 보정으로는 닫히지 않는다.
   - 제출은 틱 머리에서 받아들여져 같은 틱의 제어 단계부터 `outputs_closed` 가 거짓이 되므로, 실제로 낡은 자세에서 받은 제출은 그 틱의 자세 읽기에서 지금처럼 비상(래치)이 된다. 우회하지 않으면 준비 판정의 `slam:stale`(실제 `processed` 로만 갱신되는 4 의 신선도 증거)이 그 제출을 먼저 막는다. '텔레옵만' 은 우회하지 않은 경우의 계약이다(8 회차 P2-3).
   - **`slam` 우회의 뜻을 적어 둔다**(10 회차 P1): `slam` 을 우회하면(제출 때는 `operator_start` 우회도 함께 있어야 정지 판정을 지남) 차는 **마지막 적용 보정으로 계획하고 달릴 수 있다** — 원천이 `skipped` 만 내 추적기의 `received` 시각이 갱신되는 경우도 같다. 이것이 운용자가 SLAM 검사를 내려놓는다는 뜻이고(5 절, 사용자 원칙), 그때 남는 것은 우회 불가 목록(멈추는 쪽)과 선언 우회의 0.06 m/s 상한이다. 원천이 완전히 끊겨 추적기 나이 예외가 나면 그 틱은 지금처럼 비상(래치)이다. **총괄 결정(2026-11-19, 10 회차 P1 의 처리)**: 이 뜻 그대로 가고 좁히지 않는다 — `slam` 우회는 '마지막 적용 보정 + 오도메트리로 달려도 된다'이다(사용자 원칙). 덧붙여 **`slam` 이 우회 집합에 있는 동안 출력 속도 상한을 `declared_speed_mps`(0.06 m/s, 잠정 — S10 속도 승인 목록)로 내리고**(7b 와 같은 자리, 둘 다면 같은 값), 콘솔의 우회 띠에 '우회 중: slam — 오도메트리 주행'으로 보인다(5d 의 D5 우회 저속과 같은 방식). 원천이 완전히 끊겨 추적기 나이 예외가 나면 지금처럼 래치다. **출발과 계속의 경계**(11 회차 P2): 계획 채택과 ARM 전 정지는 SLAM 신선도가 든 `stop_now` 를 그대로 요구한다(hold 해제와 같은 값이라 나누지 않음). 그래서 `slam` 우회 중에도 **출발에는 지금 회차의 실제 `processed` 가 한 번은 신선해야** 하고, 그 뒤 무장해 달리는 동안 `processed` 가 끊기고 `skipped` 만 와도 마지막 적용 보정과 오도메트리로 계속 달린다(0.06 m/s 상한). 다만 미션 안의 다른 **정지 조건은 그대로** `stop_now` 를 요구한다 — 포켓 첫 프레임을 받기 전의 정지와 삽입 끝(승강으로 넘어가기 전)이 그렇다(12 회차 P2-2). 실제 `processed` 가 계속 없으면 차는 포켓 앞에서 서서 그 단계의 시간 초과로 끝나고, 삽입 끝에서 승강으로 넘어가지 않는다. 계속 주행은 그 정지 사이의 주행 구간에만 해당한다. 5c-2 는 이것을 시험으로 고정한다: `skipped` 만 오는 원천 + 두 우회 → 제출·계획 요청·아비터 AUTO 전이까지 열리고 ARM·비영 DRIVE 는 0 건; 무장 뒤 `processed` 가 끊기고 `skipped` 가 이어지는 동안 실제로 쓰인 비영 DRIVE 가 0.06 m/s 를 넘지 않음; 포켓 정지 전 주행 구간에서 마지막 `processed` 가 만료된 뒤 비영 DRIVE 가 실제로 한 건 이상 쓰였고 모두 0.06 m/s 이하이며, 그 가운데 적어도 한 건은 `_auto_drive()` 에 들어온 후륜축 명령이 0.06 m/s 를 넘었던 것(13 회차 P2: 상한 코드가 빠지면 잡히게); 그 뒤 포켓 앞 정지에서 시간 초과; 우회 하나만이면 제출이 열리지 않음; 원천 완전 끊김은 래치. **상한 규칙은 하나다**(12 회차 P2-1): `slam` 이 우회 집합에 있거나 그 세션에 선언 표식(7b)이 있으면 0.06 m/s. 그래서 `slam` 우회를 끄면 선언 이력이 없는 세션에서는 상한이 풀리고, 선언 이력이 있는 세션에서는 남는다(둘 다 시험).
   - **아비터의 정지 증거**(9 회차 P2-1): 아비터의 `stop_evidence`(텔레옵 진입·재무장의 `not_stopped`, `PAUSED` 의 `not_stopped_s` 뒤 STOP)는 SLAM 신선도가 든 `stop_now` 가 아니라 **물리적 정지** — 링크 신선과 3 의 MCU 표본 정지 — 를 쓴다. 루프가 `physical_stop(now)` 를 내고 실차 조립(PR 7)·rig 가 `arbiter.stop_evidence` 를 그것에 묶는다(지금 rig 는 `stop_now` 에 묶음). 재무장의 `AutoPermit.rearm_refusal()` 에 넘기는 정지 시작도 `still_since_ns` 가 아니라 3 의 `stand_since_ns` 와 그때의 MCU 부팅 id 로 바꾼다(10 회차 P2-2: 같은 부팅에서 앞 명령이 해소된 뒤 확인한 정지라는 기존 조건은 그대로). `stop_now`(SLAM 신선도 포함)는 계획 채택·ARM 전 정지·hold 해제에 그대로 쓴다. 그래야 SLAM 이 끊긴 서 있는 차가 텔레옵·재무장을 거절당하거나 `PAUSED` 에서 STOP 으로 가지 않는다.
   - `RealLoop` 은 생성 때 코어 추적기가 `SlamPoseTracker` 이면 그 `max_age_s` 가 `SLAM_MAX_AGE_S`(0.5 s)와 같은지 보고 다르면 `ValueError` 로 거절한다(신선도 증거와 추적기 나이 한도가 같은 값; 추적기를 만드는 실차 조립은 아직 없음 — PR 7).
3. **정지 판단**(1 회차 P2-3): 시도는 `_still()` 이 아니라 그 안의 **MCU 표본 정지**만 쓴다 — 명령 해소, 그 뒤 표본의 정지·승강 변화 없음, 링크 신선, 속도 판독 유효, 오도메트리 연속. SLAM 신선도는 빼고(새 응답을 요청하는 쪽이 그것을 요구하면 정지한 차가 확인을 못 함) 결과 판정에서 따로 읽는다. 루프는 `_still()` 에서 이 부분의 시작 시각을 `stand_since_ns` 로 따로 둔다. `_still()` 자체(SLAM 신선도 포함)는 바꾸지 않는다 — 그것이 내는 `stop_now` 는 운용자 시작 확인·계획 채택·ARM 전 정지뿐 아니라 **미뤄 둔 hold 해제**(`release_gate`)에도 쓰이므로, 신선도를 우회하면 만료된 `received` 가 `applied` 로 옮겨질 수 있다(4 회차 P1, 3판의 `slam` 항 우회를 철회). 이 절은 `slam` 우회의 범위를 넓히지도 좁히지도 않는다(5 절 그대로: 준비 판정의 모드·나이). 우회하지 않으면 송신 관문의 준비 판정이 프레임마다 SLAM 나이를 보므로 ARM 뒤 첫 DRIVE 전에 만료된 응답으로는 출발하지 않고, 우회하면 그 나이를 건너뛰는 것이 운용자가 고른 뜻이다(5 회차 P1: 4판의 '자율 출발에는 신선한 `processed` 가 늘 필요' 는 우회 중에는 맞지 않는 과장이라 고침). 운용자 시작 확인이 MCU 표본으로는 서 있는데 SLAM 이 낡아 거절되면 사유를 `not_still` 이 아니라 `slam_stale` 로 낸다. 콘솔은 이 MCU 표본 정지를 `real_status.standing`(불리언, 서버의 `not_still` 판정과 같은 값)으로 본다(3 회차 P2-5).
4. **응답 거르기와 출처**(1 회차 P2-2): 루프는 원천 응답을 `core.slam_replies()` 에 넘기기 전에 거른다 — `epoch`(없으면 0)가 받아들이는 회차와 다름, `stamp_s < t_epoch`, `stamp_s` 가 지금보다 뒤(미래), 앞서 넘긴 응답보다 `stamp_s` 가 앞(역순 — 추적기가 예외를 내므로), 같은 회차에서 scan id 가 앞서 넘긴 것보다 크지 않음(같은 스캔 재표기), 추적기에 아직 `received` 가 없는데 `skipped`(9 회차 P2-3: 실제 추적기는 첫 `processed` 전의 `skipped` 에 `ValueError` 를 냄) 중 하나면 넘기지 않고 사유별로 센다(`relocalize_dropped`). **적용 보정 출처**는 각 `core.slam_replies()` 호출 뒤 추적기의 `applied` 가 실제로 바뀐 경우에만 그것을 만든 응답의 `(회차, 시도, scan id, stamp_s)` 로 기록한다(`holding` 중의 `processed` 는 `received` 만 바꾸므로 출처가 아님; 한 묶음 안에서는 마지막으로 적용된 응답). `skipped` 는 출처를 바꾸지 않는다. 세대를 열 때 출처를 비운다(5 회차 P2-2 의 나이 예외는 2 의 `warming_up` 유지로 막는다 — 6판의 낡은 응답 거르기는 철회).
   - **SLAM 신선도 증거**(6 회차 P1): `_still()`·`_ready()` 가 읽는 `SlamView.pose_reply_ns` 는 실차 루프에서 **루프가 거른 응답으로 만든다** — 지금 회차의 실제 `processed` 가운데 넘긴 가장 새 것의 `stamp_s`(첫 빔 취득)를 호스트 단조 시각으로 옮긴 값. 회차를 바꿀 때(`relocalize`·`seed`) 비우고, 합성 응답은 넣지 않는다. 그래서 선언 직전의 옛 회차 응답이 아무리 신선해도 새 회차의 실제 `processed` 가 오기 전에는 운용자 시작 확인·ARM·첫 DRIVE 가 열리지 않는다(`slam` 우회 중의 준비 판정은 5 절 그대로 나이를 건너뛰지만, 3 의 정지 판정이 신선도를 요구한다). PR 7 의 `Evidence.slam` 배선은 이 값을 쓴다.
5. **시도** — `relocalize_confirm{generation}`(확인)과 `relocalize_declare{generation}`(선언, 7b). 둘 다 5 절의 `REAL_LEASED` 에 더하고 필드 검사는 `generation` 정수·불리언 아님. 운용자가 차를 시작 자세에 세우고 콘솔에 정지가 보인 뒤 보낸다. 루프는 1 의 맞춤 뒤 **즉시** 답한다.
   - 거절: 열린 세대 없음 `no_relocalization`; 다른 세대 `stale_relocalization`; 추적기 초기화가 아직 미뤄짐 `mission_active`; 허가가 OFF·LATCHED 아님 `armed`; MCU 표본 정지 아님 `not_still`; 시작 자세나 후륜축 오프셋 없음 `no_start_pose`(아래 끝).
   - 받음: 확인·선언이 같은 번호 줄의 새 시도 번호를 받고(앞 시도는 종류와 무관하게 `relocalize_superseded`, 2 회차 P2-4), 시도에 **종류, 접수 때의 lease id·제어 세대·운용자가 켜고 끄는 우회 집합(`check_bypass` 의 것 — 7b 의 선언 표식은 따로 두므로 선언이 스스로 시도를 무효로 하지 않음, 4 회차 P2-4)**, `stand_since_ns`, 접수 시각 `t_try` 를 묶는다. 그리고 **층과 오도메트리 이력을 다시 자른다**(`_reset_layer`, 2 회차 P1-1: 손으로 옮기기 전후의 스캔이 같은 원시 자세로 배치되는 것을 막음 — 바퀴 오도메트리는 들어 옮긴 거리를 모른다). 확인이면 원천에 `request_processed(epoch, attempt, not_before=t_try)`. 선언이면 원천에는 아직 아무것도 부르지 않는다(아래 첫 판정 틱). 답은 두 종류 모두 ack `{generation, attempt}`.
   - **판정**(시도가 기다리는 동안 매 틱, 제어 단계에서 원천 응답을 넘기기 **전** 한 번과 층 설치 **뒤** 한 번):
     - 무효(먼저): 이 틱 층 감시가 고장(`LAYER_ENDS`)을 냈음(`relocalize_voided:layer`, 3 회차 P2-2: 고장 판정이 잠금 해제보다 먼저), lease 가 바뀌었거나 만료, 제어 세대가 바뀜(STOP·DISARM·연결 상실이 올림), 허가가 OFF·LATCHED 를 벗어남, 우회 집합이 바뀜(1 회차 P2-5), `stand_since_ns` 가 바뀜이면 `refused`(`relocalize_voided:<lease|control|armed|bypass>` 또는 `relocalize_moved`). 콘솔 DISARM 이 그 틱의 제어 단계를 건너뛰었어도 다음 틱의 이 판정에서 같은 규칙으로 끝난다(2 회차 P2-1).
     - **선언의 자세 적용**(선언 시도의 첫 판정, 원천 응답을 넘기기 전, 무효가 아닐 때만 — 2 회차 P2-1: 무효면 원천에 `seed` 가 남지 않음): 목표 `b = base_from_rear(start_rear, rear_offset_m)`, 보정 `c = b ∘ (지금 odom_from_base)⁻¹`. 원천에 `seed(epoch+1, b, stamp_s=지금)` 를 부르고 받아들이는 회차·`t_epoch` 를 올린 뒤, `c` 를 **합성 `processed` 응답**(scan id −1, 출처 `operator_declared`, 거르기 밖) **하나만으로** `core.slam_replies()` 를 먼저 부른다 — 추적기는 그것을 `applied` 로 받고(세대가 닫힐 때까지 `warming_up` 에 머묾 — 2), 코어는 값이 바뀌었으면 격자를 다시 투영한다. 출처는 4 의 규칙대로 기록된다. 그다음 같은 틱의 원천 응답을 **따로** 넘긴다(새 회차 응답은 아직 없으므로 모두 걸러짐 — 2 회차 P2-2: 적용 순서 고정).
     - 통과(층 설치 뒤, 모두 참이면 `core.relocalized(t, gen)` 를 부르고 확인은 `confirmed`, 선언은 `declared`):
       - 세대가 그대로이고 시도가 지금 것;
       - 자세의 출처 — 확인: 그 시도 번호를 단 이 회차의 `processed` 가 넘어왔고 `stamp_s ≥ t_try`(`relocalize_no_reply`), 그리고 적용 보정 출처(4)가 이 회차이고 그 `stamp_s` 가 `SLAM_MAX_AGE_S` 안(`relocalize_stale`; 나이 한도만은 `slam` 우회 때 건너뜀 — 1 회차 P2-7). 기준 응답 뒤 다른 `processed` 가 적용돼도 된다. 선언: 적용 보정 출처가 그 선언의 합성 응답이거나 그 뒤 새 회차의 `processed`(나이는 보지 않음 — 선언이 곧 자세의 출처);
       - 자세 비교(프레임 단락의 식, 추적기 `applied` 와 `slam_odom_base()` 로 직접 계산 — `hooks.slam_rear` 의 추적기 나이 예외와 무관, 2 회차 P2-3)가 `start_rear` 에서 위치 0.10 m·각 5°(잠정) 안(`not_at_start`) — **우회 가능**(`start_pose`, 7). 선언 직후는 정의상 0 이다;
       - 층이 정상이고(`real_layer.ok(now)`: 작업자 생존, 원천 연결 — 4 회차 P2-3: 처리된 단절 경보 뒤 재연결 통지 전의 시도도 막음) 층 결과가 지금 보정 세대로 설치됐고(`real_layer.installing(ignore_correction=False)` 거짓) **그 결과의 가장 새 스캔이 `t_try` 뒤에 취득된 것**(`installed_newest_s() ≥ t_try`; 빈 격자 결과는 None 이라 안 됨 — 1 회차 P2-4, 2 회차 P1-1)(`relocalize_layer`). 층이 다시 자른 이력으로 첫 스캔을 배치하려면 첫 빔 전 `2·주기 + 지연` 의 이력이 필요하므로 대략 접수 뒤 0.3 s 부터 결과가 나온다.
     - 접수 뒤 `RELOCALIZE_WAIT_S`(잠정 3 s)가 지나도록 통과하지 못하면 그때 처음 거짓인 조건을 사유로 `refused`. 선언이 `refused` 로 끝나면 이미 적용된 합성 보정은 남지만 세대는 열린 채이므로 제출·텔레옵은 그대로 닫혀 있다.
   - 확인·선언 뒤에도 자동 재개는 없다. 다음 제출은 새 운용자 시작 확인이 필요하다(2).
6. **제출부터 첫 움직임까지 시작 자세 유지**(1 회차 P1-1, 2 회차 P1-3): 미션마다 **그 미션의 첫 비영 DRIVE 가 실제로 쓰일 때까지** 같은 비교를 한다.
   - 무장 전(`_before_arm()` 참: 받음·준비·채택 뒤 ARM 대기) 매 틱 준비 단계 판단 직전에 비교하고, 밖이면 미션을 `not_at_start` 로 끝낸다(래치 없음).
   - 무장 뒤 첫 움직임 전: **송신 관문**(`_transmit` 의 `judge(frame)`, 3 회차 P1-1)에서 그 미션의 비영 속도 DRIVE 를 쓸 때마다 — 첫 바이트 앞이든 반쯤 쓰인 프레임을 이어 쓸 때든 — 비교하고, 밖이면 `("not_at_start", "alarm")` 으로 거절한다. 관문의 기존 규칙대로 첫 바이트 앞의 거절은 경보 → 기존 정지 경로(래치·DISARM, 미션 끝 `not_at_start`), 이어 쓰기의 거절은 `link_stuck`(반쯤 쓰인 프레임을 버리고 링크 재설정)이다. ARM 응답을 기다리는 동안 밀린 차도 여기서 걸린다. `DRIVING` 전이는 큐에 넣는 시점이라 끝 경계로 쓰지 않는다.
   - 그 미션의 비영 DRIVE 하나가 끝까지 쓰여 `book.written(seq)` 를 받으면 검사를 끝낸다.
7. **우회 원칙 적용**: `start_pose` 를 5 절 `BYPASSABLE` 에 더한다(Python·JS 고정 시험 둘 다). 우회 중에는 5 의 자세 조건과 6 의 비교를 건너뛴다 — 바닥 표시와 SLAM 지도가 몇 cm 어긋난 현장에서 운용자가 육안으로 확인하고 진행할 수 있게. 확인 조건을 하나씩 끄는 우회는 이것과 5 의 나이 한도(`slam`)뿐이다: 남은 조건(새 회차의 응답, 정지, 자른 뒤의 층 결과)은 운용자 판단을 거절하는 검사가 아니라 **자세가 존재하기 위한 조건**이다. 확인이 끝내 안 되면 7b 의 선언으로 자세를 운용자가 준다.
7b. **운용자 선언 우회**(총괄 2026-11-19, 사용자 원칙): 운용자가 "차가 바닥 표시 시작 자세에 있다"고 선언해 **그 자세로 SLAM 위치를 시작**한다(현장의 초기 자세 지정과 같음). 고를 수 있는 자세는 `start_rear` 하나뿐이고, 요청·조건·판정은 5 의 시도 그대로다(자세의 출처만 합성 응답).
   - **세션 표식**(우회 집합과 별개의 세션 상태 `declared_epoch`, `real_status.bypassed` 를 낼 때만 합침 — 4 회차 P2-4): 선언의 `seed` 를 원천에 부른 때부터(3 회차 P1-2: 그 시도가 `refused` 로 끝나 뒤의 확인이 그 시드에서 이어진 추정으로 통과해도 상한이 남게) 그 세션(`session_epoch`)이 끝날 때까지 `real_status.bypassed` 에 `relocalize`(콘솔 "우회 중: relocalize(운용자 선언)")가 남고, 출력 속도 상한이 `declared_speed_mps`(0.06 m/s — 잠정, S10 속도 승인 때 `s10a.max_speed_mps` 와 함께 확정(총괄 2026-11-19); `withdraw_speed_mps` 와 같은 값; 지금 상한 `s10a.max_speed_mps` 0.15 보다 낮음)로 내려간다 — 루프 출력의 후륜축 명령 상한(`real_loop` 의 `max_speed_mps` 비교)과 같은 자리에서 둘 중 작은 값. 끌 수 없다(`check_bypass{relocalize}` 는 `not_bypassable`). 사건 기록 `relocalize_seeded{generation, attempt, odom_from_base, start_rear, base_target}` 와 결과(`declared`·`refused`).
   - **신선도**(2 회차 P2-3, 4 회차 P1): 합성 응답은 SLAM 신선도(`Evidence.slam` 의 `pose_reply_ns`)에 넣지 않는다. 선언 뒤 제출·주행의 SLAM 판정은 보통대로다(5 절의 준비 판정과 `slam` 우회, 3 의 정지 판정) — 선언은 원천의 시작 자세를 줄 뿐 그 판정을 바꾸지 않는다. 정지 판정의 신선도는 우회되지 않으므로 원천이 새 회차 `processed` 를 끝내 내지 않으면 자율 출발은 안 되고 텔레옵만 열린다(우회하지 않은 경우; 2b 끝).
   - 선언 뒤의 원천 응답은 보통대로 적용된다 — 선언은 시작점일 뿐이다. 선언 자세가 틀렸다면 SLAM 이 그 틀린 시작에서 맞추거나 어긋나며, 그것은 운용자 판단의 몫이다(우회의 뜻). 6 의 비교는 그대로 걸린다(원천 보정이 크게 옮기면 `not_at_start` — `start_pose` 우회로 끌 수 있음).
   - **F1 차단(#360)과의 관계**(2 회차 P2-6): 실차 경로는 지금 F1 차단(아비터 요청, 미션 명령 `Context.start_blocked`, AUTO 진입 단언 세 경계)을 배선하지 않으므로 겹치지 않는다. PR 7 이 그것을 실차에 붙일 때 세 경계가 **같은 실차 세션의 `declared` 기록**(`session_epoch`, 세대, 시도)을 예외로 읽고, F1 에폭 정책에도 그 예외를 적는다. 그 배선과 문구는 PR 7 이 정하고, 5c-2 는 그 입력(세션 표식과 사건 기록)만 남긴다.
8. **상태**: `real_status.relocalize = {generation, attempt, kind, state, reason, start_pose_m, start_pose_deg}` — 마지막 시도의 기록으로, **세대가 닫힌 뒤에도** 다음 시도나 세션 끝까지 남는다(1 회차 P2-6). `kind ∈ {confirm, declare}`, `state ∈ {idle, waiting, confirmed, declared, refused}`, 거리·각은 마지막 판정 때 `start_rear` 와의 차이(운용자가 우회를 판단할 근거). 기존 `relocalize_generation`(지금 열린 세대)은 그대로 두고, 3 의 `standing` 을 더한다.
9. **콘솔**(5.1 패널에 한 칸): 세대가 열려 있으면 "재위치 확인" 단추, 그 아래 "우회: 시작 자세로 선언" 단추(누르기 전 문구 '차가 바닥 표시 시작 자세에 서 있을 때만'), 그리고 `real_status.standing`(정지 표시)과 `real_status.relocalize` 의 `state`·`reason`·거리·각. 두 단추는 5.1 의 눌림 기록·한 번 해석 규칙을 따르고, **누르기 시작할 때 그려진 세대를 보낸다**(눌림 기록에 세대를 담고 `statusSig` 에 `relocalize_generation` 을 더함 — 1 회차 P2-6). 결과는 ack 의 `(generation, attempt)` 와 `real_status.relocalize` 의 같은 쌍이 맞을 때 그 시도의 결과로 보인다(확인·선언이 같은 번호 줄이라 둘이 섞이지 않음). 비슷한 벽·선반 배치에서의 SLAM 오정합은 PR 5 가 막지 못한다 — 운용자의 육안 확인이 그 몫이고(패널 문구), 정합 품질 지표는 PR 7 의 SLAM 링크가 낸다.

**시작 자세의 출처**(1 회차 P2-7): 루프 생성 인자 `start_rear`·`rear_offset_m`(기본 None). 실차 조립(`tools/real_autonomy.py`)은 S10-A 설정(로더가 `start_rear` 를 필수로 읽음)과 차체 모델에서 늘 넘긴다. None 은 그것 없이 만든 단위 시험 rig 뿐이고, 그때 시도는 `no_start_pose`, 6 의 비교는 하지 않는다.

**가짜 SLAM 원천 계약**(PR 7 의 실 원천도 지켜야 함): `replies(now_ns)` 는 지금처럼 `(reply, meta, sent_mono, received_mono)` 를 `stamp_s` 순서로 내고, 실차 원천의 `reply` 는 `epoch`·`attempt`(없으면 None) 를 단다. `stamp_s` 는 스캔 첫 빔 취득 시각을 코어 시각축으로 옮긴 값이다. **응답의 `epoch` 는 원천이 그 회차를 실제로 시작한 뒤 첫 빔을 취득한 스캔에만 단다** — 회차 시작 전에 시작한 스캔, 회차 시작 전 상태로 계산한 응답은 새 회차로 표기하지 않는다(2 회차 P2-2: 호출 시각과 실제 적용 경계가 달라도 구별됨). scan id 는 한 회차 안에서 늘기만 한다. `relocalize(epoch)`: 큐의 옛 스캔을 버리고 위치 추정을 그대로 이어 새 회차를 시작한다. `seed(epoch, map_from_base, stamp_s)`: 그 `base_link` 자세를 초기 자세로 위치 추정을 다시 시작해 새 회차를 시작한다. `request_processed(epoch, attempt, not_before)`: 첫 빔이 `not_before` 이후인 그 회차 스캔 하나를 키프레임 판정 없이 계산해 `processed` 로, `attempt` 를 달아 낸다. **정지 중 갱신**(5 회차 P2-2→P2-1): 원천은 차가 서 있어도 **응답 간격 0.2 s 이하, 첫 빔 취득부터 루프 수용까지 지연 0.25 s 이하**(잠정)로 `processed` 를 낸다 — 그러면 신선도 증거의 나이가 0.45 s 를 넘지 않아 `SLAM_MAX_AGE_S` 0.5 s 안이다(6 회차 P2-2: 간격만으로는 지연 때문에 넘을 수 있음). 키프레임 판정에 시간 조건을 더한다(이동·회전이 없으면 `skipped` 만 내는 지금 규칙으로는 세대가 닫힌 뒤 운용자 시작 확인·첫 DRIVE 전에 신선도가 끊김). 실 링크의 키프레임 규칙에 이 조건을 넣는 것은 PR 7 이고, 가짜 원천은 이를 지킨다. 원천은 호출을 받은 순서대로 처리한다(3 회차 P2-4): `relocalize`·`seed` 뒤에 받은 `request_processed` 는 그 회차가 실제로 시작된 뒤에 처리하고, 버리지 않는다. 지금 회차보다 옛 회차의 요청만 무시한다.

**5c-2 가 바꾸지 않고 넘기는 것**(10 회차 P2-3): 틱 안의 예외로 비상에 빠지면 그 틱의 `tick_end()` 에 닿지 않아 그 틱에 받은 제출의 콘솔 답이 다음 틱에 `invalid` 로 정산된다 — 4c 의 기존 동작이고 재위치와 무관하게 모든 예외 틱에 해당한다. 비상 경로에서도 받은 요청을 한 번 정산하는 일은 후속(6a 항목에 더함)으로 넘긴다.

**시험**(5c-2, 가짜 트럭 + 가짜 원천 + **실제 `SlamPoseTracker`, 비영 후륜축 오프셋**, 응답 시각을 원천이 정하는 rig): 현장 복구 완료 → 세대 열림 → 옛 증거 폐기(층 새 격자, 추적기 `warming_up`, 코어 hold 표식 없음, 출처 비움) → 거르기의 각 사유(옛 회차, `t_epoch` 전, 미래, 역순, 재표기) → 확인 요청(층 다시 자름) → 기준 응답 → `confirmed` → 제출 가능; 각 거절·`refused`·`relocalize_voided:*` 사유; 같은 콘솔 묶음의 `recovery` 직후 시도는 `not_still`; SLAM 응답이 0.5 s 넘게 없던 정지 차가 확인을 받고 통과; 시도 중 움직임 `relocalize_moved`; 시도 중 새 세대·새 시도 `relocalize_superseded`(확인 뒤 선언, 선언 뒤 확인, 선언 둘); `holding` 중 새 회차 `processed` 가 출처가 되지 않음; 세대를 연 뒤·접수 전에 쌓인 스캔이 확인 결과에 없음, 빈 격자 결과가 먼저 설치돼도 `relocalize_layer`; 미션 중 열린 세대(hold 한도)는 미션이 끝난 뒤에야 추적기를 되돌림; 준비·채택·ARM 대기 중 시작 자세를 벗어나면 `not_at_start`(채택 드리프트 한도 안의 0.099 → 0.109 m 포함), ARM 응답 대기 중 밀린 차는 송신 관문에서 첫 비영 DRIVE 대신 정지·`not_at_start`, 첫 비영 프레임을 반쯤 쓴 뒤 밀리면 이어 쓰기 거절 `link_stuck`; 첫 유효 설치와 층 고장 한도 초과가 같은 틱이면 고장이 먼저(시도 `relocalize_voided:layer`); 원천이 회차를 늦게 시작해도 확인의 강제 요청이 처리됨; 선언 뒤 원천 `processed` 가 없으면 `slam` 우회 중에도 운용자 시작 확인이 `slam_stale`, 미뤄 둔 hold 해제가 만료된 보정으로 일어나지 않음(`slam` 우회 중 포함); 모든 층 리셋이 `_placed_id` 를 비워 리셋 뒤 빈 결과가 옛 마감을 해소하지 않음, 시도 리셋 뒤 0.3 s 의 `layer_overdue` 가 래치 없이 풀림, 단절 경보 처리 뒤 재연결 전 시도가 `relocalize_layer`; 선언 표식이 같은 선언 시도를 `relocalize_voided:bypass` 로 만들지 않음; `refused` 선언 뒤의 `confirmed` 에도 0.06 m/s 상한; 운용자 입력이 0.5 s 넘게 늦어도 정지 중 갱신으로 첫 DRIVE 까지, `slam` 우회 없이 ARM 뒤 응답이 만료되면 송신 관문이 첫 DRIVE 를 막음; 세대 열림 중 응답이 끊겨도 추적기가 `warming_up` 이라 나이 예외 없이 시도가 `relocalize_stale`·시간 초과로 끝남(실제 추적기), 닫힐 때 `tracking`; 옛 회차 응답을 신선하게 만든 뒤 선언하면 새 회차 `processed` 전까지 운용자 시작 확인·ARM·첫 DRIVE 가 열리지 않음; 가짜 원천이 지연 0.25 s·간격 0.2 s 로 응답해도 시작 확인·ARM·첫 DRIVE 통과; 첫 보정 전 재부팅 → 확인 접수 → 응답 대기 동안 비상 없음(실제 추적기, 항등 보정으로 층 작업 계속); `declared` 뒤 SLAM 만 끊고 LiDAR 는 정상인 채로 `overdue_end` 너머까지 래치·STOP 없음, 그동안 콘솔 요청의 답이 같은 틱에 나감; 출력이 닫힌 동안의 hold 이동 한도 초과가 예외 없이 `hold_too_long` 세대를 엶; SLAM 이 끊긴 서 있는 차에서 `not_stopped_s` 너머까지 텔레옵 진입·재무장이 거절되지 않고 STOP 도 없음; 첫 `processed` 전의 새 회차 `skipped` 가 걸러져 비상 없음; 예약 없는 hold(화물 상태가 해제를 막음)도 출력이 닫힌 동안 한도를 넘으면 `hold_too_long`; SLAM 이 끊긴 서 있는 차의 재무장이 `stop_unconfirmed` 없이 `STOPPED → PAUSED`; 무장 중의 나이 초과는 지금처럼 비상; `max_age_s` 가 다른 실제 추적기로 만든 루프의 `ValueError`; `start_pose` 우회로 0.15 m 어긋난 자세에서 확인되고 우회 표시, 새 세션에서 사라짐; 세대 열림 중 `operator_start` 거절·실차 텔레옵 닫힘(기존 시험 유지); 선언: 합성 보정의 `base_link` 가 `base_from_rear(start_rear)` 에 놓이고 후륜축 비교가 0, 같은 묶음의 선언 뒤 `lease_release` 면 `seed` 없이 `relocalize_voided:lease`, 선언 틱의 원천 옛 회차 응답이 합성 보정을 덮지 않음, `declared` 뒤 그 세션 내내 `bypassed` 에 `relocalize`·출력 0.06 m/s 상한, `check_bypass{relocalize}` 거절, 새 세션에서 표식·상한이 사라짐, 합성 응답이 `pose_reply_ns` 를 바꾸지 않음; `start_rear` 없는 rig 의 `no_start_pose`; Isaac 경로 불변(새 메서드를 부르지 않음, `SlamPoseTracker` 기본 동작 시험 그대로).

## 7. 정지 뒤 출발과 포켓 구간 — 두 방향 질의와 D5 우회 (5d)

> **수렴**(5d, 8판 — Codex 8 회차 새 P1/P2 없음; Codex 1 회차 P1 2·P2 4, 2 회차 P1 3·P2 3, 3 회차 P1 1·P2 2, 4 회차 P1 1·P2 2, 5 회차 P1 2·P2 1, 6 회차 P1 2·P2 1, 7 회차 P1 1·P2 1 반영). 예전 판(7 회차까지 반영, 미수렴)의 두 방향 질의를 지금 코드 이름에 맞추고, 총괄 결정(2026-11-19, 사용자 원칙: 예전 'D5 끔 우회는 만들지 않음'을 바꿈)의 D5 포켓 확인 우회를 더했다. 실 D435i 깊이 경로와 실차 `PocketCheck` 생성기는 PR 7 이다(지금 `RealHooks.build_pocket_check` 는 생성기가 없으면 `pocket_check_unavailable`).

### 7.1 두 방향 질의 (4b 8 절이 넘긴 '정지 뒤 방향 판정')

실차 엔코더 속도 모형은 서 있어도 가속도 오차 항 때문에 두 방향을 남긴다. 지금 코어의 D5 분기(`MissionCore.permission`, 포켓 구간)는 그때 `_direction_limits` 의 `unsure` 로 `direction_unsure` 0·hold 를 낸다(4c 시험 `test_real_loop_s10` 이 고정). 정지 증거로 `SafetySpeed` 를 0 으로 바꾸면 역방향 정지 호 검사가 빠지므로(3 회차 P1-1) 그렇게 하지 않는다. 속도 상한·가능한 방향은 그대로 둔다.

**바꾸는 것**(코어 포켓 구간의 **실차 경로만** — `self._real_vehicle()` 이 참일 때, 가능한 방향이 하나여도 둘이어도 이 규칙(2 회차 P2-1: 요청 0 의 감속 slew 가 한 방향으로 남는 경우 포함); Isaac 은 지금 경로 그대로이고 새 속성·훅을 읽지 않는다):
1. **순서**: 포켓 구간을 `pocket_missing` → 정지 프레임 관문(`await_still_frame`) → 접근 중간 정지 프레임 걸기(`mid_still_done`) → (두 방향 질의 또는 지금의 한 방향 질의) 로 한다. 관문이 0 을 내면 그대로 0 이고 질의는 부르지 않는다. 지금은 `unsure` 검사가 관문 앞에 있다.
2. **가능한 운동**(방향 → 속도): `_direction_limits` 와 같은 규칙 — 측정 속도의 가능한 방향마다 엔코더 상한, 지금 명령의 방향에 그 크기, 루프의 `outstanding_motion`(미확인 출력)의 방향마다 그 값, 각 방향의 최대. 함수 `_possible_motion(speed)` 로 떼어 둘이 같이 쓴다. 미확인 출력 `residual(d)` 는 따로 둔다.
3. **질의**: 요청이 있으면(방향 `r`) `r` 은 `max(새 요청 속도, possible(r))` 로, 나머지 가능한 방향 `d ≠ r` 는 `possible(d)` 로 `check.limit(t, rear, kappa, d, v, lift)` 를 부른다. 요청이 0 이면 가능한 방향마다 `possible(d)` 로 부른다(`directions_[0]` 은 기록용일 뿐).
4. **판정**(1 회차 P2-2):
   - **모든 가능한 방향** `d`(요청 방향 포함): 허용치 `< possible(d)` 이면 hold(`speed_hold_`, `motion_hold`) — 그 방향으로 지금 있을 수 있는 운동의 정지 호가 덮이지 않았다(2 회차 P1-1: 낮은 양의 목표를 보내는 것으로는 그 정지 거리를 보장하지 못함). 사유는 검사기의 사유, 그것이 `ok`(부족한 양의 허용치)이면 요청 방향은 `residual_over_allowed`, 다른 방향은 `pocket_other_direction`.
   - **면제 밖(층)도 같은 비교**(3 회차 P1): 실차 포켓 구간에서는 요청 방향의 층 허용치도 `possible(r)` 로 한 번 더 묻고(`layer.limit(r, possible(r))`, 실차 경로만이라 Isaac 재생의 기록 호출은 그대로), 그것이 `possible(r)` 보다 작으면 hold(사유는 층의 사유). 다른 방향의 층 비교는 `_direction_limits` 가 이미 그 방향의 가능한 속도로 한다. 그래서 D5 가 양쪽을 허용하거나 우회 검사기가 `inf` 를 내도, 면제 밖 정지 호가 가능한 운동을 덮지 못하면 4c 의 `direction_unsure` 처럼 멈춘다.
   - hold 가 아니면 요청 방향의 새 속도를 **요청 속도로 물은 답들**(`_direction_limits` 의 층 상한과 3 항의 D5 상한) 중 작은 값으로 제한한다. `possible(r)` 로 다시 물은 층 답은 hold 판정에만 쓴다(4 회차 P2-1: 질의 속도마다 검사하는 정지 호 길이가 달라, 작은 속도의 답이 더 큰 수일 수 있음).
5. **호출 조건**(5 회차 P1-2): 4c 루프는 요청이 0 이고 측정 속도가 0.05 m/s 이하이면 `core.permission()` 을 건너뛴다. 포켓 검사기가 있는 구간(`pocket["check"]` 있음 — 7.2 의 우회 검사기 포함)에서는 요청이 0 이어도 `outstanding_motion` 이 있거나 엔코더 상한이 0 보다 크면 부른다.
6. **쓰기 여유를 둔 질의**(3 회차 P2-1, 4 회차 P1, 5 회차 P1·P2 — 5판의 '송신 순간 재질의' 를 철회: 실제 `PocketCheck` 두 방향 질의가 중앙값 27.7 ms 로 5 ms 송신 예산에 들어가지 않음): D5 와 층의 답은 **물은 시각에서의 증거**(최신 프레임 나이, 정지 호에 필요한 복셀 하나하나의 증거 나이, 층의 FREE 증거 나이)로 정해진다. 그래서 실차에서는 제어 단계의 질의를 그 틱의 정상 DRIVE 가 쓰일 수 있는 **가장 늦은 시각** `t + W` 로 한다 — `W = SLOW_TICK_S + EVIDENCE_READ_MAX_S`(0.030 + 0.005 = 0.035 s). 지금 송신 관문은 틱 시작에서 `SLOW_TICK_S` 를 넘긴 정상 DRIVE 를 `tick_slow` 대기로 돌리고 증거 읽기를 `EVIDENCE_READ_MAX_S` 로 묶으므로, 쓰인 DRIVE 는 늘 질의 시각 이전에 쓰인 것이다. 적용: 실차의 `MissionCore.permission` 이 포켓 검사기 `limit` 에 넘기는 시각, 그리고 층에는 **증거 시각** — `RealLayer.limit(now_s, ..., evidence_s=None)` 에 새 인자(실차 층만, 기본 None = `now_s`)를 더해 스캔 나이(`scan_stale`)·FREE 증거 나이 비교는 `evidence_s = t + W` 로, 상태를 바꾸는 마감 판정(`watchdog` 의 `_missed_expected`·`_overdue_since`·`_exceeded`)과 설치 판정은 실제 시각 `t` 로 한다(6 회차 P2: `t + W` 를 watchdog 에 넣으면 미래의 마감 위반을 지금 상태에 기록함). `_direction_limits`·4 항의 재질의도 같은 인자. Isaac 은 `t` 그대로이고 새 인자를 넘기지 않아 재생 기록 불변. D5 우회 중에도 층 질의는 같다(5 회차 P1-2). 질의 속도는 그 방향의 프레임 속도를 이미 포함한다(요청 방향 `max(새 요청, possible(r))`, 새 속도는 그 답 이하로 제한되므로 큐에 넣는 프레임 속도 ≤ 허용치 — 5 회차 P1-1).
   - **쓰기 경계의 기한**(6 회차 P1-1): 지금 `judge` 는 시계를 앞에서 한 번 읽고(`ns = ns0 + 5 ms`) 뒤의 층·lease·프레임 검사를 한다 — 검사 뒤 실제 쓰기 시각은 묶이지 않는다. 그래서 비영 정상 DRIVE 는 `judge` 의 **마지막 단계**에서 시계를 다시 읽어 **`min(ns, 결정 시각 + W)`** 이내일 때만 통과한다 — `ns`(= 앞쪽 읽기 + `EVIDENCE_READ_MAX_S`)는 lease·계측·준비 증거를 판정한 시각이고, 결정 시각(`expires_ns − DRIVE_LIFE_S`) + `W` 는 질의 증거의 시각이다(7 회차 P1: 둘 중 이른 쪽을 넘기면 그 사이에 만료된 권한·증거로 쓸 수 있음). 첫 쓰기·이어 쓰기 모두; 넘으면 첫 쓰기는 `write_late` 대기, 이어 쓰기는 거절 → `link_stuck`). 그 마지막 읽기에서 `write()` 시작까지는 `send_boundary` 의 잠금·일련번호 비교뿐이다. 마지막 시계 읽기부터 실제 첫·이어 쓰기까지의 지연 계약(그 안에 끝나야 하는 상한)은 6a 의 TtyLink 계약(이미 넘긴 항목)에 둔다 — 가짜 전송기는 즉시 쓴다.
   - **이번 틱이 멈추기로 했으면 이어 쓰지 않는다**(6 회차 P1-2): 이번 틱의 결정이 정지·대기(대기 프레임, 0 DRIVE, hold)이면 송신 관문은 반쯤 쓴 비영 정상 DRIVE 의 이어 쓰기를 기한과 무관하게 거절한다(`link_stuck`) — 새 정지 결정이 옛 운동 프레임보다 앞선다. 실 전송기에서 폐기한 반쪽 프레임이 장치에서 완성되지 않는 조건(TTY 재개방의 0x00 등)은 6a 의 TtyLink 계약 그대로다.
- D5 의 답은 면제 영역에 해당하는 부분만이다. 영역 밖은 주행 격자(층의 방향별 검사, 다른 방향 hold)가 지금처럼 맡는다.
- 그래서 정지 직후의 첫 출발과, 첫 DRIVE 뒤 수락·구동 지연 동안 엔코더가 아직 두 방향을 보일 때(3 회차 P2-1)도 D5 가 두 방향을 답하는 한 출발이 이어진다.

### 7.2 D5 포켓 확인 우회 `d5_pocket`

실 D435i 근거리 인식이 0/240(#594)이고 실차 `PocketCheck` 생성기도 PR 7 이므로, S10-A 를 운용자 입력 자세만으로 돌려야 할 수 있다. 5 절의 `check_bypass` 에 `d5_pocket` 을 더한다(Python·JS `BYPASSABLE`, 라벨 "포켓 확인"). 5 절 규칙 그대로 허가 OFF·LATCHED 이고 미션이 없을 때만 바뀌므로 **한 미션 안에서는 바뀌지 않는다**.

**우회 검사기**(1 회차 P1-1·P1-2·P2-1 을 한 장치로 닫음): 우회 중에는 포켓 검사기 자리에 **우회 검사기**를 넣는다 — 그 밖의 코어 경로(면제 설치·해제, 삽입 진입 요구, 삽입 끝 재검사, `acting`, 두 방향 질의)는 검사기가 있는 지금 경로 그대로 탄다.
- **루프 → 코어**: 루프는 틱 머리에서 `core.d5_bypassed = "d5_pocket" in 이 틱의 우회 집합` 을 둔다(코어 속성, 기본 False — Isaac 은 두지 않으므로 불변).
- **설치**(`_known_first_frame`, 알려진 시작의 직선 앞 정지): 실차이고 `core.d5_bypassed` 이면(이 순서로 읽음 — Isaac 은 새 속성을 읽지 않음, 2 회차 P1-2) 깊이 프레임을 기다리지 않고 같은 자리에서 새 훅 `hooks.build_bypass_pocket_check(axis_yaw, pickup)` 를 부른다 — `pickup` 은 미션이 고정한 계획 자세 `pocket["planning_pickup"]`(제출 자세, 2 회차 P2-2; 조립의 기준 팔레트 자세가 아님). 훅(`hook_contract.HOOKS` 에 등록, 실차만 구현)은 `BypassPocketCheck` 를 **만들어 돌려주기만** 한다(부수 효과 없음). 코어가 그것을 **먼저** `pocket["check"]` 에 넣고, 그다음 층의 면제 영역을 그 `region_control()` 로 걸고 기록한다(2 회차 P1-3: 그 사이에 실패해도 검사기가 있으므로 수거가 면제를 지운다). 그 틱은 0 `pocket_first_frame`, 다음 틱부터 진행 — 지금의 프레임 경로가 검사기를 세운 뒤와 같다.
- **`BypassPocketCheck`**(새, `live/real_assembly.py`): `valid = True`, `bypass = True`; `region_control()` = **등록 팔레트 자세(운용자 입력)** 중심, 길이 = 팔레트 깊이 + 2 × 0.60 m, 폭 = 팔레트 폭 + 2 × 0.20 m, 방향 = `axis_yaw`(`PocketCheck.region_control` 과 같은 띠, `PocketConfig.band_along_m`·`band_across_m`; 팔레트 치수는 S10-A 설정의 `pallet_depth_m`·`pallet_width_m`); `limit(...)` = `(inf, "ok")`; `add_frame` = 아무것도 안 함(`{"new": False}`); `summary()` = `{"bypass": True, "region": ...}`; `last_new_s = -inf`.
- **우회 검사기에서만 건너뛰는 것**(코어 D5 분기와 `transition()`, 읽는 순서는 `self._real_vehicle() and getattr(check, "bypass", False)` — Isaac 재생의 검사기 프록시가 이 속성 읽기를 기록과 대조하므로 실차가 아니면 읽지 않음, 2 회차 P1-2): 정지 프레임 관문을 걸지 않음(`next_phase == "insert"` 의 `await_still_frame` 설정과 접근 중간 정지 프레임) — 기다릴 깊이 프레임이 없다. 루프의 `_feed_depth` 는 검사기가 우회 검사기이면 깊이 원천을 부르기 **전에** 돌아간다(2 회차 P2-3: 원천의 예외가 우회 중에 비상을 일으키지 않게). `check.limit` 은 늘 `inf, ok` 이므로 질의와 hold 는 자연히 통과한다.
- **그대로인 것**: `acting = True`(포켓 구간 분기 그대로 — 영역 밖 주행 격자의 0 이 출력에 걸림, 1 회차 P1-2); 면제 해제는 `insert → lift` 전이와 미션 끝 수거(`end_reset`)의 지금 경로(`pocket["check"]` 가 있으면 `layer.exempt = None` — 층의 `exempt_serial` 이 올라 옛 결과가 다시 설치되지 않음, 1 회차 P1-1); 삽입 진입의 `pocket_check_missing_at_insert` 요구(검사기가 있으므로 통과); 삽입 끝(`_real_insert_end`)의 실측 정지·목표 자세 재검사(검사기의 답은 `ok`); 층의 0 이 5 s 이어지면 `pocket_stop`.
- **저속 상한**: `d5_pocket` 이 우회 집합에 있는 동안 출력 상한을 `declared_speed_mps`(0.06 m/s, 잠정 — S10 속도 승인 목록, 6 절 7b·`slam` 우회와 같은 자리·값). 상한 규칙: `slam` 우회 또는 `d5_pocket` 우회 또는 선언 표식이면 0.06.
- **표시·기록**: `real_status.bypassed` 에 `d5_pocket`, 콘솔 띠 "우회 중: 포켓 확인". 사건 기록 `d5_bypass_check{region, pallet, axis_yaw}`.
- 영역 안은 아무 센서도 보지 않는다 — 운용자가 고른 우회의 뜻이고, 남는 것은 저속과 우회 불가 목록(멈추는 쪽)이다. 등록 자세가 틀려 실제 팔레트가 띠 밖에 걸리면 주행 격자가 그 점유를 막는다(셀의 네 모서리가 띠 안일 때만 FREE — 1 회차 P2-4).

### 7.3 시험

- 7.1(방향별로 답이 다른 가짜 검사기, 실제 엔코더 모형 — 배선 검증에 충분, 1 회차 P2-3; 가짜 검사기는 물은 시각과 받은 프레임의 취득 시각으로 답하고 `last_new_s` 를 받은 프레임으로 갱신한다 — 지금 rig 의 `_OkCheck` 는 0.0 에 고정이라 고친다, 4 회차 P2-2; 복셀 만료 경우는 실제 `PocketCheck` 의 시각 의존 답을 흉내내는 가짜로): 정지 프레임 대기 중에는 질의 호출·비영 DRIVE 0; 프레임 뒤 양쪽이 허용하면 출발하고 몇 틱 동안 이어짐; 한쪽만 부족하면 대기 프레임(hold, `pocket_other_direction` 또는 검사기 사유); 요청 방향의 가능한 속도(엔코더 상한이나 미확인 출력)가 허용치보다 크면 `residual_over_allowed` hold(2 회차 P1-1: 엔코더 상한 0.07·미확인 0.03·허용 0.037 m/s 에서도 hold); 요청 0 에서 모든 방향 질의, 가능한 방향이 하나인 감속 slew 도 같은 규칙; 요청 0·`outstanding_motion` 있음에서도 `permission()` 이 불림; 4c 의 `direction_unsure` 시험은 새 규칙으로 바꿈; Isaac(한 방향)은 그대로(기존 시험). 실제 `PocketCheck` 합성 깊이의 양방향 정지 호 시험은 PR 7 의 깊이 경로와 함께(지금 합성 시험은 전진만 묻는다). **PR 7 선행 조건**(7 회차 P2): 실 검사기와 층 질의를 함께 넣은 실차 틱이 정상 DRIVE 를 틱 시작 + 25 ms 안에 송신 관문에 넘기는지(`tick_slow` 는 `ns0 + 5 ms` 를 + 30 ms 와 비교) Jetson 에서 확인한다 — 로컬 합성 측정의 양방향 D5 중앙값 27.7 ms 는 이 예산을 넘으므로, 넘으면 질의를 줄이거나(예: 다른 방향은 가능한 속도가 0 이면 생략) 그 측정을 근거로 다시 정한다.
- 7.2(실제 `RealLayer`, 등록 치수 = 면제 치수 — rig 의 계획 형상 폭 0.80 과 등록 폭 0.40 차이를 맞춤, 1 회차 P2-4): 우회 없이 프레임이 있고 생성기가 없는 차는 `pocket_check_unavailable`, 프레임도 없으면 `pocket_stop in approach: pocket_first_frame`(지금 동작); `d5_pocket` 우회로 같은 차가 프레임 없이 첫 정지 자리에서 우회 검사기를 세우고 삽입·승강·S10 정지까지 감, 면제 중심이 조립의 기준 자세가 아니라 제출 자세(등록 자세를 기준과 다르게 둔 경우); 면제를 **건 직후**(사건 기록 시점)에 실패를 주입해도 수거 뒤 면제가 남지 않음(3 회차 P2-2: 설정 전 실패로는 순서 회귀를 못 잡음); 면제 밖 점유로 층의 요청 방향 허용치가 0 보다 크지만 가능한 속도보다 낮으면 우회 중에도 대기 프레임(3 회차 P1); 7.1 에서 질의가 `t + W` 로 감 — 최신 프레임은 유효하고 정지 호에 필요한 복셀이 `t` 와 `t + W` 사이에 만료되는 증거면 그 틱의 답이 이미 0(`pocket_unverified`)이고, 층 FREE 증거도 같음(4·5 회차 P1); 비영 DRIVE 이어 쓰기가 결정 시각 + W 를 넘기면 `link_stuck`; judge 의 앞쪽 시계 확인 뒤 지연을 주입해 마지막 읽기가 `min(ns, 결정 + W)` 를 넘기면 첫 쓰기는 `write_late` 대기(쓰인 DRIVE 없음); 삽입 중 반쯤 쓴 비영 DRIVE 뒤 다음 틱이 D5 hold 를 결정하면 그 프레임은 완성되지 않고 `link_stuck`; 층 `limit(evidence_s=t+W)` 가 watchdog 상태를 앞당기지 않음(실제 시각의 `layer_overdue` 판정 불변); lease 가 틱 시작 + 15 ms 에 만료되고 마지막 검사가 + 20 ms 에 돌면 비영 DRIVE 가 쓰이지 않음(7 회차 P1, `min(ns, 결정 + W)`); Isaac 질의 시각 불변(재생 게이트); 조회하면 예외를 내는 깊이 원천으로도 우회 중 비상 없음; Isaac 재생 게이트 29/29(새 속성 읽기 없음); 그동안 쓰인 비영 DRIVE 가 모두 0.06 m/s 이하이고 적어도 하나는 요청이 0.06 을 넘었음; 영역 밖에 장애물을 두어 요청 방향만 막히면 비영 DRIVE 0(1 회차 P1-2); 삽입 끝 뒤와 미션 취소 뒤 `layer.exempt` 가 None 이고 해제 전 결과가 다시 설치되지 않음(1 회차 P1-1); `real_status.bypassed` 에 `d5_pocket`; 새 세션에서 우회가 사라짐; 무장 중·미션 중 `check_bypass{d5_pocket}` 거절.

## 8. 단일 실행기

**별도 PR**(`feat/executor-lock`). 11-20 보드 확인일 전에 머지하려 했으나, 기존 명령 도구(`bringup.py`·`rearm_hold_check.py`·`h743_link_check.py`)도 같은 잠금을 거쳐야 단일 점유가 성립해(그 PR 의 Codex P1) **11-20 뒤로 미뤘다**(총괄의 대안). 그때 모든 명령 도구에 같은 잠금을 건다.

- 잠금 디렉터리 대신 **링크가 연 장치 fd 자체에** `flock(LOCK_EX | LOCK_NB)` 을 매 열기(재연결 포함)마다 건다(`forklift_core.live.executor_lock`). 별칭은 같은 장치 inode 로 풀려 같은 잠금이고, 재연결이 다른 장치 노드에 붙으면 그 fd 를 새로 잠근다 — '연 fd 가 잠근 장치' 가 늘 성립한다. 디렉터리 권한(이 노트북의 `/run/lock` 은 root 전용)에 기대지 않는다.
- 잠금은 `open_raw` 가 fd 를 연 직후, termios·DTR 설정 **전에** 건다(3 회차 P2-6) — 거부된 열기는 잡힌 포트에 아무것도 설정하지 않는다.
- 프로세스가 죽거나 fd 가 닫히면 커널이 푼다. 남는 파일이 없다(조건 a). 거부는 `/proc/locks` 의 FLOCK 줄에서 찾은 pid 와 명령줄을 보인다(조건 b). 잡던 자식을 `SIGKILL` 한 뒤 다시 열림을 시험한다(조건 c).
- `ReconnectingLink(exclusive=False)` 가 기본이라 듣기만 하는 도구는 그대로다. 텔레옵 다리는 `exclusive=True` 와 시작 때 확인. 자율 실행기는 6a 의 장치 배선 때 같은 인자를 쓴다.
- 남는 한계: 잠긴 장치를 두 번째로 **여는** 것 자체는 막지 않는다(열고 잠금 실패 뒤 닫음). USB CDC 에서 그 열기는 이미 켜진 DTR 를 다시 켤 뿐이고, 마지막 닫기가 아니라 줄을 끊지 않는다. 다만 같은 값의 제어 요청도 TinyUSB 콜백을 거쳐 H743 송신 링을 비울 수 있다(소유자 쪽 STATE 몇 개가 빠짐). 거부 시 '쓰기 0' 은 사용자 공간의 프로토콜 바이트에 대한 말이다. 벤치에서 단발 거부와 반복 경합 때 소유자 쪽 STATE 간격과 정지 동작을 본다.

### 8.1 재개 (2026-11-25, 시운전 한계 사슬 #614·#615·#616 뒤)

옛 브랜치의 두 커밋(f5468245·9425a641)은 main 11311327 위에 충돌 없이 올라간다. 그 위에 범위만 넓힌다.

- **잠그는 곳 = MCU 장치를 여는 모든 도구.** 명령을 보내는 도구만이 아니다. 같은 tty 를 두 프로세스가 열면 받는 바이트를 나눠 읽는다(도착일 절차 11 절). 그래서 듣기만 하는 도구도 실행기 옆에서 STATE 를 빼앗는다. 대상은 다음과 같다.
  - `bringup.py`: 장치를 여는 모든 하위 명령(`record`·`cpr`·`step`·`scenario`·`commission`). `kth`·`clock`·`analyze` 는 기록만 읽으므로 대상이 아니다.
  - `rearm_hold_check.py`, `h743_link_check.py`, `teleop_bridge.py`.
  - `real_mcu_link.py` 의 듣기 CLI.
  - `real_autonomy.py --run` 은 지금 실 장치를 열기 전에 거부한다. `--hil` 은 제 pty 를 쓴다. 그래서 둘 다 대상이 아니다. PR 7 이 실 장치를 열 때 같은 인자를 쓴다.
- **방식**: 기존 그대로 한다. `ReconnectingLink(exclusive=True)` 가 매 열기(재연결 포함)마다 연 fd 에 `flock` 을 걸고, `open_raw(before_config=)` 로 termios·DTR 설정 전에 건다. 직접 `open_raw` 를 부르는 `h743_link_check` 는 같은 `before_config` 로 잠근다. 보유자 표시는 연 fd 의 `fstat` 으로 `/proc/locks` 를 찾는다. 별칭이 바뀌어도 다른 노드의 pid 를 보이지 않는다. 보유자를 못 찾아도 거부는 그대로다.
- **시작 거부 = 첫 열기**(설계 1 회차 P1-2·P2, 2 회차 P2): 시작 때 따로 여는 `probe` 는 없앤다. USB CDC 에서 probe 의 열기·닫기가 DTR 를 건드려 펌웨어가 STATE 를 버리거나 송신 링을 비우기 때문이다.
  - `exclusive=True` 링크가 **한 번도 연결되지 않은 상태에서** `DeviceBusy` 를 만나면 그 링크는 **거부 상태**로 끝난다. 그 한 번만 예외를 올린다. 이후 `service`·`receive`·`transmit` 은 열지도 쓰지도 않고 빈 결과를 돌려준다. 그래서 도구의 정리 경로(텔레옵 `close()` 의 DISARM 등)가 그 뒤 장치를 다시 열거나 두 번째 예외로 종료 코드를 덮지 않는다.
  - 각 도구는 그 예외를 잡아 아무것도 쓰지 않은 채 거부하고 종료 2 로 끝난다. JSON 을 내는 도구는 `{"result": "refused", "why": "<경로> is held by another executor: pid N (도구)"}` 를 낸다. `bringup.execute` 의 포괄 예외 처리는 이 예외를 `aborted` 로 바꾸지 않고 그대로 올린다.
  - 연결된 적이 있는 링크의 재연결에서 만난 잡힘은 지금처럼 `device_busy` 사건과 재시도다. 끊김이 이미 호스트 권한을 거뒀으므로 진행 중인 실행은 끊김 경로로 끝난다. 텔레옵 다리의 시작 probe 도 이 규칙으로 바꾼다.
- **열기의 첫 프레임 앞에 독 바이트열**(설계 1 회차 P1-3, 2 회차 P1·P2, 3 회차 P2): 앞 실행기가 본문까지 쓰고 끝 구분자를 못 쓴 채 죽으면, 새 열기의 구분자 `0x00` 이 그 반쪽을 온전한 프레임으로 닫는다. 유효기간이 없는 DISARM·CLEAR_FAULTS 도 그렇게 늦게 작동할 수 있다.
  - 그래서 `TtyLink` 는 열기마다 **첫 호스트 프레임 바로 앞에** 6a 의 독 바이트열 `0x01 × 96 + 0x00` 을 쓴다(6a 설계 3 절, `real_loop_transport.POISON`). 두 디코더는 구분자 사이 인코딩 길이가 91 을 넘으면 길이 초과로 버리므로, 반쪽은 어디서 끊겼든 버려진다.
  - 지금의 '열자마자 구분자 하나' 는 없앤다. 보낼 프레임이 없는 열기(듣기만 하는 `record`·`cpr`·관측 CLI)는 **한 바이트도 쓰지 않는다**. 반쪽을 닫는 쓰기가 없으니 반쪽은 무해하게 남는다. 그래서 MCU 수신 오류도 늘지 않고, S2 첫 부팅 판정(첫 STATE 부터 10 s 동안 `rx_errors` 증가 0)이 그대로다.
  - 독 바이트열이 `EAGAIN` 으로 반쯤 쓰이면, 꺼낸 프레임은 호스트에 돌려주고(`host.unsent`) 다음에 남은 바이트부터 이어 쓴다. 다 쓰기 전에는 프레임의 첫 바이트를 쓰지 않는다.
  - 직접 쓰는 `h743_link_check` 는 여는 구분자 대신 같은 독 바이트열을 `_write_all` 로 쓴다. MCU 수신 오류의 기준(`RX_BASELINE_S`)은 **독 바이트열을 다 쓴 시각**부터 센다.
  - 1판의 0.25 s 조용한 구간과 `TCIFLUSH` 는 철회한다(2 회차 P2: 만료 없는 명령을 못 막고, flush 가 수신 프레임 경계를 깸).
  - 기존 시험의 '열기마다 첫 바이트 `0x00`' 단언(USB CDC 시험)과 PTY 부분 쓰기 시험의 주입 시점(첫 DRIVE 본문이 실제로 쓰인 뒤)을 이 규칙에 맞춘다.
- **무장 전 재연결은 실행을 끝낸다**(설계 3 회차 P1): 구동을 요청하는 `bringup` 실행(`step`·`scenario`·`commission`)과 `rearm_hold_check` 는 ARM 을 요청하기 직전에 이 실행의 열기 수(`Bench.conn`)를 본다. 1 보다 크면 ARM 없이 거부한다(`the link reconnected before the ARM: run it again to confirm`). 운용자 확인은 그 연결에 대해 받은 것이기 때문이다. 기록용 재연결과, 이미 무장을 요청한 실행의 정리(정지 확인)는 지금처럼 재연결을 쓴다.
- **스테이징 사본**(설계 1 회차 P1-1): 보드 점검 절차는 `data/h743_check/code/`(`tools/h743_check/stage.sh` 가 만든 사본)를 실행한다. 그래서 이 PR 머지 뒤 다음 벤치 준비에서 **main 으로 다시 스테이징**한다. 그 사본은 잠금이 없는 옛 코드이기 때문이다. 도착일 절차의 벤치 준비 줄에 넣는다. 또 `3_link_check.sh` 는 사본의 `h743_link_check.py` 가 잠금(`before_config=`)과 독 바이트열을 쓰는지 grep 으로 보고, 아니면 멈춘다(2 회차 P3: 파일 존재만으로는 중간 커밋을 못 거름). 이 검사는 다시 스테이징한 런처부터 적용된다.
- **고정 시험은 보조**(설계 1 회차 P2): `tools/*.py` 를 AST 로 훑는다. `ReconnectingLink(` 호출에 `exclusive=True` 가, `open_raw(` 호출에 `before_config=` 가 있어야 한다. 이름을 적은 허용 목록만 예외다. 보장은 아래 도구별 시험이 한다. 범위 밖은 이 설계에 적는다. C 모형(`mcu_model --link`)과 LiDAR 런처(`scan-a2m12`)는 MCU 경로를 받는 운영 경로가 아니다. `h743_link_check.run(fd=)` 는 시험 전용이다.
- **flock 의 보장 단위**(설계 1 회차 P2): 같은 장치 노드 inode 와 협조하는 호출자다. 같은 노드를 가리키는 별칭(`/dev/forklift-mcu`, by-id)은 같은 잠금이다. USB 재열거로 새 노드가 생기면 옛 fd 는 끊긴 장치에 남는다. 잠금 둘이 곧 살아 있는 tty 의 리더 둘은 아니다. 보드 하나에 실행기 하나는 udev 별칭이 한 보드를 가리킨다는 운용 전제 위에 선다. 보드 정체 확인은 기존의 제품 문자열·UID 확인이 맡는다.
- `ReconnectingLink` 의 기본값 `exclusive=False` 는 그대로 둔다. 시험·rig 가 같은 pty 를 여러 번 여는 경우가 있어서다.
- 시험(8 절에 더해):
  - 도구별로, 모형 pty slave 에 다른 fd 로 `flock` 을 미리 건다. 그러면 첫 열기에서 아무것도 쓰지 않고 종료 2 로 끝나야 한다. 보유자가 곧 닫아도 재시도·송신이 없어야 한다.
  - 연결된 뒤의 재연결이 잡힌 장치를 만나면 `device_busy` 와 재시도.
  - 첫 거부 직후 보유자가 닫아도 열기 1 회·송신 0·종료 2(텔레옵 정리 경로 포함).
  - 첫 프레임 앞에 독 바이트열이 있고, 앞 연결이 남긴 '끝 구분자만 빠진' DISARM·DRIVE 가 MCU 모형에 닿지 않음. `record` 실행은 한 바이트도 쓰지 않음.
  - 확인 뒤 ARM 전에 끊겼다 다시 붙으면 ARM 0 건·거부.
  - 고정 시험.
  - Jetson·H743 의 DTR·재열거 시각은 실측하지 않았다. 벤치 확인 항목이다.

## 9. 시험

- 층: 속성 표(S10-A 전 구간, `snapshot` 포함), 원시 오도메트리로 놓인 스캔(보정 이중 적용 없음), 무효 빔·유효 비율 미달 스캔, 스캔이 시작하는 첫 스냅숏, 요청 열쇠 불일치 결과 버림(면제 해제·경로·기어 구간·미션), 합치기에서 마감 유지, 이동 거리 `advance`, 작업자 죽음 ①, 면제 해제 → 대기(래치 아님), SLAM 무효 뒤 입력만 복구 → 재개 없음, 계획 요청은 세대가 맞을 때만.
- 분리: `suppress=None` 바이트 동일, 표시만 빠지고 지우기는 같음, 미션 전 스캔도 빠짐, `ObstacleGrid → plan_transport` 회귀, 주행 스냅숏에는 팔레트가 남음.
- 강제·구분: 층 없음·`obstacle_act` 꺼짐에서 채택 거부, 실행 중 해제 ①, 첫 바이트의 대기 사유가 ARM 을 막음.
- 감시: 설정에서 조립; `ImuWatch` — 표본 없음·기준 없음에서 채택 거부, 끊김 ①(첫 바이트 포함), 같은 표본 재사용은 지속이 아님, 무효 값 무시, 기울기 지속 ①.
- 콘솔 통합: 5 절의 요청들로 끝까지, 금지 명령 0, 재개 없음, lease 만료와 재무장.
- 재위치: 완료 거절(DISARM 미확인·명령 남음·ID 다름), 세 원인 각각에서 세대 열림 → 증거 폐기, 확인 조건 각각의 거절, 늦은 답 무시, 확인 뒤에도 재개 없음.
- 잠금: 8 절의 시험.
- 정지 뒤 출발: 7 절의 시험. D5 가 받은 두 `(방향, 속력)` 인자를 기록으로 확인(전진 요청 + 후진 잔류).
- watchdog 시각: 첫 빔 1.00 s, 마지막 빔 1.10 s, 첫 사용 1.12 s 에서 FREE 나이 0.12 s, 처리 지연 0.02 s(정상 스캔이 `layer_overdue` 가 아님).
- 출력 큐: 관문이 닫힌 뒤 쓰인 비영 DRIVE 0 건(가장 새 DRIVE 하나).
- 스캔 버림: 구간 속도가 `v_max` 초과(밀린 차), 같은 부팅 단절 뒤 층 초기화와 재위치 세대.
- 층(3 회차 P3): 같은 스냅숏에서 앞뒤 두 방향 질의, 기어 구간이 바뀌면 옛 경로 검사가 쓰이지 않음, 조립된 허가 설정에 B 가 더해짐.
- 재위치·제출: 같은 틱의 SLAM 적용 뒤 판정, 출처 보존(`skipped` 뒤에도 확인), 거른 응답은 코어에 가지 않음, 확인 뒤 밀린 차의 제출 거부(`not_at_start`).
- Isaac: 재생 게이트 29/29, ws1 전체 pytest.

## 10. 자체 검토

- Codex 5d 구현 검토 1–6 회차(새 스레드 `01a1f400`, `50c6a5b0` … `5dba3140`): 1 회차 P1 1·P2 3·P3 1, 2 회차 P2 3, 3 회차 P2 3, 4 회차 P2 2, 5 회차 P2 1, 6 회차 **새 P1/P2 없음**(수렴). 구현에서 정한 것: 우회 검사기는 첫 정지 관문(`stop_now`) 뒤에만 설치; 루프의 요청 0 호출 조건은 `_permission_needed`; 시험 이중체(`test_core_truth_split` 의 층, `test_real_teleop_parts` 의 core stub)를 새 인자·도우미에 맞추고 예전 `direction_unsure` 네 경우를 방향별 규칙으로 바꿈; 2 회차 이후는 모두 시험 보강(요청 0 의 실제 호출, `t`–`t + W` 사이의 층·FREE·복셀 만료, 요청 방향만 막힘과 `acting`, 양의 D5 허용치 부족, 생성기 없음, 질의 속도). rig 에서는 우회 정지 자리에서 차 앞이 이미 영역 안이라 영역 밖 장애물은 차 옆에서 두 방향을 막음 — 요청 방향만 막히는 경우는 코어 단위 시험.
- Codex 5d 7 절 1–8 회차(새 스레드 `01a1f3d4`, `4ed5d74f` … `e79ba2e4`): 1 회차 P1 2·P2 4, 2 회차 P1 3·P2 3, 3 회차 P1 1·P2 2, 4 회차 P1 1·P2 2, 5 회차 P1 2·P2 1, 6 회차 P1 2·P2 1, 7 회차 P1 1·P2 1, 8 회차 **새 P1/P2 없음**(수렴). 정한 것: D5 우회는 포켓 검사기 자리의 우회 검사기(등록 제출 자세 ± `PocketCheck` 띠, 판정 늘 통과) — 설치·해제·삽입·`acting` 은 지금 경로; 실차 포켓 구간의 질의는 방향 수와 무관하게 모든 가능한 방향의 운동을 덮어야 함(층의 요청 방향도 가능한 속도로 다시 물음); 새 속성 읽기는 실차 판정 뒤(재생 기록 불변); 질의는 쓰기 여유 `t + W`(35 ms, 층은 증거 시각만), 송신 관문 끝에서 `min(ns, 결정 + W)` 를 다시 보고 정지·대기 틱은 비영 이어 쓰기 거절(4c 관문 두 곳, 총괄 승인). 철회: 5판의 송신 순간 재질의(실측 15.6–27.7 ms). PR 7 선행 조건: 실 검사기 틱이 25 ms 안. 6a: 마지막 시계 읽기부터 실제 쓰기까지의 지연 상한, 반쪽 프레임 폐기.
- Codex 5c-2 구현 검토 1–7 회차(새 스레드 `01a1f38e`, `ace584c7` … `9321857f`): 1 회차 P1 2·P2 6, 2 회차 P1 3·P2 3, 3 회차 P1 2·P2 2, 4 회차 P1 1·P2 1, 5 회차 P1 1·P2 1, 6 회차 P2 2, 7 회차 **새 P1/P2 없음**(수렴). 구현에서 정한 것: seed 와 세대 닫기는 채널 잠금 안에서 지금 시계로 다시 판정한다(lease·대기 중 연결 종료, MCU 링크 신선도, 층 작업자·watchdog, 3 s 한도, 확인의 SLAM 나이); 처리된 DISARM 의 수가 바뀌면 시도 무효, 채널에 대기 중인 STOP·DISARM 은 commit 을 보류(아비터가 거절하면 이어 감), 이 묶음의 DISARM·대기 중 STOP/DISARM 이 있으면 접수 거절 `emergency_pending`; 관전자 연결은 무효 사유가 아님; 선언 표식·0.06 상한·`relocalize_seeded` 기록은 seed 직후; 층이 없으면 세대를 닫지 않음; 출처는 hold 가 아니면 묶음의 마지막 processed; `real_status.standing` 도 지금 시계의 링크를 봄. 물리적 밀기는 rig 에서 ARM 시한(0.3 s)·과속 감시(0.225 m/s)가 먼저 래치하거나, ARM 150 ms 지연·1.5 mm/20 ms 로 밀면 송신 관문이 `first_byte:not_at_start` 로 막음(시험).
- Codex 5c-2 6 절 1–14 회차(같은 스레드 `01a1f343`, `ca49883d` … `40d49511`): 1 회차 P1 2·P2 7, 2 회차 P1 3·P2 6, 3 회차 P1 2·P2 5, 4 회차 P1 1·P2 4, 5 회차 P1 1·P2 2, 6 회차 P1 1·P2 2, 7 회차 P2 2, 8 회차 P2 4, 9 회차 P2 3, 10 회차 P1 1·P2 3, 11 회차 P2 1, 12 회차 P2 2, 13 회차 P2 1, 14 회차 **새 P1/P2 없음**(수렴). 정한 것: 확인·선언을 하나의 시도 틀로(접수 때 층·이력 다시 자름, lease·제어 세대·우회 집합 결박); 원천 회차와 회차별 신선도 증거; 세대 열림 중 추적기 `warming_up` 유지; 출력이 닫힌 동안 자세 읽기는 나이·hold 한도 예외 없이(한도는 루프 검사가 세대로); 아비터·재무장의 정지 증거는 물리적 정지; 시작 자세는 첫 비영 DRIVE 의 송신 관문까지; `slam` 우회는 4 회차 P1 로 정지 판정에 손대지 않음. 철회한 것: 3판 `_still` 의 `slam` 항 우회, 4판 닫힌 리셋의 마감 비우기, 6판 낡은 응답 거르기, 8판 '자세 없음 틱'.
- 총괄 결정(2026-11-19, 5c-2): 운용자 선언 우회 7b 추가(시작 자세 하나, 정지 중, 표시·기록, 0.06 m/s 잠정 상한 — S10 속도 승인 목록, F1 예외는 PR 7); `slam` 우회의 뜻은 '마지막 적용 보정 + 오도메트리 주행 허용'(10 회차 P1 의 처리, 좁히지 않음), 우회 중 0.06 m/s 상한과 '우회 중: slam — 오도메트리 주행' 표시; 출발에는 신선한 `processed` 한 번(채택·ARM 정지와 hold 해제가 같은 값); 예외 틱 콘솔 `invalid` 정산은 6a 후속.
- Codex 적대 1 회차(1판): P1 6(팔레트 분리가 실물체를 숨김, 스캔 자세의 보정 이중 적용, 보정 세대만으로 설치, 첫 바이트 대기와 ARM, IMU 표본 판정, 재위치 증거의 새로움), P2 4(스캔 주도 갱신·계획 경계, 미션 전 스캔 재분류, 콘솔 요청 경로, 잠금 범위), P3 1(속성 표·refresh 서명). 모두 2판의 1–8 절에 반영.
- Codex 적대 2 회차(2판): P1 4(오도메트리 계측 시각, 면제 영역 안 물체·승강 공간, 재위치 확인의 현재 자세, 잠금 디렉터리·재연결), P2 6(갱신 시작 곳·두 방향·정지 뒤 방향 판정, 설치 이동량·첫 사용 지연, 계획 격자 결합·채택 전 판정, IMU 공백·기준, 콘솔 승강 확인·상태 발행, 재위치 증거 폐기 범위·응답 출처), P3 1(새 요청 분류). 3판의 1–8 절에 반영. 승강 공간은 S10-A 감독 배치 조건과 선행 조건으로.
- Codex 적대 3 회차(3판): P1 4(정지 증거로 속도 0 — 역방향 검사 소실, B 의 정지 모형·증거 나이 반영, 승강 목표 실제값, 재위치 확인 시점), P2 7(출발 지속, 기어 구간, 계획 답의 격자 세대, 채택 전 고장의 래치, processed 출처, 잠금 순서, MCU 시각 간격 보존), P3 1(설치 정의의 방향). 4판의 1·2·3·6·7·8 절에 반영. 정지 뒤 출발은 '두 방향 질의' 로 바꿨다(계획의 포켓 구간 조건과 같은 규칙).
- Codex 적대 4 회차(4판): P1 4(역방향 D5 의 잔류 운동 hold, B 와 격자 FREE 나이 결합, 시각 오차 주장, 제출 재검사의 순서), P2 5(정지 프레임 관문 뒤 양방향, 기준점 갱신 DISARM, 채택 전 미션 수거, 보류 요청의 권한·확정, 원천 세대 유지), P3 1. 5판에 반영: 시각은 점 변환 대신 구간 덮기, 구현은 5a–5d 단계로 나눔.
- Codex 적대 5 회차(5판): P1 4(D5 질의 속도, 요청 0 일 때 D5 호출 누락, 스캔 전체의 자세 증거, 제출과 첫 계획 사이), P2 2(접근 속도의 회전 중 이동·마감 기점, 정지한 차의 재위치 확인 응답), P3 3. 6판에 반영: 스캔은 텔레옵 브리지처럼 오차표에 시각 오차를 더해 놓고, 재위치는 확인용 processed 를 요청한다.
- Codex 적대 6 회차(6판): P1 4(관문이 닫힌 틱의 옛 비영 DRIVE — 4c 의 빈틈, 스캔 오차표의 v_max 가정, 같은 부팅의 오도메트리 단절, 확인용 응답의 입력 시각), P2 2(기준 응답과 최신 출처의 충돌, 첫 빔부터 제동까지의 지연), P3 2(두 방향 질의 인자 기록 시험, watchdog 두 시각 시험 — 9 절에 추가). 7판에 반영.
- Codex 적대 7 회차(7판): P1 1(반쯤 쓴 옛 DRIVE — 단독 수정 PR 에 반영), P2 3(스캔 속도 판정과 정상 접근, 버린 스캔의 마감, 재위치 확인 중 층 대기 — 5c 로), P3 1(같은 부팅 단절의 재위치 원인·권한 중 경로 — 5c 시험). 8판: 5a 범위만 반영하고 5b–5d 는 미수렴으로 표시(총괄).
- Codex 적대 8 회차(8판, 5a 만): P1 없음, P2 3(채택과 오차표의 같은 v_scan, 도착하지 않은 스캔의 마감, D5 면제의 설치 주체), P3 1(마감 해소는 스캔 ID 로). 9판에 반영.
- Codex 적대 9 회차(9판, 5a 만): P1 없음, P2 1(지난 예상 마감은 결과 설치까지 유지), P3 1(마감 초과 뒤 해소 시험). 10판에 반영.
- Codex 5a 구현 검토 1–8 회차(같은 스레드, `fb356eef` … `27a3726a`): 1 회차 P1 4·P2 7, 2 회차 P1 3·P2 5, 3 회차 P1 1·P2 3, 4 회차 P1 1·P2 3, 5 회차 P1 1·P2 2, 6 회차 P2 2, 7 회차 P2 1, 8 회차 새 P1/P2 없음. 구현에서 정한 것: 첫 바이트가 층 watchdog·`obstacle_act`·층 동일성을 다시 읽음; 늦은 수신은 지난 예상 마감을 지우지 않고 첫 사용 지연은 마지막 빔 + B 로 스캔마다 한 번; watchdog 끝은 설치 전에 읽고 래치 때까지 고정; 같은 부팅 단절마다 재위치 세대 하나(빈 틱은 단절을 풀지 않음), 재부팅 뒤 세션 확인·지연 기준 재시작; 스캔 이동 판정은 엔코더 상한(가속 dt/2 + 양자화)으로 첫 빔 − dt 부터; 경로 검사는 작업자에서, 경로 길이로 차감; 계획 열쇠에 팔레트 자세; ARM 전(채택 뒤 포함) 어떤 래치든 미션 종료, ARMING 중 고장도 래치; 원천의 명시적 단절은 즉시 끝이고 다시 연결될 때까지 층 미준비; 거리 오차 모형은 YAML(`a2_spec`)을 따르고 계획 분리는 반사마다 자기 거리 오차만큼 키움(`suppress` 여섯째 원소, 기본 호출은 바이트 불변).
- Codex 5c-1b 13 회차(5.1 13판, 현행 Chrome 범위): **새 P1/P2 없음**(수렴). P3 1(재연결 직후 첫 상태 전 옛 값) 반영.
- Codex 5c-1b 12 회차(5.1 12판, 현행 Chrome 범위): P1 없음, P2 1(터치 탭 허용 거리 안에서 단추 밖으로 뗀 탭이 전송됨 — 브라우저가 탭으로 판정한 누름이고 누름 문맥 검사는 그대로라 의도한 동작으로 설계에 명시, 코드 변경 없음), P3 1(연결 끊김 뒤 마지막 값 표시) 반영.
- Codex 5c-1b 11 회차(5.1 11판 + 구현): P1 없음, P2 1(id 없는 합성 click 이 눌린 포인터 기록을 먼저 씀 — 미지원 브라우저 경로라 범위 밖), P3 1(차단 중 거절된 Enter 기록) 반영.
- 범위 결정(총괄 2026-11-19): 지원 브라우저 = 현행 Chrome(터치 포함); `pointerId` 없는 click 브라우저의 다중 포인터 경계는 미지원·수렴 대상 밖(서버가 세대·lease 최종 판정), 미지원이면 경고 띠.
- Codex 5c-1b 10 회차(5.1 10판 + 구현): P1 없음, P2 2(모호한 click 을 두 번 판정, 무효화된 누름의 뒤 click 통과). 11판: 한 번 판정·포인터 클릭은 자기 기록 필수·모호하면 새 누름까지 막음.
- Codex 5c-1b 9 회차(5.1 9판 + 구현): P1 없음, P2 1(모호한 click 뒤 옛 기록이 정상 탭을 막음), P3 없음. 10판에 반영.
- Codex 5c-1b 8 회차(5.1 8판 + 구현): P1 없음, P2 1(id 없는 click 의 모호한 매칭), P3 1(ack 대기 중 입력 변경). 9판에 반영.
- Codex 5c-1b 7 회차(5.1 7판 + 구현): P1 없음, P2 2(등록 뒤 고친 자세, 포인터 id 없는 click), P3 1(콜백 폐기의 안내). 8판에 반영.
- Codex 5c-1b 6 회차(5.1 6판 + 구현): P1 없음, P2 2(누름이 그리지 않은 최신 세대를 기록, 재무장의 반복 Enter), P3 2(거절된 클릭의 확인 기록, 새 세션 첫 렌더 전 옛 우회 응답). 7판에 반영.
- Codex 5c-1b 5 회차(5.1 5판 + 구현): P1 없음, P2 1(두 번째 STOP 전에 누르기 시작한 재무장), P3 2(취소된 누름 기록, 옛 우회 응답의 늦은 안내). 6판에 반영: 모든 단추의 누름 시작 문맥.
- Codex 5c-1b 4 회차(5.1 4판 + 구현): P1 없음, P2 2(두 번째 STOP 뒤 옛 재무장 단추, 같은 단추의 두 번째 포인터), P3 1(버린 전환 안내). 5판에 반영.
- Codex 5c-1b 3 회차(5.1 3판 + 구현): P1 1(낮은 데스크톱 창에서 상단 STOP 을 가림), P2 2(상태 객체 참조 비교로 정상 클릭 거절, 두 포인터가 누름 기록 공유), P3 2(이전 세션 전환을 성공으로 표시, 반납 시험 응답). 4판에 반영.
- Codex 5c-1b 2 회차(5.1 2판 + 구현): P1 1(폰에서 패널이 하단 STOP 을 가림), P2 5(Enter 반복, lease 재획득 직후 옛 상태, 우회 해제 직후 제출, 목록 revision, 첫 목록 거절), P3 3(Space·안내·반납 시험). 3판에 반영.
- Codex 5c-1b 1 회차(5.1 1판 + 구현): P1 없음, P2 6(누르는 중 다른 요청에 답, 복구 번호·권한의 시점, 우회 해제 적용 전 제출, 새 세션의 옛 등록 revision, 첫 목록 없음, 래치에서 재무장 단추 없음), P3 3(`not_ready:` 문구·툴팁, 패널 위치, 재획득 시험). 2판에 반영(`real_status.mode` 추가).
- Codex 5c-1 10 회차(29판 + 구현): **새 P1/P2 없음**(수렴). P3 1(상태 불변 시험에 확인 답·시작 확인) 반영.
- Codex 5c-1 9 회차(28판 + 구현): P1 없음, P2 1(틱 중 lease 만료 뒤 확인 답·등록 반영), P3 1(잠금 시계 시험). 29판: 모든 실차 요청이 자기 시계로 lease 먼저, 잠금 대기 중에만 시계가 가는 시험.
- Codex 5c-1 8 회차(27판 + 구현): P1 없음, P2 1(잠금 대기 중 lease 만료), P3 1(옛 연결 시험 순서). 28판에 반영.
- Codex 5c-1 7 회차(26판 + 구현): P1 없음, P2 2(연결 큐 확인과 적용 사이 경쟁, 틱 중 lease 만료 뒤 복구 완료), P3 1(옛 연결 끊김이 새 연결 우회를 버림). 27판에 반영. 채널 형식 오류 제외 결정은 Codex 동의.
- Codex 5c-1 6 회차(25판 + 구현, 재부팅 뒤 새 스레드): P1 없음, P2 3(같은 클라이언트 재연결 대기 중 우회 적용, 채널의 콘솔 DISARM 대기 중 복구 완료, 준비 판정의 `snapshot` 우회가 다른 보정 값 통과), P3 2(받을 때 시계, 채널 즉시 거절의 `last_refusal`). 26판에 P2 3·P3-1 반영; 채널이 중재기 전에 내는 형식 오류(`invalid`)는 `last_refusal` 에 싣지 않는다(형식 오류는 콘솔이 응답으로 이미 받음).
- Codex 5c-1 5 회차(24판 + 구현): P1 없음, P2 1(적용 때 lease 만료·연결 끊김 대기 미반영), P3 2(버린 기록의 켬/끔, 중재기 거절의 `last_refusal`). 25판에 반영.
- Codex 5c-1 4 회차(23판): P1 1(우회 해제 대기 중 받아들인 미션이 확인 없이 ARM), P2 2(대기 중 전환이 lease 반납 뒤 적용, DISARM 응답의 부팅 비교), P3 1(우회 기록의 lease). 24판: 전환은 미션 없음·같은 lease 일 때만(받을 때·적용할 때), 부팅 비교, 기록에 lease.
- Codex 5c-1 3 회차(22판): P1 1(`snapshot` 우회가 옛 보정 격자와 새 보정 자세를 섞음), P2 2(우회 해제와 제출이 같은 틱, 복구 완료의 틱 시작 시각), P3 1(`real_status` 의 lease 보유자). 23판: 보정 값이 같을 때만, 전환은 다음 틱 머리에 적용·대기 중 ARM 없음, 콜백에서 시계 다시 읽기, `lease.holder`.
- Codex 5c-1 2 회차(21판): P1 없음, P2 7(같은 묶음 콘솔 DISARM 처리 전 복구 완료, `operator_start` 우회 중 남은 확인, `snapshot` 우회와 `installing()`, 원장 정리 꺼짐, 브라우저 등록 뒤 revision·lease 동기화·확인 단추 묶기), P3 2(`stale_epoch`, 큰 정수 좌표). 22판: 서버 쪽 반영, 브라우저 3 건은 5c-1b(총괄 분할 승인). 같은 묶음 시작 확인+제출의 보수적 거절은 Codex 동의.
- Codex 5c-1 1 회차(20판, 5 절만): P1 1(무장 중 IMU 우회 해제가 기준 없는 감시를 남김), P2 8(공유 validate 기본값 변경, 토큰 원장 필수 아님, 같은 확인에 두 답, 앵커 DISARM 대기 중 복구 완료, 시작 확인과 제출의 시점·무효화 호출점, 전류 우회가 바퀴를 가림, 층 우회 적용점, 콘솔이 `real_status` 를 안 봄), P3 2(세션 종료의 우회 폐기, 미등록 ID). 21판에 반영. 같은 묶음의 시작 확인+제출은 보수적 거절로 둠(공유 명령 코드 불변).
- Codex 5b 9 회차(19판 + 구현): **새 P1/P2 없음**(수렴). P3 1(다른 IMU 숫자 설정의 큰 정수) 반영.
- Codex 5b 8 회차(18판 + 구현): P1 없음, P2 1(READY 경계가 조회 순서로 갈림), P3 1(`level_ref` 큰 정수). 19판에 반영: 경계는 취득 시각(`settled(경계)`), 큰 정수 거부.
- Codex 5b 7 회차(17판 + 구현): P1 없음, P2 2(READY 틱에 ARMING 표본을 무장 규칙으로 판정, `level_ref` 매핑을 키로 해석). 18판에 반영: 판정 뒤 `settled()`, `level_ref` 형식 검증.
- Codex 5b 6 회차(16판 + 구현): P1 2(참조 확정 전 공백이 관측한 자세를 지움, 정지 판정 한 틱 상실이 자세 변화 표시를 지움), P2 2(ARM 첫 바이트에서 확인한 움직임을 지속 판정으로 넘김, 순간 이상치 하나가 재무장 뒤에도 래치), P3 1(정밀도 시험 경로). 17판: 정지 구간 이력 추적(`stand`)을 버리고 **수평 참조**(`level_ref`·`level_tol_deg`)로 선 자리를 확인 — 이력에서 나온 P1·P2-2 가 함께 없어짐; ARM 응답 전 움직임은 즉시 `watch_imu_moved`.
- Codex 5b 5 회차(15판 + 구현): P1 1(정지가 2 s 넘게 이어지면 보관 창 밖 옛 자세를 잊음), P2 1(시각의 float 변환이 정밀도를 잃거나 넘침). 16판에 반영: 정지 구간의 참조·자세 변화 표시를 창과 따로, 시각은 정수만.
- Codex 5b 4 회차(14판 + 구현): P1 1(정지 중 짧은 단절 뒤 바뀐 자세가 새 기준이 됨), P2 2(`required` 가 조립 결과째 전달되지 않음, 비유한 취득 시각이 래치), P3 1(시그니처 두 곳). 15판에 반영: 기준 일관성은 정지 구간 전체 표본, `RealLoop(watches=)` 만, 비유한 시각 표본만 버림.
- Codex 5b 3 회차(13판): P1 없음, P2 3(ARM 호출부의 `now` — 기준 마지막 표본으로, IMU 대기가 기존 ARM 전 거부를 늦춤, 조회 중 도착한 표본을 미래로 버림). 14판에 반영: `arm((방향, last_ns))`, 기존 거부 먼저(`not_still` 만 대기 규칙), 원천 `upto_ns` 계약과 조회 직전 시계.
- Codex 5b 2 회차(12판): P1 2(지속 시간이 표본 보관 창보다 길면 기울기 미검출, 첫 바이트 읽기 예산 밖의 IMU 판정), P2 3(기준 고정 직전 표본이 공백을 숨김, 기준 대기의 기한과 `adopted_for` 순서, 원천 반환 순서), P3 1(`build_watches` 의 바퀴 반경). 13판에 반영.
- Codex 5b 1 회차(11판, 4 절만): P1 3(드문드문 오는 표본에서 기울기 무력화, 섞인·반대 자세의 기준, 원천이 막으면 DISARM 도 멈춤), P2 4(정지 첫 틱의 기준 부족 래치, 첫 바이트 뒤 도착 표본, `required` 강제 경로, 판정 불가 설정), P3 1(`norm_tol` 단위). 12판에 반영: `watch_imu_gap`, 기준 일관성 판정과 ARM 전 고정, 원천 비차단 계약, 기준 축적 대기, 첫 바이트 전 표본 갱신, 건식·실행 조립 공통 `build_watches`·`RealLoop` 생성 거부, `imu:` 설정 범위, `norm_tol_frac`.
- 총괄 결정(2026-11-19): 계획 격자만 분리, 텔레옵 잠금(조건 a–c, 11-20 전 머지 못 하면 텔레옵만 연기), IMU required 잠정, 선행 조건 이관.
- Codex 새 스레드 전제 검토(1판, 위를 보지 않음): 12 개. 정지 거리·빔 대표성·시계 오차·IMU 임계값·Jetson 부하는 범위 절의 S10 선행 조건으로, `+Inf`·빔 유효성(1.1), 결과 열쇠(1.2), 셀별 나이(1.2, 코드 확인), IMU 기준·표본(4), 재위치 입력 시각(6), 잠금 식별(8), 합치기 마감(1.2)은 본문에 반영.
