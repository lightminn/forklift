# 상시 운용 세션의 SLAM — 미리 만든 지도에서 위치 추정 (계획 v2.2)

작성일: 2026-10-20 · 상위: [시뮬레이션 실시간화](2026-10-17-realtime-sim.md) D9(SLAM 비동기),
[상시 운용 콘솔](2026-10-18-persistent-live-console.md) U16(2 h 연속 운용 미충족)·D9(감독 에폭),
선행 설계: [SLAM 운용 조건](https://github.com/lightminn/forklift/blob/feat/fleet-five/docs/plans/2026-10-08-slam-operating-conditions.md)
(`feat/fleet-five` bbbcb03, 재생 전용, 미실행) · [온라인 SLAM 폐루프](2026-10-04-online-slam-closed-loop.md) v3.8(지도 먼저 초안 보류)

## 1. 문제

상시 운용 콘솔은 한 세션을 몇 시간씩 띄워 두고 같은 경로로 임무를 되풀이한다. U16(2 h 연속, 배속 0.3)은 임무 7개를 마친 뒤 1.5 h 에
`slam_link_failed: scan 15090: no reply within 1.00 s` 로 끝났다. slam_toolbox 가 keyframe 하나를 처리하는 시간이 세션이 쌓일수록 늘어 D9a 의
응답 기한 1 s 를 넘었다. 기한을 늘리면 실패가 뒤로 밀릴 뿐이다. **요구: keyframe 하나의 처리 비용이 세션 길이에 묶이지 않을 것**(콘솔 세션 요청,
2026-10-19). 이 계획에서는 "sim 시간 8 h 분량(배속 1.0 이면 벽시계 8 h)에서 기한까지 여유가 충분하고, 남는 증가가 측정으로 확인된 크기 이하"로 판정한다(2.2 의 남는 선형 항
때문). 콘솔 기본 배속 0.4 의 벽시계 8 h 는 sim 3.2 h 라 이 기준에 들어간다.

별개로 U13(짧은 세션, 두 lane 동시)도 `slam_link_failed` 로 끝났다. 처리 비용은 작았고(최대 53 ms) 브리지가 scan 620 에 답하지 못한 채 끝났다 —
CPU 경합이 유력하다(2.5). 같은 계획에서 별도 항목(L5)으로 다룬다.

## 2. 원자료에서 잰 것 (2026-10-19–20)

### 2.1 U16 기록 (ws1 `test_runs/live/pc_uc_u16_pc11_14a7db6/bridge/`)

- 10 Hz scan 15,091 개 중 slam_toolbox 가 처리한 keyframe 879 개(`processed`), 나머지는 브리지가 보내지 않은 `local` 14,209 개(처리 0.8 ms).
- keyframe 처리 시간(`records.jsonl` `wall_s`, publish → /pose 수신) 80 개 구간 평균/최대: 처음 0.043/0.079 s → 마지막 0.380/1.267 s.
  시간에 단조롭지 않다(중간 구간 최대 0.896 → 0.361 s).
- 1.5 m 안 앞선 keyframe 수별 평균: 20 미만 0.080 s, 20–50 0.110, 50–100 0.180, 100–200 0.397 s. 상관: 근처 수 0.44, 순번 0.36.
- Ceres 최적화(승인된 루프 폐쇄마다 한 번) 94 회. 최적화 1.5 s 안의 keyframe 평균 0.339 s, 나머지 0.105 s.
- 끝의 map←odom 이 (3.71 m, 0.63 m, 0.945 rad) — U16 의 오도메트리는 정답에서 크게 표류했고, SLAM 보정이 그만큼을 메우고 있었다.

### 2.2 slam_toolbox 2.8.5 소스 (GitHub 태그 2.8.5)

- **지도 재생성.** `updateMap()`(`slam_toolbox_common.cpp` 646–670)은 `addScan()`(839)과 같은 `smapper_mutex_` 를 잡고,
  `CreateFromScans(GetAllProcessedScans())`(`slam_mapper.cpp` 63–70)로 처리한 모든 scan 에서 점유 격자를 다시 만든다. `map_update_interval`
  (우리 1.0 s, 벽시계 `rclcpp::Rate` — 배속 0.3 이면 sim 1 s 에 3.3 번)마다 돌고, `/map` 구독자가 없으면 건너뛴다(649). 브리지가 구독하므로 늘 돈다.
  격자 원점은 scan 경계 상자의 최솟값이라(`Karto.h` 6090–6113) 다시 만들 때마다 바뀔 수 있다.
- **localization 처리.** 첫 scan 은 `ProcessAgainstNodesNearBy` 로 저장 노드 근처에 맞추고(이 경로는 정합 응답을 성공 조건으로 검사하지 않는다,
  `Mapper.cpp` 2819–2829), 이후는 `ProcessLocalization`(2868–2945): 최근 scan 과 순차 정합 → 노드 추가 → `AddEdges` → `do_loop_closing` 이면
  `TryCloseLoop`. 지도로 끌어당기는 경로는 둘이다. ① `AddEdges` 안의 `LinkNearChains`: 근처 체인(저장 scan 포함)과 맞춘 결과를 가중 평균으로 현재
  자세에 넣는다(1492–1496, 1639–1659). 체인이 `loop_match_minimum_chain_size` 개 이상이어야 한다 — 우리 값 10 은 조사 주행 한 번 통과(1.5 m 안 약 7 개,
  v3.8 기록)로는 채워지지 않는다. ② 루프 폐쇄와 그 Ceres 최적화.
- **크기.** 새 노드는 `scan_buffer_size` 를 넘으면 가장 오래된 것부터 그래프·최적화기에서 지워진다(`AddScanToLocalizationBuffer` 2948–2971). 살아 있는
  노드는 저장 N + 버퍼 B 로 묶이고, 재생성 대상 scan 도 N + B 로 묶인다. Ceres 는 첫 노드만 고정하고(`ceres_solver.cpp` 237–239) 나머지 저장 노드는
  움직일 수 있다. 반복 횟수는 따로 정하지 않아 Ceres 기본값을 쓰고, `num_threads = 50`(빌드 상한 24)이다.
- **남는 선형 항(Codex 검토로 발견).** `FindNearChains` 는 근처 scan 마다 그 ID 에서 0 쪽으로 정수 ID 를 하나씩 거꾸로 훑고, 지워진 ID(NULL)는
  `continue` 로 넘어간다(`Mapper.cpp` 1713–1727). 세션 동안 지워진 localization scan 수만큼 조회가 늘어난다 — 노드 수가 묶여도 keyframe 하나의 연산 수는
  세션 길이에 비례한다. 한 번에 드는 것은 조회 하나라 sim 8 h(U16 비율 879 keyframe / sim 1,509 s 로 약 16,700 keyframe)에서도 작을 것으로 보지만
  재지 않았다(V1′ 에서 잰다).
- **시작.** `map_file_name`·`map_start_pose`(저장 지도의 map 좌표에서 base 자세)로 저장 pose graph 를 읽는다. 실행 파일은 `localization_slam_toolbox_node`.
  시작 drop 은 공통 `shouldProcessScan()` 이라 우리 0 문턱 설정에서 1·5 번 처리, 2–4 번 drop 이 mapping 과 같다(Codex 확인) — 브리지·tracker 의 시작 규칙은
  바꿀 필요가 없을 것으로 본다(S1 에서 확인). upstream 기본값은 `scan_buffer_size: 3`·`loop_match_minimum_chain_size: 3`.

### 2.3 SR1b — U16 keyframe 재생으로 원인 나누기 (ws1 `test_runs/live/sr1b/`, CPU 전용)

U16 의 10 Hz scan 전부를 같은 stamp 와 U16 의 실제 오도메트리(keyframe 사이 보간)로 실제 pc11 브리지 + slam_toolbox 에 보냈다. 거리값은 U16 의 마지막 지도에서
U16 의 SLAM 자세(보간)로 ray-cast 했고(잡음 없음), 브리지 컨테이너는 코어 하나를 전용으로 썼다. keyframe 854 개, slam_wall_s(초):

| 설정 | 처음 10 % 평균 | 마지막 10 % 평균 | 최대 | 근처 100–200 개 평균 |
|---|---|---|---|---|
| 운용 설정 | 0.030 | 0.223 | 0.871 | 0.292 |
| 지도 재생성 끔(`map_update_interval` 1e5) | 0.028 | 0.120 | 0.721 | 0.169 |
| 루프 폐쇄 끔 | 0.029 | 0.117 | 0.370 | 0.097 |
| 둘 다 끔 | 0.028 | 0.034 | 0.062 | 0.043 |

증가는 루프 폐쇄와 지도 재생성이 겹친 것이다. 하나만 끄면 절반쯤 줄지만 계속 커지고, 둘 다 끄면 2 h 분량 내내 평평하다. 합성 scan·전용 코어라 절대값보다 몫을 읽는다.

### 2.4 지금 시스템 (main f20b46e)

- 에폭마다 브리지 컨테이너와 slam_toolbox 가 새로 뜨고 지도는 빈 상태에서 시작한다. 저장·읽기 경로가 없다(`tools/live_supervisor.py` 7–14, `tools/slam_bridge_container.sh`).
- 시작 자세는 장면의 `scenario.start_rear` 이고 "odom = map = world"(`run_transport.py` 4446–4451). 목표·픽업·목적지·home 은 world 좌표로 등록된다.
  보정 뒤의 재표현은 호출하는 쪽이 callback 을 넘길 때만 한다(`targets.py` 122–133, `core.py` 3496–3519) — map = world 가 깨지면 이 코드가 알아서 메워 주지 않는다.
- 재생 도구는 오도메트리를 늘 기록 출발 base 자세 = 원점에서 시작한다(`slam_replay.py` 109–151, rear 초기값 `(rear_x, 0, 0)`). 재생으로 만든 지도의 map
  원점은 그래서 "기록 출발 base 자세"이고 world 가 아니다.
- SLAM 지도는 계획에만 쓰인다: `latest_map.npz` → planning memory 의 정적 층(`run_transport.py` 3934–3974, `planning_memory.py` 157–207), holding 중에는 쓰지 않는다.
  더 새로운 관측의 비움이 SLAM 칸을 지운다(`occupied()` 240–251). 그러나 같은 칸이라도 원점이 바뀐 지도가 오면 새 점유로 보고 새 날짜를 준다(169–184) —
  비운 칸이 되살아난다(Codex CPU 반례: 원점 1 mm 이동으로 재점유). 주행 허가는 SLAM 지도를 보지 않는다.
- 운용자 상태 메시지에 SLAM 모드·지도 ID 필드가 없다(`state.py` 34–99; 위치의 `source` 와 `correction_version` 만). 결과 기록은 에폭마다 새로 생기고 최근
  100 건만 보낸다(`results.py` 16–22, `live_runtime.py` 73–78).
- 브리지 컨테이너는 작업의 첫 CPU 하나에 고정된다(`deploy/console/isaac_live.sbatch` 27·40–42, ws1 `test_runs/live/live.sbatch` 같음).
- runner 의 새 인자는 근접 계약(`sim/isaac/near_tracking.py` 1150–1166 `RUN_SPECIFIC`, 1203–1216 `POST_MEASUREMENT`)과 boundary replay 의 인자 복원
  (`tools/boundary_replay.py` 190–240, argparse 를 다시 돌리지 않는다)을 거친다.

### 2.5 U13 (ws1 `test_runs/live/pc_uc_u13_pc10_26f9a18/`)

processed keyframe 23 개, 최대 0.053 s, 루프 폐쇄 없음. 브리지 기록에 scan 620 이 없다(답을 쓰기 전에 연결이 닫힘). runner 종료 기록 `pacing.stalls: 1`,
카메라 지연 최대 1.31 s. 두 프로세스가 함께 1 s 넘게 멈춘 것으로 읽히고, 코어 하나에 묶인 브리지가 붐비는 코어에서 굶었다는 해석이 유력하다(노드 load 약 32/20,
Slurm 밖 도커 컨테이너는 아무 코어나 쓴다). tick trace 가 없는 실행이라 확정은 못 한다(가설).

### 2.6 앞선 시도

- v3.8 의 "지도 먼저" 초안(45852fb)은 네 이유로 보류됐다(`2026-10-04-online-slam-closed-loop.md` 79–83): ① 조사 지도의 목적지 world 정확도 미측정
  ② mapping 모드는 이전 그래프에 계속 맞추지 않음 ③ 역직렬화 실패가 ERROR 만 남기고 새 지도로 계속 돌 수 있음 ④ 운반 끝 고정 창의 보정이 하강에 안 쓰임.
  ④는 v3.8 의 하역 직선 정합(D7c)이 따로 해결했다. 이 계획의 답: ① V2·V5, ② localization 모드, ③ L4.
- `feat/fleet-five` bbbcb03: 재생 전용 저장 지도 도구(`serialize_map` 호출, `tools/slam_localization.py` 의 `mode`·`map_file_name`·`map_start_pose` 치환,
  `isaac_slam_localization.launch.py`). live 브리지에는 연결되지 않았고 계획 O1–O6 은 돌지 않았다.

## 3. 결정 (사용자·콘솔 세션 확인 전 잠정안)

- **L1 운용 방식 — 장면마다 미리 만든 지도에서 처음부터 위치 추정.** live 세션은 세션 시작부터 localization 모드로 돈다. 세션 도중 mapping → localization
  으로 바꾸지 않는다. 이유: 고정 시점·노드 재시작·좌표 연속성 문제가 생기지 않고, 에폭이 같은 장면(같은 SEED)을 다시 만들므로 같은 지도를 그대로 쓴다.
  실차 운용도 "창고 지도를 한 번 만들고 위치만 추정"이 통상이다. mapping 모드는 플래그가 없을 때의 기본으로 남긴다(L7).
- **L2 조사 지도와 좌표.** live 와 같은 기지 장면·차체 URDF 로 조사 주행(`sim/isaac/run_slam_drive.py`, 짐을 옮기지 않음)을 따로 돌려 기록하고, 그 기록을
  mapping 으로 재생한 끝에 `/slam_toolbox/serialize_map` 으로 `.posegraph`·`.data` 를 저장한다.
  - **map = world 를 재생 쪽에서 만든다.** 재생 오도메트리의 초기 자세를 기록의 첫 world base 자세로 두는 선택 인자를 `slam_replay` 에 더한다(기본값은 지금처럼
    원점 — replay gate·기존 재생 불변). 그러면 조사 지도의 map = world 이고, live 의 `map_start_pose` = 출발 base 자세(world)다. 조사 출발점이 live 출발과 같을
    필요는 없다(명시적 world 배치가 있으므로).
  - 경로는 임무가 지나는 곳(출발·통로·픽업 구역·목적지·home)을 덮어야 한다 — 덮개 검사: 임무 경로 점이 모두 조사 keyframe 3 m 안(LiDAR 가 360° 라 방향 조건은 두지 않는다).
  - **움직이는 짐은 조사 장면에서 뺀다**(가능하면): 조사 때 있던 높은 짐(LiDAR 평면 1.05 m 에 닿는 것)이 운용 중 옮겨지면 지도와 어긋난다. 장면 생성이 짐을 뺄 수
    없으면 그 사실을 매니페스트에 적고 V5 가 그 영향을 판정한다. 바닥 팔레트(0.144 m)는 LiDAR 평면 아래라 해당하지 않는다.
  - 지도 묶음 = `.posegraph`·`.data` + 조사 때의 점유 격자(npz, 원점 고정) + 매니페스트: 세 파일의 SHA-256 으로 만든 `map_id`, 장면·URDF·설정 파일
    SHA-256, SEED, map←world(= 항등, 재생 배치로 확인), slam_toolbox 버전, 노드 수 N.
- **L3 위치 추정 설정과 지도 층.**
  - mapping 설정 파일을 그대로 쓰고 `mode: localization`·`map_file_name`·`map_start_pose` 만 바꾼다. `scan_buffer_size`·`loop_match_minimum_chain_size` 는
    {10/10(지금), 3/3(upstream)}, `do_loop_closing` 은 {켬, 끔} 가운데 S1 결과로 정한다. 2.2 대로 10/10 은 근처 체인 경로를 막을 가능성이 커서 3/3 이 유력하다.
  - **조사 격자의 시각.** 정적 층으로 실을 때 점유 시각은 조사 때 stamp 가 아니라 **이번 에폭의 모든 live 관측보다 앞선 기준 시각**(에폭 시작 stamp − 1 s)으로 둔다
    (`set_slam` 은 받은 stamp 를 점유 시각으로 쓰고 `occupied()` 는 그보다 새 clear 만 인정한다 — 조사 stamp 가 크면 새 에폭의 clear 가 지우지 못한다, Codex 반례).
    조사 시각은 매니페스트에 출처로만 남긴다. 조사 격자는 `latest_map.npz` 경로가 아니라 따로 싣는다(그 경로의 `file_before_run`·`stamp_ahead` 거절은 그대로 둔다).
  - **`/map` 은 시작 때 한 번만 받는다.** 브리지는 L4 가 쓰는 `/map`(첫 processed scan 뒤의 첫 것)을 받은 뒤 구독을 끊는다. slam_toolbox 의 `updateMap()` 은 구독자가 없으면 건너뛰므로
    지도 재생성 비용이 0 이 되고(2.3 의 한쪽 몫), 원점이 바뀐 지도가 비운 칸을 되살리는 일(2.4)도 없다. 계획용 정적 층은 **지도 묶음의 조사 격자**를 세션 시작에
    한 번 싣는다(원점 고정). 이후 바뀐 배치는 지금처럼 관측 기억이 맡는다.
- **L4 시작 검사 — 세 조건을 따로 확인한다(보류 사유 ③).** 하나라도 어긋나면 runner 는 prewarm 단계에서 세션 실패로 끝나고 임무를 받지 않는다.
  - **묶음 동일성:** 감독자가 세션(첫 에폭) 시작 때 묶음 파일을 해시해 `expected_map_id` 를 정하고 에폭 밖(감독자 상태)에 둔다. 에폭마다 runner 가 실제 세 파일을
    다시 해시해 `expected_map_id` 와 대조한다. 매니페스트의 장면·URDF·SEED 도 이번 실행과 같아야 한다. 어긋나면 `slam_map_mismatch`.
  - **그래프 적재:** 첫 processed scan 뒤에 받은 `/map`(그 전 것은 적재 전일 수 있어 쓰지 않는다)의 점유 칸이 조사 격자 점유 칸의 95 % 이상을 덮는다(빈 지도로 시작하면
    0 % 근처). warm-up 제한 안에 그런 `/map` 이 없거나 덮개가 모자라면 `slam_map_not_loaded`. 브리지는 이 `/map` 을 받은 뒤 구독을 끊는다(L3).
  - **정합 준비:** 첫 정합 응답을 slam_toolbox 가 검사하지 않으므로(2.2) 브리지가 스스로 잰다 — warm-up 의 처리된 scan 마다 끝점 중 조사 격자 점유 칸(1 칸 팽창)에
    드는 비율을 구해, bootstrap 의 5·6·7 번이 모두 0.8 이상이고 자세가 `map_start_pose` 에서 0.10 m·0.05 rad 안이어야 한다. 아니면 `slam_not_localized`.
  - **시작 과정(bootstrap) 규칙 — 필수 변경.** 출발 때 트럭이 서 있으면 지금 게이트는 처음 5 개 뒤 scan 을 보내지 않아 처리가 1·5 번 둘뿐이고, tracker 는 4 번 이상의
    첫 processed 에서 tracking 이 되어 runner 가 warm-up 을 끝낸다(`slam_link.py` 80, `slam_pose.py` 209, `run_transport.py` 6231 — Codex 반례: 정지 scan 30 개에
    처리 `[0, 4]`). localization 일 때만 시작 scan 을 7 개로 늘려 1·5·6·7 번을 `expect_pose`, 2–4 번을 drop 으로 보낸다(slam_toolbox 의 2–4 번 drop 은 그대로).
    브리지는 7 번에 답하기 전에 L4 세 조건을 끝내고, 실패면 그 답을 `failed`(이유 포함)로 보낸다. tracker 는 localization 일 때 마지막 bootstrap scan 의 processed
    에서만 tracking 이 된다 — runner 의 warm-up 은 L4 가 끝나야 끝난다. tracker 는 콘솔과 함께 쓰는 코드라 바꾸기 전에 콘솔 세션에 diff 를 보낸다. mapping 경로의
    시작 규칙은 바꾸지 않는다.
- **L5 브리지 CPU 배치.** 지금의 작업 첫 CPU 하나 고정을 (a) 작업의 CPU 전체(`--cpuset-cpus` = 작업 affinity) 또는 (b) 전용 코어 2 개(작업 CPU 를 2 개 늘려
  그 둘에 고정)로 바꾸는 안을 S4 에서 잰다. 코어를 늘려도 mutex 대기는 남으므로 L3(재생성 없음)과 함께 본다. `isaac_live.sbatch` 는 콘솔 세션의 파일이고
  `test_isaac_slam_bridge_rules.py` 127–132·239–243 이 문구를 고정하므로, 바꾸기 전에 diff 를 보내 합의한다.
- **L6 콘솔 계약(콘솔 세션 요구 2026-10-19).**
  - 상태: `slam: {mode: "localization"|"mapping", map_id, map_loaded}` 를 상태 메시지에 더한다. 결과 기록의 임무마다 `map_id` 를 남기고, 임무 폴더의
    `result.json`(에폭 폴더에 남는다)에도 쓴다. 결과 기록이 에폭마다 새로 생기므로 에폭을 넘는 연결은 이 파일들이 맡는다.
  - 감독 재시작(새 에폭)은 같은 묶음을 다시 읽고 L4 로 `expected_map_id` 를 확인한다. 장면을 다시 만들므로 트럭은 출발 자세에 있고 `map_start_pose` = 출발 자세다.
    같은 `map_id` 는 **좌표계가 같다**는 뜻일 뿐 상태가 이어진다는 뜻이 아니다 — 진행 중이던 임무·lease·arm·hold·planning memory 는 지금처럼 에폭과 함께 끝난다.
  - 아는 픽업 구역·적재 운반의 목적지·home 은 world 좌표로 정의되고 map = world 이므로 같은 `map_id` 에서 그대로 유효하다. 깨지는 경우(지도·장면 불일치)는 L4 가
    세션 시작에서 막으므로 임무 도중의 새 실패 이유는 만들지 않는다.
  - `sim_reset` 은 장면 초기화 그대로다. 같은 장면이므로 같은 지도를 쓴다. 지도를 다시 만드는 기능은 이 계획에 없다(생기면 새 이벤트·계약 변경).
  - SLAM hold 와 planning memory 는 프로세스 소유라 에폭 시작마다 비어 있다(지금과 같다). 에폭 안에서는 지도 전환이 없으므로 새로 비우는 시점은 없다.
- **L7 전환 방식과 기록 호환.** runner 인자 `--slam-map-bundle <폴더>` 를 두고, 있으면 브리지를 localization 으로 띄운다. 없으면 지금의 mapping 그대로다.
  근접 계약에는 `POST_MEASUREMENT`(과거 기록의 기본값 `None` 만 허용)로 등록하고, `RUN_SPECIFIC` 으로 빼지 않는다 — 빼면 mapping 에서 잰 조건을 localization
  실행에도 조용히 적용하게 된다. 비교 기록에는 경로 문자열 대신 `slam_mode` 와 `map_id` 를 남긴다. boundary replay 가 복원한 옛 `args` 에는 이 속성이 없으므로
  읽는 곳은 모두 `getattr(args, "slam_map_bundle", None)` 로 읽는다. replay gate 29 건은 mapping 경로라 바뀌지 않는다. 콘솔 sbatch 가 이 인자를 쓰는 것은 S5 통과 뒤다.
- **L8 바꾸지 않는 것.** 응답 기한 1.0 s, KeyframeGate 의 keyframe 규칙(localization bootstrap 예외 L4 만), holding 의미와 dead reckoning 한도(콘솔 D15 가 따로 고친다), D7c 하역 직선 정합, 주행 허가.

## 4. 단계

1. **S1 오프라인 재생 SR1c (CPU, ws1 도커).** SR1b 틀을 쓴다.
   - 입력을 둘로 나눈다: ray-cast 정답은 U16 의 SLAM 자세, 오도메트리는 U16 의 실제 오도메트리(2.1 끝: 정답에서 m 단위로 표류). 보정 없는 대조군(map←odom 을
     처음 값으로 고정)의 오차를 같이 적어 시험이 보정을 실제로 요구하는지 보인다.
   - U16 의 첫 임무 구간으로 mapping 재생 → `serialize_map` → 묶음. U16 의 나머지 전체를 그 묶음에서 localization 재생해 변형을 고른다(2 h 분량).
   - **긴 부하 순서(V1′ 에 쓴다):** U16 기록에서 출발부터 임무 7 이 끝나 home 에 선 시점까지를 한 단위로 잘라 sim 8 h 에 이를 때까지 잇는다(단위 길이에 따라 약 20 번, 약 16,700 keyframe).
     이음매에서 stamp 는 단위 길이 + 0.1 s 씩 더해 단조롭게 잇고, 오도메트리는 앞 단위의 끝 자세에 다음 단위의 상대 증분을 이어 붙여 끊기지 않게 한다. 정답
     자세의 이음매 점프(home ↔ 출발)가 0.10 m·0.05 rad 를 넘으면 그 단위를 home 에 선 구간까지 줄여 맞춘다. `/map` 을 첫 1 회만 받으므로 응답 사이 간격은 두지
     않는다.
   - 변형: 버퍼·체인 {10/10, 3/3} × 루프 폐쇄 {켬, 끔}, 모두 `/map` 첫 1 회만(L3).
   - 함께 잰다: keyframe 처리 시간 추이·기울기, 정답 대비 정렬 없는 위치·방향 오차, 시작 scan 1–5 의 처리 여부, L4 의 세 검사(정상·손상 파일·빈 파일·같은 장면의
     다른 묶음·정합 불가 scan).
2. **S2 조사 지도 도구.** `slam_replay` 의 world 배치 인자, `run_slam_drive.py` 를 live 기지 장면으로(짐 빼기 가능 여부 확인), mapping 재생 + `serialize_map` +
   조사 격자 + 매니페스트 + 덮개 검사. `feat/fleet-five` bbbcb03 의 직렬화·설정 치환 부분을 가져온다.
3. **S3 브리지·링크.** 컨테이너 스크립트의 localization 실행 경로, 첫 `/map` 뒤 구독 해제, L4 검사, 상태 필드·결과 기록의 `map_id`, 감독자의 `expected_map_id`,
   runner 의 조사 격자 적재, L7 의 계약 등록·호환. 단위·통합 시험.
4. **S4 브리지 CPU 배치 A/B.** U13 과 같은 두 lane 부하에서 지금(코어 하나)·(a)·(b) 를 각각 잰다(tick trace 켬).
5. **S5 live 확인.** S2 묶음으로 U16 과 같은 2 h 연속(배속 0.3), replay gate, ws1 pytest.

## 5. 확인 (결과 보기 전 고정)

| 확인 | 대상 | 통과 |
|---|---|---|
| V1 변형 선택 | S1, S1 묶음, U16 나머지(2 h 분량), 변형 4 개 | 채택 변형: 마지막 10 % 평균 ≤ 0.08 s, 최대 ≤ 0.3 s, `failed` 0, V2 통과. 여럿이면 최대가 작은 것 |
| V1′ 배포 묶음 8 h 비용 | S2 묶음, 긴 부하 순서(sim 8 h)를 S2 조사 격자로 ray-cast 한 재생, 채택 변형 | keyframe 처리 시간의 선형 기울기(순번에 대한 최소제곱)가 끝에서 +0.02 s 이하를 더하고, 마지막 10 % 평균 ≤ 0.08 s, 최대 ≤ 0.3 s, `failed` 0, **그리고 같은 실행 전체에서** ray-cast 정답 대비 정렬 없는 위치 최대 ≤ 0.15 m·방향 최대 ≤ 0.05 rad(정합을 잃어 가벼워진 실행을 배제); 묶음의 N·edge 수·격자 크기를 적는다 |
| V2 위치 오차 | S1, V1 과 같은 실행 | 보정 없는 대조군 오차가 채택 변형의 3 배 이상(시험이 보정을 요구함), 정렬 없는 위치 RMSE ≤ 같은 입력의 mapping 재생(SR1b 운용 설정) RMSE + 0.02 m, 위치 최대 ≤ 0.15 m, 방향 최대 ≤ 0.05 rad |
| V3 시작 검사 | S1·S3, 손상 파일·빈 파일·같은 장면의 다른 묶음·다른 장면 매니페스트·정합 불가 scan 각 1, 모두 정지 출발 | 5/5 가 warm-up 에서 해당 이유(`slam_map_mismatch`/`slam_map_not_loaded`/`slam_not_localized`)로 끝남, 임무 시작 0; 정상 묶음은 정지 출발에서 통과하고 tracker 가 bootstrap 끝(7 번) 전에 tracking 이 되지 않음; 재시작 뒤 첫 clear 가 조사 점유를 지움(단위 시험) |
| V4 CPU 배치 | S4, 배치마다 두 lane 실행 3 회 | 채택안: `slam_link_failed` 0/3 이고 keyframe 응답 p99 가 지금 배치보다 크지 않음 |
| V5 연속 운용 | S5, 2 h, 배속 0.3 | 임무 7 개 이상 `done`(U16 과 같은 양), `slam_link_failed`·`localization_stale` 없음, keyframe 처리 최대 ≤ 0.3 s, 30 분 구간마다 표본 50 개 이상이고 마지막 구간 p99 ≤ 첫 구간 p99 × 1.5, 정답 대비 추정 오차(`slam.control` 기록 전체로 집계, `live_records.py` 57–79) 위치 최대 ≤ 0.15 m·방향 최대 ≤ 0.05 rad, 각 임무의 하역 오차가 콘솔 U-케이스 기준 안 |
| V6 회귀 | S3–S5 | replay gate 29/29(mapping 기본 경로 불변), ws1 pytest 새 실패 0 |

V1 의 +0.02 s·0.08 s, V2 의 0.02 m·3 배, V5 의 1.5 배·50 개는 Claude 잠정안이다. 판정 전에 바꾸려면 이 표를 고치고 이유를 적는다.

## 6. 이 계획이 말하지 않는 것

- 트럭이 출발 자세가 아닌 곳에서 시작하는 재시작(실차의 전원 재투입, 납치) — 재위치(relocalization)가 필요하고 운용 조건 계획 O3–O5 의 범위다.
- 조사 뒤 장면 배치가 바뀌는 정도별 영향(움직이는 짐을 뺄 수 없는 경우의 크기 포함) — V5 의 운용 결과 말고는 따로 재지 않는다. 운용 조건 계획 O6 의 범위다.
- 남는 선형 항(2.2)을 upstream 에서 없애는 수정 — V1 이 크기를 재고, 기준을 넘으면 그때 이 계획을 고친다.
- 실물 RPLIDAR·엔코더 잡음, Jetson 의 처리 여유. 노드 수가 묶여도 Jetson 에서 1 s 안이 보장되지는 않는다.
- STOP 중 차체가 포켓 쪽으로 밀리는 현상(U30, 1226 s 동안 69 mm) — plant 후속으로 따로 본다.

## 7. 자체 검토

### 7.1 ① Claude 재계산 (v1 → v1.1)

- 인용한 저장소 줄 번호를 모두 원본에서 다시 열어 확인했다. slam_toolbox 줄 번호는 처음에 jazzy 브랜치 기준(649/842)으로 적었다가 2.8.5 태그 기준(646/839)으로 고쳤다.
- **전제 오류:** v1 은 "조사 주행을 live 출발에서 시작하면 map = world" 라고 썼으나, `run_slam_drive.py` 는 조사 경로 첫 점에서 출발한다(360 행). 출발을 바꾸는 안으로
  고쳤다 — 이 수정도 ② 에서 불충분한 것으로 드러났다(아래 P1-2).

### 7.2 ② Codex 적대 검토 (v1.1 → v2)

| 지적 | 처리 |
|---|---|
| P1-1 노드 수가 묶여도 `FindNearChains` 가 지워진 ID 를 하나씩 훑어 연산 수가 세션 길이에 비례 | 소스로 확인(`Mapper.cpp` 1713–1727). 1 장의 요구 문구를 "8 h 에서 기한까지 여유 + 남는 증가를 잰다"로 바꾸고, S1 에 8 h 분량, V1 에 기울기 기준을 넣었다. upstream 수정은 6 장에 미룸 |
| P1-2 출발점을 맞춰도 재생 도구가 첫 자세를 원점으로 두어 map ≠ world | 소스로 확인(`slam_replay.py` 125·143). L2 를 "재생 오도메트리를 기록의 world 출발에 배치하는 선택 인자"로 바꿨다(기본값 불변) |
| P2-3 "지도로 끌어당기는 것은 루프 폐쇄뿐"이 틀림 — `LinkNearChains` 가 가중 평균으로 자세를 바꿈 | 소스로 확인(1492–1496, 1639–1659). 2.2 를 고치고, 체인 최소 10 이 조사 한 번 통과로는 안 채워진다는 점을 더해 S1 변형에 루프 폐쇄 켬/끔을 넣었다 |
| P2-4 정답과 같은 오도메트리면 보정 없는 추정기도 V2 통과 | 부분 동의: SR1b 의 오도메트리는 U16 실제 값이라 정답에서 m 단위로 표류한다(2.1). 그래도 판별력을 보이도록 보정 없는 대조군을 S1·V2 에 넣었다 |
| P2-5 L4 가 같은 장면의 다른 묶음을 못 막고, 첫 자세가 가깝다고 정합된 것은 아님 | 감독자 `expected_map_id`, 세 조건 분리, 끝점 일치 비율 검사, V3 에 두 경우를 더했다 |
| P2-6 localization 의 `/map` 도 원점이 바뀌어 비운 칸을 되살림 | L3 을 "첫 `/map` 만 받고 구독 해제, 계획 층은 묶음의 고정 조사 격자"로 바꿨다(재생성 비용도 함께 사라짐) |
| P2-7 S1 지도와 배포 지도의 N 이 달라 절대 상한을 옮길 수 없음, V5 에 운용량·방향 오차·표본 수가 없음 | V1′(배포 묶음)을 더하고 V5 에 임무 수·표본 수·방향 오차·`slam.control` 집계를 넣었다 |
| P2-8 근접 계약 등록 위치 미정, boundary replay 의 옛 `args` | L7 에 `POST_MEASUREMENT`·`getattr` 기본값·`slam_mode`/`map_id` 기록을 정했다 |
| P3-9 targets 재표현 설명이 실제보다 강함 | 2.4 문구를 고쳤다 |
| (2차) P1 정지 출발에서는 processed 가 1·5 번뿐이라 L4 의 3 개 조건을 못 채우고 tracker 가 먼저 tracking 이 됨 | 코드로 확인. L4 에 localization bootstrap(1·5·6·7 번, 7 번 답 전 L4 완료, tracker 는 bootstrap 끝에서 tracking)을 필수 변경으로 넣고 L8 에 예외를 적었다 |
| (2차) P2 "U16 2–8 × 4" 는 8 h 가 아니고(전체 5,405 s), 배포 묶음은 2 h 만 검사 | 8 h 를 sim 시간 기준으로 정의하고(1 장), 임무 7 까지를 sim 8 h 까지 이은 긴 부하 순서(이음매 규칙 고정)를 배포 묶음에서 돌리는 V1′ 로 옮겼다 |
| (2차) P2 조사 격자의 점유 시각 계약이 없어 새 에폭의 clear 가 못 지움 | L3 에 "에폭 시작 − 1 s 기준 시각, 조사 시각은 출처로만", 별도 적재 경로, V3 에 회귀 시험을 넣었다 |
| (2차) P2-4 철회 | Codex 가 v2 입력 설명에 따라 철회 |
| (3차) P2 V1′ 이 비용만 봐서, 정합을 잃고 가벼워진 실행도 통과(`ProcessLocalization` 은 정합 점수와 무관하게 `true`) | V1′ 에 같은 8 h 실행 전체의 정렬 없는 위치·방향 오차 상한을 AND 로 붙였다 |
| 확인: 시작 drop 은 공통 경로라 1·5 번 처리 규칙이 localization 에서도 맞음 | 2.2·L4 에 반영(시작 규칙은 S1 확인 뒤에만 고침) |

### 7.3 ③ 독립 검토 (agy, Gemini 3.8 Flash High — ①·② 결론을 보이지 않음. Claude Opus 4.6 은 쿼터 소진으로 실패)

"적히지 않은 전제" 10 개를 받았다. 반영: 저장 노드가 Ceres 로 움직이는 표류의 측정(V5 의 정답 대비 오차, Ceres 는 첫 노드만 고정함을 소스로 확인 — 2.2), 조사 때 있던
높은 짐이 옮겨지는 문제(L2 의 짐 빼기), 벽시계 `map_update_interval` 이 배속 0.3 에서 3.3 배 자주 도는 낭비(L3 의 첫 `/map` 뒤 해제), Ceres 반복 상한 없음(2.2 에 기록,
V1·V1′ 최대값으로 판정), 코어를 늘려도 mutex 대기가 남음(L5). 해당 없음으로 판단: 적재 시 LiDAR 가림·차체 기울기(mapping 과 같은 조건이고 U16 이 이미 겪음 — V5 가
덮음), 에폭 사이 DDS 잔여 메시지(에폭마다 네트워크 없는 새 컨테이너), 진행 방향별 덮개(LiDAR 360°).
