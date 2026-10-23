# P1c 탐색 금지 띠 — 검증 결과 (2026-10-23)

계획: `docs/plans/2026-10-23-planner-permission-clearance.md`(v4, PR #109). 사용자 결정 2026-10-23: 방향 P1c + P6 + P3a, rack 완주는 다음 목표가 아니라 한계 기록.
구현: branch `feat/p1c-search-keepout`(5186819d).

## 무엇을 바꿨나

- `SyntheticMissionGeometry.lidar_keepout_m`(G, 기본 0)이 LiDAR 면(1.05 m)보다 높은 소품을 각 변 G 만큼 키운다. 키운 소품은 탐색(관측·접근·운반·복귀·도킹 재접근)에만 쓰고,
  고정 직선(접근 끝·삽입·꺼내기·하역·물러나기·재작도)은 실제 크기로 검사한다.
- 짐을 든 운반이 키운 띠 안에서 출발하면 먼저 곧게 물러난다(`keepout_exit_prefix`, 트레일러 출구 접두 PR #91 의 일반화). 접두 여유는 1.5·(G + 계획 여유) + 0.01 m 이고,
  접두 직선은 실제 크기 소품으로 검사한다. 재계획마다 그때의 여유로 접두를 다시 만들고, 코어는 설치된 경로의 `exit_prefix` 기록에서 접두 길이를 읽어 그 구간에도 허가를 적용한다.
- 러너 `--lidar-keepout-m`(근접 계약 기본 0.0, `--grid-planning` 과 같이 쓰면 거부), 장면 도구 `--lidar-keepout`, factory 스윕 `--lidar-keepout-m`.
  `deploy/riibotics/riib_scene.sbatch` 기본값은 PR #110 에서 `KEEPOUT=0.15` 였고, 아래 "기본값" 절의 사용자 결정으로 0 이 됐다. rack 사본의 장면 전용 금지 구역(PR #106)은
  뺐다(기둥은 실제 크기).
- G = 0 이면 계획·기록이 전과 같다(기록·장면 해시에서 필드를 뺀다).

## 결과 요약

- **CPU 계획**: 리보틱스 18 장면 모두 G 0.15 까지 계획된다. G 0.20 에서는 aisle_rack·aisle_out_rack 만 `search_found_nothing` 이다.
  factory seed 0–49 는 G 0 과 G 0.15 모두 49/50 이다(seed 47 은 둘 다 운반 탐색 시간 초과).
- **재생 게이트**(p1c_81a007b1, runs29): 29/29 exit 0. 기존 29 개 기록(꺼진 설정)의 재생 호환을 확인한 것이다.
- **ws1 전체 pytest**(5186819d): 6145 passed · 10 failed · 21 skipped. 실패 10 건은 ws1 에서만 나는 알려진 목록(`preview_docking` 6, `diagnose_detection` 2,
  `evaluate_cli` 1, `remote_model_jobs` 1)이다.
- **Isaac**: line_2_m 3/3. line_m 세션은 9 임무 중 6(seed 0 이 첫 임무부터 실패, 원인 미확정 — 아래). aisle_rack SLAM 1 회가 하역까지 갔다.
  트레일러는 같은 스냅숏 A/B 에서 G 0(출구 여유 0.30)과 G 0.15 가 모두 1/3 이고, 두 쪽 모두 비슷한 운반 시작 직후 정지가 났다. 이 표본으로는 완주율 차이를 확인하지 못했고,
  P1c 의 영향은 분리하지 못했다.
- **허가 사전 검사**(계획 6 장의 `check_path`·`check_arc`·`allowed_speed` 합성 격자 검사)는 이번에 돌리지 않았다. P6 트리거와 함께 한다.

## Isaac

### 리보틱스 장면 (p1c_81a007b1, KEEPOUT 0.15)

런처 기본값 MODE=slam NEAR=on MARKERS=on RACK=on, 장면 `test_runs/riibotics/scenes_p1c_81a007b1`, 실행 `test_runs/riibotics/pc3_*`(정답은 `pc3t_*`).

| 장면 | 실행 | 결과 |
|---|---|---|
| trailer | SLAM s0–s2 | 2/3 — s1 은 운반 시작 직후 정지 → 시간 초과 |
| trailer | 정답 1 회 | 완주(하역 0.33 cm) |
| line_2 (`line_2_m`) | SLAM s0–s2 | 3/3 |
| aisle_rack | SLAM s0 | 집기·운반·하역 통과 → 복귀 추종 실패(pos 0.0186, yaw −0.0693) |
| aisle_out_rack | SLAM s0 | 운반 `cusp_position_stall` |
| aisle_rack | 정답 1 회 | 운반 정지 → 시간 초과 |

`transport:occupied` 같은 `slowed_reasons` 집계는 허가 적용 여부를 판정하기 전에도 늘어나므로(`core.py` 1246 행) 실제로 멈춘 틱 수가 아니다. 이 문서는 그 값을 정지 판정에 쓰지 않는다.

### trailer A/B (p1c_5186819d)

같은 스냅숏(`snapshots/p1c_5186819d`)·장면(`scenes_p1c_81a007b1/trailer.json`)·런처 기본값에서 두 설정을 비교했다. 실행은 `test_runs/riibotics/pc4_{off,on}_trailer_s{0,1,2}`.

- off: `KEEPOUT=0 EXTRA_ARGS="--exit-clearance-m 0.30"` — PR #91 의 고정 출구 접두(여유 0.30).
- on: `KEEPOUT=0.15` — 탐색 금지 띠와 자동 접두(여유 0.385). 두 설정은 금지 띠만이 아니라 접두 정책도 다르다.

| 설정 | s0 | s1 | s2 |
|---|---|---|---|
| off | 완주 | 운반 시간 초과 | 운반 시간 초과 |
| on | 운반 시간 초과 | 완주 | 운반 시간 초과 |

- 두 설정 모두 1/3 이다. 같은 seed 라도 설정에 따라 결과가 바뀌었다(s0·s1).
- 실패 4 건은 모두 꺼내기 끝 자리(후축 x 0.514–0.554, y 0.124–0.145)에서 운반이 시작된 직후 멈췄다. off s2·on s0·on s2 의 첫 운반 덤프는 셀 (1.875, 0.525)을 막힌 셀로
  기록했고, off s1 의 덤프에는 막힌 셀이 없다.
- 이것은 두 설정에 공통인 실패 양상이다. 그 자리에 이르는 자세·시각·표시 상태에 P1c 가 영향을 주지 않았다는 증거는 아니다.
- 첫 배치(p1c_81a007b1)를 합치면 on 은 3/6, off 는 1/3 이다. 표본이 작아 두 설정의 차이로 읽지 않는다.
- PR #91 기준 기록(x1_trailer_slam, riib_0b7924b)은 1 회 통과했다. 그 기록과는 근접 재관측 설정(redraw on → always, 측면 0.05 → 0.10, expected off → on,
  lower gate → waive_far_confirmed)과 소스 17 개가 달라 직접 비교가 안 된다. 지금 main 에서 트레일러가 1/3 인 원인은 이 문서 범위 밖이다.

### trailer s1 — 운반 시작 직후 정지

- 운반은 꺼내기가 끝난 자리(후축 (0.536, 0.122), yaw −0.028, 정지 덤프의 추정 프레임)에서 계획됐고(t 43.65 s), 차는 그 뒤 움직이지 않았다.
- 막힌 셀은 (1.875–2.025, 0.525) 4 개다. 적재 외곽(후축 앞 1.56 m, 뒤 0.17 m, 반폭 0.40 m)의 앞 왼쪽 모서리는 (2.107, 0.478) 이다. 이 셀들은 지금 자세의 외곽에 0.01 m 를
  더해도 겹치지 않는다(Codex 재계산).
- 합성 격자에서는 지금 자세 검사가 통과하고, 첫 후진 구간 중간점의 쓸림 여유 0.01 m 가 더해질 때 같은 4 셀에서 `verified_m = 0` 이 재현된다(Codex). 합성 재현으로는 첫 틱의 허가
  정지가 **첫 후진 구간 검사**에서 나며 지금 외곽이 표시와 겹친 것이 아니다(실제 덤프에서 실패 지점을 확정한 것은 아니다). 허가 검사(`drive_permission.py` 386 행)는 중간점 여유를 더하고 지금 몸체가 덮은 셀은 뺀다.
- 그래서 "지금 외곽이 이미 겹쳐 어떤 탈출도 불가능하다"는 전제는 성립하지 않는다. P6 트리거를 넣을 때 이 자리의 탈출 가능성을 따로 확인한다.
- 세 seed 의 운반 시작 진짜 자세는 거의 같다(base x 0.863–0.881, 팔레트 옆 어긋남 −0.026…−0.011 m).

### line_m 연속 세션 (p1c_5186819d, `--lidar-keepout-m 0.15`)

`test_runs/live/ct1_far.sbatch`(SCENARIO=session, CYCLE=P-1:D-1,P-2:D-2,P-3:D-3, `MORE="--lidar-keepout-m 0.15"`), 장면 `scenes_ct1_a770031/line_m_session.json`,
스냅숏 `snapshots/p1c_5186819d`, 실행 `test_runs/live/pc3_5186819d_line_m_s{0,1,2}`.

| seed | 결과 |
|---|---|
| 0 | 0/3 — m1 이 관측 중 재계획에서 `extract:straight_collision`, 이어 m2 관측 시간 초과, m3 `prepare:all_candidates_failed:invalid_goal` |
| 1 | 3/3 |
| 2 | 3/3 |

비교: PR #107 기록(ct1_anc_74ed6a1)은 s0·s1 각 2/3(남은 실패는 cusp 단계)였다.

- 꺼내기 직선 검사(`plan_mission` 의 `extract`)에는 G 가 직접 들어가지 않는다(실제 크기 소품).
- 다만 그 검사는 소품 외에 적재 외곽·경계·계획 여유도 본다. `straight_collision` 만으로 실제 소품과 겹쳤다고 확정할 수 없다.
- 관측 경로가 바뀌어 인식 입력이 달라졌을 가능성은 있지만 확인하지 않았다. **seed 0 은 원인 미확정의 폐루프 회귀 후보로 남긴다.** 재현하려면 실패한 호출의
  target·props·bounds·여유를 기록해야 하고, 지금 기록에는 없다.

## 기본값 — P6 전까지 꺼 둔다 (사용자 결정 2026-10-23)

- `riib_scene.sbatch` 의 `KEEPOUT` 기본값을 0 으로 되돌렸다. 기능과 인자는 그대로이고, 필요하면 `KEEPOUT=0.15` 로 켠다.
- 근거:
  - Isaac 에서 이득이 확인되지 않았다. 트레일러 A/B 는 1/3 대 1/3 이고, rack 은 하역 도달 1 회뿐이다.
  - line_m seed 0 재실행에서 P-2 운반 탐색의 기어 전환이 G 0 의 2 회에서 G 0.15 의 4 회로 늘었고, 그 운반이 `cusp_position_stall` 로 끝났다(`test_runs/live/lmab_5186819d_k{0,0.15}_s0`).
    같은 seed 의 G 0 세션은 3/3 이다.
  - 표본이 작다.
- line_m seed 0 의 처음 실패(`extract:straight_collision`)는 기록 입력으로 CPU 재현하면 G 0 과 G 0.15 가 똑같이 실패했다. 원인은 인식 추정이 참값보다 4 cm 깊어 꺼내기 직선이
  다음 팔레트 상자(CT-1 D1 의 0.25 m 확대)에 0.1011 m 까지 붙은 것이다(계획 여유 0.10, 참값이면 0.1415 m). P1c 와 무관하다.
- **할 일:** P6 가 머지되면 G 0.15 와 G 0 을 trailer·line_m·rack 에서 다시 비교하고 기본값을 정한다.

## rack — 한계 갱신 (결정 3)

- 이전(PR #106): 901632b3 실행 8/8 이 하역 전에 멈췄고, 장면 전용 금지 구역(ec227121)으로 1 건만 하역까지 갔다.
- P1c(G 0.15, 기둥 실제 크기): SLAM aisle_rack s0 1 회가 하역까지 마쳤다. 정답 aisle_rack 과 aisle_out_rack 은 여전히 운반에서 멈춘다.
- G 0.20 에서는 두 rack 장면 모두 CPU 계획이 안 된다. 필요 G(새 팀 규격 표로 운반 0.175–0.215 m, 접근 0.225–0.265 m)를 탐색만 키워서는 채울 수 없다는 계획서 판단이
  그대로다. P6·P3a 가 들어간 뒤 다시 잰다.

## 재현

- CPU 계획: UBAI `/home1/zetin348/forklift-perm/p1c`(job 1038358, 스크립트 `p1c.sbatch`). 이 사본의 `pallet_mission.py` 는 5125518c–cbeca53e 와 같다(sha256 b8492604…).
  5186819d 의 차이는 필드 없는 대역 geometry 를 0 으로 읽는 것뿐이라 계획 결과가 같다.
  - 리보틱스: `tools/riibotics_isaac_scenes.py --slot-markers --rack --plan --lidar-keepout 0,0.05,0.10,0.15,0.20 --jobs 16 --timeout-s 2400 --output scenes` — 18 장면 × 5 G = 90 계획,
    결과 `scenes/*.plan.json`·`scenes_plan.txt`.
  - factory: `tools/bay_planning_sweep.py --layout factory --seeds 0-49 --settings config/isaac_transport_measured.yaml --forklift-urdf sim/models/dls08_measured/forklift.urdf
    --insertion-reserve-m 0.016 --return-home --lidar-keepout-m G --output factory_gG`(G = 0, 0.15, seed 10 개씩 나눠 실행) — 결과 `factory_g0`, `factory_g0.15`.
- Isaac 리보틱스: `deploy/riibotics/riib_scene.sbatch`, 실행마다 `riib_defaults.txt` 와 `run/result.json` 의 `arguments` 에 전체 인자가 있다.
- 재생 게이트: ws1 `test_runs/replay_gate/p1c_81a007b1`.

## 검증

- Codex: 구현 4 회차에서 수렴(cbeca53e, 새 P1/P2 없음). 5186819d 는 장애물 층 시험의 대역 geometry 에 필드가 없을 때 0 으로 읽는 한 줄 수정이다.
  이 문서는 1 회차에서 P2 3 건(A/B 결론 과함, line_m s0 원인 과함, 트레일러 정지를 지금 외곽 겹침으로 잘못 설명)과 재현 정보 P3 을 받아 고쳤고, 2 회차에서 수렴했다(새 P1/P2 없음, P3 2 건 반영).
- 시험: `tests/unit/planning/test_lidar_keepout.py` 10 건, `tests/unit/tools/test_lidar_keepout_runner.py`, `test_rack_scene.py`, `test_riib_scene_launcher.py`.
