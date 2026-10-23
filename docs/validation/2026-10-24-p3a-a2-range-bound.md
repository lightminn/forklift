# P3a A2 사양 거리 잡음·표시 한계 — 검증 결과 (2026-10-24)

계획: `docs/plans/2026-10-23-planner-permission-clearance.md` P3a(v4, PR #109). 사용자 결정 2026-10-23: P3 = (가) P3a.
구현: branch `feat/p3a-a2-range-bound`. 사용자 결정 2026-10-24: 결과에 따라 **(다)** — 경계 함수·설정 파일·`NOISE=a2` 옵션만 넣고 런처 기본값은 const 로 둔다.

## 무엇을 넣었나

- `forklift_core.perception.obstacle_grid`:
  - `a2_spec_accuracy_m`·`a2_spec_cut_m`·`a2_spec_sigma_m`: acc(d) 는 3 m 까지 1 %, 5 m 까지 2 %, 그 밖 2.5 % 다. σ = acc/2, 자르기 c = max(0.02, 1.5·acc) 다.
  - `GridConfig.range_bound_model`("const" 기본 | "a2_spec")과 `sensor_bound_at(measured)`: a2 는 표시 한계 b(d_m) = c(1.04·d_m) 이다.
  - 표시(`_marks`)와 비우기(`_clear_group`)가 빔마다 그 한계를 쓴다.
- `sim/isaac/obstacle_layer.py`:
  - 설정 `range_noise.model` 에 따라 빔마다 참거리로 σ·자르기를 정해 한 번 뽑는다. `beam_noise(count, distances)` 는 a2 일 때 참거리를 받아야 한다.
  - 계획 메모리의 끝점 원 반경은 a2 일 때 max(설정값, 가장 넓은 자르기 + 1.5√2·셀) = 0.301 m 다(셀 양자화 포함, Codex 2 회차).
- `config/obstacle_layer_single_a2.yaml`: `obstacle_layer_single.yaml` 에 `range_noise: {model: a2_spec}` 만 더한 설정이다.
- `deploy/riibotics/riib_scene.sbatch` `NOISE=const|a2`(기본 const), `riib_defaults.txt` 에 기록한다.
- const 설정은 전과 같은 난수열·한계를 쓴다(시험으로 확인, 재생 게이트 29/29).

## 출처와 가정

- 정확도 수치는 Slamtec 공식 사양 페이지 `http://www.slamtec.com/en/lidar/a2spec`(2026-10-23 읽음)의 A2 계열 공통 행에서 가져왔다.
  "1% of the range (≤3 m), 2% (3–5 m), 2.5% (5–25 m)".
- A2M12 데이터시트 v1.1(`LD310_SLAMTEC_rplidar_datasheet_A2M12_v1.1_en.pdf`, 2026-03-12, sha256 `0db66600d23096d4…`)에는 거리 정확도 항목이 없다.
- **±2σ 해석은 가정이다.** 사양은 정확도의 통계 의미를 밝히지 않는다. 이 값이 계통 오차를 포함한 최대값이라면 σ = acc/2 는 무작위 잡음을 과대평가한다.
  반대로 계통 오차(거리에 따라 한쪽으로 치우침)는 이 모형에 아예 없다. 실측 전까지 P3a 는 선택한 시뮬 모형의 한계일 뿐, 실차에서 보장된 한계가 아니다.

## 결과

### CPU

- 필요 간격(계획서 P3 측정, UBAI 잡 1038265 `gap5.jsonl`). 표시 거리 5 m, 나이 0.1 / 0.3 s, 원 방식이다.

  | 나이 표 | 대상 | 지금(상수 0.06) | P3a |
  |---|---|---|---|
  | 보고 표 | 벽 | 0.31 / 0.39 | 0.405 / 0.47 |
  | 새 팀 규격 표 | 벽 | 0.285 / 0.315 | 0.36 / 0.405 |
  | 새 팀 규격 표 | 기둥 | 0.275 / 0.275 | 0.245 / 0.275 |

  긴 벽에서는 필요 간격이 0.075–0.095 m 커진다(3–5 m 의 2 % 행). 근거리 한계는 측정거리 약 1.28 m 이하에서 0.06 → 0.02 m 로 준다.
- 덮임 검사(계획서 P3b 절): 원 방식과 b = c(1.04·d_m) 이면 0.1 s 표 219 128 건과 0.3 s 표 23 184 건에서 참 위치 누락이 0 건이다.
- **도킹 정합(Codex CPU 반례).** 같은 자세에서 사방 ±6 m 벽, 800 빔, seed 0 으로 비교했다.
  - const 는 5 cm 안 inlier 98.25 % 이고, `dock()` 시작점 7 개가 모두 통과한다.
  - A2 는 43.6 % 이고, 7 개 모두 `start_rejected:few_inliers` 다. 정합 기준은 "5 cm 안 inlier 60 %"(`scan_match.py` 111 행)다.
  - SLAM 오차가 없어도 잡음 변경만으로 도킹이 거부된다.

### 재생 게이트·시험

- runs29(p3a_907f12ed): 29/29 exit 0. const 기본값으로 기존 기록이 그대로 재생된다는 뜻이고, A2 사양이나 실차 안전을 검증하는 것은 아니다.
- ws1 전체 pytest(907f12ed): 6355 passed · 13 failed.
  - 10 건은 알려진 ws1 전용 목록이다.
  - 2 건은 새 시험 파일이 `obstacle_layer` 모듈을 두 번 등록해 생긴 시험 격리 문제였고, 고쳤다(다음 커밋).
  - 1 건(`test_far_live_worker`)은 P3a 와 무관한 시간 의존 시험이다. 재실행 결과는 아래 "검증" 절에 있다.

### Isaac (p3a_907f12ed, KEEPOUT 0)

| 장면 | a2 | const |
|---|---|---|
| trailer(`--exit-clearance-m 0.30`) s0–s2 | 3/3 | 같은 배치 없음(다른 시각의 같은 설정 main 실행은 1/6) |
| line_2_m s0–s2 | **0/3** | 3/3(같은 배치) |
| aisle_rack s0 | `extract:straight_collision` | — |

- **line_2_m a2 0/3.**
  - s0 은 하역 도킹 `start_rejected:few_inliers` → 대체 동작 → "Pallet missed the green destination center" 다. 위 CPU 반례와 같은 거부라서 잡음 가설이 강하다.
  - s1(`cusp_position_stall`)과 s2("Excessive body tilt in lift")는 도킹 거부와 다른 실패다. `NOISE=a2` 는 잡음만이 아니라 표시 반경과 비우기 길이도 바꾸므로, 이 둘은 P3a 전체 설정의 회귀 신호로 본다. 원인은 나누지 못했다.
  - SLAM 옆 오차는 a2 0.026 / 0.031 m(s0 / s2), const 0.019 / 0.009 m 다. 인식 오차는 a2 0.054 / 0.048 m, const 0.031 / 0.023 m 다. 이 인식 오차는 SLAM 자세와 지도 변환을 포함하므로 검출기 오차가 아니다.
- **trailer 3/3 은 가설로만 둔다.** 근거리 한계가 줄어 벽 띠가 얇아졌을 수 있다. 하지만 비교할 const 실행이 다른 시각의 것이고, 같은 코드·seed 도 t 8.2 s 부터 갈라지는 비결정성이 있어 실행 시각과 섞여 있다.
- **원인 경로.** 설정이 `shared_with_slam: true` 라서 장애물 층이 뽑은 잡음이 SLAM 전송값에 더해지고, 하역 도킹은 그 전송값을 쓴다(`run_transport.py` 6178 행, `core.py` 5156 행).
  홀 벽은 대부분 5 m 밖이라 σ ≈ 0.07–0.075 m(5.6–6 m)로 const 0.02 의 3.5 배다.

## 결정과 남은 일

- **런처 기본값은 const 다(사용자 결정 2026-10-24).** A2 잡음은 `NOISE=a2` 로 켠다. const 로 돌아간 것은 기존 시뮬 기준선을 되살린 것이지, A2 사양이나 실차 안전을 검증한 것이 아니다.
- 탐색 금지 띠 P1c(G 0.15)와 A2 경계를 함께 쓰는 비교는 하지 않았다.
- **별도 과제: A2 잡음에서 SLAM·스캔 도킹·근접 캡처 수용 기준과 재조정.**
  - 정할 것:
    - 먼 벽에서의 SLAM 오차 한계
    - 하역 도킹의 inlier 기준(5 cm·60 %)과 요구 정밀도의 짝
    - 근접 캡처의 자세·지도 변환 오차가 삽입에 주는 영향
    - 계획 메모리 반경 0.301 m(`--grid-planning` 때)와 픽업 끝점 철회 여유의 영향
  - 기준은 A2M12 책상 실측(P3b 와 함께, 사용자가 LiDAR 를 쓸 수 있게 되면 요청)으로 정한다. 실측은 무작위 잡음과 계통 오차를 따로 재야 한다.
- P3b(빔 방향 표시)는 그 실측 뒤로 보류한다.

## 검증

- Codex:
  - 구현은 3 회차에서 수렴했다(새 P1/P2 없음). 메모리 반경의 셀 양자화가 2 회차 P2 였다.
  - Isaac 해석은 교차검증에서 새 P1 없이 P2 1 건(도킹 기준 호환성, 위 CPU 반례)이 나왔고, 선택지 (다)에 동의했다.
- 시험: `tests/unit/perception/test_a2_range_bound.py`, `tests/unit/test_isaac_obstacle_layer_a2.py`, `tests/unit/tools/test_riib_scene_launcher.py`.
- ws1 경로: `test_runs/replay_gate/p3a_907f12ed`, `test_runs/riibotics/p3a_{a2,const}_*`, 스냅숏 `snapshots/p3a_907f12ed`.
