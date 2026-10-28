# 근접 추적 front 적용 범위와 인계 실패 정지 (C 강화형)

총괄 결정(2026-10-28): C 강화형으로 간다.
- front 상한에 적용 범위를 명시한다. 범위 판정은 accepted·anchor 표적 거리로 한다.
- 범위를 벗어나기 전에 roof 인계를 못 하면 정지한다. roof 를 잃어도 front 로 돌아가지 않는다.
- 그 뒤 새 계약으로 D8c 와 폐루프를 다시 확인한다.
- 순서는 계획 → Codex 수렴 → 시험 먼저 → Isaac 이다.
- A(관문이 거절한 관측 빼기)·B(구멍 검사 국소화)는 하지 않는다. D(근접 front 검출의 표적 일관성 제한)는 이 계획 뒤의 후속 계획으로 둔다.

**이것은 결과를 본 뒤의 규칙 변경이다.** riib line_1_m 재측정(`nrv_73ba6940`, 21 실행, 영상 끔)에서 상한표 내보내기가 거부된 뒤에 정했다. 사전 등록한 D8b·D8c 규칙(`docs/plans/2026-10-04-lidar-obstacle-map.md` D8c 3판, `docs/plans/2026-10-27-insertion-lateral-drift.md` §5.3)에는 front 적용 범위가 없었다. 아래 제외 건수와 이유는 그대로 검증 문서에 옮긴다.

## 1. 무엇이 막혔나 (재확인)

- 내보내기 거부: `the near capture at 2.516 m has no front bound`. 처음 거부한 실행은 s2_s1_v055 다. 큰 오차를 낸 실행은 s2_s3_v600 이다(Codex 재확인).
- 원인 관측: s2_s3_v600 index 138, 정답 카메라–앞면 0.018767 m(삽입 끝).
  - front 검출기는 `status valid` 로 냈고, 앞뒤 오차는 왼쪽 +1.024 m·오른쪽 +1.013 m 다.
  - 재검출 중점은 world x ≈ 1.552 m 이고, 장면 뒤쪽 `loading_pallet_1` 앞면은 x 1.548 m 다. 배경 팔레트 앞면을 잡은 것으로 보인다(Codex 재계산. 화소별 물체 ID 가 없어 확정은 아니다).
  - D8b 리플레이의 고정 관문(0.05 m)이 이 관측을 `step` 으로 거절했다. 그 행은 이미 roof 모드였다.
- 기전: `interval_bound` 의 구멍 검사 범위는 그 출처의 모든 측정 구간 중 가장 큰 앞뒤 상한 + 0.01 m 다(`near_field_bounds.py:49`). 이 1 건 때문에 front 구멍 검사 범위가 1.034 m 가 되어, 2.5 m 부근과 인계 부근(0.9 m)의 front 조회가 막힌다.
- 런타임에서는 이 관측이 front 모드에서 왔더라도 `_compare` 의 앞뒤 허용(밀리미터 단위)에 걸려 거절됐을 것이다. 문제는 추적기의 연관 판정이 아니라 **front 상한표가 front 를 쓰지 않는 거리의 관측까지 담는다**는 데 있다.

## 2. 수치 (원자료에서 다시 계산)

모두 riib line_1_m 의 차체 `dls08_measured`(실행 기록의 `forklift_urdf`)와 카메라 장착 `carriage_low_measured`(base x 0.619 m)다. `tracking_frame()` 으로 다시 계산했다.

| 양 | 값 | 출처 |
|---|---|---|
| 포크 끝 x | 0.950 m | `tracking_frame().fork_tip_x_m` |
| 카메라 광학 중심 x | 0.619 m | 실행 기록 `perception_mount.translation_m` |
| 인계 관문 g(앞면 중점 x) | 1.520 m | 포크 끝 + 0.6 − 0.03(`run_transport.py:4437`) |
| 관문에서 카메라–앞면 | 0.901 m | 1.520 − 0.619 |
| 포크 끝이 앞면에 닿을 때 카메라–앞면 | 0.331 m | 0.950 − 0.619 |
| 삽입 끝 카메라–앞면 | 약 0.001 m | 목표 앞면 x 0.620 − 0.619 |
| 15 실행의 실제 인계 거리 | 0.771–0.889 m | 진단 D8c `handoff.distance_m` |

재측정의 S2 15 실행(D8b 관측 집합. 보존 입력 전체는 S2 15 + S3 6 실행이다)의 front 관측을 정답 거리 구간별로 보면(`analysis2/observations/d8b_observations.json`):

| 구간 | 관측 | 유효 | 최대 앞뒤 오차 |
|---|---|---|---|
| 0.0–0.1 | 336 | **1** | 1.024 m |
| 0.1–0.2 | 274 | 0 | — |
| 0.2–0.3 | 275 | 0 | — |
| 0.3–0.4 | 270 | 0 | — |
| 0.4–0.5 | 405 | 319 | 0.85 mm |
| 0.5–1.2 | 1,046 | 1,042 | 0.56 mm 이하 |

0.4 m 아래의 front 는 그 1 건을 빼고 모두 무효다.

## 3. 계약

### 3.1 front 적용 하한 F

- **F = ⌈(포크 끝 x − 카메라 x) / 0.1⌉ × 0.1 m.** 포크 끝이 앞면에 닿는 카메라–앞면 거리를 구간 경계로 올린 값이다. riib line_1_m 에서 0.331 → **0.4 m** 다.
- 뜻: 포크가 포켓에 들어가기 전에 roof 가 추적을 넘겨받아야 한다. 포크가 들어간 뒤에는 front(앞면 무늬)를 쓰지 않는다.
- 인계 관문(0.901 m)과의 사이 0.5 m 는 순항 0.065 m/s(S3 진단)로 약 7.7 s, 프레임 약 77 개다. 인계는 3 프레임 연속 일치를 요구한다. 15 실행의 인계 지연은 관문 뒤 최대 0.13 m 였다.
- F 는 상한 파일에 기록한다(차체·카메라마다 다르다). 런타임은 파일의 값을 읽는다.

### 3.2 상한 파일 (`near_field_bounds/v1` 에 필드 추가)

- `observation.front_range = {"min_distance_m": F, "rule": "...", "inputs": {"fork_tip_x_m", "camera_x_m"}, "excluded": {...}}`.
- front 행은 F 이상의 구간만 담는다. F 아래 구간은 front 의 운용 범위 밖이다. 0 m 아래를 지금 구멍으로 세지 않는 것과 같은 방식으로, 구멍 검사에서 빠진다.
- `excluded`: 정답 거리가 F 아래인 front 관측의 수를 유효·무효로 나눠 적는다. 유효한 것은 실행·index·정답 거리·오차까지 적는다. 이번 자료(S2 15 실행)에서 기대되는 값은 유효 1(s2_s3_v600 index 138)과 무효 1,154 다.
- roof 는 바뀌지 않는다(범위 하한 0).
- **호환:** `front_range` 가 없는 파일(기존 `config/near_field_bounds_measured.json` 등)에서는 범위 감시(R1) 자체를 하지 않는다. 하한 0 으로 읽는 것만으로는 이전 동작이 보존되지 않는다. R1 은 오도메트리를 조회하므로, 무효 관측에서도 `unbounded` 가 새로 날 수 있다(Codex P2: 기존 파일로 unarmed·dt 68.4 s 반례). 기존 파일은 다시 내보내지 않는다.
- `interval_bound(rows, distance, floor_m)`: floor 아래 공간은 구멍이 아니다. floor 의 기본값은 0 이고, 지금의 "0 m 아래" 규칙과 같다.

### 3.3 런타임 (`NearFieldTracker`)

- **R1 범위 판정** — 상한 파일에 `front_range` 가 있고, 추적기가 무장(`armed`, 날 끝–면 1.5 m 구간에 들어섬)이고, front 모드일 때만 한다. 매 프레임 front 평가 전에 한다.
  - 무장 전에는 하지 않는다. 무장 구간의 시작(카메라–면 약 1.83 m)은 F(0.4 m)보다 한참 멀다. 그래서 무장 전 프레임은 범위 밖일 수 없다. 이렇게 하면 무장 전 긴 무효 구간에서 오도메트리 표를 넘는 나이가 새 `unbounded` 를 내지 않는다.
  - 기준: 마지막 accepted 관측 L(처음에는 anchor = 근거리 촬영). 그 held 포켓 중점을 이 프레임의 정렬 자세로 base_link 에 옮긴 점을 p, 그 축(관측 yaw)을 a, 법선을 n, 카메라 중심을 c 로 둔다. m = p − c 다.
  - 오차 반경(`_compare` 가 L 과 오차 없는 관측 사이에 쓰는 항):
    - u = L.bound.along_m + e(dt) + ρ·ψ(dt) + pallet_term
    - v = L.bound.lateral_m + e(dt) + ρ·ψ(dt) + pallet_term
    - δ = L.bound.yaw_rad + ψ(dt)
    - dt 는 L 이후 이 프레임까지의 정렬 시간 차, ρ 는 후축에서 두 포켓 중심까지의 레버 중 큰 값이다.
  - **하한 d_lo = (m·a − u)·cos δ − (|m·n| + v)·sin δ.**
    - 정답 중점은 p + x·a + y·n(|x| ≤ u, |y| ≤ v), 정답 축은 a 를 |ε| ≤ δ 만큼 돌린 것이다.
    - 이때 정답 카메라–면 거리는 (m·a + x)·cos ε + (m·n + y)·sin ε 다. m·a − u > 0 이고 δ < π/2 이면 그 최솟값이 위 식이다.
    - 첫 안에서 쓴 식(`_compare` 의 앞뒤 허용만 빼는 식)은 축이 다를 때 생기는 m·n·sin ε 항이 빠져 하한이 아니다(Codex P1 반례: 중점 가로 50 mm 에서 정답 0.39997 m, 옛 식 d_lo 0.40005 m).
  - d_lo < F 이면 이 프레임의 front 를 평가하지 않는다. 결과는 `front_eval = "out_of_range"`, **`handoff_missed = True`** 다.
  - L 의 나이가 오도메트리 표를 넘으면(`UnboundedAge`) 지금처럼 `unbounded` 다. 무장 뒤라 이 경우는 이미 `lost` 와 겹친다.
- **R2 정지:**
  - `handoff_missed` 는 종단 정지다. 러너가 `near["handoff_missed"]` 를 세우고, 코어 `near_tick` 의 종단 사유에 `handoff_missed` 를 더한다.
  - 종단끼리의 우선순위는 lost(failed) > mismatch > unbounded > handoff_missed 다. 종단은 대기(`lost_wait`·`budget`)보다 앞선다(`decide_stop` 의 지금 규칙).
  - 재시도(`retry_allowed`)에는 넣지 않는다. 실패 분류는 MISSION 이고, 콘솔 문구도 더한다.
  - 정지 거리는 기존 보호가 맡는다. 날 끝–면 0.15 m(`PROTECT_W_M`, 카메라–면 약 0.48 m)부터 보호가 켜져, 종단 정지 뒤 날 끝 sweep·벽 틈·예산을 지금처럼 본다. 이 정지가 실제로 안전한지는 §5.2 의 주입 팔로 확인한다.
- **R3 roof 소실:** roof 모드에서 front 로 돌아가는 경로는 지금도 없다(`on_frame`). 이것을 시험으로 고정한다(roof 무효가 이어져도 mode 가 roof 이고 front 를 부르지 않음). 추적기를 다시 만드는 경로는 근거리 촬영(재시도 포함)뿐이고, 그 자리는 근거리 촬영 거리(약 2.5 m)라 F 보다 멀다.
- **R4 프레임 끊김:** S4c-1 주입 `--near-drop-frames-from-m` 의 drop latch 는 `on_frame` 전에 돌아간다. 그래서 그 뒤에는 R1 이 돌지 않는다. 이 경우는 지금처럼 lost·budget 정지가 맡는다. drop latch 는 시험 주입이고 운영 경로가 아니다.
- 이 계약은 roof 의 거리 하한이나 roof 구간의 오관측을 다루지 않는다(§6).

### 3.4 내보내기·리플레이 도구

- `tools/export_near_field_bounds.py`:
  - 차체 URDF·카메라 장착에서 F 를 계산해 `front_range` 를 쓴다. 입력값과 규칙 문자열도 같이 쓴다.
  - F 아래 정답 거리 구간의 front 행은 표에서 빼고, 그 관측 수를 `excluded` 에 적는다.
  - 근거리 촬영 상한(`near_capture_check`)은 floor 를 넘긴 `interval_bound` 를 쓴다.
- `tools/d8b_observations.py` 는 바꾸지 않는다. 원본 관측과 D8b 집계는 그대로 보존한다(제외는 내보내기 단계에서만 한다).
- `tools/d8c_replay.py`:
  - 리플레이는 진단용으로 모든 프레임에 `front_fn` 을 따로 부르고(`d8c_replay.py:430`), 그다음 `NearFieldTracker` 를 부른다. 코어 `near_tick` 은 돌리지 않는다. 그래서 R1 은 추적기 안에서 들어가지만, R2 의 정지는 리플레이에서 "첫 `handoff_missed` 행"으로만 나타난다.
  - 무장은 리플레이가 지금 하는 대로 한다.
  - **(f) 범위:** 추적기의 `front_eval`(진단 호출이 아님) 가운데 그 행의 **정답** 카메라–면 거리가 F 아래인 것이 0 이어야 한다. `handoff_missed` 도 0 이어야 한다(정상 재생).
  - **(g) 하한 검사:** R1 이 계산한 모든 행에서 d_lo ≤ 그 행의 정답 카메라–면 거리여야 한다.
  - 기존 (b)의 "인계 뒤 front 가용 프레임"은 진단 호출로 세는 지금 정의를 그대로 둔다.
  - **roof 억제 변형:** roof_fn 이 늘 무효를 내게 해 같은 15 실행을 다시 돌린다. 판정은 첫 `handoff_missed` 행까지 본다.
    - (i) 모든 실행에서 `handoff_missed` 가 난다.
    - (ii) 그 행까지 추적기 `front_eval` 의 정답 거리가 모두 F 이상이다.
    - (iii) 그 행의 정답 거리가 F 이상이다.
    - 기록된 궤적을 그대로 재생하는 것이라 제동은 보지 않는다(§5.2 가 본다).
- `tools/s4_drive_terms.py`(S3)는 새 상한 파일의 SHA 에 묶이므로, 새 파일로 다시 만든다.
- 러너: S4c-1 주입에 `--near-inject-roof-off`(시험 주입, 기본 off, 근접 추적 필요)를 더한다. 추적기의 roof_fn 이 늘 무효를 낸다. 근접 추적 계약에는 실행별 인자로 등록한다(`near_tracking.py` RUN_SPECIFIC).

## 4. 시험 (먼저 쓴다)

- `near_field_bounds`:
  - `front_range` 파싱(없으면 범위 감시 없음). F 가 0.1 격자가 아니거나 front 첫 행보다 높으면 거부한다.
  - `interval_bound` floor: floor 아래는 구멍이 아니다. floor 0 은 지금과 같다. floor 위의 미측정 구간은 여전히 구멍이다.
- `NearFieldTracker`:
  - d_lo 식: 합성 사례에서 m·n ≠ 0·ε ≠ 0 인 정답 거리의 최솟값과 같음(Codex 반례 포함).
  - front 모드·무장에서 d_lo ≥ F 이면 지금과 같은 결과다(기존 시험 불변).
  - d_lo < F 이면 front 를 평가하지 않고 `handoff_missed` 다. `_evaluate` 를 부르지 않음으로 확인한다.
  - 무장 전에는 R1 이 없다. 무효 front 와 표를 넘는 나이에서도 `unbounded` 가 나지 않는다.
  - roof 모드에서 roof 무효가 이어져도 front 를 부르지 않는다(R3).
  - `front_range` 없는 상한으로는 지금과 같다. 기존 파일로 Codex 의 unarmed·dt 68.4 s 사례를 시험한다.
- 러너·코어:
  - `handoff_missed` 가 종단 정지가 되고 재시도가 되지 않는다.
  - 종단 우선순위(lost > mismatch > unbounded > handoff_missed)와 `lost_wait` 와의 동시 조건.
  - 실패 분류 MISSION, `MISSION_KEYS`, 콘솔 문구.
  - `--near-inject-roof-off` 의 인자 검사와 계약 등록.
- 도구:
  - 내보내기가 `front_range`·`excluded` 를 쓰고, F 아래 유효 관측의 오차가 front 상한에 들어가지 않는다(합성 입력에 1 m 오차 관측을 넣어 확인).
  - d8c_replay 의 (f)·(g)와 roof 억제 변형.
- 재생 게이트(runs29)는 근접 추적을 켜지 않은 실행이라 영향이 없어야 한다. 그래도 돌려서 확인한다.

## 5. 확인 (판정 규칙은 결과 전에 고정)

### 5.1 다시 내보내기·D8c·S3 (CPU, ws1, 새 Isaac 실행 없음)

- 입력: `nrv_73ba6940` 의 S2 15 + S3 6 실행(보존 중). D8b 관측은 S2 15 실행이다.
- 내보내기는 통과해야 한다. `excluded` 의 유효 건수와 그 목록을 검증 문서에 적는다.
- D8c: 사전 등록 (a)–(e)에 (f)·(g)를 더해 15 실행 모두 통과해야 한다. roof 억제 변형은 15/15 가 (i)–(iii)을 채워야 한다.
- S3: 사전 등록 S3 판정(§5.3)을 그대로 쓴다. 새 상한으로 만든 drive terms 가 confirmed 여야 한다.
- 산출: `config/near_field_bounds_riib_line_1_m.json`, `config/near_field_drive_terms_riib_line_1_m.json`(§5.3 의 이름), 검증 문서. 기존 `config/near_field_bounds_measured.json` 은 건드리지 않는다.

### 5.2 폐루프 (Isaac)

- 조건: riib line_1_m, §5.3 사양 그대로에 `--near-tracking` 과 새 두 파일을 더한다. 영상은 끈다(VIDEO=off). 근접 추적 계약이 조건 일치를 검사한다.
- **정상 팔:** seed 0–5, 6 실행. 속도는 drive terms 가 정한다. 판정(모두 만족):
  1. 6/6 삽입 완료.
  2. `handoff_missed`·`mismatch`·`lost`·`unbounded` 종단 0.
  3. `tools/s4_judge.py` (i)–(vii) 통과. 벽 오차·오도메트리·지연·이동 예산·팔레트 정지·접촉·정지 명령 구간이다. 실행에 `--record-pocket-frames` 를 준다.
  4. 정답 거리 F 아래에서 front 평가 0.
  5. 인계 거리 기록(판정 아님).
- **주입 팔:** `--near-inject-roof-off`, seed 0–2, 3 실행. 새 종단 정지가 물리적으로 안전한지 본다. 판정(모두 만족):
  1. 3/3 이 `handoff_missed` 로 끝난다.
  2. 그 순간 정답 카메라–면 거리 ≥ F.
  3. `tools/s4_judge.py` (vi) 접촉 0(정지 포함), (vii) 그 정지 명령 구간의 정답 이동이 러너가 쓴 정지 계약 S 와 보호 뒤 모서리 sweep 안이고 끝에 선다, (iv) 마지막 accepted 뒤 이동 0.03 m 안.
  4. 날 끝이 정답 앞면을 넘지 않는다.
- **표본·교체:**
  - 실행 불능(Isaac 크래시·카메라 RGB 없음·Slurm 오류)만 교체한다. 같은 팔의 다음 미사용 seed(정상 6–7, 주입 3–4)로, 팔마다 최대 2 회까지다.
  - 실패·교체한 실행도 모두 기록에 남긴다.
  - 6 실행은 이 조건의 통합 확인일 뿐이다. 6/6 이어도 실패율의 단측 95 % 상한은 약 39 % 라 신뢰성 보장으로 읽지 않는다.
- 크기: riib batch, `tools/isaac_bundle.py` 로 정한다.

## 6. 남는 위험·범위 밖

- **D(후속 계획):** 근거리 front 검출에는 기대 위치 제한이 없다. 근접 front 가 배경 팔레트를 잡는 일은 F 위에서도 날 수 있다. 런타임에서는 `_compare` 가 거절하지만, 그 관측이 상한표에 들어가면 이번처럼 표가 넓어진다. F1(원거리)과 같은 종류("검출이 지시 대상에 묶이지 않음")라, 근접 front 에도 표적 일관성 제한을 넣는 계획을 이 계획 뒤에 세운다. 검출기가 바뀌면 상한을 다시 재야 한다.
- roof 의 거리 하한은 정하지 않았다. roof 는 삽입 끝까지 쓰고, 이번 자료에서 roof 의 큰 오차는 없었다.
- F 는 "포크 끝이 면에 닿는 거리"라는 기하 규칙이다. 그 아래의 front 가 이번 자료에서 거의 무효였다는 관측과 맞지만, 측정으로 정한 값은 아니다.
- d_lo 는 측정 상한(표본 최대)과 오도메트리 표로 만든 하한이라, 그 표들과 같은 성격이다. 증명된 한계가 아니다.

## 7. PR 나누기

1. 이 계획.
2. 구현: 상한 모듈, 추적기, 러너·코어, 실패 분류·콘솔, 내보내기·d8c_replay. 시험 먼저.
3. 다시 내보내기·D8c·S3 결과: 새 config 두 파일과 검증 문서.
4. 폐루프 Isaac 결과 문서.

## 자체 검토

- ① 직접 재계산(Claude):
  - 포크 끝 0.950·카메라 0.619·관문 1.520 m 는 ws1 스냅숏 `nrv_73ba6940` 에서 `tracking_frame()` 과 실행 기록으로 다시 계산했다. 관문 거리 0.901, 포크 끝 거리 0.331, 삽입 끝 0.001 m 다.
  - 구간별 front 관측 수는 `analysis2/observations/d8b_observations.json` 에서 다시 셌다.
  - 인계 거리 0.771–0.889 m 는 진단 D8c 출력 15 개에서 읽었다.
