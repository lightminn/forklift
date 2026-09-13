# 포켓 증거 구조 변경과 M2 재평가 계획 (v7)

> 상태: **v7 — 독립 검토 18회 반영.** v1 `22e967b`, v2 `1644063`, v3 `f633b7a`, v4 `41d4e52`, v5 `3154aea`, v6 `e3d00dd`.
> **이 문서가 정본이다.** `docs/plans/2026-09-13-epal6-scene-set-and-m2-rerun.md` 는 폐기.
> **§A 의 결정을 받기 전에는 Task 2 이후를 착수하지 않는다.**

**문제:** 현행 검출기는 실물 EPAL 6 형상을 사실상 검출하지 못한다. 정면 자세에서 전 거리 미검출이고, 125 자세 격자에서 12 개만 `valid` 인데 그 12 개는 전부 `|yaw| >= 0.26` 에서 중앙 바닥판 안쪽 옆면이 우연히 보이는 경우다.

---

# A. 먼저 받아야 하는 결정

이 계획은 아래 답에 따라 **범위가 달라진다.** Task 1 과 Task 3 만 모든 분기에서 살아남는다.

### ⓪ 이 재평가를 지금 하는가 — **Task 2 착수 조건**

로드맵은 **M2 를 이미 완료로 표시**했다(`2026-09-11-development-roadmap.md:221`, 합성 한정 명시). 개발 2주차(09/18–09/24)는 "M2 기준선 + **H0/H1 조사**", 3주차가 "M2 오차 평가 + **M3 추적·소실 회귀**" 다. M3 는 로드맵에 **"실물 없이 먼저 검증"**(`:67`)이라 적혀 있고 코드가 0 줄이며, 브리프가 명시한 "삽입 중 포켓 추적"이 거기 있다. 로드맵 자신이 "인식 개발이 빨라도 실물 선행 경로를 건너뛸 수 없다"(`:171`)고 적었다.

- (a) 이 계획 전체를 지금 실행한다.
- (b) M2 를 합성 한정으로 봉인하고 M3·H0/H1 을 먼저 한다. **Task 2·3 도 보류다.**
- (c) **규칙만 반영하고 재캡처는 안 한다.** Task 2·3 만 하고 §2a 의 회귀 결과를 기존 M2 검증 기록에 덧붙인다. §2a 가 **v1 100 장면의 집계가 한 글자도 안 움직임**을 보였으므로 기존 M2 주장은 그대로 유효하다. 독립 검토 2건이 이것을 권고했다.
- (d) 축소판: 40 장 규모로 음성에 무게를 두고 합쳐서 예산을 잡는다.

### ⓪′ 필수 관문 구조를 유지하는가 — **Task 2 착수 조건**

v5 는 조사 §5.1 의 다중 특징 점수 구조를 "M2 가 포켓 관측을 산출물로 정의하므로" 물리쳤는데 **로드맵 오독이었다.** 로드맵 M2 는 "팔레트 전면 후보와 두 포켓의 경계·배치를 **함께 사용해**"(`:61`) 추정하라 하고, 무효 반환은 "부족하면"이라고만 한다.

**다만 이 결정을 지금 그대로 묻지 않는다. 두 가지가 먼저 필요하다.**

1. **기록된 반대 근거가 있다.** `docs/validation/2026-09-13-pocket-detector-m2.md:98` — "후보가 하나뿐일 때는 아무리 감점해도 그 후보가 선택되므로 점수화는 거부 조건을 대체하지 못한다"(Codex 와 공동 기각). 음성 장면에는 후보 구조가 하나뿐이다. 점수 구조를 쓰려면 **수락 문턱**이 곧 거부 조건이라는 형태여야 하고, 그 형태를 적어야 한다.
2. **v6 의 "점수 구조로 가면 §3 위양성이 사라진다" 는 근거 없이 쓴 문장이라 철회한다.** 덧셈형 점수는 오히려 §3 위양성을 **쉽게** 만들고(빠진 바닥판이 거부권을 잃고 감점으로 바뀐다), 곱셈형은 한 특징이 0 이면 무너지지만 부분 가림에서 같이 무너진다. 어느 쪽인지 정하지 않은 채 물을 수 없다.
3. **v6 의 "비용은 Task 2·8 에 갇힌다" 도 틀렸다.** `negative_open_bay` 20 장은 오직 그 위양성을 재려고 존재하므로 Task 4·6·9·10 이 같이 움직인다.

→ **먼저 할 일:** 점수 함수 형태·수락 문턱·네 구조(EPAL 6 / 블록 아홉 개 / 앞면 개방 구조 / 유령)의 점수표를 반 페이지로 만든 뒤 이 결정을 묻는다.

### ① 새 음성을 어떻게 찍는가 — **Task 4 착수 조건**

`targets.met` 에는 **위양성 목표가 없다**(`evaluation.py:210-225`). 그리고 통계력이 약하다: eval 6 장면에서 0/6 의 단측 95 % 상한은 **약 39 %** 다. 검증 기록이 이미 같은 지적을 했다(0/7 → 약 35 %, `pocket-detector-m2.md:64`).

- (a) **eval 음성 전체를 합쳐(18 장) 위양성 예산을 `targets` 에 넣는다.** 범주별로 나누면 판정이 불가능하다.
- (b) `negative_open_bay` 20 장을 **안 찍는다.** 독립 검토 두 건이 이걸 권고했다 — flush 변형은 결과를 이미 알고(§C-2), 후퇴 변형은 리그 둘이 갈렸는데(§C-2) 한 구성을 렌더링해도 그 불일치가 안 풀린다. **두 변형을 `test_opening_evidence_cases.py` 안에 상판·기둥 깊이를 매개변수로 넣는 편이 싸고 재현 가능하다.**
- (c) 지금처럼 별도 보고만 한다.

### ② 자세 범위를 어떻게 하는가 — **Task 3 착수 조건**

범위 `x 2–4 m, y ±1.0, yaw ±0.52` 는 브리프 요구가 아니라 2026-09-11 에 정한 값이고 **`build_scene_world.py:102-107` 에 하드코딩**되어 있다. 좁히면 **커밋된 v1 카탈로그가 거부된다** — 팔레트 장면 80 개 중 **54 개가 x > 3.0** 이다. 그러면 §2a 의 회귀 가드와 시험 세 파일이 같이 죽는다.

측정된 사실:
- 입사각(카메라 (0.75,0,0.5) → 팔레트 앞면): 중앙값 **16.3°**, 최대 **51.5°**, 터널 한계 20.77° 초과 **30/80(38 %)**, 양성 23/60.
- **`range_max_m = 5.0` 은 도달 불가**다. EPAL 6 지지대는 4.5 m 에서 104 로 문턱 100 에 붙는다.
- **브리프 Case C·D 는 "너무 가까워서 못 넣는 상황"** 인데 현행 범위는 2 m 미만이 **0 장면**이다. 로봇이 "후진해야 한다"를 판단하려면 그 거리에서 검출돼야 한다.

→ 선택지는 "좁힌다/안 좁힌다"가 아니라 **"재중심화하는가"** 다: 3.5–4.0 m 를 빼고 1.6–2.0 m 를 넣는가. 마운트 위치에서 1.6 m 팔레트가 아예 안 보이면 그것은 **H0 의 장착 결론**이고 4 m 장면 하나보다 가치가 크다.

### ③ MR6D 를 어떻게 하는가

CC-BY-4.0, **D435i(우리 카메라)**, 유로 팔레트 6D 정답. (a) 추가 (b) 140 장 **대체** (c) 미룸. 비용을 정직하게: MR6D 는 **EPAL 6 half 가 아니어서** 별도 prior 가 필요하고, 공개 RGB-D 세트에 **3D 포켓 라벨이 없어** 포켓 정답을 6D 자세 + 규격에서 유도해야 한다.

### ④ `negative_lookalike` 상자를 EPAL 6 규격으로 낮추는가

`build_scene_world.py:260` 에 `[0.6, 0.8, 0.30]`·z 0.15 하드코딩. 안 낮추면 144 mm 팔레트 옆에 300 mm 덩어리가 서서 "닮은 음성"이 아니게 된다. (Task 3 에서 당연히 낮추는 쪽으로 본다 — 반대 의견이 있으면 말해 달라.)

### ⑤ 재타깃으로 화면을 벗어나는 두 장면을 어떻게 하는가 — **Task 4 착수 조건**

v1 자세를 그대로 두고 포켓만 EPAL 6 로 옮기면(y ±0.18625, z 0.061, 폭 0.2275, 높이 0.078) **개구 모서리가 화면을 벗어나는 장면이 둘** 생긴다: `s058`(**positive**) u = 3.95(한계 4), `s100`(**occluded**) v = 478.3(한계 476). v1 저장값은 8.03 과 465.5 였다. 원인은 개구가 z 0.05–0.25 에서 **z 0.022–0.100** 으로 내려앉아 모서리 v 가 화면 하단으로 몰리는 것이다(재타깃 후 v 범위 300.6~478.3).

이건 메타데이터 문제가 아니다. `sample_catalogue` 는 **두 개구가 모두 보일 때까지 자세를 재추첨**하므로 v1 자세는 *v1 기하 조건부 표본*이고, `tests/simulation/test_scene_catalogue.py:76` 이 전 양성·가림에 `left_in_view and right_in_view` 를 단언하며, `docs/interfaces/scene-dataset.md:189` 가 계약으로 적어 두었다. 그리고 §2b (a) 의 검출률 ≥ 95 % 는 검출기가 돌기도 전에 **2/80 = 2.5 %**(예산의 절반)를 잃고 시작한다.

- (a) 두 장면을 제외한다 (b) 자세를 미세 조정해 완전일치를 포기한다 (c) 가시성 여유 `margin_px` 를 EPAL 6 용으로 재정의한다.

### ⑥ `min_band_points` 를 절대 개수로 둘 것인가

`min_band_points = 100` 은 **절대 점 개수**이고, 얇은 덱 v1→v4·실기하 재측정·floor-through 설계·이번 Task 8 의 "4 m 에서 지지대가 문턱의 1.44 배"가 **전부 같은 절벽**이다. 이 계획은 여섯 회차 동안 이 선택을 한 번도 묻지 않고 유지했다.

측정한 사실:
- 거리 감쇠는 **1/d² 이 아니다.** EPAL 6 지지대 최솟값 2.0~4.5 m = 1484 / 626 / 349 / 220 / 144 / 104, 로그 기울기 **−3.25**. 원근 단축 때문이고, 예측 점수를 쓰려면 대역 사각형을 실제로 투영해야 한다.
- **`u / 지지대` 비도 불변이 아니다.** 정면(y=0, yaw=0)에서는 1.52~1.79 로 거의 불변이지만, 선언된 자세 범위 전체에서는 **0.24 ~ 1.79** 로 벌어진다(최악: x 2.5, y −1.0, yaw +0.26 → u=(93,135), 지지대 364). v6 이 "비가 더 싼 길"이라고 적은 것은 **정면 표본만 보고 한 말이라 철회한다.**

→ 지금 고치지 않는다면 그 선택을 §C 에 근거와 함께 남긴다.

---

# B. 채택하는 규칙 (변이 C)

판정 증거를 **각 개구 바로 위**(`u1`·`u2`)로 옮기고 아래 덱 증거를 관문에서 뺀다.

### 규칙

1. **`min(지지대 3 개) >= min_band_points` 인 패턴만 후보로 만든다.**
2. `u1`·`u2` 와 `upper_ok = min(u1,u2) >= min_band_points` 를 기록한다.
   - **점 집합은 평면 inlier 다**(현행 `upper` 와 같다, `pocket_detector.py:338-340`). workspace 점을 쓰면 같은 장면에서 **1.85 배**가 된다(2.5 m 에서 1033/1060 대 1914/1930).
   - **횡방향 구간은 `_gap_runs` 가 돌려준 셀 경계 그대로**다. 보고용 반 셀 보정을 쓰지 않는다. 실측: 정의에 따라 96/103/111/117 로 갈리고 문턱 100 이 그 사이에 있다.
   - **z 상한을 둔다.** 현행 `u` 에는 상한이 없어 작업영역 상한(`height_m + 0.10` = 0.244)까지 센다. 그래서 **덱 위에 실린 상자가 `u` 를 3.0~3.3 배로 부풀린다**(2.5 m 에서 1060/1033 → 3214/3091). 상한을 `height_m + plane_inlier_m` 로 둔다.
3. `upper_ok` 가 거짓인 후보도 버리지 않는다. 다만 **절대 `valid` 이 될 수 없다.**
4. 정렬 1순위 `upper_ok`, 2순위 기존 점수, 3순위 `-distance`.
5. 선택된 후보가 `upper_ok` 가 아니면:
   - `max(u1,u2) >= min_band_points` 이면 기존 폭 검사·광선 분류 결과를 그대로 쓰되 **`valid` 만** `no_pallet/no_upper_deck` 로 바꾼다.
   - 아니면 결과와 무관하게 `no_pallet/no_upper_deck` 를 낸다.

**구현 주의 (규칙 5):** `dataclasses.replace(obs, status=..., reason=...)` 는 `ValueError: Non-valid observations must omit geometry and sigmas`(`pocket_observation.py:82-90`)를 내고, `detect_pockets` 의 포괄 `except`(`:533-536`)가 그걸 `invalid/exception:ValueError` 로 바꾼다. **새 `_status_observation` 을 만들어야 한다.**

**규칙 5 에 `front_fraction` 조건을 쓰지 말 것.** `_build_observation` 은 `valid` 를 내기 전에 이미 그 검사를 한다(`:407-410`). v5 는 그 조건을 규칙에 적었고 **도달 불가능한 죽은 분기**였다.

### 규칙 4 가 깨뜨리는 진단

`detect_pockets:515-517` 은 선택되지 않은 평면을 전부 `lower_pattern_score` 로 표시한다. 규칙 4 는 점수와 무관하게 `upper_ok` 를 먼저 보므로 **더 높은 점수의 평면이 "점수가 낮아 떨어졌다"고 기록된다.**

**`upper_evidence_absent` 는 "진 평면의 패턴이 이긴 평면보다 점수가 높았을 때만" 붙인다.** 다른 두 해석 — 모든 비선택 평면, 또는 비선택 평면 중 `upper_ok` 패턴이 없는 것 — 은 각각 참인 라벨을 덮어쓰거나(v1 에서 12 장면) 애매하다. 이 정의는 **v1 100 장면에서 0 회 발화한다**(정렬이 뒤집힌 장면이 없다). 발화 사례는 새 시험으로만 덮인다.

### 값 (독립 재현 3~4회)

| 규칙 | v1 재현 | EPAL 6 125 자세 | 블록 아홉 개 | 유령(y 0.80) | 후보 수 |
|---|---|---|---|---|---|
| 현행 HEAD | 기준 100/100 | **12 valid**(전부 \|yaw\| ≥ 0.26) | 거부 | **valid, 337~368 mm 오위치** | 82 |
| `min(supports, u1, u2)` | 91/100 | 95 valid | 거부 | 거부 | 70 |
| `u1 + u2` | 100/100 | — | 거부 | **valid, 327~339 mm 오위치** | 79 |
| **채택: 변이 C** | **99/100** | **95 valid** | **거부** | **거부** | **95** |

EPAL 6 125 자세에서 변이 C 는 **95 `valid`**, 위치 오차 p50 3.6 mm / p95 6.3 mm / 최대 8.5 mm. 실패 30 개는 전부 x 2.0 의 큰 측방 오프셋이나 x 2.5·y ±1.0 의 **화각 잘림**이고 `no_upper_deck` 은 0 개다.

**변이 C 와 `min` 단독안은 검출에서 차이가 없다.** 525 개 EPAL 6 자세×가림 조합에서 HEAD 30 / C 209 / `min` 209 이고 **C 가 잃는 검출은 0 개**다. 둘의 차이는 **v1 가림 9 장면의 버킷 라벨**뿐이다(`invalid/pocket_occluded` 대 `no_pallet`). v6 의 표는 C 가 검출을 되찾는 것처럼 읽혔다 — 그렇지 않다.

### 왜 합계가 아니라 최솟값인가

팔레트를 화각 가장자리에 두고 **화각 중심 쪽**에 같은 평면 상자를 놓으면 유령 쌍이 성립한다. **v1 기하 `y = 0.80` 에서 현행 HEAD 가 이미 `valid` 로 내고** 337~368 mm 오위치를 보고한다(독립 재현 4회). 기전: `y >= 0.75` 에서 먼 쪽 바깥 블록이 34.5° 수평 화각을 벗어나 진짜 패턴이 성립하지 않는다. **`y = 0.70` 에서는 네 규칙이 모두 정상(3 mm)이라 시험 자세를 0.70 으로 잡으면 조용히 통과한다.**

---

# C. 이 규칙의 한계 — 전부 측정했고, 감추지 않는다

### C-1. `max(u)` 갈래는 구멍을 **좁힐 뿐 닫지 못한다**

이 갈래가 없으면 변이 C 는 **팔레트 없는 구조에 `pocket_occluded` 를 붙인다.** 그래서 넣었다. v1 가림 아홉 장면은 막히지 않은 쪽이 `max(u)` 1341·817·761·479·368·405·1105·679·3265 를 유지하고, 블록 아홉 개는 전 거리 0/0 이라 이 문턱이 둘을 가른다. EPAL 6 가림 기하에서도 확인했다 — 폭 비율 0.6·4.0 m 에서 `min(u)` 가 80 으로 문턱을 깨지만 `max(u)` 가 264 로 남아 갈래 1 이 걸리고 `invalid/pocket_occluded:left` 가 보존된다.

**그러나 한쪽에만 상한 증거가 있는 팔레트 아닌 구조는 여전히 샌다.** 블록 아홉 개 + 왼쪽 절반 위에 얹힌 상자(앞면 정렬) + 오른쪽 개구 앞 장애물 → 60 조합 중 **38 개가 `invalid`**(그 중 16 개가 `pocket_occluded:right`)인데 **HEAD 는 60 개 전부 깨끗한 true negative** 다. 음성 범주에서 `invalid` 는 `false_positive_rate` 에 안 잡히고 예산 없는 `counts.*.invalid` 로 간다(`evaluation.py:101-103, 170-173`). 이 누출은 얹힌 구조가 평면 inlier 대역(20 mm) 안에 있을 때만 생기고 19 mm 이상 물러나면 사라진다. **계획된 140 장면 중 이걸 잡을 장면은 하나도 없다.**

### C-2. 앞면 개방 구조는 위양성이다

**앞면이 팔레트와 같은 평면에 맞물린 개방 구조**(실질적으로 바닥판을 뺀 EPAL 6)는 2.5·3.0·3.5 m 에서 `u` 가 1033/1060 · 520/543 · 358/358 로 문턱의 3.5~10 배이고 `valid` 가 나온다(오차 4 mm). 임계 사고가 아니라 구조적 통과다.

**앞면이 물러난 변형은 리그 구성에 따라 갈린다. 어느 쪽도 주장하지 않는다.** 독립 리그 둘이 다른 답을 냈다 — 하나는 2.2~2.75 m·후퇴 0.10~0.23 m 에서 위양성을 재현했고, 다른 하나는 같은 범위에서 지지대가 문턱을 못 넘어 패턴이 서지 않았다. v5 가 "앞면 정렬이 이 음성의 전부다"라고 단정한 것은 **단일 측정의 과잉 일반화였다.**

**`floor_z_m` 이 현재 바닥판 신호를 지우고 있다.** EPAL 6 바닥판은 z 0–0.022 인데 작업영역 필터가 z > **0.020** 만 남긴다 — 22 mm 판에서 2 mm 만 살아남고 그것도 블록 앞면 하단과 섞인다. 문턱을 0.004 로 내리고 **기둥 아래**에서 전면 평면 위 z ∈ [0.006, 0.020] 을 세면 EPAL 6 가 2.0/3.0/4.0 m 에서 **208·284·208 / 75·105·75 / 32·46·30** 이고 바닥판 없는 구조는 **전 거리 0/0/0** 이다. 끝까지 돌리면 바닥판 없는 구조가 전 거리 `no_bottom_board` 로 거부된다.

**단 이것이 §C-2 를 풀지는 못한다.** 기둥이 **바닥까지 내려오는** 구조는 그 대역을 채우므로 여전히 통과한다(실측). 실물 랙 기둥은 바닥까지 내려온다. 그래서 이 특징은 *바닥에서 떠 있는 블록* 계열만 가르고, 팔레트와 랙은 못 가른다. **v6 의 "이 프레임에 둘 다 잡는 규칙이 없다" 는 너무 셌다 — 한 계열은 잡는다.** Task 8 에서 `floor_z_m` 을 EPAL 6 튜닝 대상에 넣는다. 실센서에서는 바닥 잡음이 0.004 를 넘나들 테니 별도 검증 항목이다.

### C-3. "가림이면 막히지 않은 쪽이 남는다" 는 **v1 가림 상자 모양의 성질**이다

v1 가림 상자는 항상 **개구 앞의 수직 상자**라 z 0.5 카메라에서 아래부터 가려 상한 대역을 구조적으로 못 지운다. 두 반례가 그 분리를 깬다.

- **머리 위 모서리**(랙 빔·선반): x 1.60 에 걸린 면의 하단이 시선을 자른다. EPAL 6 2.5 m, z ≤ 0.105 에서 자르면 **`u = (0,0)`** 인데 지지대는 626(문턱의 6.3 배)이고 두 개구는 완전히 보인다 → **`no_pallet/no_upper_deck`**. 즉 **삽입 가능한 실물 팔레트가 "팔레트 없음"으로** 나온다. 전이가 급하다: z 0.110 → u=(134,140) `valid`, 0.105 → (0,0).
- **양쪽 가림**: 같은 모양 상자를 두 개구에 하나씩(폭 비율 1.0, 간격 0.10) → `u = (0,0)`, 지지대 323 → `no_pallet`. 폭 비율 0.8 이면 u=(112,112) 로 문턱을 12 만큼 넘는다.

**그래서 "368 대 0" 은 규칙의 여유가 아니라 "장면당 가림 상자 하나" 카탈로그의 여유다.** EPAL 6 세트의 `occluded` 범주가 어떤 가림을 담을지 캡처 전에 정하고, 담는다면 `occluded` 가 `invalid` 에서 `false_negative` 로 새는 예산 줄을 §D-2 에 넣는다.

### C-4. 게이트가 팔레트의 가장 얇은 부재 위로 옮겨간다

`u` 는 **44 mm 높이 × 227.5 mm**(상판 + 스트링거) 대역에서 나오고 지지대는 58 mm × 100 mm(가장 좁은 기둥)에서 나온다. 즉 이 변경은 판정을 **스테레오 깊이 카메라가 가장 먼저 놓치는 부분**에 건다. 잡음 없는 리그에서는 안 보이는 위험이고 **D435i 검증 전까지 미검증 전제**다.

### C-5. 정직한 진술

이 변경은 판별 근거를 "개구 아래 재료"에서 "개구 위 재료"로 **옮긴다.** 현행은 실물 팔레트를 거부하는 대가로 개방 구조를 거부했고, 변경 후는 실물 팔레트를 검출하는 대가로 앞면 개방 구조를 통과시키며 머리 위 가림에서 실물 팔레트를 잃는다.

---

# D. 합격 조건

### D-1. v1 회귀 가드 (합격 조건이 아니다)

> v1 100 장면을 동결 파라미터로 재생했을 때 HEAD 와 다른 장면은 **`s056` 하나뿐**이고, `no_pallet/no_opening_pattern` → **`no_pallet/no_upper_deck`** 이다(둘 다 올바른 거부, 둘 다 `true_negative`). 나머지 99 장면은 `diagnostics` 를 제외한 전 필드가 동일하고 **포켓 좌표·yaw·σ 는 100 장면 전부 불변**이다.
>
> **`summarize` 는 한 필드도 움직이지 않는다.** `summarize(HEAD) == summarize(변이 C)` 다 — `counts` 전체(`occluded.invalid` 15 포함), `detection_rate`, `false_positive_rate`, 전 분위수, `targets.met`, `scene_count`, `splits` 모두 같다. `elapsed_s` 는 제외한다.
>
> 후보 수는 82 → 95 로 는다. `no_upper_deck` 은 **`s056` 에서 한 번 나오므로** 재현 시험이 이 사유를 덮는다.

*v6 이 적은 "`negative_lookalike` true_negative 10→9, invalid 0→1" 은 v5 규칙의 값이다. v6 에서 `max(u)` 갈래를 넣으면서 `s056` 의 결말이 바뀌었는데 델타를 갱신하지 않았다. 직접 재측정해 위 값으로 고쳤다.*

**이것은 합격 조건이 아니다.** v1 팔레트는 개구 200 mm·덱 50 mm 라 **고치려는 실패도, 새로 들이는 실패도 v1 에서는 발생할 수 없다.** 재생 입력·기대값이 둘 다 gitignore 라 새 clone 에서는 **조용히 skip** 된다.

### D-2. 새 세트의 합격 조건 (§A 답을 받은 뒤 수치로 채우고 캡처한다)

- (a) EPAL 6 양성: 위치 p95 ≤ 20 mm, yaw p95 ≤ 2°, 검출률 ≥ 95 %(로드맵 M2 목표). **§A ⑤ 를 먼저 풀지 않으면 2.5 % 를 잃고 시작한다.**
- (b) **eval 음성 전체를 합친(18 장) 위양성률 ≤ ____.** 범주별로 나누면 0/6 의 95 % 상한이 39 % 라 판정이 불가능하다.
- (c) `negative_open_bay` 를 찍는다면 예산 밖에서 개수만 기록한다(§A ①).
- (d) **음성의 `invalid` 를 별도 집계한다** — §C-1 이 새는 구멍이다.
- (e) `occluded` 범주가 `invalid` 에서 `false_negative` 로 새는 한도 — §C-3 의 가림을 담는다면 필요하다.

---

# E. 포켓 깊이 관문을 넣지 않는다 — 구현 기각, 요구는 존치

```
p_max = (개구 대역 상단 z − 터널 바닥 z) × R_perp / (카메라 높이 − 개구 대역 상단 z)
```

EPAL 6 은 계수가 **0.2195**(= 0.090 / (0.500 − 0.090))다. `range_min_m` 0.8 m 에서 **176 mm** 가 상한이다. 0.36 m 를 요구하면 v1 검출률 0.800 → 0.362 다. 안전한 값 0.09 m 는 실질 이득이 **40 mm** 이고 이름이 360 mm 를 약속한다.

**요구는 죽지 않는다.** 조사가 기록한 업계 방식(포크 장착 2D LiDAR·레이저)은 **이 프로젝트 하드웨어 계획에 없다** — `docs/hardware.md` 는 D435i·RPLIDAR 만 확정이다. Task 10 에서 `docs/hardware.md` 와 로드맵 위험표에 **"삽입 중 독립 접촉/근접 감지가 필요하며 H0/H1 에서 결정한다"** 를 남긴다.

---

# F. 전역 제약

- **터널 너머에 의존하는 신호를 만들지 않는다.** 관통 한계 `arctan(227.5/600) = 20.77°`.
- **v1 카탈로그·데이터 세트·prior 는 읽기만 한다.** `COUNTS` 에 범주를 더해 재생성하면 `rng.shuffle`(`generate_scene_catalogue.py:237`)이 밀려 **기존 100 장 중 0 장만 동일**하다. `sample_catalogue(20260911, 100)` 은 커밋된 YAML 과 같다.
- dev 에서만 튜닝, **eval 은 한 번**. `ruff check .` · `ruff format --check .` 통과.
- **기준 회귀:** `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error`
  - untracked 데이터가 **있으면** → **759 passed, 1 deselected, skip 0**(독립 확인 4회).
  - **없으면** → skip **3 개**(`test_detector_v1_replay.py` 2 + `test_evaluate_cli.py:219` 1)가 초록으로 보인다.
  - deselect 되는 1 개는 `test_preview_renders_distinct_views_and_records_evidence`(`rendering` 마크).
- **위임자에게 넘길 untracked 경로(이름 정확히):** `data/synthetic_scenes/catalogue_v1`, `artifacts/20260912T170442Z_pocket_eval_dev_02`, **`artifacts/20260912T170558Z_pocket_eval_eval_01`** — 두 아티팩트는 **타임스탬프가 다르다.**

---

# G. 작업

## Task 0: 기준 확인 (Claude)
- [ ] 트리 깨끗, 기준 회귀 초록(skip 0 인지 확인 — skip 이 있으면 데이터가 없는 것이다).
- [ ] 아래 덱 증거의 프로덕션 경로 값 2.0/3.0/4.0 m = **0/12/9** 를 기록한다.

## Task 0.5: §A 결정 수령 (Claude)
- [ ] ⓪·⓪′ → **Task 2 착수 조건**. ② → **Task 3 착수 조건**. ①·⑤ → **Task 4 착수 조건**.
- [ ] ⓪′ 를 묻기 전에 점수 함수 형태·수락 문턱·네 구조 점수표 반 페이지를 만든다.
- [ ] D-2 (b) 를 수치로 채운다.

## Task 1: 원격 경로 수리 (위임) — **모든 분기에서 필요**

**Files:** `tools/submit_model_check.py`, `tools/remote_model_job.py`, `tests/integration/test_evaluate_cli.py`, `tools/evaluate_pocket_detector.py`, `tests/integration/test_remote_model_jobs.py`

- [ ] snapshot 허용 목록에 `config`·`tests` 의 `.yaml` 추가. 실측 정확히 **6 개**(91 → 97), 비밀 파일 없음. 현재는 `tests/fixtures/thin_deck_legacy_*.yaml` 이 **모듈 import 시점**에 로드되어 원격 pytest 가 수집에서 죽는다.
- [ ] **`git_revision`:** `test_evaluate_cli.py:313` 이 길이 40 을 요구하는데 스냅샷에 `.git` 이 없다. **assert 를 완화하지 말고 스냅샷 생성 시 메타데이터로 주입한다.** 완화하면 `run.json` 에 `None` 이 실려 Task 10 을 산출물로 증명할 수 없다.
- [ ] **`scenes` 모드에 코어 패키지를 설치하지 않는다**(`remote_model_job.py:494-506`). Task 3 이 `forklift_core` 를 import 하지 않는다. **결정 완료 항목.**
- [ ] **완료 조건:** ① 원격 `model-cpu` 초록 ② `scenes` 모드 v1 1~2 장면 스모크. **②는 v1 5 상자 경로만 검증한다.**

## Task 2: 변이 C 구현 (위임) — **⓪·⓪′ 이후**

**Files:** `src/forklift_core/perception/pocket_detector.py`, `tests/unit/perception/test_thin_deck_evidence.py`, Create `tests/unit/perception/test_opening_evidence_cases.py`, `tests/integration/test_detector_v1_replay.py`, `docs/design/2026-09-13-pocket-detector-baseline.md`, `docs/design/2026-09-13-thin-deck-evidence.md`, `docs/design/2026-09-13-floor-through-opening.md`, `docs/validation/2026-09-13-thin-deck-evidence.md`, `docs/interfaces/pocket-observation.md`

- [ ] §B 의 규칙 1–5, 구현 주의, `upper_evidence_absent` 를 구현한다.
- [ ] **`no_upper_deck` 의 status 를 `no_pallet` 으로 두는 이유를 적는다.** 음성에서 `no_pallet` 은 `true_negative`, `invalid` 는 예산 없는 버킷이다(`evaluation.py:101-108`). 이웃한 `opening_width_mismatch` 가 `invalid` 인 것과 갈리므로 의도임을 명시한다.
- [ ] `_Pattern` 에 `upper_ok: bool` 과 `lower`·`upper_left`·`upper_right`·`support_min` 을 스칼라로 추가. `support_count` 는 합계라 최솟값 지표로 못 쓴다.
- [ ] **`deck_count = lower + upper_left + upper_right`, 점수식 `score = support_count + deck_count`(`:498`)는 그대로.** 실측: 점수는 16740 → 15726 으로 바뀌지만 **v1 100 장면에서 선택 결과는 한 장면도 안 바뀐다.** 이 재정의로 `test_thin_deck_evidence.py` 시험 **정확히 3 개**가 실패한다(`-231 == 0`) — 하니스가 `deck_count − upper` 로 `lower` 를 역산하는데 **현행에서 그 역산은 정확하다**(16740 − 7866 = 8874 = 실제 `lower`). 결함은 **재정의가 만드는 것**이고 명시 필드 직독이 그 수선이다.
- [ ] `DetectionDiagnostics` 의 새 필드는 **`exception_traceback` 앞**에. 뒤에 붙이면 클래스 생성 시점에 `TypeError` 로 import 가 깨진다. 두 dataclass 모두 위치 인자 생성(`:347`, `:537-551`).
- [ ] **`SCENE_COLUMNS` 는 건드리지 않는다.** `no_upper_deck` 은 관측 *사유*라 기존 `reason` 열(`evaluate_pocket_detector.py:48`)로 들어가고, `upper_evidence_absent` 는 거부 평면 사유라 `asdict(diagnostics)`(`:134-135`)로 관측 JSON 에 그대로 저장된다. **조용히 버려지는 것은 없다.** 새 열을 만들면 `test_evaluate_cli.py:110-134` 의 헤더 리터럴만 깨진다. *(v6 의 지시는 두 해석 모두에서 틀렸다.)*
- [ ] **bool `DetectorParams` 항목을 만들지 않는다.** `_finite_scalar` 가 bool 을 거부해 import 가 깨진다.
- [ ] **`test_detector_v1_replay.py`:** 기대 델타는 `s056` 한 장면(`no_pallet/no_opening_pattern` → `no_pallet/no_upper_deck`). **보관된 `run.json`·관측 파일을 고치지 말 것.** 예상 델타를 **추적되는 상수**로 넣는다.
- [ ] **새 시험 `test_opening_evidence_cases.py`** — `tests/fixtures/synthetic_scene.py` 와 `pallet_boxes(load_pallet_geometry(...))` 로 만든다. 최소 일곱 가지:
  - **EPAL 6** 2.0~4.0 m: `valid`, 포켓 y = ±0.186 ± 0.01.
  - **블록 아홉 개**: `no_pallet/no_upper_deck`, `u1 = u2 = 0`.
  - **블록 아홉 개 + 앞쪽 장애물**: `no_pallet/no_upper_deck` — 규칙 5 의 `max(u)` 갈래가 없으면 `invalid` 가 나온다.
  - **블록 아홉 개 + 한쪽 위에 얹힌 앞면 정렬 상자 + 반대쪽 장애물**(§C-1): 현재 규칙에서 `invalid` 가 나온다. **알려진 누출로 고정**한다.
  - **앞면 개방 구조**(flush / 후퇴, 상판·기둥 깊이를 매개변수로): flush 는 `valid`. 이름을 `test_known_limitation_flush_front_open_bay_is_accepted` 로 하고 검증 기록 한계 절에 올린다.
  - **머리 위 가림**(§C-3): z 컷 0.105 에서 실물 EPAL 6 가 `no_pallet/no_upper_deck` 이 되는 것을 **알려진 실패로 고정**한다.
  - **유령 패턴**: 팔레트 x 2.5 / **y = 0.80**(0.70 은 네 규칙 모두 정상이라 조용히 통과), 상자는 **화각 중심 쪽(−y)** 바깥 블록에서 0.22 m. **사용 기하를 시험 안에 명시한다.**
- [ ] 무효가 되는 계약 갱신: `baseline.md:66`(위·아래 덱 점유), **`:68`**(최고 점수 선택 — 규칙 4 가 깨뜨림), **`:70-76`, 특히 `:74`**(두 개구 열림 → `valid`), **`:123`**(§8 사용자 결정 1 "광선 검증으로 가림이 확인되면 `invalid`" — 규칙 5 의 둘째 갈래가 광선 결과와 무관하게 뒤집는다), `thin-deck-evidence.md` 의 `min(지지대 3개, lower, upper)`, `validation/thin-deck-evidence.md` 의 "deck_count 에서 upper 를 뺀다", **`floor-through-opening.md:13`**(그 설계를 기각한 근거가 이번에 고쳐 쓰는 시험이다 — 근거가 사라졌음을 남기지 않으면 다음 회차가 같은 설계를 다시 기각하거나 부활시킨다).
- [ ] §A ⑥(문턱)과 §E(깊이 닫힌 식)를 `thin-deck-evidence.md` 에 남긴다.
- [ ] **인터페이스 문서:** `pocket-observation.md:39` 에는 사유 어휘가 **없고** 기존 여섯 개도 안 적혀 있다. 검증하는 코드도 없다(`pocket_observation.py:93-94`). **일곱 개를 전부 적거나 이 항목을 빼고 설계 문서에만 적는다.**
- [ ] **인터페이스 문서에 한 줄:** `center_m` 의 **z 는 prior 유래 상수**(`:421`)이고 정답도 같은 값이라 **`position_error_m` 의 z 성분은 구조적으로 0** 이다.

## Task 3: 월드 생성기가 두 기하를 받게 한다 (위임) — **② 이후**

**Files:** `sim/gazebo/build_scene_world.py`, `tests/simulation/test_build_scene_world.py`, Create `tests/fixtures/catalogue_epal6_min.yaml`

- [ ] **먼저 epal6 헤더 계약을 정한다**(Task 4 보다 앞이다): `catalogue_version` 문자열, 헤더 `pallet` 6 필드에 넣을 값, 정답 포켓 규약(y ±0.18625, z 0.061, 폭 0.2275, 높이 0.078). Task 3 의 광선 시험이 이 규약을 요구한다.
- [ ] **최소 fixture 카탈로그를 만든다.** 현재 `tests/simulation/test_build_scene_world.py:11,24-26` 은 모든 fixture 를 커밋된 `catalogue_v1.yaml` 에서 뽑고 합성 생성기가 없어서, 이게 없으면 Task 3 은 **자기 합격 시험을 만들 수 없다.**
- [ ] **고칠 곳은 네 곳이다**(v6 은 두 곳이라 적었다): 범주 화이트리스트(`:78-83`), `lookalike` 존재 규칙(`:88`), **버전 게이트(`:161`, `catalogue_version != "v1"` 이면 거부 — 안 풀면 22 상자 분기가 영원히 죽은 코드)**, **`lookalike` 배출부(`:252-260`, `category` 로 분기해 블록 아홉 개/개방 구조를 따로 낸다)**.
- [ ] **새 장면 키를 만들지 않는다.** `_require_keys`(`:24-26`, `:59-71`)는 카탈로그 공용 집합 동일성이라 키 하나를 더하면 v1 100 장면이 전부 거부된다. 새 음성은 기존 `lookalike` 의 `{x_m, y_m, yaw_rad}` 를 쓰고 종류는 `category` 로 구분한다. **한 범주 안에 두 구조 변형을 넣을 수 없다** — §A ①(b) 를 택하지 않는다면 변형마다 범주를 나눠야 하고 분할 산수가 바뀐다.
- [ ] **치수 출처는 `sim/models/epal6_pallet/pallet.urdf` 직독이다.** 이미 스냅샷 허용 목록에 있고(확인됨) 22 visual + 22 collision 을 이름·크기·원점과 함께 갖는다. `ET` 는 `build_scene_world.py:5` 에 이미 import 돼 있다. YAML↔URDF 드리프트는 `tests/integration/test_pallet_model_build.py:191-196` 이 바이트 비교로 이미 막는다.
- [ ] **경로는 `Path(__file__).resolve().parents[N]` 기준.** `capture_scenes.py:226-236` 이 `cwd=runtime`(스냅샷 밖)에서 띄우므로 cwd 상대경로는 로컬만 통과하고 원격에서만 죽는다.
- [ ] **v1 경로는 그대로 둔다.** `test_build_scene_world.py:37`(collision 5), `:40`(outer 2), `:144`(스페이서 이름 집합)이 계속 통과해야 한다. **계속 통과해야 하는 3 파일:** `test_build_scene_world.py`, `test_capture_scenes.py`, `test_scene_catalogue.py`.
- [ ] `sdf_parts.add_urdf_visuals` 는 재사용 불가다(시그니처에 pose 없음, 모델명 하드코딩 `:69`, collision 없음).
- [ ] §A ④ 대로 `negative_lookalike` 상자(`:260`)를 처리한다.
- [ ] **시험: 정답 포켓 중심을 지나는 광선이 팔레트 상자 어느 것과도 교차하지 않는다.**

## Task 4: 카탈로그 변환과 새 음성 (위임) — **①·⑤ 이후**

**Files:** Create `tools/retarget_scene_catalogue.py`, `tests/unit/test_retarget_scene_catalogue.py`, `sim/gazebo/scenes/catalogue_epal6.yaml`; Modify `src/forklift_core/perception/evaluation.py`, `tools/merge_scene_batches.py`, `tools/evaluate_pocket_detector.py`, `sim/gazebo/build_scene_world.py`, `tests/simulation/test_scene_catalogue.py`, `docs/interfaces/scene-dataset.md`, `docs/design/2026-09-11-pocket-observation-and-scene-set.md`, `docs/validation/2026-09-11-scene-catalogue-and-world.md`

- [ ] v1 의 자세·범주·분할·조명·표면·distractor 를 그대로 옮긴다. **자세 완전 일치를 시험으로 고정한다**(approx 아님). **§A ⑤ 의 두 장면은 그 답대로 처리한다.**
- [ ] **재계산·재설정 대상 전수:**
  - 헤더: `catalogue_version` → epal6, `generator`, `seed`, `ranges.opening_width_m`
  - **`source_provenance` 는 `synthetic` 으로 유지한다** — `build_scene_world.py:162`, `scene_dataset.PROVENANCES`, `merge_scene_batches._verify_scene` 세 곳이 고정한다. *(v6 은 이걸 바꿀 값 목록에 넣었다. 틀렸다.)*
  - 헤더 `pallet` 6 필드 — Task 3 이 정한 값(`load_catalogue:174-181` 이 완전일치를 요구)
  - `pallet.opening_width_m` → **0.2275 고정**(v1 은 80 장면에 78 가지 값). 검증기 `0.20 <= w <= 0.28`(`:108`) 안, epal6 prior `[0.2075, 0.2475]` 안.
  - `ground_truth.{left,right}.center_m` — **x 성분도 바뀐다**(정답은 `pallet + R(yaw)·(−0.3, ±offset)`). y ±0.18625, **z 0.15 → 0.061**
  - `.width_m` → 0.2275, `.height_m` **0.20 → 0.078**
  - `occluder.center_m`·`size_m` — `size[1] = fraction × opening_width_m`
  - `visibility.corners_px`, `occluded_fraction_image`(**이미 있는 필드**, s001 = 0.431511), **`left_in_view`·`right_in_view`·`occluded_side`**
- [ ] **`fraction` 은 물리 폭 비율이지 영상 가림률이 아니다**(s001: 0.3553 × 0.2474 = 0.087901). 인터페이스 문서에 적는다.
- [ ] 새 음성을 §A ①의 답대로 만든다. 분할은 `len(indices) * 7 // 10`. `_scene_ids` 정규식 `s[0-9]{3}`.
- [ ] 데이터 세트는 `data/synthetic_scenes/catalogue_epal6/`. **`catalogue_v1` 을 덮지 않는다** — 두 `run.json` 의 `dataset_dir` 가 그 경로라 재현 시험의 **주 경로**가 깨진다.
- [ ] 범주 등록 **네 곳**: `evaluation.py:18`, `merge_scene_batches.py:41-46`(새 음성 `ground_truth.status` 는 `no_pallet`), `build_scene_world.py:78-83`, `evaluate_pocket_detector.py:292-296`.

## Task 4.5: epal6 스모크 게이트 (Claude + 원격)
- [ ] **epal6 카탈로그로 1~2 장면을 원격 캡처한다.** Task 1 의 스모크는 v1 경로만 본다. 없으면 22 상자 경로·URDF 직독·`cwd=runtime` 문제가 **되돌릴 수 없는 캡처에서 처음** 드러난다. Task 5 의 명령에서 `--dry-run` 만 빼면 된다.

## Task 5: 동결 게이트 (Claude)
- [ ] `merge_batches` 는 배치 manifest 를 `expected_metadata`(`:174-187`)와 완전일치시키고 거기에 `catalogue_sha256` 뿐 아니라 **`source_snapshot_sha256`·`image_id`** 가 있다. 배치 1 부터 병합까지 **스냅샷 허용 목록의 어떤 파일도** 건드리면 전 배치가 폐기된다. 전수 커버리지도 요구한다(`:220-222`).
- [ ] **확인 명령(실제로 동작 확인됨, 원격 접속 불필요):**
  `python tools/submit_model_check.py submit --host <h> --remote-root <r> --source . --mode scenes --image <id> --catalogue <c> --scene-range s001-s002 --run-id probe --dry-run` → `source.snapshot_sha256` 출력. 캡처 시작 전과 각 배치 전에 같은 값인지 본다.
- [ ] 배치는 **중간 발견이 한 배치만 버리도록 작게** 나눈다. 재캡처 예산을 미리 잡는다(공유 `kang` 계정·GPU 대기).
- [ ] **캡처 중 Task 7 의 파라미터 파일 작업을 시작하지 않는다.**

## Task 6: 캡처·병합 · Task 7: 동결 파라미터 정리 (Claude)
- [ ] 캡처·병합·set manifest·권한 확인.
- [ ] `config/detector_params_v1.yaml` 은 **16 키**, `DetectorParams` 는 **17 필드**(`deck_evidence_tol_m` 누락, 기본 0.006). `_load_params` 는 없는 키를 조용히 기본값으로 채운다. **동결 파일을 고치지 않는다** — 두 과거 `run.json` 도 16 키라 `test_detector_v1_replay.py:31` 이 지금 True 이고 키를 더하면 False 가 된다. 시험을 **"파일 ⊆ run.params, 나머지는 코드 기본값"** 으로 바꾼다.

## Task 8: dev 튜닝 (Claude)
- [ ] 기준선 1 회 후 한 번에 하나씩.
- [ ] 지표에 **지지대 최솟값**을 넣는다. 4 m 에서 `min()` 을 결정하는 것은 `upper`(692)도 `u1/u2`(256/268)도 아닌 **지지대**(144/207/144, 최소 1.44 배)다.
- [ ] **위양성 지표에 `invalid` 를 병기한다.**
- [ ] **`floor_z_m` 을 EPAL 6 튜닝 대상에 넣는다**(§C-2). 현행 0.020 이 바닥판 신호를 지운다. 내리면 바닥판 없는 구조를 가르지만 바닥까지 내려오는 기둥은 못 가른다. **실센서 바닥 잡음은 별도 검증 항목이다.**
- [ ] **`floor_z_m >= deck_bottom_m` 을 설정 오류로 거부**한다(현행 0.020 대 EPAL 6 0.022, 2 mm 차). `DetectorParams.__post_init__` 은 prior 를 못 보므로 **`detect_pockets` 진입부와 `evaluate_pocket_detector._run` 두 곳**에 넣는다.
- [ ] **처리 시간은 합격 조건이 아니다.** 후보가 82 → 95 로 는다. 같은 기계에서 순차 best-of-3 는 +56 %, 장면별 교차 best-of-3 는 −0.3 % 다 — 부하 드리프트가 신호를 압도한다.

## Task 9·10: eval 1 회와 기록 (Claude)
- [ ] 고정 revision·clean 트리에서 eval 1 회. **§D-2 의 (a)~(e) 로 합불을 적는다.**
- [ ] 검증 기록: 위양성률과 `invalid` 병기 / **§C 의 한계 다섯 개 전부**(구멍이 좁혀졌을 뿐 닫히지 않았다는 것, 앞면 개방 구조, 머리 위 가림에서 실물 팔레트를 잃는다는 것, 얇은 부재 위에 게이트를 걸었다는 것, 판별 근거가 옮겨간 것) / `position_error_m` 의 z 성분 구조적 0 / `POSITION_TOLERANCE_M` 0.20 m 는 대응 문턱이지 도킹 허용치가 아니라는 점(포크 좌우 45 mm·수직 6 mm 와 무관) / §A ⑥ 의 문턱 절벽과 `range_max_m` 5.0 도달 불가 / 해결하지 않은 것들.
- [ ] `docs/hardware.md` 와 로드맵 위험표에 §E 의 접촉/근접 감지 요구를 남긴다.

---

# H. 이 계획이 해결하지 않는, 검증 기록이 남긴 것

`docs/validation/2026-09-13-pocket-detector-m2.md` 가 미해결로 남긴 네 가지 중 이 계획이 다루는 것은 하나 반이다.

| 미해결 | 이 계획 |
|---|---|
| **s009** — 거부될 후보 평면이 정답 평면의 802 점을 먼저 소비하는 **구조적 결함**(`:68-70`) | **두 번째로 미룬다.** 이것은 **양성**을 해치는 결함이다 |
| `negative_lookalike` 가 가장 약한 고리(`:72`) | 부분적으로 다룸(블록 아홉 개, §A ④). 기록이 지목한 "깊이 결측으로 가짜 틈" 장면은 없음 |
| σ(`position_sigma_m`·`yaw_sigma_rad`)가 여전히 `None`(`:74`) | 안 다룸 — **그런데 M3 추적이 신뢰도로 게이트하려면 이게 필요하다** |
| 근접 1.1 m 미만·기울어진 팔레트·다중 팔레트 미시험(`:74`) | 안 다룸 — **§A ② 와 직결된다.** 브리프 Case C·D 가 바로 그 거리다 |

# I. 범위 밖 (의도)

터널 너머 신호(금지), 포켓 깊이 관문(구현 기각 — 요구는 §E 존치), 형판 정합·PnP, 딥러닝, 마커, 다중 시점 융합(M3). 게이트 아키텍처 재설계는 **§A ⓪′ 의 결정 사항이지 범위 밖이 아니다.**

---

# J. 자체 검토 기록

독립 검토 **18회**(v1~v7). 각 회차는 서로의 결론을 보지 않았다.

**6회차(v6 대상, 3건)가 잡은 것과 처리:**

| 지적 | 처리 |
|---|---|
| **§2a 가 v6 규칙에서 낡았다** — `s056` 은 `no_upper_deck` 이 되고 **집계가 한 필드도 안 움직인다** | 직접 재측정해 §D-1 전면 교체. v6 에서 규칙을 바꾸고 델타를 안 고친 내 잘못 |
| `max(u)` 갈래가 구멍을 **닫지 못한다** — 한쪽 상한 증거가 있는 비팔레트가 60 조합 중 38 개에서 `invalid` | §C-1 신설. 알려진 누출로 시험에 고정 |
| "가림이면 반대쪽이 남는다"는 **v1 가림 상자 모양의 성질** — 머리 위 모서리나 양쪽 가림은 실물 팔레트를 `no_pallet` 으로 만든다 | §C-3 신설. 알려진 실패로 시험에 고정, §D-2 (e) 추가 |
| **"HEAD 는 125 자세에서 0 검출" 이 틀림 — 12 검출** | 표·문제 진술 수정 |
| **`u/지지대` 비 불변이 정면 표본에만 성립**(전 범위 0.24~1.79) | §A ⑥ 에서 철회 |
| 변이 C 와 `min` 은 **검출 차이가 없다**(525 조합에서 209 대 209) | §B 표 아래 명시 |
| 규칙 2 가 더 큰 모호성 둘을 남김: **점 집합**(1.85 배)·**z 상한**(3 배) | 규칙 2 에 못 박음 |
| 규칙 5 를 문자 그대로 구현하면 `ValueError` | 구현 주의 추가 |
| `upper_evidence_absent` 해석 셋 | (c) 로 확정, v1 에서 0 회 발화 명시 |
| `SCENE_COLUMNS` 지시가 **두 해석 모두에서 틀림** | 지시 반전 |
| 재타깃으로 **2 장면이 화면을 벗어남**(s058 positive, s100 occluded) | §A ⑤ 신설 |
| Task 3 이 순서대로 완료 불가 — `:161` 버전 게이트 누락, "두 곳"이 실제 네 곳, 자기 fixture 없음 | Task 3 전면 수정, fixture 파일 추가 |
| 게이트가 **엉뚱한 Task 를 막음** — ⓪·⓪′ 는 Task 2, ② 는 Task 3 | Task 0.5 를 세 갈래로 분리 |
| ⓪′ 비용이 Task 2·8 에 갇힌다는 주장이 틀림 | §A ⓪′ 에서 정정 |
| ⓪′ 에 **기록된 반대 근거**가 있고 내 "위양성이 사라진다"는 **근거 없음** | 철회하고 선결 조건 제시 |
| `source_provenance` 는 바꿀 수 없음(검증기 3 곳) | Task 4 에서 정정 |
| 음성 통계력 — eval 6 장에서 0/6 의 95 % 상한 39 % | §A ①(a), §D-2 (b) 합산 예산 |
| `negative_open_bay` 20 장이 **결정을 못 바꾸는 일**(busywork) | §A ①(b) 선택지로 제시 |
| 계약 갱신 목록에 `baseline.md:123`·`floor-through-opening.md:13` 누락 | 추가 |
| 아티팩트 이름 오기(타임스탬프 다름), 조용한 skip 지점이 **둘** | §F 수정, 양쪽 기준선 수치 기록 |
| 행 번호 드리프트(`:515-517`, `:498`) | 수정 |
| `min_band_points` 선택을 여섯 회차 동안 안 물음 | §A ⑥ 신설 |
| s009·σ·근접 거리 미해결이 계획에 안 보임 | §H 신설 |

**내가 직접 재계산한 것:** v6 규칙의 v1 델타(집계 불변·`s056` → `no_upper_deck`), EPAL 6 가림 기하의 `min(u)`·`max(u)`·판정, `floor_z_m` 을 내렸을 때의 바닥판 분리(208/284/208 대 0/0/0)와 그것이 바닥까지 내려오는 기둥은 못 가른다는 것, 거리별 지지대 감쇠와 `range_max_m` 도달 불가, 입사각 통계(30/80), 기준 회귀 skip 수, 아티팩트 디렉터리 이름, 카탈로그 키 집합·`lookalike` 검증·하드코딩 상자, `_build_observation` 검사 순서, 로드맵 M2·일정 원문, 검증 기록 §8 의 점수화 기각 근거.

**반박되지 않은 것:** §D-1 의 회귀 결과는 네 번 독립 재현되었고(후보 수 82/70/79/95 포함), §C-2 의 flush 위양성과 §B 의 유령 위양성도 각각 여러 번 독립 재현되었다.
