# 검증 기록: 포켓 증거 측정 리그와 개구별 상부 덱 규칙 (2026-09-14)

**무엇을 확인했는가.** ① 측정 리그(`tools/scene_rig.py`)가 동결 시험 픽스처와 **비트 단위로 같은** 장면을 렌더한다 ② 측정 도구(`tools/measure_pocket_evidence.py`)가 계획의 수치를 **커밋된 코드에서** 재생산한다 ③ 개구별 상부 덱 규칙이 v1 회귀를 깨지 않고 가림 사유를 더 정확하게 만든다.

**무엇을 확인하지 않았는가.** 실센서 데이터는 없다. 리그는 1 mm 깊이 양자화 외에 잡음이 없고, 카메라 외부 파라미터 오차가 구조적으로 0 이며, 정답의 z 는 prior 상수와 같은 값이라 **위치 오차의 z 성분이 구조적으로 0** 이다. 아래 수치는 전부 그 조건에서의 값이다.

## 1. 리그가 픽스처와 같은가 — 같다

`tests/unit/tools/test_scene_rig.py`. 빈 장면과 팔레트 장면(2.4 / 3.0 / 4.2 m)에서 `tests/fixtures/synthetic_scene.py` 와 **최대 절대차 0.0**, NaN 위치까지 일치한다.

그리고 **회전이 강체인지**를 따로 고정했다. 이전 스크래치 리그는 상자 중심을 두고 슬래브만 제자리에서 돌렸고, 그것 때문에 커밋된 60 자세 결과가 16 에서 1 로 바뀐 것을 세 회차 동안 아무도 못 봤다. 시험이 두 렌더의 깊이 차가 0.1 m 를 넘는 것을 단언한다.

## 2. 왜 채택 형상이 검출되지 않는가 — `lower` 하나 때문이다

```
$ python tools/measure_pocket_evidence.py evidence --x 3.0
# rule=head seeds=12 quantize=True camera=(0.75, 0.0, 0.5) tilt=0.0
# geometry=pallet_geometry_t11_06.yaml prior=pallet_prior_t11_06.yaml overrides=-
workspace points: 5130   plane candidates: 1

plane 0: inliers 1802 residual_p95 8.28 mm distance 1.921 m  occupied cells 24 gaps 2
  pattern spacer   80.0 mm  supports (120, 114, 120)  lower      0  upper    894
                            gate min      0 vs min_band_points 100  -> REJECT

observation: no_pallet / no_opening_pattern
```

**지지대는 문턱의 1.14 배, 상부 덱은 8.9 배인데 `lower` 가 0 이라 거부된다.** EPAL 6 도 같다 — 지지대 (349, 490, 349), 상부 1441, `lower` **0**. 두 형상 모두 개구가 바닥까지 뚫려 있어 개구 아래에 아무것도 없다.

**이 값이 표에 찍히는 것 자체가 이번 작업의 산출이다.** 계획은 두 회차 동안 `lower = 0` 을 못 보고 파라미터를 의심했다.

## 3. `floor_z_m` 의 상한이 실재한다 — 동결값이 바닥판을 통째로 지운다

`slab` 구조(바닥이 연속인 v1 형)로 바꿔도 T11 은 여전히 `lower = 0` 이다. 원인은 형상이 아니라 **`floor_z_m` = 20 mm 가 T11 의 15 mm 바닥판 전체를 z 필터에서 삭제**하는 것이다.

| `floor_z_m` | T11 `slab` 2.4 / 3.0 / 3.6 / 4.2 m |
|---|---|
| 20 mm (동결) | 0/3 · 0/3 · 0/3 · 0/3 |
| **5 mm** (`max(0.004, deck_bottom_m/3)`) | **3/3 · 3/3** · 0/3 · 0/3 |

**계획 §C-18 이 유도한 상한 `floor_z_m ≤ deck_bottom_m − deck_evidence_tol_m`(T11 에서 9 mm)이 커밋된 코드에서 확인된다.** 동결 20 mm 는 그 상한의 두 배가 넘는다.

**그리고 5 mm 로 내려도 3.6 m 이상에서 다시 죽는다** — 이것은 별개의 항(`plane_inlier_m` 20 mm 가 90 mm 팔레트에서 블록 전면과 상판 모서리를 한 평면에 섞는 것)이고 §C-12 가 "빗살" 로 적은 그 현상이다. **두 문제가 서로 독립임이 여기서 분리된다.**

EPAL 6 는 `slab` 에서 동결값 그대로 2.4 m 2/3, 3.0 ~ 4.2 m 3/3 이다 — 바닥판이 22 mm 라 20 mm 필터를 겨우 넘는다.

## 4. 근접 미검출은 검출기가 아니라 카메라 화각이다

```
$ python tools/measure_pocket_evidence.py fov --distances 1.6:2.4:0.2 --seeds 3
# vertical half-angle tan = 0.51531, horizontal = 0.68708
# band bottom z= 15.0 mm visible from range 0.941 m  -> placement x >= 2.021 m
# band top    z= 60.0 mm visible from range 0.854 m  -> placement x >= 1.934 m
# full width            visible from range 0.480 m  -> placement x >= 1.560 m

    x_m   valid   band_px   deck_px  reason
  1.600    0/3        128     24128  no_pallet/no_front_plane
  1.800    0/3        260     29113  no_pallet/no_front_plane
  2.000    0/3       2495     26972  no_pallet/no_opening_pattern
  2.200    0/3       2187     16771  no_pallet/no_opening_pattern
  2.400    0/3       1548     11449  no_pallet/no_opening_pattern
```

**1.8 m 에서 팔레트가 돌려주는 화소의 99 % 가 위에서 본 상판이고 기둥 대역은 260 개뿐이다.** 닫힌 식이 경계를 2.021 m 로 예측하고 대역 화소가 2.0 m 2495 → 1.8 m 260 → 1.6 m 128 로 무너지며 사유도 `no_opening_pattern` → `no_front_plane` 으로 바뀐다.

**어떤 파라미터도 이것을 되돌리지 못한다** — 전면이 영상 밖이면 평면이 없다. 되돌리는 것은 카메라 장착이고, **수평 화각이 660 mm 팔레트를 자르는 1.56 m 아래로는 이 초점거리로 불가능하다.** 브리프 Case C·D 와 삽입 마지막 구간이 그 안에 있다(계획 §C-19).

## 5. 개구별 상부 덱 규칙 — v1 회귀

동결 v1 100 장면 재생(`tests/integration/test_detector_v1_replay.py`):

| | 장면 수 |
|---|---|
| 상태가 바뀐 장면 | **0** |
| 사유만 바뀐 장면 | **9** (`pocket_occluded:{side}` → `upper_deck_occluded:{side}`) |
| 좌표가 바뀐 장면 | **0** |
| 이미 기록돼 있던 델타 | 1 (`s009`, 평면 추출 수정) |

**아홉 장면 모두 광선 검증과 같은 쪽을 지목했다.** 새 증거가 가림 방향을 독립적으로 같게 판정한다. 선택 결과는 100 장면 전부 불변이다(점수식의 `deck_count` 정의가 `lower + upper` 에서 `lower + upper_left + upper_right` 로 바뀌었는데도).

**부작용 하나를 고쳤다.** 얇은 덱 하니스가 `lower` 를 `deck_count − upper` 로 역산했는데 그 산술은 우연히 정확했던 것이고, 재정의 후 **−231** 을 보고했다. `_Pattern.lower` 직독으로 바꿨다.

## 한계

- **실센서 없음.** D435i 규격 오차는 거리의 약 2 % 로 2.5 m 에서 ~50 mm 이고, 이 리그가 낼 수 있는 어떤 잡음보다 크다. 위 수치는 전부 무잡음이다.
- **`floor_z_m` = 5 mm 의 4 mm 하한 클램프는 이 리그의 1 mm 양자화에서 나온 값이다**(계획 §C-18 (3)). 실센서에서 재측정해야 한다.
- **채택 형상은 아직 검출되지 않는다.** Task 2 는 계획의 축소 분기(②)로 구현했다 — `lower` 가 관문에 남아 있다. 규칙 1(관문에서 `lower` 제거)은 카메라 장착(K-2)이 정해진 뒤에 다시 본다.
- **`slab` 은 실물 형상이 아니다.** EPAL 도 T11 도 개구가 바닥까지 뚫려 있다. 이 구조는 동결 검출기가 튜닝된 형상이고, 상부 덱 규칙을 시험하기 위해 존재한다.

## 재현

```bash
export PYTHONPATH=src:.
python tools/measure_pocket_evidence.py evidence --x 3.0
python tools/measure_pocket_evidence.py evidence --structure slab --x 3.0 --set floor_z_m=0.005
python tools/measure_pocket_evidence.py fov --distances 1.6:2.4:0.2 --seeds 3
python tools/measure_pocket_evidence.py structures --distances 2.4:4.2:0.6 --seeds 3
python -m pytest tests --ignore=tests/simulation -q
```
