# DLS08 실측 반영 지게차 모델

입고한 차체를 팀이 잰 값(2026-09-29–30)을 [잠정 모델](../dls08_provisional/README.md)에 반영한
모델이다. 같은 생성기(`tools/build_forklift_model.py`)로 만든다. 잠정 모델과 그 산출물은 Gazebo
동결 기준선과 과거 기록이 참조하므로 그대로 두었다. 계획과 검토 기록:
[2026-09-30 계획](../../../docs/plans/2026-09-30-measured-chassis-model.md).

```bash
python tools/build_forklift_model.py \
  --parameters sim/models/dls08_measured/parameters.yaml --output sim/models/dls08_measured
```

## 필드별 출처

★ 실측 · △ 실측과 잠정 배치에서 계산 · ▽ 추정 유지 · ◇ 시험 설정값.
실측 원본: `forklift-presentations/week-05/build_deck.py` 의 `MEASURE_ROWS`, 같은 주 `SCRIPT.md` 2·3쪽.

| 필드 | 값 | 출처 |
|---|---|---|
| 전장 × 전폭 × 전고 · 질량 | 1.46 × 0.63 × 1.01 m · 24 kg | ★ 카탈로그 값과 같음을 실측으로 확인 |
| 축간 거리 | 0.66 m | ★ |
| 윤거 | 0.53 m | ★ 중심 간 거리로 **해석** |
| 바퀴 반지름 | 0.125 m | ★ |
| 최대 조향각 | 15° | ★ 한쪽 바퀴를 잰 값. **두 바퀴 모두의 한계**로 배정 |
| 포크 길이 · 폭 · 두께 | 0.36 · 0.055 · 0.025 m | ★ 길이는 뿌리부터 끝까지로 **해석** |
| 승강 범위 | 0.28 m | ★ |
| 뒤차축 x | −0.34 m | ▽ base_link 기준 위치 미측정 — 기준으로 고정 |
| 앞차축 x | 0.32 m | △ −0.34 + 0.66 |
| 후단 x | −0.51 m | ▽ 뒤차축–후단 미측정 |
| 포크 뿌리 x | 0.59 m | △ 포크 끝 0.95(= 후단 + 전장) − 0.36 |
| 차체 앞면 · 마스트 x | 0.50 · 0.54 m | △ 잠정값 + 0.06, 포크 뿌리 이동과 같은 양 |
| 바퀴 폭 | 0.08 m | ▽ 윤거 0.53 에서 볼트 장식이 전폭 안에 들도록 0.10 → 0.08 |
| 포크 중심 간격 | 0.29 m | ◇ 조절 범위 중심 0.055–0.38 m(안쪽 0–0.325 m, 수동) 안의 EPAL 6 시험 설정 |
| 그 밖의 치수·질량 분배·동역학 | 잠정 모델과 같음 | ▽ |

## 모델에서 나오는 값

| 값 | 결과 | 비고 |
|---|---|---|
| 뒤차축 중심 최소 회전 반경 | 2.728 m (κ 0.3665 1/m) | 두 바퀴 15°. 15° 가 바깥 바퀴였다면 2.198 m, 자전거 모델로는 2.463 m — 회전 원 실측 전까지 미확정 |
| 캐리지 삽입 한계 | 0.346 m | 포크 끝 − 가로대 앞면(두께 0.028 은 추정) |
| 삽입 목표 `min(깊이 × 0.6, 한계 − 0.046)` | EPAL 6 · T11 × 0.6 모두 0.300 m | 잠정 모델은 0.360 m |
| 차축–포크 끝 | 1.29 m | 잠정과 같음. "포크 끝 = 전장 끝"·뒤차축 −0.34 두 추정 위의 값 |

## 쓰는 곳

- Isaac 운반·SLAM(`run_transport.py`, `run_slam_drive.py`): `--forklift-urdf` 에 이 URDF 를,
  `--settings config/isaac_transport_measured.yaml` 을, 이 URDF 로 만든 base scene
  (`sim/isaac/build_base_scene.py`)을 함께 준다. 실행기가 장면 관절과 URDF, 설정 곡률과 조향
  한계를 대조해 어긋나면 시작하지 않는다.
- CPU 도구(`preview_docking`, `build_t11_drawing`, `factory_planning_sweep` 등), G1 관문,
  `run_perception_approach`·`run_pocket_insertion` 은 아직 잠정 모델 기준선이다.

## 다음 실측 요청

윤거·포크 길이·15° 의 측정 기준, 뒤차축–포크 끝 거리, 캐리지 가로대 앞면 위치, 포크 최저·최고
높이, 뒤차축–후단 거리, 회전 원.
