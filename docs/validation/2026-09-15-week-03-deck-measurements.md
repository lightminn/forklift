# 검증 기록: 3주차 발표 자료가 인용하는 재측정 (2026-09-15)

**무엇을 확인했는가.** ① 축소 T11 커밋 자세 60 개에서 동결값과 유도값의 검출 수를 **커밋된 코드로** 다시 쟀다 ② 카메라 높이 0.50 m 에서 바닥이 화면에 들어오는 최근접 거리를 구하고 그 거리 앞뒤에서 검출기가 갈리는 것을 확인했다 ③ 발표용 그림을 MuJoCo 자유 카메라로 그릴 수 없는 이유를 확인했다.

**무엇을 확인하지 않았는가.** 실센서 데이터는 없다. 리그는 1 mm 깊이 양자화 외에 잡음이 없고 외부 파라미터 오차가 0 이다. 아래 수치는 전부 그 조건의 값이다.

## 1. 축소 T11 커밋 자세 60 개 — 동결 0/60, 유도 58/60

```
# 동결값
$ python tools/measure_pocket_evidence.py poses --prior config/pallet_prior_t11_06.yaml \
    --set floor_z_m=0.02 --set plane_inlier_m=0.02 --set min_band_points=100 \
    --set min_plane_points=300 --set band_margin_m=0.01
# all-seed 0/60  x<=3.0: 0/17  x>3.0: 0/43  worst accepted 0.0 mm

# 유도값 (DetectorParams.derived_for 가 이 prior 에서 내는 값)
$ python tools/measure_pocket_evidence.py poses --prior config/pallet_prior_t11_06.yaml \
    --set floor_z_m=0.005 --set plane_inlier_m=0.0045 --set min_band_points=5 \
    --set min_plane_points=15 --set band_margin_m=0.00225
# all-seed 58/60  x<=3.0: 17/17  x>3.0: 41/43  worst accepted 509.1 mm
```

`config/detector_params_v1.yaml` 이 `poses` 의 기본 파라미터이므로 **옵션 없이 돌리면 동결값이 적용된다** — 유도값을 재려면 위처럼 다섯 항을 전부 넘겨야 한다. 다섯 항의 유도값은 `DetectorParams.derived_for(load_pallet_prior('config/pallet_prior_t11_06.yaml'))` 의 출력과 같다.

⚠️ **이 기록 이전에 발표 자료가 인용하던 "16/60 → 59/60" 은 쓰지 않는다.** 59/60 은 `floor_z_m` 3.75 mm 의 값인데 [ADR 0002](../decisions/0002-test-pallet-and-geometry-generality.md) §"네 번째 항"이 **3.75 mm 를 내는 규칙이 없다**며 인용을 금지했고, 16/60 은 어느 파라미터 조합에서도 재현되지 않았다.

⚠️ **채택 유도값이 받아들인 자세 중 최악 오위치가 509.1 mm 다.** 검출 수만 인용하고 이 값을 빼면 결과를 과장하게 된다. 발표 자료는 두 값을 함께 말한다.

## 2. 카메라 높이 0.50 m 의 최근접 바닥 — 0.97 m

리그 카메라의 세로 반각 탄젠트는 `tools/deck/render.py` 의 실측값 `V_TAN = 0.51531` 이다. 광축이 수평일 때 화면 아래 끝이 바닥에 닿는 거리는 `0.50 / 0.51531 = 0.970 m` 이고, **이보다 가까운 바닥은 화면에 들어오지 않는다.** 팔레트 앞면은 그 바닥에 붙어 있으므로 같이 사라진다.

같은 리그로 EPAL 6 을 정면에 놓고 잰 검출 결과:

| 카메라–앞면 거리 | 앞면 위치 | 검출기 판정 |
|---|---|---|
| 1.20 m | 0.97 m 보다 멀다 | `valid` (포켓 검출) |
| 0.60 m | 0.97 m 보다 가깝다 | 미검출 — 윗판만 보인다 |

재현: `python tools/deck/near_field_blind.py` (그림 `artifacts/week03_assets/06_near_field_blind.png` 를 만들면서 같은 판정을 출력한다).

## 3. MuJoCo 자유 카메라는 이 그림에 쓸 수 없다

`<visual><global fovy="54.525"/></visual>` 로 리그와 같은 화각을 지정해도, 렌더된 화면에서 역산한 세로 반각 탄젠트는 **0.344** 로 지정값 0.515 의 0.668 배다. 이 비율은 fovy 45 / 54.525 / 70 / 90° 와 화면 비율 620×470 / 470×470 에서 모두 같게 나타났다.

그래서 **같은 장면에서 그림과 검출 판정이 어긋난다** — 자유 카메라 화면에서는 1.20 m 에서도 앞면이 안 보이는데 리그 판정은 `valid` 다. 11 쪽 그림은 화면과 판정을 모두 리그의 거리 영상에서 만든다. 자유 카메라는 옆모습·외부 시점처럼 판정과 무관한 그림에만 쓴다.

## 4. 포크 끝 잔여 거리

인식이 끊기는 순간 포크 끝에서 팔레트 앞면까지 남는 거리는 장착 높이 0.27 / 0.50 / 0.90 m 에서 각각 **850 / 1050 / 1550 mm** 다. 계산은 `(0.75 + 최근접 검출 거리) − 0.95`, 여기서 0.95 m 는 `sim/models/dls08_provisional/forklift.xml` 의 `left_fork_tip` site x 이고 0.75 m 는 측정에 쓴 카메라 x 다. 재현: `python tools/deck/camera_mount.py`.

⚠️ 이전 판의 580 / 780 / 1280 mm 는 카메라 x(0.75)와 마스트 전면 x(0.48)를 섞어 **270 mm 씩 작았다.**
