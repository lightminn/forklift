# 포켓 관측 계약

M2 인식기의 출력, M3 추적기의 입력, 평가기의 정답은 동일한 `PocketObservation` 기하 타입을 사용한다. 구현은 [`pocket_observation.py`](../../src/forklift_core/perception/pocket_observation.py), 설계 근거는 [승인 설계 §4](../design/2026-09-11-pocket-observation-and-scene-set.md)다. 관측 타입을 구현한 것이 팔레트 인식기나 실물 센서 검증의 완료를 뜻하지는 않는다.

## 가정과 좌표계

팔레트는 바닥에 수평으로 놓이고, 두 개구부는 수직 직사각형이며 공통 삽입축을 갖는다. `Pocket.center_m`는 **팔레트 전면 기준면 위 개구부 중심**이다. 폭과 높이는 개구부 자체의 치수이며 삽입 깊이별 통로 치수가 아니다. 기울어진 팔레트는 이 계약의 지원 범위 밖이다.

공개 관측의 `frame_id`는 `base_link`로 고정한다. x는 전방, y는 좌측, z는 위쪽이다. optical 좌표는 x가 우측, y가 아래, z가 전방이다. optical 점은 먼저 `RigidTransform.apply`로 base에 옮기고, 방향 벡터에는 회전만 적용한 뒤 관측을 만든다. 위치 변환의 이동 성분을 방향 벡터에 더하지 않는다. 별도 `transform_observation` 헬퍼는 제공하지 않는다.

삽입 방향의 정본은 `insertion_yaw_rad` 하나다. ψ에 대해 삽입축 a = (cos ψ, sin ψ, 0), 팔레트 왼쪽축 ℓ = (−sin ψ, cos ψ, 0), 전면 바깥 법선은 −a다. 포켓별 축·법선은 저장하지 않는다.

`left`와 `right`는 ℓ·(center_left − center_right) > 0으로 구분한다. 단순한 base y값 비교가 아니다. 예를 들어 ψ = π이면 `left`의 base y값이 `right`보다 작다.

## 필드

두 타입은 `@dataclass(frozen=True)`다. `Pocket`의 필드는 다음과 같다.

| 필드 | 단위·표현 | 의미·허용값 |
|---|---|---|
| `center_m` | m, float 3개 튜플 | 관측 frame의 전면 개구 중심 (x, y, z), 모두 유한 |
| `width_m` | m | 개구부 수평 폭, 유한 양수 |
| `height_m` | m | 개구부 수직 높이, 유한 양수 |

`PocketObservation`의 모든 필드는 생성자와 JSON에서 필수다. 값이 없는 필드도 `None` 또는 JSON `null`로 명시한다.

| 필드 | 단위·표현 | 의미·허용값 |
|---|---|---|
| `stamp_ns` | ns, 정수 ≥ 0 | 촬영·측정 시각. 수신 시각이 아니며 bool 제외 |
| `clock_domain` | 문자열 | `ros_sim`, `ros_system`, `device`, `synthetic` (`CLOCK_DOMAINS`) |
| `frame_id` | 문자열 | `base_link`만 허용 (`OBSERVATION_FRAME`) |
| `source_provenance` | 문자열 | `synthetic`, `replay`, `live`, `synthetic_ground_truth` (`PROVENANCES`) |
| `status` | 문자열 | `valid`, `no_pallet`, `invalid` (`STATUSES`) |
| `left` | `Pocket` 또는 `None` | 접근 방향 기준 왼쪽 포켓. `valid`일 때 필수 |
| `right` | `Pocket` 또는 `None` | 접근 방향 기준 오른쪽 포켓. `valid`일 때 필수 |
| `insertion_yaw_rad` | rad 또는 `None` | 공통 삽입축 yaw, 유한하고 (−π, π]. `valid`일 때 필수 |
| `position_sigma_m` | m 또는 `None` | 두 중심에 공통으로 적용하는 등방 위치 1σ 상한, 유한 ≥ 0 |
| `yaw_sigma_rad` | rad 또는 `None` | yaw의 1σ 불확실성, 유한 ≥ 0 |
| `reason` | 문자열 또는 `None` | 비valid 상태의 사유. 공백만 있는 문자열은 거부 |

`None`인 σ는 **미상**이다. 0으로 대체해서는 안 된다. 합성 정답은 두 σ를 0.0으로 기록하지만, 이는 모델 기하가 정확하다는 의미이며 렌더링·깊이 양자화 오차가 없다는 뜻은 아니다.

## 검증 규칙

잘못된 입력은 `ValueError`로 거부한다. 정상적인 팔레트 부재나 관측 소실은 예외 대신 `no_pallet` 또는 `invalid` 상태와 사유로 표현한다.

1. 시각은 `numbers.Integral` 중 bool·`np.bool_`을 제외한 0 이상 정수다. clock domain·상태·출처는 위 허용 집합에 속하고, frame은 `base_link`여야 한다.
2. 포켓 중심은 유한 실수 3개이며, 폭·높이는 유한 양수다. 중심은 Python float 튜플로 복사하고 폭·높이·yaw·σ는 Python float, 시각은 Python int로 정규화한다. NumPy 스칼라로 생성해도 JSON 직렬화가 가능하다.
3. `valid`는 `Pocket` 타입인 left·right와 유한한 yaw ∈ (−π, π]를 요구하며 `reason`은 `None`이어야 한다. 두 σ는 각각 `None` 또는 유한한 0 이상 값이어야 한다.
4. `no_pallet`·`invalid`에서는 left·right·yaw·두 σ가 모두 `None`이어야 하고, `reason`은 비어 있지 않은 문자열이어야 한다.
5. 두 중심의 유클리드 거리는 (0.05, 2.0) m 안이어야 한다. 좌우 순서와 개구부 비중첩은 ℓ·(center_left − center_right) > (left.width_m + right.width_m)/2로 검사한다. 등호, 즉 개구 가장자리가 맞닿는 경우도 거부한다. 기본 예시는 중심 간격 0.35 m > 반폭 합 0.25 m로 통과한다.
6. `to_json()`은 필드명 그대로 dict를 반환하며 중심 튜플을 list로, 포켓을 중첩 dict로 표현한다. `pocket_observation_from_json(obj)`는 최상위와 중첩 포켓의 알 수 없는 키·누락 키를 거부한다. 수치 검증으로 NaN·Inf를 거부하므로 `json.dumps(obj, allow_nan=False)`로 직렬화할 수 있다.
7. `yaw_difference_rad(estimate_rad, reference_rad)`는 유한한 두 각도의 차이를 (−π, π]로 감싼다. (estimate − reference + π) mod 2π − π를 구한 뒤 정확히 −π이면 +π로 바꾼다. 여러 바퀴 차이도 허용한다.

## 평가 규약

- 포켓별 위치 오차는 추정 중심과 정답 중심의 유클리드 거리 ‖ĉ − c‖이며, 좌·우를 각각 계산한다.
- 장면 위치 오차는 두 포켓 위치 오차의 **최댓값**이다.
- yaw 오차는 `yaw_difference_rad(추정, 정답)`을 사용한다. ±π 경계에서 단순 뺄셈으로 큰 오차를 만들지 않는다.
- 검출률 분모는 `positive`와 `occluded` 장면을 분리한다. 음성 장면은 오검출률로 집계한다.
- 인식기의 `invalid` 출력은 실패 표본으로 별도 집계한다. 유효한 추정만 남겨 분모를 줄이지 않는다.

## JSON 예시

아래는 합성 관측 예시다. 실물 보정이나 측정값이 아니다.

```json
{
  "stamp_ns": 5000000000,
  "clock_domain": "ros_sim",
  "frame_id": "base_link",
  "source_provenance": "synthetic",
  "status": "valid",
  "left": {"center_m": [1.7, 0.175, 0.15], "width_m": 0.25, "height_m": 0.2},
  "right": {"center_m": [1.7, -0.175, 0.15], "width_m": 0.25, "height_m": 0.2},
  "insertion_yaw_rad": 0.0,
  "position_sigma_m": 0.01,
  "yaw_sigma_rad": 0.02,
  "reason": null
}
```

팔레트가 없는 장면의 정답은 다음처럼 표현한다.

```json
{
  "stamp_ns": 5000000000,
  "clock_domain": "ros_sim",
  "frame_id": "base_link",
  "source_provenance": "synthetic_ground_truth",
  "status": "no_pallet",
  "left": null,
  "right": null,
  "insertion_yaw_rad": null,
  "position_sigma_m": null,
  "yaw_sigma_rad": null,
  "reason": "no target pallet in scene"
}
```

## 보장 범위

이 계약은 M2 평가와 M3 입력을 위한 것이며 **M5 삽입 여유 계산에는 불충분**하다. 삽입 깊이별 통로 치수, 실제 포크 형상, 하중에 따른 처짐, 내·외부 보정 오차를 추가로 확인해야 한다. `valid`는 관측 계약의 유효성을 뜻하며 안전한 삽입이나 작업 성공 판정이 아니다. 파일 입력과 평가 표본의 소유 규칙은 [장면 데이터 세트 계약](scene-dataset.md)을 따른다.
