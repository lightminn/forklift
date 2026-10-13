# 하역 스캔 정합 순환 회귀 입력

`seed6_cycle.npz` 는 Isaac 실행이 남긴 하역 직선 앞 정합 입력(`delivery_match_1.npz`)을 그대로
복사한 것이다. 2026-10-13 ws1 잡 2503, 스냅숏 `icp_trace2_9a283ba`(a03a678 의 기록 코드와 같다),
`artifacts/20261012_recov/icp_trace/seed_6_v0.055_corr_yaw_-0.015/run/`, 실행 인자는 S4 개발 판정의
seed 6 보정 방향 −0.015 와 같다. SHA-256 `e02496ce8150e020604d9580b95e28ca1caca277780a089a97baf84f76064466`.

배열: `reference`(기준 스캔 레이저 점, 하역 자세에서 정답 장면으로 쏜 이상 스캔), `live`(그 순간의
잡음 섞인 실시간 스캔 점), `rear_from_laser`, `estimate`(스캔 순간의 SLAM 추정 뒷축 자세), `seed`
(초기값으로 쓴 계획 하역 자세), `reference_rear`(기준 스캔을 찍은 뒷축 자세), `truth_rear`(정합 순간의
정답 뒷축 자세 — 평가 전용, 정합에 넣지 않는다).

이 입력에서 지금까지의 정합은 일곱 시작점이 모두 주기 2 순환(대응 2 개 교대, 두 상태 0.109 mm·15.5 µrad)에
들어 `not_converged` 로 거부됐다. 한 입력의 회귀 시험이며 순환이 얼마나 자주 나는지의 근거가 아니다.
설계는 `docs/plans/2026-10-04-lidar-obstacle-map.md` 의 "하역 스캔 정합의 순환 정착" 항목.
