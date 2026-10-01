# 실측 차체 모델 `dls08_measured` — 실행 기록

계획: [2026-09-30 계획](../plans/2026-09-30-measured-chassis-model.md). 이 기록은 그 완료 기준 1–7 의 결과다.

## 범위

입력은 팀 실측(2026-09-29–30)과 그 위의 계산·추정이다. 필드별 출처는
[모델 README](../../sim/models/dls08_measured/README.md). Isaac 결과는 전부 합성이고 제어는 시뮬레이터
정답 자세다. **이 기록은 실측 모델이 실행 경로까지 전달되는지를 확인한 것이지, 실측 모델에서 운반·A–D 가
되는지를 확인한 것이 아니다.**

## 로컬 (완료 기준 1–6)

| 기준 | 결과 |
|---|---|
| 1 생성 | `dls08_measured` 생성·envelope 통과. `dls08_provisional` 네 산출물 재생성 시 바이트 동일(`test_committed_model_files_are_the_generator_output`) |
| 2 구동 기하 | 실측 URDF (0.66, 0.53, 0.125, 15°), 잠정 (0.64, 0.51, 0.135, 0.45). 중간 링크 합성(0.76·1.39)과 회전 체인·축·비대칭·타이어 방향·움직이는 부모 거부 |
| 3 삽입 한계 | 잠정 0.406 · 실측 0.346, 목표 두 형상 0.300, NaN·0.046 이하 거부 |
| 4 장면 검사 | 0.9 mm 통과·1.1 mm 거부, 반지름만·가로대만·축 방향·축 부호·스케일·반사·NaN 이 다른 importer 형식 USD 거부, body0 중간 링크 합성 |
| 5 하드코딩 | 운반·SLAM 에 `AckermannGeometry(0.64` 없음, `--forklift-urdf` 필수 |
| 6 시험 | `pytest tests --ignore=tests/simulation` 1,805 통과·2 실패(`test_detector_v1_replay` 2건, 변경 전과 같음), `tests/simulation/test_forklift_model.py` 통과, 변경 파일 Ruff 통과 |

ws1 기존 base scene 을 로컬로 복사해 `stage_chassis` 를 실제 Isaac 5.1 importer 출력에 돌렸다 — 잠정 URDF 와
일치, 실측 URDF 와 24.5 mm 불일치, 조향 축 X·루트 스케일 2 는 거부.

## ws1 스모크 (완료 기준 7) — Slurm 작업 595

snapshot `snapshots/measured_chassis_v1`, 출력 `artifacts/20261001_measured_chassis_smoke/`, 스크립트
`run.sbatch`.

| 단계 | 결과 |
|---|---|
| [1] `build_base_scene.py` | 성공. 장면에서 다시 읽은 값: 네 관절 body0 = `/World/Forklift/base_link`, 조향 (0.32, ±0.265, 0.125) 축 +Z 한계 ±0.2618 rad, 뒤 spin (−0.34, ±0.265, 0.125) 축 +Y, 타이어 0.125 ×4, 캐리지·포크 상자 14 개, 캐리지 한계 0.346 m (`base_scene/base_scene_manifest.json`) |
| [2] 운반 seed 0, 정답 좌표 | 장면 검사 통과로 시작. 계획 성공, 접근 20.5 s → 삽입 28.2 s, 삽입 종점 오차 7.7 mm·0.0045 rad. **들기 중 중단** — 승강 0.089 m 에서 팔레트 기울기 0.136 rad, 중단 기준 0.15 에 근접해 `Excessive body tilt in lift` |
| [3] SLAM 조사 seed 0 | 장면 검사 통과, 완주. 조사 경로 118.98 m(잠정 모델 115.6 m), 도착 오차 7.0 mm·−0.0064 rad, 스캔 2,467, 유효 빔 평균 61.4 %(최소 41.6 %), 자기 적중 0 |
| [4] 기존 잠정 장면 + 실측 URDF | 시작 전 거부: `Scene joint left_steer is 24.5 mm from …/dls08_measured/forklift.urdf` |

**[2] 의 들기 중단.** 실측 모델의 삽입 목표 0.300 m 는 EPAL 6 깊이 절반과 같아, 포크 끝이 팔레트 중심에
걸린 채로 든다(계획 D4 표의 "목표 − 깊이/2 = 0.000"). 한 번의 실행이라 원인을 확정하지 않았지만, 계획이
미뤄 둔 "46 mm 정책과 지지의 양립" 문제가 시뮬레이터에서 실제로 드러난 것으로 읽는다. 캐리지 실측과
삽입 규칙 재검토(다음 작업 3)의 입력이다. 잠정 모델(삽입 0.360 m)은 같은 seed 에서 완주했다
(2026-09-23 기록).

**[3] 의 경로 길이 차이.** 조사 경유점은 같고 계획기 곡률이 0.50 → 0.29 m⁻¹ 로 줄어 경로가 3.4 m 길어졌다.

USD 로드 시 `…/visuals` 참조 미해결 경고가 나오지만 fork tip·steering carrier 의 빈 visual 이고 잠정 장면에서도
같다(로컬 복사본에서 확인).

## 이 기록이 말하지 않는 것

- 실측 모델에서의 운반·A–D·SLAM 성적. 각 한 번의 시작 확인이다.
- 캐리지 위치·뒤차축–포크 끝·포크 길이 기준점 등 미측정 치수의 타당성.
- 최소 회전 반경 2.728 m 의 실측 확인(15° 의 정의 미확인).
