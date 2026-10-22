# 세션 장면·공통 팔레트 장애물·계획 포락선 대조 — 검증 (2026-10-22)

계획: `docs/plans/2026-10-22-session-scene-pallet-props.md`(3판). 상위 계획: `docs/plans/2026-10-22-continuous-transport-ct1.md` PR ②. 브랜치 `feat/ct1-session-scene`.

## 계획 포락선 대조 (UBAI, CPU)

- 실행: UBAI 잡 1033975(gpu6 노드 CPU 4). 커밋 `ec17008` 의 `tools/riibotics_isaac_scenes.py --session line --session random` → `tools/session_plan_check.py`.
  - 자세 사이는 원호 위에서 잰다.
  - 그 전의 현 보간 실행(잡 1033953, `4c00834`)과 표의 값이 같다.
- 설정은 장면 계획 검사와 같다.
  - D1 0.25 m, 하역된 팔레트 0.21 m.
  - URDF `dls08_measured`, 여유 0.016 m, 정렬 직선 2.1·하역 1.5·후퇴 0.55 m.
  - κ 0.29, clearance 0.10 m, 확장 30000, D7 계획기 옵션. 표본 이동 5 mm(재분할 0.5 mm).
- 범위: 파일 순서, 매번 출발점에서 출발해 돌아오는 완전 복귀, 이상적 하역(앞 팔레트는 칸 자세), 이상적 추정(관측 뒤 추정 = truth).

| 세션 | 미션 | 결과 | 다른 팔레트 truth 와의 최소 간격(하한) | 그 단계·팔레트 | 계획 시간 |
|---|---|---|---|---|---|
| line | P-1 → D-1 | 통과 | 0.343 m (0.340) | 추출, P-2 | 11.3 s |
| | P-2 → D-2 | 통과 | 0.462 m (0.460) | 운반, P-1(하역) | 13.0 s |
| | P-3 → D-3 | 통과 | 0.462 m (0.460) | 운반, P-2(하역) | 12.7 s |
| random | P-4 → D-1 | 통과 | 0.816 m (0.813) | 운반, P-5 | 21.7 s |
| | P-5 → D-2 | 통과 | 0.487 m (0.485) | 운반, P-4(하역) | 21.4 s |
| | P-3 → D-3 | 통과 | 0.487 m (0.485) | 운반, P-5(하역) | 22.8 s |

- 여섯 미션 모두 첫 관측 후보(index 0, 가까운 사다리)로 계획됐다.
- 미션 팔레트 자신과의 최소 간격은 접근 끝 0.100 m(계획된 정지 거리)다.
- 결과 파일: UBAI `~/forklift-ct1c/out/{line,random}_check.json`.
- 이 수치는 계획 포락선(차체 + 포크 사각형)과 이상적 추정·하역의 truth 사이 간격이다. SLAM 오차(0.14–0.16 m)와 실제 하역 오차(Isaac 0.02–0.10 m)는 들어 있지 않다. Isaac 에서 이웃과의 실제 최소 거리는 PR ④ 에서 기록한다.

## 시험

- `tests/unit/tools/test_session_scene.py`:
  - 세션 형식, 공칭의 기울기, truth = 공칭 + 오차.
  - 세션 미션 k 와 v1 장면 k 의 관계.
  - 로더 거부(형상 섞임·id 중복·순서), v1 변환.
  - `zone_rect` = 러너 식.
  - 등록이 v1 에서 러너의 targets 와 같다(배치·live).
- `tests/unit/tools/test_session_plan_check.py`:
  - 정확한 거리, 겹침, 표본 사이 접촉, 1 mm 평행 간격(재분할로 통과).
  - 호 처짐(`clearance_unproven`).
  - 단계표(접근은 자기 팔레트 포함, 삽입은 제외 — 접근 끝 1 cm 침범을 잡는다), `prior_failed`, 하역 칸 반영.
- `tests/unit/safety/test_pallet_permit.py`: `session_pallet_props` 의 기저별 크기·제외, `moved_ids` 로만 옮김, 팔레트별 teleop 구역.
