# 다섯 번째 동결 평가 — 미사용 seed 5000–5029 (실행 기록)

상위: [운반 단계 수정 개발 기록](2026-10-03-transport-stage-dev-run.md)(125/129, 예외 기록 후 채택), [계획](../plans/2026-10-03-transport-stage-fixes.md).
앞선 동결 평가: [G5](2026-10-02-g5-frozen-evaluation.md) 22/30, [두 번째](2026-10-02-second-frozen-evaluation.md) 26/30,
[세 번째](2026-10-03-third-frozen-evaluation.md) 25/30, [네 번째](2026-10-03-fourth-frozen-evaluation.md) 27/30(모두 불합격).

## 공개 (결과를 보기 전)

- 3순위(인식 ↔ 운반 연결)의 **다섯 번째 동결 평가**다. 사용자 결정(2026-10-03, "ㅇㅇ"). 앞의 네 평가 seed 는 이후 개발에 썼고 그 기록은 바꾸지 않는다.
- 개발 집합 125/129 는 성능 근거가 아니다. 129 seed 모두 진단·수정에 썼고, 앞의 네 번 모두 개발 집합 수치가 미사용 seed 에서 재현되지 않았다.
- 알려진 위험: 시간 예산 150 s(개발 실행의 실패 4 개 중 3 개가 예산 초과), 실행 간 변동으로 성패가 갈리는 seed(3012), 좁은 통로(3008 형 `transport:no_path`),
  검출 기각의 반복.

## 동결

| 항목 | 값 |
|---|---|
| 코드 | main `5dbeef8`(소스는 PR #14 머지 `c295a1a` 와 같음), ws1 snapshot `snapshots/eval5_5dbeef8`, 파일 목록 해시 `04fb9b6e…` |
| 해시 | `run_transport.py` `53f6ef00…`, `pallet_mission.py` `146206ef…`, `path_tracking.py` `0de86f81…`, `observation_viewpoints.py` `c323dae8…`, `pocket_detector.py` `7bd2a28c…`, `config/isaac_transport.yaml` `51f05655…` |
| 설정 | 개발 기록과 같다: 평면 예산 8, 고정 관측 후보 8 + 실행 중 후보 ≤ 6, 기존 사다리 뒤 확장 사다리(세밀 격자 ×4 예산, 0.10 m 동작), 첫 관측의 두 번 탐색, 운반 cusp 재계획(≤ 2 회) |
| 인자 | `--obstacles 4 --max-sim-seconds 150 --use-perception --planning-target perception`, 잠정 차체 `dls08_provisional`, EPAL 6, bay |
| 장면·렌더 | `base_scene_warehouse_forklift/`, RaytracedLighting, ws1 RTX 5070 Ti, Isaac Sim 5.1 |

개발 실행 snapshot(`transport_1cf9ac2`)과의 소스 차이는 `pallet_mission.py` 의 한 분기(호출자가 처음부터 세밀 격자를 줄 때 확장 사다리 유지)뿐이며, 실행기 설정
(0.2 m/10°)에서는 탐색 순서가 같다.

## 규칙

- **seed 5000–5029, 한 번만**, 순차, 한 Slurm 작업. 출력 `artifacts/20261003_fifth_eval/`.
- 미사용 확인(2026-10-03):
  - 로컬 `artifacts/` 와 ws1 `artifacts/`·`snapshots/` 에 `seed_50[0-2][0-9]` 디렉터리가 없다.
  - 로컬 `artifacts docs tools sim config deploy`, ws1 `artifacts` 의 JSON·로그·sbatch·셸·문서에 `--seed(s) 50xx`·`"seed": 50xx`·`seq 5000` 이 없다.
  - 문서·도구에 나오는 5000–5029 는 이 평가를 위한 예약뿐이다(`tools/observation_viewpoint_eval.py` 가 거부). `random.Random(20260927).sample(range(20, 10000), 40)`
    표본에도 이 범위의 값이 없다(가까운 값 5115).
  - 9/1 이후 ws1 Slurm 작업 이름에 이 범위를 쓴 작업이 없다.
- 재실행: `result.json` 없음·Slurm OOM/NODE_FAIL/CANCELLED·`phase == "startup"` 이면 같은 snapshot 으로 1 회(첫 시도 폴더는 `seed_*_crash_1`). 그 외 실패는 결과.
- 판정: **분모 30**, 계획·캡처·추종·시간 한도 실패와 계열 A 도 실패. **28/30 이상(Clopper–Pearson 단측 95 % 하한 80.47 %)이 합격** — 앞의 네 평가와 같은 기준.
- 정답 진단(G3 도구)은 30 개 결과를 이 문서에 확정한 뒤에만 연다. 미달이면 3순위는 미완료로 남고, 이 결과로 구성을 고쳐 같은 seed 로 다시 평가하지 않는다.

## 결과 (정답 진단 전에 확정, 2026-10-03)

ws1 Slurm 작업 725, 21 분 27 초, 30 회 모두 `result.json` 있음, `phase == "startup"` 0, 재실행 0, 렌더 모드 30 회 모두 RaytracedLighting,
`run_transport.py` 해시 30 회 모두 `53f6ef00…`(snapshot 과 같음). 원본 `artifacts/20261003_fifth_eval/`(로컬·ws1).

**완주 30 / 30 — 합격(합격선 28/30).** Clopper–Pearson 단측 95 % 하한 90.50 %(합격 기준 80 %).

- 시뮬 시간 63.4–119.0 s(중앙값 82.9 s, 예산 150 s), 계획 시간 최대 76.9 s(5021). 삽입 위치 오차 7.63–7.74 mm(계획 종점 대비 추종 오차 — 실제 포켓 기준
  정밀도가 아니다).
- 21 seed 는 첫 관측에서 유효 검출로 끝났다. 실행 중 관측점으로 끝난 seed 2 개: 5015(고정 후보 1 `no_pallet` → 후보 8 유효), 5021(고정 후보 1 `invalid`·
  4 `no_pallet` → 후보 8 유효). cusp 재계획이 일어난 seed 는 0, 첫 관측의 확장 사다리 패스를 쓴 seed 도 0 이다.

## 판정

**다섯 번째 동결 평가 합격.** 네 평가는 22/30 → 26/30 → 25/30 → 27/30 → **30/30** 이다. 사전 등록한 3순위 판정 기준(미사용 seed 30 개에서 28 이상)을 이 구성이
처음으로 넘었다.

## 정답 진단 (결과 확정 뒤, G3 도구)

snapshot 과 같은 커밋 `5dbeef8` 의 git worktree 에서 실행했다. 결과 `artifacts/20261003_fifth_eval/diagnosis/`. 관측 시도 42 개, 게이트 42/42 통과, 정답 사슬
일치 최고 1.0.

**시도 단위:** OK 30, D(하부 증거) 7, B 3(예산 소진 2·수직 가설 없음 1), C(개구) 2, A 0. 기각 12 시도는 모두 다음 관측점에서 회복했고, 30 seed 모두 마지막
관측이 OK 다.

## 이 합격이 말하는 것과 말하지 않는 것

- **말하는 것:** 이 구성(코드 `5dbeef8`, 잠정 차체, 합성 bay, EPAL 6, 장애물 4 개, 150 s 예산)에서 미사용 seed 30 개의 인식 → 접근 → 삽입 → 운반 → 하역 완주율의
  단측 95 % 하한이 80 % 를 넘는다(90.5 %).
- **말하지 않는 것:**
  - 로봇 위치·장애물 지도·목적지는 시뮬레이터 정답이다(`feedback = simulator_ground_truth`). LiDAR·SLAM 은 연결되지 않았다(5·6순위).
  - 삽입은 접근 시작 때의 한 번 추정으로 끝까지 간다. 근접 포켓 추적은 없다(4순위).
  - 위양성(렌더 음성 모집단), 실측 차체, T11, 실물 카메라는 평가하지 않았다.
  - 개발 집합에서 확인된 위험(시간 예산 근처의 seed, 실행 간 변동으로 갈리는 seed, 좁은 통로)은 이 30 seed 에서 나타나지 않았을 뿐 없어진 것이 아니다.
