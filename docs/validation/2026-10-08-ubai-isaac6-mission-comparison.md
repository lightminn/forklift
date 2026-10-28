# UBAI Isaac 6.0.1 임무 실행 — ws1 Isaac 5.1 기준과 비교

2026-10-08 실측. UBAI gpu6 노드 n021(NVIDIA A10, 드라이버 595.45.04), Isaac Sim **6.0.1** enroot 컨테이너, Slurm 잡 1003184·1003212.
비교 기준은 ws1(Isaac 5.1) `artifacts/20261008_gate_measured` 의 seed 1·3·5 다([관문 결과](../plans/2026-10-04-lidar-obstacle-map.md)).
모든 수치는 합성 데이터다.

> **결론.** 같은 스냅샷 코드로 UBAI 에서 임무를 돌리면 **판정은 ws1 과 같고 수치는 다르다.** 비트 단위 재현이나 벽시계 예산 판정이 필요한 실행은
> ws1 에 두고, UBAI 로 옮긴다면 기준선부터 거기서 다시 잰다.

## 왜 5.1 이 아니라 6.0.1 인가

UBAI 노드는 glibc 2.28 이라 pip 설치가 안 되고 enroot 컨테이너만 쓸 수 있다. 그 조건에서 **Isaac 5.1.0(NGC `5.1.0-rc.19`)은 headless `SimulationApp` 이
생성자를 반환하기 전에 `librtx.scenedb.plugin.so` 에서 SIGSEGV** 로 죽는다(잡 1002925·1002936 재현, 주입된 NVIDIA 라이브러리는 전부 595.45.04 로 일치).
업스트림 이슈 isaac-sim/IsaacSim#677 에서 같은 충돌이 6.0.0 으로 해결됐고, 6.0.1 은 같은 노드·드라이버에서 물리·replicator 깊이 캡처와
`sim/isaac/determinism_probe.py` 가 수정 없이 돈다(6.1.0 도 같은 깊이 해시).

## 6.x 에서 돌리기 위해 한 것

- **코드 수정 없이는 안 돈다.** Isaac 6.0 이 URDF 임포터를 3.x 로 바꾸며 `URDFCreateImportConfig`·`URDFParseAndImportFile` Kit 명령을 없앴다(`scene.py`
  `create_pallet` 외 3곳이 사용). 시험 래퍼가 두 명령을 대신 등록해 ws1(5.1)이 만든 `pallet.usd` + `configuration/` 을 복사했다 — 런타임 팔레트 URDF 가
  ws1 것과 구조가 같고 수치가 상대 1e-12 안일 때만(실제 최대 차 3.3e-16, numpy 2.3.1 vs 1.26.0 반올림). 스냅샷 코드는 그대로다.
- 준비물: ws1 스냅샷 `p5_76555c8`, base scene(S3 URL 한 줄만 로컬 경로로), Simple_Warehouse 5.1 에셋 미러(2,047 파일 553 MB — S3 공개 목록으로 받고 USD
  의존성 추적으로 `Isaac/Props/Forklift` 6개 보충). 계산 노드는 Isaac 기본 에셋 서버에 닿지 못한다.
- 컨테이너에 ws1 과 같은 PyYAML 6.0.2·Pillow 11.3.0·opencv-headless 4.11.0.86 을 따로 설치했다(numpy 는 Isaac 번들 2.3.1). 컨테이너 안에 ws1 과 같은
  경로(`/home/projects/forklift/...`)로 마운트해 실행 인자를 ws1 `estop.sbatch` 와 같게 했다. 셰이더 캐시(1.5 GB)를 유지하지 않으면 첫 장면 준비에 약 6 분이 더 든다.
- 시험 묶음(sbatch·rc.local·shim·실행 결과·ws1 기준 사본)은 UBAI 홈의 `~/fkcmp/` 에 있다.

## 결과 (seed 1·3·5)

| 항목 | ws1 (5.1) | UBAI (6.0.1) |
|---|---|---|
| 임무 판정 | 성공·완료 | 같음 |
| 시나리오·재계획 | 재계획 0 | 같음 |
| 발동한 비상정지 | 14·13·13 회, 전부 정지 | 같은 횟수 |
| 여유 46 mm 넘어짐(lift 단계 "Excessive body tilt") | 최대 0.1514–0.1517 rad | **재현**, 0.1507–0.1516 rad |
| 임무 시간(예) | 237.7 s | 252.9 s |
| 배송 오차(예) | 7.3 mm | 10.3 mm |

- 수치는 다르다: 첫 깊이 해시, 인식 목표(0.3–1.3 mm 차), 접근 경로, 임무 시간, 배송 오차.
- `collision_counts` 55/22 는 충돌 사건이 아니라 충돌 형상 개수라 비교 지표가 아니다.
- **UBAI 자체 재실행**(seed 1 두 번)에서 samples·paths·계획 기록·비상정지 기록이 같았다(시간 지표 제외).

### 측정 아티팩트 — 작은 팔레트 기울기가 0 으로 나온다

UBAI 결과에서 작은 팔레트 기울기가 거의 0 으로 나온다. 실제 기울기가 아니라 측정 쪽 문제다 — `run_transport.py` `yaw_and_tilt` 가 float32 쿼터니언으로
`acos(1-2(x²+y²))` 를 계산해, numpy 2 에서는 약 2.4e-4 rad 미만이 0 으로 반올림된다(ws1 의 기록 쿼터니언도 이 경로로 계산하면 0). 큰 기울기는 영향이 없다.
**6.x/numpy 2 로 옮기면 이 계산을 float64 로 바꿔야 한다.**

## 속도

실행당 UBAI 가 ws1 보다 **1.9–2.2 배 느리다**(wall 161.6→357.2, 156.0→303.6, 139.9→267.2 s, 계획 계산도 약 2 배 — Xeon 6240R vs Ryzen 9 7900X).
들기 단계에서 끝나는 짧은 실행은 1.5–1.7 배. 잡마다 컨테이너 이미지 import 가 약 4 분 더 든다.

## 그 밖의 주의

- 6.1.0 은 오프라인 노드에서 기동이 약 10 분 걸린다(기본 활성 확장 `isaacsim.robot.policy.examples` 가 S3 에셋을 기다린다).
- enroot 에서 `--mount <파일>:/etc/rc.local` 로 명령을 바꿀 때 끝에 `exec` 나 `exit` 가 없으면 기본 진입점(스트리밍 앱)이 이어서 돌아 segfault(139)를 낸다 — Isaac 결함이 아니다.
- 크기를 지정하지 않은 `GroundPlane` + `DynamicCuboid` 조합 스크립트 하나가 6.0.1 에서 USD 재귀 크래시를 냈다. 같은 객체에 `size=20.0`·`physics_dt` 를 주면 정상(원인 미규명).
