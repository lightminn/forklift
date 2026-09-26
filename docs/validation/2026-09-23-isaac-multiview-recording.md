# Isaac 운반 임무 다중 시점 녹화

확인일: 2026-09-23 · 실행: `20260923T0400Z_views_seed2` (ws1)

## 무엇을 했는가

4주차 발표 시각자료를 위해 `sim/isaac/run_transport.py` 에 선택 옵션 `--extra-views` 를 추가하고,
2026-09-22 의 seed 2 인식 임무(`20260922T0600Z_video_seed2`)와 같은 인자에 `--extra-views quarter,chase,perception`
만 더해 다시 실행했다. 옵션을 주지 않으면 기존 동작·출력은 그대로다.

| 시점 | 파일 | 내용 |
|---|---|---|
| 조감 | `transport.mp4` | 기존과 같다 |
| 쿼터뷰 | `view_quarter.mp4` | 작업장 모서리 위 고정 카메라. 계획 경로선(`/World/PlannedPaths`)이 보인다 |
| 추적 | `view_chase.mp4` | 차체 뒤 위에서 따라가는 카메라. 운전석 지붕이 포크를 가려 발표에는 쓰지 않았다 |
| 인식 카메라 | `view_perception.mp4` | 차체에 달린 인식 카메라의 RGB·깊이를 나란히 |

영상 프레임마다 로봇·팔레트 자세를 `frames.jsonl` 에 남기고, 인식 카메라 장착·내부 행렬을
`result.json` 의 `extra_views` 에 남긴다. `tools/deck/isaac_mission_views.py` 가 이것으로 검출 포켓을
인식 카메라 영상에 투영한다.

## 결과

| 항목 | 값 |
|---|---|
| 성공 | `success: true` |
| 임무 시간 | **75.3초** (`simulated_time_s` 75.275) — 2026-09-22 실행은 75.4초 |
| 삽입 종점 오차 | 7.67 mm / 2.77 mrad — 2026-09-22 실행은 7.65 mm / 2.83 mrad |
| 녹화 프레임 | 4,510 |

같은 seed·인자에서 카메라만 늘렸는데 수치가 조금 달라졌다. 물리 계산은 렌더와 독립이어야 하지만
이번 실행은 비트 단위 재현을 확인한 실험이 아니므로, 차이의 원인은 가리지 않았다. 발표에는 이번
실행의 값을 쓴다.

## 실행 조건

- 인터프리터 `/opt/isaacsim-env/bin/python3.11` (Isaac Sim 5.1.0.0, pip 설치), 저장소 사본 `/home/projects/forklift/repo`.
- 실행 전 `perception_adapter.py`·`scene.py`·`insertion_geometry.py`·`pallet_mission.py`·`scene_rig.py`·
  `config/isaac_transport.yaml`·`config/pallet_prior_epal6.yaml` 의 SHA-256 이 로컬과 같음을 확인했고,
  `run_transport.py` 는 로컬 `HEAD` 판과 같은 상태에서 이번 변경본으로 교체했다(이전 판은 `/home/projects/forklift/run_transport.py.bak-20260923`).
- Slurm 을 거치지 않고 `nohup` 으로 직접 실행했다(이전 실행과 같은 방식).
- 결과는 로컬 `videos-from-ws1/20260923T0400Z_views_seed2/`(git 추적 제외)에 USD 를 빼고 회수했다.

## 이 기록이 말하지 않는 것

한 사례의 녹화다. 여러 seed 성공률, 렌더 추가가 결과를 바꾸지 않는다는 재현성, 실물 카메라와의 일치는 확인하지 않았다.
인식 카메라는 시뮬레이션 설정(차체 기준 높이 0.5 m, 640 × 480)이며 실물 D435i 의 화각과 같지 않다.

## 두 번째 실행 — 표시용 인식 카메라와 넓은 쿼터뷰

실행 `20260923T0515Z_views2_seed2`, 같은 seed 2 인자에 `--extra-views quarter,perception --quarter-eye 0.3,-4.6,4.0
--quarter-target 0.3,0.5,0 --quarter-focal 1.5`.

- **인식 카메라 영상의 팔레트 절단은 근접 클리핑 때문이었다.** Isaac 카메라의 기본 near clip 은 1 m 이고
  `run_transport.py` 는 이를 바꾸지 않는다([G1 기록](2026-09-21-g1-calibration-run.md) §2). 첫 실행의
  `view_perception.mp4` 에서는 1 m 안으로 들어온 팔레트가 잘려 속이 보였다. 검출기가 쓰는 `PerceptionCamera`
  는 그대로 두고, 같은 위치·화각의 `PerceptionDisplayCamera` 를 따로 두어 near clip 을 0.05 m 로 줄여 녹화한다
  (읽어 들인 값 `[0.05, 1e6]`). 표시용 깊이는 D435i 최소 거리 0.28 m 안쪽을 검게 칠한다.
- **검출기 카메라의 1 m near clip 은 고치지 않았다.** 근접 포켓 추적(4순위)을 만들 때 반드시 다시 다뤄야 한다 —
  지금 설정으로는 1 m 안의 팔레트가 검출기 입력에서 사라진다.
- 쿼터뷰를 작업장 앞쪽 위에서 넓게 보도록 옮겨 계획 경로 전체가 한 화면에 들어온다.
- 결과: `success: true`, **75.275초, 삽입 종점 7.673 mm / 2.774 mrad — 첫 실행과 같다.** 카메라 구성을 바꾼 두
  실행에서 임무 수치가 같았으므로, 2026-09-22 실행과의 차이는 추가 카메라가 아닌 다른 원인일 가능성이 크다
  (가리지 않았다).
- `tools/deck/isaac_mission_views.py` 는 요청하지 않은 시점의 영상이 없으면 건너뛴다.
