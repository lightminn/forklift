# hall SLAM 재생: 기록 오도메트리 대 참 오도메트리 (2026-11-10 등록, 결과 보기 전 고정 — 총괄 결정: #407 다음 진단)

## 목적

- #407 진단(`2026-11-10-hall-cusp-stall.md`)은 사슬의 첫 고리를 기록으로 가르지 못했다. hall 지도 작성 모드에서 운반 초반에 생기는 −y 0.13–0.19 m 추정 편향이 오도메트리 입력에서 유도되는지, 지도 작성 SLAM 자체의 성질인지가 남았다.
- 같은 스캔을 새 지도 작성 브리지에 두 번 재생한다. 한 번은 기록된 오도메트리(`odom_base`)로, 한 번은 참값(`truth_base`)으로 준다. 두 팔의 운반 중 추정 오차를 비교한다.

## 방법

- 세션 4 개(복원본, ws1 `test_runs/live/rtb/`):
  - 정체: `g2a/g2a_01_H_sync`, `g2a/g2a_06_H_sync`, `j1_i1/j1_i1_3_off`
  - 완료 대조: `j6_c3off_b/j6_c3off_b_1_off`
- 재생 하나는 `deploy/slurm/slam_map_replay.sbatch OUT EPOCH_DIR ODOM` 이다(새로 만듦, 기존 위치 추정 재생 `slam_loc_replay.sbatch` 의 지도 작성판).
  - 컨테이너(`tools/slam_map_replay_container.sh`): live 세션과 같은 slam_toolbox online sync 와 같은 매개변수(`slam_toolbox_isaac_online.yaml`, `slam_bridge_container.sh` 의 기본값)로 돈다. 브리지는 `keyframe_gate every` 다.
  - 구동(`tools/slam_loc_replay_drive.py`, 기존 도구 그대로): live 브리지가 slam_toolbox 에 보낸 스캔(브리지 기록의 `sent != local`)만 같은 순서로 보낸다. 그래서 어느 팔이든 같은 스캔이 정합된다. 거리값은 기록된 깨끗한 거리에 러너와 같은 방식으로 다시 그린 잡음을 더한다(`default_rng([seed, 7])`, seed 0). 두 팔에 같은 잡음이고, live 의 잡음 흐름 그 자체는 아니다.
  - 팔: `--odom recorded` 와 `--odom truth`. 시간은 세션 끝까지다.
- 8 재생(4 세션 × 2 팔), 각 CPU 2·메모리 4G·한도 40 분. ws1 CPU 잡이고, rv3 가 멈춘 틈에 낸다(총괄 허용). 넓은 rv3 잡을 오래 막지 않게 배열 하나로 한꺼번에 낸다.

## 판정(`2026-11-10-hall-slam-replay/replay_eval.py`, 결과 보기 전 고정)

- 오차: 처리된 응답마다 `compose(map_from_odom, 보낸 오도메트리) − truth_base`. 단계는 live 기록의 같은 스캔 단계를 쓴다.
- 유효성(세션마다):
  - 두 재생 모두 끝까지 성공(status ok, 보낸 스캔 모두 응답).
  - 기록 팔이 live 의 편향을 재현한다. 운반 중앙값 |e| 가 live 와 0.03 m 안이고, e_y 의 부호가 같다. 재현하지 못하면 그 세션은 판정하지 않는다(재생이 live 를 대표하지 않음).
- 읽기(세션마다):
  - **오도메트리 유도**: 참 팔의 운반 중앙값 |e| 가 0.05 m 아래이고 기록 팔의 절반 아래.
  - **SLAM 자체**: 참 팔이 기록 팔과 0.03 m 안.
  - **혼합**: 그 사이.
- 전체: 유효한 세션이 모두 같은 읽기면 그 읽기, 아니면 "갈림"(세션별로 보고).
- 보고만: 각 팔의 복귀 단계 계단(시각·크기·참위치), 참 팔의 e_yaw.

## 한계

- 잡음은 다시 그린 것이다. 기록 팔의 재현 검사가 그 대표성을 본다.
- 참 오도메트리는 완벽한 오도메트리다. "오도메트리 유도"는 지금 오도메트리 모형(team_spec_quantized)의 오차가 편향을 만든다는 뜻이지, 어느 정도의 오도메트리면 충분한지는 아니다.
- 지도 작성 매개변수는 live 와 같다. 매개변수에 따른 차이는 재지 않는다.
- 결과는 진단이다. 완화책(③ 보정 직후 재계획, ④ cusp 보정 보류)은 이 결과를 본 뒤 총괄이 정한다.
