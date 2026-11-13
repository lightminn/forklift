# slam_toolbox TF 메시지 필터 탈락 — 기록 진단과 보류 (2026-11-14)

출발점: [S3 결과](2026-11-13-slam-online-bounded-s3.md) 5 절. 오프라인 재생의 무응답 12 건(S2 R0 3, S3 9)이 모두, 끊긴 바로 그 scan 시각의 slam_toolbox 로그 `Message Filter dropping message: frame 'laser' at time T for reason 'the timestamp on the message is earlier than all the data in the transform cache'` 와 함께 있었다. 완주한 실행에는 없었다.
상태: **재생 도구 결함 후보로 기록하고 보류한다(총괄 결정 2026-11-14).** 재개 계기는 4 절.

## 1. 원자료

- `2026-11-14-tf-filter-drop-records/tf_drop_live.py`: ws1 의 `test_runs`·`artifacts` 아래 slam_toolbox.log 가 있는 브리지 폴더를 모두 훑어(S2·S3 재생 폴더는 제외) 탈락 줄, 브리지 records 의 실패 응답, 같은 에폭의 `slam_link_failed` 언급을 센다. 출력 `tf_drop_live.jsonl`(ws1 분석 잡 18879).
- `live_epochs.txt`: ws1 에 남은 라이브(Isaac) 에폭마다 노드 종류·탈락 수·실행의 `failure_reason`.

## 2. 같은 경로인가

- 탈락은 slam_toolbox 공통 클래스(`SlamToolbox`, 2.8.5 `slam_toolbox_common.cpp` 484–493)의 `tf2_ros::MessageFilter` 에서 난다: scan 구독 → 목표 프레임 odom, 대기열 `scan_queue_size`, 대기 시간 `transform_timeout`(설정 0.2 s, 노드 시계 = sim 시각) → `laserCallback`. sync·localization·lifelong 노드가 모두 이 경로를 쓴다.
- 브리지(`forklift_ros/isaac_slam_bridge.py` `publish_scan`)도 두 모드에서 같다: scan 마다 /clock → /tf(odom→base_link) → keyframe 이면 /scan, 같은 stamp.
- 상류 Jazzy 의 MessageFilter 는 TF 대기 중의 어떤 예외(대기 타임아웃 포함)에도 같은 문구를 쓴다(S3 문서 5 절, Codex 검토). 문구만으로는 실제 캐시 범위 초과인지 대기 타임아웃인지 가를 수 없다.

## 3. 기록에서 본 것

- ws1 에 남은 브리지 폴더 96 개(sync 67·localization 29)를 훑었다.
  - 탈락이 있는 폴더는 오프라인 재생뿐이다: S2·S3 의 12 건(앞 문서들)과 `orin_cmp/ws1_map_r1`(오프라인 재생, sim 3,460.9 에 탈락과 60 s 실패 응답 1).
  - 위치 추정 폴더 29 개(H·A 재생 포함)에는 탈락이 없다.
- **라이브(Isaac) mapping 에폭 5 개는 모두 `slam_link_failed: scan 4851–5019: no reply within 1.00 s` 로 끝났지만, slam_toolbox 로그에 탈락이 한 번도 없다**(`live_epochs.txt`). 라이브의 slam_link_failed 는 그래프가 커진 mapping 의 1 s 기한 초과(2026-10-25 slam_link_failed 계획의 문제)이고, 탈락과 함께 나타난 적이 없다. 각 에폭 2(위치 추정 쪽)는 실패도 탈락도 없다.
- 한계: 라이브 기록 대부분은 gdrive 에 보관돼 ws1 에는 mapping 에폭 5 개만 남아 있다. 표본이 작다.

## 4. 판단과 재개 계기

- **분류: 재생 도구 결함 후보.** 지금까지 탈락은 오프라인 빠른 재생에서만 관측됐다. 실차는 벽시계로 돌아 빠른 재생의 /clock 진행이 없고, 운영 경로는 위치 추정(A)이다.
- **가설(미검증):** sim 시계로 재는 0.2 s TF 대기와 빠른 재생의 /clock 진행이 경합한다. 빠른 재생은 응답을 받자마자 다음 scan 들의 /clock·/tf 를 쏟아 내므로, 노드가 keyframe scan 의 TF 를 기다리는 동안 sim 시각이 대기 시간을 넘기거나 TF 수신 순서가 엇갈릴 수 있다.
- **재개 계기:** 재생 기반 평가를 다시 할 때, 또는 라이브에서 같은 탈락이 관측될 때. 그때 가설을 재생으로 확인한다 — 같은 입력에서 `transform_timeout` 을 늘린 실행 1–2 개(측정 칸이 필요 없는 CPU 잡)와, 탈락 시점의 필터 진입·TF 수신·ROS 시각·대기 시작/끝·예외 종류의 기록.
- 그 전까지 재생 기반 결과의 '무응답' 은 등록대로 판정 0 불합격으로 세고(S2·S3 문서), 기전은 미확정으로 둔다.
