# 개발 중간 정리 — 2026-09-11

**현재 위치: 개발 환경과 합성 센서 검증 기반 완료. 팔레트 인식·실물 구동·자율 작업은 착수 전.** 리보틱스 팀의 2026-2 임베설 프로젝트이며 전체 개발 순서는 [전체 로드맵](../plans/2026-09-11-development-roadmap.md)을 따른다.

이번 정리는 기존 실행 기록·JUnit·JSON·소스 해시를 다시 읽어 작성했다. 문서 정리를 위해 시뮬레이션이나 전체 시험을 재실행하지 않았다. 정리 시점의 실행 소스 43개는 마지막 성공한 Gazebo snapshot과 모두 일치했다. 이후 flaky 시험 수정으로 `tests/integration/`이 바뀌면 snapshot 해시도 바뀌므로, 검증 실행의 해시는 아래 값으로 남긴다.

## 결정과 현재 상태

| 항목 | 현재 상태 |
|---|---|
| 코드 관리 | 로봇 코어·ROS 2·시뮬레이션·향후 MCU 펌웨어를 한 저장소로 관리. 발표는 별도 저장소 |
| 센서 | D435i 확정, RPLIDAR 확정·A2 예정. 실제 장착 보정과 A2 세부형은 미확인 |
| 온보드 컴퓨터 | Orin Nano Super 개발자 키트 8GB + NVMe 256GB 권장 기록. 구매·실물 실행 확인 없음 |
| 차체 | 상품 자료 기반 DLS08 후보의 잠정 모델. 실물 동일성·조향·구동·승강·전원 미확인 |
| 개발 환경 | 노트북 Ubuntu 24.04 / ROS 2 Jazzy 컨테이너. 원격 Gazebo Harmonic 센서 검사, MuJoCo 빠른 모델 검사 |
| 원격 운영 | 리보틱스 팀 공간, 실행별 snapshot·Slurm·결과 회수·해시 검증 구현. 기존 연구 작업과 분리 |
| 저장소 구조 | 컨벤션 합의 완료. 2026-09-11 같은 날 후속으로 `src/forklift_core/`·`examples/`·`tests/unit|integration/` 구조 전환을 적용했다([전환 검증 기록](2026-09-11-src-layout-migration.md)) |
| M1-b (같은 날 후속) | 포켓 관측 계약·장면 로더([기록](2026-09-11-pocket-observation-contract.md)), 카탈로그 v1 100장면·장면 월드 생성([기록](2026-09-11-scene-catalogue-and-world.md)), 원격 캡처로 100장면 데이터 세트 생성·병합·로더 검증([기록](2026-09-11-scene-dataset-v1.md)). 데이터는 Git 밖(`data/synthetic_scenes/catalogue_v1/`, 로컬·원격). 팔레트 인식기(M2)는 미착수 |
| Git | 정리 시점 HEAD `fc3c1d91e93d09e15d9e3f01339073e3490ae26d`, 이후 환경·Gazebo·문서 변경은 미커밋이었다. 같은 날 사용자 승인으로 이 변경을 commit해 인계했다(해시는 `git log`) |

장비와 배포 후보의 정본은 [hardware.md](../hardware.md), 역할 분리는 [ADR 0001](../decisions/0001-development-and-deployment-platforms.md)이다. Jetson의 OS·JetPack·SDK 조합은 실제 센서 동시 구동 후 동결한다.

## 구현·검증된 것

| 작업 | 확인한 증거 | 증명하지 않는 것 |
|---|---|---|
| 센서 수학 코어 | RGB-D 역투영, 센서→base 좌표 변환, LiDAR 점 변환의 합성 시험 | 영상에서 팔레트나 포켓을 검출하는 기능 |
| 잠정 지게차 모델 | URDF/MJCF 형상·관절, MuJoCo 무부하 적분·렌더링. 원격 첫 실행 20초 적분과 4초 포크 승강 영상 | 실물 치수·물리 동등성, 폐루프 주행, 적재 안정성 |
| 원격 실행 도구 | `submit/status/collect`, 읽기 전용 소스 snapshot, Slurm 자원 제한, 종료 상태와 해시를 이용한 회수 | GPU 경로·장기 무인 운영 전체 |
| Gazebo 센서 장면 | 고정 지게차 visual·합성 팔레트·장애물, RGB/depth/CameraInfo/scan/TF/clock | 구동 가능한 Gazebo 지게차 또는 접촉·삽입 모델 |
| 센서 통합 | Slurm 979 `COMPLETED 0:0`, live 센서별 161개/32.0초, 저장·독립 replay 각각 162개/32.2초 | 실물 D435i/RPLIDAR 특성, 서로 다른 장면에 대한 인식 성능 |
| 회귀·패키징 | 마지막 호스트 145 passed / 1 rendering deselected, Jazzy 패키지 28 passed / skip 0. 두 시험 집합은 중복되므로 합산하지 않음 | Jetson ARM64 또는 실제 구동기 검증 |
| 영상 | bag에서 같은 시각의 RGB·depth·scan 162세트 추출, 1080p·32.4초 MP4, 전체 디코딩·대표 프레임 확인 | 움직이는 차체나 실제 포켓 추적 |
| M2 포켓 인식 | 합성 장면 v1 eval 30장면 1회 실행에서 양성 검출 18/18, 위치 오차 p95 8.3 mm, yaw p95 0.0019 rad, 음성 6장 위양성 0, 장면당 0.14 s. 고정 revision·clean 트리에서 실행([기록](2026-09-13-pocket-detector-m2.md)) | 실물 RGB-D 성능, 조명·재질 변화, 관측 추적(M3), 다양한 형상의 음성. lookalike는 꽉 찬 직육면체라 쉬운 음성이다 |

현재 RGB 원본 162장은 동일한 정지 장면이다. 이것을 학습/평가용 독립 표본 162개로 세거나 train/test로 나누지 않는다. 알려진 거리 ROI 검사는 데이터 연동 확인이며 팔레트 검출기가 아니다.

## 근거와 재개 위치

- [Gazebo 검증 기록](2026-09-10-gazebo-sensor-baseline.md): 실패 원인·수정·최종 결과·CPU llvmpipe 경계.
- [영상 검증 기록](2026-09-11-sensor-video.md) · [센서 영상](../../artifacts/20260910T151053Z_sensor_video_01/forklift_sensor_replay.mp4).
- [원격 MuJoCo 기록](2026-09-10-remote-model-smoke.md) · [포크 승강 영상](../../artifacts/20260910T125341Z_remote_model_smoke_01/cpu_render/physics/physics_smoke.mp4).
- 최종 Gazebo 원본: `artifacts/20260910T145835Z_gazebo_sensors_final/`. source snapshot SHA-256 `7308513575fd80103660eaddac462266219e96b1990acf9034e1c8d3a6c2682f`. 원격/로컬 결과 51개 파일 해시 일치 기록 보존.
- 이번 상태 대조: `artifacts/20260910T165749Z_roadmap_checkpoint_01/checkpoint_evidence.json`. 원격 서버를 새로 조회한 현황 보고가 아니라 보존한 실행 증거의 대조다.
- 재개 순서: **현재 변경 검토·인계 → 별도 구조 전환 → 다양한 합성 장면과 포켓 관측 계약 → 실제 영상 기반 포켓 위치 추정**. 실물 입고 일정 확인과 기계·전장 조사 준비를 병행한다.

이전 날짜의 검증 문서는 그 실행 시점 기록으로 보존한다. 현재 전체 상태는 이 문서와 README, 향후 우선순위는 로드맵에서 확인한다.
