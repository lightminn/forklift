# 하드웨어 선택과 확인 상태

기준일: 2026-09-10

이 문서는 프로젝트 장비의 확정·권장·미확인 상태를 구분하는 정본이다. 구매 권장은 주문이나 보유를 뜻하지 않으며, 후보 카탈로그 값은 실물 측정값으로 사용하지 않는다.

## 현재 상태

| 항목 | 상태 | 결정과 남은 확인 |
|---|---|---|
| RGB-D 카메라 | **확정** | Intel RealSense D435i를 사용한다. 실물 장착 위치·외부 보정·Jetson 드라이버 동작은 미검증 |
| RGB-D **장착 위치** | ⚠️ **미확정 — 결정 대기** | 합성 측정은 전부 잠정값 (0.75, 0, 0.5)에서 냈다. **장착이 검출 근접한계를 직접 정한다**(검출기 게이트가 아니라 수직 화각이다). EPAL 6 실측: 0.50 m 에서 최근접 2.3 m, 0.27 m 에서 2.1 m, **브리프가 말하는 마스트 높이 0.90 m 에서 2.8 m**. 삽입은 그보다 가까이서 끝나므로 **마스트 장착이면 마지막 접근을 못 본다.** 후보 장착은 `sim/gazebo/build_scene_world.py` 의 `APPROVED_CAMERAS` 에 등록돼 있고, 측정은 `tools/measure_pocket_evidence.py --camera-z/--camera-tilt`, 캡처용 카탈로그는 `tools/retarget_scene_catalogue.py --camera` 로 낸다. 근거: `docs/validation/2026-09-14-epal6-capture-and-evaluation.md` §6 |
| 2D LiDAR | **확정 / 세부형 미확인** | Slamtec RPLIDAR를 사용하며 과제 자료의 A2를 계획한다. A2 세부형, 인터페이스, 스캔 설정은 장비 확인 후 고정 |
| 상위 제어기 | **구매 권장 / 미구매** | NVIDIA 공식 Jetson Orin Nano Super Developer Kit 8GB 권장 |
| 저장장치 | **구매 권장 / 미구매** | M.2 2280 NVMe 256GB 권장. 128GB는 이미 보유했거나 비용 제약이 있을 때 허용 |
| 차체 | **미확인 / 입고 예상 2026-09-18~10-01** | `dls08_provisional`은 외형 대응 후보 모델이다. SKU 동일성, 치수, 조향·구동·승강 구조는 수령 후 측정. 차체·센서·Jetson 입고는 3주차 또는 4주차로 예상(2026-09-13 사용자 답변) |

과제 자료의 Gemini 335Le는 참고 센서이며 선택 장비가 아니다. 다른 목적으로 접근 가능한 Jetson도 이 프로젝트 장비로 간주하지 않는다.

## 상위 제어기 권장안

비용을 우선하면서 과제의 ROS 2 센서 처리와 온보드 실행을 시작하는 구성으로 **NVIDIA 공식 Jetson Orin Nano Super Developer Kit 8GB + 256GB M.2 2280 NVMe** 조합을 권장한다. 시장 전체의 최저가나 실제 처리속도를 보장하는 판단은 아니다.

- 공식 개발자 키트는 캐리어 보드와 전원 구성을 포함해 별도 보드 설계 없이 개발을 시작할 수 있다.
- 개발자 키트의 M.2 Key M 슬롯은 2280 NVMe와 PCIe Gen3 x4를 지원한다.
- 256GB는 ROS 2, 센서 SDK, 로그·rosbag, 컨테이너 이미지를 함께 운용할 때 기본 여유를 확보하는 선택이다.
- 128GB는 기능상 배제하지 않는다. 이미 가지고 있거나 예산을 더 줄여야 할 때 사용하되, 원본 데이터와 장시간 결과는 원격 저장소로 옮기고 여유 공간을 관리한다.
- 현재 권장안은 구매하지 않았다. 차체 전원 분배, DC-DC 정격, 냉각, 케이스와 케이블은 차체 확인 후 별도로 선정한다.

초기 설치 후보는 JetPack 7.2.1 / Jetson Linux 39.2.1의 Ubuntu 24.04 기반이다. 개발자 키트의 최초 설정에는 16GB 이상 USB 저장장치와 포함된 19V 전원 공급 장치를 사용할 수 있다. 실제 구매 후 공식 빠른 시작 절차와 펌웨어 요구 경로를 다시 확인한다.

## 버전 고정 전 검증

JetPack 7.2.1과 ROS 2 Jazzy는 공통 Ubuntu 24.04 기준으로 맞추는 제안이다. 아래 실물 검증을 통과하기 전에는 배포 버전을 최종 고정하지 않는다.

1. ARM64에서 librealsense 빌드 또는 지원 패키지 설치
2. D435i RGB·depth·IMU 열거와 지속 스트리밍
3. `realsense-ros`의 정합 영상, CameraInfo, 시각과 TF 확인
4. RPLIDAR A2 세부형 확인 후 `sllidar_ros2` 스캔과 통신 안정성 확인
5. 두 센서 동시 동작, 저장장치 처리량, 온도·전력과 재부팅 후 재현 확인

librealsense의 Jetson 설치 문서는 참고 절차를 제공하지만 Orin Nano Super의 JetPack 7.2.1과 D435i 조합을 정확히 인증하는 문서로 간주하지 않는다. 실제 장치 시험 결과를 검증 기록에 남긴 뒤 버전을 동결한다.

## 공식 참고 자료

- [JetPack SDK 다운로드와 지원 플랫폼](https://developer.nvidia.com/embedded/jetpack/downloads)
- [Jetson Orin Nano Developer Kit 빠른 시작](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html)
- [개발자 키트 하드웨어 구성과 M.2 슬롯](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html)
- [librealsense의 Jetson 설치 안내](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)
- [RealSense ROS 2 wrapper](https://github.com/realsenseai/realsense-ros)
- [Slamtec ROS 2 driver](https://github.com/Slamtec/sllidar_ros2)
