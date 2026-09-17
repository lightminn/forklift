# 하드웨어 선택과 확인 상태

기준일: 2026-09-15 (최초 작성 2026-09-10)

이 문서는 프로젝트 장비의 확정·권장·미확인 상태를 구분하는 정본이다. 구매 권장은 주문이나 보유를 뜻하지 않으며, 후보 카탈로그 값은 실물 측정값으로 사용하지 않는다.

## 현재 상태

| 항목 | 상태 | 결정과 남은 확인 |
|---|---|---|
| RGB-D 카메라 | **확정** | Intel RealSense D435i를 사용한다. 실물 장착 위치·외부 보정·Jetson 드라이버 동작은 미검증 |
| RGB-D **장착 위치** | ⚠️ **미확정 — 결정 대기** | 합성 측정은 전부 잠정값 (0.75, 0, 0.5)에서 냈다. **장착이 검출 근접한계를 직접 정한다**(검출기 게이트가 아니라 수직 화각이다). EPAL 6 실측: 0.50 m 에서 최근접 2.3 m, 0.27 m 에서 2.1 m, **브리프가 말하는 마스트 높이 0.90 m 에서 2.8 m**. 삽입은 그보다 가까이서 끝나므로 **마스트 장착이면 마지막 접근을 못 본다.** 후보 장착은 `sim/gazebo/build_scene_world.py` 의 `APPROVED_CAMERAS` 에 등록돼 있고, 측정은 `tools/measure_pocket_evidence.py --camera-z/--camera-tilt`, 캡처용 카탈로그는 `tools/retarget_scene_catalogue.py --camera` 로 낸다. 근거: `docs/validation/2026-09-14-epal6-capture-and-evaluation.md` §6 |
| 2D LiDAR | **확정 / 세부형 유력안** | Slamtec RPLIDAR를 사용한다. 과제 자료의 A2 계열 중 **A2M12가 유력하다**(2026-09-15 사용자 의견, 구매·수령 전이므로 확정이 아니다). 인터페이스와 스캔 설정은 장비 확인 후 고정. 카탈로그 값은 아래 표 참고 |
| 2D LiDAR **장착 위치** | ⚠️ **미확정** | 커밋된 산출물마다 가정이 다르다 — `sim/gazebo/scene_config.yaml` (0.75, 0, 0.50), `tools/deck/lidar_plane.py` (0.60, 0, 0.50). 목적이 다른 합성 실험이라 서로 같을 이유는 없으나 **둘 다 실측이 아니다.** 실제 장착과 후방·측면 관측 가능 범위는 H0 에서 측정한 뒤 고정한다 |
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
4. RPLIDAR 세부형(A2M12 유력) 확정 후 `sllidar_ros2` 스캔과 통신 안정성 확인
5. 두 센서 동시 동작, 저장장치 처리량, 온도·전력과 재부팅 후 재현 확인

librealsense의 Jetson 설치 문서는 참고 절차를 제공하지만 Orin Nano Super의 JetPack 7.2.1과 D435i 조합을 정확히 인증하는 문서로 간주하지 않는다. 실제 장치 시험 결과를 검증 기록에 남긴 뒤 버전을 동결한다.

## 2D LiDAR 세부형 — A2M12 유력안 (2026-09-15)

사용자가 A2M12로 갈 것 같다고 밝혔다. **구매·수령 전이므로 확정이 아니다.** 아래는 제조사 비교표의 값이며 실측값으로 쓰지 않는다. 같은 페이지에 세 변종이 함께 있고 **세부형이 바뀌면 절반이 달라지므로** 셋을 모두 적는다.

| | A2M7 | A2M8 | **A2M12 (유력)** |
|---|---|---|---|
| 측정 거리 | 0.2 ~ 16 m | 0.2 ~ 12 m | **0.2 ~ 12 m** |
| 표본율 | 16 K sample/s | 8 K sample/s | **16 K sample/s** |
| 회전 속도 | 10 Hz (5 ~ 15 Hz) | 10 Hz (5 ~ 15 Hz) | **10 Hz (5 ~ 15 Hz)** |
| 각 분해능 | 0.225° | 0.45° | **0.225°** |

출처: [Slamtec RPLIDAR A2 제품 페이지](https://www.slamtec.com/en/lidar/a2) (2026-09-15 확인). 같은 페이지가 두께를 "cut the thickness to only 4cm"로 적는다. 통신 인터페이스, 공급 전압, 무게는 이 페이지에 없어 미확인이며 제품 Spec 하위 페이지를 아직 확인하지 않았다. 데이터시트 PDF는 직접 내려받기가 막혀 있다(HTTP 403).

⚠️ **각 분해능은 회전 속도에 따라 달라진다.** 표의 0.225°는 표본율 16 K 를 기본 10 Hz 로 나눈 값(회전당 1,600점)이다. 같은 센서라도 5 Hz 에서는 0.1125°, 15 Hz 에서는 0.3375° 가 된다. 두 값을 독립된 사양처럼 읽지 않는다.

**시뮬레이션 설정과 비교하지 않는다.** `sim/gazebo/scene_config.yaml` 의 LiDAR(회전당 360점·5 Hz)는 2026-09-10 에 승인·동결된 **단발 기준 실험**의 값이다. 설계 문서가 "범용 2D scan 을 쓰며 RPLIDAR SKU 충실도를 주장하지 않는다"고 명시했고, `build_sensor_world.py` 는 이 값이 조금이라도 다르면 거부하며 생성물 SHA-256 이 시험에 고정돼 있다. 즉 실기 사양에 맞출 대상이 아니다. 현재 장면 촬영 경로인 `build_scene_world.py` 에는 LiDAR 자체가 없다(RGB-D 만). 실제 스캔 설정은 장비 입고 후 `sllidar_ros2` 로 정하고 별도 검증 기록에 남긴다.

## 공식 참고 자료

- [JetPack SDK 다운로드와 지원 플랫폼](https://developer.nvidia.com/embedded/jetpack/downloads)
- [Jetson Orin Nano Developer Kit 빠른 시작](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html)
- [개발자 키트 하드웨어 구성과 M.2 슬롯](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html)
- [librealsense의 Jetson 설치 안내](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)
- [RealSense ROS 2 wrapper](https://github.com/realsenseai/realsense-ros)
- [Slamtec ROS 2 driver](https://github.com/Slamtec/sllidar_ros2)
