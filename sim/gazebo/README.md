# Gazebo 센서 관측 기준선

정지한 합성 장면에서 Gazebo Harmonic의 RGB-D·2D LiDAR 관측을 ROS 2 Jazzy로 전달하고, 30초 이상 simulation time의 live·저장 bag·독립 재생을 각각 검사한다. 실물 D435i/RPLIDAR 드라이버, 장착 보정, 자율주행·포크 삽입·접촉·하중 검증은 구현하지 않는다.

## 실행

Jazzy/Harmonic 의존성 이미지에서 ROS 환경을 source하고 저장소 루트에서 실행한다. `/workspace` 소스는 읽기 전용, `/output`은 쓰기 가능한 별도 마운트로 사용한다.

```bash
python3 sim/gazebo/run_sensor_smoke.py --output /output --duration 30
```

`/output/sensor_smoke/`가 이미 있으면 덮어쓰지 않고 실패한다. 마운트 루트에 원격 runner의 로그가 있는 것은 허용한다. ROS 패키지는 `/output/.runtime/` 아래 colcon build/install에 설치하며 설치된 setup을 source한다. 소스 경로를 import 경로에 수동 삽입하지 않는다. 원격 runner는 종료 후 `.runtime/`을 제거한다. 빌드·시험·Ogre 정규 로그와 JUnit XML은 `sensor_smoke/runtime_evidence/`로 보존한다.

실제 원격 제출은 루트의 [원격 실행 안내](../../docs/development.md)를 따른다. 소스 스냅샷, Slurm/Docker 제한, 해시 회수는 해당 runner의 책임이다.

## 합성 장면과 독립 기대값

`scene_config.yaml`이 장면·bridge·static TF 생성의 정본이다. 이 검사는 하나의 고정 reference experiment이며 설정이 독립 검증 기대값과 다르면 거부한다. 새 센서 설정/목표물 시험은 기대값 검토와 함께 별도로 추가해야 한다.

- 기존 잠정 URDF의 모든 visual을 관절 중립 위치로 고정한다. 차체 collision과 actuator는 생성하지 않는다. 원본 URDF가 바뀌면 visual 개수도 따라간다.
- 팔레트는 상·하판과 지지대 세 개로 구성해 두 개의 실제 개구부를 만든다. 팔레트 위치/치수는 합성이다.
- 카메라와 LiDAR 원점은 base 기준 (0.75, 0, 0.5)m이다. 카메라 optical +x→base −y, +y→−z, +z→+x를 TF로 표현한다.
- 전방 평면의 앞면은 base x=3m, 좌측 평면은 y=2m이다. 따라서 전방 axial depth는 2.25m, 좌측 LiDAR 평면까지 거리는 2m이다.
- 320×240, fx=fy=160, cx=160, cy=120인 합성 pinhole에서 비중앙 픽셀 (200,80)의 기대 base 점은 (3, −0.5625, 1.0625)m이다. Gazebo principal point의 반 픽셀 차이는 작은 허용 오차 안에서 검증한다.
- 카메라 RGB/depth/CameraInfo와 360° scan은 5Hz다. noise, 실제 SKU의 정확도·노출·동기화 특성을 모사하지 않는다.

검증기는 생성기 계산을 호출하지 않는다. 실제 메시지의 encoding/해상도/stride/payload, CameraInfo, frame, TF, 시간 증가, 관측 거리와 좌표를 위의 별도 기대값에 대조한다. 알려진 ROI의 NaN/Inf는 실패이며 scan의 +Inf는 미반사로 보존한다.

## 증거와 실패 처리

Gazebo 렌더러 초기화 때문에 simulation time [0, 2)초만 고정 warmup으로 제외한다. 초기 검정 RGB 한 프레임이 실제 probe에서 확인됐다. 제외 기준은 관측 내용이나 첫 수신 시각에 따라 늘어나지 않는다. 원시 메시지는 bag에 보존하며 제외 count와 timestamp 범위를 별도 기록한다. 2초 이후 검정 RGB는 실패다.

Warmup 이후 각 동적 stream은 최소 30초를 차지하고 150개 이상 메시지를 가져야 한다. RGB·depth·CameraInfo는 30초 이상 동일 timestamp가 일치해야 한다. 센서 간 최대 누락 간격은 0.61초다. live 기록에는 종료 여유로 2초를 더 확보한다.

1. 실제 `/clock` 수신 확인 후 static TF publisher를 시작한다. 설치된 노드가 live 메시지를 검증하며 recorder가 `/clock`, `/tf_static`과 네 센서 토픽을 기록한다.
2. recorder를 정상 종료하고 Gazebo·bridge·TF 프로세스를 종료한다.
3. 별도 프로세스가 rosbag2 저장 데이터를 직접 deserialize하여 검증한다. metadata가 선언한 전체·토픽별 메시지 수와 실제 읽은 행 수를 대조해, 30초 이상이 남아 있는 일부 행 누락도 거부한다.
4. 새 validator는 원래 publisher가 없는지와 빈 초기 상태를 확인한다. player는 기록된 `/clock`을 재생한다. 별도 clock을 생성하지 않는다.
5. replay의 count·처음/끝 timestamp·기간을 저장 bag 결과와 정확히 대조한다. 큰 depth 메시지의 손실을 재전송으로 복구할 수 있도록, bridge가 제공하는 reliable QoS에 맞춰 validator도 reliable을 요청한다.

`result.json`은 전체 결과이며 `live/`, `stored_bag/`, `replay/`에 별도 결과·RGB·깊이·scan PNG가 생성된다. `depth.png`의 색상 범위는 0–5m, invalid는 검정이다. `scan.png`는 위가 +x 전방, 왼쪽이 +y이며 50px/m다. PNG 생성 성공만으로 시각 검토를 통과 처리하지 않는다.

관측 누락·잘못된 좌표/거리·잘린 bag·실행 timeout·필수 child 종료는 nonzero다. 자신이 생성한 프로세스 그룹만 종료하며 실패 산출물을 남긴다. live/replay는 실행 단계이고 `source_provenance`는 모두 `synthetic`이다.

호스트 단위시험은 ROS 없이 메시지 계약과 장면·프로세스 경계를 검증한다.

```bash
python -m pytest ros2/src/forklift_ros/test tests/simulation/test_gazebo_sensor_world.py tests/simulation/test_sensor_smoke_runner.py -q -p no:cacheprovider
```

실제 Gazebo 렌더링·ROS·bag 증거는 [날짜가 있는 검증 기록](../../docs/validation/2026-09-10-gazebo-sensor-baseline.md)을 확인한다.
