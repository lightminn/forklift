# Gazebo 센서 관측 검증 기록

확인일: 2026-09-10. 범위는 고정 잠정 지게차 모델과 합성 RGB-D·2D LiDAR의 ROS 2 관측, 기록, 재생이다. 실물 드라이버·주행·팔레트 인식 성능 검증은 포함하지 않는다.

## 실행 환경 준비

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic 8.11.0의 전용 Docker 이미지를 Slurm CPU 작업으로 빌드했다. 베이스 이미지는 `deploy/gazebo/Dockerfile`의 digest로 고정하고, 실제 설치된 패키지 목록과 최종 이미지 ID를 함께 보관했다. apt 저장소의 미래 상태까지 고정한 재빌드를 주장하지 않는다.

- 이미지: `forklift/gazebo:jazzy-harmonic`
- 실제 ID: `sha256:489f4aa6f5363c3a5938742607865ab87d3b3cb8dde3750bd03bc69503b76ffe`
- Slurm 966: 2 CPU·4GiB, `COMPLETED 0:0`, 459초. Docker에도 CPU affinity와 memory/swap 한도를 적용했다.
- Slurm 967: Gazebo 버전과 ROS·rosbag2·TF·NumPy·Pillow import 성공.
- Slurm 968: Mesa llvmpipe CPU EGL 확인. 앞선 `eglinfo --display` 호출은 지원하지 않는 옵션으로 실패했으며, 도움말에 맞춘 `-B -p surfaceless` 호출이 성공했다.
- Slurm 969: 실제 Gazebo RGB-D 깊이 메시지 생성·수신 성공. 64×48 센서의 중앙 32×24 영역에서 2m 기준 평면을 확인했다. 유효 픽셀 768개, 최대 절대 오차 약 4.77e-7m. 이는 이상적인 합성 평면 검사 결과이며 센서 정확도가 아니다.

처음에는 기준 평면이 영상 전체를 덮는다고 가정했으나, 시야 가장자리의 광선은 평면을 벗어났다. 전체 3072개 중 288개가 unknown인 원본 결과와 잘못된 최초 판정을 보존하고, 평면 내부 중앙 영역으로 검사 조건을 바로잡았다.

Slurm 971에서는 같은 단일 센서 시험에 Docker `--network none`과 `GZ_IP=127.0.0.1`을 적용해 메시지 수신·`COMPLETED 0:0`을 확인했다.

원본 준비 기록: `artifacts/20260910T132757Z_gazebo_sensor_setup_01/`. 이미지 빌드·패키지 목록·EGL·센서 로그와 Slurm 완료 원문을 포함한다.

## 원격 실행 도구

`tools/submit_model_check.py`로 소스 snapshot 생성, 원격 Slurm 제출, 완료 상태 확인, 결과 회수와 SHA-256 대조를 수행했다.

- 실행 ID: `20260910T135500Z_remote_runner_cpu_01`
- Slurm 970: `COMPLETED 0:0`, 3초.
- 원격 Python CPU 시험: 100개 통과, 실패·오류·skip 0개.
- 첫 회수는 pytest의 임시 symlink를 산출물 보안 검사에서 거부하여 실패했다. runner 임시 디렉터리를 결과 영역과 분리하고, 해당 임시 영역만 회수 대상에서 제외한 뒤 같은 실행의 회수·해시 대조에 성공했다. 실제 결과 영역의 symlink는 계속 거부한다.
- 당시 source revision은 `fc3c1d91e93d09e15d9e3f01339073e3490ae26d`, 미커밋 변경을 포함한 snapshot SHA-256은 `b504b2698a230f7349bb7c34da9272a633040120fd5040945900dfcf66bf7704`다.

수정한 runner를 새 snapshot으로 다시 실행한 Slurm 972(`20260910T140700Z_remote_runner_cpu_02`)도 102개 시험·실패/오류/skip 0개, `COMPLETED 0:0`으로 끝났고 임시 디렉터리 정리와 결과 3개 해시 회수까지 성공했다. 실행 ID의 시각 표기와 실제 시작 시각은 구분하며 실제 시작·종료는 `job_result.json`을 따른다.

원본 결과: `artifacts/20260910T135500Z_remote_runner_cpu_01/`의 `job_result.json`, `pytest.xml`, `model-cpu.log`, `slurm_status.txt`, 회수 manifest.

## 통합 센서 검사

첫 전체 성공 실행은 `20260910T144748Z_gazebo_sensors_03`, Slurm 978이다. `COMPLETED 0:0`, 83초였으며 원격 결과 52개 파일의 SHA-256이 회수본과 일치했다. 약 94MB의 bag·로그·PNG를 보관했다. 소스 snapshot SHA-256은 `e56b3525536bfa6b7776a1d8caa007b94f62e778b5ec27064ec76b25509539cf`다.

| 단계 | RGB / depth / CameraInfo / scan 각각 | 각 센서 시간 범위 | 결과 |
|---|---:|---:|---|
| 실제 Gazebo→ROS 관측 | 161개 | 32.0초 | 통과 |
| 저장 bag 직접 읽기 | 162개 | 32.2초 | 통과 |
| 새 프로세스에서 bag 재생 | 162개 | 32.2초 | 저장 내용과 개수·시각 일치 |

저장·재생 `/clock`은 3224개, 32.23초 범위였고 `/tf_static` 1개가 두 센서의 변환을 전달했다. 원래 Gazebo·bridge·TF 프로세스는 모두 종료 코드 0으로 끝난 뒤 재생했다. 검증기 PID도 live와 replay가 다르며, 재생 시작 전 다른 publisher가 없는지 검사했다.

깊이 중앙/기준 ROI는 약 2.2500007m, 좌측 LiDAR 평면은 y≈2.0001034m였다. optical 점 `[0.5625002, -0.5625002, 2.2500007]`은 base 점 `[3.0000007, -0.5625002, 1.0625002]`로 변환돼 독립 기대값과 맞았다. 오차 허용값은 코드에 명시한 합성 장면 검사 기준이며 실물 정확도 보증이 아니다.

`runtime_evidence/home/.gz/rendering/ogre2.log`에서 `GL_RENDERER = llvmpipe (LLVM 20.1.2, 256 bits)`를 확인했다. 실제 RGB·depth·scan PNG와 replay RGB를 직접 열어 팔레트의 두 개구부, 깊이 경계, 전방/좌측 평면과 축 방향을 확인했다.

## 통합 중 발견한 실패와 수정

- Slurm 975: colcon 빌드는 성공했으나 기본 시험 실행기가 pytest를 선택하지 않아 `NO TESTS RAN`, exit 5로 중단됐다. `--python-testing pytest`를 명시한 뒤 실제 컨테이너 시험 24개가 통과했다.
- Slurm 976: RGB 최초 1프레임만 검정, 이후 14프레임은 정상임을 원본 바이트·PNG로 확인했다. 고정 simulation time `[0,2)s`를 초기화 구간으로 기록하고 이후 각 센서에 별도의 30초 이상 조건을 적용했다. 초기 원본도 bag에 남으며 제외 개수와 시각을 결과에 기록한다. 초기화 이후의 검정 영상은 실패한다.
- Slurm 977: live와 bag은 통과했으나 replay depth가 165개 중 76개여서 실패했다. reliable 송신에 compatible한 reliable 수신으로 변경한 Slurm 978에서 전체 개수 일치를 확인했다. 메시지 수용 기준을 낮추지 않았다.
- 977 완료 조회 SSH가 정지해 최종 Slurm 기록이 만료됐다. 새 연결로 실제 실패 JSON·원본 bag을 확인했고, 해당 조회와 그 후손 SSH만 종료했다. 978은 임시 keepalive/timeout 설정으로 정상 제출·회수했다. 정식 도구의 제한 시간 보강과 최신 inventory·리뷰 수정의 최종 실행 결과는 아래에 추가한다.

## 최종 변경 검증

최종 확인: **2026-09-11 00:02 KST**. run ID `20260910T145835Z_gazebo_sensors_final`, Slurm **979 COMPLETED / 0:0**, 실행 83초. 임시 SSH 래퍼 없이 정식 CLI의 제출 → 완료 확인 → 회수를 수행했다.

- 호스트: `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` **145 passed, 1 deselected**. deselected는 별도 렌더링 시험이다. Ruff 검사와 52개 Python 파일 포맷 검사 통과.
- Jazzy 컨테이너: 실제 colcon build/install 및 설치 경로 import 확인, pytest **28 passed**, 실패·오류·skip 0.
- 실시간 RGB/depth/CameraInfo/scan: 각각 **161개 / 32.0초**. 고정 초기 warmup `[0, 2)`초를 제외한 관측이다.
- 저장 bag 및 새 프로세스 재생: 각각 **162개 / 32.2초**, 각 스트림의 개수와 첫·끝 타임스탬프 일치. clock 3,226개 / 32.25초, static TF 메시지 1개에 두 변환 포함.
- bag의 원시 데이터 **4,112행**과 토픽별 행 수를 metadata 선언과 실제 역직렬화 결과로 대조했다. warmup 원시 메시지도 bag에 보존했다.
- 재생 전에 기존 Gazebo·bridge·TF 발행 프로세스 종료(exit 0)를 확인했고, 새 validator 시작 시 6개 토픽의 기존 publisher가 모두 0이었다.
- 최종 live RGB/depth/scan 및 replay RGB PNG를 직접 열어 팔레트의 두 포켓, 전방·좌측 평면과 축 방향을 확인했다. 세 단계의 대표 PNG 해시도 일치한다.
- depth ROI 2.2500007m, LiDAR 전방 x 2.2500514m 및 좌측 y 2.0001034m. optical → base 축 변환도 예상 위치와 일치했다.
- 원격 결과 **51개 파일의 SHA-256**을 로컬에서 재계산해 모두 일치함을 확인했다. Slurm 최종 상태 원문과 `collection.json`은 회수 시 별도 저장했다.
- PNG 저장 예외가 성공으로 보고되지 않도록 수정했고, 종료된 leader가 남긴 자식 프로세스도 해당 실행의 process group 안에서 정리한다. SSH/rsync는 전용 process group과 제한 시간으로 종료하며, 기존 SSH multiplex 연결에 의존하지 않는다. 이 실패 경로들의 RED → GREEN 회귀시험을 수행했다.

최종 source snapshot SHA-256: `7308513575fd80103660eaddac462266219e96b1990acf9034e1c8d3a6c2682f`.

결과: `artifacts/20260910T145835Z_gazebo_sensors_final/`의 `job_result.json`, `slurm_status.txt`, `collection.json`, `sensor_smoke/result.json`, `sensor_smoke/{live,stored_bag,replay}/result.json` 및 PNG. 이 디렉터리는 실행 후 수정하지 않는다.

그래픽은 실제 Ogre 로그의 **llvmpipe (LLVM 20.1.2, 256 bits)**로 확인했다. 이번 검증은 CPU 소프트웨어 렌더링을 이용한 정적 합성 센서 연동이며, GPU 성능·실제 D435i/RPLIDAR 특성·차체 운동·팔레트 인식·자율주행을 검증한 결과가 아니다.
