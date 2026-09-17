# Isaac Sim 5.1 지게차·창고 실행 검사

사용자 요청으로 HPC의 기존 Isaac Sim에서 프로젝트 모델을 가져오고 NVIDIA
공식 예시 환경에 배치했다. 사용자는 별도 factory 파일이 없으며 NVIDIA 예시를
찾아 사용하도록 확인했다.

## 입력과 범위

- 코드 revision: `7e3d0fa62b313dc199973fe49fb1e07daa4f3c98`. 모델 원본을
  변경하지 않았다. 실행 도구는 `artifacts/`의 일회성 호환성 조사 코드다.
- 지게차: `sim/models/dls08_provisional/forklift.urdf`.
  SHA-256: `678ba31ccce95e7af71b24e243c6f3b17d5b1d893bb32c123c39885a156ff954`.
  후보 카탈로그와 추정치로 만든 잠정 모델이며 실물 검증 모델이 아니다.
- 환경: NVIDIA Isaac 5.1의 `Isaac/Environments/Simple_Warehouse/full_warehouse.usd`.
  [공식 환경 목록](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_environments.html#warehouse)의
  선반·장애물·지게차가 있는 창고 예시다. 로봇팔 조립용 Factory 학습 과제와 구분한다.
- 입력 종류: `synthetic`. 검증 종류: URDF→USD 가져오기, 무부하 물리 계산,
  외부 관찰 카메라의 RGB·깊이 렌더링.
- ROS 2, D435i/RPLIDAR 드라이버, 인지·SLAM·자율주행·포켓 추적·적재 작업은
  이번 실행에 연결하지 않았다. 과제 A–D 시나리오 통과를 뜻하지 않는다.

## 환경과 가져오기

Isaac Sim `5.1.0-rc.19+release.26219.9c81211b.gl`, NVIDIA GeForce RTX 5070
(12,227 MiB), 드라이버 `580.126.09`에서 Slurm GPU 할당으로 실행했다.
개인 SSH 별칭·계정·설치 절대 경로는 Git 제외 `ENV.local.md`에 기록했다.
기존 설치, 드라이버, 스케줄러 설정은 변경하지 않았다.

[5.1 URDF importer API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.asset.importer.urdf/docs/index.html)를
사용했다. `make_default_prim=True`, `fix_base=False`,
`import_inertia_tensor=True`, `merge_fixed_joints=False`,
`self_collision=False`, `collision_from_visuals=False`, SI 길이 단위를 명시했다.
원래 URDF 충돌 형상·질량·관성을 사용하고, 드라이브는 실험용으로 별도 지정했다.

가동 관절 7개(`fork_lift`, 좌우 조향, 바퀴 회전 4개)가
`/World/Forklift/base_link` articulation 아래에서 인식됐다. 중력 아래 자유
차체를 바닥 위 0.03 m에서 시작했다. 관절 목표를 유지하는 실험용 gain과
자기 충돌 비활성화는 실물 구동·접촉 보정을 대신하지 않는다.

## 실행 기록

실행 ID는 `20260916T132917Z_isaac_probe`다. 원격 프로젝트의 `snapshots/`에
입력과 실행 코드, `artifacts/`에 실행별 로그·USD·이미지·JSON을 보존했다.

| Slurm job | 검사 | 관측 결과 |
|---|---|---|
| 999 | 최초 URDF 변환 | USD 기본 프림 미지정으로 배치 실패. Slurm 종료 코드만으로 성공 판정하지 않음 |
| 1000 | 기본 프림 지정 후 단독 장면 | 7개 관절, 무부하 정착, RGB·깊이 출력 확인 |
| 1001 | NVIDIA 창고 정적 검사 | 26,404개 prim 장면에서 1,200 step 실행. 실제 시뮬레이터 시간 약 10초 |
| 1002 | 넓은 화각·포크 승강 기록 | 1,200 step, 10.0000005초, RGB 300프레임과 깊이 배열 생성. `COMPLETED`, exit `0:0` |
| 1003 | 새 프로세스 USD 재열기·영상 인코딩 | 저장 장면에서 7개 관절·120 step·RGB 재확인. H.264 10초 영상 생성. `COMPLETED`, exit `0:0` |

승강 실험은 1초 정지 후 2초 동안 목표를 0.15 m까지 올리고, 유지한 뒤
7–9초에 내리는 미리 지정한 관절 명령이다. 실제 최대 승강량은 **0.140629 m**,
종료 승강량은 약 **0.000000385 m**였다. 목표와 약 9.4 mm 차이가 있으므로
정밀 추종 검증으로 해석하지 않는다. 무부하이며 팔레트 적재를 시도하지 않았다.
기록된 차체 최대 수평 변위는 약 0.0535 mm였다.

카메라 출력은 1280×720이며 최종 깊이 배열의 양수 유한값 비율은 100%,
범위는 약 3.218–25.151 m였다. 관찰 카메라 출력의 수치 검사이며 실물 센서
정확도 검증은 아니다. 초기화·재질 준비·PNG 저장을 포함한 job 1002의
스크립트 시간은 약 260.7초였으므로 10초 영상 길이를 실시간 처리 속도로
해석하지 않는다. 실행 중 GPU 메모리 표본은 3,885 MiB였으며 최고 사용량을
연속 측정한 결과는 아니다.

초기 실행에서는 공유 설치의 캐시 일부에 쓰기 권한이 없어 셰이더·재질 캐시
경고가 발생했다. 후속 실행은 프로젝트 전용 `--portable-root`를 지정했다.
또한 관찰 카메라의 Isaac API 길이 단위를 확인해 초점거리 2.0
(USD `focalLength=20`)으로 전체 차체를 볼 수 있게 조정했다.

초기 importer는 시각 형상이 없는 조향 carrier와 fork-tip 링크에 대해 빈
visual 참조 경고를 출력한다. 실험 로그에 그대로 보존했다. 센서 카메라는
외부 관찰용이며 실제 D435i의 장착 위치나 내부 파라미터를 재현하지 않는다.

직접 확인한 이미지에서 창고 재질은 표시되지만 지게차는 회색으로 렌더링됐다.
URDF의 노란색·검정색 등 재질 색상 전달은 확인되지 않았으며, 이번에 형상과
물리 호환성 문제와 분리해 제한으로 남겼다.

## 회수한 결과와 재실행

- [10초 승강 영상](../../artifacts/20260916T132917Z_isaac_probe/warehouse_demo/warehouse_lift_demo.mp4)
- [승강 중 대표 화면](../../artifacts/20260916T132917Z_isaac_probe/lift_raised.png)
- [실행 지표·관절 시계열](../../artifacts/20260916T132917Z_isaac_probe/warehouse_demo/result.json)
- [USD 장면](../../artifacts/20260916T132917Z_isaac_probe/warehouse_demo/scene.usda)
- [새 프로세스 검증](../../artifacts/20260916T132917Z_isaac_probe/reopened/result.json)
- [실제 제출 명령](../../artifacts/20260916T132917Z_isaac_probe/submitted-1002.sh)와
  [일회성 실행 코드](../../artifacts/20260916T132917Z_isaac_probe/source/probe_final.py)

USD는 원격 생성 모델의 절대 참조와 NVIDIA 외부 에셋 참조를 포함한다.
**동일 HPC에서 재열기를 검증**했으며, 로컬 오프라인 독립 배포용 번들은 아니다.
재실행할 때에는 제출 스크립트의 출력 디렉터리를 새 경로로 지정하고 같은
Slurm GPU 할당을 사용한다. `--output` 경로가 이미 있으면 덮어쓰지 않고 거부한다.
재실행 코드·명령과 큰 산출물은 Git 제외 실행 아카이브에 보존한다.

회수 압축 파일 SHA-256:
`1f5bb2bfde9917b84fea8e1fd246b4bb4c76c0ba5cefa34858048bac7eecd141`.
압축 파일과 내부 **57개 파일**의 SHA-256을 로컬에서 대조했다. 원격 원본 PNG
프레임들은 서버에 보존하고, 로컬에는 영상·대표 이미지·깊이·USD·로그를 회수했다.

## 로컬 검사

로컬 전용 환경에 editable `.[dev,model]`을 설치했다. 기존 비시뮬레이션
시험은 **679 passed, 4 skipped, 1 failed**였다. 실패한
`test_prepare_core_environment_installs_snapshot_wheel_into_run_venv`는 호스트
Python 3.12에 `ensurepip`가 없어 중첩 venv 생성이 실패한 환경 문제다.
기존 프로젝트 코드는 수정하지 않았으며 이 실패를 통과로 세지 않았다.
`examples/sensor_geometry.py`는 정상 실행했다.

## 해석

현재 잠정 지게차를 Isaac Sim에서 가져와 NVIDIA 창고 안에서 물리 계산하고
관찰 영상을 생성하는 것은 가능하다. 기존 MuJoCo/Gazebo와 동역학이 같다는
검증은 아니며, Gazebo ROS 관측·재생 기준선을 대체하지 않는다. 실제 주행이나
팔레트 작업에는 별도 제어기와 센서 어댑터, 접촉·관절 파라미터 검증이 필요하다.
