# Hybrid A* 팔레트 운반 시뮬레이션 설계

사용자 요청: 전체 지형이 보이는 카메라, 60fps, 무작위 위치 팔레트 회수, 평평한 바닥의 무작위 장애물과 목적지, Hybrid A* 구현·테스트. 목적지는 초록색 원이며 장애물은 NVIDIA Isaac Sim 공장·창고 물품 에셋을 사용한다.

## 경계와 구현

- `src/forklift_core/planning/`: ROS·Isaac Sim 비의존 Hybrid A*. 후륜축 기준 연속 x/y/yaw를 bicycle model로 전개하고 격자 상태를 검색한다. 전후진, 최소 회전 반경, 기어·조향 비용, 차체/적재 외곽 충돌 검사를 포함한다. 정확한 목표 연결도 운동학·충돌 제약을 만족해야 한다. 최단 시간/전역 최적성을 주장하지 않는다.
- `src/forklift_core/control/`: 경로의 기어 전환을 보존하는 속도·조향 추종. 후륜축 오차와 경로 곡률을 사용하고, 실측 시뮬레이터 관절 상태와 추종 오차를 기록한다. 승강·삽입 순서는 시뮬레이션 mission에 둔다.
- `sim/isaac/`: 공식 환경·물품 USD 로딩, 실제 물품 외곽에서 계획 장애물 작성, seed 고정 스폰, 초록색 목적지 원, 전체 조감 카메라, 물리 실행과 결과 저장. 패키지는 editable 설치해서 사용한다. 호스트 접속·경로는 CLI와 로컬 환경 기록에 둔다.
- 경로는 비적재 접근과 적재 운반을 각각 계획한다. 접근 목표는 팔레트 포켓 방향의 정렬 지점이다. 삽입·승강 후 후진으로 포켓을 벗어나 운반 경로를 추종하고, 원 중심에 팔레트를 하역한 뒤 포크를 뺀다.
- 팔레트 질량·마찰은 기존 provisional 물리 실험의 합성 가정을 유지한다. 조향 응답 점검 결과를 바탕으로 실행 설정에서만 조향 토크를 4Nm에서 20Nm, 강성 1000·감쇠 100으로 바꾼다. 이 값은 실측 하드웨어 사양이 아니다. 물체 순간이동이나 팔레트 고정 부착으로 운반을 흉내내지 않는다. 좌표 피드백은 시뮬레이터 상태이며 영상 인식·SLAM은 이번 요청 범위가 아니다.

## 검증

계획 단위시험: 직선·곡선·후진, 목표 방향, 경계·차체 모서리·얇은 장애물, 적재 외곽, 경로 없음, seed 재현. 이전 임의 곡선의 곡률 제한 위반을 재현하고 새 경로에서 제한 준수를 검사한다. 추종기는 전진/후진/기어 전환을 독립적으로 검사한다.

실물 에셋 배치의 물리 바운딩과 유효 초기 스폰을 확인한 후 고정 seed의 end-to-end 접근·삽입·승강·장애물 운반·하역·이탈을 실행한다. 성공률에 계획 실패·제어 실패·무효 스폰을 숨기지 않는다. 장애물 접촉/겹침, 적재물 낙하, 최종 원 중심 오차, 60fps 디코딩과 카메라 구도를 검증한다. 기존 저장소 테스트와 새 테스트를 함께 실행한다.

## 근거

Hybrid A* 연속 상태 전개와 탐색의 기준: [Dolgov et al., Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf). 실제 회전 반경·전후진 비용·외곽 검사 설정의 참고: [Nav2 Smac Hybrid-A*](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/).
