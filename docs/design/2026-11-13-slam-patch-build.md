# slam_toolbox 패치 빌드 설계 — 온라인 SLAM 비교 S3 (2026-11-13)

계획: [온라인 SLAM keyframe 지연 상한](../plans/2026-11-13-slam-online-bounded.md) 3.3·3.4·5 장(S1 패치 빌드 1). 총괄 결정(S2 뒤): S3 는 C-WM → C → S1 순서로, 셋 모두 패치 빌드와 P0 동등성부터 한다. D 는 S3 결과를 보고 정한다.

## 1. 이미지

- `deploy/ros2/Dockerfile.slam_patched` → 태그 `forklift/ros2-slam:jazzy-2.8.5-p1-<패치 묶음 해시 8>`.
- 바탕은 배포 이미지 `forklift/ros2-dev:jazzy` 다. 그 안의 apt 판(`ros-jazzy-slam-toolbox 2.8.5-1noble.20260905`, ceres 2.2.0)을 그대로 두고, 소스 빌드를 `/opt/slam_patched/install` 에 덧씌운다. 같은 ROS·ceres·Eigen·boost 위에서 slam_toolbox 만 다르다.
- 소스는 GitHub `SteveMacenski/slam_toolbox` 태그 `2.8.5`(커밋 `ec8f7635`)다. 빌드 때 커밋 해시를 단언한다.
- 패치는 `deploy/ros2/slam_patches/NNNN-*.patch`(`git format-patch`)로 저장소에 두고, 빌드에서 `git am` 한다. 묶음 해시는 패치 파일들의 sha256 이다.
- 빌드 플래그: slam_toolbox 의 `CMakeLists.txt` 가 `CMAKE_BUILD_TYPE Release` 를 직접 정한다(4 행). 여기에 apt 빌드와 같은 `dpkg-buildflags` 의 CFLAGS·CXXFLAGS·LDFLAGS 를 준다. 부동소수 결과가 apt 판과 같도록 하려는 것이고, 같은지는 동등성(4 절)이 판정한다.
- 이미지 안 단위 시험(gtest, 패치가 더하는 순수 함수만)은 빌드 단계에서 돌리고, 실패하면 이미지를 만들지 않는다.

## 2. 켜고 끄기

- 패치 기능은 모두 **환경 변수**로 켠다. Mapper 생성 때 한 번 읽는다. 설정하지 않으면 모든 기능이 꺼지고 원래 코드 경로를 그대로 지난다. 이것이 P0 다.
  - `FORKLIFT_SLAM_TRACE=<파일>`: 계측(3.3 의 구간 시간). 설정하지 않으면 시계 읽기도 하지 않는다.
  - `FORKLIFT_SLAM_CWM_S=<T>`: C-WM 창(sim 초, 0 이하는 끔).
  - `FORKLIFT_SLAM_DELETE_OLDER_S=<T>`: C 삭제 나이.
  - `FORKLIFT_SLAM_S1=1`, `FORKLIFT_SLAM_S1_BUFFER=10`: S1.
- ROS 매개변수로 하지 않는 이유: slam_toolbox 의 매개변수 연결(`setParams`)과 직렬화를 건드리지 않고, 끈 상태를 '패치 코드가 한 줄도 돌지 않음'으로 단순하게 하려는 것이다. 값은 브리지 기록 옆 `slam_patch_config.json` 에 남긴다(Mapper 가 처음 읽을 때 씀).
- 재생 컨테이너(`tools/slam_cost_container.sh`)만 덧씌운 설치를 source 한다(`/opt/slam_patched/install/setup.bash` 가 있을 때). sbatch 는 `SLAM_IMAGE` 로 이미지를, `SLAM_PATCH_ENV="K=V ..."` 로 위 변수들을 docker `-e` 로 넘긴다. 라이브 컨테이너·런처는 바꾸지 않는다.

## 3. 패치 묶음 (순서 = 구현 순서)

1. **0001 계측.** keyframe 마다 한 줄 JSON:
   - 구간 시간: 전체 `Process`, scan 정합, `AddEdges`(그 안 `LinkNearChains`), `FindNearLinkedScans`(BFS 방문 노드·edge 수), `FindPossibleLoopClosure`(방문 ID 수·후보 chain 수), 굵은·고운 정합 수, `CorrectPoses`(Ceres 시간).
   - `addScan` 의 `smapper_mutex_` 대기, 지도 재생성(`updateMap`) 시간은 별도 줄(종류 표시)로 남긴다.
   - 쓰기: 링 버퍼에 쌓고 keyframe 처리 끝(잠금 밖)에서 한 번에 씀. 계측 자체의 쓰기 시간도 적는다.
2. **0002 C-WM.**
   - `FindPossibleLoopClosure` 의 시작 ID 를 '시각이 (현재 scan 시각 − T) 이상인 첫 상태 ID' 로 올린다.
   - C-WM 만 켤 때는 노드를 지우지 않아 상태 ID 가 연속이므로, 이분 탐색(`GetScan(name, id)->GetTime()`, scan 시각은 ID 순으로 단조)으로 찾는다.
   - `rStartNum` 이 이미 그보다 크면 그대로 둔다(`TryCloseLoop` 의 반복 호출과 맞음).
   - BFS·`LinkNearChains`·지도 재생성·Ceres 는 옛 그래프를 그대로 본다(계획 3.6) — 계측이 그 몫을 따로 보인다.
3. **0003 살아 있는 ID 순회 + 삭제 계약 + C.**
   - `FindPossibleLoopClosure`·`FindNearChains` 를 살아 있는 ID 의 `std::map` 반복자로 바꾼다. 이 경로는 삭제 기능(C·S1)이 켜졌을 때만 쓰고, 끄면 원래 순회다.
   - 삭제는 lifelong 순서를 따른다(`RemoveNodeFromGraph` → `RemoveScan` → `dataset_->RemoveData` → vertex 해제). 이를 위해 `dataset_` 소유자(SlamToolbox)에서 Mapper 로 삭제 콜백을 넘긴다.
   - running scans·last scan 에 든 노드는 지우지 않는다.
   - 앵커: 고정 노드를 지우면 살아 있는 가장 오래된 노드를 그 현재 자세로 새 고정점으로 정한다(`CeresSolver` 의 `first_node_`·`was_constant_set_` 갱신). 첫 `Compute` 전·후 두 경우를 시험한다.
   - C: keyframe 처리 끝에 상태 ID 순으로 sim T 초보다 오래된 노드를 지운다.
4. **0004 S1.** 계획 3.4(영구/임시, B = 10 뒤 삭제, 끊김이면 승격, 승격 기록).

## 4. 빌드와 동등성 (ws1, 총괄 신호)

- 빌드: `deploy/slurm/slam_patch_build.sbatch`(CPU 잡, `docker build`). 빌드 로그·이미지 digest·패치 묶음 해시·커밋 해시를 남긴다. 예상 30–60 분.
- 동등성 (가)(계획 5 장): P0(패치 이미지, 환경 변수 없음)와 R0(배포 이미지)를 같은 입력(sim 1,500 s 빠른 재생)으로 두 번씩 돌린다. 비교 대상:
  - 처리한 scan 열;
  - 응답 자세(차이 ≤ 1e-6 m·1e-6 rad);
  - 끝 그래프의 노드·edge 수(graph dump);
  - 유효 설정(`params_effective.yaml`).
  - 같지 않으면 다음 단계로 가지 않는다.
- 계측 켬(`FORKLIFT_SLAM_TRACE`)도 P0 와 응답이 같아야 한다(계측이 결과를 바꾸지 않음). 비용 차이는 적기만 한다.
- 패치 묶음이 바뀔 때마다(0002 → 0003 → 0004) 새 이미지로 P0 동등성을 다시 본다(약 20 분).

## 5. 시험

- gtest(이미지 안): C-WM 창 시작 ID 의 이분 탐색(경계·빈 창·전체 창), 살아 있는 ID 순회가 공백을 건너뛰고 chain 을 끊지 않음, 앵커 재지정(첫 `Compute` 전·후), S1 판정(가까운 영구 노드 있음/없음/정합 실패).
- 저장소 쪽 pytest: 컨테이너·sbatch 가 패치 환경 변수를 넘기고 라이브 경로는 손대지 않음, 패치 묶음 해시 계산.
- 재생: C-WM 켬 1 회(sim 1,500 s)에서 계측의 방문 ID 가 창 안에만 있음을 확인(동작 smoke, 판정 아님).

## 6. 미정·위험

- apt 판과 소스 빌드의 부동소수 결과가 같지 않으면(컴파일러 옵션 차이), 동등성 (가)가 실패한다. 그때는 R0 대신 P0 를 대조군으로 쓰는 것(계획 3.2 의 P0 정의)으로 충분한지, 계획 개정이 필요한지 총괄에 올린다.
- GitHub 접근이 ws1 docker 빌드에서 막히면 태그 tarball 을 노트북에서 받아 해시를 확인한 뒤 빌드 문맥으로 넘긴다.
