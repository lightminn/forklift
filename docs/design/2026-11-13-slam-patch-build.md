# slam_toolbox 패치 빌드 설계 — 온라인 SLAM 비교 S3 (2026-11-13, v3)

계획: [온라인 SLAM keyframe 지연 상한](../plans/2026-11-13-slam-online-bounded.md) 3.3·3.4·5 장(S1 패치 빌드 1). 총괄 결정(S2 뒤): S3 는 C-WM → C → S1 순서로, 셋 모두 패치 빌드와 P0 동등성부터 한다. D 는 S3 결과를 보고 정한다.

## 1. 이미지

- `deploy/ros2/Dockerfile.slam_patched` → 태그 `forklift/ros2-slam:jazzy-2.8.5-p1-<패치 묶음 해시 8>`.
- 바탕은 배포 이미지 `forklift/ros2-dev:jazzy` 다. apt 판(`ros-jazzy-slam-toolbox 2.8.5-1noble.20260905`, ceres 2.2.0)은 그대로 두고, 소스 빌드를 `/opt/slam_patched/install` 에 덧씌운다.
- 소스: GitHub `SteveMacenski/slam_toolbox` 태그 `2.8.5`(커밋 `ec8f7635`).
  - 빌드 때 커밋 해시를 단언한다.
  - ws1 docker 빌드에서 GitHub 에 닿지 않으면, 노트북에서 만든 **git bundle**(해시 확인)을 빌드 문맥으로 넘긴다(tarball 은 `.git` 이 없어 해시 단언·`git am` 이 안 됨, Codex 설계 검토 P3).
- 패치: `deploy/ros2/slam_patches/NNNN-*.patch`(`git format-patch`)를 `git am` 한다. 묶음 해시는 패치 파일들의 sha256 이다.
- **같은 Dockerfile 이 두 이미지를 만든다**(빌드 인자 `PATCHES=none|all`):
  - **U0**: 무패치 소스 빌드. 동등성 실패 원인을 빌드 차이와 패치 차이로 가르려는 것이다(Codex P2).
  - **P1 이미지**: 패치 묶음.
- 빌드 플래그: 그대로 colcon 빌드한다. slam_toolbox `CMakeLists.txt` 가 `CMAKE_BUILD_TYPE Release` 를 직접 정한다(4 행, karto 도 7 행).
  - apt 판과 같은 플래그라는 주장은 하지 않는다(Codex P2). 빌드 로그에 최종 컴파일 명령(`compile_commands.json`)·컴파일러 판·의존성 판을 남긴다.
  - 같은지는 4 절의 실측 동등성(R0↔U0, U0↔P0)이 판정한다.
- **그래프 덤퍼도 이미지 안에서, 그 이미지의 헤더·라이브러리로 빌드한다**(Codex P1).
  - 지금 `tools/slam_cost_dump.sh` 는 컨테이너 안에서 `/opt/ros/jazzy` 헤더로 덤퍼를 컴파일한다. 덧씌운 라이브러리가 로드되면 apt 헤더의 `sizeof(Mapper)` 와 패치된 생성자가 어긋나 메모리가 깨질 수 있다.
  - 이미지는 `/opt/slam_patched/bin/slam_graph_dump` 를 덧씌운 설치의 include·lib 로 만들어 둔다. `slam_cost_dump.sh` 는 덧씌운 설치가 있으면 그 덤퍼를 쓰고 컴파일하지 않는다.
  - 덤퍼의 실제 로드 경로(`ldd`)를 `dump_build.log` 에 남긴다.
- 이미지 안 단위 시험(gtest)은 빌드 단계에서 돌리고, 실패하면 이미지를 만들지 않는다.

## 2. 켜고 끄기

- 패치 기능은 모두 **환경 변수**로 켠다. Mapper 생성 때 한 번 읽는다. 설정하지 않으면 모든 기능이 꺼지고 원래 코드 경로를 지난다. 이것이 P0 다.
  - `FORKLIFT_SLAM_TRACE=<파일>`: 계측.
  - `FORKLIFT_SLAM_CWM_S=<T>`: C-WM.
  - `FORKLIFT_SLAM_DELETE_OLDER_S=<T>`: C.
  - `FORKLIFT_SLAM_S1=1`·`FORKLIFT_SLAM_S1_BUFFER=10`: S1.
- 이유: slam_toolbox 의 매개변수 연결(`setParams`)과 직렬화를 건드리지 않으려는 것이다. P0 의 계약은 관측할 수 있는 것으로 적는다: **기능은 꺼져 있고 처리 결과는 U0 와 같다. 예외는 시작 식별 줄과 설정 읽기뿐이다**(Codex 2라운드 P3).
- **패치 식별과 로드 확인**(Codex P2):
  - 패치 라이브러리는 Mapper 생성 때 한 줄을 stderr 에 찍는다: `forklift_slam_patch series=<해시> trace=… cwm=… delete=… s1=…`. 이 줄은 **패치 이미지에만** 요구한다.
  - U0 는 upstream 그대로라 이 줄이 없다. U0 는 이미지의 빌드 기록(`/opt/slam_patched/series` = none, `source_log.txt`), 라이브러리 해시, 로드 경로로 식별한다(Codex 2라운드 P2).
  - 재생 컨테이너는 `SLAM_IMAGE_KIND=patched|u0` 일 때 덧씌운 설치가 없으면 **실패**한다(조용히 apt 로 돌아가지 않음).
  - 노드가 뜬 뒤 그 프로세스의 `/proc/<pid>/maps` 에서 `libkartoSlamToolbox`·`libtoolbox_common`·ceres 솔버 플러그인의 실제 경로를 `slam_libs.txt` 에 남긴다. 기대 경로가 아니면 실패한다.
  - 이 확인은 sync·lifelong·localization 세 실행 방식 각각에서 한 번씩 smoke 로 본다.
  - 로드 경로·식별은 실행 쌍 사이의 같음 비교가 아니라 **이미지마다의 기대값 검사**다(R0 는 apt 경로, U0·P0 는 덧씌운 경로·각자의 식별).
- **지원 범위 단언**(Codex P2): 패치 기능(C-WM·C·S1)은 **새 sync mapping** 에서만 켠다.
  - lifelong·localization 노드이거나 직렬화된 그래프를 읽은(`loadSerializedPoseGraph`) Mapper 에서 기능 변수가 켜져 있으면, 시작 때 거부하고 끝낸다.
  - 이번 실험에 없는 조합(C-WM·C·S1 가운데 둘 이상 동시)도 시작 때 거부한다(Codex 2라운드 P2).
  - 이들은 자체 삭제가 있어 ID 공백이 생기고, Mapper 교체 뒤 콜백 재연결이 필요하기 때문이다.
- 재생 컨테이너(`tools/slam_cost_container.sh`)만 덧씌운 설치를 source 한다. sbatch 는 `SLAM_IMAGE` 로 이미지를, `SLAM_PATCH_ENV="K=V ..."` 로 위 변수들을 docker `-e` 로 넘긴다. 라이브 컨테이너·런처는 바꾸지 않는다.

## 3. 패치 묶음 (순서 = 구현 순서)

1. **0001 계측.**
   - keyframe 마다 한 레코드:
     - 전체 `Process`, scan 정합, `AddEdges`(그 안 `LinkNearChains`);
     - `FindNearLinkedScans`(BFS 방문 노드·edge 수);
     - `FindPossibleLoopClosure`(방문 ID 수·후보 chain 수);
     - 굵은·고운 정합 수, `CorrectPoses`(Ceres 시간).
   - `addScan` 의 `smapper_mutex_` 대기, 지도 재생성(`updateMap`) 시간도 같은 방식으로 남긴다.
   - **쓰기**(Codex P2): 레코드는 메모리 버퍼에만 쌓는다(5,000 s 재생에서 수천 개, 수 MB). 노드 종료 때, 그리고 안전을 위해 1,000 개마다 한 번만 파일에 쓴다. 쓰는 시점과 길이를 기록한다.
   - 처리 스레드의 동기 쓰기는 다음 scan 을 늦추므로 매 keyframe 에 쓰지 않는다.
   - **판정용 최종 지연은 같은 기능 설정에 TRACE 만 끈 실행으로 따로 확인한다**(계획 3.3).
2. **0002 C-WM.**
   - `FindPossibleLoopClosure` 의 시작 ID 를 `max(rStartNum, 창 시작 ID)` 로 올린다. 창 시작 ID 는 '시각이 (현재 scan 시각 − T) 이상인 첫 상태 ID' 다.
   - 지원 범위가 새 sync mapping 뿐이고 이 패치는 삭제하지 않는다. 그래서 상태 ID 가 연속이고 시각이 단조다(`shouldProcessScan` 이 역행 시각을 거름). 창 시작 ID 는 이분 탐색으로 찾는다.
   - 전제를 매번 확인한다. 이분 탐색 구간의 `GetScan` 이 nullptr 이면 **노드를 즉시 실패로 끝낸다**(abort 와 이유 한 줄). 기능을 끄고 계속하면 다른 팔의 결과가 되기 때문이다(Codex 2라운드 P2). 재생은 무응답으로 끝나 완주 실패(판정 0)가 된다.
   - BFS·`LinkNearChains`·지도 재생성·Ceres 는 옛 그래프를 그대로 본다(계획 3.6). 계측이 그 몫을 따로 보인다.
3. **0003 살아 있는 ID 순회 + 삭제 계약 + C.**
   - **순회**: `FindPossibleLoopClosure`·`FindNearChains` 를 살아 있는 ID 의 `std::map` 반복자로 바꾼다. 삭제 기능이 켜졌을 때만 이 경로를 쓰고, 끄면 원래 순회다.
   - **삭제 순서**: lifelong 순서(`RemoveNodeFromGraph` → `RemoveScan` → `dataset_->RemoveData` → vertex 해제)를 따른다.
     - `dataset_` 소유자(SlamToolbox)가 Mapper 에 삭제 콜백을 넘긴다. 콜백은 `smapper_mutex_` 를 다시 잡지 않는다(이미 그 안에서 불림, 비재귀 mutex).
     - running scans·last scan 에 든 노드는 지우지 않는다.
   - **앵커**(Codex P2):
     - `first_node_` iterator 를 쓰지 않는다. 앵커 **ID** 를 들고, 쓸 때(`Compute` 의 고정 지점)마다 `nodes_` 에서 다시 찾는다. `unordered_map` rehash 로 iterator 가 무효가 되기 때문이다.
     - 고정 노드를 지우면 살아 있는 가장 오래된 노드를 새 앵커로 정한다. 고정하는 값은 **solver 가 들고 있는 그 노드의 현재 값**(`nodes_[id]`)이다. 첫 `Compute` 전에는 `AddNode` 가 복사한 값이고, 그 뒤에는 지난 `Compute` 결과다(Codex 2라운드 P3). karto scan 자세와 다를 수 있다 — 두 값의 차이를 계측에 남긴다.
     - 첫 `Compute` 전·후 삭제 두 경우를 시험한다.
   - **C**: keyframe 처리 끝에 상태 ID 순으로 sim T 초보다 오래된 노드를 지운다.
4. **0004 S1.** 계획 3.4.
   - **가까운 영구 노드 탐색**(Codex P2): `FindNearByScan`(모든 노드 중 XY 최근접 하나)을 쓰지 않는다. **영구 노드이면서 위치 0.3 m·방향 0.35 rad 조건을 만족하는 집합에서 최근접**을 고른다. 가장 가까운 노드가 임시이거나 방향이 어긋나도, 다음으로 가까운 유효 영구 노드를 찾는다.
   - 임시 노드는 B = 10 개 뒤 3.3 의 계약으로 지운다. 지우면 끊기는 노드는 승격하고 기록한다.

## 4. 빌드와 동등성 (ws1, 총괄 신호)

- 빌드: `deploy/slurm/slam_patch_build.sbatch`(CPU 잡, `docker build` 두 번: U0·P1). 빌드 로그·이미지 digest·패치 묶음 해시·커밋 해시·`compile_commands.json` 을 남긴다. 예상 30–60 분 × 2.
- **동등성 (가)**(계획 5 장). 같은 입력(sim 1,500 s 빠른 재생)으로 R0·U0·P0 를 두 번씩 돌린다.
  - **판정은 계획대로 R0↔P0 직접 비교다**(Codex 2라운드 P2: 두 쌍이 따로 통과해도 R0↔P0 가 1e-6 을 넘을 수 있다).
  - R0↔U0(빌드 차이)와 U0↔P0(패치가 꺼진 상태의 차이)는 원인을 가르려고 함께 적는다.
- 비교 항목:
  - 처리한 scan 열;
  - 응답 자세(차이 ≤ 1e-6 m·1e-6 rad);
  - 끝 그래프의 노드·edge 수(graph dump);
  - 유효 설정;
  - `slam_libs.txt` 의 로드 경로·패치 식별 줄.
- **실패 전파**(Codex P2):
  - 동등성 실행에서는 graph dump 가 필수다. `slam_cost_dump.sh` 의 실패(serialize·덤퍼 빌드·덤프)를 컨테이너 종료 코드에 반영한다.
  - `slam_cost_summary.py --graph-dump` 는 덤프가 없으면 오류로 끝난다(지금은 조용히 생략).
- 판정:
  - R0↔P0 가 같으면 다음 단계로 간다.
  - 다르면, U0 쌍으로 원인을 가른 결과와 함께 총괄에 올린다. 빌드 차이(R0↔U0)뿐이고 U0↔P0 가 같다면, C++ 팔의 대조군을 P0 로 하는 계획 3.2 대로 진행하는 것은 계획 개정으로 정한다.
- 계측 켬(TRACE)도 P0 와 응답이 같아야 한다. 비용 차이는 적기만 한다.
- 패치 묶음이 바뀔 때마다(0002 → 0003 → 0004) **R0↔새 P0 를 직접 판정하고**, U0↔새 P0 는 원인 분리용으로 함께 본다(Codex 3라운드 P2). R0 는 같은 입력의 기존 산출물을 다시 쓴다(약 20 분).

## 5. 시험

- **gtest(이미지 안, 빌드 실패 조건)**:
  - C-WM 창 시작 ID: 경계·빈 창·전체 창. `TryCloseLoop` 처럼 반복 호출할 때 여러 chain·끝 도달 시 `rStartNum` 진행.
  - 살아 있는 ID 순회: 공백을 건너뛰고 chain 을 끊지 않음.
  - 앵커: ID 재조회, rehash 뒤에도 유효, 첫 `Compute` 전·후 삭제.
  - S1 최근접: 가장 가까운 노드가 임시인 경우, 방향이 어긋난 경우에 다음 영구 노드를 고름.
- **삭제 계약 통합 smoke**(ws1, 판정 아님, Codex P2):
  - C(T 짧게, 예 60 s)와 S1 을 켠 sim 1,500 s 재생에서 연속 삭제·running/last 보호·앵커 삭제가 일어난 뒤에도 다음 scan 처리, 지도 재생성, 직렬화(`serialize_map`)·graph dump, 정상 종료까지 지나는지 확인한다.
  - AddressSanitizer 빌드(같은 Dockerfile, `SANITIZE=address`)로 한 번 더 돌려 해제 후 접근이 없음을 본다.
- **저장소 쪽 pytest**:
  - 컨테이너·sbatch 가 패치 변수를 넘기고 라이브 경로는 손대지 않음;
  - `SLAM_IMAGE_KIND` 가 있을 때 덧씌운 설치가 없으면 실패;
  - dump 실패 전파;
  - summary 의 graph 필수;
  - 패치 묶음 해시 계산.
- **로드 경로 smoke**: sync·lifelong·localization 각각 한 번(2 절).
- **C-WM 동작 smoke**: sim 1,500 s 에서 계측의 방문 ID 가 창 안에만 있음.

## 6. 미정·위험

- R0↔U0 가 다를 수 있다(컴파일러 옵션·의존성). 그 경우는 4 절의 판정대로 총괄에 올린다.
- 앵커를 solver 값으로 고정하면, 마지막 `Compute` 뒤의 karto 자세 변화는 반영되지 않는다(차이는 계측에 남음).
- 메모리 버퍼 계측은 노드가 비정상 종료하면 마지막 1,000 개 미만을 잃는다.
- summary 는 패치 식별 줄의 설정과 sbatch 가 넘긴 `SLAM_PATCH_ENV` 가 다르면 그 실행을 거부한다(식별 줄 없는 R0·U0 는 예외).

## 7. 자체 검토

- Codex 적대 설계 검토 1라운드(v1): P1 1 — 덤퍼의 apt 헤더·패치 라이브러리 혼용. P2 7 — 실제 로드 확인, apt 플래그 재현 근거 없음(U0 추가), C-WM 전제의 적용 범위, 앵커 iterator 수명·고정값 출처, 잠금 밖 동기 쓰기의 비용 전가, S1 최근접 함수, 삭제 계약 통합 시험·실패 전파. P3 1 — tarball 과 git 절차. 모두 v2 에 반영했다(각 절의 'Codex' 표시).
- 2라운드: 1라운드 P1 해소, 새 P1 없음. P2 3 — 동등성 판정은 R0↔P0 직접 비교 유지(U0 는 원인 분리용), C-WM 전제 위반 때 기능 끔 대신 즉시 실패와 조합 거부, U0 식별은 빌드 기록·해시·경로로. P3 2 — P0 계약 표현, 첫 `Compute` 전 solver 값의 출처. 모두 반영했다(v3).
- 3라운드: 새 P1 없음. P2 1 — 패치 묶음이 바뀔 때도 R0↔P0 직접 판정을 유지. 반영했다. 4라운드에서 확인한다.
