# 상용 FMS 사례 조사와 운용 콘솔 도입 후보

작성일: 2026-10-22 · 상태: **조사·권고 초안 — 무엇을 구현할지는 사용자 결정** · 관련: [운용 콘솔 설계](2026-10-13-operator-console.md), [상시 운용 콘솔 계획](../plans/2026-10-18-persistent-live-console.md), [콘솔 계약 초안 console/1](../interfaces/2026-10-13-operator-console-contract-draft.md), [8주차 피드백](../references/week8_feedback.md) 3절

## 1. 왜, 어떻게

8주차 발표에서 콘솔을 "표준 FMS(Fleet Management System)와 비교해 보라"는 피드백과 함께 참고처로 **시스콘**을 받았다. 사용자 판단(2026-10-22)으로 FMS 에는 정해진 표준이 없으므로, 표준 대조 대신 **국내외 AMR·AGV·자율 지게차 관제 제품의 개별 사례**를 보고 우리 콘솔에 들일 만한 기능을 고른다.

- 기능은 **제조사가 공개한 페이지·문서에 적힌 것만** 쓴다. 각 표의 "근거" 칸은 원문 인용, 페이지 요약, 이미지에서 보이는 것, 업계 기사 중 무엇인지 밝힌다. 자료에 없는 기능은 쓰지 않는다(없다고 단정하지도 않는다).
- 우리 구성은 **실차 1대, Jetson 이 AP 가 되어 콘솔 기기와 1:1 연결**(콘솔 설계 7.1)이다. 차체는 입고·실측했지만 Jetson 은 아직 사지 않았고, 실차와 콘솔·구동부 연동도 아직 하지 않았다. 여러 대를 관제하는 기능은 이 구성에 맞는지로 걸러 뺀다(5절).
- 조사일은 2026-10-22 이고, 웹 페이지는 그 뒤 바뀔 수 있다.

## 2. 제품별 확인한 기능

### 2.1 시스콘로보틱스 SARICS (국내, 8주차 피드백의 참고처)

출처 A: [로봇기술 > 관제 시스템](http://sysconrobotics.com/tech_2.html)(같은 내용: [Notion](https://sysconresearch.notion.site/ACS-79151b1105e04f1990a5870f9da55722)) · 이미지 B: [관제 화면](http://sysconrobotics.com/img/t2/sub_3.png) · 이미지 C: [태블릿 화면](http://sysconrobotics.com/img/t2/sub_4.png)

| 기능 | 근거 |
|---|---|
| 작업 설계·미션 부여·명령, 현장 맵핑과 맵 관리 | A 원문: "AMR에 할당되는 작업을 사용자 친화적으로 직접 설계", "미션 부여, 명령, 현장 맵핑과 맵등을 자유롭게 관리" |
| 사용자 그룹 등록, 그룹별 권한 설정 | A 원문 기능 목록 |
| 로봇 상태 정보, 맵상 이동 경로, 외부 장비 상태 | A 원문("실시간 로봇 확인서비스") |
| UX 기반의 간편한 명령어 생성, 모바일·웹 접근 | A 원문("AMR 제어") |
| 맵상 오브젝트 생성·제어, 위험지역 설정/이동경로 제한 | A 원문("맵 컨트롤") |
| 현장 설비 인터페이스에서 데이터 수집 | A 원문 |
| 상단 탭 HOME · MISSION STATUS · SCHEDULE · LOGS, 로봇별 상태 목록(배터리 %), MISSION · ALARM 패널, 점유 격자 지도, 라인별 ON/CALL 버튼 | B 이미지에서 보이는 것 |
| 로봇 하나의 충전 상태 · X/Y/Theta · START/END, Home · Monitoring · Edit · Manual 메뉴 | C 이미지에서 보이는 것 |

확인하지 못한 것: 충전·엘리베이터 연동, 통계 화면 내용. 같은 페이지의 어두운 함대 화면 이미지(`fleet.png`, "LOGI ARIZONA FACTORY")는 시스콘 화면인지 알 수 없어 쓰지 않았다.

### 2.2 OTTO Fleet Manager (OTTO Motors, Rockwell Automation)

출처 A: [제품 페이지](https://ottomotors.com/fleet-manager) · B: [브로슈어 OTTO-FM002A-EN-AUG2024](https://www.rockwellautomation.com/content/dam/rockwell-automation/documents/pdf/events/automate/otto-fleet-manager-brochure.pdf) · C: [2022-06-10 릴리스 기사(Robotics 24/7)](https://www.robotics247.com/article/otto_motors_releases_software_to_provide_new_visibility_into_mobile_robot_fleet_performance)

| 기능 | 근거 |
|---|---|
| 한 대부터 쓴다고 밝힘 | B 원문: "Whether you have one AMR or 100" |
| 교통 규칙(속도 제한, 진입 금지 구역) | B 원문: "specific traffic rules like speed limits and exclusion zones" |
| 배터리·대기 시간·가동률로 작업에 로봇 자동 배정 | A·B |
| 작업 사이 기회 충전 | A·B("autonomously charging between jobs") |
| 로봇끼리 정보를 주고받아 막힘 예측·회피 | A·B |
| 일정 또는 태블릿·컴퓨터·버튼으로 작업 지시 | B 원문: "Set a predefined schedule or dispatch tasks directly from your tablet, computer or button" |
| MES·ERP·WMS(Open API), PLC(OPC-UA), VDA 5050 | A·B |
| 미션·작업·로봇별 실시간 생산 자료, 대시보드(차량 상태·진행 중 작업·과거 추세), 병목 진단 | B |
| 스냅숏 재생(문제 시점의 로봇 동작·상태를 영상으로) | C: "without calling customer support" |
| 예외 이력 타임라인 | C: "a historical timeline of exceptions" |
| 느리거나 실패가 잦은 작업·장소·로봇·과업 찾기 | C: "find jobs, places, robots, or tasks that are slow or failure-prone" |
| 큰 TV 용 함대 대시보드 | C: "designed for a large-screen TV" |
| 도입 전 시뮬레이션, 작업 장소 근처 주차 최적화 | A |

### 2.3 MiR — 로봇 화면, MiR Fleet, MiR Insights (Mobile Industrial Robots)

출처 A: [MiR250 Shelf Carrier User Guide v1.1(2020-07, SW 2.9)](https://download.astor.com.pl/dokumentacja/Mobile%20Industrial%20Robots/Opcje/MiR%20Shelf/Podr%C4%99czniki/mir250-shelf-carrier-user-guide-11-en.pdf)(대리점 배포본; 괄호 안은 절 번호) · B: [위치·미션 대시보드 예(Neff Automation 블로그)](https://neffautomation.com/blog/mir-amrs-positions-and-missions) · C: [MiR Insights 기사(Robotics 24/7, 2023-03-21)](http://www.robotics247.com/article/mir_insights_announces_new_cloud_based_software_for_optimizing_robot_fleets)

MiR 는 로봇마다 **로봇 화면(robot interface)**이 있고 함대 관제(MiR Fleet)는 따로다. 로봇 한 대의 화면이라 우리 구성과 가장 가깝다.

| 기능 | 근거 |
|---|---|
| 위젯으로 짜는 대시보드(미션 버튼, 지도, 미션 대기열 등), 사용자마다 따로 | A 8.11 원문: "A dashboard is made up of a number of widgets, each representing a feature in the system, for example a particular mission, the map the robot is operating on, or the current mission queue." |
| 미션 버튼 하나로 미리 정한 미션 실행(예: Pick Up at A, Drop at B) | B |
| 미션 대기열, 대기열에 넣을 때마다 매개변수 고르기 | A 8.6: "choose the value of a parameter each time the mission is added to the mission queue" |
| 사용자 그룹(기본 User · Administrator · Distributor), 화면 부분별 권한 | A 8.10: "specific user groups with specific access to different parts of the robot interface" |
| 하드웨어 상태 점검 화면 | A 5.7: "Go to Monitoring > Hardware health. Check that all elements on the page have the OK status and that they have green dots" |
| 지도 구역: Directional, Preferred, Unpreferred, Forbidden, Critical, Speed, Sound and light, Planner, I/O module, Limit-robots(MiR Fleet 전용) | A 8.4 |
| Forbidden 구역의 동작 | A 8.4: 로봇은 들어가지 않는다. 예외 셋 — 상대 이동(relative move), Forbidden 위에 Critical 구역이 겹친 곳, 수동 조종(이때 계획기는 Critical 구역처럼 통과 계획을 세울 수 있다) |
| Speed 구역 | A 8.4: 구역 안에서 속도를 바꾸되 시스템 최대 속도는 넘지 않음 |
| Sound and light 구역 | A 8.4: "blinks and/or makes a sound to catch attention" |
| Planner 구역(설정 여섯 가지) | A 8.4: No localization(레이저 스캐너 자료를 무시하고 엔코더만으로 위치추정), Look-ahead, Path timeout, Path deviation, Ignore obstacles, Obstacle history clearing |
| 수동 조종 | A 5.5: 로봇의 운전 모드 키를 Manual 로 돌린 뒤 화면 조이스틱으로. 수동 조종에서는 구역을 무시(A 8.4) |
| 주행 거리·완료 미션·가동률 KPI, 시간·장소별 히트맵(Wi-Fi 약한 곳·막힘 예상 지점), 클라우드 | C: "areas with poor Wi-Fi coverage and potential robot traffic bottlenecks" |

### 2.4 BlueBotics ANT server · ANT monitor

출처 A: [ANT server 제품 페이지](https://bluebotics.com/autonomous-navigation-technology/ant-server-fleet-software)(페이지 요약)

| 기능 | 근거 |
|---|---|
| 함대 실시간 개요 | A 원문(ANT monitor): "provides AGV operators with a complete overview of their fleet in real-time" |
| 단일 차량: 미션 관리, 미션 시뮬레이션, 배터리 충전 관리 | A |
| 여러 대: 교차로·문·엘리베이터 교통 관제 | A |
| 현장 시스템·문·엘리베이터와 API 연동 | A |

경보·통계·권한·감시 화면의 구성은 페이지에 없다.

### 2.5 Seegrid Fleet Central (자율 팔레트 차량)

출처 A: [Fleet Central 페이지](https://seegrid.com/fleet-central/)(페이지 요약)

| 기능 | 근거 |
|---|---|
| 코드 없이 작업(Job) 편집 — 장소와 과업의 순서 | A 원문: "Create dynamic, adjustable AMR Jobs—sequences of locations and tasks" |
| 배터리 확인·작업 배정·완료 확인을 실시간으로 | A 원문: "check battery status, assign jobs, and view task completions in real-time" |
| 작업 흐름 가동률 분석 | A |
| 혼잡 통로·사람 운전 차량과 섞인 교통 관리 | A |
| 충전소 자동 배차, PLC·WMS·ERP·MES·바코드 연동, VDA 5050 | A |

### 2.6 AGILOX (자율 지게차·팔레트 AMR)

출처 A: [AGILOX 홈](https://www.agilox.net/en/) · B: [All data at a glance(블로그, 2021-11-03)](https://www.agilox.net/en/blog/all-data-at-a-glance/)

| 기능 | 근거 |
|---|---|
| 앱 설치 없이 차량마다 브라우저로 접속 | A 원문: "it is not necessary to install an application", "Using a conventional web browser, you can smartly and simply connect to any robot at any time" |
| 중앙 관제 컴퓨터 없이 차량끼리 위치·상태를 주고받아 주문 분배 | A 원문: "a system that makes a master computer superfluous", 차량이 "in direct communication" 하며 "distribute orders intelligently", 위치·상태를 "several times a second" 교환 |
| 분석: 차트를 대시보드에 고정·크기 조절, 생산성 분석·오류 시각화 | B |
| 근무 시간(교대) 설정 | B |
| 차량 비교 방식 Combined · Single(위젯마다 한 대) · Sum | B |

### 2.7 VisionNav RCS (자율 지게차, 트레일러 상하차)

출처: 업계 기사만 — [Robotics 24/7](https://www.robotics247.com/article/visionnav_introduces_automated_forklift_for_cargo_handling), [New Equipment Digest](https://newequipment.com/product-news/material-handling-packaging/product/21284795/visionnav-robotics-vnst20-pro-autonomous-forklift-for-loading-unloading-trailer-trucks)

| 기능 | 근거 |
|---|---|
| RCS 가 지도·3D 인식·위치추정·경로로 상하차와 배송 경로를 계획 | 기사 |
| 트럭이 도킹하면 별도 Bright Eyes 시스템이 트럭 위치·자세를 RCS 에 넘기고 RCS 가 지게차를 배차 | 기사 |
| VNST20 PRO 는 수동 모드로 쉽게 바꿀 수 있음 | 기사 |

관제 화면의 공개 설명은 찾지 못했다.

## 3. 공통 기능과 지금 콘솔

"확인한 제품"은 2절 표에 근거가 있는 것만 적었다.

| 기능 | 확인한 제품 | 지금 콘솔 |
|---|---|---|
| 지도 위 로봇 표시(경로 포함 여부는 제품마다) | SARICS(맵상 이동 경로·지도 화면), MiR(대시보드 지도 위젯 — 경로 표시는 자료로 확인 못 함) | **있음** — 점유 격자·LiDAR·계획 경로·예상 궤적(콘솔 설계 8절) |
| 웹·모바일·버튼 등에서 작업 지시 | SARICS(모바일·웹), OTTO(태블릿·컴퓨터·버튼·일정), AGILOX(브라우저), MiR(로봇 화면의 미션 버튼) | **있음** — 브라우저 3단계 작업 지정, 지도에서 팔레트 눌러 지정 |
| 미션 진행 상태 | SARICS(MISSION STATUS), Seegrid(완료 확인), MiR(대기열) | **있음** — 아홉 단계 진행 띠, 결과 기록부(상시 운용 콘솔 D13) |
| 수동 조종 | MiR(조이스틱+운전 모드 키), SARICS(Manual 메뉴), VisionNav(수동 모드) | **있음** — 조이스틱·게임패드·키보드, 예상 궤적, 영상 조종 |
| 이벤트·경보·로그 | SARICS(ALARM·LOGS), OTTO(예외 이력) | **일부** — 이벤트 기록 화면, 결과 기록부. 경보 원장(식별자·처음 난 시각·심각도·확인)은 없음 |
| 장치 상태 점검 화면 | MiR(Hardware health) | **없음** — 영상 나이·'위치 n s 전' 같은 신선도가 화면 곳곳에 흩어져 있고, 실행기는 상태의 `freshness` 를 아직 채우지 않는다 |
| **미리 정한 미션 버튼·미션 대기열** | MiR(미션 버튼·대기열), OTTO(일정·버튼), SARICS(라인 CALL 버튼, 이미지) | **없음** — 미션은 한 번에 하나, 진행 중 새 제출은 `mission_active` 로 거절 |
| **금지·감속 구역** | OTTO(속도 제한·진입 금지 구역), MiR(Forbidden·Speed 등), SARICS(위험지역/이동경로 제한) | **없음** — 계획기는 지도 장애물만 본다 |
| 실적 요약(KPI) | OTTO, MiR Insights, AGILOX, Seegrid | **없음** — 결과 기록부는 최근 100건 목록(결과·사유·시작/끝 시각)뿐 |
| 느리거나 실패가 잦은 장소 찾기 | OTTO(Fleet Analytics), MiR Insights(막힘 예상 지점 히트맵) | **없음** |
| 기록 재생 | OTTO(스냅숏 재생) | **일부** — bag 재생(`replay_demo.sbatch`)은 있으나 콘솔 안의 재생은 없음 |
| Wi-Fi 신호 지도 | MiR Insights | **없음** — 왕복 지연만 표시 |
| 배터리·충전 | OTTO, Seegrid, BlueBotics, SARICS(배터리 %, 이미지) | **없음** — 측정 장치가 없어 "측정 장치 없음"으로 표시(콘솔 설계 8절) |
| 사용자·권한 | SARICS, MiR | **없음** — 인증은 나중 단계(사용자 결정 2026-10-13, 콘솔 설계 10절) |
| 외부 설비 연동·호출 | SARICS(외부 장비 상태), OTTO(PLC·버튼), BlueBotics(문·엘리베이터) | **없음** |

## 4. 도입 후보와 권고 순위

평가 기준: **가치**(시연·실차 운용에 무엇이 좋아지나), **구현 비용**(콘솔만 / 계약 추가 / 차량 쪽(실행기·미션·계획기·중재기) 변경), **구성 적합성**(실차 1대·Jetson AP 1:1 에서 의미가 있나). 후보마다 **최소안**과 **확장안**을 나눴다 — 최소안은 지금 오는 자료로 할 수 있는 것이다.

| 순위 | 후보 | 근거 사례 | 가치 | 구현 비용 | 구성 적합성 |
|---|---|---|---|---|---|
| 1 | **저장된 운반 버튼 + 미션 대기열** | MiR 미션 버튼·대기열, OTTO 일정·버튼, SARICS CALL | 높음 — 8주차가 밝힌 다음 목표 "연속 이송"을 콘솔에서 바로 지시. 시연 조작이 짧아짐 | **최소안 중간**: 버튼은 작업 지정 창의 저장본(콘솔만). 대기열은 콘솔이 들고 있다가 정지 확인·최신 `targets_revision` 으로 하나씩 제출 — 이미 `tools/live_console_probe.py` 의 cycle 시나리오가 같은 방식이라 차량 쪽 변경 없이 된다. 다만 실패·취소·복구 뒤 계속할지, 연결·조종 권한(lease)을 잃으면 멈출지, 재접속 뒤 이어 갈지 정책을 정하고 시험해야 한다. **확장안**: 차량이 대기열을 들고 있게(계약·실행기 변경) | 맞음 — 한 대가 차례로 수행 |
| 2 | **연결·장치 상태 한 화면 + 경보 목록** | MiR Hardware health, SARICS ALARM, OTTO 예외 이력 | 높음 — 실차를 처음 붙일 때 무엇이 안 오는지 바로 보임 | **최소안 낮음–중간**: 지금 오는 영상 나이·위치 나이·세션 상태·정지 래치·허가 사유를 콘솔이 한 패널에 모아 OK/경고로(콘솔만). 경보 시각은 화면이 처음 본 시각이라 실제 발생 시각과 다를 수 있음을 표시. **확장안 중간–높음**: 센서별 신선도(`freshness`, 실행기가 아직 안 채움)와 경보 원장(식별자·발생 시각·심각도·확인)을 차량이 내도록 계약·실행기 변경, MCU 결함 비트([mcu-link/1](../interfaces/2026-10-22-mcu-link-contract.md) `STATE.fault_flags`)는 보드 연결 뒤 | 맞음 |
| 3 | **실적 요약** | OTTO 분석, MiR Insights KPI, AGILOX(Single 모드·근무 시간), Seegrid | 중간 — 연속 운용 결과를 발표·보고에 숫자로 | **최소안 낮음**: 결과 기록부(최근 100건)의 완료·실패·취소 수, 사유별 수, 미션 소요 시간(시작–끝)을 콘솔이 집계. 세션이 바뀌면 기록부가 비고, 100건이 넘으면 앞부분이 빠져 늦게 붙은 화면은 세션 전체 수를 복원하지 못한다는 한계를 같이 보인다. **확장안 중간–높음**: 세션·하루 누적 집계, 실패한 단계·단계별 소요·주행 거리 — 결과 메시지에 없는 필드라 계약 추가와 미션 기록(`records.json` 등)의 가공·조회가 필요 | 맞음(차량 간 비교는 뺌) |
| 4 | **금지·감속 구역** | OTTO 교통 규칙, MiR Forbidden·Speed, SARICS 위험지역/이동경로 제한 | 높음 — 시연 장소(2공학관 지하 등)의 기둥·사람 통로·경사를 지도 장애물 없이 막을 수 있음 | **높음**: 계획기·주행 허가·SLAM 지도 묶음까지 차량 쪽 변경과 계약 추가(메인 세션 담당 영역), 콘솔 그리기는 그다음. 수동 조종에서 구역을 어떻게 다룰지(MiR 는 무시)도 정해야 함 | 맞음 |
| 5 | **링크 품질 지도** | MiR Insights Wi-Fi 히트맵 | 중간 — 8주차 피드백의 "지하 Wi-Fi" 걱정에 답함 | 중간 — Jetson AP 가 단말 신호 세기를 내줘야 함(Jetson 미구매라 지금은 자료원 없음). **측정 조건**: AP 가 차량에 실려 함께 움직이므로 신호 세기는 차량 위치만이 아니라 콘솔 위치에도 달려 있다. 장소 자체의 음영 지도로 읽으려면 콘솔을 고정된 자리에 두고 잰 경로별 지도로 한정해야 한다 | 조건부로 맞음 |
| 6 | **오래 멈추거나 실패가 잦은 자리 지도** | OTTO "slow or failure-prone ... places", MiR Insights 막힘 예상 지점 | 중간 — 한 대도 같은 자리에서 반복해 막히거나 오래 기다리면 장소 문제를 찾을 수 있음 | 중간 — 멈춤·실패 시각과 자세를 모아야 함(결과의 사유·시각만으로는 위치가 없음). 기록 가공 필요 | 맞음(차량 간 혼잡이 아니라 한 대의 정체) |
| 7 | **콘솔 안 기록 재생** | OTTO 스냅숏 재생 | 중간 — 실패 원인을 화면에서 바로 확인 | 중간–높음 — 상태 스트림을 남기고 다시 틀 장치가 필요. 지금은 라이브 로그·bag 이 따로 있음 | 맞음 |

권고: **1·2·3 의 최소안을 먼저**(차량 쪽 변경 없이 콘솔에서 시작할 수 있고, 다음 목표인 연속 이송과 실차 첫 연결에 바로 쓰인다). 2·3 의 확장안은 실차 연결과 함께 계약 추가로, 4 는 가치가 크지만 메인 세션의 계획기·허가 변경이 앞서야 해 별도 결정으로, 5 는 Jetson 이 생긴 뒤 측정 조건과 함께, 6·7 은 그다음.

## 5. 우리 구성에 맞지 않아 뺀 것

| 기능 | 사례 | 뺀 이유 |
|---|---|---|
| 작업에 맞는 차량 자동 배정 | OTTO, Seegrid | 차량이 한 대다. 여러 대 관제는 별도 프로젝트로 나눴다(사용자 결정 2026-10-13) |
| 차량 간 교통 관제·우선권, Limit-robots 구역 | OTTO, BlueBotics, MiR Fleet | 서로 피할 다른 차량이 없다(한 대의 정체 지도는 4절 6번으로 따로 평가) |
| 큰 화면용 함대 대시보드 | OTTO Fleet Dashboard | 여러 대를 한눈에 보는 용도다 |
| ERP·WMS·MES 연동, VDA 5050 | OTTO, Seegrid, BlueBotics | 연동할 상위 시스템이 없다. 작업 단위 API(console/1)는 이미 있어 나중에 중계하면 된다(8주차 덱 3쪽) |
| 자동 기회 충전·충전소 배차 | OTTO, Seegrid, BlueBotics | 배터리 측정 장치와 충전소가 없다. 8주차 피드백의 충전 질문은 하드웨어(충전기·충전 중 운용) 정리가 먼저다. 배터리 전압을 MCU 가 읽게 되면([핀맵 검토](2026-10-22-h743-pinmap-draft-review.md) C절, 비어 있는 ADC 핀) 상태 표시부터 넣는다 |
| 사용자 그룹·권한 | SARICS, MiR | 인증·보안은 나중 단계로 사용자가 정했다(2026-10-13) |
| 외부 설비(문·엘리베이터·라인 호출) 연동 | SARICS, OTTO, BlueBotics | 시연 환경에 연동할 설비가 없다 |
| 미션 동작을 블록으로 짜는 편집기 | MiR(미션·동작 그룹), Seegrid(Job 편집), SARICS(명령어 생성) | 우리 작업은 팔레트 운반 하나이고 단계는 미션 실행부가 정한다. 4절 1번의 저장된 운반 버튼으로 충분하다 |

## 6. 이 문서가 하지 않은 것

제품을 직접 써 보거나 데모를 받지 않았다. 각 기능의 실제 동작·화면은 공개 자료의 문장과 이미지 범위에서만 적었다. 국내 지게차 관제 제품은 시스콘 외에 공개 설명을 찾지 못했다(트위니·클로봇 등 업체명 검색으로 관제 기능 설명이 나오지 않음).

## 7. 검토 기록

- Codex 1회차(P1 2·P2 4·P3 2, `week8_feedback.md` 변경은 지적 없음): MiR Forbidden 예외(상대 이동·Critical 중첩)와 Planner "No localization"(엔코더만으로 위치추정) 바로잡음 / 비교표에서 자료로 확인 못 한 칸(OTTO·BlueBotics 지도 표시, VisionNav 작업 지시, "MiR Insights 는 읽기만") 제거 / 경보·장치 상태와 실적 요약을 최소안·확장안으로 나눔(실행기가 `freshness` 를 채우지 않음, 결과 메시지에 단계·거리 필드 없음, 최근 100건·세션마다 비움) / 링크 품질 지도에 측정 조건(AP 가 차량과 함께 움직임) / 한 대의 정체 지도를 후보 6으로 따로 평가 / "실차 미구매"를 "차체 입고·Jetson 미구매·연동 미완료"로 / 제품별 표 형식.
- Codex 2회차: 수렴(새 P1/P2 없음). P3 두 곳(MiR 은 지도 위젯까지만, AGILOX 주문 분배의 원문 근거) 반영.
