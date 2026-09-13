# 참고: 자율 지게차 팔레트·포켓 인식 조사 (2026-09-13)

우리 기하 방식이 먼 거리·비스듬한 자세에서 반복해서 무너져, 공개 프로젝트·논문·상용 제품이 이 문제를 어떻게 다루는지 조사했다. 아래는 **원문으로 확인한 것만** 옮긴 것이고, 미확인은 그렇게 표시했다.

## 1. 포켓 검출이 원거리·경사에서 무너지는 것은 우리만의 문제가 아니다

여러 출처의 수치가 같은 경계를 가리킨다.

| 출처 | 센서 | 보고된 경계 |
|---|---|---|
| INESC TEC, ROBOT 2024 | RealSense L515 + YOLOv8 | **4 m 이상에서 포켓 검출이 불안정해지고 false negative 증가. 팔레트 검출은 계속 성공** |
| NEC, IEEE Access 2025 | 스테레오·ToF 공통 | **3,000 mm 를 넘으면 정확도가 떨어진다** |
| Zhao et al., Appl. Sci. 2022 | Kinect 2 | 2 m 미만 안정. **20° 초과 시 표준편차 급증**, 최대 오차 94.6 mm |
| SICK Pallet Pocket Detection (상용) | Visionary-T Mini ToF | 전제 조건이 **팔레트 앞 1~3 m**, 짧은 면이 카메라를 향함, **대략적 위치가 이미 알려져 있음**, 탐색 영역을 좁힐 것 |
| snenyl/realtime-pose-estimation | YOLOX + LOCO | **pallet AP 24.0 %, pallet_void(포켓) AP 0.2 %** |
| ADAPT (AIT, 실물 옥외 지게차) | 스테레오 | **15° 를 넘는 각도에서는 단일 검출이 종종 불충분** |

INESC TEC 논문의 문장 하나가 상황을 요약한다. *"most existent works focus on recognising the pallet … not on detecting the pallet pockets."*

## 2. 기하학적 한계는 계산으로 확정된다

EPAL 6 포켓은 폭 227.5 mm 에 깊이 600 mm 인 터널이다.

- 관통 가능한 최대 시선각 = `arctan(227.5 / 600)` = **20.8°**
- 30° 에서 광선의 횡방향 이동 = `600 · tan 30°` = 346 mm > 227.5 mm → **관통 불가**
- 전방 2 m·측방 1 m 면 시선 방위각만 이미 `arctan(1/2)` = **26.6°**

우리 카탈로그는 yaw ±30° 에 측방 ±1.0 m 다. **터널 너머를 보는 데 의존하는 신호는 이 범위에서 구조적으로 죽는다.** 우리가 시도한 "구멍 아래 덱"과 "구멍 너머 바닥" 이 둘 다 여기에 해당한다.

## 3. 그래서 문헌은 무엇을 하는가

**깊이를 접근면에만 쓴다.** ADAPT 원문: *"Depth information is only used on the approach side, as it is visible to the sensor."* 터널 안과 뒤는 관측 대상에서 아예 뺀다. 같은 이유로 팔레트 좌표계 **원점을 접근면 중앙**에 둔다. 재투영 오차가 그쪽에서 가장 작기 때문이다.

**포켓은 규격에서 유도한다.** 확인된 사례: ADAPT(코너·엣지 키포인트 → PnP), Lang2Lift(FoundationPose CAD → 고정 변환으로 포크 프레임), Intrinsic 특허 US10007266(블록 검출 → 블록 간격으로 포켓 폭 산출), smehta9711(CAD constrained ICP, 0.5~2 m·yaw ±30° 도킹 95 %), Molter & Fottner(수직 평면 centroid → 기지 기하 검증). ifm O3R `getPallet` 은 EPAL 을 **포켓 폭 0.23 m 고정 상수**로 둔다.

**다만 양자택일이 아니다.** NEC FFS 는 YOLO 로 hole 박스를 뽑아 **pose 추정의 입력**으로 쓴다. SICK 은 좌·우 포켓 좌표를 출력한다. 포켓 관측을 증거로 쓰되 **필수 관문으로 쓰지 않는** 구조다.

**비스듬함은 매칭 전에 없앤다.** AIST 는 광각 영상을 팔레트 전면과 일치하는 가상 평면에 재투영해 정면 뷰를 만든 뒤 템플릿 매칭한다.

**한 프레임에서 결론내지 않는다.** MIT 는 Kalman, ADAPT·Lang2Lift 는 iSAM2 factor graph 로 접근 중 여러 시점을 융합한다. ADAPT 는 팔레트 목록이 안정될 때까지 로딩존 앞에서 수 초간 정지하고 필요하면 최대 30 초 재접근한다.

**삽입 중에는 다른 센서를 쓴다.** MIT 는 포크에 2D LiDAR 를 달아 삽입 중에도 추적하고, ADAPT 는 포크 장착 레이저와 리프트 실린더 유압으로 접촉을 판정한다.

## 4. 쓸 수 있는 공개 자원

| 자원 | 내용 | 라이선스 |
|---|---|---|
| **MR6D** (TU Dortmund/Fraunhofer IML) | 92 scene, **RealSense D435i**(우리 예정 카메라와 동일), **유로팔레트 6D pose GT**(VICON), 메쉬 포함. 저자가 long-range·극단 시점·자기 가림을 겨냥해 제작 | **CC-BY-4.0** |
| **LOCO** (TUM-fml) | 5,593 장 주석, 152,421 인스턴스. RGB 만, 팔레트 bbox | **CC0-1.0** |
| **opennav_docking** | ROS 2 도킹 인프라. dock pose 토픽을 구독해 필터·컨트롤러·BT 노드를 제공. **우리는 팔레트 pose 퍼블리셔만 만들면 도킹 제어를 재사용** 가능. 팔레트 전용 플러그인은 없음 | **Apache-2.0**, 활성 |
| EMAROLab/PDT | 2D LiDAR 스캔 340 라벨 | 연구 허용, 상업은 개별 협의 |

**포켓에 3D 라벨이 달린 공개 RGB-D 데이터 세트는 찾지 못했다.** FZI/KIT 리뷰(2023)도 *"대부분의 연구가 공개하지 않은 자체 데이터를 쓴다"* 고 적는다.

## 5. 우리 설계에 대한 함의

1. **포켓 관측을 필수 관문에서 빼고 선택적 증거로 강등한다.** 현재 우리는 "막힘-구멍-막힘-구멍-막힘" 을 통과해야 검출로 친다. 문헌은 코너·엣지·hole 을 모두 증거로 넣고 일부가 없어도 pose 가 나오게 한다.
2. **터널 너머에 의존하는 신호를 쓰지 않는다.** 20.8° 한계 때문에 원리적으로 무너진다. 이미 두 번 그렇게 실패했다.
3. **전환 기준은 거리가 아니라 시선각이다.** 전방 2 m·측방 1 m 만으로도 26.6° 다. 고정 거리 스위치는 문헌이 지지하지 않는다.
4. **규격 유도가 관측 부담을 없애지는 않는다.** 팔레트 종류·진입면·대칭·카메라↔포크 변환을 정확히 식별해야 하는 부담으로 옮길 뿐이고, 규격상 포켓 위치가 실제로 비어 있다는 보장도 아니다(파손·이물).
5. **MR6D 로 하드웨어 없이 실데이터 검증이 가능하다.** 우리가 찾은 유일한 그런 자원이다.

## 출처

ADAPT [arXiv 2503.14331](https://arxiv.org/pdf/2503.14331) · Lang2Lift [arXiv 2508.15427](https://arxiv.org/abs/2508.15427) · MIT Walter et al. IROS 2010 [PDF](https://ttic.edu/ripl/assets/publications/walter10a.pdf) · INESC TEC ROBOT 2024 [PDF](https://repositorio.inesctec.pt/server/api/core/bitstreams/b44a6d58-6f78-4327-b9c6-4bee8b7c27b6/content) · NEC IEEE Access 2025 [IEEE](https://ieeexplore.ieee.org/document/10870115/) · Zhao et al. Appl. Sci. 2022 [MDPI](https://doi.org/10.3390/app122010331) · AIST Kita et al. Sensors 2026 [MDPI](https://doi.org/10.3390/s26010154) · FZI/KIT 리뷰 [arXiv 2304.06009](https://arxiv.org/pdf/2304.06009) · SICK [온라인 도움말](https://www.sick.com/media/docs/3/23/423/online_help_pallet_pocket_detection_sick_sensorapps_en_im0105423.pdf) · ifm O3R [getPallet](https://ifm3d.com/latest/PDS/GetPallet/getPallet.html) · MR6D [HF](https://huggingface.co/datasets/anas-gouda/mr6d) · LOCO [GitHub](https://github.com/tum-fml/loco) · opennav_docking [GitHub](https://github.com/open-navigation/opennav_docking) · Intrinsic 특허 [US10007266](https://patents.google.com/patent/US10007266B2/en) · snenyl [GitHub](https://github.com/snenyl/realtime-pose-estimation) · smehta9711 [GitHub](https://github.com/smehta9711/pallet-6d-pose-estimation)
