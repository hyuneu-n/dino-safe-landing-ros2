# 인수인계 문서 (다른 AI 세션이 이어받을 때 이것만 읽으면 됨)

> **사진 기반 캠퍼스와 영상:** 사용자 제공 서경대 사진 3장에 맞춰 `worlds/metropolis_campus.py`를 추가했다. 기존 캠퍼스 5동을 원통형 유리 타워·아치 강의동·노란 상부 외벽·붉은 본관으로 교체, 줄무늬 광장과 분수·계단 정원 구성. 건물/목적지 개수와 광장 좌표는 유지한다. `eval/record_city_tour.sh`는 별도 Gazebo의 촬영 카메라로 MP4를 만드는 도구이며 자율비행 평가가 아니다. 기술 상태와 인터랙션 계획은 [PROJECT_STATUS.md](PROJECT_STATUS.md) 참조.


> **최신 — 생활 랜드마크 추가:** `worlds/metropolis_landmarks.py`에 SkyDrop 물류창고, 맥도날드, 써브웨이, 서경대 모티브의 창작 캠퍼스를 추가했다. 건물 148동(seed 0), 목적지 22곳. 드론 생성 위치·manifest 시작점은 창고 `(-210,-88)`, 기본 경유점은 도심 대로를 거쳐 East Gate로 연결한다.
> 캠퍼스 z=12를 포함해 높이 처리가 필요한 목적지는 4곳이다. 핵심 비전·미션 알고리즘과 대시보드는 이번에 변경하지 않았다. 신규 목적지와 검증 범위는 [CITY_ENVIRONMENT.md](CITY_ENVIRONMENT.md) 참조.


> **2026-09-14 지역 확장:** `worlds/metropolis_region.py` 추가. 강 양안·높이 24m 고가도로·34m 교량·회전 램프·산동네로 약 2.5×1.9km 범위 확장.
> 시드 0 건물 141동, 목적지 18곳. 고도 34/36/96m 목적지 3곳은 환경·센서 검증용이며 **기존 제어기의 고도 처리는 아직 미수정**이다.
> 도로·지형은 동일 메시로 렌더링/충돌 처리, 산길 5곳의 depth 검사 추가. 자세한 최신 내용은 `CITY_ENVIRONMENT.md` 참조.


> **2026-09-14 업데이트 — 다음 세션은 [CITY_ENVIRONMENT.md](CITY_ENVIRONMENT.md)를 먼저 확인.**
> 사용자가 Gazebo Classic에서 환경 제작을 자율 진행하도록 요청했다. `worlds/generate_metropolis.py`를 새로 작성해
> 도심→공장·물류→주거 타운을 연결했고, `run_city_demo.sh`의 기본 생성기를 교체했다.
> 지상 목적지 12곳 중 4곳은 지정 좌표가 점유된 시나리오다. 기존 생성기와 핵심 비전·미션 알고리즘은 보존했다.
> 환경만 보려면 `bash run_city_demo.sh 0 world`. 자동 배치 검사와 Gazebo RGB/depth 검증 도구를 추가했다.
> **대시보드·반복 배송·사용자 장애물 조작은 아직 미구현. 새 도시의 종단 간 착륙 성공률도 아직 측정하지 않았다.**
> 아래 본문은 9월 11일 기준 이력이며, 새 도시의 구조·실행법은 위 문서를 따른다.


> 작성: 2026-09-11. 이 문서는 "지금까지 뭘 했고, 뭐가 남았고, 다음에 뭘 해야 하는지"를
> 새 세션이 이 대화 기록 없이도 파악할 수 있게 만든 것이다. `README.md`는 사용자용
> 소개서, `CONTEXT.md`는 설계 결정 이력, **이 문서는 작업 인수인계용**.

## 1. 프로젝트 한 줄 정의

배송 드론이 목적지 근처에 도착한 뒤, **RGB+depth 카메라만으로 현재 장면을 분석해**
차·사람·장애물을 피한 안전한 빈 공간을 찾아 정밀 착륙하는 시스템. Gazebo Classic 11 +
ROS 2 Humble. 졸업작품(캡스톤), 마감은 **2026년 12월 말**, 그 전까지 **격주로 중간 발표**.
시간 여유는 있는 편(4개월+) — 급하게 땜질할 필요 없음.

핵심 기여: DINOv2 patch feature(semantic) + depth(geometric) **융합** — 둘 다 따로는
공개된 게 있지만 융합해서 공개 코드로 낸 건 2026-08 조사 기준 없었음.

## 2. 저장소 / 환경

- GitHub: `git@github.com:hyuneu-n/dino-safe-landing-ros2.git` (브랜치 `main`)
- 실제 작업 경로: WSL2 Ubuntu-22.04, `~/safe_landing/` (Windows에서
  `\\wsl.localhost\Ubuntu-22.04\home\hyuneun\safe_landing\`)
- 스택: ROS 2 Humble + Gazebo Classic 11 (Ogre 1.9 렌더러), Python 3.10, PyTorch(DINOv2),
  `venv_ros`에 rclpy/PIL 등
- `git log`가 짧아 보이는 이유: 히스토리를 한 번 squash해서 push함(`5bf7f43 Initial commit`).
  이 세션(개발 세션 기준)에서 한 상세 작업 이력은 여기(이 문서)와 각 파일 상단 docstring의
  날짜 붙은 주석에만 남아있음 — git blame으로 "언제 왜 바뀌었는지" 추적이 잘 안 되니,
  **파일 안의 docstring/인라인 주석을 먼저 읽을 것** (특히 `worlds/generate_city.py`,
  `worlds/generate_world.py`).

## 3. 빠른 실행

```bash
# 격자도시 + 맨해튼 + 마을 + 배송 목적지 9곳 전체 데모
cd ~/safe_landing
bash run_city_demo.sh 0          # seed=0, RViz 포함
bash run_city_demo.sh 0 norviz   # RViz 없이

# 월드만 재생성해서 확인하고 싶을 때
source /opt/ros/humble/setup.bash
python3 worlds/generate_city.py --seed 0 --mesh --out /tmp/city.world
# --no-manhattan / --no-village 로 특정 구역 생략 가능 (빠른 반복용)
```

Gazebo/gzserver를 스크립트로 죽일 때 **`pkill -f gazebo`처럼 "gazebo"라는 문자열이 들어간
패턴을 쓰면 안 됨** — bash -lc로 감싼 명령어 자체의 텍스트에 "gazebo"가 들어있으면
자기 자신을 죽여버림(이 세션에서 몇 번 당함). `pkill -x gzserver`/`pkill -x gazebo`처럼
정확한 프로세스명 매치(`-x`)를 쓸 것.

## 4. 아키텍처

```
Gazebo Classic (절차생성 월드) ── 4개 카메라 + odom ──▶ landing_detector.py
                                                            │  DINOv2 패치특징 → K-means 표면군집
                                                            │  depth → 지면거리/장애물 마스크
                                                            ▼
                                                     /landing/target (안전 착륙점)
                                                            ▼
                                              mission_controller.py (상태머신)
                                        IDLE→TAKEOFF→CRUISE→ARRIVE→SCAN→DESCEND→LANDED
                                        (CRUISE 중 전방 depth로 반응형 협곡 회피,
                                         /clicked_point로 목적지 실시간 변경 가능)
```

핵심 알고리즘 파일: `landing_detector.py`(331줄), `dino_seg.py`(단독 실행 가능),
`mission_controller.py`(379줄). **이번 세션엔 이 셋을 거의 안 건드림** — 전부 월드
생성/시각화 쪽 작업이었음.

## 5. 지금까지 완료된 것 (CONTEXT.md 작업번호 기준)

| # | 작업 | 상태 |
|---|---|---|
| 0 | git 백업 + GitHub push | ✅ (`hyuneu-n/dino-safe-landing-ros2`) |
| 1 | 절차생성 월드 스크립트 | ✅ `worlds/generate_world.py`(단일회랑), `worlds/generate_city.py`(격자대도시) |
| 2 | 클릭 → 목적지 지정 | ✅ RViz "Publish Point" → `/clicked_point` 구독, 라이브 검증됨 |
| 3 | baseline 2개 + 3열 비교 | ✅ `eval/` — PX4식 depth-only 재구현 + OpenLander(ONNX) |
| 4 | 랜덤시드 성공률 실험 | ✅ n=24: 우리 92%(MOD 3.69m) / depth-only 71%(2.22m) / OpenLander 46%(제시율83%) |
| 5 | 통합 대시보드 / RViz 정리 | ⏳ **미착수** — 아래 6절 참고 |
| 6 | Docker + README | ✅ 단, `docker build` 자체 실행 검증은 아직 안 됨(사용자 PC에서 1회 필요) |
| 7 | (여유되면) 장애물 배치 인터랙션 | 미착수 |

academic rigor(ablation study, 학습형 fusion)는 **사용자가 명시적으로 후순위 지정**
("작품이 더 우선") — 나중에 시간 되면.

### 5-1. 이번 세션(2026-09-04 ~ 09-11)에 한 것 — 절차생성 월드 확장사

시간 순서대로:

1. **도로 타일 틈 버그 수정** — `road_tile()`이 등방 스케일만 지원해서 구간 길이가
   타일폭의 배수가 아니면 틈이 생기던 버그. `cross_w`/`along_len` 분리 + 정확히
   나눠떨어지는 타일링으로 해결.
2. **격자도시 확장** — 6×5 → 11×7 노드, 여러 스트리트/애비뉴에 걸친 우회 강제
   (`BLOCKED_STREET_GAPS`/`BLOCKED_AVENUE_GAPS`), 구역별 건물 밀도 차등(도심 밀집/교외 성김).
3. **맨해튼풍 스카이스크래퍼 구역** (`build_manhattan_xml`) — 기존 도시 동쪽 100m,
   반듯한 격자, SKY_MESHES(스카이스크래퍼 5종)만 사용(산업지구 탱크/굴뚝 없음).
   ⚠️ 첫 시도(간격 32m)는 건물 폭이 원본 메시 비율 때문에 40~66m까지 뻥튀기돼서
   서로 뚫고 들어가는 참사 — 간격 70m로 넓혀 해결.
4. **비격자 마을** (`build_village_xml`) — 기존 도시 서쪽, 랜덤워크 폴리라인 길
   (`generate_winding_path`) + 골목 2개, 낮은 집(BUILDING_MESHES)/나무 비정렬 배치.
5. **도시-마을 연결로 + 집 밀도/색상** — `generate_connector_path`(steering 방식,
   도착 보장)로 도시 교차로↔마을 연결. 집 색상: SDF `<material>`을 mesh visual에
   얹으면 텍스처가 단색으로 덮인다는 걸 실측 확인 후 `VILLAGE_HOUSE_COLORS` 팔레트
   (테라코타/크림/세이지그린/슬레이트블루 등, 원색 아님)로 집집마다 다르게 칠함.
6. **Gazebo Sim(Fortress) 전환 타당성 스파이크** — `docs/fortress_spike_findings.md`에
   상세. **결론: 이 WSL 환경에선 불가.** 카메라 센서 초기화 시 100% 크래시
   (`Ogre::UnimplementedException: GL3PlusTextureGpu::copyTo` — Ogre-Next의 GL3Plus
   백엔드가 WSLg의 Mesa/D3D12 변환 GL 스택 위에서 밉맵 텍스처 copy를 구현 안 해놓음).
   Vulkan 백엔드도 미설치. **드라이버/렌더러 레벨 문제라 코드로 못 고침.**
7. **README에 스크린샷 반영** — `docs/images/`에 7장 커밋(격자도시/맨해튼/마을/연결로/
   DINO PCA/착륙오버레이).
8. **배송 목적지 9곳 + 마을 중심 광장** — "목적지가 다 똑같다, 마을이 부실하다" 피드백.
   `landing_spots_static()` + `build_village_xml()`이 만드는 마을 안 2곳 = 총 9곳:
   helipad(기본 TARGET) / plaza(도심광장) / park(근린공원) / parking(마트주차장,
   차량그리드+빈스톨) / vacant×2(산업단지 공터, 마을 어귀 공터) / street(맨해튼 협곡) /
   rooftop(옥상패드, 18m) / green(마을회관 앞마당). 각 지점 이름표 깃발 마커(kind별
   색상) + 외곽 목적지는 진입로 자동 연결. `waypoints.json`에 `landing_spots[]`
   (x/y/label/kind/note)로 노출. 마을엔 `build_village_square()`(우물+마을회관 첨탑+
   벤치+집 링)로 "중심이 있는 정착지" 느낌 추가.
   ⚠️ 이 과정에서 잡은 버그: `generate_world.flat_patch()`가 `<ambient>`만 쓰고
   `<diffuse>`가 없어서 밝은 색(회색/베이지 계열) 바닥 패치가 햇빛에 하얗게 날아감 —
   `ground_patch()`(box 기반, ambient+diffuse 둘 다)로 교체해서 해결.
   ⚠️ 옥상 패드(rooftop)는 카메라 앵글 문제로 스크린샷 검증을 못 했음 — 코드는
   다른 피처와 동일 패턴이라 문제 없을 가능성 높지만, **다음 세션이 확인해볼 것**
   (`(537, 105)` 근처, Manhattan 교차로).

모든 검증은 "코드만 보고 넘어가지 않고 실제 Gazebo 띄워서 스크린샷/로그로 확인"
방식으로 했음 — `eval/capture_frame.py --cam down/chase/iso --x .. --y .. --z .. --out ..`
로 드론을 순간이동시켜 정지 프레임을 찍는 유틸리티. 새 세션도 이 방식을 따를 것을 권장.

## 6. 사용자가 지금(2026-09-11) 명시적으로 요청한 것 — 다음 세션이 꼭 볼 것

### 6-1. 맵이 여전히 부실함 — 맵 꾸미기를 통째로 맡기고 싶어함

사용자 원문 취지: "일단 여기까지 한 거, 맵이 아직 부실해. 전에 줬던 사진들 기반으로
뉴욕시티스러운 느낌, 그리고 진짜 마을 느낌이 나는 그런 마을을 원해." 즉:

- 지금 맨해튼 구역/마을 구역이 **컨셉은 맞지만 밀도·디테일이 부족**하다는 평가.
  (참고: 이전 세션에 사용자가 실제 테헤란로/강남 항공사진/한강대로 인터체인지 사진을
  첨부하며 "이런 밀도"를 요청한 적 있음 — 이 문서만 봐서는 그 이미지 파일 자체는 없으니,
  **사용자에게 다시 요청하거나, 이미 커밋된 `docs/images/`의 결과물과 비교해 감을 잡을 것**.)
- 사용자는 **"맵 꾸미기"를 하나의 델리게이트 가능한 작업 단위로 맡기고 싶어함** —
  즉 다음 세션이 이 부분을 자율적으로 주도해서 밀도/디테일/현실감을 끌어올리는 걸
  기대하고 있음. 구체적으로 부족한 것(다음 세션이 판단할 후보):
  - 맨해튼: 건물 종류가 5종(SKY_MESHES)뿐이라 반복감이 있음, 도로 디테일(횡단보도,
    신호등, 가로수) 없음
  - 마을: 논밭/울타리/헛간 같은 "진짜 시골" 요소 없음, 집 종류가 BUILDING_MESHES
    21종이지만 다 비슷한 실루엣
  - 전반적으로 Kenney City Kit(CC0, 무료 팩 4종)의 한계 — 더 다양한 무료/저가 에셋
    조사가 필요할 수 있음 (Poly Haven, itch.io, CGTrader 등 — 단 Gazebo Classic의
    Ogre1 렌더러 한계로 "사실적인 PBR"까지는 안 됨, 아래 6-2 참고)

### 6-2. 엔진/툴 대안 제안 요청 — 특히 "ROS + Unity"

사용자 원문 취지: "혹시 다른 방법이 있다면, ROS 유니티 같은? 그걸로 옮기자는 제안을
해봐." 아래는 이 문서 작성 시점에 정리해둔 예비 분석 — **다음 세션이 조사부터 다시
시작할 필요 없도록 미리 적어둠**:

**배경**: Gazebo Classic(Ogre1)은 렌더링 한계가 뚜렷하고(그림자/PBR 없음), Gazebo
Sim/Fortress(Ogre2)는 이 WSL 환경에서 카메라 센서 크래시로 아예 못 씀(5-6번 항목).
그래서 "완전히 다른 툴로 갈아타는 게 낫나?"라는 질문이 자연스럽게 나온 상황.

**ROS + Unity 옵션 개요**:
- Unity ↔ ROS 2 연동은 성숙한 공식 경로가 있음: **Unity Robotics Hub**
  (`ROS-TCP-Connector` Unity 패키지 + `ROS-TCP-Endpoint` ROS 2 노드) — TCP로
  메시지를 주고받고, `.msg`/`.srv`를 Unity C# 클래스로 자동 생성해주는 툴도 제공됨.
- Unity는 HDRP/URP로 실제 PBR 렌더링(그림자, 반사, 라이트매핑)이 기본 지원 —
  사용자가 원하는 "뉴욕시티 같은 사실적인 그림"이 렌더러 한계 없이 나옴.
- 카메라/depth 센서: Unity 자체 카메라로 RGB는 바로 되고, depth는 커스텀 셰이더나
  **Unity Perception 패키지**(원래 합성 데이터/세그멘테이션/깊이 GT 생성용으로 만들어진
  공식 패키지)로 비교적 깔끔하게 얻을 수 있음 — 오히려 지금 GT를 기하투영으로 계산하는
  것보다 더 정확한 ground truth를 자동으로 얻을 여지도 있음(평가 파이프라인에 도움될 수 있음).
- 에셋 생태계: Unity Asset Store에 도시/자연 에셋이 훨씬 많고 저렴함(Kenney CC0보다
  다양) — "돈 주고 좋은 에셋 사서 로직에 집중"하고 싶다는 사용자의 원래 니즈에 부합.

**현실적으로 큰 비용**: 이건 렌더러만 바꾸는 게 아니라 **재플랫폼**임 —
1. 월드 생성 파이프라인 전체 재작성 (지금 Python으로 SDF 텍스트 생성 → Unity 씬은
   C# 스크립트로 절차생성하거나 에디터에서 수동 배치)
2. 물리/충돌 엔진이 PhysX로 바뀜 — 드론 kinematic 이동(`SetEntityState` 서비스로
   순간이동시키던 것) 로직을 Unity Rigidbody/Transform 기반으로 재작성
3. 센서 파이프라인 재작성 (카메라 플러그인 → Unity 카메라 컴포넌트 + Perception 패키지)
4. `landing_detector.py`/`mission_controller.py`는 ROS 토픽/서비스만 보므로 **메시지
   타입이 같으면 이 둘은 거의 안 건드려도 될 가능성이 높음** — 이게 그나마 위안.

이건 Fortress 포팅 시도(2~4일 추정, 실제론 렌더러 크래시로 시작도 못 함)보다 스코프가
더 큼 — **Fortress처럼 "하루짜리 타당성 스파이크부터" 접근을 강력히 권장**: Unity 빈
씬 + ROS-TCP-Endpoint 연결 + 카메라 토픽 하나 ROS에 퍼블리시하는 것까지만 먼저 만들어서
실제로 되는지 확인한 다음에 전체 포팅 여부를 결정할 것. (Windows 네이티브에서 Unity
Editor를 돌리고 WSL의 ROS 2와 통신하는 구조가 될 가능성이 높음 — 네트워킹 설정도
스파이크 단계에서 같이 검증해야 함.)

**다음 세션에게**: 이 분석을 사용자에게 보여주고, 시간 여유(12월 말 마감, 격주 발표)를
고려했을 때 스파이크를 해볼지, 아니면 Gazebo Classic에 남아서 에셋/디테일만 개선할지
**사용자에게 직접 물어볼 것** (AskUserQuestion 등으로) — 이건 프로젝트 방향을 바꾸는
큰 결정이라 다음 세션이 임의로 정하면 안 됨.

### 6-3. 최종 목표: "인터랙션 있는 시뮬레이션 환경"을 보여주기

사용자는 게임 같은 화려한 인터랙션을 요구하는 건 아니지만, 최종적으로 **"보여줄 수
있는" 시뮬레이션 환경**을 원함. 이미 있는 것: 클릭-리타겟(`/clicked_point`), 배송
목적지 9곳(이번 세션에 추가, 아직 RViz에서 라벨/발견 가능성 약함). **자연스러운 다음
단계는 5절의 "#5 통합 대시보드" 작업**과 겹침 — `landing_spots`를 읽어서 RViz
`MarkerArray`로 이름표 띄우는 노드를 만들면, "지도 보고 목적지 클릭 → 드론이 날아가서
착륙" 이라는 완결된 데모 흐름이 생김. 이게 사용자가 말한 "인터랙션"의 실체에 가장
가까울 것으로 판단됨(다음 세션이 사용자와 확인).

### 6-4. (나중, 지금 아님) 비행 중 시각화 강화 + YOLO 추가 여부

사용자가 "이건 나중이고"라고 명시적으로 유예함 — **지금 착수하지 말 것**, 다만 기록:

- 이미 있는 것: `dino_seg.py`의 DINOv2 PCA 시각화(`docs/images/dino_pca_viz.png`),
  착륙점 오버레이(안전=녹/장애물=적/선택점=황, `docs/images/detection_overlay.png`),
  `demo.rviz`에 하강캠 오버레이/PCA/포인트클라우드/멀티캠 레이아웃.
- 사용자가 원하는 추가: 비행 중 "무엇을 detect했다"는 실시간 표시, 비행 경로 시각화.
- YOLO 추가 여부: 사용자도 확신 없어함("추가해야하나? 이건 나중"). 판단 보류.
  참고로 지금 방법론(DINOv2 무라벨 군집 + depth)의 핵심 차별점은 **라벨/사전학습
  객체탐지기 없이** 처음 보는 장면에서 안전면을 찾는 것 — YOLO(지도학습 객체탐지)를
  섞으면 "장애물 종류 식별"에는 도움되지만, 방법론의 "무라벨" 강점 서사와 충돌할 수
  있음. 나중에 이 얘기 다시 나오면 이 트레이드오프를 먼저 짚어줄 것.

## 7. 알려진 함정 (재현/디버깅 시간 아끼려면 읽을 것)

1. **`pkill -f gazebo`류 자기자신 킬 버그** — 3절 참고.
2. **`flat_patch()` 화이트아웃** — 밝은 색은 `ground_patch()`(box 기반) 쓸 것,
   `flat_patch()`는 어두운 색(도로 등)에만 안전.
3. **WSL 백그라운드 프로세스 관리** — Bash 툴의 `run_in_background: true`로 Gazebo를
   띄우면 이상하게 즉시 죽는 경우가 있었음(원인 미확정, self-kill 버그와는 별개 현상일
   수도 있음) — 안정적으로 되는 패턴: 한 번의 foreground 호출 안에서
   "실행&(백그라운드) → sleep으로 로딩 대기 → 작업 → kill" 을 전부 처리.
4. **mesh_tower()/mesh_house() 등은 target_h/native_h로 등방 스케일** — 목표 높이를
   키우면 폭도 비례해서 커짐(원본 메시 비율 유지). 좁은 간격에 큰 target_h를 주면
   건물이 서로 뚫고 들어갈 수 있음 — 새 구역 만들 때 간격을 넉넉히 잡을 것.
5. **Gazebo Classic 11 + WSLg**: `unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER` 필요
   (run_city_demo.sh에 이미 있음), GPU 렌더링은 D3D12 변환 경유라 반복 기동 시
   가끔 WSL 자체가 불안정해짐(RPC 에러) — 심하면 `wsl --shutdown` 후 재시도.

## 8. 파일 빠른 참조

| 경로 | 줄 수 | 역할 |
|---|---|---|
| `worlds/generate_city.py` | 977 | 격자도시+맨해튼+마을+배송목적지9곳 생성기 (이번 세션 핵심 작업물) |
| `worlds/generate_world.py` | 750 | 단일회랑 절차생성 (원본, 거의 안 건드림) |
| `landing_detector.py` | 331 | DINOv2+depth 핵심 알고리즘 (안 건드림) |
| `mission_controller.py` | 379 | 비행 상태머신 (안 건드림) |
| `run_city_demo.sh` | - | 격자도시 데모 원샷 실행 |
| `docs/fortress_spike_findings.md` | - | Fortress 불가 판정 근거 |
| `docs/images/` | - | README용 스크린샷 7장 |
| `eval/capture_frame.py` | - | 정지 프레임 캡처 유틸(검증용, 새 세션도 활용 권장) |
