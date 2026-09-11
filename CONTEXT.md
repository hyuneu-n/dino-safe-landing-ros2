---
project: 졸업작품 v2 — 비전 기반 배송 드론 안전 착륙지 탐지
updated: 2026-09-11
status: 구현 진행 중 — 격자도시+맨해튼+마을+배송목적지 9곳까지 완료, 대시보드/맵 디테일 작업 남음
deadline: 2026-12월 말 (최종), 그 전까지 격주 중간발표
tags: [졸작, 드론, 비전, DINOv2, Gazebo, ROS2]
---

# 졸업작품 Context (현행)

> [!important] 새 AI 세션이면 이 파일보다 `docs/HANDOFF.md`를 먼저 읽을 것
> 이 파일은 설계 결정의 "왜"를 담은 이력이고, 지금 뭘 했고 뭐가 남았는지는
> `docs/HANDOFF.md`가 더 최신이고 더 정확하다.

> [!warning] 이 파일이 최신이다
> `시행착오/context.md` 는 **폐기된 v0(SLAM 자율탐사 로봇, 2026-04-20)** 문서다. 혼동하지 말 것.

## 한 줄 정의

배송 드론이 목적지 근처에 도착한 뒤, **RGB+depth 카메라만으로 현재 장면을 분석해** 차·사람·장애물을 피한 안전한 빈 공간을 찾아 정밀 착륙하는 시스템.

핵심 메시지: **"좌표는 GPS가 주지만, 그 자리에 지금 뭐 있는지는 카메라만 안다."**

---

## 주제 변천사 (3단계)

| 버전 | 주제 | 상태 | 위치 |
|---|---|---|---|
| v0 | SLAM 자율탐사 로봇 (Go2, 재난환경) | 폐기 | `시행착오/context.md` |
| v1 | AutoPad — UAM 헬리패드 정밀착륙 (Unity+Cesium) | 폐기 (교수님 기각) | `C:\Users\hyuneun\Documents\workspace\graduation\AutoPad\` |
| **v2** | **배송 드론 안전 착륙지 탐지 (Gazebo+ROS2)** | **현행** | **WSL `~/safe_landing/`** |

v1 기각 사유: "좌표 주고 거기 가는 거면 GPS랑 뭐가 다르냐"

---

## 프로젝트 경로 ⭐

> [!important] 실제 작업물은 Windows 파일시스템이 아니라 **WSL 안**에 있다
> Windows 탐색기 경로: `\\wsl$\Ubuntu-22.04\home\hyuneun\safe_landing\`

| 대상 | 경로 |
|---|---|
| **현행 코드** | WSL Ubuntu-22.04 : `~/safe_landing/` |
| **월드 파일** | WSL : `~/gz_worlds/residential_delivery.world` |
| PX4 (미통합) | WSL : `~/px4/` |
| 폐기된 v1 | Windows : `C:\Users\hyuneun\Documents\workspace\graduation\AutoPad\` |
| 데모 영상 | Obsidian : `graduationProject/데모영상_20260601/` |

### `~/safe_landing/` 주요 파일

| 파일 | 역할 |
|---|---|
| `landing_detector.py` | DINOv2 + depth → 안전점 판정, 오버레이/마커 발행 (핵심) |
| `mission_controller.py` | 상태머신 (IDLE→TAKEOFF→CRUISE→ARRIVE→SCAN→DESCEND→LANDED) |
| `dino_seg.py` | DINOv2 특징추출 + PCA + K-means (단독 실행 가능) |
| `run_demo.sh` / `run_headless.sh` / `go.sh` | 실행 스크립트 |
| `demo.rviz` | RViz 설정 |

---

## 진행 이력

- **~5/31**: `landing_detector.py`, `mission_controller.py` 최종 수정
- **6/1**: 데모 영상 촬영 → **종합설계 5차 발표** (대본: `시행착오/발표구성_5차_20260601.md`)
- **6/10**: Gazebo Fuel 메시 모델로 환경 사실화 시도 → **막힘. 여기서 중단**
- **8/28**: 재개. 설계 방향 확정 (아래)
- **9/2**: git init + 첫 커밋(맥 세션) → 윈도우 세션에서 순서대로:
  - #1 `worlds/generate_world.py` 절차생성 스크립트. Fuel `<include>` 전부 SDF 기본
    지오메트리로 대체, `--seed`/`--batch N` 지원, headless gzserver 스모크테스트 통과
  - #2 RViz Publish Point로 목적지 클릭 지정 (A안). `mission_controller.py`/
    `landing_detector.py` 둘 다 `/clicked_point` 구독, 라이브 검증 완료
  - `generate_layout()`/`build_world_xml()` 분리 리팩터 — #3의 GT 마스크가 실제
    배치 좌표를 그대로 재사용하게 하려고
  - #3 baseline 2개(PX4식 depth-only, OpenLander) + 3열 비교표 (`eval/`)
  - #4 같은 스크립트로 n=24 성공률 실험까지 이어서 진행 — 위 표 참고, 상세는 `eval/README.md`
  - #6 Docker + 루트 README.md. #5(RViz 레이아웃)는 GUI 없는 세션이라 사용자 판단으로
    보류, 본인 PC에서 화면 보면서 진행하기로 함

---

## ⭐ 2026-08-28 세션에서 확정된 결정

> [!success] 결정 1 — Gazebo 유지. Unity 이식 안 함
> 조사 결과 Unity로 가도 얻는 게 없음:
> - Unity에서 비전 기반 착륙지 선정하는 오픈소스 **0건** (30개+ 조사)
> - 최고 후보(Unity 공식 Apache-2.0)도 **Python gRPC 서버 별도 실행 필수** → exe 단독배포 안 됨
> - **도시/동네 맵 포함된 Unity 드론 레포 없음** → 맵은 어차피 직접 제작
> - AirSim/Colosseum의 Unity 지원은 방치·archived

> [!success] 결정 2 — 배포 목표는 "설치하면 동작" (Docker)
> "exe 더블클릭으로 어디서든"은 포기. 대신:
> - **시연** = 내 PC에서 (설치 0분) ← 진짜 요구사항
> - **재현성** = Docker 이미지 + README ← GitHub 공개용, URL 랩 어필에도 유리
> - 이 결정으로 Unity 포팅 / DINOv2→ONNX→Sentis 변환 / 유료 에셋 전부 스코프에서 제거됨

> [!success] 결정 3 — 화면 방향은 B (기술적 시각화)
> "사실적으로 예쁜 동네"(A)가 아니라 **"기술적으로 있어 보이는 화면"**(B)을 목표로 함.
> → 포인트클라우드, 세그멘테이션 오버레이, 후보 마커, 수치 패널 = 이미 갖고 있는 것
> → 로우폴리 박스 월드로도 목표 달성 가능

> [!success] 결정 4 — 인터랙션은 A 먼저, 되면 B
> | A (먼저) | B (나중) |
> |---|---|
> | 목적지만 클릭 → 드론이 가서 착륙 | 목적지 + **장애물도 직접 배치** |
> | RViz `Publish Point` 활용, 반나절 | Gazebo 모델 스폰 + 클릭 처리 |
> | | "미리 짜놓은 시나리오 아니냐" 의심 원천 차단 |

> [!success] 결정 5 — 월드는 절차생성
> Fuel 에셋 붙이기(6/10에 막힌 것) 대신 **파이썬으로 `.world` 자동 생성**.
> 요구조건: 넓음 / 집 많음 / **높이 제각각** / 건물 때문에 회피 어려움 → 전부 박스 파라미터로 해결.
> 부수효과: **랜덤 시드로 월드 N개 생성 → 정량실험 인프라가 공짜로 생김**
> 메시 업그레이드는 옵션(`--mesh` 플래그)으로 나중에. Kenney City Kit Suburban(CC0 무료, OBJ 제공)이 후보이나 저층주택 위주라 협곡은 박스로 만들어야 함.

---

## 조사 결과 (2026-08-28, 에이전트 3개 병렬 조사)

### ⭐ 가장 중요한 발견 — 내 방식이 오픈소스 공백이다

| 축 | 내 구현 | 공개 오픈소스 |
|---|---|---|
| Semantic (뭔지 안다) | DINOv2 patch feature (라벨 불필요) | OpenLander, PEACE 등 있음 |
| Geometric (평평한지 안다) | depth + K-means | PX4만 있음 |
| **둘의 융합** | **✅** | **코드 공개된 것 없음** |

- RGB만 쓰는 것들 → "아스팔트네" 알지만 **평평한지 모름**
- depth만 쓰는 PX4 → "평평하네" 알지만 **사람인지 차인지 모름**
- **PX4-Avoidance(752⭐, 이 분야 유일 메이저 스택)가 2024-04 archived** → "현재 유지되는 오픈소스 SLZ 스택이 없다"가 졸작 동기가 됨
- 이 분야 레포 대부분 별 한 자릿수 → 내 구현이 초라한 게 아니라 분야 자체 규모가 그러함

### 정량비교용 baseline (확보 완료)

| baseline | 성격 | 비용 | 출처 |
|---|---|---|---|
| PX4 `safe_landing_planner` 방식 재구현 | depth-only 기하 | **50줄 이내** | BSD-3, archived라 원본 실행 불가하나 알고리즘 문서화됨 |
| **OpenLander** | RGB-only 딥러닝 | 낮음 (ONNX, Windows 네이티브, ROS 불필요) | MIT, 156⭐ |
| RANSAC 평면적합 | K-means 대체 (ablation용) | 낮음 | `felixchenfy/ros_detect_planes_from_depth_img` |

→ **내 방식 + 위 2개 = 3열 비교표**. "부실함" 해소의 핵심.

> [!danger] 비교 시 주의
> 내 수치(Gazebo 시뮬)를 논문들 mIoU(실사 데이터셋)와 **직접 비교 금지**. 도메인 불일치.
> 정직한 방법: 같은 Gazebo 씬에 GT 마스크 만들고 3개를 동일 조건에서 실행.
> 외부 논문 수치는 관련연구 표에만 넣고 "평가 조건 상이함" 명시.

### 관련연구 인용 후보 (코드 미공개, 논문만)

| 논문 | venue | 요지 |
|---|---|---|
| SafeUAV (Marcu et al.) | ECCV-W 2018 | RGB만으로 depth+안전영역 동시추정 |
| VisLanding (Tan et al.) | IROS 2025 | Metric3D v2 depth-normal 시너지. WildUAV 76.61 mIoU |
| PEACE (Bong et al.) | 2024 | CLIPSeg 자동 프롬프트. 성공률 57%→92% |
| NeuroSymLand | arXiv 2607.02277 | semantic scene graph + 심볼릭 규칙. 72중 61 성공 (PEACE 57, SafeUAV 47) |

> NeuroSymLand의 비교표 포맷(Succ + MOD=minimum obstacle distance)은 그대로 차용 가치 있음. MOD는 내 파이프라인에서도 바로 계산 가능.
> ⚠️ NeuroSymLand는 논문에 코드 링크가 placeholder 상태 → "재현 가능"이라고 쓰지 말 것.

---

## 확정된 남은 작업 (6~8주)

| # | 작업 | 대응 (5차 발표 "남은 일") | 비고 |
|---|---|---|---|
| 0 | **백업** — git init (+ GitHub는 아직) | — | ✅ 완료 (로컬 커밋만, `gh` 미설치로 원격 push는 남음) |
| 1 | **월드 절차생성 스크립트** | #1 환경 강화 | ✅ 완료 (`worlds/generate_world.py`, 커밋 `41ccf49`) — 넓게/많게/높이 다양하게, Fuel 미사용 |
| 2 | **클릭 → 목적지 지정 (A안)** | 신규 (참여형) | ✅ 완료 (커밋 `4d72ede`) — demo.rviz에 PublishPoint 툴 추가, `/clicked_point` 구독으로 `mission_controller.py`/`landing_detector.py` 둘 다 재지정. 실제 gzserver로 라이브 검증(클릭 후 예상 시간에 도착 확인) |
| 3 | **baseline 2개 붙이기 + 3열 비교표** | #2 정량 증명 | ✅ 완료 (커밋 `a94861a`, `eval/`) — PX4식 depth-only 재구현 + OpenLander(ONNX) + 우리 방법, GT는 손라벨 아니고 `generate_layout()` 지오메트리 투영으로 계산 |
| 4 | **랜덤 시드 월드 N개로 성공률 실험** | #2 정량 증명 | ✅ 완료 (n=24, 같은 `eval/run_comparison.py`) — **우리 92%/MOD 3.69m, depth-only 71%/MOD 2.22m(장애물 가장자리에 붙음), OpenLander 46%/제시율 83%(도메인 격차로 확신에 찬 오판)**. 세 지표 모두 우리 방법이 우위. 상세: `eval/README.md` |
| 5 | 통합 대시보드 / RViz 레이아웃 정리 | #3 대시보드 | 대기 중. 선행작업 두 단계 완료됐음:<br>**① 도시 확장 1차**(커밋 `ce0675e`) — Kenney City Kit 4종(Suburban/Commercial/Industrial/Roads, CC0)로 "교외→산업지대→상업/도심(스카이스크래퍼)→목적지 교외" 단일 회랑 도시. 확립된 변환 파이프라인(텍스처 경로 평탄화→assimp OBJ→DAE→ambient 0→0.6→roll=+π/2)이 신규 팩들도 재시행착오 없이 통과. 기본 ground_plane이 작아 바닥이 하늘로 끊기던 문제도 큰 잔디 패치로 해결.<br>**② 격자 대도시 + 실제 경로계획**(커밋 `0f76af9`+`98e991e`) — "테헤란로처럼 복잡하게 헤매는" 요청에 따라 `worlds/generate_city.py` 신설: 6애비뉴×5스트리트 격자, Dijkstra로 최단경로 계산(도심 슈퍼블록으로 직선 관통 막아서 실제로 꺾이게 설계), `mission_controller.py`는 이제 이 웨이포인트를 따라 코너를 실제로 돈다(heading 추적, 전방 depth 회피를 진행방향 기준으로 회전, WAYPOINTS_FILE 없으면 예전 직행 2점 경로로 완전히 하위호환 — 둘 다 실측 검증됨: 7턴 450m 경로 54.07초 만에 완주, 예상 56.25초와 근접). `run_city_demo.sh` 신설(기존 `run_demo.sh`는 안 건드림).<br>도로는 여전히 각 구간 내에서는 직선(격자 자체가 코너를 만듦, 곡선 도로 타일은 아직 안 씀). 사용자 GUI에서 최종 확인 대기 중 |
| 6 | **Docker 이미지 + README** | 신규 (재현성) | ✅ 완료 (커밋 `4ea6a40`) — `Dockerfile`/`docker-entrypoint.sh`/루트 `README.md`. ⚠️ **미검증**: 이 세션엔 docker CLI가 없어서 `docker build`/`run`을 한 번도 못 돌려봄 — 본인 PC(Docker Desktop)에서 첫 빌드 확인 필요 |
| 7 | (여유되면) 장애물 배치 인터랙션 B안 | — | 시간 봐서 |

---

## 알려진 이슈

- **연구실 PC가 자꾸 오프라인** → 절전 때문. 다음 접속 시 조치 예정:
  - `powercfg /change standby-timeout-ac 0`, `powercfg /hibernate off`
  - 랜카드 전원관리 해제
  - Tailscale 부팅 자동실행 확인
  - (원격 Wake-on-LAN은 같은 랜에 다른 기기 필요 → 불가)
- ~~`~/safe_landing/` 이 git 저장소가 아님~~ → 9/2 해결 (git init, 원격 push만 남음)
- 6/10 Fuel 모델 로드 실패 원인 미규명 → 절차생성으로 우회하므로 조사 불필요
- **[미확인, 9/2 발견]** `logs/demo_latest.log`(6/1 데모, 구세계) 마지막 20여 초 구간에서
  `landing_detector` 출력이 `px=(320,240) score=0.85 alt=1.0m`로 완전히 고정됨.
  `h_alt=max(1.0, odom_z)`라 1.0m는 클램프값일 수 있고, alt뿐 아니라 px/world/score까지
  bit-identical하게 반복된 걸 보면 RGB/depth 콜백 자체가 멈춘 것(=Gazebo 쪽에서 드론이
  실제로 정지)으로 보임 — `mission_controller.py`의 DESCEND 단계에서 `set_entity_state`
  비동기 호출이 씹혀 명령과 실제 물리 상태가 어긋났을 가능성. 구세계 데이터라 급하지 않지만,
  절차생성 월드로 데모 재녹화할 때(작업 #5 근처) 같은 현상 재현되면 원인 조사할 것.

---

## 관련 문서

- [[시행착오/📋 졸작 v2 — 데모 정리 (교수님용)]] — 시스템 상세 (아키텍처, 알고리즘, Q&A)
- [[시행착오/발표구성_5차_20260601]] — 5차 발표 슬라이드 구성 + 대본
- [[데모영상_20260601/README_영상설명]] — 데모 영상 설명
- [[시행착오/졸작 주제 확정_빠구당함]] — v1(AutoPad) 문서, 폐기됨
- [[시행착오/context]] — v0(SLAM) 문서, 폐기됨
