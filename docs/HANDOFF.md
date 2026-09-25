# Codex 데스크톱 앱 인수인계

기준: **2026-09-23**. 이 문서가 현재 작업 상태의 기준이다. 예전 내용을 누적한 인수인계는 [보관본](archive/HANDOFF_before_desktop_20260915.md)에 있다. 보관본의 건물 수·경로·우선순위는 최신 상태로 사용하지 않는다.

> **2026-09-23 기술 방향 및 구현 갱신:** 지도교수 피드백의 **명시적인 경로 생성** 요구에 따라 전방 Depth 비용 지도 → 1·2·3스텝 후보 → 첫 목표 실행 → 재계획 구조를 구현했다. 기본은 horizon 2이며 기존 좌·중·우 반응형 회피는 비교군으로 보존했다. 순항 DINO 결합은 아직 실험 전이다. [구현 및 예비 결과](PATH_PLANNING_IMPLEMENTATION_20260923.md), [녹취 분석과 설계](PROFESSOR_FEEDBACK_AND_PATH_PLANNING.md)를 함께 본다.

## 1. 가장 먼저 알아야 할 사용자 결정

- 사용자는 **현재 맵에 만족하며 환경 제작을 이 정도로 마무리**한다고 명시했다. 임의로 맵을 더 확장하거나 캠퍼스를 다시 디자인하지 않는다.
- **Depth-only 지역 경로 생성 1차 버전**을 구현했다. 다음은 반복 실험 → legacy 기준선 → DINO 결합 비교 순서다.
- 이번 요청은 README 대폭 개편, 데스크톱 전환용 상세 인수인계, 새 지도교수님에게 설명할 종합설계 첫 발표 PPT 제작이다.
- 사용자는 자율 편집과 불필요한 확인 생략을 원한다. 이미 허용된 가역 작업은 진행한다. 다만 실제 앱의 관리형 권한·명령 승인 규칙을 해제했다고 말하지 않는다.
- 한국어로 소통한다. 기능을 완성한 것과 제안한 것, 렌더링한 것과 자율비행 검증한 것을 구분한다.

## 2. 프로젝트 목적과 현재 정확한 설명

배송 목적지 좌표만으로는 그 자리에 사람·차량·적재물이 있는지 알 수 없다. **RGB-D로 장애물과 빈 공간을 판단해 목적지 근처의 안전 착륙점을 고르는 시스템**을 만든다.

DINOv2 특징 군집과 depth를 결합한다. 객체 이름을 분류하는 YOLO식 검출은 구현하지 않았다. 위치 입력은 Gazebo odom, 비행은 `set_entity_state` 위치 지정이다. VIO/SLAM에 의한 완전한 비전 전용 위치 추정, 비행 동역학, 실기체 제어를 구현한 것으로 설명하지 않는다.

목표 시연은 목적지 선택 → 이동 중 회피 → 현장 스캔 → 빈 공간 선택 → 사람 개입 시 재평가 → 착륙 또는 대기다. 이 전체 흐름은 아직 검증되지 않았다.

## 3. 데스크톱에서 이어받기

같은 WSL 폴더 **`/home/hyuneun/safe_landing`**를 연다. Windows 경로는 기존 문서상 `\\wsl.localhost\Ubuntu-22.04\home\hyuneun\safe_landing`이다. 배포판 이름이 다르면 현재 WSL 이름을 확인한다.

새 앱 첫 메시지로 다음을 붙여 넣으면 된다.

> `docs/HANDOFF.md`, `docs/PATH_PLANNING_IMPLEMENTATION_20260923.md`, `docs/PROFESSOR_FEEDBACK_AND_PATH_PLANNING.md`를 먼저 읽어줘. 맵 제작은 완료했고 Depth-only 지역 경로 생성 1차 버전과 H1/H2/H3 n=1 예비 비교까지 끝났어. 다음은 반복 실험, legacy 비교, 순항 DINO 결합 비교야. DINO가 현재 순항 계획에 들어간 것으로 과장하지 말고 기존 변경사항을 보존해줘.

첫 확인은 `git status --short`, 최신 HANDOFF, PROJECT_STATUS, 발표 자료 README다. 사용자가 추가한 `docs/.obsidian/` 등 알 수 없는 파일은 임의 삭제하지 않는다. 이번 작업 중 커밋·푸시를 수행하지 않았다. 기존 작업 트리에 미커밋 코드·이미지가 많으므로 새 체크아웃/브랜치 이동/정리 전에 변경분을 보존한다.

## 4. 환경과 실행

| 항목 | 현재 경로/구성 |
|---|---|
| 작업 경로 | `/home/hyuneun/safe_landing` |
| Python/ROS 환경 | `/home/hyuneun/venv_ros`, `/opt/ros/humble/setup.bash` |
| 시뮬레이터 | ROS 2 Humble + Gazebo Classic 11, WSLg |
| 맵 생성물 | `worlds/generated/metropolis_seed0.world`, 옆 `_assets/`, `.waypoints.json` |
| 환경 영상 | `artifacts/metropolis_tour/metropolis_flythrough.mp4` |
| 영상 규격 | 60초, 1280×720, 24fps, H.264, 1,440프레임, 43,656,805 bytes |
| 실행 로그 | `logs/city_*.log` |
| 센서 검증 이미지 | `docs/images/metropolis/` |
| 발표 자료 | `docs/presentation/README.md` 참조 |

```bash
cd /home/hyuneun/safe_landing
bash run_city_demo.sh 0 world   # 환경 미리보기, 드론 자동 출발 없음
bash run_city_demo.sh 0         # 탐지기 + 기존 기본 미션 + RViz
bash run_city_demo.sh 0 norviz  # RViz 제외
```

기본 미션 경유점은 **창고 (-210,-88) → 도심 대로 → East Gate (150,0)**다. 새 경로의 종단 간 비행 성공을 검증한 것은 아니다. `world` 모드부터 확인한다.

GUI는 `GAZEBO_MASTER_URI`(미지정 시11345)로 접속한다. 세션 환경에 이 변수가 이미 있을 수 있다. 다른 서버가 포트를 사용하면 실행기가 중단한다. 전체 Gazebo 프로세스를 `pkill`하지 않는다. 필요하면 별도 포트와 `ROS_DOMAIN_ID`를 지정한다.

검증 서버는 `VALIDATION_MASTER_URI`/`VALIDATION_ROS_DOMAIN_ID`(기본11366/66), 영상 서버는 `TOUR_MASTER_URI`/`TOUR_ROS_DOMAIN_ID`(기본11469/169)를 사용한다. GUI의 환경변수를 그대로 상속해 충돌했던 문제를 해결한 구성이다. 실행한 스크립트는 자신이 만든 서버만 종료한다. 이전 앱의 실행 세션 ID나 PID를 새 세션에서 유효하다고 가정하지 않는다.

WSLg에서는 `LIBGL_ALWAYS_SOFTWARE`, `GALLIUM_DRIVER`를 unset하는 현재 실행기를 사용한다. 외부 Fuel 모델 데이터베이스는 localhost의 닫힌 포트로 지정해 불필요한 다운로드를 막는다. 현재 생성 맵에는 외부 에셋 다운로드가 필요 없다.

## 5. 맵 구조와 보존할 설계

- 약 2.5×1.9km, seed0 건물148동·목적지22곳. 지정 좌표가 점유된 상황은4곳.
- 뉴욕풍 도심, 공장·물류 야드, 상점, 타운, 비격자 외곽 주거지, 강 양안, 높이24m 고가도로·34m 사장교·270도 램프, 산동네.
- SkyDrop 물류창고가 실제 드론 생성 위치이며 manifest `origin`, `start_xy`, `waypoints_xy`에 반영된다.
- 맥도날드·써브웨이 간판은 메시 문자. 학교는 사용자 사진3장의 원통 유리 타워, 아치 강의동, 회색 지붕, 노란 포인트 외벽, 줄무늬 광장, 분수·계단 정원을 반영한 창작 배치다.
- 캠퍼스 목적지 `(340,330,z=12)`. 다른 높이 있는 목적지는 교량34m·산마당36m·능선96m. 총4곳은 기존 제어기 수정 전 자동 미션에 사용하지 않는다.
- 목적지 manifest에는 `requires_elevation_support`, `flight_validated=False`가 있다. 새 목적지를 비행 검증 완료로 표시하지 않는다.

| 파일 | 핵심 역할 |
|---|---|
| `worlds/generate_metropolis.py` | `Mesh`, `City`, 기본 구역·상황, SDF·DAE·manifest 생성 |
| `worlds/metropolis_region.py` | 지형·도로·강·고가 구조·산동네, 지형 절개·평탄 패드 |
| `worlds/metropolis_landmarks.py` | 물류창고·상점·출발점·간판 글리프, 캠퍼스 호출 |
| `worlds/metropolis_campus.py` | 사진 기반 학교 메시, 광장·정원·진입로 |
| `worlds/generate_world.py` | 이전 단일 회랑 평가 월드, 드론 SDF 템플릿 |
| `worlds/generate_city.py` | 이전 격자도시, 과거 경로계획 구현; 현재 기본 생성기 아님 |

주의할 지형 함정:

1. 건물 생성 함수의 고도 인자 `z`가 창문 루프에 덮이지 않게 `base_z`를 보존한다.
2. 도로/지형은 같은 DAE로 시각·충돌을 처리한다. 학교도 메시 충돌을 쓴다. 일반 수목·세부 구조 일부는 박스 근사다.
3. 평탄 패드나 캠퍼스 기초 슬래브가 경사 진입로를 덮을 수 있다. 현재 캠퍼스는 진입부를 가리는 기초 박스를 없앴다.
4. 경사로를 바꾸면 실제 down depth로 표면이 묻히지 않았는지 확인한다. 지형 샘플만 통과했다고 렌더링을 생략하지 않는다.
5. 생성 `.world`의 메시 경로는 절대 경로다. 옮긴 컴퓨터/폴더에서 재생성한다.

## 6. 코드의 현재 기술 상태

`landing_detector.py`: RGB/depth/odom 수신 → DINOv2+K-means 특징 군집 → 깊이 장애물/표면 후보 → 후보 점수화 → `/landing/target` 발행. `/landing/overlay`, `/landing/dino_pca`, `/landing/candidates`, `/landing/target_marker`도 발행한다.

`mission_controller.py`: `IDLE(선택) → TAKEOFF → CRUISE → ARRIVE → SCAN → DESCEND → LANDED`. 2026-09-23부터 기본 순항은 전방 Depth 지역 비용 지도와 horizon 2 후보 경로를 사용한다. 후보가 없으면 정지하고 제자리 yaw 재관측 후 다시 계획한다. 기본은 자동 이륙이며 `WAIT_FOR_DESTINATION=1`이면 `/clicked_point`가 올 때 물류센터에서 대기한다. 대기 중 선택은 기본 도심 경유점을 유지하고 최종 목적지를 붙이며, 비행 중 재지정만 현재 위치에서 직행한다.

중요: manifest waypoint는 맵 생성 시 미리 계산한 전역 경유점이며 카메라가 생성한 경로가 아니다. `local_path_planner.py`가 각 경유점 사이의 지역 경로를 생성한다. 기존 `compute_avoid_offset()`은 `LOCAL_PLANNER_MODE=legacy` 비교군으로 보존했다. 현재 지역 비용 지도는 Depth-only이며 순항 DINO 결합은 아직 비교 실험 전이다. 구현·예비 결과는 [경로 생성 기록](PATH_PLANNING_IMPLEMENTATION_20260923.md)에 있다.

다음은 **발견한 개선 후보이며 이번 문서/발표 작업에서 고치지 않았다**.

- `SCAN`: 후보가 없으면 목적지 좌표·z=.30으로 하강하는 폴백. 안전 대기·중단으로 바꿔야 한다.
- 착륙 후보가 오래되었거나 사라졌을 때 하강 금지와 재탐색이 충분하지 않다.
- 안전 영역 연결요소 평균점이 구멍 속 장애물 위에 놓일 수 있다. footprint 여유와 실제 마스크 내부 여부를 확인해야 한다.
- 탐지기 좌표 변환은 yaw0·GROUND_Z0. 높은 목적지와 카메라 자세를 처리하지 않는다.
- 순항22m·스캔12m·최종z.3 고정. 새 지형 고도에 맞춘 항법이 아니다.
- 지역 계획기는 NO_PATH 정지와 yaw 재관측을 구현했지만 모든 막다른 구조의 탈출이나 전역 재탐색을 보장하지 않는다.
- 자동 침입자는 `(150,9)`에서5초에 걸쳐 착륙점으로 간다. 먼 목적지에는 비현실적인 속도다.
- `LANDED` 유지 상태만 있고 반복 배송/복귀 미션은 없다.
- `eval/our_method.py`는 탐지 로직의 수동 복제. 기술 수정 후 평가와 실행 코드 일치 여부를 확인한다.
- 위치 지정 제어이므로 충돌 지오메트리가 있다는 것만으로 드론이 관통하지 않는다고 보장할 수 없다. 실제 접촉/관통 판정이 필요하다.

## 7. 검증 근거와 재현

```bash
python3 -m unittest discover -s eval -p test_metropolis.py
bash eval/validate_metropolis.sh /tmp/metropolis_validation
bash eval/validate_metropolis.sh /tmp/seokyeong_final --landmarks-only
bash eval/record_city_tour.sh artifacts/metropolis_tour --seconds 60 --fps 24
```

- 배치 검사는 seed0/7/23에서 통과했다: 건물 겹침, 모델 이름, 메시 참조, 도로 경사≤16%, 대체4×4m 공간, 드론 시작점/manifest 일치.
- 전체22개 목적지 RGB/depth를 검사했고, 마지막 학교 변경 후 새 랜드마크4곳과 학교 진입로3곳을 재검사했다. 산길5곳 검사는 이전 전체 검사에서 통과했다.
- 현재 `docs/images/metropolis/sensor_report.json`은 전체 기존 결과에 변경 구역 최신 결과를 반영한22개 기록이다. 모든 노선의 비행 성공 보고서가 아니다.
- MP4는 ffprobe로 길이/형식을 확인하고 전체 FFmpeg 디코딩 오류가 없는 것을 확인했다. 프레임별 카메라 경로 JSON이 옆에 있다.
- 이전 평가 `eval/results_n24/comparison.json` 원자료24행을 확인했다: ours22/24, depth-only17/24, OpenLander11/24; 평균MOD3.69/2.22/2.63m. **정지 한 프레임의 안전점 선택 평가**다. 도시 배송 성공률이 아니다.
- 2026-09-23 차단 회랑 예비 실험에서 H1/H2/H3를 각1회·45초 실행했다. H2만 목표10m 이내에 도달했고 SDF 명시 충돌체 관통 표본은0개였다. n=1이므로 최종 성공률이 아니다. 원자료는 `docs/results/path_planning_20260923/`에 있다.
- 2026-09-23 물류창고→맥도날드 실제 미션을 추적 카메라로 녹화했다. 124.2초·1280×720·15fps, 최종 `LANDED`, 위치 `(171.5,67.3,0.3)`, DINOv2 후보12회 중앙값 사용, 지역 계획254회 정상·초기 센서 대기7회·명시 SDF 관통0회다. 산출물은 `artifacts/autonomous_mcdonalds_delivery/`에 있으며 한 코스1회 결과다.
- `dashboard/`에는 실시간 ROS 웹 대시보드가 있다. 현재 위치/궤적, 미션·계획 상태, chase camera, landing overlay를 표시하고 목적지 카드는 `/clicked_point`를 발행한다. 현재 종단 간 검증된 맥도날드 카드만 활성화했다.
- Dockerfile/entrypoint는 보존하지만 실제 빌드·실행 검증은 미완료다.

## 8. 로컬 산출물과 이동 시 주의

Git 제외: `worlds/generated*/`, `frames/`, `logs/`, `eval/results*/`, 일부 모델 가중치, `artifacts/metropolis_tour/`.
같은 WSL 폴더를 데스크톱에서 열면 그대로 남아 있다. 새 clone에는 MP4·원자료·venv가 따라오지 않으므로 필요한 파일을 별도로 보존하거나 재생성한다.

현재 발표본은 `docs/presentation/safe_landing_review_compact.html`과 `safe_landing_review_compact.pptx`다. 사용자 제공 이전 발표 `종설_가장 마지막 진행상황.pptx`처럼 한 장의 정보 밀도를 높인 총10장 구성이다. 첫 장은 `종합설계 / 서현은` 표지이며, 본문마다 반복되던 소속·이름 머리말과 Part 전환장은 없다. ROS 토픽 경로·함수명 같은 원시 코드 표기도 청중이 이해할 수 있는 설명형 문구로 바꿨고 환경 영상도 없다. 약8~10분 대본은 `speaker_notes_compact.md`에 있으며 PPTX의 각 슬라이드 발표자 노트에도 내장했다. 생성 코드는 `build_html_deck_compact.py`와 `build_pptx_compact.py`, 전체 미리보기는 `overview_compact.jpg`다. 실제 Windows PowerPoint에서 10장 모두 열고 렌더링되는 것을 확인했다. 23장 V2와 16장 최초 버전은 이전 시안으로만 보존한다.

## 9. 발표와 다음 회의의 경계

발표는 새 지도교수님이 처음 듣는 전제로 문제 정의, 목표, RGB-D 방법, 현재 구조, 환경이 다양한 이유, 구현 상태, 이전 정지 장면 평가, 검증 한계와 시연 구상을 설명한다. “최초/유일한 연구”나 “도시 전역 자율배송 완성”을 주장하지 않는다.

9월 18일 발표 피드백으로 이동 회피를 **경로 생성 문제로 확장**하기로 했다. 첫 통합 시연 코스, 세부 격자 크기·계획 주기·비용 가중치는 아직 실험 전 초기값이며 확정값으로 취급하지 않는다. 구현과 평가는 [교수 피드백 문서](PROFESSOR_FEEDBACK_AND_PATH_PLANNING.md)의 단계와 지표를 따른다.

## 10. 다음 세션의 행동 순서

1. `WAIT_FOR_DESTINATION=1 bash run_city_demo.sh 0 norviz`와 `bash dashboard/run_dashboard.sh`로 관람객 흐름을 확인한다.
2. 현재 n=1인 H1/H2/H3 차단 회랑 실험을 여러 seed·초기조건으로 반복한다.
3. `LOCAL_PLANNER_MODE=legacy` 기준선도 같은 회랑과 지표로 측정한다.
4. 목표 이탈, NO_PATH 지속, 센서 중단을 실패 유형으로 나눠 집계한다.
5. 맥도날드 외 목적지에 전역 경로를 생성하고 코스별 종단 간 검증 후 대시보드 카드를 활성화한다.
6. DINO+Depth와 Depth-only를 비교한 뒤 순항에서 DINO를 유지할 근거를 결정한다.
