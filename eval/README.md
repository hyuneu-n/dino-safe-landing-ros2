# eval/ — 3열 비교 (남은작업 #3)

우리 방법(DINOv2 patch feature + depth 융합)과 baseline 2개를 **같은 절차생성 Gazebo 씬,
같은 GT, 같은 캡처 프레임**으로 비교한다.

## 왜 이 3개인가

| baseline | 성격 | 뭘 안 씀 |
|---|---|---|
| `baseline_depth_only.py` | PX4 `safe_landing_planner` 방식 재구현 (윈도우 표준편차 평탄도) | RGB/의미 정보 전혀 안 씀 |
| `baseline_openlander.py` | [OpenLander](https://github.com/stephansturges/OpenLander) RGB-only DNN 세그멘테이션 | depth 전혀 안 씀 |
| `our_method.py` | 우리 방법 (landing_detector.py에서 추출) | 둘 다 씀 |

CONTEXT.md 조사 결과의 핵심 발견 그대로: "semantic만"과 "geometric만"은 공개 구현이 있지만
"둘의 융합"은 공개된 게 없었다 — 이 표가 그 공백을 정량적으로 보여준다.

## 어떻게 공정하게 비교하나

1. **GT를 손으로 안 그림.** `gt_mask.py`가 `worlds/generate_world.py`의
   `generate_layout(seed)`가 실제로 배치한 오브젝트 좌표/치수를 그대로 카메라에 투영해서
   장애물 마스크를 계산한다 — 월드 정의와 정답이 항상 100% 일치.
2. **세 방법 다 정확히 같은 입력을 받는다.** `capture_frame.py`로 드론을 목적지 상공
   (SCAN_ALT=12m, `mission_controller.py`와 동일)에 순간이동시켜 RGB+depth를 한 장 캡처하고,
   그 한 프레임을 세 방법에 그대로 넣는다.
3. **"성공"의 정의를 셋에 동일하게 적용.** 선택 지점의 드론 footprint(1.0m) 반경 안에 GT
   장애물이 하나도 없으면 성공. 잔디/도로 같은 "표면 재질" 차이는 GT에 안 들어간다 — 그건
   우리 방법만의 부가 기능(색상 군집으로 잔디 배제)이라 baseline과 비교하면 불공정해진다.
4. **MOD(minimum obstacle distance)** — 선택 지점에서 가장 가까운 GT 장애물까지 실거리(m).
   NeuroSymLand 논문의 비교표 포맷(Succ + MOD)을 그대로 차용.

## 결과 (2026-09-02, seed 0~23, `--target-jitter 4.0`)

```
| 방법 | 착륙점 제시율 | 성공률(장애물 회피) | 평균 MOD(m) |
|---|---|---|---|
| 우리 방법 (DINOv2 + depth) | 100% (24/24) | 92% (22/24) | 3.69 |
| PX4식 depth-only | 100% (24/24) | 71% (17/24) | 2.22 |
| OpenLander (RGB-only DNN) | 83% (20/24) | 46% (11/24) | 2.63 |
```

(처음 seed 0~7, n=8로 먼저 검증했을 때는 88%/88%/62% — 표본을 24로 늘리니 depth-only와
OpenLander의 약점이 훨씬 뚜렷하게 갈렸다. n=8 결과는 git 히스토리의 이 파일 이전 버전 참고.)

원자료: `results_n24/comparison.json` (git에는 안 커밋 — 재현하면 다시 나옴, `.gitignore` 참고).

관찰:
- **depth-only**는 제시율은 100%로 우리 방법과 같지만 성공률이 71%로 확 떨어진다 — 낮은
  장애물(`low_garage`, 1.3m)의 평평한 지붕을 "평평하니까 안전"으로 착각해 장애물 위나
  바로 옆에 착륙을 시도하는 게 24개 중 7개 seed에서 확인됨(MOD가 0.15~0.75m로 매우 작은
  케이스들). 평균 MOD도 가장 낮다(2.22m) — 성공한 케이스조차 장애물 가장자리에 바짝 붙는
  경향.
- **OpenLander**는 제시율부터 가장 낮다(83%, 4개 seed에서 "safe" 클래스 픽셀을 하나도
  못 찾음). 저자의 합성 학습 데이터 분포와 우리 로우폴리 절차생성 씬 사이의 극심한 도메인
  격차 때문으로 보임(원본 저장소 예제 이미지에서는 같은 코드로 정상적인 3클래스 분할이
  나오는 걸 확인함 — 코드 버그 아님). 성공률도 가장 낮다(46%) — 판단이 틀렸을 때
  장애물 한복판(MOD 0.01~0.02m, 5개 seed)을 고르는 등 확신은 있지만 완전히 틀리는
  패턴이 반복적으로 나타남.
- **우리 방법**만 세 지표 모두에서 우위 — DINOv2 색상 군집으로 잔디/차량 등 "평평하지만
  위험한 표면"을 미리 배제하는 게 실제로 효과가 있음을 정량적으로 보여줌.

> [!warning] 비교 시 주의 (CONTEXT.md 그대로)
> 이 수치를 논문에 보고된 실사 데이터셋 mIoU(VisLanding 76.61 등)와 **직접 비교하지 말 것**.
> 도메인이 다르다(우리는 로우폴리 시뮬, 저쪽은 실사/포토리얼 항공영상). 관련연구 표에는
> 그 논문들 수치를 그대로 인용하되 "평가 조건 상이함"을 명시하고, 이 3열 비교표는 별도로
> "같은 씬, 같은 GT, 같은 조건"이라는 전제를 달아서 쓸 것.

## 재현

```bash
# 1) OpenLander 가중치 받기 (git에 안 커밋된 11MB 바이너리, 최초 1회)
bash models/download_openlander.sh

# 2) onnxruntime이 venv_ros에 없으면
source ~/venv_ros/bin/activate && pip install onnxruntime

# 3) 비교 실행
source ~/venv_ros/bin/activate
source /opt/ros/humble/setup.bash
cd ~/safe_landing/eval
python3 run_comparison.py --seeds 8
# 결과: results/comparison.md, results/comparison.json, seed별 캡처 프레임
```

## 파일 구성

| 파일 | 역할 |
|---|---|
| `camera.py` | landing_detector.py와 동일한 카메라 상수 + 픽셀↔world 변환 (양방향) |
| `gt_mask.py` | `generate_layout()`의 실제 지오메트리 → GT 장애물 마스크 + MOD |
| `capture_frame.py` | gzserver에서 드론을 특정 pose로 순간이동 + RGB/depth 한 장 캡처 |
| `baseline_depth_only.py` | PX4식 depth-only 재구현 |
| `baseline_openlander.py` | OpenLander ONNX 추론 래퍼 |
| `our_method.py` | landing_detector.py 알고리즘의 ROS-독립 버전 (수동 동기화 필요, 파일 상단 주석 참고) |
| `run_comparison.py` | 위 전부를 엮어서 N-seed 비교표 생성 |
| `models/` | ONNX 가중치 (git에 안 커밋, `download_openlander.sh`로 받음) |

## 알려진 한계 / 다음에 손볼 것

- `our_method.py`가 `landing_detector.py`와 로직을 강제로 동기화하는 장치가 없음(수동 복제).
  둘이 갈라지면 이 표가 실제 배포 코드를 안 대표하게 됨 — landing_detector.py를 다음에
  더 크게 리팩터할 일이 생기면 그때 공유 모듈로 합칠 것.
- GT의 차량 footprint가 yaw(회전)를 무시하고 axis-aligned bounding box로 감쌈 — 실제보다
  살짝 넓게 "장애물"로 잡을 수는 있어도 놓치지는 않음 (보수적 근사, gt_mask.py 참고).
- n=24까지는 돌려봤음 (남은작업 #4 겸용). 더 좁은 신뢰구간이 필요하면 `--seeds 50` 등으로
  더 늘리면 됨 — 스크립트 변경 없이 그대로 재사용 가능.
