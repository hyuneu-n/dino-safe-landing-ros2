# 관람객용 실시간 대시보드

ROS 2 토픽을 브라우저 화면으로 중계한다. 영상이나 가짜 수치를 재생하지 않고 실제
`/mission/status`, `/planning/status`, `/drone/odom`, chase camera, landing overlay를 표시한다.
목적지 카드를 누르면 `/clicked_point`를 발행하며 미션 컨트롤러와 착륙 탐지기가 같은
목적지를 받는다.

```bash
# 터미널 1: 목적지 선택 전 대기 모드로 시뮬레이터 실행
WAIT_FOR_DESTINATION=1 SAVE_FRAMES=0 bash run_city_demo.sh 0 norviz

# 터미널 2: 대시보드 실행
bash dashboard/run_dashboard.sh --host 0.0.0.0 --port 8080
```

브라우저에서 `http://localhost:8080`을 연다. 현재 종단 간 비행으로 검증한 목적지는
`mcdonalds_delivery` 한 곳이며 이 카드만 활성화된다. 다른 지상 목적지는 화면에
`EXPERIMENTAL`로 보이지만 선택할 수 없게 했다. 높은 목적지는 지면 높이 처리가 끝날
때까지 목록에서 제외한다.
