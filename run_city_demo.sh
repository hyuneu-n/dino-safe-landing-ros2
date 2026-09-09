#!/usr/bin/env bash
# run_city_demo.sh — 격자형 대도시(교외+산업+상업+도로망) 데모 한 방 실행 (WSL2 Ubuntu-22.04)
#   worlds/generate_city.py로 매번 새로 생성(시드로 재현 가능) → 가제보 + 착륙지탐지 +
#   미션컨트롤러(웨이포인트 추종, 코너 실제로 돎) + RViz2
# 사용: bash ~/safe_landing/run_city_demo.sh [seed]     (기본 seed=0)
#      RViz 없이: bash ~/safe_landing/run_city_demo.sh 0 norviz
# run_demo.sh(기존 단일 회랑 데모)는 안 건드림 — 이건 별도의 새 진입점.
set -e
unset LIBGL_ALWAYS_SOFTWARE
unset GALLIUM_DRIVER
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9"
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
export GAZEBO_MODEL_PATH="$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
SL="$HOME/safe_landing"
SEED="${1:-0}"
mkdir -p "$SL/logs" "$SL/frames"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$SL/logs/city_$TS.log"
export FRAMES_DIR="$SL/frames/city_$TS"
mkdir -p "$FRAMES_DIR"

WORLD="$SL/worlds/generated/city_seed${SEED}.world"
WP="$SL/worlds/generated/city_seed${SEED}.waypoints.json"
echo "[gen] 격자도시 생성 중 (seed=$SEED)..." | tee "$LOG"
python3 "$SL/worlds/generate_city.py" --seed "$SEED" --mesh --out "$WORLD" | tee -a "$LOG"
export WAYPOINTS_FILE="$WP"

echo "=== 로그: $LOG ===" | tee -a "$LOG"
echo "=== 웨이포인트: $WP ===" | tee -a "$LOG"

echo "[1/6] 기존 Gazebo/RViz 종료..."
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f gazebo   2>/dev/null || true
pkill -f rviz2    2>/dev/null || true
pkill -f landing_detector 2>/dev/null || true
pkill -f mission_controller 2>/dev/null || true
sleep 1

echo "[2/6] Gazebo (격자도시, ROS2 플러그인) 실행..."
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$WORLD" >> "$LOG" 2>&1 &
GZ=$!
sleep 14

echo "[3/6] 착륙지 탐지 노드 (DINOv2) 실행..."
( source "$HOME/venv_ros/bin/activate"; cd "$SL"; SAVE_FRAMES=1 python3 landing_detector.py 2>&1 | tee -a "$LOG" ) &
DET=$!
sleep 4

echo "[4/6] 미션 컨트롤러 (웨이포인트 추종, WAYPOINTS_FILE=$WP) 실행..."
( cd "$SL"; WAYPOINTS_FILE="$WP" python3 mission_controller.py 2>&1 | tee -a "$LOG" ) &
MC=$!
sleep 1

echo "[5/6] Frame Saver 실행..."
( cd "$SL"; FRAMES_DIR="$FRAMES_DIR" python3 frame_saver.py 2>&1 | tee -a "$LOG" ) &
FS=$!
sleep 1

RV=""
if [ "$2" != "norviz" ]; then
  echo "[6/6] RViz2 실행..."
  rviz2 -d "$SL/demo.rviz" &
  RV=$!
fi

echo
echo "=== 실행 중 (격자도시, seed=$SEED) ==="
echo "  경로: worlds/generate_city.py가 그 seed에서 계산한 격자 최단경로(코너 여러 번)"
echo "  Gazebo  : 교외→산업지대→상업/도심(스카이스크래퍼) 블록을 실제로 꺾어가며 통과 → 목적지 스캔 → 착륙"
echo "  로그    : $LOG"
echo "  상태    : ros2 topic echo /mission/status"
echo "  종료    : Ctrl+C"
trap "kill $GZ $DET $MC $FS $RV 2>/dev/null" EXIT
wait
