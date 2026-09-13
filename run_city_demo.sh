#!/usr/bin/env bash
# Connected metropolis; world = environment preview, norviz = mission without RViz.
# bash run_city_demo.sh [seed] [world|norviz]
set -eo pipefail
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
SL="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SEED="${1:-0}"
MODE="${2:-demo}"
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9"
export GAZEBO_MODEL_PATH="$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
mkdir -p "$SL/logs" "$SL/frames" "$SL/worlds/generated"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$SL/logs/city_$TS.log"
WORLD="$SL/worlds/generated/metropolis_seed${SEED}.world"
export WAYPOINTS_FILE="$SL/worlds/generated/metropolis_seed${SEED}.waypoints.json"
export FRAMES_DIR="$SL/frames/city_$TS"
mkdir -p "$FRAMES_DIR"
python3 "$SL/worlds/generate_metropolis.py" --seed "$SEED" --out "$WORLD" | tee "$LOG"

# Only processes started by this invocation are stopped on exit.
PIDS=()
cleanup() { if ((${#PIDS[@]})); then kill "${PIDS[@]}" 2>/dev/null || true; fi; }
trap cleanup EXIT
trap 'exit 130' INT TERM
if pgrep -x gzserver >/dev/null; then
  echo "Gazebo 서버가 이미 실행 중입니다. 기존 서버를 종료하거나 다른 GAZEBO_MASTER_URI / ROS_DOMAIN_ID를 지정하세요."
  if [[ -z "${GAZEBO_MASTER_URI:-}" ]]; then exit 1; fi
fi
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$WORLD" >> "$LOG" 2>&1 &
PIDS+=("$!")
echo "도시 미리보기: $WORLD"
echo "로그: $LOG"
if [[ "$MODE" == world ]]; then
  echo "환경만 실행합니다. 도심 → 물류·공장지대 → 주거 타운, 수변과 목적지 12곳."
  wait "${PIDS[0]}"
  exit
fi
sleep 18
kill -0 "${PIDS[0]}" 2>/dev/null || { echo "Gazebo 실행 실패: $LOG"; exit 1; }
( source "$HOME/venv_ros/bin/activate"; cd "$SL"; SAVE_FRAMES=1 exec python3 landing_detector.py ) >> "$LOG" 2>&1 &
PIDS+=("$!")
sleep 4
( cd "$SL"; exec python3 mission_controller.py ) >> "$LOG" 2>&1 &
PIDS+=("$!")
( cd "$SL"; exec python3 frame_saver.py ) >> "$LOG" 2>&1 &
PIDS+=("$!")
if [[ "$MODE" != norviz ]]; then
  rviz2 -d "$SL/demo.rviz" >> "$LOG" 2>&1 &
  PIDS+=("$!")
fi
echo "기본 미션: 도심 대로 (-150, 0) → East Gate (150, 0). 종료: Ctrl+C"
echo "목적지 좌표와 구역 정보: $WAYPOINTS_FILE"
wait
