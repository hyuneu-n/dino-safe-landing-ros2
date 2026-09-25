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
export GAZEBO_LOG_PATH="${GAZEBO_LOG_PATH:-$SL/logs/gazebo}"
mkdir -p "$GAZEBO_LOG_PATH"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$SL/logs/city_$TS.log"
WORLD="$SL/worlds/generated/metropolis_seed${SEED}.world"
export WAYPOINTS_FILE="$SL/worlds/generated/metropolis_seed${SEED}.waypoints.json"
export LOCAL_PLANNER_MODE="${LOCAL_PLANNER_MODE:-multistep}"
export LOCAL_PLANNER_HORIZON="${LOCAL_PLANNER_HORIZON:-2}"
export LOCAL_PLANNER_LOG="${LOCAL_PLANNER_LOG:-$SL/logs/planner_$TS.jsonl}"
export FRAMES_DIR="$SL/frames/city_$TS"
mkdir -p "$FRAMES_DIR"
python3 "$SL/worlds/generate_metropolis.py" --seed "$SEED" --out "$WORLD" | tee "$LOG"

# Only processes started by this invocation are stopped on exit.
PIDS=()
cleanup() { if ((${#PIDS[@]})); then kill "${PIDS[@]}" 2>/dev/null || true; fi; }
trap cleanup EXIT
trap 'exit 130' INT TERM
# Check the selected master port, not unrelated validation servers on other ports.
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
if python3 - "$GAZEBO_MASTER_URI" <<'PROBE'
import socket
import sys
from urllib.parse import urlparse
uri = urlparse(sys.argv[1])
try:
    with socket.create_connection((uri.hostname or "127.0.0.1", uri.port or 11345), timeout=.5):
        pass
except OSError:
    sys.exit(1)
PROBE
then
  echo "Gazebo master가 이미 실행 중입니다: $GAZEBO_MASTER_URI"
  echo "기존 창을 종료하거나 다른 GAZEBO_MASTER_URI / ROS_DOMAIN_ID를 지정하세요."
  exit 1
fi

gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$WORLD" >> "$LOG" 2>&1 &
PIDS+=("$!")
echo "도시 미리보기: $WORLD"
echo "로그: $LOG"
if [[ "$MODE" == world ]]; then
  echo "환경만 실행합니다. 도심 · 공장 · 강 · 고가도로 · 산동네, 목적지 22곳."
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
echo "기본 미션: 물류창고 (-210, -88) → 도심 대로 → East Gate (150, 0). 종료: Ctrl+C"
echo "지역 경로 계획: mode=$LOCAL_PLANNER_MODE horizon=$LOCAL_PLANNER_HORIZON 로그=$LOCAL_PLANNER_LOG"
echo "목적지 좌표와 구역 정보: $WAYPOINTS_FILE"
wait
