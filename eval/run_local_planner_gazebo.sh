#!/usr/bin/env bash
# 기존 residential_delivery.world의 중앙 차단 타워 3개를 이용한 지역 계획기 검증.
# 전체 도시의 사전 waypoint를 쓰지 않고 (-150,0)→(150,0) 직선 목표만 주므로
# 카메라에서 본 장애물을 실제로 우회해야 한다.
#
# 사용:
#   bash eval/run_local_planner_gazebo.sh /tmp/planner_h2 2 45
# 인자: 출력 디렉터리, horizon(1~3), 실행 시간(초)
set -eo pipefail
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-/tmp/safe_landing_local_planner}"
HORIZON="${2:-2}"
DURATION="${3:-45}"
mkdir -p "$OUT" "$OUT/gazebo"

export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9"
export GAZEBO_MODEL_PATH="$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MASTER_URI="${PLANNER_GAZEBO_MASTER_URI:-http://127.0.0.1:11377}"
export ROS_DOMAIN_ID="${PLANNER_ROS_DOMAIN_ID:-177}"
export GAZEBO_LOG_PATH="$OUT/gazebo"
export WAYPOINTS_FILE=""
export LOCAL_PLANNER_MODE=multistep
export LOCAL_PLANNER_HORIZON="$HORIZON"
export LOCAL_PLANNER_LOG="$OUT/planner.jsonl"
: >"$LOCAL_PLANNER_LOG"

PIDS=()
cleanup() {
  if ((${#PIDS[@]})); then
    kill "${PIDS[@]}" 2>/dev/null || true
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  "$ROOT/worlds/residential_delivery.world" >"$OUT/gazebo.log" 2>&1 &
PIDS+=("$!")

for _ in $(seq 1 20); do
  if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
    echo "gzserver가 종료되었습니다. $OUT/gazebo.log 확인" >&2
    exit 1
  fi
  if ros2 service list 2>/dev/null | grep -qx '/gazebo/set_entity_state'; then
    break
  fi
  sleep 1
done
ros2 service list 2>/dev/null | grep -qx '/gazebo/set_entity_state' || {
  echo "Gazebo 서비스가 준비되지 않았습니다." >&2
  exit 1
}

(cd "$ROOT" && python3 mission_controller.py) >"$OUT/mission.log" 2>&1 &
PIDS+=("$!")

for _ in $(seq 1 "$DURATION"); do
  kill -0 "${PIDS[0]}" 2>/dev/null || exit 1
  kill -0 "${PIDS[1]}" 2>/dev/null || exit 1
  sleep 1
done

python3 "$ROOT/eval/summarize_planner_log.py" "$OUT/planner.jsonl" \
  --world "$ROOT/worlds/residential_delivery.world" \
  --json "$OUT/summary.json" --markdown "$OUT/summary.md"
cat "$OUT/summary.md"
