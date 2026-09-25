#!/usr/bin/env bash
# Run a real autonomous mission and encode the drone-mounted chase camera to MP4.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/artifacts/autonomous_mcdonalds_delivery}"
DESTINATION="${2:-mcdonalds_delivery}"
TIMEOUT="${3:-180}"
FPS="${4:-15}"
mkdir -p "$OUT" "$OUT/gazebo"
OUT="$(cd "$OUT" && pwd)"

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
export GAZEBO_MASTER_URI="${DELIVERY_MASTER_URI:-http://127.0.0.1:11579}"
export ROS_DOMAIN_ID="${DELIVERY_ROS_DOMAIN_ID:-179}"
export GAZEBO_MODEL_DATABASE_URI=http://127.0.0.1:9
export GAZEBO_MODEL_PATH="$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_LOG_PATH="$OUT/gazebo"
export LOCAL_PLANNER_MODE=multistep
export LOCAL_PLANNER_HORIZON=2
export LOCAL_PLANNER_LOG="$OUT/planner.jsonl"
export ENABLE_INTRUDER_DEMO=0
export SAVE_FRAMES=0

python3 "$ROOT/worlds/generate_metropolis.py" --seed 0 --out "$OUT/city.world" >"$OUT/generate.log"
python3 "$ROOT/eval/prepare_delivery_route.py" "$OUT/city.waypoints.json" \
  "$DESTINATION" "$OUT/delivery_route.json" | tee "$OUT/destination.json"
export WAYPOINTS_FILE="$OUT/delivery_route.json"
export LANDING_TARGET_X="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["target_xy"][0])' "$WAYPOINTS_FILE")"
export LANDING_TARGET_Y="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["target_xy"][1])' "$WAYPOINTS_FILE")"
DEST_LABEL="$(python3 -c 'import sys; print(sys.argv[1].replace("_", " ").title())' "$DESTINATION")"
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
  "$OUT/city.world" >"$OUT/gazebo.log" 2>&1 &
PIDS+=("$!")
for _ in $(seq 1 90); do
  kill -0 "${PIDS[0]}" 2>/dev/null || { cat "$OUT/gazebo.log"; exit 1; }
  ros2 service list 2>/dev/null | rg -qx '/gazebo/set_entity_state' && break
  sleep 1
done
ros2 service list 2>/dev/null | rg -qx '/gazebo/set_entity_state' || {
  echo "Gazebo service did not become ready" >&2; exit 1;
}

( source "$HOME/venv_ros/bin/activate"; cd "$ROOT"; exec python3 landing_detector.py ) \
  >"$OUT/landing.log" 2>&1 &
PIDS+=("$!")
for _ in $(seq 1 90); do
  kill -0 "${PIDS[1]}" 2>/dev/null || { cat "$OUT/landing.log"; exit 1; }
  rg -q '준비 완료' "$OUT/landing.log" && break
  sleep 1
done
( cd "$ROOT"; exec python3 mission_controller.py ) >"$OUT/mission.log" 2>&1 &
PIDS+=("$!")

set +e
"$HOME/venv_ros/bin/python" "$ROOT/eval/record_autonomous_delivery.py" \
  --out "$OUT/autonomous_delivery.mp4" --destination "$DEST_LABEL" \
  --timeout "$TIMEOUT" --fps "$FPS" 2>&1 | tee "$OUT/recorder.log"
RECORDER_STATUS=${PIPESTATUS[0]}
set -e

python3 "$ROOT/eval/summarize_planner_log.py" "$LOCAL_PLANNER_LOG" \
  --world "$OUT/city.world" --json "$OUT/planner_summary.json" \
  --markdown "$OUT/planner_summary.md" || true
ffprobe -v error -show_entries format=duration,size -show_entries stream=width,height,r_frame_rate \
  -of json "$OUT/autonomous_delivery.mp4" >"$OUT/video_probe.json"
ffmpeg -y -loglevel error -ss 00:00:20 -i "$OUT/autonomous_delivery.mp4" \
  -frames:v 1 "$OUT/review_20s.jpg" || true
cat "$OUT/video_probe.json"
exit "$RECORDER_STATUS"
